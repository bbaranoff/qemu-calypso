#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# grgsm_shm_decode.py — le "DSP" gr-gsm AU MILIEU du shunt, via BUFFER shm
# (pas fifo/udp). Lit l'I/Q d'ENTREE du DSP shunte dans /dev/shm/calypso_dsp_shunt
# (ring de bursts ecrit par calypso_dsp_shunt_feed_iq), demodule+decode (chaine
# gr-gsm officielle), et ecrit le SI L2 decode dans la zone SORTIE du MEME shm
# (si[]+si_len+si_seq++) -> shunt_poll_si_shm le pousse dans a_cd -> l'ARM lit.
#   source /root/.env/bin/activate ; python3 /tmp/grgsm_shm_decode.py
import os, mmap, struct, time
import numpy as np
from gnuradio import gr, gsm
import pmt

SHM_PATH   = os.environ.get("SHM_PATH", "/dev/shm/calypso_dsp_shunt")
# ---- layout : DOIT matcher struct dsp_shunt_shm (calypso_dsp_shunt.c) ----
SLOTS      = 64
SLOT_LEN   = 320                       # int16 par slot
SLOT_SZ    = 8 + SLOT_LEN * 2          # fn(4)+n(4)+iq(640) = 648
OFF_IQWR   = 4
OFF_SLOTS  = 8
OFF_SISEQ  = OFF_SLOTS + SLOTS * SLOT_SZ
OFF_SILEN  = OFF_SISEQ + 4
OFF_SI     = OFF_SISEQ + 8
SHM_SIZE   = OFF_SI + 32

ARFCN      = int(os.environ.get("ARFCN", "514"))
TS         = int(os.environ.get("TS", "0"))
BIT_SIGN   = int(os.environ.get("BIT_SIGN", "1"))
THETA      = float(os.environ.get("ROT_RAD", "-1.914"))   # derotation/sample (offset freq corrige, cf demod_sweep2)
WINDOW_MF  = int(os.environ.get("WINDOW_MF", "12"))
BURST_SIZE = 148


def demod_burst(iq):
    """iq complexe (>=148) -> 148 bits durs '0'/'1' (GMSK derotation -pi/2,
    slicer imag, polarite, shift -1 -> TSC@61). Copie de qemu_bcch_grgsm.py."""
    if len(iq) < BURST_SIZE:
        return None
    s = iq[:BURST_SIZE].astype(np.complex64)
    k = np.arange(BURST_SIZE)
    y = s * np.exp(-1j * THETA * k)          # THETA=-1.914 : offset freq corrige (sweep -> TSC 24/26)
    bits = (y.imag > 0).astype(int)
    if BIT_SIGN > 0:
        bits = 1 - bits
    return "".join(str(int(b)) for b in bits)


# ---- shm ----
fd = os.open(SHM_PATH, os.O_RDWR)
mm = mmap.mmap(fd, SHM_SIZE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)

def rd_u32(off):    return struct.unpack_from("<I", mm, off)[0]
def wr_u32(off, v): struct.pack_into("<I", mm, off, v & 0xffffffff)


class ShmSISink(gr.basic_block):
    """Recoit les 'msgs' GSMTAP du control_channels_decoder ; sur un SI3 BCCH,
    ecrit le L2 dans la zone SORTIE du shm (si[]) et bumpe si_seq."""
    def __init__(self):
        gr.basic_block.__init__(self, name="shm_si_sink", in_sig=[], out_sig=[])
        self.n = 0
        self.message_port_register_in(pmt.intern("in"))
        self.set_msg_handler(pmt.intern("in"), self._h)

    def _h(self, msg):
        try:
            data = bytes(pmt.u8vector_elements(pmt.cdr(msg)))
        except Exception:
            return
        if len(data) < 16 + 3:
            return
        if data[2] != 0x01 or data[12] != 0x01:        # GSMTAP type UM + canal BCCH
            return
        l2 = data[16:16 + 23]
        if len(l2) < 3 or l2[1] != 0x06 or l2[2] != 0x1B:   # RR PD + SI3 mt
            return
        for i in range(len(l2)):
            mm[OFF_SI + i] = l2[i]
        wr_u32(OFF_SILEN, len(l2))
        wr_u32(OFF_SISEQ, rd_u32(OFF_SISEQ) + 1)            # publie : shunt poll
        self.n += 1
        print("[shm-decode] SI3 -> shm si[] (%d o, #%d) l2=%s"
              % (len(l2), self.n, l2[:3].hex()), flush=True)


def decode_block(fns, datas):
    tb  = gr.top_block("bcch block")
    src = gsm.burst_source(fns, [TS] * len(fns), datas)
    src.set_arfcn(ARFCN)
    dm  = gsm.gsm_bcch_ccch_demapper(TS)
    dec = gsm.control_channels_decoder()
    snk = ShmSISink()
    tb.msg_connect((src, "out"), (dm, "bursts"))
    tb.msg_connect((dm, "bursts"), (dec, "bursts"))
    tb.msg_connect((dec, "msgs"), (snk, "in"))
    tb.start()
    tb.wait()


def main():
    if rd_u32(0) != 0x43445350:
        print("[shm-decode] WARN magic=0x%08x != 'CDSP' (shm pas initialise par qemu ?)"
              % rd_u32(0), flush=True)
    print("[shm-decode] shm=%s (%d o) ENTREE ring[%d] -> gr-gsm -> SORTIE si[] (arfcn=%d ts=%d)"
          % (SHM_PATH, SHM_SIZE, SLOTS, ARFCN, TS), flush=True)
    last_wr = rd_u32(OFF_IQWR)
    cur_mf = None
    fns, datas = [], []
    mf_count = nfed = ndec = 0
    while True:
        wr = rd_u32(OFF_IQWR)
        if wr == last_wr:
            time.sleep(0.0005)             # poll le buffer (pas de fifo bloquante)
            continue
        if wr - last_wr > SLOTS:           # on a pris du retard -> on saute (perte loggee)
            print("[shm-decode] LAG %d bursts sautes" % (wr - last_wr - SLOTS), flush=True)
            last_wr = wr - SLOTS
        while last_wr != wr:
            base = OFF_SLOTS + (last_wr % SLOTS) * SLOT_SZ
            fn = rd_u32(base)
            n  = rd_u32(base + 4)
            last_wr += 1
            if n < BURST_SIZE * 2:
                continue
            raw = np.frombuffer(mm, dtype="<i2", count=n, offset=base + 8).astype(np.float32)
            bits = demod_burst(raw[0::2] + 1j * raw[1::2])
            if bits is None:
                continue
            nfed += 1
            if nfed <= 5 or nfed % 1000 == 0:
                print("[shm-decode] burst #%d fn=%d mf=%d pwr=%.0f"
                      % (nfed, fn, fn % 51, np.mean(np.abs(raw))), flush=True)
            mfi = fn // 51
            if cur_mf is None:
                cur_mf = mfi
            if mfi != cur_mf:
                mf_count += 1
                cur_mf = mfi
                if mf_count >= WINDOW_MF and fns:
                    ndec += 1
                    try:
                        decode_block(fns, datas)
                    except Exception as e:
                        print("[shm-decode] decode_block err: %s" % e, flush=True)
                    fns, datas = [], []
                    mf_count = 0
            fns.append(fn)
            datas.append(bits)


if __name__ == "__main__":
    main()
