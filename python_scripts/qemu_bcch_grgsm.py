#!/usr/bin/env python3
"""
qemu_bcch_grgsm.py — pont gr-gsm : I/Q DL du BTS (tee qemu) → bursts → demapper
+ decoder gr-gsm → GSMTAP → le shunt qemu (4730) → a_cd → le mobile campe.

ZÉRO hack : le VRAI signal du BTS, démodulé et décodé par la chaîne gr-gsm
officielle (gsm_bcch_ccch_demapper + control_channels_decoder, le même code
que grgsm_decode).

Chaîne :
  UDP I/Q (tee BSP qemu, port 6703)  ──parse TRXD (8 hdr + 148 cs16)──>
    ──GMSK demod différentiel (1 SPS)──>  148 bits durs
    ──gsmtap_hdr(UM_BURST) + 148 octets──>  post au demapper gr-gsm
    ──gsm_bcch_ccch_demapper (assemble 4 bursts du 51-MF)──>
    ──control_channels_decoder (deinterleave+Viterbi+FIRE)──>  GSMTAP L2
    ──network.socket_pdu(UDP_CLIENT 127.0.0.1:4730)──>  shunt feed_si → a_cd

Le démod produit les 148 bits du burst complet (3 tail | 57 | 1 | 26 train |
1 | 57 | 3 tail) ; le decoder gr-gsm extrait/corrige tout seul. Si décode jamais
(le decoder est silencieux sur CRC fail), inverser BIT_SIGN.

Usage : source /root/.env/bin/activate ; python qemu_bcch_grgsm.py
Env   : IQ_TEE_PORT(6703) GSMTAP_HOST/PORT(127.0.0.1/4730) ARFCN(514)
        TS(0) BIT_SIGN(1)
"""
import os, socket, struct
import numpy as np
from gnuradio import gr, gsm, network

IQ_TEE_PORT = int(os.environ.get("IQ_TEE_PORT", "6703"))
GSMTAP_HOST = os.environ.get("GSMTAP_HOST", "127.0.0.1")
GSMTAP_PORT = int(os.environ.get("GSMTAP_PORT", "4730"))
ARFCN       = int(os.environ.get("ARFCN", "514"))
TS          = int(os.environ.get("TS", "0"))
BIT_SIGN    = int(os.environ.get("BIT_SIGN", "1"))

# --- gsmtap.h (gr-gsm) ---
GSMTAP_VERSION       = 0x02
GSMTAP_TYPE_UM_BURST = 0x03
GSMTAP_BURST_NORMAL  = 0x06
BURST_SIZE           = 148

def demod_burst(iq):
    """iq : np.complex (>=148) -> string de 148 bits durs '0'/'1', ou None."""
    if len(iq) < BURST_SIZE:
        return None
    s = iq[:BURST_SIZE].astype(np.complex64)
    d = s[1:] * np.conj(s[:-1])
    d = np.concatenate(([d[0]], d))        # aligner 148
    q = np.imag(d) * BIT_SIGN              # ±π/2 -> ±1
    return "".join("1" if v > 0 else "0" for v in q)

def decode_block(fns, datas):
    """Run one-shot : burst_source(4 bursts BCCH) -> demapper -> decoder ->
    socket_pdu(GSMTAP 4730). gr-gsm officiel, pas de _post. fns/datas = listes
    parallèles (frame numbers + strings de 148 bits)."""
    tb  = gr.top_block("bcch block")
    src = gsm.burst_source(fns, [TS] * len(fns), datas)
    src.set_arfcn(ARFCN)
    dm  = gsm.gsm_bcch_ccch_demapper(TS)
    dec = gsm.control_channels_decoder()
    snk = network.socket_pdu("UDP_CLIENT", GSMTAP_HOST, str(GSMTAP_PORT), 10000)
    tb.msg_connect((src, "out"), (dm, "bursts"))
    tb.msg_connect((dm, "bursts"), (dec, "bursts"))
    tb.msg_connect((dec, "msgs"), (snk, "pdus"))
    tb.start()
    tb.wait()                              # burst_source émet puis poste 'done'

def udp_loop():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", IQ_TEE_PORT))
    print(f"[grgsm] I/Q in udp:{IQ_TEE_PORT} -> GSMTAP {GSMTAP_HOST}:{GSMTAP_PORT} "
          f"arfcn={ARFCN} ts={TS}", flush=True)
    acc = {}                               # fn//51 -> {mf : bits_string}
    npkt = nfed = nblk = 0
    while True:
        pkt, _ = rx.recvfrom(4096)
        npkt += 1
        if len(pkt) < 8 + BURST_SIZE * 4:
            if npkt <= 3:
                print(f"[grgsm] paquet court len={len(pkt)} (attendu>={8+BURST_SIZE*4})", flush=True)
            continue
        fn  = struct.unpack(">I", pkt[1:5])[0]
        mf  = fn % 51
        if mf not in (2, 3, 4, 5):         # BCCH Norm = fn%51 ∈ {2,3,4,5}
            continue
        raw = np.frombuffer(pkt[8:8 + BURST_SIZE * 4], dtype="<i2").astype(np.float32)
        iq  = raw[0::2] + 1j * raw[1::2]
        bits = demod_burst(iq)
        if bits is None:
            continue
        nfed += 1
        if nfed <= 5 or nfed % 200 == 0:
            print(f"[grgsm] burst #{nfed} fn={fn} mf={mf} pwr={np.mean(np.abs(iq)):.0f}", flush=True)
        blk = fn // 51
        acc.setdefault(blk, {})[mf] = (fn, bits)
        if len(acc[blk]) == 4:             # bloc BCCH complet (fn 2,3,4,5)
            b = acc.pop(blk)
            fns   = [b[m][0] for m in (2, 3, 4, 5)]
            datas = [b[m][1] for m in (2, 3, 4, 5)]
            nblk += 1
            try:
                decode_block(fns, datas)
                print(f"[grgsm] bloc BCCH #{nblk} décodé (fn={fns[0]}..{fns[3]}) "
                      f"-> GSMTAP envoyé si CRC ok", flush=True)
            except Exception as e:
                print(f"[grgsm] decode_block err: {e}", flush=True)

def main():
    udp_loop()

if __name__ == "__main__":
    main()
