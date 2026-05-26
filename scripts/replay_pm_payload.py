#!/usr/bin/env python3
"""
replay_pm_payload.py - Reproduit le payload L1CTL PM scan observe dans
les logs (hdlc_send/recv sur DLCI 5).

Encode des L1CTL_PM_REQ + decode/print des L1CTL_PM_CONF.

Trois modes :
  1. --dump      : imprime les bytes attendus (verification format)
  2. --to-socket /tmp/osmocom_l2  : push vers un socket L1CTL existant
  3. --hdlc-pty  /dev/pts/X  : enveloppe en HDLC DLCI 5 et envoie sur PTY

Format L1CTL observe (32-bit big-endian) :
  hdlc payload = CMD(4B) ARG(4B) [PAYLOAD...]

Commandes vues dans le log :
  0x08 PM_REQ  : ARG=band, payload = 2x uint16 BE (arfcn_lo, arfcn_hi)
  0x09 PM_CONF : ARG=batch_id, payload = N x uint32 BE arfcn list
  0x0d         : sub-cmd ack/next (suit chaque PM_CONF)

Usage :
  ./replay_pm_payload.py --dump
  ./replay_pm_payload.py --to-socket /tmp/osmocom_l2
  ./replay_pm_payload.py --hdlc-pty /dev/pts/1
"""

import argparse
import socket
import struct
import sys


# L1CTL command IDs (deduits du log + osmocom-bb l1ctl_proto.h)
CMD_PM_REQ   = 0x08
CMD_PM_CONF  = 0x09
CMD_DONE     = 0x0d

# GSM band IDs
BAND_GSM900  = 0x00
BAND_DCS1800 = 0x01


def encode_pm_req(band: int, arfcn_lo: int, arfcn_hi: int) -> bytes:
    """
    Encode L1CTL_PM_REQ comme observe dans le log :
      08 00 00 00 BB 00 00 00 LL LL HH HH

    band = BB (0x00 GSM900, 0x01 DCS1800)
    arfcn_lo = LL LL (16-bit BE)
    arfcn_hi = HH HH (16-bit BE)
    """
    return struct.pack(
        ">II HH",
        CMD_PM_REQ,         # 32-bit BE cmd
        band,               # 32-bit BE band
        arfcn_lo,           # 16-bit BE
        arfcn_hi,           # 16-bit BE
    )


def encode_done(sub: int = 1) -> bytes:
    """Encode L1CTL_DONE (0x0d) : 0d 00 00 00 SS 00 00 00"""
    return struct.pack(">II", CMD_DONE, sub)


def decode_pm_conf(b: bytes) -> tuple[int, int, list[int]]:
    """
    Decode L1CTL_PM_CONF :
      09 00 00 00 SS 00 00 00 [arfcn 32-bit BE]*

    Returns (cmd, sub, arfcn_list)
    """
    if len(b) < 8:
        raise ValueError("PM_CONF too short")
    cmd, sub = struct.unpack(">II", b[:8])
    if cmd != CMD_PM_CONF:
        raise ValueError(f"not a PM_CONF (cmd=0x{cmd:02x})")
    n = (len(b) - 8) // 4
    arfcns = list(struct.unpack(f">{n}I", b[8 : 8 + 4 * n]))
    return cmd, sub, arfcns


# ---------- HDLC framing (DLCI 5) ----------
#
# osmocon hdlc protocol : frame_start(0x7E) [escaped payload] frame_end(0x7E)
# Escape : 0x7D 0x5D = literal 0x7D ; 0x7D 0x5E = literal 0x7E
#
# DLCI byte prepended to payload, then HDLC framing.
HDLC_FLAG    = 0x7E
HDLC_ESC     = 0x7D
HDLC_ESC_XOR = 0x20
DLCI_SERCOMM = 5


def hdlc_escape(b: bytes) -> bytes:
    out = bytearray()
    for x in b:
        if x in (HDLC_FLAG, HDLC_ESC):
            out.append(HDLC_ESC)
            out.append(x ^ HDLC_ESC_XOR)
        else:
            out.append(x)
    return bytes(out)


def hdlc_frame(dlci: int, payload: bytes) -> bytes:
    """Wrap payload in HDLC framing for given DLCI."""
    body = bytes([dlci]) + payload
    return bytes([HDLC_FLAG]) + hdlc_escape(body) + bytes([HDLC_FLAG])


# ---------- Scenarios ----------

def build_observed_sequence() -> list[tuple[str, bytes]]:
    """Reproduit la sequence exacte du log."""
    seq = []
    # DCS 1800 scan : ARFCN 940..954
    seq.append(("PM_REQ DCS 940-954", encode_pm_req(BAND_DCS1800, 940, 954)))
    # CONF expected : list of arfcn 940..954
    arfcn_list = list(range(940, 955))
    conf = struct.pack(">II", CMD_PM_CONF, 1) + b"".join(
        struct.pack(">I", a) for a in arfcn_list
    )
    seq.append(("PM_CONF DCS 940-954 (expected)", conf))
    seq.append(("DONE sub=1", encode_done(1)))

    # GSM 900 scan : ARFCN 1..124
    seq.append(("PM_REQ GSM900 1-124", encode_pm_req(BAND_GSM900, 1, 124)))
    arfcn_list2 = list(range(1, 125))
    conf2 = struct.pack(">II", CMD_PM_CONF, 0) + b"".join(
        struct.pack(">I", a) for a in arfcn_list2
    )
    seq.append(("PM_CONF GSM900 1-124 (expected)", conf2))
    seq.append(("DONE sub=1", encode_done(1)))
    return seq


# ---------- Main ----------

def hex_pretty(b: bytes, line_len: int = 32) -> str:
    out = []
    for i in range(0, len(b), line_len):
        chunk = b[i : i + line_len]
        hexpart = " ".join(f"{x:02x}" for x in chunk)
        asciipart = "".join(chr(x) if 32 <= x < 127 else "." for x in chunk)
        out.append(f"  {hexpart:<{line_len*3}}  {asciipart}")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="Replay L1CTL PM scan payload")
    ap.add_argument("--dump", action="store_true",
                    help="print expected bytes (no I/O)")
    ap.add_argument("--to-socket", metavar="PATH",
                    help="send PM_REQs as L1CTL frames to unix socket")
    ap.add_argument("--hdlc-pty", metavar="PATH",
                    help="wrap in HDLC DLCI 5 and write to PTY")
    ap.add_argument("--band", choices=["gsm900", "dcs1800"], default="gsm900")
    ap.add_argument("--arfcn-lo", type=int, default=1)
    ap.add_argument("--arfcn-hi", type=int, default=124)
    args = ap.parse_args()

    band = BAND_DCS1800 if args.band == "dcs1800" else BAND_GSM900
    custom_req = encode_pm_req(band, args.arfcn_lo, args.arfcn_hi)

    if args.dump:
        print("=== Observed sequence reproduction ===\n")
        for label, payload in build_observed_sequence():
            print(f"-- {label} ({len(payload)} bytes) --")
            print(hex_pretty(payload))
            print()
        print("=== Custom PM_REQ ===")
        print(f"  band={args.band} arfcn={args.arfcn_lo}-{args.arfcn_hi}")
        print(hex_pretty(custom_req))
        print()
        print("=== HDLC framed (DLCI 5) ===")
        framed = hdlc_frame(DLCI_SERCOMM, custom_req)
        print(hex_pretty(framed))
        return

    if args.to_socket:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.connect(args.to_socket)
        # L1CTL frame = 16-bit BE length prefix + payload (per osmocom-bb l1ctl)
        framed = struct.pack(">H", len(custom_req)) + custom_req
        s.sendall(framed)
        print(f"sent {len(framed)} bytes to {args.to_socket}")
        # Optionally wait for CONF
        s.settimeout(2.0)
        try:
            r = s.recv(4096)
            print(f"recv {len(r)} bytes :")
            print(hex_pretty(r))
        except socket.timeout:
            print("(no response in 2s)")
        s.close()
        return

    if args.hdlc_pty:
        framed = hdlc_frame(DLCI_SERCOMM, custom_req)
        with open(args.hdlc_pty, "wb") as f:
            f.write(framed)
        print(f"sent HDLC frame ({len(framed)} bytes) to {args.hdlc_pty}")
        return

    ap.print_help()
    sys.exit(1)


if __name__ == "__main__":
    main()
