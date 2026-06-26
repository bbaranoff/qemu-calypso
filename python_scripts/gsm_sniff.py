#!/usr/bin/env python3
# gsm_sniff.py — sniff PASSIF (raw socket) des sockets UDP GSMTAP/SCH/TRXD et
# TRANSFORME en SI decode / burst decode. NE bind AUCUN port (AF_PACKET) -> ne
# vole rien aux vrais destinataires (grgsm/qemu), ne touche AUCUNE fifo.
#
# Usage : gsm_sniff.py si      # GSMTAP -> num+type SI, canal BCCH/CCCH, FN (4729,4730)
#         gsm_sniff.py burst   # SCH (4731) + TRXD v1 (5700-5702/5800-5802) -> burst
import socket, struct, sys, os

MODE = sys.argv[1] if len(sys.argv) > 1 else "si"
IFACE = os.environ.get("SNIFF_IFACE", "lo")
ETH_P_IP = 0x0800
GSMTAP_HDR_LEN = 16                         # GSMTAP v2 header (octets)

# --- GSM 04.08 / 44.018 RR message types (PD = 0x06), 10.4 -------------------
# key = RR message-type octet (L2[2]) ; value = (nom, is_si, canal DL attendu)
RR_MT = {
    0x19: ("SI1",       True,  "BCCH"),
    0x1a: ("SI2",       True,  "BCCH"),
    0x1b: ("SI3",       True,  "BCCH"),
    0x1c: ("SI4",       True,  "BCCH"),
    0x1d: ("SI2bis",    True,  "BCCH"),
    0x1e: ("SI2ter",    True,  "BCCH"),
    0x02: ("SI2quater", True,  "BCCH"),
    0x03: ("SI9",       True,  "BCCH"),
    0x00: ("SI13",      True,  "CCCH"),   # SI13 = mt 0x00 (44.018 tbl 10.4.1)
    # --- CCCH downlink non-SI ---
    0x3f: ("IMM-ASSIGN",     False, "CCCH"),   # Immediate Assignment (AGCH) <- le LU !
    0x39: ("IMM-ASSIGN-EXT", False, "CCCH"),
    0x3a: ("IMM-ASSIGN-REJ", False, "CCCH"),
    0x21: ("PAGING-REQ-1",   False, "CCCH"),   # 0x21 = PAGING type 1 (PAS SI13)
    0x22: ("PAGING-REQ-2",   False, "CCCH"),
    0x24: ("PAGING-REQ-3",   False, "CCCH"),
    0x06: ("CHAN-RELEASE",   False, "*"),
    0x2e: ("ASSIGN-CMD",     False, "*"),
    0x10: ("CIPHER-MODE-CMD",False, "*"),
    0x12: ("RR-STATUS",      False, "*"),
}

GSMTAP_CHAN = {0x00:"UNKNOWN",0x01:"BCCH",0x02:"CCCH",0x03:"RACH",0x04:"AGCH",
               0x05:"PCH",0x06:"SDCCH",0x07:"SDCCH4",0x08:"SDCCH8",0x09:"TCH/F",
               0x0a:"TCH/H",0x0b:"PACCH",0x0c:"CBCH52",0x0d:"PDTCH",0x0e:"PTCCH",0x0f:"CBCH51"}
def gsmtap_chan_name(st):
    base = GSMTAP_CHAN.get(st & 0x7f, "0x%02x" % (st & 0x7f))
    return base + ("+ACCH" if (st & 0x80) else "")
def gsmtap_chan_class(st):
    b = st & 0x7f
    if b == 0x01: return "BCCH"
    if b in (0x02,0x04,0x05): return "CCCH"
    if b == 0x03: return "RACH"
    if b in (0x06,0x07,0x08): return "SDCCH"
    return "OTHER"

def fn51_role(fn):
    m = fn % 51
    if m in (0,10,20,30,40): return "FCCH"
    if m in (1,11,21,31,41): return "SCH"
    if m in (2,3,4,5):       return "BCCH"
    if m == 50:              return "idle"
    return "CCCH"
def fn51_class(fn):
    r = fn51_role(fn)
    return r if r in ("BCCH","CCCH") else "—"

if MODE == "si":
    PORTS = {int(p) for p in os.environ.get("SNIFF_PORTS","4729,4730").split(",")}
else:
    PORTS = {int(p) for p in os.environ.get("SNIFF_PORTS","4731,5700,5701,5702,5800,5801,5802").split(",")}

s = socket.socket(socket.AF_PACKET, socket.SOCK_DGRAM, socket.htons(ETH_P_IP))
s.bind((IFACE, 0))
print("[gsm-sniff:%s] iface=%s ports=%s (passif, aucune fifo)"
      % (MODE, IFACE, sorted(PORTS)), flush=True)

def udp_payload(ip):
    if len(ip) < 20 or (ip[0] >> 4) != 4: return None, None
    ihl = (ip[0] & 0x0f) * 4
    if ip[9] != 17: return None, None
    u = ip[ihl:]
    if len(u) < 8: return None, None
    return struct.unpack(">H", u[2:4])[0], u[8:]

def decode_si(pl):
    if len(pl) < GSMTAP_HDR_LEN: return None
    ts  = pl[3]
    fn  = struct.unpack(">I", pl[8:12])[0]     # frame_number reel (BE @ offset 8)
    sub = pl[12]                               # GSMTAP sub_type / canal
    l2  = pl[16:]
    if len(l2) >= 3 and l2[1] == 0x06:
        mt = l2[2]
        name, is_si, exp = RR_MT.get(mt, ("RR/mt=0x%02x" % mt, False, "*"))
        name = "%s(0x%02x)" % (name, mt)
    else:
        name, is_si, exp = "?", False, "*"
    tag_cls = gsmtap_chan_class(sub)
    fn_role = fn51_role(fn); fn_cls = fn51_class(fn)
    flags = []
    if tag_cls != fn_cls and fn_cls != "—": flags.append("CHAN!=FN(%s/%s)" % (tag_cls, fn_cls))
    if fn_role in ("FCCH","SCH","idle"):      flags.append("BAD-FN(sur %s)" % fn_role)
    if is_si and exp == "BCCH" and fn_cls == "CCCH": flags.append("SI-sur-CCCH")
    fl = ("  [" + " ".join(flags) + "]") if flags else ""
    return "FN=%d ts%d (%%51=%d %s)  CHAN=%-12s  %-18s  L2: %s%s" % (
        fn, ts, fn % 51, fn_role, gsmtap_chan_name(sub), name, l2[:23].hex(" "), fl)

def decode_burst(dport, pl):
    if dport == 4731 and len(pl) >= 16 and pl[:4] == b"SCH1":
        bsic, fn, toa = struct.unpack("<iii", pl[4:16])
        return "SCH   bsic=%d (ncc=%d bcc=%d)  fn=%d (%%51=%d %s)  toa=%d" % (
            bsic,(bsic>>3)&7,bsic&7,fn,fn%51,fn51_role(fn),toa)
    if dport in (5700,5701,5702,5800,5801,5802) and len(pl) >= 6:
        ver = pl[0] >> 4; tn = pl[0] & 0x07
        fn  = int.from_bytes(pl[1:5], "big")
        if len(pl) >= 6 + 148:
            b = "".join("1" if x else "0" for x in pl[-148:])
            return "TRXDv%d ts%d fn=%d (%%51=%d %s) 148b %s" % (ver,tn,fn,fn%51,fn51_role(fn),b)
        return "TRXDv%d ts%d fn=%d (%%51=%d %s) NOPE(len=%d, burst vide)" % (
            ver,tn,fn,fn%51,fn51_role(fn),len(pl))
    return None

n = 0
while True:
    pkt = s.recv(65535)
    dport, pl = udp_payload(pkt)
    if dport not in PORTS or pl is None:
        continue
    line = decode_si(pl) if MODE == "si" else decode_burst(dport, pl)
    if line:
        n += 1
        print("%6d  :%-4d  %s" % (n, dport, line), flush=True)
