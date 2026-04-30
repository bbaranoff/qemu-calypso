#!/usr/bin/env python3
"""
rsl_si_tap.py — Live RSL/Abis sniffer for BCCH SI injection (étape 3).

Sniffe le trafic RSL osmo-bsc → osmo-bts (TCP lo:3003), parse les
RSL_MT_BCCH_INFO messages, extrait les blobs SI byte-exact, et populate
/dev/shm/calypso_si.bin selon doc/MMAP_SI_FORMAT.md v1.

Remplace le stub disposable scripts/populate-si.sh : les SI viennent
maintenant d'un live tap au lieu d'être figés dans un script Python.

Usage :
  rsl_si_tap.py                                # défaut /dev/shm/calypso_si.bin
  CALYPSO_SI_MMAP_PATH=/path rsl_si_tap.py
  rsl_si_tap.py -v                             # verbose

Référence constants (libosmocore include/osmocom/gsm/protocol/gsm_08_58.h) :
  ABIS_RSL_MDISC_COM_CHAN = 0x0c
  RSL_MT_BCCH_INFO        = 0x11
  RSL_IE_CHAN_NR          = 0x01
  RSL_IE_SYSINFO_TYPE     = 0x1e
  RSL_IE_FULL_BCCH_INFO   = 0x27
  RSL_SYSTEM_INFO_1..13   = 0x01, 0x02, 0x03, 0x04, ..., 0x28

SPDX-License-Identifier: GPL-2.0-or-later
"""

import os
import sys
import struct
import socket
import mmap
import signal
import logging

# ---- RSL constants (libosmocore-compatible) ----
ABIS_RSL_MDISC_COM_CHAN = 0x0c
RSL_MT_BCCH_INFO        = 0x11
RSL_IE_CHAN_NR          = 0x01
RSL_IE_SYSINFO_TYPE     = 0x1e
RSL_IE_FULL_BCCH_INFO   = 0x27

# ---- SI type → slot index (per MMAP_SI_FORMAT.md v1) ----
SI_TO_SLOT = {
    0x01: 0,   # SI1
    0x02: 1,   # SI2
    0x03: 2,   # SI3
    0x04: 3,   # SI4
    0x28: 4,   # SI13
}

# ---- mmap layout constants ----
CSI_MAGIC       = b'CSI1'
CSI_VERSION     = 0x01
CSI_SLOT_COUNT  = 5
CSI_HEADER_SIZE = 16
CSI_SLOT_SIZE   = 32
CSI_FILE_SIZE   = CSI_HEADER_SIZE + CSI_SLOT_COUNT * CSI_SLOT_SIZE  # 176
CSI_SLOT_VALID  = 0x01

DEFAULT_PATH = '/dev/shm/calypso_si.bin'

# ---- Logging ----
log = logging.getLogger('rsl_si_tap')


def ensure_mmap_file(path):
    """Create the mmap file with valid header if absent. Validate magic if exists."""
    if not os.path.exists(path):
        log.info("creating %s (176 bytes, empty slots)", path)
        with open(path, 'wb') as f:
            buf = bytearray(CSI_FILE_SIZE)
            buf[0:4] = CSI_MAGIC
            buf[4]   = CSI_VERSION
            buf[5]   = CSI_SLOT_COUNT
            # 6-7 last_update_fn (LE uint16) = 0
            # 8-15 reserved zero
            # slots already zero-init = empty
            f.write(buf)
    with open(path, 'rb') as f:
        hdr = f.read(CSI_HEADER_SIZE)
    if hdr[0:4] != CSI_MAGIC:
        raise RuntimeError(f"Bad magic in {path}: got {hdr[0:4]!r}")
    if hdr[4] != CSI_VERSION:
        log.warning("unexpected version %d (expected %d) in %s",
                    hdr[4], CSI_VERSION, path)


def write_si_to_mmap(path, slot_idx, si_type, blob):
    """Atomic-ish write of a SI blob into the mmap.

    Order : write blob bytes first, then set VALID flag last (anti-race
    with QEMU reader). Single writer assumed (only this process).
    """
    if len(blob) != 23:
        log.warning("unexpected blob len %d for SI 0x%02x slot %d, padding/truncating",
                    len(blob), si_type, slot_idx)
    # Pad or truncate to 23 bytes
    blob_23 = (blob + b'\0' * 23)[:23]

    with open(path, 'r+b') as f:
        mm = mmap.mmap(f.fileno(), CSI_FILE_SIZE)
        try:
            slot_off = CSI_HEADER_SIZE + slot_idx * CSI_SLOT_SIZE
            # 1. Write blob bytes (24 bytes block: padding+blob+pad zero)
            mm[slot_off + 4 : slot_off + 4 + 23] = blob_23
            # 2. Write fields
            mm[slot_off + 2] = 23                       # blob_len
            mm[slot_off + 3] = 0                        # padding
            mm[slot_off + 0] = si_type                  # si_type
            # 3. Set VALID flag LAST (anti-race)
            mm[slot_off + 1] = CSI_SLOT_VALID
            mm.flush()
        finally:
            mm.close()


def parse_rsl_bcch_info(rsl_data):
    """Parse RSL BCCH_INFO message, return (si_type, bcch_info_blob) or (None, None)."""
    if len(rsl_data) < 2:
        return None, None
    msg_discr = rsl_data[0]
    msg_type  = rsl_data[1]
    if msg_discr != ABIS_RSL_MDISC_COM_CHAN or msg_type != RSL_MT_BCCH_INFO:
        return None, None

    si_type   = None
    bcch_info = None
    pos = 2
    while pos < len(rsl_data):
        ie_type = rsl_data[pos]
        pos += 1
        if ie_type == RSL_IE_SYSINFO_TYPE:
            # TV : 1 byte data
            if pos >= len(rsl_data):
                break
            si_type = rsl_data[pos]
            pos += 1
        elif ie_type == RSL_IE_FULL_BCCH_INFO:
            # TLV : 1 byte length + data
            if pos >= len(rsl_data):
                break
            length = rsl_data[pos]
            pos += 1
            bcch_info = bytes(rsl_data[pos:pos + length])
            pos += length
        elif ie_type == RSL_IE_CHAN_NR:
            # TV : 1 byte data
            pos += 1
        else:
            # Unknown IE — break to avoid mis-parsing
            log.debug("unknown IE 0x%02x at pos %d, breaking IE parse", ie_type, pos - 1)
            break
    return si_type, bcch_info


def parse_packet(data):
    """Parse Ethernet/IP/TCP from raw frame, return TCP payload if RSL else None."""
    if len(data) < 14 + 20 + 20:
        return None
    eth_type = struct.unpack('>H', data[12:14])[0]
    if eth_type != 0x0800:  # IPv4 only
        return None
    ip = data[14:]
    ip_hdr_len = (ip[0] & 0x0f) * 4
    proto = ip[9]
    if proto != 6:  # TCP
        return None
    tcp = ip[ip_hdr_len:]
    if len(tcp) < 20:
        return None
    src_port = struct.unpack('>H', tcp[0:2])[0]
    dst_port = struct.unpack('>H', tcp[2:4])[0]
    # We only care about TCP packets to/from port 3003 (RSL)
    if src_port != 3003 and dst_port != 3003:
        return None
    tcp_hdr_len = ((tcp[12] >> 4) & 0x0f) * 4
    payload = tcp[tcp_hdr_len:]
    if not payload:
        return None
    return bytes(payload)


def parse_ipa_stream(payload):
    """Parse an IPA-framed stream, yield (proto, rsl_data) tuples for proto=0x00 RSL."""
    pos = 0
    while pos + 3 <= len(payload):
        ipa_len = struct.unpack('>H', payload[pos:pos+2])[0]
        ipa_proto = payload[pos+2]
        pos += 3
        if pos + ipa_len > len(payload):
            # Fragmented IPA frame, can't handle in single packet — skip
            log.debug("fragmented IPA frame (need %d, have %d), skipping",
                      ipa_len, len(payload) - pos)
            break
        if ipa_proto == 0x00:
            yield bytes(payload[pos:pos+ipa_len])
        pos += ipa_len


def main():
    import argparse
    p = argparse.ArgumentParser(description='Live RSL→mmap SI tap')
    p.add_argument('--path', default=os.environ.get('CALYPSO_SI_MMAP_PATH', DEFAULT_PATH),
                   help=f'mmap file path (default: $CALYPSO_SI_MMAP_PATH or {DEFAULT_PATH})')
    p.add_argument('--iface', default='lo', help='interface to sniff (default: lo)')
    p.add_argument('-v', '--verbose', action='store_true', help='debug logging')
    args = p.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s %(levelname)s %(message)s')

    log.info("rsl_si_tap starting (path=%s, iface=%s)", args.path, args.iface)

    ensure_mmap_file(args.path)
    log.info("mmap file ready: %s", args.path)

    # AF_PACKET raw socket — no libpcap dep
    try:
        sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW,
                             socket.htons(0x0003))  # ETH_P_ALL
        sock.bind((args.iface, 0))
    except PermissionError:
        log.error("AF_PACKET requires root. Run as root or set capability.")
        sys.exit(1)

    log.info("listening on %s for tcp/3003 RSL", args.iface)

    si_count = {0x01: 0, 0x02: 0, 0x03: 0, 0x04: 0, 0x28: 0}

    def shutdown(signum, frame):
        log.info("shutdown signal %d, exiting", signum)
        log.info("SI counts received: %s", {hex(k): v for k, v in si_count.items()})
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    while True:
        try:
            data, addr = sock.recvfrom(65535)
        except KeyboardInterrupt:
            shutdown(signal.SIGINT, None)

        # addr = (ifname, ethtype, pkttype, hatype, hwaddr)
        # pkttype 4 = PACKET_OUTGOING — skip to avoid duplicate on loopback
        if len(addr) >= 3 and addr[2] == 4:
            continue

        payload = parse_packet(data)
        if payload is None:
            continue

        for rsl_data in parse_ipa_stream(payload):
            si_type, bcch_info = parse_rsl_bcch_info(rsl_data)
            if si_type is None:
                continue
            if bcch_info is None:
                log.warning("BCCH_INFO without FULL_BCCH_INFO IE (si=0x%02x)", si_type)
                continue

            slot = SI_TO_SLOT.get(si_type)
            if slot is None:
                log.debug("ignoring SI type 0x%02x (no slot mapping)", si_type)
                continue

            try:
                write_si_to_mmap(args.path, slot, si_type, bcch_info)
                si_count[si_type] = si_count.get(si_type, 0) + 1
                log.info("BCCH_INFO si_type=0x%02x slot=%d len=%d → mmap (count=%d) head=%s",
                         si_type, slot, len(bcch_info), si_count[si_type],
                         bcch_info[:6].hex())
            except Exception as e:
                log.error("mmap write failed: %s", e)


if __name__ == '__main__':
    main()
