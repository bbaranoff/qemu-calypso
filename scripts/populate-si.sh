#!/bin/bash
# populate-si.sh — XXX TRANSITIONAL STUB (étape 2b, disposable).
#
# Écrit les 5 SI hex blobs RSL-extracted (osmo-bsc → osmo-bts trace 2026-04-30)
# dans /dev/shm/calypso_si.bin per doc/MMAP_SI_FORMAT.md v1.
#
# Permet de valider l'interface mmap (étape 2a) sans dépendance sur le RSL
# parser (étape 3). À supprimer dès que scripts/rsl_si_tap.py est opérationnel
# en steady-state.
#
# Usage:
#   ./populate-si.sh                              # défaut /dev/shm/calypso_si.bin
#   CALYPSO_SI_MMAP_PATH=/tmp/foo ./populate-si.sh
#
# Vérification:
#   xxd /dev/shm/calypso_si.bin | head           # check magic "CSI1" + slots

set -e

OUTPUT="${CALYPSO_SI_MMAP_PATH:-/dev/shm/calypso_si.bin}"

python3 - "$OUTPUT" << 'PYEOF'
import sys, struct, os

OUTPUT = sys.argv[1]

# 23-byte BCCH SI blobs, byte-exact from RSL re-attach trace
# (osmo-bsc emits BCCH INFORMATION when osmo-bts attaches via Abis OML).
# LAI 001/01/0001, CI=6001, BSIC=7, ARFCN=514.
SI1  = bytes([0x55, 0x06, 0x19, 0x8f, 0x01,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0xe5, 0x04, 0x00, 0x2b])
SI2  = bytes([0x59, 0x06, 0x1a,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0xff, 0xe5, 0x04, 0x00])
SI3  = bytes([0x49, 0x06, 0x1b, 0x17, 0x71, 0x00, 0xf1, 0x10, 0x00, 0x01, 0xc9, 0x03,
              0x05, 0x27, 0x47, 0x40, 0xe5, 0x04, 0x00, 0x2c, 0x0b, 0x2b, 0x2b])
SI4  = bytes([0x31, 0x06, 0x1c, 0x00, 0xf1, 0x10, 0x00, 0x01, 0x47, 0x40, 0xe5, 0x04,
              0x00, 0x01, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b])
SI13 = bytes([0x01, 0x06, 0x00, 0x90, 0x00, 0x18, 0x5a, 0x6f, 0xc9, 0xf2, 0xb5, 0x30,
              0x42, 0x08, 0xeb, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b])

assert len(SI1)  == 23, f"SI1  len={len(SI1)}"
assert len(SI2)  == 23, f"SI2  len={len(SI2)}"
assert len(SI3)  == 23, f"SI3  len={len(SI3)}"
assert len(SI4)  == 23, f"SI4  len={len(SI4)}"
assert len(SI13) == 23, f"SI13 len={len(SI13)}"

# Layout per MMAP_SI_FORMAT.md v1 :
#   header  (16 bytes) : magic + version + slot_count + last_update_fn + reserved
#   slots   (5 × 32B)  : si_type + flags + blob_len + pad + 23B blob + 5B pad
SLOT_VALID = 0x01

def make_slot(si_type, blob):
    s = bytearray(32)
    s[0] = si_type
    s[1] = SLOT_VALID
    s[2] = len(blob)
    s[3] = 0  # padding
    s[4:4+23] = blob
    # 27..31 padding zero (already)
    return bytes(s)

buf = bytearray(176)
# Header
buf[0:4]  = b"CSI1"
buf[4]    = 0x01      # version
buf[5]    = 0x05      # slot_count
buf[6:8]  = struct.pack("<H", 0)  # last_update_fn = 0 (stub static, no live FN)
# bytes 8..15 reserved zero (already)

# Slots in fixed order : SI1, SI2, SI3, SI4, SI13
buf[16:48]    = make_slot(0x01, SI1)
buf[48:80]    = make_slot(0x02, SI2)
buf[80:112]   = make_slot(0x03, SI3)
buf[112:144]  = make_slot(0x04, SI4)
buf[144:176]  = make_slot(0x0d, SI13)

with open(OUTPUT, "wb") as f:
    f.write(buf)

# Verify
with open(OUTPUT, "rb") as f:
    chk = f.read()
assert chk == bytes(buf), "readback mismatch"
print(f"OK : wrote {len(buf)} bytes to {OUTPUT}")
print(f"     magic={chk[0:4]!r} version={chk[4]} slot_count={chk[5]}")
for slot_idx, name in enumerate(["SI1", "SI2", "SI3", "SI4", "SI13"]):
    off = 16 + slot_idx * 32
    si_type = chk[off]
    flags   = chk[off+1]
    blob_len= chk[off+2]
    blob    = chk[off+4:off+4+23]
    print(f"     slot[{slot_idx}] {name}: type=0x{si_type:02x} flags=0x{flags:02x} "
          f"len={blob_len} blob[0:4]={blob[:4].hex()} blob[-4:]={blob[-4:].hex()}")
PYEOF
