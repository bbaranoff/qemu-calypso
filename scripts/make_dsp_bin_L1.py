#!/usr/bin/env python3
"""
make_dsp_bin_L1.py — Generate calypso_dsp.PROM0.bin with synthetic L1+L2 publisher.

Scope (extension 2026-05-28 : L1 → L2) :
- FB lock : d_fb_det + a_sync_demod[4] cells (TOA/PM/ANGLE/SNR)
- PM measurement : a_pm[3] WP0/WP1 with canned -85 dBm value
- SB lock : a_sch[5] WP0/WP1 with canned BSIC matching osmo-bsc.cfg
- BCCH SI3 (L2) : a_cd[0..14] with FIRE-OK status + 23-byte SI3 LAPDm frame
  (same canned bytes as CALYPSO_DSP_SHUNT shunt_dispatch_allc, MCC=001/MNC=01)

Entry point : 0x7120 (= SILICON-BOOT-REDIRECT target from c54x.c).
Patches existing PROM0.bin at offset 0x240 (= word 0x120, code addr 0x7120).
Rest of ROM preserved.

Architecture :
- Boot init at 0x7120 : sets SP=0x5AC8, IMR=0xFFFF
- Main loop : tight loop writing all cells continuously
- ARM reads cells anytime, always sees canned "lock detected" values

Strategy : continuous-write rather than dispatch-on-task. Simpler than full
dispatcher, plus ARM cell-read race is auto-resolved (whatever read happens
sees current canned value).

Usage :
    python3 make_dsp_bin_L1.py [input.bin] [output.bin]
    Default input  : /opt/GSM/calypso_dsp.PROM0.bin
    Default output : /opt/GSM/calypso_dsp.PROM0.bin.L1
"""
import sys
import struct

INPUT  = sys.argv[1] if len(sys.argv) > 1 else '/opt/GSM/calypso_dsp.PROM0.bin'
# *** OUTPUT path EXPLICITLY separate — JAMAIS écrase l'original ***
OUTPUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/calypso_dsp_L1stub.PROM0.bin'

# ---- C54x opcode encoders ----

def stm_lk_mmr(lk, mmr):
    """STM #lk, MMR — 2 words (0x7700 | MMR_low7, lk)"""
    return [0x7700 | (mmr & 0x7F), lk & 0xFFFF]

def ld_lk_a(lk):
    """LD #lk, A — 2 words ALU class F0xx (alu_op=2 LD)"""
    # bits 7:4 = 0010 = LD ; bit 8 = SRC ; bit 9 = DST
    # Use src=A dst=A : bit 8 = 0 bit 9 = 0
    # opcode 0xF0 << 8 | (alu_op << 4) | shift = 0xF020 (shift=0)
    return [0xF020, lk & 0xFFFF]

def stl_a_arn(n, mod):
    """STL A, *ARn[mod] — 1 word (0x80xx hi8 = STL A)
       mod ∈ {0=*ARn, 1=*ARn-, 2=*ARn+}"""
    smem = 0x80 | ((mod & 0xF) << 3) | (n & 0x7)
    return [(0x80 << 8) | smem]

def b_unc(target):
    """B pmad (unconditional NEAR branch) — 2 words.
       Use 0xF880 (= FB FAR in c54x.c emulator but sets XPC=0 always,
       which is no-op when staying within PROM0). Same encoding as the
       SILICON-BOOT-REDIRECT vector synthesis in calypso_c54x.c.
       NOT 0xF800 : that's BANZ AR0 (= conditional on AR0!=0) in this emu."""
    return [0xF880, target & 0xFFFF]

# ---- MMR table ----
MMR_IMR = 0x00
MMR_AR0 = 0x10
MMR_SP  = 0x18

# ---- Cell addresses (DSP code space) ----
# API base = 0x0800
# NDB base = 0x08D4 (= byte 0x01A8 / 2)
D_FB_DET     = 0x08F8    # NDB + 0x48 byte
D_FB_MODE    = 0x08F9    # NDB + 0x4A byte
A_SYNC_DEMOD = 0x08FA    # NDB + 0x4C..0x53 byte = 4 words
A_PM_WP0     = 0x0830    # API + 0x60 byte = read page 0 word 8
A_PM_WP1     = 0x0844    # API + 0x88 byte = read page 1 word 8
# a_sch[5] at words 15..19 of read page (after a_pm[3] words 8..10 + a_serv_demod[4] words 11..14)
A_SCH_WP0    = 0x0837    # RP0 + 15 word = 0x0828 + 15
A_SCH_WP1    = 0x084B    # RP1 + 15 word = 0x083C + 15
# a_cd[0..14] = CCCH demod result.
# Per CLAUDE.md DWARF-validated 2026-05-26 : NDB+0x1DC byte = NDB+0xEE word
# = code 0x08D4 + 0xEE = 0x09C2. Le shunt utilise 0x1FC (= probablement une
# struct variant alternative). On suit le DWARF qui est l'autorité firmware.
A_CD_BASE    = 0x09C2    # a_cd[0]
A_CD_PAYLOAD = 0x09C5    # a_cd[3] = start of 12-word SI3 packed payload

# ---- Canned values ----
FB_DET_VAL   = 0x4E20     # 20000 = nonzero detection flag
TOA_VAL      = 0x0240     # 576 (mid burst window)
PM_VAL       = 0xFB00     # signed -1280, firmware >>3 = -160, Power = (-160-520)/8 = -85 dBm
ANGLE_VAL    = 0x0000     # AFC error = 0 Hz
SNR_VAL      = 0x4E20     # 20000 = above FB1_SNR_THRESH=3000

# SB encoding for BSIC=7, t1=0, t2=0, t3=1 : sb = (bsic & 0x3f) << 2 = 0x1C
SB_BSIC      = 7
SB_PACKED    = (SB_BSIC & 0x3F) << 2  # = 0x1C
SB_LO        = SB_PACKED & 0xFFFF      # = 0x001C
SB_HI        = (SB_PACKED >> 16) & 0xFFFF  # = 0x0000

# ---- Canned SI3 L2 frame (23 bytes, identical to shunt_dispatch_allc) ----
SI3_BYTES = [
    0x49, 0x06, 0x1B,                    # L2 hdr + RR PD + SI3 mt
    0x00, 0x01,                          # Cell ID = 1
    0x00, 0xF1, 0x10,                    # MCC=001 MNC=01
    0x00, 0x01,                          # LAC = 1
    0x01, 0x00,                          # cell opts + cell select
    0x18, 0xFF, 0xFF,                    # RACH ctrl
    0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B
]
assert len(SI3_BYTES) == 23

# Pack 23 bytes → 12 LE words (lo + hi<<8); last word pads with 0x2B
SI3_WORDS = []
for i in range(0, 23, 2):
    lo = SI3_BYTES[i]
    hi = SI3_BYTES[i+1] if i+1 < 23 else 0x2B
    SI3_WORDS.append(lo | (hi << 8))
assert len(SI3_WORDS) == 12

# ---- Build code ----
code = []

# Boot init
code += stm_lk_mmr(0x5AC8, MMR_SP)        # STM #0x5AC8, SP
code += stm_lk_mmr(0xFFFF, MMR_IMR)       # STM #0xFFFF, IMR

# Compute main_loop address now that we know boot prefix length
main_loop_addr = 0x7120 + len(code)

# --- d_fb_det = 0x4E20 ---
code += stm_lk_mmr(D_FB_DET, MMR_AR0)
code += ld_lk_a(FB_DET_VAL)
code += stl_a_arn(0, 0)                    # STL A, *AR0 (no mod)

# --- a_sync_demod[0..3] = TOA, PM, ANGLE, SNR ---
code += stm_lk_mmr(A_SYNC_DEMOD, MMR_AR0)
code += ld_lk_a(TOA_VAL)
code += stl_a_arn(0, 2)                    # STL A, *AR0+ (post-inc)
code += ld_lk_a(PM_VAL)
code += stl_a_arn(0, 2)
code += ld_lk_a(ANGLE_VAL)
code += stl_a_arn(0, 2)
code += ld_lk_a(SNR_VAL)
code += stl_a_arn(0, 2)

# --- a_pm[3] WP0 ---
code += stm_lk_mmr(A_PM_WP0, MMR_AR0)
code += ld_lk_a(PM_VAL)
code += stl_a_arn(0, 2)
code += stl_a_arn(0, 2)
code += stl_a_arn(0, 2)

# --- a_pm[3] WP1 (reuse A) ---
code += stm_lk_mmr(A_PM_WP1, MMR_AR0)
code += stl_a_arn(0, 2)
code += stl_a_arn(0, 2)
code += stl_a_arn(0, 2)

# --- a_sch[5] WP0 : a_sch[0]=0 (CRC OK), [1]=0, [2]=0, [3]=SB_LO, [4]=SB_HI ---
code += stm_lk_mmr(A_SCH_WP0, MMR_AR0)
code += ld_lk_a(0x0000)
code += stl_a_arn(0, 2)                   # a_sch[0] = 0 (CRC OK)
code += stl_a_arn(0, 2)                   # a_sch[1] = 0
code += stl_a_arn(0, 2)                   # a_sch[2] = 0
code += ld_lk_a(SB_LO)
code += stl_a_arn(0, 2)                   # a_sch[3] = SB low
code += ld_lk_a(SB_HI)
code += stl_a_arn(0, 2)                   # a_sch[4] = SB high

# --- a_sch[5] WP1 same ---
code += stm_lk_mmr(A_SCH_WP1, MMR_AR0)
code += ld_lk_a(0x0000)
code += stl_a_arn(0, 2)
code += stl_a_arn(0, 2)
code += stl_a_arn(0, 2)
code += ld_lk_a(SB_LO)
code += stl_a_arn(0, 2)
code += ld_lk_a(SB_HI)
code += stl_a_arn(0, 2)

# --- a_cd[0..2] = FIRE-OK status (CRC pass, no biterr) ---
code += stm_lk_mmr(A_CD_BASE, MMR_AR0)
code += ld_lk_a(0x0000)
code += stl_a_arn(0, 2)                    # a_cd[0] = FIRE bits = 0 (CRC OK)
code += stl_a_arn(0, 2)                    # a_cd[1] = reserved = 0
code += stl_a_arn(0, 2)                    # a_cd[2] = num_biterr = 0

# --- a_cd[3..14] = 12 words of SI3 LAPDm packed payload ---
code += stm_lk_mmr(A_CD_PAYLOAD, MMR_AR0)
for w in SI3_WORDS:
    code += ld_lk_a(w)
    code += stl_a_arn(0, 2)

# --- Loop back ---
code += b_unc(main_loop_addr)

# ---- Sanity check ----
print(f"Code length: {len(code)} words ({len(code)*2} bytes)")
print(f"Main loop entry: 0x{main_loop_addr:04x}")
print(f"Code spans 0x7120 - 0x{0x7120 + len(code) - 1:04x}")

# ---- Patch PROM0.bin ----
with open(INPUT, 'rb') as f:
    prom_bytes = bytearray(f.read())

print(f"Input PROM0 size: {len(prom_bytes)} bytes ({len(prom_bytes)//2} words)")

# 0x7120 → byte offset 0x240 from PROM0 start (= 2 × (0x7120 - 0x7000))
PATCH_BYTE_OFFSET = 2 * (0x7120 - 0x7000)
print(f"Patch byte offset: 0x{PATCH_BYTE_OFFSET:04x} ({PATCH_BYTE_OFFSET})")

for i, w in enumerate(code):
    off = PATCH_BYTE_OFFSET + 2*i
    prom_bytes[off]   = w & 0xFF
    prom_bytes[off+1] = (w >> 8) & 0xFF

with open(OUTPUT, 'wb') as f:
    f.write(prom_bytes)

print(f"Wrote {OUTPUT}")
print()
print("Pour utiliser SANS toucher l'original :")
print(f"  Update QEMU launch dsp-prom0=...PROM0.bin → dsp-prom0={OUTPUT}")
print(f"  L'original /opt/GSM/calypso_dsp.PROM0.bin reste INTACT.")
