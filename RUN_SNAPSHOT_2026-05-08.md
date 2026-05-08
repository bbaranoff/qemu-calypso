# Run snapshot — 2026-05-08, after rsl_si_tap.py + BCCH_INJECT purge

## Process state

```
QEMU         PID 38836  alive  ~2m45s uptime
osmo-bts-trx PID 38906  alive  no shutdown (period=51 OK)
bridge.py    PID 38889  alive
mobile       PID 38945  alive  (stuck cell-search)
osmocon      PID 38872  alive
osmo-bsc/msc/hlr/mgw/stp/ggsn/sgsn/pcu  alive
tcpdump      PID 38947  -i any → /root/mobile-gsmtap.pcap
```

## Effective env (from `/proc/$QEMU_PID/environ`)

```
BRIDGE_CLK_FROM_QEMU=0          (default — wall-paced)
BRIDGE_CLK_PERIOD=51            (default — BTS-friendly)
BRIDGE_UL_FN_REWRITE=0          (shell residue — passthrough qfn)
CALYPSO_FBSB_SYNTH=1            (synth FB/SB still on)
CALYPSO_W1C_LATCH=0
CALYPSO_NDB_D_RACH_OFFSET=0x023a
CALYPSO_RACH_FORCE_BSIC=7
CALYPSO_DSP_FBDET_SKIP=1
CALYPSO_DSP_IDLE_FF=1
CALYPSO_ICOUNT=off
CALYPSO_BSP_DARAM_ADDR=0x3fb0
CALYPSO_DSP_ROM=/opt/GSM/calypso_dsp.txt
```

NO `CALYPSO_BCCH_INJECT`. NO `CALYPSO_SI_MMAP_PATH`. Purge confirmed.

## Bridge banner + first events

```
bridge: CLK=5700 TRXC=5701 TRXD=5702 ↔ BSP@6702
bridge: CLK IND source=wall_fn (wall-paced 217 Hz), period=51 frames
bridge: UL FN rewrite mode=off — off (passthrough qemu_fn)
bridge: QEMU tick #1 qfn=202 wall_fn=0 lag=2715446
bridge: UL #1 TN=4 qfn=235=sent=235 slot=31/51=OFF  (boot artifact)
TRXC < CMD POWEROFF  →  > RSP POWEROFF 0
TRXC < CMD RFMUTE 1  →  > RSP RFMUTE 0 1
TRXC SETFORMAT requested=v2 → forced reply v0  (BTS asks v2, bridge forces v0)
TRXC < CMD SETFORMAT 0  →  > RSP SETFORMAT 0 0 0
TRXC < CMD RXTUNE 890200  →  > RSP RXTUNE 0 890200
TRXC < CMD TXTUNE 935200  →  > RSP TXTUNE 0 935200
TRXC < CMD SETTSC 7  →  > RSP SETTSC 0 7
TRXC < CMD NOMTXPOWER  →  > RSP NOMTXPOWER 0 50
BTS: POWERON (wall_fn anchor at t0, qfn=311)
TRXC < CMD POWERON  →  > RSP POWERON 0
bridge: UL #2 TN=0 qfn=404=sent=404 slot=47/51=RACH  (initial RACH at boot)
```

## Bridge stats — most recent

```
tick=11942  clk=528  dl=214998  ul=2  ul_rewrite=0  ul_slot_in/off=1/1
wall_fn=26925  qfn=12143
ratio qfn/wall_fn = 0.451 — QEMU runs ~2.2× slower than wall
```

215k DL bursts forwarded since POWERON, only 2 UL (both boot artifacts).

## BTS state — VTY counters

```
NM State: Oper 'Enabled', Admin 'Unlocked', Avail 'OK'
OML Link state: connected.
Received paging requests (Abis):     0
Received RACH requests (Um):         0    ← no RACH ever decoded
Dropped RACH requests (Um):          0
Received AGCH requests (Abis):       0
```

## BTS scheduler — recent CLK Ind log

```
DTRX INFO Clock indication: fn=20451
DL1C INFO TRX Clock Ind: elapsed_us= 235341, elapsed_fn= 51, error_us= -24
DL1C INFO GSM clock jitter: -192us (elapsed_fn=0)
```

`error_us` ±2000us range — well inside skew tolerance, no shutdown risk.

## Mobile state — log content

```
Mobile '1' initialized, please start phone now!
Available via telnet 127.0.0.1 4247
gsm322.c:2929 using DSC of 90  (×20 in stuck cell-search loop)
```

```
0× MM_EVENT_NO_CELL_FOUND
0× RANDOM ACCESS
0× MON
0× CGI=
0× RR_EST_REQ
20× "using DSC of 90"
```

Mobile doesn't even reach the "no cell found" event yet — stuck earlier in
the cell-search state machine. Probably still in initial DSC ramp.

## CRITICAL FINDING — BSP RX FN delta growing unbounded

Recent BSP DL receive trace from qemu.log :

```
[BSP] RX tn=0 fn=26803 cur_fn=12109 delta=14694
[BSP] RX tn=0 fn=26928 cur_fn=12143 delta=14785
[BSP] RX tn=0 fn=27053 cur_fn=12170 delta=14883
[BSP] RX tn=0 fn=27178 cur_fn=12212 delta=14966
[BSP] RX tn=0 fn=27303 cur_fn=12275 delta=15028
[BSP] RX tn=0 fn=27428 cur_fn=12337 delta=15091
```

`delta = bts_fn - qfn` is **monotonically growing** (14694 → 15091 over a
few seconds) because BTS scheduler runs at wall-clock rate and QEMU at
~half wall.

`BSP_FN_MATCH_WINDOW = 64` (constant in `calypso_bsp.c`).

**delta = ~15000 ≫ 64** ⇒ every DL burst is outside the match window.

**This is the actual root blocker for DL.** The DSP CCCH demod doesn't
fail to converge — it never receives the bursts in the first place because
BSP rejects them at the FN match check.

The hack `BCCH_INJECT` masked this by writing `a_cd[]` directly without
needing BSP→DSP DMA at all. Now that the hack is gone, the FN mismatch
is exposed.

## DSP execution profile (qemu.log)

```
PC HIST insn=2018009060 top:
  cc62:166667  cc63:166667  cc65:166667  cc66:166667  cc67:166667
  cc68:166667  cc69:166667  cc6f:166667  cc6a:166666  cc6b:166666
  cc6d:166666  cc6e:166666
```

DSP spends 100% of its time in PC range `0xCC62..0xCC6F` = the **idle
dispatcher polling loop** in PROM0. Never enters fb-det, never enters CCCH
demod. Consistent with the BSP-blocks-all-DL finding above.

```
DARAM RD HIST (FB-det, reads=1.6e9):
  0062:5.0e8       ← top read (5x more than any other)
  1f00:5.0e8       ← d_fb_det polling
  1f0c:5.0e8       ← d_fb_mode polling
  3dd2:9.3e7
  0078:9.7e6
```

DSP polls `0062`, `1f00`, `1f0c` heavily — these are the fb-det / a_sync /
d_fb_det cells the dispatcher checks to decide whether to dispatch to a
real handler. They're never set (no real burst processing) so DSP stays
in poll loop.

## DSP_TASK_ALLC firing from ARM L1 (qemu.log)

```
[calypso-fbsb] on_dsp_task_change task=24 fn=12217 state=6
[calypso-fbsb] on_dsp_task_change task=24 fn=12218 state=6
... (firing every frame after FBSB synth)
```

ARM L1 IS writing `d_task_md=24` (DSP_TASK_ALLC) to ask DSP for CCCH
demod. The QEMU hook fires correctly. But DSP never *executes* CCCH demod
because :
- DSP idle dispatcher polls task slots
- task slots get the value, but DSP routine for CCCH demod expects bursts
  in DARAM via BSP DMA
- BSP DMA never delivers (FN mismatch above)

state=6 is the FBSB state machine post-FBSB-synth (FBSB_FB1_FOUND or
similar — done with FBSB).

## TDMA tick health (qemu.log)

```
[tdma-skip] fn=12200 skipped=1 work_dt=9174294
[tdma-skip] fn=12300 skipped=1 work_dt=8588146
```

TDMA tick is occasionally skipping frames because work duration (~9ms)
exceeds GSM_TDMA_NS (4.615ms). Host load issue. Not the primary blocker
but contributes to qfn lag widening.

## fw-console UART overflow (qemu.log)

```
LOST count: 7812
[fw-console] LOST 3893!
[fw-console] LOST 3729!
[fw-console] LOST 3754!
```

ARM firmware UART sercomm buffer overflows under stderr write pressure.
Known issue (review point #9 — stderr in real-time path).

## GSMTAP capture (1213 packets in pcap)

All packets are BTS-side (clock IND telemetry, RSL RF RESource IND, mobile
"DSC of 90" chatter). NO RACH, NO IMM_ASS, NO BCCH SI exchange. Confirms
DL chain dead.

## UDP socket map

```
QEMU            127.0.0.1:6702  recv-q=15360 (≈100 bursts queued, BSP backed up)
bridge          127.0.0.1:5700  CLK     ← BTS:5800
bridge          127.0.0.1:5701  TRXC    ↔ BTS:5801
bridge          127.0.0.1:5702  TRXD    ↔ BTS:5802
osmo-bts-trx    127.0.0.1:5800  ESTAB peer 5700
osmo-bts-trx    127.0.0.1:5801  ESTAB peer 5701
osmo-bts-trx    127.0.0.1:5802  ESTAB peer 5702
mobile          172.20.0.11 → 172.20.0.1:4729  (GSMTAP)
osmo-bts-trx    172.20.0.11 → 172.20.0.1:4729  (GSMTAP)
```

`Recv-Q=15360` on QEMU 6702 = bridge sending DL faster than QEMU drains
(BSP processes bursts at qfn rate). Not packet loss yet but consistent
with FN-overflow pattern.

## Files state

```
md5  bridge.py                                  972b38accea7f4a24f38cd167c1d5e3c
md5  run_si.sh                                  8af74418b33068e5e5263b7167ab46c9
md5  CLAUDE.md                                  569e2051be9491124144389feb9638bb
md5  README.md                                  4bb9c38a25c7713957370caadff4e0ba
md5  DIAG_FOR_CLAUDE_WEB.md                     7bfa047b02e8166befe51f5d2c274741
md5  hw/arm/calypso/calypso_fbsb.c              4c7a7d2a48bfa0acd0e935df05a483a3
md5  hw/arm/calypso/calypso_bsp.c               3f6dc13a9a6ffb2aae991af8fe7c3518
md5  hw/arm/calypso/calypso_trx.c               33bd5af22c37f3a0d096d0f05133b170
```

`scripts/rsl_si_tap.py` deleted. `/dev/shm/calypso_si.bin` absent.

## Resolution applied (2026-05-08 evening)

**Option A — slot-aware DL FN rewrite** implemented in `bridge.py`.

New env vars in `run_si.sh` :
- `BRIDGE_DL_FN_REWRITE` — `slot` (default), `naive`, `off`
- `BRIDGE_DL_FN_LOOKAHEAD` — default 32 (half of BSP_FN_MATCH_WINDOW=64)

Algorithm (O(1)) :
```python
def dl_slot_aware_qfn(bts_fn, current_qfn, lookahead):
    target_mod = bts_fn % 51
    delta = (target_mod - (current_qfn % 51)) % 51
    if delta > lookahead:
        return None  # drop, BCCH repeats
    return current_qfn + delta
```

Preserves `(FN % 51)` so DSP demod still types BCCH/CCCH/SDCCH correctly.
Drops bursts whose target slot is more than `lookahead` frames ahead — BCCH
repeats every 51 frames so missed slots get re-presented.

Stats counters added :
- `dl_rewrite` — number of DL bursts FN-rewritten
- `dl_drop_no_slot` — number of DL bursts dropped because no qfn within
  lookahead matches the target `% 51`

Expected post-fix observation in qemu.log :
- `[BSP] RX delta` should drop from ~15000 to 0..lookahead
- `[c54x] PC HIST` should leave the `0xCC62..0xCC6F` idle dispatcher
- `[c54x] DARAM RD HIST` should show non-zero reads on a_cd[] addresses
- `mobile.log` should show `New SYSTEM INFORMATION 3 (lai=001-01-1)` and
  `MON CGI=001-01-1-888`

## Original diagnosis

The actual blocker is **NOT** "DSP CCCH demod doesn't converge". It is :

> **BSP DL queue rejects 100% of bursts due to FN mismatch.**
> `delta = bts_fn - qfn ≈ 15000` while `BSP_FN_MATCH_WINDOW = 64`.

The DSP demod path was never even reached. The hack `BCCH_INJECT` masked
this by feeding `a_cd[]` directly. Now without the hack, the asymmetric
clock domains (wall-paced BTS, qfn-paced QEMU/BSP) make all DL bursts
land outside the BSP acceptance window.

### Two viable fixes

**A. DL FN rewrite at bridge** (symmetric to the UL rewrite work) :
   - Bridge currently forwards DL `bts_fn` unchanged to QEMU
   - Add a `BRIDGE_DL_FN_REWRITE` mode that rewrites `bts_fn → qfn + ε`
     so the burst lands inside QEMU's BSP match window
   - Need to preserve slot-modulo (BCCH/AGCH/SDCCH slot identity within
     51-multiframe) — not just any `qfn`, but the next `qfn` whose
     `% 51` matches `bts_fn % 51`
   - ~30 lines Python, env-gated, preserves wall-paced CLK IND for BTS

**B. Widen BSP_FN_MATCH_WINDOW** :
   - Set `BSP_FN_MATCH_WINDOW = max_observed_delta + margin` (so ~16384)
   - Trivial 1-line change in `calypso_bsp.c`
   - Risk : BSP queue may grow unbounded, deliveries far in the past
   - Less clean than (A) but valid as a probe

### Test plan

1. Apply (A) or (B). Confirm `cur_fn` catches up with bursts via
   `[BSP] RX delta` log → delta should drop to 0..few-frames.
2. Watch for first DSP CCCH demod attempt (PC moves out of `0xCC62..0xCC6F`).
3. Watch for `a_cd[]` write from DSP side.
4. Mobile log : expect SI 1/2/3/4 reception, then `MON CGI=001-01-1-888`.
5. Then UL chain (RACH/IMM_ASS) becomes testable for the first time
   on the legitimate path.

### If (A)/(B) doesn't unlock CCCH

Then the real DSP CCCH demod implementation in `calypso_c54x.c` is broken
on the actual GMSK soft bits. Possible candidates :
- Channel-coding inverse (deinterleaver, FIRE check)
- Soft-bit polarity convention DSP-side
- DARAM addr/word-size assumption mismatch

But fix (A) or (B) FIRST — without bursts reaching DSP, nothing else can
be diagnosed.
