# DIAG — UL RACH/IMM_ASS chain blocker (2026-05-07)

QEMU Calypso emulator. **Symptom**: Mobile completes Location Update RR_EST_REQ →
sends 8 RANDOM ACCESS bursts → T3126 5s timeout → no IMM_ASS_CMD ever arrives.
DL chain (BCCH SI 1/2/3/4) works end-to-end (mobile camps with CGI=001-01-1-888).

## TL;DR — where the chain is broken

```
mobile L3 → mobile L1 → DSP RACH encode → BSP UDP → bridge → osmo-bts-trx
   ✅          ✅           ✅                ✅        ✅          ❌
```

`osmo-bts-trx` VTY counter `Received RACH requests (Um): 0` after ~5 min of run
during which the bridge forwarded 24 UL bursts to the BTS data port.

**Bursts arrive at BTS UDP socket but are not RACH-decoded.**

## Pipeline summary (validated points)

| Step | What | Evidence |
|------|------|----------|
| L3 RACH gen | mobile sends `RR_EST_REQ`, calls `gsm48_rr.c:2613 RANDOM ACCESS` 8× per cycle | mobile.log shows `ra 0x0e, 0x0a, 0x0f, 0x03, 0x07, 0x09, 0x09, 0x01` then T3126 |
| L1 RACH path | ARM writes `d_task_ra=0x0001` + `d_rach` byte = RA value (validated via D_RACH-FINDER tracer) | qemu.log: `D_RACH-FINDER #110 off=0x01e2 val=0x1117 ... ra=0x11 bsic=0x05 fn=20451 *HOT*` (note: 0x01e2 = 0x023a*2-something? actually d_rach offset confirmed 0x023a from earlier traces) |
| DSP RACH encode | `gsm0503_rach_ext_encode(bits, ra, bsic, false)` (libosmocoding) | calypso_trx.c uses libosmocoding ; bsic forced via `CALYPSO_RACH_FORCE_BSIC=7` |
| BSP UDP send | `calypso_bsp_send_ul(tn, fn, bits)` → 156-byte TRXDv0 to `127.0.0.1:5702` | calypso_bsp.c:584 `sendto(bsp.trxd_fd, pkt, 156, ...)` |
| Bridge UL recv | bridge receives on 5702 from QEMU:6702 | bridge.log: `bridge: UL #21 TN=0 qfn=19723→wfn=45319 rssi=-60 toa=0 len=156 hdr_in=0000004d0b3c0000 hdr_out=000000b1073c0000 bits[0:16]=[+127 +127 +127 -127 +127 -127 -127 +127 ...] bits[140:148]=[-127 ×8] → BTS ('127.0.0.1', 5802)` |
| Bridge → BTS UDP | bridge sendto BTS:5802, src port 5702 | `ss -uap`: `127.0.0.1:5802 ESTAB ... 127.0.0.1:5702` (BTS connect()'d) |
| BTS RACH decode | **0 RACH decoded** | BTS VTY: `Received RACH requests (Um): 0 (0/s 0/m 0/h 0/d)` |

## Network state (relevant sockets)

```
osmo-bts-trx (PID 33273) connect()'d UDP:
  127.0.0.1:5800 ↔ 127.0.0.1:5700  (clock IND  — BTS rx)
  127.0.0.1:5801 ↔ 127.0.0.1:5701  (TRXC ctrl  — bidi)
  127.0.0.1:5802 ↔ 127.0.0.1:5702  (TRXD data  — bidi)

bridge.py (PID 33241) bind unconnected UDP:
  127.0.0.1:5700  (sends CLK IND to 5800)
  127.0.0.1:5701  (TRXC ctrl)
  127.0.0.1:5702  (TRXD data)
  127.0.0.1:6700  (recv QEMU TDMA tick)

QEMU (PID 33188) UDP:
  127.0.0.1:6702  (TRXDv0 bidi with bridge:5702)
```

Ports & roles correct per osmo-trx convention (TRX-side = 5700/01/02, BTS-side = 5800/01/02).

## Bridge UL hex evidence (UL #21, representative)

```
Header (8 B TRXDv0):  hdr_out = 00 00 00 b1 07 3c 00 00
                                ^^ ^^^^^^^^^^^ ^^ ^^^^^
                                TN  FN(BE)     RSSI ToA
                                0   0xb1073c   0x3c -60dBm  0
Payload: 148 soft bits ∈ {-127, +127} (signed bytes)
  bits[0:16]  = [+127 +127 +127 -127 +127 -127 -127 +127 -127 +127 -127 -127 -127 -127 +127 +127]
  bits[140:148] = [-127 ×8]   ← guard period zeros, validated
Total len   = 156 bytes
```

Header layout matches TRXDv0 spec (TN=1, FN=4 BE, RSSI=1, ToA=2, bits=148).

## Wall-clock vs QEMU FN drift

QEMU emulates ~2.3× slower than wall clock (DSP cycle cost). Bridge maintains
its own wall-paced FN (`wall_fn`) and **rewrites the UL header FN** before
forwarding so BTS scheduler sees a wall-aligned timestamp:

```
bridge.log:
  qfn=19723 → wfn=45319   (ratio 2.30, drift +25596 frames)
  qfn=21727   wall_fn=50047  (latest stat, BTS sees CLK IND at wall_fn)
```

CLK IND at wall-clock cadence (every CLK_IND_PERIOD=51 frames * 4.615ms = 235ms)
→ BTS log shows steady `elapsed_fn=51, error_us≈±300` ✅ scheduler stable, no
"PC clock skew" shutdown.

## What works (DL chain end-to-end)

- BCCH_INJECT writes a_cd[] from `/dev/shm/calypso_si.bin` (rsl_si_tap.py mmap)
- DSP exposes a_cd to ARM L1 — mobile L3 decodes SI 1/2/3/4 with `lai=001-01-1`
- Mobile log: `MON: f=1 lev=<=-110 snr= 0 ber=  0 CGI=001-01-1-888` (camped) ✅
- 359k+ DL bursts forwarded bridge→QEMU (mostly idle filler since no IMM_ASS to send)

## Hypotheses for the UL→BTS RACH-decode failure

Sorted by likelihood:

### (1) TRXDv0 vs TRXDv1 version negotiation mismatch

`bridge.py` echoes back any `CMD SETFORMAT N` from BTS without changing its
output format:

```python
elif verb == "SETFORMAT":
    rsp = f"RSP SETFORMAT 0 {' '.join(args)}"
```

If osmo-bts-trx asks for TRXDv1 at startup, bridge replies "agreed" but
continues sending TRXDv0 (no version byte, fixed 156-byte). osmo-bts-trx may
silently drop bursts whose layout doesn't match the negotiated version.

**Test**: log all CMD SETFORMAT exchanges in bridge.py. Verify what version
osmo-bts-trx asked for and ensure bridge actually sends matching format.

### (2) RACH window TN/FN mismatch with combined CCCH+SDCCH8 scheduler

The cell config has combined CCCH+SDCCH8 on TN=0 (default `osmo-bsc.cfg`).
RACH bursts on the uplink TN=0 are valid only at specific FN%51 slots:
{4..10, 14..20, 24..30, 34..40, 44..50} (extended set with combined chan).

Mobile L3 says `Tx-integer 12 combined yes S(lots) 115` ← S=115 is the
RACH-allowed slot mask.

Bridge **rewrites the burst FN to current `wall_fn`**. wall_fn at moment of
bridge UL forward is uncoupled from the FN at which the mobile DSP physically
emitted the burst. If wall_fn % 51 lands outside the combined-RACH slot mask,
osmo-bts-trx's scheduler discards the burst (no RACH detection scheduled at
that slot).

**Test**: bypass the FN rewrite for UL — pass through QEMU's qfn unchanged. If
BTS RACH counter goes >0, this is the cause. Note: bts may complain about FN
skew but at least RACH window match is correct.

### (3) BSIC mismatch in encoded burst

`CALYPSO_RACH_FORCE_BSIC=7` is set ; this overrides the BSIC byte in `d_rach`
before encoding. BTS expects BSIC=osmo-bsc.cfg `base_station_id_code`. If the
two don't match, the RACH FIRE check fails and BTS counts as
`Dropped RACH requests (Um)` — but the VTY shows that counter also at 0, so
the bursts aren't even reaching FIRE check. Eliminates this for now.

### (4) RA bit ordering / bit-soft polarity

`gsm0503_rach_ext_encode(bits, ra, bsic, false)` produces 36-bit bursts ; we
zero-pad bits[140:148] for the 60-bit guard period and convert each {0,1}
output bit to a signed soft-bit `(bit ? +127 : -127)`. The hex dump confirms
±127 amplitudes and zero guard. But polarity could be inverted (osmo-bts-trx
expects `-127 = 1, +127 = 0` — Yates' convention?). Worth a one-byte flip-test.

### (5) BSP UL UDP loss between QEMU and bridge

Mobile generates ~8 RA per cycle × 6+ cycles = ~50 RA total.
Bridge stat: `ul=24` only.
Loss rate = ~50%. Suggests not every L1 RACH commit makes it to bridge.

Could be QEMU `sendto` failing silently, or DSP not actually firing every
RACH commit (some L1 RACH events not reaching DSP encode under `icount=off`
slow QEMU). Less critical than (1)/(2) — even a single decoded RACH would
be a win.

### (6) BTS L1 not in RX-listen state on TN=0 UL

Possibility that osmo-bts-trx scheduler hasn't enabled the UL RX path for
TN=0 because some PHY init step never completed. The log only shows CLK IND
events, no "PHY ready" / "TRX activated" type message. But that may just be
log filter (info level). RSL is `connected` per VTY ; OML state `Enabled` ;
`Avail OK`. So it should be active.

## Files relevant to the failure

| File | Role | Lines of interest |
|------|------|------|
| `bridge.py` | UDP relay TRX↔BTS, FN rewrite | `_handle_ul()`, `_send_clk_ind()` — both use wall_fn |
| `hw/arm/calypso/calypso_bsp.c` | BSP DMA, UL UDP send | `calypso_bsp_send_ul()` lines 535-585 |
| `hw/arm/calypso/calypso_trx.c` | TRX/TPU/TSP, RACH path orchestration | grep `DB_W_D_TASK_RA = 7`, `gsm0503_rach_ext_encode` |
| `/etc/osmocom/osmo-bts-trx.cfg` | BTS config | `min-qual-rach 0`, `gsmtap-sapi rach`, `osmotrx ip local 127.0.0.1` |

## Quick discriminants (to pick the right hypothesis)

A. Add SETFORMAT logging to bridge.py to see what version was negotiated.

B. Run with bridge UL FN rewrite **disabled** (pass-through):
   ```python
   # in _handle_ul: skip the wfn rewrite
   self.trxd_sock.sendto(data, self.trxd_remote)
   ```
   If RACH counter goes >0, hypothesis (2) confirmed → need a smarter rewrite
   (rewrite only if QEMU FN is too stale, AND target the nearest valid RACH
   slot).

C. tcpdump GSMTAP on lo:4729 during a session. If IMM_ASS visible on air →
   different problem. If not + RACH=0 → confirm UL pipeline blockage.
   ```
   tcpdump -i lo -w /tmp/gsmtap.pcap udp port 4729
   ```

D. Soft-bit polarity flip test:
   ```c
   // calypso_trx.c, after gsm0503_rach_ext_encode
   for (int i = 0; i < 148; i++) bits[i] = bits[i] ? 0 : 0xFF;  // invert
   ```
   If RACH counter goes >0 with this, polarity was wrong.

## Run config (snapshot)

```bash
unset CALYPSO_NDB_D_RACH_OFFSET CALYPSO_DSP_IDLE_RANGE CALYPSO_W1C_LATCH \
      CALYPSO_UART_TRACE
CALYPSO_FBSB_SYNTH=1 CALYPSO_BCCH_INJECT=1 \
CALYPSO_RACH_FORCE_BSIC=7 \
CALYPSO_DSP_FBDET_SKIP=1 \
CALYPSO_ICOUNT=off \
BRIDGE_CLK_FROM_QEMU=1 BRIDGE_CLK_PERIOD=51 \
CALYPSO_NDB_D_RACH_OFFSET=0x023a \
./run_si.sh
```

ENV summary actually printed by run_si.sh:
```
CALYPSO_FBSB_SYNTH          = 1
CALYPSO_BCCH_INJECT         = 1
CALYPSO_W1C_LATCH           = 0
CALYPSO_NDB_D_RACH_OFFSET   = 0x023a
CALYPSO_RACH_FORCE_BSIC     = 7
BRIDGE_CLK_FROM_QEMU        = 1
CALYPSO_ICOUNT              = off
CALYPSO_DSP_IDLE_FF         = 1
```

## State of the run at diag time

```
Process    PID    Status
qemu       33188  alive (~5 min uptime)
osmo-bts   33273  alive (no shutdown — wall_fn CLK IND keeps scheduler happy)
bridge     33241  alive (24 UL forwarded, 22 FN-rewritten, 359k DL forwarded)
mobile     33312  alive (T3126 cycle, repeatedly retrying LU)
```

Bridge stats:
```
tick=21533 clk=982 dl=399976 ul=24 ul_rewrite=22 wall_fn=50047 qfn=21727
```

Mobile log most recent:
```
gsm48_rr.c:7026 (ms 1) Message 'RR_EST_REQ' received in state idle (sapi 0)
gsm48_rr.c:475 new state idle -> connection pending
gsm48_rr.c:2557 RANDOM ACCESS (requests left 8) → ra 0x0e
... (8 retries) ...
gsm48_rr.c:2561 Done with sending RANDOM ACCESS bursts
gsm48_rr.c:1013 starting T3126 with 5.000 seconds
gsm48_rr.c:922 timer T3126 has fired
gsm48_rr.c:475 new state connection pending -> idle
```

## Recommended next experiment

Try hypothesis (2) first — disable bridge FN rewrite for UL to see if BTS
decodes any RACH. This is the smallest reversible change and pivots the
diagnosis.

If (2) doesn't help, instrument SETFORMAT in bridge to validate (1).
