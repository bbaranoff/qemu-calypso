#!/usr/bin/env python3
"""
inject_fcch.py — diagnostic script: synthesize a clean FCCH (FB) burst
and inject it into QEMU's BSP via UDP 127.0.0.1:6702 (TRXDv0 format).

Purpose: validate the DSP FB-detect path independently of bridge.py /
osmo-bts-trx by feeding a known-good FCCH burst with selectable encoding.

GSM FCCH burst:
- 148 bits all "0" → after GMSK modulation: pure tone at fc + 67.7083 kHz
  (= 1625/24 kHz, modulation index h=0.5, bit rate 270.833 kbps)
- Burst duration: 148 × 3.69231 µs = 546.5 µs
- The DSP correlator at PROM0 0x77xx-0x88xx searches for this pure-tone
  burst by computing autocorrelation peak.

TRXDv0 wire format (per bridge.py + calypso_bsp.c):
  byte 0     : TN (timeslot 0..7)
  bytes 1-4  : FN (uint32 big-endian)
  byte 5     : RSSI (uint8, dBm offset)
  bytes 6-7  : TOA (int16 big-endian)
  bytes 8.. : payload (encoding-dependent, see modes)

Three injection modes available:
  --mode bytes_zero    : 148 zero bytes (default — same as inject_fb.py)
  --mode soft_neg127   : 148 × 0x81 (signed -127 = confident "0" bit)
  --mode iq_raw        : 296 int16 = 148 I/Q complex samples synthesized
                         from a +67.7 kHz GMSK sinusoid

Use --mode iq_raw if the BSP DMA path expects I/Q samples directly.
Use --mode bytes_zero or soft_neg127 if it expects soft bits.
"""
import argparse
import math
import os
import re
import socket
import struct
import sys
import time

BRIDGE_LOG = "/tmp/bridge.log"
BSP_ADDR = ("127.0.0.1", 6702)

# GSM constants
GSM_BIT_RATE   = 270833.333          # bits per second (= 13MHz / 48)
FCCH_FREQ_HZ   = 1625e3 / 24         # = 67708.333... Hz, FCCH tone offset
SYMBOL_PERIOD  = 1.0 / GSM_BIT_RATE  # 3.692 µs per symbol
NUM_SYMBOLS    = 148                 # FCCH/normal burst length

def make_iq_fcch_samples(amplitude=0.7, samples_per_symbol=1):
    """Synthesize 148*samples_per_symbol complex I/Q samples for an FCCH
    burst (pure tone at FCCH_FREQ_HZ above carrier).

    Returns int16 sequence of [I0, Q0, I1, Q1, ...] suitable for direct
    DMA injection. With samples_per_symbol=1 (default), 148 I/Q pairs =
    296 int16 = 592 bytes — matches calypso_bsp.c iq[296] buffer."""
    n = NUM_SYMBOLS * samples_per_symbol
    fs = GSM_BIT_RATE * samples_per_symbol  # sample rate
    out = bytearray()
    scale = int(amplitude * 0x7FFE)
    for k in range(n):
        t = k / fs
        phase = 2 * math.pi * FCCH_FREQ_HZ * t
        # Q15 fixed-point I/Q samples
        I = int(math.cos(phase) * scale)
        Q = int(math.sin(phase) * scale)
        # Clamp to int16
        I = max(-0x7FFE, min(0x7FFE, I))
        Q = max(-0x7FFE, min(0x7FFE, Q))
        out += struct.pack(">hh", I, Q)
    return bytes(out)

def make_burst(tn, fn, mode, rssi=20, toa=0):
    """Build TRXDv0-formatted DL burst with selected payload encoding."""
    header = (
        bytes([tn & 0x07])
        + struct.pack(">I", fn & 0xFFFFFFFF)
        + bytes([rssi])
        + struct.pack(">H", toa & 0xFFFF)
    )
    if mode == "bytes_zero":
        payload = bytes(148)             # 148 × 0x00
    elif mode == "soft_neg127":
        payload = bytes([0x81]) * 148    # 148 × signed -127 (confident "0")
    elif mode == "iq_raw":
        payload = make_iq_fcch_samples() # 148 I/Q pairs = 592 bytes
    else:
        raise ValueError(f"unknown mode {mode!r}")
    return header + payload

def latest_fn_from_bridge():
    """Walk bridge.log backwards looking for the most recent 'fn=N' tag."""
    try:
        with open(BRIDGE_LOG, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 8192))
            tail = f.read().decode("utf-8", errors="replace")
        m = list(re.finditer(r"\bfn=(\d+)", tail))
        if m:
            return int(m[-1].group(1))
    except FileNotFoundError:
        pass
    return None

def main():
    ap = argparse.ArgumentParser(
        description="Inject synthesized FCCH burst into QEMU BSP",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["bytes_zero", "soft_neg127", "iq_raw"],
                    default="bytes_zero",
                    help="payload encoding (default: bytes_zero)")
    ap.add_argument("--lookahead", type=int, default=30,
                    help="frames ahead of cur_fn to schedule the burst (default: 30)")
    ap.add_argument("--period-ms", type=float, default=4.0,
                    help="send period in milliseconds (default: 4 = ~1 TDMA frame)")
    ap.add_argument("--duration", type=float, default=30.0,
                    help="run duration in seconds (default: 30)")
    ap.add_argument("--tn", type=int, default=0, help="timeslot (default: 0)")
    ap.add_argument("--rssi", type=int, default=20, help="RSSI tag (default: 20)")
    ap.add_argument("--toa", type=int, default=0, help="TOA tag (default: 0)")
    ap.add_argument("--target", default="127.0.0.1:6702",
                    help="BSP UDP endpoint (default: 127.0.0.1:6702)")
    args = ap.parse_args()

    host, port = args.target.split(":")
    target = (host, int(port))
    period = args.period_ms / 1000.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"inject_fcch: mode={args.mode} target={target} "
          f"lookahead={args.lookahead} period={period*1000:.1f}ms "
          f"duration={args.duration}s", flush=True)

    # Wait until bridge.log has at least one fn= line
    start = time.time()
    fn = None
    while time.time() - start < 10:
        fn = latest_fn_from_bridge()
        if fn is not None:
            break
        time.sleep(0.2)
    if fn is None:
        print("inject_fcch: no fn= seen in bridge.log within 10s — "
              "starting from FN=0 (no sync)", file=sys.stderr)
        fn = 0
    else:
        print(f"inject_fcch: synced from bridge, starting FN={fn}", flush=True)

    sample = make_burst(args.tn, 0, args.mode, args.rssi, args.toa)
    print(f"inject_fcch: payload encoding {args.mode} "
          f"→ packet size {len(sample)} bytes "
          f"(header 8 + payload {len(sample)-8})", flush=True)

    sent = 0
    end = time.time() + args.duration
    next_send = time.time()
    while time.time() < end:
        # Refresh FN from bridge if available; advance manually otherwise
        live_fn = latest_fn_from_bridge()
        if live_fn is not None:
            fn = live_fn
        target_fn = (fn + args.lookahead) & 0xFFFFFFFF

        burst = make_burst(args.tn, target_fn, args.mode,
                           args.rssi, args.toa)
        try:
            sock.sendto(burst, target)
            sent += 1
            if sent <= 10 or sent % 100 == 0:
                print(f"inject_fcch: sent #{sent} TN={args.tn} "
                      f"FN={target_fn} (cur_fn={fn})", flush=True)
        except OSError as e:
            print(f"inject_fcch: send error: {e}", file=sys.stderr)

        next_send += period
        sleep = next_send - time.time()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_send = time.time()  # we slipped, resync

    print(f"inject_fcch: done — {sent} bursts sent", flush=True)

if __name__ == "__main__":
    main()
