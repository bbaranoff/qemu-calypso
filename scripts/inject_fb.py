#!/usr/bin/env python3
"""
inject_fb.py — diagnostic script: send a clean FB burst (148 zero bits)
straight to QEMU's BSP UDP port (127.0.0.1:6702), bypassing osmo-bts-trx.

Purpose: confirm whether the DSP correlator at PROM0 0x82f6 actually
writes d_fb_det when fed a perfect frequency burst. If yes → the loop
in production runs is failing on amplitude/phase/timing of the bridge's
GMSK simulation. If no → the DSP code path itself is broken.

We tail bridge.log to track QEMU's current FN (the bridge prints
"bridge: QEMU tick #N FN=X" on every TDMA tick) and send each burst
slightly in the future of the live FN so it lands in the BSP match
window (±64).
"""
import os
import re
import socket
import struct
import sys
import time

BRIDGE_LOG = "/tmp/bridge.log"
BSP_ADDR = ("127.0.0.1", 6702)
LOOKAHEAD = 30          # frames ahead of cur_fn (well inside ±64 window)
SEND_PERIOD_S = 0.004   # ~one TDMA frame
DURATION_S = 30

def make_fb_burst(tn, fn, rssi=20, toa=0):
    """TRXDv0 DL: tn(1) fn(4 BE) rssi(1) toa(2 BE) + 148 zero bits."""
    return (
        bytes([tn & 0x07])
        + struct.pack(">I", fn & 0xFFFFFFFF)
        + bytes([rssi])
        + struct.pack(">H", toa & 0xFFFF)
        + bytes(148)
    )

def latest_fn_from_bridge():
    """Walk bridge.log backwards looking for the most recent 'fn=N' tag."""
    try:
        with open(BRIDGE_LOG, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            chunk = 8192
            f.seek(max(0, size - chunk))
            tail = f.read().decode("utf-8", errors="replace")
        # Prefer DL bursts (carry bts_fn and our rewritten fn) for freshness
        m = list(re.finditer(r"\bfn=(\d+)", tail))
        if m:
            return int(m[-1].group(1))
    except FileNotFoundError:
        pass
    return None

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"inject_fb: target={BSP_ADDR} lookahead={LOOKAHEAD} duration={DURATION_S}s",
          flush=True)

    # Wait until bridge.log has at least one fn= line
    start = time.time()
    while time.time() - start < 10:
        fn = latest_fn_from_bridge()
        if fn is not None:
            break
        time.sleep(0.2)
    else:
        print("inject_fb: no fn= seen in bridge.log within 10s", file=sys.stderr)
        sys.exit(1)
    print(f"inject_fb: starting FN={fn}", flush=True)

    sent = 0
    end = time.time() + DURATION_S
    while time.time() < end:
        cur = latest_fn_from_bridge()
        if cur is None:
            cur = fn
        target_fn = cur + LOOKAHEAD
        sock.sendto(make_fb_burst(tn=0, fn=target_fn), BSP_ADDR)
        sent += 1
        if sent % 250 == 0:
            print(f"inject_fb: sent={sent} cur_fn~={cur} target_fn={target_fn}",
                  flush=True)
        time.sleep(SEND_PERIOD_S)

    print(f"inject_fb: done sent={sent}", flush=True)

if __name__ == "__main__":
    main()
