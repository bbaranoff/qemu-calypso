#!/usr/bin/env python3
"""
gsm_clock.py — Master GSM TDMA clock for the Calypso QEMU stack.

Single source of truth for frame number (FN). Sends air bursts to both
QEMU instances (BTS TRX + Mobile layer1) at GSM cadence so they stay
synchronized.  Every component derives timing from this clock.

Architecture:
  gsm_clock.py  --UDP air burst-->  QEMU TRX  (port 4800)
                --UDP air burst-->  QEMU MS   (port 4801)
                --UDP air burst-->  bridge    (port 4901)

Burst format (same as QEMU air interface):
  byte 0      : TN (timeslot)
  bytes 1..4  : FN (big-endian uint32)
  bytes 5..152: 148 hard bits (0/1)

Usage:
  python3 gsm_clock.py [-r RATE] [--trx-port 4800] [--ms-port 4801] [--bridge-port 4901]

Rate: 1.0 = real GSM (4.615ms/frame), 0.1 = 10x slower (46.15ms, matches QEMU default)
"""

import argparse
import socket
import struct
import time
import signal
import sys

GSM_HYPERFRAME = 2715648
GSM_FRAME_US = 4615  # 4.615 ms in microseconds

# 51-multiframe structure for TS0 (combined CCCH+SDCCH/4)
# FCCH: 0,10,20,30,40  SCH: 1,11,21,31,41  BCCH: 2-5  CCCH: 6-9,12-15,16-19,...
FCCH_FN51 = {0, 10, 20, 30, 40}
SCH_FN51  = {1, 11, 21, 31, 41}

# FCCH burst: all zeros (frequency correction)
FCCH_BURST = bytes(148)

# SCH training sequence (midamble bits 42..105)
SCH_TRAIN = [
    1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,
    0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1,
]

# Normal burst fill pattern (idle CCCH = 2B padding)
IDLE_L2 = bytes([0x03, 0x03, 0x01, 0x2B] + [0x2B] * 19)  # 23 bytes


def make_sch_burst(bsic, fn):
    """Build a 148-bit SCH burst with encoded BSIC+FN."""
    t1 = fn // (26 * 51)
    t2 = fn % 26
    t3 = fn % 51
    t3p = (t3 - 1) // 10 if t3 > 0 else 0

    # 25 info bits: spare(2) + BSIC(6) + T1(11) + T2(5) + T3'(3)
    info = [0, 0]
    for i in range(5, -1, -1):
        info.append((bsic >> i) & 1)
    for i in range(10, -1, -1):
        info.append((t1 >> i) & 1)
    for i in range(4, -1, -1):
        info.append((t2 >> i) & 1)
    for i in range(2, -1, -1):
        info.append((t3p >> i) & 1)

    # CRC (10 bits) + tail (4 bits) — simplified: zeros
    full = info + [0] * 10 + [0] * 4  # 39 bits

    # Convolutional encode (rate 1/2, K=5)
    reg = 0
    coded = []
    for b in full:
        reg = ((reg << 1) | b) & 0x1F
        g0 = ((reg >> 0) ^ (reg >> 3) ^ (reg >> 4)) & 1
        g1 = ((reg >> 0) ^ (reg >> 1) ^ (reg >> 3) ^ (reg >> 4)) & 1
        coded.extend([g0, g1])
    # 78 coded bits → split into 2x39

    # SCH burst: 3 tail + 39 coded + 64 train + 39 coded + 3 tail + guard
    burst = [0]*3 + coded[:39] + SCH_TRAIN + coded[39:78] + [0]*3
    burst = (burst + [0]*148)[:148]
    return bytes(burst)


def make_normal_burst(data_bits_114, tsc=0):
    """Build a 148-bit normal burst from 114 coded data bits."""
    # TSC midambles
    TSC_BITS = [
        [0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1],
        [0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1],
        [0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0],
        [0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0],
        [0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1],
        [0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0],
        [1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1],
        [1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0],
    ]
    d = list(data_bits_114)
    if len(d) < 114:
        d.extend([0] * (114 - len(d)))
    mid = TSC_BITS[tsc % 8]
    burst = [0]*3 + d[:57] + [0] + mid + [0] + d[57:114] + [0]*3 + [0]*8
    return bytes((burst + [0]*148)[:148])


def encode_l2_to_coded(l2_bytes):
    """Encode 23 L2 bytes → 456 coded bits (conv code), return 4x114 interleaved."""
    # 184 info bits
    info = []
    for byte in l2_bytes[:23]:
        for i in range(7, -1, -1):
            info.append((byte >> i) & 1)
    info = (info + [0]*184)[:184]

    # + 40 parity (zeros simplified) + 4 tail
    full = info + [0]*40 + [0]*4  # 228 bits

    # Conv encode
    reg = 0
    coded = []
    for b in full:
        reg = ((reg << 1) | b) & 0x1F
        g0 = ((reg >> 0) ^ (reg >> 3) ^ (reg >> 4)) & 1
        g1 = ((reg >> 0) ^ (reg >> 1) ^ (reg >> 3) ^ (reg >> 4)) & 1
        coded.extend([g0, g1])

    # Interleave into 4 bursts of 114 bits
    bursts = [[0]*114 for _ in range(4)]
    for k in range(456):
        bursts[k % 4][k // 4] = coded[k] if k < len(coded) else 0
    return bursts


class GSMClock:
    def __init__(self, rate, bsic, arfcn, targets):
        self.rate = rate
        self.bsic = bsic
        self.arfcn = arfcn
        self.fn = 0
        self.targets = targets  # list of (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True

        # Pre-encode idle CCCH burst data (4 bursts per block)
        self.idle_bursts_114 = encode_l2_to_coded(IDLE_L2)

    def send_burst(self, tn, fn, burst_bytes):
        """Send air burst to all targets."""
        pkt = struct.pack(">BI", tn, fn) + burst_bytes[:148]
        for target in self.targets:
            try:
                self.sock.sendto(pkt, target)
            except OSError:
                pass

    def run(self):
        frame_ns = GSM_FRAME_US * 1000  # nanoseconds
        if self.rate != 1.0:
            frame_ns = int(frame_ns / self.rate)

        print(f"gsm-clock: BSIC={self.bsic} ARFCN={self.arfcn}", flush=True)
        print(f"gsm-clock: rate={self.rate}x → {frame_ns/1e6:.3f}ms/frame", flush=True)
        print(f"gsm-clock: targets={self.targets}", flush=True)
        print(f"gsm-clock: starting TDMA...", flush=True)

        t_start = time.monotonic_ns()
        frames_sent = 0

        while self.running:
            fn = self.fn
            fn51 = fn % 51

            # TS0: generate appropriate burst based on 51-multiframe position
            if fn51 in FCCH_FN51:
                self.send_burst(0, fn, FCCH_BURST)
            elif fn51 in SCH_FN51:
                sch = make_sch_burst(self.bsic, fn)
                self.send_burst(0, fn, sch)
            else:
                # BCCH (2-5) or CCCH (6-9, 12-15, ...) or idle
                # Determine burst index within 4-burst block
                block_starts = [2, 6, 12, 16, 22, 26, 32, 36, 42, 46]
                burst_idx = None
                for start in block_starts:
                    if start <= fn51 < start + 4:
                        burst_idx = fn51 - start
                        break

                if burst_idx is not None:
                    coded_114 = self.idle_bursts_114[burst_idx]
                    nb = make_normal_burst(coded_114, tsc=self.bsic & 0x7)
                    self.send_burst(0, fn, nb)
                # else: idle frame (50), send nothing

            # Advance FN
            self.fn = (self.fn + 1) % GSM_HYPERFRAME
            frames_sent += 1

            # Log periodically
            if frames_sent <= 5 or frames_sent % 5000 == 0:
                elapsed_s = (time.monotonic_ns() - t_start) / 1e9
                fps = frames_sent / elapsed_s if elapsed_s > 0 else 0
                print(f"gsm-clock: FN={fn} frames={frames_sent} "
                      f"elapsed={elapsed_s:.1f}s fps={fps:.1f}", flush=True)

            # Precise sleep: target absolute time for next frame
            target_ns = t_start + frames_sent * frame_ns
            now_ns = time.monotonic_ns()
            sleep_ns = target_ns - now_ns
            if sleep_ns > 0:
                time.sleep(sleep_ns / 1e9)


def main():
    ap = argparse.ArgumentParser(description="Master GSM TDMA clock")
    ap.add_argument("-r", "--rate", type=float, default=0.1,
                    help="Clock rate (1.0=real GSM, 0.1=10x slow for QEMU, default: 0.1)")
    ap.add_argument("--bsic", type=int, default=63,
                    help="BSIC (default: 63)")
    ap.add_argument("--arfcn", type=int, default=100,
                    help="ARFCN (default: 100)")
    ap.add_argument("--ip", default="127.0.0.1",
                    help="Target IP (default: 127.0.0.1)")
    args = ap.parse_args()

    # Standard port scheme only: 6700 (transceiver MS-side), 6800 (bridge/mobile)
    targets = [
        (args.ip, 6700),  # transceiver MS-side air → CLK IND → 5800
        (args.ip, 6800),  # bridge/mobile DL air
    ]

    clock = GSMClock(args.rate, args.bsic, args.arfcn, targets)

    def stop(_s, _f):
        clock.running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    clock.run()
    print(f"gsm-clock: stopped at FN={clock.fn}", flush=True)


if __name__ == "__main__":
    main()
