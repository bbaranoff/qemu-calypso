#!/usr/bin/env python3
"""
air_burst_player.py — Replay bursts from bursts.txt to QEMU air interface.

Sends bursts to the QEMU mobile instance via UDP, simulating the air interface.
Format: TN(1) + FN(4) + 148 hard bits

Loops the burst file continuously, advancing FN coherently.

Usage:
  python3 air_burst_player.py /root/bursts.txt [peer_port] [rate]
"""

import socket
import struct
import sys
import time

def load_bursts(path):
    """Load bursts from file. Format: 'index fn: bits'"""
    bursts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or ':' not in line:
                continue
            parts = line.split(':')
            header = parts[0].strip().split()
            bits_str = parts[1].strip()
            if len(header) < 2 or len(bits_str) < 100:
                continue
            fn = int(header[1])
            # Pad/trim to 148 bits (file has 114, pad with zeros for guard)
            bits = [int(b) for b in bits_str[:148]]
            while len(bits) < 148:
                bits.append(0)
            bursts.append((fn, bits))
    return bursts

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/root/bursts.txt"
    peer_port = int(sys.argv[2]) if len(sys.argv) > 2 else 4801
    rate = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

    bursts = load_bursts(path)
    if not bursts:
        print("No bursts loaded", flush=True)
        sys.exit(1)

    print("Loaded %d bursts from %s" % (len(bursts), path), flush=True)
    print("Sending to 127.0.0.1:%d, rate=%.1fx" % (peer_port, rate), flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer = ("127.0.0.1", peer_port)

    # GSM frame timing: 4.615ms per frame
    frame_interval = 0.004615 / rate

    # Get FN range from file
    fn_start = bursts[0][0]
    fn_end = bursts[-1][0]
    fn_span = fn_end - fn_start
    if fn_span <= 0:
        fn_span = len(bursts)

    loop_count = 0
    total_sent = 0

    print("FN range: %d - %d (span %d)" % (fn_start, fn_end, fn_span), flush=True)
    print("Starting playback...", flush=True)

    try:
        while True:
            fn_offset = loop_count * fn_span
            t_start = time.monotonic()

            for i, (fn_orig, bits) in enumerate(bursts):
                fn = (fn_orig + fn_offset) % 2715648  # GSM hyperframe

                # Build air packet: TN(1) + FN(4) + 148 bits
                pkt = struct.pack(">BI", 0, fn) + bytes(bits)
                sock.sendto(pkt, peer)
                total_sent += 1

                if total_sent <= 5 or total_sent % 500 == 0:
                    print("  [air] #%d FN=%d (loop %d)" % (total_sent, fn, loop_count), flush=True)

                # Sleep to match GSM timing
                # Bursts in file may skip FNs, calculate proper delay
                if i + 1 < len(bursts):
                    fn_next = bursts[i + 1][0]
                    fn_delta = fn_next - fn_orig
                    if fn_delta <= 0:
                        fn_delta = 1
                    delay = frame_interval * fn_delta
                else:
                    delay = frame_interval

                time.sleep(delay)

            loop_count += 1
            elapsed = time.monotonic() - t_start
            print("  [air] Loop %d done (%d bursts in %.1fs)" % (loop_count, len(bursts), elapsed), flush=True)

    except KeyboardInterrupt:
        print("\nStopped. Total sent: %d" % total_sent, flush=True)
    finally:
        sock.close()

if __name__ == "__main__":
    main()
