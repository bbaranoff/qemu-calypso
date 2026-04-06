#!/usr/bin/env python3
"""
inject_cfile.py — feed a raw RTL-SDR cfile (float32 complex) directly into
the QEMU sercomm_gate as TRXDv0 burst packets.

Usage:
    ./inject_cfile.py /root/out_arfcn_100.cfile [--rate 1024000] [--gate 127.0.0.1:6702]

Defaults: rate=1024000 (RTL-SDR ~1 Msps), gate=127.0.0.1:6702.

The cfile is streamed sequentially. Each burst packet carries
148 symbols × 4 sps = 592 complex samples (= 1184 int16 LE) plus a
6-byte TRXDv0 header. Frame numbers are incremented per packet on TN=0
so the gate / BSP DMA / FB-det all see a steady stream of "TS0 bursts".
TDMA framing is intentionally ignored — we just want to know whether the
DSP can detect FB anywhere in the recording.

If --rate ≠ 1083333 (the GSM 4-sps rate) the script resamples linearly
(scipy.signal.resample) so each burst still represents 148 GSM symbols.
"""
import argparse, os, socket, struct, sys, time
import numpy as np

BURST_SAMPLES_OUT = 592       # 148 syms × 4 sps
GSM_RATE_OUT      = 1083333   # 4 sps × 270.833 kHz
SCALE             = 16384     # float [-1,1] → int16 [-16384,16384]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cfile", nargs="?", help="path to float32 complex IQ file (omit with --tone)")
    ap.add_argument("--rate", type=int, default=1024000,
                    help="sample rate of the cfile (default 1.024 Msps)")
    ap.add_argument("--gate", default="127.0.0.1:6702",
                    help="UDP host:port of QEMU sercomm_gate (default 127.0.0.1:6702)")
    ap.add_argument("--burst-rate", type=float, default=1733.0,
                    help="bursts/sec to send (default 1733 ≈ 8 bursts/TDMA frame)")
    ap.add_argument("--loop", action="store_true", help="restart at EOF")
    ap.add_argument("--tone", action="store_true",
                    help="ignore cfile, synthesize a pure +67.7 kHz tone "
                         "(perfect FCCH stub) at 4 sps GSM rate")
    args = ap.parse_args()

    host, port = args.gate.split(":")
    dst = (host, int(port))

    if args.tone:
        # Pure FCCH: complex exponential at +67.7083 kHz (= GSM symbol rate / 4)
        # at 4-sps GSM rate (1083333 Hz). One cycle every 16 samples.
        # Continuous phase across bursts (no discontinuities).
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        period = 1.0 / args.burst_rate
        fn = 0
        sent = 0
        phase_step = 2 * np.pi * 67708.33 / GSM_RATE_OUT
        n0 = 0
        t_next = time.monotonic()
        print(f"TONE mode: +67.7 kHz @ {GSM_RATE_OUT} Hz, phase step {phase_step:.4f} rad/sample")
        print(f"gate : {dst}")
        while True:
            n = np.arange(n0, n0 + BURST_SAMPLES_OUT)
            iq = np.exp(1j * phase_step * n).astype(np.complex64)
            n0 += BURST_SAMPLES_OUT
            iq_int16 = np.empty(2 * BURST_SAMPLES_OUT, dtype=np.int16)
            iq_int16[0::2] = np.clip(iq.real * SCALE, -32768, 32767).astype(np.int16)
            iq_int16[1::2] = np.clip(iq.imag * SCALE, -32768, 32767).astype(np.int16)
            hdr = struct.pack(">BIB", 0, fn & 0xFFFFFFFF, 0)
            sock.sendto(hdr + iq_int16.tobytes(), dst)
            sent += 1; fn += 1
            t_next += period
            dt = t_next - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            elif dt < -1.0:
                t_next = time.monotonic()
            if sent % 1000 == 0:
                print(f"  tone: sent {sent:,} bursts (fn={fn})", flush=True)

    fsize = os.path.getsize(args.cfile)
    nsamp = fsize // 8
    print(f"cfile : {args.cfile}  ({nsamp:,} complex samples, {fsize/1e6:.1f} MB)")
    print(f"rate  : {args.rate} Hz   gate : {dst}")

    # Compute how many input samples produce one output burst.
    # ratio = output rate / input rate
    if args.rate == GSM_RATE_OUT:
        in_per_burst = BURST_SAMPLES_OUT
    else:
        in_per_burst = int(round(BURST_SAMPLES_OUT * args.rate / GSM_RATE_OUT))
    print(f"in_samples_per_burst = {in_per_burst}  (resample → {BURST_SAMPLES_OUT})")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    chunk_bytes = in_per_burst * 8  # 8 = sizeof(complex64)
    period = 1.0 / args.burst_rate
    fn = 0
    t_next = time.monotonic()
    sent = 0

    with open(args.cfile, "rb") as f:
        while True:
            raw = f.read(chunk_bytes)
            if len(raw) < chunk_bytes:
                if args.loop:
                    f.seek(0); fn = 0; continue
                break
            iq = np.frombuffer(raw, dtype=np.complex64)
            if in_per_burst != BURST_SAMPLES_OUT:
                # cheap linear interpolation
                idx = np.linspace(0, len(iq) - 1, BURST_SAMPLES_OUT)
                iq_out = np.interp(idx, np.arange(len(iq)), iq.real) + \
                         1j * np.interp(idx, np.arange(len(iq)), iq.imag)
            else:
                iq_out = iq
            iq_int16 = np.empty(2 * BURST_SAMPLES_OUT, dtype=np.int16)
            iq_int16[0::2] = np.clip(iq_out.real * SCALE, -32768, 32767).astype(np.int16)
            iq_int16[1::2] = np.clip(iq_out.imag * SCALE, -32768, 32767).astype(np.int16)

            # TRXDv0 header: TN(1) FN(4 BE) ATT(1)
            tn  = 0
            hdr = struct.pack(">BIB", tn, fn & 0xFFFFFFFF, 0)
            pkt = hdr + iq_int16.tobytes()
            sock.sendto(pkt, dst)
            sent += 1
            fn   += 1

            # Pace the stream.
            t_next += period
            dt = t_next - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            elif dt < -1.0:
                t_next = time.monotonic()  # we drifted too far behind, resync

            if sent % 1000 == 0:
                print(f"  sent {sent:,} bursts (fn={fn})", flush=True)

    print(f"done — sent {sent:,} bursts")


if __name__ == "__main__":
    main()
