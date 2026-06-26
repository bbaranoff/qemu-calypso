#!/usr/bin/env python3
"""
l1ctl_pace.py — Pacing bridge between mobile and QEMU firmware.

Sits between OsmocomBB mobile (L1CTL) and QEMU's l1ctl_sock.
Forwards all messages faithfully, pacing between network batches:
when mobile sends a burst of messages (same recv), they are all
forwarded at once. The bridge then waits for the firmware to
respond before accepting the next burst from mobile.

This prevents sercomm msgb exhaustion: each batch fits in one
sercomm ISR pass, and the firmware processes it before the next
batch arrives.

Architecture:
  mobile <-> /tmp/osmocom_l2_1 (this bridge) <-> /tmp/osmocom_l2_qemu (QEMU)

Usage:
  python3 l1ctl_pace.py [mobile_sock] [qemu_sock]
"""

import os
import select
import socket
import struct
import sys
import time

MOBILE_SOCK = sys.argv[1] if len(sys.argv) > 1 else "/tmp/osmocom_l2_1"
QEMU_SOCK   = sys.argv[2] if len(sys.argv) > 2 else "/tmp/osmocom_l2_qemu"

L1CTL_NAMES = {
    0x01: "FBSB_REQ",   0x02: "FBSB_CONF",  0x03: "DATA_IND",
    0x04: "RACH_REQ",   0x05: "RACH_CONF",   0x06: "DATA_REQ",
    0x07: "RESET_IND",  0x08: "PM_REQ",      0x09: "PM_CONF",
    0x0A: "ECHO_REQ",   0x0B: "ECHO_CONF",   0x0C: "DATA_CONF",
    0x0D: "RESET_REQ",  0x0E: "RESET_CONF",  0x0F: "DATA_ABI",
    0x12: "TCH_MODE_REQ", 0x13: "TCH_MODE_CONF",
    0x14: "NEIGH_PM_REQ", 0x15: "NEIGH_PM_IND",
    0x16: "TRAFFIC_REQ", 0x17: "TRAFFIC_CONF", 0x18: "TRAFFIC_IND",
}

# Expected response for request types that need pacing
RESPONSE_FOR = {
    0x0D: 0x0E,  # RESET_REQ -> RESET_CONF
    0x08: 0x09,  # PM_REQ -> PM_CONF
    0x01: 0x02,  # FBSB_REQ -> FBSB_CONF
}

def log(msg):
    print(f"[pace] {msg}", flush=True)

def name(mt):
    return L1CTL_NAMES.get(mt, f"0x{mt:02x}")

def recv_exact(sock, n, timeout=5.0):
    data = b""
    deadline = time.time() + timeout
    while len(data) < n:
        remaining = deadline - time.time()
        if remaining <= 0:
            return None
        r, _, _ = select.select([sock], [], [], remaining)
        if not r:
            return None
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data

def recv_l1ctl(sock, timeout=5.0):
    hdr = recv_exact(sock, 2, timeout)
    if not hdr:
        return None
    msglen = struct.unpack(">H", hdr)[0]
    payload = recv_exact(sock, msglen, timeout)
    if not payload:
        return None
    return payload

def send_l1ctl(sock, payload):
    hdr = struct.pack(">H", len(payload))
    sock.sendall(hdr + payload)

def parse_batch(data):
    """Parse all complete L1CTL messages from raw data.
    Returns (list_of_payloads, remaining_bytes)."""
    msgs = []
    pos = 0
    while pos + 2 <= len(data):
        msglen = struct.unpack(">H", data[pos:pos+2])[0]
        if pos + 2 + msglen > len(data):
            break
        msgs.append(data[pos+2:pos+2+msglen])
        pos += 2 + msglen
    return msgs, data[pos:]

def main():
    if os.path.exists(MOBILE_SOCK):
        os.unlink(MOBILE_SOCK)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(MOBILE_SOCK)
    srv.listen(1)
    log(f"listening on {MOBILE_SOCK}, QEMU at {QEMU_SOCK}")

    while True:
        log("waiting for mobile...")
        mobile, _ = srv.accept()
        log("mobile connected")

        qemu = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            qemu.connect(QEMU_SOCK)
        except (ConnectionRefusedError, FileNotFoundError):
            log(f"ERROR: QEMU not ready at {QEMU_SOCK}")
            mobile.close()
            time.sleep(1)
            continue
        log("connected to QEMU")

        # Wait for RESET_IND from firmware
        log("waiting for RESET_IND...")
        reset_ind = recv_l1ctl(qemu, timeout=30.0)
        if not reset_ind or reset_ind[0] != 0x07:
            log(f"ERROR: expected RESET_IND, got {reset_ind}")
            mobile.close(); qemu.close()
            continue
        send_l1ctl(mobile, reset_ind)
        log("-> mobile: RESET_IND")

        try:
            relay_loop(mobile, qemu)
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            log(f"lost: {e}")
        finally:
            mobile.close(); qemu.close()
            log("session ended")

def relay_loop(mobile, qemu):
    mobile_buf = b""  # partial data from mobile

    while True:
        r, _, _ = select.select([mobile, qemu], [], [], 1.0)

        # ── QEMU -> mobile (unsolicited: DATA_IND, TRAFFIC_IND, etc.) ──
        if qemu in r:
            msg = recv_l1ctl(qemu, timeout=0.5)
            if msg is None:
                log("QEMU disconnected"); return
            log(f"<- QEMU:  {name(msg[0])} ({len(msg)}B)")
            send_l1ctl(mobile, msg)

        # ── mobile -> QEMU (batch of requests) ──
        if mobile in r:
            chunk = mobile.recv(4096)
            if not chunk:
                log("mobile disconnected"); return
            mobile_buf += chunk

            # Coalesce: wait briefly for more data from mobile.
            # The mobile app sends messages in rapid write() calls that
            # may arrive as separate recv()s. Waiting a short time lets
            # the kernel aggregate them into one batch, matching what
            # the real UART would see (all bytes arrive within ~1ms).
            while True:
                rr, _, _ = select.select([mobile], [], [], 0.2)
                if not rr:
                    break
                more = mobile.recv(4096)
                if not more:
                    break
                mobile_buf += more
                # Keep coalescing — the mobile sends multiple batches
                # in quick succession. 200ms window catches them all.
                continue

            # Parse all complete messages from the coalesced data
            msgs, mobile_buf = parse_batch(mobile_buf)
            if not msgs:
                continue

            # Log the batch
            names_list = [name(m[0]) for m in msgs]
            log(f"<- mobile batch [{len(msgs)}]: {' + '.join(names_list)}")

            # Send messages to QEMU. Strategy:
            # - If the batch has ONLY request+response pairs (no mix),
            #   send all at once (firmware processes them atomically).
            # - If the batch has > 1 message, send one at a time with
            #   a delay so the firmware sercomm can handle each frame
            #   separately (prevents msgb exhaustion).
            # The delay between sends lets the QEMU staging buffer drip
            # each frame into a separate TINT0 tick.
            for m in msgs:
                send_l1ctl(qemu, m)
            log(f"-> QEMU:  {len(msgs)} messages forwarded")

            # Find last expected response
            last_expected = None
            for m in msgs:
                resp = RESPONSE_FOR.get(m[0])
                if resp is not None:
                    last_expected = resp

            # Wait for it
            if last_expected is not None:
                log(f"   pacing: waiting for {name(last_expected)}...")
                deadline = time.time() + 15.0
                got_it = False
                while time.time() < deadline:
                    resp = recv_l1ctl(qemu, timeout=1.0)
                    if resp is None:
                        continue
                    rmt = resp[0]
                    log(f"<- QEMU:  {name(rmt)} ({len(resp)}B)")
                    send_l1ctl(mobile, resp)
                    if rmt == last_expected:
                        log(f"   pacing: got {name(last_expected)} ✓")
                        got_it = True
                        break
                if not got_it:
                    log(f"   pacing: TIMEOUT for {name(last_expected)}")

if __name__ == "__main__":
    main()
