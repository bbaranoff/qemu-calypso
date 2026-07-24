#!/usr/bin/env python3
"""
l1ctl_test.py — Direct L1CTL test against QEMU firmware
Connects to /tmp/osmocom_l2_1, sends L1CTL commands, prints responses.
"""
import socket, struct, sys, time

L1CTL_RESET_REQ    = 0x0D
L1CTL_RESET_IND    = 0x07
L1CTL_RESET_CONF   = 0x02
L1CTL_PM_REQ       = 0x0C  # actually varies by version
L1CTL_PM_CONF      = 0x0E
L1CTL_FBSB_REQ     = 0x08
L1CTL_FBSB_CONF    = 0x09
L1CTL_CCCH_MODE_REQ = 0x01
L1CTL_DATA_IND     = 0x03

L1CTL_TYPES = {
    0x01: "CCCH_MODE_REQ", 0x02: "RESET_CONF", 0x03: "DATA_IND",
    0x04: "DATA_REQ", 0x05: "RACH_REQ", 0x06: "RACH_CONF",
    0x07: "RESET_IND", 0x08: "FBSB_REQ", 0x09: "FBSB_CONF",
    0x0C: "PM_REQ", 0x0D: "RESET_REQ", 0x0E: "PM_CONF",
    0x0F: "CCCH_MODE_CONF", 0x10: "DM_EST_REQ",
}

def l1ctl_name(t):
    return L1CTL_TYPES.get(t, f"0x{t:02x}")

def send_msg(s, msg_type, payload=b''):
    data = bytes([msg_type, 0, 0, 0]) + payload
    # Pad to at least 4 bytes
    while len(data) < 4:
        data += b'\x00'
    hdr = struct.pack(">H", len(data))
    s.sendall(hdr + data)
    print(f">>> {l1ctl_name(msg_type)} ({len(data)} bytes)")

def recv_msg(s, timeout=5.0):
    s.settimeout(timeout)
    try:
        hdr = s.recv(2)
        if len(hdr) < 2:
            return None, None
        ml = struct.unpack(">H", hdr)[0]
        data = b''
        while len(data) < ml:
            chunk = s.recv(ml - len(data))
            if not chunk:
                return None, None
            data += chunk
        msg_type = data[0]
        print(f"<<< {l1ctl_name(msg_type)} ({ml} bytes): {data[:20].hex()}")
        return msg_type, data
    except socket.timeout:
        return None, None

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/osmocom_l2_1"

    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(path)
    print(f"Connected to {path}")

    # Wait for RESET_IND
    print("\n--- Waiting for RESET_IND ---")
    t, d = recv_msg(s)

    # Send RESET_REQ
    print("\n--- Sending RESET_REQ ---")
    # Payload: 4 bytes header padding + reset_type at byte 4
    # reset_type: 1 = full reset, 2 = scheduler reset
    send_msg(s, L1CTL_RESET_REQ, bytes([1, 0, 0, 0]))
    t, d = recv_msg(s)  # PM_CONF or RESET_CONF

    # Wait for PM_CONF
    print("\n--- Waiting for PM_CONF ---")
    for i in range(10):
        t, d = recv_msg(s, timeout=5.0)
        if t is not None:
            print(f"  GOT: {l1ctl_name(t)}")
            if t == 0x0E:  # PM_CONF
                break

    # Send FBSB_REQ for ARFCN 514
    # Format: flags(1)=1, pad(3), ARFCN(2 BE), timeout(2 BE), rxlev(1), bsic(1), ...
    print("\n--- Sending FBSB_REQ (ARFCN 514) ---")
    arfcn = 514
    fbsb = bytes([
        1, 0, 0, 0,               # flags=1 (new search), padding
        (arfcn >> 8) & 0xFF,       # ARFCN high
        arfcn & 0xFF,              # ARFCN low
        0, 100,                    # timeout = 100 frames
        0xAB,                      # rxlev_exp = -85 (signed)
        0,                         # bsic
        7, 0,                      # ccch_mode, sync_info_idx
    ])
    send_msg(s, L1CTL_FBSB_REQ, fbsb)

    # Wait for responses
    print("\n--- Waiting for responses (120s) ---")
    for i in range(60):
        t, d = recv_msg(s, timeout=2.0)
        if t is None:
            if i % 10 == 0:
                print(f"  (timeout {i})")
        else:
            print(f"  *** {l1ctl_name(t)} : {d[:16].hex()} ***")
            if t == L1CTL_DATA_IND:
                print(f"  !!! DATA_IND !!! payload={d.hex()}")

    s.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
