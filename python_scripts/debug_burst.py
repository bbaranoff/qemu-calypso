#!/usr/bin/env python3
"""
Debug tool for Calypso QEMU burst format.
Connects to QEMU monitor + PTY to inspect and inject test bursts.

Usage: debug_burst.py [--pty /dev/pts/X] [--mon /tmp/qemu-calypso-mon.sock]
"""
import sys, os, struct, time, socket, select, termios, fcntl

# Sercomm constants
FLAG = 0x7E
ESC  = 0x7D
DLCI_BURST = 4
DLCI_L1CTL = 5

def sercomm_wrap(dlci, payload):
    """Wrap payload in sercomm frame: FLAG DLCI CTRL payload FLAG"""
    frame = bytearray([FLAG])
    for b in bytes([dlci, 0x03]) + payload:
        if b in (FLAG, ESC):
            frame.append(ESC)
            frame.append(b ^ 0x20)
        else:
            frame.append(b)
    frame.append(FLAG)
    return bytes(frame)

def sercomm_parse(data):
    """Parse sercomm frames from raw bytes. Yields (dlci, payload)."""
    state = 0  # 0=idle, 1=in_frame, 2=escape
    buf = bytearray()
    for b in data:
        if state == 0:
            if b == FLAG: state = 1; buf = bytearray()
        elif state == 2:
            buf.append(b ^ 0x20); state = 1
        else:
            if b == FLAG:
                if len(buf) >= 2:
                    yield buf[0], bytes(buf[2:])
                buf = bytearray()
            elif b == ESC:
                state = 2
            else:
                buf.append(b)

def make_trxd_burst(tn=0, fn=0, rssi=-60, toa=0, bits=None):
    """Create a TRXD v0 DL burst: TN(1) FN(4) RSSI(1) TOA(2) bits(148)
    This is what osmo-bts-trx sends and what the firmware expects via sercomm."""
    if bits is None:
        # Frequency correction burst (FCCH): all zeros
        bits = bytes(148)
    pkt = bytearray()
    pkt.append(tn & 0x07)
    pkt.extend(struct.pack(">I", fn))  # FN big-endian
    pkt.append(rssi & 0xFF)            # RSSI (signed)
    pkt.extend(struct.pack(">h", toa)) # TOA big-endian
    pkt.extend(bits[:148])
    return bytes(pkt)

def make_fb_burst(tn=0, fn=0):
    """Frequency Burst (FB/FCCH) — all zero bits, strong signal."""
    bits = bytes(148)  # FCCH = all zeros
    return make_trxd_burst(tn=tn, fn=fn, rssi=-40, toa=0, bits=bits)

def make_sb_burst(tn=0, fn=0):
    """Synchronization Burst (SB) — known training sequence."""
    # SB has a specific bit pattern; for now use dummy
    bits = bytearray(148)
    # SB training sequence (bits 42-78)
    ts = [1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,0,0,1,0,1,0,1]
    for i, b in enumerate(ts):
        if 42 + i < 148:
            bits[42 + i] = b
    return make_trxd_burst(tn=tn, fn=fn, rssi=-50, toa=0, bits=bytes(bits))

def qemu_monitor_cmd(sock_path, cmd):
    """Send command to QEMU monitor, return response."""
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(sock_path)
    s.settimeout(2)
    # Read banner
    try: s.recv(4096)
    except: pass
    time.sleep(0.1)
    s.send((cmd + "\n").encode())
    time.sleep(0.3)
    try:
        resp = s.recv(8192).decode(errors='replace')
    except:
        resp = ""
    s.close()
    return resp

def read_pty_nonblock(fd, timeout=0.5):
    """Read available bytes from PTY with timeout."""
    r, _, _ = select.select([fd], [], [], timeout)
    if fd in r:
        try:
            return os.read(fd, 4096)
        except OSError:
            return b""
    return b""

def open_pty(path):
    """Open PTY in raw non-blocking mode."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[2] &= ~termios.CSIZE; attrs[2] |= termios.CS8
    attrs[3] = 0  # lflag
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    fcntl.fcntl(fd, fcntl.F_SETFL, fcntl.fcntl(fd, fcntl.F_GETFL) | os.O_NONBLOCK)
    return fd

def dump_state(mon_sock):
    """Dump relevant QEMU state via monitor."""
    print("=== QEMU State ===")
    regs = qemu_monitor_cmd(mon_sock, "info registers")
    for line in regs.split('\n'):
        if 'R' in line and '=' in line:
            print(line.strip())

    # Read UART modem registers
    print("\n=== UART Modem (0xFFFF5800) ===")
    for name, off in [("IER", 1), ("IIR", 2), ("LSR", 5), ("MCR", 4), ("MSR", 6)]:
        resp = qemu_monitor_cmd(mon_sock, f"xp /1xb 0xffff58{off:02x}")
        for line in resp.split('\n'):
            if '0x' in line and ':' in line:
                val = line.split(':')[-1].strip()
                print(f"  {name} (off {off}): {val}")

def inject_burst(pty_fd, tn=0, fn=51, burst_type='fb'):
    """Inject a single burst via PTY in sercomm DLCI 4 format."""
    if burst_type == 'fb':
        pkt = make_fb_burst(tn=tn, fn=fn)
    elif burst_type == 'sb':
        pkt = make_sb_burst(tn=tn, fn=fn)
    else:
        pkt = make_trxd_burst(tn=tn, fn=fn)

    frame = sercomm_wrap(DLCI_BURST, pkt)
    print(f"Injecting {burst_type} burst: TN={tn} FN={fn} len={len(pkt)} sercomm={len(frame)} bytes")
    print(f"  Raw: {frame[:40].hex()}...")
    os.write(pty_fd, frame)
    return frame

def sniff_tx(pty_fd, duration=2.0):
    """Sniff firmware TX for sercomm frames."""
    print(f"\nSniffing firmware TX for {duration}s...")
    end = time.time() + duration
    raw = bytearray()
    while time.time() < end:
        data = read_pty_nonblock(pty_fd, 0.1)
        if data:
            raw.extend(data)

    print(f"  Received {len(raw)} bytes")
    for dlci, payload in sercomm_parse(raw):
        if dlci == DLCI_L1CTL:
            msg_type = payload[0] if payload else 0
            print(f"  L1CTL: type=0x{msg_type:02x} len={len(payload)} [{payload[:8].hex()}]")
        elif dlci == DLCI_BURST:
            if len(payload) >= 6:
                tn = payload[0] & 0x07
                fn = struct.unpack(">I", payload[1:5])[0]
                print(f"  UL Burst: TN={tn} FN={fn} len={len(payload)}")
        else:
            print(f"  DLCI {dlci}: len={len(payload)} [{payload[:16].hex()}]")

def main():
    import argparse
    p = argparse.ArgumentParser(description="Calypso QEMU burst debugger")
    p.add_argument("--pty", default="/tmp/qemu-uart-modem", help="UART modem PTY path")
    p.add_argument("--mon", default="/tmp/qemu-calypso-mon.sock", help="QEMU monitor socket")
    p.add_argument("--inject", action="store_true", help="Inject test FB burst")
    p.add_argument("--sniff", action="store_true", help="Sniff firmware TX")
    p.add_argument("--state", action="store_true", help="Dump QEMU state")
    p.add_argument("--tn", type=int, default=0, help="Timeslot number")
    p.add_argument("--fn", type=int, default=51, help="Frame number")
    p.add_argument("--type", default="fb", choices=["fb", "sb", "raw"], help="Burst type")
    p.add_argument("--count", type=int, default=1, help="Number of bursts to inject")
    p.add_argument("--all", action="store_true", help="Run all: state + inject + sniff")
    args = p.parse_args()

    if args.state or args.all:
        dump_state(args.mon)

    if args.inject or args.all:
        pty_path = os.path.realpath(args.pty)
        print(f"\nPTY: {pty_path}")
        fd = open_pty(pty_path)
        for i in range(args.count):
            inject_burst(fd, tn=args.tn, fn=args.fn + i, burst_type=args.type)
            time.sleep(0.005)  # 5ms between bursts

        if args.sniff or args.all:
            sniff_tx(fd, duration=3.0)
        os.close(fd)
    elif args.sniff:
        pty_path = os.path.realpath(args.pty)
        fd = open_pty(pty_path)
        sniff_tx(fd, duration=5.0)
        os.close(fd)

    if not any([args.state, args.inject, args.sniff, args.all]):
        p.print_help()
        print("\nExamples:")
        print("  python3 debug_burst.py --state")
        print("  python3 debug_burst.py --inject --tn 0 --fn 51 --type fb")
        print("  python3 debug_burst.py --sniff")
        print("  python3 debug_burst.py --all --count 5")

if __name__ == "__main__":
    main()
