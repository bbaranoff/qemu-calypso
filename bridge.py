#!/usr/bin/env python3
"""
bridge.py — BTS TRX UDP bridge for QEMU Calypso

BTS side:  osmo-bts-trx (CLK/TRXC/TRXD on 5700-5702)
QEMU side: BSP receives DL bursts on UDP 6702
           QEMU sends TDMA ticks on UDP 6700 (QEMU is clock master)

The bridge has NO independent timer. It receives FN ticks from QEMU's
TDMA timer and forwards CLK IND to the BTS. This ensures FN coherence
between QEMU firmware, bridge, and BTS.

Usage: bridge.py
"""
import errno, os, select, signal, socket, struct, sys, time

GSM_HYPERFRAME = 2715648
CLK_IND_PERIOD = 102

def udp_bind(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.setblocking(False)
    return s

class Bridge:
    def __init__(self, bts_base=5700):
        # BTS TRX sockets
        self.clk_sock  = udp_bind(bts_base)
        self.trxc_sock = udp_bind(bts_base + 1)
        self.trxd_sock = udp_bind(bts_base + 2)
        self.bts_clk_addr = ("127.0.0.1", bts_base + 100)
        self.trxc_remote = None
        self.trxd_remote = None
        self.powered = False
        self.fn = 0
        self._stop = False
        self.stats = {"clk": 0, "trxc": 0, "dl": 0, "ul": 0, "tick": 0}

        # QEMU CLK tick receiver — QEMU is clock master
        self.qemu_clk_sock = udp_bind(6700)

        print(f"bridge: CLK={bts_base} TRXC={bts_base+1} TRXD={bts_base+2} → BSP@6702", flush=True)
        print(f"bridge: QEMU CLK ticks on UDP 6700 (clock-slave mode)", flush=True)

    def handle_qemu_tick(self):
        """Receive TDMA tick from QEMU — 4 bytes big-endian FN."""
        try:
            data, addr = self.qemu_clk_sock.recvfrom(64)
        except OSError:
            return
        if len(data) < 4:
            return

        fn = int.from_bytes(data[0:4], 'big')
        self.fn = fn % GSM_HYPERFRAME
        self.stats["tick"] += 1

        if self.stats["tick"] <= 3 or self.stats["tick"] % 10000 == 0:
            print(f"bridge: QEMU tick #{self.stats['tick']} FN={self.fn}", flush=True)

        # Forward CLK IND to BTS at every tick from QEMU.
        # QEMU runs slower than real-time, so we can't wait for fn%102==0
        # (that would take ~8s wall-clock and BTS would timeout).
        # The BTS tolerates receiving CLK IND at any FN cadence.
        if True:
            try:
                self.clk_sock.sendto(
                    f"IND CLOCK {self.fn}\0".encode(), self.bts_clk_addr)
                self.stats["clk"] += 1
            except OSError:
                pass

    def handle_trxc(self):
        try: data, addr = self.trxc_sock.recvfrom(256)
        except OSError: return
        if not data: return
        self.trxc_remote = addr
        raw = data.strip(b'\x00').decode(errors='replace')
        if not raw.startswith("CMD "): return
        parts = raw[4:].split(); verb = parts[0]; args = parts[1:]
        self.stats["trxc"] += 1

        if verb == "POWERON":
            self.powered = True; print(f"BTS: POWERON", flush=True)
            rsp = "RSP POWERON 0"
        elif verb == "POWEROFF":
            self.powered = False; rsp = "RSP POWEROFF 0"
        elif verb == "SETFORMAT":
            rsp = f"RSP SETFORMAT 0 {' '.join(args)}"
        elif verb == "NOMTXPOWER":
            rsp = "RSP NOMTXPOWER 0 50"
        elif verb == "MEASURE":
            freq = args[0] if args else "0"
            rsp = f"RSP MEASURE 0 {freq} -60"
        else:
            rsp = f"RSP {verb} 0 {' '.join(args)}".rstrip()

        self.trxc_sock.sendto((rsp + "\0").encode(), addr)

    def handle_trxd_from_bts(self):
        """BTS DL burst → UDP direct to BSP (port 6702 on QEMU side)."""
        try: data, addr = self.trxd_sock.recvfrom(512)
        except OSError: return
        if len(data) < 6: return
        self.trxd_remote = addr
        self.stats["dl"] += 1

        tn = data[0] & 0x07
        fn = int.from_bytes(data[1:5], 'big')
        if self.stats["dl"] <= 5 or self.stats["dl"] % 1000 == 0:
            print(f"bridge: DL burst #{self.stats['dl']} TN={tn} FN={fn} len={len(data)}", flush=True)

        # Forward raw TRXDv0 DL burst via UDP to BSP inside QEMU
        try: self.trxd_sock.sendto(data, ("127.0.0.1", 6702))
        except OSError: pass

    def run(self):
        # No start_clock() — QEMU drives the clock
        running = True
        def shutdown(s, f):
            nonlocal running; running = False
        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)

        while running:
            fds = [self.trxc_sock, self.trxd_sock, self.qemu_clk_sock]
            try:
                readable, _, _ = select.select(fds, [], [], 0.5)
            except (OSError, ValueError) as e:
                print(f"bridge: select error: {e}", flush=True)
                break

            if self.qemu_clk_sock in readable: self.handle_qemu_tick()
            if self.trxc_sock in readable: self.handle_trxc()
            if self.trxd_sock in readable: self.handle_trxd_from_bts()

            if self.stats["dl"] > 0 and self.stats["dl"] % 5000 == 0:
                print(f"bridge: tick={self.stats['tick']} clk={self.stats['clk']} dl={self.stats['dl']} ul={self.stats['ul']}", flush=True)

        self._stop = True
        print(f"bridge: tick={self.stats['tick']} clk={self.stats['clk']} trxc={self.stats['trxc']} dl={self.stats['dl']} ul={self.stats['ul']}", flush=True)

if __name__ == "__main__":
    Bridge().run()
