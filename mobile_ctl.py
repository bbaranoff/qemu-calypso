#!/usr/bin/env python3
"""
mobile_ctl.py - Telecommande mobile osmocom-bb via VTY telnet :4247.

Couvre les "calls / lua-like" : sequence de commandes a executer comme
si tu tapais au prompt mobile VTY.

Usage CLI :
  ./mobile_ctl.py --list-cells
  ./mobile_ctl.py --call 0123456789
  ./mobile_ctl.py --network-search
  ./mobile_ctl.py --power on
  ./mobile_ctl.py --sms 0123456789 "hello"
  ./mobile_ctl.py --exec "show ms ms1, show network, show subscriber"
  ./mobile_ctl.py --script /tmp/scenario.txt
  ./mobile_ctl.py --interactive       # raw VTY shell

Usage module :
  import mobile_ctl as mc
  with mc.session() as v:
      v.cmd("show ms ms1")
      v.cmd("network search")
      print(v.cmd("show subscriber"))

Le VTY est expose par le mobile osmocom-bb. La cfg mobile_group1.cfg :
  line vty
   no login
   bind 127.0.0.1 4247
"""

from __future__ import annotations
import argparse
import socket
import sys
import time
from contextlib import contextmanager
from typing import Optional


VTY_HOST = "127.0.0.1"
VTY_PORT = 4247

# Prompt regex (mobile VTY prompts) :
#   "OsmocomBB> "   = view mode
#   "OsmocomBB# "   = enable mode
PROMPT_TAILS = (b"> ", b"# ")


class VtyError(Exception):
    pass


class Vty:
    def __init__(self, host: str = VTY_HOST, port: int = VTY_PORT,
                 timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self.buf = b""

    def connect(self):
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.settimeout(self.timeout)
        self.sock = s
        # Read welcome / initial prompt
        self._read_until_prompt()

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _read_until_prompt(self, timeout: Optional[float] = None) -> bytes:
        assert self.sock
        deadline = time.time() + (timeout if timeout is not None else self.timeout)
        out = bytearray()
        while time.time() < deadline:
            try:
                self.sock.settimeout(max(0.1, deadline - time.time()))
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                out.extend(chunk)
                if any(out.endswith(p) for p in PROMPT_TAILS):
                    return bytes(out)
            except socket.timeout:
                break
        return bytes(out)

    def cmd(self, command: str, timeout: Optional[float] = None) -> str:
        """Send a VTY command, return the response (string, prompt stripped)."""
        assert self.sock, "not connected"
        self.sock.sendall(command.encode() + b"\r\n")
        raw = self._read_until_prompt(timeout)
        text = raw.decode(errors="replace")
        # strip echo of the command (first line)
        lines = text.split("\n")
        if lines and command in lines[0]:
            lines = lines[1:]
        # strip trailing prompt
        if lines and (lines[-1].endswith("> ") or lines[-1].endswith("# ")):
            lines = lines[:-1]
        return "\n".join(lines).rstrip()

    def enable(self):
        """Drop into enable mode for privileged commands."""
        return self.cmd("enable")


@contextmanager
def session(host: str = VTY_HOST, port: int = VTY_PORT, timeout: float = 5.0):
    v = Vty(host, port, timeout)
    v.connect()
    try:
        yield v
    finally:
        v.close()


# ---- High-level helpers ----

def power_on(v: Vty, ms: str = "1") -> str:
    """Power on mobile MS instance."""
    v.enable()
    return v.cmd(f"power-on {ms}")

def power_off(v: Vty, ms: str = "1") -> str:
    v.enable()
    return v.cmd(f"power-off {ms}")

def network_search(v: Vty, ms: str = "1") -> str:
    v.enable()
    return v.cmd(f"network search {ms}")

def network_select(v: Vty, ms: str, mcc: int, mnc: int) -> str:
    v.enable()
    return v.cmd(f"network select {ms} {mcc} {mnc}")

def make_call(v: Vty, number: str, ms: str = "1") -> str:
    """Place a call to a phone number."""
    v.enable()
    return v.cmd(f"call {ms} {number}")

def hangup(v: Vty, ms: str = "1") -> str:
    v.enable()
    return v.cmd(f"call {ms} hangup")

def send_sms(v: Vty, dest: str, text: str, ms: str = "1") -> str:
    """Send SMS to dest."""
    v.enable()
    return v.cmd(f"sms {ms} {dest} {text}")

def show_ms(v: Vty, ms: str = "1") -> str:
    return v.cmd(f"show ms {ms}")

def show_subscriber(v: Vty, ms: str = "1") -> str:
    return v.cmd(f"show subscriber {ms}")

def show_cell(v: Vty, ms: str = "1") -> str:
    return v.cmd(f"show cell {ms}")

def show_pld(v: Vty, ms: str = "1") -> str:
    """show ms state (cell/PLMN/RR/MM)"""
    return v.cmd(f"show ms {ms}")


# ---- Lua-like batch executor ----

def exec_batch(v: Vty, commands: list[str], delay_ms: int = 100,
               verbose: bool = False) -> list[tuple[str, str]]:
    """Execute a list of VTY commands sequentially, return [(cmd, response)]."""
    results = []
    for c in commands:
        c = c.strip()
        if not c or c.startswith("#"):
            continue
        # Special directives :
        #   sleep N  - sleep N ms
        if c.startswith("sleep "):
            ms = int(c.split()[1])
            time.sleep(ms / 1000)
            results.append((c, f"(slept {ms}ms)"))
            continue
        resp = v.cmd(c)
        results.append((c, resp))
        if verbose:
            print(f"  > {c}")
            for line in resp.splitlines():
                print(f"    {line}")
        time.sleep(delay_ms / 1000)
    return results


def exec_script(v: Vty, script_path: str, delay_ms: int = 100,
                verbose: bool = False) -> list[tuple[str, str]]:
    with open(script_path) as f:
        commands = f.read().splitlines()
    return exec_batch(v, commands, delay_ms, verbose)


# ---- Interactive shell ----

def interactive(host: str = VTY_HOST, port: int = VTY_PORT):
    """Raw VTY shell - bidirectionnel."""
    import select
    s = socket.create_connection((host, port), timeout=5.0)
    print(f"[mobile_ctl] connected {host}:{port}  (Ctrl-D to exit)")
    s.setblocking(False)
    try:
        while True:
            r, _, _ = select.select([s, sys.stdin], [], [], 0.1)
            if s in r:
                try:
                    data = s.recv(4096)
                    if not data:
                        print("\n[mobile_ctl] connection closed by remote")
                        break
                    sys.stdout.write(data.decode(errors="replace"))
                    sys.stdout.flush()
                except BlockingIOError:
                    pass
            if sys.stdin in r:
                line = sys.stdin.readline()
                if not line:
                    break
                s.sendall(line.encode())
    finally:
        s.close()


# ---- CLI ----

def main():
    ap = argparse.ArgumentParser(description="Mobile osmocom-bb VTY remote control")
    ap.add_argument("--host", default=VTY_HOST)
    ap.add_argument("--port", type=int, default=VTY_PORT)
    ap.add_argument("--ms", default="1", help="MS instance name (cfg : 'ms 1' -> '1')")
    ap.add_argument("--timeout", type=float, default=5.0)

    g = ap.add_mutually_exclusive_group()
    g.add_argument("--power", choices=["on", "off"], help="power on/off mobile")
    g.add_argument("--network-search", action="store_true")
    g.add_argument("--network-select", nargs=2, metavar=("MCC", "MNC"))
    g.add_argument("--call", metavar="NUMBER", help="place call to NUMBER")
    g.add_argument("--hangup", action="store_true")
    g.add_argument("--sms", nargs=2, metavar=("DEST", "TEXT"))
    g.add_argument("--show", choices=["ms", "subscriber", "cell"])
    g.add_argument("--exec", metavar="CMDS", help="comma-sep VTY commands")
    g.add_argument("--script", metavar="PATH", help="exec script of commands")
    g.add_argument("--interactive", action="store_true", help="raw VTY shell")
    g.add_argument("--probe", action="store_true",
                   help="quick probe : connect + show ms + close")

    ap.add_argument("--delay-ms", type=int, default=100,
                    help="ms entre cmds en batch/script")
    ap.add_argument("--verbose", "-v", action="store_true")
    args = ap.parse_args()

    if args.interactive:
        interactive(args.host, args.port)
        return

    try:
        with session(args.host, args.port, args.timeout) as v:
            if args.probe:
                print(show_ms(v, args.ms))
            elif args.power:
                print(power_on(v, args.ms) if args.power == "on" else power_off(v, args.ms))
            elif args.network_search:
                print(network_search(v, args.ms))
            elif args.network_select:
                mcc, mnc = args.network_select
                print(network_select(v, args.ms, int(mcc), int(mnc)))
            elif args.call:
                print(make_call(v, args.call, args.ms))
            elif args.hangup:
                print(hangup(v, args.ms))
            elif args.sms:
                dest, text = args.sms
                print(send_sms(v, dest, text, args.ms))
            elif args.show:
                if args.show == "ms":         print(show_ms(v, args.ms))
                elif args.show == "subscriber": print(show_subscriber(v, args.ms))
                elif args.show == "cell":     print(show_cell(v, args.ms))
            elif args.exec:
                cmds = [c.strip() for c in args.exec.split(",")]
                exec_batch(v, cmds, args.delay_ms, verbose=True)
            elif args.script:
                exec_script(v, args.script, args.delay_ms, verbose=True)
            else:
                ap.print_help()
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"[mobile_ctl] ERROR : cannot connect to {args.host}:{args.port} - {e}", file=sys.stderr)
        print("[mobile_ctl] check: mobile cfg has 'line vty / bind 127.0.0.1 4247'", file=sys.stderr)
        sys.exit(1)
    except VtyError as e:
        print(f"[mobile_ctl] VTY error : {e}", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
