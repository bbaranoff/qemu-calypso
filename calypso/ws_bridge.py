#!/usr/bin/env python3
"""
ws_bridge.py — WebSocket bridge for Calypso QEMU Monitor

Listens on:
  UDP TRXC  port 6700 (ASCII commands from osmo-bts-trx to QEMU)
  UDP TRXD  port 6701 (binary bursts, sniff only)
  UDP GSMTAP port 4729 (GSMTAP bursts from QEMU)
  WS  port  9999 (HTML frontend)

Relays everything to connected WebSocket clients in real time.

Install deps:
  pip3 install websockets

Usage:
  python3 ws_bridge.py [--trxc-port 6700] [--gsmtap-port 4729] [--ws-port 9999]
"""

import asyncio
import socket
import struct
import json
import argparse
import time
import logging
from collections import deque

try:
    import websockets
except ImportError:
    print("ERROR: pip3 install websockets")
    raise

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(levelname)s %(message)s')
log = logging.getLogger('ws_bridge')

# ---------------------------------------------------------------------------
# Shared state (mirrored from QEMU TRX bridge via sniffing)
# ---------------------------------------------------------------------------

class CalypsoState:
    def __init__(self):
        self.sync      = 'IDLE'
        self.arfcn     = 0
        self.bsic      = 0x3C
        self.rssi      = -62
        self.fn        = 0
        self.trxcPort  = 6700
        self.trxdPort  = 6701
        self.btsConn   = False
        self.powered   = False
        self.txCount   = 0
        self.rxCount   = 0
        self.taskCount = 0
        self.fbCount   = 0
        self.sbCount   = 0
        self.dspBooted = False
        self.dspPage   = 0
        self.tdmaRunning = False

    def to_dict(self):
        return self.__dict__.copy()

state = CalypsoState()
clients = set()

# ---------------------------------------------------------------------------
# Broadcast to all WS clients
# ---------------------------------------------------------------------------

async def broadcast(msg: dict):
    if not clients:
        return
    data = json.dumps(msg)
    dead = set()
    for ws in clients:
        try:
            await ws.send(data)
        except Exception:
            dead.add(ws)
    clients.difference_update(dead)

async def broadcast_state():
    await broadcast({'type': 'state', 'data': state.to_dict()})

# ---------------------------------------------------------------------------
# GSMTAP parser
# ---------------------------------------------------------------------------

GSMTAP_HDR = struct.Struct('!BBBBHbbiHHI')
GSMTAP_HDR_LEN = GSMTAP_HDR.size  # 16 bytes

def parse_gsmtap(data: bytes):
    if len(data) < GSMTAP_HDR_LEN:
        return None
    fields = GSMTAP_HDR.unpack_from(data)
    version, hdr_len, gtype, timeslot, arfcn_raw, signal_dbm, snr_db, fn, sub_type, antenna, res = fields
    uplink = bool(arfcn_raw & 0x8000)
    arfcn  = arfcn_raw & 0x7FFF
    return {
        'timeslot': timeslot,
        'arfcn':    arfcn,
        'uplink':   uplink,
        'rssi':     signal_dbm,
        'fn':       fn,
        'sub_type': sub_type,
    }

# ---------------------------------------------------------------------------
# TRXC command sniffer (UDP)
# ---------------------------------------------------------------------------

class TRXCProtocol(asyncio.DatagramProtocol):
    """Sniff TRXC datagrams between osmo-bts-trx and QEMU calypso_trx."""

    def __init__(self, loop):
        self.loop = loop
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport
        log.info("TRXC sniffer ready")

    def datagram_received(self, data, addr):
        text = data.rstrip(b'\x00').decode('ascii', errors='replace').strip()
        if not text:
            return

        asyncio.ensure_future(self._handle(text, addr), loop=self.loop)

    async def _handle(self, text, addr):
        if text.startswith('CMD '):
            verb = text[4:].split()[0] if len(text) > 4 else ''

            # Update state from TRXC commands
            state.btsConn = True
            if verb == 'POWERON':
                state.powered = True
                state.tdmaRunning = True
            elif verb == 'POWEROFF':
                state.powered = False
            elif verb.startswith('RXTUNE'):
                parts = text.split()
                if len(parts) >= 3:
                    try:
                        khz = int(parts[2])
                        state.arfcn = khz_to_arfcn(khz)
                    except ValueError:
                        pass
            elif verb.startswith('SETTSC'):
                parts = text.split()
                if len(parts) >= 3:
                    try:
                        tsc = int(parts[2])
                        state.bsic = (state.bsic & 0x38) | (tsc & 0x07)
                    except ValueError:
                        pass
            elif verb.startswith('SETBSIC'):
                parts = text.split()
                if len(parts) >= 3:
                    try:
                        state.bsic = int(parts[2]) & 0x3F
                    except ValueError:
                        pass

            await broadcast({'type': 'trxc_cmd', 'text': text})
            await broadcast_state()

        elif text.startswith('RSP '):
            await broadcast({'type': 'trxc_rsp', 'text': text})

    def error_received(self, exc):
        log.error("TRXC error: %s", exc)


# ---------------------------------------------------------------------------
# GSMTAP listener (UDP)
# ---------------------------------------------------------------------------

class GsmtapProtocol(asyncio.DatagramProtocol):
    def __init__(self, loop):
        self.loop = loop

    def connection_made(self, transport):
        log.info("GSMTAP listener ready")

    def datagram_received(self, data, addr):
        parsed = parse_gsmtap(data)
        if not parsed:
            return
        asyncio.ensure_future(self._handle(parsed), loop=self.loop)

    async def _handle(self, p):
        direction = 'UL' if p['uplink'] else 'DL'
        state.fn = p['fn']
        state.rssi = p['rssi']
        if p['arfcn']:
            state.arfcn = p['arfcn']

        if direction == 'UL':
            state.txCount += 1
        else:
            state.rxCount += 1

        # Infer sync state from burst activity
        if state.sync == 'IDLE' and state.txCount > 0:
            state.sync = 'FCCH_SEARCH'
        if state.sync in ('FCCH_SEARCH', 'FCCH_FOUND', 'SCH_SEARCH') and \
                state.txCount > 10:
            state.sync = 'LOCKED'

        await broadcast({
            'type': 'burst',
            'dir':  direction,
            'fn':   p['fn'],
            'tn':   p['timeslot'],
            'rssi': p['rssi'],
        })
        await broadcast_state()

    def error_received(self, exc):
        log.error("GSMTAP error: %s", exc)


# ---------------------------------------------------------------------------
# WebSocket server
# ---------------------------------------------------------------------------

async def ws_handler(websocket, path):
    clients.add(websocket)
    log.info("WS client connected: %s", websocket.remote_address)
    try:
        # Send current state immediately on connect
        await websocket.send(json.dumps({'type': 'state', 'data': state.to_dict()}))
        await websocket.send(json.dumps({
            'type': 'info',
            'text': f'Bridge connected — TRXC:{state.trxcPort} TRXD:{state.trxdPort} GSMTAP:4729'
        }))

        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except Exception:
                continue

            if msg.get('type') == 'trxc_send':
                # Forward raw command to QEMU's TRXC port
                text = msg.get('text', '').strip()
                if text:
                    await broadcast({'type': 'trxc_cmd', 'text': '→ ' + text})
                    log.info("WS→TRXC: %s", text)
                    # Note: actual forwarding would require a TRXC client socket
                    # For now log only — QEMU listens, osmo-bts-trx sends
            elif msg.get('type') == 'sync_override':
                state.sync = msg.get('sync', state.sync)
                await broadcast_state()

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        clients.discard(websocket)
        log.info("WS client disconnected")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def khz_to_arfcn(khz: int) -> int:
    """Best-effort: UL frequency (kHz) → ARFCN."""
    if 890000 <= khz <= 915000:
        return (khz - 890000) // 200 + 1
    if 1710200 <= khz <= 1784800:
        return (khz - 1710200) // 200 + 512
    return 1


def udp_socket(port: int):
    """Create a non-blocking UDP socket bound to port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setblocking(False)
    sock.bind(('0.0.0.0', port))
    return sock


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def main(args):
    state.trxcPort = args.trxc_port
    state.trxdPort = args.trxc_port + 1

    loop = asyncio.get_event_loop()

    # TRXC sniffer
    try:
        trxc_sock = udp_socket(args.trxc_port)
        await loop.create_datagram_endpoint(
            lambda: TRXCProtocol(loop),
            sock=trxc_sock
        )
        log.info("TRXC sniffer on UDP %d", args.trxc_port)
    except OSError as e:
        log.warning("Cannot bind TRXC port %d: %s", args.trxc_port, e)

    # GSMTAP listener
    try:
        gsmtap_sock = udp_socket(args.gsmtap_port)
        await loop.create_datagram_endpoint(
            lambda: GsmtapProtocol(loop),
            sock=gsmtap_sock
        )
        log.info("GSMTAP listener on UDP %d", args.gsmtap_port)
    except OSError as e:
        log.warning("Cannot bind GSMTAP port %d: %s", args.gsmtap_port, e)

    # WebSocket server
    ws_server = await websockets.serve(
        ws_handler, '0.0.0.0', args.ws_port
    )
    log.info("WebSocket server on ws://0.0.0.0:%d", args.ws_port)
    log.info("Open index.html in your browser")

    await asyncio.Future()  # run forever


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calypso QEMU WebSocket bridge')
    parser.add_argument('--trxc-port',   type=int, default=6700)
    parser.add_argument('--gsmtap-port', type=int, default=4729)
    parser.add_argument('--ws-port',     type=int, default=9999)
    args = parser.parse_args()
    asyncio.run(main(args))
