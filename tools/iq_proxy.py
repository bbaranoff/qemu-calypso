#!/usr/bin/env python3
"""UDP proxy DL bursts : source (bridge.py | calypso-ipc-device) → QEMU BSP.

Sit entre source et QEMU BSP pour appliquer Doppler injection + burst dump
indépendamment du chemin GMSK choisi. Réutilise tools/doppler.py.

Architecture :
  source → udp:LISTEN_PORT (default 6702)
            → [apply_doppler + dump_burst]
            → udp:127.0.0.1:FWD_PORT (default 6712)
            → QEMU BSP (listens on CALYPSO_BSP_PORT=6712)

Pour activer ce proxy entre source et QEMU :
  1. set CALYPSO_BSP_PORT=6712 (= QEMU BSP listen on alt port)
  2. lance ce proxy : iq_proxy.py --listen 6702 --forward 6712
  3. source unchanged : envoie sur 6702 → proxy intercepte → QEMU

Format UDP attendu :
  [0]    TN
  [1:5]  FN big-endian
  [5]    RSSI
  [6:8]  reserved
  [8:]   IQ payload (int16 interleaved I,Q,I,Q,...) OU soft bits

Env vars (héritées de doppler.py) :
  CALYPSO_DOPPLER_HZ  Hz de drift injecté (default 0 = pass-through)
  CALYPSO_BURST_PRINT 0/1 dump des bursts (default 0)
  CALYPSO_SAMPLE_RATE Hz sample rate (default 270833)
"""
import argparse
import os
import socket
import struct
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from doppler import apply_doppler, dump_burst


def run(listen_port, forward_host, forward_port, iq_bytes_per_burst):
    """Boucle pass-through avec Doppler injection + dump.

    iq_bytes_per_burst : longueur attendue du payload IQ après header 8B
                         (pour BSP IQ mode = 4*148 = 592 bytes typiquement).
                         Si != taille reçue → pass-through sans rotation
                         (= probablement soft bits, pas IQ).
    """
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", listen_port))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fwd_addr = (forward_host, forward_port)

    print(f"[iq_proxy] listen=:{listen_port} forward={forward_host}:{forward_port}"
          f" doppler_hz={os.environ.get('CALYPSO_DOPPLER_HZ','0')}"
          f" burst_print={os.environ.get('CALYPSO_BURST_PRINT','0')}"
          f" iq_expected={iq_bytes_per_burst}",
          flush=True)

    n_fwd = 0
    n_iq = 0
    n_soft = 0
    while True:
        try:
            pkt, src_addr = rx.recvfrom(8192)
        except KeyboardInterrupt:
            break
        if len(pkt) < 8:
            tx.sendto(pkt, fwd_addr)
            continue
        hdr = pkt[:8]
        payload = pkt[8:]
        tn = hdr[0]
        fn = struct.unpack(">I", hdr[1:5])[0]

        # Détection IQ vs soft bits : si payload size = 4*N pour N proche
        # de 148, c'est IQ. Sinon soft bits.
        if iq_bytes_per_burst > 0 and abs(len(payload) - iq_bytes_per_burst) <= 8:
            # IQ : applique Doppler rotation
            payload_out = apply_doppler(payload)
            dump_burst(payload_out, tag="proxy-IQ", fn=fn, tn=tn)
            n_iq += 1
        else:
            # Soft bits ou autre : pass-through
            payload_out = payload
            n_soft += 1

        tx.sendto(hdr + payload_out, fwd_addr)
        n_fwd += 1
        if n_fwd <= 5 or n_fwd % 5000 == 0:
            print(f"[iq_proxy] fwd #{n_fwd} (iq={n_iq} soft={n_soft})"
                  f" tn={tn} fn={fn} payload={len(payload)}B",
                  flush=True)


def main():
    ap = argparse.ArgumentParser(description="UDP DL burst proxy with Doppler injection")
    ap.add_argument("--listen", type=int,
                    default=int(os.environ.get("CALYPSO_PROXY_LISTEN", "6702")),
                    help="UDP port à écouter (default 6702 = original BSP port)")
    ap.add_argument("--forward", type=int,
                    default=int(os.environ.get("CALYPSO_BSP_PORT", "6712")),
                    help="UDP port QEMU BSP (default 6712 = alt port)")
    ap.add_argument("--forward-host", default="127.0.0.1",
                    help="Host cible (default 127.0.0.1)")
    ap.add_argument("--iq-bytes", type=int, default=4 * 148,
                    help="Taille IQ payload attendu (default 592 = 148 samples I,Q)")
    args = ap.parse_args()

    if args.listen == args.forward and args.forward_host in ("127.0.0.1", "localhost"):
        print(f"[iq_proxy] FATAL listen={args.listen} == forward={args.forward}"
              " on same host → loop. Set CALYPSO_BSP_PORT to a different port.",
              file=sys.stderr)
        sys.exit(2)

    run(args.listen, args.forward_host, args.forward, args.iq_bytes)


if __name__ == "__main__":
    main()
