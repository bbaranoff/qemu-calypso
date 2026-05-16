#!/usr/bin/env python3
"""
irda_capture.py — consomme le PTY UART_IRDA du firmware et l'écrit dans
/tmp/fw-irda.log avec le même préfixe timestamp `<epoch> +<rel>s [fw-irda]`
que les autres logs (qemu/bridge/osmocon/mobile).

Phase 3 du plan PLAN_CLAUDE_CODE_20260516_IRDA_DEBUG_CHANNEL.md.

Usage :
    # Direct
    python3 tools/irda_capture.py [pty_path]

    # Auto-symlinké par run.sh
    IRDA_PTY=/tmp/irda.pty.link python3 tools/irda_capture.py

Env vars :
    IRDA_PTY        — chemin PTY (default /tmp/irda.pty.link)
    FW_IRDA_LOG     — fichier de sortie (default /tmp/fw-irda.log)
    IRDA_BAUD       — baud rate (default 115200)
"""
from __future__ import annotations

import os
import sys
import time

IRDA_PTY    = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("IRDA_PTY", "/tmp/irda.pty.link")
FW_IRDA_LOG = os.environ.get("FW_IRDA_LOG", "/tmp/fw-irda.log")
IRDA_BAUD   = int(os.environ.get("IRDA_BAUD", "115200"))


def _open_pty():
    """Ouvre le PTY en bytes non-bufférisé. Préfère pyserial si dispo, sinon raw."""
    try:
        import serial
        return serial.Serial(IRDA_PTY, IRDA_BAUD, timeout=0.5)
    except ImportError:
        # Fallback : ouvrir en bytes raw, lire avec select
        return open(IRDA_PTY, "rb", buffering=0)


def main():
    # Wait que le PTY existe (utile si lancé en parallèle de run.sh)
    waited = 0
    while not os.path.exists(IRDA_PTY) and waited < 30:
        time.sleep(0.5); waited += 1
    if not os.path.exists(IRDA_PTY):
        sys.stderr.write(f"irda_capture: PTY {IRDA_PTY} absent après 15s, abort\n")
        sys.exit(2)

    # Écrit son PID pour le test_irda_capture_process_alive
    try:
        with open("/tmp/irda_capture.pid", "w") as f:
            f.write(str(os.getpid()))
    except Exception:
        pass

    src = _open_pty()
    out = open(FW_IRDA_LOG, "a", buffering=1)  # line-buffered
    t0 = time.time()
    sys.stderr.write(f"irda_capture: reading {IRDA_PTY} → {FW_IRDA_LOG}\n")

    buf = b""
    try:
        while True:
            try:
                chunk = src.read(4096)  # blocking ou timeout selon backend
            except Exception:
                time.sleep(0.1)
                continue
            if not chunk:
                time.sleep(0.05)
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                epoch = time.time()
                rel = epoch - t0
                txt = line.decode(errors="replace").rstrip("\r")
                out.write(f"{epoch:.3f} +{rel:.3f}s [fw-irda] {txt}\n")
    except KeyboardInterrupt:
        sys.stderr.write("irda_capture: stop\n")
    finally:
        try: out.close()
        except Exception: pass
        try: src.close()
        except Exception: pass


if __name__ == "__main__":
    main()
