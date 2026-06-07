#!/usr/bin/env python3
# grgsm_fft_live.py — FFT EN DIRECT (spectre + waterfall ASCII couleur).
#
# SOURCES (par ordre de preference, override via CFILE) :
#   - tcp://HOST:PORT   : se connecte au duplexeur iq_hub (recommande, "de partout")
#   - /chemin/x.fifo    : FIFO (O_RDWR|O_NONBLOCK, draine le backlog, garde le frais)
#   - /chemin/x.cfile   : fichier (seek-tail)
# Defaut : tcp://127.0.0.1:7070 (le hub) avec repli /tmp/iq_fft.fifo.
#
# Affiche : spectre dB barre par bande, marqueur FCCH (+67708 Hz), waterfall
# defilant (shading unicode), et un RXLEV estime (dBm) a partir de l'amplitude.
import numpy as np, time, os, stat, sys, socket

CFILE = os.environ.get("CFILE", "tcp://127.0.0.1:7070")
FS    = float(os.environ.get("FS", "1083333"))     # 4 SPS relay
N     = int(os.environ.get("NFFT", "4096"))
COLS  = int(os.environ.get("FFT_COLS", "78"))
WROWS = int(os.environ.get("FFT_WATERFALL", "16"))
FCCH  = 1625000.0 / 24                              # +67708 Hz
# Calibration RXLEV : dBm = 20*log10(rms) + OFFSET. OFFSET ajustable empiriquement.
RXLEV_OFFSET = float(os.environ.get("RXLEV_OFFSET", "-60"))

SHADE = " .:-=+*#%@"                                # 10 niveaux waterfall
def _grad(x):                                       # x in [0,1] -> char ombre
    return SHADE[min(len(SHADE) - 1, max(0, int(x * (len(SHADE) - 1))))]

# ---------------- source ----------------
sock = None; fifo_fd = None; src_kind = "?"
def _connect():
    global sock, fifo_fd, src_kind
    if CFILE.startswith("tcp://"):
        hp = CFILE[6:]; h, p = hp.rsplit(":", 1)
        try:
            s = socket.socket(); s.settimeout(3); s.connect((h, int(p)))
            s.setblocking(False); sock = s; src_kind = "TCP " + hp; return True
        except OSError:
            # repli FIFO si le hub n'est pas (encore) la
            fb = os.environ.get("FFT_FIFO_FALLBACK", "/tmp/iq_fft.fifo")
            if os.path.exists(fb) or fb.endswith(".fifo"):
                if not os.path.exists(fb): os.mkfifo(fb, 0o666)
                fifo_fd = os.open(fb, os.O_RDWR | os.O_NONBLOCK)
                src_kind = "FIFO " + fb + " (repli)"; return True
            return False
    if stat.S_ISFIFO(os.stat(CFILE).st_mode) if os.path.exists(CFILE) else CFILE.endswith(".fifo"):
        if not os.path.exists(CFILE): os.mkfifo(CFILE, 0o666)
        fifo_fd = os.open(CFILE, os.O_RDWR | os.O_NONBLOCK)
        src_kind = "FIFO " + CFILE; return True
    src_kind = "file " + CFILE; return True

def _read_iq():
    need = N * 8
    if sock is not None:
        buf = b""
        try:
            while True:
                c = sock.recv(1 << 16)
                if not c: break
                buf += c
                if len(buf) > need * 4: buf = buf[-need:]
        except (BlockingIOError, InterruptedError): pass
        except OSError: return None                # connexion perdue
        _read_iq.tail = (_read_iq.tail + buf)[-need:]
        b = _read_iq.tail
        return np.frombuffer(b[:(len(b) // 8) * 8], dtype=np.complex64) if len(b) >= need else np.array([], np.complex64)
    if fifo_fd is not None:
        buf = b""
        while True:
            try: c = os.read(fifo_fd, 1 << 16)
            except BlockingIOError: break
            if not c: break
            buf += c
            if len(buf) > need * 4: buf = buf[-need:]
        _read_iq.tail = (_read_iq.tail + buf)[-need:]
        b = _read_iq.tail
        return np.frombuffer(b[:(len(b) // 8) * 8], dtype=np.complex64) if len(b) >= need else np.array([], np.complex64)
    try: sz = os.path.getsize(CFILE)
    except OSError: sz = 0
    if sz < need: return np.array([], np.complex64)
    with open(CFILE, "rb") as f:
        f.seek(max(0, (sz // 8 - N) * 8)); return np.frombuffer(f.read(need), np.complex64)
_read_iq.tail = b""

while not _connect():
    sys.stdout.write("\r[fft] attente du hub %s ..." % CFILE); sys.stdout.flush(); time.sleep(1)

waterfall = []
while True:
    iq = _read_iq()
    if iq is None:                                 # TCP coupe -> reconnexion
        try: sock.close()
        except Exception: pass
        sock = None; fifo_fd = None
        while not _connect(): time.sleep(1)
        continue
    sys.stdout.write("\033[H\033[2J")              # home + clear (sans flicker system clear)
    print("== FFT LIVE  src=%s  fs=%.0f  N=%d ==  FCCH attendu %+.0f Hz" % (src_kind, FS, N, FCCH))
    if len(iq) < N:
        print("\n  pas (encore) de donnees -> le hub pousse-t-il l'I/Q ? (relance ./run.sh)")
        time.sleep(0.6); continue

    win = np.hanning(len(iq))
    sp  = np.abs(np.fft.fftshift(np.fft.fft(iq * win))) + 1e-12
    fr  = np.fft.fftshift(np.fft.fftfreq(len(iq), 1 / FS))
    spdb = 20 * np.log10(sp / sp.max())            # 0 dB au pic, negatif ailleurs
    pk = fr[np.argmax(sp)]
    rms = np.sqrt(np.mean(np.abs(iq) ** 2))
    rxlev_dbm = 20 * np.log10(rms + 1e-12) + RXLEV_OFFSET
    dphi = np.std(np.angle(iq[1:] * np.conj(iq[:-1])))

    print("  RXLEV~%.0f dBm  rms=%.4f  max=%.4f | PIC %+.0f Hz | dphi std=%.2f (GMSK~1.57)\n"
          % (rxlev_dbm, rms, np.abs(iq).max(), pk, dphi))

    # spectre dB par bande
    B = COLS
    step = max(1, len(sp) // B)
    spb = np.array([spdb[i * step:(i + 1) * step].max() for i in range(B)])
    frb = np.array([fr[min(len(fr) - 1, i * step)] for i in range(B)])
    FLOOR = -50.0                                   # dynamique affichee
    rows = 12
    for lvl in range(rows, 0, -1):
        thr = FLOOR * (1 - lvl / rows)
        line = "".join("#" if spb[i] >= thr else " " for i in range(B))
        print("%6.0fdB |%s" % (thr, line))
    axis = "".join("^" if abs(frb[i] - FCCH) < FS / B else ("|" if i % 10 == 0 else " ") for i in range(B))
    print("        +%s  (^ = FCCH)" % axis)
    print("   %+.0fkHz %s%+.0fkHz" % (fr[0] / 1e3, " " * (B - 14), fr[-1] / 1e3))

    # waterfall defilant
    norm = np.clip((spb - FLOOR) / (0 - FLOOR), 0, 1)
    waterfall.insert(0, "".join(_grad(v) for v in norm))
    waterfall = waterfall[:WROWS]
    print("\n  waterfall (haut=recent):")
    for ln in waterfall: print("  |%s|" % ln)
    time.sleep(0.4)
