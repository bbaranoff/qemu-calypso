#!/usr/bin/env python3
"""corr_iq.py -- diagnostic complet de l'I/Q du correlateur DSP (Calypso QEMU).

Metrique cle = COHERENCE du tone = |Sum iq[k+1].conj(iq[k])| / Sum|iq[k+1]||iq[k]|
  1.0 = tone pur (FCCH) ; ~0 = bruit/data GMSK.
dphi (rotation moyenne/sample), exprime en unites de pi/2 -> infere le SPS :
  +1.00 x pi/2  = FCCH @1SPS (ce que le correlateur veut)
  +0.25 x pi/2  = FCCH @4SPS non-decime (il faut decimer ÷4)
  negatif       = miroir spectral (I/Q swap -> CALYPSO_DL_IQ_CONJ=1)

Sources :
  shunt  : /dev/shm/dsp_iq.cfile (fc32) -- I/Q d'entree du shunt (reference propre)
  rxdump : /tmp/iq_rx_*.bin (CALYPSO_IQDUMP) -- bursts FCCH ecrits en DARAM 0x2a00
  bursts : /dev/shm/bursts.cfile (BSP_DUMP_RX_FILE, IQ16) -- idem, avec fn/tn
  daram  : 0x2a00 live via monitor qemu (best-effort, racy)
  ddump  : /dev/shm/daram_2a00.cfile (CALYPSO_DARAM_DUMP) -- MEME buffer 0x2a00
           mais dumpe DE L'INTERIEUR de qemu a l'instant ou le detecteur FB le
           lit : atomique, non-racy. C'est LA mesure de la DESTINATION.

Usage : corr_iq.py [--src auto|shunt|rxdump|bursts|daram|ddump|all] [--fs Hz]
"""
import argparse, os, glob, socket, struct, sys, time
import numpy as np

FS_SYM  = 270833.0
FCCH_HZ = FS_SYM / 4.0
PI2     = np.pi / 2.0


# ---------- metriques ----------
def tone_metrics(iq, fs):
    n = len(iq)
    if n < 8:
        return dict(n=n, rms=0, peak=0, dc=0j, coh=0.0, dphi=0.0, fpk=0.0, conc=0.0, zero=0.0)
    mag = np.abs(iq)
    rms = float(np.sqrt(np.mean(mag ** 2)))
    dc  = complex(np.mean(iq))
    prod = iq[1:] * np.conj(iq[:-1])
    den  = float(np.sum(np.abs(iq[1:]) * np.abs(iq[:-1]))) + 1e-12
    acc  = np.sum(prod)
    coh  = float(np.abs(acc) / den)
    dphi = float(np.angle(acc))
    w = iq - dc
    win = np.hanning(n)
    sp = np.fft.fftshift(np.abs(np.fft.fft(w * win)))
    fr = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / fs))
    k = int(np.argmax(sp))
    return dict(n=n, rms=rms, peak=float(mag.max()), dc=dc, coh=coh, dphi=dphi,
                fpk=float(fr[k]), conc=float(sp[k] / (np.mean(sp) + 1e-12)),
                zero=float(np.mean(mag < 1e-6)))


def verdict(m):
    if m["rms"] < 1e-9 or m["zero"] > 0.98:
        return "VIDE -- rien fed au correlateur"
    r = m["dphi"] / PI2
    near = abs(abs(m["fpk"]) - FCCH_HZ) < 0.15 * FCCH_HZ   # pic FFT a +/-67708 Hz
    # (1) burst unique coherent -> dphi fiable -> SPS + orientation
    if m["coh"] > 0.9:
        if abs(abs(r) - 1.0) < 0.20:
            return ("FCCH @1SPS PROPRE (dphi=%+.2fx pi/2) -- feed CORRECT" % r if r > 0
                    else "FCCH @1SPS MIROIR (dphi=%+.2fx pi/2) -> CALYPSO_DL_IQ_CONJ=1" % r)
        if abs(abs(r) - 0.25) < 0.10:
            return "FCCH @4SPS NON-DECIME (dphi=%+.2fx pi/2) -> decimer ÷4 (CALYPSO_BSP_IQ_DECIM=4)" % r
    # (2) flux continu / complement : pic FFT sur FCCH
    if m["conc"] > 20 and near:
        return "FCCH PRESENTE (FFT %+.0f Hz, conc=%.0f) -- signal OK (flux)" % (m["fpk"], m["conc"])
    if m["coh"] < 0.4 and m["conc"] < 10:
        return "BRUIT/DATA (coh=%.2f) -- pas un tone FCCH" % m["coh"]
    return "coherent hors FCCH std (dphi=%+.2fx pi/2, FFT %+.0f Hz)" % (r, m["fpk"])


def report(iq, fs, label):
    m = tone_metrics(iq, fs)
    print("\n=== %s ===  N=%d  fs=%.0f Hz" % (label, m["n"], fs))
    if m["n"] < 8:
        print("  (trop court)"); return m
    print("  rms=%.3g peak=%.3g |DC|=%.3g zeros=%.0f%%" % (m["rms"], m["peak"], abs(m["dc"]), 100 * m["zero"]))
    _LAST["coh"] = m["coh"]; _LAST["fft"] = m["fpk"]
    print("  coherence=%.3f  dphi=%+.3f rad/samp (%+.2fx pi/2)  FFTpic=%+.0f Hz conc=%.1fx  [FCCH@1SPS=+1.571]"
          % (m["coh"], m["dphi"], m["dphi"] / PI2, m["fpk"], m["conc"]))
    print("  VERDICT: %s" % verdict(m))
    return m


# ---------- loaders ----------
def load_fc32(path, n, off):
    with open(path, "rb") as f:
        f.seek(off * 8, os.SEEK_SET)
        raw = f.read(n * 8)
    return np.frombuffer(raw, dtype=np.complex64).astype(np.complex128)


def load_raw_bins(rxdir):
    out = []
    for f in sorted(glob.glob(os.path.join(rxdir, "iq_rx_*.bin"))):
        a = np.frombuffer(open(f, "rb").read(), dtype="<i2").astype(np.float32)
        a = a[:(len(a) // 2) * 2]
        out.append((os.path.basename(f), a[0::2] + 1j * a[1::2]))
    return out


def load_iq16(path, maxrec=400):
    out = []
    with open(path, "rb") as f:
        while len(out) < maxrec:
            hdr = f.read(12)
            if len(hdr) < 12:
                break
            magic, fn, tn, nint16, _pad = struct.unpack("<4sIBHB", hdr)
            if magic != b"IQ16":
                f.seek(-11, os.SEEK_CUR); continue
            raw = f.read(nint16 * 2)
            if len(raw) < nint16 * 2:
                break
            a = np.frombuffer(raw, dtype="<i2").astype(np.float32)
            out.append(("fn=%u/tn=%u" % (fn, tn), a[0::2] + 1j * a[1::2]))
    return out


_LAST = {}

def last_report_coh_fft():
    return _LAST.get("coh"), _LAST.get("fft")

def read_daram(mon, addr, words, tries=6):
    """Best-effort : lit `words` mots 16b a addr via HMP xp, garde la lecture la
    plus coherente (le DSP ecrit en concurrence -> racy)."""
    best = None
    for _ in range(tries):
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.settimeout(2.5)
            s.connect(mon); s.recv(4096)
            s.sendall(("xp/%dhx 0x%x\n" % (words, addr)).encode())
            buf = b""; t0 = time.time()
            while time.time() - t0 < 2.5:
                d = s.recv(65536)
                if not d: break
                buf += d
                if b"(qemu)" in buf: break
            s.close()
        except Exception as e:
            continue
        vals = []
        for line in buf.decode(errors="replace").splitlines():
            if ":" not in line: continue
            for tok in line.split(":", 1)[1].split():
                if tok.startswith("0x"):
                    try: vals.append(int(tok, 16))
                    except ValueError: pass
        a = np.array(vals, dtype=np.uint16).astype(np.int16).astype(np.float32)
        a = a[:(len(a) // 2) * 2]
        iq = a[0::2] + 1j * a[1::2]
        if len(iq) < 8: continue
        m = tone_metrics(iq, FS_SYM)
        if best is None or m["coh"] > best[0]:
            best = (m["coh"], iq)
    return best[1] if best else np.array([], dtype=complex)


# ---------- scan per-burst (rxdump / bursts) ----------
def scan_bursts(recs, fs, label):
    scored = []
    for name, iq in recs:
        if len(iq) < 8: continue
        m = tone_metrics(iq, fs)
        if m["rms"] < 1000: continue
        scored.append((m["coh"], m["dphi"], m["rms"], name, iq))
    if not scored:
        print("  (aucun burst non-nul)"); return
    # tri sur les scalaires seuls : le 5e champ est un ndarray et rend
    # la comparaison de tuples ambigue des que (coh,dphi,rms) sont egaux.
    scored.sort(key=lambda x: (x[0], x[1], x[2]), reverse=True)
    nf = sum(1 for c, *_ in scored if c > 0.85)
    print("%s : %d bursts non-nuls, %d coherents (FCCH, coh>0.85)" % (label, len(scored), nf))
    for c, d, r, name, _ in scored[:8]:
        print("  %-14s coh=%.3f dphi=%+.3f (%+.2fx pi/2) rms=%.0f %s"
              % (name, c, d, d / PI2, r, "<- FCCH" if c > 0.85 else ""))
    report(scored[0][4], fs, "burst le + coherent = %s" % scored[0][3])


# ---------- main ----------
# ---------------------------------------------------------------------------
# [2026-07-30] L'adresse du tampon corrélateur se LIT SUR LE RUN, pas dans
# l'environnement de l'appelant.
#
# Pourquoi : le défaut historique était figé sur 0xFFD04400 (mot DSP 0x2a00),
# alors que les profils natifs livrent en CALYPSO_BSP_DARAM_ADDR=0x4c00
# (-> 0xFFD08800). Résultat : « VIDE -- rien fed au correlateur » sur un run où
# 0x4c00 contenait de la FCCH parfaite (coh=0.998, dphi=+1.567). Un faux négatif
# spectaculaire, et il a survécu à une première correction parce que celle-ci
# lisait os.environ : depuis un shell interactif la variable n'est pas exportée,
# donc on retombait sur l'ancien défaut sans le voir.
#
# Ordre de résolution, du plus fiable au moins fiable — et on IMPRIME la source,
# parce qu'un instrument qui ne dit pas d'où vient son réglage produit des
# verdicts qu'on ne peut pas auditer :
#   1. /proc/<pid qemu>/environ   — l'environnement RÉEL du process qui tourne ;
#   2. le manifeste dans qemu.log — la doctrine du projet (« lire le manifeste,
#      jamais la CLI ») ;
#   3. CALYPSO_BSP_DARAM_ADDR de l'appelant ;
#   4. 0x2a00, l'ancien défaut, annoncé comme tel.
# ---------------------------------------------------------------------------
def _word_to_arm(word):
    return 0xFFD00000 + (word - 0x0800) * 2

def resolve_daram_addr(verbose=True):
    src = None; word = None
    for pid in glob.glob("/proc/[0-9]*"):
        try:
            if "qemu-system-arm" not in open(pid + "/cmdline", "rb").read().decode(errors="replace"):
                continue
            env = open(pid + "/environ", "rb").read().decode(errors="replace")
            for kv in env.split("\0"):
                if kv.startswith("CALYPSO_BSP_DARAM_ADDR="):
                    word = int(kv.split("=", 1)[1], 16); src = "/proc/%s/environ (run vivant)" % pid.split("/")[-1]
                    break
            if word is not None: break
        except Exception:
            continue
    if word is None:
        for log in ("/root/qemu.log", "/tmp/calypso/logs/qemu.log"):
            try:
                import re
                m = None
                with open(log, "rb") as f:
                    for line in f:
                        mm = re.search(rb"CALYPSO_BSP_DARAM_ADDR=(0x[0-9a-fA-F]+)", line)
                        if mm: m = mm
                if m:
                    word = int(m.group(1), 16); src = "manifeste %s" % log
                    break
            except Exception:
                continue
    if word is None:
        e = os.environ.get("CALYPSO_BSP_DARAM_ADDR", "")
        if e:
            try: word = int(e, 16); src = "CALYPSO_BSP_DARAM_ADDR de l'appelant"
            except Exception: word = None
    if word is None:
        word = 0x2a00; src = "DEFAUT 0x2a00 -- aucun run trouve, verdict a prendre avec des pincettes"
    addr = _word_to_arm(word)
    if verbose:
        print("[daram] mot DSP 0x%04x -> ARM 0x%08X   source : %s" % (word, addr, src))
    return word, addr


def main():
    ap = argparse.ArgumentParser(description="Diag I/Q correlateur DSP")
    ap.add_argument("--src", choices=["auto", "shunt", "rxdump", "bursts", "daram", "ddump", "all"], default="auto")
    ap.add_argument("--shunt", default="/dev/shm/dsp_iq.cfile")
    ap.add_argument("--rxdir", default="/tmp")
    ap.add_argument("--bursts", default="/dev/shm/bursts.cfile")
    # [2026-07-30] Chemin CORRIGÉ. L'ancien défaut /tmp/qemu-calypso-mon.sock
    # n'a jamais existé : run.sh lance QEMU avec
    #   -monitor unix:/tmp/calypso/qemu-monitor.sock,server,nowait
    # (vérifié sur la ligne de commande du process). D'où « monitor absent
    # (qemu down ?) » alors que QEMU tournait depuis 6 minutes.
    ap.add_argument("--mon", default="/tmp/calypso/qemu-monitor.sock")
    ap.add_argument("--ddump", default="/dev/shm/daram_2a00.cfile")
    _dw, _da = resolve_daram_addr(verbose=False)
    ap.add_argument("--addr", default="0x%08X" % _da)
    ap.add_argument("--words", type=int, default=296)   # daram_len = 296 int16 = 148 cplx
    ap.add_argument("--n", type=int, default=40000)
    ap.add_argument("--off", type=int, default=-1)
    ap.add_argument("--fs", type=float, default=None)
    a = ap.parse_args()

    def do_shunt():
        if not os.path.exists(a.shunt): print("shunt: absent"); return
        sz = os.path.getsize(a.shunt); tot = sz // 8
        off = a.off if a.off >= 0 else max(0, tot - a.n)
        report(load_fc32(a.shunt, a.n, off), a.fs or 1083333.0, "shunt dsp_iq.cfile @%d (fc32, entree 4SPS)" % off)

    def do_rxdump():
        recs = load_raw_bins(a.rxdir)
        if not recs: print("rxdump: aucun iq_rx_*.bin (CALYPSO_IQDUMP=1 + relance)"); return
        scan_bursts(recs, a.fs or 270833.0, "rxdump (/tmp/iq_rx, fed 0x%04x @1SPS)" % _dw)

    def do_bursts():
        if not os.path.exists(a.bursts) or os.path.getsize(a.bursts) < 13:
            print("bursts: %s absent/vide (BSP_DUMP_RX_FILE=1 + relance, mode direct ok)" % a.bursts); return
        scan_bursts(load_iq16(a.bursts), a.fs or 270833.0, "bursts.cfile (IQ16, fed 0x%04x @1SPS)" % _dw)

    def do_ddump():
        """DESTINATION, non-racy : buffer 0x2a00 dumpe par qemu au moment ou le
        detecteur FB le lit. Le kernel attend du FCCH @1SPS (dphi = +1.00x pi/2)."""
        p = a.ddump
        if not os.path.exists(p) or os.path.getsize(p) < 13:
            print("ddump: %s absent/vide." % p)
            print("       Ce n'est PAS forcement un defaut d'activation : la sonde cree ce")
            print("       fichier des l'armement (fopen), puis n'ecrit que si BOTH :")
            print("         1. exec_pc == CALYPSO_DARAM_DUMP_PC (defaut 0x9ac0 -- banque")
            print("            atteinte seulement sous native_helped, entree reroutee) ;")
            print("         2. d_fb_mode[0x08f9] != 0, sauf CALYPSO_DARAM_DUMP_ANYMODE=1.")
            print("       Et sa base filmee est CODEE EN DUR a 0x2a00. Sur un banc qui")
            print("       livre ailleurs, utilisez --src bursts ou --src daram.")
            print("       Verifier : grep -a 'DARAM-DUMP armed' /root/qemu.log")
            return
        recs = load_iq16(p, maxrec=400)
        print("\n#### DESTINATION data[0x2a00..0x2b28) -- dump interne qemu (non-racy)")
        scan_bursts(recs, a.fs or 270833.0, "ddump %s" % p)
        # verdict de conformite au kernel : il veut du FCCH @1SPS
        best = None
        for _n, iq in recs:
            if len(iq) < 8: continue
            m = tone_metrics(iq, a.fs or 270833.0)
            if best is None or m["coh"] > best["coh"]: best = m
        if best:
            r = best["dphi"] / PI2
            ok = best["coh"] > 0.85 and abs(abs(r) - 1.0) < 0.20
            print("  CONFORMITE KERNEL (attendu FCCH @1SPS, dphi=+1.00x pi/2) : %s"
                  % ("OK -- le buffer contient ce que le correlateur cherche ; "
                     "le probleme est EN AVAL du buffer" if ok else
                     "NON (coh=%.3f dphi=%+.2fx pi/2) -- le REMPLISSAGE est encore fautif "
                     "(rate/phase/pairing), pas l aval" % (best["coh"], r)))
            if not ok:
                # [2026-07-30] Le « remplissage » a un mecanisme precis, mesure dans
                # le modele. Sans ces trois lignes le verdict envoie chercher un bug
                # de signal alors que c'est une gate.
                print("  MECANISME  : ce buffer n'est PAS ecrit par l'hote. c54x_bsp_load()")
                print("               (calypso_c54x.c) copie seulement les echantillons dans")
                print("               s->bsp_buf ; c'est le DSP qui les TIRE, un mot par")
                print("               PORTR sur PA=0xF430 (ou 0x0034), bsp_pos remis a 0 par")
                print("               rafale. L'appariement I/Q depend donc du NOMBRE de")
                print("               lectures que fait le DSP : une de trop ou de moins et")
                print("               tout decale -> exactement rate/phase/pairing.")
                print("  A VERIFIER : 1) CALYPSO_FIX_PORTR — defaut 0 ! Sans elle le port ne")
                print("                  sert AUCUN echantillon (calypso_c54x.c, gate autour")
                print("                  du `if (fix_portr && (pa == 0xF430 || pa == 0x0034))`).")
                print("                  Un buffer vide ou incoherent commence par la.")
                print("               2) l'ADRESSE analysee : ce dump est fige sur 0x2a00 par")
                print("                  la sonde C, alors que CALYPSO_BSP_DARAM_ADDR peut")
                print("                  pointer ailleurs (0x4c00 par defaut en profil natif).")
                print("                  Verifier au manifeste, et utiliser --src daram")
                print("                  --addr/--words pour lire la zone reellement visee.")
                print("               3) le point d'entree du correlateur : selon")
                print("                  CALYPSO_FB_CORR_ENTRY, le demod lit 0x4c00 en stride 5")
                print("                  (polyphase) ou 0x9213 — la zone conforme n'est pas la")
                print("                  meme. Le dump ne prouve rien s'il filme la mauvaise.")

    def do_daram():
        resolve_daram_addr(verbose=True)   # trace la provenance du reglage
        # ⚠️ PIEGE, precise le 30/07 apres mesure. Ce qui a tue QEMU ce jour-la,
        # c'est la lecture de 0xFFD001A8 : cette cellule porte un MemoryRegion
        # overlay de 2 octets dont le handler appelle calypso_mbx() hors du
        # thread CPU. Ce n'est PAS la fenetre entiere : 0xFFD04400 et 0xFFD08800
        # ont ete lues plusieurs fois sur un run vivant, QEMU a survecu (577 s,
        # insn qui avance). Regle : tout sauf 0xFFD001A8, et jamais d'aveugle.
        if not os.path.exists(a.mon): print("daram: monitor %s absent (qemu down ?)" % a.mon); return
        iq = read_daram(a.mon, int(a.addr, 16), a.words)
        if len(iq) < 8:
            print("daram: lecture vide/echec @%s (0x2a00 hors fenetre API ARM ? racy) -- fie-toi a rxdump/FCCH-PROBE" % a.addr); return
        report(iq, a.fs or 270833.0, "DARAM %s (%d mots, correlateur live, best-of-6)" % (a.addr, a.words))
        # [2026-07-30] `xp` au monitor n'est PAS atomique : le BSP reecrit les 296
        # mots pendant la lecture, on obtient donc parfois un RACCORD de deux
        # bursts. Signature de ce raccord, et non du tampon : coh qui s'effondre
        # ET pic FFT a ~la MOITIE de 67708 Hz (un saut de phase au milieu double
        # la periode apparente). Mesure du 30/07, MEME tampon MEME run : 0.998 /
        # +67708 Hz, puis 0.514 / +32939 Hz -- alors que ce que le BSP ECRIVAIT
        # etait 140/140 coherent FCCH (--src bursts). D'ou cet avertissement, pour
        # qu'un artefact de lecture ne devienne pas un verdict sur le feed.
        try:
            _c, _f = last_report_coh_fft()
            if _c is not None and _c < 0.90:
                print("  ⚠ LECTURE, PAS TAMPON : coh=%.3f < 0.90 sur un `xp` non atomique." % _c)
                if _f is not None and abs(abs(_f) - 33854.0) < 6000:
                    print("     pic a %+.0f Hz ~= 67708/2 -> signature d'un RACCORD de deux bursts." % _f)
                print("     N'attendez PAS un taux : au debit de livraison courant (~22")
                print("     bursts/s pour 296 mots lus un par un), une lecture propre est RARE.")
                print("     Releve du 30/07 : 2 lectures a coh=0.998, puis 5 d'affilee entre")
                print("     0.36 et 0.51 -- MEME tampon, MEME run, contenu ECRIT parfait.")
                print("     Le juge de ce qui est ECRIT est --src bursts. Pour une capture")
                print("     NON RACY, prise depuis le thread CPU, relancez le run avec :")
                print("       CALYPSO_DARAM_DUMP=1 CALYPSO_DARAM_DUMP_ADDR=0x4c00 \\")
                print("       CALYPSO_DARAM_DUMP_PC=<un PC execute> CALYPSO_DARAM_DUMP_ANYMODE=1")
                print("     puis --src ddump. (0xb530 est mesure a ~2168 passages.)")
        except Exception:
            pass

    src = a.src
    if src == "auto":
        if os.path.exists(a.ddump) and os.path.getsize(a.ddump) > 13: src = "ddump"
        elif os.path.exists(a.bursts) and os.path.getsize(a.bursts) > 13: src = "bursts"
        elif glob.glob(os.path.join(a.rxdir, "iq_rx_*.bin")): src = "rxdump"
        else: src = "shunt"
        print("[auto] source = %s" % src)

    if src == "all":
        for fn in (do_shunt, do_bursts, do_rxdump, do_ddump, do_daram):
            try: fn()
            except Exception as e: print("  ERR:", e)
    else:
        {"shunt": do_shunt, "rxdump": do_rxdump, "bursts": do_bursts,
         "daram": do_daram, "ddump": do_ddump}[src]()


if __name__ == "__main__":
    main()
