#!/usr/bin/env python3
# Bridge : grgsm_decode -v (DECODE le vrai SI du flux continu) -> parse les
# lignes SI (PD=0x06) -> GSMTAP 16B + L2 -> 4730 (shunt feed_si -> DATA_IND ->
# mobile). Le SCH reel (patch grgsm) -> UDP 4731 (feed_sb).
#
# GESTION DU FN (corrige le bug "GSMTAP frame_number=0") :
#   - le FN reel vient des lignes SCHBSIC <bsic> <fn> <toa> (horloge fiable),
#     ou d'un "FN: <n>" explicite si grgsm -v en imprime un.
#   - on l'ecrit dans le champ frame_number du GSMTAP envoye a feed_si.
#   - rappel 51-multiframe BCCH non-combine TS0 DL (FN % 51) :
#       0,10,20,30,40 = FCCH | 1,11,21,31,41 = SCH | 2-5 = BCCH
#       6-9,12-19,22-29,32-39,42-49 = CCCH (PCH/AGCH) | 50 = idle
#     -> 31 = position SCH, 32 = debut d'un bloc CCCH, 51 = longueur multiframe.
import subprocess, socket, struct, re, sys, os, threading, time

ARFCN = int(os.environ.get("CALYPSO_CCCH_ARFCN", "514"))
CF = sys.argv[1] if len(sys.argv) > 1 else "/tmp/iq_grgsm.fifo"
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

SI = {0x1b:"SI3",0x1a:"SI2",0x1c:"SI4",0x19:"SI1",0x1d:"SI2bis",0x1e:"SI2ter",0x06:"RR"}  # NB: 0x21 = PAGING (pas SI13=0x00)
# CCCH/AGCH downlink (PORTE 3a) : l'IMM ASSIGN doit etre forwarde au mobile, pas
# seulement les SI. gr-gsm le decode (06 3f) mais si_bridge ne forwardait que les SI.
CCCH = {0x3f:"IMM-ASSIGN", 0x39:"IMM-ASSIGN-EXT", 0x3a:"IMM-ASSIGN-REJ"}
# Paging Request (PCH) : type1=0x21 (== GSM48_MT_RR_PAG_REQ_1, PAS SI13), type2=0x22, type3=0x24.
PAGING = {0x21:"PAG-REQ-1", 0x22:"PAG-REQ-2", 0x24:"PAG-REQ-3"}
GSMTAP_BCCH = 0x01
GSMTAP_AGCH = 0x04
GSMTAP_SDCCH4 = 0x07
GSMTAP_SACCH  = 0x07 | 0x80   # SDCCH/4 + ACCH = SACCH dediee SS0 (SI5/SI6 reels)

def fn51_role(fn):
    m = fn % 51
    if m in (0,10,20,30,40): return "FCCH"
    if m in (1,11,21,31,41): return "SCH"
    if m in (2,3,4,5):       return "BCCH"
    if m == 50:              return "idle"
    return "CCCH"

def gsmtap(fn, chan, l2):
    # v2, hdr_len=4 (16B), type=UM(1), ts=0, arfcn, signal_dbm=0 (RIEN d'injecte),
    # snr=0, frame_number=fn (REEL, decode), sub_type=chan, antenna=0, sub_slot=0, res=0
    hdr = struct.pack(">BBBBHbbIBBBB", 2, 4, 0x01, 0, ARFCN & 0xffff,
                      0, 0, fn & 0xffffffff, chan, 0, 0, 0)
    return hdr + l2

n = 0; nsch = 0; nsd = 0; nsa = 0; last_fn = 0; last_sd_fn = -1; last_sa_fn = -1

def handle_line(line):
    global n, nsch, nsd, nsa, last_fn, last_sd_fn, last_sa_fn
    # --- SCH reel : "SCHBSIC <bsic> <fn> <toa>" -> 4731 + horloge FN ---
    msch = re.search(r"SCHBSIC\s+(\d+)\s+(\d+)\s+(-?\d+)", line)
    if msch:
        bsic, fn, toa = int(msch.group(1)), int(msch.group(2)), int(msch.group(3))
        last_fn = fn
        s.sendto(b"SCH1" + struct.pack("<iii", bsic, fn, toa), ("127.0.0.1", 4731))
        nsch += 1
        if nsch <= 10 or nsch % 100 == 0:
            print("[si-bridge] SCH -> 4731 : BSIC=%d (ncc=%d bcc=%d) FN=%d (%%51=%d %s) TOA=%d  #%d"
                  % (bsic,(bsic>>3)&7,bsic&7,fn,fn%51,fn51_role(fn),toa,nsch), flush=True)
        return
    # --- FN explicite eventuel dans la ligne -v ---
    mfn = re.search(r"\bFN[:=]?\s*(\d+)", line)
    if mfn: last_fn = int(mfn.group(1))
    # --- ligne SI hexa ---
    lstr = line.strip()
    m = re.search(r":\s+([0-9a-fA-F][0-9a-fA-F ]+)$", lstr)
    if not m: return
    try: by = bytes(int(x,16) for x in m.group(1).split())
    except: return
    if len(by) >= 3 and by[1] == 0x06 and (by[2] in SI or by[2] in CCCH or by[2] in PAGING):
        mline_fn = re.match(r"^\s*(\d+)\b", lstr)
        si_fn = int(mline_fn.group(1)) if mline_fn else last_fn
        L2 = (by[:23] + b"\x2b"*23)[:23]
        # IMM ASSIGN (AGCH) ET Paging Request (PCH) -> meme chemin AGCH/PCH cote shunt
        # (feed_agch -> bloc CCCH -> firmware chan_nr=0x90 -> gsm48_rr_rx_pch_agch qui
        # dispatch IMM-ASS vs PAGING par msg type). SI -> BCCH.
        is_ccch = (by[2] in CCCH) or (by[2] in PAGING)
        chan = GSMTAP_AGCH if is_ccch else GSMTAP_BCCH
        s.sendto(gsmtap(si_fn, chan, L2), ("127.0.0.1", 4730))
        n += 1
        name = CCCH.get(by[2]) or PAGING.get(by[2]) or SI.get(by[2])
        if is_ccch or n <= 20 or n % 50 == 0:
            print("[si-bridge] %s (mt=0x%02x) FN=%d (%%51=%d %s) -> %s (4730)  #%d"
                  % (name, by[2], si_fn, si_fn%51, fn51_role(si_fn),
                     "feed_agch/PCH" if is_ccch else "feed_si", n), flush=True)
    # --- SDCCH/4 SS0 DL (#2) : LAPDm (UA/AUTH), pas du RR -> gate fn%51 in {22-25} ---
    mfn_sd = re.match(r"^\s*(\d+)\b", lstr)
    sd_fn = int(mfn_sd.group(1)) if mfn_sd else last_fn
    if len(by) >= 3 and (sd_fn % 51) in (22, 23, 24, 25):
        L2 = (by[:23] + b"\x2b"*23)[:23]
        # DEDUP PAR FN (corrige) : gr-gsm peut re-decoder le MEME bloc on-air (meme FN)
        # -> il faut sauter ce doublon (re-decode). MAIS la BTS RE-EMET legitimement le
        # MEME contenu (UA byte-identique) sur un FN DIFFERENT en reponse a une
        # retransmission SABM T200 du mobile : ce re-UA DOIT etre forwarde sinon le
        # mobile ne confirme jamais (UA rate -> boucle SABM -> "SABM not expected in
        # timer recovery"). L'ancien dedup-par-contenu (L2 != last_sd_l2) DROPPAIT ce
        # re-UA legitime. On dedup donc par FN decode (sd_fn) : grgsm emet chaque FN
        # on-air UNE seule fois (FN monotone, verifie) -> dedup-par-FN supprime le
        # re-decode (meme FN) ET forwarde le re-envoi BTS (FN distinct, meme contenu).
        # On saute toujours le bourrage LAPDm (UI c=0x03).
        ctrl = by[1] if len(by) > 1 else 0
        is_fill = (ctrl == 0x03)
        if (not is_fill) and sd_fn != last_sd_fn:
            last_sd_fn = sd_fn
            s.sendto(gsmtap(sd_fn, GSMTAP_SDCCH4, L2), ("127.0.0.1", 4730))
            nsd += 1
            if nsd <= 20 or nsd % 50 == 0:
                print("[si-bridge] SDCCH/4 SS0 DL (a0=0x%02x c=0x%02x) FN=%d (%%51=%d) -> feed_sdcch (4730)  #%d [new]"
                      % (by[0], ctrl, sd_fn, sd_fn%51, nsd), flush=True)
    # --- SACCH (SDCCH/4 SS0) DL : SI5(0x1d)/SI6(0x1e) REELS -> feed_sacch ---
    # Slots SACCH SS0 (combine CCCH+SDCCH/4) : fn%51 in {42-45}. grgsm donne le
    # bloc 23o = [L1 hdr 2][LAPDm: 03 03 len 06 mt L3...]. On detecte le RR header
    # "06 1d"/"06 1e" et on forwarde le bloc TEL QUEL au shunt (sub_type 0x87 =
    # SDCCH4|ACCH) -> feed_sacch REEL, remplace la fabrication SI3->SI6.
    if len(by) >= 8 and (sd_fn % 51) in (42, 43, 44, 45):
        mt_sa = None
        for off in range(2, min(9, len(by) - 1)):
            if by[off] == 0x06 and by[off + 1] in (0x1d, 0x1e):
                mt_sa = by[off + 1]; break
        if mt_sa is not None and sd_fn != last_sa_fn:
            last_sa_fn = sd_fn
            L2 = (by[:23] + b"\x2b" * 23)[:23]
            s.sendto(gsmtap(sd_fn, GSMTAP_SACCH, L2), ("127.0.0.1", 4730))
            nsa += 1
            if nsa <= 20 or nsa % 50 == 0:
                print("[si-bridge] SACCH SI%d (mt=0x%02x) FN=%d (%%51=%d) -> feed_sacch REEL (4730)  #%d"
                      % (5 if mt_sa == 0x1d else 6, mt_sa, sd_fn, sd_fn % 51, nsa), flush=True)

# === STAGE 3 : DECHIFFREMENT DL — DEUX grgsm QUI SE DELEGUENT ================
# PROBLEME resolu : tuer grgsm pour le relancer avec -k au CIPHER MODE retirait
# le lecteur de la FIFO -> l'ipc-device se figeait (DL FIFO plein -> osmo-trx
# SETPOWER no-response -> feed_iq gele -> pas de LU accept).
# SOLUTION (design B) : on ne tue JAMAIS le grgsm CLAIR. Deux process tournent
# en parallele, chacun sur SA FIFO (les deux alimentees par l'ipc-device relay) :
#   - grgsm CLAIR  sur CF (iq_grgsm.fifo)  : decode etablissement/ID/AUTH(RAND)/
#     CIPHER MODE COMMAND. Ne sort QUE des trames CRC-valides -> se tait tout seul
#     sur le dedie une fois le DL chiffre.
#   - grgsm CHIFFRE sur CIPH_CF (-e -k)    : spawne des qu'un Kc apparait (fenetre
#     RAND->cipher : il a le temps de locker FCCH/SCH AVANT que le DL passe chiffre),
#     decode le dedie chiffre (LU ACCEPT...). Tue seulement au release (Kc efface).
# Les deux sorties sont fusionnees par handle_line (sous lock) : le CRC de grgsm
# fait le tri, le dedup par FN evite les doublons. ZERO trou de lecteur sur la
# FIFO clair -> l'ipc-device ne gele plus. Le churn (spawn/kill) du grgsm chiffre
# est inoffensif grace au writer ipc-device non-bloquant (relay_fifo_writer).
CIPH_CF = os.environ.get("CALYPSO_CIPH_FIFO", "/tmp/iq_grgsm_ciph.fifo")
_hl_lock = threading.Lock()

# grgsm_decode HARDCODE le port GSMTAP UDP 4729 (socket_pdu UDP_SERVER+CLIENT, pas
# d'option). Deux instances -> "bind: Address already in use" -> le 2e crash
# (silencieux car stderr=DEVNULL) -> jamais de decode chiffre. On lit le STDOUT
# (-v), pas le 4729 -> on patche une COPIE pour le grgsm chiffre sur un port libre
# (4799). Genere au demarrage -> reproductible, survit au reclone du conteneur.
import shutil
def make_ciph_binary():
    src = shutil.which("grgsm_decode")
    if not src:
        return "grgsm_decode"
    dst = "/tmp/grgsm_decode_ciph"
    try:
        with open(src) as f:
            code = f.read()
        code = code.replace('"4729"', '"4799"')   # port GSMTAP unique (pas de conflit avec le clair)
        with open(dst, "w") as f:
            f.write(code)
        os.chmod(dst, 0o755)
        print("[si-bridge] grgsm chiffre patche -> %s (GSMTAP 4729->4799)" % dst, flush=True)
        return dst
    except Exception as e:
        print("[si-bridge] patch grgsm chiffre echoue (%s) -> fallback partage 4729" % e, flush=True)
        return src
GRGSM_CIPH = make_ciph_binary()

def spawn_grgsm(fifo, algo=0, kc_hex=None, binary="grgsm_decode", errlog=None):
    cmd = [binary, "-m", "BCCH_SDCCH4", "-t", "0", "-a", str(ARFCN),
           "-c", fifo, "-s", "1083333", "-v"]
    if kc_hex and algo in (1, 2, 3):
        cmd += ["-e", str(algo), "-k", kc_hex]
    err = open(errlog, "w") if errlog else subprocess.DEVNULL
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=err, text=True)

def reader_loop(proc, tag):
    # log brut par grgsm (diagnostic decipher) : /root/grgsm_clair.raw / _ciph.raw
    try: raw = open("/root/grgsm_%s.raw" % tag, "w")
    except Exception: raw = None
    for line in proc.stdout:
        if raw:
            try: raw.write(line); raw.flush()
            except Exception: pass
        with _hl_lock:
            handle_line(line)
    if raw:
        try: raw.close()
        except Exception: pass
    print("[si-bridge] grgsm %s termine (n=%d nsch=%d nsd=%d nsa=%d)"
          % (tag, n, nsch, nsd, nsa), flush=True)

def read_kc():
    try:
        with open("/dev/shm/calypso_kc", "rb") as f:
            b = f.read(32)
    except OSError:
        return (0, 0, None)
    if len(b) < 14:
        return (0, 0, None)
    seq = int.from_bytes(b[0:4], "little")
    if seq == 0:
        return (0, 0, None)
    return (seq, b[4], b[6:14].hex())

# --- grgsm CLAIR : toujours vivant (watchdog respawn si crash), JAMAIS tue au cipher ---
clear = {"proc": None}
def ensure_clear():
    p = clear["proc"]
    if p is None or p.poll() is not None:
        clear["proc"] = spawn_grgsm(CF)
        threading.Thread(target=reader_loop, args=(clear["proc"], "clair"),
                         daemon=True).start()
        print("[si-bridge] grgsm CLAIR spawn sur %s (jamais tue au cipher)" % CF, flush=True)

ensure_clear()

# --- grgsm CHIFFRE : spawne/tue selon le Kc, TOUJOURS sur CIPH_CF (jamais sur CF) ---
ciph = {"applied": (0, None), "proc": None}
while True:
    time.sleep(0.5)
    ensure_clear()                     # watchdog du grgsm clair
    seq, algo, kc = read_kc()
    want = (algo, kc) if (seq and algo in (1, 2, 3)) else (0, None)
    if want == ciph["applied"]:
        continue
    # etat Kc change -> on (re)cycle UNIQUEMENT le grgsm chiffre (sur CIPH_CF)
    p = ciph["proc"]
    if p is not None:
        try: p.kill(); p.wait(timeout=2)
        except Exception: pass
        ciph["proc"] = None
    ciph["applied"] = want
    if want[0]:
        np = spawn_grgsm(CIPH_CF, algo=want[0], kc_hex=want[1], binary=GRGSM_CIPH,
                         errlog="/root/grgsm_ciph.err")
        ciph["proc"] = np
        threading.Thread(target=reader_loop, args=(np, "chiffre"),
                         daemon=True).start()
        print("[si-bridge] grgsm CHIFFRE spawn sur %s : A5/%d kc=%s (decipher DL)"
              % (CIPH_CF, want[0], want[1]), flush=True)
    else:
        print("[si-bridge] Kc efface -> grgsm chiffre arrete (clair seul)", flush=True)
