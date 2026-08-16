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

# ---- SOUS-VOIE DEDIEE ACTIVE : lue, plus jamais supposee (12/08/2026) --------
#
# Ce fichier ne relayait le descendant dedie que pour `fn%51 in {22..25}`, soit le
# SEUL sous-canal SS0 du SDCCH/4 combine. Mesure du 12/08 : sur trois tentatives
# de LOCATION UPDATING consecutives, le BSC a donne SS2, puis SS1, puis SS0 ; les
# deux premieres sont mortes sur `RR_REL_IND` a +6 s faute d'UA descendante, et
# seule la troisieme a rendu `RR_EST_CNF` puis `LOCATION UPDATING ACCEPT`.
# 48 secondes pour un LU, et deux tentatives perdues, par cette seule ligne.
#
# gr-gsm, lui, decode BIEN les quatre sous-canaux — releve sur /root/grgsm_clair.raw
# (toutes les lignes du decodeur, AVANT cette gate) :
#     fn%51=22 : 43 blocs (SS0)   fn%51=26 : 56 (SS1)
#     fn%51=32 : 56   (SS2)       fn%51=36 : 36 (SS3)
# Le probleme n'a donc jamais ete le decodage : c'etait ce filtre.
#
# SOURCE AUTORITAIRE — la MEME que celle qui pilote deja la fenetre de
# presentation du shunt : le `chan_nr` que le FIRMWARE annonce, publie par
# l1ctl_sock.c:418 dans /dev/shm/calypso_dcch_cfg.
#   16 o : seq u32 LE @0 | kind @4 (0=SDCCH/4, 1=SDCCH/8) | ss @5 | tn @6 | chan_nr @7
# ⚠️ NE PAS apprendre la sous-voie depuis les IMM ASSIGN du CCCH : il porte ceux
#    de TOUS les abonnes (mesure du 08/08 : 68 IMM ASSIGN d'un RA etranger pour
#    un RACH a nous). C'est la fausse piste que l1ctl_sock.c documente en tete.
# ⚠️ os.pread obligatoire : un BufferedReader avec seek(0)+read() rendrait
#    eternellement la premiere valeur.
DCCH_CFG_PATH = "/dev/shm/calypso_dcch_cfg"
_SD4_DL    = (22, 26, 32, 36)   # base fn%51  du canal principal, SDCCH/4 combine
_SD4_SACCH = (42, 46, 93, 97)   # base fn%102 de la SACCH        (GSM 05.02)
_dcch_vu = [None]

def dcch_sousvoie():
    """(base_dl fn%51, base_sacch fn%102, kind). Repli (22, 42, 0) = ancien defaut."""
    try:
        fd = os.open(DCCH_CFG_PATH, os.O_RDONLY)
        try:
            b = os.pread(fd, 16, 0)
        finally:
            os.close(fd)
    except OSError:
        return 22, 42, 0
    if len(b) < 8 or int.from_bytes(b[0:4], "little") == 0:
        return 22, 42, 0
    kind, ss = b[4], b[5]
    if kind:
        # SDCCH/8 : base principale ss*4, alignee sur calypso_dsp_shunt_set_dcch.
        # Sa SACCH n'est modelisee NI ici NI dans le shunt (sacch_base n'a que 4
        # entrees, indexees par les bases du /4) — et de toute facon aucun
        # decodeur SDCCH/8 ne tourne : `-m BCCH_SDCCH4` ne peut pas les produire.
        # On ne fabrique donc pas une fenetre SACCH qu'on ne sait pas calculer.
        return (ss & 7) * 4, None, 1
    i = ss & 3
    return _SD4_DL[i], _SD4_SACCH[i], 0

def dcch_annonce(base_dl, base_sa, kind):
    """Une ligne a CHAQUE changement de sous-voie : sans elle, un relais sur la
    mauvaise sous-voie serait indiscernable d'une absence de relais."""
    etat = (base_dl, base_sa, kind)
    if _dcch_vu[0] == etat:
        return
    _dcch_vu[0] = etat
    if kind:
        print("[si-bridge] sous-voie dediee : SDCCH/8 base fn%%51 %d-%d — "
              "AUCUN decodeur ne produit ces blocs (-m BCCH_SDCCH4 ne les voit "
              "pas) et leur SACCH n'est pas modelisee : ce dedie restera muet"
              % (base_dl, base_dl + 3), flush=True)
    else:
        print("[si-bridge] sous-voie dediee : SDCCH/4 SS%d -> relais DL fn%%51 "
              "%d-%d, SACCH fn%%102 %d-%d"
              % (_SD4_DL.index(base_dl), base_dl, base_dl + 3,
                 base_sa, base_sa + 3), flush=True)

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
    # --- DL du canal dedie (#2) : LAPDm (UA/AUTH), pas du RR ---
    # La fenetre suit la sous-voie REELLEMENT active (cf. dcch_sousvoie), elle
    # n'est plus figee sur SS0. C'est la correction du 12/08 : deux LU sur trois
    # mouraient parce que le BSC avait donne SS2 puis SS1.
    mfn_sd = re.match(r"^\s*(\d+)\b", lstr)
    sd_fn = int(mfn_sd.group(1)) if mfn_sd else last_fn
    _b_dl, _b_sa, _kind = dcch_sousvoie()
    dcch_annonce(_b_dl, _b_sa, _kind)
    if len(by) >= 3 and _b_dl <= (sd_fn % 51) <= _b_dl + 3:
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
                print("[si-bridge] SDCCH/%d DL (a0=0x%02x c=0x%02x) FN=%d (%%51=%d, base %d) -> feed_sdcch (4730)  #%d [new]"
                      % (8 if _kind else 4, by[0], ctrl, sd_fn, sd_fn%51, _b_dl, nsd), flush=True)
            # ARMEMENT DU TCH : l'ASSIGNMENT COMMAND passe par ce meme bloc DL.
            check_assignment(by, sd_fn)
    # --- SACCH (SDCCH/4 SS0) DL : SI5(0x1d)/SI6(0x1e) REELS -> feed_sacch ---
    # Slots SACCH SS0 (combine CCCH+SDCCH/4) : fn%51 in {42-45}. grgsm donne le
    # bloc 23o = [L1 hdr 2][LAPDm: 03 03 len 06 mt L3...]. On detecte le RR header
    # "06 1d"/"06 1e" et on forwarde le bloc TEL QUEL au shunt (sub_type 0x87 =
    # SDCCH4|ACCH) -> feed_sacch REEL, remplace la fabrication SI3->SI6.
    # ⚠️ fn%102 et NON fn%51 : en 05.02 les SACCH de SS0/SS1 sont dans la
    # premiere moitie du cycle 102 et celles de SS2/SS3 dans la seconde
    # (42,46,93,97). Teste modulo 51, {42..45} acceptait AUSSI la SACCH de SS2
    # (93-96 % 51 = 42-45) : on livrait la SACCH d'une autre sous-voie comme si
    # c'etait la notre, et on ne voyait jamais celles de SS1/SS3.
    if len(by) >= 8 and _b_sa is not None and _b_sa <= (sd_fn % 102) <= _b_sa + 3:
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

# =============================================================================
# ARMEMENT DU TCH : ASSIGNMENT COMMAND -> calypso_tch_cfg -> 3e grgsm_decode
# =============================================================================
# [2026-08-11] RECONSTRUIT. Cette logique existait (08/08) et a disparu de
# l'arbre le 11/08. Sans elle la panne est en cascade et SILENCIEUSE :
#   calypso_tch_cfg reste a 0 octet -> aucun `grgsm_decode -m TCHF` -> aucun
#   GSMTAP dedie -> feed_tch_sacch JAMAIS appele -> la SACCH du TCH n'est pas
#   alimentee -> le MS y lit le SI5/SI6 du camp (a_cd est PARTAGE camp<->TCH,
#   avec DEUX conventions d'en-tete) -> "Short header message type 0x07
#   unsupported" une fois par seconde, a partir de ~30 s apres l'ASSIGNMENT.
#
# CONTRAT DU FICHIER — lu chez son CONSOMMATEUR (qemu_wrap.c
# calypso_tch_cfg_read), jamais devine :
#   0..3 seq uint32 LE (≠0)   4 TN   5 TSC   6..7 ARFCN uint16 LE   (16 o)
# Le seq est ecrit EN DERNIER : publication atomique, le lecteur ne voit jamais
# un demi-enregistrement.
#
# JUGES : calypso_tch_cfg non vide ; un process `-m TCHF` vivant ;
#         `feed_tch_sacch` > 0 dans qemu.log ; `Short header 0x07` = 0.
TCH_CFG_PATH = "/dev/shm/calypso_tch_cfg"
TCH_FIFO     = os.environ.get("CALYPSO_TCH_FIFO", "/tmp/iq_grgsm_tch.fifo")
TCH_SPEECH   = "/dev/shm/calypso_tch_speech.gsm"
# --- TCH DL CHIFFRE (design B, comme le SDCCH) : un 2e decodeur -e/-k tourne en
#     parallele du clair JAMAIS tue, sur SA propre fifo IQ dediee (relay-fed).
#     Le clair reste vivant : un Kc faux ne detruit alors que le flux chiffre,
#     pas la voix en clair (le TCHF de gr-gsm est un if/else EXCLUSIF). ---------
TCH_CIPH_FIFO   = os.environ.get("CALYPSO_TCH_CIPH_FIFO", "/tmp/iq_grgsm_tch_ciph.fifo")
TCH_SPEECH_CIPH = "/dev/shm/calypso_tch_speech_ciph.gsm"
_tch_proc = None
_tch_cur  = None          # (tn, tsc, arfcn) actuellement arme
_tch_seq  = 0
_tch_ciph_proc    = None
_tch_ciph_applied = (0, None, None)   # (algo, kc_hex, (tn,tsc,arfcn)) applique au decodeur chiffre

def _tch_write_cfg(tn, tsc, arfcn):
    """Publie TN/TSC/ARFCN. seq en dernier = publication atomique."""
    global _tch_seq
    _tch_seq += 1
    buf = bytearray(16)
    buf[4] = tn & 0xff
    buf[5] = tsc & 0xff
    buf[6:8] = int(arfcn).to_bytes(2, "little")
    buf[0:4] = _tch_seq.to_bytes(4, "little")
    fd = os.open(TCH_CFG_PATH, os.O_CREAT | os.O_WRONLY, 0o644)
    try:
        os.pwrite(fd, bytes(buf), 0)
    finally:
        os.close(fd)

# ---- TAILEUR VOIX DL : calypso_tch_speech.gsm -> /dev/shm/calypso_tch_dl ----
#
# CHAINON MANQUANT, retabli le 12/08/2026. `grgsm_decode -d FR -o` ecrit la voix
# descendante decodee en trames GSM-FR brutes de 33 o (verifie : taille toujours
# multiple de 33, nibble haut du 1er octet = 0xD sur 200/200 trames, debit
# 1650 o/s = exactement 50 trames/s). Le shunt QEMU, lui, lit un ANNEAU sideband.
# Entre les deux il faut un taileur — il n'existait plus nulle part : mesure du
# 12/08, appel ETABLI (`Call is answered`), `calypso_tch_speech.gsm` a +1650 o/s
# et `/dev/shm/calypso_tch_dl` a **0 octet**. Aucun son possible, quelle que
# soit la qualite du decodage en amont.
#
# FORMAT — contrat de `calypso_tch_dl_poll()` (calypso_dsp_shunt.c:983-1137) :
#   entete 8 o : w_seq u32 LE @0 | n_slots u32 LE @4
#   puis n_slots x 48 o ; slot du seq k = (k-1) % n_slots
#   slot 48 o  : seq u32 LE @0 | fn u32 LE @4 | fr[33] @8
#
# ⚠️ ORDRE D'ECRITURE OBLIGATOIRE : le SLOT d'abord, w_seq ENSUITE. Le lecteur
#    se protege deja (`sseq != next` -> il repassera), mais annoncer un seq dont
#    le slot n'est pas ecrit lui ferait voir une trame a moitie remplie.
# ⚠️ n_slots DOIT valoir 16 : hors de ]0,4096] le shunt retombe sur l'ancien
#    format « slot unique », celui qui perdait 25 % des trames (mesure du 08/08).
# ⚠️ Le champ `fn` est laisse a 0 : le fichier de gr-gsm ne porte AUCUN numero de
#    trame. Y ecrire un compteur maison donnerait un nombre qui RESSEMBLE a un FN
#    sans en etre un ; le shunt ne s'en sert pas.
TCH_DL_PATH  = "/dev/shm/calypso_tch_dl"
TCH_DL_SLOTS = 16          # = TCH_DL_RING_N du shunt, ne pas desynchroniser
TCH_DL_SLOT  = 48
FR_BYTES     = 33

# Source voix DL ACTIVE, pilotee par le watcher Kc (sync_tch_ciph) : le fichier
# -o du decodeur CLAIR (TCH_SPEECH) tant que le DL est en clair, celui du
# decodeur CHIFFRE (TCH_SPEECH_CIPH) des qu'un Kc est applique. Les deux ne
# produisent JAMAIS de voix en meme temps (le CRC de gr-gsm ne laisse passer que
# le flux du bon algo) -> un seul ecrivain de l'anneau, jamais de course sur w_seq.
_tch_active_speech = [TCH_SPEECH]

def _tch_dl_tailer():
    """Publie chaque nouvelle trame FR de TCH_SPEECH dans l'anneau sideband."""
    fdw = os.open(TCH_DL_PATH, os.O_CREAT | os.O_RDWR, 0o644)
    os.ftruncate(fdw, 8 + TCH_DL_SLOTS * TCH_DL_SLOT)
    os.pwrite(fdw, (0).to_bytes(4, "little")
                 + TCH_DL_SLOTS.to_bytes(4, "little"), 0)
    print("[si-bridge] taileur voix DL arme : %s -> %s (anneau %d x %d o)"
          % (TCH_SPEECH, TCH_DL_PATH, TCH_DL_SLOTS, TCH_DL_SLOT), flush=True)
    seq = 0
    fdr = None
    off = 0
    rest = b""
    npub = 0
    cur_path = _tch_active_speech[0]
    while True:
        try:
            want_path = _tch_active_speech[0]
            if want_path != cur_path:
                # bascule clair<->chiffre : on ferme et on repart du fichier
                # ACTIF depuis sa FIN (le retard accumule est du passe, pas de la
                # voix a jouer). Le seq de l'anneau reste monotone.
                if fdr is not None:
                    try: os.close(fdr)
                    except Exception: pass
                fdr = None
                cur_path = want_path
                rest = b""
            if fdr is None:
                try:
                    fdr = os.open(cur_path, os.O_RDONLY)
                except OSError:
                    time.sleep(0.2)
                    continue
                # On repart de la FIN du fichier. Le retard deja accumule n'est
                # pas de la voix a jouer, c'est du passe : le publier d'un coup
                # deborderait l'anneau et ne produirait qu'une rafale.
                off = os.fstat(fdr).st_size
                rest = b""
            st = os.fstat(fdr)
            if st.st_size < off:            # decodeur relance -> fichier tronque
                off = 0
                rest = b""
            if st.st_size == off:
                time.sleep(0.005)
                continue
            # Jamais plus d'un anneau d'un coup : si on a pris du retard, c'est
            # au shunt de compter le trou (il le fait, TCH-DL DEBORDEMENT), pas
            # a nous de le noyer.
            want = min(st.st_size - off, TCH_DL_SLOTS * FR_BYTES)
            data = os.pread(fdr, want, off)
            if not data:
                time.sleep(0.005)
                continue
            off += len(data)
            rest += data
            while len(rest) >= FR_BYTES:
                fr, rest = rest[:FR_BYTES], rest[FR_BYTES:]
                seq += 1
                rec = (seq.to_bytes(4, "little") + b"\x00\x00\x00\x00" + fr
                       + b"\x00" * (TCH_DL_SLOT - 8 - FR_BYTES))
                os.pwrite(fdw, rec, 8 + ((seq - 1) % TCH_DL_SLOTS) * TCH_DL_SLOT)
                os.pwrite(fdw, seq.to_bytes(4, "little"), 0)   # w_seq EN DERNIER
                npub += 1
                if npub <= 3 or npub % 250 == 0:
                    print("[si-bridge] TCH-DL : %d trames FR publiees (w_seq=%d)"
                          % (npub, seq), flush=True)
        except Exception as e:
            print("[si-bridge] taileur voix DL interrompu : %s" % e, flush=True)
            if fdr is not None:
                try: os.close(fdr)
                except Exception: pass
                fdr = None
            time.sleep(0.5)

threading.Thread(target=_tch_dl_tailer, daemon=True, name="tch-dl-tailer").start()

def make_tch_binary():
    """Copie patchee de grgsm_decode pour le TCH — DEUX ports a corriger.

    grgsm_decode code en dur 127.0.0.1:4729 DEUX fois (l.122-123) :
      - l.122 UDP_SERVER : ne sert QU'A eviter les ICMP port-unreachable. Deux
        instances -> « bind: Address already in use » -> la 2e MEURT. C'est ce
        qui faisait mourir le decodeur TCH aussitot lance (14 reamorcages
        mesures le 11/08), donc feed_tch_sacch = 0, donc SACCH dediee muette.
      - l.123 UDP_CLIENT : c'est la VRAIE sortie GSMTAP. En mode TCHF,
        tch_f_decoder ET cch_decoder y publient (l.213-214).
    On bind donc le serveur sur un port libre, et on pointe le client droit sur
    4730, que le shunt ecoute : il dispatche par sub_type — FACCH/F (0x09) ->
    feed_facch, SACCH du dedie (0x89) -> feed_tch_sacch. ⚠️ Ces deux valeurs
    sont celles de libosmocore/gr-gsm (GSMTAP_CHANNEL_FACCH_F = 0x09) ; le shunt
    les comparait a 0x08/0x88 jusqu'au 12/08, ce qui rendait les deux feeds
    inatteignables (0x08 est en fait SDCCH/8).
    Pas de parsing de stdout : les donnees vont du decodeur au shunt en direct.
    ⚠️ Remplacements CIBLES ligne par ligne : un `replace("4729","4730")` global
    ferait binder le serveur sur 4730 et entrerait en conflit avec le shunt.
    """
    src = shutil.which("grgsm_decode")
    if not src:
        return "grgsm_decode"
    dst = "/tmp/grgsm_decode_tch"
    try:
        with open(src) as f:
            code = f.read()
        a = '"UDP_SERVER", "127.0.0.1", "4729"'
        b = '"UDP_CLIENT", "127.0.0.1", "4729"'
        if a not in code or b not in code:
            print("[si-bridge] grgsm TCH : motifs de port introuvables -> "
                  "fallback non patche (conflit 4729 probable)", flush=True)
            return src
        code = code.replace(a, '"UDP_SERVER", "127.0.0.1", "4739"')
        code = code.replace(b, '"UDP_CLIENT", "127.0.0.1", "4730"')
        with open(dst, "w") as f:
            f.write(code)
        os.chmod(dst, 0o755)
        print("[si-bridge] grgsm TCH patche -> %s (bind 4739, GSMTAP -> 4730)" % dst,
              flush=True)
        return dst
    except Exception as e:
        print("[si-bridge] patch grgsm TCH echoue (%s)" % e, flush=True)
        return src
GRGSM_TCH = make_tch_binary()

def make_tch_ciph_binary():
    """Copie patchee pour le TCH CHIFFRE (design B). Identique a make_tch_binary
    mais le serveur UDP bind un port ENCORE different (4749) : le clair tient deja
    4739 ; deux binds sur le meme port -> le 2e MEURT (silencieux). Le client
    GSMTAP pointe le MEME 4730 que le clair -> le shunt recoit la FACCH(0x09) et
    la SACCH(0x89) DECHIFFREES exactement comme celles du clair. Les deux
    decodeurs ne sortent jamais de trame CRC-valide en meme temps (algo different),
    donc aucune collision de contenu sur 4730 ni sur l'anneau voix."""
    src = shutil.which("grgsm_decode")
    if not src:
        return "grgsm_decode"
    dst = "/tmp/grgsm_decode_tch_ciph"
    try:
        with open(src) as f:
            code = f.read()
        a = '"UDP_SERVER", "127.0.0.1", "4729"'
        b = '"UDP_CLIENT", "127.0.0.1", "4729"'
        if a not in code or b not in code:
            print("[si-bridge] grgsm TCH chiffre : motifs de port introuvables -> "
                  "fallback non patche (conflit de bind probable)", flush=True)
            return src
        code = code.replace(a, '"UDP_SERVER", "127.0.0.1", "4749"')
        code = code.replace(b, '"UDP_CLIENT", "127.0.0.1", "4730"')
        with open(dst, "w") as f:
            f.write(code)
        os.chmod(dst, 0o755)
        print("[si-bridge] grgsm TCH chiffre patche -> %s (bind 4749, GSMTAP -> 4730)" % dst,
              flush=True)
        return dst
    except Exception as e:
        print("[si-bridge] patch grgsm TCH chiffre echoue (%s)" % e, flush=True)
        return src
GRGSM_TCH_CIPH = make_tch_ciph_binary()

def _tch_spawn(tn, arfcn):
    """3e decodeur, sur le tube IQ dedie (alimente par CALYPSO_RELAY_FIFOS)."""
    cmd = [GRGSM_TCH, "-m", "TCHF", "-t", str(tn), "-a", str(arfcn),
           "-c", TCH_FIFO, "-s", "1083333", "-v", "-d", "FR", "-o", TCH_SPEECH]
    try:
        err = open("/tmp/osmo-nitb/logs/grgsm_decode_tch.log", "w")
    except Exception:
        err = subprocess.DEVNULL
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=err, text=True)

def arm_tch(tn, tsc, arfcn):
    global _tch_proc, _tch_cur
    cfg = (tn, tsc, arfcn)
    vivant = _tch_proc is not None and _tch_proc.poll() is None
    if cfg == _tch_cur and vivant:
        return                                   # deja arme sur cette config
    if _tch_proc is not None and vivant:
        try: _tch_proc.terminate()
        except Exception: pass
    _tch_write_cfg(tn, tsc, arfcn)
    _tch_proc = _tch_spawn(tn, arfcn)
    _tch_cur = cfg
    print("[si-bridge] TCH arme : TN=%d TSC=%d ARFCN=%d -> %s + grgsm -m TCHF (pid %s)"
          % (tn, tsc, arfcn, TCH_CFG_PATH, _tch_proc.pid), flush=True)

def _tch_ciph_spawn(tn, arfcn, algo, kc_hex):
    """Decodeur TCH DL CHIFFRE : meme -m TCHF que le clair, sur SA fifo IQ dediee
    (TCH_CIPH_FIFO, relay-fed), avec -e/-k et une sortie voix SEPAREE."""
    cmd = [GRGSM_TCH_CIPH, "-m", "TCHF", "-t", str(tn), "-a", str(arfcn),
           "-c", TCH_CIPH_FIFO, "-s", "1083333", "-v", "-d", "FR",
           "-o", TCH_SPEECH_CIPH, "-e", str(algo), "-k", kc_hex]
    try:
        err = open("/tmp/osmo-nitb/logs/grgsm_decode_tch_ciph.log", "w")
    except Exception:
        err = subprocess.DEVNULL
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=err, text=True)

def sync_tch_ciph(seq, algo, kc):
    """Aligne le decodeur TCH chiffre sur (Kc present) ET (TCH arme). Appele par
    le watcher Kc (toutes les 0,5 s), JAMAIS par l'ASSIGNMENT COMMAND : a
    l'instant de l'assignation le Kc vient d'etre efface par le DM_EST_REQ que le
    firmware emet a chaque activation de canal dedie (osmocon l'efface) ->
    read_kc() rendrait 0. Le decouplage par le watcher — exactement comme le grgsm
    chiffre du SDCCH — resout ce piege. Le decodeur CLAIR (arm_tch) n'est JAMAIS
    tue ici."""
    global _tch_ciph_proc, _tch_ciph_applied
    cfg = _tch_cur
    want = (algo, kc, cfg) if (seq and algo in (1, 2, 3) and cfg is not None) else (0, None, None)
    if want == _tch_ciph_applied:
        return
    p = _tch_ciph_proc
    if p is not None:
        try: p.kill(); p.wait(timeout=2)
        except Exception: pass
        _tch_ciph_proc = None
    _tch_ciph_applied = want
    if want[0]:
        tn, tsc, arfcn = cfg
        _tch_ciph_proc = _tch_ciph_spawn(tn, arfcn, want[0], want[1])
        _tch_active_speech[0] = TCH_SPEECH_CIPH     # voix DL active = flux chiffre
        print("[si-bridge] TCH CHIFFRE spawn : A5/%d kc=%s TN=%d ARFCN=%d -> %s "
              "(voix DL active=chiffre, pid %s)"
              % (want[0], want[1], tn, arfcn, TCH_CIPH_FIFO, _tch_ciph_proc.pid), flush=True)
    else:
        _tch_active_speech[0] = TCH_SPEECH          # voix DL active = flux clair
        print("[si-bridge] TCH chiffre arrete (Kc absent ou TCH non arme) -> "
              "voix DL active=clair", flush=True)

def check_assignment(by, fn):
    """Detecte un ASSIGNMENT COMMAND (RR PD=0x06, MT=0x2e) sur le SDCCH DL.

    Channel Description 2 (GSM 04.08 10.5.2.5), 3 octets, decodage aligne sur
    `struct gsm48_chan_desc` de libosmocore :
        b0 : chan_nr           -> TN = b0 & 0x07
        b1 : tsc(3) h(1) arfcn_high(2) spare(2)
             TSC = (b1>>5)&7   H = (b1>>4)&1   ARFCN_high = b1 & 0x03
        b2 : ARFCN_low         -> ARFCN = (ARFCN_high<<8) | b2   [si H=0]
    """
    for off in range(2, min(7, len(by) - 5)):
        if by[off] != 0x06 or by[off + 1] != 0x2e:
            continue
        b0, b1, b2 = by[off + 2], by[off + 3], by[off + 4]
        tn  = b0 & 0x07
        tsc = (b1 >> 5) & 0x07
        h   = (b1 >> 4) & 0x01
        if h:
            # ⚠️ REFUS EXPLICITE. Cette chaine ne gere PAS le saut de frequence
            # (cf. la meme limite cote injection UL). Fabriquer une ARFCN a
            # partir des champs MAIO/HSN donnerait un nombre FAUX en silence,
            # et le decodeur ecouterait a cote sans que rien ne le signale.
            print("[si-bridge] ASSIGNMENT COMMAND avec SAUT DE FREQUENCE (H=1) : "
                  "TCH NON arme (chaine sans hopping). FN=%d" % fn, flush=True)
            return
        arfcn = ((b1 & 0x03) << 8) | b2
        print("[si-bridge] ASSIGNMENT COMMAND FN=%d -> chan_nr=0x%02x TN=%d TSC=%d ARFCN=%d"
              % (fn, b0, tn, tsc, arfcn), flush=True)
        arm_tch(tn, tsc, arfcn)
        return

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
    sync_tch_ciph(seq, algo, kc)       # design B du TCH : suit (Kc present) ET (TCH arme)
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
