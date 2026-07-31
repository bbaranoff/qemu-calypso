# =============================================================================
#  40-qemu — l'émulateur Calypso (ARM7TDMI + DSP TMS320C54x)
# =============================================================================
MOD_REGISTER qemu "Émulateur Calypso (QEMU)"
MOD_REQUIRED[qemu]=1
MOD_DEPS[qemu]="prereqs"
MOD_PROFILES[qemu]="calypso hybrid"
MOD_TIMEOUT[qemu]=30

# Plafond du journal, en octets. Le modèle émet des sondes très bavardes : quand
# rien ne répond en face (pas de TRX, pas de BTS), le DSP boucle à vide et
# `POST-BOOTSTUB-RET` seul produit ~2,8 Mo/s — 337 Mo en deux minutes, mesuré.
# Un garde-fou tronque le fichier au-delà de cette taille, plutôt que de remplir
# le disque de lignes sans valeur de diagnostic.
: "${QEMU_LOG_MAX:=$((64 * 1024 * 1024))}"
# Tête figée du journal : ce que la troncature emporterait sinon.
: "${QEMU_LOG_HEAD:=$((8 * 1024 * 1024))}"

mod_qemu_check() {
    [ -x "${QEMU_BIN:-}" ] || { mod_fail "binaire QEMU absent : ${QEMU_BIN:-<non défini>}"
                                mod_hint "compilez-le : ./configure --target-list=arm-softmmu && ninja -C build qemu-system-arm"
                                return $MOD_RC_FAIL; }
    [ -r "${FIRMWARE_ELF:-}" ] || { mod_fail "firmware ARM introuvable : ${FIRMWARE_ELF:-<non défini>}"; return $MOD_RC_FAIL; }
    local r
    for r in "$DSP_PROM0" "$DSP_PROM1" "$DSP_PROM2" "$DSP_PROM3" "$DSP_DROM" "$DSP_PDROM"; do
        [ -r "$r" ] || { mod_fail "ROM du DSP manquante : $r"
                         mod_hint "les six binaires du DSP sont indispensables au démarrage"
                         return $MOD_RC_FAIL; }
    done
    mod_ok
}

mod_qemu_status() { have_proc "qemu-system-arm.*calypso"; }

# Le MANIFESTE est la seule source fiable de ce qui s'applique réellement (la
# ligne de commande ment : des variables en reposent d'autres). QEMU l'imprime
# au démarrage — donc au DÉBUT du journal, précisément ce que la troncature
# efface. On le recopie à part dès qu'il apparaît, une fois pour toutes.
_qemu_save_manifest() {
    local src="$1" dst="$2" i=0
    while [ $i -lt 60 ]; do
        if grep -q "calypso-manifest" "$src" 2>/dev/null; then
            grep "calypso-manifest" "$src" > "$dst" 2>/dev/null
            return 0
        fi
        sleep 1; i=$((i+1))
    done
    return 1
}

# Garde-fou de journal : quand le plafond est atteint, on repart d'un fichier
# vide plutôt que de remplir le disque. Le modèle émet des sondes très bavardes —
# quand rien ne répond en face, `POST-BOOTSTUB-RET` seul produit ~2,8 Mo/s,
# mesuré à 337 Mo en deux minutes. Le manifeste, lui, est déjà sauvegardé.
# _qemu_save_head <source> <dest> <octets> — fige les premiers octets du journal.
# Le garde-fou tronque par le DÉBUT ; sur un emballement (storm), le début est
# atteint en quelques secondes et c'est justement lui qu'on veut lire. On le
# recopie une fois, dès qu'il est disponible, et on ne le touche plus.
_qemu_save_head() {
    local src="$1" dst="$2" taille="$3" i=0
    while [ "$i" -lt 120 ]; do
        if [ -s "$src" ] && [ "$(stat -c %s "$src" 2>/dev/null || echo 0)" -ge "$taille" ]; then
            head -c "$taille" "$src" > "$dst" 2>/dev/null
            return 0
        fi
        sleep 1; i=$(( i + 1 ))
    done
    # Le journal n'a jamais atteint la taille voulue : on fige ce qu'il y a.
    [ -s "$src" ] && head -c "$taille" "$src" > "$dst" 2>/dev/null
    return 0
}

_qemu_log_guard() {
    local f="$1" max="$2" qpid="$3" n=0
    while kill -0 "$qpid" 2>/dev/null; do
        if [ -f "$f" ] && [ "$(stat -c %s "$f" 2>/dev/null || echo 0)" -gt "$max" ]; then
            n=$((n+1))
            printf '\n--- journal remis à zéro (%d fois) : plafond de %s octets atteint.\n--- Le manifeste de démarrage est conservé dans qemu-manifest.log ---\n' \
                   "$n" "$max" > "$f"
        fi
        sleep 5
    done
}

mod_qemu_start() {
    local mach="calypso"
    mach="$mach,dsp-prom0=$DSP_PROM0,dsp-prom1=$DSP_PROM1,dsp-prom2=$DSP_PROM2"
    mach="$mach,dsp-prom3=$DSP_PROM3,dsp-drom=$DSP_DROM,dsp-pdrom=$DSP_PDROM"
    mach="$mach,dsp-registers=$DSP_REGISTERS"
    local qlog="${LOG_DIR}/qemu.log"
    mod_say "machine  : $mach"
    mod_say "journal  : $qlog (plafond ${QEMU_LOG_MAX} o)"

    # [2026-07-30] Les drapeaux calcules par 08-accel.sh ont enfin un CONSOMMATEUR.
    # Avant : ce module codait tout en dur et 08-accel.sh journalisait
    # « ATTENTION : 40-qemu.sh ne lit pas encore ces drapeaux » — donc
    # CALYPSO_ICOUNT / CALYPSO_MTTCG / CALYPSO_QEMU_HALT apparaissaient au
    # manifeste sans AUCUN effet sur la ligne de commande. Verifie le 30/07 :
    # `CALYPSO_ICOUNT=auto` au manifeste, zero `-icount` dans les args du process.
    #
    # ⚠️ Le defaut du depot a ete remis a `off` dans calypso.env EN MEME TEMPS que
    # ce branchement : sinon tous les runs auraient recu `-icount auto` du jour au
    # lendemain. Dans ce projet un changement de cadence change le COMPORTEMENT de
    # la L1, pas seulement la vitesse (cf. le piege CALYPSO_DSP_YIELD=0, TODO §0quinquies).
    local -a xflags=()
    [ -n "${QEMU_ICOUNT_FLAG:-}" ] && xflags+=($QEMU_ICOUNT_FLAG)
    [ -n "${QEMU_ACCEL_FLAG:-}"  ] && xflags+=($QEMU_ACCEL_FLAG)
    [ -n "${QEMU_HALT_FLAG:-}"   ] && xflags+=($QEMU_HALT_FLAG)
    local gdbflag="${QEMU_GDB_FLAG:--gdb tcp::1234}"

    # Garde-fou : -icount et MTTCG sont incompatibles ; QEMU meurt au demarrage
    # avec un message que personne ne relie a sa cause. On le dit ici.
    if [ -n "${QEMU_ICOUNT_FLAG:-}" ] && [ -n "${QEMU_ACCEL_FLAG:-}" ]; then
        mod_hint "posez CALYPSO_ICOUNT=off, ou desactivez MTTCG — les deux ensemble tuent QEMU au demarrage"
        mod_fail "icount ($QEMU_ICOUNT_FLAG) et accel ($QEMU_ACCEL_FLAG) demandes ensemble"
        return $MOD_RC_FAIL
    fi
    if [ ${#xflags[@]} -gt 0 ]; then
        mod_say "drapeaux : ${xflags[*]}  (issus de 08-accel.sh, desormais APPLIQUES)"
    else
        mod_say "drapeaux : aucun (CALYPSO_ICOUNT=${CALYPSO_ICOUNT:-<vide>})"
    fi

    "$QEMU_BIN" -M "$mach" -cpu arm946 \
        "${xflags[@]}" $gdbflag -serial pty -serial pty \
        -monitor "unix:${RUN_DIR}/qemu-monitor.sock,server,nowait" \
        -kernel "$FIRMWARE_ELF" >>"$qlog" 2>&1 &
    local qpid=$!
    printf '%s\n' "$qpid" > "${RUN_DIR}/qemu.pid"
    _qemu_save_manifest "$qlog" "${LOG_DIR}/qemu-manifest.log" &
    _qemu_save_head     "$qlog" "${LOG_DIR}/qemu-tete.log" "${QEMU_LOG_HEAD:-8388608}" &
    _qemu_log_guard     "$qlog" "$QEMU_LOG_MAX" "$qpid" &
    mod_say "manifeste : ${LOG_DIR}/qemu-manifest.log"
    mod_ok
}

# BARRIÈRE — démarré n'est pas prêt.
# Le socket du moniteur seul est un critère trop faible : il existe dès que QEMU
# a ouvert son écoute, même si le DSP boucle à vide. On exige donc deux choses :
#   1. le moniteur RÉPOND (QEMU est vivant et sert son protocole) ;
#   2. le DSP a réellement PROGRESSÉ (le compteur d'instructions avance entre
#      deux relevés) — sinon le modèle est planté et rien ne le signalerait.
mod_qemu_wait() {
    local sock="${RUN_DIR}/qemu-monitor.sock" qlog="${LOG_DIR}/qemu.log"

    wait_until "${MOD_TIMEOUT[qemu]}" "socket du moniteur QEMU" have_unix "$sock" || return $MOD_RC_FAIL

    # Le processus est-il toujours là ? Un QEMU qui meurt à l'init laisse sa socket.
    local qpid; qpid="$(cat "${RUN_DIR}/qemu.pid" 2>/dev/null || echo 0)"
    if ! kill -0 "$qpid" 2>/dev/null; then
        mod_hint "regardez la fin de $qlog : QEMU s'est arrêté pendant l'initialisation"
        mod_fail "QEMU a démarré puis s'est arrêté"
        return $MOD_RC_FAIL
    fi

    # Le DSP progresse-t-il ? On compare la taille du journal à 1,5 s d'intervalle.
    # C'est grossier mais suffisant pour distinguer « vivant » de « figé », et ça
    # ne dépend d'aucune sonde particulière.
    local a b
    a="$(stat -c %s "$qlog" 2>/dev/null || echo 0)"
    sleep 1.5
    b="$(stat -c %s "$qlog" 2>/dev/null || echo 0)"
    if [ "$a" = "$b" ]; then
        mod_hint "le modèle ne produit aucune trace : vérifiez les ROM du DSP et le firmware"
        mod_fail "QEMU tourne mais le DSP semble figé"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_qemu_stop() {
    local qpid; qpid="$(cat "${RUN_DIR}/qemu.pid" 2>/dev/null || echo 0)"
    [ "$qpid" != 0 ] && kill "$qpid" 2>/dev/null
    pkill -f "qemu-system-arm.*calypso" 2>/dev/null
    rm -f "${RUN_DIR}/qemu.pid" "${RUN_DIR}/qemu-monitor.sock"
    return 0
}
