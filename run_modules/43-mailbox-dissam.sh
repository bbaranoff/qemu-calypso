# =============================================================================
#  37-mailbox-dissam — croisement mailbox × désassemblage, régénéré en continu
# =============================================================================
#  Le moniteur mailbox (calypso_mailbox.c) écrit $LOG_DIR/mailbox.log : chaque
#  accès ARM↔DSP, avec le PC de l'instruction responsable côté DSP. Seul, ce
#  journal dit « quelqu'un a écrit d_burst_d ». Croisé avec le désassembleur
#  (tools/tic54x-dis.py) il dit QUELLE instruction l'a écrit — et c'est ça qui
#  fait avancer le diagnostic :
#
#      0x0829  d_burst_d/rp0  DSP>WR  3082  0xb446  stl *AR1+, A
#                                                    ↑ post-incrément = memset
#                                                      de la read page, pas un bug
#
#  Ce module régénère le croisement pendant le run, pour qu'il soit consultable
#  sans rien lancer à la main. Lecture seule : il ne touche ni au modèle ni au
#  journal source.
# -----------------------------------------------------------------------------
MOD_REGISTER mailbox-dissam "Croisement mailbox × désassemblage c54x"
MOD_REQUIRED[mailbox-dissam]=0
MOD_DEPS[mailbox-dissam]="qemu"
MOD_PROFILES[mailbox-dissam]="calypso hybrid faketrx core"
MOD_TIMEOUT[mailbox-dissam]=15
MOD_ENABLED_IF[mailbox-dissam]='[ "${CALYPSO_MAILBOX:-0}" != 0 ]'

: "${CALYPSO_MAILBOX_DISSAM:=${QEMU_TREE}/tools/mailbox-annote.py}"
: "${CALYPSO_MAILBOX_DISSAM_OUT:=${LOG_DIR:-/root/calypso/logs}/mail_dissam.log}"
: "${CALYPSO_MAILBOX_DISSAM_PERIODE:=2}"     # secondes entre deux régénérations
: "${CALYPSO_MAILBOX_DISSAM_TOP:=60}"        # nb de lignes du tableau

# Motif = le SCRIPT de boucle, qui vit en permanence. Le python, lui,
# ne tourne que ~150 ms toutes les 2 s : le chercher avec pgrep rend
# « absent » neuf fois sur dix et fait croire à un module mort.
_mbxdis_pat() { printf '%s' "mailbox-dissam-boucle.sh"; }

mod_mailbox_dissam_check() {
    [ -r "$CALYPSO_MAILBOX_DISSAM" ] || {
        mod_hint "posez CALYPSO_MAILBOX_DISSAM=/chemin/vers/mailbox-annote.py"
        mod_skip "mailbox-annote.py absent : $CALYPSO_MAILBOX_DISSAM"
        return $MOD_RC_SKIP
    }
    command -v python3 >/dev/null 2>&1 || {
        mod_skip "python3 absent"
        return $MOD_RC_SKIP
    }
    mod_ok
}

mod_mailbox_dissam_status() {
    have_proc "$(_mbxdis_pat)" && return $MOD_RC_OK
    return $MOD_RC_FAIL
}

mod_mailbox_dissam_start() {
    local src="${LOG_DIR:-/root/calypso/logs}/mailbox.log"
    local out="$CALYPSO_MAILBOX_DISSAM_OUT"
    local log="${LOG_DIR:-/root/calypso/logs}/mod/mailbox-dissam.log"
    local boucle="${RUN_DIR:-/tmp/calypso}/mailbox-dissam-boucle.sh"

    mod_say "source   : $src"
    mod_say "sortie   : $out  (toutes les ${CALYPSO_MAILBOX_DISSAM_PERIODE}s)"

    # [2026-07-29] Premier passage SYNCHRONE. Deux raisons : la barrière trouve
    # le fichier sans attendre un cycle, et une erreur de l'outil apparaît ici,
    # dans le journal du module, au lieu de disparaître dans une boucle muette.
    # Version précédente : tout était dans le sous-shell détaché, le journal
    # restait à 0 octet et on ne pouvait pas savoir si la boucle avait démarré.
    if [ -s "$src" ]; then
        python3 "$CALYPSO_MAILBOX_DISSAM" "$src" --top "$CALYPSO_MAILBOX_DISSAM_TOP" \
            > "$out.tmp" 2>>"$log" && mv -f "$out.tmp" "$out" \
            || mod_say "ATTENTION : premier passage en échec, voir $log"
    else
        mod_say "mailbox.log encore vide — la boucle rattrapera"
    fi

    # La boucle vit dans un SCRIPT, pas dans un `bash -c` en ligne : on peut la
    # lire, la relancer à la main, et son nom est un motif pgrep fiable — alors
    # que le python ne tourne que ~150 ms toutes les 2 s et échappe donc à tout
    # `pgrep` fait entre deux cycles.
    cat > "$boucle" <<EOS
#!/bin/sh
# Régénère le croisement mailbox × désassemblage. Écrit par 37-mailbox-dissam.sh.
while :; do
    if [ -s "$src" ]; then
        python3 "$CALYPSO_MAILBOX_DISSAM" "$src" --top "$CALYPSO_MAILBOX_DISSAM_TOP" \
            > "$out.tmp" 2>>"$log" && mv -f "$out.tmp" "$out"
    fi
    sleep "$CALYPSO_MAILBOX_DISSAM_PERIODE"
done
EOS
    chmod +x "$boucle"
    setsid "$boucle" </dev/null >>"$log" 2>&1 &
    mod_say "boucle       : $boucle (PID $!)"

    mod_ok
}

# BARRIÈRE — on exige le FICHIER, pas un motif de journal : le premier passage
# peut tomber sur un mailbox.log encore vide, auquel cas on laisse le temps
# d'un cycle. (cf. la règle « barrières = observables ».)
mod_mailbox_dissam_wait() {
    wait_until "${MOD_TIMEOUT[mailbox-dissam]}" \
        "croisement $CALYPSO_MAILBOX_DISSAM_OUT" \
        test -s "$CALYPSO_MAILBOX_DISSAM_OUT"
}

mod_mailbox_dissam_stop() {
    # [2026-07-29] ⚠️ NE JAMAIS « pkill -f mailbox-annote » ici. Le motif figure
    # dans la ligne de commande du shell qui exécute le pkill, qui se tue donc
    # lui-même (SIGTERM, exit 143) — vécu deux fois dans la même journée, une
    # fois sur `pkill -f` générique et une fois ici. On cible des PID, et on
    # exclut explicitement soi-même et son parent.
    local motif pid n=0
    motif="mailbox-""annote.py"        # concaténé : n'apparaît pas tel quel ici
    for pid in $(pgrep -f -- "$motif" 2>/dev/null); do
        [ "$pid" = "$$" ] && continue
        [ "$pid" = "${PPID:-0}" ] && continue
        kill -TERM "$pid" 2>/dev/null && n=$((n + 1))
    done
    [ "$n" -gt 0 ] && mod_say "boucle arrêtée ($n processus)"
    mod_ok
}
