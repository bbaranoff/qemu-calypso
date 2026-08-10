#!/bin/bash
# =============================================================================
#  run.sh — point d'entrée unique de la pile GSM (Calypso QEMU + Osmocom)
# =============================================================================
#
#  CE FICHIER EST COURT, ET C'EST VOULU.
#  L'ancien run.sh (2426 lignes, conservé en run.sh.legacy) mélangeait le menu,
#  la résolution de configuration, le lancement de vingt processus et la mise en
#  page tmux. Un échec au milieu — osmocon absent, BTS sans horloge, mobile
#  jamais démarré — n'apparaissait nulle part : le script continuait.
#
#  Ici run.sh ne fait plus que quatre choses :
#    1. lire les options ;
#    2. charger la configuration (environnement/load.env) ;
#    3. construire un PLAN à partir de run_modules/ ;
#    4. l'exécuter en affichant un état par étape, et rendre un code de sortie.
#
#  Toute la logique métier vit dans run_modules/NN-<slug>.sh, un module par
#  étape, tous conformes au contrat décrit dans run_modules/_lib/mod.sh.
#
#  CHAÎNE DE CONFIGURATION — NE PAS LA CASSER
#      VAR=x ./run.sh          ← la ligne de commande gagne toujours
#        -> environnement/load.env  (sourcé `set -a`, donc exporté vers QEMU)
#           -> paths / modes / domaines / debug / fixes / crutches / hérité
#  Vérifier ce qui s'applique réellement : grep "calypso-manifest" sur le log.
# -----------------------------------------------------------------------------
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODDIR="$HERE/run_modules"

# --- options ------------------------------------------------------------------
DRY=0 ONLY="" SKIP="" VERBOSE=0 ACTION=start PROFILE="${CALYPSO_PROFILE:-hybrid}" FORCE=0
# Attachement tmux en fin de sequence : actif par defaut, refusable par
# --no-attach ou CALYPSO_NO_ATTACH=1 (utile quand on enchaine des commandes).
NO_ATTACH="${CALYPSO_NO_ATTACH:-0}"

usage() {
    cat <<'USAGE'
Usage : ./run.sh [options]

  --list              affiche le plan et sort, sans rien lancer
  --dry-run           déroule le plan sans effet de bord
  --only  <slugs>     ne joue que ces modules (séparés par des virgules)
  --skip  <slugs>     saute ces modules
  --profile <nom>     calypso (défaut) | faketrx | hybrid | core
  --stop              arrête la pile (plan en ordre inverse)
  --status            interroge l'état de chaque module
  --force             relance même les modules déjà démarrés
  --no-attach         ne pas s'attacher à tmux à la fin
  --verbose           montre la sortie des modules
  --check-paths       vérifie que les dépendances déclarées existent
  --restart           = --reset puis démarrage — la relance sûre au quotidien
  --reset             repart d'un état propre : arrête la pile, TUE le serveur
                      tmux (son environnement global fossilise les CALYPSO_*
                      du premier run), archive les journaux, purge sockets/FIFO
  -h, --help          cette aide

Toute variable CALYPSO_* passée en préfixe est transmise à QEMU :
  CALYPSO_MODE=native ./run.sh
USAGE
}

# =============================================================================
#  --configure — fixer le comportement PAR DÉFAUT, une fois pour toutes
# =============================================================================
#  Écrit environnement/local.env, que load.env charge EN PREMIER. Comme tous les
#  fichiers de configuration utilisent l'idiome « := » (le premier qui pose une
#  valeur gagne), ce fichier l'emporte sur les défauts du dépôt — mais JAMAIS sur
#  la ligne de commande, qui est déjà dans l'environnement quand il est lu.
#
#      ligne de commande  >  local.env (ce fichier)  >  défauts du dépôt
#
#  Rien n'est lancé : on répond aux questions, on relance ./run.sh ensuite.
# -----------------------------------------------------------------------------
_cfg_demander() {   # _cfg_demander <invite> <defaut> <choix...>
    local invite="$1" defaut="$2"; shift 2
    local choix=("$@") i rep
    printf '\n%s\n' "$invite" >&2
    for i in "${!choix[@]}"; do
        printf '   %d) %s%s\n' "$((i+1))" "${choix[$i]}" \
               "$([ "${choix[$i]}" = "$defaut" ] && printf '   [defaut]')" >&2
    done
    printf '   choix [%s] : ' "$defaut" >&2
    read -r rep </dev/tty 2>/dev/null || rep=""
    [ -z "$rep" ] && { printf '%s' "$defaut"; return; }
    case "$rep" in
        (*[!0-9]*|"") printf '%s' "$rep" ;;                  # valeur tapée telle quelle
        (*) [ "$rep" -ge 1 ] && [ "$rep" -le "${#choix[@]}" ] \
                && printf '%s' "${choix[$((rep-1))]}" || printf '%s' "$defaut" ;;
    esac
}

_configure() {
    local dest="${QEMU_TREE:-.}/environnement/local.env"
    printf '\nConfiguration des valeurs par défaut — rien ne sera lancé.\n' >&2
    printf 'Destination : %s\n' "$dest" >&2

    local profil pipeline mode langue attache
    profil="$(_cfg_demander   'Profil — quels composants lancer ?' \
                              "${CALYPSO_PROFILE:-hybrid}" \
                              hybrid calypso faketrx core)"
    mode="$(_cfg_demander     'Mode d émulation du DSP' \
                              "${CALYPSO_MODE:-shunt_legit}" \
                              empty none bare shunt_legit shunt_legit_no_inject native native_twl native_twl_host_demod native_helped)"
    pipeline="$(_cfg_demander 'Chaîne radio' \
                              "${CALYPSO_PIPELINE:-full-grgsm}" \
                              full-grgsm full shunt shunt-ipc bridge bare free)"
    langue="$(_cfg_demander   'Langue des messages' \
                              "${CALYPSO_LANG:-en}" \
                              en fr de es pt it hi zh ar)"
    attache="$(_cfg_demander  'Attacher la session tmux à la fin ?' oui oui non)"

    {
        printf '# environnement/local.env — écrit par « ./run.sh --configure ».\n'
        printf '#\n'
        printf '# Chargé EN PREMIER par load.env : avec l idiome « := », le premier qui pose\n'
        printf '# une valeur gagne, donc ce fichier l emporte sur les défauts du dépôt. La\n'
        printf '# ligne de commande le précède toujours (elle est déjà dans l environnement).\n'
        printf '#\n'
        printf '# Modifiable à la main, ou régénérable par « ./run.sh --configure ».\n'
        printf '\n'
        printf ': "${CALYPSO_PROFILE:=%s}"\n'  "$profil"
        printf ': "${CALYPSO_MODE:=%s}"\n'     "$mode"
        printf ': "${CALYPSO_PIPELINE:=%s}"\n' "$pipeline"
        printf ': "${CALYPSO_LANG:=%s}"\n'     "$langue"
        printf ': "${CALYPSO_NO_ATTACH:=%s}"\n' "$([ "$attache" = non ] && echo 1 || echo 0)"
    } > "$dest" || { printf 'écriture impossible : %s\n' "$dest" >&2; return 1; }

    printf '\nÉcrit :\n' >&2
    sed 's/^/   /' "$dest" >&2
    printf '\nLancez maintenant : ./run.sh\n\n' >&2
}

# -----------------------------------------------------------------------------
#  --reset — repartir d'un état propre, et surtout REPRODUCTIBLE
# -----------------------------------------------------------------------------
#  [2026-07-29] Motif : le serveur tmux hérite de l'environnement du PREMIER
#  « ./run.sh » qui l'a créé et le garde comme environnement global. Toute
#  relance faite depuis un pane hérite donc des CALYPSO_* de la ligne d'avant.
#  Vécu : « CALYPSO_MODE=native ./run.sh » rendait un manifeste portant
#  INJECT_SB=1, SHUNT_REAL_FB=1, FRAME_IT_NATIVE=1 — jamais tapées — et deux
#  runs de la MÊME commande divergeaient. Tuer le serveur est le seul moyen sûr
#  de repartir sans ce fossile.
#
#  Archive aussi les journaux au lieu de les laisser écraser : sans historique
#  on ne peut pas répondre à « ça marchait il y a cinq minutes ».
_reset() {
    local horodat rep n=0
    printf '\nRemise à zéro — rien ne sera relancé.\n\n' >&2

    # 1. arrêt propre via le plan existant (best-effort).
    printf '  arrêt de la pile…\n' >&2
    "$0" --stop >/dev/null 2>&1 || true

    # 2. archivage des journaux AVANT de toucher quoi que ce soit.
    local logdir="${LOG_DIR:-/root/calypso/logs}"
    if [ -d "$logdir" ] && [ -n "$(ls -A "$logdir" 2>/dev/null)" ]; then
        horodat="$(date +%Y%m%d-%H%M%S)"
        rep="$logdir/../archives/$horodat"
        mkdir -p "$rep" 2>/dev/null && \
        if cp -a "$logdir/." "$rep/" 2>/dev/null; then
            printf '  journaux archivés     %s\n' "$rep" >&2
        fi

        # [2026-07-30] MENAGE. Sans ça, les archives s'empilent sans limite :
        # /tmp a été saturé DEUX FOIS le 30/07 (512 Mo pleins), avec pour
        # conséquences une compilation qui échoue (« No space left on device »
        # sur le .s du compilateur) et le module « gabarits » qui casse la
        # relance. Les journaux du banc vivent sur un tmpfs de 512 Mo partagé
        # avec /tmp système — mailbox.log seul monte à 230 ko/s, soit ~800 Mo/h.
        #
        # On cible UNIQUEMENT l'arborescence du banc. Jamais /tmp lui-même :
        # le compilateur, dpkg et le reste y écrivent aussi.
        #   CALYPSO_ARCHIVES_KEEP=N   nombre d'archives gardées (défaut 3)
        #   CALYPSO_ARCHIVE_MAX_MB=N  au-delà, le fichier est archivé TRONQUÉ à
        #                             sa TÊTE (défaut 32) — sur un emballement,
        #                             c'est le début qui porte la cause.
        local arcdir="$logdir/../archives"
        local keep="${CALYPSO_ARCHIVES_KEEP:-3}"
        local maxmb="${CALYPSO_ARCHIVE_MAX_MB:-32}"
        local f taille avant apres

        # a) tronquer dans l'archive qu'on vient de créer les fichiers énormes
        for f in "$rep"/*; do
            [ -f "$f" ] || continue
            taille=$(stat -c %s "$f" 2>/dev/null || echo 0)
            if [ "$taille" -gt $(( maxmb * 1024 * 1024 )) ]; then
                head -c $(( maxmb * 1024 * 1024 )) "$f" > "$f.tete" 2>/dev/null &&
                mv -f "$f.tete" "$f" 2>/dev/null &&
                printf '  archive tronquée      %s (%s Mo -> %s Mo, tête conservée)\n' \
                       "$(basename "$f")" "$(( taille / 1048576 ))" "$maxmb" >&2
            fi
        done

        # b) ne garder que les N archives les plus récentes
        if [ -d "$arcdir" ]; then
            avant=$(du -sm "$arcdir" 2>/dev/null | cut -f1)
            ls -1t "$arcdir" 2>/dev/null | tail -n +$(( keep + 1 )) | while read -r vieux; do
                [ -n "$vieux" ] && rm -rf -- "$arcdir/$vieux"
            done
            apres=$(du -sm "$arcdir" 2>/dev/null | cut -f1)
            [ "${avant:-0}" -gt "${apres:-0}" ] 2>/dev/null &&
                printf '  archives purgées      %s gardées (%s Mo -> %s Mo)\n' \
                       "$keep" "$avant" "$apres" >&2
        fi

        # c) repartir sur des journaux vides : ils viennent d'être archivés
        rm -f "$logdir"/*.log "$logdir"/*.cfile 2>/dev/null
        printf '  journaux remis à zéro %s\n' "$logdir" >&2
        df -h "$logdir" 2>/dev/null | tail -1 | \
            awk '{printf "  espace disponible     %s (%s utilisé)\n", $4, $5}' >&2
    fi

    # 3. le serveur tmux — c'est LUI qui fossilise l'environnement.
    if command -v tmux >/dev/null 2>&1 && tmux ls >/dev/null 2>&1; then
        tmux kill-server 2>/dev/null || true
        printf '  serveur tmux tué      (environnement global purgé)\n' >&2
    fi

    # 4. les traînards. On cible des PID, jamais « pkill -f » à l'aveugle :
    #    le motif attraperait le shell qui exécute ce script.
    local motifs="qemu-system-arm osmocon osmo-bts-trx osmo-trx-ipc \
calypso-ipc-device fake_trx trxcon grgsm_decode si_bridge.py qemu_bcch_grgsm"
    local m p pids
    for m in $motifs; do
        pids="$(pgrep -f -- "$m" 2>/dev/null || true)"
        for p in $pids; do
            [ "$p" = "$$" ] && continue
            [ "$p" = "${PPID:-0}" ] && continue
            kill -TERM "$p" 2>/dev/null && n=$((n + 1))
        done
    done
    [ "$n" -gt 0 ] && printf '  processus arrêtés     %d\n' "$n" >&2

    # 5. sockets, FIFO et segments partagés — uniquement des artefacts d'exécution.
    local art
    for art in /tmp/osmocom_l2 /tmp/ms2_l2 /tmp/iq_grgsm.fifo \
               /dev/shm/dsp_iq.fifo /dev/shm/calypso_rach \
               /dev/shm/calypso_sdcch_ul /dev/shm/calypso_tch_dl \
               /dev/shm/calypso_tch_facch_ul /dev/shm/calypso_tch_sacch_ul \
               /dev/shm/calypso_tch_cfg /dev/shm/calypso_tch_sacch_air /dev/shm/calypso_dcch_cfg /dev/shm/calypso_tch_speech.gsm \
               /tmp/iq_grgsm_tch.fifo /tmp/iq_grgsm_sdcch.fifo \
               /dev/shm/calypso_dsp_shunt; do
        [ -e "$art" ] && rm -f "$art" 2>/dev/null && printf '  supprimé              %s\n' "$art" >&2
    done

    printf '\nÉtat propre. Relancez, la ligne de commande fera foi :\n' >&2
    printf '   CALYPSO_MODE=native ./run.sh\n\n' >&2
}

while [ $# -gt 0 ]; do
    case "$1" in
        --list)        ACTION=list ;;
        --dry-run)     DRY=1 ;;
        --only)        ONLY="${2:-}"; shift ;;
        --skip)        SKIP="${2:-}"; shift ;;
        --profile)     PROFILE="${2:-}"; shift ;;
        --stop)        ACTION=stop ;;
        --status)      ACTION=status ;;
        --force)       FORCE=1 ;;
        --no-attach)   NO_ATTACH=1 ;;
        --verbose)     VERBOSE=1 ;;
        --check-paths) ACTION=checkpaths ;;
        --configure)   ACTION=configure ;;
        --reset)       ACTION=reset ;;
        --restart)     ACTION=restart ;;
        --lang)        CALYPSO_LANG="${2:-}"; export CALYPSO_LANG; shift ;;
        -h|--help)     usage; exit 0 ;;
        *) # `t()` vient de i18n.sh, sourcé APRÈS le parsing : sans ce repli le
           # message d'erreur est « t: command not found ». Corrigé 2026-07-29.
           if command -v t >/dev/null 2>&1; then
               printf '%s\n\n' "$(t opt_inconnue "$1")" >&2
           else
               printf 'option inconnue : %s\n\n' "$1" >&2
           fi
           usage >&2; exit 2 ;;
    esac
    shift
done

if [ "$ACTION" = configure ]; then _configure; exit $?; fi

# --- configuration ------------------------------------------------------------
if [ -f "$HERE/environnement/load.env" ]; then
    set -a; . "$HERE/environnement/load.env"; set +a
else
    printf 'configuration introuvable : %s\n' "$HERE/environnement/load.env" >&2
    exit 2
fi
if [ "$ACTION" = reset ]; then _reset; exit 0; fi

# --restart = --reset puis le démarrage normal. On ne ré-exécute PAS le script
# (ça relirait load.env dans un environnement déjà pollué) : on enchaîne dans
# le même processus, dont l'environnement est exactement la ligne de commande.
if [ "$ACTION" = restart ]; then _reset; ACTION=start; fi

LOGDIR="${LOG_DIR:-/root/calypso/logs}"
mkdir -p "$LOGDIR/mod" 2>/dev/null || true

# --- affichage ----------------------------------------------------------------
# En TTY on réécrit la ligne « [ .. ] » en place ; sinon on imprime deux lignes,
# pour qu'un fichier de log ou un pipe reste lisible (pas de \r parasite).
if [ -t 1 ] && [ -z "${NO_COLOR:-}" ]; then
    TTY=1; C_OK=$'\033[32m'; C_KO=$'\033[31m'; C_SK=$'\033[33m'; C_DIM=$'\033[2m'; C_Z=$'\033[0m'
else
    TTY=0; C_OK=""; C_KO=""; C_SK=""; C_DIM=""; C_Z=""
fi

say_begin() { if [ $TTY -eq 1 ]; then printf '[ %s.. %s] %s' "$C_DIM" "$C_Z" "$1"; else printf '[ .. ] %s\n' "$1"; fi; }
say_end()   { # $1=tag $2=couleur $3=libellé $4=détail
    if [ $TTY -eq 1 ]; then printf '\r\033[K'; fi
    # %-4s : les étiquettes traduites n'ont pas toutes la même longueur ; sans
    # ce gabarit la colonne des libellés se décale d'une ligne à l'autre.
    printf '[%s%-4s%s] %s' "$2" "$1" "$C_Z" "$3"
    [ -n "${4:-}" ] && printf ' %s(%s)%s' "$C_DIM" "$4" "$C_Z"
    printf '\n'
}

# --- découverte des modules ---------------------------------------------------
. "$MODDIR/_lib/mod.sh"
shopt -s nullglob
for f in "$MODDIR"/[0-9][0-9]-*.sh; do . "$f"; done
shopt -u nullglob

if [ ${#MOD_ORDER[@]} -eq 0 ]; then
    printf '%s\n' "$(t aucun_mod "$MODDIR")" >&2
    exit 2
fi

# --- sélection ----------------------------------------------------------------
in_csv() { case ",$1," in *",$2,"*) return 0;; esac; return 1; }
selected=()
for m in "${MOD_ORDER[@]}"; do
    [ -n "$ONLY" ] && { in_csv "$ONLY" "$m" || continue; }
# Les modules lisent le profil dans l'ENVIRONNEMENT (voir _lib/radio.sh, qui
# décale le plan d'adressage du side-car en profil « hybrid »). La variable
# locale de run.sh ne leur parviendrait pas.
export CALYPSO_PROFILE="$PROFILE"

    [ -n "$SKIP" ] && { in_csv "$SKIP" "$m" && continue; }
    case " ${MOD_PROFILES[$m]} " in *" $PROFILE "*) ;; *) continue;; esac
    selected+=("$m")
done

# --- actions simples ----------------------------------------------------------
if [ "$ACTION" = list ]; then
    printf '%s\n\n' "$(t plan "$PROFILE" "${#selected[@]}")"
    for m in "${selected[@]}"; do
        printf '  %-18s %-42s %s%s\n' "$m" "$(t_mod "$m")" \
            "$([ "${MOD_REQUIRED[$m]}" = 1 ] && t obligatoire || t optionnel)" \
            "$([ -n "${MOD_DEPS[$m]}" ] && printf '  %s' "$(t apres "${MOD_DEPS[$m]}")")"
    done
    exit 0
fi

if [ "$ACTION" = checkpaths ]; then
    rc=0
    for v in QEMU_BIN FIRMWARE_ELF DSP_PROM0 OSMOCON OSMOCOM_CFG; do
        p="${!v:-}"
        if [ -z "$p" ];      then say_end "$(t fail)" "$C_KO" "$v" "non défini"; rc=1
        elif [ -e "$p" ];    then say_end " $(t ok) " "$C_OK" "$v" "$p"
        else                      say_end "$(t fail)" "$C_KO" "$v" "introuvable : $p"; rc=1; fi
    done
    exit $rc
fi

# --- exécution ----------------------------------------------------------------
declare -A STATE=()
nb_ok=0 nb_fail=0 nb_skip=0 nb_warn=0
[ "$ACTION" = stop ] && { tmp=(); for ((i=${#selected[@]}-1;i>=0;i--)); do tmp+=("${selected[$i]}"); done; selected=("${tmp[@]}"); }

for m in "${selected[@]}"; do
    p="$(mod_prefix "$m")"
    log="${MOD_LOG[$m]:-$LOGDIR/mod/$m.log}"
    _MOD_REASON=""; _MOD_HINT=""

    if [ "$ACTION" = stop ]; then
        say_begin "$(t arret "$(t_mod "$m")")"
        declare -F "${p}_stop" >/dev/null && "${p}_stop" >>"$log" 2>&1
        say_end " $(t ok) " "$C_OK" "$(t arret "$(t_mod "$m")")"
        continue
    fi

    if [ "$ACTION" = status ]; then
        if declare -F "${p}_status" >/dev/null && "${p}_status" >>"$log" 2>&1; then
            say_end "$(t up)" "$C_OK" "$(t_mod "$m")"
        else
            say_end "$(t down)" "$C_SK" "$(t_mod "$m")"
        fi
        continue
    fi

    # 1. porte d'activation
    if ! eval "${MOD_ENABLED_IF[$m]}" 2>/dev/null; then
        say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "$(t desactive "${MOD_ENABLED_IF[$m]}")"
        STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue
    fi

    # 2. dépendances
    dep_ko=""
    for d in ${MOD_DEPS[$m]}; do
        case "${STATE[$d]:-absent}" in ok|skip) ;; *) dep_ko="$d"; break;; esac
    done
    if [ -n "$dep_ko" ]; then
        say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "dépendance non satisfaite : $dep_ko"
        STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue
    fi

    say_begin "$(t_mod "$m")"

    # 3. déjà démarré ?
    if [ $FORCE -eq 0 ] && declare -F "${p}_status" >/dev/null && "${p}_status" >>"$log" 2>&1; then
        say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "$(t deja)"
        STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue
    fi

    # 4. prérequis
    if declare -F "${p}_check" >/dev/null; then
        "${p}_check" >>"$log" 2>&1; rc=$?
        # En simulation, les dépendances n'ont PAS tourné : un prérequis qui
        # porte sur leur ÉTAT (socket L1CTL, PTY série, port en écoute) ne peut
        # pas être satisfait. L'échec est alors informatif — il décrit ce que le
        # module vérifiera pour de bon — et ne doit pas interrompre le plan.
        # Sans cette exception, --dry-run s'arrêtait au premier module dépendant
        # d'un service simulé et n'affichait jamais la fin de la séquence.
        if [ $rc -eq $MOD_RC_FAIL ] && [ $DRY -eq 1 ]; then
            say_end "$(t dry)" "$C_DIM" "$(t_mod "$m")" \
                    "$(t prereq_sim "${_MOD_REASON:-}")"
            STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue
        fi
        if [ $rc -eq $MOD_RC_FAIL ]; then
            say_end "$(t fail)" "$C_KO" "$(t_mod "$m")" "${_MOD_REASON:-$(t sans_raison)}"
            [ -n "$_MOD_HINT" ] && printf '       %s→ %s%s\n' "$C_DIM" "$_MOD_HINT" "$C_Z"
            printf '       %s%s%s\n' "$C_DIM" "$(t journal "$log")" "$C_Z"
            STATE[$m]=fail; nb_fail=$((nb_fail+1))
            [ "${MOD_REQUIRED[$m]}" = 1 ] && { printf '\n%s\n' "$(t interrompu)"; exit 1; }
            nb_warn=$((nb_warn+1)); continue
        elif [ $rc -eq $MOD_RC_SKIP ]; then
            say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "${_MOD_REASON:-}"
            STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue
        fi
    fi

    # 5. simulation
    if [ $DRY -eq 1 ] && [ "${MOD_PURE[$m]}" != 1 ]; then
        say_end " -- " "$C_DIM" "$(t_mod "$m")" "$(t simule)"
        STATE[$m]=ok; continue
    fi

    # 6. lancement
    "${p}_start" >>"$log" 2>&1; rc=$?
    case $rc in
        $MOD_RC_ALREADY) say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "${_MOD_REASON:-$(t deja)}"
                         STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue ;;
        $MOD_RC_SKIP)    say_end "$(t skip)" "$C_SK" "$(t_mod "$m")" "${_MOD_REASON:-}"
                         STATE[$m]=skip; nb_skip=$((nb_skip+1)); continue ;;
        $MOD_RC_FAIL)    say_end "$(t fail)" "$C_KO" "$(t_mod "$m")" "${_MOD_REASON:-$(t sans_raison)}"
                         [ -n "$_MOD_HINT" ] && printf '       %s→ %s%s\n' "$C_DIM" "$_MOD_HINT" "$C_Z"
                         printf '       %s%s%s\n' "$C_DIM" "$(t journal "$log")" "$C_Z"
                         STATE[$m]=fail; nb_fail=$((nb_fail+1))
                         [ "${MOD_REQUIRED[$m]}" = 1 ] && { printf '\n%s\n' "$(t interrompu)"; exit 1; }
                         nb_warn=$((nb_warn+1)); continue ;;
    esac

    # 7. barrière : démarré, mais prêt ?
    if declare -F "${p}_wait" >/dev/null; then
        if ! "${p}_wait" >>"$log" 2>&1; then
            say_end "$(t fail)" "$C_KO" "$(t_mod "$m")" "$(t pas_pret "${_MOD_REASON:-délai dépassé}")"
            printf '       %s%s%s\n' "$C_DIM" "$(t journal "$log")" "$C_Z"
            STATE[$m]=fail; nb_fail=$((nb_fail+1))
            [ "${MOD_REQUIRED[$m]}" = 1 ] && { printf '\n%s\n' "$(t interrompu)"; exit 1; }
            nb_warn=$((nb_warn+1)); continue
        fi
    fi

    say_end " $(t ok) " "$C_OK" "$(t_mod "$m")"
    STATE[$m]=ok; nb_ok=$((nb_ok+1))
done

printf '\n%s\n' "$(t bilan "$nb_ok" "$nb_skip" "$nb_fail")"

# --- épilogue : où est passée la pile, et comment la reprendre en main ---------
# Le moteur rend la main au lieu de s'attacher lui-même à tmux : on peut donc
# l'utiliser dans un script ou une CI. Mais il faut dire à l'opérateur où aller.
if [ "$ACTION" = start ] && [ $DRY -eq 0 ] && [ $nb_fail -eq 0 ]; then
    _sess="${TMUX_SESSION:-calypso}"
    printf '\n'
    if tmux has-session -t "$_sess" 2>/dev/null; then
        printf '  %s%s%s   tmux attach -t %s\n' "$C_OK" "$(t connecter)" "$C_Z" "$_sess"
    else
        printf '  %s%s%s   %s\n' "$C_DIM" "$(t connecter)" "$C_Z" "$(t pas_session "$_sess")"
    fi
    printf '  %s%s%s       %s\n' "$C_DIM" "$(t journaux)" "$C_Z" "$LOGDIR"
    printf '  %s%s%s           ./run.sh --status\n' "$C_DIM" "$(t etat)" "$C_Z"
    printf '  %s%s%s        ./run.sh --stop\n' "$C_DIM" "$(t arreter)" "$C_Z"
    printf '\n'

    # Attachement automatique. On remplace le processus (exec) : le shell rend la
    # main directement a tmux, et quitter la session termine proprement.
    # Conditions : un terminal, aucun echec, et l'attachement non refuse.
    if [ $TTY -eq 1 ] && [ "$NO_ATTACH" != 1 ] && [ -z "${TMUX:-}" ] \
       && tmux has-session -t "$_sess" 2>/dev/null; then
        printf '  %sconnexion a la session…  (Ctrl-b d pour detacher)%s\n\n' "$C_DIM" "$C_Z"
        exec tmux attach -t "$_sess"
    fi
fi

[ $nb_fail -gt 0 ] && exit 1
exit 0
