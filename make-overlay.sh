#!/bin/bash
# make-overlay.sh — back-port qemu-src (working tree, SOURCE OF TRUTH) into the
#   qemu-calypso overlay. Reverse of make-fork.sh (which does overlay -> fork).
#
#     genuine QEMU  +  qemu-calypso (overlay)   =  qemu-src (assembled working tree)
#     make-fork.sh    : overlay + genuine  ->  fork/working-tree
#     make-overlay.sh : working-tree       ->  overlay            (this script)
#
# Copies every file the overlay TRACKS (git ls-files) from qemu-src into the overlay,
# preserving layout. Files that live only in the overlay (backups, *.minimal) and all
# of vanilla QEMU are left untouched. Nothing is ever deleted.
#
# New Calypso files created in qemu-src (e.g. calypso.nu.env) are NOT auto-added:
# `git add` them in the overlay once, then this script keeps them in sync.
#
# Usage: ./make-overlay.sh [--dry-run] [OVERLAY_DIR] [SRC_DIR]
set -euo pipefail
DRY=0; [ "${1:-}" = "--dry-run" ] && { DRY=1; shift; }
OVERLAY="${1:-/opt/GSM/qemu-calypso}"
SRC="${2:-$(cd "$(dirname "$0")" && pwd)}"
[ -d "$OVERLAY/.git" ] || { echo "!! overlay git repo not found: $OVERLAY" >&2; exit 1; }
[ -f "$SRC/VERSION" ]  || { echo "!! qemu-src (VERSION) not found: $SRC" >&2; exit 1; }
echo "src (truth) : $SRC"
echo "overlay     : $OVERLAY"
[ "$DRY" = 1 ] && echo "mode        : DRY-RUN (no writes)"
echo "----"
sync=0; same=0; only=0
while IFS= read -r f; do
    if [ -f "$SRC/$f" ]; then
        if cmp -s "$SRC/$f" "$OVERLAY/$f" 2>/dev/null; then
            same=$((same+1))
        else
            echo "  sync  $f"
            if [ "$DRY" = 0 ]; then
                mkdir -p "$OVERLAY/$(dirname "$f")"
                cp -a "$SRC/$f" "$OVERLAY/$f"
            fi
            sync=$((sync+1))
        fi
    else
        only=$((only+1))
    fi
done < <(cd "$OVERLAY" && git ls-files)

# --- Passe 2 : auto-ajout du NOUVEL ARBRE Calypso ---------------------------
# Les repertoires Calypso de l'overlay sont 100% overlay (absents de QEMU vanilla).
# On ramene tout NOUVEAU fichier cree dans qemu-src sous ces racines (nouvelles
# sources calypso_*, docs unifies, MASTER/STATUS/TODO) et on le `git add` dans
# l'overlay pour qu'il soit suivi ensuite. Exclut artefacts de build et backups.
# Racines surchargées via CALYPSO_OVERLAY_ROOTS.
# [2026-07-29] Ajout des quatre racines nées du refactor. Elles sont 100 %
# Calypso (absentes de QEMU vanilla), donc éligibles à l'auto-ajout :
#   run_modules/    le lanceur modulaire (contrat check/status/start/wait/stop)
#   tmux_modules/   une disposition tmux par profil
#   environnement/  la configuration découpée par domaine
#   firmware/       l'image layer1 versionnée avec le dépôt
# Sans elles, 102 fichiers — tout le lanceur — restaient hors de l'overlay, donc
# hors image et hors ISO, sans aucun message. On n'ajoute QUE du Calypso : les
# répertoires vanilla (accel/, block/, target/…) doivent rester dehors, c'est le
# principe du montage overlay + genuine.
ADD_ROOTS="${CALYPSO_OVERLAY_ROOTS:-hw/arm/calypso tools/calypso-ipc-device run_modules tmux_modules environnement firmware}"
added=0
for root in $ADD_ROOTS; do
    [ -d "$SRC/$root" ] || continue
    while IFS= read -r f; do
        case "$f" in
            *.o|*.d|*.orig|*~|*.pyc|*.a|*.so|*.tmp|*.tmp.*|*.swp|*.rej) continue;;
            *.bak|*.bak.*|*.bak-*|*.bak_*|*preNoCell*|*preBRINT0*|*preFNPROBE*) continue;;
            */build/*|*/.git/*|*/meson-*|*/.ninja*) continue;;
        esac
        # deja tracke dans l'overlay -> gere par la passe 1, on saute
        if (cd "$OVERLAY" && git ls-files --error-unmatch "$f" >/dev/null 2>&1); then
            continue
        fi
        echo "  add   $f"
        if [ "$DRY" = 0 ]; then
            mkdir -p "$OVERLAY/$(dirname "$f")"
            cp -a "$SRC/$f" "$OVERLAY/$f"
            (cd "$OVERLAY" && git add "$f")
        fi
        added=$((added+1))
    done < <(cd "$SRC" && find "$root" -type f | sort)
done

# --- Passe 2b : fichiers Calypso a la RACINE de qemu-src ---------------------
# Les env de modes (calypso.env socle + calypso_<mode>.env) vivent a la racine,
# hors ADD_ROOTS -> la passe 2 ne les voit pas. On les auto-ajoute ici (100%%
# Calypso, absents de QEMU vanilla). Globs surchargeables CALYPSO_OVERLAY_ROOT_GLOBS.
# [2026-07-27] RAPPORT_DFBDET.md ajoute : rapport d enquete indexe dans le README,
# il doit suivre l overlay comme run_results.md (sinon lien mort cote overlay).
# [2026-07-28] glob RAPPORT_*.md + PLAN_*.md : couvre RAPPORT_DFBDET/OPCODES/BRINT0/
# SYNTHESE et PLAN_APPLICATION sans re-editer ce script a chaque nouveau rapport.
# [2026-07-29] start-oqc.sh : point d'entrée du dépôt frère (osmo-nitb), appelé
# par la documentation ; il vivait à la racine sans être suivi par l'overlay.
# [2026-08-04] `tools/*.py` AJOUTE — les 11 outils d'analyse Calypso qui vivent a
# la racine de tools/ (corr_iq.py, tic54x-dis.py, mailbox-annote.py, fcch_*.py,
# gmsk_grgsm.py, iq_proxy.py, doppler.py, dsp_txt2bin.py, irda_capture.py,
# mailbox-gdb.py) n'etaient couverts par RIEN : ADD_ROOTS ne liste que
# `tools/calypso-ipc-device`, et la passe 1 ne voit que ce qui est deja tracke.
# Un outil NOUVEAU restait donc hors overlay, hors image et hors ISO **sans aucun
# message** — exactement le mode d'echec que la passe 2 avait ete ecrite pour
# supprimer (les 102 fichiers du lanceur, cf. le commentaire du 29/07).
#
# ⚠️ POURQUOI UN GLOB ET PAS UNE RACINE `tools`. `tools/` n'est PAS 100 % Calypso :
# il contient `ebpf/` et `i386/`, qui sont du QEMU vanilla. L'ajouter a ADD_ROOTS
# ferait entrer du vanilla dans l'overlay et casserait le principe overlay+genuine.
# Le glob ne prend que les `.py` de la RACINE de tools/, tous Calypso (verifie le
# 04/08 : 11 fichiers, aucun vanilla). Si un jour QEMU livre un `tools/*.py`
# vanilla, ce glob devra etre restreint a une liste explicite.
ROOT_GLOBS="${CALYPSO_OVERLAY_ROOT_GLOBS:-calypso*.env QUICK_START.md run_results.md RAPPORT_*.md PLAN_*.md start-oqc.sh TODO.md tools/*.py}"
for pat in $ROOT_GLOBS; do
    for f in $SRC/$pat; do
        [ -f "$f" ] || continue
        rel="${f#$SRC/}"
        if (cd "$OVERLAY" && git ls-files --error-unmatch "$rel" >/dev/null 2>&1); then
            continue   # deja tracke -> passe 1
        fi
        echo "  add   $rel"
        if [ "$DRY" = 0 ]; then
            # [2026-08-04] mkdir -p ajoute : depuis que ROOT_GLOBS accepte un
            # sous-repertoire (tools/*.py), la destination peut ne pas exister
            # dans l'overlay. Sans ca, `cp` echouait et `set -e` tuait le script.
            mkdir -p "$OVERLAY/$(dirname "$rel")"
            cp -a "$f" "$OVERLAY/$rel"
            (cd "$OVERLAY" && git add "$rel")
        fi
        added=$((added+1))
    done
done

echo "----"
if [ "$DRY" = 1 ]; then
    echo "$sync file(s) WOULD sync, $added new file(s) WOULD add, $same unchanged, $only overlay-only (untouched)"
    echo "re-run without --dry-run to apply."
else
    echo "$sync file(s) synced, $added new file(s) added (git add), $same unchanged, $only overlay-only (untouched)"
fi
echo "review: cd $OVERLAY && git status -s && git diff --stat"
