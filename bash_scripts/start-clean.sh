#!/bin/bash
# start-clean.sh -- lance le pipeline Calypso avec les overrides de calypso.env.
# Layout overlay/fork : ce script vit dans bash_scripts/, calypso.env à la racine.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
set -a
. "$ROOT/calypso.env"
# [2026-07-11] LIGNE DE BASE NUE : override final qui retire toutes les forces/oracles
# (FORCE_IMR, FRAME_VEC28, SEED5AC8, ARM2DSP_CONT, BCTC_SM, FORCE_NB, NB_DATAIND,
#  FORCE_FBSB, F70_SETBIT1). Sourcé APRÈS calypso.env = dernier mot, gagne sur les doublons.
# Revenir à l'avant : supprimer/renommer calypso.nu.env.
if [ -f "$ROOT/calypso.nu.env" ]; then . "$ROOT/calypso.nu.env"; fi
set +a
exec "$ROOT/bash_scripts/run.sh" "$@"
