#!/bin/bash
# start-clean.sh -- lance le pipeline Calypso avec les overrides de calypso.env.
# Layout overlay/fork : ce script vit dans bash_scripts/, calypso.env à la racine.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
set -a
. "$ROOT/calypso.env"
set +a
exec "$ROOT/bash_scripts/run.sh" "$@"
