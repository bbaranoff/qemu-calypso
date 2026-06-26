#!/bin/bash
# start-clean.sh -- lance le pipeline Calypso avec les overrides de calypso.env
set -euo pipefail
cd "$(dirname "$0")"
set -a
. ./calypso.env
set +a
exec ./run.sh "$@"
