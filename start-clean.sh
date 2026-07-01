#!/bin/bash
# start-clean.sh — alias racine : délègue à bash_scripts/start-clean.sh
# (qui détecte ROOT, source calypso.env, exec bash_scripts/run.sh).
exec "$(cd "$(dirname "$0")" && pwd)/bash_scripts/start-clean.sh" "$@"
