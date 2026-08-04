#!/bin/bash
# =============================================================================
#  start-clean.sh — point d'entrée historique, conservé pour compatibilité
# =============================================================================
#  Appelé par osmo_egprs/start-direct.sh (mode « qemu ») et par l'habitude.
#
#  Il ne fait plus rien lui-même : run.sh charge désormais la configuration
#  (environnement/load.env) et construit son plan. Ce fichier reste pour que la
#  chaîne existante continue de fonctionner sans modification en amont :
#
#      CALYPSO_SHUNT_LEGIT=1 ... ./start-clean.sh
#
#  Les variables passées en préfixe traversent jusqu'à QEMU, comme avant :
#  load.env n'utilise que `:=`, donc la ligne de commande gagne toujours.
# -----------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"
exec ./run.sh "$@"
