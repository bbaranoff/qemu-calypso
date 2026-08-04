#!/bin/bash
# =============================================================================
#  start-oqc.sh — point d'entrée d'osmo-qemu-calypso
# =============================================================================
#
#  C'est par ici qu'on lance la pile. Le nom est explicite pour qu'on le retrouve
#  dans un historique ou une liste de processus : « oqc » = osmo-qemu-calypso.
#
#      ./start-oqc.sh                    profil calypso (défaut)
#      ./start-oqc.sh --list             le plan, sans rien lancer
#      ./start-oqc.sh --dry-run          déroule sans effet de bord
#      ./start-oqc.sh --status           où en est chaque étape
#      ./start-oqc.sh --stop             arrête, en ordre inverse
#      ./start-oqc.sh --check-paths      vérifie les dépendances de la machine
#
#  Toute variable CALYPSO_* passée en préfixe traverse jusqu'à QEMU :
#
#      CALYPSO_MODE=native ./start-oqc.sh
#      CALYPSO_SHUNT_LEGIT=1 CALYPSO_SHUNT_NO_CANNED=1 ./start-oqc.sh
#
#  La configuration se règle dans environnement/ — commencez par paths.env, qui
#  dit où sont les dépendances sur VOTRE machine, puis modes.env pour le profil.
#  La vérité de ce qui s'applique est le manifeste imprimé au démarrage, jamais
#  la ligne de commande : certaines variables en reposent d'autres.
#
#  Ce fichier ne contient volontairement aucune logique : run.sh construit le
#  plan à partir de run_modules/ et l'exécute. Les points d'entrée historiques
#  (start-clean.sh, appelé par osmo_egprs/start-direct.sh) continuent de
#  fonctionner et aboutissent ici.
# -----------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"
exec ./run.sh "$@"
