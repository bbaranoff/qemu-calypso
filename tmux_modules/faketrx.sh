# =============================================================================
#  tmux_modules/faketrx.sh — disposition du profil « faketrx »
# =============================================================================
#  PAS de QEMU : le téléphone est entièrement logiciel (trxcon + mobile) et la
#  radio est un commutateur de bursts. Banc déterministe, sans modélisation I/Q
#  — c'est ce qui le rend rapide et reproductible.
#
#      1 radio   fake_trx · trxcon · BTS
#      2 coeur   MSC · BSC · HLR · MGW+STP
#      3 ms      la pile du téléphone · GSMTAP
#      4 voix    GAPK · passerelle SIP
#      5 shell
# -----------------------------------------------------------------------------
TMUX_FENETRE_PREMIERE=radio
TMUX_RESUME="5 fenêtres : radio · coeur · ms · voix · shell"

tmux_layout_premiere() {
    printf "tail -n 200 -F '%s/fake-trx.log' 2>/dev/null | stdbuf -oL tr -d '\\\\007' || sleep infinity" \
           "${LOG_DIR:-/root/calypso/logs}"
}

tmux_layout() {
    local L="${LOG_DIR:-/root/calypso/logs}"

    tmux select-pane -t "$TMUX_SESSION:radio" -T \
        "fake_trx | commutateur de bursts UDP - remplace modem et RF" 2>/dev/null
    _paint radio "$C_MS2"
    _split radio "trxcon | couche 1 du telephone - TRXD <-> L1CTL" \
        "$(_tail "'$L/trxcon-1.log'")" "$C_MS2"
    _split radio "BTS | osmo-bts-trx - cote reseau du commutateur" \
        "$(_tail "'$L/osmo-bts-trx.log'")" "$C_MS2"

    _fenetre_coeur

    _w     ms "MS | pile L2/L3 du telephone logiciel - camp, LU, appels" \
        "$(_tail "'$L/mobile.log'")" "$C_MS1"
    _split ms "GSMTAP | capture pcap - a ouvrir dans Wireshark" \
        "$(_tail "'$L/gsmtap.log'")" "$C_MS1"

    _fenetre_voix
    _fenetre_shell "MS    : telnet 127.0.0.1 4247   puis  show ms"
}
