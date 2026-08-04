# =============================================================================
#  tmux_modules/hybrid.sh — disposition du profil « hybrid »
# =============================================================================
#  DEUX téléphones sur UN SEUL cœur, chacun avec sa station de base :
#
#      MS#1  firmware Calypso émulé  -> BTS#0  ARFCN 514  unit-id 6001
#      MS#2  trxcon + fake_trx       -> BTS#1  ARFCN 516  unit-id 6002
#
#  Même LAC, même MSC : MS#2 peut appeler MS#1 sans sortir de la maquette.
#  D'où une fenêtre par abonné — les mélanger rendrait illisible la seule chose
#  qu'on regarde vraiment ici : lequel des deux progresse.
#
#      1 radio   QEMU · osmocon · pont I/Q · osmo-trx-ipc
#      2 coeur   MSC · BSC · HLR · MGW+STP
#      3 ms1     MS#1 · BTS#0 · gr-gsm · GSMTAP
#      4 ms2     MS#2 · trxcon · BTS#1 · fake_trx
#      5 voix    GAPK · passerelle SIP
#      6 shell
# -----------------------------------------------------------------------------
TMUX_FENETRE_PREMIERE=radio
TMUX_RESUME="8 fenêtres : radio · coeur · dsp · asm · ms1 · ms2 (side-car) · voix · shell"

tmux_layout_premiere() {
    printf "tail -n 200 -F '%s/qemu.log' 2>/dev/null | stdbuf -oL tr -d '\\\\007' || sleep infinity" \
           "${LOG_DIR:-/root/calypso/logs}"
}

tmux_layout() {
    local L="${LOG_DIR:-/root/calypso/logs}"

    # ── 1 · radio — la chaîne du téléphone ÉMULÉ (MS#1) ──────────────────────
    tmux select-pane -t "$TMUX_SESSION:radio" -T \
        "QEMU | baseband emule (ARM7 + DSP c54x) - traces du modele" 2>/dev/null
    _paint radio "$C_RADIO"
    _split radio "osmocon | lien serie firmware - romload puis transport L1CTL" \
        "$(_tail "'$L/osmocon.log'")" "$C_RADIO"
    _split radio "ipc-device | pont I/Q QEMU <-> memoire partagee" \
        "$(_tail "'$L/calypso-ipc-device.log'")" "$C_RADIO"
    _split radio "osmo-trx-ipc | I/Q <-> bursts, cote antenne du BTS#0" \
        "$(_tail "'$L/osmo-trx-ipc.log'")" "$C_RADIO"

    _fenetre_coeur
    _fenetre_dsp
    _fenetre_asm

    # ── 3 · ms1 — le premier abonné, celui qui passe par le firmware ─────────
    _w     ms1 "MS#1 | pile L2/L3 du telephone emule - camp, LU, appels" \
        "$(_tail "'$L/mobile.log'")" "$C_MS1"
    _split ms1 "BTS#0 | sa station de base - ARFCN 514, unit-id 6001" \
        "$(_tail "'$L/bts.log'")" "$C_MS1"
    _split ms1 "gr-gsm | demodulation hote - SI et SCH decodes du flux I/Q" \
        "$(_tail "'$L/grgsm_decode.log'")" "$C_MS1"
    _split ms1 "GSMTAP | capture pcap - a ouvrir dans Wireshark" \
        "$(_tail "'$L/gsmtap.log'")" "$C_MS1"

    # ── 4 · ms2 — le second abonné, radio simulée (bursts, pas d'I/Q) ────────
    _w     ms2 "MS#2 | second abonne - campe sur le BTS#1, appelle le MS#1" \
        "$(_tail "'$L/sidecar-mobile.log'")" "$C_MS2"
    _split ms2 "trxcon | sa couche 1 - TRXD <-> L1CTL (remplace le firmware)" \
        "$(_tail "'$L/sidecar-trxcon.log'")" "$C_MS2"
    _split ms2 "BTS#1 | seconde station - ARFCN 516, unit-id 6002" \
        "$(_tail "'$L/sidecar-bts.log'")" "$C_MS2"
    _split ms2 "fake_trx | commutateur de bursts UDP 5720 <-> 6720" \
        "$(_tail "'$L/sidecar-faketrx.log'")" "$C_MS2"

    _fenetre_voix
    _fenetre_shell \
        "MS#1  : telnet 127.0.0.1 4247   puis  show ms" \
        "MS#2  : telnet 127.0.0.1 4248   puis  call 1 10001"
}
