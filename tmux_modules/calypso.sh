# =============================================================================
#  tmux_modules/calypso.sh — disposition du profil « calypso »
# =============================================================================
#  UN téléphone : le baseband Calypso émulé par QEMU, sa chaîne radio complète,
#  et le cœur Osmocom. C'est le banc historique.
#
#      1 radio   QEMU · osmocon · pont I/Q · osmo-trx-ipc
#      2 coeur   MSC · BSC · HLR · MGW+STP
#      3 ms1     la pile du téléphone · son BTS · gr-gsm · GSMTAP
#      4 voix    GAPK · passerelle SIP
#      5 shell
# -----------------------------------------------------------------------------
TMUX_FENETRE_PREMIERE=radio
TMUX_RESUME="7 fenêtres : radio · coeur · dsp · asm · ms1 · voix · shell"

tmux_layout_premiere() {   # commande de la fenêtre créée avec la session
    printf "tail -n 200 -F '%s/qemu.log' 2>/dev/null | stdbuf -oL tr -d '\\\\007' || sleep infinity" \
           "${LOG_DIR:-/root/calypso/logs}"
}

tmux_layout() {
    local L="${LOG_DIR:-/root/calypso/logs}"

    # ── 1 · radio — la chaîne du téléphone émulé, de l'ARM jusqu'à l'antenne ─
    tmux select-pane -t "$TMUX_SESSION:radio" -T \
        "QEMU | baseband emule (ARM7 + DSP c54x) - traces du modele" 2>/dev/null
    _paint radio "$C_RADIO"
    _split radio "osmocon | lien serie firmware - romload puis transport L1CTL" \
        "$(_tail "'$L/osmocon.log'")" "$C_RADIO"
    _split radio "ipc-device | pont I/Q QEMU <-> memoire partagee" \
        "$(_tail "'$L/calypso-ipc-device.log'")" "$C_RADIO"
    _split radio "osmo-trx-ipc | I/Q <-> bursts, cote antenne" \
        "$(_tail "'$L/osmo-trx-ipc.log'")" "$C_RADIO"

    _fenetre_coeur
    _fenetre_dsp
    _fenetre_asm

    # ── 3 · ms1 — ce que le téléphone comprend du signal ─────────────────────
    _w     ms1 "MS | pile L2/L3 du telephone emule - camp, LU, appels" \
        "$(_tail "'$L/mobile.log'")" "$C_MS1"
    _split ms1 "BTS | osmo-bts-trx - ARFCN 514, unit-id 6001" \
        "$(_tail "'$L/bts.log'")" "$C_MS1"
    _split ms1 "gr-gsm | demodulation hote - SI et SCH decodes du flux I/Q" \
        "$(_tail "'$L/grgsm_decode.log'")" "$C_MS1"
    _split ms1 "GSMTAP | capture pcap - a ouvrir dans Wireshark" \
        "$(_tail "'$L/gsmtap.log'")" "$C_MS1"

    _fenetre_voix
    _fenetre_shell "MS    : telnet 127.0.0.1 4247   puis  show ms"
}
