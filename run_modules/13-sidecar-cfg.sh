# =============================================================================
#  13-sidecar-cfg — les configurations du BTS#1 (side-car fake_trx)
# =============================================================================
#
#  RÔLE
#    Le profil « hybrid » fait tourner DEUX stations de base sur un seul cœur :
#
#        BTS#0   QEMU Calypso   ARFCN 514   unit-id 6001   TRXD 5700
#        BTS#1   side-car       ARFCN 516   unit-id 6002   TRXD 5720
#                (fake_trx + trxcon + un second mobile)
#
#    Les deux campent sur le même MSC, donc MS#2 peut appeler MS#1 sans sortir
#    de la maquette. Ce module ne lance rien : il pose les trois fichiers dont
#    la chaîne a besoin.
#
#  POURQUOI SI TÔT (numéro 13, avant 14-bsc)
#    Le bloc « bts 1 » doit être dans osmo-bsc.cfg AVANT que le BSC ne démarre.
#    L'insérer plus tard obligerait à redémarrer le BSC, ce qui ferait tomber
#    l'OML du BTS#0 — donc couperait la station qui marchait pour ajouter la
#    seconde. Le prix à payer est cet ordre inhabituel ; il est délibéré.
#
#  SUCCÈS   les trois fichiers existent et le BSC déclare l'unit-id du side-car.
#  JOURNAL  $LOG_DIR/mod/sidecar-cfg.log
# -----------------------------------------------------------------------------
. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"

MOD_REGISTER sidecar-cfg "Configurations du BTS#1 (side-car)"
MOD_REQUIRED[sidecar-cfg]=1
MOD_DEPS[sidecar-cfg]="config"
MOD_PROFILES[sidecar-cfg]="hybrid"
MOD_PURE[sidecar-cfg]=1
MOD_TIMEOUT[sidecar-cfg]=10

: "${BSC_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-bsc.cfg}"

_sc_bsc_declare() { grep -qE "^[[:space:]]*ipa unit-id ${SC_UNIT_ID} 0" "$BSC_CFG" 2>/dev/null; }

mod_sidecar_cfg_check() {
    [ -r "$BSC_CFG" ] || {
        mod_hint "posez BSC_CFG, ou déployez /etc/osmocom/osmo-bsc.cfg"
        mod_fail "configuration BSC illisible : $BSC_CFG"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_cfg_status() {
    _sc_bsc_declare && [ -r "$SC_BTS_CFG" ] && [ -r "$SC_MOBILE_CFG" ]
}

# Bloc « bts 1 » : même LAC que le BTS#0 (donc même zone de localisation, pas de
# mise à jour de localisation en passant de l'un à l'autre), ARFCN et BSIC
# distincts pour que le mobile les sépare.
_sc_bts_block() {
    cat <<BLOC
 bts 1
  type osmo-bts
  band DCS1800
  cell_identity ${SC_UNIT_ID}
  location_area_code 0x0001
  base_station_id_code 8
  ms max power 15
  cell reselection hysteresis 4
  rxlev access min 0
  radio-link-timeout 32
  channel allocator mode chan-req descending
  channel allocator mode assignment ascending
  channel allocator mode handover ascending
  rach tx integer 9
  rach max transmission 7
  ipa unit-id ${SC_UNIT_ID} 0
  oml ipa stream-id 255 line 0
  neighbor-list mode automatic
  codec-support fr
  gprs mode none
  trx 0
   rf_locked 0
   arfcn ${SC_ARFCN}
   nominal power 23
   max_power_red 0
   rsl e1 tei 0
$(for t in 0 1 2 3 4 5 6 7; do
    # TS0 = CCCH+SDCCH4 (4 SDCCH repliés), TS1 = SDCCH8 dédié (8 SDCCH de plus),
    # TS2..7 = voix. Symétrique du BTS#0 : sans ce TS1, toute la signalisation
    # du side-car tient dans les 4 sous-canaux du TS0.
    case "$t" in
      0) pchan='CCCH+SDCCH4' ;;
      1) pchan="${SC_TS1_PCHAN:-SDCCH8}" ;;
      *) pchan='TCH/F' ;;
    esac
    printf '   timeslot %s\n    phys_chan_config %s\n    hopping enabled 0\n' "$t" "$pchan"
  done)
BLOC
}

mod_sidecar_cfg_start() {
    # --- 1. le bloc « bts 1 » dans la configuration du BSC -------------------
    if _sc_bsc_declare; then
        mod_say "bloc « bts 1 » (unit-id $SC_UNIT_ID) déjà déclaré dans $BSC_CFG"
    else
        grep -qE '^msc 0' "$BSC_CFG" || {
            mod_hint "le bloc « bts 1 » s'insère avant « msc 0 » ; cette ancre est absente"
            mod_fail "structure inattendue dans $BSC_CFG"; return $MOD_RC_FAIL; }
        cp -f "$BSC_CFG" "${BSC_CFG}.avant-sidecar" 2>/dev/null
        local tmp; tmp="$(mktemp)"
        awk -v bloc="$(_sc_bts_block)" '
            /^msc 0/ && !fait { print bloc; fait = 1 }
            { print }
        ' "$BSC_CFG" > "$tmp" || { rm -f "$tmp"; mod_fail "réécriture de $BSC_CFG impossible"; return $MOD_RC_FAIL; }
        # On réécrit le CONTENU, on ne remplace pas le FICHIER : `mv` depuis un
        # mktemp imposerait le mode 0600 du temporaire, et osmo-bsc (utilisateur
        # « osmocom ») ne pourrait plus lire sa configuration. Le message qu'il
        # renvoie alors — « No such file or directory » — désigne un refus de
        # lecture, pas un fichier manquant : piège vérifié le 2026-07-29.
        cat "$tmp" > "$BSC_CFG" && rm -f "$tmp"
        _sc_bsc_declare || { mod_fail "insertion du bloc « bts 1 » sans effet"; return $MOD_RC_FAIL; }
        mod_say "bloc « bts 1 » inséré (sauvegarde : ${BSC_CFG}.avant-sidecar)"
    fi

    # --- 2. la configuration du second osmo-bts-trx --------------------------
    [ -r "$SC_BTS_CFG" ] || {
        mod_hint "attendu : $SC_BTS_CFG (VTY $SC_BTS_VTY_PORT, base-port remote $SC_TRX_PORT)"
        mod_fail "configuration du BTS#1 absente"; return $MOD_RC_FAIL; }
    local port; port="$(radio_cfg_val "$SC_BTS_CFG" "osmotrx base-port remote")"
    [ "${port:-$SC_TRX_PORT}" = "$SC_TRX_PORT" ] && mod_say "BTS#1 : $SC_BTS_CFG (TRXD $SC_TRX_PORT)" \
        || mod_say "⚠ $SC_BTS_CFG annonce base-port remote $port, le side-car attend $SC_TRX_PORT"

    # --- 3. la configuration du second mobile -------------------------------
    if [ ! -r "$SC_MOBILE_CFG" ]; then
        mod_hint "attendu : $SC_MOBILE_CFG (layer2-socket $SC_L2_SOCK, VTY $SC_MOBILE_VTY_PORT)"
        mod_fail "configuration du MS#2 absente"; return $MOD_RC_FAIL
    fi
    local imsi; imsi="$(awk '$1=="imsi"{print $2; exit}' "$SC_MOBILE_CFG" 2>/dev/null)"
    mod_say "MS#2 : $SC_MOBILE_CFG (imsi ${imsi:-?}, socket $SC_L2_SOCK)"
    mod_ok
}

mod_sidecar_cfg_stop() { return 0; }
