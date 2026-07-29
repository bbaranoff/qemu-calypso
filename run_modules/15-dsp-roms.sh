# =============================================================================
#  15-dsp-roms — ROM du DSP TMS320C54x : découpe, variantes, argument machine
# =============================================================================
#
#  RÔLE
#    Garantir que les sept binaires de ROM attendus par `-M calypso,dsp-*`
#    existent et sont à jour vis-à-vis de leur source texte, appliquer les
#    variantes opt-in (fixture DARAM, stub L1), et composer l'argument machine
#    correspondant. Reprend run.sh.legacy #28 (L1618-1714), #29 (L1716-1763) et
#    la part « composition » de #30 (L1765-1780).
#
#  PRÉREQUIS  : 05-config.
#  SUCCÈS     : les sections actives sont lisibles et au moins aussi récentes
#               que la source .txt ; CALYPSO_MACHINE_ARG contient une clé
#               `dsp-<section>` par section active.
#  JOURNAL    : $LOG_DIR/mod/dsp-roms.log
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. POURQUOI LA DÉCOUPE EST CONDITIONNELLE, ET SUR LA DATE.
#     calypso_dsp.txt fait ~700 Ko ; le redécouper à chaque run coûte plusieurs
#     secondes pour rien. Le legacy comparait donc `.txt -nt .bin` (L1686) : on
#     ne redécoupe que si une section manque ou si la source a bougé. C'est
#     aussi le seul garde-fou contre un .bin périmé — une ROM d'une version
#     antérieure produit un DSP qui « tourne » et ne détecte rien.
#
#  2. POURQUOI ON N'UTILISE PAS `readlink -f` SUR LA SOURCE.
#     Sur cette installation, $DSP_ROM_DIR/calypso_dsp.txt est un LIEN vers
#     ${QEMU_TREE}/calypso_dsp.txt. Les .bin sont écrits « à côté de la
#     source » : à côté du LIEN (donc dans $DSP_ROM_DIR, ce que paths.env
#     attend), et surtout PAS à côté de la cible — ce qui écrirait dans
#     osmo-qemu-calypso, arbre voisin en lecture seule. On résout donc le répertoire sur
#     le chemin du lien, jamais sur sa cible.
#
#  3. POURQUOI CE MODULE SYNCHRONISE DEUX FAMILLES DE NOMS.
#     paths.env nomme les ROM DSP_PROM0…DSP_REGISTERS et c'est ce que
#     40-qemu.sh lit pour construire `-M`. Le legacy, lui, nommait
#     CALYPSO_DSP_PROM0… et composait MACHINE_ARG à partir de ces variables-là.
#     Les deux familles doivent désigner les mêmes fichiers, sinon une variante
#     opt-in (stub L1) serait posée d'un côté et ignorée de l'autre. Les
#     CALYPSO_DSP_* sont donc initialisées depuis les DSP_*, et la propagation
#     inverse n'a lieu QUE pour une variante explicitement demandée.
#
#  4. CE QUE CE MODULE NE FAIT PAS.
#     Il ne vérifie pas la présence des six ROM : c'est déjà le rôle de
#     mod_qemu_check (40-qemu.sh L22-27), avec le bon message. Il ne construit
#     pas non plus la ligne de commande : 40-qemu.sh compose la sienne et ne lit
#     pas CALYPSO_MACHINE_ARG. La variable est posée et tracée pour que l'écart
#     soit visible le jour où l'on arbitre ce point.
# -----------------------------------------------------------------------------

MOD_REGISTER dsp-roms "ROM du DSP (découpe et variantes)"
MOD_REQUIRED[dsp-roms]=1
MOD_DEPS[dsp-roms]="config"
MOD_PROFILES[dsp-roms]="calypso hybrid"
MOD_TIMEOUT[dsp-roms]=120

: "${DSP_SECTIONS:=PROM0 PROM1 PROM2 PROM3 DROM PDROM}"
: "${DSP_TXT2BIN:=}"      # résolu dans check, laissé surchargeable

_dsp_txt() { printf '%s' "${CALYPSO_DSP_ROM_TXT:-${DSP_ROM_DIR:-/opt/GSM}/calypso_dsp.txt}"; }
_dsp_dir() { dirname "$(_dsp_txt)"; }                       # cf. POURQUOI 2
_dsp_base() { basename "$(_dsp_txt)" .txt; }
_dsp_sec_bin() { printf '%s/%s.%s.bin' "$(_dsp_dir)" "$(_dsp_base)" "$1"; }

# Une découpe est nécessaire si une section manque ou si la source est plus
# récente (legacy L1682-1690).
_dsp_need_split() {
    local sec bin txt; txt="$(_dsp_txt)"
    [ -r "$txt" ] || return 1
    for sec in $DSP_SECTIONS; do
        bin="$(_dsp_sec_bin "$sec")"
        [ -r "$bin" ] || return 0
        [ "$txt" -nt "$bin" ] && return 0
    done
    return 1
}

mod_dsp_roms_check() {
    local txt bins_ok=1 sec
    txt="$(_dsp_txt)"

    for sec in $DSP_SECTIONS; do
        [ -r "$(_dsp_sec_bin "$sec")" ] || bins_ok=0
    done

    # Résolution de l'outil de découpe, en lecture seule.
    if [ -z "$DSP_TXT2BIN" ]; then
        if   [ -r "${QEMU_TREE:-}/python_scripts/dsp_txt2bin.py" ]; then DSP_TXT2BIN="${QEMU_TREE}/python_scripts/dsp_txt2bin.py"
        elif [ -r "${QEMU_TREE:-}/dsp_txt2bin.py" ];                then DSP_TXT2BIN="${QEMU_TREE}/dsp_txt2bin.py"
        fi
    fi

    if [ "$bins_ok" = "0" ] && [ ! -r "$txt" ]; then
        mod_hint "posez CALYPSO_DSP_ROM_TXT sur le dump texte de la ROM, ou DSP_ROM_DIR sur le dossier des .bin"
        mod_fail "ni les sections .bin ni la source $txt ne sont lisibles"
        return $MOD_RC_FAIL
    fi
    if [ "$bins_ok" = "0" ] && [ -z "$DSP_TXT2BIN" ]; then
        mod_hint "attendu : \$QEMU_TREE/python_scripts/dsp_txt2bin.py"
        mod_fail "sections .bin absentes et dsp_txt2bin.py introuvable : impossible de les produire"
        return $MOD_RC_FAIL
    fi
    if [ -z "$DSP_TXT2BIN" ]; then
        mod_say "dsp_txt2bin.py introuvable — pas de redécoupe possible ; les .bin existants seront utilisés tels quels"
    fi
    mod_ok
}

# Pas de notion de « déjà démarré » : la fraîcheur des .bin doit être
# reconsidérée à chaque run (la source peut avoir changé entre deux).
mod_dsp_roms_status() { return $MOD_RC_FAIL; }

mod_dsp_roms_start() {
    local sec var val txt; txt="$(_dsp_txt)"

    # --- 1. fixture DARAM : elle prend toute la place (legacy L1658-1672) ----
    # Quand un blob est fourni, il est la SEULE source de code du DSP : on vide
    # explicitement les sections pour qu'aucune ROM ne soit chargée par-dessus
    # (collision d'overlay en DARAM).
    if [ -n "${CALYPSO_DSP_BLOB:-}" ]; then
        for sec in $DSP_SECTIONS REGISTERS; do eval "CALYPSO_DSP_${sec}=''"; done
        mod_say "CALYPSO_DSP_BLOB=$CALYPSO_DSP_BLOB : toutes les sections désactivées (le blob est seul)"
    else
        # --- 2. découpe à la demande (cf. POURQUOI 1) ------------------------
        if _dsp_need_split; then
            if [ -n "$DSP_TXT2BIN" ]; then
                mod_say "découpe $txt -> $(_dsp_dir)/$(_dsp_base).{PROM0..3,DROM,PDROM}.bin"
                python3 "$DSP_TXT2BIN" "$txt" "$(_dsp_dir)/$(_dsp_base).bin" \
                    || mod_say "dsp_txt2bin.py a échoué — on poursuit avec les .bin en place (la barrière tranchera)"
            else
                mod_say "sections périmées mais dsp_txt2bin.py absent : rien à faire"
            fi
        else
            mod_say "sections à jour vis-à-vis de $txt : pas de découpe"
        fi

        # --- 3. les deux familles de noms se rejoignent (cf. POURQUOI 3) -----
        # `+x` et non `:-` : une variable posée VIDE signifie « section
        # volontairement désactivée » et doit le rester (legacy L1703-1704).
        for sec in $DSP_SECTIONS; do
            var="CALYPSO_DSP_${sec}"
            eval "val=\${${var}+x}"
            if [ -z "$val" ]; then
                eval "val=\${DSP_${sec}:-}"
                [ -n "$val" ] && [ -r "$val" ] || val="$(_dsp_sec_bin "$sec")"
                eval "$var=\"\$val\""
            fi
        done
        eval "val=\${CALYPSO_DSP_REGISTERS+x}"
        if [ -z "$val" ]; then
            val="${DSP_REGISTERS:-}"
            [ -n "$val" ] && [ -r "$val" ] || val="$(_dsp_dir)/$(_dsp_base).Registers.bin"
            CALYPSO_DSP_REGISTERS="$val"
        fi

        # --- 4. variante « stub L1 » (legacy L1716-1763) ---------------------
        # Publisher synthétique L1+L2 patché dans PROM0 : le mobile campe sans
        # dépendre de la corrélation I/Q. Le reste de la ROM est intact.
        # Deux noms historiques pour la même chose ; on accepte les deux.
        if [ "${CALYPSO_DSP_L1STUB:-0}" = "1" ] || [ "${CALYPSO_DSP_L1_STUB:-0}" = "1" ]; then
            local stub_out="${RUN_DIR:-/tmp/calypso}/calypso_dsp_L1stub.PROM0.bin"
            local stub_script="${QEMU_TREE:-}/scripts/make_dsp_bin_L1.py"
            if [ ! -r "$stub_script" ]; then
                mod_hint "attendu : \$QEMU_TREE/scripts/make_dsp_bin_L1.py"
                mod_fail "stub L1 demandé mais make_dsp_bin_L1.py est introuvable"
                return $MOD_RC_FAIL
            fi
            mod_say "stub L1 : ${CALYPSO_DSP_PROM0} -> $stub_out"
            if ! python3 "$stub_script" "${CALYPSO_DSP_PROM0}" "$stub_out"; then
                mod_hint "retirez CALYPSO_DSP_L1STUB pour repartir sur la ROM réelle"
                mod_fail "make_dsp_bin_L1.py a échoué : PROM0 non patchée"
                return $MOD_RC_FAIL
            fi
            CALYPSO_DSP_PROM0="$stub_out"
            DSP_PROM0="$stub_out"     # 40-qemu.sh lit DSP_PROM0 — cf. POURQUOI 3
            export DSP_PROM0
            mod_say "PROM0 = $stub_out (publisher L1+L2 actif, reste de la ROM intact)"
        fi
    fi

    # --- 5. argument machine (legacy L1765-1780) -----------------------------
    CALYPSO_MACHINE_ARG="calypso"
    if [ -n "${CALYPSO_DSP_BLOB:-}" ]; then
        CALYPSO_MACHINE_ARG="${CALYPSO_MACHINE_ARG},dsp-blob=${CALYPSO_DSP_BLOB}"
    fi
    for sec in $DSP_SECTIONS REGISTERS; do
        eval "val=\${CALYPSO_DSP_${sec}:-}"
        [ -n "$val" ] || continue
        CALYPSO_MACHINE_ARG="${CALYPSO_MACHINE_ARG},dsp-$(printf '%s' "$sec" | tr '[:upper:]' '[:lower:]')=${val}"
    done
    export CALYPSO_MACHINE_ARG
    mod_say "CALYPSO_MACHINE_ARG = $CALYPSO_MACHINE_ARG"
    mod_say "NB : 40-qemu.sh compose son propre -M et ne lit pas cette variable (cf. POURQUOI 4)"

    calypso_export_sweep
    mod_ok
}

# BARRIÈRE — critère observable : chaque section ACTIVE (variable non vide) est
# lisible, non vide, et pas plus ancienne que la source texte. La fraîcheur est
# le point important : une ROM périmée ne provoque aucune erreur au lancement,
# elle produit un DSP qui tourne et ne détecte rien — le pire des symptômes.
_dsp_sections_ready() {
    local sec val txt; txt="$(_dsp_txt)"
    if [ -n "${CALYPSO_DSP_BLOB:-}" ]; then
        [ -r "$CALYPSO_DSP_BLOB" ] && [ -s "$CALYPSO_DSP_BLOB" ]
        return
    fi
    for sec in $DSP_SECTIONS REGISTERS; do
        eval "val=\${CALYPSO_DSP_${sec}:-}"
        [ -n "$val" ] || continue
        [ -r "$val" ] && [ -s "$val" ] || return 1
        [ -r "$txt" ] && [ "$txt" -nt "$val" ] && return 1
    done
    return 0
}
_dsp_bad_sections() {
    local sec val out="" txt; txt="$(_dsp_txt)"
    [ -n "${CALYPSO_DSP_BLOB:-}" ] && { [ -s "${CALYPSO_DSP_BLOB}" ] || out=" blob:${CALYPSO_DSP_BLOB}"; printf '%s' "${out# }"; return 0; }
    for sec in $DSP_SECTIONS REGISTERS; do
        eval "val=\${CALYPSO_DSP_${sec}:-}"
        [ -n "$val" ] || continue
        if [ ! -s "$val" ];              then out="$out $sec:illisible"
        elif [ -r "$txt" ] && [ "$txt" -nt "$val" ]; then out="$out $sec:périmée"
        fi
    done
    printf '%s' "${out# }"
}

mod_dsp_roms_wait() {
    if ! wait_until "${MOD_TIMEOUT[dsp-roms]}" "sections de ROM DSP" _dsp_sections_ready; then
        mod_hint "relancez la découpe : python3 ${DSP_TXT2BIN:-<dsp_txt2bin.py>} $(_dsp_txt) $(_dsp_dir)/$(_dsp_base).bin"
        mod_fail "sections de ROM inutilisables : $(_dsp_bad_sections)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# On ne supprime AUCUN .bin : ils sont coûteux à reproduire et partagés avec les
# autres points d'entrée. Seule la variante temporaire du run est retirée.
mod_dsp_roms_stop() {
    rm -f "${RUN_DIR:-/tmp/calypso}/calypso_dsp_L1stub.PROM0.bin" 2>/dev/null
    return 0
}
