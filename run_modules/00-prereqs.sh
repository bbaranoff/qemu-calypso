# 00-prereqs — vérifie que l'installation est utilisable, sans rien lancer.
# MOD_PURE=1 : aucun effet de bord, donc exécuté même en --dry-run pour que le
# plan affiché soit exact.
MOD_REGISTER prereqs "Vérification des prérequis"
MOD_REQUIRED[prereqs]=1
MOD_PURE[prereqs]=1
MOD_PROFILES[prereqs]="calypso faketrx hybrid core"

mod_prereqs_check() {
    local missing=""
    for v in QEMU_BIN FIRMWARE_ELF DSP_PROM0; do
        local p="${!v:-}"
        [ -n "$p" ] && [ -e "$p" ] || missing="$missing $v"
    done
    if [ -n "$missing" ]; then
        mod_hint "réglez les chemins dans environnement/paths.env, puis ./run.sh --check-paths"
        mod_fail "dépendances introuvables :$missing"
        return $MOD_RC_FAIL
    fi
    mod_ok
}
mod_prereqs_start()  { mod_ok; }
mod_prereqs_status() { return $MOD_RC_FAIL; }   # module pur : jamais « déjà démarré »
