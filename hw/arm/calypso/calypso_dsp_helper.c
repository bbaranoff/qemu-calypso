/*
 * calypso_dsp_helper.c — mode-neutral NDB-write PRIMITIVES for the Calypso
 * DSP shunt. Split out VERBATIM from calypso_dsp_shunt.c (pure mechanical
 * move, no logic change). The shunt GLUE lives in calypso_dsp_shunt.c and
 * DEFINES the shared state (g_shunt / g_canned) declared in the internal
 * header below.
 */

#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"
#include "hw/arm/calypso/calypso_trf6151.h"
#include "hw/arm/calypso/calypso_dsp_internal.h"

/* CALYPSO_DSP=c54x : route les ordres+I/Q vers le VRAI c54x (pas de mock).
 * getenv lu une seule fois (idiome memoize du fichier). */
bool shunt_route_c54x(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("CALYPSO_DSP");
        v = (e && strcmp(e, "c54x") == 0) ? 1 : 0;
    }
    return v;
}

/* Tag de log : en mode no-shunt/c54x le shunt agit en ASSIST -> ne pas
 * afficher [feed-daram-dsp] (trompeur). SHUNT_LOG/SHUNT_ERR prefixent le tag runtime. */
const char *shunt_tag(void)
{ return shunt_route_c54x() ? "[dsp/c54x]" : "[feed-daram-dsp]"; }

/* ---- Helpers : read/write API RAM via AddressSpace (16-bit LE) ---- */
uint16_t shunt_read_w(uint32_t addr)
{
    uint16_t v = 0;
    /* [2026-07-27] AS-NULL guard : le shunt peut etre inactif (g_shunt.as jamais affecte) et des appelants natifs non gardes passent quand meme ici -> SIGSEGV. */
    if (!g_shunt.as) return 0;
    dma_memory_read(g_shunt.as, addr, &v, sizeof(v), MEMTXATTRS_UNSPECIFIED);
    return le16_to_cpu(v);
}

void shunt_write_w(uint32_t addr, uint16_t v)
{
    uint16_t le = cpu_to_le16(v);
    /* [2026-07-27] AS-NULL guard : le shunt peut etre inactif (g_shunt.as jamais affecte) et des appelants natifs non gardes passent quand meme ici -> SIGSEGV. */
    if (!g_shunt.as) return;
    dma_memory_write(g_shunt.as, addr, &le, sizeof(le), MEMTXATTRS_UNSPECIFIED);
}

/* Lit l1s.current_time.fn (FN L1 du firmware) en ARM RAM. current_time = champ 0
 * de struct l1s_state @ 0x836508 ; fn = champ 0 de struct gsm_time -> offset 0.
 * C'est LE FN que le firmware utilise pour ses blocs (BCCH/CCCH) et mémorise pour
 * la RACH. On gate la présentation a_cd dessus (et NON s->fn = calypso_trx_get_fn,
 * qui diffère de l1s d'un offset run-variant -> blocs CCCH décalés -> AGCH raté). */
/* [2026-07-27] Resolution DYNAMIQUE d'un symbole du firmware ELF (robuste aux
 * rebuilds). Chemin = env CALYPSO_FIRMWARE_ELF, sinon l'arg -kernel de
 * /proc/self/cmdline. Parse ELF32 LE .symtab. Retourne 0 si introuvable. */
/* [2026-08-08] Extrait de shunt_fw_sym : le MEME ELF sert desormais a deux
 * lecteurs — les symboles ici, et les offsets de champs NDB resolus du DWARF
 * cote shunt (shunt_ndb_resolve_offsets). Une seule source de verite pour le
 * chemin : deux copies divergeraient le jour ou l'une des deux change. */
const char *shunt_fw_elf_path(void)
{
    static char path[1024];
    static int done;
    if (done) return path[0] ? path : NULL;
    done = 1;
    const char *env = getenv("CALYPSO_FIRMWARE_ELF");
    if (env && *env) { snprintf(path, sizeof(path), "%s", env); return path; }
    FILE *cf = fopen("/proc/self/cmdline", "rb");
    if (cf) {
        static char cl[16384];
        size_t nr = fread(cl, 1, sizeof(cl) - 1, cf);
        fclose(cf);
        cl[nr] = 0;
        for (size_t i = 0; i < nr; ) {
            size_t l = strlen(cl + i);
            if (!strcmp(cl + i, "-kernel") && i + l + 1 < nr) {
                snprintf(path, sizeof(path), "%s", cl + i + l + 1);
                break;
            }
            i += l + 1;
        }
    }
    return path[0] ? path : NULL;
}

static uint32_t shunt_fw_sym(const char *want)
{
    const char *p = shunt_fw_elf_path();
    if (!p) return 0;
    char path[1024];
    snprintf(path, sizeof(path), "%s", p);
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 52 || sz > (64L << 20)) { fclose(f); return 0; }
    uint8_t *b = g_malloc((size_t)sz);
    size_t got = fread(b, 1, (size_t)sz, f);
    fclose(f);
    uint32_t ret = 0;
    if (got == (size_t)sz && b[0] == 0x7f && b[1] == 'E' && b[2] == 'L' && b[3] == 'F' && b[4] == 1) {
#define R16(o) ((uint32_t)b[o] | ((uint32_t)b[(o)+1] << 8))
#define R32(o) ((uint32_t)b[o] | ((uint32_t)b[(o)+1] << 8) | ((uint32_t)b[(o)+2] << 16) | ((uint32_t)b[(o)+3] << 24))
        uint32_t shoff = R32(0x20), shent = R16(0x2e), shnum = R16(0x30);
        for (uint32_t si = 0; si < shnum; si++) {
            uint32_t sh = shoff + si * shent;
            if ((long)(sh + 40) > sz) break;
            if (R32(sh + 4) == 2) { /* SHT_SYMTAB */
                uint32_t symoff = R32(sh + 0x10), symsz = R32(sh + 0x14);
                uint32_t link = R32(sh + 0x18), entsz = R32(sh + 0x24);
                uint32_t strsh = shoff + link * shent;
                if ((long)(strsh + 40) > sz || entsz < 16) break;
                uint32_t stroff = R32(strsh + 0x10), strsz = R32(strsh + 0x14);
                for (uint32_t o = 0; o + 16 <= symsz && (long)(symoff + o + 16) <= sz; o += entsz) {
                    uint32_t ni = R32(symoff + o);
                    uint32_t val = R32(symoff + o + 4);
                    if (ni < strsz) {
                        const char *nm = (const char *)(b + stroff + ni);
                        if (!strcmp(nm, want)) { ret = val; break; }
                    }
                }
                break;
            }
        }
#undef R16
#undef R32
    }
    g_free(b);
    return ret;
}

uint32_t shunt_l1s_fn(void)
{
    static uint32_t addr = 0;
    if (!addr) {
        const char *e = getenv("CALYPSO_L1S_FN_ADDR");
        if (e && *e) addr = (uint32_t)strtoul(e, NULL, 0);
        else { addr = shunt_fw_sym("l1s"); if (!addr) addr = 0x836508; }
    }
    uint32_t v = 0;
    /* [2026-07-27] AS-NULL guard : le shunt peut etre inactif (g_shunt.as jamais affecte) et des appelants natifs non gardes passent quand meme ici -> SIGSEGV. */
    if (!g_shunt.as) return 0;
    dma_memory_read(g_shunt.as, addr, &v, sizeof(v), MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

/* Lit last_rach.fn : le FN EXACT que le firmware a memorise pour la DERNIERE RACH
 * (prim_rach.c:94 last_rach.fn = current_time.fn-1, pose au tick l1s_tx_rach_resp)
 * et qu'il a envoye au mobile via L1CTL_RACH_CONF (prim_rach.c:114). C'EST la valeur
 * que le mobile compare a la req-ref de l'IMM ASSIGN (gsm48_rr.c:3372). La lire
 * directement = match EXACT, sans le skew variable de g_rach_l1s_fn[ra] (capture au
 * tick d_rach/cmd, -4 frames AVANT que le memo soit pose au tick resp -> l'ecart
 * cmd<->resp varie par-RACH, c'est lui qui faisait derailler tout adj fixe).
 * struct { uint32_t fn; uint16_t band_arfcn; } last_rach @ 0x836500, fn @ offset 0. */
uint32_t shunt_last_rach_fn(void)
{
    static uint32_t addr = 0;
    if (!addr) {
        const char *e = getenv("CALYPSO_LAST_RACH_FN_ADDR");
        if (e && *e) addr = (uint32_t)strtoul(e, NULL, 0);
        else { addr = shunt_fw_sym("last_rach"); if (!addr) addr = 0x836500; }
    }
    uint32_t v = 0;
    /* [2026-07-27] AS-NULL guard : le shunt peut etre inactif (g_shunt.as jamais affecte) et des appelants natifs non gardes passent quand meme ici -> SIGSEGV. */
    if (!g_shunt.as) return 0;
    dma_memory_read(g_shunt.as, addr, &v, sizeof(v), MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

uint32_t wp_base(uint8_t page_idx) {
    return page_idx ? BASE_API_W_PAGE_1 : BASE_API_W_PAGE_0;
}
uint32_t rp_base(uint8_t page_idx) {
    return page_idx ? BASE_API_R_PAGE_1 : BASE_API_R_PAGE_0;
}

bool shunt_is_canned(unsigned bit) { return (g_canned & bit) != 0; }

static uint16_t shunt_snr_val(void);   /* cf. @BEQUILLE SNR FABRIQUE, plus bas */

/* [2026-07-22] Echo de d_burst_d pour RP_D_BURST_D. Le shunt echo le burst
 * COMMANDE (WP_D_BURST_D), mais l1s_nb_resp attend le burst DEMODULE (decale du
 * pipeline cmd->resp + toggle w_page/r_page) -> mismatch systematique +1
 * (BURST ID 2!=1). Gate CALYPSO_SHUNT_BURST_M1=1 : echo (d_burst_d - 1) mod 4
 * pour coller au resp. Racine = le shunt echo le mauvais page/timing. */
uint16_t shunt_burst_echo(void)
{
    /* [2026-07-22] Phase-lock burst. Echoter WP_D_BURST_D (l'horloge d_dsp_page)
     * DERIVE contre le schedule per-burst du mobile -> BURST ID mismatch jittery.
     * CALYPSO_SHUNT_BURST_FN=1 : calcule d_burst_d depuis le FN L1 REEL
     * (shunt_l1s_fn(), l'horloge que le mobile lit) -> phase-locke. Offset
     * ajustable CALYPSO_SHUNT_BURST_OFS (0..3) pour caler la phase. Defaut =
     * echo WP (ancien comportement). */
    /* @BEQUILLE — SHUNT_BURST_FN / _OFS / _M1  (CALYPSO_SHUNT_BURST_FN, _OFS, _M1)
     *   masque  : la derivation de d_burst_d (0..3) depuis la fenetre TPU. Le shunt le
     *             synthetise, soit par echo de la commande ARM (+ofs), soit depuis
     *             l1s_fn — aucune de ces deux sources n'existe sur silicium.
     *   retirer : quand la fenetre RX TPU/BDLENA cadence le burst-id cote DSP (RANK2).
     *   NB      : SHUNT_BURST_M1 n'est consulte que si SHUNT_BURST_OFS est absente.
     */
    static int fn_mode = -1, ofs = -99;
    if (fn_mode < 0) {
        /* [2026-07-22] ECHO = DEFAUT. Le fn ne peut PAS suivre un burst_id
         * block-relatif : les blocs CCCH ne demarrent pas tous a 4-aligne
         * (starts 6,12,16,22,26,32,36,42,46 -> mix 0/2 mod 4) -> fn&3 chaotique.
         * L echo suit la sequence de commande ARM (g_shunt.d_burst_d, capturee
         * en shunt_latch_task) qui EST propre 0,1,2,3 par bloc. fn=experimental. */
        const char *e = getenv("CALYPSO_SHUNT_BURST_FN");
        fn_mode = (e && *e) ? atoi(e) : 0;
    }
    if (ofs == -99) {
        const char *e = getenv("CALYPSO_SHUNT_BURST_OFS");
        if (e && *e) ofs = atoi(e);
        else if (getenv("CALYPSO_SHUNT_BURST_M1")) ofs = -1;
        /* echo : la write page (commande ARM) precede le resp de +2 bursts
         * (SCHED prim_rx_nb.c:213-214 : frame N -> resp(k)+cmd(k+2)) -> ofs=-2.
         * Offset CONSTANT (pas jittery) : si residuel, c est -1 ou -3 (timing
         * intra-trame latch/resp), sweepable. -2 == +2 mod 4. */
        else ofs = fn_mode ? 2 : -2;
    }
    if (fn_mode)
        return (uint16_t)(((int)shunt_l1s_fn() + ofs + 4) & 3);
    return (uint16_t)((g_shunt.d_burst_d + ofs + 4) & 3);
}

/* Valeur TOA pour a_*_demod[TOA] : cannée (23 = on-time) si CAN_TOA, sinon le
 * TOA REEL mesuré par gr-gsm (sb_toa) dès qu'un SCH a été décodé ; fallback 23
 * tant qu'aucun SCH (pas 0 : évite de catastropher l'alignement avant lock). */
int shunt_toa_val(void)
{
    if (shunt_is_canned(CAN_TOA))
        return SHUNT_CANNED_TOA;
    return g_shunt.sb_valid ? g_shunt.sb_toa : SHUNT_CANNED_TOA;
}

/* Pack {bsic, t1, t2, t3} into 32-bit sb (inverse of prim_fbsb.c:125-144). */
uint32_t shunt_encode_sb(uint8_t bsic, uint16_t t1, uint8_t t2, uint8_t t3)
{
    uint8_t t3p = (t3 == 0) ? 0 : ((t3 - 1) / 10);
    uint32_t sb = 0;
    sb |= ((uint32_t)(bsic & 0x3f)) << 2;
    sb |= ((uint32_t)(t1 & 0x001)) << 23;
    sb |= ((uint32_t)(t1 & 0x1fe)) << 7;
    sb |= ((uint32_t)(t1 & 0x600)) >> 9;
    sb |= ((uint32_t)(t2 & 0x1f))  << 18;
    sb |= ((uint32_t)(t3p & 1))    << 24;
    sb |= ((uint32_t)(t3p & 6))    << 15;
    return sb;
}

/* ---- DISPATCH : FB writes NDB only ---- */
void shunt_dispatch_fb(uint8_t page_idx)
{
    /* @BEQUILLE — INJECT_FB  (CALYPSO_INJECT_FB=1, EQ1 strict — PAS de fallback
     *              CALYPSO_SHUNT_LEGIT, contrairement aux 5 autres INJECT_*)
     *   masque  : la publication du resultat FB (d_fb_det + a_sync_demod
     *             TOA/PM/ANGLE/SNR) par le correlateur DSP.
     *   retirer : quand d_fb_det natif est produit.
     *   NB      : cette variable n'est posee dans AUCUN .env livre -> shunt_dispatch_fb()
     *             est un no-op dans tous les profils, et avec lui le bloc SHUNT_REAL_FB
     *             ci-dessous. Le FB passe en realite par calypso_dsp_shunt.c
     *             (api_ram[0x08F8..] + real_fb_read).
     */
    { static int _ginj = -1; if (_ginj < 0) { const char *_e = getenv("CALYPSO_INJECT_FB"); _ginj = (_e && *_e == '1') ? 1 : 0; } if (!_ginj) return; }  /* [2026-07-23] HACK injection sortie, DEFAUT OFF (natif) ; =CALYPSO_INJECT_FB=1 pour reactiver */
    /* [2026-07-22] REAL FB (gate CALYPSO_SHUNT_REAL_FB) : injecte les valeurs
     * REELLES calculees depuis la RX (g_shunt.rx_*) au lieu des cannes. */
    {
        static int real_fb = -1;
        /* @BEQUILLE — SHUNT_REAL_FB (helper)  (CALYPSO_SHUNT_REAL_FB, defaut OFF)
         *   masque  : idem — detection FB cote hote a la place du correlateur DSP.
         *   retirer : quand d_fb_det natif fonctionne. */
        if (real_fb < 0) { const char *e = getenv("CALYPSO_SHUNT_REAL_FB"); real_fb = (e && *e == '1') ? 1 : 0; }
        if (real_fb) {
            shunt_write_w(BASE_API_NDB + NDB_D_FB_DET, g_shunt.rx_fb_det ? 1 : 0);
            shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_TOA   * 2, g_shunt.rx_toa);
            shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_PM    * 2, g_shunt.last_pm);
            shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_ANGLE * 2, (uint16_t)g_shunt.rx_afc);
            shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_SNR   * 2, g_shunt.rx_snr);
            shunt_write_w(rp_base(page_idx) + RP_D_TASK_MD, FB_DSP_TASK);
            return;
        }
    }
    /* d_fb_det = 1 ("FOUND"). prim_fbsb.c:404 reads this from NDB.
     * Canned CAN_FBDET = on force "trouvé" (pas de vrai détecteur FB ici). */
    /* FBDET non-canné = état RÉEL de détection gr-gsm : "trouvé" ssi un SCH a
     * été décodé (sb_valid). Avant lock → 0 (FB pas trouvé, comme un vrai DSP). */
    shunt_write_w(BASE_API_NDB + NDB_D_FB_DET,
                  (shunt_is_canned(CAN_FBDET) || g_shunt.sb_valid) ? 1 : 0);

    /* a_sync_demod[4] @ NDB+0x4C, 4 consecutive 16-bit words. Read by
     * read_fb_result (prim_fbsb.c:306-309) from NDB. Chaque mesure : valeur
     * cannée si son token est dans CALYPSO_CANNED, sinon 0 (pas encore de
     * vraie source → un-canner sans source casse, c'est voulu/visible). */
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_TOA   * 2, shunt_toa_val());
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM)    ? SHUNT_CANNED_PM    : g_shunt.last_pm);
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_ANGLE * 2, shunt_is_canned(CAN_ANGLE) ? SHUNT_CANNED_ANGLE : 0);
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_SNR   * 2, shunt_snr_val());

    /* Ack on the read page (echo). Not strictly required for the FB path
     * (firmware reads d_fb_det from NDB, not read-page) but mirrors the
     * real DSP's task-completion echo. */
    shunt_write_w(rp_base(page_idx) + RP_D_TASK_MD, FB_DSP_TASK);

    SHUNT_LOG("DISPATCH FB page=%u → d_fb_det=1 TOA=%d PM=0x%x "
        "ANGLE=%d SNR=0x%x (NDB only)\n",
        page_idx, SHUNT_CANNED_TOA, SHUNT_CANNED_PM,
        SHUNT_CANNED_ANGLE, SHUNT_CANNED_SNR);
}

/* [2026-08-03] @BEQUILLE — SNR FABRIQUE  (CALYPSO_SHUNT_SNR_CANNED, defaut 1)
 *
 *   (1) C'EST UNE BEQUILLE : le SNR ecrit dans a_sync_demod / a_serv_demod n'est
 *       PAS mesure. C'est la constante SHUNT_CANNED_SNR = 0x7000 — le meme mot que
 *       celui repere dans le champ de puissance sous le nom des « 448 dBm », et qui
 *       se trouve etre l'adresse de base de PROM0. Une adresse dans un champ de
 *       grandeur physique.
 *
 *   (2) CE QU'ELLE MASQUE : le DSP n'ecrit jamais son propre SNR. Le raisonnement
 *       d'origine — « gr-gsm a decode, donc le SNR etait forcement suffisant » —
 *       est defendable ; ce qui ne l'est pas, c'est de le traduire par une
 *       constante magique ET de la faire echapper a l'interrupteur prevu pour ca.
 *
 *   (3) LE DEFAUT CORRIGE ICI : l'expression etait
 *           (shunt_is_canned(CAN_SNR) || g_shunt.sb_valid) ? SHUNT_CANNED_SNR : 0
 *       Le `|| sb_valid` ecrivait la valeur cannee des que gr-gsm avait decode,
 *       SANS PASSER par CALYPSO_SHUNT_NO_CANNED. Mesure du 03/08, profil
 *       native_twl avec NO_CANNED=1 et CALYPSO_CANNED vide : `snr=0x7000` present
 *       dans data[] ET api[] a chaque releve, pendant que `pm` (20929/20595/19927)
 *       et `ang` (-186/-710) variaient — eux sont reels. Un profil qui declare
 *       « jamais de valeur fabriquee » en fabriquait une.
 *
 *   (4) QUAND LA RETIRER : quand le DSP ecrit a_sync_demod[D_SNR] lui-meme.
 *
 *   DEFAUT = 1 = comportement historique STRICTEMENT inchange. C'est deliberé :
 *   le basculer pendant un bissect en cours melangerait deux variables. Poser
 *   CALYPSO_SHUNT_SNR_CANNED=0 fait ecrire 0 a la place — a tester dans un run
 *   DEDIE, parce qu'un SNR nul peut faire rejeter la SB et casser le camp en
 *   shunt_legit. */
static uint16_t shunt_snr_val(void)
{
    static int on = -1;
    if (on < 0) {
        on = calypso_gate("CALYPSO_SHUNT_SNR_CANNED", 1);
        if (!on)
            fprintf(stderr, "[shunt] SNR_CANNED=0 : plus de SNR fabrique "
                    "(0x%04x) — on ecrit 0. Le firmware peut rejeter la SB.\n",
                    SHUNT_CANNED_SNR);
    }
    if (!on)
        return 0;
    return (shunt_is_canned(CAN_SNR) || g_shunt.sb_valid) ? SHUNT_CANNED_SNR : 0;
}

/* ---- DISPATCH : SB writes READ PAGE only ---- */
void shunt_dispatch_sb(uint8_t page_idx)
{
    /* @BEQUILLE — INJECT_SB  (CALYPSO_INJECT_SB=1, fallback CALYPSO_SHUNT_LEGIT=1)
     *   masque  : la production du burst SB (BSIC/FN) par le DSP apres detection FCCH ;
     *             ici l'encodage vient du SCH decode par gr-gsm.
     *   retirer : quand le correlateur natif enchaine FB -> SB.
     */
    /* [2026-08-03] CONVERTI a calypso_gate — un `=0` EXPLICITE doit couper.
     * Avant : `_ginj = (INJECT_SB=='1'); if (!_ginj) _ginj = (SHUNT_LEGIT=='1');`
     * donc le parapluie ECRASAIT le 0 pose a la main : sous SHUNT_LEGIT=1 il etait
     * IMPOSSIBLE d'eteindre l'injection SB, et un banc qui posait INJECT_SB=0 au
     * manifeste mesurait quand meme le SB fabrique. C'est le bug corrige sur
     * INJECT_ACD le 30/07 (cf. shunt_dispatch_allc juste en dessous) ; ce site-la
     * etait reste en arriere. Desormais le parapluie n'est qu'un DEFAUT. */
    { static int _ginj = -1;
      if (_ginj < 0) { const char *_l = getenv("CALYPSO_SHUNT_LEGIT");
                       _ginj = calypso_gate("CALYPSO_INJECT_SB", (_l && *_l == '1') ? 1 : 0); }
      if (!_ginj) return; }
    uint32_t rp = rp_base(page_idx);

    /* gr-gsm (= le DSP) a-t-il poste un vrai SCH (BSIC/FN reels via UDP 4731) ?
     * En mode no-canned (full-grgsm), tant qu'aucun SCH reel n'est arrive on ne
     * dispatch PAS le SB : le firmware FBSB attend le vrai SCH, comme un vrai
     * mobile. Pas de BSIC canne -> aucun masquage d'echec de decode. */
    static int no_canned = -1;
    if (no_canned < 0) {
        const char *e = getenv("CALYPSO_SHUNT_NO_CANNED");
        no_canned = (e && *e == '1') ? 1 : 0;
    }
    if (!g_shunt.sb_valid && no_canned) {
        static unsigned waitlog = 0;
        if (waitlog++ < 10)
            SHUNT_LOG("SB: pas encore de SCH reel (gr-gsm) "
                    "-> pas de dispatch (no-canned, le firmware attend)\n");
        return;
    }

    /* ---- FRAICHEUR DE LA SB (2026-08-03) -----------------------------------
     * DEFAUT CONSTATE : `sb_valid` ne redescend JAMAIS. Une fois qu'un seul SCH est
     * arrive, ce dispatch republie indefiniment le MEME (bsic, fn, toa) a chaque
     * tache SB, sur les deux pages de lecture. Mesure sur run sain : 7039 dispatches
     * pour 690 FN distinctes (x10 de replay) — benin, la SB a ~10 trames d'age.
     * Mesure sur un run degrade : `FN=20278` republie pendant que l'horloge trame
     * atteignait 22534, soit ~2250 trames (~10 s) de peremption, SANS AUCUN SIGNAL.
     * Le firmware recale son horloge sur cette FN a chaque re-sync : il se recale
     * donc dans le passe, et decroche. Candidat direct au « ca marche puis on perd
     * au bout de quelques secondes ».
     *
     * On mesure l'age TOUJOURS (diagnostic gratuit, plafonne). L'EXPIRATION, elle,
     * est gatee : CALYPSO_SHUNT_SB_MAX_AGE=<trames>, absente/0 = comportement
     * historique inchange. Quand elle est posee et l'age depasse, on ne dispatch
     * PAS — le firmware voit « pas de SB » et continue de chercher, comme un vrai
     * mobile, au lieu de recevoir une FN fausse. Meme philosophie que le no-canned
     * juste au-dessus : un echec VISIBLE plutot qu'un succes fabrique.
     * Valeur raisonnable : 104 (= 2 multitrames de 51, ~0,48 s ; le SCH arrive
     * toutes les 10 trames en regime normal, la marge est large). */
    if (g_shunt.sb_valid) {
        static int max_age = -1;
        if (max_age < 0) {
            const char *e = getenv("CALYPSO_SHUNT_SB_MAX_AGE");
            max_age = (e && *e) ? atoi(e) : 0;
            if (max_age > 0)
                SHUNT_LOG("SB-FRAICHEUR : peremption armee a %d trames "
                          "(au-dela -> pas de dispatch, echec visible)\n", max_age);
        }
        uint32_t age = calypso_trx_get_fn() - g_shunt.sb_capture_fn;   /* wrap-safe */
        int seuil = (max_age > 0) ? max_age : 104;   /* seuil de SIGNALEMENT */
        if ((int)age > seuil) {
            static unsigned stale_log = 0;
            if (stale_log++ < 20 || (stale_log % 500) == 0)
                SHUNT_LOG("SB PERIMEE : age=%u trames (SCH capture a trx_fn=%u, "
                          "FN publiee=%u) — %s\n", age, g_shunt.sb_capture_fn,
                          g_shunt.sb_fn,
                          (max_age > 0) ? "PAS de dispatch (peremption armee)"
                                        : "republiee quand meme (peremption NON armee, "
                                          "poser CALYPSO_SHUNT_SB_MAX_AGE=104)");
            if (max_age > 0)
                return;
        }
    }

    /* BSIC/FN : REELS (gr-gsm decode_sch) si dispo, sinon canned (legacy only).
     * FN -> {t1,t2,t3} GSM : T1=FN/(26*51), T2=FN%26, T3=FN%51 (encode_sb derive T3'). */
    uint8_t  bsic = g_shunt.sb_valid ? g_shunt.sb_bsic : SHUNT_CANNED_BSIC;
    uint32_t fn   = g_shunt.sb_valid ? g_shunt.sb_fn   : 0;
    uint16_t t1   = (uint16_t)(fn / (26u * 51u));
    uint8_t  t2   = (uint8_t)(fn % 26u);
    uint8_t  t3   = (uint8_t)(fn % 51u);

    /* a_sch[0] CRC bit clear = success (prim_fbsb.c:181, B_SCH_CRC=8).
     * CAN_CRC canné = on FORCE le pass (0). Non-canné = pas de faux succès :
     * sans vraie source CRC on écrit le bit d'échec → fail VISIBLE (le SB sera
     * rejeté) au lieu de masquer. Défaut canné → pass → camping inchangé. */
    shunt_write_w(rp + RP_A_SCH + 0 * 2,
                  (uint16_t)((shunt_is_canned(CAN_CRC) || g_shunt.sb_valid)
                             ? 0x0000 : B_SCH_CRC));   /* pass RÉEL ssi SCH décodé */

    /* sb = encode_sb(bsic, t1, t2, t3) → a_sch[3] | a_sch[4]<<16
     * (prim_fbsb.c:198). Two separate 16-bit stores, both LE. */
    uint32_t sb = shunt_encode_sb(bsic, t1, t2, t3);
    shunt_write_w(rp + RP_A_SCH + 3 * 2, (uint16_t)(sb & 0xFFFF));
    shunt_write_w(rp + RP_A_SCH + 4 * 2, (uint16_t)(sb >> 16));

    /* a_sch[1] / a_sch[2] are unused by l1s_decode_sb; zero them. */
    shunt_write_w(rp + RP_A_SCH + 1 * 2, 0x0000);
    shunt_write_w(rp + RP_A_SCH + 2 * 2, 0x0000);

    /* a_serv_demod[4] @ +0x10. read_sb_result reads from READ PAGE here,
     * NOT NDB (prim_fbsb.c:148-151). Chaque mesure cannée/0 selon CALYPSO_CANNED. */
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM)    ? SHUNT_CANNED_PM    : g_shunt.last_pm);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_ANGLE * 2, shunt_is_canned(CAN_ANGLE) ? SHUNT_CANNED_ANGLE : 0);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_SNR   * 2, shunt_snr_val());

    /* Ack on read page. */
    shunt_write_w(rp + RP_D_TASK_MD, SB_DSP_TASK);

    /* [2026-08-03] DEDUPE. Cette ligne pesait 62 % du journal QEMU (12 421 sur
     * 20 000 lignes releves), pour 55 ko/s au total : tmux, qui est mono-thread,
     * passait son temps a la rendre et le defilement devenait pateux.
     *
     * Le contenu est IDENTIQUE d'une trame a l'autre — c'est le meme SB republie
     * (cf. la note « fraicheur de la SB » plus haut : ~10 rejeux par SCH). On
     * n'imprime donc que ce qui CHANGE, plus un resume periodique pour garder la
     * cadence visible. Aucune information n'est perdue : chaque SB distinct
     * sort toujours. Diagnostic seul, zero effet sur le comportement. */
    {
        static uint32_t l_sb = 0xFFFFFFFFu; static uint8_t l_bsic = 0xFF;
        static uint32_t l_fn = 0xFFFFFFFFu; static unsigned long long rep = 0;
        if (sb != l_sb || bsic != l_bsic || fn != l_fn) {
            if (rep)
                SHUNT_LOG("DISPATCH SB × %llu (identique, non repete)\n", rep);
            rep = 0; l_sb = sb; l_bsic = bsic; l_fn = fn;
            SHUNT_LOG("DISPATCH SB page=%u → sb=0x%08x BSIC=%u FN=%u %s TOA=%d\n",
                page_idx, sb, bsic, fn,
                g_shunt.sb_valid ? "(gr-gsm REEL)" : "(canned legacy)", shunt_toa_val());
        } else if (++rep % 2000 == 0) {
            SHUNT_LOG("DISPATCH SB × %llu (meme SB rejoue, FN=%u)\n", rep, fn);
        }
    }
}

void shunt_dispatch_allc(uint8_t page_idx)
{
    /* @BEQUILLE — INJECT_ACD  (CALYPSO_INJECT_ACD=1, fallback CALYPSO_SHUNT_LEGIT=1)
     *   masque  : l'etage NB du DSP qui devrait remplir a_cd[0..14] (statut CRC + 23
     *             octets L2) apres demodulation d'un bloc CCCH/BCCH.
     *   retirer : quand le chemin natif correlateur -> NB -> a_cd livre le bloc.
     *   NB      : garde d'entree de shunt_dispatch_allc -> conditionne AUSSI toutes les
     *             bequilles AGCH / SDCCH / SACCH / BCCH ci-dessous.
     */
    /* [2026-07-30] CONVERTI a calypso_gate — un `=0` EXPLICITE doit couper.
     * Avant : `_ginj = (INJECT_ACD=='1'); if (!_ginj) _ginj = (SHUNT_LEGIT=='1');`
     * donc le parapluie ECRASAIT le 0 pose a la main. Mesure du 30/07 : profil
     * `native_twl` avec INJECT_ACD=0 ET FEED_SI=0 au manifeste, et les SI
     * arrivaient quand meme dans a_cd parce que SHUNT_LEGIT=1 avait fuite
     * (tmux fossilise l'env du 1er run). Le banc repondait a sa propre question
     * par les SI de gr-gsm. Desormais le parapluie n'est qu'un DEFAUT. */
    { static int _ginj = -1;
      if (_ginj < 0) { const char *_l = getenv("CALYPSO_SHUNT_LEGIT");
                       _ginj = calypso_gate("CALYPSO_INJECT_ACD", (_l && *_l == '1') ? 1 : 0); }
      if (!_ginj) return; }
    /* a_cd layout (cf osmocom-bb prim_rx_nb.c) :
     *   a_cd[0]   = FIRE status bits (B_FIRE0/B_FIRE1) -> 0x0000 = CRC pass
     *   a_cd[1]   = (reserved / BLUD bit)              -> 0x0000
     *   a_cd[2]   = num_biterr                          -> 0x0000
     *   a_cd[3..14] = 23 bytes L2 frame (SI3 here)
     */
    uint32_t addr_a_cd = BASE_API_NDB + NDB_A_CD;

    /* "sans hack" : CALYPSO_SHUNT_NO_CANNED=1 → on n'injecte JAMAIS le SI3
     * canned. Tant que le démod réel (bridge gr-gsm via feed_si) n'a rien
     * livré (si_valid=0), on ne dispatch rien → le firmware bail (pas de
     * DATA_IND) → le mobile ne campe QUE sur le VRAI SI décodé de l'I/Q du
     * BTS. C'est ça qui rend la victoire non-truquée : si le démod casse,
     * rien ne campe (le bug est visible, pas masqué par le canned). */
    static int no_canned = -1;
    if (no_canned < 0) {
        const char *e = getenv("CALYPSO_SHUNT_NO_CANNED");
        no_canned = (e && *e == '1') ? 1 : 0;
    }
    if (no_canned && !g_shunt.si_valid)
        return;

    /* === AGCH (#11) : IMM ASSIGN présenté dans a_cd sur un bloc CCCH ===========
     * Si un IMM ASSIGN est en attente, on le présente A LA PLACE du SI sur les
     * blocs CCCH (combiné CCCH+SDCCH4 : fn%51 ∈ {6-9,12-19}). Le firmware, sur son
     * read CCCH_COMB, tague chan_nr=0x90 -> gsm48_rr_rx_pch_agch -> rx_imm_ass ->
     * gsm48_match_ra. Présenté sur CHAQUE bloc CCCH tant que valide (TTL) : le
     * firmware le lit une fois, multi-présentation = robuste à l'alignement FN
     * (RR dédup via cr_hist). Les SI restent inchangés (blocs BCCH). Tunables :
     * CALYPSO_SHUNT_AGCH(=1 def), _AGCH_OFS (offset FN), _AGCH_TTL (ticks, def 100). */
    /* @BEQUILLE — SHUNT_AGCH (+ _OFS / _TTL)  (CALYPSO_SHUNT_AGCH, ON-sauf-0, defaut ON)
     *   masque  : le decodage CCCH/AGCH par le DSP. On ecrit l'IMM-ASSIGN directement
     *             dans a_cd (statut CRC pass force) sur chaque bloc CCCH de la fenetre
     *             fn%51, avec TTL maison, au lieu que le DSP produise un bloc decode.
     *   retirer : quand le correlateur natif alimente a_cd et pose a_cd[0] reel.
     *   NB      : SHUNT_AGCH_TTL a 3 consommateurs de semantiques differentes
     *             (peremption ici, expiry et fenetre de drop paging cote dsp_shunt.c).
     */
    static int agch_on = -1, agch_ofs = 0, agch_ttl = 100;
    if (agch_on < 0) {
        const char *e = getenv("CALYPSO_SHUNT_AGCH");     agch_on  = (!e || *e != '0') ? 1 : 0;
        const char *o = getenv("CALYPSO_SHUNT_AGCH_OFS"); agch_ofs = o ? atoi(o) : 0;
        const char *t = getenv("CALYPSO_SHUNT_AGCH_TTL"); if (t && *t) agch_ttl = atoi(t);
    }
    if (agch_on && g_shunt.agch_valid) {
        if ((uint32_t)(g_shunt.tick_cnt - g_shunt.agch_tick) > (uint32_t)agch_ttl) {
            g_shunt.agch_valid = false;                   /* périmé -> rendre la main aux SI */
        } else {
            /* gate sur le FN L1 FIRMWARE (l1s), pas s->fn : c'est l'horloge des
             * vrais blocs CCCH du firmware -> alignement run-invariant. */
            int tc = (int)((((long)shunt_l1s_fn() + agch_ofs) % 51 + 51) % 51);
            int is_ccch = (tc >= 6 && tc <= 9) || (tc >= 12 && tc <= 19);
            if (is_ccch) {
                uint32_t aa = BASE_API_NDB + NDB_A_CD;
                shunt_write_w(aa + 0, 0x0000);            /* a_cd[0] FIRE = CRC pass */
                shunt_write_w(aa + 2, 0x0000);
                shunt_write_w(aa + 4, 0x0000);
                const uint8_t *m = g_shunt.agch_buf;
                for (int i = 0; i < 23; i += 2) {
                    uint8_t lo = m[i], hi = (i + 1 < 23) ? m[i + 1] : 0x2B;
                    shunt_write_w(aa + 6 + i, lo | (hi << 8));
                }
                uint32_t rpA = rp_base(page_idx);
                shunt_write_w(rpA + RP_D_TASK_D,  ALLC_DSP_TASK);
                shunt_write_w(rpA + RP_D_BURST_D, shunt_burst_echo());
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM) ? SHUNT_CANNED_PM : g_shunt.last_pm);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_ANGLE * 2, 0);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_SNR   * 2, SHUNT_CANNED_SNR);
                static unsigned n_agch = 0;
                if (n_agch++ < 40 || (n_agch % 50) == 0)
                    SHUNT_LOG("DISPATCH AGCH IMM-ASS #%u burst_d=%u "
                            "tc=%d -> a_cd (chan_nr=0x90 attendu)\n",
                            n_agch, g_shunt.d_burst_d, tc);
                return;                                   /* ce dispatch = l'IMM ASSIGN */
            }
        }
    }

    /* === SDCCH/4 SS0 DL (#2) : UA/AUTH presente dans a_cd sur le bloc SDCCH/4 ===
     * Miroir EXACT de la branche AGCH ci-dessus. Si un bloc SDCCH DL est en
     * attente (feed_sdcch), on le presente A LA PLACE du SI sur le bloc SDCCH/4
     * SS0 (fn%51 in {22-25}). Le firmware (l1s_nb_cmd pose ALLC_DSP_TASK=24 pour
     * TOUS les NB DL, SDCCH inclus) tourne MF_TASK_SDCCH4_0 a ce FN -> tague
     * chan_nr=0x20 -> lapdm_dcch -> UA/AUTH -> L3. Gate sur shunt_l1s_fn() (FN L1
     * firmware), PAS calypso_trx_get_fn(), comme l'AGCH. Tunables :
     * CALYPSO_SHUNT_SDCCH(=1 def), _SDCCH_OFS (offset FN), _SDCCH_TTL (def 100). */
    /* @BEQUILLE — SHUNT_SDCCH (+ _RING / _OFS / _TTL / _MAXPRES)
     *              (CALYPSO_SHUNT_SDCCH, ON-sauf-0, defaut ON)
     *   masque  : idem AGCH pour le canal dedie — la ring rejoue le bloc L2 dans a_cd
     *             sur la fenetre fn%51 deduite de sdcch_ss, avec drop force apres
     *             MAXPRES presentations pour compenser un d_burst_d bloque.
     *   retirer : quand d_burst_d natif progresse 0->3 par bloc et que a_cd est
     *             alimente par le decodeur DSP.
     */
    static int sdcch_on = -1, sdcch_ofs = 0, sdcch_ttl = 4000;
    if (sdcch_on < 0) {
        const char *e = getenv("CALYPSO_SHUNT_SDCCH");     sdcch_on  = (!e || *e != '0') ? 1 : 0;
        const char *o = getenv("CALYPSO_SHUNT_SDCCH_OFS"); sdcch_ofs = o ? atoi(o) : 0;
        const char *t = getenv("CALYPSO_SHUNT_SDCCH_TTL"); if (t && *t) sdcch_ttl = atoi(t);
    }
    static int sdcch_ring_on = -1;
    if (sdcch_ring_on < 0) { const char *e = getenv("CALYPSO_SHUNT_SDCCH_RING"); sdcch_ring_on = (!e || *e != '0') ? 1 : 0; }
    /* [2026-07-27] g_shunt.sdcch_ss contient DIRECTEMENT la base DL fn%%51 de la
     * voie dediee assignee (SDCCH/4 ou /8), calculee dans feed_agch. Fenetre DL =
     * base..base+3 (NB_QUAD = 4 bursts). Remplace le hardcode SS0 (22-28). */
    /* [2026-07-27] fenetre-UNION : shunt_dispatch_allc n'est appele que quand le
     * firmware poste ALLC = a la sous-voie REELLE du mobile. On accepte donc l'UA
     * sur toute la region SDCCH du type de canal (pas besoin de deviner SS) :
     *   SDCCH/4 : fn%%51 [22,39] (SS0-3) ; SDCCH/8 : [0,31] (SS0-7). */
    /* [2026-08-21] RACINE DE « LA 1re LU DU RUN RATE ».
     * g_shunt.sdcch_ch8 demarre a FALSE et n'est arme que par set_dcch(), appelee
     * au PREMIER DATA_IND du canal dedie (l1ctl_sock : seul le flux firmware->mobile
     * est parse ; DM_EST_REQ est du code mort, le socket l1ctl est orphelin).
     * Tant qu'il est faux la fenetre vaut [22,39] = la region du SDCCH/4. Or l'UA
     * d'un SDCCH/8 arrive en fn%51 [0,3] : elle tombait HORS FENETRE, restait dans
     * l'anneau et se faisait evincer au TTL. Le mobile ne voyait pas l'UA et
     * retransmettait sa SABM ; la BTS, elle, avait deja etabli le lien et repondait
     * « SABM L>0 not expected in timer recovery state » SANS renvoyer d'UA -> la
     * tentative etait irrattrapable -> T200 x6 -> « Location update failed ».
     * Une seule fois par run, parce que calypso_dsp_shunt_l1_reset() ne remet PAS
     * sdcch_ch8 a false (defaut deja note l.2854 de calypso_dsp_shunt.c).
     *
     * TANT QUE LE TYPE N'EST PAS POSITIVEMENT CONNU, ON PREND L'UNION DES DEUX
     * REGIONS [0,39] au lieu de PARIER sur le /4. Cette fenetre n'est qu'un
     * garde-fou grossier : la selection fine de la sous-voie est faite par l'ALLC
     * que poste le firmware (cf. le commentaire juste au-dessus). Une fois arme,
     * on retrouve exactement le comportement d'avant. */
    int _armed = g_shunt.sdcch_ss_set;
    int _lo = _armed ? (g_shunt.sdcch_ch8 ? 0  : 22) : 0;
    int _hi = _armed ? (g_shunt.sdcch_ch8 ? 31 : 39) : 39;
    if (sdcch_on && sdcch_ring_on) {
        while (g_shunt.sdcch_ring_tail != g_shunt.sdcch_ring_head) {
            uint32_t hidx = g_shunt.sdcch_ring_head % SDCCH_RING_N;
            if ((uint32_t)(g_shunt.tick_cnt - g_shunt.sdcch_ring[hidx].tick) > (uint32_t)sdcch_ttl) {
                g_shunt.evict_ttl++;
                g_shunt.sdcch_ring[hidx].used = false; g_shunt.sdcch_ring_head++; continue;
            }
            int tc = (int)((((long)shunt_l1s_fn() + sdcch_ofs) % 51 + 51) % 51);
            if (!(tc >= _lo && tc <= _hi)) break;
            uint32_t aa = BASE_API_NDB + NDB_A_CD;
            shunt_write_w(aa + 0, 0x0000); shunt_write_w(aa + 2, 0x0000); shunt_write_w(aa + 4, 0x0000);
            const uint8_t *m = g_shunt.sdcch_ring[hidx].l2;
            for (int i = 0; i < 23; i += 2) { uint8_t lo = m[i], hi = (i + 1 < 23) ? m[i + 1] : 0x2B; shunt_write_w(aa + 6 + i, lo | (hi << 8)); }
            uint32_t rpA = rp_base(page_idx);
            shunt_write_w(rpA + RP_D_TASK_D,  ALLC_DSP_TASK);
            shunt_write_w(rpA + RP_D_BURST_D, shunt_burst_echo());
            shunt_write_w(rpA + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
            shunt_write_w(rpA + RP_A_SERV_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM) ? SHUNT_CANNED_PM : g_shunt.last_pm);
            shunt_write_w(rpA + RP_A_SERV_DEMOD + D_ANGLE * 2, 0);
            shunt_write_w(rpA + RP_A_SERV_DEMOD + D_SNR   * 2, SHUNT_CANNED_SNR);
            static unsigned n_sdcch = 0;
            if (n_sdcch++ < 60 || (n_sdcch % 50) == 0)
                SHUNT_LOG("DISPATCH SDCCH[ring] #%u fn=%u c=0x%02x burst_d=%u tc=%d depth=%u delta=%d\n",
                        n_sdcch, g_shunt.sdcch_ring[hidx].fn, m[1], g_shunt.d_burst_d, tc, g_shunt.sdcch_ring_tail - g_shunt.sdcch_ring_head,
                        (int)((int32_t)g_shunt.sdcch_ring[hidx].fn - (int32_t)shunt_l1s_fn()));
            if ((n_sdcch % 20) == 0)
                SHUNT_LOG("EVICT-STATS overflow=%u ttl=%u reps=%u\n", g_shunt.evict_overflow, g_shunt.evict_ttl, g_shunt.evict_reps);
            /* [2026-07-27] anti-stall : drop garanti apres MAXPRES presentations
             * meme si d_burst_d reste coince (mode DSP //) -> la ring draine, le
             * UA frais n'est plus bloque derriere les blocs perimes. */
            static int sdcch_maxpres = -1;
            if (sdcch_maxpres < 0) { const char *e = getenv("CALYPSO_SHUNT_SDCCH_MAXPRES"); sdcch_maxpres = (e && *e) ? atoi(e) : 8; }
            g_shunt.sdcch_ring[hidx].reps++;
            int _by_reps = (g_shunt.sdcch_ring[hidx].reps >= (uint16_t)sdcch_maxpres);
            if (g_shunt.d_burst_d >= 3 || _by_reps) {
                if (_by_reps && g_shunt.d_burst_d < 3) g_shunt.evict_reps++;
                g_shunt.sdcch_ring[hidx].used = false; g_shunt.sdcch_ring_head++;
            }
            if (g_shunt.sdcch_ring_tail == g_shunt.sdcch_ring_head) g_shunt.sdcch_valid = false;
            return;
        }
        if (g_shunt.sdcch_ring_tail == g_shunt.sdcch_ring_head) g_shunt.sdcch_valid = false;
    } else if (sdcch_on && g_shunt.sdcch_valid) {
        if ((uint32_t)(g_shunt.tick_cnt - g_shunt.sdcch_tick) > (uint32_t)sdcch_ttl) {
            g_shunt.sdcch_valid = false;
        } else {
            int tc = (int)((((long)shunt_l1s_fn() + sdcch_ofs) % 51 + 51) % 51);
            if (tc >= _lo && tc <= _hi) {
                uint32_t aa = BASE_API_NDB + NDB_A_CD;
                shunt_write_w(aa + 0, 0x0000); shunt_write_w(aa + 2, 0x0000); shunt_write_w(aa + 4, 0x0000);
                const uint8_t *m = g_shunt.sdcch_buf;
                for (int i = 0; i < 23; i += 2) { uint8_t lo = m[i], hi = (i + 1 < 23) ? m[i + 1] : 0x2B; shunt_write_w(aa + 6 + i, lo | (hi << 8)); }
                uint32_t rpA = rp_base(page_idx);
                shunt_write_w(rpA + RP_D_TASK_D,  ALLC_DSP_TASK);
                shunt_write_w(rpA + RP_D_BURST_D, shunt_burst_echo());
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM) ? SHUNT_CANNED_PM : g_shunt.last_pm);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_ANGLE * 2, 0);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_SNR   * 2, SHUNT_CANNED_SNR);
                if (g_shunt.d_burst_d >= 3) g_shunt.sdcch_valid = false;
                return;
            }
        }
    }

    /* === SACCH SDCCH/4 SS0 DL : presente le SI6 (B4) sur les slots SACCH du SS0 ===
     * Sinon le mobile lit du garbage sur la SACCH dediee -> 'Short header 0x07'.
     * Slots SACCH SS0 (combine CCCH+SDCCH/4, GSM 05.02) : fn%51 in {42-45} ET
     * (fn/51)%2==0. Gate CALYPSO_SHUNT_SACCH (def ON). */
    {
        /* @BEQUILLE — SHUNT_SACCH (+ _PAR / _OFS)  (CALYPSO_SHUNT_SACCH, ON-sauf-0, defaut ON)
         *   masque  : la presentation du bloc SACCH dedie que le DSP devrait demoduler ;
         *             on rejoue sacch_buf dans a_cd sur une fenetre tc calculee a la main.
         *   retirer : quand le chemin natif demodule la SACCH.
         */
        static int sacch_on = -1;
        if (sacch_on < 0) { const char *e = getenv("CALYPSO_SHUNT_SACCH"); sacch_on = (!e || *e != '0') ? 1 : 0; }
        if (sacch_on && g_shunt.sacch_have) {
            long fn = shunt_l1s_fn();
            int tc    = (int)(((fn % 51) + 51) % 51);
            int mf102 = (int)(((fn / 51) % 2 + 2) % 2);
            /* [2026-07-26] FIX RLF-SACCH : le gate parite dur mf102==0 est
             * FN-phase-fragile. l1s_fn se recale a l'execution (recale -556/-552),
             * ce qui INVERSE la parite -> la SACCH dediee SS0 n'est plus presentee
             * pendant des secondes (prouve : gap dispatch 723.28->732.18 avec
             * sacch_have=true + 11 feed_sacch dans le trou) -> le mobile mesure
             * rxlev=-110 sur ~50%% des blocs SACCH -> compteur radio-link epuise ->
             * 'Radio link is released' (RLF) = mort dominante des LU. On rend la
             * parite REGLABLE : CALYPSO_SHUNT_SACCH_PAR = 0(pair, legacy) / 1(impair)
             * / 2(les deux, def) ; le firmware ne consomme que sur SON vrai bloc
             * SACCH read, donc presenter sur les deux parites ne fait que garantir
             * la presence quelle que soit la phase FN. CALYPSO_SHUNT_SACCH_OFS decale
             * la fenetre tc si besoin. PAR=0 restaure le comportement d'origine. */
            /* @BEQUILLE — SHUNT_SACCH_PAR (+ _OFS)  (CALYPSO_SHUNT_SACCH_PAR, VALEUR, defaut 2)
             *   masque  : PAR=2 presente sur LES DEUX parites mf102 parce que le recale de
             *             shunt_l1s_fn INVERSE la parite en cours de run — la bequille compense
             *             une derive d'horloge, pas une absence de decodage.
             *   retirer : quand l1s_fn ne subit plus de recale a l'execution (parite mf102
             *             stable) ; alors PAR=0 (parite paire seule) redevient correct.
             */
            static int sac_par = -1, sac_ofs = 0;
            if (sac_par < 0) {
                const char *e = getenv("CALYPSO_SHUNT_SACCH_PAR"); sac_par = e ? atoi(e) : 2;
                const char *o = getenv("CALYPSO_SHUNT_SACCH_OFS"); sac_ofs = o ? atoi(o) : 0;
            }
            int tco = (int)((((long)fn + sac_ofs) % 51 + 51) % 51);
            int par_ok = (sac_par == 2) || (mf102 == sac_par);
            if (tco >= 42 && tco <= 46 && par_ok) {
                uint32_t aa = BASE_API_NDB + NDB_A_CD;
                shunt_write_w(aa + 0, 0x0000);
                shunt_write_w(aa + 2, 0x0000);
                shunt_write_w(aa + 4, 0x0000);
                const uint8_t *m = g_shunt.sacch_buf;
                for (int i = 0; i < 23; i += 2) {
                    uint8_t lo = m[i], hi = (i + 1 < 23) ? m[i + 1] : 0x2B;
                    shunt_write_w(aa + 6 + i, lo | (hi << 8));
                }
                uint32_t rpA = rp_base(page_idx);
                shunt_write_w(rpA + RP_D_TASK_D,  ALLC_DSP_TASK);
                shunt_write_w(rpA + RP_D_BURST_D, shunt_burst_echo());
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM) ? SHUNT_CANNED_PM : g_shunt.last_pm);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_ANGLE * 2, 0);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_SNR   * 2, SHUNT_CANNED_SNR);
                static unsigned n_sacch = 0;
                if (n_sacch++ < 20 || (n_sacch % 50) == 0)
                    SHUNT_LOG("DISPATCH SACCH SI6 #%u tc=%d -> a_cd\n", n_sacch, tc);
                return;
            }
        }
    }

    /* [FIX #3 corrige] Bloque le SI du camp UNIQUEMENT quand un SDCCH DEDIE est en
     * attente (ring non-vide). PAS agch_valid : le PAGING (feed_agch) est continu
     * pendant le camp et sa branche AGCH presente sur des blocs CCCH (!= blocs BCCH
     * du SI) -> le bloquer tuait le SI -> camp casse. Le SDCCH n'est actif qu'en
     * mode dedie (post-IMM-ASSIGN) -> sdcch_valid faux pendant le camp -> SI passe. */
    if (g_shunt.sdcch_valid)
        return;

    /* (A) ROTATION par bloc : au début du bloc (burst 0) on avance au prochain
     * type SI disponible et on le copie dans si_buf (STABLE pour les 4 bursts).
     * Le mobile collecte ainsi TOUT le set (SI1/2/3/4) au fil des blocs au lieu
     * du seul SI3. Round-robin = aucune dépendance FN (jitter-proof). */
    if (g_shunt.d_burst_d == 0) {
        for (int k = 1; k <= 6; k++) {
            int s = (g_shunt.si_rr + k) % 6;
            if (g_shunt.si_set_have[s]) {
                memcpy(g_shunt.si_buf, g_shunt.si_set[s], 23);
                g_shunt.si_rr = s;
                break;
            }
        }
    }

    /* #12 ORDONNANCEMENT BCCH (no-hack) : présenter le SI UNIQUEMENT sur les
     * blocs BCCH du multiframe-51 (TC = fn%51 ∈ [2,5]). Sur un bloc CCCH le SI3
     * fuiterait en PCH/AGCH ("Unknown PCH/AGCH message"). d_fn = vraie FN (#4).
     * Gated CALYPSO_SHUNT_BCCH_SCHED (défaut 1). */
    /* @BEQUILLE — SHUNT_BCCH_SCHED (+ _OFS)  (CALYPSO_SHUNT_BCCH_SCHED, EQ1, defaut OFF)
     *   masque  : l'ordonnancement mf-51 que le DSP devrait imposer — on filtre a la
     *             main TC dans [2,5] pour eviter que le SI3 fuite en PCH/AGCH, avec une
     *             garde anti-famine qui degrade en "SI partout" apres 200 dispatches.
     *   retirer : quand le dispatcher natif presente a_cd sur le bon type de bloc.
     *   ATTENTION : le commentaire d'en-tete annonce "defaut 1" — PERIME, le code teste
     *             (e && *e=='1') donc le defaut est OFF.
     */
    static int bcch_sched = -1, bcch_ofs = 0;
    if (bcch_sched < 0) {
        const char *e = getenv("CALYPSO_SHUNT_BCCH_SCHED");
        bcch_sched = (e && *e == '1') ? 1 : 0;        /* DEFAUT OFF (chan_nr pas le gate du camping) */
        const char *o = getenv("CALYPSO_SHUNT_BCCH_OFS");
        bcch_ofs = o ? atoi(o) : 0;
    }
    if (bcch_sched) {
        /* FN = le device (vraie FN GSM de la BTS, alignée mf-51), PAS d_fn
         * (que le firmware laisse à 0). Bloc BCCH non-combiné C0T0 = TC ∈ [2,5]
         * (FCCH@0/10/.., SCH@1/11/.., BCCH@2-5, CCCH@6-9/12-15..). Offset
         * réglable CALYPSO_SHUNT_BCCH_OFS si l'alignement dispatch≠bloc. */
        static unsigned long n_disp = 0, n_bcch = 0, n_since_bcch = 0;
        int tc = (int)((((long)calypso_trx_get_fn() + bcch_ofs) % 51 + 51) % 51);
        int is_bcch = (tc >= 2 && tc <= 5);
        n_disp++;
        if (is_bcch) { n_bcch++; n_since_bcch = 0; } else n_since_bcch++;
        if ((n_disp % 51) == 0)
            SHUNT_LOG("#12 BCCH-sched: %lu disp / %lu BCCH "
                    "(tc=%d ofs=%d)\n", n_disp, n_bcch, tc, bcch_ofs);
        /* Garde anti-famine : grace au boot (200 disp) + si 0 BCCH depuis 102
         * dispatches (désalignement total) on présente quand même → dégrade
         * vers "SI partout" au lieu de famine totale. */
        if (!is_bcch && n_disp > 200 && n_since_bcch < 102) {
            uint32_t addr0 = BASE_API_NDB + NDB_A_CD;
            uint32_t rp_c  = rp_base(page_idx);
            shunt_write_w(addr0 + 0, 0x0003);          /* a_cd[0] FIRE = CRC fail */
            shunt_write_w(rp_c + RP_D_TASK_D,  ALLC_DSP_TASK);
            shunt_write_w(rp_c + RP_D_BURST_D, shunt_burst_echo());
            return;                          /* pas de SI sur le CCCH */
        }
    }

    /* a_cd[0..2] = status words. CAN_CRC canné = CRC pass (0) ; non-canné =
     * pas de faux pass → FIRE=fail (0x0003) visible. a_cd[1/2] biterr = 0. */
    shunt_write_w(addr_a_cd + 0,
                  (shunt_is_canned(CAN_CRC) || g_shunt.si_valid) ? 0x0000 : 0x0003);  /* a_cd[0] FIRE : pass RÉEL ssi SI décodé */
    shunt_write_w(addr_a_cd + 2, 0x0000);  /* a_cd[1] */
    shunt_write_w(addr_a_cd + 4, 0x0000);  /* a_cd[2] */

    /* a_cd[3..14] = 23B L2 frame, packé en 12 mots LE.
     * Source : le SI RÉEL démodulé (gr-gsm ou C natif via feed_si) si dispo,
     * sinon le SI3 canned (fallback). C'est le swap canned→réel = le "sans hack". */
    const uint8_t *si = g_shunt.si_buf;  /* no-hack : vrai SI grgsm seulement */
    for (int i = 0; i < 23; i += 2) {
        uint8_t lo = si[i];
        uint8_t hi = (i + 1 < 23) ? si[i + 1] : 0x2B;
        uint16_t w = lo | (hi << 8);
        shunt_write_w(addr_a_cd + 6 + i, w);   /* +6 = a_cd[3] base */
    }

    /* IMPORTANT : firmware prim_rx_nb.c:79 fait
     *   if (db_r->d_burst_d != burst_id) return 0;
     * et attend la sequence burst 0,1,2,3 pour assembler la frame.
     * On echo le d_burst_d que l'ARM a poste dans la read page pour que
     * le check passe. Sinon le firmware bail avant dsp_memcpy_from_api()
     * et n'envoie JAMAIS L1CTL_DATA_IND. */
    /* [2026-07-22] DUAL-PAGE : le fix d'offset d_dsp_page a rendu page_idx
     * alternant (avant il etait fige a 0 via le garbage 0xf600 = w_page&1). Or le
     * mobile lit db_r[r_page] (r_page toggle INDEPENDAMMENT du w_page porte par
     * d_dsp_page). On ecrit donc les champs read-page sur LES DEUX pages -> le
     * mobile les lit quel que soit r_page. Gate CALYPSO_SHUNT_DUAL_PAGE (def ON). */
    /* @BEQUILLE — SHUNT_CANNED + SHUNT_DUAL_PAGE  (CALYPSO_SHUNT_CANNED, EXISTS -> "=0"
     *              l'ACTIVE ; CALYPSO_SHUNT_DUAL_PAGE, ON-sauf-0, defaut ON)
     *   masque  : SHUNT_CANNED remplace a_serv_demod[PM] et [SNR] mesures par des
     *             constantes. SHUNT_DUAL_PAGE ecrit les champs read-page sur LES DEUX
     *             pages parce que le basculement de r_page cote lecture n'est pas
     *             modelise (on ne sait pas quelle page le mobile va lire).
     *   retirer : PM/SNR quand ils viennent du modele RF ou du DSP ; DUAL_PAGE quand
     *             r_page est deduit du protocole et non devine.
     */
    static int canned_on = -1, dual = -1;
    if (canned_on < 0) canned_on = calypso_gate("CALYPSO_SHUNT_CANNED", 0);
    if (dual < 0) { const char *ed = getenv("CALYPSO_SHUNT_DUAL_PAGE"); dual = (ed && *ed == '0') ? 0 : 1; }
    for (int pg = 0; pg < 2; pg++) {
        if (!dual && pg != page_idx) continue;
        uint32_t rp = rp_base(pg);
        shunt_write_w(rp + RP_D_TASK_D,  ALLC_DSP_TASK);
        shunt_write_w(rp + RP_D_BURST_D, shunt_burst_echo());
        shunt_write_w(rp + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
        shunt_write_w(rp + RP_A_SERV_DEMOD + D_PM    * 2,
                      (canned_on || shunt_is_canned(CAN_PM)) ? SHUNT_CANNED_PM : g_shunt.last_pm);
        shunt_write_w(rp + RP_A_SERV_DEMOD + D_ANGLE * 2, shunt_is_canned(CAN_ANGLE) ? SHUNT_CANNED_ANGLE : 0);
        shunt_write_w(rp + RP_A_SERV_DEMOD + D_SNR   * 2,
                      (canned_on || shunt_is_canned(CAN_SNR)) ? SHUNT_CANNED_SNR : g_shunt.rx_snr);
    }

    SHUNT_LOG("DISPATCH ALLC page=%u burst_d=%u -> SI3 a_cd[3..14] + a_serv_demod %s\n",
        page_idx, g_shunt.d_burst_d, canned_on ? "CANNED(hack)" : "reel");
}

/* ---- DISPATCH PM : tâche power-measurement (md=1). Écrit a_pm[3] @ +0x18,
 * que le power scan (l1s pm_cmd) lit pour dériver le rxlev. Sans ça a_pm=0 →
 * rxlev=-110 (plancher) → la cellule est rejetée AVANT même la sync, quel que
 * soit le SI. Valeur réglable via CALYPSO_SHUNT_PM (défaut SHUNT_CANNED_PM,
 * haut → rxlev fort). C'est le pendant "scan" du PM canné FB/SB. ---- */
void shunt_dispatch_pm(uint8_t page_idx)
{
    uint32_t rp = rp_base(page_idx);
    int pm_val;
    {
        /* [2026-07-26 RANK5] a_pm calibre via le modele trf6151 (gain vivant
         * suivi par TSP) : a_pm = (rf_cible + system_inherent_gain + trf_gain)*64,
         * de sorte que le firmware rapporte rf_cible dBm (rxlev fort) quel que
         * soit le gain que l'AGC programme. CALYPSO_TRF_TARGET_RF (defaut -60).
         * Legacy : CALYPSO_SHUNT_PM force une valeur brute a_pm (bypass modele). */
        /* @BEQUILLE — SHUNT_PM  (CALYPSO_SHUNT_PM, VALEUR, defaut -1 = modele)
         *   masque  : la mesure de puissance (tache PM md=1). Une valeur brute decretee est
         *             ecrite dans a_pm[0..2], court-circuitant meme le modele trf6151.
         *   retirer : quand le DSP produit a_pm depuis l'I/Q.
         *   NB      : shunt_dispatch_pm n'a AUCUN gate INJECT_* — cette bequille est vivante
         *             en NATIVE et NATIVE_HELPED ; seul SHUNT_NO_FAKE_PM=1 la coupe.
         */
        static int raw = -2;           /* -2 = pas encore lu */
        if (raw == -2) {
            const char *e = getenv("CALYPSO_SHUNT_PM");
            raw = (e && *e) ? (int)strtol(e, NULL, 0) : -1;   /* -1 = utiliser le modele */
        }
        if (raw >= 0) {
            pm_val = raw;
        } else {
            /* @BEQUILLE — TRF_RXLEV + TRF_TARGET_RF  (CALYPSO_TRF_RXLEV=1, fallback
             *              CALYPSO_SHUNT_LEGIT=1 ; cible defaut -60 dBm)
             *   masque  : a_pm que le vrai DSP ecrit a 0 (aucune mesure). On substitue
             *             apm_for_rf(TARGET_RF) : le niveau RF cible est une constante decretee.
             *   retirer : quand a_pm natif est non nul. Le modele trf6151 (gain suivi par TSP)
             *             reste legitime — seule la CIBLE figee est la bequille.
             */
            static int trf = -1, target = -60;
            if (trf < 0) {
                /* [2026-08-03] `CALYPSO_TRF_RXLEV=0` ne coupait pas sous
                 * SHUNT_LEGIT=1 : le parapluie ecrasait un 0 explicite. Il
                 * devient un DEFAUT (cf. calypso_c54x.c, meme correction). */
                const char *t = getenv("CALYPSO_TRF_TARGET_RF");
                const char *l = getenv("CALYPSO_SHUNT_LEGIT");
                trf = calypso_gate("CALYPSO_TRF_RXLEV", (l && *l == '1') ? 1 : 0);
                if (t && *t) target = atoi(t);
            }
            pm_val = trf ? calypso_trf6151_apm_for_rf(target) : SHUNT_CANNED_PM;
        }
    }
    shunt_write_w(rp + RP_A_PM + 0 * 2, (uint16_t)pm_val);
    shunt_write_w(rp + RP_A_PM + 1 * 2, (uint16_t)pm_val);
    shunt_write_w(rp + RP_A_PM + 2 * 2, (uint16_t)pm_val);
    shunt_write_w(rp + RP_D_TASK_MD, PM_DSP_TASK);
    static unsigned pm_log = 0;
    if (pm_log++ < 5)
        SHUNT_LOG("DISPATCH PM page=%u → a_pm[0..2]=0x%04x (rxlev)\n",
                page_idx, (uint16_t)pm_val);
}
