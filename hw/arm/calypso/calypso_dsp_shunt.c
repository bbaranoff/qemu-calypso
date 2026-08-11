/*
 * calypso_dsp_shunt.c — DSP-side mock honoring the ARM↔DSP API-RAM contract.
 *
 * When CALYPSO_DSP_SHUNT=1, the c54x emulator is skipped entirely (no opcode
 * execution, no INTM gymnastics, no DARAM-side compute). This file replaces
 * the DSP by a thin state machine that respects the only protocol the ARM
 * firmware actually sees:
 *
 *   1. ARM writes a task descriptor into W_PAGE_(w_page) — d_task_d /
 *      d_task_md / d_task_ra / d_burst_d / d_fn / ...
 *   2. ARM signals "go" by writing 0xFFD001A8 (NDB+0 = d_dsp_page) with
 *      bit 1 (B_GSM_TASK) set; bit 0 carries the page index.
 *   3. DSP (= us) consumes the task, computes the result, writes:
 *        - FB result into NDB:  d_fb_det @+0x48, a_sync_demod[4] @+0x4C
 *        - SB result into R_PAGE_(page_idx): a_sch[5] @ +0x1E, a_serv_demod
 *          [4] @ +0x10
 *      then the result is visible at the NEXT TDMA frame.
 *   4. No separate "DSP done" IRQ: the TPU FRAME IRQ (INTH bit 4) ticks
 *      every 1ms and the ARM polls there.
 *
 * Design notes (review by c-web 2026-05-26):
 *   - Latch on write to NDB+0, but SERVICE on the next FRAME IRQ tick.
 *     This respects the ARM firmware's poste-then-wait-frame model and
 *     gives multi-frame tasks (FB search) a natural cadence.
 *   - Disjoint write surfaces: FB goes to NDB only, SB goes to READ PAGE
 *     only. The fw's read sites (prim_fbsb.c:181/198/306/404) are the
 *     ground truth.
 *   - Offsets are DWARF-validated against THE container ELF
 *     (/opt/GSM/firmware/board/compal_e88/layer1.highram.elf — sha256
 *     27cd04...). NOT the host build — the container build was the one
 *     loaded by run.sh `-kernel`. Same offsets confirmed across both.
 *   - Canned phase 1 = dispatch each post on next FRAME IRQ. No
 *     simulated wide→narrow FB search; angle=0 keeps AFC loop from
 *     iterating. TOA tuned so synchronize_tdma yields bits_delta≈0.
 *   - ALLC/NB UL/RA UL = LOG_UNIMP. We don't need them to clear
 *     FBSB_CONF — those are downstream of the current wall.
 */

#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"
#include "sysemu/dma.h"
#include "qemu/main-loop.h"
#include "calypso_dsp_shunt.h"
#include "hw/arm/calypso/calypso_trf6151.h"
#include "hw/arm/calypso/calypso_twl3025.h"
#include "calypso_c54x.h"   /* C54xState + c54x_bsp_load/run/interrupt_ex/wake (CALYPSO_DSP=c54x route) */
#include "calypso_layer1.h" /* calypso_l1_c_active() : ungate SB/SI (+FB) sous CALYPSO_L1=c */
#include "hw/arm/calypso/calypso_dsp_internal.h" /* shared state + NDB-write primitives (split) */
extern int g_c54x_int3_src;  /* diag source INT3 (RO) */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

/* FN TDMA reelle (calypso_trx.c) pour recoder la FN du shunt (LATCH d_fn=0) :
 * declaree dans calypso_dsp_internal.h (partagee avec le helper). */
extern void l1ctl_inject_dl_si(const uint8_t *l2, int l2len, uint32_t fn);
/* FN-FIX : FN du dernier L1CTL_RACH_CONF (= memo exact du mobile), capture dans
 * l1ctl_sock.c au moment de l'envoi au mobile (race-free vs last_rach.fn@0x836500). */
extern volatile uint32_t g_last_rach_conf_fn;
extern volatile uint32_t g_rach_conf_fn[256];   /* per-ra : FN exact du RACH_CONF keye par ra (defini l1ctl_sock.c) */

struct dsp_shunt_state g_shunt;

/* [2026-07-27] Value-list CALYPSO_SHUNT_LEGIT / CALYPSO_SHUNT_NO_LEGIT :
 *   =1              -> mode nu (rien de plus)
 *   =DSP            -> lance AUSSI le DSP c54x en // (CALYPSO_DSP_RUN_C54X=1)
 *   =NO_CANNED      -> mode sans cannes (CALYPSO_SHUNT_NO_CANNED=1)
 *   =DSP,NO_CANNED  -> les deux (virgule/espace, casse libre)
 * Normalise UNE fois au chargement (avant tout getenv cache statique) puis
 * canonicalise la base a "1" pour que les ~20 checks existants (*e=='1') restent
 * valides -> aucun site a modifier. */
/* @BEQUILLE — SHUNT_LEGIT / SHUNT_NO_LEGIT (parapluies)  (CALYPSO_SHUNT_LEGIT,
 *              CALYPSO_SHUNT_NO_LEGIT ; EQ1 apres canonicalisation)
 *   masque  : le mur RANK3 — le correlateur natif n'ecrit jamais d_fb_det. Les
 *             deux parapluies transportent la detection reelle de gr-gsm vers le
 *             resultat DSP (api_ram[0x08F8..0x08FD]), forcent d_fb_det et a_pm cote
 *             c54x, falsifient d_task_d / d_burst_d cote lecture ARM, et servent de
 *             FALLBACK implicite a une dizaine d'autres gates (INJECT_*, FEED_SI,
 *             TRF_RXLEV, UL_RACH_FROM_DRACH, SHUNT_REAL_FB).
 *   retirer : quand le correlateur natif pose d_fb_det (RANK3 leve).
 *   ATTENTION : ce constructeur fait des setenv() AVANT main() — "=DSP" cree
 *             CALYPSO_DSP_RUN_C54X=1 et "=NO_CANNED" cree CALYPSO_SHUNT_NO_CANNED=1.
 *             Ces variables apparaissent au manifeste sans avoir ete tapees.
 *   NB      : SHUNT_NO_LEGIT n'est PAS un synonyme — seuls 5 sites l'acceptent,
 *             les autres ne retombent que sur SHUNT_LEGIT.
 */
static void __attribute__((constructor)) shunt_env_value_list(void)
{
    static const char *keys[2] = { "CALYPSO_SHUNT_LEGIT", "CALYPSO_SHUNT_NO_LEGIT" };
    for (int k = 0; k < 2; k++) {
        const char *v = getenv(keys[k]);
        if (!v || !*v || !strcmp(v, "0"))
            continue;   /* absent / off */
        if (strstr(v, "DSP") || strstr(v, "dsp"))
            setenv("CALYPSO_DSP_RUN_C54X", "1", 1);        /* lance le c54x en // */
        if (strstr(v, "NO_CANNED") || strstr(v, "no_canned"))
            setenv("CALYPSO_SHUNT_NO_CANNED", "1", 1);     /* mode sans cannes */
        /* [2026-08-03] NO_CANNED EST DESORMAIS LE DEFAUT SOUS PARAPLUIE.
         * Avant, il fallait l'ecrire : `CALYPSO_SHUNT_LEGIT=NO_CANNED`. Un simple
         * `=1` (ou une surcharge CLI, l'idiome `:=` laissant la CLI gagner) faisait
         * retomber le banc en mode canne SANS LE DIRE — c'est-a-dire avec des
         * sorties DSP fabriquees (d_fb_det=1, TOA=23, PM/SNR=0x7000) qui masquent
         * les echecs de decode. Le defaut sur = pas de valeur fabriquee.
         * Seul un `=0` EXPLICITE re-canne ; une variable absente OU VIDE compte ici
         * comme non posee (on ne peut pas distinguer "vide" de "declaree" dans les
         * .env, cf. armdsp.env qui ecrit `: "${CALYPSO_SHUNT_NO_CANNED:=}"`). */
        {
            const char *nc = getenv("CALYPSO_SHUNT_NO_CANNED");
            if (!nc || !*nc)
                setenv("CALYPSO_SHUNT_NO_CANNED", "1", 1);
        }
        setenv(keys[k], "1", 1);   /* canonicalise -> checks *e=='1' OK */
    }
    /* Manifeste de run : dump les CALYPSO_* EFFECTIVES (post value-list) en tete
     * de log. Reproductibilite : ce constructeur fait des setenv() AVANT main()
     * -> la config effective differe de celle tapee ; on la trace. */
    {
        /* [2026-07-27] C1 BUILD-STAMP : le 27/07 le binaire vivant a ete rebuilde
     * PENDANT le run (inode supprime) et des heures ont ete perdues a comparer
     * des sources disque avec un binaire different. L estampille de compilation
     * rend la confusion impossible : elle est DANS le binaire qui tourne. */
    fprintf(stderr, "[calypso-manifest] BUILD-STAMP compile le %s %s\n",
            __DATE__, __TIME__);
    fprintf(stderr, "[calypso-manifest] ===== CALYPSO_* effectives (post value-list) =====\n");
        for (char **e = environ; e && *e; e++)
            if (!strncmp(*e, "CALYPSO_", 8))
                fprintf(stderr, "[calypso-manifest] %s\n", *e);
        fprintf(stderr, "[calypso-manifest] =================================================\n");
    }
}

/* SONDE B : table RA -> FN L1 firmware (l1s.current_time.fn) au moment de la RACH.
 * Remplie par calypso_trx.c (hook write d_rach). Sert à réécrire la req-ref de
 * l'IMM ASSIGN au FN exact que le mobile a mémorisé (preuve que le FN = dernier mur). */
static uint32_t g_rach_l1s_fn[256];
volatile uint8_t g_last_recorded_ra = 0;   /* per-ra FN-FIX : ra de la derniere RACH (lu par l1ctl_sock.c) */
static uint8_t  g_rach_l1s_valid[256];
/* [2026-08-08] Fenetre de presentation du SDCCH, posee par la SOURCE AUTORITAIRE.
 *
 * g_shunt.sdcch_ss etait pose dans feed_agch, donc par N'IMPORTE QUEL IMM ASSIGN
 * traversant le CCCH — y compris ceux destines aux autres abonnes (mesure du
 * 08/08 : 68 IMM ASSIGN de RA=0x07 et 12 de RA=0x0a pour un RACH a nous de
 * RA=0x08). La fenetre ou le shunt presente a_cd suivait donc le trafic des
 * voisins, et le descendant du canal dedie tombait a cote.
 *
 * Appele depuis l1ctl_sock quand le FIRMWARE annonce son propre chan_nr
 * (DATA_CONF / DATA_IND). kind : 0 = SDCCH/4 combine, 1 = SDCCH/8.
 * Base DL fn%51 : /4 -> {22,26,32,36}[ss] ; /8 -> ss*4. */
void calypso_dsp_shunt_set_dcch(int kind, int ss);   /* -Werror=missing-prototypes */
void calypso_dsp_shunt_set_dcch(int kind, int ss)
{
    static const uint8_t b4[4] = { 22, 26, 32, 36 };
    uint8_t base = kind ? (uint8_t)((ss & 7) * 4) : b4[ss & 3];
    /* [2026-08-09] LA GARDE S ARME AVANT LE RETOUR ANTICIPE.
     * Elle etait posee plus bas, apres le « if inchange » : a la DEUXIEME
     * ouverture d un dedie sur la MEME sous-voie, la fonction sortait avant de
     * l atteindre et la fenetre restait ouverte. Mesure : mobile passe en dedie
     * 2 fois, une seule ligne DCCH-WINDOW, 10 « 0x07 » toujours la.
     * Cet appel signale une OUVERTURE de canal, pas un changement de sous-voie —
     * la garde doit donc suivre l appel, pas la comparaison. */
    /* [2026-08-09] ARMEMENT RETIRE D ICI. Il posait le drapeau a la main, sans
     * journaliser : on lisait alors 0 armement pour 121 peremptions, un compte
     * indecidable qui m a fait accuser la mauvaise piece. L armement passe
     * desormais par la seule porte qui parle -- calypso_dsp_shunt_set_dcch_active,
     * appelee par l1ctl_sock sur CHAQUE bloc dedie, donc juste avant cet appel-ci.
     * Y toucher ici serait redondant et remuet. */
    if (g_shunt.sdcch_ss_set && g_shunt.sdcch_ss == base
        && g_shunt.sdcch_ch8 == (kind != 0))
        return;                                  /* sous-voie inchangee */
    g_shunt.sdcch_ss     = base;
    g_shunt.sdcch_ss_set = true;
    g_shunt.sdcch_ch8    = (kind != 0);
    SHUNT_LOG("DCCH-WINDOW : SDCCH/%d SS=%d -> presentation a_cd sur fn%%51 %u-%u "
              "(source unique = chan_nr du firmware ; feed_agch n'y touche plus)\n",
              kind ? 8 : 4, ss, base, base + 3);
}

void calypso_dsp_shunt_record_rach(uint8_t ra)
{
    if (!g_shunt.active) return;
    g_rach_l1s_fn[ra]    = shunt_l1s_fn();
    g_rach_l1s_valid[ra] = 1;
    g_last_recorded_ra   = ra;   /* per-ra FN-FIX : permet a l1ctl_sock de keyer le RACH_CONF par ra */
}

/* SDCCH/SACCH UL sideband (#12) : QEMU publie la L2 montante (a_cu[3..], 23o) vers
 * qemu_wrap via /dev/shm/calypso_sdcch_ul (fichier régulier, pas un FIFO). qemu_wrap
 * l'encode (gsm0503_xcch) + module (burst normal TSC7) + injecte sur le slot UL.
 * Layout 48o : seq@0(u32) l1s_fn@4(u32) fn@8(u32) task_u@12(u16) l1s%51@14(u8) l2[23]@16. */
static void calypso_sdcch_ul_publish(const uint8_t *l2, uint16_t task_u,
                                     uint32_t fn, uint32_t l1s_fn)
{
    static int fd = -2;
    if (fd == -2) {
        fd = open("/dev/shm/calypso_sdcch_ul", O_CREAT | O_RDWR, 0644);
        if (fd >= 0 && ftruncate(fd, 48) < 0) { /* best-effort */ }
    }
    if (fd < 0) return;
    /* #2 PUBLISH-NO-IDLE : NE PAS republier la trame de remplissage (UI, ctrl=0x03).
     * Le firmware poste pu_get_idle_frame()=01 03 01 dans a_cu entre les bursts SABM
     * (burst_id==0, rien en file L23). Chaque publish bumpait seq -> ecrasait la SABM
     * transitoire (~4 frames) dans la slot unique du sideband AVANT que le consommateur
     * (qemu_wrap, 1 pread/frame) ne l'echantillonne -> seul l'idle remontait, jamais la
     * SABM (01 3f) -> osmo-bts jamais de SABM -> jamais d'UA -> T200xN200 -> RR released.
     * En ne publiant QUE les trames signalisantes (ctrl != 0x03), tout nouveau seq est
     * porteur et ne peut plus etre clobbere par le fill -> la SABM tient jusqu'a ce que
     * le consommateur sticky la capture. CALYPSO_UL_PUB_IDLE=1 retablit l'ancien comportement. */
    /* @BEQUILLE — UL_PUB_IDLE  (CALYPSO_UL_PUB_IDLE, EQ1, defaut OFF = filtre ACTIF)
     *   masque  : la profondeur du sideband UL (slot unique, 1 pread/frame cote
     *             qemu_wrap). Le filtre par defaut (ne pas publier les trames de fill
     *             ctrl==0x03) empeche l'ecrasement de la SABM transitoire — il compense
     *             l'absence de file.
     *   retirer : quand le sideband UL est une file (ring) et non un slot unique ;
     *             alors publier l'idle redevient sans risque.
     */
    static int pub_idle = -1;
    if (pub_idle < 0) { const char *e = getenv("CALYPSO_UL_PUB_IDLE"); pub_idle = (e && *e == '1') ? 1 : 0; }
    if (!pub_idle && l2[1] == 0x03) return;   /* trame de fill (UI) : ne pas ecraser la signalisation */
    static uint32_t seq = 0; seq++;
    uint8_t buf[48] = {0};
    memcpy(buf + 4,  &l1s_fn, sizeof(l1s_fn));
    memcpy(buf + 8,  &fn,     sizeof(fn));
    memcpy(buf + 12, &task_u, sizeof(task_u));
    buf[14] = (uint8_t)(l1s_fn % 51);
    memcpy(buf + 16, l2, 23);
    memcpy(buf + 0,  &seq, sizeof(seq));   /* seq en dernier (publication) */
    if (pwrite(fd, buf, sizeof(buf), 0) < 0) { /* best-effort */ }
}

static uint16_t shunt_pm_decan_apm(int fallback_target);   /* fwd : defini plus bas */

/* Coherence interne des #define de repli (l'arithmetique NDB_W, pas l'accord
 * avec le firmware — c'est le role du resolveur DWARF ci-dessous). */
QEMU_BUILD_BUG_ON(!(NDB_W_CHECK));

/* ═══════════════════════════════════════════════════════════════════════════
 * OFFSETS NDB RESOLUS DU DWARF DU FIRMWARE VIVANT (2026-08-08)
 *
 * Un #define ne peut pas savoir que le firmware a ete recompile. Si la struct
 * T_NDB_MCU_DSP bouge, QEMU ecrit A COTE **sans rien dire**, et le symptome
 * sort tres loin en aval : le 2026-06-02, a_cd suppose a 0x1DC donnait
 * num_biterr=0xff + CRC fail, et il a fallu des jours pour remonter jusqu'a
 * l'offset. Le seul juge d'une struct est le binaire qui la contient.
 *
 * On lit donc les offsets au demarrage, dans le DWARF de l'ELF reellement
 * charge (`-kernel`), via binutils-arm-none-eabi. Meme principe que
 * shunt_fw_sym(), qui resout deja les SYMBOLES du firmware pour la meme raison
 * — ceci en est le pendant pour les CHAMPS.
 *
 * Politique en cas d'echec : on GARDE les #define et on le DIT fort. Un repli
 * silencieux redonnerait exactement le mode de panne qu'on cherche a tuer.
 * ═══════════════════════════════════════════════════════════════════════════ */
struct ndb_offsets {
    uint32_t a_cd, a_fd, a_dd_0, a_cu, a_fu, a_du_1;
    bool     resolved;
};
static struct ndb_offsets g_ndb = {
    .a_cd = NDB_A_CD, .a_fd = NDB_A_FD, .a_dd_0 = NDB_A_DD_0,
    .a_cu = NDB_A_CU, .a_fu = NDB_A_FU, .a_du_1 = NDB_A_DU_1,
    .resolved = false,
};

/* Adresse MOT dans c54x->data[] d'un offset NDB (version runtime de NDB_W). */
static inline unsigned ndb_w(uint32_t off)
{
    return 0x0800u + ((0x01A8u + off) >> 1);
}

static void shunt_ndb_resolve_offsets(void)
{
    const char *elf = shunt_fw_elf_path();
    if (!elf || !*elf) {
        SHUNT_ERR("NDB-OFFSETS : chemin de l'ELF firmware inconnu -> offsets #define "
                  "conserves (NON verifies contre le firmware)");
        return;
    }
    const char *tool = getenv("CALYPSO_NDB_TOOL");
    char toolbuf[1024];
    if (!tool || !*tool) {
        const char *tree = getenv("QEMU_TREE");
        snprintf(toolbuf, sizeof(toolbuf), "%s/tools/ndb-offsets.py",
                 (tree && *tree) ? tree : "/opt/GSM/qemu-src");
        tool = toolbuf;
    }
    const char *readelf = getenv("CALYPSO_READELF");
    if (!readelf || !*readelf) readelf = "arm-none-eabi-readelf";

    char cmd[2600];
    snprintf(cmd, sizeof(cmd), "python3 '%s' '%s' '%s' 2>/dev/null", tool, elf, readelf);
    FILE *p = popen(cmd, "r");
    if (!p) {
        SHUNT_ERR("NDB-OFFSETS : popen(%s) impossible -> offsets #define conserves", tool);
        return;
    }
    struct { const char *name; uint32_t *slot; uint32_t def; } map[] = {
        { "a_cd",   &g_ndb.a_cd,   NDB_A_CD   },
        { "a_fd",   &g_ndb.a_fd,   NDB_A_FD   },
        { "a_dd_0", &g_ndb.a_dd_0, NDB_A_DD_0 },
        { "a_cu",   &g_ndb.a_cu,   NDB_A_CU   },
        { "a_fu",   &g_ndb.a_fu,   NDB_A_FU   },
        { "a_du_1", &g_ndb.a_du_1, NDB_A_DU_1 },
    };
    int got = 0, diff = 0;
    char line[256];
    while (fgets(line, sizeof(line), p)) {
        char key[64]; unsigned val;
        if (sscanf(line, "%63[^=]=0x%x", key, &val) != 2)
            continue;
        for (unsigned i = 0; i < ARRAY_SIZE(map); i++) {
            if (strcmp(key, map[i].name))
                continue;
            *map[i].slot = val;
            got++;
            if (val != map[i].def) {
                diff++;
                SHUNT_ERR("NDB-OFFSETS : %s = 0x%03x dans le firmware, mais 0x%03x "
                          "en dur dans le code — le DWARF fait foi, #define PERIME",
                          map[i].name, val, map[i].def);
            }
        }
    }
    int rc = pclose(p);
    if (got != (int)ARRAY_SIZE(map)) {
        SHUNT_ERR("NDB-OFFSETS : %d/%zu champs resolus (rc=%d) -> offsets #define "
                  "conserves pour les manquants. Verifier %s et %s sur %s",
                  got, ARRAY_SIZE(map), rc, tool, readelf, elf);
        return;
    }
    g_ndb.resolved = true;
    /* La sonde s'annonce TOUJOURS, meme quand tout concorde : un silence ne doit
     * jamais pouvoir passer pour une verification reussie. */
    SHUNT_LOG("NDB-OFFSETS resolus du DWARF de %s : a_cd=0x%03x a_fd=0x%03x "
              "a_dd_0=0x%03x a_cu=0x%03x a_fu=0x%03x a_du_1=0x%03x (%s)\n",
              elf, g_ndb.a_cd, g_ndb.a_fd, g_ndb.a_dd_0, g_ndb.a_cu,
              g_ndb.a_fu, g_ndb.a_du_1,
              diff ? "DIVERGENCES ci-dessus" : "identiques aux #define");
    /* Le chemin a_cd du camp ecrit data[0x9D2] en CONSTANTE LITTERALE (hors de
     * ce resolveur). On ne le reecrit pas — il campe — mais on refuse qu'il
     * derive en silence. */
    if (ndb_w(g_ndb.a_cd) != 0x9D2u)
        SHUNT_ERR("NDB-OFFSETS : a_cd resolu tombe sur data[0x%03x], or le bloc SI du "
                  "camp ecrit data[0x9D2] EN DUR -> il ecrit desormais A COTE. "
                  "Corriger ces litteraux avant de se fier au camp.",
                  ndb_w(g_ndb.a_cd));
}

/* ---- TCH UL : trois sidebands, un par flux (2026-08-08) --------------------
 *
 * POURQUOI TROIS FICHIERS ET PAS UN SEUL AVEC UN CHAMP « TYPE ».
 * FACCH, SACCH et voix coexistent sur le meme canal a des cadences differentes
 * (voix 50/s, SACCH 1 bloc / 26 trames, FACCH sporadique). Dans un slot unique,
 * la voix ecraserait la FACCH avant que le consommateur (un pread par trame)
 * ne l'echantillonne — exactement la panne PUBLISH-NO-IDLE deja rencontree sur
 * le sideband SDCCH, ou l'idle effacait la SABM. Un fichier par flux supprime
 * la classe de panne au lieu de la filtrer.
 *
 * Layout 48 o des deux flux de signalisation : IDENTIQUE a calypso_sdcch_ul
 * (seq@0 l1s_fn@4 fn@8 task_u@12 l1s%51@14 l2[23]@16) -> le consommateur reutilise
 * le meme lecteur. Voix : 64 o, seq@0 l1s_fn@4 fn@8 fr[33]@16. */
#define TCH_UL_FACCH_PATH  "/dev/shm/calypso_tch_facch_ul"
#define TCH_UL_SACCH_PATH  "/dev/shm/calypso_tch_sacch_ul"
#define TCH_UL_SPEECH_PATH "/dev/shm/calypso_tch_ul"

static void tch_ul_publish_l2(const char *path, int *fdp, uint32_t *seq,
                              const uint8_t *l2,
                              uint16_t task_u, uint32_t fn, uint32_t l1s_fn)
{
    if (*fdp == -2) {
        *fdp = open(path, O_CREAT | O_RDWR, 0644);
        if (*fdp >= 0 && ftruncate(*fdp, 48) < 0) { /* best-effort */ }
    }
    if (*fdp < 0) return;
    (*seq)++;
    uint8_t buf[48] = {0};
    memcpy(buf + 4,  &l1s_fn, sizeof(l1s_fn));
    memcpy(buf + 8,  &fn,     sizeof(fn));
    memcpy(buf + 12, &task_u, sizeof(task_u));
    buf[14] = (uint8_t)(l1s_fn % 51);
    memcpy(buf + 16, l2, 23);
    memcpy(buf + 0,  seq, sizeof(*seq));   /* seq en dernier = publication atomique */
    if (pwrite(*fdp, buf, sizeof(buf), 0) < 0) { /* best-effort */ }
}

static void tch_ul_publish_speech(const uint8_t *fr, uint32_t fn, uint32_t l1s_fn)
{
    static int fd = -2;
    if (fd == -2) {
        fd = open(TCH_UL_SPEECH_PATH, O_CREAT | O_RDWR, 0644);
        if (fd >= 0 && ftruncate(fd, 64) < 0) { /* best-effort */ }
    }
    if (fd < 0) return;
    static uint32_t seq = 0; seq++;
    uint8_t buf[64] = {0};
    memcpy(buf + 4,  &l1s_fn, sizeof(l1s_fn));
    memcpy(buf + 8,  &fn,     sizeof(fn));
    memcpy(buf + 16, fr, 33);
    memcpy(buf + 0,  &seq, sizeof(seq));
    if (pwrite(fd, buf, sizeof(buf), 0) < 0) { /* best-effort */ }
}

/* Lit un bloc UL de 23 o de L2 a NDB+off (L2 a [3] = +6, la ou
 * dsp_memcpy_to_api l'a ecrit — pas de fenetre a scanner ici, contrairement a
 * a_cu sur SDCCH dont l'en-tete L1 SACCH decale la trame).
 *
 * CONSOMMATION. On efface B_BLUD apres lecture, comme le fait le DSP reel :
 * le firmware ARME le bit a chaque nouveau bloc mais ne l'efface JAMAIS
 * (prim_tch.c:443 et 495 posent (1<<B_BLUD), rien ne le retire). Sans effacement
 * cote shunt, « bloc present » resterait vrai a vie et on republierait la meme
 * trame a chaque trame TDMA : le premier ASSIGNMENT COMPLETE deviendrait un flux
 * continu et on ne saurait plus distinguer une retransmission T200 reelle d'un
 * echo. Effacer, c'est rendre la fraicheur DECIDABLE. */
static bool shunt_ndb_take_ul(uint32_t ndb_off, uint8_t *out, int n)
{
    uint16_t *d = (g_shunt.c54x && g_shunt.c54x->data) ? g_shunt.c54x->data : NULL;
    if (!d) return false;
    unsigned w = ndb_w(ndb_off);
    if (!(d[w] & (1u << B_BLUD)))
        return false;                       /* pas de bloc neuf */
    for (int i = 0; i < n; i += 2) {
        uint16_t v = d[w + 3 + i / 2];
        if (n == 33) {                      /* voix : BE=1 */
            out[i] = (uint8_t)(v >> 8);
            if (i + 1 < n) out[i + 1] = (uint8_t)(v & 0xff);
        } else {                            /* L2 : BE=0 */
            out[i] = (uint8_t)(v & 0xff);
            if (i + 1 < n) out[i + 1] = (uint8_t)(v >> 8);
        }
    }
    d[w] &= (uint16_t)~(1u << B_BLUD);      /* consomme */
    return true;
}

/* Capture UL du canal dedie TCH, routee par d_task_u.
 *   TCHT(13) -> a_fu   = FACCH montante  (ASSIGNMENT COMPLETE, puis L3 de l'appel)
 *            -> a_du_1 = voix montante   (PIEGE #1 : sub0 lit a_du_1, pas a_du_0)
 *   TCHA(14) -> a_cu   = SACCH montante  (rapports de mesure ; sans eux la BTS
 *                                         declare une defaillance de lien radio)
 *   TCHD(28) -> rien   (tache muette : RX-only, aucune donnee UL a relayer)
 * Rend true si la tache a ete traitee ici (le chemin SDCCH ne doit alors pas
 * tourner : il lirait a_cu avec la fenetre SDCCH et publierait sur le mauvais
 * sideband — c'est ce que faisait le code du 27/07, 4804 fois par run). */
static bool shunt_capture_tch_ul(uint16_t task_u)
{
    static int fd_facch = -2, fd_sacch = -2;
    static uint32_t seq_facch = 0, seq_sacch = 0;
    uint16_t t = task_u & 0x7FFF;
    uint32_t fn = calypso_trx_get_fn(), l1s = shunt_l1s_fn();

    if (t == TCHT_DSP_TASK) {
        uint8_t l2[23], fr[33];
        if (shunt_ndb_take_ul(g_ndb.a_fu, l2, 23)) {
            tch_ul_publish_l2(TCH_UL_FACCH_PATH, &fd_facch, &seq_facch, l2, task_u, fn, l1s);
            static unsigned n = 0;
            if (n++ < 40 || (n % 50) == 0)
                SHUNT_LOG("TCH-FACCH-UL #%u a_fu -> sideband : %02x %02x %02x %02x %02x %02x\n",
                          n, l2[0], l2[1], l2[2], l2[3], l2[4], l2[5]);
        }
        if (shunt_ndb_take_ul(g_ndb.a_du_1, fr, 33)) {
            tch_ul_publish_speech(fr, fn, l1s);
            static unsigned n = 0;
            if (n++ < 10 || (n % 500) == 0)
                SHUNT_LOG("TCH-SPEECH-UL #%u a_du_1 -> sideband (sig=0x%x)\n", n, fr[0] >> 4);
        }
        return true;
    }
    if (t == TCHA_DSP_TASK) {
        uint8_t l2[23];
        if (shunt_ndb_take_ul(g_ndb.a_cu, l2, 23)) {
            tch_ul_publish_l2(TCH_UL_SACCH_PATH, &fd_sacch, &seq_sacch, l2, task_u, fn, l1s);
            static unsigned n = 0;
            if (n++ < 40 || (n % 50) == 0)
                SHUNT_LOG("TCH-SACCH-UL #%u a_cu -> sideband : %02x %02x %02x %02x %02x %02x\n",
                          n, l2[0], l2[1], l2[2], l2[3], l2[4], l2[5]);
        }
        return true;
    }
    if (t == TCHD_DSP_TASK)
        return true;                        /* tache muette : rien a relayer */
    return false;                           /* pas du TCH -> chemin SDCCH */
}

static void shunt_poll_si_shm(void);                /* fwd : poll SI shm (gr-gsm→a_cd) */
static bool shunt_grgsm_off(void);                  /* fwd : CALYPSO_SHUNT_NO_GRGSM */
static void shunt_poll_tch_cfg(void);               /* fwd : /dev/shm/calypso_tch_cfg */

/* ---- LATCH : called on ARM write to NDB+0 (d_dsp_page) ---- */
/* [2026-07-30] ONE_PAGE — la page de lecture courante, et elle seule.
 *
 * Rend l'index de page que d_dsp_page (mot DSP 0x08D4, bit0) designe. Antiseche
 * osmocom-bb calypso/dsp.c:471 : `d_dsp_page = B_GSM_TASK | w_page`. Mesure du
 * 30/07 : le DSP suit ce bit0 a 99,6 % pour choisir sa page de sortie ; l'hote,
 * lui, ecrivait LES DEUX — ce qui annule le double buffer (l'ARM lit la meme chose
 * quelle que soit sa r_page, une page fraiche devient indiscernable d'une perimee).
 *
 * Gate CALYPSO_ONE_PAGE, defaut 0 : ce chemin porte le camp du profil shunt.
 */
static int shunt_one_page_on(void)
{
    static int c = -1;
    if (c < 0) {
        c = calypso_gate("CALYPSO_ONE_PAGE", 0);
        if (c)
            fprintf(stderr, "[shunt] ONE_PAGE=1 : l'hote n'ecrit plus que la page de "
                    "lecture designee par d_dsp_page bit0 (double buffer restaure)\n");
    }
    return c;
}

static uint8_t shunt_cur_rpage(void)
{
    if (!g_shunt.c54x || !g_shunt.c54x->data) {
        return 0;
    }
    return (uint8_t)(g_shunt.c54x->data[0x08D4] & 1u);
}

static void shunt_latch_task(uint16_t new_d_dsp_page)
{
    if (!(new_d_dsp_page & B_GSM_TASK)) {
        /* [2026-07-27] d_dsp_page=0 = l1s_reset_hw() (fermeture canal dedie SMS/LU
         * ou Ctrl-C mobile). Clear les latches IMM-ASS/SDCCH -> le gate SI se rouvre.
         * CHEMIN VIVANT (l'ancien hook arm2dsp/trx.c 0x01A8 etait mort : d_dsp_page
         * vit en API-RAM, pas en MMIO). Desactivable CALYPSO_L1_RESET_WIRE=0. */
        if (new_d_dsp_page == 0) {
            /* @BEQUILLE — L1_RESET_WIRE  (CALYPSO_L1_RESET_WIRE, ON-sauf-0, defaut ON)
             *   masque  : la remise a zero d'etat que le DSP reel subit sur l1s_reset_hw().
             *             Ici il n'y a pas d'etat DSP a reinitialiser, seulement des latches
             *             FABRIQUES par le shunt (IMM-ASSIGN, SDCCH) qui, non nettoyes,
             *             bloquent la reouverture du gate SI apres un Ctrl-C mobile.
             *   retirer : avec les latches eux-memes, c'est-a-dire avec les injections
             *             INJECT_AGCH / INJECT_SDCCH.
             */
            static int l1rst_on = -1;
            if (l1rst_on < 0) { const char *e = getenv("CALYPSO_L1_RESET_WIRE"); l1rst_on = (e && *e == '0') ? 0 : 1; }
            if (l1rst_on) {
                /* ═══════════════════════════════════════════════════════════
                 * [2026-08-04] COMPTEUR CUMULATIF a cote du plafond.
                 *
                 * POURQUOI. Cette sonde etait plafonnee a 30 lignes SANS
                 * compteur. Resultat : `grep -c L1-RESET` rendait 30 dans TOUS
                 * les runs, et ce 30 a ete cite plusieurs fois comme « 30
                 * reselections » — alors qu'il ne mesurait que le plafond. On ne
                 * pouvait donc pas savoir si les reselections augmentaient ou
                 * diminuaient, c'est-a-dire precisement le juge du mur restant.
                 *
                 * C'est le 4e artefact de ce type en une journee (RDMA_MASK,
                 * *FCCH*, contenu 0/296, celui-ci). REGLE : toute sonde
                 * plafonnee doit publier un TOTAL cumulatif a cote, sinon son
                 * plafond finit par etre lu comme une mesure.
                 * ═══════════════════════════════════════════════════════════ */
                static unsigned nrst = 0;
                nrst++;
                if (nrst <= 30)
                    SHUNT_LOG("L1-RESET #%u: d_dsp_page=0 -> clear latches (SI revient)\n", nrst);
                else if ((nrst % 50) == 0)
                    SHUNT_LOG("L1-RESET-TOTAL %u (plafond de lignes atteint a 30 ; "
                              "CE nombre est le vrai compte)\n", nrst);
                calypso_dsp_shunt_l1_reset();
            }
        }
        return; /* not a real task signal (might be d_dsp_page=0 reset) */
    }

    uint8_t  page_idx = (new_d_dsp_page & B_GSM_PAGE) ? 1 : 0;
    uint32_t wp       = wp_base(page_idx);

    g_shunt.page_idx  = page_idx;
    g_shunt.d_task_d  = shunt_read_w(wp + WP_D_TASK_D);
    /* [2026-07-22] PERCMD : ne PAS ecraser d_burst_d ici (horloge scenario =
     * aliasee, capture souvent cmd=0 debut de bloc) ; le mirror per-commande
     * (calypso_dsp_shunt_wp_burst_write) en est le seul maitre -> X suit la
     * vraie sequence 0,1,2,3. Gate off (=0) revient a l ancienne capture. */
    /* @BEQUILLE — SHUNT_BURST_PERCMD (capture latch)  (CALYPSO_SHUNT_BURST_PERCMD)
     *   masque  : la derivation de d_burst_d (0..3) depuis la fenetre TPU. Le shunt
     *             la synthetise (echo de la commande ARM ou l1s_fn) au lieu de la lire
     *             du materiel.
     *   retirer : quand la fenetre RX TPU/BDLENA cadence le burst-id cote DSP (RANK2).
     *   PIEGE   : idiome divergent entre les deux sites — ICI "(e && *e == 0) ? 0 : 1"
     *             (seule la chaine VIDE coupe) et dans wp_burst_write "*e == '0'"
     *             (la valeur "0" coupe). Poser =0 desarme le miroir ET laisse la
     *             capture desarmee : etat ni-l'un-ni-l'autre.
     */
    { static int pc = -1; if (pc < 0) { const char *e = getenv("CALYPSO_SHUNT_BURST_PERCMD");
                                        pc = (e && *e == 0) ? 0 : 1; }
      if (!pc) g_shunt.d_burst_d = shunt_read_w(wp + WP_D_BURST_D); }
    g_shunt.d_task_u  = shunt_read_w(wp + WP_D_TASK_U);
    g_shunt.d_task_md = shunt_read_w(wp + WP_D_TASK_MD);
    g_shunt.d_task_ra = shunt_read_w(wp + WP_D_TASK_RA);
    g_shunt.d_fn      = shunt_read_w(wp + WP_D_FN);
    /* RECODE FN (#4) : le firmware poste souvent d_fn=0 (FBSB = recherche, pas
     * de frame precise). On substitue la VRAIE FN TDMA pour le frame_nr aval
     * (DATA_IND / sync). */
    if (g_shunt.d_fn == 0)
        g_shunt.d_fn = (uint16_t)(calypso_trx_get_fn() & 0xFFFF);
    g_shunt.pending   = true;

    /* SDCCH/SACCH UL (#12 PIÈCE 1) : quand un NB UL est posté (d_task_u != 0,
     * DUL_DSP_TASK=12 en dédié), lire la L2 a_cu[3..] (23o @ NDB 0x264+6, octets
     * packés 2/mot) et la PUBLIER vers le sideband pour qemu_wrap (encode+module+
     * injecte). a_cu[0..2]=header. La L2 porte le SABM / SACCH meas / I-frames. */
    /* [2026-08-08] ROUTAGE PAR TACHE — le TCH d'abord.
     * Avant : ce bloc lisait a_cu (0x264, fenetre SDCCH) pour TOUT d_task_u non nul.
     * Sur canal dedie TCH, d_task_u vaut 13 (TCHT) ou 14 (TCHA) : la FACCH montante
     * vit dans a_fu (0x282), pas dans a_cu — on lisait donc a cote, et on publiait le
     * resultat sur le sideband SDCCH, qui l'injecte sur le slot SDCCH/4 SS0. Mesure du
     * run 08/08 : 4804 latch task_u=13 et 200 task_u=14, pour 6 SDCCH-UL publies (tous
     * task_u=12). L'ASSIGNMENT COMPLETE ne pouvait donc pas partir, et l'appel mourait
     * en ASSIGNMENT FAILURE (cause #1) six secondes plus tard, MO comme MT. */
    if (g_shunt.d_task_u != 0 && shunt_capture_tch_ul(g_shunt.d_task_u)) {
        /* traite par le chemin TCH ; ne pas retomber sur la fenetre SDCCH */
    } else if (g_shunt.d_task_u != 0) {
        uint8_t l2[23];
        /* a_cu UL : l'offset exact de la trame LAPDm varie (header L1 SACCH 2o /
         * type SABM-I-fill / packing) -> un offset fixe rate. On lit une FENETRE et
         * on SCANNE le debut de trame : 1er octet = addr SDCCH valide (EA=1, SAPI 0/3)
         * suivi d'un control non-fill. Base fenetre = gate CALYPSO_UL_ACU_OFS (def 6). */
        static int acu_ofs = -1;
        if (acu_ofs < 0) { const char *e = getenv("CALYPSO_UL_ACU_OFS"); acu_ofs = (e && *e) ? atoi(e) : 6; }
        uint8_t win[30];
        uint32_t wbase = BASE_API_NDB + 0x264u + (uint32_t)acu_ofs;
        for (int i = 0; i < 30; i += 2) {
            uint16_t w = shunt_read_w(wbase + i);
            win[i] = (uint8_t)(w & 0xff);
            if (i + 1 < 30) win[i + 1] = (uint8_t)((w >> 8) & 0xff);
        }
        int kk = -1;
        for (int j = 0; j <= 6; j++) {
            uint8_t a = win[j], c = win[j + 1];
            int sapi = (a >> 2) & 7;
            if ((a & 0x01) && (sapi == 0 || sapi == 3) &&
                c != 0x2b && c != 0x00 && c != 0xff) { kk = j; break; }
        }
        if (kk < 0) kk = 0;
        for (int i = 0; i < 23; i++) l2[i] = win[kk + i];
          /* [2026-08-09] CONSOMMATION UNIQUE DU MONTANT SDCCH.
           *
           * LE DEFAUT. Ce bloc publiait a CHAQUE passage, sans jamais consommer :
           * il balaie une fenetre de a_cu et ne teste pas B_BLUD, contrairement a
           * shunt_ndb_take_ul() qui efface le bit apres lecture. Tant que le
           * firmware laissait sa trame dans a_cu, on la rejouait a chaque trame.
           * Mesure du 09/08, la MEME SABM sur des trames consecutives :
           *   SDCCH-UL *NONIDLE* l1s%51=0  L2: 01 3f 49 05 08 70 00 f1
           *   SDCCH-UL *NONIDLE* l1s%51=1  L2: 01 3f 49 05 08 70 00 f1
           *   SDCCH-UL *NONIDLE* l1s%51=2  L2: 01 3f 49 05 08 70 00 f1
           * (0x3f = SABM, 05 08 = LOCATION UPDATING REQUEST).
           *
           * CE QUE CA CASSE. Le lien s etablit sur la premiere : le BSC passe bien
           * en ESTABLISHED. Les suivantes arrivent alors sur un lien deja etabli,
           * ou une SABM porteuse d information est ILLEGALE. La BTS repond
           * « ERROR INDICATION cause=SABM frame with information not allowed in
           * this state », quatre fois, puis libere : WAIT_BEFORE_RF_RELEASE ->
           * RR_REL_IND -> Location update failed. Le LU echouait APRES avoir
           * reussi. Meme defaut que le RACH rejoue, corrige la-bas par « 1 write
           * = 1 burst, pas de sticky », jamais applique au SDCCH montant.
           *
           * LE CHOIX. B_BLUD n est pas utilisable ici : ce chemin existe justement
           * parce que l offset de la trame varie et qu il faut balayer une fenetre.
           * On deduplique donc sur le CONTENU -- mais une retransmission T200 est
           * legitime et porte le meme contenu. On n etouffe que les repetitions
           * RAPPROCHEES : identiques ET moins de TTL ticks apres la precedente.
           * Defaut 60 ticks (~275 ms), sous le T200 du SDCCH.
           *
           * CALYPSO_SDCCH_UL_ONCE=0 retablit la republication systematique ;
           * CALYPSO_SDCCH_UL_TTL regle la fenetre. */
          {
              static int once = -1, ttl = 60;
              if (once < 0) {
                  /* [2026-08-10] DEFAUT ON — valide. Empeche la republication de
                   * la meme SABM a chaque trame, qui faisait protester le BSC
                   * (« SABM frame with information not allowed in this state »)
                   * sur un lien deja etabli. =0 pour republier comme avant. */
                  const char *e = getenv("CALYPSO_SDCCH_UL_ONCE");
                  once = (e && *e == '0') ? 0 : 1;
                  const char *t = getenv("CALYPSO_SDCCH_UL_TTL");
                  if (t && *t) ttl = atoi(t);
                  SHUNT_LOG("SDCCH-UL consommation unique : %s (fenetre %d ticks)\n",
                            once ? "ACTIVE" : "desactivee", ttl);
              }
              static uint8_t  dernier[23];
              static uint32_t dernier_tick;
              static int      a_deja_publie;
              static unsigned long long publiees, etouffees;
              int identique = a_deja_publie && !memcmp(dernier, l2, 23);
              int rapproche = (uint32_t)(g_shunt.tick_cnt - dernier_tick) < (uint32_t)ttl;
              if (once && identique && rapproche) {
                  etouffees++;
                  if (etouffees <= 3 || (etouffees % 500) == 0)
                      SHUNT_LOG("SDCCH-UL etouffee #%llu : repetition rapprochee "
                                "(ctrl=0x%02x, %u ticks) -- publiees=%llu\n",
                                etouffees, l2[1],
                                (unsigned)(g_shunt.tick_cnt - dernier_tick), publiees);
              } else {
                  memcpy(dernier, l2, 23);
                  dernier_tick  = g_shunt.tick_cnt;
                  a_deja_publie = 1;
                  publiees++;
                  calypso_sdcch_ul_publish(l2, g_shunt.d_task_u,
                                           calypso_trx_get_fn(), shunt_l1s_fn());
              }
          }
        /* Log : quelques idle pour sanity (capé), mais TOUJOURS les frames NON-IDLE
         * (ctrl=l2[1] != 0x03) -> capte le SABM (ctrl 0x3F) et les I-frames. */
        static int ul_log = 0;
        int non_idle = (l2[1] != 0x03);
        if (non_idle || ul_log < 6) {
            if (!non_idle) ul_log++;
            SHUNT_LOG("SDCCH-UL%s task_u=0x%04x l1s%%51=%u "
                    "L2: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                    non_idle ? " *NONIDLE*" : "", g_shunt.d_task_u,
                    (unsigned)(shunt_l1s_fn() % 51),
                    l2[0], l2[1], l2[2], l2[3], l2[4], l2[5], l2[6], l2[7]);
        }
    }

    /* PM : valeur statique, écrite IMMÉDIATEMENT (pas de service déféré au
     * prochain frame IRQ). Sinon le firmware lit a_pm AVANT le dispatch déféré
     * → 0 stale → rxlev=-110. On écrit a_pm sur la page lue tout de suite. */
    if (g_shunt.d_task_md == PM_DSP_TASK)
        shunt_dispatch_pm(page_idx);

    /* [2026-08-03] meme traitement que DISPATCH SB : on n'imprime que les
     * changements. 2 554 lignes sur 20 000 pour un contenu qui ne bouge pas. */
    {
        static uint32_t l_key = 0xFFFFFFFFu; static unsigned long long rep = 0;
        uint32_t key = ((uint32_t)page_idx << 24)
                     ^ ((uint32_t)g_shunt.d_task_md << 16)
                     ^ ((uint32_t)g_shunt.d_task_d  << 8)
                     ^ ((uint32_t)g_shunt.d_task_u)
                     ^ ((uint32_t)g_shunt.d_task_ra << 12);
        if (key != l_key) {
            if (rep) SHUNT_LOG("LATCH × %llu (identique, non repete)\n", rep);
            l_key = key; rep = 0;
            SHUNT_LOG("LATCH page=%u task_md=%u task_d=%u task_u=%u task_ra=%u fn=%u\n",
                page_idx, g_shunt.d_task_md, g_shunt.d_task_d, g_shunt.d_task_u,
                g_shunt.d_task_ra, g_shunt.d_fn);
        } else if (++rep % 2000 == 0) {
            SHUNT_LOG("LATCH × %llu (identique, fn=%u)\n", rep, g_shunt.d_fn);
        }
    }
}

/* ---- Canned tuning ----
 *
 * TOA target : prim_fbsb.c does `last_fb->toa -= 23` then derives ntdma/qbits.
 * Picking raw TOA=23 yields ntdma=0, qbits=0 → "perfectly on time", which
 * sidesteps the "DSP reports SB in bit that is N bits in the future" guard
 * and the `time_alignment` becomes 0 (clean baseline for synchronize_tdma).
 *
 * PM is shifted (>>3) by read_fb_result / read_sb_result. 0x7000 raw → 0xE00
 * after the shift, well above any AFC/threshold.
 *
 * SNR is read raw and compared against AFC_SNR_THRESHOLD. 0x7000 clears it
 * easily.
 *
 * ANGLE = 0 → ANGLE_TO_FREQ(0) = 0 → AFC correction null → the loop does
 * not re-iterate looking for AFC convergence (c-web's caution about
 * the AFC loop spinning if angle is non-zero but unchanged).
 *
 * BSIC = 63 (max, matches osmo-bsc.cfg default `base_station_id_code 63`).
 * t1=t2=t3=0 in encoded sb → l1s_decode_sb yields time->fn = 0 (seeds the
 * mobile's FN-counter at zero, which is FN-agnostic for canned dispatch).
 * Real FN coherence is a Phase 2 problem.
 */
/* ---- CALYPSO_CANNED : énumère EXPLICITEMENT chaque sortie DSP encore
 * FABRIQUÉE (canned) par le shunt, au lieu de la cacher derrière une valeur
 * « plausiblement juste ». CSV insensible casse ; "FULL"/"ALL" = tout canné,
 * "NONE"/"=" vide = rien. Var absente = DÉFAUT = CAN_DEFAULT (désormais RIEN
 * canné : toutes les sorties sont pilotées par le vrai décode gr-gsm). On
 * re-canne sélectivement avec CALYPSO_CANNED=<token>. BSIC/SI ne sont PAS ici :
 * déjà réels via gr-gsm / feed_si ; leur état est loggué au boot. */
/* TOUT DÉGATÉ (testés, camping tient), piloté par le vrai état de décode gr-gsm :
 *   FBDET = sb_valid (FB trouvé ssi SCH décodé)
 *   TOA   = timing SCH réel gr-gsm
 *   ANGLE = 0 (résidu réel post-correction freq)
 *   CRC   = sb_valid/si_valid (pass ssi vraiment décodé, sinon fail)
 *   PM    = 0 (a_serv_demod[PM] FB/SB ne pilote pas le rxlev — vient de dispatch_pm)
 *   SNR   = sb_valid ? bon : 0 (gr-gsm a décodé = preuve SNR suffisant ≥ seuil ;
 *           magnitude pas mesurée mais conditionnée au vrai décode, pas fabriquée
 *           inconditionnellement → SI ne casse pas).
 * DÉFAUT = RIEN canné. CALYPSO_CANNED=<token> re-canne sélectivement ; FULL=CAN_ALL. */

unsigned g_canned = CAN_DEFAULT;   /* résolu dans calypso_dsp_shunt_init */

static int can_tok_eq(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca -= 32;
        if (cb >= 'a' && cb <= 'z') cb -= 32;
        if (ca != cb) return 0;
        a++; b++;
    }
    return *a == 0 && *b == 0;
}

/* @BEQUILLE — CANNED  (CALYPSO_CANNED, LISTE ; vide/NONE = rien canne ;
 *              defaut effectif = 0 dans tous les profils livres)
 *   masque  : les sorties du correlateur/demodulateur DSP non produites —
 *             d_fb_det, TOA, PM, SNR, ANGLE, statut CRC — remplacees par des
 *             constantes plausibles (CAN_* dans calypso_dsp_internal.h).
 *   retirer : quand le correlateur natif produit ces six valeurs. La variable est
 *             deja a 0 partout : la retirer ne change que le vocabulaire.
 */
static unsigned shunt_parse_canned(void)
{
    const char *e = getenv("CALYPSO_CANNED");
    if (!e)                                          return CAN_DEFAULT;  /* var ABSENTE = défaut (= rien canné, tout réel) */
    if (!*e || can_tok_eq(e, "NONE"))                return 0;            /* "=" vide EXPLICITE = RIEN canné */
    if (can_tok_eq(e, "FULL") || can_tok_eq(e, "ALL")) return CAN_ALL;
    char buf[160];
    strncpy(buf, e, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    unsigned m = 0;
    for (char *t = strtok(buf, ", "); t; t = strtok(NULL, ", ")) {
        if      (can_tok_eq(t, "FBDET")) m |= CAN_FBDET;
        else if (can_tok_eq(t, "TOA"))   m |= CAN_TOA;
        else if (can_tok_eq(t, "PM"))    m |= CAN_PM;
        else if (can_tok_eq(t, "SNR"))   m |= CAN_SNR;
        else if (can_tok_eq(t, "ANGLE")) m |= CAN_ANGLE;
        else if (can_tok_eq(t, "CRC"))   m |= CAN_CRC;
        else if (can_tok_eq(t, "FULL") || can_tok_eq(t, "ALL")) m = CAN_ALL;
        else SHUNT_ERR("CALYPSO_CANNED: token inconnu '%s' ignore", t);
    }
    return m;
}

/* Canned SI3 bytes — 23 L2-frame bytes (RR PD + SI3 mt + payload).
 * Format conforme a osmocom-bb prim_rx_nb.c:154 :
 *   dsp_memcpy_from_api(rxnb.di->data, &dsp_api.ndb->a_cd[3], 23, 0);
 * Donc a_cd[0..2] = STATUS (CRC, biterr), a_cd[3..14] = 23B L2 frame.
 *
 * Layout L2+L3 RR SI3 :
 *   [0]=0x49 LI=18 EL=1   [1]=0x06 RR PD   [2]=0x1B SI3 mt
 *   [3..4]=Cell ID
 *   [5..7]=MCC/MNC encoded (0x00 0xF1 0x10 = MCC 001 MNC 01)
 *   [8..9]=LAC
 *   [10..11]=cell options + cell select
 *   [12..14]=RACH ctrl
 *   [15..22] = padding 0x2B */
/* SHUNT_CANNED_SI3_L2 RETIRÉ (no-hack 2026-06-03) : le SI vient
 * UNIQUEMENT du vrai décode grgsm (g_shunt.si_buf via feed_si). */

static void shunt_dispatch_nb(uint8_t page_idx, uint16_t task_d)
{
    /* TODO : NB DL = decoded BCCH/CCCH burst payload into NDB a_cd[].
     * NB UL = consume burst bits from DARAM for TX (forwarded to bridge). */
    SHUNT_LOG("DISPATCH NB page=%u task_d=%u (TODO)\n",
        page_idx, task_d);
}

/* [2026-08-09] GARDE SI PENDANT L OUVERTURE D UN CANAL DEDIE.
 *
 * DEFAUT CORRIGE : le bloc SI du camp s ecrit dans a_cd a CHAQUE tick, garde par
 * !sdcch_valid. Or sdcch_valid ne passe a vrai qu a la PREMIERE trame SDCCH
 * decodee par gr-gsm — environ 480 ms apres l ouverture du canal. Pendant cette
 * fenetre le mobile est DEJA en dedie : il lit les blocs SI, cadres BCCH, comme
 * des SACCH. gsm48_rr_rx_acch() (osmocom-bb) ne distingue B4 de Bter QUE PAR LA
 * LONGUEUR — 19 octets contre 21 — donc un bloc de 21 est pris pour un en-tete
 * court et son msg_type est lu sur des bits arbitraires. D ou le
 * « Short header message type 0x07 unsupported ».
 *
 * MESURE QUI L ETABLIT : 42 occurrences cote Calypso, 0 cote MS2 sur fake_trx
 * (qui ne passe pas par le shunt) ; et zero pendant un TCH etabli — 70 blocs
 * SACCH lus sans une erreur — parce que la, c est tch_cfg_valid qui garde.
 * Le motif « hors appel oui, pendant l appel non » designe exactement le SDCCH.
 *
 * DUREE : du DM_EST_REQ au DM_REL_REQ. Le TTL n est plus la duree nominale mais
 * un FILET DE SECURITE (~60 s) : si le DM_REL_REQ est manque, la garde se leve
 * seule plutot que d affamer le camp en SI pour toujours — le defaut
 * no-cell-info a deja ete paye une fois. CALYPSO_DCCH_SI_GUARD=0 la desactive
 * entierement, une autre valeur change le filet (en ticks de 4,6 ms). */
/* [2026-08-09] LES DEUX FRONTS DU CANAL DEDIE.
 * Appele depuis l1ctl_sock.c sur DM_EST_REQ (0x05) et DM_REL_REQ (0x12), dans le
 * sens mobile->firmware — le meme point qui remet deja le chiffrement a zero, donc
 * un chemin exerce a chaque appel.
 * POURQUOI DEUX FRONTS ET PAS UNE FENETRE. Premiere version : garde de 120 ticks
 * a l ouverture. Mesure : les « 0x07 » tombent a +3, +5, +7 ... +16 s apres
 * l entree en dedie, pas dans la premiere demi-seconde. Le camp ne clobbe pas
 * seulement au demarrage : il clobbe dans TOUS les trous entre deux presentations
 * SDCCH, parce que sdcch_valid retombe entre les blocs. Il faut donc tenir la
 * garde du debut a la fin du canal, pas la temporiser. */
void calypso_dsp_shunt_set_dcch_active(int on);
void calypso_dsp_shunt_set_dcch_active(int on)
{
    if (on) {
        bool was = g_shunt.dcch_guard_armed;
        g_shunt.dcch_guard_tick  = g_shunt.tick_cnt;   /* rafraichi a chaque bloc dedie */
        g_shunt.dcch_guard_armed = true;
        if (!was)                                      /* journaliser la TRANSITION seule :
                                                        * une ligne par bloc noierait le journal
                                                        * (121 paires mesurees en un run). */
            SHUNT_LOG("DCCH-GARDE : ARMEE -- SI du camp supprime dans a_cd\n");
    } else {
        if (g_shunt.dcch_guard_armed)
            SHUNT_LOG("DCCH-GARDE : levee -- SI du camp retabli\n");
        g_shunt.dcch_guard_armed = false;
    }
}

/* [2026-08-10] LA PRESENTATION a_cd N'A DE SENS QUE SUR SDCCH.
 * set_dcch() n'est appelee que pour SDCCH/4 et SDCCH/8 (kind >= 0) : sur un TCH
 * elle ne l'est PAS, et g_shunt.sdcch_ss gardait donc la fenetre du SDCCH
 * precedent. Comme la GARDE, elle, est bien rafraichie sur tout canal dedie
 * (TCH compris, c'est ce qui avait tue le 0x07 en signalisation), la
 * presentation continuait de tirer pendant tout l'appel voix sur une fenetre
 * fn%51 qui ne veut rien dire dans une multitrame de 26 -> le firmware relisait
 * du SI5/SI6 pose hors phase dans a_cd et rendait « Short header message type
 * 0x07 unsupported » EN RAFALE PENDANT LA COMMUNICATION (mesure du 10/08 :
 * 5 occurrences entre 09:12:05 et 09:12:29, sur un run ou le LU passait).
 * On coupe donc la presentation sur TCH : le SACCH de l'appel arrive par le
 * decodeur TCH (si_bridge, is_sacch = fn%26), pas par cette fenetre-ci. */
void calypso_dsp_shunt_set_dcch_tch(int on);   /* -Werror=missing-prototypes */
void calypso_dsp_shunt_set_dcch_tch(int on)
{
    bool v = (on != 0);
    if (g_shunt.dcch_is_tch == v) {
        return;                                 /* transition seule : pas de bruit */
    }
    g_shunt.dcch_is_tch = v;
    SHUNT_LOG("DCCH-TYPE : dedie = %s -- presentation SACCH a_cd %s\n",
              v ? "TCH" : "SDCCH", v ? "SUSPENDUE" : "retablie");
}

/* Declaration anticipee : la garde interroge la fraicheur du SACCH dedie,
 * defini plus bas. Idiome du fichier, il n y a pas d en-tete. */
static bool shunt_tch_fresh(bool valid, uint32_t tick);
static bool shunt_dcch_si_guard(void)
{
    static int ttl = -1;
    if (ttl < 0) { const char *e = getenv("CALYPSO_DCCH_SI_GUARD");
                   ttl = (e && *e) ? atoi(e) : 430; }     /* ~2 s sans bloc dedie = canal termine */
    if (ttl == 0 || !g_shunt.dcch_guard_armed) return false;

    /* [2026-08-09 REVISION] NE PAS SUPPRIMER SANS REMPLACANT.
     *
     * CE QUE LA PREMIERE VERSION FAISAIT DE FAUX. Elle coupait le SI du camp des
     * que la garde etait armee. Or a_cd n a que DEUX ecrivains, et le releve du
     * run le montre : « CAMP: a_cd<-SI » 1459 fois, « TCH-SACCH-DL » 46 fois, et
     * AUCUN ecrivain SDCCH (0 occurrence). Pendant un SDCCH dedie -- c est-a-dire
     * pendant tout l etablissement d appel -- la garde coupait donc le seul
     * contenu qui arrivait, et rien ne le remplacait : SACCH MUET a l instant
     * precis ou le mobile doit monter son LAPDm.
     *
     * L ARBITRAGE. Un SI du camp presente sur un canal dedie est un contenu FAUX :
     * la couche 3 le rejette avec un simple NOTICE (« Short header message type
     * 0x07 unsupported »), le canal survit. Un SACCH absent est une FAMINE : le
     * compteur de lien radio descend et l etablissement echoue. Entre les deux,
     * le bruit vaut mieux que le silence -- d ou les REJ puis DISC observes sur
     * a_fu, et l ASSIGNMENT COMPLETE qui ne trouvait aucun lien pour partir.
     *
     * On ne supprime donc que si un SACCH dedie alimente REELLEMENT a_cd. Sinon
     * on laisse passer le camp, on ravale les 0x07, et le canal vit. */
    /* [2026-08-09] LE REMPLACANT EXISTE MAINTENANT — cf. shunt_dcch_sacch_fill().
     * La revision precedente renoncait a supprimer quand aucun SACCH dedie
     * n alimentait a_cd (mesure : elle renoncait 3 fois sur 4, faute de source
     * SDCCH). On ne renonce plus : le site d appel ecrit une trame de bourrage
     * LAPDm valide a la place. La garde redevient donc stricte sur tout canal
     * dedie -- c est le sens de l original, sans la famine. */
    return true;
    if ((uint32_t)(g_shunt.tick_cnt - g_shunt.dcch_guard_tick) > (uint32_t)ttl) {
        g_shunt.dcch_guard_armed = false;      /* plus de bloc dedie depuis ttl : canal fini */
        SHUNT_LOG("DCCH-GARDE : levee (peremption) -- SI du camp retabli\n");
        return false;
    }
    return true;
}

/* ---- TCH/F DL (JALON 1) : sideband /dev/shm/calypso_tch_dl -> a_dd_0 ----
 * Producteur = qemu_wrap/gr-gsm (decode 8 bursts -> gsm0503_tch_fr_decode -> 33o FR) ou
 * l'injecteur de test (tone 600). Layout 48o : seq@0(u32 LE) fn@4(u32 LE) fr[33]@8.
 * Consume-once par seq (modele calypso_rach_read). */
/* Sideband voix DL — ANNEAU de 16 trames (2026-08-08).
 *
 * POURQUOI UN ANNEAU. C'etait un SLOT UNIQUE : si_bridge y ecrit a 50 trames/s
 * (le debit plein d'un TCH/F) et QEMU le relit au rythme de son tick. Toute
 * irregularite du tick ecrase une trame avant lecture, DEFINITIVEMENT et SANS
 * TRACE. Mesure du run 21:34 : 659 trames decodees par gr-gsm, 500 presentees
 * dans a_dd_0, 491 TRAFFIC_IND -> ~25 % perdues dans le passage, soit ~37/s au
 * lieu de 50/s. C'est exactement ce que l'oreille entend comme un son
 * intermittent : la chaine est bonne, elle fuit au transfert.
 *
 * Layout : entete 8 o [w_seq(u32) | n_slots(u32)] puis 16 x 48 o, slot k =
 * ((seq-1) % 16). Le producteur ecrit le slot PUIS publie w_seq (ordre
 * important : un lecteur ne doit jamais voir un seq annonce dont le slot n'est
 * pas encore ecrit). Compat : si le fichier fait 48 o (ancien format), on
 * retombe sur le slot unique.
 *
 * PERTE COMPTEE, PAS DEDUITE : si le producteur a pris plus de 16 trames
 * d'avance, on saute et on l'annonce avec un TOTAL CUMULATIF. Une perte
 * silencieuse redeviendrait indiscernable d'un decodage incomplet en amont. */
#define TCH_DL_RING_N 16
#define TCH_DL_SLOT   48

#define TCH_DL_Q_N 8
/* [2026-08-09] MARGE ENTRE LE POLL ET LE DISPATCH.
 * Le poll tourne a chaque tick de trame (~217/s), le dispatch ne consomme que
 * sur les ticks TCHT (~50/s). Avec un emplacement unique, la moindre gigue
 * faisait tomber le dispatch sur du vide : 20 ms de voix perdus, gapk prive de
 * bloc, sa sortie ALSA en famine. C'est ce qui rendait le patch de
 * dimensionnement gapk necessaire — il compensait EN AVAL un manque de marge
 * cree ICI. Quelques trames d'avance suffisent a l'absorber.
 * On ne re-presente JAMAIS une trame consommee (cf. la consommation unique du
 * 08/08) : on en garde d'avance, ce qui est different d'un doublon.
 * CALYPSO_TCH_DL_PREFETCH=1 retablit le comportement d'un seul emplacement. */
static int shunt_tch_dl_prefetch(void)
{
    static int v = -1;
    if (v < 0) { const char *e = getenv("CALYPSO_TCH_DL_PREFETCH");
                 v = (e && *e) ? atoi(e) : 4;
                 if (v < 1) v = 1;
                 if (v > TCH_DL_Q_N) v = TCH_DL_Q_N; }
    return v;
}
static unsigned shunt_tch_dl_qdepth(void)
{ return g_shunt.tch_dl_q_tail - g_shunt.tch_dl_q_head; }
static void shunt_tch_dl_qpush(const uint8_t *fr, uint32_t seq)
{
    unsigned i = g_shunt.tch_dl_q_tail % TCH_DL_Q_N;
    memcpy(g_shunt.tch_dl_q[i], fr, 33);
    g_shunt.tch_dl_q_seq[i] = seq;
    g_shunt.tch_dl_q_tail++;
}
static bool shunt_tch_dl_qpop(uint8_t *fr, uint32_t *seq, int peek)
{
    if (g_shunt.tch_dl_q_head == g_shunt.tch_dl_q_tail) return false;
    unsigned i = g_shunt.tch_dl_q_head % TCH_DL_Q_N;
    memcpy(fr, g_shunt.tch_dl_q[i], 33);
    if (seq) *seq = g_shunt.tch_dl_q_seq[i];
    if (!peek) g_shunt.tch_dl_q_head++;
    return true;
}

static void calypso_tch_dl_poll(void)
{
    static int fd = -2;
    if (fd == -2)
        fd = open("/dev/shm/calypso_tch_dl", O_CREAT | O_RDWR, 0644);
    if (fd < 0)
        return;

    uint8_t hdr[8];
    if (pread(fd, hdr, sizeof(hdr), 0) != (ssize_t)sizeof(hdr))
        return;
    uint32_t w_seq, n_slots;
    memcpy(&w_seq, hdr, 4);
    memcpy(&n_slots, hdr + 4, 4);

    if (n_slots == 0 || n_slots > 4096) {       /* ancien format 48 o : slot unique */
        uint8_t buf[TCH_DL_SLOT];
        if (pread(fd, buf, sizeof(buf), 0) != (ssize_t)sizeof(buf))
            return;
        uint32_t seq; memcpy(&seq, buf, 4);
        if (seq == 0 || seq == g_shunt.tch_dl_seq)
            return;
        g_shunt.tch_dl_seq = seq;
        memcpy(g_shunt.tch_dl_fr, buf + 8, 33);
        g_shunt.tch_dl_valid = true;
        shunt_tch_dl_qpush(buf + 8, seq);
        return;
    }

    if (w_seq == 0 || w_seq == g_shunt.tch_dl_seq)
        return;                                  /* rien de neuf */

    /* [2026-08-08] LE PRODUCTEUR REPART A 1 A CHAQUE APPEL. si_bridge recree son
     * tailer par session TCH, avec un `seq` local remis a zero, alors qu'ici
     * tch_dl_seq garde la valeur de l'appel precedent. Au nouvel appel on avait
     * donc w_seq(1) < tch_dl_seq(655), et la soustraction NON SIGNEE ci-dessous
     * debordait : le journal a affiche « 4294966626 trames sautees » (= 2^32-670),
     * un compteur de perte absurde qui aurait fait chercher une fuite inexistante.
     * Un recul du seq ne peut signifier qu'une chose : nouveau producteur. */
    if (w_seq < g_shunt.tch_dl_seq) {
        SHUNT_LOG("TCH-DL : le producteur a redemarre (w_seq=%u < %u) -> "
                  "resynchronisation\n", w_seq, g_shunt.tch_dl_seq);
        g_shunt.tch_dl_seq = 0;
        g_shunt.tch_dl_valid = false;
        g_shunt.tch_dl_q_head = g_shunt.tch_dl_q_tail = 0;   /* file videe avec le producteur */
    }
    if (shunt_tch_dl_qdepth() >= (unsigned)shunt_tch_dl_prefetch())
        return;                                  /* on a deja assez d'avance */

    uint32_t next = g_shunt.tch_dl_seq + 1;
    if (g_shunt.tch_dl_seq == 0 || (w_seq - next) >= n_slots) {
        /* Le producteur a debordé l'anneau : on repart sur la plus ancienne
         * trame encore intacte, et on COMPTE ce qu'on saute. */
        /* Borne explicite : ce compteur a deja affiche 2^32-670 par debordement.
         * Un compteur de perte faux est pire que pas de compteur. */
        uint32_t behind = (w_seq > next) ? (w_seq - next) : 0;
        uint32_t skipped = (g_shunt.tch_dl_seq == 0 || behind < n_slots)
                         ? 0 : behind - (n_slots - 1);
        /* [2026-08-08] ON PREND LA PLUS RECENTE, PAS LA PLUS ANCIENNE.
         * Prendre w_seq-(N-1) visait a « ne rien perdre », mais c'est le slot que
         * le producteur va ECRASER EN PREMIER : a 50 trames/s le controle
         * `sseq != next` echouait presque toujours, on repartait sans avancer, et
         * comme tch_dl_seq ne bougeait pas on retombait sur le meme slot au tour
         * suivant. INTERBLOCAGE : mesure du 22:10, le sideband avancait a 50/s
         * (w_seq 1429->1579) pendant que TRAFFIC_IND restait a 0/s.
         * La plus recente vient d'etre publiee : elle est stable ~20 ms, et pour
         * de la voix c'est de toute facon la bonne a garder. */
        next = w_seq;
        if (skipped) {
            static unsigned long long lost_total = 0; static unsigned nlog = 0;
            lost_total += skipped;
            if (nlog++ < 20 || (nlog % 50) == 0)
                SHUNT_LOG("TCH-DL DEBORDEMENT : %u trames sautees (TOTAL CUMULE %llu) "
                          "-- l'anneau de %u ne suit pas ; c'est un trou AUDIBLE, "
                          "pas un defaut de decodage\n",
                          skipped, lost_total, n_slots);
        }
    }

    uint8_t buf[TCH_DL_SLOT];
    off_t off = 8 + (off_t)((next - 1) % n_slots) * TCH_DL_SLOT;
    if (pread(fd, buf, sizeof(buf), off) != (ssize_t)sizeof(buf))
        return;
    uint32_t sseq; memcpy(&sseq, buf, 4);
    if (sseq != next)
        return;                                  /* slot pas encore ecrit : on repassera */
    g_shunt.tch_dl_seq = next;
    memcpy(g_shunt.tch_dl_fr, buf + 8, 33);
    g_shunt.tch_dl_valid = true;
    shunt_tch_dl_qpush(buf + 8, next);
}


/* ---- TCH : primitives d'ecriture NDB (2026-08-08) --------------------------
 *
 * POURQUOI ECRIRE data[] DIRECTEMENT ET PAS shunt_write_w().
 * shunt_write_w passe par dma_memory_write -> calypso_dsp_write, qui prend
 * calypso_pcb_daram_lock. Ce verrou n'est PAS recursif, et ces dispatch sont
 * appeles depuis on_frame_tick — le meme contexte qui a impose shunt_c54x_api_rd()
 * en lecture directe (cf le commentaire de cette fonction). Le chemin a_cd du camp,
 * lui, ecrit deja data[0x9D2] en direct et c'est CE chemin qui campe. On aligne
 * donc les buffers TCH sur le chemin prouve, pas sur le chemin theorique.
 *
 * PACKING. Le firmware relit avec dsp_memcpy_from_api(dst, src, n, BE) :
 *   BE=0 (a_cd / a_fd, 23 o de L2)   -> mot = lo | (hi << 8)
 *   BE=1 (a_dd_0, 33 o de FR)        -> mot = (hi << 8) | lo   [l'INVERSE]
 * Les deux conventions coexistent dans la MEME struct ; les melanger donne un
 * bloc qui passe le CRC cote shunt et sort en charabia cote gapk. */
static inline uint16_t *shunt_ndb_data(void)
{
    return (g_shunt.c54x && g_shunt.c54x->data) ? g_shunt.c54x->data : NULL;
}

/* En-tete d'un bloc NDB : [0] = statut, [1] libre, [2] = num_biterr.
 * blud=true -> B_BLUD arme + bits FIRE a 0 = « bloc present, CRC bon ». */
static void shunt_ndb_hdr(uint16_t *d, unsigned w, bool blud)
{
    d[w + 0] = blud ? (uint16_t)(1u << B_BLUD) : 0x0000;
    d[w + 1] = 0x0000;
    d[w + 2] = 0x0000;                  /* num_biterr = 0 */
}

/* 23 o de L2 -> [3..14], packing BE=0 (comme a_cd). Bourrage LAPDm 0x2B. */
static void shunt_ndb_put_l2(uint16_t *d, unsigned w, const uint8_t *l2)
{
    for (int i = 0; i < 23; i += 2) {
        uint8_t lo = l2[i], hi = (i + 1 < 23) ? l2[i + 1] : 0x2B;
        d[w + 3 + i / 2] = (uint16_t)(lo | (hi << 8));
    }
}

/* 33 o de FR -> [3..], packing BE=1 (a_dd_0/a_du_*). Mot impair = fr[32] en
 * poids FORT (le firmware ne lit que l'octet haut du dernier mot). */
static void shunt_ndb_put_fr(uint16_t *d, unsigned w, const uint8_t *fr)
{
    for (int i = 0; i < 32; i += 2)
        d[w + 3 + i / 2] = (uint16_t)(((uint16_t)fr[i] << 8) | fr[i + 1]);
    d[w + 3 + 16] = (uint16_t)((uint16_t)fr[32] << 8);
}

/* a_serv_demod des READ PAGES (mots 8..11 : page0 data[0x830..0x833], page1
 * data[0x844..0x847]).
 *
 * INDISPENSABLE SUR TCH, et c'est mesurable : l1s_tch_resp (prim_tch.c:214-231)
 * lit ces 4 mots A CHAQUE burst pour alimenter afc_input(), toa_input() et
 * rffe_compute_gain(). Sans eux le mobile a publie, run du 08/08, des
 * « MEAS REP: meas-invalid=1 rxlev-full=-110 » une fois par seconde pendant
 * toute la fenetre TCH — c'est-a-dire un canal juge muet, plus une AFC nourrie
 * au hasard. Meme raisonnement, memes valeurs et memes gates DECAN que le bloc
 * a_cd du camp : on ne fabrique une constante que la ou le modele ne fournit
 * rien encore, et le gate le dit. */
static void shunt_tch_serv_demod(uint16_t *d)
{
    static int dc_toa = -1, dc_snr, dc_ang;
    if (dc_toa < 0) {
        const char *M = getenv("CALYPSO_DECAN");
        int m = (M && M[0] == '1');
        const char *t = getenv("CALYPSO_DECAN_TOA");
        const char *s = getenv("CALYPSO_DECAN_SNR");
        const char *a = getenv("CALYPSO_DECAN_ANGLE");
        dc_toa = m || (t && t[0] == '1');
        dc_snr = m || (s && s[0] == '1');
        dc_ang = m || (a && a[0] == '1');
    }
    /* @BEQUILLE — a_serv_demod du TCH  (gates CALYPSO_DECAN*, OFF par defaut)
     *   masque  : l'absence de mesure per-burst sur le canal dedie. gr-gsm decode
     *             le TCH mais ne publie pas (encore) TOA/PM/ANGLE/SNR par burst,
     *             donc OFF on repose les memes constantes que le camp.
     *   retirer : quand le pont TCH publiera ses mesures par burst, comme le SCH
     *             publie deja BSIC/FN/TOA sur 4731.
     */
    uint16_t toa_v = (dc_toa && g_shunt.sb_valid) ? (uint16_t)g_shunt.rx_toa : 23;
    uint16_t pm_v  = shunt_pm_decan_apm(-60);
    uint16_t ang_v = dc_ang ? (uint16_t)g_shunt.rx_afc : 0;
    uint16_t snr_v = dc_snr ? g_shunt.rx_snr : 0x7000;
    /* page 0 */ d[0x830] = toa_v; d[0x831] = pm_v; d[0x832] = ang_v; d[0x833] = snr_v;
    /* page 1 */ d[0x844] = toa_v; d[0x845] = pm_v; d[0x846] = ang_v; d[0x847] = snr_v;
}

/* Age maximum (en ticks) d'une trame de signalisation DL avant d'arreter de la
 * re-presenter. Meme garde-fou que SHUNT_SB_MAX_AGE : une trame tenue sans borne
 * finit par etre rejouee en silence, et le journal ment (on croit voir un flux,
 * on voit un echo). 0 = pas de peremption. */
/* Gate A/B commun aux trois flux DL du TCH (voix, FACCH, SACCH) :
 * CALYPSO_TCH_DL_REPEAT=1 retablit la re-presentation d'avant le 08/08. */
static bool shunt_tch_dl_repeat(void)
{
    static int v = -1;
    if (v < 0) { const char *e = getenv("CALYPSO_TCH_DL_REPEAT"); v = (e && *e == '1') ? 1 : 0; }
    return v != 0;
}

static uint32_t shunt_tch_dl_ttl(void)
{
    static int v = -1;
    if (v < 0) { const char *e = getenv("CALYPSO_TCH_DL_TTL"); v = (e && *e) ? atoi(e) : 26; }
    return (uint32_t)v;
}

static bool shunt_tch_fresh(bool valid, uint32_t tick)
{
    uint32_t ttl = shunt_tch_dl_ttl();
    if (!valid) return false;
    return (ttl == 0) || ((uint32_t)(g_shunt.tick_cnt - tick) <= ttl);
}

/* ---- SONDE CALYPSO_TCH_DL_PROBE : ce qu'on ECRIT vs ce qui PART ------------
 *
 * [2026-08-09] Constat qui l'impose : chaine descendante mesuree saine jusqu'au
 * sideband (50,0 trames/s, 250/250 distinctes, 0 % de silence), la chaine de
 * lecture GAPK du mobile s'execute bien ~50 fois/s (prouve : gapk_io_dequeue_ul
 * n'a qu'UN appelant, juste apres gapk_io_enqueue_dl -> les 48,7 TRAFFIC_REQ/s
 * mesures ne peuvent venir que d'autant de TRAFFIC_IND enfiles), et pourtant
 * l'ecouteur du MS est du ZERO NUMERIQUE EXACT (crete 0 sur 48000 echantillons).
 * Il ne reste qu'une hypothese testable : les octets FR qui arrivent au mobile
 * ne sont pas ceux qu'on a ecrits dans a_dd_0.
 *
 * On memorise donc les N dernieres trames REELLEMENT ecrites, et l1ctl_sock
 * confronte a cet anneau les 33 octets qui partent vraiment. L'anneau (et non
 * la seule derniere trame) parce que le firmware relit a_dd_0 avec un retard de
 * quelques ticks : comparer a la derniere ecrite fabriquerait des ecarts qui
 * n'existent pas.
 *
 * Sonde de comparaison, pas de taux : elle rend des compteurs CUMULATIFS. */
#define TCH_DL_PROBE_N 8
static struct { uint8_t fr[33]; uint32_t seq; } g_tch_dl_probe[TCH_DL_PROBE_N];
static unsigned g_tch_dl_probe_w;

static void shunt_tch_dl_probe_note(const uint8_t *fr, uint32_t seq)
{
    unsigned i = g_tch_dl_probe_w++ % TCH_DL_PROBE_N;
    memcpy(g_tch_dl_probe[i].fr, fr, 33);
    g_tch_dl_probe[i].seq = seq;
}

/* Rend le seq de la trame ecrite qui correspond, ou -1. Appele par l1ctl_sock. */
int calypso_dsp_shunt_tch_dl_written(const uint8_t *fr33);
int calypso_dsp_shunt_tch_dl_written(const uint8_t *fr33)
{
    for (unsigned i = 0; i < TCH_DL_PROBE_N; i++)
        if (g_tch_dl_probe[i].seq && !memcmp(g_tch_dl_probe[i].fr, fr33, 33))
            return (int)g_tch_dl_probe[i].seq;
    return -1;
}

/* TCHT (13) DL = voix (a_dd_0) + FACCH (a_fd) + mesures. Le firmware relit
 * a_dd_0 en fin de bloc (fn%13%4==3) ssi B_BLUD, et a_fd au meme instant.
 * On presente les deux : la FACCH porte le CONNECT/ALERTING, la voix le son. */
/* PRESENTATION DU SI6 SUR LA SACCH DU CANAL DEDIE — le site qui manquait.
 *
 * LE TROU. sacch_buf contient un SI6 valide au format B4 : soit le SI5/SI6 REEL
 * decode par gr-gsm (calypso_dsp_shunt_feed_sacch, l.2672), soit, tant qu aucun
 * reel n est arrive, un SI6 derive du SI3 (l.3826) avec l identite de cellule et
 * le LAI VERITABLES de la cellule. sacch_have marque sa disponibilite. Un
 * commentaire annonce meme « presente fn%51 {42-45} »... mais AUCUN code ne le
 * presentait. Le tampon etait rempli et jamais lu.
 *
 * CONSEQUENCE MESUREE. Sur SDCCH -- donc pendant TOUT l etablissement d appel --
 * a_cd n avait que deux ecrivains : le SI du camp (contenu FAUX sur un canal
 * dedie, rejete en « Short header message type 0x07 ») et TCH-SACCH-DL, qui ne
 * tire que sur TCH. Le mobile n avait donc jamais de SACCH valide pendant qu il
 * devait monter son LAPDm : cote a_fu on observait RR, puis REJ, puis DISC, et
 * cote BSC « Timeout (rll_ready=no) » -- 41 echecs d assignation en une journee.
 *
 * POURQUOI CE N EST PAS UNE BEQUILLE. On ne fabrique aucune information : le SI6
 * porte l identite de cellule et le LAI lus dans le SI3 de cette meme cellule,
 * et il cede la place au SI5/SI6 reel des que gr-gsm en decode un (sacch_real).
 * C est la presentation qui manquait, pas la donnee.
 *
 * NOTE SUR UNE FAUSSE PISTE. Une premiere version presentait ici une trame de
 * bourrage LAPDm vide (L=0, remplissage 0x2B), en croyant reproduire ce qu emet
 * une BTS sans rien a dire. Le mobile l a REFUSEE : gsm48_rr.c:6259 traite toute
 * trame B4 de longueur N201_B4 comme un SYSTEM INFORMATION et commute sur le
 * premier octet L3 -- il lisait donc le bourrage, d ou « ACCH message type 0x2b
 * unknown » puis « MDL-Error (cause 11) ». Sur SACCH dediee, osmocom-bb n accepte
 * QUE SI5 / SI5bis / SI5ter / SI6. Il n existe pas de SACCH vide.
 *
 * CALYPSO_DCCH_SACCH_FILL=0 desactive cette presentation (retour a l etat
 * anterieur : SI du camp et 0x07). */
static void shunt_dcch_sacch_present(uint16_t *d)
{
    static int on = -1;
    if (on < 0) {
        /* [2026-08-10] DEFAUT OFF POUR LE BISSECT. Ce correctif supprime les
         * « Short header 0x07 » et fait decoder un jeu complet de SI5, mais un
         * binaire le contenant a casse le LOCATION UPDATING ACCEPT. On l eteint
         * par defaut le temps d isoler le coupable. =1 pour l allumer. */
        /* [2026-08-10] DEFAUT ON — valide. Presente le SI5/SI6 reel sur la SACCH
         * du canal dedie, UNIQUEMENT dans la fenetre GSM 05.02 du sous-canal.
         * Supprime les « Short header 0x07 » sans casser le LOCATION UPDATING
         * ACCEPT, ce que la version « partout sauf le canal principal » faisait.
         * =0 pour revenir au SI du camp. */
        const char *e = getenv("CALYPSO_DCCH_SACCH_FILL");
        on = (e && *e == '0') ? 0 : 1;
        SHUNT_LOG("DCCH-SACCH : presentation du SI6 sur canal dedie %s\n",
                  on ? "ACTIVE" : "desactivee");
    }
    if (!on) return;
    /* [2026-08-10] cf. calypso_dsp_shunt_set_dcch_tch : sur TCH la fenetre
     * fn%51 heritee du SDCCH est hors phase -> 0x07 en rafale. */
    if (g_shunt.dcch_is_tch) return;

    /* [2026-08-09] NE PAS ECRASER LE CANAL PRINCIPAL.
     * La premiere version presentait a CHAQUE tick ou la garde est armee, sans
     * aucune condition de position de trame : ~22000 ecritures dans a_cd contre 62
     * presentations de l anneau SDCCH, deux ordres de grandeur. Or a_cd porte AUSSI
     * le canal principal du SDCCH, donc le UA repondant au SABM -- on l ecrasait
     * avant que le firmware l ait lu.
     * Symptome exact : le mobile appliquait bien la SACCH (« DL SACCH indicates ta
     * / tx_power », jeu complet de SI5) mais ne recevait JAMAIS le UA ; il
     * retransmettait sa SABM sur T200, le BSC -- deja passe en ESTABLISHED --
     * repondait « SABM frame with information not allowed in this state » puis
     * liberait le canal.
     * Deux garde-fous :
     *   1. ne pas ecrire sur un bloc non lu (B_BLUD encore arme) -- meme principe
     *      que la garde de a_dd_0, qui a fait tomber forcages de 13949 a 0 ;
     *   2. ne pas empieter sur la fenetre du canal principal, fn%51 dans
     *      [base, base+3] ; la SACCH a ses propres blocs ailleurs.
     * Mesure apres correctif : 5001 ecritures evitees, et les ERROR INDICATION
     * cessent. */
    /* [2026-08-10] FENETRE SACCH POSITIVE — corrige la version precedente.
     *
     * CE QUI ETAIT FAUX. La condition disait « presenter partout SAUF dans la
     * fenetre du canal principal [base, base+3] ». Or a_cd n est pas libre le reste
     * du temps : sur un SDCCH/4 la SACCH a ses PROPRES blocs, et le firmware lit
     * a_cd a des instants differents selon qu il attend du canal principal ou de la
     * SACCH. En ecrivant a tout autre moment, on lui livrait du SI5 quand il
     * attendait autre chose. Mesure du 10/08 : CALYPSO_DCCH_SACCH_FILL=1 cassait le
     * LOCATION UPDATING ACCEPT, meme avec la garde anti-ecrasement active.
     *
     * LA BONNE CONDITION (GSM 05.02, SDCCH/4 + SACCH sur TS0 combine, cycle 102) :
     *   SS0 -> fn%102 42..45     SS1 -> 46..49
     *   SS2 -> fn%102 93..96     SS3 -> 97..100
     * On ne presente QUE la. Le canal principal garde ses blocs, et le UA passe.
     *
     * Par-dessus, on conserve la garde anti-ecrasement : meme dans la bonne
     * fenetre, on n ecrit pas sur un bloc que le firmware n a pas encore lu.
     * CALYPSO_DCCH_SACCH_GUARD=0 retire cette garde seule. */
    {
        static int garde = -1;
        if (garde < 0) {
            const char *g = getenv("CALYPSO_DCCH_SACCH_GUARD");
            garde = (g && *g == '0') ? 0 : 1;
            SHUNT_LOG("DCCH-SACCH : garde du canal principal %s\n",
                      garde ? "ACTIVE" : "desactivee");
        }
        static const uint8_t sacch_base[4] = { 42, 46, 93, 97 };
        unsigned ss_idx;
        switch (g_shunt.sdcch_ss) {          /* base fn%51 -> index de sous-canal */
        case 26: ss_idx = 1; break;
        case 32: ss_idx = 2; break;
        case 36: ss_idx = 3; break;
        default: ss_idx = 0; break;          /* 22 = SS0 */
        }
        uint32_t f102 = (uint32_t)calypso_trx_get_fn() % 102u;
        uint32_t b    = sacch_base[ss_idx];
        if (f102 < b || f102 > b + 3u) {
            static unsigned long long hors;
            if (++hors % 20000 == 1)
                SHUNT_LOG("DCCH-SACCH : fn%%102=%u hors fenetre SACCH [%u-%u] du SS%u "
                          "(#%llu) -- on ne presente pas\n",
                          f102, b, b + 3u, ss_idx, hors);
            return;
        }
        if (garde && (d[ndb_w(g_ndb.a_cd)] & (1u << B_BLUD))) {
            static unsigned long long non_lu;
            if (++non_lu % 500 == 1)
                SHUNT_LOG("DCCH-SACCH : bloc non lu (#%llu) -- on se tait\n", non_lu);
            return;
        }
    }

    if (!g_shunt.sacch_have) {
        /* Rien de valide a presenter : on ne met PAS de bourrage (le mobile le
         * refuserait), on laisse le camp reprendre la main -- ses 0x07 sont un
         * NOTICE, alors qu une trame invalide leve une MDL-Error. */
        static unsigned long long muet;
        if (++muet % 5000 == 1)
            SHUNT_LOG("DCCH-SACCH : aucun SI6 disponible (#%llu) -- le camp reprend\n", muet);
        return;
    }

    shunt_ndb_hdr(d, ndb_w(g_ndb.a_cd), true);
    shunt_ndb_put_l2(d, ndb_w(g_ndb.a_cd), g_shunt.sacch_buf);

    static unsigned long long n;
    if (++n <= 3 || (n % 2000) == 0)
        /* [2026-08-09] On imprime les 8 premiers octets, pas seulement mt.
         * Le mobile discrimine le format SACCH par la seule LONGUEUR :
         *   l3len == 21 (N201_Bter_SACCH) -> en-tete court -> il lit le 1er octet
         *                                     comme un type de message (d ou les
         *                                     « SYSTEM INFORMATION 10 » parasites)
         *   l3len == 19 (N201_B4)         -> SI5 / SI6, ce qu on veut
         * Mesure : 30 SI10 pour 3 SI5 -> nos trames arrivent a 21, deux octets de
         * trop. Voir les octets permet de dire OU sont ces deux octets : en-tete
         * L1 SACCH (2 o) mal place, ou en-tete LAPDm absent la ou on le croit.
         * Attendu pour du B4 : [0..1] en-tete L1, [2] addr (0x03), [3] ctrl (0x03),
         * puis le L3 qui commence par 0x06 (PD RR) et 0x1d/0x1e (SI5/SI6). */
        SHUNT_LOG("DCCH-SACCH #%llu : a_cd <- %02x %02x %02x %02x %02x %02x %02x %02x"
                  " (mt@6=0x%02x, %s)\n",
                  n, g_shunt.sacch_buf[0], g_shunt.sacch_buf[1],
                  g_shunt.sacch_buf[2], g_shunt.sacch_buf[3],
                  g_shunt.sacch_buf[4], g_shunt.sacch_buf[5],
                  g_shunt.sacch_buf[6], g_shunt.sacch_buf[7],
                  g_shunt.sacch_buf[6],
                  g_shunt.sacch_real ? "REEL gr-gsm" : "derive du SI3");
}

static void shunt_dispatch_tch_dl(uint8_t page_idx)
{
    (void)page_idx;                     /* a_dd_0/a_fd non pages (T_NDB partage) */
    uint16_t *d = shunt_ndb_data();
    if (!d)
        return;

    shunt_tch_serv_demod(d);            /* toujours : les mesures valent pour tout burst */

    /* ---- voix : CONSOMMATION UNIQUE (2026-08-08, correctif) ----------------
     *
     * Avant, ce bloc reecrivait a_dd_0 a CHAQUE dispatch TCHT tant que
     * tch_dl_valid, or ce drapeau ne redescendait jamais. Une trame decodee
     * etait donc re-presentee jusqu'a l'arrivee de la suivante, avec B_BLUD
     * arme : le firmware la comptait comme un bloc NEUF a chaque fois. Mesure
     * du run 17:33 : 5000 presentations pour 371 trames REELLEMENT decodees par
     * gr-gsm, soit ~3 TRAFFIC_IND sur 4 qui etaient des doublons. Le mobile
     * recevait un flux d'apparence continue, en fait bourre de repetitions —
     * exactement le « replay infini » deja rencontre sur sb_valid, et corrige
     * la-bas par SHUNT_SB_MAX_AGE.
     *
     * On consomme donc la trame : presentee une fois, puis plus de B_BLUD tant
     * que gr-gsm n'en fournit pas une nouvelle. Les trous deviennent VISIBLES —
     * et c'est voulu : gapk a un ECU (« ecu/fr » dans sa chaine) dont le role
     * est precisement de masquer les blocs manquants. Mieux vaut un trou traite
     * par l'etage prevu pour ca qu'un doublon presente comme une trame fraiche.
     * CALYPSO_TCH_DL_REPEAT=1 retablit l'ancien comportement pour comparer. */
    static int repeat = -1;
    if (repeat < 0) { const char *e = getenv("CALYPSO_TCH_DL_REPEAT");
                      repeat = (e && *e == '1') ? 1 : 0; }
    uint8_t _fr[33]; uint32_t _sq = 0;
    /* NE PAS ECRASER UNE TRAME NON LUE -- MAIS BORNER L ATTENTE.
     *
     * LE DEFAUT. Le lecteur ne prend a_dd_0 qu en fin de bloc (fn%13%4==3), soit
     * une fois pour quatre bursts, alors que le dispatch TCHT tourne a ~200/s.
     * Quand deux trames arrivent entre deux lectures, la seconde ecrase la
     * premiere avant qu elle ait ete vue. Mesure sur appel etabli : 13000 trames
     * servies contre 11000 TRAFFIC_IND -- 2000 perdues, soit 15 %, un ecart huit
     * fois superieur au pas des compteurs (250), donc pas de la quantification.
     * La sonde disait « identiques a a_dd_0 : 11000, differentes : 0 » : aucune
     * trame abimee, donc ecrasees et non corrompues. Une sur sept qui saute, ca
     * s entend comme un son hache.
     *
     * POURQUOI BORNER. Premiere version : attendre INDEFINIMENT que B_BLUD
     * retombe. Mesure : 40000 retenues et une file collee a 8 sur 8, sa capacite.
     * Une fois pleine, le collecteur cesse de prendre les trames de l anneau : on
     * ajoute 160 ms de retard et on perd a l entree ce qu on a cesse de perdre a
     * la sortie. J avais deplace le probleme, pas supprime.
     *
     * LE COMPROMIS. On attend au plus HOLD dispatches, puis on presente quand
     * meme : perdre UNE trame vaut mieux que bloquer la file entiere. Le defaut 3
     * vient du rapport de cadences mesure (4 dispatches par trame de parole).
     *
     * CALYPSO_TCH_DL_HOLD=0 revient a l ecrasement systematique (comportement
     * d avant, pour l A/B) ; une valeur elevee revient a la version qui saturait.
     * Compteurs CUMULATIFS et sonde qui s annonce : un forcage frequent voudrait
     * dire que le lecteur decroche, et ca doit se voir. */
    static int hold = -1;
    if (hold < 0) {
        const char *e = getenv("CALYPSO_TCH_DL_HOLD");
        /* [2026-08-09] DEFAUT 5, MESURE — surtout pas 3.
         * Le firmware ne lit a_dd_0 qu en fin de bloc, (fn%13)%4==3, soit une
         * fenetre toutes les 4,330 dispatches TCHT (216,5 dispatches/s mesures
         * pour 50,0 trames/s). Avec hold=3 la garde expirait DONC TOUJOURS avant
         * la fenetre suivante, et le forcage ecrasait une trame que le lecteur
         * allait prendre : 13949 forcages, 14,8 %, son hache. A 5 la fenetre
         * couvre la periode entiere -- mesure sur appel etabli : forcages = 0.
         * Regle generale : un seuil d attente se compare a la PERIODE DU
         * CONSOMMATEUR, mesuree. Sous cette periode, le garde-fou devient un
         * destructeur periodique. */
        hold = (e && *e) ? atoi(e) : 5;
        if (hold < 0) hold = 0;
        SHUNT_LOG("TCH-DL garde anti-ecrasement : attente max %d dispatches%s\n",
                  hold, hold ? "" : " -- DESACTIVEE (ecrasement systematique)");
    }
    static unsigned attente = 0;
    static unsigned long long retenues = 0, forcages = 0;
    bool lecteur_pret = true;
    if (hold > 0 && (d[ndb_w(g_ndb.a_dd_0)] & (uint16_t)(1u << B_BLUD))) {
        if (attente < (unsigned)hold) {
            attente++; retenues++;
            if (retenues <= 3 || (retenues % 2000) == 0)
                SHUNT_LOG("TCH-DL retenue #%llu : precedente pas lue, on garde "
                          "(profondeur=%u, forcages=%llu)\n",
                          retenues, shunt_tch_dl_qdepth(), forcages);
            lecteur_pret = false;
        } else {
            forcages++;
            if (forcages <= 3 || (forcages % 2000) == 0)
                SHUNT_LOG("TCH-DL forcage #%llu : lecteur en retard apres %d "
                          "attentes, presentation forcee (profondeur=%u)\n",
                          forcages, hold, shunt_tch_dl_qdepth());
        }
    }
    if (lecteur_pret) attente = 0;
    if (lecteur_pret && shunt_tch_dl_qpop(_fr, &_sq, repeat)) {
        shunt_ndb_hdr(d, ndb_w(g_ndb.a_dd_0), true);
        shunt_ndb_put_fr(d, ndb_w(g_ndb.a_dd_0), _fr);
        shunt_tch_dl_probe_note(_fr, _sq);
        g_shunt.tch_dl_valid = (shunt_tch_dl_qdepth() > 0);
        static unsigned n = 0, dup = 0;
        if (repeat) dup++;
        if (n++ < 20 || (n % 250) == 0)
            SHUNT_LOG("TCH-DL #%u a_dd_0 <- FR seq=%u sig=0x%x profondeur=%u%s\n",
                      n, _sq, _fr[0] >> 4, shunt_tch_dl_qdepth(),
                      repeat ? " (REPEAT : doublons possibles)" : "");
    }
    if (shunt_tch_fresh(g_shunt.facch_dl_valid, g_shunt.facch_dl_tick)) {
        shunt_ndb_hdr(d, ndb_w(g_ndb.a_fd), true);
        shunt_ndb_put_l2(d, ndb_w(g_ndb.a_fd), g_shunt.facch_dl);
        /* [2026-08-08] CONSOMMATION UNIQUE, comme la voix.
         * Ce bloc reecrivait a_fd a CHAQUE dispatch TCHT tant que le TTL tenait
         * (26 ticks), en rearmant B_BLUD a chaque fois : le firmware remontait
         * donc la MEME trame LAPDm des dizaines de fois. Mesure du run 21:25 :
         * 12 « MDL-Error (cause 6) » en UNE seconde, cause 6 =
         * MDL_CAUSE_UNSOL_SPRV_RESP (lapd_core.h:38) = trame de supervision NON
         * SOLLICITEE — c'est-a-dire la meme supervision relivree en boucle.
         * Le firmware efface lui-meme l'en-tete apres lecture (prim_tch.c:293,
         * a_fd[0] = 1<<B_FIRE1), donc ne plus reecrire ne perd rien : le bloc
         * reste disponible jusqu'a ce qu'il le consomme. */
        if (!shunt_tch_dl_repeat())
            g_shunt.facch_dl_valid = false;
        static unsigned n = 0;
        if (n++ < 40 || (n % 100) == 0)
            SHUNT_LOG("TCH-FACCH-DL #%u a_fd <- %02x %02x %02x %02x (age=%u ticks)\n",
                      n, g_shunt.facch_dl[0], g_shunt.facch_dl[1],
                      g_shunt.facch_dl[2], g_shunt.facch_dl[3],
                      g_shunt.tick_cnt - g_shunt.facch_dl_tick);
    }
}

/* TCHA (14) DL = SACCH du canal dedie -> a_cd (l1s_tch_a_resp, prim_tch.c:667).
 *
 * PIEGE : ce chemin partage a_cd avec le BCCH du camp, MAIS PAS LA CONVENTION
 * D'EN-TETE. l1s_nb_resp (BCCH) ne teste que les bits FIRE et le camp ecrit donc
 * a_cd[0]=0x0000 ; l1s_tch_a_resp teste `a_cd[0] & (1<<B_BLUD)` et ce meme 0x0000
 * le ferait sauter le bloc en silence. Ici : B_BLUD arme. */
static void shunt_dispatch_tch_sacch(uint8_t page_idx)
{
    (void)page_idx;
    uint16_t *d = shunt_ndb_data();
    if (!d)
        return;

    shunt_tch_serv_demod(d);

    if (!shunt_tch_fresh(g_shunt.tsacch_dl_valid, g_shunt.tsacch_dl_tick))
        return;
    shunt_ndb_hdr(d, ndb_w(g_ndb.a_cd), true);          /* B_BLUD : cf piege ci-dessus */
    shunt_ndb_put_l2(d, ndb_w(g_ndb.a_cd), g_shunt.tsacch_dl);
    /* Consommation unique, meme raison que la FACCH ci-dessus : l1s_tch_a_resp
     * efface a_cd[0] apres lecture (prim_tch.c:700), donc re-presenter ne fait
     * que dupliquer des blocs de signalisation deja livres. */
    if (!shunt_tch_dl_repeat())
        g_shunt.tsacch_dl_valid = false;
    static unsigned n = 0;
    if (n++ < 40 || (n % 50) == 0)
        SHUNT_LOG("TCH-SACCH-DL #%u a_cd <- %02x %02x %02x %02x %02x %02x\n",
                  n, g_shunt.tsacch_dl[0], g_shunt.tsacch_dl[1], g_shunt.tsacch_dl[2],
                  g_shunt.tsacch_dl[3], g_shunt.tsacch_dl[4], g_shunt.tsacch_dl[5]);
}

/* CALYPSO_DSP=c54x : pilote le VRAI DSP depuis le frame tick du shunt.
 * Les ordres (d_task_md/d_task_d/d_task_u/d_task_ra) sont DEJA dans l'api_ram
 * partagee (c54x->api_ram == dsp_ram cote ARM), donc pas de recopie du
 * descripteur ici. On (a) DMA la write-page API -> DARAM 0x0586 (le trx skippe
 * cette DMA quand le shunt est actif, on la refait nous-memes), (b) recharge le
 * dernier burst I/Q dans bsp_buf, (c) leve INT3 (FRAME) + wake, (d) execute le
 * budget c54x_run.
 *
 * FIX (verif report) : la write-page MCU->DSP est a BASE_API_W_PAGE_0/1
 * (0xFFD00000 / 0xFFD00028), PAS a BASE_API_NDB (0xFFD001A8). On reutilise le
 * helper wp_base() existant. Replique la DMA de trx calypso_dsp_done(@711) :
 * data[0x0584]=page, data[0x0585]=fn, data[0x0586+i]=wp[i] (i<20), et le mirror
 * api_ram[0x08E2 - C54X_API_BASE]=page (d_dsp_page cote DSP, lu par le firmware). */
/* Lecture DIRECTE de l'espace data[] du c54x pour une adresse API ARM,
 * SANS round-trip MMIO calypso_dsp_read (qui prend calypso_pcb_daram_lock,
 * mutex non-recursif -> re-lock/abort quand on est deja dans le contexte
 * frame-tick). Meme mapping que calypso_dsp_read : ARM off O -> data[O/2+0x800]. */
static inline uint16_t shunt_c54x_api_rd(C54xState *dsp, uint32_t arm_addr)
{
    return dsp->data[((arm_addr - 0xFFD00000UL) >> 1) + 0x0800];
}

/* [2026-07-24] CADENCE SPLIT : shunt_route_to_c54x() etait appelee UNIQUEMENT
 * quand g_shunt.pending (= l'ARM vient de poster un NOUVEAU d_dsp_page), soit
 * ~1 fois toutes les ~12 trames reelles en pratique (mesure runtime : 708 insn
 * DSP/tour mais ~57ms reels/tour -- le go-live retry loop du DSP n'a donc
 * qu'une fraction de son temps reel pour attraper la fenetre de survie de
 * data[0x0810]/d_ctrl_system, contrairement au natif ou c54x_run tourne a
 * chaque trame SANS gate sur un dispatch ARM frais). Le natif ne connait pas
 * ce probleme : calypso_tdma_tick() appelle c54x_run() inconditionnellement
 * (gate uniquement sur running/idle/shunt_active), jamais sur "tache neuve".
 *
 * Fix : separer le header (partie A, a besoin de page_idx/d_fn FRAIS, donc
 * reste gate sur pending -- cf le fix staleness du meme jour juste au-dessus)
 * du wake+run (partie B, ne depend pas d'un dispatch frais : rejoue le
 * dernier IQ connu, tire l'IT frame, tourne le budget DSP -- exactement ce
 * que native fait a chaque trame). Partie B devient appelable a CHAQUE trame,
 * pending ou pas, pour retrouver une cadence proche du natif (~217Hz au lieu
 * de ~17Hz). */
static void shunt_route_to_c54x_header(uint8_t page_idx)
{
    C54xState *dsp = g_shunt.c54x;
    if (!dsp)
        return;
    fprintf(stderr, "[c54x-route] enter page=%u dsp=%p\n", (unsigned)page_idx, (void*)dsp);

    /* (a) API write-page -> DARAM 0x0586 (replique de la DMA trx gatee a :711).
     * wp_base(page_idx) = adresse MMIO absolue de la write-page (== dsp_ram).
     * Le mot d_dsp_page (NDB+0 = 0xFFD001A8) est lu live (= s->dsp_ram[0x01A8/2]
     * cote trx) pour data[0x0584] et le mirror 0x08E2. */
    {
        uint32_t wbase    = wp_base(page_idx);
        fprintf(stderr, "[c54x-route] a1 wbase=0x%08x\n", wbase);
        /* [2026-07-24] FIX RACE : l'ancien lisait dsp->data[NDB_D_DSP_PAGE]=data[0x08E2]
         * ICI-MEME (mid-frame), une cellule que data[0x08E2] toggle entre ce
         * moment et le FRAME-IT qui suit dans CETTE MEME invocation (confirme
         * runtime : a2 voit 186/186 fois 0x0000, FRAME-IT# voit 559/559 fois
         * 0x0003, sur la MEME cellule -- pas un bug d'adresse, un bug de
         * timing/staleness). page_idx (parametre) est DEJA la valeur fraiche,
         * capturee au moment de l'ecriture ARM (cf ligne ~135 :
         * page_idx = (new_d_dsp_page & B_GSM_PAGE) ? 1 : 0). On reconstruit
         * dsp_page depuis cette valeur connue-fraiche au lieu de relire une
         * cellule sujette a race, evitant de repropager du perime dans
         * data[0x0584]/api_ram[0x08E2] ci-dessous. */
        uint16_t dsp_page = B_GSM_TASK | page_idx;
        /* [2026-07-29] La « race » decrite juste au-dessus n'en etait pas une :
         * a2 affichait data[0x08E2] (jamais ecrit) et le FRAME-IT affichait
         * api_ram[0x08E2] (ecrit par le miroir ci-dessous) — deux tableaux
         * distincts, deux valeurs, aucune course. Et 0x08E2 n'etait de toute
         * facon pas la bonne cellule : d_dsp_page = 0x08D4 (cf calypso_fbsb.h).
         * On affiche desormais la cellule que la ROM lit reellement, dans le
         * tableau qu'elle lit reellement (api_ram). */
        fprintf(stderr, "[c54x-route] a2 dsp_page=0x%04x (from page_idx=%u, "
                "api_ram[0x%04x]=0x%04x) api_ram=%p\n",
                dsp_page, (unsigned)page_idx, NDB_D_DSP_PAGE,
                dsp->api_ram ? dsp->api_ram[NDB_D_DSP_PAGE - C54X_API_BASE]
                             : dsp->data[NDB_D_DSP_PAGE],
                (void*)dsp->api_ram);
        dsp->data[0x0584] = dsp_page;
        dsp->data[0x0585] = (uint16_t)(g_shunt.d_fn & 0xFFFF);
        fprintf(stderr, "[c54x-route] a3 data-hdr-ok\n");
        for (int i = 0; i < 20; i++)
            dsp->data[0x0586 + i] = shunt_c54x_api_rd(dsp, wbase + (uint32_t)i * 2);
        fprintf(stderr, "[c54x-route] a4 wp-copy-ok\n");
        /* [2026-07-29] Miroir d_dsp_page cote DSP. Ecrivait 0x08E2 = d_dsp_state :
         * la page ecrasait le C_DSP_IDLE3 pose par l'ARM (dsp.c:215), et la ROM,
         * qui lit 0x08D4 (0xa51c/0xc8ea), ne voyait rien. Corrige a la source —
         * ce qui remplit la condition de retrait de la bequille FIX_DPAGE_OFF
         * (calypso_c54x.c), retiree du meme coup. */
        if (dsp->api_ram)
            dsp->api_ram[NDB_D_DSP_PAGE - C54X_API_BASE] = dsp_page;
    }
    fprintf(stderr, "[c54x-route] a-daram-ok\n");
}

static void shunt_route_to_c54x_run(void)
{
    C54xState *dsp = g_shunt.c54x;
    if (!dsp)
        return;

    /* (b) rejoue le dernier burst I/Q (cs16 entrelace I,Q) dans bsp_buf. */
    if (g_shunt.last_iq_valid && g_shunt.last_iq_n > 0)
        c54x_bsp_load(dsp, (const uint16_t *)g_shunt.last_iq, g_shunt.last_iq_n);

    fprintf(stderr, "[c54x-route] b-bsp-load-ok n=%d\n", g_shunt.last_iq_n);
    /* (c) INT3 FRAME + wake : reveille le DSP s'il etait idle/halt. */
    g_c54x_int3_src = 3;
    /* [2026-07-22] FRAME_IT_NATIVE : tick propre — livre le scheduler frame
     * (vec28/bit12) DIRECTEMENT au frame-tick, pas via le remap 19/3. Le
     * c54x_irq_level_check le prend quand INTM=0 (prise naturelle, pas de force).
     * = le vrai primitif HW frame-sync. Sinon (legacy) : vec19/bit3 (+remap VEC28). */
    {
        /* @BEQUILLE — FRAME_IT_NATIVE  (CALYPSO_FRAME_IT_NATIVE, EXISTS ; :=1 en
         *              native / native_helped / wire)
         *   masque  : l'absence de cablage frame-TPU -> vecteur DSP. On appelle
         *             directement c54x_interrupt_ex(dsp,28,12) au frame-tick au lieu de
         *             19/3 (le stub vec19 est un RETE).
         *   retirer : quand le TPU delivre l'IT frame sur le bon vecteur tout seul
         *             (idem VEC28_REMAP cote calypso_c54x.c).
         */
        static int fin = -1;
        if (fin < 0) fin = calypso_gate("CALYPSO_FRAME_IT_NATIVE", 0);
        if (fin)
            c54x_interrupt_ex(dsp, 28, 12);   /* scheduler frame IT, tick propre */
        else
            c54x_interrupt_ex(dsp, C54X_INT_FRAME_VEC, C54X_INT_FRAME_BIT);
        /* [2026-07-23] TINT MASTER CLOCK sync frame : fire TINT au MEME tick TDMA
         * (pas per-2000-insn). Handler 0x72d3 = driver slots op.
         * [2026-08-03] CAL000 §5.1 : TINT = bit3/vec19, pas bit4/vec20 (= RINT/SPI
         * receive). Bascule sous le sas CALYPSO_IT_TABLE_DOC, cf. calypso_c54x.c. */
        {
            /* @BEQUILLE — TINT0_MASTER (fire au frame-tick)  (CALYPSO_TINT0_MASTER, EXISTS,
             *              defaut OFF hors profil WIRE)
             *   masque  : la configuration/demarrage du TIMER0 par le ROM. Le firmware arrete
             *             le timer (TSS=1) dans une init non-tournee ; on fabrique TINT
             *             a la cadence trame.
             *   retirer : quand la sequence d'init TIMER0 du ROM s'execute (TCR programme).
             */
            static int _t0m = -1;
            if (_t0m < 0) _t0m = calypso_gate("CALYPSO_TINT0_MASTER", 0);
            if (_t0m) {
                static int _doc = -1;
                if (_doc < 0) _doc = calypso_gate("CALYPSO_IT_TABLE_DOC", 0);
                if (_doc)   /* §5.1 : TINT = IMR bit 3 / vec 19 */
                    c54x_interrupt_ex(dsp, C54X_IT_TINT_VEC, C54X_IT_TINT_BIT);
                else        /* legacy SPRU131 : en fait RINT / SPI receive */
                    c54x_interrupt_ex(dsp, C54X_IT_SPI_RX_VEC, C54X_IT_SPI_RX_BIT);
            }
        }
    }
    c54x_wake(dsp);
    /* revive: c54x_run loop gate = (running && !idle). c54x_wake ne clear que
     * idle ; en mode route_c54x le chemin trx qui posait running=true est gate
     * off -> forcer running ici sinon la boucle c54x_run est sautee (0 insn). */
    dsp->running = true;

    fprintf(stderr, "[c54x-route] c-wake-ok running=%d idle=%d\n", dsp->running, dsp->idle);
    /* (d) execute le budget (1 trame nominale ~256000 insns ; ajustable env). */
    {
        static int budget = -1;
        if (budget < 0) {
            const char *b = getenv("CALYPSO_DSP_BUDGET");
            budget = (b && *b) ? atoi(b) : 256000;
            if (budget <= 0) budget = 256000;
        }
        fprintf(stderr, "[c54x-route] d-pre-c54x_run budget=%d\n", budget);
        c54x_run(dsp, budget);
        fprintf(stderr, "[c54x-route] d-c54x_run-RETURNED\n");
    }
}

/* ---- Service hook : called from calypso_trx frame_irq tick ---- */
/* [AFC loop-close v2] derniere mesure de frequence BRUTE du FCCH (Hz), pour
 * recalculer rx_afc = brut - freq_deja_compensee_par_DAC a CHAQUE tick (et pas
 * seulement quand feed_iq/det re-fire, ce qui cesse une fois le DAC enroule). */
static double g_rx_raw_hz = 0.0;
static int    g_rx_raw_valid = 0;

/* [DECAN PM 2026-07-26] rxlev a_pm depuis la VRAIE magnitude MAV (last_pm) au lieu
 * de la cible -60 figee : rf_dbm = 20*log10(last_pm/MAV_REF)+RF_REF, puis apm_for_rf
 * (modele trf6151). Ancrage MAV_REF->RF_REF (def 20929->-60 = niveau courant) faute
 * de reference hardware ; la valeur SUIT desormais le signal reel (fading/niveau).
 * Gate CALYPSO_DECAN_PM (ou master CALYPSO_DECAN). OFF -> apm_for_rf(fallback). */
/* @BEQUILLE — DECAN_PM (+ _MAV_REF / _RF_REF)  (CALYPSO_DECAN_PM ou master
 *              CALYPSO_DECAN, EQ1 ; OFF = bequille de stabilisation)
 *   masque  : la mesure de puissance du DSP. ON : rf_dbm derive de la magnitude
 *             MAV via un ancrage arbitraire (MAV_REF -> RF_REF). OFF : a_pm fige a
 *             apm_for_rf(-60), une constante decretee.
 *   retirer : quand a_pm est produit par le DSP depuis l'I/Q (les deux branches
 *             disparaissent alors, ON comme OFF).
 */
static uint16_t shunt_pm_decan_apm(int fallback_target)
{
    static int dc = -1;
    static double mav_ref = 20929.0, rf_ref = -60.0;
    if (dc < 0) {
        /* PM sous le master DECAN (LU accept valide avec PM decan en shunt_legit
         * 2026-07-26) : plancher -75 + seuil last_pm>1000 gardent le camp. */
        const char *M = getenv("CALYPSO_DECAN");
        const char *p = getenv("CALYPSO_DECAN_PM");
        dc = ((M && M[0] == '1') || (p && p[0] == '1')) ? 1 : 0;
        const char *mr = getenv("CALYPSO_DECAN_PM_MAV_REF"); if (mr && *mr) mav_ref = atof(mr);
        const char *rr = getenv("CALYPSO_DECAN_PM_RF_REF");  if (rr && *rr) rf_ref = atof(rr);
        if (mav_ref < 1.0) mav_ref = 1.0;
    }
    /* [fix bootstrap 2026-07-26] seuil + plancher : a l'acquisition last_pm peut
     * etre transitoirement infime -> rf -> -inf -> apm=0 -> rxlev=-138 -> cell
     * rejetee -> NO_CELL_FOUND -> JAMAIS de camp (chicken-egg : il faut camper
     * pour avoir un bon last_pm). En dessous du seuil -> fallback -60 (campable) ;
     * au-dessus -> de-can borne a [-85,-40] dBm (suit le MAV sans jamais rejeter). */
    if (dc && g_shunt.last_pm > 1000) {
        double rf = 20.0 * log10((double)g_shunt.last_pm / mav_ref) + rf_ref;
        if (rf < -75.0) rf = -75.0;
        if (rf > -40.0) rf = -40.0;
        int rfi = (int)(rf >= 0 ? rf + 0.5 : rf - 0.5);
        return calypso_trf6151_apm_for_rf(rfi);
    }
    return calypso_trf6151_apm_for_rf(fallback_target);
}

void calypso_dsp_shunt_on_frame_tick(void)
{
    if (!g_shunt.active)
        return;
    shunt_poll_si_shm();   /* gr-gsm a-t-il ecrit un nouveau SI dans le shm ? */
    calypso_tch_dl_poll(); /* nouvelle trame FR DL dans le sideband ? (toujours, hors gate pending) */
    shunt_poll_tch_cfg();  /* canal dedie TCH arme/libere par si_bridge (ASSIGNMENT COMMAND) */

    /* [2026-07-24] CADENCE FIX : le run C54x (wake+IT-frame+c54x_run) ne
     * depend d'AUCUNE donnee fraiche de dispatch (page_idx/d_fn) -- seul le
     * header ci-dessous en a besoin. On le sort du gate pending pour tourner
     * a CHAQUE trame reelle, comme le fait calypso_tdma_tick() en natif (qui
     * appelle c54x_run() inconditionnellement, jamais sur "tache neuve").
     * Avant ce fix : c54x_run() ne tournait qu'aux ~1/12 trames ou l'ARM
     * venait de poster un d_dsp_page frais (mesure : 708 insn DSP/tour mais
     * ~57ms reels/tour) -- le DSP n'avait donc qu'une fraction de son temps
     * reel pour attraper la fenetre de survie de data[0x0810]/d_ctrl_system. */
    if (g_shunt.c54x && shunt_route_c54x()) {
        static int run_c54x = -1;
        if (run_c54x < 0) {
            const char *e = getenv("CALYPSO_DSP_RUN_C54X");
            run_c54x = (e && *e == '1') ? 1 : 0;
        }
        /* [2026-07-27] anti double-run : depuis le split du gate, le tick TDMA
         * natif execute deja le DSP en mode ASSIST. Ce runner n a de sens que
         * si le shunt SUBSTITUE (ou sur demande explicite). */
        static int _drive = -1;
        if (_drive < 0) { const char *d = getenv("CALYPSO_SHUNT_DRIVE_DSP");
                          _drive = (d && *d == '1') ? 1 : 0; }
        if (run_c54x && (_drive || calypso_dsp_shunt_substitutes()))
            shunt_route_to_c54x_run();
    }

    /* [2026-07-26] SHUNT_LEGIT (option 3) HORS gate pending : quand gr-gsm a
     * DETECTE la SCH (sb_valid), transporter la vraie detection vers le DSP
     * result (d_fb_det=1 + TOA/SNR reels) A CHAQUE trame, independamment du
     * dispatch natif (qui ne pose jamais d_fb_det -- mur RANK3). L'ARM L1 lit
     * d_fb_det -> deroule le vrai flux FBSB->SB->BSIC->sysinfo. Thread DSP = safe. */
    {
        /* @BEQUILLE — SHUNT_LEGIT (transport du resultat FB au frame-tick)
         *              (CALYPSO_SHUNT_LEGIT ou CALYPSO_SHUNT_NO_LEGIT =1)
         *   masque  : le correlateur natif qui ne pose jamais d_fb_det. On ecrit
         *             directement api_ram[0x08F8..0x08FD] (d_fb_det=1, TOA/PM/ANGLE/SNR)
         *             et a_pm des que gr-gsm a decode la SCH (sb_valid).
         *   retirer : quand data[0x08f8] est ecrit par le DSP (RANK3 leve).
         */
        /* [2026-07-30] DECOUPLAGE — ce bloc etait soude aux parapluies
         * SHUNT_LEGIT / SHUNT_NO_LEGIT, alors qu'il fait deux choses de natures
         * DIFFERENTES qui n'ont rien a faire couplees :
         *
         *   (a) SUPPLEER UN BLOC ABSENT : le transport du resultat de la chaine
         *       analogique (TWL/gr-gsm) vers l'API, la boucle AFC fermee
         *       (rx_afc recalcule chaque tick), a_pm/rxlev. Le DSP ne sait pas
         *       encore faire ca -> legitime, et necessaire pour mesurer le reste.
         *   (b) ECRASER UN RESULTAT : poser rx_fb_det = 1, que l'ARM relit ensuite
         *       par calypso_dsp_shunt_real_fb_read() (offsets 0x01F0/0x01F4/0x01FA).
         *       CELA SEUL interdit de juger le correlateur natif.
         *
         * Consequence vecue le 30/07 : `CALYPSO_SHUNT_REAL_FB=0` desarme bien le
         * helper (calypso_dsp_helper.c:259) mais PAS ce bloc-ci, donc les lectures
         * de d_fb_det rendaient 1 alors que les seules ecritures DSP mesurees
         * valaient 0 (0xb2cc `st #0x0000`, 0x778a `andm #0xfffe`). On croyait tenir
         * une detection ; c'etait l'interception. Il n'existait AUCUNE combinaison
         * de gates donnant le transport ET un d_fb_det honnete.
         *
         * Desormais : (a) reste sous les parapluies, (b) a son propre gate.
         *   CALYPSO_SHUNT_PUBLISH_FB=1  -> publie rx_fb_det (ancien comportement)
         *   CALYPSO_SHUNT_PUBLISH_FB=0  -> transport ouvert, resultat NON substitue
         *                                  => data[0x08f8] devient le verdict
         * Defaut = valeur du parapluie, pour ne RIEN changer aux runs existants.
         * Le mode `native_twl` (cf. doc/ETAT_ACTUEL.md §12.9) le pose a 0. */
        static int legit = -1, publish = -1;
        if (legit < 0) { const char *e = getenv("CALYPSO_SHUNT_LEGIT"); const char *nl = getenv("CALYPSO_SHUNT_NO_LEGIT"); legit = ((e && *e == '1') || (nl && *nl=='1')) ? 1 : 0; }
        if (publish < 0) {
            publish = calypso_gate("CALYPSO_SHUNT_PUBLISH_FB", legit);
            fprintf(stderr, "[shunt] PUBLISH_FB = %d (transport=%d) : d_fb_det %s\n",
                    publish, legit,
                    publish ? "SUBSTITUE par l'hote (bequille)"
                            : "laisse au DSP -> data[0x08f8] est le verdict");
            fprintf(stderr, "[shunt] transport analogique (AFC/a_pm/TOA) : %s\n",
                    (legit || publish) ? "OUVERT" : "ferme");
        }
        /* [2026-07-30, correctif du decouplage] La condition etait
         * `legit && publish`. Bug : `native_twl` ne pose PAS le parapluie mais
         * demande explicitement PUBLISH_FB=1 — donc `legit=0` tuait tout le
         * bloc, transport COMPRIS (AFC ferme, a_pm, rx_toa). Mesure du 30/07 :
         *   [shunt] PUBLISH_FB = 1 (transport=0)
         *   [fbsb]  fb0_att=13 sb_att=8  api[](det=0 ...)
         * ([2026-08-03] le `fb0_ret=0` de cette trace est retire : compteur mort,
         *  jamais incremente. C'est `api[](det=0)` qui portait la demonstration.)
         * L'hote ne publiait donc RIEN, alors que le profil promet « FB/SB =
         * TWL ». Le mobile recevait un SB (INJECT_SB -> BSIC=7) mais jamais de
         * detection FB, d'ou une reselection de cellule en boucle toutes les 10 s.
         *
         * Correct : le bloc tourne si l'un OU l'autre le demande, et seule la
         * SUBSTITUTION du resultat (b) reste sous `publish`. Table de verite :
         *   legit=1 publish=1 (defaut)  -> transport + substitution   (inchange)
         *   legit=1 publish=0           -> transport seul, verdict au DSP
         *   legit=0 publish=1 (native_twl) -> transport + substitution  [CORRIGE]
         *   legit=0 publish=0 (native)  -> rien                        (inchange) */
        if ((legit || publish) && g_shunt.c54x && g_shunt.sb_valid) {
            /* L'ARM lit d_fb_det/snr/toa via calypso_dsp_shunt_real_fb_read
             * (0x01F0/0x01FA/0x01F4) qui retourne g_shunt.rx_*. On pose ces
             * champs depuis la detection gr-gsm reelle -> l'ARM voit FB found. */
            if (publish) {
                g_shunt.rx_fb_det = 1;   /* (b) SUBSTITUTION — sous `publish` seul */
            }
            /* [AFC loop-close v2 2026-07-26] recalcule rx_afc CHAQUE tick :
             * brut(FCCH memorise) - freq DEJA compensee par le DAC courant
             * (get_afc_hz). Sans ca, feed_iq/det gele rx_afc une fois le DAC
             * enroule -> le firmware enroule au rail sur une valeur stale. Ici
             * l'erreur effective DECROIT a mesure que le DAC monte -> converge. */
            if (g_rx_raw_valid) {
                double _eff = g_rx_raw_hz - calypso_twl3025_get_afc_hz();
                double _a = _eff * (65536.0 / 86208.0);
                if (_a >  32767.0) _a =  32767.0;
                if (_a < -32768.0) _a = -32768.0;
                g_shunt.rx_afc = (int16_t)_a;
                static unsigned _afl = 0;
                if ((_afl++ % 200) == 0)
                    fprintf(stderr, "[AFC-LOOP] raw=%.0fHz dac_hz=%.0f eff=%.0fHz -> rx_afc=%d\n",
                            g_rx_raw_hz, calypso_twl3025_get_afc_hz(), _eff, g_shunt.rx_afc);
            }
            /* [DECAN model-fidelity] garder le VRAI rx_snr (coherence feed_iq)
             * quand DECAN_SNR actif, sinon le de-can SNR est defait ici. */
            static int dsnr = -1;
            if (dsnr < 0) {
                const char *Ms = getenv("CALYPSO_DECAN");
                const char *ss = getenv("CALYPSO_DECAN_SNR");
                dsnr = ((Ms && Ms[0] == '1') || (ss && ss[0] == '1'));
            }
            if (!dsnr) g_shunt.rx_snr = 0x7000;   /* SNR canne (fallback) */
            g_shunt.rx_toa    = (uint16_t)g_shunt.sb_toa;
            /* [2026-07-26] FORMAT NATIF : ecrire la fenetre api_ram PARTAGEE
             * (c54x->api_ram) EXACTEMENT comme le DSP no-shunt le ferait :
             * DSP word W -> api_ram[W - C54X_API_BASE], lu par l'ARM sans intercept.
             * shunt_dispatch_fb ecrivait BASE_API_NDB+NDB_D_FB_DET => mauvaise
             * cellule (api_ram[0x550] au lieu de [0xF8]). Ici on pose le VRAI bloc
             * FB (a_sync_demod) aux offsets natifs, ce que le firmware polle. */
            if (g_shunt.c54x->api_ram) {
                uint16_t *ar = g_shunt.c54x->api_ram;
                ar[0x08F8 - C54X_API_BASE] = 1;                        /* d_fb_det = FOUND */
                ar[0x08FA - C54X_API_BASE] = (uint16_t)g_shunt.sb_toa; /* a_sync_TOA */
                ar[0x08FB - C54X_API_BASE] = g_shunt.last_pm;          /* a_sync_PM  */
                ar[0x08FC - C54X_API_BASE] = (uint16_t)g_shunt.rx_afc; /* a_sync_ANG */
                ar[0x08FD - C54X_API_BASE] = dsnr ? g_shunt.rx_snr : 0x7000; /* a_sync_SNR (DECAN) */

                /* [2026-07-30] ACQUITTEMENT VERS LE DSP — CALYPSO_TWL_ACK_DSP.
                 *
                 * Le bloc ci-dessus ecrit `api_ram[]`, la vue ARM. Le DSP, lui,
                 * lit `data[]` : deux tableaux distincts du modele. Consequence
                 * mesuree le 30/07 en `native_twl` :
                 *     api[] (det=1 toa=23 pm=20929 ang=-186 snr=0x735b)
                 *     data[](det=0 toa=0  pm=0     ang=0    snr=0x0000)
                 *     fb0_att=17  sb_att=9
                 * ([2026-08-03] `fb0_ret=0` retire de cette trace : compteur
                 *  mort. La divergence api[]/data[] ci-dessus est la mesure.)
                 * L'hote a dit a l'ARM que la FB etait trouvee, et ne l'a JAMAIS
                 * dit au DSP : sa machine d'etat reste bloquee a l'etape FB,
                 * cherche 17 fois, et n'atteint jamais le CCCH — c'est-a-dire la
                 * question meme que ce banc pose.
                 *
                 * Formule par l'utilisateur : « il faut qu'en mode native_twl
                 * l'ARM ack (det=1 toa=23 pm=…) au DSP ».
                 *
                 * @BEQUILLE — TWL_ACK_DSP
                 *   c'en est une : `data[0x08F8..0x08FD]` est desormais ECRIT PAR
                 *     L'HOTE. Ces cinq cellules cessent d'etre un verdict — on ne
                 *     peut plus citer `data[0x08f8]` comme « ce que le DSP a
                 *     produit ». C'est le prix pour que la machine d'etat avance.
                 *   masque : l'incapacite du correlateur natif a poser d_fb_det.
                 *   ce qui reste HONNETE : `a_cd` (0x09D0..) et donc `WATCH-ACD`.
                 *     La question « le DSP traite-t-il le SI ? » garde son juge,
                 *     puisque rien ici n'ecrit a_cd.
                 *   retirer : quand le correlateur natif pose d_fb_det seul.
                 * Defaut 0 — le profil `native_twl` le pose a 1.
                 */
                {
                    static int ack = -1;
                    if (ack < 0) {
                        ack = calypso_gate("CALYPSO_TWL_ACK_DSP", 0);
                        if (ack)
                            fprintf(stderr, "[shunt] TWL_ACK_DSP=1 : le resultat FB "
                                    "hote est aussi ecrit dans data[0x08F8..0x08FD] "
                                    "-> le DSP voit la FB trouvee et peut avancer. "
                                    "BEQUILLE : ces 5 cellules ne sont plus un "
                                    "verdict ; le juge du SI, lui, reste intact.\n");
                    }
                    if (ack && g_shunt.c54x->data) {
                        uint16_t *dd = g_shunt.c54x->data;
                        dd[0x08F8] = 1;
                        dd[0x08FA] = (uint16_t)g_shunt.sb_toa;
                        dd[0x08FB] = g_shunt.last_pm;
                        dd[0x08FC] = (uint16_t)g_shunt.rx_afc;
                        dd[0x08FD] = dsnr ? g_shunt.rx_snr : 0x7000;
                    }
                }

                /* [2026-07-30] L'ACQUITTEMENT VERS L'ARM — CALYPSO_TWL_ACK_ARM.
                 *
                 * Symetrique de TWL_ACK_DSP. Quand l'hote realise une tache a la
                 * place du DSP, il faut aussi POSER LA COMPLETION dans la page de
                 * lecture, sinon l'ARM ne sait pas que c'est fini. Le firmware est
                 * explicite (prim_rx_nb.c:72) :
                 *     if (dsp_api.db_r->d_task_d == 0) { puts("EMPTY"); return 0; }
                 * `d_task_d` a zero => rapport jete avant meme le test du burst-id.
                 *
                 * On ECHO la tache commandee (d_task_md, page d'ecriture 0 ou 1)
                 * dans `d_task_d` des DEUX pages de lecture (0x0828, 0x083c), dans
                 * les deux tableaux du modele. On ne touche PAS `d_burst_d` : son
                 * desaliasage fonctionne depuis ce soir (calypso_trx.c), et deux
                 * mecanismes sur la meme cellule, c'est exactement le conflit qu'on
                 * vient de defaire.
                 *
                 * @BEQUILLE — TWL_ACK_ARM
                 *   c'en est une : la COMPLETION est fabriquee par l'hote.
                 *   masque : le DSP qui n'acquitte pas les taches qu'il n'execute pas.
                 *   ce qu'elle NE fabrique PAS : `a_cd`. Le juge du SI reste intact.
                 *   retirer : quand le DSP execute et acquitte lui-meme.
                 * Defaut 0 ; le profil `native_twl` le pose a 1.
                 */
                {
                    static int acka = -1;
                    if (acka < 0) {
                        acka = calypso_gate("CALYPSO_TWL_ACK_ARM", 0);
                        if (acka)
                            fprintf(stderr, "[shunt] TWL_ACK_ARM=1 : la completion "
                                    "de tache est posee dans d_task_d (0x0828/0x083c) "
                                    "-> l'ARM ne voit plus « EMPTY ». BEQUILLE : la "
                                    "completion est fabriquee, le contenu non.\n");
                    }
                    if (acka && g_shunt.c54x->data) {
                        uint16_t *dd = g_shunt.c54x->data;
                        uint16_t md = dd[0x0804] ? dd[0x0804] : dd[0x0818];
                        if (md) {
                            if (shunt_one_page_on()) {
                                dd[shunt_cur_rpage() ? 0x083C : 0x0828] = md;
                            } else {
                                dd[0x0828] = md;
                                dd[0x083C] = md;
                            }
                            if (g_shunt.c54x->api_ram) {
                                uint16_t *a2 = g_shunt.c54x->api_ram;
                                if (shunt_one_page_on()) {
                                    a2[(shunt_cur_rpage() ? 0x083C : 0x0828)
                                       - C54X_API_BASE] = md;
                                } else {
                                    a2[0x0828 - C54X_API_BASE] = md;
                                    a2[0x083C - C54X_API_BASE] = md;
                                }
                            }
                        }
                    }
                }
                /* [2026-07-26 RANK5] a_pm (rxlev) au format natif : le vrai DSP
                 * ecrit a_pm=0 sur les read pages -> ecrase dispatch_pm. On pose
                 * directement, chaque tick (apres le run DSP), la valeur calibree
                 * trf6151 aux offsets read-page exacts (p0 woff 0x30..32, p1 0x44..46)
                 * lus par l1ddsp_meas_read (dsp_api.db_r->a_pm[i]). */
                {
                    static int trf = -1, target = -60;
                    if (trf < 0) {
                        /* [2026-08-03] cf. calypso_c54x.c : `=0` explicite doit couper. */
                        const char *l = getenv("CALYPSO_SHUNT_LEGIT");
                        const char *t = getenv("CALYPSO_TRF_TARGET_RF");
                        trf = calypso_gate("CALYPSO_TRF_RXLEV", (l && *l == '1') ? 1 : 0);
                        if (t && *t) target = atoi(t);
                    }
                    if (trf) {
                        uint16_t apm = shunt_pm_decan_apm(target);
                        ar[0x30] = apm; ar[0x31] = apm; ar[0x32] = apm;  /* read page 0 */
                        ar[0x44] = apm; ar[0x45] = apm; ar[0x46] = apm;  /* read page 1 */
                    }
                }
            }
            shunt_dispatch_fb(0);
            /* SB : encode le burst SB depuis gr-gsm (BSIC=%d/sb_fn) sur les 2 pages
             * -> l'ARM decode BSIC reel + FN au lieu de BSIC=0/vide. */
            if (shunt_one_page_on()) {
                shunt_dispatch_sb(shunt_cur_rpage());   /* page courante seulement */
            } else {
                shunt_dispatch_sb(0);
                shunt_dispatch_sb(1);
            }

            /* [2026-07-30] TWL_ACK_SB — publier a_sch dans data[], la vue que le
             * firmware lit REELLEMENT.
             *
             * @BEQUILLE — TWL_ACK_SB  (CALYPSO_TWL_ACK_SB, defaut 0)
             *   (1) C'EST UNE BEQUILLE : le resultat SB est fabrique par l'hote a
             *       partir de gr-gsm, pas demodule par le DSP.
             *   (2) CE QU'ELLE MASQUE : le DSP n'ecrit jamais a_sch. Or le firmware
             *       REPOSE lui-meme le bit d'echec a chaque bascule de page —
             *       osmocom-bb layer1/sync.c, dans l1_sync() :
             *           dsp_api.db_r->a_sch[0] = (1<<B_SCH_CRC);
             *           /\* TSM30 does it: Set crc result as "SB not found". *\/
             *       et l1s_sbdet_resp() (prim_fbsb.c:181) rejette sur ce bit. Le
             *       defaut est donc « SB non trouve » : sans ecriture DSP, le test
             *       echoue A TOUS LES COUPS, quoi que trouve le correlateur.
             *   (3) QUAND LA RETIRER : quand A_SCH-WR (calypso_c54x.c:3492, zone
             *       0x0837..0x083B / 0x084B..0x084F) montre le DSP ecrivant a_sch
             *       lui-meme avec le bit CRC a 0.
             *   CE QU'ELLE NE FABRIQUE PAS : a_cd. Le juge du SI (A_CD-WR, sans
             *   gate) reste intact — c'est la question que ce banc pose.
             *
             * POURQUOI ICI ET PAS DANS shunt_dispatch_sb : celui-ci publie via
             * shunt_write_w() = dma_memory_write() sur BASE_API_R_PAGE_0/1. Le
             * commentaire du bloc a_cd, quelques lignes plus bas, dit que ce chemin
             * n'atteint PAS ce que lit le firmware (dsp->data via calypso_trx.c:213)
             * — c'est pour cela qu'a_cd a ete bascule en ecriture directe. a_sch
             * etait reste sur l'ancien chemin. On aligne les deux.
             */
            {
                static int _asch = -1;
                if (_asch < 0) {
                    _asch = calypso_gate("CALYPSO_TWL_ACK_SB", 0);
                    if (_asch)
                        fprintf(stderr, "[shunt] TWL_ACK_SB=1 : a_sch[0..4] ecrit en "
                                "direct dans data[0x0837..] / [0x084B..] (les deux "
                                "pages de lecture) avec B_SCH_CRC EFFACE. BEQUILLE : "
                                "le SB est fabrique depuis gr-gsm ; a_cd, lui, n'est "
                                "pas touche — le juge du SI reste intact.\n");
                }
                if (_asch && g_shunt.sb_valid && g_shunt.c54x && g_shunt.c54x->data) {
                    uint32_t _fn = g_shunt.sb_fn;
                    uint32_t _sb = shunt_encode_sb(g_shunt.sb_bsic,
                                                   (uint16_t)(_fn / (26u * 51u)),
                                                   (uint8_t)(_fn % 26u),
                                                   (uint8_t)(_fn % 51u));
                    uint16_t *dd = g_shunt.c54x->data;
                    /* db_r p0 a_sch[0..4] = 0x0837..0x083B ; p1 = 0x084B..0x084F */
                    static const uint16_t _p[2] = { 0x0837, 0x084B };
                    int _k0 = 0, _k1 = 2;
                    if (shunt_one_page_on()) {          /* page courante seulement */
                        _k0 = shunt_cur_rpage(); _k1 = _k0 + 1;
                    }
                    for (int _k = _k0; _k < _k1; _k++) {
                        uint16_t b = _p[_k];
                        dd[b + 0] = 0x0000;                      /* CRC clear = pass */
                        dd[b + 1] = 0x0000;                      /* inutilise        */
                        dd[b + 2] = 0x0000;                      /* inutilise        */
                        dd[b + 3] = (uint16_t)(_sb & 0xFFFF);
                        dd[b + 4] = (uint16_t)(_sb >> 16);
                        if (g_shunt.c54x->api_ram) {
                            uint16_t *aa = g_shunt.c54x->api_ram;
                            aa[b + 0 - C54X_API_BASE] = 0x0000;
                            aa[b + 1 - C54X_API_BASE] = 0x0000;
                            aa[b + 2 - C54X_API_BASE] = 0x0000;
                            aa[b + 3 - C54X_API_BASE] = (uint16_t)(_sb & 0xFFFF);
                            aa[b + 4 - C54X_API_BASE] = (uint16_t)(_sb >> 16);
                        }
                    }
                    static unsigned _n = 0;
                    if (_n++ < 20 || (_n % 500) == 0)
                        fprintf(stderr, "[shunt] TWL_ACK_SB #%u a_sch <- sb=0x%08x "
                                "BSIC=%u FN=%u (CRC efface, 2 pages, data[]+api_ram[])\n",
                                _n, _sb, g_shunt.sb_bsic, _fn);
                }
            }
            /* [2026-07-26 camp] SI -> a_cd sur le VRAI array data[] (le firmware lit
             * dsp->data via calypso_trx.c:213, PAS dsp_ram ou vont les shunt_write_w).
             * a_cd @ NDB_A_CD=0x1FC -> data word 0x9D2 (a_cd[0]), SI3 en a_cd[3]=0x9D5.
             * Rotation SI1/2/3/4 toutes les 8 ticks (stable sur un bloc de 4 bursts)
             * -> le mobile collecte tout le set au fil des blocs. Packing = m[i]|(m[i+1]<<8). */
            /* [2026-07-26 LU] NE PAS ecraser a_cd avec le SI du camp quand un DL
             * DEDIE (SDCCH UA/AUTH/LU-ACCEPT, ou AGCH IMM-ASSIGN) est en attente :
             * dispatch_allc presente le UA/IMM-ASSIGN dans data[0x9D2] sur son bloc,
             * et l'ecriture SI chaque tick le clobbait -> SABM jamais confirme (T3211
             * retry). En mode dedie le mobile ne lit pas le BCCH -> SI inutile ici. */
            /* [no-cell-info fix 2026-07-26] SI camp supprime SEULEMENT si un IMM
             * ASSIGN (mt 0x3f/0x3a/0x3b) est pending sur l'AGCH (fenetre dediee : ne
             * pas clobber le grant) -- PAS sur le PAGING de routine (0x21/0x22/0x24)
             * qui tourne en continu en idle et affamait le SI3 (SI3 livre que pendant
             * l'acquisition -> moniteur famine -> no-cell-info chaque seconde).
             * agch_buf[2] = mt du dernier AGCH. LU intact (IMM ASSIGN protege). */
            /* [2026-07-27 MT-SMS fix] EXPIRE l'IMM-ASSIGN latche : sans clear,
             * agch_valid poisonnait le SI (gate ci-dessous) ET le paging (feed_agch)
             * a vie -> no-cell-info permanent apres un canal dedie. Meme TTL que le
             * drop paging (CALYPSO_SHUNT_AGCH_TTL, def 100). SI + paging reprennent. */
            {
                /* [2026-07-27] OPT-IN (defaut OFF apres regression MO-SMS shunt_legit) :
                 * l'expiry agch corrige le no-cell-info post-dedie MAIS clear le grant
                 * IMM-ASSIGN -> peut casser l'etablissement SDCCH. Activer via
                 * CALYPSO_SHUNT_AGCH_EXPIRE=1 (no-legit / experiences). */
                /* @BEQUILLE — SHUNT_AGCH_EXPIRE (+ SHUNT_AGCH_TTL)  (CALYPSO_SHUNT_AGCH_EXPIRE,
                 *              atoi>0, defaut OFF ; shunt_no_legit.env:=1)
                 *   masque  : rien de reel — c'est le correctif d'une autre bequille : le latch
                 *             IMM-ASSIGN fabrique par feed_agch empoisonne le gate SI et le paging
                 *             a vie. On le fait expirer au bout d'un TTL.
                 *   retirer : avec l'injection AGCH elle-meme (INJECT_AGCH) — quand a_cd est
                 *             alimente par le decodeur natif, il n'y a plus de latch a expirer.
                 *   NB      : SHUNT_AGCH_TTL a 3 consommateurs de semantiques differentes.
                 */
                static int _agex_on = -1, _agex = 100;
                if (_agex_on < 0) { const char *e = getenv("CALYPSO_SHUNT_AGCH_EXPIRE"); _agex_on = (e && atoi(e) > 0) ? 1 : 0;
                    const char *t = getenv("CALYPSO_SHUNT_AGCH_TTL"); if (t && *t) _agex = atoi(t); }
                if (_agex_on && g_shunt.agch_valid && (uint32_t)(g_shunt.tick_cnt - g_shunt.agch_tick) > (uint32_t)_agex)
                    g_shunt.agch_valid = false;
            }
            /* [2026-08-08] ... ni quand un TCH est arme. a_cd est partage : le camp
             * y met les SI, et l1s_tch_a_resp y lit la SACCH du canal dedie. Ce bloc
             * tourne a CHAQUE tick (hors gate pending) tandis que le dispatch SACCH
             * ne tourne qu'au tick pending -> sans cette condition, la SACCH du dedie
             * serait ecrasee par un SI entre deux dispatch, et le mobile lirait un SI
             * la ou il attend un rapport de mesure. Meme raison, meme forme que le
             * garde-fou sdcch_valid juste a cote. En dedie le mobile ne lit pas le
             * BCCH : aucun SI n'est perdu. */
            if (g_shunt.si_valid && !g_shunt.sdcch_valid && !shunt_dcch_si_guard()
                && !g_shunt.tch_cfg_valid
                && !(g_shunt.agch_valid && (g_shunt.agch_buf[2] == 0x3f
                     || g_shunt.agch_buf[2] == 0x3a || g_shunt.agch_buf[2] == 0x3b))
                && g_shunt.c54x && g_shunt.c54x->data) {
                /* [no-cell-info fix v2 2026-07-26] rotation SI sur compteur LIBRE
                 * (avance CHAQUE tick, hors pending) et PAS sur tick_cnt qui stalle
                 * en idle (incremente seulement apres le gate pending, l.733) ->
                 * sinon si_rr FIGE une fois came et SI3 n'est plus livre en idle ->
                 * moniteur cellule servante famine -> no-cell-info chaque seconde. */
                static unsigned si_rot = 0;
                /* [2026-07-27] rotation SI accELErEe : avance si_rr a chaque bloc
                 * (mask 0) au lieu de tous les 8 -> le mobile re-collecte SI1+SI2+SI3
                 * VITE apres un dedie (sinon sync timeout -> No service post-SMS).
                 * Tunable CALYPSO_SHUNT_SI_ROT_MASK (0=chaque bloc, 7=ancien). */
                /* @BEQUILLE — SHUNT_SI_ROT_MASK  (CALYPSO_SHUNT_SI_ROT_MASK, VALEUR, defaut 7)
                 *   masque  : l'ordonnancement mf-51 des SI que le DSP devrait produire : on fait
                 *             tourner a la main les slots si_set[] pour re-livrer SI1..SI4.
                 *   retirer : quand a_cd est alimente par la demodulation native (les SI arrivent
                 *             alors a leur place dans le multiframe).
                 */
                static int _rotmask = -1;
                if (_rotmask < 0) { const char *e = getenv("CALYPSO_SHUNT_SI_ROT_MASK"); _rotmask = (e && *e) ? atoi(e) : 7; }
                if ((si_rot++ & (unsigned)_rotmask) == 0) {
                    for (int k = 1; k <= 6; k++) {
                        int si = (g_shunt.si_rr + k) % 6;
                        if (g_shunt.si_set_have[si]) { g_shunt.si_rr = si; break; }
                    }
                }
                const uint8_t *si = g_shunt.si_set[g_shunt.si_rr];
                uint16_t *d = g_shunt.c54x->data;
                d[0x9D2] = 0x0000;   /* a_cd[0] FIRE/CRC = pass    */
                d[0x9D3] = 0x0000;   /* a_cd[1]                    */
                d[0x9D4] = 0x0000;   /* a_cd[2] num_biterr = 0     */
                for (int i = 0; i < 23; i += 2) {   /* a_cd[3..14] = data[0x9D5..0x9E0] */
                    uint8_t lo = si[i], hi = (i + 1 < 23) ? si[i + 1] : 0x2B;
                    d[0x9D5 + i / 2] = (uint16_t)(lo | (hi << 8));
                }
                /* [2026-07-26 camp] a_serv_demod des READ PAGES du NB (words 8..11) :
                 * read page0 = data[0x830..0x833], page1 = data[0x844..0x847].
                 * nb_resp lit ces 4 mots par burst pour l'AFC (afc_input) + rx_level.
                 * Quand BURST_OFS aligne les 4 bursts, ils sont TOUS traites -> sans
                 * ces valeurs, afc_input(garbage) fait DERIVER l'AFC -> sync perdue ->
                 * SI casses. On pose D_ANGLE=0 (aucune erreur de freq -> AFC stable),
                 * D_SNR haut (>AFC_SNR_THRESHOLD=2560), D_TOA=23, D_PM = a_pm calibre. */
                {
                    /* [DECAN model-fidelity 2026-07-26] gate par modele (+ master
                     * CALYPSO_DECAN). OFF par defaut => cannes de stabilisation a
                     * l'identique (baseline LU-accept intact). ON => vraie sortie du
                     * modele emule (feed_iq / trf6151 / gr-gsm) pour verifier s'il
                     * tient le camp. rx_snr/rx_afc exigent CALYPSO_SHUNT_REAL_FB=1
                     * (sinon rx_snr reste 0x7000 canne et rx_afc reste 0). */
                    /* @BEQUILLE — DECAN_TOA / DECAN_SNR / DECAN_ANGLE / DECAN_PM  (CALYPSO_DECAN_*
                     *              ou master CALYPSO_DECAN ; l'ETAT OFF est la bequille)
                     *   masque  : OFF -> a_serv_demod des read-pages recoit des constantes cannees :
                     *             TOA=23, SNR=0x7000, ANGLE=0, PM=apm_for_rf(-60), a la place de la
                     *             sortie du modele (sb_toa gr-gsm, rx_snr / rx_afc de feed_iq, MAV).
                     *             Elles masquent l'absence de mesure DSP native.
                     *   retirer : quand le correlateur natif ecrit lui-meme a_sync_demod
                     *             [TOA/PM/ANGLE/SNR].
                     *   NB      : dc_pm calcule ici n'alimente que la condition du fprintf ; le gate
                     *             PM effectif est dans shunt_pm_decan_apm().
                     */
                    static int dc_toa = -1, dc_pm, dc_snr, dc_ang;
                    if (dc_toa < 0) {
                        const char *M = getenv("CALYPSO_DECAN");
                        int m = (M && M[0] == '1');
                        const char *t = getenv("CALYPSO_DECAN_TOA");
                        const char *p = getenv("CALYPSO_DECAN_PM");
                        const char *s = getenv("CALYPSO_DECAN_SNR");
                        const char *a = getenv("CALYPSO_DECAN_ANGLE");
                        dc_toa = m || (t && t[0] == '1');
                        dc_pm  = m || (p && p[0] == '1');
                        dc_snr = m || (s && s[0] == '1');
                        dc_ang = m || (a && a[0] == '1');
                    }
                    uint16_t pm_c  = calypso_trf6151_apm_for_rf(-60);
                    uint16_t toa_v = (dc_toa && g_shunt.sb_valid) ? (uint16_t)g_shunt.rx_toa : 23;
                    uint16_t pm_v  = shunt_pm_decan_apm(-60);
                    uint16_t ang_v = dc_ang ? (uint16_t)g_shunt.rx_afc   : 0;
                    uint16_t snr_v = dc_snr ? g_shunt.rx_snr            : 0x7000;
                    /* page 0 */ d[0x830]=toa_v; d[0x831]=pm_v; d[0x832]=ang_v; d[0x833]=snr_v;
                    /* page 1 */ d[0x844]=toa_v; d[0x845]=pm_v; d[0x846]=ang_v; d[0x847]=snr_v;
                    {
                        static unsigned _dcl = 0;
                        if ((dc_toa || dc_pm || dc_snr || dc_ang) && _dcl++ < 24)
                            fprintf(stderr, "[DECAN] wrote toa=%u(c23) pm=%u(c%u) ang=%d(c0) "
                                    "snr=0x%x(c0x7000) | model: sb_valid=%d rx_toa=%u last_pm=%u "
                                    "rx_afc=%d rx_snr=0x%x\n",
                                    toa_v, pm_v, pm_c, (int16_t)ang_v, snr_v,
                                    g_shunt.sb_valid, g_shunt.rx_toa, g_shunt.last_pm,
                                    g_shunt.rx_afc, g_shunt.rx_snr);
                    }
                }
                static unsigned _acd = 0;
                if (_acd++ < 5000)
                    SHUNT_LOG("CAMP: a_cd<-SI type_slot=%d have[0-5]=%d%d%d%d%d%d tick=%u",
                              g_shunt.si_rr, g_shunt.si_set_have[0],g_shunt.si_set_have[1],g_shunt.si_set_have[2],g_shunt.si_set_have[3],g_shunt.si_set_have[4],g_shunt.si_set_have[5], g_shunt.tick_cnt);
            } else if (shunt_dcch_si_guard()
                       && !shunt_tch_fresh(g_shunt.tsacch_dl_valid, g_shunt.tsacch_dl_tick)
                       && !g_shunt.tch_cfg_valid
                       && g_shunt.c54x && g_shunt.c54x->data) {
                /* Canal dedie, garde armee, et AUCUNE source de SACCH descendante :
                 * c est le cas du SDCCH pendant tout l etablissement d appel. On ne
                 * laisse ni le SI du camp (0x07) ni le vide (famine) -- on presente
                 * une trame de bourrage LAPDm valide. Cf. shunt_dcch_sacch_fill(). */
                shunt_dcch_sacch_present(g_shunt.c54x->data);
            }
            static unsigned _lg = 0;
            if (_lg++ < 12)
                SHUNT_LOG("SHUNT_LEGIT: detection gr-gsm reelle (fn=%u bsic=%d toa=%d) "
                          "-> d_fb_det pose au DSP result (hors pending, MAC court-circuite)",
                          g_shunt.sb_fn, g_shunt.sb_bsic, (int)g_shunt.sb_toa);
        }
    }

    if (!g_shunt.pending) {
        return;
    }
    g_shunt.tick_cnt++;

    /* SONDE A_CD-RD — le firmware LIT-il le canal dedie ?
     *
     * CE QU ON CHERCHE. Le LU echoue alors que TOUT l amont est prouve sain :
     * RACH emis, IMM ASSIGN decodee (chan_nr 0x28 -> SDCCH/4 SS1), fenetre
     * descendante juste (fn%51 26-29, offset confirme par le DWARF de l ELF
     * charge), base montante juste (ul_base=UL4[ss], dcch_cfg present et exact),
     * UA present a l entree du shunt (10 blocs c=0x73), sorti du ring sans
     * debordement ni eviction, et la MSC recoit la demande et alloue un TMSI.
     * Un seul ecart mesure : les DATA_IND remontes au mobile portent uniquement
     * chan_nr 0x80 (BCCH) et 0x90 (CCCH), JAMAIS 0x2x/0x3x. Rien du canal dedie
     * ne franchit la couche L1CTL.
     *
     * Le segment non instrumente est donc : presentation dans a_cd -> lecture par
     * le firmware -> emission du DATA_IND. Cette sonde coupe ce segment en deux.
     *
     * L INSTRUMENT. Meme principe que A_DD_0-RD, qui a tranche pour la voix :
     * le lecteur est le firmware emule, on ne peut pas l instrumenter, mais
     * chaque transition 1 -> 0 de B_BLUD EST une lecture, et chaque 0 -> 1 une
     * presentation. Ne consomme rien.
     *
     * ⚠️ LIMITE CONNUE, a garder en tete avant d interpreter : on echantillonne
     * en tete de tick et APRES le retour anticipe « if (!g_shunt.pending) », donc
     * on est aveugle aux cycles sub-tick. Sur a_dd_0 la meme sonde sous-comptait
     * d environ 9 %. Elle repond a « le firmware lit-il, oui ou non », PAS a
     * « combien exactement ».
     *
     * CALYPSO_ACD_PROBE=1 pour l activer ; inerte sinon. */
    {
        static int on = -1;
        if (on < 0) {
            const char *e = getenv("CALYPSO_ACD_PROBE");
            on = (e && *e == '1') ? 1 : 0;
            SHUNT_LOG("SONDE A_CD-RD %s\n", on ? "ACTIVE" : "inactive");
        }
        if (on && g_shunt.c54x && g_shunt.c54x->data) {
            uint16_t *dd = g_shunt.c54x->data;
            unsigned wcd = ndb_w(g_ndb.a_cd);
            int arme = (dd[wcd] & (1u << B_BLUD)) ? 1 : 0;
            static int precedent = -1;
            static unsigned long long presentations, lectures, pose_tick, attente_max;
            if (precedent < 0) precedent = arme;
            if (precedent == 0 && arme == 1) {
                presentations++;
                pose_tick = g_shunt.tick_cnt;
            } else if (precedent == 1 && arme == 0) {
                unsigned long long dt = g_shunt.tick_cnt - pose_tick;
                lectures++;
                if (dt > attente_max) attente_max = dt;
            }
            precedent = arme;
            /* Un seul point d impression, pour que les deux compteurs soient
             * CO-TEMPORELS : les comparer depuis deux lignes differentes du
             * journal fabrique un ratio imaginaire (paye le 09/08). */
            static unsigned long long tick_dernier;
            if (g_shunt.tick_cnt - tick_dernier >= 2000) {
                tick_dernier = g_shunt.tick_cnt;
                SHUNT_LOG("A_CD-RD : presentations=%llu lectures=%llu "
                          "non_lues=%lld pire_attente=%llu ticks\n",
                          presentations, lectures,
                          (long long)presentations - (long long)lectures,
                          attente_max);
            }
        }
    }

    /* SONDE A_DD_0-RD — a quelle cadence le FIRMWARE lit-il la voix ?
     *
     * CE QU ON CHERCHE. La garde bornee compte 1000 forcages pour ~5250 trames
     * servies : une sur cinq est presentee alors que la precedente n avait pas
     * ete lue, donc ecrasee. Et « profondeur=0 » pendant les retenues montre que
     * la file est VIDE quand on attend : le goulot n est pas entre le poll et le
     * dispatch, il est dans la lecture de a_dd_0 par le firmware, qui ne suit pas
     * 50/s. Reste a le CHIFFRER au lieu de le deduire.
     *
     * L INSTRUMENT. Le lecteur n est pas du code QEMU : c est le firmware emule
     * qui efface B_BLUD apres avoir pris le bloc. On ne peut donc pas l instrumenter
     * directement -- mais chaque transition 1 -> 0 du bit EST une lecture. On
     * echantillonne en tete de tick, avant que le dispatch ne reecrive.
     *
     * Ce qu on en tire : le nombre de lectures, l intervalle moyen en ticks entre
     * deux lectures (attendu ~4 si le firmware suit, plus si il decroche), et le
     * temps qu une trame passe non lue. Compteurs CUMULATIFS, sonde qui s annonce.
     * CALYPSO_ADD_PROBE=1 pour l activer ; inerte sinon. */
    {
        static int on = -1;
        if (on < 0) {
            const char *e = getenv("CALYPSO_ADD_PROBE");
            on = (e && *e == '1') ? 1 : 0;
            SHUNT_LOG("SONDE A_DD_0-RD %s\n", on ? "ACTIVE" : "inactive");
        }
        if (on && g_shunt.c54x && g_shunt.c54x->data) {
            uint16_t *dd = g_shunt.c54x->data;
            unsigned wdd = ndb_w(g_ndb.a_dd_0);
            int arme = (dd[wdd] & (1u << B_BLUD)) ? 1 : 0;
            static int precedent = -1;
            static unsigned long long lectures, ticks_cumules, pose_tick, attente_max;
            if (precedent < 0) precedent = arme;
            if (precedent == 0 && arme == 1) {
                pose_tick = g_shunt.tick_cnt;          /* le shunt vient de deposer */
            } else if (precedent == 1 && arme == 0) {
                unsigned long long dt = g_shunt.tick_cnt - pose_tick;
                lectures++; ticks_cumules += dt;
                if (dt > attente_max) attente_max = dt;
                if (lectures <= 3 || (lectures % 500) == 0)
                    SHUNT_LOG("A_DD_0-RD #%llu : lue apres %llu ticks "
                              "(moyenne %llu.%02llu, pire %llu)\n",
                              lectures, dt,
                              ticks_cumules / lectures,
                              (ticks_cumules * 100 / lectures) % 100,
                              attente_max);
            }
            precedent = arme;
        }
    }

    /* SONDE A_FU — l ASSIGNMENT COMPLETE atteint-il seulement le tampon ?
     *
     * CE QU ON CHERCHE. Le BSC echoue 41 fois aujourd hui sur
     * « Timeout (rll_ready=no, rtp_require=yes, voice_ready=yes) » : tout est
     * pret sauf l etablissement LAPDm sur le TCH. Le mobile, lui, journalise
     * « ASSIGNMENT COMPLETE (cause #0) » puis « ASSIGNMENT FAILURE (cause #1) ».
     * Or la capture FACCH montante du shunt ne rend que des trames de
     * supervision VIDES : RR (0x91), REJ (0x59), DISC (0x53), toutes avec un
     * octet de longueur a 0x01, c est-a-dire zero octet de L3. Jamais de SABM,
     * jamais de contenu.
     *
     * DEUX LECTURES POSSIBLES, et c est cette sonde qui tranche :
     *   (a) le firmware ne depose PAS l ASSIGNMENT COMPLETE dans a_fu ;
     *   (b) il l y depose, mais la fenetre de tache TCHT(13) du shunt rate la
     *       rafale qui le porte et ne voit que la supervision qui suit.
     * On regarde donc a_fu a CHAQUE tick, hors de toute fenetre de tache. Si un
     * bloc a longueur non nulle passe ici sans etre vu par la voie normale,
     * c est (b) -- ma fenetre. Si rien ne passe jamais, c est (a) -- en amont.
     *
     * NE CONSOMME RIEN : B_BLUD n est pas efface, sinon la sonde volerait les
     * blocs au chemin reel et fabriquerait la panne qu elle observe.
     * Compteurs CUMULATIFS, et elle s annonce au demarrage : une sonde muette
     * rend un silence indecidable. CALYPSO_AFU_PROBE=1 pour l activer.
     * Diagnostic pur : aucun effet sur le flux quand elle est eteinte. */
    {
        static int on = -1;
        if (on < 0) {
            const char *e = getenv("CALYPSO_AFU_PROBE");
            on = (e && *e == '1') ? 1 : 0;
            SHUNT_LOG("SONDE A_FU %s\n", on ? "ACTIVE" : "inactive");
        }
        if (on && g_shunt.c54x && g_shunt.c54x->data) {
            uint16_t *dd = g_shunt.c54x->data;
            unsigned wfu = ndb_w(g_ndb.a_fu);
            /* [2026-08-09] ON DETECTE UN CHANGEMENT, PAS UN DRAPEAU.
             * Premiere version : journaliser des que B_BLUD etait arme. Mesure :
             * 1600 lignes au contenu STRICTEMENT identique (e4 f4 74 f0), une par
             * tick, des avant le premier appel -- B_BLUD reste arme en permanence
             * a cet endroit et la charge est remanente. Une sonde qui parle sans
             * arret est aussi inutile qu une sonde muette : elle noie le signal.
             * On ne signale donc que les mots qui CHANGENT. */
            static unsigned long long vus, avec_l3, derniere_vue;
            static uint32_t precedent = 0xFFFFFFFFu;
            uint32_t courant = ((uint32_t)dd[wfu + 3] << 16) | dd[wfu + 4];
            if ((dd[wfu] & (1u << B_BLUD)) && courant != precedent) {
                precedent = courant;
                uint16_t w0 = dd[wfu + 3], w1 = dd[wfu + 4];
                uint8_t b0 = (uint8_t)(w0 & 0xff), b1 = (uint8_t)(w0 >> 8);
                uint8_t b2 = (uint8_t)(w1 & 0xff), b3 = (uint8_t)(w1 >> 8);
                unsigned l3 = (unsigned)(b2 >> 2);      /* champ longueur LAPDm */
                vus++;
                if (l3) avec_l3++;
                /* On journalise TOUT bloc porteur de L3 (le cas recherche), et
                 * seulement un echantillon des blocs vides, qui defilent. */
                if (l3 || vus <= 5 || (vus % 500) == 0) {
                    SHUNT_LOG("A_FU-PROBE #%llu : %02x %02x %02x %02x  L3=%u o  "
                              "task_u=0x%04x task_md=0x%04x  (avec L3 : %llu, "
                              "delta_tick=%llu)\n",
                              vus, b0, b1, b2, b3, l3,
                              g_shunt.d_task_u, g_shunt.d_task_md,
                              avec_l3, (unsigned long long)g_shunt.tick_cnt - derniere_vue);
                }
                derniere_vue = g_shunt.tick_cnt;
            }
        }
    }

    uint8_t  page = g_shunt.page_idx;
    uint16_t md   = g_shunt.d_task_md;
    uint16_t td   = g_shunt.d_task_d;

    /* Priority order: md tasks (FB/SB) > NB DL > NB UL > ALLC.
     * Refine when canned policies land. */
    if (shunt_route_c54x() && g_shunt.c54x) {
        /* CALYPSO_DSP=c54x : overlay des écritures NDB gr-gsm (rxlev/FB/SB/SI réels)
         * par-dessus le poison 0x70c4 du c54x -> le mobile campe et fait sa LU.
         * Le RUN du VRAI c54x (route_to_c54x -> c54x_run) est OPT-IN car le revival
         * est encore instable (crash qemu). Défaut : overlay seul = réception via le
         * shunt, aucun c54x exécuté, pas de crash. CALYPSO_DSP_RUN_C54X=1 pour le lancer. */
        {
            static int run_c54x = -1;
            if (run_c54x < 0) {
                const char *e = getenv("CALYPSO_DSP_RUN_C54X");
                run_c54x = (e && *e == '1') ? 1 : 0;
                fprintf(stderr, "[c54x-gate] getenv RUN_C54X=%s CRASHPC=%s DSP=%s -> run_c54x=%d\n",
                        e ? e : "(null)",
                        getenv("CALYPSO_C54X_CRASHPC") ? getenv("CALYPSO_C54X_CRASHPC") : "(null)",
                        getenv("CALYPSO_DSP") ? getenv("CALYPSO_DSP") : "(null)", run_c54x);
            }
            if (run_c54x) shunt_route_to_c54x_header(page);
        }
        if (md == PM_DSP_TASK)                          do { static int nf_pm=-1; if(nf_pm<0){const char*e=getenv("CALYPSO_SHUNT_NO_FAKE_PM");nf_pm=(e&&*e=='1')?1:0;} if(!nf_pm) shunt_dispatch_pm(page); } while(0);
        else if (md == FB_DSP_TASK) {
            /* [2026-07-22] Gate le fake FB en revive : CALYPSO_SHUNT_NO_FAKE_FB=1
             * -> skip le d_fb_det bidon (NDB only) pour laisser le VRAI correlateur
             * produire le resultat (isole le go-live : d_fb_det reste 0 si bloque). */
            static int no_fake_fb = -1;
            if (no_fake_fb < 0) { const char *e = getenv("CALYPSO_SHUNT_NO_FAKE_FB"); no_fake_fb = (e && *e == '1') ? 1 : 0; }
            /* [2026-07-26] MODE SHUNT_LEGIT : le MAC natif ne deroule pas (RANK3), mais
             * gr-gsm DETECTE reellement la SCH. On TRANSPORTE cette vraie detection vers
             * le DSP result (d_fb_det=1 + TOA/SNR reels) UNIQUEMENT quand sb_valid (=
             * gr-gsm a decode). Pas de fake : l'ARM L1 deroule alors le VRAI flux
             * FBSB->SB->BSIC->sysinfo natif, on court-circuite juste l'etage MAC. */
            static int legit = -1;
            if (legit < 0) { const char *e = getenv("CALYPSO_SHUNT_LEGIT"); const char *nl = getenv("CALYPSO_SHUNT_NO_LEGIT"); legit = ((e && *e == '1') || (nl && *nl=='1')) ? 1 : 0; }
            if (legit && !no_fake_fb) {   /* [2026-07-26] NO_FAKE_FB=1 retire le fake FB MEME en shunt_legit (isole le go-live natif) */
                if (g_shunt.sb_valid) {
                    shunt_dispatch_fb(page);
                    static unsigned _lg = 0;
                    if (_lg++ < 12)
                        SHUNT_LOG("SHUNT_LEGIT: gr-gsm detecte (sb_fn=%u bsic=%d toa=%d) "
                                  "-> d_fb_det transporte au DSP result (MAC natif court-circuite)",
                                  g_shunt.sb_fn, g_shunt.sb_bsic, (int)g_shunt.sb_toa);
                }
            } else if (!no_fake_fb) {
                shunt_dispatch_fb(page);
            }
        }
        else if (md == SB_DSP_TASK && g_shunt.sb_valid) shunt_dispatch_sb(page);
        if (td == ALLC_DSP_TASK)                        shunt_dispatch_allc(page);
    } else if (md == PM_DSP_TASK) {
        do { static int nf_pm=-1; if(nf_pm<0){const char*e=getenv("CALYPSO_SHUNT_NO_FAKE_PM");nf_pm=(e&&*e=='1')?1:0;} if(!nf_pm) shunt_dispatch_pm(page); } while(0);
    } else if (md == FB_DSP_TASK) {
        shunt_dispatch_fb(page);
    } else if (md == SB_DSP_TASK) {
        shunt_dispatch_sb(page);
    } else if (td == ALLC_DSP_TASK) {
        shunt_dispatch_allc(page);
    } else if (td != 0 && (td & 0x7FFF) != TCHT_DSP_TASK
                       && (td & 0x7FFF) != TCHA_DSP_TASK
                       && (td & 0x7FFF) != TCHD_DSP_TASK) {
        shunt_dispatch_nb(page, td);
    }

    /* ---- TCH DL : COMMUN AUX DEUX BRANCHES (2026-08-08) --------------------
     * Le dispatch TCH etait le dernier `else if` de la chaine ci-dessus, donc
     * dans la branche « pas de route c54x » UNIQUEMENT. Or shunt_route_c54x()
     * ne teste que CALYPSO_DSP=c54x — que calypso_shunt_legit.env pose, avec
     * CALYPSO_DSP_RUN_C54X=0. En profil shunt_legit c'est donc TOUJOURS la
     * premiere branche qui s'execute, et shunt_dispatch_tch_dl n'a JAMAIS ete
     * appele : du code mort dans le seul profil qui devait s'en servir. Le
     * symptome cote mobile etait un canal dedie parfaitement muet
     * (rxlev-full=-110, meas-invalid=1) — indiscernable d'un probleme de RF.
     * On sort donc le TCH de la chaine : il ne depend pas de la route c54x. */
    switch (td & 0x7FFF) {
    case TCHT_DSP_TASK: shunt_dispatch_tch_dl(page);    break;   /* voix + FACCH */
    case TCHA_DSP_TASK: shunt_dispatch_tch_sacch(page); break;   /* SACCH dediee */
    default: break;
    }
    /* RA UL (d_task_ra) handled separately — TBD when TX flow gated */

    /* Mock task done. Real DSP would keep its state for multi-attempt
     * tasks (FB search across 11 frames). Phase 1 canned can keep the
     * pending bit set for FB until d_fb_det is consumed (zeroed by ARM
     * in read_fb_result @ prim_fbsb.c:318). */
    g_shunt.pending = false;
}

/* ---- MMIO overlay on NDB+0 (d_dsp_page trigger) ---- */
static void shunt_d_dsp_page_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    /* [2026-07-30] CORRECTIF — l'ancien commentaire ici disait : « Write also
     * commits the value in the underlying RAM region [...] this overlay is
     * registered with higher priority but pass-through semantics ». C'ETAIT
     * FAUX, et ca a coute une semaine.
     *
     * Dans QEMU, memory_region_add_subregion_overlap ne superpose pas : la
     * region de plus haute priorite qui couvre l'adresse traite l'acces
     * EXCLUSIVEMENT. La region du dessous (calypso.dsp_api, priorite 0) ne le
     * voit jamais. Cet overlay fait 2 octets et couvre exactement 0xFFD001A8 =
     * d_dsp_page. Consequence mesuree le 30/07 :
     *   - calypso_dsp_write() n'etait JAMAIS appele pour ce mot -> aucune sonde
     *     ne pouvait le voir (DDP-ANY, ungated depuis le 22/07 : 0 tir sur 17
     *     journaux ; WR-OP ; moniteur mailbox ; DPAGE_HUNT) ;
     *   - la valeur n'etait stockee NULLE PART -> la cellule gardait son dechet
     *     de boot 0xf600, dont le bit1 (B_GSM_TASK) est 0 : le DSP s'entendait
     *     dire « aucune tache GSM » a chaque trame, et bit0=0 lui faisait
     *     latcher la page 0 a vie (il ne relit d_dsp_page qu'UNE fois par run) ;
     *   - shunt_d_dsp_page_read ci-dessous « passait a la RAM » et retournait
     *     donc consciencieusement ce 0xf600.
     * Le bouchon data[0x43d8]=0xab38 (=RET) etait une CONSEQUENCE de ca.
     *
     * On commite donc reellement la valeur, dans les DEUX banques (dsp_ram que
     * l'ARM relit et qui est la source du miroir par tick de calypso_trx.c, et
     * data[] que le DSP lit) : n'ecrire que data[] serait ecrase au tick
     * suivant par ce miroir. Le commit passe par un helper direct, PAS par
     * l'espace d'adressage : un shunt_write_w ici retomberait sur cet overlay
     * -> recursion. */
    shunt_latch_task((uint16_t)value);
    calypso_trx_api_commit_w(BASE_API_NDB + NDB_D_DSP_PAGE - 0xFFD00000UL,
                             (uint16_t)value);
}

static uint64_t shunt_d_dsp_page_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    /* Read passes through to RAM — ARM polls this for handshake state.
     * We return the actual RAM value to be transparent. */
    return shunt_read_w(BASE_API_NDB + NDB_D_DSP_PAGE);
}

static const MemoryRegionOps shunt_ndb_trigger_ops = {
    .read  = shunt_d_dsp_page_read,
    .write = shunt_d_dsp_page_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl  = { .min_access_size = 2, .max_access_size = 2 },
};

/* ---- GSMTAP listener : reçoit le SI décodé par gr-gsm (front py) ----
 * gr-gsm (grgsm_decode -m BCCH) sort des frames GSMTAP. On écoute sur un port
 * UDP (CALYPSO_SHUNT_GSMTAP_PORT, défaut 4730 pour ne pas taper le 4729 du
 * BTS), on extrait le L2 (après le hdr GSMTAP de 16 o) des frames BCCH, et on
 * appelle feed_si → a_cd. = le pont gr-gsm→a_cd, côté qemu. */
#define GSMTAP_HDR_LEN          16
#define GSMTAP_TYPE_UM          0x01
#define GSMTAP_CHANNEL_BCCH     0x01
#define GSMTAP_CHANNEL_AGCH     0x04   /* IMM ASSIGN (PORTE 3a / #11) */
#define GSMTAP_CHANNEL_SDCCH4   0x07   /* SDCCH/4 SS0 DL (UA/AUTH / #2) */
#define GSMTAP_CHANNEL_ACCH     0x80   /* bit ACCH (SACCH) GSMTAP */
#define GSMTAP_CHANNEL_SACCH    (GSMTAP_CHANNEL_SDCCH4 | GSMTAP_CHANNEL_ACCH) /* 0x87 : SI5/SI6 reels */
#define GSMTAP_CHANNEL_TCH_F    0x08   /* FACCH/F du canal dedie (2026-08-08) */
#define GSMTAP_CHANNEL_TCH_ACCH (GSMTAP_CHANNEL_TCH_F | GSMTAP_CHANNEL_ACCH)  /* 0x88 : SACCH du TCH */
static int g_gsmtap_fd = -1;

/* AGCH (#11) : range l'IMM ASSIGN forwardé par si_bridge (tag GSMTAP AGCH 0x04)
 * dans agch_buf — DISTINCT de si_buf, pour que la rotation des SI ne l'écrase pas.
 * shunt_dispatch_allc le présentera dans a_cd sur un bloc CCCH (le firmware tague
 * alors chan_nr=0x90 -> gsm48_rr_rx_pch_agch -> gsm48_rr_rx_imm_ass). */
static void calypso_dsp_shunt_feed_agch(const uint8_t *l2, int len)
{
    /* [2026-08-09] BLOC D APPRENTISSAGE DE LA SOUS-VOIE : SUPPRIME.
     *
     * Il lisait le chan_nr des IMM ASSIGN passant sur le CCCH pour poser la
     * fenetre de presentation, en pretendant ne retenir que la NOTRE via
     * l2[7] == g_last_recorded_ra. Ce filtre ne filtrait rien : 5 acceptations
     * sur 5 avec ra=0x07 quand le mobile emettait 0x0c/0x04/0x03, puis 0 sur 25.
     * Deux mobiles partagent ce CCCH, donc il apprenait les assignations de
     * l autre -- l objection exacte que le commentaire l.155 avait deja actee.
     *
     * PIRE : il a fabrique le symptome qu il devait expliquer. Les fenetres
     * « SDCCH/8 SS=0,1,2 -> fn%51 0-3 » attribuees au firmware venaient de LUI,
     * seul second appelant de set_dcch(). Avec le bloc inerte, la mesure donne
     * chan_nr=0x28 -> SDCCH/4 SS=1 -> fenetre 26-29 : le decodage etait juste
     * depuis le debut (offset confirme par le DWARF de l ELF charge).
     * Et le LU echoue a l identique fenetres justes : cette piste n etait ni
     * necessaire ni suffisante.
     *
     * set_dcch() retrouve donc la source unique que son propre commentaire
     * revendique : le chan_nr annonce par le firmware, via l1ctl_sock.
     *
     * ⚠️ DEFAUT LATENT MIS AU JOUR AU PASSAGE, non corrige ici : une IMM ASSIGN
     * etrangere basculait g_shunt.sdcch_ch8 a true, et calypso_dsp_shunt_l1_reset()
     * ne remet PAS ce drapeau a false (seulement sdcch_ss_set). La fenetre-union
     * de calypso_dsp_helper.c:629 restait alors collee a [0,31] a travers tout
     * reset L1 -- excluant 32-39 (SDCCH/4 SS2 et SS3) et acceptant a tort les
     * blocs CCCH 0-21. Supprimer l apprentissage retire la seule facon connue
     * d armer ce piege, mais le drapeau reste non reinitialise. */
    if (!l2 || len < 3) return;

    /* Priorite IMM ASSIGN : ne pas laisser un PAGING REQUEST (mt 0x21/0x22/0x24)
     * ecraser un IMM ASSIGN (0x3f/0x3a/0x3b) encore valide en attente de
     * presentation sur l'AGCH. L'IMM ASSIGN est la reponse time-critical au RACH
     * (un seul agch_buf partage) ; le paging est best-effort. Sur un reseau a
     * paging dense le flood clobberait sinon le grant -> RACH en boucle, pas de LU. */
    {
        uint8_t in_mt = l2[2];
        int in_is_imm = (in_mt == 0x3f || in_mt == 0x3a || in_mt == 0x3b);
        uint8_t cur_mt = g_shunt.agch_buf[2];
        int cur_is_imm = (cur_mt == 0x3f || cur_mt == 0x3a || cur_mt == 0x3b);
        if (!in_is_imm && g_shunt.agch_valid && cur_is_imm) {
            static int ttl = -1;
            if (ttl < 0) { const char *t = getenv("CALYPSO_SHUNT_AGCH_TTL");
                           ttl = (t && *t) ? atoi(t) : 100; }
            if ((uint32_t)(g_shunt.tick_cnt - g_shunt.agch_tick) <= (uint32_t)ttl) {
                static unsigned drop = 0;
                if (drop++ < 20 || (drop % 200) == 0)
                    SHUNT_LOG("feed_agch: PAGING mt=0x%02x DROP "
                            "(IMM ASSIGN 0x%02x encore valide en attente)\n", in_mt, cur_mt);
                return;
            }
        }
    }

    int n = len < 23 ? len : 23;
    memcpy(g_shunt.agch_buf, l2, n);
    for (int i = n; i < 23; i++) g_shunt.agch_buf[i] = 0x2B;
    /* [2026-07-27] SONDE sous-voie SDCCH de l IMM-ASS (channel desc [4..6]) :
     * confirme le mismatch SS0-hardcode vs sous-voie assignee (cause rejet SMS flaky). */
    if (g_shunt.agch_buf[2] == 0x3f) {
        uint8_t cd0 = g_shunt.agch_buf[4];
        /* [2026-07-27] Base DL fn%%51 de la voie dediee assignee (GSM 05.02, cf
         * firmware mframe_sched.c). chan_desc.chan_nr = cd0. Le RESEAU ICI assigne
         * du SDCCH/8 (01SSS, DL base=SS*4) et parfois SDCCH/4 (001SS). On calcule
         * la base DL et on la memorise pour presenter l'UA sur la BONNE fenetre. */
        int _ss = -1, _base = -1; const char *_ct = "?";
        if ((cd0 & 0xE0) == 0x20) {            /* SDCCH/4 combined : 001SS */
            _ss = (cd0 >> 3) & 0x03; _ct = "SDCCH/4";
            static const int b4[4] = { 22, 26, 32, 36 };
            _base = b4[_ss];
        } else if ((cd0 & 0xC0) == 0x40) {     /* SDCCH/8 : 01SSS, DL base = SS*4 */
            _ss = (cd0 >> 3) & 0x07; _ct = "SDCCH/8";
            _base = _ss * 4;                    /* 0,4,8,12,16,20,24,28 (mod 51) */
        }
        /* [2026-08-08] CE CHEMIN N'ECRIT PLUS LA FENETRE. Il la posait depuis
         * N'IMPORTE QUEL IMM ASSIGN du CCCH — donc aussi ceux des autres
         * abonnes (mesure : 68 de RA=0x07 et 12 de RA=0x0a pour un RACH a nous
         * de RA=0x08). Ajouter la source autoritaire sans retirer celle-ci ne
         * suffisait pas : les deux ecrivains se marchaient dessus, et le test
         * anti-repetition du setter le faisait sortir en silence quand feed_agch
         * avait deja pose la valeur (constate au run 18:31 : DCCH #2 SS=1 detecte,
         * aucune DCCH-WINDOW correspondante). La fenetre vient desormais
         * UNIQUEMENT de calypso_dsp_shunt_set_dcch(), appelee sur le chan_nr que
         * le firmware annonce lui-meme. On garde la sonde ci-dessous, qui reste
         * utile pour voir ce que le reseau distribue. */
        static unsigned _iac = 0;
        if (_iac++ < 80) {
            fprintf(stderr, "[dsp-shunt] IMM-ASS chan_desc=[%02x %02x %02x] %s SS=%d base=%d TN=%u req-ref=[%02x %02x %02x]\n",
                    g_shunt.agch_buf[4], g_shunt.agch_buf[5], g_shunt.agch_buf[6],
                    _ct, _ss, _base, cd0 & 0x07,
                    g_shunt.agch_buf[7], g_shunt.agch_buf[8], g_shunt.agch_buf[9]);
        }
    }

    /* FN-FIX (le vrai fix, ON par defaut ; CALYPSO_REQREF_REWRITE=0 pour A/B) :
     * reecrit la request-reference de l'IMM ASSIGN (octets L2 [8],[9]) au FN EXACT que
     * le firmware a memorise pour la derniere RACH = last_rach.fn (@0x836500). RAISON :
     * le FN de la req-ref vit dans l'horloge osmo-trx (sample-position, base ~2465144),
     * le mobile le compare a SA propre horloge L1 (la valeur recue en L1CTL_RACH_CONF =
     * last_rach.fn, prim_rach.c:114) ; ces deux compteurs free-running ont une phase de
     * depart non controlee et variable par-RACH -> mismatch gsm48_rr.c:3382. Aucun
     * cal_off/ul_fnoff/fn_adj cote device ne peut les aligner.
     * On lit donc DIRECTEMENT last_rach.fn (la valeur que le mobile a memorisee, par
     * construction) au lieu de g_rach_l1s_fn[ra]+adj : ce dernier capturait current_time
     * au tick d_rach/cmd, soit -4 frames AVANT que le firmware pose last_rach.fn =
     * current_time-1 au tick rach_resp -> skew variable (constate : adj devait passer de
     * -1 a +1 entre deux runs). last_rach.fn n'a aucun skew ni collision RA (1 seule RACH
     * en vol cote mobile). Le check du mobile (gsm48_rr.c:3372) est PUREMENT local : il
     * matche (ra,T1,T2,T3) contre son propre cr_hist ; le FN est informationnel. Donc
     * req-ref := last_rach.fn => match exact, sans constante FN magique ni adj.
     * RA = L2[7] (log seulement). Encodage req-ref (04.08) :
     *   [8] = (T1'<<3) | (T3>>3) ; [9] = ((T3&7)<<5) | T2 ; T1'=(FN/1326)%32.
     * adj=0 par defaut (last_rach.fn EST le memo) ; surchargeable CALYPSO_REQREF_ADJ. */
    {
        /* @BEQUILLE — REQREF_LAST_RACH / REQREF_PERRA / REQREF_REWRITE / REQREF_ADJ
         *   masque  : la coherence FN entre l'horloge L1 du mobile et la req-ref emise par
         *             la BTS. On REECRIT agch_buf[8..9] (T1'/T2/T3) de l'IMM-ASSIGN recu
         *             pour qu'il matche last_rach.fn lu dans la RAM ARM — c'est-a-dire
         *             qu'on falsifie le message reseau pour compenser un skew local.
         *   retirer : quand shunt_l1s_fn() et calypso_trx_get_fn() sont alignees sur la
         *             SCH sans recale residuel (RANK4) — la req-ref native matche alors.
         */
        static int reqref_rw = -1, reqref_perra = -1, rr_adj = -99999;
        if (reqref_rw < 0)    { const char *e = getenv("CALYPSO_REQREF_REWRITE"); reqref_rw = (e && *e == '1') ? 1 : 0; }  /* defaut OFF : ancien rewrite GLOBAL (50% multi-RACH) */
        if (reqref_perra < 0) { const char *e = getenv("CALYPSO_REQREF_PERRA");   reqref_perra = (e && *e == '0') ? 0 : 1; } /* defaut ON : req-ref PER-RA (FN exact du RACH_CONF keye par ra) */
        if (rr_adj == -99999) { const char *e = getenv("CALYPSO_REQREF_ADJ");     rr_adj = e ? atoi(e) : 0; }
        if ((reqref_perra || reqref_rw) && n >= 10 && g_shunt.agch_buf[2] == 0x3f) {
            uint8_t ra = g_shunt.agch_buf[7];
            /* [2026-07-27] FIX SMS (mt-sms-works) : prefere last_rach.fn@0x836500
             * (FN EXACT memorise par le firmware = match req-ref garanti), defaut ON.
             * Fallback auto sur g_rach_conf_fn[ra] si lecture=0. Gate CALYPSO_REQREF_LAST_RACH=0. */
            static int use_lr = -1;
            if (use_lr < 0) { const char *e = getenv("CALYPSO_REQREF_LAST_RACH"); use_lr = (e && *e == '0') ? 0 : 1; }
            uint32_t lr = use_lr ? shunt_last_rach_fn() : 0;
            uint32_t memo_fn = lr ? lr
                             : (reqref_perra && g_rach_conf_fn[ra]) ? g_rach_conf_fn[ra]
                             : (reqref_rw ? g_last_rach_conf_fn : 0);   /* last_rach exact, sinon per-ra, sinon global */
            { static unsigned dbg = 0;
              if (dbg++ < 40)
                  SHUNT_LOG("FN-FIX probe RA=0x%02x "
                          "memo_fn(RACH_CONF)=%u last_rach@500=%u l1s_fn=%u n=%d\n",
                          ra, memo_fn, shunt_last_rach_fn(), shunt_l1s_fn(), n); }
            if (memo_fn) {
                int64_t fn = (int64_t)memo_fn + rr_adj;
                if (fn < 0) fn = 0;
                uint16_t t1p = (uint16_t)(((uint32_t)fn / 1326u) % 32u);
                uint8_t  t2  = (uint8_t)((uint32_t)fn % 26u);
                uint8_t  t3  = (uint8_t)((uint32_t)fn % 51u);
                g_shunt.agch_buf[8] = (uint8_t)((t1p << 3) | ((t3 >> 3) & 7));
                g_shunt.agch_buf[9] = (uint8_t)(((t3 & 7) << 5) | (t2 & 0x1f));
                static unsigned rwlog = 0;
                if (rwlog++ < 30)
                    SHUNT_LOG("FN-FIX req-ref RA=0x%02x reecrite -> "
                            "fn=%u (T1'=%u T2=%u T3=%u) adj=%d [last_rach.fn]\n",
                            ra, (uint32_t)fn, t1p, t2, t3, rr_adj);
            }
        }
    }

    g_shunt.agch_valid = true;
    g_shunt.agch_tick  = g_shunt.tick_cnt;
    SHUNT_LOG("feed_agch: IMM-ASS mt=0x%02x -> agch_buf "
            "(a presenter sur bloc CCCH)\n", l2[2]);
}

/* SDCCH/4 SS0 DL (#2) : range le bloc L2 (UA/AUTH) forwarde par si_bridge
 * (tag GSMTAP SDCCH4 0x07) dans sdcch_buf -- DISTINCT de si_buf/agch_buf.
 * shunt_dispatch_allc le presentera dans a_cd sur le bloc SDCCH/4 SS0
 * (fn%51 in {22-25}) ; le firmware tague alors chan_nr=0x20 -> lapdm_dcch ->
 * UA/AUTH -> L3 (miroir de feed_agch, SANS la sonde req-ref). */
static void calypso_dsp_shunt_feed_sdcch(const uint8_t *l2, int len, uint32_t fn)
{
    /* @BEQUILLE — INJECT_SDCCH  (CALYPSO_INJECT_SDCCH=1, fallback CALYPSO_SHUNT_LEGIT=1)
     *   masque  : la demodulation du SDCCH DL par le DSP (UA/AUTH/L3), remplacee par
     *             les blocs L2 forwardes par si_bridge.
     *   retirer : quand le chemin natif demodule le SDCCH.
     */
    { static int _ginj = -1; if (_ginj < 0) { const char *_e = getenv("CALYPSO_INJECT_SDCCH"); _ginj = (_e && *_e == '1') ? 1 : 0; if (!_ginj) { const char *_l = getenv("CALYPSO_SHUNT_LEGIT"); _ginj = (_l && *_l == '1') ? 1 : 0; } } if (!_ginj) return; }
    if (!l2 || len < 3) return;
    static int ring_on = -1;
    if (ring_on < 0) { const char *e = getenv("CALYPSO_SHUNT_SDCCH_RING"); ring_on = (!e || *e != '0') ? 1 : 0; }
    int n = len < 23 ? len : 23;
    if (!ring_on) {
        memcpy(g_shunt.sdcch_buf, l2, n);
        for (int i = n; i < 23; i++) g_shunt.sdcch_buf[i] = 0x2B;
        g_shunt.sdcch_valid = true; g_shunt.sdcch_tick = g_shunt.tick_cnt;
        return;
    }
    if (fn && fn == g_shunt.sdcch_last_fn) return;
    g_shunt.sdcch_last_fn = fn;
    if (g_shunt.sdcch_ring_tail - g_shunt.sdcch_ring_head >= SDCCH_RING_N) {
        /* [2026-07-27] eviction d overflow TRACEE (etait silencieuse) : quand le
         * ring sature, on drope le plus vieux -> perte de bloc DL. Log + compteur
         * pour diagnostiquer un SMS/LU intermittent sans deviner. */
        g_shunt.evict_overflow++;
        static unsigned n_ovf = 0;
        if (n_ovf++ < 40 || (n_ovf % 100) == 0)
            SHUNT_LOG("feed_sdcch: RING OVERFLOW #%u -> drop head fn=%u c=0x%02x (depth=%u/%u)\n",
                    n_ovf, g_shunt.sdcch_ring[g_shunt.sdcch_ring_head % SDCCH_RING_N].fn,
                    g_shunt.sdcch_ring[g_shunt.sdcch_ring_head % SDCCH_RING_N].l2[1],
                    g_shunt.sdcch_ring_tail - g_shunt.sdcch_ring_head, SDCCH_RING_N);
        g_shunt.sdcch_ring[g_shunt.sdcch_ring_head % SDCCH_RING_N].used = false;
        g_shunt.sdcch_ring_head++;
    }
    uint32_t idx = g_shunt.sdcch_ring_tail % SDCCH_RING_N;
    memcpy(g_shunt.sdcch_ring[idx].l2, l2, n);
    for (int i = n; i < 23; i++) g_shunt.sdcch_ring[idx].l2[i] = 0x2B;
    g_shunt.sdcch_ring[idx].fn = fn; g_shunt.sdcch_ring[idx].tick = g_shunt.tick_cnt;
    g_shunt.sdcch_ring[idx].reps = 0; g_shunt.sdcch_ring[idx].used = true; g_shunt.sdcch_ring_tail++;
    g_shunt.sdcch_valid = true;
    SHUNT_LOG("feed_sdcch: ENQUEUE fn=%u c=0x%02x [depth=%u]\n", fn, l2[1], g_shunt.sdcch_ring_tail - g_shunt.sdcch_ring_head);
}

/* SACCH SS0 DL REELLE : SI5(0x1d)/SI6(0x1e) decodes par grgsm, forwardes par
 * si_bridge (sub_type 0x87). Le bloc grgsm = 23o : [L1 hdr 2][LAPDm: 03 03 len
 * 06 mt L3...] -> exactement le layout B4 attendu par le dispatch SACCH. On
 * garde la L3 REELLE mais on ZERO le header L1 (tx_power/TA) : les valeurs
 * osmo-bts ne sont pas pour notre air emule (idem fabrication). sacch_real=true
 * fait CESSER la fabrication SI3->SI6 (sinon SI3 du BCCH clobbe le SI5/SI6 reel). */
static void calypso_dsp_shunt_feed_sacch(const uint8_t *l2, int len)
{
    /* @BEQUILLE — INJECT_SACCH  (CALYPSO_INJECT_SACCH=1, fallback CALYPSO_SHUNT_LEGIT=1)
     *   masque  : la demodulation SACCH par le DSP (SI5/SI6 du canal dedie),
     *             remplacee par les blocs gr-gsm avec header L1 zerote.
     *   retirer : quand le chemin natif demodule la SACCH.
     */
    { static int _ginj = -1; if (_ginj < 0) { const char *_e = getenv("CALYPSO_INJECT_SACCH"); _ginj = (_e && *_e == '1') ? 1 : 0; if (!_ginj) { const char *_l = getenv("CALYPSO_SHUNT_LEGIT"); _ginj = (_l && *_l == '1') ? 1 : 0; } } if (!_ginj) return; }  /* [2026-07-23] HACK injection sortie, DEFAUT OFF (natif) ; =CALYPSO_INJECT_SACCH=1 pour reactiver */
    if (!l2 || len < 7) return;
    int n = len < 23 ? len : 23;
    /* trouve le RR header (06 1d / 06 1e) pour valider que c'est bien SI5/SI6 */
    int rr = -1;
    for (int i = 2; i + 1 < n && i < 8; i++)
        if (l2[i] == 0x06 && (l2[i + 1] == 0x1d || l2[i + 1] == 0x1e)) { rr = i; break; }
    if (rr < 0) return;                       /* pas un SI5/SI6 -> ignore */
    uint8_t *s = g_shunt.sacch_buf;
    memcpy(s, l2, n);
    for (int i = n; i < 23; i++) s[i] = 0x2b;
    /* [2026-08-09] ON N ECRASE PLUS LES DEUX PREMIERS OCTETS.
     *
     * L ANCIEN CODE les forcait a zero, en les croyant systematiquement l en-tete
     * L1 SACCH (puissance ordonnee / avance de temps). Mais le balayage juste
     * au-dessus, « for (i = 2; i < 8) » a la recherche de 06 1d / 06 1e, dit le
     * contraire : la position de l en-tete RR VARIE, donc le bloc rendu par gr-gsm
     * ne porte pas toujours ses deux octets L1. Quand il ne les porte pas, l2[0] et
     * l2[1] sont l ADRESSE et le CONTROLE LAPDm -- et on les remplacait par 0x00.
     *
     * CONSEQUENCE MESUREE (09/08). Le LAPDm du mobile ne reconnait alors plus
     * l en-tete, retombe sur le format Bter et rend 21 octets de L3 au lieu de 19.
     * Or gsm48_rr_rx_acch() discrimine par la seule LONGUEUR :
     *   l3len == 21 (N201_Bter_SACCH) -> en-tete court, il lit le 1er octet comme
     *                                    un type de message -> « New SYSTEM
     *                                    INFORMATION 10 » et « SI10: BA_IND 0 !=
     *                                    BA_IND 1 of SI5! »
     *   l3len == 19 (N201_B4)         -> SI5 / SI6, ce qu on veut
     * Comptage sur un run : 30 SI10 pour 3 SI5 et 1 SI6.
     *
     * LA VOIE TCH NE FAISAIT DEJA PAS CETTE ERREUR : feed_tch_sacch laisse les deux
     * octets intacts et le commentaire y explique pourquoi -- le mobile APPLIQUE
     * ces valeurs (« DL SACCH indicates ta / tx_power »), et un TA force a 0 sur un
     * lien qui en demande 4 desaligne l emission montante. La meme raison vaut ici.
     * On aligne donc les deux voies sur celle qui fonctionne : on ne touche a rien. */
    g_shunt.sacch_have = true;
    g_shunt.sacch_real = true;                /* coupe la fabrication SI3->SI6 */
    static unsigned nf = 0;
    if (nf++ < 20 || (nf % 50) == 0)
        SHUNT_LOG("feed_sacch REEL: SI%d %do (mt=0x%02x) -> sacch_buf\n",
                (l2[rr + 1] == 0x1d) ? 5 : 6, n, l2[rr + 1]);
}

/* ---- TCH DL de signalisation : FACCH et SACCH-du-dedie (2026-08-08) --------
 * Source = le grgsm TCHF lance par si_bridge sur le timeslot assigne, achemine
 * en GSMTAP 4730 (sous-types 0x08 / 0x88). Aucune fabrication ici : si gr-gsm
 * ne decode pas, rien n'est presente et le mobile le voit — c'est le point de
 * la doctrine shunt_legit (le decodeur hote tient le role du DSP, il ne l'imite
 * pas). Gate commun aux deux : CALYPSO_INJECT_TCH, sinon SHUNT_LEGIT. */
static bool shunt_tch_inject_on(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("CALYPSO_INJECT_TCH");
        v = (e && *e == '1') ? 1 : 0;
        if (!v) { const char *l = getenv("CALYPSO_SHUNT_LEGIT"); v = (l && *l == '1') ? 1 : 0; }
    }
    return v != 0;
}

static void calypso_dsp_shunt_feed_facch(const uint8_t *l2, int len)
{
    if (!shunt_tch_inject_on() || !l2 || len < 3) return;
    int n = len < 23 ? len : 23;
    memcpy(g_shunt.facch_dl, l2, n);
    for (int i = n; i < 23; i++) g_shunt.facch_dl[i] = 0x2B;
    g_shunt.facch_dl_valid = true;
    g_shunt.facch_dl_tick  = g_shunt.tick_cnt;
    static unsigned nf = 0;
    if (nf++ < 40 || (nf % 25) == 0)
        SHUNT_LOG("feed_facch: %do -> a_fd (%02x %02x %02x %02x) #%u\n",
                  n, l2[0], l2[1], l2[2], (n > 3) ? l2[3] : 0, nf);
}

static void calypso_dsp_shunt_feed_tch_sacch(const uint8_t *l2, int len)
{
    if (!shunt_tch_inject_on() || !l2 || len < 3) return;
    int n = len < 23 ? len : 23;
    uint8_t *s = g_shunt.tsacch_dl;
    memcpy(s, l2, n);
    for (int i = n; i < 23; i++) s[i] = 0x2B;
    /* En-tete L1 SACCH (2 o : ordre de puissance / avance de temps). gr-gsm rend
     * le bloc tel qu'il est passe sur l'air ; on le laisse INTACT, contrairement
     * a feed_sacch qui le zerote. Raison : sur canal dedie le mobile APPLIQUE ces
     * deux octets (gsm48_rr.c « DL SACCH indicates ta / tx_power »), et un TA
     * force a 0 sur un lien qui en demande 4 desaligne l'emission montante. */
    g_shunt.tsacch_dl_valid = true;
    g_shunt.tsacch_dl_tick  = g_shunt.tick_cnt;
    static unsigned nf = 0;
    if (nf++ < 40 || (nf % 25) == 0)
        SHUNT_LOG("feed_tch_sacch: %do -> a_cd (pwr=%02x ta=%02x, %02x %02x %02x) #%u\n",
                  n, s[0], s[1], s[2], s[3], s[4], nf);
}

/* Config du canal dedie, publiee par si_bridge apres decodage de l'ASSIGNMENT
 * COMMAND : /dev/shm/calypso_tch_cfg, 16 o = seq@0(u32) tn@4 tsc@5 arfcn@6(u16)
 * chan_nr@8. seq==0 -> pas de TCH en cours. Le shunt s'en sert pour le journal
 * et pour savoir qu'un dedie TCH est arme ; qemu_wrap s'en sert pour le slot UL. */
static void shunt_poll_tch_cfg(void)
{
    static int fd = -2;
    if (fd == -2)
        fd = open("/dev/shm/calypso_tch_cfg", O_CREAT | O_RDWR, 0644);
    if (fd < 0) return;
    uint8_t b[16];
    if (pread(fd, b, sizeof(b), 0) != (ssize_t)sizeof(b)) return;
    uint32_t seq; memcpy(&seq, b, 4);
    static uint32_t last = 0;
    if (seq == 0) {
        if (g_shunt.tch_cfg_valid) {
            g_shunt.tch_cfg_valid = false;
            /* canal libere : ne PAS re-presenter la signalisation du dedie precedent */
            g_shunt.facch_dl_valid = g_shunt.tsacch_dl_valid = false;
            SHUNT_LOG("TCH-CFG: canal dedie libere (seq=0)\n");
        }
        last = 0;
        return;
    }
    if (seq == last) return;
    last = seq;
    g_shunt.tch_tn  = b[4];
    g_shunt.tch_tsc = b[5];
    memcpy(&g_shunt.tch_arfcn, b + 6, 2);
    g_shunt.tch_cfg_valid = true;
    /* nouveau canal : la signalisation de l'ancien n'a plus cours */
    g_shunt.facch_dl_valid = g_shunt.tsacch_dl_valid = false;
    SHUNT_LOG("TCH-CFG #%u: TN=%u TSC=%u ARFCN=%u chan_nr=0x%02x\n",
              seq, g_shunt.tch_tn, g_shunt.tch_tsc, g_shunt.tch_arfcn, b[8]);
}

static void shunt_gsmtap_read(void *opaque)
{
    uint8_t buf[512];
    for (;;) {
        ssize_t n = recv(g_gsmtap_fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            break;
        }
        if (n < GSMTAP_HDR_LEN + 1)
            continue;
        uint8_t type     = buf[2];   /* GSMTAP hdr : type @2 */
        uint8_t sub_type = buf[12];  /* channel @12 */
        if (type != GSMTAP_TYPE_UM)
            continue;
        /* L2 = buf+16 : [0]=pseudo-len, [1]=PD, [2]=message type.
         * RR PD (0x06) requis pour BCCH/AGCH (SI/IMM-ASS) ; le SDCCH/4 (#2)
         * porte de la LAPDm (buf[17]=controle, PAS un PD RR) -> on n'exige
         * PAS 0x06 pour sub_type 0x07. */
        if (n < GSMTAP_HDR_LEN + 3)
            continue;
        /* [2026-08-08] Le TCH est exempte du filtre « PD RR » au meme titre que le
         * SDCCH/4 : FACCH et SACCH du dedie portent de la LAPDm (buf[17] = controle),
         * pas un PD RR. Les exiger a 0x06 jetterait tout le trafic de l'appel. */
        if (sub_type != GSMTAP_CHANNEL_SDCCH4 && sub_type != GSMTAP_CHANNEL_SACCH &&
            sub_type != GSMTAP_CHANNEL_TCH_F  && sub_type != GSMTAP_CHANNEL_TCH_ACCH &&
            buf[GSMTAP_HDR_LEN + 1] != 0x06)
            continue;                               /* pas RR PD (BCCH/AGCH) */
        uint8_t mt = buf[GSMTAP_HDR_LEN + 2];
        if (sub_type == GSMTAP_CHANNEL_BCCH) {
            /* (A) SET BCCH COMPLET (SI1/2/3/4/2bis/2ter), pas juste SI3 — sinon le
             * mobile n'a jamais le set complet ("No sysinfo yet"). feed_si range
             * chaque type dans son slot, dispatch_allc tourne dessus. */
            switch (mt) {
            case 0x19: case 0x1a: case 0x1b:        /* SI1 SI2 SI3 */
            case 0x1c: case 0x1d: case 0x1e:        /* SI4 SI2bis SI2ter */
                break;
            default:
                continue;                           /* paging/SI13... sur BCCH : drop */
            }
            calypso_dsp_shunt_feed_si(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN);
        } else if (sub_type == GSMTAP_CHANNEL_AGCH) {
            /* (#11) IMM ASSIGN / EXT / REJ + (#SMS) PAGING REQ 1/2/3 -> agch_buf
             * (presente sur bloc CCCH -> firmware chan_nr=0x90 -> gsm48_rr_rx_pch_agch
             * qui dispatch IMM-ASS vs PAGING par msg type). 0x21=PAG_REQ_1 (pas SI13). */
            if (mt == 0x3f || mt == 0x39 || mt == 0x3a ||
                mt == 0x21 || mt == 0x22 || mt == 0x24)
                calypso_dsp_shunt_feed_agch(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN);
        } else if (sub_type == GSMTAP_CHANNEL_SDCCH4) {
            /* (#2) SDCCH/4 SS0 DL (UA/AUTH) -> sdcch_buf (presente sur le bloc
             * SDCCH/4 SS0, fn%51 in {22-25}). LAPDm : aucun filtre message-type
             * (le gate canal = le FN cote si_bridge + le dispatch). */
            uint32_t sd_fn = ((uint32_t)buf[8] << 24) | ((uint32_t)buf[9] << 16) | ((uint32_t)buf[10] << 8) | (uint32_t)buf[11];
            calypso_dsp_shunt_feed_sdcch(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN, sd_fn);
        } else if (sub_type == GSMTAP_CHANNEL_SACCH) {
            /* SACCH SS0 DL : SI5/SI6 REELS (si_bridge fn%51 {42-45}) -> sacch_buf
             * REEL (presente fn%51 {42-45}). Remplace la fabrication SI3->SI6. */
            calypso_dsp_shunt_feed_sacch(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN);
        } else if (sub_type == GSMTAP_CHANNEL_TCH_F) {
            /* FACCH du canal dedie -> a_fd. C'est la que passent ASSIGNMENT
             * COMPLETE (montant, cf shunt_capture_tch_ul) et, en descendant,
             * ALERTING / CONNECT / DISCONNECT de l'appel. */
            calypso_dsp_shunt_feed_facch(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN);
        } else if (sub_type == GSMTAP_CHANNEL_TCH_ACCH) {
            /* SACCH du canal dedie -> a_cd. Sans elle le mobile decompte les
             * blocs SACCH manquants et declare une defaillance de lien radio. */
            calypso_dsp_shunt_feed_tch_sacch(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN);
        }
        /* autres canaux : drop */
    }
}

static void shunt_gsmtap_init(void)
{
    if (shunt_grgsm_off()) {
        SHUNT_ERR("gr-gsm COUPE : listener GSMTAP/SI :4730 NON arme "
                  "-> si_buf (BCCH/SI) doit venir du DSP");
        return;
    }
    const char *p = getenv("CALYPSO_SHUNT_GSMTAP_PORT");
    int port = (p && *p) ? atoi(p) : 4730;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        SHUNT_ERR("GSMTAP socket() failed: %s", strerror(errno));
        return;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons((uint16_t)port);
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        SHUNT_ERR("GSMTAP bind(:%d) failed: %s", port, strerror(errno));
        close(fd);
        return;
    }
    g_gsmtap_fd = fd;
    qemu_set_fd_handler(fd, shunt_gsmtap_read, NULL, NULL);
    SHUNT_ERR("GSMTAP listener udp:127.0.0.1:%d → feed_si(a_cd) "
                 "(gr-gsm grgsm_decode -m BCCH y envoie le SI réel)", port);
}

/* ---- SCH listener : recoit le BSIC/FN REELS decodes par gr-gsm (= le DSP) ----
 * grgsm_relay_decode.py forwarde le tuple ('sch',bsic,fn) du port `measurements`
 * de gsm.receiver en UDP {magic 'SCH1', int32 bsic, int32 fn, LE} sur ce port
 * (CALYPSO_SHUNT_SCH_PORT, defaut 4731 — distinct du GSMTAP 4730). On stocke le
 * resultat -> shunt_dispatch_sb encode le VRAI BSIC/FN au lieu de
 * SHUNT_CANNED_BSIC. C'est le "DSP qui poste son decode SCH dans le NDB". */
static int g_sch_fd = -1;

static void shunt_sch_read(void *opaque)
{
    uint8_t buf[64];
    for (;;) {
        ssize_t n = recv(g_sch_fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            break;
        }
        if (n < 12 || memcmp(buf, "SCH1", 4) != 0)
            continue;                       /* pas notre datagramme */
        uint32_t bsic_le, fn_le;
        memcpy(&bsic_le, buf + 4, 4);
        memcpy(&fn_le,   buf + 8, 4);
        int32_t bsic = (int32_t)le32_to_cpu(bsic_le);
        int32_t fn   = (int32_t)le32_to_cpu(fn_le);
        /* TOA reel : 3e int (>=16 o). Absent (ancien format 12 o) -> on garde 23.
         * Clamp a +-64 qbits autour de 23 : au-dela = gr-gsm desaligne, on retombe
         * sur 23 (on-time) pour ne pas catastropher l'alignement firmware. */
        int32_t toa = 23;
        if (n >= 16) {
            uint32_t toa_le; memcpy(&toa_le, buf + 12, 4);
            toa = (int32_t)le32_to_cpu(toa_le);
            if (toa < 23 - 64 || toa > 23 + 64) toa = 23;
        }
        bool first = !g_shunt.sb_valid;
        g_shunt.sb_bsic  = (uint8_t)(bsic & 0x3f);
        g_shunt.sb_fn    = (uint32_t)fn;
        g_shunt.sb_toa   = (int16_t)toa;
        g_shunt.sb_valid = true;
        g_shunt.sb_capture_fn = calypso_trx_get_fn();   /* horodatage : cf. fraicheur */
        static unsigned schlog = 0;
        if (first || schlog++ < 20 || (schlog % 200) == 0)
            SHUNT_LOG("SCH reel (gr-gsm): BSIC=%d "
                    "(ncc=%d bcc=%d) FN=%d TOA=%d%s\n", (int)g_shunt.sb_bsic,
                    (g_shunt.sb_bsic >> 3) & 7, g_shunt.sb_bsic & 7,
                    (int)fn, (int)g_shunt.sb_toa, first ? " [1er]" : "");

        /* [2026-07-25] FN-ALIGN probe : mesure PROPRE de l'offset horloge.
         * Le DSP suit s->fn (calypso_trx_get_fn) ; le SCH porte la FN REELLE du
         * BTS. delta = sch_fn - trx_fn = l'offset a recaler (un vrai mobile cale
         * son horloge sur le SCH ; ici le DSP garde l'horloge TRX offset).
         * delta constant sur les SCH -> offset fixe pose a l'init (fix 1 ligne).
         * toa = offset residuel intra-burst en qbits (23 = on-time). */
        {
            uint32_t trx_fn = calypso_trx_get_fn();
            int32_t d = (int32_t)((uint32_t)fn - trx_fn);
            static unsigned an = 0;
            if (an++ < 60 || (an % 200) == 0)
                fprintf(stderr, "[feed-daram-dsp] FN-ALIGN sch_fn=%u trx_fn=%u "
                        "delta=%d sch%%51=%u toa=%d\n",
                        (unsigned)fn, trx_fn, d, (unsigned)((uint32_t)fn % 51),
                        (int)g_shunt.sb_toa);
        }
    }
}

/* [2026-07-28] CALYPSO_SHUNT_NO_GRGSM : voir en-tete du patch. */
static bool shunt_grgsm_off(void)
{
    static int v = -1;
    if (v < 0) { const char *e = getenv("CALYPSO_SHUNT_NO_GRGSM");
                 v = (e && *e == '1') ? 1 : 0; }
    return v != 0;
}

static void shunt_sch_init(void)
{
    if (shunt_grgsm_off()) {
        SHUNT_ERR("gr-gsm COUPE (CALYPSO_SHUNT_NO_GRGSM=1) : listener SCH :4731 NON arme "
                  "-> sb_bsic/sb_fn/sb_toa doivent venir du DSP");
        return;
    }
    const char *p = getenv("CALYPSO_SHUNT_SCH_PORT");
    int port = (p && *p) ? atoi(p) : 4731;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        SHUNT_ERR("SCH socket() failed: %s", strerror(errno));
        return;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons((uint16_t)port);
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        SHUNT_ERR("SCH bind(:%d) failed: %s", port, strerror(errno));
        close(fd);
        return;
    }
    g_sch_fd = fd;
    qemu_set_fd_handler(fd, shunt_sch_read, NULL, NULL);
    SHUNT_ERR("SCH listener udp:127.0.0.1:%d → feed_sb(BSIC/FN reels "
                 "gr-gsm) → shunt_dispatch_sb (remplace SHUNT_CANNED_BSIC)", port);
}

/* ========================================================================
 * Buffers partages (shm) — gr-gsm AU MILIEU du shunt DSP (pas de FIFO/UDP).
 *   ENTREE du DSP shunte : l'I/Q que la BSP livre (DARAM 0x2a00) est recopiee
 *     ici via calypso_dsp_shunt_feed_iq() ; gr-gsm la LIT.
 *   SORTIE du DSP shunte : gr-gsm ECRIT le SI decode ; le shunt le LIT au frame
 *     tick (shunt_poll_si_shm) et le pousse dans a_cd -> l'ARM le lit.
 * Semantique BUFFER (pas fifo) : un compteur de sequence par sens ; le lecteur
 * poll le seq et ne consomme que s'il a change. shm POSIX /calypso_dsp_shunt.
 * ====================================================================== */
#define SHM_NAME      "/calypso_dsp_shunt"
#define SHM_IQ_SLOTS  64           /* ring de bursts (absorbe les stalls du decode gr-gsm) */
#ifndef SHM_IQ_LEN
#define SHM_IQ_LEN    320          /* int16 par slot (>= 296 = 148 complexes cs16) */
#endif

struct shm_iq_slot {
    uint32_t fn;                   /* frame number du burst */
    uint32_t n;                    /* nb d'int16 valides (I,Q entrelaces) */
    int16_t  iq[SHM_IQ_LEN];
};

struct dsp_shunt_shm {
    uint32_t magic;                /* 0x43445350 = 'CDSP' */
    /* --- ENTREE : ring de bursts I/Q (shunt ecrit <- BSP, gr-gsm lit) --- */
    volatile uint32_t iq_wr;       /* nb total de bursts ecrits (compteur write) */
    struct shm_iq_slot iq[SHM_IQ_SLOTS];
    /* --- SORTIE : SI decode (gr-gsm ecrit, shunt lit -> a_cd) --- */
    volatile uint32_t si_seq;      /* bumpe a chaque nouveau SI decode */
    uint32_t          si_len;      /* octets L2 (<=23) */
    uint8_t           si[32];
};

static struct dsp_shunt_shm *g_shm;
/* [2026-07-27] FB-STREAM : ring d'echantillons FCCH decimes (I/Q entrelaces)
 * que le DSP consomme via intercept de lecture 0x9213/0x9215 (c54x.c).
 * Remplit une VRAIE fenetre dans le workzone 0x2a00. FBS_RING = pow2. */
#define FBS_RING 16384
static int16_t  g_fbs[FBS_RING];
static uint32_t g_fbs_wr, g_fbs_rd;
static uint32_t              g_shm_last_si_seq;
static FILE                 *g_iq_cfile2;  /* cfile #2 FN-espace (zero-fill) -> test grgsm SACCH */

/* [2026-07-30] PLAFOND du cfile #2. Le zero-fill produit un flux CONTINU : spf
 * (def 2500) floats par trame TDMA = 10 ko/trame, soit ~2,2 Mo/s en temps réel
 * = 7,8 Go/h — pour un /dev/shm de 8 Go, qui est de la RAM. C'était donc le seul
 * dump non plafonné du projet, et il ne pouvait pas être activé par défaut.
 * Plafonné, il peut l'être : à l'atteinte du plafond on ferme proprement (le
 * fichier reste décodable) et on le dit UNE fois.
 *   CALYPSO_IQ_CFILE2_MAX_MB=0 -> illimité (à vos risques).
 * Cf. la règle « toute sonde PLAFONNÉE » (TODO.md §4). */
static int64_t g_c2_written;    /* octets écrits */
static int64_t g_c2_max = -1;   /* -1 = non résolu, 0 = illimité */

static void cfile2_wr(const void *buf, size_t nfloats)
{
    if (!g_iq_cfile2) return;
    if (g_c2_max < 0) {
        const char *e = getenv("CALYPSO_IQ_CFILE2_MAX_MB");
        int mb = (e && *e) ? atoi(e) : 512;
        g_c2_max = (mb <= 0) ? 0 : (int64_t)mb * 1024 * 1024;
    }
    if (g_c2_max && g_c2_written >= g_c2_max) {
        SHUNT_ERR("cfile #2 : plafond %lld Mo atteint -> fermeture, le fichier "
                  "reste decodable (CALYPSO_IQ_CFILE2_MAX_MB=0 pour illimite)",
                  (long long)(g_c2_max / (1024 * 1024)));
        fclose(g_iq_cfile2);
        g_iq_cfile2 = NULL;
        return;
    }
    fwrite(buf, sizeof(float), nfloats, g_iq_cfile2);
    g_c2_written += (int64_t)nfloats * (int64_t)sizeof(float);
}
static int                   g_iq_fd      = -1;   /* fd brut I/Q : fichier ou FIFO live */
static int                   g_iq_is_fifo = 0;    /* 1 = FIFO -> non bloquant + drop */
static char                  g_iq_path[256];      /* chemin memorise pour retry FIFO */
static FILE                 *g_iq_rec;            /* record disque .cfile contigu (rejeu), EN PLUS du live */

static void shunt_shm_init(void)
{
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
        SHUNT_ERR("shm_open(%s): %s", SHM_NAME, strerror(errno));
        return;
    }
    if (ftruncate(fd, sizeof(struct dsp_shunt_shm)) != 0) {
        SHUNT_ERR("ftruncate shm: %s", strerror(errno));
        close(fd);
        return;
    }
    void *m = mmap(NULL, sizeof(struct dsp_shunt_shm),
                   PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED) {
        SHUNT_ERR("mmap shm: %s", strerror(errno));
        return;
    }
    g_shm = m;
    g_shm->magic = 0x43445350;
    g_shm_last_si_seq = g_shm->si_seq;
    SHUNT_ERR("shm %s (=/dev/shm%s, %zu o) : I/Q in (feed_iq->gr-gsm) "
                 "+ SI out (gr-gsm->a_cd). gr-gsm AU MILIEU du shunt.",
                 SHM_NAME, SHM_NAME, sizeof(struct dsp_shunt_shm));

    /* Enregistrement .cfile (gr_complex fc32 I,Q normalise) de l'I/Q d'entree
     * du DSP shunte, pour rejeu deterministe (grgsm_cfile_decode.py). Defaut
     * /tmp/dsp_iq.cfile ; CALYPSO_SHUNT_IQ_CFILE= (vide) pour desactiver. */
    const char *cf = getenv("CALYPSO_SHUNT_IQ_CFILE");
    if (!cf)
        cf = "/root/dsp_iq.cfile";
    if (*cf) {
        struct stat st;
        g_iq_is_fifo = (stat(cf, &st) == 0 && S_ISFIFO(st.st_mode));
        snprintf(g_iq_path, sizeof(g_iq_path), "%s", cf);
        if (g_iq_is_fifo) {
            g_iq_fd = open(cf, O_WRONLY | O_NONBLOCK);          /* FIFO : jamais bloquant, pas de create */
            if (g_iq_fd >= 0)
                SHUNT_ERR("I/Q -> %s (FIFO live fc32, non bloquant)", cf);
            else if (errno == ENXIO)
                SHUNT_ERR("FIFO %s sans lecteur — open differe au feed", cf);
            else
                SHUNT_ERR("open(%s) FIFO: %s", cf, strerror(errno));
        } else {
            g_iq_fd = open(cf, O_WRONLY | O_CREAT | O_TRUNC, 0644);   /* cfile rejeu */
            if (g_iq_fd >= 0)
                SHUNT_ERR("enregistre l'I/Q -> %s (cfile fc32)", cf);
            else
                SHUNT_ERR("open(%s) cfile: %s", cf, strerror(errno));
        }
    }
    /* Record disque .cfile contigu (capture brute fc32) EN PLUS de la sortie live :
     * la FIFO sert au live (FFT) sans rien garder, ce record garde tout pour le
     * rejeu deterministe (grgsm_cfile_decode.py). Fichier regulier -> fwrite jamais
     * bloquant. Defaut /dev/shm/dsp_iq.cfile ; CALYPSO_SHUNT_IQ_RECORD= (vide) pour
     * desactiver. On evite le double-open si le record vise le meme fichier que la
     * sortie live (cas live=fichier, pas FIFO). */
    const char *rec = getenv("CALYPSO_SHUNT_IQ_RECORD");
    if (!rec)
        rec = "/dev/shm/dsp_iq.cfile";
    if (*rec && !(g_iq_fd >= 0 && !g_iq_is_fifo && strcmp(rec, g_iq_path) == 0)) {
        g_iq_rec = fopen(rec, "wb");
        if (g_iq_rec)
            SHUNT_ERR("record disque I/Q -> %s (cfile fc32 contigu)", rec);
        else
            SHUNT_ERR("fopen(%s) record: %s", rec, strerror(errno));
    }
    /* cfile #2 : reconstruction FN-espacee (zero-fill des trames manquantes) pour
     * que grgsm retrouve la 51-mf et decode la SACCH (SI5/SI6). Test offline, ne
     * touche PAS au cfile live. Active via CALYPSO_SHUNT_IQ_CFILE2=<chemin>. */
    const char *cf2 = getenv("CALYPSO_SHUNT_IQ_CFILE2");
    if (cf2 && *cf2) {
        g_iq_cfile2 = fopen(cf2, "wb");
        if (g_iq_cfile2)
            SHUNT_ERR("cfile #2 FN-espace -> %s (gap zero-fill)", cf2);
    }
}

/* ENTREE du DSP shunte : la BSP appelle ceci avec l'I/Q DL (cs16, n int16
 * entrelaces I,Q) qu'elle DMA dans la DARAM. Publie dans le shm pour gr-gsm. */
/* [2026-07-22] Injection READ-SIDE FB/SB REELS. Voir header. La detection
 * (feed_iq) met a jour g_shunt.rx_* ; c'est ICI, sur le read MMIO ARM, qu'on
 * livre la valeur -> aucune dependance de timing write/read intra-trame.
 *   FB  (NDB)  : 0x01F0 d_fb_det, 0x01F4 TOA, 0x01F6 PM, 0x01F8 ANGLE, 0x01FA SNR
 *   SB  (db_r) : 0x0060 (page0) / 0x0088 (page1) a_serv_demod[D_TOA] */
/* @BEQUILLE — DECAN (effet de bord : allume SHUNT_REAL_FB)  (CALYPSO_DECAN=1)
 *   masque  : DECAN=1 suffit a allumer l'intercept ci-dessous, et cette fonction
 *             n'a AUCUNE garde g_shunt.active : elle est appelee depuis
 *             calypso_trx.c sur chaque read MMIO ARM 16 bits. Or calypso_native.env
 *             et calypso_native_helped.env posent DECAN:=1 -> en mode "natif", le
 *             d_fb_det lu par l'ARM vient de l'HOTE, jamais de l'API-RAM native.
 *             Le mode natif ne mesure donc pas le natif.
 *   retirer : DECAN!=1 ET SHUNT_REAL_FB!=1 ET SHUNT_LEGIT!=1 (les 3 fallbacks),
 *             ou ajouter une garde g_shunt.active en tete de fonction.
 */
bool calypso_dsp_shunt_real_fb_read(uint32_t off, uint16_t *out)
{
    static int real_fb = -1;
    if (real_fb < 0) {
        /* @BEQUILLE — SHUNT_REAL_FB (intercept du resultat)  (CALYPSO_SHUNT_REAL_FB, defaut OFF)
         *   masque  : le correlateur DSP natif. Le resultat FB est calcule COTE HOTE
         *             et court-circuite ce que le DSP aurait produit.
         *   retirer : quand d_fb_det natif est ecrit par le DSP.
         *   ⚠️ Cette bequille MASQUE le natif : ne jamais l activer pour juger de
         *   l etat du mode natif. Seule data[0x08f8] via DETECTOR-RUN le mesure. */
        const char *e = getenv("CALYPSO_SHUNT_REAL_FB");
        const char *dm = getenv("CALYPSO_DECAN");  /* master DECAN implique REAL_FB */
        /* [2026-07-30] L'implication DECAN -> intercept est le TROISIEME couplage de
         * la meme famille, et il rouvrait la substitution en douce : le parapluie
         * SHUNT_LEGIT charge calypso_shunt_legit.env qui pose DECAN:=1, donc meme
         * avec SHUNT_REAL_FB=0 ET PUBLISH_FB=0 l'ARM relisait g_shunt.rx_fb_det.
         * Comme pour l'implication SHUNT_LEGIT ci-dessous, on la conditionne a
         * PUBLISH_FB (defaut 1 = comportement historique inchange).
         * `SHUNT_REAL_FB=1` reste un opt-in EXPLICITE et n'est pas touche. */
        real_fb = (e && *e == '1') ? 1 : 0;
        if (!real_fb && dm && dm[0] == '1')
            real_fb = calypso_gate("CALYPSO_SHUNT_PUBLISH_FB", 1) ? 1 : 0;
        /* [2026-07-26] SHUNT_LEGIT implique l'intercept de lecture : c'est LUI
         * qui livre rx_fb_det/rx_snr (detection gr-gsm) a l'ARM. Sans ca, le
         * feed legit n'atteint jamais la lecture ARM.
         *
         * [2026-07-30] DECOUPLE. Cette implication est LE piege qui a produit un
         * faux positif : avec `SHUNT_REAL_FB=0` et `SHUNT_LEGIT=1`, l'intercept
         * restait ARME, donc l'ARM lisait `d_fb_det = 1` alors que les SEULES
         * ecritures DSP mesurees valaient 0 (`0xb2cc` = `st #0x0000`, `0x778a` =
         * `andm #0xfffe`). La lecture ne touchait jamais la cellule : elle rendait
         * `g_shunt.rx_fb_det`, pose par le demod HOTE (l.~1832, det = coh>0.95 &&
         * |resid|<0.13) — qui n'est gate par aucun parapluie.
         * L'implication est donc desormais conditionnee par PUBLISH_FB, meme gate
         * que le bloc de transport. PUBLISH_FB=0 => l'ARM lit la VRAIE cellule.
         * NB : `SHUNT_REAL_FB=1` et `DECAN=1` restent des opt-in explicites et
         * gardent l'ancien comportement — on ne desarme que l'implication. */
        if (!real_fb) {
            const char *l = getenv("CALYPSO_SHUNT_LEGIT");
            int legit_on = (l && *l == '1') ? 1 : 0;
            real_fb = (legit_on && calypso_gate("CALYPSO_SHUNT_PUBLISH_FB", 1)) ? 1 : 0;
        }
    }
    if (!real_fb) return false;
    switch (off) {
    case 0x01F0: *out = g_shunt.rx_fb_det ? 1 : 0;    return true; /* d_fb_det */
    case 0x01F4: *out = g_shunt.rx_toa;               return true; /* a_sync TOA */
    case 0x01F6: *out = g_shunt.last_pm;              return true; /* a_sync PM  */
    case 0x01F8: *out = (uint16_t)g_shunt.rx_afc;     return true; /* a_sync ANGLE(AFC) */
    case 0x01FA: *out = g_shunt.rx_snr;               return true; /* a_sync SNR */
    /* SB via db_r a_serv_demod[D_TOA] (page0/page1) : seulement si SB reel poste */
    case 0x0060: case 0x0088:
        if (g_shunt.sb_valid) { *out = g_shunt.rx_toa; return true; }
        return false;
    default: return false;
    }
}

void calypso_dsp_shunt_feed_iq(uint32_t fn, const int16_t *iq, int n)
{
    if (!iq || n <= 0)
        return;
    if (!g_shm && !(shunt_route_c54x() && g_shunt.c54x))
        return;   /* sans shm ET sans route c54x, rien a faire */
    if (n > SHM_IQ_LEN)
        n = SHM_IQ_LEN;
    /* PM REEL : magnitude moyenne (MAV) du burst DL -> g_shunt.last_pm. Pas de
     * sqrt/math.h ; signal-derive, plus de 0x7000 canne. Le dispatch l'ecrit
     * dans a_serv_demod[D_PM] -> rxlev reel cote firmware. */
    {
        uint64_t acc = 0;
        for (int i = 0; i < n; i++) { int v = iq[i]; acc += (v < 0) ? (uint32_t)(-v) : (uint32_t)v; }
        uint32_t mav = (uint32_t)(acc / (uint32_t)n);
        g_shunt.last_pm = (mav > 0xffff) ? 0xffff : (uint16_t)mav;
    }

    /* [2026-07-26 golive-mac] ROOT-CAUSE d_fb_det=0 : le kernel FB natif
     * (reroute 0x94f5 -> a076) walk la DARAM 0x2a00 mais y lit une CONSTANTE
     * DC 0x12ed (writer rx_burst degenere) -> MAC sur signal plat -> det=0.
     * feed_iq DETIENT les vrais samples FCCH (coh=0.999). On les ecrit DIRECT
     * en DARAM 0x2a00, DECIMES ->1-SPS (ce que le kernel attend).
     * Gate CALYPSO_FB_IQ_DARAM ; FCCH-only si CALYPSO_FB_IQ_FCCH_ONLY.
     * (Mettre CALYPSO_BSP_DIRECT_FEED=0 pour tuer le writer 0x12ed concurrent.) */
    {
        /* @BEQUILLE — FB_IQ_DARAM (+ _BASE, _FCCH_ONLY)  (CALYPSO_FB_IQ_DARAM, atoi>0,
         *              defaut OFF ; native_helped.env:=1)
         *   masque  : le DMA on-chip RX -> DARAM. feed_iq ecrit 0x128 mots d'IQ decime
         *             DIRECTEMENT dans g_shunt.c54x->data[base..], hors data_write — donc
         *             invisible de WATCH_2A00 / WATCH_9200 / WMAP.
         *   retirer : quand la chaine BSP -> BDLENA -> DARAM alimente le buffer seule
         *             (writer 0x12ed non degenere).
         */
        static int _fid = -1, _decim = 4, _fcch = 0;
        if (_fid < 0) {
            const char *e = getenv("CALYPSO_FB_IQ_DARAM"); _fid = (e && atoi(e) > 0) ? 1 : 0;
            const char *d = getenv("CALYPSO_BSP_IQ_DECIM"); if (d && *d) _decim = atoi(d);
            if (_decim < 1) _decim = 1;
            const char *f = getenv("CALYPSO_FB_IQ_FCCH_ONLY"); _fcch = (f && atoi(f) > 0) ? 1 : 0;
        }
        static uint16_t _iqbase = 0;
        if (_iqbase == 0) {
            const char *b = getenv("CALYPSO_FB_IQ_BASE");
            _iqbase = (b && *b) ? (uint16_t)strtol(b, NULL, 0) : 0x2a00;
        }
        /* [2026-07-27 diag] HUNK-ENTER : prouve si feed_iq atteint le hunk marker,
         * et expose les gardes (fid/c54x/data) qui decident l ecriture 0x2a00. */
        { static unsigned _he = 0;
          if (_he++ < 20)
            fprintf(stderr, "[feed-daram-dsp] HUNK-ENTER fid=%d c54x=%p data=%p n=%d fn=%u\n",
                    _fid, (void*)g_shunt.c54x,
                    g_shunt.c54x ? (void*)g_shunt.c54x->data : (void*)0, n, fn); }
        int _p = (int)(fn % 51);
        /* [2026-07-26] FCCH positions {1,11,21,31,41} (offset +1 vs canon 0/10/20/30/40,
         * confirme par FN-ALIGN sch%51=1,21,31,41). */
        int _is_fcch = (_p % 10 == 1) && (_p <= 41);
        /* @BEQUILLE — FB_IQ_MARKER  (CALYPSO_FB_IQ_MARKER, atoi>0, defaut OFF)
         *   masque  : rien de reel — remplace l'IQ par une RAMPE 0x1000+woff pour tester
         *             la reachabilite de la vue DARAM du noyau. Court-circuite la branche
         *             IQ reelle (else if) et ignore FCCH_ONLY.
         *   retirer : des que la reachabilite est etablie ; ne jamais laisser en run.
         */
        static int _mark = -1;
        if (_mark < 0) { const char *m = getenv("CALYPSO_FB_IQ_MARKER"); _mark = (m && atoi(m) > 0) ? 1 : 0; }
        if (_fid && _mark && g_shunt.c54x && g_shunt.c54x->data) {
            /* TEST REACHABILITE : ecrit une RAMPE 0x1000+woff a CHAQUE frame (ignore
             * iq + fcch). Si IQ-READ voit la rampe -> feed_iq atteint bien la vue
             * DARAM du kernel (probleme = contenu iq). Sinon -> mismatch objet/mapping. */
            uint16_t base = _iqbase; int dl = 0x128;
            for (int woff = 0; woff < dl; woff++)
                g_shunt.c54x->data[base + woff] = (uint16_t)(0x1000 + woff);
            static unsigned _lm = 0;
            if (_lm++ < 8)
                fprintf(stderr, "[feed-daram-dsp] FB-IQ-MARKER fn=%u c54x=%p wrote RAMP 0x1000.. "
                        "base[0]=0x%04x base[1]=0x%04x base[2]=0x%04x\n", fn, (void*)g_shunt.c54x,
                        g_shunt.c54x->data[base], g_shunt.c54x->data[base+1], g_shunt.c54x->data[base+2]);
        } else if (_fid && g_shunt.c54x && g_shunt.c54x->data && (!_fcch || _is_fcch)) {
            uint16_t base = _iqbase; int dl = 0x128; int woff = 0;
            for (int k = 0; 2*(k*_decim)+1 < n && woff < dl; k++) {
                g_shunt.c54x->data[base + woff++] = (uint16_t)iq[2*(k*_decim)];
                if (woff < dl)
                    g_shunt.c54x->data[base + woff++] = (uint16_t)iq[2*(k*_decim)+1];
            }
            static unsigned _l = 0;
            if (_l++ < 16)
                fprintf(stderr, "[feed-daram-dsp] FB-IQ-DARAM fn=%u p=%d wrote=%d decim=%d "
                        "s0=0x%04x s1=0x%04x s2=0x%04x\n", fn, _p, woff, _decim,
                        g_shunt.c54x->data[base], g_shunt.c54x->data[base+1],
                        g_shunt.c54x->data[base+2]);
        }
    }

    /* [2026-07-27] FB-STREAM push : pousse l'IQ FCCH decime dans le ring que
     * l'intercept 0x9213/0x9215 (c54x.c) sert au demod. Gate CALYPSO_FB_STREAM. */
    {
        static int _fs = -1, _fsd = 4;
        /* @BEQUILLE — FB_STREAM (alimentation du ring)  (CALYPSO_FB_STREAM, defaut OFF)
         *   masque  : cote emetteur du meme contournement — pousse l IQ decime dans
         *             le ring que l intercept de lecture sert au demod.
         *   retirer : en meme temps que l intercept de lecture. */
        if (_fs < 0) { const char *e = getenv("CALYPSO_FB_STREAM"); _fs = (e && atoi(e) > 0) ? 1 : 0;
            const char *d = getenv("CALYPSO_FB_STREAM_DECIM"); if (d && *d) _fsd = atoi(d); if (_fsd < 1) _fsd = 1; }
        if (_fs) {
            /* [2026-07-27] SKIP frames all-zero (startup fn 0-4) : elles polluent le
             * ring que le demod lit au front -> il tombe sur des zeros au lieu de la
             * vraie FCCH poussee ensuite. On ne pousse que si la frame a du signal. */
            int _nz = 0;
            for (int i = 0; i < n && i < 64; i++) if (iq[i]) { _nz = 1; break; }
            if (_nz) {
                for (int k = 0; 2*(k*_fsd)+1 < n; k++) {
                    g_fbs[g_fbs_wr++ & (FBS_RING-1)] = iq[2*(k*_fsd)];
                    g_fbs[g_fbs_wr++ & (FBS_RING-1)] = iq[2*(k*_fsd)+1];
                }
            }
        }
    }

    /* [2026-07-22] Detection FCCH REELLE (gate CALYPSO_SHUNT_REAL_FB) : coherence
     * + dphi sur la vraie RX -> d_fb_det/AFC/SNR/TOA reels (bypass go-live DSP). */
    {
        /* @BEQUILLE — SHUNT_REAL_FB (calcul hote de rx_fb_det/AFC/SNR/TOA)
         *              (CALYPSO_SHUNT_REAL_FB=1 ou master CALYPSO_DECAN=1)
         *   masque  : le correlateur DSP. Coherence + dphi calcules cote hote sur l'I/Q RX
         *             -> g_shunt.rx_* -> livres a l'ARM par real_fb_read.
         *   retirer : quand data[0x08f8] est ecrit par le DSP.
         *   ATTENTION : DECAN=1 SUFFIT a l'allumer, et native/native_helped/shunt_legit/
         *             shunt_no_legit posent tous DECAN=1 : mettre SHUNT_REAL_FB=0 ne coupe
         *             RIEN.
         */
        static int real_fb = -1;
        /* [2026-08-03] `CALYPSO_SHUNT_REAL_FB=0` ne coupait pas quand DECAN=1 :
         * un maitre a le droit d'IMPLIQUER un sous-gate, pas d'ECRASER un 0 pose a
         * la main. DECAN devient le DEFAUT. */
        if (real_fb < 0) { const char *dm = getenv("CALYPSO_DECAN");
                           real_fb = calypso_gate("CALYPSO_SHUNT_REAL_FB",
                                                  (dm && dm[0] == '1') ? 1 : 0); }
        if (real_fb) {
            int nc = n / 2;
            if (nc >= 8) {
                double ar = 0, ai = 0, den = 0;
                for (int k = 1; k < nc; k++) {
                    double i0 = iq[2*(k-1)], q0 = iq[2*(k-1)+1];
                    double i1 = iq[2*k],     q1 = iq[2*k+1];
                    ar += i1*i0 + q1*q0; ai += q1*i0 - i1*q0;
                    den += sqrt((i0*i0+q0*q0)*(i1*i1+q1*q1));
                }
                double coh = (den > 0) ? sqrt(ar*ar + ai*ai) / den : 0;
                double dphi = atan2(ai, ar);
                /* [DECAN/AFC-fix v2 2026-07-26] IQ @4 SPS (CONSTAT empirique : le
                 * burst le plus coherent, coh~0.9995, sort dphi~=0.39=pi/8, PAS
                 * pi/2). A 4 SPS le ton FCCH +fc/4 = +22.5deg = +pi/8 par sample ;
                 * fs = 4*270833 = 1083333. Nominal FIXE pi/8 (mon pi/2 v1 -> residu
                 * geant -> Angle=-32kHz hors capture -> FCCH jamais lock). */
                double resid = dphi - M_PI/8.0;          /* residu de phase/sample (rad) */
                if (resid >  M_PI) resid -= 2.0*M_PI;
                if (resid < -M_PI) resid += 2.0*M_PI;
                /* det = VRAI FCCH : ton pur (coh haute) ET proche du nominal (dans
                 * la fenetre de capture FB0 +/-20kHz -> |resid|<0.13). On ne met a
                 * jour l'AFC/SNR QUE sur le FCCH : feed_iq tourne sur CHAQUE burst,
                 * et un burst DATA a un dphi aleatoire (ex coh=0.96/dphi=0.062, pas
                 * le FCCH) qu'on injectait avant comme garbage -> AFC divergeait. */
                int det = (coh > 0.95) && (fabs(resid) < 0.13);
                g_shunt.rx_fb_det = det;
                if (det) {
                    /* ANGLE fidele : Df_Hz = resid*fs/(2pi), fs=1083333 ;
                     * angle = Df*65536/86208 = resid*131072. BTS cale => residu~0
                     * => angle~0 => AFC converge (ANGLE=0 canne, mais MESURE). */
                    /* [AFC loop-close 2026-07-26] FERME la boucle : le firmware
                     * afc_correct enroule d_afc ; le modele twl3025 en deduit la
                     * frequence DEJA compensee (get_afc_hz) -> on la SOUSTRAIT de la
                     * mesure brute. Sans ca la mesure reste ~constante et le DAC
                     * s'enroule sans fin (-700 -> -1800...). Avec, l'erreur effective
                     * -> 0 => convergence (gain~1, init -700). fs=1083333 (4 SPS). */
                    double raw_hz = resid * (1083333.0 / (2.0 * M_PI));
                    g_rx_raw_hz = raw_hz; g_rx_raw_valid = 1; /* memo pour recompute per-tick */
                    double eff_hz = raw_hz - calypso_twl3025_get_afc_hz();
                    double a = eff_hz * (65536.0 / 86208.0);
                    if (a >  32767.0) a =  32767.0;
                    if (a < -32768.0) a = -32768.0;
                    g_shunt.rx_afc = (int16_t)a;
                    /* SNR fx6.10 (word=snr_dB*1024, seuil 2560=2.5dB) depuis M=coh^2. */
                    double M = coh * coh;
                    if (M > 0.9999) M = 0.9999;
                    double snr_db = 10.0 * log10(M / (1.0 - M));
                    if (snr_db < 0.0)  snr_db = 0.0;
                    if (snr_db > 30.0) snr_db = 30.0;
                    int w = (int)(snr_db * 1024.0);
                    if (w > 0x7FFF) w = 0x7FFF;
                    g_shunt.rx_snr = (uint16_t)w;
                }
                /* det=0 (burst non-FCCH) : on GARDE le dernier rx_afc/rx_snr lockes
                 * (pas de garbage data). */
                g_shunt.rx_toa    = 23;
                /* [2026-07-22] Ecrit d_fb_det+sync dans le NDB PAR FRAME (comme le vrai
                 * DSP qui tourne 12 frames apres 1 dispatch) -> l'ARM lit 1 sur chaque
                 * attempt (il le remet a 0 apres lecture, prim_fbsb.c:318). Fix desync. */
                /* Ecrit dsp->data[] DIRECT (comme shunt_route_to_c54x l.396) : le
                 * shunt_write_w/dma_memory_write ne mirror PAS vers dsp->data ou
                 * l'ARM+fbsb lisent (0x08F8..). DSP-words : d_fb_det=0x08F8,
                 * a_sync_demod TOA=0x08FA PM=0x08FB ANG=0x08FC SNR=0x08FD. */
                static unsigned _wl = 0;
                if (_wl++ < 12)
                    fprintf(stderr, "[feed-daram-dsp] FB-WRITE-DBG det=%d c54x=%p data=%p api=%p\n",
                            det, (void*)g_shunt.c54x,
                            g_shunt.c54x ? (void*)g_shunt.c54x->data : (void*)0,
                            g_shunt.c54x ? (void*)g_shunt.c54x->api_ram : (void*)0);
                /* [2026-07-22] Write async dsp->data[] SUPPRIME : la livraison
                 * FB/SB se fait desormais READ-SIDE (calypso_dsp_shunt_real_fb_read
                 * appelee par calypso_dsp_read), immunisee contre l'ordonnancement
                 * intra-trame. feed_iq ne fait QUE mettre a jour g_shunt.rx_*. */
                static unsigned rfl = 0;
                if (rfl < 20 || (det && rfl < 300)) {
                    fprintf(stderr, "[feed-daram-dsp] REAL-FB fn=%u nc=%d coh=%.3f dphi=%.3f "
                            "det=%d SNR=0x%04x AFC=%d\n", fn, nc, coh, dphi, det,
                            g_shunt.rx_snr, g_shunt.rx_afc);
                    rfl++;
                }
                /* [2026-07-28] SHUNT_DSP_FB : voir en-tete du patch. */
                {
                    /* @BEQUILLE — SHUNT_DSP_FB (+ _ENTRY / _BUDGET / _MAX / _SP)
                     *              (CALYPSO_SHUNT_DSP_FB, EQ1, defaut OFF)
                     *   masque  : l'ORDONNANCEMENT natif du correlateur. Le shunt sauvegarde le
                     *             contexte c54x, force PC=ENTRY et SP=scratch, execute BUDGET
                     *             instructions hors trame, puis restaure — le DSP n'a jamais decide
                     *             d'entrer la.
                     *   retirer : quand le dispatcher natif (frame-IT -> 0x8341 -> correlateur) atteint
                     *             l'entree seul.
                     *   NB      : niche dans le bloc real_fb : inerte si DECAN=0 ET SHUNT_REAL_FB!=1.
                     */
                    static int _sdf = -1; static uint16_t _entry = 0x94f5;
                    static int _budget = 20000; static unsigned _sdfn = 0;
                    static unsigned _sdfmax = 40;   /* borne totale d excursions */
                    static uint16_t _spscratch = 0x5000;  /* pile dediee a l excursion */
                    if (_sdf < 0) {
                        const char *e = getenv("CALYPSO_SHUNT_DSP_FB");
                        _sdf = (e && *e == '1') ? 1 : 0;
                        const char *p = getenv("CALYPSO_SHUNT_DSP_FB_ENTRY");
                        if (p && *p) _entry = (uint16_t)strtol(p, NULL, 0);
                        const char *b = getenv("CALYPSO_SHUNT_DSP_FB_BUDGET");
                        if (b && *b) _budget = atoi(b);
                        const char *m = getenv("CALYPSO_SHUNT_DSP_FB_MAX");
                        if (m && *m) _sdfmax = (unsigned)atoi(m);
                        const char *sp = getenv("CALYPSO_SHUNT_DSP_FB_SP");
                        if (sp && *sp) _spscratch = (uint16_t)strtol(sp, NULL, 0);
                        if (_sdf)
                            SHUNT_ERR("SHUNT_DSP_FB arme : entree=0x%04x budget=%d "
                                      "(correlateur DSP pilote par le shunt ; REAL_FB reste l oracle)",
                                      _entry, _budget);
                    }
                    /* borne stricte : au-dela, inerte — sinon on affame osmocon/mobile (28/07). */
                    if (_sdf && g_shunt.c54x && _sdfn < _sdfmax) {
                        C54xState *_d = g_shunt.c54x;
                        /* --- sauvegarde du contexte --- */
                        uint16_t _pc = _d->pc, _xpc = _d->xpc, _sp = _d->sp;
                        uint16_t _st0 = _d->st0, _st1 = _d->st1, _t = _d->t;
                        int64_t  _a = _d->a, _b = _d->b;
                        bool     _idle = _d->idle;
                        uint16_t _ar[8];
                        for (int _k = 0; _k < 8; _k++) _ar[_k] = _d->ar[_k];
                        /* --- excursion bornee dans le correlateur --- */
                        /* pile DEDIEE : l excursion ne doit jamais ecrire dans la pile du DSP
                         * (restaurer SP ne restaure pas le CONTENU ecrase). */
                        _d->sp = _spscratch;
                        _d->pc = _entry; _d->idle = false; _d->running = true;
                        c54x_run(_d, _budget);
                        _sdfn++;
                        if (_sdfn <= _sdfmax) {
                            fprintf(stderr, "[feed-daram-dsp] DSP-FB fn=%u PC=0x%04x "
                                    "A=0x%010llx B=0x%010llx T=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x\n"
                                    "                          wz[2c00..07]=%04x %04x %04x %04x %04x %04x %04x %04x"
                                    " | ORACLE det=%d coh=%.3f\n",
                                    fn, _d->pc,
                                    (unsigned long long)(_d->a & 0xFFFFFFFFFFULL),
                                    (unsigned long long)(_d->b & 0xFFFFFFFFFFULL),
                                    _d->t, _d->ar[3], _d->ar[4], _d->ar[5], _d->ar[6],
                                    _d->data[0x2c00], _d->data[0x2c01], _d->data[0x2c02],
                                    _d->data[0x2c03], _d->data[0x2c04], _d->data[0x2c05],
                                    _d->data[0x2c06], _d->data[0x2c07], det, coh);
                        }
                        /* --- restauration : le DSP retrouve son etat exact --- */
                        _d->pc = _pc; _d->xpc = _xpc; _d->sp = _sp;
                        _d->st0 = _st0; _d->st1 = _st1; _d->t = _t;
                        _d->a = _a; _d->b = _b; _d->idle = _idle;
                        for (int _k = 0; _k < 8; _k++) _d->ar[_k] = _ar[_k];
                    }
                }
            }
        }
    }

    /* CALYPSO_DSP=c54x : stash du dernier burst (cs16 I,Q) ; rejoue dans
     * bsp_buf depuis shunt_route_to_c54x() au frame tick. */
    if (shunt_route_c54x() && g_shunt.c54x) {
        int m = (n > SHM_IQ_LEN) ? SHM_IQ_LEN : n;
        memcpy(g_shunt.last_iq, iq, (size_t)m * sizeof(int16_t));
        g_shunt.last_iq_n     = m;
        g_shunt.last_iq_fn    = fn;
        g_shunt.last_iq_valid = true;
    }

    if (g_shm) {
        struct shm_iq_slot *slot = &g_shm->iq[g_shm->iq_wr % SHM_IQ_SLOTS];
        slot->fn = fn;
        slot->n  = (uint32_t)n;
        memcpy(slot->iq, iq, (size_t)n * sizeof(int16_t));
        __sync_synchronize();
        g_shm->iq_wr++;               /* publie le burst (le lecteur poll iq_wr) */
    }

    /* Sorties fc32 (I,Q normalise) : (a) live -> FIFO (FFT, drop) ou fichier, via
     * g_iq_fd ; (b) record disque contigu -> g_iq_rec (rejeu deterministe). fbuf
     * calcule une seule fois et diffuse aux deux. */
    if (g_iq_is_fifo && g_iq_fd < 0)
        g_iq_fd = open(g_iq_path, O_WRONLY | O_NONBLOCK);     /* retry : le lecteur est-il apparu ? */
    if (g_iq_fd >= 0 || g_iq_rec) {
        float fbuf[SHM_IQ_LEN];
        for (int i = 0; i < n; i++)
            fbuf[i] = (float)iq[i] / 32768.0f;
        if (g_iq_fd >= 0) {
            ssize_t w = write(g_iq_fd, fbuf, (size_t)n * sizeof(float));
            if (w < 0 && g_iq_is_fifo && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                /* pipe plein -> drop ce burst (FFT live, perte tolerable) */
            } else if (w < 0 && g_iq_is_fifo && (errno == EPIPE || errno == ENXIO)) {
                close(g_iq_fd); g_iq_fd = -1;   /* lecteur parti -> on reessaiera */
            }
        }
        if (g_iq_rec)                                  /* record disque : jamais bloquant */
            fwrite(fbuf, sizeof(float), (size_t)n, g_iq_rec);
    }
    /* cfile #2 FN-espace : chaque burst TS0 a sa position de trame
     * ((fn-base)*spf int16), trames manquantes zero-fillees -> grgsm retrouve la
     * 51-mf -> SACCH (SI5/SI6) decodable. spf = int16/trame TDMA (def 2500=1x,
     * sweepable via CALYPSO_IQ_CFILE_SPF pour le test offline). */
    if (g_iq_cfile2) {
        static int spf = -1; static uint32_t base_fn = 0; static int64_t pos = 0; static int have_base = 0;
        if (spf < 0) { const char *e = getenv("CALYPSO_IQ_CFILE_SPF"); spf = (e && *e) ? atoi(e) : 2500; }
        if (!have_base) { base_fn = fn; pos = 0; have_base = 1; }
        int64_t target = (int64_t)fn - (int64_t)base_fn;
        if (target < 0) target += 2715648;            /* hyperframe wrap */
        target *= spf;
        int64_t gap = target - pos;
        if (gap < 0 || gap > (int64_t)spf * 300) { base_fn = fn; pos = 0; gap = 0; }  /* rebase si saut anormal */
        static const float zeros[512] = {0};
        while (gap > 0 && g_iq_cfile2) { int c = gap > 512 ? 512 : (int)gap; cfile2_wr(zeros, (size_t)c); pos += c; gap -= c; }
        float fbuf2[SHM_IQ_LEN];
        for (int i = 0; i < n; i++) fbuf2[i] = (float)iq[i] / 32768.0f;
        cfile2_wr(fbuf2, (size_t)n);
        pos += n;
    }
    /* cfile #2 FN-espace : chaque burst TS0 a sa position de trame
     * ((fn-base)*spf int16), trames manquantes zero-fillees -> grgsm retrouve la
     * 51-mf -> SACCH (SI5/SI6) decodable. spf = int16/trame TDMA (def 2500=1x,
     * sweepable via CALYPSO_IQ_CFILE_SPF pour le test offline). */
    if (g_iq_cfile2) {
        static int spf = -1; static uint32_t base_fn = 0; static int64_t pos = 0; static int have_base = 0;
        if (spf < 0) { const char *e = getenv("CALYPSO_IQ_CFILE_SPF"); spf = (e && *e) ? atoi(e) : 2500; }
        if (!have_base) { base_fn = fn; pos = 0; have_base = 1; }
        int64_t target = (int64_t)fn - (int64_t)base_fn;
        if (target < 0) target += 2715648;            /* hyperframe wrap */
        target *= spf;
        int64_t gap = target - pos;
        if (gap < 0 || gap > (int64_t)spf * 300) { base_fn = fn; pos = 0; gap = 0; }  /* rebase si saut anormal */
        static const float zeros[512] = {0};
        while (gap > 0 && g_iq_cfile2) { int c = gap > 512 ? 512 : (int)gap; cfile2_wr(zeros, (size_t)c); pos += c; gap -= c; }
        float fbuf2[SHM_IQ_LEN];
        for (int i = 0; i < n; i++) fbuf2[i] = (float)iq[i] / 32768.0f;
        cfile2_wr(fbuf2, (size_t)n);
        pos += n;
    }
}

/* SORTIE du DSP shunte : gr-gsm a-t-il ecrit un nouveau SI ? Si oui -> a_cd. */
static void shunt_poll_si_shm(void)
{
    if (shunt_grgsm_off()) return;   /* SI shm = ecrit par gr-gsm */
    if (!g_shm)
        return;
    uint32_t seq = g_shm->si_seq;
    if (seq == g_shm_last_si_seq)
        return;
    __sync_synchronize();
    g_shm_last_si_seq = seq;
    uint32_t len = g_shm->si_len;
    if (len == 0 || len > sizeof(g_shm->si))
        return;
    calypso_dsp_shunt_feed_si(g_shm->si, (int)len);
}

/* ---- init : called from machine setup when CALYPSO_DSP_SHUNT=1 ---- */
void calypso_dsp_shunt_init(MemoryRegion *system_memory, AddressSpace *as)
{
    /* Actif si CALYPSO_DSP_SHUNT=1 OU CALYPSO_L1=c : dans ce dernier cas le HLE
     * (calypso_layer1.c) pilote le FB, mais SB (a_sch) + SI (a_cd) n'existent que
     * dans le shunt -> on l'arme aussi pour fournir le chemin réception prouvé
     * (FB+SB+SI) qui va jusqu'au LU accept. Le shunt on_frame_tick tourne ~1ms
     * après le tick L1=c, donc ses écritures d_fb_det/a_sch/a_cd priment. */
    /* @BEQUILLE — DSP_SHUNT (armement du mock)  (CALYPSO_DSP_SHUNT, EQ1 ; defaut 1 via
     *              CALYPSO_MODE=full-grgsm dans run.sh, PAS via un calypso*.env)
     *   masque  : le DSP entier. Un mock cote ARM ecrit d_fb_det / a_sch / a_cd a sa
     *             place. NB : le shunt s'arme AUSSI via CALYPSO_L1=c ou simplement
     *             CALYPSO_DSP=c54x (shunt_route_c54x) — donc il est arme meme dans les
     *             profils dits "natifs".
     *   retirer : quand le correlateur natif produit d_fb_det seul (RANK3 leve).
     */
    const char *env = getenv("CALYPSO_DSP_SHUNT");
    bool shunt_env_on = (env && strcmp(env, "1") == 0);
    if (!shunt_env_on && !calypso_l1_c_active() && !shunt_route_c54x()) {
        g_shunt.active = false;
        return;
    }

    g_shunt.active = true;
    g_shunt.as     = as;
    g_shunt.pending = false;
    g_shunt.tick_cnt = 0;

    /* Overlay the single d_dsp_page word as IO. The rest of the API RAM
     * stays as plain RAM that the firmware reads/writes directly. */
    MemoryRegion *trigger = g_new0(MemoryRegion, 1);
    memory_region_init_io(trigger, NULL, &shunt_ndb_trigger_ops, NULL,
                          "calypso-dsp-shunt-trigger", 2);
    memory_region_add_subregion_overlap(system_memory,
                                        BASE_API_NDB + NDB_D_DSP_PAGE,
                                        trigger,
                                        /*priority=*/10);

    /* Offsets NDB : lus dans le DWARF du firmware CHARGE, avant tout dispatch.
     * A faire en premier — tout ce qui suit ecrit dans ces buffers. */
    shunt_ndb_resolve_offsets();

    /* Pont gr-gsm → a_cd : écoute le SI décodé (GSMTAP) et l'injecte. */
    shunt_gsmtap_init();

    /* Pont gr-gsm → SB : écoute le BSIC/FN réels (SCH) et les injecte dans
     * shunt_dispatch_sb (remplace SHUNT_CANNED_BSIC). */
    shunt_sch_init();

    /* Buffers shm : gr-gsm au milieu du shunt (I/Q in + SI out, pas de fifo). */
    shunt_shm_init();

    /* CALYPSO_CANNED : résoudre + ÉNUMÉRER explicitement la dette restante. */
    g_canned = shunt_parse_canned();
    {
        const char *no_canned = getenv("CALYPSO_SHUNT_NO_CANNED");
        SHUNT_ERR("CALYPSO_CANNED (dette fabriquée EXPLICITE) : "
                     "FBDET=%d TOA=%d PM=%d SNR=%d ANGLE=%d CRC=%d  "
                     "[non-canné=valeur réelle/0]. Hors var : BSIC=%s, SI=%s.",
                     !!(g_canned & CAN_FBDET), !!(g_canned & CAN_TOA),
                     !!(g_canned & CAN_PM), !!(g_canned & CAN_SNR),
                     !!(g_canned & CAN_ANGLE), !!(g_canned & CAN_CRC),
                     "réel via gr-gsm (fallback 63 si pas no-canned)",
                     (no_canned && *no_canned == '1')
                        ? "réel via feed_si (no-canned, gate si absent)"
                        : "réel via feed_si (+ fallback legacy possible)");
    }

    SHUNT_ERR("active — c54x emulator should be skipped, "
                 "BSP DMA→DARAM should be gated. Watch /tmp/qemu.log for "
                 "LATCH/DISPATCH lines.");
}

/* Phase-2 hook (IPC integration) — calypso-ipc-device will call this with
 * the result of GMSK demod from osmo-trx-ipc instead of canned values. */
void calypso_dsp_shunt_feed_fb_result(int found, int16_t toa,
                                      int16_t pm, int16_t angle, int16_t snr)
{
    /* TODO Phase 2 */
    (void)found; (void)toa; (void)pm; (void)angle; (void)snr;
}

/* Point d'injection COMMUN du SI réel (2026-06-02) : gr-gsm (via pont) OU la
 * démod C native appellent ceci avec une frame L2 de 23 octets décodée depuis
 * l'I/Q réel du BTS. Le shunt l'écrit ensuite dans a_cd (shunt_dispatch_allc)
 * à la place du SI3 canned → "sans hack", vrai signal. len doit être 23 (XCCH
 * L2). Réécrit à chaque nouveau SI (rotation SI1/2/3/4 du BCCH). */


/* [2026-07-22] DE-ALIAS du d_burst_d. RACINE du jitter burst-ID : g_shunt.d_burst_d
 * etait capture dans shunt_latch_task (horloge d_dsp_page/scenario) qui SOUS-
 * ECHANTILLONNE le flux commande NB propre 0,1,2,3 -> sequence aliasee periode-12,
 * phase figee au boot (non-deterministe). Ici on capture+mirror A CHAQUE ecriture
 * ARM de la write-page d_burst_d (WP_D_BURST_D : offset 0x0002 page0 / 0x002A page1),
 * 1:1 avec les commandes -> plus d'aliasing. On ecrit l'echo (avec l'offset SCHED)
 * sur LES DEUX read-pages pour que le mobile lise toujours la commande la plus
 * recente quel que soit r_page. Gate CALYPSO_SHUNT_BURST_PERCMD (defaut ON). */
void calypso_dsp_shunt_wp_burst_write(uint32_t off, uint16_t value)
{
    /* [2026-07-27] GARDE SHUNT-INACTIF : appele SANS CONDITION depuis
     * calypso_dsp_write() (calypso_trx.c), y compris en mode NATIF ou le
     * shunt n est pas arme. Sans cette garde, g_shunt.as == NULL et
     * shunt_write_w() segfault (backtrace gdb : address_space_write as=0x0).
     * C est une feature du shunt : hors shunt, elle ne doit rien faire. */
    if (!g_shunt.active) return;
    /* @BEQUILLE — SHUNT_BURST_PERCMD (miroir per-commande)  (CALYPSO_SHUNT_BURST_PERCMD,
     *              calypso_gate, defaut = ON sous shunt_legit, OFF ailleurs)
     *   masque  : la derivation materielle de d_burst_d. On capture chaque ecriture ARM
     *             de la write-page et on MIROITE l'echo sur LES DEUX read-pages, parce
     *             que le r_page du mobile n'est pas modelise.
     *   retirer : quand la fenetre RX TPU cadence le burst-id et que r_page est
     *             modelise fidelement.
     *
     * [2026-07-30] DEFAUT RESTREINT A shunt_legit — un mecanisme de trop.
     *
     * Deux mecanismes ecrivaient db_r->d_burst_d : celui-ci (chemin ECRITURE) et le
     * desaliasage par FIFO de calypso_trx.c (chemin LECTURE, CALYPSO_BURST_ID_DEALIAS,
     * defaut 1). Le commentaire voisin annoncait deja « deux mecanismes sur la meme
     * cellule, c'est exactement le conflit qu'on vient de defaire » — ils coexistaient
     * toujours.
     *
     * Celui-ci est le mauvais, et c'est mesure : shunt_burst_echo() vaut
     * (g_shunt.d_burst_d + ofs + 4) & 3 avec ofs = -2, or l'ARM ecrit
     * db_w->d_burst_d = 0 dans 355 cas sur 364 (page 0) et 64 sur 79 (page 1).
     * Ces zeros ne sont pas des commandes : c'est le `dsp_api_memset(db_w, ...)` que
     * l1_sync() execute en debut de CHAQUE trame (osmocom-bb layer1/sync.c:244).
     * Commande 0 -> echo 2. Resultat mesure (CLOBBER-WHO-RUN) : d_burst_d FIGE a
     * 0x0002 dans 0x0829 ET 0x083D, rafraichi toutes les 12 trames
     * (fn=110,123,135,...,339). Le firmware exige la sequence 0,1,2,3 pour assembler
     * un bloc CCCH de 4 bursts : il lit toujours 2, rejette 3 rapports sur 4,
     * n'assemble jamais le bloc, ne consomme jamais a_cd -> pas de SI -> read timeout
     * -> L1CTL_RESET_REQ FULL. Le desaliasage par FIFO, lui, derive de la sequence
     * REELLEMENT commandee.
     *
     * On ne le supprime pas franchement parce qu'il PORTE le camp du profil
     * shunt_legit. Il y reste donc actif par defaut ; ailleurs (natif, native_twl)
     * il disparait et le FIFO reste seule source. Le latch g_shunt.d_burst_d, lui,
     * est preserve : 12 consommateurs en dependent.
     */
    static int en = -1;
    if (en < 0) {
        const char *l = getenv("CALYPSO_SHUNT_LEGIT");
        int def = (l && *l == '1') ? 1 : 0;
        en = calypso_gate("CALYPSO_SHUNT_BURST_PERCMD", def);
        fprintf(stderr, "[feed-daram-dsp] SHUNT_BURST_PERCMD=%d (defaut %d : ON sous "
                "shunt_legit, OFF ailleurs) — miroir per-commande de d_burst_d sur les "
                "deux read-pages%s\n", en, def,
                en ? "" : " DESACTIVE : le desaliasage FIFO (CALYPSO_BURST_ID_DEALIAS) "
                          "est seule source");
    }
    if (!en) return;
    if (off != 0x0002 && off != 0x002A) return;   /* WP_D_BURST_D page0/1 */
    g_shunt.d_burst_d = (uint16_t)(value & 3);
    uint16_t x = shunt_burst_echo();
    shunt_write_w(BASE_API_R_PAGE_0 + RP_D_BURST_D, x);
    shunt_write_w(BASE_API_R_PAGE_1 + RP_D_BURST_D, x);
    { static unsigned n = 0, z = 0;
      if ((value & 3) != 0 && n < 40) { n++;
        fprintf(stderr, "[feed-daram-dsp] WP-BURST-NONZERO off=0x%04x cmd=%u -> X=%u insn\n",
                (unsigned)off, value & 3, x); }
      else if ((value & 3) == 0) { z++;
        if (z % 500 == 1) fprintf(stderr, "[feed-daram-dsp] WP-BURST cmd=0 x%u (aucun non-zero: ARM ne commande QUE burst 0)\n", z); } }
}

void calypso_dsp_shunt_feed_si(const uint8_t *l2, int len)
{
    if (!l2 || len <= 0) {
        g_shunt.si_valid = false;
        return;
    }
    /* [2026-07-22] Gate test natif : CALYPSO_SHUNT_FEED_SI=0 coupe l'injection du
     * SI reel dans a_cd -> a_cd ne se remplit QUE si la demod native (corr 0x8d00
     * -> NB) produit vraiment le bloc. Prouve natif vs plomberie. Defaut ON (=1). */
    {
        /* @BEQUILLE — SHUNT_FEED_SI  (CALYPSO_SHUNT_FEED_SI=1, fallback CALYPSO_SHUNT_LEGIT=1)
         *   masque  : la production des blocs SI par la demodulation native (correlateur
         *             0x8d00 -> NB -> a_cd). Le SI vient de gr-gsm, est range par type dans
         *             si_set[0..5], avec fabrication d'un SI6 seed depuis le SI3.
         *   retirer : quand a_cd se remplit par la demodulation native.
         */
        /* [2026-07-30] CONVERTI a calypso_gate : un `=0` EXPLICITE doit couper.
         * Avant, le repli sur SHUNT_LEGIT ECRASAIT le 0 explicite :
         *   fs = (e=='1'); if (!fs) fs = (SHUNT_LEGIT=='1');
         * Consequence mesuree le 30/07 : profil `native_twl` avec FEED_SI=0 au
         * manifeste, et 184 injections `feed_si` quand meme, parce que
         * SHUNT_LEGIT=1 avait fuite dans l'environnement (tmux fossilise l'env du
         * 1er run de la session). Le banc repondait donc « les SI arrivent »
         * alors qu'ils venaient de gr-gsm — exactement la question qu'il posait.
         * calypso_gate(nom, defaut) : la variable, si posee, GAGNE toujours ;
         * le parapluie ne sert plus que de DEFAUT. */
        static int fs = -1;
        if (fs < 0) {
            const char *l = getenv("CALYPSO_SHUNT_LEGIT");
            fs = calypso_gate("CALYPSO_SHUNT_FEED_SI", (l && *l == '1') ? 1 : 0);
        }
        if (!fs) { g_shunt.si_valid = false; return; }
    }
    int n = len < 23 ? len : 23;
    /* (A) range la frame dans le slot de SON type (RR PD=0x06, mt=l2[2]) :
     *   SI1=0x19 SI2=0x1a SI3=0x1b SI4=0x1c SI2bis=0x1d SI2ter=0x1e.
     * shunt_dispatch_allc tourne ensuite sur les slots dispo (set complet). */
    int slot = -1;
    if (n >= 3 && l2[1] == 0x06) {
        switch (l2[2]) {
        case 0x19: slot = 0; break;  /* SI1   */
        case 0x1a: slot = 1; break;  /* SI2   */
        case 0x1b: slot = 2; break;  /* SI3   */
        case 0x1c: slot = 3; break;  /* SI4   */
        case 0x1d: slot = 4; break;  /* SI2bis*/
        case 0x1e: slot = 5; break;  /* SI2ter*/
        default:   break;
        }
    }
    if (slot >= 0) {
        memcpy(g_shunt.si_set[slot], l2, n);
        for (int i = n; i < 23; i++) g_shunt.si_set[slot][i] = 0x2B;
        g_shunt.si_set_have[slot] = true;
    }
    /* SI3 (slot 2) -> SEED SI6 fabrique (B4) pour la SACCH dediee, UNIQUEMENT en
     * fallback tant qu'aucun SI5/SI6 REEL n'est arrive (g_shunt.sacch_real). Des
     * que feed_sacch recoit le vrai SI5/SI6 (grgsm), sacch_real=true et ce bloc
     * ne tourne plus -> le SI3 du BCCH ne clobbe plus le SACCH reel. Le seed evite
     * le 'Short header 0x07 unsupported' au tout debut d'un canal dedie (avant que
     * grgsm ait decode la 1ere SACCH ~480ms). */
    if (slot == 2 && n >= 10 && !g_shunt.sacch_real) {
        uint8_t *s6 = g_shunt.sacch_buf;
        memset(s6, 0x2b, sizeof(g_shunt.sacch_buf));
        /* Layout B4 reel (lapdm.c) : header L1 SACCH (2o, non strippe par la L1
         * osmocom-bb) + LAPDm addr + LAPDm control UI (-> fmt B4, l3len=19) + L3. */
        s6[0] = 0x00;                          /* L1 SACCH : tx_power */
        s6[1] = 0x00;                          /* L1 SACCH : TA */
        s6[2] = 0x03;                          /* LAPDm address : SAPI0, C/R, EA=1 */
        s6[3] = 0x03;                          /* LAPDm control : UI -> format B4 */
        s6[4] = (uint8_t)((11 << 2) | 0x01);   /* L3 pseudo-length L=11 */
        s6[5] = 0x06;                          /* RR PD, skip=0 */
        s6[6] = 0x1e;                          /* SYSTEM INFORMATION TYPE 6 */
        s6[7] = l2[3]; s6[8] = l2[4];          /* cell identity (SI3 @3..4) */
        s6[9]  = l2[5]; s6[10] = l2[6]; s6[11] = l2[7];
        s6[12] = l2[8]; s6[13] = l2[9];        /* LAI (SI3 @5..9) */
        s6[14] = 0x0f;                         /* cell options : radio-link-timeout long */
        s6[15] = 0xff;                         /* NCC permitted : tous */
        /* [16..22] = 0x2b rest octets (l3 total = [4..22] = 19o) */
        g_shunt.sacch_have = true;
    }
    /* compat / fallback : si_buf = dernier reçu */
    memcpy(g_shunt.si_buf, l2, n);
    /* pad fin avec 0x2B (filler LAPDm) si la frame est plus courte */
    for (int i = n; i < 23; i++)
        g_shunt.si_buf[i] = 0x2B;
    g_shunt.si_valid = true;
    /* Hop 5 : injecte AUSSI directement en L1CTL DATA_IND -> mobile (gated
     * CALYPSO_SHUNT_DL_INJECT, defaut ON). FN reelle via calypso_trx_get_fn. */
    {
        /* @BEQUILLE — SHUNT_DL_INJECT  (CALYPSO_SHUNT_DL_INJECT, EQ1, defaut OFF ;
         *              shunt_no_legit.env:=1)
         *   masque  : TOUT le chemin descendant a_cd -> L1 firmware -> UART -> L1CTL. Le SI
         *             est pousse directement en L1CTL_DATA_IND vers le mobile ; aucune
         *             partie de la chaine emulee n'est exercee. C'est la bequille la plus
         *             intrusive du modele.
         *   retirer : des que le SI atteint le mobile via a_cd (deja le cas par defaut :
         *             seul calypso_shunt_no_legit.env la repose a 1).
         */
        static int inj = -1;
        if (inj < 0) { const char *e = getenv("CALYPSO_SHUNT_DL_INJECT");
                       inj = (e && *e == '1') ? 1 : 0; }  /* [2026-07-23] DEFAUT OFF (natif) */
        if (inj) l1ctl_inject_dl_si(g_shunt.si_buf, 23, calypso_trx_get_fn());
    }
    SHUNT_LOG("feed_si: SI réel %d o injecté → a_cd "
            "(L2[0..2]=%02x %02x %02x)\n", n, l2[0],
            n > 1 ? l2[1] : 0, n > 2 ? l2[2] : 0);
}

/* Public getter — gate condition for BSP/TPU DMA into DARAM. */
bool calypso_dsp_shunt_sb_valid(void) { return g_shunt.sb_valid; }
bool calypso_dsp_shunt_si_valid(void) { return g_shunt.si_valid; }
uint16_t calypso_dsp_shunt_burst_d(void) { return shunt_burst_echo(); }  /* d_burst_d gate (OFS/FN) */
bool calypso_dsp_shunt_active(void)
{
    return g_shunt.active;
}

/* [2026-07-27] SPLIT DU GATE — voir en-tete du patch.
 * active()      = l infrastructure shunt est armee (overlay NDB, ponts, feeds).
 *                 VRAI aussi en mode ASSIST (CALYPSO_DSP=c54x).
 * substitutes() = le shunt REMPLACE le DSP (mock ARM) -> et LUI SEUL doit
 *                 gater les c54x_run natifs. FAUX en mode assist, ou le vrai
 *                 DSP execute son firmware et doit tourner a la cadence trame. */
/* @BEQUILLE — DSP_SHUNT (substitution du DSP)  (CALYPSO_DSP_SHUNT, EQ1, ou
 *              CALYPSO_L1=c)
 *   masque  : l'execution du DSP. Quand cette fonction retourne vrai, TOUS les
 *             c54x_run natifs (calypso_trx.c) sont gates : le mock remplace le
 *             processeur de signal.
 *   retirer : quand le correlateur natif produit d_fb_det seul (RANK3 leve).
 */
bool calypso_dsp_shunt_substitutes(void)
{
    if (!g_shunt.active) return false;
    if (calypso_l1_c_active()) return true;      /* L1=c : modele HLE remplace le DSP */
    { const char *e = getenv("CALYPSO_DSP_SHUNT");
      return (e && strcmp(e, "1") == 0); }      /* mock explicite */
}

/* Public getter — mission courante du DSP (d_task_md, lu du write-page ARM).
 * Sert a gater les wires inter-blocs (BSP BRINT0 / TPU DSP_INT_PG) sur la
 * mission FB/SB reelle. Valeurs (osmo l1_environment.h) : FB_DSP_TASK=5,
 * SB_DSP_TASK=6, TCH_FB=8, TCH_SB=9. 0 = pas de tache. */
uint16_t calypso_dsp_shunt_get_task_md(void)
{
    if (g_shunt.d_task_md) return g_shunt.d_task_md;
    /* [2026-07-27] FALLBACK NATIF : hors shunt, g_shunt.d_task_md n est JAMAIS
     * alimente (pose uniquement par shunt_latch_task) -> l accesseur renvoyait 0
     * en permanence et tuait silencieusement tous ses appelants cote natif.
     * On lit alors la cellule API RAM du DSP : d_task_md page0 = 0x0804,
     * page1 = 0x0818. La branche shunt ci-dessus reste prioritaire. */
    if (g_shunt.c54x && g_shunt.c54x->data) {
        uint16_t _a = g_shunt.c54x->data[0x0804];
        if (_a) return _a;
        return g_shunt.c54x->data[0x0818];
    }
    return 0;
}

/* CALYPSO_DSP=c54x : relie le handle du VRAI DSP (depuis calypso_mb.c). */
static bool g_c54x_early_booted = false;
bool calypso_dsp_shunt_early_booted(void) { return g_c54x_early_booted; }

/* [2026-07-27] FB-STREAM : pop la prochaine paire I/Q du ring. false = underrun. */
bool calypso_dsp_shunt_fb_stream_next(uint16_t *outI, uint16_t *outQ)
{
    if (g_fbs_rd + 1 >= g_fbs_wr) return false;
    *outI = (uint16_t)g_fbs[g_fbs_rd++ & (FBS_RING-1)];
    *outQ = (uint16_t)g_fbs[g_fbs_rd++ & (FBS_RING-1)];
    /* [2026-07-27] B2IN (gated CALYPSO_B2IN) : mesure la VRAIE entree corr
     * (0x9213/0x9215 = CE stream), pas la sortie 0x2a00. max|I|/|Q| + energie +
     * indice du max sur 296 -> tranche "entree morte/DC" vs "vrai ton FCCH". */
    {
        static int _b2i = -1; static unsigned _n = 0, _imax = 0, _qmax = 0; static int _iidx = -1;
        static uint64_t _e = 0; static unsigned _wn = 0;
        if (_b2i < 0) _b2i = calypso_gate("CALYPSO_B2IN", 0);
        if (_b2i) {
            int16_t _I = (int16_t)*outI, _Q = (int16_t)*outQ;
            unsigned _ai = _I < 0 ? (unsigned)(-_I) : (unsigned)_I;
            unsigned _aq = _Q < 0 ? (unsigned)(-_Q) : (unsigned)_Q;
            if (_ai > _imax) { _imax = _ai; _iidx = (int)_wn; }
            if (_aq > _qmax) _qmax = _aq;
            _e += (uint64_t)_I * _I + (uint64_t)_Q * _Q;
            if (++_wn >= 296) {
                if (_n++ < 30)
                    fprintf(stderr, "[dsp-shunt] B2IN (0x9213/0x9215) win=296 max|I|=%u@%d max|Q|=%u energy=%llu\n",
                            _imax, _iidx, _qmax, (unsigned long long)_e);
                _wn = 0; _imax = 0; _qmax = 0; _iidx = -1; _e = 0;
            }
        }
    }
    return true;
}

/* [2026-07-27] Reset L1 (Ctrl-C mobile / L1CTL_RESET_REQ FULL) : clear l'etat
 * transitoire du shunt (IMM-ASSIGN / SDCCH-DL latches par un SMS) qui sinon
 * supprime le SI apres la relance -> le mobile ne re-campe pas. Appele depuis
 * calypso_arm2dsp.c quand l'ARM ecrit d_dsp_page=0 (l1s_reset_hw). */
void calypso_dsp_shunt_l1_reset(void)
{
    g_shunt.agch_valid  = false;
    g_shunt.sdcch_valid = false;
    g_shunt.sdcch_ss_set = false;   /* reset -> defaut base 22 (SDCCH/4 SS0) */
    /* PAS de si_rr=0 : d_dsp_page=0 fire aussi sur les mesures (l23_api.c:414) /
     * FBSB -> reset si_rr figerait la rotation SI en no_canned. */
}

void calypso_dsp_shunt_set_c54x(C54xState *s)
{
    g_shunt.c54x = s;

    /* [c54x-earlyboot] FIX race d'ordre golive (2026-07-20, mode B).
     * Root cause : l'ARM poste le golive (data[0x0fff]=cmd 2/4, data[0x0ffe]=entry)
     * a fn=0/+0.073s, MAIS le c54x ne bootait qu'a +5.6s (1er shunt_route_to_c54x)
     * -> son init-IDLE a 0xb419 (ST #1,*0xfff) ecrasait le 0x0002 de l'ARM -> spin
     * eternel a 0xb41c. Etat PERSISTE entre wakes (verifie : 0xb419 ne tourne
     * qu'une fois, insn accumule, meme objet DSP). Fix = booter le c54x ICI
     * (machine-init, AVANT que le vCPU ARM tourne -> AVANT le golive), pour qu'il
     * pose son IDLE et se parke a 0xb41c AVANT l'ecriture ARM. 0xb419 ne re-tourne
     * plus (PC persiste) -> le 0x0002 survit -> le 1er wake shunt le consomme ->
     * golive natif (le firmware fait son propre RSBX INTM). Zero FORCE_ : on force
     * le QUAND du boot, aucune valeur de mailbox. One-shot, gate mode revive. */
    /* [2026-07-27] DECOUPLE du routage shunt : l ordre de boot du c54x ne
     * depend pas de CALYPSO_DSP=c54x. Gate sur CALYPSO_DSP_RUN_C54X seul,
     * sinon le mode natif re-reset le DSP et ecrase la cmd bootloader de
     * l ARM -> spin eternel a 0xb41c (voir en-tete du patch). */
    {
        static int rc = -1;
        if (rc < 0) { const char *e = getenv("CALYPSO_DSP_RUN_C54X"); rc = (e && *e == '1') ? 1 : 0; }
        if (s && rc) {
            uint16_t pc0 = s->pc;
            s->running = true;
            c54x_run(s, 2000);   /* reset(0xff80) -> 0xb419 (pose IDLE) -> park 0xb41c */
            if (s->pc >= 0xb41c && s->pc <= 0xb428) {
                g_c54x_early_booted = true;   /* gate le re-reset trx:701 */
                fprintf(stderr, "[c54x-earlyboot] PARK pc=0x%04x (de 0x%04x) insn=%u "
                        "data[0x0fff]=0x%04x data[0x0ffe]=0x%04x (attendu IDLE 0x0001)\n",
                        s->pc, pc0, s->insn_count, s->data[0x0fff], s->data[0x0ffe]);
            } else
                fprintf(stderr, "[c54x-earlyboot] WARN pas parque pc=0x%04x insn=%u "
                        "-> B invalide, basculer sur A (execution continue)\n",
                        s->pc, s->insn_count);
        }
    }
}

/* Predicat dedie : shunt actif ET route c54x demandee. Utilise par
 * calypso_trx.c pour autoriser la DMA page->DARAM en mode c54x sans
 * reactiver le c54x_run du trx (le shunt possede c54x_run). */
bool calypso_dsp_shunt_route_c54x_active(void)
{
    return g_shunt.active && shunt_route_c54x();
}
