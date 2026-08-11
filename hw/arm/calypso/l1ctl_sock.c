/*
 * l1ctl_sock.c — L1CTL unix socket server (legacy QEMU-internal path)
 *
 * État runtime actuel (2026-05-25) : ce socket est INACTIF dans le run
 * orchestré par scripts/run.sh. run.sh:458 override l'env L1CTL_SOCK vers
 * /tmp/qemu_l1ctl_disabled pour le child QEMU, donc ce module crée son
 * socket à une adresse-poubelle et personne ne s'y connecte. Le VRAI
 * socket /tmp/osmocom_l2 que le mobile osmocom-bb utilise est créé par
 * osmocon (-m romload -s /tmp/osmocom_l2), pas par QEMU.
 *
 * Le path historique « Replaces the Python bridge » reste possible si on
 * lance QEMU sans override env — utile pour des tests sans osmocon, mais
 * pas le mode de fonctionnement principal. Voir doc/L1CTL_SOCK_FLOW.md
 * et le commentaire à run.sh:458.
 *
 * Quand actif : provides a unix socket at /tmp/osmocom_l2 that speaks
 * L1CTL (length-prefixed messages) to OsmocomBB mobile.
 *
 * Internally translates between:
 *   - sercomm framing (FLAG/ESCAPE/DLCI) on the firmware UART side
 *   - L1CTL length-prefix on the mobile socket side
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "hw/arm/calypso/calypso_uart.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <errno.h>

/* Sercomm constants */
#define SERCOMM_FLAG       0x7E
#define SERCOMM_ESCAPE     0x7D
#define SERCOMM_ESCAPE_XOR 0x20
#define SERCOMM_DLCI_L1CTL 5

/* L1CTL socket path */
#define L1CTL_SOCK_PATH    "/tmp/osmocom_l2"

#define L1CTL_LOG(fmt, ...) \
    fprintf(stderr, "[l1ctl-sock] " fmt "\n", ##__VA_ARGS__)

/* Nom lisible des types L1CTL (l1ctl_proto.h) — diagnostic pur pour suivre la
 * conversation firmware↔mobile à l'œil. NB : ce mobile cause par osmocon/hdlc
 * (serial), pas par ce socket unix ; ce log ne voit que le sens firmware→mobile
 * via sercomm. Le vrai flux mobile↔firmware se lit dans osmocon.log (hdlc). */
static inline const char *l1ctl_tname(uint8_t t)
{
    switch (t) {
    case 0x01: return "FBSB_REQ";       case 0x02: return "FBSB_CONF";
    case 0x03: return "DATA_IND";       case 0x04: return "RACH_REQ";
    case 0x05: return "DM_EST_REQ";     case 0x06: return "DATA_REQ";
    case 0x07: return "RESET_IND";      case 0x08: return "PM_REQ";
    case 0x09: return "PM_CONF";        case 0x0c: return "RACH_CONF";
    case 0x0d: return "RESET_REQ";      case 0x0e: return "RESET_CONF";
    case 0x0f: return "DATA_CONF";      case 0x10: return "CCCH_MODE_REQ";
    case 0x11: return "CCCH_MODE_CONF"; case 0x12: return "DM_REL_REQ";
    case 0x13: return "PARAM_REQ";      default:   return "?";
    }
}

/* ---- Sercomm TX parser (firmware → mobile) ---- */

typedef enum {
    SC_IDLE,      /* waiting for FLAG */
    SC_IN_FRAME,  /* collecting frame bytes */
    SC_ESCAPE,    /* next byte is escaped */
} SercommState;

typedef struct L1CTLSock {
    /* Server socket */
    int srv_fd;

    /* Client connection */
    int cli_fd;

    /* Sercomm TX parser (firmware UART output → mobile) */
    SercommState sc_state;
    uint8_t  sc_buf[512];
    int      sc_len;

    /* L1CTL RX parser (mobile → firmware UART input) */
    uint8_t  lp_buf[4096];  /* length-prefix accumulator */
    int      lp_len;

    /* Reference to UART modem for RX injection */
    CalypsoUARTState *uart;
} L1CTLSock;

static L1CTLSock g_l1ctl;

/* FN-FIX : le FN que le firmware envoie au mobile dans L1CTL_RACH_CONF (msg type
 * 0x0c), capture ICI au moment EXACT ou le mobile le recoit (= ce qu'il memorise
 * pour matcher la req-ref de l'IMM ASSIGN, gsm48_rr.c:3372). Lu par le shunt
 * (calypso_dsp_shunt.c) pour reecrire la req-ref. Source race-free : pas de lecture
 * paresseuse de last_rach.fn @0x836500 (qui est asynchrone vs l'IMM ASS du BTS). */
volatile uint32_t g_last_rach_conf_fn = 0;
volatile uint32_t g_rach_conf_fn[256] = {0};  /* per-ra FN-FIX : RACH_CONF fn keye par g_last_recorded_ra */
extern volatile uint8_t g_last_recorded_ra;   /* defini dans calypso_dsp_shunt.c (record_rach) */
extern void calypso_dsp_shunt_set_dcch(int kind, int ss);  /* fenetre SDCCH du shunt */
extern void calypso_dsp_shunt_set_dcch_tch(int on);        /* dedie = TCH ? */
extern void calypso_dsp_shunt_set_dcch_active(int on);     /* garde SI pendant le dedie */
extern int calypso_dsp_shunt_tch_dl_written(const uint8_t *fr33); /* sonde TCH-DL */

/* ---- SONDE CALYPSO_TCH_DL_PROBE : les octets FR qui partent VRAIMENT ---------
 *
 * Confronte les 33 octets de voix d'un TRAFFIC_IND a l'anneau des trames que le
 * shunt a reellement ecrites dans a_dd_0 (cf. le commentaire de la sonde dans
 * calypso_dsp_shunt.c). Repond a UNE question : le firmware relaie-t-il ce qu'on
 * lui donne, ou autre chose ?
 *
 * Cadrage : TRAFFIC_IND = l1ctl_hdr(4) + l1ctl_info_dl(12) + data(33) = 49, ce
 * que confirme le `len=49` deja journalise. On ne touche a rien si la taille
 * n'est pas celle-la — une sonde qui devine son cadrage ne prouve rien.
 *
 * Compteurs CUMULATIFS (jamais un taux : cf. les deux chiffres faux annonces le
 * 09/08 en divisant des `grep -c` sur un journal bufferise). */
static void l1ctl_tch_dl_probe(const uint8_t *payload, int plen)
{
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("CALYPSO_TCH_DL_PROBE");
        on = (e && *e == '1') ? 1 : 0;
        /* Une sonde muette est indecidable : elle s'annonce, dans les deux sens. */
        L1CTL_LOG("SONDE TCH-DL-PROBE %s (CALYPSO_TCH_DL_PROBE)",
                  on ? "ACTIVE" : "inactive");
    }
    if (!on || payload[0] != 0x1e /* L1CTL_TRAFFIC_IND */)
        return;

    static unsigned long long n = 0, ok = 0, bad = 0, mauvais_cadrage = 0;
    /* [2026-08-10] L'EN-TETE DECIDE DU SORT DE LA TRAME, PAS SEULEMENT SON CONTENU.
     * La chaine de lecture du mobile est 'source/tch_fb -> ... -> ecu/fr -> codec/fr
     * -> sink/alsa' : l'ECU remplace toute trame marquee ERRONEE par du confort,
     * c'est-a-dire du silence -- sans une ligne de journal. On observe justement
     * 50 TRAFFIC_IND/s dont les 33 octets sont IDENTIQUES a a_dd_0 (donc de la
     * vraie parole, 250/250 trames distinctes mesurees dans l'anneau) et un sink
     * PulseAudio a crete 0. Il faut donc lire fire_crc et num_biterr, qui portent
     * ce verdict. Cadrage l1ctl_info_dl (offset relatif a payload+4) :
     *   chan_nr(0) link_id(1) band_arfcn(2..3) frame_nr(4..7)
     *   rx_level(8) snr(9) num_biterr(10) fire_crc(11) */
    static unsigned long long fire_crc_nz = 0, biterr_nz = 0;
    n++;
    if (plen != 49) {
        mauvais_cadrage++;
        if (mauvais_cadrage <= 3)
            L1CTL_LOG("TCH-DL-PROBE : TRAFFIC_IND de %d o, attendu 49 "
                      "(4 hdr + 12 info_dl + 33 FR) -- cadrage a reverifier "
                      "avant toute conclusion", plen);
    } else {
        const uint8_t *fr = payload + 16;
        uint8_t nbiterr = payload[14], fcrc = payload[15];
        if (fcrc)    fire_crc_nz++;
        if (nbiterr) biterr_nz++;
        if ((fcrc || nbiterr) && (fire_crc_nz + biterr_nz) <= 3)
            L1CTL_LOG("TCH-DL-PROBE : trame MARQUEE ERRONEE -- fire_crc=%u "
                      "num_biterr=%u rx_level=%u snr=%u : l'ECU du mobile la "
                      "remplacera par du silence", fcrc, nbiterr,
                      payload[12], payload[13]);
        int seq = calypso_dsp_shunt_tch_dl_written(fr);
        if (seq >= 0) {
            ok++;
        } else {
            bad++;
            if (bad <= 5) {
                char h[64];
                int p = 0;
                for (int i = 0; i < 12; i++)
                    p += snprintf(h + p, sizeof(h) - p, "%02x ", fr[i]);
                L1CTL_LOG("TCH-DL-PROBE ECART #%llu : sortant sig=0x%x [%s...] ne "
                          "correspond a AUCUNE des 8 dernieres trames ecrites "
                          "dans a_dd_0", bad, fr[0] >> 4, h);
            }
        }
    }
    if ((n % 250) == 0)
        L1CTL_LOG("TCH-DL-PROBE : %llu TRAFFIC_IND -- identiques a a_dd_0 : %llu, "
                  "differentes : %llu, cadrage inattendu : %llu, "
                  "MARQUEES ERRONEES : fire_crc!=0 %llu, num_biterr!=0 %llu",
                  n, ok, bad, mauvais_cadrage, fire_crc_nz, biterr_nz);
}

/* ---- Sercomm helpers ---- */

static int sercomm_wrap(uint8_t dlci, const uint8_t *payload, int plen,
                        uint8_t *out, int out_size)
{
    int pos = 0;
    if (pos >= out_size) return -1;
    out[pos++] = SERCOMM_FLAG;

    /* DLCI + CTRL */
    uint8_t hdr[2] = { dlci, 0x03 };
    for (int i = 0; i < 2; i++) {
        if (hdr[i] == SERCOMM_FLAG || hdr[i] == SERCOMM_ESCAPE) {
            if (pos + 2 > out_size) return -1;
            out[pos++] = SERCOMM_ESCAPE;
            out[pos++] = hdr[i] ^ SERCOMM_ESCAPE_XOR;
        } else {
            if (pos + 1 > out_size) return -1;
            out[pos++] = hdr[i];
        }
    }

    /* Payload */
    for (int i = 0; i < plen; i++) {
        if (payload[i] == SERCOMM_FLAG || payload[i] == SERCOMM_ESCAPE) {
            if (pos + 2 > out_size) return -1;
            out[pos++] = SERCOMM_ESCAPE;
            out[pos++] = payload[i] ^ SERCOMM_ESCAPE_XOR;
        } else {
            if (pos + 1 > out_size) return -1;
            out[pos++] = payload[i];
        }
    }

    if (pos >= out_size) return -1;
    out[pos++] = SERCOMM_FLAG;
    return pos;
}

/* ---- Send L1CTL message to mobile (length-prefix) ---- */

static void l1ctl_send_to_mobile(L1CTLSock *s, const uint8_t *payload, int len)
{
    if (s->cli_fd < 0 || len <= 0 || len > UINT16_MAX) return;

    uint8_t hdr[2] = { (uint8_t)(len >> 8), (uint8_t)(len & 0xFF) };
    struct iovec iov[2] = {
        { .iov_base = hdr,                  .iov_len = sizeof(hdr) },
        { .iov_base = (void *)payload,      .iov_len = (size_t)len },
    };
    struct msghdr msg = { .msg_iov = iov, .msg_iovlen = 2 };

    int total = (int)sizeof(hdr) + len;
    ssize_t sent = sendmsg(s->cli_fd, &msg, MSG_NOSIGNAL);
    if (sent != total) {
        L1CTL_LOG("client send error (%zd/%d), closing", sent, total);
        close(s->cli_fd);
        s->cli_fd = -1;
    }
}

/* Hop 5 : injection directe DL SI -> mobile en L1CTL DATA_IND (court-circuite
 * a_cd->ARM->UART qui perd des octets). Appele par le shunt GSMTAP listener. */
void l1ctl_inject_dl_si(const uint8_t *l2, int l2len, uint32_t fn)
{
    if (g_l1ctl.cli_fd < 0 || !l2 || l2len <= 0) return;
    if (l2len > 23) l2len = 23;
    uint8_t pl[16 + 23];
    memset(pl, 0, sizeof(pl));
    pl[0] = 0x03;                                  /* L1CTL_DATA_IND */
    pl[4] = 0x80;                                  /* chan_nr = BCCH */
    pl[6] = (uint8_t)(514 >> 8); pl[7] = (uint8_t)(514 & 0xFF);  /* band_arfcn 514 */
    pl[8]=(uint8_t)(fn>>24); pl[9]=(uint8_t)(fn>>16);
    pl[10]=(uint8_t)(fn>>8);  pl[11]=(uint8_t)fn;  /* frame_nr (BE) */
    pl[12] = 40;                                   /* rx_level */
    pl[13] = 30;                                   /* snr */
    /* pl[14]=num_biterr=0, pl[15]=fire_crc=0 (CRC OK) */
    memcpy(pl + 16, l2, l2len);
    l1ctl_send_to_mobile(&g_l1ctl, pl, 16 + l2len);
    L1CTL_LOG("INJECT DL DATA_IND BCCH fn=%u l2len=%d -> mobile", fn, l2len);
}

/* ---- Process a complete sercomm frame from firmware TX ---- */

static void sercomm_frame_complete(L1CTLSock *s)
{
    if (s->sc_len < 2) return;  /* need at least DLCI + CTRL */

    uint8_t dlci = s->sc_buf[0];
    /* uint8_t ctrl = s->sc_buf[1]; */
    uint8_t *payload = &s->sc_buf[2];
    int plen = s->sc_len - 2;

    if (dlci == SERCOMM_DLCI_L1CTL && plen > 0) {
        /* ===== GATES de déblocage (oracle FORCE_TOA, gate-par-gate) =====
         * Le mobile reçoit par CE socket (mobile.cfg: layer2-socket
         * /tmp/osmocom_l2). Deux gates bridgent les trous du demod DSP, pour
         * prouver que tout l'aval (camp/SI/IMM-ASS) marche quand le DSP fournit
         * son résultat. Purement oracle — à retirer quand le DSP demod marche.
         *   CALYPSO_FORCE_FBSB=1 : bridge blocker #1 (Channel sync error) =
         *                          FBSB_CONF(0x02) result@[18] → 0=SUCCESS.
         *   CALYPSO_FORCE_AGCH=1 : bridge blocker #2 (pas de sysinfo) sur les
         *                          DATA_IND(0x03) : BCCH(chan 0x80) rote le type
         *                          SI ; AGCH/PCH(chan 0x90) injecte IMM ASS.
         *                          Port exact du GDB mutate_agch.
         * Layout payload : l1ctl_hdr(4) + l1ctl_info_dl(12) + corps ;
         * → FBSB result @18 ; DATA_IND chan_nr @4, L3 @16. */
        /* @BEQUILLE — FORCE_FBSB / FORCE_AGCH  (CALYPSO_FORCE_FBSB, CALYPSO_FORCE_AGCH,
         *              EQ1, defaut 0 ; VERROUILLES a 0 par run.sh en mode full-grgsm)
         *   masque  : le resultat du demod DSP vu par le mobile. FBSB : force le resultat
         *             de FBSB_CONF a SUCCESS. AGCH : rote le type SI du BCCH et ECRASE le
         *             L3 du PCH par un IMM ASSIGNMENT en dur.
         *   retirer : quand le demod DSP publie un a_cd valide (SI reels decodes).
         *   NB      : assignation dure ("=0" puis export) en full-grgsm -> les poser en
         *             ligne de commande n'a AUCUN effet dans le mode par defaut.
         */
        static int g_fbsb = -1, g_agch = -1;
        if (g_fbsb < 0) {
            const char *a = getenv("CALYPSO_FORCE_FBSB");
            const char *b = getenv("CALYPSO_FORCE_AGCH");
            g_fbsb = (a && *a == '1') ? 1 : 0;
            g_agch = (b && *b == '1') ? 1 : 0;
        }
        if (g_fbsb && payload[0] == 0x02 && plen >= 19 && payload[18] != 0) {
            L1CTL_LOG("GATE-FBSB #1: FBSB_CONF result 0x%02x → 0", payload[18]);
            payload[18] = 0;
        }
        if (g_agch && payload[0] == 0x03 && plen >= 16 + 3) {
            uint8_t chan_nr = payload[4];
            uint8_t *l3 = &payload[16];
            if (chan_nr == 0x80) {
                static const uint8_t si[4] = { 0x19, 0x1a, 0x1b, 0x1c };
                static int r = 0;
                l3[2] = si[r]; r = (r + 1) & 3;
                L1CTL_LOG("GATE-AGCH #2 bcch: SI type → 0x%02x", l3[2]);
            } else if (chan_nr == 0x90 && plen >= 16 + 23) {
                static const uint8_t imm[23] = {
                    0x2d, 0x06, 0x3f, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
                    0x2b, 0x2b, 0x2b };
                memcpy(l3, imm, sizeof(imm));
                L1CTL_LOG("GATE-AGCH #2 pch: IMM ASSIGNMENT injecté");
            }
        }
        /* FN-FIX : capture le FN du RACH_CONF (0x0c) = le FN que le mobile memorise.
         * Layout : l1ctl_hdr(4) + l1ctl_info_dl ; frame_nr (BE) @ payload[8..11]. */
        if (payload[0] == 0x0c && plen >= 12) {
            g_last_rach_conf_fn = ((uint32_t)payload[8] << 24) | ((uint32_t)payload[9] << 16) |
                                  ((uint32_t)payload[10] << 8) | (uint32_t)payload[11];
            g_rach_conf_fn[g_last_recorded_ra] = g_last_rach_conf_fn;   /* per-ra : keye par le ra de la derniere RACH */
            L1CTL_LOG("FN-FIX: RACH_CONF fn=%u capture (memo mobile, ra=0x%02x)", g_last_rach_conf_fn, g_last_recorded_ra);
        }
        /* ═══════════════════════════════════════════════════════════════════
         * CANAL DEDIE COURANT -> /dev/shm/calypso_dcch_cfg  (2026-08-08)
         *
         * OU LE LIRE. Premiere tentative : depuis les IMM ASSIGN du CCCH, cote
         * si_bridge. FAUX — le CCCH porte ceux de TOUS les abonnes (68 de
         * RA=0x07, 12 de RA=0x0a pour un RACH a nous de RA=0x08) : la sous-voie
         * active sautait 60 fois par run. Deuxieme tentative : DM_EST_REQ dans
         * l1ctl_client_readable. FAUX AUSSI, et pour une raison structurelle
         * documentee en tete de ce fichier : ce socket est INACTIF, osmocon
         * detient /tmp/osmocom_l2 et relaie par le pty. Mesure : `RX←mobile` = 0
         * occurrence sur tout le journal, alors qu'osmocon voit bien 6 DM_EST.
         *
         * ICI, en revanche, on est dans le sens firmware->mobile, qui est le
         * SEUL flux L1CTL que QEMU parse reellement. DATA_CONF (0x0f) et
         * DATA_IND (0x03) portent l1ctl_info_dl.chan_nr en payload[4], rempli
         * par le firmware depuis SON ordonnanceur mframe : c'est donc bien le
         * canal que NOTRE mobile utilise, pas celui d'un voisin.
         *
         * chan_nr (GSM 08.58 9.3.1) : 001SSTTT = SDCCH/4, 01SSSTTT = SDCCH/8.
         * BCCH (0x80) / CCCH (0x90) / TCH (00001TTT) sont ignores ici.
         * ═══════════════════════════════════════════════════════════════════ */
        if ((payload[0] == 0x0f || payload[0] == 0x03) && plen >= 5) {
            uint8_t chan_nr = payload[4];
            int kind = -1, ss = 0;
            if ((chan_nr & 0xE0) == 0x20)      { kind = 0; ss = (chan_nr >> 3) & 0x03; }
            else if ((chan_nr & 0xC0) == 0x40) { kind = 1; ss = (chan_nr >> 3) & 0x07; }
            static uint8_t last_chan_nr = 0xFF;
            /* [2026-08-09] FRONT DE LIBERATION DU DEDIE, dans le sens que QEMU
             * parse REELLEMENT. Premiere tentative : accrocher DM_REL_REQ (0x12)
             * dans le bloc mobile->firmware plus bas. C'est du CODE MORT ici :
             * le socket l1ctl de QEMU est orphelin (le mobile parle a osmocon),
             * mesure « RX←mobile » = 0 occurrence sur tout le journal. Ce meme
             * bloc porte aussi la remise a zero du Kc a chaque DM_EST/DM_REL —
             * elle ne s'execute donc jamais non plus, a verifier avant d'activer
             * l'A5/1.
             * Ici on est dans DATA_CONF/DATA_IND, qui EST parse : quand chan_nr
             * repasse sur du non-dedie (BCCH 0x80 / CCCH 0x90), le canal est
             * termine et la garde SI doit se lever. Sans ce front, seule la
             * peremption de 60 s la libere, et le camp reste prive de SI. */
            /* [2026-08-09] REMANENCE, PAS UN FRONT. Version precedente : lever la
             * garde des qu un chan_nr non-dedie passait. Mesure : 121 armements
             * et 121 levees pour 2 canaux dedies — parce qu en dedie le mobile
             * lit AUSSI les BCCH voisines pour ses mesures, donc chan_nr bascule
             * sans arret. La garde clignotait et le camp reprenait la main entre
             * deux blocs. On rafraichit donc sur chaque bloc DEDIE et on laisse
             * la peremption faire la fermeture. */
            /* [2026-08-09] LE DEDIE NE SE RESUME PAS AU SDCCH.
             * `kind` ne vaut >= 0 que pour SDCCH/4 et SDCCH/8 : il sert a
             * calculer la fenetre de presentation a_cd, qui n a de sens que la.
             * Mais la GARDE, elle, doit tenir sur tout canal dedie -- TCH/F et
             * TCH/H compris, dont le SACCH passe aussi par a_cd.
             * Sans ca, pendant un appel voix kind restait -1 en permanence, la
             * garde n etait jamais rafraichie, elle perimait au bout de 2 s et le
             * camp reecrivait son SI dans a_cd. Mesure du 09/08 : 0 armement
             * journalise, 121 peremptions, et 8 « Short header message type 0x07
             * unsupported » en rafale reguliere PENDANT la communication.
             * Codage GSM 08.58 du chan_nr (bits 7..3) :
             *   00001TTT TCH/F | 0001xTTT TCH/H | 001..... SDCCH/4 | 01...... SDCCH/8 */
            bool dedie = ((chan_nr & 0xF8) == 0x08)      /* TCH/F   */
                      || ((chan_nr & 0xF0) == 0x10)      /* TCH/H   */
                      || ((chan_nr & 0xE0) == 0x20)      /* SDCCH/4 */
                      || ((chan_nr & 0xC0) == 0x40);     /* SDCCH/8 */
            if (dedie) calypso_dsp_shunt_set_dcch_active(1);   /* rafraichit */
            /* [2026-08-10] et on dit AU SHUNT de quel type de dedie il s'agit :
             * la fenetre de presentation a_cd n'a de sens que sur SDCCH. */
            if (dedie) {
                bool tch = ((chan_nr & 0xF8) == 0x08)      /* TCH/F */
                        || ((chan_nr & 0xF0) == 0x10);     /* TCH/H */
                calypso_dsp_shunt_set_dcch_tch(tch ? 1 : 0);
            }
            if (kind >= 0 && chan_nr != last_chan_nr) {
                static uint32_t dcch_seq;
                last_chan_nr = chan_nr;
                dcch_seq++;
                uint8_t b[16];
                memset(b, 0, sizeof(b));
                memcpy(b, &dcch_seq, 4);
                b[4] = (uint8_t)kind; b[5] = (uint8_t)ss;
                b[6] = chan_nr & 0x07; b[7] = chan_nr;
                int dfd = open("/dev/shm/calypso_dcch_cfg",
                               O_WRONLY | O_CREAT | O_TRUNC, 0666);
                if (dfd >= 0) {
                    if (write(dfd, b, sizeof(b)) < 0) { /* ignore */ }
                    close(dfd);
                }
                L1CTL_LOG("DCCH #%u : chan_nr=0x%02x -> SDCCH/%d SS=%d TN=%u "
                          "(vu sur %s)", dcch_seq, chan_nr, kind ? 8 : 4, ss,
                          chan_nr & 0x07, l1ctl_tname(payload[0]));
                /* La MEME verite pilote la fenetre de presentation a_cd du shunt,
                 * qui suivait jusqu'ici les IMM ASSIGN des autres abonnes. */
                calypso_dsp_shunt_set_dcch(kind, ss);
            }
        }
        L1CTL_LOG("TX→mobile: dlci=%d len=%d type=0x%02x %s", dlci, plen, payload[0],
                  l1ctl_tname(payload[0]));
        l1ctl_tch_dl_probe(payload, plen);
        l1ctl_send_to_mobile(s, payload, plen);
    }
    /* Ignore other DLCIs (debug console, loader, etc.) */
}

/* ---- Feed firmware UART TX bytes into sercomm parser ---- */

void l1ctl_sock_uart_tx_byte(uint8_t byte)
{
    L1CTLSock *s = &g_l1ctl;

    switch (s->sc_state) {
    case SC_IDLE:
        if (byte == SERCOMM_FLAG) {
            s->sc_state = SC_IN_FRAME;
            s->sc_len = 0;
        }
        break;

    case SC_IN_FRAME:
        if (byte == SERCOMM_FLAG) {
            if (s->sc_len > 0) {
                sercomm_frame_complete(s);
            }
            /* Stay in IN_FRAME for next frame */
            s->sc_len = 0;
        } else if (byte == SERCOMM_ESCAPE) {
            s->sc_state = SC_ESCAPE;
        } else {
            if (s->sc_len < (int)sizeof(s->sc_buf)) {
                s->sc_buf[s->sc_len++] = byte;
            }
        }
        break;

    case SC_ESCAPE:
        if (s->sc_len < (int)sizeof(s->sc_buf)) {
            s->sc_buf[s->sc_len++] = byte ^ SERCOMM_ESCAPE_XOR;
        }
        s->sc_state = SC_IN_FRAME;
        break;
    }
}

/* ---- Receive L1CTL from mobile, inject into firmware UART RX ---- */

static void l1ctl_client_readable(void *opaque)
{
    L1CTLSock *s = (L1CTLSock *)opaque;

    uint8_t tmp[4096];
    ssize_t n = recv(s->cli_fd, tmp, sizeof(tmp), MSG_DONTWAIT);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return;  /* no data available yet */
        L1CTL_LOG("client recv error: %s", strerror(errno));
        qemu_set_fd_handler(s->cli_fd, NULL, NULL, NULL);
        close(s->cli_fd);
        s->cli_fd = -1;
        s->lp_len = 0;
        return;
    }
    if (n == 0) {
        L1CTL_LOG("client disconnected");
        qemu_set_fd_handler(s->cli_fd, NULL, NULL, NULL);
        close(s->cli_fd);
        s->cli_fd = -1;
        s->lp_len = 0;
        return;
    }

    /* Accumulate in length-prefix buffer */
    if (s->lp_len + (int)n > (int)sizeof(s->lp_buf)) {
        s->lp_len = 0;  /* overflow, reset */
    }
    memcpy(&s->lp_buf[s->lp_len], tmp, n);
    s->lp_len += (int)n;

    /* Parse complete L1CTL messages */
    while (s->lp_len >= 2) {
        int msglen = (s->lp_buf[0] << 8) | s->lp_buf[1];
        if (s->lp_len < 2 + msglen) break;  /* incomplete */

        uint8_t *payload = &s->lp_buf[2];

        /* === CAPTURE Kc (chiffrement A5) : L1CTL_CRYPTO_REQ (0x15) mobile->fw ===
         * payload : [0]=0x15 [1]flags [2..3]pad [4]chan_nr [5]link_id [6..7]pad
         * [8]algo [9]key_len [10..]Kc. On ecrit /dev/shm/calypso_kc (seq,algo,
         * key_len,Kc) -> l'ipc-device chiffre l'UL (osmo_a5) et si_bridge relance
         * grgsm -k pour dechiffrer le DL. Le Kc capture = celui derive par le
         * mobile (A8) = exactement celui du reseau. */
        if (payload[0] == 0x15 && msglen >= 10) {
            uint8_t algo = payload[8];
            uint8_t klen = payload[9];
            if (klen > 16) klen = 16;
            /* [2026-08-08] GARDE SUR algo, parite avec l'ecrivain VIVANT
             * (osmocon.c:1300). Ce chemin-ci est mort (osmocon detient
             * /tmp/osmocom_l2 ; « RX<-mobile » = 0 occurrence mesuree), mais il
             * ecrivait un seq NON NUL meme pour algo=0/klen=0 : un lecteur y
             * verrait un Kc « present » et chiffrerait avec une cle nulle. Fusil
             * charge pose sur la table — on met la securite. */
            if (algo >= 1 && algo <= 3 && 10 + (int)klen <= msglen) {
                static uint32_t kc_seq = 0;
                uint8_t kbuf[32];
                memset(kbuf, 0, sizeof(kbuf));
                kc_seq++;
                memcpy(kbuf, &kc_seq, 4);              /* [0..3] seq (LE) */
                kbuf[4] = algo; kbuf[5] = klen;        /* [4]algo [5]key_len */
                memcpy(kbuf + 6, &payload[10], klen);  /* [6..] Kc */
                int kfd = open("/dev/shm/calypso_kc",
                               O_WRONLY | O_CREAT | O_TRUNC, 0666);
                if (kfd >= 0) {
                    if (write(kfd, kbuf, sizeof(kbuf)) < 0) { /* ignore */ }
                    close(kfd);
                }
                L1CTL_LOG("CRYPTO_REQ: algo=%u klen=%u "
                          "Kc=%02x%02x%02x%02x%02x%02x%02x%02x -> "
                          "/dev/shm/calypso_kc#%u", algo, klen,
                          payload[10], payload[11], payload[12], payload[13],
                          payload[14], payload[15], payload[16], payload[17],
                          kc_seq);
            }
        }
        /* Reset cipher a l'etablissement/liberation du canal dedie : chaque
         * nouveau canal demarre EN CLAIR jusqu'a son propre CIPHER MODE COMMAND
         * (sinon un Kc perime chiffrerait la SABM du canal suivant). */
        if (payload[0] == 0x05 || payload[0] == 0x12) {   /* DM_EST_REQ / DM_REL_REQ */
            /* ⚠️ CE BLOC EST MORT dans la configuration actuelle : « RX←mobile »
             * ne compte 0 occurrence, le socket l1ctl de QEMU etant orphelin (le
             * mobile parle a osmocon). La remise a zero du Kc ci-dessous ne
             * s'execute donc JAMAIS — a verifier avant d'activer l'A5/1. La garde
             * SI du dedie est branchee plus haut, sur DATA_CONF/DATA_IND. */
            int kfd = open("/dev/shm/calypso_kc",
                           O_WRONLY | O_CREAT | O_TRUNC, 0666);
            if (kfd >= 0) {
                uint8_t z[32]; memset(z, 0, sizeof(z));
                if (write(kfd, z, sizeof(z)) < 0) { /* ignore */ }
                close(kfd);
            }

        }

        /* Wrap in sercomm and inject into UART RX */
        uint8_t frame[1024];
        int flen = sercomm_wrap(SERCOMM_DLCI_L1CTL, payload, msglen,
                                frame, sizeof(frame));
        if (flen > 0 && s->uart) {
            L1CTL_LOG("RX←mobile: len=%d type=0x%02x %s → sercomm %d bytes",
                      msglen, payload[0], l1ctl_tname(payload[0]), flen);
            /* Hex dump of sercomm frame being injected */
            {
                fprintf(stderr, "[l1ctl-sock] INJECT %d bytes:", flen);
                for (int j = 0; j < flen && j < 32; j++)
                    fprintf(stderr, " %02x", frame[j]);
                if (flen > 32) fprintf(stderr, " ...");
                fprintf(stderr, "\n");
            }
            calypso_uart_receive(s->uart, frame, flen);
        }

        /* Consume from buffer */
        int consumed = 2 + msglen;
        memmove(s->lp_buf, &s->lp_buf[consumed], s->lp_len - consumed);
        s->lp_len -= consumed;
    }
}

/* ---- Accept new client connection ---- */

static void l1ctl_accept_cb(void *opaque)
{
    L1CTLSock *s = (L1CTLSock *)opaque;

    int fd = accept(s->srv_fd, NULL, NULL);
    if (fd < 0) return;

    /* Only one client at a time */
    if (s->cli_fd >= 0) {
        L1CTL_LOG("replacing existing client");
        qemu_set_fd_handler(s->cli_fd, NULL, NULL, NULL);
        close(s->cli_fd);
    }

    /* Set non-blocking */
    int flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    s->cli_fd = fd;
    s->lp_len = 0;
    s->sc_state = SC_IDLE;
    s->sc_len = 0;

    qemu_set_fd_handler(fd, l1ctl_client_readable, NULL, s);
    L1CTL_LOG("client connected (fd=%d)", fd);
}

/* ---- Init ---- */

void l1ctl_sock_init(CalypsoUARTState *uart, const char *path)
{
    L1CTLSock *s = &g_l1ctl;
    memset(s, 0, sizeof(*s));
    s->srv_fd = -1;
    s->cli_fd = -1;
    s->uart = uart;

    if (!path) path = L1CTL_SOCK_PATH;

    /* Remove stale socket */
    unlink(path);

    /* Create unix socket server */
    s->srv_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (s->srv_fd < 0) {
        L1CTL_LOG("ERROR: socket(): %s", strerror(errno));
        return;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind(s->srv_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        L1CTL_LOG("ERROR: bind(%s): %s", path, strerror(errno));
        close(s->srv_fd);
        s->srv_fd = -1;
        return;
    }

    if (listen(s->srv_fd, 1) < 0) {
        L1CTL_LOG("ERROR: listen(): %s", strerror(errno));
        close(s->srv_fd);
        s->srv_fd = -1;
        return;
    }

    /* Set non-blocking */
    int flags = fcntl(s->srv_fd, F_GETFL);
    fcntl(s->srv_fd, F_SETFL, flags | O_NONBLOCK);

    qemu_set_fd_handler(s->srv_fd, l1ctl_accept_cb, NULL, s);
    L1CTL_LOG("listening on %s", path);
}

/* ---- Manual poll (called from TDMA tick) ---- */

void l1ctl_sock_poll(void)
{
    L1CTLSock *s = &g_l1ctl;

    /* Try to accept a pending client */
    if (s->srv_fd >= 0 && s->cli_fd < 0) {
        int fd = accept(s->srv_fd, NULL, NULL);
        if (fd >= 0) {
            int flags = fcntl(fd, F_GETFL);
            fcntl(fd, F_SETFL, flags | O_NONBLOCK);
            s->cli_fd = fd;
            s->lp_len = 0;
            s->sc_state = SC_IDLE;
            s->sc_len = 0;
            qemu_set_fd_handler(fd, l1ctl_client_readable, NULL, s);
            L1CTL_LOG("client connected via poll (fd=%d)", fd);
        }
    }

    /* Try to read from connected client */
    if (s->cli_fd >= 0) {
        l1ctl_client_readable(s);
    }
}

bool l1ctl_client_active(void)
{
    return g_l1ctl.cli_fd >= 0;
}
