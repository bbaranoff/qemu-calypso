/*
 * calypso_sim.c — ISO 7816 / GSM 11.11 SIM emulator for the Calypso
 *
 * Replaces the historical 1-line ATR-pulse stub. Implements:
 *   - ATR delivery on CMDSTART
 *   - APDU framing through the TX/RX FIFO
 *   - Minimum viable GSM 11.11 file system (MF, DF_GSM, DF_TELECOM)
 *   - Standard commands: SELECT (A4), READ_BINARY (B0), READ_RECORD (B2),
 *                         GET_RESPONSE (C0), STATUS (F2), VERIFY_CHV (20),
 *                         RUN_GSM_ALGORITHM (88)
 *
 * Test SIM identity (matches the standard test PLMN 001/01):
 *   IMSI = 001 01 0000000001  (15 digits)
 *   ICCID = 8901010000000000001 F  (BCD swapped)
 *   Ki = 00..00 (16 bytes — auth always returns deterministic SRES/Kc)
 *
 * Bytes flow: firmware writes APDU bytes one at a time to DTX. We parse
 * the 5-byte ISO 7816 header, optionally collect data bytes (P3 of length
 * for outgoing case 3) and dispatch. Response bytes are pushed to RX FIFO,
 * IT_RX raised, IRQ pulsed.
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "hw/arm/calypso/calypso_sim.h"

#define SIM_LOG(...) do { fprintf(stderr, "[sim] " __VA_ARGS__); fputc('\n', stderr); } while(0)

#define APDU_MAX_LEN     261
#define RX_FIFO_SIZE     512
#define ATR_DELAY_NS     1000000  /* 1 ms simulated */

/* Minimal valid ATR (4 bytes):
 *   TS = 0x3B  (direct convention)
 *   T0 = 0x02  (Y1=0 → no TA/TB/TC/TD interface bytes,
 *               K =2 → 2 historical bytes follow)
 *   Hist[0..1] = 0x14 0x50  (arbitrary, identifies a test card)
 * No TCK byte because no T!=0 protocol indicated.
 * Total bytes the firmware will read = 4. */
static const uint8_t SIM_ATR[] = { 0x3B, 0x02, 0x14, 0x50 };

/* ---------- file system ---------------------------------------------- */

typedef enum {
    EF_TRANSPARENT = 0,
    EF_LINEAR_FIXED = 1,
    EF_CYCLIC = 2,
    EF_DF = 0xF0,    /* not a real EF — directory entry */
    EF_MF = 0xF1,
} EfStructure;

typedef struct SimFile {
    uint16_t fid;
    uint16_t parent;
    uint8_t  structure;
    uint16_t size;        /* total bytes (transparent) or rec_len * nrec */
    uint8_t  rec_len;     /* records (linear/cyclic) */
    uint8_t  data[64];    /* in-line storage (small EFs only) */
} SimFile;

/* SIM identity defaults (overridden at boot from the osmocom-bb mobile
 * config — see load_config_from_file). EF_IMSI is GSM 11.11 BCD-packed
 * with the standard length-byte + parity-nibble layout. */

static SimFile sim_files[] = {
    { 0x3F00, 0x0000, EF_MF, 0, 0, {0} },          /* MF root */
    { 0x7F20, 0x3F00, EF_DF, 0, 0, {0} },          /* DF_GSM */
    { 0x7F10, 0x3F00, EF_DF, 0, 0, {0} },          /* DF_TELECOM */
    { 0x2FE2, 0x3F00, EF_TRANSPARENT, 10, 0,       /* EF_ICCID */
      { 0x98, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF1 } },
    { 0x6F07, 0x7F20, EF_TRANSPARENT, 9, 0,        /* EF_IMSI */
      { 0x08, 0x09, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0xF1 } },
    { 0x6F30, 0x7F20, EF_TRANSPARENT, 24, 0,       /* EF_PLMNsel: 001 01 = FFFFFF empty list */
      { 0x00, 0xF1, 0x10,  /* PLMN 001 01 */
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } },
    { 0x6F38, 0x7F20, EF_TRANSPARENT, 14, 0,       /* EF_SST: services */
      { 0xFF, 0x33, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
    { 0x6F78, 0x7F20, EF_TRANSPARENT, 2, 0,        /* EF_ACC: access class */
      { 0x00, 0x01 } },
    { 0x6FAD, 0x7F20, EF_TRANSPARENT, 4, 0,        /* EF_AD: admin data */
      { 0x00, 0x00, 0x00, 0x02 } },
    { 0x6F7E, 0x7F20, EF_TRANSPARENT, 11, 0,       /* EF_LOCI: location info */
      { 0xFF, 0xFF, 0xFF, 0xFF,           /* TMSI */
        0xFF, 0xFF, 0xFF,                 /* LAI = unknown */
        0x00, 0x00,                       /* TMSI time */
        0xFF, 0x00 } },                   /* update status: not updated */
    { 0x6F74, 0x7F20, EF_TRANSPARENT, 16, 0,       /* EF_BCCH */
      { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } },
    { 0x6F46, 0x7F20, EF_TRANSPARENT, 17, 0,       /* EF_SPN */
      { 0x01, 'Q','E','M','U','-','S','I','M', ' ',' ',' ',' ',' ',' ',' ',' ' } },
};
#define SIM_FILE_COUNT (sizeof(sim_files) / sizeof(sim_files[0]))

static SimFile *find_file(uint16_t fid, uint16_t parent_pref)
{
    /* Try parent-scoped first (for relative selects), then global. */
    if (parent_pref) {
        for (size_t i = 0; i < SIM_FILE_COUNT; i++)
            if (sim_files[i].fid == fid && sim_files[i].parent == parent_pref)
                return &sim_files[i];
    }
    for (size_t i = 0; i < SIM_FILE_COUNT; i++)
        if (sim_files[i].fid == fid)
            return &sim_files[i];
    return NULL;
}

/* ---------- IMSI BCD encoding --------------------------------------- */

/* Encode a numeric IMSI string (e.g. "001010000000001") into the GSM 11.11
 * EF_IMSI 9-byte layout:
 *   byte 0:   length of the actual data following (1..8)
 *   byte 1:   high nibble = first IMSI digit, low nibble = parity
 *             (parity = 9 if odd # digits, 1 if even)
 *   bytes 2..N: pairs of digits, low digit in low nibble
 *               (last byte may have F filler in high nibble) */
static int encode_imsi_bcd(const char *imsi_str, uint8_t out[9])
{
    int n = 0;
    while (imsi_str[n] >= '0' && imsi_str[n] <= '9') n++;
    if (n < 1 || n > 15) return -1;
    bool odd = (n & 1) != 0;
    int data_len = (n / 2) + 1;
    out[0] = (uint8_t)data_len;
    out[1] = (uint8_t)(((imsi_str[0] - '0') << 4) | (odd ? 0x9 : 0x1));
    int wpos = 2, ipos = 1;
    while (ipos < n) {
        uint8_t lo = imsi_str[ipos] - '0';
        uint8_t hi = (ipos + 1 < n) ? (imsi_str[ipos + 1] - '0') : 0xF;
        out[wpos++] = (uint8_t)((hi << 4) | lo);
        ipos += 2;
    }
    while (wpos < 9) out[wpos++] = 0xFF;
    return data_len + 1;
}

/* ---------- card state ----------------------------------------------- */

struct CalypsoSim {
    qemu_irq    irq;
    QEMUTimer  *atr_timer;
    QEMUTimer  *wt_timer;       /* fires IT_WT after RX FIFO drains
                                 * — tells firmware "no more bytes coming" */
    uint8_t     ki[16];         /* COMP128 secret key from mobile.cfg */
    bool        ki_valid;

    /* register shadows */
    uint16_t    cmd, stat, conf1, conf2, maskit, it_cd;

    /* IT register — bits set as events occur, cleared on read */
    uint16_t    it;

    /* TX FIFO (firmware → SIM) — APDU assembly */
    uint8_t     apdu[APDU_MAX_LEN];
    int         apdu_pos;       /* bytes received so far */
    int         apdu_expected;  /* total expected length */

    /* RX FIFO (SIM → firmware) */
    uint8_t     rx[RX_FIFO_SIZE];
    int         rx_head, rx_tail;

    /* selected file context */
    uint16_t    selected_df;   /* current directory (MF or DF_GSM/TELECOM) */
    uint16_t    selected_ef;   /* last selected EF (for SELECT response) */

    /* GET RESPONSE pending data */
    uint8_t     resp_buf[64];
    int         resp_len;

    bool        powered;
};

/* ---------- RX FIFO --------------------------------------------------- */

G_GNUC_UNUSED
static int rx_count(CalypsoSim *s)
{
    int c = s->rx_head - s->rx_tail;
    if (c < 0) c += RX_FIFO_SIZE;
    return c;
}

static void rx_push(CalypsoSim *s, uint8_t b)
{
    int next = (s->rx_head + 1) % RX_FIFO_SIZE;
    if (next == s->rx_tail) {
        SIM_LOG("RX FIFO overflow");
        return;
    }
    s->rx[s->rx_head] = b;
    s->rx_head = next;
}

G_GNUC_UNUSED
static int rx_pop(CalypsoSim *s, uint8_t *out)
{
    if (s->rx_head == s->rx_tail) return 0;
    *out = s->rx[s->rx_tail];
    s->rx_tail = (s->rx_tail + 1) % RX_FIFO_SIZE;
    return 1;
}

static void rx_push_n(CalypsoSim *s, const uint8_t *buf, int n)
{
    for (int i = 0; i < n; i++) rx_push(s, buf[i]);
}

/* Forward decl */
static void update_irq(CalypsoSim *s);

/* IT_WT semantics: fires when no new char arrives within the configured
 * "wait time" after the last byte. The osmocom-bb sim_irq_handler uses
 * IT_WT to flag rxDoneFlag when calypso_sim_receive was invoked with
 * expected_length=0 (open-ended ATR receive). We schedule WT a few
 * milliseconds after the FIFO drains. */
#define WT_DELAY_NS  2000000  /* 2 ms simulated */

static void fire_wt(void *opaque)
{
    CalypsoSim *s = opaque;
    if (rx_count(s) > 0) return;        /* new bytes arrived — cancel WT */
    s->it |= CALYPSO_SIM_IT_WT;
    SIM_LOG("WT timeout fired (RX FIFO empty)");
    update_irq(s);
}

static void schedule_wt(CalypsoSim *s)
{
    if (s->wt_timer)
        timer_mod_ns(s->wt_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + WT_DELAY_NS);
}

/* IT_RX semantics per Calypso TRM: the bit is set whenever there is
 * a byte waiting to be read in the RX FIFO (DRX). It auto-clears on
 * read access to DRX (when the FIFO empties). We mirror that by
 * recomputing IT_RX from the FIFO occupancy at every IT/DRX touch. */
static void refresh_it_rx(CalypsoSim *s)
{
    if (rx_count(s) > 0) s->it |= CALYPSO_SIM_IT_RX;
    else                 s->it &= ~CALYPSO_SIM_IT_RX;
}

/* Update the IRQ line from the current IT register state. The Calypso
 * INTH is level-sensitive — a pulse can be missed when the ARM has
 * IRQs masked (CPSR I=1) at the moment of the rise/fall. We hold the
 * line high while any unmasked IT bit is set. */
static void update_irq(CalypsoSim *s)
{
    if (!s->irq) return;
    refresh_it_rx(s);
    bool active = (s->it & ~(uint16_t)s->maskit) != 0;
    static unsigned log_count;
    static bool last_active;
    if ((active != last_active && log_count < 30) || (log_count < 10)) {
        SIM_LOG("IRQ %s  IT=0x%04x MASKIT=0x%04x rx_count=%d",
                active ? "RAISE" : "lower",
                s->it, s->maskit, rx_count(s));
        log_count++;
        last_active = active;
    }
    if (active) qemu_irq_raise(s->irq);
    else        qemu_irq_lower(s->irq);
}

static void raise_rx_irq(CalypsoSim *s)
{
    update_irq(s);
}

/* ---------- ATR delivery --------------------------------------------- */

static void deliver_atr(void *opaque)
{
    CalypsoSim *s = opaque;
    if (!s->powered) return;
    rx_push_n(s, SIM_ATR, sizeof(SIM_ATR));
    SIM_LOG("ATR queued (%zu bytes)", sizeof(SIM_ATR));
    raise_rx_irq(s);
}

G_GNUC_UNUSED
static void schedule_atr(CalypsoSim *s)
{
    timer_mod_ns(s->atr_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ATR_DELAY_NS);
}

/* ---------- SELECT response (FCP / response template) --------------- */

static int build_select_response(CalypsoSim *s, SimFile *f, uint8_t *out)
{
    /* GSM 11.11 SELECT response (status info data). 22 bytes for EF, 22 bytes
     * for DF/MF. We build a simple but firmware-friendly template. */
    int p = 0;
    out[p++] = 0x00; out[p++] = 0x00;            /* RFU */
    out[p++] = (f->size >> 8) & 0xFF;            /* file size MSB */
    out[p++] =  f->size       & 0xFF;            /* file size LSB */
    out[p++] = (f->fid >> 8) & 0xFF;             /* file ID MSB */
    out[p++] =  f->fid       & 0xFF;             /* file ID LSB */
    if (f->structure == EF_DF || f->structure == EF_MF) {
        out[p++] = (f->structure == EF_MF) ? 0x01 : 0x02;  /* file type */
        out[p++] = 0x00;                          /* RFU */
        out[p++] = 0x00; out[p++] = 0x00;
        out[p++] = 0x00; out[p++] = 0x00;
        out[p++] = 0x00; out[p++] = 0x00;
        out[p++] = 0x00;
        out[p++] = 0x09;                          /* GSM data length */
        out[p++] = 0x91;                          /* file characteristics */
        out[p++] = 0x00;                          /* num child DFs */
        out[p++] = 0x00;                          /* num child EFs */
        out[p++] = 0x04;                          /* CHVs */
        out[p++] = 0x00;
        out[p++] = 0x83;                          /* CHV1 status */
        out[p++] = 0x83;                          /* unblock */
        out[p++] = 0x83;                          /* CHV2 */
        out[p++] = 0x83;
    } else {
        out[p++] = 0x04;                          /* file type EF */
        out[p++] = 0x00;
        out[p++] = 0x00; out[p++] = 0x00;
        out[p++] = 0x00;
        out[p++] = 0xAA;                          /* access conditions */
        out[p++] = 0x00; out[p++] = 0x00;
        out[p++] = 0x00;                          /* file status */
        out[p++] = 0x02;                          /* length of remaining */
        out[p++] = (f->structure == EF_TRANSPARENT) ? 0x00
                : (f->structure == EF_LINEAR_FIXED) ? 0x01 : 0x03;
        out[p++] = f->rec_len;                    /* record length */
    }
    return p;
}

/* ---------- APDU dispatch ------------------------------------------- */

static void respond_sw(CalypsoSim *s, uint8_t sw1, uint8_t sw2)
{
    rx_push(s, sw1);
    rx_push(s, sw2);
}

/* For T=0, GSM 11.11 has special procedure: when response data exists,
 * card sends:  SW1=0x9F (or 0x6C/0x61) + SW2=length
 * Then firmware does GET_RESPONSE to read the actual data. */
static void respond_with_data_pending(CalypsoSim *s,
                                       const uint8_t *data, int len)
{
    if (len > (int)sizeof(s->resp_buf)) len = sizeof(s->resp_buf);
    memcpy(s->resp_buf, data, len);
    s->resp_len = len;
    /* 9F xx = "data available, do GET_RESPONSE for xx bytes" */
    respond_sw(s, 0x9F, (uint8_t)len);
}

static void cmd_select(CalypsoSim *s, uint8_t p1, uint8_t p2,
                       uint8_t lc, const uint8_t *data)
{
    if (lc != 2) { respond_sw(s, 0x67, 0x00); return; }
    uint16_t fid = (data[0] << 8) | data[1];

    /* Pick parent context: 3F00 reset, 7Fxx changes DF, 2Fxx EF under MF,
     * 6Fxx EF under current DF. */
    SimFile *f = NULL;
    if (fid == 0x3F00) {
        f = find_file(0x3F00, 0);
        s->selected_df = 0x3F00;
    } else if ((fid & 0xFF00) == 0x7F00) {
        f = find_file(fid, 0);
        if (f) s->selected_df = fid;
    } else if ((fid & 0xFF00) == 0x2F00) {
        f = find_file(fid, 0x3F00);
    } else {
        f = find_file(fid, s->selected_df);
        if (!f) f = find_file(fid, 0);
    }

    if (!f) {
        SIM_LOG("SELECT 0x%04x → file not found", fid);
        respond_sw(s, 0x6A, 0x82);
        return;
    }
    s->selected_ef = f->fid;

    uint8_t resp[32];
    int n = build_select_response(s, f, resp);
    SIM_LOG("SELECT 0x%04x (%s) → %d bytes pending",
            fid,
            f->structure == EF_MF ? "MF" :
            f->structure == EF_DF ? "DF" : "EF",
            n);
    respond_with_data_pending(s, resp, n);
}

static void cmd_get_response(CalypsoSim *s, uint8_t le)
{
    if (s->resp_len == 0) {
        respond_sw(s, 0x6F, 0x00);
        return;
    }
    int n = (le == 0) ? s->resp_len : (le > s->resp_len ? s->resp_len : le);
    rx_push_n(s, s->resp_buf, n);
    respond_sw(s, 0x90, 0x00);
    s->resp_len = 0;
}

static void cmd_read_binary(CalypsoSim *s, uint8_t p1, uint8_t p2, uint8_t le)
{
    SimFile *f = find_file(s->selected_ef, 0);
    if (!f || f->structure != EF_TRANSPARENT) {
        respond_sw(s, 0x94, 0x00);
        return;
    }
    int offset = (p1 << 8) | p2;
    int n = (le == 0) ? 256 : le;
    if (offset + n > f->size) {
        respond_sw(s, 0x67, 0x00);
        return;
    }
    rx_push_n(s, f->data + offset, n);
    SIM_LOG("READ_BINARY EF=0x%04x off=%d len=%d", s->selected_ef, offset, n);
    respond_sw(s, 0x90, 0x00);
}

static void cmd_status(CalypsoSim *s, uint8_t le)
{
    SimFile *df = find_file(s->selected_df, 0);
    if (!df) { respond_sw(s, 0x6F, 0x00); return; }
    uint8_t resp[32];
    int n = build_select_response(s, df, resp);
    int outn = (le == 0 || le > n) ? n : le;
    rx_push_n(s, resp, outn);
    respond_sw(s, 0x90, 0x00);
}

static void cmd_verify_chv(CalypsoSim *s, uint8_t p2, uint8_t lc,
                           const uint8_t *data)
{
    /* Test SIM accepts any PIN. Real card would compare and decrement
     * remaining attempts on mismatch. */
    (void)p2; (void)lc; (void)data;
    SIM_LOG("VERIFY_CHV → always OK");
    respond_sw(s, 0x90, 0x00);
}

static void cmd_run_gsm_algo(CalypsoSim *s, uint8_t lc, const uint8_t *data)
{
    /* RAND in (16 bytes), SRES out (4 bytes) + Kc (8 bytes) = 12 bytes. */
    if (lc != 16) { respond_sw(s, 0x67, 0x00); return; }
    /* Test SIM: deterministic SRES = first 4 bytes of RAND ^ 0xAA, Kc = 0. */
    uint8_t resp[12];
    for (int i = 0; i < 4; i++) resp[i]    = data[i] ^ 0xAA;
    for (int i = 0; i < 8; i++) resp[4+i] = 0x00;
    SIM_LOG("RUN_GSM_ALGORITHM (RAND[0]=0x%02x) → SRES[0]=0x%02x",
            data[0], resp[0]);
    respond_with_data_pending(s, resp, 12);
}

static void dispatch_apdu(CalypsoSim *s)
{
    uint8_t cla = s->apdu[0];
    uint8_t ins = s->apdu[1];
    uint8_t p1  = s->apdu[2];
    uint8_t p2  = s->apdu[3];
    uint8_t p3  = s->apdu[4];
    uint8_t *data = (s->apdu_pos > 5) ? s->apdu + 5 : NULL;
    int     dlen = s->apdu_pos - 5;

    SIM_LOG("APDU CLA=%02x INS=%02x P1=%02x P2=%02x P3=%02x dlen=%d",
            cla, ins, p1, p2, p3, dlen > 0 ? dlen : 0);

    /* GSM 11.11 expects CLA=A0; ISO 7816 base allows other classes. */
    switch (ins) {
    case 0xA4: cmd_select(s, p1, p2, p3, data ? data : (const uint8_t *)""); break;
    case 0xB0: cmd_read_binary(s, p1, p2, p3); break;
    case 0xC0: cmd_get_response(s, p3); break;
    case 0xF2: cmd_status(s, p3); break;
    case 0x20: cmd_verify_chv(s, p2, p3, data ? data : (const uint8_t *)""); break;
    case 0x88: cmd_run_gsm_algo(s, p3, data ? data : (const uint8_t *)""); break;
    default:
        SIM_LOG("INS=0x%02x not supported → SW=6D00", ins);
        respond_sw(s, 0x6D, 0x00);
        break;
    }

    raise_rx_irq(s);
}

/* When firmware writes a byte to DTX, accumulate. After 5 header bytes
 * we know the case (incoming/outgoing) and how much more to read. */
G_GNUC_UNUSED
static void apdu_tx_byte(CalypsoSim *s, uint8_t b)
{
    if (s->apdu_pos < APDU_MAX_LEN) {
        s->apdu[s->apdu_pos++] = b;
    }
    if (s->apdu_pos == 5) {
        /* For simplicity: if INS is a known WRITE-class command (data sent
         * by firmware), expect P3 more bytes. Otherwise (READ class),
         * dispatch immediately. */
        uint8_t ins = s->apdu[1];
        bool is_outgoing_data =
            (ins == 0xA4) || (ins == 0xD6) || (ins == 0xDC) ||
            (ins == 0x20) || (ins == 0x24) || (ins == 0x88);
        if (is_outgoing_data && s->apdu[4] != 0) {
            s->apdu_expected = 5 + s->apdu[4];
        } else {
            s->apdu_expected = 5;
        }
        /* Procedure byte: T=0 expects card to ACK by echoing INS.
         * The firmware reads this from DRX before sending the data part. */
        if (is_outgoing_data && s->apdu[4] != 0) {
            rx_push(s, ins);
            raise_rx_irq(s);
        }
    }
    if (s->apdu_pos == s->apdu_expected) {
        dispatch_apdu(s);
        s->apdu_pos = 0;
        s->apdu_expected = 0;
    }
}

/* ---------- public register interface ------------------------------- */

uint16_t calypso_sim_reg_read(CalypsoSim *s, hwaddr off)
{
    switch (off) {
    case CALYPSO_SIM_REG_CMD:
        return s->cmd;
    case CALYPSO_SIM_REG_STAT: {
        uint16_t v = 0;
        if (s->powered) v |= CALYPSO_SIM_STAT_NOCARD;   /* card detected */
        v |= CALYPSO_SIM_STAT_TXPAR;                    /* parity always OK */
        if (rx_count(s) == 0) v |= CALYPSO_SIM_STAT_FIFOEMPTY;
        if (rx_count(s) >= RX_FIFO_SIZE - 1) v |= CALYPSO_SIM_STAT_FIFOFULL;
        return v;
    }
    case CALYPSO_SIM_REG_CONF1:  return s->conf1;
    case CALYPSO_SIM_REG_CONF2:  return s->conf2;
    case CALYPSO_SIM_REG_IT: {
        refresh_it_rx(s);
        uint16_t v = s->it;
        /* Edge bits (NATR/WT/OV/TX) are read-clear; level bit RX stays. */
        s->it &= CALYPSO_SIM_IT_RX;
        update_irq(s);
        return v;
    }
    case CALYPSO_SIM_REG_DRX: {
        uint8_t b = 0;
        rx_pop(s, &b);
        update_irq(s);                                  /* maybe clear IT_RX */
        if (rx_count(s) == 0) schedule_wt(s);           /* arm WT when FIFO drains */
        return b | (1 << 8);                            /* parity OK */
    }
    case CALYPSO_SIM_REG_DTX:    return 0;
    case CALYPSO_SIM_REG_MASKIT: return s->maskit;
    case CALYPSO_SIM_REG_IT_CD:  return s->it_cd;
    default: return 0;
    }
}

void calypso_sim_reg_write(CalypsoSim *s, hwaddr off, uint16_t val)
{
    switch (off) {
    case CALYPSO_SIM_REG_CMD:
        s->cmd = val;
        if (val & CALYPSO_SIM_CMD_START) {
            s->powered = true;
            SIM_LOG("CMDSTART → ATR scheduled");
            schedule_atr(s);
        }
        if (val & CALYPSO_SIM_CMD_STOP) {
            s->powered = false;
            SIM_LOG("CMDSTOP");
        }
        if (val & (CALYPSO_SIM_CMD_CARDRST | CALYPSO_SIM_CMD_IFRST)) {
            SIM_LOG("RESET");
            s->apdu_pos = 0;
            s->apdu_expected = 0;
            s->resp_len = 0;
            s->rx_head = s->rx_tail = 0;
            s->selected_df = 0x3F00;
            s->selected_ef = 0x3F00;
            if (s->powered) schedule_atr(s);
        }
        break;
    case CALYPSO_SIM_REG_STAT:    s->stat   = val; break;
    case CALYPSO_SIM_REG_CONF1:   s->conf1  = val; break;
    case CALYPSO_SIM_REG_CONF2:   s->conf2  = val; break;
    case CALYPSO_SIM_REG_IT:      /* W1C ignored */    break;
    case CALYPSO_SIM_REG_DRX:     /* read-only */      break;
    case CALYPSO_SIM_REG_DTX:
        apdu_tx_byte(s, (uint8_t)(val & 0xFF));
        break;
    case CALYPSO_SIM_REG_MASKIT:
        s->maskit = val;
        update_irq(s);     /* re-evaluate line after mask change */
        break;
    case CALYPSO_SIM_REG_IT_CD:   s->it_cd  = val; break;
    default: break;
    }
}

/* ---------- mobile.cfg parser --------------------------------------- */

/* Parse the osmocom-bb layer23 mobile config:
 *   ms 1
 *     test-sim
 *       imsi 001010000000001
 *       ki comp128 00 11 22 ... 16 hex bytes
 * We pull the imsi (→ EF_IMSI) and ki (→ s->ki for RUN_GSM_ALGORITHM). */
static void load_config_from_file(CalypsoSim *s, const char *path)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        SIM_LOG("config %s not found — keeping default file system", path);
        return;
    }
    char line[256];
    bool in_test_sim = false;
    while (fgets(line, sizeof(line), fp)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (strncmp(p, "test-sim", 8) == 0) {
            in_test_sim = true;
            continue;
        }
        if (!in_test_sim) continue;
        /* Section ends at a non-indented keyword (e.g. "ms", "exit", "!") */
        if (line[0] != ' ' && line[0] != '\t' && line[0] != '\n') {
            in_test_sim = false;
            continue;
        }
        if (strncmp(p, "imsi ", 5) == 0) {
            const char *imsi = p + 5;
            uint8_t bcd[9];
            if (encode_imsi_bcd(imsi, bcd) > 0) {
                SimFile *ef = find_file(0x6F07, 0x7F20);
                if (ef) {
                    memcpy(ef->data, bcd, 9);
                    ef->size = 9;
                    SIM_LOG("EF_IMSI loaded from %s: %.15s", path, imsi);
                }
            }
        } else if (strncmp(p, "ki comp128 ", 11) == 0) {
            const char *hex = p + 11;
            int n = 0;
            while (n < 16 && *hex) {
                while (*hex == ' ') hex++;
                if (!*hex) break;
                unsigned v;
                if (sscanf(hex, "%2x", &v) != 1) break;
                s->ki[n++] = (uint8_t)v;
                hex += 2;
            }
            if (n == 16) {
                s->ki_valid = true;
                SIM_LOG("Ki loaded from %s (16 bytes)", path);
            }
        }
    }
    fclose(fp);
}

CalypsoSim *calypso_sim_new(qemu_irq sim_irq)
{
    CalypsoSim *s = g_new0(CalypsoSim, 1);
    s->irq = sim_irq;
    s->atr_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, deliver_atr, s);
    s->wt_timer  = timer_new_ns(QEMU_CLOCK_VIRTUAL, fire_wt,     s);
    s->selected_df = 0x3F00;
    s->selected_ef = 0x3F00;

    /* Pull IMSI / Ki from the same config the layer23 `mobile` is using.
     * The launcher (run_new.sh) sets CALYPSO_SIM_CFG to its $MOBILE_CFG,
     * so the SIM and the mobile stay in sync without us hardcoding any
     * path. If the env var is missing we keep the file-system defaults. */
    const char *cfg_path = getenv("CALYPSO_SIM_CFG");
    if (cfg_path && *cfg_path) {
        load_config_from_file(s, cfg_path);
    } else {
        SIM_LOG("CALYPSO_SIM_CFG unset — keeping default IMSI/Ki");
    }
    return s;
}
