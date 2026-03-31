/*
 * calypso_trx.h — Calypso DSP/TPU/TRX bridge for virtual GSM
 *
 * Includes FCCH/SCH ARFCN sync simulation for TRX firmware.
 */

#ifndef CALYPSO_TRX_H
#define CALYPSO_TRX_H

#include "hw/irq.h"
#include "exec/memory.h"

/* =====================================================================
 * Correct Calypso IRQ map (from OsmocomBB calypso/irq.h)
 * ===================================================================== */

#define CALYPSO_IRQ_WATCHDOG       0
#define CALYPSO_IRQ_TIMER1         1
#define CALYPSO_IRQ_TIMER2         2
#define CALYPSO_IRQ_TSP_RX         3
#define CALYPSO_IRQ_TPU_FRAME      4
#define CALYPSO_IRQ_TPU_PAGE       5
#define CALYPSO_IRQ_SIM            6
#define CALYPSO_IRQ_UART_MODEM     7
#define CALYPSO_IRQ_KEYPAD_GPIO    8
#define CALYPSO_IRQ_RTC_TIMER      9
#define CALYPSO_IRQ_RTC_ALARM      10
#define CALYPSO_IRQ_ULPD_GAUGING   11
#define CALYPSO_IRQ_EXTERNAL       12
#define CALYPSO_IRQ_SPI            13
#define CALYPSO_IRQ_DMA            14
#define CALYPSO_IRQ_API            15
#define CALYPSO_IRQ_SIM_DETECT     16
#define CALYPSO_IRQ_EXTERNAL_FIQ   17
#define CALYPSO_IRQ_UART_IRDA      18
#define CALYPSO_IRQ_ULPD_GSM_TIMER 19
#define CALYPSO_IRQ_GEA            20
#define CALYPSO_NUM_IRQS           32

/* =====================================================================
 * Hardware addresses
 * ===================================================================== */

#define CALYPSO_DSP_BASE      0xFFD00000
#define CALYPSO_DSP_SIZE      (64 * 1024)

/* DB/NDB byte offsets from DSP_BASE (ARM view)
 *
 * DSP word addr → ARM byte offset = word_addr * 2
 *
 *   DSP word 0x0000 → byte 0x0000 : DB Write Page 0
 *   DSP word 0x0800 → byte 0x1000 : DB Write Page 1
 *   DSP word 0x1000 → byte 0x2000 : DB Read Page 0
 *   DSP word 0x1800 → byte 0x3000 : DB Read Page 1
 *   DSP word 0x2000 → byte 0x4000 : NDB
 *   DSP word 0x2400 → byte 0x4800 : PARAM
 */
/* DSP API page layout — MUST match firmware's BASE_API_* defines.
 * These are byte offsets from DSP_BASE (0xFFD00000).
 * Each DB page is 20 words (0x28 bytes). */
#define DSP_API_W_PAGE0       0x0000
#define DSP_API_W_PAGE1       0x0028
#define DSP_API_R_PAGE0       0x0050
#define DSP_API_R_PAGE1       0x0078
#define DSP_API_NDB           0x01A8
#define DSP_API_PARAM         0x0862
#define DSP_PAGE_SIZE         0x0028

#define CALYPSO_TPU_BASE      0xFFFF1000
#define CALYPSO_TPU_SIZE      0x0100

#define CALYPSO_TPU_RAM_BASE  0xFFFF9000
#define CALYPSO_TPU_RAM_SIZE  0x0800

#define CALYPSO_TSP_BASE      0xFFFE0800
#define CALYPSO_TSP_SIZE      0x0100

#define CALYPSO_SIM_BASE      0xFFFE0000
#define CALYPSO_SIM_SIZE      0x0100

#define CALYPSO_ULPD_BASE     0xFFFE2800
#define CALYPSO_ULPD_SIZE     0x0100

/* TPU register offsets (from OsmocomBB calypso/tpu.h) */
#define TPU_CTRL              0x0000
#define TPU_INT_CTRL          0x0002
#define TPU_INT_STAT          0x0004
#define TPU_OFFSET            0x000C
#define TPU_SYNCHRO           0x000E
#define TPU_IT_DSP_PG         0x0020
#define TPU_RAM_BASE          0x0400

/* TPU_CTRL bits (from OsmocomBB tpu.c enum tpu_ctrl_bits) */
#define TPU_CTRL_RESET        (1 << 0)
#define TPU_CTRL_PAGE         (1 << 1)
#define TPU_CTRL_EN           (1 << 2)
#define TPU_CTRL_DSP_EN       (1 << 4)
#define TPU_CTRL_MCU_RAM_ACC  (1 << 6)
#define TPU_CTRL_TSP_RESET    (1 << 7)
#define TPU_CTRL_IDLE         (1 << 8)
#define TPU_CTRL_WAIT         (1 << 9)
#define TPU_CTRL_CK_ENABLE    (1 << 10)
#define TPU_CTRL_FULL_WRITE   (1 << 11)

/* TPU INT_CTRL bits */
#define ICTRL_MCU_FRAME       (1 << 0)
#define ICTRL_MCU_PAGE        (1 << 1)
#define ICTRL_DSP_FRAME       (1 << 2)
#define ICTRL_DSP_FRAME_FORCE (1 << 3)

/* TSP register offsets */
#define TSP_TX_REG            0x00
#define TSP_CTRL1             0x02
#define TSP_CTRL2             0x04
#define TSP_TX_SIZE           0x06
#define TSP_RX_REG            0x08
#define TSP_MASK1             0x0A
#define TSP_ACT               0x0C
#define TSP_ACT_L             0x0E

/* ULPD register offsets */
#define ULPD_SETUP_CLK13      0x00
#define ULPD_SETUP_SLICER     0x02
#define ULPD_SETUP_VTCXO      0x04
#define ULPD_SETUP_RF         0x06
#define ULPD_DCXO_SETUP       0x08
#define ULPD_ITP_1            0x12
#define ULPD_ITP_2            0x14
#define ULPD_COUNTER_HI       0x1C
#define ULPD_COUNTER_LO       0x1E
#define ULPD_GAUGING_CTRL     0x24
#define ULPD_GSM_TIMER        0x28

/* GSM timing */
/* Real GSM: 4615000 ns. Slowed 10x for emulation to give the firmware
 * enough virtual CPU time between frames to process L1CTL and sercomm.
 * FN is resynced to fake_trx via CLK IND every 102 frames. */
#define GSM_TDMA_NS           46150000
#define GSM_HYPERFRAME        2715648
#define GSM_BURST_BITS        148
#define GSM_BURST_WORDS       78

/* TRX UDP protocol (TRXD v0) */
#define TRX_DEFAULT_PORT      4729
#define TRX_HDR_LEN_TX        6
#define TRX_HDR_LEN_RX        8
#define TRX_BURST_LEN         148
#define TRX_PKT_LEN_TX        (TRX_HDR_LEN_TX + TRX_BURST_LEN)
#define TRX_PKT_LEN_RX        (TRX_HDR_LEN_RX + TRX_BURST_LEN)

/* =====================================================================
 * DB write page structure — word offsets from page base
 *
 * The firmware writes d_task_d / d_task_u at the start of the
 * active DB write page.  Exact layout (from dsp_api.h):
 *
 *   Word 0:  d_task_d   (DL task command + burst_id + tsc)
 *   Word 1:  d_burst_d  (DL burst identifier)
 *   Word 2:  d_task_u   (UL task command)
 *   Word 3:  d_burst_u  (UL burst identifier)
 * ===================================================================== */

#define DB_W_D_TASK_D         0   /* word offset */
#define DB_W_D_BURST_D        1
#define DB_W_D_TASK_U         2
#define DB_W_D_BURST_U        3
#define DB_W_D_TASK_MD        4   /* monitoring/PM task */

/* DB read page — PM result array (a_pm[4])
 * Confirmed by disasm of l1s_pm_resp: db_r_ptr + 24 bytes = word 12.
 * The firmware does a_pm[i] >> 3 before passing to agc_inp_dbm8_by_pm. */
#define DB_R_A_PM             12  /* word offset: a_pm[0] in read page */
#define DB_R_A_PM_COUNT       4   /* 4 PM measurement slots */

/* PM raw value calibration.
 * agc_inp_dbm8_by_pm: dbm8 = (pm_raw >> 3) - (rffe_gain + agc_byte) * 8
 * With default gain ≈ 138:  pm_raw = (target_dbm8 + gain*8) * 8
 * For -62 dBm target: pm_raw = (-496 + 1104) * 8 = 4864 */
#define PM_RAW_STRONG         4864   /* ≈ -62 dBm at RF */
#define PM_RAW_NOISE          64     /* ≈ -137 dBm at RF (near noise floor) */

/* =====================================================================
 * NDB structure — word offsets from NDB base (byte 0x4000)
 *
 * IMPORTANT: These offsets MUST match the T_NDB_MCU_TO_DSP struct
 * compiled into the firmware you are running.  The values below
 * match the standard OsmocomBB TCS211-compatible DSP API.
 *
 * If your firmware uses different offsets, enable TRX_DEBUG_DSP=1
 * and look at which NDB words are read after an FB/SB task.
 * Then update these constants accordingly.
 *
 * The NDB is a FLAT shared-memory area used in both directions:
 *   MCU→DSP (control): d_dsp_page, d_fn, task mirrors, etc.
 *   DSP→MCU (results): d_fb_det, a_cd[], a_sch26[], etc.
 * ===================================================================== */

/* MCU→DSP control fields */
#define NDB_W_D_DSP_PAGE      0x0000  /* Current active page (bit 0) */
#define NDB_W_D_TASK_D        0x0001  /* DL task mirror */
#define NDB_W_D_TASK_U        0x0002  /* UL task mirror */
#define NDB_W_D_FN            0x0003  /* Frame number (low 16 bits) */

/*
 * DSP→MCU result fields (FB / SB / PM)
 *
 * The offsets below are for the standard Calypso TCS211 API.
 * d_fb_det and friends sit after ~20 control words in the NDB.
 *
 * TUNING NOTE: If your firmware doesn't sync, the most likely
 * cause is that these offsets are wrong.  Enable TRX_DEBUG_DSP
 * and search the log for NDB reads in the 0x4014-0x4080 range
 * that happen right after FB/SB tasks fire.
 */
/* Confirmed by firmware disasm: d_fb_det read at 0x01F0 = NDB+0x48 = word 36 */
#define NDB_W_D_FB_DET        36   /* FB detection flag: 1=found */
#define NDB_W_D_FB_MODE       37   /* FB mode / attempt count */

/* a_sync_demod[4]: FB/SB demod results (TOA, PM, ANGLE, SNR) */
#define NDB_W_A_CD_TOA        38   /* Time Of Arrival (quarter-bits) */
#define NDB_W_A_CD_PM         39   /* Power Measurement (1/64 dBm) */
#define NDB_W_A_CD_ANGLE      40   /* Freq offset angle (Hz, signed) */
#define NDB_W_A_CD_SNR        41   /* Signal/Noise Ratio (dB, fx6.10) */

/* a_sch26[5]: SCH decoded data (25 info bits + parity) */
#define NDB_W_A_SCH26         42   /* 5 consecutive words */
#define NDB_W_A_SCH26_LEN     5

/* =====================================================================
 * DSP boot / download status
 *
 * Location: word 0 of the entire API RAM (byte offset 0x0000)
 * The firmware polls this during dsp_power_on():
 *   0x0000 → DSP in reset
 *   0x0001 → DSP ROM running, waiting for patches
 *   0x0002 → DSP ready (patches applied)
 *
 * API version is read from a nearby offset after status == 2.
 * Standard Calypso DSP version: 0x3606 0x0000
 * ===================================================================== */

#define DSP_DL_STATUS_ADDR    0x0FFE  /* byte offset in API RAM (end of page 0) */
#define DSP_API_VER_ADDR      0x01b4  /* byte offset: version word 1 */
#define DSP_API_VER2_ADDR     0x01b6  /* byte offset: version word 2 */

#define DSP_DL_STATUS_RESET   0x0000
#define DSP_DL_STATUS_BOOT    0x0001  /* DSP ROM ready for patches */
#define DSP_DL_STATUS_READY   0x0002
#define DSP_API_VERSION       0x3606

/* =====================================================================
 * ARFCN sync state machine
 * ===================================================================== */

typedef enum {
    SYNC_IDLE = 0,       /* No sync in progress */
    SYNC_FCCH_SEARCH,    /* Searching for FCCH (frequency burst) */
    SYNC_FCCH_FOUND,     /* FCCH detected, preparing for SCH */
    SYNC_SCH_SEARCH,     /* Waiting for SCH (sync burst) decode */
    SYNC_LOCKED,         /* TDMA locked to reference cell */
} SyncState;

/* =====================================================================
 * Simulated cell parameters (virtual BTS we pretend to see)
 * ===================================================================== */

#define SYNC_DEFAULT_ARFCN     514    /* Default reference ARFCN (DCS1800, matches osmo-bsc arfcn 514) */
#define SYNC_DEFAULT_BSIC      7      /* BSIC=7, matches BSC config */
#define SYNC_DEFAULT_RSSI       -62   /* dBm */
#define SYNC_FB_DETECT_DELAY    5     /* Frames until FB found */
#define SYNC_SB_DECODE_DELAY    2     /* Frames until SB decoded */

/* =====================================================================
 * SCH encoding helpers
 *
 * GSM 05.02 §3.3.2.2.1: SCH carries 25 info bits:
 *   bits[0..5]   = BSIC (NCC:3 + BCC:3)
 *   bits[6..16]  = T1   (FN / 1326)
 *   bits[17..19] = T3'  ((T3-1)/10, where T3 = FN mod 51)
 *   bits[20..24] = T2   (FN mod 26)
 *
 * The DSP packs the decoded result into a_sch26[5] (5 × 16-bit words).
 * Standard packing: bits are MSB-first across the words.
 *   a_sch26[0] = bits[0..15]
 *   a_sch26[1] = bits[16..24] << 7  (upper 9 bits, rest zero)
 *   a_sch26[2..4] = 0 (unused / CRC residue)
 * ===================================================================== */

static inline void sch_encode(uint16_t *a_sch26, uint8_t bsic, uint32_t fn)
{
    /* Compute T1, T2, T3, T3' from frame number */
    uint32_t t1  = fn / (26 * 51);            /* 11 bits */
    uint32_t t2  = fn % 26;                    /*  5 bits */
    uint32_t t3  = fn % 51;                    /*  6 bits */
    uint32_t t3p = (t3 >= 1) ? ((t3 - 1) / 10) : 0;  /* 3 bits */

    /* Pack 25 bits: BSIC[5:0] T1[10:0] T3'[2:0] T2[4:0] */
    uint32_t packed = 0;
    packed |= ((uint32_t)(bsic & 0x3F)) << 19;   /* bits 24..19 */
    packed |= ((uint32_t)(t1 & 0x7FF))  <<  8;   /* bits 18..8  */
    packed |= ((uint32_t)(t3p & 0x07))  <<  5;   /* bits  7..5  */
    packed |= ((uint32_t)(t2 & 0x1F))   <<  0;   /* bits  4..0  */

    /* Store in a_sch26[] — MSB-first, 16-bit words */
    a_sch26[0] = (packed >> 9) & 0xFFFF;   /* upper 16 of 25 bits */
    a_sch26[1] = (packed & 0x01FF) << 7;   /* lower 9 bits, shifted */
    a_sch26[2] = 0;                         /* CRC ok (zero residue) */
    a_sch26[3] = 0;
    a_sch26[4] = 0;
}

/* =====================================================================
 * Public interface
 * ===================================================================== */

void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs, int trx_port,
                      int air_local_port, int air_peer_port);

/* Apply firmware patches (cons_puts NOP, talloc fix, abort fix).
 * Call once from the earliest peripheral access after firmware load. */
void calypso_fw_patch_apply(void);

#endif /* CALYPSO_TRX_H */
