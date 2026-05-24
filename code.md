===== ./configs/devices/arm-softmmu/default.mak =====
# Default configuration for arm-softmmu

CONFIG_CALYPSO=y

# Uncomment the following lines to disable these optional devices:
# CONFIG_I2C_DEVICES=n
# CONFIG_PCI_DEVICES=n
# CONFIG_TEST_DEVICES=n

# Boards are selected by default, uncomment to keep out of the build.
# CONFIG_ARM_VIRT=n

# These are selected by default when TCG is enabled, uncomment them to
# keep out of the build.
# CONFIG_CUBIEBOARD=n
# CONFIG_EXYNOS4=n
# CONFIG_HIGHBANK=n
# CONFIG_INTEGRATOR=n
# CONFIG_FSL_IMX31=n
# CONFIG_MUSICPAL=n
# CONFIG_MPS3R=n
# CONFIG_MUSCA=n
# CONFIG_SX1=n
# CONFIG_STELLARIS=n
# CONFIG_STM32VLDISCOVERY=n
# CONFIG_B_L475E_IOT01A=n
# CONFIG_REALVIEW=n
# CONFIG_VERSATILE=n
# CONFIG_VEXPRESS=n
# CONFIG_ZYNQ=n
# CONFIG_NPCM7XX=n
# CONFIG_COLLIE=n
# CONFIG_ASPEED_SOC=n
# CONFIG_NETDUINO2=n
# CONFIG_NETDUINOPLUS2=n
# CONFIG_OLIMEX_STM32_H405=n
# CONFIG_MPS2=n
# CONFIG_RASPI=n
# CONFIG_DIGIC=n
# CONFIG_SABRELITE=n
# CONFIG_EMCRAFT_SF2=n
# CONFIG_MICROBIT=n
# CONFIG_FSL_IMX25=n
# CONFIG_FSL_IMX7=n
# CONFIG_FSL_IMX6UL=n
# CONFIG_ALLWINNER_H3=n

===== ./include/hw/arm/calypso/calypso_spi.h =====
/*
 * calypso_spi.h — Calypso SPI + TWL3025 ABB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_CALYPSO_SPI_H
#define HW_SSI_CALYPSO_SPI_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_CALYPSO_SPI "calypso-spi"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoSPIState, CALYPSO_SPI)

struct CalypsoSPIState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq     irq;

    /* Registers matching real Calypso SPI layout */
    uint16_t set1;      /* 0x00 SET1 */
    uint16_t set2;      /* 0x02 SET2 */
    uint16_t ctrl;      /* 0x04 CTRL */
    uint16_t status;    /* 0x06 STATUS */
    uint16_t tx_data;   /* 0x08/0x0A TX_LSB/MSB */
    uint16_t rx_data;   /* 0x0C/0x0E RX_LSB/MSB */

    /* TWL3025 shadow registers (256 possible addresses) */
    uint16_t abb_regs[256];
};

/* TWL3025 important register addresses */
#define ABB_VRPCDEV    0x01
#define ABB_VRPCSTS    0x02
#define ABB_VBUCTRL    0x03
#define ABB_VBDR1      0x04
#define ABB_TOGBR1     0x09
#define ABB_TOGBR2     0x0A
#define ABB_AUXLED     0x17
#define ABB_ITSTATREG  0x1B

/* SPI status bits (real Calypso) */
#define SPI_STATUS_RE        (1 << 1)   /* Ready / transfer complete */

/* Legacy compat */
#define SPI_STATUS_TX_READY  SPI_STATUS_RE
#define SPI_STATUS_RX_READY  SPI_STATUS_RE

#endif /* HW_SSI_CALYPSO_SPI_H */

===== ./include/hw/arm/calypso/calypso_uart.h =====
/*
 * calypso_uart.h — Calypso UART device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_CALYPSO_UART_H
#define HW_CHAR_CALYPSO_UART_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_CALYPSO_UART "calypso-uart"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoUARTState, CALYPSO_UART)

/*
 * Large RX FIFO to tolerate Compal/sercomm bursts.
 */
#define CALYPSO_UART_RX_FIFO_SIZE 8192

typedef struct CalypsoUARTState {
    SysBusDevice parent_obj;

    /* MMIO */
    MemoryRegion iomem;

    /* QEMU backend — PTY control */
    CharBackend chr;
    qemu_irq irq;

    /* Debug label ("modem", "irda") */
    char *label;

    /* Base registers */
    uint8_t ier;
    uint8_t iir;
    uint8_t fcr;
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr;
    uint8_t msr;
    uint8_t spr;
    uint8_t dll;
    uint8_t dlh;
    uint8_t mdr1;

    /* Extended/banked registers used by Calypso loader/uart driver */
    uint8_t efr;
    uint8_t xon1;
    uint8_t xon2;
    uint8_t xoff1;
    uint8_t xoff2;
    uint8_t scr;
    uint8_t ssr;

    /* RX FIFO */
    uint8_t rx_fifo[CALYPSO_UART_RX_FIFO_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;
    uint16_t rx_count;

    /* TX empty fires once per THR transition */
    bool thr_empty_pending;

    /* TX burst drain: count consecutive IIR(TX_EMPTY) reads without
     * a THR write.  Allows firmware ISR to loop and drain multiple
     * bytes per invocation.  Clear pending only after 2 reads without
     * a write (ISR has nothing left to send). */
    uint8_t tx_empty_reads;

    /* Periodic RX poll timer — works around QEMU not delivering
     * chardev input while the CPU runs in a tight loop. */
    QEMUTimer *rx_poll_timer;

} CalypsoUARTState;

/* Char backend callbacks */
int calypso_uart_can_receive(void *opaque);
void calypso_uart_receive(void *opaque, const uint8_t *buf, int size);

/* Inject bytes directly into RX FIFO, bypassing sercomm DLCI parser.
 * Used by l1ctl_sock to avoid interference with bridge DLCI 4 parsing. */
void calypso_uart_inject_raw(CalypsoUARTState *s, const uint8_t *buf, int size);

/* Force IRQ re-evaluation if RX data is pending */
void calypso_uart_kick_rx(CalypsoUARTState *s);
void calypso_uart_fifo_push(CalypsoUARTState *s, uint8_t data);

/* Tell the chardev backend we can accept more data. */
void calypso_uart_poll_backend(CalypsoUARTState *s);

/* Nudge TX: if TX_EMPTY IRQ is enabled, set pending to trigger ISR.
 * This ensures queued sercomm data gets drained even without console output. */
void calypso_uart_kick_tx(CalypsoUARTState *s);
void calypso_uart_force_init(CalypsoUARTState *s);

/* L1CTL socket — sercomm↔L1CTL relay */
void l1ctl_sock_init(CalypsoUARTState *uart, const char *path);
void l1ctl_sock_uart_tx_byte(uint8_t byte);
bool l1ctl_client_active(void);
bool l1ctl_burst_mode(void);
void l1ctl_set_burst_mode(bool on);

#endif /* HW_CHAR_CALYPSO_UART_H */

===== ./include/hw/arm/calypso/calypso_timer.h =====
/*
 * calypso_timer.h — Calypso GP/Watchdog Timer QOM device
 *
 * 16-bit down-counter with auto-reload and IRQ support.
 * Clocked from 13 MHz / (prescaler + 1).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_CALYPSO_TIMER_H
#define HW_TIMER_CALYPSO_TIMER_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_CALYPSO_TIMER "calypso-timer"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoTimerState, CALYPSO_TIMER)

struct CalypsoTimerState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    QEMUTimer    *timer;
    qemu_irq     irq;

    uint16_t load;        /* Reload value */
    uint16_t count;       /* Current counter */
    uint16_t ctrl;        /* Control: bit0=start, bit1=auto-reload, bit2=irq-en */
    uint16_t prescaler;
    int64_t  tick_ns;     /* Nanoseconds per tick */
    bool     running;
};

#endif /* HW_TIMER_CALYPSO_TIMER_H */

===== ./include/hw/arm/calypso/sercomm_gate.h =====
/*
 * sercomm_gate.h — Sercomm DLCI router (HDLC demux)
 *
 * Emulates the hardware separation between:
 *   - BSP path (bursts) : DLCI 4 → calypso_trx_rx_burst → c54x BSP
 *   - UART path (L1CTL) : all other DLCIs → re-wrap → FIFO → firmware
 *
 * On real Calypso, bursts come from the ABB via BSP hardware, not UART.
 * In QEMU, the bridge sends them as sercomm DLCI 4 on the PTY, so the
 * gate intercepts them and routes to the BSP emulation layer.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef SERCOMM_GATE_H
#define SERCOMM_GATE_H

#include "hw/arm/calypso/calypso_uart.h"

/* Feed raw bytes from PTY into the sercomm HDLC parser.
 * Complete frames are routed by DLCI:
 *   4 (bursts) → calypso_trx_rx_burst (BSP emulation)
 *   * (L1CTL, debug, console) → re-wrap → UART FIFO (firmware) */
void sercomm_gate_feed(CalypsoUARTState *s, const uint8_t *buf, int size);

#endif /* SERCOMM_GATE_H */

===== ./include/hw/arm/calypso/calypso_soc.h =====
/*
 * calypso_soc.h - TI Calypso System-on-Chip
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_CALYPSO_SOC_H
#define HW_ARM_CALYPSO_SOC_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/arm/calypso/calypso_inth.h"
#include "hw/arm/calypso/calypso_timer.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_spi.h"

#define TYPE_CALYPSO_SOC "calypso-soc"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoSoCState, CALYPSO_SOC)

#define CALYPSO_SOC_NUM_IRQS  2

struct CalypsoSoCState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iram;

    CalypsoINTHState  inth;
    CalypsoTimerState timer1;
    CalypsoTimerState timer2;
    CalypsoUARTState  uart_modem;
    CalypsoUARTState  uart_irda;
    CalypsoSPIState   spi;

    void *trx;

    qemu_irq cpu_irq;
    qemu_irq cpu_fiq;

    /* IRAM-at-zero alias (controlled by CNTL register) */
    MemoryRegion iram_alias;
    bool iram_at_zero;
    MemoryRegion cntl_iomem;
    uint16_t extra_conf;

};

#endif /* HW_ARM_CALYPSO_SOC_H */

===== ./include/hw/arm/calypso/calypso_trx.h =====
/*
 * calypso_trx.h — Calypso DSP/TPU hardware emulation
 * Pure hardware — no sockets, no DSP processing. Firmware does everything.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_TRX_H
#define CALYPSO_TRX_H

#include "hw/irq.h"
#include "exec/memory.h"

/* IRQ map */
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

/* Hardware addresses */
#define CALYPSO_DSP_BASE      0xFFD00000
#define CALYPSO_DSP_SIZE      (64 * 1024)
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

/* TPU register offsets */
#define TPU_CTRL              0x0000
#define TPU_INT_CTRL          0x0002
#define TPU_INT_STAT          0x0004
#define TPU_OFFSET            0x000C
#define TPU_SYNCHRO           0x000E
#define TPU_IT_DSP_PG         0x0020

/* TPU_CTRL bits */
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

/* TSP */
#define TSP_RX_REG            0x08

/* ULPD */
#define ULPD_SETUP_CLK13      0x00
#define ULPD_COUNTER_HI       0x1C
#define ULPD_COUNTER_LO       0x1E
#define ULPD_GAUGING_CTRL     0x24
#define ULPD_GSM_TIMER        0x28

/* GSM timing — real TDMA frame period: 13MHz / 60000 ≈ 4.615ms */
#define GSM_TDMA_NS           4615000
#define GSM_HYPERFRAME        2715648

/* DSP boot */
#define DSP_DL_STATUS_ADDR    0x0FFE
#define DSP_API_VER_ADDR      0x01B4
#define DSP_API_VER2_ADDR     0x01B6
#define DSP_DL_STATUS_RESET   0x0000
#define DSP_DL_STATUS_BOOT    0x0001
#define DSP_DL_STATUS_READY   0x0002
#define DSP_API_VERSION       0x3606

void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs);
void calypso_stub_console(void);

/* Sercomm burst transport (DLCI 4) — called by UART hardware */
void calypso_trx_rx_burst(const uint8_t *data, int len);
bool calypso_trx_burst_ready(void);
void calypso_trx_tx_burst_poll(void);

#endif /* CALYPSO_TRX_H */

===== ./include/hw/arm/calypso/calypso_inth.h =====
/*
 * calypso_inth.h — Calypso INTH (Interrupt Handler) QOM device
 *
 * Two-level interrupt controller with 32 IRQ lines,
 * priority-based arbitration, and IRQ/FIQ routing.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INTC_CALYPSO_INTH_H
#define HW_INTC_CALYPSO_INTH_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_CALYPSO_INTH "calypso-inth"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoINTHState, CALYPSO_INTH)

#define CALYPSO_INTH_NUM_IRQS  32

struct CalypsoINTHState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;

    /* Output lines to CPU */
    qemu_irq parent_irq;   /* CPU IRQ line */
    qemu_irq parent_fiq;   /* CPU FIQ line */

    /* Interrupt Level Registers: bits[4:0]=priority, bit[8]=FIQ */
    uint16_t ilr[CALYPSO_INTH_NUM_IRQS];

    uint16_t ith_v;        /* Current highest-priority active IRQ number */
    int irq_in_service;    /* IRQ being serviced (-1 = none). Set on IRQ_NUM read,
                            * cleared on IRQ_CTRL write. Prevents ith_v update
                            * so IRQ_CTRL acks the correct interrupt. */
    uint32_t levels;       /* Bitmask of current input levels (level-sensitive) */
    uint32_t mask;         /* Bitmask: 1 = masked (disabled) */
};

#endif /* HW_INTC_CALYPSO_INTH_H */

===== ./BUGS_AND_FIXES.md =====
# C54x DSP Emulator - All Bugs and Fixes

## Session 2026-04-05 (this context)

### Bug 1: SP leak via RPTB EXIT
- **Symptom**: SP incremented +1 at each RPTB EXIT (0xe999->0xe9b0)
- **Root cause**: RPTB itself was not the issue, the SP leak came from other instructions
- **Fix**: Added SP change detector to identify culprit instructions

### Bug 2: SP corruption via STH B writing to MMR zone
- **Symptom**: SP drops from 0x5AC8 to 0x0000 at PC=0x8a46
- **Root cause**: STH B with DP=0 writes B(high)=0 to addr 0x18 = SP register (MMR)
- **Fix**: Set SP init to 0x5AC8 (Calypso boot ROM value). Later fixed properly via CALLD/RPTBD corrections.

### Bug 3: PROM1 mirror stopping at 0xFF7F
- **Symptom**: Interrupt vectors at 0xFF80-0xFFFF empty
- **Root cause**: Mirror range was `addr16 >= 0xE000 && addr16 < 0xFF80`
- **Fix**: Changed to `addr16 >= 0xE000` (include vectors)

### Bug 4: Interrupt vectors 1-7 overwritten by IDLE stubs
- **Symptom**: TINT0 and other interrupts branch to IDLE instead of real handlers
- **Root cause**: c54x_reset() installed IDLE at vec 1-7 positions, overwriting PROM1 ROM vectors
- **Fix**: Only install vec 0 (reset). Vec 1-7 come from PROM1 ROM mirror.

### Bug 5: F272 decoded as NORM instead of RPTBD
- **Symptom**: Code at 0x76FB consumed 1 word instead of 2, corrupting instruction flow
- **Root cause**: F272 = RPTBD (repeat block delayed, 2 words) per tic54x-opc.c. Was incorrectly decoded as NORM (normalize accumulator).
- **Fix**: Added specific checks for F272=RPTBD, F274=CALLD, F273=RETD before LMS handler

### Bug 6: F274 decoded as NORM instead of CALLD
- **Symptom**: Code at 0x7702 treated as normalize instead of delayed call
- **Root cause**: Same as Bug 5 - F274 is CALLD pmad (delayed call, 2 words)
- **Fix**: F274 now pushes PC+2 and branches to pmad

### Bug 7: Indirect addressing modes 0xC-0xF not consuming 2nd word
- **Symptom**: Instructions with `*(lk)`, `*+ARn(lk)`, `*ARn(lk)`, `*+ARn(lk)%` used AR as address without offset, and the offset word was executed as next instruction
- **Root cause**: resolve_smem had `/* handled by caller */` for modes C-F but no caller actually handled them
- **Fix**: resolve_smem now reads prog_fetch(s, s->pc+1) for modes C-F, sets s->lk_used=true. All `return consumed` changed to `return consumed + s->lk_used`.

### Bug 8: BANZ testing old AR value
- **Symptom**: BANZ at 0xB36B branching incorrectly
- **Root cause**: Code saved `old_arp_val = s->ar[arp(s)]` BEFORE resolve_smem modified AR, then tested old value
- **Fix**: Test `s->ar[arp(s)]` AFTER resolve_smem

### Bug 9: F6xx instructions treated as NOP
- **Symptom**: MVDD (data-to-data move) silently dropped
- **Root cause**: F6xx handler only had cases for sub=2 and sub=6, rest fell to NOP
- **Fix**: Added MVDD for sub >= 0x8

### Bug 10: RPT/RPTB interaction
- **Symptom**: RPT inside RPTB causes infinite loop - RPTB redirect fires during RPT
- **Root cause**: RPTB check (`pc == rea + 1`) runs before exec and redirects PC during active RPT
- **Fix**: Skip RPTB check when `rpt_active` is true

### Bug 11: RPT F5xx return value
- **Symptom**: Changing RPT from `s->pc += 1; return 0` to `return 1` caused RPT to re-execute itself infinitely
- **Root cause**: RPT handler in main loop does `continue` (skips pc += consumed). With return 1, PC never advances past RPT instruction.
- **Fix**: Reverted to `s->pc += 1; return 0` - RPT must advance PC itself since the main loop's RPT handler will prevent further PC advance.

### Bug 12: NORM bit extraction
- **Symptom**: NORM checking wrong bits of accumulator
- **Root cause**: Was checking bits 31/30 of 24-bit masked value instead of bits 39/38 of 40-bit accumulator
- **Fix**: Changed to `(*acc >> 39) & 1` and `(*acc >> 38) & 1`

### Bug 13: L1CTL RESET_IND lost before client connects
- **Symptom**: ccch_scan connects but never receives RESET_IND
- **Root cause**: QEMU boots immediately, firmware sends RESET_IND before client is listening
- **Fix**: QEMU starts with `-S` (paused). `vm_start()` called when client connects.

### Bug 14: L1CTL reconnect stuck at "waiting for firmware"
- **Symptom**: Kill and relaunch mobile -> stuck at "waiting for firmware"
- **Root cause**: `cli_rx_enabled = false` on reconnect, firmware won't re-send RESET_IND
- **Fix**: On reconnect (VM already running), enable `cli_rx_enabled` immediately

### Bug 15: soft_to_int16 overflow in sercomm_udp.py
- **Symptom**: soft_bit=255 produces -33025 (outside int16 range)
- **Root cause**: No clamping
- **Fix**: `max(-32768, min(32767, ...))`

### Bug 16: Boot ROM at prog[0x0000-0x007F] empty
- **Symptom**: CALA to A(low)=0x0000 executes garbage (all zeros)
- **Root cause**: Internal Calypso boot ROM not in PROM dump. prog[0x0000] was uninitialized.
- **Fix**: Fill prog[0x0000-0x007F] with RET (0xF073) stubs

### Bug 17: Duplicate F0/F1 handler blocks
- **Symptom**: Second F0/F1 handler block is dead code, never reached
- **Root cause**: First `if (hi8 == 0xF0 || hi8 == 0xF1)` block catches everything
- **Status**: Identified, not yet cleaned up

### Bug 18: TINT0 timer scattered across files
- **Symptom**: Timer logic in calypso_trx.c mixed with DSP/ARM glue code
- **Root cause**: No dedicated timer module
- **Fix**: Created calypso_tint0.c/h as master clock module. Removed calypso_tdma_hw.c/h (slave, never connected).

## Session 2026-04-05 night

### Bug 19: F4EB = RETE (alternate encoding)
- **Symptom**: Interrupt vector stubs (F4EB F495 F495 F495) fell through to BD handler, branching to 0xF495
- **Root cause**: F4EB not recognized as RETE. Per tic54x-opc.c it's the real RETE encoding.
- **Fix**: Added exact-match F4EB handler with APTS-aware pop

### Bug 20: RPTBD (F272) RSA = PC+2 instead of PC+4
- **Symptom**: Delay slots executed inside the repeat loop instead of once before it. Init MVDD sweep ran 64x corrupting MMRs. Dispatch RPTB at 0x8F8B included STM in loop body.
- **Root cause**: RPTBD is delayed — 2 delay slots after the 2-word instruction. RSA must be PC+4, not PC+2.
- **Fix**: Changed `s->rsa = (uint16_t)(s->pc + 4)` in both F272 handlers. THIS was the root cause of init MMR corruption and DSP never reaching IDLE.

### Bug 21: F4xx arithmetic decoded as branch/call
- **Symptom**: DSP never reached IDLE, infinite dispatch loops, stack corruption. F48E (EXP) treated as CALLD (push+branch), F483 (SAT) as unknown, F48C (MPYA) as CALLD, etc.
- **Root cause**: F4xx switch used `sub = (op >> 4) & 0xF` mapping sub=0→B, sub=6→CALL, sub=8→CALLD. But per tic54x-opc.c, F400-F4BF are arithmetic instructions (ADD, SUB, LD, SFTA, SFTL, EXP, NORM, MPYA, SAT, NEG, ABS, CMPL, RND, MAX, MIN, SUBC, ROR, ROL, MACA, SQUR).
- **Fix**: Replaced nibble switch with exact-match/mask-based handlers for all F4xx arithmetic. Added 20+ instruction implementations.

### Bug 22: MAR (0x6D) not implemented
- **Symptom**: 0x6DEC at 0xE906 treated as UNIMPL (1 word). Next word (0x0003 = lk) executed as instruction, causing PC misalignment and d_dsp_page corruption.
- **Root cause**: 0x6Dxx = MAR (Modify Address Register) per tic54x-opc.c. Was missing from decoder.
- **Fix**: Added MAR handler — calls resolve_smem for side effects only (AR modification), no data access.

### Bug 23: CC/BC condition tables wrong
- **Symptom**: Unknown conditions defaulted to `take=true`, causing wrong branches. cond=0x05 was always taken instead of AEQ (A==0).
- **Root cause**: Condition codes followed wrong mapping. Per tic54x-dis.c cc2[]: 0x02=AGEQ, 0x03=ALT, 0x04=ANEQ, 0x05=AEQ, etc.
- **Fix**: Rewrote CC and BC with correct switch table. Unknown compound conditions still default to true.

### Bug 24: prog_write allows PROM1 corruption
- **Symptom**: MVDP/WRITA could overwrite ROM at 0xE000-0xFFFF
- **Root cause**: No protection in prog_write for PROM1 range
- **Fix**: `if (addr16 >= 0xE000) return;`

### Bug 25: d_dsp_page not synced to DSP API RAM
- **Symptom**: DSP read d_dsp_page=0x0000 always, never saw ARM's 0x0002/0x0003
- **Root cause**: Sync to api_ram was inside SINT17 block (gated on dsp_init_done). Before init, DSP never saw the page value.
- **Fix**: Sync api_ram[0xD4] on ARM write to dsp_ram[0x01A8/2], not per-tick.

### Bug 26: BRINT0 not fired
- **Symptom**: DSP stuck in frame processing waiting for BSP data that never arrives
- **Root cause**: c54x_bsp_load loaded samples but never fired BRINT0 interrupt
- **Fix**: Fire c54x_interrupt_ex(dsp, 21, 5) after bsp_load, gated on dsp_init_done

### Bug 27: Double-nested include headers
- **Symptom**: include/hw/arm/calypso/calypso/ had divergent copies (GSM_TDMA_NS 10x off, missing irq_in_service)
- **Root cause**: Leftover from refactoring
- **Fix**: Removed nested calypso/calypso/ directory

===== ./run_tmp.sh =====
#!/bin/bash
# run_tmp.sh — Lance fake_trx + BTS + QEMU + POWERON + mobile en tmux
set -e

SESSION="osmocom"
TMUX_SOCKET="/tmp/osmocom_tmux"

echo "=== [1] Kill old processes ==="
pkill -9 qemu-system-arm 2>/dev/null || true
pkill -9 mobile 2>/dev/null || true
pkill -f l1ctl_bridge 2>/dev/null || true
pkill -9 -f fake_trx 2>/dev/null || true
systemctl stop osmo-bts-trx 2>/dev/null || true
tmux -S "$TMUX_SOCKET" kill-session -t "$SESSION" 2>/dev/null || true
sleep 1
rm -f /tmp/osmocom_l2* /tmp/qemu-calypso-mon*.sock /tmp/osmocom_tmux

echo "=== [2] Create tmux ==="
tmux -S "$TMUX_SOCKET" new-session -d -s "$SESSION" -n faketrx
tmux -S "$TMUX_SOCKET" new-window -t "$SESSION" -n qemu0
tmux -S "$TMUX_SOCKET" new-window -t "$SESSION" -n ue_g1

echo "=== [3] Start fake_trx ==="
tmux -S "$TMUX_SOCKET" send-keys -t "$SESSION:faketrx" \
  "python3 /opt/GSM/osmocom-bb/src/target/trx_toolkit/fake_trx.py" C-m
sleep 3

echo "=== [4] Start osmo-bts-trx ==="
systemctl start osmo-bts-trx
sleep 5

echo "=== [5] Send RXTUNE+TXTUNE+POWERON to fake_trx MS (6701) ==="
echo -ne 'CMD RXTUNE 1805600\0' | socat - UDP:127.0.0.1:6701
echo -ne 'CMD TXTUNE 1710600\0' | socat - UDP:127.0.0.1:6701
echo -ne 'CMD POWERON\0' | socat - UDP:127.0.0.1:6701
sleep 1

echo "=== [6] Start QEMU ==="
tmux -S "$TMUX_SOCKET" send-keys -t "$SESSION:qemu0" \
  "CALYPSO_TRX_PORT=0 /opt/GSM/qemu/build/qemu-system-arm -M calypso -cpu arm946 -display none -serial pty -serial pty -monitor unix:/tmp/qemu-calypso-mon-ms1.sock,server,nowait -kernel /opt/GSM/firmware/board/compal_e88/layer1.highram.elf 2>&1 | tee /var/log/osmocom/qemu.log" C-m
sleep 5

echo "=== [6b] Detect PTY and start l1ctl_bridge ==="
PTY=$(grep -o '/dev/pts/[0-9]*' /var/log/osmocom/qemu.log 2>/dev/null | head -1)
if [ -z "$PTY" ]; then
  echo "ERROR: no PTY detected"
else
  echo "PTY=$PTY"
  # Send cont to QEMU monitor
  printf 'cont\n' | socat - UNIX-CONNECT:/tmp/qemu-calypso-mon-ms1.sock 2>/dev/null || true
  sleep 2
  tmux -S "$TMUX_SOCKET" new-window -t "$SESSION" -n bridge
  tmux -S "$TMUX_SOCKET" send-keys -t "$SESSION:bridge" \
    "cd /opt/GSM/qemu && BRIDGE_AIR_PORT=6702 python3 l1ctl_bridge.py $PTY /tmp/osmocom_l2_1" C-m
fi
sleep 10

echo "=== [7] Start mobile ==="
tmux -S "$TMUX_SOCKET" send-keys -t "$SESSION:ue_g1" \
  "mobile -c /root/.osmocom/bb/mobile_group1.cfg" C-m

echo ""
echo "╔════════════════════════════════════════════════════╗"
echo "║  Attach: tmux -S $TMUX_SOCKET attach               ║"
echo "║  Windows: faketrx | qemu0 | ue_g1                 ║"
echo "║  QEMU log:   /tmp/qemu-bridge.log                 ║"
echo "║  Bridge log: /var/log/osmocom/bridge.log           ║"
echo "║                                                    ║"
echo "║  tail -f /tmp/qemu-bridge.log  (dans un autre term)║"
echo "╚════════════════════════════════════════════════════╝"

===== ./code.txt =====

===== ./PROJECT_STATUS.md =====
# QEMU Calypso GSM Phone Emulator - Project Status

## Goal
Emulate a Calypso GSM phone in QEMU: ARM7 CPU running OsmocomBB layer1 firmware + TMS320C54x DSP running the Calypso DSP ROM. The emulated phone connects to osmo-bts-trx (GSM BTS) and registers on the network.

## Architecture

```
                    osmo-bsc
                       |
                  osmo-bts-trx
                       |
              sercomm_udp.py (bridge)
                  UDP 5700-5702
                       |
              QEMU (calypso machine)
              +-------------------+
              | ARM7 (layer1.elf) |
              |        |          |
              | API RAM (shared)  |
              |        |          |
              | C54x DSP (ROM)    |
              +-------------------+
                       |
               L1CTL socket
               /tmp/osmocom_l2_1
                       |
              mobile / ccch_scan
```

## Components

### Working
- QEMU Calypso machine (calypso_mb.c, calypso_soc.c)
- ARM CPU boots layer1.highram.elf
- DSP ROM loaded from calypso_dsp.txt (dumped from real phone)
- UART sercomm (firmware <-> external world)
- L1CTL socket (firmware <-> mobile/ccch_scan)
- sercomm_udp.py bridge (UART <-> BTS UDP)
- DSP bootloader protocol (ARM uploads code, jumps to 0x7000)
- MVPD simulation (PROM0 -> DARAM overlay copy)
- PROM1 mirror to 0xE000-0xFFFF including interrupt vectors
- Boot ROM stubs at prog[0x0000-0x007F]
- TINT0 master clock (calypso_tint0.c)
- osmo-bts-trx receives clock indications and configures
- DSP instruction decoding (~98% of C54x ISA)
  - Full F4xx arithmetic: EXP, NORM, MPYA, SAT, NEG, ABS, CMPL, RND, MAX, MIN, SUBC, SFTL, ROR, ROL, MACA, SQUR, ADD, SUB, LD, SFTA
  - MAR (0x6D) modify address register
  - F4EB = RETE (alternate encoding), SSBX/RSBX range
  - RPTBD RSA = PC+4 (delay slots correct)
  - CC/BC conditional tables per tic54x-dis.c
  - All indirect addressing modes 0xC-0xF with lk offset
  - CMPS (Viterbi), FIRS (FIR filter), LMS, MVDD
- DSP init: boots, reaches IDLE at PC=0x770C (dispatch idle) at fn=8
  - SP=0x5AC8 after cleanup, stable at 0x0024-0x0027 during processing
  - IMR properly configured by DSP code (0xE7FF, 0xFFFF)
  - Returns to IDLE between frames
- d_dsp_page alternates 0x0002/0x0003 correctly
- L1CTL: RESET_IND/CONF, PM_REQ/CONF, FBSB_REQ/CONF all work
- FBSB_CONF sent with result=2, bsic=2
- Zero UNIMPL opcodes, zero F4xx unhandled

### Partially Working
- DSP frame processing
  - SINT17 dispatches frame processing correctly
  - DSP runs MAC/FIR/equalizer code (0x81xx, 0xF4xx subroutines)
  - Does not return to IDLE after first SINT17 (needs burst data)
- Burst pipeline
  - BRINT0 (vec 21, bit 5) fires when BSP samples loaded
  - c54x_bsp_load stores samples in BSP buffer
  - DSP reads via PORTR PA=0x0000
  - Bridge runs but no DL bursts received from BTS yet

### Not Yet Working
- DL burst reception (bridge → DSP BSP pipeline)
- DSP completing frame processing (returning to IDLE after SINT17)
- API IRQ (IRQ15) to ARM after frame completion
- DATA_IND to mobile/ccch_scan
- Frequency burst detection (FBSB result=0)
- SCH decode, CCCH decode
- Network registration
- Voice/data

## Key Bug History (55+ bugs fixed across sessions)

### Session 2026-04-04 (22 bugs)
- IDLE wake on any interrupt (masked or unmasked)
- INTH irq_in_service tracking
- UART address mapping
- sercomm DLCI filter
- PROM1 mirror, OVLY range
- F8xx branch before RPT fallthrough
- XPC not used in prog_fetch
- Bootloader protocol (BL_CMD_STATUS)
- STH/STL, DELAY, EA/BANZ conflict
- ST0 init, SP init, CALL push order
- 12 opcode fixes from SPRU172C
- ROM write protection

### Session 2026-04-05 day (20 bugs)
- BC/BCD conditional branch placement
- STLM B (0xAB), RPTB C6/C7 encodings
- PROM0 XPC threshold fix
- dsp_init_done gate for SINT17
- F272=RPTBD, F274=CALLD, F273=RETD
- Indirect addressing lk_used (modes C-F)
- BANZ AR test order
- RPTB/RPT priority (skip RPTB during active RPT)
- NORM bit 39/38 extraction
- F6xx MVDD implementation
- PROM1 vectors not overwritten
- Boot ROM stubs
- L1CTL reconnect handling
- soft_to_int16 clamping
- TINT0 master clock extraction

### Session 2026-04-05 night (13+ bugs)
- F4EB = RETE (alternate encoding per tic54x-opc.c)
- **RPTBD (F272) RSA = PC+4** — delay slots were inside the loop instead of before it. Root cause of init sweep corruption and infinite dispatch loops.
- **F4xx arithmetic decoder rewrite** — entire F400-F4BF range was decoded as B/CALL/CALLD via nibble switch. Actually arithmetic instructions (ADD, SUB, LD, SFTA, SFTL, EXP, NORM, MPYA, SAT, NEG, ABS, CMPL, RND, MAX, MIN, SUBC, ROR, ROL, MACA, SQUR). This was THE critical bug preventing DSP from reaching IDLE and doing real signal processing.
- MAR (0x6D) instruction implemented — was UNIMPL causing PC misalignment in PROM1 code
- CC/BC condition tables corrected per tic54x-dis.c cc2[] (ALT, AEQ, ALEQ, etc.)
- prog_write PROM1 (E000-FFFF) protection — prevent ROM corruption
- d_dsp_page API RAM sync on ARM write (not every tick)
- BRINT0 (vec 21, bit 5) fires on BSP sample load, gated on dsp_init_done
- SSBX/RSBX catch-all for F4E0-F4FF status bit range
- Double-nested include headers removed (calypso/calypso/ directory)
- build.sh script with auto-timestamp and md5 verification

## Docker Setup
- Container: `trying` (image: osmo-qemu)
- QEMU source: `/root/qemu/`
- DSP ROM: `/opt/GSM/calypso_dsp.txt`
- Firmware: `/opt/GSM/firmware/board/compal_e88/layer1.highram.elf`
- Launch: `/root/qemu/run.sh`
- Build: `/home/nirvana/qemu-src/build.sh` (auto-sync, timestamp, md5)
- Host mirror: `/home/nirvana/qemu-src/hw/arm/calypso/`
- Git repo: `/home/nirvana/qemu-calypso/`

## Next Steps
1. Fix DL burst pipeline (osmo-bts-trx → bridge → calypso_trx_rx_burst)
2. DSP returns to IDLE after frame processing (with burst data)
3. API IRQ fires → DATA_IND to mobile/ccch_scan
4. FBSB detection (result=0 instead of 2)
5. SCH/CCCH decode
6. Network registration

===== ./hw/arm/calypso/calypso_mb.c =====
/*
 * calypso_mb.c - Calypso development board machine
 * DEBUG BUILD — verbose flash/memory debug
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/block/flash.h"
#include "hw/char/serial.h"
#include "sysemu/sysemu.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "qemu/error-report.h"
#include "exec/address-spaces.h"
#include "elf.h"
#include "target/arm/cpu.h"
#include "sysemu/reset.h"

#include "hw/arm/calypso/calypso_soc.h"

#define CALYPSO_XRAM_BASE     0x01000000
#define CALYPSO_XRAM_SIZE     (8 * 1024 * 1024)

#define CALYPSO_FLASH_BASE    0x00000000
#define CALYPSO_FLASH_SIZE    (4 * 1024 * 1024)

typedef struct CalypsoMachineState {
    MachineState parent;
    ARMCPU *cpu;
    CalypsoSoCState soc;
    MemoryRegion xram;
    MemoryRegion bootrom;
} CalypsoMachineState;

#define TYPE_CALYPSO_MACHINE MACHINE_TYPE_NAME("calypso")
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoMachineState, CALYPSO_MACHINE)

/*
 * Firmware patches applied after ROM blobs are loaded into memory.
 * Called from qemu_system_reset() which runs after machine_init.
 *
 * 1) NOP cons_puts: prevents console output from filling the 32-slot
 *    msgb pool, which causes talloc panic during boot.
 *
 * 2) Talloc panic → retry with IRQs: if the pool fills despite (1),
 *    re-enable IRQs and retry instead of halting. The NOP at the
 *    cons_puts call site prevents recursive allocation.
 *
 * 3) handle_abort → loop with IRQs enabled: prevents a stray data
 *    abort from permanently disabling IRQs and halting the system.
 */
static void calypso_machine_init(MachineState *machine)
{
    CalypsoMachineState *s = CALYPSO_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    Error *err = NULL;

    fprintf(stderr, "[MB] === calypso_machine_init START ===\n");

    /* ---- CPU ---- */
    cpuobj = object_new(machine->cpu_type);
    s->cpu = ARM_CPU(cpuobj);
    if (!qdev_realize(DEVICE(cpuobj), NULL, &err)) {
        error_report_err(err);
        exit(1);
    }

    /* ---- SoC ---- */
    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_CALYPSO_SOC);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->soc), &err)) {
        error_report_err(err);
        exit(1);
    }

    sysbus_connect_irq(SYS_BUS_DEVICE(&s->soc), 0,
        qdev_get_gpio_in(DEVICE(&s->cpu->parent_obj), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->soc), 1,
        qdev_get_gpio_in(DEVICE(&s->cpu->parent_obj), ARM_CPU_FIQ));

    /* ---- External RAM ---- */
    memory_region_init_ram(&s->xram,
                           OBJECT(&s->soc.parent_obj),
                           "calypso.xram",
                           CALYPSO_XRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, CALYPSO_XRAM_BASE, &s->xram);
    fprintf(stderr, "[MB] XRAM @ 0x%08x (%d MiB)\n",
            CALYPSO_XRAM_BASE, CALYPSO_XRAM_SIZE / (1024*1024));

    /* ---- Flash NOR @ 0x00000000 ----
     *
     * Real Compal E88: Intel 28F320 (4 MiB) on CS0 at 0x00000000.
     * 16-bit bus width (Calypso CS0 is 16-bit).
     * Manufacturer 0x0089 = Intel, Device 0x0018 = 28F320J3.
     * 64 KiB sectors.
     *
     * The loader does CFI queries here. If there's no pflash or
     * something else shadows this address, we get "Failed to
     * initialize flash!".
     */
    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);

    fprintf(stderr, "[MB] Flash: registering pflash_cfi01 @ 0x%08x\n",
            CALYPSO_FLASH_BASE);
    fprintf(stderr, "[MB]   size=%d MiB, sector=64K, width=2 (16-bit)\n",
            CALYPSO_FLASH_SIZE / (1024*1024));
    fprintf(stderr, "[MB]   mfr=0x0089 (Intel), dev=0x0018 (28F320J3)\n");
    fprintf(stderr, "[MB]   drive=%s\n", dinfo ? "attached" : "NONE (blank 0xFF)");

    pflash_cfi01_register(CALYPSO_FLASH_BASE,
                          "calypso.flash",
                          CALYPSO_FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          64 * 1024,   /* sector size */
                          1,           /* 8-bit bus width */
                          0x0089,      /* Intel */
                          0x0018,      /* 28F320J3 */
                          0, 0, 0);

    fprintf(stderr, "[MB] Flash: pflash_cfi01 registered OK\n");

    /* ---- Synthetic boot ROM at address 0 ----
     *
     * The real Calypso has internal ROM at 0x00000000 containing
     * exception vector stubs that branch to IRAM exception handlers.
     * OsmocomBB firmware installs handlers at IRAM+0x1C through IRAM+0x34.
     * The boot ROM vectors use: ldr pc, [pc, #0x18] + address table.
     *
     * Layout (0x00-0x3F):
     *   0x00-0x1C: ldr pc, [pc, #0x18] for each exception
     *   0x20-0x3C: handler addresses in IRAM
     */
    {
        uint32_t bootrom_data[16];
        /* ARM instruction: ldr pc, [pc, #0x18] = 0xe59ff018 */
        for (int i = 0; i < 8; i++) {
            bootrom_data[i] = 0xe59ff018;
        }
        /* Handler addresses (read via the ldr pc above):
         * Each vector at offset N reads from offset N+0x20 */
        bootrom_data[8]  = 0x00820000;  /* reset → _start */
        bootrom_data[9]  = 0x0080001C;  /* undef → IRAM _undef_instr */
        bootrom_data[10] = 0x00800020;  /* SWI → IRAM _sw_interr */
        bootrom_data[11] = 0x00800024;  /* prefetch abort → IRAM */
        bootrom_data[12] = 0x00800028;  /* data abort → IRAM */
        bootrom_data[13] = 0x0080002C;  /* reserved → IRAM */
        bootrom_data[14] = 0x00800030;  /* IRQ → IRAM _irq */
        bootrom_data[15] = 0x00800034;  /* FIQ → IRAM _fiq */

        memory_region_init_ram(&s->bootrom, NULL,
                                "calypso.bootrom", 64, &error_fatal);
        memory_region_add_subregion_overlap(sysmem, 0x00000000,
                                             &s->bootrom, 1);
        /* Write vector table into the boot ROM RAM */
        {
            void *ptr = memory_region_get_ram_ptr(&s->bootrom);
            memcpy(ptr, bootrom_data, sizeof(bootrom_data));
        }
        fprintf(stderr, "[MB] Boot ROM @ 0x00000000 (64 bytes, exception vectors)\n");
    }

    /* ---- Firmware load ---- */
    if (machine->kernel_filename) {
        uint64_t entry;
        int ret;

        ret = load_elf(machine->kernel_filename, NULL, NULL, NULL,
                       &entry, NULL, NULL, NULL,
                       0, EM_ARM, 1, 0);

        if (ret < 0) {
            ret = load_image_targphys(machine->kernel_filename,
                                      CALYPSO_XRAM_BASE,
                                      CALYPSO_XRAM_SIZE);
            if (ret < 0) {
                error_report("Could not load firmware '%s'",
                             machine->kernel_filename);
                exit(1);
            }
            entry = CALYPSO_XRAM_BASE;
        }

        cpu_set_pc(CPU(s->cpu), entry);

        fprintf(stderr, "[MB] Firmware: '%s'\n", machine->kernel_filename);
        fprintf(stderr, "[MB]   entry=0x%08lx  size=%d bytes\n",
                (unsigned long)entry, ret);

    }

    fprintf(stderr, "[MB] === Machine ready ===\n");
    fprintf(stderr, "[MB]   Flash:  0x%08x–0x%08x (%d MiB pflash_cfi01)\n",
            CALYPSO_FLASH_BASE,
            CALYPSO_FLASH_BASE + CALYPSO_FLASH_SIZE - 1,
            CALYPSO_FLASH_SIZE / (1024*1024));
    fprintf(stderr, "[MB]   IRAM:   0x00800000–0x0083FFFF (256 KiB)\n");
    fprintf(stderr, "[MB]   XRAM:   0x%08x–0x%08x (%d MiB)\n",
            CALYPSO_XRAM_BASE,
            CALYPSO_XRAM_BASE + CALYPSO_XRAM_SIZE - 1,
            CALYPSO_XRAM_SIZE / (1024*1024));
}

static void calypso_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Calypso SoC development board (modular architecture)";
    mc->init = calypso_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm946");
    mc->default_ram_size = 0;
    mc->alias = "calypso-high";
}

static const TypeInfo calypso_machine_info = {
    .name          = TYPE_CALYPSO_MACHINE,
    .parent        = TYPE_MACHINE,
    .instance_size = sizeof(CalypsoMachineState),
    .class_init    = calypso_machine_class_init,
};

static void calypso_machine_register_types(void)
{
    type_register_static(&calypso_machine_info);
}

type_init(calypso_machine_register_types)

===== ./hw/arm/calypso/meson.build =====
# hw/arm/calypso/meson.build
arm_ss.add(when: 'CONFIG_CALYPSO', if_true: files(
  'calypso_mb.c',
  'calypso_soc.c',
  'calypso_trx.c',
  'calypso_c54x.c',
  'l1ctl_sock.c',
  'sercomm_gate.c',
  'calypso_tint0.c',
))

===== ./hw/arm/calypso/doc/C54X_INSTRUCTIONS.md =====
---
name: C54x instruction encoding from SPRU172C
description: Key instruction encodings verified from TI SPRU172C doc, corrects emulator bugs
type: project
---

## XC (Execute Conditionally) — SPRU172C p.4-198
- Opcode: `1111 11N1 CCCCCCCC` (1 word)
- N=0 (bit 9=0) → n=1 instruction = opcode 0xFDxx
- N=1 (bit 9=1) → n=2 instructions = opcode 0xFFxx
- If cond true: execute next n instructions normally
- If cond false: treat next n instructions as NOP
- Condition codes (8-bit, combinable):
  - UNC=0x00, BIO=0x03, NBIO=0x02, C=0x0C, NC=0x08
  - TC=0x30, NTC=0x20
  - AEQ=0x45, ANEQ=0x44, AGT=0x46, AGEQ=0x42, ALT=0x43, ALEQ=0x47
  - BEQ=0x4D, BNEQ=0x4C, BGT=0x4E, BGEQ=0x4A, BLT=0x4B, BLEQ=0x4F
  - AOV=0x70, ANOV=0x60, BOV=0x78, BNOV=0x68

## FRET — SPRU172C p.4-61
- Opcode: `1111 0Z01 1101 0100` (Z=0 normal, Z=1 delayed)
- Execution: (TOS)→XPC, SP+1, (TOS)→PC, SP+1
- **Pops 2 words**: first XPC, then PC

## FCALL — SPRU172C p.4-57
- Opcode: `1111 10Z1 1` + 7-bit pmad(22-16) + 16-bit pmad(15-0) 
- 2 words total
- Execution: SP-1, PC+2→TOS, SP-1, XPC→TOS, pmad(15-0)→PC, pmad(22-16)→XPC
- **Pushes 2 words**: first PC+2, then XPC

## RETE — SPRU172C p.4-140
- Execution: (TOS)→PC, SP+1, 0→INTM
- **Pops 1 word** (just PC), clears INTM

## TRAP K — SPRU172C p.4-195
- Pushes PC+1, branches to interrupt vector K
- Not affected by INTM

## 0xEA = BANZ (confirmed, not XC)

**How to apply:** Fix XC (0xFD/0xFF), FRET (2 pops), FCALL (2 pushes) in calypso_c54x.c

## BANZ — SPRU172C p.4-16
- Opcode: `0111 1Z0I AAAA AAAA` + 16-bit pmad (2 words)
- BANZ = 0x78xx, BANZD = 0x7Axx
- Sind encodes indirect addressing (which AR and modify mode)
- Execution: if (AR[x] != 0) then pmad→PC, else PC+2→PC
- AR[x] is always decremented (even when condition is false)
- **NOT 0xEA!** 0xEA is something else (needs identification)

## BC — SPRU172C p.4-18
- Opcode: `1111 10Z0 CCCCCCCC` + 16-bit pmad (2 words)  
- BC = 0xF8xx, BCD = 0xFAxx

## CC — SPRU172C p.4-29
- Opcode: `1111 10Z1 CCCCCCCC` + 16-bit pmad (2 words)
- CC = 0xF9xx, CCD = 0xFBxx

===== ./hw/arm/calypso/doc/CALYPSO_HW.md =====
# Calypso Hardware Reference

## Chip Overview
TI Calypso (TWL3014/DBB) — GSM baseband processor
- ARM7TDMI CPU (ARM926EJ-S in QEMU, close enough)
- TMS320C54x DSP core
- Shared API RAM between ARM and DSP
- TPU (Time Processing Unit) for radio timing
- ABB (Analog Baseband) for RF
- BSP (Baseband Serial Port) for DSP↔ABB data
- DMA controller

## ARM Memory Map
| Address | Size | Peripheral |
|---------|------|-----------|
| 0x00000000 | 2MB | Internal ROM |
| 0x00800000 | 256KB | Internal SRAM |
| 0xFFFE0000 | | TPU registers |
| 0xFFFF0000 | | INTH (interrupt controller) |
| 0xFFFF1000 | | TPU control |
| 0xFFFC0000 | | UART modem |
| 0xFFFC8000 | | UART IrDA |
| 0xFFD00000 | 64KB | DSP API RAM (shared with C54x) |

## DSP API RAM (0xFFD00000)
- Total: 64KB (32K words of 16-bit)
- ARM accesses as bytes, DSP accesses as 16-bit words
- ARM byte offset X = DSP word address 0x0800 + X/2
- Double-buffered pages: Write page 0/1, Read page 0/1

## TPU
- Controls radio timing sequences
- TPU_CTRL register:
  - Bit 0: TPU_CTRL_EN (enable scenario execution)
  - Bit 2: TPU_CTRL_IDLE (idle status)
  - Bit 4: TPU_CTRL_DSP_EN (enable DSP frame interrupt)
- Firmware writes TPU scenarios then sets EN
- When scenario completes: clears EN, fires FRAME interrupt
- DSP_EN causes FRAME interrupt to also wake DSP

## INTH (Interrupt Controller)
- Level-sensitive
- IRQ 4: TPU_FRAME
- IRQ 0: Watchdog
- IRQ 7: UART

## TDMA Timing
- GSM frame: 4.615ms (216.7 frames/sec)
- 8 timeslots per frame
- Hyperframe: 2715648 frames
- QEMU uses 10x slowed timing for emulation stability

## DSP C54x Integration
- DSP boots from internal ROM at IPTR*128 (0xFF80 for IPTR=0x1FF)
- ARM communicates via API RAM (shared memory)
- ARM signals DSP via:
  - Writing d_dsp_page in NDB
  - Setting TPU_CTRL_DSP_EN
  - TPU generates FRAME interrupt to DSP (SINT17, vec 2)
- DSP signals ARM by writing results in Read page

## OsmocomBB Layer1 Flow
1. `l1_sync()` called every TDMA frame (TPU FRAME IRQ)
2. Updates page pointers (db_w, db_r)
3. Runs TDMA scheduler (`tdma_sched_execute`)
4. If tasks scheduled: writes d_task_d/d_task_md, calls `dsp_end_scenario()`
5. `dsp_end_scenario()`: writes d_dsp_page = B_GSM_TASK | w_page, toggles w_page
6. Enables TPU DSP frame interrupt
7. DSP wakes, reads d_dsp_page, dispatches task, writes results
8. Next frame: ARM reads results from db_r

## TRXD Protocol (BTS ↔ Bridge)
### DL (BTS → MS) — TRXD v0
| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | TN (timeslot number, 3 bits) |
| 1-4 | 4 | FN (frame number, big-endian) |
| 5 | 1 | RSSI |
| 6-7 | 2 | TOA (big-endian) |
| 8+ | 148 | Soft bits (0=strong 1, 127=uncertain, 255=strong 0) |

### UL (MS → BTS) — TRXD v0
| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | TN |
| 1-4 | 4 | FN (big-endian) |
| 5 | 1 | PWR |
| 6+ | 148 | Hard bits (0/1) |

## Sercomm Protocol
- Flag: 0x7E
- Escape: 0x7D (next byte XOR 0x20)
- Frame: FLAG + DLCI + CTRL(0x03) + payload + FLAG
- DLCI 4: burst data
- DLCI 5: L1CTL messages

## L1CTL Protocol (mobile ↔ firmware)
- Length-prefixed: 2-byte big-endian length + message
- Message: type(1) + flags(1) + padding(2) + payload
- Key types:
  - 0x01: FBSB_REQ
  - 0x02: FBSB_CONF (result byte: 0=success, 255=fail)
  - 0x03: DATA_IND
  - 0x04: RACH_REQ
  - 0x05: DM_EST_REQ
  - 0x06: DATA_REQ
  - 0x07: RESET_IND (sent on boot)
  - 0x08: PM_REQ
  - 0x09: PM_CONF
  - 0x0D: RESET_REQ (payload: reset_type, 1=full)
  - 0x0E: RESET_CONF
  - 0x10: CCCH_MODE_REQ
  - 0x11: CCCH_MODE_CONF

===== ./hw/arm/calypso/doc/SESSION_20260405_NIGHT4.md =====
# Session 2026-04-05 Night 4 — C54x Opcode Audit

## Summary

Massive opcode audit of calypso_c54x.c against the authoritative `tic54x-opc.c`
from binutils-2.21.1 (found at `/var/lib/docker/overlay2/.../gnuarm/src/binutils-2.21.1/opcodes/tic54x-opc.c`).

**17 bugs fixed.** DSP now boots correctly: IDLE reached, IMR configured, dispatch loop active.

## Fixes

### ALU Instructions (Fix 1)
- **F0xx was READA** → now ADD/SUB/LD/AND/OR/XOR #lk,shift,src,dst
- Also added F06x (ADD/SUB/LD #lk,16 + MPY/MAC #lk)
- Also added F08x-F0Fx (accumulator AND/OR/XOR/SFTL with shift)
- Encoding: bits 7:4=op, bits 9:8=src/dst, bits 3:0=shift, 2nd word=lk
- Source: tic54x-opc.c line 254 `{ "add", 0xF000, 0xFCF0 }`

### RSBX/SSBX (Fixes 2-5)
Per tic54x-opc.c: RSBX=0xF4B0 mask 0xFDF0, SSBX=0xF5B0 mask 0xFDF0.
This covers 4 opcode ranges:

| Fix | Opcode | Was | Now | Encoding |
|-----|--------|-----|-----|----------|
| 2 | F4Bx | NOP catch-all | RSBX ST0 | bit9=0, bit8=0 |
| 3 | F5Bx | RPT #0xBx (176+!) | SSBX ST0 | bit9=0, bit8=1 |
| 4 | F6Bx | MVDD Xmem,Ymem | RSBX ST1 | bit9=1, bit8=0 |
| 5 | F7Bx | LD #k8 → AR7 | SSBX ST1 | bit9=1, bit8=1 |

### Return/Branch Instructions (Fixes 6-12)

| Fix | Opcode | Was | Now | tic54x-opc.c |
|-----|--------|-----|-----|---------------|
| 6 | FC00 | LD #k<<16, B | RET (return) | line 391 |
| 7 | FE00 | LD #k, B | RETD (return delayed) | line 392 |
| 8 | F073 | RET (pop stack) | B pmad (branch, 2-word) | line 264 |
| 9 | F074 | RETE (pop+clr INTM) | CALL pmad (call, 2-word) | line 279 |
| 10 | F072 | FRET (far return) | RPTB pmad (block repeat) | line 410 |
| 11 | F070 | RET (catch-all) | RPT #lku (repeat, 2-word) | line 409 |
| 12 | F071 | RET (catch-all) | RPTZ dst,#lku (repeat-zero) | line 412 |

### CALLD Return Address (Fix 13)
- F274 CALLD pushed PC+2, should be **PC+4** (skip 2-word instruction + 2 delay slots)
- Both copies fixed (line 960 and 1210)

### IDLE/FRET (Fixes 14-16)

| Fix | Opcode | Was | Now | tic54x-opc.c |
|-----|--------|-----|-----|---------------|
| 14 | F4E4 | IDLE | FRET (far return) | line 306 |
| 15 | F4E1 | NOP (catch-all) | IDLE (the real one) | line 310 |
| 16 | F4E5 | NOP (catch-all) | FRETE (far return from IRQ) | line 308 |

### TINT0 Interrupt Mapping (Fix 17)
- `calypso_tint0.h`: IFR bit 4 → **bit 3**, vector 20 → **vector 19**
- Per TMS320C5410A datasheet: TINT0 = IFR/IMR bit 3, interrupt vector 19
- Previous mapping (bit 4) was BRINT0 (BSP receive), not TINT0

### Condition Evaluator (part of Fix 7)
Replaced incomplete if-else chain with proper tic54x-opc.c encoding:
- CC1=0x40 (accumulator test), CCB=0x08 (B vs A)
- EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
- OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08

## Results

### Before (all 17 bugs present)
- DSP stuck in dispatch loop at 0x81AF forever
- IMR=0x0000 (never configured)
- idle=0 (IDLE never reached)
- TINT0 on wrong interrupt bit

### After
- DSP boots in 173 instructions
- IMR=0x002D configured (bits 0,2,3,5 = INT0,INT2,TINT0,BXINT0)
- Dispatch loop active at 0x81AF
- TINT0 on correct bit 3 (enabled in IMR)

### Remaining Issue
- **SP drift**: 0x5AC8 → 0x8FFE after boot. FRET at 0x770C pops XPC+PC from
  empty stack (init code reached via branch, not FCALL). The popped values are
  garbage → DSP executes DARAM data as code → SP corrupts gradually.
- **No return to IDLE**: dispatch loop never exits because the handler RET/RETD
  pops garbage return addresses. The real IDLE (F4E1) at 0xA6A0 is never reached.
- **INTM=1 stuck**: handler sets INTM=1 (SSBX 1,8 at 0x7710) and never clears it,
  preventing interrupt servicing.

## Key Reference
tic54x-opc.c location in container:
```
/var/lib/docker/overlay2/a242f59.../diff/root/gnuarm/src/binutils-2.21.1/opcodes/tic54x-opc.c
```

## Files Modified
- `hw/arm/calypso/calypso_c54x.c` — md5: 5a474083
- `hw/arm/calypso/calypso_tint0.h` — md5: 65a588f7

===== ./hw/arm/calypso/doc/PROJECT_STATUS.md =====
# Calypso GSM Baseband Emulator — Project Status

## Goal
Run real OsmocomBB layer1.highram.elf firmware on emulated TI Calypso (ARM7 + TMS320C54x DSP) in QEMU. Connect to a real BTS via TRX protocol through a bridge. Mobile/ccch_scan sees the BTS and decodes BCCH/CCCH.

## Architecture
```
BTS (osmo-bts-trx)
  ↕ UDP (TRXC 5701, TRXD 5702, CLK 5700)
bridge.py
  ↕ PTY/UART (sercomm DLCI 4 = bursts, DLCI 5 = L1CTL)
QEMU (ARM7 calypso_trx.c + C54x calypso_c54x.c)
  ↕ Unix socket /tmp/osmocom_l2_1 (L1CTL length-prefixed)
mobile / ccch_scan (OsmocomBB host tools)
```

## Key Files
| File | Role |
|------|------|
| `calypso_c54x.c` | TMS320C54x DSP emulator (~1800 lines) |
| `calypso_c54x.h` | DSP state struct, constants, API |
| `calypso_trx.c` | Calypso SoC: DSP API RAM, TPU, TDMA, burst RX |
| `l1ctl_sock.c` | Unix socket L1CTL ↔ sercomm bridge |
| `calypso_uart.c` | UART with sercomm DLCI routing |
| `calypso_inth.c` | ARM interrupt controller |
| `calypso_soc.c` | SoC glue, memory map |
| `calypso_mb.c` | Machine/board definition |
| `bridge.py` | BTS TRX UDP ↔ UART sercomm bridge |
| `run.sh` | Launches QEMU + bridge + BTS + ccch_scan in tmux |
| `sync.sh` | Syncs files between host and docker container |
| `l1ctl_test.py` | Direct L1CTL test script |

## DSP ROM
- Dumped from real Motorola C1xx phone via osmocon + ESP32
- File: `/opt/GSM/calypso_dsp.txt` (131168 words)
- Sections: Registers [00000-0005f], DROM [09000-0dfff], PDROM [0e000-0ffff], PROM0 [07000-0dfff], PROM1 [18000-1ffff], PROM2 [28000-2ffff], PROM3 [38000-39fff]
- DSP version: 0x3606

## Memory Map (DSP side)
| DSP Address | ARM Address | Content |
|-------------|-------------|---------|
| 0x0000-0x001F | — | MMR (registers) |
| 0x0020-0x007F | — | DARAM (low, scratch) |
| 0x0080-0x07FF | — | DARAM (overlay with prog when OVLY=1) |
| 0x0800-0x27FF | 0xFFD00000-0xFFD03FFF | API RAM (shared ARM↔DSP) |
| 0x7000-0xDFFF | — | PROM0 (program ROM) |
| 0x8000-0xFFFF | — | PROM1 mirror (program ROM page 1) |
| 0x9000-0xDFFF | — | DROM (data ROM) |
| 0xE000-0xFFFF | — | PDROM (patch data ROM) |

## API RAM Layout
| ARM Offset | DSP Address | Name |
|------------|-------------|------|
| 0x0000 (W_PAGE_0) | 0x0800 | Write page 0 (MCU→DSP, 20 words) |
| 0x0028 (W_PAGE_1) | 0x0814 | Write page 1 |
| 0x0050 (R_PAGE_0) | 0x0828 | Read page 0 (DSP→MCU, 20 words) |
| 0x0078 (R_PAGE_1) | 0x083C | Read page 1 |
| 0x01A8 (NDB) | 0x08D4 | d_dsp_page (first word of NDB) |
| 0x0862 (PARAM) | 0x0C31 | DSP parameters |

## d_dsp_page values
- `0x0000` = no task
- `0x0002` = B_GSM_TASK | page 0
- `0x0003` = B_GSM_TASK | page 1
- B_GSM_TASK = (1 << 1) = 0x0002

## Write page structure (T_DB_MCU_TO_DSP, 20 words)
| Offset | Field |
|--------|-------|
| 0 | d_task_d (downlink task) |
| 1 | d_burst_d |
| 2 | d_task_u (uplink task) |
| 3 | d_burst_u |
| 4 | d_task_md (monitoring/FB/SB task: 5=FB, 6=SB) |
| 5 | d_background |
| 6 | d_debug |
| 7 | d_task_ra |
| 8-19 | results area |

## DSP Boot Sequence
1. ARM writes DSP_DL_STATUS_READY
2. `c54x_reset()` → PMST=0xFFE0, IPTR=0x1FF, OVLY=0, PC=0xFF80
3. `c54x_run(10M)` → DSP executes PROM1 reset code → calls PROM0 init → 86K insns → IDLE@0xFFFE
4. ARM continues, firmware initializes
5. Each TDMA frame: ARM writes d_dsp_page + tasks, fires TPU_CTRL_EN
6. `calypso_dsp_done()` → copies write page to DARAM 0x0586, wakes DSP
7. DSP jumps to 0x8000 (TDMA slot table), processes, returns to IDLE

## TDMA Slot Table (PROM1 0x8000-0x801F)
```
0x8000: 12f8 3fc5 f4e3 f4e4  → slot 0: SUB data[0x3FC5], A; SSBX INTM; IDLE
0x8004: 12f8 322a f4e3 f4e4  → slot 1
...8 slots of 4 words each...
0x8020: processing code starts (reads d_dsp_page, dispatches tasks)
```

## Frame Dispatch Routine (PROM0)
- Entry: 0xC8E7 → reads d_dsp_page at DSP 0x08D4
- Configures page pointers (0x0800/0x0814/0x0828/0x083C)
- Called via BANZ at 0xC8CD
- d_dsp_page also read at 0xA51C during init

## Bugs Fixed This Session
1. **Timer HW (TCR/PSC/TDDR)** — Real C54x timer behavior, TSS=1 at reset
2. **IDLE PC** — return 0 (stay at IDLE addr for interrupt handler)
3. **PC wrap 16-bit** — `s->pc &= 0xFFFF`
4. **F4E2/F4E3 (RSBX/SSBX INTM)** — 98 occurrences decoded as BD instead of interrupt enable/disable. CRITICAL BUG.
5. **Interrupt vec = bit + 16** then reverted to **SINT17 vec 2 bit 1** — correct vector for TPU frame
6. **FRET** — Must pop 2 words (XPC then PC) per SPRU172C p.4-61
7. **FCALL** — Must push 2 words (PC+2 then XPC) per SPRU172C p.4-57
8. **XC (Execute Conditionally)** — Opcode 0xFDxx (n=1) / 0xFFxx (n=2), NOT 0xEAxx. 0xEA = BANZ.
9. **TRXD header** — 8 bytes (TN+FN+RSSI+TOA), not 6. Soft bit conversion.
10. **L1CTL sequencing** — 1 message per callback to let ARM process between messages
11. **OVLY activation** — Enabled after boot for DARAM code execution
12. **IDLE wake** — Jump to 0x8000 (TDMA loop) on wake from boot IDLE
13. **Interrupt PC+1** — Push PC+1 when waking from IDLE (resume after IDLE)
14. **Timer FN increment** — TINT0 increments frame number in DARAM

## Current State (2026-04-03)
- Base: no_cell_found (API RAM intercepts for FB/SB/PM)
- No_cell_found loop works: RESET→PM→FBSB cycle running
- C54x DSP boots in parallel via TDMA ticks (2000 insns/frame)
- SINT17 controlled by TPU_CTRL_DSP_EN (per hardware spec)
- 0 UNIMPL instructions — all opcodes emulated
- SP=0x06AA-0x0C2E after boot (correct, in DARAM)
- DSP reaches PROM0 0x7000 init + PROM1 0x8159 processing

## Bugs Fixed (session 2026-04-03)
1. **RC conditionnel (F2xx)** — was unconditional RET, now eval_condition
2. **PROM0 read protection** — prog_read returns prog[] for 0x7000-0xDFFF
3. **PROM0 write protection** — prog_write ignores writes to ROM area
4. **prog_write double-write** — return after prog[ext] for addr>=0x8000
5. **DELAY instruction (D4/D5)** — pipeline delay, was UNIMPL
6. **BCD 0xEF** — added to EE/ED branch conditional handler
7. **XOR/OR #lk16 (B0/B1/B8/B9)** — was UNIMPL
8. **Timer0 hardware** — TIM/PRD/TCR with prescaler and TINT0
9. **IDLE skip TDMA slots** — 0x8000-0x801F IDLE treated as NOP
10. **Parallel DSP boot** — no blocking c54x_run(10M), boot via TDMA ticks
11. **DSP_EN SINT17** — interrupt only when firmware sets TPU_CTRL_DSP_EN
12. **eval_condition** — full XC/RC condition decoder per SPRU172C Table 3-2
13. **Interrupt dispatch** — check IFR&IMR in main loop each cycle
14. **API IRQ** — raise IRQ15 in dsp_done, unmask at boot

## d_fb_det Location
- ARM offset: 0x01F0 (NDB + 0x48)
- DSP address: 0x08F8
- DSP ROM writes it at PROM0 0x7730-0x7990

## Next Steps
1. DSP boot must reach IDLE@0xFFFE — currently runs but doesn't converge
2. Once boot works, remove API RAM intercepts (let DSP produce results)
3. Verify burst samples reach DSP at correct DARAM address
4. Achieve FBSB_CONF(result=0) → SB decode → BCCH → mobile registered

## Docker
- Container: `osmo-operator-1` (always running)
- Images: `calypso-qemu:YYYYMMDD-HHMM` (snapshots)
- Build: `ninja -C /opt/GSM/qemu-src/build`
- Run: `bash /opt/GSM/qemu-src/run.sh`
- DSP ROM: `/opt/GSM/calypso_dsp.txt`
- Firmware: `/opt/GSM/firmware/board/compal_e88/layer1.highram.elf`

## Reference Documents
- `doc/spru172c.pdf` — TMS320C54x DSP Reference Set Volume 2: Mnemonic Instruction Set
- `doc/C54X_INSTRUCTIONS.md` — Key instruction encodings extracted from SPRU172C

===== ./hw/arm/calypso/doc/DSP_ROM_MAP.md =====
# Calypso DSP ROM Map

## ROM Dump Sections
Source: `/opt/GSM/calypso_dsp.txt` — dumped from Motorola C1xx via osmocon + ESP32

| Section | Address Range | Size (words) | Loaded Into |
|---------|--------------|--------------|-------------|
| Registers | 0x00000-0x0005F | 96 | data[0x00-0x5F] |
| DROM | 0x09000-0x0DFFF | 20480 | data[0x9000-0xDFFF] |
| PDROM | 0x0E000-0x0FFFF | 8192 | data[0xE000-0xFFFF] |
| PROM0 | 0x07000-0x0DFFF | 28672 | prog[0x7000-0xDFFF] |
| PROM1 | 0x18000-0x1FFFF | 32768 | prog[0x18000-0x1FFFF] + mirror at prog[0x8000-0xFFFF] |
| PROM2 | 0x28000-0x2FFFF | 32768 | prog[0x28000-0x2FFFF] |
| PROM3 | 0x38000-0x39FFF | 8192 | prog[0x38000-0x39FFF] |

## Key Code Locations

### PROM1 (mirrored to 0x8000-0xFFFF)
| Address | Content |
|---------|---------|
| 0xFF80-0xFFFE | RESET boot code (runs sequentially, NOT separate interrupt vectors) |
| 0xFFFE | IDLE instruction (end of boot) |
| 0x8000-0x801F | TDMA slot table (8 slots × 4 words: SUB + SSBX INTM + IDLE) |
| 0x8020+ | Processing code (after TDMA slots) |

### PROM0 (0x7000-0xDFFF)
| Address | Content |
|---------|---------|
| 0x7000-0x7025 | Boot init routines (called from PROM1 RESET handler) |
| 0x7026-0x71FF | Boot polling loop (writes API RAM tables) |
| 0xA4CA-0xA530 | Frame init / page setup |
| 0xA51C | Reads d_dsp_page (instruction: `10f8 08d4`) |
| 0xC860-0xC8C8 | Frame dispatcher setup |
| 0xC8CD | BANZ to 0xC8E7 (dispatch entry) |
| 0xC8E7-0xC920 | Frame dispatch: reads d_dsp_page, configures pages, branches to task handlers |
| 0xC920+ | Task processing code |

### PDROM (data space 0xE000-0xFFFF, prog space via XPC=0)
| Address | Content |
|---------|---------|
| 0xE000+ | DSP runtime code (accessed as prog[0x8000+] with XPC=0) |

## Interrupt Vector Table (IPTR=0x1FF → base 0xFF80)
The table at 0xFF80-0xFFFF in PROM1 is **boot code**, not separate handlers.
Vectors 0-31 fall into inline boot code. Only useful vectors:
- Vec 0 (0xFF80): RESET entry point
- Most other vectors: inline boot code (context save/restore + RETE)

## MVPD Locations in PROM0
16 MVPD (0x8Cxx) instructions at: 0x75C0, 0x8700, 0x8C80, 0x8CA0
These are NOT reached during the 86K-instruction boot — they're in processing code.

## Key Data Addresses (DSP data space)
| Address | Content |
|---------|---------|
| 0x0007 | Used by TDMA slot table (LD/ST with offsets) |
| 0x08D4 | d_dsp_page (NDB offset 0) |
| 0x08D5 | d_error_status (NDB offset 1) |
| 0x0800-0x0813 | Write page 0 |
| 0x0814-0x0827 | Write page 1 |
| 0x0828-0x083B | Read page 0 |
| 0x083C-0x084F | Read page 1 |
| 0x3FB0 | Internal: page state variable |
| 0x3FC1-0x3FC2 | Internal: current page pointers |
| 0x3FDC-0x3FE0 | Internal: boot state variables |

===== ./hw/arm/calypso/doc/SESSION_20260403.md =====
# Session 2026-04-03 — Fix LD #k9,DP (0xEA) + TDMA boot

## Bug Found: 0xEA decoded as BANZ instead of LD #k9,DP

### Root cause
The C54x emulator treated opcode 0xEA as BANZ (branch on auxiliary register
not zero). In reality:
- **BANZ** = opcode `0110 11Z0 IAAAAAAA` = **0x6C/0x6E** (per SPRU172C p.4-16)
- **0xEA** = `1110 101k kkkk kkkk` = **LD #k9, DP** (load data page pointer)

This caused the DSP to branch to random DARAM addresses (e.g., 0x1231) instead
of continuing sequential execution. The DSP would get lost in empty DARAM.

### Evidence
- 0xEA appears 202 times in the Calypso DSP ROM
- 114 uses have bit7=0 (direct addressing) which is impossible for BANZ (requires indirect)
- BANZ (0x6C/0x6E) appears 304 times separately in the ROM
- Confirmed via GNU binutils tic54x-opc.c: `LD #k9,DP` = opcode 0xEA00, mask 0xFE00

### Fix
Replaced the 0xEA BANZ handler with:
```c
if ((op & 0xFE00) == 0xEA00) {
    uint16_t k9 = op & 0x01FF;
    s->st0 = (s->st0 & ~ST0_DP_MASK) | k9;
    return consumed;  // 1 word
}
```

### Impact
- DSP boot now executes 197M instructions (was 86K) — full ROM init including DARAM population
- `data[0x1231] = 0x115F` (was 0x0000) — DARAM correctly populated with processing code
- `PMST = 0x4D86` (OVLY=1) — DSP correctly enables overlay mode
- `d_dsp_page` alternates 0x0002/0x0003 — firmware/DSP page protocol works
- `task_md=5` (FB search) correctly dispatched to DSP

## Boot approach: TDMA-tick (from GSMTAP_WORKING)

Instead of `c54x_run(10M)` at DSP_DL_STATUS_READY, we now:
1. `c54x_reset()` + `running = true`
2. DSP runs 1M instructions per TDMA frame tick
3. SINT17 interrupt delivered when firmware sets TPU_CTRL_DSP_EN

This lets the timer (TINT0) fire between frames, which the DSP boot code needs.

## Remaining issues

### FB detection (result=255)
DSP runs FB search (task_md=5) but never sets d_fb_det=1 at NDB+0x48 (0xFFD001F0).
Burst samples arrive via calypso_trx_rx_burst() and are written to DARAM at 0x03F0
and 0x04F0, but the DSP FB code may read from different addresses.

### Unimplemented opcodes (non-blocking, 145 hits / 197M instructions)
| Opcode | Mnemonic | Words | Encoding |
|--------|----------|-------|----------|
| 0x9Bxx | RPT #k (11-bit) | 1 | `1001 1kkk kkkk kkkk` |
| 0xC9xx | STL B, Smem | 1 | `1100 1SII IIII IIII` |
| 0xA1xx | LD Smem, B | 1 | `1010 0SII IIII IIII` |
| 0xD6xx | LD Smem, T | 1 | `1101 0III IIII IIII` |
| 0xE0xx | BANZ pmad, *ARn- | 2 | standard BANZ alt? needs investigation |
| 0xF1xx | FIRS Xmem,Ymem,pmad | 2 | Symmetric FIR filter |

Priority: RPT #k (0x98-0x9F) is critical — used for block copies during boot/processing.

## Files modified
- `calypso_c54x.c` line 810: 0xEA BANZ -> LD #k9,DP
- `calypso_trx.c` line 135: boot approach changed to TDMA-tick
- `calypso_trx.c` line 292: added DSP run + SINT17 in TDMA tick

## Key addresses reference
- d_fb_det = NDB+0x48 = ARM 0xFFD001F0 = DSP 0x08F8
- d_dsp_page = NDB+0x00 = ARM 0xFFD001A8 = DSP 0x08D4
- Write page 0 = ARM 0xFFD00000 = DSP 0x0800
- Write page 1 = ARM 0xFFD00028 = DSP 0x0814
- Read page 0 = ARM 0xFFD00050 = DSP 0x0828
- Read page 1 = ARM 0xFFD00078 = DSP 0x083C

===== ./hw/arm/calypso/calypso_trx.c =====
/*
 * calypso_trx.c — Calypso hardware emulation + DSP C54x emulation
 * No sockets. Firmware speaks UART only. DSP results in shared RAM.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "exec/address-spaces.h"
#include "hw/irq.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_c54x.h"
#include "calypso_tint0.h"
void calypso_tint0_do_tick(uint32_t fn);
#include "chardev/char-fe.h"

extern CalypsoUARTState *g_uart_modem;
extern CalypsoUARTState *g_uart_irda;

#define TRX_LOG(fmt, ...) \
    fprintf(stderr, "[calypso-trx] " fmt "\n", ##__VA_ARGS__)

#define DSP_API_W_PAGE0  0x0000
#define DSP_API_W_PAGE1  0x0028
#define DSP_API_NDB      0x01A8
#define DB_W_D_TASK_D    0
#define DB_W_D_TASK_U    2
#define DB_W_D_TASK_MD   4

typedef struct CalypsoTRX {
    qemu_irq *irqs;
    MemoryRegion dsp_iomem;
    uint16_t     dsp_ram[CALYPSO_DSP_SIZE / 2];
    uint8_t      dsp_page;
    bool         dsp_booted;
    uint32_t     boot_frame;
    MemoryRegion tpu_iomem;
    MemoryRegion tpu_ram_iomem;
    uint16_t     tpu_regs[CALYPSO_TPU_SIZE / 2];
    uint16_t     tpu_ram[CALYPSO_TPU_RAM_SIZE / 2];
    MemoryRegion tsp_iomem;
    uint16_t     tsp_regs[CALYPSO_TSP_SIZE / 2];
    MemoryRegion ulpd_iomem;
    uint16_t     ulpd_regs[CALYPSO_ULPD_SIZE / 2];
    uint32_t     ulpd_counter;
    MemoryRegion sim_iomem;
    uint16_t     sim_regs[CALYPSO_SIM_SIZE / 2];
    QEMUTimer   *sim_atr_timer;
    uint32_t     fn;
    bool         tdma_running;
    bool         tpu_en_pending;  /* set by TPU_CTRL_EN, cleared by tint0 tick */
    bool         dsp_init_done;  /* true after DSP reaches first IDLE */

    /* C54x DSP emulator */
    C54xState   *dsp;
} CalypsoTRX;

static CalypsoTRX *g_trx;

/* ---- Console stub ----
 * The firmware has debug logging (cons_puts/puts) that busy-waits on
 * UART TX. No console is connected, so it blocks forever.
 * Replace with BX LR (immediate return) — equivalent to -DNDEBUG. */
void calypso_stub_console(void)
{
    uint32_t val, bx_lr = 0xe12fff1e, nop = 0xe1a00000;

    /* Stub cons_puts and puts — they busy-wait on UART TX with no console */
    cpu_physical_memory_read(0x0082a1b0, &val, 4);
    if (val && val != bx_lr) {
        cpu_physical_memory_write(0x0082a1b0, &bx_lr, 4);
        TRX_LOG("stub: cons_puts → BX LR");
    }
    cpu_physical_memory_read(0x00829ea0, &val, 4);
    if (val && val != bx_lr) {
        cpu_physical_memory_write(0x00829ea0, &bx_lr, 4);
        TRX_LOG("stub: puts → BX LR");
    }

    /* NOP debug BL calls that target console functions.
     * These are BL instructions (0x0Bxxxxxx) in the init path
     * that call logging functions — without console they deadlock. */
    static const hwaddr bl[] = {0x828914, 0x828828, 0x828830, 0x828858, 0x828880};
    for (int i = 0; i < 5; i++) {
        cpu_physical_memory_read(bl[i], &val, 4);
        if ((val & 0x0F000000) == 0x0B000000) {
            cpu_physical_memory_write(bl[i], &nop, 4);
        }
    }
}

/* ---- DSP API RAM ---- */
static uint64_t calypso_dsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return 0;
    uint64_t val = (size == 2) ? s->dsp_ram[offset/2] :
                   (size == 4) ? (s->dsp_ram[offset/2] | ((uint32_t)s->dsp_ram[offset/2+1] << 16)) :
                   ((uint8_t *)s->dsp_ram)[offset];
    /* DSP boot */
    if (offset == DSP_DL_STATUS_ADDR && !s->dsp_booted) {
        if (++s->boot_frame > 3) {
            s->dsp_ram[DSP_DL_STATUS_ADDR/2] = DSP_DL_STATUS_BOOT;
            s->dsp_ram[DSP_API_VER_ADDR/2] = DSP_API_VERSION;
            s->dsp_ram[DSP_API_VER2_ADDR/2] = 0;
            s->dsp_booted = true;
            TRX_LOG("DSP boot ver=0x%04x", DSP_API_VERSION);
            val = DSP_DL_STATUS_BOOT;
        }
    }
    /* Pure read — C54x DSP handles all results via shared API RAM */
    return val;
}

static void calypso_dsp_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return;
    if (size == 2) s->dsp_ram[offset/2] = value;
    else if (size == 4) { s->dsp_ram[offset/2] = value; s->dsp_ram[offset/2+1] = value >> 16; }
    else ((uint8_t *)s->dsp_ram)[offset] = value;

    /* Sync d_dsp_page writes to DSP API RAM immediately */
    if (offset == 0x01A8) {
        static int page_wr_log = 0;
        if (page_wr_log < 20) {
            TRX_LOG("d_dsp_page WR = 0x%04x (sz=%d fn=%u)", (unsigned)value, size, s->fn);
            page_wr_log++;
        }
        if (s->dsp && s->dsp->api_ram)
            s->dsp->api_ram[0x08D4 - C54X_API_BASE] = s->dsp_ram[0x01A8/2];
    }
    /* DSP bootloader protocol (BL_CMD_STATUS at offset 0x0FFE)
     * The real bootloader lives in DSP internal ROM (not in our dump).
     * ARM writes commands, bootloader responds with BL_STATUS_IDLE (1).
     * BL_CMD_COPY_BLOCK(2) with size=0: jump DSP PC to BL_ADDR_LO.
     * BL_CMD_COPY_BLOCK(2) with size>0: copy block (we let ARM write API directly).
     * BL_CMD_COPY_MODE(4): set data write mode (ack only). */
    if (offset == 0x0FFE) {
        uint16_t cmd = (uint16_t)value;
        TRX_LOG("DSP bootloader: ARM cmd=%d", cmd);
        if (cmd == 0 || cmd == 2 || cmd == 4) {
            /* Simulate BL protocol: immediately acknowledge */
            s->dsp_ram[0x0FFE/2] = 1;  /* BL_STATUS_IDLE */
        }
        if (cmd == 2 && s->dsp) {
            uint16_t bl_size = s->dsp_ram[0x0FFA/2];
            uint16_t bl_addr = s->dsp_ram[0x0FFC/2];
            if (bl_size == 0 && bl_addr != 0) {
                /* Jump: reset DSP (does MVPD copy), then set PC to target */
                TRX_LOG("DSP bootloader: JUMP to 0x%04x", bl_addr);
                c54x_reset(s->dsp);
                s->dsp->pc = bl_addr;
                /* Emulate Calypso internal boot ROM (0xFF80 + 0xB400).
                 * The real boot ROM runs before the bootloader JUMP and
                 * initializes registers. From ROM dump:
                 *   0xFF84: STM #0x5140, AR3 → AR3=0x5140
                 *   0xFF89: STM #0x56FD, AR3 → AR3=0x56FD (overwritten)
                 *   0xFF8F: STM #0x015E, BK  → BK=0x015E
                 *   0xB401: STM #0x1800, ST0 → ST0=0x1800
                 *   0xB403: STM #0x5AC8, SP  → SP=0x5AC8
                 *   0xB408: LD  #57, DP      → DP=57
                 *   0xB40F: RSBX INTM        → interrupts enabled */
                s->dsp->st0 = 0x1800;
                s->dsp->sp = 0x5AC8;
                s->dsp->ar[3] = 0x56FD;
                s->dsp->ar[0] = 0x0040;  /* above MMR range */
                s->dsp->ar[1] = 0x727D;  /* from B356 frame init */
                s->dsp->ar[2] = 0x0E4E;  /* from B356 frame init */
                s->dsp->bk = 0x015E;
                s->dsp->st1 &= ~ST1_INTM; /* RSBX INTM */
                s->dsp->running = true;
                s->dsp_booted = true;
                s->dsp_ram[0x01A8/2] = 0;
                TRX_LOG("DSP boot: PC=0x%04x SP=0x%04x AR3=0x%04x BK=0x%04x",
                        bl_addr, s->dsp->sp, s->dsp->ar[3], s->dsp->bk);
            } else if (bl_size > 0) {
                TRX_LOG("DSP bootloader: COPY %d words → DSP 0x%04x", bl_size, bl_addr);
                for (int i = 0; i < bl_size && i < 0x2000; i++)
                    s->dsp->data[bl_addr + i] = s->dsp_ram[i];
            }
        }
    }

    /* DSP page */
    if (offset == DSP_API_NDB) s->dsp_page = value & 1;
}

static const MemoryRegionOps calypso_dsp_ops = {
    .read = calypso_dsp_read, .write = calypso_dsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size=1,.max_access_size=4}, .impl = {.min_access_size=1,.max_access_size=4},
};

/* ---- TPU ---- */
/* calypso_tint0_start() is in calypso_tint0.c */

static uint64_t calypso_tpu_read(void *o, hwaddr off, unsigned sz) {
    CalypsoTRX *s=o; if (off==TPU_IT_DSP_PG) return s->dsp_page;
    return (off/2<CALYPSO_TPU_SIZE/2)?s->tpu_regs[off/2]:0;
}
static void calypso_tpu_write(void *o, hwaddr off, uint64_t val, unsigned sz) {
    CalypsoTRX *s=o; if (off/2<CALYPSO_TPU_SIZE/2) s->tpu_regs[off/2]=val;
    if (off==TPU_CTRL) {
        uint16_t reg = (uint16_t)val;
        { static int ctrl_log = 0; if (ctrl_log < 20) { TRX_LOG("TPU_CTRL write 0x%04x", reg); ctrl_log++; } }
        if (reg & TPU_CTRL_EN) {
            /* Flag for next TINT0 tick — no separate timer */
            s->tpu_regs[TPU_CTRL/2] &= ~TPU_CTRL_IDLE;
            calypso_tint0_tpu_en();
            l1ctl_set_burst_mode(true);
            static int tpu_en_log = 0;
            if (tpu_en_log < 10) { TRX_LOG("TPU_CTRL_EN fn=%u", s->fn); tpu_en_log++; }
        }
        /* DSP_EN handled by dsp_done via SINT17 */
    }
    if (off==TPU_INT_CTRL && !(val&ICTRL_MCU_FRAME) && !calypso_tint0_running()) calypso_tint0_start();
    if (off==TPU_IT_DSP_PG) s->dsp_page=val&1;
}
static const MemoryRegionOps calypso_tpu_ops = {
    .read=calypso_tpu_read,.write=calypso_tpu_write,.endianness=DEVICE_LITTLE_ENDIAN,
    .valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},
};
static uint64_t calypso_tpu_ram_read(void *o,hwaddr off,unsigned sz){CalypsoTRX*s=o;return(off/2<CALYPSO_TPU_RAM_SIZE/2)?s->tpu_ram[off/2]:0;}
static void calypso_tpu_ram_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off/2<CALYPSO_TPU_RAM_SIZE/2)s->tpu_ram[off/2]=v;}
static const MemoryRegionOps calypso_tpu_ram_ops={.read=calypso_tpu_ram_read,.write=calypso_tpu_ram_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},};

/* ---- TSP ---- */
static uint64_t calypso_tsp_read(void *o,hwaddr off,unsigned sz){CalypsoTRX*s=o;return(off==TSP_RX_REG)?0xFFFF:(off/2<CALYPSO_TSP_SIZE/2)?s->tsp_regs[off/2]:0;}
static void calypso_tsp_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off/2<CALYPSO_TSP_SIZE/2)s->tsp_regs[off/2]=v;}
static const MemoryRegionOps calypso_tsp_ops={.read=calypso_tsp_read,.write=calypso_tsp_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},};

/* ---- ULPD ---- */
static uint64_t calypso_ulpd_read(void *o,hwaddr off,unsigned sz){
    CalypsoTRX*s=o;if(off>=0x20&&off<=0x40)return 0;
    switch(off){case ULPD_SETUP_CLK13:return 0x2003;case ULPD_COUNTER_HI:s->ulpd_counter+=100;return(s->ulpd_counter>>16)&0xFFFF;
    case ULPD_COUNTER_LO:return s->ulpd_counter&0xFFFF;case ULPD_GAUGING_CTRL:return 1;case ULPD_GSM_TIMER:return s->fn&0xFFFF;
    default:return(off/2<CALYPSO_ULPD_SIZE/2)?s->ulpd_regs[off/2]:0;}
}
static void calypso_ulpd_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off>=0x20&&off<=0x40)return;if(off/2<CALYPSO_ULPD_SIZE/2)s->ulpd_regs[off/2]=v;}
static const MemoryRegionOps calypso_ulpd_ops={.read=calypso_ulpd_read,.write=calypso_ulpd_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=2},.impl={.min_access_size=1,.max_access_size=2},};

/* ---- SIM ---- */
#define SIM_IT 0x08
#define SIM_STAT 0x02
#define SIM_MASKIT 0x0E
static void calypso_sim_atr_cb(void *opaque){CalypsoTRX*s=opaque;uint32_t v=1;cpu_physical_memory_write(0x00830510,&v,4);s->sim_regs[SIM_IT/2]|=1;s->sim_regs[SIM_STAT/2]=1;TRX_LOG("SIM ATR");qemu_irq_pulse(s->irqs[CALYPSO_IRQ_SIM]);}
static uint64_t calypso_sim_read(void *o,hwaddr off,unsigned sz){CalypsoTRX*s=o;uint16_t v=(off/2<CALYPSO_SIM_SIZE/2)?s->sim_regs[off/2]:0;if(off==SIM_IT)s->sim_regs[SIM_IT/2]=0;return v;}
static void calypso_sim_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off/2<CALYPSO_SIM_SIZE/2)s->sim_regs[off/2]=v;if(off==SIM_MASKIT)timer_mod_ns(s->sim_atr_timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+50000);}
static const MemoryRegionOps calypso_sim_ops={.read=calypso_sim_read,.write=calypso_sim_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},};

/* ---- TINT0: single master clock for the entire Calypso ----
 * Ticks at GSM TDMA frame rate (4.615ms = 13MHz / 60000).
 * Drives: UART, DSP (SINT17 + run-until-idle), ARM (frame IRQ + API IRQ).
 * On real Calypso, ARM and DSP share the 13MHz oscillator.
 * TPU generates frame sync; DSP TINT0 (IMR bit 4, vec 20) is internal.
 * We unify both into one QEMU timer tick. */

/* Called by calypso_tint0.c on each TDMA frame tick */
void calypso_tint0_do_tick(uint32_t fn) {
    CalypsoTRX *s = g_trx;
    if (!s) return;
    s->fn = fn;

    /* TINT0: set IFR bit 4 on DSP (Timer 0 interrupt, vec 20).
     * On real hardware this fires when TIM reaches 0.
     * Here, QEMU timer replaces the per-instruction timer tick. */
    if (s->dsp && s->dsp->running) {
        c54x_interrupt_ex(s->dsp, TINT0_VEC, TINT0_IFR_BIT);
    }

    /* Lower previous frame IRQ — allows re-raise at end of this tick */
    qemu_irq_lower(s->irqs[CALYPSO_IRQ_TPU_FRAME]);

    /* 1. UART — poll backend + flush TX/RX (both UARTs) */
    if (g_uart_modem) {
        calypso_uart_poll_backend(g_uart_modem);
        calypso_uart_kick_rx(g_uart_modem);
        calypso_uart_kick_tx(g_uart_modem);
    }
    if (g_uart_irda) {
        calypso_uart_poll_backend(g_uart_irda);
        calypso_uart_kick_rx(g_uart_irda);
        calypso_uart_kick_tx(g_uart_irda);
    }

    /* 2. UL burst poll */
    calypso_trx_tx_burst_poll();

    /* 3. DSP frame processing */
    bool dsp_ran = false;
    bool dsp_should_run = false;

    if (s->dsp && calypso_tint0_tpu_en_pending()) {
        calypso_tint0_tpu_en_clear();
        s->tpu_regs[TPU_CTRL/2] &= ~TPU_CTRL_EN;


        /* DMA: copy API write page to DSP DARAM */
        uint16_t page = s->dsp_ram[0x01A8/2] & 1;
        uint16_t *wp = page ?
            &s->dsp_ram[DSP_API_W_PAGE1/2] : &s->dsp_ram[DSP_API_W_PAGE0/2];
        s->dsp->data[0x0584] = s->dsp_ram[0x01A8/2];
        s->dsp->data[0x0585] = s->fn & 0xFFFF;
        for (int i = 0; i < 20; i++)
            s->dsp->data[0x0586 + i] = wp[i];

        /* Sync d_dsp_page to API RAM */
        if (s->dsp->api_ram)
            s->dsp->api_ram[0x08D4 - C54X_API_BASE] = s->dsp_ram[0x01A8/2];

        /* BSP: rewind position */
        s->dsp->bsp_pos = 0;

        /* SINT17 — frame interrupt to DSP */
        c54x_interrupt_ex(s->dsp, C54X_INT_FRAME_VEC, C54X_INT_FRAME_BIT);
        s->dsp->idle = false;  /* wake from IDLE for new frame */
        dsp_should_run = true;
    }
    /* Also run DSP if still initializing (not yet reached first IDLE) */
    if (s->dsp && s->dsp->running && !s->dsp->idle && !dsp_should_run) {
        dsp_should_run = true;
    }

    if (dsp_should_run && s->dsp) {

        /* Run DSP until IDLE — budget allows init MVPD copies to complete.
         * Real C54x @ 100 MHz ≈ 461K insn per 4.615ms frame.
         * During init (before first IDLE), give unlimited budget.
         * After init, use 500K per frame. */
        int budget = 5000000;  /* 5M for init */
        int ran = 0, chunk;
        while (!s->dsp->idle && ran < budget) {
            chunk = c54x_run(s->dsp, 100000);
            ran += chunk;
            if (chunk == 0) break;
            if (g_uart_modem) {
                calypso_uart_poll_backend(g_uart_modem);
                calypso_uart_kick_rx(g_uart_modem);
                calypso_uart_kick_tx(g_uart_modem);
            }
            if (g_uart_irda) {
                calypso_uart_poll_backend(g_uart_irda);
                calypso_uart_kick_rx(g_uart_irda);
                calypso_uart_kick_tx(g_uart_irda);
            }
        }
        /* If DSP didn't finish, dump last instructions for debugging */
        if (!s->dsp->idle && ran >= 500000000) {
            static int timeout_trace = 0;
            TRX_LOG("DSP TIMEOUT: %d insn PC=0x%04x XPC=%d IMR=0x%04x IFR=0x%04x INTM=%d SP=0x%04x PMST=0x%04x unimpl=%u last_unimpl=0x%04x",
                    ran, s->dsp->pc, s->dsp->xpc, s->dsp->imr, s->dsp->ifr,
                    (s->dsp->st1 >> 11) & 1, s->dsp->sp, s->dsp->pmst,
                    s->dsp->unimpl_count, s->dsp->last_unimpl);
            if (timeout_trace < 5) {
                /* Trace 50 instructions to see the loop */
                for (int t = 0; t < 50; t++) {
                    uint16_t pc0 = s->dsp->pc;
                    uint16_t op = s->dsp->prog[pc0];
                    (void)c54x_run(s->dsp, 1);
                    TRX_LOG("  T%02d PC=%04x op=%04x → PC=%04x SP=%04x %s",
                            t, pc0, op, s->dsp->pc, s->dsp->sp,
                            s->dsp->idle ? "IDLE!" : "");
                    if (s->dsp->idle) break;
                }
                timeout_trace++;
            }
        }
        dsp_ran = true;

        /* Write PM/SNR results to API read page after DSP completes.
         * The DSP should compute these from radio samples, but without
         * real BSP data it writes 0. Override with realistic values
         * so the firmware's PM measurement works. */
        if (s->dsp->idle) {
            uint16_t rpage = s->dsp_ram[0x01A8/2] & 1;
            uint16_t *rp = rpage ?
                &s->dsp_ram[0x0078/2] : &s->dsp_ram[0x0050/2];
            /* Non-DSP33 layout: rp[8..10]=a_pm[3], rp[11..14]=a_serv_demod[4] */
            rp[8]  = 4864;   /* a_pm[0] — PM strong signal */
            rp[9]  = 4864;   /* a_pm[1] */
            rp[10] = 4864;   /* a_pm[2] */
            rp[11] = 0;      /* D_TOA */
            rp[12] = 4864;   /* D_PM */
            rp[13] = 0;      /* D_ANGLE */
            rp[14] = 100;    /* D_SNR */
            /* d_fb_det in NDB at ARM offset 0x01F0 = dsp_ram[0xF8]
             * Set to 1 when burst mode active (firmware is searching for FB) */
            if (l1ctl_burst_mode()) {
                s->dsp_ram[0xF8] = 1;  /* d_fb_det = FOUND */
                /* sync_demod: TOA, PM, ANGLE, SNR at 0x01F4-0x01FA */
                s->dsp_ram[0xFA] = 0;     /* TOA */
                s->dsp_ram[0xFB] = 4864;  /* PM */
                s->dsp_ram[0xFC] = 0;     /* ANGLE */
                s->dsp_ram[0xFD] = 100;   /* SNR */
            }
        }

        static int done_log = 0;
        if (done_log < 30) {
            TRX_LOG("TINT0: fn=%u page=0x%04x ran=%d PC=0x%04x idle=%d IMR=0x%04x",
                    s->fn, s->dsp_ram[0x01A8/2], ran, s->dsp->pc,
                    s->dsp->idle, s->dsp->imr);
            done_log++;
        }
    }

    /* Detect first IDLE → DSP init complete. Clean up registers
     * corrupted by init RPTB sweep so SINT17 handler works. */
    if (s->dsp && s->dsp->idle && !s->dsp_init_done) {
        s->dsp_init_done = true;
        /* Reset registers corrupted by init RPTB at 0x76FE */
        s->dsp->sp = 0x5AC8;
        s->dsp->pmst = 0xFFA8;
        /* Leave IMR/IFR as the DSP configured them */
        /* Restore ARs to frame processing values (subroutine 0xB356) */
        s->dsp->ar[0] = 0x0040;
        s->dsp->ar[1] = 0x727D;
        s->dsp->ar[2] = 0x0E4E;
        s->dsp->ar[3] = 0x56FD;
        s->dsp->bk = 0x015E;
        /* Stop any active RPTB from init */
        s->dsp->rptb_active = false;
        s->dsp->st1 &= ~ST1_BRAF;
        s->dsp->rpt_active = false;
        TRX_LOG("DSP init complete (first IDLE) fn=%u PC=0x%04x SP=0x%04x IMR=0x%04x",
                s->fn, s->dsp->pc, s->dsp->sp, s->dsp->imr);
    }
    /* 4. API IRQ (IRQ15) — only when DSP actually processed a frame.
     * Lower first (edge-like), raise only if DSP ran this tick. */
    qemu_irq_lower(s->irqs[CALYPSO_IRQ_API]);
    if (dsp_ran && s->dsp && s->dsp->idle) {
        qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]);
    }

    /* 5. TPU frame IRQ (IRQ4) — ARM TDMA scheduler.
     * Raise and hold — firmware ACKs via INTH register write. */
    qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME]);

}  /* end calypso_tint0_do_tick */

/* TINT0 start is now in calypso_tint0.c */

/* ---- Sercomm burst transport (DLCI 4) ---- */

/* Condition check: is the firmware ready to receive a DL burst?
 * Used by UART filter to gate DLCI 4 frames.
 * Conditions: DSP booted, TDMA running, L1CTL client active.
 * Only 1 burst per TDMA frame — like real TPU/BSP. */
bool calypso_trx_burst_ready(void)
{
    if (!g_trx) return false;
    CalypsoTRX *s = g_trx;

    /* Restaurant closed: no client, no FBSB_REQ, no service */
    if (!s->dsp_booted || !calypso_tint0_running() || !l1ctl_burst_mode())
        return false;

    /* One burst per frame */
    static uint32_t last_fn = 0xFFFFFFFF;
    if (s->fn == last_fn) return false;
    last_fn = s->fn;
    return true;
}

/* RX burst from bridge (DL) — store in DSP RAM for firmware to read. */
void calypso_trx_rx_burst(const uint8_t *data, int len)
{
    if (!g_trx || len < 6) return;
    CalypsoTRX *s = g_trx;

    uint32_t fn = ((uint32_t)data[1]<<24)|((uint32_t)data[2]<<16)|
                  ((uint32_t)data[3]<<8)|(uint32_t)data[4];
    uint8_t tn = data[0] & 0x07; (void)tn;

    /* Sync FN */
    s->fn = fn % GSM_HYPERFRAME;

    /* DL format: header(8) + GMSK I/Q samples (16-bit LE, 2 bytes each)
     * Bridge sends: TN(1) FN(4) RSSI(1) TOA(2) + samples(N x int16_t) */
    int nsamples = (len - 8) / 2;  /* 16-bit samples */
    if (nsamples > 148) nsamples = 148;
    if (nsamples < 0) nsamples = 0;
    int nbits = nsamples;

    /* RX_BURST logging removed */

    /* Load GMSK I/Q samples into C54x DARAM + BSP buffer */
    if (s->dsp) {
        uint16_t samples[160];
        for (int i = 0; i < nbits; i++) {
            /* Read 16-bit LE GMSK I/Q samples from bridge */
            int16_t s16 = (int16_t)(data[8 + i*2] | (data[8 + i*2 + 1] << 8));
            samples[i] = (uint16_t)s16;
        }
        c54x_bsp_load(s->dsp, samples, nbits);

        /* Fire BRINT0 — BSP receive complete (vec 21, IMR bit 5)
         * Only after init — during boot the DSP doesn't expect bursts */
        if (s->dsp_init_done)
            c54x_interrupt_ex(s->dsp, 21, 5);

        /* Write to DARAM at multiple candidate addresses */
        /* Address from pointer at 0x00B9 */
        uint16_t ptr = s->dsp->data[0x00B9];
        if (ptr >= 0x0020 && ptr < 0x0800) {
            for (int i = 0; i < nbits; i++)
                s->dsp->data[ptr + i] = samples[i];
        }
        /* Also write at fixed addresses used by BSP DMA on Calypso:
         * 0x03F0-0x04FF is a common DMA target area */
        for (int i = 0; i < nbits; i++) {
            s->dsp->data[0x03F0 + i] = samples[i];
            s->dsp->data[0x04F0 + i] = samples[i];
        }
    }

    /* Update read page metadata */
    uint16_t *rp = s->dsp_page ?
        &s->dsp_ram[0x0078/2] : &s->dsp_ram[0x0050/2];
    rp[1] = (uint16_t)(fn % 4);  /* d_burst_d */
    rp[8]  = 0;                   /* TOA */
    rp[9]  = 4864;                /* PM */
    rp[10] = 0;                   /* ANGLE */
    rp[11] = 100;                 /* SNR */
}

/* TX burst: send UL burst from DSP write page via UART TX as sercomm DLCI 4 */
static void calypso_trx_send_ul_burst(CalypsoTRX *s, uint16_t task_u)
{
    if (!g_uart_modem || task_u == 0) return;

    /* Read UL burst from write page.
     * d_burst_u at word 3, burst data follows in NDB a_cu area. */
    uint16_t *wp = s->dsp_page ?
        &s->dsp_ram[DSP_API_W_PAGE1 / 2] : &s->dsp_ram[DSP_API_W_PAGE0 / 2];

    /* Build TRXD v0 UL packet: TN(1) FN(4) RSSI(1) TOA256(2) bits(148) = 156 */
    uint8_t pkt[8 + 148];
    uint8_t tn = wp[3] & 0x07;  /* d_burst_u has TN info */
    uint32_t fn = s->fn;

    pkt[0] = tn;
    pkt[1] = (fn >> 24) & 0xFF;
    pkt[2] = (fn >> 16) & 0xFF;
    pkt[3] = (fn >> 8) & 0xFF;
    pkt[4] = fn & 0xFF;
    pkt[5] = (uint8_t)(-60);  /* RSSI */
    pkt[6] = 0;  /* TOA256 high */
    pkt[7] = 0;  /* TOA256 low */

    /* Read burst bits from NDB UL area — for now send dummy burst */
    memset(&pkt[8], 0, 148);

    /* Wrap in sercomm DLCI 4 and send via UART TX */
    uint8_t frame[512];
    int pos = 0;
    frame[pos++] = 0x7E;  /* FLAG */
    /* Header: DLCI + CTRL, with escaping */
    uint8_t hdr[2] = { 0x04, 0x03 };
    for (int i = 0; i < 2; i++) {
        if (hdr[i] == 0x7E || hdr[i] == 0x7D) {
            frame[pos++] = 0x7D;
            frame[pos++] = hdr[i] ^ 0x20;
        } else {
            frame[pos++] = hdr[i];
        }
    }
    /* Payload with escaping */
    int pkt_len = 8 + 148;
    for (int i = 0; i < pkt_len && pos < 500; i++) {
        if (pkt[i] == 0x7E || pkt[i] == 0x7D) {
            frame[pos++] = 0x7D;
            frame[pos++] = pkt[i] ^ 0x20;
        } else {
            frame[pos++] = pkt[i];
        }
    }
    frame[pos++] = 0x7E;  /* FLAG */

    /* Write to UART chardev (goes to PTY → bridge reads it) */
    qemu_chr_fe_write_all(&g_uart_modem->chr, frame, pos);
}

void calypso_trx_tx_burst_poll(void)
{
    if (!g_trx) return;
    /* Check if firmware wrote a UL task */
    CalypsoTRX *s = g_trx;
    uint16_t *wp = s->dsp_page ?
        &s->dsp_ram[DSP_API_W_PAGE1 / 2] : &s->dsp_ram[DSP_API_W_PAGE0 / 2];
    uint16_t task_u = wp[DB_W_D_TASK_U];
    if (task_u != 0) {
        calypso_trx_send_ul_burst(s, task_u);
        wp[DB_W_D_TASK_U] = 0;  /* clear after sending */
    }
}

/* ---- Init ---- */
void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs)
{
    CalypsoTRX *s = g_new0(CalypsoTRX, 1);
    g_trx = s; s->irqs = irqs;
    TRX_LOG("=== Calypso hardware init ===");

    memory_region_init_io(&s->dsp_iomem,NULL,&calypso_dsp_ops,s,"calypso.dsp_api",CALYPSO_DSP_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_DSP_BASE,&s->dsp_iomem);
    /* DSP starts unbooted — ARM firmware drives the boot sequence */
    s->dsp_ram[DSP_DL_STATUS_ADDR/2] = 0; s->dsp_booted = false; s->dsp_init_done = false;

    memory_region_init_io(&s->tpu_iomem,NULL,&calypso_tpu_ops,s,"calypso.tpu",CALYPSO_TPU_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_TPU_BASE,&s->tpu_iomem);
    memory_region_init_io(&s->tpu_ram_iomem,NULL,&calypso_tpu_ram_ops,s,"calypso.tpu_ram",CALYPSO_TPU_RAM_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_TPU_RAM_BASE,&s->tpu_ram_iomem);
    memory_region_init_io(&s->tsp_iomem,NULL,&calypso_tsp_ops,s,"calypso.tsp",CALYPSO_TSP_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_TSP_BASE,&s->tsp_iomem);
    memory_region_init_io(&s->ulpd_iomem,NULL,&calypso_ulpd_ops,s,"calypso.ulpd",CALYPSO_ULPD_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_ULPD_BASE,&s->ulpd_iomem);
    memory_region_init_io(&s->sim_iomem,NULL,&calypso_sim_ops,s,"calypso.sim",CALYPSO_SIM_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_SIM_BASE,&s->sim_iomem);
    s->sim_atr_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,calypso_sim_atr_cb,s);

    /* TINT0 master clock is created by calypso_tint0.c */

    /* C54x DSP emulator */
    {
        const char *rom_path = getenv("CALYPSO_DSP_ROM");
        if (!rom_path) rom_path = "/opt/GSM/calypso_dsp.txt";
        s->dsp = c54x_init();
        if (s->dsp) {
            c54x_set_api_ram(s->dsp, s->dsp_ram);
            if (c54x_load_rom(s->dsp, rom_path) == 0) {
                /* Don't reset/boot yet — wait for ARM to write DSP_DL_STATUS_READY */
                s->dsp->running = false;
                TRX_LOG("BUILD 2026-04-05T20:30:16 F4EB=RETE IMR_keep");
                TRX_LOG("C54x DSP loaded from %s (waiting for ARM)", rom_path);
            } else {
                TRX_LOG("C54x DSP ROM not found at %s", rom_path);
                free(s->dsp);
                s->dsp = NULL;
            }
        }
    }

    TRX_LOG("=== Hardware ready ===");
}

===== ./hw/arm/calypso/Kconfig =====
config CALYPSO
    bool
    default y
    select ARM_V4T
    select PFLASH_CFI01

===== ./hw/arm/calypso/calypso_tint0.c =====
/*
 * calypso_tint0.c -- TINT0 master clock for Calypso GSM virtualization
 *
 * Emulates the C54x DSP Timer 0 as a QEMU virtual timer.
 * On real hardware, Timer 0 runs off the 13 MHz DSP clock divided by
 * (PRD+1)*(TDDR+1) to produce a 4.615 ms TDMA frame tick (TINT0).
 * TINT0 drives the entire Calypso timing: DSP frame processing,
 * TPU sync, ARM frame IRQ, and UART polling.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "hw/irq.h"
#include "calypso_tint0.h"
#include "calypso_c54x.h"
#include "hw/core/cpu.h"

#define TINT0_LOG(fmt, ...) \
    fprintf(stderr, "[tint0] " fmt "\n", ##__VA_ARGS__)

/* calypso_trx.c implements the actual frame work (DSP run, IRQs, UART) */
extern void calypso_tint0_do_tick(uint32_t fn);

/* ---- State ---- */
static struct {
    QEMUTimer *timer;
    uint32_t   fn;
    bool       running;
    bool       tpu_en_pending;
} tint0;

/* ---- Timer callback (fires every 4.615ms) ---- */
static void tint0_tick_cb(void *opaque)
{
    tint0.fn = (tint0.fn + 1) % GSM_HYPERFRAME;

    /* Delegate frame work to calypso_trx */
    calypso_tint0_do_tick(tint0.fn);

    /* Re-arm timer */
    if (tint0.running) {
        timer_mod_ns(tint0.timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TINT0_PERIOD_NS);
    }

    /* Kick ARM CPU to process pending IRQs */
qemu_notify_event();
}

/* ---- Public API ---- */

void calypso_tint0_start(void)
{
    if (tint0.running) return;

    if (!tint0.timer) {
        tint0.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tint0_tick_cb, NULL);
    }

    tint0.running = true;
    tint0.fn = 0;
    TINT0_LOG("started (period=%.3f ms, IFR bit %d, vec %d)",
              TINT0_PERIOD_NS / 1e6, TINT0_IFR_BIT, TINT0_VEC);
    timer_mod_ns(tint0.timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TINT0_PERIOD_NS);
}

void calypso_tint0_tpu_en(void)
{
    tint0.tpu_en_pending = true;
}

bool calypso_tint0_tpu_en_pending(void)
{
    return tint0.tpu_en_pending;
}

void calypso_tint0_tpu_en_clear(void)
{
    tint0.tpu_en_pending = false;
}

uint32_t calypso_tint0_fn(void)
{
    return tint0.fn;
}

void calypso_tint0_set_fn(uint32_t fn)
{
    tint0.fn = fn % GSM_HYPERFRAME;
}

bool calypso_tint0_running(void)
{
    return tint0.running;
}

===== ./hw/arm/calypso/calypso_c54x.h =====
/*
 * calypso_c54x.h — TMS320C54x DSP emulator for Calypso
 *
 * Emulates the C54x DSP core found in the TI Calypso baseband chip.
 * Loads ROM dump, executes instructions, shares API RAM with ARM.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_C54X_H
#define CALYPSO_C54X_H

#include <stdint.h>
#include <stdbool.h>

/* Memory sizes (in 16-bit words) */
#define C54X_PROG_SIZE   0x40000  /* 256K words program space */
#define C54X_DATA_SIZE   0x10000  /* 64K words data space */
#define C54X_IO_SIZE     0x10000  /* 64K words I/O space */

/* API RAM: shared between ARM and DSP */
#define C54X_API_BASE    0x0800   /* DSP data address of API RAM */
#define C54X_API_SIZE    0x2000   /* 8K words */

/* DSP start address (after boot) */
#define C54X_DSP_START   0x7000

/* MMR addresses (data memory 0x00-0x1F) */
#define MMR_IMR   0x00
#define MMR_IFR   0x01
#define MMR_ST0   0x06
#define MMR_ST1   0x07
#define MMR_AL    0x08
#define MMR_AH    0x09
#define MMR_AG    0x0A
#define MMR_BL    0x0B
#define MMR_BH    0x0C
#define MMR_BG    0x0D
#define MMR_T     0x0E
#define MMR_TRN   0x0F
#define MMR_AR0   0x10
#define MMR_AR1   0x11
#define MMR_AR2   0x12
#define MMR_AR3   0x13
#define MMR_AR4   0x14
#define MMR_AR5   0x15
#define MMR_AR6   0x16
#define MMR_AR7   0x17
#define MMR_SP    0x18
#define MMR_BK    0x19
#define MMR_BRC   0x1A
#define MMR_RSA   0x1B
#define MMR_REA   0x1C
#define MMR_PMST  0x1D
#define MMR_XPC   0x1E

/* Timer registers (memory-mapped at 0x0024-0x0026) */
#define TIM_ADDR  0x0024   /* Timer counter */
#define PRD_ADDR  0x0025   /* Timer period */
#define TCR_ADDR  0x0026   /* Timer control */

/* TCR bit positions (TMS320C54x hardware spec) */
#define TCR_TDDR_MASK  0x000F   /* bits 3:0 — prescaler reload value */
#define TCR_TSS        (1 << 4) /* bit 4 — Timer Stop Status (1=stopped) */
#define TCR_TRB        (1 << 5) /* bit 5 — Timer Reload (write 1 reloads) */
#define TCR_PSC_SHIFT  6        /* bits 9:6 — prescale counter */
#define TCR_PSC_MASK   (0xF << TCR_PSC_SHIFT)
#define TCR_SOFT       (1 << 10)
#define TCR_FREE       (1 << 11)

/* ST0 bit positions */
#define ST0_DP_MASK  0x01FF  /* bits 8-0: data page pointer */
#define ST0_OVB      (1 << 9)
#define ST0_OVA      (1 << 10)
#define ST0_C        (1 << 11)
#define ST0_TC       (1 << 12)
#define ST0_ARP_SHIFT 13
#define ST0_ARP_MASK (7 << ST0_ARP_SHIFT)

/* ST1 bit positions */
#define ST1_ASM_MASK 0x001F  /* bits 4-0: accumulator shift mode */
#define ST1_CMPT     (1 << 5)
#define ST1_FRCT     (1 << 6)
#define ST1_C16      (1 << 7)
#define ST1_SXM      (1 << 8)
#define ST1_OVM      (1 << 9)
#define ST1_INTM     (1 << 11)
#define ST1_HM       (1 << 12)
#define ST1_XF       (1 << 13)
#define ST1_BRAF     (1 << 14)

/* PMST bit positions (per SPRU131: SST=0 SMUL=1 CLKOFF=2 DROM=3 APTS=4 OVLY=5 MP/MC=6) */
#define PMST_SST     (1 << 0)
#define PMST_SMUL    (1 << 1)
#define PMST_CLKOFF  (1 << 2)
#define PMST_DROM    (1 << 3)
#define PMST_APTS    (1 << 4)
#define PMST_OVLY    (1 << 5)
#define PMST_MP_MC   (1 << 6)
#define PMST_IPTR_SHIFT 7
#define PMST_IPTR_MASK (0x1FF << PMST_IPTR_SHIFT)

/* Interrupt vectors */
#define C54X_INT_RESET   0
#define C54X_INT_NMI     1
/* TMS320C54x interrupt mapping: vector = IMR_bit + 2
 * IMR bit 0 → INT0 → vec 2    IMR bit 5 → BRINT0 → vec 7
 * IMR bit 1 → INT1 → vec 3    IMR bit 6 → BXINT0 → vec 8
 * IMR bit 2 → INT2 → vec 4    IMR bit 7 → DMAC0  → vec 9
 * IMR bit 3 → INT3 → vec 5    IMR bit 8 → DMAC1  → vec 10
 * IMR bit 4 → TINT0→ vec 6    IMR bit 9 → INT4   → vec 11
 *                              IMR bit 10→ INT5   → vec 12 */
/* TMS320C54x interrupt vector mapping (SPRU131):
 * Vec 0: RESET     Vec 16: INT0 (IMR bit 0)
 * Vec 1: NMI       Vec 17: INT1 (IMR bit 1)
 * Vec 2: SINT17    Vec 18: INT2 (IMR bit 2)
 * Vec 3: SINT18    Vec 19: INT3 (IMR bit 3)
 * Vec 4: SINT19    Vec 20: TINT0 (IMR bit 4)
 * Vec 5: SINT20    Vec 21: BRINT0 (IMR bit 5)
 * ...              Vec 22: BXINT0 (IMR bit 6)
 *                  Vec 23: DMAC0 (IMR bit 7)
 *                  Vec 24: DMAC1 (IMR bit 8)
 * Formula: vec = imr_bit + 16 */
#define C54X_INT_FRAME_VEC   2   /* SINT17 (IMR bit 1) — enters boot code which dispatches frames */
#define C54X_INT_FRAME_BIT   1   /* IMR bit 1 */
#define C54X_NUM_INTS        16

typedef struct C54xState {
    /* Accumulators (40-bit) stored as int64 for convenience */
    int64_t a;   /* A accumulator: bits 39-0 */
    int64_t b;   /* B accumulator: bits 39-0 */

    /* Auxiliary registers */
    uint16_t ar[8];

    /* Other registers */
    uint16_t t;      /* Temporary register */
    uint16_t trn;    /* Transition register (Viterbi) */
    uint16_t sp;
    uint16_t bk;     /* Circular buffer size */
    uint16_t brc;    /* Block repeat counter */
    uint16_t rsa;    /* Block repeat start address */
    uint16_t rea;    /* Block repeat end address */

    /* Status registers */
    uint16_t st0;
    uint16_t st1;
    uint16_t pmst;

    /* Interrupt registers */
    uint16_t imr;
    uint16_t ifr;

    /* Program counter */
    uint32_t pc;     /* 16-bit (or 23-bit with XPC) */
    uint16_t xpc;

    /* Timer0 prescale counter (PSC) — not memory-mapped directly */
    uint16_t timer_psc;

    /* RPT state */
    uint16_t rpt_count;  /* remaining RPT iterations */
    uint16_t rpt_pc;     /* PC of repeated instruction */
    bool     rpt_active;
    uint16_t par;        /* Program Address Register (for READA/WRITA/MACD/MACP) */
    bool     par_set;
    bool     lk_used;    /* resolve_smem consumed extra word for lk */
    uint16_t mvpd_src;   /* MVPD auto-increment source address during RPT */

    /* RPTB state */
    bool     rptb_active;

    /* Memory */
    uint16_t prog[C54X_PROG_SIZE];   /* Program memory */
    uint16_t data[C54X_DATA_SIZE];   /* Data memory */

    /* API RAM pointer (shared with ARM calypso_trx.c) */
    uint16_t *api_ram;  /* points into ARM's dsp_ram[] */

    /* State */
    bool     running;
    bool     idle;       /* IDLE instruction executed */
    uint64_t cycles;
    uint32_t insn_count;

    /* BSP (Baseband Serial Port) — burst sample buffer */
    uint16_t bsp_buf[160];  /* burst samples from radio */
    int      bsp_len;       /* number of samples */
    int      bsp_pos;       /* read position */

    /* Debug */
    uint32_t unimpl_count;
    uint16_t last_unimpl;
} C54xState;

/* Feed burst samples to BSP (called by calypso_trx) */
void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n);

/* Create and initialize C54x state */
C54xState *c54x_init(void);

/* Load ROM dump from text file */
int c54x_load_rom(C54xState *s, const char *path);

/* Link API RAM (shared memory with ARM) */
void c54x_set_api_ram(C54xState *s, uint16_t *api_ram);

/* Reset the DSP */
void c54x_reset(C54xState *s);

/* Execute N instructions (returns actual count executed) */
int c54x_run(C54xState *s, int n_insns);

/* Raise an interrupt */
/* Send interrupt: vec = vector number (for PC), imr_bit = bit in IMR/IFR */
void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit);

/* Wake from IDLE */
void c54x_wake(C54xState *s);

#endif /* CALYPSO_C54X_H */

===== ./hw/arm/calypso/calypso_c54x.c =====
/*
 * calypso_c54x.c — TMS320C54x DSP emulator for Calypso
 *
 * Minimal C54x core: enough to run the Calypso DSP ROM for GSM
 * signal processing (Viterbi, deinterleaving, burst decode).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "calypso_c54x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_boot_trace = 0;

#define C54_LOG(fmt, ...) \
    fprintf(stderr, "[c54x] " fmt "\n", ##__VA_ARGS__)

/* ================================================================
 * Helpers
 * ================================================================ */

/* Sign-extend 40-bit accumulator */
static inline int64_t sext40(int64_t v)
{
    if (v & ((int64_t)1 << 39))
        v |= ~(((int64_t)1 << 40) - 1);
    else
        v &= ((int64_t)1 << 40) - 1;
    return v;
}

/* Saturate 40-bit to 32-bit (OVM mode) */
static inline int64_t sat32(int64_t v)
{
    if (v > 0x7FFFFFFF) return 0x7FFFFFFF;
    if (v < (int64_t)(int32_t)0x80000000) return (int64_t)(int32_t)0x80000000;
    return v;
}

/* Get ARP from ST0 */
static inline int arp(C54xState *s)
{
    return (s->st0 >> ST0_ARP_SHIFT) & 7;
}

/* Get DP from ST0 */
static inline uint16_t dp(C54xState *s)
{
    return s->st0 & ST0_DP_MASK;
}

/* Get ASM from ST1 (5-bit signed) */
static inline int asm_shift(C54xState *s)
{
    int v = s->st1 & ST1_ASM_MASK;
    if (v & 0x10) v |= ~0x1F;  /* sign extend */
    return v;
}

/* ================================================================
 * Memory access
 * ================================================================ */

static uint16_t data_read(C54xState *s, uint16_t addr)
{
    /* Log reads from API RAM at 0x08D4 (d_dsp_page) */
    if (addr == 0x08D4) {
        static int dsp_page_log = 0;
        if (dsp_page_log < 50) {
            C54_LOG("d_dsp_page RD = 0x%04x PC=0x%04x insn=%u SP=0x%04x",
                    s->api_ram ? s->api_ram[addr - 0x0800] : s->data[addr],
                    s->pc, s->insn_count, s->sp);
            dsp_page_log++;
        }
    }
    /* Timer registers (0x0024-0x0026) — read returns current value */
    if (addr == TIM_ADDR) return s->data[TIM_ADDR];
    if (addr == PRD_ADDR) return s->data[PRD_ADDR];
    if (addr == TCR_ADDR) {
        /* TCR: PSC is read from bits 9:6, rest from stored value */
        uint16_t tcr = s->data[TCR_ADDR] & ~TCR_PSC_MASK;
        tcr |= (s->timer_psc & 0xF) << TCR_PSC_SHIFT;
        return tcr;
    }

    /* MMR region */
    if (addr < 0x20) {
        switch (addr) {
        case MMR_IMR:  return s->imr;
        case MMR_IFR:
        {
            static int ifr_log = 0;
            if ((s->ifr & 0x0020) && ifr_log < 10) {
                C54_LOG("IFR READ=0x%04x (TINT0!) PC=0x%04x", s->ifr, s->pc);
                ifr_log++;
            }
            return s->ifr;
        }
        case MMR_ST0:  return s->st0;
        case MMR_ST1:  return s->st1;
        case MMR_AL:   return (uint16_t)(s->a & 0xFFFF);
        case MMR_AH:   return (uint16_t)((s->a >> 16) & 0xFFFF);
        case MMR_AG:   return (uint16_t)((s->a >> 32) & 0xFF);
        case MMR_BL:   return (uint16_t)(s->b & 0xFFFF);
        case MMR_BH:   return (uint16_t)((s->b >> 16) & 0xFFFF);
        case MMR_BG:   return (uint16_t)((s->b >> 32) & 0xFF);
        case MMR_T:    return s->t;
        case MMR_TRN:  return s->trn;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            return s->ar[addr - MMR_AR0];
        case MMR_SP:   return s->sp;
        case MMR_BK:   return s->bk;
        case MMR_BRC:  return s->brc;
        case MMR_RSA:  return s->rsa;
        case MMR_REA:  return s->rea;
        case MMR_PMST: return s->pmst;
        case MMR_XPC:  return s->xpc;
        default: return 0;
        }
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        if (s->api_ram) {
            uint16_t val = s->api_ram[addr - C54X_API_BASE];
            /* Log ALL API reads during interrupt handler (first 100) */
            static int api_rd_log = 0;
            if (api_rd_log < 100 && s->insn_count > 66000) {
                C54_LOG("API RD [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                        addr, val, s->pc, s->insn_count);
                api_rd_log++;
            }
            return val;
        }
    }

    /* Log data reads during SINT17 handler (PC in 0xFFC0-0xFFFF) */
    if (s->pc >= 0xFFC0 && s->insn_count > 66090) {
        static int handler_rd_log = 0;
        if (handler_rd_log < 30) {
            C54_LOG("H_RD [0x%04x]=0x%04x PC=0x%04x", addr, s->data[addr], s->pc);
            handler_rd_log++;
        }
    }

    return s->data[addr];
}

static void data_write(C54xState *s, uint16_t addr, uint16_t val)
{
    /* Timer registers (0x0024-0x0026) — before MMR check */
    if (addr == TCR_ADDR) {
        /* TRB: write 1 → reload TIM from PRD, PSC from TDDR */
        if (val & TCR_TRB) {
            s->data[TIM_ADDR] = s->data[PRD_ADDR];
            s->timer_psc = val & TCR_TDDR_MASK;
        }
        /* Store TCR without TRB (TRB is write-only, always reads 0) */
        s->data[TCR_ADDR] = val & ~TCR_TRB;
        return;
    }
    if (addr == TIM_ADDR) { s->data[TIM_ADDR] = val; return; }
    if (addr == PRD_ADDR) { s->data[PRD_ADDR] = val; return; }

    /* MMR region */
    if (addr < 0x20) {
        switch (addr) {
        case MMR_IMR:
            if (val != s->imr)
                C54_LOG("IMR change 0x%04x → 0x%04x PC=0x%04x", s->imr, val, s->pc);
            s->imr = val; return;
        case MMR_IFR:  s->ifr &= ~val; return;  /* write 1 to clear */
        case MMR_ST0:  s->st0 = val; return;
        case MMR_ST1:  s->st1 = val; return;
        case MMR_AL:   s->a = (s->a & ~0xFFFF) | val; return;
        case MMR_AH:   s->a = (s->a & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_AG:   s->a = (s->a & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_BL:   s->b = (s->b & ~0xFFFF) | val; return;
        case MMR_BH:   s->b = (s->b & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_BG:   s->b = (s->b & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_T:    s->t = val; return;
        case MMR_TRN:  s->trn = val; return;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            s->ar[addr - MMR_AR0] = val; return;
        case MMR_SP:   s->sp = val; return;
        case MMR_BK:   s->bk = val; return;
        case MMR_BRC:  s->brc = val; return;
        case MMR_RSA:  s->rsa = val; return;
        case MMR_REA:  s->rea = val; return;
        case MMR_PMST:
            if (val != s->pmst) {
                uint16_t old_iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                uint16_t new_iptr = (val >> PMST_IPTR_SHIFT) & 0x1FF;
                C54_LOG("PMST change 0x%04x → 0x%04x (IPTR=0x%03x→0x%03x OVLY=%d) PC=0x%04x SP=0x%04x insn=%u",
                        s->pmst, val, old_iptr, new_iptr, !!(val & PMST_OVLY), s->pc, s->sp, s->insn_count);
            }
            s->pmst = val; return;
        case MMR_XPC:
            /* Calypso DSP doesn't use extended program space.
             * Some code writes arbitrary values here during block copies. */
            s->xpc = val & 3;
            return;
        default: return;
        }
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        if (s->api_ram)
            s->api_ram[addr - C54X_API_BASE] = val;
        /* Always log writes to d_dsp_page (0x08D4) */
        if (addr == 0x08D4) {
            C54_LOG("DSP WR d_dsp_page = 0x%04x PC=0x%04x insn=%u", val, s->pc, s->insn_count);
        }
    }

    /* Log DARAM writes to code target area and count total */
    if (addr >= 0x0020 && addr < 0x0800) {
        static int dw_total = 0;
        dw_total++;
        if (addr >= 0x1200 && addr <= 0x1240) {
            C54_LOG("DARAM WR [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                    addr, val, s->pc, s->insn_count);
        }
        if (dw_total == 1 || dw_total == 100 || dw_total == 1000 || dw_total == 10000)
            C54_LOG("DARAM write count: %d (last: [0x%04x]=0x%04x)", dw_total, addr, val);
    }

    s->data[addr] = val;
}

/* Instruction fetch: uses mirrored PROM1 at 0x8000-0xFFFF, ignores XPC */
static uint16_t prog_fetch(C54xState *s, uint16_t pc)
{
    /* OVLY: map DARAM into program space 0x0080-0x27FF only.
     * Calypso has 10K words DARAM (0x0000-0x27FF).
     * PROM0 (0x7000-0xDFFF) is always accessible in program space. */
    if ((s->pmst & PMST_OVLY) && pc >= 0x80 && pc < 0x2800)
        return s->data[pc];
    /* prog_fetch: PC is always 16-bit, never uses XPC banking.
     * Per version 222807: only OVLY overlay applies to instruction fetch.
     * XPC is only used by prog_read (data/operand reads). */
    return s->prog[pc];
}
static uint16_t prog_read(C54xState *s, uint32_t addr)
{
    uint16_t addr16 = addr & 0xFFFF;
    /* OVLY: DARAM visible in program space for 0x0080-0x27FF */
    if ((s->pmst & PMST_OVLY) && addr16 >= 0x80 && addr16 < 0x2800)
        return s->data[addr16];
    /* For addresses >= 0x8000: use XPC to select extended page.
     * prog_read is used for data/operand reads (MVPD, FIRS coeff, etc.)
     * which need XPC banking — unlike prog_fetch which is PC-only. */
    if (addr16 >= 0x8000) {
        uint32_t ext = ((uint32_t)s->xpc << 16) | addr16;
        ext &= (C54X_PROG_SIZE - 1);
        return s->prog[ext];
    }
    return s->prog[addr16];
}

static void __attribute__((unused)) prog_write(C54xState *s, uint32_t addr, uint16_t val)
{
    uint16_t addr16 = addr & 0xFFFF;
    /* PROM1 (0xE000-0xFFFF) is ROM — reject writes */
    if (addr16 >= 0xE000) return;
    if ((s->pmst & PMST_OVLY) && addr16 >= 0x80 && addr16 < 0x2800)
        s->data[addr16] = val;
    if (addr16 >= 0x8000) {
        uint32_t ext = ((uint32_t)s->xpc << 16) | addr16;
        ext &= (C54X_PROG_SIZE - 1);
        s->prog[ext] = val;
    }
    s->prog[addr16] = val;
}

/* ================================================================
 * Addressing mode helpers
 * ================================================================ */

/* Resolve Smem operand: direct or indirect addressing.
 * Returns the data memory address. */
static uint16_t resolve_smem(C54xState *s, uint16_t opcode, bool *indirect)
{
    if (opcode & 0x80) {
        /* Indirect addressing */
        *indirect = true;
        int mod = (opcode >> 3) & 0x0F;
        int nar = opcode & 0x07;
        int cur_arp = arp(s);
        uint16_t addr = s->ar[cur_arp];

        /* Post-modify */
        switch (mod) {
        case 0x0: /* *ARn */
            break;
        case 0x1: /* *ARn- */
            s->ar[cur_arp]--;
            break;
        case 0x2: /* *ARn+ */
            s->ar[cur_arp]++;
            break;
        case 0x3: /* *+ARn */
            addr = ++s->ar[cur_arp];
            break;
        case 0x4: /* *ARn-0 */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0x5: /* *ARn+0 */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0x6: /* *ARn-0B (bit-reversed) */
            /* Simplified: just subtract */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0x7: /* *ARn+0B (bit-reversed) */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0x8: /* *ARn-% (circular) */
            if (s->bk == 0) s->ar[cur_arp]--;
            else {
                uint16_t base = s->ar[cur_arp] - (s->ar[cur_arp] % s->bk);
                s->ar[cur_arp]--;
                if (s->ar[cur_arp] < base) s->ar[cur_arp] = base + s->bk - 1;
            }
            break;
        case 0x9: /* *ARn+% (circular) */
            if (s->bk == 0) s->ar[cur_arp]++;
            else {
                uint16_t base = s->ar[cur_arp] - (s->ar[cur_arp] % s->bk);
                s->ar[cur_arp]++;
                if (s->ar[cur_arp] >= base + s->bk) s->ar[cur_arp] = base;
            }
            break;
        case 0xA: /* *ARn-0% */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0xB: /* *ARn+0% */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0xC: /* *(lk) — absolute address from next word */
            addr = prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        case 0xD: /* *+ARn(lk) — pre-add offset from next word */
            addr = s->ar[cur_arp] + prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        case 0xE: /* *ARn(lk) — post-add (addr=AR, then AR += lk) */
            addr = s->ar[cur_arp];
            s->ar[cur_arp] += prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        case 0xF: /* *+ARn(lk)% — circular with offset */
            addr = s->ar[cur_arp] + prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        }

        /* Update ARP */
        s->st0 = (s->st0 & ~ST0_ARP_MASK) | (nar << ST0_ARP_SHIFT);

        return addr;
    } else {
        /* Direct addressing: DP:offset */
        *indirect = false;
        uint16_t offset = opcode & 0x7F;
        return (dp(s) << 7) | offset;
    }
}

/* ================================================================
 * Instruction execution
 * ================================================================ */

/* Execute one instruction. Returns number of words consumed (1 or 2). */
/* PC ring buffer for pre-IDLE trace */
static uint16_t pc_ring[256];
static int pc_ring_idx = 0;

static int c54x_exec_one(C54xState *s)
{
    uint16_t op = prog_fetch(s, s->pc);
    uint16_t op2;
    bool ind;
    uint16_t addr;
    int consumed = 1;
    s->lk_used = false;  /* reset before each instruction */


    uint8_t hi4 = (op >> 12) & 0xF;
    uint8_t hi8 = (op >> 8) & 0xFF;

    /* Detect when DSP enters DARAM code zone (0x0080-0x27FF) from ROM */
    {
        static uint16_t prev_pc = 0;
        static int daram_log = 0;
        if (s->pc >= 0x0080 && s->pc < 0x2800 && prev_pc >= 0x7000 && daram_log < 3) {
            C54_LOG("ROM->DARAM jump: 0x%04x->0x%04x op=0x%04x insn=%u SP=0x%04x XPC=%d",
                    prev_pc, s->pc, op, s->insn_count, s->sp, s->xpc);
            C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                    pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                    pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                    pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                    pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
            daram_log++;
        }
        prev_pc = s->pc;
    }
    if (s->pc >= 0xFE00 && s->pc <= 0xFFFF && op == 0x0000) {
        static int nop_slide = 0;
        if (nop_slide == 0) {
            C54_LOG("NOP-SLIDE PC=0x%04x insn=%u SP=0x%04x PMST=0x%04x XPC=%d OVLY=%d",
                    s->pc, s->insn_count, s->sp, s->pmst, s->xpc, !!(s->pmst & PMST_OVLY));
            C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                    pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                    pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                    pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                    pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
        }
        nop_slide++;
    }

    switch (hi4) {
    case 0xF:
        /* 0xF --- large group: branches, misc, short immediates */
        if (op == 0xF495) return consumed + s->lk_used;  /* NOP */

        /* XC n, cond — Execute Conditionally (SPRU172C p.4-198)
         * Opcode: 1111 11N1 CCCCCCCC
         * 0xFDxx = XC 1, cond (N=0, execute next 1 instruction)
         * 0xFFxx = XC 2, cond (N=1, execute next 2 instructions)
         * If condition true: execute normally. If false: skip n instructions. */
        if (hi8 == 0xFD || hi8 == 0xFF) {
            int n_insns = (hi8 == 0xFF) ? 2 : 1;
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition code per SPRU172C condition table */
            /* Conditions can be combined (OR'd bits), but common single conditions: */
            if (cc == 0x00)      cond = true;                          /* UNC */
            else if (cc == 0x0C) cond = (s->st0 & ST0_C) != 0;       /* C */
            else if (cc == 0x08) cond = !(s->st0 & ST0_C);            /* NC */
            else if (cc == 0x30) cond = (s->st0 & ST0_TC) != 0;       /* TC */
            else if (cc == 0x20) cond = !(s->st0 & ST0_TC);           /* NTC */
            else if (cc == 0x45) cond = (sext40(s->a) == 0);          /* AEQ */
            else if (cc == 0x44) cond = (sext40(s->a) != 0);          /* ANEQ */
            else if (cc == 0x46) cond = (sext40(s->a) > 0);           /* AGT */
            else if (cc == 0x42) cond = (sext40(s->a) >= 0);          /* AGEQ */
            else if (cc == 0x43) cond = (sext40(s->a) < 0);           /* ALT */
            else if (cc == 0x47) cond = (sext40(s->a) <= 0);          /* ALEQ */
            else if (cc == 0x4D) cond = (sext40(s->b) == 0);          /* BEQ */
            else if (cc == 0x4C) cond = (sext40(s->b) != 0);          /* BNEQ */
            else if (cc == 0x4E) cond = (sext40(s->b) > 0);           /* BGT */
            else if (cc == 0x4A) cond = (sext40(s->b) >= 0);          /* BGEQ */
            else if (cc == 0x4B) cond = (sext40(s->b) < 0);           /* BLT */
            else if (cc == 0x4F) cond = (sext40(s->b) <= 0);          /* BLEQ */
            else if (cc == 0x70) cond = (s->st0 & ST0_OVA) != 0;     /* AOV */
            else if (cc == 0x60) cond = !(s->st0 & ST0_OVA);          /* ANOV */
            else if (cc == 0x78) cond = (s->st0 & ST0_OVB) != 0;     /* BOV */
            else if (cc == 0x68) cond = !(s->st0 & ST0_OVB);          /* BNOV */
            else {
                /* Combined conditions: OR the individual condition bits */
                cond = false;
                if (cc & 0x0C) cond |= ((cc & 0x04) ? (s->st0 & ST0_C) != 0 : !(s->st0 & ST0_C));
                if (cc & 0x30) cond |= ((cc & 0x10) ? (s->st0 & ST0_TC) != 0 : !(s->st0 & ST0_TC));
                if (cc & 0x40) {
                    int64_t acc = (cc & 0x08) ? s->b : s->a;
                    int c3 = cc & 0x07;
                    switch (c3) {
                    case 0x5: cond |= (sext40(acc) == 0); break;
                    case 0x4: cond |= (sext40(acc) != 0); break;
                    case 0x6: cond |= (sext40(acc) > 0); break;
                    case 0x2: cond |= (sext40(acc) >= 0); break;
                    case 0x3: cond |= (sext40(acc) < 0); break;
                    case 0x7: cond |= (sext40(acc) <= 0); break;
                    default: cond = true; break;
                    }
                }
                if (cc & 0x70 && !(cc & 0x40)) {
                    if (cc & 0x08) cond |= (s->st0 & ST0_OVB) != 0;
                    else           cond |= (s->st0 & ST0_OVA) != 0;
                }
            }
            if (!cond) {
                /* Skip n instructions — count consumed words for skipped insns */
                /* Each skipped insn is 1 word (simplified — multi-word insns rare after XC) */
                return 1 + n_insns;
            }
            return consumed + s->lk_used;  /* condition true: just advance past XC, execute next normally */
        }

        /* F4E2 = RSBX INTM (enable interrupts), F4E3 = SSBX INTM (disable interrupts) */
        if (op == 0xF4E2) { s->st1 &= ~ST1_INTM; return consumed + s->lk_used; }
        if (op == 0xF4E3) { s->st1 |= ST1_INTM; return consumed + s->lk_used; }
        /* F4E1: IDLE — halt until interrupt.
         * Per tic54x-opc.c: IDLE 0xF4E1 mask 0xFCFF. */
        if ((op & 0xFCFF) == 0xF4E1) {
            static int idle_log = 0;
            if (idle_log < 20)
                C54_LOG("IDLE @0x%04x INTM=%d IMR=0x%04x SP=0x%04x insns=%u",
                        s->pc, !!(s->st1 & ST1_INTM), s->imr, s->sp, s->insn_count);
            idle_log++;
            if (s->pc >= 0x8000 && s->pc < 0x8020) {
                return consumed + s->lk_used;
            }
            s->idle = true;
            return 0;
        }
        /* F4E0-F4FF: misc F4Ex — treat as NOP (most don't affect emulation) */
        if (op >= 0xF4E0 && op <= 0xF4FF && op != 0xF4E4 && op != 0xF4E5 && op != 0xF4EB) {
            return consumed + s->lk_used;
        }
        /* F4EB = RETE (return from interrupt, alternate encoding per tic54x-opc.c) */
        if (op == 0xF4EB) {
            if (s->pmst & PMST_APTS) {
                s->xpc = data_read(s, s->sp); s->sp++;
                if (s->xpc > 3) s->xpc &= 3;
            }
            uint16_t ra = data_read(s, s->sp); s->sp++;
            s->st1 &= ~ST1_INTM;
            s->pc = ra; return 0;
        }
        /* F4E4: FRET — far return.
         * Per tic54x-opc.c: 0xF4E4 mask 0xFFFF.
         * Pop XPC then PC from stack. */
        if (op == 0xF4E4) {
            s->xpc = data_read(s, s->sp); s->sp++;
            if (s->xpc > 3) s->xpc &= 3;
            uint16_t ra = data_read(s, s->sp); s->sp++;
            s->pc = ra;
            return 0;
        }
        /* F4E5: FRETE — far return from interrupt.
         * Pop XPC then PC, clear INTM. */
        if (op == 0xF4E5) {
            s->xpc = data_read(s, s->sp); s->sp++;
            if (s->xpc > 3) s->xpc &= 3;
            uint16_t ra = data_read(s, s->sp); s->sp++;
            s->st1 &= ~ST1_INTM;
            s->pc = ra;
            return 0;
        }
        if (hi8 == 0xF4) {
            /* F4xx: unconditional branch/call and special instructions.
             * Some F4xx instructions are 1-word (FRET, FRETE, RETE, TRAP, NOP, etc.)
             * Must check specific opcodes BEFORE the 2-word switch. */

            /* Note: 0xF4E4 = IDLE (handled above, not FRET).
             * Real FRET = 0xF072 (algebraic), handled in F0xx section. */
            /* NOP — F495 per SPRU172C p.4-121 */
            if (op == 0xF495) {
                return 1; /* 1-word NOP */
            }
            /* TRAP K — F4C0-F4DF per SPRU172C p.4-195:
             * SP-1, PC+1 → TOS, vector(IPTR*128 + K*4) → PC */
            if ((op & 0xFFE0) == 0xF4C0) {
                int k = op & 0x1F;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + k * 4;
                C54_LOG("TRAP #%d → PC=0x%04x (from PC=0x%04x)", k, s->pc,
                        (uint16_t)(s->pc - (iptr * 0x80 + k * 4) + 1 - 1));
                return 0;
            }

            /* F4xx arithmetic instructions (1-word, per tic54x-opc.c).
             * These MUST be checked before the 2-word branch/call switch. */
            {
                /* F483/F583: SAT src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF483) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    int64_t val = sext40(*acc);
                    if (val > 0x7FFFFFFFLL) *acc = sext40(0x7FFFFFFFLL);
                    else if (val < -0x80000000LL) *acc = sext40(-0x80000000LL);
                    return consumed + s->lk_used;
                }
                /* F484/F584: NEG src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF484) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(-val); else s->a = sext40(-val);
                    return consumed + s->lk_used;
                }
                /* F485/F585: ABS src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF485) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    if (val < 0) val = -val;
                    if (dst) s->b = sext40(val); else s->a = sext40(val);
                    return consumed + s->lk_used;
                }
                /* F48C/F58C: MPYA dst (mask FEFF, 1 word)
                 * Multiply T * A(high), accumulate into dst */
                if ((op & 0xFEFF) == 0xF48C) {
                    int dst = (op >> 8) & 1;
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                    return consumed + s->lk_used;
                }
                /* F48D/F58D: SQUR A,dst (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF48D) {
                    int dst = (op >> 8) & 1;
                    int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                    int64_t prod = (int64_t)ah * (int64_t)ah;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(prod); else s->a = sext40(prod);
                    return consumed + s->lk_used;
                }
                /* F48E/F58E: EXP src (mask FEFF, 1 word)
                 * Count leading sign bits of accumulator, store in T */
                if ((op & 0xFEFF) == 0xF48E) {
                    int src = (op >> 8) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    int exp = 0;
                    if (val == 0 || val == -1) { exp = 31; }
                    else {
                        uint64_t uv = (val < 0) ? ~val : val;
                        uv &= 0xFFFFFFFFFFULL;
                        /* Count leading zeros from bit 38 down */
                        for (int i = 38; i >= 0; i--) {
                            if (uv & (1ULL << i)) break;
                            exp++;
                        }
                        exp -= 8; /* EXP = leading sign bits - 8 */
                    }
                    s->t = (uint16_t)(int16_t)exp;
                    return consumed + s->lk_used;
                }
                /* F48F/F58F: NORM src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF48F) {
                    /* Simplified: NOP (normalize uses T from EXP) */
                    return consumed + s->lk_used;
                }
                /* F492/F592: MAX src (mask FEFF, 1 word) — keep max of A,B */
                if ((op & 0xFEFF) == 0xF492) {
                    int64_t sa = sext40(s->a), sb = sext40(s->b);
                    if (sa < sb) { s->a = s->b; s->st0 |= ST0_C; }
                    else { s->st0 &= ~ST0_C; }
                    return consumed + s->lk_used;
                }
                /* F493/F593: MIN src (mask FEFF, 1 word) — keep min of A,B */
                if ((op & 0xFEFF) == 0xF493) {
                    int64_t sa = sext40(s->a), sb = sext40(s->b);
                    if (sa > sb) { s->a = s->b; s->st0 |= ST0_C; }
                    else { s->st0 &= ~ST0_C; }
                    return consumed + s->lk_used;
                }
                /* F49E/F59E: SUBC src (mask FEFF, 1 word) — conditional subtract for division */
                if ((op & 0xFEFF) == 0xF49E) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    int64_t val = sext40(*acc);
                    if (val >= 0) { *acc = sext40((val << 1) + 1); }
                    else { *acc = sext40(val << 1); }
                    return consumed + s->lk_used;
                }
                /* F49F: DELAY (pipeline flush, NOP) */
                if (op == 0xF49F) { return consumed + s->lk_used; }
                /* F490/F590: ROR src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF490) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    uint16_t c = (s->st0 >> 8) & 1; /* carry */
                    uint16_t lsb = *acc & 1;
                    *acc = sext40(((uint64_t)(*acc & 0xFFFFFFFFFFULL) >> 1) | ((uint64_t)c << 39));
                    if (lsb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                    return consumed + s->lk_used;
                }
                /* F491/F591: ROL src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF491) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    uint16_t c = (s->st0 >> 8) & 1;
                    uint16_t msb = (*acc >> 39) & 1;
                    *acc = sext40(((*acc << 1) & 0xFFFFFFFFFFULL) | c);
                    if (msb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                    return consumed + s->lk_used;
                }
                /* F488/F588: MACA T,src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF488) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((src ? s->b : s->a) >> 16);
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                    return consumed + s->lk_used;
                }
                /* F486/F586: CMPL src (complement, mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF486) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    *acc = sext40(~(*acc) & 0xFFFFFFFFFFULL);
                    return consumed + s->lk_used;
                }
                /* F487/F587: RND src (round, mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF487) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    *acc = sext40(*acc + 0x8000);
                    return consumed + s->lk_used;
                }
                /* F480/F580: ADD src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF480) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                    return consumed + s->lk_used;
                }
                /* F481/F581: SUB src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF481) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                    return consumed + s->lk_used;
                }
                /* F482/F582: LD src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF482) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                /* F4xx accumulator shift/load (1-word, mask FCE0):
                 * F400: ADD src,shift,dst  F420: SUB  F440: LD  F460: SFTA */
                if ((op & 0xFCE0) == 0xF400) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF420) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF440) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF460) {
                    /* SFTA src,shift,dst — arithmetic shift accumulator */
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF4A0) {
                    /* SFTL src,shift,dst — logical shift accumulator */
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    uint64_t uv = (uint64_t)((src ? s->b : s->a) & 0xFFFFFFFFFFULL);
                    if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                    uv &= 0xFFFFFFFFFFULL;
                    if (dst) s->b = sext40(uv); else s->a = sext40(uv);
                    return consumed + s->lk_used;
                }
            }
                        /* F4Bx: RSBX -- reset bit in ST0 (bit 9=0, bit 8=0).
             * Per tic54x-opc.c: RSBX 0xF4B0 mask 0xFDF0. */
            if ((op & 0xFFF0) == 0xF4B0) {
                int bit = op & 0x0F;
                s->st0 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            /* Remaining F4xx: unhandled — treat as 1-word NOP */
            C54_LOG("F4xx unhandled: 0x%04x PC=0x%04x", op, s->pc);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xF0 || hi8 == 0xF1) {
            /* FIRS -- catch before other F1xx handlers */
            if (hi8 == 0xF1) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int xar = (op >> 4) & 0x07;
                int yar = op & 0x07;
                uint16_t xval = data_read(s, s->ar[xar]);
                uint16_t yval = data_read(s, s->ar[yar]);
                uint8_t xmod = (op >> 4) & 0xF;
                if ((xmod & 0x1) == 0) s->ar[xar]++; else s->ar[xar]--;
                if ((op & 0x08) == 0) s->ar[yar]++; else s->ar[yar]--;
                int16_t coeff = (int16_t)prog_read(s, op2);
                int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                int64_t product = (int64_t)ah * (int64_t)coeff;
                if (s->st1 & ST1_FRCT) product <<= 1;
                s->b = sext40(s->b + product);
                int32_t sum = (int32_t)(int16_t)xval + (int32_t)(int16_t)yval;
                s->a = sext40((int64_t)sum << 16);
                return consumed + s->lk_used;
            }
            /* F073: B pmad — unconditional branch (2-word).
             * Per tic54x-opc.c: 0xF073 mask 0xFFFF. */
            if (op == 0xF073) {
                op2 = prog_fetch(s, s->pc + 1);
                s->pc = op2;
                return 0;
            }
            /* F074: CALL pmad — unconditional call (2-word).
             * Per tic54x-opc.c: 0xF074 mask 0xFFFF.
             * Push PC+2 (return address), branch to pmad. */
            if (op == 0xF074) {
                op2 = prog_fetch(s, s->pc + 1);
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }







            /* F072: RPTB pmad — block repeat (2-word, non-delayed).
             * Per tic54x-opc.c: 0xF072 mask 0xFFFF.
             * RSA = PC+2, REA = pmad. */
            if (op == 0xF072) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 2);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed + s->lk_used;
            }
            /* F07x: RPT/RPTZ/misc (F072-F074 handled above) */
            if (op == 0xF070) {
                /* F070: RPT #lku — repeat next instruction lku+1 times (2-word) */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rpt_count = op2;
                s->rpt_active = true;
                s->pc += 2;
                return 0;
            }
            if (op == 0xF071) {
                /* F071: RPTZ dst, #lku — zero accumulator and repeat (2-word) */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int dst = (op >> 8) & 1; /* bit 8 via FEFF mask */
                if (dst) s->b = 0; else s->a = 0;
                s->rpt_count = op2;
                s->rpt_active = true;
                s->pc += 2;
                return 0;
            }
            if ((op & 0xFFF0) == 0xF070) {
                /* F075-F07F: undefined, treat as 1-word NOP */
                return consumed + s->lk_used;
            }
            /* F0Bx/F1Bx: RSBX/SSBX */
            if ((op & 0x00F0) == 0x00B0) {
                int bit = op & 0x0F;
                int set = (op >> 8) & 1;
                int st = (op >> 5) & 1;
                if (st == 0) { if (set) s->st0 |= (1<<bit); else s->st0 &= ~(1<<bit); }
                else         { if (set) s->st1 |= (1<<bit); else s->st1 &= ~(1<<bit); }
                return consumed + s->lk_used;
            }
            /* F0xx/F1xx ALU with #lk immediate (2-word).
             * Per tic54x-opc.c: bits 7:4 = op (0=ADD,1=SUB,2=LD,3=AND,4=OR,5=XOR),
             * bit 8 = SRC (ADD/SUB/AND/OR/XOR) or DST (LD), bit 9 = DST,
             * bits 3:0 = shift. Second word = lk. */
            {
                uint8_t alu_op = (op >> 4) & 0xF;
                if (alu_op <= 5) {
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    int shift = op & 0xF;
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t src_val = src_sel ? s->b : s->a;
                    int64_t *dst = (alu_op == 2)
                        ? (src_sel ? &s->b : &s->a)
                        : (dst_sel ? &s->b : &s->a);
                    int64_t lk_val;
                    if (alu_op <= 2)
                        lk_val = (int64_t)(int16_t)op2 << shift;
                    else
                        lk_val = (int64_t)(uint16_t)op2 << shift;
                    switch (alu_op) {
                    case 0: *dst = sext40(src_val + lk_val); break; /* ADD */
                    case 1: *dst = sext40(src_val - lk_val); break; /* SUB */
                    case 2: *dst = sext40(lk_val); break;           /* LD  */
                    case 3: *dst = src_val & lk_val; break;         /* AND */
                    case 4: *dst = src_val | lk_val; break;         /* OR  */
                    case 5: *dst = src_val ^ lk_val; break;         /* XOR */
                    }
                    return consumed + s->lk_used;
                }
                if (alu_op == 6) {
                    /* F06x: ADD/SUB/LD/AND/OR/XOR #lk,16 + MPY/MAC #lk */
                    uint8_t sub6 = op & 0xF;
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t src_val = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    switch (sub6) {
                    case 0: *dst = sext40(src_val + ((int64_t)(int16_t)op2 << 16)); break;
                    case 1: *dst = sext40(src_val - ((int64_t)(int16_t)op2 << 16)); break;
                    case 2: dst = src_sel ? &s->b : &s->a;
                            *dst = sext40((int64_t)(int16_t)op2 << 16); break;
                    case 3: *dst = src_val & ((int64_t)(uint16_t)op2 << 16); break;
                    case 4: *dst = src_val | ((int64_t)(uint16_t)op2 << 16); break;
                    case 5: *dst = src_val ^ ((int64_t)(uint16_t)op2 << 16); break;
                    case 6: /* MPY #lk, dst */
                            dst = src_sel ? &s->b : &s->a;
                            { int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                              if (s->st1 & ST1_FRCT) p <<= 1;
                              *dst = sext40(p); } break;
                    case 7: /* MAC #lk, src[,dst] */
                            { int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                              if (s->st1 & ST1_FRCT) p <<= 1;
                              *dst = sext40(src_val + p); } break;
                    default: break;
                    }
                    return consumed + s->lk_used;
                }
                if (alu_op >= 8) {
                    /* F08x-F0Fx: accumulator-to-accumulator ops (1-word).
                     * bits 7:5 = op (100=AND,101=OR,110=XOR,111=SFTL)
                     * bits 4:0 = shift (signed 5-bit), bits 9:8 = src,dst */
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t sv = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    int shift = op & 0x1F;
                    if (shift > 15) shift -= 32;
                    uint8_t aop = (op >> 5) & 0x7;
                    int64_t shifted;
                    if (shift >= 0) shifted = sv << shift;
                    else            shifted = sv >> (-shift);
                    switch (aop) {
                    case 4: *dst = sext40(sv) & sext40(shifted); break;
                    case 5: *dst = sext40(sv) | sext40(shifted); break;
                    case 6: *dst = sext40(sv) ^ sext40(shifted); break;
                    case 7: { uint64_t uv = (uint64_t)(sv & 0xFFFFFFFFFFULL);
                              if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                              *dst = sext40(uv & 0xFFFFFFFFFFULL); } break;
                    default: break;
                    }
                    return consumed + s->lk_used;
                }
            }
            goto unimpl;
        }
        /* F272/F274/F273: RPTBD/CALLD/RETD — must check BEFORE LMS */
        if (op == 0xF272) {
            /* RPTBD pmad — delayed block repeat (2 words).
             * Delayed: 2 delay slots after the 2-word instruction.
             * RSA = PC + 4 (skip RPTBD + 2 delay slot words). */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (op == 0xF274) {
            /* CALLD pmad — delayed call (2 words, 2 delay slots).
             * Push PC+4 (past CALLD + 2 delay slots), branch to pmad. */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 4));
            s->pc = op2;
            return 0;
        }
        if (op == 0xF273) {
            /* RETD — delayed return (1 word) */
            uint16_t ra = data_read(s, s->sp); s->sp++;
            s->pc = ra;
            return 0;
        }
        /* LMS Xmem, Ymem — Least Mean Square step (1-word dual-operand)
         * Encoding: 1111 001D XXXX YYYY
         * Per SPRU172C: dst += T * Xmem; Ymem += rnd(AH * T); T = Xmem */
        if (hi8 == 0xF2 || hi8 == 0xF3) {
            int xar_l = (op >> 4) & 0x07;
            int yar_l = op & 0x07;
            uint16_t xval_l = data_read(s, s->ar[xar_l]);
            uint16_t yval_l = data_read(s, s->ar[yar_l]);
            /* MAC: dst += T * Xmem */
            int64_t prod_l = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_l;
            if (s->st1 & ST1_FRCT) prod_l <<= 1;
            int dst_l = hi8 & 1;
            if (dst_l) s->b = sext40(s->b + prod_l);
            else       s->a = sext40(s->a + prod_l);
            /* LMS coefficient update: Ymem += rnd(AH * T) */
            int16_t ah_l = (int16_t)((s->a >> 16) & 0xFFFF);
            int32_t update = (int32_t)ah_l * (int32_t)(int16_t)s->t;
            if (s->st1 & ST1_FRCT) update <<= 1;
            update += 0x8000; /* round */
            int16_t new_ym = (int16_t)yval_l + (int16_t)(update >> 16);
            data_write(s, s->ar[yar_l], (uint16_t)new_ym);
            /* T = Xmem */
            s->t = xval_l;
            /* Post-modify */
            if ((op >> 7) & 1) s->ar[xar_l]--; else s->ar[xar_l]++;
            if ((op & 0x08) == 0) s->ar[yar_l]++; else s->ar[yar_l]--;
            return consumed + s->lk_used;
        }
        if (op == 0xF495) {
            /* NOP */
            return consumed + s->lk_used;
        }
        if (op == 0xF4E2) { s->st1 &= ~ST1_INTM; return consumed + s->lk_used; }
        if (op == 0xF4E3) { s->st1 |= ST1_INTM; return consumed + s->lk_used; }
        if ((op & 0xFCFF) == 0xF4E1) {
            /* IDLE */
            s->idle = true;
            return 0;  /* PC stays on IDLE; wake code advances PC */
        }
        if (op == 0xF4E4) {
            /* FRET — far return */
            s->xpc = data_read(s, s->sp); s->sp++;
            if (s->xpc > 3) s->xpc &= 3;
            uint16_t ra = data_read(s, s->sp); s->sp++;
            s->pc = ra;
            return 0;
        }
        /* FXXX short immediates and misc */
        if (hi8 == 0xF0 || hi8 == 0xF1) {
        /* FIRS Xmem, Ymem, pmad -- Symmetric FIR filter step (2 words)
         * Encoding: 1111 0001 XXXX YYYY + pmad
         * Execution: B = rnd(B + A(32-16) * prog[pmad])
         *            A = (Xmem + Ymem) << 16
         * Per SPRU172C p.4-59 */
        if (hi8 == 0xF1 && (op & 0xFF00) == 0xF100) {
            uint8_t xmod = (op >> 4) & 0xF;
            uint8_t ymod = op & 0xF;
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* Resolve Xmem and Ymem via dual indirect addressing */
            /* Xmem uses ARP, Ymem uses AR(ymod & 7) */
            int xar = (op >> 4) & 0x07;
            int yar = ymod & 0x07;
            uint16_t xval = data_read(s, s->ar[xar]);
            uint16_t yval = data_read(s, s->ar[yar]);
            /* Post-modify: Xmem uses xmod, Ymem uses ymod */
            if ((xmod & 0x1) == 0) s->ar[xar]++; else s->ar[xar]--;
            if ((ymod & 0x8) == 0) s->ar[yar]++; else s->ar[yar]--;
            /* Read coefficient from program memory */
            int16_t coeff = (int16_t)prog_read(s, op2);
            /* B += A(32-16) * coeff */
            int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
            int64_t product = (int64_t)ah * (int64_t)coeff;
            if (s->st1 & ST1_FRCT) product <<= 1;
            s->b = sext40(s->b + product);
            /* A = (Xmem + Ymem) << 16 */
            int32_t sum = (int32_t)(int16_t)xval + (int32_t)(int16_t)yval;
            s->a = sext40((int64_t)sum << 16);
            return consumed + s->lk_used;
        }
            /* F0xx/F1xx: various - SSBX, RSBX, etc. */
            if ((op & 0xFFF0) == 0xF070) {
                /* F07x: misc control */
                uint8_t sub = op & 0x0F;
                if (sub == 0x3) {
                    /* F073: B pmad — unconditional branch (2-word) */
                    op2 = prog_fetch(s, s->pc + 1);
                    s->pc = op2;
                    return 0;
                }
                if (sub == 0x4) {
                    /* F074: CALL pmad — unconditional call (2-word).
                     * Push PC+2, branch to pmad. */
                    op2 = prog_fetch(s, s->pc + 1);
                    s->sp--;
                    data_write(s, s->sp, (uint16_t)(s->pc + 2));
                    s->pc = op2;
                    return 0;
                }








                goto unimpl;
            }
            /* F0Bx / F1Bx: RSBX/SSBX */
            if ((op & 0xFE00) == 0xF000 && (op & 0x00F0) == 0x00B0) {
                /* RSBX/SSBX bit in ST0/ST1 */
                int bit = op & 0x0F;
                int set = (op >> 8) & 1;
                int st = (op >> 5) & 1;  /* 0=ST0, 1=ST1 */
                if (st == 0) {
                    if (set) s->st0 |= (1 << bit);
                    else     s->st0 &= ~(1 << bit);
                } else {
                    if (set) s->st1 |= (1 << bit);
                    else     s->st1 &= ~(1 << bit);
                }
                return consumed + s->lk_used;
            }
        }
        /* F8xx: branches, RPT, BANZ, CALL, RET variants */
        if (hi8 == 0xF8) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x0) {
                /* F600/F601: ABS src[,dst] — absolute value of accumulator */
                int src = op & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val < 0) val = -val;
                *acc = sext40(val);
                /* Set C if input was -2^39 (saturate), clear OVx if OVM */
                return consumed + s->lk_used;
            }
            if (sub == 0x2) {
                /* F82x: RPTB pmad */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 2);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed + s->lk_used;
            }
            if (sub == 0x3) {
                /* F83x: RPT #k (short) — advance PC past RPT+imm */
                op2 = prog_fetch(s, s->pc + 1);
                s->rpt_count = op2;
                s->rpt_active = true;
                s->pc += 2;
                return 0;
            }
            /* F88x/F89x: B pmad — unconditional branch (2 words)
             * F8Ax/F8Bx: BD pmad — delayed branch (2 words)
             * Per SPRU172C: 1111 1000 1SSS DDDD pmad */
            if (sub >= 0x8 && sub <= 0xB) {
                op2 = prog_fetch(s, s->pc + 1);
                s->pc = op2;
                return 0;
            }
            /* F86x/F87x: BANZ *ARn, pmad — branch if ARn != 0 (2 words) */
            if (sub == 0x6 || sub == 0x7) {
                op2 = prog_fetch(s, s->pc + 1);
                int ar_idx = op & 0x07;
                if (s->ar[ar_idx] != 0) {
                    s->ar[ar_idx]--;
                    s->pc = op2;
                    return 0;
                }
                return 2;  /* skip 2 words, fall through */
            }
            /* F84x/F85x: BANZ with condition / CALL variants */
            if (sub == 0x4 || sub == 0x5) {
                op2 = prog_fetch(s, s->pc + 1);
                /* BANZ ARn, pmad */
                int ar_idx = op & 0x07;
                if (s->ar[ar_idx] != 0) {
                    s->ar[ar_idx]--;
                    s->pc = op2;
                    return 0;
                }
                return 2;
            }
            /* F8Cx-F8Fx: CALL/CALLD pmad (2 words) */
            if (sub >= 0xC) {
                op2 = prog_fetch(s, s->pc + 1);
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }
            /* F80x-F81x: BANZ pmad, Smem (2 words)
             * Per SPRU172C + tic54x-opc.c: entire F8xx range is BANZ.
             * Smem determines AR post-modify; test AR(ARP) != 0 → branch. */
            if (sub <= 0x1) {
                uint16_t ar_idx = arp(s);
                uint16_t old_ar = s->ar[ar_idx];
                addr = resolve_smem(s, op, &ind);
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->ar[ar_idx]--; /* BANZ always decrements AR(ARP) */
                if (old_ar != 0) {
                    s->pc = op2;
                    return 0;
                }
                return consumed + s->lk_used;
            }
            /* Fallback: RPT Smem (F8xx sub not handled above) */
            addr = resolve_smem(s, op, &ind);
            s->rpt_count = data_read(s, addr);
            s->rpt_active = true;
            s->pc += consumed;
            return 0;
        }
        /* F2xx: NORM *ARn, dst — Normalize accumulator
         * Per SPRU172C p.4-120: 1111 0q10 0111 D000
         * Shifts accumulator left by 1 if MSBs match, decrements ARn.
         * Simplified: treat as NOP for DSP signal processing. */
        if (hi8 == 0xF2) {
            /* F2xx: per tic54x-opc.c (GDB binutils):
             * F272 = RPTBD pmad (repeat block delayed, 2 words)
             * F274 = CALLD pmad (call delayed, 2 words)
             * F273 = RETD (return delayed, 1 word) */
            if (op == 0xF272) {
                /* RPTBD pmad — delayed block repeat (2 words).
                 * REA = pmad, RSA = PC+4 (2 delay slots). */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 4);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed + s->lk_used;
            }
            if (op == 0xF274) {
                /* CALLD pmad — delayed call (2 words, 2 delay slots).
                 * Push PC+4 (past CALLD + 2 delay slots), branch to pmad. */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 4));
                s->pc = op2;
                return 0;
            }
            if (op == 0xF273) {
                /* RETD — delayed return (1 word). Pop PC. */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->pc = ra;
                return 0;
            }
            /* Other F27x: NORM *ARn, dst */
            if ((op & 0xFFF0) == 0xF270) {
                int dst = (op >> 3) & 1;
                int64_t *acc = dst ? &s->b : &s->a;
                int bit39 = (*acc >> 39) & 1;
                int bit38 = (*acc >> 38) & 1;
                if (bit39 == bit38) {
                    *acc = sext40(*acc << 1);
                    s->ar[arp(s)]--;
                    s->st0 &= ~ST0_TC;
                } else {
                    s->st0 |= ST0_TC;
                }
                return consumed + s->lk_used;
            }
            goto unimpl;
        }
        /* F3xx: various */
        if (hi8 == 0xF3) {
            uint8_t sub3 = (op >> 5) & 0x07;
            if (sub3 == 0) {
                /* F300-F31F: INTR k — software interrupt (branch to vector k) */
                int vec = op & 0x1F;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                s->st1 |= ST1_INTM;
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + vec * 4;
                return 0;
            }
            /* F320+: LD #k9, DP */
            uint16_t k9 = op & 0x01FF;
            s->st0 = (s->st0 & ~ST0_DP_MASK) | k9;
            return consumed + s->lk_used;
        }
        /* F6xx: various — LD/ST acc-acc, ABDST, SACCD, etc. */
        if (hi8 == 0xF6) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x0) {
                /* F600/F601: ABS src[,dst] — absolute value of accumulator */
                int src = op & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val < 0) val = -val;
                *acc = sext40(val);
                /* Set C if input was -2^39 (saturate), clear OVx if OVM */
                return consumed + s->lk_used;
            }
            if (sub == 0x2) {
                /* F62x: LD A, dst_shift, B or LD B, dst_shift, A */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed + s->lk_used;
            }
            if (sub == 0x6) {
                /* F66x: LD A/B with shift to other acc */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed + s->lk_used;
            }
            if (sub == 0xB) {
                /* F6Bx: RSBX -- reset bit in ST1 (bit 9=1, bit 8=0).
                 * Per tic54x-opc.c: RSBX 0xF4B0 mask 0xFDF0 covers F6Bx. */
                int bit = op & 0x0F;
                s->st1 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            if (sub >= 0x8) {
                /* F68x-F6Fx: MVDD Xmem, Ymem — dual data-memory operand move
                 * Encoding: 1111 0110 XXXX YYYY
                 *   bit 7   = Xmod (0=inc, 1=dec)
                 *   bits 6:4 = Xar  (source AR register)
                 *   bit 3   = Ymod (0=inc, 1=dec)
                 *   bits 2:0 = Yar  (dest AR register) */
                int xar = (op >> 4) & 0x07;
                int yar = op & 0x07;
                uint16_t val = data_read(s, s->ar[xar]);
                data_write(s, s->ar[yar], val);
                if ((op >> 7) & 1) s->ar[xar]--; else s->ar[xar]++;
                if ((op >> 3) & 1) s->ar[yar]--; else s->ar[yar]++;
                return consumed + s->lk_used;
            }
            /* Other F6xx: treat as NOP for now */
            return consumed + s->lk_used;
        }
        /* F8xx: BANZ pmad, Smem — Branch if AR not zero (2-word)
         * Per SPRU172C: modify AR per Smem addressing; if AR[ARP] != 0, branch to pmad
         * Encoding: 1111 1000 IAAA AAAA + pmad */
        if (hi8 == 0xF8) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t cur_arp_val = s->ar[arp(s)];
            {
                static int banz_log = 0;
                if (banz_log < 5)
                    C54_LOG("BANZ PC=0x%04x ARP=%d AR[ARP]=0x%04x target=0x%04x %s",
                            s->pc, arp(s), cur_arp_val, op2,
                            cur_arp_val != 0 ? "TAKEN" : "NOT TAKEN");
                banz_log++;
            }
            if (cur_arp_val != 0) {
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* F5xx: SSBX or RPT #k */
        if (hi8 == 0xF5) {
            /* F5Bx: SSBX -- set bit in ST0 (bit 9=0, bit 8=1).
             * Per tic54x-opc.c: SSBX 0xF5B0 mask 0xFDF0. */
            if ((op & 0xFFF0) == 0xF5B0) {
                int bit = op & 0x0F;
                s->st0 |= (1 << bit);
                return consumed + s->lk_used;
            }
            /* RPT #k (short immediate) — advance PC past RPT now,
             * the NEXT instruction at PC+1 will be repeated. */
            s->rpt_count = op & 0xFF;
            s->rpt_active = true;
            s->pc += 1;
            return 0;
        }
        /* F7xx: LD/ST #k to various registers */
        if (hi8 == 0xF7) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x0) {
                /* F600/F601: ABS src[,dst] — absolute value of accumulator */
                int src = op & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val < 0) val = -val;
                *acc = sext40(val);
                /* Set C if input was -2^39 (saturate), clear OVx if OVM */
                return consumed + s->lk_used;
            }
            uint16_t k = op & 0xFF;
            switch (sub) {
            case 0x0: /* F70x: LD #k8, ASM */
                s->st1 = (s->st1 & ~ST1_ASM_MASK) | (k & ST1_ASM_MASK);
                break;
            case 0x1: /* F71x: LD #k8, AR0 */
                s->ar[0] = k; break;
            case 0x2: /* F72x: LD #k8, AR1 */
                s->ar[1] = k; break;
            case 0x3: s->ar[2] = k; break;
            case 0x4: s->ar[3] = k; break;
            case 0x5: s->ar[4] = k; break;
            case 0x6: s->ar[5] = k; break;
            case 0x7: s->ar[6] = k; break;
            case 0x8: /* F78x: LD #k8, T */
                s->t = (s->st1 & ST1_SXM) ? (uint16_t)(int8_t)k : k; break;
            case 0x9: /* F79x: LD #k8, DP */
                s->st0 = (s->st0 & ~ST0_DP_MASK) | (k & ST0_DP_MASK); break;
            case 0xA: /* F7Ax: LD #k8, ARP */
                s->st0 = (s->st0 & ~ST0_ARP_MASK) | ((k & 7) << ST0_ARP_SHIFT); break;
            case 0xB: /* F7Bx: SSBX -- set bit in ST1 (bit 9=1, bit 8=1).
                       * Per tic54x-opc.c: SSBX 0xF5B0 mask 0xFDF0 covers F7Bx. */
                s->st1 |= (1 << (op & 0x0F)); break;
            case 0xC: s->bk = k; break;
            case 0xD: s->sp = k; break;
            case 0xE: /* F7Ex: LD #k8, BRC */
                s->brc = k; break;
            case 0xF: /* F7Fx: LD #k8, ... */
                break;
            }
            return consumed + s->lk_used;
        }
        /* F9xx: RPT #lk (16-bit immediate) */
        if (hi8 == 0xF9) {
            /* F9xx: BC pmad, cond — conditional branch (2 words)
             * Per SPRU172C + tic54x-opc.c: F900 mask FF00 = BC.
             * Encoding: 1111 1001 CCCC QQQQ pmad */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond_code = (op >> 4) & 0xF;
            uint8_t qual = op & 0xF;
            bool take = false;
            int64_t acc = (qual & 0x8) ? s->b : s->a;
            switch (cond_code) {
            case 0x0: take = true; break;
            case 0x1: take = (acc < 0); break;
            case 0x2: take = (acc <= 0); break;
            case 0x3: take = (acc != 0); break;
            case 0x4: take = (acc == 0); break;
            case 0x5: take = (acc >= 0); break;
            case 0x6: take = (acc > 0); break;
            case 0x8: take = !!(s->st0 & ST0_TC); break;
            case 0x9: take = !(s->st0 & ST0_TC); break;
            case 0xA: take = !!(s->st0 & ST0_C); break;
            case 0xB: take = !(s->st0 & ST0_C); break;
            default: take = true; break;
            }
            if (take) {
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FAxx: RPT Smem or conditional ops */
        if (hi8 == 0xFA) {
            /* FA3x: BC with delay, FA4x: conditional etc. */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* Simplified: treat as delayed branch */
            s->pc = op2;
            return 0;
        }
        /* FBxx: LD #k, 16, A/B (short immediate shift 16) */
        if (hi8 == 0xFB) {
            int8_t k = (int8_t)(op & 0xFF);
            int64_t v = (int64_t)k << 16;
            s->a = sext40(v);
            return consumed + s->lk_used;
        }
        /* FCxx: LD #k, 16, B */
        /* FCxx: RC cond / RET -- return conditional (1-word).
         * Per tic54x-opc.c: RET=0xFC00, RC=0xFC00 mask 0xFF00. */
        if (hi8 == 0xFC) {
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition per tic54x-opc.c encoding:
             * CC1=0x40: accumulator test, CCB=0x08: use B (else A)
             * EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
             * OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08 */
            if (cc == 0x00) cond = true; /* UNC */
            else if (cc & 0x40) {
                /* Accumulator condition */
                int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
                uint8_t test = cc & 0x07;
                bool ov = (cc & 0x08) ? (s->st0 & (1<<9)/*OVB*/) : (s->st0 & (1<<8)/*OVA*/);
                if ((cc & 0x70) == 0x70) cond = ov;        /* AOV/BOV */
                else if ((cc & 0x70) == 0x60) cond = !ov;  /* ANOV/BNOV */
                else {
                    switch (test) {
                    case 0x05: cond = (acc == 0); break;  /* EQ */
                    case 0x04: cond = (acc != 0); break;  /* NEQ */
                    case 0x03: cond = (acc < 0); break;   /* LT */
                    case 0x07: cond = (acc <= 0); break;  /* LEQ */
                    case 0x06: cond = (acc > 0); break;   /* GT */
                    case 0x02: cond = (acc >= 0); break;  /* GEQ */
                    default: cond = true; break;
                    }
                }
            }
            else if ((cc & 0x30) == 0x30) cond = (s->st0 & ST0_TC) != 0; /* TC */
            else if ((cc & 0x30) == 0x20) cond = !(s->st0 & ST0_TC);     /* NTC */
            else if ((cc & 0x0C) == 0x0C) cond = (s->st0 & ST0_C) != 0;  /* C */
            else if ((cc & 0x0C) == 0x08) cond = !(s->st0 & ST0_C);      /* NC */
            else cond = true; /* unknown: take it */
            if (cond) {
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->pc = ra;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FDxx: LD #k, A (no shift) */
        if (hi8 == 0xFD) {
            int8_t k = (int8_t)(op & 0xFF);
            s->a = sext40((int64_t)k);
            return consumed + s->lk_used;
        }
        /* FExx: RCD cond / RETD -- return conditional delayed (1-word).
         * Per tic54x-opc.c: RETD=0xFE00, RCD=0xFE00 mask 0xFF00.
         * Simplified: immediate return (delay slots skipped). */
        if (hi8 == 0xFE) {
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition per tic54x-opc.c encoding:
             * CC1=0x40: accumulator test, CCB=0x08: use B (else A)
             * EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
             * OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08 */
            if (cc == 0x00) cond = true; /* UNC */
            else if (cc & 0x40) {
                /* Accumulator condition */
                int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
                uint8_t test = cc & 0x07;
                bool ov = (cc & 0x08) ? (s->st0 & (1<<9)/*OVB*/) : (s->st0 & (1<<8)/*OVA*/);
                if ((cc & 0x70) == 0x70) cond = ov;        /* AOV/BOV */
                else if ((cc & 0x70) == 0x60) cond = !ov;  /* ANOV/BNOV */
                else {
                    switch (test) {
                    case 0x05: cond = (acc == 0); break;  /* EQ */
                    case 0x04: cond = (acc != 0); break;  /* NEQ */
                    case 0x03: cond = (acc < 0); break;   /* LT */
                    case 0x07: cond = (acc <= 0); break;  /* LEQ */
                    case 0x06: cond = (acc > 0); break;   /* GT */
                    case 0x02: cond = (acc >= 0); break;  /* GEQ */
                    default: cond = true; break;
                    }
                }
            }
            else if ((cc & 0x30) == 0x30) cond = (s->st0 & ST0_TC) != 0; /* TC */
            else if ((cc & 0x30) == 0x20) cond = !(s->st0 & ST0_TC);     /* NTC */
            else if ((cc & 0x0C) == 0x0C) cond = (s->st0 & ST0_C) != 0;  /* C */
            else if ((cc & 0x0C) == 0x08) cond = !(s->st0 & ST0_C);      /* NC */
            else cond = true; /* unknown: take it */
            if (cond) {
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->pc = ra;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FFxx: ADD/SUB short immediate */
        if (hi8 == 0xFF) {
            int8_t k = (int8_t)(op & 0x7F);
            int dst = (op >> 7) & 1;
            /* Typically ADD #k, A or SUB */
            if (dst) s->b = sext40(s->b + ((int64_t)k << 16));
            else     s->a = sext40(s->a + ((int64_t)k << 16));
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xE:
        /* Exxxx: single-word ALU, status, misc */
        /* CMPS src, Smem — Compare, Select, and Store (Viterbi)
         * Encoding: 1110 00SD IAAAAAAA (1 word)
         * Per SPRU172C p.4-35: if |A(32-16)| >= |Smem| then TC=1,
         * TRN = (TRN<<1)|1, dst=A; else TC=0, TRN=(TRN<<1), dst=Smem<<16 */
        if ((op & 0xFC00) == 0xE000) {
            int src_s = (op >> 9) & 1;
            int dst_d = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t acc = src_s ? s->b : s->a;
            int32_t ah = (int32_t)((acc >> 16) & 0xFFFF);
            if (ah < 0) ah = -ah;
            int32_t sv = (int16_t)val;
            if (sv < 0) sv = -sv;
            s->trn <<= 1;
            if (ah >= sv) {
                s->st0 |= ST0_TC;
                s->trn |= 1;
            } else {
                s->st0 &= ~ST0_TC;
                int64_t nv = (int64_t)(int16_t)val << 16;
                if (dst_d) s->b = sext40(nv); else s->a = sext40(nv);
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xFE00) == 0xEA00) {
            /* LD #k9, DP -- Load Data Page pointer with 9-bit immediate.
             * Encoding: 1110 101k kkkk kkkk (1 word)
             * Per SPRU172C p.4-70 and GNU binutils tic54x-opc.c */
            uint16_t k9 = op & 0x01FF;
            s->st0 = (s->st0 & ~ST0_DP_MASK) | k9;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEC) {
            /* EC: BC pmad, cond (conditional branch, 2 words)
             * Same condition table as CC (tic54x-dis.c cc2[]) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            int64_t sa = sext40(s->a), sb = sext40(s->b);
            bool take = false;
            switch (cond) {
            case 0x00: take = true; break;          /* UNC */
            case 0x02: take = (sa >= 0); break;     /* AGEQ */
            case 0x03: take = (sa < 0); break;      /* ALT */
            case 0x04: take = (sa != 0); break;     /* ANEQ */
            case 0x05: take = (sa == 0); break;     /* AEQ */
            case 0x06: take = (sa > 0); break;      /* AGT */
            case 0x07: take = (sa <= 0); break;     /* ALEQ */
            case 0x0A: take = (sb >= 0); break;     /* BGEQ */
            case 0x0B: take = (sb < 0); break;      /* BLT */
            case 0x0C: take = (sb != 0); break;     /* BNEQ */
            case 0x0D: take = (sb == 0); break;     /* BEQ */
            case 0x0E: take = (sb > 0); break;      /* BGT */
            case 0x0F: take = (sb <= 0); break;     /* BLEQ */
            case 0x20: take = !(s->st0 & ST0_C); break;  /* NC */
            case 0x21: take = (s->st0 & ST0_C) != 0; break; /* C */
            case 0x30: take = !(s->st0 & ST0_TC); break; /* NTC */
            case 0x31: take = (s->st0 & ST0_TC) != 0; break; /* TC */
            case 0x60: take = !(s->st0 & ST0_OVA); break; /* ANOV */
            case 0x61: take = (s->st0 & ST0_OVA) != 0; break; /* AOV */
            case 0x68: take = !(s->st0 & ST0_OVB); break; /* BNOV */
            case 0x69: take = (s->st0 & ST0_OVB) != 0; break; /* BOV */
            default: take = true; break;  /* compound cond — take conservatively */
            }
            if (take) { s->pc = op2; return 0; }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE5) {
            /* E5xx: MVMM mmr, mmr */
            int src = (op >> 4) & 0xF;
            int dst = op & 0xF;
            uint16_t val;
            if (src >= 0 && src <= 7) val = s->ar[src];
            else if (src == 8) val = s->sp;
            else val = data_read(s, src + 0x10);
            if (dst >= 0 && dst <= 7) s->ar[dst] = val;
            else if (dst == 8) s->sp = val;
            else data_write(s, dst + 0x10, val);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE4) {
            /* E4xx: BITF Smem, #lk (2-word) or BIT Smem, bit */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t val = data_read(s, addr);
            s->st0 = (val & op2) ? (s->st0 | ST0_TC) : (s->st0 & ~ST0_TC);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE7) {
            /* E7xx: ST #k, Smem or LD #k,16, dst (short immediate) */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE9) {
            /* E9xx: CC pmad, cond (conditional call, 2 words)
             * Condition codes per tic54x-dis.c cc2[] table (SPRU172C Table 3-3):
             * 0x00=UNC, 0x02=AGEQ, 0x03=ALT, 0x04=ANEQ, 0x05=AEQ,
             * 0x06=AGT, 0x07=ALEQ, 0x0A=BGEQ, 0x0B=BLT, 0x0C=BNEQ,
             * 0x0D=BEQ, 0x0E=BGT, 0x0F=BLEQ,
             * 0x20=NC, 0x21=C, 0x30=NTC, 0x31=TC,
             * 0x40=NBIO, 0x60=ANOV, 0x61=AOV, 0x68=BNOV, 0x69=BOV */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            int64_t sa = sext40(s->a), sb = sext40(s->b);
            bool take = false;
            switch (cond) {
            case 0x00: take = true; break;          /* UNC */
            case 0x02: take = (sa >= 0); break;     /* AGEQ */
            case 0x03: take = (sa < 0); break;      /* ALT */
            case 0x04: take = (sa != 0); break;     /* ANEQ */
            case 0x05: take = (sa == 0); break;     /* AEQ */
            case 0x06: take = (sa > 0); break;      /* AGT */
            case 0x07: take = (sa <= 0); break;     /* ALEQ */
            case 0x0A: take = (sb >= 0); break;     /* BGEQ */
            case 0x0B: take = (sb < 0); break;      /* BLT */
            case 0x0C: take = (sb != 0); break;     /* BNEQ */
            case 0x0D: take = (sb == 0); break;     /* BEQ */
            case 0x0E: take = (sb > 0); break;      /* BGT */
            case 0x0F: take = (sb <= 0); break;     /* BLEQ */
            case 0x20: take = !(s->st0 & ST0_C); break;  /* NC */
            case 0x21: take = (s->st0 & ST0_C) != 0; break; /* C */
            case 0x30: take = !(s->st0 & ST0_TC); break; /* NTC */
            case 0x31: take = (s->st0 & ST0_TC) != 0; break; /* TC */
            case 0x60: take = !(s->st0 & ST0_OVA); break; /* ANOV */
            case 0x61: take = (s->st0 & ST0_OVA) != 0; break; /* AOV */
            case 0x68: take = !(s->st0 & ST0_OVB); break; /* BNOV */
            case 0x69: take = (s->st0 & ST0_OVB) != 0; break; /* BOV */
            default: take = true; break;  /* unknown compound cond — take conservatively */
            }
            if (take) {
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE1) {
            /* E1xx: single-word acc ops — NEG, ABS, CMPL, SAT, EXP, etc. */
            uint8_t sub = op & 0xFF;
            switch (sub) {
            case 0xE0: s->a = ~s->a; s->a = sext40(s->a); break;  /* CMPL A */
            case 0xE1: s->b = ~s->b; s->b = sext40(s->b); break;  /* CMPL B */
            case 0xE2: s->a = -s->a; s->a = sext40(s->a); break;  /* NEG A */
            case 0xE3: s->b = -s->b; s->b = sext40(s->b); break;  /* NEG B */
            case 0xE4: /* SAT A */ if (s->st0 & ST0_OVA) s->a = (s->a < 0) ? (int64_t)0xFF80000000LL : 0x7FFFFFFFLL; break;
            case 0xE5: /* SAT B */ if (s->st0 & ST0_OVB) s->b = (s->b < 0) ? (int64_t)0xFF80000000LL : 0x7FFFFFFFLL; break;
            case 0xE8: /* ABS A */ s->a = (s->a < 0) ? -s->a : s->a; s->a = sext40(s->a); break;
            case 0xE9: /* ABS B */ s->b = (s->b < 0) ? -s->b : s->b; s->b = sext40(s->b); break;
            case 0xEA: /* ROR A */ { uint16_t c = s->st0 & ST0_C ? 1 : 0; if (s->a & 1) s->st0 |= ST0_C; else s->st0 &= ~ST0_C; s->a = (s->a >> 1) | ((int64_t)c << 39); s->a = sext40(s->a); } break;
            case 0xEB: /* ROL A */ { uint16_t c = s->st0 & ST0_C ? 1 : 0; if (s->a & ((int64_t)1<<39)) s->st0 |= ST0_C; else s->st0 &= ~ST0_C; s->a = (s->a << 1) | c; s->a = sext40(s->a); } break;
            default:
                /* EXP A/B etc — return 0 for now */
                break;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEF) {
            /* EFxx: RPTZ dst, #lk — Zero accumulator and repeat (2 words)
             * Per SPRU172C: dst = 0; RPT #lk
             * Encoding: 1110 1111 xxxx xxxx + lk_word */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int rptz_dst = (op >> 0) & 1;
            if (rptz_dst) s->b = 0; else s->a = 0;
            s->rpt_count = op2;
            s->rpt_active = true;
            s->pc += 2;
            return 0;
        }
        if (hi8 == 0xEB) {
            /* EBxx: RPTB[D] pmad — Block repeat (2 words)
             * Per SPRU172C: REA = pmad, RSA = PC+2, BRAF=1 */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE6) {
            /* E6xx: SFTA/SFTL acc, #shift (single-word immediate shift) */
            int shift = op & 0x1F;
            if (shift & 0x10) shift |= ~0x1F;  /* sign extend 5-bit */
            int dst = (op >> 5) & 1;
            int logical = (op >> 6) & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            if (logical) {
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            } else {
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEE) {
            /* EExx: BCD pmad, cond (conditional delayed branch, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x03) take = (s->a == 0);
            else if (cond == 0x0B) take = (s->b == 0);
            else if (cond == 0x02) take = (s->a != 0);
            else if (cond == 0x0A) take = (s->b != 0);
            else if (cond == 0x00) take = true;  /* UNC */
            else if (cond == 0x08) take = (s->b < 0);
            else if (cond == 0x04) take = (s->a > 0);
            else if (cond == 0x0C) take = (s->b > 0);
            else if (cond == 0x40) take = (s->st0 & ST0_TC) != 0;
            else if (cond == 0x41) take = !(s->st0 & ST0_TC);
            else if (cond == 0x20) take = (s->st0 & ST0_C) != 0;
            else if (cond == 0x21) take = !(s->st0 & ST0_C);
            else if ((cond & 0x3A) == 0x3A) take = true; /* unconditional-ish */
            else take = true;
            if (take) { s->pc = op2; return 0; }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xED) {
            /* EDxx: BCD pmad, cond (conditional branch delayed, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x00) take = true;            /* UNC */
            else if (cond == 0x08) take = (s->b < 0);
            else if (cond == 0x02) take = (s->a != 0);
            else if (cond == 0x0A) take = (s->b != 0);
            else if (cond == 0x03) take = (s->a == 0);
            else if (cond == 0x0B) take = (s->b == 0);
            else if (cond == 0x04) take = (s->a > 0);
            else if (cond == 0x0C) take = (s->b > 0);
            else if (cond == 0x40) take = (s->st0 & ST0_TC) != 0;
            else if (cond == 0x41) take = !(s->st0 & ST0_TC);
            else take = true;
            if (take) { s->pc = op2; return 0; }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE8) {
            /* E8xx: CMPR cond, ARn */
            int cmp_cond = (op >> 4) & 3;
            int n = arp(s);
            bool result = false;
            switch (cmp_cond) {
            case 0: result = (s->ar[n] == s->ar[0]); break;
            case 1: result = (s->ar[n] < s->ar[0]); break;
            case 2: result = (s->ar[n] > s->ar[0]); break;
            case 3: result = (s->ar[n] != s->ar[0]); break;
            }
            if (result) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x6: case 0x7:
        /* 6Dxx: MAR Smem — modify address register (side effects only) */
        if (hi8 == 0x6D) {
            addr = resolve_smem(s, op, &ind);
            /* MAR only modifies AR via addressing mode, no data access */
            return consumed + s->lk_used;
        }
        /* 76xx: LDM MMR, dst */
        if (hi8 == 0x76) {
            uint8_t mmr = op & 0x7F;
            uint16_t val = data_read(s, mmr);
            s->a = (int64_t)(int16_t)val << 16;
            return consumed + s->lk_used;
        }
        /* 77xx: STM #lk, MMR (2 words) */
        if (hi8 == 0x77) {
            uint8_t mmr = op & 0x7F;
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, mmr, op2);
            return consumed + s->lk_used;
        }
        /* LD / ST operations */
        if ((op & 0xF800) == 0x7000) {
            /* 70xx: STL src, Smem */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x7800) {
            /* 78xx-7Fxx: STH src, Smem
             * Note: BANZ (0x78xx per doc) shares this range but is handled
             * via F84x (BANZ with condition) in the F8xx group. */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6000) {
            /* 60xx: LD Smem, dst */
            int dst_acc = (op >> 9) & 1;
            int shift = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)val : val;
            if (shift) v <<= 16;  /* LD Smem, 16, dst */
            if (dst_acc) s->b = sext40(v); else s->a = sext40(v);
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6800) {
            /* 68xx: LD Smem, T */
            addr = resolve_smem(s, op, &ind);
            s->t = data_read(s, addr);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x1:
        /* 1xxx: SUB variants */
        addr = resolve_smem(s, op, &ind);
        {
            int dst = (op >> 8) & 1;
            uint16_t val = data_read(s, addr);
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)val : val;
            v <<= 16;
            if (dst) s->b = sext40(s->b - v);
            else     s->a = sext40(s->a - v);
        }
        return consumed + s->lk_used;

    case 0x0:
        /* 0xxx: ADD variants */
        addr = resolve_smem(s, op, &ind);
        {
            int dst = (op >> 8) & 1;
            uint16_t val = data_read(s, addr);
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)val : val;
            v <<= 16;
            if (dst) s->b = sext40(s->b + v);
            else     s->a = sext40(s->a + v);
        }
        return consumed + s->lk_used;

    case 0x3:
        /* 3xxx: MAC / MAS */
        addr = resolve_smem(s, op, &ind);
        {
            int dst = (op >> 8) & 1;
            uint16_t val = data_read(s, addr);
            int64_t product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
            if (s->st1 & ST1_FRCT) product <<= 1;
            if (dst) s->b = sext40(s->b + product);
            else     s->a = sext40(s->a + product);
        }
        return consumed + s->lk_used;

    case 0x2:
        /* 2xxx: MPY, SQUR, MAS, MAC variants */
        {
            int sub = (op >> 8) & 0xF;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t product;
            int dst;
            switch (sub) {
            case 0x0: case 0x1: /* MPY Smem, A/B */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            case 0x4: case 0x5: /* SQUR Smem, A/B */
                product = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                s->t = val;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            case 0x8: case 0x9: /* MPYA Smem (A = T * Smem, B += A) or variants */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) { s->a += s->b; s->b = sext40(product); }
                else         { s->b += s->a; s->a = sext40(product); }
                return consumed + s->lk_used;
            case 0xA: case 0xB: /* MACA[R] Smem, A/B (A += B * Smem then B = T * Smem) */
                dst = sub & 1;
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (dst) { s->a = sext40(s->a + s->b); s->b = sext40(product); }
                else     { s->b = sext40(s->b + s->a); s->a = sext40(product); }
                s->t = val;
                return consumed + s->lk_used;
            default:
                /* MAS variants and others */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                dst = sub & 1;
                if (dst) s->b = sext40(s->b - product);
                else     s->a = sext40(s->a - product);
                return consumed + s->lk_used;
            }
        }

    case 0x4:
        /* 4xxx: AND, OR, XOR */
        addr = resolve_smem(s, op, &ind);
        {
            int sub = (op >> 8) & 0xF;
            uint16_t val = data_read(s, addr);
            switch (sub & 0x3) {
            case 0: /* AND */
                if (sub & 4) s->b = (s->b & 0xFFFF0000) | (((uint16_t)s->b) & val);
                else         s->a = (s->a & 0xFFFF0000) | (((uint16_t)s->a) & val);
                break;
            case 1: /* OR */
                if (sub & 4) s->b |= val;
                else         s->a |= val;
                break;
            case 2: /* XOR */
                if (sub & 4) s->b ^= val;
                else         s->a ^= val;
                break;
            }
        }
        return consumed + s->lk_used;

    case 0x5:
        /* 56xx: MVPD pmad, Smem — Move Program to Data (2 words) */
        if (hi8 == 0x56 || hi8 == 0x57) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t src = s->rpt_active ? s->mvpd_src : op2;
            uint16_t pval = prog_fetch(s, src);
            data_write(s, addr, pval);
            s->mvpd_src = src + 1;
            return consumed + s->lk_used;
        }
        /* 5xxx: shifts — SFTA, SFTL, various forms */
        {
            int dst = (op >> 8) & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            int sub = (op >> 9) & 0x7;
            if (sub <= 1) {
                /* 50xx/51xx: SFTA src, ASM shift */
                int shift = asm_shift(s);
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            } else if (sub == 2 || sub == 3) {
                /* 54xx/55xx: SFTA src, #shift (immediate in Smem) */
                addr = resolve_smem(s, op, &ind);
                int shift = (int16_t)data_read(s, addr);
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            } else if (sub == 4 || sub == 5) {
                /* 58xx/59xx: SFTL src, ASM shift (logical) */
                int shift = asm_shift(s);
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            } else if (sub == 6 || sub == 7) {
                /* 5Cxx/5Dxx/5Exx/5Fxx: SFTL with Smem or other */
                addr = resolve_smem(s, op, &ind);
                int shift = (int16_t)data_read(s, addr);
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            }
        }
        return consumed + s->lk_used;

    case 0x8: case 0x9:
        /* 8xxx/9xxx: Memory moves, PORTR/PORTW */

        /* ---- Dual-operand MAC Xmem, Ymem, dst (1-word) ----
         * 0x90: MAC Xmem,Ymem,A   0x92: MAC Xmem,Ymem,B
         * 0x91: MACR Xmem,Ymem,A  0x93: MACR Xmem,Ymem,B
         * Same encoding as 0xA4 family: OOOO OOOD XXXX YYYY */
        if (hi8 == 0x90 || hi8 == 0x91 || hi8 == 0x92 || hi8 == 0x93) {
            int xar_m = (op >> 4) & 0x07;
            int yar_m = op & 0x07;
            uint16_t xval_m = data_read(s, s->ar[xar_m]);
            uint16_t yval_m = data_read(s, s->ar[yar_m]);
            if ((op >> 7) & 1) s->ar[xar_m]--; else s->ar[xar_m]++;
            if ((op & 0x08) == 0) s->ar[yar_m]++; else s->ar[yar_m]--;
            int64_t prod_m = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_m;
            if (s->st1 & ST1_FRCT) prod_m <<= 1;
            if (hi8 & 0x01) prod_m += 0x8000; /* round */
            int dst_m = (hi8 & 0x02) ? 1 : 0;
            if (dst_m) s->b = sext40(s->b + prod_m);
            else       s->a = sext40(s->a + prod_m);
            s->t = yval_m;
            return consumed + s->lk_used;
        }

        /* 94xx: MVDK Smem, dmad — Move data(Smem) to data(dmad) (2 words) */
        if (hi8 == 0x94) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 95xx: MVKD dmad, Smem — Move data(dmad) to data(Smem) (2 words) */
        if (hi8 == 0x95) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 96xx: MVDP Smem, pmad — Move data to program (2 words) */
        if (hi8 == 0x96) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->prog[op2] = data_read(s, addr);
            return consumed + s->lk_used;
        }

        /* 0x98/0x99: STH src, Smem — store high accumulator (no shift)
         * 0x9A/0x9B: STL src, Smem — store low accumulator (no shift)
         * Per SPRU172C: 1001 100S = STH, 1001 101S = STL */
        if (hi8 == 0x98 || hi8 == 0x99) {
            addr = resolve_smem(s, op, &ind);
            int src_sth = hi8 & 1;
            int64_t acc = src_sth ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9A || hi8 == 0x9B) {
            addr = resolve_smem(s, op, &ind);
            int src_stl = hi8 & 1;
            int64_t acc = src_stl ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }

        /* 0x9C-0x9F range: SACCD/SRCCD/STRCD — conditional stores */

        /* SACCD src, Xmem, cond — Conditional accumulator store
         * Encoding: 1001 11SD XXXX COND per SPRU172C p.4-152 */
        if ((op & 0xFC00) == 0x9C00) {
            int src_s = (op >> 9) & 1;
            int64_t acc = src_s ? s->b : s->a;
            int xar_s = (op >> 4) & 0x07;
            uint16_t xaddr = s->ar[xar_s];
            int cond = op & 0x0F;
            /* Evaluate condition */
            int take = 0;
            switch (cond) {
            case 0x0: take = (acc == 0); break;    /* EQ */
            case 0x1: take = (acc != 0); break;    /* NEQ */
            case 0x2: take = (acc > 0); break;     /* GT */
            case 0x3: take = (acc < 0); break;     /* LT */
            case 0x4: take = (acc >= 0); break;    /* GEQ */
            case 0x5: take = (acc == 0); break;    /* AEQ */
            case 0x6: take = (acc > 0); break;     /* AGT */
            case 0x7: take = (acc <= 0); break;    /* LEQ/ALEQ */
            default: take = 0; break;
            }
            int asm_val = asm_shift(s);
            if (take) {
                /* Store shifted accumulator high part */
                int64_t shifted = acc << (asm_val > 0 ? asm_val : 0);
                if (asm_val < 0) shifted = acc >> (-asm_val);
                uint16_t val = (uint16_t)((shifted >> 16) & 0xFFFF);
                data_write(s, xaddr, val);
            } else {
                /* Read and write back (no change) */
                uint16_t val = data_read(s, xaddr);
                data_write(s, xaddr, val);
            }
            /* Xmem post-modify */
            if ((op >> 7) & 1) s->ar[xar_s]--; else s->ar[xar_s]++;
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8A) {
            /* MVDK Smem, dmad */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9A) {
            /* MVKD dmad, Smem */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x88 || hi8 == 0x80) {
            /* MVDD Smem, Smem (data→data) — 2 address forms */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8C) {
            /* MVPD pmad, Smem (prog→data) */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t mvpd_val = prog_read(s, op2);
            data_write(s, addr, mvpd_val);
            { static int mvpd_log = 0; if (mvpd_log++ < 20)
                C54_LOG("MVPD: prog[0x%04x]=0x%04x → data[0x%04x] PC=0x%04x insn=%u",
                        op2, mvpd_val, addr, s->pc, s->insn_count); }
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8E) {
            /* MVDP Smem, pmad (data→prog) */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            prog_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8F) {
            /* PORTR PA, Smem — read I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* PA=0xF430: BSP data register — return next burst sample */
            if (op2 == 0xF430 && s->bsp_pos < s->bsp_len) {
                data_write(s, addr, s->bsp_buf[s->bsp_pos++]);
            } else {
                data_write(s, addr, 0);
            }
            /* Log PORTR calls */
            {
                static int portr_log = 0;
                if (portr_log < 50) {
                    C54_LOG("PORTR PA=0x%04x → [0x%04x] val=0x%04x PC=0x%04x",
                            op2, addr, data_read(s, addr), s->pc);
                    portr_log++;
                }
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9F) {
            /* PORTW Smem, PA — write I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* Log I/O port writes */
            {
                uint16_t wval = data_read(s, addr);
                static int portw_log = 0;
                if (portw_log < 30) {
                    C54_LOG("PORTW PA=0x%04x val=0x%04x PC=0x%04x", op2, wval, s->pc);
                    portw_log++;
                }
            }
            return consumed + s->lk_used;
        }
        /* 85xx: MVPD pmad, Smem (prog→data, different encoding) */
        if (hi8 == 0x85) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, prog_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 86xx: MVDM dmad, MMR */
        if (hi8 == 0x86) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t mmr = op & 0x1F;
            data_write(s, mmr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 87xx: MVMD MMR, dmad */
        if (hi8 == 0x87) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t mmr = op & 0x1F;
            data_write(s, op2, data_read(s, mmr));
            return consumed + s->lk_used;
        }
        /* 81xx: STL src, ASM, Smem (store with shift) */
        if (hi8 == 0x81) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int64_t v = s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)(v & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 82xx: STH src, ASM, Smem */
        if (hi8 == 0x82) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int64_t v = s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)((v >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 89xx: ST src, Smem with shift or MVDK variants */
        if (hi8 == 0x89) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 8Bxx: MVDK with long address */
        if (hi8 == 0x8B) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 8Dxx: MVDD Smem, Smem */
        if (hi8 == 0x8D) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 83xx: WRITA Smem (write A to prog), 84xx: READA Smem */
        if (hi8 == 0x83) {
            addr = resolve_smem(s, op, &ind);
            prog_write(s, (uint16_t)(s->a & 0xFFFF), data_read(s, addr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x84) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, prog_read(s, (uint16_t)(s->a & 0xFFFF)));
            return consumed + s->lk_used;
        }
        /* 91xx: MVKD dmad, Smem (another encoding) */
        if (hi8 == 0x91) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 95xx: ST #lk, Smem (another encoding) */
        if (hi8 == 0x95) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        /* ST #lk, Smem (2-word) */
        if (hi8 == 0x96 || hi8 == 0x97) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xA: case 0xB:
        /* Axx/Bxx: STLM, LDMM, misc accumulator ops */

        /* ---- Dual-operand MAC/MAS Xmem, Ymem, dst (1-word) ----
         * MAC:  dst += T * Xmem; T = Ymem
         * MACR: dst += rnd(T * Xmem); T = Ymem
         * MAS:  dst -= T * Xmem; T = Ymem
         * MASR: dst -= rnd(T * Xmem); T = Ymem
         * Encoding: OOOO OOOD XXXX YYYY (1 word)
         *   Xmem: AR[ARP], post-mod by bit4 (0=inc,1=dec)
         *   Ymem: AR[bits2:0], post-mod by bit3 (0=inc,1=dec)
         *   D: 0=A, 1=B
         * hi8 mapping per SPRU172C:
         *   0xA4/0xA5: MAC[R] Xmem,Ymem,A   0xA6/0xA7: MAC[R] Xmem,Ymem,B
         *   0xB4/0xB5: MAS[R] Xmem,Ymem,A   0xB6/0xB7: MAS[R] Xmem,Ymem,B
         *   0xB0/0xB1: MAC[R] Xmem,Ymem,A (alt)  0xB2/0xB3 already handled
         */
        if (hi8 == 0xA4 || hi8 == 0xA5 || hi8 == 0xA6 || hi8 == 0xA7 ||
            hi8 == 0xB4 || hi8 == 0xB5 || hi8 == 0xB6 || hi8 == 0xB7 ||
            hi8 == 0xB0 || hi8 == 0xB1 || hi8 == 0xB2) {
            int xar_d = (op >> 4) & 0x07;
            int yar_d = op & 0x07;
            uint16_t xval_d = data_read(s, s->ar[xar_d]);
            uint16_t yval_d = data_read(s, s->ar[yar_d]);
            /* Post-modify */
            if ((op >> 7) & 1) s->ar[xar_d]--; else s->ar[xar_d]++;
            if ((op & 0x08) == 0) s->ar[yar_d]++; else s->ar[yar_d]--;
            /* Multiply T * Xmem */
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_d;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            /* Round if R bit set (odd hi8) */
            if (hi8 & 0x01) prod += 0x8000;
            /* Determine dest and operation */
            int is_sub = (hi8 >= 0xB4 && hi8 <= 0xB7);
            int dst_b;
            if (hi8 >= 0xA4 && hi8 <= 0xA7) dst_b = (hi8 >= 0xA6);
            else if (hi8 >= 0xB4 && hi8 <= 0xB7) dst_b = (hi8 >= 0xB6);
            else dst_b = (hi8 & 0x02) ? 1 : 0; /* 0xB0/B1→A, 0xB2/B3→B */
            if (dst_b) {
                if (is_sub) s->b = sext40(s->b - prod);
                else        s->b = sext40(s->b + prod);
            } else {
                if (is_sub) s->a = sext40(s->a - prod);
                else        s->a = sext40(s->a + prod);
            }
            /* T = Ymem */
            s->t = yval_d;
            return consumed + s->lk_used;
        }

        /* SQDST Xmem, Ymem — Squared Distance (1-word dual-operand)
         * Encoding: 1010 0001 XXXX YYYY
         * Per SPRU172C: B += (AH - Xmem)^2; A = Ymem << 16; T = Xmem */
        if (hi8 == 0xA1) {
            int xar_sq = (op >> 4) & 0x07;
            int yar_sq = op & 0x07;
            uint16_t xval_sq = data_read(s, s->ar[xar_sq]);
            uint16_t yval_sq = data_read(s, s->ar[yar_sq]);
            if ((op >> 7) & 1) s->ar[xar_sq]--; else s->ar[xar_sq]++;
            if ((op & 0x08) == 0) s->ar[yar_sq]++; else s->ar[yar_sq]--;
            int16_t ah_sq = (int16_t)((s->a >> 16) & 0xFFFF);
            int32_t diff = (int32_t)ah_sq - (int32_t)(int16_t)xval_sq;
            int64_t sq = (int64_t)diff * (int64_t)diff;
            if (s->st1 & ST1_FRCT) sq <<= 1;
            s->b = sext40(s->b + sq);
            s->a = sext40((int64_t)(int16_t)yval_sq << 16);
            s->t = xval_sq;
            return consumed + s->lk_used;
        }

        /* POLY Xmem, Ymem — Polynomial evaluation (1-word dual-operand)
         * Encoding: 1011 110D XXXX YYYY (0xBC=A, 0xBD=B)
         *           1011 111D XXXX YYYY (0xBE/0xBF variants — ABDST or POLY)
         * Per SPRU172C: B += AH * T (with round); A = Xmem << 16; T = Ymem */
        if (hi8 == 0xBC || hi8 == 0xBD || hi8 == 0xBE || hi8 == 0xBF) {
            int xar_p = (op >> 4) & 0x07;
            int yar_p = op & 0x07;
            uint16_t xval_p = data_read(s, s->ar[xar_p]);
            uint16_t yval_p = data_read(s, s->ar[yar_p]);
            if ((op >> 7) & 1) s->ar[xar_p]--; else s->ar[xar_p]++;
            if ((op & 0x08) == 0) s->ar[yar_p]++; else s->ar[yar_p]--;
            int16_t ah_p = (int16_t)((s->a >> 16) & 0xFFFF);
            int64_t prod_p = (int64_t)ah_p * (int64_t)(int16_t)s->t;
            if (s->st1 & ST1_FRCT) prod_p <<= 1;
            prod_p += 0x8000; /* round */
            s->b = sext40(s->b + prod_p);
            s->a = sext40((int64_t)(int16_t)xval_p << 16);
            s->t = yval_p;
            return consumed + s->lk_used;
        }

        /* B8-BB: MAS/MASR Xmem, Ymem (subtract variants) or POLY-like */
        if (hi8 == 0xB8 || hi8 == 0xB9 || hi8 == 0xBA || hi8 == 0xBB) {
            /* Check if it's actually LDMM (BA) or POPM (BD) — those are handled below */
            if (hi8 == 0xBA) goto ba_handler;
            int xar_b8 = (op >> 4) & 0x07;
            int yar_b8 = op & 0x07;
            uint16_t xval_b8 = data_read(s, s->ar[xar_b8]);
            uint16_t yval_b8 = data_read(s, s->ar[yar_b8]);
            if ((op >> 7) & 1) s->ar[xar_b8]--; else s->ar[xar_b8]++;
            if ((op & 0x08) == 0) s->ar[yar_b8]++; else s->ar[yar_b8]--;
            int64_t prod_b8 = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_b8;
            if (s->st1 & ST1_FRCT) prod_b8 <<= 1;
            if (hi8 & 0x01) prod_b8 += 0x8000;
            int dst_b8 = (hi8 & 0x02) ? 1 : 0;
            /* MAS: subtract */
            if (dst_b8) s->b = sext40(s->b - prod_b8);
            else        s->a = sext40(s->a - prod_b8);
            s->t = yval_b8;
            return consumed + s->lk_used;
        }
ba_handler:
        if (hi8 == 0xAA || hi8 == 0xAB) {
            /* STLM src, MMR — AA=A, AB=B */
            int src_acc = (hi8 == 0xAB) ? 1 : 0;
            uint16_t mmr = op & 0x1F;
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, mmr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xBD) {
            /* BDxx: POPM / delayed branch variants */
            uint16_t mmr = op & 0x1F;
            data_write(s, mmr, data_read(s, s->sp));
            s->sp++;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xBA) {
            /* LDMM MMR, dst */
            uint16_t mmr = op & 0x1F;
            int dst = (op >> 4) & 1;
            int64_t v = (int64_t)(int16_t)data_read(s, mmr);
            if (dst) s->b = sext40(v << 16);
            else     s->a = sext40(v << 16);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xA8 || hi8 == 0xA9) {
            /* A8xx/A9xx: AND #lk, src[, dst] (2-word) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            *acc = sext40(*acc & ((int64_t)op2 << 16));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xA0) {
            /* A0xx: LD src, dst (accumulator to accumulator) or NEG/ABS/NOT */
            uint8_t sub = op & 0xFF;
            if (sub == 0x00) { s->a = s->b; }
            else if (sub == 0x01) { s->b = s->a; }
            else if (sub == 0x08) { s->a = -s->a; } /* NEG A */
            else if (sub == 0x09) { s->b = -s->b; } /* NEG B */
            else if (sub == 0x0A) { s->a = (s->a < 0) ? -s->a : s->a; } /* ABS A */
            else if (sub == 0x0B) { s->b = (s->b < 0) ? -s->b : s->b; } /* ABS B */
            return consumed + s->lk_used;
        }
        if (hi8 == 0xA5) {
            /* CMPS src, Smem — compare and select (Viterbi) */
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int src = (op >> 4) & 1;
            int64_t acc = src ? s->b : s->a;
            int64_t cmp = (int64_t)(int16_t)val << 16;
            /* TRN shift left, TC set based on comparison */
            s->trn <<= 1;
            if (acc >= cmp) {
                s->st0 |= ST0_TC;
                s->trn |= 1;
            } else {
                s->st0 &= ~ST0_TC;
                if (src) s->b = cmp; else s->a = cmp;
            }
            return consumed + s->lk_used;
        }
        /* AExx/AFxx: MACD Smem, pmad, dst — MAC + data move (2 words)
         * dst += T * Smem, then data(Smem) → data(dmad)
         * pmad in second word auto-increments during RPT */
        if (hi8 == 0xAE || hi8 == 0xAF) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t sval = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)sval;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int dst = (hi8 & 0x01);
            if (dst) s->b = sext40(s->b + prod);
            else     s->a = sext40(s->a + prod);
            /* Data move: read from prog[pmad], write to data[addr] */
            uint16_t psrc = s->rpt_active ? s->mvpd_src : op2;
            data_write(s, addr, prog_fetch(s, psrc));
            s->mvpd_src = psrc + 1;
            s->t = sval;  /* T = old Smem value (before overwrite) */
            return consumed + s->lk_used;
        }
        /* ACxx/ADxx: MACP Smem, pmad, dst — MAC + program fetch (2 words) */
        if (hi8 == 0xAC || hi8 == 0xAD) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t sval = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)sval;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int dst = (hi8 & 0x01);
            if (dst) s->b = sext40(s->b + prod);
            else     s->a = sext40(s->a + prod);
            /* Coeff fetch from program memory */
            uint16_t psrc = s->rpt_active ? s->mvpd_src : op2;
            s->t = prog_fetch(s, psrc);
            s->mvpd_src = psrc + 1;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xB3) {
            /* LD #lk, dst (long immediate, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = (op >> 0) & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(v << 16);
            else     s->a = sext40(v << 16);
            return consumed + s->lk_used;
        }
        /* ADD #lk, src[, dst] */
        if (hi8 == 0xA2) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(s->b + (v << 16));
            else     s->a = sext40(s->a + (v << 16));
            return consumed + s->lk_used;
        }
        /* SUB #lk */
        if (hi8 == 0xA3) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(s->b - (v << 16));
            else     s->a = sext40(s->a - (v << 16));
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xC: case 0xD:
        /* C/Dxxx: PSHM, POPM, PSHD, POPD, RPT, FRAME, etc. */

        /* ---- Dual-operand MAC/MAS Xmem, Ymem, dst (1-word) ----
         * 0xD0: MAC Xmem,Ymem,A   0xD2: MAC Xmem,Ymem,B
         * 0xD1: MACR Xmem,Ymem,A  0xD3: MACR Xmem,Ymem,B
         * 0xD4-0xD7: MAS variants (subtract) */
        if (hi8 >= 0xD0 && hi8 <= 0xD9 && hi8 != 0xDA) {
            int xar_c = (op >> 4) & 0x07;
            int yar_c = op & 0x07;
            uint16_t xval_c = data_read(s, s->ar[xar_c]);
            uint16_t yval_c = data_read(s, s->ar[yar_c]);
            if ((op >> 7) & 1) s->ar[xar_c]--; else s->ar[xar_c]++;
            if ((op & 0x08) == 0) s->ar[yar_c]++; else s->ar[yar_c]--;
            int64_t prod_c = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_c;
            if (s->st1 & ST1_FRCT) prod_c <<= 1;
            if (hi8 & 0x01) prod_c += 0x8000; /* round */
            int is_sub_c = (hi8 >= 0xD4);
            int dst_c = (hi8 & 0x02) ? 1 : 0;
            if (dst_c) {
                if (is_sub_c) s->b = sext40(s->b - prod_c);
                else          s->b = sext40(s->b + prod_c);
            } else {
                if (is_sub_c) s->a = sext40(s->a - prod_c);
                else          s->a = sext40(s->a + prod_c);
            }
            s->t = yval_c;
            return consumed + s->lk_used;
        }

        /* DBxx: MASA Xmem, Ymem, dst — MAC with accumulator sign extension
         * Per SPRU172C: same as MAC but T loaded from Xmem instead of Ymem.
         * dst += T * Xmem, T = Xmem */
        if (hi8 == 0xDB) {
            int xar_db = (op >> 4) & 0x07;
            int yar_db = op & 0x07;
            uint16_t xval_db = data_read(s, s->ar[xar_db]);
            (void)data_read(s, s->ar[yar_db]); /* Ymem read (unused) */
            if ((op >> 7) & 1) s->ar[xar_db]--; else s->ar[xar_db]++;
            if ((op & 0x08) == 0) s->ar[yar_db]++; else s->ar[yar_db]--;
            int64_t prod_db = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_db;
            if (s->st1 & ST1_FRCT) prod_db <<= 1;
            s->a = sext40(s->a + prod_db);
            s->t = xval_db;
            return consumed + s->lk_used;
        }

        /* DCxx: SQUR Xmem, dst — Square and accumulate (1-word dual-operand)
         * Per SPRU172C p.4-165: T = Xmem, dst = dst + T * T
         * Encoding: 1101 1100 XXXX YYYY (Ymem read but unused for SQUR) */
        if (hi8 == 0xDC) {
            int xar_dc = (op >> 4) & 0x07;
            int yar_dc = op & 0x07;
            uint16_t xval_dc = data_read(s, s->ar[xar_dc]);
            (void)data_read(s, s->ar[yar_dc]); /* Ymem pipeline read */
            if ((op >> 7) & 1) s->ar[xar_dc]--; else s->ar[xar_dc]++;
            if ((op & 0x08) == 0) s->ar[yar_dc]++; else s->ar[yar_dc]--;
            s->t = xval_dc;
            int64_t prod_dc = (int64_t)(int16_t)xval_dc * (int64_t)(int16_t)xval_dc;
            if (s->st1 & ST1_FRCT) prod_dc <<= 1;
            s->a = sext40(s->a + prod_dc);
            return consumed + s->lk_used;
        }

        /* CA/CB: ST || LD parallel (dual-operand, 1-word) — like C8/C9 */
        if (hi8 == 0xCA || hi8 == 0xCB) {
            int s_acc_ca = (hi8 == 0xCB) ? 1 : 0;
            int xar_ca = (op >> 4) & 0x07;
            int yar_ca = op & 0x07;
            int64_t st_val_ca = s_acc_ca ? s->b : s->a;
            data_write(s, s->ar[yar_ca], (uint16_t)(st_val_ca & 0xFFFF));
            uint16_t ld_val_ca = data_read(s, s->ar[xar_ca]);
            int d_acc_ca = s_acc_ca ? 0 : 1;
            int64_t loaded_ca = (int64_t)(int16_t)ld_val_ca << 16;
            if (d_acc_ca) s->b = sext40(loaded_ca); else s->a = sext40(loaded_ca);
            if ((op >> 7) & 1) s->ar[xar_ca]--; else s->ar[xar_ca]++;
            if ((op & 0x08) == 0) s->ar[yar_ca]++; else s->ar[yar_ca]--;
            return consumed + s->lk_used;
        }
        /* CF: variant parallel or DELAY */
        if (hi8 == 0xCF) {
            /* Treat as NOP for now — rare instruction */
            return consumed + s->lk_used;
        }
        /* RPTB[D] pmad — Block repeat (2 words)
         * C2xx: RPTB pmad, C3xx: RPTBD pmad (delayed)
         * Per SPRU172C: RSA = PC+2, REA = pmad, BRAF = 1 */
        if (hi8 == 0xC2 || hi8 == 0xC3 || hi8 == 0xC6 || hi8 == 0xC7) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC5) {
            /* PSHM MMR */
            uint16_t mmr = op & 0x1F;
            s->sp--;
            data_write(s, s->sp, data_read(s, mmr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xCD) {
            /* POPM MMR */
            uint16_t mmr = op & 0x1F;
            data_write(s, mmr, data_read(s, s->sp));
            s->sp++;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xCE) {
            /* FRAME #k (signed 8-bit) */
            int8_t k = (int8_t)(op & 0xFF);
            s->sp += k;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC4) {
            /* C4xx: PSHD dmad (push data from absolute addr) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, data_read(s, op2));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC0 || hi8 == 0xC1) {
            /* PSHD Smem / RPT Smem variants */
            addr = resolve_smem(s, op, &ind);
            if (hi8 == 0xC0) {
                /* PSHD Smem */
                s->sp--;
                data_write(s, s->sp, data_read(s, addr));
            } else {
                /* RPT Smem */
                s->rpt_count = data_read(s, addr);
                s->rpt_active = true;
                s->pc += consumed;
                return 0;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xCC) {
            /* CCxx: SACCD Smem, ARmem — Store Acc Conditionally (1-word)
             * Per SPRU172C: conditionally store AH or BH to Smem.
             * Simplified: always store (condition always true). */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->a >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDA) {
            /* DAxx: RPTBD pmad (block repeat delayed, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4); /* delayed: skip 2 delay slots */
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDD) {
            /* POPD Smem */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, data_read(s, s->sp));
            s->sp++;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDE) {
            /* DExx: POPD dmad (2-word) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, s->sp));
            s->sp++;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDF) {
            /* DELAY Smem — shift delay line: data(Smem) → data(Smem+1)
             * Per SPRU172C: used with RPT for FIR filter delay lines */
            addr = resolve_smem(s, op, &ind);
            uint16_t dval = data_read(s, addr);
            data_write(s, addr + 1, dval);
            return consumed + s->lk_used;
        }
        /* 0xC8/0xC9: ST || LD parallel instruction (1 word, dual indirect)
         * Store low acc to Ymem while loading Xmem to other acc */
        if (hi8 == 0xC8 || hi8 == 0xC9) {
            int s_acc = (hi8 == 0xC9) ? 1 : 0;
            int xar = (op >> 4) & 0x07;
            int yar = op & 0x07;
            int64_t st_val = s_acc ? s->b : s->a;
            data_write(s, s->ar[yar], (uint16_t)(st_val & 0xFFFF));
            uint16_t ld_val = data_read(s, s->ar[xar]);
            int d_acc = s_acc ? 0 : 1;
            int64_t loaded = (int64_t)(int16_t)ld_val << 16;
            if (d_acc) s->b = sext40(loaded); else s->a = sext40(loaded);
            if ((op >> 7) & 1) s->ar[xar]--; else s->ar[xar]++;
            if ((op & 0x08) == 0) s->ar[yar]++; else s->ar[yar]--;
            return consumed + s->lk_used;
        }
        goto unimpl;

    default:
        break;
    }

unimpl:
    s->unimpl_count++;
    if (s->unimpl_count <= 200 || op != s->last_unimpl) {
        C54_LOG("UNIMPL @0x%04x: 0x%04x (hi8=0x%02x) [#%u]",
                s->pc, op, hi8, s->unimpl_count);
        s->last_unimpl = op;
    }
    return consumed + s->lk_used;
}

/* ================================================================
 * Main execution loop
 * ================================================================ */

int c54x_run(C54xState *s, int n_insns)
{
    int executed = 0;

    /* Log first 10 instructions of each run (for 2nd cycle debug) */
    static int run_num = 0;
    run_num++;

    while (executed < n_insns && s->running && !s->idle) {
        /* Record PC in ring buffer */
        pc_ring[pc_ring_idx & 255] = s->pc;
        pc_ring_idx++;

        /* Track SP changes inside RPTB loops */
        uint16_t sp_before = s->sp;

        /* Trace DSP init - log once per unique PC in E900-E960 */
        if (s->pc >= 0xE900 && s->pc < 0xE960 && !s->rpt_active) {
            static uint16_t seen_pcs[96];
            int idx = s->pc - 0xE900;
            if (!seen_pcs[idx]) {
                seen_pcs[idx] = 1;
                C54_LOG("INIT PC=0x%04x op=0x%04x SP=0x%04x BRC=%d rptb=%d RSA=0x%04x REA=0x%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp, s->brc,
                        s->rptb_active, s->rsa, s->rea);
            }
        }

        /* Trace SINT17 handler (0x8a00-0x8a5f) */
        if (s->pc >= 0x8a00 && s->pc < 0x8a60) {
            static int sint17_log = 0;
            if (sint17_log < 500) {
                C54_LOG("SINT17 PC=0x%04x op=0x%04x SP=0x%04x DP=0x%03x A=0x%010llx B=0x%010llx AR0=%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp, dp(s),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->ar[0]);
                sint17_log++;
            }
        }

        /* Sample PC every 1M instructions to find stuck loops */
        if (executed > 0 && (executed % 1000000) == 0) {
            static int sample_log = 0;
            if (sample_log < 20)
                C54_LOG("@%dM: PC=0x%04x op=0x%04x SP=0x%04x insn=%u",
                        executed/1000000, s->pc, prog_read(s, s->pc), s->sp, s->insn_count);
            sample_log++;
        }
        if (run_num <= 2 && executed < 2000) {
            C54_LOG("BOOT[%d.%d] PC=0x%04x op=0x%04x SP=0x%04x A=0x%010llx B=0x%010llx",
                    run_num, executed, s->pc, prog_fetch(s, s->pc), s->sp,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                    (unsigned long long)(s->b & 0xFFFFFFFFFFLL));
        }
        /* Check RPTB (block repeat) — skip during RPT (single repeat has priority) */
        if (s->rptb_active && !s->rpt_active && s->pc == s->rea + 1) {
            static int rptb_log = 0;
            if (rptb_log < 20) { C54_LOG("RPTB redirect PC=0x%04x→RSA=0x%04x REA=0x%04x BRC=%d", s->pc, s->rsa, s->rea, s->brc); rptb_log++; }
            if (s->brc > 0) {
                s->brc--;
                s->pc = s->rsa;
            } else {
                s->rptb_active = false; C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x", s->pc, s->rsa, s->rea, s->insn_count, s->sp);
                s->st1 &= ~ST1_BRAF;
            }
        }

        /* Trace the IMR loop: how does the DSP reach 0x03F0? */
        /* Trace RPTB entry at 0x76FD: dump all AR values */
        if (s->pc == 0x76FD) {
            static int rptb_entry_log = 0;
            if (rptb_entry_log < 30)
                C54_LOG("RPTB-ENTRY PC=0x76FD AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x ARP=%d DP=%d BRC=%d SP=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        arp(s), dp(s), s->brc, s->sp);
            rptb_entry_log++;
        }
        if (s->pc == 0x03F0) {
            static int f3_log = 0;
            if (f3_log < 2) {
                C54_LOG("PC=0x03F0 op=0x%04x insn=%u SP=0x%04x IMR=0x%04x XPC=%d PMST=0x%04x",
                        prog_fetch(s, s->pc), s->insn_count, s->sp, s->imr, s->xpc, s->pmst);
                C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                        pc_ring[(pc_ring_idx-20)&255], pc_ring[(pc_ring_idx-19)&255],
                        pc_ring[(pc_ring_idx-18)&255], pc_ring[(pc_ring_idx-17)&255],
                        pc_ring[(pc_ring_idx-16)&255], pc_ring[(pc_ring_idx-15)&255],
                        pc_ring[(pc_ring_idx-14)&255], pc_ring[(pc_ring_idx-13)&255],
                        pc_ring[(pc_ring_idx-12)&255], pc_ring[(pc_ring_idx-11)&255],
                        pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                        pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                        pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                        pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                        pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                f3_log++;
            }
        }

        /* Boot trace */
        if (g_boot_trace > 0) {
            C54_LOG("BOOT[%d] PC=0x%04x op=0x%04x SP=0x%04x PMST=0x%04x",
                    51 - g_boot_trace, s->pc, prog_fetch(s, s->pc), s->sp, s->pmst);
            g_boot_trace--;
        }

        /* Execute instruction */
        int consumed;
        uint16_t exec_pc = s->pc;
        uint16_t exec_op = prog_fetch(s, s->pc);
        consumed = c54x_exec_one(s);

        /* Detect SP changes — only log after init (insn > 490M) */
        if (s->sp != sp_before && s->insn_count > 490000000) {
            static int sp_leak_log = 0;
            if (sp_leak_log < 100) {
                C54_LOG("SP %+d PC=0x%04x op=0x%04x SP 0x%04x→0x%04x insn=%u",
                        (int16_t)(s->sp - sp_before), exec_pc, exec_op, sp_before, s->sp, s->insn_count);
                sp_leak_log++;
            }
        }

        /* RPT: after executing an instruction while repeat is active,
         * re-execute the SAME instruction (don't advance PC) until count=0. */
        if (s->rpt_active && !s->idle) {
            if (s->rpt_count > 0) {
                s->rpt_count--;
                /* Don't advance PC — re-execute same instruction next cycle */
                s->cycles++;
                executed++;
                if (s->rpt_count == 0) {
                    static int rpt_done_log = 0;
                    if (rpt_done_log < 10)
                        C54_LOG("RPT DONE PC=0x%04x op=0x%04x count_was=%d", s->pc, prog_fetch(s, s->pc), 0);
                    rpt_done_log++;
                }
                continue;
            } else {
                s->rpt_active = false;
                s->par_set = false;
            }
        }

        if (consumed > 0)
            s->pc += consumed;
        s->pc &= 0xFFFF;  /* C54x has 16-bit PC (23-bit with XPC, but wrap at 16-bit) */
        /* consumed == 0 means PC was set by branch */

        s->cycles++;
        s->insn_count++;
        executed++;
    }
    return executed;
}

/* ================================================================
 * ROM loader
 * ================================================================ */

int c54x_load_rom(C54xState *s, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        C54_LOG("Cannot open ROM dump: %s", path);
        return -1;
    }

    char line[1024];
    int section = -1; /* 0=regs, 1=DROM, 2=PDROM, 3-6=PROM0-3 */
    
    int total_words = 0;

    while (fgets(line, sizeof(line), f)) {
        /* Section headers */
        if (strstr(line, "DSP dump: Registers"))  { section = 0; continue; }
        if (strstr(line, "DSP dump: DROM"))        { section = 1; continue; }
        if (strstr(line, "DSP dump: PDROM"))       { section = 2; continue; }
        if (strstr(line, "DSP dump: PROM0"))       { section = 3; continue; }
        if (strstr(line, "DSP dump: PROM1"))       { section = 4; continue; }
        if (strstr(line, "DSP dump: PROM2"))       { section = 5; continue; }
        if (strstr(line, "DSP dump: PROM3"))       { section = 6; continue; }
        if (section < 0) continue;

        /* Parse data lines: "ADDR : XXXX XXXX XXXX ..." */
        uint32_t addr;
        if (sscanf(line, "%x :", &addr) != 1) continue;

        char *p = strchr(line, ':');
        if (!p) continue;
        p++;

        uint16_t word;
        while (sscanf(p, " %hx%n", &word, (int[]){0}) == 1) {
            int n;
            sscanf(p, " %hx%n", &word, &n);
            p += n;

            if (section == 0) {
                /* Registers: store in data memory */
                if (addr < 0x60) s->data[addr] = word;
            } else if (section == 1 || section == 2) {
                /* DROM/PDROM: data memory */
                if (addr < C54X_DATA_SIZE) s->data[addr] = word;
            } else {
                /* PROM: program memory.
                 * The dump uses extended addresses (XPC pages):
                 *   PROM0: 0x07000-0x0DFFF → prog space 0x7000-0xDFFF
                 *   PROM1: 0x18000-0x1FFFF → prog space 0x8000-0xFFFF (page 1)
                 *   PROM2: 0x28000-0x2FFFF → prog space 0x8000-0xFFFF (page 2)
                 *   PROM3: 0x38000-0x39FFF → prog space 0xF800-0xFFFF (page 3)
                 * For 16-bit PC access, map all PROM to lower 64K too.
                 * PROM0 is already at 0x7000. For PROM1-3, also mirror
                 * to the 16-bit alias (0x8000-0xFFFF). */
                if (addr < C54X_PROG_SIZE) s->prog[addr] = word;
                /* Mirror PROM1 (page 1: 0x18000-0x1FFFF) to 16-bit space.
                 * PROM0 occupies 0x7000-0xDFFF — only mirror PROM1 above that
                 * (0xE000-0xFFFF) to avoid overwriting PROM0 data.
                 * This gives us interrupt vectors at 0xFF80. */
                if (section == 4) {  /* PROM1 only */
                    uint16_t addr16 = addr & 0xFFFF;
                    /* Mirror PROM1 to 0xE000-0xFF7F only.
                     * 0xFF80-0xFFFF is the interrupt vector table,
                     * populated by the DSP boot ROM (not PROM1). */
                    if (addr16 >= 0xE000)
                        s->prog[addr16] = word;
                }
            }
            addr++;
            total_words++;
        }
    }

    fclose(f);
    C54_LOG("Loaded ROM: %d words from %s", total_words, path);
    return 0;
}

/* ================================================================
 * Init / Reset / Interrupts
 * ================================================================ */

C54xState *c54x_init(void)
{
    C54xState *s = calloc(1, sizeof(C54xState));
    if (!s) return NULL;
    return s;
}

void c54x_set_api_ram(C54xState *s, uint16_t *api_ram)
{
    s->api_ram = api_ram;
}

void c54x_reset(C54xState *s)
{
    g_boot_trace = 50;
    s->a = 0; s->b = 0;
    memset(s->ar, 0, sizeof(s->ar));
    s->t = 0; s->trn = 0;
    s->sp = 0x5AC8; s->bk = 0;  /* SP init per Calypso boot ROM */
    s->brc = 0; s->rsa = 0; s->rea = 0;
    s->st0 = 0;
    s->st1 = ST1_INTM;  /* interrupts disabled at reset */
    s->pmst = 0xFFE0;   /* IPTR = 0x1FF, OVLY = 0 at reset */
    s->imr = 0;
    s->ifr = 0;
    s->xpc = 0;
    s->timer_psc = 0;
    s->data[TCR_ADDR] = TCR_TSS;  /* Timer stopped at reset (TSS=1) per HW spec */
    s->data[TIM_ADDR] = 0xFFFF;   /* TIM = max at reset */
    s->data[PRD_ADDR] = 0xFFFF;   /* PRD = max at reset */
    s->rpt_active = false;
    s->rptb_active = false; C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x", s->pc, s->rsa, s->rea, s->insn_count, s->sp);
    s->idle = false;
    s->running = true;
    s->cycles = 0;
    s->insn_count = 0;
    s->unimpl_count = 0;

    /* Boot ROM MVPD: copy PROM0 code to DARAM overlay.
     * On real Calypso, the internal boot ROM copies PROM0[0x7080..0x9FFF]
     * to DARAM data[0x0080..0x27FF] before jumping to user code.
     * This populates the DARAM code overlay that the DSP executes with OVLY=1. */
    for (int i = 0; i < 0x2780; i++)
        s->data[0x0080 + i] = s->prog[0x7080 + i];

    /* Install boot ROM interrupt vectors at 0xFF80 (IPTR=0x1FF).
     * These are from the Calypso internal boot ROM, not in the PROM dump.
     * Vec0 (reset): B 0xB410 (bootloader entry) */
    s->prog[0xFF80] = 0xF880;  /* B pmad */
    s->prog[0xFF81] = 0xB410;  /* target: bootloader */
    s->prog[0xFF82] = 0xF495;  /* NOP */
    s->prog[0xFF83] = 0xF495;  /* NOP */
    /* Vec1-7: use PROM1 ROM vectors (already mirrored to 0xFF84-0xFFFF).
     * Do NOT overwrite — the ROM contains the real interrupt handlers. */

    /* Boot ROM stubs at 0x0000-0x007F.
     * The internal Calypso boot ROM occupies prog 0x0000-0x007F but is not
     * in the PROM dump. The DSP init code does CALA with A(low)=0x0000 to
     * call boot ROM routines. The handler at 0x8A07 expects B(high) = SP
     * (restored via STH B,SP at 0x8A46 after RETE). Stub: LDM SP,B; RET */
    for (int i = 0; i < 0x80; i++)
        s->prog[i] = 0xFC00;  /* RET (per tic54x-opc.c) */
    /* Entry point 0x0000: save SP into B for the SINT17 handler */
    s->prog[0x0000] = 0xBA18;  /* LDMM SP, B — read MMR 0x18(SP) into B(high) */
    s->prog[0x0001] = 0xFC00;  /* RET (per tic54x-opc.c) */

    /* Reset vector: IPTR * 0x80 */
    uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
    s->pc = iptr * 0x80;  /* 0xFF80 for default PMST */

    C54_LOG("Reset: PC=0x%04x PMST=0x%04x SP=0x%04x prog[PC]=0x%04x",
            s->pc, s->pmst, s->sp, s->prog[s->pc]);
}

void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit)
{
    if (vec < 0 || vec >= 32) return;
    if (imr_bit < 0 || imr_bit >= 16) return;
    s->ifr |= (1 << imr_bit);

    bool unmasked = (s->imr & (1 << imr_bit)) != 0;

    /* Per SPRU131: IDLE exits on ANY interrupt (masked or unmasked).
     * - Unmasked: branch to vector, set INTM=1
     * - Masked: just resume after IDLE, IFR bit stays set */
    if (s->idle) {
        s->idle = false;
        if (unmasked) {
            /* Service the interrupt: branch to vector */
            s->ifr &= ~(1 << imr_bit);
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 1));
            if (s->pmst & PMST_APTS) {
                s->sp--;
                data_write(s, s->sp, s->xpc);
            }
            s->st1 |= ST1_INTM;
            uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            s->pc = (iptr * 0x80) + vec * 4;
        }
        /* If masked: just wake, advance PC past IDLE */
        if (!unmasked) {
            s->pc++;  /* resume at instruction after IDLE */
        }
    } else if (!(s->st1 & ST1_INTM) && unmasked) {
        /* Normal (non-IDLE) interrupt servicing */
        s->ifr &= ~(1 << imr_bit);
        s->sp--;
        data_write(s, s->sp, (uint16_t)s->pc);
        if (s->pmst & PMST_APTS) {
            s->sp--;
            data_write(s, s->sp, s->xpc);
        }
        s->st1 |= ST1_INTM;
        uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        s->pc = (iptr * 0x80) + vec * 4;
    }

    /* Log first few interrupts */
    static int int_log_count = 0;
    if (int_log_count < 5) {
        C54_LOG("IRQ vec=%d bit=%d: INTM=%d IMR=0x%04x IFR=0x%04x idle=%d PC=0x%04x",
                vec, imr_bit, !!(s->st1 & ST1_INTM), s->imr, s->ifr, s->idle, s->pc);
        int_log_count++;
    }
}

void c54x_wake(C54xState *s)
{
    s->idle = false;
}

void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n)
{
    if (n > 160) n = 160;
    memcpy(s->bsp_buf, samples, n * sizeof(uint16_t));
    s->bsp_len = n;
    s->bsp_pos = 0;
}

===== ./hw/arm/calypso/calypso_tint0.h =====
/*
 * calypso_tint0.h -- TINT0 master clock for Calypso GSM virtualization
 *
 * On real Calypso hardware, the C54x DSP Timer 0 (TIM/PRD/TCR at
 * DSP addresses 0x0024-0x0026) generates TINT0 (IFR bit 4, vector 20).
 * The DSP ROM configures Timer 0 to fire at TDMA frame rate (4.615ms).
 * TINT0 is the master clock that synchronizes everything:
 *
 *   TINT0 (DSP Timer 0, 4.615ms)
 *     +-- DSP: SINT17 frame interrupt -> process GSM tasks
 *     +-- TPU: frame sync -> burst scheduling
 *     +-- ARM: TPU_FRAME IRQ -> L1 scheduler
 *     +-- ARM: API IRQ -> read DSP results
 *     +-- UART: poll TX/RX
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_TINT0_H
#define CALYPSO_TINT0_H

#include <stdint.h>
#include <stdbool.h>

/* Hardware constants from TMS320C54x Timer 0 */
#define TINT0_PERIOD_US     4615         /* 4.615 ms per TDMA frame */
#define TINT0_PERIOD_NS     4615000ULL   /* in nanoseconds */
#define GSM_HYPERFRAME      2715648      /* GSM hyperframe modulus */

/* C54x Timer 0 registers (DSP data addresses) */
#define TINT0_TIM_ADDR      0x0024       /* Timer counter */
#define TINT0_PRD_ADDR      0x0025       /* Timer period */
#define TINT0_TCR_ADDR      0x0026       /* Timer control */

/* C54x IFR/IMR bit for TINT0 */
#define TINT0_IFR_BIT       3            /* IFR/IMR bit 4 */
#define TINT0_VEC           19           /* Interrupt vector 20 (offset 0x50) */

/* Start the master clock */
void calypso_tint0_start(void);

/* Notify that TPU_CTRL_EN was written (ARM requests frame processing) */
void calypso_tint0_tpu_en(void);
bool calypso_tint0_tpu_en_pending(void);
void calypso_tint0_tpu_en_clear(void);

/* Frame number access */
uint32_t calypso_tint0_fn(void);
void calypso_tint0_set_fn(uint32_t fn);
bool calypso_tint0_running(void);

#endif /* CALYPSO_TINT0_H */

===== ./hw/arm/calypso/sercomm_gate.c =====
/*
 * sercomm_gate.c — Sercomm DLCI router (HDLC demux)
 *
 * On real Calypso hardware, two separate paths exist:
 *   1. BSP (Baseband Serial Port) — radio bursts from ABB → DSP via PORTR 0xF430
 *   2. UART sercomm — L1CTL commands from host → firmware ARM via DLCI callbacks
 *
 * In QEMU, the bridge sends everything on one PTY:
 *   - Bursts as sercomm DLCI 4 (emulating BSP path)
 *   - L1CTL as sercomm DLCI 5 (real sercomm path)
 *
 * This gate separates them:
 *   DLCI 4 → calypso_trx_rx_burst() → c54x_bsp_load() + BRINT0  (BSP emulation)
 *   Others → re-wrap HDLC → UART RX FIFO → firmware sercomm layer (real path)
 *
 * Frame format (per osmocom sercomm.c):
 *   FLAG(7E) | DLCI(1) | CTRL(03) | DATA(N) | FLAG(7E)
 *   Escaping: 7E, 7D, 00 → 7D + (byte ^ 0x20)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/sercomm_gate.h"

#define SERCOMM_FLAG   0x7E
#define SERCOMM_ESCAPE 0x7D
#define SERCOMM_XOR    0x20

#define DLCI_BURST     4   /* SC_DLCI_DEBUG in osmocom — used for BSP emulation */

#define GATE_BUF_SIZE  512

/* HDLC parser state — matches osmocom sercomm.c rx_state */
enum gate_rx_state {
    GATE_WAIT_FLAG,   /* waiting for opening 0x7E */
    GATE_IN_FRAME,    /* collecting frame bytes */
    GATE_ESCAPE,      /* next byte is escaped (XOR 0x20) */
};

static uint8_t  sc_buf[GATE_BUF_SIZE];
static int      sc_len;
static enum gate_rx_state sc_state;

/*
 * Re-wrap a decoded frame and push into UART RX FIFO.
 * The firmware's sercomm_drv_rx_char() will parse it again — this is
 * intentional: it preserves the real firmware code path exactly.
 */
static void gate_push_to_fifo(CalypsoUARTState *s,
                              const uint8_t *frame, int len)
{
    calypso_uart_fifo_push(s, SERCOMM_FLAG);
    for (int i = 0; i < len; i++) {
        uint8_t c = frame[i];
        if (c == SERCOMM_FLAG || c == SERCOMM_ESCAPE || c == 0x00) {
            calypso_uart_fifo_push(s, SERCOMM_ESCAPE);
            calypso_uart_fifo_push(s, c ^ SERCOMM_XOR);
        } else {
            calypso_uart_fifo_push(s, c);
        }
    }
    calypso_uart_fifo_push(s, SERCOMM_FLAG);
}

/*
 * Dispatch a complete decoded sercomm frame.
 * frame[0] = DLCI, frame[1] = CTRL (0x03), frame[2..] = payload
 */
static void gate_dispatch(CalypsoUARTState *s,
                          const uint8_t *frame, int len)
{
    if (len < 2) return;

    uint8_t dlci = frame[0];

    if (dlci == DLCI_BURST) {
        /* BSP emulation path — skip DLCI(1) + CTRL(1) header */
        calypso_trx_rx_burst(&frame[2], len - 2);
        return;
    }

    /* All other DLCIs → firmware via FIFO (L1CTL, debug, console, echo) */
    gate_push_to_fifo(s, frame, len);
}

void sercomm_gate_feed(CalypsoUARTState *s, const uint8_t *buf, int size)
{
    for (int i = 0; i < size; i++) {
        uint8_t b = buf[i];

        switch (sc_state) {
        case GATE_WAIT_FLAG:
            if (b == SERCOMM_FLAG) {
                sc_state = GATE_IN_FRAME;
                sc_len = 0;
            } else {
                /* Pre-sercomm raw bytes → FIFO directly */
                calypso_uart_fifo_push(s, b);
            }
            break;

        case GATE_ESCAPE:
            if (sc_len < GATE_BUF_SIZE)
                sc_buf[sc_len++] = b ^ SERCOMM_XOR;
            sc_state = GATE_IN_FRAME;
            break;

        case GATE_IN_FRAME:
            if (b == SERCOMM_FLAG) {
                /* End of frame — dispatch if non-empty */
                if (sc_len > 0)
                    gate_dispatch(s, sc_buf, sc_len);
                sc_len = 0;
                /* Stay in GATE_IN_FRAME — next flag starts new frame */
            } else if (b == SERCOMM_ESCAPE) {
                sc_state = GATE_ESCAPE;
            } else {
                if (sc_len < GATE_BUF_SIZE)
                    sc_buf[sc_len++] = b;
            }
            break;
        }
    }
}

===== ./hw/arm/calypso/l1ctl_sock.c =====
/*
 * l1ctl_sock.c — L1CTL unix socket server for Calypso QEMU
 *
 * Replaces the Python bridge: provides a unix socket at /tmp/osmocom_l2
 * that speaks L1CTL (length-prefixed messages) to OsmocomBB mobile.
 *
 * Internally translates between:
 *   - sercomm framing (FLAG/ESCAPE/DLCI) on the firmware UART side
 *   - L1CTL length-prefix on the mobile socket side
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "sysemu/runstate.h"
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
#define L1CTL_SOCK_PATH    "/tmp/osmocom_l2_tmp"

#define L1CTL_LOG(fmt, ...) \
    fprintf(stderr, "[l1ctl-sock] " fmt "\n", ##__VA_ARGS__)

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

    /* TX buffer: firmware messages before client connects */
    uint8_t  tx_pending[8192];
    int      tx_pending_len;

    /* Reference to UART modem for RX injection */
    CalypsoUARTState *uart;

    /* Client/server: client read handler deferred until firmware speaks first */
    bool cli_rx_enabled;

    /* Burst mode gating: needs BOTH fbsb_requested AND TPU_CTRL_EN */
    bool burst_mode;
    bool fbsb_requested;  /* set on FBSB_REQ, cleared on RESET_REQ/disconnect */
} L1CTLSock;

static L1CTLSock g_l1ctl;

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
    if (s->cli_fd < 0) return;

    uint8_t hdr[2];
    hdr[0] = (len >> 8) & 0xFF;
    hdr[1] = len & 0xFF;

    /* Best-effort send */
    if (send(s->cli_fd, hdr, 2, MSG_NOSIGNAL) != 2 ||
        send(s->cli_fd, payload, len, MSG_NOSIGNAL) != len) {
        L1CTL_LOG("client send error, closing");
        close(s->cli_fd);
        s->cli_fd = -1;
    }
}

static void l1ctl_client_readable(void *opaque);

/* ---- Process a complete sercomm frame from firmware TX ---- */

static void sercomm_frame_complete(L1CTLSock *s)
{
    if (s->sc_len < 2) return;  /* need at least DLCI + CTRL */

    uint8_t dlci = s->sc_buf[0];
    /* uint8_t ctrl = s->sc_buf[1]; */
    uint8_t *payload = &s->sc_buf[2];
    int plen = s->sc_len - 2;

    if (dlci == SERCOMM_DLCI_L1CTL && plen > 0) {
        /* Firmware spoke first — enable client→firmware path */
        if (!s->cli_rx_enabled && s->cli_fd >= 0) {
            /* Drain stale data buffered before firmware was ready */
            uint8_t drain[4096];
            ssize_t drained = recv(s->cli_fd, drain, sizeof(drain), MSG_DONTWAIT);
            if (drained > 0)
                L1CTL_LOG("drained %zd stale bytes from mobile", drained);
            s->lp_len = 0;
            s->cli_rx_enabled = true;
            qemu_set_fd_handler(s->cli_fd, l1ctl_client_readable, NULL, s);
            L1CTL_LOG("firmware ready — accepting mobile input");
        }
        /* Parse L1CTL header: msg_type(1) flags(1) padding(2) */
        {
            static const char *l1ctl_names[] = {
                [0x00]="NONE", [0x01]="FBSB_REQ", [0x02]="FBSB_CONF",
                [0x03]="DATA_IND", [0x04]="RACH_REQ", [0x05]="RACH_CONF",
                [0x06]="DATA_REQ", [0x07]="RESET_IND", [0x08]="PM_REQ",
                [0x09]="PM_CONF", [0x0a]="ECHO_REQ", [0x0b]="ECHO_CONF",
                [0x0c]="DATA_CONF", [0x0d]="RESET_REQ", [0x0e]="RESET_CONF",
                [0x0f]="DATA_ABI", [0x10]="SIM_REQ", [0x11]="SIM_CONF",
                [0x12]="TCH_MODE_REQ", [0x13]="TCH_MODE_CONF",
                [0x14]="NEIGH_PM_REQ", [0x15]="NEIGH_PM_IND",
                [0x16]="TRAFFIC_REQ", [0x17]="TRAFFIC_CONF", [0x18]="TRAFFIC_IND",
            };
            uint8_t mt = payload[0];
            const char *name = (mt < 0x19) ? l1ctl_names[mt] : "UNKNOWN";
            L1CTL_LOG("TX→mobile: %s (0x%02x) len=%d", name, mt, plen);

            /* Decode specific messages */
            if (mt == 0x09 && plen >= 8) { /* PM_CONF */
                uint16_t arfcn = payload[4] | (payload[5] << 8);
                int16_t pm = (int16_t)(payload[6] | (payload[7] << 8));
                L1CTL_LOG("  PM_CONF: arfcn=%u pm=%d", arfcn & 0x3FF, pm);
            }
            if (mt == 0x02 && plen >= 8) { /* FBSB_CONF */
                int16_t snr = (int16_t)(payload[4] | (payload[5] << 8));
                uint8_t result = payload[6];
                uint8_t bsic = payload[7];
                L1CTL_LOG("  FBSB_CONF: snr=%d result=%u bsic=%u", snr, result, bsic);
            }
        }
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
    ssize_t n = recv(s->cli_fd, tmp, sizeof(tmp), 0);
    if (n <= 0) {
        L1CTL_LOG("client disconnected");
        qemu_set_fd_handler(s->cli_fd, NULL, NULL, NULL);
        close(s->cli_fd);
        s->cli_fd = -1;
        s->lp_len = 0;
        s->cli_rx_enabled = false;
        s->burst_mode = false;
        s->fbsb_requested = false;
        return;
    }

    /* Accumulate in length-prefix buffer */
    if (s->lp_len + (int)n > (int)sizeof(s->lp_buf)) {
        s->lp_len = 0;  /* overflow, reset */
    }
    memcpy(&s->lp_buf[s->lp_len], tmp, n);
    s->lp_len += (int)n;

    /* Parse ALL complete L1CTL messages in buffer */
    while (s->lp_len >= 2) {
        int msglen = (s->lp_buf[0] << 8) | s->lp_buf[1];
        if (s->lp_len < 2 + msglen)
            break;  /* incomplete message, wait for more data */

        uint8_t *payload = &s->lp_buf[2];

        /* Track L1CTL state for burst gating */
        if (msglen > 0) {
            if (payload[0] == 0x01) {  /* FBSB_REQ */
                s->fbsb_requested = true;
                L1CTL_LOG("FBSB_REQ → waiting for TPU_CTRL_EN");
            } else if (payload[0] == 0x0d) {  /* RESET_REQ */
                s->burst_mode = false;
                s->fbsb_requested = false;
                L1CTL_LOG("RESET_REQ → burst mode OFF");
            }
        }

        /* Wrap in sercomm and inject into UART RX */
        uint8_t frame[1024];
        int flen = sercomm_wrap(SERCOMM_DLCI_L1CTL, payload, msglen,
                                frame, sizeof(frame));
        if (flen > 0 && s->uart) {
            static const char *l1ctl_names[] = {
                [0x00]="NONE", [0x01]="FBSB_REQ", [0x02]="FBSB_CONF",
                [0x03]="DATA_IND", [0x04]="RACH_REQ", [0x05]="RACH_CONF",
                [0x06]="DATA_REQ", [0x07]="RESET_IND", [0x08]="PM_REQ",
                [0x09]="PM_CONF", [0x0d]="RESET_REQ", [0x0e]="RESET_CONF",
            };
            uint8_t mt = payload[0];
            const char *name = (mt < 0x0f) ? l1ctl_names[mt] : "UNKNOWN";
            if (!name) name = "UNKNOWN";
            L1CTL_LOG("RX←mobile: %s (0x%02x) len=%d → sercomm %d bytes",
                      name, mt, msglen, flen);

            if (mt == 0x01 && msglen >= 12) { /* FBSB_REQ */
                /* hdr(4) + band_arfcn(2) timeout(2) freq_err1(2) freq_err2(2) ... */
                uint16_t arfcn = payload[4] | (payload[5] << 8);
                uint16_t timeout = payload[6] | (payload[7] << 8);
                L1CTL_LOG("  FBSB_REQ: arfcn=%u band=%u timeout=%u",
                          arfcn & 0x3FF, (arfcn >> 10) & 0x3F, timeout);
            }
            if (mt == 0x08 && msglen >= 8) { /* PM_REQ */
                uint16_t arfcn = payload[4] | (payload[5] << 8);
                L1CTL_LOG("  PM_REQ: arfcn=%u", arfcn & 0x3FF);
            }

            calypso_uart_inject_raw(s->uart, frame, flen);
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
    /* Resume VM if it was started with -S (paused).
     * This ensures the firmware doesn't boot until a client is listening,
     * so the RESET_IND sent at boot reaches the mobile. */
    if (!runstate_is_running()) {
        s->cli_rx_enabled = false;
        L1CTL_LOG("client connected (fd=%d) — waiting for firmware boot", fd);
        vm_start();
    } else {
        /* VM already running (reconnect): enable reading immediately
         * so mobile can send RESET_REQ to the running firmware. */
        s->cli_rx_enabled = true;
        qemu_set_fd_handler(fd, l1ctl_client_readable, NULL, s);
        L1CTL_LOG("client reconnected (fd=%d) — firmware running", fd);
    }
}

/* Is the L1CTL client (mobile) connected and active? */
bool l1ctl_client_active(void)
{
    return g_l1ctl.cli_fd >= 0 && g_l1ctl.cli_rx_enabled;
}

/* Is the system in burst mode? */
bool l1ctl_burst_mode(void)
{
    return g_l1ctl.burst_mode;
}

/* Called by calypso_trx when firmware writes TPU_CTRL_EN.
 * Burst mode only activates if FBSB_REQ was received first. */
void l1ctl_set_burst_mode(bool on)
{
    if (on && g_l1ctl.fbsb_requested && !g_l1ctl.burst_mode) {
        g_l1ctl.burst_mode = true;
        L1CTL_LOG("burst mode ON (FBSB_REQ + TPU_CTRL_EN)");
    }
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

===== ./hw/arm/calypso/calypso_soc.c =====
/*
 * Calypso SoC - TI Calypso DBB (Digital Baseband)
 * DEBUG BUILD — verbose memory map logging
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "sysemu/sysemu.h"
#include "hw/arm/calypso/calypso_soc.h"
#include "hw/arm/calypso/calypso_trx.h"

/* Global references for TDMA tick to kick UARTs */
CalypsoUARTState *g_uart_modem;
CalypsoUARTState *g_uart_irda;
#include "chardev/char-fe.h"
#include "chardev/char.h"
#include "qemu/error-report.h"
#include "hw/arm/calypso/calypso_uart.h"

/* ---- Memory map ---- */
#define CALYPSO_IRAM_BASE     0x00800000
#define CALYPSO_IRAM_SIZE     (256 * 1024)

/* ---- Peripheral addresses ---- */
#define CALYPSO_INTH_BASE     0xFFFFFA00
#define CALYPSO_TIMER1_BASE   0xFFFE3800
#define CALYPSO_TIMER2_BASE   0xFFFE3C00
#define CALYPSO_SPI_BASE      0xFFFE3000
#define CALYPSO_KEYPAD_BASE   0xFFFE4800

/* Firmware perspective: UART_IRDA=0 → 0xFFFF5000, UART_MODEM=1 → 0xFFFF5800.
 * Sercomm (L1CTL) is on UART_MODEM (0xFFFF5800). */
#define CALYPSO_UART_IRDA     0xFFFF5000
#define CALYPSO_UART_MODEM    0xFFFF5800

/* ---- IRQ numbers ---- */
#define IRQ_TIMER1            1
#define IRQ_TIMER2            2
#define IRQ_UART_MODEM        7
#define IRQ_SPI               13
#define IRQ_UART_IRDA         18

/* ---- Stub MMIO ---- */

static uint64_t calypso_mmio8_read(void *o, hwaddr a, unsigned s) { return 0; }
static void calypso_mmio8_write(void *o, hwaddr a, uint64_t v, unsigned s) {}
static const MemoryRegionOps calypso_mmio8_ops = {
    .read = calypso_mmio8_read, .write = calypso_mmio8_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static uint64_t calypso_mmio16_read(void *o, hwaddr a, unsigned s) { return 0; }
static void calypso_mmio16_write(void *o, hwaddr a, uint64_t v, unsigned s) {}
static const MemoryRegionOps calypso_mmio16_ops = {
    .read = calypso_mmio16_read, .write = calypso_mmio16_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* ---- CNTL register (EXTRA_CONF) at 0xFFFFFD00 ----
 * Bit [8:9] = bootrom mapping control
 *   When cleared (0), IRAM is aliased at 0x00000000
 *   When set (3), internal ROM is mapped at 0x00000000
 *
 * On real Calypso, firmware calls calypso_bootrom(0) to disable
 * bootrom and enable IRAM at address 0 for exception vectors.
 */
static uint64_t calypso_cntl_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoSoCState *s = CALYPSO_SOC(opaque);
    if (offset == 0) return s->extra_conf;
    return 0;
}

static void calypso_cntl_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    CalypsoSoCState *s = CALYPSO_SOC(opaque);
    if (offset != 0) return;

    s->extra_conf = (uint16_t)value;

    /* Bits [9:8] control bootrom/IRAM mapping at address 0 */
    bool bootrom_enabled = (value >> 8) & 3;

    if (!bootrom_enabled && !s->iram_at_zero) {
        /* Map IRAM at address 0 (higher priority than flash) */
        MemoryRegion *sysmem = get_system_memory();
        memory_region_init_alias(&s->iram_alias, OBJECT(s),
                                  "calypso.iram_at_zero",
                                  &s->iram, 0, CALYPSO_IRAM_SIZE);
        memory_region_add_subregion_overlap(sysmem, 0x00000000,
                                             &s->iram_alias, 1);
        s->iram_at_zero = true;
        fprintf(stderr, "[SOC] CNTL: IRAM aliased at 0x00000000 (bootrom disabled)\n");
    } else if (bootrom_enabled && s->iram_at_zero) {
        /* Remove IRAM alias */
        memory_region_del_subregion(get_system_memory(), &s->iram_alias);
        object_unparent(OBJECT(&s->iram_alias));
        s->iram_at_zero = false;
        fprintf(stderr, "[SOC] CNTL: IRAM alias removed (bootrom enabled)\n");
    }
}

static const MemoryRegionOps calypso_cntl_ops = {
    .read = calypso_cntl_read,
    .write = calypso_cntl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

static uint64_t calypso_kp_read(void *o, hwaddr a, unsigned s) { return 0xFF; }
static void calypso_kp_write(void *o, hwaddr a, uint64_t v, unsigned s) {}
static const MemoryRegionOps calypso_keypad_ops = {
    .read = calypso_kp_read, .write = calypso_kp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void add_stub(MemoryRegion *sys, const char *name,
                     hwaddr base, const MemoryRegionOps *ops)
{
    MemoryRegion *mr = g_new(MemoryRegion, 1);
    memory_region_init_io(mr, NULL, ops, NULL, name, 0x100);
    memory_region_add_subregion(sys, base, mr);
    fprintf(stderr, "[SOC] stub '%s' @ 0x%08lx (0x100)\n", name, (unsigned long)base);
}

/* ================================================================
 * SoC realize
 * ================================================================ */

static void calypso_soc_realize(DeviceState *dev, Error **errp)
{
    CalypsoSoCState *s = CALYPSO_SOC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    MemoryRegion *sysmem = get_system_memory();
    Error *err = NULL;

    fprintf(stderr, "[SOC] === calypso_soc_realize START ===\n");

    /* ---- IRAM at 0x00800000 ONLY ----
     * NO alias at 0x00000000 — flash lives there (board-level).
     */
    memory_region_init_ram(&s->iram, OBJECT(dev), "calypso.iram",
                           CALYPSO_IRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, CALYPSO_IRAM_BASE, &s->iram);
    fprintf(stderr, "[SOC] IRAM @ 0x%08x (%d KiB) — NO alias at 0x00000000\n",
            CALYPSO_IRAM_BASE, CALYPSO_IRAM_SIZE / 1024);

    /* ---- INTH ---- */
    object_initialize_child(OBJECT(dev), "inth", &s->inth, TYPE_CALYPSO_INTH);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->inth), &err)) {
        error_propagate(errp, err); return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->inth), 0, CALYPSO_INTH_BASE);

    /* Pass INTH's output IRQs (parent_irq, parent_fiq) through
     * the SoC device so the board can connect them to the CPU.
     * This avoids the ordering issue where sysbus_connect_irq
     * captures a NULL qemu_irq before the board connects it. */
    sysbus_pass_irq(sbd, SYS_BUS_DEVICE(&s->inth));

    #define INTH_IRQ(n) qdev_get_gpio_in(DEVICE(&s->inth), (n))

    /* ---- Timer 1 ---- */
    object_initialize_child(OBJECT(dev), "timer1", &s->timer1, TYPE_CALYPSO_TIMER);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer1), &err)) {
        error_propagate(errp, err); return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer1), 0, CALYPSO_TIMER1_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer1), 0, INTH_IRQ(IRQ_TIMER1));

    /* ---- Timer 2 ---- */
    object_initialize_child(OBJECT(dev), "timer2", &s->timer2, TYPE_CALYPSO_TIMER);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer2), &err)) {
        error_propagate(errp, err); return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer2), 0, CALYPSO_TIMER2_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer2), 0, INTH_IRQ(IRQ_TIMER2));

    /* ---- I2C stub ---- */
    DeviceState *i2c_dev = qdev_new("calypso-i2c");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(i2c_dev), 0, 0xFFFE1800);

    /* ---- SPI ---- */
    object_initialize_child(OBJECT(dev), "spi", &s->spi, TYPE_CALYPSO_SPI);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi), &err)) {
        error_propagate(errp, err); return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->spi), 0, CALYPSO_SPI_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi), 0, INTH_IRQ(IRQ_SPI));

    /* ---- UART MODEM ---- */
    {
        Chardev *chr = qemu_chr_find("modem");
        if (!chr) chr = serial_hd(0);

        fprintf(stderr, "[SOC] UART modem: chardev → %s\n",
                chr ? (chr->label ? chr->label : "(no label)") : "NULL");

        object_initialize_child(OBJECT(dev), "uart-modem",
                                &s->uart_modem, TYPE_CALYPSO_UART);
        qdev_prop_set_string(DEVICE(&s->uart_modem), "label", "modem");

        if (chr) {
            qdev_prop_set_chr(DEVICE(&s->uart_modem), "chardev", chr);
        }

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->uart_modem), &err)) {
            error_propagate(errp, err); return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->uart_modem), 0, CALYPSO_UART_MODEM);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart_modem), 0,
                           INTH_IRQ(IRQ_UART_MODEM));
        g_uart_modem = &s->uart_modem;

        /* L1CTL socket: sercomm↔L1CTL relay — firmware binds sercomm to MODEM UART */
        {
            const char *l1ctl_path = getenv("L1CTL_SOCK");
            l1ctl_sock_init(&s->uart_modem, l1ctl_path ? l1ctl_path : "/tmp/osmocom_l2_1");
        }
    }

    /* ---- UART IRDA ---- */
    {
        Chardev *chr = qemu_chr_find("irda");
        if (!chr) chr = serial_hd(1);

        object_initialize_child(OBJECT(dev), "uart-irda",
                                &s->uart_irda, TYPE_CALYPSO_UART);
        qdev_prop_set_string(DEVICE(&s->uart_irda), "label", "irda");

        if (chr) {
            qdev_prop_set_chr(DEVICE(&s->uart_irda), "chardev", chr);
        }

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->uart_irda), &err)) {
            error_propagate(errp, err); return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->uart_irda), 0, CALYPSO_UART_IRDA);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart_irda), 0,
                           INTH_IRQ(IRQ_UART_IRDA));
        g_uart_irda = &s->uart_irda;
    }

    /* ---- TRX bridge (pure hardware) ---- */
    {
        qemu_irq *irqs = g_new0(qemu_irq, CALYPSO_NUM_IRQS);
        for (int i = 0; i < CALYPSO_NUM_IRQS; i++)
            irqs[i] = INTH_IRQ(i);
        calypso_trx_init(sysmem, irqs);
    }

    #undef INTH_IRQ

    /* ---- Stubs ----
     *
     * IMPORTANT: NO stub at 0x00000300 ("calypso.low300")!
     * That address falls inside the flash range 0x00000000–0x003FFFFF
     * and would shadow pflash CFI queries → "Failed to initialize flash!"
     */
    add_stub(sysmem, "calypso.keypad",     CALYPSO_KEYPAD_BASE, &calypso_keypad_ops);
    add_stub(sysmem, "calypso.tmr6800",    0xFFFE6800, &calypso_mmio8_ops);
    add_stub(sysmem, "calypso.mmio_80xx",  0xFFFE8000, &calypso_mmio8_ops);
    add_stub(sysmem, "calypso.conf",       0xFFFEF000, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.mmio_98xx",  0xFFFF9800, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.dpll",       0xFFFFF000, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.rhea",       0xFFFFF900, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.clkm",       0xFFFFFB00, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.mmio_fcxx",  0xFFFFFC00, &calypso_mmio16_ops);
    /* CNTL (EXTRA_CONF) - controls IRAM-at-zero mapping */
    memory_region_init_io(&s->cntl_iomem, OBJECT(dev), &calypso_cntl_ops, s,
                          "calypso.cntl", 0x100);
    memory_region_add_subregion(sysmem, 0xFFFFFD00, &s->cntl_iomem);
    s->extra_conf = 0x0300; /* bootrom enabled at reset */
    s->iram_at_zero = false;
    add_stub(sysmem, "calypso.dio",        0xFFFFFF00, &calypso_mmio8_ops);
    /* NO calypso.low300 — it overlaps flash! */

    /* Catch-all (lowest priority) */
    {
        MemoryRegion *mr = g_new(MemoryRegion, 1);
        memory_region_init_io(mr, NULL, &calypso_mmio8_ops, NULL,
                              "calypso.catchall", 0x100000);
        memory_region_add_subregion_overlap(sysmem, 0xFFF00000, mr, -1);
    }

    fprintf(stderr, "[SOC] === calypso_soc_realize DONE ===\n");
}

/* ---- QOM boilerplate ---- */

static Property calypso_soc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void calypso_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = calypso_soc_realize;
    device_class_set_props(dc, calypso_soc_properties);
    dc->user_creatable = false;
}

static const TypeInfo calypso_soc_type_info = {
    .name          = TYPE_CALYPSO_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoSoCState),
    .class_init    = calypso_soc_class_init,
};

static void calypso_soc_register_types(void)
{
    type_register_static(&calypso_soc_type_info);
}

type_init(calypso_soc_register_types)

===== ./hw/arm/meson.build =====
arm_ss = ss.source_set()
arm_ss.add(files('boot.c'))
arm_ss.add(when: 'CONFIG_ARM_VIRT', if_true: files('virt.c'))
arm_ss.add(when: 'CONFIG_ACPI', if_true: files('virt-acpi-build.c'))
arm_ss.add(when: 'CONFIG_DIGIC', if_true: files('digic_boards.c'))
arm_ss.add(when: 'CONFIG_EMCRAFT_SF2', if_true: files('msf2-som.c'))
arm_ss.add(when: 'CONFIG_HIGHBANK', if_true: files('highbank.c'))
arm_ss.add(when: 'CONFIG_INTEGRATOR', if_true: files('integratorcp.c'))
arm_ss.add(when: 'CONFIG_MICROBIT', if_true: files('microbit.c'))
arm_ss.add(when: 'CONFIG_MPS3R', if_true: files('mps3r.c'))
arm_ss.add(when: 'CONFIG_MUSICPAL', if_true: files('musicpal.c'))
arm_ss.add(when: 'CONFIG_NETDUINOPLUS2', if_true: files('netduinoplus2.c'))
arm_ss.add(when: 'CONFIG_OLIMEX_STM32_H405', if_true: files('olimex-stm32-h405.c'))
arm_ss.add(when: 'CONFIG_NPCM7XX', if_true: files('npcm7xx.c', 'npcm7xx_boards.c'))
arm_ss.add(when: 'CONFIG_REALVIEW', if_true: files('realview.c'))
arm_ss.add(when: 'CONFIG_SBSA_REF', if_true: files('sbsa-ref.c'))
arm_ss.add(when: 'CONFIG_STELLARIS', if_true: files('stellaris.c'))
arm_ss.add(when: 'CONFIG_STM32VLDISCOVERY', if_true: files('stm32vldiscovery.c'))
arm_ss.add(when: 'CONFIG_ZYNQ', if_true: files('xilinx_zynq.c'))
arm_ss.add(when: 'CONFIG_SABRELITE', if_true: files('sabrelite.c'))

arm_ss.add(when: 'CONFIG_ARM_V7M', if_true: files('armv7m.c'))
arm_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4210.c'))
arm_ss.add(when: 'CONFIG_DIGIC', if_true: files('digic.c'))
arm_ss.add(when: 'CONFIG_OMAP', if_true: files('omap1.c'))
arm_ss.add(when: 'CONFIG_ALLWINNER_A10', if_true: files('allwinner-a10.c', 'cubieboard.c'))
arm_ss.add(when: 'CONFIG_ALLWINNER_H3', if_true: files('allwinner-h3.c', 'orangepi.c'))
arm_ss.add(when: 'CONFIG_ALLWINNER_R40', if_true: files('allwinner-r40.c', 'bananapi_m2u.c'))
arm_ss.add(when: 'CONFIG_RASPI', if_true: files('bcm2836.c', 'raspi.c'))
arm_ss.add(when: ['CONFIG_RASPI', 'TARGET_AARCH64'], if_true: files('bcm2838.c', 'raspi4b.c'))
arm_ss.add(when: 'CONFIG_STM32F100_SOC', if_true: files('stm32f100_soc.c'))
arm_ss.add(when: 'CONFIG_STM32F205_SOC', if_true: files('stm32f205_soc.c'))
arm_ss.add(when: 'CONFIG_STM32F405_SOC', if_true: files('stm32f405_soc.c'))
arm_ss.add(when: 'CONFIG_B_L475E_IOT01A', if_true: files('b-l475e-iot01a.c'))
arm_ss.add(when: 'CONFIG_STM32L4X5_SOC', if_true: files('stm32l4x5_soc.c'))
arm_ss.add(when: 'CONFIG_XLNX_ZYNQMP_ARM', if_true: files('xlnx-zynqmp.c', 'xlnx-zcu102.c'))
arm_ss.add(when: 'CONFIG_XLNX_VERSAL', if_true: files('xlnx-versal.c', 'xlnx-versal-virt.c'))
arm_ss.add(when: 'CONFIG_FSL_IMX25', if_true: files('fsl-imx25.c', 'imx25_pdk.c'))
arm_ss.add(when: 'CONFIG_FSL_IMX31', if_true: files('fsl-imx31.c', 'kzm.c'))
arm_ss.add(when: 'CONFIG_FSL_IMX6', if_true: files('fsl-imx6.c'))
arm_ss.add(when: 'CONFIG_ASPEED_SOC', if_true: files(
  'aspeed.c',
  'aspeed_soc_common.c',
  'aspeed_ast2400.c',
  'aspeed_ast2600.c',
  'aspeed_ast10x0.c',
  'aspeed_eeprom.c',
  'fby35.c'))
arm_ss.add(when: ['CONFIG_ASPEED_SOC', 'TARGET_AARCH64'], if_true: files('aspeed_ast27x0.c'))
arm_ss.add(when: 'CONFIG_MPS2', if_true: files('mps2.c'))
arm_ss.add(when: 'CONFIG_MPS2', if_true: files('mps2-tz.c'))
arm_ss.add(when: 'CONFIG_MSF2', if_true: files('msf2-soc.c'))
arm_ss.add(when: 'CONFIG_MUSCA', if_true: files('musca.c'))
arm_ss.add(when: 'CONFIG_ARMSSE', if_true: files('armsse.c'))
arm_ss.add(when: 'CONFIG_FSL_IMX7', if_true: files('fsl-imx7.c', 'mcimx7d-sabre.c'))
arm_ss.add(when: 'CONFIG_ARM_SMMUV3', if_true: files('smmuv3.c'))
arm_ss.add(when: 'CONFIG_FSL_IMX6UL', if_true: files('fsl-imx6ul.c', 'mcimx6ul-evk.c'))
arm_ss.add(when: 'CONFIG_NRF51_SOC', if_true: files('nrf51_soc.c'))
arm_ss.add(when: 'CONFIG_XEN', if_true: files(
  'xen-stubs.c',
  'xen-pvh.c',
))

system_ss.add(when: 'CONFIG_ARM_SMMUV3', if_true: files('smmu-common.c'))
system_ss.add(when: 'CONFIG_COLLIE', if_true: files('collie.c'))
system_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4_boards.c'))
system_ss.add(when: 'CONFIG_NETDUINO2', if_true: files('netduino2.c'))
system_ss.add(when: 'CONFIG_RASPI', if_true: files('bcm2835_peripherals.c'))
system_ss.add(when: 'CONFIG_RASPI', if_true: files('bcm2838_peripherals.c'))
system_ss.add(when: 'CONFIG_STRONGARM', if_true: files('strongarm.c'))
system_ss.add(when: 'CONFIG_SX1', if_true: files('omap_sx1.c'))
system_ss.add(when: 'CONFIG_VERSATILE', if_true: files('versatilepb.c'))
system_ss.add(when: 'CONFIG_VEXPRESS', if_true: files('vexpress.c'))

subdir('calypso')

hw_arch += {'arm': arm_ss}

===== ./hw/arm/Kconfig =====
config CALYPSO
    bool
    default y
    depends on ARM
    select PFLASH_CFI01

config ARM_VIRT
    bool
    default y
    depends on ARM
    imply PCI_DEVICES
    imply TEST_DEVICES
    imply VFIO_AMD_XGBE
    imply VFIO_PLATFORM
    imply VFIO_XGMAC
    imply TPM_TIS_SYSBUS
    imply TPM_TIS_I2C
    imply NVDIMM
    imply IOMMUFD
    select ARM_GIC
    select ACPI
    select ARM_SMMUV3
    select GPIO_KEY
    select DEVICE_TREE
    select FW_CFG_DMA
    select PCI_EXPRESS
    select PCI_EXPRESS_GENERIC_BRIDGE
    select PFLASH_CFI01
    select PL011 if !HAVE_RUST # UART
    select X_PL011_RUST if HAVE_RUST # UART
    select PL031 # RTC
    select PL061 # GPIO
    select GPIO_PWR
    select PLATFORM_BUS
    select SMBIOS
    select VIRTIO_MMIO
    select ACPI_PCI
    select MEM_DEVICE
    select DIMM
    select ACPI_HW_REDUCED
    select ACPI_APEI
    select ACPI_VIOT
    select VIRTIO_MEM_SUPPORTED
    select ACPI_CXL
    select ACPI_HMAT

config CUBIEBOARD
    bool
    default y
    depends on TCG && ARM
    select ALLWINNER_A10

config DIGIC
    bool
    default y
    depends on TCG && ARM
    select PTIMER
    select PFLASH_CFI02

config EXYNOS4
    bool
    default y
    depends on TCG && ARM
    imply I2C_DEVICES
    select A9MPCORE
    select I2C
    select LAN9118
    select PL310 # cache controller
    select PTIMER
    select SDHCI
    select USB_EHCI_SYSBUS
    select OR_IRQ

config HIGHBANK
    bool
    default y
    depends on TCG && ARM
    select A9MPCORE
    select A15MPCORE
    select AHCI
    select ARM_TIMER # sp804
    select ARM_V7M
    select PL011 if !HAVE_RUST # UART
    select X_PL011_RUST if HAVE_RUST # UART
    select PL022 # SPI
    select PL031 # RTC
    select PL061 # GPIO
    select PL310 # cache controller
    select XGMAC # ethernet

config INTEGRATOR
    bool
    default y
    depends on TCG && ARM
    select ARM_TIMER
    select INTEGRATOR_DEBUG
    select PL011 if !HAVE_RUST # UART
    select X_PL011_RUST if HAVE_RUST # UART
    select PL031 # RTC
    select PL041 # audio
    select PL050 # keyboard/mouse
    select PL110 # pl111 LCD controller
    select PL181 # display
    select SMC91C111

config MPS3R
    bool
    default y
    depends on TCG && ARM

config MUSCA
    bool
    default y
    depends on TCG && ARM
    select ARMSSE
    select PL011 if !HAVE_RUST # UART
    select X_PL011_RUST if HAVE_RUST # UART
    select PL031
    select SPLIT_IRQ
    select UNIMP

config MARVELL_88W8618
    bool

config MUSICPAL
    bool
    default y
    depends on TCG && ARM
    select OR_IRQ
    select BITBANG_I2C
    select MARVELL_88W8618
    select PTIMER
    select PFLASH_CFI02
    select SERIAL_MM
    select WM8750

config NETDUINO2
    bool
    default y
    depends on TCG && ARM
    select STM32F205_SOC

config NETDUINOPLUS2
    bool
    default y
    depends on TCG && ARM
    select STM32F405_SOC

config OLIMEX_STM32_H405
    bool
    default y
    depends on TCG && ARM
    select STM32F405_SOC

config OMAP
    bool
    select FRAMEBUFFER
    select I2C
    select NAND
    select PFLASH_CFI01
    select SD
    select SERIAL_MM

config REALVIEW
    bool
    default y
    depends on TCG && ARM
    imply PCI_DEVICES
    imply PCI_TESTDEV
    imply I2C_DEVICES
    select SMC91C111
    select LAN9118
    select A9MPCORE
    select A15MPCORE
    select ARM11MPCORE
    select ARM_TIMER
    select VERSATILE_PCI
    select WM8750 # audio codec
    select LSI_SCSI_PCI
    select PCI
    select PL011 if !HAVE_RUST # UART
    select X_PL011_RUST if HAVE_RUST # UART
    select PL031  # RTC
    select PL041  # audio codec
    select PL050  # keyboard/mouse
    select PL061  # GPIO
    select PL080  # DMA controller
    select PL110
    select PL181  # display
    select PL310  # cache controller
    select ARM_SBCON_I2C
    select DS1338 # I2C RTC+NVRAM
    select USB_OHCI_SYSBUS

config SBSA_REF
    bool
    default y
    depends on TCG && AARCH64
    imply PCI_DEVICES
    select DEVICE_TREE
    select AHCI
    select ARM_SMMUV3
    select GPIO_KEY
    select PCI_EXPRESS
    select PCI_EXPRESS_GENERIC_BRIDGE
    select PFLASH_CFI01
    select PL011 if !HAVE_RUST # UART
    select X_PL011_RUST if HAVE_RUST # UART
    select PL031 # RTC
    select PL061 # GPIO
    select USB_XHCI_SYSBUS
    select WDT_SBSA
    select BOCHS_DISPLAY
    select IDE_BUS
    select IDE_DEV

config SABRELITE
    bool
    default y
    depends on TCG && ARM
    select FSL_IMX6
    select SSI_M25P80

config STELLARIS
    bool
    default y
    depends on TCG && ARM
    imply I2C_DEVICES
    select ARM_V7M
    select CMSDK_APB_WATCHDOG
    select I2C
    select PL011 if !HAVE_RUST # UART
    select X_PL011_RUST if HAVE_RUST # UART
    select PL022 # SPI
    select PL061 # GPIO
    select SSD0303 # OLED display
    select SSD0323 # OLED display
    select SSI_SD
    select STELLARIS_GAMEPAD
    select STELLARIS_ENET # ethernet
    select STELLARIS_GPTM # general purpose timer module
    select UNIMP

config STM32VLDISCOVERY
    bool
    default y
    depends on TCG && ARM
    select STM32F100_SOC

config STRONGARM
    bool
    select PXA2XX_TIMER
    select SSI

config COLLIE
    bool
    default y
    depends on TCG && ARM
    select PFLASH_CFI01
    select ZAURUS_SCOOP
    select STRONGARM

config SX1
    bool
    default y
    depends on TCG && ARM
    select OMAP

config VERSATILE
    bool
    default y
    depends on TCG && ARM
    select ARM_TIMER # sp804
    select PFLASH_CFI01
    select LSI_SCSI_PCI
    select PL050  # keyboard/mouse
    select PL080  # DMA controller
    select PL190  # Vector PIC
    select REALVIEW
    select USB_OHCI_SYSBUS

config VEXPRESS
    bool
    default y
    depends on TCG && ARM
    select DEVICE_TREE
    select A9MPCORE
    select A15MPCORE
    select ARM_MPTIMER
    select ARM_TIMER # sp804
    select LAN9118
    select PFLASH_CFI01
    select PL011 if !HAVE_RUST # UART
    select X_PL011_RUST if HAVE_RUST # UART
    select PL041 # audio codec
    select PL181  # display
    select REALVIEW
    select SII9022
    select VIRTIO_MMIO

config ZYNQ
    bool
    default y
    depends on TCG && ARM
    select A9MPCORE
    select CADENCE # UART
    select PFLASH_CFI02
    select PL310 # cache controller
    select PL330
    select SDHCI
    select SSI_M25P80
    select USB_CHIPIDEA
    select XILINX # UART
    select XILINX_AXI
    select XILINX_SPI
    select XILINX_SPIPS
    select ZYNQ_DEVCFG

config ARM_V7M
    bool
    # currently v7M must be included in a TCG build due to translate.c
    default y
    depends on TCG && ARM
    select PTIMER

config ALLWINNER_A10
    bool
    select AHCI
    select ALLWINNER_A10_PIT
    select ALLWINNER_A10_PIC
    select ALLWINNER_A10_CCM
    select ALLWINNER_A10_DRAMC
    select ALLWINNER_WDT
    select ALLWINNER_EMAC
    select ALLWINNER_I2C
    select ALLWINNER_A10_SPI
    select AXP2XX_PMU
    select SERIAL_MM
    select UNIMP
    select USB_OHCI_SYSBUS

config ALLWINNER_H3
    bool
    default y
    depends on TCG && ARM
    select ALLWINNER_A10_PIT
    select ALLWINNER_SUN8I_EMAC
    select ALLWINNER_I2C
    select ALLWINNER_WDT
    select SERIAL_MM
    select ARM_TIMER
    select ARM_GIC
    select UNIMP
    select USB_OHCI_SYSBUS
    select USB_EHCI_SYSBUS
    select SD

config ALLWINNER_R40
    bool
    default y if TCG && ARM
    select AHCI
    select ALLWINNER_SRAMC
    select ALLWINNER_A10_PIT
    select ALLWINNER_WDT
    select AXP2XX_PMU
    select SERIAL_MM
    select ARM_TIMER
    select ARM_GIC
    select UNIMP
    select USB_OHCI_SYSBUS
    select USB_EHCI_SYSBUS
    select SD

config RASPI
    bool
    default y
    depends on TCG && ARM
    select FRAMEBUFFER
    select PL011 if !HAVE_RUST # UART
    select X_PL011_RUST if HAVE_RUST # UART
    select SDHCI
    select USB_DWC2
    select BCM2835_SPI
    select BCM2835_I2C

config STM32F100_SOC
    bool
    select ARM_V7M
    select STM32F2XX_USART
    select STM32F2XX_SPI

config STM32F205_SOC
    bool
    select ARM_V7M
    select OR_IRQ
    select STM32F2XX_TIMER
    select STM32F2XX_USART
    select STM32F2XX_SYSCFG
    select STM32F2XX_ADC
    select STM32F2XX_SPI

config STM32F405_SOC
    bool
    select ARM_V7M
    select OR_IRQ
    select STM32_RCC
    select STM32F4XX_SYSCFG
    select STM32F4XX_EXTI

config B_L475E_IOT01A
    bool
    default y
    depends on TCG && ARM
    select STM32L4X5_SOC
    imply DM163

config STM32L4X5_SOC
    bool
    select ARM_V7M
    select OR_IRQ
    select STM32L4X5_EXTI
    select STM32L4X5_SYSCFG
    select STM32L4X5_RCC
    select STM32L4X5_GPIO
    select STM32L4X5_USART

config XLNX_ZYNQMP_ARM
    bool
    default y if PIXMAN
    depends on TCG && AARCH64
    select AHCI
    select ARM_GIC
    select CADENCE
    select CPU_CLUSTER
    select DDC
    select DPCD
    select DEVICE_TREE
    select SDHCI
    select SSI
    select SSI_M25P80
    select XILINX_AXI
    select XILINX_SPIPS
    select XLNX_CSU_DMA
    select XLNX_DISPLAYPORT
    select XLNX_ZYNQMP
    select XLNX_ZDMA
    select USB_DWC3

config XLNX_VERSAL
    bool
    default y
    depends on TCG && AARCH64
    select ARM_GIC
    select CPU_CLUSTER
    select DEVICE_TREE
    select PL011 if !HAVE_RUST # UART
    select X_PL011_RUST if HAVE_RUST # UART
    select CADENCE
    select VIRTIO_MMIO
    select UNIMP
    select XLNX_ZDMA
    select XLNX_ZYNQMP
    select OR_IRQ
    select XLNX_BBRAM
    select XLNX_EFUSE_VERSAL
    select XLNX_USB_SUBSYS
    select XLNX_VERSAL_TRNG
    select XLNX_CSU_DMA

config NPCM7XX
    bool
    default y
    depends on TCG && ARM
    select A9MPCORE
    select ADM1266
    select ADM1272
    select ARM_GIC
    select SMBUS
    select AT24C  # EEPROM
    select MAX34451
    select ISL_PMBUS_VR
    select PL310  # cache controller
    select PMBUS
    select SERIAL_MM
    select SSI
    select UNIMP
    select PCA954X
    select USB_OHCI_SYSBUS

config FSL_IMX25
    bool
    default y
    depends on TCG && ARM
    imply I2C_DEVICES
    select IMX
    select IMX_FEC
    select IMX_I2C
    select USB_CHIPIDEA
    select WDT_IMX2
    select SDHCI

config FSL_IMX31
    bool
    default y
    depends on TCG && ARM
    imply I2C_DEVICES
    select SERIAL_MM
    select IMX
    select IMX_I2C
    select WDT_IMX2
    select LAN9118

config FSL_IMX6
    bool
    imply PCIE_DEVICES
    imply I2C_DEVICES
    select A9MPCORE
    select IMX
    select IMX_FEC
    select IMX_I2C
    select IMX_USBPHY
    select WDT_IMX2
    select PL310  # cache controller
    select PCI_EXPRESS_DESIGNWARE
    select SDHCI
    select USB_CHIPIDEA

config ASPEED_SOC
    bool
    default y
    depends on TCG && ARM
    select DS1338
    select FTGMAC100
    select I2C
    select DPS310
    select PCA9552
    select SERIAL_MM
    select SMBUS_EEPROM
    select PCA954X
    select SSI
    select SSI_M25P80
    select TMP105
    select TMP421
    select EMC141X
    select UNIMP
    select LED
    select PMBUS
    select MAX31785
    select FSI_APB2OPB_ASPEED
    select AT24C

config MPS2
    bool
    default y
    depends on TCG && ARM
    imply I2C_DEVICES
    select ARMSSE
    select LAN9118
    select MPS2_FPGAIO
    select MPS2_SCC
    select OR_IRQ
    select PL022    # SPI
    select PL080    # DMA controller
    select SPLIT_IRQ
    select UNIMP
    select CMSDK_APB_WATCHDOG
    select ARM_SBCON_I2C

config FSL_IMX7
    bool
    default y
    depends on TCG && ARM
    imply PCI_DEVICES
    imply TEST_DEVICES
    imply I2C_DEVICES
    select A15MPCORE
    select PCI
    select IMX
    select IMX_FEC
    select IMX_I2C
    select WDT_IMX2
    select PCI_EXPRESS_DESIGNWARE
    select SDHCI
    select UNIMP
    select USB_CHIPIDEA

config ARM_SMMUV3
    bool

config FSL_IMX6UL
    bool
    default y
    depends on TCG && ARM
    imply I2C_DEVICES
    select A15MPCORE
    select IMX
    select IMX_FEC
    select IMX_I2C
    select WDT_IMX2
    select SDHCI
    select USB_CHIPIDEA
    select UNIMP

config MICROBIT
    bool
    default y
    depends on TCG && ARM
    select NRF51_SOC

config NRF51_SOC
    bool
    imply I2C_DEVICES
    select I2C
    select ARM_V7M
    select UNIMP

config EMCRAFT_SF2
    bool
    default y
    depends on TCG && ARM
    select MSF2
    select SSI_M25P80

config MSF2
    bool
    select ARM_V7M
    select PTIMER
    select SERIAL_MM
    select SSI
    select UNIMP

config ARMSSE
    bool
    select ARM_V7M
    select ARMSSE_CPU_PWRCTRL
    select ARMSSE_CPUID
    select ARMSSE_MHU
    select CMSDK_APB_TIMER
    select CMSDK_APB_DUALTIMER
    select CMSDK_APB_UART
    select CMSDK_APB_WATCHDOG
    select CPU_CLUSTER
    select IOTKIT_SECCTL
    select IOTKIT_SYSCTL
    select IOTKIT_SYSINFO
    select OR_IRQ
    select SPLIT_IRQ
    select TZ_MPC
    select TZ_MSC
    select TZ_PPC
    select UNIMP
    select SSE_COUNTER
    select SSE_TIMER

===== ./hw/intc/meson.build =====
system_ss.add(files('intc.c'))
system_ss.add(when: 'CONFIG_ARM_GIC', if_true: files(
  'arm_gic.c',
  'arm_gic_common.c',
  'arm_gicv2m.c',
  'arm_gicv3_common.c',
  'arm_gicv3_its_common.c',
))
system_ss.add(when: 'CONFIG_ARM_GICV3_TCG', if_true: files(
  'arm_gicv3.c',
  'arm_gicv3_dist.c',
  'arm_gicv3_its.c',
  'arm_gicv3_redist.c',
))
system_ss.add(when: 'CONFIG_ALLWINNER_A10_PIC', if_true: files('allwinner-a10-pic.c'))
system_ss.add(when: 'CONFIG_ASPEED_SOC', if_true: files('aspeed_vic.c'))
system_ss.add(when: 'CONFIG_ASPEED_SOC', if_true: files('aspeed_intc.c'))
system_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4210_gic.c', 'exynos4210_combiner.c'))
system_ss.add(when: 'CONFIG_GOLDFISH_PIC', if_true: files('goldfish_pic.c'))
system_ss.add(when: 'CONFIG_HEATHROW_PIC', if_true: files('heathrow_pic.c'))
system_ss.add(when: 'CONFIG_I8259', if_true: files('i8259_common.c', 'i8259.c'))
system_ss.add(when: 'CONFIG_IMX', if_true: files('imx_avic.c', 'imx_gpcv2.c'))
system_ss.add(when: 'CONFIG_IOAPIC', if_true: files('ioapic_common.c'), if_false: files('ioapic-stub.c'))
system_ss.add(when: 'CONFIG_OMAP', if_true: files('omap_intc.c'))
system_ss.add(when: 'CONFIG_OPENPIC', if_true: files('openpic.c'))
system_ss.add(when: 'CONFIG_PL190', if_true: files('pl190.c'))
system_ss.add(when: 'CONFIG_RASPI', if_true: files('bcm2835_ic.c', 'bcm2836_control.c'))
system_ss.add(when: 'CONFIG_REALVIEW', if_true: files('realview_gic.c'))
system_ss.add(when: 'CONFIG_SLAVIO', if_true: files('slavio_intctl.c'))
system_ss.add(when: 'CONFIG_XILINX', if_true: files('xilinx_intc.c'))
system_ss.add(when: 'CONFIG_XLNX_ZYNQMP', if_true: files('xlnx-zynqmp-ipi.c'))
system_ss.add(when: 'CONFIG_XLNX_ZYNQMP_PMU', if_true: files('xlnx-pmu-iomod-intc.c'))

if config_all_devices.has_key('CONFIG_APIC') or \
   config_all_devices.has_key('CONFIG_I8259') or \
   config_all_devices.has_key('CONFIG_MC146818RTC')
    system_ss.add(files('kvm_irqcount.c'))
endif

specific_ss.add(when: 'CONFIG_APIC', if_true: files('apic.c', 'apic_common.c'))
specific_ss.add(when: 'CONFIG_ARM_GIC', if_true: files('arm_gicv3_cpuif_common.c'))
specific_ss.add(when: 'CONFIG_ARM_GICV3_TCG', if_true: files('arm_gicv3_cpuif.c'))
specific_ss.add(when: 'CONFIG_ARM_GIC_KVM', if_true: files('arm_gic_kvm.c'))
specific_ss.add(when: ['CONFIG_ARM_GIC_KVM', 'TARGET_AARCH64'], if_true: files('arm_gicv3_kvm.c', 'arm_gicv3_its_kvm.c'))
specific_ss.add(when: 'CONFIG_ARM_V7M', if_true: files('armv7m_nvic.c'))
specific_ss.add(when: 'CONFIG_GRLIB', if_true: files('grlib_irqmp.c'))
specific_ss.add(when: 'CONFIG_IOAPIC', if_true: files('ioapic.c'))
specific_ss.add(when: 'CONFIG_LOONGSON_LIOINTC', if_true: files('loongson_liointc.c'))
specific_ss.add(when: 'CONFIG_MIPS_CPS', if_true: files('mips_gic.c'))
specific_ss.add(when: 'CONFIG_OMPIC', if_true: files('ompic.c'))
specific_ss.add(when: ['CONFIG_KVM', 'CONFIG_OPENPIC'],
		if_true: files('openpic_kvm.c'))
specific_ss.add(when: 'CONFIG_POWERNV', if_true: files('xics_pnv.c', 'pnv_xive.c', 'pnv_xive2.c'))
specific_ss.add(when: 'CONFIG_PPC_UIC', if_true: files('ppc-uic.c'))
specific_ss.add(when: 'CONFIG_RX_ICU', if_true: files('rx_icu.c'))
specific_ss.add(when: 'CONFIG_S390_FLIC', if_true: files('s390_flic.c'))
specific_ss.add(when: 'CONFIG_S390_FLIC_KVM', if_true: files('s390_flic_kvm.c'))
specific_ss.add(when: 'CONFIG_SH_INTC', if_true: files('sh_intc.c'))
specific_ss.add(when: 'CONFIG_RISCV_ACLINT', if_true: files('riscv_aclint.c'))
specific_ss.add(when: 'CONFIG_RISCV_APLIC', if_true: files('riscv_aplic.c'))
specific_ss.add(when: 'CONFIG_RISCV_IMSIC', if_true: files('riscv_imsic.c'))
specific_ss.add(when: 'CONFIG_SIFIVE_PLIC', if_true: files('sifive_plic.c'))
specific_ss.add(when: 'CONFIG_XICS', if_true: files('xics.c', 'xive2.c'))
specific_ss.add(when: ['CONFIG_KVM', 'CONFIG_XICS'],
		if_true: files('xics_kvm.c'))
specific_ss.add(when: 'CONFIG_PSERIES', if_true: files('xics_spapr.c', 'spapr_xive.c'))
specific_ss.add(when: 'CONFIG_XIVE', if_true: files('xive.c'))
specific_ss.add(when: ['CONFIG_KVM', 'CONFIG_XIVE'],
		if_true: files('spapr_xive_kvm.c'))
specific_ss.add(when: 'CONFIG_M68K_IRQC', if_true: files('m68k_irqc.c'))
specific_ss.add(when: 'CONFIG_LOONGSON_IPI_COMMON', if_true: files('loongson_ipi_common.c'))
specific_ss.add(when: 'CONFIG_LOONGSON_IPI', if_true: files('loongson_ipi.c'))
specific_ss.add(when: 'CONFIG_LOONGARCH_IPI', if_true: files('loongarch_ipi.c'))
specific_ss.add(when: 'CONFIG_LOONGARCH_PCH_PIC', if_true: files('loongarch_pch_pic.c'))
specific_ss.add(when: 'CONFIG_LOONGARCH_PCH_MSI', if_true: files('loongarch_pch_msi.c'))
specific_ss.add(when: 'CONFIG_LOONGARCH_EXTIOI', if_true: files('loongarch_extioi.c'))
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_inth.c'))

===== ./hw/intc/calypso_inth.c =====
/*
 * calypso_inth.c — Calypso INTH (Interrupt Handler)
 *
 * Level-sensitive interrupt controller at 0xFFFFFA00.
 * 32 IRQ inputs, priority-based arbitration, IRQ/FIQ routing via ILR.
 *
 * The Calypso INTH is LEVEL-SENSITIVE: it tracks the current level of
 * each input line. When a peripheral deasserts its IRQ (e.g. UART clears
 * TX_EMPTY by reading IIR), the INTH immediately sees the change and
 * lowers its output if no other active interrupts remain.
 *
 * The firmware does NOT use IRQ_CTRL to acknowledge interrupts — it
 * relies on the peripheral clearing its interrupt source instead.
 *
 * Register map (16-bit, offsets from base):
 *   0x00        IT_REG1   (active bits [15:0], read-only)
 *   0x02        IT_REG2   (active bits [31:16], read-only)
 *   0x04        MASK_IT_REG1 (mask low)
 *   0x06        MASK_IT_REG2 (mask high)
 *   0x20..0x5F  ILR[0..31] (2 bytes each: bits[4:0]=prio, bit[8]=FIQ)
 *   0x10        IRQ_NUM   (current IRQ number, read-only)
 *   0x12        FIQ_NUM   (current FIQ number, read-only)
 *   0x14        IRQ_CTRL  (write to acknowledge — kept for compat)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "hw/arm/calypso/calypso_inth.h"

/* ---- Priority arbitration ---- */

static void calypso_inth_update(CalypsoINTHState *s)
{
    uint32_t active = s->levels & ~s->mask;
    int best_irq = -1;
    int best_prio = 0x7F;
    int is_fiq = 0;

    for (int i = 0; i < CALYPSO_INTH_NUM_IRQS; i++) {
        if (active & (1u << i)) {
            int prio = s->ilr[i] & 0x1F;
            if (prio < best_prio) {
                best_prio = prio;
                best_irq = i;
                is_fiq = (s->ilr[i] >> 8) & 1;
            }
        }
    }

    if (best_irq >= 0) {
        if (s->irq_in_service < 0) {
            /* No IRQ in service — present the new highest-priority IRQ */
            s->ith_v = best_irq;
            if (is_fiq) {
                qemu_irq_raise(s->parent_fiq);
                qemu_irq_lower(s->parent_irq);
            } else {
                qemu_irq_raise(s->parent_irq);
                qemu_irq_lower(s->parent_fiq);
            }
        }
        /* While an IRQ is in service, CPU lines stay low (no nesting).
         * New IRQs are latched in levels and will be presented after
         * the firmware writes IRQ_CTRL to end service. */
    } else {
        if (s->irq_in_service < 0) {
            s->ith_v = 0;
        }
        qemu_irq_lower(s->parent_irq);
        qemu_irq_lower(s->parent_fiq);
    }
}

/* ---- GPIO input handler (one per IRQ line) ---- */

static void calypso_inth_set_irq(void *opaque, int irq, int level)
{
    CalypsoINTHState *s = CALYPSO_INTH(opaque);

    /* Level-sensitive: track current input level directly */
    if (level) {
        s->levels |= (1u << irq);
    } else {
        s->levels &= ~(1u << irq);
    }

    calypso_inth_update(s);
}

/* ---- MMIO read/write ---- */

static uint64_t calypso_inth_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoINTHState *s = CALYPSO_INTH(opaque);

    switch (offset) {
    case 0x00: /* IT_REG1 — active bits [15:0] */
        return s->levels & 0xFFFF;
    case 0x02: /* IT_REG2 — active bits [31:16] */
        return (s->levels >> 16) & 0xFFFF;
    case 0x08: /* MASK_IT_REG1 */
        return s->mask & 0xFFFF;
    case 0x0a: /* MASK_IT_REG2 */
        return (s->mask >> 16) & 0xFFFF;
    case 0x10: /* IRQ_NUM — start-of-service: latch the current IRQ */
    case 0x80: /* IRQ_NUM (legacy) */
    {
        uint16_t num = s->ith_v;
        s->irq_in_service = num;
        /* Clear level for edge-like sources that won't self-deassert */
        if (num == 4 || num == 5) {
            s->levels &= ~(1u << num);
        }
        /* Lower CPU IRQ line while ISR runs */
        qemu_irq_lower(s->parent_irq);
        qemu_irq_lower(s->parent_fiq);
        {
            static uint32_t irq_counts[32];
            static uint32_t total = 0;
            if (num < 32) irq_counts[num]++;
            total++;
            if (total <= 20 || total == 100 || total == 500 || total == 1000) {
                fprintf(stderr, "[INTH] IRQ_NUM=%u (#%u) levels=0x%08x mask=0x%08x svc=%d\n",
                        num, total, s->levels, s->mask, s->irq_in_service);
            }
        }
        return num;
    }
    case 0x12: /* FIQ_NUM */
    case 0x82: /* FIQ_NUM (legacy) */
        return s->ith_v;
    case 0x14: /* IRQ_CTRL */
    case 0x84: /* IRQ_CTRL (legacy) */
        return 0;
    default:
        if (offset >= 0x20 && offset < 0x60) {
            int idx = (offset - 0x20) / 2;
            return s->ilr[idx];
        }
        return 0;
    }
}

static void calypso_inth_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    CalypsoINTHState *s = CALYPSO_INTH(opaque);

    switch (offset) {
    case 0x08: /* MASK_IT_REG1 */
        s->mask = (s->mask & 0xFFFF0000) | (value & 0xFFFF);
        calypso_inth_update(s);
        break;
    case 0x0a: /* MASK_IT_REG2 */
        s->mask = (s->mask & 0x0000FFFF) | ((value & 0xFFFF) << 16);
        calypso_inth_update(s);
        break;
    case 0x14: /* IRQ_CTRL — end-of-service: ack the latched IRQ */
    case 0x84:
    {
        static uint32_t ctrl_count = 0;
        ctrl_count++;
        if (ctrl_count <= 20 || ctrl_count == 100 || ctrl_count == 500) {
            fprintf(stderr, "[INTH] IRQ_CTRL write val=0x%04x svc=%d levels=0x%08x\n",
                    (uint16_t)value, s->irq_in_service, s->levels);
        }
        /* Ack the IRQ that was latched on IRQ_NUM read */
        if (s->irq_in_service >= 0 && s->irq_in_service < CALYPSO_INTH_NUM_IRQS) {
            s->levels &= ~(1u << s->irq_in_service);
        }
        s->irq_in_service = -1;
        calypso_inth_update(s);
        break;
    }
    default:
        if (offset >= 0x20 && offset < 0x60) {
            int idx = (offset - 0x20) / 2;
            s->ilr[idx] = value & 0x1FFF;
        }
        break;
    }
}

static const MemoryRegionOps calypso_inth_ops = {
    .read = calypso_inth_read,
    .write = calypso_inth_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
    .impl  = { .min_access_size = 1, .max_access_size = 2 },
};

/* ---- QOM lifecycle ---- */

static void calypso_inth_realize(DeviceState *dev, Error **errp)
{
    CalypsoINTHState *s = CALYPSO_INTH(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_inth_ops, s,
                          "calypso-inth", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /* Two output lines: IRQ and FIQ to CPU */
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->parent_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->parent_fiq);

    /* 32 input IRQ lines */
    qdev_init_gpio_in(dev, calypso_inth_set_irq, CALYPSO_INTH_NUM_IRQS);
}

static void calypso_inth_reset(DeviceState *dev)
{
    CalypsoINTHState *s = CALYPSO_INTH(dev);

    s->levels = 0;
    s->mask = 0x00000000;
    s->ith_v = 0;
    s->irq_in_service = -1;
    memset(s->ilr, 0, sizeof(s->ilr));
}

static void calypso_inth_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = calypso_inth_realize;
    device_class_set_legacy_reset(dc, calypso_inth_reset);
    dc->desc = "Calypso INTH interrupt controller";
}

static const TypeInfo calypso_inth_info = {
    .name          = TYPE_CALYPSO_INTH,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoINTHState),
    .class_init    = calypso_inth_class_init,
};

static void calypso_inth_register_types(void)
{
    type_register_static(&calypso_inth_info);
}

type_init(calypso_inth_register_types)

===== ./hw/char/meson.build =====
system_ss.add(when: 'CONFIG_CADENCE', if_true: files('cadence_uart.c'))
system_ss.add(when: 'CONFIG_CMSDK_APB_UART', if_true: files('cmsdk-apb-uart.c'))
system_ss.add(when: 'CONFIG_ESCC', if_true: files('escc.c'))
system_ss.add(when: 'CONFIG_GRLIB', if_true: files('grlib_apbuart.c'))
system_ss.add(when: 'CONFIG_IBEX', if_true: files('ibex_uart.c'))
system_ss.add(when: 'CONFIG_IMX', if_true: files('imx_serial.c'))
system_ss.add(when: 'CONFIG_IPACK', if_true: files('ipoctal232.c'))
system_ss.add(when: 'CONFIG_ISA_BUS', if_true: files('parallel-isa.c'))
system_ss.add(when: 'CONFIG_ISA_DEBUG', if_true: files('debugcon.c'))
system_ss.add(when: 'CONFIG_NRF51_SOC', if_true: files('nrf51_uart.c'))
system_ss.add(when: 'CONFIG_PARALLEL', if_true: files('parallel.c'))
system_ss.add(when: 'CONFIG_PL011', if_true: files('pl011.c'))
system_ss.add(when: 'CONFIG_SCLPCONSOLE', if_true: files('sclpconsole.c', 'sclpconsole-lm.c'))
system_ss.add(when: 'CONFIG_SERIAL', if_true: files('serial.c'))
system_ss.add(when: 'CONFIG_SERIAL_ISA', if_true: files('serial-isa.c'))
system_ss.add(when: 'CONFIG_SERIAL_MM', if_true: files('serial-mm.c'))
system_ss.add(when: 'CONFIG_SERIAL_PCI', if_true: files('serial-pci.c'))
system_ss.add(when: 'CONFIG_SERIAL_PCI_MULTI', if_true: files('serial-pci-multi.c'))
system_ss.add(when: 'CONFIG_SHAKTI_UART', if_true: files('shakti_uart.c'))
system_ss.add(when: 'CONFIG_VIRTIO_SERIAL', if_true: files('virtio-console.c'))
system_ss.add(when: 'CONFIG_XEN_BUS', if_true: files('xen_console.c'))
system_ss.add(when: 'CONFIG_XILINX', if_true: files('xilinx_uartlite.c'))

system_ss.add(when: 'CONFIG_AVR_USART', if_true: files('avr_usart.c'))
system_ss.add(when: 'CONFIG_COLDFIRE', if_true: files('mcf_uart.c'))
system_ss.add(when: 'CONFIG_DIGIC', if_true: files('digic-uart.c'))
system_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4210_uart.c'))
system_ss.add(when: 'CONFIG_OMAP', if_true: files('omap_uart.c'))
system_ss.add(when: 'CONFIG_RASPI', if_true: files('bcm2835_aux.c'))
system_ss.add(when: 'CONFIG_RENESAS_SCI', if_true: files('renesas_sci.c'))
system_ss.add(when: 'CONFIG_SIFIVE_UART', if_true: files('sifive_uart.c'))
system_ss.add(when: 'CONFIG_SH_SCI', if_true: files('sh_serial.c'))
system_ss.add(when: 'CONFIG_STM32F2XX_USART', if_true: files('stm32f2xx_usart.c'))
system_ss.add(when: 'CONFIG_STM32L4X5_USART', if_true: files('stm32l4x5_usart.c'))
system_ss.add(when: 'CONFIG_MCHP_PFSOC_MMUART', if_true: files('mchp_pfsoc_mmuart.c'))
system_ss.add(when: 'CONFIG_HTIF', if_true: files('riscv_htif.c'))
system_ss.add(when: 'CONFIG_GOLDFISH_TTY', if_true: files('goldfish_tty.c'))

specific_ss.add(when: 'CONFIG_TERMINAL3270', if_true: files('terminal3270.c'))
specific_ss.add(when: 'CONFIG_VIRTIO', if_true: files('virtio-serial-bus.c'))
specific_ss.add(when: 'CONFIG_PSERIES', if_true: files('spapr_vty.c'))
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_uart.c'))

===== ./hw/char/calypso_uart.c =====
/*
 * calypso_uart.c — Calypso UART
 *
 * Pragmatic emulation for the Compal/Calypso loader path:
 *  - strict 8-bit MMIO accesses
 *  - banked registers via LCR[7] / LCR==0xBF
 *  - SCR / SSR implemented
 *  - RX FIFO with verbose debug
 *  - raw RX/TX dumps to /tmp/qemu-*.raw
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "chardev/char-fe.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/sercomm_gate.h"

/* Register offsets */
#define REG_RBR_THR   0x00
#define REG_IER       0x01
#define REG_IIR_FCR   0x02
#define REG_LCR       0x03
#define REG_MCR       0x04
#define REG_LSR       0x05
#define REG_MSR       0x06
#define REG_SPR       0x07
#define REG_MDR1      0x08
#define REG_SCR       0x10
#define REG_SSR       0x11

/* IER bits */
#define IER_RX_DATA   (1 << 0)
#define IER_TX_EMPTY  (1 << 1)
#define IER_RX_LINE   (1 << 2)

/* IIR values */
#define IIR_NO_INT    0x01
#define IIR_RX_LINE   0x06
#define IIR_RX_DATA   0x04
#define IIR_TX_EMPTY  0x02

/* LCR bits */
#define LCR_DLAB      (1 << 7)
#define LCR_CONF_BF   0xBF

/* LSR bits */
#define LSR_DR        (1 << 0)
#define LSR_OE        (1 << 1)
#define LSR_THRE      (1 << 5)
#define LSR_TEMT      (1 << 6)

/* MSR bits */
#define MSR_CTS       (1 << 4)
#define MSR_DSR       (1 << 5)
#define MSR_DCD       (1 << 7)

/* FCR bits */
#define FCR_FIFO_EN   (1 << 0)
#define FCR_RX_RESET  (1 << 1)
#define FCR_TX_RESET  (1 << 2)

/* SSR bits (minimal model) */
#define SSR_TX_FIFO_FULL  (1 << 0)

/**
 * uart_log_raw - Log raw UART data to a file
 * @path: Path to the log file
 * @buf: Buffer containing the data
 * @len: Length of the data
 *
 * Appends binary data to the specified file. Used for debugging
 * modem and IrDA traffic. Silently ignores errors.
 */
static void uart_log_raw(const char *path, const uint8_t *buf, size_t len)
{
    FILE *f = fopen(path, "ab");
    if (!f) {
        return;
    }

    fwrite(buf, 1, len, f);
    fclose(f);
}

/* ---- FIFO helpers ---- */

/**
 * fifo_reset - Reset the RX FIFO state
 * @s: UART device state
 */
static void fifo_reset(CalypsoUARTState *s)
{
    s->rx_head = 0;
    s->rx_tail = 0;
    s->rx_count = 0;
}

/**
 * fifo_push - Push a byte into the RX FIFO
 * @s: UART device state
 * @data: Byte to push
 *
 * Sets overrun error flag if FIFO is full.
 */
void calypso_uart_fifo_push(CalypsoUARTState *s, uint8_t data)
{
    if (s->rx_count >= CALYPSO_UART_RX_FIFO_SIZE) {
        s->lsr |= LSR_OE;
        fprintf(stderr,
                "[UART:%s] RX FIFO OVERFLOW drop=0x%02x count=%u size=%u\n",
                s->label ? s->label : "?",
                data,
                (unsigned)s->rx_count,
                (unsigned)CALYPSO_UART_RX_FIFO_SIZE);
        return;
    }

    s->rx_fifo[s->rx_head] = data;
    s->rx_head = (s->rx_head + 1) % CALYPSO_UART_RX_FIFO_SIZE;
    s->rx_count++;
}

/**
 * fifo_pop - Pop a byte from the RX FIFO
 * @s: UART device state
 *
 * Returns: The popped byte, or 0 if FIFO is empty.
 */
static uint8_t fifo_pop(CalypsoUARTState *s)
{
    uint8_t data = 0;

    if (s->rx_count == 0) {
        return 0;
    }

    data = s->rx_fifo[s->rx_tail];
    s->rx_tail = (s->rx_tail + 1) % CALYPSO_UART_RX_FIFO_SIZE;
    s->rx_count--;

    return data;
}

/* ---- IRQ ---- */

static void calypso_uart_update_irq(CalypsoUARTState *s)
{
    uint8_t iir = IIR_NO_INT;
    bool want = false;

    if ((s->ier & IER_RX_LINE) && (s->lsr & LSR_OE)) {
        iir = IIR_RX_LINE;
        want = true;
    } else if ((s->ier & IER_RX_DATA) && (s->lsr & LSR_DR)) {
        iir = IIR_RX_DATA;
        want = true;
    } else if ((s->ier & IER_TX_EMPTY) && s->thr_empty_pending) {
        iir = IIR_TX_EMPTY;
        want = true;
    }

    s->iir = iir;

    /* Force edge transition so INTH always sees the change.
     * After IRQ_CTRL ack clears levels[n], a steady-high line
     * needs a low→high pulse to re-register in the INTH. */
    qemu_irq_lower(s->irq);
    if (want) {
        qemu_irq_raise(s->irq);
    }
}

void calypso_uart_kick_rx(CalypsoUARTState *s)
{
    if (s->rx_count > 0 && (s->lsr & LSR_DR)) {
        /* Force IRQ re-evaluation by pulsing the IRQ line */
        qemu_irq_lower(s->irq);
        calypso_uart_update_irq(s);
    }
}

void calypso_uart_poll_backend(CalypsoUARTState *s)
{
    qemu_chr_fe_accept_input(&s->chr);
}

void calypso_uart_kick_tx(CalypsoUARTState *s)
{
    /* Re-check TX interrupt state — if THR is empty and IER TX enabled,
     * fire the interrupt so firmware can write next byte. */
    calypso_uart_update_irq(s);
}

void calypso_uart_inject_raw(CalypsoUARTState *s, const uint8_t *buf, int len)
{
    if (!s) return;
    for (int i = 0; i < len; i++) {
        if (s->rx_count < CALYPSO_UART_RX_FIFO_SIZE) {
            int idx = (s->rx_head + s->rx_count) % CALYPSO_UART_RX_FIFO_SIZE;
            s->rx_fifo[idx] = buf[i];
            s->rx_count++;
        }
    }
    if (s->rx_count > 0) {
        s->lsr |= LSR_DR;
        calypso_uart_update_irq(s);
    }
}

void calypso_uart_force_init(CalypsoUARTState *s)
{
    /* Force UART into operational state for firmware that gets stuck
     * before completing its own UART init (e.g. trx.highram.elf).
     * Sets MDR1=UART16x, enables RX+TX interrupts. */
    if (s->mdr1 != 0x00) {
        s->mdr1 = 0x00;  /* UART 16x mode */
        s->scr = 0x01;
    }
    s->ier = 0x03;  /* RX + TX interrupts enabled */
    calypso_uart_update_irq(s);
}

/* ---- RX poll timer ----
 * QEMU's chardev backend (PTY) only delivers data during the main event
 * loop. If the ARM CPU runs in a tight loop without yielding, incoming
 * bytes accumulate in the PTY buffer and never reach calypso_uart_receive.
 * This periodic timer forces QEMU to check for pending chardev input. */

#define UART_RX_POLL_NS  (10 * 1000 * 1000)  /* 10 ms */

static void calypso_uart_rx_poll(void *opaque)
{
    CalypsoUARTState *s = (CalypsoUARTState *)opaque;

    /* Kick the main loop to process any pending I/O sources.
     * This is necessary because the CPU may run for long periods
     * without returning to the event loop, starving chardev I/O. */
    qemu_chr_fe_accept_input(&s->chr);
    main_loop_wait(false);  /* non-blocking poll of all I/O sources */

    /* Re-arm (realtime, 50ms) */
    timer_mod(s->rx_poll_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 50);
}

/* ---- Control PTY callbacks ---- */

int calypso_uart_can_receive(void *opaque)
{
    CalypsoUARTState *s = (CalypsoUARTState *)opaque;
    return CALYPSO_UART_RX_FIFO_SIZE - s->rx_count;
}

void calypso_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    CalypsoUARTState *s = (CalypsoUARTState *)opaque;

    /* Debug: only log non-modem UART (modem is too verbose) */
    if (!s->label || strcmp(s->label, "modem") != 0) {
        fprintf(stderr,
                "[UART:%s] <<<RX %d bytes from host (rx_count=%u free=%u):",
                s->label ? s->label : "?",
                size,
                (unsigned)s->rx_count,
                (unsigned)(CALYPSO_UART_RX_FIFO_SIZE - s->rx_count));
        for (int i = 0; i < size && i < 64; i++)
            fprintf(stderr, " %02x", buf[i]);
        if (size > 64) fprintf(stderr, " ...");
        fprintf(stderr, "\n");
    }

    /* Route modem UART via sercomm gate:
     * DLCI 4 (bursts) → BSP emulation (calypso_trx_rx_burst)
     * All others      → re-wrap → FIFO (firmware sercomm layer) */
    if (s->label && !strcmp(s->label, "modem")) {
        sercomm_gate_feed(s, buf, size);
    }

    if (s->rx_count > 0) {
        s->lsr |= LSR_DR;
    }

    calypso_uart_update_irq(s);
}

/* ---- MMIO ---- */

static uint64_t calypso_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoUARTState *s = CALYPSO_UART(opaque);
    uint64_t val = 0;

    switch (offset) {
    case REG_RBR_THR:
        if (s->lcr & LCR_DLAB) {
            val = s->dll;
        } else {
            val = fifo_pop(s);

            if (s->rx_count > 0) {
                s->lsr |= LSR_DR;
            } else {
                s->lsr &= ~LSR_DR;
            }

            /* RBR debug: too verbose for modem, skip */

            calypso_uart_update_irq(s);
        }
        break;

    case REG_IER:
        if (s->lcr & LCR_DLAB) {
            val = s->dlh;
        } else {
            val = s->ier;
        }
        break;

    case REG_IIR_FCR:
        if (s->lcr == LCR_CONF_BF) {
            val = s->efr;
        } else {
            val = s->iir;
            if ((s->iir & 0x0F) == IIR_TX_EMPTY) {
                /* TX burst drain: don't clear pending on the first read.
                 * This lets the firmware ISR loop and drain multiple bytes.
                 * Clear only after 2 consecutive reads without a THR write
                 * (meaning the ISR has no more data to send). */
                s->tx_empty_reads++;
                if (s->tx_empty_reads >= 2) {
                    s->thr_empty_pending = false;
                    s->tx_empty_reads = 0;
                    calypso_uart_update_irq(s);
                }
            }
        }
        break;

    case REG_LCR:
        val = s->lcr;
        break;

    case REG_MCR:
        if (s->lcr == LCR_CONF_BF) {
            val = s->xon1;
        } else {
            val = s->mcr;
        }
        break;

    case REG_LSR:
        if (s->lcr == LCR_CONF_BF) {
            val = s->xon2;
        } else {
            val = s->lsr;
            s->lsr &= ~LSR_OE;
        }
        break;

    case REG_MSR:
        if (s->lcr == LCR_CONF_BF) {
            val = s->xoff1;
        } else {
            val = MSR_CTS | MSR_DSR | MSR_DCD;
        }
        break;

    case REG_SPR:
        if (s->lcr == LCR_CONF_BF) {
            val = s->xoff2;
        } else {
            val = s->spr;
        }
        break;

    case REG_MDR1:
        val = s->mdr1;
        break;

    case REG_SCR:
        val = s->scr;
        break;

    case REG_SSR:
        val = s->ssr & ~SSR_TX_FIFO_FULL;
        break;

    default:
        break;
    }

    return val;
}

static void calypso_uart_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    CalypsoUARTState *s = CALYPSO_UART(opaque);

    switch (offset) {
    case REG_RBR_THR:
        if (s->lcr & LCR_DLAB) {
            s->dll = value;
        } else {
            uint8_t ch = (uint8_t)value;

            /* TX logging disabled to reduce noise */

            if (s->label && !strcmp(s->label, "modem")) {
                uart_log_raw("/tmp/qemu-modem-tx.raw", &ch, 1);
            } else if (s->label && !strcmp(s->label, "irda")) {
                uart_log_raw("/tmp/qemu-irda-tx.raw", &ch, 1);
            }

            qemu_chr_fe_write_all(&s->chr, &ch, 1);

            /* Feed TX byte to L1CTL socket (sercomm parser) */
            if (s->label && !strcmp(s->label, "modem")) {
                l1ctl_sock_uart_tx_byte(ch);
            }

            s->lsr |= LSR_THRE | LSR_TEMT;
            s->thr_empty_pending = true;
            s->tx_empty_reads = 0;  /* reset burst counter — ISR wrote a byte */
            calypso_uart_update_irq(s);
        }
        break;

    case REG_IER:
        if (s->lcr & LCR_DLAB) {
            s->dlh = value;
        } else {
            uint8_t old = s->ier;
            s->ier = value & 0x0F;

            if (old != s->ier && s->label && strcmp(s->label, "modem") != 0) {
                fprintf(stderr, "[UART:%s] IER=0x%02x (RX=%d TX=%d)\n",
                        s->label ? s->label : "?",
                        s->ier,
                        !!(s->ier & IER_RX_DATA),
                        !!(s->ier & IER_TX_EMPTY));
            }

            if (!(old & IER_TX_EMPTY) &&
                (s->ier & IER_TX_EMPTY) &&
                (s->lsr & LSR_THRE)) {
                s->thr_empty_pending = true;
            }

            calypso_uart_update_irq(s);
        }
        break;

    case REG_IIR_FCR:
        if (s->lcr == LCR_CONF_BF) {
            s->efr = value;
        } else {
            s->fcr = value;

            if (value & FCR_RX_RESET) {
                if (s->rx_count > 0) {
                    fprintf(stderr, "[UART:%s] FCR_RX_RESET with %u bytes in FIFO!\n",
                            s->label ? s->label : "?", (unsigned)s->rx_count);
                }
                fifo_reset(s);
                s->lsr &= ~LSR_DR;
            }

            if (value & FCR_TX_RESET) {
                s->thr_empty_pending = false;
                s->lsr |= LSR_THRE | LSR_TEMT;
            }

            calypso_uart_update_irq(s);
        }
        break;

    case REG_LCR:
        s->lcr = value;
        break;

    case REG_MCR:
        if (s->lcr == LCR_CONF_BF) {
            s->xon1 = value;
        } else {
            s->mcr = value;
        }
        break;

    case REG_LSR:
        if (s->lcr == LCR_CONF_BF) {
            s->xon2 = value;
        }
        break;

    case REG_MSR:
        if (s->lcr == LCR_CONF_BF) {
            s->xoff1 = value;
        }
        break;

    case REG_SPR:
        if (s->lcr == LCR_CONF_BF) {
            s->xoff2 = value;
        } else {
            s->spr = value;
        }
        break;

    case REG_MDR1:
        s->mdr1 = value;
        /* MDR1 write — UART mode select. Stub console functions. */
        calypso_stub_console();
        fprintf(stderr, "[UART:%s] MDR1=0x%02x\n",
                s->label ? s->label : "?",
                (unsigned)value);
        break;

    case REG_SCR:
        s->scr = value;
        fprintf(stderr, "[UART:%s] SCR=0x%02x\n",
                s->label ? s->label : "?",
                (unsigned)value);
        break;

    case REG_SSR:
        s->ssr = value;
        break;

    default:
        break;
    }
}

static const MemoryRegionOps calypso_uart_ops = {
    .read = calypso_uart_read,
    .write = calypso_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
    .valid = { .min_access_size = 1, .max_access_size = 1 },
};

/* ---- QOM ---- */

static void calypso_uart_realize(DeviceState *dev, Error **errp)
{
    CalypsoUARTState *s = CALYPSO_UART(dev);
    bool connected;

    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_uart_ops, s,
                          "calypso-uart", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    connected = qemu_chr_fe_backend_connected(&s->chr);

    fprintf(stderr, "### UART PATCH ACTIVE ###\n");
    fprintf(stderr, "[UART:%s] realize: chardev %s\n",
            s->label ? s->label : "?",
            connected ? "CONNECTED" : "NONE");

    if (connected) {
        qemu_chr_fe_set_handlers(&s->chr,
                                 calypso_uart_can_receive,
                                 calypso_uart_receive,
                                 NULL, NULL,
                                 s,
                                 NULL, true);
        fprintf(stderr, "[UART:%s] handlers installed, opaque=%p\n",
                s->label ? s->label : "?",
                (void *)s);

        /* Start RX poll timer using REALTIME clock to force the CPU to
         * yield and process chardev I/O from the PTY backend. */
        s->rx_poll_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                        calypso_uart_rx_poll, s);
        timer_mod(s->rx_poll_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 10);
    }

}

static void calypso_uart_reset_state(DeviceState *dev)
{
    CalypsoUARTState *s = CALYPSO_UART(dev);

    s->ier = 0;
    s->iir = IIR_NO_INT;
    s->fcr = 0;
    s->lcr = 0;
    s->mcr = 0;
    s->lsr = LSR_THRE | LSR_TEMT;
    s->msr = MSR_CTS | MSR_DSR | MSR_DCD;
    s->spr = 0;
    s->dll = 0;
    s->dlh = 0;
    s->mdr1 = 0;

    s->efr = 0;
    s->xon1 = 0;
    s->xon2 = 0;
    s->xoff1 = 0;
    s->xoff2 = 0;
    s->scr = 0;
    s->ssr = 0;

    s->thr_empty_pending = false;

    fifo_reset(s);
}

static Property calypso_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", CalypsoUARTState, chr),
    DEFINE_PROP_STRING("label", CalypsoUARTState, label),
    DEFINE_PROP_END_OF_LIST(),
};

static void calypso_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = calypso_uart_realize;
    device_class_set_legacy_reset(dc, calypso_uart_reset_state);
    dc->desc = "Calypso UART";
    device_class_set_props(dc, calypso_uart_properties);
}

static const TypeInfo calypso_uart_info = {
    .name          = TYPE_CALYPSO_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoUARTState),
    .class_init    = calypso_uart_class_init,
};

static void calypso_uart_register_types(void)
{
    type_register_static(&calypso_uart_info);
}

type_init(calypso_uart_register_types)

===== ./hw/ssi/calypso_spi.c =====
/*
 * calypso_spi.c — Calypso SPI + TWL3025 ABB
 *
 * REWRITE: Correct register map + poweroff blocking.
 *
 * The OsmocomBB loader calls twl3025_power_off() (writes TOGBR1 bit 0)
 * whenever flash_init() fails. In QEMU we block this to keep the
 * loader alive so osmoload can still inject firmware.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "hw/arm/calypso/calypso_spi.h"

/* Register offsets */
#define SPI_REG_SET1     0x00
#define SPI_REG_SET2     0x02
#define SPI_REG_CTRL     0x04
#define SPI_REG_STATUS   0x06
#define SPI_REG_TX_LSB   0x08
#define SPI_REG_TX_MSB   0x0A
#define SPI_REG_RX_LSB   0x0C
#define SPI_REG_RX_MSB   0x0E

/* CTRL bits */
#define SPI_CTRL_START   (1 << 0)

/* ---- TWL3025 ABB SPI transaction ---- */

static uint16_t twl3025_spi_xfer(CalypsoSPIState *s, uint16_t tx)
{
    int read  = (tx >> 15) & 1;
    int addr  = (tx >> 6) & 0x1FF;
    int wdata = tx & 0x3F;

    if (addr >= 256) {
        addr = 0;
    }

    if (read) {
        fprintf(stderr, "[SPI] ABB read  addr=0x%02x → 0x%04x\n",
                addr, s->abb_regs[addr]);
        return s->abb_regs[addr];
    } else {
        fprintf(stderr, "[SPI] ABB write addr=0x%02x data=0x%02x", addr, wdata);

        /* ---- TOGBR1 (0x09): power control toggle ----
         * Bit 0 (TOGB) = power off the phone.
         * The loader calls twl3025_power_off() which writes 1 here
         * whenever flash_init() fails.
         * We BLOCK this to keep the loader alive in QEMU.
         */
        if (addr == ABB_TOGBR1 && (wdata & 0x01)) {
            fprintf(stderr, " *** POWEROFF BLOCKED (TOGBR1 bit 0) ***\n");
            return 0;  /* Don't store, don't poweroff */
        }

        /* ---- TOGBR2 (0x0A): other toggles ---- */
        if (addr == ABB_TOGBR2) {
            fprintf(stderr, " (TOGBR2)\n");
            s->abb_regs[addr] = wdata;
            return 0;
        }

        fprintf(stderr, "\n");

        s->abb_regs[addr] = wdata;

        if (addr == ABB_VRPCDEV) {
            s->abb_regs[ABB_VRPCSTS] = 0x1F;
        }
        return 0;
    }
}

/* ---- MMIO read ---- */

static uint64_t calypso_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoSPIState *s = CALYPSO_SPI(opaque);

    switch (offset) {
    case SPI_REG_SET1:
        return s->set1;
    case SPI_REG_SET2:
        return s->set2;
    case SPI_REG_CTRL:
        return s->ctrl;
    case SPI_REG_STATUS:
        return SPI_STATUS_RE;
    case SPI_REG_TX_LSB:
        return s->tx_data & 0xFF;
    case SPI_REG_TX_MSB:
        return (s->tx_data >> 8) & 0xFF;
    case SPI_REG_RX_LSB:
        return s->rx_data & 0xFF;
    case SPI_REG_RX_MSB:
        return (s->rx_data >> 8) & 0xFF;
    default:
        qemu_log_mask(LOG_UNIMP, "calypso-spi: read at 0x%02x\n",
                       (unsigned)offset);
        return 0;
    }
}

/* ---- MMIO write ---- */

static void calypso_spi_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    CalypsoSPIState *s = CALYPSO_SPI(opaque);

    switch (offset) {
    case SPI_REG_SET1:
        s->set1 = value & 0xFFFF;
        break;
    case SPI_REG_SET2:
        s->set2 = value & 0xFFFF;
        break;
    case SPI_REG_CTRL:
        s->ctrl = value & 0xFFFF;
        if (value & SPI_CTRL_START) {
            s->rx_data = twl3025_spi_xfer(s, s->tx_data);
            qemu_irq_pulse(s->irq);
        }
        break;
    case SPI_REG_STATUS:
        break;
    case SPI_REG_TX_LSB:
        s->tx_data = (s->tx_data & 0xFF00) | (value & 0xFF);
        break;
    case SPI_REG_TX_MSB:
        s->tx_data = (s->tx_data & 0x00FF) | ((value & 0xFF) << 8);
        break;
    case SPI_REG_RX_LSB:
    case SPI_REG_RX_MSB:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "calypso-spi: write 0x%04x at 0x%02x\n",
                       (unsigned)value, (unsigned)offset);
        break;
    }
}

static const MemoryRegionOps calypso_spi_ops = {
    .read = calypso_spi_read,
    .write = calypso_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* ---- QOM lifecycle ---- */

static void calypso_spi_realize(DeviceState *dev, Error **errp)
{
    CalypsoSPIState *s = CALYPSO_SPI(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_spi_ops, s,
                          "calypso-spi", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static void calypso_spi_reset(DeviceState *dev)
{
    CalypsoSPIState *s = CALYPSO_SPI(dev);

    s->set1 = 0;
    s->set2 = 0;
    s->ctrl = 0;
    s->status = SPI_STATUS_RE;
    s->tx_data = 0;
    s->rx_data = 0;
    memset(s->abb_regs, 0, sizeof(s->abb_regs));

    s->abb_regs[ABB_VRPCSTS] = 0x1F;
    s->abb_regs[ABB_ITSTATREG] = 0x00;
}

static void calypso_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = calypso_spi_realize;
    device_class_set_legacy_reset(dc, calypso_spi_reset);
    dc->desc = "Calypso SPI controller + TWL3025 ABB";
}

static const TypeInfo calypso_spi_info = {
    .name          = TYPE_CALYPSO_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoSPIState),
    .class_init    = calypso_spi_class_init,
};

static void calypso_spi_register_types(void)
{
    type_register_static(&calypso_spi_info);
}

type_init(calypso_spi_register_types)

===== ./hw/ssi/meson.build =====
system_ss.add(when: 'CONFIG_ALLWINNER_A10_SPI', if_true: files('allwinner-a10-spi.c'))
system_ss.add(when: 'CONFIG_ASPEED_SOC', if_true: files('aspeed_smc.c'))
system_ss.add(when: 'CONFIG_MSF2', if_true: files('mss-spi.c'))
system_ss.add(when: 'CONFIG_NPCM7XX', if_true: files('npcm7xx_fiu.c', 'npcm_pspi.c'))
system_ss.add(when: 'CONFIG_PL022', if_true: files('pl022.c'))
system_ss.add(when: 'CONFIG_SIFIVE_SPI', if_true: files('sifive_spi.c'))
system_ss.add(when: 'CONFIG_SSI', if_true: files('ssi.c'))
system_ss.add(when: 'CONFIG_STM32F2XX_SPI', if_true: files('stm32f2xx_spi.c'))
system_ss.add(when: 'CONFIG_XILINX_SPI', if_true: files('xilinx_spi.c'))
system_ss.add(when: 'CONFIG_XILINX_SPIPS', if_true: files('xilinx_spips.c'))
system_ss.add(when: 'CONFIG_XLNX_VERSAL', if_true: files('xlnx-versal-ospi.c'))
system_ss.add(when: 'CONFIG_IMX', if_true: files('imx_spi.c'))
system_ss.add(when: 'CONFIG_IBEX', if_true: files('ibex_spi_host.c'))
system_ss.add(when: 'CONFIG_BCM2835_SPI', if_true: files('bcm2835_spi.c'))
system_ss.add(when: 'CONFIG_PNV_SPI', if_true: files('pnv_spi.c'))
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_spi.c'))
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_i2c.c'))

===== ./hw/ssi/calypso_i2c.c =====
/*
 * Calypso I2C Controller - Minimal stub
 * Returns "ready" immediately to avoid firmware blocking
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"

#define TYPE_CALYPSO_I2C "calypso-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoI2CState, CALYPSO_I2C)

struct CalypsoI2CState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
};

static uint64_t calypso_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset) {
    case 0x04: /* STATUS - always ready */
        return 0x04; /* ARDY (access ready) */
    default:
        return 0;
    }
}

static void calypso_i2c_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    /* Accept all writes silently */
}

static const MemoryRegionOps calypso_i2c_ops = {
    .read = calypso_i2c_read,
    .write = calypso_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

static void calypso_i2c_realize(DeviceState *dev, Error **errp)
{
    CalypsoI2CState *s = CALYPSO_I2C(dev);
    
    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_i2c_ops, s,
                          "calypso-i2c", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void calypso_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = calypso_i2c_realize;
    dc->desc = "Calypso I2C stub";
}

static const TypeInfo calypso_i2c_info = {
    .name          = TYPE_CALYPSO_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoI2CState),
    .class_init    = calypso_i2c_class_init,
};

static void calypso_i2c_register_types(void)
{
    type_register_static(&calypso_i2c_info);
}

type_init(calypso_i2c_register_types)

===== ./hw/timer/meson.build =====
system_ss.add(when: 'CONFIG_A9_GTIMER', if_true: files('a9gtimer.c'))
system_ss.add(when: 'CONFIG_ALLWINNER_A10_PIT', if_true: files('allwinner-a10-pit.c'))
system_ss.add(when: 'CONFIG_ARM_MPTIMER', if_true: files('arm_mptimer.c'))
system_ss.add(when: 'CONFIG_ARM_TIMER', if_true: files('arm_timer.c'))
system_ss.add(when: 'CONFIG_ARM_V7M', if_true: files('armv7m_systick.c'))
system_ss.add(when: 'CONFIG_ASPEED_SOC', if_true: files('aspeed_timer.c'))
system_ss.add(when: 'CONFIG_CADENCE', if_true: files('cadence_ttc.c'))
system_ss.add(when: 'CONFIG_CMSDK_APB_DUALTIMER', if_true: files('cmsdk-apb-dualtimer.c'))
system_ss.add(when: 'CONFIG_CMSDK_APB_TIMER', if_true: files('cmsdk-apb-timer.c'))
system_ss.add(when: 'CONFIG_RENESAS_TMR', if_true: files('renesas_tmr.c'))
system_ss.add(when: 'CONFIG_RENESAS_CMT', if_true: files('renesas_cmt.c'))
system_ss.add(when: 'CONFIG_DIGIC', if_true: files('digic-timer.c'))
system_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4210_mct.c'))
system_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4210_pwm.c'))
system_ss.add(when: 'CONFIG_GRLIB', if_true: files('grlib_gptimer.c'))
system_ss.add(when: 'CONFIG_HPET', if_true: files('hpet.c'))
system_ss.add(when: 'CONFIG_I8254', if_true: files('i8254_common.c', 'i8254.c'))
system_ss.add(when: 'CONFIG_IMX', if_true: files('imx_epit.c'))
system_ss.add(when: 'CONFIG_IMX', if_true: files('imx_gpt.c'))
system_ss.add(when: 'CONFIG_MIPS_CPS', if_true: files('mips_gictimer.c'))
system_ss.add(when: 'CONFIG_MSF2', if_true: files('mss-timer.c'))
system_ss.add(when: 'CONFIG_NPCM7XX', if_true: files('npcm7xx_timer.c'))
system_ss.add(when: 'CONFIG_NRF51_SOC', if_true: files('nrf51_timer.c'))
system_ss.add(when: 'CONFIG_PXA2XX_TIMER', if_true: files('pxa2xx_timer.c'))
system_ss.add(when: 'CONFIG_RASPI', if_true: files('bcm2835_systmr.c'))
system_ss.add(when: 'CONFIG_SH_TIMER', if_true: files('sh_timer.c'))
system_ss.add(when: 'CONFIG_SLAVIO', if_true: files('slavio_timer.c'))
system_ss.add(when: 'CONFIG_SSE_COUNTER', if_true: files('sse-counter.c'))
system_ss.add(when: 'CONFIG_SSE_TIMER', if_true: files('sse-timer.c'))
system_ss.add(when: 'CONFIG_STELLARIS_GPTM', if_true: files('stellaris-gptm.c'))
system_ss.add(when: 'CONFIG_STM32F2XX_TIMER', if_true: files('stm32f2xx_timer.c'))
system_ss.add(when: 'CONFIG_XILINX', if_true: files('xilinx_timer.c'))
specific_ss.add(when: 'CONFIG_IBEX', if_true: files('ibex_timer.c'))
system_ss.add(when: 'CONFIG_SIFIVE_PWM', if_true: files('sifive_pwm.c'))

specific_ss.add(when: 'CONFIG_AVR_TIMER16', if_true: files('avr_timer16.c'))
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_timer.c'))

===== ./hw/timer/calypso_timer.c =====
/*
 * calypso_timer.c — Calypso GP/Watchdog Timer
 *
 * 16-bit down-counter with auto-reload, prescaler, and IRQ.
 * Calypso base clock: 13 MHz. Effective rate = 13 MHz / (prescaler + 1).
 *
 * Register map (16-bit, offsets from base):
 *   0x00  CNTL       Control (bit0=start, bit1=auto-reload, bit2=irq-enable)
 *   0x02  LOAD       Reload value (written before starting)
 *   0x04  READ       Current count (read-only)
 *   0x06  PRESCALER  Clock divider
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "hw/arm/calypso/calypso_timer.h"

#define TIMER_CTRL_START   (1 << 0)
#define TIMER_CTRL_RELOAD  (1 << 1)
#define TIMER_CTRL_IRQ_EN  (1 << 2)

#define CALYPSO_BASE_CLK   13000000LL  /* 13 MHz */

static void calypso_timer_tick(void *opaque)
{
    CalypsoTimerState *s = CALYPSO_TIMER(opaque);

    if (!s->running) {
        return;
    }

    s->count--;
    if (s->count == 0) {
        /* Fire IRQ if enabled */
        if (s->ctrl & TIMER_CTRL_IRQ_EN) {
            qemu_irq_raise(s->irq);
        }
        /* Auto-reload or stop */
        if (s->ctrl & TIMER_CTRL_RELOAD) {
            s->count = s->load;
        } else {
            s->running = false;
            return;
        }
    }

    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->tick_ns);
}

static void calypso_timer_start(CalypsoTimerState *s)
{
    if (s->load == 0) {
        return;
    }
    s->count = s->load;
    s->running = true;
    int64_t freq = CALYPSO_BASE_CLK / (s->prescaler + 1);
    s->tick_ns = NANOSECONDS_PER_SECOND / freq;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->tick_ns);
}

/* ---- MMIO ---- */

static uint64_t calypso_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTimerState *s = CALYPSO_TIMER(opaque);

    switch (offset) {
    case 0x00: return s->ctrl;
    case 0x02: return s->load;
    case 0x04: return s->count;
    case 0x06: return s->prescaler;
    default:   return 0;
    }
}

static void calypso_timer_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    CalypsoTimerState *s = CALYPSO_TIMER(opaque);

    switch (offset) {
    case 0x00: /* CNTL */
        s->ctrl = value & 0x07;
        if (value & TIMER_CTRL_START) {
            calypso_timer_start(s);
        } else {
            s->running = false;
            timer_del(s->timer);
        }
        break;
    case 0x02: /* LOAD */
        s->load = value;
        break;
    case 0x06: /* PRESCALER */
        s->prescaler = value;
        break;
    }
}

static const MemoryRegionOps calypso_timer_ops = {
    .read = calypso_timer_read,
    .write = calypso_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* ---- QOM lifecycle ---- */

static void calypso_timer_realize(DeviceState *dev, Error **errp)
{
    CalypsoTimerState *s = CALYPSO_TIMER(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_timer_ops, s,
                          "calypso-timer", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_timer_tick, s);
}

static void calypso_timer_reset(DeviceState *dev)
{
    CalypsoTimerState *s = CALYPSO_TIMER(dev);

    s->load = 0;
    s->count = 0;
    s->ctrl = 0;
    s->prescaler = 0;
    s->running = false;
    timer_del(s->timer);
}

static void calypso_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = calypso_timer_realize;
    device_class_set_legacy_reset(dc, calypso_timer_reset);
    dc->desc = "Calypso GP/Watchdog timer";
}

static const TypeInfo calypso_timer_info = {
    .name          = TYPE_CALYPSO_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoTimerState),
    .class_init    = calypso_timer_class_init,
};

static void calypso_timer_register_types(void)
{
    type_register_static(&calypso_timer_info);
}

type_init(calypso_timer_register_types)

===== ./l1ctl_passthrough.py =====
#!/usr/bin/env python3
"""
l1ctl_passthrough.py — Relay between mobile (layer23) and QEMU firmware.

Synthesizes RESET_CONF/RESET_IND (firmware does not implement L1CTL reset).
Everything else is pure relay to/from the firmware.

Usage:
  python3 l1ctl_passthrough.py /dev/pts/X [/tmp/osmocom_l2_ms]
"""

import errno, fcntl, os, select, signal, socket, struct, sys, termios, time

DLCI_L1CTL   = 5
DLCI_DEBUG   = 4
DLCI_CONSOLE = 10
FLAG         = 0x7E
ESCAPE       = 0x7D
ESCAPE_XOR   = 0x20
L1CTL_SOCK   = "/tmp/osmocom_l2"

L1CTL_RESET_IND  = 7
L1CTL_RESET_REQ  = 8
L1CTL_RESET_CONF = 9
L1CTL_PM_REQ     = 11
L1CTL_PM_CONF    = 12

L1CTL_NAMES = {
    1:"FBSB_REQ", 2:"FBSB_CONF", 3:"DATA_IND", 4:"RACH_REQ", 5:"RACH_CONF",
    6:"DATA_REQ", 7:"RESET_IND", 8:"RESET_REQ", 9:"RESET_CONF", 10:"DATA_CONF",
    11:"PM_REQ", 12:"PM_CONF", 13:"ECHO_REQ", 14:"ECHO_CONF",
    18:"CCCH_MODE_REQ", 19:"CCCH_MODE_CONF",
    20:"DM_EST_REQ", 21:"DM_FREQ_REQ", 22:"DM_REL_REQ",
    29:"TCH_MODE_REQ", 30:"TCH_MODE_CONF",
    33:"TRAFFIC_REQ", 34:"TRAFFIC_CONF", 35:"TRAFFIC_IND",
}

def set_raw(fd):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = (attrs[2] & ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)) | termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def set_nonblock(fd):
    fcntl.fcntl(fd, fcntl.F_SETFL, fcntl.fcntl(fd, fcntl.F_GETFL) | os.O_NONBLOCK)

def sercomm_escape(data):
    out = bytearray()
    for b in data:
        if b in (FLAG, ESCAPE):
            out.append(ESCAPE)
            out.append(b ^ ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)

def sercomm_wrap(dlci, payload):
    return bytes(bytearray([FLAG]) + sercomm_escape(bytes([dlci, 0x03]) + payload) + bytearray([FLAG]))

def l1ctl_send(client, msg_type, payload=b""):
    hdr = struct.pack("BBxx", msg_type, 0)
    msg = hdr + payload
    frame = struct.pack(">H", len(msg)) + msg
    try:
        client.sendall(frame)
    except Exception:
        pass

class SercommParser:
    def __init__(self):
        self.buf = bytearray()
        self.in_frame = False
        self.escape = False

    def feed(self, data):
        frames = []
        for b in data:
            if b == FLAG:
                if self.in_frame and len(self.buf) >= 2:
                    frames.append((self.buf[0], bytes(self.buf[2:])))
                self.buf.clear()
                self.in_frame = True
                self.escape = False
            elif self.in_frame:
                if self.escape:
                    self.buf.append(b ^ ESCAPE_XOR)
                    self.escape = False
                elif b == ESCAPE:
                    self.escape = True
                else:
                    self.buf.append(b)
        return frames

class LengthPrefixReader:
    def __init__(self):
        self.buf = b""

    def feed(self, data):
        self.buf += data
        msgs = []
        while len(self.buf) >= 2:
            mlen = struct.unpack(">H", self.buf[:2])[0]
            if len(self.buf) < 2 + mlen:
                break
            msgs.append(self.buf[2:2 + mlen])
            self.buf = self.buf[2 + mlen:]
        return msgs

def main():
    if len(sys.argv) < 2:
        print("Usage: %s /dev/pts/X [socket_path]" % sys.argv[0])
        sys.exit(1)

    pty_path = sys.argv[1]
    sock_path = sys.argv[2] if len(sys.argv) > 2 else L1CTL_SOCK

    try:
        os.unlink(sock_path)
    except FileNotFoundError:
        pass

    pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
    set_raw(pty_fd)
    set_nonblock(pty_fd)
    print("l1ctl-passthrough: PTY=%s" % pty_path, flush=True)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(sock_path)
    srv.listen(1)
    srv.setblocking(False)
    print("l1ctl-passthrough: listening on %s" % sock_path, flush=True)

    client = None
    parser = SercommParser()
    reader = LengthPrefixReader()
    tx_count = 0
    rx_count = 0
    running = True

    def shutdown(_s, _f):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    while running:
        fds = [pty_fd, srv]
        if client:
            fds.append(client)
        try:
            readable, _, _ = select.select(fds, [], [], 0.5)
        except (ValueError, OSError):
            if client:
                client.close()
                client = None
            continue

        for fd in readable:
            # New mobile connection
            if fd is srv:
                if client:
                    client.close()
                client, _ = srv.accept()
                client.setblocking(False)
                reader = LengthPrefixReader()
                print("l1ctl-passthrough: mobile connected", flush=True)
                # Send RESET_IND(BOOT) on connect — firmware doesn't send it
                l1ctl_send(client, L1CTL_RESET_IND, struct.pack("Bxxx", 0))
                print("  [synth] -> RESET_IND(BOOT)", flush=True)
                continue

            # mobile -> firmware
            if fd is client:
                try:
                    chunk = client.recv(4096)
                except Exception:
                    chunk = b""
                if not chunk:
                    print("l1ctl-passthrough: mobile disconnected", flush=True)
                    client.close()
                    client = None
                    continue

                for msg in reader.feed(chunk):
                    if len(msg) < 1:
                        continue
                    msg_type = msg[0]
                    name = L1CTL_NAMES.get(msg_type, "0x%02x" % msg_type)
                    print("  [mobile->] %s len=%d" % (name, len(msg)), flush=True)

                    # Handle ECHO locally
                    if msg_type == 13:  # ECHO_REQ
                        l1ctl_send(client, 14, msg[4:])  # ECHO_CONF
                        continue

                    # Synthesize RESET_CONF
                    if msg_type == L1CTL_RESET_REQ:
                        reset_type = msg[4] if len(msg) > 4 else 1
                        l1ctl_send(client, L1CTL_RESET_CONF, struct.pack("Bxxx", reset_type))
                        print("  [synth] RESET_REQ type=%d -> RESET_CONF" % reset_type, flush=True)
                        # Don't forward RESET_REQ to firmware — it doesn't handle it
                        continue

                    # Synthesize PM_CONF (firmware doesn't handle L1CTL PM_REQ)
                    if msg_type == L1CTL_PM_REQ and len(msg) >= 12:
                        arfcn_from = (msg[8] << 8) | msg[9]
                        arfcn_to = (msg[10] << 8) | msg[11]
                        TARGET_ARFCN = 100
                        # Send in small batches (max 10 entries per PM_CONF)
                        batch = b""
                        count = 0
                        for a in range(arfcn_from, arfcn_to + 1):
                            rxlev = 48 if a == TARGET_ARFCN else 0
                            batch += struct.pack(">HBB", a, rxlev, rxlev)
                            count += 1
                            if count >= 10:
                                l1ctl_send(client, L1CTL_PM_CONF, batch)
                                batch = b""
                                count = 0
                        if batch:
                            l1ctl_send(client, L1CTL_PM_CONF, batch)
                        print("  [synth] PM_REQ %d-%d -> PM_CONF" % (arfcn_from, arfcn_to), flush=True)
                        continue

                    # Relay everything else to firmware
                    frame = sercomm_wrap(DLCI_L1CTL, msg)
                    try:
                        os.write(pty_fd, frame)
                    except Exception:
                        pass
                    tx_count += 1
                    if tx_count <= 50 or tx_count % 100 == 0:
                        hexd = " ".join("%02x" % b for b in msg[:16])
                        print("  [mobile->fw] #%d %s len=%d [%s]" % (tx_count, name, len(msg), hexd), flush=True)
                continue

            # firmware -> mobile
            if fd == pty_fd:
                try:
                    chunk = os.read(pty_fd, 4096)
                except Exception:
                    continue
                if not chunk:
                    continue

                for dlci, payload in parser.feed(chunk):
                    if dlci == DLCI_L1CTL and client and payload and len(payload) >= 4:
                        # Validate: L1CTL messages are at most ~200 bytes
                        if len(payload) > 512:
                            print("  [fw->mobile] SKIP oversized %d bytes" % len(payload), flush=True)
                            continue
                        # Skip ECHO_CONF from firmware — mobile doesn't expect it back
                        if payload[0] == 14:  # ECHO_CONF
                            continue
                        frame = struct.pack(">H", len(payload)) + payload
                        try:
                            client.sendall(frame)
                        except Exception:
                            pass
                        rx_count += 1
                        name = L1CTL_NAMES.get(payload[0], "0x%02x" % payload[0]) if payload else "?"
                        if rx_count <= 50 or rx_count % 100 == 0:
                            hexd = " ".join("%02x" % b for b in payload[:16])
                            print("  [fw->mobile] #%d %s len=%d [%s]" % (rx_count, name, len(payload), hexd), flush=True)
                    elif dlci in (DLCI_CONSOLE, DLCI_DEBUG):
                        try:
                            sys.stderr.buffer.write(payload)
                            sys.stderr.flush()
                        except Exception:
                            pass
                continue

    print("l1ctl-passthrough: done (tx=%d rx=%d)" % (tx_count, rx_count), flush=True)

if __name__ == "__main__":
    main()

===== ./run_calypso.sh =====
#!/bin/bash
# Run Calypso GSM stack in Docker container osmo-operator-1
# Usage: docker exec -it osmo-operator-1 bash /opt/GSM/qemu/run_calypso.sh

set -e

FW=/opt/GSM/firmware/board/compal_e88
QEMU=/opt/GSM/qemu/build/qemu-system-arm
BRIDGE=/opt/GSM/qemu/l1ctl_bridge.py
XCVR=/opt/GSM/osmocom-bb-transceiver/src/host/layer23/src/transceiver/transceiver

# Kill previous
pkill -9 -f qemu-system-arm 2>/dev/null || true
pkill -9 -f transceiver 2>/dev/null || true
pkill -9 -f l1ctl_bridge 2>/dev/null || true
pkill -9 -f "mobile " 2>/dev/null || true
pkill -9 -f osmo-bts-trx 2>/dev/null || true
rm -f /tmp/osmocom_l2* /tmp/qemu-*.sock
tmux kill-session -t calypso 2>/dev/null || true
sleep 1

# Create tmux session
tmux new-session -d -s calypso -n trx0
tmux split-window -t calypso:trx0 -h
tmux new-window -t calypso -n mob
tmux split-window -t calypso:mob -h
tmux new-window -t calypso -n stack
tmux split-window -t calypso:stack -v
tmux split-window -t calypso:stack.1 -v

# Window layout:
#   trx0:  [QEMU TRX0 | Bridge TRX0]
#   mob:   [QEMU Mobile | Bridge Mobile]
#   stack: [Transceiver | BTS | Mobile app]

echo "Starting QEMU TRX0..."
tmux send-keys -t calypso:trx0.0 \
  "CALYPSO_TRX_PORT=0 CALYPSO_AIR_LOCAL=4800 CALYPSO_AIR_PEER=4801 $QEMU -M calypso -cpu arm946 -serial pty -serial pty -monitor unix:/tmp/qemu-trx0.sock,server,nowait -kernel $FW/trx.highram.elf" Enter
sleep 4

echo "Starting QEMU Mobile..."
tmux send-keys -t calypso:mob.0 \
  "CALYPSO_TRX_PORT=0 CALYPSO_AIR_LOCAL=4801 CALYPSO_AIR_PEER=4800 $QEMU -M calypso -cpu arm946 -serial pty -serial pty -monitor unix:/tmp/qemu-mobile.sock,server,nowait -kernel $FW/layer1.highram.elf" Enter
sleep 4

# Get PTYs
TRX0_S0=$(echo 'info chardev' | socat - UNIX-CONNECT:/tmp/qemu-trx0.sock 2>/dev/null | grep serial0 | grep -o '/dev/pts/[0-9]*')
MOB_S0=$(echo 'info chardev' | socat - UNIX-CONNECT:/tmp/qemu-mobile.sock 2>/dev/null | grep serial0 | grep -o '/dev/pts/[0-9]*')
echo "TRX0 PTY: $TRX0_S0"
echo "Mobile PTY: $MOB_S0"

if [ -z "$TRX0_S0" ] || [ -z "$MOB_S0" ]; then
    echo "ERROR: Could not get PTY paths"
    exit 1
fi

echo "Starting bridges..."
tmux send-keys -t calypso:trx0.1 \
  "cd /opt/GSM/qemu && BRIDGE_AIR_PORT=0 python3 l1ctl_bridge.py $TRX0_S0" Enter
tmux send-keys -t calypso:mob.1 \
  "cd /opt/GSM/qemu && BRIDGE_AIR_PORT=0 python3 l1ctl_bridge.py $MOB_S0 /tmp/osmocom_l2_ms" Enter
sleep 2

echo "Starting transceiver..."
tmux send-keys -t calypso:stack.0 \
  "$XCVR -a 100" Enter
sleep 3

echo "Starting osmo-bts-trx..."
tmux send-keys -t calypso:stack.1 \
  "osmo-bts-trx -c /root/osmo-bts-trx.cfg" Enter
sleep 5

echo "Starting mobile..."
tmux send-keys -t calypso:stack.2 \
  "mobile -c /etc/osmocom/mobile.cfg" Enter
sleep 3

echo ""
echo "=== All started ==="
echo "tmux attach -t calypso"
echo ""
echo "Windows:"
echo "  0:trx0   - QEMU TRX0 | Bridge TRX0"
echo "  1:mob    - QEMU Mobile | Bridge Mobile"
echo "  2:stack  - Transceiver | BTS | Mobile app"

===== ./bridge.py =====
#!/usr/bin/env python3
"""
bridge.py — BTS TRX ↔ QEMU UART sercomm bridge
BTS side:  faketrx clone (CLK/TRXC/TRXD on 5700-5702)
MS side:   UART PTY (sercomm) — reads TX, writes RX
Usage: bridge.py <pty-path>
"""
import errno, fcntl, os, select, signal, socket, struct, sys, termios, threading, time

GSM_HYPERFRAME = 2715648
GSM_FRAME_US = 4615.0
CLK_IND_PERIOD = 102
SERCOMM_FLAG = 0x7E
SERCOMM_ESCAPE = 0x7D
SERCOMM_XOR = 0x20
DLCI_L1CTL = 5
DLCI_BURST = 4  # L1A — burst data

def udp_bind(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.setblocking(False)
    return s

def sercomm_wrap(dlci, payload):
    out = bytearray([SERCOMM_FLAG])
    for b in bytes([dlci, 0x03]) + payload:
        if b in (SERCOMM_FLAG, SERCOMM_ESCAPE):
            out.append(SERCOMM_ESCAPE)
            out.append(b ^ SERCOMM_XOR)
        else:
            out.append(b)
    out.append(SERCOMM_FLAG)
    return bytes(out)

class SercommParser:
    def __init__(self):
        self.buf = bytearray()
    def feed(self, data):
        self.buf.extend(data)
        frames = []
        while True:
            try: s = self.buf.index(SERCOMM_FLAG)
            except ValueError: self.buf.clear(); break
            if s > 0: del self.buf[:s]
            try: e = self.buf.index(SERCOMM_FLAG, 1)
            except ValueError: break
            raw = bytes(self.buf[1:e]); del self.buf[:e+1]
            if not raw: continue
            out = bytearray(); i = 0
            while i < len(raw):
                if raw[i] == SERCOMM_ESCAPE and i+1 < len(raw):
                    i += 1; out.append(raw[i] ^ SERCOMM_XOR)
                else: out.append(raw[i])
                i += 1
            if len(out) >= 2: frames.append((out[0], bytes(out[2:])))
        return frames

class Bridge:
    def __init__(self, pty_path, burst_pty_path=None, bts_base=5700):
        # UART PTY (L1CTL — modem)
        self.pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(self.pty_fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 1; attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.pty_fd, termios.TCSANOW, attrs)
        fcntl.fcntl(self.pty_fd, fcntl.F_SETFL,
                    fcntl.fcntl(self.pty_fd, fcntl.F_GETFL) | os.O_NONBLOCK)

        # L1CTL unix socket (mobile ↔ bridge ↔ PTY)
        l1ctl_path = "/tmp/osmocom_l2_1"
        try: os.unlink(l1ctl_path)
        except: pass
        self.l1ctl_srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.l1ctl_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.l1ctl_srv.bind(l1ctl_path)
        self.l1ctl_srv.listen(1)
        self.l1ctl_srv.setblocking(False)
        self.l1ctl_cli = None
        self.l1ctl_buf = b""  # length-prefix accumulator

        # BTS TRX sockets
        self.clk_sock  = udp_bind(bts_base)
        self.trxc_sock = udp_bind(bts_base + 1)
        self.trxd_sock = udp_bind(bts_base + 2)
        self.bts_clk_addr = ("127.0.0.1", bts_base + 100)
        self.trxc_remote = None
        self.trxd_remote = None
        self.powered = False
        self.fn = 0
        self._stop = threading.Event()
        self.parser = SercommParser()
        self.stats = {"clk": 0, "trxc": 0, "dl": 0, "ul": 0}
        print(f"bridge: pty={pty_path} l1ctl={l1ctl_path} CLK={bts_base} TRXC={bts_base+1} TRXD={bts_base+2}", flush=True)

    def start_clock(self):
        threading.Thread(target=self._clock_loop, daemon=True).start()

    def _clock_loop(self):
        tick_ns = int(GSM_FRAME_US * 1000)
        t_next = time.monotonic_ns()
        while not self._stop.is_set():
            t_next += tick_ns
            dt = t_next - time.monotonic_ns()
            if dt > 0: time.sleep(dt / 1e9)
            elif dt < -tick_ns * 10: t_next = time.monotonic_ns()
            self.fn = (self.fn + 1) % GSM_HYPERFRAME
            if self.fn % CLK_IND_PERIOD == 0 and self.powered:
                try:
                    self.clk_sock.sendto(f"IND CLOCK {self.fn}\0".encode(), self.bts_clk_addr)
                    self.stats["clk"] += 1
                except OSError: pass

    def handle_trxc(self):
        try: data, addr = self.trxc_sock.recvfrom(256)
        except OSError: return
        if not data: return
        self.trxc_remote = addr
        raw = data.strip(b'\x00').decode(errors='replace')
        if not raw.startswith("CMD "): return
        parts = raw[4:].split(); verb = parts[0]; args = parts[1:]
        self.stats["trxc"] += 1

        if verb == "POWERON":
            self.powered = True; print(f"BTS: POWERON", flush=True)
            rsp = "RSP POWERON 0"
        elif verb == "POWEROFF":
            self.powered = False; rsp = "RSP POWEROFF 0"
        elif verb == "SETFORMAT":
            rsp = f"RSP SETFORMAT 0 {' '.join(args)}"
        elif verb == "NOMTXPOWER":
            rsp = "RSP NOMTXPOWER 0 50"
        elif verb == "MEASURE":
            freq = args[0] if args else "0"
            rsp = f"RSP MEASURE 0 {freq} -60"
        else:
            rsp = f"RSP {verb} 0 {' '.join(args)}".rstrip()

        self.trxc_sock.sendto((rsp + "\0").encode(), addr)

    def handle_trxd_from_bts(self):
        """BTS DL burst → UDP 6802 → QEMU calypso_trx_rx_burst."""
        try: data, addr = self.trxd_sock.recvfrom(512)
        except OSError: return
        if len(data) < 6: return
        self.trxd_remote = addr
        self.stats["dl"] += 1

        # Forward DL burst as sercomm DLCI 4 on PTY (MS side)
        tn = data[0] & 0x07
        if tn == 0:
            frame = sercomm_wrap(DLCI_BURST, data)
            try: os.write(self.pty_fd, frame)
            except OSError: pass

    def handle_uart_from_qemu(self):
        """QEMU firmware sercomm TX → parse → forward bursts to BTS."""
        try: raw = os.read(self.pty_fd, 4096)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK): return
            raise
        if not raw: return

        for dlci, payload in self.parser.feed(raw):
            if dlci == DLCI_BURST and payload and len(payload) >= 6:
                # Firmware sent a burst → forward as TRXD to BTS
                if self.trxd_remote:
                    try: self.trxd_sock.sendto(payload, self.trxd_remote)
                    except OSError: pass
                    self.stats["ul"] += 1
            elif dlci == DLCI_L1CTL and payload:
                # Firmware sent L1CTL → forward to mobile
                self.handle_l1ctl_to_mobile(dlci, payload)

    def handle_l1ctl_accept(self):
        """Accept new mobile L1CTL client."""
        try:
            cli, _ = self.l1ctl_srv.accept()
            cli.setblocking(False)
        except OSError: return
        if self.l1ctl_cli:
            try: self.l1ctl_cli.close()
            except: pass
        self.l1ctl_cli = cli
        self.l1ctl_buf = b""
        print("bridge: L1CTL client connected", flush=True)
        # Send synthetic RESET_IND via sercomm DLCI 5 to PTY
        reset_ind = bytes([0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        frame = sercomm_wrap(DLCI_L1CTL, reset_ind)
        try: os.write(self.pty_fd, frame)
        except OSError: pass
        # Also send RESET_IND directly to mobile
        hdr = struct.pack(">H", 8)
        try: cli.sendall(hdr + reset_ind)
        except: pass

    def handle_l1ctl_from_mobile(self):
        """Mobile → L1CTL → sercomm DLCI 5 → PTY."""
        if not self.l1ctl_cli: return
        try:
            data = self.l1ctl_cli.recv(4096)
        except OSError: return
        if not data:
            print("bridge: L1CTL client disconnected", flush=True)
            self.l1ctl_cli.close()
            self.l1ctl_cli = None
            return
        self.l1ctl_buf += data
        # Parse length-prefixed L1CTL messages
        while len(self.l1ctl_buf) >= 2:
            msglen = struct.unpack(">H", self.l1ctl_buf[:2])[0]
            if len(self.l1ctl_buf) < 2 + msglen:
                break
            payload = self.l1ctl_buf[2:2+msglen]
            self.l1ctl_buf = self.l1ctl_buf[2+msglen:]
            # Wrap in sercomm DLCI 5 → PTY
            frame = sercomm_wrap(DLCI_L1CTL, payload)
            try: os.write(self.pty_fd, frame)
            except OSError: pass

    def handle_l1ctl_to_mobile(self, dlci, payload):
        """PTY sercomm DLCI 5 → L1CTL length-prefixed → mobile."""
        if not self.l1ctl_cli or not payload: return
        hdr = struct.pack(">H", len(payload))
        try: self.l1ctl_cli.sendall(hdr + payload)
        except OSError:
            print("bridge: L1CTL send error, closing", flush=True)
            self.l1ctl_cli.close()
            self.l1ctl_cli = None

    def run(self):
        self.start_clock()
        running = True
        def shutdown(s, f):
            nonlocal running; running = False
        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)

        while running:
            fds = [self.pty_fd, self.trxc_sock, self.trxd_sock, self.l1ctl_srv]
            if self.l1ctl_cli:
                fds.append(self.l1ctl_cli)
            try:
                readable, _, _ = select.select(fds, [], [], 0.1)
            except (OSError, ValueError) as e:
                print(f"bridge: select error: {e}", flush=True)
                break

            if self.trxc_sock in readable: self.handle_trxc()
            if self.trxd_sock in readable: self.handle_trxd_from_bts()
            if self.pty_fd in readable: self.handle_uart_from_qemu()
            if self.l1ctl_srv in readable: self.handle_l1ctl_accept()
            if self.l1ctl_cli and self.l1ctl_cli in readable: self.handle_l1ctl_from_mobile()

            if self.stats["dl"] > 0 and self.stats["dl"] % 5000 == 0:
                print(f"bridge: clk={self.stats['clk']} dl={self.stats['dl']} ul={self.stats['ul']}", flush=True)

        self._stop.set()
        os.close(self.pty_fd)
        print(f"bridge: clk={self.stats['clk']} trxc={self.stats['trxc']} dl={self.stats['dl']} ul={self.stats['ul']}", flush=True)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <pty-path>"); sys.exit(1)
    burst_pty = sys.argv[2] if len(sys.argv) > 2 else None
    Bridge(sys.argv[1], burst_pty).run()

===== ./l1ctl_test.py =====
#!/usr/bin/env python3
"""
l1ctl_test.py — Direct L1CTL test against QEMU firmware
Connects to /tmp/osmocom_l2_1, sends L1CTL commands, prints responses.
"""
import socket, struct, sys, time

L1CTL_RESET_REQ    = 0x0D
L1CTL_RESET_IND    = 0x07
L1CTL_RESET_CONF   = 0x02
L1CTL_PM_REQ       = 0x0C  # actually varies by version
L1CTL_PM_CONF      = 0x0E
L1CTL_FBSB_REQ     = 0x08
L1CTL_FBSB_CONF    = 0x09
L1CTL_CCCH_MODE_REQ = 0x01
L1CTL_DATA_IND     = 0x03

L1CTL_TYPES = {
    0x01: "CCCH_MODE_REQ", 0x02: "RESET_CONF", 0x03: "DATA_IND",
    0x04: "DATA_REQ", 0x05: "RACH_REQ", 0x06: "RACH_CONF",
    0x07: "RESET_IND", 0x08: "FBSB_REQ", 0x09: "FBSB_CONF",
    0x0C: "PM_REQ", 0x0D: "RESET_REQ", 0x0E: "PM_CONF",
    0x0F: "CCCH_MODE_CONF", 0x10: "DM_EST_REQ",
}

def l1ctl_name(t):
    return L1CTL_TYPES.get(t, f"0x{t:02x}")

def send_msg(s, msg_type, payload=b''):
    data = bytes([msg_type, 0, 0, 0]) + payload
    # Pad to at least 4 bytes
    while len(data) < 4:
        data += b'\x00'
    hdr = struct.pack(">H", len(data))
    s.sendall(hdr + data)
    print(f">>> {l1ctl_name(msg_type)} ({len(data)} bytes)")

def recv_msg(s, timeout=5.0):
    s.settimeout(timeout)
    try:
        hdr = s.recv(2)
        if len(hdr) < 2:
            return None, None
        ml = struct.unpack(">H", hdr)[0]
        data = b''
        while len(data) < ml:
            chunk = s.recv(ml - len(data))
            if not chunk:
                return None, None
            data += chunk
        msg_type = data[0]
        print(f"<<< {l1ctl_name(msg_type)} ({ml} bytes): {data[:20].hex()}")
        return msg_type, data
    except socket.timeout:
        return None, None

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/osmocom_l2_1"

    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(path)
    print(f"Connected to {path}")

    # Wait for RESET_IND
    print("\n--- Waiting for RESET_IND ---")
    t, d = recv_msg(s)

    # Send RESET_REQ
    print("\n--- Sending RESET_REQ ---")
    # Payload: 4 bytes header padding + reset_type at byte 4
    # reset_type: 1 = full reset, 2 = scheduler reset
    send_msg(s, L1CTL_RESET_REQ, bytes([1, 0, 0, 0]))
    t, d = recv_msg(s)  # PM_CONF or RESET_CONF

    # Wait for PM_CONF
    print("\n--- Waiting for PM_CONF ---")
    for i in range(10):
        t, d = recv_msg(s, timeout=5.0)
        if t is not None:
            print(f"  GOT: {l1ctl_name(t)}")
            if t == 0x0E:  # PM_CONF
                break

    # Send FBSB_REQ for ARFCN 514
    # Format: flags(1)=1, pad(3), ARFCN(2 BE), timeout(2 BE), rxlev(1), bsic(1), ...
    print("\n--- Sending FBSB_REQ (ARFCN 514) ---")
    arfcn = 514
    fbsb = bytes([
        1, 0, 0, 0,               # flags=1 (new search), padding
        (arfcn >> 8) & 0xFF,       # ARFCN high
        arfcn & 0xFF,              # ARFCN low
        0, 100,                    # timeout = 100 frames
        0xAB,                      # rxlev_exp = -85 (signed)
        0,                         # bsic
        7, 0,                      # ccch_mode, sync_info_idx
    ])
    send_msg(s, L1CTL_FBSB_REQ, fbsb)

    # Wait for responses
    print("\n--- Waiting for responses (120s) ---")
    for i in range(60):
        t, d = recv_msg(s, timeout=2.0)
        if t is None:
            if i % 10 == 0:
                print(f"  (timeout {i})")
        else:
            print(f"  *** {l1ctl_name(t)} : {d[:16].hex()} ***")
            if t == L1CTL_DATA_IND:
                print(f"  !!! DATA_IND !!! payload={d.hex()}")

    s.close()
    print("\nDone.")

if __name__ == "__main__":
    main()

===== ./build.sh =====
#!/bin/bash
# build.sh — sync host→container, update build tag, build, sync back
set -euo pipefail

CONTAINER="trying"
HOST_DIR="/home/nirvana/qemu-src/hw/arm/calypso"
CONT_DIR="/root/qemu/hw/arm/calypso"
TRX="$HOST_DIR/calypso_trx.c"
TAG="BUILD $(date '+%Y-%m-%dT%H:%M:%S')"

# 1. Update build tag in calypso_trx.c
sed -i "s/BUILD [0-9T:_-]*/$(echo "$TAG" | sed 's/[&/]/\\&/g')/" "$TRX"
echo "$TAG"

# 2. Sync host → container
docker cp "$HOST_DIR/." "$CONTAINER:$CONT_DIR/"

# 3. Build
docker exec "$CONTAINER" bash -c 'cd /root/qemu/build && ninja' 2>&1 | tail -5

# 4. Verify md5
echo "--- md5 ---"
H_TRX=$(md5sum "$HOST_DIR/calypso_trx.c" | cut -d' ' -f1)
H_C54=$(md5sum "$HOST_DIR/calypso_c54x.c" | cut -d' ' -f1)
C_TRX=$(docker exec "$CONTAINER" md5sum "$CONT_DIR/calypso_trx.c" | cut -d' ' -f1)
C_C54=$(docker exec "$CONTAINER" md5sum "$CONT_DIR/calypso_c54x.c" | cut -d' ' -f1)

if [ "$H_TRX" = "$C_TRX" ] && [ "$H_C54" = "$C_C54" ]; then
    echo "OK synced"
else
    echo "MISMATCH!"
    echo "  trx: host=$H_TRX container=$C_TRX"
    echo "  c54: host=$H_C54 container=$C_C54"
    exit 1
fi

===== ./sercomm_udp.py =====
#!/usr/bin/env python3
"""
sercomm_udp.py — BTS TRX <-> QEMU UART sercomm bridge
BTS side:  OsmoTRX protocol (CLK/TRXC/TRXD on 5700-5702)
MS side:   UART PTY (sercomm DLCI 4=burst, DLCI 5=l1ctl)

DL: osmo-bts-trx sends TRXD soft bits -> convert to int16 -> sercomm DLCI 4 -> PTY
UL: PTY sercomm DLCI 4 -> TRXD hard bits -> osmo-bts-trx

Usage: sercomm_udp.py <pty-path> [bts-base-port]
"""
import errno, fcntl, os, select, signal, socket, struct, sys, termios, threading, time

GSM_HYPERFRAME = 2715648
GSM_FRAME_US = 4615.0
CLK_IND_PERIOD = 102
SERCOMM_FLAG = 0x7E
SERCOMM_ESCAPE = 0x7D
SERCOMM_XOR = 0x20
DLCI_L1CTL = 5
DLCI_BURST = 4

def udp_bind(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('127.0.0.1', port))
    s.setblocking(False)
    return s

def sercomm_wrap(dlci, payload):
    out = bytearray([SERCOMM_FLAG])
    for b in bytes([dlci, 0x03]) + payload:
        if b in (SERCOMM_FLAG, SERCOMM_ESCAPE):
            out.append(SERCOMM_ESCAPE)
            out.append(b ^ SERCOMM_XOR)
        else:
            out.append(b)
    out.append(SERCOMM_FLAG)
    return bytes(out)

class SercommParser:
    def __init__(self):
        self.buf = bytearray()
    def feed(self, data):
        self.buf.extend(data)
        frames = []
        while True:
            try: s = self.buf.index(SERCOMM_FLAG)
            except ValueError: self.buf.clear(); break
            if s > 0: del self.buf[:s]
            try: e = self.buf.index(SERCOMM_FLAG, 1)
            except ValueError: break
            raw = bytes(self.buf[1:e]); del self.buf[:e+1]
            if not raw: continue
            out = bytearray(); i = 0
            while i < len(raw):
                if raw[i] == SERCOMM_ESCAPE and i+1 < len(raw):
                    i += 1; out.append(raw[i] ^ SERCOMM_XOR)
                else: out.append(raw[i])
                i += 1
            if len(out) >= 2: frames.append((out[0], bytes(out[2:])))
        return frames

def soft_to_int16(soft_bit):
    """Convert osmocom soft bit (0=strong 1, 255=strong 0) to int16.
    Maps: 0 -> +32767, 127 -> 0, 255 -> -32767"""
    return max(-32768, min(32767, int((127 - soft_bit) * 32767 / 127)))

def soft_bits_to_gmsk_iq(soft_bits_raw, scale=4096):
    """Convert TRXD soft bits to GMSK I/Q int16 samples for DSP BSP.
    Calypso ABB delivers baseband I/Q from antenna; we generate equivalent
    by GMSK-modulating the hard bits (BT=0.3, h=0.5, 1 sample/symbol)."""
    import numpy as np
    from scipy.ndimage import gaussian_filter1d
    n = len(soft_bits_raw)
    hard = np.array([0.0 if b < 128 else 1.0 for b in soft_bits_raw])
    nrz = 1.0 - 2.0 * hard  # 0->+1, 1->-1 (GSM convention)
    bt = 0.3
    filtered = gaussian_filter1d(nrz, 1.0 / (2.0 * 3.141592653589793 * bt))
    phase = np.cumsum(filtered) * 3.141592653589793 * 0.5
    iq = np.clip(np.cos(phase) * scale, -32768, 32767).astype(np.int16)
    return iq

class Bridge:
    def __init__(self, pty_path, bts_base=5700):
        self.pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(self.pty_fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 1; attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.pty_fd, termios.TCSANOW, attrs)
        fcntl.fcntl(self.pty_fd, fcntl.F_SETFL,
                    fcntl.fcntl(self.pty_fd, fcntl.F_GETFL) | os.O_NONBLOCK)

        self.clk_sock  = udp_bind(bts_base)
        self.trxc_sock = udp_bind(bts_base + 1)
        self.trxd_sock = udp_bind(bts_base + 2)
        self.bts_clk_addr = ('127.0.0.1', bts_base + 100)
        self.trxc_remote = None
        self.trxd_remote = None
        self.powered = False
        self.fn = 0
        self._stop = threading.Event()
        self.parser = SercommParser()
        self.stats = {'clk': 0, 'trxc': 0, 'dl': 0, 'ul': 0}
        self.cfile_fd = open('/tmp/gsm_burst.cfile', 'wb')
        print('sercomm_udp: pty=%s CLK=%d TRXC=%d TRXD=%d' % (pty_path, bts_base, bts_base+1, bts_base+2), flush=True)

    def start_clock(self):
        threading.Thread(target=self._clock_loop, daemon=True).start()

    def _clock_loop(self):
        tick_ns = int(GSM_FRAME_US * 1000)
        t_next = time.monotonic_ns()
        while not self._stop.is_set():
            t_next += tick_ns
            dt = t_next - time.monotonic_ns()
            if dt > 0: time.sleep(dt / 1e9)
            elif dt < -tick_ns * 10: t_next = time.monotonic_ns()
            self.fn = (self.fn + 1) % GSM_HYPERFRAME
            if self.fn % CLK_IND_PERIOD == 0 and self.powered:
                try:
                    self.clk_sock.sendto(('IND CLOCK %d\0' % self.fn).encode(), self.bts_clk_addr)
                    self.stats['clk'] += 1
                except OSError: pass

    def handle_trxc(self):
        try: data, addr = self.trxc_sock.recvfrom(256)
        except OSError: return
        if not data: return
        self.trxc_remote = addr
        raw = data.strip(b'\x00').decode(errors='replace')
        if not raw.startswith('CMD '): return
        parts = raw[4:].split(); verb = parts[0]; args = parts[1:]
        self.stats['trxc'] += 1

        if verb == 'POWERON':
            self.powered = True; print('BTS: POWERON', flush=True)
            rsp = 'RSP POWERON 0'
        elif verb == 'POWEROFF':
            self.powered = False; rsp = 'RSP POWEROFF 0'
        elif verb == 'SETFORMAT':
            # Force TRXD v0 — we only support v0 format
            rsp = 'RSP SETFORMAT 0 0'
        elif verb == 'NOMTXPOWER':
            rsp = 'RSP NOMTXPOWER 0 50'
        elif verb == 'MEASURE':
            freq = args[0] if args else '0'
            rsp = 'RSP MEASURE 0 %s -60' % freq
        else:
            rsp = ('RSP %s 0 %s' % (verb, ' '.join(args))).rstrip()
        self.trxc_sock.sendto((rsp + '\0').encode(), addr)

    def handle_trxd_from_bts(self):
        """BTS DL: TRXD v0 soft bits -> int16 samples -> sercomm DLCI 4 -> PTY.
        TRXD v0 DL: TN(1) FN(4) RSSI(1) TOA256(2) soft_bits(148)
        Sercomm payload: TN(1) FN(4) RSSI(1) TOA256(2) int16_samples(148*2)"""
        try: data, addr = self.trxd_sock.recvfrom(512)
        except OSError: return
        if len(data) < 8: return
        self.trxd_remote = addr
        if not self.powered: return
        self.stats['dl'] += 1

        hdr = data[:8]
        soft_bits = data[8:]
        nbits = min(len(soft_bits), 148)

        # GMSK modulate soft bits → I/Q int16 samples for DSP BSP
        iq_samples = soft_bits_to_gmsk_iq(soft_bits[:nbits])
        payload = bytearray(hdr)
        payload.extend(iq_samples.tobytes())

        # Log I/Q as complex float32 cfile for grgsm_decode
        if self.cfile_fd is not None:
            import numpy as np
            cf = (iq_samples.astype(np.float32) / 4096.0).view(np.float32)
            # Write as complex: I + 0j (real-only for now)
            cplx = np.zeros(len(iq_samples) * 2, dtype=np.float32)
            cplx[0::2] = cf  # I
            # cplx[1::2] = 0  # Q = 0 (already zeroed)
            self.cfile_fd.write(cplx.tobytes())

        frame = sercomm_wrap(DLCI_BURST, bytes(payload))
        try: os.write(self.pty_fd, frame)
        except OSError: pass

    def handle_uart_from_qemu(self):
        try: raw = os.read(self.pty_fd, 4096)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK): return
            raise
        if not raw: return
        for dlci, payload in self.parser.feed(raw):
            if dlci == DLCI_BURST and payload and len(payload) >= 6:
                if self.trxd_remote:
                    try: self.trxd_sock.sendto(payload, self.trxd_remote)
                    except OSError: pass
                    self.stats['ul'] += 1

    def run(self):
        self.start_clock()
        running = True
        def shutdown(s, f):
            nonlocal running; running = False
        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)

        while running:
            try:
                readable, _, _ = select.select(
                    [self.pty_fd, self.trxc_sock, self.trxd_sock], [], [], 0.1)
            except (OSError, ValueError):
                break
            if self.trxc_sock in readable: self.handle_trxc()
            if self.trxd_sock in readable: self.handle_trxd_from_bts()
            if self.pty_fd in readable: self.handle_uart_from_qemu()

        self._stop.set()
        os.close(self.pty_fd)
        print('sercomm_udp: stats=%s' % self.stats, flush=True)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: %s <pty-path>' % sys.argv[0]); sys.exit(1)
    Bridge(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 5700).run()

===== ./air_burst_player.py =====
#!/usr/bin/env python3
"""
air_burst_player.py — Replay bursts from bursts.txt to QEMU air interface.

Sends bursts to the QEMU mobile instance via UDP, simulating the air interface.
Format: TN(1) + FN(4) + 148 hard bits

Loops the burst file continuously, advancing FN coherently.

Usage:
  python3 air_burst_player.py /root/bursts.txt [peer_port] [rate]
"""

import socket
import struct
import sys
import time

def load_bursts(path):
    """Load bursts from file. Format: 'index fn: bits'"""
    bursts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or ':' not in line:
                continue
            parts = line.split(':')
            header = parts[0].strip().split()
            bits_str = parts[1].strip()
            if len(header) < 2 or len(bits_str) < 100:
                continue
            fn = int(header[1])
            # Pad/trim to 148 bits (file has 114, pad with zeros for guard)
            bits = [int(b) for b in bits_str[:148]]
            while len(bits) < 148:
                bits.append(0)
            bursts.append((fn, bits))
    return bursts

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/root/bursts.txt"
    peer_port = int(sys.argv[2]) if len(sys.argv) > 2 else 4801
    rate = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

    bursts = load_bursts(path)
    if not bursts:
        print("No bursts loaded", flush=True)
        sys.exit(1)

    print("Loaded %d bursts from %s" % (len(bursts), path), flush=True)
    print("Sending to 127.0.0.1:%d, rate=%.1fx" % (peer_port, rate), flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer = ("127.0.0.1", peer_port)

    # GSM frame timing: 4.615ms per frame
    frame_interval = 0.004615 / rate

    # Get FN range from file
    fn_start = bursts[0][0]
    fn_end = bursts[-1][0]
    fn_span = fn_end - fn_start
    if fn_span <= 0:
        fn_span = len(bursts)

    loop_count = 0
    total_sent = 0

    print("FN range: %d - %d (span %d)" % (fn_start, fn_end, fn_span), flush=True)
    print("Starting playback...", flush=True)

    try:
        while True:
            fn_offset = loop_count * fn_span
            t_start = time.monotonic()

            for i, (fn_orig, bits) in enumerate(bursts):
                fn = (fn_orig + fn_offset) % 2715648  # GSM hyperframe

                # Build air packet: TN(1) + FN(4) + 148 bits
                pkt = struct.pack(">BI", 0, fn) + bytes(bits)
                sock.sendto(pkt, peer)
                total_sent += 1

                if total_sent <= 5 or total_sent % 500 == 0:
                    print("  [air] #%d FN=%d (loop %d)" % (total_sent, fn, loop_count), flush=True)

                # Sleep to match GSM timing
                # Bursts in file may skip FNs, calculate proper delay
                if i + 1 < len(bursts):
                    fn_next = bursts[i + 1][0]
                    fn_delta = fn_next - fn_orig
                    if fn_delta <= 0:
                        fn_delta = 1
                    delay = frame_interval * fn_delta
                else:
                    delay = frame_interval

                time.sleep(delay)

            loop_count += 1
            elapsed = time.monotonic() - t_start
            print("  [air] Loop %d done (%d bursts in %.1fs)" % (loop_count, len(bursts), elapsed), flush=True)

    except KeyboardInterrupt:
        print("\nStopped. Total sent: %d" % total_sent, flush=True)
    finally:
        sock.close()

if __name__ == "__main__":
    main()

===== ./run.sh =====
#!/bin/bash
# run.sh — QEMU Calypso pipeline
#
# QEMU (layer1 firmware) -serial pty→ sercomm_udp.py (5700-5702) ↔ osmo-bts-trx
# QEMU exposes /tmp/osmocom_l2_1 for mobile/ccch_scan
#
set -euo pipefail

SESSION="calypso"
QEMU="/root/qemu/build/qemu-system-arm"
FW="/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
BRIDGE="/opt/GSM/qemu-src/sercomm_udp.py"
BTS_CFG="/etc/osmocom/osmo-bts-trx.cfg"
L1CTL_SOCK="/tmp/osmocom_l2_1"
QEMU_LOG="/tmp/qemu.log"

# ── Cleanup ──
tmux kill-session -t "$SESSION" 2>/dev/null || true
pkill -9 qemu-system-arm 2>/dev/null || true
pkill -9 -f sercomm_udp.py 2>/dev/null || true
pkill -9 osmo-bts-trx 2>/dev/null || true
pkill -9 mobile 2>/dev/null || true
pkill -9 ccch_scan 2>/dev/null || true
systemctl stop osmo-bts-trx 2>/dev/null || true
rm -f "$L1CTL_SOCK"
sleep 1

# ── QEMU ──
$QEMU -M calypso \
  -kernel "$FW" \
  -serial pty \
    -S -display none \
  > "$QEMU_LOG" 2>&1 &
QEMU_PID=$!
sleep 2

PTY=$(grep -o "/dev/pts/[0-9]*" "$QEMU_LOG" | head -1)
if [ -z "$PTY" ]; then
    echo "FATAL: QEMU did not allocate a PTY"
    cat "$QEMU_LOG"
    exit 1
fi
echo "QEMU PID=$QEMU_PID PTY=$PTY"

# Wait for L1CTL socket
echo -n "Waiting for L1CTL socket..."
for i in $(seq 1 30); do
    [ -S "$L1CTL_SOCK" ] && break
    sleep 1; echo -n "."
done
[ -S "$L1CTL_SOCK" ] && echo " OK" || echo " TIMEOUT (continuing)"

# ── tmux ──
tmux new-session -d -s "$SESSION" -n qemu
tmux send-keys -t "$SESSION:qemu" "tail -f $QEMU_LOG" C-m

tmux new-window -t "$SESSION" -n bridge
tmux send-keys -t "$SESSION:bridge" "python3 $BRIDGE $PTY" C-m
sleep 1

tmux new-window -t "$SESSION" -n bts
tmux send-keys -t "$SESSION:bts" "osmo-bts-trx -c $BTS_CFG" C-m

#tmux new-window -t "$SESSION" -n ccch
#tmux send-keys -t "$SESSION:ccch" "sleep 5 && ccch_scan -i 127.0.0.1 -a 514 -s $L1CTL_SOCK" C-m
tmux new-window -t "$SESSION" -n mobile
tmux send-keys -t "$SESSION:mobile" "sleep 5 && mobile -c /root/.osmocom/bb/mobile_group1.cfg" C-m

tmux new-window -t "$SESSION" -n grgsm
tmux send-keys -t "$SESSION:grgsm" "echo 'grgsm_decode ready — run: grgsm_decode -c /tmp/gsm_burst.cfile -m BCCH_SDCCH4 -v'" C-m

tmux new-window -t "$SESSION" -n shell

tmux select-window -t "$SESSION:qemu"
exec tmux attach -t "$SESSION"

===== ./l1ctl_bridge.py =====
#!/usr/bin/env python3
"""
l1ctl_bridge.py — Bridge between mobile (layer23) and QEMU firmware via sercomm.

Firmware-first mode: all L1CTL messages forwarded to firmware.
Only RESET_REQ and PM_REQ are handled synthetically (firmware can't handle them).
The firmware handles FBSB, CCCH, RACH, DM_EST, DATA, etc. via its real DSP/TPU.

Air socket: receives DL bursts from fake_trx, decodes CCCH, sends DATA_IND to mobile.
(Used when QEMU has trx_port=0 and doesn't talk to fake_trx directly.)

Architecture:
  mobile <-> /tmp/osmocom_l2 (L1CTL) <-> this bridge <-> PTY (sercomm) <-> QEMU firmware
                                              |
                                         air socket (DL bursts from fake_trx)

Usage:
  BRIDGE_AIR_PORT=6702 python3 l1ctl_bridge.py /dev/pts/X [/tmp/osmocom_l2_ms]
"""

import errno
import fcntl
import os
import select
import signal
import socket
import struct
import sys
import termios
import time

# ═══════════════════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════════════════

DLCI_L1CTL   = 5
DLCI_LOADER  = 9
DLCI_DEBUG   = 4
DLCI_CONSOLE = 10
CTRL         = 0x03
FLAG         = 0x7E
ESCAPE       = 0x7D
ESCAPE_XOR   = 0x20

L1CTL_SOCK = "/tmp/osmocom_l2"

L1CTL_NAMES = {
    0x01: "FBSB_REQ",      0x02: "FBSB_CONF",     0x03: "DATA_IND",
    0x04: "RACH_REQ",      0x05: "RACH_CONF",      0x06: "DATA_REQ",
    0x07: "RESET_IND",     0x08: "PM_REQ",         0x09: "PM_CONF",
    0x0A: "ECHO_REQ",      0x0B: "ECHO_CONF",      0x0C: "DATA_CONF",
    0x0D: "RESET_REQ",     0x0E: "RESET_CONF",
    0x10: "CCCH_MODE_REQ", 0x11: "CCCH_MODE_CONF",
    0x14: "DM_EST_REQ",    0x15: "DM_FREQ_REQ",    0x16: "DM_REL_REQ",
    0x1D: "TCH_MODE_REQ",  0x1E: "TCH_MODE_CONF",
    0x21: "TRAFFIC_REQ",   0x22: "TRAFFIC_CONF",   0x23: "TRAFFIC_IND",
}

MSG_RESET_IND  = 0x07
MSG_PM_REQ     = 0x08
MSG_PM_CONF    = 0x09
MSG_RESET_REQ  = 0x0D
MSG_RESET_CONF = 0x0E
MSG_DATA_IND   = 0x03

TARGET_ARFCN = 514


# ═══════════════════════════════════════════════════════════════════════════════
# GSM channel decoding (for air socket CCCH decode)
# ═══════════════════════════════════════════════════════════════════════════════

def decode_ccch_block(bursts_data):
    """Decode 4-burst CCCH block using libosmocoding."""
    import ctypes
    try:
        lib = ctypes.CDLL("/usr/local/lib/libosmocoding.so")
    except Exception:
        return None

    iB = (ctypes.c_int8 * (4 * 116))()
    for i in range(4):
        for j in range(57):
            iB[i * 116 + j] = -127 if bursts_data[i][j] else 127
        iB[i * 116 + 57] = 0
        iB[i * 116 + 58] = 0
        for j in range(57):
            iB[i * 116 + 59 + j] = -127 if bursts_data[i][57 + j] else 127

    l2 = (ctypes.c_uint8 * 23)()
    n_err = ctypes.c_int(0)
    n_bits = ctypes.c_int(0)
    lib.gsm0503_xcch_decode(l2, iB, ctypes.byref(n_err), ctypes.byref(n_bits))
    return bytes(l2)


def extract_burst_data(bits_148):
    """Extract 114 coded bits from 148-bit normal burst."""
    raw = list(bits_148[:148])
    if len(raw) < 148:
        raw.extend([0] * (148 - len(raw)))
    if max(raw) > 1:
        hard = [1 if b > 127 else 0 for b in raw]
    else:
        hard = raw
    return hard[3:60] + hard[88:145]


# ═══════════════════════════════════════════════════════════════════════════════
# Sercomm framing
# ═══════════════════════════════════════════════════════════════════════════════

def sercomm_escape(data):
    out = bytearray()
    for b in data:
        if b in (FLAG, ESCAPE):
            out.append(ESCAPE)
            out.append(b ^ ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)


def sercomm_unescape(data):
    out = bytearray()
    i = 0
    while i < len(data):
        if data[i] == ESCAPE:
            i += 1
            if i < len(data):
                out.append(data[i] ^ ESCAPE_XOR)
        else:
            out.append(data[i])
        i += 1
    return bytes(out)


def sercomm_wrap(dlci, payload):
    inner = bytes([dlci, CTRL]) + payload
    return bytes([FLAG]) + sercomm_escape(inner) + bytes([FLAG])


class SercommParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        frames = []
        while True:
            try:
                start = self.buf.index(FLAG)
            except ValueError:
                self.buf.clear()
                break
            if start > 0:
                del self.buf[:start]
            try:
                end = self.buf.index(FLAG, 1)
            except ValueError:
                break
            raw = bytes(self.buf[1:end])
            del self.buf[:end + 1]
            if not raw:
                continue
            frame = sercomm_unescape(raw)
            if len(frame) < 2:
                continue
            frames.append((frame[0], frame[2:]))
        return frames


class LengthPrefixReader:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        msgs = []
        while len(self.buf) >= 2:
            msglen = struct.unpack(">H", self.buf[:2])[0]
            if len(self.buf) < 2 + msglen:
                break
            msgs.append(bytes(self.buf[2:2 + msglen]))
            del self.buf[:2 + msglen]
        return msgs


# ═══════════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def hexdump(data, maxlen=32):
    h = " ".join(f"{b:02x}" for b in data[:maxlen])
    if len(data) > maxlen:
        h += f" ... ({len(data)} bytes)"
    return h


def set_raw(fd):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def set_nonblock(fd):
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)


def l1ctl_send(client, msg_type, payload=b""):
    hdr = struct.pack("BBxx", msg_type, 0)
    msg = hdr + payload
    frame = struct.pack(">H", len(msg)) + msg
    try:
        client.sendall(frame)
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <pty-path> [socket-path]", file=sys.stderr)
        sys.exit(1)

    pty_path = sys.argv[1]
    if len(sys.argv) > 2:
        global L1CTL_SOCK
        L1CTL_SOCK = sys.argv[2]

    try:
        os.unlink(L1CTL_SOCK)
    except FileNotFoundError:
        pass

    # ── Open PTY to QEMU ──
    pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
    set_raw(pty_fd)
    set_nonblock(pty_fd)

    # ── L1CTL server socket ──
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(L1CTL_SOCK)
    srv.listen(1)
    srv.setblocking(False)

    # ── Air UDP socket (DL bursts from fake_trx for CCCH decode) ──
    AIR_PORT = int(os.environ.get("BRIDGE_AIR_PORT", "0"))
    air_sock = None
    if AIR_PORT > 0:
        air_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        air_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            air_sock.bind(("0.0.0.0", AIR_PORT + 100))
            air_sock.setblocking(False)
            print(f"l1ctl-bridge: air UDP on port {AIR_PORT + 100}", flush=True)
        except OSError:
            air_sock = None
            print(f"l1ctl-bridge: air UDP port {AIR_PORT + 100} busy", flush=True)

    print(f"l1ctl-bridge: PTY={pty_path} socket={L1CTL_SOCK}", flush=True)
    print(f"l1ctl-bridge: firmware-first mode (synth: RESET, PM only)", flush=True)
    print(f"l1ctl-bridge: TARGET_ARFCN={TARGET_ARFCN}", flush=True)

    parser = SercommParser()
    lp = LengthPrefixReader()
    client = None
    running = True
    stats = {"tx": 0, "rx": 0, "console": 0, "burst": 0, "decoded": 0}

    # Burst accumulator for CCCH decoding
    burst_buf = {}

    def shutdown(_sig, _frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        while running:
            rlist = [srv, pty_fd]
            if air_sock:
                rlist.append(air_sock)
            if client is not None:
                rlist.append(client)

            try:
                readable, _, _ = select.select(rlist, [], [], 0.5)
            except (OSError, ValueError):
                break

            # ── Accept new mobile connection ──
            if srv in readable:
                conn, _ = srv.accept()
                conn.setblocking(False)
                if client is not None:
                    try:
                        client.close()
                    except OSError:
                        pass
                client = conn
                lp = LengthPrefixReader()
                print("l1ctl-bridge: mobile connected", flush=True)

            # ══════════════════════════════════════════════════════════════
            # mobile -> firmware
            # ══════════════════════════════════════════════════════════════
            if client is not None and client in readable:
                try:
                    raw = client.recv(4096)
                except BlockingIOError:
                    raw = b""

                if not raw:
                    print("l1ctl-bridge: mobile disconnected", flush=True)
                    try:
                        client.close()
                    except OSError:
                        pass
                    client = None
                else:
                    for payload in lp.feed(raw):
                        stats["tx"] += 1
                        msg_type = payload[0] if payload else 0xFF
                        name = L1CTL_NAMES.get(msg_type, f"0x{msg_type:02x}")

                        # ── RESET_REQ — synthetic (firmware can't handle) ──
                        if msg_type == MSG_RESET_REQ:
                            reset_type = payload[4] if len(payload) > 4 else 1
                            l1ctl_send(client, MSG_RESET_CONF,
                                       struct.pack("Bxxx", reset_type))
                            print(f"  [synth] RESET_REQ type={reset_type} -> RESET_CONF",
                                  flush=True)
                            continue  # don't forward

                        # ── PM_REQ — synthetic (firmware causes msgb exhaustion) ──
                        if msg_type == MSG_PM_REQ and len(payload) >= 12:
                            arfcn_from = (payload[8] << 8) | payload[9]
                            arfcn_to = (payload[10] << 8) | payload[11]
                            entries = []
                            for a in range(arfcn_from, arfcn_to + 1):
                                rxlev = 0x30 if a == TARGET_ARFCN else 0x00
                                entries.append(struct.pack(">HBB", a, rxlev, rxlev))

                            batch_size = 50
                            for i in range(0, len(entries), batch_size):
                                batch = entries[i:i + batch_size]
                                is_last = (i + batch_size >= len(entries))
                                hdr = bytes([MSG_PM_CONF,
                                             0x01 if is_last else 0x00,
                                             0x00, 0x00])
                                pm_data = hdr + b"".join(batch)
                                msg = struct.pack(">H", len(pm_data)) + pm_data
                                try:
                                    client.sendall(msg)
                                except (BrokenPipeError, OSError):
                                    pass
                            print(f"  [synth] PM_REQ {arfcn_from}-{arfcn_to} -> PM_CONF",
                                  flush=True)
                            continue  # don't forward

                        # ── Everything else -> forward to firmware ──
                        frame = sercomm_wrap(DLCI_L1CTL, payload)
                        try:
                            os.write(pty_fd, frame)
                        except OSError as e:
                            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                                raise

                        if stats["tx"] <= 10 or stats["tx"] % 50 == 0:
                            print(f"  [mobile->fw] #{stats['tx']} {name} "
                                  f"len={len(payload)} {hexdump(payload, 16)}",
                                  flush=True)

            # ══════════════════════════════════════════════════════════════
            # air bursts -> CCCH decode -> DATA_IND to mobile
            # ══════════════════════════════════════════════════════════════
            if air_sock and air_sock in readable:
                try:
                    data, addr = air_sock.recvfrom(512)
                except OSError:
                    data = b""
                if len(data) >= 9 and client is not None:
                    tn = data[0] & 0x07
                    fn = struct.unpack(">I", data[1:5])[0]
                    soft_bits = data[8:]
                    stats["burst"] += 1

                    if tn != 0:
                        pass  # only TS0 for BCCH/CCCH
                    else:
                        coded = extract_burst_data(soft_bits)
                        fn51 = fn % 51
                        bcch_ccch_starts = [2, 6, 12, 16]
                        block_id = None
                        burst_idx = None
                        chan_nr = 0x80

                        for start in bcch_ccch_starts:
                            if start <= fn51 < start + 4:
                                burst_idx = fn51 - start
                                block_id = (fn // 51, start)
                                chan_nr = 0x80 if start == 2 else 0x90
                                break

                        if block_id is not None and burst_idx is not None:
                            if block_id not in burst_buf:
                                burst_buf[block_id] = [None] * 4
                            burst_buf[block_id][burst_idx] = coded

                            if all(b is not None for b in burst_buf[block_id]):
                                l2 = decode_ccch_block(burst_buf[block_id])
                                del burst_buf[block_id]

                                if l2 and len(l2) == 23:
                                    hdr = struct.pack("BBxx", MSG_DATA_IND, 0)
                                    info_dl = struct.pack(">BBHIBBBB",
                                        chan_nr, 0, TARGET_ARFCN, fn, 48, 20, 0, 0)
                                    msg = hdr + info_dl + l2
                                    frame = struct.pack(">H", len(msg)) + msg
                                    try:
                                        client.sendall(frame)
                                    except (BrokenPipeError, OSError):
                                        pass
                                    stats["decoded"] += 1
                                    if stats["decoded"] <= 10 or stats["decoded"] % 200 == 0:
                                        print(f"  [air->mobile] DATA_IND #{stats['decoded']} "
                                              f"FN={fn} ch=0x{chan_nr:02x} {l2.hex()[:46]}",
                                              flush=True)

                        # Clean stale blocks
                        stale = [k for k in burst_buf if fn - k[0] * 51 > 200]
                        for k in stale:
                            del burst_buf[k]

            # ══════════════════════════════════════════════════════════════
            # firmware -> mobile
            # ══════════════════════════════════════════════════════════════
            if pty_fd in readable:
                try:
                    raw = os.read(pty_fd, 4096)
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                        continue
                    raise

                if not raw:
                    print("l1ctl-bridge: PTY closed", flush=True)
                    break

                for dlci, payload in parser.feed(raw):
                    if dlci == DLCI_L1CTL:
                        stats["rx"] += 1

                        # Forward ALL firmware L1CTL to mobile
                        if client is not None:
                            msg = struct.pack(">H", len(payload)) + payload
                            try:
                                client.sendall(msg)
                            except (BrokenPipeError, OSError):
                                print("l1ctl-bridge: mobile send error", flush=True)
                                try:
                                    client.close()
                                except OSError:
                                    pass
                                client = None

                        if stats["rx"] <= 10 or stats["rx"] % 100 == 0:
                            msg_type = payload[0] if payload else 0xFF
                            rx_name = L1CTL_NAMES.get(msg_type, f"0x{msg_type:02x}")
                            extra = ""
                            if msg_type == 0x02 and len(payload) >= 20:
                                extra = f" result={payload[18]} bsic={payload[19]}"
                            print(f"  [fw->mobile] #{stats['rx']} {rx_name} "
                                  f"len={len(payload)}{extra} {hexdump(payload, 20)}",
                                  flush=True)

                    elif dlci == DLCI_CONSOLE:
                        stats["console"] += 1
                        text_bytes = payload
                        if text_bytes and text_bytes[0] == 0x03:
                            text_bytes = text_bytes[1:]
                        try:
                            sys.stderr.write(
                                text_bytes.decode("ascii", errors="replace"))
                            sys.stderr.flush()
                        except Exception:
                            pass

                    elif dlci == DLCI_LOADER:
                        pass

    finally:
        if client is not None:
            try:
                client.close()
            except OSError:
                pass
        srv.close()
        try:
            os.close(pty_fd)
        except OSError:
            pass
        try:
            os.unlink(L1CTL_SOCK)
        except OSError:
            pass
        print(f"l1ctl-bridge: done (tx={stats['tx']} rx={stats['rx']} "
              f"decoded={stats['decoded']} console={stats['console']})",
              flush=True)


if __name__ == "__main__":
    main()

===== ./sercomm_relay.py =====
#!/usr/bin/env python3
"""
Sercomm relay for Osmocom loader.

Socket side:
    [2-byte big-endian length][payload]

UART/PTTY side:
    [0x7e][dlci][ctrl=0x03][escaped payload][0x7e]

This relay keeps osmoload untouched, but splits large LOADER_MEM_WRITE
messages into smaller sub-writes on the wire, then synthesizes one final
reply back to osmoload.

Why:
- ping works already
- large memload frames fail with bad CRC on the loader side
- splitting at protocol level is safer than touching osmoload
"""

import errno
import fcntl
import os
import select
import signal
import socket
import struct
import sys
import termios

DLCI_TX = 9
DLCI_RX = 9
CTRL = 0x03

FLAG = 0x7E
ESCAPE = 0x7D
ESCAPE_XOR = 0x20

SOCK_PATH = os.environ.get("SERCOMM_SOCK_PATH", "/tmp/osmocom_loader")

LOADER_PING = 0x01
LOADER_MEM_WRITE = 0x08

# Keep this comfortably below the original 248-byte socket payload size.
# 64 data bytes per sub-write is a conservative starting point.
MEMWRITE_CHUNK = 64


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02x}" for b in data)


def set_raw(fd: int) -> None:
    attrs = termios.tcgetattr(fd)

    # iflag
    attrs[0] = 0

    # oflag
    attrs[1] = 0

    # cflag: 8N1, local, read enabled
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD

    # lflag
    attrs[3] = 0

    # ispeed / ospeed
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200

    # cc
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def set_nonblock(fd: int) -> None:
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)


def sercomm_escape(data: bytes) -> bytes:
    out = bytearray()
    for b in data:
        if b in (FLAG, ESCAPE):
            out.append(ESCAPE)
            out.append(b ^ ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)
    

def sercomm_unescape(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        b = data[i]
        if b == ESCAPE:
            i += 1
            if i >= len(data):
                break
            out.append(data[i] ^ ESCAPE_XOR)
        else:
            out.append(b)
        i += 1
    return bytes(out)


def sercomm_wrap(dlci: int, payload: bytes) -> bytes:
    inner = bytes([dlci, CTRL]) + payload
    return bytes([FLAG]) + sercomm_escape(inner) + bytes([FLAG])


def osmo_crc16(data: bytes) -> int:
    """
    CRC-16/CCITT as used by Osmocom loader code via osmo_crc16(0, ...).
    Init = 0x0000, poly = 0x1021, no xorout.
    """
    crc = 0x0000
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class SercommParser:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []

        while True:
            try:
                start = self.buf.index(FLAG)
            except ValueError:
                self.buf.clear()
                break

            if start > 0:
                del self.buf[:start]

            try:
                end = self.buf.index(FLAG, 1)
            except ValueError:
                break

            raw = bytes(self.buf[1:end])
            del self.buf[: end + 1]

            if not raw:
                continue

            frame = sercomm_unescape(raw)
            if len(frame) < 2:
                continue

            dlci = frame[0]
            ctrl = frame[1]
            payload = frame[2:]

            # Keep permissive; log only if needed.
            _ = ctrl
            frames.append((dlci, payload))

        return frames


class LengthPrefixReader:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        msgs = []

        while True:
            if len(self.buf) < 2:
                break

            msglen = struct.unpack(">H", self.buf[:2])[0]
            if len(self.buf) < 2 + msglen:
                break

            payload = bytes(self.buf[2 : 2 + msglen])
            del self.buf[: 2 + msglen]
            msgs.append(payload)

        return msgs


def split_memwrite(payload: bytes, chunk_size: int = MEMWRITE_CHUNK):
    """
    Input payload format from osmoload:
        [cmd=0x08][nbytes][crc16][address][data...]

    Returns None for non-MEM_WRITE payloads.
    Returns a dict describing split chunks otherwise.
    """
    if len(payload) < 8:
        return None

    cmd = payload[0]
    if cmd != LOADER_MEM_WRITE:
        return None

    total_nbytes = payload[1]
    if len(payload) < 8 + total_nbytes:
        return None

    orig_crc = struct.unpack(">H", payload[2:4])[0]
    base_addr = struct.unpack(">I", payload[4:8])[0]
    data = payload[8 : 8 + total_nbytes]

    chunks = []
    off = 0
    while off < len(data):
        chunk = data[off : off + chunk_size]
        chunk_crc = osmo_crc16(chunk)

        sub = bytearray()
        sub.append(LOADER_MEM_WRITE)
        sub.append(len(chunk))
        sub.extend(struct.pack(">H", chunk_crc))
        sub.extend(struct.pack(">I", base_addr + off))
        sub.extend(chunk)

        chunks.append(bytes(sub))
        off += len(chunk)

    return {
        "original_cmd": cmd,
        "original_nbytes": total_nbytes,
        "original_crc": orig_crc,
        "original_addr": base_addr,
        "chunks": chunks,
    }


def make_socket_msg(payload: bytes) -> bytes:
    return struct.pack(">H", len(payload)) + payload


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <pty-path>", file=sys.stderr)
        sys.exit(1)

    pty_path = sys.argv[1]

    try:
        os.unlink(SOCK_PATH)
    except FileNotFoundError:
        pass

    pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
    set_raw(pty_fd)
    set_nonblock(pty_fd)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK_PATH)
    srv.listen(1)
    srv.setblocking(False)

    print(f"sercomm-relay: listening on {SOCK_PATH}, PTY={pty_path}", flush=True)
    print(
        f"sercomm-relay: TX DLCI={DLCI_TX}, RX DLCI={DLCI_RX}, CTRL=0x{CTRL:02x}",
        flush=True,
    )

    parser = SercommParser()
    lp_reader = LengthPrefixReader()

    client = None
    running = True

    # For split MEM_WRITE aggregation.
    pending_reply = None

    def shutdown(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        while running:
            rlist = [srv, pty_fd]
            if client is not None:
                rlist.append(client)

            try:
                readable, _, _ = select.select(rlist, [], [], 0.5)
            except (OSError, ValueError):
                break

            if srv in readable:
                conn, _ = srv.accept()
                conn.setblocking(False)

                if client is not None:
                    try:
                        client.close()
                    except OSError:
                        pass

                client = conn
                lp_reader = LengthPrefixReader()
                pending_reply = None
                print("sercomm-relay: client connected", flush=True)

            if client is not None and client in readable:
                try:
                    raw = client.recv(4096)
                except BlockingIOError:
                    raw = b""

                if not raw:
                    print("sercomm-relay: client disconnected", flush=True)
                    try:
                        client.close()
                    except OSError:
                        pass
                    client = None
                    pending_reply = None
                else:
                    print(f"  [sock raw] {hexdump(raw)}", flush=True)

                    for payload in lp_reader.feed(raw):
                        print(f"  [sock msg] payload={hexdump(payload)}", flush=True)

                        split = split_memwrite(payload, chunk_size=MEMWRITE_CHUNK)

                        if split is None:
                            frame = sercomm_wrap(DLCI_TX, payload)
                            print(f"  [->PTY] {hexdump(frame)}", flush=True)
                            try:
                                os.write(pty_fd, frame)
                            except OSError as e:
                                if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                                    raise
                        else:
                            pending_reply = {
                                "cmd": split["original_cmd"],
                                "nbytes": split["original_nbytes"],
                                "crc": split["original_crc"],
                                "addr": split["original_addr"],
                                "remaining": len(split["chunks"]),
                            }

                            print(
                                f"  [memwrite split] "
                                f"addr=0x{pending_reply['addr']:08x} "
                                f"total={pending_reply['nbytes']} "
                                f"chunks={pending_reply['remaining']}",
                                flush=True,
                            )

                            for idx, sub in enumerate(split["chunks"], start=1):
                                frame = sercomm_wrap(DLCI_TX, sub)
                                print(
                                    f"  [->PTY sub {idx}/{len(split['chunks'])}] "
                                    f"{hexdump(frame)}",
                                    flush=True,
                                )
                                try:
                                    os.write(pty_fd, frame)
                                except OSError as e:
                                    if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                                        raise

            if pty_fd in readable:
                try:
                    raw = os.read(pty_fd, 4096)
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                        raw = b""
                    else:
                        raise

                if not raw:
                    print("sercomm-relay: PTY closed", flush=True)
                    break

                print(f"  [PTY raw] {hexdump(raw)}", flush=True)

                for dlci, payload in parser.feed(raw):
                    print(f"  [PTY frame] dlci={dlci} payload={hexdump(payload)}", flush=True)

                    if dlci != DLCI_RX or client is None:
                        continue

                    # Aggregate split MEM_WRITE replies and synthesize one final reply
                    # matching osmoload's original expectation.
                    if (
                        pending_reply is not None
                        and len(payload) >= 8
                        and payload[0] == LOADER_MEM_WRITE
                    ):
                        pending_reply["remaining"] -= 1
                        print(
                            f"  [memwrite ack] remaining={pending_reply['remaining']}",
                            flush=True,
                        )

                        if pending_reply["remaining"] == 0:
                            synth = bytearray()
                            synth.append(pending_reply["cmd"])
                            synth.append(pending_reply["nbytes"])
                            synth.extend(struct.pack(">H", pending_reply["crc"]))
                            synth.extend(struct.pack(">I", pending_reply["addr"]))

                            msg = make_socket_msg(bytes(synth))
                            print(f"  [->sock synth] {hexdump(msg)}", flush=True)
                            try:
                                client.sendall(msg)
                            except (BrokenPipeError, OSError):
                                try:
                                    client.close()
                                except OSError:
                                    pass
                                client = None

                            pending_reply = None

                        continue

                    # Normal passthrough reply
                    msg = make_socket_msg(payload)
                    print(f"  [->sock] {hexdump(msg)}", flush=True)
                    try:
                        client.sendall(msg)
                    except (BrokenPipeError, OSError):
                        try:
                            client.close()
                        except OSError:
                            pass
                        client = None
                        pending_reply = None

    finally:
        try:
            if client is not None:
                client.close()
        except OSError:
            pass

        try:
            srv.close()
        except OSError:
            pass

        try:
            os.close(pty_fd)
        except OSError:
            pass

        try:
            os.unlink(SOCK_PATH)
        except OSError:
            pass

        print("sercomm-relay: done", flush=True)


if __name__ == "__main__":
    main()

===== ./l1ctl_min.py =====
#!/usr/bin/env python3
"""
l1ctl_min.py — Minimal L1CTL bridge: QEMU firmware ↔ mobile
Synth: RESET_REQ, PM_REQ. Everything else forwarded to firmware.
"""

import errno, fcntl, os, re, select, signal, socket, struct, subprocess, sys, termios, time

# Sercomm
FLAG = 0x7E; ESCAPE = 0x7D; ESCAPE_XOR = 0x20
DLCI_L1CTL = 5; CTRL = 0x03

# L1CTL message types
MSG_RESET_REQ  = 0x0D; MSG_RESET_CONF = 0x0E
MSG_PM_REQ = 0x08; MSG_PM_CONF = 0x09

TARGET_ARFCN = int(os.environ.get("TARGET_ARFCN", "514"))


def sercomm_wrap(dlci, payload):
    frame = bytes([dlci, CTRL, len(payload) >> 8, len(payload) & 0xFF]) + payload
    out = bytearray([FLAG])
    for b in frame:
        if b in (FLAG, ESCAPE):
            out += bytes([ESCAPE, b ^ ESCAPE_XOR])
        else:
            out.append(b)
    out.append(FLAG)
    return bytes(out)


class SercommParser:
    def __init__(self):
        self.buf = bytearray()
        self.in_frame = False
        self.escape = False

    def feed(self, data):
        frames = []
        for b in data:
            if b == FLAG:
                if self.in_frame and len(self.buf) >= 4:
                    dlci = self.buf[0]
                    plen = (self.buf[2] << 8) | self.buf[3]
                    payload = bytes(self.buf[4:4+plen])
                    frames.append((dlci, payload))
                self.buf = bytearray()
                self.in_frame = True
                self.escape = False
            elif self.in_frame:
                if self.escape:
                    self.buf.append(b ^ ESCAPE_XOR)
                    self.escape = False
                elif b == ESCAPE:
                    self.escape = True
                else:
                    self.buf.append(b)
        return frames


def l1ctl_send(sock, msg_type, payload=b""):
    data = bytes([msg_type, 0, 0, 0]) + payload
    msg = struct.pack(">H", len(data)) + data
    sock.sendall(msg)


def launch_qemu(kernel, qemu_bin, mon_sock, env_extra=None):
    try: os.unlink(mon_sock)
    except FileNotFoundError: pass

    env = os.environ.copy()
    if env_extra:
        env.update(env_extra)

    cmd = [qemu_bin, "-M", "calypso", "-cpu", "arm946", "-display", "none",
           "-serial", "pty", "-serial", "pty",
           "-monitor", f"unix:{mon_sock},server,nowait", "-kernel", kernel]
    print(f"l1ctl-min: QEMU: {' '.join(cmd)}", flush=True)

    log_fd = open("/tmp/qemu-min.log", "w")
    proc = subprocess.Popen(cmd, stdout=log_fd, stderr=subprocess.STDOUT, env=env)

    # Detect PTY from log
    pty_path = None
    for _ in range(100):
        time.sleep(0.1)
        with open("/tmp/qemu-min.log") as f:
            for line in f:
                m = re.search(r'char device redirected to (/dev/pts/\d+)', line)
                if m:
                    pty_path = m.group(1)
                    break
        if pty_path: break

    if not pty_path:
        print("l1ctl-min: ERROR: no PTY found", flush=True)
        proc.kill()
        sys.exit(1)

    print(f"l1ctl-min: PTY={pty_path}", flush=True)

    # Send 'cont' to QEMU monitor
    time.sleep(1)
    try:
        import subprocess as sp
        sp.run(["socat", "-", f"UNIX-CONNECT:{mon_sock}"],
               input=b"cont\n", timeout=3, capture_output=True)
        print("l1ctl-min: sent 'cont'", flush=True)
    except Exception:
        pass

    # Open PTY
    fd = os.open(pty_path, os.O_RDWR | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0; attrs[1] = 0; attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0; attrs[4] = termios.B115200; attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0; attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    return fd, proc


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("-k", "--kernel", required=True)
    p.add_argument("-s", "--socket", default="/tmp/osmocom_l2_1")
    p.add_argument("--qemu-bin", default="/opt/GSM/qemu/build/qemu-system-arm")
    p.add_argument("--env", action="append", default=[])
    args = p.parse_args()

    env_extra = {}
    for e in args.env:
        k, v = e.split("=", 1)
        env_extra[k] = v

    # POWERON to fake_trx
    trxc_port = int(os.environ.get("CALYPSO_TRX_PORT", "0"))
    if trxc_port > 0:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dest = ("127.0.0.1", trxc_port + 1)
        if TARGET_ARFCN >= 512 and TARGET_ARFCN <= 885:
            dl = 1805200 + (TARGET_ARFCN - 512) * 200
            ul = 1710200 + (TARGET_ARFCN - 512) * 200
        else:
            dl = 935000 + TARGET_ARFCN * 200
            ul = 890000 + TARGET_ARFCN * 200
        for cmd in [f"CMD RXTUNE {dl}", f"CMD TXTUNE {ul}", "CMD POWERON"]:
            s.sendto(cmd.encode() + b'\0', dest)
            print(f"l1ctl-min: → fake_trx: {cmd}", flush=True)
        time.sleep(0.5)
        s.close()

    # Launch QEMU
    pty_fd, qemu_proc = launch_qemu(args.kernel, args.qemu_bin,
                                     "/tmp/qemu-calypso-mon-min.sock", env_extra)
    time.sleep(3)

    # L1CTL server socket
    sock_path = args.socket
    try: os.unlink(sock_path)
    except FileNotFoundError: pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)
    srv.setblocking(False)
    print(f"l1ctl-min: listening on {sock_path}", flush=True)

    parser = SercommParser()
    client = None
    running = True

    def shutdown(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    while running:
        rlist = [pty_fd, srv]
        if client:
            rlist.append(client)
        try:
            readable, _, _ = select.select(rlist, [], [], 0.1)
        except (ValueError, OSError):
            break

        # Accept new client
        if srv in readable:
            conn, _ = srv.accept()
            conn.setblocking(False)
            if client:
                try: client.close()
                except: pass
            client = conn
            print("l1ctl-min: client connected", flush=True)

        # PTY → firmware sends L1CTL to mobile
        if pty_fd in readable:
            try:
                data = os.read(pty_fd, 4096)
            except OSError:
                break
            if not data:
                break
            for dlci, payload in parser.feed(data):
                if dlci == DLCI_L1CTL and client and len(payload) > 0:
                    msg_type = payload[0]
                    msg = struct.pack(">H", len(payload)) + payload
                    try:
                        client.sendall(msg)
                        print(f"  [fw→mobile] type=0x{msg_type:02x} len={len(payload)}", flush=True)
                    except (BrokenPipeError, OSError):
                        client = None

        # Mobile → L1CTL to firmware
        if client and client in readable:
            try:
                data = client.recv(4096)
            except (ConnectionResetError, OSError):
                data = b""
            if not data:
                print("l1ctl-min: client disconnected", flush=True)
                client = None
                continue

            # Parse L1CTL: 2-byte length + payload
            while len(data) >= 2:
                plen = struct.unpack(">H", data[:2])[0]
                if len(data) < 2 + plen:
                    break
                payload = data[2:2+plen]
                data = data[2+plen:]
                msg_type = payload[0] if payload else 0

                # Synth RESET_REQ
                if msg_type == MSG_RESET_REQ:
                    rt = payload[4] if len(payload) > 4 else 1
                    l1ctl_send(client, MSG_RESET_CONF, struct.pack("Bxxx", rt))
                    print(f"  [synth] RESET type={rt}", flush=True)
                    continue

                # Synth PM_REQ
                if msg_type == MSG_PM_REQ and len(payload) >= 12:
                    a_from = (payload[8] << 8) | payload[9]
                    a_to = (payload[10] << 8) | payload[11]
                    entries = []
                    for a in range(a_from, a_to + 1):
                        rxlev = 0x30 if a == TARGET_ARFCN else 0x00
                        entries.append(struct.pack(">HBB", a, rxlev, rxlev))
                    hdr = bytes([MSG_PM_CONF, 0x01, 0x00, 0x00])
                    pm_data = hdr + b"".join(entries)
                    msg = struct.pack(">H", len(pm_data)) + pm_data
                    client.sendall(msg)
                    print(f"  [synth] PM {a_from}-{a_to}", flush=True)
                    continue

                # Forward everything else to firmware
                frame = sercomm_wrap(DLCI_L1CTL, payload)
                try:
                    os.write(pty_fd, frame)
                    print(f"  [mobile→fw] type=0x{msg_type:02x} len={len(payload)}", flush=True)
                except OSError:
                    pass

    print("l1ctl-min: done", flush=True)
    qemu_proc.terminate()


if __name__ == "__main__":
    main()

===== ./REFACTORING.md =====
# Code Refactoring Guide

## Current File Structure

```
hw/arm/calypso/
  calypso_c54x.c    (~2600 lines) - TMS320C54x DSP emulator
  calypso_c54x.h    (~170 lines)  - DSP state struct, constants
  calypso_trx.c     (~610 lines)  - Calypso HW glue (DSP-ARM, TPU, TSP, SIM)
  calypso_trx.h     (~80 lines)   - TRX definitions, IRQ numbers
  calypso_tint0.c   (~100 lines)  - TINT0 master clock (NEW)
  calypso_tint0.h   (~55 lines)   - TINT0 constants (NEW)
  calypso_mb.c                     - Machine board
  calypso_soc.c                    - SoC init
  l1ctl_sock.c                     - L1CTL unix socket (firmware <-> mobile)
  meson.build                      - Build config
```

## What Was Refactored

### TINT0 Extraction
- **Before**: Timer creation, tick callback, start/stop all inside calypso_trx.c
- **After**: calypso_tint0.c owns the QEMU timer. calypso_trx.c exposes `calypso_tint0_do_tick(fn)` as the work callback.
- **Removed**: calypso_tdma_hw.c/h (was a "slave" TDMA counter, never connected to build)

### What Still Needs Refactoring

1. **Remove DSP internal timer from c54x main loop**
   - The per-instruction TIM/PRD/TCR tick at lines ~2490-2520 of calypso_c54x.c
   - TINT0 should be the only timer source
   - Remove the FN increment from timer expiry (FN comes from calypso_tint0)

2. **Clean up duplicate F0/F1 handler blocks in c54x**
   - Two identical `if (hi8 == 0xF0 || hi8 == 0xF1)` blocks
   - The second one (starting around line 736) is dead code
   - Delete the second block entirely

3. **Separate DSP instruction decoder into its own file**
   - calypso_c54x.c is 2600 lines and growing
   - Could split: c54x_decode.c (instruction handlers), c54x_core.c (exec loop, memory, interrupts), c54x_rom.c (ROM loader)

4. **calypso_trx.c does too much**
   - Currently handles: DSP API, TPU regs, TSP regs, ULPD regs, SIM regs, bootloader protocol, burst I/O, frame processing
   - Could split: calypso_tpu.c, calypso_sim.c, calypso_dsp_api.c

5. **Normalize header includes**
   - Some files use `#include "hw/arm/calypso/foo.h"`, others use `#include "foo.h"`
   - Standardize to one convention

===== ./gsm_clock.py =====
#!/usr/bin/env python3
"""
gsm_clock.py — Master GSM TDMA clock for the Calypso QEMU stack.

Single source of truth for frame number (FN). Sends air bursts to both
QEMU instances (BTS TRX + Mobile layer1) at GSM cadence so they stay
synchronized.  Every component derives timing from this clock.

Architecture:
  gsm_clock.py  --UDP air burst-->  QEMU TRX  (port 4800)
                --UDP air burst-->  QEMU MS   (port 4801)
                --UDP air burst-->  bridge    (port 4901)

Burst format (same as QEMU air interface):
  byte 0      : TN (timeslot)
  bytes 1..4  : FN (big-endian uint32)
  bytes 5..152: 148 hard bits (0/1)

Usage:
  python3 gsm_clock.py [-r RATE] [--trx-port 4800] [--ms-port 4801] [--bridge-port 4901]

Rate: 1.0 = real GSM (4.615ms/frame), 0.1 = 10x slower (46.15ms, matches QEMU default)
"""

import argparse
import socket
import struct
import time
import signal
import sys

GSM_HYPERFRAME = 2715648
GSM_FRAME_US = 4615  # 4.615 ms in microseconds

# 51-multiframe structure for TS0 (combined CCCH+SDCCH/4)
# FCCH: 0,10,20,30,40  SCH: 1,11,21,31,41  BCCH: 2-5  CCCH: 6-9,12-15,16-19,...
FCCH_FN51 = {0, 10, 20, 30, 40}
SCH_FN51  = {1, 11, 21, 31, 41}

# FCCH burst: all zeros (frequency correction)
FCCH_BURST = bytes(148)

# SCH training sequence (midamble bits 42..105)
SCH_TRAIN = [
    1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,
    0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1,
]

# Normal burst fill pattern (idle CCCH = 2B padding)
IDLE_L2 = bytes([0x03, 0x03, 0x01, 0x2B] + [0x2B] * 19)  # 23 bytes


def make_sch_burst(bsic, fn):
    """Build a 148-bit SCH burst with encoded BSIC+FN."""
    t1 = fn // (26 * 51)
    t2 = fn % 26
    t3 = fn % 51
    t3p = (t3 - 1) // 10 if t3 > 0 else 0

    # 25 info bits: spare(2) + BSIC(6) + T1(11) + T2(5) + T3'(3)
    info = [0, 0]
    for i in range(5, -1, -1):
        info.append((bsic >> i) & 1)
    for i in range(10, -1, -1):
        info.append((t1 >> i) & 1)
    for i in range(4, -1, -1):
        info.append((t2 >> i) & 1)
    for i in range(2, -1, -1):
        info.append((t3p >> i) & 1)

    # CRC (10 bits) + tail (4 bits) — simplified: zeros
    full = info + [0] * 10 + [0] * 4  # 39 bits

    # Convolutional encode (rate 1/2, K=5)
    reg = 0
    coded = []
    for b in full:
        reg = ((reg << 1) | b) & 0x1F
        g0 = ((reg >> 0) ^ (reg >> 3) ^ (reg >> 4)) & 1
        g1 = ((reg >> 0) ^ (reg >> 1) ^ (reg >> 3) ^ (reg >> 4)) & 1
        coded.extend([g0, g1])
    # 78 coded bits → split into 2x39

    # SCH burst: 3 tail + 39 coded + 64 train + 39 coded + 3 tail + guard
    burst = [0]*3 + coded[:39] + SCH_TRAIN + coded[39:78] + [0]*3
    burst = (burst + [0]*148)[:148]
    return bytes(burst)


def make_normal_burst(data_bits_114, tsc=0):
    """Build a 148-bit normal burst from 114 coded data bits."""
    # TSC midambles
    TSC_BITS = [
        [0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1],
        [0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1],
        [0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0],
        [0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0],
        [0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1],
        [0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0],
        [1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1],
        [1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0],
    ]
    d = list(data_bits_114)
    if len(d) < 114:
        d.extend([0] * (114 - len(d)))
    mid = TSC_BITS[tsc % 8]
    burst = [0]*3 + d[:57] + [0] + mid + [0] + d[57:114] + [0]*3 + [0]*8
    return bytes((burst + [0]*148)[:148])


def encode_l2_to_coded(l2_bytes):
    """Encode 23 L2 bytes → 456 coded bits (conv code), return 4x114 interleaved."""
    # 184 info bits
    info = []
    for byte in l2_bytes[:23]:
        for i in range(7, -1, -1):
            info.append((byte >> i) & 1)
    info = (info + [0]*184)[:184]

    # + 40 parity (zeros simplified) + 4 tail
    full = info + [0]*40 + [0]*4  # 228 bits

    # Conv encode
    reg = 0
    coded = []
    for b in full:
        reg = ((reg << 1) | b) & 0x1F
        g0 = ((reg >> 0) ^ (reg >> 3) ^ (reg >> 4)) & 1
        g1 = ((reg >> 0) ^ (reg >> 1) ^ (reg >> 3) ^ (reg >> 4)) & 1
        coded.extend([g0, g1])

    # Interleave into 4 bursts of 114 bits
    bursts = [[0]*114 for _ in range(4)]
    for k in range(456):
        bursts[k % 4][k // 4] = coded[k] if k < len(coded) else 0
    return bursts


class GSMClock:
    def __init__(self, rate, bsic, arfcn, targets):
        self.rate = rate
        self.bsic = bsic
        self.arfcn = arfcn
        self.fn = 0
        self.targets = targets  # list of (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True

        # Pre-encode idle CCCH burst data (4 bursts per block)
        self.idle_bursts_114 = encode_l2_to_coded(IDLE_L2)

    def send_burst(self, tn, fn, burst_bytes):
        """Send air burst to all targets."""
        pkt = struct.pack(">BI", tn, fn) + burst_bytes[:148]
        for target in self.targets:
            try:
                self.sock.sendto(pkt, target)
            except OSError:
                pass

    def run(self):
        frame_ns = GSM_FRAME_US * 1000  # nanoseconds
        if self.rate != 1.0:
            frame_ns = int(frame_ns / self.rate)

        print(f"gsm-clock: BSIC={self.bsic} ARFCN={self.arfcn}", flush=True)
        print(f"gsm-clock: rate={self.rate}x → {frame_ns/1e6:.3f}ms/frame", flush=True)
        print(f"gsm-clock: targets={self.targets}", flush=True)
        print(f"gsm-clock: starting TDMA...", flush=True)

        t_start = time.monotonic_ns()
        frames_sent = 0

        while self.running:
            fn = self.fn
            fn51 = fn % 51

            # TS0: generate appropriate burst based on 51-multiframe position
            if fn51 in FCCH_FN51:
                self.send_burst(0, fn, FCCH_BURST)
            elif fn51 in SCH_FN51:
                sch = make_sch_burst(self.bsic, fn)
                self.send_burst(0, fn, sch)
            else:
                # BCCH (2-5) or CCCH (6-9, 12-15, ...) or idle
                # Determine burst index within 4-burst block
                block_starts = [2, 6, 12, 16, 22, 26, 32, 36, 42, 46]
                burst_idx = None
                for start in block_starts:
                    if start <= fn51 < start + 4:
                        burst_idx = fn51 - start
                        break

                if burst_idx is not None:
                    coded_114 = self.idle_bursts_114[burst_idx]
                    nb = make_normal_burst(coded_114, tsc=self.bsic & 0x7)
                    self.send_burst(0, fn, nb)
                # else: idle frame (50), send nothing

            # Advance FN
            self.fn = (self.fn + 1) % GSM_HYPERFRAME
            frames_sent += 1

            # Log periodically
            if frames_sent <= 5 or frames_sent % 5000 == 0:
                elapsed_s = (time.monotonic_ns() - t_start) / 1e9
                fps = frames_sent / elapsed_s if elapsed_s > 0 else 0
                print(f"gsm-clock: FN={fn} frames={frames_sent} "
                      f"elapsed={elapsed_s:.1f}s fps={fps:.1f}", flush=True)

            # Precise sleep: target absolute time for next frame
            target_ns = t_start + frames_sent * frame_ns
            now_ns = time.monotonic_ns()
            sleep_ns = target_ns - now_ns
            if sleep_ns > 0:
                time.sleep(sleep_ns / 1e9)


def main():
    ap = argparse.ArgumentParser(description="Master GSM TDMA clock")
    ap.add_argument("-r", "--rate", type=float, default=0.1,
                    help="Clock rate (1.0=real GSM, 0.1=10x slow for QEMU, default: 0.1)")
    ap.add_argument("--bsic", type=int, default=63,
                    help="BSIC (default: 63)")
    ap.add_argument("--arfcn", type=int, default=100,
                    help="ARFCN (default: 100)")
    ap.add_argument("--ip", default="127.0.0.1",
                    help="Target IP (default: 127.0.0.1)")
    args = ap.parse_args()

    # Standard port scheme only: 6700 (transceiver MS-side), 6800 (bridge/mobile)
    targets = [
        (args.ip, 6700),  # transceiver MS-side air → CLK IND → 5800
        (args.ip, 6800),  # bridge/mobile DL air
    ]

    clock = GSMClock(args.rate, args.bsic, args.arfcn, targets)

    def stop(_s, _f):
        clock.running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    clock.run()
    print(f"gsm-clock: stopped at FN={clock.fn}", flush=True)


if __name__ == "__main__":
    main()

===== ./sync.sh =====
#!/bin/bash
# Sync qemu-src between host and osmo-operator-1 container
# Usage: ./sync.sh [push|pull|check]
#   pull  = container → host (default)
#   push  = host → container
#   check = compare only

CONTAINER="osmo-operator-1"
HOST_DIR="/home/nirvana/qemu-src"
CONT_DIR="/opt/GSM/qemu-src"

FILES=(
  hw/arm/calypso/calypso_trx.c
  hw/arm/calypso/calypso_c54x.c
  hw/arm/calypso/calypso_c54x.h
  hw/arm/calypso/calypso_tdma_hw.c
  hw/arm/calypso/calypso_tdma_hw.h
  hw/arm/calypso/calypso_soc.c
  hw/arm/calypso/calypso_mb.c
  hw/arm/calypso/l1ctl_sock.c
  hw/arm/calypso/meson.build
  hw/char/calypso_uart.c
  hw/intc/calypso_inth.c
  include/hw/arm/calypso/calypso_trx.h
  include/hw/arm/calypso/calypso_soc.h
  include/hw/arm/calypso/calypso_uart.h
  bridge.py
  run.sh
)

MODE="${1:-pull}"
DIRTY=0

for f in "${FILES[@]}"; do
  h=$(md5sum "$HOST_DIR/$f" 2>/dev/null | cut -d' ' -f1)
  c=$(docker exec "$CONTAINER" md5sum "$CONT_DIR/$f" 2>/dev/null | cut -d' ' -f1)

  if [ -z "$h" ] && [ -z "$c" ]; then
    continue
  elif [ "$h" = "$c" ]; then
    echo "✅ $f"
  else
    DIRTY=1
    if [ "$MODE" = "check" ]; then
      echo "❌ $f  host=${h:-MISSING} cont=${c:-MISSING}"
    elif [ "$MODE" = "pull" ]; then
      docker cp "$CONTAINER:$CONT_DIR/$f" "$HOST_DIR/$f"
      echo "⬇️  $f  (pulled)"
    elif [ "$MODE" = "push" ]; then
      docker cp "$HOST_DIR/$f" "$CONTAINER:$CONT_DIR/$f"
      echo "⬆️  $f  (pushed)"
    fi
  fi
done

if [ "$DIRTY" = "0" ]; then
  echo "All files in sync."
elif [ "$MODE" = "check" ]; then
  echo "Files out of sync. Use: ./sync.sh pull|push"
fi

# Snapshot on push
if [ "$MODE" = "push" ] && [ "$DIRTY" = "1" ]; then
  STAMP=$(date +%Y%m%d-%H%M%S)
  EVENT="${2:-}"
  if [ -n "$EVENT" ]; then
    SNAP="/home/nirvana/ALL-QEMUs/qemu-calypso-${STAMP}-${EVENT}"
  else
    SNAP="/home/nirvana/ALL-QEMUs/qemu-calypso-${STAMP}"
  fi
  mkdir -p "$SNAP"
  for f in "${FILES[@]}"; do
    mkdir -p "$SNAP/$(dirname $f)"
    cp "$HOST_DIR/$f" "$SNAP/$f" 2>/dev/null
  done
  echo "$STAMP ${EVENT:-push}" >> "/home/nirvana/ALL-QEMUs/HISTORY.log"
  echo "📸 Snapshot: $SNAP"
fi

# Check binary age vs source files
BINARY="$CONT_DIR/build/qemu-system-arm"
BIN_TS=$(docker exec "$CONTAINER" stat -c %Y "$BINARY" 2>/dev/null)
if [ -n "$BIN_TS" ]; then
  BIN_DATE=$(docker exec "$CONTAINER" date -d "@$BIN_TS" "+%H:%M:%S" 2>/dev/null)
  STALE=0
  for f in "${FILES[@]}"; do
    [[ "$f" == *.py || "$f" == *.sh ]] && continue
    SRC_TS=$(docker exec "$CONTAINER" stat -c %Y "$CONT_DIR/$f" 2>/dev/null)
    if [ -n "$SRC_TS" ] && [ "$SRC_TS" -gt "$BIN_TS" ]; then
      SRC_DATE=$(docker exec "$CONTAINER" date -d "@$SRC_TS" "+%H:%M:%S" 2>/dev/null)
      echo "⚠️  STALE: $f ($SRC_DATE) newer than binary ($BIN_DATE)"
      STALE=1
    fi
  done
  if [ "$STALE" = "0" ]; then
    echo "🔨 Binary up to date ($BIN_DATE)"
  else
    echo "🔨 Binary OUTDATED ($BIN_DATE) — run: ninja -C build"
  fi
else
  echo "⚠️  Binary not found at $BINARY"
fi

