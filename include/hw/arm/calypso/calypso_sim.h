#ifndef HW_ARM_CALYPSO_SIM_H
#define HW_ARM_CALYPSO_SIM_H

#include "qemu/osdep.h"
#include "exec/hwaddr.h"
#include "hw/irq.h"

/* Calypso SIM controller register offsets (relative to 0xFFFE0000) */
#define CALYPSO_SIM_REG_CMD      0x00
#define CALYPSO_SIM_REG_STAT     0x02
#define CALYPSO_SIM_REG_CONF1    0x04
#define CALYPSO_SIM_REG_CONF2    0x06
#define CALYPSO_SIM_REG_IT       0x08
#define CALYPSO_SIM_REG_DRX      0x0A
#define CALYPSO_SIM_REG_DTX      0x0C
#define CALYPSO_SIM_REG_MASKIT   0x0E
#define CALYPSO_SIM_REG_IT_CD    0x10

/* CMD bits */
#define CALYPSO_SIM_CMD_CARDRST       (1 << 0)
#define CALYPSO_SIM_CMD_IFRST         (1 << 1)
#define CALYPSO_SIM_CMD_STOP          (1 << 2)
#define CALYPSO_SIM_CMD_START         (1 << 3)
#define CALYPSO_SIM_CMD_MODULE_CLK_EN (1 << 4)

/* STAT bits */
#define CALYPSO_SIM_STAT_NOCARD       (1 << 0)
#define CALYPSO_SIM_STAT_TXPAR        (1 << 1)
#define CALYPSO_SIM_STAT_FIFOFULL     (1 << 2)
#define CALYPSO_SIM_STAT_FIFOEMPTY    (1 << 3)

/* IT bits — interrupt sources */
#define CALYPSO_SIM_IT_NATR           (1 << 0)
#define CALYPSO_SIM_IT_WT             (1 << 1)
#define CALYPSO_SIM_IT_OV             (1 << 2)
#define CALYPSO_SIM_IT_TX             (1 << 3)
#define CALYPSO_SIM_IT_RX             (1 << 4)

/* CONF1 bits */
#define CALYPSO_SIM_CONF1_BYPASS      (1 << 8)
#define CALYPSO_SIM_CONF1_SVCCLEV     (1 << 9)
#define CALYPSO_SIM_CONF1_SRSTLEV     (1 << 10)

typedef struct CalypsoSim CalypsoSim;

CalypsoSim *calypso_sim_new(qemu_irq sim_irq);

uint16_t calypso_sim_reg_read(CalypsoSim *s, hwaddr off);
void     calypso_sim_reg_write(CalypsoSim *s, hwaddr off, uint16_t val);

#endif
