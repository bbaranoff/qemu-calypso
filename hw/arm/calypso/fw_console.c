/*
 * fw_console.c — diagnostic poller for the layer1 firmware printf_buffer
 *
 * The osmocom-bb compal_e88 firmware (layer1.highram.elf) builds printf
 * output in `printf_buffer` (symbol at 0x00831018) via cons_puts. On real
 * hardware cons_puts pushes the assembled string to the LCD framebuffer
 * (fb_bw8_putstr at 0x82a1b4); QEMU does not emulate the LCD, so the
 * strings just sit in RAM and get overwritten on the next printf.
 *
 * This poller wakes every FW_POLL_MS simulated milliseconds, snapshots the
 * buffer via cpu_physical_memory_read, and emits the string to
 * /tmp/qemu-fw-console.log + stderr whenever its content changes. Polling
 * misses printfs that get overwritten between two ticks; for noisy paths
 * lower FW_POLL_MS or mirror the buffer to stderr at additional event
 * sites (DSP IDLE, IRQ entry, etc.).
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "exec/cpu-common.h"
#include "hw/arm/calypso/fw_console.h"

#define FW_PRINTF_ADDR  0x00831018u
#define FW_PRINTF_LEN   512u
#define FW_POLL_MS      10u

static QEMUTimer *fw_poll_timer;
static FILE *fw_log_fp;
static uint8_t fw_last[FW_PRINTF_LEN];

static void fw_console_emit(const uint8_t *buf, size_t len)
{
    while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r'))
        len--;
    if (len == 0)
        return;
    if (fw_log_fp) {
        fprintf(fw_log_fp, "[fw] %.*s\n", (int)len, buf);
        fflush(fw_log_fp);
    }
    fprintf(stderr, "[fw-console] %.*s\n", (int)len, buf);
}

static void fw_console_poll(void *opaque)
{
    uint8_t cur[FW_PRINTF_LEN];
    cpu_physical_memory_read(FW_PRINTF_ADDR, cur, FW_PRINTF_LEN);

    if (memcmp(cur, fw_last, FW_PRINTF_LEN) != 0) {
        size_t len = 0;
        while (len < FW_PRINTF_LEN && cur[len] != 0)
            len++;
        if (len > 0)
            fw_console_emit(cur, len);
        memcpy(fw_last, cur, FW_PRINTF_LEN);
    }

    timer_mod_ns(fw_poll_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 (uint64_t)FW_POLL_MS * 1000000ULL);
}

void fw_console_init(void)
{
    fw_log_fp = fopen("/tmp/qemu-fw-console.log", "w");
    if (fw_log_fp) {
        setvbuf(fw_log_fp, NULL, _IOLBF, 0);
        fprintf(fw_log_fp,
                "# QEMU firmware console - poll printf_buffer @ 0x%08x every %u ms\n",
                FW_PRINTF_ADDR, FW_POLL_MS);
    }

    fw_poll_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fw_console_poll, NULL);
    timer_mod_ns(fw_poll_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 (uint64_t)FW_POLL_MS * 1000000ULL);

    fprintf(stderr,
            "[fw-console] polling 0x%08x every %u ms -> /tmp/qemu-fw-console.log\n",
            FW_PRINTF_ADDR, FW_POLL_MS);
}
