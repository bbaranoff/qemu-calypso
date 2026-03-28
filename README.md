# QEMU Calypso — GSM Baseband Emulation

Emulation of the TI Calypso GSM baseband processor in QEMU 9.2.4, running the OsmocomBB Layer 1 firmware. Full L1CTL protocol support enables OsmocomBB's `mobile` application to perform cell selection against the emulated phone.

## What Works

```
Mobile (layer23)        L1CTL Bridge           QEMU Calypso (firmware)
     |                       |                        |
     |── RESET_REQ ─────────>|──── sercomm ──────────>|  boot + DSP init
     |<─ RESET_CONF ─────────|<─── sercomm ───────────|
     |                       |                        |
     |── PM_REQ (3 bands) ──>|  INTERCEPT             |
     |<─ PM_CONF ────────────|  ARFCN 1022 = -62 dBm  |  (not forwarded)
     |                       |                        |
     |── FBSB_REQ ──────────>|──── sercomm ──────────>|  FCCH/SCH search
     |<─ FBSB_CONF ─────────|<─── sercomm ───────────|  result=255 (no BTS)
```

- RESET_REQ → RESET_CONF
- PM_REQ → PM_CONF (E-GSM 940-1023, GSM900 0-124, DCS1800 512-885)
- FBSB_REQ → FBSB_CONF (result=255 without BTS, ready for real radio)

## Quick Start

### Option 1: Automated build

```bash
./download_and_patch.sh
cd qemu-calypso-build/qemu-9.2.4
./run-mobile.sh /path/to/layer1.highram.elf
```

### Option 2: Manual

```bash
# Build
./configure --target-list=arm-softmmu --disable-docs
cd build && ninja -j$(nproc)

# Run
cd ..
./build/qemu-system-arm -M calypso -cpu arm946 \
  -serial pty -serial pty \
  -monitor unix:/tmp/qemu-calypso-mon.sock,server,nowait \
  -kernel /path/to/layer1.highram.elf

# In another terminal: unpause
echo cont | socat - UNIX-CONNECT:/tmp/qemu-calypso-mon.sock

# Bridge
python3 l1ctl_bridge.py /dev/pts/X   # X from QEMU output

# Mobile
mobile
```

### Prerequisites

```bash
sudo apt install build-essential ninja-build python3-venv \
  libglib2.0-dev libpixman-1-dev wget socat
```

OsmocomBB `mobile` binary and `layer1.highram.elf` firmware required separately.

## Architecture

```
┌─────────────┐    unix socket     ┌──────────────┐   sercomm/PTY    ┌─────────────────┐
│   mobile    │◄──────────────────►│ l1ctl_bridge │◄────────────────►│  QEMU Calypso   │
│  (layer23)  │  /tmp/osmocom_l2  │    (.py)     │  /dev/pts/X     │ ARM946 + L1 fw  │
└─────────────┘                   └──────────────┘                  └────────┬────────┘
                                    PM intercept                             │
                                    (fake PM_CONF)                    TRXD/TRXC UDP
                                                                      6700/6701
                                                                    ┌────────▼────────┐
                                                                    │  osmo-bts-trx   │
                                                                    │  or SDR/cfile   │
                                                                    │   (future)      │
                                                                    └─────────────────┘
```

## File Map

### QEMU Hardware (C)

| File | Description |
|------|-------------|
| `hw/arm/calypso/calypso_mb.c` | Machine board: CPU, RAM, Flash, boot ROM |
| `hw/arm/calypso/calypso_soc.c` | SoC: peripheral instantiation, memory map |
| `hw/arm/calypso/calypso_trx.c` | DSP/TPU/TRX/SIM/ULPD + firmware runtime patches |
| `hw/char/calypso_uart.c` | UART with TX burst drain for sercomm |
| `hw/intc/calypso_inth.c` | 32-input level-sensitive interrupt controller |
| `hw/timer/calypso_timer.c` | Hardware timers |
| `hw/ssi/calypso_spi.c` | SPI + TWL3025 ABB power management |
| `hw/ssi/calypso_i2c.c` | I2C controller |

### Headers

| File | Description |
|------|-------------|
| `include/hw/arm/calypso/calypso_trx.h` | DSP API defines, PM calibration, sync state machine |
| `include/hw/arm/calypso/calypso_uart.h` | UART state with TX burst drain |
| `include/hw/arm/calypso/calypso_soc.h` | SoC type declarations |
| `include/hw/arm/calypso/calypso_inth.h` | INTH type declarations |
| `include/hw/arm/calypso/calypso_timer.h` | Timer type declarations |
| `include/hw/arm/calypso/calypso_spi.h` | SPI type declarations |

### Scripts

| File | Description |
|------|-------------|
| `l1ctl_bridge.py` | Sercomm relay + PM_REQ interception (ARFCN 1022) |
| `run-mobile.sh` | One-command launcher: QEMU + bridge + mobile |
| `download_and_patch.sh` | Download QEMU 9.2.4, patch, build |

### Build Integration

| File | Change |
|------|--------|
| `hw/arm/calypso/Kconfig` | CONFIG_CALYPSO definition |
| `hw/arm/calypso/meson.build` | Calypso source list |
| `hw/arm/meson.build` | `subdir('calypso')` |
| `hw/char/meson.build` | `calypso_uart.c` |
| `hw/intc/meson.build` | `calypso_inth.c` |
| `hw/timer/meson.build` | `calypso_timer.c` |
| `hw/ssi/meson.build` | `calypso_spi.c`, `calypso_i2c.c` |
| `configs/devices/arm-softmmu/default.mak` | `CONFIG_CALYPSO=y` |

## Firmware Runtime Patches

The emulator patches the OsmocomBB firmware binary at startup to work around timing differences between emulated and real hardware:

| What | Why |
|------|-----|
| NOP `cons_puts`, `puts` | Prevent msgb pool exhaustion from debug output |
| NOP `bl printf` in `frame_irq` | Suppress LOST/debug messages in hot path |
| Expand msgb pool 32→148 | Real pool too small for emulation timing |
| Talloc panic → retry+IRQ | Replace infinite loop with recoverable retry |
| Data abort → IRQs enabled | Prevent system freeze on stray faults |

All patches are applied at first UART MDR1 write (before any console output). Addresses are specific to `layer1.highram.elf` (osmocon_v0.0.0-1784-g9516a0e5-modified).

## Documentation

- [`doc/CALYPSO-QEMU.md`](doc/CALYPSO-QEMU.md) — Full technical documentation
- [`doc/hardware-map.md`](doc/hardware-map.md) — Memory map, peripherals, DSP API layout, firmware symbols

## Next Steps

- Connect `osmo-bts-trx` or SDR to TRXD/TRXC ports (UDP 6700/6701) for real FCCH/SCH bursts
- FBSB_CONF result=0 → BCCH decode → cell camping → location update

## License

GPL-2.0-or-later (same as QEMU)
