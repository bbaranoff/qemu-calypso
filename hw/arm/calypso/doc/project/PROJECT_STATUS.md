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

### Partially Working
- DSP instruction decoding (~90% of C54x ISA)
  - All common instructions implemented
  - F272=RPTBD, F274=CALLD, F273=RETD correctly decoded
  - Indirect addressing modes 0xC-0xF with lk offset
  - CMPS (Viterbi), FIRS (FIR filter), LMS
  - Some F6xx instructions still NOP'd
- DSP init: boots, configures registers, enters init loop
  - SP stable, no more MMR corruption
  - IMR=0x0000 (not yet configured by init code)
  - Does not reach IDLE yet
- L1CTL: RESET_IND/CONF, PM_REQ/CONF, FBSB_REQ work
  - FBSB_CONF sent with result=255 (FB not found)
  - d_dsp_page alternated 2/3 briefly then DSP diverges

### Not Yet Working
- DSP reaching IDLE (init completion)
- DSP frame processing (d_dsp_page ping-pong steady state)
- Frequency burst detection (FBSB result=0)
- SCH decode, CCCH decode
- Network registration
- Voice/data

## Key Bug History (42 bugs fixed across sessions)

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

### Session 2026-04-05 (20 bugs)
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

## Docker Setup
- Container: `trying` (image: osmo-qemu)
- QEMU source: `/opt/GSM/qemu-src/`
- DSP ROM: `/opt/GSM/calypso_dsp.txt`
- Firmware: `/opt/GSM/firmware/board/compal_e88/layer1.highram.elf`
- Launch: `/opt/GSM/qemu-src/run.sh`
- Build: `cd /opt/GSM/qemu-src/build && ninja`
- Host sync: `docker cp trying:/opt/GSM/qemu-src/. /home/nirvana/QEMU/`

## Next Steps
1. Complete TINT0 refactor (remove DSP internal timer, drive IFR bit 4 directly)
2. Debug DSP init loop (why it doesn't reach IDLE)
3. Get d_dsp_page 2/3 alternation stable
4. Fix FBSB detection (result=0 instead of 255)
5. Decode SCH/CCCH
6. Network registration
