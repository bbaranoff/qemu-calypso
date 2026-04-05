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
