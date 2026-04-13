# Calypso HW — C54x DSP Emulator Context

## Opcode Debug Workflow

1. Find the suspect opcode value (from boot trace or PC HIST)
2. Check `tic54x-opc.c` in binutils: `grep "0xXX" /home/nirvana/gnuarm/src/binutils-2.21.1/opcodes/tic54x-opc.c`
3. Cross-reference with SPRU172C (TMS320C54x instruction set)
4. Fix in `calypso_c54x.c` in `c54x_exec_one()` switch
5. Build, run, check DSP IDLE + SP + IMR

## ROM Reader

```bash
bash /opt/GSM/dsp_read.sh <section> <addr_hex>
# Sections: regs, drom, pdrom, prom0, prom1, prom2, prom3
# Example: bash /opt/GSM/dsp_read.sh prom0 0x770C
```

## DSP Boot Trace Format

```
[c54x] BOOT[phase.step] PC=0xXXXX op=0xXXXX SP=0xXXXX A=... B=...
```
- phase 1 = first boot, phase 2 = second boot (after DSP_DL_STATUS_READY)
- Check SP changes to detect stack corruption
- SP should stay near 0x5AC8 during boot

## C54x Addressing Modes (resolve_smem)

- Bit 7 = 0: Direct addressing → (DP << 7) | (op & 0x7F)
- Bit 7 = 1: Indirect → modes 0x0-0xF with AR[ARP]
- Modes 0xC-0xF: lk_used = consume extra word from prog

## Critical DSP State at IDLE

Healthy boot produces:
```
IDLE @0x770C INTM=1 IMR=0xFFFF SP=0x5AC8
```
If IMR=0x0000: init code was skipped (opcode bug caused branch over init)
If SP < 0x5000: stack overflow (opcode doing spurious PUSH/CALL)

## Firmware Symbols (from nm)

| Symbol | Address | Purpose |
|--------|---------|---------|
| main | 0x820190 | ARM main loop |
| l1a_l23_handler | 0x823f9c | L1CTL message dispatch |
| l1s_pm_test | 0x825424 | Schedule PM in TDMA |
| l1s_fbsb_req | 0x826778 | Schedule FB/SB |
| l1s_fbdet_cmd | 0x8262cc | Write d_task_md=5 |
| l1a_compl_execute | 0x825180 | Main loop completions |
| sim_handler | 0x82266c | SIM (patched to BX LR) |
| tdma_sched_execute | 0x828ef8 | TDMA scheduler |
| sercomm | 0x832428 | Sercomm state |
| dsp_api | 0x82f9c4 | DSP API pointers |
| l1s | 0x836508 | L1S state |
| fbs | 0x8307ec | FBSB state |
| l23_rx_queue | 0x82f854 | L1CTL RX queue |
