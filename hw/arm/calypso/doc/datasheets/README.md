# Calypso DSP / TMS320C54x Reference Datasheets

Collected 2026-04-28 to provide ground truth for the QEMU C54x emulator and to
resolve open questions about Calypso DBB boot semantics.

## Files

| File | Source | Size | Purpose |
|---|---|---|---|
| `TI_SPRU131G_C54x_CPU_and_Peripherals.pdf` | ti.com/lit/ug/spru131g | 2.2 MB | TMS320C54x DSP Reference Set Vol.1: CPU. **The C54x reference**. ST0/ST1 layout, reset values, IDLE modes, IMR/IFR semantics, MMR layout, interrupt handling. |
| `TI_SPRU172C_C54x_Mnemonic_Instruction_Set.pdf` | local copy | 1.1 MB | Vol.2: Mnemonic Instruction Set. Per-opcode reference (already cross-referenced for opcode bug fixes). |
| `TI_SPRU288_C548_C549_Bootloader.pdf` | ti.com/lit/ug/spru288 | 313 KB | **C548/C549 Bootloader and ROM Code Contents**. Describes the on-chip ROM at 0xF800-0xFFFF (only mapped when MP/MC=0). Calypso has MP/MC=1 → this ROM is NOT used. |
| `TI_SPRA036_DSP_Interrupts.pdf` | ti.com/lit/an/spra036 | 173 KB | Setting Up TMS320 DSP Interrupts in C — vector handler patterns, ISR entry/exit, RSBX/SSBX INTM usage. |
| `TI_SPRA618_C5402_Bootloader.pdf` | ti.com/lit/an/spra618 | 309 KB | C5402 Bootloader (related variant). For comparison. |
| `dsp-rom-3606-dump.txt` | freecalypso.org/pub/GSM/Calypso | 722 KB | Calypso DSP ROM v3606 (Pirelli DP-L10). Most common version — matches our local `calypso_dsp.txt` (14 diff lines / 9821 = essentially identical). |
| `dsp-rom-3416-dump.txt` | freecalypso.org/pub/GSM/Calypso | 722 KB | Calypso DSP ROM v3416 (FCDEV3B-751774). |
| `dsp-rom-3311-dump.txt` | freecalypso.org/pub/GSM/Calypso | 722 KB | Calypso DSP ROM v3311 (D-Sample C05). |
| `DSP-ROM-dump.txt` | freecalypso-tools/doc | 1.8 KB | FreeCalypso documentation about the ROM dumping methodology. |

---

## Key findings (collected here, applied/cross-referenced in code)

### 1. C548 On-Chip ROM Layout (SPRU288 §1.2, Figure 1-1)

```
When MP/MC=0 (microcomputer mode), program space looks like:

  0x0000-0xF7FF : External program space        ← user code lives here
  0xF800-0xFBFF : Bootloader (TI mask ROM)
  0xFC00-0xFCFF : µ-law table
  0xFD00-0xFDFF : A-law table
  0xFE00-0xFEFF : Sine look-up table
  0xFF00-0xFF7F : Built-in self-test
  0xFF80-0xFFFF : Interrupt vector table

When MP/MC=1 (microprocessor mode):
  0x0000-0xFFFF : All external — TI internal ROM is NOT mapped.
```

### 2. Calypso uses MP/MC=1

Calypso reset value `PMST = 0xFFA8` (per silicon dumps below) includes bit 6
(MP_MC) = 1 → **microprocessor mode**. The TI on-chip ROM at 0xF800-0xFFFF is
**not used**. The osmocom firmware fully provides PROM1 mirror including vector
table at 0xFF80-0xFFFF.

This is consistent with osmocom-bb `dsp_bootcode.c`:
```c
/* We don't really need any DSP boot code, it happily works with its own ROM */
static const struct dsp_section *dsp_bootcode = NULL;
```

The "own ROM" referred to is the **Calypso-specific mask ROM cast at the silicon
level**, which contains the GSM signal-processing routines. This ROM is what the
osmocom community has dumped (3 versions: 3311, 3416, 3606) and is exactly what
the QEMU emulator loads at PROM0/1/2/3.

### 3. Reset / post-bootloader state — empirical (4 dumps cross-checked)

The Registers section [0x00000-0x0005F] in each ROM dump captures the DSP MMR
state **post-bootloader-handshake but pre-application-init** (the dumper asserts
DSP into reset, releases, reads bootloader version, then dumps registers while
the DSP is in its idle loop waiting for BL_CMD instructions from ARM).

Cross-checked across 3 silicon phones (3311, 3416, 3606) + our local
`calypso_dsp.txt`. Invariants across all 4:

| MMR | Addr | Value | Notes |
|---|---|---|---|
| **ST0** | 0x06 | `0x181F` | DP=0x01F (low 9 bits), other bits 0 |
| **ST1** | 0x07 | `0x2900` | bit 8 SXM=1, bit 11 INTM=1, bit 13 XF=1 |
| AR1 | 0x11 | `0x005F` | preserved bootloader pointer |
| AR2 | 0x12 | `0x0813` | API_RAM-related |
| AR3 | 0x13 | `0x0014` | preserved |
| AR4 | 0x14 | `0x0003` | preserved |
| AR5 | 0x15 | `0x0014` | preserved |
| **SP** | 0x18 | `0x1100` | **bootloader stack base — NOT 0x5AC8** |
| **PMST** | 0x1D | `0xFFA8` | IPTR=0x1FF, MP_MC=1, OVLY=1, DROM=1 |
| (ext) | 0x22 | `0x0800` | API_RAM base reference |
| (ext) | 0x25 | `0xFFFF` | invariant |
| (ext) | 0x28 | `0x7FFF` | invariant |
| (ext) | 0x29 | `0xF802` | invariant |

Variable across versions (= programmed by ROM-version-specific bootloader):

| MMR | 3311 | 3416 | 3606 | osmocom local |
|---|---|---|---|---|
| IMR | 0xF6FF | 0xF6F6 | 0x3000 | 0x52FD |
| AR0 | 0x3375 | 0xFEBF | 0x42A4 | 0xFF75 |
| BK | 0xFFFE | 0x36FC | 0x000E | 0xFFF6 |

### 4. Implications for QEMU c54x_reset()

**Current QEMU values (incorrect per silicon):**
```c
s->sp = 0x5AC8;       // wrong: silicon shows 0x1100
s->st0 = 0;           // wrong: silicon shows 0x181F (DP=0x01F)
s->st1 = ST1_INTM;    // partially correct (INTM=1), missing SXM and XF
                      //   silicon shows 0x2900 = INTM | SXM | XF
s->pmst = 0xFFE0;     // wrong: silicon shows 0xFFA8 (OVLY=1, DROM=1)
```

**Per-silicon aligned values (justified empirically by 3-dump consensus):**
```c
s->sp = 0x1100;                              // bootloader stack base
s->st0 = 0x181F;                             // DP=0x01F, no flag bits
s->st1 = ST1_INTM | ST1_SXM | ST1_XF;        // 0x2900: INTM=1, SXM=1, XF=1
s->pmst = 0xFFA8;                            // IPTR=0x1FF, MP_MC=1, OVLY=1, DROM=1
```

**Caveat on SP=0x1100:** The osmocom firmware likely repoints SP to 0x5AC8 early
in its init (post-handshake user code). The `0x5AC8` value in `c54x_reset` may
be a shortcut anticipating the firmware's own SP setup. To be safe and faithful,
align SP with silicon (0x1100) and let the firmware repoint as it does on real
hardware.

**Caveat on OVLY=1 in PMST:** With OVLY=1 the DARAM is mapped into program space
0x0080-0x27FF, so `prog_read(addr)` for `addr in [0x0080, 0x27FF]` returns
`s->data[addr]`. The MVPD copy in `c54x_reset` lines 4595-4612 already populates
this overlay. With OVLY=0 (current QEMU reset), the overlay is NOT active during
the very first instructions, so the DSP fetches from `s->prog[]` (which contains
PROM dump in 0x7000+ but is empty in 0x0080-0x6FFF). If the DSP is supposed to
execute overlay code from PC<0x2800 from the start, OVLY=1 at reset is required.

### 5. ST1.INTM at reset — confirmation

Per SPRU131G §4 Status Registers, on C54x reset:
- INTM = 1 (interrupts globally disabled)
- SXM = 1 (sign extension mode)
- All other bits cleared

The 3 FreeCalypso ROM dumps + our local `calypso_dsp.txt` all snapshot
`ST1 = 0x2900` post-bootloader-handshake. **INTM=1 stays after the Calypso
silicon hardware reset.** The osmocom firmware is responsible for clearing INTM
when ready (via RSBX INTM or by RETE from the first ISR).

The earlier hypothesis "Calypso clears INTM on reset" is **infirmed** by these
3 independent silicon dumps. The `s->st1 = 0` test in QEMU was therefore a hack
that should be reverted — not because it didn't appear to help, but because it's
not a faithful emulation of silicon behavior.

### 6. What does NOT exist (ruled out)

- **Boot ROM TI utilities at 0x0000-0x007F**: This zone is "External program
  space" per Figure 1-1. On Calypso (MP/MC=1) it's RAM/external. Our QEMU stub
  of FRET in this zone is benign (these addresses are not "TI utility routines"
  the firmware would normally call).

- **NMI initial trigger**: Not documented as automatic on Calypso reset. The
  first NMI would have to come from ARM-side via a specific MMIO write
  (likely `REG_API_CONTROL` bit 2 = APIC_W_DSPINT, but not yet verified).

### 7. Open question

If INTM=1 stays at reset and there is no TI boot ROM doing RSBX INTM, **how does
the Calypso DSP normally get INTM cleared on silicon?** Three remaining
candidates:

1. **The osmocom firmware does RSBX INTM** as part of its own init sequence at
   one of the 14 known F6BB sites in PROM0/PROM1. In QEMU, none of these sites
   is currently reached on the boot path — possibly due to an opcode mis-decode
   that drifts the PC away from the init path.

2. **ARM-side write to APIC_W_DSPINT** triggers an interrupt the DSP services
   regardless of INTM (NMI-equivalent on Calypso). Test: hook write to
   `REG_API_CONTROL = 0xFFFE0000` bit 2 in calypso_trx.c → call into the DSP
   with vec=1 (NMI) which bypasses IMR. This requires extending
   `c54x_interrupt_ex` to accept `imr_bit < 0` as "non-maskable".

3. **PMST/ST0/ST1 misalignment** at reset (see §4 above) causes the DSP to
   fetch from wrong memory regions for the very first instructions, drifting
   the PC and missing the init path. Aligning PMST=0xFFA8, ST0=0x181F,
   ST1=0x2900 per silicon dumps may resolve this.

Empirical priority: try (3) first (cheap, documented, justified by silicon
dumps), then (2) if (3) fails, then (1) by tracing what the firmware actually
attempts on the new aligned boot path.

---

## Sources

- Texas Instruments Literature: https://www.ti.com/lit/
- FreeCalypso ROM dumps: https://www.freecalypso.org/pub/GSM/Calypso/
- FreeCalypso tools docs: https://www.freecalypso.org/hg/freecalypso-tools/
- OsmocomBB Hardware wiki (Anubis-blocked): https://osmocom.org/projects/baseband/wiki/HardwareCalypsoDSP
