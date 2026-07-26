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
