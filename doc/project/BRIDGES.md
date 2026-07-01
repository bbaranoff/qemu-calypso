# Bridges & Communication Map — Calypso QEMU

État après merge `l1ctl_sock_init` (2026-04-08).

## 1. Vue d'ensemble

```
┌──────────────┐                                              ┌──────────────┐
│  mobile L23  │                                              │ osmo-bts-trx │
└──────┬───────┘                                              └──┬─┬─┬───────┘
       │ unix /tmp/osmocom_l2_1                                  │ │ │
       │ length-prefix BE16 + L1CTL                              │ │ │
       │                                       UDP 5700 (CLK)   │ │ │
       │                                  ┌────────────────────►│ │ │
       │                                  │ bridge_clk.py       │ │ │
       │                                  └─────────────────────┘ │ │
       │                                       UDP 5701 (TRXC)    │ │
       │                                  ┌──────────────────────►│ │
       │                                  │ bridge_trxc.py        │ │
       │                                  │ (local stub)          │ │
       │                                  └──┬────────────────────┘ │
       │                                     │                       │
       │                                     │   UDP 5702 (DL)       │
       │                                  ┌──┴───────────────────────┘
       │                                  │ bridge_bsp.py
       │                                  │  ─DL→ UDP 6702 → QEMU bsp_udp_dl_cb
       │                                  │  ◄UL─ UDP 6802 ← QEMU bsp_udp_ul_send
       │                                  │       └── tap → unix client
       │                                  │              /tmp/osmocom_l2_1
       │                                  └──────────────┐
       │                                                 │
       ▼                                                 ▼
┌────────────────────────────────────────────────────────────────────────┐
│                      QEMU  qemu-system-arm -M calypso                  │
│                                                                        │
│  ┌─────────────────────────┐    ┌──────────────────────┐               │
│  │ l1ctl_sock.c            │◄──►│ calypso_uart.c       │               │
│  │ srv /tmp/osmocom_l2_1   │    │ inject_raw / TX hook │               │
│  └────────┬────────────────┘    └─────────┬────────────┘               │
│           │ sercomm DLCI 5                 │ FIFO + IRQ_UART_MODEM     │
│           ▼                                ▼                           │
│  ┌─────────────────────────────────────────────────────┐               │
│  │ ARM7 firmware (osmocom-bb layer1 / sercomm / L1S)   │               │
│  └────────┬───────────────────────────────────┬────────┘               │
│           │ MMIO API 0xFFD00000               │ frame_irq              │
│           ▼                                   │                        │
│  ┌─────────────────────────┐    ┌─────────────┴──────┐                 │
│  │ calypso_trx.c           │◄──►│ calypso_tint0.c    │                 │
│  │ • dsp_ram_write hook    │    │ 4.615 ms TINT0     │                 │
│  │ • api_write_cb mirror   │    └────────────────────┘                 │
│  │ • PM-STUB / FBSB-PUB    │                                           │
│  │ • SINT17 wake (gated)   │                                           │
│  └────────┬────────────────┘                                           │
│           │ shared dsp_ram + api_ram                                   │
│           ▼                                                            │
│  ┌─────────────────────────┐    ┌──────────────────────┐               │
│  │ calypso_c54x.c (DSP)    │◄──►│ calypso_bsp.c        │               │
│  │ TMS320C54x emulator     │    │ DL recv UDP 6702     │               │
│  │ DARAM 0x2e80 (DL win)   │    │ UL send UDP 6802     │               │
│  │ DARAM 0x0900 (UL bits)  │    │ BRINT0 wake (gated)  │               │
│  └─────────────────────────┘    └──────────────────────┘               │
└────────────────────────────────────────────────────────────────────────┘
```

## 2. Bridges Python (un process par route)

| Bridge | Bind | Send | Rôle | Payload dispatch |
|---|---|---|---|---|
| `bridge_clk.py` | — | UDP 5700 | source CLK_IND périodique | `IND CLOCK <fn>` |
| `bridge_trxc.py` | UDP 5701 | UDP 5701 | stub TRXC ASCII | POWERON / SETSLOT / SETFORMAT(0) / SETPOWER / NOMTXPOWER … |
| `bridge_bsp.py` | UDP 5702 (DL), UDP 6802 (UL) | UDP 6702 (DL→QEMU), UDP 5802 (UL→bts), unix client `/tmp/osmocom_l2_1` (UL tap) | TRXD forwarder | DL parse `(ver,tn,fn,att)`, UL parse `(tn,fn,rssi,toa)` |
| `bridge_pty.py` | unix server `/tmp/calypso_modem.sock` | (modem only) | sercomm UART debug | DLCI 1=LOADER, 2=DEBUG, 3=ECHO, 4=TRXC obs |

⚠ `bridge_pty.py` ne doit **plus** bind `/tmp/osmocom_l2_1` — `l1ctl_sock` côté QEMU en est désormais propriétaire.

## 3. QEMU side — fichiers, fonctions, adresses, IRQ

### `hw/arm/calypso/l1ctl_sock.c`
- `l1ctl_sock_init(uart, "/tmp/osmocom_l2_1")` — bind+listen+nonblock, `qemu_set_fd_handler(srv_fd, l1ctl_accept_cb)`
- `l1ctl_accept_cb` → `accept()` → `cli_fd`, `qemu_set_fd_handler(cli_fd, l1ctl_sock_read)`
- `l1ctl_sock_read` — length-prefix BE16 → wrap sercomm DLCI 5 → `calypso_uart_inject_raw(s->uart, frame, flen)`
- `l1ctl_sock_uart_tx_byte(byte)` — sercomm parser état SC_IDLE/SC_IN_FRAME/SC_ESCAPE, frame complète DLCI 5 → BE16 length + payload → `write(cli_fd)`

### `hw/char/calypso_uart.c`
- `calypso_uart_inject_raw(s, buf, len)` — push FIFO + raise `IRQ_UART_MODEM`
- DR write callback → `l1ctl_sock_uart_tx_byte(ch)` (ligne 477)

### `hw/arm/calypso/calypso_soc.c`
- ligne 234 : `l1ctl_sock_init(&s->uart_modem, getenv("L1CTL_SOCK_PATH") ?: "/tmp/osmocom_l2_1")`
- branche IRQ_UART_MODEM via `INTH_IRQ(IRQ_UART_MODEM)`

### `hw/arm/calypso/calypso_trx.c`
- `g_trx` (singleton), `calypso_trx_dsp_init_done()` getter
- `dsp_ram_write` — hook ARM writes :
  - `0x01A8/2` : mirror `d_dsp_page` vers `dsp->api_ram[0x08D4 - C54X_API_BASE]`
  - `0x0008` / `0x0030` : `d_task_md[p0/p1]` — log + PM-STUB (a_pm[0..2]=110 dans db_r p0/p1) + `IRQ_API` pulse + `calypso_fbsb_on_dsp_task_change`
- `trx_dsp_api_write_cb(woff, val)` — DSP→ARM mirror : `s->dsp_ram[woff]=val`, `qemu_irq_pulse(IRQ_API)`, hook `woff==NDB_OFF_D_FB_DET`
- `calypso_tint0_do_tick(fn)` — DMA wp[], SINT17 wake **gated `dsp_init_done && idle`** (vec 19, bit 3), DSP run budget, `qemu_irq_raise(IRQ_TPU_FRAME)`
- BRINT0 trx path (l702-703) : `if (s->dsp_init_done) c54x_interrupt_ex(s->dsp, 21, 5)`

### `hw/arm/calypso/calypso_bsp.c`
- `bsp_udp_dl_cb` — UDP recv `:6702`, parse TRXDv0 6+148, `calypso_bsp_rx_burst(tn, fn, samples, n)`
- `calypso_bsp_rx_burst` — DMA dans DARAM `bsp.daram_addr` (= 0x2e80), BRINT0 wake **gated `bsp.dsp->idle && calypso_trx_dsp_init_done()`** (vec 21, bit 5)
- `calypso_bsp_tx_burst(tn, fn)` — lit DARAM 0x0900, `bsp_udp_ul_send` → UDP `:6802`

### `hw/arm/calypso/calypso_tint0.c`
- `tint0_tick_cb` — QEMUTimer 4.615 ms, `tint0.fn = (tint0.fn+1) % GSM_HYPERFRAME`, `calypso_tint0_do_tick(fn)`, `qemu_notify_event()`
- `calypso_tint0_set_fn(fn)` / `calypso_tint0_fn()` / `tpu_en_pending`

### `hw/arm/calypso/calypso_fbsb.c`
- `calypso_fbsb_on_dsp_task_change(s, d_task_md, fn)` :
  - `DSP_TASK_FB(5)` → `publish_fb_found(toa=0, pm=80, ang=0, snr=100)` → state `FB0_FOUND`
  - `DSP_TASK_SB(6)` → `publish_sb_found(bsic=0)` → state `SB_FOUND`
- NDB cells : `NDB_D_FB_DET=0x08F8`, `NDB_D_FB_MODE=0x08F9`, `NDB_A_SYNC_DEMOD_TOA/PM/ANG/SNR=0x08FA..0x08FD`

### `hw/arm/calypso/calypso_c54x.c`
- `c54x_run(budget)` boucle fetch/execute
- `c54x_interrupt_ex(s, vec, imr_bit)` — push PC, vector ISR
- IDLE → `s->idle = true`
- 3 wakes :
  - SINT17 frame (vec 19, bit 3) — `calypso_trx.c:calypso_tint0_do_tick`
  - BRINT0 BSP rx (vec 21, bit 5) — `calypso_bsp.c:calypso_bsp_rx_burst`
  - TINT0 timer (vec 20, bit 4) — masqué

## 4. Adresses / ports / sockets

| Endroit | Type | Adresse |
|---|---|---|
| Mobile L23 ↔ QEMU L1CTL | unix server | `/tmp/osmocom_l2_1` (l1ctl_sock dans QEMU) |
| QEMU UART chardev | unix server | `/tmp/calypso_modem.sock` (bridge_pty) |
| BTS ↔ bridge CLK | UDP | bts:5800 ↔ bridge:5700 |
| BTS ↔ bridge TRXC | UDP | bts:5801 ↔ bridge:5701 |
| BTS ↔ bridge TRXD DL | UDP | bts:5802 ↔ bridge:5702 → qemu:6702 |
| BTS ↔ bridge TRXD UL | UDP | qemu:6802 → bridge → bts:5802 |
| API mailbox DSP | C54x word | base 0x0800 |
| `d_dsp_page` | NDB | DSP word 0x08D4, ARM byte 0xFFD001A8 |
| `d_task_md p0/p1` | API | ARM byte 0xFFD00008 / 0xFFD00030 |
| `d_fb_det` | NDB | DSP word 0x08F8 |
| BSP DL DMA window | DARAM | 0x2e80 (`CALYPSO_BSP_DARAM_ADDR`) |
| BSP UL bits | DARAM | 0x0900 |
| TINT0 period | timer | 4.615 ms (`TINT0_PERIOD_NS`) |

## 5. Wakes DSP — table de gating

| Wake | Vec | IMR bit | Source | Gate |
|---|---|---|---|---|
| SINT17 (frame) | 19 | 3 | `calypso_trx.c:calypso_tint0_do_tick` | `dsp_init_done && idle` |
| BRINT0 (BSP rx, trx path) | 21 | 5 | `calypso_trx.c` (legacy) | `dsp_init_done` |
| BRINT0 (BSP rx, bsp path) | 21 | 5 | `calypso_bsp.c:calypso_bsp_rx_burst` | `dsp->idle && calypso_trx_dsp_init_done()` |
| TINT0 (timer) | 20 | 4 | masqué côté DSP | — |

## 6. Run

`run.sh` lance dans tmux : `b_pty`, `b_bsp`, `b_trxc`, `b_clk`, `qemu`, `bts`, `mobile`, `calypso_trx` (orchestrator log filter), `discover` (DARAM RD HIST).
