# Calypso QEMU — Full call flow (2026-04-08)

> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](../DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> Concrètement pour la section 3 (FBSB) : la synthèse FB côté hôte a été retirée le 2026-05-28 (`calypso_fbsb.c:3-11`). Le callback défensif `dsp_api_write_cb`/`trx_dsp_api_write_cb` (étapes 6-7, diagramme §7) **n'existe pas** dans le code ; le hook générique `api_write_cb` est déclaré (`calypso_c54x.h:204`, invoqué `calypso_c54x.c:3357-3358`) mais **jamais assigné** (`grep 'api_write_cb =' → 0`). La publication FB (`calypso_fbsb_publish_fb_found`) n'est ni immédiate ni inconditionnelle : elle n'a lieu que si `calypso_orch() && fbsb_synth_enabled()` ET que le corrélateur BSP renvoie une détection réelle (`calypso_fbsb.c:113-119`). Les constantes `pm=80/snr=100` et le PC `0xe26e` n'apparaissent nulle part.

End-to-end ordering across all layers, top→bottom, for the mobile↔BTS path.
Use this as a single source of truth when something hangs and you need to
locate which layer dropped the ball.

## 0. Process layout

```
mobile (osmocom-bb)                — userland L23
    │  AF_UNIX  /tmp/osmocom_l2_1  ← length-prefixed L1CTL frames
    ▼
calypso-ipc-device (run_all server)         — Python broker / decoder
    │  AF_UNIX  /tmp/calypso_modem.sock  (UART modem)
    │  UDP      127.0.0.1:5700/5701/5702 (BTS-side TRX)
    │  UDP      127.0.0.1:6700/6702/6802 (BB-side TRX)
    ▼
qemu-system-arm -M calypso         — ARM7TDMI + TMS320C54x DSP
    │  CalypsoSoC: uart_modem, uart_irda, timers, INTH, dsp_api MMIO,
    │              calypso_trx, calypso_bsp, calypso_fbsb, calypso_tint0
    ▼
firmware (osmocom layer1.highram)  — runs unmodified inside ARM
    │  sercomm DLCI 5 (L1CTL), DLCI 4 (TRXC mirror), DLCI 1/2/3 (loader/dbg)
    ▼
DSP (TMS320C54x emulator)          — PROM0+PROM1, NDB mailbox at 0x0800
```

## 1. Boot

| step | actor                | action                                                        |
|------|----------------------|---------------------------------------------------------------|
| 1    | run.sh               | kill_all → start tmux session `calypso`                       |
| 2    | calypso-ipc-device            | bind UDP 5700/5701/5702 + 6802 ; AF_UNIX listen on `/tmp/calypso_modem.sock` and `/tmp/osmocom_l2_1` |
| 3    | qemu-system-arm      | client connect to `/tmp/calypso_modem.sock` (`reconnect=1`)   |
| 4    | calypso_soc.c        | realize uart_modem (label `modem`), set chardev handler `calypso_uart_receive` |
| 5    | calypso_trx.c        | `calypso_trx_init` → `c54x_init` → `c54x_load_rom` → `calypso_bsp_init(dsp)` → `calypso_iota_init` |
| 6    | calypso_bsp.c        | bind UDP 6702, register `bsp_udp_dl_cb` via `qemu_set_fd_handler` |
| 7    | firmware             | reset, sercomm boot, L1S thread starts                        |
| 8    | firmware → bridge    | DLCI 5 `L1CTL_RESET_IND` → `[pty-rx] fw→mobile L1CTL RESET_IND` |
| 9    | bridge               | sets `firmware_ready=True`, drains buffered mobile msgs       |
| 10   | osmo-bts-trx         | TRXC handshake POWERON/SETFORMAT/SETSLOT/… on UDP 5701        |
| 11   | bridge               | local stub (`trxc_response()`) replies on the same socket; CMDs are mirrored to UART DLCI 4 for observation |
| 12   | bridge clk thread    | every `CLK_IND_PERIOD=216` frames send `IND CLOCK fn-3` on UDP 5700 |

## 2. Mobile attach (after `start` via VTY 4247)

| step | actor              | action                                                         |
|------|--------------------|----------------------------------------------------------------|
| 1    | mobile             | open `/tmp/osmocom_l2_1` AF_UNIX                               |
| 2    | bridge             | accept → `L1CTL client connected`                              |
| 3    | mobile → bridge    | `L1CTL_RESET_REQ` (length-prefixed)                            |
| 4    | bridge → PTY       | `sercomm_wrap(DLCI=5, msg)` → `os.write(pty_fd, …)`            |
| 5    | calypso_uart_receive | parse → `sercomm_gate_feed`                                  |
| 6    | sercomm_gate_feed  | DLCI 5 → `gate_push_to_fifo` → re-inject raw frame into UART RX FIFO |
| 7    | firmware sercomm   | reads RBR via MMIO, parses HDLC, dispatches DLCI 5 → `l1a_l23` |
| 8    | firmware L1A       | RESET_REQ handler → emits `L1CTL_RESET_CONF`                    |
| 9    | firmware → bridge  | sercomm wrap DLCI 5 → THR → modem chardev → `pty-rx` → mobile  |
| 10   | mobile → bridge    | `L1CTL_PM_REQ` (cell scan)                                     |
| 11   | bridge → fw        | DLCI 5 forward → firmware L1S → `L1CTL_PM_CONF`                |
| 12   | mobile → bridge    | `L1CTL_FBSB_REQ` (band_arfcn, sync_info_idx, …)                |

## 3. FBSB — the load-bearing path

| step | actor                | action                                                        |
|------|----------------------|---------------------------------------------------------------|
| 1    | bridge → fw          | DLCI 5 forward of `L1CTL_FBSB_REQ`                            |
| 2    | firmware L1S         | enqueues FB-det task, writes `d_task_md = DSP_TASK_FB` (1) into NDB at `0xFFD00008` (page 0) or `0xFFD00030` (page 1) |
| 3    | calypso_trx.c::dsp_ram_write | sees `offset==0x08\|0x30` and `value!=0` → lazy-init `g_fbsb` and call `calypso_fbsb_on_dsp_task_change(&g_fbsb, value, fn)` |
| 4    | calypso_fbsb.c       | ~~enters `FBSB_FB0_SEARCH` → immediately calls `calypso_fbsb_publish_fb_found(toa=0, pm=80, ang=0, snr=100)` → writes `d_fb_det=1`~~ — **FAUX** : le passage à `FBSB_FB0_SEARCH` ne fait que logger (`calypso_fbsb.c:107-121`). La publication n'a lieu QUE si `calypso_orch() && fbsb_synth_enabled()` (env `CALYPSO_SYNTH_FBSB`/`CALYPSO_DSP_L1_STUB`) ET que `calypso_bsp_get_fb_detection()` renvoie une vraie détection ; les valeurs viennent du corrélateur BSP, **pas** de constantes (`pm=80/snr=100` absents du code). En pratique **d_fb_det reste 0**. |
| 5    | calypso_fbsb.c       | ~~transitions to `FBSB_FB0_FOUND` and dumps state~~ — n'a lieu que si l'étape 4 a réellement publié (jamais sur le run réel) |
| 6    | DSP                  | (DSP déraille : POST-BOOTSTUB-RET, PC=0x0000 ; aucune publication hôte ne l'écrase — la synthèse hôte a été retirée le 2026-05-28) |
| 7    | ~~calypso_trx.c::dsp_api_write_cb~~ | **FAUX** : aucun `dsp_api_write_cb`/`trx_dsp_api_write_cb` n'existe (`grep → 0`), le PC `0xe26e` n'apparaît nulle part. Le hook générique `api_write_cb` (`calypso_c54x.h:204`, invoqué `calypso_c54x.c:3357-3358`) n'est **jamais assigné**. |
| 8    | firmware L1S         | next frame poll: reads `d_fb_det == 1` and clean a_sync_demod → builds `L1CTL_FBSB_CONF` with `result=0, bsic=0` |
| 9    | firmware → bridge    | DLCI 5 → `L1CTL_FBSB_CONF`                                    |
| 10   | bridge → mobile      | length-prefix forward → mobile receives confirm               |
| 11   | mobile               | enters "cell synced" state, awaits `L1CTL_DATA_IND`           |

## 4. DL — SI broadcast (BCCH/CCCH)

| step | actor                | action                                                        |
|------|----------------------|---------------------------------------------------------------|
| 1    | osmo-bts-trx         | every TS0 BCCH/CCCH frame sends DL TRXDv0 burst on UDP 5702   |
| 2    | bridge.bts_trxd_cb   | parse 6-byte hdr (`(ver<<4)|tn`, `fn` BE, `att`) + 148 hard ubits |
| 3    | bridge               | filter `tn==0`, compute `fn51=fn%51`, classify into CCCH block window (start ∈ {2,6,12,16}, idx=fn51-start) |
| 4    | bridge               | accumulate `burst_buf[(fn//51, start)] = [114-bit chunks]`    |
| 5    | bridge               | when 4 bursts collected: `gsm0503_xcch_decode(iB)` → 23-byte L2 |
| 6    | bridge → mobile      | build `L1CTL_DATA_IND` = `hdr(4) + info_dl(12) + l2(23)` → cli.sendall length-prefixed |
| 7    | mobile               | parses SI1/SI2/SI3/SI4 → "cell found / cell info OK"          |

The DSP/BSP path is **bypassed** for DL CCCH — we don't try to make the
DSP correlate real samples. ~~Only `calypso_fbsb` synthesizes FBSB into NDB.~~ — **FAUX** : la synthèse FBSB côté hôte a été retirée le 2026-05-28 (`calypso_fbsb.c:3-11`) ; `calypso_fbsb.c` ne fait plus que logger les changements de tâche DSP. Sur le run réel, personne n'écrit `d_fb_det` et il **reste 0**.

## 5. UL — burst from DSP/L1 to BTS

| step | actor                | action                                                        |
|------|----------------------|---------------------------------------------------------------|
| 1    | firmware             | when L1S tx slot is due, writes 148 bits into DSP UL DARAM `0x0900..0x0993` |
| 2    | calypso_trx.c        | TINT0 frame tick → `calypso_trx_send_ul_burst` → `calypso_bsp_tx_burst(tn, fn, bits)` |
| 3    | calypso_bsp.c        | reads `dsp->data[0x0900+i]` for i=0..147 (LSB), packs into `bits[148]` |
| 4    | calypso_bsp.c        | always-ship: `bsp_udp_ul_send` builds 156-byte packet: `tn(1) + fn(4 BE) + rssi=60(1) + toa=0(2) + sbits×127(148)` → sendto 127.0.0.1:6802 |
| 5    | bridge.bb_trxd_cb    | recv → `[burst UL qemu→bts]` → forward to `bts_trxd_dst = (127.0.0.1, 5802)` |
| 6    | osmo-bts-trx         | demods sbits, decodes uplink (RACH/SDCCH/TCH)                 |

## 6. TRXC — control plane

| step | actor                | action                                                        |
|------|----------------------|---------------------------------------------------------------|
| 1    | osmo-bts-trx         | sends `CMD POWERON/SETFORMAT/SETSLOT/…\0` from local 5801 to bridge 5701 |
| 2    | bridge.bts_trxc_cb   | `trxc_response(pkt)` builds RSP (`POWERON 0`, `SETFORMAT 0 0` to force TRXDv0, etc.) |
| 3    | bridge               | sendto same addr (5801) → BTS sees the RSP, completes handshake |
| 4    | bridge               | mirror raw CMD to UART DLCI 4 (observation; firmware ignores) |

The CMD never round-trips through QEMU — bridge is authoritative.

## 7. Layer wiring summary

```
mobile          ──┐
                  │ unix /tmp/osmocom_l2_1   (L1CTL length-prefixed)
                  ▼
              calypso-ipc-device
              ├── server unix /tmp/calypso_modem.sock     ──► QEMU UART modem (sercomm DLCI 5/4)
              ├── server unix /tmp/osmocom_l2_1            ──► mobile
              ├── bind  UDP 5700  IND CLOCK out           ──► osmo-bts-trx 5800
              ├── bind  UDP 5701  TRXC stub               ◄─► osmo-bts-trx 5801
              ├── bind  UDP 5702  TRXD DL recv            ◄── osmo-bts-trx 5802
              └── bind  UDP 6802  TRXD UL recv from QEMU  ◄── BSP
                                                          ──► osmo-bts-trx 5802

QEMU
├── calypso_uart_receive (modem chardev) ──► sercomm_gate_feed
│       ├── DLCI 5 → gate_push_to_fifo → UART RX FIFO → firmware sercomm
│       ├── DLCI 4 → gate_trxc_handle (observation, currently inert)
│       └── other DLCIs → FIFO
├── calypso_bsp.c
│       ├── bsp_udp_dl_cb  (bind 6702)  ◄── bridge UL forward (currently NOT used for DL)
│       └── calypso_bsp_tx_burst        ──► bsp_udp_ul_send → 127.0.0.1:6802
├── calypso_trx.c
│       ├── dsp_ram_write    (intercept ARM writes to NDB mailbox)
│       │     └─ d_task_md   ──► calypso_fbsb_on_dsp_task_change (log-only)
│       └── (INEXISTANT) trx_dsp_api_write_cb  — FAUX: aucun tel callback dans le code.
│             Le hook générique api_write_cb (calypso_c54x.h:204, appel .c:3357-3358)
│             n'est JAMAIS assigné → aucune interception des écritures DSP.
├── calypso_fbsb.c   (log-only depuis 2026-05-28 ; ne synthétise plus rien dans la NDB)
└── c54x DSP emulator
        └── runs PROM0/PROM1 puis DÉRAILLE (POST-BOOTSTUB-RET, PC=0x0000) ; d_fb_det reste 0
```
