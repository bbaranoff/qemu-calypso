# Calypso QEMU — flow overview

## Components

```mermaid
flowchart LR
    L23[mobile L23]
    subgraph QEMU["qemu-system-arm -M calypso"]
        SOCK[l1ctl_sock.c<br/>/tmp/osmocom_l2_1]
        UART[calypso_uart.c]
        ARM[ARM L1S fw]
        TRX[calypso_trx.c<br/>orchestrator]
        DSP[c54x DSP]
        BSP[calypso_bsp.c<br/>6702 / 6802]
        ORCH[calypso_orch.c<br/>UDP 6920]
        STEAL[l1ctl_steal<br/>UDP 6921]
    end
    CLK[bridge_clk.py]
    TRXC[bridge_trxc.py]
    BURSTS[bridge_orch_bursts.py<br/>5702↔6702 6802↔5802]
    OPTY[bridge_orch_pty.py<br/>L1CTL/D_TASK/FBSB]
    BTS[osmo-bts-trx]

    L23 <-->|unix BE16+L1CTL| SOCK
    SOCK <-->|sercomm DLCI5| UART
    UART <-->|FIFO+IRQ| ARM
    ARM <-->|API mailbox| TRX
    TRX <-->|dsp_ram / IRQs| DSP
    DSP <-->|DARAM DMA| BSP
    SOCK -->|on_l1ctl_fw| TRX
    BSP -->|on_ul_burst_raw| TRX
    TRX -->|publish| ORCH
    BSP -->|publish| ORCH
    SOCK -->|publish| ORCH
    ORCH -.subscribe.-> BURSTS
    ORCH -.subscribe.-> OPTY
    BURSTS -->|steal 6921| STEAL
    STEAL --> UART
    CLK -->|UDP 5700| BTS
    BTS <-->|UDP 5701| TRXC
    BTS <-->|UDP 5702/5802| BURSTS
    BURSTS <-->|UDP 6702/6802| BSP
```

## L1CTL cycle (PM → FBSB)

```mermaid
sequenceDiagram
    participant L23 as mobile L23
    participant SOCK as l1ctl_sock
    participant UART as calypso_uart
    participant ARM as ARM L1S
    participant TRX as calypso_trx
    participant DSP as c54x DSP
    participant BSP as calypso_bsp
    participant BTS as osmo-bts-trx

    L23->>SOCK: RESET_REQ
    SOCK->>UART: sercomm DLCI5
    UART->>ARM: IRQ
    ARM-->>SOCK: RESET_CONF
    SOCK->>TRX: on_l1ctl_fw(0x0e)
    SOCK-->>L23: RESET_CONF

    L23->>ARM: PM_REQ
    ARM->>DSP: d_task_md=1 (PM)
    TRX->>TRX: dsp_ram_write → publish D_TASK
    DSP-->>ARM: a_pm[]
    ARM-->>L23: PM_CONF
    SOCK->>TRX: on_l1ctl_fw(0x09) → pulse IRQ_API

    L23->>ARM: FBSB_REQ
    ARM->>DSP: d_task_md=5 (FB)
    BTS->>BSP: DL burst (5702→6702)
    BSP->>DSP: DMA + BRINT0 [gated dsp_init_done]
    DSP-->>ARM: d_fb_det=1
    ARM-->>L23: FBSB_CONF

    Note over DSP,BTS: UL burst path
    DSP->>BSP: tx_burst (DARAM 0x0900)
    BSP->>TRX: on_ul_burst_raw
    BSP->>BTS: UDP 6802→5802
```

## Wake table (DSP)

| Wake    | Vec | IMR bit | Source                               | Gate                           |
|---------|-----|---------|--------------------------------------|--------------------------------|
| SINT17  | 19  | 3       | calypso_trx.c calypso_tint0_do_tick  | `dsp_init_done && idle`        |
| BRINT0  | 21  | 5       | calypso_bsp.c calypso_bsp_rx_burst   | `dsp->idle && dsp_init_done`   |
| TINT0   | 20  | 4       | masked by firmware                   | inactive                       |

## Layers on the orch bus

| Code | Name     | Publisher                         |
|------|----------|-----------------------------------|
| 0x01 | L1CTL    | `l1ctl_sock.c` after send_to_mobile |
| 0x02 | UL_BURST | `calypso_bsp.c` bsp_udp_ul_send   |
| 0x03 | DL_BURST | `calypso_bsp.c` bsp_udp_dl_cb     |
| 0x04 | DSP_API  | `calypso_trx.c` api_write_cb      |
| 0x05 | D_TASK   | `calypso_trx.c` dsp_ram_write     |
| 0x06 | FBSB     | `calypso_fbsb.c` publish_fb/sb    |
| 0x07 | TRXC     | reserved (bridge_trxc future)     |
