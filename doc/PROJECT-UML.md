# QEMU Calypso — Project UML

All diagrams use Mermaid syntax.

---

## 1. System Context

```mermaid
C4Context
    title System Context — QEMU Calypso GSM Emulation

    Person(user, "User/Researcher")
    System(mobile, "mobile (layer23)", "OsmocomBB GSM stack<br>Cell selection, MM, RR")
    System(bridge, "l1ctl_bridge.py", "Sercomm relay<br>PM intercept")
    System(qemu, "QEMU Calypso", "ARM946 emulation<br>OsmocomBB L1 firmware")
    System_Ext(bts, "BTS / SDR", "osmo-bts-trx<br>or cfile capture")

    Rel(user, mobile, "telnet 4247")
    Rel(mobile, bridge, "L1CTL", "/tmp/osmocom_l2")
    Rel(bridge, qemu, "sercomm", "PTY /dev/pts/X")
    Rel(qemu, bts, "TRXD/TRXC", "UDP 6700/6701")
```

---

## 2. Component Diagram

```mermaid
graph TB
    subgraph "Host Machine"
        subgraph "OsmocomBB"
            mobile["mobile<br>(layer23)"]
        end

        subgraph "Bridge"
            bridge["l1ctl_bridge.py"]
            pm_intercept["PM Intercept<br>ARFCN 1022 = -62 dBm"]
        end

        subgraph "QEMU Process"
            subgraph "ARM946 CPU"
                firmware["OsmocomBB L1<br>layer1.highram.elf"]
            end

            subgraph "Calypso SoC (calypso_soc.c)"
                mb["Machine Board<br>calypso_mb.c"]
                iram["IRAM 256K<br>0x00800000"]
                xram["XRAM 8M<br>0x01000000"]
                flash["Flash 4M<br>pflash_cfi01"]
                bootrom["Boot ROM 64B<br>exception vectors"]
            end

            subgraph "Peripherals"
                uart_m["UART Modem<br>calypso_uart.c<br>0xFFFF5800"]
                uart_i["UART IrDA<br>calypso_uart.c<br>0xFFFF5000"]
                inth["INTH<br>calypso_inth.c<br>0xFFFFFA00"]
                timer1["Timer 1<br>calypso_timer.c"]
                timer2["Timer 2<br>calypso_timer.c"]
                spi["SPI+ABB<br>calypso_spi.c"]
                i2c["I2C<br>calypso_i2c.c"]
            end

            subgraph "TRX Bridge (calypso_trx.c)"
                dsp["DSP API RAM<br>0xFFD00000"]
                tpu["TPU<br>0xFFFF1000"]
                tsp["TSP"]
                ulpd["ULPD"]
                sim["SIM ctrl"]
                tdma["TDMA Timer<br>46.15ms"]
                sync["Sync FSM<br>FB/SB detect"]
                fw_patch["FW Patches<br>talloc/cons_puts"]
            end
        end

        subgraph "Radio (future)"
            bts["osmo-bts-trx<br>or SDR"]
        end
    end

    mobile <-->|"L1CTL<br>unix socket"| bridge
    bridge -->|"PM_REQ"| pm_intercept
    pm_intercept -->|"PM_CONF"| mobile
    bridge <-->|"sercomm<br>PTY"| uart_m

    firmware --> dsp
    firmware --> tpu
    firmware --> uart_m
    firmware --> inth
    firmware --> spi
    firmware --> sim

    inth -->|IRQ4| tpu
    inth -->|IRQ7| uart_m
    inth -->|IRQ15| dsp
    inth -->|IRQ/FIQ| firmware

    tdma -->|"TPU_FRAME"| inth
    tpu -->|"API IRQ"| inth

    dsp <-->|"TRXD v0<br>UDP 6701"| bts
    tpu <-->|"TRXC ASCII<br>UDP 6700"| bts

    fw_patch -.->|"patch at boot"| firmware
```

---

## 3. L1CTL Message Sequence

```mermaid
sequenceDiagram
    participant M as mobile (layer23)
    participant B as l1ctl_bridge.py
    participant F as Firmware (QEMU)

    Note over F: Boot: DSP init, SIM ATR,<br>TDMA timer start

    M->>B: RESET_REQ (0x0D)
    B->>F: sercomm DLCI 5
    F->>B: RESET_CONF (0x0E)
    B->>M: RESET_CONF

    M->>B: PM_REQ E-GSM (940-1023)
    Note over B: INTERCEPT<br>ARFCN 1022 = rxlev 48<br>others = 0
    B->>M: PM_CONF (84 entries, 1 strong)

    M->>B: PM_REQ GSM900 (0-124)
    Note over B: INTERCEPT<br>all noise
    B->>M: PM_CONF (125 entries, 0 strong)

    M->>B: PM_REQ DCS1800 (512-885)
    Note over B: INTERCEPT<br>all noise
    B->>M: PM_CONF (374 entries, 0 strong)

    Note over M: Cell selection:<br>ARFCN 1022 wins (-62 dBm)

    M->>B: FBSB_REQ (0x03) ARFCN 1022
    B->>F: sercomm DLCI 5
    Note over F: L1S schedules FB task<br>TPU scenario on ARFCN 1022<br>Searches for FCCH/SCH
    F->>B: FBSB_CONF (0x04) result=255
    B->>M: FBSB_CONF result=255

    Note over M: No cell found<br>(no BTS connected)
    Note over M: Retry → rescan PM
```

---

## 4. Firmware Boot Sequence

```mermaid
sequenceDiagram
    participant CPU as ARM946
    participant MB as calypso_mb.c
    participant SOC as calypso_soc.c
    participant TRX as calypso_trx.c
    participant UART as calypso_uart.c
    participant FW as Firmware ELF

    Note over MB: Machine init

    MB->>CPU: Create ARM946
    MB->>SOC: Realize SoC
    SOC->>UART: Create UART modem + IrDA
    SOC->>TRX: calypso_trx_init()
    TRX->>TRX: Bind UDP 6700/6701
    MB->>MB: Create Flash, XRAM, Boot ROM
    MB->>FW: load_elf(layer1.highram.elf)
    MB->>CPU: set PC = 0x820000

    Note over CPU: CPU starts executing

    CPU->>UART: MDR1 write (serial init)
    UART->>TRX: calypso_fw_patch_apply()
    Note over TRX: Patch cons_puts → bx lr<br>Patch puts → bx lr<br>NOP bl printf (frame_irq)<br>Pool 32→148 slots<br>Talloc panic → retry<br>Abort → IRQs enabled

    CPU->>TRX: DSP status read (polling)
    TRX->>CPU: status=1 (BOOT) after 3 reads
    CPU->>TRX: DSP status write 0x0002 (READY)
    TRX->>TRX: Plant API version 0x3606

    CPU->>TRX: SIM MASKIT write
    TRX->>CPU: SIM ATR (rxDoneFlag=1)

    CPU->>TRX: TPU INT_CTRL write (enable frame IRQ)
    TRX->>TRX: calypso_tdma_start()
    Note over TRX: TDMA timer: 46.15ms per frame

    loop Every TDMA frame
        TRX->>CPU: TPU_FRAME IRQ (via INTH)
        CPU->>TRX: TPU_EN (execute DSP task)
        TRX->>CPU: API IRQ (task done)
    end
```

---

## 5. DSP Task Processing

```mermaid
flowchart TD
    A[TDMA tick] -->|"raise TPU_FRAME IRQ"| B[Firmware frame_irq]
    B --> C{Read previous results}
    C --> D[Write new tasks to W_PAGE]
    D --> E[Set TPU_CTRL_EN]
    E --> F[calypso_dsp_process]

    F --> G{task_d type?}
    G -->|FB| H[sync_inject_fb_result<br>SYNC_FCCH_SEARCH → FOUND]
    G -->|SB| I[sync_inject_sb_result<br>SCH encode BSIC+FN]
    G -->|NB| J[Copy rx_burst to R_PAGE]
    G -->|RACH| K[Send UL burst via TRXD]
    G -->|none| L[skip]

    F --> M{task_md != 0?}
    M -->|yes| N[Inject PM result<br>dsp_ram 52/72 = 4864<br>≈ -62 dBm]
    M -->|no| O[skip]

    F --> P[Clear tasks, update FN]
    P --> Q[Schedule API IRQ]
    Q --> R[Firmware reads results]
```

---

## 6. INTH Interrupt Flow

```mermaid
flowchart LR
    subgraph Sources
        TPU_FRAME["TPU_FRAME<br>IRQ 4"]
        UART["UART_MODEM<br>IRQ 7"]
        API["DSP API<br>IRQ 15"]
        WDT["Watchdog<br>IRQ 0"]
        SIM_IRQ["SIM<br>IRQ 6"]
    end

    subgraph INTH["INTH (calypso_inth.c)"]
        levels["levels register<br>32-bit bitmask"]
        mask["mask register"]
        ilr["ILR[0..31]<br>priority + FIQ"]
        arb["Priority arbiter<br>lowest prio# wins"]
        irq_num["IRQ_NUM register<br>read-to-ack"]
    end

    subgraph CPU
        irq_entry["irq_entry<br>0x821f9c"]
        handler["Dispatch to<br>handler[IRQ_NUM]"]
    end

    TPU_FRAME -->|level| levels
    UART -->|level| levels
    API -->|pulse| levels
    WDT -->|level| levels
    SIM_IRQ -->|level| levels

    levels --> arb
    mask --> arb
    ilr --> arb
    arb --> irq_num
    arb -->|"raise/lower"| irq_entry

    irq_entry -->|"read IRQ_NUM"| irq_num
    irq_num -->|"ack edge-like<br>(IRQ 4,5)"| levels
    irq_entry --> handler
```

---

## 7. UART TX Burst Drain

```mermaid
sequenceDiagram
    participant FW as Firmware ISR
    participant UART as calypso_uart.c
    participant INTH as INTH

    Note over FW: UART_MODEM IRQ delivered

    FW->>UART: Read IIR
    Note over UART: IIR = TX_EMPTY<br>tx_empty_reads = 1<br>pending stays true!
    UART-->>FW: 0x02 (TX_EMPTY)

    FW->>UART: Write THR (byte 1)
    Note over UART: tx_empty_reads = 0<br>pending = true
    UART-->>INTH: IRQ stays high

    FW->>UART: Read IIR
    Note over UART: IIR = TX_EMPTY<br>tx_empty_reads = 1<br>pending stays true!
    UART-->>FW: 0x02 (TX_EMPTY)

    FW->>UART: Write THR (byte 2)
    Note over UART: tx_empty_reads = 0

    Note over FW: ... loop drains multiple bytes ...

    FW->>UART: Read IIR
    Note over UART: tx_empty_reads = 1
    UART-->>FW: 0x02 (TX_EMPTY)

    Note over FW: No more data to send

    FW->>UART: Read IIR (no THR write)
    Note over UART: tx_empty_reads = 2<br>pending = false<br>IRQ lowered
    UART-->>FW: 0x01 (NO_INT)
    UART-->>INTH: IRQ low
```

---

## 8. Sync State Machine

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> FCCH_SEARCH: FB task detected<br>(d_task_d low nibble = 4/12)
    FCCH_SEARCH --> FCCH_FOUND: countdown = 0<br>inject FB result<br>(d_fb_det=1, TOA, PM, SNR)
    FCCH_FOUND --> SCH_SEARCH: SB task detected<br>(d_task_d low nibble = 5/13)
    SCH_SEARCH --> LOCKED: countdown = 0<br>inject SCH result<br>(BSIC, T1, T2, T3)

    LOCKED --> LOCKED: Update NDB FN<br>Process NB bursts

    note right of IDLE: No sync in progress
    note right of FCCH_SEARCH: Searching FCCH<br>5 frame delay
    note right of FCCH_FOUND: FCCH detected<br>waiting for SB task
    note right of SCH_SEARCH: Decoding SCH<br>2 frame delay
    note right of LOCKED: TDMA synchronized<br>ARFCN 1022, BSIC 0x3C
```

---

## 9. PM Bridge Intercept

```mermaid
flowchart TD
    A[mobile sends PM_REQ] --> B{msg_type == 0x08?}
    B -->|no| C[Forward to firmware]
    B -->|yes| D[Parse ARFCN range]
    D --> E{ARFCN == 1022?}
    E -->|yes| F["PM entry: rxlev=48 (-62 dBm)"]
    E -->|no| G["PM entry: rxlev=0 (noise)"]
    F --> H[Collect entries]
    G --> H
    H --> I[Batch into PM_CONF<br>50 entries per message]
    I --> J{Last batch?}
    J -->|no| K["Set flags=0x00"]
    J -->|yes| L["Set flags=0x01 (last)"]
    K --> M[Send to mobile]
    L --> M
    M --> N[Do NOT forward to firmware]

    style F fill:#2d5,color:#fff
    style G fill:#d55,color:#fff
```

---

## 10. Firmware Patch Points

```mermaid
flowchart LR
    subgraph "Firmware Binary (IRAM)"
        cons["cons_puts<br>0x82a1b0"]
        puts_fn["puts<br>0x829ea0"]
        printf_calls["bl printf × 5<br>in frame_irq"]
        talloc["_talloc_zero<br>0x82c2bc"]
        pool["msgb pool<br>0x833b5c<br>332 B × 148"]
        abort["handle_abort<br>0x821f5c"]
        panic["talloc panic<br>0x82c34c"]
    end

    subgraph "Patches Applied"
        p1["bx lr<br>(return immediately)"]
        p2["bx lr"]
        p3["nop × 5"]
        p4["cmp r3, #148<br>(was #32)"]
        p5["msr CPSR_c, r8<br>b retry"]
        p6["msr CPSR_c, #0<br>(IRQs enabled)"]
    end

    cons -.->|NOP| p1
    puts_fn -.->|NOP| p2
    printf_calls -.->|NOP| p3
    talloc -.->|expand| p4
    panic -.->|retry| p5
    abort -.->|keep IRQs| p6

    style p1 fill:#f80,color:#fff
    style p2 fill:#f80,color:#fff
    style p3 fill:#f80,color:#fff
    style p4 fill:#08f,color:#fff
    style p5 fill:#08f,color:#fff
    style p6 fill:#08f,color:#fff
```

---

## 11. Class Diagram (QEMU Device Types)

```mermaid
classDiagram
    class CalypsoMachine {
        +ARMCPU *cpu
        +CalypsoSoCState soc
        +MemoryRegion xram
        +MemoryRegion bootrom
        +calypso_machine_init()
    }

    class CalypsoSoC {
        +MemoryRegion iram
        +CalypsoUARTState uart_modem
        +CalypsoUARTState uart_irda
        +CalypsoINTHState inth
        +qemu_irq[32] irqs
        +int32_t trx_port
        +bool enable_trx
        +calypso_soc_realize()
    }

    class CalypsoTRX {
        +qemu_irq *irqs
        +uint16_t dsp_ram[32K]
        +uint16_t tpu_regs[]
        +QEMUTimer *tdma_timer
        +uint32_t fn
        +SyncState sync_state
        +uint32_t pm_count
        +bool fw_patched
        +calypso_trx_init()
        +calypso_fw_patch()
        +calypso_dsp_process()
        +calypso_sync_tick()
        +calypso_tdma_tick()
    }

    class CalypsoUART {
        +CharBackend chr
        +qemu_irq irq
        +uint8_t rx_fifo[8192]
        +bool thr_empty_pending
        +uint8_t tx_empty_reads
        +QEMUTimer *rx_poll_timer
        +calypso_uart_kick_rx()
        +calypso_uart_kick_tx()
        +calypso_uart_poll_backend()
    }

    class CalypsoINTH {
        +uint32_t levels
        +uint32_t mask
        +uint16_t ilr[32]
        +uint16_t ith_v
        +qemu_irq parent_irq
        +qemu_irq parent_fiq
        +calypso_inth_update()
    }

    class SyncState {
        <<enumeration>>
        IDLE
        FCCH_SEARCH
        FCCH_FOUND
        SCH_SEARCH
        LOCKED
    }

    CalypsoMachine *-- CalypsoSoC
    CalypsoSoC *-- CalypsoUART : uart_modem
    CalypsoSoC *-- CalypsoUART : uart_irda
    CalypsoSoC *-- CalypsoINTH
    CalypsoSoC --> CalypsoTRX : calypso_trx_init()
    CalypsoTRX --> SyncState
    CalypsoTRX --> CalypsoINTH : irqs
    CalypsoUART --> CalypsoINTH : irq line
```

---

## 12. Deployment Diagram

```mermaid
graph TB
    subgraph "Linux Host"
        subgraph "Process: QEMU"
            qemu_bin["qemu-system-arm<br>-M calypso -cpu arm946"]
            pty1["/dev/pts/X<br>(modem)"]
            pty2["/dev/pts/Y<br>(irda)"]
            mon["monitor socket<br>/tmp/qemu-calypso-mon.sock"]
            udp1["UDP 6700<br>TRXC"]
            udp2["UDP 6701<br>TRXD"]
            udp3["UDP 4729<br>GSMTAP"]
        end

        subgraph "Process: Bridge"
            bridge_py["python3 l1ctl_bridge.py"]
            l2sock["/tmp/osmocom_l2<br>unix socket"]
        end

        subgraph "Process: Mobile"
            mobile_bin["mobile<br>(osmocom-bb)"]
            telnet["telnet :4247"]
        end

        subgraph "Process: Wireshark (optional)"
            ws["wireshark<br>-k -i lo -f 'udp port 4729'"]
        end
    end

    mobile_bin <-->|L1CTL| l2sock
    l2sock <--> bridge_py
    bridge_py <-->|sercomm| pty1
    pty1 --- qemu_bin
    pty2 --- qemu_bin
    mon --- qemu_bin
    udp3 -.-> ws

    style qemu_bin fill:#236,color:#fff
    style bridge_py fill:#362,color:#fff
    style mobile_bin fill:#623,color:#fff
```
