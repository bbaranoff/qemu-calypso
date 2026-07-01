# Sercomm Gate Architecture — QEMU Calypso

## 1. Le vrai hardware Calypso

Le Calypso a deux chemins de données complètement séparés :

### Chemin radio (bursts)
```
Antenne → RF frontend → ABB (Analog Baseband)
    → BSP (Baseband Serial Port, hardware)
    → DSP lit via PORTR PA=0xF430
    → DSP traite (FIR, equalizer, Viterbi)
    → Résultats dans API RAM (DB read page)
    → ARM lit les résultats
```
Le BSP est un port série hardware (registre à 0xF430 dans l'espace I/O du DSP C54x).
Le DSP reçoit un BRINT0 (interrupt vec 21, IMR bit 5) quand un burst complet est disponible.
L'ARM ne touche jamais aux bursts radio — c'est 100% hardware BSP → DSP.

### Chemin contrôle (L1CTL / sercomm)
```
Host (mobile/ccch_scan) → UART PTY → sercomm HDLC
    → DLCI 5 (L1A_L23) → firmware ARM (l1a_l23_rx callback)
    → Firmware écrit tâches dans API DB write page
    → d_dsp_page = B_GSM_TASK | page
    → TPU frame IRQ → SINT17 → DSP exécute
```
L'UART ne transporte que du L1CTL et du debug. Jamais de bursts.

## 2. Protocole sercomm (source: osmocom-bb/src/target/firmware/comm/sercomm.c)

### Format trame
```
FLAG(0x7E) | DLCI(1) | CTRL(0x03) | DATA(N) | FLAG(0x7E)
```

### Escaping
Les octets 0x7E, 0x7D et 0x00 sont échappés :
- Remplacés par `0x7D` suivi de `octet XOR 0x20`
- Décodage : quand on reçoit 0x7D, le byte suivant est XOR 0x20

### DLCIs enregistrés par le firmware layer1
| DLCI | Constante | Callback | Usage |
|------|-----------|----------|-------|
| 4 | SC_DLCI_DEBUG | **aucun** dans layer1 | Debug (non utilisé) |
| 5 | SC_DLCI_L1A_L23 | `l1a_l23_rx` | **L1CTL** — commandes mobile↔firmware |
| 9 | SC_DLCI_LOADER | `cmd_handler` (loader only) | Chargement firmware |
| 10 | SC_DLCI_CONSOLE | non enregistré dans layer1 | Console texte |
| 128 | SC_DLCI_ECHO | `sercomm_sendmsg` (loopback) | Test echo |

### State machine RX (sercomm_drv_rx_char)
```
WAIT_START ──(0x7E)──→ ADDR ──(byte)──→ CTRL ──(byte)──→ DATA
                                                           │
                                                    (0x7D)→ ESCAPE ──(byte^0x20)──→ DATA
                                                    (0x7E)→ dispatch_rx_msg(dlci, msg) → WAIT_START
```

### Binding UART
- Compal E88 : `sercomm_bind_uart(UART_MODEM)` (board/compal_e88/init.c:104)
- UART modem = 0xFFFF5800 (notre calypso_uart "modem")
- IRQ handler : `uart_irq_handler_sercomm` dans calypso/uart.c

### Flow RX complet (vrai hardware)
```
UART RHR register → uart_irq_handler_sercomm (IRQ)
    → uart_getchar_nb() lit chaque byte
    → sercomm_drv_rx_char(ch) parse HDLC
    → quand trame complète: dispatch_rx_msg(dlci, msg)
    → callback[dlci](dlci, msg)
    → pour DLCI 5: l1a_l23_rx() enqueue dans l23_rx_queue
    → l1a_l23_handler() (appelé depuis main loop) déqueue et traite
```

### Flow TX complet (vrai hardware)
```
Firmware veut envoyer (ex: L1CTL_FBSB_CONF) :
    → sercomm_sendmsg(SC_DLCI_L1A_L23, msg)
    → msgb_push(msg, 2) pour ajouter DLCI + CTRL en tête
    → enqueue dans dlci_queues[5]
    → uart_irq_enable(UART_IRQ_TX_EMPTY, 1)
    → uart_irq_handler_sercomm (THR interrupt)
    → sercomm_drv_pull(&ch) lit un byte de la queue
    → uart_putchar_nb(ch) écrit dans UART THR
    → byte sort sur le PTY → host
```

## 3. DSP Frame Dispatch (source: calypso/dsp.c)

### Séquence par frame TDMA
```
1. ARM écrit tâches dans DB write page :
   dsp_api.db_w->d_task_d  = FB_DSP_TASK (5) ou NB_DSP_TASK (21) etc.
   dsp_api.db_w->d_burst_d = burst_id (0-3)
   dsp_api.db_w->d_ctrl_system |= tsc & 7

2. ARM appelle dsp_end_scenario() :
   dsp_api.ndb->d_dsp_page = B_GSM_TASK | dsp_api.w_page
   dsp_api.w_page ^= 1  (flip page)
   tpu_dsp_frameirq_enable()  → TPU_CTRL |= DSP_EN
   tpu_frame_irq_en(1, 1)

3. TPU hardware génère SINT17 (frame IRQ) au DSP

4. DSP ROM dispatcher :
   - Lit d_dsp_page (DSP addr 0x08D4)
   - Vérifie B_GSM_TASK (bit 1)
   - Lit page number (bit 0) → sélectionne DB page 0 ou 1
   - Exécute d_task_d (DL), d_task_u (UL), d_task_md (monitoring)
   - Écrit résultats dans DB read page (a_pm, a_serv_demod, a_sch)
   - Fait IDLE

5. ARM lit résultats de DB read page :
   dsp_api.db_r->a_serv_demod[D_TOA/D_PM/D_ANGLE/D_SNR]
   dsp_api.db_r->a_pm[0..2]
```

### Constantes
```c
B_GSM_TASK       = (1 << 1) = 0x02  // Task flag in d_dsp_page
B_GSM_PAGE       = (1 << 0) = 0x01  // Page select in d_dsp_page
// d_dsp_page = 0x02 (page 0, task) ou 0x03 (page 1, task)

BASE_API_NDB     = 0xFFD001A8  // ARM address
BASE_API_W_PAGE_0= 0xFFD00000  // 20 words MCU→DSP
BASE_API_W_PAGE_1= 0xFFD00028
BASE_API_R_PAGE_0= 0xFFD00050  // 20 words DSP→MCU
BASE_API_R_PAGE_1= 0xFFD00078
BASE_API_PARAM   = 0xFFD00862  // 57 words params
```

### DSP Task IDs (l1_environment.h)
```c
NO_DSP_TASK       =  0  // No task
FB_DSP_TASK       =  5  // Frequency Burst (idle)
SB_DSP_TASK       =  6  // Sync Burst (idle)
TCH_FB_DSP_TASK   =  8  // Frequency Burst (dedicated)
TCH_SB_DSP_TASK   =  9  // Sync Burst (dedicated)
RACH_DSP_TASK     = 10  // RACH transmit
AUL_DSP_TASK      = 11  // SACCH UL
DUL_DSP_TASK      = 12  // SDCCH UL
TCHT_DSP_TASK     = 13  // TCH traffic
NBN_DSP_TASK      = 17  // Normal BCCH neighbour
EBN_DSP_TASK      = 18  // Extended BCCH neighbour
NBS_DSP_TASK      = 19  // Normal BCCH serving
NP_DSP_TASK       = 21  // Normal Paging
EP_DSP_TASK       = 22  // Extended Paging
ALLC_DSP_TASK     = 24  // CCCH reading
CB_DSP_TASK       = 25  // CBCH
DDL_DSP_TASK      = 26  // SDCCH DL
ADL_DSP_TASK      = 27  // SACCH DL
TCHD_DSP_TASK     = 28  // TCH traffic DL
CHECKSUM_DSP_TASK = 33  // DSP checksum
```

## 4. BSP — Baseband Serial Port

### Hardware
Le BSP est un port série synchrone du DSP C54x qui connecte directement à l'ABB (Analog Baseband).
- **Port address** : 0xF430 (BSP data register)
- **Interrupt** : BRINT0 (vec 21, IMR bit 5) — "BSP Receive Interrupt"
- **Data format** : int16 I/Q samples, 1 sample par symbole GSM

### Flow DL (downlink — BTS → phone)
```
Vrai hardware :
  ABB convertit le signal RF en baseband I/Q
  → BSP DMA transfère les samples dans un buffer DSP
  → BRINT0 signale "burst reçu"
  → DSP traite : dérotation, FIR, equalizer, Viterbi decode
  → Résultats dans API RAM

QEMU émulation :
  osmo-bts-trx → TRXD UDP (soft bits)
  → bridge (sercomm_udp.py) GMSK modulation → int16 I/Q
  → calypso_trx_rx_burst()
  → c54x_bsp_load(dsp, samples, n)  // charge bsp_buf[]
  → c54x_interrupt_ex(dsp, 21, 5)   // BRINT0
  → DSP lit via PORTR PA=0xF430     // bsp_buf[bsp_pos++]
```

### C54x BSP implementation (calypso_c54x.c)
```c
// Structure
uint16_t bsp_buf[160];  // burst samples
int      bsp_len;       // number of samples
int      bsp_pos;       // read position

// Load (called by calypso_trx.c)
void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n);

// Read (called by PORTR instruction)
if (op2 == 0xF430 && s->bsp_pos < s->bsp_len)
    data_write(s, addr, s->bsp_buf[s->bsp_pos++]);
```

## 5. Architecture QEMU — Ce qu'il faut implémenter

### Chemins de données
```
┌─────────────────────────────────────────────────────────┐
│                    QEMU Calypso                         │
│                                                         │
│  ┌──────────┐    TRXD UDP     ┌──────────────────┐     │
│  │  Bridge   │───────────────→│ calypso_trx.c    │     │
│  │ (python)  │                │  rx_burst()      │     │
│  └──────────┘                 │  → c54x_bsp_load │     │
│       ↑                       │  → BRINT0        │     │
│       │ PTY                   └────────┬─────────┘     │
│       │                                │               │
│  ┌────┴─────┐                    ┌─────┴──────┐       │
│  │  UART    │    sercomm_gate    │  DSP C54x  │       │
│  │  modem   │───────────────→    │  PORTR F430│       │
│  │          │  DLCI 5 → FIFO    │  bsp_buf[] │       │
│  └──────────┘  (L1CTL only)     └────────────┘       │
│       ↑                                               │
│       │ L1CTL socket                                  │
│  ┌────┴─────┐                                         │
│  │ l1ctl    │                                         │
│  │ _sock.c  │  ← mobile/ccch_scan                    │
│  └──────────┘                                         │
└─────────────────────────────────────────────────────────┘
```

### sercomm_gate.c — Rôle exact
Le gate parse le flux sercomm entrant sur l'UART modem et route par DLCI :
- **Tous les DLCIs** → re-wrap et push dans le FIFO UART (firmware ARM les traite)
- **Pas de routage spécial** pour DLCI 4 — le firmware n'a pas de handler pour DLCI 4
- Le gate ne touche PAS aux bursts — ils arrivent par un autre chemin (TRXD → BSP)

Le gate remplace le parser sercomm inline qui était dans calypso_uart.c.
C'est un simple parser HDLC qui re-injecte les trames dans le FIFO.

### Bridge (sercomm_udp.py) — Deux rôles
1. **Bursts DL** : BTS TRXD → GMSK modulation → écriture directe vers QEMU
   (actuellement via PTY sercomm DLCI 4 — **à changer** en UDP/socket direct)
2. **Clock** : CLK IND → BTS pour synchronisation
3. **Bursts UL** : PTY sercomm DLCI 4 → TRXD → BTS
   (firmware envoie les bursts UL via sercomm_sendmsg)

### Problème actuel du bridge
Le bridge envoie les bursts DL via le PTY en sercomm DLCI 4. C'est incorrect :
- Sur le vrai hardware, les bursts DL arrivent par le BSP, pas l'UART
- Le firmware n'a pas de handler pour DLCI 4 (SC_DLCI_DEBUG)
- Les bursts DL dans le FIFO UART polluent le firmware

**Solution** : le bridge doit envoyer les bursts DL par un canal séparé
(UDP socket, pipe, ou shared memory) directement à calypso_trx_rx_burst(),
qui charge le BSP via c54x_bsp_load() et fire BRINT0.

### NDB d_dsp_page — Mapping mémoire
```
ARM offset 0x01A8 = DSP addr 0x08D4 = d_dsp_page
  bit 0 = page number (0 ou 1)
  bit 1 = B_GSM_TASK (1 = tâche à exécuter)
  
ARM offset 0x01C4 = DSP addr 0x08E2 = d_dsp_state
  0 = run, 1 = Idle1, 2 = Idle2, 3 = Idle3
  Firmware init: d_dsp_state = 3 (C_DSP_IDLE3)
```

## 6. Résumé des fichiers

| Fichier | Rôle | Touche aux bursts ? |
|---------|------|---------------------|
| sercomm_gate.c | Parse sercomm UART → FIFO (L1CTL) | **Non** |
| calypso_uart.c | Hardware UART, appelle sercomm_gate | **Non** |
| calypso_trx.c | TDMA tick, BSP load, SINT17, TPU | **Oui** (rx_burst → bsp_load) |
| calypso_c54x.c | DSP emulation, PORTR 0xF430 | **Oui** (bsp_buf read) |
| sercomm_udp.py | Bridge BTS↔QEMU | **Oui** (TRXD → GMSK → PTY/BSP) |
| l1ctl_sock.c | L1CTL socket ↔ mobile | **Non** |
