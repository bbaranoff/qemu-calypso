# L1CTL socket flow — QEMU ↔ osmocom-bb layer23

> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](../DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> Spécifiquement pour ce doc : l'API **burst-mode** (`l1ctl_burst_mode`, `l1ctl_set_burst_mode`, champ `burst_mode`, `fbsb_requested`) N'EXISTE PAS dans le code (grep = 0 dans `l1ctl_sock.c`, headers, `calypso_trx.c`). Le handshake `vm_start` / `cli_rx_enabled` de `l1ctl_accept_cb` N'EXISTE PAS non plus. Et `l1ctl_sock_init` N'EST PAS commentée : elle est **active** à `calypso_soc.c:333-334` (path défaut = `/tmp/osmocom_l2`). Voir annotations inline.

Référence: `hw/arm/calypso/l1ctl_sock.c` (QEMU) et
`/opt/GSM/osmocom-bb/src/host/layer23/src/common/l1l2_interface.c` (mobile/L23).

## 1. Topologie

```
                       /tmp/osmocom_l2_1 (AF_UNIX SOCK_STREAM)
                                  │
        ┌─────────────────────────┴───────────────────┐
        │                                             │
   ┌────┴────┐                                  ┌─────┴────┐
   │  QEMU   │                                  │  mobile  │
   │ l1ctl_  │  ◀─── len-prefix L1CTL  ───▶     │ ccch_scan│
   │ sock.c  │                                  │   (L23)  │
   └────┬────┘                                  └──────────┘
        │
        ▼ (sercomm wrap/unwrap, DLCI=L1CTL)
   ┌─────────┐
   │ firmware│
   │  UART   │
   └─────────┘
```

QEMU est le **serveur** (listen + accept). Mobile/ccch_scan sont les
**clients** (connect via `osmo_sock_unix_init_ofd(... OSMO_SOCK_F_CONNECT)`).

## 2. Format wire

Identique des deux côtés :
```
[len_hi:1] [len_lo:1] [L1CTL message]
```
- `len` = big-endian, 16 bits, longueur du message L1CTL (sans le header de
  2 bytes).
- L1CTL message format (cf `include/l1ctl_proto.h`) :
  ```
  msg_type:1   flags:1   padding:2   payload[*]
  ```

## 3. Flow d'init

### Côté QEMU (`l1ctl_sock_init`, l1ctl_sock.c:436)

```c
unlink(path);
srv_fd = socket(AF_UNIX, SOCK_STREAM, 0);
bind(srv_fd, ...);
listen(srv_fd, 1);
fcntl(srv_fd, O_NONBLOCK);
qemu_set_fd_handler(srv_fd, l1ctl_accept_cb, NULL, s);
```

Appelée depuis `calypso_soc.c:333-334` (~~actuellement **commentée**~~ — **FAUX**: le code est **actif/non commenté** à `calypso_soc.c:333-334`; path défaut = `/tmp/osmocom_l2`) :
```c
const char *l1ctl_path = getenv("L1CTL_SOCK");
l1ctl_sock_init(&s->uart_modem, l1ctl_path ? l1ctl_path : "/tmp/osmocom_l2");
```

### Côté osmocom L23 (`layer2_open`, l1l2_interface.c:105)

```c
osmo_sock_unix_init_ofd(&ms->l2_wq.bfd, SOCK_STREAM, 0,
                        socket_path, OSMO_SOCK_F_CONNECT);
osmo_wqueue_init(&ms->l2_wq, 100);
ms->l2_wq.read_cb  = layer2_read;
ms->l2_wq.write_cb = layer2_write;
```

`socket_path` par défaut = `L2_DEFAULT_SOCKET_PATH = "/tmp/osmocom_l2"`,
overridable via `-s` sur la ligne de commande mobile. Dans nos runs c'est
`/tmp/osmocom_l2_1`.

## 4. Flow connection

### QEMU `l1ctl_accept_cb` (l1ctl_sock.c:407)

1. `accept()` → `cli_fd`. Un seul client à la fois (l'ancien est fermé).
2. `cli_fd` non bloquant.
3. ~~Si VM en `-S` (paused) → `vm_start()`, mais `cli_rx_enabled = false`~~ —
   **FAUX**: `l1ctl_accept_cb` (l1ctl_sock.c:407-432) ne contient **ni**
   `vm_start` **ni** `cli_rx_enabled` (grep = 0 dans le fichier). Il reset
   `lp_len`/`sc_state`/`sc_len` puis register `l1ctl_client_readable`
   **inconditionnellement** (l1ctl_sock.c:430).
4. ~~Si VM déjà running (reconnect) → `cli_rx_enabled = true`~~ —
   **FAUX**: pas de gating `cli_rx_enabled`; le handler RX est armé dès
   l'accept dans tous les cas (l1ctl_sock.c:430).

## 5. Flow firmware → mobile (TX)

Hook `l1ctl_sock_uart_tx_byte()` (l1ctl_sock.c:257) appelé par
`calypso_uart.c` pour chaque byte sortant de l'UART firmware.

Machine d'états sercomm :
```
SC_IDLE       → byte == FLAG ? → SC_IN_FRAME, sc_len=0
SC_IN_FRAME   → byte == FLAG  ? → si sc_len>0 → sercomm_frame_complete()
              → byte == ESC   ? → SC_ESCAPE
              → sinon         → sc_buf[sc_len++] = byte
SC_ESCAPE     → sc_buf[sc_len++] = byte ^ ESCAPE_XOR ; → SC_IN_FRAME
```

`sercomm_frame_complete` (l1ctl_sock.c:189) :
1. `dlci = sc_buf[0]`, `payload = sc_buf+2`, `plen = sc_len-2`
2. Si `dlci == SERCOMM_DLCI_L1CTL` :
   - ~~**Premier frame firmware** : drain stale bytes mobile → `cli_rx_enabled=true` → register `l1ctl_client_readable`~~ —
     **FAUX**: aucun gating "premier frame" ni `cli_rx_enabled` dans
     `sercomm_frame_complete` (l1ctl_sock.c:189-253); le handler RX est déjà
     armé depuis `l1ctl_accept_cb`.
   - Log `TX→mobile: <name> (0x<mt>) len=<plen>`
   - Décode messages spécifiques (PM_CONF, FBSB_CONF, etc.)
   - `l1ctl_send_to_mobile(s, payload, plen)` →
     ```c
     hdr[0] = (len >> 8) & 0xFF;
     hdr[1] = len & 0xFF;
     send(cli_fd, hdr, 2);
     send(cli_fd, payload, len);
     ```

Côté L23, `layer2_read` (l1l2_interface.c:44) :
```c
read(fd, &len, 2);          // 16 bits
len = ntohs(len);
read(fd, msg->l1h, len);    // payload
l1ctl_recv(ms, msg);        // dispatch dans le L23
```

## 6. Flow mobile → firmware (RX)

`l1ctl_client_readable` (l1ctl_sock.c:296) :
1. `recv(cli_fd, tmp, sizeof(tmp))` → accumule dans `lp_buf`
2. Tant que `lp_len >= 2 + msglen` :
   - `msglen = (lp_buf[0]<<8) | lp_buf[1]`
   - `payload = &lp_buf[2]`
   - ~~**Track burst gating** : `payload[0]==0x01 (FBSB_REQ)` → `fbsb_requested=true`; `payload[0]==0x0d (RESET_REQ)` → `burst_mode=false; fbsb_requested=false`~~ —
     **FAUX**: ni `fbsb_requested` ni `burst_mode` n'existent (grep = 0). Aucun
     tracking d'état burst dans `l1ctl_client_readable` (l1ctl_sock.c:296).
   - `flen = sercomm_wrap(SERCOMM_DLCI_L1CTL, payload, msglen, frame, ...)`
     (encapsule en sercomm avec FLAG/ESCAPE)
   - `calypso_uart_inject_raw(uart, frame, flen)` → bytes injectés dans
     l'UART RX du firmware → reçus par `comm/sercomm.c` côté firmware

Côté L23, `layer2_write` (l1l2_interface.c:89) écrit directement
`msg->data` (qui contient déjà le len-prefix posé en amont par le code
constructeur du msg).

## 7. Burst gating — ⚠️ SECTION PÉRIMÉE (API inexistante)

> **FAUX — cette API n'existe pas dans le code.** `grep 'burst_mode\|l1ctl_burst_mode\|l1ctl_set_burst_mode'`
> = **0 occurrence** dans `l1ctl_sock.c`, les headers, et `calypso_trx.c`. La
> struct `L1CTLSock` (l1ctl_sock.c:74) n'a pas de champ `burst_mode`. Les
> lignes `l1ctl_sock.c:380/387` citées ci-dessous sont **à l'intérieur de
> `l1ctl_client_readable`** (296) et ne sont pas une API burst.

~~`l1ctl_burst_mode()` (l1ctl_sock.c:380) : `return g_l1ctl.burst_mode;`~~ — n'existe pas.

~~`burst_mode = true` ssi `fbsb_requested == true` ET TPU_CTRL_EN écrit par firmware
(calypso_trx.c appelle `l1ctl_set_burst_mode(true)`)~~ — n'existe pas.

~~Utilisé pour gater le forwarding TRXD UDP~~ — le rationale de gating TRXD UDP
est de la prose non vérifiable : aucun mécanisme correspondant dans le code.

## 8. Conflit avec calypso-ipc-device

Commentaire dans `calypso_soc.c:228-229` :
> "Having both QEMU and calypso-ipc-device write to the same socket interleaves
>  bytes and corrupts L1CTL headers (Short L1CTL message len=1)"

Concrètement :
- `calypso-ipc-device` lit le PTY de QEMU (sercomm UART) et écrit sur sa propre
  socket Unix `/tmp/osmocom_l2_1` (en Python, même format len-prefix).
- Si on active aussi `l1ctl_sock_init` → QEMU crée la même socket → race
  bind() OU bytes mobile arrivent dans une seule des deux instances → désync.

> **MàJ audit 2026-07-01** : `l1ctl_sock_init` n'est plus commentée — elle est
> **active** à `calypso_soc.c:333-334`. L'étape 1 ci-dessous (« décommenter »)
> est donc **DONE** ; il reste à traiter la coexistence avec calypso-ipc-device
> (étapes 2-3).

Solution pour réactiver QEMU L1CTL :
1. ~~Décommenter~~ **✅ DONE** — `l1ctl_sock_init(&s->uart_modem, ...)` est
   déjà appelée (active) à `calypso_soc.c:333-334`.
2. Modifier calypso-ipc-device pour **ne plus créer la socket Unix** : retire la
   logique L1CTL bridge ↔ socket, ne garde que TRXD UDP + PTY (s'il y en
   a encore besoin pour autre chose) — ou virer calypso-ipc-device complètement
   du flow L1CTL.
3. Vérifier que `l1ctl_sock_uart_tx_byte()` est bien hooké dans
   `calypso_uart.c` sur le chemin TX du firmware.

## 8b. Hook UART → l1ctl_sock (vérifié)

`l1ctl_sock_uart_tx_byte()` n'est PAS orphelin : il est bien câblé depuis
`hw/char/calypso_uart.c:702-704`, sur le chemin TX du THR (Transmit Holding
Register) du UART, mais **uniquement pour le label `"modem"`** :

```c
(void)qemu_chr_fe_write(&s->chr, &ch, 1);   // calypso_uart.c:697 → PTY, NON-BLOQUANT (drop si backpressure)
if (s->label && !strcmp(s->label, "modem")) {  // :702
    l1ctl_sock_uart_tx_byte(ch);            // :703 → parser sercomm interne
}
```

> Note (corrigé): le write vers le chardev PTY est `qemu_chr_fe_write` (l.697),
> **non bloquant et volontairement pas `write_all`** (cf commentaire
> calypso_uart.c:690-696 : le backpressure amplifiait la boucle LOST). La ligne
> 505 citée auparavant est sans rapport.

Donc chaque byte TX du UART modem du firmware part **simultanément** vers
deux destinations : le chardev PTY (que calypso-ipc-device lit) ET le parser sercomm
interne `l1ctl_sock`. L'UART irda n'a pas ce double-tap.

Si `l1ctl_sock_init` est actif mais qu'aucun client n'est connecté,
`l1ctl_sock_uart_tx_byte` parse quand même les frames mais `l1ctl_send_to_mobile`
les drop dès la première ligne (`if (s->cli_fd < 0) return;`) → no-op silencieux.
(NB: `l1ctl_sock_init` **n'est plus désactivé** — cf calypso_soc.c:333-334.)

## 8c. Master/Slave et ordre de démarrage

| Connexion           | Type                 | Master (server)                | Slave (client)            | État actuel                 |
|---------------------|----------------------|--------------------------------|---------------------------|------------------------------|
| `/tmp/osmocom_l2_1` | AF_UNIX SOCK_STREAM  | **calypso-ipc-device** (Python)         | mobile, ccch_scan         | actif                        |
| `/tmp/osmocom_l2_1` | (alt) AF_UNIX        | QEMU `l1ctl_sock_init`         | mobile, ccch_scan         | **actif** (calypso_soc.c:333-334, path défaut `/tmp/osmocom_l2`) |
| PTY `/dev/pts/N`    | tty                  | QEMU (`-serial pty`, master)   | calypso-ipc-device (lit le slave)  | actif                        |
| `udp:6700` (CLK)    | UDP datagram         | QEMU `sercomm_gate.c`          | calypso-ipc-device                 | actif                        |
| `udp:6701` (TRXC)   | UDP datagram         | QEMU                           | calypso-ipc-device                 | actif                        |
| `udp:6702` (TRXD)   | UDP datagram         | QEMU                           | calypso-ipc-device / inject_cfile  | actif                        |
| `tcp:1234` (gdb)    | TCP gdb stub         | QEMU (`-gdb tcp::1234`)        | gdb / hack_gdb.py         | actif (run_all_debug.sh)     |

**Ordre de démarrage `run_all_debug.sh`** :

```
1. QEMU start (-S -gdb)
   ├─ alloue PTY               (master tty)
   ├─ ouvre UDP 6700/6701/6702 (server)
   ├─ ouvre gdb stub tcp:1234   (server)
   └─ VM paused (waiting for vm_start)

2. calypso-ipc-device start
   ├─ bind /tmp/osmocom_l2_1   (server SOCK_STREAM)
   ├─ open(PTY)                 (slave reader/writer)
   └─ connect UDP 6700-6702     (client of QEMU UDP servers)

3. inject_cfile.py start
   └─ connect UDP 6702          (client) — push I/Q samples

4. mobile start (sleep 5)
   └─ connect /tmp/osmocom_l2_1 (client) → calypso-ipc-device accept()
       └─ calypso-ipc-device forward L1CTL bytes vers PTY → QEMU UART RX

5. (hack_gdb.py désactivé)
   └─ aurait connecté tcp:1234 et posé des BP firmware
```

**Path L1CTL complet TX firmware → mobile** (état actuel) :
```
firmware code
  → write THR sur UART modem (calypso_uart.c)
  → qemu_chr_fe_write_all → PTY master
  → calypso-ipc-device read PTY (slave fd)
  → calypso-ipc-device parse sercomm (Python)
  → calypso-ipc-device send len-prefix sur /tmp/osmocom_l2_1 (server fd)
  → mobile recv (client fd)
  → mobile l1ctl_recv → L23 dispatch
```

(Le hook `l1ctl_sock_uart_tx_byte` côté QEMU parse aussi en parallèle
mais drop tout puisque `cli_fd<0`.)

**Path RX mobile → firmware** :
```
mobile l1ctl_send → write sur /tmp/osmocom_l2_1 (client)
  → calypso-ipc-device recv (server)
  → calypso-ipc-device wrap sercomm
  → calypso-ipc-device write PTY (slave)
  → QEMU lit PTY → injection UART RX du firmware
  → comm/sercomm.c côté firmware → unwrap → l1ctl_recv firmware
```

## 8d. Sync avec osmocom (côté L23)

| QEMU `l1ctl_sock.c`                          | osmocom `l1l2_interface.c`                                 |
|----------------------------------------------|------------------------------------------------------------|
| `socket(AF_UNIX, SOCK_STREAM)` + listen      | `osmo_sock_unix_init_ofd(SOCK_STREAM, OSMO_SOCK_F_CONNECT)` |
| `hdr[0]=(len>>8)&0xFF; hdr[1]=len&0xFF;`     | `read(fd, &len, 2); len = ntohs(len);`                     |
| `send(hdr,2); send(payload, len)`            | `read(fd, msg->l1h, len)`                                  |
| `recv(lp_buf, ...); msglen=lp[0]<<8\|lp[1]`  | `write(fd, msg->data, msg->len)` (msg already len-prefixed)|

Le wire format est **strictement compatible** : len 16-bit big-endian +
payload L1CTL brut. Pas d'endianness différente, pas de framing
supplémentaire. Direct drop-in entre les deux.

## 9. Variables d'env

- `L1CTL_SOCK` : override le path par défaut `/tmp/osmocom_l2` (lu dans
  `calypso_soc.c:333`).
- Côté osmocom : `mobile -s /tmp/osmocom_l2_1 ...` ou `ccch_scan -s ...`.

## 10. Récap mapping types L1CTL

| msg_type | Nom         | Direction       |
|----------|-------------|-----------------|
| 0x01     | FBSB_REQ    | mobile → fw     |
| 0x02     | FBSB_CONF   | fw → mobile     |
| 0x03     | DATA_IND    | fw → mobile     |
| 0x04     | RACH_REQ    | mobile → fw     |
| 0x05     | RACH_CONF   | fw → mobile     |
| 0x06     | DATA_REQ    | mobile → fw     |
| 0x07     | RESET_IND   | fw → mobile     |
| 0x08     | PM_REQ      | mobile → fw     |
| 0x09     | PM_CONF     | fw → mobile     |
| 0x0d     | RESET_REQ   | mobile → fw     |
| 0x0e     | RESET_CONF  | fw → mobile     |
