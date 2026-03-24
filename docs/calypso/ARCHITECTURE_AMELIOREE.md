# Architecture Calypso Améliorée - Guide Complet

## Vue d'ensemble

Cette nouvelle architecture suit les meilleures pratiques QEMU avec une séparation claire entre :
- **Machine** (`calypso_mb.c`) - Composants au niveau carte (CPU, RAM externe, Flash)
- **SoC** (`calypso_soc.c`) - Périphériques intégrés au chip
- **Périphériques** (UART, Timer, etc.) - Devices QOM réutilisables

## Comparaison : Ancienne vs Nouvelle Architecture

### ❌ Ancienne architecture (calypso_high.c)
```
calypso_high.c (tout dans un fichier)
├── CPU
├── RAM interne
├── RAM externe  
├── Flash
├── INTH (créé inline)
├── Timer1 (créé inline)
├── Timer2 (créé inline)
├── UART modem (créé inline)
├── UART irda (créé inline)
├── SPI (créé inline)
└── TRX bridge (créé inline)
```

**Problèmes:**
- Tout dans un seul fichier (500+ lignes)
- Difficile à maintenir
- Pas de réutilisation possible
- Mélange des niveaux d'abstraction

### ✅ Nouvelle architecture (modulaire)

```
calypso_mb.c (Machine)
├── CPU (ARM946E-S)
├── Calypso SoC (device QOM)
│   ├── RAM interne (256 KiB)
│   ├── INTH (interrupt controller)
│   ├── Timer1
│   ├── Timer2
│   ├── UART modem
│   ├── UART irda
│   ├── SPI + TWL3025
│   └── TRX bridge (DSP/TPU/TSP/ULPD)
├── RAM externe (8 MiB)
└── Flash (4 MiB NOR)
```

**Avantages:**
- Séparation claire des responsabilités
- SoC réutilisable dans d'autres machines
- Facilité de test et debug
- Code plus maintenable
- Suit les conventions QEMU

## Structure des fichiers

### 1. calypso_soc.h / calypso_soc.c
**Rôle:** Device QOM représentant le SoC complet

**Contient:**
- RAM interne (256 KiB)
- Tous les périphériques intégrés (INTH, Timers, UARTs, SPI, TRX)
- Connexions internes entre périphériques
- Exports : 2 IRQ outputs (IRQ/FIQ vers CPU)

**Propriétés configurables:**
```c
qdev_prop_set_int32(soc, "trx-port", 4729);      // Port UDP pour TRX
qdev_prop_set_bit(soc, "enable-trx", true);      // Activer/désactiver TRX
```

**Usage typique:**
```c
CalypsoSoCState soc;
object_initialize_child(OBJECT(machine), "soc", &soc, TYPE_CALYPSO_SOC);
sysbus_realize(SYS_BUS_DEVICE(&soc), &err);

// Connecter au CPU
sysbus_connect_irq(SYS_BUS_DEVICE(&soc), 0, cpu_irq);
sysbus_connect_irq(SYS_BUS_DEVICE(&soc), 1, cpu_fiq);
```

### 2. calypso_mb.c
**Rôle:** Machine board complète

**Contient:**
- Instanciation du CPU
- Instanciation du SoC
- RAM externe (8 MiB)
- Flash NOR (4 MiB)
- Chargement du firmware
- Aliases mémoire (boot vectors)

**Commande QEMU:**
```bash
qemu-system-arm -M calypso \
  -cpu arm946 \
  -kernel loader.highram.elf \
  -serial pty \
  -monitor stdio \
  -nographic
```

## Améliorations détaillées

### 1. Gestion des périphériques QOM

**Avant (calypso_high.c):**
```c
DeviceState *dev = qdev_new(TYPE_CALYPSO_TIMER);
sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
// Device orphelin, pas de gestion du cycle de vie
```

**Après (calypso_soc.c):**
```c
object_initialize_child(OBJECT(dev), "timer1", &s->timer1, TYPE_CALYPSO_TIMER);
sysbus_realize(SYS_BUS_DEVICE(&s->timer1), &err);
// Device enfant, destruction automatique, meilleure gestion
```

### 2. Connexion UART/Serial

**Avant:** Connexion hardcodée dans la machine

**Après:** 
```c
// Dans calypso_mb.c
Chardev *chr = serial_hd(0);
if (chr) {
    qdev_prop_set_chr(DEVICE(&s->soc.uart_irda), "chardev", chr);
}
```
Permet de configurer via ligne de commande :
```bash
-serial pty                    # Pseudo-terminal
-serial tcp::4321,server       # TCP server
-serial stdio                  # Entrée/sortie standard
```

### 3. Configuration TRX flexible

**Avant:** Port hardcodé à 4729

**Après:**
```c
// Machine peut configurer le port
qdev_prop_set_int32(DEVICE(&soc), "trx-port", 5000);

// Ou désactiver complètement
qdev_prop_set_bit(DEVICE(&soc), "enable-trx", false);
```

### 4. Gestion de la mémoire

**Organisation:**
```
0x00000000 - 0x0001FFFF : Alias RAM interne (128K, boot vectors)
0x00800000 - 0x0083FFFF : RAM interne (256K)
0x01000000 - 0x017FFFFF : RAM externe (8M)
0x02000000 - 0x023FFFFF : Flash NOR (4M)
0xFFFF0000 - 0xFFFFFFFF : High vectors (alias RAM interne)
```

**Aliases pour boot ARM:**
- `ram_alias0` : Permet au CPU de booter depuis 0x00000000
- `high_vectors` : Support des high vectors (0xFFFF0000)

### 5. Chargement du firmware

**Support multiple formats:**
```c
// 1. ELF avec point d'entrée automatique
load_elf(filename, ..., &entry, ...);

// 2. Raw binary
load_image_targphys(filename, CALYPSO_XRAM_BASE, size);

// 3. Sans firmware (boot ROM)
// CPU démarre au reset vector
```

## Intégration dans QEMU

### Fichiers à ajouter

```
hw/arm/calypso/
├── calypso_mb.c          # ✅ NOUVEAU - Machine complète
├── calypso_soc.c         # ✅ NOUVEAU - SoC complet
├── calypso_soc.h         # ✅ NOUVEAU - Header SoC
├── calypso_trx.c         # ✅ CORRIGÉ - TRX bridge
├── calypso_trx.h
└── meson.build           # À créer

hw/intc/
├── calypso_inth.c
└── calypso_inth.h

hw/timer/
├── calypso_timer.c
└── calypso_timer.h

hw/char/
├── calypso_uart.c
└── calypso_uart.h

hw/ssi/
├── calypso_spi.c
└── calypso_spi.h
```

### meson.build pour hw/arm/calypso/

```meson
# Calypso machine and SoC
arm_softmmu_ss.add(when: 'CONFIG_CALYPSO', if_true: files(
  'calypso_mb.c',
  'calypso_soc.c',
  'calypso_trx.c',
))
```

### meson.build pour chaque périphérique

**hw/intc/meson.build:**
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_inth.c'))
```

**hw/timer/meson.build:**
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_timer.c'))
```

**hw/char/meson.build:**
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_uart.c'))
```

**hw/ssi/meson.build:**
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_spi.c'))
```

### Kconfig

**hw/arm/Kconfig:**
```kconfig
config CALYPSO
    bool "TI Calypso SoC (ARM946E-S based GSM baseband)"
    default y if ARM
    depends on ARM
    select PFLASH_CFI01
```

## Tests et validation

### 1. Compilation
```bash
cd ~/qemu/build
ninja

# Vérifier les warnings
ninja 2>&1 | grep -i calypso
```

### 2. Vérifier la machine
```bash
./qemu-system-arm -M help | grep calypso
# Devrait afficher:
# calypso              Calypso SoC development board (modular architecture)
```

### 3. Test de boot
```bash
./qemu-system-arm \
  -M calypso \
  -cpu arm946 \
  -kernel loader.highram.elf \
  -serial stdio \
  -monitor none \
  -nographic \
  -d guest_errors

# Devrait afficher:
# Calypso firmware loaded:
#   File:  loader.highram.elf
#   Entry: 0x01000000
#   Size:  XXXXX bytes
# 
# Calypso Machine Configuration:
#   CPU:          ARM946E-S
#   Internal RAM: 256 KiB @ 0x00800000
#   ...
```

### 4. Test avec osmocon
```bash
# Terminal 1 : QEMU
./qemu-system-arm -M calypso -cpu arm946 \
  -serial pty -monitor stdio -nographic

# Note le device pty (ex: /dev/pts/5)

# Terminal 2 : osmocon
osmocon -p /dev/pts/5 -m c123xor loader.highram.bin
```

### 5. Test TRX/BTS
```bash
# Terminal 1 : osmo-bts-trx
osmo-bts-trx -i 127.0.0.1

# Terminal 2 : QEMU Calypso
./qemu-system-arm -M calypso ...
```

## Debugging

### Activer les logs QEMU
```bash
./qemu-system-arm -M calypso ... \
  -d guest_errors,int \
  -D qemu.log
```

### Activer les logs TRX
Dans `calypso_trx.c`:
```c
#define TRX_DEBUG_DSP    1
#define TRX_DEBUG_SYNC   1
```

### GDB debugging
```bash
# QEMU
./qemu-system-arm -M calypso -kernel fw.elf -s -S

# GDB
arm-none-eabi-gdb fw.elf
(gdb) target remote :1234
(gdb) b main
(gdb) c
```

## Migration depuis calypso_high.c

Si vous utilisez actuellement `calypso_high.c` :

### Étape 1 : Compiler les deux versions
```meson
# Dans meson.build, ajouter temporairement les deux
arm_softmmu_ss.add(when: 'CONFIG_CALYPSO', if_true: files(
  'calypso_high.c',      # Ancienne machine
  'calypso_mb.c',        # Nouvelle machine
  'calypso_soc.c',
  'calypso_trx.c',
))
```

### Étape 2 : Tester côte à côte
```bash
# Ancienne
./qemu-system-arm -M calypso-high -kernel fw.elf

# Nouvelle
./qemu-system-arm -M calypso -kernel fw.elf
```

### Étape 3 : Vérifier l'équivalence
Les deux devraient se comporter de manière identique.

### Étape 4 : Supprimer l'ancienne
Une fois validé, supprimer `calypso_high.c` du build.

## Avantages de la nouvelle architecture

### 1. Maintenabilité
- Code séparé en fichiers logiques
- Chaque périphérique est indépendant
- Modifications isolées

### 2. Réutilisabilité
- Le SoC peut être utilisé dans d'autres machines
- Les périphériques sont génériques

### 3. Testabilité
- Tests unitaires par périphérique
- Tests d'intégration au niveau SoC
- Tests système au niveau machine

### 4. Extensibilité
- Facile d'ajouter de nouveaux périphériques
- Facile de créer des variantes de machines
- Support de plusieurs boards avec le même SoC

### 5. Performances
- Pas de différence de performance
- Optimisations compilateur identiques
- Même overhead qu'avant

## Conclusion

Cette architecture modulaire représente une amélioration significative par rapport à l'approche monolithique. Elle suit les meilleures pratiques QEMU et facilite grandement la maintenance et l'évolution du code.

**Recommandation:** Utiliser `calypso_mb.c` + `calypso_soc.c` pour tout nouveau développement.
