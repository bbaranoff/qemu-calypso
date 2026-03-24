# Configuration de compilation pour Calypso dans QEMU

## Structure des fichiers recommandée

### Option 1 : Tout dans hw/arm/calypso/
```
hw/arm/calypso/
├── calypso_high.c           # Machine principale
├── calypso_trx.c            # DSP/TPU/TRX bridge (✅ corrigé)
├── calypso_inth.c           # Interrupt controller
├── calypso_timer.c          # Timer
├── calypso_uart.c           # UART
├── calypso_spi.c            # SPI + TWL3025
├── calypso_socket.c         # Socket (optionnel)
│
├── calypso_trx.h
├── calypso_inth.h
├── calypso_timer.h
├── calypso_uart.h
├── calypso_spi.h
└── calypso_socket.h
```

### Option 2 : Séparés par catégorie (plus propre)
```
hw/arm/
└── calypso_high.c           # Machine

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

hw/arm/calypso/             # Spécifiques Calypso
├── calypso_trx.c
└── calypso_trx.h
```

## Fichier meson.build

### Pour hw/arm/calypso/meson.build

```meson
# Calypso SoC emulation
arm_softmmu_ss.add(when: 'CONFIG_CALYPSO', if_true: files(
  'calypso_high.c',
  'calypso_trx.c',
  'calypso_inth.c',
  'calypso_timer.c',
  'calypso_uart.c',
  'calypso_spi.c',
))
```

### Pour hw/arm/Kconfig

```kconfig
config CALYPSO
    bool "TI Calypso SoC (ARM946E-S based GSM baseband)"
    depends on ARM
    select PFLASH_CFI01
    # Ajoutez d'autres dépendances si nécessaire
```

### Pour default-configs/devices/arm-softmmu.mak

Ajouter :
```
CONFIG_CALYPSO=y
```

## Commandes de compilation

```bash
# 1. Configuration initiale (si pas déjà fait)
cd ~/qemu
mkdir -p build
cd build
../configure --target-list=arm-softmmu --enable-debug

# 2. Recompiler après modifications
cd ~/qemu/build
ninja

# 3. Vérifier que la machine est disponible
./qemu-system-arm -M help | grep calypso

# Devrait afficher:
# calypso-high         Calypso SoC (highram) with INTH, timers, UART, SPI/ABB, TRX
# calypso-min          Calypso SoC minimal machine (si inclus)
```

## Test de base

```bash
# Lancer QEMU avec la machine Calypso
./qemu-system-arm \
    -M calypso-high \
    -cpu arm946 \
    -kernel /path/to/loader.highram.elf \
    -serial pty \
    -monitor stdio \
    -nographic \
    -s -S

# Dans un autre terminal, se connecter avec GDB
arm-none-eabi-gdb /path/to/loader.highram.elf
(gdb) target remote :1234
(gdb) continue
```

## Vérification de la compilation

### Erreurs courantes et solutions

1. **"undefined reference to..."**
   - Vérifier que tous les fichiers sont dans meson.build
   - Vérifier que les includes sont corrects

2. **"unknown type name..."**
   - Ajouter les includes manquants en haut du fichier
   - Vérifier l'ordre des includes (qemu/osdep.h DOIT être premier)

3. **"implicit declaration of function..."**
   - Ajouter le header qui déclare cette fonction
   - Vérifier que la fonction est bien exportée (pas static)

4. **Warnings -Wshadow**
   - Renommer les variables locales qui masquent les paramètres
   - Utiliser des noms différents pour éviter les conflits

## Ordre des includes recommandé

```c
/* TOUJOURS en premier */
#include "qemu/osdep.h"

/* Headers QEMU généraux */
#include "qapi/error.h"
#include "qemu/timer.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"
#include "hw/irq.h"

/* Headers système si nécessaire */
#include <sys/socket.h>
#include <netinet/in.h>

/* Headers locaux en dernier */
#include "calypso_trx.h"
```

## Optimisations de compilation

### Pour le développement (debug)
```bash
../configure --target-list=arm-softmmu --enable-debug --disable-werror
```

### Pour la production
```bash
../configure --target-list=arm-softmmu --enable-strip
```

### Options utiles
```bash
--extra-cflags="-DDEBUG_CALYPSO"  # Activer les logs debug
--enable-sanitizers               # Détecter les bugs mémoire
--enable-trace-backends=log       # Activer le tracing
```

## Fichiers à exclure de la compilation

Ces fichiers ne doivent PAS être compilés (à moins de les compléter) :

- ❌ `calypso_mb.c` - Incomplet, redondant avec calypso_high.c
- ❌ `calypso_soc.c` - Incomplet, redondant avec calypso_high.c
- ⚠️ `calypso_socket.c` - À remplacer par calypso_socket_improved.c
- ⚠️ `calypso.c` - Machine minimale, optionnelle

## Vérification finale

Après compilation réussie, vérifier :

```bash
# 1. La machine est listée
./qemu-system-arm -M help | grep calypso

# 2. Les régions mémoire sont correctes
./qemu-system-arm -M calypso-high -d guest_errors 2>&1 | grep "memory"

# 3. Pas d'erreurs au démarrage
./qemu-system-arm -M calypso-high -kernel test.elf 2>&1 | grep -i error
```

## Support TRX UDP

Pour activer le bridge TRX (communication avec osmo-bts-trx):

```bash
# Dans calypso_high.c, la fonction init appelle déjà :
calypso_trx_init(sysmem, irqs, 4729);

# Le port 4729 est le port par défaut pour TRXD
# Modifier si nécessaire dans l'appel
```

Ensuite, lancer osmo-bts-trx sur le même port :

```bash
osmo-bts-trx -i 127.0.0.1 -p 4729
```

## Logs de débogage

Pour activer les logs détaillés :

```c
// Dans calypso_trx.c, ligne ~82-87
#define TRX_DEBUG_DSP    1  // Logs DSP RAM accesses
#define TRX_DEBUG_TPU    1  // Logs TPU operations
#define TRX_DEBUG_TSP    1  // Logs TSP operations
#define TRX_DEBUG_SYNC   1  // Logs FCCH/SCH sync (recommandé)
```

Recompiler après modification.
