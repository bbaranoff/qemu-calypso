# Calypso QEMU - Guide de Démarrage Rapide

## Installation en 5 étapes

### Étape 1 : Préparer les fichiers

Copier tous les fichiers fournis dans votre arborescence QEMU :

```bash
# Créer la structure de répertoires
cd ~/qemu
mkdir -p hw/arm/calypso
mkdir -p include/hw/arm/calypso

# Machine et SoC
cp calypso_mb.c hw/arm/calypso/
cp calypso_soc.c hw/arm/calypso/
cp calypso_soc.h include/hw/arm/calypso/
cp calypso_trx.c hw/arm/calypso/
cp calypso_trx.h include/hw/arm/calypso/
cp meson.build hw/arm/calypso/

# Périphériques (si pas déjà présents)
cp calypso_inth.c hw/intc/
cp calypso_inth.h include/hw/arm/calypso/

cp calypso_timer.c hw/timer/
cp calypso_timer.h include/hw/arm/calypso/

cp calypso_uart.c hw/char/
cp calypso_uart.h include/hw/arm/calypso/

cp calypso_spi.c hw/ssi/
cp calypso_spi.h include/hw/arm/calypso/
```

### Étape 2 : Mettre à jour meson.build

**hw/arm/calypso/meson.build** (déjà fourni)

**hw/intc/meson.build** - Ajouter :
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_inth.c'))
```

**hw/timer/meson.build** - Ajouter :
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_timer.c'))
```

**hw/char/meson.build** - Ajouter :
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_uart.c'))
```

**hw/ssi/meson.build** - Ajouter :
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_spi.c'))
```

### Étape 3 : Configurer Kconfig

**hw/arm/Kconfig** - Ajouter dans la section ARM :
```kconfig
config CALYPSO
    bool "TI Calypso SoC (ARM946E-S based GSM baseband)"
    default y if ARM
    depends on ARM
    select PFLASH_CFI01
    select SERIAL
```

**default-configs/devices/arm-softmmu.mak** - Ajouter :
```
CONFIG_CALYPSO=y
```

### Étape 4 : Compiler

```bash
cd ~/qemu/build

# Si premier build
../configure --target-list=arm-softmmu --enable-debug

# Compiler
ninja

# Vérifier la compilation
echo "Build completed with status: $?"
```

### Étape 5 : Tester

```bash
# Vérifier que la machine est disponible
./qemu-system-arm -M help | grep calypso

# Test sans firmware (devrait afficher config)
./qemu-system-arm -M calypso -cpu arm946 -nographic -monitor none

# Test avec firmware
./qemu-system-arm -M calypso -cpu arm946 \
  -kernel /path/to/loader.highram.elf \
  -serial stdio \
  -nographic
```

## Utilisation avec OsmocomBB

### Scénario 1 : Test de base avec osmocon

```bash
# Terminal 1 : QEMU avec pseudo-terminal
./qemu-system-arm -M calypso -cpu arm946 \
  -serial pty \
  -monitor stdio \
  -nographic

# Noter le device PTY (exemple: /dev/pts/5)

# Terminal 2 : osmocon
osmocon -p /dev/pts/5 -m c123xor loader.highram.bin
```

### Scénario 2 : Test avec osmo-bts-trx (GSM stack complet)

```bash
# Terminal 1 : osmo-bts-trx
osmo-bts-trx -i 127.0.0.1

# Terminal 2 : QEMU Calypso
./qemu-system-arm -M calypso -cpu arm946 \
  -kernel osmocom-bb-trx.elf \
  -serial stdio \
  -nographic

# Les deux devraient communiquer via UDP port 4729
```

### Scénario 3 : Debugging avec GDB

```bash
# Terminal 1 : QEMU en mode debug
./qemu-system-arm -M calypso -cpu arm946 \
  -kernel loader.highram.elf \
  -serial stdio \
  -s -S \
  -nographic

# Terminal 2 : GDB
arm-none-eabi-gdb loader.highram.elf
(gdb) target remote :1234
(gdb) b main
(gdb) c
```

## Résolution des problèmes courants

### Problème 1 : "unknown machine type 'calypso'"

**Cause:** Machine non compilée

**Solution:**
```bash
# Vérifier que CONFIG_CALYPSO=y dans default-configs/devices/arm-softmmu.mak
grep CALYPSO default-configs/devices/arm-softmmu.mak

# Recompiler complètement
cd build
rm -rf *
../configure --target-list=arm-softmmu
ninja
```

### Problème 2 : Erreurs de compilation "undefined reference"

**Cause:** Fichiers manquants dans meson.build

**Solution:**
```bash
# Vérifier que tous les .c sont listés
grep -r "calypso" hw/*/meson.build

# Nettoyer et recompiler
ninja clean
ninja
```

### Problème 3 : "Could not load firmware"

**Cause:** Firmware incompatible ou chemin incorrect

**Solution:**
```bash
# Vérifier que le fichier existe
ls -lh /path/to/firmware.elf

# Tester avec le chemin absolu
./qemu-system-arm -M calypso -cpu arm946 \
  -kernel $(readlink -f firmware.elf) \
  -nographic

# Vérifier le format ELF
file firmware.elf
# Devrait afficher: ELF 32-bit LSB executable, ARM, ...
```

### Problème 4 : Pas de sortie sur serial

**Cause:** Configuration serial incorrecte

**Solution:**
```bash
# Essayer avec stdio
./qemu-system-arm -M calypso -cpu arm946 \
  -kernel fw.elf \
  -serial stdio \
  -nographic

# Ou avec fichier log
./qemu-system-arm -M calypso -cpu arm946 \
  -kernel fw.elf \
  -serial file:uart.log \
  -nographic

# Puis surveiller le fichier
tail -f uart.log
```

### Problème 5 : TRX ne communique pas

**Cause:** Port UDP bloqué ou incorrect

**Solution:**
```bash
# Vérifier que le port est libre
sudo netstat -ulnp | grep 4729

# Tester avec un autre port
./qemu-system-arm -M calypso -cpu arm946 \
  -device calypso-soc,trx-port=5000 \
  ...

# Vérifier les logs TRX
# Activer TRX_DEBUG_SYNC=1 dans calypso_trx.c et recompiler
```

## Options avancées

### Désactiver TRX (pour tests sans GSM)
```bash
./qemu-system-arm -M calypso -cpu arm946 \
  -global calypso-soc.enable-trx=false \
  -kernel fw.elf
```

### Changer le port TRX
```bash
./qemu-system-arm -M calypso -cpu arm946 \
  -global calypso-soc.trx-port=5000 \
  -kernel fw.elf
```

### Logs détaillés
```bash
./qemu-system-arm -M calypso -cpu arm946 \
  -kernel fw.elf \
  -d guest_errors,int \
  -D qemu.log
  
# Surveiller les logs
tail -f qemu.log
```

### Traçage mémoire
```bash
./qemu-system-arm -M calypso -cpu arm946 \
  -kernel fw.elf \
  -trace "memory_*" \
  -D trace.log
```

## Scripts utiles

### Script de build automatique

```bash
#!/bin/bash
# build-calypso.sh

set -e

cd ~/qemu/build

echo "=== Cleaning ==="
ninja clean

echo "=== Building ==="
ninja 2>&1 | tee build.log

echo "=== Checking for errors ==="
if grep -i "error" build.log; then
    echo "Build FAILED"
    exit 1
fi

echo "=== Checking for warnings ==="
grep -i "warning.*calypso" build.log || echo "No warnings"

echo "=== Verifying machine ==="
./qemu-system-arm -M help | grep calypso

echo "=== Build SUCCESS ==="
```

### Script de test rapide

```bash
#!/bin/bash
# quick-test.sh

FW="${1:-loader.highram.elf}"

./qemu-system-arm -M calypso -cpu arm946 \
  -kernel "$FW" \
  -serial stdio \
  -monitor none \
  -nographic
```

## Prochaines étapes

1. **Lire ARCHITECTURE_AMELIOREE.md** pour comprendre la structure
2. **Lancer le script de test** : `./test-calypso.sh all`
3. **Tester avec votre firmware** OsmocomBB
4. **Activer les logs debug** si problèmes de sync GSM
5. **Consulter BUILD_CONFIGURATION.md** pour options avancées

## Support

En cas de problème :
1. Vérifier les logs : `-d guest_errors -D qemu.log`
2. Activer TRX_DEBUG_SYNC dans calypso_trx.c
3. Tester avec la machine minimale `calypso-min` (si disponible)
4. Comparer avec la version originale `calypso-high`

## Ressources

- **ARCHITECTURE_AMELIOREE.md** - Architecture détaillée
- **BUILD_CONFIGURATION.md** - Configuration de build
- **CORRECTIONS_ET_RECOMMANDATIONS.md** - Problèmes corrigés
- **test-calypso.sh** - Script de test automatisé
