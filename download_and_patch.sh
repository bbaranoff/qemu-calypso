#!/bin/bash
# download_and_patch.sh — Download QEMU 9.2.4, patch with Calypso, build
#
# Usage:
#   ./download_and_patch.sh [build_dir]
#
# Default build_dir: ./qemu-calypso-build
set -euo pipefail

QEMU_VERSION="9.2.4"
QEMU_URL="https://download.qemu.org/qemu-${QEMU_VERSION}.tar.xz"
BUILD_DIR="${1:-./qemu-calypso-build}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

info()  { echo -e "${CYAN}[*]${NC} $1"; }
ok()    { echo -e "${GREEN}[+]${NC} $1"; }
fail()  { echo -e "${RED}[-]${NC} $1"; exit 1; }
header(){ echo -e "\n${BOLD}=== $1 ===${NC}"; }

# ============================================================
header "Calypso QEMU Builder"
echo "  QEMU version : $QEMU_VERSION"
echo "  Build dir    : $BUILD_DIR"
echo "  Patches from : $SCRIPT_DIR"
echo ""

# ---- Prerequisites ----
header "1. Checking prerequisites"
for cmd in wget tar ninja gcc python3 pkg-config; do
    command -v $cmd >/dev/null || fail "Missing: $cmd"
done
pkg-config --exists glib-2.0 || fail "Missing: libglib2.0-dev"
pkg-config --exists pixman-1 || fail "Missing: libpixman-1-dev"
ok "All prerequisites found"

# ---- Download ----
header "2. Downloading QEMU ${QEMU_VERSION}"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

TARBALL="qemu-${QEMU_VERSION}.tar.xz"
if [ -f "$TARBALL" ]; then
    info "Tarball already exists, skipping download"
else
    info "Downloading $QEMU_URL..."
    wget -q --show-progress "$QEMU_URL" -O "$TARBALL"
fi
ok "Tarball ready"

# ---- Extract ----
header "3. Extracting"
QEMU_SRC="qemu-${QEMU_VERSION}"
if [ -d "$QEMU_SRC" ]; then
    info "Source directory exists, removing..."
    rm -rf "$QEMU_SRC"
fi
tar xf "$TARBALL"
ok "Extracted to $QEMU_SRC"

cd "$QEMU_SRC"

# ============================================================
header "4. Patching QEMU with Calypso support"
# ============================================================

# ---- 4a. Calypso source files ----
info "Copying Calypso source files..."

# hw/arm/calypso/
mkdir -p hw/arm/calypso
cp "$SCRIPT_DIR"/hw/arm/calypso/calypso_mb.c   hw/arm/calypso/
cp "$SCRIPT_DIR"/hw/arm/calypso/calypso_soc.c  hw/arm/calypso/
cp "$SCRIPT_DIR"/hw/arm/calypso/calypso_trx.c  hw/arm/calypso/
cp "$SCRIPT_DIR"/hw/arm/calypso/meson.build     hw/arm/calypso/
cp "$SCRIPT_DIR"/hw/arm/calypso/Kconfig         hw/arm/calypso/

# hw/char/
cp "$SCRIPT_DIR"/hw/char/calypso_uart.c hw/char/

# hw/intc/
cp "$SCRIPT_DIR"/hw/intc/calypso_inth.c hw/intc/

# hw/timer/
cp "$SCRIPT_DIR"/hw/timer/calypso_timer.c hw/timer/

# hw/ssi/
cp "$SCRIPT_DIR"/hw/ssi/calypso_spi.c hw/ssi/
cp "$SCRIPT_DIR"/hw/ssi/calypso_i2c.c hw/ssi/

# include/
mkdir -p include/hw/arm/calypso
cp "$SCRIPT_DIR"/include/hw/arm/calypso/*.h include/hw/arm/calypso/

ok "Source files copied"

# ---- 4b. Kconfig ----
info "Patching hw/arm/Kconfig..."
if ! grep -q "CONFIG_CALYPSO" hw/arm/Kconfig 2>/dev/null; then
    cat >> hw/arm/Kconfig <<'EOF'

config CALYPSO
    bool
    default y
    depends on ARM
    select PFLASH_CFI01
EOF
    ok "Kconfig patched"
else
    info "Kconfig already has CALYPSO"
fi

# ---- 4c. meson.build files ----
info "Patching meson.build files..."

# hw/arm/meson.build — add subdir('calypso') before hw_arch line
if ! grep -q "calypso" hw/arm/meson.build; then
    sed -i "/^hw_arch += {'arm': arm_ss}/i subdir('calypso')" hw/arm/meson.build
    ok "hw/arm/meson.build patched"
else
    info "hw/arm/meson.build already patched"
fi

# hw/char/meson.build
if ! grep -q "calypso" hw/char/meson.build; then
    echo "system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_uart.c'))" >> hw/char/meson.build
    ok "hw/char/meson.build patched"
else
    info "hw/char/meson.build already patched"
fi

# hw/intc/meson.build
if ! grep -q "calypso" hw/intc/meson.build; then
    echo "system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_inth.c'))" >> hw/intc/meson.build
    ok "hw/intc/meson.build patched"
else
    info "hw/intc/meson.build already patched"
fi

# hw/timer/meson.build
if ! grep -q "calypso" hw/timer/meson.build; then
    echo "system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_timer.c'))" >> hw/timer/meson.build
    ok "hw/timer/meson.build patched"
else
    info "hw/timer/meson.build already patched"
fi

# hw/ssi/meson.build
if ! grep -q "calypso" hw/ssi/meson.build; then
    cat >> hw/ssi/meson.build <<'EOF'
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_spi.c'))
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_i2c.c'))
EOF
    ok "hw/ssi/meson.build patched"
else
    info "hw/ssi/meson.build already patched"
fi

# ---- 4d. default.mak ----
info "Patching configs/devices/arm-softmmu/default.mak..."
if ! grep -q "CONFIG_CALYPSO" configs/devices/arm-softmmu/default.mak; then
    sed -i '1a CONFIG_CALYPSO=y' configs/devices/arm-softmmu/default.mak
    ok "default.mak patched"
else
    info "default.mak already patched"
fi

# ---- 4e. Scripts ----
info "Copying scripts..."
cp "$SCRIPT_DIR"/l1ctl_bridge.py .
cp "$SCRIPT_DIR"/run-mobile.sh .
cp "$SCRIPT_DIR"/run-layer1.sh . 2>/dev/null || true
cp "$SCRIPT_DIR"/test-mobile.sh . 2>/dev/null || true
chmod +x run-mobile.sh run-layer1.sh test-mobile.sh l1ctl_bridge.py 2>/dev/null || true

# ---- 4f. Documentation ----
mkdir -p docs
cp "$SCRIPT_DIR"/docs/CALYPSO-QEMU.md docs/ 2>/dev/null || true

ok "All patches applied"

# ============================================================
header "5. Configuring QEMU (arm-softmmu only)"
# ============================================================
mkdir -p build
cd build
../configure --target-list=arm-softmmu --disable-docs 2>&1 | tail -5
ok "Configuration done"

# ============================================================
header "6. Building"
# ============================================================
NPROC=$(nproc 2>/dev/null || echo 4)
info "Building with $NPROC jobs..."
ninja -j"$NPROC" 2>&1 | tail -5
ok "Build complete"

# ---- Verify ----
if ./qemu-system-arm -M help 2>/dev/null | grep -q calypso; then
    ok "Machine 'calypso' registered!"
else
    fail "Machine 'calypso' NOT found in build"
fi

# ============================================================
header "Done!"
# ============================================================
QEMU_BIN="$(pwd)/qemu-system-arm"
echo ""
echo -e "  ${GREEN}QEMU binary:${NC} $QEMU_BIN"
echo ""
echo -e "  ${BOLD}Quick start:${NC}"
echo "    cd $(dirname $QEMU_BIN)/.."
echo "    ./run-mobile.sh /path/to/layer1.highram.elf"
echo ""
echo -e "  ${BOLD}Manual:${NC}"
echo "    $QEMU_BIN -M calypso -cpu arm946 \\"
echo "      -serial pty -serial pty \\"
echo "      -monitor unix:/tmp/qemu-calypso-mon.sock,server,nowait \\"
echo "      -kernel /path/to/layer1.highram.elf"
echo ""
echo -e "  ${BOLD}Docs:${NC} docs/CALYPSO-QEMU.md"
echo ""
