#!/bin/bash
# build.sh — sync host→container, update build tag, build, sync back
set -euo pipefail

CONTAINER="trying"
HOST_DIR="/home/nirvana/qemu-src/hw/arm/calypso"
CONT_DIR="/root/qemu/hw/arm/calypso"
TRX="$HOST_DIR/calypso_trx.c"
TAG="BUILD $(date '+%Y-%m-%dT%H:%M:%S')"

# 1. Update build tag in calypso_trx.c
sed -i "s/BUILD [0-9T:_-]*/$(echo "$TAG" | sed 's/[&/]/\\&/g')/" "$TRX"
echo "$TAG"

# 2. Sync host → container
docker cp "$HOST_DIR/." "$CONTAINER:$CONT_DIR/"

# 3. Build
docker exec "$CONTAINER" bash -c 'cd /root/qemu/build && ninja' 2>&1 | tail -5

# 4. Verify md5
echo "--- md5 ---"
H_TRX=$(md5sum "$HOST_DIR/calypso_trx.c" | cut -d' ' -f1)
H_C54=$(md5sum "$HOST_DIR/calypso_c54x.c" | cut -d' ' -f1)
C_TRX=$(docker exec "$CONTAINER" md5sum "$CONT_DIR/calypso_trx.c" | cut -d' ' -f1)
C_C54=$(docker exec "$CONTAINER" md5sum "$CONT_DIR/calypso_c54x.c" | cut -d' ' -f1)

if [ "$H_TRX" = "$C_TRX" ] && [ "$H_C54" = "$C_C54" ]; then
    echo "OK synced"
else
    echo "MISMATCH!"
    echo "  trx: host=$H_TRX container=$C_TRX"
    echo "  c54: host=$H_C54 container=$C_C54"
    exit 1
fi
