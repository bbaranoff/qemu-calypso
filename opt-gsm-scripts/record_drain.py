#!/usr/bin/env python3
# record_drain.py — draine le FIFO live iq_record.fifo -> record.cfile en RING
# (128 MB, fseek wrap), HORS du hot-path qemu. C'est l'EXTERNALISATION du ring
# cfile que qemu ecrivait avant dans son hot-path DL (= ce qui causait les
# underruns). Ici qemu ne fait qu'un write() non-bloquant vers le FIFO ; ce
# process (independant) absorbe le flux et l'ecrit sur disque a son rythme.
# grgsm_decode -c relit record.cfile offline comme avant => SI preserve.
#
# Ouverture du FIFO en O_RDWR : ne bloque pas a l'ouverture ET ne voit jamais
# d'EOF (on garde nous-memes un write-end), donc qemu (O_WRONLY|O_NONBLOCK)
# trouve toujours un lecteur => son open reussit et le flux coule.
import os, sys

FIFO = os.environ.get("CALYPSO_RECORD_FIFO", "/tmp/iq_record.fifo")
OUT  = os.environ.get("CALYPSO_RECORD_FILE", "/tmp/record.cfile")
RING = int(os.environ.get("CALYPSO_RECORD_RING", str(128 << 20)))   # 128 MB

if not os.path.exists(FIFO):
    os.mkfifo(FIFO, 0o666)

fd = os.open(FIFO, os.O_RDWR)            # O_RDWR : pas de blocage / pas d'EOF
out = open(OUT, "wb", buffering=1 << 20)
w = 0
sys.stderr.write("[record-drain] %s -> %s (ring %d MB)\n" % (FIFO, OUT, RING >> 20))
sys.stderr.flush()
while True:
    b = os.read(fd, 1 << 16)            # 64 KB
    if not b:
        continue
    out.write(b)
    w += len(b)
    if w >= RING:                        # RING : rembobine (grgsm relit depuis 0)
        out.seek(0); w = 0
    out.flush()
