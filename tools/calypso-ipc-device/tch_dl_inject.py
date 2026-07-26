#!/usr/bin/env python3
# tch_dl_inject.py — injecteur de test JALON 1 du chemin DL TCH.
#
# Pousse des trames GSM-FR (33 o, codec 06.10 = celui de gapk) dans le sideband
# /dev/shm/calypso_tch_dl, au format lu par calypso_tch_dl_poll() du shunt :
#   seq@0 (u32 LE)  fn@4 (u32 LE)  fr[33]@8   (total 48 o, consume-once par seq)
#
# But : valider shunt -> a_dd_0 -> firmware L1CTL_TRAFFIC_IND -> L23/gapk -> ALSA,
# SANS dependre de gr-gsm. Si on entend le ton (web /audio ou HP) pendant la
# fenetre TCH d'un appel, la chaine DL + le packing BE=1 sont bons.
#
# Usage : tch_dl_inject.py [freq_hz] [path]
#   freq_hz : 0 = silence FR, sinon ton (defaut 600). path : defaut /dev/shm/calypso_tch_dl
import ctypes, struct, math, os, sys, time

FREQ = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
PATH = sys.argv[2] if len(sys.argv) > 2 else "/dev/shm/calypso_tch_dl"
RATE = 8000
SPF  = 160          # echantillons / trame FR (20 ms)

# --- vocoder FR (libgsm 06.10, standard 33 o ; meme codec que gapk gsmfr) ---
lib = ctypes.CDLL("libgsm.so.1")
lib.gsm_create.restype = ctypes.c_void_p
lib.gsm_encode.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_short),
                           ctypes.POINTER(ctypes.c_ubyte)]
h = lib.gsm_create()

# 160 ech de ton (600 Hz @ 8 kHz = 12 cycles pile -> phase-continue en boucle).
src = (ctypes.c_short * SPF)()
for i in range(SPF):
    src[i] = int(10000 * math.sin(2 * math.pi * FREQ * i / RATE)) if FREQ > 0 else 0
dst = (ctypes.c_ubyte * 33)()
lib.gsm_encode(h, src, dst)
fr = bytes(dst)
assert (fr[0] >> 4) == 0xD, "signature FR attendue 0xD, got 0x%x" % (fr[0] >> 4)
print("[inject] trame FR (%s) : %s" % ("silence" if FREQ == 0 else "%g Hz" % FREQ,
                                       fr.hex()), flush=True)

fd = os.open(PATH, os.O_RDWR | os.O_CREAT, 0o644)
os.ftruncate(fd, 48)
seq = 0
fn = 0
print("[inject] -> %s @50 Hz (Ctrl+C pour arreter)" % PATH, flush=True)
while True:
    seq += 1
    fn = (fn + 4) & 0xFFFFFFFF
    buf = struct.pack("<II", seq, fn) + fr + b"\x00" * (48 - 8 - 33)
    os.pwrite(fd, buf, 0)
    time.sleep(0.020)        # 1 trame / 20 ms = cadence bloc TCH/F
