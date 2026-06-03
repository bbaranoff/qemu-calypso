#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# grgsm_relay_decode.py — décodeur gr-gsm sur l'I/Q CONTINU relayée (mode
# full-grgsm). Lit l'I/Q du BTS relayée par calypso-ipc-device (UDP 5810,
# fc32), la passe dans le receiver gr-gsm complet (gsm.receiver : FCCH/SCH
# sync + démod GMSK), décode le BCCH/CCCH, et envoie le L2 en GSMTAP au
# listener du shunt (4730) → feed_si → a_cd → le mobile (sur osmocon) campe.
#
# C'est un DÉCODEUR (pas un transceiver) : pas besoin de trxcon. La chaîne
# est celle de grgsm_decode mais avec une source UDP au lieu d'un fichier.
#
#   udp_source(5810, fc32) → lpf → gsm.receiver(osr) → C0 bursts →
#       gsm_bcch_ccch_demapper → control_channels_decoder → GSMTAP(4730)
#
# osr DOIT matcher le SPS d'osmo-trx (tx-sps=1 → osr=1). Si la sync n'accroche
# pas à osr=1, passer osmo-trx en SPS=4 et CALYPSO_TRX_OSR=4.
import os, sys

from gnuradio import gr, network, filter
from gnuradio.filter import firdes
from gnuradio.fft import window
from gnuradio import gsm

# OSR = oversampling vu par gsm.receiver. Il a besoin de ~4 pour locker la
# FCCH/SCH et la récupération de timing GMSK. L'I/Q relayée est à 1 SPS
# (osmo-trx tx-sps=1) → on RESAMPLE 1→OSR dans le flowgraph (pas de chirurgie
# device, pas de SPS=4 côté osmo-trx).
OSR        = int(os.environ.get("CALYPSO_TRX_OSR", "4"))
# --- Correction de rate "produit en croix" 4908 (gated, defaut no-op) -------
# Le firmware traite une frame EFFECTIVE de 4908 qbits (header magic-23) au lieu
# de 5000 nominal. Rate nominal grgsm = 26e6/24 = 1083333. Rate corrige =
# 5e9/4908 = 1018744 (= 1083333 x (24x5000)/(26x4908) = x0.9404). Pour matcher,
# on resample le flux par RATIO = 1083333/1018744 = 1.0634 (input/output).
# CALYPSO_GRGSM_RESAMP_RATIO=1.0634 active la correction ; defaut 1.0 = no-op.
RESAMP_RATIO = float(os.environ.get("CALYPSO_GRGSM_RESAMP_RATIO", "1.0"))
# DC-blocker (FFT cfile : grosse composante DC parasite l'accroche FCCH). Defaut ON.
DCBLOCK = os.environ.get("CALYPSO_GRGSM_DCBLOCK", "1") == "1"
IN_SPS     = int(os.environ.get("CALYPSO_TRX_IN_SPS", "1"))   # SPS du relais (osmo-trx)
SYM_RATE   = 1625000.0 / 6.0                       # 270833.33 Hz
SAMP_RATE  = SYM_RATE * OSR                         # rate après resampling
RX_PORT    = int(os.environ.get("CALYPSO_TRX_IQ_RX_PORT", "5810"))
GSMTAP_HOST = os.environ.get("GSMTAP_HOST", "127.0.0.1")
GSMTAP_PORT = int(os.environ.get("CALYPSO_SHUNT_GSMTAP_PORT", "4730"))
TS         = int(os.environ.get("TS", "0"))

tb = gr.top_block("grgsm relay decode")

# Source : I/Q continu relayé (fc32, 625 samples/datagram = 5000 o). src_zeros
# → flux continu si pas de data (le receiver ne stalle pas).
src = network.udp_source(gr.sizeof_gr_complex, 1, RX_PORT,
                         0, 5000, False, True, False)

# DC-blocker complexe (retire l'offset DC qui domine la FFT et noie le FCCH)
dcb = None
if DCBLOCK:
    try:
        dcb = filter.dc_blocker_cc(32, True)
        print("[grgsm-relay-decode] DC-blocker ON (D=32)", flush=True)
    except AttributeError:
        dcb = None

# Resampling 1→OSR (IN_SPS→OSR) : donne à gsm.receiver les ~4 SPS qu'il attend.
resamp = filter.rational_resampler_ccc(interpolation=OSR, decimation=IN_SPS)

lpf = filter.fir_filter_ccf(1, firdes.low_pass(
    1, SAMP_RATE, 125e3, 5e3, window.WIN_HAMMING, 6.76))

# REFRAME 4908 -> 5000 : le device livre une frame EFFECTIVE de 4908 qbits,
# grgsm/gsm.receiver attend 5000. On interpole par 5000/4908 = 1250/1227 (SENS
# 4908->5000, l'inverse de l'essai precedent). Defaut ON (CALYPSO_GRGSM_REFRAME=0
# pour couper). RESAMP_RATIO (mmse) reste dispo pour sweep manuel.
REFRAME = os.environ.get("CALYPSO_GRGSM_REFRAME", "1") == "1"
corr = None
if REFRAME:
    corr = filter.rational_resampler_ccc(interpolation=1250, decimation=1227)
    print("[grgsm-relay-decode] REFRAME 4908->5000 (resample 1250/1227)", flush=True)
elif abs(RESAMP_RATIO - 1.0) > 1e-9:
    try:    corr = filter.mmse_resampler_cc(0.0, RESAMP_RATIO)
    except AttributeError: corr = filter.fractional_resampler_cc(0.0, RESAMP_RATIO)
    print("[grgsm-relay-decode] RESAMP_RATIO=%.5f" % RESAMP_RATIO, flush=True)

rx  = gsm.receiver(OSR, [0], [])              # cell_allocation [0], tseq []
dm  = gsm.gsm_bcch_ccch_demapper(TS)
dec = gsm.control_channels_decoder()
snk = network.socket_pdu("UDP_CLIENT", GSMTAP_HOST, str(GSMTAP_PORT), 10000)

if dcb is not None:
    tb.connect((src, 0), (dcb, 0)); tb.connect((dcb, 0), (resamp, 0))
else:
    tb.connect((src, 0), (resamp, 0))
tb.connect((resamp, 0), (lpf, 0))
if corr is not None:
    tb.connect((lpf, 0), (corr, 0))
    tb.connect((corr, 0), (rx, 0))
else:
    tb.connect((lpf, 0), (rx, 0))
tb.msg_connect((rx, 'C0'), (dm, 'bursts'))
tb.msg_connect((dm, 'bursts'), (dec, 'bursts'))
tb.msg_connect((dec, 'msgs'), (snk, 'pdus'))

print("[grgsm-relay-decode] I/Q udp:%d (osr=%d, %.0f Hz) -> GSMTAP %s:%d -> feed_si"
      % (RX_PORT, OSR, SAMP_RATE, GSMTAP_HOST, GSMTAP_PORT), flush=True)

tb.start()
try:
    tb.wait()
except KeyboardInterrupt:
    tb.stop(); tb.wait()
