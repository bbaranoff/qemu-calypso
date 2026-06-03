#!/usr/bin/env python3
# grgsm_fft_live.py — FFT EN DIRECT du cfile BSP (/tmp/dsp_iq.cfile, fc32).
# Le cfile = le lien grgsm<->BSP (mailing, comme ARM<->DSP mais DSP shunté).
# Affiche un spectre ASCII qui se rafraichit + marque le pic FCCH (+67708 Hz).
import numpy as np, time, os, sys
CFILE=os.environ.get("CFILE","/tmp/dsp_iq.cfile")
FS=float(os.environ.get("FS","270833"))     # 1 SPS BSP
N=int(os.environ.get("NFFT","4096"))
COLS=70
FCCH=1625000.0/24                            # +67708 Hz
while True:
    try: sz=os.path.getsize(CFILE)
    except OSError: sz=0
    os.system("clear")
    print("== FFT LIVE cfile %s (%d o, fs=%.0f, N=%d) ==  FCCH attendu %+.0f Hz"%(CFILE,sz,FS,N,FCCH))
    if sz < N*8:
        print("\n  cfile vide/petit -> relancer avec CALYPSO_RELAY_ALSO_BSP=1 (le BSP nourrit feed_iq)")
        time.sleep(1); continue
    with open(CFILE,"rb") as f:
        f.seek(max(0,(sz//8-N)*8)); iq=np.frombuffer(f.read(N*8),dtype=np.complex64)
    if len(iq)<N: time.sleep(0.5); continue
    sp=np.abs(np.fft.fftshift(np.fft.fft(iq*np.hanning(len(iq)))))
    fr=np.fft.fftshift(np.fft.fftfreq(len(iq),1/FS))
    pk=fr[np.argmax(sp)]; mag=np.abs(iq)
    print("  ampl moy=%.5f max=%.4f | PIC %+.0f Hz | dphi std=%.2f (GMSK~1.57)\n"%(
        mag.mean(),mag.max(),pk,np.std(np.angle(iq[1:]*np.conj(iq[:-1])))))
    # spectre ASCII : 24 lignes, bins regroupes
    B=24; step=len(sp)//B; spb=np.array([sp[i*step:(i+1)*step].max() for i in range(B)])
    frb=np.array([fr[i*step] for i in range(B)]); m=spb.max()+1e-9
    for i in range(B):
        bar=int(COLS*spb[i]/m); tag=" <-FCCH" if abs(frb[i]-FCCH)<FS/B else ""
        print("%+7.0f|%s%s"%(frb[i],"#"*bar,tag))
    time.sleep(1)
