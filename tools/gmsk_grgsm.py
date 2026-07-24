"""GMSK modulator via GNU Radio (gnuradio.digital.gmskmod_bc).

Remplace le custom soft_bits_to_gmsk_iq de bridge.py par une implémentation
spec-conforme. Si gnuradio absent, raise ImportError — bridge.py fallback
sur sa version custom.

GSM params : samples_per_sym=1 (= 1 sample/symbol, ~270.833 kHz baseband),
L=4 (pulse length), beta=0.3 (BT product = 0.3).

API :
  bits_to_iq_grgsm(soft_bits, scale=4096) -> bytes (int16 interleaved I,Q)
"""
import struct

from gnuradio import gr, blocks
from gnuradio.digital import gmskmod_bc


_TB = None  # top_block cache (reused across calls)
_SRC = None
_SINK = None


def _build_flowgraph():
    """Construit le flowgraph gmsk_mod réutilisable.

    vector_source (packed bytes) → gmskmod_bc → vector_sink (complex).
    Le source/sink sont updated par call (set_data + reset).
    """
    global _TB, _SRC, _SINK
    if _TB is not None:
        return
    _SRC = blocks.vector_source_b([], False)
    _MOD = gmskmod_bc(samples_per_sym=1, L=4, beta=0.3)
    _SINK = blocks.vector_sink_c()
    _TB = gr.top_block()
    _TB.connect(_SRC, _MOD, _SINK)


def bits_to_iq_grgsm(soft_bits_raw, scale=4096):
    """Convert TRXD soft bits → GMSK I/Q int16 baseband via gr-digital.

    soft_bits_raw : iterable of soft bits (0=strong 1, 255=strong 0,
                    threshold 128).
    scale         : multiplier int16 (default 4096 = même que custom).
    Retourne bytes (2N int16 interleaved I,Q,I,Q,...).
    """
    _build_flowgraph()
    # gmskmod_bc avec samples_per_sym=1 = 1 byte d'entrée = 1 symbole = 1
    # sample complexe out. Pas de bit-packing.
    #
    # Encoding gmskmod_bc :
    #   byte=0   → rotation 0       (NRZ -1, pas de tone)
    #   byte=1   → rotation +π/2    (NRZ +1, tone +fs/4)
    #
    # Notre convention soft bit (osmocom) :
    #   soft < 128 → bit décodé = 0 (FCCH spec : 0 bit → +π/2 tone)
    #   soft >= 128 → bit décodé = 1
    #
    # Donc invert : soft<128 → gmsk input 1 (tone), soft>=128 → 0.
    n_bits = len(soft_bits_raw)
    bits_for_mod = [1 if b < 128 else 0 for b in soft_bits_raw]

    _SRC.set_data(bits_for_mod)
    _SINK.reset()
    _TB.run()
    cx = _SINK.data()  # liste de complex (numpy complex64)

    # samples_per_sym=1 → out length = n_bits (pas de transient)
    cx = cx[:n_bits]

    out = bytearray()
    for c in cx:
        i_int = max(-32768, min(32767, int(c.real * scale)))
        q_int = max(-32768, min(32767, int(c.imag * scale)))
        out += struct.pack("<hh", i_int, q_int)
    return bytes(out)
