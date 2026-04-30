# MMAP_SI_FORMAT — Interface dynamique d'injection BCCH SI

> Spec v1 — 2026-04-30. Interface mmap shared file pour injection dynamique des
> System Information BCCH bursts dans `calypso_fbsb.c`, alimentée par tap RSL
> côté bridge / sniffer indépendant.

## Rationale

- **Avant** : SI3 hardcodé dans `calypso_fbsb.c::si3_blob[]`. Viole règle #1 "no stubs".
- **Après** : SI1/SI2/SI3/SI4/SI13 lus dynamiquement depuis `/dev/shm/calypso_si.bin`,
  alimenté par un sniffer RSL qui tap les `BCCH INFORMATION` messages osmo-bsc → osmo-bts.

## Architecture

```
osmo-bsc          osmo-bts
   │                 │
   │ RSL/IPA tcp:3003│
   ├────────────────►│
   │                 │
        ▲
        │ pcap sniff
        │
   scripts/rsl_si_tap.py (process séparé, single-writer)
        │
        │ mmap write
        ▼
   /dev/shm/calypso_si.bin (176 bytes)
        ▲
        │ mmap read
        │
   QEMU calypso_fbsb.c::on_dsp_task_change ALLC (single-reader)
        │
        ▼
   DATA_IND → ARM L1S → mobile L23
```

**Properties** :
- Single writer (rsl_si_tap.py) / single reader (QEMU). Pas de mutex.
- Race condition possible mais bénigne : QEMU peut lire un slot half-written
  une fois sur 10000 → SI invalide → mobile retry 4 frames plus tard.
- 32-byte slot alignment = 1 cache line = écriture atomique de fait sur x86.

## Layout binaire fixe (176 bytes total)

### Header (16 bytes, offset 0-15)

| Offset | Field | Size | Description |
|---|---|---|---|
| 0 | `magic` | 4 | `"CSI1"` (0x43 0x53 0x49 0x31). Validation au mmap. |
| 4 | `version` | 1 | `0x01` (room for evolution: SI5/6 SACCH ajout = v2) |
| 5 | `slot_count` | 1 | `5` (current : SI1, SI2, SI3, SI4, SI13) |
| 6 | `last_update_fn` | 2 | LE uint16, FN du dernier write (debug, watch live) |
| 8 | `reserved` | 8 | zero-fill |

### Slots (5 × 32 bytes, offset 16-175)

Slot indices fixes pour lookup direct :

| Slot # | si_type | Contenu |
|---|---|---|
| 0 | 0x01 | SI1 |
| 1 | 0x02 | SI2 |
| 2 | 0x03 | SI3 |
| 3 | 0x04 | SI4 |
| 4 | 0x0d | SI13 |

Layout par slot (32 bytes) :

| Offset (relatif) | Field | Size | Description |
|---|---|---|---|
| 0 | `si_type` | 1 | 0x01..0x0d. `0x00` = slot vide / pas encore reçu |
| 1 | `slot_flags` | 1 | bit 0 = `VALID`, bit 1 = `UPDATED_SINCE_LAST_READ` (optionnel) |
| 2 | `blob_len` | 1 | toujours `23` (BCCH burst LAPDm-wrapped) |
| 3 | `padding` | 1 | zero |
| 4-26 | `blob` | 23 | 23 bytes BCCH burst (premier byte = L2 pseudo length) |
| 27-31 | `padding` | 5 | zero (align à 32 bytes) |

Exemple slot SI3 (RSL-extracted run 2026-04-30 12:13) :

```
Offset 16+2*32 = 80 (slot 2 = SI3) :
  80: 03                            si_type=SI3
  81: 01                            slot_flags = VALID
  82: 17                            blob_len = 23
  83: 00                            padding
  84-106: 49 06 1b 17 71 00 f1 10 00 01 c9 03 05 27 47 40
          e5 04 00 2c 0b 2b 2b      ← BCCH burst SI3
  107-111: 00 00 00 00 00           padding
```

## Scheduling BCCH par TC (TS 44.018 §3.4 table 1)

```
TC = (FN / 51) mod 8

TC=0  → SI1
TC=1  → SI2
TC=2  → SI3
TC=3  → SI4
TC=4  → SI3
TC=5  → SI2
TC=6  → SI3
TC=7  → SI4
```

**SI3 émis 3× par cycle multiframe** (TC=2,4,6) car critique pour cell selection.
Implémentation stricte de cette table — **pas de round-robin naïf**.

**Fallback slot vide** : si `si_type=0x00` au TC demandé → fallback SI3 (slot 2). Justification : SI3 est le SI le plus fréquemment broadcast réel,
le mobile l'attend toujours.

## Configuration runtime — `CALYPSO_SI_MMAP_PATH`

```c
const char *path = getenv("CALYPSO_SI_MMAP_PATH");
if (!path) path = "/dev/shm/calypso_si.bin";
int fd = open(path, O_RDONLY);
if (fd < 0) {
    /* fallback to hardcoded SI3 with warning log */
    return -1;
}
```

- `CALYPSO_SI_MMAP_PATH=/path/to/file` : override
- Non défini : `/dev/shm/calypso_si.bin` par défaut
- File absent au boot : log warning, fallback SI3 hardcoded (transition
  graceful pendant dev, à supprimer une fois bridge.py stable)

## Atomicity policy

**Single writer / single reader, pas de mutex.**

- Writer (rsl_si_tap.py) : update slot complet en un seul `mm[off:off+32] = bytes(32)` (atomic Python memoryview → kernel syscall).
- Reader (QEMU) : copie le slot dans local stack avant utilisation.
- Race rare : si lecture happens during 32-byte write → blob potentiellement half-old/half-new. Détection : `slot_flags & VALID`. Si non valid → fallback SI3.
- Memory barrier x86 : implicite pour writes alignées 8 bytes.

**Évolution v2 si race observée** : utiliser `slot_flags bit 1 = UPDATED`, write
blob d'abord, set bit en dernier. QEMU read clear bit. Pas implémenté en v1.

## Process responsibilities

### `scripts/rsl_si_tap.py` — single writer

- pcap sniff `lo:3003` (RSL/Abis IP)
- Parse IPA header (3 bytes `len + 0x00`)
- Parse RSL message : check msg discriminator `0x0c` (CCHAN), `0x1e` (BCCH INFO IE)
- Extract `RSL_IE_FULL_BCCH_INFO` (IE 0x21) + `RSL_IE_SYSINFO_TYPE` (IE 0x1e)
  - SI1=0, SI2=1, SI3=2, SI4=3, SI13=8 (osmocom encoding)
- Map vers slot index : SI1→slot 0, SI2→slot 1, SI3→slot 2, SI4→slot 3, SI13→slot 4
- Write slot via mmap, update header `last_update_fn`

### `calypso_fbsb.c` — single reader

- Au boot : open + mmap O_RDONLY, valide magic = "CSI1"
- Si absent / magic invalide → fallback SI3 hardcoded existant + log warning
- Sur `case DSP_TASK_ALLC` :
  1. Calculer TC = `(s->fn / 51) % 8`
  2. Lookup slot SI selon table TC ci-dessus
  3. Si slot.si_type == 0x00 ou !VALID → fallback slot 2 (SI3)
  4. Copier blob 23 bytes dans `a_cd[3..14]` (packing LE word→byte standard)
  5. Echo `d_task_d=24, d_burst_d=N` dans read pages

## Status v1

- [x] Spec écrite (ce document)
- [ ] QEMU consumer (étape 2a) — `calypso_fbsb.c` mmap + lookup
- [ ] Stub populator (étape 2b, **disposable**) — `scripts/populate-si.sh` write hex blobs RSL-extracted
- [ ] Test intermédiaire — vérif `MM_EVENT_CELL_SELECTED` + LU REQ
- [ ] RSL sniffer (étape 3) — `scripts/rsl_si_tap.py`
- [ ] Test integration — bridge.py replace stub
- [ ] Cleanup — suppr `scripts/populate-si.sh`, suppr fallback hardcoded SI3 dans `calypso_fbsb.c`

## Évolutions futures (v2+)

- Slot 5+ : SI5 / SI6 (SACCH FILLING) — pour cycle dédié post-LU
- Header field : `bts_id` pour multi-BTS support
- Memory barrier strict via `slot_flags bit 1 = UPDATED`
