# Calypso BSP/RIF DMA — QEMU implementation

## Rôle

Sur le vrai matériel, le **BSP** (Baseband Serial Port) est le lien
série synchrone qui DMA les samples I/Q de l'IOTA RF frontend
directement dans la **DARAM du DSP C54x**. Le code DSP (FCCH/SCH/burst
detection en PROM0) lit ces samples depuis un buffer DARAM fixe et
poste les résultats dans le NDB.

Dans QEMU on n'émule pas le bus série IOTA. À la place :

```
   bridge.py  (gr-gsm GMSK modulateur)
        │
        │  UDP TRXD v0  (port 6802)
        ▼
   sercomm_gate.c::trxd_cb()
        │
        │  calypso_bsp_rx_burst(tn, fn, int16 *iq, n_int16)
        ▼
   calypso_bsp.c::calypso_bsp_rx_burst()
        │
        │  copie de mots dans dsp->data[<DARAM target>]
        ▼
   C54x DARAM  ──read──▶  PROM0 FB-det handler  ──▶  NDB d_fb_det
```

Pas de poke dans le NDB, pas de faux `d_fb_det`, pas d'ancien header
`rx_burst`. Le seul boulot du module BSP est de poser le flux int16
au bon endroit en DARAM ; le firmware DSP fait tout le reste.

## Fichiers

| Path | Rôle |
|---|---|
| `include/hw/arm/calypso/calypso_bsp.h` | API publique |
| `hw/arm/calypso/calypso_bsp.c` | Implémentation DMA |
| `hw/arm/calypso/sercomm_gate.c::trxd_cb` | Appelle `calypso_bsp_rx_burst` |
| `hw/arm/calypso/calypso_trx.c::calypso_trx_init` | Appelle `calypso_bsp_init(s->dsp)` |
| `hw/arm/calypso/calypso_c54x.c::data_read` | Tracer FBDET / d_spcx_rif |
| `hw/arm/calypso/meson.build` | Ajoute `calypso_bsp.c` |

## Configuration

Deux variables d'environnement (lues une fois à `calypso_bsp_init`) :

| Env var | Défaut | Sens |
|---|---|---|
| `CALYPSO_BSP_DARAM_ADDR` | `0` | Adresse de mot dans le data-space DSP (DARAM) |
| `CALYPSO_BSP_DARAM_LEN` | `1184` | Nombre max de mots int16 copiés par burst (148×4×2) |

Quand `CALYPSO_BSP_DARAM_ADDR=0`, le BSP tourne en mode **DISCOVERY** :
chaque burst est loggué mais rien n'est écrit en DARAM. C'est le
défaut sûr pendant le debug DSP.

## Procédure de discovery

L'adresse cible du buffer DARAM utilisée par la routine FB-det est
identifiée à l'exécution en traçant les data reads émis quand le PC
DSP est dans le handler FB-det en PROM0 (plage `0x7730..0x7990`,
mémoire `project_dsp_fb_det`) :

```
[c54x] FBDET RD [0x<addr>]=0x<val> PC=0x<pc> insn=<n>
```

Le cluster d'`<addr>` (avec PC juste avant la première boucle MAC/FIR)
est la cible BSP. Mettre `CALYPSO_BSP_DARAM_ADDR` en conséquence et
relancer.

## Statut (2026-04-06)

- Infra BSP complète, build clean (link manuel `-lm` requis).
- Runtime vérifié : les bursts arrivent au BSP avec une enveloppe GMSK
  correcte (|z| ≈ 32766), `d_spcx_rif` programmé par ARM à `0x0179`
  à fn=0.
- **Discovery bloquée** : le DSP n'entre jamais dans la plage PC FB-det ;
  il busy-loop dans le dispatcher à `0x81a7..0x81d6`. La cause n'est
  *pas* un bug DSP — c'est que le firmware ARM **n'écrit jamais**
  `d_task_md = FB_DSP_TASK`. Voir `TODO.md`.

Dès que le DSP atteindra `0x7730+`, le tracer FBDET révélera le buffer
DARAM et on pourra fixer `CALYPSO_BSP_DARAM_ADDR`.
