# Documentation — QEMU-Calypso

> ⭐ **[ETAT_ACTUEL.md](ETAT_ACTUEL.md)** est la **source de vérité unique** (ce qui marche,
> l'architecture réelle, l'état du DSP natif, les fausses pistes). En cas de conflit, il prime.
>
> Démarrage rapide : **[QUICK_START.md](../../../../QUICK_START.md)** (racine du dépôt).

Les docs ci-dessous sont la **référence courante durable**. L'historique (sessions datées,
rapports d'étape, pistes closes) est rangé par thème dans **[archive/](archive/README.md)**.

---

## Référence — Adresses & DSP

*Cartes canoniques ARM↔DSP↔API RAM, mask-ROM.*

- [`SHUNT_LEGIT_ADDRESS_MAP.md`](SHUNT_LEGIT_ADDRESS_MAP.md) — mapping des cellules DSP (data/api_ram), chaîne FB/SB/rxlev/a_cd. **Réf. d'adresses.**
- [`DSP_ROM_MAP.md`](DSP_ROM_MAP.md) — carte de la mask-ROM DSP.
- [`DSP_ADDRESS_MAP.md`](DSP_ADDRESS_MAP.md) — carte des cellules DSP (data/api_ram) par adresse. **Réf. d'adresses.**
- [`DSP_ARM_LINKAGE.md`](DSP_ARM_LINKAGE.md) — correspondance ARM↔DSP (loi d'adressage, MMIO). **Réf. d'adresses.**

## Référence — Matériel

*SoC Calypso, périphériques, canal Sercomm.*

- [`CALYPSO_HW.md`](CALYPSO_HW.md) — carte matérielle du SoC.
- [`hardware-map.md`](hardware-map.md) — plan mémoire / périphériques.
- [`SERCOMM_GATE_ARCHITECTURE.md`](SERCOMM_GATE_ARCHITECTURE.md) — canal Sercomm / L1CTL.

## Référence — ISA C54x

*Jeu d'instructions TMS320C54x (décodeur QEMU).*

- [`C54X_INSTRUCTIONS.md`](C54X_INSTRUCTIONS.md) — jeu d'instructions.
- [`opcodes/`](opcodes/) — cartes d'opcodes (`0x68_0x6F`, `0xF3`, `tic54x_hi8_map`).
- [`project/AUDIT_DECODER_20260508.md`](project/AUDIT_DECODER_20260508.md) — audit du décodeur.
- [`project/STEP2_BC_CONDS.md`](project/STEP2_BC_CONDS.md) — conditions de branchement (BC).

## Bugs, index & suivi

- [`project/BUGS_AND_FIXES.md`](project/BUGS_AND_FIXES.md) — bugs connus + correctifs.
- [`project/REPORT_CLAUDE_WEB_PIPELINE.md`](project/REPORT_CLAUDE_WEB_PIPELINE.md) — pipeline du projet.
- [`TODO.md`](TODO.md) — index des tâches (P0/P1/P2).
- [`VOIX_PLAN.md`](VOIX_PLAN.md) — plan d'implémentation voix (TCH/F), pour le P1 « Voix ».
- Ancien index maître archivé : [`archive/MASTER.md`](archive/MASTER.md) (supplanté par ETAT_ACTUEL + ce README).

## Datasheets constructeur

- [`datasheets/`](datasheets/) — PDF TI (SPRU131/172/288…), FreeCalypso, dumps DSP-ROM.

## Archive

- [`archive/`](archive/README.md) — **doc historique, navigation par thème**. N'est plus la vérité courante.
