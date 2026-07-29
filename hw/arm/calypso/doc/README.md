# QEMU-Calypso — documentation

Ce README **oriente**. Il n'explique rien et ne fait autorité sur rien : chaque fait détaillé
vit dans un document qui, lui, fait autorité (§3). Si ce README contredit
[`ETAT_ACTUEL.md`](ETAT_ACTUEL.md), c'est ce README qui a tort.

---

## 1. Ce que c'est

Ce dépôt est une émulation QEMU du baseband GSM **TI Calypso** — le SoC des téléphones
Openmoko/Motorola C1xx ciblés par osmocom-bb. Le Calypso est un **bi-processeur** : un **ARM7**
qui exécute le firmware Layer 1 (osmocom-bb), et un **DSP TMS320C54x** qui exécute un firmware
propriétaire en mask-ROM et fait tout le traitement du signal (corrélation FCCH, démodulation,
égalisation). Les deux communiquent par une **API RAM** partagée (le DSP écrit `data[0x08f8]`,
l'ARM lit le MMIO `0xFFD001F0`). L'émulation modélise l'ARM, le DSP, la TPU/TSP, le BSP (chemin
I/Q) et la RF (trf6151/twl3025), et fait tourner ce mobile émulé **face à une vraie pile Osmocom**
(osmo-bts-trx, osmo-bsc, osmo-msc) — le mobile doit donc réellement se synchroniser, camper,
faire une Location Update et des SMS, pas simuler qu'il le fait.

---

## 2. Où on en est

**En une phrase :** le mode **shunt** (FBSB fait côté hôte) campe, fait la LU et les SMS ; le mode
**natif** (le DSP c54x fait le FBSB) reçoit bien le signal — l'entrée du démodulateur est vivante —
mais sa sortie est du DC plat et `d_fb_det` reste 0.

Le statut n'existe **pas dans l'absolu** : il dépend du mode. Ne jamais citer un statut sans son mode.

| Fonction | `SHUNT_LEGIT=1` | `SHUNT_NO_LEGIT=1` | `SHUNT_LEGIT=DSP,NO_CANNED` | `NATIVE` / `NATIVE_HELPED` | Instrument de vérification |
|---|---|---|---|---|---|
| FB/SB sync | OK (host) | OK | OK | **KO** : `d_fb_det=0` | `grep DETECTOR-RUN /root/qemu.log` ; `grep REAL-FB` |
| rxlev serving | OK (−47 dBm) | OK | OK | OK mais **mocké** (`shunt_dispatch_pm` non gaté) | `grep "MON: f=" /root/mobile.log` |
| Camp (C3) + sysinfo | OK | OK | OK | **KO** « No sysinfo » | `grep -c sysinfo /root/mobile.log` |
| LU + TMSI | OK | OK | OK | KO | `grep -icE "LOCATION UPDATING ACCEPT" /root/mobile.log` |
| SMS MO / MT | OK | OK | **WIP flaky** | KO | log SMSC |
| Ctrl-C mobile → ré-acquisition | OK | OK | OK | non testé | hook `on_arm_write(d_dsp_page,0)` |
| Voix TCH/F | **WIP** : ASSIGNMENT FAILURE | WIP | WIP | KO | [`VOIX_PLAN.md`](VOIX_PLAN.md) |
| c54x exécuté à la cadence trame | non | non | **oui** | **oui** | `dsp_n_exec_2/5` ; absence de `DSP Error Status: 2048` |
| Entrée du démod alimentée | s.o. | s.o. | non mesuré | **OK** : `data[0x4c00]`, stride 5 | `CALYPSO_WATCH_9F00_RD=1` (PC `0x9fb5`) |
| Sortie du démod exploitable | s.o. | s.o. | non mesuré | **KO** : DC plat (\|DC\|≈rms, dφ≈0) | `CALYPSO_DARAM_DUMP=1` + `tools/corr_iq.py --src ddump` |

Le mode **`CALYPSO_L1=c`** (Layer 1 haut niveau en C) est **inexécutable en l'état** : il s'auto-annule
(`calypso_layer1_tick` n'est appelé que si le shunt est inactif, or `L1=c` arme le shunt). Ne pas
s'en servir comme référence.

**Chiffre à ne pas mal citer :** les compteurs `REAL-FB` sont plafonnés par le logger
(`calypso_dsp_shunt.c:1670`) — « 280/300 » décrit le contenu des 300 premières lignes loguées,
**pas** un taux de détection sur le run.

---

## 3. Où est la vérité : quel document ouvre quoi

Un seul document fait autorité par sujet. En cas de conflit, celui de la colonne « autorité sur »
prime sur toute citation faite ailleurs, y compris ici.

| Document | Autorité sur | Quand l'ouvrir |
|---|---|---|
| **[`ETAT_ACTUEL.md`](ETAT_ACTUEL.md)** | **l'état courant** : matrice statut × mode, architecture réelle, fausses pistes closes | **toujours en premier.** Prime sur tout autre doc en cas de conflit |
| [`TODO.md`](TODO.md) | **la suite** : quoi faire, par mode et par priorité (P1/P2/P3), et ce qui est déjà fait | avant de choisir sur quoi travailler |
| [`../../../../RAPPORT_DFBDET.md`](../../../../RAPPORT_DFBDET.md) | **l'enquête `d_fb_det`** : pourquoi le corrélateur natif ne publie pas, publisher `0x79e4`, bancarisation | pour toute question sur le blocage natif |
| [`../../../../QUICK_START.md`](../../../../QUICK_START.md) | **le démarrage** : lancer la pile, choisir un mode | première session, ou pour rejouer un run |
| [`../../../../run_results.md`](../../../../run_results.md) | **les runs datés et chiffrés** | pour citer un chiffre plutôt qu'une impression |
| [`C54X_INSTRUCTIONS.md`](C54X_INSTRUCTIONS.md) + [`opcodes/`](opcodes/) | **la sémantique des instructions C54x** ; `opcodes/tic54x_hi8_map.md` fait foi sur le décodage hi8 | avant de toucher au décodeur, et pour arbitrer « le firmware fait X » |
| [`DSP_ADDRESS_MAP.md`](DSP_ADDRESS_MAP.md), [`DSP_ARM_LINKAGE.md`](DSP_ARM_LINKAGE.md), [`DSP_ROM_MAP.md`](DSP_ROM_MAP.md), [`SHUNT_LEGIT_ADDRESS_MAP.md`](SHUNT_LEGIT_ADDRESS_MAP.md) | **les cartes mémoire** : cellules DSP, loi d'adressage ARM↔DSP↔API RAM, mask-ROM | dès qu'une adresse est en jeu — ne jamais deviner une adresse |
| [`CALYPSO_HW.md`](CALYPSO_HW.md), [`hardware-map.md`](hardware-map.md), [`SERCOMM_GATE_ARCHITECTURE.md`](SERCOMM_GATE_ARCHITECTURE.md) | le SoC, les périphériques, le canal Sercomm/L1CTL | travail sur un périphérique ou sur le lien hôte |
| [`VOIX_PLAN.md`](VOIX_PLAN.md) | la voix TCH/F | P1 voix |
| [`project/`](project/) | audit du décodeur, conditions `BC`, bugs et correctifs | dette et régressions du décodeur |
| [`datasheets/`](datasheets/), `datasheets/TI_SPRU172C_C54x_Mnemonic_Instruction_Set.pdf` | la documentation constructeur TI | arbitrage final sur une sémantique d'instruction |
| [`archive/`](archive/) | l'**historique** (sessions datées, pistes closes) | **n'est plus la vérité courante** — n'y chercher qu'un contexte, jamais un statut |

Il n'y a **pas** de document faisant autorité sur les variables `CALYPSO_*`. La liste vraie se
regénère (≈ 300 variables, 375 sites) :

```bash
docker exec osmo-operator-1 bash -lc 'cd ${QEMU_TREE}/hw/arm/calypso && \
  grep -rhoE "getenv\(\"CALYPSO_[A-Z0-9_]+\"\)" *.c | grep -oE "CALYPSO_[A-Z0-9_]+" | sort -u'
```

Un `grep CALYPSO_` nu renvoie un sur-ensemble bruité (macros d'IRQ, registres SIM, gardes d'include).
`CALYPSO_DEBUG` est un **namespace séparé** (≈ 98 tokens) :
`grep -rhoE '(calypso_debug_enabled|cdbg_env)\("[^"]*"' .`

---

## 4. Règles de travail non négociables

**4.1 — Le runtime est DANS le conteneur.** L'arbre vivant est `${QEMU_TREE}` **à l'intérieur**
de `osmo-operator-1`. Tout accès passe par `docker exec osmo-operator-1 bash -lc '...'`. Un éditeur
qui voit un fichier depuis l'hôte ne voit **pas** le runtime.

**4.2 — `${GSM_ROOT}/qemu-calypso` est un overlay MORT au runtime.** Ne rien y écrire : la modification
n'aura aucun effet sur le run. Après toute écriture dans `doc/`, propager :
`cd ${QEMU_TREE} && ./make-overlay.sh`.

**4.3 — On ne change pas un défaut de configuration.** Documenter, pas modifier. Toute variation se
fait **en CLI** (l'idiome `: "${VAR:=…}"` du projet garantit que la CLI gagne sur les `.env`).
Exception connue à cette garantie : `CALYPSO_MODE=full-grgsm` verrouille cinq variables avec `=`
et non `:=` (`SHUNT_NO_CANNED`, `DSP_L1STUB`, `DSP_L1_STUB`, `FORCE_FBSB`, `FORCE_AGCH`) —
« variable posée » n'y est pas « variable effective ».

**4.4 — Toute sonde est gatée par une variable d'environnement, OFF par défaut.** Une sonde ne doit
jamais changer le comportement d'un run qui ne la demande pas. Attention : **le projet contient
quatre idiomes de gate incompatibles** — `getenv(X) ? 1 : 0` (que `unset` désactive, pas `X=0`),
`atoi(e) > 0`, `*e == '1'`, et des gates **ON par défaut** qu'on coupe par `X=0` ou par une variable
`X_OFF=1`. Vérifier l'idiome avant de conclure qu'une sonde est éteinte.

**4.5 — Ne pas relancer la pile, ne pas toucher à git.** L'utilisateur relance lui-même
(`start-clean.sh`, qemu, osmocon, mobile) ; sur les logs, lecture seule. Le dépôt est déjà commité.

**4.6 — Distinguer explicitement MESURE / HYPOTHÈSE / INVALIDÉ**, et nommer l'**instrument** ou la
**commande** de chaque affirmation technique. Trois pièges d'hygiène déjà payés :
- `BUILD-STAMP` **ne dit pas** la fraîcheur du binaire (c'est le `__DATE__` de `calypso_dsp_shunt.c`,
  pas de l'unité modifiée). Instrument correct : mtime du `.o` recompilé + `lstart` du process.
- Toute mesure prise via le **monitor QMP** est hors fenêtre API RAM et *racy*. Les mesures valides
  se prennent **de l'intérieur** (log au point d'écriture, ou dump interne `ddump`).
- Comparer le **mtime des artefacts** (`/dev/shm/*.cfile`) au `lstart` du process avant de les
  interpréter : un fichier périmé d'un run antérieur ne mesure pas le run courant.

### La méthode de sonde — quatre règles

Payées quatre fois. À appliquer **avant** d'écrire une nouvelle sonde.

1. **Une sonde se conçoit par sa CONDITION DE DÉCLENCHEMENT, pas par son adresse.** Un plafond global
   est mangé par le PC le plus bruyant.
2. **Préférer un AGRÉGAT** (compte tout le run, imprime un tableau) **à un flux plafonné**, et
   **prévoir un témoin de saturation** (heartbeat) — c'est lui qui distingue « pas d'événement » de
   « sonde morte ».
3. **Distinguer « varie dans l'espace » de « varie dans le temps ».** Une courbe sur N cellules n'est
   pas un signal ; seule la variation temporelle **à cellule figée** en est un.
4. **« Pas de log » n'est jamais « pas d'événement »** tant que la sonde n'est pas vérifiée VIVANTE et
   sa fenêtre vérifiée COUVRANTE. Causes déjà rencontrées : plafond saturé ; seuil de dump trop haut ;
   plage écrite **côté hôte** donc invisible du chemin d'écriture DSP ; variable absente du run.

---

## 5. Lancer

Voir **[`../../../../QUICK_START.md`](../../../../QUICK_START.md)** — choix du mode, commandes de
démarrage, et où lire les logs. Pour un premier run qui marche, prendre le mode fiable
(`CALYPSO_SHUNT_LEGIT=1`) ; pour travailler le corrélateur natif, prendre le run de référence
documenté dans [`ETAT_ACTUEL.md`](ETAT_ACTUEL.md) et [`../../../../RAPPORT_DFBDET.md`](../../../../RAPPORT_DFBDET.md).


## Rapports ajoutes le 2026-07-28

| Document | Autorite sur |
|---|---|
| `../../../../RAPPORT_OPCODES.md` | Audit du decodeur c54x : ~40 findings, ~15 de gravite 1 (longueur d'instruction fausse). Rendu brut de 10 agents, **rien d'applique**. Lire ses trois reserves en tete. |
| `../../../../PLAN_APPLICATION.md` | Ordre d'application des correctifs et tests de non-regression (produit par le workflow `calypso-reste-a-faire`). |
| `opcodes/tic54x-opc.c` | **Table binutils — autorite sur l'encodage et la LONGUEUR** des instructions. Format : `{ "mnemo", MOTS, cycles, classe, OPCODE, MASQUE, ... }`. Le champ MOTS fait foi : une longueur fausse desynchronise tout le decodage en aval. |

Ordre d'autorite sur les opcodes : `opcodes/tic54x-opc.c` > `spru172c.pdf` (semantique) > le code
> les tableaux de synthese. **Ne jamais conclure depuis un commentaire de code** : plusieurs se
sont averes perimes le 2026-07-28.


## Recenser les béquilles : le marqueur `@BEQUILLE`

Le projet a accumulé des dizaines de contournements (`FB_CORR_ENTRY`, `FB_ENERGY`, `FB_STREAM`,
`DARAM_FORCE`, `NATIVE_HELPED`, forçage de `DP`, `SEED5AC8_VAL`…) sans étiquette commune — au
point qu'on ne savait plus lequel masquait quoi. Le 2026-07-28, des heures ont été passées à
analyser un étage de démodulation qui, en mode natif, **n'est jamais exécuté** : il ne l'était
que par un reroute qu'on croyait retiré.

**Règle : toute béquille porte le marqueur `@BEQUILLE`.** Un seul grep les liste toutes :

Trois greps, selon ce qu'on cherche. Ils s'appliquent depuis `${QEMU_TREE}` et **excluent
`doc/`** : sinon on attrape aussi la présente page, qui décrit la convention sans être une
béquille, et le décompte est faux.

```bash
# A) la liste complète, avec fichier:ligne — le grep de référence
grep -rn "@BEQUILLE" hw/arm/calypso/*.c hw/arm/calypso/*.h calypso*.env

# B) juste les noms, dédupliqués — pour un coup d'œil
grep -rhoE "@BEQUILLE — [A-Za-z_0-9]+" hw/arm/calypso/*.c hw/arm/calypso/*.h calypso*.env | sort -u

# C) combien de fichiers en contiennent
grep -rl "@BEQUILLE" hw/arm/calypso/*.c hw/arm/calypso/*.h calypso*.env | wc -l
```

État au 2026-07-28 — (B) renvoie :

```
@BEQUILLE — FB_ENERGY          reroute du corrélateur (avec FB_CORR_ENTRY)
@BEQUILLE — FB_STREAM          injection d'échantillons à la place du DMA on-chip
@BEQUILLE — FIX_BRINT0_UNMASK  démasquage artificiel de l'IMR bit 5 (diagnostic)
@BEQUILLE — NATIVE_HELPED      profil qui repose FB_CORR_ENTRY / FB_ENERGY / FB_IQ_*
@BEQUILLE — SHUNT_REAL_FB      détection FB côté hôte, court-circuite le corrélateur DSP
```

Une même béquille peut apparaître à **plusieurs** sites (émetteur et récepteur d'une injection,
par exemple) : (A) les montre tous, (B) les regroupe.

Format imposé, juste au-dessus du bloc :

```c
/* @BEQUILLE — NOM_DU_GATE  (VARIABLE_ENV, defaut OFF)
 *   masque  : ce que ce contournement remplace (la branche réelle non implémentée)
 *   retirer : la condition qui le rend inutile
 */
```

Trois exigences, non négociables :
1. **gatée par variable d'environnement, défaut OFF** — le comportement sans variable reste celui
   d'origine ;
2. **annoncée comme béquille avant le run**, pas après le résultat — sinon on lance une mesure en
   croyant tester un correctif ;
3. **toute mesure obtenue sous béquille est étiquetée comme telle** quand on la cite.

Distinguer trois choses qui se ressemblent et ne se traitent pas pareil :

| | vit | finit |
|---|---|---|
| **béquille** | tant que la branche réelle manque | remplacée par la branche réelle |
| **sas** (`CALYPSO_FIXES`) | le temps d'un test sous charge | dégatée (confirmée) ou supprimée (infirmée) |
| **diagnostic** | le temps d'une question | retiré — ne se confirme **jamais** |

Le sas se vide, la béquille reste : ne jamais laisser vieillir un correctif validé derrière son
gate, c'est ainsi qu'un sas devient une béquille.
