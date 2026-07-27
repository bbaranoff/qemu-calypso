# Run results — mesures chiffrées, règles de décision, reproduction

Résultats **mesurés** (pas d'affirmation sans chiffre), chacun confronté à une règle de
décision explicite. Le statut dépend du **mode** : chaque section nomme le sien via le
manifeste de run (`[calypso-manifest]` en tête de log = config `CALYPSO_*` **effective**
après le parseur value-list, donc reproductible).

> Ce fichier est régénéré à la main après chaque campagne de mesure. Les commandes
> d'extraction sont données pour que **n'importe qui rejoue les chiffres** sur ses logs.

---

## Run A — `SHUNT_LEGIT` (mode fiable, DSP off)

**Manifeste :** `CALYPSO_SHUNT_LEGIT=1  CALYPSO_SHUNT_NO_CANNED=1  CALYPSO_DSP_RUN_C54X=0`
· `CALYPSO_NATIVE=0` · `CALYPSO_FRAME_IT_NATIVE=1`. Run 123.7 s, une LU complète.

| # | Mesure | Valeur | Règle de décision | Verdict |
|---|---|---|---|---|
| A1 | éviction ring (3 politiques) | **overflow=0, ttl=0, reps=0** (`EVICT-STATS`) | ≥2 des 3 à zéro ⇒ code mort à retirer | ✅ **les 3 mortes** |
| A2 | profondeur ring | **max 1** (bucket 0-1 uniquement, 2 runs) | max ≤ 2-3 ⇒ buffer 1-slot déguisé | ✅ ring = 1-slot |
| A3 | `delta = fn_bloc − fn_L1` au DISPATCH | **−553 (±1), n=106, aucune dérive** | petit/stable ⇒ sélection FN inutile ; dérive ⇒ à faire | ✅ **stable, sélection FN inutile** |
| A4 | ENQUEUE vs DISPATCH | **11** vs **44** (0.09/s vs 0.36/s, ratio 4:1 = 4 bursts/bloc) | comptes proches ⇒ déséquilibre résorbé | ✅ pas de déséquilibre |
| A5 | RACH → LU ACCEPT | **2.70 s** (2 runs), **0 retry T3211** | un chiffre vaut mieux que « quasi systématique » | ✅ LU 2.7 s, 1er coup |

**Lecture.** La saturation `depth=32` que j'avais constatée existait en mode **DSP en //**
(jitter c54x), **pas** ici. En `SHUNT_LEGIT`, producteur (0.09/s) ≪ consommateur, file
jamais > 1, **aucune** éviction (les 3 compteurs à 0). Le `fn` stocké-non-utilisé et la
triple politique d'éviction (overflow / TTL / reps) sont, dans ce mode, **du code mort à
retirer**, pas un bug latent.

**A3 en détail.** Le `delta` est **stable à −553 ± 1** sur 106 présentations, **sans
dérive** au cours du run. Ce n'est pas un décalage de sélection : −553 est l'**offset de
phase inter-horloge connu** (le même que corrige le recale FN / la ré-écriture req-ref) —
une constante entre l'horloge du bloc (gr-gsm) et l'horloge L1 firmware. Par ta règle
(stable ⇒ inutile), **la sélection par FN n'apporte rien** : elle ne changerait pas quel
bloc est présenté (tous décalés du même −553). Le `fn` du bloc reste non-utilisé **à
raison**. Recommandation retirable, preuve chiffrée à l'appui.

### Reproduire (Run A)

```bash
# A1 overflow (+ split via EVICT-STATS sur binaire instrumenté)
grep -c "RING OVERFLOW" /root/qemu.log
grep "EVICT-STATS" /root/qemu.log | tail -1
# A2 histogramme profondeur
grep -oE "depth=[0-9]+" /root/qemu.log | grep -oE "[0-9]+" | \
  awk '{if($1>m)m=$1;b[($1<=1)?"0-1":($1<=3)?"2-3":($1<=7)?"4-7":"8+"]++}END{print "max",m;for(k in b)print k,b[k]}'
# A3 delta (binaire instrumenté)
grep -oE "delta=-?[0-9]+" /root/qemu.log | grep -oE "\-?[0-9]+" | \
  sort -n | awk '{a[NR]=$1}END{print "min",a[1],"med",a[int(NR/2)],"max",a[NR]}'
# A4 débits
echo "ENQUEUE=$(grep -c 'feed_sdcch: ENQUEUE' /root/qemu.log) DISPATCH=$(grep -c 'DISPATCH SDCCH' /root/qemu.log)"
# A5 temps LU + retries
grep -E "CHANNEL REQUEST: 00|LOCATION UPDATING ACCEPT" /root/mobile.log | head
grep -c "T3211" /root/mobile.log
```

---

## Run B — `NATIVE_HELPED` : diagnostic `d_fb_det = 0` (CLOS, chiffré)

**Manifeste :** `CALYPSO_NATIVE_HELPED=1` (⇒ `CALYPSO_FB_IQ_DARAM=1 CALYPSO_FB_IQ_BASE=0x9210`
→ feed réel de l'entrée démod `0x9213`(I)/`0x9215`(Q)). rxlev réel **−47 dBm** (trf6151/DECAN).

**Rappel adressage** (souvent confondu) : `0x2a00` = **workzone SORTIE** du démod (BSP write,
réécrit par le DSP) ; l'**entrée** corrélateur = `0x9213/0x9215` via FB-STREAM (`FB_IQ_BASE=0x9210`).
Donc `CALYPSO_BSP_DARAM_ADDR=0x2a00` ne nourrit **pas** le corrélateur — c'est `FB_IQ_BASE` qui le fait.

| # | Mesure | Résultat | Verdict |
|---|---|---|---|
| B1 | table réf `0x2c00` au kernel `0xa076` | **peuplée** (écrite par PC `0x9fd5`, démod), pas vide ; se stabilise à `001f…` (plat) | ✅ pas « corrèle contre du vide » |
| B2 | module accu A/B + max fenêtre 296 | `\|A\|=294908 \|B\|=36863` (non-nuls), `in(2a00):max=21229@0` (spike **index 0 = DC**, pas un ton) | ✅ le MAC calcule ; entrée douteuse (DC) |
| B4B | flux instruction par instruction après `0x9ac0` | `STL A` → boucle de **normalisation** (A≫1 jusqu'à 0) → `952c`→`9511`→`a033` (setup pointeurs) → re-boucle. **XPC reste 0, n'atteint JAMAIS `0xec07`** | ✅ mur de flux : boucle sans sortir vers la décision |
| B4 | watchpoint écritures `data[0x08f8]` | **`count = 0`** — jamais écrit | 🔑 **d_fb_det jamais écrit** = chemin pas atteint (≠ « écrit 0 ») |
| SCAN | refs `0x08f8` dans la PROM (bank 0) | **30+** occurrences, dont des **writers** (`STL A` @0xd2c0/0xd30e, cluster `0xa335/0xa33b/0xa3cb`, RMW `0xff20` @0xe5af) | ✅ les writers **existent**… |

### Conclusion (prouvée instruction par instruction)
Le firmware DSP **contient** le code qui écrit `d_fb_det` (SCAN : 30+ refs, plusieurs writers),
le corrélateur **calcule** bien (B2 : A/B non-nuls), **mais** son flux **boucle dans le bank 0**
(`0x8d00`→`0xa07x`) sans jamais atteindre l'étage publish/décision (B4B) → **aucun writer ne
s'exécute** (B4 : `data[0x08f8]` jamais écrit). C'est le **mur de contrôle de flux « RANK3 »,
désormais chiffré**, pas une conjecture.

**Donc le corrélateur natif émulé ne complète pas la FBSB.** Piste résiduelle (non poursuivie) :
l'entrée `0x2a00` a son max **à l'index 0 (DC)**, pas un ton FCCH → le corrélateur cherche un ton
absent → ne décide jamais. **Le shunt host-side EST la voie qui marche** (LU/SMS OK en shunt,
KO en natif) — voir la matrice statut × mode dans `hw/arm/calypso/doc/ETAT_ACTUEL.md`.

### Reproduire (Run B)
```bash
CALYPSO_NATIVE_HELPED=1 CALYPSO_B2=1 CALYPSO_B4=1 CALYPSO_B4B=1 CALYPSO_SCAN_08F8=1 ./start-direct.sh
grep -E "B2 @0x9ac0|B4-DFBDET-WR|B4B-FLOW|SCAN-08F8" /root/qemu.log | head -60
```

---

## Acquis contextuels (autres modes, pour situer)

- `SHUNT_LEGIT` : registration (LU ACCEPT + TMSI), SMS MO/MT bidirectionnel, service tenu,
  Ctrl-C recover — **DONE** (cf `hw/arm/calypso/doc/ETAT_ACTUEL.md`, matrice statut × mode).
- Voix TCH/F : call atteint l'ASSIGNMENT COMMAND → ASSIGNMENT FAILURE (shunt ne présente pas
  le TCH DL) ; call fake_trx = ACTIVE+audio ⇒ réseau OK (cf `hw/arm/calypso/doc/VOIX_PLAN.md`).
