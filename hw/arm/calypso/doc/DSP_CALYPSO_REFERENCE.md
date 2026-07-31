# Le DSP du Calypso — référence matérielle, API, tâches, et ce qu'on en mesure

> **Source** : wiki Osmocom, *The Calypso DSP* (`HardwareCalypsoDSP`), complété par
> `TSM30Layer1` pour la liste des tâches.
> **Confrontation** : mesures de ce dépôt, 2026-07-28 → 30.
>
> Document jumeau de [`CHAINE_RF_MATERIELLE.md`](CHAINE_RF_MATERIELLE.md), qui
> couvre la chaîne RF. Celui-ci couvre le DSP lui-même.
>
> ⚠️ Ce document contient **le chiffre qui a manqué trois jours** : le nombre
> d'échantillons que chaque tâche consomme (§5).

---

## 1. Le matériel

| élément | ce que dit la doc | notre modèle |
|---|---|---|
| cœur | **TMS320c5x LEAD2** | `calypso_c54x.c`, interprété — ⚠️ ~96× plus lent que le silicium |
| bus | le DSP a **ses propres bus d'adresses et de données**, indépendants du bus ARM | modélisé implicitement (espaces séparés `data[]` / `prog[]`) |
| pont | **pont RHEA** entre le DSP et ses périphériques | présent : région `calypso.rhea` à `0xFFFFF900` (`calypso_soc.c`), en stub |
| horloge | fournie par la **DPLL du Calypso** | registre présent (`0xFFFF9800`, étiquetage corrigé le 30/07) mais **en stub** : la fréquence n'est pas modélisée. `f = 26 MHz × mult / (div+1)`, mult 1..30, div 0..2 |
| reset | la ligne de reset du DSP est **pilotée par l'ARM** | partiellement (`calypso_arm2dsp.c`) |
| API RAM | fenêtre partagée ARM↔DSP de **8 kWords, soit 16 ko** | ⚠️ notre moniteur mailbox ne couvre que **`0x0800..0x0FFF`** (2 kWords) — son silence au-delà ne prouve rien |

## 2. Le logiciel du DSP

Le logiciel est **essentiellement en mask-ROM** et implémente la partie
traitement du signal de GSM — **surtout côté Rx**.

**Patches.** Le firmware normal du téléphone **télécharge des patches dans la RAM
du DSP, en passant par la mémoire API**. La ROM du DSP contient un programme de
téléchargement de patches, exécuté après le reset. Les patches servent
vraisemblablement à corriger des bugs et à étendre des fonctionnalités.

> **Chez nous** : `osmocom-bb` a bien `calypso/dsp.c`, `dsp_bootcode.c` et
> `dsp_params.c`. Le mécanisme fonctionne « gratuitement » dans notre modèle, à
> condition que l'ARM écrive bien dans l'API RAM et que la ROM exécute son
> loader — mais **nous ne l'avons jamais vérifié explicitement**. À instrumenter
> le jour où un comportement ROM semble incohérent : nous exécutons peut-être
> une ROM non patchée là où le firmware croit avoir patché.

## 3. L'API — la structure de la fenêtre partagée

Elle se compose de :

- une **page de lecture** (*DB Read*), **double bufferisée**, sens DSP→ARM,
  essentiellement des valeurs de réponse ;
- une **page d'écriture** (*DB Write*), **double bufferisée**, sens ARM→DSP,
  essentiellement des commandes ;
- une page **NDB**, **non** double bufferisée ;
- une **zone de paramètres**.

> **Ce que ça explique chez nous, immédiatement** : les paires que le moniteur
> mailbox montrait sans qu'on les nomme — `d_task_md/wp0` (`0x0804`) et
> `d_task_md/wp1` (`0x0818`), `d_task_d/rp0` (`0x0828`) et `d_task_d/rp1`
> (`0x083c`), `d_burst_d/rp0` (`0x0829`) et `/rp1` (`0x083d`). **`wp`/`rp` =
> write page / read page**, et le suffixe 0/1 est **le double buffer**. Le code
> du modèle le savait (`calypso_mailbox.c` les nomme ainsi) ; la doc dit
> pourquoi.
>
> Et `d_fb_det` est dans la page **NDB** — donc **non** double bufferisée : une
> seule copie, écrasable à tout moment. À garder en tête avant de conclure d'une
> lecture ponctuelle.

### Paramètres notables

`ndb.d_tch_mode` :
- **bit 11** : sélection du modèle d'ABB. **1 = IOTA**, 0 = autre chose. Change
  **l'adresse à laquelle le registre BULDATA est attendu**.
- **bits [10:7]** : nombre de bits de garde insérés avant les données lors de
  l'émission d'un burst (**valeur réelle − 4**). Le maximum semble être 8 bits de
  garde ; au-delà, « le code du DSP fait des choses étranges ». Poser 4 émet donc
  4+4 = 8 bits de garde : `11111111000xxxxx…` — les `1` sont les gardes, puis
  trois `0` de queue, puis les données.

> **Chez nous** : à vérifier avant tout travail sur l'UL. Si `d_tch_mode` bit 11
> n'est pas posé à 1, le DSP attend `BULDATA` **ailleurs** — et notre modèle
> d'ABB ne le saurait pas.

## 4. Le flux temporel — et pourquoi l'IT de trame n'était pas optionnelle

Le flux, pour l'exploitation **mono-slot** :

1. L'ARM utilise la **page d'écriture DB** pour définir l'ensemble des actions
   (*tasks*) que le DSP doit exécuter.
   - il indique **quelles tâches** (GSM et/ou MISC) exécuter, par un champ de la
     page **NDB** ;
   - il indique **laquelle des deux pages d'écriture** double bufferisées il a
     utilisée, également dans la NDB.
2. L'ARM **demande au TPU d'émettre une interruption FRAME à la trame GSM
   suivante**.
3. À la **première interruption FRAME**, le DSP commence à traiter les tâches.
   - il lui faut un temps de préparation de **66 quart-de-bits** avant de
     recevoir les échantillons RF (`DSP_SETUP_TIME`) ;
   - il reçoit et traite les échantillons ;
   - les résultats sont rangés au fur et à mesure dans la **page de lecture DB**.
4. À l'**interruption FRAME suivante**, l'ARM peut lire les résultats.

Ces flux se **recouvrent** normalement : dès après la première IT, l'ARM peut
déjà écrire le jeu de commandes suivant dans **l'autre** page d'écriture.

Certaines tâches prennent **plus d'une trame**. Le code ARM doit savoir combien
d'interruptions attendre avant de lire les résultats.

> 🔑 **Confirmation directe pour nous** : l'IT de trame TPU→DSP **n'était pas
> délivrée** dans notre modèle avant le 2026-07-30 (`calypso_tpu.c`,
> `CALYPSO_TPU_DSP_FRAME_IT`). Sans elle, l'étape 3 n'arrive jamais : le DSP ne
> commence **aucune** tâche, quelles que soient les commandes écrites par l'ARM.
> Ce câblage est donc une **correction de modélisation**, pas une béquille — la
> doc le dit noir sur blanc.
>
> 🔑 **Et une conséquence de méthode** : lire un résultat dans la même trame que
> la commande n'a pas de sens. C'est la trame **suivante**. Nos mesures qui
> échantillonnent `d_fb_det` sans tenir compte de ce décalage sont à relire.

## 5. Les tâches — et le nombre d'échantillons ⭐

Ne pas confondre **tâches DSP** et **tâches Layer1** (côté ARM) : les deux
s'appellent « tâches » et partagent un espace de noms, mais chaque tâche Layer1
utilise une **séquence** d'une ou plusieurs tâches DSP.

| tâche | ce qu'elle fait | durée | échantillons consommés |
|---|---|---|---|
| **CHECKSUM** | somme de contrôle du code programme du DSP, pour valider patches + ROM | 1 trame | — |
| **PM / AGC** | mesure du niveau reçu, pour piloter le gain de la chaîne Rx analogique. L'ARM choisit **1, 2 ou 3** mesures — c'est ce nombre qui détermine la durée | variable | — |
| **FB** | détection/décodage du burst de fréquence, en mode veille | **max 13 trames** — la 1ʳᵉ est « idle », les **12 suivantes** sont examinées pour le résultat, **dans la page NDB** | — |
| **SB** | détection/décodage du burst de synchronisation, en veille | | **190 échantillons I/Q**, fenêtre de recherche TSC de **50 bits à partir de l'échantillon r39**, corrélation sur les **64 bits complets** (voir `PROM0:7C2C`) |
| **RX NB** | réception de burst normal, en veille et en mode dédié. **À appeler quatre fois** (les 4 bursts) ; les données finales se récupèrent à la **4ᵉ** réponse | | **150 échantillons I/Q**, fenêtre TSC de **10 bits à partir de r68**, corrélation sur **16 bits seulement** (TSC[10..25]) (voir `PROM0:7C27`) |
| **TCH_FB** / **TCH_SB** | burst de fréquence / de synchro, en mode **dédié** | | |

### 🏁 La confrontation, et c'est un défaut de notre modèle

```
le DSP consomme :   RX NB → 150 échantillons     SB → 190 échantillons
nous livrions   :                    148 échantillons  (296 int16, figés en C)
```

**148, c'est la longueur d'un burst GSM en BITS** (3 + 57 + 1 + 26 + 1 + 57 + 3).
Nous livrions donc **exactement un burst à 1 SPS, sans aucune marge de
recherche** — alors que la fenêtre de recherche est justement la raison pour
laquelle le DSP demande davantage.

Conséquence : le DSP lit **au-delà** de ce qu'on dépose — 2 échantillons de trop
pour un NB, **42 pour un SB** — et ce qu'il trouve après est le contenu DARAM
voisin, du bruit périmé, **pile dans la fenêtre de corrélation du SB**.

C'était figé en C (`int16_t iq[296]`) avec un garde `n > 296` qui **refusait**
toute valeur supérieure : aucune variable d'environnement ne pouvait le corriger.
Corrigé le 30/07 — `BSP_IQ_MAX_I16 = 384` (192 échantillons), **défaut inchangé à
296** pour ne rien modifier tant que le test n'est pas fait :

```bash
CALYPSO_BSP_DARAM_LEN=380   # 190 échantillons = ce que la tâche SB demande
```

### Autres conséquences des durées

- **FB dure jusqu'à 13 trames**, et le résultat se lit dans la **NDB** sur les
  **12 trames** suivant la première. Nos runs échantillonnent `d_fb_det` sans
  respecter cette fenêtre — une valeur nulle lue hors fenêtre ne prouve rien.
- **RX NB doit être appelée 4 fois**, et seule la **4ᵉ** réponse porte les
  données. Cela éclaire d'un jour nouveau le rejet firmware qu'on a mesuré :
  `prim_rx_nb.c:80` compare `d_burst_d` à `burst_id` **précisément parce que les
  quatre appels doivent se suivre dans l'ordre 0,1,2,3**. Notre modèle écrit
  `d_burst_d` depuis `@0xb007` et le firmware **lit 2 quand il attend 1** — le
  compteur de burst n'est pas en phase avec la séquence des quatre appels.

## 6. Formats de données — et une validation de notre modèle

| grandeur | codage |
|---|---|
| **Angle** | écart entre l'horloge porteuse reçue et notre horloge issue du LO synthétisé. En **radians**, notation **fx1.15** (virgule fixe 16 bits : 1 bit entier, 15 bits fractionnaires) |
| **SNR** | en **dB**, notation **fx6.10** |
| **Power** | puissance du signal présent à l'entrée bande de base de l'ADC, codée en **1/64 dBm** |
| **TOA** | à la lecture d'un NB : exprimé en **bits entiers**, et semble **toujours positif** |

> ✅ **Le codage de la puissance valide notre modèle de gain** : `calypso_trf6151.c`
> pose `a_pm = (rf_dbm + total_gain) × 64`, c'est-à-dire exactement du **1/64 dBm**.
> Ce n'était jusqu'ici justifié que par la chaîne firmware (`prim_pm.c` :
> `pm = a_pm >> 3`, puis `bb_dbm = pm / 8`) ; la doc matérielle le confirme
> indépendamment.
>
> ⚠️ **TOA positif seulement** : notre modèle clampe le TOA à ±64 qbits autour de
> 23 (`calypso_dsp_shunt.c`). Compatible, mais à revoir si un TOA négatif
> apparaît — ce serait un signe d'erreur, pas une valeur.
>
> ⚠️ **Angle en fx1.15** : la plage utile est donc [−1, +1[ radian. Notre
> `ANGLE_TO_FREQ` côté firmware en dépend ; toute valeur d'angle fabriquée par le
> modèle doit respecter ce format, sous peine d'un AFC absurde.

## 7. Ce que ce document change pour nous — liste actionnable

1. **Livrer 190 échantillons** (`CALYPSO_BSP_DARAM_LEN=380`) et re-mesurer. Le
   code l'autorise depuis le 30/07 ; le défaut reste 296 en attendant le test.
2. **Respecter la fenêtre de 13 trames de la tâche FB** avant de conclure que
   `d_fb_det` reste nul.
3. **Vérifier la séquence des 4 appels RX NB** — c'est probablement la racine du
   `BURST ID 2!=1` qui fait jeter tous les rapports par `prim_rx_nb.c:80`.
4. **Vérifier que les patches DSP sont bien téléchargés** dans notre modèle ; on
   exécute peut-être une ROM non patchée.
5. **Étendre le moniteur mailbox au-delà de `0x0FFF`** — l'API RAM fait
   **8 kWords**, notre fenêtre d'observation en couvre le quart.
6. **Vérifier `ndb.d_tch_mode` bit 11 = 1 (IOTA)** avant tout travail sur l'UL :
   il détermine l'adresse où le DSP attend `BULDATA`.
7. Lire les résultats **à la trame suivante**, jamais dans la même.

---

## 8. Rappel des règles de lecture

`data[]` = ce que le **DSP** a écrit ; `api[]` = ce que l'**hôte** a écrit. Le
**manifeste** est la seule source de vérité sur ce qu'un run a obtenu. Le rendu du
**désassembleur est le premier producteur d'erreurs** du projet (`TODO.md` §0) :
une correspondance *mesurée* vaut, une lecture d'opcode seule ne vaut pas.

L'état chiffré du front natif — ce que le DSP fait réellement, avec ses mesures et
ses rétractations — vit dans `TODO.md` §0bis et suivants.
