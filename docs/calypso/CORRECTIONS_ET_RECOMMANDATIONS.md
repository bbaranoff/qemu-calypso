# Corrections apportées à calypso_trx.c

## Problème principal corrigé

**Fichier:** `calypso_trx.c`  
**Lignes:** 268-274  
**Erreur:** Déclaration de structure dupliquée et mal formée

### Code erroné (avant)
```c
if (gsmtap_fd >= 0) {
    struct {
        struct gsmtap_hdr h;
        uint8_t payload[GSM_BURST_BITS];
    struct {
        struct gsmtap_hdr h;
        uint8_t payload[GSM_BURST_BITS];
    } gsmtap_pkt;

    memset(&pkt, 0, sizeof(pkt));
```

### Code corrigé (après)
```c
if (gsmtap_fd >= 0) {
    struct {
        struct gsmtap_hdr h;
        uint8_t payload[GSM_BURST_BITS];
    } pkt;

    memset(&pkt, 0, sizeof(pkt));
```

### Explication
La déclaration de structure était dupliquée avec une syntaxe incorrecte :
- Première structure sans accolade fermante ni nom de variable
- Deuxième structure qui créait la confusion
- Variable `pkt` utilisée sans être correctement déclarée

Cette erreur causait une cascade d'erreurs car le compilateur considérait que toutes les fonctions suivantes étaient déclarées à l'intérieur de `trx_send_burst`.

---

## Recommandations pour les autres fichiers

### 1. calypso_high.c
**Statut:** ✅ Semble correct  
**Remarque:** Machine de haut niveau bien structurée avec tous les périphériques QOM

### 2. calypso_uart.c
**Statut:** ✅ Correctement implémenté  
**Points positifs:**
- FIFO RX correctement géré
- IRQ bien gérées
- CharBackend bien intégré

### 3. calypso_timer.c
**Statut:** ✅ Bon  
**Remarque:** Timer basique mais fonctionnel

### 4. calypso_spi.c
**Statut:** ✅ Bon  
**Remarque:** SPI + émulation TWL3025 ABB fonctionnelle

### 5. calypso_inth.c
**Statut:** ✅ Bon  
**Remarque:** Contrôleur d'interruption avec priorités

### 6. calypso_socket.c
**Statut:** ⚠️ À vérifier  
**Problèmes potentiels:**
- Thread créé dans `realize()` qui pourrait causer des problèmes
- Pas de gestion de fin de thread propre
- `select()` avec timeout court dans une boucle

**Recommandation:** Envisager d'utiliser le système d'événements QEMU (`qemu_set_fd_handler`) au lieu d'un thread séparé.

### 7. calypso_mb.c et calypso_soc.c
**Statut:** ⚠️ Incomplets  
**Remarque:** Ces fichiers semblent être des ébauches non utilisées. Ils devraient probablement être supprimés ou complétés.

### 8. calypso.c
**Statut:** ✅ Machine minimale fonctionnelle  
**Remarque:** Version simplifiée pour tester, semble complète

---

## Structure de compilation recommandée

Pour compiler votre émulation Calypso, vous devriez avoir :

```
hw/arm/calypso/
├── calypso_high.c      # Machine principale (à utiliser)
├── calypso_trx.c       # ✅ CORRIGÉ - Bridge DSP/TPU/TRX
├── calypso_inth.c      # Contrôleur d'interruption
├── calypso_timer.c     # Timers
├── calypso_uart.c      # UART
├── calypso_spi.c       # SPI + TWL3025
└── calypso.c           # Machine minimale (optionnel)

hw/intc/
└── calypso_inth.c      # Ou ici selon votre organisation

hw/char/
└── calypso_uart.c      # Ou ici selon votre organisation

hw/timer/
└── calypso_timer.c     # Ou ici selon votre organisation

hw/ssi/
└── calypso_spi.c       # Ou ici selon votre organisation
```

---

## Prochaines étapes

1. **Compiler à nouveau** avec `calypso_trx.c` corrigé
2. **Vérifier les warnings** qui pourraient subsister
3. **Tester** avec le firmware OsmocomBB
4. **Ajuster les offsets NDB** si la synchronisation ne fonctionne pas (voir commentaires dans calypso_trx.h)

---

## Notes importantes sur le debugging

### Pour déboguer la synchronisation ARFCN:
```c
// Dans calypso_trx.c, mettre à 1:
#define TRX_DEBUG_SYNC   1
```

### Pour déboguer les accès DSP:
```c
// Pour voir tous les accès à la RAM DSP:
#define TRX_DEBUG_DSP    1
```

### Tester la compilation:
```bash
cd ~/qemu/build
ninja
```

Si d'autres erreurs apparaissent, vérifiez:
- Les includes manquants
- Les types non déclarés
- Les fonctions forward-declared mais pas définies
