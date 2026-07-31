/*
 * calypso_mailbox.h — moniteur COMPLET de la mailbox ARM <-> DSP
 *
 * [2026-07-29] Pourquoi ce module existe.
 *
 *   La mailbox (API RAM, mots DSP 0x0800..0x0FFF) est le SEUL point de contact
 *   entre l'ARM et le c54x. Tout ce qu'on a passé la journée à chercher — quelle
 *   tâche est commandée, qui écrit d_burst_d, pourquoi le DSP ne voit pas la
 *   tâche 24, quelle cellule le dispatcher interroge — est un événement de cette
 *   frontière. Jusqu'ici on l'observait avec des sondes ponctuelles, écrites une
 *   par une, chacune câblée en dur sur UNE adresse ou UNE valeur :
 *
 *     - TASKGO / FBCALL  : câblées sur d_task_md == 5. Quand l'ARM a enfin
 *                          commandé ALLC (24), on était AVEUGLES dessus.
 *     - ARM-WRITE-0810   : une seule cellule.
 *     - WATCH_WR_ADDR    : une adresse (étendue à 8 aujourd'hui, dans l'urgence).
 *
 *   À chaque question nouvelle il fallait écrire, compiler et relancer une sonde
 *   de plus — et une sonde absente rend zéro, ce qui ressemble à une réponse.
 *   Neuf conclusions fausses en une journée sont venues de là.
 *
 *   Ce module trace TOUT le flux, dans les deux sens, tout le temps.
 *
 * SORTIE SÉPARÉE, et c'est délibéré : le moniteur écrit dans SON fichier, pas
 * sur stderr. Un flux complet sur stderr noierait qemu.log et tronquerait les
 * autres sondes — exactement l'accident du matin (480 898 lignes, 35 Mo, les
 * traces voisines effacées). Ici les deux journaux sont indépendants.
 *
 * FORMAT UNIFORME, pour que deux runs soient DIFFÉRENTIABLES. C'est le but :
 * capturer une séquence en `shunt_legit` (qui marche, donc exerce la vraie
 * séquence de commande) puis en natif, et lire le delta — le delta EST la liste
 * de ce qui n'est pas câblé.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_MAILBOX_H
#define CALYPSO_MAILBOX_H

#include <stdint.h>

/* Sens de l'accès, du point de vue de la mailbox. */
typedef enum {
    MBX_ARM_WR = 0,   /* l'ARM écrit   (commande) */
    MBX_ARM_RD,       /* l'ARM lit     (résultat) */
    MBX_DSP_WR,       /* le DSP écrit  (résultat) */
    MBX_DSP_RD,       /* le DSP lit    (commande) */
} CalypsoMbxSens;

/* Testé en ligne sur les chemins CHAUDS (data_read / data_write du c54x sont
 * appelés à chaque instruction) : quand le moniteur est éteint, le coût se
 * réduit à la lecture d'un int.
 *
 * TROIS états, et c'est nécessaire : -1 = pas encore initialisé, 0 = éteint,
 * 1 = actif. Première version : le drapeau démarrait à 0, donc l'enveloppe en
 * ligne n'appelait jamais calypso_mbx_evt(), donc calypso_mbx_init() n'était
 * jamais atteint, donc le drapeau restait à 0 — aucun fichier n'était créé.
 * Avec -1 le premier accès passe, l'init tranche, et les suivants sont filtrés. */
extern int calypso_mbx_actif;

/* Ouvre le fichier et lit la configuration. Idempotent. Appelé paresseusement
 * au premier événement, donc rien à ordonnancer au démarrage. */
void calypso_mbx_init(void);

/* Enregistre un événement. `avant` n'a de sens que pour les écritures (sinon 0).
 * `ctx` = PC du DSP, ou offset MMIO côté ARM. */
void calypso_mbx_evt(CalypsoMbxSens sens, uint16_t mot, uint16_t val,
                     uint16_t avant, uint32_t ctx, uint32_t fn, uint32_t insn);

/* Enveloppes en ligne : le test du drapeau évite l'appel quand c'est éteint. */
static inline void calypso_mbx(CalypsoMbxSens sens, uint16_t mot, uint16_t val,
                               uint16_t avant, uint32_t ctx, uint32_t fn,
                               uint32_t insn)
{
    if (calypso_mbx_actif) {
        calypso_mbx_evt(sens, mot, val, avant, ctx, fn, insn);
    }
}

#endif /* CALYPSO_MAILBOX_H */
