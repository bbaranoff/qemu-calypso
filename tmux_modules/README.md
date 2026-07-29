# tmux_modules — une disposition par profil

`run_modules/35-tmux.sh` ne décrit plus aucune fenêtre : il charge
`_commun.sh` puis le fichier du profil courant, et appelle `tmux_layout`.

```
_commun.sh    couleurs, _w / _split / _tail, fenêtres coeur/voix/shell, habillage
calypso.sh    un téléphone émulé (QEMU) + le cœur
hybrid.sh     DEUX téléphones : MS#1 sur QEMU, MS#2 sur fake_trx — un seul cœur
faketrx.sh    pas de QEMU : téléphone logiciel, radio simulée
defaut.sh     repli (profil « core », ou tout profil sans fichier dédié)
```

## Ajouter un profil

Créer `<profil>.sh` définissant trois choses :

| | |
|---|---|
| `TMUX_FENETRE_PREMIERE` | nom de la fenêtre créée avec la session |
| `tmux_layout_premiere()` | la commande qu'elle exécute |
| `tmux_layout()` | toutes les autres fenêtres, via `_w` / `_split` |

Rien d'autre à modifier : le chargeur trouve le fichier par le nom du profil, et
retombe sur `defaut.sh` s'il n'existe pas.

## Règles

- **La couleur porte le rôle**, pas la position — voir l'en-tête de `_commun.sh`.
  Quatre `tail` côte à côte sont indiscernables sans elle.
- **Les panes du cœur suivent `/var/log/osmocom/osmo-*.log`**, les journaux des
  SERVICES. Suivre `$LOG_DIR/mod/*.log` ne montrerait que les traces de
  démarrage des modules — l'erreur qui a rendu ces fenêtres inutiles longtemps.
- **`tail -F`**, jamais `-f` : le journal peut ne pas exister encore, et le
  garde-fou de `40-qemu` tronque `qemu.log` au-delà du plafond.
- **`|| sleep infinity`** en repli : sans lui, un journal absent ferait mourir le
  pane et refermerait la fenêtre en emportant les autres.
- Une disposition ne lance **aucun service**. tmux ne sert qu'à REGARDER ; sa
  session est cosmétique et son absence n'empêche rien de tourner.
