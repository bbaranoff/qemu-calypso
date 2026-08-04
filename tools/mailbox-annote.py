#!/usr/bin/env python3
"""
mailbox-annote.py — croise mailbox.log avec le désassembleur c54x.

Le moniteur mailbox (calypso_mailbox.c) note le PC de chaque accès DSP :

    54068145  0  DSP>WR 0x0829 d_burst_d/rp0  0x0002 -> 0x0000  @0xb007

« quelqu'un a écrit d_burst_d » est déjà utile ; « QUELLE instruction l'a
écrit » l'est bien davantage. Cet outil relève tous les PC distincts du
journal, les désassemble, et rend le tableau croisé cellule × instruction.

Exemple de ce que ça donne — un cas réel du 2026-07-29 :
    0xb446  stl *AR1+, A     ← post-incrément : c'est un memset de la read page,
                               pas un bug. Les « 0x0000 -> 0x0000 » répétés sur
                               d_burst_d s'expliquent d'un coup.

Usage :
    tools/mailbox-annote.py [mailbox.log] [--rom ...] [--cellule 0x0829]
"""
import argparse
import collections
import io
import os
import re
import sys

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
import importlib.util

_spec = importlib.util.spec_from_file_location(
    "tic54xdis", os.path.join(ICI, "tic54x-dis.py"))
_dis = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_dis)

# « insn fn sens mot nom ... @0xPC » — le nom peut être vide.
LIGNE = re.compile(
    r"^\s*(\d+)\s+(\d+)\s+(ARM>WR|ARM<RD|DSP>WR|DSP<RD)\s+(0x[0-9a-f]{4})\s+"
    r"(.*?)@(0x[0-9a-f]+)(?:\s+x(\d+))?\s*$")


def charger_roms(chemins):
    """Rend [(base, contenu)] pour chaque ROM lisible."""
    roms = []
    for c in chemins:
        base = _dis.ROMS.get(os.path.basename(c), {}).get("base") \
            if isinstance(_dis.ROMS.get(os.path.basename(c)), dict) else None
        if base is None:
            base = 0xE000 if "PDROM" in c else 0x7000
        try:
            with open(c, "rb") as f:
                roms.append((base, f.read()))
        except OSError:
            pass
    return roms


def desassembler_pc(tab, roms, pc):
    """Première instruction à ce PC, ou None si aucune ROM ne le couvre.

    `_dis.desassembler()` IMPRIME au lieu de rendre : on capture sa sortie. Écrit
    ainsi plutôt qu'en modifiant le désassembleur, pour ne pas toucher un outil
    déjà utilisé ailleurs."""
    import contextlib
    for base, rom in roms:
        n = len(rom) // 2
        if not (base <= pc < base + n):
            continue
        tampon = io.StringIO()
        try:
            with contextlib.redirect_stdout(tampon):
                r = _dis.desassembler(tab, rom, base, pc, pc + 1)
        except Exception:
            continue
        txt = tampon.getvalue().strip().splitlines()
        if txt:
            # « 0xb446  8091  stl *AR1+, A » -> on garde à partir du mnémonique
            l = txt[0].strip()
            parties = l.split(None, 1)
            return parties[1].strip() if len(parties) > 1 else l
        if r:
            return str(r[0])
    return None


def flux(tab, roms, cible):
    """Annote le flux au fil de l'eau. Une ligne d'entrée -> une ligne de sortie.

    Le désassemblage est mis en cache par PC : une boucle serrée ne coûte donc
    qu'une recherche, pas une par ligne. Sortie non tamponnée pour que le pane
    tmux défile en direct."""
    cache = {}
    for l in sys.stdin:
        l = l.rstrip("\n")
        if l.startswith("#") or not l.strip():
            print(l, flush=True)
            continue
        m = LIGNE.match(l)
        if not m:
            print(l, flush=True)
            continue
        _insn, _fn, sens, mot, _reste, pc, _rep = m.groups()
        if cible is not None and int(mot, 16) != cible:
            continue
        if not sens.startswith("DSP"):
            print(l, flush=True)      # côté ARM le contexte est un offset MMIO
            continue
        p = int(pc, 16)
        if p not in cache:
            cache[p] = desassembler_pc(tab, roms, p) or ""
        print("%s   %s" % (l, cache[p]), flush=True)


def main():
    import time
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("journal", nargs="?", default="/tmp/calypso/logs/mailbox.log")
    p.add_argument("--rom", action="append", default=[],
                   help="ROM à utiliser (répétable ; défaut PROM0 + PDROM)")
    p.add_argument("--table", default=None, help="table binutils tic54x-opc.c")
    p.add_argument("--cellule", default=None,
                   help="ne détailler que cette cellule, ex. 0x0829")
    p.add_argument("--top", type=int, default=25, help="nb de lignes (défaut 25)")
    p.add_argument("--flux", action="store_true",
                   help="mode FLUX : lit l'entrée standard et annote CHAQUE ligne "
                        "avec l'instruction qui la produit, au fil de l'eau. "
                        "Prévu pour « tail -F mailbox.log | mailbox-annote.py --flux ». "
                        "Le mode par défaut agrège ; celui-ci déroule.")
    p.add_argument("--fenetre", type=int, default=200000, metavar="N",
                   help="ne lire que les N DERNIÈRES lignes du journal "
                        "(défaut 200000, 0 = tout). Le journal grossit en "
                        "continu : le relire en entier à chaque cycle rend le "
                        "rafraîchissement de plus en plus lent jusqu'à paraître "
                        "figé. Une fenêtre glissante est aussi plus parlante en "
                        "direct — elle dit ce qui se passe MAINTENANT, pas les "
                        "totaux depuis le démarrage.")
    p.add_argument("--suivre", type=float, metavar="SEC", default=None,
                   help="relire en boucle toutes les SEC secondes (live). Le "
                        "moniteur vide son tampon toutes les 1024 lignes, donc "
                        "le journal est exploitable pendant que QEMU tourne.")
    a = p.parse_args()

    if not a.rom:
        a.rom = ["/opt/GSM/calypso_dsp.PROM0.bin", "/opt/GSM/calypso_dsp.PDROM.bin"]
    if not a.table:
        # même emplacement que tic54x-dis.py : la table binutils versée au dépôt.
        a.table = os.path.join(os.path.dirname(ICI),
                               "hw/arm/calypso/doc/opcodes/tic54x-opc.c")
    tab = _dis.charger_table(a.table)
    roms = charger_roms(a.rom)
    if not roms:
        sys.exit("aucune ROM lisible parmi : %s" % ", ".join(a.rom))

    cible = int(a.cellule, 0) if a.cellule else None

    if a.flux:
        flux(tab, roms, cible)
        return

    # (mot, sens, pc) -> occurrences (en tenant compte du « x N » du repliement)
    compte = collections.Counter()
    noms = {}
    try:
        f = io.open(a.journal, encoding="utf-8", errors="replace")
    except OSError as e:
        sys.exit("journal illisible : %s" % e)
    with f:
        if a.fenetre:
            # On se place à la fin moins une estimation généreuse (les lignes
            # font ~70 o), puis on jette la première, forcément tronquée.
            import os as _os
            taille = _os.fstat(f.fileno()).st_size
            saut = taille - a.fenetre * 80
            if saut > 0:
                f.seek(saut)
                f.readline()
        for l in f:
            if l.startswith("#"):
                continue
            m = LIGNE.match(l.rstrip("\n"))
            if not m:
                continue
            _insn, _fn, sens, mot, reste, pc, rep = m.groups()
            if not sens.startswith("DSP"):
                continue              # côté ARM le contexte est un offset MMIO
            mo = int(mot, 16)
            if cible is not None and mo != cible:
                continue
            nom = reste.split()[0] if reste.split() else ""
            if nom and not nom.startswith("0x") and nom not in ("=",):
                noms[mo] = nom
            compte[(mo, sens, int(pc, 16))] += 1 + (int(rep) - 1 if rep else 0)

    if not compte:
        sys.exit("aucun accès DSP trouvé dans %s" % a.journal)

    cache = {}
    if a.suivre:
        os.system("clear")
    print("# croisement mailbox × désassemblage — %s" % a.journal)
    if a.fenetre:
        print("# fenêtre glissante : ~%d dernières lignes (--fenetre 0 pour tout)"
              % a.fenetre)
    print("# %-8s %-14s %-6s %-8s %-9s %s"
          % ("cellule", "nom", "sens", "occurr.", "pc", "instruction"))
    for (mo, sens, pc), n in compte.most_common(a.top):
        if pc not in cache:
            cache[pc] = desassembler_pc(tab, roms, pc)
        instr = cache[pc] or "(hors ROM chargée)"
        print("  0x%04x   %-14s %-6s %8d  0x%04x   %s"
              % (mo, noms.get(mo, ""), sens, n, pc, instr))

    if a.suivre:
        time.sleep(a.suivre)
        sys.argv = [sys.argv[0]] + [x for x in sys.argv[1:]]
        main()


if __name__ == "__main__":
    main()
