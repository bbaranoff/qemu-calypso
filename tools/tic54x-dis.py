#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tic54x-dis — désassembleur TMS320C54x piloté par la table binutils.

POURQUOI CET OUTIL. Le firmware DSP du Calypso n'est disponible qu'en binaire
(les PROM*.bin du dépôt). Décoder ses instructions à la main est faisable mais
peu fiable : trois conclusions fausses ont été tirées ainsi en une soirée — un
`LD` pris pour du DP-relatif alors qu'il était en `*(lk)`, un `ST #2` cru sur le
chemin d'exécution alors qu'un `B` le saute, une condition `NEQ` lue à l'envers.

Il ne réimplémente donc AUCUNE table d'opcodes : il lit
`hw/arm/calypso/doc/opcodes/tic54x-opc.c`, la table binutils versée au dépôt,
qui fait autorité — y compris pour le NOMBRE DE MOTS de chaque instruction,
sans lequel tout le décodage aval se décale.

USAGE
    tools/tic54x-dis.py --rom calypso_dsp.PROM0.bin --base 0x7000 0xde80 0xdea0
    tools/tic54x-dis.py --rom calypso_dsp.PROM0.bin --base 0x7000 --count 24 0xddeb
    tools/tic54x-dis.py --list-roms

Les adresses et le nombre de mots s'écrivent en hexa (0x…) ou en décimal.
"""
import argparse, io, os, re, struct, sys

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TABLE  = os.path.join(RACINE, "hw/arm/calypso/doc/opcodes/tic54x-opc.c")

# Cartographie des ROM du dépôt. La base est celle à laquelle le DSP les voit ;
# se tromper de base décale TOUT le désassemblage sans qu'aucune erreur
# n'apparaisse — c'est le piège le plus coûteux ici.
ROMS = {
    "calypso_dsp.PROM0.bin": 0x7000,
    "calypso_dsp.PROM1.bin": None,   # base à confirmer avant usage
    "calypso_dsp.PROM2.bin": None,
    "calypso_dsp.PROM3.bin": None,
}

# ── la table ────────────────────────────────────────────────────────────────
ENTREE = re.compile(
    r'\{\s*"([^"]+)"\s*,\s*(\d+)\s*,[^,]*,[^,]*,\s*(0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\s*,\s*\{([^}]*)\}')

def charger_table(chemin):
    """Rend [(masque, opcode, nom, mots, operandes)], trié du plus spécifique au
    plus large : un masque large capturerait sinon les instructions précises."""
    if not os.path.exists(chemin):
        sys.exit("table binutils introuvable : %s" % chemin)
    src = io.open(chemin, encoding="utf-8", errors="replace").read()
    # on ignore la table parallèle (paroptab) : elle décrit des paires, pas des
    # mots isolés, et la mélanger produit de faux positifs.
    fin = src.find("tic54x_paroptab")
    if fin > 0:
        src = src[:fin]
    tab = []
    for m in ENTREE.finditer(src):
        nom, mots, op, masque, ops = m.group(1), int(m.group(2)), \
            int(m.group(3), 16), int(m.group(4), 16), m.group(5).strip()
        if nom == "???":
            continue
        tab.append((masque, op, nom, mots, ops))
    tab.sort(key=lambda e: bin(e[0]).count("1"), reverse=True)
    return tab

def trouver(tab, mot):
    for masque, op, nom, mots, ops in tab:
        if (mot & masque) == op:
            return nom, mots, ops
    return None, 1, ""

# ── opérandes ───────────────────────────────────────────────────────────────
MOD = {
    0x0: "*AR%d",      0x1: "*AR%d-",      0x2: "*AR%d+",     0x3: "*+AR%d",
    0x4: "*AR%d-0B",   0x5: "*AR%d-0",     0x6: "*AR%d+0",    0x7: "*AR%d+0B",
    0x8: "*AR%d-%%",   0x9: "*AR%d-0%%",   0xA: "*AR%d+%%",   0xB: "*AR%d+0%%",
}
COND = {  # cc & 0x07 quand cc & 0x40 (test accumulateur) — cf. SPRU172C
    0x5: "EQ", 0x4: "NEQ", 0x3: "LT", 0x7: "LEQ", 0x6: "GT", 0x2: "GEQ",
}

def decrire_smem(mot, lk):
    """Le champ Smem est l'octet bas. bit7=0 : direct (DP-relatif) ; bit7=1 :
    indirect. Le mode 0xF est `*(lk)` — adresse ABSOLUE dans le mot suivant,
    et c'est précisément celui qu'on décode de travers à la main."""
    bas = mot & 0xFF
    if not (bas & 0x80):
        return "@0x%02x" % (bas & 0x7F), False        # direct : dma = DP<<7 | off
    mod, ar = (bas >> 3) & 0x0F, bas & 0x07
    if mod in MOD:
        return MOD[mod] % ar, False
    if mod == 0xC:  return "*AR%d(0x%04x)"  % (ar, lk), True
    if mod == 0xD:  return "*+AR%d(0x%04x)" % (ar, lk), True
    if mod == 0xE:  return "*+AR%d(0x%04x)%%" % (ar, lk), True
    if mod == 0xF:  return "*(0x%04x)" % lk, True     # ABSOLU
    return "?mod%x" % mod, False

def decrire(nom, ops, mot, lk):
    """Rend (texte des opérandes, mot long consommé PAR Smem).

    Seul le Smem en mode 0xC..0xF consomme un mot NON compté par la table : la
    cible d'un branchement (pmad) et une constante (lk) y sont déjà incluses.
    Confondre les deux allonge les branchements d'un mot et décale tout."""
    o, smem_lk = [], False
    if "OP_Smem" in ops or "OP_Sind" in ops:
        t, u = decrire_smem(mot, lk); o.append(t); smem_lk = u
    if "OP_xpmad" in ops or "OP_pmad" in ops:
        o.append("0x%04x" % lk)
    if "OP_lk" in ops or "OP_LK" in ops:
        o.append("#0x%04x" % lk)
    if "OP_CC" in ops or nom in ("bc", "bcd", "cc", "ccd", "rc", "rcd", "xc"):
        cc = mot & 0xFF
        if cc & 0x40:
            acc = "B" if (cc & 0x08) else "A"
            o.append("%s%s" % (acc, COND.get(cc & 0x07, "cc%02x" % cc)))
        elif cc:
            o.append("cc0x%02x" % cc)
    if "OP_SRC" in ops or "OP_DST" in ops:
        o.append("B" if (mot & 0x0100) else "A")
    return ", ".join(o), smem_lk

# ── désassemblage ───────────────────────────────────────────────────────────
def lire(rom, base, adr):
    o = (adr - base) * 2
    if o < 0 or o + 1 >= len(rom):
        return None
    return struct.unpack("<H", rom[o:o+2])[0]

def desassembler(tab, rom, base, debut, fin):
    adr = debut
    while adr < fin:
        mot = lire(rom, base, adr)
        if mot is None:
            print("  0x%04x  <hors ROM>" % adr); return
        suiv = lire(rom, base, adr + 1) or 0
        nom, mots, ops = trouver(tab, mot)
        if nom is None:
            print("  0x%04x  %04x                  .word 0x%04x" % (adr, mot, mot))
            adr += 1
            continue
        texte, lk_pris = decrire(nom, ops, mot, suiv)
        # LONGUEUR RÉELLE. La table donne le nombre de mots de l'instruction,
        # mais un opérande Smem en mode 0xC..0xF porte SA PROPRE constante longue
        # dans un mot supplémentaire. L'oublier décale tout le désassemblage en
        # aval sans qu'aucune erreur n'apparaisse — on lit alors des
        # instructions fantômes (constaté : « sub *AR4-, B » là où il n'y a que
        # l'adresse 0x098c).
        reels = mots + (1 if lk_pris else 0)
        extras = [lire(rom, base, adr + i) for i in range(1, reels)]
        brut = " ".join(["%04x" % mot] + ["%04x" % (e or 0) for e in extras])
        # Quand l'instruction porte À LA FOIS une constante et une adresse
        # longue, on affiche les mots supplémentaires tels quels plutôt que de
        # deviner leur ordre : mieux vaut une lecture brute qu'une affirmation
        # fausse.
        if reels > 2:
            texte = "%s   ; mots suppl. %s" % (
                texte, " ".join("0x%04x" % (e or 0) for e in extras))
        print("  0x%04x  %-16s %-7s %s" % (adr, brut, nom, texte))
        adr += reels

def main():
    p = argparse.ArgumentParser(description="Désassembleur c54x (table binutils du dépôt)")
    p.add_argument("--rom",   help="fichier ROM (défaut : calypso_dsp.PROM0.bin)")
    p.add_argument("--base",  help="adresse de base de la ROM (défaut : celle de la carte)")
    p.add_argument("--count", help="nombre de MOTS à désassembler depuis l'adresse")
    p.add_argument("--table", default=TABLE, help="table binutils")
    p.add_argument("--list-roms", action="store_true", help="cartographie des ROM connues")
    p.add_argument("adresses", nargs="*", help="début [fin]")
    a = p.parse_args()

    if a.list_roms:
        print("ROM connues (base = adresse vue par le DSP) :")
        for n, b in ROMS.items():
            c = os.path.join(RACINE, n)
            etat = "%d mots" % (os.path.getsize(c)//2) if os.path.exists(c) else "absente"
            print("  %-28s base=%s  %s" % (n, ("0x%04x" % b) if b else "À CONFIRMER", etat))
        return 0

    if not a.adresses:
        p.error("donnez au moins une adresse de début")
    nb = lambda s: int(s, 0)
    rom_nom = a.rom or "calypso_dsp.PROM0.bin"
    chemin  = rom_nom if os.path.exists(rom_nom) else os.path.join(RACINE, rom_nom)
    if not os.path.exists(chemin):
        sys.exit("ROM introuvable : %s" % rom_nom)
    base = nb(a.base) if a.base else ROMS.get(os.path.basename(chemin))
    if base is None:
        sys.exit("base inconnue pour %s — donnez --base (une base fausse décale "
                 "TOUT le désassemblage en silence)" % os.path.basename(chemin))

    rom = io.open(chemin, "rb").read()
    tab = charger_table(a.table)
    debut = nb(a.adresses[0])
    if a.count:      fin = debut + nb(a.count)
    elif len(a.adresses) > 1: fin = nb(a.adresses[1])
    else:            fin = debut + 16

    print("# %s  base=0x%04x  %d entrées de table" %
          (os.path.basename(chemin), base, len(tab)))
    desassembler(tab, rom, base, debut, fin)
    return 0

if __name__ == "__main__":
    sys.exit(main())
