#!/usr/bin/env python3
# ndb-offsets.py — extrait les offsets des champs de T_NDB_MCU_DSP du DWARF du
# firmware, avec binutils-arm-none-eabi. Sortie : des lignes `champ=0xNNN`.
#
# POURQUOI CET OUTIL EXISTE
# -------------------------
# Les offsets NDB (a_cd, a_fd, a_dd_0, a_cu, a_fu, a_du_1...) etaient des
# #define dans calypso_dsp_internal.h. Un #define ne peut pas savoir que le
# firmware a ete recompile : si la struct bouge, QEMU ecrit A COTE, en silence,
# et le symptome apparait tres loin en aval (2026-06-02 : a_cd suppose a 0x1DC
# -> num_biterr=0xff + CRC fail, cherche pendant des jours). Le seul juge est le
# binaire qui tourne ; on le lit donc au lieu de le supposer.
#
# CE QUE CE N'EST PAS : un parseur DWARF. On delegue a readelf, et on ne fait
# que suivre les liens entre DIE.
#
# PIEGE DU DUMP : `T_NDB_MCU_DSP` est un DW_TAG_typedef, pas une structure. La
# structure qui SUIT immediatement le typedef dans le dump en est une autre
# (byte_size 292, verifie). Il faut suivre le DW_AT_type du typedef vers l'offset
# de DIE de la vraie structure. Prendre "la structure d'apres" donnerait des
# offsets plausibles et faux.
import re, subprocess, sys, os

TYPEDEF = os.environ.get("CALYPSO_NDB_TYPEDEF", "T_NDB_MCU_DSP")
# Champs qui interessent le shunt. On sort tout ce qu'on trouve parmi ceux-la ;
# un champ absent est signale par l'appelant, pas invente ici.
WANTED = ["a_cd", "a_fd", "a_dd_0", "a_dd_1", "a_cu", "a_fu", "a_du_0", "a_du_1",
          "d_fb_det", "a_sync_demod", "d_tch_mode"]

# ' <1><dcc8>: Abbrev Number: 22 (DW_TAG_typedef)'
RE_DIE = re.compile(r"^\s*<(\d+)><([0-9a-f]+)>:\s+Abbrev Number:\s+\d+\s+\(([A-Za-z_]+)\)")
RE_NAME = re.compile(r"DW_AT_name\s*:.*?:\s*(\S+)\s*$")
RE_NAME_DIRECT = re.compile(r"DW_AT_name\s*:\s*(\S+)\s*$")
RE_TYPE = re.compile(r"DW_AT_type\s*:\s*<0x([0-9a-f]+)>")
RE_LOC = re.compile(r"DW_AT_data_member_location.*?DW_OP_plus_uconst:\s*(\d+)")
RE_LOC_PLAIN = re.compile(r"DW_AT_data_member_location\s*:\s*(\d+)\s*$")
RE_SIZE = re.compile(r"DW_AT_byte_size\s*:\s*(\d+)")


def die_name(line):
    m = RE_NAME.search(line) or RE_NAME_DIRECT.search(line)
    return m.group(1) if m else None


def extract(elf, readelf):
    try:
        out = subprocess.run([readelf, "--debug-dump=info", elf],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             check=True).stdout.decode("utf-8", "replace")
    except (OSError, subprocess.CalledProcessError) as e:
        print("# ERREUR: %s sur %s : %s" % (readelf, elf, e), file=sys.stderr)
        return None

    lines = out.splitlines()

    # --- passe 1 : typedef -> offset de DIE de la structure cible -------------
    target = None
    cur_tag = None
    cur_name = None
    for ln in lines:
        m = RE_DIE.match(ln)
        if m:
            if cur_tag == "DW_TAG_typedef" and cur_name == TYPEDEF and target:
                break
            cur_tag, cur_name, pending = m.group(3), None, None
            continue
        if cur_tag != "DW_TAG_typedef":
            continue
        n = die_name(ln)
        if n:
            cur_name = n
        t = RE_TYPE.search(ln)
        if t and cur_name == TYPEDEF:
            target = t.group(1)
    if cur_tag == "DW_TAG_typedef" and cur_name == TYPEDEF and target:
        pass
    if not target:
        print("# ERREUR: typedef %s introuvable dans le DWARF" % TYPEDEF, file=sys.stderr)
        return None

    # --- passe 2 : membres de la structure a l'offset `target` ---------------
    res, size = {}, None
    in_struct = False
    struct_level = None
    member_name = None
    for ln in lines:
        m = RE_DIE.match(ln)
        if m:
            level, off, tag = int(m.group(1)), m.group(2), m.group(3)
            if not in_struct:
                if off == target and tag == "DW_TAG_structure_type":
                    in_struct, struct_level = True, level
                continue
            # dans la structure : on sort des qu'on remonte au niveau du parent
            if level <= struct_level:
                break
            member_name = None if tag != "DW_TAG_member" else ""
            continue
        if not in_struct:
            continue
        s = RE_SIZE.search(ln)
        if s and size is None:
            size = int(s.group(1))
        if member_name is None:
            continue
        n = die_name(ln)
        if n:
            member_name = n
            continue
        loc = RE_LOC.search(ln) or RE_LOC_PLAIN.search(ln)
        if loc and member_name:
            res[member_name] = int(loc.group(1))
    return res, size


def main():
    if len(sys.argv) < 2:
        print("usage: ndb-offsets.py <firmware.elf> [readelf]", file=sys.stderr)
        return 2
    elf = sys.argv[1]
    readelf = sys.argv[2] if len(sys.argv) > 2 else \
        os.environ.get("CALYPSO_READELF", "arm-none-eabi-readelf")
    got = extract(elf, readelf)
    if not got:
        return 1
    res, size = got
    if size is not None:
        print("sizeof=0x%x" % size)
    missing = [w for w in WANTED if w not in res]
    for w in WANTED:
        if w in res:
            print("%s=0x%x" % (w, res[w]))
    if missing:
        # signale, ne fabrique pas : l'appelant decide quoi faire d'un manque
        print("# ABSENTS: %s" % " ".join(missing), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
