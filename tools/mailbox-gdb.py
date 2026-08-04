#!/usr/bin/env python3
"""
mailbox-gdb.py — lire la mailbox ARM↔DSP EN DIRECT via le gdbstub de QEMU.

Complément du moniteur : `mailbox.log` est un HISTORIQUE, ceci est l'état
INSTANTANÉ, à la demande.

⚠️ Se connecter au gdbstub ARRÊTE le guest le temps de la lecture. Sur une pile
   GSM temps réel, le mobile perdra sa synchro. Inspection ponctuelle, pas un
   mode de travail continu — pour le continu, c'est mailbox.log.

⚠️ Ne montre QUE la fenêtre API RAM (mots DSP 0x0800..0x0FFF), la seule que l'ARM
   voie. Les cellules INTERNES du DSP (0x3f92, 0x5a00, 0x43d8…) ne sont PAS
   accessibles ici : elles vivent dans data[] côté modèle, hors de portée du
   guest. Pour celles-là : CALYPSO_MAILBOX_CELLS=0x3f92,0x5a00 dans le moniteur.

⚠️ `gdb` x86 ne parle pas ARM (« Undefined item: arm », « Truncated register 16
   in remote 'g' packet ») — il faut gdb-multiarch.

Adressage : ARM = 0xFFD00000 + (mot_DSP - 0x0800) * 2

Usage :
    tools/mailbox-gdb.py                   les cellules usuelles
    tools/mailbox-gdb.py 0x0804 0x08f8     des mots DSP précis
    tools/mailbox-gdb.py --port 1234
"""
import argparse
import os
import subprocess
import sys
import tempfile

BASE_ARM = 0xFFD00000
BASE_DSP = 0x0800

USUELLES = [
    (0x0804, "d_task_md/wp0"), (0x0818, "d_task_md/wp1"),
    (0x0810, "d_ctrl_system"),
    (0x0828, "d_task_d/rp0"),  (0x0829, "d_burst_d/rp0"),
    (0x083c, "d_task_d/rp1"),  (0x083d, "d_burst_d/rp1"),
    (0x08d5, "etat_dsp"),      (0x08e2, "d_dsp_page"),
    (0x08f8, "d_fb_det"),      (0x08f9, "d_fb_mode"),
    (0x08fa, "a_sync_TOA"),    (0x08fb, "a_sync_PM"),
    (0x08fc, "a_sync_ANGLE"),  (0x08fd, "a_sync_SNR"),
]


def adresse_arm(mot):
    return BASE_ARM + (mot - BASE_DSP) * 2


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mots", nargs="*", help="mots DSP à lire, ex. 0x0804")
    p.add_argument("--port", default=os.environ.get("CALYPSO_GDB_PORT", "1234"))
    p.add_argument("--elf", default=os.environ.get(
        "CALYPSO_GDB_ELF",
        "/opt/GSM/osmo-qemu-calypso/firmware/compal_e88/layer1.highram.elf"))
    p.add_argument("--gdb", default="gdb-multiarch")
    a = p.parse_args()

    cellules = [(int(m, 0), "(demandée)") for m in a.mots] if a.mots else USUELLES

    lignes = ["set pagination off", "set confirm off"]
    if os.path.exists(a.elf):
        lignes.append("file %s" % a.elf)
    lignes.append("target remote :%s" % a.port)
    # ⚠️ PAS d'argument CHAÎNE dans le printf de gdb : il devrait allouer la
    # chaîne DANS la cible, et rend « evaluation of this expression requires the
    # program to have a function "malloc" ». Les libellés vont donc dans le
    # format lui-même ; seule la valeur lue reste un argument.
    lignes.append(r'printf "  PC ARM                     ->  0x%08x\n", $pc')
    for mot, nom in cellules:
        etiquette = "%-16s 0x%04x" % (nom, mot)
        lignes.append(
            'printf "  %s  ->  0x%%04x\\n", *(unsigned short*)0x%08x'
            % (etiquette, adresse_arm(mot)))

    lignes.append("detach")

    with tempfile.NamedTemporaryFile("w", suffix=".gdb", delete=False) as f:
        f.write("\n".join(lignes) + "\n")
        script = f.name

    sys.stderr.write("\n  mailbox ARM<->DSP — instantané via gdbstub :%s\n"
                     "  (le guest est ARRÊTÉ pendant la lecture)\n\n" % a.port)
    try:
        r = subprocess.run([a.gdb, "-q", "-batch", "-x", script],
                           capture_output=True, text=True, timeout=60)
    except FileNotFoundError:
        sys.exit("  %s introuvable — le gdb x86 ne parle pas ARM, il faut "
                 "gdb-multiarch" % a.gdb)
    except subprocess.TimeoutExpired:
        sys.exit("  délai dépassé — le gdbstub :%s répond-il ?" % a.port)
    finally:
        os.unlink(script)

    vues = [l for l in r.stdout.splitlines() if l.startswith("  ") and "->" in l]
    if not vues:
        sys.stderr.write(r.stdout[-800:] + r.stderr[-800:] + "\n")
        sys.exit("  aucune cellule lue — QEMU tourne-t-il avec -gdb tcp::%s ?"
                 % a.port)
    print("\n".join(vues))
    print()


if __name__ == "__main__":
    main()
