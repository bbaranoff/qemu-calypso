#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
extract_sources.py
------------------
Extrait d'un dossier (recursivement) uniquement les fichiers dont l'extension
fait partie d'une liste donnee. Par defaut : py, md, sh, c, h, d.

Modes :
  - copie vers un dossier de sortie en conservant l'arborescence (defaut)
  - --flat   : copie tout a plat dans le dossier de sortie
  - --zip F  : ecrit un .zip au lieu de copier
  - --list   : n'ecrit rien, affiche juste les fichiers trouves (dry-run)

Exemples :
  python3 extract_sources.py ./mon_repo
  python3 extract_sources.py ./mon_repo -o /tmp/extrait
  python3 extract_sources.py ./mon_repo --flat
  python3 extract_sources.py ./mon_repo --zip sources.zip
  python3 extract_sources.py ./mon_repo --ext py,md --list
"""

import os
import sys
import shutil
import zipfile
import argparse
from pathlib import Path

DEFAULT_EXTS = ["py", "md", "sh", "c", "h", "d"]
# Dossiers ignores par defaut (bruit / non source).
SKIP_DIRS = {".git", ".hg", ".svn", "__pycache__", "node_modules",
             ".venv", "venv", ".mypy_cache", ".pytest_cache", "build", "dist"}


def normalize_exts(raw):
    """'py,.md, SH' -> {'.py', '.md', '.sh'} (insensible a la casse)."""
    exts = set()
    for part in raw:
        for e in str(part).split(","):
            e = e.strip().lower().lstrip(".")
            if e:
                exts.add("." + e)
    return exts


def find_files(src, exts, include_hidden=False):
    """Genere les chemins des fichiers correspondant aux extensions voulues."""
    src = Path(src)
    for root, dirs, files in os.walk(src):
        # Elagage des dossiers a ignorer (modif in-place de `dirs`).
        dirs[:] = [d for d in dirs
                   if d not in SKIP_DIRS
                   and (include_hidden or not d.startswith("."))]
        for name in files:
            if not include_hidden and name.startswith("."):
                continue
            if Path(name).suffix.lower() in exts:
                yield Path(root) / name


def unique_dest(dest_dir, name):
    """Evite les collisions de noms en mode --flat (ajoute _1, _2, ...)."""
    target = dest_dir / name
    if not target.exists():
        return target
    stem, suffix = Path(name).stem, Path(name).suffix
    i = 1
    while True:
        cand = dest_dir / f"{stem}_{i}{suffix}"
        if not cand.exists():
            return cand
        i += 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", help="Dossier source a parcourir")
    ap.add_argument("-o", "--output", default="extracted",
                    help="Dossier de sortie (defaut: ./extracted)")
    ap.add_argument("--ext", default=",".join(DEFAULT_EXTS),
                    help="Extensions a extraire, separees par des virgules "
                         f"(defaut: {','.join(DEFAULT_EXTS)})")
    ap.add_argument("--flat", action="store_true",
                    help="Copier tous les fichiers a plat (sans arborescence)")
    ap.add_argument("--zip", dest="zip_path", default=None,
                    help="Ecrire un .zip a ce chemin au lieu de copier")
    ap.add_argument("--list", action="store_true",
                    help="Lister seulement, ne rien ecrire (dry-run)")
    ap.add_argument("--include-hidden", action="store_true",
                    help="Inclure les fichiers/dossiers caches et .git, etc.")
    args = ap.parse_args()

    src = Path(args.source)
    if not src.is_dir():
        sys.exit(f"Erreur : '{src}' n'est pas un dossier.")

    exts = normalize_exts([args.ext])
    files = sorted(find_files(src, exts, args.include_hidden))

    if not files:
        print(f"Aucun fichier {sorted(exts)} trouve dans {src}.")
        return

    # Mode liste : on affiche et on sort.
    if args.list:
        for f in files:
            print(f)
    elif args.zip_path:
        with zipfile.ZipFile(args.zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
            for f in files:
                arc = f.name if args.flat else f.relative_to(src)
                zf.write(f, arcname=str(arc))
        print(f"Archive ecrite : {args.zip_path}")
    else:
        out = Path(args.output)
        out.mkdir(parents=True, exist_ok=True)
        for f in files:
            if args.flat:
                dest = unique_dest(out, f.name)
            else:
                dest = out / f.relative_to(src)
                dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(f, dest)
        print(f"{len(files)} fichier(s) copie(s) vers {out}/")

    # Recapitulatif par extension.
    by_ext = {}
    for f in files:
        by_ext[f.suffix.lower()] = by_ext.get(f.suffix.lower(), 0) + 1
    summary = ", ".join(f"{ext}:{n}" for ext, n in sorted(by_ext.items()))
    print(f"Total : {len(files)} fichier(s)  ({summary})")


if __name__ == "__main__":
    main()
