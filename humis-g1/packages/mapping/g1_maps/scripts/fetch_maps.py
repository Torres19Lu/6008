#!/usr/bin/env python3
"""Fetch g1_maps assets from Hugging Face, pinned by commit + sha256.

The map state (geometry binaries *.pcd/*.g2o/*.bin + geometry YAML, overlays, and the
global topology.yaml) is NOT committed to git; this script pulls the files listed in
config/maps_manifest.yaml from the HF dataset repo at the pinned revision, verifies
each file's sha256, and restores it to its package-relative path (dest_dir is the
package root, so maps/ and topology/ both land correctly). Idempotent: a file already
present with the correct sha256 is skipped.

Not a ROS node. Run once at install time, before running map-management demos/tests
(see docs/INSTALL.md). For the private HF repo it needs a read token in the ambient
huggingface_hub auth (hf auth login, or the HF_TOKEN env var).
"""
import argparse
import hashlib
import pathlib
import shutil
import sys

import yaml

PKG_ROOT = pathlib.Path(__file__).resolve().parent.parent  # g1_maps/
MANIFEST = PKG_ROOT / "config" / "maps_manifest.yaml"


def sha256_of(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    argparse.ArgumentParser(
        description="Fetch g1_maps assets from Hugging Face (pinned by commit + sha256)."
    ).parse_args()

    if not MANIFEST.exists():
        print(f"ERROR: manifest not found: {MANIFEST}", file=sys.stderr)
        return 1
    spec = yaml.safe_load(MANIFEST.read_text()) or {}
    try:
        repo = spec["hf_repo"]
        revision = spec["hf_revision"]
    except KeyError as exc:
        print(f"ERROR: manifest missing required key {exc}", file=sys.stderr)
        return 1
    repo_type = spec.get("hf_repo_type", "dataset")
    dest_dir = PKG_ROOT / spec.get("dest_dir", "maps")
    files = spec.get("files") or []
    if not files:
        print(f"maps: no files listed in {MANIFEST.name}; nothing to fetch.")
        return 0

    # Import lazily so --help and an offline contract check do not require the dep.
    try:
        from huggingface_hub import hf_hub_download
    except ImportError as exc:
        print(f"ERROR: huggingface_hub not installed ({exc}); "
              f"`pip install huggingface_hub` (or use the conda env).", file=sys.stderr)
        return 1

    dest_dir.mkdir(parents=True, exist_ok=True)
    fetched = skipped = 0
    for entry in files:
        fname = entry["filename"]          # relative path, preserved under dest_dir
        want = entry["sha256"]
        target = dest_dir / fname
        if target.exists() and sha256_of(target) == want:
            skipped += 1
            continue
        try:
            cached = hf_hub_download(repo_id=repo, repo_type=repo_type,
                                     filename=fname, revision=revision)
        except Exception as exc:  # HTTP / auth / offline / bad-revision -> clean exit
            # 401/403: private repo needs a read token (hf auth login / HF_TOKEN).
            # 404: wrong revision, or the LFS object was purged.
            print(f"ERROR: download failed for {fname}: {exc}", file=sys.stderr)
            return 1
        got = sha256_of(pathlib.Path(cached))
        if got != want:
            print(f"ERROR: sha256 mismatch for {fname}: got {got}, want {want}",
                  file=sys.stderr)
            return 1
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(cached, target)
        fetched += 1
        print(f"  fetched {fname}")

    print(f"maps: {fetched} fetched, {skipped} up-to-date, {len(files)} total -> {dest_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
