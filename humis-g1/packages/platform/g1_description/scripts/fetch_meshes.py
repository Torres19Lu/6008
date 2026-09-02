#!/usr/bin/env python3
"""Fetch g1_description meshes from Hugging Face, pinned by commit + sha256.

The STL visual meshes are NOT committed to git; this script pulls them from the
HF dataset repo at the revision pinned in config/meshes_manifest.yaml, verifies
each file's sha256, and places it in the package's meshes/ directory. Idempotent:
a file already present with the correct sha256 is skipped.

Not a ROS node. Run once at install time, before building / launching RViz
(see docs/INSTALL.md). For the private HF repo it needs a read token in the
ambient huggingface_hub auth (hf auth login, or the HF_TOKEN env var).
"""
import hashlib
import pathlib
import shutil
import sys

import yaml
from huggingface_hub import hf_hub_download
from huggingface_hub.errors import HfHubHTTPError

PKG_ROOT = pathlib.Path(__file__).resolve().parent.parent  # g1_description/
MANIFEST = PKG_ROOT / "config" / "meshes_manifest.yaml"


def sha256_of(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    spec = yaml.safe_load(MANIFEST.read_text())
    repo = spec["hf_repo"]
    repo_type = spec.get("hf_repo_type", "dataset")
    revision = spec["hf_revision"]
    dest_dir = PKG_ROOT / spec.get("dest_dir", "meshes")
    dest_dir.mkdir(parents=True, exist_ok=True)
    files = spec["files"]

    # Flat destination: basenames must be unique or a download would clobber.
    basenames = [pathlib.Path(e["filename"]).name for e in files]
    if len(set(basenames)) != len(basenames):
        print("ERROR: manifest has basename collisions; cannot flatten.", file=sys.stderr)
        return 1

    fetched = skipped = 0
    for entry in files:
        fname = entry["filename"]
        want = entry["sha256"]
        target = dest_dir / pathlib.Path(fname).name
        if target.exists() and sha256_of(target) == want:
            skipped += 1
            continue
        try:
            cached = hf_hub_download(repo_id=repo, repo_type=repo_type,
                                     filename=fname, revision=revision)
        except HfHubHTTPError as exc:
            # 401/403: private repo needs a read token (hf auth login / HF_TOKEN).
            # 404: wrong revision, or the LFS object was purged.
            print(f"ERROR: download failed for {fname}: {exc}", file=sys.stderr)
            return 1
        got = sha256_of(pathlib.Path(cached))
        if got != want:
            print(f"ERROR: sha256 mismatch for {fname}: got {got}, want {want}", file=sys.stderr)
            return 1
        shutil.copyfile(cached, target)
        fetched += 1
        print(f"  fetched {target.name}")

    print(f"meshes: {fetched} fetched, {skipped} up-to-date, {len(files)} total -> {dest_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
