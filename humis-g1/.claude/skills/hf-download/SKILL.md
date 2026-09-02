---
name: hf-download
description: Pull files from a Hugging Face Hub repository at a pinned revision into the local cache or a chosen target directory. Wraps `huggingface_hub.snapshot_download` as the universal one-call primitive (parallel batch with glob filtering, covers single-file via narrow filter, whole-repo via default, supports both cache-only and local-dir output), and falls back to `hf_hub_download` per-file when the consumer needs per-file sha256 verification or conditional skip logic. Interactive invocations gate the download via `AskUserQuestion` (`download` / `abort`) with the planned call (including any local files that would be overwritten by a `local_dir=` target) rendered inline; the scaffolded `fetch_weights.py` reference implementation runs unattended in CI / robot bootstrap and intentionally contains no prompts (the gate sits one level up at scaffolding time in `hf-setup` Phase 6b). Use whenever the user needs to fetch HF Hub artifacts and the repo plus auth are already configured: "download this model from huggingface", "pull these weights to my robot", "get this dataset from HF", "CI needs the v2 model checkpoint", "fetch the elevator detector", "snapshot the whole repo locally", or any combination. Also use when wiring a manifest-pinned consumer (the `fetch_weights.py` pattern documented in `hf-setup` Phase 6): the per-file `hf_hub_download` loop with sha256 verify is the canonical production implementation. For first-time setup (no HF account, no repo, not logged in for private repos), invoke `hf-setup` instead and return here at Phase 6. CLI `hf download` and library auto-loaders like `AutoModel.from_pretrained` are special-case shells around these primitives; this skill is the canonical answer for any non-trivial download because `snapshot_download` is the only single call that parallel-fetches N files with glob filtering and version pinning while supporting both cache-only and local-dir modes.
---

# Goal

Pull files from a Hugging Face Hub repository at a pinned revision into the local cache or a chosen target directory. Use `snapshot_download` as the universal one-call primitive (parallel batch, glob filtering, cache or local-dir output) and drop to `hf_hub_download` per file only when per-file sha256 verification or conditional skip is required.

# When NOT to use

* HF is not yet configured (no account, no repo, not logged in for private repos): invoke `hf-setup` first, then return here.
* The user only wants to load a model into a runtime library (`transformers.AutoModel.from_pretrained`, `diffusers.DiffusionPipeline.from_pretrained`): those wrappers call `snapshot_download` internally and instantiate the model in one step. Use them when the goal is "model in memory", not "files on disk".
* The user wants a one-off click-through download: HF web UI's file page has a download button.

# Prerequisites

For public repos, no auth is needed; `snapshot_download` and `hf_hub_download` work anonymously. For private repos, verify auth first:

```bash
hf auth whoami --format agent
```

Expected: `user=<name> orgs=<a,b,c>`. If `Invalid user token` or no output, the user is not logged in; for first-time auth defer to `hf-setup` Phase 4, otherwise run `hf auth login --force` to refresh.

The skill needs four concrete inputs:

1. **`repo_id`** in `<owner>/<name>` form.
2. **`repo_type`** (`model`, `dataset`, or `space`; defaults to `model` when not stated).
3. **`revision`**: the full 40-character commit SHA, never a branch name or short hash. For manifest-pinned consumers the revision is in the manifest; otherwise ask or retrieve from HF Hub's commit list.
4. **What to download**: a filename, a glob, a subdirectory, or "all". Confirm via `AskUserQuestion` when ambiguous.

Never guess the revision. A wrong revision either 404s loudly (recoverable) or silently downloads a different version than expected (insidious); both are avoidable by pinning explicitly.

# Procedure

This procedure governs the **interactive** invocation of the skill (a user in conversation asking Claude to download). The scaffolded `fetch_weights.py` reference implementation (see the example below) is for unattended runs in CI / robot bootstrap and intentionally contains no interactive prompts; that asymmetry is deliberate. Hard rules at the bottom enforce both shapes.

1. **Read the conversation for context.** Resolve all four prerequisite inputs (see the Prerequisites section above) before step 2:
   * `repo_id`, `repo_type`, `revision`: if clear from prior turns or a manifest in scope, do not re-ask; otherwise ask via `AskUserQuestion`.
   * **File selection** (single filename, glob, subdirectory, or "all"): when not stated explicitly, ask via `AskUserQuestion` with options scoped to the situation (e.g. `single file: <name>` / `glob: *.pt` / `subdir: <path>/` / `everything`). Never silently default to "all"; "all" can be tens of GB and is rarely what the user actually wants.

2. **Decide between `snapshot_download` and `hf_hub_download` loop.** Default to `snapshot_download`. Drop to a per-file `hf_hub_download` loop only when one of these applies:

   * Per-file sha256 verification post-download (the manifest-pinned pattern).
   * Conditional skip: do not download if a local file already matches the expected sha256.
   * Flat output layout: HF's in-repo path is nested but consumer code expects `<target>/<basename>`.

3. **Verify `revision` is a 40-character SHA**, not `main` or a branch name. Use Python: `assert len(revision) == 40 and all(c in "0123456789abcdef" for c in revision)`.

4. **Confirm via `AskUserQuestion` before pressing go.** Render the planned call inline as a fenced code block (the exact `snapshot_download` or `hf_hub_download` invocation with `repo_id`, `revision`, the `allow_patterns` or per-file `filename` list, and the destination: cache only vs `local_dir=<path>`), then ask with two options: **download** / **abort**.
   * **download**: proceed to step 5 with the rendered call.
   * **abort**: STOP. Nothing is written to disk.
   When `local_dir=<path>` is set and the target directory already contains files, the rendered block MUST include a one-line note listing the existing entries that would be overwritten (matched by basename against the planned download), so the user sees the local-state cost before approving. A bare "download" / "yes download" / "go ahead" typed in the current turn counts as confirmation; ambiguous replies do not.

5. **Run the call.** Use the canonical recipe below, with arguments matching the approved step-4 rendering verbatim.

6. **Capture the return value.** `snapshot_download` returns the directory containing downloaded files; `hf_hub_download` returns the path to a single file inside the HF cache.

7. **(Manifest-pinned consumers only)** Post-download, verify per-file sha256 against the manifest. Skip otherwise.

# Canonical recipe

The default path for any download:

```python
from huggingface_hub import snapshot_download
import os

local_dir = snapshot_download(
    repo_id="<owner>/<repo>",
    repo_type="model",                       # or "dataset" / "space"
    revision="<40-char SHA>",                # full SHA, never main / branch / short hash
    allow_patterns=["*.pt"],                 # glob filter; omit for everything
    # ignore_patterns=["*.md"],              # optional reverse filter
    local_dir="<target-dir>",                # optional; omit = cache only at ~/.cache/huggingface/hub/
    token=os.environ.get("HF_TOKEN"),        # private repo needs token; public ignores
)
print(local_dir)
```

Key behaviours:

* **Parallel by default**: `snapshot_download` uses multi-threaded HTTP transfers internally, so N files are an order of magnitude faster than N serial `hf_hub_download` calls.
* **Content-addressed cache**: identical content is stored once on disk regardless of how many consumer projects pull it. Symlinks (or copies, depending on filesystem) materialize files at `local_dir`.
* **Resumable**: interrupted LFS transfers resume on retry without re-downloading bytes already in cache.
* **Idempotent**: repeat calls hit the cache; only first-time pulls and changed revisions trigger network I/O.

# Operation patterns

The non-obvious shapes; the obvious "fetch one file" / "fetch everything" follow directly from the canonical recipe by setting `allow_patterns` appropriately.

| User intent | Call shape |
|---|---|
| Manifest-pinned with per-file sha256 verify | loop `hf_hub_download` per manifest entry + sha256 compare; use `snapshot_download` only when no per-file verification is needed |
| Flat output (HF has nested layout, consumer wants flat) | `hf_hub_download` per file then `shutil.copyfile(src, target / pathlib.Path(remote_path).name)`; `snapshot_download` mirrors HF's nested layout in `local_dir` |
| Just inspect HF state without downloading | `HfApi().repo_info(repo_id, repo_type=..., files_metadata=True)` returns file list with LFS sha256 OIDs; faster than a download when only metadata is needed |
| Existing model into a runtime library | `AutoModel.from_pretrained("<repo>", revision="<sha>")` for transformers; `DiffusionPipeline.from_pretrained` for diffusers; both call `snapshot_download` internally |

# Example: manifest-pinned pull (fetch_weights.py reference)

The canonical production pattern when a consumer repo pins HF revisions via a yaml manifest (the `hf-setup` Phase 6 convention). Uses `hf_hub_download` per file because per-file sha256 verification and conditional skip require per-file granularity:

```python
#!/usr/bin/env python3
import hashlib, os, pathlib, shutil, sys
import yaml
from huggingface_hub import hf_hub_download
from huggingface_hub.errors import HfHubHTTPError

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO_ROOT / "config" / "model_weights.yaml"
DEST_DIR = REPO_ROOT / "models"


def _sha256_of(path):
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    spec = yaml.safe_load(CONFIG_PATH.read_text())
    DEST_DIR.mkdir(parents=True, exist_ok=True)

    # Basename collision guard: flat layout on disk means two manifest entries
    # with the same basename would silently overwrite each other.
    basenames = [pathlib.Path(e["filename"]).name for e in spec["weights"].values()]
    dupes = sorted({b for b in basenames if basenames.count(b) > 1})
    if dupes:
        print(f"[fetch_weights] manifest basename collisions: {dupes}", file=sys.stderr)
        return 1

    token = os.environ.get("HF_TOKEN")
    for name, entry in spec["weights"].items():
        target = DEST_DIR / pathlib.Path(entry["filename"]).name
        if target.exists() and _sha256_of(target) == entry["sha256"]:
            print(f"[fetch_weights] {name}: cached")
            continue
        try:
            src = hf_hub_download(
                repo_id=spec["hf_repo"],
                filename=entry["filename"],
                revision=spec["hf_revision"],
                token=token,
            )
        except HfHubHTTPError as err:
            # 404 often means the LFS object was purged on HF (Settings ->
            # Storage -> Manage LFS Files) even though the git commit itself
            # still resolves. Re-upload or pick a fresh revision.
            print(f"[fetch_weights] {name}: {err}", file=sys.stderr)
            return 1
        shutil.copyfile(src, target)
        if _sha256_of(target) != entry["sha256"]:
            print(f"[fetch_weights] {name}: sha256 drift", file=sys.stderr)
            return 1
        print(f"[fetch_weights] {name}: downloaded")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

Key choices:

* `hf_hub_download` per entry, not `snapshot_download`, because the sha256 check happens before deciding to download (skip when cached + valid).
* `shutil.copyfile(src, target)` flattens HF's potentially nested layout to a single directory; the basename collision guard up front prevents silent overwrites.
* `HfHubHTTPError` catches network and HF-side errors; the comment documents the most surprising failure cause (LFS object purge).

# Failure modes

| Error | Cause | Action |
|---|---|---|
| `HfHubHTTPError 401` | Token missing or expired (private repo only) | `hf auth login --force`; verify with `hf auth whoami --format agent` |
| `HfHubHTTPError 403` | Token lacks Read access on the repo | Issue a fine-grained Read token scoped to this specific repo |
| `HfHubHTTPError 404` on a file | Wrong `filename`, or the LFS object was purged on HF Settings -> Storage | Verify path against `repo_info(files_metadata=True)`; if the path is right, the LFS payload is gone and you need to re-upload or pick a fresh revision |
| `HfHubHTTPError 404` on a repo | `repo_id` typo, or private with wrong token | Verify owner/name spelling; check visibility |
| `HfHubHTTPError 429` | Bandwidth rate limit on the token's namespace | Back off; HF's per-namespace bandwidth quotas apply |
| `LocalEntryNotFoundError` | Offline mode + file not in cache | Restore network or unset `HF_HUB_OFFLINE` |
| `RevisionNotFoundError` | Wrong commit SHA, or a non-main branch the revision lives on | Verify SHA against `HfApi().list_repo_refs(repo_id)` |
| Network timeout mid-transfer | Slow or spotty connection | Re-run; LFS transfers resume from the cached partial |

All `HfHubHTTPError` instances expose `.response.status_code` and a body; surface both verbatim to the user before suggesting a fix.

# Hard rules

* Never pass `revision="main"`, a branch name, or a short hash. Use the full 40-character commit SHA. Branches move; short hashes risk ambiguity; the pin must be immutable so drift is detectable.
* NEVER skip the step 4 `AskUserQuestion` gate when this skill is invoked interactively. The download writes to local disk (the HF cache, or `local_dir` which can overwrite existing files), and the approval rendering is what gives the user the chance to spot a wrong revision or an unintended overwrite before bytes hit the filesystem.
* Conversely, NEVER add interactive prompts to a scaffolded `fetch_weights.py` (the manifest-pinned reference implementation). That script is run unattended from CI, build systems, and robot bootstrap; an `AskUserQuestion`-equivalent there would block automation. The asymmetry between this skill (interactive, gated) and the generated script (unattended, ungated) is intentional: gating happens once at scaffolding time (the `hf-setup` skill writes the file after its own gate), not on every fetch.
* For repos using a manifest-pinned downloader pattern, never skip the per-file sha256 verification after download. The pin is meaningless if you do not verify what landed.
* Never display, log, or persist the HF token in chat, scratch files, or commit messages. The token lives only at `~/.cache/huggingface/token` (after `hf auth login`) or in CI secret stores as `HF_TOKEN`.
* Never block a per-render / per-request / per-frame hot path on a download. Pre-download during build or setup (e.g. `catkin_install_python`-installed `fetch_weights.py` invoked from a bootstrap step); at runtime only verify, never fetch.
* Never recommend `git clone + git lfs pull` as a download path. `snapshot_download` / `hf_hub_download` is the supported route; the git workflow adds a local `git-lfs install` dependency and does not parallelize beyond one HTTP connection per file.
* If a `404 on file` surfaces despite a valid `repo_id` + `revision`, check HF's LFS storage page before assuming the path is wrong. Purged LFS objects keep the git commit resolvable while the underlying blob is gone; the symptom is identical to a typo from the API's perspective.
