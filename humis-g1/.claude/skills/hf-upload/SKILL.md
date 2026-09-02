---
name: hf-upload
description: Run a single atomic Hugging Face Hub commit that adds, replaces, deletes, or mixes any combination of file operations in one shot, then capture the new commit SHA and URL for downstream version pinning. Wraps `huggingface_hub.HfApi.create_commit` plus `CommitOperationAdd` / `CommitOperationDelete`. Confirms via `AskUserQuestion` (`upload` / `edit` / `abort`) before pressing go; any `CommitOperationDelete` triggers a WARNING line above the rendered op list naming each deletion target. Use whenever the user needs to push files to HF Hub and the repo plus auth are already configured: "upload this to HF", "push this weight to huggingface", "update the model on HF", "replace v1 with v2 on the hub", "add a checkpoint to my HF repo", "delete the old file on HF", or any combination of those. Also use when bumping a manifest-pinned weight in a consumer repo: the skill returns the SHA that goes into `model_weights.yaml::hf_revision`, closing the loop with the downstream `fetch_weights.py` pattern. For first-time setup (no HF account, no repo, no auth yet), invoke `hf-setup` instead and return here at its Phase 5. CLI shortcuts (`hf upload <repo> <local> <remote>`) and git push workflows are special cases of the same primitive; this skill is the canonical answer for any non-trivial upload because `create_commit` is the only path that handles N>1 files atomically, returns the commit SHA directly, and accepts delete + rename ops.
---

# Goal

Run one atomic HF Hub commit covering any combination of add, replace, delete, or rename operations on files in a repo, then return the commit SHA and URL for follow-up steps (manifest sync, README updates, deploy notifications). Wraps `HfApi.create_commit`, which is the only upload primitive that handles multi-file atomicity, mixed op types, and SHA capture in one call. CLI `hf upload` and git push are convenience shells around subsets of this primitive.

# When NOT to use

* HF is not yet configured (no account, no repo, or not logged in): invoke `hf-setup` first, then return here.
* The user is contributing to a public repo via PR review: use `hf upload --create-pr` or the HF web UI's "Open Pull Request" instead. This skill commits straight to `main`.
* The user wants to clone the repo locally first and use ordinary git workflow: that is also valid but heavier (requires `git-lfs` locally + a full clone). Only walk that path when the user explicitly asks.

# Prerequisites

Verify before any upload action:

```bash
hf auth whoami --format agent
```

Expected: `user=<name> orgs=<a,b,c>`. If `Invalid user token` or no output, the user is not logged in; have them run `hf auth login` (or `hf auth login --force` if a stale token is cached) before continuing. For first-time auth, defer to `hf-setup` Phase 4.

The skill also needs three concrete inputs from the user, by `AskUserQuestion` or explicit prior context:

1. **`repo_id`** in `<owner>/<name>` form (e.g. `g1-intellect/g1-camera-models`).
2. **`repo_type`** (`model`, `dataset`, or `space`).
3. **For each operation**: the local path (for adds) and the `path_in_repo` (always). For replace, `path_in_repo` must exactly match the existing HF path; mismatch creates a duplicate file rather than replacing.

Never guess any of these; ask via `AskUserQuestion` when unsure. `create_commit` surfaces a clean 404 if `repo_id` is wrong, but a wrong replace `path_in_repo` silently creates a duplicate file rather than failing, so confirm paths before pressing go.

# Procedure

1. **Read the conversation for context.** When invoked as a sub-skill from `hf-setup` Phase 5b, the caller passes `repo_id` (from `hf-setup` Phase 3c), `repo_type` (from Phase 3b), and the in-repo layout convention (from Phase 5a) forward; reuse them as-is. Otherwise, if any of `repo_id` / `repo_type` / layout convention is not clear from prior turns, ask via `AskUserQuestion`. Authentication is verified once at the Prerequisites step above and is not re-probed here.

2. **Enumerate the operation set.** Apply the layout convention from step 1 to construct each `path_in_repo`. List every file that will be touched, with one bullet per planned op, in the form `add <local-path> -> <path_in_repo>`, `replace <path_in_repo> with <local-path>`, `delete <path_in_repo>`, or `rename <old_path_in_repo> -> <new_path_in_repo> from <local-path>`. Show the list inline as a fenced code block. For multi-op uploads (e.g. replace one weight + delete an obsolete one + update README in the same commit), keep them all in one list so the user reviews the entire atomic commit at once.

3. **If a consumer manifest will pin this upload, plan sha256 capture.** Otherwise skip. The hash is needed for the manifest (Phase 6a in `hf-setup`), not for the upload itself, so timing is a trade-off:
   * **Default (files under ~1 GB)**: compute now so the hash appears in the step-5 gate render and the user can spot drift between intent and actual bytes.
   * **Large-file path (multi-GB weights where the user may abort)**: defer sha256 to AFTER step 5 approval, before step 6 `create_commit`. Document the deferral inline so the manifest follow-up in step 8 knows to compute then.

   ```bash
   sha256sum <local-path>
   ```

4. **Draft the `commit_message` inline.** Compose a single short imperative sentence summarizing the atomic operation set from step 2 (e.g. `add button_cls v2 weight`, `replace yolo11s-seg with v3 + drop v1`, `bump README + delete obsolete fp16 weights`). Hold the draft in chat as a fenced block; do NOT write it to a file (`commit_message` flows into `create_commit` via the Python kwarg, not via a file). When the operation set is a manifest-version bump for a downstream consumer, prefer naming the artifact and the version in the message so the future reader can scan history without cross-referencing the manifest.

5. **Confirm via `AskUserQuestion` before pressing go.** Render the operation list from step 2, the step-4 `commit_message` draft, and the step-3 sha256 capture (when computed) as one fenced block, then ask with three options: **upload** / **edit** / **abort**.
   * **upload**: proceed to step 6 with the listed ops and the drafted message.
   * **edit**: the user replies with free-text changes (drop an op, change a `path_in_repo`, swap a local path, rewrite the commit message); apply, re-render, re-ask. Loop until approved or aborted.
   * **abort**: STOP. Nothing is uploaded to HF.
   If the operation list contains any `CommitOperationDelete`, prepend a single-line WARNING above the rendered list naming each deletion target; deletions are reversible only by re-uploading the bytes and are exactly the failure mode this gate exists to prevent. A bare "upload" / "yes upload" / "go ahead" typed in the current turn counts as confirmation; ambiguous replies do not.

6. **Run `create_commit`** with the recipe below, with `operations` matching the approved step-2 list verbatim and `commit_message` substituting the approved step-4 draft verbatim. Use one `CommitOperationAdd` per add or replace, one `CommitOperationDelete` per delete, all in the same `operations` list.

7. **Capture `info.oid` and `info.commit_url`** from the return value. The OID is the 40-character HF commit SHA; the URL is the human-clickable commit page.

8. **Follow-up.** If the consumer repo pins HF revisions via a manifest (the `hf-setup` Phase 6 pattern), sync `hf_revision` plus the relevant per-file `sha256` entries in that manifest and commit the yaml in git. Otherwise the upload is complete.

# Canonical recipe

The full template that covers every upload scenario:

```python
from huggingface_hub import HfApi, CommitOperationAdd, CommitOperationDelete

api = HfApi()
info = api.create_commit(
    repo_id="<owner>/<repo>",
    repo_type="model",        # or "dataset" / "space"
    operations=[
        # add or replace (same path -> replace, new path -> add)
        CommitOperationAdd(path_in_repo="<remote-path>", path_or_fileobj="<local-path>"),
        # delete
        CommitOperationDelete(path_in_repo="<remote-path>"),
        # ... any number of any op type, all atomic ...
    ],
    commit_message="<concise message>",
)
print("OID:", info.oid)           # 40-char HF commit SHA -> manifest hf_revision
print("URL:", info.commit_url)    # browser-clickable commit page
```

Behaviours to know:

* **Atomicity**: if any op fails, the whole commit is rejected; HF never lands a half-applied state.
* **LFS routing is HF-side**: files matching the repo's default `.gitattributes` LFS patterns (most ML extensions: `*.pt`, `*.bin`, `*.safetensors`, `*.onnx`, ...) auto-route to LFS storage. The consumer repo does not need `git lfs install`, and no local clone is required.

# Operation patterns

The non-obvious shapes; add / replace single / multi-file follow directly from the canonical recipe.

| User intent | `operations` list shape |
|---|---|
| Add or replace (any N) | `[CommitOperationAdd(...), ...]`; same `path_in_repo` as an existing file means replace, new path means add |
| Delete | `[CommitOperationDelete(path_in_repo=...)]` |
| Rename | `[CommitOperationDelete(old), CommitOperationAdd(new, local)]` in one list |
| Mixed atomic release (e.g. delete v1 + add v2 + update README) | freely mix `CommitOperationAdd` and `CommitOperationDelete` in the same list |

`path_in_repo` is always required.

# Example: add a new weight + bump manifest in one logical operation

```bash
# 1. Hash locally first (manifest needs it)
sha256sum /path/to/new_weight.pt
# -> abcdef1234... (record this)

# 2. Upload
python - <<'PY'
from huggingface_hub import HfApi, CommitOperationAdd
api = HfApi()
info = api.create_commit(
    repo_id="g1-intellect/g1-camera-models",
    repo_type="model",
    operations=[
        CommitOperationAdd(
            path_in_repo="new_weight.pt",
            path_or_fileobj="/path/to/new_weight.pt",
        ),
    ],
    commit_message="add new_weight v1",
)
print("OID:", info.oid)
print("URL:", info.commit_url)
PY
# -> OID: 4268c2a3b3...
# -> URL: https://huggingface.co/g1-intellect/g1-camera-models/commit/4268c2a3...

# 3. Update consumer manifest (only if the consumer pins HF revisions)
#    Edit g1_camera/config/model_weights.yaml:
#      hf_revision: 4268c2a3b3190ada3545886d395efba0ffda5f5a
#      weights.<name>.sha256: abcdef1234...

# 4. Verify locally
python3 g1_camera/scripts/fetch_weights.py
# Expect: downloaded -> ... then sha256 OK

# 5. Commit the yaml change in the consumer git repo (separate workflow)
```

# Failure modes

| Error | Cause | Action |
|---|---|---|
| `HfHubHTTPError 401` | Token missing, expired, or revoked | Run `hf auth login --force`; verify with `hf auth whoami --format agent` |
| `HfHubHTTPError 403` | Token scope lacks Write on this repo | Token is Read-only or scoped to a different repo; create a Write token at `https://huggingface.co/settings/tokens` |
| `HfHubHTTPError 404` | `repo_id` does not exist or is private and the token has no access | Verify spelling; confirm visibility; verify org membership via `hf auth whoami --format agent` |
| `HfHubHTTPError 413` | Single file exceeds 50 GB (HF hard limit) | Split or compress; no workaround for individual files above 50 GB |
| LFS storage quota exceeded | Owner's private storage past 100 GB free tier | Upgrade plan or delete unused LFS objects via Settings -> Storage |
| `FileNotFoundError` on local path | Local file path wrong | Check the `path_or_fileobj` arg; absolute paths are safest |

All `HfHubHTTPError` instances expose `.response.status_code` and a body; surface both verbatim to the user before suggesting a fix.

# Hard rules

* Never invent `repo_id`, `path_in_repo`, or local paths. If any is unclear, ask the user via `AskUserQuestion`. A wrong `path_in_repo` on a replace silently creates a duplicate; a wrong local path either fails fast or uploads the wrong file.
* NEVER skip the step 5 `AskUserQuestion` gate. The atomic `create_commit` is the irreversible action this skill produces; uploading without an explicit per-operation approval is the exact failure mode the gate exists to prevent. Especially with any `CommitOperationDelete` present, the gate's WARNING line MUST name each deletion target verbatim so the user sees what they are confirming.
* NEVER write `commit_message` to a file on disk. The message is held inline in chat between step 4 and step 6 and flows into `create_commit` via its Python kwarg only; this mirrors the no-file discipline of `git-commit` and `git-merge`.
* Never run the commit before `hf auth whoami --format agent` returns a valid `user=...` line. Authentication failures partway through a long upload waste time and bandwidth.
* Never display, log, or persist the HF token in chat, scratch files, or commit messages. The token lives only at `~/.cache/huggingface/token` or in CI secret stores.
* Never paraphrase or hide `info.oid` from the user. The full 40-character SHA is the version pin downstream consumers depend on; abbreviating it risks the user pasting a short hash into a manifest.
* Never recommend `git clone` + `git push` for an upload that `create_commit` can do. The git workflow requires `git-lfs install` locally, a full clone, and serialized push semantics; `create_commit` is the direct path.
* For repos using a manifest-pinned downloader pattern (the `hf-setup` Phase 6 convention), the upload is not complete until the consumer's manifest is bumped with the new `hf_revision` and per-file `sha256`. Surface this as a required follow-up step, not an optional one.

