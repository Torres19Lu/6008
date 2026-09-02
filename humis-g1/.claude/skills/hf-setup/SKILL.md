---
name: hf-setup
description: Guide a user through setting up a Hugging Face Hub repository to host and version-control large binary files (model weights, datasets, checkpoints, ROS bags, anything too big or churny for regular git). Use whenever the user has files exceeding ~50 MB that they want to share, version, or pull at runtime, especially when GitHub LFS account quotas are a concern or when per-project HF repos would scale better. Also trigger when the user mentions Hugging Face Hub, hosting model weights, sharing trained checkpoints, a git LFS alternative, or asks how to make large artifacts available to teammates / CI / deployments. Defaults to **inspection mode**: Phase 0 runs read-only probes for the `hf` CLI, the `huggingface_hub` Python library, and `hf auth whoami` login state, then walks the user step-by-step through any missing piece without itself installing software, logging in, or signing the user up. Once prereqs are green, Phase 0c asks via `AskUserQuestion` whether the run is **full setup**, **configure-only**, or **abort**, so a fresh invocation never silently creates remote state, modifies the local token cache, or writes files in the consumer repo. Operates as progressive Q&A: detects existing HF accounts / orgs / repos before suggesting new ones, helps pick repo type (Model vs Dataset vs Bucket vs Space), recommends license and visibility based on context, walks through fine-grained access token creation, and scaffolds optional code-side glue (manifest YAML + downloader script + .gitignore rule) so the consuming repo gets reproducible builds. Most operations are user-executed in the HF web UI or via the `hf` CLI; the skill's job is to ask the right questions, surface sensible defaults with rationale, and verify state at every step.
---

# Goal

Walk a user from "I have large files that don't belong in git" to a working Hugging Face Hub repository that hosts those files, with optional code-side glue for reproducible downloads at build or runtime. Most actions happen in the HF web UI or via the `hf` CLI on the user's machine. The skill's job is to ask the right questions, detect existing state before suggesting new things, recommend sensible defaults with rationale, and scaffold the code-side integration when wanted.

# When NOT to use

* Files under ~50 MB and infrequently churning: plain `git` (with a binary entry in `.gitattributes`) is simpler.
* The user only needs a private artifact bucket without git semantics: HF Hub is overkill; use S3, R2, B2, or GitHub Releases directly.
* The files are confidential and must not leave a self-hosted boundary: HF is hosted; consider self-hosted Forgejo or Gitea + LFS, or DVC with a private remote instead.

# Procedure

Seven phases (Phase 0 to Phase 6). The skill defaults to **inspection mode**: Phase 0 is read-only probes plus user-driven prerequisite onboarding, Phase 1 and Phase 2 are Q&A only, and the first action that mutates remote state or writes a file in the consumer repo is gated by an execution-stage `AskUserQuestion` immediately preceding it (Phase 3f, Phase 5b via `hf-upload`, Phase 6a, Phase 6b, Phase 6c, Phase 6d). Skip a phase when context or a successful probe already answered its questions, but never invent answers; ask via `AskUserQuestion` whenever unsure. Each `AskUserQuestion` call must list 2 to 4 mutually exclusive options with concrete tradeoffs in the descriptions; the user can always type a custom answer.

**Two layers of `AskUserQuestion` are required and they are NOT interchangeable.**

* *Design-stage prompts* (owner, repo type, repo name, license, visibility, layout, and Phase 0c run-scope) pick *what* the setup should look like and *what subset* of phases this invocation should run.
* *Execution-stage gates* pick *now is the time to do it* and run immediately before any operation that creates remote state, modifies the local token cache, or writes / edits a file in the consumer repo. Each pairs an action verb with `abort`. Required execution gates:
  * Phase 3f: `create` / `abort` (CLI `hf repos create` only; the web UI path is user-driven and has no Claude-side gate to run).
  * Phase 5b: gate is owned by `hf-upload` step 5 (`upload` / `edit` / `abort`); this skill delegates rather than re-asking.
  * Phase 6a: `write` for a brand-new manifest YAML, `edit` for an existing one; either renders the full content (or diff) inline before the gate.
  * Phase 6b: `write` / `abort` for a brand-new `fetch_weights.py`; renders the full proposed content inline.
  * Phase 6c: `patch` / `abort` per consumer file, only when Claude is doing the edit on the user's behalf.
  * Phase 6d: `append` / `abort` against the existing `.gitignore` (via `Edit`, never `Write`).

  Skipping the design-stage prompt is fine when context already answered it; skipping the execution-stage gate is NEVER fine.

## Phase 0: Pre-flight checks and prerequisite onboarding

This phase NEVER changes a file, the local token cache, or any remote state. Its sole job is to detect what is already in place and walk the user through whatever is missing before the skill enters Phase 1. The default outcome of a fresh invocation is to leave the user with a confirmed plan, not to land remote state or scaffold files.

### 0a. Probe environment and account

Run the three read-only probes as three independent `Bash` tool calls in a single tool-batch so they run concurrently:

```bash
which hf || echo "hf-missing"
```
```bash
python3 -c "import huggingface_hub; print(huggingface_hub.__version__)" 2>&1 || echo "lib-missing"
```
```bash
hf auth whoami --format agent 2>&1 | head -n 1
```

The third probe (`whoami`) requires probe 1 (`hf` CLI) to have succeeded; if probe 1 returns `hf-missing`, treat probe 3's output as "Invalid user token" by definition and skip ahead to 0b's onboarding for the CLI install. Running all three in the same batch still saves the `python3` round-trip even when `hf` is absent.

Render the result inline as a checklist:

```
prereq check:
  [x|.] hf CLI installed     : <version> or hf-missing
  [x|.] huggingface_hub lib  : <version> or lib-missing
  [x|.] logged in            : user=<name> orgs=<...> or Invalid user token
```

`hf auth whoami --format agent` returns a single line `user=<name> orgs=<a,b,c>` whether stdout is a TTY or a captured Bash subprocess, which is why the agent format is mandatory here and again in Phase 2. The capture from this probe is shared with the rest of the skill; do not re-run the probe in Phase 2 unless the user changed identity in between (logged out, switched org, swapped tokens).

### 0b. Guide the user through missing prerequisites

For each unchecked box, render the canonical fix the user runs on their own machine. Do NOT invoke `pip install`, `hf auth login`, or any remote signup from this skill; install and auth actions are user-driven so the skill never modifies the user's interpreter, shell profile, or token cache without an in-person decision.

* **`hf` CLI missing** (`hf-missing`): the CLI ships with the `huggingface_hub` library starting v1.0; recommend `pip install -U huggingface_hub` inside whichever virtualenv or conda env the user wants the CLI in. For a system Python that refuses, suggest `pipx install huggingface_hub` or `uv tool install huggingface_hub`. After install, ask the user to confirm, then re-run probe 0a.
* **`huggingface_hub` Python lib missing** (`lib-missing`): same `pip install -U huggingface_hub` command. The Python API is what `hf-upload`, `hf-download`, and the scaffolded `fetch_weights.py` all import; missing it blocks both the CLI and Phase 5/6.
* **Not logged in** (`Invalid user token` or empty `whoami`): defer the actual `hf auth login` step to Phase 4 (which has the full token-creation flow). Phase 1 design questions and Phase 2 ownership choice are still safe to run while logged out; only Phase 3a (`list_models` / `list_datasets` on private repos), Phase 3f (`hf repos create`), and Phase 5b (`hf-upload`) require an authenticated session.
* **No HF account at all** (the user says they have no account): direct to `https://huggingface.co/join`. Wait for the user to confirm signup before re-running probe 0a; never assume signup completed based on time elapsing.

If any prerequisite is missing, STOP here, surface the missing-pieces list, and wait for the user to report each fix is done. Re-enter Phase 0 after each fix; never silently flip a box from `.` to `[x]` without a fresh probe.

### 0c. Confirm the scope of this run via `AskUserQuestion`

Once Phase 0a reports all boxes checked (or the user explicitly opts in to a discovery-only pass despite gaps), ask with three options:

* **full setup** (offer this option ONLY when every prereq is green): proceed through Phase 1 to Phase 6 with every execution-stage gate intact. Each state-changing step still asks its own per-action `AskUserQuestion` immediately before execution; "full setup" approves the *pipeline*, not the *individual mutations*.
* **configure-only** (always offered): run Phase 1 and Phase 2 only, then STOP before Phase 3. Use this to refine the design (modules in scope, owner, repo type, name candidate, license, visibility, layout) without creating remote state or touching any file. The session ends with a written recommendation the user can re-enter from later.
* **abort**: STOP. No further phases run.

The default outcome of a fresh invocation is NOT "full setup". Never silently promote a logged-in workstation to full-setup mode; a user with a working CLI may still want only a configure-only pass.

## Phase 1: Establish context

Phase 0 already detected the HF account state (or guided the user through signup), so this phase focuses on what the user is hosting and at what scope. Read the conversation first. If any of these are already answered, do not re-ask. Otherwise gather them, one `AskUserQuestion` per group:

1. **What files?** Approximate sizes, formats, count, churn rate. Affects repo type and quota planning.
2. **Consuming repo?** Where the files will be loaded from (a GitHub repo, a CI pipeline, a robot, an unrelated workstation). Affects whether to scaffold Phase 6 (downloader).
3. **Who uses them?** Solo developer, small team, public release. Affects org-vs-personal and visibility.
4. **Scope of this setup?** Brand-new repo (Phase 3 will create), supplement to a repo the user already owns (Phase 3a will reuse, Phase 5b will add to it), or a fresh sibling repo in an existing per-project family (e.g. add `<project>-arm-models` next to existing `<project>-perception-models`). This decides whether Phase 3f runs at all.
5. **Which module(s) and artifact(s)?** The named component(s) and artifact name(s) being hosted (e.g. `perception/button_cls`, `arm/grip_policy`, `voice/wakeword`). Drives the repo name candidate in Phase 3c and the manifest key naming in Phase 6a; capture every module the user wants in scope NOW so later phases do not re-litigate scope creep mid-flow.

## Phase 2: Resolve owner (user vs organization)

Reuse Phase 0a's `hf auth whoami --format agent` capture; do not re-probe unless the user changed identity since Phase 0 (logged out, switched org, swapped tokens). The expected shape is a single line `user=<name> orgs=<a,b,c>`. Plain `hf auth whoami` (no flag) prints a multi-line human block in a real shell, which is fine for the user to read but harder to parse, hence the agent format.

If Phase 0 reported `Invalid user token` AND the user chose **configure-only** at Phase 0c, the ownership decision can still proceed against the username the user names verbally; the actual auth-required calls live in Phase 3a, Phase 3f, and Phase 5b, all of which gate themselves.

Then decide the owner via `AskUserQuestion`:

* **Personal account `<username>`**: simpler, fewer concepts. Right for solo experiments, scratch work, single-maintainer projects.
* **Existing organization `<org-name>`**: project-level ownership, multi-admin, survives personnel changes, separate quota pool. Right for team projects with a shared GitHub repo.
* **Create new organization**: when the project has a stable identity and likely more than one contributor over its lifetime. The user creates it at `https://huggingface.co/organizations/new`; wait for them to confirm creation, then re-run `hf auth whoami --format agent` to verify membership before proceeding.

Org naming heuristic: derive from the project or company name. If the consuming code repo is `acme/foo-bot`, suggest org `acme` or `foo-bot-team`. Use hyphens, lowercase. Confirm with the user; do not auto-create.

## Phase 3: Create or reuse the HF repo

### 3a. Detect existing repos

Check both Model and Dataset listings under the owner, since the repo type is decided in 3b and either could already exist:

```bash
# Model repos:
python -c "from huggingface_hub import HfApi; print([r.id for r in HfApi().list_models(author='<owner>')])"
# Dataset repos:
python -c "from huggingface_hub import HfApi; print([r.id for r in HfApi().list_datasets(author='<owner>')])"
```

If a repo for this purpose already exists, ask via `AskUserQuestion` whether to reuse it or create a fresh one. Never silently push into an existing repo without confirmation; its contents could be unrelated.

### 3b. Choose repo type

Repo type is set at creation and is immutable afterward. Use `AskUserQuestion` to pick once:

The HF "New" menu shows several entries; only some are storage:

| Entry | Use for | Notes |
|---|---|---|
| **Model** | ML weights (.pt, .bin, .safetensors, .onnx, .gguf, .ckpt) | Default `.gitattributes` already LFS-tracks ML extensions; visible in model search; supports inference widget metadata. Default for ML weights. |
| **Dataset** | Training data, evaluation sets, labeled corpora, anything described as "data" rather than a model | Different default `.gitattributes` tuned for data formats; surfaces in dataset search. |
| **Space** | Runnable Gradio or Streamlit demos | Not for static storage; this is a hosted runtime. Skip for hosting purposes. |
| **Bucket** | Generic large-blob storage without git semantics | Object-storage style; no commit history, so manifest pinning (Phase 6) is not possible. Skip when reproducible version pinning is required. |
| Article, Collection, Access Token | Not storage | Article = blog post; Collection = grouping pointer for existing repos; Access Token entry routes to the token settings page. Skip for hosting purposes. |

For binary weights or checkpoints: **Model**. For training data, rosbags, or large CSVs: **Dataset**. Bucket only when version history is genuinely irrelevant.

### 3c. Pick a name

The module list captured in Phase 1 Q5 is the primary input to this step: collapse it into a `<scope>` reflecting which module(s) the new repo will host. Combine with the consuming repo's package or directory layout when those names diverge from what Phase 1 Q5 said (rare but possible).

Convention: `<scope>-<purpose>` where scope reflects the project or module and purpose reflects the artifact class. Patterns:

* `<owner>/<project>-models`: all models for one project in one repo. Fits when Phase 1 Q5 listed only one module or the user explicitly wants a single combined repo.
* `<owner>/<project>-<module>-models`: per-module split when Phase 1 Q5 listed two or more modules and Phase 1 Q4 said "fresh sibling repo in an existing per-project family" (e.g. `acme/foo-bot-perception-models`, `acme/foo-bot-arm-models`).
* `<owner>/<project>-<dataset>-data`: for datasets.

Naming character rules: lowercase ASCII letters, digits, and `-` only. No underscores (HF convention; some tools confuse `_` with separators). Hyphens are the word separator. Stay well under 96 chars.

Render the resulting name candidate(s) inline. If two or more candidates are equally good, surface them through `AskUserQuestion`; otherwise confirm the single best candidate before creation. Never invent a `<scope>` that does not trace back to a Phase 1 Q5 module name unless the user explicitly approves a custom value.

### 3d. License

Set a license at creation, even for private repos. Empty license is harder to add retroactively across team licensing reviews. Use `AskUserQuestion` with these defaults:

* **Apache-2.0** (default for ML model weights): de facto standard in ML; explicit patent grant and reciprocal-termination clause matter when artifacts end up in commercial products. Even private repos benefit from licensing now in case they flip public later. Example fit: perception weights that may ship inside a robot SDK.
* **MIT** when the user has a repo-wide MIT convention and explicitly prefers it. Shorter, broadly compatible, lacks the patent grant.
* **CC-BY-4.0** for datasets (de facto standard for data sharing).
* **Other** (CC-BY-NC, OpenRAIL, custom) only when the user explicitly asks. These have known compatibility quirks with commercial or downstream-fine-tune use; flag the quirk to the user before they pick.

### 3e. Visibility

Decide via `AskUserQuestion`:

* **Public**: anyone can `hf_hub_download` without auth. Best for community models, published research weights, open datasets.
* **Private**: requires authenticated download. Free private storage is 100 GB per account or org (separate from any public quota). Default for proprietary, pre-release, or sensitive artifacts.

When private is chosen, note that every consuming machine (CI, robots, teammates) will need a read token; this becomes Phase 4 + Phase 6e work.

### 3f. Create the repo

Render the resolved repo spec inline as a fenced block:

```
owner   : <owner>
name    : <name>
type    : model | dataset | space
license : <license-id>
visible : public | private
```

If Claude will run the CLI itself, ask via `AskUserQuestion` (**create** / **abort**) before invoking `hf repos create`. If the user prefers the web UI, there is no Claude-side gate to run; the user clicks through `https://huggingface.co/new` and confirms creation back in the conversation. Either way, the repo once created occupies the owner's namespace under that name; even immediate deletion leaves the slug reserved for a cooldown window, so a wrong name costs more than the bandwidth of the first push.

Web UI is easier the first time (visual confirmation, license dropdown, type description): `https://huggingface.co/new` to fill the form. CLI for repeats:

```bash
hf repos create <owner>/<name> --type model    # add --private as needed
```

Note: the CLI subcommand is `repos` (plural) in v1.15+; the older singular `hf repo create` is deprecated and prints a warning.

After creation, confirm by visiting `https://huggingface.co/<owner>/<name>`; the auto-generated `.gitattributes` and `README.md` should be there.

## Phase 4: Authentication

### 4a. Token strategy

HF v1.x supports fine-grained tokens. Recommend a two-token split from the start:

* **Dev token (Write)**: scoped to the new repo, persisted on the user's workstation via `hf auth login`. Used for pushing new files, updating README, managing repo settings. Created in Phase 4b.
* **Deploy token (Read-only)**: scoped to the same repo, distributed to CI, robots, or teammates' machines. Used for `hf_hub_download` only. Created in Phase 6e when integration is actually wired up; keep this separate from the dev token so leaked deploy credentials cannot push.

### 4b. Create the dev token

Direct the user to `https://huggingface.co/settings/tokens` → `Create new token`:

* Type: **Fine-grained** (not classic; classic tokens are over-scoped and read every repo the user can read).
* Permissions → Organization permissions (if org) or Repo permissions (if personal): select the new repo.
* Check `Write access to contents/settings of selected repos` (also auto-checks Read).
* Name: `<repo-name>-dev` (e.g. `acme-foo-bot-models-dev`); names are operator-facing labels, pick something searchable.
* Save the `hf_...` value once (the UI shows it only on creation).

If the user belongs to an org but the org does not appear in the fine-grained permissions UI, the org admin must enable fine-grained tokens for that org under org settings. Surface this to the user before they retry token creation.

Never display, log, or store the token value in conversation or files Claude touches. The token must live only at `~/.cache/huggingface/token` (after `hf auth login`) or in CI secret stores. The user pastes it directly into `hf auth login` and nowhere else.

### 4c. Log in on the workstation

```bash
hf auth login
```

If a token is already cached, use `--force` to force re-login even when the cached token is still valid:

```bash
hf auth login --force
```

`hf auth login` can accept the credential-helper choice either interactively (in some versions) or via flag (in current versions). Either way the choice is:

* `--add-to-git-credential` (or answer `y` interactively): pick this if the user might `git clone https://huggingface.co/<owner>/<repo>` directly via git HTTPS. Stores the token in `~/.git-credentials`.
* `--no-add-to-git-credential` (or answer `n`): pick this if only the `hf` CLI and `huggingface_hub` Python lib will use the token. Smaller blast radius if leaked.

Verify:

```bash
hf auth whoami --format agent
```

Expected: `user=<username> orgs=<list>`. If `Invalid user token`, the paste failed; ask the user to re-run `hf auth login --force` and try again.

### 4d. CLI rename note

`huggingface_hub` v1.0 (released 2025) renamed `huggingface-cli` to `hf`. Older tutorials and Stack Overflow answers still use the old name; substitute when reading them. The Python API (`from huggingface_hub import ...`) is unchanged. The cheat sheet at the bottom carries the full command mapping.

## Phase 5: First upload

### 5a. Decide HF in-repo layout

Use `AskUserQuestion` if the choice is ambiguous; otherwise pick the simpler option and confirm.

* **Flat** (`<repo>/<filename>`): simplest, right when the repo holds one logical artifact class.
* **Subdirs** (`<repo>/<group>/<filename>`): when one repo will hold multiple categories (e.g. `perception/`, `arm/`, `voice/`). Group by what the consuming code calls the artifact, not by file format.

If the consuming code expects flat on-disk paths (common for ROS, CLI tools, training scripts) but you want subdirs on HF for clarity, the downloader script (Phase 6) can flatten via basename. Document this choice explicitly in the manifest so future readers do not confuse the two layouts.

**Basename collision warning**: if subdirs are chosen on HF, no two artifacts may share a basename across subdirs; the basename-flatten in Phase 6b would overwrite the loser silently. The `hf-download` skill's reference script includes a collision check up front; rely on it.

### 5b. Upload via atomic commit

Invoke the `hf-upload` skill. Pass forward the context already established here so hf-upload short-circuits its own prerequisite probes: `repo_id` from Phase 3c, `repo_type` from Phase 3b, current auth from Phase 4c, and the planned in-repo layout from Phase 5a. hf-upload owns the canonical `create_commit` template, the operation-pattern shapes, sha256 pre-step timing, and failure-mode handling. When it returns, capture `info.oid` (the 40-character HF commit SHA); it becomes the `hf_revision` pin in Phase 6a.

### 5c. Record the commit SHA

`hf-upload` already prints `info.oid` and `info.commit_url`; record both. The OID becomes the `hf_revision` pin in Phase 6a.

If the user reused an existing repo in Phase 3a and skipped 5b entirely (no new upload needed), recover the current main SHA retroactively:

```python
from huggingface_hub import HfApi
api = HfApi()
sha = [b.target_commit for b in
       api.list_repo_refs("<owner>/<repo>", repo_type="model").branches
       if b.name == "main"][0]
print(sha)
```

Pin that SHA in Phase 6a.

## Phase 6 (optional): Manifest-pinned integration

Skip if the user only wants to host files for manual browsing or ad-hoc download. Use this phase when:

* The files are pulled by deterministic code (CI, robot startup, training pipeline).
* Drift between expected and actual artifacts would be a silent failure.
* Multiple machines (developers, robots, CI) must agree on which version they have.

### 6a. Manifest YAML

Draft `<consumer-repo>/<package>/config/model_weights.yaml` (adapt path to the consumer's layout). Render the full file content inline first, including the resolved `hf_repo`, the 40-character `hf_revision` from Phase 5c, and one entry per weight with its sha256:

```yaml
# Manifest pinning HF Hub artifacts to a specific commit.
# Updating: push to HF, then bump hf_revision + per-file sha256 in this yaml.
hf_repo: <owner>/<repo>
hf_revision: <40-char commit SHA from Phase 5c>
weights:
  <logical-name-1>:
    filename: <HF in-repo path>     # may be nested
    sha256: <hex digest of local file>
  <logical-name-2>:
    filename: <...>
    sha256: <...>
```

Compute sha256 with `sha256sum <local-file>` (already done in Phase 5b if you followed the order).

`hf_revision` must be the full 40-character commit SHA. The Hard rules section at the bottom explains why short hashes and branch names are forbidden.

Choose the gate verb by target state: **write** / **abort** when the target path does not exist (a brand-new file via `Write`), **edit** / **abort** when the target path already exists (in-place change via `Edit`, preserving unrelated lines). Render either the full proposed content (new file) or the resulting diff (existing file) inline before asking. Never run `Write` against an existing `model_weights.yaml`; that would clobber any hand-tuned entries.

### 6b. Downloader script

Lift the canonical `fetch_weights.py` reference implementation from the `hf-download` skill (per-file `hf_hub_download` loop with basename collision guard, sha256-keyed conditional skip, `shutil.copyfile` flatten to a single directory, and the `HfHubHTTPError` comment documenting LFS-object-purge as a common 404 cause). Adapt path constants (`REPO_ROOT`, `CONFIG_PATH`, `DEST_DIR`) to the consumer's layout; reuse the manifest format from Phase 6a verbatim.

Do NOT invoke the `hf-download` skill from here; that skill's interactive `AskUserQuestion` gate is for one-off downloads and does not apply to a script that runs unattended in CI / robot bootstrap. The scaffolded script intentionally contains no prompts (see `hf-download`'s Hard rules for the unattended-vs-interactive split); this skill's gate sits one level up at scaffolding time, not inside the generated code.

Render the full proposed file content inline (and a diff when the target path already exists), then ask via `AskUserQuestion` with two options: **write** / **abort**. Only on **write** invoke the `Write` tool against `<consumer-repo>/<package>/scripts/fetch_weights.py` (or whatever name the consumer's conventions prefer). Then proceed to Phase 6c for the runtime verification hook.

### 6c. Runtime verification (recommended)

In the actual consumer code (the ROS node, the training script, the CLI tool), re-verify sha256 at startup before loading the artifact. This catches:

* Operator forgot to run `fetch_weights.py` after pulling new code that bumped the manifest.
* Disk file silently corrupted (rare but real on SD-card-backed robots).
* Someone manually edited or replaced the file.

Verification reuses the same manifest as Phase 6b, preserving the single source of truth. Pattern (symbol names match the `hf-download` skill's reference implementation; substitute your logger and helpers):

```python
def verify_pinned_weight(path):
    spec = yaml.safe_load(CONFIG_PATH.read_text())
    entry = next(
        (v for v in spec["weights"].values()
         if os.path.basename(v["filename"]) == os.path.basename(path)),
        None,
    )
    if entry is None:
        return  # not manifest-listed; caller's responsibility
    if not os.path.exists(path):
        log_fatal(f"Pinned weight missing: {path}. Run fetch_weights.py.")
        sys.exit(1)
    if _sha256_of(pathlib.Path(path)) != entry["sha256"]:
        log_fatal(f"Pinned weight sha256 drift: {path}. Re-run fetch_weights.py.")
        sys.exit(1)
```

`log_fatal` is a stand-in for the consumer's logger: use `rospy.logfatal` in a ROS node, `logging.critical` in plain Python, the project's existing fatal logger elsewhere.

Call-site convention: invoke `verify_pinned_weight(resolved_path)` after path resolution and before the model is instantiated (or any file handle is opened). Two anchors that work in practice: (a) immediately after `resolve_model_path()` (or equivalent path helper) returns and before passing the path to the model constructor; (b) inside the optional-feature gate when the weight is controlled by a config flag, so verification only runs when the feature is enabled. Both anchors keep verification on the same code path as model loading, so a missing or drifted weight surfaces in the same line range an operator already inspects when a node fails to start.

For consumers with an optional weight (lazy-load when a feature is enabled), gate `verify_pinned_weight` on the enable flag. If the weight path is in the manifest, missing or drift is fatal; if the path is a custom non-manifest one, `verify_pinned_weight` returns silently and the existing lazy-skip behaviour is preserved.

If the user wants Claude to add the `verify_pinned_weight` call sites on their behalf (rather than copying the pattern themselves), render the per-file diff inline first and ask via `AskUserQuestion` with two options: **patch** / **abort**. One gate per consumer file being edited; do not batch unrelated files behind a single prompt. When the user is doing the edit themselves and only wants the pattern, no gate is required (Claude is not writing anything).

### 6d. `.gitignore` the download target

The downloaded files must not be committed to the consumer repo; that would defeat the whole point of using HF. Add the destination directory to the consumer repo's `.gitignore`:

```text
# Model weights are pulled from Hugging Face Hub by scripts/fetch_weights.py
# (see config/model_weights.yaml). Do not commit the downloaded files.
models/
```

Document the why in the comment so a future reader does not re-add the files thinking the rule is mistaken.

Render the planned `.gitignore` diff inline (append at the end of the existing file unless a clearly related block already exists), then ask via `AskUserQuestion` with two options: **append** / **abort**. Only on **append** invoke the `Edit` tool against `<consumer-repo>/.gitignore`. Never run `Write` against an existing `.gitignore`; use `Edit` so unrelated rules are preserved.

### 6e. Deploy token + build / CI integration

Now create the Read-only deploy token deferred from Phase 4a:

* `https://huggingface.co/settings/tokens` → `Create new token` → **Fine-grained**.
* Permissions: Read access to contents of the new repo only. Do not check any Write permissions.
* Name: `<repo-name>-deploy` (operator-facing label distinguishing it from the dev token).
* Save the value once; distribute via deployment channels below.

Wire the download into whatever orchestrates the consumer:

* **catkin / ROS**: register `fetch_weights.py` in `catkin_install_python`; call it from a bootstrap script before `catkin_make`, or have the launch system invoke it as a precondition.
* **Python package**: add a `make fetch` target, a `setup.py` postinstall hook, or call from `__init__` lazily.
* **Docker image**: run `fetch_weights.py` in a `RUN` step with `--secret id=hf_token` so the token does not bake into a layer.
* **CI**: store the deploy token as a CI secret; export as `HF_TOKEN`; the fetcher's `os.environ.get("HF_TOKEN")` picks it up automatically.
* **Robot deploy**: write the deploy token to `/etc/environment` or a systemd unit's `EnvironmentFile=`. Never check it into the robot's deployment repo.

Timing contract: `fetch_weights.py` must run to success at least once before any launch / test / runtime model load. Two acceptable placements: (a) a bootstrap step before `catkin_make` / `pip install -e .` so the build itself fails fast if weights cannot be fetched; (b) the launch system invokes it as a system precondition before spawning consumer nodes. The two-phase split (build-time fetch with network, runtime verify with no network) is the contract; collapsing them is prohibited by the Hard rules.

The skill's job ends when the user can run `fetch_weights.py` on a fresh checkout and see all weights download, verify sha256, and the consumer code start without `verify_pinned_weight` complaining. Confirm that end-to-end before declaring success.

# Defaults and rationale

* **Two-token split** prevents leaked deploy credentials from also being write-capable; minor extra setup, large reduction in blast radius.
* **Apache-2.0 default for ML weights**: the patent grant matters more often than people realize when artifacts end up in commercial systems.
* **Per-package HF repos** (`<project>-<module>-models`) over one giant `<project>-models` repo: smaller diff surface, independent versioning, finer access control, one bad commit in one module does not pollute other modules' history.
* **Manifest with full SHA + per-file sha256** over branch tags or short SHAs: branches move, sha256 is immutable, drift becomes detectable instead of insidious.
* **Basename-flatten on download** decouples HF layout from on-disk conventions; HF can reshape its in-repo paths without breaking the consumer's launch files. The collision check in Phase 6b is the price of that decoupling.
* **`hf_hub_download` to cache + `shutil.copyfile` to target**, rather than `local_dir=` to target: the HF cache is shared across multiple consumer checkouts on the same machine, so the same file is not stored N times.
* **Fine-grained tokens over classic**: classic personal tokens can read every repo the user can read; fine-grained scopes to specific repos.
* **`hf auth whoami --format agent` for detection**: agent format is single-line and stable across TTY modes, so the same probe works in a Bash subprocess and on a user's shell.

# Hard rules

* NEVER enter Phase 1 or any later phase before Phase 0 has either reported every prerequisite green (`hf` CLI, `huggingface_hub` lib, and `hf auth whoami --format agent` returns a `user=...` line) AND Phase 0c has been answered with **full setup**, OR Phase 0c has been answered with **configure-only** (in which case the run stops after Phase 2). A fresh invocation defaults to inspection, not to creation; silently promoting a logged-in workstation to "full setup" because the probes happen to be green is the exact blind execution this rule blocks.
* NEVER invoke `pip install`, `pipx`, `uv tool`, `hf auth login`, or any HF web signup from inside this skill. Phase 0b only renders the canonical command for the user to run on their own machine; the user re-confirms each fix before Phase 0a re-probes. The skill must never modify the user's interpreter, shell profile, or token cache without an in-person decision.
* Never display, log, or write a user-pasted HF token into chat, scratch files, or committed configs. The token must live only at `~/.cache/huggingface/token` (after `hf auth login`) and in CI secret stores. Anything else is a leak.
* Design-stage prompts NEVER substitute for execution-stage gates. The execution gates listed in the Procedure intro (Phase 3f for repo creation, Phase 5b via `hf-upload` step 5, Phase 6a manifest YAML, Phase 6b `fetch_weights.py`, Phase 6c `verify_pinned_weight` call-site patches when Claude is doing the edit, Phase 6d `.gitignore`) MUST each fire immediately before their corresponding mutation. Blind scaffolding (writing or editing a file in the consumer repo without first rendering the proposed content or diff inline and getting the action verb `AskUserQuestion` approval) is the failure mode this rule blocks.
* Never suggest `git lfs install` in the consumer code repo. HF Hub uses LFS internally on its own side; the consumer repo does not need LFS configured to use `hf_hub_download`. Reaching for `git lfs install` is a leftover habit from the GitHub LFS workflow.
* Never pin `hf_revision` to `main`, a branch name, or a short SHA. Use the full 40-character commit SHA only. Branches move; short hashes risk collision and tooling ambiguity; the pin must be immutable to make drift detectable.
* Never commit downloaded artifacts into the consumer repo. Add the download target to `.gitignore` at the same time as wiring up the fetcher; never one without the other.
* Never repurpose a Write token for deployment. Create a separate fine-grained Read token even when it feels like an extra step.
* Never trigger an HF download from a consumer node's runtime startup path. The build-time `fetch_weights.py` and the runtime `verify_pinned_weight` are deliberately separate phases: fetch is allowed to use network and may fail recoverably; verify is fatal on missing with no fallback. Collapsing them masks "operator forgot the bootstrap step" failures and adds blocking network I/O to a startup hot path. See Phase 6e for placement guidance.
* When the user has an existing HF account, org, or repo, detect it first (`hf auth whoami --format agent`, `list_models(author=...)`, `list_datasets(author=...)`, `repo_info`) and confirm reuse vs new-create. Do not auto-suggest a new name on top of unknown existing state.

# Reference: `hf` CLI cheat sheet

Old (deprecated) to new mapping for users with old-tutorial muscle memory:

| Old | New |
|---|---|
| `huggingface-cli login` | `hf auth login` |
| `huggingface-cli whoami` | `hf auth whoami` |
| `huggingface-cli logout` | `hf auth logout` |
| `huggingface-cli repo create <name>` | `hf repos create <owner>/<name> --type <type>` |
| `huggingface-cli upload ...` | `hf upload ...` |
| `huggingface-cli download ...` | `hf download ...` |

Note: `hf repo` (singular) is deprecated in v1.15+ in favor of `hf repos` (plural); the table above already uses the new form.

Most-used commands:

```bash
hf auth whoami --format agent        # parseable single-line: user=... orgs=...
hf auth login [--force]              # paste a token; --force forces re-login
hf repos create <owner>/<name> --type model [--private]
hf upload <repo> <local> <remote> --commit-message "..."
hf download <repo> <remote> --local-dir <dir>
```

Python API for atomic multi-file commit: see the `hf-upload` skill (canonical template covering add / replace / delete / mixed ops, plus failure-mode handling).

Python API for downloads: see the `hf-download` skill (`snapshot_download` for the universal batch + glob primitive, `hf_hub_download` for per-file manifest-pinned loops, plus failure-mode handling).

Probe HF state without a browser:

```python
from huggingface_hub import HfApi
api = HfApi()
info = api.repo_info("<owner>/<repo>", repo_type="model", files_metadata=True)
print("private:", info.private, "HEAD:", info.sha)
for s in info.siblings:
    lfs = getattr(s, "lfs", None)
    oid = (lfs.get("sha256") if isinstance(lfs, dict)
           else getattr(lfs, "sha256", None) if lfs else None)
    print(f"  {s.rfilename}  lfs_sha256={oid}")
```

Note: `files_metadata=True` is required to populate the LFS OID field; without it, `s.lfs` is `None` even for LFS-tracked files.
