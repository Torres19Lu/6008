---
name: git-merge
description: Merge one local branch into another via direct `git merge --no-ff` (always generates a merge commit; never fast-forwards, so feature topology stays visible in `git log --graph`). The user may name source and target in the current turn (e.g. "merge feature/foo into main", "merge main", or a `git-pull` handoff with `source=origin/<branch>`). Asks via `AskUserQuestion` when source or target is ambiguous. When the target is not the current HEAD, asks via `AskUserQuestion` (`checkout` / `abort` / `choose`) whether to checkout the target, abort, or pick an alternate target; checks out only after explicit user approval and a clean working tree. Before merging, runs a single targeted `git fetch origin` scoped to the local-side refs in the merge to verify remote state and prompts via `AskUserQuestion` (abort recommended) when the local target is stale or diverged relative to origin; sides that are already remote-tracking refs (e.g. `origin/<branch>` source) are skipped because they ARE the remote. Local merge only (no push, pull, rebase, branch cleanup, or PR). Drafts the merge commit message inline in chat per the repo's .gitmessage spec, in concise bullet form, with no AI or Claude co-author, and commits it via a `git commit -F -` heredoc (no file ever written). Use when the user wants to merge a branch into another.
---

# Goal

Run a single `git merge --no-ff <source>` against the current HEAD. The skill performs the merge only; it does not push, pull, rebase, delete the source branch, or auto-resolve conflicts. The only network call is a single read-only `git fetch origin` scoped to the local-side refs in the merge (`<target>` always, plus `<source>` when source is a local branch; a `git-pull` handoff with `source=origin/<branch>` therefore fetches only `<target>`). When the eligible local ref is stale or diverged relative to its `origin/<...>` counterpart, the skill renders the per-ref findings inline and asks via `AskUserQuestion` (abort recommended) before continuing. When the resolved target differs from the current HEAD, the skill asks via `AskUserQuestion` whether to (a) checkout the target and proceed, (b) abort, or (c) pick an alternate target. The checkout is performed only after explicit user approval AND a clean working tree. Every merge generates a merge commit (no fast-forward) whose message MUST follow `.gitmessage` at the repo root, stay in concise bullet form, and contain no AI, agent, Claude, or Anthropic co-author. The drafting rules below match the `git-commit` skill verbatim.

# Procedure

1. **Resolve source and target.**
   * **Explicit form**: "merge X into Y" / "merge X to Y" / "merge X onto Y": source=X, target=Y.
   * **Single-branch form**: "merge X": source=X, target=current HEAD (matches plain `git merge X` semantics).
   * **No-arg form**: "merge": STOP and ask via `AskUserQuestion` for both source and target. Do not guess.
   * If `source == target`, refuse with a one-line explanation.

2. **Align HEAD with target.** Run `git rev-parse --abbrev-ref HEAD` (literal `HEAD` means detached) and `git status --short` in parallel (the status capture is reused in step 7's dirty-tree refusal, so this is the single canonical read). If HEAD already equals the resolved target, proceed to step 3. Otherwise, when the tree is dirty (status non-empty), STOP and offer the `git-commit` skill so `git checkout` cannot carry uncommitted edits across branches. When the tree is clean and HEAD differs from target, ask via `AskUserQuestion` with three options:
   * **checkout**: run `git checkout <target>` then proceed to step 3. If checkout fails (for example branch missing, detached HEAD recovery refused), surface the verbatim error and stop.
   * **abort**: STOP; no git state has been modified.
   * **choose**: prompt the user via free-text for an alternate target name. After receiving the new name, re-run step 2 with that name as the resolved target. If the new target still differs from HEAD, the same three-option prompt appears again; the user can keep choosing or pick **checkout** / **abort** at any iteration.

3. **Verify remote state (targeted fetch).** Classify each side of the merge as either a *local* ref (e.g. `main`, `feature/foo`) or an *already-remote-tracking* ref (matches `^<remote>/`, in practice `^origin/`). For this skill, target is always a local ref because step 2 aligned HEAD with it; source may be either form, since `git-pull` hands off divergence cases with `source=origin/<branch>`.

   Build the fetch refspec list from local-side refs only:
   ```
   refs_to_fetch=()
   for ref in <source> <target>; do
     case "$ref" in origin/*) ;; *) refs_to_fetch+=("$ref") ;; esac
   done
   ```
   If `refs_to_fetch` is non-empty, run:
   ```
   git fetch origin "${refs_to_fetch[@]}"
   ```
   If it is empty (both sides already remote-tracking, an unusual case but possible), skip the fetch entirely; the refs ARE the remote and need no refresh.

   This is read-only against the working tree. If a listed ref has no counterpart on `origin`, `git fetch` emits a warning and exits zero; record that ref as "no remote tracking" for step 4 and proceed. If `git fetch` exits non-zero (network, auth, proxy), surface stderr verbatim and STOP. NEVER pass `--force`, `--prune`, or `--tags`; the fetch refreshes ONLY the local-side refs about to be merged. A broader remote sweep belongs to the `git-fetch` skill, not here.

4. **Decide based on remote divergence.** Run the divergence check only for local-side refs that ALSO have a corresponding `origin/<X>` (probe with `git rev-parse --verify --quiet origin/<X>`). Skip any side that is already a remote-tracking ref: it IS the remote, so "local-vs-remote" comparison does not apply to it. In the common `git-pull` handoff (source=`origin/<branch>`, target=`<branch>`), only the target is checked here.

   For each eligible ref, run in parallel:
   * `git rev-list --left-right --count <branch>...origin/<branch>` (ahead / behind counts).

   Classify each ref:
   * **0 / 0**: local matches remote; no action.
   * **N / 0** (local ahead, remote behind): local has unpublished commits; informational only.
   * **0 / N** (local behind): local is STALE; merging here would land the merge commit on a stale tip and a later push would require a follow-up integration.
   * **M / N** (diverged): local and remote both moved; merging would miss remote commits on that branch.

   If any eligible ref is stale or diverged, render the per-ref findings inline (ahead / behind counts plus `git log <branch>..origin/<branch> --oneline` capped at 10 lines per ref so the user sees what would be missed) and ask via `AskUserQuestion` with two options:
   * **abort (recommended)**: STOP. Suggest the operator invoke `/git-pull <stale-branch>` to refresh the stale ref (the current-HEAD case fast-forwards; the off-HEAD case uses `git fetch . origin/<branch>:<branch>` per the `git-pull` skill), then re-run this skill.
   * **proceed anyway**: continue with the local refs as-is. Record this choice so step 12 can repeat the warning in the success report; the warning is not silently lost.

   If every eligible ref is in sync (or has no remote counterpart), proceed to step 5 without prompting. In the `git-pull` handoff case the target is usually diverged by construction (that is why git-pull handed off in the first place); this prompt is the explicit reconfirmation point and should NOT be skipped on the assumption that the user already saw the divergence in git-pull's report.

5. **Gather state.** Run in parallel:
   * `git rev-parse --verify <source> 2>/dev/null` (source must resolve as a local ref; surface the verbatim error if not).
   * `git log <target>..<source> --oneline 2>/dev/null` (commits the merge will land).
   * `git diff --stat <target>..<source>` (fuel for the inline merge-commit body).
   * `git log -1 --format='%H %s'` (so a hook failure can be diagnosed against the last commit).

   The working-tree status was captured in step 2 and is reused in step 7; do NOT re-run `git status --short` here.

6. **Refuse on no-op.** If `git log <target>..<source>` is empty, the source contains no commits beyond target. STOP and tell the user there is nothing to merge.

7. **Refuse on dirty tree.** If the step-2 `git status --short` capture is non-empty, STOP and offer to invoke the `git-commit` skill first. Do NOT stash; merging over a dirty tree entangles unrelated edits into the merge commit.

8. **Plan the merge.** Always use `--no-ff` to generate a merge commit (never fast-forward), so the source branch's topology stays visible in `git log --graph`. Plan the two-command sequence shown in step 10. Compose the draft inline per the **Merge commit message specification** below. Pick `<scope>` per `.gitmessage`:
   * `feature/<scope>/<desc>` source: use `<scope>`.
   * Otherwise infer the dominant scope from `git diff --stat <target>..<source>`; fall back to `repo` for cross-area integrations (for example back-merging `main` into a feature branch).

9. **Show the plan and confirm.**
   * Display the commits about to land (`git log <target>..<source> --oneline`) and the exact two-command sequence.
   * Render the drafted merge commit message as a fenced code block inline in the chat immediately above the question.
   * Ask via `AskUserQuestion` with three options: **merge** / **edit** / **abort**.
     * **merge**: proceed to step 10 with the current draft.
     * **edit**: the user replies with edits (free-text instructions or a complete replacement message). Apply the edits, re-render the draft inline, and re-ask **merge** / **edit** / **abort**. Loop until approved or aborted.
     * **abort**: stop. No git state has been modified.
   * A bare "merge" / "yes merge" / "go ahead" typed in the current turn counts as confirmation; ambiguous replies do not.

10. **Run the merge.** Always run, in sequence:
    ```
    git merge --no-ff --no-commit <source>
    git commit -F - <<'COMMIT_MSG_EOF'
    <approved message substituted verbatim, preserving the blank line between header and body>
    COMMIT_MSG_EOF
    ```
    The single-quoted heredoc terminator prevents shell expansion of the body. Substitute the approved draft between the markers verbatim. If the approved draft contains a line equal to `COMMIT_MSG_EOF`, pick a fresh terminator (for example `COMMIT_MSG_EOF_2`) that does not appear in the body for that invocation. If `git merge --no-ff --no-commit` fails (for example conflicts), do NOT proceed to `git commit`; handle per step 11.

11. **Handle failures. Do NOT retry blindly.**
    * **Conflicts** (`git diff --name-only --diff-filter=U` non-empty): list the conflicted files and STOP. Do NOT auto-resolve, do NOT `git merge --abort`, do NOT `git reset --hard`. Tell the user to resolve the conflicts themselves; once resolved, ask this skill to finish (the approved draft is still in the conversation and the same heredoc from step 10 commits it), or run `git merge --abort` to back out.
    * **Hook failure on the commit step**: the merge index is still in place and the approved draft is still in the conversation. Surface stderr verbatim and stop. Do NOT pass `--no-verify` or `--amend`. After the user fixes the underlying issue, ask this skill to finish: re-run the step 10 `git commit -F -` heredoc with the still-approved draft.
    * **Other**: surface verbatim and stop.

12. **Report success.** Show:
    * The new target tip (`git log -1 --format='%h %s'`), which is the new merge commit.
    * `git log --oneline --graph -5 <target>` so the user can confirm the merged-in branch topology.
    * When step 4 was answered with **proceed anyway**, repeat the per-ref stale or diverged finding verbatim and recommend `/git-pull <branch>` plus a follow-up integration before the next push, so the warning is not silently lost.

# Merge commit message specification

The merge commit message MUST follow `.gitmessage` (at the repo root). It is the same shape used by the `git-commit` skill; the only difference is that the header type is fixed to `merge` and the body summarizes themes from the source branch instead of staged-diff bullets. Any change to the bullet or footer rules MUST be mirrored in `git-commit/SKILL.md`.

* **Header (line 1)** in the form `merge(<scope>): integrate <source> into <target>`. Imperative mood, no trailing period, target 50 characters, hard limit 72. Use the `merge` type literally; pick `<scope>` per step 8.
* **Line 2** is blank.
* **Body** is one concise bullet per major theme being integrated, each prefixed with `- `. Each bullet:
  * MUST be a SINGLE physical line; do NOT wrap inside a bullet.
  * Summarizes a group of related commits from `git log <target>..<source> --oneline`, naming the affected component when one stands out.
  * Explains the why when non-obvious.
  * Skips trivial commits (formatting touch-ups, typo fixes) that do not carry semantic weight.
  * For tiny back-merges (a single commit being integrated), a single bullet restating that commit's outcome is enough.
* **Footer block** (optional, separated by one blank line from the body). Include ONLY when the user named one of these in the current turn:
  * `Closes #<id>` / `Fixes #<id>` / `Refs #<id>`.
  * `BREAKING CHANGE: <message>`.
  * `Co-authored-by: <name> <email>` (only for a real human collaborator the user named; never an AI agent).
* The message ends after the last meaningful line. Do not include the `.gitmessage` template comment lines.

# Hard rules

* NEVER write the merge commit message to a file on disk (no `tmp/commit_msg.txt`, no `.git/COMMIT_EDITMSG` pre-population, no scratch file under the repo root, system tmp, or `/var/tmp`). The message lives only in the chat conversation and the heredoc that feeds `git commit -F -`.
* `git checkout` is permitted ONLY in the step-2 approve path described above; never outside that step, and never against a path that was not named as the resolved target or the user-supplied alternate.
* NEVER `git push`, `git pull`, or `git rebase`. The only network call permitted is the single read-only `git fetch` in step 3, scoped to the local-side refs in the merge (a `git-pull` handoff with `source=origin/<branch>` therefore fetches only `<target>`, not both); it carries no `--force`, `--prune`, or `--tags`. A broader remote sweep belongs to the `git-fetch` skill.
* NEVER delete or modify the source branch after merging. Branch cleanup is out of scope.
* NEVER auto-resolve conflicts, run `git merge --abort`, or run `git reset --hard` to recover from a failed merge. Surface the obstruction and let the user decide.
* NEVER use `--force`, `--strategy=ours`, `--strategy-option=theirs`, or any flag that hides conflicts. Surface the conflict and stop.
* NEVER pass `--ff` or `--ff-only`, and NEVER omit `--no-ff`. This skill always generates a merge commit so the source branch's topology stays visible in `git log --graph`. Fast-forward is out of scope.
* NEVER pass `--no-verify` or `--no-gpg-sign` on the `git commit` that follows `git merge --no-ff --no-commit`. Hooks fail for a reason; fix the underlying issue.
* NEVER add `Generated with Claude Code`, `Co-Authored-By: Claude`, `Co-Authored-By: Anthropic`, or any AI, agent, or assistant attribution. Do not invent co-authors. The merge commit message MUST follow `.gitmessage` (at the repo root), stay in concise bullet form, and stay human-authored.
* NEVER skip the step 9 confirmation. Confirming the exact commits and the drafted message is the safety net. NEVER skip the step 4 prompt when remote divergence is detected; proceeding silently against a stale ref is the exact failure mode the prompt prevents.
* This skill does NOT commit unrelated edits, push, pull, rebase, or clean up branches; the only `git fetch` it runs is the step-3 targeted refresh of `origin/<source>` and `origin/<target>`. When broader operations are needed, surface the situation and suggest the matching skill (`git-commit`, `git-push`, `git-pull`, `git-fetch`); let the user invoke it. The only branch-switching the skill ever performs is the user-approved step-2 checkout described above.
