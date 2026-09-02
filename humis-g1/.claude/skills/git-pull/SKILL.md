---
name: git-pull
description: Pull a local branch from origin via fast-forward only. Defaults to the current HEAD; the user may name any local branch in the current turn (e.g. "pull main", "pull feature/foo"). Refuses non-fast-forward; never touches the working tree of a branch that is not the current HEAD. When divergence forces a merge commit, hands off to the `git-merge` skill, which drafts the merge message inline in chat per the repo's .gitmessage spec, in concise bullet form, with no AI or Claude co-author, and commits via a `git commit -F -` heredoc (no file ever written). Use when the user wants to update a local branch with new commits from origin.
---

# Goal

Bring a local branch up to date with `origin/<branch>` via fast-forward only. The skill performs the pull only; it does not merge another branch in, rebase, resolve conflicts, or stash. When the requested branch matches the current HEAD, the pull updates the working tree; otherwise the skill updates the local branch ref via `git fetch origin <branch>:<branch>` without checking the branch out. If integration requires a non-fast-forward merge (the "pull triggers merge" case), this skill refuses and hands off to the `git-merge` skill. Any resulting merge commit MUST follow `.gitmessage` at the repo root, stay in concise bullet form, and contain no AI, agent, Claude, or Anthropic co-author (enforced by the `git-merge` skill). This skill itself NEVER drafts or writes a commit message.

# Procedure

1. **Resolve target branch.**
   * If the user named a branch in the current turn (for example "pull main", "pull feature/foo"), use that name.
   * Else default to the current HEAD: `git rev-parse --abbrev-ref HEAD`. If the literal output is `HEAD`, the working copy is detached.
   * If no name was given AND HEAD is detached, STOP and ask via `AskUserQuestion` which local branch to pull. Do not guess.

2. **Gather state for the target branch.** First batch, run in parallel:
   * `git status --short` (working-tree state).
   * `git fetch origin <branch>` (refresh the remote ref).

   Then, after the fetch completes, run in parallel:
   * `git log <branch>..origin/<branch> --oneline 2>/dev/null` (incoming commits).
   * `git log origin/<branch>..<branch> --oneline 2>/dev/null` (local-only commits).

   If `<branch>` does not exist locally, or `origin/<branch>` is missing after fetch, surface the error verbatim and stop.

3. **Classify and refuse on no-op or non-fast-forward.**
   * **Both lists empty**: local is already up to date with `origin/<branch>`. STOP and tell the user.
   * **Only local-only is non-empty**: local is ahead, remote has nothing new. STOP and tell the user there is nothing to pull (suggest `git-push` if they want to publish).
   * **Both lists non-empty (divergence)**: local and remote have diverged, which would require a non-fast-forward merge. STOP. Show both lists. This is the "pull triggers merge" case: this skill never produces the merge commit itself. Tell the user to `git checkout <branch>` (if not already on it), then invoke the `git-merge` skill with `origin/<branch>` as the source. The `git-merge` skill drafts the merge commit message inline in the chat per the repo's `.gitmessage` spec, in concise bullet form (header `merge(<scope>): integrate origin/<branch> into <branch>`), with no AI, agent, Claude, or Anthropic co-author, and commits via a `git commit -F -` heredoc with no file ever written. Do NOT auto-resolve from this skill.
   * **Only incoming is non-empty**: fast-forward is possible; proceed.

4. **Handle dirty tree.**
   * If the current HEAD equals the target branch AND `git status --short` is non-empty: warn that any local edits to files touched by the incoming commits will block the fast-forward, and offer to invoke `git-commit` first or abort. Do NOT stash; that is out of scope for a simple pull.
   * If the current HEAD differs from the target branch, the working tree is untouched; mention this in the plan output and proceed.

5. **Show the plan and confirm.**
   * Display the commits about to land (`git log <branch>..origin/<branch> --oneline`) and the exact command:
     * **Current HEAD == target**: `git merge --ff-only origin/<branch>` (the fetch already happened in step 2).
     * **Current HEAD != target**: `git fetch . origin/<branch>:<branch>` (local-only refspec; step 2 already refreshed `origin/<branch>`, so no second network round-trip is needed, and the bare `<branch>:<branch>` form stays FF-only).
   * Ask via `AskUserQuestion` (options: **pull** / **abort**). A "pull" / "yes pull" / "go ahead" typed in the current turn counts as confirmation; ambiguous replies do not.

6. **Run the pull.** Execute the command exactly as displayed. Capture output verbatim.

7. **Handle failures. Do NOT retry blindly.**
   * **Non-fast-forward rejection** (a race between step 2 and step 6): re-fetch, surface the new divergence per step 3, stop. Tell the user to `git checkout <branch>` (if not already on it) and invoke the `git-merge` skill with `origin/<branch>` as the source.
   * **"Your local changes would be overwritten"**: surface verbatim, suggest `git-commit` first, stop. Do NOT auto-stash.
   * **Network or auth**: surface verbatim and stop.

8. **Report success.** Show:
   * The pulled range (`<old-sha>..<new-sha>`).
   * The target branch name and whether the working tree was updated (current HEAD case) or only the branch ref (off-HEAD case).
   * `git log --oneline -5 <branch>` so the user can confirm the new tip.

# Hard rules

* NEVER use `--force`, `--rebase`, `+<branch>:<branch>`, or any flag that rewrites history. The skill is fast-forward only.
* NEVER auto-resolve a non-fast-forward by rebasing, merging another branch in, or stashing. Divergence means STOP-and-hand-off to the `git-merge` skill.
* NEVER touch the working tree of a branch that is not the current HEAD. Use `git fetch <ref>:<ref>` for off-HEAD targets; never `git checkout` to switch first.
* NEVER skip the step 5 confirmation. Confirming the incoming commits is the safety net.
* NEVER draft a commit message and NEVER write one to a file (no `tmp/commit_msg.txt`, no `.git/COMMIT_EDITMSG` pre-population, no scratch file). When a merge commit is required, the `git-merge` skill takes over and keeps the drafted message inline in the chat only.
* NEVER allow an AI, agent, Claude, or Anthropic co-author trailer to slip into any merge commit produced after this skill hands off. The merge commit message MUST follow `.gitmessage` (at the repo root), stay in concise bullet form, and stay human-authored.
* If the pull fails for any reason, report the failure verbatim. Let the user diagnose.
