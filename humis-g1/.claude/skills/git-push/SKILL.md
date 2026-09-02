---
name: git-push
description: Push a local branch to origin safely. Defaults to the current HEAD; the user may name any local branch in the current turn (e.g. "push main", "push feature/foo"). Refuses force-push, refuses pushes to protected branches (`main`, `develop`, `support/*`) without an explicit per-turn opt-in, and refuses non-fast-forward pushes. Use when the user wants to publish a branch, share commits, or push their work to the remote.
---

# Goal

Publish a local branch to `origin` safely. Never force-push. Refuse pushes to protected branches (`main`, `develop`, `support/*`) unless the user has explicitly opted in for the current turn. The skill performs the push only; it does not commit, merge, rebase, or rescue commits onto another branch.

# Procedure

1. **Resolve target branch.**
   * If the user named a branch in the current turn (e.g. "push main", "push feature/foo"), use that name.
   * Else default to the current HEAD: `git rev-parse --abbrev-ref HEAD`. If the literal output is `HEAD`, the working copy is detached.
   * If no name was given AND HEAD is detached, STOP and ask via `AskUserQuestion` which local branch to push. Do not guess.

2. **Gather state for the target branch.** First batch, run in parallel:
   * `git status --short` (working-tree state).
   * `git fetch origin <branch>` (refresh the remote ref; a remote ref that does not yet exist on first push is not an error).

   Then, after the fetch completes, run in parallel:
   * `git log origin/<branch>..<branch> --oneline 2>/dev/null` (commits about to land; on first push this lists everything reachable from `<branch>`).
   * `git log <branch>..origin/<branch> --oneline 2>/dev/null` (remote-only commits; non-empty means non-fast-forward).

   If `<branch>` does not exist locally, the `git log` calls error out; surface that error verbatim and stop.

3. **Protected-branch gate.** If `<branch>` is `main`, `develop`, or matches `support/*`:
   * Require an explicit opt-in from the user in the current turn (e.g. "yes, push `<branch>` directly", "I'm intentionally pushing main"). Opt-ins from earlier turns do NOT carry over.
   * Without an in-turn opt-in, refuse and explain the alternatives:
     * For `main`: checkout main and invoke `git-merge` to integrate, then re-invoke this skill; or open a PR.
     * For `develop` / `support/*`: the user must explicitly opt in.

4. **Refuse on non-fast-forward.** If `git log <branch>..origin/<branch>` is non-empty, the remote has commits the local branch lacks:
   * STOP. Show both lists (local-only and remote-only).
   * Recommend `git pull --rebase origin <branch>`. The `git-pull` skill is the right call only when the local branch has no unpublished commits (i.e. `git log origin/<branch>..<branch>` is empty); otherwise it will refuse on divergence.
   * Do NOT auto-resolve with `--force` or `--force-with-lease`.

5. **Refuse when nothing to push.** If the target has an upstream and `git log origin/<branch>..<branch>` is empty, tell the user the remote is already up to date and stop.

6. **Handle dirty tree.**
   * If the current HEAD equals the target branch AND `git status --short` is non-empty: warn that uncommitted changes will NOT be included, and offer to invoke `git-commit` first.
   * If the current HEAD differs from the target branch, mention in the plan output that uncommitted changes on the current HEAD are not part of this push, and proceed.

7. **Show the plan and confirm.**
   * Display the commits about to land (`git log origin/<branch>..<branch> --oneline`) and the exact command:
     * **No upstream and current HEAD == target**: `git push -u origin <branch>` (sets tracking on first push).
     * **No upstream and current HEAD != target**: `git push origin <branch>:<branch>`; afterward, advise the user to run `git branch --set-upstream-to=origin/<branch> <branch>` if they want tracking on the local ref.
     * **Has upstream, fast-forward**: `git push origin <branch>`.
   * Ask via `AskUserQuestion` (options: **push** / **abort**). A "push" / "yes push" / "go ahead" typed in the current turn counts as confirmation; ambiguous replies do not.

8. **Run the push.** Execute the command exactly as displayed. Capture output verbatim.

9. **Handle failures. Do NOT retry blindly.**
   * **Server-side branch protection rejection**: surface the rule output. Do NOT attempt to bypass (no `--force`, no deploy-key swap). Tell the user they need to disable the rule, push via a PR, or contact a maintainer.
   * **Non-fast-forward rejection** (a race between step 2 and step 8): re-fetch, surface the new divergence per step 4, stop.
   * **Auth / network / hook**: surface the error verbatim and stop.

10. **Report success.** Show:
    * The pushed range (`<old-sha>..<new-sha>`) or `[new branch]` on first push.
    * Branch name and upstream.
    * `git log --oneline -5 <branch>` so the user can confirm what is now on the remote.

# Hard rules

* NEVER use `--force` or `--force-with-lease` unless the user explicitly types the flag themselves in the current turn.
* NEVER push to `main`, `develop`, or `support/*` without an explicit in-turn opt-in from the user. Prior-turn approval does not carry over.
* NEVER auto-resolve a rejection (non-FF, branch protection, hook failure) by force-pushing, rebasing destructively, deleting commits, or any other workaround. A rejection means STOP-and-explain.
* NEVER skip the step 7 confirmation. Confirming the exact commits is the safety net.
* This skill does NOT commit, merge, rebase, or rescue commits onto another branch. When those are needed, surface the situation and suggest the matching skill (`git-commit`, `git-merge`, `git-pull`, `git-branch-create`); let the user invoke it.
* If the push fails for any reason, report the failure verbatim. Let the user diagnose.
