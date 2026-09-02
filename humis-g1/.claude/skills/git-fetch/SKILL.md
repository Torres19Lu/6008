---
name: git-fetch
description: Fetch origin's refs (branches and tags) and report how the current branch stands relative to origin/main and its own upstream. Read-only against the working tree, the current branch ref, the index, and the stash; the only writes are to `refs/remotes/origin/*` and the local tag namespace. Never merges, rebases, pulls, checks out, or invokes another skill. Use when the user wants to see what is new on origin, check ahead/behind status, or asks "is my branch behind main", without integrating anything yet.
---

# Goal

Refresh `refs/remotes/origin/*` and tags from `origin`, then print one tight report describing how the current branch stands against the new remote state. **Never modify the working tree, the current branch ref, the index, or the stash.** Integration (merge, rebase, checkout, pull) is explicitly out of scope and belongs to other skills.

This is the one skill in the git family that runs without an `AskUserQuestion` confirmation. The justification is that `git fetch` only writes to `refs/remotes/origin/*` and the local tag namespace, both of which are transient tracking metadata that integration skills (`git-pull`, `git-merge`) already verify before acting; running fetch without prompt costs nothing and saves an unnecessary back-and-forth. Any state-changing operation outside that narrow scope must move to a different skill.

# Procedure

1. **Capture current branch state.** Run in parallel:
   * `git rev-parse --abbrev-ref HEAD` (current branch; literal `HEAD` when detached).
   * `git rev-parse --abbrev-ref @{u} 2>/dev/null || true` (upstream tracking ref, may be empty).
   * `git status --short` (dirty marker for the report banner only; does NOT change behaviour).

2. **Fetch.** Run exactly:
   ```
   git fetch origin --prune --tags
   ```
   * `--prune` removes local `refs/remotes/origin/*` entries whose remote counterparts no longer exist.
   * `--tags` pulls release tags.
   * Do NOT pass `--all` (the repo has only `origin` today).
   * Do NOT pass `--force`.
   * If `git fetch` exits non-zero (network, auth, proxy), surface stderr verbatim and STOP. No retry, no credential helper, no fallback.

   Capture `git fetch`'s stderr; it lists new branches and pulled tags inline (lines starting with `* [new branch]` and `* [new tag]`). The report consumes that directly instead of computing a separate diff.

3. **Compute ahead / behind counts.** Run in parallel (independent reads of post-fetch refs):
   * `git rev-list --left-right --count HEAD...@{u}` (only when step 1 resolved an upstream) for ahead/behind vs upstream.
   * `git rev-list --left-right --count HEAD...origin/main` (skip when the current branch IS `main`, because the upstream block already covers it).
   * `git log --oneline HEAD..origin/main` capped at 10 lines for the incoming-commits list on `main`.

4. **Render the report.** Use this layout exactly. Every field is required; empty data renders the right-column phrase to keep the layout uniform.

   | Field                              | Empty-state phrase                                |
   |------------------------------------|---------------------------------------------------|
   | current branch line                | always present                                    |
   | dirty marker                       | omit when working tree clean                      |
   | vs upstream block                  | "no upstream tracking" when no `@{u}`             |
   | vs origin/main block               | always present (omit only when current branch IS `main`) |
   | new remote branches list           | "none"                                            |
   | incoming commits on origin/main    | "already up to date with origin/main"             |
   | tags pulled this fetch             | "none"                                            |

   Example fully populated report:
   ```
   ## fetch report
   Fetched: origin (prune + tags)

   Current branch: feature/x/y    (dirty: 3 files unstaged)
   - vs upstream origin/feature/x/y : ahead 2, behind 0
   - vs origin/main                 : ahead 5, behind 3

   New remote branches:
     + origin/feature/new-thing

   Incoming on origin/main (HEAD..origin/main, last 10):
     abcd123  feat(scope): subject
     ef45678  fix(scope): subject
     (3 commit(s) not shown; use `git log HEAD..origin/main` for full list)

   Tags pulled: v1.4.0, v1.4.1
   ```

   * Cap the incoming commit list at 10 lines; when truncated print the count of unshown commits and the full `git log HEAD..origin/main` command.
   * Cap the tags list at 10 names with the same overflow phrasing.
   * The new-branches list comes straight from the `git fetch` stderr capture; do NOT compute a separate pre/post `for-each-ref` diff. The user's downstream skills (`git-pull`, `git-branch-create`) re-check ref state when they need it.

# Hard rules

* NEVER run `git merge`, `git rebase`, `git checkout` to switch branches, `git pull`, `git reset`, or `git stash` from this skill. The skill is read-only against the working tree, the current branch ref, the index, and the stash; the only writes are to `refs/remotes/origin/*` and the local tag namespace.
* NEVER auto-invoke another skill. The fetch report is informational only; the operator decides whether to invoke `git-pull`, `git-merge`, or `git-branch-create` next.
* NEVER pass `--force` to `git fetch`. The default fetch refspec is strong enough; `--force` would overwrite local refs and is out of scope.
* NEVER swallow `git fetch` errors. Stderr is surfaced verbatim and the skill stops.
* NEVER ask the user to commit or stash before running. `git fetch` does not touch the working tree.
* NEVER fetch from any remote other than `origin`. Extending to `--all` requires an explicit edit to this skill, not a one-off override.
* NEVER add an `AskUserQuestion` gate inside this skill. The skill's whole reason to exist is to be the one cheap, no-prompt remote-state check; adding a gate would just shift the prompt-and-still-do-nothing cost onto every call site. Any skill that integrates remote state (`git-pull`, `git-merge`, `git-push`, `git-branch-create` via `/git-pull`) carries its own gate at the right moment.
