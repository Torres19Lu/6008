---
name: git-branch-create
description: Create a new short-lived branch (feature/*, bugfix/*, hotfix/*, release/*) following the repo's .gitmessage naming rules and branched off the correct base. Confirms via `AskUserQuestion` (`create` / `abort`) before running the checkout; never auto-fetches or auto-pulls the base ref (route the user to `/git-pull <base>` when refresh is wanted). Use when the user wants to start a new feature, fix, hotfix, or release branch, or asks how to create a branch.
---

# Goal

Help the user create a properly-named local branch off the correct base, following the repo's `.gitmessage` branch-management spec. **Do NOT push the branch**. Publishing is the `git-push` skill's job.

# Procedure

1. **Read the spec.** Read the repo's `.gitmessage` to discover the allowed branch types, the naming variant in use, and the protected-branch list. If `.gitmessage` is missing, fall back to the defaults below.

2. **Gather state.** Run in parallel:
   * `git rev-parse --abbrev-ref HEAD` (current branch)
   * `git branch --list` (existing local branches; avoid name collisions)
   * `git status --short` (warn if dirty before switching base)

   Note: this skill does NOT run `git fetch`. When the user wants the base refreshed, route them to `/git-pull <base>` (which itself gates the fetch + fast-forward with `AskUserQuestion`), then rerun this skill. The no-fetch policy keeps the skill scoped to branch creation and prevents accidentally moving the base ref without an explicit per-operation confirmation; the Hard rules section restates it for emphasis.

3. **Choose the branch type** (`feature`, `bugfix`, `hotfix`, `release`, `support`). If the user did not say, ask via `AskUserQuestion` with these options:
   * `feature/*` (new functionality). Base: `develop` (or `main` if the repo uses GitHub Flow).
   * `bugfix/*` (non-urgent defect fix targeting next release). Base: `develop` (or `main`).
   * `hotfix/*` (urgent fix against production). Base: `main`.
   * `release/*` (release stabilization). Base: `develop` (or `main`).
   * `support/*` (long-lived maintenance line for a previous major). Base: a release tag on `main`.

4. **Choose the naming variant** (per `.gitmessage`'s "Branch Naming" section):
   * Variant A: `<type>/<issue-id>-<short-desc>` (when an issue tracker is the source of truth)
   * Variant B: `<type>/<scope>/<short-desc>` (when work is organized by module/package)
   * Variant C: `<type>/<short-desc>` (when PR titles carry context)
   If unclear, inspect existing local + remote branches with `git branch -a --list 'feature/*' 'bugfix/*' 'hotfix/*'` and infer the dominant variant. If still ambiguous, ask.

5. **Validate the name** against `.gitmessage`'s "Naming Character Rules":
   * Lowercase ASCII letters, digits, and `-` only inside path segments.
   * `/` strictly as the type/scope separator.
   * No spaces, underscores, uppercase, accents, or non-ASCII.
   * Total length 60 characters or fewer.
   * REFUSE protected names: `main`, `develop`, anything matching `support/*` (those are long-lived, not created via this skill).

6. **Verify the base.** Confirm the base branch exists locally (`git rev-parse --verify <base>`). When HEAD already equals base, the working tree is preserved across the new-branch creation regardless of dirty state, so no extra prompt is needed.

   When HEAD differs from base AND `git status --short` is non-empty, `git checkout <base>` would either carry the uncommitted edits onto the base branch (when the incoming `<base>` does not modify the same paths) or refuse outright (when it does). Both outcomes are bad surprises, so ask via `AskUserQuestion` with two options:
   * **commit-first (recommended)**: STOP here and hand off to `/git-commit` so the user can land the pending edits on the current branch first; after the commit, re-run this skill from step 1.
   * **abort**: STOP. The user keeps the pending edits and decides separately what to do with them.
   Do NOT offer a "switch anyway" option; `git checkout --` and `git reset --hard` are forbidden by the Hard rules below and silent stashing is out of scope for this skill.

7. **Show the plan and ask for confirmation via `AskUserQuestion`.** Render the exact command sequence, then ask with two options: **create** / **abort**. Only proceed to step 8 on an explicit **create** selection (a bare "create" / "yes create" / "go ahead" typed in the current turn also counts; ambiguous replies do not).
   * **HEAD already equals base**:
     ```
     git checkout -b <type>/<short-desc> <base>
     ```
   * **HEAD differs from base** (clean tree only; otherwise step 6 already STOPPED, either via the **commit-first** handoff or via **abort**):
     ```
     git checkout <base>
     git checkout -b <type>/<short-desc> <base>
     ```
   Pull is intentionally NOT part of this plan. If the user wants the base aligned with `origin/<base>` first, instruct them to run `/git-pull <base>` (which has its own `AskUserQuestion` gate) before re-running this skill.

8. **Create the branch.** Execute the approved plan verbatim. Report the new branch name and suggest the next step: "Make commits, then `/git-push` when ready to publish."

# Hard rules

* NEVER create a branch with a protected name (`main`, `develop`, `support/*`).
* NEVER `git fetch`, `git pull`, or otherwise refresh the base ref from inside this skill. When the user wants `origin/<base>` integrated, route to `/git-pull <base>`; that skill performs its own `AskUserQuestion` gate before touching the working tree.
* NEVER discard uncommitted work to switch bases. Offer the `git-commit` skill first; refuse `git reset --hard` and `git checkout --` as shortcuts.
* NEVER skip the step 7 `AskUserQuestion`. Branch creation is cheap to undo (`git branch -D`), but a wrong-name branch left lying around clutters tooling; the confirmation is the cheapest safety net against typos.
* If the user's proposed name violates the naming rules, suggest a corrected version and ask via `AskUserQuestion` before using it.
