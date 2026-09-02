---
name: git-commit
description: Stage the user-named files (or the entire workspace by default, with confirmation), draft a commit message inline in chat per the repo's .gitmessage spec, get user approval (or edits), then run `git commit -F -` with a heredoc. Use when the user wants to commit pending changes, says "commit this", "commit <paths>", "make a commit", or asks for help landing local changes as a commit.
---

# Goal

Take the user's pending workspace from "edited" to "committed" in one guided flow. The draft MUST follow `.gitmessage` at the repo root, stay in concise bullet form, and contain no AI, agent, Claude, or Anthropic co-author. See **Procedure** for the staged flow and **Hard rules** for the bypass / amend / sensitive-content prohibitions.

# Procedure

1. **Gather state.** Run in parallel:
   * `git rev-parse --show-toplevel` (repo root, so the procedure can verify it is run from inside the working tree).
   * `git status --short`.
   * `git diff --cached --stat` (currently staged, shape summary for the user).
   * `git diff --stat` (currently unstaged, shape summary for the user).
   * `git diff --cached` (full staged diff, fuel for the draft).
   * `git diff` (full unstaged diff, fuel if any of these will be staged).
   * `git rev-parse --abbrev-ref HEAD` (current branch; literal `HEAD` means detached).
   * `git log --oneline -10` (recent style reference, also covers HEAD subject for hook-failure diagnosis).

2. **Refuse when nothing to commit.** If `git status` shows no staged, unstaged, or untracked entries, STOP and tell the user the tree is clean.

3. **Ensure a feature branch is checked out — handle BEFORE asking the user about scope, stage choice, or anything else.** If HEAD is on `main`, `develop`, `support/*`, OR detached, the branch gate fires unconditionally and must be resolved first. Do NOT prompt the user about commit scope, stage choice, or untracked files until a short-lived branch is checked out — those questions belong to step 4 and assume a writable feature branch.

   a. **Skip this step ONLY when** the user has explicitly typed something like "yes, commit directly on `<branch>`" in the current turn. Prior-turn approval does NOT carry over.

   b. **Infer a sensible branch name** from the staged + unstaged diff gathered in step 1. Pick the narrowest top-level scope that captures the change (typically the dominant package/directory under the repo root) and a kebab-case short-desc summarising the change in 2-5 words. Form `feature/<scope>/<short-desc>` (e.g. `feature/g1-camera/torch-optional-yolo`). If the change legitimately spans multiple packages, fall back to scope `repo` (e.g. `feature/repo/share-map-and-button-cls`). Existing local branches (`git branch --list`) are a good style reference.

   c. **Invoke the `git-branch-create` skill** with the proposed name as the argument, for example `Skill(git-branch-create, "feature/g1-camera/torch-optional-yolo from main")`. The branch-create skill owns its own naming validation and `AskUserQuestion` confirmation — do not duplicate either here. `git checkout -b <new-branch>` carries uncommitted working-tree changes onto the new branch by default; git refuses the carry only when an incoming branch file would clobber a local edit, which is not the case for a brand-new branch.

   d. **After `git-branch-create` returns**, verify with `git rev-parse --abbrev-ref HEAD` that a short-lived branch is now checked out, then re-enter this skill at step 1 to re-gather state on the new branch.

4. **Resolve the commit scope.**
   * **User-named scope in the current turn** (for example "commit src/foo.py and src/bar.py", "commit the launch dir", "commit only what is already staged"): stage exactly that set and nothing else. Use `git add <path>...` for explicit paths, and `git reset HEAD -- <path>` to unstage anything the user excluded.
   * **No scope specified, default is the entire workspace**: list every pending change (staged, unstaged, and untracked separately) and ask via `AskUserQuestion` whether to (a) commit everything pending, (b) commit only the currently staged set, (c) commit a user-supplied subset of paths, or (d) abort. Option (a) is the recommended default; selecting it stages all tracked modifications via `git add -u` (safe because `-u` never picks up untracked files) and stages each untracked file ONLY after the user confirms it by name.
   * **Both staged and unstaged non-empty with no scope said**: same prompt as above, and surface that a partial-stage exists so the user can decide intentionally.
   * `git add -u` is permitted ONLY for the tracked-modification bulk-stage in option (a) above. NEVER run `git add .` or `git add -A`, and NEVER use a path glob that could pick up untracked files. Stage untracked files only by explicit name after per-file confirmation.

5. **Sanity-check sensitive content.** Before drafting, scan the final staged diff for likely secrets or absolute home paths:
   * Patterns to flag: `.env`, `credentials.*`, `*.pem`, `*_rsa`, `api[_-]?key`, `token`, `password`, `secret`, `/home/<user>/`, `~/`.
   * If any match, STOP and ask for confirmation per file. Suggest `.gitignore` additions rather than committing.

6. **Draft the message inline (no file).** Read `.gitmessage` from the repo root for the authoritative type, scope, and footer rules; if absent, fall back to the spec below. Using the final staged diff (refresh `git diff --cached` if step 4 restaged anything since step 1) and the recent log gathered in step 1, compose the draft per the **Commit message specification** below. Hold the draft in chat as a fenced code block.

7. **Show the draft and confirm.** Display the drafted message inline as a fenced code block in chat, then ask via `AskUserQuestion` (use the `preview` field to show the draft text alongside the options): **approve** / **edit** / **abort**.
   * **approve**: proceed to step 8 with the current draft.
   * **edit**: the user replies with edits (free-text instructions such as "drop bullet 3", "add `Refs #42`", "shorten the subject", or a complete replacement message). Apply the edits, re-render the draft inline, and re-ask **approve** / **edit** / **abort**. Loop until approved or aborted.
   * **abort**: stop. The staged set stays as is; the draft is discarded with the conversation.

8. **Commit via heredoc, no file.** Run the commit through a shell heredoc that pipes the approved message to `git commit -F -`. The quoted heredoc terminator prevents shell expansion of the body. Use exactly:
   ```
   git commit -F - <<'COMMIT_MSG_EOF'
   <type>(<scope>): <subject>

   - bullet 1
   - bullet 2
   COMMIT_MSG_EOF
   ```
   Substitute the approved draft verbatim between the heredoc markers, preserving the blank line between header and body. If the approved draft contains a line equal to `COMMIT_MSG_EOF`, pick a fresh terminator (for example `COMMIT_MSG_EOF_2`) that does not appear in the body for that invocation. Do NOT pass `--no-verify`, `--no-gpg-sign`, `-c commit.gpgsign=false`, `--amend`, or `--allow-empty` unless the user explicitly typed the flag themselves in the current turn. Capture the exit code and full output.

9. **Handle hook failure.** If the commit fails because a pre-commit, commit-msg, or pre-push hook blocked it:
   * The commit did NOT happen. The previous commit at HEAD is unchanged.
   * Surface the hook's exact output to the user.
   * STOP. Do NOT retry with `--no-verify`. Do NOT `--amend` (that would modify the previous commit since the new one never landed). Help the user fix the underlying issue, then re-run this skill from step 1. The drafted message stays in the conversation; the user may approve it again unchanged after the fix.

10. **Report success.** On a successful commit, show:
    * The new commit SHA and subject (`git log -1 --format='%h %s'`).
    * `git status --short` (anything still pending that was not part of this commit).
    * The natural next step: typically `git-push` to publish, or another `git-commit` if more pending changes remain.

# Commit message specification

The draft MUST follow `.gitmessage` at the repo root, stay in concise bullet form, and contain no AI, agent, Claude, or Anthropic co-author. The `git-merge` skill follows the same body and footer rules for its merge commit; any change here MUST be mirrored in `git-merge/SKILL.md`.

* **Header (line 1)** in the form `<type>(<scope>): <subject>`:
  * `<type>` is exactly one entry from `.gitmessage`'s allowed list (`feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`, `revert`, `merge`, `deps`, `config`, `env`, `ui`, `security`).
  * `<scope>` is the narrowest stable identifier (package, directory, or component) whose contract changed. When the change legitimately spans multiple scopes, prefer one broader scope (for example `repo`) over a comma list, and tell the user the commit may be worth splitting.
  * `<subject>` is imperative mood, no trailing period, target 50 characters, hard limit 72. Describe outcome (`enable X by default`), not mechanics (`change line 80`).
* **Line 2** is blank.
* **Body** is one bullet per logical change, each prefixed with `- `. Each bullet:
  * MUST be a SINGLE physical line; do NOT wrap inside a bullet.
  * References concrete files or symbols so a reader can locate the change.
  * Explains the why when non-obvious; for pure renames or formatting, why may be absent.
  * Groups co-edits that exist only to keep things in sync (for example "sync README to match the new launch default").
  * Skips incidental whitespace or auto-formatter churn.
* **Footer block** (optional, separated by one blank line from the body). Include ONLY when the user named one of these in the current turn:
  * `Closes #<id>` / `Fixes #<id>` / `Refs #<id>`.
  * `BREAKING CHANGE: <message>`.
  * `Co-authored-by: <name> <email>` (only for a real human collaborator the user named; never an AI agent).
* The message ends after the last meaningful line. Do not include the `.gitmessage` template comment lines.

# Hard rules

* NEVER write the commit message to a file on disk (no `tmp/commit_msg.txt`, no `.git/COMMIT_EDITMSG` pre-population, no scratch file under the repo root, system tmp, or `/var/tmp`). The message lives only in the chat conversation and the heredoc that feeds `git commit -F -`.
* NEVER stage files with `git add .` or `git add -A`. Stage explicit paths only, and confirm each untracked file by name before staging it.
* NEVER pass `--no-verify`, `--no-gpg-sign`, or any other hook-skipping flag unless the user typed it themselves. Hooks fail for a reason; fix the underlying issue.
* NEVER `--amend` after a hook failure. The failed commit never landed, so `--amend` would silently rewrite the PREVIOUS commit. Always create a new commit instead.
* NEVER commit files matching the sensitive-content patterns in step 5 without per-file user confirmation.
* NEVER commit directly on `main`, `develop`, `support/*`, or a detached HEAD without first invoking `git-branch-create` to land the work on a fresh short-lived branch. The only exception is an explicit "yes, commit on `<branch>`" from the user in the same turn.
* NEVER add `Generated with Claude Code`, `Co-Authored-By: Claude`, `Co-Authored-By: Anthropic`, or any AI, agent, or assistant attribution to the commit message. Do not invent co-authors. The commit message MUST follow `.gitmessage` (at the repo root), stay in concise bullet form, and stay human-authored.
* NEVER include credentials, API keys, or absolute paths under `/home/<user>/` in the body. Use repo-relative paths only.
* If `git commit -F -` fails for any reason other than a hook (for example a corrupt index or missing GPG key), surface the error verbatim and stop. Do not invent a workaround.
