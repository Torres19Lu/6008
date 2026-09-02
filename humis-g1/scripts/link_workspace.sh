#!/usr/bin/env bash
# Regenerate catkin_ws/src symlinks to the first-party g1_* packages and the
# external/* runtime submodules. First-party packages all live under packages/
# (g1_msgs, g1_bringup plus the domain folders platform/ slam/ mapping/
# navigation/); they are discovered by scanning packages/ for package.xml at any
# depth, so new domain folders or packages need no edit here. catkin_ws/ is
# git-ignored, so symlinks are not tracked; run this once after clone and after
# adding packages/submodules. Idempotent: correct symlinks are left as-is.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${REPO_ROOT}/catkin_ws/src"
mkdir -p "${SRC}"

link_one() {
  local target="$1" name; name="$(basename "$target")"
  [[ -f "${target}/package.xml" ]] || return 0   # only catkin packages
  ln -sfn "${target}" "${SRC}/${name}"
  echo "[link] ${name} -> ${target}"
}

# First-party packages: everything under packages/, at any depth (root-level
# g1_msgs/g1_bringup plus the domain folders). references/ holds unbuilt upstream
# sources and is never scanned; external/ is linked separately below.
while IFS= read -r -d '' pkgxml; do
  link_one "$(dirname "$pkgxml")"
done < <(find "${REPO_ROOT}/packages" -name package.xml -print0)
# Runtime submodules under external/ (one level)
for d in "${REPO_ROOT}"/external/*; do [[ -d "$d" ]] && link_one "$d"; done

echo "[ok] catkin_ws/src linked under ${SRC}"
