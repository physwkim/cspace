#!/bin/bash
# Every tracked shell script must pass shellcheck.
#
# 74 tracked `.sh` files across tools/ci/, ros/, tools/moveit-oracle/ and a
# few one-off scripts had never been linted at all before this gate. 16 of
# them carry `# shellcheck source=tools/moveit-oracle/src-digest.sh`
# directive comments -- written for a linter that had never run, so nothing
# had ever confirmed those directives even resolve.
#
# A missing shellcheck is a FAIL, not a skip. A checker that quietly no-ops
# when its tool is missing is the exact failure mode this repository keeps
# finding (`gate-lib.sh`'s `oracle_stamp_verdict`/`oracle_stamp_explain` exist
# because six gates once collapsed "docker refused us" into the same silent
# state as "nothing to check"). shellcheck is not optional infrastructure
# like the oracle container -- there is no legitimate environment in this
# repository's CI where every one of its own shell scripts is unlintable --
# so unlike the `docker-absent` SKIP those gates allow, a missing shellcheck
# here is unconditionally a FAIL.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

if ! command -v shellcheck >/dev/null 2>&1; then
  echo "FAIL shellcheck is not installed -- no tracked .sh file was checked." >&2
  echo "FAIL install it (e.g. apt-get install shellcheck) and re-run; do not treat this as a pass." >&2
  exit 1
fi

# The corpus is every tracked `.sh` file, asked of git, rather than a root
# list written here. Three gates in this tree (check-no-lint-suppression.sh,
# check-audit-completion.sh, check-ls-files-deduplicate.sh) were each found
# blind to ros/ this way -- a hand-maintained root list acquires that blind
# spot the moment a directory is added, and a green run does not distinguish
# "searched and found none" from "did not search". `git ls-files` is also
# gitignore-free by construction.
sh_files=()
while IFS= read -r -d '' f; do sh_files+=("$f"); done < <(git ls-files --deduplicate -z -- '*.sh')
require_nonempty "${#sh_files[@]}" "tracked .sh files to search"

# Global disables, never per-site (a per-site `# shellcheck disable=` reaches
# green by hiding the one instance a reviewer can see instead of the
# structural cause). Both codes below are noise this codebase's own
# conventions produce, not defects:
#
# SC1091 (not following: FILE was not specified as input) -- every gate here
# sources tools/ci/gate-lib.sh (and several source
# tools/moveit-oracle/src-digest.sh) as `. "$REPO_ROOT/tools/ci/gate-lib.sh"`,
# where REPO_ROOT is resolved at runtime from `${BASH_SOURCE[0]}`. shellcheck
# cannot statically follow a path built from a runtime variable, so this
# fires on nearly every one of the 59 tools/ci/ scripts regardless of
# whether the sourced file is correct. Per-site `source=` directives are the
# documented escape for this, but this codebase intentionally resolves
# REPO_ROOT dynamically rather than hardcoding it (so a script keeps working
# from any worktree), and requiring every one of ~60 call sites to carry a
# directive naming a path that already appears one line above it is the
# per-site burden this exclusion avoids -- see this file's own header for why
# a global, stated reason is preferred over scattering suppressions.
#
# SC2154 (VAR is referenced but not assigned) -- `gate-lib.sh` and
# `src-digest.sh` are sourced for the functions and variables they define
# (`require_nonempty`, `require_caller_tree`, `ORACLE_MOVEIT2_SHA`, ...).
# Because SC1091 above already means shellcheck cannot see into those files
# from a call site that sources them dynamically, it also cannot see that
# names used afterward were assigned there, and reports them as unassigned
# under this codebase's near-universal `set -u`. Fixing the SC1091 cause
# fixes this one too; they are one structural cause, not two.
#
# Everything else stays enabled and unmodified.
EXCLUDE=(
  --exclude=SC1091
  --exclude=SC2154
)

if ! shellcheck "${EXCLUDE[@]}" -- "${sh_files[@]}"; then
  exit 1
fi

echo "OK: shellcheck passed on ${#sh_files[@]} tracked .sh files"
