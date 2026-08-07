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
# structural cause). The three codes below are noise this codebase's own
# conventions produce, not defects. Note that a comment line beginning
# `# shellcheck ` is itself parsed as a directive -- keep prose about a
# code off the start of a line, or this file fails its own check.
#
# SC1091 (not following: FILE was not specified as input) -- 13 occurrences
# in 12 files, measured on the corpus this gate actually passes. Nine are
# `tools/ci/` scripts sourcing gate-lib.sh as
# `. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"`, plus
# `tools/mpr-vs-epa/build.sh` reaching it as `"$HERE/../ci/gate-lib.sh"`;
# A path built from a runtime expression is one that cannot be followed
# statically. The remaining two are `ros/entrypoint.sh` and
# `tools/moveit-oracle/entrypoint.sh` sourcing `/ws/install/setup.bash`, an
# absolute path that exists only inside the oracle image -- no `source=`
# directive can resolve a file that is not in this tree at all.
#
# The nine-plus-one gate-lib sites COULD each carry
# `# shellcheck source=tools/ci/gate-lib.sh`, and those directives do
# resolve (the 16 existing `source=` directives for src-digest.sh prove it:
# they report "was not specified as input", which is shellcheck declining to
# follow an external file, not failing to find one). Retiring this exclusion
# in favour of ten directives is open work, not a settled preference.
#
# An earlier version of this comment claimed SC1091 "fires on nearly every
# one of the 59 tools/ci/ scripts"; it fires on nine of sixty. It also
# excluded SC2154 on the theory that an unfollowable source makes the names
# it defines read as unassigned -- SC2154 has zero occurrences across all 75
# tracked `.sh` files, so that exclusion suppressed nothing and is gone.
# Both figures were written before shellcheck was installed here.
#
# SC2016 (expressions don't expand in single quotes) -- 41 sites, 40 of them
# in `check-evidence-retention-discriminates.sh` and one in
# `check-document-sections-discriminates.sh`. In every one the single quotes
# are REQUIRED rather than merely chosen: a `sed -i 's#...#...#'` script, or
# a string of Korean prose and Markdown holding backticks and a literal `$`
# (shell-transcript fixture text like `$ tools/ci/measure-beta.sh out`).
# Double quotes there would change what the script does. Restructuring is
# not available for the remainder either -- 1c598517 already converted the
# one run of `echo` lines that could become a quoted heredoc, and a sed
# expression has to stay a sed expression. This is the code being right and
# the check unable to tell.
#
# SC2317 (command appears to be unreachable) -- 6 sites, all inside
# `ros/verify-move-action-interop.sh`'s cleanup function, which runs from a
# `trap`. shellcheck's own message names this case ("or ignore if invoked
# indirectly"): it does not model a trap as a call, so the whole handler
# reads as dead code.
#
# Everything else stays enabled and unmodified.
EXCLUDE=(
  --exclude=SC1091
  --exclude=SC2016
  --exclude=SC2317
)

if ! shellcheck "${EXCLUDE[@]}" -- "${sh_files[@]}"; then
  exit 1
fi

echo "OK: shellcheck passed on ${#sh_files[@]} tracked .sh files"
