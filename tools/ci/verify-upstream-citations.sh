#!/bin/bash
# Resolves every `<file>.cpp:N-M` citation in the continuous-collision port
# against the two upstream trees it was ported from, and fails when one does
# not point at code. `tools/ci/upstream-citations.py` holds the rules and
# the argument for why there are three of them; this script is what finds the
# trees and refuses to grade against the wrong ones.
#
# # Why a citation needs a gate at all
#
# The port documents itself by citing upstream: a doc comment names a C++
# symbol and gives the file and line span its behaviour was read off. 465 of
# them, and until this gate nothing in this repository resolved a single one.
# 46 were wrong when it was written -- 26 opening on a blank line, 20 pointing
# at a neighbouring function or the wrong branch of one -- and all 46 survived
# review, `cargo doc`, clippy at `-D warnings`, and
# `verify-private-doc-links.sh`, none of which can see outside the crate.
#
# # Why the revision check is not decoration
#
# A line number means nothing without the tree it counts lines in. Both
# upstreams are pinned in the port's own provenance headers -- `moveit2 @
# e017c91e`, `bullet3 @ 7dee3436` -- and this script reads the pin from those
# headers rather than carrying its own copy, so it cannot drift from the code
# it grades. Against a checkout at any other revision the run is not a weaker
# check, it is a check of a different text: it would clear citations that are
# wrong and flag citations that are right. So a mismatched or dirty upstream
# tree SKIPs loudly. So does a missing one -- `third_party/` is untracked here
# and absent from every worktree, which is exactly the condition under which a
# silent skip would read as a pass.
#
# Runs in about two seconds, needs no docker and no oracle image.
#
#   tools/ci/verify-upstream-citations.sh
#
# `MOVEIT2_SRC` overrides where moveit2 is looked for; bullet3 is taken from
# `third_party/bullet3` because that is where the port's build expects it.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
# No -e, so a failed cd cannot leave every path below resolving against the
# caller's directory instead.
cd "$REPO_ROOT" || exit 1

CHECKER="tools/ci/upstream-citations.py"
MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"
BULLET3_SRC="${BULLET3_SRC:-$REPO_ROOT/third_party/bullet3}"

# The pin each upstream is cited at, read out of the port's provenance headers.
# `sort -u` over every header is the check that they agree: two revisions in
# this list means the port cites two texts and no single tree can grade it.
pinned_rev() {  # <label> -- e.g. moveit2
  rg -N -o --no-filename "$1 @ [0-9a-f]{40}" \
    crates/cspace-bullet/src crates/cspace-bullet-cast/src \
    crates/cspace-collision/src/bullet_ccd.rs 2>/dev/null |
    sed "s/^$1 @ //" | sort -u
}

# <label> <path> <expected rev> -> 0 when that tree is usable, 1 otherwise.
# Prints its own reason; the caller turns the first failure into the skip.
usable_tree() {
  local label="$1" path="$2" want="$3" got dirty
  if [ -z "$want" ]; then
    echo "SKIP  the port names no $label revision -- its provenance headers changed shape." >&2
    return 1
  fi
  if [ "$(printf '%s\n' "$want" | wc -l)" -ne 1 ]; then
    echo "SKIP  the port cites $label at more than one revision:" >&2
    printf '%s\n' "$want" | sed 's/^/        /' >&2
    return 1
  fi
  if [ ! -d "$path" ]; then
    echo "SKIP  $label is not at $path." >&2
    return 1
  fi
  got="$(git -C "$path" rev-parse HEAD 2>/dev/null)"
  if [ -z "$got" ]; then
    echo "SKIP  $path is not a git checkout, so its revision cannot be established." >&2
    return 1
  fi
  if [ "$got" != "$want" ]; then
    echo "SKIP  $label is at $got, the port cites $want." >&2
    echo "        Line numbers do not carry across revisions; this would grade a different text." >&2
    return 1
  fi
  dirty="$(git -C "$path" status --porcelain 2>/dev/null | head -3)"
  if [ -n "$dirty" ]; then
    echo "SKIP  $label is at the pinned revision but has uncommitted changes:" >&2
    printf '%s\n' "$dirty" | sed 's/^/        /' >&2
    echo "        An edited upstream file shifts the lines every citation counts." >&2
    return 1
  fi
  echo "ok   $label at $got ($path)"
  return 0
}

echo
echo "=== upstream citations in the continuous-collision port ==="
echo

want_moveit2="$(pinned_rev moveit2)"
want_bullet3="$(pinned_rev bullet3)"

blocked=0
usable_tree moveit2 "$MOVEIT2_SRC" "$want_moveit2" || blocked=1
usable_tree bullet3 "$BULLET3_SRC" "$want_bullet3" || blocked=1

if [ "$blocked" -ne 0 ]; then
  echo >&2
  skip_not_measured blocked \
    "no citation in the port was resolved by this run." \
    "this is not a pass."
fi

echo
# Not piped: a pipeline reports the filter's status, which is how a checker
# that never ran becomes a green gate.
python3 "$CHECKER" "$REPO_ROOT" "$MOVEIT2_SRC" "$BULLET3_SRC"
rc=$?

echo
if [ "$rc" -ne 0 ]; then
  echo "FAIL $CHECKER exited $rc -- see the citations named above." >&2
  exit 1
fi

echo "OK every upstream citation in the port resolves to one file and points at code."
