#!/bin/bash
# Resolves every `<file>.cpp:NNN` / `.hpp` / `.h` citation in the tracked
# `.md` and `.rs` files against the pinned upstream checkout, and fails on the
# ones that no longer land where the citing text says they do.
#
# This is the half `tools/ci/check-citation-drift.py` names in its own header
# as out of scope ("not resolvable without a local upstream checkout and are
# not covered here"), and it is where the drift is. Hand-checking five upstream
# citations during one merge found three wrong, one of them a citation that
# round had just corrected: `planning_scene.cpp:2451-2490` for
# `getCostSources`'s trajectory pair, whose two overloads are `2451-2455` and
# `2457-2491`. Three more `2451-2490`s were in the tree behind it, and nothing
# mechanical was looking at any of them.
#
# Deliberately NOT a `check-*.sh`: that glob is what `.github/workflows/ci.yml`
# runs, on a bare runner with nothing but this repository. This gate walks
# `$MOVEIT2_SRC`, an upstream checkout outside it -- the same reason
# `verify-port-coverage.sh` and `verify-upstream-license-provenance.sh` carry
# the `verify-` name. `tools/ci/verify-all.sh`'s glob picks it up.
#
# Absent checkout SKIPs, loudly, in `verify-mpr-vs-epa.sh`'s shape: a silent
# skip is indistinguishable from a pass. A checkout at the WRONG revision is a
# hard failure instead, and that asymmetry is the point -- every line number in
# this corpus is relative to one upstream revision, so checking a citation
# against a different one produces confident verdicts about a file nobody
# cited. That is worse than not checking, because it reads as coverage and
# argues for edits.
#
#   tools/ci/verify-upstream-citations.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"
if [[ ! -d "$MOVEIT2_SRC" ]]; then
  echo "SKIP $MOVEIT2_SRC not present -- no upstream citation was resolved."
  echo "SKIP this is not a pass; clone https://github.com/moveit/moveit2 there to cover them."
  exit 0
fi

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"

if ! head_sha="$(git -C "$MOVEIT2_SRC" rev-parse HEAD 2>/dev/null)"; then
  echo "FAIL $MOVEIT2_SRC is not a git checkout -- the pinned revision cannot be confirmed." >&2
  exit 1
fi
if [[ "$head_sha" != "$ORACLE_MOVEIT2_SHA" ]]; then
  echo "FAIL $MOVEIT2_SRC is at $head_sha, not the pinned $ORACLE_MOVEIT2_SHA." >&2
  echo "FAIL every upstream line number in this repository is relative to the pinned" >&2
  echo "FAIL revision; a citation checked against another one is worse than unchecked." >&2
  exit 1
fi

# The vendored packages this corpus cites by their `third_party/` path, each
# with the revision a tracked document already records for it. Same
# gitignored-external-checkout arrangement `verify-upstream-license-provenance.sh`
# and `verify-fixture-provenance.sh` depend on, and the same asymmetry as
# `$MOVEIT2_SRC` above: an absent checkout SKIPs loudly and leaves its
# citations in the unresolvable list the script already prints, a checkout at
# the WRONG revision is a hard failure. `third_party/moveit_msgs` and
# `third_party/moveit_resources` are deliberately not here: no tracked
# document records a revision for either, so there is nothing to check a
# checkout against, and neither is cited with a line number.
THIRD_PARTY_SRC="${THIRD_PARTY_SRC:-$REPO_ROOT/third_party}"
declare -A THIRD_PARTY_PINS=(
  # geometric_shapes 2.3.3      -- PORTING-PLAN.md:12828, crates/moveit-geometry/src/shapes.rs:15
  [geometric_shapes]=192801cebacc07d0e9f719576cdd1c9b36d0bc28
  # srdfdom 2.0.8               -- PORTING-PLAN.md:12829
  [srdfdom]=58ee1eccd1c34498f67022eb2080daec5e8bc162
  # octomap v1.9.7              -- PORTING-PLAN.md:12830, doc/claim-audit/moveit-octomap.md:8
  [octomap]=aa6372b87eaf7e89bb1c9421f61d58bd634477cb
  # orocos_kinematics_dynamics v1.5.1 -- doc/upstream-bugs.md:216, which records
  # the tag and the short `db25b7e` rather than a full SHA; this is the commit
  # `git rev-parse v1.5.1^{}` resolves to, and it satisfies both.
  [orocos_kinematics_dynamics]=db25b7e480e068df068232064f2443b8d52a83c7
)

sources=()
for pkg in "${!THIRD_PARTY_PINS[@]}"; do
  dir="$THIRD_PARTY_SRC/$pkg"
  pin="${THIRD_PARTY_PINS[$pkg]}"
  if [[ ! -d "$dir" ]]; then
    echo "SKIP $dir not present -- its citations stay in the unresolvable list below."
    echo "SKIP this is not a pass; clone it there at $pin to cover them."
    continue
  fi
  if ! pkg_sha="$(git -C "$dir" rev-parse HEAD 2>/dev/null)"; then
    echo "FAIL $dir is not a git checkout -- the pinned revision cannot be confirmed." >&2
    exit 1
  fi
  if [[ "$pkg_sha" != "$pin" ]]; then
    echo "FAIL $dir is at $pkg_sha, not the pinned $pin." >&2
    echo "FAIL the line numbers citing it are relative to that revision; checking them" >&2
    echo "FAIL against another one is worse than not checking them." >&2
    exit 1
  fi
  sources+=(--source "third_party/$pkg=$dir")
done

./tools/ci/measure-upstream-citations.py --upstream "$MOVEIT2_SRC" "${sources[@]}"
