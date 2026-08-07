#!/bin/bash
# Runs the oracle container as a plain stdin/stdout filter for moveit-diff.
#
# --urdf/--srdf paths are host paths; the repo root and the moveit2 checkout are
# mounted read-only at the same paths inside the container so the caller does
# not have to think about translation.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"

# An image built from older oracle sources answers with old behaviour and no
# error, so a fixture captured against it is wrong in a way nothing downstream
# can see. The tag is derived from this tree's oracle sources *and* the
# resolved non-file build inputs (see `oracle_stamp`), so a worktree with
# different sources -- or the same sources built against a different base
# image or package list -- resolves to a different image rather than to
# whatever another worktree built last.
want="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
IMAGE="${IMAGE:-$(oracle_image_tag "$want")}"

# The tag alone would be enough if tags were immutable; they are not, so the
# stamp inside the image is still what is trusted. This also catches an image
# predating the stamp entirely -- and, unlike the single `have` string this
# used to compare, it says which of those it was: every caller here has to
# rebuild for some of the causes and join the `docker` group for another, and
# one message cannot prescribe both.
stamp="$(oracle_stamp_verdict "$IMAGE" "$want")"
if [ "$stamp" != ok ]; then
  oracle_stamp_explain "$stamp" "$IMAGE" "$want"
  exit 1
fi

# Caucus worktrees keep `third_party/` (gitignored fixture data: moveit_msgs,
# moveit_resources, orocos_kinematics_dynamics) as a symlink into the shared
# checkout at the session root, to avoid every worktree cloning it separately.
# A bind mount of $REPO_ROOT does not follow that symlink to its target, so
# the target needs its own mount whenever it resolves outside $REPO_ROOT.
THIRD_PARTY_REAL="$(readlink -f "$REPO_ROOT/third_party" 2>/dev/null || true)"
extra_mounts=()
if [[ -n "$THIRD_PARTY_REAL" && "$THIRD_PARTY_REAL" != "$REPO_ROOT/third_party" ]]; then
  extra_mounts=(-v "$THIRD_PARTY_REAL:$THIRD_PARTY_REAL:ro")
fi

exec docker run --rm -i \
  -v "$REPO_ROOT:$REPO_ROOT:ro" \
  -v "$MOVEIT2_SRC:$MOVEIT2_SRC:ro" \
  "${extra_mounts[@]}" \
  "$IMAGE" "$@"
