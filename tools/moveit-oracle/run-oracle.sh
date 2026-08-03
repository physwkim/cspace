#!/bin/bash
# Runs the oracle container as a plain stdin/stdout filter for moveit-diff.
#
# --urdf/--srdf paths are host paths; the repo root and the moveit2 checkout are
# mounted read-only at the same paths inside the container so the caller does
# not have to think about translation.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"

# An image built from older oracle sources answers with old behaviour and no
# error, so a fixture captured against it is wrong in a way nothing downstream
# can see. The tag is derived from this tree's oracle sources, so a worktree
# with different sources resolves to a different image rather than to whatever
# another worktree built last.
want="$(oracle_src_digest "$REPO_ROOT/tools/moveit-oracle")"
IMAGE="${IMAGE:-$(oracle_image_tag "$want")}"

# The tag alone would be enough if tags were immutable; they are not, so the
# stamp inside the image is still what is trusted. This also catches an image
# predating the stamp entirely.
have="$(docker run --rm --entrypoint cat "$IMAGE" /usr/local/share/oracle-src.sha256 2>/dev/null || true)"
if [[ "$have" != "$want" ]]; then
  echo "$IMAGE was built from different oracle sources than the working tree" >&2
  echo "  image: ${have:-<missing or unstamped>}" >&2
  echo "  tree:  $want" >&2
  echo "rebuild with tools/moveit-oracle/build.sh" >&2
  exit 1
fi

exec docker run --rm -i \
  -v "$REPO_ROOT:$REPO_ROOT:ro" \
  -v "$MOVEIT2_SRC:$MOVEIT2_SRC:ro" \
  "$IMAGE" "$@"
