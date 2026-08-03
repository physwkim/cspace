#!/bin/bash
# Runs the oracle container as a plain stdin/stdout filter for moveit-diff.
#
# --urdf/--srdf paths are host paths; the repo root and the moveit2 checkout are
# mounted read-only at the same paths inside the container so the caller does
# not have to think about translation.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"
IMAGE="${IMAGE:-moveit-rs/oracle:latest}"

# An image built from older oracle sources answers with old behaviour and no
# error, so a fixture captured against it is wrong in a way nothing downstream
# can see. Compare the source digest the image was stamped with (see the
# Dockerfile) against the working tree before handing it any work.
# Paths must be relative: sha256sum hashes the name alongside the bytes, and
# the sources sit at /ws/oracle-src in the image but under the repo here.
want="$(cd "$REPO_ROOT/tools/moveit-oracle" \
        && find . -type f \
             \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name 'CMakeLists.txt' \) \
             -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1)"
have="$(docker run --rm --entrypoint cat "$IMAGE" /usr/local/share/oracle-src.sha256 2>/dev/null || true)"
if [[ "$have" != "$want" ]]; then
  echo "$IMAGE was built from different oracle sources than the working tree" >&2
  echo "  image: ${have:-<unstamped, predates this check>}" >&2
  echo "  tree:  $want" >&2
  echo "rebuild with tools/moveit-oracle/build.sh" >&2
  exit 1
fi

exec docker run --rm -i \
  -v "$REPO_ROOT:$REPO_ROOT:ro" \
  -v "$MOVEIT2_SRC:$MOVEIT2_SRC:ro" \
  "$IMAGE" "$@"
