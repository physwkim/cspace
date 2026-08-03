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

exec docker run --rm -i \
  -v "$REPO_ROOT:$REPO_ROOT:ro" \
  -v "$MOVEIT2_SRC:$MOVEIT2_SRC:ro" \
  "$IMAGE" "$@"
