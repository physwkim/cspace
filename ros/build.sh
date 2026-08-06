#!/bin/bash
# Builds the ROS 2 Rolling + Rust image used to build/test ros/moveit-ros.
#
# BASE_IMAGE is read from tools/moveit-oracle/src-digest.sh's
# ORACLE_BASE_IMAGE rather than repeated here -- a second copy of that digest
# is a second place it can drift when someone repins it. Sourcing the file
# reads a shared constant; it does not copy the oracle's scripts (see
# tools/ci/check-audit-scripts-not-copied.sh, which flags copied *audit*
# commands specifically -- this reads one variable).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
MOVEIT_MSGS_SRC="${MOVEIT_MSGS_SRC:-$REPO_ROOT/third_party/moveit_msgs}"

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"

IMAGE="${IMAGE:-moveit-rs/ros-dev:latest}"

if [[ ! -d "$MOVEIT_MSGS_SRC/.git" ]]; then
  echo "moveit_msgs reference checkout not found at $MOVEIT_MSGS_SRC" >&2
  echo "check it out (git clone https://github.com/moveit/moveit_msgs.git)," >&2
  echo "or override with MOVEIT_MSGS_SRC=<path>" >&2
  exit 1
fi

# Same export-at-HEAD pattern as tools/moveit-oracle/build.sh, and for the
# same reason: a dirty third_party/moveit_msgs working tree must not leak
# into this image, and .git must stay out of the build context.
CTX="$(mktemp -d "$REPO_ROOT/.ros-ctx.XXXXXX")"
trap 'rm -rf "$CTX"' EXIT
trap 'exit 130' INT

mkdir -p "$CTX/moveit_msgs"
git -C "$MOVEIT_MSGS_SRC" archive HEAD | tar -x -C "$CTX/moveit_msgs"
cp "$REPO_ROOT/ros/Dockerfile" "$REPO_ROOT/ros/entrypoint.sh" "$CTX/"

docker build \
  --build-arg "BASE_IMAGE=$ORACLE_BASE_IMAGE" \
  -t "$IMAGE" \
  -f "$CTX/Dockerfile" \
  "$CTX"

echo "built $IMAGE from BASE_IMAGE=$ORACLE_BASE_IMAGE, moveit_msgs@$(git -C "$MOVEIT_MSGS_SRC" rev-parse HEAD)"
