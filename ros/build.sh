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

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"

IMAGE="${IMAGE:-moveit-rs/ros-dev:latest}"

docker build \
  --build-arg "BASE_IMAGE=$ORACLE_BASE_IMAGE" \
  -t "$IMAGE" \
  -f "$REPO_ROOT/ros/Dockerfile" \
  "$REPO_ROOT/ros"

echo "built $IMAGE from BASE_IMAGE=$ORACLE_BASE_IMAGE"
