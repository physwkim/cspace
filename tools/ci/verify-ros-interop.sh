#!/bin/bash
# Thin caller for ros/verify-ros-interop.sh, so tools/ci/verify-all.sh's
# glob reaches it.
#
# ros/moveit-ros lives outside the root workspace (D5, PORTING-PLAN.md
# §129.4) with its own `[workspace]` and its own docker-dependent gate
# script, so that script cannot itself live under tools/ci/ or match
# check-*.sh -- it needs docker, which check-*.sh's CI runners do not
# have. Round 17's audit found the consequence: with the script visible
# only from ros/, nothing invoked it at all -- the exact boundary that
# already bit this repo once, when ros/moveit-ros broke on `main` while
# every root-workspace gate (which cannot see into ros/'s separate
# `[workspace]`) stayed green. This file is the fix: a name under
# tools/ci/ that verify-all.sh's `verify-*.sh` glob reaches, whose only
# job is to run the real script.
#
# What this wiring does NOT close -- unchanged from ros/verify-ros-interop.sh's
# own "what this does NOT check" section: nothing plans (every live leg
# asserts a typed error, not a trajectory), no cross-workspace check against
# crates/ (a breaking crates/moveit-* API change is only caught here if
# ros/moveit-ros happens to use the changed symbol), no moveit_msgs
# schema-drift check, no oracle/fixture comparison. Root-workspace CI
# (ci.yml's check-*.sh glob) still never builds or tests ros/moveit-ros --
# D5's workspace separation is unaffected, only the never-runs gap is.
#
# "No live ROS 2 graph" used to head that list here and in the script itself.
# It stopped being true when §241 added the `/plan_kinematic_path` round trip
# and stayed wrong through §250's `/move_action` server; both copies are
# corrected now. A gate's own account of its coverage is the thing readers
# trust instead of re-deriving, so a stale one is worse than none.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
exec "$REPO_ROOT/ros/verify-ros-interop.sh" "$@"
