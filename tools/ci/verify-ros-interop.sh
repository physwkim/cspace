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
# own "what this does NOT check" section: no live ROS 2 graph, no
# cross-workspace check against crates/ (a breaking crates/moveit-* API
# change is only caught here if ros/moveit-ros happens to use the changed
# symbol), no moveit_msgs schema-drift check, no oracle/fixture
# comparison. Root-workspace CI (ci.yml's check-*.sh glob) still never
# builds or tests ros/moveit-ros -- D5's workspace separation is
# unaffected, only the never-runs gap is.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "$REPO_ROOT/ros/verify-ros-interop.sh" "$@"
