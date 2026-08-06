#!/bin/bash
# Phase 5's second completion condition, as a command: 10,000 states drawn
# from `moveit-constraints`' samplers, each fed back through the `decide()`
# of the constraints its sampler was configured from.
#
# The measurement itself is
# `crates/moveit-constraints/tests/sampler_self_validation.rs`'s
# `every_sampled_state_satisfies_its_own_constraints`, which is `#[ignore]`d
# on cost (10.2-13.1s across three runs, against 0.7s for that crate's
# other 103 tests together).
# This script is what runs it, for the reason that file's own doc comment
# gives and `moveit-scene/tests/cost_sources_parity.rs` states as the rule:
# a test left `#[ignore]`d with nothing invoking it never runs again, which
# reads as coverage while providing none.
#
# Named `verify-*.sh` so `tools/ci/verify-all.sh`'s glob picks it up per
# merge round with no list to keep in sync. Unlike its neighbours it needs
# neither docker nor `third_party/` -- it is a pure-Rust sweep over this
# crate's own committed fixtures -- but it is still not a `check-*.sh`:
# ~12s is real time to add to every CI push, and the `verify-*.sh` set is
# where this repo already puts per-round sweeps that the per-push set does
# not carry.
#
#   tools/ci/verify-sampler-self-validation.sh
#
# Exits non-zero if any sampler fails to produce its share of the 10,000 or
# produces a state that violates its own constraints.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

# `--no-capture` so the per-sampler attempted/produced/satisfied table
# reaches the caller on a *passing* run too: the completion condition is a
# set of numbers, and a bare "1 test passed" is not those numbers. It also
# forces nextest to run the test in this process's own foreground, which is
# what keeps the table interleaved with the verdict rather than buffered
# behind it.
exec cargo nextest run \
  -p moveit-constraints \
  --run-ignored all \
  --no-capture \
  -E 'test(every_sampled_state_satisfies_its_own_constraints)'
