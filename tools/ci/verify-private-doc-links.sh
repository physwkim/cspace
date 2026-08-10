#!/bin/bash
# Runs the doc build again with `--document-private-items`, which is the
# only way rustdoc's own link lints reach a private module's `//!` header.
#
# PORTING-PLAN.md §198 recorded this as an exposure with no cheap closure:
# `cargo doc` checks intra-doc links only for the items it documents, and a
# private `mod foo;` is not one of them -- so `crates/cspace-collision/src/parry.rs`,
# whose module doc is one of the longest in the tree, had every bracket link
# in it unchecked by any gate. There *is* a cheap closure for that half: this
# script. It found 36 broken links on the first run.
#
# The `#[cfg(test)]` half of §198 stays open and is NOT closed by this
# script. Adding `--cfg test` to RUSTDOCFLAGS makes rustdoc see those
# modules but the build then fails on unresolved dev-dependency imports
# (`approx`, `rand_chacha`, `cspace_core::sampling`, ...), because a doc build
# does not link dev-dependencies. Do not "fix" that by deleting the flag
# and calling the gap closed.
#
# Deliberately NOT named `check-*.sh`: it is a second full doc build of the
# workspace, which roughly doubles the doc-gate cost, and `ci.yml` already
# runs the public-item build.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

if ! RUSTDOCFLAGS="--document-private-items" cargo doc --workspace --no-deps; then
  echo "FAIL rustdoc rejected a link reachable only with --document-private-items." >&2
  echo "FAIL these are real broken links; the public-item doc gate cannot see them." >&2
  exit 1
fi

echo "OK private-item doc links resolve"
