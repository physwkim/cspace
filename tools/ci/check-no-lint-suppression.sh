#!/bin/bash
# No lint may be silenced; it must be fixed at source.
#
# This exists because the failure it catches is invisible in a green build:
# `cargo clippy -- -D warnings` passes just as cleanly over an `#[allow(...)]`
# as over code that has no warning to begin with, so a suppression added in one
# change is never surfaced again by any later run. Twice now a suppression has
# been standing in for a real defect -- once for a transcribed constant that
# should have been a fixture, once for an eight-argument constructor whose
# interchangeable `size_*`/`origin_*` floats were the actual hazard the lint was
# pointing at.
#
# `expect` is covered too: it is `allow` that warns when unused, which makes it
# the obvious way to sidestep a grep for `allow`.
#
# There is no escape hatch on purpose. If a lint is genuinely wrong for this
# codebase, turn it off deliberately and once in [workspace.lints] with a
# comment saying why, rather than scattering per-site suppressions.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

# Attribute forms only: `#[allow(`, `#![allow(`, and the `expect` equivalents.
# This deliberately does not match `Option::expect` or `Result::expect`, which
# are ordinary calls and take a string literal rather than a lint name.
pattern='^\s*#!?\[\s*(allow|expect)\s*\('

# grep -P, not rg: this pattern is plain PCRE (\s, alternation, anchors) with
# no rg-only feature, and GNU grep's -P is built on the same libpcre2 that
# ships in Ubuntu's base image -- unlike ripgrep, which `ubuntu-latest`
# runners do not have preinstalled. `check-pilz-tolerance-overrides.sh`
# states the same principle for its own awk/grep/sed parse: don't require a
# tool the gate doesn't structurally need. `--include='*.rs'` over `crates`
# and `tools` matches rg's `--glob '*.rs' crates tools` scope; there is no
# per-crate `target/` under either directory to filter (`.gitignore` anchors
# it at `/target`), so grep's lack of gitignore-awareness changes nothing
# here -- verified equal hit counts against `rg` before this rewrite.
#
# grep's three exit codes are three different answers and only two of them
# are this check's business: 0 is "found suppressions", 1 is "found none",
# and 2 is "grep itself failed" (unreadable path, bad pattern). Testing the
# command in an `if` collapses 1 and 2 into the same branch, so a broken
# search would print OK -- the whole failure mode this file exists to
# prevent, one level up.
status=0
hits=$(grep -rn --include='*.rs' -P "$pattern" crates tools) || status=$?
case "$status" in
  0)
    echo "lint suppression is not permitted -- fix the lint at source:" >&2
    echo "$hits" >&2
    exit 1
    ;;
  1) ;;  # no matches: the pass case
  *)
    echo "grep failed (exit $status) -- this check did not run" >&2
    exit "$status"
    ;;
esac

echo "OK: no lint suppression in crates/ or tools/"
