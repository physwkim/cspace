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

# The corpus is every tracked `.rs` file, asked of git, rather than a root
# list written here. This check spent its whole life reading `crates tools`,
# which excluded `ros/` -- outside the root workspace by design, 28 tracked
# `.rs` files, and the location of the only `#[allow(...)]` in the tree. The
# gate passed by not looking where the suppression was, and would have kept
# passing: a hand-maintained root list acquires that blind spot the moment a
# directory is added, and nothing about a green run distinguishes "searched
# and found none" from "did not search". `git ls-files` is also gitignore-free
# by construction, so the `target/` reasoning the old scope needed is gone.
#
# The process substitution cannot swallow a git failure here: it would yield
# an empty list, and `require_nonempty` fails the gate on exactly that.
rs_files=()
while IFS= read -r -d '' f; do rs_files+=("$f"); done < <(git ls-files --deduplicate -z -- '*.rs')
require_nonempty "${#rs_files[@]}" "tracked .rs files to search"

# grep -P, not rg: this pattern is plain PCRE (\s, alternation, anchors) with
# no rg-only feature, and GNU grep's -P is built on the same libpcre2 that
# ships in Ubuntu's base image -- unlike ripgrep, which `ubuntu-latest`
# runners do not have preinstalled. `check-pilz-tolerance-overrides.sh`
# states the same principle for its own awk/grep/sed parse: don't require a
# tool the gate doesn't structurally need. `-H` because grep omits the
# filename when handed exactly one path, and the list's length is derived
# rather than known here.
#
# grep's three exit codes are three different answers and only two of them
# are this check's business: 0 is "found suppressions", 1 is "found none",
# and 2 is "grep itself failed" (unreadable path, bad pattern). Testing the
# command in an `if` collapses 1 and 2 into the same branch, so a broken
# search would print OK -- the whole failure mode this file exists to
# prevent, one level up.
status=0
hits=$(grep -nH -P "$pattern" -- "${rs_files[@]}") || status=$?
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

echo "OK: no lint suppression in any of the ${#rs_files[@]} tracked .rs files"
