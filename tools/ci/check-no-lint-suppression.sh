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

cd "$(dirname "${BASH_SOURCE[0]}")/../.."

if ! command -v rg >/dev/null 2>&1; then
  echo "ripgrep (rg) not found" >&2
  exit 1
fi

# Attribute forms only: `#[allow(`, `#![allow(`, and the `expect` equivalents.
# This deliberately does not match `Option::expect` or `Result::expect`, which
# are ordinary calls and take a string literal rather than a lint name.
pattern='^\s*#!?\[\s*(allow|expect)\s*\('

if hits=$(rg -n --pcre2 "$pattern" --glob '*.rs' crates tools 2>/dev/null); then
  echo "lint suppression is not permitted -- fix the lint at source:" >&2
  echo "$hits" >&2
  exit 1
fi

echo "OK: no lint suppression in crates/ or tools/"
