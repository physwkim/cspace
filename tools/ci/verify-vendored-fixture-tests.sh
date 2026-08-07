#!/bin/bash
# Runs the tests that `#[ignore]` themselves because they need
# `third_party/moveit_resources`, which `.gitignore` excludes wholesale so a
# fresh clone genuinely lacks it.
#
# PORTING-PLAN.md §197.3: an unconditional `#[ignore]` cannot tell "the
# resource is absent" from "the resource is present", so on every machine that
# *can* run these they never do -- the same defect §184 removed elsewhere,
# wearing a precondition as a disguise. A runtime early-return inside the tests
# is not the fix: that manufactures §196's vacuous pass, where green means
# "there was nothing to check". Gating from outside keeps the tests honest and
# keeps a bare clone buildable.
#
# The skip path prints and is visible in the output on purpose. A silent skip
# is indistinguishable from a real pass, which is the whole failure this script
# exists to avoid.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

RESOURCE_DIR='third_party/moveit_resources'

# Derived from the `#[ignore = "needs third_party/moveit_resources"]`
# attributes themselves, not kept in sync by hand: a hand-typed list and the
# attributes it shadows drift, and the drift is silent in the direction that
# matters (a new ignored test simply never runs -- the exact never-runs
# problem this script exists to close, one level up). The run set and the
# derived set are now the same set by construction.
#
# The `--type rust` restriction is load-bearing: without it the count below
# also matches this comment's own quotation of the attribute, and the check
# passes for the wrong reason (found by p1-fixtures running it both ways).
IGNORE_ATTR='ignore = "needs third_party/moveit_resources'
mapfile -t TESTS < <(
  rg -A1 --no-heading --no-line-number --type rust -e "$IGNORE_ATTR" . \
    | rg -oP 'fn \K\w+(?=\()'
)

# The derivation assumes `fn` is the line right after the attribute -- true at
# both current sites. A site with another attribute wedged between the two
# would under-derive that one name, so the counts are compared rather than
# trusted: an under-derivation fails here loudly instead of quietly shrinking
# coverage.
attr_count="$(rg -o --type rust -e "$IGNORE_ATTR" . | wc -l)"
if [[ ${#TESTS[@]} -ne "$attr_count" ]]; then
  echo "FAIL derived ${#TESTS[@]} test name(s) from $attr_count ignore attribute(s):" >&2
  printf '  %s\n' "${TESTS[@]}" >&2
  echo "FAIL an attribute is not immediately followed by its own 'fn' line." >&2
  exit 1
fi
if [[ ${#TESTS[@]} -eq 0 ]]; then
  echo "FAIL no '$IGNORE_ATTR...' attribute found at all -- the tests this script" >&2
  echo "FAIL covers were renamed, moved, or deleted; passing here would be vacuous." >&2
  exit 1
fi

if [[ ! -d "$RESOURCE_DIR" ]]; then
  echo "SKIP (blocked) $RESOURCE_DIR not present -- ${#TESTS[@]} vendored-fixture test(s) not run:"
  printf '  %s\n' "${TESTS[@]}"
  skip_not_measured blocked "this is not a pass; fetch the resource to cover them."
fi

filter=""
for t in "${TESTS[@]}"; do
  [[ -n "$filter" ]] && filter+=" or "
  filter+="test(${t})"
done

echo "running ${#TESTS[@]} vendored-fixture test(s) against $RESOURCE_DIR"
if ! cargo nextest run -p moveit-diff --run-ignored all --release -E "$filter"; then
  echo "FAIL vendored-fixture tests failed" >&2
  exit 1
fi

# A nextest filter that matches nothing still exits 0, so a renamed or deleted
# test would silently stop being covered while this script kept printing OK --
# the same never-runs failure it exists to close, one level up. Confirm each
# name individually rather than trusting the aggregate run above.
listing="$(cargo nextest list -p moveit-diff --run-ignored all -E "$filter" --color never 2>/dev/null)"
missing=()
for t in "${TESTS[@]}"; do
  grep -qF -- "$t" <<<"$listing" || missing+=("$t")
done
if [[ ${#missing[@]} -gt 0 ]]; then
  echo "FAIL these test(s) no longer exist -- renamed or removed, and no longer covered:" >&2
  printf '  %s\n' "${missing[@]}" >&2
  exit 1
fi

echo "OK ${#TESTS[@]} vendored-fixture test(s) passed"
