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
cd "$REPO_ROOT"

RESOURCE_DIR='third_party/moveit_resources'

# Kept in sync by hand with the `#[ignore = "needs third_party/moveit_resources"]`
# attributes in tools/moveit-diff. Two sites as of this writing; the sweep that
# established there are exactly two is in p1-fixtures' round-14 report. A new
# ignored test that needs the same resource must be added here, or it inherits
# exactly the never-runs problem this script closes.
TESTS=(
  near_placement_never_touches_more_than_one_link_at_once
  a_real_mismatching_case_touches_exactly_one_link
)

if [[ ! -d "$RESOURCE_DIR" ]]; then
  echo "SKIP $RESOURCE_DIR not present -- ${#TESTS[@]} vendored-fixture test(s) not run:"
  printf '  %s\n' "${TESTS[@]}"
  echo "SKIP this is not a pass; fetch the resource to cover them."
  exit 0
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
