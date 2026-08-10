#!/usr/bin/env bash
# Every per-case `*_TOLERANCE` constant in pilz_blend_parity.rs must have a
# companion `#[should_panic]` test proving the case's own measured
# divergence exceeds `Tolerances::SHARED` in that channel.
#
# The defect this catches: an override constant that is looser than the
# shared constant it replaces, for a case whose own real divergence never
# needed the extra room -- a hole that quietly turns off regression
# detection in that channel for that case. A round of manual audit found two
# of these (`CORNER60_ACCELERATION_TOLERANCE`,
# `CORNER110_VELOCITY_TOLERANCE`, both since removed). That audit read doc
# prose and a measured-max number recorded there by hand -- sound at the
# time, but the doc comment can drift out of sync with the code the next
# time either changes, and nothing would notice.
#
# This check does not re-derive that audit from prose, on purpose: parsing
# an English "measures `X`, above `Y`" sentence and trusting the parsed
# number is a checker that fails toward silence the moment the prose
# doesn't match the pattern expected (see this repo's own "Checkers fail
# toward silence" lesson). Instead it enforces a structural precondition
# that makes the same fact executable: every override constant
# `<CASE>_<CHANNEL>_TOLERANCE` must have a companion
# `blend_panda_arm_<case>_needs_its_own_<channel>_tolerance` test, annotated
# `#[should_panic]`, that drops that one channel back to the shared
# tolerance (holding every other tolerance the case actually needs) and
# asserts the comparison then fails. `cargo nextest run -p cspace-planners`
# running that test green is the actual audit, re-run on every test
# invocation against today's real divergence, not a number copied into a
# comment once. This check only verifies the companion test exists with the
# right name and attribute -- it cannot verify the test body asserts the
# right tolerances, that is ordinary code review same as any other test.
#
# Run by CI via the `tools/ci/check-*.sh` glob. Needs no docker.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$repo_root/tools/ci/gate-lib.sh"

require_caller_tree "$repo_root"
cd "$repo_root"

file="crates/cspace-planners/tests/pilz_blend_parity.rs"

if [[ ! -f "$file" ]]; then
  echo "FAIL: $file not found -- did it move?" >&2
  exit 1
fi

# No `command -v rg` guard here, deliberately: this script parses with awk,
# grep and sed and never invokes ripgrep, so requiring it would abort the
# gate on any host without ripgrep installed for a reason that has nothing
# to do with what the gate checks. No `check-*` gate needs such a guard any
# more: `check-no-lint-suppression.sh` was the last one that called ripgrep,
# and it now uses `grep -P` -- the GitHub runner has no ripgrep, so its
# guard was the whole `ci checks` step's exit 1 on two consecutive runs.

# The four base constants `Tolerances::SHARED` names -- not a hardcoded
# list: read out of the `const SHARED: Self = Self { ... };` block itself,
# so a renamed or added SHARED field is picked up without editing this
# script.
shared_block=$(awk '
  /const SHARED: Self = Self \{/ { in_block = 1 }
  in_block { print }
  in_block && /\};/ { exit }
' "$file")
mapfile -t shared_names < <(grep -oE '[A-Z0-9_]+_TOLERANCE' <<<"$shared_block" | sort -u)
if [[ ${#shared_names[@]} -eq 0 ]]; then
  echo "FAIL: could not find Tolerances::SHARED's field list in $file -- did its shape change?" >&2
  exit 1
fi

# Every `*_TOLERANCE` constant declared in the file.
mapfile -t all_names < <(grep -oE '^const [A-Z0-9_]+_TOLERANCE' "$file" | awk '{print $2}' | sort -u)
if [[ ${#all_names[@]} -eq 0 ]]; then
  echo "FAIL: found zero \`const *_TOLERANCE\` declarations in $file -- did the naming convention change?" >&2
  exit 1
fi

# Override constants = all constants minus the shared four.
override_names=()
for name in "${all_names[@]}"; do
  is_shared=0
  for shared in "${shared_names[@]}"; do
    [[ "$name" == "$shared" ]] && is_shared=1 && break
  done
  [[ "$is_shared" -eq 0 ]] && override_names+=("$name")
done

# Zero overrides is a legitimate state, not a parse failure: it means no
# case's own divergence exceeds a shared constant, so no case needs a hole.
# It was reached once already, when the port's blend samples started being
# rotated by `Eigen::Quaterniond::slerp` instead of nalgebra's and all ten
# overrides became unnecessary at once. The "did the naming convention
# change?" question this used to fail on is answered by the `all_names`
# check above instead, which cannot be satisfied by an empty file.
if [[ ${#override_names[@]} -eq 0 ]]; then
  echo "OK: $file declares no per-case tolerance override, so there is none to justify"
  exit 0
fi

missing=()
unparseable=()
for name in "${override_names[@]}"; do
  # Split `<PREFIX>_<CHANNEL>_TOLERANCE` on the trailing channel word --
  # the same four words the SHARED block's own fields use.
  channel=""
  for c in POSITION VELOCITY ACCELERATION TIME; do
    if [[ "$name" == *"_${c}_TOLERANCE" ]]; then
      channel="$c"
      prefix="${name%_"${c}"_TOLERANCE}"
      break
    fi
  done
  if [[ -z "$channel" ]]; then
    unparseable+=("$name")
    continue
  fi

  case_suffix="$(tr '[:upper:]' '[:lower:]' <<<"$prefix")"
  channel_suffix="$(tr '[:upper:]' '[:lower:]' <<<"$channel")"
  expected_fn="blend_panda_arm_${case_suffix}_needs_its_own_${channel_suffix}_tolerance"

  fn_line=$(grep -n "^fn ${expected_fn}(" "$file" | head -1 | cut -d: -f1) || true
  if [[ -z "$fn_line" ]]; then
    missing+=("$name -> expected test fn ${expected_fn}() not found")
    continue
  fi

  # `#[should_panic]` must appear somewhere in the (small) attribute block
  # directly above the fn -- scan back up to 5 lines, stopping at the first
  # non-attribute, non-blank line.
  window_start=$((fn_line - 5))
  [[ "$window_start" -lt 1 ]] && window_start=1
  attrs=$(sed -n "${window_start},$((fn_line - 1))p" "$file")
  if ! grep -q '#\[should_panic' <<<"$attrs"; then
    missing+=("$name -> ${expected_fn}() exists but is not #[should_panic]")
  fi
done

if [[ ${#unparseable[@]} -gt 0 ]]; then
  echo "FAIL: override constant(s) whose name does not end in a known channel word (position/velocity/acceleration/time):" >&2
  for name in "${unparseable[@]}"; do
    echo "  $name" >&2
  done
  exit 1
fi

if [[ ${#missing[@]} -gt 0 ]]; then
  echo "FAIL: override constant(s) with no #[should_panic] necessity test:" >&2
  for entry in "${missing[@]}"; do
    echo "  $entry" >&2
  done
  echo "Add a #[should_panic] test named blend_panda_arm_<case>_needs_its_own_<channel>_tolerance that drops this constant's channel to the shared tolerance and asserts the comparison then fails -- see the existing *_needs_its_own_*_tolerance tests for the pattern." >&2
  exit 1
fi

echo "OK: all ${#override_names[@]} per-case tolerance overrides in $file have a #[should_panic] necessity test"
