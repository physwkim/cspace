#!/usr/bin/env bash
# Fails if a crate opts out of `[lints] workspace = true` and drops a lint the
# workspace sets, rather than restating it.
#
# Two crates legitimately cannot inherit wholesale: cspace-kinematics and
# cspace-planners-sbp both use `linkme::distributed_slice` for their D4
# registries, every such static expands to a `#[link_section]` item, and the
# workspace's `unsafe_code = "forbid"` cannot be downgraded per-site (forbid
# refuses even an `#[allow]`, and check-no-lint-suppression.sh would reject
# the attempt anyway). Both handle it correctly today: they restate every
# workspace lint and relax `unsafe_code` alone, each with the reasoning in the
# manifest.
#
# The risk is the next one. Opting out is a whole-table replacement -- a crate
# that writes `[lints.rust] unsafe_code = "allow"` and nothing else silently
# loses `warnings = "deny"` and `missing_docs`, and nothing fails: the CLI
# `-D warnings` in CI papers over the first, and the second produces no error
# at all. So the requirement is presence, not value: an opt-out must name
# every key the workspace names, which forces each divergence to be written
# down where the reason for it already lives.
#
# Run by CI via the `tools/ci/check-*.sh` glob. Needs no docker.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
require_caller_tree "$repo_root"
cd "$repo_root"

# Keys under a `[workspace.lints.<group>]` table in the root manifest.
workspace_keys() {
  awk -v want="[workspace.lints.$1]" '
    /^\[/ { in_tbl = ($0 == want); next }
    in_tbl && $0 !~ /^[[:space:]]*#/ && $0 ~ /=/ {
      split($0, a, "="); gsub(/[[:space:]]/, "", a[1]); print a[1]
    }
  ' Cargo.toml
}

# Keys under a `[lints.<group>]` table in a crate manifest.
crate_keys() {
  awk -v want="[lints.$2]" '
    /^\[/ { in_tbl = ($0 == want); next }
    in_tbl && $0 !~ /^[[:space:]]*#/ && $0 ~ /=/ {
      split($0, a, "="); gsub(/[[:space:]]/, "", a[1]); print a[1]
    }
  ' "$1"
}

status=0

for group in rust clippy; do
  if [ -z "$(workspace_keys "$group")" ]; then
    echo "Cargo.toml: no [workspace.lints.$group] table to compare against" >&2
    status=1
  fi
done

# Every tracked package manifest, not the `crates/*` + `tools/*` roots this
# used to name. `ros/cspace-ros` is a package that matches neither glob -- it
# is its own workspace root (D5), so it cannot inherit and is precisely the
# case this check exists for, and it was the one manifest never read. It had
# in fact dropped `missing_docs`, which is the exact scenario the header
# above describes. A crate outside the enumerated roots is the one most
# likely to have diverged, so enumerated roots are the wrong instrument.
#
# `[package]` is the filter rather than a path shape: the root manifest holds
# the `[workspace.lints]` tables being compared against and has no `[lints]`
# of its own to check, and a future virtual manifest would be the same.
manifests=()
while IFS= read -r manifest; do
  grep -q '^\[package\]' "$manifest" || continue
  manifests+=("$manifest")
done < <(git ls-files --deduplicate -- '*/Cargo.toml' | sort)
require_nonempty "${#manifests[@]}" "tracked package manifest"

for manifest in "${manifests[@]}"; do
  # The inheriting form: `[lints]` followed by `workspace = true`.
  if awk '/^\[lints\]/ { in_tbl = 1; next } /^\[/ { in_tbl = 0 } in_tbl' "$manifest" |
     grep -Eq '^[[:space:]]*workspace[[:space:]]*=[[:space:]]*true'; then
    continue
  fi

  # Not inheriting. It must then restate every workspace lint key, in both
  # groups -- a crate that defines neither table is the worst case, not an
  # exempt one, so this is not conditional on the table existing.
  for group in rust clippy; do
    # Checked substitution, not `done < <(workspace_keys "$group")`: process
    # substitution discards the producer's exit status, so a `workspace_keys`
    # failure would drive zero loop iterations -- zero per-key checks, no
    # diagnostic -- and this manifest's group would report clean having
    # examined nothing. The already-empty case (flagged above at line 52) is
    # unaffected: an empty `$keys` here-strings one blank line, which the
    # `-n` guard below already skips.
    if ! keys="$(workspace_keys "$group")"; then
      echo "$manifest: workspace_keys $group failed -- nothing was checked for this group" >&2
      status=1
      continue
    fi
    while IFS= read -r key; do
      [ -n "$key" ] || continue
      if ! crate_keys "$manifest" "$group" | grep -qx "$key"; then
        echo "$manifest: opts out of [lints] workspace = true but does not set" >&2
        echo "  [lints.$group] $key -- the workspace sets it, so it is silently dropped here." >&2
        echo "  Restate it (relaxing the value if that is the intent, with the reason)." >&2
        status=1
      fi
    done <<<"$keys"
  done
done

if [ "$status" -ne 0 ]; then
  exit 1
fi

echo "OK: none of the ${#manifests[@]} tracked package manifests silently drops a workspace lint"
