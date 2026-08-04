#!/usr/bin/env bash
# Fails if a crate opts out of `[lints] workspace = true` and drops a lint the
# workspace sets, rather than restating it.
#
# Two crates legitimately cannot inherit wholesale: moveit-kinematics and
# moveit-planners-sbp both use `linkme::distributed_slice` for their D4
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

mapfile -t manifests < <(git ls-files -- 'crates/*/Cargo.toml' 'tools/*/Cargo.toml' | sort)
require_nonempty "${#manifests[@]}" "crate manifest under crates/ or tools/"

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
    while IFS= read -r key; do
      [ -n "$key" ] || continue
      if ! crate_keys "$manifest" "$group" | grep -qx "$key"; then
        echo "$manifest: opts out of [lints] workspace = true but does not set" >&2
        echo "  [lints.$group] $key -- the workspace sets it, so it is silently dropped here." >&2
        echo "  Restate it (relaxing the value if that is the intent, with the reason)." >&2
        status=1
      fi
    done < <(workspace_keys "$group")
  done
done

if [ "$status" -ne 0 ]; then
  exit 1
fi

echo "OK: no crate silently drops a workspace lint"
