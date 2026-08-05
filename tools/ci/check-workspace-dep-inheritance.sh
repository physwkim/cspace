#!/usr/bin/env bash
# Fails if one workspace member depends on another without going through
# `[workspace.dependencies]`.
#
# Three crates declared their sibling inline -- `moveit-trajectory = { path =
# "../moveit-trajectory", version = "0.1.0" }` in moveit-planners-chomp and
# moveit-planners-stomp, `moveit-scene` likewise in moveit-planners-sbp --
# while every other inter-crate edge went through the root table. Both forms
# build, so nothing surfaced the split. It matters on a version bump: the
# table entries move in one edit, an inline `version = "0.1.0"` silently does
# not, and `cargo publish` then resolves a sibling to a version that no longer
# exists. It also means the root manifest stops being a complete picture of
# the crate graph, which is what it is read for.
#
# Same shape as check-license-matches-upstream.sh: the rule is derived from
# the tree (any dependency key naming a `crates/*` or `tools/*` member), not
# from a list this script would have to be taught about each new crate.
#
# Run by CI via the `tools/ci/check-*.sh` glob. Needs no docker.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
cd "$repo_root"

# Every local member's package name, taken from its directory name -- which
# this repo keeps equal to the package name (a mismatch would show up as an
# unrecognised dependency key below, i.e. as a miss, not a false alarm).
mapfile -t members < <(
  git ls-files --deduplicate -- 'crates/*/Cargo.toml' 'tools/*/Cargo.toml' |
    sed 's|/Cargo.toml$||' | sed 's|.*/||' | sort -u
)
require_nonempty "${#members[@]}" "workspace member under crates/ or tools/"

mapfile -t manifests < <(git ls-files --deduplicate -- 'crates/*/Cargo.toml' 'tools/*/Cargo.toml' | sort)
require_nonempty "${#manifests[@]}" "crate manifest under crates/ or tools/"

status=0

for manifest in "${manifests[@]}"; do
  # Cargo accepts two spellings for the same edge, so both are scanned:
  #
  #   [dependencies]                    <- key form
  #   moveit-scene = { path = "..." }
  #
  #   [dependencies.moveit-scene]       <- sub-table form
  #   path = "..."
  #
  # awk emits one `<member> <line>` record per offending declaration. The
  # sub-table form is only reported once its whole table has been read,
  # because `workspace = true` inside it is the accepted spelling too.
  #
  # Checked substitution, not `done < <(printf ... | awk ... "$manifest")`:
  # process substitution discards the producer's exit status, so an awk
  # read/parse failure on `$manifest` would drive zero loop iterations --
  # zero inline-dependency findings -- and this manifest would report clean
  # having examined nothing. An empty `$records` here-strings one blank
  # line, which the `-n` guard below skips, so the genuinely-clean case is
  # unaffected.
  if ! records="$(
    printf '%s\n' "${members[@]}" |
    awk '
      NR == FNR { member[$0] = 1; next }

      # A pending sub-table ends at the next header or at EOF.
      function flush_subtable() {
        if (sub_member != "" && !sub_inherits)
          print sub_member " [dependencies." sub_member "]"
        sub_member = ""; sub_inherits = 0
      }

      /^\[/ {
        flush_subtable()
        in_deps = ($0 ~ /dependencies\]$/)
        # [dependencies.<name>] / [dev-dependencies.<name>] /
        # [target.<cfg>.dependencies.<name>]
        if (match($0, /dependencies\.[^]]+\]$/)) {
          name = substr($0, RSTART, RLENGTH)
          sub(/^dependencies\./, "", name)
          sub(/\]$/, "", name)
          if (name in member) { sub_member = name; sub_inherits = 0 }
          in_deps = 0
        }
        next
      }

      /^[[:space:]]*#/ { next }

      sub_member != "" && /^[[:space:]]*workspace[[:space:]]*=[[:space:]]*true/ {
        sub_inherits = 1; next
      }

      in_deps && /=/ {
        key = $0
        sub(/=.*$/, "", key)
        gsub(/[[:space:]]/, "", key)
        if (key ~ /\.workspace$/) next   # `foo.workspace = true` -- the wanted form
        if (key in member) print key " " $0
      }

      END { flush_subtable() }
    ' - "$manifest"
  )"; then
    echo "$manifest: inline-dependency scan failed -- nothing was checked for this manifest" >&2
    status=1
    continue
  fi
  while IFS= read -r record; do
    [ -n "$record" ] || continue
    m="${record%% *}"
    line="${record#* }"
    echo "$manifest: '$m' is a workspace member declared inline:" >&2
    echo "    $line" >&2
    echo "  Add it to [workspace.dependencies] in the root Cargo.toml and" >&2
    echo "  write '$m.workspace = true' here." >&2
    status=1
  done <<<"$records"
done

if [ "$status" -ne 0 ]; then
  exit 1
fi

echo "OK: every inter-member dependency goes through [workspace.dependencies]"
