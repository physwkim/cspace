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
cd "$repo_root"

# Every local member's package name, taken from its directory name -- which
# this repo keeps equal to the package name (a mismatch would show up as an
# unrecognised dependency key below, i.e. as a miss, not a false alarm).
mapfile -t members < <(
  git ls-files -- 'crates/*/Cargo.toml' 'tools/*/Cargo.toml' |
    sed 's|/Cargo.toml$||' | sed 's|.*/||' | sort -u
)

status=0

for manifest in $(git ls-files -- 'crates/*/Cargo.toml' 'tools/*/Cargo.toml' | sort); do
  while IFS= read -r line; do
    key="${line%%=*}"
    key="$(printf '%s' "$key" | tr -d '[:space:]')"
    # `foo.workspace = true` arrives here as key `foo.workspace`; that is the
    # form we want, so strip it before matching and let it pass.
    case "$key" in
      *.workspace) continue ;;
    esac
    for m in "${members[@]}"; do
      if [ "$key" = "$m" ]; then
        echo "$manifest: '$m' is a workspace member declared inline:" >&2
        echo "    $line" >&2
        echo "  Add it to [workspace.dependencies] in the root Cargo.toml and" >&2
        echo "  write '$m.workspace = true' here." >&2
        status=1
      fi
    done
  done < <(
    # Dependency-table lines only: everything from a [*dependencies*] header
    # until the next section header.
    awk '
      /^\[/ { in_deps = ($0 ~ /dependencies\]/); next }
      in_deps && $0 !~ /^[[:space:]]*#/ && $0 ~ /=/ { print }
    ' "$manifest"
  )
done

if [ "$status" -ne 0 ]; then
  exit 1
fi

echo "OK: every inter-member dependency goes through [workspace.dependencies]"
