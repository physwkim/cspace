#!/usr/bin/env bash
# Fails if a crate's declared license disagrees with the SPDX identifier its
# own source files carry.
#
# Every moveit2 package this workspace ports is BSD-3-Clause, so for a long
# time `license.workspace = true` was right everywhere and nobody had to
# think about it. `cspace-stomp-core` broke that: it ports
# ros-industrial/stomp, which is Apache-2.0 (`LICENSE`, `package.xml:9`, and
# a per-file header on every source). Inheriting the workspace license there
# would relabel Apache-2.0-derived code as BSD-3-Clause -- and nothing about
# writing `license.workspace = true` would look wrong while doing it, which
# is exactly why this is a gate and not a comment in the root manifest.
#
# The rule is derived from the tree, not from a table this script would have
# to be taught about each new upstream:
#
#   1. every tracked `.rs` file under a crate carries an
#      `SPDX-License-Identifier:`,
#   2. all of a crate's files agree on one identifier, and
#   3. the crate's effective manifest license equals that identifier
#      (explicit `license = "..."`, else the inherited
#      `[workspace.package] license`).
#
# A new crate whose sources are attributed to a differently-licensed upstream
# therefore fails here until its manifest says so, whichever way the mistake
# was made: right header + inherited manifest, or explicit manifest + copied
# header.
#
# Run by CI via the `tools/ci/check-*.sh` glob. Needs no docker.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
require_caller_tree "$repo_root"
cd "$repo_root"

workspace_license="$(sed -n 's/^license[[:space:]]*=[[:space:]]*"\([^"]*\)".*/\1/p' Cargo.toml | head -1)"
if [ -z "$workspace_license" ]; then
  echo "Cargo.toml: no [workspace.package] license to inherit from" >&2
  exit 1
fi

status=0

mapfile -t manifests < <(git ls-files --deduplicate -- 'crates/*/Cargo.toml' 'tools/*/Cargo.toml' | sort)
require_nonempty "${#manifests[@]}" "crate manifest under crates/ or tools/"

for manifest in "${manifests[@]}"; do
  crate_dir="$(dirname "$manifest")"

  # Effective license: an explicit value wins, otherwise the crate must say it
  # is inheriting. Saying neither is itself an error -- cargo would publish it
  # with no license at all.
  declared="$(sed -n 's/^license[[:space:]]*=[[:space:]]*"\([^"]*\)".*/\1/p' "$manifest" | head -1)"
  if [ -z "$declared" ]; then
    if grep -Eq '^license\.workspace[[:space:]]*=[[:space:]]*true' "$manifest"; then
      declared="$workspace_license"
    else
      echo "$manifest: no license and no license.workspace = true" >&2
      status=1
      continue
    fi
  fi

  # Checked substitution, not `mapfile -t sources < <(git ls-files ... | sort)`:
  # process substitution discards the producer's exit status, so a broken
  # `git ls-files` (the same `.git`-less-export scenario that broke
  # check-audit-scripts-not-copied.sh) would be indistinguishable from "this
  # crate genuinely has no .rs files" -- both silently `continue` past the
  # crate with no diagnostic. The genuinely-empty case is still a silent
  # `continue`, unchanged: a manifest-only crate legitimately has nothing to
  # check here.
  if ! sources_raw="$(git ls-files --deduplicate -- "$crate_dir/*.rs" | sort)"; then
    echo "$manifest: git ls-files failed for $crate_dir -- nothing was checked" >&2
    status=1
    continue
  fi
  if [ -z "$sources_raw" ]; then
    continue
  fi
  mapfile -t sources <<<"$sources_raw"

  ids=()
  missing=0
  for f in "${sources[@]}"; do
    id="$(sed -n 's/.*SPDX-License-Identifier:[[:space:]]*\([^[:space:]]*\).*/\1/p' "$f" | head -1)"
    if [ -z "$id" ]; then
      echo "$f: no SPDX-License-Identifier" >&2
      status=1
      missing=1
      continue
    fi
    ids+=("$id")
  done

  # A crate with an unattributed file has no single answer to compare against;
  # reporting a second, derived complaint about its manifest would just point
  # at the wrong file.
  if [ "$missing" -ne 0 ]; then
    continue
  fi

  mapfile -t distinct < <(printf '%s\n' "${ids[@]}" | sort -u)
  if [ "${#distinct[@]}" -gt 1 ]; then
    echo "$crate_dir: sources carry more than one SPDX identifier: ${distinct[*]}" >&2
    echo "  A crate counts against exactly one upstream; split it or fix the headers." >&2
    status=1
    continue
  fi

  if [ "${distinct[0]}" != "$declared" ]; then
    echo "$manifest: declares $declared but its sources are ${distinct[0]}" >&2
    echo "  Set 'license = \"${distinct[0]}\"' explicitly, or fix the source headers" >&2
    echo "  -- whichever matches the upstream this crate actually ports." >&2
    status=1
  fi
done

if [ "$status" -ne 0 ]; then
  exit 1
fi

echo "OK: every crate's license matches the SPDX identifier in its own sources"
