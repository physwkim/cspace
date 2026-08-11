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
#   2. the crate's licence is the set of terms its files impose -- the union
#      of the `AND`-separated atoms across every header, and
#   3. the crate's effective manifest license names that same set (explicit
#      `license = "..."`, else the inherited `[workspace.package] license`).
#
# A new crate whose sources are attributed to a differently-licensed upstream
# therefore fails here until its manifest says so, whichever way the mistake
# was made: right header + inherited manifest, or explicit manifest + copied
# header.
#
# Comparing sets, rather than requiring one identifier per crate, is what makes
# a mixed crate expressible instead of merely unreported. `cspace-planners`
# ports pilz_industrial_motion_planner, which vendors PAL Robotics'
# Apache-2.0 `joint_limits.hpp`; `pilz/limits.rs` and `pilz/mod.rs` say
# `BSD-3-Clause AND Apache-2.0` and the rest of the crate says `BSD-3-Clause`.
# That is not two crates fighting over one label -- it is one crate under both
# sets of terms, which is what SPDX `AND` means and what crates.io stores. The
# earlier rule could not say so, and the `\([^[:space:]]*\)` it extracted with
# stopped at the first space, so `BSD-3-Clause AND Apache-2.0` read back as
# `BSD-3-Clause` and the gate vouched for a manifest that dropped half its
# obligations. Atoms are compared as an unordered set so no header has to be
# rewritten into a canonical order to pass.
#
# `OR` is rejected outright rather than parsed: a disjunction is a choice a
# person makes and records, and nothing in the tree says which arm was taken.
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
    # The whole expression, not its first token: truncating here is what let a
    # dual-licensed header pass as its permissive half (see the note above).
    id="$(sed -n 's/.*SPDX-License-Identifier:[[:space:]]*\(.*[^[:space:]]\)[[:space:]]*$/\1/p' "$f" | head -1)"
    if [ -z "$id" ]; then
      echo "$f: no SPDX-License-Identifier" >&2
      status=1
      missing=1
      continue
    fi
    if [[ " $id " == *" OR "* ]]; then
      echo "$f: SPDX expression uses OR ($id) -- this gate cannot derive which" >&2
      echo "  arm was taken; record the choice in the header instead." >&2
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

  # `AND` is associative and commutative, so a crate's obligations are the
  # union of its files' atoms and the manifest must name that same set. Sorted
  # only to compare; neither side is required to be written in this order.
  mapfile -t atoms < <(printf '%s\n' "${ids[@]}" | sed 's/[[:space:]]\+AND[[:space:]]\+/\n/g' | sort -u)
  if [[ " $declared " == *" OR "* ]]; then
    echo "$manifest: declares $declared -- this gate cannot derive which arm of" >&2
    echo "  an OR was taken; name the terms that actually apply." >&2
    status=1
    continue
  fi
  mapfile -t declared_atoms < <(printf '%s\n' "$declared" | sed 's/[[:space:]]\+AND[[:space:]]\+/\n/g' | sort -u)

  if [ "${atoms[*]}" != "${declared_atoms[*]}" ]; then
    effective="$(printf '%s AND ' "${atoms[@]}")"
    effective="${effective% AND }"
    echo "$manifest: declares $declared but its sources impose ${atoms[*]}" >&2
    echo "  Set 'license = \"$effective\"' explicitly, or fix the source headers" >&2
    echo "  -- whichever matches the upstreams this crate actually ports." >&2
    status=1
    continue
  fi

  # A crate cargo can publish is a crate whose tarball is a redistribution, and
  # BSD and Apache both require the licence text to travel with it. The SPDX
  # field in the manifest is metadata about the terms, not the terms. Naming
  # the file after the atom is what makes a dual-licensed crate carry both:
  # one atom is `LICENSE`, several are `LICENSE-<atom>`.
  if grep -Eq '^publish[[:space:]]*=[[:space:]]*false' "$manifest"; then
    continue
  fi
  for atom in "${atoms[@]}"; do
    if [ "${#atoms[@]}" -eq 1 ]; then
      want="$crate_dir/LICENSE"
    else
      want="$crate_dir/LICENSE-$atom"
    fi
    if [ ! -f "$want" ]; then
      echo "$crate_dir: publishable and imposes $atom, but $want does not exist" >&2
      echo "  cargo package would ship the terms' name without their text." >&2
      status=1
    fi
  done
done

if [ "$status" -ne 0 ]; then
  exit 1
fi

echo "OK: every crate's license names the same terms its own sources impose"
