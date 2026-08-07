#!/bin/bash
# Enforces that every recorded measurement's `measured_sources` still names the
# tree's own version of each source it was produced by.
#
# The Phase 7 and Phase 8 instruments both write a `measured_sources` map into
# their result JSON -- path to content digest, for the scripts, harnesses,
# planner crates and oracle that produced the numbers. A file's digest is its
# own `git hash-object` blob id; a directory's is a hash over every tracked file
# beneath it, so a planner crate enters the record as one entry that no new
# module can slip past. Both sides compute it through `gate-lib.sh`'s
# `measured_source_digest`, so the value cannot be produced one way and checked
# another. verify-phase7-benchmark.sh:1118 says
# what it is for: the record closes over its inputs by CONTENT rather than by
# revision, so a number cannot be read as current when the thing that computed
# it has moved.
#
# Nothing read it back. Both instruments wrote the map and no gate compared it
# to anything, so the map recorded the drift instead of reporting it. Measured
# when this check was added: doc/phase8-optimizer-properties.json named five
# sources, three of which no longer matched the tree -- and one of those three
# was the harness whose 6.0s wall clock is the reason that file publishes
# `chomp/panda/condition1: FAIL` at all. The FAIL was an artifact of a bound
# the C++ arm did not have, the harness had since been fixed, and the record
# still read as a current fact about the port because nothing said otherwise.
#
# Drift is a hard failure, not a warning and not a grade. A stale record is not
# a smaller version of a current one: every number in it was produced by code
# this tree no longer contains, and the only honest repair is to re-run the
# instrument. Reporting that as a lesser class is how a drifting citation
# survives review.
#
# The set of records is discovered, never enumerated: any tracked `.json` under
# `doc/` carrying a top-level `measured_sources` object is checked. A new
# instrument therefore arrives already covered, and no list here can fall
# behind the filesystem.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

mapfile -t candidates < <(git ls-files --deduplicate -- 'doc/**/*.json' 'doc/*.json')

records=()
for f in "${candidates[@]}"; do
  # `jq -e` distinguishes "no such key" from "the key is null"; a file that is
  # not valid JSON is a failure of its own rather than a silent non-match,
  # because a record that stopped parsing would otherwise leave the set.
  if ! jq -e 'type == "object"' "$f" >/dev/null 2>&1; then
    if ! jq -e . "$f" >/dev/null 2>&1; then
      echo "FAIL $f is tracked under doc/ but is not valid JSON -- it cannot be" >&2
      echo "  checked for measured_sources, so it would silently leave this gate" >&2
      exit 2
    fi
    continue
  fi
  if jq -e 'has("measured_sources")' "$f" >/dev/null 2>&1; then
    records+=("$f")
  fi
done

if [[ ${#records[@]} -eq 0 ]]; then
  echo "FAIL no tracked doc/ JSON carries a measured_sources map -- both the" >&2
  echo "  Phase 7 and Phase 8 instruments write one, so an empty set means this" >&2
  echo "  gate is looking in the wrong place rather than that nothing is recorded" >&2
  exit 2
fi

drift=()
missing=()
checked=0
for record in "${records[@]}"; do
  while IFS=$'\t' read -r path recorded; do
    checked=$((checked + 1))
    if [[ ! -e "$path" ]]; then
      missing+=("$record: $path no longer exists in the tree")
      continue
    fi
    # Through the same helper the producers use, so a file's blob id and a
    # subtree's digest cannot be computed one way when written and another when
    # checked. A helper failure is not drift: it exits rather than reporting a
    # mismatch it did not measure.
    if ! current="$(measured_source_digest "$path")"; then
      echo "FAIL $record: could not digest $path -- see above" >&2
      exit 2
    fi
    if [[ "$current" != "$recorded" ]]; then
      drift+=("$record: $path recorded $recorded, tree has $current")
    fi
  done < <(jq -r '.measured_sources | to_entries[] | "\(.key)\t\(.value)"' "$record")
done

if [[ ${#missing[@]} -gt 0 || ${#drift[@]} -gt 0 ]]; then
  echo "FAIL ${#drift[@]} drifted and ${#missing[@]} missing source(s) across" \
       "${#records[@]} record(s), $checked source(s) checked:" >&2
  printf '  %s\n' "${drift[@]}" "${missing[@]}" >&2
  echo "  every number in an affected record was produced by code this tree no" >&2
  echo "  longer contains; re-run that record's instrument rather than editing" >&2
  echo "  the recorded hash, which would make the record vouch for itself" >&2
  exit 1
fi

echo "OK ${#records[@]} measurement record(s), $checked measured source(s), all current"
