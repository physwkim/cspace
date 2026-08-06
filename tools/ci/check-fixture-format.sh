#!/bin/bash
# Committed oracle-response fixtures must stay diffable.
#
# These files exist so that a change in the C++ oracle shows up as a reviewable
# diff rather than as a silently-updated expectation. A fixture captured
# straight off the oracle's stdout is one long line, which defeats exactly that
# purpose: any change to it renders as a single 21 KB line. The rule is uniform
# across every crate so there is no per-directory boundary to remember.
#
# Canonical form: 2-space indent, keys sorted, one trailing newline. Key order
# is already sorted on the wire (nlohmann::json orders object keys), so sorting
# here only pins that guarantee rather than reordering anything.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

# Globbed off the filesystem rather than out of `git ls-files`: the check is
# about what is on disk, and asking git makes it fail outright in an export
# with no .git -- which is exactly how the oracle image builds its context.
shopt -s nullglob
files=(crates/*/tests/fixtures/*.json)
shopt -u nullglob
if [[ ${#files[@]} -eq 0 ]]; then
  echo "no fixture JSON found -- did the glob or the layout change?" >&2
  exit 1
fi

bad=()
for f in "${files[@]}"; do
  if ! python3 -c '
import json, pathlib, sys
p = pathlib.Path(sys.argv[1])
raw = p.read_text()
sys.exit(0 if json.dumps(json.loads(raw), indent=2, sort_keys=True) + "\n" == raw else 1)
' "$f"; then
    bad+=("$f")
  fi
done

if [[ ${#bad[@]} -gt 0 ]]; then
  printf 'fixture is not in canonical form: %s\n' "${bad[@]}" >&2
  echo >&2
  echo "rewrite with:" >&2
  echo "  python3 -c 'import json,pathlib,sys; p=pathlib.Path(sys.argv[1]); p.write_text(json.dumps(json.loads(p.read_text()), indent=2, sort_keys=True) + chr(10))' <file>" >&2
  exit 1
fi

echo "OK: ${#files[@]} oracle fixtures are in canonical form"
