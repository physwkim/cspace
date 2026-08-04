#!/bin/bash
# Enforces that the workspace resolves serde_json with `float_roundtrip`.
#
# serde_json's default f64 parser is fast but not correctly rounded. It reads
# `10.049999999999999` as the f64 nearest `10.05` -- one ULP away from what
# `str::parse::<f64>` and the C++ oracle that wrote the literal both mean by
# it. Measured across every committed fixture at the time this check was
# added: 6859 of 84221 float literals, 8.1%, in 29 files across 9 crates came
# back one ULP wrong. With the feature on, 0 do.
#
# That error is invisible under any tolerance, which is exactly why it needs a
# mechanical check rather than a comment: it is fatal under `assert_eq!`, and
# it silently sets the floor that every tolerance bisection in this repo
# measures against -- a bisection that stops at a parsing artifact reports a
# floor that is not the port's own error.
#
# The feature is declared once, on `[workspace.dependencies] serde_json`.
# Cargo unions features across the graph, so a member re-declaring
# `serde_json` with its own feature list (as `moveit-distance-field` does for
# `raw_value`) still inherits this one -- provided it goes through
# `workspace = true`. A member that declares serde_json independently would
# not, and that is the case this check exists to catch.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

# `cargo tree` runs on its own rather than at the head of a pipe: piped, a
# resolution failure would reach `grep` as empty input and read as "feature not
# found", which is the same exit status as a real miss but a very different
# fact. Same reasoning as check-dep-direction.sh's `cargo tree` handling.
tree_status=0
tree="$(cargo tree -e features -i serde_json --workspace)" || tree_status=$?
if [[ $tree_status -ne 0 ]]; then
  echo "FAIL cargo tree -e features -i serde_json exited $tree_status -- nothing was checked" >&2
  exit 2
fi

if ! grep -qF 'serde_json feature "float_roundtrip"' <<<"$tree"; then
  echo "FAIL the workspace resolves serde_json without \"float_roundtrip\"" >&2
  echo "  every fixture f64 whose shortest representation needs 17 significant" >&2
  echo "  digits can now deserialize one ULP away from the value the oracle wrote" >&2
  echo "  fix: [workspace.dependencies] serde_json = { version = \"1\", features = [\"float_roundtrip\"] }" >&2
  exit 1
fi

echo 'OK: serde_json resolves with "float_roundtrip"'
