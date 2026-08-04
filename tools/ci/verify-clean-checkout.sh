#!/usr/bin/env bash
# Runs the ci.yml `rust` job against a fresh clone of HEAD, locally.
#
# There is no git remote yet, so `.github/workflows/ci.yml` has never executed
# on a runner. That leaves a specific unknown: the workspace test run passes
# here, in a working directory that also holds `third_party/moveit_resources`
# -- a gitignored external checkout a runner does not get. A test that quietly
# depends on it would pass locally forever and fail on the first real CI run.
#
# So this clones HEAD (the clone has no gitignored files by construction),
# asserts `third_party/` is absent, and runs the job. It does not re-list the
# steps: it extracts them from ci.yml itself, so the two cannot drift. That is
# the point -- a hand-kept copy of the step list is the same
# convention-only-consistency failure the check-*.sh gates exist to close.
#
# Deliberately NOT named `check-*.sh`: that glob is what ci.yml runs, and a
# script that re-runs ci.yml from inside ci.yml would recurse. This is a
# pre-push / periodic check, run by hand.
#
# Usage: tools/ci/verify-clean-checkout.sh [--keep]
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

keep=0
[ "${1:-}" = "--keep" ] && keep=1

workflow=".github/workflows/ci.yml"
[ -f "$workflow" ] || { echo "$workflow: not found" >&2; exit 1; }

clone_dir="$(mktemp -d "${TMPDIR:-/tmp}/moveit-rs-clean-checkout.XXXXXX")"
cleanup() { [ "$keep" -eq 1 ] || rm -rf "$clone_dir"; }
trap cleanup EXIT

echo "== cloning HEAD into $clone_dir"
git clone --quiet --local --no-hardlinks "$repo_root" "$clone_dir"

cd "$clone_dir"
echo "   $(git log --oneline -1)"

if [ -e third_party ]; then
  echo "third_party/ present in a fresh clone -- it is supposed to be gitignored," >&2
  echo "so this script cannot tell a runner-safe test run from a local one." >&2
  exit 1
fi

# Extract `- name:` / `run:` pairs from the workflow, including block scalars.
# Records are emitted as `<name>\034<command>`, one per line, with newlines
# inside the command encoded as \035.
#
# Command substitution, not `mapfile -t steps < <(python3 ...)`: `mapfile`
# does not propagate the producer's exit status and `set -e` does not see it
# either, so the parser's own `sys.exit("no run: steps found in the
# workflow")` printed to stderr and this script then reported "OK: every
# ci.yml step passes" having run zero steps -- reproduced by p1-fixtures on a
# synthetic workflow with no `run:` steps. `check-dep-direction.sh`'s header
# names the same mechanism for `cargo tree`; this is the same fix.
if ! steps_raw="$(
  python3 - "$repo_root/$workflow" <<'PY'
import sys

lines = open(sys.argv[1]).read().split("\n")
name = None
out = []
i = 0
while i < len(lines):
    line = lines[i]
    stripped = line.strip()
    if stripped.startswith("- name:"):
        name = stripped[len("- name:"):].strip()
        i += 1
        continue
    if stripped.startswith("run:") and name:
        rest = stripped[len("run:"):].strip()
        if rest and rest != "|":
            out.append((name, rest))
            name = None
            i += 1
            continue
        # Block scalar: take every following line indented deeper than `run:`.
        run_indent = len(line) - len(line.lstrip())
        i += 1
        body = []
        while i < len(lines):
            nxt = lines[i]
            if nxt.strip() == "":
                body.append("")
                i += 1
                continue
            indent = len(nxt) - len(nxt.lstrip())
            if indent <= run_indent:
                break
            body.append(nxt)
            i += 1
        # Dedent by the smallest indent among non-blank lines.
        widths = [len(b) - len(b.lstrip()) for b in body if b.strip()]
        cut = min(widths) if widths else 0
        out.append((name, "\n".join(b[cut:] if b.strip() else "" for b in body)))
        name = None
        continue
    i += 1

if not out:
    sys.exit("no run: steps found in the workflow")
for n, c in out:
    print(n + "\034" + c.replace("\n", "\035"))
PY
)"; then
  echo "FAIL step extraction from $workflow failed -- nothing was checked." >&2
  exit 1
fi
mapfile -t steps <<<"$steps_raw"

# The parser exits nonzero rather than printing nothing, so this is
# unreachable through it -- it is here so "green" cannot mean "ran zero
# steps" by any route, including a future parser that returns an empty
# success.
if [ "${#steps[@]}" -eq 0 ] || [ -z "${steps[0]}" ]; then
  echo "FAIL no steps extracted from $workflow -- nothing was checked." >&2
  exit 1
fi

echo "== ${#steps[@]} steps extracted from $workflow"
echo

status=0
for record in "${steps[@]}"; do
  step_name="${record%%$'\034'*}"
  command="${record#*$'\034'}"
  command="${command//$'\035'/$'\n'}"

  printf '== %s\n' "$step_name"
  if output="$(bash -e -c "$command" 2>&1)"; then
    printf '   PASS\n'
  else
    printf '   FAIL\n'
    printf '%s\n' "$output" | tail -20 | sed 's/^/   | /'
    status=1
  fi
done

echo
if [ "$status" -ne 0 ]; then
  echo "FAILED: ci.yml would not pass on a fresh checkout"
  exit 1
fi
echo "OK: every ci.yml step passes on a fresh checkout with no third_party/"
