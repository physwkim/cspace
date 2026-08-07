#!/usr/bin/env bash
# Runs the ci.yml `rust` job against a fresh clone of HEAD, locally.
#
# `.github/workflows/ci.yml` has executed on a runner twice now (both pushes
# to `origin/main` so far) and failed both times on the `ci checks` step --
# run 31135034802, job 92732383116 -- while every gate in this tree was
# 26/26 green. The cause was a gap this script had: the workspace test run
# passes here, in a working directory that also holds
# `third_party/moveit_resources` (a gitignored external checkout a runner
# does not get) AND has `ripgrep` on PATH (a tool this dev sandbox installed
# as local preference, per `~/.claude/CLAUDE.md`, that `ubuntu-latest` does
# not ship). `check-no-lint-suppression.sh` calling `rg --pcre2` was invisible
# to this script for the same reason the `third_party/` dependency would have
# been: both are "present on every machine that has ever run this gate" and
# absent on the one machine that decides CI's verdict.
#
# So this clones HEAD (the clone has no gitignored files by construction),
# asserts `third_party/` is absent, builds a PATH that denies the same local-
# preference tools the runner lacks, and runs the job under it. It does not
# re-list the steps: it extracts them from ci.yml itself, so the two cannot
# drift. That is the point -- a hand-kept copy of the step list is the same
# convention-only-consistency failure the check-*.sh gates exist to close.
#
# Deliberately NOT named `check-*.sh`: that glob is what ci.yml runs, and a
# script that re-runs ci.yml from inside ci.yml would recurse. This is a
# pre-push / periodic check, run by hand.
#
# Usage: tools/ci/verify-clean-checkout.sh [--keep]
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$repo_root/tools/ci/gate-lib.sh"

require_caller_tree "$repo_root"
cd "$repo_root"

keep=0
[ "${1:-}" = "--keep" ] && keep=1

workflow=".github/workflows/ci.yml"
[ -f "$workflow" ] || { echo "$workflow: not found" >&2; exit 1; }

clone_dir="$(mktemp -d "${TMPDIR:-/tmp}/moveit-rs-clean-checkout.XXXXXX")"
bare_bin="$(mktemp -d "${TMPDIR:-/tmp}/moveit-rs-bare-runner-bin.XXXXXX")"
cleanup() { [ "$keep" -eq 1 ] || rm -rf "$clone_dir" "$bare_bin"; }
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

# Tools this dev sandbox has installed as local preference
# (`~/.claude/CLAUDE.md` "Shell tool preferences") that the GitHub-hosted
# `ubuntu-latest` runner is not known to ship. `check-no-lint-suppression.sh`
# calling `rg --pcre2` is the confirmed case: present here, absent on the
# runner, and that gap is the entire reason both pushes to `origin/main`
# failed while this tree's gates were all green. Extend this list if a
# future check-* gate reaches for another one of them.
denied_tools=(rg fd fd-find sd eza bat tokei delta hyperfine dust duf procs btm hexyl)

# Build a PATH that resolves every command exactly as the real one does --
# same search order, same targets -- except the denied names are never
# linked into it, so `command -v` and a direct exec both report them
# genuinely absent instead of merely broken.
#
# A same-name stub file placed earlier in PATH does not work here: bash's
# `command -v` skips a match that is not executable and keeps searching, so
# a non-executable (or `exit 127`) stub cannot hide a real binary that sits
# later in PATH -- only never linking the name at all can. Verified against
# this host's own `rg` before relying on it.
declare -A linked
IFS=':' read -ra path_dirs <<<"$PATH"
for dir in "${path_dirs[@]}"; do
  [ -d "$dir" ] || continue
  for entry in "$dir"/*; do
    if [ ! -f "$entry" ] || [ ! -x "$entry" ]; then continue; fi
    name="$(basename "$entry")"
    [ -n "${linked[$name]:-}" ] && continue
    linked[$name]=1
    deny=0
    for denied in "${denied_tools[@]}"; do
      if [ "$name" = "$denied" ]; then
        deny=1
        break
      fi
    done
    [ "$deny" -eq 1 ] && continue
    ln -s "$entry" "$bare_bin/$name"
  done
done

for denied in "${denied_tools[@]}"; do
  if command -v "$denied" >/dev/null 2>&1; then
    echo "== denying $denied (present on this host, not carried into the bare-runner PATH)"
  fi
done
if PATH="$bare_bin" command -v rg >/dev/null 2>&1; then
  echo "FAIL rg is still resolvable inside the bare-runner PATH -- the deny list did not take." >&2
  exit 1
fi
echo "== bare-runner PATH built: $(find "$bare_bin" -mindepth 1 | wc -l) commands, ${#denied_tools[@]} denied"

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
  if output="$(PATH="$bare_bin" bash -e -c "$command" 2>&1)"; then
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
echo "OK: every ci.yml step passes on a fresh checkout with no third_party/ and no ${denied_tools[*]}"
