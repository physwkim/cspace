#!/usr/bin/env python3
"""A tracked directory that no code file outside it names is committed evidence
nobody can re-derive, and this fails on it before it grows.

The instance that motivated this file was `.phase8-baseline-500-work/`: 3817
tracked files, 81.6% of the repo, 19M, put there by `8479d9aa` "recovered
uncommitted work from a disposed worktree" -- caucus's cleanup queue emptying a
worktree, not a decision to retain anything. It survived because every gate in
`tools/ci/` reads a corpus it names for itself, so a directory nobody named was
in nobody's corpus. Finding it took a hand audit of all 4680 tracked files. The
next one would take another.

The rule is deliberately not "is this directory big" -- it is **is anything
executable in this repository aware that it exists**. A directory of committed
evidence earns its place by having a gate, a script or a source file that reads
it; that reader is what makes the evidence re-derivable rather than merely
present. `doc/phase8-baseline-500/` has `check-phase8-baseline-500.sh`,
`fixtures/` has `verify-fixture-provenance.sh`, `crates/` has the workspace
manifest. `.phase8-baseline-500-work/` had nothing, and its only mention
anywhere in the tree was a handoff note observing that nothing read it.

Prose does not count as a reader, and the `CODE_SUFFIXES` restriction is what
enforces that. Measured on the pre-deletion tree: this check names the drop as
7 rows -- the directory itself plus its 6 subdirectories -- and adding `.md` to
the suffix list drops exactly the first, because `doc/handoff-2026-08-07.md`
names the parent path and nothing names the children. So Markdown would not
have hidden the defect here, but it would have hidden the row that says how big
it is, and it does so on the only reasoning a reader could act on. A `.md` file
naming a path proves someone wrote about it once, not that anything reads it.

The threshold exists so that a stray pair of files is not a CI failure, and is
set far below the smallest real directory (`fixtures/`, 49) and far above the
noise. Every directory prefix is grouped, not just top-level ones, so a bulk
drop under `doc/` is caught the same way as one at the root.

    tools/ci/check-no-orphan-bulk-directory.py
"""
import collections
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# A reader must be something that runs. See the module doc: counting `.md`
# would have let the motivating instance through, because a handoff note
# mentioned it.
CODE_SUFFIXES = (".sh", ".py", ".rs", ".toml", ".yml", ".yaml", ".pl", ".cpp", ".hpp")

# Depth 2 covers every shape this repository actually uses (`crates/<crate>`,
# `doc/<evidence-dir>`, a root-level drop). Going deeper would start flagging
# per-crate `src/` and `tests/` trees, which are named by their manifest rather
# than by path.
MAX_DEPTH = 2

# Below this a directory is a couple of stray files, not a bulk drop. The
# smallest genuinely-read directory in the tree is `fixtures/` at 49.
MIN_FILES = 25


def git(*args):
    out = subprocess.run(
        ["git", "-C", str(REPO_ROOT), *args], capture_output=True, text=True
    )
    if out.returncode not in (0, 1):  # 1 is git grep's "no match"
        print(f"FAIL git {' '.join(args)} failed (rc={out.returncode}): {out.stderr.strip()}",
              file=sys.stderr)
        sys.exit(2)
    return out.stdout.split("\n")


def main():
    files = [f for f in git("ls-files", "--deduplicate") if f]
    if not files:
        print("FAIL git ls-files returned nothing -- this check did not run", file=sys.stderr)
        return 2

    counts = collections.Counter()
    for f in files:
        parts = f.split("/")
        for depth in range(1, min(MAX_DEPTH + 1, len(parts))):
            counts["/".join(parts[:depth])] += 1

    candidates = sorted(d for d, n in counts.items() if n >= MIN_FILES)
    orphans = []
    for d in candidates:
        # `git grep -l -- <path>` over the tree, then drop the directory's own
        # files: a scratch file inside the drop naming its own directory is not
        # a reader of it.
        readers = [
            h for h in git("grep", "-l", "--", d)
            if h and not h.startswith(d + "/") and h.endswith(CODE_SUFFIXES)
        ]
        if not readers:
            orphans.append((counts[d], d))

    if orphans:
        print(
            f"{len(orphans)} tracked director(ies) of {MIN_FILES}+ files that no "
            f"executable file outside them names -- commit a reader (a gate that "
            f"re-derives them) or remove them:",
            file=sys.stderr,
        )
        for n, d in sorted(orphans, reverse=True):
            print(f"  {d}  ({n} tracked files)", file=sys.stderr)
        return 1

    print(
        f"OK every one of the {len(candidates)} tracked director(ies) holding "
        f"{MIN_FILES}+ files is named by at least one {'/'.join(CODE_SUFFIXES)} "
        f"file outside itself, across {len(files)} tracked files"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
