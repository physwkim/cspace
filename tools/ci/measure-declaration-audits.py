#!/usr/bin/env python3
"""Measure which *ported* upstream files carry a declaration-level audit.

`measure-port-coverage.py` answers a file-level question: does some `.rs`
header block name this upstream path?  That is the whole basis on which 158
files are called ported, and `doc/port-coverage.md` 1 says so plainly.  It
says nothing about whether the declarations *inside* those files are
accounted for -- a crate can cite a 1764-line header and port six of its 228
public declarations, and no instrument in this tree notices.

This script does not decide that either: no regex can read prose and tell an
enumeration from a narrative.  What it does is make the *record* checkable.
`doc/declaration-audit-coverage.md` carries one row per ported file with a
verdict a human wrote and a pointer a reader can open; this script asserts
that the row set is exactly the ported set, that every verdict is one of the
two allowed words, and that an `audited` row carries a pointer while a `none`
row does not.  So a file that becomes ported (a new header block) fails here
until someone rules on it, and a verdict cannot be left blank.

The counts are printed on the OK line because the doc quotes them, and
`port-coverage.md` 4's own three-way split -- which nothing checks -- is the
standing example of what happens to a quoted number with no gate under it.

Usage:
    tools/ci/measure-declaration-audits.py [--upstream DIR] [--repo DIR]
                                           [--list] [--check DOC]

Named `measure-*`, not `check-*`: like its sibling it prints a measurement by
default and only asserts under `--check`.  `verify-declaration-audits.sh` is
the thin caller CI's glob reaches.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# Importing a sibling writes `tools/ci/__pycache__/` unless this is set, and
# an untracked directory appearing under a tracked one after every gate run is
# a dirty tree nobody asked for -- silenced here rather than in `.gitignore`,
# so the artifact is never created instead of being created and hidden.
sys.dont_write_bytecode = True

# Imported, never copied: `cited_paths()`'s citation-block grammar is subtle
# (brace expansion, whole-directory citations, indented members) and a second
# copy is how `count-relative-eq.pl` came to return two different answers
# depending on which crate ran it -- see check-audit-scripts-not-copied.sh.
_spec = importlib.util.spec_from_file_location(
    "measure_port_coverage", os.path.join(HERE, "measure-port-coverage.py")
)
mpc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mpc)

VERDICTS = {"audited", "none"}

# `| `<upstream path>` | <verdict> | <citers> | <pointer> |`
#
# Rows are picked out of the file by the `| `moveit_` prefix, so no other
# table in the document may put a backticked upstream path in its first
# column -- 3's per-crate summary would otherwise parse as eight malformed
# rows.  It lists the crate first for that reason, and says so there.
ROW_RE = re.compile(r"^\| `(moveit_[^`]+)` \| ([a-z]+) \| ([^|]*)\| ([^|]*)\|\s*$")

# One pointer grammar, no exceptions: `path:line`.  The alternative was a
# free-text column, and a free-text column is where `doc/port-coverage.md 5`
# sat until this rule made it `doc/port-coverage.md:211` -- a form the reader
# can jump to and this script can resolve.
POINTER_RE = re.compile(r"^`([^`:]+):([0-9]+)`$")

NO_POINTER = ("—", "-")


def ported_with_citers(upstream: str, repo: str) -> dict[str, list[str]]:
    corpus = mpc.corpus_files(upstream)
    cites = mpc.cited_paths(repo, corpus)
    return {f: sorted(set(cites[f])) for f in corpus if f in cites}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--upstream", default=mpc.DEFAULT_UPSTREAM)
    ap.add_argument("--repo", default=os.getcwd())
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--check", metavar="DOC")
    args = ap.parse_args()

    ported = ported_with_citers(args.upstream, args.repo)
    print(f"ported {len(ported)}")

    if args.list:
        for path, citers in ported.items():
            print(path + "\t" + ",".join(citers))

    if not args.check:
        return 0

    with open(args.check, encoding="utf-8") as fh:
        lines = fh.read().split("\n")

    failures = []
    # A list, not a set, for the same reason the sibling gate keeps one: a
    # duplicated row otherwise cancels out of both differences below and the
    # closing line reports the deduplicated count as if it were the file's.
    listed: list[str] = []
    verdicts: dict[str, str] = {}
    for line_no, line in enumerate(lines, 1):
        if not line.startswith("| `moveit_"):
            continue
        match = ROW_RE.match(line)
        if match is None:
            failures.append(f"{args.check}:{line_no}: row does not match the "
                            f"'| `<path>` | <verdict> | <citers> | <pointer> |' "
                            f"grammar: {line[:100]}")
            continue
        path, verdict, _citers, pointer = match.groups()
        listed.append(path)
        verdicts[path] = verdict
        if verdict not in VERDICTS:
            failures.append(f"{args.check}:{line_no}: verdict {verdict!r} is not "
                            f"one of {sorted(VERDICTS)}")
            continue
        pointer = pointer.strip()
        if verdict == "none":
            if pointer not in NO_POINTER:
                failures.append(f"{args.check}:{line_no}: `{path}` is `none` but "
                                f"carries a pointer {pointer!r} -- say `audited` "
                                f"or drop the pointer")
            continue
        # verdict == "audited": the pointer must resolve, because an audit
        # nobody can open is not one.  Line citations go stale on their own --
        # a merge that inserts a paragraph above one moves it silently -- so
        # existence and in-range are checked here.  What is NOT checked, and
        # cannot be by any regex: that the cited line still begins the audit.
        pointer_match = POINTER_RE.match(pointer)
        if pointer_match is None:
            failures.append(f"{args.check}:{line_no}: `{path}` is `audited` but "
                            f"its pointer {pointer!r} is not `path:line`")
            continue
        target, target_line = pointer_match.group(1), int(pointer_match.group(2))
        try:
            with open(os.path.join(args.repo, target), encoding="utf-8") as fh:
                length = sum(1 for _ in fh)
        except OSError:
            failures.append(f"{args.check}:{line_no}: `{path}` cites {target} "
                            f"-- no such file in the tree")
            continue
        if not 1 <= target_line <= length:
            failures.append(f"{args.check}:{line_no}: `{path}` cites "
                            f"{target}:{target_line}, past its {length} lines")

    # Zero rows is the failure this exists to make loud: an empty parse and a
    # correct file otherwise share an exit code.
    if not listed:
        failures.append(f"{args.check}: parsed zero rows -- the table grammar "
                        f"changed and this checked nothing")

    rows = set(listed)
    for path in sorted(set(ported) - rows):
        failures.append(f"MISSING ROW  {path} -- newly ported, no verdict yet")
    for path in sorted(rows - set(ported)):
        failures.append(f"STALE ROW    {path} -- no longer measured as ported")
    for path in sorted({f for f in rows if listed.count(f) > 1}):
        failures.append(f"DUPLICATE ROW  {path} ({listed.count(path)} rows)")

    # The `ported N` this doc transcribes from our own stdout was reconciled
    # by nothing until now -- the row rules above check the table only, which
    # is the same hole that let `cited-outside-corpus` drift 20 -> 24 in
    # `port-coverage.md`.  This doc prints the figure twice; both must agree.
    failures.extend(
        mpc.check_transcribed_figures("\n".join(lines), args.check, {"ported": len(ported)})
    )

    if failures:
        for failure in failures:
            print("FAIL " + failure, file=sys.stderr)
        return 1

    audited = sum(1 for v in verdicts.values() if v == "audited")
    print(
        f"OK {args.check}: {len(listed)} rows == {len(ported)} ported files; "
        f"{audited} audited, {len(listed) - audited} none"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
