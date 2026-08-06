#!/usr/bin/env python3
"""Keeps PORTING-PLAN.md's evidence-retention tables in step with the tree.

`tools/ci/measure-*` does not split in two. One shape writes a TRACKED artifact
(`measure-port-coverage.py` -> `doc/port-coverage.md`) and carries a `--check`
mode, so a reader can re-derive what the plan quotes. Another takes a
caller-named output directory and writes nothing tracked, so its numbers reach
PORTING-PLAN.md and its output reaches a scratch directory that is deleted. A
third writes nothing at all and reads only the tracked tree, so re-running it
IS the evidence.

The first casualty is §269.3: port vs upstream C++ CHOMP/STOMP over 500
problems, four arms, medians to sixteen digits. Every instrument behind it is
committed; none of its output is. §269.3's condition 3 takes each side's median
over ITS OWN solved set and §269.4 records 363 both-solved / 17 port-only /
7 cpp-only, so the two published medians are over different problems -- and the
un-confounded number, the median over the 363, cannot be computed from this
tree at all. Recovering it costs a re-run of all four arms.

So the rule this gate enforces is not "commit the NDJSONs". It is: every
instrument whose output is untracked, and every plan section publishing from
one, is named in a table, with its evidence either pointed at in the tree or
declared missing. A gap that is written down is a gap someone can close; a gap
that is only implicit re-opens every round.

# What is checked

Two tables in PORTING-PLAN.md, each found by its exact header row and each
required to be unique:

  CENSUS  | 계측기 | 산출물 | 부류 |
    The 계측기 column must be EXACTLY the `tools/ci/measure-*` set on disk --
    no missing entry, no extra one. 산출물 is a comma-separated list of repo
    paths, or `없음`; each path must be tracked, must exist, and must appear
    literally inside the instrument's own text. The column is a list rather
    than one path because `measure-upstream-citations.py` regenerates two
    (`doc/citation-classes.txt` and `doc/upstream-citation-classes.txt`), and a
    one-path column would have made the second invisible. 부류 is one of three
    and must agree with whether 산출물 is a path:

      추적 산출물      has tracked counterpart file(s) and a `--check` /
                       `--write-*` / `--emit-*` mode that reconciles against
                       them, so a reader can re-derive what the plan quotes
      트리에서 재실행  writes nothing, but reads only the tracked tree (plus
                       the pinned upstream checkout), so a re-run re-derives
      미보존 산출물    produces per-problem output from a planner sweep that
                       costs hours, and that output lands nowhere in the tree

    Both the middle and the last class are findings rather than conveniences.
    `measure-port-coverage-independent.py` and `measure-requirement-closure.py`
    commit no artifact and hold no output directory, so calling either one of
    the other two shapes would be false. And the last class is NOT "takes an
    `out_dir`": three of its four members do, but
    `measure-phase8-optimizer-properties.sh` takes none -- it opens a
    `mktemp -d` and deletes it on exit, and declares
    `doc/phase8-optimizer-properties.json` at its line 105, which it writes only
    under `MODE=full`. That mode has never run and the file is neither tracked
    nor present, so its 산출물 is `없음`. Naming the class after the argument
    would have let that one instrument out of the census.

  ROWS    | 계측기 | 절 | 증거 | 행 출처 | 비고 |
    One row per (untracked-output instrument, publishing section). 절 must
    resolve to a heading. 증거 is a tracked, existing path or `없음`. 행 출처
    is `자동` (this gate derived the pair: that section's text names that
    instrument) or `수동` (a human recorded it: the section publishes the
    instrument's numbers without naming it). 비고 must be non-empty.

    The `자동` rows must equal the derived set EXACTLY, in both directions.

    비고 exists because 증거 is one cell and coverage is often partial. §300.2
    publishes both a population split that `doc/phase8-condition2-stomp/`
    re-derives and a wall-clock table that it does not -- the committed NDJSON
    carries no `wall_secs` field. A row that answered only "which path" would
    have called that section covered. Its text is free: this gate requires that
    something is said, not what.

# Which direction absence is checkable, and which it is not

CHECKABLE, and not silenceable from the plan: a new `tools/ci/measure-*` on
disk has no census row, and this gate fails until someone rules on it. Nothing
written in PORTING-PLAN.md can turn that off -- the trigger is the file.

CHECKABLE, and this is what makes the rule survive a deletion: deleting the
sentence that names an instrument does not silence its row, it BREAKS it. The
pair stops being derived, its `자동` row becomes an extra one, and the gate
fails naming it. Silencing it takes editing 행 출처 to `수동`, which leaves the
evidence disposition standing in the table where it was.

NOT CHECKABLE, stated plainly because it is the hole: a section that publishes
an untracked run's numbers WITHOUT naming the instrument produces no derived
pair. §269.3 is exactly that shape -- it publishes the four-arm table and names
no script; §269.2 names the instrument one subsection earlier. Such a section
is recorded only by a `수동` row, and nothing in this tree can prove one is
missing: the numbers are digits, and no rule separates a digit measured by a
deleted sweep from any other digit. The `수동` rows are a human's enumeration,
and this gate checks that each one points at a real section and a real
untracked-output instrument -- not that they are all there.

Nor is `산출물` proved to be what the instrument WRITES. The path must be
tracked, must exist, and must be named inside the instrument's own text; that
it is the file the script produces is asserted by the row, not measured here.

Two further bounds, both measured rather than assumed:

  * The derivation corpus is PORTING-PLAN.md alone. No `doc/*.md` publishes a
    table from an untracked-output instrument today (the only files under
    `doc/` naming one are `doc/phase8-condition2-stomp/README.md`, which
    documents the committed evidence itself, its own `run-subset.sh`, and the
    generated `doc/citation-classes-in-repo.txt`).
  * The instrument family is `tools/ci/measure-*`. Nine `tools/ci/verify-*`
    scripts open a `mktemp -d` too, `verify-phase8-benchmark.sh` among them,
    and this gate does not look at them. That is a named hole, not a claim
    they are clean.

Named `check-*` so `ci.yml`'s prefix glob runs it: python3, the tracked files
and `git ls-files`. No docker, no cargo, no upstream checkout.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

INSTRUMENT_DIR = "tools/ci"
INSTRUMENT_PREFIX = "measure-"

CENSUS_HEADER = ["계측기", "산출물", "부류"]
ROWS_HEADER = ["계측기", "절", "증거", "행 출처", "비고"]

CLASS_TRACKED = "추적 산출물"
CLASS_RERUN = "트리에서 재실행"
CLASS_UNRETAINED = "미보존 산출물"
UNTRACKED_CLASSES = (CLASS_RERUN, CLASS_UNRETAINED)
ALL_CLASSES = (CLASS_TRACKED, CLASS_RERUN, CLASS_UNRETAINED)
NONE_TOKEN = "없음"
ORIGIN_DERIVED = "자동"
ORIGIN_MANUAL = "수동"

HEADING_RE = re.compile(r"^(#{1,6})\s*§?(\d+(?:\.\d+)*)\s")
TOP_HEADING_RE = re.compile(r"^##\s")
SEPARATOR_RE = re.compile(r"^\s*\|[\s:|-]*\|\s*$")
CODE_RE = re.compile(r"`([^`]+)`")


class Fail(Exception):
    """Every give-up path in this file raises this.

    There is no branch that returns a verdict it did not reach: a checker that
    skips what it cannot parse reports OK on a document it only half-read, and
    this repository has shipped that failure more than once.
    """


def cells(line):
    body = line.strip()
    if not body.startswith("|") or not body.endswith("|"):
        raise Fail(f"not a table row: {line!r}")
    return [c.strip() for c in body[1:-1].split("|")]


def find_table(lines, header, label):
    """The unique table whose header row is exactly `header`.

    Zero matches or more than one is a failure, not a skip: this gate's whole
    subject is the table, so "could not find it" and "it is empty" must not
    spell the same as "it passed".
    """
    starts = []
    for i, line in enumerate(lines):
        if not line.strip().startswith("|"):
            continue
        try:
            got = cells(line)
        except Fail:
            continue
        if got == header:
            starts.append(i)
    if len(starts) != 1:
        raise Fail(
            f"expected exactly 1 {label} table with header "
            f"| {' | '.join(header)} | in the document, found {len(starts)}"
        )
    start = starts[0]
    if start + 1 >= len(lines) or not SEPARATOR_RE.match(lines[start + 1]):
        raise Fail(
            f"the {label} table's header row at line {start + 1} is not "
            f"followed by a separator row"
        )
    body = []
    i = start + 2
    while i < len(lines) and lines[i].strip().startswith("|"):
        row = cells(lines[i])
        if len(row) != len(header):
            raise Fail(
                f"{label} table line {i + 1}: {len(row)} cells, expected {len(header)}"
            )
        body.append((i + 1, row))
        i += 1
    if not body:
        raise Fail(f"the {label} table has a header and no rows")
    return start + 1, i, body


def only_code(cell, label, where):
    """A cell that must be exactly one backticked token and nothing else."""
    found = CODE_RE.findall(cell)
    if len(found) != 1 or CODE_RE.sub("", cell).strip():
        raise Fail(f"{where}: {label} must be exactly one `backticked` token, got {cell!r}")
    return found[0]


def code_list(cell, label, where):
    """A cell that must be one or more backticked tokens, comma-separated.

    A list rather than a single token because one instrument can regenerate
    several tracked files -- `measure-upstream-citations.py` writes two -- and a
    column that held one path would have recorded that instrument as retained
    while saying nothing about its second artifact.
    """
    found = CODE_RE.findall(cell)
    if not found or CODE_RE.sub("", cell).strip(" ,\t"):
        raise Fail(
            f"{where}: {label} must be one or more `backticked` tokens separated "
            f"by commas, got {cell!r}"
        )
    return found


def headings(lines):
    marks = []
    for i, line in enumerate(lines, 1):
        m = HEADING_RE.match(line)
        if m:
            marks.append((i, m.group(2), bool(TOP_HEADING_RE.match(line))))
    if not marks:
        raise Fail("the document has no numbered headings -- nothing to resolve 절 against")
    return marks


def owning_section(marks, line_no):
    owner = None
    for ln, num, _ in marks:
        if ln <= line_no:
            owner = num
        else:
            break
    return owner


def top_span(marks, line_no, total):
    """The `## §N` section containing `line_no`, as 1-based (first, last)."""
    first = None
    for ln, _num, is_top in marks:
        if is_top and ln <= line_no:
            first = ln
        elif is_top and ln > line_no:
            return (first, ln - 1) if first else None
    return (first, total) if first else None


def tracked_paths(root):
    out = subprocess.run(
        ["git", "-C", str(root), "ls-files", "--deduplicate"],
        capture_output=True,
        text=True,
    )
    if out.returncode != 0:
        raise Fail(f"git ls-files failed in {root} ({out.returncode}): {out.stderr.strip()}")
    paths = [p for p in out.stdout.split("\n") if p]
    if not paths:
        raise Fail(f"git ls-files listed no files in {root} -- this gate would check nothing")
    return set(paths)


def check_evidence_path(root, path, tracked, label, where):
    """A claimed evidence path must be tracked AND present.

    A directory counts when at least one tracked file lives under it, which is
    what `doc/phase8-condition2-stomp/` is -- the one place in this tree where a
    sweep's raw output was committed.
    """
    if path in tracked:
        if not (root / path).exists():
            raise Fail(f"{where}: {label} `{path}` is tracked but missing from the worktree")
        return
    prefix = path if path.endswith("/") else path + "/"
    under = [p for p in tracked if p.startswith(prefix)]
    if not under:
        raise Fail(
            f"{where}: {label} `{path}` is not a tracked file and no tracked file "
            f"lives under it -- an evidence pointer that resolves to nothing is "
            f"the defect this gate exists for"
        )
    if not (root / path).is_dir():
        raise Fail(
            f"{where}: {label} `{path}` names {len(under)} tracked file(s) but is "
            f"not a directory in the worktree"
        )


def instruments_on_disk(root):
    d = root / INSTRUMENT_DIR
    if not d.is_dir():
        raise Fail(
            f"{INSTRUMENT_DIR}/ is not a directory under {root} -- the instrument "
            f"family cannot be enumerated"
        )
    found = sorted(p.name for p in d.iterdir() if p.is_file() and p.name.startswith(INSTRUMENT_PREFIX))
    if not found:
        raise Fail(
            f"no {INSTRUMENT_DIR}/{INSTRUMENT_PREFIX}* found under {root} -- either "
            f"the naming convention moved or this gate would report OK having "
            f"examined nothing"
        )
    return found


def derive_pairs(lines, marks, untracked_instruments, excluded):
    """(instrument, section) for every mention outside the registry's own section.

    Mentions inside fenced blocks count: `$ tools/ci/measure-chomp-objective.sh`
    in a reproduction recipe publishes from the instrument exactly as a prose
    sentence does, and a rule that skipped fences would be silenced by moving
    the sentence into one.
    """
    pairs = set()
    for i, line in enumerate(lines, 1):
        if excluded and excluded[0] <= i <= excluded[1]:
            continue
        for inst in untracked_instruments:
            if inst in line:
                sec = owning_section(marks, i)
                if sec is None:
                    raise Fail(
                        f"line {i} names {inst} before any numbered heading -- "
                        f"it belongs to no section"
                    )
                pairs.add((inst, sec))
    return pairs


def run(doc, root, want_derived):
    lines = doc.read_text().split("\n")

    marks = headings(lines)
    tracked = tracked_paths(root)
    on_disk = instruments_on_disk(root)

    c_start, _c_end, census = find_table(lines, CENSUS_HEADER, "census")
    r_start, r_end, rows = find_table(lines, ROWS_HEADER, "rows")

    # --- census: the instrument column IS the filesystem set -----------------
    klass = {}
    seen = []
    for ln, (inst_c, art_c, class_c) in census:
        where = f"{doc.name}:{ln}"
        inst = only_code(inst_c, "계측기", where)
        if inst in klass:
            raise Fail(f"{where}: {inst} has a second census row")
        if class_c not in ALL_CLASSES:
            raise Fail(
                f"{where}: 부류 must be one of "
                + ", ".join(f"`{c}`" for c in ALL_CLASSES)
                + f", got {class_c!r}"
            )
        if art_c == NONE_TOKEN:
            if class_c not in UNTRACKED_CLASSES:
                raise Fail(f"{where}: 산출물 {NONE_TOKEN} but 부류 {class_c}")
        else:
            arts = code_list(art_c, "산출물", where)
            if class_c != CLASS_TRACKED:
                raise Fail(
                    f"{where}: 산출물 names {', '.join(f'`{a}`' for a in arts)} "
                    f"but 부류 {class_c}"
                )
            script = root / INSTRUMENT_DIR / inst
            text = script.read_text() if script.is_file() else ""
            for art in arts:
                check_evidence_path(root, art, tracked, "산출물", where)
                if art not in text:
                    raise Fail(
                        f"{where}: `{art}` does not appear anywhere in "
                        f"{INSTRUMENT_DIR}/{inst} -- the pairing is asserted by nobody"
                    )
        klass[inst] = class_c
        seen.append(inst)

    missing = sorted(set(on_disk) - set(seen))
    extra = sorted(set(seen) - set(on_disk))
    if missing:
        raise Fail(
            f"{len(missing)} {INSTRUMENT_DIR}/{INSTRUMENT_PREFIX}* on disk have no "
            f"census row: {', '.join(missing)}"
        )
    if extra:
        raise Fail(
            f"{len(extra)} census row(s) name no file in {INSTRUMENT_DIR}/: "
            f"{', '.join(extra)}"
        )

    untracked = sorted(i for i in seen if klass[i] in UNTRACKED_CLASSES)
    if not untracked:
        raise Fail(
            "the census declares no instrument whose output is untracked -- the "
            "rows table would then have nothing to be about, and this gate would "
            "pass having checked nothing"
        )

    # --- rows ----------------------------------------------------------------
    section_numbers = {num for _ln, num, _t in marks}
    excluded = top_span(marks, c_start, len(lines))
    r_span = top_span(marks, r_start, len(lines))
    if excluded is None or r_span is None:
        raise Fail(
            "a registry table sits outside any `##` section -- its own mentions "
            "cannot be excluded from the derivation"
        )
    if excluded != r_span:
        raise Fail(
            f"the two registry tables are in different `##` sections (lines "
            f"{excluded} and {r_span}); this gate excludes one span from the "
            f"derivation and cannot exclude two"
        )
    if not (excluded[0] <= r_end <= excluded[1]):
        raise Fail("the rows table runs past the end of its own section")

    declared_auto = set()
    manual = 0
    retained = 0
    for ln, (inst_c, sec_c, ev_c, origin_c, note_c) in rows:
        where = f"{doc.name}:{ln}"
        if not note_c:
            raise Fail(
                f"{where}: 비고 is empty -- 증거 is one cell and coverage is "
                f"often partial, so every row must say what its evidence does "
                f"and does not reach"
            )
        inst = only_code(inst_c, "계측기", where)
        if inst not in klass:
            raise Fail(f"{where}: {inst} has no census row")
        if klass[inst] not in UNTRACKED_CLASSES:
            raise Fail(
                f"{where}: {inst} is `{CLASS_TRACKED}`-class; the rows table is "
                f"only about instruments whose output is not tracked"
            )
        # The section sign is optional for the same reason the census gate's
        # sibling spells its fixture headings without it: a synthetic `§N` in a
        # tracked test script is a dangling reference to
        # `check-section-references.sh`, which cannot tell a fixture from a
        # citation. Both spellings resolve against the same heading set.
        m = re.fullmatch(r"§?(\d+(?:\.\d+)*)", sec_c)
        if not m:
            raise Fail(f"{where}: 절 must be `§N` or `§N.M`, got {sec_c!r}")
        sec = m.group(1)
        if sec not in section_numbers:
            raise Fail(f"{where}: §{sec} resolves to no heading in {doc.name}")
        if origin_c == ORIGIN_DERIVED:
            if (inst, sec) in declared_auto:
                raise Fail(f"{where}: ({inst}, §{sec}) has a second {ORIGIN_DERIVED} row")
            declared_auto.add((inst, sec))
        elif origin_c == ORIGIN_MANUAL:
            manual += 1
        else:
            raise Fail(
                f"{where}: 행 출처 must be `{ORIGIN_DERIVED}` or `{ORIGIN_MANUAL}`, "
                f"got {origin_c!r}"
            )
        if ev_c != NONE_TOKEN:
            ev = only_code(ev_c, "증거", where)
            check_evidence_path(root, ev, tracked, "증거", where)
            retained += 1

    derived = derive_pairs(lines, marks, untracked, excluded)
    if want_derived:
        return "\n".join(f"{i}\t§{s}" for i, s in sorted(derived))

    undeclared = sorted(derived - declared_auto)
    stale = sorted(declared_auto - derived)
    if undeclared:
        raise Fail(
            f"{len(undeclared)} section(s) name an instrument with untracked output "
            f"and have no {ORIGIN_DERIVED} row: "
            + ", ".join(f"{i} in §{s}" for i, s in undeclared)
        )
    if stale:
        raise Fail(
            f"{len(stale)} {ORIGIN_DERIVED} row(s) whose section no longer names "
            f"the instrument: "
            + ", ".join(f"{i} in §{s}" for i, s in stale)
            + f" -- if the section still publishes those numbers the row is "
            f"{ORIGIN_MANUAL}, not {ORIGIN_DERIVED}; deleting the citation does not "
            f"delete the obligation"
        )

    by_class = {c: sum(1 for i in seen if klass[i] == c) for c in ALL_CLASSES}
    return (
        f"OK {doc.name}: {len(seen)} {INSTRUMENT_DIR}/{INSTRUMENT_PREFIX}* instruments -- "
        + ", ".join(f"{by_class[c]} `{c}`" for c in ALL_CLASSES)
        + f"; {len(rows)} publishing row(s) over the {len(untracked)} whose output is "
        f"untracked ({len(declared_auto)} {ORIGIN_DERIVED}, exactly the sections naming "
        f"an instrument, and {manual} {ORIGIN_MANUAL}); {retained} row(s) point at "
        f"committed evidence and {len(rows) - retained} declare it {NONE_TOKEN}. "
        f"A section publishing such a run's numbers without naming the instrument is "
        f"invisible here -- see this gate's header."
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--doc", default=None, help="the plan document (default: the tree's PORTING-PLAN.md)")
    ap.add_argument(
        "--root",
        default=None,
        help="repository root to resolve instruments and tracked paths against "
        "(default: this script's own tree; the discriminator points it at a fixture repo)",
    )
    ap.add_argument(
        "--derived",
        action="store_true",
        help="print the derived (instrument, section) pairs instead of checking",
    )
    args = ap.parse_args()

    root = Path(args.root).resolve() if args.root else REPO_ROOT
    doc = Path(args.doc) if args.doc else root / "PORTING-PLAN.md"
    if not doc.is_file():
        print(f"FAIL {doc} does not exist", file=sys.stderr)
        return 1
    try:
        print(run(doc, root, args.derived))
    except Fail as exc:
        print(f"FAIL {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
