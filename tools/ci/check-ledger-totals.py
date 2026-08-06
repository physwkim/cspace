#!/usr/bin/env python3
"""Check that every declared `**Totals:` paragraph closes against itself.

A Totals declaration is an arithmetic statement, not prose: it says how
many things were examined and then partitions them. Three drifted out of
arithmetic in a single session and no gate read any of them --

  * `ledger-p9-ros.md` §10: "70 sites examined, 68 in-family, 4
    not-this-family". 68 + 4 = 72. An earlier round widened
    `planning.rs:773,784,817,1046` from three sites to four without
    bumping the total, and the round that added six more inherited the
    gap. Corrected to 73 / 69 / 4 at `9e6873a`.
  * `ledger-p9-ros.md` §14's running total, corrected at `0c81359`.
  * `ledger-p9-ros.md` §11: "22 not-this-family (5 clause-2 ..., 1
    clause-2 ..., 1 clause-1 ..., 13 clause-3 ...)". 5 + 1 + 1 + 13 = 20;
    the clause-3 term was two short of the table's own rows.

Each was found by a human adding the line up by hand. This gate does the
addition.

THREE grammars exist in the tree, and each rule below is grounded in one
of them rather than guessed:

  R1  An explicit `Sum: a+b+c+... = N` claim (both `lgpl-provenance-
      audit.md` files write one). Evaluate it.
  R2  A parenthesis of the form `(a+b+c)` right after a number -- the
      same files' per-bucket breakdowns. Sum it.
  R3  A parenthesis whose comma items all begin with an integer, e.g.
      `(5 clause-2 ..., 1 clause-1 ..., 13 clause-3 ...)`. Sum it against
      the number in front. This is the rule that catches §11.
  R4  The top-level partition: some SUFFIX of the numbers outside every
      parenthesis must sum to the first one. A suffix, not all of them,
      because a leading term can qualify a different noun --
      `message-mapping.md` opens "109 fields across 17 types, 75 touched,
      34 dropped", where 75 + 34 = 109 and the 17 counts types, not
      fields. R4 is skipped where R1 applies, since a paragraph that
      states its own sum is already checked there.

What this gate deliberately does NOT do: derive the totals from the table
above them. A row's site count lives in free text ("all 8", "both", "was
3") that no parser can read without becoming a second, weaker author of
the census. This gate checks the statement against itself; enumerating
the table stays a human step.

A paragraph where no rule found anything to check is REPORTED, never
silently passed -- a checker that fails toward silence has measured
nothing. Ledgers carrying no Totals declaration at all are listed with a
count; that is a visible gap, not a failure, since 15 of the 16 summarize
in prose and imposing a grammar on them is their owners' call.

Named `check-` so `.github/workflows/ci.yml`'s glob runs it. Needs
nothing but python3 and the tracked files -- no docker, no cargo, no
upstream checkout.
"""

import re
import subprocess
import sys

MARKER = "**Totals:"
INT = re.compile(r"(?<![\w.:+-])(\d+)(?![\w.])")
SUM_CLAIM = re.compile(r"Sum:\s*([\d\s+]+?)\s*=\s*(\d+)")
PLUS_LIST = re.compile(r"^\s*\d+(?:\s*\+\s*\d+)+\s*$")


def tracked_markdown():
    out = subprocess.run(
        ["git", "ls-files", "--deduplicate", "*.md"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return [p for p in out.split("\n") if p]


def blank_code_spans(text):
    """Blank backticked spans so `planning.rs:609,729` is not read as numbers."""
    out, i, n = [], 0, len(text)
    while i < n:
        if text[i] == "`":
            j = text.find("`", i + 1)
            if j == -1:
                out.append(" " * (n - i))
                break
            out.append(" " * (j - i + 1))
            i = j + 1
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def totals_paragraphs(body):
    """Every paragraph containing the marker, as (first_line_no, text)."""
    found, lines, i, n = [], body.split("\n"), 0, len(body.split("\n"))
    while i < n:
        if MARKER in lines[i]:
            start = i
            while i < n and lines[i].strip():
                i += 1
            found.append((start + 1, "\n".join(lines[start:i])))
        else:
            i += 1
    return found


def split_parens(text):
    """(text with parens blanked, [(index_of_open_paren, inner_text)]) or (None, None)."""
    flat, groups, depth, start = [], [], 0, None
    for i, ch in enumerate(text):
        if ch == "(":
            if depth == 0:
                start = i + 1
            depth += 1
            flat.append(" ")
        elif ch == ")":
            depth -= 1
            if depth == 0 and start is not None:
                groups.append((start - 1, text[start:i]))
                start = None
            if depth < 0:
                return None, None
            flat.append(" ")
        else:
            flat.append(" " if depth else ch)
    if depth != 0:
        return None, None
    return "".join(flat), groups


def comma_breakdown(inner):
    """If every comma item begins with an integer, return those integers."""
    items = [s.strip() for s in inner.split(",") if s.strip()]
    if len(items) < 2:
        return None
    values = []
    for item in items:
        m = re.match(r"^(\d+)\s+\S", item)
        if not m:
            return None
        values.append(int(m.group(1)))
    return values


def check_paragraph(path, lineno, raw):
    """Return (failures, checks_made, prose_parens)."""
    fails, checks, prose = [], 0, 0
    text = blank_code_spans(raw)
    where = f"{path}:{lineno}"

    # R1 -- the paragraph's own `Sum: a+b+c = N` claim.
    sum_claims = list(SUM_CLAIM.finditer(text))
    for m in sum_claims:
        terms = [int(t) for t in re.findall(r"\d+", m.group(1))]
        declared = int(m.group(2))
        checks += 1
        if sum(terms) != declared:
            fails.append(
                f"{where}: `Sum: {m.group(1)} = {declared}` is wrong -- "
                f"the terms add to {sum(terms)}."
            )

    flat, groups = split_parens(text)
    if flat is None:
        fails.append(
            f"{where}: unbalanced parentheses in the `**Totals:` paragraph, so "
            f"nothing in it could be checked."
        )
        return fails, checks, prose

    for idx, inner in groups:
        blanked = blank_code_spans(inner)
        before = [int(m.group(1)) for m in INT.finditer(flat[:idx])]
        values = None
        if PLUS_LIST.match(blanked):  # R2
            values = [int(t) for t in re.findall(r"\d+", blanked)]
        else:  # R3
            values = comma_breakdown(blanked)
        if values is None:
            prose += 1
            continue
        if not before:
            fails.append(
                f"{where}: the breakdown ({', '.join(map(str, values))}) has no "
                f"number in front of it to qualify."
            )
            continue
        checks += 1
        if sum(values) != before[-1]:
            fails.append(
                f"{where}: `{before[-1]}` is broken down as "
                f"{' + '.join(map(str, values))} = {sum(values)}. "
                f"Breakdown: ({' '.join(inner.split())[:120]})"
            )

    # R4 -- the top-level partition, unless R1 already covered the paragraph.
    if not sum_claims:
        top = [int(m.group(1)) for m in INT.finditer(flat)]
        if len(top) < 2:
            fails.append(
                f"{where}: the `**Totals:` paragraph declares fewer than two "
                f"top-level numbers, so it partitions nothing: "
                f"{' '.join(raw.split())[:120]!r}"
            )
        else:
            total, rest = top[0], top[1:]
            if not any(
                sum(rest[k:]) == total for k in range(len(rest))
            ):
                fails.append(
                    f"{where}: `**Totals:` opens with {total}, and no suffix of "
                    f"its remaining top-level numbers {rest} adds to it. "
                    f"{' '.join(raw.split())[:160]!r}"
                )
            else:
                checks += 1

    return fails, checks, prose


def main():
    failures, prose_total, checks_total = [], 0, 0
    declared, ledgers, unchecked = [], [], []
    for path in tracked_markdown():
        try:
            body = open(path, encoding="utf-8").read()
        except (OSError, UnicodeDecodeError) as exc:
            print(f"FAIL {path}: unreadable ({exc})", file=sys.stderr)
            return 1
        if "assertion-discrimination-ledger" in path:
            ledgers.append(path)
        paragraphs = totals_paragraphs(body)
        if paragraphs:
            declared.append(path)
        for lineno, raw in paragraphs:
            fails, checks, prose = check_paragraph(path, lineno, raw)
            failures.extend(fails)
            prose_total += prose
            checks_total += checks
            if checks == 0 and not fails:
                unchecked.append(f"{path}:{lineno}")

    silent = [p for p in ledgers if p not in declared]
    print(
        f"{len(declared)} document(s) declare a `**Totals:` paragraph; "
        f"{checks_total} arithmetic claim(s) checked, "
        f"{prose_total} parenthetical(s) were prose and not summed."
    )
    if unchecked:
        print(
            f"--- {len(unchecked)} `**Totals:` paragraph(s) yielded no checkable "
            f"claim. Nothing verified them: ---"
        )
        for u in unchecked:
            print(f"  {u}")
    if silent:
        print(
            f"--- {len(silent)} ledger(s) declare no `**Totals:` paragraph, so "
            f"nothing here checks their arithmetic. Not a failure: the grammar "
            f"is their owners' call. ---"
        )
        for p in silent:
            print(f"  {p}")

    if failures:
        for f in failures:
            print(f"FAIL {f}", file=sys.stderr)
        print(
            f"FAIL {len(failures)} Totals claim(s) do not close. Fix the "
            f"declaration against the table's own rows -- never the other way "
            f"round.",
            file=sys.stderr,
        )
        return 1

    print("OK every checkable claim in a `**Totals:` paragraph closes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
