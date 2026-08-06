#!/usr/bin/env python3
"""Enumerates PORTING-PLAN.md's "this round did not X" bullet lists and keeps
`doc/residual-claims-census.md` in sync with a fresh derivation.

§291 (task #31) hand-swept a sample of these lists once and marked it done.
It did not hold: §258.6 sat unclosed for a fixed defect (task #11) with
nothing rechecking it, because nothing enumerated the population -- §291
itself covered 24/224 claim-units (10.7%), chosen by which sections that
round's brief happened to name.

§301 tried two keyword approaches and both undercounted against ground
truth (§291.5's own body-keyword regex; §301's independent body-keyword
regex) for the same reason a *different* keyword filter run the same day
undercounted (`관통` AND a completion-negation verb, which cannot see a
bullet about robots): matching on the bullet's *subject* vocabulary cannot
work, because that vocabulary is whatever the round measured, unbounded by
construction.

Matching on subject was never required. Every list in the corpus -- heading
form (`### §270.2 이 절이 하지 않은 것`) and the plain-prose form this script
exists because of (`이 절이 하지 않은 것 (첫째는 §298이...):`, found at
PORTING-PLAN.md's §284.3/§284.7, neither one a heading at all) -- opens with
one of a small, closed set of LEAD-IN phrases, and every claim in it is a
top-level bullet immediately under that line. Enumerating lead-ins, not
verbs, is what finds §284.3's "관통 분기는 판정하지 않았다" clause: no verb
list this file has tried contains 판정하지, but every version of the lead-in
phrase (본 스크립트, §291.1, this file's own predecessors) already listed
하지 않은 것 -- 판정하지 is caught as a suffix of the bare 하지 alternative,
not because anyone had to think of it.

What this script does NOT claim: whether an open bullet is still true. That
question needs the tool or source the bullet names run again -- §258.6 and
§250.6 (PORTING-PLAN.md §301) were both closed that way, not by pattern
matching, and every subject-keyword filter tried this round (this file's,
§291.5's, and the one that found gapaudit's five closures) missed at least
one real instance for the same underlying reason. So this script only makes
the population visible and countable -- which sub-heading, which round, open
or `거짓 → 닫힘 (§N)` -- and fails when the committed census has drifted from
a fresh derivation, the same contract `classify-unported.py --check` uses
for the unported-file table.

Named `check-*` so `ci.yml`'s glob runs it: python3 and the tracked file,
nothing else -- no docker, no cargo, no upstream checkout.
"""

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DOC = REPO_ROOT / "PORTING-PLAN.md"
DEFAULT_CENSUS = REPO_ROOT / "doc" / "residual-claims-census.md"

FENCE_RE = re.compile(r"^ {0,3}`{3,}[^`]*$")
INLINE_CODE_RE = re.compile(r"(`+)(.+?)\1")
HEADING_RE = re.compile(r"^#{1,6}\s")
BULLET_RE = re.compile(r"^-\s+\S")
TABLE_ROW_RE = re.compile(r"^\s*\|")
SECTION_ID_RE = re.compile(r"^#{1,6}\s*§?(\d+(?:\.\d+)?)")
CLOSURE_RE = re.compile(r"거짓\s*→\s*닫힘\s*\(([^)]+)\)")

# The closed lead-in vocabulary. Every list in the corpus today -- heading or
# plain prose -- ends its lead-in line on one of these, optionally followed
# by a parenthetical annotation and/or a colon (`이 절이 하지 않은 것 (첫째는
# §298이...):`). The bare `하지` alternative deliberately also matches as a
# suffix of a longer verb (판정하지, 증명하지, ...) -- see the module
# docstring for why that is load-bearing, not an accident.
LEADIN_RE = re.compile(
    r"(?:하지|닫지|재지|훑지|증명하지|고치지|묻지|잡지|쓰지|열지|보지|알지)"
    r"\s*(?:않은|못한|않는|못하는)\s*것"
    r"|남은\s*것|남는\s*것|열어\s*두는\s*것|남긴\s*것|못\s*본\s*것"
)


def blank_inline_code(text):
    return INLINE_CODE_RE.sub(lambda m: " " * len(m.group(0)), text)


def is_leadin_line(raw_line):
    """A lead-in is decided on the line with inline code blanked (so a
    citation like `이 절이 하지 않은 것` quoted in backticks elsewhere is not
    read as a second list start), and only matches if the lead-in phrase sits
    at or near the end of the line -- ruling out ordinary prose that merely
    mentions "재지 않은 것" mid-sentence about something else.
    """
    text = raw_line.lstrip("#").strip()
    masked = blank_inline_code(text)
    tail = masked.rstrip()
    # Strip a trailing colon and/or one parenthetical annotation before
    # checking the line's own ending, e.g. "...하지 않은 것 (첫째는 §298이
    # 닫았다):" -> "...하지 않은 것".
    tail = re.sub(r":\s*$", "", tail)
    tail = re.sub(r"\s*\([^()]*\)\s*$", "", tail)
    tail = tail.rstrip()
    m = LEADIN_RE.search(tail)
    return bool(m) and tail.endswith(tail[m.start() : m.end()])


def find_enclosing_section(headings, line_no):
    section_id = None
    for hline, hid in headings:
        if hline > line_no:
            break
        section_id = hid
    return section_id


def parse(doc_path):
    with open(doc_path, encoding="utf-8") as fh:
        raw_lines = fh.read().split("\n")

    in_fence = False
    prose = {}
    headings = []  # (line_no, section_id or None)
    for i, line in enumerate(raw_lines, 1):
        if FENCE_RE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        prose[i] = line
        if HEADING_RE.match(line):
            m = SECTION_ID_RE.match(line)
            headings.append((i, m.group(1) if m else None))

    entries = []  # (leadin_line, section_id, leadin_text, [bullets])
    for i in sorted(prose):
        line = prose[i]
        if not (HEADING_RE.match(line) or line.strip()):
            continue
        if TABLE_ROW_RE.match(line) or BULLET_RE.match(line):
            continue
        if not is_leadin_line(line):
            continue

        j = i + 1
        while j in prose and prose[j].strip() == "":
            j += 1
        bullets = []
        while j in prose and BULLET_RE.match(prose[j]):
            bstart = j
            btext = prose[j]
            j += 1
            # One rule for where an item ends, applied to blank and non-blank
            # alike: the next TOP-LEVEL bullet, a heading, a table row, or any
            # non-blank line at column 0. A blank line is not a boundary --
            # `-` item + blank + INDENTED paragraph is markdown's continuation
            # paragraph, still this item, and reading the blank as the end of
            # the list dropped whatever bullet followed the paragraph. It did:
            # 16387969 gave PORTING-PLAN.md §281.6's `cylinder × box` item a
            # continuation paragraph, and the still-open `관통 분기는 건드리지
            # 않았다` bullet after it left this census silently, total 169 ->
            # 168, with a fresh derivation that agreed with itself.
            while j in prose:
                nxt = prose[j]
                if nxt.strip() == "":
                    k = j
                    while k in prose and prose[k].strip() == "":
                        k += 1
                    # Out of prose (EOF or a fence), a sibling bullet, or an
                    # unindented line: this item is over. The outer loop reads
                    # `prose[j]` again and continues the list iff it is one.
                    if k not in prose or not prose[k].startswith((" ", "\t")):
                        j = k
                        break
                    j = k
                    continue
                if (
                    BULLET_RE.match(nxt)
                    or HEADING_RE.match(nxt)
                    or TABLE_ROW_RE.match(nxt)
                ):
                    break
                btext += " " + nxt.strip()
                j += 1
            bullets.append((bstart, btext))

        if not bullets:
            continue
        section_id = find_enclosing_section(headings, i)
        entries.append((i, section_id, line.strip(), bullets))

    return entries


def render(entries, doc_label):
    """`doc_label` is the citation prefix every row carries, taken from the
    document actually parsed. It was the literal `PORTING-PLAN.md` until a
    fixture run under `--doc` printed rows citing PORTING-PLAN.md line numbers
    that belong to another file -- and those rows are citations
    `check-citation-drift.py` resolves out of this tracked document, so a wrong
    prefix resolves against the wrong file rather than failing."""
    lines = []
    lines.append("<!-- GENERATED by tools/ci/check-residual-claims-census.py --emit")
    lines.append(
        "     doc/residual-claims-census.md"
    )
    lines.append(
        "     Do not hand-edit: `--check doc/residual-claims-census.md` fails if this"
    )
    lines.append("     drifts from a fresh derivation. -->")
    lines.append("")
    lines.append("# 잔여-주장(\"이 절/회차가 하지 않은 것\") 전수 — 어느 것이 열려 있나")
    lines.append("")
    lines.append(
        "PORTING-PLAN.md §301(및 그 이전 §291)이 만든 문서. 헤딩이든 평문이든, "
        "이 절이/회차가 하지/닫지/재지/... 않은 것 계열의 lead-in 줄 바로 아래 "
        "최상위 불릿을 전부 모은다 — 본문 어휘(무엇을 안 쟀는지)가 아니라 "
        "lead-in 어휘(안 쟀다는 것 자체)로 찾으므로, 이 절이 잰 것을 부르는 "
        "단어가 무엇이든 걸린다. **닫힘 여부는 `거짓 → 닫힘 (§N)`이 그 불릿 "
        "자신의 텍스트 안에 있는지만 본다** — 한 불릿에 여러 절이 섞여 있고 "
        "그중 일부만 닫힌 경우(예: PORTING-PLAN.md §284.3), 그 표식이 있으면 "
        "전체가 CLOSED로 잡힌다. 부분 닫힘은 이 표가 못 보고, 여는 사람이 "
        "본문을 읽어야 한다."
    )
    lines.append("")

    total = sum(len(b) for _, _, _, b in entries)
    closed = sum(
        1 for _, _, _, bullets in entries for _, t in bullets if CLOSURE_RE.search(t)
    )
    lines.append(
        f"lead-in {len(entries)}건, 최상위 불릿 {total}건 "
        f"(CLOSED {closed} / OPEN {total - closed})."
    )
    lines.append("")
    lines.append("| 절 | lead-in (줄) | 불릿 | 상태 |")
    lines.append("|---|---|---|---|")
    for leadin_line, section_id, leadin_text, bullets in entries:
        sect = f"§{section_id}" if section_id else "(no §)"
        leadin_short = leadin_text.strip("# ").strip()
        if len(leadin_short) > 60:
            leadin_short = leadin_short[:57] + "..."
        for bline, btext in bullets:
            m = CLOSURE_RE.search(btext)
            status = f"CLOSED ({m.group(1)})" if m else "OPEN"
            claim = re.sub(r"\s+", " ", btext).strip()
            if len(claim) > 90:
                claim = claim[:87] + "..."
            lines.append(
                f"| {sect} | {doc_label}:{leadin_line} {leadin_short} "
                f"| {doc_label}:{bline} {claim} | {status} |"
            )
    lines.append("")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--doc", default=str(DEFAULT_DOC))
    parser.add_argument("--emit")
    parser.add_argument("--check")
    args = parser.parse_args()

    doc_path = Path(args.doc)
    if not doc_path.is_file():
        print(f"FAIL {doc_path} is missing", file=sys.stderr)
        return 2

    entries = parse(doc_path)
    if not entries:
        print(
            "FAIL parsed zero lead-in lists -- the lead-in vocabulary or bullet "
            "grammar changed shape and this checked nothing",
            file=sys.stderr,
        )
        return 1

    try:
        doc_label = str(doc_path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        doc_label = doc_path.name
    rendered = render(entries, doc_label)

    if args.emit:
        Path(args.emit).write_text(rendered, encoding="utf-8")
        total = sum(len(b) for _, _, _, b in entries)
        print(f"wrote {args.emit}: {len(entries)} lead-ins, {total} bullets")
        return 0

    check_path = Path(args.check) if args.check else DEFAULT_CENSUS
    if not check_path.is_file():
        print(f"FAIL {check_path} is missing -- run --emit first", file=sys.stderr)
        return 2
    committed = check_path.read_text(encoding="utf-8")
    if committed != rendered:
        total = sum(len(b) for _, _, _, b in entries)
        print(
            f"FAIL {check_path} does not match a fresh derivation from "
            f"{doc_path} ({len(entries)} lead-ins, {total} bullets today) -- "
            f"regenerate with tools/ci/check-residual-claims-census.py --emit "
            f"{check_path} and commit the result",
            file=sys.stderr,
        )
        return 1

    total = sum(len(b) for _, _, _, b in entries)
    closed = sum(
        1 for _, _, _, bullets in entries for _, t in bullets if CLOSURE_RE.search(t)
    )
    print(
        f"OK {check_path}: {len(entries)} lead-ins, {total} top-level bullets "
        f"(CLOSED {closed} / OPEN {total - closed}), matches a fresh derivation "
        f"from {doc_path}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
