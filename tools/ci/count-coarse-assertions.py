#!/usr/bin/env python3
# Usage: tools/ci/count-coarse-assertions.py [PATH ...]
#
# Assertion-discrimination sweep instrument (see
# doc/assertion-discrimination-census.md): mechanically list every
# assertion in the tree whose asserted value is a *coarse-fail signal* --
# a value that says "this failed" or "this is absent" without naming
# which branch of the code under test produced it.
#
# This exists because the sweep's earlier instrument, referred to
# throughout the ledgers as `ledger_scan.py`, was never committed. Its
# number (289) and its grammar (`matches!` / `.is_err()` / `.is_none()`
# only) are quoted in four documents and cannot be re-run or inspected by
# anyone. This script is committed so that every figure it produces can
# be reproduced and disagreed with. It is NOT a re-implementation of that
# scanner and its total is not comparable to 289: the grammar below is
# deliberately wider, and the two were never run over the same scope.
#
# Recognized shapes, one `kind` each:
#
#   matches    assert!(matches!(...))            -- variant match
#   is_err     .is_err()                         -- Result failed
#   is_none    .is_none()                        -- Option absent
#   is_some    .is_some()                        -- Option present
#   is_empty   .is_empty()                       -- collection empty
#   contains_msg     .contains(..) on a rendered error -- which message
#   contains_member  .contains(..) on a collection     -- membership
#   eq_none    assert_eq!(x, None)               -- Option absent
#   eq_err     assert_eq!(x, Err(...))           -- Result failed
#
# `is_some` and a non-empty `.is_empty()` assertion are included because
# the negated forms (`assert!(x.is_some())`, `assert!(!x.is_empty())`)
# are the same coarse signal pointed the other way; census §9 clause 1
# (mechanism) is what decides whether a given hit is in-family, and that
# is a judgment call made in prose, not by this script.
#
# Comments (`//`, `///`, `/* */` including nested) and string-literal
# contents are blanked before matching, preserving line numbering, the
# same way count-narrowing-sweep.sh and count-public-declarations.sh do.
# Raw strings (`r"..."`, `r#"..."#`) are handled.
#
# What this script CANNOT do, stated so no reader mistakes its output for
# a census:
#
#   * It does not know what the assertion is asserting *about*. A
#     `.is_empty()` on a test's own fixture vector and one on the return
#     value of the code under test are the same text to it. Census §9
#     clause 3 (subject) separates them, by reading.
#   * `contains_msg` vs `contains_member` is decided by looking 60 bytes
#     back from `.contains(` for a rendering call (`to_string()`,
#     `unwrap_err()`, `format!`, a `rendered`/`message`/`msg` binding). A
#     helper that renders the error on an earlier line and asserts on a
#     later one reads as `contains_member` here and has to be
#     reclassified by reading -- `assert_err_mentions` is exactly that
#     shape, so its 35 call sites do NOT appear as `contains_msg`.
#   * It counts assertion SITES, not branches. A guard folding N operands
#     into one construction site is one site here and N covered branches
#     in fact -- see doc/folded-operand-guards.md.
#   * A macro whose argument spans an unbalanced-looking string or a
#     nested macro is taken by paren depth, which is correct for
#     well-formed Rust but yields the whole outer call for a hit inside a
#     closure passed to the assertion.
#
# Output: one line per hit, `file:line:kind:scope:text`, where scope is
# `test` for a hit inside a `#[cfg(test)]` module or under a `tests/`
# directory, and `src` otherwise. With no PATH given, walks crates/, ros/
# and tools/, skipping target/ directories.

import re
import sys
from pathlib import Path

MACROS = ("assert", "assert_eq", "assert_ne", "debug_assert", "debug_assert_eq")


def blank_comments_and_strings(text):
    """Replace comment and string-literal contents with spaces, keeping
    every byte position and every newline exactly where it was."""
    out = list(text)
    i, n = 0, len(text)
    depth = 0  # /* */ nesting depth
    while i < n:
        c = text[i]
        if depth:
            if text.startswith("/*", i):
                depth += 1
                out[i] = out[i + 1] = " "
                i += 2
                continue
            if text.startswith("*/", i):
                depth -= 1
                out[i] = out[i + 1] = " "
                i += 2
                continue
            if c != "\n":
                out[i] = " "
            i += 1
            continue
        if text.startswith("/*", i):
            depth = 1
            out[i] = out[i + 1] = " "
            i += 2
            continue
        if text.startswith("//", i):
            while i < n and text[i] != "\n":
                out[i] = " "
                i += 1
            continue
        if c == "r" and i + 1 < n and text[i + 1] in '#"':
            j = i + 1
            hashes = 0
            while j < n and text[j] == "#":
                hashes += 1
                j += 1
            if j < n and text[j] == '"':
                close = '"' + "#" * hashes
                end = text.find(close, j + 1)
                end = n if end == -1 else end + len(close)
                for k in range(i, end):
                    if text[k] != "\n":
                        out[k] = " "
                i = end
                continue
        if c == '"':
            j = i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == '"':
                    j += 1
                    break
                j += 1
            for k in range(i, min(j, n)):
                if text[k] != "\n":
                    out[k] = " "
            i = j
            continue
        if c == "'":
            # char literal or lifetime; only a literal can hide a quote
            if i + 2 < n and (text[i + 1] != "\\" and text[i + 2] == "'"):
                out[i + 1] = " "
                i += 3
                continue
            if i + 1 < n and text[i + 1] == "\\":
                j = text.find("'", i + 2)
                if j != -1:
                    for k in range(i, j + 1):
                        out[k] = " "
                    i = j + 1
                    continue
        i += 1
    return "".join(out)


def test_spans(masked):
    """Byte ranges covered by a `#[cfg(test)]` item, by brace depth."""
    spans = []
    for m in re.finditer(r"#\[cfg\(test\)\]", masked):
        j = masked.find("{", m.end())
        if j == -1:
            continue
        depth, k = 0, j
        while k < len(masked):
            if masked[k] == "{":
                depth += 1
            elif masked[k] == "}":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        spans.append((m.start(), k))
    return spans


def arg_span(masked, open_paren):
    """End index (exclusive) of the macro call opened at `open_paren`."""
    depth, i = 0, open_paren
    while i < len(masked):
        c = masked[i]
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return len(masked)


def classify(body):
    kinds = []
    if re.search(r"\bmatches!\s*\(", body):
        kinds.append("matches")
    for meth in ("is_err", "is_none", "is_some", "is_empty"):
        if re.search(r"\.\s*" + meth + r"\s*\(\s*\)", body):
            kinds.append(meth)
    for m in re.finditer(r"\.\s*contains\s*\(", body):
        recv = body[max(0, m.start() - 60) : m.start()]
        msg = r"(to_string\s*\(\)|unwrap_err\s*\(\)|expect_err\s*\(|\.err\s*\(\)|format!|\brendered\b|\bmessage\b|\bmsg\b)"
        kinds.append("contains_msg" if re.search(msg, recv) else "contains_member")
        break
    if re.search(r",\s*None\s*[,)]?\s*$", body) or re.search(r",\s*None\s*,", body):
        kinds.append("eq_none")
    if re.search(r",\s*Err\s*\(", body):
        kinds.append("eq_err")
    return kinds


def scan(path):
    text = path.read_text(encoding="utf-8", errors="replace")
    masked = blank_comments_and_strings(text)
    spans = test_spans(masked)
    is_tests_dir = "/tests/" in str(path).replace("\\", "/")
    hits = []
    seen = set()
    for m in re.finditer(r"\b(" + "|".join(MACROS) + r")\s*!\s*\(", masked):
        if m.start() in seen:
            continue
        seen.add(m.start())
        open_paren = masked.index("(", m.end() - 1)
        end = arg_span(masked, open_paren)
        body = masked[open_paren : end + 1]
        kinds = classify(body)
        if not kinds:
            continue
        line = masked.count("\n", 0, m.start()) + 1
        scope = (
            "test"
            if is_tests_dir or any(a <= m.start() <= b for a, b in spans)
            else "src"
        )
        shown = " ".join(text[m.start() : end + 1].split())
        if len(shown) > 120:
            shown = shown[:117] + "..."
        hits.append((line, ",".join(kinds), scope, shown))
    return hits


def main(argv):
    roots = [Path(a) for a in argv[1:]] or [Path("crates"), Path("ros"), Path("tools")]
    files = []
    for r in roots:
        if r.is_file():
            files.append(r)
        else:
            files += [p for p in r.rglob("*.rs") if "target" not in p.parts]
    for path in sorted(files):
        for line, kinds, scope, shown in scan(path):
            print(f"{path}:{line}:{kinds}:{scope}:{shown}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
