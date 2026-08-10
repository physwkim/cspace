#!/usr/bin/env python3
"""Workspace-wide division/NaN-boundary site sweep, production code only.

Finds every candidate site where a C++-to-Rust port could carry a
division-by-zero or NaN-boundary divergence: `/`, `/=`, `.recip()`,
`.div(`, `.powi(-1)`. It does not judge any site -- that is triage work,
done by hand per finding, not by this script -- it only enumerates
candidates so nothing is missed by search fatigue, and gives a
reproducible total two people (or one person twice) can agree on.

Built and validated against `cspace_planners::pilz` (59 sites, file-by-file
identical across two independent runs) before being generalized to the
whole workspace; see that crate's own findings for what a triage pass
against this script's output looks like.

# Scope

- Roots: `crates/`, `ros/`, `tools/` (override with positional args).
- Only files under a `src/` directory survive -- `tests/`, `examples/`,
  `target/` and anything else are not "production code" in the sense this
  sweep cares about: a bug in a test fixture or a benchmark harness does
  not corrupt a planner's output.
- Within a surviving file, only the code above the first column-0
  `#[cfg(test)]` counts as production; an indented `#[cfg(test)]` (an
  inner helper module, not the file's test boundary) does not end the
  scan. This mirrors the pilz sweep's own convention exactly.

# Why a character-state-machine strips comments/strings, not a per-line filter

A per-line regex filter cannot see a string or block comment that spans
multiple physical lines -- a division-looking character sequence inside
prose or a multi-line literal reads as a hit. The state machine below
walks the raw file character by character (states: code / line_comment /
block_comment / string), blanking non-code content while preserving every
newline so line numbers stay correct.

Every span the state machine consumes in one bulk step (a `//`/`/*`/`*/`
marker, a string's opening/closing quote, a backslash escape, a char
literal) is blanked through one `blank()` helper that maps each character
to itself if it is `\n` and to a space otherwise, rather than assuming the
span is newline-free. That assumption previously held for most of those
spans but not all: Rust's string line-continuation (a backslash
immediately followed by a newline, which the *language* uses to elide the
newline and following indentation from the string's value) was blanked as
two spaces, silently deleting a `\n` byte from the stripped text and
shifting every subsequent line's hits onto the wrong line number for the
rest of the file. `blank()` closes that as a class rather than special-
casing the one escape branch it was found in, since any bulk-consumed span
is one language grammar rule away from being able to contain a newline.

# Usage

    tools/ci/sweep-division-nan-sites.py [root ...]

With no arguments, sweeps `crates ros tools`. Prints one block per crate
with its files and hit lines, a per-crate subtotal, and a grand total.
This is a report, not a gate: it always exits 0, and it does not judge
whether any site is a defect.
"""

import re
import sys
from pathlib import Path

DIV_RE = re.compile(r"[a-z0-9_)\]] */[ =]*[a-z0-9_(]")
DIVEQ_RE = re.compile(r"[a-z0-9_)\]] */=")
RECIP_RE = re.compile(r"\.recip\(\)")
DIV_METHOD_RE = re.compile(r"\.div\(")
POWI_NEG1_RE = re.compile(r"\.powi\(\s*-1\s*\)")


def blank(span):
    """Replace every character in `span` with a space, except a literal
    `\\n`, which passes through unchanged. Every bulk-consumed span below
    (comment markers, quotes, escapes, char literals) goes through this --
    never a bare `" " * len(span)` -- so a span that happens to contain a
    newline (Rust's string line-continuation `\\<newline>` is the case that
    exists in this workspace) cannot silently delete it and shift every
    later line number in the file."""
    return "".join(c if c == "\n" else " " for c in span)


def strip_strings_and_comments(text):
    """Character state machine: blank out string/char literal contents and
    comment contents (keep newlines so line numbers survive), leave code
    untouched."""
    out = []
    i = 0
    n = len(text)
    state = "code"
    while i < n:
        c = text[i]
        if state == "code":
            if text.startswith("//", i):
                state = "line_comment"
                out.append(blank(text[i:i + 2]))
                i += 2
                continue
            if text.startswith("/*", i):
                state = "block_comment"
                out.append(blank(text[i:i + 2]))
                i += 2
                continue
            if c == '"':
                state = "string"
                out.append(blank(c))
                i += 1
                continue
            if c == "'" and i + 1 < n:
                # crude char-literal skip: 'x' or '\n' -- not a lifetime 'a,
                # which never closes within a couple characters.
                m = re.match(r"'(\\.|[^'\\])'", text[i:i + 4])
                if m:
                    out.append(blank(m.group(0)))
                    i += len(m.group(0))
                    continue
            out.append(c)
            i += 1
        elif state == "line_comment":
            if c == "\n":
                state = "code"
                out.append(c)
            else:
                out.append(" ")
            i += 1
        elif state == "block_comment":
            if text.startswith("*/", i):
                state = "code"
                out.append(blank(text[i:i + 2]))
                i += 2
                continue
            out.append(c if c == "\n" else " ")
            i += 1
        elif state == "string":
            if c == "\\" and i + 1 < n:
                out.append(blank(text[i:i + 2]))
                i += 2
                continue
            if c == '"':
                state = "code"
                out.append(blank(c))
                i += 1
                continue
            out.append(c if c == "\n" else " ")
            i += 1
    return "".join(out)


def is_test_boundary(line):
    return line.startswith("#[cfg(test)]")


def sweep_file(path):
    raw = path.read_text()
    lines_raw = raw.split("\n")
    stripped = strip_strings_and_comments(raw)
    lines_stripped = stripped.split("\n")

    test_start = None
    for idx, line in enumerate(lines_raw):
        if is_test_boundary(line):
            test_start = idx
            break

    hits = []
    for idx, line in enumerate(lines_stripped):
        if test_start is not None and idx >= test_start:
            break
        matched = (
            DIV_RE.search(line)
            or RECIP_RE.search(line)
            or DIV_METHOD_RE.search(line)
            or POWI_NEG1_RE.search(line)
            or DIVEQ_RE.search(line)
        )
        if matched:
            hits.append((idx + 1, lines_raw[idx].strip()))
    return hits


def discover_files(root):
    """Every `*.rs` under a `src/` directory beneath `root`, excluding
    `target/` build output."""
    root = Path(root)
    if not root.is_dir():
        return []
    files = []
    for p in root.rglob("*.rs"):
        parts = p.parts
        if "target" in parts:
            continue
        if "src" not in parts:
            continue
        files.append(p)
    return sorted(files)


def crate_name(root, path):
    """The path segment right after `root` -- the crate/package directory
    name, independent of how deep `src/` sits beneath it."""
    rel = path.relative_to(root)
    return rel.parts[0]


def main():
    roots = sys.argv[1:] or ["crates", "ros", "tools"]
    grand_total = 0
    per_root_totals = {}

    for root in roots:
        files = discover_files(root)
        by_crate = {}
        for f in files:
            by_crate.setdefault(crate_name(root, f), []).append(f)

        root_total = 0
        print(f"### {root}/ ({len(files)} production src files, "
              f"{len(by_crate)} crates)")
        for crate in sorted(by_crate):
            crate_total = 0
            crate_lines = []
            for f in sorted(by_crate[crate]):
                hits = sweep_file(f)
                if hits:
                    crate_lines.append(f"  == {f} : {len(hits)} ==")
                    for ln, code in hits:
                        crate_lines.append(f"    {ln}\t{code}")
                    crate_total += len(hits)
            if crate_total:
                print(f"-- {crate}: {crate_total} --")
                for line in crate_lines:
                    print(line)
            root_total += crate_total
        print(f"### {root}/ total: {root_total}\n")
        per_root_totals[root] = root_total
        grand_total += root_total

    print("=" * 60)
    for root, total in per_root_totals.items():
        print(f"{root}/: {total}")
    print(f"GRAND TOTAL: {grand_total}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
