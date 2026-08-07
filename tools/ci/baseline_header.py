"""The generated-baseline header check, shared by every gate that owns one.

A generated baseline carries two things: rows, and a `#` header describing
them. Every gate here already re-derives the rows and fails on a difference.
Nothing re-derived the header, and that gap ships. A merge auto-merges the
sorted rows from both sides -- they are append-only text and git resolves
them without asking -- and then resolves the single `# Citations:` line from
one side. The file lands describing a corpus it does not carry, and no gate
reads that line, so it is green.

Measured on main's own first-parent line: `doc/citation-classes.txt` said
2217 keys while carrying 2218 rows at `df42b3e6`, and 2225 while carrying
2228 at `60c352f4`. Both commits passed every gate in `tools/ci`.

`# Source commit:` is excluded by name. It pins the tree the scan read, and
a baseline whose rows still match today legitimately keeps the older sha
until something regenerates it -- rewriting it on every unrelated commit
would make the one line that records provenance mean nothing.

Comparison is line-for-line against the exact list the writer would emit,
not field-by-field against a parsed grammar. A grammar is a second thing
that can drift out of step with the writer, and its failure mode is
silence: a header it no longer recognises reads back as a header with no
fields, and "no fields differ" is how a checker spells "I checked nothing".
"""

PROVENANCE = "# Source commit:"


def header_lines(text):
    """The leading run of `#` lines -- the header by construction.

    Bounded at the first non-`#` line rather than filtering the whole file,
    so a row that happens to begin with `#` cannot shift the alignment and
    report every line below it as drifted.
    """
    out = []
    for line in text.split("\n"):
        if not line.startswith("#"):
            break
        out.append(line)
    return out


def drifted(on_disk_text, fresh_header):
    """`(n, on_disk, fresh)` for each header line that a fresh run would
    write differently. `None` on either side means the headers are different
    lengths and that side has no line `n`."""
    have, want = header_lines(on_disk_text), list(fresh_header)
    out = []
    for i in range(max(len(have), len(want))):
        a = have[i] if i < len(have) else None
        b = want[i] if i < len(want) else None
        if a == b:
            continue
        if a is not None and b is not None \
                and a.startswith(PROVENANCE) and b.startswith(PROVENANCE):
            continue
        out.append((i + 1, a, b))
    return out


def report(rel, on_disk_text, fresh_header, regen, out):
    """Print the drift and return True to fail the run."""
    rows = drifted(on_disk_text, fresh_header)
    if not rows:
        return False
    print(f"FAIL {rel} has the right rows but a header that describes a "
          f"different tree: {len(rows)} generated header line(s) do not match "
          f"what a fresh run writes", file=out)
    for n, a, b in rows:
        print(f"  header line {n}: file says {a!r}", file=out)
        print(f"  {' ' * len(str(n))}             tree says {b!r}", file=out)
    print(f"  regenerate with: {regen}", file=out)
    return True
