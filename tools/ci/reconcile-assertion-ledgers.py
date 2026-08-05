#!/usr/bin/env python3
# Usage: tools/ci/reconcile-assertion-ledgers.py [--emit-orphans] [--emit-unresolved]
#                                                 [--write-orphans] [--verify]
#
# Assertion-discrimination sweep instrument (see
# doc/assertion-discrimination-census.md, and the p3-acm ledger's "Round 14
# -- closing audit" section): partitions every site
# `count-coarse-assertions.py` finds into MATCHED (some ledger row accounts
# for it) or ORPHAN (no ledger row does), so that partition is reproducible
# from a clean checkout instead of resting on an uncommitted scratchpad
# script and a hand-written table.
#
# This does NOT re-verify any ledger's discrimination verdicts. It answers
# one narrower question: does every coarse-assertion site the scanner finds
# have a ledger row, or a documented reason for having none?
#
# Three ways a ledger row can account for a scanner site:
#
#   1. Exact match -- the row's first-column `file:line` is exactly a
#      scanner site.
#   2. Unique nearby match -- the row's citation is within
#      NEARBY_WINDOW lines of exactly one scanner site in the same file.
#      Small, silent line drift (a comment added a line above, say) is
#      common enough that treating every one of these as a human question
#      would bury the real gaps. A window match against MULTIPLE
#      candidate sites is deliberately NOT auto-resolved -- see
#      `ambiguous_window` below.
#   3. A vetted equivalence in `assertion-ledger-equivalences.json` --
#      for the two shapes a window can't cover: (a) a ledger that cites a
#      guard's own production line instead of the assert that exercises
#      it (moveit-collision's `tools.rs:68 (x)/(y)/(z)` vs `tools.rs:259,
#      271,283`), confirmed by reading that ledger's own prose explaining
#      the correspondence; (b) line drift larger than the window,
#      confirmed by matching the row's own named test function to its
#      current line in the source, not by nearest-line proximity (nearest-
#      line proximity picked the WRONG site at least once while this
#      instrument's equivalences were being derived -- see the JSON
#      file's own comments). Every entry names the evidence; there is no
#      "trust me" entry.
#
# Anything a ledger cites that resolves under none of the three is left in
# `unresolved_citations` -- reported, never silently dropped and never
# silently guessed at. A best-effort heuristic (`classify_citation`) reads
# the actual line the ledger cites and tags *why* automated matching
# failed (a comment line, a `?`-propagation guard line, a `#[test]`
# attribute or `fn` signature line the citation drifted onto, or no clue
# at all), strictly to help a human router, never to auto-resolve.
#
# `--verify` enforces the sweep's closing invariant -- EVERY scanner site
# has an accounting ledger row, and EVERY ledger citation resolves to a
# site -- and only then diffs the orphan set against the committed
# `doc/assertion-discrimination-orphans.txt`. The invariant is checked
# first on purpose. A snapshot diff alone gates drift, not gaps: a new
# orphan is laundered green by `--write-orphans`-ing it into the expected
# set, which is a one-line change no reviewer would flag. The unresolved
# side is worse -- it is invisible to the orphan set entirely whenever
# another ledger still matches the same site, which is exactly what
# happened when a +9 line shift stranded two panels' `ruckig_filter.rs`
# citations while the orphan gate stayed green at 0/0 throughout.
# `--write-orphans` prints that file's exact intended contents (self-dating
# header plus body) so regenerating it is one redirect, not a hand-typed
# header.
import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCANNER = REPO_ROOT / "tools" / "ci" / "count-coarse-assertions.py"
EQUIVALENCES_FILE = REPO_ROOT / "tools" / "ci" / "assertion-ledger-equivalences.json"
ORPHANS_FILE = REPO_ROOT / "doc" / "assertion-discrimination-orphans.txt"
NEARBY_WINDOW = 5


def discover_ledgers():
    """Every assertion-discrimination ledger, found by glob rather than a
    hardcoded list. A fixed list silently stops covering a panel's citations
    the moment that panel adds a new ledger file -- this is exactly what
    happened when `p1-joints` split trajectory content into its own
    `assertion-discrimination-ledger-moveit-trajectory.md`; a hardcoded list
    would keep parsing four other ledgers correctly while quietly never
    reading the fifth again, with no error to say so."""
    return sorted(
        str(p.relative_to(REPO_ROOT))
        for p in (REPO_ROOT / "doc").glob("assertion-discrimination-ledger-*.md")
    )


# Trailing text is intentionally permitted between the line number(s) and the
# closing `|` (e.g. "tools.rs:68 (x)") -- some ledgers' first column disambiguates
# a fanned-out guard-line citation with an "(x)/(y)/(z)" suffix, and a regex
# that required the pipe to follow immediately would silently drop those rows
# from parsing altogether (worse than reporting them unresolved).
FIRST_COL_RE = re.compile(
    r"^\|\s*`?((?:[\w./-]+/)?[\w.-]+\.rs):(\d+(?:\s*,\s*\d+)*)\s*`?[^|]*\|"
)


def current_commit():
    return subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout.strip()


def run_scanner():
    """Live scanner sites, excluding helper_body. Never reads a cached file --
    this is the whole point of "reproducible from a clean checkout"."""
    out = subprocess.run(
        [sys.executable, str(SCANNER), "crates/", "ros/", "tools/"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    sites = {}
    basenames = {}
    for line in out.splitlines():
        if not line.strip():
            continue
        path, lineno, kind, rest = line.split(":", 3)
        scope = rest.split(":", 1)[0]
        if scope == "helper_body":
            continue
        lineno = int(lineno)
        sites[(path, lineno)] = kind
        basenames.setdefault((path.rsplit("/", 1)[-1], lineno), []).append(path)
    return sites, basenames


def parse_ledger_citations(ledger_rel):
    """(cited_file, cited_line, raw_row) for every genuine table-row site
    citation in one ledger. Deliberately does NOT expand hyphen ranges
    (`state.rs:369-513`): every ledger sampled cites those as prose
    references to a whole code region, not a per-site table row -- expanding
    them manufactured ~145 phantom citations from one row the first time
    this was tried by hand. Only comma-separated first-column citations are
    genuine multi-site rows."""
    citations = []
    text = (REPO_ROOT / ledger_rel).read_text(encoding="utf-8")
    for row in text.splitlines():
        m = FIRST_COL_RE.match(row)
        if not m:
            continue
        fname_part, nums_part = m.group(1), m.group(2)
        for n in nums_part.split(","):
            citations.append((fname_part, int(n.strip()), row.strip()[:160]))
    return citations


def path_matches(full_path, fname_part):
    """True if `fname_part`'s path components appear, in order, as a
    subsequence of `full_path`'s components, ending at the same basename.
    A plain `endswith` misses the shorthand some ledgers use --
    `moveit-geometry/bodies.rs` for `crates/moveit-geometry/src/bodies.rs`,
    omitting the `src/`/`tests/` segment -- and that citation was landing
    in `classify_citation` as "cited file not found under this root" even
    though the site is real and already matched by a different ledger's
    (full-path) citation of the same line. Subsequence matching, not a
    second `endswith` variant, because the omitted segment can be `src`,
    `tests`, or (rarer) a nested module dir -- the rule is "these
    components occur in this order ending here", not "this one specific
    segment is optional"."""
    want = fname_part.split("/")
    have = full_path.split("/")
    if not have or have[-1] != want[-1]:
        return False
    it = iter(have[:-1])
    return all(part in it for part in want[:-1])


def resolve(fname_part, lineno, sites, basenames):
    """Exact match, else a UNIQUE match within +/- NEARBY_WINDOW lines in the
    same file. Returns (resolved_site_or_None, status) where status is one
    of "exact", "window", "ambiguous-exact", "ambiguous-window", "none"."""
    if "/" in fname_part:
        exact = [p for (p, ln) in sites if ln == lineno and path_matches(p, fname_part)]
    else:
        exact = basenames.get((fname_part, lineno), [])
    exact = sorted(set(exact))
    if len(exact) == 1:
        return (exact[0], lineno), "exact"
    if len(exact) > 1:
        return None, "ambiguous-exact"

    candidates = set()
    for dl in range(1, NEARBY_WINDOW + 1):
        for cand_line in (lineno - dl, lineno + dl):
            if "/" in fname_part:
                hits = [p for (p, ln) in sites if ln == cand_line and path_matches(p, fname_part)]
            else:
                hits = basenames.get((fname_part, cand_line), [])
            for h in hits:
                candidates.add((h, cand_line))
    if len(candidates) == 1:
        return next(iter(candidates)), "window"
    if len(candidates) > 1:
        return None, "ambiguous-window"
    return None, "none"


TEST_ATTR_OR_SIG_RE = re.compile(r"^(#\[test\]|#\[.*\]|fn\s+\w+|let\s|\}\s*$|\{\s*$)")


def classify_citation(fname_part, lineno):
    """Best-effort, report-only heuristic for why a citation didn't
    resolve -- read the actual line the ledger points at and say what shape
    it is. Returns (category, detail); category is a stable tag for
    aggregate counting, detail is the free-text explanation. Never used to
    auto-match; only to help a human triage `unresolved_citations` faster."""
    basename = fname_part.rsplit("/", 1)[-1]
    candidates = [
        c for c in REPO_ROOT.glob(f"**/{basename}")
        if "target" not in c.parts
        and path_matches(str(c.relative_to(REPO_ROOT)), fname_part)
    ]
    for c in candidates:
        try:
            lines = c.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        if 0 < lineno <= len(lines):
            src = lines[lineno - 1].strip()
            if src.startswith("//") or src.startswith("///"):
                return "comment-line", "cites a comment line, not an assertion"
            if re.search(r"\)\?\s*;?\s*$", src) or re.search(r"\)\?\s*[,;)]", src):
                return (
                    "guard-propagation-line",
                    "cites a `?`-propagation guard line, not an assertion -- outside the scanner's grammar by design",
                )
            if TEST_ATTR_OR_SIG_RE.match(src):
                return (
                    "test-attribute-or-signature-line",
                    f"cites a `#[test]`/`fn`/brace/`let` line, not an assertion: {src[:80]!r}",
                )
            return "no-scanner-site-nearby", f"no scanner match; source there reads: {src[:80]!r}"
    return "file-not-found", "cited file not found under this root"


def load_equivalences():
    if not EQUIVALENCES_FILE.exists():
        return {}
    data = json.loads(EQUIVALENCES_FILE.read_text(encoding="utf-8"))
    out = {}
    for entry in data["equivalences"]:
        key = (entry["ledger"], entry["cited_file"], entry["cited_line"])
        out[key] = entry
    return out


def reconcile():
    """The whole reconciliation, independent of how the result is reported.
    Returns a dict so every mode (default report, --emit-orphans,
    --emit-unresolved, --write-orphans, --verify) computes the partition
    exactly once, the same way."""
    sites, basenames = run_scanner()
    equivalences = load_equivalences()
    ledgers = discover_ledgers()

    matched_sites = set()
    match_notes = []  # (ledger, cited_file, cited_line, resolved_site, how)
    unresolved = []  # (ledger, cited_file, cited_line, raw_row, status, category, detail)
    non_scope = []  # (ledger, cited_file, cited_line, reason)

    for ledger in ledgers:
        for fname_part, lineno, raw in parse_ledger_citations(ledger):
            resolved, status = resolve(fname_part, lineno, sites, basenames)
            if resolved is not None:
                matched_sites.add(resolved)
                match_notes.append((ledger, fname_part, lineno, resolved, status))
                continue

            eq = equivalences.get((ledger, fname_part, lineno))
            if eq is not None:
                if eq["resolution"] == "non_scope":
                    non_scope.append((ledger, fname_part, lineno, eq["reason"]))
                elif eq["resolution"] == "matches":
                    # A guard-line citation can fan out to more than one
                    # assert site (moveit-collision's tools.rs:68 case: one
                    # guard, three axis-isolating tests) -- accept either a
                    # single [file, line] pair or a list of them.
                    raw_sites = eq["scanner_sites"] if "scanner_sites" in eq else [
                        [eq["scanner_file"], eq["scanner_line"]]
                    ]
                    for sf, sl in raw_sites:
                        resolved = (sf, sl)
                        matched_sites.add(resolved)
                        match_notes.append((ledger, fname_part, lineno, resolved, "equivalence: " + eq["reason"]))
                else:
                    raise ValueError(f"unknown equivalence resolution {eq['resolution']!r}")
                continue

            category, detail = classify_citation(fname_part, lineno)
            unresolved.append((ledger, fname_part, lineno, raw, status, category, detail))

    all_scanner_sites = set(sites)
    orphans = sorted(all_scanner_sites - matched_sites)

    return {
        "sites": sites,
        "ledgers": ledgers,
        "total": len(all_scanner_sites),
        "matched_count": len(matched_sites),
        "orphans": orphans,
        "match_notes": match_notes,
        "unresolved": unresolved,
        "non_scope": non_scope,
    }


def orphan_lines(result):
    return [f"{path}:{line}:{result['sites'][(path, line)]}" for path, line in result["orphans"]]


HEADER_FIELD_RE = re.compile(r"^#\s*([A-Za-z ]+?):\s*(\S+)\s*$")


def write_orphans_header(result, commit):
    return [
        "# Orphan enumeration: coarse-assertion scanner sites with no accounting ledger row.",
        "# Generated by: python3 tools/ci/reconcile-assertion-ledgers.py --write-orphans",
        f"# Source commit: {commit}",
        f"# Scanner sites (excl. helper_body): {result['total']}",
        f"# Matched (some ledger row accounts for the site): {result['matched_count']}",
        f"# Orphans (no ledger row accounts for the site): {len(result['orphans'])}",
        "# This file goes stale the moment the corpus or a ledger changes without",
        "# regenerating it -- run with --verify (wired into tools/ci/verify-all.sh via",
        "# tools/ci/verify-orphan-enumeration.sh) to catch that before it ships.",
        "# Format: file:line:kind (kind per tools/ci/count-coarse-assertions.py's classify())",
    ]


def read_committed_orphans():
    if not ORPHANS_FILE.exists():
        return None, []
    header = {}
    body = []
    for line in ORPHANS_FILE.read_text(encoding="utf-8").splitlines():
        if line.startswith("#"):
            m = HEADER_FIELD_RE.match(line)
            if m:
                header[m.group(1).strip().lower()] = m.group(2)
            continue
        if line.strip():
            body.append(line.strip())
    return header, body


def main(argv):
    emit_orphans = "--emit-orphans" in argv
    emit_unresolved = "--emit-unresolved" in argv
    write_orphans = "--write-orphans" in argv
    verify = "--verify" in argv

    result = reconcile()
    total = result["total"]
    matched = result["matched_count"]
    orphan_count = len(result["orphans"])
    match_notes = result["match_notes"]
    unresolved = result["unresolved"]
    non_scope = result["non_scope"]

    if write_orphans:
        for line in write_orphans_header(result, current_commit()):
            print(line)
        for line in orphan_lines(result):
            print(line)
        return 0

    if verify:
        # The invariant, before the snapshot diff. Regenerating the
        # snapshot can make a fresh orphan look expected, and an
        # unresolved citation never reaches the snapshot at all when some
        # other ledger still matches the site -- neither is catchable by
        # comparing two orphan lists.
        if orphan_count or unresolved:
            print(f"FAIL the sweep's invariant is broken: {orphan_count} orphan site(s), "
                  f"{len(unresolved)} unresolved ledger citation(s) "
                  f"(both must be 0; scanner sites, excl. helper_body: {total})")
            for site, line in result["orphans"]:
                print(f"  orphan   {site}:{line}")
            for ledger, fname_part, lineno, _raw, _status, category, detail in unresolved:
                print(f"  citation {Path(ledger).name} -> {fname_part}:{lineno} :: {category}; {detail}")
            print()
            print("An orphan is an assertion no ledger accounts for; an unresolved citation is a "
                  "ledger row pointing at no assertion. Fix the ledger (or add a vetted entry to "
                  "tools/ci/assertion-ledger-equivalences.json naming the evidence) -- do NOT "
                  "regenerate the orphan snapshot to absorb it.")
            return 1

        header, committed_body = read_committed_orphans()
        if header is None:
            print(f"FAIL {ORPHANS_FILE.relative_to(REPO_ROOT)} does not exist -- run --write-orphans first")
            return 1
        live_body = orphan_lines(result)
        committed_set = set(committed_body)
        live_set = set(live_body)
        added = sorted(live_set - committed_set)
        removed = sorted(committed_set - live_set)

        print(f"scanner sites (excl. helper_body), live: {total}")
        print(f"orphans, live: {orphan_count}  |  orphans, committed file: {len(committed_body)}")
        if not added and not removed:
            print(f"OK {ORPHANS_FILE.relative_to(REPO_ROOT)} matches the live orphan set exactly "
                  f"({orphan_count} sites, commit {current_commit()[:12]})")
            return 0

        print(f"FAIL {ORPHANS_FILE.relative_to(REPO_ROOT)} is stale: "
              f"{len(added)} site(s) added, {len(removed)} site(s) removed since it was generated "
              f"(file's own header says source commit {header.get('source commit', '<missing>')})")
        if added:
            print(f"--- {len(added)} orphan site(s) now present but missing from the committed file ---")
            for line in added:
                print(f"  + {line}")
        if removed:
            print(f"--- {len(removed)} site(s) in the committed file that are no longer orphans ---")
            for line in removed:
                print(f"  - {line}")
        print()
        print("regenerate with: python3 tools/ci/reconcile-assertion-ledgers.py --write-orphans "
              "> doc/assertion-discrimination-orphans.txt")
        return 1

    print(f"ledgers discovered (doc/assertion-discrimination-ledger-*.md): {len(result['ledgers'])}")
    for ledger in result["ledgers"]:
        print(f"  {ledger}")
    print()
    print(f"scanner sites (excl. helper_body): {total}")
    print(f"matched (some ledger row accounts for the site): {matched}")
    print(f"orphans (no ledger row accounts for the site):    {orphan_count}")
    print(f"check: matched + orphans == scanner sites -> {matched + orphan_count == total}")
    print()
    print(f"ledger citations resolved via vetted equivalence: "
          f"{sum(1 for n in match_notes if n[4].startswith('equivalence'))}")
    print(f"ledger citations explained as non-scanner-scope (not gaps): {len(non_scope)}")
    for ledger, f, ln, reason in non_scope:
        print(f"  [{Path(ledger).stem}] {f}:{ln} -- {reason}")

    print(f"ledger citations still unresolved (reported, not guessed): {len(unresolved)}")
    by_category = {}
    for ledger, f, ln, raw, status, category, detail in unresolved:
        by_category.setdefault(category, []).append((ledger, f, ln, detail))
    print("  by cause:")
    for category, items in sorted(by_category.items(), key=lambda kv: -len(kv[1])):
        print(f"    {category}: {len(items)}")
    for ledger, f, ln, raw, status, category, detail in unresolved:
        print(f"  [{Path(ledger).stem}] {f}:{ln} -- {status}; {detail}")

    if emit_orphans:
        print()
        print(f"--- {orphan_count} orphans (file:line:kind) ---")
        for line in orphan_lines(result):
            print(line)

    if emit_unresolved:
        print()
        print(f"--- {len(unresolved)} unresolved ledger citations ---")
        for ledger, f, ln, raw, status, category, detail in unresolved:
            print(f"[{Path(ledger).stem}] {f}:{ln} :: {category}; {detail}")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
