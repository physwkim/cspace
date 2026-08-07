#!/usr/bin/env python3
# Usage: tools/ci/reconcile-assertion-ledgers.py [--emit-orphans] [--emit-unresolved]
#                                                 [--write-orphans] [--write-comparison-baseline]
#                                                 [--verify]
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
#   2. Unique containing match -- the row's citation falls inside exactly
#      one scanner site's own span in the same file. A multi-line assert
#      is one site, and a row is entitled to cite the clause that carries
#      the discrimination rather than the macro's opening line. This
#      replaced a +/- 5 line proximity window, which resolved drift as
#      though it were that convention; resolve() carries the measurement.
#      A citation contained by MULTIPLE candidate spans is deliberately
#      NOT auto-resolved -- see `ambiguous-span` below.
#   3. A vetted equivalence in `assertion-ledger-equivalences.json` --
#      for the two shapes containment can't cover: (a) a ledger that cites
#      a guard's own production line instead of the assert that exercises
#      it (moveit-collision's `tools.rs:68 (x)/(y)/(z)` vs `tools.rs:259,
#      271,283`), confirmed by reading that ledger's own prose explaining
#      the correspondence; (b) drift that lands outside the assertion's
#      span, confirmed by matching the row's own named test function to its
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
import os
import re
import subprocess
import sys
from pathlib import Path

import baseline_header

REPO_ROOT = Path(__file__).resolve().parents[2]
SCANNER = REPO_ROOT / "tools" / "ci" / "count-coarse-assertions.py"
# The corpus, named once. Both the scanner invocation and the citation
# classifier must search exactly these roots: if the classifier can reach a
# file the scanner never scanned, it will describe source the sweep never
# saw. `.caucus/worktrees/` holds a full checkout per caucus panel, so an
# unrestricted `REPO_ROOT.glob("**/name.rs")` matches a dozen copies of
# every crate file and reports whichever it hits first. That happened: a
# citation to `robot_model.rs:2051` was reported as "cites an assertion"
# from a worktree copy while the real file had a `fn` on that line.
SCAN_ROOTS = ("crates", "ros", "tools")
EQUIVALENCES_FILE = REPO_ROOT / "tools" / "ci" / "assertion-ledger-equivalences.json"
ORPHANS_FILE = REPO_ROOT / "doc" / "assertion-discrimination-orphans.txt"

# SECOND POPULATION: sites the scanner only finds because half_plane/
# cmp_compound exist now (PORTING-PLAN.md §307), INCLUDING an assertion-helper
# fn's call sites that only became visible because the helper's OWN internal
# assertion just became classifiable (see `helpers` in count-coarse-
# assertions.py's scan()) -- a call site can carry kind `via:<fn>` with
# neither new kind in that string at all, so membership is computed by
# diffing against a `CCA_LEGACY_KINDS_ONLY=1` re-run (see run_scanner), not by
# filtering on a kind string. Folding these orphans into ORPHANS_FILE's
# 0-must-hold invariant would do exactly what this gate's own header warns
# against -- "a new orphan is laundered green by --write-orphans-ing it into
# the expected set" -- except at the scale of hundreds of sites in one commit
# instead of one. So, same as check-citation-drift.py's IN_REPO_BASELINE/
# IN_REPO_HARD_FAIL split for its own second population: these orphans get a
# SEPARATE baseline file, declared and drift-checked on their own, while
# ORPHANS_FILE and its 0-orphan invariant keep meaning exactly what they meant
# before this round -- every site under the kinds that existed before it
# still has, and must keep having, an accounting ledger row.
COMPARISON_BASELINE = REPO_ROOT / "doc" / "assertion-discrimination-orphans-comparison.txt"
# Flipped to True once the backlog is triaged into ledger rows (or vetted
# equivalences). Until then the population is declared, counted and
# drift-checked -- a site leaving or a NEW site joining the live set without
# the baseline being regenerated still fails, so the backlog cannot grow
# silently -- but its current size does not fail the run by itself.
COMPARISON_HARD_FAIL = False


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


def run_scanner(legacy=False):
    """Live scanner sites, excluding helper_body. Never reads a cached file --
    this is the whole point of "reproducible from a clean checkout".

    `legacy=True` sets CCA_LEGACY_KINDS_ONLY, which makes the scanner
    recompute the corpus as it stood before half_plane/cmp_compound existed
    (count-coarse-assertions.py's own flag, not this script's). Diffing that
    set against the live one is how the two populations below are told
    apart -- see COMPARISON_KINDS' comment for why a kind-string filter
    alone cannot do it.

    Returns `(sites, basenames, spans)`, where spans is
    `{(path, first_line): (first_line, last_line)}` for every site in THIS
    call's population -- returned, not parked in a module global, because
    reconcile() runs this twice and the legacy run is second. A global would
    leave every later resolve() doing containment against the pre-half_plane
    corpus, silently unresolving any citation that lands inside a
    `half_plane` site's span."""
    env = dict(os.environ)
    # Spans, not just opening lines: see resolve().
    env["CCA_EMIT_SPAN"] = "1"
    if legacy:
        env["CCA_LEGACY_KINDS_ONLY"] = "1"
    else:
        env.pop("CCA_LEGACY_KINDS_ONLY", None)
    out = subprocess.run(
        [sys.executable, str(SCANNER), *(f"{r}/" for r in SCAN_ROOTS)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
        env=env,
    ).stdout
    sites = {}
    basenames = {}
    spans = {}
    for line in out.splitlines():
        if not line.strip():
            continue
        path, span, kind, rest = line.split(":", 3)
        scope = rest.split(":", 1)[0]
        if scope == "helper_body":
            continue
        first, _, last = span.partition("-")
        lineno = int(first)
        sites[(path, lineno)] = kind
        spans[(path, lineno)] = (lineno, int(last or first))
        basenames.setdefault((path.rsplit("/", 1)[-1], lineno), []).append(path)
    return sites, basenames, spans


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


def resolve(fname_part, lineno, sites, basenames, spans):
    """Exact match, else the UNIQUE scanner site whose span CONTAINS the cited
    line. Returns (resolved_site_or_None, status) where status is one of
    "exact", "ambiguous-exact", "inside-span", "ambiguous-span", "none".

    Containment, not proximity. A ledger row is allowed to cite any line of
    the assertion it accounts for, not only the macro's opening line --
    several ledgers deliberately cite the discriminating clause two lines in
    (`err.to_string().contains("panda_joint1"),`). That names the site
    exactly. This used to be approximated by accepting a unique site within
    +/- 5 lines, on the reasoning that small silent drift is common and
    treating each one as a human question would bury the real gaps.

    Measured at 478d6ff6, that window resolved 27 citations non-exactly. 16
    are the convention above -- the row cites a clause inside the very
    assertion it accounts for. The other 11 were line drift, reported as
    matched. Containment rejects 10 of them, which is how
    `repoint-ledger-citations.py` got to see them at all: at that commit the
    gate goes from 4 orphans / 1 unresolved to 14 / 11.

    Those 10 had drifted 1-4 lines and proximity happened to land on the
    right assertion anyway. The 11th is both why proximity is the wrong rule
    and where containment stops: `collision_parity.rs:1636` had drifted 142
    lines, so BOTH rules hand it `:1633` -- an assertion whose span is
    1633-1638, inside `pr2_world_object_pair_flip_case_122_both_sides_are_
    real_vertices` -- while the row reads `pr2_self_wheel_same_pair_oracle_
    magnitude_is_implausible`, whose own assertion sat in the orphan list.
    Nearness to a stale line number cannot tell "this row's assertion moved
    three lines" from "a different test's assertion happens to sit three
    lines away", and containment cannot either once the drift lands inside a
    neighbour's span; under either reading the row vouches for an assertion
    nobody measured. This file's header already warned that nearest-line
    proximity picked the wrong site while the equivalences were being derived
    by hand; the automatic path had the same defect and no reviewer.

    Only the row's subject column separates those two, which is the key
    `repoint-ledger-citations.py` relocates by. For a row whose subject
    column is a phrase rather than a test name, no rule here can, and such a
    row is repaired by hand against the source text of the commit that last
    held the assertion."""
    if "/" in fname_part:
        exact = [p for (p, ln) in sites if ln == lineno and path_matches(p, fname_part)]
    else:
        exact = basenames.get((fname_part, lineno), [])
    exact = sorted(set(exact))
    if len(exact) == 1:
        return (exact[0], lineno), "exact"
    if len(exact) > 1:
        return None, "ambiguous-exact"

    containing = {
        (p, first)
        for (p, first), (lo, hi) in spans.items()
        if lo <= lineno <= hi
        and (path_matches(p, fname_part) if "/" in fname_part
             else p.rsplit("/", 1)[-1] == fname_part)
    }
    if len(containing) == 1:
        return next(iter(containing)), "inside-span"
    if len(containing) > 1:
        return None, "ambiguous-span"
    return None, "none"


# ---- the row's own subject, and checking a resolution against it -----------
# A line number says WHERE; only the row's subject column says WHAT. Every
# rule above resolves by position, and position alone cannot tell "this row's
# assertion moved" from "a different test's assertion is now at that number"
# -- see resolve()'s two worked cases. So a resolution is additionally
# checked against the test the row itself names, whenever this file can
# establish what that is. Measured at 478d6ff6: 449 of the 1122 resolved
# citations carry a name this can check, and two disagreed with the site they
# had resolved to -- one citation pointing into a neighbour's assertion, one
# row naming the wrong test for a citation that was correct.
#
# This machinery started in repoint-ledger-citations.py, which needs the same
# notion to relocate a drifted citation by content. It lives here because the
# gate is what has to reject a mis-attribution; the repair tool imports it.

# A whole cell that is one snake_case identifier, optionally backticked.
# Anything else -- `(same test)`, a prose phrase, two names -- yields no
# subject key, which is the intended outcome: there is nothing
# content-grounded to check that row against.
IDENT_CELL_RE = re.compile(r"^`?([a-z_][a-z0-9_]{5,})`?$")
FN_DEF_RE = re.compile(r"^\s*(?:pub\s+)?(?:async\s+)?fn\s+([A-Za-z0-9_]+)")

_fn_cache = {}


def fn_defs(rel_path):
    """[(line, name)] for every `fn` in a source file, in file order."""
    if rel_path not in _fn_cache:
        lines = (REPO_ROOT / rel_path).read_text(encoding="utf-8").splitlines()
        _fn_cache[rel_path] = [
            (i + 1, m.group(1))
            for i, line in enumerate(lines)
            if (m := FN_DEF_RE.match(line))
        ]
    return _fn_cache[rel_path]


def fn_span(rel_path, name):
    """[(first_line, last_line)] for each definition of `name`. The span ends
    at the next `fn` rather than at a matching brace: a nested `fn` inside a
    test body would end the span early, and no ledger's subject function has
    one. Two definitions of the same name (two `mod tests` in one file) make
    the key ambiguous and the row is skipped."""
    defs = fn_defs(rel_path)
    out = []
    for i, (line, n) in enumerate(defs):
        if n == name:
            end = defs[i + 1][0] - 1 if i + 1 < len(defs) else 10**9
            out.append((line, end))
    return out


def enclosing_fn(rel_path, line):
    best = None
    for ln, name in fn_defs(rel_path):
        if ln <= line:
            best = name
        else:
            break
    return best


def row_cells(raw_row):
    return raw_row.split("|")


def column_key(index):
    """The whole of column `index` is one snake_case identifier."""
    def key(raw_row, _cited_line):
        cells = row_cells(raw_row)
        if len(cells) <= index:
            return None
        m = IDENT_CELL_RE.match(cells[index].strip())
        return m.group(1) if m else None
    key.name = f"column {index}"
    return key


def adjacent_key(raw_row, cited_line):
    """``:914 (`allowed_planning_time_boundaries_are_not_observable...`)`` --
    a multi-site row that names each site's own test right after that site's
    line number. Anchored on the cited line, so a row listing several sites
    yields the right name for each."""
    m = re.search(
        r":" + str(cited_line) + r"`?\s*\(`([a-z_][a-z0-9_]{5,})`\)", raw_row
    )
    return m.group(1) if m else None


adjacent_key.name = "`:NNN (`fn`)` adjacency"

CANDIDATE_KEYS = [column_key(i) for i in range(2, 7)] + [adjacent_key]
# 100% agreement on a handful of rows is still thin evidence; below this many
# validating samples in one ledger a key is not adopted at all.
MIN_SAMPLE = 3


def full_rows(ledger):
    """{(fname_part, cited_line): the WHOLE row}. parse_ledger_citations()
    truncates the row it reports to 160 characters, which is right for a
    human-facing report and wrong here: pilz's subject column is column 4 and
    sits past that cut, so keying off the reported text silently sees no
    subject at all on exactly the rows that need one."""
    out = {}
    for line in (REPO_ROOT / ledger).read_text(encoding="utf-8").split("\n"):
        m = FIRST_COL_RE.match(line)
        if not m:
            continue
        for n in m.group(2).split(","):
            out[(m.group(1), int(n.strip()))] = line
    return out


def learn_keys(sites, basenames, spans):
    """{ledger: [key, ...]} -- for each ledger, the candidate keys that named
    the site's true enclosing function every time they named anything at all,
    over that ledger's EXACTLY-resolving citations. Ordered by sample size, so
    the best-evidenced key is tried first.

    Learned per ledger rather than fixed, because the ledgers do not share a
    column layout: p3-acm's subject is column 3, pilz's is column 4, and
    p9-ros names each site's test inline. A key is adopted only at zero
    misses -- a key that is right most of the time would launder exactly the
    mis-attributions this exists to catch. Exact resolutions are the training
    set precisely because they are the ones position already settles."""
    learned = {}
    for ledger in discover_ledgers():
        rows = full_rows(ledger)
        scored = []
        for key in CANDIDATE_KEYS:
            hits = misses = 0
            for fname_part, lineno, _short in parse_ledger_citations(ledger):
                raw = rows.get((fname_part, lineno))
                if raw is None:
                    continue
                resolved, status = resolve(fname_part, lineno, sites, basenames, spans)
                if status != "exact":
                    continue
                path, site_line = resolved
                name = key(raw, lineno)
                if name is None or name not in {n for _, n in fn_defs(path)}:
                    continue
                if name == enclosing_fn(path, site_line):
                    hits += 1
                else:
                    misses += 1
            if misses == 0 and hits >= MIN_SAMPLE:
                scored.append((hits, key))
        learned[ledger] = [k for _, k in sorted(scored, key=lambda x: -x[0])]
    return learned


def subject_mismatch(ledger, raw_row, cited_line, resolved, keys):
    """The reason this row cannot be credited with `resolved`, or None.

    Silent on every row this file cannot check: no validated key for the
    ledger, no identifier in the row, or a name that is not a function in the
    resolved file (a row naming an upstream C++ symbol, say). Only a name
    that IS defined in that file and IS a different function than the site
    sits in is a mismatch -- the row and the citation then disagree about
    which assertion is being accounted for, and one of them is wrong."""
    if raw_row is None:
        return None
    name = next(
        (n for k in keys if (n := k(raw_row, cited_line)) is not None), None
    )
    if name is None:
        return None
    path, site_line = resolved
    if name not in {n for _, n in fn_defs(path)}:
        return None
    enclosing = enclosing_fn(path, site_line)
    if enclosing == name:
        return None
    return (
        f"resolved to {path}:{site_line}, which is inside `{enclosing}`, but "
        f"this row names `{name}`. Position resolved it; the row's own "
        f"subject says it is a different assertion."
    )


TEST_ATTR_OR_SIG_RE = re.compile(r"^(#\[test\]|#\[.*\]|fn\s+\w+|let\s|\}\s*$|\{\s*$)")


def classify_citation(fname_part, lineno):
    """Best-effort, report-only heuristic for why a citation didn't
    resolve -- read the actual line the ledger points at and say what shape
    it is. Returns (category, detail); category is a stable tag for
    aggregate counting, detail is the free-text explanation. Never used to
    auto-match; only to help a human triage `unresolved_citations` faster."""
    basename = fname_part.rsplit("/", 1)[-1]
    candidates = [
        c for root in SCAN_ROOTS for c in (REPO_ROOT / root).glob(f"**/{basename}")
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
    sites, basenames, spans = run_scanner()
    legacy_sites, _, _ = run_scanner(legacy=True)
    comparison_sites = set(sites) - set(legacy_sites)
    equivalences = load_equivalences()
    ledgers = discover_ledgers()
    keys = learn_keys(sites, basenames, spans)

    matched_sites = set()
    match_notes = []  # (ledger, cited_file, cited_line, resolved_site, how)
    unresolved = []  # (ledger, cited_file, cited_line, raw_row, status, category, detail)
    non_scope = []  # (ledger, cited_file, cited_line, reason)

    for ledger in ledgers:
        rows = full_rows(ledger)
        for fname_part, lineno, raw in parse_ledger_citations(ledger):
            resolved, status = resolve(fname_part, lineno, sites, basenames, spans)
            why = None
            if resolved is not None:
                why = subject_mismatch(
                    ledger, rows.get((fname_part, lineno)), lineno, resolved,
                    keys.get(ledger, []),
                )
                if why is None:
                    matched_sites.add(resolved)
                    match_notes.append((ledger, fname_part, lineno, resolved, status))
                    continue
                # Deliberately NOT matched: an equivalence entry can still
                # vouch for it below, naming the evidence, which is the one
                # way a row and its citation are allowed to disagree.
                status = "subject-mismatch"

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

            if why is not None:
                # The row's own subject already says what is wrong; the
                # positional heuristic below would replace that with a guess
                # about the line's shape.
                category, detail = "subject-mismatch", why
            else:
                category, detail = classify_citation(fname_part, lineno)
            unresolved.append((ledger, fname_part, lineno, raw, status, category, detail))

    all_scanner_sites = set(sites)
    orphans = sorted(all_scanner_sites - matched_sites)
    orphans_first = [s for s in orphans if s not in comparison_sites]
    orphans_second = [s for s in orphans if s in comparison_sites]

    return {
        "sites": sites,
        # Both derived from the same scanner run as `sites`, so a caller
        # that wants to re-run resolve() (repoint-ledger-citations.py does)
        # cannot reconstruct a basenames/spans pair that disagrees with the
        # partition below.
        "basenames": basenames,
        "spans": spans,
        "ledgers": ledgers,
        "total": len(all_scanner_sites),
        "matched_count": len(matched_sites),
        "orphans": orphans,
        "orphans_first": orphans_first,
        "orphans_second": orphans_second,
        # First-population total/matched, i.e. this file's numbers with the
        # second population's sites excluded entirely -- ORPHANS_FILE's
        # header describes only what its own 0-orphan invariant covers.
        "total_first": len(all_scanner_sites) - len(comparison_sites),
        "matched_count_first": len(matched_sites - comparison_sites),
        "total_second": len(comparison_sites),
        "matched_count_second": len(matched_sites & comparison_sites),
        "match_notes": match_notes,
        "unresolved": unresolved,
        "non_scope": non_scope,
    }


def orphan_lines(result, key="orphans"):
    return [f"{path}:{line}:{result['sites'][(path, line)]}" for path, line in result[key]]


# A header field is a comment line whose text ends in `<label>: <one token>`.
# The label deliberately allows any character: three of the four fields
# `write_orphans_header` emits carry a parenthesised qualifier ("Scanner sites
# (excl. helper_body)"), and the older `[A-Za-z ]` label class matched none of
# them -- so this parsed exactly one field, `Source commit`, and every count in
# the file was read back as absent. The single-token value is what keeps the
# prose lines out: "Generated by: python3 tools/... --write-orphans" and
# "Format: file:line:kind (kind per ...)" both have whitespace after the colon
# and so match nothing.
HEADER_FIELD_RE = re.compile(r"^#\s*(.+?):\s*(\S+)\s*$")


def write_orphans_header(result, commit):
    return [
        "# Orphan enumeration: coarse-assertion scanner sites with no accounting ledger row.",
        "# First population only -- excludes the half_plane/cmp_compound comparison",
        "# population, which has its own baseline (see doc/assertion-discrimination-",
        "# orphans-comparison.txt and run_scanner(legacy=...) in this script).",
        "# Generated by: python3 tools/ci/reconcile-assertion-ledgers.py --write-orphans",
        f"# Source commit: {commit}",
        f"# Scanner sites (excl. helper_body): {result['total_first']}",
        f"# Matched (some ledger row accounts for the site): {result['matched_count_first']}",
        f"# Orphans (no ledger row accounts for the site): {len(result['orphans_first'])}",
        "# This file goes stale the moment the corpus or a ledger changes without",
        "# regenerating it -- run with --verify (wired into tools/ci/verify-all.sh via",
        "# tools/ci/verify-orphan-enumeration.sh) to catch that before it ships.",
        "# Format: file:line:kind (kind per tools/ci/count-coarse-assertions.py's classify())",
    ]


def write_comparison_header(result, commit):
    return [
        "# Second-population baseline: half_plane/cmp_compound orphans (PORTING-PLAN.md",
        "# §307). Declared and drift-checked like the first population, but its size does",
        "# NOT fail --verify by itself (COMPARISON_HARD_FAIL=False in reconcile-assertion-",
        f"# ledgers.py) -- these {result['total_second']} sites were never in any ledger's",
        "# corpus before this baseline was cut, so they are first-ever results, not",
        "# regressions. A site leaving or a new site joining this set without the",
        "# baseline being regenerated still fails -- the backlog is pinned, not laundered.",
        "# Generated by: python3 tools/ci/reconcile-assertion-ledgers.py --write-comparison-baseline",
        f"# Source commit: {commit}",
        f"# Scanner sites in this population: {result['total_second']}",
        f"# Matched (some ledger row accounts for the site): {result['matched_count_second']}",
        f"# Orphans (backlog, not yet reconciled into a ledger): {len(result['orphans_second'])}",
        "# Format: file:line:kind (kind per tools/ci/count-coarse-assertions.py's classify())",
    ]


def read_committed_orphans(path=ORPHANS_FILE):
    if not path.exists():
        return None, []
    header = {}
    body = []
    for line in path.read_text(encoding="utf-8").splitlines():
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
    write_comparison_baseline = "--write-comparison-baseline" in argv
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
        for line in orphan_lines(result, "orphans_first"):
            print(line)
        return 0

    if write_comparison_baseline:
        lines = write_comparison_header(result, current_commit()) + orphan_lines(result, "orphans_second")
        COMPARISON_BASELINE.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"wrote {COMPARISON_BASELINE.relative_to(REPO_ROOT)}: "
              f"{len(result['orphans_second'])} orphan(s) in the comparison population")
        return 0

    if verify:
        # The invariant, before the snapshot diff. Regenerating the
        # snapshot can make a fresh orphan look expected, and an
        # unresolved citation never reaches the snapshot at all when some
        # other ledger still matches the site -- neither is catchable by
        # comparing two orphan lists. Scoped to the FIRST population only
        # (see COMPARISON_KINDS above) -- the second population's own
        # baseline is checked separately, below.
        orphan_count = len(result["orphans_first"])
        if orphan_count or unresolved:
            print(f"FAIL the sweep's invariant is broken: {orphan_count} orphan site(s), "
                  f"{len(unresolved)} unresolved ledger citation(s) "
                  f"(both must be 0; scanner sites, excl. helper_body: {result['total_first']})")
            for site, line in result["orphans_first"]:
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
        live_body = orphan_lines(result, "orphans_first")
        committed_set = set(committed_body)
        live_set = set(live_body)
        added = sorted(live_set - committed_set)
        removed = sorted(committed_set - live_set)

        # The body is only half the file. Every other line the writer emits is a
        # count, and a count of zero orphans is compatible with any scanner
        # total at all -- so comparing bodies alone let `Scanner sites: 756`
        # survive into a tree with 759, on a run that printed OK. The header is
        # re-derived from the same writer rather than spot-checked field by
        # field, so a field added to `write_orphans_header` later is covered
        # without anyone remembering to extend this.
        #
        # `source commit` is deliberately excluded: the snapshot is generated at
        # one commit and verified at later ones, so it is provenance, not a
        # value the tree determines. Everything else must be.
        live_header = {}
        for line in write_orphans_header(result, current_commit()):
            m = HEADER_FIELD_RE.match(line)
            if m:
                live_header[m.group(1).strip().lower()] = m.group(2)
        counted = {f: v for f, v in live_header.items() if f != "source commit"}
        if not counted:
            # The failure this whole block was added to catch, one level up: a
            # header grammar the parser no longer recognises reads back as a
            # file with no counts in it, and "no counts differ" is how a
            # checker spells "I checked nothing".
            print(f"FAIL parsed no count fields out of {ORPHANS_FILE.relative_to(REPO_ROOT)}'s "
                  "own generated header -- HEADER_FIELD_RE and write_orphans_header have "
                  "drifted apart, so this comparison would pass on any file")
            return 1
        drifted = sorted(
            (field, header.get(field, "<missing>"), value)
            for field, value in counted.items()
            if header.get(field) != value
        )

        total_first = result["total_first"]
        print(f"scanner sites (excl. helper_body), live: {total_first}")
        print(f"orphans, live: {orphan_count}  |  orphans, committed file: {len(committed_body)}")
        first_failed = False
        if not added and not removed and not drifted:
            print(f"OK {ORPHANS_FILE.relative_to(REPO_ROOT)} matches the live orphan set exactly "
                  f"({orphan_count} sites, commit {current_commit()[:12]})")
        elif drifted and not added and not removed:
            first_failed = True
            print(f"FAIL {ORPHANS_FILE.relative_to(REPO_ROOT)} has the right orphan set but a "
                  f"stale header: {len(drifted)} count(s) no longer describe the tree "
                  f"(file's own header says source commit "
                  f"{header.get('source commit', '<missing>')})")
            for field, was, now in drifted:
                print(f"  {field}: file says {was}, tree says {now}")
            print()
            print("regenerate with: python3 tools/ci/reconcile-assertion-ledgers.py "
                  "--write-orphans > doc/assertion-discrimination-orphans.txt")
        else:
            first_failed = True
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

        # Second population: drift-checked the same way, but a nonzero
        # backlog does not fail the run by itself (COMPARISON_HARD_FAIL).
        comparison_header, comparison_committed = read_committed_orphans(COMPARISON_BASELINE)
        if comparison_header is None:
            print(f"FAIL {COMPARISON_BASELINE.relative_to(REPO_ROOT)} does not exist -- run "
                  "--write-comparison-baseline first")
            return 1
        # The first population's header is re-derived above; this one's was
        # read into `comparison_header` and never compared. Same gap, same
        # fix -- see tools/ci/baseline_header.py.
        comparison_header_failed = baseline_header.report(
            str(COMPARISON_BASELINE.relative_to(REPO_ROOT)),
            COMPARISON_BASELINE.read_text(encoding="utf-8"),
            write_comparison_header(result, "-"),
            "python3 tools/ci/reconcile-assertion-ledgers.py --write-comparison-baseline",
            sys.stdout)
        comparison_live = set(orphan_lines(result, "orphans_second"))
        comparison_committed_set = set(comparison_committed)
        comparison_added = sorted(comparison_live - comparison_committed_set)
        comparison_removed = sorted(comparison_committed_set - comparison_live)
        second_failed = bool(comparison_added or comparison_removed
                             or comparison_header_failed)
        stream_note = " (backlog; COMPARISON_HARD_FAIL=False)" if not second_failed else ""
        print(f"second population (half_plane/cmp_compound), live orphans: {len(comparison_live)}"
              f"  |  baseline: {len(comparison_committed_set)}{stream_note}")
        if second_failed:
            if comparison_added:
                print(f"--- {len(comparison_added)} second-population orphan(s) not in "
                      f"{COMPARISON_BASELINE.relative_to(REPO_ROOT)} ---")
                for line in comparison_added:
                    print(f"  + {line}")
            if comparison_removed:
                print(f"--- {len(comparison_removed)} baselined site(s) no longer orphaned "
                      f"(reconciled into a ledger?) ---")
                for line in comparison_removed:
                    print(f"  - {line}")
            print("regenerate with: python3 tools/ci/reconcile-assertion-ledgers.py "
                  "--write-comparison-baseline")
        elif COMPARISON_HARD_FAIL:
            print(f"FAIL COMPARISON_HARD_FAIL is set and the backlog is still {len(comparison_live)}")
            second_failed = True

        return 1 if (first_failed or second_failed) else 0

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
