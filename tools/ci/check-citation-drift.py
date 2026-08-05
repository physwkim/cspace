#!/usr/bin/env python3
# Usage: tools/ci/check-citation-drift.py
#
# Resolves every `path.rs:NNN` / `path.rs:NNN-MMM` citation in every tracked
# `.md` file against the `.rs` file it names, and reports which ones no
# longer land on what the citing text itself says they land on.
#
# Why this exists: nine branches merged into main in one round, each
# inserting lines into `.rs` files other branches' documents cite by
# `file.rs:NNN`. `tools/ci/verify-orphan-enumeration.sh` stayed green
# throughout -- and green there is not evidence the citations are right.
# Its matcher (`tools/ci/reconcile-assertion-ledgers.py`) accepts a citation
# within `NEARBY_WINDOW = 5` lines of exactly one real assertion as a match,
# with no check that the assertion it lands on is the one the citation's own
# text is about. That is content-blind: a citation for
# `resample_dt_negative_is_rejected_not_silently_truncated` drifted from its
# real line (1051) down to 1043, which is NOT within the window of 1051 but
# IS within the window of a different test's assertion at 1038 --
# `resample_dt_zero_is_rejected_not_hung`'s. The window matcher silently
# accepted that wrong match. This script is what would have caught it: it
# does not trust proximity, it checks whether the function the citation
# itself names is the function the cited line is actually inside.
#
# What "resolves" means, precisely:
#
#   1. The citation's path resolves to exactly one tracked `.rs` file --
#      an exact repo-relative path, or a bare/partial filename matching
#      exactly one tracked `.rs` file by the same subsequence-of-path-
#      components rule `reconcile-assertion-ledgers.py` uses. A citation
#      matching zero or more than one tracked file is a hard failure: it is
#      never guessed at, and never parked in a report-only bucket either --
#      that bucket held 367 citations, 17% of this corpus, checked by
#      nothing while the run still printed a total. The one exemption is by
#      path SHAPE, not by an allowlist: a citation that names a dependency
#      (`<crate>-<version>/src/...`) or a build artifact (`target/...`) is
#      counted and named separately, since no tracked file can ever satisfy
#      it. See EXTERNAL_PATH_RE.
#   2. The cited line (both ends of a `NNN-MMM` range) is in-bounds for
#      that file's current line count. Out-of-bounds is unambiguous drift
#      and always a hard failure.
#   3. If the citing text carries a nameable anchor -- a backtick-quoted
#      identifier that is also a real `fn NAME` in the resolved file,
#      either paired directly with the citation or, for a citation in the
#      first column of a table that heads that column `file:line`, named
#      anywhere in the row -- the cited line must fall within that
#      function's own brace-matched body span. A citation whose named
#      function exists elsewhere in the file, just not around the cited
#      line, is exactly the "content-blind window match" failure mode
#      above, made mechanical. See `find_tight_anchors` and `find_row_anchors`
#      for which of those two shapes a name has to have before it counts
#      as a claim at all.
#   4. Failing that, if the citing text QUOTES the code -- a backtick span
#      that is a code fragment rather than prose -- that fragment must
#      occur literally at one of the lines the citation names. See
#      `quotations_near` and MIN_QUOTATION.
#
# Rule 4 exists because rule 3 was structurally unable to check 88% of this
# corpus, and that is not a coverage gap, it is where drift lived. Rule 3
# can only check a claim of the shape "the cited line is inside function
# NAME"; what these documents actually cite is overwhelmingly not that -- a
# `const`, a struct field's doc comment, an `#[ignore]` attribute, one
# argument of one call, a `rng.random_range(0.005..0.015)` bound. None of
# those can be expressed as "inside fn NAME", so 1924 citations sat in a
# bucket the run counted and nothing checked, and fourteen citations into
# `tools/moveit-diff/src/main.rs` rotted there across the rounds that grew
# that file from ~2100 lines to 4298 -- every one still naming the line its
# subject occupied several rounds earlier.
#
# The general form of rule 3's claim is a QUOTATION, not a name: the citing
# text says what the line says, and the check is whether the line still says
# it. That covers every shape above, it is what a reader does by hand when
# they follow a citation, and it is what would have caught all fourteen --
# each already quoted its subject in its own sentence.
#
# The converse does NOT hold and is deliberately not checked: a citation
# whose quoted fragment is absent from the cited line is NOT reported as
# drift, because in this corpus a nearby backtick span is at least as often
# the function a test is ABOUT (`` `butterworth.rs:153` `` next to
# `ButterworthFilter::new`, citing an assertion inside the test that calls
# it) as it is a quotation OF the cited line. Measured here: of 641
# citations carrying a qualifying backtick span within 8 characters, 476 do
# not have that span at the line they cite, and raising the bar does not
# separate them -- restricting to spans of 40+ characters that do occur
# somewhere in the cited file still leaves 303. So a hit is evidence and a
# miss is not, and this script reports the miss as unverified rather than
# asserting drift it cannot back up.
#
# A citation with neither anchor is UNANCHORED: bounds-checked only, and
# reported as such rather than silently counted as verified -- this script
# does not manufacture confidence it cannot back up. That count is on the OK
# line for the same reason count/orphan totals are elsewhere in this repo's
# `check-*`/`verify-*` scripts: a run that verified nothing must not read
# the same as a run that verified everything. It is broken down per citing
# document rather than aggregated, because one corpus-wide number is not
# something the panel that owns one document can act on, and closing that
# residual is what would let the anchor become a requirement instead of a
# report -- see the note above the per-document listing in `main`.
#
# Named `check-*` so `ci.yml`'s glob runs it: this needs nothing but
# python3 and the tracked files -- no docker, no cargo, no upstream
# checkout. Known scope limit: only `path.rs:NNN` citations are checked.
# Upstream `.cpp`/`.hpp`/`.h` citations (also present in PORTING-PLAN.md
# and doc/port-coverage.md) are not resolvable without a local upstream
# checkout and are not covered here.
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def tracked_files():
    out = subprocess.run(
        ["git", "ls-files", "--deduplicate", "-z"], cwd=REPO_ROOT, capture_output=True, check=True
    ).stdout.decode("utf-8")
    return [p for p in out.split("\0") if p]


def path_matches(full_path, fname_part):
    """Same rule as reconcile-assertion-ledgers.py's path_matches: fname_part's
    components must occur, in order, as a subsequence of full_path's
    components, ending at the same basename."""
    want = fname_part.split("/")
    have = full_path.split("/")
    if not have or have[-1] != want[-1]:
        return False
    it = iter(have[:-1])
    return all(part in it for part in want[:-1])


CITATION_RE = re.compile(
    r"`((?:[\w./-]+/)?[\w.-]+\.rs):(\d+)(?:-(\d+))?(?:,(\d+(?:-\d+)?(?:,\d+(?:-\d+)?)*))?`"
)
# Identifiers plausible as Rust fn names -- long enough, snake_case, not a
# hex SHA (`[0-9a-f]{7,40}`, which also matches `\w+` and shows up constantly
# in these documents as commit citations, e.g. `52e38a3`).
IDENT_IN_BACKTICKS_RE = re.compile(r"`([a-z][a-z0-9_]{3,})`")
HEX_SHA_RE = re.compile(r"^[0-9a-f]{7,40}$")
FN_DEF_RE = re.compile(
    r"^\s*(?:pub(?:\([^)]*\))?\s+)?(?:default\s+)?(?:async\s+)?(?:const\s+)?"
    r"(?:unsafe\s+)?(?:extern\s+\"[^\"]*\"\s+)?fn\s+(\w+)"
)
# `` `path:NNN` (`fn_name`) `` -- the anchor immediately follows the citation,
# separated only by "(`...`)" with no other content, e.g.
# `` `mesh_search_paths.rs:139,140` (`non_package_uri_does_not_resolve`) ``.
# Kept as tight/adjacent as the backward-window rule in find_anchor, for the
# same reason: a loose forward scan would walk into the next citation's own
# anchor on a line that cites several functions in sequence.
FOLLOWING_ANCHOR_RE = re.compile(r"^\s*\(`([a-z][a-z0-9_]{3,})`\)")


def parse_cited_lines(match):
    """Every individual line number a citation's regex match covers: both
    ends of an `NNN-MMM` range, plus every comma-separated further line or
    range (`path.rs:139,140` cites lines 139 AND 140, not the range between
    them -- and both must independently resolve, not just the first, which
    an earlier version of this script silently dropped by only ever reading
    match groups 2 and 3)."""
    lines = [int(match.group(2))]
    if match.group(3):
        lines.append(int(match.group(3)))
    if match.group(4):
        for part in match.group(4).split(","):
            if "-" in part:
                a, b = part.split("-", 1)
                lines.append(int(a))
                lines.append(int(b))
            else:
                lines.append(int(part))
    return lines


def mask_non_code(text):
    """Blank out comment and string/char literal contents, byte-for-byte
    (newlines preserved), so brace-counting only sees real code braces.
    Doesn't special-case raw strings beyond their opening `"` -- rare enough
    in test bodies that a false split there would show up as a bogus
    function span, which is checked against by spot-reading, not assumed
    correct."""
    out = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j == -1 else j
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            depth, j = 1, i + 2
            while j < n and depth > 0:
                if text.startswith("/*", j):
                    depth += 1
                    j += 2
                elif text.startswith("*/", j):
                    depth -= 1
                    j += 2
                else:
                    j += 1
            out.append("".join(ch if ch == "\n" else " " for ch in text[i:j]))
            i = j
        elif c == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" and j + 1 < n else 1
            j = min(j + 1, n)
            out.append("".join(ch if ch == "\n" else " " for ch in text[i:j]))
            i = j
        elif c == "'":
            m = re.match(r"'(\\.|[^'\\\n]){1}'", text[i : i + 4])
            if m:
                out.append(" " * len(m.group(0)))
                i += len(m.group(0))
            else:
                out.append(c)
                i += 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


# A citation that names a source this repository does not contain. Both forms
# are recognised from the citation text alone, so the property holds by
# construction and there is no side table to keep in sync as versions move:
#
#   `parry3d-f64-0.30.0/src/shape/trimesh.rs:1808` -- a crates.io dependency,
#       leading component `<name>-<semver>` exactly as it appears under
#       `~/.cargo/registry/src/<index>/`. The version is what makes the line
#       number mean anything, so requiring it in the path is a stricter
#       citation than the bare `trimesh.rs:1808` this replaced, not a laxer one.
#   `target/.../moveit_msgs.rs:8031` -- a build artifact (r2r generates this
#       from the `.msg` files); it exists only inside a build tree, and its
#       line numbers are that build's, not the repo's.
#
# Anything else that fails to resolve is a hard failure: a bare `lib.rs`
# matching 23 tracked files, or a path matching none, is an unverified
# citation, and letting those accumulate in a report-only bucket is how 17%
# of this corpus went unchecked while the run still printed a total.
EXTERNAL_PATH_RE = re.compile(r"^(?:target/|[A-Za-z0-9_]+(?:-[A-Za-z0-9_]+)*-\d+\.\d+\.\d+/)")

TEST_ATTR_RE = re.compile(r"^\s*#\[[^\]]*\btest\b[^\]]*\]\s*$")
ATTR_LINE_RE = re.compile(r"^\s*#\[[^\]]*\]\s*$")
# `///` only, never `//!`: an outer doc comment belongs to the item directly
# below it, an inner one to the enclosing module, and it is the outer kind a
# citation to "this function's doc" means.
DOC_LINE_RE = re.compile(r"^\s*///")


def function_spans(path):
    """{name: [(start_line, end_line, is_test), ...]} for every `fn NAME` in
    path, brace-matched on the masked source. Multiple functions can share a
    name (nested `mod tests`, rare); every occurrence is kept and a citation
    is accepted if it falls in ANY of them.

    `is_test` -- whether one of the (up to 3) lines directly above `fn` is a
    `#[test]`/`#[tokio::test]`/... attribute -- is not for containment
    checking (a citation is either in a span or it is not, `#[test]` or no).
    It is for CANDIDATE SELECTION in the two `find_*_anchors`: this corpus
    cites two
    different things under the same "name mentioned near a citation" shape
    -- a citation legitimately inside the NAMED function's own body
    (`` `build_group_states` (`robot_model.rs:1557-1595`) ``, a direct claim
    about that production function's span), and a citation inside some
    OTHER, unnamed test that merely exercises the named production function
    (`` `acceleration_filter.rs:565` | contains | in-family | unique
    substring vs. `do_smoothing`'s other (non-folded) guard ... `` --  565
    is inside a *test*, `do_smoothing` is only what that test is about, and
    565 is nowhere near `do_smoothing`'s own body). A tight, directly-
    adjacent naming idiom (`` `name` (`path:range`) ``) disambiguates this
    by itself and doesn't need `is_test` at all; a looser "found this name
    somewhere in the row" scan does not carry that signal and produces
    exactly the `do_smoothing` false positive above unless it is
    additionally restricted to true `#[test]` functions, which this
    corpus's assertion-ledger rows are near-universally citing when the
    naming is that loose.
    """
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return {}
    masked = mask_non_code(text)
    lines = text.split("\n")
    spans = {}
    for line_no, line in enumerate(lines, 1):
        m = FN_DEF_RE.match(line)
        if m is None:
            continue
        # Find this fn's opening `{` starting from its own line (signatures
        # can wrap onto later lines before the brace appears).
        offset = sum(len(l) + 1 for l in lines[: line_no - 1])
        brace_at = masked.find("{", offset)
        if brace_at == -1:
            continue
        depth = 0
        j = brace_at
        while j < len(masked):
            if masked[j] == "{":
                depth += 1
            elif masked[j] == "}":
                depth -= 1
                if depth == 0:
                    break
            j += 1
        end_line = masked.count("\n", 0, j) + 1
        is_test = any(
            TEST_ATTR_RE.match(lines[i]) for i in range(max(0, line_no - 4), line_no - 1)
        )
        # A citation to a test commonly starts at its `#[test]` (or
        # `#[rstest]`, `#[tokio::test]`, ...) line, one line above `fn` --
        # a real, common convention, not drift. Extend the span's start to
        # include any run of single-line attributes directly above `fn`, so
        # `` `path.rs:334-345` `` for a test whose `#[test]` sits at 334 and
        # `fn` at 335 is judged against 334, not 335.
        #
        # The `///` run above those attributes is absorbed by the same walk,
        # for the same reason and by the same rule rather than a second,
        # special-cased one. `doc/claim-audit/*` cites a *claim*, and a claim
        # about upstream behaviour lives in the doc comment of the test that
        # pins it, never in its body: `moveit-trajectory.md:172` cites
        # `time_optimal_trajectory_generation.rs:962-974` naming
        # `upstream_test_custom_limits` (whose `#[test]` is at 975), and
        # `moveit-smoothing.md:32` cites `acceleration_filter.rs:446-457`
        # naming `joint_acceleration_bounds_fails_without_acceleration_limits`
        # (`#[test]` at 458). Attributes and doc comment are both leading
        # declaration material of the one function; splitting the span
        # between them would mean the citation convention this corpus
        # actually uses is checkable for one and not the other.
        start_line = line_no
        while start_line > 1 and (
            ATTR_LINE_RE.match(lines[start_line - 2])
            or DOC_LINE_RE.match(lines[start_line - 2])
        ):
            start_line -= 1
        spans.setdefault(m.group(1), []).append((start_line, end_line, is_test))
    return spans


LOCAL_WINDOW = 60

TABLE_SEP_RE = re.compile(r"^\|[\s:|-]+\|\s*$")
# The first-column headers under which a table's column-one citation is a
# source location INSIDE the function the row discusses. That is what makes
# rule 2 below sound, and it is a convention of the assertion-discrimination
# ledgers only: today 96 tables head that column `file:line` and two
# `current file:line`, a split the run now prints for itself rather than
# leaving here to rot. The claim-audit tables head it `where` -- "where in
# the port this claim is written" -- which is the doc comment that MAKES the
# claim, not an assertion inside the test the row names. Those two happen to
# coincide whenever a test states its own rationale in its own doc comment,
# and diverge as soon as the claim lives in a file header
# (`doc/claim-audit/moveit-trajectory.md:186` cites `ruckig_smoothing.rs`'s
# header comment while naming the test 150 lines below it).
#
# Every entry here must match at least one real table -- `main` fails the
# run otherwise. This is a DECLARATION of the spellings this corpus uses,
# so an entry matching nothing is either a spelling that drifted or dead
# configuration, and both are worth a failure: an entry that quietly stops
# matching takes its tables' citations out of rule 2 and leaves them in
# passing buckets. A total-count floor would not catch that, because the
# other entries keep the total non-zero -- measured, rewriting this one
# string to `File:line` still leaves 2 tables, 11 rows and 1 rule-2
# anchoring, so `> 0` on any aggregate passes while 188 citations stop
# being checked.
LEDGER_FIRST_COLUMN_HEADERS = {"file:line", "current file:line"}


def ledger_row_lines(lines):
    """`(row_line_numbers, {header: table_count})` for the tables whose first
    column is a `file:line` location -- see LEDGER_FIRST_COLUMN_HEADERS. Row
    line numbers are 1-based.

    The per-header table counts are returned rather than a bare total
    because that is what `main`'s guard needs: rule 2 checking nothing is
    invisible in every verdict (its citations just land in content-verified
    or unanchored, both passing), so the parse result itself has to be
    reported and floored, per declared spelling."""
    out = set()
    tables = {}
    for i, line in enumerate(lines):
        if not TABLE_SEP_RE.match(line) or i == 0:
            continue
        header = lines[i - 1]
        if not header.lstrip().startswith("|"):
            continue
        first = header.split("|")[1].strip()
        if first not in LEDGER_FIRST_COLUMN_HEADERS:
            continue
        tables[first] = tables.get(first, 0) + 1
        j = i + 1
        while j < len(lines) and lines[j].lstrip().startswith("|"):
            out.add(j + 1)
            j += 1
    return out, tables


def _valid_idents(text, spans, require_test=False):
    """Every backtick-quoted token in text that is a real `fn` in the target
    file (and, if require_test, has at least one `#[test]`-attributed
    occurrence), deduplicated, order not significant (candidates are
    combined with an ANY-match, not picked among)."""
    found = {}
    for m in IDENT_IN_BACKTICKS_RE.finditer(text):
        name = m.group(1)
        if HEX_SHA_RE.match(name) or name not in spans:
            continue
        if require_test and not any(is_test for (_, _, is_test) in spans[name]):
            continue
        found[name] = None
    return list(found)


def find_tight_anchors(line, match_start, match_end, spans, prev_citation_end=0, is_range=False):
    """Every plausible name anchor for one citation -- a SET of candidates,
    not a single best guess, because this corpus has no one column that
    reliably holds "the" function a citation belongs to.

    Rule 1 of two. Rule 2 is `find_row_anchors`, which the caller tries only
    when this returns nothing. They are two functions rather than one with a
    `ledger_row` argument so that WHICH rule answered is a fact about the
    call the caller made, not a value handed back with the answer: a
    returned `tight` flag was read as the containment quantifier, which gave
    one citation shape two verdicts. A caller that must never quantify by
    provenance should not be handed provenance to quantify by. The split is
    also what lets the run count rule 2's firings (`rule2_anchored`), which
    a merged function cannot report without returning that flag again.

    An earlier version of this script picked exactly one anchor per
    citation (first backtick-identifier in the row, or the row's own
    "column 3") and required the cited line to fall inside THAT function's
    span. Two real corpus shapes broke that, each surfaced by actually
    reading the false positives this script produced along the way rather
    than trusting the count:

    - A row commonly names several functions at once -- production code
      discussed in the evidence column, plus the test's own name elsewhere
      -- each paired with its OWN citation
      (`` `move_object` (`scene.rs:976`) ``, `` `frame_transform`
      (`scene.rs:1312-1332`) ``). A single row-wide anchor applied one
      row's citations to a DIFFERENT row citation's function and failed a
      citation that was exactly right -- the same content-blind failure
      shape `NEARBY_WINDOW` has, just relocated into this script.
    - Even the seemingly reliable "column 3 is the test name" shape is not
      a corpus-wide convention: in
      `doc/assertion-discrimination-ledger-pilz.md:189`, column 3 opens
      with the PRODUCTION function's name
      (`` `determine_and_check_sampling_time`'s folded ... `` ), and the
      TEST name sits in column 4 instead.
    - Widening the scan to "any name anywhere in the row" (to catch the
      column-4 case above) reintroduced the FIRST problem from the other
      direction: `doc/assertion-discrimination-ledger-p1-fixtures.md:938`
      cites `acceleration_filter.rs:565` in a row whose only nearby name is
      `do_smoothing` -- the production function *being tested*, not the
      function *containing* line 565 (which is some other, unnamed test).
      `do_smoothing`'s own span does not contain 565, and the row never
      claimed it would; the row names what the test is ABOUT, not what
      contains it. Scanning wider without also asking "is this really a
      claim about containment" turns "about" into a false "inside".

    The signal that separates a real containment claim from an "about"
    mention: a NAME DIRECTLY, ADJACENTLY PAIRED WITH A CITATION (either
    `` `name` (`path:range`) `` or `` `path:range` (`name`) ``) is always a
    containment claim -- `build_group_states` (`robot_model.rs:1557-1595`)
    asserts build_group_states' OWN span is 1557-1595, and it turns out to
    be genuinely stale (real span: 1598-1636). A name with no such direct
    pairing -- just present somewhere in the row's prose -- is NOT
    trustworthy as a containment claim on its own; it is trustworthy only
    when it is provably a *test* (`#[test]`-attributed), since this
    corpus's rows overwhelmingly name the test function loosely and the
    production function it exercises tightly-or-not-at-all, never the
    reverse.

    So, tried in order, first match wins:

    1. Tight pairing -- `` (`name`) `` immediately after this citation
       (FOLLOWING_ANCHOR_RE), or the NEAREST valid name in a bounded
       trailing window (`LOCAL_WINDOW` chars, never crossing before
       `prev_citation_end` -- see the comment at its call site).

       Adjacency alone is not always a containment claim, though: this
       corpus also pairs a production function's name tightly with a
       SINGLE test-assertion line that merely calls it --
       `doc/assertion-discrimination-ledger-p3-acm.md:634` reads "...but
       `knows_transform` is a separate function (proves only
       `world.rs:1250`'s sensitivity)" -- 1250 is
       `assert!(!world.knows_transform("nothing"))`, a line INSIDE THE
       TEST `transform_lookup_unknown_name_errors`, not inside
       `knows_transform` itself (whose own span is nowhere near 1250);
       the row is citing evidence *about* `knows_transform`, not
       asserting where it lives. What distinguishes a real "this IS the
       function's span" claim (`` `build_group_states`
       (`robot_model.rs:1557-1595`) ``, `` `decouple_parent`
       (`scene.rs:2003-2021`) `` -- both correctly caught real drift) from
       an "evidence of calling it" mention is that the real span claims
       are always CITED AS A RANGE (`NNN-MMM`, `is_range`) spanning enough
       lines to plausibly be a function body -- nobody cites a 20+ line
       range as "the one line that calls this". So: a single-LINE tight
       pairing is trusted only if the paired name is itself `#[test]`-
       attested (the `move_object` (`scene.rs:976`) case, citing that
       test's own single assertion, or a bare fn declaration line);
       a range pairing is trusted regardless, since the range shape
       itself is strong enough evidence of a containment claim.
    2. `find_row_anchors` -- see there.

    Accept the citation if it falls inside ANY candidate's span -- not
    exactly one -- for the same reason a row can mention several correctly-
    cited functions at once (`move_object`, `decouple_parent`,
    `frame_transform` in one ledger's disposition list).

    An empty result leaves the citation unverified (bounds-only) rather
    than guessed at -- see the module docstring.
    """
    require_test = not is_range
    candidates = []
    following = FOLLOWING_ANCHOR_RE.match(line[match_end:])
    if following is not None:
        name = following.group(1)
        if (
            not HEX_SHA_RE.match(name)
            and name in spans
            and (not require_test or any(t for (_, _, t) in spans[name]))
        ):
            candidates.append(name)

    window_start = max(prev_citation_end, match_start - LOCAL_WINDOW)
    window = line[window_start:match_start]
    for name in _valid_idents(window, spans, require_test=require_test):
        if name not in candidates:
            candidates.append(name)
    return candidates


def find_row_anchors(line, match_start, match_end, spans):
    """Rule 2: for a citation in the FIRST COLUMN of a table row, with no
    rule-1 pairing found, in a table that heads that column `file:line` --
    if the row loosely names at least one `#[test]`-attested function in
    the cited file, every valid identifier anywhere else in the row;
    otherwise nothing. The caller owns the table gate (`ledger_row_lines`)
    and reaches this only for a row inside one.

    Two independent gates, and both are needed. The table gate is what
    makes the rule's premise true at all: only a `file:line` column says
    the cited line is a location inside what the row discusses. The
    claim-audit tables head that column `where` -- "where in the port this
    claim is written" -- which is the doc comment that MAKES the claim and
    need not sit inside the test the row names
    (`doc/claim-audit/moveit-trajectory.md:186` cites
    `ruckig_smoothing.rs`'s file header while naming the test 150 lines
    below it). Firing rule 2 there verified fifteen claim-audit rows on a
    premise that does not hold for them.

    The row gate decides which names in a qualifying row count.
    `#[test]`-attestation gates the ROW, not each candidate. What it
    establishes is that this row is using the ledger's loose test-naming
    idiom at all; the `do_smoothing` row in `find_tight_anchors`' docstring
    names NO test in `acceleration_filter.rs`, which is exactly why its
    lone production-function mention must not be trusted. Once a row is in
    the idiom, the function containing the cited line is sometimes a plain
    helper rather than a test -- a row citing a `mod tests` fixture builder
    and naming the tests that cover it is the shape -- and rejecting it for
    want of a `#[test]` attribute rejects a correct citation over a fact
    about the containing function that the check never claimed to be about.
    Attesting each candidate separately instead was measured over this
    corpus at 75 new failures, `do_smoothing`'s own citation
    (`p1-fixtures.md:938`, `acceleration_filter.rs:565`) among them.

    The names come from anywhere in the row, so they are the functions the
    row *discusses*. That does NOT license a weaker containment quantifier
    than rule 1's -- see the caller.
    """
    if not line.lstrip().startswith("|"):
        return []
    pipes = [i for i, ch in enumerate(line) if ch == "|"]
    if not (len(pipes) >= 2 and pipes[0] < match_start < pipes[1]):
        return []
    rest_of_row = line[:match_start] + line[match_end:]
    if not _valid_idents(rest_of_row, spans, require_test=True):
        return []
    return _valid_idents(rest_of_row, spans, require_test=False)


# Rule 4's floor for a backtick span to count as a quotation of code rather
# than a word of prose. Both halves matter, and both were set by reading
# what they let through: without the delimiter test, `` `revolute` ``/
# `` `contains` ``/`` `Default` `` land somewhere in almost any Rust file by
# accident and would verify a citation against nothing; without the length
# floor, `` `..` ``/`` `&mut` `` do the same. Sweeping the floor over this
# corpus moves the rule-4 count 646 (6) / 621 (8) / 580 (10) -- no cliff, so
# 8 is chosen as the point where every single-word token in a spot-read of
# the newly-verified set was still a real identifier.
MIN_QUOTATION = 8
QUOTATION_DELIMS = ("(", ")", "::", "!", "=", "..", "->", "[", "]", '"', "&", ".", "/", "_")
BACKTICK_SPAN_RE = re.compile(r"`([^`\n]+)`")
# `path.ext:NNN` for ANY extension, not just `.rs`: an upstream citation
# (`planning_scene.cpp:1496`) sits next to a port citation constantly in
# these documents, and a pointer is never a quotation of the port's line.
ANY_CITATION_RE = re.compile(r"^[\w./-]+\.\w+:\d+(?:[-,]\d+)*$")
BARE_PATH_RE = re.compile(r"^[\w./-]+\.\w+$")
# Markdown escapes a cell-splitting `|` inside a table; the source being
# quoted has the bare operator. Without this, every folded-operand guard
# quoted in `doc/folded-operand-guards.md` (`a.is_empty() \|\| b.is_empty()`)
# fails to match the code it is quoting verbatim.
MD_ESCAPES = (("\\|", "|"), ("\\*", "*"), ("\\_", "_"), ("\\<", "<"), ("\\`", "`"))


def citing_context(lines, index):
    """The text a citation's claim is made in, 0-based `index` being its own
    line. A table row is its own context -- a neighbouring row is a different
    claim about a different site. Prose is the whole blank-line-delimited
    paragraph, because these documents wrap Korean prose at ~72 columns and a
    citation's subject lands on the line above or below as readily as on its
    own: `PORTING-PLAN.md:9384` names its test on the preceding line, and
    `:9916` names `satisfied` on the preceding line."""
    if lines[index].lstrip().startswith("|"):
        return lines[index]
    start = index
    while start > 0 and lines[start - 1].strip() and not lines[start - 1].lstrip().startswith("|"):
        start -= 1
    end = index
    while (
        end + 1 < len(lines)
        and lines[end + 1].strip()
        and not lines[end + 1].lstrip().startswith("|")
    ):
        end += 1
    return "\n".join(lines[start : end + 1])


def quotations_near(context):
    """Every backtick span in `context` that is a quotation of code rather
    than a word of prose or a pointer: at least MIN_QUOTATION characters,
    carrying at least one QUOTATION_DELIMS delimiter, and not itself a
    citation, a bare path, or a commit SHA."""
    out = []
    for m in BACKTICK_SPAN_RE.finditer(context):
        token = m.group(1).strip()
        for escaped, raw in MD_ESCAPES:
            token = token.replace(escaped, raw)
        if len(token) < MIN_QUOTATION or HEX_SHA_RE.match(token):
            continue
        if ANY_CITATION_RE.match(token) or BARE_PATH_RE.match(token):
            continue
        if not any(d in token for d in QUOTATION_DELIMS):
            continue
        out.append(token)
    return out


def cited_regions(match):
    """The (start, end) spans a citation names -- spans, not the flat
    endpoint list `parse_cited_lines` returns. Rule 4 asks whether the quoted
    text is anywhere INSIDE a cited range, which the endpoints alone cannot
    answer: `main.rs:2731-2741` quotes `oracle_only`, which is at 2739."""
    regions = [(int(match.group(2)), int(match.group(3) or match.group(2)))]
    if match.group(4):
        for part in match.group(4).split(","):
            if "-" in part:
                a, b = part.split("-", 1)
                regions.append((int(a), int(b)))
            else:
                regions.append((int(part), int(part)))
    return regions


def content_anchor(quotations, body, regions):
    """The first quotation occurring literally somewhere in a cited region,
    or None. `body` is the resolved file's lines."""
    for start, end in regions:
        region = "\n".join(body[start - 1 : min(end, len(body))])
        for token in quotations:
            if token in region:
                return token
    return None


def resolve_path(fname_part, rs_files_by_basename, rs_files_set):
    if "/" in fname_part:
        if fname_part in rs_files_set:
            return [fname_part]
        return [p for p in rs_files_set if path_matches(p, fname_part)]
    basename = fname_part
    return sorted(rs_files_by_basename.get(basename, []))


def main():
    tracked = tracked_files()
    md_files = [p for p in tracked if p.endswith(".md")]
    rs_files_set = {p for p in tracked if p.endswith(".rs")}
    rs_files_by_basename = {}
    for p in rs_files_set:
        rs_files_by_basename.setdefault(p.rsplit("/", 1)[-1], []).append(p)

    span_cache = {}

    def spans_for(path):
        if path not in span_cache:
            span_cache[path] = function_spans(REPO_ROOT / path)
        return span_cache[path]

    total = 0
    unresolved = []  # (md, line_no, fname_part, reason)
    external = []  # (md, line_no, fname_part) -- EXTERNAL_PATH_RE, exempt by shape
    out_of_bounds = []  # (md, line_no, fname_part, cited_line, resolved_path, file_len)
    anchor_mismatch = []  # (md, line_no, fname_part, cited_line, anchor, resolved_path, spans_for_anchor)
    anchor_verified = 0
    partly_anchored = 0
    content_verified = 0
    unanchored_by_file = {}
    # Rule 2's own parse result, at each of the three stages it can go quiet
    # at: which declared header spellings matched a table, how many rows
    # those tables held, and how many citations the rule actually anchored.
    # None of the three is visible in any verdict -- see the guard below.
    ledger_tables = {}
    ledger_rows_total = 0
    rule2_anchored = 0

    for md in md_files:
        text = (REPO_ROOT / md).read_text(encoding="utf-8", errors="replace")
        lines = text.split("\n")
        ledger_rows, tables_here = ledger_row_lines(lines)
        for header, n in tables_here.items():
            ledger_tables[header] = ledger_tables.get(header, 0) + n
        ledger_rows_total += len(ledger_rows)
        for line_no, line in enumerate(lines, 1):
            prev_citation_end = 0
            for m in CITATION_RE.finditer(line):
                fname_part = m.group(1)
                cited_lines = parse_cited_lines(m)
                total += 1
                # Captured before updating prev_citation_end for the NEXT
                # iteration, so this citation's own anchor search still
                # sees where the PREVIOUS citation ended, regardless of
                # which `continue` below this one takes.
                window_floor, prev_citation_end = prev_citation_end, m.end()

                if EXTERNAL_PATH_RE.match(fname_part):
                    external.append((md, line_no, fname_part))
                    continue

                candidates = resolve_path(fname_part, rs_files_by_basename, rs_files_set)
                if len(candidates) != 1:
                    unresolved.append(
                        (
                            md,
                            line_no,
                            fname_part,
                            "no tracked .rs file matches"
                            if not candidates
                            else f"ambiguous: matches {len(candidates)} tracked files: {candidates}",
                        )
                    )
                    continue
                resolved_path = candidates[0]
                file_len = len(
                    (REPO_ROOT / resolved_path).read_text(encoding="utf-8", errors="replace").split("\n")
                )
                oob = [ln for ln in cited_lines if not (1 <= ln <= file_len)]
                if oob:
                    out_of_bounds.append((md, line_no, fname_part, cited_lines, resolved_path, file_len))
                    continue

                spans = spans_for(resolved_path)
                body = (
                    (REPO_ROOT / resolved_path)
                    .read_text(encoding="utf-8", errors="replace")
                    .split("\n")
                )
                # Rule 4's verdict, computed once here so that every path
                # which used to fall through to bounds-only reaches it: a
                # citation is never filed as unverified while it is in fact
                # quoting the line it names.
                quoted = content_anchor(
                    quotations_near(citing_context(lines, line_no - 1)), body, cited_regions(m)
                )

                # A citation landing entirely above the file's first function
                # is to file-level content -- the header comment, the module
                # doc, the `use` block -- and no function span can contain it,
                # so asking whether one does is a category error rather than a
                # drift check. `doc/claim-audit/moveit-trajectory.md:186`
                # cites `tests/ruckig_smoothing.rs:16-19`, the paragraph of
                # that file's header stating the `single_waypoint` deviation,
                # in a row whose evidence prose happens to name the test that
                # deviation belongs to; its sibling rows citing the same kind
                # of header (`:13`, `:30`) pass only because no name in them
                # resolves. Left unverified rather than guessed at, the same
                # disposition an absent anchor gets.
                first_fn = min(
                    (start for occ in spans.values() for (start, _e, _t) in occ),
                    default=None,
                )
                if first_fn is not None and all(ln < first_fn for ln in cited_lines):
                    if quoted:
                        content_verified += 1
                    else:
                        unanchored_by_file[md] = unanchored_by_file.get(md, 0) + 1
                    continue

                anchors = find_tight_anchors(
                    line,
                    m.start(),
                    m.end(),
                    spans,
                    window_floor,
                    is_range=m.group(3) is not None,
                )
                if not anchors and line_no in ledger_rows:
                    anchors = find_row_anchors(line, m.start(), m.end(), spans)
                    if anchors:
                        rule2_anchored += 1
                if not anchors:
                    if quoted:
                        content_verified += 1
                    else:
                        unanchored_by_file[md] = unanchored_by_file.get(md, 0) + 1
                    continue

                # A cited line lands if it is in SOME candidate's span -- not
                # all cited lines in the SAME one. A single citation can
                # legitimately name several sibling functions at once (e.g.
                # `robot_model.rs:2329,2344` for `no_root_link_errors`'s and
                # `multiple_root_links_errors`' own assert lines in one
                # comma-list); requiring one span to cover every line broke
                # exactly that shape, discovered by spot-reading this
                # script's own output against a citation this round's fixes
                # had just corrected.
                landed = [
                    ln
                    for ln in cited_lines
                    if any(
                        start <= ln <= end
                        for name in anchors
                        for (start, end, _is_test) in spans[name]
                    )
                ]
                # How many of the cited lines must land is a property of the
                # CITATION'S SHAPE, not of how its anchor happened to attach.
                # It used to be both, and the two disagreed about one shape:
                # a partly-contained comma list dropped to bounds-only when a
                # tight pairing named the function and counted as
                # anchor-verified when a rule-2 row did. Same citation, same
                # evidence, two verdicts -- and the passing one put nine of
                # `p9-ros.md:323`'s eleven lines under an "anchor-verified
                # (cited line inside the named function's body)" total that
                # was false for them.
                #
                # A comma list is an ENUMERATION: `p9-ros.md:323` cites all
                # 11 of `scene/collision_object.rs`'s assertion sites and
                # names the two whose citation that round corrected, `:326`
                # cites 3 and names 1. All 14 are exact current
                # `count-coarse-assertions.py` sites, so demanding every one
                # sit inside a named test failed citations that were right.
                # A RANGE is not an enumeration -- `a-b` claims one span, so
                # `a` inside and `b` outside is the range straddling the
                # function's end, which is drift and stays a failure.
                #
                # ZERO containment is a failure under either shape; that is
                # what caught `p1-robotmodel.md:825`, whose 2 cited lines were
                # in neither named test.
                if len(landed) == len(cited_lines):
                    anchor_verified += 1
                elif landed and m.group(4) is not None:
                    # Falls through to rule 4 rather than passing or failing:
                    # partial containment carries no drift signal either way,
                    # and both of today's two are quoted, so rule 4 has real
                    # per-line evidence to offer them that this bucket would
                    # throw away. Counted here so the shape is visible on the
                    # OK line instead of hiding inside another bucket's total.
                    partly_anchored += 1
                    if quoted:
                        content_verified += 1
                    else:
                        unanchored_by_file[md] = unanchored_by_file.get(md, 0) + 1
                else:
                    anchor_mismatch.append(
                        (md, line_no, fname_part, cited_lines, landed, anchors, resolved_path, spans)
                    )

    hard_fail = bool(out_of_bounds) or bool(anchor_mismatch) or bool(unresolved)
    counts = (
        f"{anchor_verified} anchor-verified (EVERY cited line inside a named function's body), "
        f"{content_verified} content-verified (the citing text's own quotation of the code is "
        f"at the cited line), "
        f"{sum(unanchored_by_file.values())} unanchored (bounds-checked only -- the citing text "
        f"neither names a containing function nor quotes the line), "
        f"{len(external)} exempt (names a dependency or a build artifact, see above), "
        f"{len(out_of_bounds)} out-of-bounds, {len(anchor_mismatch)} anchor-mismatch, "
        f"{len(unresolved)} unresolvable"
        f"; {partly_anchored} of those are partly-anchored enumerations (a comma list whose "
        f"named functions hold some of its cited lines and not others), counted in whichever "
        f"bucket rule 4 put them"
        f"; rule 2 read "
        + ", ".join(f"{ledger_tables.get(h, 0)} `{h}`" for h in sorted(LEDGER_FIRST_COLUMN_HEADERS))
        + f" table(s) totalling {ledger_rows_total} row(s) and anchored "
        f"{rule2_anchored} citation(s)"
    )

    if total == 0:
        print("FAIL parsed zero `path.rs:NNN` citations across tracked .md files -- the citation grammar changed and this checked nothing", file=sys.stderr)
        return 1

    # Rule 2 going quiet is invisible in every verdict: the citations it
    # would have anchored land in content-verified or unanchored instead,
    # and both of those pass. So its parse result is floored directly, at
    # each stage that can independently reach nothing -- and PER DECLARED
    # HEADER rather than on a total, because the totals stay comfortably
    # non-zero while a spelling dies. Measured by mutation on this corpus:
    # rewriting `file:line` to `File:line` above leaves 2 tables, 11 rows
    # and 1 rule-2 anchoring -- every aggregate still positive -- while
    # anchor-verified falls from 249 to 63, 188 citations quietly move into
    # content-verified and unanchored, and the run exits 0.
    #
    # Of the three, the per-header floor and the anchoring floor each catch
    # a mutation nothing else here catches (that same header rewrite; and
    # `find_row_anchors` returning [] with tables and rows intact). The row
    # floor does not: neutralising the body-row scan takes the anchoring
    # count to zero too, so the run fails either way. It is kept because the
    # anchoring floor's message is then WRONG -- it says "its tables and
    # rows parsed", which is exactly what did not happen -- and a gate that
    # fails with a false cause sends the next reader into the wrong
    # function.
    for got, what in (
        *(
            (ledger_tables.get(h, 0), f"matched no table heading its first column `{h}` -- that "
                                      f"spelling is in LEDGER_FIRST_COLUMN_HEADERS, so it either "
                                      f"drifted in the ledgers or is dead configuration")
            for h in sorted(LEDGER_FIRST_COLUMN_HEADERS)
        ),
        (ledger_rows_total, "matched tables but found no body rows under any of them -- the row "
                            "scan in `ledger_row_lines` no longer recognises this corpus"),
        (rule2_anchored, "anchored no citation at all -- its tables and rows parsed, so the "
                         "first-column or `#[test]`-attestation gate in `find_row_anchors` is "
                         "matching nothing"),
    ):
        if got == 0:
            print(f"FAIL rule 2 {what}", file=sys.stderr)
            return 1

    if out_of_bounds:
        print(f"--- {len(out_of_bounds)} out-of-bounds citation(s) ---", file=sys.stderr)
        for md, line_no, fname_part, cited_lines, resolved_path, file_len in out_of_bounds:
            print(
                f"FAIL {md}:{line_no}: cites {fname_part}:{'-'.join(map(str, cited_lines))}, "
                f"but {resolved_path} has only {file_len} lines",
                file=sys.stderr,
            )

    if anchor_mismatch:
        print(f"--- {len(anchor_mismatch)} anchor-mismatch citation(s) ---", file=sys.stderr)
        for md, line_no, fname_part, cited_lines, landed, anchors, resolved_path, spans in anchor_mismatch:
            candidate_desc = "; ".join(
                f"`{name}` spans {', '.join(f'{s}-{e}' for s, e, _ in spans[name])}" for name in anchors
            )
            # `landed` is spelled out rather than summarised as "none": a
            # partly-contained RANGE reaches here (only an enumeration is
            # allowed to be partial), and telling its author that none of
            # `723-740` is inside the function that holds 723 sends them
            # looking for the wrong defect.
            missed = [ln for ln in cited_lines if ln not in landed]
            which = (
                "inside none of"
                if not landed
                else f"has {', '.join(map(str, missed))} outside every one of"
            )
            print(
                f"FAIL {md}:{line_no}: cites {fname_part}:{','.join(map(str, cited_lines))}, "
                f"{which} its {len(anchors)} nearby candidate function(s) in {resolved_path}: "
                f"{candidate_desc}",
                file=sys.stderr,
            )

    if external:
        print(
            f"--- {len(external)} citation(s) exempt: the path names a source this "
            f"repository does not contain ---",
            file=sys.stderr,
        )
        for md, line_no, fname_part in external:
            why = (
                "build artifact, regenerated per build tree"
                if fname_part.startswith("target/")
                else f"dependency source, pinned at {fname_part.split('/', 1)[0]}"
            )
            print(f"  {md}:{line_no}: `{fname_part}` -- {why}", file=sys.stderr)

    if unanchored_by_file:
        # Per citing document, never one corpus-wide total. The residual is a
        # work list and the panel that can act on it owns one document, so
        # `PORTING-PLAN.md: 195` is actionable in a way "1419 unanchored" is
        # not. It is also the gate's own precondition: an unanchored citation
        # cannot be made a hard failure while 1419 of them exist across 46
        # documents, so the number here is what has to reach zero before this
        # bucket can stop being a report and start being a requirement.
        print(
            f"--- {sum(unanchored_by_file.values())} unanchored citation(s), by citing "
            f"document: bounds-checked only, so an insertion in the file they name rots "
            f"them silently and nothing here can tell. Give one a claim to check by "
            f"quoting, in its own sentence or table row, a fragment of the line it cites "
            f"(>= {MIN_QUOTATION} characters, carrying a code delimiter) ---",
            file=sys.stderr,
        )
        for md, count in sorted(unanchored_by_file.items(), key=lambda kv: (-kv[1], kv[0])):
            print(f"  {count:5d}  {md}", file=sys.stderr)

    if unresolved:
        print(f"--- {len(unresolved)} citation(s) to an unresolvable path ---", file=sys.stderr)
        for md, line_no, fname_part, reason in unresolved:
            print(
                f"FAIL {md}:{line_no}: `{fname_part}` -- {reason}. Qualify it to the "
                f"repo-relative path it means, or -- if it names a dependency or a "
                f"build artifact -- write it in the form that says so "
                f"(`<crate>-<version>/src/...`, `target/...`).",
                file=sys.stderr,
            )

    if hard_fail:
        print(
            f"FAIL of {total} `.rs` citations across {len(md_files)} tracked .md files "
            f"(corpus: every `` `path.rs:NNN[-MMM]` `` span in every tracked .md file): "
            f"{counts}",
            file=sys.stderr,
        )
        return 1

    print(f"OK {total} `.rs` citations across {len(md_files)} tracked .md files: {counts}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
