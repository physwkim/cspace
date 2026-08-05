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
#      components rule `reconcile-assertion-ledgers.py` uses (a citation
#      matching zero or more than one tracked file is reported, never
#      guessed at).
#   2. The cited line (both ends of a `NNN-MMM` range) is in-bounds for
#      that file's current line count. Out-of-bounds is unambiguous drift
#      and always a hard failure.
#   3. If the citing text carries a nameable anchor -- a backtick-quoted
#      identifier that is also a real `fn NAME` in the resolved file, found
#      in the same table row (up to its 4th `|`, which is where every
#      ledger sampled puts its test-name column, never its free-text
#      explanation column) or on the same prose line -- the cited line must
#      fall within that function's own brace-matched body span. A citation
#      whose named function exists elsewhere in the file, just not around
#      the cited line, is exactly the "content-blind window match" failure
#      mode above, made mechanical.
#
# A citation with no such anchor is bounds-checked only and reported as
# such, not silently counted as verified -- this script does not manufacture
# confidence it cannot back up with a name to check against. That count is
# on the OK line for the same reason count/orphan totals are elsewhere in
# this repo's `check-*`/`verify-*` scripts: a run that verified nothing must
# not read the same as a run that verified everything.
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
        ["git", "ls-files", "-z"], cwd=REPO_ROOT, capture_output=True, check=True
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
    It is for CANDIDATE SELECTION in find_anchors: this corpus cites two
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


def find_anchors(line, match_start, match_end, spans, prev_citation_end=0, is_range=False):
    """Every plausible name anchor for one citation -- a SET of candidates,
    not a single best guess, because this corpus has no one column that
    reliably holds "the" function a citation belongs to.

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
    2. Table row, first column, with no tight pairing found: every valid
       identifier anywhere else in the row THAT IS `#[test]`-attested.
       Production-function names loosely mentioned in the same row are
       excluded here precisely because they are not reliably containment
       claims (`do_smoothing` above).

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
    if candidates:
        return candidates

    stripped = line.lstrip()
    if stripped.startswith("|"):
        pipes = [i for i, ch in enumerate(line) if ch == "|"]
        if len(pipes) >= 2 and pipes[0] < match_start < pipes[1]:
            rest_of_row = line[:match_start] + line[match_end:]
            return _valid_idents(rest_of_row, spans, require_test=True)

    return []


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
    out_of_bounds = []  # (md, line_no, fname_part, cited_line, resolved_path, file_len)
    anchor_mismatch = []  # (md, line_no, fname_part, cited_line, anchor, resolved_path, spans_for_anchor)
    anchor_verified = 0
    bounds_only = 0

    for md in md_files:
        text = (REPO_ROOT / md).read_text(encoding="utf-8", errors="replace")
        lines = text.split("\n")
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
                    bounds_only += 1
                    continue

                anchors = find_anchors(
                    line, m.start(), m.end(), spans, window_floor, is_range=m.group(3) is not None
                )
                if not anchors:
                    bounds_only += 1
                    continue

                # Each cited line must land in SOME candidate's span -- not
                # all cited lines in the SAME one. A single citation can
                # legitimately name several sibling functions at once (e.g.
                # `robot_model.rs:2329,2344` for `no_root_link_errors`'s and
                # `multiple_root_links_errors`' own assert lines in one
                # comma-list); requiring one span to cover every line broke
                # exactly that shape, discovered by spot-reading this
                # script's own output against a citation this round's fixes
                # had just corrected.
                owned = [
                    ln
                    for ln in cited_lines
                    if any(
                        start <= ln <= end
                        for name in anchors
                        for (start, end, _is_test) in spans[name]
                    )
                ]
                if len(owned) == len(cited_lines):
                    anchor_verified += 1
                elif owned and m.group(4) is not None:
                    # A comma list is an ENUMERATION, and a row that
                    # enumerates names the tests it discusses, not one per
                    # cited line: `p9-ros.md:323` cites all 11 of
                    # `scene/collision_object.rs`'s assertion sites and names
                    # the two whose citation that round corrected, `:326`
                    # cites 3 and names 1. Every one of those 14 lines is an
                    # exact current `count-coarse-assertions.py` site, so
                    # demanding all 11 sit inside the 2 named tests failed
                    # citations that were right. Partial containment is the
                    # census shape and carries no drift signal either way, so
                    # it drops to bounds-only rather than passing or failing.
                    # ZERO containment stays a failure -- that is what caught
                    # `p1-robotmodel.md:825`, whose 2 cited lines were in
                    # neither named test.
                    bounds_only += 1
                else:
                    anchor_mismatch.append(
                        (md, line_no, fname_part, cited_lines, anchors, resolved_path, spans)
                    )

    failures = list(out_of_bounds) or list(anchor_mismatch)
    hard_fail = bool(out_of_bounds) or bool(anchor_mismatch)

    if total == 0:
        print("FAIL parsed zero `path.rs:NNN` citations across tracked .md files -- the citation grammar changed and this checked nothing", file=sys.stderr)
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
        for md, line_no, fname_part, cited_lines, anchors, resolved_path, spans in anchor_mismatch:
            candidate_desc = "; ".join(
                f"`{name}` spans {', '.join(f'{s}-{e}' for s, e, _ in spans[name])}" for name in anchors
            )
            print(
                f"FAIL {md}:{line_no}: cites {fname_part}:{'-'.join(map(str, cited_lines))}, "
                f"inside none of its {len(anchors)} nearby candidate function(s) in {resolved_path}: "
                f"{candidate_desc}",
                file=sys.stderr,
            )

    if unresolved:
        print(f"--- {len(unresolved)} citation(s) to an unresolvable path (not counted as failures; report only) ---", file=sys.stderr)
        for md, line_no, fname_part, reason in unresolved:
            print(f"  {md}:{line_no}: `{fname_part}` -- {reason}", file=sys.stderr)

    if hard_fail:
        print(
            f"FAIL {len(out_of_bounds)} out-of-bounds + {len(anchor_mismatch)} anchor-mismatch "
            f"(of {total} `.rs` citations checked; corpus: every `` `path.rs:NNN[-MMM]` `` "
            f"span in every tracked .md file)",
            file=sys.stderr,
        )
        return 1

    print(
        f"OK {total} `.rs` citations across {len(md_files)} tracked .md files: "
        f"{anchor_verified} anchor-verified (cited line inside the named function's body), "
        f"{bounds_only} bounds-checked only (no nameable anchor in the citing text), "
        f"{len(unresolved)} unresolved-path (reported above, not a hard failure), "
        f"0 out-of-bounds, 0 anchor-mismatch"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
