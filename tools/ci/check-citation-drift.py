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
#   3. If the citing text names a Rust ITEM of the resolved file DIRECTLY
#      against the citation -- `` `name`(`path.rs:N`) `` or
#      `` `path.rs:N` (`name`) ``, nothing between them but a space and the
#      paren -- every cited line must be inside that item's own span (its
#      declaration, doc comment and attributes), or the name must occur on
#      one of the lines the citation itself names, which is that same
#      parenthetical written as a quotation of the line rather than as an
#      anchor. This one has no third outcome: adjacency IS the claim, so it
#      is confirmed or it is a hard failure, never parked in the unanchored
#      bucket. Item means item, resolved from the
#      source: `fn`, `trait`, `struct`, `enum`, `union`, `mod`, `type`,
#      `const`/`static`, `macro_rules!`, and every `impl` block under the
#      type it is for. See `find_item_anchor`, `item_spans`, `anchor_zones`.
#   3a. Failing that, if the citing text carries a nameable anchor -- a
#      backtick-quoted identifier that is also a real `fn NAME` in the
#      resolved file,
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
from collections import namedtuple
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# The class a citation is assigned, as a VALUE. Every citation gets exactly
# one, and every count this script prints is derived from the assignments --
# no counter is incremented directly.
#
# The three verified/unverified classes are RANKED by how much of the
# citation's own claim was actually checked, and that ranking is the point:
# `anchor-verified` checked that the cited line is inside the function the
# text names, `content-verified` checked that the text's quotation is at the
# cited line, `unanchored` checked only that the line number is within the
# file. Drift moves a citation DOWN this ladder -- shifting
# `velocity_profile.rs:423` to `:823` takes it from content-verified to
# unanchored -- and with only totals printed, a demotion is indistinguishable
# from a document being edited. That is why 6 of 8 randomly shifted citations
# used to pass: the shift was absorbed as a class demotion instead of being
# reported. CLASS_BASELINE freezes the per-citation class so a demotion is a
# failure and can only be accepted by declaring it.
ANCHOR_VERIFIED = "anchor-verified"
CONTENT_VERIFIED = "content-verified"
UNANCHORED = "unanchored"
CLASS_RANK = {ANCHOR_VERIFIED: 3, CONTENT_VERIFIED: 2, UNANCHORED: 1}
# Not on the ladder. `exempt` names a path no tracked file can ever satisfy,
# so it is not a weaker check of the same claim but a different claim; the
# other three are already hard failures and never reach the baseline as a
# passing class. All four compare by equality only.
EXEMPT = "exempt"
OUT_OF_BOUNDS = "out-of-bounds"
ANCHOR_MISMATCH = "anchor-mismatch"
UNRESOLVED = "unresolvable"
# A cited line that is empty or whitespace-only. Not a class on the ladder --
# see the predicate itself for why it sits above it.
BLANK_LINE = "blank-line"
HARD_FAIL_CLASSES = (OUT_OF_BOUNDS, ANCHOR_MISMATCH, UNRESOLVED, BLANK_LINE)

Citation = namedtuple("Citation", "md line_no spec cls detail partly")

# The committed per-citation class baseline -- same arrangement as
# `doc/assertion-discrimination-orphans.txt` and its
# `verify-orphan-enumeration.sh`: a generated file, regenerated by the
# command named in its own header, compared exactly on every run.
CLASS_BASELINE = "doc/citation-classes.txt"
BASELINE_ROW_RE = re.compile(r"^(\S+)\t(\S+)\t(.+)$")


def class_map(records):
    """`{(citing_doc, citation_spec): sorted_class_list}` over every citation.

    Keyed by the citation's TEXT rather than by its line in the citing
    document, and valued by the multiset of classes its occurrences got
    rather than by one class per occurrence, because both alternatives churn
    on edits that are not drift: inserting a paragraph renumbers every
    citation below it, and reordering two table rows swaps the occurrence
    indices of the 629 citations (27% of this corpus) whose `(doc, spec)`
    key is not unique -- `declaration-audit-coverage.md` cites
    `crates/moveit-planners-chomp/src/lib.rs:99` twelve times. Under either,
    a real demotion would be one line lost in a churn diff.

    What this key CANNOT absorb is the citation's own line number changing,
    which is the thing being watched: shifting `velocity_profile.rs:423` to
    `:823` retires one key and introduces another, and an undeclared key is
    a failure.
    """
    out = {}
    for r in records:
        out.setdefault((r.md, r.spec), []).append(r.cls)
    return {k: sorted(v, key=lambda c: (-CLASS_RANK.get(c, 0), c)) for k, v in out.items()}


def render_classes(classes):
    """One tab-separated row per `(doc, spec)`, classes most-checked first."""
    rows = []
    for (md, spec), clss in sorted(classes.items()):
        counted = []
        for cls in dict.fromkeys(clss):
            k = clss.count(cls)
            counted.append(cls if k == 1 else f"{cls}*{k}")
        rows.append(f"{md}\t{spec}\t{' '.join(counted)}")
    return rows


def parse_classes(text):
    """Inverse of `render_classes`. Returns `(map, n_malformed)` -- a
    malformed row is counted rather than skipped, so a format change cannot
    quietly shrink the baseline to the rows that still happen to parse."""
    out = {}
    malformed = 0
    for line in text.split("\n"):
        if not line.strip() or line.startswith("#"):
            continue
        m = BASELINE_ROW_RE.match(line)
        if m is None:
            malformed += 1
            continue
        clss = []
        for tok in m.group(3).split():
            # `*`, not `x`: `exempt` contains an x, so partitioning on one
            # split it into `e` + `empt` and 24 of this corpus's 1912 rows
            # parsed as malformed. Counting malformed rows rather than
            # skipping them is what surfaced that.
            cls, _, k = tok.partition("*")
            if cls not in CLASS_RANK and cls not in (EXEMPT, *HARD_FAIL_CLASSES):
                malformed += 1
                break
            if k and not k.isdigit():
                malformed += 1
                break
            clss.extend([cls] * (int(k) if k else 1))
        else:
            out[(m.group(1), m.group(2))] = sorted(
                clss, key=lambda c: (-CLASS_RANK.get(c, 0), c)
            )
    return out, malformed


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
# Every named item kind. The `fn` alternative must stay first: `const fn
# foo` has to read as a fn named `foo`, not as a const named `fn`.
ITEM_DEF_RE = re.compile(
    r"^\s*(?:pub(?:\([^)]*\))?\s+)?(?:default\s+)?(?:"
    r"(?:async\s+)?(?:const\s+)?(?:unsafe\s+)?(?:extern\s+\"[^\"]*\"\s+)?fn\s+(?P<fn>\w+)"
    r"|(?:unsafe\s+)?(?:auto\s+)?trait\s+(?P<trait>\w+)"
    r"|struct\s+(?P<struct>\w+)"
    r"|enum\s+(?P<enum>\w+)"
    r"|union\s+(?P<union>\w+)"
    r"|(?:unsafe\s+)?mod\s+(?P<mod>\w+)"
    r"|type\s+(?P<type>\w+)"
    r"|(?:const|static)\s+(?:mut\s+)?(?P<const>[A-Za-z_]\w*)\s*:"
    r"|macro_rules!\s*(?P<macro>\w+)"
    r")"
)
IMPL_HEAD_RE = re.compile(r"^\s*(?:unsafe\s+)?impl[\s<]")
# `` `path:NNN` (`fn_name`) `` -- the anchor immediately follows the citation,
# separated only by "(`...`)" with no other content, e.g.
# `` `mesh_search_paths.rs:139,140` (`non_package_uri_does_not_resolve`) ``.
# Kept as tight/adjacent as the backward-window rule in find_anchor, for the
# same reason: a loose forward scan would walk into the next citation's own
# anchor on a line that cites several functions in sequence.
FOLLOWING_ANCHOR_RE = re.compile(r"^\s*\(`((?:[A-Za-z_]\w*::)*[A-Za-z_]\w{3,})`\)")
# The mirror shape: the name immediately PRECEDES its citation, with nothing
# between but an optional space and the opening paren that wraps it --
# `` `construct_goal_pose_constraints`(`.../utils.rs:291`) ``,
# `` `DEFAULT_MAX_SAMPLING_ATTEMPTS` (`.../sampler.rs:71`, cited against ...) ``.
# `$`-anchored against the text before the citation, so the gap is the whole
# gap; a name three words back is prose ABOUT the citation, not a claim that
# the cited line is inside it -- the distinction `find_tight_anchors`'
# docstring draws, and the reason this cannot simply scan backwards.
PRECEDING_ANCHOR_RE = re.compile(r"`((?:[A-Za-z_]\w*::)*[A-Za-z_]\w{3,})` ?\(?$")


def citation_extent(line, match_end):
    """Where one citation's territory ends: past its closing backtick, and
    past the `` (`name`) `` pairing that belongs to it if it has one.

    A tight pairing is owned by exactly one citation -- the one it follows.
    The forward scan has said so since it was written (see the comment above
    FOLLOWING_ANCHOR_RE: it stays adjacent so it cannot walk into the NEXT
    citation's anchor); the backward window is the same rule from the other
    side, and used to floor at the previous citation's closing backtick,
    which is BEFORE that citation's pairing. So on

        `` `orientation.rs:187-190` (`invalid_parameterization_is_rejected`),
           `orientation.rs:217-219` (`degenerate_orientation_is_rejected`) ``

    the second citation swept up the first's anchor and carried both.
    Measured on the corpus at c178722: 5 citations carry a neighbour's
    anchor that way, all in
    `doc/assertion-discrimination-ledger-p9-ros.md:315,320,321`, and none
    changes class today because each also has its own anchor and lands
    inside it. It is a latent wrong answer in both directions -- a citation
    whose own anchor drifted stays "anchor-verified" on the neighbour's
    span, and a citation with NO anchor of its own is failed
    anchor-mismatch against a function it never named. The second is not
    hypothetical: it is what a 72-citation anchoring sample hit on
    `doc/assertion-discrimination-ledger-p3-acm.md:1011`, where an anchor
    added to `tree.rs:1944` landed inside `tree.rs:1972`'s backward window.
    """
    following = FOLLOWING_ANCHOR_RE.match(line[match_end:])
    return match_end + (following.end() if following is not None else 0)


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


def item_marker(masked, offset):
    """`("{", index)` or `(";", index)` for whichever comes first at bracket
    depth 0 at or after offset, `(None, None)` for neither.

    Which of the two an item ends at is a fact about the item.
    `const NAME: T = ...;`, `type Alias = ...;`, `mod name;` and
    `struct Unit;` are whole items that end at their `;`. Only `fn` reads a
    `;` as "this declaration has no body here, so it owns no lines" -- and
    it must: 64 of this tree's 5441 `fn`s are bodiless (trait method
    declarations like `fn extract_motion_plan_info(&self, ...) ->
    Result<()>;` and `extern` block declarations), and scanning past the `;`
    for the next `{` finds the NEXT item's brace and hands that item's whole
    extent to the bodiless name.
    `crates/moveit-planners-pilz/src/trajectory_generator.rs`'s declaration
    at 562 was given 562-636 that way -- swallowing `generate`, whose own
    span is 598-636 -- so `:606-636` would anchor-verify against
    `extract_motion_plan_info`, a function it is not in and that has no
    lines at all. All 64 invented a span; every one names a real function
    whose body is somewhere else entirely.

    The `;` terminates only at bracket depth 0: `-> [u8; 4]` is a return
    type, not a declaration end.
    """
    depth = 0
    for j in range(offset, len(masked)):
        c = masked[j]
        if c in "([":
            depth += 1
        elif c in ")]":
            depth -= 1
        elif depth == 0 and c in "{;":
            return c, j
    return None, None


def fn_spans(items):
    """{name: [(start_line, end_line, is_test), ...]} for every `fn NAME` in
    `item_spans`' output -- rules 1 and 2's view of a file, which is `fn`
    and nothing else. Multiple functions can share a name (nested
    `mod tests`, rare); every occurrence is kept and a citation is accepted
    if it falls in ANY of them.

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
    return {
        name: [(s, e, t) for (s, e, kind, t) in occ if kind == "fn"]
        for name, occ in items.items()
        if any(kind == "fn" for (_s, _e, kind, _t) in occ)
    }


def item_spans(path):
    """{name: [(start_line, end_line, kind, is_test), ...]} for every named
    Rust ITEM in path -- `fn`, `trait`, `struct`, `enum`, `union`, `mod`,
    `type`, `const`/`static`, `macro_rules!`, plus every `impl` block under
    the name of the type it is FOR.

    One parser, because a second one over the same source drifts from this
    one silently: `fn_spans` is this filtered to `fn`, not its own scan.
    The `fn` half is exactly what it was -- brace-matched on masked source,
    bodiless declarations skipped (see `item_marker`), span extended
    upward over the item's leading `#[...]` attributes and `///` lines.

    Extending the span over the doc comment is not a nicety for `fn` and a
    liberty for the rest: `doc/claim-audit/*` cites a CLAIM, and a claim
    about upstream behaviour is written in the doc comment of the item that
    pins it. Judged against the declaration line alone, 30 citations in this
    corpus that point at exactly the right doc comment read as drift.

    An `impl` block is registered under its self type (`impl Foo`,
    `impl<T> Trait for Foo<T>` -- both `Foo`) because that is what a
    citation naming a type and pointing into one of its methods means. The
    trait side of `impl Trait for Foo` is deliberately NOT registered: the
    block is Foo's code, not the trait's definition, and the trait has its
    own `trait` span elsewhere.

    `;`-terminated items own their declaration: `const NAME: T = ...;`,
    `type Alias = ...;`, `mod name;`, `struct Unit;` span from their first
    attribute/doc line to the `;`. Only `fn` reads a `;` as "no body here"
    and drops out.
    """
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return {}
    masked = mask_non_code(text)
    lines = text.split("\n")
    line_offsets = []
    acc = 0
    for l in lines:
        line_offsets.append(acc)
        acc += len(l) + 1
    spans = {}
    for line_no, line in enumerate(lines, 1):
        m = ITEM_DEF_RE.match(line)
        if m is not None:
            kind = m.lastgroup
            name = m.group(kind)
        elif IMPL_HEAD_RE.match(line):
            kind, name = "impl", None
        else:
            continue
        # Find this item's `{` or `;` starting from its own line (headers
        # can wrap onto later lines before either appears).
        offset = line_offsets[line_no - 1]
        marker, at = item_marker(masked, offset)
        if marker is None or (marker == ";" and kind == "fn"):
            continue
        if kind == "impl":
            if marker != "{":
                continue
            name = impl_self_type(masked[offset:at])
            if name is None:
                continue
        if marker == ";":
            end_line = masked.count("\n", 0, at) + 1
        else:
            depth = 0
            j = at
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
        spans.setdefault(name, []).append((start_line, end_line, kind, is_test))
    return spans


FOR_KW_RE = re.compile(r"\bfor\b")
WHERE_KW_RE = re.compile(r"\bwhere\b")
TYPE_HEAD_RE = re.compile(r"([A-Za-z_]\w*)\s*$")


def impl_self_type(header):
    """The bare type name an `impl` header is FOR, or None.

    `impl Foo`, `impl<T> Foo<T>`, `impl Trait for Foo`, `impl<'a> Trait for
    &'a mut Foo<T>`, `impl fmt::Display for Foo` all give `Foo`. The trait
    side is dropped on purpose (see `item_spans`), as are generic
    arguments, references, lifetimes and the module path -- a citation
    names the type, not its instantiation.
    """
    header = header.strip()
    if not header.startswith("impl"):
        return None
    rest = header[4:]
    # `impl<...>`'s own generic parameters, balanced -- `impl<T: Into<U>>`.
    rest = rest.lstrip()
    if rest.startswith("<"):
        depth, i = 0, 0
        for i, ch in enumerate(rest):
            if ch == "<":
                depth += 1
            elif ch == ">":
                depth -= 1
                if depth == 0:
                    break
        rest = rest[i + 1 :]
    # `for` and `where` only bind at angle depth 0: `impl Foo<Bar for Baz>`
    # is not a thing, but `impl Trait<A> for B where A: C` is, and cutting
    # at the first textual `where` inside a generic argument would truncate
    # the type this is about.
    depth = 0
    cut_for = cut_where = None
    for i, ch in enumerate(rest):
        if ch == "<":
            depth += 1
        elif ch == ">":
            depth -= 1
        elif depth == 0:
            if cut_for is None and FOR_KW_RE.match(rest, i):
                cut_for = i
            elif cut_where is None and WHERE_KW_RE.match(rest, i):
                cut_where = i
                break
    if cut_for is not None:
        rest = rest[cut_for + 3 : cut_where] if cut_where else rest[cut_for + 3 :]
    elif cut_where is not None:
        rest = rest[:cut_where]
    # Trim generic arguments and any trailing `{`, then take the last path
    # segment: `fmt::Display`'s `Display`, `crate::robot_model::RobotModel`'s
    # `RobotModel`.
    depth = 0
    head = []
    for ch in rest:
        if ch == "<":
            depth += 1
        elif ch == ">":
            depth -= 1
        elif depth == 0:
            head.append(ch)
    text = "".join(head).replace("&", " ").replace("(", " ").replace("{", " ")
    text = re.sub(r"\bmut\b|\bdyn\b|'\w+", " ", text).strip()
    if not text:
        return None
    last = text.split()[-1].rstrip(":").split("::")[-1]
    m = TYPE_HEAD_RE.search(last)
    return m.group(1) if m is not None else None


def find_item_anchor(line, match_start, match_end):
    """Rule 0: the name DIRECTLY adjacent to this citation, whichever side
    it is on, or None. The trailing `` (`name`) `` wins over a leading one
    when both are present.

    That precedence is not a tie-break, it is the corpus's own disambiguator.
    `ros/moveit-ros/doc/message-mapping.md:418` reads

        `OcTree::read_binary_data`/`read_data`
        (`crates/moveit-octomap/src/tree.rs:1244-1246` (`read_binary_data`)/`:1272`)

    -- two names before the citation and the referent parenthesised after
    it. A scan that takes the nearest preceding name answers `read_data`
    and fails a citation that is exactly right; the parenthesised name is
    the one the author wrote to say WHICH of the two this citation is for.

    A `::`-qualified name reduces to its last segment: `Self::read_data`,
    `JointConstraint::merged` and `fmt::Display` name items this script
    resolves within one file, and the qualifier is the reader's context,
    not part of the definition's name.

    Unlike rule 1 this asks nothing about `#[test]`. Rule 1 needs that gate
    because it also scans a 60-character backward window, where a name can
    be there for any reason; adjacency is itself the claim, and demanding a
    `#[test]` attribute of a `trait`, a `const` or a production `fn` named
    right against its own citation rejects the shape this rule exists for.
    """
    following = FOLLOWING_ANCHOR_RE.match(line[match_end:])
    if following is not None:
        return following.group(1).split("::")[-1]
    preceding = PRECEDING_ANCHOR_RE.search(line[:match_start])
    if preceding is not None:
        return preceding.group(1).split("::")[-1]
    return None


def anchor_zones(items, name):
    """Every line range in one file where `name` is defined: each of its
    item spans, declaration, doc comment and attributes included.

    An empty list means the name is not an item of this file, and the caller
    must leave the citation to the later rules rather than fail it --
    `` `msg` ``, a struct field, a `.msg` wire field or a C++ symbol can all
    sit adjacent to a citation, and none of them is a claim this rule can
    adjudicate. That is not a technicality: `ros/moveit-ros/src/
    planning.rs`'s crate doc names `reference_trajectories`,
    `attached_collision_objects` and `multi_dof_joint_state`, three wire
    fields that are Rust items nowhere, and an earlier draft of this rule
    that armed on a `//!` mention failed all three of
    `doc/assertion-discrimination-ledger-p9-ros.md:326`'s correct citations
    against "spans" that were the paragraph discussing them.

    `//!` paragraphs were then tried as an EXTRA landing zone for names that
    ARE items -- `doc/claim-audit/moveit-trajectory.md:76` cites
    `crates/moveit-trajectory/src/lib.rs:31-34`, the crate doc's entry for
    the `time_optimal_trajectory_generation` module, whose `pub mod` is at
    `:499` -- and measured at zero: with the mechanism and without it, this
    corpus fails the same six citations. Both crate-doc citations pass
    because they QUOTE the line they cite, which the caller checks anyway.
    Dropped rather than kept for the shape's sake: a 30-line paragraph is a
    wide zone to hand a symbol on the strength of one mention, and a
    widener that changes no verdict is a place for the next drift to land
    unnoticed.
    """
    return [(s, e) for (s, e, _kind, _t) in items.get(name, ())]


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


def find_tight_anchors(line, match_start, match_end, spans, prev_citation_extent=0, is_range=False):
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
       `prev_citation_extent` -- the end of the previous citation TOGETHER
       WITH its own pairing, see `citation_extent`).

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

    window_start = max(prev_citation_extent, match_start - LOCAL_WINDOW)
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


# ---------------------------------------------------------------------------
# SECOND POPULATION: in-repo citations that are not `.md` -> `.rs`.
#
# The corpus above has two independent holes, and closing only one leaves the
# worst sites invisible because they are exactly the ones that fall in the
# other:
#
#   1. TARGET extension. CITATION_RE hard-codes `\.rs`, so a citation naming
#      an in-repo `.md`/`.sh`/`.py`/`.toml`/`.urdf` target is not parsed as a
#      citation at all -- never resolved, never bounds-checked, never counted.
#   2. CITER extension. main() builds md_files from `.md` only, so a citation
#      living in a `.json`, `.sh`, `.py` or `.rs` file is outside the corpus
#      no matter what it names. This gate's OWN source cites
#      `PORTING-PLAN.md:9384` and could not see it.
#
# WHY THIS IS A SEPARATE POPULATION WITH A SEPARATE BASELINE. Folding these
# into CLASS_BASELINE would add ~280 rows that were never checked before to a
# file whose whole purpose is to make a DELTA meaningful. A reader could not
# tell "this citation is newly visible" from "this citation newly broke", and
# a re-freeze would absorb both into one passing count -- which is the exact
# laundering CLASS_BASELINE exists to prevent. So the two populations are
# declared, counted and reported independently. The set difference IS the
# separation: everything here is by construction absent from CLASS_BASELINE.
#
# WHY NO CLASS LADDER HERE. Rust-fn anchoring does not apply to a `.md` or
# `.sh` target, so there is no "inside the function the text names" to check.
# What replaces it is SECTION CONTAINMENT: when the citing text names a
# section (`§129.3`) tightly adjacent to the citation, the cited line must sit
# inside that section's own span in the target. That is the rule that catches
# a shift onto a live line -- the failure mode the blank predicate cannot see,
# measured in PORTING-PLAN.md §299.7.
# A real fence toggle is 3+ backticks and nothing else with a backtick on the
# line. Copied from check-shorthand-citations.py, which had to solve the same
# case: these documents write ```` ```text ```` as an INLINE span while
# discussing fences, and a naive startswith("```") reads those as toggles and
# desyncs for the rest of the file. The `.rs` population above does not skip
# fences at all; this one does, because a citation shown inside a fence is
# displayed sample text rather than a claim about the tree.
FENCE_RE = re.compile(r"^ {0,3}`{3,}[^`]*$")
IN_REPO_BASELINE = "doc/citation-classes-in-repo.txt"
# Every tracked text extension that is not `.rs`. `.rs` targets belong to the
# population above; listing them here would double-count them.
IN_REPO_TARGET_EXT = ("md", "sh", "py", "yml", "yaml", "toml", "urdf", "srdf", "txt", "json", "xacro")
IN_REPO_CITATION_RE = re.compile(
    r"`((?:[\w./-]+/)?[\w.-]+\.(?:" + "|".join(IN_REPO_TARGET_EXT) + r")):"
    r"(\d+(?:-\d+)?(?:,\d+(?:-\d+)?)*)`"
)
# Copied verbatim from tools/ci/check-section-references.sh's HEADING_RE so the
# two gates cannot disagree about what a section heading is.
IN_REPO_SECTION_RE = re.compile(r"^#{2,6}\s+§?(\d+(?:\.\d+)*)\b")
# Tight adjacency, same discipline as FOLLOWING_ANCHOR_RE/PRECEDING_ANCHOR_RE
# above and for the same reason: a loose scan finds a section number three
# clauses back that is prose ABOUT the citation, not a claim that the cited
# line is inside it. A wide scan was implemented and measured before being
# discarded -- it produced 20 attributions and got the two census sites the
# tight rule gets right WRONG, and attributed a textbook's own "4.4.1" and
# another document's local "17.5" as if they named sections of this plan.
# Both are spelled without the section sigil here for the same reason
# PORTING-PLAN.md §299.3 respells a wrong citation: quoted as data, they
# would otherwise read as live references to sections that do not exist.
IN_REPO_SECTION_BEFORE_RE = re.compile(r"§(\d+(?:\.\d+)*)[^`§]{0,40}$")
IN_REPO_SECTION_AFTER_RE = re.compile(r"^[ ,)]{0,3}§(\d+(?:\.\d+)*)")
IN_REPO_RESOLVED = "resolved"
IN_REPO_SECTION_VERIFIED = "section-verified"
IN_REPO_UNRESOLVED = "unresolvable"
# A path that matches NO tracked file names something this repository does not
# contain -- an upstream MoveIt source, an oracle-container path, an elided
# `.../foo.md`. Those belong to measure-upstream-citations.py, whose domain is
# exactly the files this one cannot open, so calling them a failure here would
# report a defect against the wrong gate. Counted and declared rather than
# dropped, so this population stays a total enumeration of its corpus rather
# than of the part this gate can adjudicate. `unresolvable` is kept for the
# case that IS this gate's: a path that matches several tracked files.
IN_REPO_EXTERNAL = "external"
IN_REPO_OOB = "out-of-bounds"
IN_REPO_BLANK = "blank-line"
IN_REPO_SECTION_MISMATCH = "section-mismatch"
IN_REPO_FAILING = (IN_REPO_UNRESOLVED, IN_REPO_OOB, IN_REPO_BLANK, IN_REPO_SECTION_MISMATCH)
# Flipped to True once the population's findings are triaged. Until then the
# corpus is declared, counted and delta-checked -- a NEW failure still fails,
# because it arrives as an undeclared row -- but the backlog it arrives with
# does not fail the run. See PORTING-PLAN.md §299.9.
IN_REPO_HARD_FAIL = False


def in_repo_section_spans(lines):
    """{section number: (first line, last line)} for a document's own headings.

    A section runs to the line before the next heading at ANY depth, so
    `§299` ends where `§299.1` begins. Containment is therefore checked
    against the most specific section that holds the line, which is what the
    citing text means when it writes `§299.1`.
    """
    heads = []
    for i, line in enumerate(lines, 1):
        m = IN_REPO_SECTION_RE.match(line)
        if m:
            heads.append((m.group(1), i))
    spans = {}
    for idx, (num, start) in enumerate(heads):
        end = heads[idx + 1][1] - 1 if idx + 1 < len(heads) else len(lines)
        # A number can appear twice (an index line and the section itself);
        # keep the widest span so containment does not fail on the duplicate.
        if num in spans:
            spans[num] = (min(spans[num][0], start), max(spans[num][1], end))
        else:
            spans[num] = (start, end)
    return spans


def in_repo_section_claim(line, start, end):
    """The section number the citing text attaches to THIS citation, or None.

    `start`/`end` are the citation match's own bounds on the line, so the
    windows are the text immediately before and immediately after it.
    """
    before = IN_REPO_SECTION_BEFORE_RE.search(line[:start])
    if before:
        return before.group(1)
    after = IN_REPO_SECTION_AFTER_RE.match(line[end:])
    return after.group(1) if after else None


def scan_in_repo(tracked):
    """Every in-repo citation whose target is not `.rs`, from EVERY citer."""
    tset = set(tracked)
    by_base = {}
    for p in tracked:
        by_base.setdefault(p.rsplit("/", 1)[-1], []).append(p)

    body_cache = {}

    def body(path):
        if path not in body_cache:
            body_cache[path] = (
                (REPO_ROOT / path).read_text(encoding="utf-8", errors="replace").split("\n")
            )
        return body_cache[path]

    span_cache = {}

    def spans(path):
        if path not in span_cache:
            span_cache[path] = in_repo_section_spans(body(path)) if path.endswith(".md") else {}
        return span_cache[path]

    out = []
    for citer in tracked:
        if citer.rsplit(".", 1)[-1] not in IN_REPO_TARGET_EXT + ("rs",):
            continue
        try:
            text = (REPO_ROOT / citer).read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        lines = text.split("\n")
        in_fence = False
        for line_no, line in enumerate(lines, 1):
            # Only `.md` uses ``` fences; in a `.py` or `.sh` citer the same
            # bytes are ordinary content and toggling on them desyncs the
            # rest of the file.
            if citer.endswith(".md"):
                if FENCE_RE.match(line):
                    in_fence = not in_fence
                    continue
                if in_fence:
                    continue
            for m in IN_REPO_CITATION_RE.finditer(line):
                name, spec = m.group(1), m.group(2)
                cited = []
                for tok in spec.split(","):
                    cited.extend(int(x) for x in tok.split("-"))
                full = f"{name}:{spec}"
                if name in tset:
                    target = name
                else:
                    cands = [p for p in by_base.get(name.rsplit("/", 1)[-1], []) if path_matches(p, name)]
                    if not cands:
                        out.append((citer, line_no, full, None, IN_REPO_EXTERNAL, None))
                        continue
                    if len(cands) > 1:
                        out.append((citer, line_no, full, None, IN_REPO_UNRESOLVED, cands))
                        continue
                    target = cands[0]
                tl = body(target)
                if any(not (1 <= n <= len(tl)) for n in cited):
                    out.append((citer, line_no, full, target, IN_REPO_OOB, len(tl)))
                    continue
                # Same predicate, same reason, same range semantics as the
                # `.rs` population: only NAMED lines, never a range interior.
                blank = [n for n in cited if not tl[n - 1].strip()]
                if blank:
                    out.append((citer, line_no, full, target, IN_REPO_BLANK, blank))
                    continue
                claim = in_repo_section_claim(line, m.start(), m.end())
                sp = spans(target)
                if claim and claim in sp:
                    lo, hi = sp[claim]
                    outside = [n for n in cited if not (lo <= n <= hi)]
                    if outside:
                        out.append(
                            (citer, line_no, full, target, IN_REPO_SECTION_MISMATCH, (claim, lo, hi, outside))
                        )
                    else:
                        out.append((citer, line_no, full, target, IN_REPO_SECTION_VERIFIED, claim))
                    continue
                out.append((citer, line_no, full, target, IN_REPO_RESOLVED, None))
    return out


def render_in_repo(rows):
    counts = {}
    for citer, _, spec, _, verdict, _ in rows:
        counts.setdefault((citer, spec, verdict), 0)
        counts[(citer, spec, verdict)] += 1
    return [
        f"{citer}\t{spec}\t{verdict}" + (f"*{n}" if n > 1 else "")
        for (citer, spec, verdict), n in sorted(counts.items())
    ]


def report_in_repo(tracked):
    """Report the second population against its own baseline. Returns True to
    fail the run.

    The two populations are reported independently and never summed. A reader
    has to be able to see that these citations were never checked before --
    a single blended total would make a first-ever finding indistinguishable
    from a regression, which is what having two files prevents.
    """
    rows = scan_in_repo(tracked)
    path = REPO_ROOT / IN_REPO_BASELINE
    if not path.exists():
        print(
            f"FAIL {IN_REPO_BASELINE} is missing -- regenerate with "
            f"tools/ci/check-citation-drift.py --write-classes",
            file=sys.stderr,
        )
        return True

    declared = {}
    for line in path.read_text(encoding="utf-8").split("\n"):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        citer, spec, verdict = line.split("\t")
        n = 1
        if "*" in verdict:
            verdict, _, mult = verdict.partition("*")
            n = int(mult)
        declared[(citer, spec, verdict)] = n

    live = {}
    for r in rows:
        live[(r[0], r[2], r[4])] = live.get((r[0], r[2], r[4]), 0) + 1

    counts = {}
    for r in rows:
        counts[r[4]] = counts.get(r[4], 0) + 1
    summary = ", ".join(f"{counts[k]} {k}" for k in sorted(counts))
    findings = sum(counts.get(k, 0) for k in IN_REPO_FAILING)

    undeclared = sorted(k for k in live if k not in declared)
    retired = sorted(k for k in declared if k not in live)
    recounted = sorted(k for k in live if k in declared and live[k] != declared[k])

    failed = False
    for citer, spec, verdict in undeclared:
        failed = True
        print(
            f"FAIL {citer}: `{spec}` ({verdict}) is not in {IN_REPO_BASELINE} -- a citation "
            f"whose line number changed retires its old key and arrives as a new one",
            file=sys.stderr,
        )
    for citer, spec, verdict in retired:
        failed = True
        print(
            f"FAIL {citer}: `{spec}` ({verdict}) is in {IN_REPO_BASELINE} but no longer in "
            f"the tree",
            file=sys.stderr,
        )
    for citer, spec, verdict in recounted:
        failed = True
        print(
            f"FAIL {citer}: `{spec}` ({verdict}) occurred {declared[(citer, spec, verdict)]}x, "
            f"now {live[(citer, spec, verdict)]}x",
            file=sys.stderr,
        )

    stream = sys.stderr if (failed or IN_REPO_HARD_FAIL) else sys.stdout
    print(
        f"--- second population: {len(rows)} in-repo non-`.rs` citations across "
        f"{len({r[0] for r in rows})} citing files: {summary} ---",
        file=stream,
    )
    if findings:
        for r in rows:
            if r[4] in IN_REPO_FAILING:
                print(f"    {r[4]:17s} {r[0]}:{r[1]}  `{r[2]}`", file=stream)
        print(
            f"--- {findings} of them are findings. These were in no gate's corpus before "
            f"this population was declared, so they are first-ever results, not "
            f"regressions; IN_REPO_HARD_FAIL flips them to failing once triaged. ---",
            file=stream,
        )
    return failed or (IN_REPO_HARD_FAIL and bool(findings))


def main(write_classes=False):
    tracked = tracked_files()
    md_files = [p for p in tracked if p.endswith(".md")]
    rs_files_set = {p for p in tracked if p.endswith(".rs")}
    rs_files_by_basename = {}
    for p in rs_files_set:
        rs_files_by_basename.setdefault(p.rsplit("/", 1)[-1], []).append(p)

    span_cache = {}
    item_cache = {}

    def items_for(path):
        if path not in item_cache:
            item_cache[path] = item_spans(REPO_ROOT / path)
        return item_cache[path]

    def spans_for(path):
        if path not in span_cache:
            span_cache[path] = fn_spans(items_for(path))
        return span_cache[path]

    # ONE record per citation, holding the one class it was assigned. Every
    # count and every failure listing below is derived from this list, and
    # nothing increments a counter directly.
    #
    # It used to be the other way round: eight scattered `+= 1`/`.append`
    # sites, and a citation's class existed only as "which counter went up".
    # That is why a drift could be absorbed rather than reported -- shifting
    # `velocity_profile.rs:423` to `:823` moved one citation from
    # content-verified to unanchored, both passing, and the run's only
    # visible change was two totals moving by one. With the class a value,
    # it can be written down per citation and compared against a committed
    # baseline (CLASS_BASELINE), which is what turns that demotion into a
    # failure. See CLASS_RANK.
    records = []

    def record(md, line_no, spec, cls, detail=None, partly=False):
        records.append(Citation(md, line_no, spec, cls, detail, partly))

    # Rule 2's own parse result, at each of the three stages it can go quiet
    # at: which declared header spellings matched a table, how many rows
    # those tables held, and how many citations the rule actually anchored.
    # None of the three is visible in any verdict -- see the guard below.
    ledger_tables = {}
    ledger_rows_total = 0
    rule2_anchored = 0
    rule0_anchored = 0

    for md in md_files:
        text = (REPO_ROOT / md).read_text(encoding="utf-8", errors="replace")
        lines = text.split("\n")
        ledger_rows, tables_here = ledger_row_lines(lines)
        for header, n in tables_here.items():
            ledger_tables[header] = ledger_tables.get(header, 0) + n
        ledger_rows_total += len(ledger_rows)
        for line_no, line in enumerate(lines, 1):
            prev_citation_extent = 0
            for m in CITATION_RE.finditer(line):
                fname_part = m.group(1)
                cited_lines = parse_cited_lines(m)
                spec = m.group(0).strip("`")
                # Captured before updating prev_citation_extent for the NEXT
                # iteration, so this citation's own anchor search still
                # sees where the PREVIOUS citation ended, regardless of
                # which `continue` below this one takes.
                window_floor, prev_citation_extent = (
                    prev_citation_extent,
                    citation_extent(line, m.end()),
                )

                if EXTERNAL_PATH_RE.match(fname_part):
                    record(md, line_no, spec, EXEMPT, fname_part)
                    continue

                candidates = resolve_path(fname_part, rs_files_by_basename, rs_files_set)
                if len(candidates) != 1:
                    record(
                        md,
                        line_no,
                        spec,
                        UNRESOLVED,
                        (
                            fname_part,
                            "no tracked .rs file matches"
                            if not candidates
                            else f"ambiguous: matches {len(candidates)} tracked files: {candidates}",
                        ),
                    )
                    continue
                resolved_path = candidates[0]
                body_lines = (
                    (REPO_ROOT / resolved_path)
                    .read_text(encoding="utf-8", errors="replace")
                    .split("\n")
                )
                file_len = len(body_lines)
                oob = [ln for ln in cited_lines if not (1 <= ln <= file_len)]
                if oob:
                    record(
                        md,
                        line_no,
                        spec,
                        OUT_OF_BOUNDS,
                        (fname_part, cited_lines, resolved_path, file_len),
                    )
                    continue

                # ABOVE THE LADDER, deliberately. A blank line cannot be any
                # citation's subject, so this is not a weaker class of check
                # that a stronger one can outrank -- it disqualifies the
                # citation whatever the anchoring says.
                #
                # Putting it ON the ladder was the bug. Blank-target citations
                # were landing in all three PASSING classes (7 unanchored, 3
                # content-verified, 1 anchor-verified), so no class rejected
                # them and the top class actively certified one:
                # `doc/claim-audit/moveit-scene.md:35` cites
                # `crates/moveit-scene/src/scene.rs:2585`, a blank line inside
                # `decouple_parent_then_mutating_the_former_parent_is_not_observed`.
                # "Inside the function the citing text names" was satisfied,
                # so ANCHOR_VERIFIED -- the strongest thing this gate can say
                # -- certified a citation pointing at nothing. The assertion it
                # means is one line up, at `:2584`.
                #
                # That is the second time the top class has vouched for a
                # citation carrying none of its claim; the previous round found
                # seven that had slid onto a comment or onto a `#[test]`
                # attribute line. Both say the same thing about this ladder:
                # every class on it answers "is the cited line plausibly
                # nearby", and none answers "does the cited line carry the
                # claim". This predicate is the cheapest available step toward
                # the second question -- it cannot confirm a claim, but it can
                # reject a line that carries nothing at all.
                #
                # RANGE SEMANTICS. The test is applied to the line numbers the
                # citation NAMES -- both ends of `NNN-MMM` and every member of
                # a comma list -- not to the interior of a span.
                # `parse_cited_lines` already returns exactly that set, and
                # those are the numbers a reader checks. So `:2584-2590` must
                # have content at 2584 and 2590; it says nothing about 2587.
                # The alternative considered was "fail only when every named
                # line is blank", which would have passed all six of the
                # ranges that start one line early and run into the doc comment
                # they cite -- an off-by-one is exactly what this should catch,
                # and it is one keystroke to fix once named.
                blank = [ln for ln in cited_lines if not body_lines[ln - 1].strip()]
                if blank:
                    record(
                        md,
                        line_no,
                        spec,
                        BLANK_LINE,
                        (fname_part, cited_lines, resolved_path, blank),
                    )
                    continue

                spans = spans_for(resolved_path)
                body = body_lines
                # Rule 4's verdict, computed once here so that every path
                # which used to fall through to bounds-only reaches it: a
                # citation is never filed as unverified while it is in fact
                # quoting the line it names.
                quoted = content_anchor(
                    quotations_near(citing_context(lines, line_no - 1)), body, cited_regions(m)
                )

                # Rule 0, before every other disposition including the
                # file-header early-out below: a name written DIRECTLY
                # against its citation is a claim about where that name
                # lives, and this rule either confirms it or fails it. It
                # never leaves one unanchored, which is what let
                # `PORTING-PLAN.md:7801` -- `` `construct_goal_pose_
                # constraints`(`crates/moveit-constraints/src/utils.rs:291`) ``,
                # naming the symbol in backticks touching its own citation,
                # 71 lines above that function's real span -- pass every
                # gate for as long as it has. `unanchored` bounds-checks and
                # asks nothing else; it is the right disposition for a
                # citation carrying no claim, and the wrong one for a
                # citation carrying this one.
                #
                # It must precede the `first_fn` early-out because two of
                # the shapes it adjudicates land there by construction: a
                # `//!` paragraph is the enclosing module's doc, so it is
                # above every item in the file.
                item_name = find_item_anchor(line, m.start(), m.end())
                zones = (
                    anchor_zones(items_for(resolved_path), item_name)
                    if item_name
                    else []
                )
                if zones:
                    rule0_anchored += 1
                    landed = [ln for ln in cited_lines if any(s <= ln <= e for (s, e) in zones)]
                    # The same parenthetical carries a SECOND, weaker claim in
                    # `doc/claim-audit/*`, and it is a quotation, not an
                    # anchor: that table's column 1 parenthesises a fragment
                    # of the FIRST cited line. Two rows either side of the one
                    # that made this visible settle what the convention is --
                    # `moveit-trajectory.md:164` cites `:332-338` as
                    # `` (`use std::collections::HashMap;`) `` and `:165`
                    # cites `:341-342` as
                    # `` (`use crate::trajectory::Trajectory;`) ``, and 332
                    # and 341 are exactly those lines. `:163`'s
                    # `` (`totg_compute_time_stamps`) `` on `:308-319` is the
                    # same convention with an identifier-shaped fragment: 308
                    # reads "(including `totg_compute_time_stamps`'s
                    # internally-recomputed", a crate-doc line, while the
                    # function itself is at 623.
                    #
                    # So the two readings are confirmed independently and
                    # either one satisfies the citation. Requiring
                    # containment alone fails a row that is quoting exactly
                    # the line it cites; dropping to the quotation alone
                    # would let `PORTING-PLAN.md:7801` pass on nothing, since
                    # `utils.rs:291` does not say `construct_goal_pose_
                    # constraints` either.
                    named_here = any(
                        re.search(r"\b" + re.escape(item_name) + r"\b", body[ln - 1])
                        for ln in cited_lines
                        if 1 <= ln <= len(body)
                    )
                    if len(landed) == len(cited_lines) or named_here:
                        record(md, line_no, spec, ANCHOR_VERIFIED)
                    else:
                        record(
                            md,
                            line_no,
                            spec,
                            ANCHOR_MISMATCH,
                            (
                                fname_part,
                                cited_lines,
                                landed,
                                [item_name],
                                resolved_path,
                                {item_name: [(s, e, None) for (s, e) in zones]},
                            ),
                        )
                    continue

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
                    record(md, line_no, spec, CONTENT_VERIFIED if quoted else UNANCHORED)
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
                    record(md, line_no, spec, CONTENT_VERIFIED if quoted else UNANCHORED)
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
                    record(md, line_no, spec, ANCHOR_VERIFIED)
                elif landed and m.group(4) is not None:
                    # Falls through to rule 4 rather than passing or failing:
                    # partial containment carries no drift signal either way,
                    # and both of today's two are quoted, so rule 4 has real
                    # per-line evidence to offer them that this bucket would
                    # throw away. Counted here so the shape is visible on the
                    # OK line instead of hiding inside another bucket's total.
                    record(
                        md,
                        line_no,
                        spec,
                        CONTENT_VERIFIED if quoted else UNANCHORED,
                        partly=True,
                    )
                else:
                    record(
                        md,
                        line_no,
                        spec,
                        ANCHOR_MISMATCH,
                        (fname_part, cited_lines, landed, anchors, resolved_path, spans),
                    )

    # Everything below is DERIVED from `records`. Two sources of truth for
    # one citation's class is the defect this replaced.
    total = len(records)
    by_class = {}
    for r in records:
        by_class.setdefault(r.cls, []).append(r)
    n = lambda cls: len(by_class.get(cls, ()))  # noqa: E731
    external = [(r.md, r.line_no, r.detail) for r in by_class.get(EXEMPT, ())]
    unresolved = [(r.md, r.line_no, *r.detail) for r in by_class.get(UNRESOLVED, ())]
    out_of_bounds = [(r.md, r.line_no, *r.detail) for r in by_class.get(OUT_OF_BOUNDS, ())]
    anchor_mismatch = [(r.md, r.line_no, *r.detail) for r in by_class.get(ANCHOR_MISMATCH, ())]
    unanchored_by_file = {}
    for r in by_class.get(UNANCHORED, ()):
        unanchored_by_file[r.md] = unanchored_by_file.get(r.md, 0) + 1
    anchor_verified = n(ANCHOR_VERIFIED)
    content_verified = n(CONTENT_VERIFIED)
    partly_anchored = sum(1 for r in records if r.partly)

    blank_line = [(r.md, r.line_no, *r.detail) for r in by_class.get(BLANK_LINE, ())]

    hard_fail = (
        bool(out_of_bounds) or bool(anchor_mismatch) or bool(unresolved) or bool(blank_line)
    )
    counts = (
        f"{anchor_verified} anchor-verified (EVERY cited line inside a named function's body), "
        f"{content_verified} content-verified (the citing text's own quotation of the code is "
        f"at the cited line), "
        f"{sum(unanchored_by_file.values())} unanchored (bounds-checked only -- the citing text "
        f"neither names a containing function nor quotes the line), "
        f"{len(external)} exempt (names a dependency or a build artifact, see above), "
        f"{len(out_of_bounds)} out-of-bounds, {len(anchor_mismatch)} anchor-mismatch, "
        f"{len(unresolved)} unresolvable, {len(blank_line)} blank-line"
        f"; {partly_anchored} of those are partly-anchored enumerations (a comma list whose "
        f"named functions hold some of its cited lines and not others), counted in whichever "
        f"bucket rule 4 put them"
        f"; rule 0 (a Rust item named directly against its citation, resolved against that "
        f"item's own span in the source) adjudicated "
        f"{rule0_anchored} citation(s)"
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

    # Rule 0 goes quiet the same way and is invisible the same way: its
    # citations fall back into content-verified or unanchored, both passing.
    # Its one number is floored for that reason -- an adjacency regex that
    # stops matching, or an item scan that stops registering `trait`, takes
    # this to zero while every other total stays healthy.
    if rule0_anchored == 0:
        print(
            "FAIL rule 0 adjudicated no citation at all -- either the adjacency regexes in "
            "`find_item_anchor` no longer match this corpus's `` `name`(`path.rs:N`) `` / "
            "`` `path.rs:N` (`name`) `` shapes, or `item_spans` is registering no items",
            file=sys.stderr,
        )
        return 1

    live = class_map(records)
    if write_classes:
        head_sha = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, capture_output=True, text=True
        ).stdout.strip()
        header = [
            "# Per-citation verification class for every `path.rs:NNN` citation in every",
            "# tracked .md file. The class says WHAT WAS CHECKED, and the classes are",
            "# ranked: anchor-verified (the cited line is inside the function the citing",
            "# text names) > content-verified (the citing text's quotation of the code is",
            "# at the cited line) > unanchored (only that the line number is inside the",
            "# file). Drift moves a citation DOWN that ladder, and without this file a",
            "# demotion is invisible -- two totals move by one and the run still passes.",
            "#",
            f"# Generated by: tools/ci/check-citation-drift.py --write-classes",
            f"# Source commit: {head_sha}",
            f"# Citations: {total} in {len(live)} distinct (document, citation) keys",
        ]
        for cls in (ANCHOR_VERIFIED, CONTENT_VERIFIED, UNANCHORED, EXEMPT, *HARD_FAIL_CLASSES):
            header.append(f"#   {cls}: {n(cls)}")
        header += [
            "# A citation appearing here with a hard-failing class (out-of-bounds,",
            "# anchor-mismatch, unresolvable) is NOT excused by being recorded -- those",
            "# fail the run on their own, above. It is written down so this file stays a",
            "# total enumeration of the corpus rather than of its passing part.",
            "# Format: <citing document>\\t<citation>\\t<class>[*N] ...",
        ]
        (REPO_ROOT / CLASS_BASELINE).write_text("\n".join(header + render_classes(live)) + "\n")
        print(f"wrote {CLASS_BASELINE}: {total} citations, {len(live)} keys")

        rows = scan_in_repo(tracked)
        by_verdict = {}
        for r in rows:
            by_verdict[r[4]] = by_verdict.get(r[4], 0) + 1
        head = [
            "# The SECOND citation population: every in-repo citation whose target is",
            "# not `.rs`, from EVERY tracked citer -- not just `.md` ones. It is a",
            "# separate file from doc/citation-classes.txt on purpose: these citations",
            "# were in no gate's corpus until they were declared here, so folding them",
            "# into that file would mix `newly visible` with `newly broken` in one",
            "# count. The set difference between the two files IS that separation.",
            "#",
            "# Verdicts. resolved = the target exists here and every named line is in",
            "# bounds and non-blank. section-verified = the citing text names a section",
            "# tightly against the citation and every named line is inside that",
            "# section's span. external = the path names a file this repository does",
            "# not contain, which is measure-upstream-citations.py's domain, not this",
            "# one's. The rest are findings: blank-line, out-of-bounds,",
            "# section-mismatch, and unresolvable (a path matching several tracked",
            "# files, which names none of them).",
            "#",
            "# Generated by: tools/ci/check-citation-drift.py --write-classes",
            f"# Source commit: {head_sha}",
            f"# Citations: {len(rows)} across {len({r[0] for r in rows})} citing files",
        ]
        for k in sorted(by_verdict):
            head.append(f"#   {k}: {by_verdict[k]}")
        head.append("# Format: <citing file>\\t<citation>\\t<verdict>[*N]")
        (REPO_ROOT / IN_REPO_BASELINE).write_text("\n".join(head + render_in_repo(rows)) + "\n")
        print(f"wrote {IN_REPO_BASELINE}: {len(rows)} citations")
        return 0

    baseline_path = REPO_ROOT / CLASS_BASELINE
    if not baseline_path.exists():
        print(
            f"FAIL {CLASS_BASELINE} is missing -- every citation's class is unchecked "
            f"without it; regenerate with tools/ci/check-citation-drift.py --write-classes",
            file=sys.stderr,
        )
        return 1
    baseline, malformed = parse_classes(baseline_path.read_text(encoding="utf-8"))
    if malformed or not baseline:
        print(
            f"FAIL {CLASS_BASELINE} parsed {len(baseline)} rows and {malformed} malformed "
            f"-- a baseline that does not parse checks nothing; regenerate with "
            f"tools/ci/check-citation-drift.py --write-classes",
            file=sys.stderr,
        )
        return 1

    demoted, promoted, recounted, undeclared, retired = [], [], [], [], []
    for key, clss in sorted(live.items()):
        was = baseline.get(key)
        if was is None:
            undeclared.append((key, clss))
        elif was != clss:
            # Which of the three, for the MESSAGE only -- the check itself is
            # exact equality, so any difference is a failure to be declared.
            # The count case is split out because calling it a demotion names
            # the wrong defect: `crates/moveit-constraints/src/lib.rs:119`
            # going from four occurrences to three is a document losing a
            # citation, not a citation losing its check, and the two want
            # different fixes.
            rank = lambda cs: sorted((CLASS_RANK.get(c, 0) for c in cs), reverse=True)  # noqa: E731
            if len(clss) != len(was):
                recounted.append((key, was, clss))
            elif rank(clss) < rank(was):
                demoted.append((key, was, clss))
            else:
                promoted.append((key, was, clss))
    for key, clss in sorted(baseline.items()):
        if key not in live:
            retired.append((key, clss))

    baseline_fail = bool(demoted or promoted or recounted or undeclared or retired)
    if baseline_fail:
        print(
            f"--- {len(demoted)} demoted, {len(recounted)} recounted, {len(undeclared)} "
            f"undeclared, {len(retired)} retired, {len(promoted)} promoted vs "
            f"{CLASS_BASELINE} ---",
            file=sys.stderr,
        )
        for (md, spec), was, now in demoted:
            print(
                f"FAIL {md}: `{spec}` was {' '.join(was)}, is now {' '.join(now)} -- less of this "
                f"citation's claim is checked than when the baseline was written, which is what "
                f"drift looks like here",
                file=sys.stderr,
            )
        for (md, spec), was, now in recounted:
            print(
                f"FAIL {md}: `{spec}` occurred {len(was)}x ({' '.join(was)}), now {len(now)}x "
                f"({' '.join(now)}) -- the document gained or lost an occurrence of this exact "
                f"citation; if one was renumbered, its new spelling is in the undeclared list",
                file=sys.stderr,
            )
        for (md, spec), clss in undeclared:
            print(
                f"FAIL {md}: `{spec}` ({' '.join(clss)}) is not in {CLASS_BASELINE} -- a citation "
                f"whose line number changed retires its old key and arrives as a new one, so an "
                f"undeclared citation is how a shift shows up",
                file=sys.stderr,
            )
        for (md, spec), clss in retired:
            print(
                f"FAIL {md}: `{spec}` ({' '.join(clss)}) is in {CLASS_BASELINE} but no longer in "
                f"the document",
                file=sys.stderr,
            )
        for (md, spec), was, now in promoted:
            print(
                f"FAIL {md}: `{spec}` was {' '.join(was)}, is now {' '.join(now)} -- more is "
                f"checked than before, which is good and still has to be recorded",
                file=sys.stderr,
            )
        print(
            f"FAIL regenerate with tools/ci/check-citation-drift.py --write-classes and read the "
            f"diff: a demotion accepted there is a citation nobody is checking any more",
            file=sys.stderr,
        )

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

    if blank_line:
        print(
            f"--- {len(blank_line)} citation(s) whose cited line is blank ---",
            file=sys.stderr,
        )
        for md, line_no, fname_part, cited, resolved_path, blank in blank_line:
            print(
                f"FAIL {md}:{line_no}: `{fname_part}:"
                f"{','.join(str(c) for c in cited)}` -> {resolved_path}: line(s) "
                f"{blank} are blank or whitespace-only. A blank line carries no claim, "
                f"so no anchoring can make it the subject -- re-derive the line "
                f"(a span starting one line early is the common case).",
                file=sys.stderr,
            )

    in_repo_fail = report_in_repo(tracked)

    if hard_fail or baseline_fail or in_repo_fail:
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
    args = sys.argv[1:]
    if args not in ([], ["--write-classes"]):
        print(f"usage: {sys.argv[0]} [--write-classes]", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(write_classes=bool(args)))
