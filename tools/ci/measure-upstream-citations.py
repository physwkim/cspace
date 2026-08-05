#!/usr/bin/env python3
# Usage: tools/ci/measure-upstream-citations.py --upstream <moveit2 checkout>
#
# The upstream half of `tools/ci/check-citation-drift.py`. That script's own
# header states the gap this one closes:
#
#   Known scope limit: only `path.rs:NNN` citations are checked. Upstream
#   `.cpp`/`.hpp`/`.h` citations (also present in PORTING-PLAN.md and
#   doc/port-coverage.md) are not resolvable without a local upstream
#   checkout and are not covered here.
#
# That is where the drift actually lives. Hand-checking five upstream
# citations while merging one branch found three wrong -- including
# `planning_scene.cpp:2451-2490` for `getCostSources`'s trajectory pair,
# whose two overloads really span `2451-2455` and `2457-2491`, so the
# citation is wrong at BOTH ends and had just been "fixed" that round.
# Nothing looked mechanically, so nothing found them.
#
# Not a `check-*`: it needs an upstream checkout, which `ci.yml`'s runners do
# not have. It is driven by `tools/ci/verify-upstream-citations.sh`, which
# owns the precondition (checkout present, pinned SHA) and is picked up by
# `tools/ci/verify-all.sh`'s glob. Same split as
# `measure-port-coverage.py` / `verify-port-coverage.sh`.
#
# What is checked, in the order a citation goes through it:
#
#   1. PATH RESOLUTION to exactly one upstream file. An exact repo-relative
#      path, or a partial one matched by the same subsequence-of-path-
#      components rule `check-citation-drift.py` and
#      `reconcile-assertion-ledgers.py` both use, with two additions this
#      corpus needs: a literal `...` component is an elision and is dropped
#      before matching (`chomp/.../chomp_planning_context.cpp`), and when a
#      bare basename matches several upstream files, `moveit_py/**`
#      candidates are dropped. `moveit_py` is a pybind11 binding shim, it is
#      outside the ported corpus `doc/port-coverage.md` §1 defines, and it
#      accounts for 25 of the 27 ambiguous basenames on its own; every
#      citation in this repo that does mean it writes the `moveit_py/`
#      prefix. A citation still matching zero or several files is reported,
#      never guessed at.
#
#   2. BOUNDS. Every line a citation names must exist in the resolved file.
#      Out-of-bounds is unambiguous drift and always a hard failure -- it
#      needs no anchor and admits no interpretation.
#
#   3. SYMBOL CONTAINMENT. When the citing text names a C++ symbol in
#      backticks, tightly paired with this citation, and that symbol has a
#      real definition span in the resolved file, the cited line must fall
#      inside that span. "Definition span" means a brace-matched body: a
#      function/method/constructor definition, or a `class`/`struct`/`union`/
#      `enum` body. A name with no such body in that file (a field, an
#      enumerator, a declaration in a header) yields no span and the citation
#      stays bounds-only rather than being judged against something this
#      script cannot compute.
#
#   4. SPAN EXACTNESS for ranges. A range whose FIRST line is exactly some
#      span's first line is a claim about that whole definition, so its last
#      line must be exactly that definition's last line -- or the last line
#      of a contiguous run of same-named definitions starting there, which
#      is how this corpus cites an overload pair (`:2451-2490 (pair)`).
#      "Contiguous" means every line between the runs is blank in the file.
#      This is the check that catches the defect above, and it is deliberately
#      exact: `2451-2490` is one line short of `2491`, and a checker that
#      accepted "close enough" would have passed all three of the citations
#      hand-checking found wrong. The same requirement fires from the other
#      end when the range's LAST line is exactly some span's last line.
#
#      A range that matches neither endpoint is a sub-region of a body (a
#      loop, a branch, a statement group) and is only required to sit inside
#      one span. That distinction is what keeps `chomp_planner.cpp:119-136`
#      -- a wrap-around loop inside `solve`'s `66-306` body -- from being
#      demanded to equal `solve`.
#
# Bare `` `:NNN` `` continuations inherit the file of the nearest preceding
# resolved citation ON THE SAME LINE, which is exactly how this corpus writes
# them (`` `collision_common.cpp:646`, `:650-659`, `:662-663` ``). A bare
# citation with no preceding path on its line is skipped, not attached to
# whatever came before on some earlier line.
#
# Corpus: tracked `.md` AND `.rs` files. The `.rs` files carry these
# citations in module and item doc comments in the same grammar the `.md`
# files use, and nothing else reads them.
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# ---------------------------------------------------------------- C++ parsing

# Keywords that can be followed by `(`...`)` and then `{` without being a
# function definition. Without this list every `if (...)` in the tree becomes
# a symbol named `if` whose span is the branch body.
CXX_KEYWORDS = {
    "if", "for", "while", "switch", "catch", "return", "sizeof", "alignof",
    "decltype", "throw", "new", "delete", "and", "or", "not", "case", "do",
    "else", "using", "typedef", "static_cast", "dynamic_cast", "const_cast",
    "reinterpret_cast", "noexcept", "typeid", "operator", "template",
    "explicit", "constexpr", "static_assert", "defined", "__attribute__",
}


def mask_non_code(text):
    """Blank out comment and literal contents byte-for-byte (newlines kept),
    so brace and paren counting only sees real code. C++ block comments do
    not nest, unlike the Rust masker in `check-citation-drift.py`; raw
    strings (`R"tag(...)tag"`) are handled because a `{` inside one would
    otherwise unbalance every span after it in the file."""
    out, i, n = [], 0, len(text)
    while i < n:
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j == -1 else j
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j == -1 else j + 2
            out.append("".join(c if c == "\n" else " " for c in text[i:j]))
            i = j
        elif text.startswith('R"', i) and re.match(r'R"([^("\\\s]{0,16})\(', text[i:]):
            m = re.match(r'R"([^("\\\s]{0,16})\(', text[i:])
            close = ")" + m.group(1) + '"'
            j = text.find(close, i + len(m.group(0)))
            j = n if j == -1 else j + len(close)
            out.append("".join(c if c == "\n" else " " for c in text[i:j]))
            i = j
        elif text[i] == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            out.append("".join(c if c == "\n" else " " for c in text[i:j]))
            i = j
        elif text[i] == "'":
            m = re.match(r"'(\\.|[^'\\\n])'", text[i : i + 4])
            if m:
                out.append(" " * len(m.group(0)))
                i += len(m.group(0))
            else:
                out.append("'")
                i += 1
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


IDENT_CALL_RE = re.compile(r"\b([A-Za-z_]\w*)\s*\(")
# A return type and specifiers, and nothing else: identifiers, `::`, template
# brackets, `&`/`*`, array brackets, whitespace. Anything with `.`, `->`, `=`,
# `(`, or a statement keyword is a call, not a declaration.
DECL_PREFIX_RE = re.compile(
    r"^(?!.*\b(?:return|throw|new|delete|else|case|co_return|co_await)\b)"
    r"[\w:<>,&*\[\]\s~]+$"
)
TAG_RE = re.compile(
    r"\b(class|struct|union|enum)\s+(?:class\s+|struct\s+)?"
    r"(?:[A-Za-z_]\w*_EXPORT\s+|MOVEIT_[A-Z_]*\s+)?([A-Za-z_]\w*)\b"
)


def _line_starts(text):
    starts, pos = [0], 0
    for line in text.split("\n")[:-1]:
        pos += len(line) + 1
        starts.append(pos)
    return starts


def _line_of(starts, off):
    lo, hi = 0, len(starts) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if starts[mid] <= off:
            lo = mid
        else:
            hi = mid - 1
    return lo + 1


def _match_brace(masked, open_at):
    depth, j, n = 0, open_at, len(masked)
    while j < n:
        if masked[j] == "{":
            depth += 1
        elif masked[j] == "}":
            depth -= 1
            if depth == 0:
                return j
        j += 1
    return -1


def _stmt_start_line(lines, line_no):
    """Walk up from the line holding the declarator to the first line of the
    whole declaration: signatures wrap, and a return type or a `template<>`
    header can sit on its own line above the name. Stop at anything that
    closed a previous statement or block (`;` `{` `}`), at an access
    specifier (`public:`), at a blank line, or at a preprocessor line --
    without those stops the walk runs back through a whole class body,
    because comments are already masked to spaces and never stop it."""
    i = line_no
    while i > 1:
        s = lines[i - 2].strip()
        if not s or s.startswith("#") or s.endswith((";", "{", "}", ":")):
            break
        i -= 1
    return i


def symbol_spans(text, allow_decls):
    """{name: [(start_line, end_line, kind)]} for every brace-matched
    definition in `text`; `kind` is 'fn', 'decl' or the tag keyword. Sorted by
    start line, because the overload-run rule in `part_verdict` walks them in
    file order.

    `allow_decls` is set for HEADERS only. `Type name(args);` is a member
    declaration in a header and a variable definition with a parenthesized
    initializer in a `.cpp` -- C++'s most vexing parse, and not decidable by
    the kind of scan this script does. Recording the `.cpp` shape gave
    `chomp_planner.cpp`'s local `RobotState goal_state(...)` a one-line
    "declaration span" for a name that is a variable. Headers are also where
    this corpus's bodiless citations actually point (a pure-virtual, an
    out-of-line-defined member, an overload set with only one inline body),
    so the restriction costs nothing it was added for."""
    masked = mask_non_code(text)
    lines = masked.split("\n")
    starts = _line_starts(masked)
    n = len(masked)
    spans = {}

    for m in IDENT_CALL_RE.finditer(masked):
        name = m.group(1)
        if name in CXX_KEYWORDS:
            continue
        depth, j = 0, m.end() - 1
        while j < n:
            if masked[j] == "(":
                depth += 1
            elif masked[j] == ")":
                depth -= 1
                if depth == 0:
                    break
            j += 1
        if j >= n:
            continue
        # Between the parameter list and the body: cv/ref qualifiers,
        # `noexcept`, `override`, a trailing return type, or a constructor
        # initializer list (which has its own parens, hence `pdepth`). A `;`
        # first means this was a declaration, a `)` or `,` first means the
        # whole thing was a nested call in some larger expression.
        k, pdepth, brace_at, semi_at = j + 1, 0, -1, -1
        while k < n:
            c = masked[k]
            if c == "(":
                pdepth += 1
            elif c == ")":
                pdepth -= 1
                if pdepth < 0:
                    break
            elif pdepth == 0:
                if c == ",":
                    break
                if c == ";":
                    semi_at = k
                    break
                if c == "{":
                    brace_at = k
                    break
            k += 1

        sig_line = _line_of(starts, m.start())
        start_line = _stmt_start_line(lines, sig_line)

        if brace_at == -1:
            # No body. A bodiless `NAME(...)` is either a DECLARATION (which
            # this corpus cites constantly -- a header's pure-virtual or
            # out-of-line-defined member is a real, nameable location) or a
            # CALL STATEMENT, which must not become a span or containment
            # becomes trivially satisfiable. What separates them is the text
            # between the start of the statement and the name: a declaration
            # is preceded by its return type and specifiers and nothing else,
            # while a call is preceded by nothing, by a receiver (`x.`,
            # `p->`), by an assignment, or by an enclosing call's `(`.
            if semi_at == -1 or not allow_decls:
                continue
            prefix = masked[starts[start_line - 1] : m.start()]
            # `collision_detection::removeCostSources(costs, ...);` -- a
            # qualified CALL statement, whose prefix is nothing but a
            # namespace. A declaration always has a return type separated
            # from the name by whitespace, so a prefix that is one
            # unbroken qualifier is a call.
            if prefix.rstrip().endswith(":"):
                continue
            if not prefix.strip() or not DECL_PREFIX_RE.match(prefix):
                continue
            spans.setdefault(name, []).append(
                (start_line, _line_of(starts, semi_at), "decl")
            )
            continue
        end_at = _match_brace(masked, brace_at)
        if end_at == -1:
            continue
        spans.setdefault(name, []).append((start_line, _line_of(starts, end_at), "fn"))

    for m in TAG_RE.finditer(masked):
        k, brace_at = m.end(), -1
        while k < n:
            c = masked[k]
            if c == ";":
                break
            if c == "{":
                brace_at = k
                break
            k += 1
        if brace_at == -1:
            continue
        end_at = _match_brace(masked, brace_at)
        if end_at == -1:
            continue
        sig_line = _line_of(starts, m.start())
        spans.setdefault(m.group(2), []).append(
            (_stmt_start_line(lines, sig_line), _line_of(starts, end_at), m.group(1))
        )

    for v in spans.values():
        v.sort()
    return spans


# ------------------------------------------------------------- citation model

CXX_EXT = "cpp|hpp|h|cc|cxx"
SPEC = r"\d+(?:-\d+)?(?:[,·]\d+(?:-\d+)?)*"
# One alternation, scanned in positional order, because a bare `` `:NNN` ``
# means "the same file as the last one named on this line" and getting that
# wrong invents drift that is not there. Every token that can name a file has
# to be in the same pass as the citations, including the ones with no line
# number at all:
#
#   PATH  `planning_scene.cpp:2451-2490`   -- names an upstream file and cites it
#   RS    `scene.rs:1859`                  -- names a PORT file; every bare
#                                             `:NNN` after it belongs to
#                                             `check-citation-drift.py`, not here
#   FILE  `chomp_optimizer.cpp`            -- names a file without citing it
#                                             ("no definition anywhere in
#                                             `chomp_optimizer.cpp` -- only
#                                             `perturbTrajectory` (`:959-990`)")
#   REBASE `cpp:259` / `.cpp`              -- switches to the sibling
#                                             translation unit of the current
#                                             file ("Header (`x.hpp:147-159`)
#                                             ... the `.cpp` definition
#                                             (`:338-341`)"). The sibling is
#                                             found by basename, not by
#                                             swapping the extension in place:
#                                             upstream headers live under
#                                             `include/`, sources under `src/`.
#   BARE  `:2490`                          -- inherits whatever came last
TOKEN_RE = re.compile(
    rf"`(?P<path>(?:[\w./-]+/)?[\w.+-]+\.(?:{CXX_EXT})):(?P<pspec>{SPEC})`"
    rf"|`(?P<rs>(?:[\w./-]+/)?[\w.+-]+\.rs):(?:{SPEC})`"
    rf"|`(?P<rebase>{CXX_EXT}):(?P<rspec>{SPEC})`"
    rf"|`:(?P<bare>{SPEC})`"
    rf"|`(?P<file>(?:[\w./-]+/)?[\w.+-]+\.(?:{CXX_EXT}|rs))`"
    rf"|`\.(?P<ext>{CXX_EXT})`"
)
IDENT_IN_BACKTICKS_RE = re.compile(r"`([A-Za-z_][\w:]{2,})(?:\(\))?`")
HEX_SHA_RE = re.compile(r"^[0-9a-f]{7,40}$")
# `` `file:NNN` (`name`) `` -- the anchor immediately follows the citation.
# Trailing words inside the parens are allowed (`(`getCollisionCost` body)`),
# but the paren must OPEN with the backticked name, so a following clause that
# merely mentions some other symbol cannot be picked up as this citation's
# anchor.
FOLLOWING_ANCHOR_RE = re.compile(r"^\s*\(\s*`([A-Za-z_][\w:]{2,})(?:\(\))?`")
LOCAL_WINDOW = 60
# Backtick-quoted words that name a type or a language construct, not a symbol
# whose definition a citation could be inside. `bool` reaches `symbol_spans`
# as a real entry -- C++ has `bool(...)` conversions and functional casts --
# so without this a doc sentence containing `` `bool` `` next to a citation
# anchors that citation on whatever line the cast happens to be at.
NOT_A_SYMBOL = {
    "bool", "int", "void", "char", "long", "short", "float", "double",
    "unsigned", "signed", "size_t", "auto", "const", "static", "inline",
    "true", "false", "nullptr", "std", "this", "struct", "class", "enum",
    "union", "namespace", "public", "private", "protected", "virtual",
    "override", "final", "template", "typename", "operator", "friend",
    "mutable", "explicit", "constexpr", "noexcept",
}
TIGHT_GAP_RE = re.compile(r"^[\s(,:;/·—–\-'’\"]*$")
# How far short of a definition's real last line a range may stop before it is
# called drift rather than a deliberate sub-region. See `part_verdict`.
BRACE_SLACK = 5


def parse_parts(spec):
    """`2451-2490,2517` -> [(2451, 2490), (2517, 2517)]. Each comma-separated
    part is an independent claim; `check-citation-drift.py` learned the hard
    way that reading only the first silently drops the rest."""
    parts = []
    for chunk in re.split(r"[,·]", spec):
        if "-" in chunk:
            a, b = chunk.split("-", 1)
            parts.append((int(a), int(b)))
        else:
            parts.append((int(chunk), int(chunk)))
    return parts


def path_matches(full_path, want):
    """`check-citation-drift.py`'s rule, plus: a literal `...` component is an
    elision this corpus writes for a long upstream path
    (`moveit_py/.../planning_response.cpp`) and is dropped -- the subsequence
    match already allows the gap it stands for."""
    w = [c for c in want.split("/") if c and c != "..."]
    have = full_path.split("/")
    if not have or not w or have[-1] != w[-1]:
        return False
    it = iter(have[:-1])
    return all(part in it for part in w[:-1])


def resolve_path(cited, upstream_files, by_basename):
    if cited.startswith("/"):
        cited = cited.lstrip("/")
    if "/" in cited or "..." in cited:
        if cited in upstream_files:
            return [cited]
        cands = [p for p in upstream_files if path_matches(p, cited)]
    else:
        cands = list(by_basename.get(cited, []))
    if len(cands) > 1:
        narrowed = [p for p in cands if not p.startswith("moveit_py/")]
        if narrowed:
            cands = narrowed
    return sorted(cands)


def sibling_with_ext(rel, ext, by_basename):
    """The translation unit next to `rel` with extension `ext`. Resolved by
    basename against the upstream tree rather than by rewriting `rel` in
    place, because upstream puts a header under `<pkg>/include/moveit/<mod>/`
    and its source under `<pkg>/src/` -- the two paths differ in more than
    the suffix."""
    stem = rel.rsplit("/", 1)[-1].rsplit(".", 1)[0]
    cands = [
        p
        for p in by_basename.get(f"{stem}.{ext}", [])
        if not p.startswith("moveit_py/")
    ]
    return cands[0] if len(cands) == 1 else None


OBSOLETE_H_RE = re.compile(r'#pragma message\("\.h header is obsolete')
SHIM_INCLUDE_RE = re.compile(r'^\s*#include\s*[<"]([^>"]+)[>"]', re.MULTILINE)


def shim_target(shim_lines, upstream_set):
    """The `.hpp` an obsolete `.h` forwards to, taken from the shim's own
    `#include` rather than inferred. The shims are generated by upstream's
    `create_deprecated_headers.py` and hold exactly one include, so the name
    is stated, not guessed -- and `sibling_with_ext` cannot supply it here
    anyway, since a basename like `utils.hpp` is ambiguous across packages
    while the include path is not."""
    for m in SHIM_INCLUDE_RE.finditer("\n".join(shim_lines)):
        inc = m.group(1)
        cands = [p for p in upstream_set if p.endswith("/include/" + inc)]
        if len(cands) == 1:
            return cands[0]
    return None


def find_anchors(line, start, end, spans, window_floor):
    """Names tightly paired with this citation that have a definition span in
    the resolved file. Tight only, for `check-citation-drift.py`'s reason: a
    name merely present in the same table row names what a citation is ABOUT
    at least as often as what contains it, and treating the two the same is
    the content-blind match this whole family of scripts exists to reject."""
    found = []

    def add(raw):
        name = raw.split("::")[-1]
        if name in NOT_A_SYMBOL or HEX_SHA_RE.match(name) or name not in spans:
            return
        if name not in found:
            found.append(name)

    m = FOLLOWING_ANCHOR_RE.match(line[end:])
    if m:
        add(m.group(1))
    window = line[max(window_floor, start - LOCAL_WINDOW) : start]
    # Two hard clause boundaries the window must not reach across, both found
    # by reading this script's own first-run failures rather than assumed:
    #
    #   `|` -- a markdown table cell. Every claim-audit row states the CLAIM in
    #   one column and the EVIDENCE in another, and the two name different
    #   functions on purpose: "nothing else in `doSmoothing`/`reset`/
    #   `getVelAccelJerkBounds` touches `node` | CONFIRMED | `ruckig_filter.cpp:
    #   52-59` (only use of `node`)" -- `52-59` is `initialize`, and the three
    #   names in the previous cell are what it is contrasted WITH.
    #
    #   `)` -- a closed parenthetical belonging to the citation before this
    #   one: "`acceleration_filter.cpp:309-310` (... in `doSmoothing`),
    #   `:397-398` (... in `reset`)". `prev_citation_end` does not stop this,
    #   because the parenthetical opens AFTER the previous citation ends.
    for sep in "|)":
        cut = window.rfind(sep)
        if cut != -1:
            window = window[cut + 1 :]
    for m in IDENT_IN_BACKTICKS_RE.finditer(window):
        # The gap between the name and the citation must be punctuation only.
        # A gap with WORDS in it is prose about the symbol, not a pointer at
        # it: "`integrateBackward` failure paths, `...hpp:184`" cites where
        # `end_trajectory_` is DECLARED, and `integrateBackward` is merely
        # what writes it; "use `knowsFrameTransform` to tell the two apart"
        # (`planning_scene.hpp:204`)" cites the doc comment that says so.
        # Both read as containment claims to a scan that only measures
        # distance, and neither is one.
        if TIGHT_GAP_RE.match(window[m.end() :]):
            add(m.group(1))
    return found


# Nouns with which the citing text asserts, in prose, that a range IS a whole
# definition. This is the second half of the span-exactness check and the half
# that catches the motivating defect in the form it actually shipped:
# `getCostSources`'s `:2451-2490` is written as "(the `trajectory`-taking pair,
# `planning_scene.cpp:2451-2490`)" and as "`planning_scene.cpp:2451-2490`
# (pair)", and neither carries a C++ symbol whose span this script can look up
# -- `trajectory` is a parameter name and `path_cost_sources` is the port's own
# name for the thing. The claim is in the noun, so the noun is what fires.
#
# Every word here was kept or dropped by running the rule over the corpus and
# reading each hit, not by judging the word in the abstract. Dropped, and why:
#
#   `span`/`spans` -- this corpus's most common word for an arbitrary line
#   range, not for a definition ("output-trajectory fill-in span
#   `chomp_planner.cpp:255-268`" is a `for` loop; "spans exactly `:263-270`" is
#   an if/else). It fired on five ranges that are correctly cited sub-regions.
#
#   `body`/`bodies`/`본체` -- ambiguous between a function body and a loop or
#   `if` body ("upstream's own loop body (`planning_scene.cpp:2376-2422`)"),
#   and in `moveit-geometry` it is a geometric body. The cost of dropping it
#   is real and is stated in the report: `setSubframesOfObject`'s bodies at
#   `world.cpp:262-278` had to be found by hand.
#
#   `정확히`/`is exactly` -- attaches to a sameness claim about the PORT
#   ("상류와 정확히 같다"), not to the extent of the range.
SPAN_ASSERTION_RE = re.compile(
    r"(?<![\w-])(?:pairs?|overloads?|entire|whole)(?![\w-])|전체|오버로드|쌍",
    re.IGNORECASE,
)
# The noun must be ADJACENT to the citation, by the same reasoning
# `find_anchors` uses for symbol names and enforced by the same `TIGHT_GAP_RE`:
# a noun elsewhere in the sentence describes the subject, not the extent. It
# fires on "...pair, `planning_scene.cpp:2451-2490`" and on
# "`planning_scene.hpp:553-609` (4 overloads:", and not on
# "`:1452-1460` -- 평범한 `Octomap` 오버로드에는" or "(`...:151-164`) -- upstream
# overloads the *output*", where the noun is what the citation is ABOUT.
TRAILING_ASSERTION_RE = re.compile(
    r"^[\s(]*(?:\d+\s+)?(?:pairs?|overloads?|entire|whole|전체|오버로드|쌍)", re.IGNORECASE
)


def span_assertion(line, start, end, window_floor):
    """Whether the text immediately around this citation calls the range a
    whole definition."""
    if TRAILING_ASSERTION_RE.match(line[end:]):
        return True
    window = line[max(window_floor, start - LOCAL_WINDOW) : start]
    for sep in "|)":
        cut = window.rfind(sep)
        if cut != -1:
            window = window[cut + 1 :]
    for m in SPAN_ASSERTION_RE.finditer(window):
        if TIGHT_GAP_RE.match(window[m.end() :]):
            return True
    return False


def contiguous_run_end(span_list, first, file_lines, all_spans_sorted):
    """Every legal last line for a range starting at `first`: the span that
    starts there, then each further definition reachable across nothing but
    blank lines.

    The run walks ALL definitions in the file, not only the anchored name's,
    because an adjacent pair need not share a name:
    `lexical_casts.cpp:45-58` is `toStringImpl` (45-53), one blank line, and
    `toString(double)` (55-58) -- and the citing text names both. A same-name
    run is the `getCostSources` overload-pair case; this is the same shape
    with two names. What it must NOT admit is a range ending inside a body
    or at an arbitrary line, which is why the gap test is "blank in the raw
    file": a doxygen comment between two overloads stops the run, so
    `distanceToCollisionUnpadded`'s four comment-separated overloads cannot
    absorb a range that overshoots them."""
    idx = next((i for i, s in enumerate(span_list) if s[0] == first), None)
    if idx is None:
        return None
    end = span_list[idx][1]
    ends = [end]
    for nxt_start, nxt_end in all_spans_sorted:
        if nxt_start <= end:
            continue
        if all(not g.strip() for g in file_lines[end : nxt_start - 1]):
            end = nxt_end
            ends.append(end)
        else:
            break
    return ends


def all_spans(spans, anchors):
    out = []
    for name in anchors:
        out.extend(spans[name])
    return sorted(out)


def part_verdict(lo, hi, span_list, file_lines, anchors, all_file_spans):
    """OK / a reason string for one `lo-hi` part against the anchors' spans."""
    inside = [s for s in span_list if s[0] <= lo and hi <= s[1]]
    starts_a_span = any(s[0] == lo for s in span_list)
    ends_a_span = any(s[1] == hi for s in span_list)

    if lo == hi:
        if inside:
            return None
        # A single line is a pinpoint -- at a statement, a declaration, or a
        # CALL of the named symbol from somewhere else entirely. The corpus
        # cites all three under the same tight pairing:
        # `getFrameTransform` (`planning_scene.cpp:1606`) is the line where
        # `processAttachedCollisionObjectMsg` CALLS it, 400 lines from its
        # own body, and that is what the sentence means. So a line that
        # names the symbol satisfies the pairing; only a line that neither
        # sits inside it nor mentions it is drift.
        if any(name in file_lines[lo - 1] for name in anchors):
            return None
        return "neither inside nor mentioning the named symbol"

    # A range that lands exactly on some definition's boundaries is pinned to
    # a real span even when the tightly-paired name is a different one. That
    # happens where the port's own name for a thing is not upstream's:
    # "`Se3Space::distance` -- `floating_joint_model.cpp:120-126`(translation)"
    # is exactly `FloatingJointModel::distanceTranslation`, cited correctly.
    if (lo, hi) in all_file_spans:
        return None

    sorted_all = sorted(all_file_spans)
    if starts_a_span:
        legal_ends = contiguous_run_end(span_list, lo, file_lines, sorted_all) or []
        if hi in legal_ends:
            return None
        if inside and not ends_a_span:
            # Starts exactly at a definition's first line but stops short of
            # its last. Two different things wear this shape, and only one is
            # drift:
            #
            #   `:2451-2490` for `getCostSources`, whose real run ends at
            #   2491 -- the closing brace left outside the range. That is the
            #   defect, and it is exact by one line.
            #
            #   `:131-154` for `executeMoveCallbackPlanAndExecute`, which
            #   really ends at 189 -- a deliberate citation of the early-return
            #   block the sentence is about, starting from the function head
            #   because that is where the reader should start reading.
            #
            # The separator is how far short it stops. A brace, or a nest of
            # them, is a handful of lines; a described sub-region is tens.
            # BRACE_SLACK is where this draws the line, and it is a real limit
            # on what this script can see: a range that stops 50 lines short
            # of its function is not reported, whether it meant to or not.
            shortfall = min(e - hi for e in legal_ends if e > hi)
            if shortfall > BRACE_SLACK:
                return None
            return (
                f"starts at a definition's first line ({lo}) but ends at {hi}, "
                f"{shortfall} line(s) short; that definition (and its contiguous "
                f"run) ends at {'/'.join(map(str, legal_ends[:3]))}"
            )
        if not inside:
            return (
                f"starts at a definition's first line ({lo}) but {hi} is past "
                f"its end ({'/'.join(map(str, legal_ends[:3]))})"
            )
        return None
    if ends_a_span and not inside:
        return f"ends at a definition's last line ({hi}) but starts at {lo}, outside it"
    if inside:
        return None
    # Not a span claim at all: a range that does not touch either boundary of
    # the named symbol AND does not contain its definition head is citing
    # where the symbol is USED. `robot_state.cpp:1836-1863` is the branch in
    # `setFromIK` that forwards to `setFromIKSubgroups` (whose own body is at
    # 2049); the sentence says exactly that. The head test is what keeps this
    # from excusing `:1416-1433` for `createOctomap`, which straddles that
    # function's own first line and is a span claim that drifted.
    if not any(lo <= s <= hi for (s, _e, _k) in span_list) and any(
        name in "\n".join(file_lines[lo - 1 : hi]) for name in anchors
    ):
        return None
    return "not inside any span of the named symbol"


# ------------------------------------------------------------------ the sweep


EXEMPTIONS_PATH = REPO_ROOT / "tools/ci/upstream-citation-exemptions.json"


def load_exemptions():
    """{(doc, line, resolved_path, spec)} for citations this repo writes
    deliberately wrong -- it quotes a briefing's citation in order to refute
    it on the next line, so "fixing" it would delete the finding. Same
    mechanism and same burden of proof as
    `tools/ci/section-reference-external.json`: each entry carries the reason
    in the file, and each is pinned to an exact document line and an exact
    spec, so it stops applying the moment either moves."""
    if not EXEMPTIONS_PATH.exists():
        return set()
    data = json.loads(EXEMPTIONS_PATH.read_text(encoding="utf-8"))
    return {(e["doc"], e["line"], e["upstream"], e["spec"]) for e in data["exemptions"]}


def tracked_files():
    out = subprocess.run(
        ["git", "ls-files", "-z"], cwd=REPO_ROOT, capture_output=True, check=True
    ).stdout.decode("utf-8")
    return [p for p in out.split("\0") if p]


def upstream_tracked(upstream):
    out = subprocess.run(
        ["git", "ls-files", "-z"], cwd=upstream, capture_output=True, check=True
    ).stdout.decode("utf-8")
    return [p for p in out.split("\0") if p.endswith((".cpp", ".hpp", ".h", ".cc", ".cxx"))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--upstream", required=True)
    args = ap.parse_args()
    upstream = Path(args.upstream)

    files = upstream_tracked(upstream)
    upstream_set = set(files)
    by_basename = {}
    for p in files:
        by_basename.setdefault(p.rsplit("/", 1)[-1], []).append(p)

    text_cache, span_cache = {}, {}

    def lines_of(rel):
        if rel not in text_cache:
            body = (upstream / rel).read_text(encoding="utf-8", errors="replace").split("\n")
            # A file ending in a newline splits to a trailing "" that is not a
            # line. Left in, `len()` is one too many and the bounds check
            # accepts a citation one line past the end of the file -- the
            # exact off-by-one this script exists to catch, in the script.
            if body and body[-1] == "":
                body.pop()
            text_cache[rel] = body
        return text_cache[rel]

    def spans_of(rel):
        if rel not in span_cache:
            span_cache[rel] = symbol_spans(
                "\n".join(lines_of(rel)), allow_decls=rel.endswith((".hpp", ".h", ".hh"))
            )
        return span_cache[rel]

    exemptions = load_exemptions()
    total = 0
    anchor_verified = 0
    bounds_only = 0
    inherited_checked = 0
    out_of_bounds = []
    obsolete_header = []
    span_mismatch = []
    unresolved = {}
    exempted = 0

    corpus = [p for p in tracked_files() if p.endswith((".md", ".rs"))]
    for doc in corpus:
        for line_no, line in enumerate(
            (REPO_ROOT / doc).read_text(encoding="utf-8", errors="replace").split("\n"), 1
        ):
            base = None  # the file a bare `:NNN` on this line would inherit
            prev_end = 0
            for m in TOKEN_RE.finditer(line):
                start, end = m.start(), m.end()
                window_floor, prev_end = prev_end, end
                g = m.groupdict()

                if g["rs"] is not None:
                    base = None
                    continue
                if g["file"] is not None:
                    if g["file"].endswith(".rs"):
                        base = None
                    else:
                        cands = resolve_path(g["file"], upstream_set, by_basename)
                        base = cands[0] if len(cands) == 1 else None
                    continue
                if g["ext"] is not None:
                    base = sibling_with_ext(base, g["ext"], by_basename) if base else None
                    continue

                if g["path"] is not None:
                    spec = g["pspec"]
                    cands = resolve_path(g["path"], upstream_set, by_basename)
                    if len(cands) != 1:
                        unresolved.setdefault(
                            g["path"],
                            "no upstream file matches" if not cands else f"ambiguous: {cands}",
                        )
                        base = None
                        continue
                    resolved = base = cands[0]
                elif g["rebase"] is not None:
                    spec = g["rspec"]
                    resolved = base = (
                        sibling_with_ext(base, m.group(0)[1:].split(":")[0], by_basename)
                        if base
                        else None
                    )
                    if resolved is None:
                        continue
                else:
                    spec = g["bare"]
                    if base is None:
                        continue
                    resolved = base
                    inherited_checked += 1

                total += 1
                if (doc, line_no, resolved, spec) in exemptions:
                    exempted += 1
                    continue
                file_lines = lines_of(resolved)
                n_lines = len(file_lines)
                parts = parse_parts(spec)
                if resolved.endswith(".h") and OBSOLETE_H_RE.search("\n".join(file_lines[:60])):
                    obsolete_header.append(
                        (doc, line_no, resolved, spec, shim_target(file_lines, upstream_set))
                    )
                    continue
                if [x for lo, hi in parts for x in (lo, hi) if not 1 <= x <= n_lines]:
                    out_of_bounds.append((doc, line_no, resolved, spec, n_lines))
                    continue

                spans = spans_of(resolved)
                anchors = find_anchors(line, start, end, spans, window_floor)
                # A comma list enumerates SITES, not a span: the four lines of
                # `collision_env_hybrid.cpp:49,61,69,169` are three
                # constructor base-init calls plus one `setWorld` call, and
                # the name they are all paired with is what they are about.
                # There is no containment claim to check, so these are
                # bounds-checked only -- which is what caught
                # `world.cpp:220,326,650,655` anyway.
                if not anchors and len(parts) == 1 and parts[0][0] != parts[0][1]:
                    # No symbol anchor, but the SENTENCE asserts the range is a
                    # whole definition. That is a containment claim stated in
                    # words instead of in a name, and it needs no anchor to be
                    # checkable: the range must equal a real span.
                    lo, hi = parts[0]
                    if span_assertion(line, start, end, window_floor):
                        every_span = {(s, e) for v in spans.values() for (s, e, _k) in v}
                        legal = contiguous_run_end(
                            [(s, e, "") for (s, e) in every_span if s == lo],
                            lo,
                            file_lines,
                            sorted(every_span),
                        )
                        if (lo, hi) not in every_span and hi not in (legal or []):
                            span_mismatch.append(
                                (
                                    doc,
                                    line_no,
                                    resolved,
                                    spec,
                                    ["<text asserts a whole definition>"],
                                    [
                                        f"{lo}-{hi}: the text calls this range a whole "
                                        f"definition, but no definition in the file spans "
                                        + (
                                            f"{lo}-{hi} (one starting at {lo} ends at "
                                            f"{'/'.join(map(str, legal[:3]))})"
                                            if legal
                                            else f"{lo}-{hi}, and none starts at {lo}"
                                        )
                                    ],
                                    [(lo, e, "") for e in (legal or [])],
                                )
                            )
                            continue
                        anchor_verified += 1
                        continue
                if not anchors or len(parts) > 1:
                    bounds_only += 1
                    continue
                span_list = all_spans(spans, anchors)
                every_span = {(s, e) for v in spans.values() for (s, e, _k) in v}
                reasons = [
                    f"{lo}-{hi}: {r}" if lo != hi else f"{lo}: {r}"
                    for lo, hi in parts
                    if (r := part_verdict(lo, hi, span_list, file_lines, anchors, every_span))
                    is not None
                ]
                if reasons:
                    span_mismatch.append((doc, line_no, resolved, spec, anchors, reasons, span_list))
                else:
                    anchor_verified += 1

    if total == 0:
        print(
            "FAIL parsed zero upstream `path.cpp:NNN` citations across tracked "
            ".md/.rs files -- the citation grammar changed and this checked nothing",
            file=sys.stderr,
        )
        return 1

    if out_of_bounds:
        print(f"--- {len(out_of_bounds)} out-of-bounds citation(s) ---", file=sys.stderr)
        for doc, line_no, resolved, spec, n_lines in out_of_bounds:
            print(
                f"FAIL {doc}:{line_no}: cites {resolved}:{spec}, but that file "
                f"has only {n_lines} lines",
                file=sys.stderr,
            )

    if obsolete_header:
        print(
            f"--- {len(obsolete_header)} citation(s) into an obsolete `.h` "
            f"forwarding header ---",
            file=sys.stderr,
        )
        for doc, line_no, resolved, spec, hpp in obsolete_header:
            print(
                f"FAIL {doc}:{line_no}: cites {resolved}:{spec}, but that file is "
                f"upstream's `.h header is obsolete` forwarding shim -- it holds "
                f"nothing but a licence block and one `#include`. The code cited "
                f"is in {hpp or '(no .hpp sibling found)'}",
                file=sys.stderr,
            )

    if span_mismatch:
        print(f"--- {len(span_mismatch)} span-mismatch citation(s) ---", file=sys.stderr)
        for doc, line_no, resolved, spec, anchors, reasons, span_list in span_mismatch:
            desc = "; ".join(
                f"`{n}` " + ", ".join(f"{s}-{e}" for s, e, _k in span_list) for n in anchors[:1]
            )
            print(
                f"FAIL {doc}:{line_no}: cites {resolved}:{spec} anchored on "
                f"{'/'.join(anchors)} -- {'; '.join(reasons)} [{desc}]",
                file=sys.stderr,
            )

    if unresolved:
        print(
            f"--- {len(unresolved)} distinct unresolvable cited path(s) "
            f"(reported, not a failure: outside the upstream tree or ambiguous) ---",
            file=sys.stderr,
        )
        for p, why in sorted(unresolved.items()):
            print(f"  `{p}` -- {why}", file=sys.stderr)

    if out_of_bounds or span_mismatch or obsolete_header:
        print(
            f"FAIL {len(out_of_bounds)} out-of-bounds + {len(obsolete_header)} "
            f"obsolete-header + {len(span_mismatch)} span-mismatch (of {total} "
            f"upstream citations resolved across {len(corpus)} tracked .md/.rs files)",
            file=sys.stderr,
        )
        return 1

    print(
        f"OK {total} upstream citations across {len(corpus)} tracked .md/.rs files "
        f"against {upstream}: {anchor_verified} span-verified (cited lines inside "
        f"the named symbol's definition), {bounds_only} bounds-checked only (no "
        f"tightly-paired symbol with a definition span), {exempted} exempted "
        f"(tools/ci/upstream-citation-exemptions.json), {inherited_checked} of the "
        f"total reached through a bare `:NNN` continuation, {len(unresolved)} "
        f"distinct unresolvable paths (reported above), 0 out-of-bounds, "
        f"0 obsolete-header, 0 span-mismatch"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
