#!/usr/bin/env python3
# Usage: tools/ci/measure-upstream-citations.py --upstream <moveit2 checkout>
#                                              [--source PREFIX=<checkout> ...]
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
#      Resolution is over ONE index built from every pinned source root, not
#      over moveit2 with the vendored trees as a fallback. This corpus cites
#      the four vendored packages under `third_party/` -- geometric_shapes,
#      srdfdom, octomap, orocos_kinematics_dynamics -- 77 times, and before
#      those roots were indexed all 77 landed in the unresolvable list, which
#      was reported and did not fail. An unresolvable citation is an
#      unchecked one, so that list reading as "not a failure" was the same
#      silence a skipped check produces. What is left unresolvable now has to
#      be DECLARED, in `upstream-citation-exemptions.json`'s `unresolvable`
#      key, with the project it names and why no tree covers it; an undeclared
#      path fails and so does a declaration nothing cites any more. The list
#      still is not a pass -- a declaration says no tree covers the citation,
#      not that the citation is right -- but it is now a list somebody signed.
#      A fallback would have resolved them
#      too, and would also have hidden any basename the two trees share: it
#      answers with the first root while reading as a unique match. One index
#      makes such a basename ambiguous, which is what it is. Four exist today
#      (`aabb.cpp`, `aabb.h`, `config.h`, `main.cpp`) and none is cited with
#      a line number anywhere in this repository.
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
# A HISTORICAL citation, `` `oracle.cpp@c0838b4^:4752` ``, is none of the
# above and goes through step 1 and step 2 only, with step 2 run against the
# named revision rather than against HEAD. See `HIST_REV`.
#
# Corpus: tracked `.md` AND `.rs` files. The `.rs` files carry these
# citations in module and item doc comments in the same grammar the `.md`
# files use, and nothing else reads them.
import argparse
import collections
import json
import re
import subprocess
import sys
from pathlib import Path

import baseline_header

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
# A one-line declaration that declares a NAME rather than calling something:
# everything up to a `;`, with an optional initializer cut off first. The
# shapes are read off the corpus, not imagined --
#
#   static const std::string OCTOMAP_NS;                  planning_scene.hpp:113
#   PlannerConfigurationMap config_settings_;      planning_interface.hpp:210
#   std::shared_ptr<rclcpp::Node> node_;             planning_pipeline.hpp:257
#   std::size_t max_contacts_per_pair = 1;           collision_common.hpp:176
#   static const unsigned int DEFAULT_MAX_SAMPLING_ATTEMPTS = 2;
#                                                   constraint_sampler.hpp:64
#
# -- and every one of them is a name this corpus anchors a citation on and
# `symbol_spans` had no span for, which is what `why_bounds_only`'s "field,
# macro, alias, namespace" bucket was counting.
MEMBER_DECL_RE = re.compile(r"^\s*(?P<decl>[^;=]*?)\s*(?:=[^;]*)?;\s*$")
# The first word rules the line out: a tag is a forward declaration or a
# definition `TAG_RE` already owns, and the rest introduce something that is
# not a declarator. `DECL_PREFIX_RE` covers the statement keywords.
NOT_A_DECL_HEAD = {
    "class", "struct", "union", "enum", "namespace", "template", "friend",
    "using", "typedef", "public", "private", "protected", "operator",
    "extern", "export", "static_assert",
}
IDENT_RE = re.compile(r"[A-Za-z_]\w*")


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
    # Every `NAME(...) { ... }` block, keyword-headed or not. The member rule
    # below uses this to tell a class-scope declaration from a local, and it
    # must not depend on the head's name surviving `CXX_KEYWORDS`: the
    # `std::stringstream msg;` at `eigen_test_utils.hpp:65` is a local inside
    # an `operator()` body, and `operator` is a keyword, so a name-derived
    # exclusion would hand `msg` a "member" span in a header.
    blocks = []

    for m in IDENT_CALL_RE.finditer(masked):
        name = m.group(1)
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
            if name in CXX_KEYWORDS:
                continue
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
        blocks.append((start_line, _line_of(starts, end_at)))
        if name in CXX_KEYWORDS:
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

    # Data members and static constants. HEADERS only, and for the same reason
    # `Type name(args);` is: in a `.cpp` this shape is a local variable, and
    # giving a local a "definition span" is how `benchmarks`
    # (`BenchmarkExecutor.cpp:1012`, a `for` body 40 lines from the `double
    # benchmarks;` that would have carried the span) turns into a confident
    # verdict about a name the citing sentence does not mean.
    if allow_decls:
        # A header carries inline bodies too, and a local declared inside one
        # is exactly the `.cpp` shape this restriction exists to keep out. The
        # blocks are already known by here, so the exclusion is exact rather
        # than a brace-depth guess that a namespace or a class body would
        # throw off -- and a class body, which is what a member is declared
        # in, is not a `NAME(...) { ... }` block and so never excludes.
        bodies = sorted(blocks)
        for i, line in enumerate(lines, 1):
            if "(" in line or ")" in line or "{" in line or "}" in line:
                continue
            if any(s < i <= e for (s, e) in bodies):
                continue
            m = MEMBER_DECL_RE.match(line)
            if not m:
                continue
            decl = m.group("decl")
            if not decl or not DECL_PREFIX_RE.match(decl):
                continue
            names = IDENT_RE.findall(decl)
            # A type and a declarator, in that order, and nothing after the
            # declarator but array bounds: one identifier is an expression
            # statement, and a declarator that is not last is a call or a cast
            # this scan has no business naming.
            if len(names) < 2 or names[0] in NOT_A_DECL_HEAD:
                continue
            if not re.fullmatch(r"\s*(?:\[[^\]]*\])*", decl[decl.rindex(names[-1]) + len(names[-1]) :]):
                continue
            spans.setdefault(names[-1], []).append((i, i, "member"))

    for v in spans.values():
        v.sort()
    return spans


# ------------------------------------------------------------- citation model

CXX_EXT = "cpp|hpp|h|cc|cxx"
SPEC = r"\d+(?:-\d+)?(?:[,·]\d+(?:-\d+)?)*"
# The revision half of a HISTORICAL citation, `<path>@<rev>:<spec>`.
#
# Some line numbers in this corpus are not claims about the file today and must
# not be re-pointed at it. `PORTING-PLAN.md` §138.3 records a defect by citing
# the two lines that carried it, and the fix it reports DELETED both; pointing
# them at today's file erases the finding the paragraph exists to make. Writing
# them as bare `<path>:<line>` is worse still -- that is a live claim, false by
# construction, and every gate here would either fail it or (as happened for
# years) never resolve the path and report nothing.
#
# So the revision moves INSIDE the token, between the extension and the colon.
# That placement is what makes the shape unambiguous rather than merely
# unrecognised: no `<path>:<line>` grammar can match it by accident, and the
# number is checked -- against `git show <rev>:<path>` in the checkout the path
# resolved into, not against HEAD. Bounds only: the content at that revision is
# what the citing sentence quotes, and this script does not parse a historical
# blob for symbol spans.
#
# The path must still resolve TODAY, which is deliberate: a historical citation
# into a file that has since been renamed or deleted stops resolving and has to
# be rewritten, rather than sitting on a path nothing can find.
HIST_REV = r"[0-9a-f]{7,40}\^{0,3}"
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
#   HIST  `oracle.cpp@c0838b4^:4752`       -- a line number that was true at a
#                                             NAMED REVISION and is not a claim
#                                             about the file today. See
#                                             HIST_REV below for why this shape
#                                             exists and what it is checked
#                                             against.
TOKEN_RE = re.compile(
    rf"`(?P<hist>(?:[\w./-]+/)?[\w.+-]+\.(?:{CXX_EXT}))@(?P<hrev>{HIST_REV}):(?P<hspec>{SPEC})`"
    rf"|`(?P<path>(?:[\w./-]+/)?[\w.+-]+\.(?:{CXX_EXT})):(?P<pspec>{SPEC})`"
    rf"|`(?P<rs>(?:[\w./-]+/)?[\w.+-]+\.rs):(?:{SPEC})`"
    rf"|`(?P<rebase>{CXX_EXT}):(?P<rspec>{SPEC})`"
    rf"|`:(?P<bare>{SPEC})`"
    rf"|`(?P<file>(?:[\w./-]+/)?[\w.+-]+\.(?:{CXX_EXT}|rs))`"
    rf"|`\.(?P<ext>{CXX_EXT})`"
)
# A bare `:NNN` inherits the last file named to its left, and that inheritance
# is only sound while the line has named exactly ONE coordinate system. It does
# not survive a switch. §292 measured the whole population: of 469 inherited
# citations, 287 sat on a line that had already named a port `.rs` file, and 17
# of those meant a file the inheritance did not give them -- 16 the port file,
# one a `.srdf` fixture. All 17 passed, because the file they were wrongly
# given was long enough to hold the line number.
#
# No rule recovers the referent from the text. Two rows of the same table, same
# schema, same shape, settle it: `doc/claim-audit/moveit-scene.md`'s
# `getTransforms` row cites `:260` meaning the port, and its
# `getCollisionDetectorName` row cites `:304` meaning upstream -- and BOTH sit
# inside the `.rs` range named earlier on their own line, so the one numeric
# signal available answers them identically and is wrong once either way. The
# `.srdf` case closes the other escape: its referent is in neither candidate,
# so even a perfect two-way chooser could not reach it. `check-shorthand-
# citations.py`'s header reaches the same verdict from its own three refuted
# rules -- the governing path is a discourse fact, not a lexical one.
#
# So the file is not inferred across a switch; it is required. The 287 were
# rewritten with their path spelled out (which is also what the sibling gate
# wants -- converting a shorthand is the thing its budget counts down), and
# this is the rule that keeps them that way. Inheritance still serves the 182
# citations on single-coordinate lines, where the only file named IS the one
# meant.
ANY_EXT_CITATION_RE = re.compile(
    rf"`(?:[\w./-]+/)?[\w.+-]+\.([A-Za-z][\w+]{{0,7}}):{SPEC}`"
)
KNOWN_CITED_EXTS = frozenset(CXX_EXT.split("|")) | {"rs"}


def foreign_switch(line):
    """The column of the first citation to a file in a coordinate system
    TOKEN_RE cannot track, or None.

    TOKEN_RE sees C++ and `.rs`. A `` `fixtures/panda.srdf:80` `` is neither,
    so it can neither arm a base nor clear one -- it passes through invisibly
    and the `:73-81` after it kept the `model.cpp` named two clauses earlier.
    Being unable to FOLLOW that file is fine; silently continuing the previous
    one across it is not."""
    for m in ANY_EXT_CITATION_RE.finditer(line):
        if m.group(1) not in KNOWN_CITED_EXTS:
            return m.start()
    return None


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

# ------------------------------------------------- content verification (rule 4)
#
# The third class, ranked between span-verified and bounds-only: the citing
# text's OWN backticked quotation of the code is at the cited line. This is
# `check-citation-drift.py`'s rule 4, ported rather than reinvented, because a
# second dialect of "what counts as a quotation" would make the two gates
# disagree about the same sentence.
#
# It exists because bounds-only is silence. §287.8 measured that directly: a
# 57-line insertion in `oracle.cpp` invalidated five citations, and the four
# that were bounds-only resolved, passed, and pointed at the wrong code --
# `oracle.cpp:7035-7040` landed mid doc-comment and the gate said OK. Anchoring
# is the only remedy this script had, and anchoring a sentence that does not
# name its symbol means rewriting the sentence to suit the gate. A quotation
# that is ALREADY in the sentence needs no rewrite at all.
#
# The constants are the sibling's, unchanged, and C++ needs no additions to
# them: every delimiter Rust reaches for here (`::`, `->`, `..`, `&`, `(`, `[`,
# `=`, `!`, `"`, `/`, `.`, `_`) is also a C++ delimiter, and the two that are
# idiomatically Rust (`..`, `->`) are harmless rather than wrong. The one
# C++ shape the borrowed floor under-serves is a bare template or type name --
# `` `CollisionRequest` `` carries no delimiter and does not qualify -- which is
# the same treatment Rust's `` `revolute` `` gets, and for the same reason: a
# single undelimited word lands somewhere in a 7000-line file by accident and
# would verify a citation against nothing.
# The three ranked classes, and the file that freezes which one each citation
# got. Without the freeze this whole section would be decoration: content
# verification can only ever say YES, so a citation that stops being
# content-verified just becomes bounds-only, both pass, and the run's visible
# change is two totals moving by one. That is the exact silence
# `check-citation-drift.py`'s own header describes, and it is why that gate
# carries `doc/citation-classes.txt`. This is the same mechanism for the same
# reason, over the upstream half of the corpus.
#
# HISTORICAL `path@rev:NNN` citations are recorded here too, classified against
# the blob at their own pinned revision. An earlier version left them out on the
# reasoning that nothing about HEAD can move a pinned citation, so the rows
# could not change. That is true and beside the point: what moves is the PIN.
# Re-pinning `oracle.cpp@3241bbab:6584` to a revision whose file merely happens
# to be long enough was measured to pass in silence -- in bounds, so no
# `unreadable-historical`, and outside the baseline, so no demotion either. The
# rows are what make a re-pin fail. Both halves were measured: re-pinning that
# citation retires its key and arrives undeclared (1 + 1 failures), and editing
# the quotation out of the citing sentence while leaving the pin alone demotes
# it content-verified -> bounds-only (1 failure).
SPAN_VERIFIED = "span-verified"
CONTENT_VERIFIED = "content-verified"
BOUNDS_ONLY = "bounds-only"
CLASS_RANK = {SPAN_VERIFIED: 3, CONTENT_VERIFIED: 2, BOUNDS_ONLY: 1}
UPSTREAM_CLASS_BASELINE = "doc/upstream-citation-classes.txt"


def upstream_class_header(head_sha, live, anchor_verified, content_verified,
                          bounds_only):
    """The `#` header `doc/upstream-citation-classes.txt` carries for this
    tree. A function rather than a literal at the write site because the read
    path calls it too -- see tools/ci/baseline_header.py."""
    return [
        "# Per-citation verification class for every upstream `path.cpp:NNN`",
        "# citation in every tracked .md/.rs file. The class says WHAT WAS",
        "# CHECKED, and the classes are ranked: span-verified (the cited lines",
        "# are inside the definition of the symbol the citing text names) >",
        "# content-verified (the citing text's own quotation of the code is at a",
        "# cited line) > bounds-only (only that the line number is inside the",
        "# file). Drift moves a citation DOWN that ladder, and without this file",
        "# a demotion is invisible -- two totals move by one and the run passes.",
        "#",
        "# Generated by: tools/ci/verify-upstream-citations.sh --write-classes",
        f"# Source commit: {head_sha}",
        f"# Citations: {sum(len(v) for v in live.values())} in {len(live)} distinct "
        "(document, citation) keys",
        f"#   span-verified: {anchor_verified}",
        f"#   content-verified: {content_verified}",
        f"#   bounds-only: {bounds_only}",
        "#",
        "# Historical `path@rev:NNN` citations ARE here, on this same ladder,",
        "# classified by the same `classify_citation` against the blob at their",
        "# own pinned revision. An earlier version of this header said they were",
        "# not, on the reasoning that nothing about HEAD can move a pinned",
        "# citation; that is true and beside the point, because what moves is the",
        "# PIN. A re-pin to a revision whose file merely happens to be long enough",
        "# is in bounds and outside the baseline, so only a row makes it fail.",
        "# Citations that FAIL (out-of-bounds, span-mismatch, obsolete-header,",
        "# unreadable-historical) are not here -- the run already fails",
        "# on them, so a row would only offer them a place to be recorded in.",
        "# Format: <citing document>\\t<citation>\\t<class>[*N] ...",
    ]


BASELINE_ROW_RE = re.compile(r"^([^\t]+)\t([^\t]+)\t(.+)$")
MIN_QUOTATION = 8
QUOTATION_DELIMS = ("(", ")", "::", "!", "=", "..", "->", "[", "]", '"', "&", ".", "/", "_")
BACKTICK_SPAN_RE = re.compile(r"`([^`\n]+)`")
# A quotation cannot itself be a citation or a bare path -- a pointer at code is
# not a quotation of it. Any extension, not just the C++ ones: an upstream
# citation sits next to a port citation constantly in these documents.
ANY_CITATION_RE = re.compile(r"^[\w./-]+\.\w+(?:@[0-9a-f]{7,40}\^{0,3})?:\d+(?:[-,·]\d+)*$")
BARE_PATH_RE = re.compile(r"^[\w./-]+\.\w+$")
# Markdown escapes a cell-splitting `|` inside a table; the source being quoted
# has the bare operator.
MD_ESCAPES = (("\\|", "|"), ("\\*", "*"), ("\\_", "_"), ("\\<", "<"), ("\\`", "`"))


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


def citing_context(doc_lines, index):
    """The text a citation's claim is made in, 0-based `index` being its own
    line. `check-citation-drift.py`'s rule, unchanged: a table row is its own
    context, because a neighbouring row is a different claim about a different
    site; prose is the whole blank-line-delimited paragraph, because these
    documents wrap at ~72 columns and a citation's subject lands on the line
    above as readily as on its own."""
    if doc_lines[index].lstrip().startswith("|"):
        return doc_lines[index]
    start = index
    while (
        start > 0
        and doc_lines[start - 1].strip()
        and not doc_lines[start - 1].lstrip().startswith("|")
    ):
        start -= 1
    end = index
    while (
        end + 1 < len(doc_lines)
        and doc_lines[end + 1].strip()
        and not doc_lines[end + 1].lstrip().startswith("|")
    ):
        end += 1
    return "\n".join(doc_lines[start : end + 1])


def quotations_near(context):
    """Every backtick span in `context` that is a quotation of code rather than
    a word of prose or a pointer: at least MIN_QUOTATION characters, carrying at
    least one QUOTATION_DELIMS delimiter, and not itself a citation, a bare
    path, or a commit SHA.

    `BACKTICK_SPAN_RE` forbids a newline inside the span, so a quotation the
    document wrapped across two source lines is invisible here. That is the
    sibling's blind spot and it is kept rather than fixed, because the two gates
    reading the same sentence differently is worse than either reading it
    narrowly; the fix belongs in the document, which is where §289 applied it."""
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


def content_anchor(quotations, file_lines, parts):
    """The first quotation of the citing text that occurs literally inside one
    of the lines the citation names, or None.

    Verification only. The inverse -- "this quotation is NOT at the cited
    line, so the citation is wrong" -- was implemented, measured, and removed:
    it produced 489 failures on this corpus, and reading them showed the
    premise is false. A paragraph quotes several things, and the ones that are
    not the cited line's own text land wherever they happen to occur; cited
    lines 271-272 with `` `joint_name` `` at 76 is a sentence mentioning an
    identifier, not drift. Text present elsewhere in a 7000-line file
    falsifies nothing.

    What DOES make this class fail is the same thing that makes the sibling's
    fail: the class is written down per citation in UPSTREAM_CLASS_BASELINE,
    and a citation that drops down the ladder is a demotion. See CLASS_RANK."""
    for lo, hi in parts:
        for n in range(lo, min(hi, len(file_lines)) + 1):
            for token in quotations:
                if token in file_lines[n - 1]:
                    return token
    return None


def render_classes(classes):
    """One tab-separated row per `(document, citation text)`, classes
    most-checked first. Keyed by the citation's TEXT, not by its line in the
    citing document, so inserting a paragraph above it is not a diff; the
    citation's own line NUMBER changing is what must not be absorbed, and
    changing it changes the key."""
    rows = []
    for (doc, spec), clss in sorted(classes.items()):
        counted = []
        for cls in dict.fromkeys(clss):
            k = clss.count(cls)
            counted.append(cls if k == 1 else f"{cls}*{k}")
        rows.append(f"{doc}\t{spec}\t{' '.join(counted)}")
    return rows


def parse_classes(text):
    """Inverse of `render_classes`. Returns `(map, n_malformed)`; a malformed
    row is counted rather than skipped, so a format change cannot quietly
    shrink the baseline to the rows that still happen to parse."""
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
            cls, _, k = tok.partition("*")
            if cls not in CLASS_RANK or (k and not k.isdigit()):
                malformed += 1
                break
            clss.extend([cls] * (int(k) if k else 1))
        else:
            out[(m.group(1), m.group(2))] = sorted(
                clss, key=lambda c: (-CLASS_RANK.get(c, 0), c)
            )
    return out, malformed


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


def why_bounds_only(line, start, end, spans, window_floor, parts, anchors):
    """Which of `find_anchors`' gates this citation fell through.

    A single "N bounds-checked only" figure says how much of the corpus is
    unverified but not what it would take to verify it, and those are
    different questions with very different answers. Measured here rather
    than guessed at, and printed on every run rather than behind a flag: the
    composition is the part that decides whether the number can move, and a
    breakdown nobody asks for is a breakdown nobody reads."""
    if len(parts) > 1:
        return "multi-part spec: enumerates sites, makes no containment claim"
    if anchors:
        return "anchored, but the range is not exactly a definition span"
    window = line[max(window_floor, start - LOCAL_WINDOW) : start]
    for sep in "|)":
        cut = window.rfind(sep)
        if cut != -1:
            window = window[cut + 1 :]
    names = [m.group(1) for m in IDENT_IN_BACKTICKS_RE.finditer(window)]
    if not names:
        return "no backticked name within 60 characters before the citation"
    tight = [
        m.group(1)
        for m in IDENT_IN_BACKTICKS_RE.finditer(window)
        if TIGHT_GAP_RE.match(window[m.end() :])
    ]
    if not tight:
        return "name is there but words separate it: prose about it, not a pointer at it"
    bare = [n.split("::")[-1] for n in tight]
    if any(n not in NOT_A_SYMBOL and not HEX_SHA_RE.match(n) and n not in spans for n in bare):
        return "tightly paired name has no definition span in the file (field, macro, alias, namespace)"
    return "tightly paired name is a stopword in NOT_A_SYMBOL"


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
    # The same contiguous run, entered from a definition that is NOT the named
    # one. `contiguous_run_end`'s own reason for walking every definition in
    # the file -- "an adjacent pair need not share a name ... and the citing
    # text names both" -- applies whichever of the pair the tight-gap rule
    # happened to reach, and it reaches only one: in
    # "(`planner_plugin_loader_`/`planner_map_`,
    # `planning_pipeline.hpp:262-263`)" the first name is separated from the
    # citation by the second, so only `planner_map_` (263) anchors, and the
    # range starts one line above its span. Every part of the claim is still
    # checked exactly -- both ends land on a definition's own boundary, the
    # gaps between are blank in the raw file, and the named symbol's
    # definition is one of the run's members -- so this admits a pair and not
    # a range that overshoots one. `getCostSources`'s `:2451-2490` still
    # fails: 2490 is no definition's last line.
    if not inside and any(s == lo for (s, _e) in all_file_spans):
        run = contiguous_run_end(
            [(s, e, "") for (s, e) in all_file_spans if s == lo], lo, file_lines, sorted_all
        )
        if run and hi in run and any(lo <= s and e <= hi for (s, e, _k) in span_list):
            return None
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
    # where the symbol is USED. `robot_state.cpp:1836-1866` is the branch in
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


def load_unresolvable_declarations():
    """{cited path: project} for every path that resolves against no root.

    Printing these and passing is the same silence a skipped check produces:
    an unresolvable citation is an unverified one, and a list of them under a
    green OK line reads as coverage. Both directions are failures instead --
    an undeclared path, so a typo or a renamed upstream file cannot quietly
    join the pile, and a declared path nothing cites any more, so this file
    cannot accumulate claims about a corpus that moved. Matching is on the
    cited string exactly as the corpus writes it, so two spellings of one file
    are two rows and shortening a citation cannot hide under a neighbour."""
    if not EXEMPTIONS_PATH.exists():
        return {}
    data = json.loads(EXEMPTIONS_PATH.read_text(encoding="utf-8"))
    out = {}
    for group in data.get("unresolvable", []):
        for path in group["paths"]:
            if path in out:
                raise SystemExit(
                    f"FAIL `{path}` is declared twice in {EXEMPTIONS_PATH.name} "
                    f"({out[path]} and {group['project']})"
                )
            out[path] = group["project"]
    return out


# `--deduplicate`, on both, and it is not cosmetic. During an unresolved
# merge `git ls-files` prints a conflicted path once per stage, so a
# three-way conflict in PORTING-PLAN.md makes this corpus read every
# citation in it three times: a run mid-merge reported 2728 citations
# across 349 files where the resolved tree has 1968 across 347. A corpus
# count is the only thing this gate publishes, so an inflated one is a
# wrong answer, not a slow one. On the upstream side a duplicate is worse
# still now that `source_index` rejects a repeated key -- the gate would
# die claiming two source roots collide when one root simply has a
# conflict.
def tracked_files():
    out = subprocess.run(
        ["git", "ls-files", "--deduplicate", "-z"], cwd=REPO_ROOT, capture_output=True, check=True
    ).stdout.decode("utf-8")
    return [p for p in out.split("\0") if p]


def upstream_tracked(upstream):
    out = subprocess.run(
        ["git", "ls-files", "--deduplicate", "-z"], cwd=upstream, capture_output=True, check=True
    ).stdout.decode("utf-8")
    return [p for p in out.split("\0") if p.endswith((".cpp", ".hpp", ".h", ".cc", ".cxx"))]


def source_index(roots):
    """`({path as this corpus writes it: file on disk}, {same key: (root, rel)})`
    across every pinned root.

    One namespace, not a primary root plus fallbacks: a fallback resolves
    only what the first root missed, so a basename that exists in both would
    silently answer with the first while reading as unique. Here every
    ambiguity is an ambiguity, reported like any other.

    The prefix is what makes the two namespaces disjoint. moveit2's files are
    cited by their upstream-relative path and carry no prefix; a vendored
    checkout's files are cited as `third_party/<pkg>/...`, which is where
    they sit in this repository, so that is the name they are indexed under.
    This repository itself is a root with no prefix and no vendored path: its
    own files are cited by their repo-relative path exactly as they sit here.

    The second map is what a HISTORICAL citation needs -- `git show
    <rev>:<rel>` has to run in the checkout the path resolved into, and
    reconstructing that from the on-disk path afterwards would guess at
    exactly the thing the index already knows.
    """
    index, origins = {}, {}
    for prefix, root in roots:
        for rel in upstream_tracked(root):
            key = prefix + rel
            if key in index:
                raise SystemExit(
                    f"FAIL two source roots both hold {key} -- one would silently "
                    f"win and every citation to it would read as resolved"
                )
            index[key] = Path(root) / rel
            origins[key] = (Path(root), rel)
    return index, origins


def blob_at(root, rev, rel):
    """The lines of `rel` at `rev` in the checkout at `root`, or None when that
    revision does not carry that path. None is a hard failure at the call site,
    not a skip: a historical citation whose revision cannot be read is a record
    nobody can check, which is the state this whole shape exists to leave."""
    out = subprocess.run(
        ["git", "show", f"{rev}:{rel}"], cwd=root, capture_output=True
    )
    if out.returncode != 0:
        return None
    body = out.stdout.decode("utf-8", errors="replace").split("\n")
    if body and body[-1] == "":
        body.pop()
    return body


def parse_source(arg):
    prefix, _, path = arg.partition("=")
    if not path:
        raise argparse.ArgumentTypeError(f"--source needs PREFIX=DIR, got {arg!r}")
    if prefix and not prefix.endswith("/"):
        prefix += "/"
    return (prefix, Path(path))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--upstream", required=True)
    ap.add_argument(
        "--source",
        action="append",
        default=[],
        type=parse_source,
        metavar="PREFIX=DIR",
        help="an additional pinned source root, indexed under PREFIX "
        "(repeatable); the wrapper passes one per vendored third_party/ package",
    )
    ap.add_argument(
        "--write-classes",
        action="store_true",
        help=f"regenerate {UPSTREAM_CLASS_BASELINE} from this run instead of "
        "checking against it; read the diff, because a demotion accepted there "
        "is a citation nobody is checking any more",
    )
    ap.add_argument(
        "--missing-source",
        action="append",
        default=[],
        metavar="PREFIX",
        help="a pinned root the wrapper knows about but did not pass via "
        "--source because the tree is absent (repeatable). Declares this "
        "run's source set incomplete: an unresolvable citation is then "
        "reported but not required to be declared in "
        f"{EXEMPTIONS_PATH.name}, since this run cannot tell a genuinely bad "
        "citation from one that only needed the missing root",
    )
    args = ap.parse_args()
    if args.write_classes and args.missing_source:
        print(
            "FAIL --write-classes and --missing-source were both given -- a "
            "freeze taken with a known-missing root is exactly the freeze "
            "`refuse_freeze` exists to refuse; the wrapper must not reach "
            "here with both set.",
            file=sys.stderr,
        )
        return 1
    upstream = Path(args.upstream)

    index, origins = source_index([("", upstream)] + args.source)
    upstream_set = set(index)
    by_basename = {}
    for p in index:
        by_basename.setdefault(p.rsplit("/", 1)[-1], []).append(p)

    text_cache, span_cache = {}, {}

    def lines_of(rel):
        if rel not in text_cache:
            body = index[rel].read_text(encoding="utf-8", errors="replace").split("\n")
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

    blob_cache = {}

    def historical_lines(resolved, rev):
        root, rel = origins[resolved]
        if (resolved, rev) not in blob_cache:
            blob_cache[(resolved, rev)] = blob_at(root, rev, rel)
        return blob_cache[(resolved, rev)]

    hist_span_cache = {}

    def historical_spans(resolved, rev, hist_lines):
        if (resolved, rev) not in hist_span_cache:
            hist_span_cache[(resolved, rev)] = symbol_spans(
                "\n".join(hist_lines),
                allow_decls=resolved.endswith((".hpp", ".h", ".hh")),
            )
        return hist_span_cache[(resolved, rev)]

    exemptions = load_exemptions()
    total = 0
    historical = 0
    historical_bad = []
    anchor_verified = 0
    content_verified = 0
    # ONE record per classified citation. Every count below is derived from
    # this list, so a class and the number reporting it cannot drift apart.
    classes = {}

    def classify(doc, spec, cls):
        # TOKEN_RE keeps the backticks a citation is written in; the sibling
        # gate's corpus is uniformly backticked so its key never carries them.
        # Strip here so both baselines spell a key the same way and the FAIL
        # message can wrap it in backticks without doubling them.
        classes.setdefault((doc, spec.strip("`")), []).append(cls)

    bounds_only = 0
    bounds_only_why = collections.Counter()
    inherited_checked = 0
    ambiguous_base = []
    out_of_bounds = []
    obsolete_header = []
    span_mismatch = []
    unresolved = {}
    exempted = 0
    # Every (doc, key) this run SAW but could not resolve to a class, keyed
    # exactly like `classify()` -- same (doc, token-with-backticks-stripped)
    # pair. This is what tells "the citation is gone from the document" (a
    # real retirement: the token is never seen at all, so it never reaches
    # `mark_unresolved` either) apart from "the citation is still there but
    # this run's source set could not resolve it" (it lands here instead of
    # in `classes`). Without this a resolution failure and a deleted
    # citation are the same shape at the baseline-comparison step below --
    # both are simply absent from `classes` -- which is the read-path half
    # of the defect `refuse_freeze` above closes on the write path.
    unresolved_citations = collections.Counter()

    def mark_unresolved(doc, token):
        unresolved_citations[(doc, token.strip("`"))] += 1

    def classify_citation(
        doc, doc_lines, line, line_no, start, end, window_floor, resolved, spec, token,
        file_lines, spans, parts
    ):
        """Put one in-bounds citation on the span / content / bounds ladder.

        One classifier, two callers: a live `path:NNN` runs it against HEAD's
        file, a historical `path@rev:NNN` against the blob at its own pinned
        revision. Copying the ladder for the second caller was the alternative,
        and a copy is what lets the two drift into different dialects of
        "verified" -- the divergence this whole family of scripts reports on.

        Returns a `span_mismatch` tuple for the caller to record, or `None`
        when the citation was classified (and already recorded)."""
        nonlocal anchor_verified, content_verified, bounds_only

        anchors = find_anchors(line, start, end, spans, window_floor)
        # A comma list enumerates SITES, not a span: the four lines of
        # `collision_env_hybrid.cpp:49,61,69,169` are three
        # constructor base-init calls plus one `setWorld` call, and
        # the name they are all paired with is what they are about.
        # There is no containment claim to check, so these are
        # bounds-checked only -- which is what caught
        # `world.cpp:220,326,650,655` anyway.
        #
        # `check-citation-drift.py`'s rule 0 does adjudicate a comma list, so
        # the port of it tried that too and the trial is what settled this:
        # running `part_verdict` over every element produced five failures and
        # four were correct citations. `getCurvature`'s
        # `time_optimal_trajectory_generation.cpp:321,327,333` is the
        # `getPathSegment` call site in each of THREE functions, and the
        # sentence names all three; `asyncExecute`'s
        # `move_group_interface.hpp:732,741,750,759` is two `asyncExecute`
        # declarations and two `execute` ones, and the sentence names both;
        # `setPlannerConfigurations`'s `planning_interface.hpp:56-72,193` is
        # the settings struct plus the method that takes it. The adjacent name
        # in a comma list is the SUBJECT, and only a single-range citation
        # makes the containment claim this rule can check.
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
                    return (
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
                anchor_verified += 1
                classify(doc, token, SPAN_VERIFIED)
                return None
        if not anchors or len(parts) > 1:
            # No symbol to check containment against -- but the sentence
            # may already quote the code, which is a claim about the
            # cited lines that needs no anchor and no rewrite.
            if content_anchor(
                quotations_near(citing_context(doc_lines, line_no - 1)),
                file_lines,
                parts,
            ):
                content_verified += 1
                classify(doc, token, CONTENT_VERIFIED)
                return None
            bounds_only += 1
            classify(doc, token, BOUNDS_ONLY)
            bounds_only_why[
                why_bounds_only(line, start, end, spans, window_floor, parts, anchors)
            ] += 1
            return None
        span_list = all_spans(spans, anchors)
        every_span = {(s, e) for v in spans.values() for (s, e, _k) in v}
        reasons = [
            f"{lo}-{hi}: {r}" if lo != hi else f"{lo}: {r}"
            for lo, hi in parts
            if (r := part_verdict(lo, hi, span_list, file_lines, anchors, every_span))
            is not None
        ]
        if reasons:
            return (doc, line_no, resolved, spec, anchors, reasons, span_list)
        anchor_verified += 1
        classify(doc, token, SPAN_VERIFIED)
        return None

    corpus = [p for p in tracked_files() if p.endswith((".md", ".rs"))]
    for doc in corpus:
        doc_lines = (REPO_ROOT / doc).read_text(encoding="utf-8", errors="replace").split("\n")
        for line_no, line in enumerate(doc_lines, 1):
            base = None  # the file a bare `:NNN` on this line would inherit
            # The column past which this line no longer has one coordinate
            # system, so inheritance stops meaning anything. See `foreign_switch`.
            switch_at = foreign_switch(line)
            prev_end = 0
            for m in TOKEN_RE.finditer(line):
                start, end = m.start(), m.end()
                window_floor, prev_end = prev_end, end
                g = m.groupdict()

                if g["hist"] is not None:
                    # A historical citation lends its file to nothing: a bare
                    # `:NNN` after it would be a live claim inheriting a
                    # revision-pinned one, which is the confusion this shape
                    # exists to remove.
                    base = None
                    cands = resolve_path(g["hist"], upstream_set, by_basename)
                    if len(cands) != 1:
                        unresolved.setdefault(
                            g["hist"],
                            "no upstream file matches" if not cands else f"ambiguous: {cands}",
                        )
                        mark_unresolved(doc, m.group(0))
                        continue
                    resolved, rev = cands[0], g["hrev"]
                    hist_lines = historical_lines(resolved, rev)
                    if hist_lines is None:
                        historical_bad.append(
                            (doc, line_no, resolved, rev, g["hspec"], None)
                        )
                        continue
                    oob = [
                        x
                        for lo, hi in parse_parts(g["hspec"])
                        for x in (lo, hi)
                        if not 1 <= x <= len(hist_lines)
                    ]
                    if oob:
                        historical_bad.append(
                            (doc, line_no, resolved, rev, g["hspec"], len(hist_lines))
                        )
                        continue
                    # Bounds at the pinned revision is the weakest thing that
                    # could be said about a citation, and it is the thing this
                    # script exists to stop calling verification: re-pin a
                    # citation to any revision whose file is merely long enough
                    # and an in-bounds number reads as checked. So a historical
                    # citation goes on the same ladder as a live one, computed
                    # against its own revision, and lands in the class baseline
                    # -- where a re-pin retires the old key and a re-pin that
                    # loses the quotation demotes the class. Both are failures.
                    historical += 1
                    total += 1
                    mismatch = classify_citation(
                        doc, doc_lines, line, line_no, start, end, window_floor,
                        f"{resolved}@{rev}", g["hspec"], m.group(0), hist_lines,
                        historical_spans(resolved, rev, hist_lines),
                        parse_parts(g["hspec"]),
                    )
                    if mismatch is not None:
                        span_mismatch.append(mismatch)
                    continue
                if g["rs"] is not None:
                    base = None
                    switch_at = start if switch_at is None else min(switch_at, start)
                    continue
                if g["file"] is not None:
                    if g["file"].endswith(".rs"):
                        base = None
                        switch_at = start if switch_at is None else min(switch_at, start)
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
                        mark_unresolved(doc, m.group(0))
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
                        # `base` itself is unresolved (the file this rebase
                        # switches from never resolved), or no sibling
                        # translation unit exists for it. Either way this
                        # citation is unverifiable this run, not gone --
                        # see `mark_unresolved`.
                        mark_unresolved(doc, m.group(0))
                        continue
                else:
                    spec = g["bare"]
                    if base is None:
                        # Inherits a file that never resolved this run
                        # (`g["file"]`/`g["path"]` above already fell
                        # through). Same "unverifiable, not gone" case.
                        mark_unresolved(doc, m.group(0))
                        continue
                    if switch_at is not None and switch_at < start:
                        ambiguous_base.append((doc, line_no, m.group(0), base))
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

                mismatch = classify_citation(
                    doc, doc_lines, line, line_no, start, end, window_floor,
                    resolved, spec, m.group(0), file_lines, spans_of(resolved), parts
                )
                if mismatch is not None:
                    span_mismatch.append(mismatch)

    if total == 0:
        print(
            "FAIL parsed zero upstream `path.cpp:NNN` citations across tracked "
            ".md/.rs files -- the citation grammar changed and this checked nothing",
            file=sys.stderr,
        )
        return 1

    live = {k: sorted(v, key=lambda c: (-CLASS_RANK[c], c)) for k, v in classes.items()}
    if anchor_verified + content_verified + bounds_only != sum(len(v) for v in live.values()):
        # A classified citation that never reached `classify` would sit outside
        # the baseline for ever, unwatched, while the totals still looked right.
        print(
            f"FAIL {anchor_verified + content_verified + bounds_only} citations were "
            f"classified but {sum(len(v) for v in live.values())} were recorded -- a "
            f"classification path does not write its class",
            file=sys.stderr,
        )
        return 1

    if args.write_classes:
        head_sha = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, capture_output=True, text=True
        ).stdout.strip()
        header = upstream_class_header(head_sha, live, anchor_verified,
                                       content_verified, bounds_only)
        (REPO_ROOT / UPSTREAM_CLASS_BASELINE).write_text(
            "\n".join(header + render_classes(live)) + "\n"
        )
        print(
            f"wrote {UPSTREAM_CLASS_BASELINE}: "
            f"{sum(len(v) for v in live.values())} citations, {len(live)} keys"
        )
        return 0

    if ambiguous_base:
        print(
            f"--- {len(ambiguous_base)} bare citation(s) after a coordinate "
            f"switch ---",
            file=sys.stderr,
        )
        for doc, line_no, token, would_be in ambiguous_base:
            print(
                f"FAIL {doc}:{line_no}: {token} follows a citation to a different "
                f"file's line numbering on this line, so which file it means is a "
                f"discourse fact this gate cannot read. It would have inherited "
                f"{would_be}; §292 measured 17 such citations that meant something "
                f"else and passed anyway. Write the path: `{would_be.rsplit('/', 1)[-1]}"
                f":{token.strip('`').lstrip(':')}`, or the port file if that is what "
                f"is meant",
                file=sys.stderr,
            )

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

    if historical_bad:
        print(
            f"--- {len(historical_bad)} unreadable historical citation(s) ---",
            file=sys.stderr,
        )
        for doc, line_no, resolved, rev, spec, n_lines in historical_bad:
            print(
                f"FAIL {doc}:{line_no}: cites {resolved}@{rev}:{spec}, but "
                + (
                    f"`git show {rev}:{resolved}` does not resolve -- the pinned "
                    f"revision is unreachable or did not carry that path"
                    if n_lines is None
                    else f"that file had only {n_lines} lines at {rev}"
                ),
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

    declared = load_unresolvable_declarations()
    undeclared = sorted(set(unresolved) - set(declared))
    stale = sorted(set(declared) - set(unresolved))

    if sum(bounds_only_why.values()) != bounds_only:
        # A classifier that silently drops a case reports a smaller blind spot
        # than the one that exists, which is the direction that reads as
        # progress. The buckets are exhaustive by construction or this fails.
        print(
            f"FAIL bounds-only breakdown sums to {sum(bounds_only_why.values())}, "
            f"not {bounds_only} -- `why_bounds_only` grew a case it does not name",
            file=sys.stderr,
        )
        return 1

    if bounds_only_why:
        print(
            f"--- the {bounds_only} bounds-checked-only citation(s), by what "
            f"would be needed to verify them ---",
            file=sys.stderr,
        )
        for reason, n in bounds_only_why.most_common():
            print(f"  {n:5d}  {reason}", file=sys.stderr)

    if unresolved:
        by_project = {}
        for p in unresolved:
            by_project.setdefault(declared.get(p, "<undeclared>"), []).append(p)
        print(
            f"--- {len(unresolved)} distinct unresolvable cited path(s) across "
            f"{len(by_project)} project(s) ---",
            file=sys.stderr,
        )
        for project in sorted(by_project):
            print(f"  {project}:", file=sys.stderr)
            for p in sorted(by_project[project]):
                print(f"    `{p}` -- {unresolved[p]}", file=sys.stderr)

    source_incomplete = bool(args.missing_source)
    if undeclared and source_incomplete:
        print(
            f"--- {len(undeclared)} unresolvable cited path(s) NOT checked "
            f"against {EXEMPTIONS_PATH.name} -- this run is missing "
            f"{', '.join(sorted(args.missing_source))}, so an unresolved path "
            f"may only be unreachable this run, not genuinely undeclared. "
            f"Declaring it here would be a lie the next full-source run "
            f"believes; rerun with the full pinned source set to check "
            f"these ---",
            file=sys.stderr,
        )
        for p in undeclared:
            print(f"  `{p}` -- {unresolved[p]}", file=sys.stderr)
    elif undeclared:
        print(
            f"--- {len(undeclared)} undeclared unresolvable cited path(s) ---",
            file=sys.stderr,
        )
        for p in undeclared:
            print(
                f"FAIL `{p}` resolves against no root and is not declared in "
                f"{EXEMPTIONS_PATH.name}. Every citation to it is unverified; "
                f"declare which project it names and why no tree covers it, or "
                f"pass that tree as a `--source` root.",
                file=sys.stderr,
            )
    if stale:
        print(f"--- {len(stale)} stale unresolvable declaration(s) ---", file=sys.stderr)
        for p in stale:
            print(
                f"FAIL `{p}` is declared unresolvable in {EXEMPTIONS_PATH.name} "
                f"({declared[p]}) but nothing cites it any more -- the declaration "
                f"now vouches for a corpus that moved.",
                file=sys.stderr,
            )

    baseline_path = REPO_ROOT / UPSTREAM_CLASS_BASELINE
    if not baseline_path.exists():
        print(
            f"FAIL {UPSTREAM_CLASS_BASELINE} is missing -- every citation's class is "
            f"unchecked, so a content-verified citation can silently become "
            f"bounds-only. Regenerate it with --write-classes.",
            file=sys.stderr,
        )
        return 1
    baseline_text = baseline_path.read_text(encoding="utf-8")
    # The rows are checked against a fresh derivation below; the header
    # describing them was checked by nothing until this line.
    header_failed = baseline_header.report(
        UPSTREAM_CLASS_BASELINE, baseline_text,
        upstream_class_header("-", live, anchor_verified, content_verified,
                              bounds_only),
        "tools/ci/verify-upstream-citations.sh --write-classes", sys.stderr)
    baseline, malformed = parse_classes(baseline_text)
    if malformed:
        print(
            f"FAIL {UPSTREAM_CLASS_BASELINE} parsed {len(baseline)} rows and "
            f"{malformed} malformed one(s) -- a baseline that half-parses vouches "
            f"for the half it read",
            file=sys.stderr,
        )
        return 1

    def rank(cs):
        return sorted((CLASS_RANK[c] for c in cs), reverse=True)

    # A key this run could not resolve at all MUST NOT be compared against
    # the baseline: `live` has no entry for it (see `mark_unresolved`), so it
    # reads exactly like a citation the document deleted, and demanding a
    # `--write-classes` refreeze on that basis is how an absent
    # `third_party/<pkg>` checkout turns into a drift-shaped failure. A key
    # whose (doc, token) never reached `classify` OR `mark_unresolved` this
    # run is the genuine case -- the document no longer contains the token at
    # all -- and that one still fails below, unchanged.
    unresolved_keys = set(unresolved_citations)

    demoted, promoted, recounted, excluded_baseline = [], [], [], []
    for key in sorted(set(baseline) & (set(live) | unresolved_keys)):
        was = baseline[key]
        if key in unresolved_keys:
            now = live.get(key)
            if now is None or now != was:
                excluded_baseline.append((key, was, now))
            continue
        now = live[key]
        if was == now:
            continue
        if len(was) != len(now):
            recounted.append((key, was, now))
        elif rank(now) < rank(was):
            demoted.append((key, was, now))
        else:
            promoted.append((key, was, now))
    undeclared_cls = sorted(set(live) - set(baseline))
    retired_cls = sorted(set(baseline) - set(live) - unresolved_keys)

    print(
        f"--- {len(demoted)} demoted, {len(recounted)} recounted, "
        f"{len(undeclared_cls)} undeclared, {len(retired_cls)} retired, "
        f"{len(promoted)} promoted, {len(excluded_baseline)} excluded (source "
        f"absent this run) vs {UPSTREAM_CLASS_BASELINE} ---",
        file=sys.stderr,
    )
    if excluded_baseline:
        print(
            f"--- {len(excluded_baseline)} citation(s) NOT compared against "
            f"{UPSTREAM_CLASS_BASELINE}: at least one occurrence failed to "
            f"resolve this run, which absence and drift both look like from "
            f"here -- rerun with the full pinned source set to check them ---",
            file=sys.stderr,
        )
        for (doc, spec), was, now in sorted(excluded_baseline):
            now_desc = f"now {' '.join(now)}" if now else "now unresolved"
            print(f"  {doc}: `{spec}` was {' '.join(was)}, {now_desc}", file=sys.stderr)
    for (doc, spec), was, now in demoted:
        print(
            f"FAIL {doc}: `{spec}` was {' '.join(was)}, now {' '.join(now)} -- the "
            f"citation dropped down the ladder; nothing is checked about it now "
            f"that was checked before, which is what drift looks like from here",
            file=sys.stderr,
        )
    for (doc, spec), was, now in recounted:
        print(
            f"FAIL {doc}: `{spec}` occurred {len(was)}x ({' '.join(was)}), now "
            f"{len(now)}x ({' '.join(now)}) -- the document gained or lost an "
            f"occurrence of this exact citation",
            file=sys.stderr,
        )
    for doc, spec in undeclared_cls:
        print(
            f"FAIL {doc}: `{spec}` ({' '.join(live[(doc, spec)])}) is not in "
            f"{UPSTREAM_CLASS_BASELINE} -- a citation whose line number changed "
            f"retires its old key and arrives as a new one, so an undeclared "
            f"citation is how a shift shows up",
            file=sys.stderr,
        )
    for doc, spec in retired_cls:
        print(
            f"FAIL {doc}: `{spec}` ({' '.join(baseline[(doc, spec)])}) is in "
            f"{UPSTREAM_CLASS_BASELINE} but no longer in the document",
            file=sys.stderr,
        )
    class_drift = bool(demoted or recounted or undeclared_cls or retired_cls
                       or header_failed)
    if class_drift:
        print(
            f"FAIL regenerate with tools/ci/verify-upstream-citations.sh "
            f"--write-classes and read the diff: a demotion accepted there is a "
            f"citation nobody is checking any more",
            file=sys.stderr,
        )

    # `undeclared` is only a hard failure with the full pinned source set --
    # see the `source_incomplete` block above. `n_source_absent_unresolved`
    # is the paths that clause excused, reported here so the FAIL summary's
    # own total accounts for every unresolved path, not just the ones it
    # chose to fail on.
    n_undeclared_fail = 0 if source_incomplete else len(undeclared)
    n_source_absent_unresolved = len(undeclared) if source_incomplete else 0

    if (
        out_of_bounds
        or span_mismatch
        or obsolete_header
        or (undeclared and not source_incomplete)
        or stale
        or historical_bad
        or class_drift
        or ambiguous_base
    ):
        print(
            f"FAIL {len(out_of_bounds)} out-of-bounds + {len(obsolete_header)} "
            f"obsolete-header + {len(span_mismatch)} span-mismatch + "
            f"{n_undeclared_fail} undeclared-unresolvable + {len(stale)} "
            f"stale-declaration + {len(historical_bad)} unreadable-historical + "
            f"{len(demoted)} demoted + {len(recounted)} recounted + "
            f"{len(undeclared_cls)} undeclared-class + {len(retired_cls)} retired-class "
            f"+ {len(ambiguous_base)} ambiguous-base "
            f"(of {total} upstream citations resolved across "
            f"{len(corpus)} tracked .md/.rs files; {len(excluded_baseline)} citation(s) "
            f"and {n_source_absent_unresolved} unresolvable path(s) excluded from "
            f"comparison, source absent this run)",
            file=sys.stderr,
        )
        return 1

    # An empty prefix is this repository indexing itself; naming it by the
    # prefix would print an empty string, which reads as a root that failed to
    # load rather than as the one whose files are cited by repo-relative path.
    roots = f"{upstream}" + (
        f" + {len(args.source)} extra root(s) "
        f"({', '.join(p.rstrip('/') or str(root) for p, root in args.source)})"
        if args.source
        else " (no extra root passed)"
    )
    n_declared_cited = len(unresolved) - len(undeclared)
    unresolved_desc = (
        f"{len(unresolved)} distinct unresolvable paths all declared in "
        f"{EXEMPTIONS_PATH.name} (reported above, and unverified -- a "
        f"declaration says no tree covers them, not that they are right)"
        if not source_incomplete
        else (
            f"{len(unresolved)} distinct unresolvable paths -- {n_declared_cited} "
            f"declared in {EXEMPTIONS_PATH.name}, {len(undeclared)} excused because "
            f"{', '.join(sorted(args.missing_source))} (absent this run) could "
            f"cover them (reported above, and unverified either way)"
        )
    )
    excluded_desc = (
        f", {len(excluded_baseline)} citation(s) excluded from the "
        f"{UPSTREAM_CLASS_BASELINE} comparison because source absent this run left "
        f"them unresolvable (reported above)"
        if excluded_baseline
        else ""
    )
    print(
        f"OK {total} upstream citations across {len(corpus)} tracked .md/.rs files "
        f"against {roots}: {anchor_verified} span-verified (cited lines inside "
        f"the named symbol's definition), {content_verified} content-verified "
        f"(the citing text's own quotation of the code is at a cited line), "
        f"{bounds_only} bounds-checked only (no "
        f"tightly-paired symbol with a definition span, and no quotation "
        f"either), {exempted} exempted "
        f"(tools/ci/upstream-citation-exemptions.json), {inherited_checked} of the "
        f"total reached through a bare `:NNN` continuation on a line that named "
        f"one coordinate system (a continuation across a switch is a hard failure, "
        f"not an inference -- see `foreign_switch`), {unresolved_desc}, "
        f"{historical} of the total are historical `path@rev:NNN` citation(s), "
        f"put on the same ladder against their own pinned revision instead of "
        f"against HEAD, "
        f"0 out-of-bounds, 0 obsolete-header, 0 span-mismatch{excluded_desc}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
