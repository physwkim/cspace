#!/usr/bin/env bash
# Fails if a source file declares a permissive SPDX identifier while citing an
# upstream file that is copyleft.
#
# `check-license-matches-upstream.sh` compares a crate's manifest license
# against the SPDX identifier its own files carry. Both can agree and both be
# wrong: the identifier is what someone typed, not what the cited upstream
# actually permits. That gap is not hypothetical. `moveit-kinematics`'s
# `lib.rs`, `newton_raphson.rs` and `velocity.rs` each say BSD-3-Clause while
# citing `moveit_kinematics/kdl_kinematics_plugin/.../
# chainiksolver_vel_mimic_svd.{hpp,cpp}`, which is KDL vendored into moveit2
# with its LGPL-2.1-or-later header intact -- and `velocity.rs`'s own doc
# comments describe the port as "exactly as upstream's `result = vel1 +
# multiplier * vel2` accumulation does".
#
# What made that invisible was an assumption, not an oversight: the D11 audit
# enumerated files derived from `third_party/orocos_kinematics_dynamics/`, and
# these cite a `moveit_kinematics/...` path. **An upstream repository being
# BSD does not make every file in it BSD.** So the rule here is derived per
# *file*, from the licence text in the cited file itself -- never from the
# repository it sits in, and never from a table this script would have to be
# taught about each new upstream.
#
# Deliberately NOT named `check-*.sh`: it reads the upstream checkouts
# (`$MOVEIT2_SRC` and friends, plus the gitignored `third_party/` trees), which
# are outside this repository and absent from a CI runner, exactly like
# `verify-fixture-provenance.sh`'s vendored tree. A script that always skipped
# there would read as coverage while providing none.
#
# `geometric_shapes`, `srdfdom` and `octomap` had to be fetched into
# `third_party/` for this check to open them at all. Their ports cite exact
# versions and record how the sources were obtained -- `shapes.rs` even pins
# geometric_shapes to commit `192801ce`, which the fetched tag `2.3.3` resolves
# to -- but the trees themselves were transient, so nothing on disk could
# re-verify the claim. A provenance record that cannot be re-opened is a record
# of what someone did once, not a property of the tree.
#
# An unresolved citation is a failure, not a skip: a header naming an upstream
# file this script cannot open is precisely the case where nobody has checked
# the licence, so reporting it as clean would be the failure mode this gate
# exists to close.
#
# The second rule here (UNJUSTIFIED) exists because the first one reads only
# citation *paths*, and a licence can enter a file without any path being
# named. `cart_to_jnt.rs` carried `Copyright (c) 2013, Sachin Chitta, Willow
# Garage` -- verbatim the copyright line of the LGPL-2.1-or-later
# `chainiksolver_vel_mimic_svd.{hpp,cpp}`, and of no other file in that
# upstream directory -- while citing only the BSD `kdl_kinematics_plugin.cpp`,
# whose own copyright is `2012, Willow Garage, Inc.` and which names Sachin
# Chitta as author only, in a comment. Nothing above could see it: there was no
# LGPL path in the header to open. A human reading the file found it, which is
# not a property of the tree.
#
# So every non-`moveit-rs contributors` copyright line a file asserts must be
# reproduced by a copyright line in one of that file's own cited files, on both
# year and holder. Attribution is a claim about provenance exactly like a
# citation is, and it is checkable against the same sources already on disk.
# This catches the leaked-holder case above, and equally a year that drifted
# from the file it was copied out of -- both are one defect, a copyright line
# no cited source supports.
#
# Matching is exact rather than fuzzy on purpose. A notice that does not
# reproduce its source's wording is worth a report even when the intent is
# obvious: the fix is to make the two agree, and a matcher lenient enough to
# accept a reworded holder is lenient enough to accept a wrong one.
#
# The third rule (UNRETAINED) is that one's mirror, and the one with a licence
# clause behind it rather than a provenance argument: BSD-3-Clause and
# Apache-2.0 both require a derivative work to retain its source's copyright
# notice. Asserting a notice nothing supports is a provenance error; dropping
# one the licence obliges you to carry is a compliance error, and this tree had
# 36 of the latter -- including `joint/planar.rs` and `joint/revolute.rs`
# needing exactly the `2013, Ioan A. Sucan` line that four other files assert
# without having ported from him.
#
# It applies only where the header says `Ported from`, never `Used by`: the
# second names the upstream caller of something, which creates no derivative
# work. `velocity.rs` is the case that makes the distinction load-bearing --
# what it cites is the call site that *would* have used the LGPL solver it
# deliberately did not port.
#
#   tools/ci/verify-upstream-license-provenance.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"
STOMP_SRC="${STOMP_SRC:-$HOME/work/stomp}"
FCL_SRC="${FCL_SRC:-$HOME/work/fcl}"
LIBCCD_SRC="${LIBCCD_SRC:-$HOME/work/libccd}"
# `geometric_shapes` and `srdfdom` are cited by their clone-directory name, so
# the root that resolves them is `third_party/` itself. octomap's citations are
# package-relative (`include/octomap/...`), and its repository nests the package
# one level down, so it needs a root of its own. Both are the same gitignored
# external-checkout arrangement `verify-fixture-provenance.sh` already depends
# on, pinned to the versions the citing headers name: geometric_shapes 2.3.3,
# srdfdom 2.0.8, octomap 1.9.7.
THIRD_PARTY_SRC="${THIRD_PARTY_SRC:-$REPO_ROOT/third_party}"
OCTOMAP_SRC="${OCTOMAP_SRC:-$THIRD_PARTY_SRC/octomap/octomap}"

for src in "$MOVEIT2_SRC" "$STOMP_SRC" "$FCL_SRC" "$LIBCCD_SRC" "$THIRD_PARTY_SRC" "$OCTOMAP_SRC"; do
  if [[ ! -d "$src" ]]; then
    echo "$src is absent -- this check needs the upstream checkouts it compares against" >&2
    exit 1
  fi
done

python3 - "$REPO_ROOT" "$MOVEIT2_SRC" "$STOMP_SRC" "$FCL_SRC" "$LIBCCD_SRC" \
  "$THIRD_PARTY_SRC" "$OCTOMAP_SRC" <<'PYEOF'
import os
import re
import subprocess
import sys

repo_root, *upstream_roots = sys.argv[1:]
# The repo itself last: a few headers cite vendored sources under third_party/.
search_roots = upstream_roots + [repo_root]

# A citation is an indented path on its own line inside the leading comment
# block, which is the shape every "Ported from"/"Used by" header in this tree
# uses. Anything else in the header is prose and is not a claim about a file.
#
# Two forms occur. Either the whole repo-relative path sits on one line, or a
# directory line ending in `/` is followed by more-indented bare filenames --
# `moveit-planning/src/lib.rs` cites nine adapters that way. Reading the second
# form as nine bare filenames finds none of them, which reports as "cannot
# open" rather than as a licence anyone checked.
#
# A path may be followed by a parenthetical naming which symbols were taken
# (`utils.cpp (resolveConstraintFrames, cpp:623-675)`). Requiring the path to
# be the entire line dropped 24 such citations across the tree -- silently, and
# in the same direction as every other failure this gate is written against: it
# reported them as nothing to check rather than as unchecked. The closing paren
# is not required, because a few of these run onto the following line.
CITATION = re.compile(r"^//(\s{2,})(\S+?)(?:\s+\(.*)?\s*$")
# `.hxx` is octomap's extension for its header-inlined template bodies, which
# is where `OcTreeIterator.hxx` -- the whole subject of `moveit-octomap`'s
# `iter.rs` -- lives.
FILENAME = re.compile(r"\.(?:cpp|hpp|hxx|h|cc|cxx|c|py)$")
SPDX = re.compile(r"^//\s*SPDX-License-Identifier:\s*(.+?)\s*$")
# Paths after this marker are the opposite of a citation: they name what was
# read and deliberately left unported, so their licence cannot reach this file.
#
# The marker has to be the line that *introduces* such a list, so it is pinned
# to the shape the two files using it share: a `//` header line (not a `//!` or
# `///` doc line) whose last words are `not ported:`. Matching the phrase
# anywhere instead made prose end the citation list -- `key.rs` explains "what
# was and was not ported" one line above its only citation, and that citation
# was never opened.
NOT_PORTED = re.compile(r"^//(?![!/])[^\n]*\bnot ported:\s*$", re.I)
# Matched against the cited file's own header, not against a repo-level LICENSE.
COPYLEFT = re.compile(r"GNU Lesser General Public|GNU General Public|LGPL|GPL-", re.I)
# Identifiers this workspace uses that a copyleft upstream cannot support.
PERMISSIVE = re.compile(r"^(BSD-|MIT|Apache-)", re.I)
# A copyright line in either tree. The year is what anchors the match: without
# requiring it, `@copyright Copyright (c) 2016, ...` -- how the stomp headers
# write it -- matches at the first `Copyright` and yields the second one as the
# holder. Upstream also writes `Copyright  (C)  2013  Sachin Chitta, Willow
# Garage`, and this tree writes `Copyright (c) 2011-2012, Georgia Tech Research
# Corporation`; all three reduce to the same (year, holder) pairs below.
COPYRIGHT = re.compile(
    r"Copyright\s*(?:\(c\)|©)?\s*((?:\d{4})[\d\s,–-]*)[,\s]+(\S[^\n]*?)\s*$",
    re.I,
)
# An *assertion* is a copyright line in this tree's licence header, which is
# `//`-prefixed and starts with the word. A `//!` doc line quoting an upstream
# header -- `velocity.rs` and `lib.rs` both explain which LGPL notice the file
# deliberately does not carry -- is a description of someone else's notice, and
# reading it as this file's own claim inverts what those paragraphs say.
ASSERTION = re.compile(r"^//\s*Copyright\b", re.I)
# This tree's own line, which by construction has no upstream to justify it.
OURS = re.compile(r"moveit-rs contributors", re.I)
# The two verbs a header uses to introduce citations, and the reason the
# retention rule below can tell them apart. `Ported from` means this file is a
# derivative work of what follows; `Used by` means the opposite direction --
# `velocity.rs` cites the call site that *would* have used the LGPL solver it
# deliberately did not port. Only a derivation has a notice to retain.
DERIVATION = re.compile(r"^//\s*(Ported|Derived|Adapted)\b", re.I)
REFERENCE = re.compile(r"^//\s*Used by\b", re.I)


def copyright_claims(lines, anchored=False):
    """(year, normalized-holder) pairs a comment block's lines carry.

    `anchored` restricts to lines that assert a copyright rather than mention
    one; upstream files are read unanchored, since every line there is theirs.
    """
    claims = set()
    for line in lines:
        if OURS.search(line):
            continue
        if anchored and not ASSERTION.match(line):
            continue
        match = COPYRIGHT.search(line)
        if not match:
            continue
        # Fold to the shape the two trees share: case, punctuation, and the
        # corporate suffix upstream applies inconsistently to one holder
        # (`Willow Garage, Inc.` in one file, `Willow Garage` in the next).
        holder = re.sub(r"[^a-z0-9 ]", " ", match.group(2).lower())
        holder = re.sub(r"\s+", " ", holder).strip()
        holder = re.sub(r"\s+(inc|llc|ltd|gmbh|corp|corporation)$", "", holder)
        if not holder:
            continue
        for year in re.findall(r"\d{4}", match.group(1)):
            claims.add((year, holder))
    return claims

BRACE = re.compile(r"^([^{}]*)\{([^{}]+)\}(.*)$")


def expand_braces(token):
    """`dir/{a/b.hpp,c.cpp}` is two citations written as one path."""
    match = BRACE.match(token)
    if not match:
        return [token]
    head, body, tail = match.groups()
    return [
        expanded
        for part in body.split(",")
        for expanded in expand_braces(head + part.strip() + tail)
    ]


def resolve_citation(citation):
    """Every upstream file a citation names, or `[]` if it names none.

    One rule for both shapes a citation takes. A path names one file. A
    directory with no filenames indented under it -- how `moveit-planners-pilz`
    and `moveit-planners-stomp` cite the packages they port whole -- names the
    sources beneath it, and dropping it silently was the same failure as any
    other unopened citation.
    """
    for root in search_roots:
        candidate = os.path.join(root, citation)
        if os.path.isfile(candidate):
            return [candidate]
        if citation.endswith("/") and os.path.isdir(candidate):
            found = [
                os.path.join(where, name)
                for where, _, names in os.walk(candidate)
                for name in names
                if FILENAME.search(name)
            ]
            if found:
                return sorted(found)
    return []


tracked = [
    path
    for path in subprocess.run(
        ["git", "ls-files", "--", "crates/", "tools/", "ros/"],
        cwd=repo_root, capture_output=True, text=True, check=True,
    ).stdout.split()
    if path.endswith(".rs")
]

def header_of(path):
    """The leading `//` comment block, blank lines tolerated inside it."""
    out = []
    with open(os.path.join(repo_root, path), encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.rstrip("\n")
            if line.startswith("//"):
                out.append(line)
                continue
            if line.strip() == "":
                continue
            break
    return out

conflicts = []
unresolved = []
unjustified = []
unretained = []
checked = 0

for path in tracked:
    spdx = ""
    citations = []
    derivations = set()
    derived = False
    header = header_of(path)
    asserted = copyright_claims(header, anchored=True)
    prefix = None          # (indent width, directory path) of the enclosing `.../` line
    for line in header:
        match = SPDX.match(line)
        if match:
            spdx = match.group(1)
        if DERIVATION.match(line):
            derived = True
        elif REFERENCE.match(line):
            derived = False
        if NOT_PORTED.match(line):
            break
        match = CITATION.match(line)
        if not match:
            continue
        indent, token = len(match.group(1)), match.group(2)
        if token.endswith("/"):
            # Held rather than emitted: if indented filenames follow, they are
            # what was cited. If none do, the directory itself is the citation
            # and gets emitted when the header ends.
            if prefix is not None and not prefix[2]:
                citations.append(prefix[1])
            prefix = [indent, token, False]
            continue
        expanded = [part for part in expand_braces(token) if FILENAME.search(part)]
        if not expanded:
            continue
        if prefix is not None and indent > prefix[0]:
            prefix[2] = True
            citations.extend(prefix[1] + part for part in expanded)
        else:
            prefix = None
            citations.extend(expanded)
        if derived:
            derivations.update(citations[-len(expanded):])
    if prefix is not None and not prefix[2]:
        citations.append(prefix[1])
    if not citations and not asserted:
        continue

    justified = set()
    retain = set()
    for citation in citations:
        resolved = resolve_citation(citation)
        if not resolved:
            unresolved.append((path, citation))
            continue
        reported = False
        for member in resolved:
            checked += 1
            with open(member, encoding="utf-8", errors="replace") as handle:
                head = handle.read(8000)
            found = copyright_claims(head.splitlines())
            justified |= found
            # A package directory names too many holders for a file header to
            # reproduce, so retention is asked only of a citation that resolves
            # to one file. That is a stated limit of this rule, not a claim
            # that a whole-package port owes nothing.
            if citation in derivations and len(resolved) == 1:
                retain |= found
            if not reported and COPYLEFT.search(head) and PERMISSIVE.match(spdx):
                # One report per citation: a package directory is cited once,
                # however many copyleft files happen to sit under it.
                conflicts.append((path, spdx, citation))
                reported = True

    for year, holder in sorted(asserted - justified):
        unjustified.append((path, year, holder, tuple(citations)))
    for year, holder in sorted(retain - asserted):
        unretained.append((path, year, holder))

status = 0
if conflicts:
    status = 1
    for path, spdx, citation in conflicts:
        print(f"COPYLEFT   {path} declares {spdx} but cites a copyleft upstream file:", file=sys.stderr)
        print(f"           {citation}", file=sys.stderr)

if unresolved:
    status = 1
    for path, citation in sorted(set(unresolved)):
        print(f"UNRESOLVED {path} cites an upstream file this check cannot open:", file=sys.stderr)
        print(f"           {citation}", file=sys.stderr)
    print("", file=sys.stderr)
    print("Point the matching *_SRC variable at that checkout, or correct the", file=sys.stderr)
    print("citation. An unopened citation is an unchecked licence, not a clean one.", file=sys.stderr)

if unjustified:
    status = 1
    for path, year, holder, citations in unjustified:
        print(f"UNJUSTIFIED {path} asserts a copyright no cited file reproduces:", file=sys.stderr)
        print(f"            {year}, {holder}", file=sys.stderr)
        for citation in citations:
            print(f"            cites {citation}", file=sys.stderr)
        if not citations:
            print("            cites nothing", file=sys.stderr)
    print("", file=sys.stderr)
    print("Either the line was copied out of a file this header does not cite,", file=sys.stderr)
    print("or its year/holder drifted from the file it came from. Make the notice", file=sys.stderr)
    print("reproduce the source it claims, or drop it.", file=sys.stderr)

if unretained:
    status = 1
    for path, year, holder in unretained:
        print(f"UNRETAINED  {path} ports from a file whose notice it drops:", file=sys.stderr)
        print(f"            {year}, {holder}", file=sys.stderr)
    print("", file=sys.stderr)
    print("BSD-3-Clause and Apache-2.0 both require retaining the copyright", file=sys.stderr)
    print("notice of the source a derivative work is built from. Add the line", file=sys.stderr)
    print("as the cited file writes it, or say `Used by` if nothing was ported.", file=sys.stderr)

print(f"checked {checked} upstream file(s) cited by {len(tracked)} tracked source file(s)")
if status == 0:
    print("OK: no permissive-SPDX file cites a copyleft upstream file, every")
    print("    asserted upstream copyright is reproduced by a file that file cites,")
    print("    and every ported-from file's notice is retained")
sys.exit(status)
PYEOF
