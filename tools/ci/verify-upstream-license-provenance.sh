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
FILENAME = re.compile(r"\.(?:cpp|hpp|h|cc|cxx|c|py)$")
SPDX = re.compile(r"^//\s*SPDX-License-Identifier:\s*(.+?)\s*$")
# Paths after this marker are the opposite of a citation: they name what was
# read and deliberately left unported, so their licence cannot reach this file.
NOT_PORTED = re.compile(r"not ported|deliberately not", re.I)
# Matched against the cited file's own header, not against a repo-level LICENSE.
COPYLEFT = re.compile(r"GNU Lesser General Public|GNU General Public|LGPL|GPL-", re.I)
# Identifiers this workspace uses that a copyleft upstream cannot support.
PERMISSIVE = re.compile(r"^(BSD-|MIT|Apache-)", re.I)

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
checked = 0

for path in tracked:
    spdx = ""
    citations = []
    prefix = None          # (indent width, directory path) of the enclosing `.../` line
    for line in header_of(path):
        match = SPDX.match(line)
        if match:
            spdx = match.group(1)
        if NOT_PORTED.search(line):
            break
        match = CITATION.match(line)
        if not match:
            continue
        indent, token = len(match.group(1)), match.group(2)
        if token.endswith("/"):
            prefix = (indent, token)
            continue
        if not FILENAME.search(token):
            continue
        if prefix is not None and indent > prefix[0]:
            citations.append(prefix[1] + token)
        else:
            prefix = None
            citations.append(token)
    if not citations:
        continue

    for citation in citations:
        resolved = None
        for root in search_roots:
            candidate = os.path.join(root, citation)
            if os.path.isfile(candidate):
                resolved = candidate
                break
        if resolved is None:
            unresolved.append((path, citation))
            continue
        checked += 1
        with open(resolved, encoding="utf-8", errors="replace") as handle:
            head = handle.read(8000)
        if COPYLEFT.search(head) and PERMISSIVE.match(spdx):
            conflicts.append((path, spdx, citation))

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

print(f"checked {checked} upstream citation(s) across {len(tracked)} tracked source file(s)")
if status == 0:
    print("OK: no permissive-SPDX file cites a copyleft upstream file")
sys.exit(status)
PYEOF
