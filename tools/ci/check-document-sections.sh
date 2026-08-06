#!/bin/bash
# Checks that no commit, and no uncommitted merge resolution, drops a section
# out of a tracked Markdown document without saying so.
#
# `check-porting-plan-sections.sh` already does this for one document at one
# depth: a top-level `§N` present in a parent must still be present in the
# child. It was written because `8f9db1f` deleted §250 from a branch and the
# merge took that one-sided deletion silently -- 165 of the section's 183 lines
# left the merge with no conflict marker, and only the 18-line tail touching
# §251 conflicted. A reviewer who read the conflict block and nothing else
# would have shipped it.
#
# That gate covers `PORTING-PLAN.md`'s `##` headings. Everything else was
# blind. Measured by deleting one section from each document in turn and
# running the whole gate suite against the result, with the suite's own
# baseline subtracted so a gate already red on `main` could not be counted as a
# catch: of 68 documents, 5 were caught (`PORTING-PLAN.md` by the gate above,
# `doc/upstream-bugs.md`, `doc/declaration-audit-coverage.md`,
# `doc/port-coverage.md` and `doc/client-endpoint-surface.md` by gates that
# happen to index them for other reasons), 1 had nothing to delete, and 62 were
# invisible -- including every claim-audit file, every assertion-discrimination
# ledger, and `PORTING-PLAN.md`'s own `###` sub-sections, which is the
# granularity §250 was actually written at.
#
# ---------------------------------------------------------------------------
# The key problem, and five rules measured before this one
# ---------------------------------------------------------------------------
#
# Heading text cannot be the key. It is edited in place by design here: a
# ledger heading carries a running count (`tools/moveit-diff (16 sites)` ->
# `(17 sites)`, `Summary (12)`), and a section number is assigned at merge
# time, so the placeholder heading becomes a numbered one. Each of the five
# shapes below was replayed over all 2590 commits reachable from HEAD, against
# every parent of every commit, before picking the sixth:
#
#   (1) positional deletion hunks >= 20 lines: 99 runs in 64 merges. Rejected.
#       A reordered table reads as a deletion -- `doc/upstream-bugs.md`'s Index
#       reorders on nearly every merge and shows a 59-line "loss".
#
#   (2) content present in a parent and nowhere in the child, runs >= 20
#       lines: 77 runs in 46 merges. Rejected. A stale branch tip holds the
#       pre-edit text of a block `main` has since rewritten, and every merge of
#       that branch reads as a loss of it.
#
#   (3) the three-way form of (2) -- in the merge base, still in the parent
#       that did not touch it, gone from the other parent and gone from the
#       merge -- runs >= 30 lines: 36 runs in 28 merges. Rejected, and this one
#       is the informative rejection: it is the exact shape of the incident and
#       it still cannot separate a rewritten section from a deleted one,
#       because content mass alone does not carry the distinction.
#
#   (4) heading text present in a parent, absent from the child: 271 losses in
#       178 commits across 16 documents. Rejected -- almost all renames.
#
#   (5) (4) conjoined with "and the body did not survive anywhere the child
#       added": 167 in 119 commits. Better, still too noisy, and the noise is
#       instructive: it is dominated by the same removal being charged again to
#       every later merge whose *other* parent is an old tip that still had the
#       section.
#
# ---------------------------------------------------------------------------
# The rule
# ---------------------------------------------------------------------------
#
# A section is removed at commit C when all three hold:
#
#   - its key is present in EVERY parent of C that has the document, and in no
#     heading of C. Requiring every parent, rather than any, is what fixes (5):
#     a merge is charged only for what it dropped on its own, and the commit
#     that did the deleting is charged instead -- which is the honest
#     attribution, and still reddens the gate at the merge, because that commit
#     becomes reachable there. A resolution that drops a section neither parent
#     dropped is the case where the merge itself is charged, and it is caught
#     by the same clause with nothing special-cased for it.
#
#   - no section the child ADDED holds at least half of its body lines. That is
#     the rename test, and it is why (4) fails alone. Half, not a tuned-down
#     figure: the threshold is the reading of "renamed rather than removed", and
#     at 35% the count over this history falls from 18 to 12 only by excusing
#     six sections that lost 53-61% of their body.
#
#   - its body is non-empty. A section with no body cannot be adjudicated by
#     the rename test at all, so it is a hard failure naming the section rather
#     than a silent pass. Zero sections in this history are in that state; the
#     clause is here so that stays visible if it changes.
#
# The key itself is taken from structure, never from prose, in this order: a
# leading number (`## 12.`, `## §141`, `### §250.2`), then a leading backticked
# slug (`### `chomp-iteration-double-increment``), then -- only when neither
# exists -- the heading text, which then leans entirely on the rename test.
# The number is what makes the running-count and placeholder-assignment renames
# invisible to this check: `### §259.6 <old title>` and `### §259.6 <new title>`
# are the same section. Rule (5) with prose keys fires 167 times; the same rule
# with structural keys fires 110.
#
# A removed section whose enclosing section this commit also removed is part of
# that removal, not a second one. §250 went out as `## §250` plus six `###`
# children; it is one event and it takes one declaration. The grouping is the
# heading nesting, not a numeric prefix rule -- prose-keyed documents get it
# too.
#
# Over the 2590 commits reachable from this script's parent, across 70
# documents, the rule fires 18 times in 16 commits. Every one is a real section
# removal, and all 18 are declared in this script's own commit message. That is
# one declaration per 144 commits, which is the rate at which this project
# actually removes sections -- not a tuning failure.
#
# ---------------------------------------------------------------------------
# Two layers, and the declaration
# ---------------------------------------------------------------------------
#
# The rule runs over every commit reachable from HEAD, not just HEAD against
# its parents: CI runs once per push, and the §250 deletion was in an earlier
# commit of its push. Walking the whole graph also means a rewritten history
# cannot smuggle one in, and it is why there is no baseline commit here -- a
# baseline is a one-line edit that silences every removal before it, where a
# declaration silences exactly one.
#
# It then runs once more over the working tree against HEAD, because the
# history walk by construction says nothing about a merge that has not been
# committed, and that is the state a reviewer resolves a merge in.
#
# The declaration travels in the commit message, like the sibling gate's, and
# for the same reason: a removal is a property of one commit, and a list beside
# the script is a second place to keep in step with git. It may be written by
# any commit reachable from HEAD, which is the only way to declare the 18 that
# predate this check, and what a merger needs when the offending commit is
# already in the graph.
#
#   Section-removed: <path>#<token> from <parent-sha> -- <why>
#
# The token is a hash of the document path and the section's structural key.
# The heading text is not usable as a written key -- it runs to a hundred
# characters of Korean prose with backticks and dashes in it -- and a bare
# number would not say which document. The gate prints the exact line to paste.
# The human record of what was removed is the `<why>`, which is required, and
# the failure message names the heading so it can be written.
#
# It names the PARENT's sha because a commit cannot know its own before it
# exists but always knows its parent's, and because the parent is what the
# removal is measured against.
#
# Two-sided: a declaration that matches no actual removal is a failure too.
# Otherwise this accumulates permissions for removals that were later undone,
# and the next silent deletion lands under one of them.
#
# Every failure mode is a hard failure, never a skip: an unreadable file, a
# document that is not UTF-8, an unclosed fence (which hides every section
# below it), a parse that finds no sections at all, and a removed section with
# no body. Counts are on the OK line for the same reason. Measured cost: 23 s
# for 2591 commits over 6671 parent edges and 1296 distinct document
# revisions, of 1835 sections in the working tree.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

python3 - <<'PY'
import hashlib
import re
import subprocess
import sys


def git(*args):
    return subprocess.run(
        ["git", *args], capture_output=True, check=True
    ).stdout.decode("utf-8", "replace")


HEADING = re.compile(r"^(#{2,6})\s+(.*?)\s*$")
NUMBER = re.compile(r"^§?([0-9]+(?:\.[0-9]+)*)\.?(?:\s|$)")
SLUG = re.compile(r"^`([^`]+)`")
# An opening fence is a run of >= 3 backticks whose info string carries no
# backtick, closed only by a run at least as long alone on its line. Toggling
# on any three-backtick line reads a document that *discusses* fences wrong,
# and the error hides sections rather than inventing them: that mistake
# swallowed 102 of `PORTING-PLAN.md`'s 229 sections in an earlier checker,
# which reported OK having looked at just over half the file.
FENCE = re.compile(r"^(`{3,})([^`]*)$")
TRAILER = re.compile(
    r"^Section-removed:[ \t]*(\S+)#([0-9a-f]{8})"
    r"[ \t]+from[ \t]+([0-9a-fA-F]{7,40})"
    r"[ \t]+--[ \t]+(\S.*)$",
    re.MULTILINE,
)
SURVIVES = 0.5

failures = []


def key_of(text):
    found = NUMBER.match(text)
    if found is not None:
        return ("num", found.group(1))
    found = SLUG.match(text)
    if found is not None:
        return ("slug", found.group(1))
    return ("prose", text)


def token_of(path, depth, key):
    kind, value = key
    raw = "\0".join((path, str(depth), kind, value)).encode("utf-8")
    return hashlib.sha1(raw).hexdigest()[:8]


class Unparsable(Exception):
    pass


def sections(text):
    """-> [(depth, heading, line_no, frozenset(body), key, enclosing index)].

    A section's extent runs to the next heading of the same or shallower
    depth, so a `##` owns its `###` children and editing a heading directly
    above a sub-section does not read as a section with an empty body.
    """
    lines = text.split("\n")
    heads = []
    body_lines = []
    fence, fence_at = 0, 0
    for index, line in enumerate(lines):
        found = FENCE.match(line)
        if found is not None:
            run = len(found.group(1))
            if fence == 0:
                fence, fence_at = run, index + 1
            elif run >= fence and found.group(2).strip() == "":
                fence = 0
            body_lines.append(line.strip())
            continue
        found = None if fence else HEADING.match(line)
        if found is not None:
            heads.append((len(found.group(1)), found.group(2), index))
            body_lines.append(None)
        else:
            body_lines.append(line.strip())
    if fence:
        raise Unparsable(
            f"the fence opened at line {fence_at} is never closed, so every "
            f"heading below it was skipped"
        )
    out = []
    for n, (depth, heading, index) in enumerate(heads):
        end = len(lines)
        for depth_below, _, at in heads[n + 1:]:
            if depth_below <= depth:
                end = at
                break
        enclosing = None
        for above in range(n - 1, -1, -1):
            if heads[above][0] < depth:
                enclosing = above
                break
        body = frozenset(x for x in body_lines[index + 1:end] if x)
        out.append((depth, heading, index + 1, body, key_of(heading), enclosing))
    return out


def removed(parents, child):
    """Indices into parents[0] removed by a child with all of these parents.

    -> {index: best surviving fraction, or None when the body is empty}
    """
    child_keys = {(s[0], s[4]) for s in child}
    common = None
    for parent in parents:
        parent_keys = {(s[0], s[4]) for s in parent}
        added = [s for s in child if (s[0], s[4]) not in parent_keys]
        here = {}
        for n, (depth, _h, _l, body, key, _e) in enumerate(parent):
            if (depth, key) in child_keys:
                continue
            if not body:
                here[(depth, key)] = (n, None)
                continue
            best = 0.0
            for _d, _h2, _l2, other, _k, _e2 in added:
                best = max(best, len(body & other) / len(body))
            if best < SURVIVES:
                here[(depth, key)] = (n, best)
        common = here if common is None else {
            k: v for k, v in common.items() if k in here
        }
    if not common:
        return {}
    # A removal inside a section this commit also removed is part of it.
    gone = {n for n, _ in common.values()}
    out = {}
    for n, best in common.values():
        enclosing = parents[0][n][5]
        while enclosing is not None and enclosing not in gone:
            enclosing = parents[0][enclosing][5]
        if enclosing is None:
            out[n] = best
    return out


documents = [p for p in
             git("ls-files", "--deduplicate", "-z", "--", "*.md").split("\0") if p]
if not documents:
    print("FAIL no tracked Markdown documents were named -- this gate would "
          "report OK having examined nothing.", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# The working tree against HEAD.
# ---------------------------------------------------------------------------
head = git("rev-parse", "HEAD").strip()
working = {}
working_sections = 0
for path in documents:
    try:
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
    except OSError as error:
        failures.append(f"{path}: cannot be read, so it was not checked for "
                        f"removed sections: {error}")
        continue
    except UnicodeDecodeError as error:
        failures.append(f"{path}: is not valid UTF-8, so it was not checked "
                        f"for removed sections: {error}")
        continue
    try:
        working[path] = sections(text)
    except Unparsable as why:
        failures.append(f"{path}: {why}")
        continue
    working_sections += len(working[path])

if working and not working_sections:
    failures.append(
        f"parsed zero `##` headings across all {len(working)} documents -- the "
        f"heading grammar changed and this gate checked nothing"
    )

# ---------------------------------------------------------------------------
# Every commit reachable from HEAD, against every parent that has the document.
# ---------------------------------------------------------------------------
if git("rev-parse", "--is-shallow-repository").strip() == "true":
    failures.append(
        "the checkout is shallow, so no commit has a parent here and this "
        "gate would compare nothing and pass -- clone with full history "
        "(`fetch-depth: 0` for actions/checkout)"
    )
    graph = []
else:
    graph = []
    for row in git("rev-list", "--parents", "HEAD").split("\n"):
        if row.strip():
            shas = row.split()
            graph.append((shas[0], shas[1:]))
    if not graph:
        failures.append("`git rev-list --parents HEAD` named zero commits -- "
                        "this gate walked no history")

blob_of = {}
if graph:
    revisions = list(dict.fromkeys([sha for child, _ in graph for sha in (child,)]
                                   + [sha for _, ps in graph for sha in ps]))
    query = "".join(f"{sha}:{path}\n" for sha in revisions for path in documents)
    answered = subprocess.run(
        ["git", "cat-file", "--batch-check=%(objectname) %(objecttype)"],
        input=query.encode(), capture_output=True, check=True,
    ).stdout.decode("utf-8", "replace").rstrip("\n")
    probe = answered.split("\n") if answered else []
    if len(probe) != len(revisions) * len(documents):
        failures.append(
            f"`git cat-file --batch-check` answered {len(probe)} of "
            f"{len(revisions) * len(documents)} document revisions -- the "
            f"history walk would have compared a truncated set"
        )
        graph = []
    else:
        rows = iter(probe)
        for sha in revisions:
            for path in documents:
                field = next(rows).split()
                if len(field) >= 2 and field[1] == "blob":
                    blob_of[(sha, path)] = field[0]

found = []
blobs_read = 0
edges = 0
if graph:
    contents = subprocess.Popen(
        ["git", "cat-file", "--batch"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
    )

    def parse_blob(blob, path, cache):
        if blob in cache:
            return cache[blob]
        contents.stdin.write((blob + "\n").encode())
        contents.stdin.flush()
        header = contents.stdout.readline().split()
        size = int(header[2])
        raw = contents.stdout.read(size)
        contents.stdout.read(1)
        try:
            value = sections(raw.decode("utf-8"))
        except (UnicodeDecodeError, Unparsable) as why:
            value = why
        cache[blob] = value
        return value

    for path in documents:
        cache = {}
        for child, parents in graph:
            here = blob_of.get((child, path))
            holders = [(p, blob_of[(p, path)]) for p in parents
                       if (p, path) in blob_of]
            if not holders or all(blob == here for _, blob in holders):
                continue
            edges += len(holders)
            if here is None:
                found.append((child, path, 0, "<the whole document>", 0, 0.0,
                              [p for p, _ in holders]))
                continue
            child_sections = parse_blob(here, path, cache)
            if isinstance(child_sections, Exception):
                failures.append(f"{path} at {child[:9]}: {child_sections}")
                continue
            parent_sections = []
            broken = False
            for parent, blob in holders:
                one = parse_blob(blob, path, cache)
                if isinstance(one, Exception):
                    failures.append(f"{path} at {parent[:9]}: {one}")
                    broken = True
                    break
                parent_sections.append(one)
            if broken:
                continue
            for n, best in removed(parent_sections, child_sections).items():
                depth, heading, _line, body, _key, _enc = parent_sections[0][n]
                found.append((child, path, depth, heading, len(body), best,
                              [p for p, _ in holders]))

        # The same rule once more against HEAD, while this document's parsed
        # revisions are still in hand. The history walk by construction says
        # nothing about a merge that has not been committed, and that is the
        # state a reviewer resolves a merge in.
        at_head = blob_of.get((head, path))
        if at_head is not None and path in working:
            was = parse_blob(at_head, path, cache)
            if isinstance(was, Exception):
                failures.append(f"{path} at HEAD {head[:9]}: {was}")
            else:
                for n, _best in removed([was], working[path]).items():
                    depth, heading, _line, body, key, _enc = was[n]
                    token = token_of(path, depth, key)
                    failures.append(
                        f"{path}: the working tree drops the {'#' * depth} "
                        f"section {heading[:70]!r} ({len(body)} body lines), "
                        f"which HEAD ({head[:9]}) still has -- if this is a "
                        f"merge you are resolving, git took a one-sided "
                        f"deletion you have not seen; check the whole file, "
                        f"not the conflict block. If the removal is "
                        f"deliberate, commit it with `Section-removed: "
                        f"{path}#{token} from {head[:9]} -- <why>`"
                    )
        blobs_read += len(cache)

    contents.stdin.close()
    contents.wait()

# ---------------------------------------------------------------------------
# Declarations, matched both ways.
# ---------------------------------------------------------------------------
declared = []
if graph:
    for path, token, sha, why in TRAILER.findall(git("log", "--format=%B", "HEAD")):
        declared.append((path, token, sha.lower(), why))

used = set()
for child, path, depth, heading, body_lines, best, holders in found:
    if depth == 0:
        token = token_of(path, 0, ("doc", path))
    else:
        token = token_of(path, depth, key_of(heading))
    ok = False
    for n, (dpath, dtoken, dsha, _why) in enumerate(declared):
        if dpath == path and dtoken == token and any(p.startswith(dsha)
                                                     for p in holders):
            used.add(n)
            ok = True
    if ok:
        continue
    if best is None:
        failures.append(
            f"{path}: {child[:9]} removes the {'#' * depth} section "
            f"{heading[:70]!r}, whose body is empty -- this gate cannot tell a "
            f"removal from a rename without a body, so it refuses to guess"
        )
        continue
    where = "the whole document" if depth == 0 else f"{'#' * depth} {heading[:70]!r}"
    failures.append(
        f"{path}: {child[:9]} removes {where} ({body_lines} body lines, "
        f"{best:.0%} of it survives elsewhere in that commit), which every "
        f"parent it has still had. Declare it with `Section-removed: "
        f"{path}#{token} from {holders[0][:9]} -- <why>`"
    )

for n, (path, token, sha, _why) in enumerate(declared):
    if n not in used:
        failures.append(
            f"`Section-removed: {path}#{token} from {sha}` matches no removal "
            f"this gate found -- a declaration that authorises nothing is a "
            f"standing permission for the next silent deletion"
        )

if failures:
    for failure in failures:
        print("FAIL " + failure, file=sys.stderr)
    sys.exit(1)

print(
    f"OK {len(working)} tracked documents, {working_sections} sections in the "
    f"working tree, none dropped against HEAD ({head[:9]}); {len(graph)} "
    f"commits over {edges} parent edges and {blobs_read} distinct document "
    f"revisions remove {len(found)} sections, under {len(declared)} "
    f"declarations"
)
PY
