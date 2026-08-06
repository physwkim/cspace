#!/bin/bash
# Checks that `PORTING-PLAN.md`'s section numbers are unique, that no branch
# has left an unassigned-number placeholder behind, and that no commit has
# dropped a top-level section its parent still had.
#
# Why this exists: parallel panels each append a section and each picks the
# next free number by reading the file in its own worktree. Every branch is
# therefore correct alone and wrong together -- in one round three separate
# branches all chose `§226` and a fourth pair both chose `§220`. Git merges
# them without complaint because the appends do not overlap textually, so the
# collision is invisible until a reader follows a cross-reference and lands in
# another panel's section. Renumbering afterwards is worse than it sounds: the
# references live in Rust doc comments and other docs too, and the round that
# renumbered `§226 -> §227` in the Markdown left four references in
# `moveit-planners-pilz/src/lib.rs` pointing at the *other* panel's §226,
# which reads as correct because that heading exists.
#
# The convention this enforces: where the number would go, a worker writes the
# section sigil `§` immediately followed by `NEW` -- so `## `, that token, then
# the title; suffix `.1` for a sub-section of it, suffix `2` for a second
# section on the same branch. Never a number. The number is assigned by the
# single agent that can see all branches at once -- the one doing the merge.
# This script is what makes that a rule rather than a habit: a duplicate number
# fails here, and so does a placeholder that reached the trunk unassigned.
#
# That token is written in two pieces everywhere in this file, and built from
# an escape in the code below (`PLACEHOLDER`), so this file's own bytes never
# contain it. That is load-bearing, not fastidiousness. The scan used to be
# narrowed to `.md` and `.rs` for exactly one real reason -- this script named
# the token it was looking for, and would have failed on itself -- with the
# narrowing justified after the fact by "sections are cited in documentation
# and in doc comments, never in build or CI scripts". That was false, and a
# placeholder duly reached the trunk through the gap, in a `.sh` comment. With
# the literal absent here, the scan needs no exception, so it has none: every
# tracked file, no suffix list. The one place the token may legitimately appear
# is a real, unassigned placeholder.
#
# The same parallel-append shape has a second, louder failure mode, and it is
# the reason for the history check below. `8f9db1f` deleted §250 from its own
# branch, correctly: a renumber commit had imported main's §250 into a branch
# that had no business carrying it. Merging that branch into main gave git a
# one-sided deletion of a block main had meanwhile extended, so it took the
# deletion: 165 of §250's 183 lines went out of the merge with no conflict
# marker at all, and only the 18-line tail touching §251 conflicted. Measured
# by replaying the merge with `git merge-tree 10a9c13 8f9db1f`, whose result
# has zero §250 headings. Nothing here saw it -- the number was not duplicated,
# it was gone -- and `git diff --stat` said `1 file changed`. A reviewer who
# read the conflict block and nothing else would have shipped it.
#
# Two designs were measured against this repository's 2391 reachable commits
# before picking one:
#
#   (A) taken: a top-level number present in a parent must still be present in
#       the child. Fires on 2 of 2391 commits, both genuine removals on worker
#       branches (`8f9db1f` §250, `3a8727c` §216 renumbered to §218) and both
#       declared below; fires on zero merges.
#
#   (B) rejected: continuity of the integer sequence, with the unassigned
#       numbers listed. Two numbers are unassigned today -- 222 and 223, and
#       written without the section sigil on purpose, because the sentence is
#       about numbers that resolve to no heading and `check-section-references`
#       scans this file like any other. Against a declaration of exactly those
#       two, 99 of 2391 commits fail -- 14
#       distinct gap shapes, 12 of them on this branch's own first-parent
#       line, because a gap is the *normal* transient state while parallel
#       branches take numbers out of order. Worse, (B) is structurally blind to
#       the case that matters most: removing the highest-numbered section
#       leaves no gap at all. Deleting §253 from today's tree keeps the gap set
#       at exactly [222, 223] and (B) passes. Every section is the highest one
#       for the round in which it is written -- 234 distinct maxima across this
#       history -- so every new section spends its most exposed period in the
#       state (B) cannot see. §250 was caught by (B)'s rule only by the
#       accident that §251 had already landed.
#
# The rule is evaluated over every commit reachable from HEAD, not just HEAD
# against its parents. CI runs once per push, at the tip; a deletion in any
# earlier commit of that push is invisible to a one-step rule, and the §250
# deletion was in exactly such a commit. Walking the whole graph also means a
# rewritten or force-pushed history cannot smuggle one in. Measured cost: 2.1 s
# for 2391 commits over 418 distinct blobs.
#
# The rule is applied once more to the working tree against HEAD, because the
# history walk by construction says nothing about a merge that has not been
# committed -- and that is the state a reviewer resolves a merge in.
#
# Named `check-*` so `ci.yml`'s glob runs it: needs nothing but python3, git,
# and the tracked files. It does need real history, so `ci.yml` checks out with
# `fetch-depth: 0`. A shallow checkout is a hard failure rather than a skip,
# because `git rev-list --parents -1 HEAD` in a depth-1 clone prints the commit
# with *no parents at all*: the comparison would find nothing to compare and
# report OK.
#
# Every failure mode is a hard failure, never a skip -- including a parse that
# finds zero sections, because "the heading grammar changed and this checked
# nothing" otherwise spells itself exactly like success. The counts are on the
# OK line for the same reason.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

if [[ ! -s PORTING-PLAN.md ]]; then
  echo "FAIL PORTING-PLAN.md is missing or empty" >&2
  exit 2
fi

python3 - <<'PY'
import re
import subprocess
import sys


def git(*args):
    return subprocess.run(
        ["git", *args], capture_output=True, check=True
    ).stdout.decode("utf-8", "replace")


tracked = [p for p in git("ls-files", "--deduplicate", "-z").split("\0") if p]

with open("PORTING-PLAN.md", encoding="utf-8") as handle:
    lines = handle.read().split("\n")

# Two heading grammars are live and both are load-bearing. Sections 0-140 were
# written as `## 12. title`; everything from 141 on is `## §141 title`. A
# checker that knew only the newer one would silently skip the first 140
# sections, so both are parsed and a heading matching neither is a failure,
# not a skip.
NUMBERED = re.compile(r"^(#{2,4}) (§?)([0-9]+(?:\.[0-9]+)*)\.? ")

# Fenced blocks quote other documents, so a `## ` inside one is not a section.
# Tracking them by toggling on any line starting with three backticks reads
# this file wrong twice over, and both errors hide sections rather than
# inventing them. First, five paragraphs here *discuss* fences and open with a
# four-backtick inline span (```` ```rust ```` ...); a toggle counts each as a
# fence and the run of backticks is odd, so the flag ends stuck on. Second, a
# block opened with four backticks is not closed by the three-backtick lines
# nested inside it. Together they swallowed 102 of the 229 sections -- the
# checker reported OK having looked at just over half the file.
#
# So: an opening fence is a run of >= 3 backticks whose info string carries no
# backtick of its own, and only a run at least as long, alone on its line,
# closes it. A fence still open at EOF is a hard failure for the same reason.
FENCE = re.compile(r"^(`{3,})([^`]*)$")

# The unassigned-number placeholder. `§` is the section sigil, escaped
# rather than written, so the bytes of this file do not contain the token it
# looks for -- see the header for why that is what lets the scan below cover
# every tracked file with no exceptions. An escape rather than two adjacent
# literals because a formatter will happily join those back together. Every
# message that shows the token to a reader interpolates this name, so the
# spelling still reaches the terminal.
PLACEHOLDER = "\u00a7NEW"


def candidates(text_lines):
    """Every line that can open or close a fence, or be a numbered heading.

    Nothing else can match either regex, which is what lets the history walk
    below feed this the output of a single `git grep '^[#`]'` across 418 blobs
    instead of decoding and splitting 330 MB of Markdown -- 2.1 s against
    8.9 s, verified to produce identical results on all 418.
    """
    return [(n, t) for n, t in enumerate(text_lines, 1) if t[:1] in ("#", "`")]


def scan(cands):
    """-> (top_level, all_ids, duplicates, fenced_spans, unclosed_fence_at)

    `top_level` maps a `##` id to the line of its FIRST occurrence; a repeat is
    recorded in `duplicates` rather than overwriting, so both lines can be
    named. `fenced_spans` are inclusive of both delimiter lines, so a
    placeholder written into a fence marker itself counts as fenced, as it did
    before this was factored out.
    """
    top_level, all_ids, duplicates, spans = {}, set(), [], []
    fence_len, fence_at = 0, 0
    for line_no, line in cands:
        match = FENCE.match(line)
        if match is not None:
            run, info = len(match.group(1)), match.group(2).strip()
            if fence_len == 0:
                fence_len, fence_at = run, line_no
            elif run >= fence_len and info == "":
                spans.append((fence_at, line_no))
                fence_len = 0
            continue
        if fence_len:
            continue
        match = NUMBERED.match(line)
        if match is None:
            continue
        hashes, _, section = match.groups()
        all_ids.add(section)
        if len(hashes) != 2:
            continue
        if section in top_level:
            duplicates.append((section, top_level[section], line_no))
        else:
            top_level[section] = line_no
    if fence_len:
        spans.append((fence_at, None))
    return top_level, all_ids, duplicates, spans, (fence_at if fence_len else 0)


failures = []
placeholders = []

top_level, all_ids, duplicates, spans, unclosed_at = scan(candidates(lines))

for section, first, again in duplicates:
    failures.append(
        f"PORTING-PLAN.md:{again}: duplicate section §{section} "
        f"(first at line {first}) -- two branches picked the same number; "
        f"the merge must renumber one of them"
    )


def fenced(line_no):
    return any(a <= line_no and (b is None or line_no <= b) for a, b in spans)


for line_no, line in enumerate(lines, 1):
    if PLACEHOLDER in line and not fenced(line_no):
        placeholders.append((line_no, "PORTING-PLAN.md", line.strip()))

# A placeholder anywhere in the tree, not just in the plan: the whole point is
# that the worker writes it in its Rust doc comments too, and the merger
# rewrites all of them together.
#
# Every tracked file, with no suffix list. The rule used to be prose and Rust
# only, justified by "sections are cited in documentation and in doc comments,
# never in build or CI scripts". That is false, and measurably so: 123 section
# citations live in 33 tracked files that are neither `.md` nor `.rs` -- 93 in
# 21 `.sh`, 11 in 4 `.py`, 11 in `tools/moveit-oracle/src/oracle.cpp`, 3 in 3
# `.toml`, 3 in 2 `.json`, 1 in `tools/mpr-vs-epa/mpr_case104.c` and 1 in
# `ros/Dockerfile`. A placeholder duly reached the trunk through that gap:
# `ros/verify-ros-interop.sh` carried one in the comment that ends "with
# nothing looking at it." -- `a746945`, where it landed. Quoted rather than
# given as a line number on purpose: that file has taken 221 added lines since,
# and the number this sentence used to carry was already wrong by 48. The
# quotation still finds it. The other gate did not catch it either --
# `check-section-references.sh` reads `.sh`, but its reference pattern requires
# a digit after the sigil, so an unassigned one matches nothing there by
# construction.
#
# Widening to that script's own `SCANNED_SUFFIXES` would not have been enough
# and is the wrong shape anyway: `ros/Dockerfile` has no suffix at all, and the
# `.c`/`.cpp` citations are outside that list too. A list of file kinds is the
# thing that was wrong; a longer list is the same thing, later. So there is no
# list -- the population is `git ls-files`, and the only exclusion is
# PORTING-PLAN.md, which is scanned above with the fence rule it needs.
#
# Bytes, not decoded text. A text loop has to decide what to do about the 35
# tracked files that are not valid UTF-8 (the binary meshes under `fixtures/`),
# and the previous one decided to `continue`: a file it could not read was a
# file it silently did not check. Matching on bytes means the question does not
# arise, and the only way a file goes unscanned is an unreadable path -- which
# is a failure naming the file, not a skip. `scanned` is counted and printed for
# the same reason, and a scan that covered nothing is a failure too.
needle = PLACEHOLDER.encode("utf-8")
scanned = 0
for path in tracked:
    if path == "PORTING-PLAN.md":
        continue
    try:
        with open(path, "rb") as handle:
            blob = handle.read()
    except OSError as error:
        failures.append(
            f"{path}: cannot be read, so it was not scanned for the "
            f"unassigned-section placeholder: {error}"
        )
        continue
    scanned += 1
    if needle not in blob:
        continue
    for line_no, raw in enumerate(blob.split(b"\n"), 1):
        if needle in raw:
            placeholders.append(
                (line_no, path, raw.decode("utf-8", "replace").strip())
            )

if not scanned:
    failures.append(
        "scanned zero tracked files for the unassigned-section placeholder -- "
        "`git ls-files` returned nothing but PORTING-PLAN.md, so this check "
        "covered one file out of the tree"
    )

for line_no, path, text in placeholders:
    failures.append(
        f"{path}:{line_no}: unassigned section placeholder -- the merge must "
        f"replace {PLACEHOLDER} with the number it assigns: {text[:90]}"
    )

if unclosed_at:
    failures.append(
        f"PORTING-PLAN.md:{unclosed_at}: fence opened here is never closed "
        f"-- every section below it was skipped by this check"
    )

if not top_level:
    failures.append(
        "PORTING-PLAN.md:1: parsed zero `##` sections -- the heading grammar "
        "changed and this script checked nothing"
    )

# ---------------------------------------------------------------------------
# No commit removes a top-level section its parent still had.
#
# The declaration travels in the commit message, not in a file beside this
# script. A removal is a property of one commit, and a list of them beside the
# script is a second place that has to be kept in step with git -- the same
# shape as the line-keyed exemption tables here that go stale whenever the
# document they index moves. The trailer names the PARENT's sha, not the
# commit's own, because a commit cannot know its own sha before it exists but
# always knows its parent's; and because it is the parent that the removal is
# measured against, which is the distinction that matters on a merge, where
# §250 was present in one parent and absent in the other.
#
# It may be written by any commit reachable from HEAD, not only by the one
# doing the removal. That is not laxity, it is the only way to declare a
# removal made before this check existed: `8f9db1f` and `3a8727c` predate it
# and history here is never rewritten, so their declarations are written by the
# commit that adds this check. It is also what a merger needs, since the
# offending commit is by then already in the graph.
#
# Two-sided, like `upstream-citation-exemptions.json`'s unresolvable list: a
# declaration that matches no actual removal is a failure too. Otherwise this
# accumulates permissions for removals that were later undone, and the next
# silent deletion lands under one of them.
# ---------------------------------------------------------------------------
TRAILER = re.compile(
    r"^Plan-section-removed:[ \t]*§?([0-9]+(?:\.[0-9]+)*)"
    r"[ \t]+from[ \t]+([0-9a-fA-F]{7,40})"
    r"[ \t]+--[ \t]+(\S.*)$",
    re.MULTILINE,
)

history_checked = None

if git("rev-parse", "--is-shallow-repository").strip() == "true":
    failures.append(
        "the checkout is shallow, so no commit has a parent here and this "
        "check would compare nothing and pass -- clone with full history "
        "(`fetch-depth: 0` for actions/checkout)"
    )
else:
    graph = []
    for row in git("rev-list", "--parents", "HEAD").split("\n"):
        if row.strip():
            shas = row.split()
            graph.append((shas[0], shas[1:]))
    if not graph:
        failures.append(
            "`git rev-list --parents HEAD` named zero commits -- this check "
            "walked no history"
        )
    else:
        known = {child for child, _ in graph}

        # One blob id per commit, and one parse per distinct blob: 2391 commits
        # share 418 blobs here.
        wanted = list(dict.fromkeys(
            [sha for child, parents in graph for sha in (child, *parents)]
        ))
        probe = subprocess.run(
            ["git", "cat-file", "--batch-check"],
            input="".join(f"{sha}:PORTING-PLAN.md\n" for sha in wanted).encode(),
            capture_output=True, check=True,
        ).stdout.decode("utf-8", "replace").rstrip("\n").split("\n")
        blob_of = {}
        for sha, row in zip(wanted, probe):
            field = row.split()
            if len(field) >= 2 and field[1] == "blob":
                blob_of[sha] = field[0]

        representative = {}
        for sha, blob in blob_of.items():
            representative.setdefault(blob, sha)
        revs = list(representative.values())

        heading_lines = {}
        for start in range(0, len(revs), 400):   # keep the argv well short of ARG_MAX
            chunk = revs[start:start + 400]
            # `--no-color` and an explicit `-E` so a `color.grep = always` or a
            # `grep.patternType` in the caller's config cannot change what is
            # parsed here. Exit 1 means no line matched, which is not an error
            # to this command -- the per-revision check below is what turns it
            # into a failure, and it can name the revision.
            found = subprocess.run(
                ["git", "grep", "--no-color", "-n", "-E", "^[#`]",
                 *chunk, "--", "PORTING-PLAN.md"],
                capture_output=True,
            )
            if found.returncode not in (0, 1):
                failures.append(
                    "`git grep` failed over the plan's history: "
                    + found.stderr.decode("utf-8", "replace").strip()[:200]
                )
                break
            out = found.stdout.decode("utf-8", "replace")
            for row in out.split("\n"):
                if not row:
                    continue
                rev, _path, line_no, text = row.split(":", 3)
                heading_lines.setdefault(rev, []).append((int(line_no), text))

        ids_of_blob = {}
        for blob, sha in representative.items():
            if sha not in heading_lines:
                failures.append(
                    f"{sha[:9]}: PORTING-PLAN.md has no line starting with "
                    f"`#` or a backtick -- this blob parsed to nothing"
                )
                continue
            hist_top, _, _, _, hist_unclosed = scan(heading_lines[sha])
            if hist_unclosed:
                failures.append(
                    f"{sha[:9]}: PORTING-PLAN.md:{hist_unclosed}: fence opened "
                    f"here is never closed -- the sections below it were not "
                    f"seen, so a removal among them could not be detected"
                )
            elif not hist_top:
                failures.append(
                    f"{sha[:9]}: PORTING-PLAN.md parsed zero `##` sections -- "
                    f"the heading grammar changed and this commit was not "
                    f"checked"
                )
            ids_of_blob[blob] = frozenset(hist_top)

        def sections_of(sha):
            blob = blob_of.get(sha)
            return ids_of_blob.get(blob, frozenset()) if blob else frozenset()

        removals = []
        edges = 0
        for child, parents in graph:
            child_ids = sections_of(child)
            for parent in parents:
                edges += 1
                gone = sections_of(parent) - child_ids
                for section in sorted(gone, key=lambda i: [int(p) for p in i.split(".")]):
                    removals.append((child, parent, section))
        if not edges:
            failures.append(
                "HEAD has no parent commit here, so no pair was compared -- "
                "this check looked at nothing"
            )

        declared = {}
        log = git("log", "--format=%x01%H%x00%B", "HEAD")
        records = [r for r in log.split("\x01") if r]
        if len(records) != len(graph):
            failures.append(
                f"read {len(records)} commit messages for {len(graph)} commits "
                f"-- the log split is wrong and declarations may be missing"
            )
        for record in records:
            sha, _, body = record.partition("\x00")
            for section, prefix, why in TRAILER.findall(body):
                hits = [c for c in known if c.startswith(prefix.lower())]
                if len(hits) != 1:
                    failures.append(
                        f"{sha[:9]}: `Plan-section-removed: {section} from "
                        f"{prefix}` names "
                        f"{'no commit' if not hits else 'an ambiguous commit'} "
                        f"reachable from HEAD"
                    )
                    continue
                declared.setdefault((hits[0], section), (sha, why))

        # Capped, and the cap says so. One commit that replaces the whole file
        # removes every section at once, which is 253 near-identical lines
        # today -- enough to bury a second, unrelated failure below them. The
        # remainder is printed as a count rather than dropped silently.
        undeclared = [r for r in removals if (r[1], r[2]) not in declared]
        LISTED = 20
        if len(undeclared) > LISTED:
            failures.append(
                f"{len(undeclared)} undeclared top-level section removals; "
                f"the first {LISTED} follow"
            )
        for child, parent, section in undeclared[:LISTED]:
            failures.append(
                f"{child[:9]} drops top-level section §{section}, which its "
                f"parent {parent[:9]} still had -- a merge deletes a section "
                f"without conflicting on it (see "
                f"doc/claim-audit/tools-ci-gates.md). If the removal is "
                f"deliberate, say so in any commit's message: "
                f"`Plan-section-removed: {section} from {parent[:9]} -- <why>`"
            )

        used = {(parent, section) for _, parent, section in removals}
        for (parent, section), (sha, why) in sorted(declared.items()):
            if (parent, section) not in used:
                failures.append(
                    f"{sha[:9]} declares `Plan-section-removed: {section} from "
                    f"{parent[:9]}` but no child of {parent[:9]} removes "
                    f"§{section} -- the declaration outlived what it permitted "
                    f"({why[:60]})"
                )

        # The same rule against the working tree, which no commit covers yet.
        # The history walk answers "did a commit drop a section", and CI asks it
        # at the tip of every push, so a bad merge is caught -- after it is
        # committed. But the moment the §250 loss was reviewable was before
        # that: `git merge` left the resolution in the working tree, the gates
        # were run there, and this check had nothing to say because there was no
        # commit yet. Verified by dropping §253 twice, once uncommitted and once
        # committed: before this block only the committed drop failed. HEAD's
        # section set is already parsed by the walk above, so the comparison
        # against the file this script read at the top is one set difference.
        #
        # No trailer can excuse this one: a declaration names a parent sha, and
        # an uncommitted removal has no commit to carry it. A deliberate removal
        # therefore commits with its declaration and passes both checks at once
        # -- the tree then equals HEAD and this comparison is empty.
        head = git("rev-parse", "HEAD").strip()
        for section in sorted(
            sections_of(head) - set(top_level),
            key=lambda i: [int(p) for p in i.split(".")],
        ):
            failures.append(
                f"the working tree drops top-level section §{section}, which "
                f"HEAD ({head[:9]}) still has -- if this is a merge you are "
                f"resolving, git took a one-sided deletion you have not seen. "
                f"Check the whole file, not the conflict block. If the removal "
                f"is deliberate, commit it with "
                f"`Plan-section-removed: {section} from {head[:9]} -- <why>`"
            )

        history_checked = (len(graph), edges, len(revs), len(removals), len(declared))

if failures:
    for failure in failures:
        print("FAIL " + failure, file=sys.stderr)
    sys.exit(1)

commits, edges, blobs, removals, declared_n = history_checked
print(
    f"OK PORTING-PLAN.md: {len(top_level)} top-level sections, "
    f"{len(all_ids)} numbered headings, all distinct; no {PLACEHOLDER} "
    f"placeholder in it or in the {scanned} other tracked files read as bytes "
    f"({len(tracked)} tracked); {commits} commits over {edges} parent "
    f"edges and {blobs} distinct revisions of the plan remove {removals} "
    f"top-level sections, under {declared_n} declarations"
)
PY
