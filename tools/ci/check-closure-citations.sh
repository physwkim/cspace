#!/bin/bash
# Checks the evidence a residual-claim closure declaration cites.
#
# PORTING-PLAN.md's per-section "이 절이 하지 않은 것" / "닫지 않은 것" lists
# assert claims about the tree at the time the section was written. §291
# (2026-08-06) found that these go stale silently: a later round does exactly
# what a bullet says was not done, and the bullet is left standing, so the
# document asserts a claim and its refutation at once. §291's own fix was to
# hand-sweep a sample and mark confirmed-false items with a bold
# `**... 거짓 → 닫힘 (§N)**` declaration (`거짓 → 닫힘` chosen here because,
# unlike bare `닫혔다` -- used 90+ times in this file for unrelated senses --
# it is unused outside this exact family; every occurrence today is a
# residual-claim closure). §291.5 designed a forward check for these
# declarations' own citations (does the cited §N resolve, does a cited commit
# hash exist) but never built it -- "이 회차는 게이트를 짓지 않았다" -- which
# is why the sweep needed redoing rather than holding: the declarations
# themselves were never machine-checked, so a wrong citation inside one is as
# invisible as the stale bullet it closes.
#
# The §N half is already covered by check-section-references.sh, which scans
# every `§N` in every tracked file, this one included. What that script does
# not do is chase a `` `commit hash` `` cited as evidence out to `git` and
# confirm it names a real commit reachable from HEAD -- so that is the only
# thing this script checks.
#
# Deliberately narrow to the `거짓 → 닫힘` bold declaration, not every spelling
# §291.3 catalogued in the wild (`닫혔다(§N)`, `그 제약은 오늘 없다`, `절반
# 닫혔다(§N)`, ...): those forms overlap with senses of "닫혔다" used
# throughout the file for things that are not this claim-closure family (a
# crate closing, a gap closing, a test closing), so keying off them would
# either miss nothing distinctive or match hundreds of unrelated sentences.
# `거짓 → 닫힘` has no such collision in the file today -- widening the marker
# vocabulary is a decision for whoever next writes a closure that doesn't fit
# this shape, not something this script should guess at.
#
# Named `check-*` so `ci.yml`'s glob runs it. Needs `git` (to resolve hashes
# against this checkout's history) and python3; no docker, no cargo.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"
DOC="$REPO_ROOT/PORTING-PLAN.md"

if [[ ! -s "$DOC" ]]; then
  echo "FAIL $DOC is missing or empty" >&2
  exit 2
fi

python3 - "$DOC" <<'PY'
import re
import subprocess
import sys

path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    lines = handle.read().split("\n")

failures = []


def fail(line_no, message):
    failures.append(f"{path}:{line_no}: {message}")


# Same fence/inline-code handling as check-phase-status.sh, for the same
# reason: a quotation of a declaration (in backticks, or inside a fenced
# example) must not be read as a second declaration, and a hash cited only to
# be discussed as a literal string (rather than as evidence) is not something
# this script can tell apart from real evidence -- so inline code is where the
# hash tokens actually live, and it is also where a quoted marker would hide.
# Blanking it before matching the marker, then searching the ORIGINAL text for
# hash tokens inside the declaration's own span, keeps both correct: the
# marker match ignores quotations, the hash search still sees real citations.
FENCE_RE = re.compile(r"^ {0,3}`{3,}[^`]*$")
INLINE_CODE_RE = re.compile(r"(`+)(.+?)\1")
MARKER_RE = re.compile(r"\*\*([^*]*거짓\s*→\s*닫힘[^*]*)\*\*")
HASH_RE = re.compile(r"`([0-9a-f]{7,40})`")
# 16-hex-char tokens in this file are oracle image digest tags
# (`oracle_image_tag`/`src-digest.sh`), not git commit hashes -- e.g.
# `46ff0fa82d650830`. A commit-hash-shaped 16-hex token is not a real
# ambiguity in git (git hashes are never presented at exactly that length by
# any tool in this repo), so this exclusion cannot misclassify a genuine
# citation, only skip a token that was never one.
DIGEST_LEN = 16
BULLET_START_RE = re.compile(r"^\s*[-*]\s")
TABLE_ROW_RE = re.compile(r"^\s*\|")


def blank_inline_code(text):
    return INLINE_CODE_RE.sub(lambda m: " " * len(m.group(0)), text)


in_fence = False
prose = {}  # line_no -> raw text, outside fences
for i, line in enumerate(lines, 1):
    if FENCE_RE.match(line):
        in_fence = not in_fence
        continue
    if in_fence:
        continue
    prose[i] = line

if in_fence:
    fail(len(lines), "unterminated ``` fence -- declarations after it were not read")

# A declaration's own claim block: from its line to the end of the same table
# row (already one line) or the same bullet (following lines up to the next
# blank line, next bullet start, or next heading/table row). Prose paragraphs
# outside a bullet or table use the same continuation rule with a blank line
# or heading as the only boundary.
declarations = []  # (line_no, marker_text, [line_no for span])
for i in sorted(prose):
    text = prose[i]
    masked = blank_inline_code(text)
    for m in MARKER_RE.finditer(masked):
        marker_text = text[m.start(1) : m.end(1)].strip()
        span = [i]
        if TABLE_ROW_RE.match(text):
            pass  # one line is the whole row
        else:
            j = i + 1
            while j in prose:
                nxt = prose[j]
                if nxt.strip() == "":
                    break
                if nxt.startswith("#"):
                    break
                if TABLE_ROW_RE.match(nxt):
                    break
                if BULLET_START_RE.match(nxt):
                    break
                span.append(j)
                j += 1
        declarations.append((i, marker_text, span))

if not declarations:
    fail(1, "found zero `**...거짓 → 닫힘...**` declarations -- the marker spelling "
            "changed, or check-closure-citations.sh no longer sees PORTING-PLAN.md; "
            "either way this checked nothing")


def commit_resolves(token):
    result = subprocess.run(
        ["git", "cat-file", "-e", f"{token}^{{commit}}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


checked = 0
for line_no, marker_text, span in declarations:
    span_text = "\n".join(prose[j] for j in span)
    for h in HASH_RE.finditer(span_text):
        token = h.group(1)
        if len(token) == DIGEST_LEN:
            continue
        checked += 1
        if not commit_resolves(token):
            fail(
                line_no,
                f"declaration {marker_text!r} cites commit `{token}`, which does "
                f"not resolve as a commit reachable from this checkout -- the "
                f"evidence for this closure cannot be followed",
            )

if failures:
    for failure in failures:
        print("FAIL " + failure, file=sys.stderr)
    sys.exit(1)

print(
    f"OK PORTING-PLAN.md: {len(declarations)} '거짓 → 닫힘' closure declaration(s), "
    f"{checked} cited commit hash(es), all resolve as commits reachable from HEAD "
    f"(§N citations inside them are check-section-references.sh's job, not this "
    f"script's)"
)
PY
