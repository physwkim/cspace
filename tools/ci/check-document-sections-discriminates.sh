#!/bin/bash
# Asserts that `check-document-sections.sh` still tells the cases apart.
#
# That gate answers one question -- did a section leave a document without
# anyone saying so -- and every way of getting it wrong is quiet. A rule that
# has stopped discriminating still prints `OK` with a large count on it, and
# five of the six shapes measured before it fail exactly that way: they run,
# they report, and what they report is renames. So the claim "this gate fires
# on a silent removal and not on a rename" is asserted here rather than left to
# the gate's own green line.
#
# Each scenario is an orphan repository built from nothing, so the subject is
# the constructed history and never this one. Every guard in that gate was
# neutralized in turn against these scenarios; what follows is the surviving
# set, one scenario per guard, with the guard each one isolates named beside
# it. Two of the neutralizations take down a family rather than a single
# scenario, which is recorded on those lines rather than smoothed over:
#
#   turning the rename test off        reddens prose_rename AND renumber
#   keying on prose instead of number  reddens number_key, and also declared
#                                      and stale_tip_merge, whose declarations
#                                      stop matching once the key changes
#
# `two_sided_merge` is the incident and the reason the gate exists: one branch
# deletes a section, the other extends the same document elsewhere, and git
# merges both cleanly because the edits do not overlap. Deleting lines in a
# single commit is a different and much easier event to catch, so it is not
# what is asserted here.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

GATE="$REPO_ROOT/tools/ci/check-document-sections.sh"
if [[ ! -x "$GATE" ]]; then
  echo "FAIL $GATE is missing or not executable -- there is nothing to test" >&2
  exit 2
fi

ROOT="$(mktemp -d)"
trap 'rm -rf "$ROOT"' EXIT
failures=0
checked=0

new_repo() {
  local d="$ROOT/$1"
  mkdir -p "$d/tools/ci" "$d/doc"
  cp "$GATE" "$d/tools/ci/check-document-sections.sh"
  cp "$REPO_ROOT/tools/ci/gate-lib.sh" "$d/tools/ci/gate-lib.sh"
  git -C "$d" init -q -b main
  git -C "$d" config user.email selftest@example.invalid
  git -C "$d" config user.name "section selftest"
  git -C "$d" config commit.gpgsign false
  printf '%s\n' "$d"
}

commit() { git -C "$1" add -A && git -C "$1" commit -q "${@:2}"; }

base_doc() {
  cat <<'EOF'
# Title

## 1. alpha

alpha line one
alpha line two
alpha line three

## 2. beta

beta line one
beta line two
beta line three
beta line four

## 3. gamma

gamma line one
gamma line two
EOF
}

drop_beta() {
  python3 - "$1" <<'PY'
import re
import sys
path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    text = handle.read()
with open(path, "w", encoding="utf-8") as handle:
    handle.write(re.sub(r"## 2. beta\n\n(beta line \w+\n)+\n", "", text))
PY
}

# token <path> <depth> <kind> <key> -- the same key the gate derives.
token() {
  python3 - "$@" <<'PY'
import hashlib
import sys
print(hashlib.sha1("\0".join(sys.argv[1:]).encode("utf-8")).hexdigest()[:8])
PY
}

# expect <name> <dir> <expected-status> <expected-FAIL-lines> <substring>
expect() {
  local name="$1" dir="$2" want_status="$3" want_fails="$4" want_text="$5"
  local out status fails
  if out="$( cd "$dir" && ./tools/ci/check-document-sections.sh 2>&1 )"; then
    status=0
  else
    status=$?
  fi
  fails="$(printf '%s\n' "$out" | grep -c '^FAIL ' || true)"
  checked=$((checked + 1))
  if [ "$status" -ne "$want_status" ] || [ "$fails" -ne "$want_fails" ] \
     || ! printf '%s' "$out" | grep -qF -- "$want_text"; then
    failures=$((failures + 1))
    echo "FAIL scenario $name: expected exit $want_status with $want_fails FAIL" \
         "lines containing '$want_text', got exit $status with $fails:" >&2
    printf '%s\n' "$out" | sed 's/^/       /' >&2
  fi
}

# --- the incident: a branch deletes a section, the trunk edits elsewhere -----
# isolates: nothing on its own -- this is the event, not a guard.
r="$(new_repo two_sided_merge)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
git -C "$r" checkout -q -b deleter
drop_beta "$r/doc/thing.md"
commit "$r" -m "delete 2. on the branch"
git -C "$r" checkout -q main
printf 'gamma line three\n' >> "$r/doc/thing.md"
commit "$r" -m "extend 3. on main"
if ! git -C "$r" merge -q --no-edit deleter -m "Merge deleter" >/dev/null 2>&1; then
  echo "FAIL scenario two_sided_merge: the merge conflicted, so git did not take" \
       "the one-sided deletion this scenario is about" >&2
  exit 2
fi
if grep -q '2. beta' "$r/doc/thing.md"; then
  echo "FAIL scenario two_sided_merge: the merge kept 2., so this asserts nothing" >&2
  failures=$((failures + 1))
fi
expect two_sided_merge "$r" 1 1 "removes ## '2. beta'"

# --- a resolution that drops a section neither parent dropped ----------------
# isolates: nothing on its own -- the every-parent clause's other side.
r="$(new_repo merge_only)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
git -C "$r" checkout -q -b other
sed -i 's/beta line one/beta line one, edited on the branch/' "$r/doc/thing.md"
commit "$r" -m "edit 2. on the branch"
git -C "$r" checkout -q main
sed -i 's/beta line one/beta line one, edited on main/' "$r/doc/thing.md"
commit "$r" -m "edit 2. on main"
if git -C "$r" merge -q --no-edit other >/dev/null 2>&1; then
  echo "FAIL scenario merge_only: the merge did not conflict, so the resolution" \
       "this scenario is about never happened" >&2
  exit 2
fi
python3 - "$r/doc/thing.md" <<'PY'
import re
import sys
path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    text = handle.read()
with open(path, "w", encoding="utf-8") as handle:
    handle.write(re.sub(r"## 2. beta\n(.|\n)*?## 3.", "## 3.", text))
PY
git -C "$r" add -A
git -C "$r" commit -q -m "Merge other, resolved by dropping 2."
expect merge_only "$r" 1 1 "removes ## '2. beta'"

# --- the same merge with nothing removed ------------------------------------
# isolates: nothing -- the control that says the two above are not vacuous.
r="$(new_repo control)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
git -C "$r" checkout -q -b other
sed -i 's/alpha line one/alpha line one, extended on the branch/' "$r/doc/thing.md"
commit "$r" -m "extend 1. on the branch"
git -C "$r" checkout -q main
sed -i 's/gamma line two/gamma line two, edited on main/' "$r/doc/thing.md"
commit "$r" -m "edit 3. on main"
if ! git -C "$r" merge -q --no-edit other -m "Merge other" >/dev/null 2>&1; then
  echo "FAIL scenario control: the merge conflicted, so it is not the clean" \
       "two-sided merge the removal scenarios are compared against" >&2
  exit 2
fi
expect control "$r" 0 0 "remove 0 sections"

# --- a later merge whose other parent is a tip that still had the section ----
# isolates: the every-parent clause. With "any parent" this reddens, and the
# incident above reports the removal twice.
r="$(new_repo stale_tip)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
git -C "$r" checkout -q -b old
git -C "$r" checkout -q main
drop_beta "$r/doc/thing.md"
parent="$(git -C "$r" rev-parse HEAD)"
commit "$r" -m "delete 2. on main

Section-removed: doc/thing.md#$(token doc/thing.md 2 num 2) from $parent -- declared once"
git -C "$r" checkout -q old
printf 'gamma line three\n' >> "$r/doc/thing.md"
commit "$r" -m "extend 3. on the old branch"
git -C "$r" checkout -q main
if ! git -C "$r" merge -q --no-edit old -m "Merge the stale branch" >/dev/null 2>&1; then
  echo "FAIL scenario stale_tip: the merge conflicted, so the stale-tip parent" \
       "this scenario is about never reached the merge" >&2
  exit 2
fi
expect stale_tip "$r" 0 0 "remove 1 sections, under 1 declarations"

# --- a prose-keyed section retitled, body untouched --------------------------
# isolates: the rename test (with renumber, below -- one family).
r="$(new_repo prose_rename)"
printf '# T\n\n## Background audit\n\np1\np2\np3\np4\n\n## Other\n\no1\n' \
  > "$r/doc/thing.md"
commit "$r" -m base
sed -i 's/## Background audit/## Background audit, superseded/' "$r/doc/thing.md"
commit "$r" -m "retitle a prose-keyed section, body untouched"
expect prose_rename "$r" 0 0 "remove 0 sections"

# --- 2. renumbered to 4., body untouched ------------------------------------
# isolates: the rename test (with prose_rename, above -- one family).
r="$(new_repo renumber)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
sed -i 's/## 2. beta/## 4. beta/' "$r/doc/thing.md"
commit "$r" -m "renumber 2. to 4."
expect renumber "$r" 0 0 "remove 0 sections"

# --- the same number, a new title, and a wholly rewritten body ---------------
# isolates: the structural key. Keying on prose reddens this, and also the
# declared and stale_tip scenarios, whose declarations stop matching.
r="$(new_repo number_key)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
sed -i 's/## 2. beta/## 2. beta re-measured (17 sites)/' "$r/doc/thing.md"
sed -i 's/beta line \(\w*\)/wholly different text \1/' "$r/doc/thing.md"
commit "$r" -m "rewrite 2. in place under a new title"
expect number_key "$r" 0 0 "remove 0 sections"

# --- a sub-section under a surviving parent ---------------------------------
# isolates: nothing -- the granularity 250 was written at, which the
# per-document gate that preceded this one could not see at all.
r="$(new_repo subsection_only)"
printf '# T\n\n## 2. beta\n\nb1\nb2\n\n### 2.1 beta one\n\nsub line one\nsub line two\nsub line three\n\n### 2.2 beta two\n\nother sub line\n' \
  > "$r/doc/thing.md"
commit "$r" -m base
python3 - "$r/doc/thing.md" <<'PY'
import re
import sys
path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    text = handle.read()
with open(path, "w", encoding="utf-8") as handle:
    handle.write(re.sub(r"### 2\.1 beta one\n\n(sub line \w+\n)+\n", "", text))
PY
commit "$r" -m "delete 2..1 only"
expect subsection_only "$r" 1 1 "removes ### '2.1 beta one'"

# --- a section and both its children, together ------------------------------
# isolates: the subtree collapse. Without it this reports three removals to
# declare instead of the one event that happened.
r="$(new_repo subtree_collapse)"
printf '# T\n\n## 2. beta\n\nb1\nb2\n\n### 2.1 beta one\n\nsub line one\nsub line two\n\n### 2.2 beta two\n\nother sub line\n\n## 3. gamma\n\ng1\ng2\n' \
  > "$r/doc/thing.md"
commit "$r" -m base
python3 - "$r/doc/thing.md" <<'PY'
import re
import sys
path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    text = handle.read()
with open(path, "w", encoding="utf-8") as handle:
    handle.write(re.sub(r"## 2. beta\n(.|\n)*?## 3.", "## 3.", text))
PY
commit "$r" -m "delete 2. and both of its sub-sections"
expect subtree_collapse "$r" 1 1 "removes ## '2. beta'"

# --- the incident, declared -------------------------------------------------
# isolates: nothing -- says the declaration is a way through, not decoration.
r="$(new_repo declared)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
parent="$(git -C "$r" rev-parse HEAD)"
drop_beta "$r/doc/thing.md"
commit "$r" -m "delete 2.

Section-removed: doc/thing.md#$(token doc/thing.md 2 num 2) from $parent -- superseded by the rewrite"
expect declared "$r" 0 0 "remove 1 sections, under 1 declarations"

# --- a declaration matching no removal --------------------------------------
# isolates: the second side of the declaration check.
r="$(new_repo stale_declaration)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
parent="$(git -C "$r" rev-parse HEAD)"
printf 'gamma line three\n' >> "$r/doc/thing.md"
commit "$r" -m "extend 3.

Section-removed: doc/thing.md#deadbeef from $parent -- nothing was removed"
expect stale_declaration "$r" 1 1 "matches no removal this gate found"

# --- an uncommitted removal -------------------------------------------------
# isolates: the working-tree layer, which is the state a merge is resolved in.
r="$(new_repo working_tree)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
drop_beta "$r/doc/thing.md"
expect working_tree "$r" 1 1 "the working tree drops the ## section '2. beta'"

# --- a heading inside a fenced block ----------------------------------------
# isolates: fence tracking. Without it the quotation reads as a section, and
# replacing what is quoted reads as removing it.
r="$(new_repo fenced_heading)"
printf '# T\n\n## 1. alpha\n\na1\na2\n\nquoting another document:\n\n```markdown\n## quoted heading\n\nquoted body one\nquoted body two\nquoted body three\n```\n\ntail line\n' \
  > "$r/doc/thing.md"
commit "$r" -m base
python3 - "$r/doc/thing.md" <<'PY'
import sys
path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    text = handle.read()
text = text.replace(
    "## quoted heading\n\nquoted body one\nquoted body two\nquoted body three",
    "an entirely different quotation, with no heading in it at all")
with open(path, "w", encoding="utf-8") as handle:
    handle.write(text)
PY
commit "$r" -m "replace what the fence quotes"
expect fenced_heading "$r" 0 0 "remove 0 sections"

# --- a fence that is never closed -------------------------------------------
# isolates: the unclosed-fence failure. Without it every heading below the
# fence is silently absent from the parse.
r="$(new_repo unclosed_fence)"
base_doc > "$r/doc/thing.md"
printf '\n```rust\nlet x = 1;\n' >> "$r/doc/thing.md"
commit "$r" -m base
expect unclosed_fence "$r" 1 1 "is never closed"

# --- a removed section with no body -----------------------------------------
# isolates: the refusal to adjudicate without a body.
r="$(new_repo empty_body)"
printf '# T\n\n## 1. alpha\n## 2. beta\n\nb1\nb2\n' > "$r/doc/thing.md"
commit "$r" -m base
sed -i '/## 1. alpha/d' "$r/doc/thing.md"
commit "$r" -m "delete the body-less 1."
expect empty_body "$r" 1 1 "whose body is empty"

# --- documents with no headings at all --------------------------------------
# isolates: the zero-section failure, which is how a changed heading grammar
# would otherwise spell itself.
r="$(new_repo no_sections)"
printf 'no headings at all\njust prose\n' > "$r/doc/thing.md"
commit "$r" -m base
expect no_sections "$r" 1 1 "parsed zero"

# --- no documents at all ----------------------------------------------------
# isolates: the empty-population failure.
r="$(new_repo no_documents)"
rmdir "$r/doc"
printf 'x\n' > "$r/notes.txt"
commit "$r" -m base
expect no_documents "$r" 1 1 "no tracked Markdown documents were named"

# --- a shallow checkout -----------------------------------------------------
# isolates: the shallow failure. `rev-list --parents` there prints commits with
# no parents at all, so the comparison finds nothing and would pass.
r="$(new_repo shallow_source)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
printf 'gamma line three\n' >> "$r/doc/thing.md"
commit "$r" -m second
git clone -q --depth 1 "file://$r" "$ROOT/shallow"
expect shallow "$ROOT/shallow" 1 1 "the checkout is shallow"

# --- invoked by absolute path from another worktree -------------------------
# isolates: `require_caller_tree`.
r="$(new_repo foreign)"
base_doc > "$r/doc/thing.md"
commit "$r" -m base
git -C "$r" worktree add -q -b side "$ROOT/foreign-side"
if out="$( cd "$ROOT/foreign-side" && "$r/tools/ci/check-document-sections.sh" 2>&1 )"; then
  status=0
else
  status=$?
fi
checked=$((checked + 1))
if [ "$status" -eq 0 ] || ! printf '%s' "$out" \
     | grep -qF -- "both are worktrees of the same repository"; then
  failures=$((failures + 1))
  echo "FAIL scenario foreign_worktree: the gate measured another worktree" >&2
  printf '%s\n' "$out" | sed 's/^/       /' >&2
fi

require_nonempty "$checked" "scenarios to run against check-document-sections.sh"

if [ "$failures" -ne 0 ]; then
  echo "FAIL $failures of $checked scenarios did not discriminate as asserted" >&2
  exit 1
fi

echo "OK check-document-sections.sh discriminates on all $checked scenarios: the" \
     "two-sided merge and the merge-only resolution redden it, a rename, a" \
     "renumber, an in-place rewrite, a fenced quotation and a later merge" \
     "against a stale tip do not"
