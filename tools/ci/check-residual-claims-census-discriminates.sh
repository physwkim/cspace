#!/bin/bash
# Asserts that `check-residual-claims-census.py` still collects the bullets it
# claims to, and still stops where it claims to.
#
# That gate's whole product is a population count: "lead-in 42건, 최상위 불릿
# 169건". A parser that quietly collects fewer of them does not fail -- it
# emits a smaller census, `--check` compares that census against the same
# smaller derivation, and the two agree. The gate is green precisely when it
# is wrong, which is why the rule is asserted here instead of being left to
# its own OK line.
#
# The incident: 16387969 gave PORTING-PLAN.md §281.6's `cylinder × box` item a
# continuation paragraph -- blank line, then an indented paragraph belonging to
# that same item -- and the parser read the blank as the end of the list. The
# still-open `관통 분기는 건드리지 않았다` bullet after the paragraph left the
# census silently (169 -> 168) and a fresh derivation agreed with itself. The
# scenarios below are one per rule in the item-boundary logic, with what a
# neutralization of each does, measured rather than predicted:
#
#   blank ends the item                    reddens continuation only --
#   (the pre-fix rule)                       blank_separated_siblings survives
#                                            it, because the outer loop still
#                                            re-reads a bullet at the blank
#   blank never ends the item               reddens unindented_ends_list
#   drop the BULLET_RE boundary             reddens two_bullets_stay_two, and
#                                            with it continuation,
#                                            unindented_ends_list and
#                                            closure_marker_is_per_bullet
#   drop the HEADING_RE boundary            reddens heading_ends_item, on its
#                                            text assertion only -- the count
#                                            does not move
#   drop the TABLE_ROW_RE boundary          reddens table_ends_item, likewise
#                                            on text only
#   search the closure marker document-wide reddens closure_marker_is_per_bullet
#   hardcode the row citation prefix        reddens continuation's text pair
#   drop an unbulleted lead-in (the old      reddens prose_only_leadin and
#   bare `continue`)                          prose_then_bullets
#   report every unbulleted lead-in as       reddens prose_then_bullets
#   `불릿 없음`
#
# Every line above was produced by applying that neutralization to
# `check-residual-claims-census.py` and running this script, not predicted:
# two of the six do not move any count at all, and the first one reddens half
# of what a reading of the code says it should.
#
# The fixture headings use PORTING-PLAN.md's bare `## 900.1` spelling rather
# than its section-sign one, and the closed bullet cites the real §301 that
# built this census: check-section-references.sh scans THIS file's own text and
# cannot tell a constructed fixture from a citation, so a made-up number in the
# section-sign form is a dangling reference to it -- that gate's own header
# records having written the same line twice for the same reason. Both
# spellings parse identically (`SECTION_ID_RE`'s optional sign), and no
# scenario below is about which one is used.
#
# The fixtures are synthetic documents in a temp dir, never this tree's
# PORTING-PLAN.md: a scenario that reads the real document would change its
# own expectations every time someone writes a section.
#
# expiry_marker / expiry_marker_removed / expiry_blank_trigger below are not
# about the continuation-paragraph incident above -- they discriminate
# PORTING-PLAN.md §308.4's A3 other legitimate exit, `OPEN → 만료 조건
# (<trigger>)`, added the same round these three were: with a stated trigger
# the marked bullet tallies EXPIRY (not OPEN); strip the marker back off and
# OPEN rises again (the removal mutation); leave the trigger sentence blank
# and the gate refuses to emit a census at all, because A3's own wording
# does not accept an expiry conversion with no stated trigger time.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

GATE="$REPO_ROOT/tools/ci/check-residual-claims-census.py"
if [[ ! -x "$GATE" ]]; then
  echo "FAIL $GATE is missing or not executable -- there is nothing to test" >&2
  exit 2
fi

ROOT="$(mktemp -d)"
trap 'rm -rf "$ROOT"' EXIT
failures=0
checked=0

# expect <name> <fixture-file> <want-leadins> <want-bullets> <want-labels...>
#
# Labels are the `[A]`..`[Z]` tags each fixture bullet opens with, so a
# scenario states WHICH bullets it expects, not only how many: a rule that
# loses one bullet and gains another keeps the count and fails here.
expect() {
  local name="$1" fixture="$2" want_leadins="$3" want_bullets="$4"
  shift 4
  local out status got_leadins got_bullets got_labels want_labels
  checked=$((checked + 1))
  if out="$("$GATE" --doc "$fixture" --emit "$ROOT/$name.census" 2>&1)"; then
    status=0
  else
    status=$?
  fi
  if [ "$status" -ne 0 ]; then
    failures=$((failures + 1))
    echo "FAIL scenario $name: the gate exited $status on its own fixture" >&2
    printf '%s\n' "$out" | sed 's/^/       /' >&2
    return
  fi
  got_leadins="$(printf '%s' "$out" | sed -n 's/.*: \([0-9]*\) lead-ins.*/\1/p')"
  got_bullets="$(printf '%s' "$out" | sed -n 's/.*lead-ins, \([0-9]*\) bullets.*/\1/p')"
  got_labels="$(grep -o '\[[A-Z]\]' "$ROOT/$name.census" | tr -d '[]' | sort | tr -d '\n')"
  want_labels="$(printf '%s\n' "$@" | sort | tr -d '\n')"
  if [ "$got_leadins" != "$want_leadins" ] || [ "$got_bullets" != "$want_bullets" ] \
     || [ "$got_labels" != "$want_labels" ]; then
    failures=$((failures + 1))
    echo "FAIL scenario $name: expected $want_leadins lead-ins, $want_bullets" \
         "bullets, labels [$want_labels]; got $got_leadins, $got_bullets," \
         "labels [$got_labels]" >&2
    printf '%s\n' "$out" | sed 's/^/       /' >&2
  fi
}

# expect_text <name> <fixture> present|absent <substring>
#
# `%DOC%` in the substring expands to the fixture's basename. Writing the
# citation out literally is what a `<file>.md:<line>` assertion costs here:
# check-citation-drift.py reads every tracked file for in-repo citations, so
# the literal is a citation to a temp path it cannot resolve, and the freeze
# file gains a key that retires on the next edit.
expect_text() {
  local name="$1" fixture="$2" mode="$3" want="$4"
  want="${want//%DOC%/$(basename "$fixture")}"
  checked=$((checked + 1))
  if ! "$GATE" --doc "$fixture" --emit "$ROOT/$name.census" >/dev/null 2>&1; then
    failures=$((failures + 1))
    echo "FAIL scenario $name: the gate exited nonzero on its own fixture" >&2
    return
  fi
  local found=absent
  grep -qF -- "$want" "$ROOT/$name.census" && found=present
  if [ "$found" != "$mode" ]; then
    failures=$((failures + 1))
    echo "FAIL scenario $name: expected '$want' $mode in the census, it is $found" >&2
    grep -n '^|' "$ROOT/$name.census" | sed 's/^/       /' >&2
  fi
}

# --- the incident: a continuation paragraph inside an item ------------------
# isolates: that a blank line is a separator, not a boundary.
cat > "$ROOT/continuation.md" <<'EOF'
## 900.1 이 절이 하지 않은 것

- **[A] 첫째는 재지 않았다.** 바로 이어지는 줄.

  빈 줄 뒤에 들여쓴 이어짐 문단. 이 항목의 일부다.
- **[B] 둘째는 훑지 않았다.**
EOF
expect continuation "$ROOT/continuation.md" 1 2 A B

# --- a blank line between two siblings ---------------------------------------
# isolates: the same rule from the other side -- the item after the blank is a
# bullet, so the list continues without any indented paragraph in between.
cat > "$ROOT/blank_separated_siblings.md" <<'EOF'
## 900.2 이 회차가 못 본 것

- **[A] 첫째.**

- **[B] 둘째.**
EOF
expect blank_separated_siblings "$ROOT/blank_separated_siblings.md" 1 2 A B

# --- an unindented paragraph ends the list -----------------------------------
# isolates: the column-0 boundary. Without it the parser runs past the list and
# swallows every later top-level bullet in the document, [C] included.
cat > "$ROOT/unindented_ends_list.md" <<'EOF'
## 900.3 이 절이 재지 않은 것

- **[A] 첫째.**
- **[B] 둘째.**

들여쓰지 않은 문단. 목록은 여기서 끝난다.

- **[C] 이 불릿은 다른 목록이고 lead-in 아래가 아니다.**
EOF
expect unindented_ends_list "$ROOT/unindented_ends_list.md" 1 2 A B

# --- two bullets stay two ----------------------------------------------------
# isolates: the BULLET_RE boundary inside the item scan. Neutralized, [B]'s
# line is appended to [A]'s text and the count halves.
cat > "$ROOT/two_bullets_stay_two.md" <<'EOF'
## 900.4 이 절이 하지 않은 것

- **[A] 첫째.**
- **[B] 둘째.**
EOF
expect two_bullets_stay_two "$ROOT/two_bullets_stay_two.md" 1 2 A B

# --- a heading ends the item -------------------------------------------------
# isolates: the HEADING_RE boundary. Neutralized, the next section's heading and
# its prose land inside [A]'s claim text.
cat > "$ROOT/heading_ends_item.md" <<'EOF'
## 900.5 이 절이 하지 않은 것

- **[A] 첫째.**
### 900.6 다음 절
평문.
EOF
expect heading_ends_item "$ROOT/heading_ends_item.md" 1 1 A
expect_text heading_ends_item "$ROOT/heading_ends_item.md" absent "다음 절"

# --- a table row ends the item -----------------------------------------------
# isolates: the TABLE_ROW_RE boundary. Neutralized, the table's own rows are
# concatenated into [A], and a `|` inside a claim breaks the census's own table.
cat > "$ROOT/table_ends_item.md" <<'EOF'
## 900.7 이 회차가 못 본 것

- **[A] 첫째.**
| 열 | 값 |
|---|---|
| 가 | 1 |
EOF
expect table_ends_item "$ROOT/table_ends_item.md" 1 1 A
expect_text table_ends_item "$ROOT/table_ends_item.md" absent "열 | 값"

# --- the closure marker is read per bullet, not per document -----------------
# isolates: that CLOSURE_RE is searched in the bullet's own text. Neutralized to
# a document-wide search, [B] reports CLOSED (§301) too.
cat > "$ROOT/closure_marker_is_per_bullet.md" <<'EOF'
## 900.8 이 절이 하지 않은 것

- **[A] 첫째. 거짓 → 닫힘 (§301).** 이유.
- **[B] 둘째.** 아직 열려 있다.
EOF
expect closure_marker_is_per_bullet "$ROOT/closure_marker_is_per_bullet.md" 1 2 A B
expect_text closure_marker_is_per_bullet \
  "$ROOT/closure_marker_is_per_bullet.md" present "| CLOSED (§301) |"
expect_text closure_marker_is_per_bullet \
  "$ROOT/closure_marker_is_per_bullet.md" present "[B] 둘째.** 아직 열려 있다. | OPEN |"

# --- the expiry marker is EXPIRY, not OPEN, and removing it raises OPEN back -
# isolates: PORTING-PLAN.md §308.4's A3 other legitimate exit (an expiry
# condition with a stated trigger, not a `거짓 → 닫힘` measurement). Two
# fixtures, same two-bullet shape: with the marker on [A], the header tallies
# EXPIRY 1 / OPEN 1; strip the marker back off (mutation) and the header
# tallies EXPIRY 0 / OPEN 2 -- the same bullet that left OPEN comes back the
# moment the marker is gone, which is the removal-raises-OPEN mutation the
# brief asked to be demonstrated, not merely asserted.
cat > "$ROOT/expiry_marker.md" <<'EOF'
## 900.14 이 절이 하지 않은 것

- **[A] 첫째. OPEN → 만료 조건 (moveit-octomap이 생기면 만료된다).** 이유.
- **[B] 둘째.** 아직 열려 있다.
EOF
expect expiry_marker "$ROOT/expiry_marker.md" 1 2 A B
expect_text expiry_marker "$ROOT/expiry_marker.md" present "(CLOSED 0 / EXPIRY 1 / OPEN 1)."
expect_text expiry_marker "$ROOT/expiry_marker.md" present \
  "moveit-octomap이 생기면 만료된다).** 이유. | EXPIRY |"
expect_text expiry_marker "$ROOT/expiry_marker.md" present "[B] 둘째.** 아직 열려 있다. | OPEN |"

cat > "$ROOT/expiry_marker_removed.md" <<'EOF'
## 900.15 이 회차가 못 본 것

- **[A] 첫째.** 이유.
- **[B] 둘째.** 아직 열려 있다.
EOF
expect expiry_marker_removed "$ROOT/expiry_marker_removed.md" 1 2 A B
expect_text expiry_marker_removed "$ROOT/expiry_marker_removed.md" present \
  "(CLOSED 0 / EXPIRY 0 / OPEN 2)."

# --- an expiry marker with no trigger sentence fails the gate, not "OPEN" ----
# isolates: A3's own condition (PORTING-PLAN.md:35208) accepts an expiry
# conversion only when it states WHEN it expires -- an empty or
# whitespace-only trigger is the disallowed unconditional "permanent, no
# trigger" exit wearing this marker's shape, and the gate must refuse to emit
# a census at all rather than silently reading it as EXPIRY or OPEN.
checked=$((checked + 1))
cat > "$ROOT/expiry_blank_trigger.md" <<'EOF'
## 900.16 이 절이 하지 않은 것

- **[A] 첫째. OPEN → 만료 조건 ().** 발화 시점을 안 적었다.
- **[B] 둘째. OPEN → 만료 조건 (   ).** 공백만 있다.
EOF
if out="$("$GATE" --doc "$ROOT/expiry_blank_trigger.md" --emit "$ROOT/expiry_blank_trigger.census" 2>&1)"; then
  failures=$((failures + 1))
  echo "FAIL scenario expiry_blank_trigger: the gate exited 0 on two blank-trigger markers" >&2
elif ! printf '%s' "$out" | grep -qF -- "FAIL 2 \`OPEN → 만료 조건 ()\` marker(s)"; then
  failures=$((failures + 1))
  echo "FAIL scenario expiry_blank_trigger: nonzero exit but not the blank-trigger message:" >&2
  printf '%s\n' "$out" | sed 's/^/       /' >&2
elif [ -f "$ROOT/expiry_blank_trigger.census" ]; then
  failures=$((failures + 1))
  echo "FAIL scenario expiry_blank_trigger: gate wrote a census despite the blank trigger" >&2
fi

# --- a lead-in with no top-level bullet is listed, not dropped ---------------
# isolates: the second population. It used to be a bare `continue`, so a section
# stating its residual claims in prose left no trace at all -- 17 of
# PORTING-PLAN.md's 60 lead-ins, the hole §301.5 recorded without a size.
cat > "$ROOT/prose_only_leadin.md" <<'EOF'
## 900.10 이 절이 하지 않은 것

주장이 프로즈로만 적혀 있다. 최상위 불릿이 하나도 없다.

## 900.11 이 회차가 못 본 것

- **[A] 이 절은 불릿이 있다.**
EOF
expect prose_only_leadin "$ROOT/prose_only_leadin.md" 1 1 A
expect_text prose_only_leadin "$ROOT/prose_only_leadin.md" present "불릿 없음 (프로즈만)"
expect_text prose_only_leadin "$ROOT/prose_only_leadin.md" present "lead-in 1건 (위 표의 1건과 별개)"

# --- lead-in, prose, then the list: recorded with the bullet it could not reach
# isolates: the forward scan that tells the two shapes apart. Collapsed to one
# shape, a section whose list is merely one paragraph away reads the same as a
# section with no list at all, and the reader cannot tell which needs writing.
cat > "$ROOT/prose_then_bullets.md" <<'EOF'
## 900.12 이 절이 재지 않은 것

먼저 설명 문단이 온다. 목록은 그 다음이다:

- 이 불릿은 lead-in 바로 아래가 아니다.

## 900.13 이 회차가 못 본 것

- **[A] 이 절은 불릿이 있다.**
EOF
expect prose_then_bullets "$ROOT/prose_then_bullets.md" 1 1 A
expect_text prose_then_bullets "$ROOT/prose_then_bullets.md" present "| 프로즈 뒤 불릿 |"
expect_text prose_then_bullets "$ROOT/prose_then_bullets.md" present "%DOC%:5 |"

# --- each row cites the document that was parsed -----------------------------
# isolates: the citation prefix. It was the literal `PORTING-PLAN.md`, so every
# row of this file's own fixtures cited PORTING-PLAN.md line numbers belonging
# to a temp file -- and the census is tracked, so `check-citation-drift.py`
# resolves those rows against whatever the prefix names.
expect_text continuation "$ROOT/continuation.md" present "| %DOC%:1 "
expect_text continuation "$ROOT/continuation.md" absent "PORTING-PLAN.md:"

# --- a document with no lead-in is a failure, not an empty census ------------
# isolates: the zero-entry guard. Without it a vocabulary change emits a census
# with no rows and exits 0.
checked=$((checked + 1))
cat > "$ROOT/no_leadin.md" <<'EOF'
## 900.9 그냥 절

- 불릿이지만 lead-in 아래가 아니다.
EOF
if out="$("$GATE" --doc "$ROOT/no_leadin.md" --emit "$ROOT/no_leadin.census" 2>&1)"; then
  failures=$((failures + 1))
  echo "FAIL scenario no_leadin: the gate exited 0 on a document with no lead-in" >&2
elif ! printf '%s' "$out" | grep -qF -- "parsed zero lead-in lists"; then
  failures=$((failures + 1))
  echo "FAIL scenario no_leadin: nonzero exit but not the zero-lead-in message:" >&2
  printf '%s\n' "$out" | sed 's/^/       /' >&2
fi

require_nonempty "$checked" "scenarios to run against check-residual-claims-census.py"

if [ "$failures" -ne 0 ]; then
  echo "FAIL $failures of $checked scenarios did not discriminate as asserted" >&2
  exit 1
fi

echo "OK check-residual-claims-census.py discriminates on all $checked scenarios:" \
     "a continuation paragraph and a blank line keep the list open, an" \
     "unindented paragraph closes it, a sibling bullet / heading / table row" \
     "ends the item, the closure marker is read per bullet, each row cites the" \
     "document that was parsed, an expiry marker with a stated trigger tallies" \
     "EXPIRY and removing it raises OPEN back, a blank expiry trigger fails" \
     "the gate outright, and a document with no lead-in is a failure" \
     "rather than an empty census"
