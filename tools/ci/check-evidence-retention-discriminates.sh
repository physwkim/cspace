#!/bin/bash
# Asserts that `check-evidence-retention.py` still fails on each thing it claims
# to fail on, and still passes on the shapes it claims are legal.
#
# That gate's product is a pair of tables and a verdict about them. A parser
# that quietly read fewer rows would print a smaller "OK N rows" line and stop
# checking the rest, and nothing downstream would notice -- the same failure
# mode `check-residual-claims-census-discriminates.sh` exists for. So each rule
# gets one scenario, and each scenario is a whole synthetic repository rather
# than this tree: a scenario that read the real PORTING-PLAN.md would change its
# own expectations every time someone wrote a section.
#
# # What each rule is worth, measured
#
# Every line below was produced by breaking exactly one of the gate's 53 rules,
# running this script's 56 scenarios, and reading which scenario names appeared.
# It is a measurement, not a reading of the code. Fourteen results are not what
# reading it predicts, and they are marked.
#
#   rule neutralized                                     scenario that actually reddens
#   ---------------------------------------------------  ------------------------------
#   census missing-row check                             new_instrument
#   census extra-row check  (*)                          gate_in_census, ghost_instrument
#   stale 자동 row check                                 deleted_citation
#   undeclared derived-pair check  (*)                   fenced_mention, unrecorded_mention
#   only_code exactly-one-token check                    bare_instrument
#   code_list one-or-more-tokens check                   artifact_with_prose
#   evidence path tracked check  (*)                     deleted_evidence_dir,
#                                                        second_evidence_path_bad, untracked_evidence
#   evidence path worktree-exists check                  deleted_evidence
#   artifact-named-in-its-own-script check               unclaimed_artifact
#   부류/산출물 agreement, 없음 direction                class_disagrees
#   부류/산출물 agreement, path direction                class_disagrees_path
#   find_table uniqueness (take the first match)  (*)    duplicate_census
#   rows-table 추적 산출물 exclusion                     tracked_class_row
#   절-resolves-to-a-heading check                       unresolvable_section
#   registry-section exclusion in derive_pairs  (*)      baseline, broken_but_runs,
#                                                        broken_for_another_reason,
#                                                        broken_reason_fixed,
#                                                        broken_with_committed_txt,
#                                                        data_file_needs_no_role, deleted_citation,
#                                                        duplicate_dir_row, empty_dir_note,
#                                                        evidence_dir_ghost, evidence_dir_unlisted,
#                                                        manual_survives_deletion, no_evidence_dirs,
#                                                        rederive_crashes, rederive_deleted,
#                                                        rederive_output_drifted,
#                                                        rederive_txt_untracked,
#                                                        reproduces_with_missing_file,
#                                                        two_evidence_paths, unknown_dir_state
#   empty-table check in find_table                      empty_rows
#   행 출처 closed token set                             unknown_origin
#   duplicate census row check                           duplicate_instrument_row
#   duplicate 자동 row check                             duplicate_auto_row
#   at-least-one-untracked-instrument floor              all_tracked
#   table row cell-count check  (*)                      wrong_cell_count
#   two-registry-tables-same-section check               split_registry
#   empty producer family floor  (*)                     no_producers
#   empty git ls-files floor  (*)                        empty_index
#   non-empty 비고 check, rows table                     empty_note
#   extra-row present/absent message split  (*)          gate_in_census
#   every 증거 path checked, not just the first          second_evidence_path_bad
#   증거 is a list, not a single token  (*)              second_evidence_path_bad, two_evidence_paths
#   every 산출물 path checked, not just the first        unclaimed_artifact
#   mention-before-any-heading check                     mention_before_heading
#   numbered-heading floor                               unnumbered_headings
#   header-row-followed-by-separator check               no_separator_row
#   registry-table-inside-a-##-section check             registry_before_heading
#   undeclared prefix is a failure                       undeclared_prefix
#   data files skip the role check                       data_file_needs_no_role
#   tools/ci is a directory                              no_tools_ci_dir
#   at-least-one-tracked-file-under-tools/ci floor  (*)  empty_family
#   reader-class instruments carry no row  (*)           reader_row
#   evidence-dir family floor (tracked doc NDJSON)       no_evidence_dirs
#   duplicate evidence-dir row                           duplicate_dir_row
#   family-from-data: unlisted directory                 evidence_dir_unlisted
#   family-from-data: ghost row                          evidence_dir_ghost
#   non-empty 비고 check, dirs table                     empty_dir_note
#   rederive.py must be tracked                          rederive_deleted
#   재현됨: 빠진 파일 must be 없음                       reproduces_with_missing_file
#   재현됨: rc must be 0                                 rederive_crashes
#   재현됨: rederive.txt must be tracked  (*)            rederive_txt_untracked
#   재현됨: byte comparison against rederive.txt         rederive_output_drifted
#   깨짐: the named file must really be absent           broken_reason_fixed
#   깨짐: the script must really fail  (*)               broken_but_runs
#   깨짐: the failure must name that file                broken_for_another_reason
#   깨짐: no committed rederive.txt                      broken_with_committed_txt
#   상태 closed token set                                unknown_dir_state
#
# (*) the fourteen that reading the code gets wrong:
#
#   - Dropping the extra-row check reddens `gate_in_census` too: a census row for
#     a `check-*` script is an extra row, not a separate rule. What the
#     present/absent split buys is only the MESSAGE -- without it the gate says
#     "names no file in tools/ci/" about a file that is sitting right there,
#     which sends the reader hunting a typo instead of reading the role table.
#   - Without the reader rule `reader_row` still exits 1, via the
#     추적 산출물 branch -- and that message is FALSE, the instrument is a
#     reader. The guard buys a true reason, not a failure.
#   - Reverting 증거 to a single token reddens `second_evidence_path_bad` too,
#     and on the WRONG message: the row never reaches `check_evidence_path`,
#     it dies on "must be exactly one token". So the two list scenarios do not
#     pin two independent rules under that neutralization -- only the [0]-slice
#     neutralization isolates the per-path loop, and it reddens exactly one.
#   - The undeclared check is one rule with two shapes. Dropping it reddens both
#     the prose mention and the fenced one; they are not separate rules, and a
#     fixture set with only the prose one would leave the fence claim unproven.
#   - Making the tracked test unconditional also reddens `deleted_evidence_dir`,
#     which is nominally about a different branch: with the tracked test always
#     taken, a deleted directory reports "tracked but missing" instead of the
#     directory message. The three branches of `check_evidence_path` do not
#     neutralize independently.
#   - `find_table`'s `len(starts) != 1` looks like one rule and is two.
#     Weakening it to "the first match wins" reddens `duplicate_census` and
#     leaves `missing_census` red on its own, because zero matches still fail.
#     A fixture set with only `missing_census` would have measured this rule as
#     fully covered while the duplicate half was gone.
#   - Without the cell-count check `wrong_cell_count` still exits non-zero, by
#     an uncaught ValueError from tuple unpacking. The scenario reddens on the
#     MESSAGE, not the exit code -- so what that guard buys is a named failure,
#     and an expectation written as "it exits non-zero" would have called the
#     guard unnecessary.
#   - Dropping the registry-section exclusion reddens 20 of the 56 scenarios,
#     `baseline` and `manual_survives_deletion` -- the two that are supposed to
#     PASS -- among them. It reddened three before the directory table existed;
#     the new scenarios did not add a rule, they widened one blast radius,
#     because every fixture's census names the instruments in its own cells. No
#     dedicated fixture detects this rule; the pass cases are the detector.
#   - The empty-`git ls-files` floor reddened NOTHING on the first pass of this
#     sweep: every other rule had a scenario and that one had none. `empty_index`
#     below was written afterwards for it. An unexercised guard is a claim, and
#     it took the sweep, not a reading, to find which one it was.
#   - Two of the three floors under the instrument family -- "one tracked file
#     under tools/ci" and "one producer among them" -- both measured as
#     reddening NOTHING, and neither was unexercised. `empty_family` asserted
#     the shared tail "would report OK having examined nothing", which BOTH
#     messages end with, so each floor's failure satisfied the other's
#     expectation. Narrowing that assertion to the head of the message and
#     adding `no_producers` separated them. An expectation string is part of the
#     rule a fixture pins, not commentary on it.
#   - The exit-code test on the 깨짐 branch also reddened nothing:
#     `broken_reason_fixed` commits the missing file and dies one check earlier,
#     on the absent-file test, so nothing ever reached this one.
#     `broken_but_runs` -- named file still absent, script fixed -- was written
#     for it afterwards.
#   - So did the tracked test on `rederive.txt`. `rederive_txt_untracked` was
#     written for it afterwards. Both of these were found by the same sweep, in
#     the same round that added the rules; a rule and its fixture written
#     together are not evidence that the fixture reaches the rule.
#   - A fourteenth rule is not in the table because it was deleted: the span
#     logic also checked that the rows table's last row falls inside the
#     excluded `##` span, and no fixture could reach it. The equality above it
#     already forces both tables into one span, and a table's rows are
#     contiguous `|` lines, so the last row cannot leave the span its header is
#     in without a `##` between them -- which would have split the table first.
#     Unreachable, so removed rather than described.
#
# # The fixture spelling
#
# Fixture section numbers are written bare (`## 900`) and so are the rows
# table's 절 cells. `check-section-references.sh` scans this file's own text and
# cannot tell a constructed fixture from a citation, so a made-up section sign
# here would be a dangling reference to it. The gate accepts both spellings; no
# scenario below is about which one is used. For the same reason no fixture path
# is written with a `:NNN` suffix, which `check-citation-drift.py` would read as
# an in-repo citation to a temp directory.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

GATE="$REPO_ROOT/tools/ci/check-evidence-retention.py"
if [[ ! -x "$GATE" ]]; then
  echo "FAIL $GATE is missing or not executable -- there is nothing to test" >&2
  exit 2
fi

ROOT="$(mktemp -d)"
trap 'rm -rf "$ROOT"' EXIT
failures=0
checked=0

# build_fixture <dir>
#
# A whole synthetic repository: three instruments, TWO tracked artifacts behind
# one of them, a tracked evidence directory, and one spare tracked file. The two
# artifacts are not decoration -- `measure-upstream-citations.py` regenerates
# two files in the real tree, so a fixture with one per instrument would leave
# the multi-artifact path unexercised. `git add` alone is enough: the gate reads
# `git ls-files`, which is the index, so no commit and no user identity is
# needed here.
build_fixture() {
  local d="$1"
  mkdir -p "$d/tools/ci" "$d/doc/evidence"
  printf '#!/bin/bash\n# writes doc/alpha-artifact.md and doc/alpha-second.md\n' > "$d/tools/ci/measure-alpha.sh"
  printf '#!/bin/bash\n# sweeps into a directory it is handed; writes nothing tracked\n' > "$d/tools/ci/measure-beta.sh"
  printf '#!/usr/bin/env python3\n"""reads the tree and prints."""\n' > "$d/tools/ci/measure-gamma.py"
  printf 'artifact\n' > "$d/doc/alpha-artifact.md"
  printf 'second artifact\n' > "$d/doc/alpha-second.md"
  printf '{"id": 0}\n' > "$d/doc/evidence/run.ndjson"
  printf '{"id": 1}\n' > "$d/doc/other-run.ndjson"
  # The evidence directory carries the shape §305.6 checks: a re-derivation
  # script beside the data and its committed output. `other-run.ndjson` sits at
  # doc/ top level on purpose -- its parent is a second family member, so the
  # family is two directories and one row can never vouch for both.
  printf '#!/usr/bin/env python3\nimport json, pathlib\nd = pathlib.Path(__file__).resolve().parent\nn = len(json.loads((d / "run.ndjson").read_text())) \nprint(f"fields {n}")\n' > "$d/doc/evidence/rederive.py"
  printf 'fields 1\n' > "$d/doc/evidence/rederive.txt"
  printf '#!/usr/bin/env python3\nimport pathlib\nd = pathlib.Path(__file__).resolve().parent\nprint((d / "gone.ndjson").read_text())\n' > "$d/doc/rederive.py"
  git -C "$d" -c init.defaultBranch=main init -q
  git -C "$d" add -A
}

# plan <dir> [beta-mention:yes|no] [gamma-mention:yes|no]
#
# The baseline document. The two mention arguments are the sentences that make a
# pair derivable; a scenario drops one to test what the gate does when a
# citation disappears.
plan() {
  local d="$1" beta_mention="${2-yes}" gamma_mention="${3-yes}"
  {
    echo '## 900 증거 보존'
    echo
    echo '### 900.1 계측기 전수'
    echo
    echo '| 계측기 | 산출물 | 부류 |'
    echo '|---|---|---|'
    echo '| `measure-alpha.sh` | `doc/alpha-artifact.md`, `doc/alpha-second.md` | 추적 산출물 |'
    echo '| `measure-beta.sh` | 없음 | 미보존 산출물 |'
    echo '| `measure-gamma.py` | 없음 | 트리에서 재실행 |'
    echo
    echo '### 900.2 출판 행'
    echo
    echo '| 계측기 | 절 | 증거 | 행 출처 | 비고 |'
    echo '|---|---|---|---|---|'
    echo '| `measure-beta.sh` | 901.1 | `doc/evidence/` | 자동 | 스윕 출력이 그대로 있다 |'
    echo '| `measure-beta.sh` | 901.2 | 없음 | 수동 | 같은 실행, 스크립트 이름 없음 |'
    echo '| `measure-gamma.py` | 901.3 | 없음 | 자동 | 다시 돌리면 나온다 |'
    echo
    echo '### 900.3 증거 디렉터리'
    echo
    echo '| 증거 디렉터리 | 상태 | 빠진 파일 | 비고 |'
    echo '|---|---|---|---|'
    echo '| `doc/evidence/` | 재현됨 | 없음 | 출력이 커밋된 rederive.txt와 같다 |'
    echo '| `doc/` | 깨짐 | `doc/gone.ndjson` | 그 팔이 커밋되지 않았다 |'
    echo
    echo '## 901 측정한 절'
    echo
    echo '### 901.1 스윕'
    echo
    if [ "$beta_mention" = yes ]; then
      echo '이 표는 measure-beta.sh 실행에서 나왔다.'
    else
      echo '이 표는 어느 스윕에서 나왔다.'
    fi
    echo
    echo '### 901.2 같은 스윕의 다른 표'
    echo
    echo '스크립트를 이름으로 적지 않고 같은 실행의 수를 싣는다.'
    echo
    echo '### 901.3 트리에서 재실행'
    echo
    if [ "$gamma_mention" = yes ]; then
      echo '    $ tools/ci/measure-gamma.py'
    else
      echo '    $ 어떤 계측기'
    fi
  } > "$d/PLAN.md"
}

# new <name> -> echoes a fresh fixture dir with the baseline plan in it
new() {
  local d="$ROOT/$1"
  build_fixture "$d"
  plan "$d"
  echo "$d"
}

# expect_ok <name> <dir>
expect_ok() {
  local name="$1" d="$2" out status
  checked=$((checked + 1))
  if out="$("$GATE" --root "$d" --doc "$d/PLAN.md" 2>&1)"; then
    status=0
  else
    status=$?
  fi
  if [ "$status" -ne 0 ]; then
    failures=$((failures + 1))
    echo "FAIL scenario $name: expected exit 0, got $status" >&2
    printf '%s\n' "$out" | sed 's/^/       /' >&2
  fi
}

# expect_fail <name> <dir> <substring the message must carry>
#
# The substring matters: a scenario that only asserted "it failed" would pass
# when the gate failed for an unrelated reason, which is how a fixture stops
# testing the rule it is named after.
expect_fail() {
  local name="$1" d="$2" want="$3" out status
  checked=$((checked + 1))
  if out="$("$GATE" --root "$d" --doc "$d/PLAN.md" 2>&1)"; then
    status=0
  else
    status=$?
  fi
  if [ "$status" -eq 0 ]; then
    failures=$((failures + 1))
    echo "FAIL scenario $name: expected a failure, the gate exited 0" >&2
    printf '%s\n' "$out" | sed 's/^/       /' >&2
    return
  fi
  if ! printf '%s' "$out" | grep -qF -- "$want"; then
    failures=$((failures + 1))
    echo "FAIL scenario $name: exited $status but the message does not carry '$want'" >&2
    printf '%s\n' "$out" | sed 's/^/       /' >&2
  fi
}

# --- the shape that must pass ------------------------------------------------
# isolates: that a well-formed registry is accepted at all, and that the
# registry's OWN section -- which names all three instruments, in its tables --
# is excluded from the derivation. Without that exclusion every table cell would
# demand a row about itself.
d="$(new baseline)"
expect_ok baseline "$d"

# --- a new instrument on disk has no row -------------------------------------
# isolates: census exactness in the direction nothing written in the plan can
# silence. This is the one absence claim this gate can make.
d="$(new new_instrument)"
printf '#!/bin/bash\n' > "$d/tools/ci/measure-delta.sh"
git -C "$d" add -A
expect_fail new_instrument "$d" "have no census row: measure-delta.sh"

# --- a census row naming no file ---------------------------------------------
# isolates: the other direction of the same exactness. A row can outlive the
# script it names, and then the table describes a tree nobody has.
d="$(new ghost_instrument)"
sed -i 's#^| `measure-gamma.py` | 없음 | 트리에서 재실행 |#&\n| `measure-zeta.sh` | 없음 | 미보존 산출물 |#' "$d/PLAN.md"
expect_fail ghost_instrument "$d" "name no file in tools/ci/: measure-zeta.sh"

# --- one instrument, two census rows -----------------------------------------
# isolates: uniqueness within the census. Two rows can carry two different
# classes, and the set comparison further down would still balance.
d="$(new duplicate_instrument_row)"
sed -i 's#^| `measure-beta.sh` | 없음 | 미보존 산출물 |#&\n| `measure-beta.sh` | 없음 | 트리에서 재실행 |#' "$d/PLAN.md"
expect_fail duplicate_instrument_row "$d" "has a second census row"

# --- deleting the citation breaks the row, it does not silence it ------------
# isolates: the constraint that this gate must not be satisfiable by removing
# the sentence that names the instrument. The 자동 row stops being derived and
# becomes an extra one.
d="$(new deleted_citation)"
plan "$d" no yes
expect_fail deleted_citation "$d" "no longer names the instrument"

# --- ... and the escape hatch keeps the disposition in the table -------------
# isolates: that 수동 is the only way out, and that taking it leaves the
# evidence column standing where it was.
d="$(new manual_survives_deletion)"
plan "$d" no yes
sed -i 's#^| `measure-beta.sh` | 901.1 | \(.*\) | 자동 |#| `measure-beta.sh` | 901.1 | \1 | 수동 |#' "$d/PLAN.md"
expect_ok manual_survives_deletion "$d"

# --- a section that names an instrument with no row --------------------------
# isolates: the derived-but-undeclared direction. A new section quoting a sweep
# fails until someone rules on its evidence.
d="$(new unrecorded_mention)"
printf '\n### 901.4 새 절\n\nmeasure-beta.sh 를 다시 돌렸다.\n' >> "$d/PLAN.md"
expect_fail unrecorded_mention "$d" "have no 자동 row"

# --- a mention inside a fenced block still counts ----------------------------
# isolates: that the derivation does not skip code fences. If it did, moving the
# sentence into a reproduction recipe would silence the rule.
d="$(new fenced_mention)"
{ printf '\n### 901.5 재현\n\n'; printf '```\n$ tools/ci/measure-beta.sh out\n```\n'; } >> "$d/PLAN.md"
expect_fail fenced_mention "$d" "have no 자동 row"

# --- the same pair claimed twice ---------------------------------------------
# isolates: uniqueness within the 자동 rows. Set arithmetic alone cannot see a
# duplicate, so the published 자동 count would exceed the derived pair count.
d="$(new duplicate_auto_row)"
sed -i 's#^| `measure-gamma.py` | 901.3 | 없음 | 자동 | .*#&\n&#' "$d/PLAN.md"
expect_fail duplicate_auto_row "$d" "has a second 자동 row"

# --- an evidence path that is present but untracked --------------------------
# isolates: the tracked test. A file that exists only in someone's worktree is
# exactly the state this gate is for -- it is what a scratch directory looks
# like from inside the session that made it.
d="$(new untracked_evidence)"
mkdir -p "$d/doc/scratch"
printf '{"id": 0}\n' > "$d/doc/scratch/run.ndjson"
sed -i 's#| 901.1 | `doc/evidence/` |#| 901.1 | `doc/scratch/run.ndjson` |#' "$d/PLAN.md"
expect_fail untracked_evidence "$d" "is not a tracked file and no tracked file lives under it"

# --- an evidence path that is tracked and gone -------------------------------
# isolates: the worktree test, which the tracked test does not imply.
d="$(new deleted_evidence)"
sed -i 's#| 901.1 | `doc/evidence/` |#| 901.1 | `doc/other-run.ndjson` |#' "$d/PLAN.md"
rm "$d/doc/other-run.ndjson"
expect_fail deleted_evidence "$d" "is tracked but missing from the worktree"

# --- two evidence paths in one cell, both good --------------------------------
# isolates: that 증거 is a list, not a single token. One section's figures can
# need two producers' output at once, and before this was a list the real
# PORTING-PLAN.md's §269.4 row could not be written at all.
d="$(new two_evidence_paths)"
sed -i 's#| 901.1 | `doc/evidence/` |#| 901.1 | `doc/evidence/`, `doc/other-run.ndjson` |#' "$d/PLAN.md"
expect_ok two_evidence_paths "$d"

# --- two evidence paths, the second one bad ----------------------------------
# isolates: that EVERY path in the list is checked, not just the first. A list
# whose head resolves would otherwise vouch for whatever follows it.
d="$(new second_evidence_path_bad)"
mkdir -p "$d/doc/scratch"
printf '{"id": 0}\n' > "$d/doc/scratch/run.ndjson"
sed -i 's#| 901.1 | `doc/evidence/` |#| 901.1 | `doc/evidence/`, `doc/scratch/run.ndjson` |#' "$d/PLAN.md"
expect_fail second_evidence_path_bad "$d" "is not a tracked file and no tracked file lives under it"

# --- an evidence directory that is gone --------------------------------------
# isolates: the third branch of the same helper. A directory pointer resolves
# through the index, so deleting the directory leaves the tracked files under it
# still listed and the pointer still looking satisfied.
d="$(new deleted_evidence_dir)"
rm -rf "$d/doc/evidence"
expect_fail deleted_evidence_dir "$d" "but is not a directory in the worktree"

# --- an artifact the instrument never names ----------------------------------
# isolates: the weakest of the three artifact tests, and the one that stops a
# row from pairing a script with a tracked file it has nothing to do with.
d="$(new unclaimed_artifact)"
sed -i 's#^| `measure-alpha.sh` | .* | 추적 산출물 |#| `measure-alpha.sh` | `doc/alpha-artifact.md`, `doc/evidence/run.ndjson` | 추적 산출물 |#' "$d/PLAN.md"
expect_fail unclaimed_artifact "$d" "does not appear anywhere in tools/ci/measure-alpha.sh"

# --- a 산출물 cell that is a list plus prose ---------------------------------
# isolates: the list cell's shape, which is looser than the one-token rule the
# 계측기 column uses and therefore needs its own boundary. Everything outside
# the backticks must be separators, so "`a`, `b` 정도" is a failure rather than
# a two-path row with a hedge attached.
d="$(new artifact_with_prose)"
sed -i 's#^| `measure-alpha.sh` | \(.*\) | 추적 산출물 |#| `measure-alpha.sh` | \1 정도 | 추적 산출물 |#' "$d/PLAN.md"
expect_fail artifact_with_prose "$d" "산출물 must be one or more"

# --- 부류 claiming an artifact the row does not have -------------------------
# isolates: the class/artifact agreement, in the direction that would let an
# instrument be filed as retained while its 산출물 cell says 없음.
d="$(new class_disagrees)"
sed -i 's#^| `measure-beta.sh` | 없음 | 미보존 산출물 |#| `measure-beta.sh` | 없음 | 추적 산출물 |#' "$d/PLAN.md"
expect_fail class_disagrees "$d" "산출물 없음 but 부류 추적 산출물"

# --- ... and the other direction ---------------------------------------------
# isolates: the same agreement where a row names a real artifact but files the
# instrument as one whose output is untracked, which would drag it into the rows
# table's subject.
d="$(new class_disagrees_path)"
sed -i 's#^| `measure-alpha.sh` | \(.*\) | 추적 산출물 |#| `measure-alpha.sh` | \1 | 미보존 산출물 |#' "$d/PLAN.md"
expect_fail class_disagrees_path "$d" "but 부류 미보존 산출물"

# --- a 추적 산출물 instrument in the rows table -----------------------------
# isolates: the rows table's subject. An instrument with a committed artifact
# and a --check mode has no evidence question, and letting it in would dilute
# the count the OK line publishes.
d="$(new tracked_class_row)"
sed -i 's#^| `measure-gamma.py` | 901.3 | 없음 | 자동 | .*#&\n| `measure-alpha.sh` | 901.2 | 없음 | 수동 | 추적 부류인데 여기 있다 |#' "$d/PLAN.md"
expect_fail tracked_class_row "$d" "the rows table is only about instruments whose output is not tracked"

# --- every census row filed as 추적 산출물 -----------------------------------
# isolates: the floor under the rows table. If nothing is declared untracked the
# rows table is about nothing, and the gate would print an OK line having
# checked no evidence pointer at all.
d="$(new all_tracked)"
printf '#!/bin/bash\n# writes doc/beta-artifact.md\n' > "$d/tools/ci/measure-beta.sh"
printf '#!/usr/bin/env python3\n"""writes doc/gamma-artifact.md."""\n' > "$d/tools/ci/measure-gamma.py"
printf 'b\n' > "$d/doc/beta-artifact.md"
printf 'g\n' > "$d/doc/gamma-artifact.md"
git -C "$d" add -A
sed -i 's#^| `measure-beta.sh` | 없음 | 미보존 산출물 |#| `measure-beta.sh` | `doc/beta-artifact.md` | 추적 산출물 |#' "$d/PLAN.md"
sed -i 's#^| `measure-gamma.py` | 없음 | 트리에서 재실행 |#| `measure-gamma.py` | `doc/gamma-artifact.md` | 추적 산출물 |#' "$d/PLAN.md"
expect_fail all_tracked "$d" "the rows table would then have nothing to be about"

# --- a 절 that resolves to no heading ----------------------------------------
# isolates: that a row cannot point at a section nobody wrote.
d="$(new unresolvable_section)"
sed -i 's#| `measure-beta.sh` | 901.2 |#| `measure-beta.sh` | 909.9 |#' "$d/PLAN.md"
expect_fail unresolvable_section "$d" "resolves to no heading"

# --- an empty 비고 cell ------------------------------------------------------
# isolates: that every row must say what its evidence reaches. 증거 is one cell
# and coverage is often partial -- §300.2 in the real tree publishes both a
# population split its committed NDJSON re-derives and a wall-clock table it
# does not -- so a row that names a path and says nothing else overstates.
d="$(new empty_note)"
sed -i 's#^| `measure-beta.sh` | 901.1 | \(.*\) | 자동 | .*#| `measure-beta.sh` | 901.1 | \1 | 자동 |  |#' "$d/PLAN.md"
expect_fail empty_note "$d" "비고 is empty"

# --- an unknown 행 출처 token ------------------------------------------------
# isolates: the closed token set. A typo must fail rather than fall into a
# default branch and be counted as neither.
d="$(new unknown_origin)"
sed -i 's#| `measure-beta.sh` | 901.2 | 없음 | 수동 |#| `measure-beta.sh` | 901.2 | 없음 | 아마도 |#' "$d/PLAN.md"
expect_fail unknown_origin "$d" "행 출처 must be"

# --- a cell that is not a backticked token -----------------------------------
# isolates: the cell shape. A bare token would let prose ("아마 measure-beta.sh
# 였다") sit in a column the gate then compares against a filesystem set.
d="$(new bare_instrument)"
sed -i 's#^| `measure-beta.sh` | 없음 | 미보존 산출물 |#| measure-beta.sh | 없음 | 미보존 산출물 |#' "$d/PLAN.md"
expect_fail bare_instrument "$d" "계측기 must be exactly one"

# --- the table is missing ----------------------------------------------------
# isolates: that a document this gate cannot parse is a FAILURE. Renaming a
# column must not turn the gate off.
d="$(new missing_census)"
sed -i 's#^| 계측기 | 산출물 | 부류 |#| 계측기 | 산출물 | 종류 |#' "$d/PLAN.md"
expect_fail missing_census "$d" "found 0"

# --- the table appears twice -------------------------------------------------
# isolates: the uniqueness half of the same rule. Two registries mean two
# answers, and taking the first would leave the second unchecked.
d="$(new duplicate_census)"
{
  echo
  echo '### 900.3 두 번째 전수'
  echo
  echo '| 계측기 | 산출물 | 부류 |'
  echo '|---|---|---|'
  echo '| `measure-beta.sh` | 없음 | 미보존 산출물 |'
} >> "$d/PLAN.md"
expect_fail duplicate_census "$d" "found 2"

# --- a table with a header and no rows ---------------------------------------
# isolates: the zero-row guard. Without it, emptying the rows table is a way to
# pass having checked nothing.
d="$(new empty_rows)"
sed -i '/^| `measure-beta.sh` | 901/d; /^| `measure-gamma.py` | 901/d' "$d/PLAN.md"
expect_fail empty_rows "$d" "has a header and no rows"

# --- a row with the wrong number of cells ------------------------------------
# isolates: the shape check. A dropped `|` must not silently shift the columns.
d="$(new wrong_cell_count)"
sed -i 's#^| `measure-beta.sh` | 901.2 | 없음 | 수동 |#| `measure-beta.sh` | 901.2 | 수동 |#' "$d/PLAN.md"
expect_fail wrong_cell_count "$d" "4 cells, expected 5"

# --- a header row with no separator under it ---------------------------------
# isolates: the separator test. Without it the parser starts one line late and
# reads the table minus its first row -- a smaller table, a smaller OK line, and
# nothing that says a row went missing.
d="$(new no_separator_row)"
sed -i '/^|---|---|---|---|---|$/d' "$d/PLAN.md"
expect_fail no_separator_row "$d" "is not followed by a separator row"

# --- a document whose headings carry no number -------------------------------
# isolates: the heading floor. 절 cells resolve against the heading set, so an
# empty set would make every row's section unresolvable at once -- or, with the
# resolution check gone too, make them all vacuously fine.
d="$(new unnumbered_headings)"
sed -i 's/^\(#\+\) [0-9][0-9.]* /\1 절 /' "$d/PLAN.md"
expect_fail unnumbered_headings "$d" "no numbered headings"

# --- the registry tables under no `##` section -------------------------------
# isolates: the span that gets excluded from the derivation. If the tables sit
# under no top-level heading there is nothing to exclude, and every instrument
# name inside the census itself would derive a pair about its own row.
d="$(new registry_before_heading)"
sed -i '/^## 900 증거 보존$/d' "$d/PLAN.md"
expect_fail registry_before_heading "$d" "sits outside any"

# --- an instrument named before the first heading ----------------------------
# isolates: that a mention which belongs to no section is a failure rather than
# a pair with an empty section. The plan is edited top-down and a line can end up
# above the first heading; a pair keyed on None would then be compared against
# rows that can never contain it.
d="$(new mention_before_heading)"
sed -i '1i measure-beta.sh 를 첫 제목보다 먼저 적었다.' "$d/PLAN.md"
expect_fail mention_before_heading "$d" "before any numbered heading"

# --- the two tables in different top-level sections --------------------------
# isolates: the exclusion span. The gate removes one section from the
# derivation, so two registries in two sections means one of them derives pairs
# about itself.
d="$(new split_registry)"
sed -i 's%^### 900.2 출판 행$%## 902 다른 절의 출판 행%' "$d/PLAN.md"
expect_fail split_registry "$d" 'different `##` sections'

# --- an empty index ----------------------------------------------------------
# isolates: the other floor, on the tracked set. Every evidence verdict in this
# gate is "is it tracked", so a `git ls-files` that returns nothing would make
# every 없음 row legal and every path row illegal, uniformly and silently. This
# scenario exists because the first neutralization pass measured that guard as
# reddening nothing -- it had no fixture, and an unexercised guard is a claim.
d="$ROOT/empty_index"
build_fixture "$d"
plan "$d"
git -C "$d" rm -r --cached . -q
expect_fail empty_index "$d" "this gate would check nothing"

# --- nothing tracked under tools/ci ------------------------------------------
# isolates: the floor on the DIRECTORY LISTING. The expectation was once the
# shared tail "would report OK having examined nothing", and the sweep measured
# this scenario as reddening for neither this floor nor the producer floor below
# -- both messages end in that sentence, so each masked the other. The
# discriminating half of the message is the head.
d="$(new empty_family)"
rm "$d/tools/ci"/measure-*
git -C "$d" add -A
expect_fail empty_family "$d" "no tracked file under tools/ci/"

# --- tools/ci tracked, but not one producer among them -----------------------
# isolates: the floor on the PARTITION's output, which is a different claim from
# the one above: the directory is populated and every name in it was classified,
# just never as something that measures. Without it an all-gates tools/ci would
# reach the census with an empty family and fail on whatever rows were there.
d="$(new no_producers)"
rm "$d/tools/ci"/measure-*
printf '#!/bin/bash\n# a gate\n' > "$d/tools/ci/check-something.sh"
git -C "$d" add -A
expect_fail no_producers "$d" "no producer under tools/ci/"

# --- tools/ci is not a directory in the worktree -----------------------------
# isolates: the third floor in the same chain. The index still lists the scripts,
# so the family would enumerate; what is gone is the directory this gate reads
# the scripts OUT of, which is what a partial checkout looks like.
d="$(new no_tools_ci_dir)"
rm -rf "$d/tools/ci"
expect_fail no_tools_ci_dir "$d" "is not a directory under"

# --- a tools/ci script whose prefix has no declared role ---------------------
# isolates: the prefix partition. This is the rule the first cut of the gate did
# not have, and its absence is exactly how `compare-phase8-port-vs-cpp.py` --
# the script that computed §269.4's four-way split -- stayed outside the census.
d="$(new undeclared_prefix)"
printf '#!/bin/bash\n# a new producing shape nobody classified\n' > "$d/tools/ci/frobnicate-things.sh"
git -C "$d" add -A
expect_fail undeclared_prefix "$d" "no declared role for its prefix"

# --- a data file with no declared prefix is not a script ---------------------
# isolates: the extension test that runs BEFORE the prefix test. Without it the
# real tree's four tracked `.json`/`.txt` inputs would each demand a role.
d="$(new data_file_needs_no_role)"
printf '{}\n' > "$d/tools/ci/frobnicate-inputs.json"
git -C "$d" add -A
expect_ok data_file_needs_no_role "$d"

# --- a gate given a census row -----------------------------------------------
# isolates: that role, not existence, decides census membership -- and that the
# message says so. `check-*` runs in CI every time, so its verdict is live and
# it has nothing to retain.
d="$(new gate_in_census)"
printf '#!/bin/bash\n# a gate\n' > "$d/tools/ci/check-something.sh"
git -C "$d" add -A
sed -i 's#^| `measure-gamma.py` | 없음 | 트리에서 재실행 |#| `measure-gamma.py` | 없음 | 트리에서 재실행 |\n| `check-something.sh` | 없음 | 트리에서 재실행 |#' "$d/PLAN.md"
expect_fail gate_in_census "$d" "not a producer: check-something.sh"

# --- a reader given a publishing row -----------------------------------------
# isolates: the reader rule. A reader's numbers come out of a file some producer
# wrote, so its retention obligation is already carried by that producer's row;
# a second row for the reader would double-count the same obligation and make an
# unretained sweep look covered twice.
d="$(new reader_row)"
printf '#!/usr/bin/env python3\n"""reads two NDJSON handed to it and prints."""\n' > "$d/tools/ci/compare-two-arms.py"
git -C "$d" add -A
sed -i 's#^| `measure-gamma.py` | 없음 | 트리에서 재실행 |#| `measure-gamma.py` | 없음 | 트리에서 재실행 |\n| `compare-two-arms.py` | 없음 | 입력이 증거 |#' "$d/PLAN.md"
sed -i 's#^| `measure-gamma.py` | 901.3 | 없음 | 자동 | 다시 돌리면 나온다 |#| `measure-gamma.py` | 901.3 | 없음 | 자동 | 다시 돌리면 나온다 |\n| `compare-two-arms.py` | 901.2 | 없음 | 수동 | 이 행이 있으면 안 된다 |#' "$d/PLAN.md"
expect_fail reader_row "$d" "입력이 증거"

# --- a rederive.py whose output no longer matches its committed txt ----------
# isolates: the byte-comparison. Everything else about the directory is intact
# -- script tracked, txt tracked, exit 0 -- and it is still not evidence.
d="$(new rederive_output_drifted)"
printf '{"id": 0, "extra": 1}\n' > "$d/doc/evidence/run.ndjson"
git -C "$d" add -A
expect_fail rederive_output_drifted "$d" "differs from the committed"

# --- a rederive.py that does not run -----------------------------------------
# isolates: that the script is RUN, not just looked for. A 재현됨 row over a
# script that dies is the exact state this table exists to make impossible.
d="$(new rederive_crashes)"
rm "$d/doc/evidence/run.ndjson"
git -C "$d" rm -q --cached doc/evidence/run.ndjson
printf '{"id": 0}\n' > "$d/doc/evidence/other.ndjson"
git -C "$d" add -A
expect_fail rederive_crashes "$d" "says 재현됨 but exited"

# --- an evidence directory with no rederive.py -------------------------------
# isolates: the direction that makes deleting the script a failure. Taking the
# family from the NDJSON rather than from the script is the whole reason.
d="$(new rederive_deleted)"
rm "$d/doc/evidence/rederive.py"
git -C "$d" rm -q --cached doc/evidence/rederive.py
expect_fail rederive_deleted "$d" "is not tracked -- a directory of committed"

# --- a 깨짐 row whose named file is present ----------------------------------
# isolates: that 깨짐 is a PIN, not a waiver. Commit the missing arm and the row
# has to be rewritten; it cannot keep excusing a directory that now works.
d="$(new broken_reason_fixed)"
printf 'x\n' > "$d/doc/gone.ndjson"
git -C "$d" add -A
expect_fail broken_reason_fixed "$d" "is present now"

# --- a 깨짐 row whose script has started working ------------------------------
# isolates: the exit-code test on the 깨짐 branch, which reddened NOTHING on the
# first sweep -- `broken_reason_fixed` dies on the earlier absent-file test.
# Here the named file is still absent and the script succeeds anyway, so only
# this rule can catch it.
d="$(new broken_but_runs)"
printf '#!/usr/bin/env python3\nprint("nothing to do")\n' > "$d/doc/rederive.py"
git -C "$d" add -A
expect_fail broken_but_runs "$d" "exited 0"

# --- a 재현됨 row whose rederive.txt is not tracked ---------------------------
# isolates: the tracked test on the output, which also reddened nothing at
# first. The script runs and its output is right; what is missing is the
# committed copy, so nothing in any commit pins what it printed.
d="$(new rederive_txt_untracked)"
git -C "$d" rm -q --cached doc/evidence/rederive.txt
expect_fail rederive_txt_untracked "$d" "nothing pins"

# --- a 깨짐 row whose script fails for another reason -------------------------
# isolates: that the pinned reason is checked against the actual failure. A row
# that named any absent file would otherwise absorb every future breakage.
d="$(new broken_for_another_reason)"
printf '#!/usr/bin/env python3\nraise SystemExit("unrelated")\n' > "$d/doc/rederive.py"
git -C "$d" add -A
expect_fail broken_for_another_reason "$d" "for none of the reasons the row names"

# --- a 깨짐 row that ships a rederive.txt anyway ------------------------------
# isolates: that a committed output whose script cannot reproduce it is worse
# than none -- nothing in the tree can say what produced it.
d="$(new broken_with_committed_txt)"
printf 'stale output\n' > "$d/doc/rederive.txt"
git -C "$d" add -A
expect_fail broken_with_committed_txt "$d" "is tracked while the row says"

# --- a directory of tracked NDJSON with no row -------------------------------
# isolates: the family-from-data direction. Adding evidence without recording it
# is the shape this whole section is about.
d="$(new evidence_dir_unlisted)"
mkdir -p "$d/doc/newarm"
printf '{"id": 0}\n' > "$d/doc/newarm/run.ndjson"
git -C "$d" add -A
expect_fail evidence_dir_unlisted "$d" "have no row: doc/newarm"

# --- a row naming a directory with no tracked NDJSON -------------------------
# isolates: the other direction of the same set equality.
# The row is ADDED, not swapped: swapping leaves the real directory unlisted and
# the gate fails on that first, which is a true message about the other rule.
d="$(new evidence_dir_ghost)"
sed -i 's#^\(| `doc/evidence/` | 재현됨 |.*\)$#\1\n| `doc/nowhere/` | 재현됨 | 없음 | 존재하지 않는다 |#' "$d/PLAN.md"
expect_fail evidence_dir_ghost "$d" "no tracked doc NDJSON"

# --- an unknown 상태 token ----------------------------------------------------
# isolates: the closed token set. A third word must not read as "not 깨짐".
d="$(new unknown_dir_state)"
sed -i 's#^| `doc/evidence/` | 재현됨 |#| `doc/evidence/` | 아마도 |#' "$d/PLAN.md"
expect_fail unknown_dir_state "$d" "상태 must be"

# --- one directory, two rows --------------------------------------------------
# isolates: uniqueness in the directory table. The set comparison below works on
# sets, so a duplicated row balances against the family perfectly and two rows
# could carry two different 상태 for the same directory.
d="$(new duplicate_dir_row)"
sed -i 's#^| `doc/evidence/` | 재현됨 | 없음 | .*#&\n| `doc/evidence/` | 깨짐 | `doc/evidence/run.ndjson` | 같은 디렉터리의 두 번째 행 |#' "$d/PLAN.md"
expect_fail duplicate_dir_row "$d" "has a second row"

# --- an empty 비고 in the directory table ------------------------------------
# isolates: that this table's 비고 is required too. It is a separate check from
# the rows table's, in a separate function, and the sweep measured it as
# reddening nothing until this scenario existed.
d="$(new empty_dir_note)"
sed -i 's#^| `doc/evidence/` | 재현됨 | 없음 | .*#| `doc/evidence/` | 재현됨 | 없음 |  |#' "$d/PLAN.md"
expect_fail empty_dir_note "$d" "비고 is empty"

# --- a 재현됨 row that also names a missing file ------------------------------
# isolates: that the two columns cannot disagree. A row saying the directory
# re-derives while listing a file it is missing is either a stale 깨짐 that was
# half-edited or a 재현됨 with a caveat nobody checks; both have to be one thing.
d="$(new reproduces_with_missing_file)"
sed -i 's#^| `doc/evidence/` | 재현됨 | 없음 | .*#| `doc/evidence/` | 재현됨 | `doc/evidence/gone.ndjson` | 재현되는데 빠진 파일이 있다 |#' "$d/PLAN.md"
expect_fail reproduces_with_missing_file "$d" "빠진 파일 is not 없음"

# --- no tracked NDJSON at all -------------------------------------------------
# isolates: the family floor. With no evidence directory the table vouches for
# nothing, and an OK line over zero rows is the failure mode this file exists
# to prevent.
d="$(new no_evidence_dirs)"
git -C "$d" rm -q --cached doc/evidence/run.ndjson doc/other-run.ndjson
expect_fail no_evidence_dirs "$d" "this table would vouch for nothing"

require_nonempty "$checked" "scenarios to run against check-evidence-retention.py"

if [ "$failures" -ne 0 ]; then
  echo "FAIL $failures of $checked scenarios did not discriminate as asserted" >&2
  exit 1
fi

echo "OK check-evidence-retention.py discriminates on all $checked scenarios:" \
     "the census tracks the instrument set in both directions, deleting a" \
     "citation breaks its 자동 row instead of silencing it, 수동 is the only" \
     "way out and keeps the evidence column, a mention inside a fence still" \
     "counts, an evidence path must be both tracked and present, an artifact" \
     "must be named by its own script, and a document the gate cannot parse" \
     "is a failure rather than an empty pass"
