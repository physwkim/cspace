#!/usr/bin/env python3
"""Partitions doc/residual-claims-census.md's OPEN population into what can
actually still be measured and what cannot, for PORTING-PLAN.md §308.4's
A3 ("doc/residual-claims-census.md OPEN 0").

`check-residual-claims-census.py` deliberately answers one question only
("does the literal `거짓 → 닫힘 (§N)` marker sit inside this bullet's own
text"), and its own header says why: partial closure inside a bullet that
mixes several claims is invisible to that rule (PORTING-PLAN.md §284.3).
Reading the raw OPEN count (198 as of this writing) as "the size of A3" is
wrong in two directions the census's single rule cannot distinguish:

  (b) some OPEN bullets are not measurements at all. PORTING-PLAN.md §4.7
      ("명시적 범위 밖 — 영구히 C++로 남는 것") lists five packages that stay
      C++ forever by decision, not by unfinished work -- no `거짓 → 닫힘`
      will ever legitimately land on `moveit_setup_assistant` (§4.7).
  (c) some OPEN bullets are already false in substance, just spelled with a
      closure word the census's marker regex does not recognize. §291.5
      (PORTING-PLAN.md:31540) found and named this exact gap: seven
      residual items it rewrote that round picked seven different
      spellings (`닫혔다(§269)`, `절반 닫혔다(§286.9)`, ...), and
      `check-closure-citations.sh` deliberately declined to widen its own
      marker vocabulary to catch them, because those spellings collide
      with 90+ unrelated senses of "닫혔다" used across the whole file (a
      crate closing, a gap closing, a test closing). That decision is
      correct at whole-file scope. It does not apply here: this script's
      input is not the whole file, it is already the ~200-bullet
      residual-claims population `check-residual-claims-census.py`
      isolated, where a closure word attached to a `§N` is overwhelmingly
      about closing *this* claim, not some unrelated sentence.

Everything left over is (a): an open measurement someone could still go
make. That count -- not 198 -- is A3's real size.

Classification rule (mechanical, applied per bullet in the census's own
OPEN set):

  scope           lead-in text matches out-of-scope-forever wording
                  (`범위 밖` + `영구히`) AND the bullet itself has the
                  package-declaration shape (`` `crate` (loc) — reason ``).
                  Never earns a `거짓 → 닫힘` marker by construction.
  closed-unmarked bullet text contains a closure word (닫혔다/닫았다/
                  해소되었다/해소됐다) bound to a `§N` citation, with no
                  remainder-qualifier word (절반/남는 것은/여전히/그대로다/
                  지금도 없다) anywhere in the same bullet. Verified against
                  PORTING-PLAN.md §291.2's independent hand-sweep table:
                  every bullet this rule tags matches a `거짓 → 닫힘`
                  verdict §291.2 already recorded elsewhere in the same
                  document, and every bullet §291.2 marked `절반` (half)
                  is excluded by the remainder-qualifier check.
  measurement     everything else -- the actual size of A3.

A bullet that contains BOTH a closure word and a remainder qualifier (the
§284.3 shape: part of the claim closed, part still open) is kept in
`measurement` -- real work remains -- but is separately flagged `mixed` in
the emitted doc, so this script's own blind spot on partial closure is
named rather than inherited silently, the same way the census names its
own.

Reuses `check-residual-claims-census.py`'s `parse()`/`CLOSURE_RE` (already
fixed against the 169->168 continuation-paragraph undercount) instead of
re-deriving bullet extraction from scratch, which would risk reintroducing
that exact bug. `--check` still cannot trust one parser's self-agreement,
so it layers three independent things instead of comparing this script's
fresh output only to its own committed doc:

  1. Delegates "has the residual-claims population itself drifted" to
     `check-residual-claims-census.py --check` against the live
     PORTING-PLAN.md and the committed census -- the tool that already
     owns that invariant and is hardened against the undercount. A bullet
     deleted from PORTING-PLAN.md fails here first.
  2. Reads the committed census's own header line
     ("최상위 불릿 N건 (CLOSED c / OPEN o)") and asserts this script's own
     classify loop accounted for exactly `o` bullets -- catches a bug in
     *this* script's loop losing a bullet even when the census parser is
     fine.
  3. Compares a fresh render of the triage doc against the committed one,
     the same contract `check-residual-claims-census.py --check` uses.

Named `triage-*` (not `check-*`) so it is not picked up by ci.yml's
`check-*.sh`/`check-*.py` glob on its own -- it depends on
`check-residual-claims-census.py` already having been run in the same job,
which the glob does not order.
"""

import argparse
import importlib.util
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DOC = REPO_ROOT / "PORTING-PLAN.md"
DEFAULT_CENSUS = REPO_ROOT / "doc" / "residual-claims-census.md"
DEFAULT_TRIAGE = REPO_ROOT / "doc" / "residual-claims-triage.md"
CENSUS_SCRIPT = REPO_ROOT / "tools" / "ci" / "check-residual-claims-census.py"


def _load_census_module():
    spec = importlib.util.spec_from_file_location(
        "check_residual_claims_census", CENSUS_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# -- kind (b): permanent scope declarations ---------------------------------
# Both signals are required. The heading signal alone would misfire if a
# future "…범위 밖…영구히…" section ever mixed in a real residual sentence;
# the bullet-shape signal alone would misfire on an unrelated bullet that
# happens to cite a package name. Together they match exactly PORTING-PLAN.md
# §4.7's five bullets today and nothing else in the corpus.
SCOPE_LEADIN_RE = re.compile(r"범위\s*밖")
SCOPE_BULLET_RE = re.compile(r"^-\s*`[^`]+`\s*\([\d,]+\)\s*[—-]")

# -- kind (c): closed in substance, never given the canonical marker --------
# Bound to a `§N` citation so this cannot fire on an unrelated "closed"
# sentence that merely appears near a section number; see module docstring
# for why this is safe at this script's (already-filtered) scope but was
# correctly rejected at check-closure-citations.sh's whole-file scope.
CLOSURE_EVIDENCE_RE = re.compile(
    r"(?:닫혔다|해소되었다|해소됐다)\s*\(\s*§[\d.]+\s*\)"
    r"|§[\d.]+\s*(?:에서|가|이)?\s*닫았다"
)
REMAINDER_QUALIFIER_RE = re.compile(r"절반|남는\s*것은|여전히|그대로다|지금도\s*없다")


def classify(section_id, leadin_text, bullet_text):
    is_scope_leadin = bool(SCOPE_LEADIN_RE.search(leadin_text)) and "영구히" in leadin_text
    if is_scope_leadin and SCOPE_BULLET_RE.match(bullet_text.strip()):
        return "scope"
    has_closure = bool(CLOSURE_EVIDENCE_RE.search(bullet_text))
    has_remainder = bool(REMAINDER_QUALIFIER_RE.search(bullet_text))
    if has_closure and not has_remainder:
        return "closed-unmarked"
    return "measurement"


def is_mixed(bullet_text):
    return bool(CLOSURE_EVIDENCE_RE.search(bullet_text)) and bool(
        REMAINDER_QUALIFIER_RE.search(bullet_text)
    )


# -- round-split tagging for kind (a) only -----------------------------------
# Mechanical keyword tags so the round-split proposal re-derives with the
# rest of this tool instead of going stale as a hand-typed table. Tagged per
# LEAD-IN GROUP (every measurement bullet under one section's lead-in, joined),
# not per individual bullet: a section's residual list is one investigative
# thread in practice, and most of its own bullets restate that thread's
# vocabulary only in the first one or two items ("C++ 스윕의 wall_secs" says
# nothing about CHOMP/STOMP by name, but it is §269.10's fifth item in a
# five-item CHOMP/STOMP benchmark list). Per-bullet tagging measured 72/187
# (39%) coverage; per-group tagging measured 174/187 (93%) on today's corpus.
# The first (highest-priority) matching theme wins, so the split stays a
# partition; a section matching none is left `unclassified` rather than
# force-fit, and is its own round (read individually, not paired).
THEMES = [
    (
        "ci-not-wired",
        re.compile(
            r"게이트는\s*CI에서\s*돌지|원격이\s*없어|docker가\s*없다|Actions\s*자체는|"
            r"crates\.io\s*신규\s*해석|툴체인이\s*떠\s*있다"
        ),
    ),
    (
        "penetration-branch",
        re.compile(r"관통\s*분기"),
    ),
    (
        "planner-benchmark-parity",
        re.compile(
            r"CHOMP|STOMP|max_iterations|seed\s*(?:lottery|base)|씨앗|"
            r"RRTConnect|OMPL|PRM|RRT\*|KPIECE|ompl_interface"
        ),
    ),
    (
        "pilz-pipeline",
        re.compile(
            r"pilz|어댑터\s*체인|PlanningContext|trajectory_generator|"
            r"tip_frame_getter|JointNumberMismatch|TipFrameException"
        ),
    ),
    (
        "move-group-service-parity",
        re.compile(
            r"plan_kinematic_path|PLANNING_FAILED|move_action|scene\s*토픽|"
            r"바이너리\s*이름|check_state_validity"
        ),
    ),
    (
        "collision-distance-accuracy",
        re.compile(
            r"상류\s*결함을?\s*재현하지|허용오차|접선|tangent|minimum_distance|"
            r"distance|collision|충돌|8\.9e-5|6,854|분리\s*분기|Phase\s*3|"
            r"HybridCollisionEnv|attached_body|GJK|바닥\s*높이",
            re.IGNORECASE,
        ),
    ),
    (
        "mesh-geometry-coverage",
        # `메시(?!지)`: bare "메시" (mesh) must not swallow "메시지" (message) --
        # §294.7's bullet about sibling *messages* it did not enumerate is not
        # about mesh geometry, and matched here before this exclusion was added.
        re.compile(r"메쉬|메시(?!지)|BVHModel|self_collision|self_distance"),
    ),
    (
        "citation-audit-hygiene",
        re.compile(r"미감사|인용|claim-audit|색인|절\s*번호를?\s*붙이지"),
    ),
]


# Hand-authored sub-split of the three themes too large for one session
# (see render()'s round-split section for why). Each tuple is
# (round label, theme it draws from, {section ids it takes from that theme}).
# Grouped by which fixture/instrument a round would actually have to go run,
# not by section number order -- e.g. C1 is every section still arguing about
# the prbt bool/distance tolerance and upstream-defect reproduction, C2 is the
# distance-row bookkeeping and re-sweep sections that came after §247 settled
# the cause, C3 is the floor-lowering/tangency/world-frame residue (the
# separate `penetration-branch` theme stays its own standalone round below --
# only 4 bullets, but a different fixture: the 42,259-case corpus, not the
# tolerance/tangency work C3 covers). This grouping is a proposal to revisit
# whenever the corpus shifts, not something `--check` enforces.
ROUND_SPLIT_PROPOSAL = [
    (
        "C1 prbt bool/distance 허용오차 + 상류 결함 재현",
        "collision-distance-accuracy",
        ["229.4", "230.5", "232.4", "233.4", "237.4", "247.6"],
    ),
    (
        "C2 distance 행 근거 이관 + upstream-bugs.md 기록 + 재스윕",
        "collision-distance-accuracy",
        ["248.9", "251.6", "260.8", "262.5", "265.8"],
    ),
    (
        "C3 바닥 낮춤 / 접선 / world-frame 잔차 + 관통 분기",
        "collision-distance-accuracy",
        ["70.3", "102.3", "216.4", "220.7", "275.4", "284.3", "288.9", "297.5", "298.6"],
    ),
    (
        "P1 Phase 7/8 seed-lottery + OMPL 대안 플래너",
        "planner-benchmark-parity",
        ["219.8", "264.12", "269.10", "286.11", "300.9"],
    ),
    (
        "P2 플래너 레지스트리 키 + CHOMP mesh-trajectory 검사",
        "planner-benchmark-parity",
        ["285.9", "296.8"],
    ),
    (
        "PZ pilz PlanningContext 통합 + 예외 감사 마무리",
        "pilz-pipeline",
        ["130.3", "227.4", "227.6", "227.7", "234.5", "240.7", "263.7", "266.7"],
    ),
]


def tag_theme(group_measurement_texts):
    """`group_measurement_texts` is every measurement bullet's text under one
    lead-in, already joined -- see the module-level comment on THEMES for why
    grouping beats per-bullet tagging here."""
    combined = " ".join(group_measurement_texts)
    matched = [name for name, pattern in THEMES if pattern.search(combined)]
    return matched[0] if matched else "unclassified"


def render(entries, doc_label, closure_re):
    kinds = {"scope": [], "closed-unmarked": [], "measurement": []}
    mixed_flagged = []
    for leadin_line, section_id, leadin_text, bullets in entries:
        for bline, btext in bullets:
            if closure_re.search(btext):
                continue  # CLOSED by the canonical marker; not this tool's input
            kind = classify(section_id, leadin_text, btext)
            row = (section_id, leadin_line, leadin_text, bline, btext)
            kinds[kind].append(row)
            if kind == "measurement" and is_mixed(btext):
                mixed_flagged.append(row)

    open_total = sum(len(v) for v in kinds.values())

    lines = []
    lines.append("<!-- GENERATED by tools/ci/triage-residual-claims-census.py --emit")
    lines.append("     doc/residual-claims-triage.md")
    lines.append(
        "     Do not hand-edit: `--check doc/residual-claims-triage.md` fails if this"
    )
    lines.append("     drifts from a fresh derivation. -->")
    lines.append("")
    lines.append(
        "# 잔여-주장 OPEN 전수의 3분할 — 잴 수 있는 것과 영원히 못 잴 것"
    )
    lines.append("")
    lines.append(
        "`doc/residual-claims-census.md`의 OPEN 불릿을 셋으로 나눈다. PORTING-PLAN.md "
        "§308.4의 A3(\"doc/residual-claims-census.md OPEN 0\")가 재는 것은 그 문서의 "
        "OPEN 전체이지만, OPEN 전체가 다 '언젠가 잴 수 있는 것'은 아니다 — 아래 "
        "`scope`가 그 반례다. 분류 규칙은 이 파일 자신의 docstring에 있다."
    )
    lines.append("")
    lines.append(
        f"OPEN {open_total}건 = measurement {len(kinds['measurement'])} + "
        f"scope {len(kinds['scope'])} + closed-unmarked {len(kinds['closed-unmarked'])}."
    )
    lines.append("")
    lines.append(
        "**A3의 실제 크기는 198이 아니라 "
        f"{len(kinds['measurement'])}이다** — `scope` {len(kinds['scope'])}건은 "
        "정의상 마커를 받을 수 없고, `closed-unmarked` {n}건은 이미 본문상 닫혔지만 "
        "정식 마커가 없을 뿐이다.".format(n=len(kinds["closed-unmarked"]))
    )
    lines.append("")

    lines.append("## scope — 정의상 영원히 안 닫히는 것")
    lines.append("")
    lines.append(
        f"{len(kinds['scope'])}건. lead-in이 \"범위 밖 ... 영구히\"를 선언하고 "
        "불릿 자신이 `크레이트 (LOC) — 사유` 모양인 경우. 측정으로 참/거짓을 가릴 "
        "대상이 아니라 결정 선언이므로 `거짓 → 닫힘`이 붙을 일이 없다."
    )
    lines.append("")
    lines.append("| 절 | 불릿 |")
    lines.append("|---|---|")
    for section_id, leadin_line, leadin_text, bline, btext in kinds["scope"]:
        sect = f"§{section_id}" if section_id else "(no §)"
        claim = re.sub(r"\s+", " ", btext).strip()
        lines.append(f"| {sect} | {doc_label}:{bline} {claim} |")
    lines.append("")

    lines.append("## closed-unmarked — 본문상 이미 닫혔지만 정식 마커가 없는 것")
    lines.append("")
    lines.append(
        f"{len(kinds['closed-unmarked'])}건. `닫혔다(§N)`/`닫았다`/`해소되었다` 같은 "
        "비정규 철자로 스스로 닫혔다고 적었을 뿐, `거짓 → 닫힘 (§N)` 정규 마커가 "
        "없어 census가 OPEN으로 센다. 이 중 넷(§248.9④, §263.7①, §274.6①②)은 "
        "PORTING-PLAN.md §291.2의 손 판정 표가 독립적으로 `거짓 → 닫힘`로 이미 "
        "확인한 것과 일치한다 — 이 도구는 그 표를 읽지 않고 본문만으로 같은 답을 "
        "냈다. 나머지 둘(§229.4, §264.12④)은 §291.2가 다루지 않은 절이라 이 도구가 "
        "새로 찾은 것이고, 아래 목록에 그대로 남긴다 — 확인은 이 표가 아니라 "
        "인용된 절(§247, §293)을 열어서 해야 한다. 정식 마커로 옮기는 것은 편집 "
        "결정이라 이 도구가 대신 쓰지 않는다."
    )
    lines.append("")
    lines.append("| 절 | 불릿 |")
    lines.append("|---|---|")
    for section_id, leadin_line, leadin_text, bline, btext in kinds["closed-unmarked"]:
        sect = f"§{section_id}" if section_id else "(no §)"
        claim = re.sub(r"\s+", " ", btext).strip()
        if len(claim) > 160:
            claim = claim[:157] + "..."
        lines.append(f"| {sect} | {doc_label}:{bline} {claim} |")
    lines.append("")

    lines.append("## measurement — A3가 실제로 재야 하는 것")
    lines.append("")
    lines.append(
        f"{len(kinds['measurement'])}건. 나머지 전부 — 미래의 어느 라운드가 실제로 "
        "가서 재거나 고칠 수 있는 항목."
    )
    lines.append("")
    lines.append(
        "**예외 하나, 자동 분류하지 않음: PORTING-PLAN.md:918 (§7.4).** "
        "\"`moveit-error`/`moveit-geometry` 착수 완료. 워크스페이스 테스트 14/14 "
        "통과.\" — 이 불릿은 부정 어휘가 전혀 없다(닫힘 증거도, `아직/않았다/못했다`류 "
        "잔여 서술도 없다). '남은 것' lead-in 아래 앉아 있지만 내용은 완료 보고문이다. "
        "같은 방식으로 '부정 어휘 없음'을 규칙화해 자동으로 걸러 보는 실험을 이 도구를 "
        "설계할 때 한 번 해봤다 — 이 문장은 그 실험의 고정된 기록이지 매 --emit마다 "
        "다시 재는 값이 아니다(당시 measurement 187건 중 39건이 걸렸고, 그중 38건은 "
        "'거부한다'/'그대로다'/'미결'처럼 이 정규식이 놓친 다른 부정 표현으로 여전히 "
        "열린 진짜 잔여 claim이었다) — 부정-어휘-부재는 이 코퍼스에서 신뢰할 수 없는 "
        "신호였다(38/39가 오탐, 코퍼스가 지금처럼 자라도 이 결론이 뒤집힐 정도로 "
        "빡빡한 비율은 아니었다). 그래서 이 도구는 918을 `measurement`에 "
        "그대로 두고, 이 한 줄만 산문으로 이름 붙인다. 편집자가 볼 때: 이 불릿을 지우거나 "
        "'완료' 절로 옮기는 것은 결정이지 측정이 아니라서, 이 도구가 대신 하지 않는다."
    )
    lines.append("")
    if mixed_flagged:
        lines.append(
            f"**혼합 불릿 {len(mixed_flagged)}건 — 부분 닫힘이 이 표에도 숨어 "
            "있음을 밝힌다.** 마커 규칙이 놓치는 것과 같은 모양이다(PORTING-PLAN.md "
            "§284.3): 한 불릿 안에 닫힘 어휘와 '아직 남았다' 어휘가 같이 있어 "
            "전체를 `measurement`로 두지만, 이 표는 그 사실을 숨기지 않고 아래에 "
            "따로 적는다 — 다음에 여는 사람이 이 불릿의 절반만 손대면 된다."
        )
        lines.append("")
        lines.append("| 절 | 불릿 |")
        lines.append("|---|---|")
        for section_id, leadin_line, leadin_text, bline, btext in mixed_flagged:
            sect = f"§{section_id}" if section_id else "(no §)"
            claim = re.sub(r"\s+", " ", btext).strip()
            if len(claim) > 160:
                claim = claim[:157] + "..."
            lines.append(f"| {sect} | {doc_label}:{bline} {claim} |")
        lines.append("")

    lines.append("### 라운드 분할 제안 (테마별, 섹션 번호 아님)")
    lines.append("")
    lines.append(
        "measurement 불릿을 lead-in(=절)별로 묶고, 그 절의 measurement 불릿 전체를 "
        "합친 텍스트에 아래 THEMES 규칙(이 파일 상단)을 적용한다 — 한 절의 잔여 "
        "목록은 대개 하나의 조사 스레드이고, 개별 불릿 단위로 매기면 다섯 번째 "
        "항목처럼 스레드 이름을 반복하지 않는 항목이 새어 나간다(모듈 docstring 참고 "
        "— 이 도구를 설계할 때 measurement가 187건이던 코퍼스에서 한 번 측정한 고정된 "
        "비교치: 불릿 단위 72/187(39%) 대 절 단위 174/187(93%). 코퍼스가 자란 지금 "
        "다시 재면 분모/분자 모두 바뀌지만 그룹 단위가 더 잘 덮는다는 결론 자체는 "
        "이 비율 차이가 뒤집힐 만큼 크지 않았다). 매치되는 첫 테마(우선순위 순)가 "
        "그 절 전체의 테마다. 이 절은 제안이고, `--check`가 강제하는 것은 위 3분할 "
        "개수뿐이다 — 코퍼스가 바뀌면 `--emit`으로 다시 뽑아야 한다."
    )
    lines.append("")
    by_leadin = {}
    for row in kinds["measurement"]:
        by_leadin.setdefault(row[1], []).append(row)
    theme_groups = {}
    for leadin_line, rows in by_leadin.items():
        theme = tag_theme([r[4] for r in rows])
        theme_groups.setdefault(theme, []).extend(rows)
    lines.append("| 테마 | 불릿 수 | 절 |")
    lines.append("|---|---|---|")
    theme_order = [name for name, _ in THEMES] + ["unclassified"]
    theme_section_lists = {}
    for theme in theme_order:
        rows = theme_groups.get(theme, [])
        if not rows:
            continue
        sections_with_counts = sorted(
            {(sid, sum(1 for r in rows if r[0] == sid)) for sid in {r[0] for r in rows} if sid}
        )
        theme_section_lists[theme] = sections_with_counts
        rendered_sections = ", ".join(f"§{sid}({n})" for sid, n in sections_with_counts)
        lines.append(f"| {theme} | {len(rows)} | {rendered_sections} |")
    lines.append("")

    split_themes = {theme for _, theme, _ in ROUND_SPLIT_PROPOSAL}
    split_theme_summary = ", ".join(
        f"`{t}` {len(theme_groups.get(t, []))}" for t in sorted(split_themes)
    )
    lines.append(
        f"{len(split_themes)}개 테마({split_theme_summary})는 세션 하나로 하기엔 "
        "크다. 아래는 그 절들을 군집으로 다시 쪼갠 라운드 제안이다 — 이 하위 분할은 "
        "위 테마 표(기계적)와 달리 손으로 짠 것이고, 위 표의 절별 개수를 근거로 "
        "삼는다. 코퍼스가 바뀌면 절별 개수부터 다시 뽑아 이 제안을 다시 짜야 한다."
    )
    lines.append("")
    lines.append("| 라운드 | 테마 | 절 | 불릿 수 |")
    lines.append("|---|---|---|---|")
    covered_sections_by_theme = {}
    for round_name, theme, wanted_sections in ROUND_SPLIT_PROPOSAL:
        rows = [r for r in theme_groups.get(theme, []) if r[0] in wanted_sections]
        covered_sections_by_theme.setdefault(theme, set()).update(wanted_sections)
        lines.append(
            f"| {round_name} | {theme} | "
            f"{', '.join(f'§{s}' for s in wanted_sections)} | {len(rows)} |"
        )
    lines.append("")

    # ROUND_SPLIT_PROPOSAL hardcodes which sections feed each round. If the
    # corpus shifts under it (a section gains/loses its match, or a new
    # section joins one of these three themes) the round table above would
    # silently stop covering the theme's full count, with nothing in the
    # rendered doc to say so. Surface that here instead of letting it drift
    # unnoticed -- on the next --emit this paragraph either drops out (if
    # coverage is restored) or lists exactly what ROUND_SPLIT_PROPOSAL now
    # misses, and either way `--check` catches the change.
    stale_notes = []
    for theme in sorted(split_themes):
        theme_sections = {sid for sid, *_ in theme_groups.get(theme, []) if sid}
        covered = covered_sections_by_theme.get(theme, set())
        missing = sorted(theme_sections - covered)
        if missing:
            stale_notes.append(
                f"`{theme}`: {', '.join(f'§{s}' for s in missing)}"
            )
    if stale_notes:
        lines.append(
            "**주의 — 위 라운드 표가 코퍼스를 다 못 덮는다.** "
            "ROUND_SPLIT_PROPOSAL(이 파일 상단)이 아직 이름 붙이지 않은 절: "
            f"{'; '.join(stale_notes)}. 코퍼스가 바뀌었다는 뜻이니 "
            "ROUND_SPLIT_PROPOSAL을 다시 짜야 한다."
        )
        lines.append("")

    leftover_themes = [t for t in theme_order if t not in split_themes and theme_groups.get(t)]
    leftover_summary = ", ".join(
        f"`{t}` {len(theme_groups[t])}" for t in leftover_themes if t != "unclassified"
    )
    unclassified_rows = theme_groups.get("unclassified", [])
    unclassified_section_count = len({r[0] for r in unclassified_rows if r[0]})
    lines.append(
        f"나머지 {len(leftover_themes) - (1 if 'unclassified' in leftover_themes else 0)}개 "
        f"테마({leftover_summary})와 `unclassified`({len(unclassified_rows)}, "
        f"{unclassified_section_count}개 절 — 서로 무관해 한 절씩 개별 검토)는 각각 "
        "세션 하나 안에 들어가는 크기라 그대로 한 라운드씩이다."
    )
    lines.append("")

    return "\n".join(lines) + "\n", open_total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--doc", default=str(DEFAULT_DOC))
    parser.add_argument("--census", default=str(DEFAULT_CENSUS))
    parser.add_argument("--emit")
    parser.add_argument("--check")
    parser.add_argument(
        "--skip-census-check",
        action="store_true",
        help="internal: skip delegating to check-residual-claims-census.py "
        "--check (used by this script's own regression tests, which feed a "
        "mutated --doc that the real census.md intentionally does not match)",
    )
    args = parser.parse_args()

    doc_path = Path(args.doc)
    census_path = Path(args.census)
    if not doc_path.is_file():
        print(f"FAIL {doc_path} is missing", file=sys.stderr)
        return 2

    census_module = _load_census_module()

    entries, _unbulleted = census_module.parse(doc_path)
    if not entries:
        print(
            "FAIL parsed zero lead-in lists from "
            f"{doc_path} -- check-residual-claims-census.py's own parser "
            "changed shape or found nothing; this script has nothing to "
            "triage",
            file=sys.stderr,
        )
        return 1

    try:
        doc_label = str(doc_path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        doc_label = doc_path.name

    rendered, open_total = render(entries, doc_label, census_module.CLOSURE_RE)

    if args.emit:
        Path(args.emit).write_text(rendered, encoding="utf-8")
        print(f"wrote {args.emit}: {open_total} OPEN bullets triaged")
        return 0

    check_path = Path(args.check) if args.check else DEFAULT_TRIAGE

    # Layer 1: delegate population drift (a bullet added/removed from the
    # corpus) to the tool that already owns that invariant. This is what
    # catches a bullet deleted from PORTING-PLAN.md, independent of anything
    # this script's own render() loop does -- if the corpus itself is stale
    # or has drifted, this script refuses to classify it rather than quietly
    # triaging the wrong population.
    if not args.skip_census_check:
        result = subprocess.run(
            [
                sys.executable,
                str(CENSUS_SCRIPT),
                "--doc",
                str(doc_path),
                "--check",
                str(census_path),
            ],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            print(
                "FAIL check-residual-claims-census.py --check did not pass -- "
                "the residual-claims population itself is stale or has "
                "drifted, so this script will not triage it:",
                file=sys.stderr,
            )
            print(result.stdout, file=sys.stderr)
            print(result.stderr, file=sys.stderr)
            return 1

    # Layer 2: read the committed census's own header count independently of
    # this script's classify loop, and assert the loop accounted for every
    # bullet the census claims exist. Catches a bug in *this* script losing a
    # bullet even when check-residual-claims-census.py's own parser is fine.
    if not census_path.is_file():
        print(f"FAIL {census_path} is missing -- run its own --emit first", file=sys.stderr)
        return 2
    census_text = census_path.read_text(encoding="utf-8")
    header_match = re.search(
        r"최상위\s*불릿\s*(\d+)\s*건\s*\(CLOSED\s*(\d+)\s*/\s*OPEN\s*(\d+)\)", census_text
    )
    if not header_match:
        print(
            f"FAIL could not find the 'CLOSED n / OPEN n' header line in "
            f"{census_path} -- its format changed and this script cannot "
            "cross-check its own bullet count against it",
            file=sys.stderr,
        )
        return 1
    census_open = int(header_match.group(3))
    if open_total != census_open:
        print(
            f"FAIL this script's own classify loop accounted for {open_total} "
            f"OPEN bullet(s), but {census_path} states OPEN {census_open} -- "
            "a bullet was lost (or gained) inside this script, independent of "
            "check-residual-claims-census.py's own parser",
            file=sys.stderr,
        )
        return 1

    # Layer 3: fresh render vs. committed triage doc, same contract
    # check-residual-claims-census.py --check uses.
    if not check_path.is_file():
        print(f"FAIL {check_path} is missing -- run --emit first", file=sys.stderr)
        return 2
    committed = check_path.read_text(encoding="utf-8")
    if committed != rendered:
        print(
            f"FAIL {check_path} does not match a fresh derivation from "
            f"{doc_path} ({open_total} OPEN bullets today) -- regenerate with "
            f"tools/ci/triage-residual-claims-census.py --emit {check_path} "
            "and commit the result",
            file=sys.stderr,
        )
        return 1

    print(
        f"OK {check_path}: {open_total} OPEN bullets triaged, matches a fresh "
        f"derivation from {doc_path}, and agrees with {census_path}'s own "
        f"OPEN {census_open} header"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
