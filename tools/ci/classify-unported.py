#!/usr/bin/env python3
"""Answer three questions about every unported corpus file, and check the answers.

`measure-port-coverage.py` splits the corpus into ported and unported, and
`doc/port-coverage.md` gives each unported file a class
(`decided-non-port` / `gap` / `ported-elsewhere`).  Neither says what the file
*is*, where the decision that excluded it is written, or which Phase
completion condition it blocks.  This produces all three, and -- more
importantly -- verifies the second one instead of transcribing it.

  1. WHAT      the symbols the upstream file declares, read from the file.
  2. WHY       where the decision is recorded.  Four loci, and the
               distinction between the first three and the last is the whole
               point of this script:
                 `§`          a PORTING-PLAN.md section, verified to resolve
                              to a real heading
                 `D`          a D1..D14 project decision
                 `crate-doc`  a sentence in a crate's `lib.rs`/module doc,
                              which `doc/port-coverage.md` 3 accepts as
                              evidence -- verified here by opening the cited
                              `.rs` span and requiring it to NAME the upstream
                              file.  A citation that resolves but does not
                              name the file is reported as `UNVERIFIED`, not
                              silently accepted.
                 `none`       no locus at all.  By the brief's rule this is a
                              gap, not a decision.
  3. BLOCKS    which of the UNMET rows of PORTING-PLAN.md 5's table this file
               blocks.  The blocker of each UNMET row is read from the section
               that row cites, NOT guessed from the file's directory -- see
               UNMET_BLOCKERS below.

Usage:
    tools/ci/classify-unported.py [--upstream DIR] [--repo DIR]
                                  [--emit DOC] [--check DOC]
                                  [--phase-table-only]

`--phase-table-only` runs just the §5 cross-check and stops before anything
that needs `--upstream`; `tools/ci/check-unmet-blockers.sh` is the `check-*`
entry point for it, and its header says why that split exists.
"""

from __future__ import annotations

import argparse
import collections
import importlib.util
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_UPSTREAM = "/home/stevek/work/moveit2"

COVERAGE_DOC = "doc/port-coverage.md"
PLAN = "PORTING-PLAN.md"

ROW = re.compile(r"^\| `(moveit_[^`]+)` \| ([a-z-]+) \| (.*?) \| (.*?) \|?\s*$")
SECTION_TOKEN = re.compile(r"§(\d+(?:\.\d+)*)")
DECISION_TOKEN = re.compile(r"\bD(1[0-4]|[1-9])\b")
# `path.rs:12`, `path.rs:12-34`, and the comma form `path.rs:21-28,64-73`.
# Requiring a closing backtick right after the span missed the comma form and
# reported `moveit_core/exceptions/src/exceptions.cpp` as having no citation
# at all -- a fabricated gap.
RS_CITE = re.compile(r"`([A-Za-z0-9_./-]+\.rs):(\d+)(?:-(\d+))?[0-9,-]*`")
HEADING = re.compile(r"^#+\s+§?(\d+(?:\.\d+)*)\b")

# What actually blocks each not-yet-MET row, read from the section the row
# cites.  Keyed by (phase, short condition) -- the condition is a substring
# that must appear in the §5 table row, so `check_phase_coverage()` below can
# tie each entry to a live row instead of trusting this copy.
#
# This dict USED to hold its own idea of which rows were UNMET ("three"), and
# that idea went stale the moment §260 flipped the `distance: f64` row to
# PARTIAL in a parallel branch.  Nothing failed -- the tool kept printing a
# verdict about a row that no longer said what the tool thought.  The table
# lives in PORTING-PLAN.md; the tool now reads it and errors when this dict
# and that table disagree.
#
# `candidates` is the set of upstream path prefixes the row's own section
# places its mechanism in.  A file under one of those prefixes is a CANDIDATE
# blocker and the row must then be adjudicated by hand; a file under none of
# them cannot block the row.  The distinction matters: an earlier revision of
# this table carried empty path sets, which made the BLOCKS column read
# `none` for every file *by construction* -- a column that cannot say anything
# else is not a measurement.  `Phase 3`'s prefixes below select 9 of the set,
# so the column can fire and is a real test.
#
# `adjudication` is why the candidates that do fire still do not block, and it
# is quoted from the cited section, not asserted here.
UNMET_BLOCKERS = {
    # Cited §229.1 until §251 replaced that section's DIAGNOSIS and the §5 table
    # moved the citation to §251.4.  §229.1 itself says its conclusion ("upstream
    # has no convention") was "valid but incomplete", so this entry was repeating
    # a reading the plan had withdrawn -- the fourth drift check_phase_coverage()
    # has found, and the first where the section number and the prose were wrong
    # for different reasons.
    ("Phase 3", "collision: bool"): {
        "section": "275.2",
        "blocker": "fcl's narrowphase specialization registry stands where a "
                   "convention would, rather than there being no convention: "
                   "§251.1 finds all 49 of 49 cells decided by whether fcl "
                   "registered a libccd-bypassing specialization for that shape "
                   "pair, and prbt's `cylinder x box` is a blank cell in it.  "
                   "§275.1 moves the harness floor off that tangency and the "
                   "6,854 go to 0 of 10,000, so no port defect is left in the "
                   "count -- but the two implementations still split at exact "
                   "tangency, and the committed fixture scene is where the row "
                   "is measured, so it stays UNMET",
        "candidates": (
            "moveit_core/collision_detection/",
            "moveit_core/collision_detection_fcl/",
            "moveit_core/collision_detection_bullet/",
        ),
        "adjudication":
            "the sweep measures the port AGAINST the oracle over 10,000 samples "
            "and reports 6,854 mismatches, so both sides produced values -- the "
            "row fails on a semantic disagreement, not on an absent file.  §265 "
            "pins all 6,854 to the single pair floor/prbt_base_link, whose world "
            "pose no joint moves, so every sampled state hits the same tie; §270 "
            "reproduces the whole 5-robot table on merged main.  The mechanism "
            "lives in collision_detection_fcl, which doc/port-coverage.md §1 "
            "excludes from the corpus, so no unported file can be it.",
    },
    # ("Phase 3", "distance: f64") lived here while the penetration-branch row
    # read UNMEASURED.  Both of that clause's rows are MET now: the separated
    # branch since §260, and the penetration branch on the sub-population where
    # none of the three upstream defects can fire, measured by
    # tools/ci/verify-phase3-penetration-subset.sh.  The entry is removed rather
    # than reworded because this table's contract is "every not-yet-MET row",
    # and a blocker entry for a MET row is what check_phase_coverage() calls
    # drift.  What the entry used to carry that the row does not -- that the
    # rest of the branch (queries with two or more pairs, `box x box`, meshes)
    # is measurable only against a patched oracle -- is in the row's own clause
    # and in the round section that row cites.
    # ("Phase 8", "CHOMP/STOMP") was here until the §5 table took MET.  Its
    # blocker was condition 2 alone, left unspecified because no implementation
    # reached 100% after 0.01 densification -- §269 counted 1 invalid path of
    # upstream C++ CHOMP's 370, 2 of C++ STOMP's 446, 1 of the port CHOMP's 380
    # and 3 of the port STOMP's 441.  §286 specified the condition instead of
    # widening it: each planner is held to the discretisation its own upstream
    # implementation checks at, which is the same rule Phase 7 applies to
    # RRTConnect at setStateValidityCheckingResolution.  What refuted the finer
    # bar is a measurement, not a preference -- changing only the planner RNG
    # seed base moves upstream C++ CHOMP's own condition-2 failure count between
    # 1 and 0 over the same 500 problems.  The row's candidates were 8 ROS
    # plugin-wiring files that §263's crate-level harness never goes through, so
    # this closing did not port any of them; they now fall to the same `none`
    # blocker as every other unported file no non-MET row names.
    # ("Phase 9", "MoveGroupInterface") was here until §273 measured the row MET
    # and the §5 table took the verdict.  Its blocker was "there is no planner to
    # call, a decision D8 owns"; D8 wired one, and ros/verify-move-action-interop.sh
    # now gates upstream's unmodified client receiving a trajectory from
    # /move_action in both start-state spellings, graded by upstream's own
    # moveit_core.  check_phase_coverage() required this deletion -- a MET row may
    # not keep a blocker entry -- so the gate caught the row closing, which is the
    # direction it had never fired in before.
}

# Upstream declaration forms.  Deliberately shallow: this reports what the
# file declares so a reader can see WHAT it is, and every row is spot-openable
# from the path in the same row.
DECL = re.compile(
    r"^(?:template\s*<[^>]*>\s*)?"
    r"(?:class|struct)\s+(?:[A-Z_]+_(?:EXPORT|PUBLIC)\s+)?([A-Z][A-Za-z0-9_]*)"
    r"\s*(?::|\{|;|$)",
    re.M,
)
DEFINE = re.compile(r"^#define\s+([A-Z][A-Z0-9_]*)", re.M)
FREEFN = re.compile(
    r"^(?:[A-Za-z_][A-Za-z0-9_:<>,\s*&]*?\s)?([a-z][A-Za-z0-9_]*)\s*\([^;{]*\)\s*\{",
    re.M,
)
# The type prefix is OPTIONAL and may end in `::` as well as in whitespace.
# Optional because a constructor definition starts at the class name itself
# (`ConstructException::ConstructException(`); `::`-terminated because the
# namespace-qualified form has no space before the class name
# (`void planning_interface::MotionPlanResponse::getMessage(`).  Requiring
# whitespace there silently un-named planning_response.cpp and
# planning_context_loader.cpp while fixing exceptions.cpp.
MEMBER_DEF = re.compile(
    r"^(?:[A-Za-z_][A-Za-z0-9_:<>,\s*&]*?(?:\s|::))?"
    r"([A-Z][A-Za-z0-9_]*)::([~A-Za-z_][A-Za-z0-9_]*)\s*\(",
    re.M,
)
FREEDECL = re.compile(
    r"^[A-Za-z_][A-Za-z0-9_:<>,\s*&]*?\s([a-z][A-Za-z0-9_]*)\s*\([^;{]*\)\s*(?:const\s*)?;",
    re.M,
)
CONSTANT = re.compile(
    r"^(?:static\s+)?const\s+[A-Za-z_][A-Za-z0-9_:<>]*\s+([A-Z][A-Z0-9_]*)\s*=", re.M
)
# Three corpus files declare no class, no function and no constant, so the
# ladder above answers "what is it" with nothing at all.  What they DO
# contain is the honest answer: `planning_request.hpp` is one typedef, and
# both `cached_*_kinematics_plugin.cpp` are nothing but plugin registrations.
TYPEDEF = re.compile(
    r"^(?:typedef\s+.*?\s([A-Za-z_][A-Za-z0-9_]*)\s*;|using\s+([A-Za-z_][A-Za-z0-9_]*)\s*=)",
    re.M,
)
PLUGIN_EXPORT = re.compile(
    r"^PLUGINLIB_EXPORT_CLASS\(\s*([A-Za-z_][A-Za-z0-9_:]*(?:<[^>]*>)?)", re.M
)


def load_unported(upstream: str, repo: str) -> list[str]:
    """The unported list, straight from measure-port-coverage.py -- never redefined."""
    prior = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec = importlib.util.spec_from_file_location(
            "measure_port_coverage", os.path.join(HERE, "measure-port-coverage.py")
        )
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        sys.dont_write_bytecode = prior
    corpus = mod.corpus_files(upstream)
    cites = mod.cited_paths(repo, corpus)
    return [f for f in corpus if f not in cites]


def symbols(upstream: str, rel: str, limit: int = 4) -> str:
    path = os.path.join(upstream, rel)
    try:
        text = open(path, encoding="utf-8", errors="replace").read()
    except OSError:
        return "(unreadable)"
    names = list(dict.fromkeys(DECL.findall(text)))
    if not names:
        names = ["#define " + n for n in dict.fromkeys(DEFINE.findall(text))]
    if not names:
        # Out-of-line member definitions -- what a `.cpp` with no class
        # declaration of its own actually contains (`AttachedBody::setScale`).
        names = [f"{c}::{f}()" for c, f in dict.fromkeys(MEMBER_DEF.findall(text))]
    if not names:
        names = [n + "()" for n in dict.fromkeys(FREEFN.findall(text))]
    if not names:
        # Free-function DECLARATIONS (a header of helpers ends them in `;`,
        # so the body-requiring pattern above finds nothing) and file-scope
        # constants (`capability_names.hpp` is one `static const std::string`).
        names = [n + "()" for n in dict.fromkeys(FREEDECL.findall(text))]
        names += list(dict.fromkeys(CONSTANT.findall(text)))
        names = [n for n in names if n]
    if not names:
        names = [f"typedef {a or b}" for a, b in dict.fromkeys(TYPEDEF.findall(text))]
        names += [
            "PLUGINLIB_EXPORT_CLASS(" + n + ")"
            for n in dict.fromkeys(PLUGIN_EXPORT.findall(text))
        ]
    if not names:
        return "(no class/struct/#define/function declaration found)"
    shown = names[:limit]
    more = f" +{len(names) - limit}" if len(names) > limit else ""
    return ", ".join(f"`{n}`" for n in shown) + more


def declared(upstream: str, rel: str) -> tuple[str, ...]:
    """Every class/struct/#define name the upstream file declares."""
    try:
        text = open(os.path.join(upstream, rel), encoding="utf-8", errors="replace").read()
    except OSError:
        return ()
    return tuple(dict.fromkeys(DECL.findall(text) + DEFINE.findall(text)))


def plan_headings(repo: str) -> set[str]:
    out = set()
    with open(os.path.join(repo, PLAN), encoding="utf-8") as fh:
        for line in fh:
            m = HEADING.match(line)
            if m:
                out.add(m.group(1))
    return out


def rows(repo: str) -> list[tuple[str, str, str, str]]:
    out = []
    with open(os.path.join(repo, COVERAGE_DOC), encoding="utf-8") as fh:
        for line in fh:
            m = ROW.match(line)
            if m:
                out.append(m.groups())
    return out


BRACE = re.compile(r"\{([^{}]*)\}")
TOKEN = re.compile(r"`([A-Za-z0-9_.*/{},-]+)`")

# Comment-body helpers for the paragraph scoping below.  `_continues` is
# false on the lines a doc paragraph ends at: a blank `//!`, a new `- `
# bullet, a markdown heading, and the `// Ported from` citation header.
_BODY = re.compile(r"^\s*//[!/]?(.*)$")


def _comment(line: str) -> bool:
    return bool(_BODY.match(line))


def _continues(line: str) -> bool:
    m = _BODY.match(line)
    if not m:
        return False
    t = m.group(1).strip()
    if not t or t.startswith("- ") or t.startswith("#") or t.startswith("Ported from moveit2"):
        return False
    return True


def brace_expand(tok: str) -> list[str]:
    """`a.{hpp,cpp}` -> [a.hpp, a.cpp]; `a{,_b}` -> [a, a_b].  One level."""
    m = BRACE.search(tok)
    if not m:
        return [tok]
    out = []
    for part in m.group(1).split(","):
        out += brace_expand(tok[: m.start()] + part.strip() + tok[m.end():])
    return out


# A directory token only decides a file when the LINE carrying it also says
# the directory is excluded.  Without this the rule matches any directory the
# doc mentions -- `crates/moveit-planners-chomp/src/lib.rs`'s module doc names
# `chomp_motion_planner/` as the subpackage it DID port, and a bare
# ancestor-directory rule reported `chomp_optimizer.cpp` (a ported file) as
# decided-non-port.
EXCLUSION = re.compile(
    r"is not ported|not ported|is excluded|are excluded|excluded per|out of scope",
    re.I,
)


def _token_in(needle: str, span: str) -> bool:
    """`needle` appears in `span` as a whole token, not as a substring.

    Plain `in` lets a shorter name borrow a longer one's mention: the doc
    sentence names `detail/NearestNeighborsGNAT.hpp`, and `NearestNeighbors`
    is a substring of it, so the abstract base header `NearestNeighbors.hpp`
    -- a different upstream file, decided nowhere -- reported `named`.  The
    same hazard applies to declared symbols (`NearestNeighbors` vs
    `NearestNeighborsGNAT`).
    """
    return re.search(
        r"(?<![A-Za-z0-9_])" + re.escape(needle) + r"(?![A-Za-z0-9_])", span
    ) is not None


def names_file(span: str, upstream_rel: str, decls: tuple[str, ...] = ()) -> tuple[bool, str]:
    """Does this doc span name the upstream file?  Returns (matched, how).

    Four shapes occur, and a basename-only test sees just the first:
      * `basename` -- the basename or stem verbatim
      * `symbol`   -- the class/macro the file DECLARES, which is what
                      `doc/port-coverage.md` 3 actually asks for ("그 파일
                      또는 그 파일이 선언하는 클래스"): `class_forward.hpp`
                      is decided by a sentence naming `MOVEIT_CLASS_FORWARD`,
                      never the file
      * `glob`     -- `planning_context_loader*.{hpp,cpp}` covers five files
                      and is a substring of none of them
      * `dir`      -- `chomp_interface/` decides a whole subpackage, but only
                      where the line says so (see EXCLUSION above)
    """
    import fnmatch

    base = os.path.basename(upstream_rel)
    stem = base.rsplit(".", 1)[0]
    if _token_in(base, span) or _token_in(stem, span):
        return True, "basename"
    for d in decls:
        if len(d) >= 4 and _token_in(d, span):
            return True, "symbol"
    parts = upstream_rel.split("/")
    for line in span.split("\n"):
        for tok in TOKEN.findall(line):
            for cand in brace_expand(tok):
                cand = cand.strip()
                if not cand:
                    continue
                if cand.endswith("/"):
                    if (cand.rstrip("/").split("/")[-1] in parts[:-1]
                            and EXCLUSION.search(line)):
                        return True, "dir"
                    continue
                # A bare-extension glob decides nothing: `*.hpp` in a sentence
                # about which extensions were searched matches every header in
                # the corpus.  Require a stem of real length outside the
                # extension before a glob is allowed to name a file.
                stem_part = re.sub(r"[.*?\[\]]", "", cand.rsplit(".", 1)[0])
                if len(stem_part) < 4:
                    continue
                if fnmatch.fnmatch(base, cand) or fnmatch.fnmatch(upstream_rel, "*" + cand):
                    return True, "glob"
    return False, ""


def governing(repo: str, rs_path: str, line_no: int, look_back: int = 40) -> str:
    """The nearest preceding exclusion sentence above `line_no`, and its D-numbers.

    A crate doc states the reason once, as a lead-in, then lists the files as
    bullets: `crates/moveit-planners-pilz/src/lib.rs:110-111` says the bullets
    below "are excluded by PORTING-PLAN.md's D1 (no ROS dependency) and D2",
    and every bullet down to `:157` inherits it.  Returning that lead-in is
    what turns a `crate-doc` locus into a named D decision.
    """
    lines = open(os.path.join(repo, rs_path), encoding="utf-8", errors="replace").read().split("\n")
    for i in range(line_no - 1, max(-1, line_no - 1 - look_back), -1):
        if not _comment(lines[i]):
            break
        if EXCLUSION.search(lines[i]):
            ctx = " ".join(lines[max(0, i - 2):i + 2])
            ds = sorted(set("D" + d for d in DECISION_TOKEN.findall(ctx)))
            return f"lead-in :{i + 1}" + (f" -> {', '.join(ds)}" if ds else "")
    return ""


def verify_crate_doc(repo: str, upstream_rel: str, evidence: str,
                     decls: tuple[str, ...] = ()) -> tuple[str, str]:
    """Does some cited `.rs:span` actually NAME this upstream file?

    Returns (verdict, detail).  `verdict` is `named`, `UNVERIFIED` (the spans
    resolve but none mentions the file), or `UNRESOLVED` (a cited span is out
    of bounds or the file is missing).
    """
    cites = RS_CITE.findall(evidence)
    if not cites:
        return "none", ""
    checked = []
    for path, lo, hi in cites:
        full = os.path.join(repo, path)
        if not os.path.exists(full):
            return "UNRESOLVED", f"{path} does not exist"
        lines = open(full, encoding="utf-8", errors="replace").read().split("\n")
        a = int(lo)
        b = int(hi) if hi else a
        if a > len(lines) or b > len(lines):
            return "UNRESOLVED", f"{path}:{lo}-{hi or lo} past EOF ({len(lines)} lines)"
        # Widen to the enclosing BULLET or PARAGRAPH, not to the whole comment
        # run.  `crates/moveit-planners-chomp/src/lib.rs`'s module doc is one
        # 306-line `//!` run that discusses the ported half at length; taking
        # the run made a citation at `:20` "mention" `chomp_optimizer.cpp`,
        # which is ported.  A citation points at a sentence, so the unit is
        # the bullet it sits in.
        lo_i, hi_i = a - 1, b - 1
        while lo_i > 0 and _continues(lines[lo_i]) and _comment(lines[lo_i - 1]):
            lo_i -= 1
        while hi_i + 1 < len(lines) and _comment(lines[hi_i + 1]) and _continues(lines[hi_i + 1]):
            hi_i += 1
        span = "\n".join(lines[lo_i:hi_i + 1])
        checked.append(f"{path}:{lo}")
        ok, how = names_file(span, upstream_rel, decls)
        if ok:
            return f"named ({how})", f"{path}:{lo}"
    # The citation resolves but does not land on a sentence naming this file.
    # Before calling that unverified, look for the sentence elsewhere in the
    # SAME crate file: in every case here it exists a few lines further down
    # (`lib.rs:116` cited where `:127` carries the decision), which makes this
    # a miscitation to repair, not a missing decision to make.
    found: list[tuple[str, int, str]] = []
    for path, _lo, _hi in cites:
        full = os.path.join(repo, path)
        in_cite = False
        for ln, line in enumerate(open(full, encoding="utf-8", errors="replace"), 1):
            # A `// Ported from moveit2 @ <sha>:` block lists files this crate
            # DID port.  Searching it for the decision finds `lib.rs:7` --
            # the ported-header path line -- and calls a citation to a
            # non-port decision "found".  Skip the block outright.
            if "Ported from moveit2 @" in line:
                in_cite = True
                continue
            if not _comment(line):
                in_cite = False
                continue
            if in_cite:
                continue
            ok, how = names_file(line, upstream_rel, decls)
            if ok:
                found.append((path, ln, how))
    # Prefer a mention that sits under an exclusion lead-in.  Taking the first
    # mention instead picks `lib.rs:46` -- "trajectory generation, ported from
    # `pilz_industrial_motion_planner`" -- where the deciding bullet is `:140`;
    # the package name and the file stem are the same string.
    for path, ln, how in found:
        gov = governing(repo, path, ln)
        if gov:
            return f"MISCITED (decides at :{ln}, {how}; {gov})", ", ".join(checked)
    if found:
        path, ln, how = found[0]
        return f"MISCITED (mentions at :{ln}, {how}; no exclusion lead-in)", ", ".join(checked)
    return "UNVERIFIED", ", ".join(checked)


def classify(upstream: str, repo: str) -> list[dict]:
    unported = load_unported(upstream, repo)
    by_path = {p: (c, e, n) for p, c, e, n in rows(repo)}
    headings = plan_headings(repo)
    out = []
    for f in unported:
        cls, evid, note = by_path.get(f, ("(NO ROW)", "", ""))
        blob = evid + " " + note
        secs = sorted(set(SECTION_TOKEN.findall(blob)), key=lambda s: [int(x) for x in s.split(".")])
        decs = sorted(set("D" + d for d in DECISION_TOKEN.findall(blob)))
        bad = [s for s in secs if s not in headings]
        if secs:
            locus, where = "§", ", ".join("§" + s for s in secs)
            verdict = "UNRESOLVED §" + ", §".join(bad) if bad else "resolves"
        elif decs:
            locus, where, verdict = "D", ", ".join(decs), "resolves"
        else:
            v, detail = verify_crate_doc(repo, f, blob, declared(upstream, f))
            if cls == "ported-elsewhere" and v == "UNVERIFIED" and detail:
                # A `ported-elsewhere` row's evidence points at where the
                # CONTENT went, not at a sentence naming the upstream file.
                # `exceptions.cpp` is `moveit_error::Error`; demanding that
                # `moveit-error/src/lib.rs` say "exceptions.cpp" reports a
                # correct row as unverified.
                v = "content-elsewhere (cite resolves)"
            if v == "none":
                locus, where, verdict = "none", "-", "GAP: no §, no D, no crate-doc citation"
            else:
                locus, where, verdict = "crate-doc", detail, v
        cand = [
            f"{p} ({c})"
            for (p, c), spec in UNMET_BLOCKERS.items()
            if any(f.startswith(pre) for pre in spec["candidates"])
        ]
        blocks = (
            "candidate, adjudicated no: " + "; ".join(sorted(set(cand)))
            if cand
            else "none"
        )
        out.append({
            "path": f,
            "symbols": symbols(upstream, f),
            "class": cls,
            "locus": locus,
            "where": where,
            "verdict": verdict,
            "blocks": blocks,
        })
    return out


PHASE_ROW = re.compile(
    r"^\| (Phase \d+) \| (.*?) \| (MET|UNMET|PARTIAL|UNMEASURED) \| §([\d.]+) \|", re.M
)
TABLE_HEADING = "### 완료 조건 현황표"
# A real fence toggle is 3+ backticks and nothing else with a backtick on the
# line -- PORTING-PLAN also writes ```` ```text ```` as an INLINE span while
# discussing fence conventions, and a naive startswith("```") reads those as
# toggles and desyncs for the rest of the file.
FENCE_RE = re.compile(r"^ {0,3}`{3,}[^`]*$")


def phase_rows(text: str) -> list[tuple[str, str, str, str]]:
    """The rows of the LIVE §5 status table, and only those.

    A whole-file `findall` cannot do this, and read one for its whole life.
    §269.9 quotes the row it proposes to replace inside a fence -- verdict,
    citation and all -- and that quotation is a well-formed row citing the
    section the live row has since moved off.  With both in scope no value of
    `section` below can satisfy the check: setting it to the live row's §
    fails on the quotation, and setting it to the quotation's § fails on the
    live row.  The gate was unsatisfiable in one direction and nobody saw it,
    because nothing runs this file (see check-unmet-blockers.sh).

    So the read is scoped the way check-phase-status.sh scopes the same table:
    fences are tracked, and the region ends at ANY heading rather than only a
    `## ` one, so a subsection inserted after the table cannot silently extend
    it and a later `## ` cannot silently exempt rows below it.
    """
    rows, in_fence, in_table = [], False, False
    for line in text.split("\n"):
        if FENCE_RE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        if line.startswith("#"):
            in_table = line.strip() == TABLE_HEADING
            continue
        if in_table:
            match = PHASE_ROW.match(line)
            if match is not None:
                rows.append(match.groups())
    return rows


def check_phase_coverage(repo: str) -> tuple[list[str], dict]:
    """Tie UNMET_BLOCKERS to the live §5 table.  Returns (errors, verdicts).

    The dict above is a copy of facts that live in PORTING-PLAN.md, and a copy
    goes stale silently: §260 flipped `distance: f64` from UNMET to PARTIAL in
    a parallel branch and nothing here noticed.  Every row that is not MET must
    have an entry, and every entry must match a row that is not MET.
    """
    with open(os.path.join(repo, PLAN), encoding="utf-8") as fh:
        rows = phase_rows(fh.read())
    errors: list[str] = []
    # An empty parse and a clean table are otherwise the same result: every
    # loop below iterates nothing and the function returns no errors.
    if not rows:
        errors.append(
            f"parsed zero rows from {PLAN}'s `{TABLE_HEADING}` table -- the "
            f"heading or the row shape changed and this checked nothing"
        )
        return errors, {}
    verdicts: dict = {}
    open_rows = [r for r in rows if r[2] != "MET"]
    for phase, cond, verdict, sec in open_rows:
        hit = [k for k in UNMET_BLOCKERS if k[0] == phase and k[1] in cond]
        if not hit:
            errors.append(
                f"§5 row `{phase} | {cond[:60]}` is {verdict} but UNMET_BLOCKERS "
                f"has no entry for it"
            )
            continue
        verdicts[hit[0]] = verdict
        want = UNMET_BLOCKERS[hit[0]]["section"]
        if want != sec:
            errors.append(
                f"§5 row `{phase} | {cond[:40]}` cites §{sec}; UNMET_BLOCKERS "
                f"says §{want}"
            )
    for k in UNMET_BLOCKERS:
        if k not in verdicts:
            errors.append(
                f"UNMET_BLOCKERS has an entry for {k} but no §5 row that is "
                f"not MET matches it -- the row was closed or reworded"
            )
    return errors, verdicts


def row_line(i: dict) -> str:
    """The one table row for `i`.  `--emit` writes it and `--check` re-derives
    it, so the two cannot disagree about the format they are comparing."""
    return (
        f"| `{i['path']}` | {i['symbols']} | {i['class']} | "
        f"{i['locus']} {i['where']} | {i['verdict']} | {i['blocks']} |"
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--upstream", default=DEFAULT_UPSTREAM)
    ap.add_argument("--repo", default=os.getcwd())
    ap.add_argument("--emit", metavar="DOC", help="write the classification table to DOC")
    ap.add_argument("--check", metavar="DOC", help="verify DOC has one row per unported file")
    ap.add_argument(
        "--phase-table-only",
        action="store_true",
        help="check UNMET_BLOCKERS against §5's table and stop -- needs no upstream checkout",
    )
    args = ap.parse_args()

    errors, verdicts = check_phase_coverage(args.repo)
    for e in errors:
        print(f"PHASE TABLE DRIFT  {e}", file=sys.stderr)
    if errors:
        print(f"FAIL: UNMET_BLOCKERS disagrees with PORTING-PLAN.md's §5 table "
              f"in {len(errors)} place(s)", file=sys.stderr)
        return 1

    # Everything above reads PORTING-PLAN.md and nothing else; everything below
    # needs the upstream checkout.  The split is what lets check-unmet-blockers.sh
    # put the part above into the `check-*` glob, whose contract is python3 plus
    # the tracked files.  Without it this check ran only when someone invoked
    # this script by hand -- which is how the two stale citations it is designed
    # to catch sat in the tree until a merge happened to run it.
    if args.phase_table_only:
        print(
            f"OK PORTING-PLAN.md §5: {len(verdicts)} not-yet-MET row(s) each have "
            f"an UNMET_BLOCKERS entry citing the same § the row cites"
        )
        return 0

    items = classify(args.upstream, args.repo)
    locus = collections.Counter(i["locus"] for i in items)
    locus_by_class = collections.Counter((i["locus"], i["class"]) for i in items)
    verdict = collections.Counter(i["verdict"] for i in items)
    blocks = collections.Counter(i["blocks"] for i in items)

    print(f"unported files classified                 {len(items)}")
    # Split each locus by class.  §249.4 counts the crate-doc rows that are
    # `decided-non-port` and gets 35; this block counted every row with that
    # locus and got 40, and for one round the two numbers were carried around
    # as two instruments disagreeing about one set.  They are two sets: the
    # extra 5 are `ported-elsewhere`, every one of them already enumerated in
    # §249.5.  A bare per-locus total cannot say that, so it no longer prints
    # one alone.
    print("  decision locus:")
    for k, v in locus.most_common():
        split = ", ".join(
            f"{n} {cls}" for (lo, cls), n in sorted(locus_by_class.items()) if lo == k
        )
        print(f"      {k:12s} {v}  ({split})")
    print("  locus verdict:")
    for k, v in verdict.most_common():
        print(f"      {k:60s} {v}")
    print("  blocks a not-yet-MET row:")
    for k, v in blocks.most_common():
        print(f"      {v:4d}  {k}")
    print("  per-row candidate count (files under the row's own mechanism):")
    for (p, c), spec in UNMET_BLOCKERS.items():
        n = sum(
            1 for i in items if any(i["path"].startswith(pre) for pre in spec["candidates"])
        )
        print(f"      {p} ({c}) [{verdicts[(p, c)]}] via §{spec['section']}: "
              f"{n} of {len(items)} candidates")

    if args.check:
        with open(args.check, encoding="utf-8") as fh:
            doc = fh.read()
        listed = re.findall(r"^\| `(moveit_[^`]+)` \|", doc, re.M)
        want = [i["path"] for i in items]
        missing = sorted(set(want) - set(listed))
        extra = sorted(set(listed) - set(want))
        dup = sorted({p for p in listed if listed.count(p) > 1})
        for p in missing:
            print(f"MISSING ROW  {p}", file=sys.stderr)
        for p in extra:
            print(f"STALE ROW    {p}", file=sys.stderr)
        for p in dup:
            print(f"DUPLICATE    {p}", file=sys.stderr)
        if missing or extra or dup:
            print(f"FAIL {args.check}: {len(missing)} missing, {len(extra)} stale, "
                  f"{len(dup)} duplicated", file=sys.stderr)
            return 1

        # Comparing only the row SET leaves every other column unchecked --
        # including the `crates/.../lib.rs:NNN` in the locus column.  Those
        # citations are bare line numbers, which `check-citation-drift.py`
        # files under "unanchored (bounds-checked only)": a citation that
        # moves inside its file still passes there.  Re-deriving the whole
        # row and comparing it verbatim is what makes them drift-checked --
        # if a crate doc shifts, the locus line or the verdict changes and
        # this comparison fails.
        actual = {m[1]: m[0] for m in re.findall(r"^(\| `(moveit_[^`]+)` \|.*)$", doc, re.M)}
        drifted = []
        for i in items:
            if actual.get(i["path"], "").rstrip() != row_line(i).rstrip():
                drifted.append(i["path"])
        if drifted:
            for p in drifted:
                print(f"COLUMN DRIFT {p}", file=sys.stderr)
                print(f"    doc:   {actual.get(p, '(row absent)')}", file=sys.stderr)
                print(f"    fresh: {row_line(next(i for i in items if i['path'] == p))}",
                      file=sys.stderr)
            print(f"FAIL {args.check}: {len(drifted)} row(s) differ from a fresh "
                  f"derivation -- re-run --emit", file=sys.stderr)
            return 1
        print(f"OK {args.check}: {len(listed)} rows == {len(items)} unported files, "
              f"all 6 columns match a fresh derivation")

    if args.emit:
        with open(args.emit, "w", encoding="utf-8") as fh:
            fh.write(EMIT_HEADER.format(n=len(items)))
            fh.write(
                "\n## 아직 MET가 아닌 §5 행 — 무엇이 막고 있고, "
                f"{len(items)}건 중 몇이 후보인가\n\n"
                "판정어는 `PORTING-PLAN.md`의 §5 표에서 읽는다. 이 문서가 자기\n"
                "사본을 들고 있으면 §260이 `distance: f64`를 PARTIAL로 바꿨을 때처럼\n"
                "조용히 낡는다 — `check_phase_coverage()`가 어긋나면 실패한다.\n\n"
            )
            for (p, c), spec in UNMET_BLOCKERS.items():
                hits = [
                    i["path"] for i in items
                    if any(i["path"].startswith(pre) for pre in spec["candidates"])
                ]
                fh.write(f"### {p} — `{c}` — **{verdicts[(p, c)]}** (§{spec['section']})\n\n")
                fh.write(f"- **막는 것:** {spec['blocker']}\n")
                fh.write(
                    "- **후보 경로 접두사:** "
                    + ", ".join(f"`{pre}`" for pre in spec["candidates"])
                    + f" → {len(items)}건 중 **{len(hits)}건**이 후보\n"
                )
                fh.write(f"- **판정:** {spec['adjudication']}\n")
                if hits:
                    fh.write("- **후보 전건:**\n")
                    for h in hits:
                        fh.write(f"  - `{h}`\n")
                fh.write("\n")
            fh.write("\n| 상류 파일 | 심볼 | 분류 | 결정 위치 | 검증 | 막는 §5 행 |\n")
            fh.write("|---|---|---|---|---|---|\n")
            for i in items:
                fh.write(row_line(i) + "\n")
        print(f"wrote {args.emit}")
    return 0


EMIT_HEADER = """<!-- GENERATED by tools/ci/classify-unported.py --emit doc/unported-classification.md
     Do not hand-edit: `--check doc/unported-classification.md` fails if the row
     set drifts from the measured unported set. -->

# 미포팅 {n}건 — 무엇인가 / 왜 안 됐나 / 무엇을 막는가

행 수는 `tools/ci/measure-port-coverage.py`의 미포팅 집합과 같아야 하고,
`tools/ci/classify-unported.py --check`가 그것을 강제한다.

- **심볼** — 상류 파일이 선언하는 것. 파일을 읽어서 뽑는다.
- **결정 위치** — `§`(PORTING-PLAN.md 절, 실제 제목으로 해석되는지 확인),
  `D`(D1..D14), `crate-doc`(크레이트 doc 문장 — 인용한 `.rs` 구간을 열어
  그 구간이 상류 파일 이름을 **실제로 부르는지** 확인), `none`(아무 것도
  없음 = 구멍).
- **검증** — `resolves` / `named` / `UNVERIFIED`(인용은 열리는데 파일
  이름을 부르지 않음) / `UNRESOLVED`(인용이 안 열림) / `GAP`.
- **막는 §5 행** — 아직 MET가 아닌 행 각각을 막는 것은 그 행이 인용한
  절에서 읽었다(`UNMET_BLOCKERS`). 디렉터리로 추측하지 않는다. 어느 행이
  아직 MET가 아닌지는 `PORTING-PLAN.md`의 §5 표에서 읽으며, 이 문서가 그
  목록의 사본을 들고 있지 않다. 값이 `none`인 것은 그 파일이 어느 행의
  기전 경로에도 없다는 **측정 결과**다: Phase 3 `collision: bool`의
  접두사가 9건을, Phase 8의 접두사가 8건을 실제로 고르므로 이 열은 발화할
  수 있고, 발화한 건은 아래 판정 문단에서 개별로 기각된다.
"""


if __name__ == "__main__":
    sys.exit(main())
