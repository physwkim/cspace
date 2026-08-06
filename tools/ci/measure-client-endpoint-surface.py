#!/usr/bin/env python3
"""Enumerate upstream `MoveGroupInterface`'s public surface and the wire
endpoint each declaration reaches.

Phase 9's completion condition names the *unmodified* C++
`MoveGroupInterface` as the client that has to attach.  Everything measured
about that condition so far was measured one blockage at a time: §226.4
called the blockage "the server side", §250.2 split that into four, and
running it then found a rejection earlier than any of the four predicted.
That cadence -- one round, one rejection -- is what happens when the
requirement is never enumerated: each round can only see the next `return`
the client happened to hit.

This script enumerates it in one pass.  Two levels, both printed, because a
single number that cannot be opened is not a measurement:

  1. DECLARATIONS.  Every public function declaration of `MoveGroupInterface`
     in `move_group_interface.hpp`, by line.  `move_group_interface.h` is not
     read: at the pinned revision it is a 52-line deprecation shim that
     `#include`s the `.hpp` and declares nothing.
  2. ENDPOINTS.  For each declaration, which of the client's ten handles it
     reaches, and therefore which ROS endpoint a port has to answer.

The count is cross-checked against `count-public-declarations.sh`, which
parses the same header with an independently written awk pass and counts
*all* public declarations at class-brace depth 1 -- functions plus the data
member, the `MOVEIT_STRUCT_FORWARD` macro, and the two nested struct
definitions this script reports separately.  The two totals must agree; a
parser that drifts stops agreeing.  That check is the reason this script does
not carry its own copy of that counting rule (`check-audit-scripts-not-copied.sh`).

Usage:
    tools/ci/measure-client-endpoint-surface.py --upstream DIR [--rows]
                                                [--check DOC]

Named `measure-*`, like its siblings: it prints by default and only asserts
under `--check`.  `verify-client-endpoint-surface.sh` owns the precondition
(checkout present, pinned SHA) and is what `verify-all.sh`'s glob reaches.
"""

from __future__ import annotations

import argparse
import collections
import os
import re
import subprocess
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))

HPP = ("moveit_ros/planning_interface/move_group_interface/include/moveit/"
       "move_group_interface/move_group_interface.hpp")
CPP = ("moveit_ros/planning_interface/move_group_interface/src/"
       "move_group_interface.cpp")
CLASS = "MoveGroupInterface"

# The ten handles the client owns, with the endpoint each one names and where
# that name is defined.  Declared rather than derived because the names are
# `static const std::string` constants in three other translation units, and
# resolving a C++ constant is not something this parser can honestly do; what
# it CAN do is refuse to let the table go stale, which `check_handles` below
# does -- every handle here must be assigned in the client, and every handle
# assigned from a `create_*`/`getShared*` call in the client must be here.
#
# Every name is RELATIVE -- no leading slash.  The client resolves each one
# through `rclcpp::names::append(opt_.move_group_namespace, ...)` at each
# creation site, so an absolute `/move_action` would be a different endpoint:
# it would ignore the `move_group_namespace` option the client exists to
# honour.  The two `PlanningSceneMonitor` names are cited to their `.cpp`
# definitions rather than their header declarations on purpose.  The header
# declares them without values and annotates them with a trailing comment
# that is wrong -- `static const std::string DEFAULT_JOINT_STATES_TOPIC;  //
# "/joint_states"` at `planning_scene_monitor.hpp:105` against
# `= "joint_states"` at `planning_scene_monitor.cpp:70`.  That comment is
# where this table's leading slashes came from before this correction; see
# `psm-topic-header-comments-claim-absolute-names` in `doc/upstream-bugs.md`.
ENDPOINTS = {
    "move_action_client_": (
        "move_action", "action",
        "move_group/capability_names.hpp:52"),
    "execute_action_client_": (
        "execute_trajectory", "action",
        "move_group/capability_names.hpp:45"),
    "query_service_": (
        "query_planner_interface", "service",
        "move_group/capability_names.hpp:46-47"),
    "get_params_service_": (
        "get_planner_params", "service",
        "move_group/capability_names.hpp:48-49"),
    "set_params_service_": (
        "set_planner_params", "service",
        "move_group/capability_names.hpp:50-51"),
    "cartesian_path_service_": (
        "compute_cartesian_path", "service",
        "move_group/capability_names.hpp:59-60"),
    "trajectory_event_publisher_": (
        "trajectory_execution_event", "topic-pub",
        "trajectory_execution_manager.cpp:50"),
    "attached_object_publisher_": (
        "attached_collision_object", "topic-pub",
        "planning_scene_monitor.cpp:71"),
    "current_state_monitor_": (
        "joint_states", "topic-sub",
        "planning_scene_monitor.cpp:70"),
    "constraints_storage_": (
        "warehouse", "non-ros",
        "move_group_interface.cpp:1197"),
}

# The pimpl constructor creates every handle, so propagating its touches
# would make every caller of every constructor touch everything -- that is
# how the first attempt at this saturated.  Creating a client is not calling
# it, so the constructor and destructor are excluded as propagation sources
# and reported on their own terms instead (`CTOR_ENDPOINTS`).
NOT_A_CALL = {"MoveGroupInterfaceImpl", "~MoveGroupInterfaceImpl"}

# What the constructor itself puts on the wire, as opposed to what it merely
# creates.  Two blocking waits on action servers, and the model load, which
# reads a parameter and falls back to a latched topic
# (`synchronized_string_parameter.cpp:101`, `:125`) -- once per description,
# and there are two of them (`rdf_loader.cpp:92-99`).  Not derived: each of
# these is a call through a handle-typed local or a free function, not a use
# of a member the classifier can see.
CTOR_ENDPOINTS = ("move_action", "execute_trajectory", "robot_description",
                  "robot_description_semantic")

# Call edges a name-based resolver cannot decide, because the overloads that
# share the name do NOT share an endpoint set.  Each entry pins one call site
# to the one definition it really binds; `check_resolutions` fails if the
# named definition is no longer a definition of that name, and any *other*
# disagreeing edge that is not listed here fails too.  Edges whose candidates
# all agree need no entry: the union is exact when there is nothing to
# choose between.
#
#   (caller's first line, callee as written) -> callee's first line
RESOLUTIONS = {
    # `setNamedTarget` passes `it->second`, a `std::vector<double>`.
    (1546, "setJointValueTarget"): 1570,
    # `(joint_name, double)` forwards to `(joint_name, vector<double>)`.
    (1634, "setJointValueTarget"): 1640,
    # `(JointState)` forwards to `(names, values)`.
    (1655, "setJointValueTarget"): 1603,
    # `(Isometry3d, eef)` converts and forwards to the `Pose` overload, which
    # is the one that seeds from the start state.
    (1672, "setJointValueTarget"): 1660,
    # The `std::string` overload reads the warehouse; the `Constraints` one
    # stores the message it was handed.
    (2156, "impl_->setPathConstraints"): 1093,
    (2161, "impl_->setPathConstraints"): 1088,
}

KEYWORD = {"if", "for", "while", "switch", "return", "catch", "sizeof",
           "else", "do", "throw", "new", "delete", "case"}

# A call on `this`.  Anything reached through `::`, `.`, `->` or `~` is some
# other entity -- `std::move(x)` is not `MoveGroupInterface::move()`, and the
# `~MoveGroupInterface()` in a destructor's signature is not a call at all.
# Both of those attributed `/move_action` to the special members before the
# lookbehind was here.
CALL = re.compile(r"(?<![:.>~\w])([A-Za-z_]\w*)\s*\(")
IMPL_CALL = re.compile(r"impl_->\s*([A-Za-z_]\w*)")
ATTRIBUTE = re.compile(r"\[\[[^\]]*\]\]")
DECLARATOR = re.compile(r"(~?[A-Za-z_]\w*)\s*\(")
MACRO = re.compile(r"^[A-Z_][A-Z_0-9]*\s*\(")


def strip_cxx(text: str) -> str:
    """Blank comments and string bodies, keeping every newline in place.

    Line-prefix filtering is not enough: this header's doxygen continuation
    lines start with prose rather than `*`, so a prefix filter merges them
    into the declaration that follows and invents a method named after the
    last word of a sentence.
    """
    out = []
    i, n, state = 0, len(text), "code"
    while i < n:
        c, two = text[i], text[i:i + 2]
        if state == "code":
            if two == "/*":
                state, i = "block", i + 2
                out.append("  ")
                continue
            if two == "//":
                state, i = "line", i + 2
                out.append("  ")
                continue
            if c == '"':
                state = "str"
                out.append(" ")
                i += 1
                continue
            out.append(c)
            i += 1
        elif state == "block":
            if two == "*/":
                state, i = "code", i + 2
                out.append("  ")
                continue
            out.append("\n" if c == "\n" else " ")
            i += 1
        elif state == "line":
            if c == "\n":
                state, i = "code", i + 1
                out.append("\n")
                continue
            out.append(" ")
            i += 1
        else:
            if c == '"' and text[i - 1] != "\\":
                state = "code"
            out.append("\n" if c == "\n" else " ")
            i += 1
    return "".join(out)


def read(upstream: str, rel: str) -> list[str]:
    path = os.path.join(upstream, rel)
    if not os.path.isfile(path):
        sys.exit(f"FAIL {path} is absent -- nothing was enumerated.")
    with open(path, encoding="utf-8") as handle:
        raw = handle.read()
    lines = strip_cxx(raw).splitlines()
    if len(lines) != len(raw.splitlines()):
        sys.exit(f"FAIL stripping comments changed {rel}'s line count.")
    return lines


# ------------------------------------------------------------- declarations

def public_region(lines: list[str]) -> tuple[int, int]:
    """First and last line of the class's own `public:` region."""
    head = re.compile(r"^class\s+([A-Z_][A-Z_0-9]*\s+)?" + CLASS + r"\b")
    start = next((n for n, s in enumerate(lines, 1) if head.match(s)), None)
    if start is None:
        sys.exit(f"FAIL no `class {CLASS}` head found.")
    depth, first, i = 0, None, start
    while i <= len(lines):
        s = lines[i - 1]
        depth += s.count("{") - s.count("}")
        if depth == 1 and first is None and s.strip() == "public:":
            first = i + 1
        elif depth == 1 and first is not None and \
                s.strip() in ("protected:", "private:"):
            return first, i - 1
        if depth <= 0 and i > start:
            sys.exit(f"FAIL class {CLASS} closes with no access section after "
                     "`public:`.")
        i += 1
    sys.exit(f"FAIL class {CLASS} never closes.")


def declarations(lines: list[str], lo: int, hi: int):
    """(line, kind, name, text) per declaration at the class's own depth.

    A declaration ends at `;` or at the `{` of an inline body; an inline
    body is then consumed whole, so a `return f(...)` inside one is not read
    as a second declaration.  Nested `struct` bodies are consumed the same
    way, which is what keeps `Options`' and `Plan`' own members out -- they
    are other types, with their own surface.
    """
    units, buf, start, depth, skip = [], [], None, 0, 0
    for n in range(lo, hi + 1):
        s = lines[n - 1].strip()
        if not s:
            continue
        if skip:
            skip += s.count("{") - s.count("}")
            continue
        if not buf:
            start = n
        buf.append(s)
        depth += s.count("(") - s.count(")")
        if depth <= 0 and (s.endswith(";") or s.endswith("{")):
            units.append((start, " ".join(buf), s[-1]))
            if s[-1] == "{":
                skip = 1
            buf, depth = [], 0
    functions, others = [], []
    for n, text, term in units:
        if MACRO.match(text):
            others.append((n, text, "macro invocation"))
            continue
        if text.startswith(("struct ", "class ", "enum ", "union ")):
            others.append((n, text, "nested type definition"))
            continue
        if "operator=" in text:
            kind = "assign/deleted" if "= delete" in text else "assign"
            functions.append((n, kind, "operator=", text))
            continue
        match = DECLARATOR.search(ATTRIBUTE.sub(" ", text))
        if not match:
            others.append((n, text, "data member"))
            continue
        name = match.group(1)
        kind = {CLASS: "ctor", "~" + CLASS: "dtor"}.get(name, "method")
        if "= delete" in text:
            kind += "/deleted"
        if term == "{":
            kind += "/inline"
        functions.append((n, kind, name, text))
    return functions, others


def param_types(text: str) -> str:
    """The parameter list, names and defaults removed, for pairing.

    Pairing a header declaration to its out-of-line definition by name alone
    is a guess as soon as the name is overloaded ten times, which
    `setJointValueTarget` is.  Comparing normalised parameter types makes the
    pairing checkable, and `pair_declarations` fails rather than guessing
    when it does not come out one to one.
    """
    depth, start, end = 0, None, None
    for i, c in enumerate(text):
        if c == "(":
            depth += 1
            if depth == 1:
                start = i + 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                end = i
                break
    if start is None or end is None:
        return ""
    inner, out, buf, depth = text[start:end], [], "", 0
    for c in inner:
        if c in "(<[":
            depth += 1
        elif c in ")>]":
            depth -= 1
        if c == "," and depth == 0:
            out.append(buf)
            buf = ""
        else:
            buf += c
    if buf.strip():
        out.append(buf)
    normalised = []
    for param in out:
        param = param.split("=")[0].strip()
        param = re.sub(r"\b[A-Za-z_]\w*$", "", param).strip()
        normalised.append(re.sub(r"\s+", "", param))
    return ",".join(normalised)


# --------------------------------------------------------------- definitions

def bodies(lines: list[str], lo: int, hi: int, base: int, qualified: bool):
    """(name, first, last) for every body opened at `base` brace depth."""
    out, depth, i = [], base, lo
    while i <= hi:
        opens = lines[i - 1].count("{")
        closes = lines[i - 1].count("}")
        if depth == base and opens > closes:
            j = i
            while j > lo and lines[j - 2].strip() and \
                    not lines[j - 2].strip().endswith((";", "{", "}", ":")):
                j -= 1
            sig = ATTRIBUTE.sub(
                " ", " ".join(lines[k - 1].strip() for k in range(j, i + 1)))
            if qualified:
                match = re.search(
                    CLASS + r"::(~?[A-Za-z_]\w*|operator=)\s*[(&]", sig)
            else:
                found = [m for m in DECLARATOR.finditer(sig)
                         if m.group(1) not in KEYWORD]
                match = found[0] if found else None
            depth_now, k = depth + opens - closes, i
            while depth_now > base and k < hi:
                k += 1
                depth_now += lines[k - 1].count("{") - lines[k - 1].count("}")
            if match:
                out.append((match.group(1), j, k, sig))
            depth, i = base, k + 1
            continue
        depth += opens - closes
        i += 1
    return out


def bare_calls(lines: list[str], first: int, last: int) -> set[str]:
    text = "\n".join(lines[first - 1:last])
    brace = text.find("{")
    body = text[brace + 1:] if brace >= 0 else text
    return {m.group(1) for m in CALL.finditer(body) if m.group(1) not in KEYWORD}


def impl_class_span(lines: list[str]) -> tuple[int, int]:
    head = re.compile(r"^class\s+" + CLASS + r"::" + CLASS + r"Impl\b")
    first = next((n for n, s in enumerate(lines, 1) if head.match(s)), None)
    if first is None:
        sys.exit("FAIL no pimpl class head found.")
    depth, i = 0, first
    while i <= len(lines):
        depth += lines[i - 1].count("{") - lines[i - 1].count("}")
        if depth <= 0 and i > first:
            return first, i
        i += 1
    sys.exit("FAIL the pimpl class never closes.")


def check_handles(lines: list[str], first: int, last: int) -> None:
    """Every declared handle is assigned here, and nothing else is.

    The half that matters is the second one: a handle added upstream is a
    ROS endpoint added to the requirement, and the failure this guards
    against is that it lands with the table still reading complete.
    """
    # The three shapes an outside connection is opened in this translation
    # unit.  `create_callback_group` is excluded by name rather than by the
    # member it lands in: it is an executor detail with no counterpart on
    # the graph, and excluding the factory keeps the rule about what the call
    # does instead of about what somebody called the variable.
    assigned = set(re.findall(
        r"([a-z_]\w*_)\s*=\s*(?:node_->)?(?:rclcpp_action::)?"
        r"(?:create_(?!callback_group)\w+|getShared\w+"
        r"|std::make_unique<moveit_warehouse::\w+)",
        "\n".join(lines[first - 1:last])))
    undeclared = sorted(h for h in assigned if h not in ENDPOINTS)
    missing = sorted(h for h in ENDPOINTS if h not in assigned)
    for handle in undeclared:
        print(f"UNDECLARED HANDLE  {handle} is created here and names no "
              "endpoint in ENDPOINTS")
    for handle in missing:
        print(f"STALE HANDLE       {handle} is declared but nothing creates it")
    if undeclared or missing:
        sys.exit(f"FAIL {len(undeclared) + len(missing)} handle(s) disagree "
                 "with ENDPOINTS -- the endpoint set is not what this table says.")


# ------------------------------------------------------------- classification

def classify(lines: list[str]):
    impl_first, impl_last = impl_class_span(lines)
    check_handles(lines, impl_first, impl_last)

    members = bodies(lines, impl_first + 2, impl_last - 1, 1, qualified=False)
    iface = bodies(lines, impl_last + 1, len(lines), 0, qualified=True)

    by_line = {a: (n, a, b) for n, a, b, _ in members}
    by_line.update({a: (n, a, b) for n, a, b, _ in iface})
    impl_defs = collections.defaultdict(list)
    for name, a, b, _ in members:
        impl_defs[name].append(a)
    iface_defs = collections.defaultdict(list)
    for name, a, b, _ in iface:
        iface_defs[name].append(a)

    touch = {}
    calls = {}
    for name, a, b, _ in members + iface:
        text = "\n".join(lines[a - 1:b])
        touch[a] = {h for h in ENDPOINTS if h in text}
        edges = set()
        for callee in bare_calls(lines, a, b):
            if callee in iface_defs:
                edges.add((callee, tuple(iface_defs[callee])))
            elif callee in impl_defs and (name, a) in [(n, x) for n, x, _, _ in members]:
                edges.add((callee, tuple(impl_defs[callee])))
        for match in IMPL_CALL.finditer(text):
            callee = match.group(1)
            if callee in impl_defs:
                edges.add(("impl_->" + callee, tuple(impl_defs[callee])))
        calls[a] = edges

    resolved, undecided = {}, []
    for a, edges in calls.items():
        out = set()
        for callee, candidates in edges:
            targets = [c for c in candidates
                       if c != a and by_line[c][0] not in NOT_A_CALL]
            if not targets:
                continue
            pick = RESOLUTIONS.get((a, callee))
            if pick is not None:
                if pick not in targets:
                    undecided.append((a, callee, targets,
                                      f"declared {pick} is not a candidate"))
                    continue
                out.add(pick)
            else:
                out.update(targets)
        resolved[a] = out

    # Fixpoint over the resolved edges.  It does not saturate: the two
    # sources that would make it saturate -- the pimpl constructor and
    # unresolved overload sets -- are excluded above.
    reach = {a: set(touch[a]) for a in touch}
    for _ in range(len(reach) + 1):
        changed = False
        for a, targets in resolved.items():
            for t in targets:
                if not reach[t] <= reach[a]:
                    reach[a] |= reach[t]
                    changed = True
        if not changed:
            break
    else:
        sys.exit("FAIL the endpoint fixpoint did not converge.")

    # An edge needs a declared resolution only when the overloads it could
    # bind disagree about the answer; when they agree the union is exact.
    for a, edges in calls.items():
        for callee, candidates in edges:
            targets = [c for c in candidates
                       if c != a and by_line[c][0] not in NOT_A_CALL]
            if len(targets) > 1 and (a, callee) not in RESOLUTIONS:
                answers = {frozenset(reach[c]) for c in targets}
                if len(answers) > 1:
                    undecided.append((a, callee, targets, "overloads disagree"))
    for a, callee, targets, why in undecided:
        print(f"UNDECIDED EDGE     cpp:{a} --{callee}--> {targets}  ({why})")
    if undecided:
        sys.exit(f"FAIL {len(undecided)} call edge(s) have no declared "
                 "resolution and their overloads answer differently.")
    return iface, reach


def pair_declarations(decls, iface, lines):
    """hpp declaration -> its cpp definition, checked by parameter types."""
    defs = collections.defaultdict(list)
    for name, a, b, sig in iface:
        defs[name].append((a, param_types(sig)))
    paired, unpaired = {}, []
    for n, kind, name, text in decls:
        if "delete" in kind or "inline" in kind:
            continue
        want = param_types(text)
        hits = [a for a, got in defs[name] if got == want]
        if len(hits) != 1:
            unpaired.append((n, name, want, [a for a, _ in defs[name]]))
            continue
        paired[n] = hits[0]
    for n, name, want, cands in unpaired:
        print(f"UNPAIRED           hpp:{n} {name}({want}) matched "
              f"{len(cands)} definitions")
    if unpaired:
        sys.exit(f"FAIL {len(unpaired)} declaration(s) could not be paired to "
                 "exactly one definition by parameter type.")
    return paired


def endpoints_of(kind, hpp_line, cpp_line, reach):
    if kind.startswith("ctor") and "delete" not in kind and hpp_line != MOVE_CTOR:
        return list(CTOR_ENDPOINTS)
    if cpp_line is None:
        return []
    return sorted(ENDPOINTS[h][0] for h in reach.get(cpp_line, ()))


MOVE_CTOR = None  # set once the declarations are read; see `main`


# ------------------------------------------------------------------ port side
#
# The other half of the requirement: of the endpoints the client binds, which
# ones does THIS workspace open, and where.
#
# Hand-keeping that column is what rotted.  The round that produced this
# script wrote the port paths into a prose table in `PORTING-PLAN.md`; the
# same round renamed `plan_kinematic_path_server.rs` to `move_group.rs`, and
# all four citations to it were wrong on arrival.  Nothing could see it --
# `verify-upstream-citations.sh` resolves upstream paths, not port ones, and
# `check-citation-drift.py` puts an unanchored port citation in a
# bounds-checked-only bucket where a file that no longer exists still passes
# if some other file supplies the line.  A derived column moves with the
# rename instead, and a removed endpoint flips its verdict rather than
# leaving a sentence that reads true.
#
# Direction matters and is derived, not assumed: the client CALLS a service
# or action, so the port must SERVE it; the client PUBLISHES
# `trajectory_execution_event` and `attached_collision_object`, so the port
# must SUBSCRIBE; the client SUBSCRIBES to `joint_states`, so something must
# PUBLISH it.  A port site with the right name and the wrong direction is
# reported as `role-mismatch`, not as `bound`.

CLIENT_USE = {
    "action": "calls", "service": "calls", "topic-pub": "publishes",
    "topic-sub": "subscribes", "parameter": "reads",
}

# Per kind: the prose the doc prints, and the set of port-side roles that
# actually satisfy it.  Two fields rather than one because `parameter`'s
# prose names two things and only one of them is on the graph at all -- the
# client reads the parameter from its OWN node, which no other process can
# set, so the latched publisher is the whole of what a port can do about it.
# While the prose WAS the equality key, `create_publisher("robot_description")`
# -- that one thing -- compared unequal to the string naming it and the row
# read `role-mismatch`: a port that had done the only available thing was
# reported as having done the wrong thing.
PORT_ROLE = {
    "action": ("action server", {"action server"}),
    "service": ("service server", {"service server"}),
    "topic-pub": ("subscriber", {"subscriber"}),
    "topic-sub": ("publisher", {"publisher"}),
    "parameter": ("parameter or latched publisher", {"publisher"}),
}

# The model load is not one of the ten handles -- it is a free function the
# constructor calls before any handle exists -- but the client cannot be
# constructed without it, so it is a row here.  BOTH descriptions are rows:
# `RDFLoader`'s constructor calls `loadInitialValue` once for the name it was
# given and once for that name + `"_semantic"`, so a client whose node
# carries neither parameter waits out two timeouts, not one, and builds its
# model from whatever both produced.
PARAM_ENDPOINTS = {
    "robot_description": ("parameter", "common_objects.cpp:115-130"),
    "robot_description_semantic": ("parameter", "rdf_loader.cpp:96-99"),
}

# r2r's factories, and the role each one fills.  Client-side factories are
# listed too: a port that opened `create_client("move_action")` would satisfy
# nothing, and this table makes that visible instead of silently absent.
PORT_FACTORY = {
    "create_service": "service server",
    "create_action_server": "action server",
    "create_publisher": "publisher",
    "create_subscription": "subscriber",
    "subscribe": "subscriber",
    "create_client": "service client",
    "create_action_client": "action client",
}

PORT_OPENER = re.compile(
    r"\b(" + "|".join(sorted(PORT_FACTORY, key=len, reverse=True)) +
    r")\s*::\s*<[^;{}]*?\(\s*\"([^\"]+)\"", re.S)

RAW_STR = re.compile(r'r(#*)"')
CHAR_LIT = re.compile(r"'(?:[^'\\\n]|\\.)'")


def strip_rust(text: str) -> str:
    """Blank Rust comments, keeping every newline and every string literal.

    String literals survive because the endpoint name IS a string literal.
    Comments must go: `crates/moveit-planners-pilz/src/lib.rs:122` and
    `crates/moveit-kinematics/src/lib.rs:266` both name a factory in prose.
    Block comments nest in Rust, raw strings suspend escaping, and `'a'` is a
    literal where `'a` is a lifetime -- all three are handled, and the caller
    asserts the line count is unchanged so a mis-parse cannot quietly delete
    a site.
    """
    out: list[str] = []
    i, n = 0, len(text)
    while i < n:
        two = text[i:i + 2]
        if two == "//":
            end = text.find("\n", i)
            end = n if end < 0 else end
            out.append(" " * (end - i))
            i = end
        elif two == "/*":
            depth, j = 1, i + 2
            out.append("  ")
            while j < n and depth:
                if text[j:j + 2] == "/*":
                    depth += 1
                    out.append("  ")
                    j += 2
                elif text[j:j + 2] == "*/":
                    depth -= 1
                    out.append("  ")
                    j += 2
                else:
                    out.append("\n" if text[j] == "\n" else " ")
                    j += 1
            i = j
        elif text[i] == "r" and RAW_STR.match(text, i):
            match = RAW_STR.match(text, i)
            close = '"' + match.group(1)
            end = text.find(close, match.end())
            end = n if end < 0 else end + len(close)
            out.append(text[i:end])
            i = end
        elif text[i] == '"':
            j = i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == '"':
                    j += 1
                    break
                j += 1
            out.append(text[i:j])
            i = j
        elif text[i] == "'" and CHAR_LIT.match(text, i):
            match = CHAR_LIT.match(text, i)
            out.append(match.group(0))
            i = match.end()
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def port_sites():
    """Every endpoint this workspace opens, as (name, role, file, line)."""
    listed = subprocess.run(
        ["git", "-C", REPO_ROOT, "ls-files", "--deduplicate", "--", "*.rs"],
        capture_output=True, text=True, check=True).stdout.split()
    if not listed:
        sys.exit(f"FAIL `git ls-files --deduplicate -- '*.rs'` found nothing under "
                 f"{REPO_ROOT} -- the port side cannot be measured, and "
                 f"reporting every endpoint absent would look like a result.")
    sites = []
    for rel in listed:
        with open(os.path.join(REPO_ROOT, rel), encoding="utf-8") as handle:
            raw = handle.read()
        text = strip_rust(raw)
        if len(text.splitlines()) != len(raw.splitlines()):
            sys.exit(f"FAIL stripping comments changed {rel}'s line count; "
                     f"every port line number below would be shifted.")
        for match in PORT_OPENER.finditer(text):
            sites.append((match.group(2).lstrip("/"),
                          PORT_FACTORY[match.group(1)],
                          rel, text.count("\n", 0, match.start()) + 1))
    if not sites:
        sys.exit(f"FAIL no endpoint-opening call found in any of "
                 f"{len(listed)} tracked `.rs` files.  The port opens some; "
                 f"an empty scan is a broken scanner, not an empty port.")
    return sorted(sites)


def port_table(sites):
    """One row per endpoint either side names: what is needed, what is open."""
    need = {endpoint: kind for endpoint, kind, _ in ENDPOINTS.values()
            if kind != "non-ros"}
    need.update({name: kind for name, (kind, _) in PARAM_ENDPOINTS.items()})
    rows = []
    for endpoint in sorted(set(need) | {s[0] for s in sites}):
        same_name = [s for s in sites if s[0] == endpoint]
        if endpoint not in need:
            role, use = "--", "--"
            verdict = "surplus"
            chosen = same_name[0]
        else:
            kind = need[endpoint]
            role, accepted = PORT_ROLE[kind]
            use = CLIENT_USE[kind]
            exact = [s for s in same_name if s[1] in accepted]
            chosen = exact[0] if exact else (same_name[0] if same_name else None)
            verdict = "bound" if exact else (
                "role-mismatch" if same_name else "absent")
        site = f"{chosen[2]}:{chosen[3]}" if chosen else None
        rows.append((endpoint, use, role, site, verdict))
    return rows


def main() -> int:
    global MOVE_CTOR
    ap = argparse.ArgumentParser()
    ap.add_argument("--upstream", default=os.path.expanduser("~/work/moveit2"))
    ap.add_argument("--rows", action="store_true")
    ap.add_argument("--emit-doc", action="store_true",
                    help="print doc/client-endpoint-surface.md in full")
    ap.add_argument("--check", metavar="DOC")
    args = ap.parse_args()

    hpp = read(args.upstream, HPP)
    cpp = read(args.upstream, CPP)

    lo, hi = public_region(hpp)
    decls, others = declarations(hpp, lo, hi)

    total = subprocess.run(
        ["bash", os.path.join(HERE, "count-public-declarations.sh"),
         os.path.join(args.upstream, HPP), CLASS],
        capture_output=True, text=True, check=True).stdout.strip()
    if int(total) != len(decls) + len(others):
        sys.exit(f"FAIL count-public-declarations.sh says {total}, this "
                 f"parser says {len(decls)} functions + {len(others)} other "
                 f"= {len(decls) + len(others)}.")

    # The move constructor takes no node and creates no impl; it steals one.
    MOVE_CTOR = next((n for n, kind, name, text in decls
                      if kind == "ctor" and param_types(text) == CLASS + "&&"),
                     None)
    if MOVE_CTOR is None:
        sys.exit("FAIL no move constructor found -- CTOR_ENDPOINTS would be "
                 "attributed to it.")

    iface, reach = classify(cpp)
    paired = pair_declarations(decls, iface, cpp)

    rows = []
    for n, kind, name, text in decls:
        eps = endpoints_of(kind, n, paired.get(n), reach)
        rows.append((n, kind, name, eps))

    # The two deprecated `computeCartesianPath` shims are defined in the
    # header and forward to the out-of-line overload, so their answer is that
    # overload's; `paired` skips them because they have no definition to pair.
    for i, (n, kind, name, eps) in enumerate(rows):
        if kind.endswith("/inline") and not eps:
            body = " ".join(hpp[k].strip() for k in range(n, min(n + 8, len(hpp))))
            for m, k2, n2, e2 in rows:
                if n2 == name and m != n and e2 and name + "(" in body:
                    rows[i] = (n, kind, name, e2)
                    break

    wired = [r for r in rows if r[3]]
    special = sum(1 for _, kind, _, _ in rows
                  if kind.startswith(("ctor", "dtor", "assign")))

    ports = port_table(port_sites())

    if args.check:
        return check_doc(args.check, rows, ports)
    if args.emit_doc:
        emit_doc(rows, others, total, special, ports)
        return 0

    print(f"public function declarations                  {len(decls)}")
    print(f"  ctor/dtor/copy/move special members         {special}")
    print(f"  named operations                            {len(decls) - special}")
    print(f"non-function public declarations              {len(others)}")
    for n, text, why in others:
        print(f"      hpp:{n:<5} [{why}] {text[:60]}")
    print(f"count-public-declarations.sh, same header     {total}")
    print()
    print(f"declarations that reach the wire              {len(wired)}")
    print(f"declarations that are client-local            {len(rows) - len(wired)}")
    per = collections.Counter(e for _, _, _, eps in rows for e in eps)
    for endpoint, count in sorted(per.items()):
        print(f"      {endpoint:<30} {count}")
    print()
    verdicts = collections.Counter(v for _, _, _, _, v in ports)
    print(f"endpoints the client binds                    "
          f"{sum(1 for _, _, _, _, v in ports if v != 'surplus')}")
    for verdict, count in sorted(verdicts.items()):
        print(f"      {verdict:<30} {count}")
    for endpoint, use, role, site, verdict in ports:
        print(f"      {endpoint:<30} {verdict:<14} {site or '--'}")
    if args.rows:
        print()
        for n, kind, name, eps in rows:
            print(f"hpp:{n}\t{kind}\t{name}\t{','.join(eps) or '-'}")
    return 0


def emit_doc(rows, others, total, special, ports) -> None:
    wired = sum(1 for _, _, _, eps in rows if eps)
    print("# `MoveGroupInterface`'s public surface and the endpoint each "
          "declaration reaches")
    print()
    print("Generated. Regenerate with")
    print()
    print("    tools/ci/measure-client-endpoint-surface.py \\")
    print("        --upstream ~/work/moveit2 --emit-doc "
          "> doc/client-endpoint-surface.md")
    print()
    print("and check it with `tools/ci/verify-client-endpoint-surface.sh`, "
          "which owns")
    print("the pinned-revision precondition. Every `hpp:` line number is "
          "relative to")
    print("that revision; every port path is relative to this repository and "
          "is read")
    print("from the working tree, so a rename moves it. `-- ` in the last "
          "column of")
    print("the declaration table means the declaration puts nothing on the "
          "wire.")
    print()
    print("Endpoint names are **relative** -- the client resolves each one "
          "through")
    print("`rclcpp::names::append(opt_.move_group_namespace, ...)`, so a "
          "leading slash")
    print("would name a different endpoint. `robot_description` is the model "
          "load: a")
    print("parameter read that falls back to a latched topic, not a "
          "`move_group`")
    print("endpoint, but the client's constructor cannot complete without it.")
    print()
    print(f"    public function declarations   {len(rows)}")
    print(f"      special members              {special}")
    print(f"      named operations             {len(rows) - special}")
    print(f"    non-function declarations      {len(others)}")
    print(f"    count-public-declarations.sh   {total}")
    print(f"    reach the wire                 {wired}")
    print(f"    client-local                   {len(rows) - wired}")
    print()
    verdicts = collections.Counter(v for _, _, _, _, v in ports)
    for verdict, count in sorted(verdicts.items()):
        print(f"    port side, {verdict:<18} {count}")
    print()
    print("## What the port binds")
    print()
    print("`bound` means this workspace opens that endpoint in the direction "
          "the")
    print("client needs. `absent` means nothing here opens it. "
          "`role-mismatch` means")
    print("something here opens the name in the wrong direction. `surplus` "
          "means the")
    print("port opens an endpoint no `MoveGroupInterface` declaration asks "
          "for.")
    print()
    print("`bound` is a static fact about the socket and nothing more. Whether "
          "the")
    print("handler behind it then answers or rejects is a separate question "
          "this")
    print("table cannot reach -- it is Phase 9's (a)-versus-(b) split, and it "
          "needs")
    print("a read of the handler or a run of the node. `absent` is the whole "
          "of (c).")
    print()
    print("| endpoint | the client | the port must provide | opened at | "
          "verdict |")
    print("|---|---|---|---|---|")
    for endpoint, use, role, site, verdict in ports:
        cell = f"`{site}`" if site else "--"
        print(f"| `{endpoint}` | {use} | {role} | {cell} | {verdict} |")
    print()
    print("## Every public declaration")
    print()
    print("| declaration | name | endpoints |")
    print("|---|---|---|")
    for n, kind, name, eps in rows:
        cell = " ".join(f"`{e}`" for e in eps) if eps else "--"
        print(f"| `hpp:{n}` | `{name}` | {cell} |")


ROW = re.compile(r"^\|\s*`hpp:(\d+)`\s*\|\s*`([^`]+)`\s*\|\s*([^|]*?)\s*\|")
# Five cells, so it cannot match the three-cell declaration rows above.
PORT_ROW = re.compile(
    r"^\|\s*`([^`]+)`\s*\|\s*([^|]*?)\s*\|\s*([^|]*?)\s*\|\s*"
    r"(?:`([^`]*)`|--)\s*\|\s*([a-z-]+)\s*\|\s*$")


def check_doc(path: str, rows, ports) -> int:
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    have = {}
    have_ports = {}
    for line in text.splitlines():
        match = ROW.match(line)
        if match:
            have[int(match.group(1))] = (
                match.group(2),
                tuple(sorted(e for e in match.group(3).replace("`", "").split()
                             if e not in ("--", "-"))))
            continue
        match = PORT_ROW.match(line)
        if match:
            have_ports[match.group(1)] = (
                match.group(2), match.group(3), match.group(4), match.group(5))
    want = {n: (name, tuple(sorted(eps)) if eps else ())
            for n, kind, name, eps in rows}
    want_ports = {endpoint: (use, role, site, verdict)
                  for endpoint, use, role, site, verdict in ports}
    bad = 0
    for n in sorted(set(have) | set(want)):
        if n not in have:
            print(f"MISSING ROW        hpp:{n} {want[n][0]}")
            bad += 1
        elif n not in want:
            print(f"STALE ROW          hpp:{n} {have[n][0]}")
            bad += 1
        elif have[n] != want[n]:
            print(f"ROW DISAGREES      hpp:{n} doc {have[n]} measured {want[n]}")
            bad += 1
    for endpoint in sorted(set(have_ports) | set(want_ports)):
        if endpoint not in have_ports:
            print(f"MISSING PORT ROW   {endpoint} {want_ports[endpoint][3]}")
            bad += 1
        elif endpoint not in want_ports:
            print(f"STALE PORT ROW     {endpoint} {have_ports[endpoint][3]}")
            bad += 1
        elif have_ports[endpoint] != want_ports[endpoint]:
            print(f"PORT ROW DISAGREES {endpoint} doc {have_ports[endpoint]} "
                  f"measured {want_ports[endpoint]}")
            bad += 1
    if bad:
        print(f"FAIL {bad} row(s) in {path} disagree with the measurement.")
        return 1
    bound = sum(1 for _, _, _, _, v in ports if v == "bound")
    required = sum(1 for _, _, _, _, v in ports if v != "surplus")
    print(f"OK: {len(want)} declarations, "
          f"{sum(1 for _, _, _, e in rows if e)} on the wire, "
          f"{bound}/{required} endpoints bound, {path} agrees")
    return 0


if __name__ == "__main__":
    sys.exit(main())
