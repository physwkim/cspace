# Claim audit — moveit-srdf

**This crate had never been audited and had never been swept**, because it
had never been assigned to a panel — see `doc/crate-ownership.md`, "미배정 두
건". The coordinator took it directly rather than leaving it as the one crate
whose cleanliness was an assumption.

Upstream is `srdfdom` 2.0.8 at `third_party/srdfdom/`, not moveit2: this
crate is written from scratch against `include/srdfdom/model.h` (struct
layout, accessor names) and `src/model.cpp` (parsing and validation
semantics), per `PORTING-PLAN.md` §2. Every row below was verified by
opening `third_party/srdfdom/src/model.cpp` in this round — no verdict comes
from reading the port and reasoning backwards.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/parse.rs:9-13` | the load order is upstream's `initXml` order | CONFIRMED | `model.cpp:684-693` — virtual_joints, groups, group_states, end_effectors, link_sphere_approximations, collision_defaults, enable_collisions, disable_collisions, passive_joints, joint_properties; `parse.rs:47-57` is the same sequence | |
| `src/parse.rs:110-112` | upstream checks name, child_link, parent_frame, type in that order and stops at the first missing one | CONFIRMED | `model.cpp:92,97,107,112` — four `if (!x) { log; continue; }` in exactly that order | |
| `src/parse.rs:134-137` | an unknown virtual-joint type keeps the joint as `fixed` rather than dropping it | CONFIRMED | `model.cpp:121-127` — `vj.type_ = "fixed"` after the "Assuming 'fixed' instead" log, then `push_back` | |
| `src/parse.rs:129` (implicit) | type is trimmed and case-folded before the comparison | CONFIRMED | `model.cpp:118-120` — `boost::trim` then `std::transform(..., ::tolower)` | |
| `src/parse.rs:212-219` | subgroup resolution runs to a fixpoint, so a forward reference is legal, and both unknown-subgroup and cyclic groups are dropped | CONFIRMED | `model.cpp:266-294` — `while (update)` over all groups, then `known_groups.size() != groups_.size()` prunes the rest with "has unsatisfied subgroups" | |
| `src/parse.rs:282-284` | repeating a joint inside one group state concatenates the value lists rather than replacing | CONFIRMED | `model.cpp:376` — `gs.joint_values_[jname_str].push_back(val)` into a `std::map<std::string, std::vector<double>>`, so a second `<joint name=...>` appends | |
| `src/parse.rs:494-496` | upstream's child walk never descends, which is what stops a `<passive_joint>` nested in a `<group>` from becoming a model-level passive joint | CONFIRMED | every loop in `model.cpp` is `X->FirstChildElement(tag)` / `NextSiblingElement(tag)` on the immediate parent — `:85,140,154,173,192,250,311,345,397,455,476,552,574` | |
| `src/parse.rs:504-509` | on a malformed token upstream stores the failed extraction's zero rather than rejecting | CONFIRMED | `model.cpp:370-377` — `while (ss.good() && !ss.eof()) { double val; ss >> val >> std::ws; push_back(val); }`; since C++11 a failed arithmetic `operator>>` writes `0` and sets failbit, so `"1.0 abc"` yields `[1.0, 0.0]` | |
| `src/parse.rs:520-521` | upstream extracts exactly three doubles for a sphere centre and never checks what follows, so trailing text is ignored | CONFIRMED | `model.cpp:491-493` — `center.exceptions(failbit\|badbit); center >> x >> y >> z;` with no eof check afterwards | |
| `src/parse.rs:533-535` | `toDouble` allows leading whitespace and rejects trailing whitespace | CONFIRMED | `model.cpp:53-66` — `stream >> result` skips leading ws by default, then `if (stream.fail() \|\| !stream.eof()) throw`; a trailing space leaves eofbit clear, so it throws | |
| `src/model.rs:148-150` | upstream leaves `parent_group_` as `""` when the attribute is absent | CONFIRMED | `model.cpp:444-448` — the assignment is guarded by `if (parent_group)`, and `parent_group_` is a default-constructed `std::string` | |
| `src/model.rs:352-365` | the three radius-zero rules, with "positive" meaning strictly greater than epsilon | CONFIRMED | `model.cpp:517-540` — `if (sphere.radius_ > std::numeric_limits<double>::epsilon())` clears the vector on the first positive sphere; the `else if (non_0_radius_sphere_cnt == 0)` branch collapses to one origin-centred zero sphere. `parse.rs:382-393` is the same branch structure, and `f64::EPSILON` == `std::numeric_limits<double>::epsilon()` == 2.220446049250313e-16 | |

## Second pass — the structural and deviation claims

The first pass covered the 12 behavioural claims and recorded the rest as a
known gap. This pass closes it for `src/`.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/model.rs`, 24 sites | the upstream symbols named: the structs `Group`, `GroupState`, `VirtualJoint`, `EndEffector`, `Sphere`, `LinkSpheres`, `CollisionPair`, `JointProperty`, `PassiveJoint`, `Model`; the entry points `initXml`/`initString`/`initFile`; the accessors `getName`, `getGroups`, `getGroupStates`, `getVirtualJoints`, `getEndEffectors`, `getLinkSphereApproximations`, `getNoDefaultCollisionLinks`, `getEnabledCollisionPairs`, `getDisabledCollisionPairs`, `getPassiveJoints`, `getJointProperties` (both overloads) | CONFIRMED | checked mechanically against `include/srdfdom/model.h` — structs at `:83,117,133,150,164,176,186,199,206`, `Model` at `:59`, init at `:71-77`, accessors at `:219-292`. 24 asserted, 24 present | |
| `src/model.rs:53` | upstream stores a chain as `std::pair<std::string, std::string>`, ordered base then tip | CONFIRMED | `model.h:105` `std::vector<std::pair<std::string, std::string>> chains_;`, and `:102-104`'s own comment says "specified as a pair of base link and tip link" — the order claim is upstream's own words, not an inference | |
| `src/model.rs:194` | `reason` is carried verbatim, without trimming, as upstream does | CONFIRMED | `model.cpp:585-586` — `CollisionPair pair{ boost::trim_copy(link1), boost::trim_copy(link2), reason ? reason : "" }`. The asymmetry is inside one initializer: the two link names are trimmed, `reason` is not | |
| `src/model.rs:77`, `:409` | a `BTreeMap` matches upstream's `std::map` iteration order | CONFIRMED | `model.h:160` `std::map<std::string, std::vector<double>> joint_values_;` and `:324` `std::map<std::string, std::vector<JointProperty>> joint_properties_;` — `std::map` iterates in key order, as `BTreeMap` does | |
| `src/model.rs:252` | upstream wraps each passive joint in a struct rather than storing a bare name | CONFIRMED | `model.h:199-203` — `struct PassiveJoint { std::string name_; }` | |
| `src/model.rs:247` (deviation 4) | upstream leaves `name_` as `""` for an SRDF with no `name` attribute, indistinguishable from `name=""` | CONFIRMED | `model.cpp:672-682` — `name_` is assigned only inside the `else`; with no attribute it stays default-constructed, and upstream logs "No name given for the robot." | |
| `src/model.rs:326` (deviation) | upstream's failed `istringstream` extraction leaves the `double` at 0 and stores it, so `value="oops"` and `value="0"` produce the same state | CONFIRMED | `model.cpp:370-377`, the same loop as the first pass's `parse.rs:504` row. `"oops"` pushes one `0.0`; `"0"` pushes one `0.0`. Since `0.0` is a legal joint position the failure is undetectable downstream — the port's stated consequence is exact | |
| `src/diagnostic.rs:14-22` | upstream writes these to `console_bridge` and returns a model carrying no trace; a diagnostic is never fatal; the only two fatal conditions are malformed XML and a non-`robot` root | CONFIRMED | every `loadX` in `model.cpp` returns `void` and only logs, so none can change the result. `initXml` returns `false` at exactly one place, `:669` (missing or non-`robot` root); the other two `false` returns are `initFile:722` (file could not be opened) and `initString:731` (`xml_doc.Parse != XML_SUCCESS`). Nothing else fails | |

## Summary

- **20 claims verified across both passes, 20 CONFIRMED, 0 EXPIRED, 0
  UNVERIFIABLE.** `crates/moveit-srdf/src` is now fully audited: every
  `upstream`-asserting line is either covered by a row above or is a
  cross-reference to one that is.
- The upstream source is present locally (`third_party/srdfdom/`), so nothing
  in this crate falls under the "source absent" bucket that forces
  UNVERIFIABLE elsewhere.
- Still unaudited: the 17 `upstream` mentions in `tests/boundaries.rs` and
  `tests/fixtures.rs`. Those describe what a test pins rather than asserting
  upstream behaviour independently, but they have not been opened, and that
  is a gap rather than a pass.

## Two bounded differences found while verifying (neither is a defect)

- `::tolower` (upstream, `model.cpp:120`) is locale-dependent for bytes
  outside ASCII; the port's `to_ascii_lowercase` is not. Identical for the
  three keywords `fixed`/`planar`/`floating`, which is the whole domain of
  the comparison.
- Upstream's virtual-joint walk has a URDF link-existence check between the
  `child_link` and `parent_frame` checks (`model.cpp:102-106`) that the port
  does not perform. Already covered by deviation 1 on `SrdfModel` ("No URDF
  is consulted"), so the "checks ... in this order" claim is about attribute
  presence only. Stated here because the citation alone does not make that
  visible.

## §172 narrowing sweep (first ever run on this crate)

- Upstream-first anchor: `int`/`unsigned`/`size_t`/`long`/`short`/`uintN_t`/
  `intN_t` declarations initialized from a floating-point expression, across
  `third_party/srdfdom/src/model.cpp` and
  `third_party/srdfdom/include/srdfdom/model.h` — **0 hits**. The reason is
  checkable rather than incidental: every `double` in upstream srdfdom is a
  value it stores and hands back (`center_x_/y_/z_`, `radius_`,
  `joint_values_`) or the local in `toDouble`; none is ever converted to an
  integer. The one integer that interacts with the float logic,
  `non_0_radius_sphere_cnt` (`model.cpp:517-540`), is a count of spheres.
- Port-side anchor: `as (i8|i16|i32|i64|isize|u8|u16|u32|u64|usize)` across
  `crates/moveit-srdf/src` and `crates/moveit-srdf/tests` — **0 hits**. This
  crate performs no integer casts at all.
- Both anchors run, both zero. Per §172 a zero from one anchor is not a
  zero; both are recorded.
