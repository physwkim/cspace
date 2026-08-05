# Claim audit — moveit-model

Format and rule: PORTING-PLAN.md §175. One row per claim a doc comment in
this tree makes about upstream. `evidence` is a file:line I opened myself
this round, not an inference from the port's own behavior. Appended as
found, not batched at report time.

## §172 upstream-first narrowing sweep — negative result, logged for §153.1 expiry tracking

Anchor: `int`/`size_t`/`unsigned`/`static_cast<(int\|unsigned\|size_t\|...)>`
whose initializing expression is floating-point, in the upstream C++ files
this crate's Rust sources cite. Swept via `rg` (`static_cast<...>`,
C-style `(int)`/`(size_t)`/`(unsigned)` casts, `floor/ceil/round/sqrt/pow`
near an integer declaration, integer decls with a float-literal or
division RHS) against each file below, output enumerated on screen.
`robot_model/src/aabb.cpp` additionally read in full (it is the one file
in this crate's cited set that does real floating-point geometry math, so
it got the closer read). Every file: 0 hits.

| where (upstream file swept) | claim | verdict | evidence |
|---|---|---|---|
| `robot_model/include/.../aabb.h` | no int/size_t/unsigned decl or `static_cast` with a floating-point initializer | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../aabb.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../fixed_joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../floating_joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../joint_model_group.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../link_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../planar_joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../prismatic_joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../revolute_joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../robot_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/aabb.cpp` | same; read in full — `extendWithTransformedBox` is all-`double`, no integer variables at all | CONFIRMED (absent) | full file read |
| `robot_model/src/fixed_joint_model.cpp` | no int/size_t/unsigned decl or `static_cast` with a floating-point initializer | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/floating_joint_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/joint_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/joint_model_group.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/link_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/planar_joint_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/prismatic_joint_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/revolute_joint_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/robot_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |

Port-side secondary sweep (`as (i8\|i16\|i32\|i64\|isize\|u8\|u16\|u32\|u64\|usize)`
receiving an `f64` expression, in `crates/moveit-model/src/`): 0 raw hits.

Expires (§153.1): if a future round adds an `int`/`size_t`/`unsigned`
declaration with a floating-point initializer to any file above, or a new
upstream file citing floating-point-narrowing joins this crate's cited
set, this table's `CONFIRMED (absent)` rows must be re-swept, not assumed
to still hold.

## Directory-level unported-file audit (this crate's scope only)

Answered by file count first, not by what the port already cites — an
unported file produces no audit row from inside the port, so only listing
upstream's own directory finds it. `find moveit_core/robot_model -name
'*.cpp' -o -name '*.hpp' -o -name '*.h'`, `.h` files excluded from the
count below: every one is a `create_deprecated_headers.py`-generated
forwarding shim (`#include <.../foo.hpp>` plus a deprecation
`#pragma message`), not a second porting unit — spot-checked
`link_model.h` in full.

`PORTING-PLAN.md:142` scopes `moveit-model` to `robot_model` (7,909
lines) plus URDF/SRDF loading; `robot_state` (9,202 lines, `:143`) is a
different, not-yet-started crate (`moveit-state`) — its absence here is a
crate boundary, not a missing file.

| upstream file (`moveit_core/robot_model/`) | status | evidence |
|---|---|---|
| `include/.../aabb.hpp` + `src/aabb.cpp` | ported: `crates/moveit-model/src/aabb.rs` | cited (1 hit) |
| `include/.../fixed_joint_model.hpp` + `src/fixed_joint_model.cpp` | ported: `joint/fixed.rs` | cited (1 hit) |
| `include/.../floating_joint_model.hpp` + `src/floating_joint_model.cpp` | ported: `joint/floating.rs` | cited (1 hit) |
| `include/.../joint_model_group.hpp` + `src/joint_model_group.cpp` | ported: `joint_model_group.rs` | cited (3 hits) |
| `include/.../joint_model.hpp` + `src/joint_model.cpp` | ported: `joint/model.rs` | cited (6 hits) |
| `include/.../link_model.hpp` + `src/link_model.cpp` | ported: `link_model.rs` | cited (1 hit) |
| `include/.../planar_joint_model.hpp` + `src/planar_joint_model.cpp` | ported: `joint/planar.rs` | cited (1 hit) |
| `include/.../prismatic_joint_model.hpp` + `src/prismatic_joint_model.cpp` | ported: `joint/prismatic.rs` | cited (1 hit) |
| `include/.../revolute_joint_model.hpp` + `src/revolute_joint_model.cpp` | ported: `joint/revolute.rs` | cited (1 hit) |
| `include/.../robot_model.hpp` + `src/robot_model.cpp` | ported: `robot_model.rs` | cited (4 hits) |
| `test/test.cpp` | not a literal port; upstream's own unit test, not public API. This port's oracle-fixture tests (`tests/*_parity.rs`, `tests/fixtures/{panda,fanuc}_model_info.json`) are the coverage for this crate's own behaviour, per this repo's established convention (commit `8e45543`) of deserializing an oracle fixture rather than transcribing one C++ test's literals | `rg -c` on each `.cpp`/`.hpp` name against `crates/moveit-model/{src,tests}`, verified >0 for all ten pairs above |

Result: 0 missing files. Every substantive (non-`.h`-shim) upstream file
in `robot_model/` has a cited Rust counterpart. This does not audit
symbol-level completeness within each file (e.g. `link_model.rs`'s
documented mesh-collision-shape and effort/acceleration-limit
deviations, `PORTING-PLAN.md` §13.4 deviation 4 and `:1501`/`:3606`,
already tracked there) — only that no whole file was silently dropped.

## Test-assertion discrimination sweep (moveit-constraints round; recorded
here for the `moveit-model` half)

Not an upstream-claim row (no doc comment here claims anything about
upstream) — recorded per the same table for continuity with the sibling
finding in `moveit-constraints.md`. Anchor `is_err\(\)|is_none\(\)|unwrap_err\(\)`,
swept workspace-wide, classified for this crate's hits: does any assertion
sit next to a sibling branch producing a different error on the same
call, so a bare `.is_err()` cannot identify which one fired?

`robot_model.rs`'s `no_root_link_errors`/`multiple_root_links_errors`
are sibling arms of the same `match root_candidates.as_slice()`
(`:215-230`, `[]` vs `names`) but both only asserted `.is_err()`.
Bite-checked: merged the two arms' messages into one shared string —
both tests reddened (confirmed the merge, not just one arm, broke both),
reverted, then applied the fix (each test now checks its own arm's
message substring) and reran the same merge — both tests correctly
reddened again. Every other `is_err()`/`is_none()`/`unwrap_err()` hit in
this crate has exactly one reachable branch for its exercised input, or
is a plain field-access `Option` (`JointModel::mimic()`) with no sibling
reason to conflate — not this defect shape.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `robot_model.rs:2329,2344` (`no_root_link_errors`/`multiple_root_links_errors`) | Two sibling tests, each pinning a different arm of the same `match`, asserted only `.is_err()` | CONFIRMED same-defect, fixed | See sweep note above | `fe3bd82` |

## Round 10 follow-up — `rigidly_connected_parent_link`

`RobotModel::getRigidlyConnectedParentLinkModel` is what
`moveit-kinematics`' `setFromIK` port builds its whole tip-matching rule
on (see [`doc/claim-audit/moveit-kinematics.md`](moveit-kinematics.md)),
so its claims are audited here rather than there. Both rows were
re-derived by opening the cited ranges in the pinned checkout
(`e017c91ee12984393a28ba246075c65f69cde3bf`) this round.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-model/src/robot_model.rs:395-411` (`rigidly_connected_parent_link`) | Walks up from `link_index` through joints that hold the link still — fixed joints, plus joints outside `group` when a group is given — and returns the first link a movable joint controls. | CONFIRMED | `robot_model.cpp:1367-1399`, read in full. `:1383-1390` is `is_fixed_or_not_in_jmg`, whose two `return true` arms are `getType() == FIXED` and "not in the non-empty jmg"; the port's `held_still` is the same disjunction. Two differences that had to be checked rather than assumed. First, the loop shape: upstream advances `link = parent_link; joint = link->getParentJointModel(); parent_link = joint->getParentLinkModel();` (`:1394-1396`), so its next parent comes off the *joint*, while the port re-derives `parent_link_index()` off the *link* each turn — equivalent because `robot_model.cpp:861` and `:866` set the joint's and the child link's parent link to the same `parent`. Second, membership: upstream is pointer identity via `std::find` over `jmg->getJointModels()` (`:1387`), the port is `has_joint_model(joint.name())` — the same set, joint names being unique within a `RobotModel`. | |
| `crates/moveit-model/src/robot_model.rs:389-392` (the empty-`jmg` note) | Upstream's `begin != end` guard makes an empty group contribute no membership test, and the port needs no counterpart because an empty group cannot be constructed. | CONFIRMED; the cited line was wrong and is corrected | The guard is `robot_model.cpp:1386-1388`, not `:1385` — `:1385` is the `return true` of the *fixed-joint* arm above it, so the old citation named the wrong one of the lambda's two arms. The unreachability half holds: `crates/moveit-model/src/robot_model.rs:1667` pushes `Diagnostic::EmptyGroup` rather than building the group. The nullptr case upstream handles alongside it (`:1369-1370`, `if (!link) return link`) has no counterpart either, the port taking a `usize`; its caller refuses the same input one level up, at `crates/moveit-kinematics/src/set_from_ik.rs:269-271`. | |
