# Folded-operand guards — the population, from two instruments

A guard that folds N≥2 independently-named operands into one condition
reaching a single construction site is **N covered branches, not one**.
`if radius < 0.0 || length < 0.0 { return Err(Error::construct(...)) }`
has one `Error::construct` and two branches: dropping the `radius`
operand fails `Cylinder::new(-1.0, 1.0).is_err()` at `crates/moveit-geometry/src/shapes.rs:1624`,
dropping `length` fails `Cylinder::new(1.0, -1.0).is_err()` at `:1625`.

A constructor-token count sees the one and reports `single-branch`. It is
not a sloppy instrument — it measures construction sites, and the branch
count is a property of the *condition*. Twelve `moveit-geometry` sites
carried a wrong verdict from this (`ab0f1ff`), five of which a prior audit
had confirmed by reading each function body.

## Two anchors, neither complete

**p3-acm's** (literal, from the cited line's own tokens):

```
rg -n --type rust -g '!target' -F '< 0.0 ||'
rg -n --type rust -g '!target' -F '.is_empty() ||'
rg -n --type rust -g '!target' -e '\|\| .*\.is_empty\(\)'
```

**The orchestrator's** (structural): mask comments; find `if <cond> {`
where `cond` contains `||` or `&&`; require the block to open with
`return Err(` / `Err(` / `None`; count distinct operands bearing a
comparison, `.is_empty()`, `.is_none()` or `.is_nan()`.

They are complementary, and the difference is the lesson. p3-acm's three
literal anchors were taken from the shape of the site that was cited, so
they find `< 0.0 ||` and `.is_empty() ||` and nothing else — they miss
every `&&` guard, every `>=`/`==`/`!=` comparison, and every operand pair
that is neither a float-negativity nor an emptiness check. The structural
anchor misses what p3-acm's caught: guards whose body is `continue`
inside a loop rather than a `return`, and guards returning a plain tuple
whose assertions are `assert_eq!` and therefore outside this sweep's
grammar. **The union is the population; either alone is a sample.**

## Population (union, main @ `0379f9d`)

| site | operands | condition | owner |
|---|---:|---|---|
| `moveit-collision/src/tools.rs:68` | 3 | `min[i] >= max[i]` per axis | fixed, `d24494d` |
| `moveit-constraints/src/ik_sampler.rs:134` | 2 | `position_constraint.is_none() && orientation_constraint.is_none()` | p1-fixtures |
| `moveit-constraints/src/joint.rs:120` | 2 | `tolerance_above < 0.0 \|\| tolerance_below < 0.0` | p1-fixtures |
| `moveit-geometry/src/bodies.rs:2209` | 3 | `half_length/half_width/half_height < 0.0` | fixed, `ab0f1ff` |
| `moveit-geometry/src/shapes.rs:686,714,801,823` | 2 | `radius < 0.0 \|\| length < 0.0` | fixed, `ab0f1ff` |
| `moveit-geometry/src/shapes.rs:905,929` | 3 | `x \| y \| z < 0.0` | fixed, `ab0f1ff` |
| `moveit-model/src/robot_model.rs:841` | 2 | `child_link != root \|\| parent_frame.is_empty()` (`continue`) | p9-ros |
| `moveit-planners-pilz/src/trajectory_functions.rs:549` | 2 | `n1 < 2 && n2 < 2` | p1-robotmodel |
| `moveit-scene/src/scene.rs:1212` | 2 | `shapes.is_empty() \|\| shapes.len() != shape_poses.len()` | p1-fixtures |
| `moveit-smoothing/src/acceleration_filter.rs:223` | 2 | `l > 0.0 \|\| u < 0.0` | p1-fixtures |
| `moveit-trajectory/src/robot_trajectory.rs:235,304` | 2 | `index == 0 && value/dt != 0.0` | p1-robotmodel |
| `moveit-trajectory/src/robot_trajectory.rs:261,349` | 2 | `waypoints.is_empty() && dt != 0.0` | p1-robotmodel |
| `moveit-trajectory/src/robot_trajectory.rs:535` | 2 | `duration < 0.0 \|\| waypoints.is_empty()` (tuple return) | p1-robotmodel |
| `ros/moveit-ros/src/constraints/position.rs:151` | 2 | `!meshes.is_empty() \|\| !mesh_poses.is_empty()` | p9-ros |
| `ros/moveit-ros/src/conversion_coverage.rs:219` | 2 | `from_base.is_empty() \|\| to_base.is_empty()` | p9-ros |
| `ros/moveit-ros/src/planning.rs:112` | 2 | `!joint_names.is_empty() \|\| !points.is_empty()` | p9-ros |
| `ros/moveit-ros/src/scene/collision_object.rs:345` | 3 | `primitives/meshes/planes.is_empty()` | p9-ros |
| `ros/moveit-ros/src/scene/planning_scene.rs:289` | 2 | `if msg.entry_names.len() != msg.entry_values.len()`, `\|\| msg.default_entry_names.len() != msg.default_entry_values.len()` | kind 2, fixed — `an_acm_with_mismatched_entry_lengths_is_rejected` |
| `ros/moveit-ros/src/state.rs:112` | 4 | `!joint_names/transforms/twist/wrench.is_empty()` | p9-ros |
| `ros/moveit-ros/src/trajectory.rs:165` | 2 | `i == 0 && t != 0.0` | p9-ros |
| `tools/moveit-diff/src/rust_impl.rs:393` | 2 | `link_names[0].is_empty() \|\| link_names[1].is_empty()` | closed, see below |

One row postdates the `0379f9d` enumeration and was found by mutating, not
by either anchor: the guard reading
`if msg.entry_names.len() != msg.entry_values.len()` (`planning_scene.rs:289`)
arrived with `4f2c9df` (p11-scenetopic's `usePlanningSceneMsg` port) after
this table was measured.
Its `default_entry_*` operand had a test (`an_acm_with_mismatched_default_lengths_is_rejected`),
its `entry_*` operand had none — replacing that clause with `false` left the
whole `moveit-ros` suite at 203 passed. Kind 2, now closed with
`an_acm_with_mismatched_entry_lengths_is_rejected`. The lesson is the same
one the two anchors already teach: a table measured at one main is a sample
of a moving population, so new ported code owes this check again rather
than inheriting the old row set.

**Excluded, different shape:** `ros/moveit-ros/src/trajectory.rs:45` and
`moveit-distance-field`'s `checked_max_distance_sq` fold one variable
checked three ways (finite / `>= 0` / `<= MAX`), not N named operands.

**A folded guard is not automatically a sweep row.** This sweep is about
*assertions* that cannot name what produced their result. A folded-operand
guard with no assertion targeting it at all is a coverage gap, which is a
different (and usually smaller) problem: there is no wrong verdict to
correct, because nobody recorded a verdict. Sites in the table above split
into three kinds and only the first is a sweep finding:

1. a guard whose assertions exist and were verdicted `single-branch` on a
   constructor count — the misverdict this table exists for;
2. a guard whose assertions exist but exercise the operands only jointly —
   a blind site (`acceleration_filter.rs:302` and `ruckig_filter.rs`'s
   `reset`, both fixed in `3c2d72f`/`2829ca2`; `tools.rs:68`, whose x axis
   was covered and whose y and z were not, fixed in `d24494d`);
3. a guard with no assertion at all — note it, do not manufacture a
   verdict for it.

`tools/moveit-diff/src/rust_impl.rs:393` is kind 3, and it is **closed with
no action**. `distance_pair` is private, has no direct unit test, and is
exercised only through the parity harness. Its guard is character-for-
character upstream's: `oracle.cpp:2711` reads `if (d.link_names[0].empty()
|| d.link_names[1].empty()) return nullptr;`, and even the port's comment
saying the names are "empty together" is inherited from upstream's own
comment four lines above it. The `||`/"together" mismatch is upstream's
wording, not a porting error, so neither the guard nor the comment should
be changed here — doing so would deviate from the text the port mirrors.
`moveit-diff` is also tool code, not a ported crate, so it carries no
parity obligation of its own.

## The check each site owes

Neutralize one operand's clause; confirm the assertion that targets *that*
operand fails while its sibling, isolated under the same mutation, stays
green; then the mirror. `assert!` short-circuits, so sibling assertions in
the same test function must be commented out to isolate. An operand no
test covers is a blind site and gets a test, not a comment.
