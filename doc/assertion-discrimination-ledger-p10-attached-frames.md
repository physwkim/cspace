# Assertion-discrimination ledger — p10-attached-frames (`AttachedFrames for PlanningScene`)

The five tests added with `impl moveit_kinematics::AttachedFrames for
PlanningScene` (`crates/moveit-scene/src/scene.rs:2083-2108`), and the
mutations that say what each one discriminates.

The impl is the only production implementor of the seam `set_from_ik` uses
to reach attached bodies, so what its tests must pin is not "an answer came
back" but *which* of its three answers — resolve-or-miss, `link_pose_frame`,
`link_name` — the test would notice going wrong. Three of the five tests
land in the coarse-assertion sweep's corpus (`is_none`, `matches`); the
other two assert against a measured tolerance and so are outside it. They
were bitten anyway — section 2 — because a guard whose bite was never run is
a guard whose test was never checked.

## Method

Baselines at the time of every bite below:

    cargo nextest run -p moveit-scene       # 107 tests, 107 pass
    cargo nextest run -p moveit-kinematics  # 101 tests, 101 pass

Each bite is applied to the named source file, the whole crate is run with
`--no-fail-fast` so a mutation that takes down something elsewhere cannot
hide, and the file is restored with `git checkout -- <path>`. Every bite
that crosses into `moveit-kinematics` was run against both crates and both
results are recorded.

One harness note, because it cost a real revert: `git checkout -- <path>`
restores from the *index*, so it deletes an uncommitted change outright.
The first B1 run reverted the impl under test along with the bite. The
bites below were all run with the round's work `git add`-ed first, which is
what makes the restore a revert of the bite alone; `git status --porcelain`
after each run shows exactly the round's own files and no other
modification.

## 1. The three scanner sites

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-scene/tests/attached_frames_reach_ik.rs:234` | `assert!(AttachedFrames::attached_frame(&scene, "no_such_body").is_none())` — the `?` on `PlanningScene::attached_frame`'s id tier | `a_name_no_attached_body_owns_is_not_an_attached_frame` | discriminating | Bite B3: `PlanningScene::attached_frame(self, frame).unwrap_or(("panda_hand", Isometry3::identity()))`, so every miss answers with the attach link at identity. That one test FAILS, panicking at `:234` itself; the other 106 stay green. Separated from `:239` by B3b, which fails `:239` and leaves `:234` passing — so neither line is reading the other's branch. |
| `crates/moveit-scene/tests/attached_frames_reach_ik.rs:239` | `assert!(AttachedFrames::attached_frame(&scene, "grasped_box/no_such_subframe").is_none())` — the `?` reached through the subframe tier, where the *body id prefix* does match | `a_name_no_attached_body_owns_is_not_an_attached_frame` | discriminating | Bite B3b: the same fallback as B3 but applied only when `frame.contains('/')`, so an unowned bare name still misses and only the unreal subframe resolves. That one test FAILS, panicking at `:239`; the other 106 stay green. This is the line that says a real `grasped_box` prefix cannot stand in for a subframe `grasped_box` does not have — `subframe_pose` returning `None` is what the miss rests on, not the `strip_prefix` that precedes it. |
| `crates/moveit-scene/tests/attached_frames_reach_ik.rs:318` | `assert!(matches!(error, Error::UnknownName { kind: "IK frame", ref name } if name == "grasped_box/grip"))` — `rigid_parent_link`'s `ok_or_else` (`crates/moveit-kinematics/src/set_from_ik.rs:272-274`) | `the_same_target_without_the_seam_is_an_unknown_ik_frame` | discriminating | Bite B6a: that site's `kind` changed from `"IK frame"` to `"rigid parent frame"`. That one test FAILS, panicking at `:318`; the other 106 stay green, and in `moveit-kinematics` only `a_target_naming_nothing_in_the_model_is_unknown_name_not_no_match` moves. Separated from the port's *other* `unknown_name("IK frame", ...)` site by B6b — see section 3, which is also why that site is listed there rather than credited here. |

## 2. Bites on the two guards outside the corpus

Both of these tests compare poses against a measured tolerance, so the
scanner emits no site for them. Each still has a mutation that fails it and
nothing else.

| bite | mutation | result |
|---|---|---|
| B1 | `if !frame.contains('/') { return None; }` ahead of the delegation, so a bare attached-body id no longer resolves | `a_bare_attached_body_id_resolves_with_no_local_offset` FAILS; the other 106 stay green. The id tier is not a special case of the subframe tier: this is the only test that would notice it going missing. |
| B2 | `let link_pose_frame = Isometry3::from(link_pose_frame.translation);` — the stored translation is kept, the stored rotation is dropped | `an_attached_subframe_carries_the_pose_the_scene_stores` FAILS; the other 106 stay green, `the_attach_link_is_the_one_the_rigid_parent_match_reads` among them. That sibling survives because a local-frame rotation moves no frame origin, which is why the fixture's subframe carries one: it is the part of the stored pose that the end-to-end solve is blind to, and so the part only a direct comparison can pin. |
| B4 | `let link_name = if link_name.is_empty() { link_name } else { "panda_link0" };` — a link that exists, so nothing fails to resolve; it is simply not the link the body hangs off | `the_attach_link_is_the_one_the_rigid_parent_match_reads` FAILS; the other 106 stay green. `getRigidlyConnectedParentLinkModel` reads the link and not the pose, so a wrong `link_name` is not recoverable from a right global pose — that is the whole reason `AttachedFrame` carries the two together. |

## 3. Two results worth recording rather than filing as rows

**B5 — `NoAttachedFrames` made to resolve.** `NoAttachedFrames::attached_frame`
was changed to answer `Some(AttachedFrame { link_name: "panda_hand",
link_pose_frame: Isometry3::identity() })`. `the_same_target_without_the_seam_is_an_unknown_ik_frame`
FAILS — but at `:316`, the `expect_err`, not at the `matches!` on `:318`,
because the call now returns `Ok`. So B5 is evidence for *that test's*
premise (without the seam the frame does not resolve) and not for the
coarse site; `:318`'s row cites B6a for that reason. The same bite also
fails `moveit-kinematics`' `a_target_naming_nothing_in_the_model_is_unknown_name_not_no_match`,
so this test is not the only guard on `NoAttachedFrames`. Its work here is
to be the control for `the_attach_link_is_the_one_the_rigid_parent_match_reads`:
same fixture, same target, same solver, seam removed.

**B6b — `frame_transform`'s own `unknown_name` site is unreached.**
`crates/moveit-kinematics/src/set_from_ik.rs:299-301` carries a second
`Error::unknown_name("IK frame", frame)`. Changing its `kind` string fails
*no* test in either crate — `moveit-scene` 107/107 and `moveit-kinematics`
101/101 both stay green. That is not a gap this ledger can close by adding
an assertion: `frame_transform` is called only from `resolve_ik_queries`'
rigid-parent branch, which is reached only after `rigid_parent_link` has
already resolved the same string through the same `AttachedFrames`, so the
lookup cannot miss the second time. The site is unreachable by
construction rather than merely untested, and B6b is the measurement that
says so.
