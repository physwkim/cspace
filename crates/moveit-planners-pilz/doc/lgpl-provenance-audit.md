# LGPL provenance audit — round 21, item 0 (PORTING-PLAN.md §151/§152, D11)

Measurement only. No code or header text in any of the three files below was
changed to produce this report — per §152.1, the rewrite (classification-1
symbols only, one commit per finding, `path_circle.rs`'s standard) happens in
follow-up commits *after* this report is committed, using this table as the
scope evidence.

## Method

Every function/`impl` block in the three files is classified into exactly
one bucket:

1. **transcription** — sentence structure/variable names/branch order follow
   the LGPL original. Cited by the original's exact line range.
2. **independently derivable** — same result, derived directly from
   elementary math/standard algorithms. The derivation is written out as an
   actual paragraph, not asserted.
3. **interface fact only** — constant values, signatures, unit conventions,
   argument order, member layout — not authorial expression.
4. **derived from moveit2 (BSD), not KDL** — sourced from moveit2's own BSD
   `dynamics_solver.{hpp,cpp}`, not from orocos_kdl. No LGPL provenance
   question applies to this bucket regardless of how tight the
   correspondence is, since moveit2 and this workspace are both
   BSD-3-Clause.

Every line of every file is assigned to exactly one of: file-header **meta**
(the copyright/`Ported from` block itself — the subject of this audit, not
its output), **original** (prose/scaffolding with no upstream counterpart —
module docs describing this port's own deviations, `use` imports, this
port's own `#[cfg(test)]` modules, blank separator lines), or one of the
four numbered buckets above. Line ranges are assigned by the rule "a blank
separator line belongs to the item that follows it; a block's own closing
brace belongs to the item it closes" so that ranges partition the file with
no gaps and no overlaps. Sums below were checked to equal each file's exact
`wc -l` line count.

## `velocity_profile_trap.rs` (197 lines)

Ported from `orocos_kdl/src/velocityprofile_trap.{hpp,cpp}` only (no
moveit2/BSD citation in this file — bucket 4 does not apply).

| Region | Lines | Span | Classification | Basis |
|---|---|---|---|---|
| File header | 1-13 | 13 | meta | — |
| Module doc (deviations) | 14-41 | 28 | original | This port's own deviation writeup, no upstream counterpart |
| Struct doc | 42-46 | 5 | original | — |
| Struct scaffolding (derive/decl/braces/blank, `impl` open) | 47,48,53-55,65,69,71 | 8 | original | — |
| Struct fields `max_vel,max_acc,start_pos,end_pos` | 49-52 | 4 | **3** | Field names/units are `VelocityProfile_Trap`'s own public accessor vocabulary; a 4-field struct holding a bounded scalar has no room for authorial expression |
| Struct fields `a1..c3,duration,t1,t2` | 56-64,66-68 | 12 | **1** | Same coefficient names, same grouping (three cubics `a1..a3`/`b1..b3`/`c1..c3`) as `VelocityProfile_Trap`'s private members, `velocityprofile_trap.hpp` |
| `new()` | 72-92 | 21 | **3** | Trivial all-zero constructor; no creative expression possible in a zero-initializer |
| `set_profile()` | 94-133 | 40 | **1** | Same variable names (`t1,t2,a1..c3`), same branch order (triangular-vs-trapezoidal test, then coefficient fill), same formula sequence as `SetProfile`, `velocityprofile_trap.cpp:61-89` |
| `duration()` | 135-138 | 4 | **3** | Trivial getter |
| `pos()` | 140-153 | 14 | **1** | Same 4-branch structure and same Horner-evaluated cubic coefficients as `Pos`, `velocityprofile_trap.cpp:139-151` |
| impl block close | 154 | 1 | original | — |
| tests module | 156-197 | 42 | original | This port's own tests, no upstream equivalent bundled in this file |
| (5 inter-item blanks folded per rule above) | 70,93,134,139,155 | 5 | original | — |

**Totals:** meta 13 · original 89 · bucket 1 (transcription) **66** · bucket 2 **0** · bucket 3 (interface fact) **29** · bucket 4 **0**. Sum: 13+89+66+0+29+0 = 197 ✓.

Of LGPL-relevant content (buckets 1-3, 95 lines): transcription 66/95 ≈ **69.5%**, interface-fact 29/95 ≈ **30.5%**, independently-derivable 0%.

**Conclusion: (C) partial rewrite.** `set_profile()`, `pos()`, and the
`a1..c3,duration,t1,t2` struct fields (66 lines) are bucket 1 and need D11
rewrite. `new()`, `duration()`, and the `max_vel/max_acc/start_pos/end_pos`
fields (29 lines) are already bucket 3 and do not need rewriting. Note
bucket 2 is empty as currently written — the trapezoid-profile physics
*is* independently derivable in principle (constant-acceleration
kinematics, cruise-distance matching against a triangular fallback), but
the current text is a direct translation of `SetProfile`/`Pos`, not an
independently-authored equivalent, so it is measured as bucket 1 not
bucket 2.

## `path_line.rs` (272 lines)

Ported from `orocos_kdl/src/path_line.{hpp,cpp}`,
`rotational_interpolation_sa.{hpp,cpp}`, and `frames.cpp` (`GetRotAngle`,
`Rot2`). No moveit2/BSD citation — bucket 4 does not apply.

| Region | Lines | Span | Classification | Basis |
|---|---|---|---|---|
| File header | 1-13 | 13 | meta | — |
| Module doc (deviations) + blank | 14-42 | 29 | original | This port's own deviation writeup |
| Imports + blanks | 43-47 | 5 | original | — |
| `kdl_normalize()` + blank | 48-63 | 16 | **1** | Same unit-X/norm-`0.0` degenerate-fallback convention as `Vector::Normalize`, `frames.cpp:147-156` — this convention is a specific, non-obvious design choice (not IEEE `NaN`), not elementary-math-derivable |
| `get_rot_angle()` + blank | 64-113 | 50 | **1** | Same `eps`/`eps2` singularity thresholds, same largest-diagonal 3-way branch for the `angle == PI` case, same `atan2(‖axis‖/2, f)` general-case formula, variable-for-variable, as `Rotation::GetRotAngle`, `frames.cpp:358-428` |
| `PathLine` struct doc+decl + blank | 114-128 | 15 | **3** | Field names/roles (`v_base_start`,`v_start_end`,`path_length`,`scale_lin`,`scale_rot`) mirror `Path_Line`'s own member list (`V_base_start`,`V_start_end`,`pathlength`,`scalelin`,`scalerot`) plus the folded-in `RotationalInterpolation_SingleAxis` state (`orient_start`,`rot_axis`) — member-layout convention, not expression |
| `PathLine::new()` + blank | 129-163 | 35 | **1** | `eqradius` threshold branching matches `Path_Line`'s constructor, `path_line.cpp:47-84`, exactly; the `SetStartEnd`+`Angle()` composition matches `RotationalInterpolation_SingleAxis::SetStartEnd`, `rotational_interpolation_sa.cpp:52-57`, exactly |
| `PathLine::path_length()` + blank | 164-168 | 5 | **3** | Trivial getter |
| `PathLine::pos()` + blank + impl close | 169-178 | 10 | **2** | See derivation below |
| tests module + blank | 179-272 | 94 | original | This port's own tests |

**`pos()`'s derivation (bucket 2):** the interpolated pose at arclength `s`
decomposes into two independent parts. The translation,
`v_base_start + s*scale_lin*v_start_end`, is a first-order interpolation
along the straight-line displacement vector — elementary vector algebra,
one term times a scalar plus a base point. The rotation is built by
composing the start orientation with a rotation of angle `s*scale_rot`
about the fixed axis computed once in `new()`; given an axis and an angle,
the standard construction for "rotation of angle θ about axis n" is
`UnitQuaternion::from_axis_angle` — Rodrigues' formula in quaternion form
(`cos(θ/2) + sin(θ/2)·n`), the one canonical way to build that rotation,
not a coincidental match to `Rotation::Rot2`'s different (explicit 3×3
matrix Rodrigues expansion) code path, which this function does not use at
all. Composing `orient_start * rotation_of(θ)` and packing the two
independently-computed parts into one `Isometry3` is the only way to
combine a separately-computed translation and rotation into one pose.

**Totals:** meta 13 · original 128 (29+5+94) · bucket 1 (transcription)
**101** (16+50+35) · bucket 2 **10** · bucket 3 (interface fact) **20**
(15+5) · bucket 4 **0**. Sum: 13+128+101+10+20+0 = 272 ✓.

Of LGPL-relevant content (buckets 1-3, 131 lines): transcription
101/131 ≈ **77.1%**, interface-fact 20/131 ≈ **15.3%**, independently-derivable
10/131 ≈ **7.6%**.

**Conclusion: (C) partial rewrite**, but concentrated — `kdl_normalize`,
`get_rot_angle`, and `PathLine::new` (101 of the file's 272 lines, and 77%
of its LGPL-relevant content) are bucket 1 and need D11 rewrite.
`path_length` (bucket 3) and `pos` (bucket 2, already independently
derived) do not.

## `dynamics.rs` (581 lines)

Cites both `orocos_kdl` (`chainidsolver_recursive_newton_euler.{hpp,cpp}`,
`frames.{hpp,inl}`, `rigidbodyinertia.{hpp,cpp}`, `rotationalinertia.hpp`,
`segment.{hpp,cpp}`, `joint.{hpp,cpp}`, `chain.{hpp,cpp}`) and moveit2's own
BSD `dynamics_solver.{hpp,cpp}`. Both `dynamics_solver.hpp` and
`dynamics_solver.cpp` were read in full for this audit (previously
unread by this port's own commit history) to separate bucket 4 from
buckets 1-3.

| Region | Lines | Span | Classification | Basis |
|---|---|---|---|---|
| File header | 1-18 | 18 | meta | — |
| Module doc (5 deviations) + imports + blanks | 19-119 | 101 | original | This port's own deviation writeup (already argues the `X[i]`/`S[i]` substitution in deviation 1, reused as bucket-2 basis below) |
| `Twist` struct + blank | 120-127 | 8 | **3** | `vel`,`rot` field names/order mirror `KDL::Twist`'s own member layout; a 2-field data struct has no room for expression |
| `impl Twist` (`zero`/`scale`/`add`) + blank | 128-150 | 23 | **2** | See vector-space derivation below |
| `Wrench` struct + blank | 151-158 | 8 | **3** | Same reasoning as `Twist`, vs `KDL::Wrench`'s `force`,`torque` |
| `impl Wrench` (`zero`/`add`/`sub`) + blank | 159-181 | 23 | **2** | Same vector-space derivation as `impl Twist` |
| `RigidBodyInertia` struct + blank | 182-193 | 12 | **3** | `mass`,`h`,`inertia` mirror `KDL::RigidBodyInertia`'s private `m`,`h`,`I` |
| `rigid_body_inertia_from_link()` + blank | 194-209 | 16 | **1** | `inertia = ic - m*(c*c^T - c.dot(c)*Id)` is a variable-for-variable match to the two-argument constructor's `I=Ic-c x c x` expansion, `rigidbodyinertia.cpp:35-40` |
| `frame_inverse_twist()` + blank | 210-219 | 10 | **1** | Exact match to `Frame::Inverse(const Twist&)`, `frames.inl:282-288` (Rust doc comment already quotes the formula verbatim) |
| `frame_mul_wrench()` + blank | 220-227 | 8 | **1** | Exact match to `Frame::operator*(const Wrench&)`, `frames.inl:156-163` |
| `twist_cross()` + blank | 228-236 | 9 | **1** | Exact match to `operator*(const Twist&,const Twist&)`, `frames.inl:379-382` |
| `twist_cross_wrench()` + blank | 237-246 | 10 | **1** | Exact match to `operator*(const Twist&,const Wrench&)`, `frames.inl:383-386` |
| `dot_twist_wrench()` + blank | 247-251 | 5 | **1** | Exact match to `dot(const Twist&,const Wrench&)`, `frames.inl:1016-1018` |
| `inertia_mul_twist()` + blank | 252-260 | 9 | **1** | Exact match to `operator*(const RigidBodyInertia&,const Twist&)`, `rigidbodyinertia.cpp:50-52` |
| `MaxPayload` struct + blank | 261-274 | 14 | **4** | Data-carrying return type for `getMaxPayload`'s `double& payload, unsigned int& joint_saturated` out-parameters, `dynamics_solver.hpp`'s signature — moveit2-sourced, not KDL |
| `DynamicsSolver` struct + blank | 275-287 | 13 | **4** | Holds the moveit2-sourced responsibility (`model`/`group`/`gravity`/`max_torques` vs. upstream's `robot_model_`/`joint_model_group_`/`gravity_`/`max_torques_`), though the field set differs substantially (no `kdl_chain_`, no `state_`, no `base_name_`/`tip_name_` — those are the documented `RobotModel`-based deviation) |
| `impl` open + `new()` + blank | 288-345 | 58 | **4** | Precondition sequence (chain check, mimic-joint check, parent-link check) matches `DynamicsSolver::DynamicsSolver`'s own check order, `dynamics_solver.cpp` constructor — moveit2-sourced, restructured from `RCLCPP_ERROR`+early-return-with-null-group into `Result` |
| `max_torques()` + blank | 346-351 | 6 | **3** | Trivial accessor |
| `num_active()` + blank | 352-355 | 4 | **3** | Trivial accessor, no upstream counterpart (this port's own active/full index-space split) |
| `num_segments()` + blank | 356-359 | 4 | **3** | Trivial accessor |
| `torques()` + blank | 360-374 | 15 | **4** | Corresponds to `getTorques`'s role (zero external wrenches, call the RNE solve) — moveit2-sourced, though most of `getTorques`'s own body (KDL type marshaling, size validation) has no counterpart since `sweep()` takes plain slices |
| `max_payload()` + blank | 375-426 | 52 | **4** | `candidate = max((max_torque-zero_torque)/delta, (-max_torque-zero_torque)/delta)` is a variable-for-variable match to `getMaxPayload`'s own `payload_joint = max((max_torques_[i]-zero_torques[i])/(torques[i]-zero_torques[i]), (-max_torques_[i]-zero_torques[i])/(...))` — moveit2-sourced, not KDL, so the tightness of this match raises no LGPL question |
| `payload_torques()` + blank | 427-444 | 18 | **4** | Corresponds to `getPayloadTorques` — moveit2-sourced |
| `sweep()` signature + validation + blank | 445-479 | 35 | **3** | Validates the same array-length facts as `CartToJnt`'s `E_SIZE_MISMATCH` check, `chainidsolver_recursive_newton_euler.cpp:49`, in a different (per-array, per-error-message) shape |
| `ag` computation + blank | 480-485 | 6 | **1** | Exact match to the constructor's `ag=-Twist(grav,Vector::Zero())`, `chainidsolver_recursive_newton_euler.cpp:31` |
| Per-sweep state vec setup + blank | 486-493 | 8 | **2** | See `X[i]`/`S[i]` derivation below |
| Per-segment `sk`/`is_active`/`xk` setup | 494-535 | 42 | **2** | See `X[i]`/`S[i]` derivation below |
| `vj` computation + blank | 536-537 | 2 | **2** | See `vj` derivation below |
| `v`/`a` recurrence | 538-551 | 14 | **1** | Exact structural match (including the `i==0` special case) to `v[i]=X[i].Inverse(v[i-1])+vj; a[i]=X[i].Inverse(a[i-1])+S[i]*qdotdot_+v[i]*vj;`, `chainidsolver_recursive_newton_euler.cpp:74-80` |
| Inertia + `fk` computation + blank | 552-556 | 5 | **1** | Exact match to `f[i]=Ii*a[i]+v[i]*(Ii*v[i])-f_ext[i];`, `chainidsolver_recursive_newton_euler.cpp:84` |
| Accumulation (`base_to_tip`, vec pushes, `prev_v`/`prev_a`) + blank | 557-564 | 8 | **2** | See accumulation derivation below |
| Backward sweep + blank | 565-577 | 13 | **1** | Matches `torques(j)=dot(S[i],f[i]); ... f[i-1]=f[i-1]+X[i]*f[i];`, `chainidsolver_recursive_newton_euler.cpp:91,96`, minus the joint-inertia term (documented omission, module doc's final section) |
| Return + fn/impl close + blank | 578-581 | 4 | **3** | Trivial |

**Bucket-2 derivations:**

- **`impl Twist`/`impl Wrench` arithmetic (`zero`/`scale`/`add`/`sub`):**
  `Twist`/`Wrench` are ordered pairs of two 3-vectors. Their zero element is
  the pair of zero vectors — the additive identity of the product vector
  space ℝ³×ℝ³; scaling multiplies each component vector by the scalar;
  addition/subtraction adds/subtracts component-wise. There is exactly one
  way to define `+`, scaling, and `0` that makes a Cartesian product of two
  copies of ℝ³ a vector space, so independent authorship reproduces the
  identical expression regardless of prior exposure to `KDL::Twist::Zero`/
  `operator*(double)`/`operator+`.
- **`X[i]`/`S[i]` setup (per-segment block, lines 494-535, and the vec setup
  feeding it, 486-493):** already derived in this file's own module doc,
  deviation 1 (lines 34-50 of this file) — `X[i]` is provably
  `LinkModel::joint_origin_transform` composed with
  `JointModel::compute_transform` (the one physically correct answer for a
  URDF joint's pose-as-a-function-of-its-variable, independently verified
  against the oracle's `fk` op by `crate::state`), and `S[i]` is provably
  the joint's local axis with no reference-point offset (URDF's own
  convention places a joint's axis through the child frame's origin). This
  is the port's alternative to `chain.getSegment(i).pose(q_)`/
  `chain.getSegment(i).twist(q_,1.0)`, which it cannot call — the module doc
  already states there is no reachable `KDL::Chain` in this design.
- **`vj` computation:** `vj = S[i]*qdot`, the segment's velocity
  contribution from its own joint motion, follows directly from `S[i]`'s
  definition as the joint's unit twist — one unit of joint velocity
  produces exactly `S[i]` as the resulting spatial velocity, by definition
  of a 1-DOF joint's motion subspace, so `qdot` units of joint velocity
  produce `qdot*S[i]`. This restates the standard spatial-velocity
  decomposition for a single-DOF joint; it does not transcribe
  `X[i].M.Inverse(chain.getSegment(i).twist(q_,qdot_))`, which computes the
  identical physical quantity via a `Segment`-object route this port's
  `RobotModel`-based design has no equivalent object for.
- **Accumulation block:** `base_to_tip *= xk` accumulated across the
  forward sweep is the standard definition of a chain's total transform as
  the ordered product of its per-segment transforms
  (`X[0]*X[1]*...*X[ns-1]`) — needed here, unlike upstream (which instead
  queries `RobotState::getFrameTransform` for the same quantity, per the
  module doc's deviation-1 closing sentence, which already states this
  identity), because this port has no `RobotState`. Storing `xk`/`sk`/`fk`
  per index for the backward sweep to read is the ordinary two-pass-
  algorithm pattern — retaining forward-pass state for a backward pass is
  the only structurally possible way to run a two-sweep algorithm without
  recomputing the forward pass a second time.

**Totals:** meta 18 · original 101 · bucket 1 (transcription) **105**
(16+10+8+9+10+5+9+6+14+5+13) · bucket 2 (independently derivable) **106**
(23+23+8+42+2+8) · bucket 3 (interface fact) **81**
(8+8+12+6+4+4+35+4) · bucket 4 (moveit2-BSD) **170**
(14+13+58+15+52+18).
Sum: 18+101+105+106+81+170 = 581 ✓.

Of KDL-LGPL-relevant content only (buckets 1-3, excluding bucket 4's
moveit2/BSD content which raises no LGPL question at all: 292 lines):
transcription 105/292 ≈ **36.0%**, independently-derivable 106/292 ≈
**36.3%**, interface-fact 81/292 ≈ **27.7%**.

**Special check — does the header wording match what this measurement
found?** The header reads:

> Every operator below was diffed against the headers the oracle's own
> image ships under `/usr/include/kdl` before being trusted

This phrasing describes verification-by-reading — implying the Rust text
was independently written and then checked against KDL, not copied from
it. The measurement does not support that reading for the 105 bucket-1
lines: `frame_inverse_twist`, `frame_mul_wrench`, `twist_cross`,
`twist_cross_wrench`, `dot_twist_wrench`, and `inertia_mul_twist` all have
doc comments that **quote the upstream formula verbatim**, and each was
confirmed byte-for-byte against the actual upstream source in this audit
(`frames.inl`, `rigidbodyinertia.cpp`) — not merely structurally similar,
but the same expression, same variable order. `sweep()`'s `ag` computation,
its `v`/`a` recurrence, its inertia/force computation, and its backward
sweep are equally tight matches to `ChainIdSolver_RNE`'s constructor and
`CartToJnt`. That is transcription, not verification-reading. The header
wording is accurate only for the parts of the file that are genuinely
bucket 2 (the `X[i]`/`S[i]`/`vj`/accumulation substitutions, which really
were independently derived and then checked for equivalence) and bucket 4
(the `DynamicsSolver` public API, legitimately read from and checked
against moveit2's own BSD source, not KDL). **The header wording
understates what happened for the 105 bucket-1 lines** — it needs both a
wording fix (for the file as a whole) and code rewrite (for the bucket-1
symbols specifically), not a wording fix alone.

**Conclusion: (C) partial rewrite**, cutting across two axes at once.
Bucket 1 (needs D11 rewrite, 105 lines): `rigid_body_inertia_from_link`,
`frame_inverse_twist`, `frame_mul_wrench`, `twist_cross`,
`twist_cross_wrench`, `dot_twist_wrench`, `inertia_mul_twist`, and within
`sweep()`: the `ag` computation, the `v`/`a` recurrence, the inertia/`fk`
computation, and the backward sweep. Bucket 2/3 (no rewrite,
201 lines): the `Twist`/`Wrench` structs and arithmetic, `RigidBodyInertia`
struct, `sweep()`'s `X[i]`/`S[i]`/`vj`/accumulation logic and its
validation prologue. Bucket 4 (no rewrite, license-compatible regardless of
transcription tightness, 156 lines): `MaxPayload`, `DynamicsSolver`,
`new`, `max_torques`/`num_active`/`num_segments`, `torques`,
`max_payload`, `payload_torques`.

## Cross-file summary

| File | Total | meta | original | B1 transcription | B2 derivable | B3 interface | B4 moveit2-BSD | Verdict |
|---|---|---|---|---|---|---|---|---|
| `velocity_profile_trap.rs` | 197 | 13 | 89 | 66 | 0 | 29 | 0 | (C) — rewrite `set_profile`,`pos`,coefficient fields |
| `path_line.rs` | 272 | 13 | 128 | 101 | 10 | 20 | 0 | (C) — rewrite `kdl_normalize`,`get_rot_angle`,`PathLine::new` |
| `dynamics.rs` | 581 | 18 | 101 | 105 | 106 | 81 | 170 | (C) — rewrite the `frames.inl`/`rigidbodyinertia.cpp` helpers and `sweep`'s recurrence/force/backward-sweep; header wording also needs correcting |

272 lines across the three files (66+101+105) are bucket-1 transcription
and are the D11 rewrite scope for the next round of commits.
