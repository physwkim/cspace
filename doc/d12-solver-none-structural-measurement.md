# D12 — measuring whether `solver: None` is a crate-layering defect

Round 22 brief, item 1. Three rounds running (20, 21, 22) the same UNFIXED
has recurred: `Goal::Constraints`/goal-region sampling with no
`JointConstraint` coverage falls back to uniform sampling because
`select_default_sampler`'s IK path needs a `KinematicsSolver` and no
production call site has one to give (`solver: None` hardcoded or defaulted
at every call site). The brief's instruction: measure before touching
anything, and if the measurement does not support extracting `KinematicsSolver`
into a crate below `moveit-model`, name what actually blocks it and stop.

This is a report commit only. No code changes accompany it, per the brief's
explicit ordering ("이번 라운드 산출물은 보고서 커밋 하나가 먼저다").

## 1. What would have to move

`moveit-kinematics::registry` (`crates/moveit-kinematics/src/registry.rs`,
190 lines) defines `trait KinematicsSolver` with 5 required methods (plus one
default-bodied `solve`, not counted since it only calls
`solve_with_options`). Every type each signature touches:

| Method | Types referenced | Crate that defines it | Below `moveit-model`? |
|---|---|---|---|
| `group_name(&self) -> &str` | `str` | std | yes (std) |
| `joint_names(&self) -> &[String]` | `String` | std | yes |
| `base_frame(&self) -> &str` | `str` | std | yes |
| `tip_frame(&self) -> &str` | `str` | std | yes |
| `solve_with_options(&mut self, seed: &[f64], target: &Isometry3, options: &mut SolveOptions) -> Option<Vec<f64>>` | `f64`, `Vec<f64>` | std | yes |
| ″ | `Isometry3` | `moveit-geometry` | **yes** — `moveit-model/Cargo.toml` already depends on `moveit-geometry` directly |
| ″ | `SolveOptions` (`consistency_limits: Option<&[f64]>`, `solution_callback: Option<&mut SolutionCallback>`) | `moveit-kinematics::registry` itself | moves with the trait |
| ″ | `SolutionCallback = dyn FnMut(&[f64]) -> bool` | `moveit-kinematics::registry` itself | moves with the trait |

Result: **every type the 5 trait methods touch is either `std` or
`moveit-geometry`.** Nothing in the trait's own signatures names
`RobotModel`, `JointModelGroup`, or any other `moveit-model` type. A new
crate — call it `moveit-kinematics-base` per the brief's working name —
holding `KinematicsSolver` + `SolveOptions` + `SolutionCallback` would need
only `moveit-geometry` (and, for the panic-vs-error convention the trait's
own doc uses, optionally `moveit-error`, though nothing here actually
returns `moveit_error::Result`). Both already sit below `moveit-model` in
the existing graph, so `moveit-model` could depend on
`moveit-kinematics-base` with no new cycle.

`SolverRegistration`/`KINEMATICS_SOLVERS` (the `linkme` compile-time
registry, and `construct: fn(&RobotModel, ...) -> Result<Box<dyn
KinematicsSolver>>`) are a **separate** struct from the trait — they do name
`RobotModel`, and they stay in `moveit-kinematics` (which keeps depending on
`moveit-model`, unchanged). Only the trait + its two small support types
would move.

**Verdict on this step: no Rust-specific obstacle.** The extraction is
mechanically straightforward.

## 2. Could `JointModelGroup` hold a solver, and what would the minimal API be

Yes, mechanically. Upstream's shape (`joint_model_group.hpp:100`,
`kinematics::KinematicsBasePtr solver_instance_;`; `:537-545`, two
`getSolverInstance()` overloads returning it; `:529-535`,
`setSolverAllocators(const SolverAllocatorFn&, ...)` where
`SolverAllocatorFn = std::function<KinematicsBasePtr(const
JointModelGroup*)>`, `joint_model_group.hpp:56`) would port as, minimally:

```rust
pub(crate) solver: Option<Arc<dyn moveit_kinematics_base::KinematicsSolver>>,
// + fn solver(&self) -> Option<&dyn KinematicsSolver>
// + fn set_solver(&mut self, solver: Arc<dyn KinematicsSolver>)
```

`Arc`, not `Box`, is load-bearing, not a style choice: `JointModelGroup`
derives `Clone` (`joint_model_group.rs:51`) and that `Clone` is used in
production, not just tests —
`moveit-constraints/src/constraint_sampler_manager.rs:152`,
`crates/moveit-constraints/src/sampler.rs:184`, `crates/moveit-constraints/src/sampler.rs:377` all call `.clone()` on a
`JointModelGroup` fetched from the model. `Box<dyn Trait>` has no blanket
`Clone`; `Arc<dyn Trait>` does (a refcount bump), which is exactly why
upstream's own `solver_instance_` is a `shared_ptr` and not a
`unique_ptr` — this port would be reproducing that choice for the same
reason, not inventing it.

`PartialEq` (`joint_model_group.rs:51`, also derived) is the second cost.
No call site comparing two whole `JointModelGroup` values with `==` turned
up (`rg` for `assert_eq!(group` / `group ==` / `== group` against
`JointModelGroup` across every crate's `src/`+`tests/` — no hit outside
field- or method-level comparisons, e.g. `group.joint_names() == [...]`).
Trait objects have no structural `PartialEq`, so `#[derive(PartialEq)]`
would stop compiling the moment a `solver` field is added; the fix would be
a hand-written impl that either ignores the field or compares by
`Arc::ptr_eq`, silently narrowing what `==` means for this type. Not a
blocker (nothing found depends on the current derive), but not free either.

## 3. `check-dep-direction.sh` compatibility

Read first, not assumed. The script's actual rule (`tools/ci/check-dep-direction.sh:1-4`,
its own header comment): enforces PORTING-PLAN.md §3 — **no workspace
member may depend on a ROS client library**. The banned-package regex
(`BANNED_RE='^(r2r|r2r_.*|rclrs|ros2-client|rustdds|rosidl_.*)$'`) is
checked with `cargo tree -p $pkg -e normal,dev,build` against every
workspace member.

It does **not** enforce any internal crate-layering/DAG rule among
`moveit-*` crates — that direction is enforced only by Cargo's own
cycle rejection at build time, not by this script. A new
`moveit-kinematics-base` crate depending only on `moveit-geometry`/std
introduces no ROS dependency, so `check-dep-direction.sh` would pass
trivially regardless of where the new crate sits in the internal graph.
The brief's framing ("새 크레이트 배치가 이 규칙 안에 있는지") presupposes
this script polices layering; it does not — it polices ROS-independence.
`check-workspace-dep-inheritance.sh` is the one that would actually apply:
a new crate must be added to root `Cargo.toml`'s `[workspace.dependencies]`
and referenced everywhere as `moveit-kinematics-base.workspace = true`
(`members = ["crates/*", ...]` already globs in any new directory under
`crates/`, no separate registration needed there).

## 4. Crate census — who depends on `moveit-kinematics`, and how

Non-self dependents (`cargo tree -i moveit-kinematics` transitively, and
each crate's own `Cargo.toml`/source):

| Crate | Direct Cargo dep? | Uses `KinematicsSolver` trait / `SolveOptions`? | Uses `KINEMATICS_SOLVERS` / concrete solvers? |
|---|---|---|---|
| `moveit-constraints` | yes (`Cargo.toml:15`) | yes (`constraint_sampler_manager.rs`, `ik_sampler.rs`) | no |
| `moveit-planners-stomp` | yes (`Cargo.toml:15`) | yes (`planner.rs`, `sample_goal_state`'s `solver: Option<Box<dyn KinematicsSolver>>` parameter, and its `SolveOptions` test double) | no |
| `moveit-planners-pilz` | yes (`Cargo.toml:15`) | yes (`trajectory_functions.rs`) | **yes** — `trajectory_generator{,_circ,_lin,_ptp}.rs` all import `KINEMATICS_SOLVERS`/`SolverParams` and look solvers up by name |
| `moveit-planners-sbp` | **no** — not in `Cargo.toml` at all | no — `rg 'moveit_kinematics::'` in this crate's `src/` finds one hit, and it is a doc-comment reference inside a prose sentence, not a `use` | no |

At this round (`cc377ffc`), `moveit-planners-sbp` reached `moveit-kinematics`
only transitively, twice over (`cargo tree -p moveit-planners-sbp -i
moveit-kinematics`: `moveit-kinematics ← moveit-constraints ←
moveit-planners-sbp`, and again via `moveit-scene ← moveit-planners-sbp`). It
named the trait nowhere, because `select_default_sampler`'s `solver:
Option<Box<dyn KinematicsSolver>>` parameter was filled with `None` at all
four of its call sites in the crate — three in `registry.rs` and one in
`goal_sampler.rs`, at that revision — and type inference resolves `None`'s
type from the callee's signature, so the caller never had to spell
`KinematicsSolver` itself. That corrected the brief's "moveit-kinematics에
의존하는 것은 5개" framing for one member: sbp was a transitive, zero-usage
dependent, not a direct one.

**갱신 (2026-08-06): 이 행과 위 문단은 `9796b2c0` 이후로 참이 아니다.**
That commit ("planners-sbp(registry): wire a caller-supplied solver into the
goal sampler") closes the §163.3 follow-up to this report's own rejection, and
it makes sbp a direct dependent on every column this table measures.
`crates/moveit-planners-sbp/Cargo.toml:22` carries
`moveit-kinematics.workspace = true`. The crate now names the trait:
`crates/moveit-planners-sbp/src/registry.rs:228` is
`use moveit_kinematics::{KinematicsSolver, SolveOptions};`, and
`crates/moveit-planners-sbp/src/registry.rs:686-688` opens
`impl KinematicsSolver for SharedKinematicsSolver` and its first method,
`fn group_name(&self) -> &str {`, on a private adapter this crate defines. The production call site inside `resolve_constraint_sampler`,
`crates/moveit-planners-sbp/src/registry.rs:747`, no longer passes `None` — it
passes the caller's own solver. The third column is still `no`: nothing in the
crate looks a solver up by name, which is D4's standing exclusion, and the
`Answer` paragraph below therefore still holds for `KINEMATICS_SOLVERS`.

The four line numbers the paragraph above used to carry were dropped rather
than renumbered, and deliberately are not restated here in citation form: each
of them named a `select_default_sampler(` call at `cc377ffc` and none does
today, so writing them as `path:line` spans would put four claims about the
current tree into a gate's corpus that are false by construction. The crate
has since grown past all four, and a later content-equality remap moved them
onto lines that hold the same text without holding the same meaning. Today the
calls are `crates/moveit-planners-sbp/src/registry.rs:747` and
`crates/moveit-planners-sbp/src/registry.rs:3238` (two, not three) and
`crates/moveit-planners-sbp/src/goal_sampler.rs:304` and
`crates/moveit-planners-sbp/src/goal_sampler.rs:400` (two, not one), so no
substitution preserves the sentence's count either.

**Answer to "trait-only vs needs the full registry":** of the 3 crates with
a *direct* Cargo dependency, 2 (`moveit-constraints`, `moveit-planners-stomp`)
use only the trait + `SolveOptions`/`SolutionCallback` — both could switch
to depending on `moveit-kinematics-base` alone. 1 (`moveit-planners-pilz`)
uses the compile-time solver registry itself and must keep depending on the
full `moveit-kinematics`. `moveit-planners-sbp` needs neither directly.

**갱신 (2026-08-06):** `9796b2c0` moves sbp into the first group, not out of
the count: it is now a 4th direct dependent that uses the trait alone, so it
could switch to `moveit-kinematics-base` on the same terms as the other two.
The 1 crate that must keep the full `moveit-kinematics` is unchanged.

## 5. Decisive finding — this was already decided, twice, and not for a crate-layering reason

Steps 1-4 show the extraction is buildable. It should not be executed
anyway, because **whether `JointModelGroup` should hold a solver mapping at
all has already been decided, on grounds independent of the crate cycle,
twice, earlier in this same panel's history:**

- `PORTING-PLAN.md` §68.4 (p1-robotmodel round 8): the same question was
  raised — upstream's `ConstraintSamplerManager::selectDefaultSampler` finds
  a group's solver via `jmg->getGroupKinematics()`, this port has no such
  mapping. **결정: 매핑을 만들지 않는다. `IKConstraintSampler`가 solver를
  인자로 받는다.** Reasoning given: `moveit-kinematics::KINEMATICS_SOLVERS`
  is D4's compile-time *algorithm* registry; per-group *default solver
  assignment* is upstream's runtime configuration layer
  (`kinematics.yaml`/`robot_model_loader`), the exact plugin-dispatch-by-name
  category D4 already excludes wholesale. Planting `group_kinematics_` onto
  `JointModelGroup` would reopen that excluded layer through a side door.
- `PORTING-PLAN.md` §77.1 (later round, porting `default_constraint_samplers.cpp`
  line-for-line): reaffirms it — "D4 결정대로 solver는 인자로 받고... 의존
  간선이 생겼고 `check-dep-direction.sh`는 통과한다" — recording that the
  `moveit-constraints -> moveit-kinematics` edge this already requires was
  checked against exactly this script and passes.
- D4 itself (`PORTING-PLAN.md` §4.4, decided 2026-08-03) lists
  `KinematicsSolver` explicitly as one of the traits following the
  compile-time-registry pattern (alongside `CollisionDetectorAllocator`,
  `PlannerManager`, etc.) — a caller picks and constructs a concrete
  implementation; there is no automatic "the default solver for group X"
  resolution anywhere in this design, by the same decision that already
  excluded pluginlib's runtime `.so` lookup.

So the crate-cycle obstacle (§1-3 above) is real but not the actual cause.
Even with `moveit-kinematics-base` extracted and `JointModelGroup` capable
of holding `Arc<dyn KinematicsSolver>`, nothing would populate that field
automatically — upstream's own mechanism for doing so
(`RobotModel::setKinematicsAllocators`, populated from ROS parameter-server
config at `robot_model_loader` construction time) is itself the runtime
configuration layer D4 excludes. Extracting the trait would let a caller
*store* a solver on a group; it would not manufacture one from nothing,
which is what "no caller has anything to give" actually names.

**This measurement does not support the extraction. Not executed.**

## What would actually close the recurring UNFIXED

Not a `moveit-model`/`moveit-kinematics` restructuring — a caller-side
wiring gap, entirely within the planner crates that already depend on both
`moveit-kinematics` (for `KINEMATICS_SOLVERS`) and `moveit-constraints` (for
`select_default_sampler`). No production entry point in
`moveit-planners-sbp::registry::RrtConnectContext::solve` or
`moveit-planners-stomp`'s callers ever looks a solver up by name and
constructs one to pass in — every call site chooses `None` outright rather
than failing to find one. Threading an optional, explicitly-constructed
solver through `PlanningRequest` (sbp) so a caller who *does* want reliable
Cartesian-pose goal sampling can supply one is a small, scoped feature for a
future round, not attempted here since this round's brief scopes to the
extraction measurement only.

## Exclusion expiry

This is not an absence-grounded exclusion (§153.1's sense — "no dependency
yet"); the blocker is a standing decision, not a missing layer. It should
be revisited only if a future round decides D4's runtime-configuration
exclusion itself needs to be narrowed for kinematics specifically (i.e. D4
is amended or a D-numbered decision supersedes §68.4/§77.1's reasoning) —
short of that, re-raising "should `JointModelGroup` hold a solver" without
new grounds beyond the crate-cycle argument measured here would be re-asking
a question this panel's own history already answered twice.

## Scope note

Steps 1-4 above touch `moveit-model`/`moveit-kinematics` only by reading
them — no changes were made to either, per the brief's instruction to
commit the report before any execution, and per D12's "5. 위 4개를 doc/
아래 보고서로 먼저 커밋해라. 그 다음에 측정 결과가 추출을 지지하면
실행해라. 지지하지 않으면 무엇 때문인지 적고 멈춰라." The measurement does
not support execution; nothing further is executed this round.
