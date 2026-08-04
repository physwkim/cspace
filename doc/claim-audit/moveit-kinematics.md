# Claim audit — moveit-kinematics

Type-b claim audit per PORTING-PLAN.md §175. This crate has no plugin/adapter
dispatch layer the way moveit-planners-pilz does (see
[`doc/claim-audit/moveit-planners-pilz.md`](../claim-audit/moveit-planners-pilz.md)
for the crate where that risk actually materialized), so this pass is a
grep-plus-read sweep rather than a full per-claim upstream re-derivation.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/lib.rs:33-42` | `kdl_kinematics_plugin` builds a `KDL::Chain` from the URDF and gets its Jacobian/FK from KDL's own solvers (`ChainJntToJacSolver`, `ChainFkSolverPos_recursive`); this port replaces both with `moveit_state::Posed::jacobian`/`global_link_transform`. | CONFIRMED | `kdl_kinematics_plugin.cpp` — `KDLKinematicsPlugin::initialize` builds `kdl_chain_` via `tree.getChain(...)`; `getPositionFK`/velocity-IK paths construct `ChainFkSolverPos_recursive`/`ChainJntToJacSolver` against it. Matches the claim; not a dispatch-reachability claim (no earlier branch could redirect around this), so lower audit priority than the pilz pattern, but checked anyway since it's the crate's central architectural claim. |  |
| `src/params.rs:35-36` | `joint_weights`'s default-then-override behavior matches `KDLKinematicsPlugin::getJointWeights`. | CONFIRMED (spot check only, not full re-derivation this round) | `kdl_kinematics_plugin.cpp`'s `getJointWeights` — default `1.0` per joint, overridden by name from parsed weights. Consistent with the doc claim. |  |

## Swept, no claim found needing verification

`src/chain.rs:518`, `src/velocity.rs:95`, `src/lma.rs:4` — self-referential or
"see this method's own doc comment" pointers, not claims about a separate
upstream code path's reachability.
