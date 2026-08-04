# Claim audit — moveit-state

Type-b claim audit per PORTING-PLAN.md §175. No plugin/dispatch layer in this
crate's scope (variable storage, position setters, mimic propagation, bounds,
FK, inverse dynamics) — swept for the "upstream does X" claim shape, found
none resting on unverified reachability.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/state.rs:118` | `joint.first_variable_index()` is called upstream the way this port's `RobotModel` exposes it. | UNVERIFIABLE(not re-derived this round — this is a same-crate cross-reference to this port's own `RobotModel` API, not a claim about a separate upstream file's behavior; out of the type-b shape this audit targets) | — |  |
| `src/dynamics.rs:517` | This port reproduces a specific upstream indexing bug (see that comment for detail). | CONFIRMED (carried over from earlier-round fix, not re-opened this pass) | Already verified and committed in an earlier round of this session (dynamics_solver.cpp indexing bug port). Not re-derived again this pass; flagged here only so the claim-audit inventory records it was checked at some point rather than never. |  |

## Swept, no claim found needing verification

`src/state.rs:866`, `src/dynamics.rs:129` — describe this port's own call
ordering / proof strategy, not a separate upstream code path's reachability.
