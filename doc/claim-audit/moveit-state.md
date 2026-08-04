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

## §172 two-anchor narrowing sweep — negative result, logged for §153.1 expiry tracking

Anchor 1 (upstream, run first per §172): `static_cast<int|size_t|unsigned|
long|short>`, C-style `(int)`/`(size_t)`/`(unsigned)`/`(long)` casts, an
`int`/`size_t`/`unsigned`/`long`/`short` declaration whose RHS is a float
literal or a division, and `floor`/`ceil`/`round`/`sqrt`/`pow` near an
integer declaration — swept via `rg` against every upstream file this
crate's `Ported from` headers cite (`robot_state.{hpp,cpp}`,
`dynamics_solver.{hpp,cpp}`), each file present and read at the path
cited. All four sweep patterns: 0 hits across the whole set.

Anchor 2 (port side): `as (i8|i16|i32|i64|isize|u8|u16|u32|u64|usize)` in
`crates/moveit-state/src/`, enumerated on screen. 0 hits.

| where (upstream file swept, anchor 1) | claim | verdict | evidence |
|---|---|---|---|
| `robot_state/include/.../robot_state.hpp` | no int/size_t/unsigned/long/short decl or cast with a floating-point initializer | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_state/src/robot_state.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `dynamics_solver/include/.../dynamics_solver.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `dynamics_solver/src/dynamics_solver.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |

Anchor 2 hit, classified: none — `rg` returned 0 matches in
`crates/moveit-state/src/`, so there is nothing to enumerate this round.

Expires (§153.1): if a future round adds an int/size_t/unsigned/long/short
declaration with a floating-point initializer to any file above, adds a
new upstream file citing floating-point narrowing to this crate's cited
set, or adds an `as iNN/uNN/usize/isize` cast receiving an `f64` expression
to `crates/moveit-state/src/`, this table's `CONFIRMED (absent)` rows must
be re-swept, not assumed to still hold.
