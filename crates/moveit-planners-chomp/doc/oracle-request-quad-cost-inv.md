# Oracle request: `ChompCost::getQuadraticCostInverse()` decomposition-family check

Not an implementation — this is a request document for the human
orchestrator (`tools/moveit-oracle/` is not this worker's file). Filed per
round 17's Item 2, closing the gap round 16 left open in
`crates/moveit-planners-chomp/src/cost.rs`'s module doc.

## The gap

`chomp::ChompCost::getQuadraticCostInverse()` returns `quad_cost_.inverse()`
(`Eigen::MatrixXd::inverse()`). Because `Eigen::MatrixXd`'s row count is
`Eigen::Dynamic` at compile time regardless of its actual runtime size,
`Eigen/src/LU/InverseImpl.h`'s dispatch always routes this call through
`PartialPivLU`, never through the closed-form small-matrix path that exists
only for compile-time-fixed sizes (`Matrix4d` and smaller).

`nalgebra::DMatrix::try_inverse()` (this port's equivalent, in
`ChompCost::new`, `crates/moveit-planners-chomp/src/cost.rs:166-169`)
dispatches on the matrix's *runtime* size instead
(`nalgebra-0.35.0/src/linalg/inverse.rs`): a closed-form cofactor/Cramer's
expansion for runtime size 1-4, `lu::try_invert_to` (partial row pivoting,
same strategy as `PartialPivLU`) for runtime size >= 5.

So for `num_vars_free >= 5` both implementations are the same algorithm
family — floating-point rounding is the only expected divergence. For
`num_vars_free` in `1..=4` (`num_vars_all` in `13..=16`, a reachable range
for a short CHOMP trajectory), Eigen still uses `PartialPivLU` while
nalgebra uses a genuinely different algorithm (closed-form cofactor
expansion vs. partial-pivoted LU elimination). Two different, individually
correct algorithms for the same linear system are not guaranteed to produce
bit-identical floating-point results — only residual-close ones. Round 16's
test (`quad_cost_inv_stays_a_true_inverse_across_both_algorithm_branches`)
confirmed the nalgebra result is a numerically sound inverse in both
branches (residual `0.0` and `1.7763568394002505e-15` respectively), but
that is not the same claim as "matches Eigen's actual output" — only an
oracle running real Eigen against real upstream `ChompCost` can answer that.

## Upstream symbol: public, already known

Confirmed directly against `chomp_cost.hpp:59` (re-read this round, not
assumed carried over from round 16):

```cpp
public:
  const Eigen::MatrixXd& getQuadraticCostInverse() const;
```

`public`, no ROS type in the signature, callable given only a constructed
`ChompCost`. `ChompCost`'s constructor (`chomp_cost.hpp:52`) is
`ChompCost(const ChompTrajectory& trajectory, int joint_number, const
std::vector<double>& derivative_costs, double ridge_factor = 0.0)` — also
public.

## What the constructor actually reads from `trajectory`

Read in full in round 16 (`chomp_cost.cpp`'s constructor body): only
`trajectory.getNumPoints()` and `trajectory.getDiscretization()` are ever
read. `joint_number` is unused (round 16 already confirmed and dropped it
from this port's `ChompCost::new`). No robot model, no group, no actual
joint values are read at all — `ChompCost`'s matrices are a pure function
of `(num_points, discretization, derivative_costs, ridge_factor)`. This
means the oracle harness does not need a real robot fixture to answer this
request; a `ChompTrajectory` built with any valid `(robot_model, num_points,
discretization, group_name)` — the actual robot/group are irrelevant to the
result — is sufficient, or the harness may skip `ChompTrajectory`
construction entirely and replicate `ChompCost`'s constructor body directly
against the four scalar/vector inputs if that is easier to wire into the
oracle's existing op-authoring convention. That packaging choice is the
oracle owner's, not dictated here.

## Request JSON shape

Per case:

```json
{
  "num_points": 14,
  "discretization": 0.1,
  "derivative_costs": [0.0, 1.0, 0.0],
  "ridge_factor": 1e-6
}
```

- `num_points` (int): drives `num_vars_all = num_points`,
  `num_vars_free = num_points - 2*(DIFF_RULE_LENGTH-1) = num_points - 12`
  (`DIFF_RULE_LENGTH = 7`, confirmed in `chomp_utils.hpp`, already ported as
  [`crate::utils::DIFF_RULE_LENGTH`]).
- `discretization` (f64): multiplies each derivative cost's contribution,
  compounding per stencil row (`multiplier *= discretization` once per
  `derivative_costs` entry, in order) — must be echoed exactly, not just a
  placeholder, since it changes `quad_cost_`'s actual values.
- `derivative_costs` (array of f64, 1-3 entries, indexing
  `DIFF_RULES[0..]` = velocity/acceleration/jerk in that order): use
  `[0.0, 1.0, 0.0]` (acceleration only) to match this port's existing test
  fixtures (`crates/moveit-planners-chomp/src/optimizer.rs`'s
  `joint_costs` helper) unless the orchestrator prefers a different vector;
  either way, use the **same vector at every `num_points` case** so the
  only varying input across cases is the algorithm-branch boundary.
- `ridge_factor` (f64): non-zero, so `quad_cost_` stays invertible even at
  `num_vars_free == 1` (a `1x1` matrix is singular only if its one entry is
  exactly `0.0`).

## Response shape

The **full inverse matrix**, not a scalar or a residual. A residual check
is exactly what round 16 already did and is not evidence of bit-for-bit
agreement — the point of this request is to compare Eigen's actual
`PartialPivLU`-derived entries against nalgebra's cofactor-derived entries
element-by-element. Response per case:

```json
{
  "num_vars_free": 2,
  "quad_cost_inverse": [[..., ...], [..., ...]]
}
```

Row-major, full `f64` precision (no truncation — this comparison is
specifically about float-level agreement, so any serialization that loses
precision defeats the request). `num_vars_free` echoed back for a
self-checking response, since it's derivable from `num_points` but a
mismatch there would itself be informative (would mean `DIFF_RULE_LENGTH`
or the boundary formula diverged from what this port assumes).

## Verified (round 18): the branch boundary claim above is correct

Re-checked directly against the vendored source for the exact `nalgebra`
version this workspace pins (`Cargo.lock`: `nalgebra 0.35.0`), not assumed
from the crate's public docs:
`~/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/nalgebra-0.35.0/src/linalg/inverse.rs`,
`SquareMatrix::try_inverse_mut`'s `match dim { ... }`:

- `0` — trivial `true` (an empty matrix "inverts" to itself; not a case in
  this request, since `num_vars_free` starts at 1 below).
- `1` — closed form, `1 / determinant`.
- `2` — closed form, 2×2 adjugate/determinant.
- `3` — closed form, 3×3 cofactor expansion.
- `4` — closed form, `do_inverse4` (a loop-unrolled MESA-derived cofactor
  expansion, a distinct code path from cases 1-3 but still not LU).
- `_` (i.e. `>= 5`) — `lu::try_invert_to`, partial-pivoted LU.

So the boundary is exactly `1..=4` closed-form vs. `>=5` LU, confirming
the table below needed no correction.

## Cases needed: 5, one per side of the algorithm-branch boundary

| `num_points` | `num_vars_free` | Algorithm (nalgebra side) |
|---:|---:|---|
| 13 | 1 | cofactor (1×1, trivially `1/x`) |
| 14 | 2 | cofactor (2×2 closed form) |
| 15 | 3 | cofactor (3×3 closed form) |
| 16 | 4 | cofactor (4×4 closed form) |
| 20 | 8 | `lu::try_invert_to` (partial-pivoted LU) |

The last row reuses `num_points = 20` from this port's own existing test
fixtures (`cost.rs`'s `quad_cost_inv_stays_a_true_inverse_across_both_
algorithm_branches`), so its result also re-validates that test's residual
claim against the real value, not just a second algorithm-family boundary.

## How this port will use the response

Once received, this port adds a test comparing `ChompCost::new(...)
.quadratic_cost_inverse()` against each case's `quad_cost_inverse` entries.
If the `num_vars_free <= 4` cases match to a small ULP-scale tolerance
(measured, not guessed, per this port's own established convention — see
`crates/moveit-planners-chomp/src/cost.rs`'s `RESIDUAL_TOL` note), the
decomposition-family gap closes as "different algorithm, same answer to
float precision" and the module doc's open-gap note is retired. If they
diverge beyond float-rounding levels, that is a real, reportable
parity finding, not a rounding footnote — and this port would then need to
decide whether `nalgebra`'s cofactor path needs to be bypassed (e.g. forcing
`nalgebra`'s own LU path even for small matrices) to match upstream, which
is a decision to bring back to the orchestrator, not to make unilaterally.
