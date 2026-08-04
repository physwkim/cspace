# Claim audit: moveit-sampling

Per PORTING-PLAN.md §175. One row per item, appended as found, not
batched at report time. `evidence` is the upstream `file:line` actually
opened this round -- inference from the port is not evidence.

Upstream root for this crate: `/home/stevek/work/moveit2` (pinned
`e017c91ee12984393a28ba246075c65f69cde3bf`), the two
`multivariate_gaussian.hpp` files (stomp's and chomp's).

## §172 sweep (round 33), upstream-first per `0ad8c67`

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `moveit2 moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp`, `moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.{h,hpp}` | No float-derived `int`/`unsigned`/`size_t` narrowing anywhere in either upstream `multivariate_gaussian` header this crate ports (nor its stray `.h` twin, checked though not cited) | CONFIRMED, 0 hits | Full-file grep of all three files, read in this tree; zero matches for `static_cast<int-family>`, C-style int-family casts, or float-initialized int-family declarations | (none) |
| `crates/moveit-sampling/src/*.rs` (port-side anchor: `as i8..u128/usize` receiving `f64`) | Zero occurrences of the anchor pattern anywhere in this crate | CONFIRMED, 0 hits (run, not skipped) | Read in this tree only | (none) |

## §167.6 bare-directory-citation sweep (this round)

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| Every `Ported from` header in this crate (`lib.rs:5`, `multivariate_gaussian.rs:5`) | Neither cites a bare package/directory line with no filenames indented beneath it -- both list the two `multivariate_gaussian.hpp` files explicitly | CONFIRMED, 0 hits of the shape the parser now closes | Read both headers in full in this tree; `tools/ci/verify-upstream-license-provenance.sh` also run over the whole workspace this round: `checked 334 upstream file(s) cited by 242 tracked source file(s)`, 0 findings | (none) |

## §194 port-only API sweep (this round): `moveit-sampling`

This crate's share of the cross-crate sweep triggered by `a682f63`
(full rationale in `moveit-stomp-core`'s own claim-audit doc). This
crate has exactly one public type.

| API | port-only? | upstream state newly reachable | invariant at risk? |
|---|---|---|---|
| `MultivariateGaussian::new(mean, covariance)` | no -- direct port of `stomp_moveit::math::MultivariateGaussian`/`chomp::MultivariateGaussian`'s constructor, with a stated deviation (see this crate's own "Deviation: construction can fail") | none -- the deviation moves in the *safe* direction: upstream's constructor always succeeds and silently produces `NaN`-sampling state for a non-positive-definite `covariance`; this port's `Option`-returning constructor makes that state *unconstructable* instead of opening anything new | no -- the opposite pattern from `with_cancel_handle`: this narrows what was reachable upstream |
| `size()`, `sample_with_covariance`, `sample_without_covariance` | no -- upstream-faithful ported methods (the latter two are `sample(output, true)`/`sample(output, false)` split into two names, see this crate's own "Deviation: two named methods" doc) over already-validated state | none | no |

**Conclusion:** no port-only API exists in this crate at all --
every public item is a direct port of an upstream counterpart, one
with a deviation that removes reachable bad state rather than adding
reachable state. No finding.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-sampling/src/multivariate_gaussian.rs`, every `pub fn`/`pub struct` | No port-only API exists in this crate; the one constructor's deviation narrows reachable state, it does not expand it | CONFIRMED, 4 methods on 1 type enumerated and classified above | Read every public item in this tree, cross-referenced against both upstream `multivariate_gaussian.hpp` files | (pending, see report) |
