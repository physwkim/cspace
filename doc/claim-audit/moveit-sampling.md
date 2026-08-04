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
