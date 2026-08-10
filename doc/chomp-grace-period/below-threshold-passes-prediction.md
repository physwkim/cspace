# Prediction: `below_threshold_passes`, before the measurement exists

Written before `optimize_benchmark_chomp.rs` (owned by `residual-ci-wired` this
round) switches to `solve_with_trace` and the number lands. Grounded against
`crates/cspace-planners/src/chomp/optimizer.rs`'s live `optimize()` body, not
against the port's docs about it.

Benchmark parameters are `ChompParameters::default()` with only
`planning_time_limit` overridden (`optimize_benchmark_chomp.rs:611-614`), so
`filter_mode: false`, `collision_threshold: 0.07`,
`max_iterations_after_collision_free: 5` all apply
(`parameters.rs:106-126`) — the collision-threshold branch is live, not
categorically disabled, for this population.

## The two branches and the latch (`optimizer.rs:1844-1969`)

`should_break_out` is declared once, **outside** the `while` loop
(`:1861`), not reset per pass. Once any pass sets it, it stays `true` for
every later pass of that call. Two independent `if`s can each set it on the
same pass, in this order:

- mesh check (`:1934-1942`, every 10th iteration): `num_collision_free_iterations = 0`.
- threshold check (`:1944-1951`, `c_cost < collision_threshold`):
  `num_collision_free_iterations = max_iterations_after_collision_free` (5),
  and increments `below_threshold_passes`.

If both fire on the same pass, the threshold write happens second and wins
— `num_collision_free_iterations` ends at 5, not 0. The break gate
(`:1958-1969`) only exits once `collision_free_iteration` (incremented on
every pass from the first trigger onward, since the latch never resets)
exceeds `num_collision_free_iterations`. A pass can retrigger the threshold
condition later and reset the grace budget back to 5 from that point,
extending the window further — `below_threshold_passes` counts *retriggers*,
not "how many grace passes elapsed."

**Consequence: `evaluations == 1` (the trivial/seed-already-good exit)
structurally requires `below_threshold_passes == 0` for that record.** An
immediate break needs `num_collision_free_iterations == 0` at the gate,
which only the mesh branch produces alone; if the threshold condition had
also been true on that same pass, its write would have overwritten the 0
with 5 and the loop could not have exited at one evaluation. This is a
necessity of the break-out arithmetic, not something the run has to
demonstrate.

## Prediction 1: `below_threshold_passes == 0` across the whole population

**Establishes:** on this exact population (this obstacle set, this seed
generator, `collision_threshold = 0.07`, `filter_mode = false`), the
collision-threshold branch was never taken on any measured problem's
optimizer call.

**Does not establish:** that the branch is unreachable, dead, or
non-functional in general, and — this is the sharp part — it establishes
**nothing at all** about the ~69-87% of solved problems that exit at
`evaluations == 1`. Those records are guaranteed to show
`below_threshold_passes == 0` by the arithmetic above, regardless of
whether the branch works. Counting them into a "the branch never fires"
verdict is circular: it fires (or doesn't) precisely on problems that run
past the seed evaluation, and the seed-solve majority never gets the
chance either way.

**Correct denominator:** solved problems with `evaluations > 1` — problems
whose optimizer ran at least one pass beyond the initial seed evaluation.
A `below_threshold_passes == 0` result restricted to *that* subpopulation
is the first result that says anything about reachability on this
workload; a `== 0` result over the full solved set does not.

## Prediction 2: `below_threshold_passes > 0` on some problems

**Establishes:** on at least one pass, `c_cost` (`get_collision_cost()` —
the sphere/distance-field term, not the mesh check) dropped below 0.07,
forcing `is_collision_free = true` and extending the loop for a grace
window with no further requirement that mesh safety hold.

**Does not establish that a regressed trajectory was returned.** Two gaps
sit between "grace period entered" and "regressed trajectory returned":

1. **Acceptance uses total cost, not `c_cost`.** `best_group_trajectory`
   is only overwritten when `cost < progress.best.total()`
   (`:1898-1904`), where `cost` is smoothness + collision combined. A pass
   inside the grace window does not need to be an accept, and an accept
   does not need to happen inside the grace window. The trajectory finally
   copied out at `:1987-1990` is whichever pass had the best *total* cost
   across the whole call — not necessarily anything evaluated during a
   below-threshold pass at all.
2. **Nothing re-verifies mesh safety before returning.** Upstream's own
   detector for exactly this exists and is inert:
   `chomp_optimizer.cpp:488-491`, a commented-out `checkCurrentIterValidity()`
   re-check gated on `collision_free_iteration_ > num_collision_free_iterations_`
   (the grace period's expiry), guarding a commented-out
   `ROS_WARN("Apparently regressed")`. The port does not carry even a dead
   copy of this forward — `optimizer.rs:1960-1962` documents folding both
   upstream break arms into one, explicitly because the second "also
   guards dead, commented-out logging." Neither side runs this check today.

**What would close the gap:** re-run the mesh-to-mesh predicate
(`mesh_to_mesh_collision_free`, the same closure `optimize()` already
takes) on `best_group_trajectory` at the moment the grace period expires
— i.e. resurrect `chomp_optimizer.cpp:488-491` as a live check, recording
its result as a new trace field, rather than a warning nobody reads. Short
of that, the only thing derivable from ndjson fields already planned this
round is a correlation, not a proof: `below_threshold_passes > 0` together
with `condition2_valid == false` (the harness's own independent post-loop
mesh check on the returned trajectory) is circumstantial evidence, and
`accepted > 0` narrows it further by showing an improving accept actually
occurred somewhere in the call — but none of that localizes the accept to
inside the grace window, so it cannot stand in for the resurrected check.
