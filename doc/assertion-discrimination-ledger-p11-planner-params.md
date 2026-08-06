# Assertion-discrimination ledger — planner params (`ros/moveit-ros`)

The twelve sites the sweep's scanner emits for
`ros/moveit-ros/src/planner_params.rs`, the module added with the
`query_planner_interface` / `get_planner_params` / `set_planner_params` trio
(upstream's single `MoveGroupQueryPlannersService`). All twelve are the
`is_empty`/`contains` kinds; the module's other assertions are `assert_eq!`
against exact values, which the scanner's kinds do not cover.

A separate file rather than rows appended to
`doc/assertion-discrimination-ledger-p9-ros.md`: that ledger is another
panel's live fence this round, and `discover_ledgers` globs rather than
hardcodes, so a new file is accounted for without touching it.

Verdict legend (same as the other ledgers): **discriminating** (proven to
distinguish ≥2 sibling outcomes), **single-branch** (exactly one construction
site reaches this assertion — nothing to discriminate), **joint-collapse**
(≥2 real sibling sites are asserted at once here and it cannot say which was
wrong), **not-this-family** (excluded by census §9).

## Method

Every row's evidence is a mutation applied to this worktree, built and run
inside the ROS image (`moveit-rs/ros-dev:latest`, the only place this crate
compiles — r2r needs a local ROS 2 install), then reverted, with the revert
confirmed by an empty `git status --porcelain`. The runner restores the
pristine file from memory in a `finally`, so a mutation cannot outlive its
own run.

`cargo test -p moveit-ros --no-fail-fast`, not `cargo nextest run`: the image
has no `cargo-nextest` (`error: no such command: nextest`), so the workspace's
usual runner is unavailable here. Baseline 217 passed, 0 failed.

Each bite is reported by the **panic line**, not only by the failing test
name — several of these sites share a test function and `assert!` aborts at
the first failure, so a test name alone cannot say which of them fired.

Two mutations had to be rewritten because this crate sets
`warnings = "deny"`: dropping a parameter's last use makes the crate fail to
build (`unused variable`), which is a build error and not evidence about an
assertion. Both were replaced with a version that still reads the variable —
`L4` by `L4'`, `L9` by `L9'` — and only the rewritten form is cited below.

### One mutation that is not evidence

`L5` replaced `configs.get(&req.planner_config)` with
`configs.values().find(|s| s.name.ends_with(&req.planner_config))`, intending
to make a global read match a group-keyed entry. It compiled, ran, and every
test passed. That is **not** a blind spot in `:551`: the group key is
`arm[RRTConnect]`, which ends in `]`, so `ends_with("RRTConnect")` is false
and the mutated branch never selected anything — the bite was mis-designed
and never exercised the path it claimed to. Recorded rather than dropped,
because a surviving mutation that is silently deleted reads afterwards like a
mutation that was never tried. `:551`'s evidence is `L10`, which does select
the entry.

### A note on the line numbers `L6` reports

`L6` inserts three lines into `apply_set`, above the test module, so the
panic lines it prints are each three higher than the site they belong to
(`666`/`677`/`716` reported, for the sites `:663`/`:674`/`:713`). The rows
below cite the site in the committed file; the shift is an artifact of that
one bite adding lines, and is stated here so the two numbers cannot be read
as a disagreement.

## Sites (12)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `ros/moveit-ros/src/planner_params.rs:479` | `assert!(!registered_interface_names().is_empty(), ...)` — the `distributed_slice` reached this binary at all | `the_linked_registry_is_not_empty` | discriminating | Bite `L1` appends `.take(0)` to the registry read in `registered_interface_names`, so the slice links but yields nothing: 1 test fails, panicking at `:479`. Sibling outcome separated from `:483` by `L2`, which leaves this site passing. |
| `ros/moveit-ros/src/planner_params.rs:483` | `assert!(registered_interface_names().contains(&"rrt_connect"))` — and the registration that arrived is the expected one | `the_linked_registry_is_not_empty` | discriminating | Bite `L2` rewrites the read as `.map(\|_\| "rrt_connect_BITE")`: the list is still non-empty so `:479` passes and execution reaches this site, which panics at `:483`. A non-empty registry carrying the wrong name is exactly what `:479` alone cannot see. |
| `ros/moveit-ros/src/planner_params.rs:513` | `assert!(desc.planner_ids.is_empty())` — upstream's `getPlanningAlgorithms` base default, not a stub filled in here | `a_description_carries_the_name_an_empty_pipeline_id_and_no_planner_ids` | discriminating | Bite `L3` makes `query_response` emit `planner_ids: vec![(*name).to_string()]`: 1 test fails, panicking at `:513`. The three `assert_eq!`s above it (name, `pipeline_id`, len) all pass under this bite, so the site is not a restatement of them. |
| `ros/moveit-ros/src/planner_params.rs:518` | `assert!(query_response(&[]).planner_interfaces.is_empty())` — an empty registry answers an empty list rather than inventing one | `an_empty_registry_answers_an_empty_interface_list` | discriminating | Bite `L4'` makes `query_response` substitute `&["ghost"][..]` when its argument is empty: 1 test fails, panicking at `:518`. (`L4`, which replaced the argument outright, is not the evidence — it fails to build under `warnings = "deny"` with `unused variable: interface_names`, which says nothing about this assertion.) |
| `ros/moveit-ros/src/planner_params.rs:551` | `assert!(pairs(&get_response(..., &get_request("", "RRTConnect", ""))).is_empty())` — a global read must not see a group-specific write | `a_group_set_is_keyed_as_group_bracket_config` | discriminating | Bite `L10` replaces the default fetch with `configs.values().next()`, so any stored entry answers any request: this site panics at `:551` (and `:738` in another test). The three assertions above it — `contains_key("arm[RRTConnect]")`, `.group`, `.name` — all pass under the bite, so the key having the right *shape* is a separate claim from the global read not reaching it. |
| `ros/moveit-ros/src/planner_params.rs:663` | `assert!(configs.is_empty())` — a mismatched key/value pair leaves no half-written entry | `mismatched_key_and_value_counts_write_nothing` | discriminating | Bite `L6` moves the `entry(...).or_default()` above `apply_set`'s guards, so a rejected request still creates its entry. The preceding `assert!(!apply_set(...))` still passes — the return value is unchanged — and this site is the one that panics (reported at `666`, see the shift note). "Returned false" and "wrote nothing" are two claims, and this bite realises the case where only the first holds. |
| `ros/moveit-ros/src/planner_params.rs:674` | `assert!(configs.is_empty())` — a set for a pipeline this node does not serve writes nothing | `a_nonempty_pipeline_id_resolves_to_nothing_on_both_legs` | discriminating | Bite `L6`, same half-write: this site panics in its own test (reported at `677`). Not reachable by the `resolves_pipeline` bite, which changes the return value and so fails the `assert!(!apply_set(...))` above it first — that is why the half-write bite is the evidence here rather than a pipeline bite. |
| `ros/moveit-ros/src/planner_params.rs:683` | `assert!(pairs(&get_response(..., &get_request("ompl", ...))).is_empty())` — the read leg rejects a named pipeline too, even when the default pipeline holds that config | `a_nonempty_pipeline_id_resolves_to_nothing_on_both_legs` | discriminating | Bite `L8` drops `resolves_pipeline` from `get_response`'s guard (`if interface_count > 0 {`): 1 test fails, panicking at `:683`. `:674` passes under this bite — it is the write leg — so the two legs are independently falsifiable, which is what the test name claims. |
| `ros/moveit-ros/src/planner_params.rs:713` | `assert!(configs.is_empty())` — with no planner manager registered, a set writes nothing | `with_no_registered_interface_neither_leg_does_anything` | discriminating | Bite `L6`, same half-write: this site panics in its own test (reported at `716`). |
| `ros/moveit-ros/src/planner_params.rs:720` | `assert!(pairs(&get_response(&configs, 0, ...)).is_empty())` — and with none registered the read leg is empty even when the store is not | `with_no_registered_interface_neither_leg_does_anything` | discriminating | Bite `L9'` weakens the count test to `interface_count < 999`, always true for the values under test while still reading the variable: 1 test fails, panicking at `:720`. The store is deliberately non-empty by then, so the site distinguishes "no interface" from "nothing stored". (`L9`, which dropped the variable, is not the evidence — `unused variable: interface_count` is a build failure.) |
| `ros/moveit-ros/src/planner_params.rs:738` | `assert!(pairs(&get_response(..., &get_request("", "PRM", ""))).is_empty())` — an unknown `planner_config` reads empty rather than erroring or returning someone else's config | `an_unknown_planner_config_reads_empty_rather_than_failing` | discriminating | Bite `L10` (`configs.values().next()`): this site panics at `:738`. The stored entry is `RRTConnect` and the request names `PRM`, so what the bite returns is another key's configuration — the failure mode a client cannot tell from its own. |
| `ros/moveit-ros/src/planner_params.rs:762` | `assert!(response.params.descriptions.is_empty())` — `descriptions` is inert on both legs upstream | `get_returns_key_sorted_index_paired_arrays` | discriminating | Bite `L11` fills `descriptions` in `get_response`: 1 test fails, panicking at `:762`. The two `assert_eq!`s above it — key order and index pairing — pass under the bite, so this is a third independent claim about the same reply and not a restatement of either. |

## What no site here covers

Nothing in this file asserts that a stored configuration reaches a planner,
because nothing in the port hands it to one: upstream's `setParams` ends at
`setPlannerConfigurations` on the instance the pipeline plans with, and there
is no equivalent call here. That is a gap in the port, recorded in the
module's own doc and in PORTING-PLAN.md, not an assertion this ledger is
missing — there is no such behaviour yet to be coarse about.

The live-graph half of these services (that the three names are actually
advertised, and that a `set` survives the wire) is not assertable from unit
tests at all and is gated separately by
`ros/verify-planner-params-interop.sh`.
