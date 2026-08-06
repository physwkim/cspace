# Assertion-discrimination ledger — planner configurations (`moveit-planning`)

The two sites the sweep's scanner emits for `moveit_planning::planner`'s
configuration-selection tests, added with
`moveit_planning::configuration_for` (PORTING-PLAN.md §285 — the round that
made a stored `/set_planner_params` configuration reach the planner that
plans under it). Both are `is_none`; the module's other new assertions are
`assert_eq!` against exact values, which the scanner's kinds do not cover.

A separate file rather than rows appended to
`doc/assertion-discrimination-ledger-d8-planner.md`: that ledger is the D8
round's record of the `moveit-planning`/`moveit-planner-registry`/sbp seam,
and `discover_ledgers` globs rather than hardcodes, so a new file is
accounted for without touching it.

Verdict legend (same as the other ledgers): **discriminating** (proven to
distinguish ≥2 sibling outcomes), **single-branch** (exactly one cause can
reach this assertion — nothing to name), **joint-collapse** (≥2 real sibling
branches fire together and that is correct), **not-this-family** (excluded by
census §9).

## Method

Every row's evidence is a mutation applied to this worktree, run with
`cargo nextest run -p moveit-planning --no-fail-fast`, then reverted from a
pristine copy taken before the first bite; the revert is confirmed by `rg`
for each bite's own marker returning nothing and by the crate's full suite
(64 tests, 64 passed) running green again.

Each bite is reported by the **panic line**, not only by the failing test
name: both sites live in tests that carry a second assertion below them, and
`assert!` aborts at the first failure, so a test name alone cannot say which
of the two fired.

### The line numbers the bites report

`K2` deletes three lines from `configuration_for` and adds one, and `K3`
adds three, so each bite's reported panic line is offset from the site in the
committed file — `-2` for `K2`, `+3` for `K3`. The rows below cite the
committed file; the offsets are stated here so the two numbers cannot be read
as a disagreement.

## The three bites

- **`K1`** — `configuration_for`'s fallback becomes
  `configs.get(group_name).or_else(|| configs.get(planner_id))`, i.e. a
  configuration stored under a bare planner name governs a query that names
  a group. This is not a hypothetical: `/set_planner_params` with an empty
  `group` writes exactly that key
  (`ros/moveit-ros/src/planner_params.rs:296`), so the bite is the shape the
  port would take if the writer's key rule and the reader's disagreed.
- **`K2`** — the `planner_id` lookup is removed (`let _ = &key;` in place of
  the `configs.get(&key)` early return), leaving only the group fallback.
- **`K3`** — a total miss returns a fabricated default
  (`.or_else(|| Some(BITE.get_or_init(PlannerConfigurationSettings::default)))`)
  instead of `None`, i.e. every query is "configured" whether or not anything
  was stored.

## Sites (2)

| file:line | anchor | test fn | verdict | evidence |
|---|---|---|---|---|
| `crates/moveit-planning/src/planner.rs:442` | `assert!(configuration_for(&configs, "arm", "RRTConnect").is_none())` — a configuration keyed by planner name alone must not govern a query that names a group | `a_global_configuration_does_not_govern_a_grouped_query` | discriminating | Bite `K1`: 1 test fails, panicking at `crates/moveit-planning/src/planner.rs:442`; the other seven selection tests pass, so this site alone sees the collapsed key rule. Both directions against its own sibling: under `K2` this site **passes** and the `assert_eq!` two lines below it — the same entry *does* govern an ungrouped query — is what panics (reported at `444`, see the offset note). "The global entry is unreachable from a grouped query" and "the global entry is reachable at all" are therefore separately falsifiable, which is what the test claims. |
| `crates/moveit-planning/src/planner.rs:454` | `assert!(configuration_for(&PlannerConfigurationMap::new(), "arm", "RRTConnect").is_none())` — an unconfigured manager gets no configuration, which is what makes every default `resolve_planner(name, &PlannerConfigurationMap::new())` plan on its compiled-in fields | `an_empty_map_selects_nothing` | single-branch | On an empty map both lookups in `configuration_for` — `configs.get(&key)` and `configs.get(group_name)` — have an unsatisfiable condition, so exactly one cause reaches this `None` and there is nothing for it to name. Measured rather than assumed: `K1` and `K2` each leave this site **green** while failing a site above it, and `K3`, the one bite that does fail it (panicking at `457`, see the offset note), fails `crates/moveit-planning/src/planner.rs:442` in the same run. No mutation separates this site from that one, which is the definition of the verdict rather than a gap in the bite set. Kept despite that: it is the base case the overlay design rests on, and a bite that returns a fabricated `Some` on a total miss is a real design temptation — `K3` is that design, not a contrivance. |

## What no site here covers

That the selected configuration *changes a plan* is not assertable in this
crate: `moveit-planning` declares the `PlannerManager` trait and has no
concrete planner to run. It is covered one crate over, by
`moveit-planners-sbp`'s
`a_range_configuration_reaches_the_registry_planner_and_changes_the_plan`
and its two siblings, whose assertions are `assert_ne!`/`assert!(a > b)`/
`assert_eq!` and so fall outside this scanner's kinds; and end to end over
DDS by `ros/verify-planner-params-interop.sh`'s leg C, which is not
assertable from unit tests at all.
