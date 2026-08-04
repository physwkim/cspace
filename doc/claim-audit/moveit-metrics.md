# Claim audit — moveit-metrics

Prose-citation audit against upstream MoveIt2 pinned at `e017c91e`. Scope:
every citation in `crates/moveit-metrics/src/*.rs`. Same round/method as
`moveit-scene.md` — see that file for the full methodology note.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-metrics/src/lib.rs` (citation `kinematics_metrics.cpp:56-103`) | (not itemized individually by subagent — reported as part of the "fine" bucket for the response.rs/adapters/metrics/attached_body.rs batch) | CONFIRMED | subagent-reported only, not independently re-opened by me | |

## Summary

- 1 site total, this crate's only prose citation
- CONFIRMED (aggregate, not individually itemized): 1
- No EXPIRED findings in this crate

## §172 narrowing sweep (separate exercise, same round)

Not a citation audit, but recorded here since it also swept this crate
exhaustively and is worth keeping off the compaction-risk conversation
context per §175's same reasoning.

- Upstream-first direction: swept `moveit_core/kinematics_metrics/src/kinematics_metrics.cpp`
  (the only upstream file this crate ports) for `int`/`unsigned`/`size_t`/
  `std::size_t`/`long`/`uint32_t`/`int32_t` declarations or `static_cast<...>`
  narrowing an initializer that is itself a floating-point expression.
  4 hits, all `for (unsigned int i = 0; i < singular_values.rows(); ++i)` /
  `for (int i = 0; i < singular_values.rows(); ++i)` style loop counters —
  `.rows()` on an Eigen matrix returns an integer row count (`Eigen::Index`),
  not a float. **0 real narrowing sites** (all 4 hits are `distinct`: true
  integer loop counters).
- Port-side direction: `rg '\bas\s+(i8|i16|i32|i64|i128|isize|u8|u16|u32|u64|u128|usize)\b'`
  across `crates/moveit-metrics` (src + tests) — **0 hits**.
- Both directions swept, both zero, no fix needed.
