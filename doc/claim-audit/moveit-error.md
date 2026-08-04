# Claim audit — moveit-error

Second of the two crates that had no owner (see `doc/crate-ownership.md`);
like `moveit-srdf.md`, never audited and never swept before this round.
Coordinator-owned now.

Upstream is moveit2 pinned at `e017c91e`, plus
`third_party/moveit_msgs/msg/MoveItErrorCodes.msg`. Every row was verified by
opening the upstream file in this round.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `src/lib.rs:9` header | the numeric values are wire-exact with `MoveItErrorCodes.msg` | CONFIRMED | `third_party/moveit_msgs/msg/MoveItErrorCodes.msg` holds 31 `int32 NAME = value` constants; extracted and diffed mechanically against the port's `UPSTREAM` table — 31 vs 31, no value in one and not the other, and every name matches after `UPPER_SNAKE` → `PascalCase` | |
| `src/lib.rs:276` (`Display`) | reproduces upstream `MoveItErrorCode::toString()` in `moveit_core/utils/src/moveit_error_code.cpp` | **EXPIRED — the citation names nothing that exists** | `moveit_core/utils/src/` contains only `lexical_casts.cpp`, `logger.cpp`, `message_checks.cpp`, `rclcpp_utils.cpp`, `robot_model_test_utils.cpp` — no `moveit_error_code.cpp`. There is no `MoveItErrorCode::toString()` member; the only `toString` in that directory is `moveit::core::toString(double)`, a float formatter. The real symbol is the free function `errorCodeToString` at `moveit_core/utils/include/moveit/utils/moveit_error_code.hpp:82` | `79bd7e8` |
| `src/lib.rs:276` (`Display`), corrected | the 31 strings match `errorCodeToString` | CONFIRMED | `moveit_error_code.hpp:86-147` read in full; all 31 `return std::string("...")` compared one by one against the port's match arms | `ade1b04` |
| `src/lib.rs:286` (`Display`, `Unknown` arm) | — (no claim was made) | **gap, now documented** | upstream's `switch` has no `default`; anything outside the 31 reaches `moveit_error_code.hpp:150`, `"Unrecognized MoveItErrorCode. This should never happen!"`, discarding the value. The port renders `UNKNOWN(v)`. A real deviation that no doc comment mentioned | `dcf08a1` |
| `src/lib.rs:387` | "-20 is a gap in the upstream msg" | CONFIRMED, and incomplete | the `.msg` has no `-20`; it also has no `-8`, `-9`. The comment names one of three gaps. Not wrong, and the test's chosen values are valid | |

## What the audit found that the tests did not

The wire-exactness claim was guarded (`discriminants_match_upstream_msg`,
`round_trips_through_i32`) and the guard bites — mutating one `as_i32` arm
fails two tests, dropping one `from_i32` arm fails one.

The `Display` claim was **not** guarded. Changing `NO_IK_SOLUTION` to
`NO_IK_SOLN` left all four tests green. That is why the wrong citation
survived: nothing ever compared the strings to anything, so nothing had cause
to open the file the citation named and discover it does not exist.

`ade1b04` closes it with `display_matches_upstream_error_code_to_string`,
re-probed after landing: the same spelling mutation now fails, and dropping a
row from the new table fails the cross-table coverage assertion rather than
silently shrinking what is checked.

## §172 narrowing sweep (first ever run on this crate)

- Port-side anchor: `as (i8|i16|i32|i64|isize|u8|u16|u32|u64|usize)` across
  `crates/moveit-error` — **0 hits**. The crate has no casts; `as_i32` and
  `from_i32` are exhaustive `match` arms over integer literals.
- Upstream-first anchor: `exceptions.hpp`, `moveit_error_code.hpp` and
  `MoveItErrorCodes.msg` contain no floating-point type at all, so no
  float → integer narrowing exists to port. `.msg` values are `int32`
  literals.
- Both anchors run, both zero.
