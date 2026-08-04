# Claim audit: moveit-stomp-core

Per PORTING-PLAN.md §175. One row per item, appended as found, not
batched at report time. `evidence` is the upstream `file:line` actually
opened this round -- inference from the port is not evidence.

Upstream root for this crate: `/home/stevek/work/stomp` (pinned
`b1a87c80f7338caae25a5c689b876da15492aa75`, `ros-industrial/stomp`).

## §172 sweep (round 33), upstream-first per `0ad8c67`

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `/home/stevek/work/stomp/src/utils.cpp:39` (`(int)order`) | `order` is `DerivativeOrder`-equivalent (an enum), cast to `int` for `pow(dt, (int)order)` -- integer/enum-derived, not real-valued | CONFIRMED distinct | `/home/stevek/work/stomp/src/utils.cpp:39` -- `double multiplier = 1.0 / pow(dt, (int)order);` | (none) |
| `/home/stevek/work/stomp/{include/stomp/*.h,src/*.cpp}` full sweep | No float-derived `int`/`unsigned`/`size_t`/`long` declaration or `static_cast` anywhere else in this crate's upstream | CONFIRMED, 0 additional hits | Full-file grep of `stomp.h`, `task.h`, `utils.h`, `stomp.cpp`, `utils.cpp`; all `int`/`size_t`/`unsigned` declarations found (`stomp.cpp:117-119,158,231,303,435-480`; `utils.cpp:44,66-67,113-115`) are `.rows()`/`.cols()`/`.size()`/config-count-derived | (none) |
| `crates/moveit-stomp-core/src/{stomp,utils}.rs` (port-side anchor: `as i8..u128/usize` receiving `f64`) | All 12 hits narrow an enum (`DerivativeOrder`), a loop/index variable, or an integer count -- none are real-valued | CONFIRMED distinct, 12 sites: `stomp.rs:550,792,817,878`; `utils.rs:330-332,339-340,421-422,424,543,555` | Read in this tree only | (none) |

## §172 row 3 (round 33) re-verified with the full anchor enumeration, per review

The round-33 row above asserts "12 hits, none real-valued" without
showing the enumeration. Re-run here with the anchor regex on screen,
because the count itself was off (16 casts across 14 lines, not 12 --
the underlying "none float-derived" conclusion is unchanged, but it
was not actually a 12-site claim):

**Anchor:** `` as (i8|i16|i32|i64|i128|u8|u16|u32|u64|u128|usize|isize)\b `` (`rg`, `crates/moveit-stomp-core/src/`)

**Sites** (16 casts, 14 lines):

| site | expression | source | classification |
|---|---|---|---|
| `utils.rs:330` | `order as i32` | `DerivativeOrder` enum param | enum, not float |
| `utils.rs:331` | `(FINITE_DIFF_RULE_LENGTH / 2) as isize` | `FINITE_DIFF_RULE_LENGTH: usize = 7` (`utils.rs:253`) | integer constant, not float |
| `utils.rs:332` | `num_time_steps as isize` | `num_time_steps: usize` fn param | integer param, not float |
| `utils.rs:339` | `i as usize` | `i: isize` loop var (`0..n`, `n: isize`) | integer loop index, not float |
| `utils.rs:339` | `index as usize` | `index = i + j` (`isize` arithmetic) | integer arithmetic, not float |
| `utils.rs:340` | `order as usize` | `DerivativeOrder` enum param | enum, not float |
| `utils.rs:340` | `(j + half) as usize` | `j`, `half: isize` loop vars | integer arithmetic, not float |
| `utils.rs:421` | `order as usize` | `DerivativeOrder` enum param | enum, not float |
| `utils.rs:422` | `order as usize` | `DerivativeOrder` enum param | enum, not float |
| `utils.rs:424` | `order as i32` | `DerivativeOrder` enum param | enum, not float |
| `utils.rs:543` | `DerivativeOrder::Velocity as usize` | enum literal (test) | enum, not float |
| `utils.rs:555` | `DerivativeOrder::Acceleration as usize` | enum literal (test) | enum, not float |
| `stomp.rs:550` | `self.current_iteration as usize` | `current_iteration: i32` field | integer field, not float |
| `stomp.rs:792` | `r as i32` | `r: usize` loop var (`0..num_rollouts`) | integer loop index, not float |
| `stomp.rs:817` | `r as i32` | `r: usize` loop var | integer loop index, not float |
| `stomp.rs:878` | `r as i32` | `r: usize` loop var | integer loop index, not float |

**Same defect at:** none -- zero of the 16 casts narrow a value derived
from an `f64` computation.

**Distinct, skip:** all 16 -- every source is an enum discriminant, an
already-integer field/param, or an integer loop index. None of the
`kernel_bounds`-style divergence applies here at all: that divergence
is specifically float-to-int narrowing (Rust's `as` saturates,
C++'s `static_cast` is UB on overflow); every cast in this crate is
int-to-int or enum-to-int, where both languages truncate identically
and no divergence class exists to test a boundary against.

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-stomp-core/src/{stomp,utils}.rs` full anchor re-enumeration | 16 casts (not 12), 0 float-derived; no boundary test is owed because int-to-int/enum-to-int narrowing has no Rust/C++ divergence class to begin with | CONFIRMED distinct, 16/16 sites enumerated and classified above | `rg` anchor search + type of every cast's source read in this tree | `3a0e278` |

## §167.6 bare-directory-citation sweep (this round)

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| Every `Ported from` header in this crate (`lib.rs:5`, `task.rs:5`, `stomp.rs:5`, `utils.rs:5`) | None cite a bare package/directory line with no filenames indented beneath it -- every citation lists explicit `include/stomp/*.h`/`src/*.cpp` files | CONFIRMED, 0 hits of the shape the parser now closes | Read all four headers in full in this tree; `tools/ci/verify-upstream-license-provenance.sh` also run over the whole workspace this round: `checked 334 upstream file(s) cited by 242 tracked source file(s)`, 0 findings | (none) |
