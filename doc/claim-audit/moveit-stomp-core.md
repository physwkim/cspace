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

## §167.6 bare-directory-citation sweep (this round)

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| Every `Ported from` header in this crate (`lib.rs:5`, `task.rs:5`, `stomp.rs:5`, `utils.rs:5`) | None cite a bare package/directory line with no filenames indented beneath it -- every citation lists explicit `include/stomp/*.h`/`src/*.cpp` files | CONFIRMED, 0 hits of the shape the parser now closes | Read all four headers in full in this tree; `tools/ci/verify-upstream-license-provenance.sh` also run over the whole workspace this round: `checked 334 upstream file(s) cited by 242 tracked source file(s)`, 0 findings | (none) |
