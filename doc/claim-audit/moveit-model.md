# Claim audit — moveit-model

Format and rule: PORTING-PLAN.md §175. One row per claim a doc comment in
this tree makes about upstream. `evidence` is a file:line I opened myself
this round, not an inference from the port's own behavior. Appended as
found, not batched at report time.

## §172 upstream-first narrowing sweep — negative result, logged for §153.1 expiry tracking

Anchor: `int`/`size_t`/`unsigned`/`static_cast<(int\|unsigned\|size_t\|...)>`
whose initializing expression is floating-point, in the upstream C++ files
this crate's Rust sources cite. Swept via `rg` (`static_cast<...>`,
C-style `(int)`/`(size_t)`/`(unsigned)` casts, `floor/ceil/round/sqrt/pow`
near an integer declaration, integer decls with a float-literal or
division RHS) against each file below, output enumerated on screen.
`robot_model/src/aabb.cpp` additionally read in full (it is the one file
in this crate's cited set that does real floating-point geometry math, so
it got the closer read). Every file: 0 hits.

| where (upstream file swept) | claim | verdict | evidence |
|---|---|---|---|
| `robot_model/include/.../aabb.h` | no int/size_t/unsigned decl or `static_cast` with a floating-point initializer | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../aabb.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../fixed_joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../floating_joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../joint_model_group.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../link_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../planar_joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../prismatic_joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../revolute_joint_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/include/.../robot_model.hpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/aabb.cpp` | same; read in full — `extendWithTransformedBox` is all-`double`, no integer variables at all | CONFIRMED (absent) | full file read |
| `robot_model/src/fixed_joint_model.cpp` | no int/size_t/unsigned decl or `static_cast` with a floating-point initializer | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/floating_joint_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/joint_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/joint_model_group.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/link_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/planar_joint_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/prismatic_joint_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/revolute_joint_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |
| `robot_model/src/robot_model.cpp` | same | CONFIRMED (absent) | file opened, `rg` swept, 0 hits |

Port-side secondary sweep (`as (i8\|i16\|i32\|i64\|isize\|u8\|u16\|u32\|u64\|usize)`
receiving an `f64` expression, in `crates/moveit-model/src/`): 0 raw hits.

Expires (§153.1): if a future round adds an `int`/`size_t`/`unsigned`
declaration with a floating-point initializer to any file above, or a new
upstream file citing floating-point-narrowing joins this crate's cited
set, this table's `CONFIRMED (absent)` rows must be re-swept, not assumed
to still hold.
