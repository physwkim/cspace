# Claim audit: moveit-geometry

Per PORTING-PLAN.md §175. One row per item, appended as found, not
batched at report time. `evidence` is the upstream `file:line` actually
opened this round -- inference from the port is not evidence.

Upstream roots for this crate: `third_party/geometric_shapes/` (tag
`192801cebacc07d0e9f719576cdd1c9b36d0bc28`) and
`/home/stevek/work/moveit2` (pinned `e017c91ee12984393a28ba246075c65f69cde3bf`).

## §172 sweep (round 33), upstream-first per `0ad8c67`

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-geometry/src/stl.rs:161-167` (doc bullet: `createMeshFromShape` unported) | `createMeshFromShape(const Cylinder&)`/`createMeshFromShape(const Cone&)` narrow `ceil(...)` to `unsigned int` (`tot`, `h_num`); port correctly excludes both (verified zero `moveit_core` callers against the pinned tree, only `moveit_ros` callers which are out of scope) | CONFIRMED not-applicable | `third_party/geometric_shapes/geometric_shapes/src/mesh_operations.cpp:523,527` (Cylinder: `unsigned int tot = std::max<unsigned int>(6, ceil(tot_for_unit_cylinder * r)); ... unsigned int h_num = ceil(std::abs(h) / circle_edge);`) and `:594,598` (Cone: `unsigned int tot = tot_for_unit_cone * r; ... unsigned int h_num = ceil(h / circle_edge);`) | (none) |
| `third_party/geometric_shapes/geometric_shapes/src/{bodies,aabb,obb,shapes}.cpp`, `moveit2 moveit_core/robot_model/src/robot_model.cpp` (`constructShape`), `moveit2 moveit_core/transforms/src/transforms.cpp` | No float-derived `int`/`unsigned`/`size_t` narrowing anywhere else in the files this crate ports | CONFIRMED, 0 additional hits | Full-file grep of each, read in this tree; all `int`/`unsigned`/`size_t` declarations found are integer-derived (`.size()`, vertex/triangle counts, an enum cast) | (none) |
| `crates/moveit-geometry/src/{bodies,shapes,stl}.rs` (port-side anchor: `as u8..u128/usize` receiving `f64`) | All 10 hits narrow an already-integer triangle/vertex index or byte count, not a real-valued quantity | CONFIRMED distinct, 10 sites: `bodies.rs:2552-2554,2684,2830-2832,2912-2914`; `crates/moveit-geometry/src/shapes.rs:1131,1238-1240,1269-1271,1323-1325`; `stl.rs:195,231,320` | Read in this tree only -- each narrows a `tri[N]`/`idx`/`.len()` value | (none) |

## §167.6 bare-directory-citation sweep (this round)

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| Every `Ported from` header in this crate (`crates/moveit-geometry/src/lib.rs:5`, `bodies.rs:9`, `transforms.rs:5`, `stl.rs:6`, `crates/moveit-geometry/src/shapes.rs:5`) | None cite a bare package/directory line (a `.../` line with no filenames indented beneath it) -- every citation lists explicit files, a brace-expansion of files, or an indented filename block | CONFIRMED, 0 hits of the shape the parser now closes | Read all five headers in full in this tree; `tools/ci/verify-upstream-license-provenance.sh` also run over the whole workspace this round: `checked 334 upstream file(s) cited by 242 tracked source file(s)`, 0 findings | (none) |
