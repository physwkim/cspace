# 포트 커버리지 — 상류 코퍼스의 포팅/미포팅 분할과 미포팅 89건의 분류

`PORTING-PLAN.md` §217이 이 파일을 가리킨다. 여기 있는 모든 수는
`tools/ci/measure-port-coverage.py`가 뽑은 것이고, 그 스크립트는 이 표를
`--check doc/port-coverage.md`로 되짚어 검사한다 — 표의 행 집합과 계기가
계산한 미포팅 집합이 다르면 non-zero로 끝난다. 그래서 이 표는 산문이
아니라 계기의 입력이며, 행 하나가 사라지거나 남으면 게이트가 운다.

## 1. 코퍼스 정의와 §1과의 차이

코퍼스 = 이 포트가 책임지는 상류 파일 집합. 기준 체크아웃은
`/home/stevek/work/moveit2` @ `e017c91ee12984393a28ba246075c65f69cde3bf`.

- `moveit_core/*` — 단 `controller_manager`, `collision_detection_bullet`,
  `collision_detection_fcl`, `version` 네 서브디렉터리 제외
- `moveit_kinematics/*`
- `moveit_planners/{chomp,stomp,pilz_industrial_motion_planner}`
- 확장자 `*.cpp *.hpp *.h`
- 경로에 `test`/`tests` 성분이 있으면 제외
- 내용이 `.h header is obsolete. Please use the .hpp header instead.` 인
  자동 생성 `.h` 포워딩 shim(상류 PR #3113)은 제외 — 이름이 아니라 내용으로
  판정하므로 진짜 `.h` 헤더가 실수로 빠지지 않는다

§1의 정의와 **세 군데** 다르다. 실측으로 확인한 결과:

1. §1은 파일 종류에 `*.cc`를 포함한다. 위 세 트리의 `.cc` 파일 수는
   **0**이므로 무영향이다.
   ```console
   $ U=/home/stevek/work/moveit2
   $ find $U/moveit_core $U/moveit_kinematics $U/moveit_planners/chomp \
          $U/moveit_planners/stomp \
          $U/moveit_planners/pilz_industrial_motion_planner -name '*.cc' | wc -l
   0
   ```
2. §1의 파일 수(`moveit_core` 292 등)는 테스트와 `.h` shim을 **포함한**
   전수이다. 이 코퍼스는 둘 다 뺀다.
3. `moveit_core/version`은 §1에 **한 번도 나오지 않는다**. 이 코퍼스는
   빌드 스탬프뿐인 그 디렉터리를 뺀다. 두 정의가 실제로 어긋나는 파일은
   `moveit_core/version/version.cpp` **하나**이고, §1 쪽 정의를 따르면
   코퍼스는 246이 된다.

"포팅됨"의 판정: `crates/` 또는 `ros/` 아래 어떤 `.rs` 파일의
`// Ported from moveit2 @ <40-hex-sha>:` 헤더 블록 안에 그 상류 경로가
나오면 포팅됨. `// Behaviorally derived from ...`은 이 판정에 들지 않는다
(그래서 `attached_body.hpp`는 아래 표에 `ported-elsewhere`로 남는다).

## 2. 실측 분할

```console
$ ./tools/ci/measure-port-coverage.py
corpus   245
ported   156
unported 89
cited-outside-corpus 20
```

두 계기가 245에 독립적으로 도달한다. 셸 파이프라인 쪽(다섯 루트를 `$R`):

```console
$ find $R \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) | wc -l
498
$ ... | grep -v -E '(^|/)tests?(/|$)' | wc -l
430
$ ... | grep -v -E '^moveit_core/(controller_manager|collision_detection_bullet|collision_detection_fcl|version)/' | wc -l
386
$ ... | 내용이 shim인 .h 141개 제외 | wc -l
245
```

`.h` 파일 142개 중 141개가 shim이고, shim이 아닌 단 하나는
`moveit_kinematics/ikfast_kinematics_plugin/templates/ikfast.h`다.
두 목록을 정렬해 `diff`한 결과는 동일(차이 0줄)이다.

`cited-outside-corpus` 20건은 `moveit_ros/*`·`moveit_core`의 제외
서브디렉터리처럼 코퍼스 밖 상류 파일을 인용한 헤더 블록이다. 인용된 서로
다른 상류 경로 166개는 전부 기준 체크아웃에 실제로 존재한다(파싱 쓰레기 0).

## 3. 분류 규칙

세 값 중 정확히 하나. 규칙을 먼저 적는다 — 표가 규칙을 만드는 것이 아니라
규칙이 표를 만든다.

- **`decided-non-port`** — `PORTING-PLAN.md`의 절 번호나 D1..D14 결정,
  또는 크레이트 `lib.rs`/모듈 doc의 문장이 **그 파일(또는 그 파일이
  선언하는 클래스)을 이름으로 지목해** 포팅하지 않기로 정한 경우. 표의
  증거 칸에 파일:줄을, 비고 칸에 그 문장을 인용한다.
- **`gap`** — 그런 문장을 찾지 못한 경우. **근거를 지어내지 않는다.**
  만료 조건이 이미 충족된 배제도 여기 들어간다 — 유예는 판정이 아니다.
  `collision_detector_allocator.hpp`가 그런 경우였고, §225.4가 유예를
  판정으로 바꾸면서 `decided-non-port`로 옮겼다.
- **`ported-elsewhere`** — 내용이 다른 이름으로 트리 안에 있는 경우.
  증거 칸에 `.rs` 파일과 심볼을 적는다. 잔여분이 있으면 비고에 남긴다 —
  잔여분이 결정되지 않았고 파일의 대부분이 트리에 없으면 `gap`이다.

부재 주장은 전부 `crates/ ros/ tools/ doc/ PORTING-PLAN.md` 코퍼스에
대한 `rg` 결과이고, 비고 칸에 그 명령을 적었다.

## 4. 미포팅 89건 (2026-08-06 실측)

`decided-non-port` 53 / `gap` 25 / `ported-elsewhere` 11.

| 상류 파일 | 분류 | 증거 | 비고 |
|---|---|---|---|
| `moveit_core/collision_detection/include/moveit/collision_detection/allvalid/collision_detector_allocator_allvalid.hpp` | decided-non-port | `PORTING-PLAN.md` §225.4; `crates/moveit-collision/src/env.rs:40-83` | the class body is one `CollisionDetectorAllocatorTemplate<CollisionEnvAllValid, CollisionDetectorAllocatorAllValid>` instantiation plus `static const std::string NAME` (verified upstream), so it follows its template's decision. §225.4 declines the allocator indirection itself: it exists to defer the backend *type* to a runtime string, and this port names the type at the call site (`E: CollisionEnv<..>`), so a registry would have registrants and no consumer |
| `moveit_core/collision_detection/include/moveit/collision_detection/collision_detector_allocator.hpp` | decided-non-port | `PORTING-PLAN.md` §225.4, §4.5; `crates/moveit-collision/src/env.rs:40-83` | env.rs used to *defer* this ("a compile-time registry needs at least one registrant to be worth adding"); that expiry was met -- three backends now implement `CollisionEnv` -- and §225.4 decided it rather than deferring again. Three grounds: `rg -n -i 'collision_detector|detector_name' crates/ ros/ tools/ --glob '*.rs'` finds 12 hits, all doc comments and no selection site; §177's `linkme` linker-order hazard; and no single `allocateEnv` signature serves `ParryCollisionEnv::new(world, padding_scale)`, `HybridCollisionEnv::new(.., link_body_decompositions, distance_field_config, collision_tolerance)` and the argument-less `AllValidCollisionEnv`. Narrows §4.5, which kept the trait for FCL-FFI extensibility -- that purpose is carried by `CollisionEnv` itself |
| `moveit_core/collision_detection/include/moveit/collision_detection/collision_plugin.hpp` | decided-non-port | `crates/moveit-collision/src/lib.rs:37-49` | "`CollisionPlugin::initialize` also takes a `planning_scene::PlanningScenePtr` (`collision_plugin.hpp:93`); `PlanningScene` lives in `moveit-scene`, which already depends on `moveit-collision`, so accepting it here would be a circular crate dependency" |
| `moveit_core/collision_detection/include/moveit/collision_detection/collision_plugin_cache.hpp` | decided-non-port | `crates/moveit-collision/src/lib.rs:37-49` | "its entire body is pluginlib runtime class loading ... plus `rclcpp` logging -- no algorithm exists independent of that ROS mechanism" |
| `moveit_core/collision_detection/include/moveit/collision_detection/collision_tools.hpp` | ported-elsewhere | `crates/moveit-collision/src/lib.rs:17` | its `.cpp` is cited as ported; the pure `CostSource` half is `total_cost`/`intersect_cost_sources`/`remove_overlapping` in `moveit-collision`. Residual: the four `visualization_msgs`/`moveit_msgs` marker/message functions (D1) |
| `moveit_core/collision_detection/include/moveit/collision_detection/occupancy_map.hpp` | gap | `crates/moveit-collision/src/lib.rs:51-68` | the text is a routing decision, not a decision not to port: "It is genuinely `RobotState`-free and portable, so 'no portable piece at all' was also false for this header ... request it against `moveit-octomap`" |
| `moveit_core/collision_detection/include/moveit/collision_detection/test_collision_common_panda.hpp` | gap | none | upstream's shared test body, but it carries no `test/` path component so the corpus rule keeps it; 0 hits in `crates/ ros/ doc/ PORTING-PLAN.md` |
| `moveit_core/collision_detection/include/moveit/collision_detection/test_collision_common_pr2.hpp` | gap | none | same |
| `moveit_core/collision_detection/src/collision_plugin_cache.cpp` | decided-non-port | `crates/moveit-collision/src/lib.rs:37-49` | same sentence |
| `moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_detector_allocator_distance_field.hpp` | decided-non-port | `crates/moveit-distance-field/src/lib.rs:541-553` | "both `CollisionDetectorAllocatorTemplate<...>` ROS-pluginlib-style runtime plugin registrations. D-decision: D4" |
| `moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_detector_allocator_hybrid.hpp` | decided-non-port | `crates/moveit-distance-field/src/lib.rs:541-553` | same sentence |
| `moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler_allocator.hpp` | decided-non-port | `crates/moveit-constraints/src/lib.rs:553-562` | "D4: the whole plugin-allocator interface is excluded (D4 already excludes runtime plugin-by-string dispatch) -- nothing in this crate implements this interface" |
| `moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler_tools.hpp` | decided-non-port | `PORTING-PLAN.md` §225.1, `crates/moveit-constraints/src/lib.rs:574-611` | 3 of 4 declarations are D1; §225.1 decides the fourth: "루프의 종료 조건이 벽시계다 ... 호출 하나가 구조적으로 1초 이상 걸린다. 이 워크스페이스의 어떤 테스트도 그 출력에 단언을 걸 수 없다", and the same `valid / total` is already measured deterministically by `tests/sampler_self_validation.rs`'s `attempted`/`produced` accounting |
| `moveit_core/constraint_samplers/src/constraint_sampler.cpp` | ported-elsewhere | `crates/moveit-constraints/src/sampler.rs:184,377`, module doc "`constraint_sampler.cpp`: where its two function bodies went" | the file holds two bodies. The ctor's one substantive line, `jmg_ = scene->getRobotModel()->getJointModelGroup(group_name)`, is `JointConstraintSampler::new`/`UnionConstraintSampler::new`'s `model.joint_model_group(group_name)?`. Residual `clear()` decided non-port by `PORTING-PLAN.md` §225.2 (it exists for upstream's reconfigure and partial-configure-rollback paths, neither of which a fallible `new()` can reach). The previous `gap` reason cited `getName()`, which is not declared in this file at all |
| `moveit_core/constraint_samplers/src/constraint_sampler_tools.cpp` | decided-non-port | `PORTING-PLAN.md` §225.1, `crates/moveit-constraints/src/lib.rs:574-611` | same four declarations, same decision; this is the file §225.1 quotes line numbers from (`:82,92` for the wall-clock loop bound, `:68` for the one caller) |
| `moveit_core/exceptions/src/exceptions.cpp` | ported-elsewhere | `crates/moveit-error/src/lib.rs:21-28,64-73` | "An unrecoverable error, replacing upstream's `moveit::Exception` hierarchy" -- `moveit_error::Error`, with `Error::Construct` for `ConstructException` |
| `moveit_core/macros/include/moveit/macros/class_forward.hpp` | decided-non-port | `crates/moveit-trajectory/src/lib.rs:51-52`, `:370` | "`MOVEIT_CLASS_FORWARD(TimeParameterization)`/`MOVEIT_CLASS_FORWARD(TimeOptimalTrajectoryGeneration)` -- both unported"; the header's whole content is `MOVEIT_CLASS_FORWARD`/`MOVEIT_STRUCT_FORWARD` (2 `#define`s, verified upstream) |
| `moveit_core/macros/include/moveit/macros/console_colors.hpp` | gap | none | `rg -n -F console_colors crates/ ros/` -> 0 hits; 9 ANSI-escape `#define`s upstream |
| `moveit_core/macros/include/moveit/macros/declare_ptr.hpp` | decided-non-port | `crates/moveit-distance-field/src/lib.rs:873-876` | "`MOVEIT_DECLARE_PTR_MEMBER(VoxelGrid)` -- unported: a C++ smart-pointer-alias macro with no Rust equivalent needed"; the header's whole content is `MOVEIT_DECLARE_PTR`/`MOVEIT_DECLARE_PTR_MEMBER` (2 `#define`s, verified upstream) |
| `moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/smoothing_base_class.hpp` | decided-non-port | `crates/moveit-smoothing/src/lib.rs:28-37` | "excluded (D1 + D4). `SmoothingBaseClass` is a pluginlib abstract interface: `initialize` takes `rclcpp::Node::SharedPtr` in the trait itself (D1)"; `lib.rs:71-79` records the header itself as fully audited |
| `moveit_core/online_signal_smoothing/src/smoothing_base_class.cpp` | decided-non-port | `crates/moveit-smoothing/src/lib.rs:28-37` | same sentence, plus "`.cpp` has no content to port regardless: it is a default constructor/destructor" |
| `moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.hpp` | gap | `crates/moveit-planners-sbp/src/registry.rs:1-12`, `lib.rs:68-95` | a D1/D4-adapted stand-in exists (`PlannerManager`/`PlanningContext` in `registry.rs`), but the crate's own 25-declaration audit leaves 22 unported and undecided |
| `moveit_core/planning_interface/include/moveit/planning_interface/planning_request.hpp` | ported-elsewhere | `crates/moveit-planning/src/request.rs:4-10` | "[`PlanningRequest`] replaces the fields of `moveit_msgs::msg::MotionPlanRequest` this crate's six request adapters actually read"; `WorkspaceBounds` replaces `WorkspaceParameters` minus its D1 header |
| `moveit_core/planning_interface/include/moveit/planning_interface/planning_request_adapter.hpp` | ported-elsewhere | `crates/moveit-planning/src/lib.rs:404` | "Replaces `planning_interface::PlanningRequestAdapter`" -- the `PlanningRequestAdapter` trait in `moveit-planning` |
| `moveit_core/planning_interface/include/moveit/planning_interface/planning_response.hpp` | ported-elsewhere | `crates/moveit-planning/src/response.rs:33` | cites `planning_response.hpp:48-70` for `PlanningResponse`'s field set |
| `moveit_core/planning_interface/include/moveit/planning_interface/planning_response_adapter.hpp` | ported-elsewhere | `crates/moveit-planning/src/lib.rs:420` | the response-adapter equivalent of the row above |
| `moveit_core/planning_interface/src/planning_interface.cpp` | gap | none | no citation and no exclusion; the stand-in in `registry.rs` is not a port of this file |
| `moveit_core/planning_interface/src/planning_response.cpp` | gap | `crates/moveit-planning/src/response.rs:100` | the one reference is to `moveit_py`'s same-named file, not this one |
| `moveit_core/robot_state/include/moveit/robot_state/attached_body.hpp` | ported-elsewhere | `crates/moveit-scene/src/attached_body.rs:1-7`, `:56` | `// Behaviorally derived from moveit2 @ ...: .../attached_body.hpp` -- `pub struct AttachedBody`. The instrument counts it unported because `Behaviorally derived from` is not the `Ported from` header form |
| `moveit_core/robot_state/include/moveit/robot_state/conversions.hpp` | gap | none | `robotStateToStream`/`streamToRobotState`/`jointTrajPointToRobotState` measured absent from the same corpus; the `moveit_msgs` half is D1/D6 and lives in `ros/moveit-ros` |
| `moveit_core/robot_state/src/attached_body.cpp` | gap | `crates/moveit-scene/src/attached_body.rs` | partially covered by the row above; residual measured absent from `crates/ ros/ tools/ doc/ PORTING-PLAN.md`: `setScale`, `setPadding`, `computeTransform`, `getGlobalSubframeTransform` |
| `moveit_core/robot_state/src/conversions.cpp` | gap | none | same |
| `moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_parameterization.hpp` | decided-non-port | `crates/moveit-trajectory/src/time_optimal_trajectory_generation.rs:9-10`, `:121-160` | listed under `// Considered and deliberately not ported:` with a full `# Not ported: `TimeParameterization`` section |
| `moveit_core/utils/include/moveit/utils/eigen_test_utils.hpp` | gap | none | `rg -n -F eigen_test_utils crates/ ros/` -> 0 hits |
| `moveit_core/utils/include/moveit/utils/lexical_casts.hpp` | gap | `crates/moveit-error/src/lib.rs:312` | the one reference explains upstream's `toString`, it does not port it; no Rust equivalent is named |
| `moveit_core/utils/include/moveit/utils/logger.hpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:162` | "`<moveit/utils/logger.hpp>`'s `rclcpp::Logger` plus every `RCLCPP_*` call" is excluded as D1 across the workspace |
| `moveit_core/utils/include/moveit/utils/message_checks.hpp` | ported-elsewhere | `ros/moveit-ros/src/scene/collision_object.rs:11` | its `.cpp` is cited as ported (`isEmpty(Pose):77`) in `moveit-ros` |
| `moveit_core/utils/include/moveit/utils/rclcpp_utils.hpp` | gap | none | `rg -n -F rclcpp_utils crates/ ros/` -> 0 hits. D1 by content (`rclcpp`), but no text in this repo says so |
| `moveit_core/utils/include/moveit/utils/robot_model_test_utils.hpp` | gap | none | `rg -n -F robot_model_test_utils crates/ ros/` -> 0 hits |
| `moveit_core/utils/src/lexical_casts.cpp` | gap | `crates/moveit-error/src/lib.rs:312` | same |
| `moveit_core/utils/src/logger.cpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:162` | same D1 exclusion |
| `moveit_core/utils/src/rclcpp_utils.cpp` | gap | none | same |
| `moveit_core/utils/src/robot_model_test_utils.cpp` | gap | none | same |
| `moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/detail/GreedyKCenters.hpp` | decided-non-port | `crates/moveit-kinematics/src/lib.rs:279-287` | header quotes verbatim: "Copyright (c) 2011, Rice University" / "Author: Mark Moll" / "// This file is a slightly modified version of <ompl/datastructures/GreedyKCenters.h>" -- an OMPL file vendored into this plugin, not MoveIt-authored. `grep -rln GreedyKCenters /home/stevek/work/moveit2/` (whole checkout, not just this plugin) names exactly one includer outside the `.h`-forwarding shim: `NearestNeighborsGNAT.hpp:47,482,751`, where it is the pivot-selection helper for that class's metric tree. `NearestNeighborsGNAT.hpp` is already `decided-non-port` on this same row set ("a linear scan ... is microseconds, not a bottleneck worth porting"); `GreedyKCenters.hpp` has no caller once that tree is not built, so the same decision covers it. Nothing in this port provides the same structure (`rg -n -i 'greedy.?k.?center' crates/ ros/ doc/ PORTING-PLAN.md` -> 0 hits outside this row); the functional role GNAT's tree served (nearest cached entry by pose) is filled instead by the linear scan in `IkCache::nearest` (`crates/moveit-kinematics/src/ik_cache.rs:193`) |
| `moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/detail/NearestNeighbors.hpp` | decided-non-port | `crates/moveit-kinematics/src/lib.rs:279-287` | header quotes verbatim: "Copyright (c) 2008, Willow Garage, Inc." / "Author: Ioan Sucan" / "// This file is a slightly modified version of <ompl/datastructures/NearestNeighbors.h>" -- also OMPL-derived, not MoveIt-original. `grep -rln 'NearestNeighbors\.hpp' /home/stevek/work/moveit2/` (whole checkout) names exactly one includer outside the `.h`-forwarding shim: `NearestNeighborsGNAT.hpp`, whose class is this abstract interface's only implementation anywhere in the corpus. Same GNAT decision applies for the same reason -- no caller of the abstract interface survives once the one concrete implementation is not ported. Same "no equivalent structure" `rg` anchor and same functional-role note (`IkCache::nearest`, `crates/moveit-kinematics/src/ik_cache.rs:193`) as the `GreedyKCenters.hpp` row above |
| `moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/detail/NearestNeighborsGNAT.hpp` | decided-non-port | `crates/moveit-kinematics/src/lib.rs:279-287` | "A linear scan over at most 10k entries ... is microseconds, not a bottleneck worth porting `detail/NearestNeighborsGNAT.hpp` (755 lines implementing a general-purpose metric tree) to avoid" |
| `moveit_kinematics/cached_ik_kinematics_plugin/src/cached_ik_kinematics_plugin.cpp` | decided-non-port | `crates/moveit-kinematics/src/lib.rs:306-311` | "it is pluginlib `PLUGINLIB_EXPORT_CLASS` boilerplate, and this crate's compile-time [`KINEMATICS_SOLVERS`] registry (decision D4) replaces that mechanism entirely" |
| `moveit_kinematics/cached_ik_kinematics_plugin/src/cached_ur_kinematics_plugin.cpp` | decided-non-port | `crates/moveit-kinematics/src/lib.rs:294-306` | "the entire file is a `PLUGINLIB_EXPORT_CLASS` registration of `CachedIKKinematicsPlugin<ur_kinematics::URKinematicsPlugin>` ... this crate does not have it and is not porting it" |
| `moveit_kinematics/ikfast_kinematics_plugin/templates/ikfast.h` | decided-non-port | `PORTING-PLAN.md` §60.4; `crates/moveit-kinematics/src/lib.rs:241-250` | §60.4: "`ikfast_kinematics_plugin` 미포팅(이식할 알고리즘 없음, codegen 템플릿)". The one `.h` in the corpus that is not a deprecation shim |
| `moveit_kinematics/ikfast_kinematics_plugin/templates/ikfast61_moveit_plugin_template.cpp` | decided-non-port | `PORTING-PLAN.md` §60.4; `crates/moveit-kinematics/src/lib.rs:241-250` | §60.4 as above; lib.rs: "a 1421-line C++ template with placeholder tokens that OpenRave's separate, external IKFast code generator fills in with a *robot-specific* closed-form analytic solution" |
| `moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/chainiksolver_vel_mimic_svd.hpp` | decided-non-port | `PORTING-PLAN.md` §159.1, §162; `crates/moveit-kinematics/src/lib.rs:57-76` | LGPL-2.1 file in a BSD-3-Clause workspace; re-derived rather than transcribed (D11), see `velocity.rs` |
| `moveit_kinematics/kdl_kinematics_plugin/src/chainiksolver_vel_mimic_svd.cpp` | decided-non-port | `PORTING-PLAN.md` §159.1, §162; `crates/moveit-kinematics/src/velocity.rs:17` | same decision |
| `moveit_kinematics/srv_kinematics_plugin/include/moveit/srv_kinematics_plugin/srv_kinematics_plugin.hpp` | decided-non-port | `crates/moveit-kinematics/src/lib.rs:232-240` | "excluded, D1/D2 (no ROS dependency) ... There is no numeric solver here to port -- the entire class *is* the ROS surface" |
| `moveit_kinematics/srv_kinematics_plugin/src/srv_kinematics_plugin.cpp` | decided-non-port | `crates/moveit-kinematics/src/lib.rs:232-240` | same sentence |
| `moveit_planners/chomp/chomp_interface/include/chomp_interface/chomp_interface.hpp` | decided-non-port | `crates/moveit-planners-chomp/src/lib.rs:20-32` | "`chomp_interface/` is excluded per `PORTING-PLAN.md` D1/D2 ... Nothing in `chomp_interface/` is algorithmic" |
| `moveit_planners/chomp/chomp_interface/include/chomp_interface/chomp_planning_context.hpp` | decided-non-port | `crates/moveit-planners-chomp/src/lib.rs:20-32` | same sentence |
| `moveit_planners/chomp/chomp_interface/src/chomp_interface.cpp` | decided-non-port | `crates/moveit-planners-chomp/src/lib.rs:20-32` | same sentence |
| `moveit_planners/chomp/chomp_interface/src/chomp_planning_context.cpp` | decided-non-port | `crates/moveit-planners-chomp/src/lib.rs:20-32` | same sentence |
| `moveit_planners/chomp/chomp_interface/src/chomp_plugin.cpp` | decided-non-port | `crates/moveit-planners-chomp/src/lib.rs:20-32` | same sentence |
| `moveit_planners/pilz_industrial_motion_planner/include/joint_limits_copy/joint_limits_rosparam.hpp` | decided-non-port | `PORTING-PLAN.md` §224.4; `crates/moveit-planners-pilz/src/joint_limits_aggregator.rs:35-67` | §224.4: "302줄 ... `node->` 출현 53회 ... 상류에서도 이 파일은 파라미터 서버 어댑터이지 pilz의 계산이 아니다" (파일 머리의 상류 주석이 ros2_control DRAFT PR #462 복사본이라고 말한다). D1, 남는 비-ROS 잔여분 0 |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/capability_names.hpp` | gap | none | `rg -n -F capability_names crates/moveit-planners-pilz/` -> 0 hits |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/joint_limits_interface_extension.hpp` | decided-non-port | `PORTING-PLAN.md` §224.4; `crates/moveit-planners-pilz/src/joint_limits_aggregator.rs:35-67` | §224.4: "파일 전체가 100줄, 내용은 인라인 함수 둘뿐이고 둘 다 `const rclcpp::Node::SharedPtr&`를 받는다". 헤더가 광고하는 deceleration 확장은 실제로는 `joint_limits_extension.hpp`에 있고 `limits.rs`가 이미 인용해 포팅했다. D1, 남는 비-ROS 잔여분 0 |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/move_group_sequence_action.hpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:110-115` | "`actionlib`/`rclcpp` action and service servers wrapping the planner for `move_group`; nothing here computes a trajectory" |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/move_group_sequence_service.hpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:110-115` | same sentence |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/pilz_industrial_motion_planner.hpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:118-119` | "the `planning_interface::PlannerManager` plugin itself, i.e. the `move_group` entry point" -- the sentence names the `.cpp`; this header declares that same class |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_base.hpp` | gap | none | `rg -n -F PlanningContextBase crates/moveit-planners-pilz/` -> 0 hits; lib.rs excludes `planning_context_loader*`, not the contexts themselves |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_circ.hpp` | gap | none | `rg -n -F PlanningContextCIRC crates/moveit-planners-pilz/` -> 0 hits |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_lin.hpp` | gap | none | `rg -n -F PlanningContextLIN crates/moveit-planners-pilz/` -> 0 hits |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_loader.hpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:116-117` | "`planning_context_loader*.{hpp,cpp}` -- a `pluginlib`-loaded factory ... its entire job is ROS plugin registration, not planning math" |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_loader_circ.hpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:116-117` | same glob |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_loader_lin.hpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:116-117` | same glob |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_loader_polyline.hpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:116-117` | same glob |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_loader_ptp.hpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:116-117` | same glob |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_polyline.hpp` | gap | none | same search shape -> 0 hits |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_context_ptp.hpp` | gap | none | `rg -n -F PlanningContextPTP crates/moveit-planners-pilz/` -> 0 hits |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/planning_exceptions.hpp` | gap | none | `rg -n -F planning_exceptions crates/moveit-planners-pilz/` -> 0 hits |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/tip_frame_getter.hpp` | ported-elsewhere | `crates/moveit-planners-pilz/src/trajectory_functions.rs:795` | "(`tip_frame_getter.hpp`), minus the 'more than one tip frame' case" -- the residual is the multi-tip branch |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generation_exceptions.hpp` | gap | none | `rg -n -F trajectory_generation_exceptions crates/moveit-planners-pilz/` -> 0 hits |
| `moveit_planners/pilz_industrial_motion_planner/src/move_group_sequence_action.cpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:110-115` | same sentence |
| `moveit_planners/pilz_industrial_motion_planner/src/move_group_sequence_service.cpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:110-115` | same sentence |
| `moveit_planners/pilz_industrial_motion_planner/src/pilz_industrial_motion_planner.cpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:118-119` | the sentence names this file |
| `moveit_planners/pilz_industrial_motion_planner/src/planning_context_loader.cpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:116-117` | same glob |
| `moveit_planners/pilz_industrial_motion_planner/src/planning_context_loader_circ.cpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:116-117` | same glob |
| `moveit_planners/pilz_industrial_motion_planner/src/planning_context_loader_lin.cpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:116-117` | same glob |
| `moveit_planners/pilz_industrial_motion_planner/src/planning_context_loader_polyline.cpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:116-117` | same glob |
| `moveit_planners/pilz_industrial_motion_planner/src/planning_context_loader_ptp.cpp` | decided-non-port | `crates/moveit-planners-pilz/src/lib.rs:116-117` | same glob |
| `moveit_planners/stomp/include/stomp_moveit/stomp_moveit_planning_context.hpp` | ported-elsewhere | `crates/moveit-planners-stomp/src/planner.rs` | its `.cpp` is cited as ported; `sample_goal_state`/`extract_seed_trajectory` are the round-25 ports named in `lib.rs:93-97` |
| `moveit_planners/stomp/include/stomp_moveit/trajectory_visualization.hpp` | decided-non-port | `crates/moveit-planners-stomp/src/lib.rs:119-124` | "its own includes are `visualization_msgs::msg::MarkerArray`/`Marker`, `std_msgs::msg::ColorRGBA`, `tf2_eigen/tf2_eigen.hpp` -- ROS message and `tf2` types are the function signatures themselves" |
| `moveit_planners/stomp/src/stomp_moveit_planner_plugin.cpp` | decided-non-port | `crates/moveit-planners-stomp/src/lib.rs:111-118` | "a ROS-hosted plugin entry point, not a computation this crate could port independently of `rclcpp`" |
