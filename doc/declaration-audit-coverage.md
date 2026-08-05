# 선언 단위 감사 커버리지 — 포팅됨 158건 중 어디까지 세어 봤는가

`doc/port-coverage.md`는 **미포팅** 87건을 분류한다. 이 파일은 반대쪽,
**포팅됨 158건**을 다룬다. 두 문서가 재는 것이 다르다는 점이 이 파일이
존재하는 이유다.

`port-coverage.md` §1이 "포팅됨"의 판정을 그대로 적어 두었다: 어떤 `.rs`
파일의 `// Ported from moveit2 @ <40-hex-sha>:` 헤더 블록 안에 그 상류
경로가 나오면 포팅됨. **파일 단위 주장이다.** 그 상류 파일이 선언하는
것들이 이쪽에서 다 처리됐는지는 아무 계기도 보지 않는다. 1,764줄짜리
헤더를 인용하면서 그 228개 public 선언 중 **몇 개를 옮겼든** 게이트는 전부
초록이다 — 그 헤더가 바로 아래 표의 `robot_state.hpp`이고, 몇 개인지는
아무도 세어 본 적이 없다는 것이 이 문서의 요지다.

## 1. 코퍼스와 "포팅됨"의 정의

이 표의 행 집합은 `port-coverage.md` §1·§2의 코퍼스에서 파생된 것이지
독립 정의가 아니다. 그러므로 정의는 거기 한 곳에만 있다:

- 코퍼스 = `moveit_core/*`(단 `controller_manager`,
  `collision_detection_bullet`, `collision_detection_fcl`, `version` 제외),
  `moveit_kinematics/*`, `moveit_planners/{chomp,stomp,pilz_industrial_motion_planner}`,
  확장자 `*.cpp *.hpp *.h`, 경로에 `test`/`tests` 성분이 있으면 제외,
  내용이 shim인 자동 생성 `.h` 제외. 기준 체크아웃은
  `/home/stevek/work/moveit2` @ `e017c91ee12984393a28ba246075c65f69cde3bf`.
- 포팅됨 = 위의 헤더 블록 인용이 있는 것.

```console
$ ./tools/ci/measure-declaration-audits.py
ported 158
```

같은 158이며, 계기가 `measure-port-coverage.py`의 `corpus_files()`/
`cited_paths()`를 **import**한다(복사가 아니다 —
`check-audit-scripts-not-copied.sh` 헤더가 적은 이유 그대로).

## 2. 판정 규칙 — "선언 단위 감사"란 무엇인가

두 값뿐이다. 규칙을 먼저 적는다.

- **`audited`** — 트리 어딘가의 문장이 그 상류 파일(또는 그 파일이
  선언하는 클래스)의 **모든** public 선언을 열거했다고 *주장하고*, 선언마다
  처분(ported as / 배제 + D번호 / unported + 사유)을 붙인 경우. 증거 칸에
  그 문장의 `파일:줄`을 적는다. 열어 보면 확인된다는 것이 이 칸의 전부다.
- **`none`** — 그런 완전성 주장이 트리 어디에도 없는 경우. 그 파일을
  *언급하지 않았다*는 뜻이 아니다.

**엄격한 쪽으로 정했고, 그 대가를 적어 둔다.** 경계에 있는 것을 두 건
직접 열어 확인했다:

- `moveit_core/exceptions/include/moveit/exceptions/exceptions.hpp` —
  `crates/moveit-error/src/lib.rs:21-28`이 이 헤더가 선언하는 **두** 클래스
  (`moveit::Exception`, `moveit::ConstructException`, 각각 public 선언 1개 —
  `count-public-declarations.sh`로 확인)를 모두 이름으로 대응시킨다. 사실상
  전수지만 전수라고 주장하지 않는다. 규칙대로 `none`이다.
- `moveit_planners/stomp/include/stomp_moveit/filter_functions.hpp` —
  `crates/moveit-planners-stomp/src/filter_functions.rs`는 `simpleSmoothingMatrix`와
  `FilterFn`을 다루지만 헤더 전체를 열거하지 않는다. `none`이다.

즉 `none`은 "아무도 셌다고 말한 적 없다"는 뜻이고, 이 문서가 재려는 구멍이
정확히 그것이다.

**한계 하나를 명시한다.** 감사 대부분은 `.hpp`/`.cpp` 쌍을 **함께** 다룬다 —
클래스가 헤더에 선언되고 짝이 되는 `.cpp`에 정의되기 때문이다. 이 표도 그
관례를 따라 쌍을 같은 값으로 적는다. 따라서 `.cpp`에만 있는 파일-지역
헬퍼는 `audited` 행에서도 열거된 바 없을 수 있다. 이것이 이론이 아님을
트리 자신이 적어 두었다:
`crates/moveit-smoothing/src/lib.rs:71-78`이 반례를 이름으로 든다 —
`time_optimal_trajectory_generation.cpp`에는 헤더 선언이 아예 없는 클래스가
둘 있다(`Path`, `Trajectory`). 그 둘은 `crates/moveit-trajectory/src/lib.rs:90-93`이
따로 처리했고, 그래서 그 `.cpp`는 헤더를 통해서가 아니라 자기 자격으로
`audited`다.

## 3. 실측 (2026-08-06)

```console
$ ./tools/ci/verify-declaration-audits.sh
ported 158
OK doc/declaration-audit-coverage.md: 158 rows == 158 ported files; 73 audited, 85 none
```

측정을 시작했을 때는 **69 / 89**였다. 그 중 4건 —
`collision_detection/{world,collision_matrix}.{hpp,cpp}` — 을 이 라운드에서
실제로 감사해 `audited`로 넘겼고, 그래서 지금 값이 73 / 85다. 두 수를 다
적는 이유는 89가 이미 여러 곳에서 인용될 수 있는 값이라서가 아니라, 이
문서를 만든 라운드가 그 값을 스스로 4만큼 움직였다는 사실이 표 자체에는
남지 않기 때문이다.

앞의 문서와 달리 **이 세 수는 게이트가 검사한다.** `--check`가 행 집합·판정
어휘·증거 칸을 전부 대조한다. `audited`인데 증거가 없으면 실패하고,
증거가 있어도 `파일:줄` 형태가 아니거나 그 파일이 없거나 그 줄이 파일 끝을
넘으면 실패한다 — 열 수 없는 감사는 감사가 아니다. 검사하지 **않는** 것:
그 줄이 여전히 감사문의 시작인지. 정규식으로는 확인할 수 없다.

미감사 85건이 어디에 몰려 있는지가 이 측정의 요점이다:

이 요약표는 크레이트를 먼저 적는다. §5의 행이 `| \`moveit_…` 로 시작하고
`measure-declaration-audits.py`가 바로 그 접두사로 행을 잡아내므로, 상류
경로를 첫 칸에 두면 요약표 여덟 줄이 데이터 행으로 파싱되어 게이트가
문법 오류로 실패한다. 크레이트 이름은 하이픈(`moveit-`)이라 충돌하지
않는다.

| 인용하는 크레이트 | 상류 디렉터리 | `none` |
|---|---|---:|
| `moveit-planners-pilz` | `moveit_planners/pilz_industrial_motion_planner/{include,src}` | 39 |
| `moveit-model` (+`moveit-geometry` 1건) | `moveit_core/robot_model/{include,src}` | 20 |
| `moveit-collision` | `moveit_core/collision_detection/{include,src}` | 9 |
| `moveit-planners-stomp`, `moveit-sampling` | `moveit_planners/stomp/{include,src}` | 7 |
| `moveit-state`, `moveit-kinematics` | `moveit_core/robot_state/{robot_state.hpp,robot_state.cpp}` | 2 |
| `moveit-state` | `moveit_core/dynamics_solver/{include,src}` | 2 |
| `moveit-geometry` | `moveit_core/transforms/{include,src}` | 2 |
| `moveit-error`, `moveit-ros`, `moveit-kinematics` | `moveit_core/exceptions`, `moveit_core/utils`(×2), `kdl_kinematics_plugin/joint_mimic.hpp` | 4 |

세 크레이트가 85건 중 **68건**을 지고 있다. 셋 다 라운드 시작 시점에는 선언
단위 감사가 **0**이었다 — 각각 크레이트 doc에 범위를 산문으로 적지만 선언을
세어 처분한 문장은 어느 쪽에도 없었다. 이 라운드가 `moveit-collision`의
13건 중 4건을 그 상태에서 꺼냈고, `moveit-planners-pilz` 39건과
`moveit-model` 20건은 그대로 남아 있다.

**단일 파일로 가장 큰 것은 `robot_state.hpp`다.** `count-public-declarations.sh`로
`RobotState` 자체만 **228** public 선언(파일 1,764줄)이고, 두 크레이트가
인용하며, 감사는 없다. 이 라운드는 그것을 하지 않았다 — 228건은 이 라운드
안에 끝나지 않고, 절반만 한 감사는 완전성 주장을 할 수 없으므로 위 규칙상
`none`과 구별되지 않는다.

## 4. `moveit-test-support`

크레이트 22개 중 `doc/claim-audit/`에 대응 파일이 없는 유일한 크레이트다
(나머지 21개 + `moveit-ros` + `tools-ci-gates.md` = 23개 파일). 다만 이
표에는 아무 행도 기여하지 않는다: 이 크레이트의 `.rs`에는 `// Ported from
moveit2 @ ...` 헤더 블록이 하나도 없어서 인용하는 상류 파일이 0건이다.
따라서 claim-audit 파일의 부재는 이 문서가 재는 구멍이 아니라 별개의
구멍이며, 여기서는 사실만 적고 판정하지 않는다.

## 5. 이 라운드가 실제로 감사한 4건

`moveit-collision` 13건 중 중심 두 클래스 — `World`(37)와
`AllowedCollisionMatrix`(29), 둘 다 `count-public-declarations.sh` 실측 —
와 그 짝 `.cpp`. 가장 큰 미감사 단일 파일인 `robot_state.hpp`(228)를 고르지
않은 이유는 §3에 적었다.

감사문은 `crates/moveit-collision/src/world.rs:119`와
`crates/moveit-collision/src/matrix.rs:13`에 있고, 선언마다 처분이 붙는다.
두 헤더의 스크립트 계수는 각각 손으로 열거한 목록과 정확히 일치했다(37, 29).
`World`의 중첩 `struct Object`는 스크립트가 셀 수 없어(`class` 만 매칭,
깊이 1만 계수) `world.hpp:78-117`에서 손으로 8건을 열거했다고 감사문에
적었다.

처분이 `ported`가 아닌 것은 다음뿐이다.

- **decided-non-port 8건.** `~World()`(관찰자 해제뿐, 관찰자가 없음),
  `using const_iterator`(컨테이너 표현 공개), `ObserverHandle` /
  `ObserverCallbackFn` / `addObserver` / `removeObserver`(deviation 4,
  만료 조건은 감사문에 명시), `MOVEIT_CLASS_FORWARD(AllowedCollisionMatrix)`,
  `AllowedCollisionMatrix::print`.
- **unported-in-scope 2건.** `AllowedCollisionMatrix(const
  moveit_msgs::msg::AllowedCollisionMatrix&)`와 `getMessage()`. D6/§4.3이
  `moveit-ros`의 `TryFrom` 층으로 보내는데 아직 없다. 새로 발견한 미처리가
  아니라 이미 트리에 이름이 적힌 구멍이다 —
  `ros/moveit-ros/src/scene/planning_scene.rs:19-24`가
  `allowed_collision_matrix`를 미변환 `PlanningScene` 필드 목록에 넣어 두었고,
  그 파일을 열어 확인했다. 만료는 그 변환이 들어오는 날.

`doc/port-coverage.md`에 새 `gap` 행은 생기지 않았고, 생길 수 없다: 그
표는 **미포팅 파일**을 분류하는데 이 4건은 전부 포팅된 파일이고, 위 2건은
파일이 아니라 파일 안의 선언이다. 선언 단위의 미처리를 적을 자리는 그 표가
아니라 이 문서이며, 그 자리가 없었다는 것이 이 문서가 생긴 이유이기도 하다.

감사 중에 상류 서술 오류 하나를 고쳤다. `matrix.rs`가
`AllowedCollisionMatrix::print`를 "`rclcpp` 로거(`RCLCPP_WARN_STREAM_THROTTLE`)로
포맷하므로 미포팅"이라고 적고 있었다. `collision_matrix.cpp:428-491`을 열어
보면 로깅이 없다 — 호출자가 준 `std::ostream&`에 ASCII 표를 쓴다.
`RCLCPP_WARN_STREAM_THROTTLE`은 이 체크아웃 전체에서
`collision_detection/collision_common.cpp:60` 한 곳에만 있고 `print`와 무관하다. 실제 사유(호출자
0건인 디버그 프린터)로 교체했다.

## 6. 표

증거 칸의 `파일:줄`은 감사 문장의 시작 줄이다. `—`는 그런 문장이 없다는
뜻이다.

| 상류 파일 | 판정 | 인용한 `.rs` | 감사 증거 |
|---|---|---|---|
| `moveit_core/collision_detection/include/moveit/collision_detection/allvalid/collision_env_allvalid.hpp` | none | `crates/moveit-collision/src/all_valid.rs` | — |
| `moveit_core/collision_detection/include/moveit/collision_detection/collision_common.hpp` | none | `crates/moveit-collision/src/common.rs`, `crates/moveit-collision/src/lib.rs` | — |
| `moveit_core/collision_detection/include/moveit/collision_detection/collision_env.hpp` | none | `crates/moveit-collision/src/env.rs`, `crates/moveit-collision/src/lib.rs` | — |
| `moveit_core/collision_detection/include/moveit/collision_detection/collision_matrix.hpp` | audited | `crates/moveit-collision/src/lib.rs`, `crates/moveit-collision/src/matrix.rs` | `crates/moveit-collision/src/matrix.rs:13` |
| `moveit_core/collision_detection/include/moveit/collision_detection/collision_octomap_filter.hpp` | none | `crates/moveit-collision/src/lib.rs`, `crates/moveit-collision/src/octomap_filter.rs` | — |
| `moveit_core/collision_detection/include/moveit/collision_detection/world.hpp` | audited | `crates/moveit-collision/src/lib.rs`, `crates/moveit-collision/src/world.rs` | `crates/moveit-collision/src/world.rs:119` |
| `moveit_core/collision_detection/include/moveit/collision_detection/world_diff.hpp` | audited | `crates/moveit-scene/src/lib.rs`, `crates/moveit-scene/src/world_diff.rs` | `crates/moveit-scene/src/lib.rs:60` |
| `moveit_core/collision_detection/src/allvalid/collision_env_allvalid.cpp` | none | `crates/moveit-collision/src/all_valid.rs` | — |
| `moveit_core/collision_detection/src/collision_common.cpp` | none | `crates/moveit-collision/src/common.rs`, `crates/moveit-collision/src/lib.rs` | — |
| `moveit_core/collision_detection/src/collision_env.cpp` | none | `crates/moveit-collision/src/env.rs`, `crates/moveit-collision/src/lib.rs` | — |
| `moveit_core/collision_detection/src/collision_matrix.cpp` | audited | `crates/moveit-collision/src/lib.rs`, `crates/moveit-collision/src/matrix.rs` | `crates/moveit-collision/src/matrix.rs:13` |
| `moveit_core/collision_detection/src/collision_octomap_filter.cpp` | none | `crates/moveit-collision/src/lib.rs`, `crates/moveit-collision/src/octomap_filter.rs` | — |
| `moveit_core/collision_detection/src/collision_tools.cpp` | none | `crates/moveit-collision/src/lib.rs`, `crates/moveit-collision/src/tools.rs` | — |
| `moveit_core/collision_detection/src/world.cpp` | audited | `crates/moveit-collision/src/lib.rs`, `crates/moveit-collision/src/world.rs` | `crates/moveit-collision/src/world.rs:119` |
| `moveit_core/collision_detection/src/world_diff.cpp` | audited | `crates/moveit-scene/src/lib.rs`, `crates/moveit-scene/src/world_diff.rs` | `crates/moveit-scene/src/lib.rs:60` |
| `moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_common_distance_field.hpp` | audited | `crates/moveit-distance-field/src/collision_common_distance_field.rs` | `crates/moveit-distance-field/src/lib.rs:511` |
| `moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_distance_field_types.hpp` | audited | `crates/moveit-distance-field/src/collision_distance_field_types.rs`, `crates/moveit-distance-field/src/lib.rs` | `crates/moveit-distance-field/src/lib.rs:511` |
| `moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_env_distance_field.hpp` | audited | `crates/moveit-distance-field/src/collision_env_distance_field.rs` | `crates/moveit-distance-field/src/lib.rs:511` |
| `moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_env_hybrid.hpp` | audited | `crates/moveit-distance-field/src/collision_env_hybrid.rs` | `crates/moveit-distance-field/src/lib.rs:511` |
| `moveit_core/collision_distance_field/src/collision_common_distance_field.cpp` | audited | `crates/moveit-distance-field/src/collision_common_distance_field.rs` | `crates/moveit-distance-field/src/lib.rs:511` |
| `moveit_core/collision_distance_field/src/collision_distance_field_types.cpp` | audited | `crates/moveit-distance-field/src/collision_distance_field_types.rs`, `crates/moveit-distance-field/src/lib.rs` | `crates/moveit-distance-field/src/lib.rs:511` |
| `moveit_core/collision_distance_field/src/collision_env_distance_field.cpp` | audited | `crates/moveit-distance-field/src/collision_env_distance_field.rs` | `crates/moveit-distance-field/src/lib.rs:511` |
| `moveit_core/collision_distance_field/src/collision_env_hybrid.cpp` | audited | `crates/moveit-distance-field/src/collision_env_hybrid.rs` | `crates/moveit-distance-field/src/lib.rs:511` |
| `moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler.hpp` | audited | `crates/moveit-constraints/src/lib.rs`, `crates/moveit-constraints/src/sampler.rs` | `crates/moveit-constraints/src/lib.rs:312` |
| `moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler_manager.hpp` | audited | `crates/moveit-constraints/src/constraint_sampler_manager.rs`, `crates/moveit-constraints/src/lib.rs` | `crates/moveit-constraints/src/lib.rs:312` |
| `moveit_core/constraint_samplers/include/moveit/constraint_samplers/default_constraint_samplers.hpp` | audited | `crates/moveit-constraints/src/ik_sampler.rs`, `crates/moveit-constraints/src/lib.rs`, `crates/moveit-constraints/src/sampler.rs` | `crates/moveit-constraints/src/lib.rs:312` |
| `moveit_core/constraint_samplers/include/moveit/constraint_samplers/union_constraint_sampler.hpp` | audited | `crates/moveit-constraints/src/lib.rs`, `crates/moveit-constraints/src/sampler.rs` | `crates/moveit-constraints/src/lib.rs:312` |
| `moveit_core/constraint_samplers/src/constraint_sampler_manager.cpp` | audited | `crates/moveit-constraints/src/constraint_sampler_manager.rs`, `crates/moveit-constraints/src/lib.rs` | `crates/moveit-constraints/src/lib.rs:312` |
| `moveit_core/constraint_samplers/src/default_constraint_samplers.cpp` | audited | `crates/moveit-constraints/src/ik_sampler.rs`, `crates/moveit-constraints/src/lib.rs`, `crates/moveit-constraints/src/sampler.rs` | `crates/moveit-constraints/src/lib.rs:312` |
| `moveit_core/constraint_samplers/src/union_constraint_sampler.cpp` | audited | `crates/moveit-constraints/src/lib.rs`, `crates/moveit-constraints/src/sampler.rs` | `crates/moveit-constraints/src/lib.rs:312` |
| `moveit_core/distance_field/include/moveit/distance_field/distance_field.hpp` | audited | `crates/moveit-distance-field/src/distance_field.rs`, `crates/moveit-distance-field/src/lib.rs` | `crates/moveit-distance-field/src/lib.rs:858` |
| `moveit_core/distance_field/include/moveit/distance_field/find_internal_points.hpp` | audited | `crates/moveit-distance-field/src/find_internal_points.rs`, `crates/moveit-distance-field/src/lib.rs` | `crates/moveit-distance-field/src/lib.rs:858` |
| `moveit_core/distance_field/include/moveit/distance_field/propagation_distance_field.hpp` | audited | `crates/moveit-distance-field/src/lib.rs`, `crates/moveit-distance-field/src/propagation.rs` | `crates/moveit-distance-field/src/lib.rs:858` |
| `moveit_core/distance_field/include/moveit/distance_field/voxel_grid.hpp` | audited | `crates/moveit-distance-field/src/lib.rs`, `crates/moveit-distance-field/src/voxel_grid.rs` | `crates/moveit-distance-field/src/lib.rs:858` |
| `moveit_core/distance_field/src/distance_field.cpp` | audited | `crates/moveit-distance-field/src/distance_field.rs`, `crates/moveit-distance-field/src/lib.rs` | `crates/moveit-distance-field/src/lib.rs:858` |
| `moveit_core/distance_field/src/find_internal_points.cpp` | audited | `crates/moveit-distance-field/src/find_internal_points.rs`, `crates/moveit-distance-field/src/lib.rs` | `crates/moveit-distance-field/src/lib.rs:858` |
| `moveit_core/distance_field/src/propagation_distance_field.cpp` | audited | `crates/moveit-distance-field/src/lib.rs`, `crates/moveit-distance-field/src/propagation.rs` | `crates/moveit-distance-field/src/lib.rs:858` |
| `moveit_core/dynamics_solver/include/moveit/dynamics_solver/dynamics_solver.hpp` | none | `crates/moveit-state/src/dynamics.rs`, `crates/moveit-state/src/lib.rs` | — |
| `moveit_core/dynamics_solver/src/dynamics_solver.cpp` | none | `crates/moveit-state/src/dynamics.rs`, `crates/moveit-state/src/lib.rs` | — |
| `moveit_core/exceptions/include/moveit/exceptions/exceptions.hpp` | none | `crates/moveit-error/src/lib.rs` | — |
| `moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp` | audited | `crates/moveit-constraints/src/joint.rs`, `crates/moveit-constraints/src/lib.rs`, `crates/moveit-constraints/src/orientation.rs`, `crates/moveit-constraints/src/position.rs`, `crates/moveit-constraints/src/set.rs`, `crates/moveit-constraints/src/visibility.rs` | `crates/moveit-constraints/src/lib.rs:119` |
| `moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/utils.hpp` | audited | `crates/moveit-constraints/src/utils.rs` | `crates/moveit-constraints/src/lib.rs:119` |
| `moveit_core/kinematic_constraints/src/kinematic_constraint.cpp` | audited | `crates/moveit-constraints/src/joint.rs`, `crates/moveit-constraints/src/lib.rs`, `crates/moveit-constraints/src/orientation.rs`, `crates/moveit-constraints/src/position.rs`, `crates/moveit-constraints/src/set.rs`, `crates/moveit-constraints/src/visibility.rs` | `crates/moveit-constraints/src/lib.rs:119` |
| `moveit_core/kinematic_constraints/src/utils.cpp` | audited | `crates/moveit-constraints/src/utils.rs`, `crates/moveit-planning/src/request_adapters/resolve_constraint_frames.rs` | `crates/moveit-constraints/src/lib.rs:119` |
| `moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.hpp` | audited | `crates/moveit-kinematics/src/lib.rs`, `crates/moveit-kinematics/src/registry.rs` | `crates/moveit-kinematics/src/lib.rs:94` |
| `moveit_core/kinematics_base/src/kinematics_base.cpp` | audited | `crates/moveit-kinematics/src/lib.rs` | `crates/moveit-kinematics/src/lib.rs:94` |
| `moveit_core/kinematics_metrics/include/moveit/kinematics_metrics/kinematics_metrics.hpp` | audited | `crates/moveit-metrics/src/lib.rs` | `crates/moveit-metrics/src/lib.rs:14` |
| `moveit_core/kinematics_metrics/src/kinematics_metrics.cpp` | audited | `crates/moveit-metrics/src/lib.rs` | `crates/moveit-metrics/src/lib.rs:14` |
| `moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/acceleration_filter.hpp` | audited | `crates/moveit-smoothing/src/acceleration_filter.rs` | `crates/moveit-smoothing/src/lib.rs:14` |
| `moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/butterworth_filter.hpp` | audited | `crates/moveit-smoothing/src/butterworth.rs`, `crates/moveit-smoothing/src/lib.rs` | `crates/moveit-smoothing/src/lib.rs:14` |
| `moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/ruckig_filter.hpp` | audited | `crates/moveit-smoothing/src/ruckig_filter.rs` | `crates/moveit-smoothing/src/lib.rs:14` |
| `moveit_core/online_signal_smoothing/src/acceleration_filter.cpp` | audited | `crates/moveit-smoothing/src/acceleration_filter.rs` | `crates/moveit-smoothing/src/lib.rs:14` |
| `moveit_core/online_signal_smoothing/src/butterworth_filter.cpp` | audited | `crates/moveit-smoothing/src/butterworth.rs`, `crates/moveit-smoothing/src/lib.rs` | `crates/moveit-smoothing/src/lib.rs:14` |
| `moveit_core/online_signal_smoothing/src/ruckig_filter.cpp` | audited | `crates/moveit-smoothing/src/ruckig_filter.rs` | `crates/moveit-smoothing/src/lib.rs:14` |
| `moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.hpp` | audited | `crates/moveit-scene/src/lib.rs`, `crates/moveit-scene/src/scene.rs` | `crates/moveit-scene/src/scene.rs:50` |
| `moveit_core/planning_scene/src/planning_scene.cpp` | audited | `crates/moveit-scene/src/lib.rs`, `crates/moveit-scene/src/scene.rs`, `ros/moveit-ros/src/scene/attached.rs`, `ros/moveit-ros/src/scene/collision_object.rs`, `ros/moveit-ros/src/scene/planning_scene.rs` | `crates/moveit-scene/src/scene.rs:50` |
| `moveit_core/robot_model/include/moveit/robot_model/aabb.hpp` | none | `crates/moveit-model/src/aabb.rs` | — |
| `moveit_core/robot_model/include/moveit/robot_model/fixed_joint_model.hpp` | none | `crates/moveit-model/src/joint/fixed.rs`, `crates/moveit-model/src/joint/mod.rs`, `crates/moveit-model/src/lib.rs` | — |
| `moveit_core/robot_model/include/moveit/robot_model/floating_joint_model.hpp` | none | `crates/moveit-model/src/joint/floating.rs`, `crates/moveit-model/src/joint/mod.rs`, `crates/moveit-model/src/lib.rs` | — |
| `moveit_core/robot_model/include/moveit/robot_model/joint_model.hpp` | none | `crates/moveit-model/src/joint/bounds.rs`, `crates/moveit-model/src/joint/mod.rs`, `crates/moveit-model/src/joint/model.rs`, `crates/moveit-model/src/lib.rs` | — |
| `moveit_core/robot_model/include/moveit/robot_model/joint_model_group.hpp` | none | `crates/moveit-model/src/joint_model_group.rs` | — |
| `moveit_core/robot_model/include/moveit/robot_model/link_model.hpp` | none | `crates/moveit-model/src/link_model.rs` | — |
| `moveit_core/robot_model/include/moveit/robot_model/planar_joint_model.hpp` | none | `crates/moveit-model/src/joint/mod.rs`, `crates/moveit-model/src/joint/planar.rs`, `crates/moveit-model/src/lib.rs` | — |
| `moveit_core/robot_model/include/moveit/robot_model/prismatic_joint_model.hpp` | none | `crates/moveit-model/src/joint/mod.rs`, `crates/moveit-model/src/joint/prismatic.rs`, `crates/moveit-model/src/lib.rs` | — |
| `moveit_core/robot_model/include/moveit/robot_model/revolute_joint_model.hpp` | none | `crates/moveit-model/src/joint/mod.rs`, `crates/moveit-model/src/joint/revolute.rs`, `crates/moveit-model/src/lib.rs` | — |
| `moveit_core/robot_model/include/moveit/robot_model/robot_model.hpp` | none | `crates/moveit-model/src/robot_model.rs` | — |
| `moveit_core/robot_model/src/aabb.cpp` | none | `crates/moveit-model/src/aabb.rs` | — |
| `moveit_core/robot_model/src/fixed_joint_model.cpp` | none | `crates/moveit-model/src/joint/fixed.rs` | — |
| `moveit_core/robot_model/src/floating_joint_model.cpp` | none | `crates/moveit-model/src/joint/floating.rs` | — |
| `moveit_core/robot_model/src/joint_model.cpp` | none | `crates/moveit-model/src/joint/model.rs` | — |
| `moveit_core/robot_model/src/joint_model_group.cpp` | none | `crates/moveit-model/src/joint_model_group.rs`, `crates/moveit-model/src/robot_model.rs` | — |
| `moveit_core/robot_model/src/link_model.cpp` | none | `crates/moveit-model/src/link_model.rs` | — |
| `moveit_core/robot_model/src/planar_joint_model.cpp` | none | `crates/moveit-model/src/joint/planar.rs` | — |
| `moveit_core/robot_model/src/prismatic_joint_model.cpp` | none | `crates/moveit-model/src/joint/prismatic.rs` | — |
| `moveit_core/robot_model/src/revolute_joint_model.cpp` | none | `crates/moveit-model/src/joint/revolute.rs` | — |
| `moveit_core/robot_model/src/robot_model.cpp` | none | `crates/moveit-geometry/src/stl.rs`, `crates/moveit-model/src/diagnostic.rs`, `crates/moveit-model/src/joint/urdf.rs`, `crates/moveit-model/src/robot_model.rs` | — |
| `moveit_core/robot_state/include/moveit/robot_state/cartesian_interpolator.hpp` | audited | `crates/moveit-kinematics/src/cartesian_interpolator.rs`, `crates/moveit-kinematics/src/lib.rs` | `crates/moveit-kinematics/src/cartesian_interpolator.rs:142` |
| `moveit_core/robot_state/include/moveit/robot_state/conversions.hpp` | audited | `crates/moveit-state/src/conversions.rs`, `crates/moveit-state/src/lib.rs` | `doc/port-coverage.md:211` |
| `moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp` | none | `crates/moveit-kinematics/src/lib.rs`, `crates/moveit-kinematics/src/set_from_ik.rs`, `crates/moveit-state/src/lib.rs`, `crates/moveit-state/src/state.rs` | — |
| `moveit_core/robot_state/src/cartesian_interpolator.cpp` | audited | `crates/moveit-kinematics/src/cartesian_interpolator.rs`, `crates/moveit-kinematics/src/lib.rs` | `crates/moveit-kinematics/src/cartesian_interpolator.rs:142` |
| `moveit_core/robot_state/src/conversions.cpp` | audited | `crates/moveit-state/src/conversions.rs`, `crates/moveit-state/src/lib.rs` | `doc/port-coverage.md:211` |
| `moveit_core/robot_state/src/robot_state.cpp` | none | `crates/moveit-kinematics/src/lib.rs`, `crates/moveit-kinematics/src/set_from_ik.rs`, `crates/moveit-state/src/lib.rs`, `crates/moveit-state/src/state.rs` | — |
| `moveit_core/robot_trajectory/include/moveit/robot_trajectory/robot_trajectory.hpp` | audited | `crates/moveit-trajectory/src/lib.rs`, `crates/moveit-trajectory/src/robot_trajectory.rs` | `crates/moveit-trajectory/src/lib.rs:363` |
| `moveit_core/robot_trajectory/src/robot_trajectory.cpp` | audited | `crates/moveit-trajectory/src/lib.rs`, `crates/moveit-trajectory/src/robot_trajectory.rs` | `crates/moveit-trajectory/src/lib.rs:363` |
| `moveit_core/trajectory_processing/include/moveit/trajectory_processing/ruckig_traj_smoothing.hpp` | audited | `crates/moveit-trajectory/src/ruckig_smoothing.rs` | `crates/moveit-trajectory/src/lib.rs:40` |
| `moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_optimal_trajectory_generation.hpp` | audited | `crates/moveit-trajectory/src/lib.rs`, `crates/moveit-trajectory/src/path.rs`, `crates/moveit-trajectory/src/path_segment/mod.rs`, `crates/moveit-trajectory/src/time_optimal_trajectory_generation.rs`, `crates/moveit-trajectory/src/trajectory.rs` | `crates/moveit-trajectory/src/lib.rs:40` |
| `moveit_core/trajectory_processing/include/moveit/trajectory_processing/trajectory_tools.hpp` | audited | `crates/moveit-trajectory/src/trajectory_tools.rs` | `crates/moveit-trajectory/src/lib.rs:40` |
| `moveit_core/trajectory_processing/src/ruckig_traj_smoothing.cpp` | audited | `crates/moveit-trajectory/src/ruckig_smoothing.rs` | `crates/moveit-trajectory/src/lib.rs:40` |
| `moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp` | audited | `crates/moveit-trajectory/src/lib.rs`, `crates/moveit-trajectory/src/numeric.rs`, `crates/moveit-trajectory/src/path.rs`, `crates/moveit-trajectory/src/path_segment/circular.rs`, `crates/moveit-trajectory/src/path_segment/linear.rs`, `crates/moveit-trajectory/src/path_segment/mod.rs`, `crates/moveit-trajectory/src/time_optimal_trajectory_generation.rs`, `crates/moveit-trajectory/src/trajectory.rs` | `crates/moveit-trajectory/src/lib.rs:40` |
| `moveit_core/trajectory_processing/src/trajectory_tools.cpp` | audited | `crates/moveit-trajectory/src/trajectory_tools.rs` | `crates/moveit-trajectory/src/lib.rs:40` |
| `moveit_core/transforms/include/moveit/transforms/transforms.hpp` | none | `crates/moveit-geometry/src/lib.rs`, `crates/moveit-geometry/src/transforms.rs` | — |
| `moveit_core/transforms/src/transforms.cpp` | none | `crates/moveit-geometry/src/lib.rs`, `crates/moveit-geometry/src/transforms.rs` | — |
| `moveit_core/utils/include/moveit/utils/moveit_error_code.hpp` | none | `crates/moveit-error/src/lib.rs` | — |
| `moveit_core/utils/src/message_checks.cpp` | none | `ros/moveit-ros/src/scene/collision_object.rs` | — |
| `moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin-inl.hpp` | audited | `crates/moveit-kinematics/src/cached_solver.rs` | `crates/moveit-kinematics/src/lib.rs:256` |
| `moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin.hpp` | audited | `crates/moveit-kinematics/src/cached_solver.rs`, `crates/moveit-kinematics/src/ik_cache.rs` | `crates/moveit-kinematics/src/lib.rs:256` |
| `moveit_kinematics/cached_ik_kinematics_plugin/src/ik_cache.cpp` | audited | `crates/moveit-kinematics/src/ik_cache.rs`, `crates/moveit-kinematics/src/ik_cache/format.rs` | `crates/moveit-kinematics/src/lib.rs:256` |
| `moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/joint_mimic.hpp` | none | `crates/moveit-kinematics/src/lib.rs` | — |
| `moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.hpp` | audited | `crates/moveit-kinematics/src/lib.rs`, `crates/moveit-kinematics/src/newton_raphson.rs` | `crates/moveit-kinematics/src/lib.rs:94` |
| `moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp` | audited | `crates/moveit-kinematics/src/cart_to_jnt.rs`, `crates/moveit-kinematics/src/chain.rs`, `crates/moveit-kinematics/src/lib.rs`, `crates/moveit-kinematics/src/newton_raphson.rs` | `crates/moveit-kinematics/src/lib.rs:94` |
| `moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_cost.hpp` | audited | `crates/moveit-planners-chomp/src/cost.rs`, `crates/moveit-planners-chomp/src/lib.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_optimizer.hpp` | audited | `crates/moveit-planners-chomp/src/lib.rs`, `crates/moveit-planners-chomp/src/optimizer.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_parameters.hpp` | audited | `crates/moveit-planners-chomp/src/lib.rs`, `crates/moveit-planners-chomp/src/parameters.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_planner.hpp` | audited | `crates/moveit-planners-chomp/src/planner.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_trajectory.hpp` | audited | `crates/moveit-planners-chomp/src/lib.rs`, `crates/moveit-planners-chomp/src/trajectory.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_utils.hpp` | audited | `crates/moveit-planners-chomp/src/lib.rs`, `crates/moveit-planners-chomp/src/utils.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.hpp` | audited | `crates/moveit-planners-chomp/src/lib.rs`, `crates/moveit-sampling/src/lib.rs`, `crates/moveit-sampling/src/multivariate_gaussian.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/src/chomp_cost.cpp` | audited | `crates/moveit-planners-chomp/src/cost.rs`, `crates/moveit-planners-chomp/src/lib.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp` | audited | `crates/moveit-planners-chomp/src/lib.rs`, `crates/moveit-planners-chomp/src/optimizer.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/src/chomp_parameters.cpp` | audited | `crates/moveit-planners-chomp/src/lib.rs`, `crates/moveit-planners-chomp/src/parameters.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/src/chomp_planner.cpp` | audited | `crates/moveit-planners-chomp/src/planner.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/chomp/chomp_motion_planner/src/chomp_trajectory.cpp` | audited | `crates/moveit-planners-chomp/src/lib.rs`, `crates/moveit-planners-chomp/src/trajectory.rs` | `crates/moveit-planners-chomp/src/lib.rs:99` |
| `moveit_planners/pilz_industrial_motion_planner/include/joint_limits_copy/joint_limits.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/limits.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/cartesian_trajectory.hpp` | none | `crates/moveit-planners-pilz/src/cartesian_trajectory.rs`, `crates/moveit-planners-pilz/src/lib.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/cartesian_trajectory_point.hpp` | none | `crates/moveit-planners-pilz/src/cartesian_trajectory.rs`, `crates/moveit-planners-pilz/src/lib.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/command_list_manager.hpp` | none | `crates/moveit-planners-pilz/src/command_list_manager.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/joint_limits_aggregator.hpp` | none | `crates/moveit-planners-pilz/src/joint_limits_aggregator.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/joint_limits_container.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/limits.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/joint_limits_extension.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/limits.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/joint_limits_validator.hpp` | none | `crates/moveit-planners-pilz/src/joint_limits_validator.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/limits_container.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/limits.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/path_circle_generator.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/path_circle.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/path_polyline_generator.hpp` | none | `crates/moveit-planners-pilz/src/path_polyline_generator.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/plan_components_builder.hpp` | none | `crates/moveit-planners-pilz/src/plan_components_builder.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_blend_request.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_blender_transition_window.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_blend_response.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_blender_transition_window.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_blender.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_blender_transition_window.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_blender_transition_window.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_functions.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_functions.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_generator.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator_circ.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_generator_circ.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator_lin.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_generator_lin.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator_polyline.hpp` | none | `crates/moveit-planners-pilz/src/trajectory_generator_polyline.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator_ptp.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_generator_ptp.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/velocity_profile_atrap.hpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/velocity_profile.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/command_list_manager.cpp` | none | `crates/moveit-planners-pilz/src/command_list_manager.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/joint_limits_aggregator.cpp` | none | `crates/moveit-planners-pilz/src/joint_limits_aggregator.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/joint_limits_container.cpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/limits.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/joint_limits_validator.cpp` | none | `crates/moveit-planners-pilz/src/joint_limits_validator.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/limits_container.cpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/limits.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/path_circle_generator.cpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/path_circle.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/path_polyline_generator.cpp` | none | `crates/moveit-planners-pilz/src/path_polyline_generator.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/plan_components_builder.cpp` | none | `crates/moveit-planners-pilz/src/plan_components_builder.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/trajectory_blender_transition_window.cpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_blender_transition_window.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/trajectory_functions.cpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_functions.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator.cpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_generator.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_circ.cpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_generator_circ.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_lin.cpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_generator_lin.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_polyline.cpp` | none | `crates/moveit-planners-pilz/src/trajectory_generator_polyline.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_ptp.cpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/trajectory_generator_ptp.rs` | — |
| `moveit_planners/pilz_industrial_motion_planner/src/velocity_profile_atrap.cpp` | none | `crates/moveit-planners-pilz/src/lib.rs`, `crates/moveit-planners-pilz/src/velocity_profile.rs` | — |
| `moveit_planners/stomp/include/stomp_moveit/conversion_functions.hpp` | none | `crates/moveit-planners-stomp/src/conversion_functions.rs`, `crates/moveit-planners-stomp/src/lib.rs` | — |
| `moveit_planners/stomp/include/stomp_moveit/cost_functions.hpp` | none | `crates/moveit-planners-stomp/src/cost_functions.rs`, `crates/moveit-planners-stomp/src/lib.rs` | — |
| `moveit_planners/stomp/include/stomp_moveit/filter_functions.hpp` | none | `crates/moveit-planners-stomp/src/filter_functions.rs`, `crates/moveit-planners-stomp/src/lib.rs` | — |
| `moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp` | none | `crates/moveit-sampling/src/lib.rs`, `crates/moveit-sampling/src/multivariate_gaussian.rs` | — |
| `moveit_planners/stomp/include/stomp_moveit/noise_generators.hpp` | none | `crates/moveit-planners-stomp/src/lib.rs`, `crates/moveit-planners-stomp/src/noise_generators.rs` | — |
| `moveit_planners/stomp/include/stomp_moveit/stomp_moveit_task.hpp` | none | `crates/moveit-planners-stomp/src/composable_task.rs`, `crates/moveit-planners-stomp/src/lib.rs` | — |
| `moveit_planners/stomp/src/stomp_moveit_planning_context.cpp` | none | `crates/moveit-planners-stomp/src/lib.rs`, `crates/moveit-planners-stomp/src/planner.rs` | — |

