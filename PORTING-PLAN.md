# moveit-rs — MoveIt 2 Rust 포팅 계획

- **상류 기준점:** `/home/stevek/work/moveit2` @ `e017c91ee12984393a28ba246075c65f69cde3bf`
  (2026-08-02, `main` = ROS 2 rolling 타깃)
- **작성일:** 2026-08-03
- **라이선스:** MoveIt 2는 BSD-3-Clause. 포팅한 모든 파일은 원본 저작권 헤더를
  유지하고, 대응하는 상류 경로를 파일 상단에 명시한다.

---

## 0. 확정된 결정 (사용자 승인, 2026-08-03)

| # | 결정 | 선택 |
|---|---|---|
| D1 | 최종 형태 | **ROS 독립 Rust 모션플래닝 라이브러리** |
| D2 | ROS 2 바인딩 | **r2r 0.9.5** — 단, 선택적 `moveit-ros` 크레이트에 격리 |
| D3 | OMPL | **순수 Rust 플래너 우선**, cxx FFI는 후순위 |
| D4 | 플러그인 모델 | **컴파일타임 레지스트리** (trait + `linkme`) |

D1과 D2의 조합에 대한 해석 — 이 계획서 전체가 이 해석 위에 서 있다:

> **코어 크레이트는 ROS 타입을 일절 참조하지 않는다.** `moveit_msgs`,
> `geometry_msgs`, `rclcpp`에 해당하는 것은 코어 안에서 순수 Rust 타입으로
> 새로 정의한다. ROS 2 연동은 **선택적** `moveit-ros` 크레이트 하나에만 존재하며,
> 그 크레이트만 r2r에 의존하고 `From`/`Into` 변환을 제공한다. 코어는 ROS 2
> 설치 없이 `cargo test`로 전부 검증된다.

이 해석이 틀렸다면 Phase 0 착수 전에 말씀해 주십시오. Phase 1 이후에 뒤집으면
전 크레이트의 타입 시그니처가 바뀝니다.

---

## 1. 실측 규모

`moveit2` HEAD 기준 실측 (`*.cpp`, `*.hpp`, `*.h`, `*.cc`):

| 패키지 | 파일 | LOC | 이 계획의 처분 |
|---|---:|---:|---|
| moveit_ros | 413 | 77,463 | **범위 밖** (선택적 `moveit-ros`가 일부만 커버) |
| moveit_core | 292 | 70,215 | **주 포팅 대상** |
| moveit_planners | 307 | 68,409 | 부분 (아래 분해 참조) |
| moveit_setup_assistant | 194 | 27,714 | **범위 밖** (Qt GUI) |
| moveit_kinematics | 28 | 6,912 | **주 포팅 대상** |
| moveit_py | 43 | 5,368 | **범위 밖** (PyO3 재작성은 별건) |
| moveit_plugins | 19 | 2,990 | **범위 밖** (ros2_control 결합) |
| **합계** | **1,296** | **259,071** | |

`moveit_core` 서브디렉터리별 LOC — 포팅 순서의 근거:

```
robot_state            9,202     collision_detection_bullet  4,278   [드롭]
robot_model            7,909     trajectory_processing       3,614
collision_detection    7,031     collision_detection_fcl     3,037   [parry로 대체]
constraint_samplers    6,008     robot_trajectory            2,209
kinematic_constraints  4,996     utils                       1,953
collision_distance_field 4,775   online_signal_smoothing     1,878
planning_scene         4,593     planning_interface            976
distance_field         4,541     kinematics_base               873
                                 dynamics_solver               533
                                 transforms                    524
                                 kinematics_metrics            474
                                 macros                        331
                                 controller_manager            264   [범위 밖]
                                 exceptions                    164
```

`moveit_planners` 분해:

```
pilz_industrial_motion_planner       23,956   Phase 8 (해석적 LIN/PTP/CIRC — 이식성 높음)
test_configs                         21,898   범위 밖 (테스트 리소스)
ompl                                 11,799   Phase 7에서 네이티브 Rust로 대체
pilz_..._testutils                    4,656   범위 밖
chomp                                 3,771   Phase 8
stomp                                 2,329   Phase 8
```

### 1.1 외부 네이티브 의존 실측

`#include <...>` 헤더 출현 횟수:

```
rclcpp 565   pluginlib 99   ompl 80   boost 71   tf2_ros 69
geometric_shapes 68   tf2_eigen 63   Eigen 61   kdl 53   tf2 44
tf2_geometry_msgs 32   urdf_parser 21   rclcpp_action 15   fcl 15
octomap 13   urdf 10   srdfdom 7   tf2_kdl 5   tf2_eigen_kdl 5
kdl_parser 5   ruckig 2   bullet 2   btBulletCollisionCommon 1
```

### 1.2 ROS 결합도 — 디커플링 작업량의 실체

`moveit_core` 292개 파일 중 **74개가 ROS 메시지 타입을 참조**하고 **95개가
`rclcpp`를 직접 참조**한다. `moveit_core`는 오늘 ROS-free가 아니다. D1은 단순
언어 변환이 아니라 **디커플링 + 언어 변환**이다. 디커플링 밀도가 높은 곳:

```
constraint_samplers   14/22 파일     robot_state          9/17
trajectory_processing  8/14          kinematic_constraints 6/8
collision_detection    6/42          utils                5/20
planning_interface     5/12          planning_scene       4/6
```

`constraint_samplers`와 `kinematic_constraints`가 최악이다. 이 둘은
`moveit_msgs::Constraints` 계열 타입을 로직 안에서 직접 소비한다. 코어의 순수
Rust 제약 타입 설계(§4.3)가 Phase 5의 실제 난이도를 결정한다.

---

## 2. Rust 생태계 현황 (2026-08-03 `cargo search` 실측)

| 용도 | C++ | Rust 대체 | 버전 | 판정 |
|---|---|---|---|---|
| 선형대수 | Eigen | `nalgebra` | 0.35.0 | 채택. `Isometry3<f64>` ↔ `Eigen::Isometry3d` |
| 충돌 검사 | FCL / Bullet | `parry3d-f64` | 0.30.0 | 채택. 거리·접촉·연속충돌 모두 보유 |
| URDF 파싱 | urdfdom | `urdf-rs` | 0.9.0 | 채택 |
| **SRDF 파싱** | srdfdom | **없음** | — | **직접 작성** (~800 LOC) |
| 기구학 체인 | KDL | `k` | 0.32.0 | **미채택** — 사유 §4.2 |
| 저크 제한 스무딩 | ruckig | `rsruckig` | 3.0.0 | 평가 후 채택 (§4.6) |
| Octree 점유맵 | octomap | `bye_octomap_rs` | 0.1.1 | **성숙도 미달** — 자체 구현 검토 |
| QP 솔버 | OSQP | `osqp` | 1.0.1 | 채택 (STOMP/pilz용) |
| **샘플링 플래너** | OMPL | **없음** | — | **직접 작성** (D3) |
| 참고: RRT | — | `rrt` | 0.7.0 | 참고용. dual-RRT-connect만 |
| 참고: 통합 프레임워크 | — | `openrr-planner` | 0.1.0 | 참고용. 0.1.0, 규모 미달 |
| ROS 2 (선택) | rclcpp | `r2r` | 0.9.5 | D2 |

툴체인: `cargo 1.97.0` / `rustc 1.97.0` 확인됨.

**핵심 공백 3개:** SRDF 파서, OMPL 대응물, 성숙한 octomap. 이 셋은 계획에
직접 구현으로 반영되어 있다.

---

## 3. 크레이트 레이아웃

```
moveit-rs/
├── Cargo.toml                    # workspace, resolver = "3"
├── crates/
│   ├── moveit-error/             # exceptions(164) → thiserror 계층
│   ├── moveit-geometry/          # geometric_shapes 대응 + transforms(524)
│   ├── moveit-srdf/              # 신규 SRDF 파서
│   ├── moveit-model/             # robot_model(7,909) + URDF/SRDF 로딩
│   ├── moveit-state/             # robot_state(9,202) + metrics(474) + dynamics(533)
│   ├── moveit-collision/         # collision_detection(7,031) + parry 백엔드
│   ├── moveit-distance-field/    # distance_field(4,541) + collision_distance_field(4,775)
│   ├── moveit-scene/             # planning_scene(4,593)
│   ├── moveit-constraints/       # kinematic_constraints(4,996) + constraint_samplers(6,008)
│   ├── moveit-trajectory/        # robot_trajectory(2,209) + trajectory_processing(3,614)
│   ├── moveit-smoothing/         # online_signal_smoothing(1,878)
│   ├── moveit-kinematics/        # kinematics_base(873) + IK 솔버(6,912)
│   ├── moveit-planning/          # planning_interface(976) + PlannerPlugin 레지스트리
│   ├── moveit-planners-sbp/      # 신규: RRT-Connect / PRM / RRT* / KPIECE
│   ├── moveit-planners-chomp/    # chomp(3,771)
│   ├── moveit-planners-stomp/    # stomp(2,329)
│   ├── moveit-planners-pilz/     # pilz(23,956)
│   └── moveit-ros/               # [선택] r2r 0.9.5, 이 크레이트만 ROS 의존
└── tools/
    ├── moveit-oracle/            # C++ 측 차등 테스트 오라클 (CMake, moveit2 링크)
    └── moveit-diff/              # Rust 측 차등 테스트 러너
```

**의존 방향 규칙 (CI로 강제):** `moveit-ros`를 제외한 어떤 크레이트도
`r2r`, `rclrs`, `ros2-client`에 의존하지 않는다. `cargo tree` 기반 CI 검사로
위반 시 빌드 실패시킨다.

---

## 4. 설계 결정 — 사인오프가 필요한 항목

### 4.1 `RobotState`의 dirty-flag 캐시 → 명시적 상태로 대체

C++ `robot_state.h`는 `dirty_link_transforms_`, `dirty_collision_body_transforms_`,
`dirty_joint_transforms_` 포인터 플래그와 값 배열을 병행 유지한다. 값의 의미가
플래그에 따라 달라지는 구조 — Rust로 그대로 옮기면 같은 종류의 버그를 그대로
상속한다.

**제안:** 변환 캐시를 `enum TransformCache { Stale, Fresh(Isometry3<f64>) }` 또는
`Option`이 아닌 전용 sum type으로 모델링하고, 무효화는 단일 소유자
(`RobotState::mark_dirty_from(joint_idx)`)만 수행한다. 읽기 경로는
`&mut self`를 요구해 lazy 갱신을 타입으로 강제한다.

**대가:** 읽기 API가 `&self`에서 `&mut self`로 바뀐다. C++ API 형태와
어긋나며, 동시 읽기가 필요한 지점에서 호출자가 `update_link_transforms()`를
먼저 부르는 패턴으로 바뀐다. **사인오프 필요.**

### 4.2 `k` 크레이트 미채택 사유

`k 0.32`는 체인 FK/야코비안/수치 IK를 제공하지만 MoveIt의 `RobotModel`이
가진 것을 갖고 있지 않다: JointModelGroup과 서브그룹, mimic joint,
multi-DOF joint(floating/planar), 그룹별 end-effector 정의, SRDF 유래
가상 조인트, 그룹 상태 프리셋. `k` 위에 이것들을 얹는 비용이 `robot_model`
7,909줄을 직접 이식하는 비용보다 크다. `k`는 초기 FK 정확성 교차검증용
참조로만 사용한다.

### 4.3 순수 Rust 제약 타입 (D1의 실제 비용)

`moveit_msgs::Constraints`(JointConstraint / PositionConstraint /
OrientationConstraint / VisibilityConstraint)와 `RobotState`,
`RobotTrajectory`, `PlanningScene`, `MotionPlanRequest`, `MotionPlanResponse`,
`CollisionObject`, `AttachedCollisionObject`에 대응하는 순수 Rust 타입을
`moveit-planning`과 `moveit-constraints`에 정의한다.

**설계 원칙:** ROS 메시지의 필드 구조를 그대로 베끼지 않는다. ROS 메시지는
`Option`이 없어 `bool has_x` + `x` 쌍으로 선택성을 표현하는데(§4.1과 같은
이중 의미 문제), Rust 타입에서는 `Option<T>`와 enum으로 표현한다. 변환
손실은 `moveit-ros`의 `TryFrom`이 명시적 에러로 보고한다.

### 4.4 컴파일타임 플러그인 레지스트리 (D4)

pluginlib 클래스 106개 → trait + `linkme::distributed_slice`. 대상 trait:

```rust
trait CollisionDetectorAllocator   // fcl, bullet, distance_field
trait KinematicsSolver             // kdl, cached_ik, srv, ikfast
trait PlannerManager               // ompl, chomp, stomp, pilz
trait PlanningRequestAdapter       // default_request_adapters
trait PlanningResponseAdapter      // default_response_adapters
trait SmoothingFilter              // acceleration, butterworth, ruckig
```

**의미 변화 (사인오프 완료, D4):** 플러그인 추가에 재빌드가 필요하다.
서드파티가 `.so`만 떨궈 넣는 기존 워크플로는 깨진다. `.so` 요구가 실제로
나올 때 feature-gate로 C ABI 로더를 추가하는 경로는 열어 둔다.

### 4.5 충돌 검사 백엔드

FCL과 Bullet 백엔드 2개(3,037 + 4,278 LOC)를 `parry3d-f64` 백엔드 1개로
대체한다. `CollisionDetectorAllocator` trait은 유지하므로 나중에 FCL FFI
백엔드를 추가할 수 있다.

**알려진 차이:** parry와 FCL은 접촉점/법선 산출 알고리즘이 다르다. 차등
테스트는 `(collision: bool)`과 `(distance: f64, 허용오차)`를 비교하고,
접촉점 좌표의 정확한 일치는 검증 항목에서 제외한다. 이 완화는 §6.2에
명시적 검증 한계로 기록한다.

### 4.6 저크 제한 스무딩

`rsruckig 3.0.0`(Ruckig의 Rust 이식)과 `ferromotion-ruckig 0.50.0`
(Community S-curve 이식)이 있다. Phase 6 착수 시 두 크레이트를 상류
`moveit_core/online_signal_smoothing/ruckig_filter.cpp`의 테스트 벡터로
평가하고 채택 여부를 결정한다. 둘 다 미달이면 자체 구현(~1,500 LOC).

### 4.7 명시적 범위 밖 — 영구히 C++로 남는 것

- `moveit_setup_assistant` (27,714) — Qt5/rviz 위젯
- `moveit_ros/visualization` (13,346) — RViz 디스플레이 플러그인
- `moveit_ros/perception` (7,877) — mesh_filter가 OpenGL/GLUT 의존
- `moveit_plugins` (2,990) — ros2_control 컨트롤러 인터페이스
- `moveit_py` (5,368) — 필요 시 PyO3로 별도 재작성

---

## 5. 단계별 계획

각 단계는 **검증 가능한 완료 조건**을 갖는다. 조건을 만족하지 못하면 다음
단계로 넘어가지 않는다.

추정치의 전제: **숙련 Rust + 로보틱스 엔지니어 1인**, 테스트와 차등 검증
하네스 작성을 포함한 순생산성 **300 LOC/일**. Rust LOC는 대응 C++ LOC의
0.8–1.0배로 잡았다.

### Phase 0 — 스캐폴딩과 차등 검증 하네스 (2주, ~2,000 LOC)

1. workspace, CI(fmt / clippy `-D warnings` / nextest), 의존 방향 검사
2. `tools/moveit-oracle`: moveit2를 링크하는 C++ 바이너리. stdin으로 JSON
   시나리오를 받아 FK / 충돌 / IK / 플래닝 결과를 JSON으로 출력
3. `tools/moveit-diff`: 같은 시나리오를 Rust 구현에 먹이고 오라클 출력과 비교
4. `moveit_resources`(panda, prbt, fanuc) 체크아웃 — 테스트 픽스처

**완료 조건:** 오라클이 panda URDF/SRDF에 대해 임의 관절값 1,000세트의 FK를
출력하고, `moveit-diff`가 그것을 읽어 "Rust 구현 없음"으로 1,000건 전부
실패 보고한다. (하네스가 동작함을 실패로 증명)

### Phase 1 — 모델 계층 (6주, ~10,000 LOC)

`moveit-error`, `moveit-geometry`, `moveit-srdf`, `moveit-model`

- SRDF 파서 신규 작성: group, group_state, end_effector, virtual_joint,
  disable_collisions, passive_joint, link_sphere_approximation
- `RobotModel`: JointModel 계층(revolute/prismatic/planar/floating/fixed),
  mimic joint, LinkModel, JointModelGroup, 서브그룹, KinematicChain 해석

**완료 조건:** panda / prbt / fanuc 3종에 대해 링크 수, 조인트 수, 그룹
구성, 조인트 한계값, mimic 관계가 오라클과 완전 일치.

### Phase 2 — 상태 계층 (5주, ~8,000 LOC)

`moveit-state` — FK, 야코비안, 랜덤 상태, 보간, 거리 메트릭, `dynamics_solver`

**완료 조건:**
- 임의 관절값 10,000세트 × 3로봇의 모든 링크 FK가 오라클과 `1e-9` 이내 일치
- 야코비안(6×N)이 `1e-7` 이내 일치 (열 순서 규약 포함)
- 관절 한계 클램핑, mimic 전파, floating/planar 조인트 보간이 일치

### Phase 3 — 충돌 검사 (7주, ~14,000 LOC)

`moveit-collision` (parry 백엔드), `moveit-distance-field`

- `CollisionRequest` / `CollisionResult` / `AllowedCollisionMatrix`
- self-collision, world-collision, 거리 질의, 접촉 열거
- distance_field + collision_distance_field (§2 공백 없음, 직접 이식)

**완료 조건:**
- 10,000 상태 × 3로봇에서 `collision: bool` 이 오라클과 **100% 일치**
- `distance: f64` 가 `1e-4` 이내 일치
- 접촉점 좌표는 비교 대상에서 제외 (§4.5, 검증 한계로 기록)

### Phase 4 — 역기구학 (5주, ~5,000 LOC)

`moveit-kinematics` — `KinematicsSolver` trait + KDL 대응 수치 IK
(Newton-Raphson, LMA), position-only IK, 관절 한계 처리

**완료 조건:** 도달 가능한 목표 자세 5,000개에 대해 (a) 성공률이 C++ KDL
플러그인 이상, (b) 성공한 해의 FK가 목표 자세와 `1e-6` 이내 일치.
IK 해 자체의 일치는 요구하지 않는다 (랜덤 시드 의존).

### Phase 5 — 씬과 제약 (8주, ~13,000 LOC)

`moveit-scene`, `moveit-constraints`

- `PlanningScene`: 월드 오브젝트, 부착 오브젝트, ACM, diff 적용
- Joint / Position / Orientation / Visibility 제약과 그 샘플러
- **§4.3의 순수 Rust 제약 타입 설계가 이 단계의 산출물**

**완료 조건:**
- 제약 조합 2,000건에 대한 `decide()` 결과가 오라클과 100% 일치
- 제약 샘플러가 생성한 상태 10,000개가 전부 자기 제약을 만족 (자체 검증)
- 씬 diff 적용 후 충돌 결과가 오라클과 100% 일치

### Phase 6 — 궤적 (5주, ~6,000 LOC)

`moveit-trajectory`, `moveit-smoothing`

- `RobotTrajectory`, TOTG(time_optimal_trajectory_generation),
  IPTP, ruckig 스무딩(§4.6), acceleration / butterworth 필터

**완료 조건:** 동일 waypoint 입력에 대해 TOTG 산출 시간 파라미터화가
오라클과 `1e-6` 이내 일치. 스무딩 필터는 상류 단위 테스트 벡터로 검증.

### Phase 7 — 플래닝 코어와 네이티브 플래너 (8주, ~7,000 LOC)

`moveit-planning`, `moveit-planners-sbp`

- `PlannerManager` / `PlanningContext` trait, 요청·응답 어댑터 체인
- 네이티브 RRT-Connect, PRM, RRT*, (여력 시) KPIECE
- 상태 공간: `RealVectorStateSpace`, `SE3StateSpace`, 그룹별 조합

**완료 조건 (경로 자체는 비교 불가 — 속성으로 검증):**
- 벤치마크 문제 500건에서 성공률이 C++ OMPL RRTConnect의 90% 이상
- 산출 경로 100%가 `moveit-scene`의 충돌 검사와 제약을 통과
- 경로 길이 중앙값이 C++ OMPL 대비 1.3배 이내

### Phase 8 — 추가 플래너 (14주, ~22,000 LOC)

`moveit-planners-chomp`(3,771), `moveit-planners-stomp`(2,329),
`moveit-planners-pilz`(23,956 — LIN/PTP/CIRC + sequence blending)

pilz는 해석적이라 결과가 결정론적이다 — **차등 테스트에서 궤적을 직접
비교할 수 있는 유일한 플래너.**

**완료 조건:** pilz LIN/PTP/CIRC 궤적이 오라클과 `1e-6` 이내 일치.
CHOMP/STOMP는 Phase 7과 같은 속성 기반 검증.

### Phase 9 — 선택적 ROS 2 연동 (5주, ~6,000 LOC)

`moveit-ros` — r2r 0.9.5. 코어 타입 ↔ `moveit_msgs` `TryFrom` 변환,
`/plan_kinematic_path` 서비스, `/move_action` 액션 서버,
planning scene 토픽 구독.

**완료 조건:** 기존 C++ `MoveGroupInterface` 클라이언트가 코드 변경 없이
`moveit-ros` 노드에 플래닝 요청을 보내 유효한 궤적을 받는다.

---

## 6. 일정과 리스크

### 6.1 합계

| 구간 | 기간 | Rust LOC |
|---|---:|---:|
| Phase 0–7 (사용 가능한 라이브러리) | **46주** | ~65,000 |
| Phase 8 (추가 플래너) | 14주 | ~22,000 |
| Phase 9 (선택적 ROS) | 5주 | ~6,000 |
| **전체** | **65주 ≈ 15개월** | **~93,000** |

1인 기준이다. caucus 병렬 패널로 Phase 3·4·6을 동시 진행하면 wall-clock은
줄지만 Phase 1→2→3의 의존 사슬은 직렬이므로 단축 한계가 있다.

### 6.2 검증의 한계 — 명시

차등 테스트로 **증명되지 않는** 항목:

1. **접촉점/법선 좌표** — parry와 FCL의 알고리즘 차이 (§4.5)
2. **IK 해의 구체적 값** — 랜덤 시드 의존
3. **샘플링 기반 플래너의 경로** — 본질적으로 비결정론적
4. **부동소수점 누적 오차** — 장시간 궤적에서 발산 가능

1–3은 속성 기반 검증으로 대체한다. 4는 미해결이며 실측이 필요하다.

### 6.3 리스크

| 리스크 | 영향 | 완화 |
|---|---|---|
| §4.1 `&mut self` API 전환이 하류 사용성을 해침 | Phase 2 재설계 | Phase 2 착수 전 프로토타입으로 API 형태 확정 |
| §4.3 제약 타입 설계 실패 | Phase 5 전면 재작업 | Phase 1에서 타입 스케치를 먼저 고정 |
| 네이티브 SBP 플래너가 OMPL 성공률에 미달 | Phase 7 완료 조건 미충족 | D3의 후순위 FFI 경로로 폴백 |
| `bye_octomap_rs` 0.1.1 성숙도 미달 | 점유맵 자체 구현 +3주 | Phase 3에서 조기 평가 |
| Eigen↔nalgebra 야코비안 열 순서 규약 불일치 | 무성 오차 | Phase 2 완료 조건에 규약 일치를 명시 (완료) |
| 상류 `main`이 계속 움직임 | 포팅 대상 드리프트 | 기준 SHA `e017c91e` 고정, 갱신은 명시적 리베이스 작업으로 |
| **오라클 하네스 빌드 환경 부재 (실측 확인됨, §7.1)** | **Phase 0 착수 불가** | §7.1의 A/B/C 중 택일 — 사용자 결정 대기 |

---

## 7. Phase 0 — 완료 (2026-08-03)

§7.1의 블로커는 선택지 B로 해소되었다.

### 7.1 오라클 환경 (해소)

Docker 29.7.1 설치 확인. `moveit/moveit2:rolling-ci`(5.14 GB) 위에
`--packages-up-to moveit_core`로 상류 `e017c91e`를 빌드하고 그 위에 오라클을
얹은 `moveit-rs/oracle:latest` 이미지가 동작한다. 호스트에는 ROS 2도 colcon도
설치하지 않았다.

`tools/moveit-oracle/build.sh`는 `git archive`로 컨텍스트를 만든다 — 각 트리를
커밋된 HEAD에 고정하므로 더러운 작업 트리가 참조 구현에 새어 들어갈 수 없고,
`.git`이 데몬으로 전송되지 않는다.

### 7.2 Phase 0 완료 조건 — 충족

> 오라클이 panda URDF/SRDF에 대해 임의 관절값 1,000세트의 FK를 출력하고,
> `moveit-diff`가 그것을 읽어 "Rust 구현 없음"으로 1,000건 전부 실패 보고한다.

실행 결과 (`--cases 1000 --seed 1`):

```
oracle model: panda (12 links, 12 joints, 3 groups)
cases:  1001      passed: 0      failed: 1001
   1x  model_info : moveit-model not implemented (Phase 1)
1000x  fk[0..999] : moveit-state not implemented (Phase 2)
```

fanuc(9 links, 9 joints, 1 group)로도 51/51 동일하게 동작.

### 7.3 Phase 0에서 확정된 설계 — 오라클이 랜덤 상태를 소유한다

초안은 러너가 오라클이 보고한 경계 안에서 변수별로 균등 샘플링했다. 이것이
동시에 세 가지를 틀리게 만든다:

1. floating 조인트의 병진 경계는 무한대다. JSON은 무한대를 표현하지 못하고
   nlohmann은 `null`을 낸다. 게다가 그 변수의 `position_bounded_`는 `true`라서
   "bounded=false면 무한"이라는 규칙도 성립하지 않는다.
2. floating 조인트의 쿼터니언 4성분을 독립 샘플링하면 정규화되지 않는다.
3. mimic 조인트 값은 파생값이지 자유 변수가 아니다.

셋 다 포팅이 아니라 **테스트의 결함**으로 나타난다. 구조적 해결: 오라클이
`RobotModel::getVariableRandomPositions`로 상태를 생성하고 러너는 그 값을 양쪽에
그대로 먹인다. 이 함수가 세 가지를 이미 전부 처리하며, 재현성은 오라클에
넘긴 seed가 보장한다. 러너의 자체 RNG는 삭제했다.

`ModelInfo`의 경계는 무한한 쪽을 `null`로 싣는다 — 비유한 double을 그대로
직렬화하는 대신 "무경계"를 명시적으로 말한다.

### 7.4 남은 것

- `moveit-error`(에러 코드 + 예외), `moveit-geometry`(Transforms) 착수 완료.
  워크스페이스 테스트 14/14 통과.
- prbt 픽스처는 xacro라 확장이 필요하다 (컨테이너에 xacro 있음). Phase 1 준비물.
- `.github/workflows/ci.yml`은 작성했으나 원격이 없어 아직 실행된 적 없다.

### 7.5 다음 단계 — Phase 1 착수 전 사인오프 2건

§4.1 `RobotState` dirty-flag → sum type (읽기 API가 `&self` → `&mut self`),
§4.3 순수 Rust 제약 타입. 둘 다 Phase 1 이후에 뒤집으면 전 크레이트의 타입
시그니처가 바뀐다.
