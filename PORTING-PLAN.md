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
먼저 부르는 패턴으로 바뀐다.

**결정 (2026-08-03) — sum type은 채택, `&mut self` 읽기는 채택하지 않는다.**
§8.2를 따른다.

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

**결정 (2026-08-03) — 위 설계 원칙 그대로 채택.** §8.3을 따른다.

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

상류에 저크 제한 궤적 생성이 필요한 지점이 둘 있고, 둘 다 외부 `ruckig`
C++ 라이브러리를 감싸기만 할 뿐 OTG(Online Trajectory Generation) 알고리즘
자체는 moveit_core 안에 없다:

- `moveit_core/online_signal_smoothing/ruckig_filter.cpp` (`RuckigFilter`,
  `SmoothingBaseClass`) — ROS/pluginlib에 결합된 스트리밍 필터
  (`moveit-smoothing`의 `lib.rs` deferral 참고).
- `moveit_core/trajectory_processing/ruckig_traj_smoothing.cpp`
  (`RuckigSmoothing`) — 이미 시간 매개변수화된
  `robot_trajectory::RobotTrajectory`에 대해 한 번 실행하는 스무딩 패스.

`ruckig_traj_smoothing.cpp`/`.hpp`를 전문 읽었다. 실제로 하는 일: 연속된
웨이포인트 쌍마다 현재 웨이포인트의 위치/속도/가속도와 다음 웨이포인트의
목표 위치/속도/가속도로 `ruckig::InputParameter`를 구성하고(속도/가속도
한계는 `RobotModel::getVariableBounds`에서 가져오거나 호출자가 넘긴
맵에서 가져오며, 한계가 정의되지 않은 관절은 5 rad/s / 10 rad/s² /
1000 rad/s³ 기본값으로 대체), `ruckig::Ruckig::calculate()`를 호출한 뒤
그 결과 duration을 `RobotTrajectory::setWayPointDurationFromPrevious`로
되써 넣는다. Ruckig는 각도 랩어라운드를 다루지 못하므로 먼저
`trajectory.unwind()`로 연속 관절을 풀어준다. 실패하거나(옵션으로) 오버슈트가
감지되면 — 10ms 간격으로 샘플링해 위치 오차의 부호가 `overshoot_threshold`를
넘겨 뒤집히는지로 판정 — 해당 세그먼트의 duration을 1.1배로 늘리고 캐시해둔
깊은 복사본으로 전체 궤적을 리셋한 뒤 웨이포인트 루프를 처음부터 다시 돈다.
최대 50배까지 늘리며, 그래도 안 되면 실패를 보고한다. TOTG와 달리 경계
상태가 정지 상태일 필요가 없다: `initializeRuckigState`는 첫 웨이포인트의
실제(0이 아닐 수 있는) 속도/가속도로 Ruckig를 시드한다.

핵심은 **이 파일 안에 OTG 알고리즘이 전혀 없다**는 점이다. 모든 함수가
`RobotTrajectory&`, `RobotState`, `JointModelGroup*` 중 하나를 받는다 —
파일 전체가 한계값 조회와 `RobotTrajectory` 변형 글루 코드이고, 실제
저크 제한 스텝 생성기는 외부 `ruckig` C++ 라이브러리 안에 있지 moveit_core
안에 있지 않다. TOTG의 `Path`/`Trajectory`(상류 스스로 RobotModel 의존성
없이 구현해 두어서 `moveit-trajectory`가 모델 독립 수치 코어를 그대로
추출할 수 있었던 것)와 달리, `ruckig_traj_smoothing.cpp` 안에는 추출할
동등한 코어가 없다.

Rust 크레이트 확인(crates.io, 직접 조회):

- `rsruckig` (github.com/petrikosk/rsruckig, 홈페이지 ruckig.com) — 순수
  Rust, 의존성은 `arrayvec`/`num-traits`/`thiserror`뿐 (`-sys` 크레이트도
  C/C++ 링크 단계도 없음 — D1/D3의 순수 Rust 우선 원칙을 만족). 2023-12-01
  부터 공개, 3.0.0까지 22개 버전, 누적 다운로드 68,326 / 최근 16,217.
  실제로 쓰이고 있는 재구현이지 FFI 바인딩이 아니다.
- `ferromotion-ruckig` (github.com/dcharlot-physicalai-bmi/ferromotion) —
  역시 순수 Rust, 의존성 0개. 공개일 2026-07-12(조회 시점 기준 3주 전),
  그 사이 이미 50개 버전을 릴리스했고 다운로드는 973(전부 "최근" — 실사용
  기반이 없음). 릴리스 빈도와 낯선 홈페이지(`physicalai-bmi.org`)만으로도
  아직 검증되지 않았다고 보기에 충분하다 — 둘 중 실적이 있는 쪽은
  `rsruckig`뿐이다.

**결정: 보류(defer).** 두 지점 모두. 이유는 크레이트 문제가 아니다 —
`rsruckig`는 사용 가능한 순수 Rust, non-`-sys` 후보이고 D3을 만족한다.
막고 있는 것은 **`robot_trajectory::RobotTrajectory`가 아직 Rust로
포팅되지 않았다**는 사실이다. `RuckigSmoothing`의 공개/비공개 메서드
전부가 그 타입을 다룬다(`getWayPointCount`, `getWayPointPtr`,
`setWayPointDurationFromPrevious`, `unwind`, 깊은 복사 생성) — 그리고
`moveit-trajectory`의 이번 단계 범위는 의도적으로 `RobotTrajectory`/
`RobotModel` 의존성이 전혀 없는 수치 코어로 한정돼 있다(이 문서의 크레이트
지도에서 `moveit-trajectory` = `robot_trajectory` + `trajectory_processing`
인 것과, `TimeOptimalTrajectoryGeneration`을 같은 이유로 제외한 것을
참고). `RobotState`(위치/속도/가속도 접근자)와 `VariableBounds`
(`jerk_bounded_`/`max_jerk_` 포함)는 이미 포팅돼 있으므로(`moveit-state`,
`moveit-model`), `RobotTrajectory`가 들어온 뒤에는 `RuckigSmoothing`의
오케스트레이션 뒤에 `rsruckig`를 연결하는 일은 처음부터 수치 알고리즘을
포팅하는 게 아니라 감싸기(wrap)만 하면 된다 — 이후 테스트 벡터 평가에서
`ferromotion-ruckig`을 선호할 구체적 근거가 나오지 않는 한 `rsruckig`를
쓴다.

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

§4.1 `RobotState` dirty-flag → sum type, §4.3 순수 Rust 제약 타입. 둘 다
Phase 1 이후에 뒤집으면 전 크레이트의 타입 시그니처가 바뀐다.
**해소됨 (2026-08-03): §8.2, §8.3 참조.**

## 8. Phase 1 — 진행 중 (2026-08-03)

`moveit-srdf`(SRDF 파서)와 `moveit-model`의 joint 계층이 들어왔다. 둘 다
오라클 대조로 검증했고, 기대값을 Rust 리터럴로 옮겨 적는 대신 오라클의
`model_info` 응답 JSON을 픽스처로 커밋해 역직렬화 비교한다 — 전사 오타가
조용히 통과하는 경로를 없애고, 오라클이 바뀌면 픽스처 diff로 드러나게
하려는 것이다. 같은 이유로 픽스처 URDF/SRDF도 크레이트 안에 벤더링한다:
`third_party/`는 gitignore라 fresh clone과 CI에 존재하지 않는다.

### 8.1 확정된 설계 결정 2건

**`group_state`의 파싱 불가 값은 해당 joint를 버린다.** 상류 srdfdom은
추출 실패 시 `0.0`을 저장하는데, `0.0`은 합법적인 joint 위치라서 하류에서
실패를 구분할 방법이 없다 — 명명된 자세의 오타 하나가 "0으로 이동"으로
조용히 바뀐다. 파싱된 모델이 상류와 달라지는 유일한 지점이고, 잘못된
입력에 한해서만 갈린다.

**URDF `<limit>` 태그의 부재 판정은 RobotModel 계층이 소유한다.**
`urdf_rs::Joint::limit`은 `Option`이 아니라 `#[serde(default)]`여서 태그
부재와 전부 0인 명시적 limit을 구분하지 못한다 (상류는 널 포인터로
구분한다). joint 계층은 이 구분을 할 수 없는 자리에 있으므로 판정을
위로 올린다: URDF와 SRDF를 함께 쥐는 RobotModel이 원본 XML에서 `<limit>`
존재 여부를 직접 읽어 joint 계층에 넘긴다. 방치하면 `<limit>` 없는
revolute joint가 `[0, 0]`으로 잠긴다 — panda와 fanuc은 모든 관절이 명시적
non-zero limit을 가져 드러나지 않을 뿐이다.

### 8.2 §4.1 결정 — dirty 상태는 sum type, 읽기는 빌림 검사기가 지킨다

§4.1의 제안 중 sum type은 채택하고, "읽기 API를 `&mut self`로 바꾼다"는
대가는 **치르지 않는다**. 그 대가는 필요 없다.

상류를 다시 읽고 확인한 사실 하나가 설계를 바꾼다: `dirty_link_transforms_`는
bool이 아니라 `const JointModel*`이고, 더럽혀진 여러 서브트리의 **공통
조상**을 담는다 (`robot_state.hpp:1682`, `RobotModel::getCommonRoot`). 즉
lazy 갱신은 "나중에 전부 다시 계산"이 아니라 "최소 서브트리만 다시 계산"이다.
플래너가 한 그룹의 관절만 반복해서 세팅하는 경로가 정확히 이 최적화를 쓴다.
게으름을 없애고 매번 전체 트리를 갱신하는 설계는 그 핫패스에서 성능 퇴행이다.

따라서 상태는 이렇게 나눈다:

- `RobotState` — 변수 위치 + `dirty: Option<JointIndex>` (더러운 서브트리의
  공통 루트). 모든 변경은 여기서 일어나고 변환은 건드리지 않는다.
- `RobotState::update(&mut self) -> &Posed<'_>` — 더러운 서브트리만 다시
  계산하고 `dirty`를 비운 뒤, 변환 읽기를 `&self`로 노출하는 뷰를 돌려준다.

이러면 "변환을 읽을 수 있다 ⟺ dirty가 아니다"가 **빌림 검사기로** 성립한다.
`&Posed<'_>`가 `self`를 불변으로 재빌림하므로 뷰를 쥔 동안 변경이 불가능하고,
소비자 쪽에 `if dirty` 런타임 분기가 아예 없다. 읽기는 `&self`로 남고,
`Sync`도 유지된다 — 상류가 `const_cast`/`mutable`로 우회하던 지점이다.

값의 의미가 플래그에 따라 달라지는 이중 의미는 사라진다: `Posed`가 존재한다는
사실 자체가 변환이 현재 위치에 대응한다는 증거다.

**FK 차등 테스트에 미치는 영향 없음** — 두 설계 모두 같은 수를 낸다.

#### 8.2.1 프로토타입 검증 결과 (2026-08-03)

§6.3 리스크 표가 "Phase 2 착수 전 프로토타입으로 API 형태 확정"을 완화책으로
지정했으므로, 크레이트 밖에서 이 빌림 패턴만 떼어 컴파일해 확인했다.
`Posed<'a>(&'a RobotState)`를 `update(&mut self) -> Posed<'_>`가 돌려주는
형태다. 확인된 것:

| # | 질문 | 결과 |
|---|---|---|
| 1 | 패턴이 컴파일되는가 | 예 |
| 2 | 뷰가 살아 있는 동안 변경이 막히는가 | 예 — `E0499`로 거부 |
| 3 | 뷰가 살아 있는 동안 **원본 핸들**로 읽을 수 있는가 | **아니오 — `E0502`** |
| 4 | 변경→갱신→읽기 루프가 되는가 | 예 (NLL) |
| 5 | `Posed`가 `Send + Sync`인가 | 예 |
| 6 | 뷰가 drop된 뒤 핸들이 다시 쓰이는가 | 예 |
| 7 | 변환과 위치를 함께 쓰는 함수에 뷰를 넘길 수 있는가 | 예 |
| 8 | 서로 다른 두 상태를 동시에 pose할 수 있는가 | 예 |

**3번이 이 설계의 실제 대가다.** `update`가 돌려준 뷰가 살아 있는 동안에는
`&mut self`에서 파생된 빌림이 유지되므로 `state.variable_position(i)` 같은
`&self` 호출이 `E0502`로 막힌다. 따라서 **`Posed`가 호출자에게 필요한 읽기를
전부 위임해야 한다** — 변환뿐 아니라 관절 위치, 그룹 질의까지.

이것은 §4.1이 우려한 "읽기 API가 `&mut self`가 된다"와는 다른 비용이고, 더
가볍다: 뷰를 쥐지 않은 코드는 `&self`로 자유롭게 읽고, 제약은 FK 결과에
의존하는 구간에만 걸린다. 그 구간은 원래 변환을 만지면 안 되는 구간이다.
Phase 2는 `Posed`의 읽기 표면을 `RobotState`의 것과 같게 유지하는 비용을
예산에 넣어야 한다 — 새 개념이 아니라 위임 보일러플레이트다.

### 8.3 §4.3 결정 — `Option`/enum 채택

§4.3의 설계 원칙을 그대로 채택한다. D1이 코어 크레이트에서 ROS 타입을
금지하므로 `bool has_x` + `x` 쌍을 그대로 옮길 이유가 애초에 없고, 옮기면
§4.1이 지적한 것과 같은 이중 의미를 새 코드에 심는 꼴이 된다. `moveit_msgs`
왕복 변환은 `moveit-ros`의 `TryFrom`에만 두고, 변환 손실은 조용히 넘기지 말고
명시적 에러로 보고한다.

### 8.4 픽스처 현황

`third_party/moveit_resources`에 prbt는 없다 (§7.4의 서술은 틀렸다).
커밋된 로봇 기술 파일은 저장소 루트 `fixtures/` 한 곳에 둔다 —
`third_party/`는 gitignore라 fresh clone과 CI에 존재하지 않기 때문이다.
크레이트 안에 흩어진 사본(`crates/moveit-srdf/tests/fixtures/`,
`crates/moveit-model/tests/fixtures/`)을 이쪽으로 접는 작업은 아직 절반만
끝났다.

쓸 수 있는 것은 panda, fanuc, dual-arm panda(xacro 전개본이
`fixtures/dual_arm_panda.urdf`로 커밋됨), pr2다. pr2는 오라클로 확인한
바로 95 links / 95 joints / 8 groups, Revolute 40·Fixed 49·Prismatic 5·
Planar 1, mimic 6개, 그리고 `position_bounded=false`인 continuous revolute
19개를 갖는다. planar 가상 조인트 `world_joint`의 변수는 `x`/`y`/`theta`이고
`theta`만 무경계다 — `PlanarJoint`와 `normalize_angle`은 panda·fanuc에
planar 조인트가 없어 지금까지 오라클 대조를 받지 못했고, pr2가 그 경로를
연다.

---

## 9. Phase 1 완료, Phase 2·3·7 착수 (2026-08-03)

Phase 1의 완료 조건은 "링크 수, 조인트 수, 그룹 구성, 조인트 한계값, mimic
관계가 오라클과 완전 일치"였다. panda와 fanuc에 대해 충족했고, 대조는
`crates/moveit-model/tests/fixtures/{panda,fanuc}_model_info.json`을 통해
이루어진다. 픽스처가 낡거나 손으로 고쳐지지 않았음은 오라클을 다시
질의해 필드 단위로 비교하는 방식으로 확인했다. dual-arm panda와 pr2는
아직 대조 전이다 — §8.4가 적은 대로 pr2가 planar·continuous 경로를 처음
여는 픽스처다.

### 9.1 `geometric_shapes` 도형 계층 — 소스 부재 상황의 검증 절차

`geometric_shapes`는 moveit2와 별개 패키지이고 이 기계에 소스가 없다.
오라클 이미지에는 헤더와 컴파일된 `.so`만 들어 있다. 그래서 GitHub 태그
`2.3.3`에서 받은 소스를 근거로 삼되, 그 소스가 실제로 링크되는 바이너리와
같은 것임을 두 방향으로 확인했다.

- 헤더는 Debian 패키지(`2.3.3-1noble.20260113.113114`)의 것과 byte-identical.
- `.cpp`는 `shapes.cpp`에만 나타나는 예외/경고 문자열 6개가
  `strings libgeometric_shapes.so.2.3.3`에 각각 정확히 1회 등장함으로 확인.

그 위에, 컨테이너 안에서 실제 라이브러리에 링크한 C++ 프로브를 컴파일해
sphere/cylinder/cone/cuboid/mesh의 `scaleAndPadd` 연쇄와 extents·bounding
sphere 값 30개를 뽑아 Rust 테스트 기대값과 자릿수까지 대조했다. 소스를
읽어 옮긴 것이 맞는지를 소스가 아니라 바이너리에 물어본 셈이다. 같은
절차를 `bodies::` 계층에도 적용한다.

컴파일 방법 — `shape_operations.h`가 `shape_msgs`를 끌어오므로 패키지
자신의 include 디렉터리만으로는 부족하다:

```
source /opt/ros/rolling/setup.bash
INC="-I/usr/include/eigen3"; for d in /opt/ros/rolling/include/*/; do INC="$INC -I$d"; done
g++ -O0 -o /tmp/x /w/x.cpp $INC -L/opt/ros/rolling/lib -lgeometric_shapes
```

### 9.2 이식하며 닫은 upstream 결함 3건

포팅은 upstream을 그대로 옮기는 것이 원칙이지만, 다음 셋은 재현이 아니라
차단을 택했다. 각각 이탈을 `shapes.rs`에 명시한다.

1. `Mesh::scaleAndPadd`가 padding이 0인 scale-only 호출에서도
   `vertex_normals`를 무조건 역참조한다. `vertex_normals`가 null인 메시로
   호출하면 SIGSEGV(exit 139) — 컨테이너에서 재현 확인. 2인자 생성자
   `Mesh(v_count, t_count)`는 배열을 할당하므로 upstream 자체 테스트는
   이 경로를 지나지 않는다. Rust는 `Error::Construct`를 돌려준다.
2. 면적이 0인 삼각형에서 Eigen `normalize()`가 NaN을 만들고, 그것이
   `computeVertexNormals`의 가중 평균으로 번진다. Rust는
   `try_normalize(0.0)`로 영벡터를 대신 넣는다.
3. 삼각형 정점 인덱스가 범위를 벗어나면 upstream은 검사 없이 읽는다.
   `Mesh::new`가 거부한다.

### 9.3 지금 병렬로 도는 작업

- `moveit-collision` — ACM과 collision_common, 오라클 `acm` op 포함
- `moveit-distance-field` — VoxelGrid, PropagationDistanceField
- `moveit-trajectory` — TOTG (`PathSegment`/`Path`/`Trajectory`)
- `moveit-geometry` — `bodies::`가 main에 들어왔으나, §9.1이 이 패키지의
  검증 경로로 못박은 C++ 프로브를 거치지 않았다(담당은 upstream gtest
  리터럴로 대체했다고 보고). 프로브를 직접 만들어 돌린 결과 두 건이
  어긋난다 — `ConvexMesh::ray_intersections`가 scale+padding 아래에서
  교점 개수와 위치 모두 다르고, `OBB::extend_approx`의 extents가 다르다.
  브랜치 `probe-parity`(`dbf50a7`)에 픽스처와 테스트가 있고 담당에게
  넘겼다. gtest 리터럴이 약한 지점은 정확히 하나다: `test/*.cpp`는 `.so`에
  컴파일되지 않으므로, 받아온 소스와 실제 바이너리를 잇는 문자열 테이블
  대조가 그 값들을 덮지 못한다. 계층 (Phase 3 선행 조건)
- `moveit-state` — `RobotState`와 FK, §8.2 설계
- `moveit-planners-sbp` — Phase 7 착수. 오라클이 없는 유일한 단계이므로
  검증은 대조가 아니라 불변식이다: 반환된 경로의 모든 인접 쌍이
  `MotionValidator`를 통과할 것, 같은 시드는 같은 경로를 낼 것,
  최근접 이웃 질의가 brute-force와 일치할 것.

### 9.4 검증 게이트 추가 2건

CI 스텝이 `fmt` / `clippy -D warnings` / `nextest` / doctests /
`check-dep-direction.sh` 다섯이었는데, 그 다섯을 모두 통과하면서도 통과
사실이 거짓인 경우가 두 번 나왔다. 각각을 스크립트로 닫았다.

**`tools/ci/check-no-lint-suppression.sh`** — `crates/`와 `tools/` 어디에도
`#[allow(...)]` / `#[expect(...)]`를 두지 않는다. `clippy -D warnings`는
`#[allow]`을 얹은 코드와 애초에 경고가 없는 코드를 구분하지 않으므로,
한 번 들어간 억제는 이후 어떤 실행으로도 다시 드러나지 않는다. 실제로 두
번 모두 억제된 린트가 진짜 결함을 덮고 있었다 — 오라클 픽스처로 갔어야 할
전사 상수 하나, 그리고 `size_x/y/z`와 `origin_x/y/z`가 타입상 서로 구별되지
않는 8인자 `VoxelGrid::new` 하나. `expect`는 "쓰이지 않으면 경고하는
`allow`"이므로 같이 막는다. 예외 통로는 두지 않았다. 린트 자체가 이
코드베이스에 맞지 않는다면 `[workspace.lints]`에서 한 번, 이유를 적어
끄는 것이 옳다.

**`tools/ci/check-fixture-format.sh`** — 커밋된 오라클 응답 JSON은
2-space indent, 정렬된 키, 끝 개행. 픽스처를 커밋하는 이유가 "오라클이
바뀌면 diff로 드러나게" 하는 것인데 `pr2`와 `dual_arm_panda` 캡처가 각각
21 KB·5 KB 단일 행으로 들어와 그 목적이 무너져 있었다. indent 폭도
`moveit-model` 2 / `moveit-collision` 4로 갈려 있었다. 두 파일만 고치는
대신 전 크레이트 균일 규칙으로 못박았다 — 경계마다 다른 규칙은 다음
캡처가 어느 쪽을 따라야 하는지를 다시 묻게 만든다.

두 스크립트 모두 `git ls-files`가 아니라 파일시스템 glob으로 파일을
찾는다. 첫 판은 `git ls-files`를 썼고, `git archive` 익스포트에 대해
"not a git repository"로 1을 냈다 — 오라클 이미지가 빌드 컨텍스트를
만드는 방식이 정확히 그 익스포트다.

CI는 이 저장소에 git remote가 없어 한 번도 실행된 적이 없다. 대신
`git archive HEAD | tar -x -C $(mktemp -d)` 로 뽑은 트리에서
`.github/workflows/ci.yml`의 `run:` 스텝 7개를 순서대로 돌려
전부 통과함을 확인했다.

---

## 10. 1차 병렬 라운드 병합 (2026-08-03)

§9.3이 나열한 여섯 갈래 중 넷이 main에 들어왔다. 현재 크레이트 아홉,
Rust 19,686 LOC, `cargo nextest run --workspace` 351/351.

| 크레이트 | 내용 | 검증 근거 |
| --- | --- | --- |
| `moveit-geometry` | `geometric_shapes` 도형 계층 | 컨테이너 C++ 프로브 (§9.1) |
| `moveit-collision` | ACM, `World` | 오라클 `acm`·`world` op |
| `moveit-distance-field` | VoxelGrid, PropagationDistanceField | 단위 테스트만 — 오라클 대조 미실시 |
| `moveit-planners-sbp` | StateSpace, GNAT, RRT-Connect | 불변식 (§9.3) |
| `moveit-trajectory` | TOTG | upstream gtest 벡터 |
| `moveit-smoothing` | Butterworth | upstream 단위 벡터 |

`moveit-geometry`의 `bodies::`와 `moveit-state`의 `RobotState`/FK는 아직
작업 중이다.

### 10.1 병합 전 게이트에서 되돌린 것

워커 보고만으로 병합한 것은 없다. 세 건은 보고가 통과라고 적었는데
독립 확인에서 어긋났다.

- `sample_near`가 n-ball 대신 이를 감싸는 상자를 뽑고 있었고, 이를 고친
  1차 수정은 단위 상자 안 rejection sampling이라 채택률이 `V_n/2^n`으로
  떨어졌다. 실측: 차원 7에서 83.6 µs, 12에서 12,915.8 µs, 14에서
  154,737.8 µs/샘플. `fixtures/dual_arm_panda.urdf`가 14-DOF,
  PR2 `arms_and_torso`가 15-DOF이므로 가정이 아니라 실제 차원이다.
  Box-Muller로 교체한 최종본을 다시 실측해 6.85(7) → 25.10(30) µs,
  차원에 선형임을 확인했다. 워커가 보고한 "모든 차원에서 0.5–0.6 µs
  평탄"은 재현되지 않는다 — O(n) 알고리즘이 평탄할 수 없다.
- `rrt_connect`가 `Instant::now()`를 무조건 읽어, 시드 재현성 테스트가
  한가한 기계에서만 성립했다. 워커는 이를 "설계 불확실성"으로 적었으나
  우회된 근본 원인이다. `Termination::{Iterations,Deadline,Both}`로
  갈라, deadline이 없는 경로에는 읽을 시계 자체가 없게 했다.
- `f64::from_bits(0x3fe9_21fb_5444_1d52)`가 `approx_constant` 린트를
  피하려고 들어와 있었고, 이를 정당화하는 주석은 그 값이 π/4의 최근접
  f64이며 1 ULP 차이라고 적었다. 실측 4,550 ULP. 값 36개 전부를 JSON
  픽스처로 옮기고 upstream `.cpp`의 `<<` 블록과 순서대로 대조했다.

### 10.2 검증 게이트 3번째 — `cargo doc` (추가 완료)

`[workspace.lints.rust] warnings = "deny"`는 rustdoc 린트까지 deny로
올리므로, 공개 문서가 비공개 항목을 `[`...`]`로 가리키면 하드 에러다.
그런데 CI 스텝 일곱 중 rustdoc을 실행하는 것이 하나도 없어, 이 에러는
손으로 볼 때만 발견됐다. 네 크레이트에 걸쳐 8곳까지 쌓인 뒤에야 눈에
띄었고, 그것을 담당에게 돌려보내는 사이 `moveit-model`에 새 코드 한
라운드가 들어오며 7 → 11로 늘었다. 증가 속도가 처리 속도보다 빨랐다는
뜻이므로 개별 반환을 그만두고 전부 닫은 뒤
`cargo doc --workspace --no-deps`를 CI 스텝으로 넣었다. 11곳 모두 공개
문서가 크레이트 내부 헬퍼나 세터를 가리킨 경우로, 링크를 만족시키려고
`pub`으로 올릴 만한 것은 하나도 없어 이름을 평문 백틱으로 남겼다.

CI 스텝은 이제 여덟이고, git remote가 없어 여전히 한 번도 실행된 적이
없다. `git archive HEAD`로 뽑은 트리에서 여덟 개를 순서대로 돌려 전부
통과함을 확인했다.

### 10.3 Phase 2 — FK 완료 조건 충족, 야코비안 미착수

`moveit-state`가 §8.2 설계대로 들어왔다. `RobotState::update(&mut self)`가
`Posed<'_>`를 돌려주고 변환 읽기는 그 뷰에만 있으므로, 갱신 없이 변환을
읽는 코드는 컴파일되지 않는다 — 관례가 아니라 빌림 검사기가 지킨다.

Phase 2의 FK 조건은 "임의 관절값 10,000세트 × 3로봇의 모든 링크 FK가
오라클과 `1e-9` 이내 일치"였다. 워커가 4로봇 40,004건으로 통과 보고했고,
seed를 바꿔(424242) 4로봇 8,004건을 독립 재현해 동일하게 0건 불일치를
확인했다. 다만 두 실행 모두 기록이 산문뿐이라
`tools/ci/run-oracle-sweep.sh`로 명령화했다. 커밋된 회귀 테스트인
`crates/moveit-state/tests/fk_parity.rs`는 로봇당 4건만 담으므로 포팅이
아주 깨지는 것은 잡아도 아무도 캡처하지 않은 자세에서만 어긋나는
드리프트는 잡지 못한다 — 그 간극을 메우는 것이 이 스크립트이고, docker가
필요해 `cargo nextest`에는 들어갈 수 없다.

같은 단계의 야코비안(6×N, `1e-7`)과 보간·거리 메트릭은 아직 착수 전이다.
`setFromIK`/`setFromDiffIK`, 부착 바디, 속도·가속도·토크 추적도 마찬가지.

### 10.4 지금 병렬로 도는 작업

- `moveit-collision` — `CollisionRequest`/`CollisionResult`/`CollisionEnv`
  trait. 요청 플래그와 결과 필드의 짝을 bool + 값 병렬 필드로 재현하지
  않고, 요청하지 않은 필드가 표현 불가능하도록 한다 (§4.1의 재적용).
  병합 완료(`6973eed`..`00ce5d6`). 리뷰에서 세 건을 잡아 고쳤다:
  `remove_overlapping`이 제거된 박스를 다시 제거자로 쓰던 것,
  `check_collision`의 여유 판정이 pair 수가 아니라 contact 총수를 보던 것,
  `LinkPaddingScale`의 "tracked"가 맵 두 개로 갈라져 있던 것.

### 10.5 parry 백엔드가 닫아야 할 `max_contacts`

`CollisionEnv`의 메서드는 각자 소유한 `CollisionResult`를 반환하고
`check_collision`이 둘을 병합한다. 업스트림은 `CollisionResult&` 하나를
두 호출에 함께 넘기므로, robot 검사 콜백이 이미 쌓인 self 검사의
contact 수를 보고 `max_contacts`에서 멈춘다. 반환-소유 방식에서는 robot
검사가 self 검사의 결과를 볼 수 없어, 병합 결과가 `max_contacts`를
넘을 수 있다.

`check_collision`의 진입 판정은 업스트림과 일치하지만 (pair 수 대조,
`db31a4c`), 백엔드 내부의 누적 상한은 아직 아무도 강제하지 않는다.
구체 백엔드(parry)를 붙일 때 닫아야 한다 — 남은 예산을 요청과 함께
백엔드에 넘기는 형태가 유력하다. 지금은 백엔드가 없어 관측되지 않는다.
- `moveit-distance-field` — 오라클 `distance_field` op. 이 크레이트는
  현재 C++을 읽고 쓴 단위 테스트만 있고 Phase 3 완료 조건인 `1e-4`
  대조가 없다.
- `moveit-planners-sbp` — `So2Space`/`Se3Space`/`CompoundSpace`.
  `StateSpace` trait 모양이 wraparound와 SO(3)를 감당하는지는 지금까지
  주석의 주장이었지 검증된 적이 없다. 감당하지 못하면 trait을 고친다.
- `moveit-smoothing` — §4.6이 Phase 6 착수 시점으로 미룬 ruckig 크레이트
  채택 결정. 결정과 근거를 §4.6에 되써넣는다.
- `moveit-geometry` — `bodies::`가 main에 들어왔으나, §9.1이 이 패키지의
  검증 경로로 못박은 C++ 프로브를 거치지 않았다(담당은 upstream gtest
  리터럴로 대체했다고 보고). 프로브를 직접 만들어 돌린 결과 두 건이
  어긋난다 — `ConvexMesh::ray_intersections`가 scale+padding 아래에서
  교점 개수와 위치 모두 다르고, `OBB::extend_approx`의 extents가 다르다.
  브랜치 `probe-parity`(`dbf50a7`)에 픽스처와 테스트가 있고 담당에게
  넘겼다. gtest 리터럴이 약한 지점은 정확히 하나다: `test/*.cpp`는 `.so`에
  컴파일되지 않으므로, 받아온 소스와 실제 바이너리를 잇는 문자열 테이블
  대조가 그 값들을 덮지 못한다.

### 10.6 픽스처 재검증 — 스탬프 이미지 대조 (2026-08-03)

지금까지 커밋된 오라클 픽스처는 전부 **스탬프가 없는** 이미지에서 뜬
것이다. 소스 다이제스트 게이트(`run-oracle.sh`)를 넣은 뒤 처음으로
`build.sh`를 돌려 스탬프된 이미지를 만들었고(`88c9e573…`), 커밋된 요청을
그대로 다시 태워 응답을 대조했다. 11건 전부 바이트 단위로 동일하다:

- `acm` — panda / fanuc / dual_arm_panda / pr2
- `model_info` — panda / fanuc / dual_arm_panda / pr2
- `world` — `world_request.json`
- `distance_field` — 양수·음수 전파 2건

즉 게이트 도입 이전에 뜬 픽스처들의 출처 불명 상태는 해소됐다. 게이트가
막으려던 것(바뀐 op이 조용히 다른 답을 주는 것)은 실제로 일어나지
않았다.

`model_info`를 처음 대조했을 때 4건 모두 어긋난 것으로 나왔는데, 차이는
요청의 `id` 에코 하나뿐이었다(커밋본은 `id: 0`, 재질의는 `id: 1`). 응답
본문은 처음부터 동일했다. 픽스처를 다시 뜰 때 요청을 그대로 재생하지
않으면 이런 가짜 차이가 난다.

FK 스윕도 스탬프 이미지로 다시 돌렸다 — 이전 검증에 쓰지 않은 seed 7,
로봇 4종 각 10,001 케이스, 전부 통과(실패 0). Phase 2의 FK 완료 조건은
이제 출처가 확인된 오라클에 대해 성립한다.

이 이미지를 쓰려면 이 머신에서는 `sg docker -c '...'`로 감싸야 한다.
`build.sh`를 그냥 부르면 docker 소켓 권한에서 실패하는데, 로그를 보지
않고 셸의 종료 코드만 보면 성공으로 오독하기 쉽다.

한 가지가 이 과정에서 드러났다. 첫 스윕은 panda 통과 직후 fanuc에서
게이트에 막혔다 — 다른 워크트리가 그 사이 `moveit-rs/oracle:latest`를
다시 빌드해 이미지를 갈아끼웠기 때문이다. 워크트리 7개가 도커 데몬
하나를 공유하고 그중 셋이 동시에 오라클 C++을 고치고 있었으니, 가변
태그 하나는 마지막에 빌드한 쪽이 조용히 모두의 오라클이 된다는 뜻이다.
`5e60782`에서 태그를 소스 다이제스트로 파생시켜(`moveit-rs/oracle:<digest16>`)
이 경합을 없앴다. 게이트가 없었다면 fanuc은 실패가 아니라 *다른 오라클의
답*을 받았을 것이고, 그건 아무 데서도 보이지 않는다.

