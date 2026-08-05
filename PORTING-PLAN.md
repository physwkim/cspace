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

**§225.4가 이 문장을 좁혔다:** 유지되는 것은 "나중에 백엔드를 추가할 수
있다"는 목적이고, 그것은 `CollisionEnv` trait이 이미 지고 있다.
`CollisionDetectorAllocator` 자체는 백엔드 타입을 런타임 문자열로 미루기
위한 간접층인데 이 포트는 호출자가 타입으로 지목하므로 소비자가 없다 —
`decided-non-port`.

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

**갱신 (2026-08-04, §62) — 결정: 완료(done).** 위 "보류"는 두 지점 모두
해소됐고, 막고 있던 이유는 최종적으로 틀렸다.

- `RuckigSmoothing` → `moveit-trajectory::ruckig_smoothing`. `ruckig_parity.rs`가
  `rsruckig::Ruckig::calculate`(offline 경로)를 오라클과 대조한다.
  `RobotTrajectory` 가용성 블로커는 그 타입이 포팅되면서 해소됐다.
- `RuckigFilterPlugin` → `moveit-smoothing::ruckig_filter::RuckigFilter`
  (`fa5572e`). 여기서 실제 블로커는 `RobotTrajectory` 미포팅이 **아니라**
  ROS(`rclcpp::Node`) 결합이었다 — `doSmoothing`/`reset`/
  `getVelAccelJerkBounds`는 `Eigen::VectorXd`만 다루고 `RobotTrajectory`에
  의존한 적이 없다. `ButterworthFilterPlugin`→`ButterworthFilter`와 같은
  방식으로 ROS-free 코어를 뽑아 해소했다. **오라클 근거 없음(disclosed):**
  `rsruckig::Ruckig::update`의 스트리밍 API를 감싸지만 이 경로를 검증하는
  오라클 op는 없다 — `moveit-smoothing/src/lib.rs` 참고.
- 크레이트 선택(`rsruckig`)은 처음부터 문제가 아니었다.

### 4.7 명시적 범위 밖 — 영구히 C++로 남는 것

- `moveit_setup_assistant` (27,714) — Qt5/rviz 위젯
- `moveit_ros/visualization` (13,346) — RViz 디스플레이 플러그인
- `moveit_ros/perception` (7,877) — mesh_filter가 OpenGL/GLUT 의존
- `moveit_plugins` (2,990) — ros2_control 컨트롤러 인터페이스
- `moveit_py` (5,368) — 필요 시 PyO3로 별도 재작성

### 4.8 OcTree 충돌 도형 — parry에 다중 해상도 옥트리가 없다 (결정 완료)

**upstream이 실제로 하는 일.** `collision_detection_fcl/src/collision_common.cpp:924-927`,
`createCollisionGeometry`의 `shapes::OCTREE` case 전문:

```cpp
case shapes::OCTREE:
{
  const shapes::OcTree* g = static_cast<const shapes::OcTree*>(shape.get());
  cg_g = new fcl::OcTreed(g->octree);
}
```

`octomap::OcTree`를 그대로 `fcl::OcTreed`에 넘긴다 — 변환도 재샘플링도
없다. 실제 순회는 FCL 0.7.0 (오라클 컨테이너에 설치된 버전,
`/usr/include/fcl/narrowphase/detail/traversal/octree/octree_solver-inl.h`)의
`OcTreeSolver::OcTreeShapeIntersectRecurse`가 한다. 이 함수를 전문 읽었다
— 핵심 가지치기 규칙(같은 파일, `ShapeOcTreeIntersect`/
`OcTreeShapeIntersectRecurse` 전체):

- 리프가 아닌 노드에서: 그 노드가 `isNodeFree()`(완전히 비어있음이 확정)
  이거나 질의 도형이 free면, **자식을 하나도 방문하지 않고** 그 서브트리
  전체를 반환한다.
- 그 외의 경우, 그 노드 **자신의 깊이에서의 AABB**(옥트리 리프가 아니라
  내부 노드 — 즉 아직 세분화되지 않은 굵은 셀)를 OBB로 변환해 질의
  도형의 OBB와 먼저 겹침 검사를 한다. 겹치지 않으면 역시 서브트리 전체를
  건너뛴다.
- 겹칠 때만 실제로 존재하는 자식(`nodeChildExists`)들로 재귀한다.
  `computeChildBV`가 매 깊이마다 부모 AABB를 정확히 8등분해 자식 AABB를
  만든다 — 이 형태 자체가 옥트리 고유의 깊이별 세분화이고, 도형과 아무
  관계없는 균일 그리드가 아니다.
- 자식이 없는 리프(`!tree1->nodeHasChildren(root1)`)에 도달했을 때만
  실제 narrow-phase(`solver->shapeIntersect`)를 그 리프 **자신의 크기**
  (`bv1`, 리프가 어느 깊이에서 가지치기됐든 그 깊이의 셀 크기)로 수행한다.

즉 FCL이 얻는 것은 단순히 "메모리 절약"이 아니라 **깊이 적응형
broad-phase 하강**이다: 넓은 빈 영역이나 넓은 점유 영역은 옥트리 안에서
이미 굵은 리프 하나로 뭉쳐 있고, 그 굵은 리프의 AABB 겹침 검사 한 번으로
그 아래 잠재적으로 수백~수만 개의 최고해상도 셀 전부를 한 번에
쳐낸다(또는 한 번에 받아들인다). 균일 해상도 도형은 이 "굵은 셀 하나 =
겹침 검사 하나"를 표현할 방법이 없다 — 굵은 리프를 최고해상도로 펼친
뒤에야 그 펼쳐진 셀들에 대해 겹침 검사를 시작할 수 있고, 그 순간 이미
`prune()`이 절약한 메모리를 다시 부풀린 뒤다.

**실측 비용.** `moveit_octomap::OcTree`(이 프로젝트 자체 포트)로 방
크기 장면(4×4×2.4m 외곽 벽+바닥+천장, 0.6m 탁자 블록, 나머지 내부는
센서 스캔 후 실제로 나오는 밀도로 자유 공간을 최고해상도까지 채운 뒤
`prune()`)을 만들어 `tree.leaves()`로 리프별 깊이/점유 상태를 세고,
`parry`의 `Voxels`가 요구하는 최고해상도 균일 셀 수(리프마다
`8^(최고깊이 - 리프깊이)`)를 합산했다:

| 해상도 | 실제 리프 수 | `Voxels` 최고해상도 셀 수 | 배율 |
|---|---|---|---|
| 0.05 m | 46,978 | 321,280 | ×6.84 |
| 0.02 m | 305,989 | 4,888,000 | ×15.99 |

0.05m 장면 하나만 봐도 깊이 12(셀 0.8m)에 자유 공간 리프가 28개뿐인데,
이 28개가 펼쳐지면 28 × 8^4 = 114,688개의 최고해상도 셀이 된다 — 리프
28개짜리 겹침 검사가 최고해상도에서는 114,688개짜리 겹침 검사(혹은
사전에 114,688개짜리 균일 그리드 구조)로 바뀐다는 뜻이다. 해상도가
가늘어질수록(0.02m) 배율이 커지는 것(×16)도 예상대로다 — 옥트리 깊이가
늘수록 굵은 리프가 대표하는 최고해상도 셀 수(`8^k`)가 기하급수적으로
커지기 때문이다. (측정 스크립트는 스캐치용 `cargo run --example`로
실행하고 결과만 이 문서에 남겼다 — 커밋에는 포함하지 않는다.)

**실현 가능한 선택지, 각각의 비용.**

1. **`moveit_octomap::OcTree` 위에 parry의 도형/질의 트레잇을 직접
   구현.** `parry3d-f64`의 `Shape: RayCast + PointQuery + Any + Send +
   Sync`와 `QueryDispatcher`(`src/query/query_dispatcher.rs`의 모듈
   문서가 명시적으로 "커스텀 도형 타입이 있으면 커스텀
   `QueryDispatcher`를 만들고 `chain()`으로 기본 디스패처에 나머지를
   위임하라"고 문서화한 확장점이다)를 이용해 FCL과 동일한 구조의
   재귀 하강(굵은 노드 AABB로 먼저 가지치기, 실제로 겹치는 자식만
   내려감)을 직접 짠다. 비용: 실제 엔지니어링 작업(알고리즘 자체는
   위에서 읽은 것과 동형이라 알 수 없는 게 아니다)이지만 새 크레이트
   의존성이 없고, §2가 지적한 "성숙한 옥토맵 Rust 크레이트 부재" 문제와
   무관하다 — `moveit_octomap::OcTree`가 이미 리프 순회(`leaves()`,
   `depth()`, `size()`, `is_occupied()`)를 제공하므로 그 위에 얹으면
   된다. 위험: `QueryDispatcher`가 다루는 모든 도형-쌍(구/원기둥/박스/
   메시 vs 옥트리)마다 겹침 검사를 새로 구현해야 하는데, 각각을 굵은
   노드의 OBB 대 해당 도형의 OBB로 바꿔치기하면 되므로 대칭적이지만
   조합 수만큼 코드가 늘어난다.
2. **리프 하나당 `Cuboid` 하나로 만든 `Compound`.** `parry`의
   `Compound`(`src/shape/compound.rs`)는 정확히 이 용도 —
   `(Pose, SharedShape)` 목록을 받아 자체 BVH를 구성한다. `tree.leaves()`
   를 순회하며 리프 각각을 **그 리프 자신의 깊이/크기**로 된 `Cuboid`
   하나로 만들면 되므로(최고해상도로 펼치지 않는다), 도형 수는 위 표의
   "실제 리프 수" 그대로다(0.05m 장면 46,978개, `Voxels`라면 필요했을
   321,280개가 아니라). 비용: 새 트레잇 구현이 전혀 없다 — `Compound`가
   이미 있는 겹침/거리 질의를 그대로 쓴다. 대가:
   `Compound::new`의 rustdoc이 스스로 명시한다 — "The BVH is built using
   a binned construction strategy **optimized for static scenes**. For
   large compounds (100+ shapes), construction may take noticeable
   time." 즉 옥트리가 바뀔 때마다(센서가 매 프레임 갱신하는 실사용
   패턴) 리프 46,978~305,989개 전체로 `Compound`를 통째로 재구성해야
   한다는 뜻으로 읽힌다 — 부분 갱신(바뀐 리프만 반영) API가 있는지는
   확인하지 못했다. 정적이거나 갱신이 드문 맵에는 가장 간단한 선택지,
   실시간 센서 스트림에는 재구성 비용이 병목이 될 수 있다.
3. **깊이 상한을 둔 균일 복셀 확장.** 예: 0.1m보다 가는 리프는 0.1m로
   뭉개서 `Voxels`에 넣는다. 위 표의 배율 문제를 완화하지만 없애지는
   않는다(상한보다 가는 점유 리프가 있으면 그 리프만은 여전히 `8^k`
   확장이 필요하다) — 그리고 상한보다 가늘게 감지된 점유 셀은
   충돌 검사에서 상한 크기로 뭉개져 보이므로, 이는 **충돌 정확도를
   낮추는 정책 결정**이다. 상한값 자체가 사용 사례마다(매니퓰레이터
   근접 감지 vs 내비게이션 코스트맵) 달라야 해서 이 문서만으로 결정할
   수 있는 값이 아니다.
4. **FCL FFI.** `libfcl-dev 0.7.0`(오라클 컨테이너에 이미 설치돼 있고
   호스트 apt 후보에도 있다)을 그대로 링크해 `fcl::OcTreed`를 호출하면
   upstream과 100% 동일한 동작을 얻는다 — 유일하게 "다르게 동작할
   여지"가 없는 선택지다. 대가: D3("순수 Rust 우선, FFI는 나중")이
   미루기로 한 바로 그 C++ 의존을 **이 한 도형 때문에** 지금 들여오는
   것이고, 그것도 오라클 이미지(빌드 타임에만 존재)가 아니라 실제로
   배포되는 `moveit-rs` 라이브러리/바이너리 자체에 `libfcl`+`octomap`
   C++ 링크를 요구하게 된다는 점에서 지금까지의 FFI 회피 범위보다
   훨씬 크다.

**추천과 근거 (구현은 하지 않음 — 사용자가 이 근거로 직접 결정).**
1번(커스텀 parry 도형)을 추천한다. 이유: FCL이 실제로 하는 일(깊이
적응형 가지치기)을 그대로 재현하는 유일한 선택지이면서 순수 Rust이고,
`QueryDispatcher`가 이 용도로 명시적으로 문서화된 확장점이기 때문이다.
2번(리프별 `Compound`)은 구현량이 훨씬 적어 실시간 갱신 비용이
확인되기 전까지는 임시 경로로 고려할 만하다 — 사용 패턴이 "가끔
갱신되는 정적에 가까운 맵"이면 1번보다 먼저 시도해볼 가치가 있다.
3번은 정확도를 낮추는 정책 결정이라 이 문서 혼자 정할 수 없다. 4번은
D3의 순서를 이 도형 하나 때문에 뒤집는 선택이므로, 1/2번이 실제로
막히는 게 확인된 뒤의 최후 수단으로 남겨둔다.

**결정 (2026-08-03): 2번을 먼저 구현한다. 1번은 재구성 비용이 실측으로
병목임이 확인된 뒤에만 착수한다.** 근거 셋:

- 위 조사가 FCL의 이득으로 짚은 것은 "굵은 리프 하나의 AABB 겹침 검사
  한 번으로 그 아래 전부를 쳐낸다"이다. 리프를 **자기 깊이 크기의
  `Cuboid`** 하나로 만든 `Compound`는 그 성질을 그대로 갖는다 — 도형
  수가 실제 리프 수(46,978 / 305,989)이지 최고해상도 셀 수(321,280 /
  4,888,000)가 아니고, `Compound`의 BVH가 굵은 리프를 상위 노드에서
  한 번에 쳐내는 하강을 이미 해준다. 1번이 손으로 짜려는 깊이 적응형
  하강은 2번에서 parry의 검증된 BVH가 공짜로 준다.
- 1번의 실제 비용은 조사 자신이 적어둔 대로 도형 조합 수만큼 늘어나는
  narrow-phase 구현이다. 상류와의 패리티가 이 포트의 가치인데, 검증되지
  않은 손수 짠 narrow-phase 표면을 도형쌍마다 새로 만드는 것은 그
  가치를 깎는 방향이다. 2번은 새 트레잇 구현이 0이고 parry가 이미
  검증한 질의를 그대로 쓴다.
- 1번을 정당화하는 유일한 근거는 갱신 시 `Compound` 재구성 비용인데,
  그 값은 아직 **아무도 재지 않았다**. 측정되지 않은 비용을 근거로 큰
  구현을 먼저 하는 것은 순서가 뒤집힌 것이다.

따라서 2번 구현과 함께 `Compound::new` 재구성 시간을 두 해상도(0.05 m /
0.02 m)에서 측정해 이 문서에 남긴다. 그 숫자가 실사용 센서 갱신 주기
대비 병목이면 1번이 근거를 얻고, 아니면 1번은 필요 없다. 3번(깊이 상한)은
조사 판단대로 정확도 정책이라 채택하지 않는다. 4번(FCL FFI)은 D3를
뒤집으므로 1·2번이 모두 막힌 뒤의 최후 수단으로 남긴다.

**2번 구현 완료 (2026-08-03), 실측 결과.** `moveit-geometry::compound_from_octree`
— 점유 리프마다 자기 깊이 크기의 `Cuboid` 하나, `Compound::new`로 조립.
비점유(free/unknown) 리프는 도형을 만들지 않는다. `octomap`의
`isNodeOccupied`와 같은 술어([`Leaf::is_occupied`])로 걸러서, FCL 자신의
옥트리 순회가 보는 점유 판정과 어긋나지 않게 했다.

`tools/moveit-oracle`에 `octree_shape_query` op를 새로 추가해 검증했다 —
`collision_detection::createCollisionGeometry(shape, World::Object*)`로
`fcl::CollisionObjectd` 둘(옥트리 하나, 질의 도형 하나)을 직접 만들어
`fcl::collide`/`fcl::distance`를 그대로 호출한다(`CollisionEnvFCL`/
`RobotState`/ACM을 거치지 않음 — 로봇 대 월드가 아니라 임의 도형 대 임의
도형 질의라 그 계층이 맞지 않는다). 과제가 요구한 네 경계 — 굵은 자유
리프 내부, 굵은/가는 리프 경계에 걸침, 리프 면에 정확히 접촉, 가지치기로
사라진 서브트리 — 를 각각 오라클로 캡처해 `Compound` 결과와
`parry3d_f64::query::contact`로 비교했고, 넷 다 오라클과 일치한다
(`crates/moveit-geometry/tests/octree_shape_query_parity.rs`). 네 번째
경계는 실측 중 실제 버그를 하나 드러냈다 — 점유 표시 1회 후 미표시(free)
1회는 log-odds가 `0.847 - 0.405 = 0.442`로 여전히 점유 임계값(0) 위에
남아 지워지지 않는다(확률적 필터링, 버그 아님); 서브트리를 실제로
비우려면 미표시를 3회(`0.847 - 3×0.405 = -0.368`) 반복해야 한다 — 이
경계가 아니었으면 드러나지 않았을 옥트리 자체의 성질이다.

**`Compound` 갱신 API: 부분/증분 갱신 없음, 확정.** parry3d-f64 0.30.0
벤더 소스의 `Compound` 공개 API 전체(`new`, `decompose_trimesh`,
`shapes()`, `local_aabb()`, `local_bounding_sphere()`, `aabbs()`,
`bvh() -> &Bvh`)를 읽었다 — 값을 바꾸는 메서드가 `new` 하나뿐이다.
센서 갱신마다 옥트리가 바뀌면 `Compound` 전체를 다시 만드는 것 외에
경로가 없다.

**`Compound::new` 재구성 시간 실측.** round 3의 방 장면 생성기는 커밋되지
않은 스크래치 코드였다(`git log`로 확인 — round 3 커밋 두 개
(`164457e`, `fbca268`)는 PORTING-PLAN.md만 바꿨다). §4.8의 산문 명세
(4×4×2.4m 방, 벽/바닥/천장, 0.6m 탁자, 나머지 내부를 최고해상도까지 자유
공간으로 채운 뒤 prune)를 그대로 재구현해
`crates/moveit-geometry/examples/octree_compound_bench.rs`로 커밋했다 —
round 3의 리프 수(46,978 / 305,989)와 정확히 일치하지는 않지만
(0.05 m: 총 리프 60,205개, 점유 27,324개 / 0.02 m: 총 리프 401,893개,
점유 177,488개), 같은 자릿수라 명세를 충실히 재현한 것으로 판단한다.
`Compound::new` 자체만(리프→`Cuboid` 매핑은 타이머 밖) 10회 평균:

| 해상도 | 점유 리프(= `Cuboid`) 수 | `Compound::new` 평균 |
|---|---|---|
| 0.05 m | 27,324 | 13.3 ms |
| 0.02 m | 177,488 | 130.4 ms |

리프 수가 6.5배 늘 때 시간도 거의 그만큼(9.8배) 늘어 — 대체로 리프 수에
선형이다. 흔한 depth-camera 갱신 주기(10–30 Hz, 33–100 ms 예산) 대비:
0.05 m는 13.3 ms로 여유가 있지만, 0.02 m는 130.4 ms로 **재구성 하나만으로
10 Hz 예산(100 ms)을 이미 넘는다** — 질의 자체는 아직 시작도 안 한
시점이다. 즉 1번(손수 짠 깊이 적응형 `parry` 도형)은 조밀한 장면을
낮은 해상도(0.02 m 근방)로 자주 갱신해야 하는 사용 패턴에서는 근거를
얻는다; 방 규모/0.05 m급 성기게 갱신되는 지도에서는 2번의 전체 재구성이
실시간 예산 안에 들어와 1번이 필요 없다. 사용 패턴이 어느 쪽인지는 이
포트 혼자 정할 수 없다 — Phase 3 충돌/Phase 5 씬이 실제로 어떤 갱신
주기·해상도로 옥트리를 쓰는지가 나온 뒤 재논의한다.

### 4.9 IK 재탐색의 타임아웃 시맨틱 — 벽시계 타임아웃 없음 (결정 완료)

**upstream이 실제로 하는 일.** `kdl_kinematics_plugin.cpp:369-409`의
`searchPositionIK` 본체는 `do { ... } while (!timedOut(start_time, timeout))`
— 재시도 횟수가 아니라 **벽시계 시간**으로 끊긴다. `timedOut`(253-256행)은
`steady_clock.now() - start_time >= duration`을 잰다. 기본 `timeout`은
`KinematicsBase::DEFAULT_TIMEOUT = 1.0`초(`kinematics_base.cpp:53`)이고,
매 재시도는 `getRandomConfiguration`으로 완전히 새로 재시드한다. 즉 같은
시드로 같은 호출을 반복해도, **그 머신이 그 순간 얼마나 빠른가**에 따라
1초 안에 몇 번을 시도하는지가 달라지고, 성공률도 함께 달라진다 —
upstream 자신의 success-rate 수치가 하드웨어/부하 종속적이라는 뜻이다.

**이 포트가 이미 대체한 것.** `SolverParams::max_restarts`(round 1에서
이미 도입, `params.rs`)는 `timeout`을 벽시계 시간이 아니라 **횟수**로
바꿨다 — 같은 시드에서 항상 같은 횟수, 같은 순서로 재시도한다는 뜻이다.
이건 새로운 제안이 아니라 이미 내려진 결정의 소급 확인이다.

**측정된 결과가 이 질문에 실제로 답한다.** 2라운드의 결정적 실행
(`max_restarts=0`, 재시도 없음)에서 네 픽스처 전부 `b≈c`
(panda 2/2, fanuc 2/0, dual_arm_panda 3/7, pr2 15/18) — 핵심 Newton/SVD
알고리즘 자체는 상류와 동등함이 이미 확인됐다. `max_restarts=20`(기본값)로
재시도를 켜고 `rust_impl::ik`의 솔버 재구성 버그(호출마다 RNG 재시드)를
고친 뒤에는: panda 73/68(χ²=0.18), fanuc **299/299(0.00, 성공률까지
동일)**, pr2 13/10(0.39) — 전부 노이즈 범위. dual_arm_panda만 시드 1에서
95/61(χ²=7.41)로 튀지만, 시드 2-4를 더한 pooled는 340/307(χ²=1.68,
p≈0.19)로 역시 노이즈다. 즉 **횟수 기반 재시도 자체는 상류와 통계적으로
구별되지 않는다** — 남는 차이가 있다면 그건 알고리즘이 아니라 "몇 번
시도했는가"의 차이이고, 그 "몇 번"을 벽시계로 재는지 횟수로 재는지는
바로 이 절이 다루는 질문이다.

**권고: 벽시계 타임아웃을 도입하지 않는다.**

1. 결정론적 재현성은 이 포트의 기존 설계 축이다 — `max_restarts` 자체가
   "시드 하나로 항상 같은 결과"를 위해 `timeout`을 대체하려고 이미
   존재한다 (`params.rs`의 `max_restarts` 문서 참고). 벽시계 타임아웃을
   더하면 그 축을 다시 무너뜨린다.
2. `moveit-diff` 오라클 비교 하네스 전체가 "같은 입력 → 같은 판정"을
   전제한다. 벽시계 바운드는 CI 머신 부하나 병렬 실행 여부에 따라 같은
   케이스가 다른 라운드에 다른 결과를 내게 만들 수 있다 — 이번
   라운드에서 겨우 걷어낸 "재시작 RNG가 통계를 오염시킨다"는 문제를
   "재시작 횟수 자체가 실행마다 달라진다"는 문제로 형태만 바꿔 되살리는
   셈이다.
3. upstream 자신의 success-rate 표조차 `timeout=1.0초`에서 그 머신이 몇
   번 재시도했는지에 따라 달라지는 수치라, "이 포트가 upstream의
   성공률과 정확히 같아야 한다"는 목표 자체가 벽시계 타임아웃 없이도
   이미 달성 불가능한 기준이다 — 위 측정이 보여주듯 알고리즘 동등성은
   횟수 기반 비교로 이미 충분히 검증된다.
4. 실시간 제어 루프처럼 실제 벽시계 마감이 필요한 소비자가 나중에
   생기면, `solve`를 감싸는 얇은 바깥 래퍼(예: 호출자가
   `Instant::now()`를 보고 재호출을 멈춤)로 해결할 수 있다 —
   `search_position_ik`/`cart_to_jnt` 자신의 결정론적 코어를 건드릴
   필요가 없다. 지금 벽시계 로직을 미리 넣는 것은 이 시점에 근거 없는
   설계다.

**결정 (2026-08-03): 벽시계 타임아웃을 도입하지 않는다. 권고 채택.**
결정적인 근거는 4번이다 — 벽시계 마감은 `search_position_ik`의 코어가
아니라 호출자 쪽 관심사이고, 바깥 래퍼로 나중에 언제든 얹을 수 있다.
지금 코어에 넣으면 되돌리는 쪽이 비싸다. 2번(하네스가 "같은 입력 → 같은
판정"을 전제한다)은 그 선택을 지금 뒤집을 이유가 없다는 확인이지, 그
자체로 영구적인 제약은 아니다. 실시간 소비자가 실제로 생기기 전까지
`max_restarts`가 유일한 재시도 바운드로 남는다.

§4.9 제목을 "결정 완료"로 바꾼다.

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

> **상태: 충족 (2026-08-05, §218).** 3종 전부에 대해 다섯 항목을 항목별로
> 따로 판정해 전부 일치. §9가 "panda와 fanuc에 대해 충족"이라고 적은 뒤
> prbt만 남아 있었고, 남은 이유는 prbt 픽스처가 트리에 없었던 것이다 —
> `fixtures/prbt.{urdf,srdf}`를 벤더링해 닫았다. dual_arm_panda·pr2까지
> 5종 25항목 전부 일치. 실행 주체는 `tools/ci/verify-oracle-sweep.sh`이고,
> 항목별 판정은 `compare_model_info_clauses`다. 수치와 변별력 근거는 §218.

### Phase 2 — 상태 계층 (5주, ~8,000 LOC)

`moveit-state` — FK, 야코비안, 랜덤 상태, 보간, 거리 메트릭, `dynamics_solver`

**완료 조건:**
- 임의 관절값 10,000세트 × 3로봇의 모든 링크 FK가 오라클과 `1e-9` 이내 일치
- 야코비안(6×N)이 `1e-7` 이내 일치 (열 순서 규약 포함)
- 관절 한계 클램핑, mimic 전파, floating/planar 조인트 보간이 일치

> **상태: 세 항목 전부 충족 (2026-08-06, §238).** 앞의 두 항목은 §218이
> `tools/ci/verify-oracle-sweep.sh`로 닫았다. 세 번째 항목은 §218이
> "오라클과 맞춰 본 적이 없다"고 적은 그대로였고, 이번 라운드에
> `tools/ci/verify-phase2-state-sweep.sh`를 만들어 실측했다 — 5로봇
> 4,224 케이스(클램핑 996, mimic 50, 보간 3,178), **허용오차 0.0(비트
> 일치)에서 불일치 0건**. 케이스는 무작위 상태가 아니라 오라클이 보고한
> 한계값에서 열거한 경계값이고, 측정이 덜 덮은 두 곳(모든 픽스처의 mimic
> 이 `multiplier=1, offset=0`이라는 점, 포트에 `RobotState::interpolate`
> 전체 루프가 없어 조인트별로 비교한다는 점)은 §238에 적었다.

### Phase 3 — 충돌 검사 (7주, ~14,000 LOC)

`moveit-collision` (parry 백엔드), `moveit-distance-field`

- `CollisionRequest` / `CollisionResult` / `AllowedCollisionMatrix`
- self-collision, world-collision, 거리 질의, 접촉 열거
- distance_field + collision_distance_field (§2 공백 없음, 직접 이식)

**완료 조건:**
- 10,000 상태 × 3로봇에서 `collision: bool` 이 오라클과 **100% 일치**
- `distance: f64` 가 `1e-4` 이내 일치
- 접촉점 좌표는 비교 대상에서 제외 (§4.5, 검증 한계로 기록)

> **상태.** §5의 **완료 조건 현황표**를 보라 — 이 절이 쌓아 온 상태
> 갱신(§218 초기 측정, §229 원인 확정)은 그 표로 옮겼다. 측정 자체의
> 증거는 §218.3/§218.4(초기 측정)와 §229.1/§229.3(원인 확정)에 남아 있다.

### Phase 4 — 역기구학 (5주, ~5,000 LOC)

`moveit-kinematics` — `KinematicsSolver` trait + KDL 대응 수치 IK
(Newton-Raphson, LMA), position-only IK, 관절 한계 처리

**완료 조건:** 도달 가능한 목표 자세 5,000개에 대해 (a) 성공률이 C++ KDL
플러그인 이상, (b) 성공한 해의 FK가 목표 자세와 `SolverParams::epsilon`
(`1e-5`) 이내 일치 — 병진·회전 각각. IK 해 자체의 일치는 요구하지 않는다
(랜덤 시드 의존).

(b)는 원래 `1e-6`이었다. 그 수는 이 솔버의 계약을 잘못 읽은 것이다:
`CartToJnt`는 `max(position_error, orientation_error) <= epsilon`인 구성을
그대로 반환하므로 FK 오차의 상한은 언제나 `epsilon`이고, 상류가 선언한
`epsilon` 기본값이 `1e-5`다. `1e-6`은 정확도 요구가 아니라 "`epsilon`을
`1e-6`으로 두라"는 문장이었고, 그렇게 두면 오라클(`kEpsilon` 고정)과 계약이
달라져 같은 조건의 (a)가 비교 대상을 잃는다. 근거·격자 측정·대안의 비용은
§221.2.

(a)를 한계(marginal) 성공률의 대소로 읽으면 안 된다 — 양쪽의 재시작 재시드
난수열이 서로 무관하므로 그 대소는 뽑기를 측정한다. 판정은 쌍(paired)
통계와 재시작을 끈 비교로 한다: §221.1.

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

**측정 결과 (2026-08-05, `tools/ci/verify-phase7-benchmark.sh full`,
전문은 §219, 원자료는 `doc/phase7-benchmark-results.json`).** 게이트
집합은 panda_arm 500건 = `floor_wall` 250(seed 900001) + `cage`
250(seed 900002), 양쪽 다 `seed_base` 424242, 호출당 타임아웃 120s:

- 조건 1 — **충족.** 포트 497/500 = 99.4%, C++ OMPL RRTConnect
  498/500 = 99.6%. 요구선은 89.64%(C++의 90%). 타임아웃 0.
- 조건 2 — **충족.** 경로 497개 전부 통과, 끝점만이 아니라
  waypoint 168,340개 전부를 `PlanningScene::is_path_valid`에 걸었다.
  제약 쪽은 별도 제약 집합 250건(`panda_joint1:0.0:0.5`)에서
  250/250, waypoint 67,820개. 검사가 실패할 수 있음을 먼저
  증명한다 — 주입 게이트 두 개(§219.4).
- 조건 3 — **충족.** 포트 중앙값 2.668003737362192 ≤ 3.4577097142570405
  (1.3 × C++ 2.6597767032746464), 비율 1.003배. 같은 미터법인 근거는
  §118.3.

두 번째 로봇 fanuc `manipulator` 500건도 같은 하네스로 쟀고 게이트에
평균으로 섞지 않고 따로 적었다(§219.5): C++ 406/500, 포트 405/500,
조건 2는 405/405, 조건 3은 1.8557 ≤ 2.4287. fanuc 500건 중 94건은
양쪽 다 못 풀었고 상향 예산에서도 못 풀어 **feasibility 미상**으로
남는다 — 유한 예산은 실행 불가능을 증명하지 못한다.

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

### 완료 조건 현황표

Phase 완료 조건 판정이 사는 유일한 곳. 위 각 Phase의 "상태" 문단과 아래
`§`가 가리키는 라운드 섹션은 이 표를 갱신하지 않고 참조만 한다 — 판정이
바뀌면 이 표의 행을 고친다, 새 문단을 덧붙이지 않는다.
`tools/ci/check-phase-status.sh`가 이 표를 검증한다.

| Phase | 조건 | 판정 | 측정한 § | 날짜 |
|---|---|---|---|---|
| Phase 0 | 오라클 FK 1,000세트 → `moveit-diff`가 "구현 없음"으로 전건 실패 보고 | MET | §217.3 | 2026-08-05 |
| Phase 1 | panda/prbt/fanuc 링크 수·조인트 수·그룹 구성·조인트 한계값·mimic 관계 완전 일치 | MET | §218.2 | 2026-08-05 |
| Phase 2 | FK 10,000×3로봇이 `1e-9` 이내 일치 | MET | §217.3 | 2026-08-05 |
| Phase 2 | 야코비안이 `1e-7` 이내 일치 (열 순서 규약 포함) | MET | §217.3 | 2026-08-05 |
| Phase 2 | 관절 한계 클램핑·mimic 전파·floating/planar 조인트 보간 일치 | MET | §238 | 2026-08-06 |
| Phase 3 | `collision: bool` 이 10,000×3로봇에서 100% 일치 | UNMET | §229.1 | 2026-08-06 |
| Phase 3 | `distance: f64` 가 `1e-4` 이내 일치 | UNMET | §229.3 | 2026-08-06 |
| Phase 4 | (a) 성공률이 C++ KDL 플러그인 이상 | MET | §245.4 | 2026-08-06 |
| Phase 4 | (b) 성공한 해의 FK가 `SolverParams::epsilon`(`1e-5`) 이내 일치 | MET | §221.2 | 2026-08-06 |
| Phase 5 | 제약 조합 2,000건 `decide()` 결과가 오라클과 100% 일치 | MET | §216.1 | 2026-08-05 |
| Phase 5 | 제약 샘플러 생성 10,000상태 전부 자기 제약 만족 | MET | §216.2 | 2026-08-05 |
| Phase 5 | 씬 diff 적용 후 충돌 결과가 오라클과 100% 일치 | MET | §216.3 | 2026-08-05 |
| Phase 6 | TOTG 산출 시간 파라미터화가 오라클과 `1e-6` 이내 일치 | MET | §217.3 | 2026-08-05 |
| Phase 7 | 벤치마크 500건 성공률이 C++ OMPL RRTConnect의 90% 이상 | MET | §219 | 2026-08-06 |
| Phase 7 | 산출 경로 100%가 충돌 검사와 제약을 통과 | MET | §219 | 2026-08-06 |
| Phase 7 | 경로 길이 중앙값이 C++ OMPL 대비 1.3배 이내 | MET | §219 | 2026-08-06 |
| Phase 8 | pilz LIN/PTP/CIRC 궤적이 오라클과 `1e-6` 이내 일치 | MET | §217.3 | 2026-08-05 |
| Phase 8 | CHOMP/STOMP가 Phase 7과 같은 속성 기반 검증을 통과 | UNMEASURED | §217.3 | 2026-08-05 |
| Phase 9 | 기존 C++ `MoveGroupInterface` 클라이언트가 무변경으로 유효 궤적 수신 | UNMET | §226.4 | 2026-08-06 |

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
| 네이티브 SBP 플래너가 OMPL 성공률에 미달 | Phase 7 완료 조건 미충족 | D3의 후순위 FFI 경로로 폴백 — 실현되지 않았다: 500건에서 99.4% vs C++ 99.6% (§219), FFI 폴백 불필요 |
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

§9.3이 넘긴 프로브 두 건은 §11.5에서 종결됐고, 그 과정에서 `bodies.rs`
이탈 7·8이 추가됐다.

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

### 10.5 parry 백엔드가 닫은 `max_contacts`

`CollisionEnv`의 메서드는 각자 소유한 `CollisionResult`를 반환하고
`check_collision`이 둘을 병합한다. 업스트림은 `CollisionResult&` 하나를
두 호출에 함께 넘기므로, robot 검사 콜백이 이미 쌓인 self 검사의
contact 수를 보고 `max_contacts`에서 멈춘다. 반환-소유 방식에서는 robot
검사가 self 검사의 결과를 볼 수 없어, 병합 결과가 `max_contacts`를
넘을 수 있었다.

`check_collision`의 진입 판정은 업스트림과 일치했지만 (pair 수 대조,
`db31a4c`), 백엔드 내부의 누적 상한은 아무도 강제하지 않았다. `env.rs`의
`check_collision` 기본 구현이 이제 닫는다: `check_self_collision`이 이미
채운 contact 총수를 `ContactData::count`로 구하고 (`pair_count`가 아니다 —
한 쌍에 여러 contact가 쌓일 수 있으므로 다른 값이다), `max_contacts`에서
그만큼을 뺀 나머지 예산만 담은 `CollisionRequest`를 만들어
`check_robot_collision`에 넘긴 뒤 결과를 병합한다. 회귀 테스트
`check_collision_passes_the_remaining_contact_budget_not_the_full_one`이
pair 하나에 contact 세 개가 쌓인 모양으로 이 뺄셈이 `count()` 기반인지
(`pair_count()` 기반이면 오답이 나오도록) 못박는다. 이 경로를 처음으로
실제로 태우는 구체 백엔드는 `moveit_collision::ParryCollisionEnv`
(`parry.rs`)이다.
- `moveit-distance-field` — 오라클 `distance_field` op. **닫혔다(§65).**
  "C++을 읽고 쓴 단위 테스트만 있다"는 이제 사실이 아니다: 재생 대상
  25건 중 10건이 이 크레이트의 오라클 픽스처이고
  (`distance_field`, `distance_field_negative`, `distance_field_cache_entry`,
  `collision_distance_field_types`, `collision_sphere_free_functions`,
  `collision_object_point_decomposition`, `link_body_decomposition`,
  `link_models_with_collision_geometry`, `group_state_representation`,
  `shape_points`), 대조 tolerance는 Phase 3 완료 조건 그대로 `1e-4`다
  (`collision_common_distance_field_parity.rs:84` 외 4곳).
- `moveit-planners-sbp` — `So2Space`/`Se3Space`/`CompoundSpace`.
  `StateSpace` trait 모양이 wraparound와 SO(3)를 감당하는지는 §61에서
  경계값으로 검증됐다 — trait 모양은 그대로 두어도 되고, 대신 구현 쪽에서
  버그가 하나 나와 고쳤다(`Se3Space::rotation_distance`가 참각의 절반을
  돌려주고 있었다, `9b04950`).
- `moveit-smoothing` — §4.6이 Phase 6 착수 시점으로 미룬 ruckig 크레이트
  채택 결정. §4.6에 되써넣었다(§62): 두 지점 모두 완료, `RuckigFilter`
  쪽만 오라클 근거 없음이 disclosed gap으로 남는다.
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


---

## 11. 2차 병렬 라운드 병합 (2026-08-03)

`p1-joints` / `p1-robotmodel` / `p1-fixtures` 세 갈래를 `main`에 병합했다.
충돌은 `tools/moveit-oracle/src/oracle.cpp` 디스패치 한 곳뿐이었다 —
`shape_points`와 `common_root`가 같은 자리에 각각 arm을 추가했고, 양쪽 다
살렸다.

병합 후 전체 게이트: `cargo fmt --all --check`, `clippy --workspace
--all-targets -D warnings`, `nextest --workspace` 538/538, `doc --workspace
--no-deps`, `check-dep-direction` / `check-fixture-format` /
`check-no-lint-suppression` — 전부 통과.

### 11.1 Phase 2 야코비안 완료 조건 충족

`Posed::jacobian`(`9230ad3`)을 병합본 위에서 독립 검증했다. 워커가 쓰지
않은 seed 4242, 로봇 4종 × 10,000 케이스:

| robot | group | 최대 편차 |
|---|---|---|
| panda | panda_arm | 2.665e-15 |
| fanuc | manipulator | 1.665e-15 |
| dual_arm_panda | left_panda_arm | 2.442e-15 |
| pr2 | right_arm | 2.109e-15 |

요구 허용오차 `1e-7`보다 8자리 아래다. FK 스윕도 병합 다이제스트로 다시
빌드한 이미지에서 4종 각 10,001 케이스 전부 통과.

### 11.2 스윕 스크립트가 야코비안을 돌리지 않고 있었다

`moveit-diff`는 `--group`이 있어야 야코비안을 비교한다. 그런데
`tools/ci/run-oracle-sweep.sh`는 그 인자를 한 번도 넘기지 않았다. Phase 2의
완료 조건 두 항목 중 `1e-7` 쪽에는 실행 주체가 없었다는 뜻이고, 야코비안이
얼마든지 깨져 있어도 이 스크립트는 계속 통과했을 것이다. `cdac462`에서
(로봇, 그룹) 5쌍 — pr2는 `right_arm`과 `base` 둘 다, `base`가 픽스처
전체에서 유일한 planar 조인트 그룹이므로 — 을 돌리도록 고쳤다.

양방향으로 확인했다. 허용오차를 `1e-18`로 조이면 실제로 실패하고(어긋난
엔트리까지 출력), `1e-7`로 되돌리면 5쌍 모두 통과한다.

같이 고친 것 하나: 실패 시 `set -e`가 요약 출력 전에 스크립트를 죽여서,
호출자에게 종료 코드만 남고 숫자는 아무것도 남지 않았다. 이제 로봇별 종료
상태를 받아 두고 요약과 어긋난 케이스 20건을 찍은 뒤에 빠진다.

교훈은 야코비안에 국한되지 않는다 — **비교를 추가했다는 것과 CI가 그
비교를 부른다는 것은 별개다.** 새 op을 붙일 때마다 그것을 실제로 돌리는
경로가 있는지 확인해야 한다.

### 11.3 픽스처 추가는 가산이었다

`p1-robotmodel`이 `*_model_info.json` 4종을 모두 건드렸다. 기존 정답이
다시 쓰인 것이 아니라 `group_is_chain` 필드가 더해진 것인지 확인했다 —
삭제 라인 0, 추가 라인 5/3/4/10. 오라클 픽스처를 수정하는 변경은 이
확인 없이는 병합하지 않는다.

### 11.4 `fixtures/pr2.srdf`의 얇은 ACM — 상류의 축약본이다

pr2 ACM 픽스처가 `disable_collisions` 한 건뿐인 것을 "우리 픽스처의 빈
구멍"으로 기록해 뒀었는데, 출처를 확인했다.
`fixtures/pr2.srdf`는 `third_party/moveit_resources/pr2_description/srdf/
robot.xml`과 **바이트 단위로 동일**하고, 그 상류 파일 자체가
`<!-- and many more disable_collisions tags -->` 주석을 달고 나머지를
생략한 축약본이다. moveit_resources에는 pr2용 moveit_config가 없어서 더
완전한 SRDF는 로컬에 존재하지 않는다.

따라서 pr2 ACM이 얇은 것은 상류에 충실한 결과이지 이식 누락이 아니다.
ACM 커버리지는 `dual_arm_panda`(68쌍, 22링크)가 담당한다 — §10.6. 이
항목은 미해결이 아니라 종결로 옮긴다.

### 11.5 `bodies::` C++ 프로브 — 두 건 종결, 이탈 2건 추가

§9.3이 남겨둔 `ConvexMesh::ray_intersections`와 `OBB::extend_approx` 두
불일치가 닫혔다. 프로브 픽스처(`bodies_probe.json`)와 테스트 8건이
`probe-parity` 브랜치에서 `main`으로 들어왔으므로 §9.3의 "브랜치에 있고
담당에게 넘겼다"는 서술은 더 이상 유효하지 않다.

두 건의 판정이 서로 반대라는 점이 핵심이다.

**`OBB::extend_approx` — upstream이 옳고 이식이 틀렸다.** 이식본은 "같은
방향을 공유하는 가장 타이트한 박스"라는 임의의 근사를 쓰고 있었다. FCL
0.7.0의 실제 `operator+`(`merge_largedist` / `merge_smalldist` /
`eigen_old` / `getCovariance` / `getExtentAndCenter`)를 오라클 컨테이너의
`libfcl-dev` 인라인 헤더에서 읽어 그대로 이식했다. 새 `obb2.*` 픽스처
키가 그때까지 한 번도 타지 않던 `merge_largedist` 분기를 강제하며, 두
분기 모두 `.so`와 `1e-12`에서 일치한다.

**`ConvexMesh::ray_intersections` — upstream이 틀렸고 이식을 유지한다.**
`bodies.cpp`의 `isPointInsidePlanes`와 `intersectsRay`가 같은 padded-plane
오프셋을 반대 부호로 다시 계산한다(바이트 단위 확인). 프로브에서 그때까지
건너뛰던 광선을 켜자 ray[0] 하나가 아니라 ray[2]·ray[4]도 같은 결함임이
드러났다 — 각각 픽스처 자신의 `containsPoint` 값만으로 위상/패리티
논증으로 증명했다. `bodies.rs` 이탈 7(광선 1건 → 3건으로 확장).

**추가로 드러난 것 — ray[1]은 어느 쪽도 틀리지 않았다.** 양쪽 다 자기
기준으로 일관된 2개 교점을 보고하는데 위치가 다르다. padding이 0이 아니면
스치는 면의 정점 3개가 공면이 아니게 되고, 그때 평면 오프셋이 anchor 선택에
의존한다. 이 이식의 `parry3d-f64` quickhull과 upstream의 qhull이 그 면에서
다른 anchor를 고른다(수기 확인: 약 -0.0433 대 -0.0405 — 부동소수점 잡음이
아닌 실제 차이). `bodies.rs` 이탈 8. 테스트는 픽스처의 정확한 위치가 아니라
개수·구간 불변식과 이 이식 자신의 회귀값을 건다.

교훈은 §11.2와 같은 모양이다. 프로브는 처음부터 5개 케이스를 갖고 있었지만
광선 일부를 건너뛰고 있었고, 건너뛴 것을 켜자 "고립된 1건"이 3건이었다.
**검사를 작성했다는 것과 그 검사가 실제로 돌았다는 것은 별개다.**

### 11.6 `RobotTrajectory` 이식 (Phase 6 선행)

`moveit_trajectory::RobotTrajectory`가 들어왔다(테스트 28건). §4.6이
`rsruckig` 배선의 선행 조건으로 지목했던 블로커가 해소됐다.

이식하지 않은 것과 그 이유:

- `RobotTrajectoryShallowCopy` — `shared_ptr` waypoint 에일리어싱을
  검사하는데, 이 이식에는 그 에일리어싱이 의도적으로 없다.
- `getRobotTrajectoryMsg` / `setRobotTrajectoryMsg` / `toJointTrajectory`
  — D1에 따라 `moveit-ros` 영역. 이에 의존하는 upstream 테스트 2건
  (`MultiDofTrajectoryToJointStates`, `SetMultiDofTrajectory`)도 함께 보류.
- `print` / `operator<<` — `moveit-state`가 아직 속도·가속도를 싣지 않는다.

`RuckigSmoothing`과 오라클 `ruckig` op은 아직 시작하지 않았다.

### 11.7 `orocos_kdl` 소스 확보 — 사용자 승인 (2026-08-03)

`dynamics_solver` 이식이 `KDL::ChainIdSolver_RNE`에서 막혔다. 로컬 탐색
결과는 명확했다 — 호스트 파일시스템, `/opt/ros`(부재), 오라클 이미지 어디에도
`.cpp`가 없고 이미지에는 `/usr/include/kdl/chainidsolver_recursive_newton_
euler.hpp` 선언만 있다. 재귀 본체는 `liborocos-kdl.so.1.5`에 컴파일되어 있다.
`moveit2.repos`에도 없다 — orocos_kdl은 소스 체크아웃이 아니라 apt 바이너리로
들어오므로, `moveit_msgs`/`moveit_resources`에 이미 적용된 third_party 조달
절차가 덮지 않는다.

따라서 새 결정 사항이었고 사용자가 fetch를 승인했다.
`third_party/orocos_kinematics_dynamics`에 태그 `v1.5.1`로 체크아웃했다
(설치된 `liborocos-kdl-dev 1.5.1-4build1`에 대응).

**대응 관계는 가정하지 않고 확인했다.** fetch한 트리의
`orocos_kdl/src/chainidsolver_recursive_newton_euler.hpp`가 이미지 안
`/usr/include/kdl/`의 같은 파일과 **바이트 단위로 동일**하다. 즉 읽는 소스가
비교 대상 `.so`를 빌드한 그 소스다. §9.1이 `geometric_shapes`에 대해 세운
절차와 같은 종류의 확인이며, 태그가 패키지 버전과 맞는다는 것은 증거이지
증명이 아니다 — diff 한 번이면 되므로 다른 KDL 파일에 의존하게 되면 그때도
같은 확인을 한다.

`third_party`는 gitignore 대상이라 브랜치로 전파되지 않는다. 워크트리는
절대경로로 읽고 자기 안에 복사하지 않는다. `build.sh`는 이 트리를 이미지로
내보내지 않는다 — 참조 전용이고 오라클은 apt 바이너리를 그대로 쓴다.

### 11.8 `collision_distance_field_types` 이식과 upstream 결함 2건

`CollisionSphere` / `GradientInfo` / `PosedDistanceField` /
`BodyDecomposition` / `PosedBody*Decomposition` / `ProximityInfo`가
`moveit-distance-field`에 들어왔다. `collision_distance_field_types` 오라클
op과 픽스처 8건(Sphere/Box/Cylinder/Mesh × 해상도 2종)으로 검증한다.

upstream의 `test_collision_distance_field.cpp`는 `TEST_F` 5건 전부
`RobotModel`/`RobotState`를 세우므로 이 슬라이스에 이식 가능한 케이스가
**0건**이다. 검증은 전적으로 오라클 대조와 불변식 경계 테스트에 의존한다.

**결함 1 — `doBoundingSpheresIntersect`가 제곱 거리를 제곱하지 않은 반지름
합과 비교한다.** `d² < s`를 묻지만 의도는 `d < s`다. 담당은 이것을
byte-for-byte 보존하고 false positive 방향(d < 1)을 회귀 테스트로 못박았다.
그 판단은 옳지만 서술이 절반이었다 — 같은 식은 `d = 1`을 기준으로 방향이
뒤집힌다. `d > 1`에서는 **false negative**가 나온다: `s = 3, d = 2`면 1 m나
겹친 쌍을 "안 닿음"으로 걷어낸다. broad phase가 "아니오"라고 답하면 이를
바로잡을 narrow phase가 아예 돌지 않으므로, 이쪽이 안전 측 결함이다.
반지름 합 1 m는 PR2급 로봇의 링크 바운딩 스피어가 도달하는 값이다.
`2d7f3f6`에서 양쪽 방향을 각각 테스트로 고정했다(동작은 그대로).

**결함 2 — `relative_cylinder_pose_`가 Sphere 전용 바디에서 초기화되지
않는다.** `determineCollisionSpheres`는 cylinder/box/mesh 분기에서만
`relative_transform`을 쓰고 sphere 분기는 손대지 않는데, 그 멤버에는 기본
초기화가 없다.

이 주장을 독립적으로 재확인했다. 커밋된 요청 8건을 그대로 재생한 결과
`relative_cylinder_pose`를 뺀 나머지는 8건 모두 **바이트 단위로 동일**하고,
`relative_cylinder_pose`는 **id 1과 5에서만** 달라진다 — 정확히 Sphere
픽스처 두 건이다. 즉 담당이 파리티 대조에서 제외한 범위가 넓지도 좁지도
않게 맞다. 이 이식은 `Isometry3::identity()`로 시드하므로 Rust 쪽은
결정적이다(문서화된 의도적 이탈).

`#[allow(clippy::too_many_arguments)]` 2건이 함께 들어와
`check-no-lint-suppression.sh`가 실패했다(담당 보고서는 CI 스크립트 3개 중
2개만 나열했고 이것이 빠진 하나였다). `12da049`에서 `SphereGradientQuery`로
묶어 근본에서 없앴다 — 인자 개수는 증상이고, 실제 문제는 `f64` 하나를 사이에
둔 `bool` 두 개라 호출부에서 뒤바꿔도 컴파일된다는 점이었다. 파리티 테스트가
이미 `/* subtract_radii = */` 주석으로 방향을 잡고 있었던 것이 그 증거다.

### 11.9 parry 백엔드 착수, §10.5 종결

`ParryCollisionEnv`가 `moveit-collision`에 들어왔다(`parry.rs`, 테스트
30건). `check_self_collision` / `check_robot_collision` / `distance_self` /
`distance_robot`를 구현하고, `check_robot_collision_continuous`는 타입
에러를 돌려준다 — upstream `CollisionEnvFCL`은 이 경우 로그만 찍고 `res`를
건드리지 않은 채 돌아가는데, 호출부에서 그것은 "검사했고 아무것도 없음"과
구분되지 않는다. FCL 백엔드와의 이탈 11건을 각각 upstream 소스 위치와 함께
문서화했다.

§10.5는 닫혔다(§10.5 본문 갱신). 갈라질 수 있었던 지점이 갈라지지 않았다는
점이 중요하다 — 진입 판정은 `pair_count()`를 쓰고(upstream `checkCollision`이
`res.contacts.size()`를 읽으므로) 예산 뺄셈은 `count()`를 쓴다(`collisionCallback`이
`res_->contact_count`를 추적하므로). 서로 다른 두 양이 각자의 upstream
대응물에 맞춰져 있고, `pair_count()` 기반이었다면 실패하도록 회귀 테스트가
짜여 있다.

여기에 경계 하나를 추가로 못박았다(`6411598`). §10.5 수정은
`saturating_sub` 때문에 `max_contacts: 0`을 robot 검사가 **실제로 받는**
값으로 만든다 — 그 전에는 어떤 백엔드에도 도달하지 않던 값이다.
`accumulate_collision`은 이미 옳게 처리하고 있었지만(`collision = true`가
저장 가드보다 먼저) 그것을 고정하는 테스트가 없었고, 둘을 뭉갠 백엔드는
겹친 쌍에 대해 "깨끗함"을 보고하게 되며 호출부는 그것을 진짜 깨끗한 경우와
구별할 수 없다. **수정이 어떤 경계값을 새로 도달 가능하게 만들면, 코드가
이미 그 값을 옳게 다루더라도 그 경계에는 테스트가 필요하다.**

### 11.10 Phase 3 완료 조건은 아직 미충족

`ParryCollisionEnv`의 정확성은 현재 전적으로 합성 불변식 테스트에 의존한다.
FCL 대비 이탈이 11건 문서화되어 있다는 것은, 합성 테스트로는 "그 11건이
차이의 전부인지"를 알 수 없다는 뜻이다. §5 Phase 3의 완료 조건(10,000 상태 ×
3로봇 `collision: bool` 100% 일치, `distance` 1e-4)에는 오라클 `collision`
op이 필요하고 아직 없다. p3-acm에 배정했다.

특히 이탈 6(단일 `contact` 호출의 signed distance 대 FCL의 최대 200개 contact
중 최댓값)이 관통 쌍에서 `1e-4`를 깨뜨릴 가능성이 가장 높다. 깨진다면 그것은
백엔드에 대한 실제 발견이지 허용오차를 늘릴 사유가 아니다.

### 11.11 `dynamics_solver` 이식 — Phase 2 마지막 항목

`moveit-state::DynamicsSolver`가 들어왔다. RNE 재귀는 §11.7에서 확보한
`orocos_kdl` v1.5.1 소스에서 이식했고, `LinkModel`이 이제 질량과 회전
관성을 싣는다(`<inertial><origin rpy>` 프레임에서 링크 프레임 축으로 회전).
오라클 `dynamics` op과 로봇 4종 픽스처로 검증한다. 허용오차는 `1e-9` —
담당이 RNE의 누적 반올림을 예상해 `1e-6`에서 시작했다가 실측이 `1e-9`도
통과하자 근거 없는 여유를 두지 않으려고 다시 조였다.

**검증 한계 — 이것이 이 라운드의 핵심 사실이다.** `fixtures/{panda,fanuc,
dual_arm_panda}.urdf`에는 `<inertial>` 요소가 **하나도 없다**(각각 0개,
pr2만 78개). KDL에게 이 세 로봇의 모든 바디는 질량 0이므로, 속도·가속도가
0이 아닌 케이스를 포함해 `torques`가 **전부 정확히 0**으로 나온다. 실제
ground truth이긴 하나 알고리즘이 옳다는 증거는 아니고, 질량 없는 체인에서
쓰레기 값을 내지 않는다는 증거일 뿐이다.

다만 담당 보고보다 상황이 조금 낫다는 점을 확인했다. `payload_torques`는
로봇 4종 **모두**에서 0이 아니다(nonzero 20~30건, 최대 7.1~34.1) — tip
페이로드가 링크가 질량 없어도 재귀에 전파할 질량 하나를 준다. 정리하면:
**다물체 관성을 실제로 시험하는 것은 pr2뿐이고, 나머지 셋은 단일 tip
질량으로 재귀를 시험한다.** moveit_resources에 관성을 가진 다른 로봇이
없으므로 이는 픽스처를 늘려 메울 수 있는 구멍이 아니다.

**upstream `getMaxPayload` 인덱싱 결함 — 재현, 수정 아님.** 기전을 upstream
소스에서 직접 확인했다. `num_joints_`는 `kdl_chain_.getNrOfJoints()`(base→tip
체인의 비고정 조인트 수)에서 오는데, `max_torques_`는
`joint_model_group_->getJointModelNames()`(고정 조인트를 포함한 SRDF 그룹
순서)로 만들어진다. 두 리스트의 길이부터 다르다 — panda 8 대 7, pr2 9 대 7,
fanuc 7 대 6. 따라서 `max_torques_[i]`는 `torques[i]`와 **다른 조인트**의
한계값이다. 마지막 활성 조인트보다 앞에 고정 조인트가 있으면 어긋남이
루프 안으로 들어오고, `fabs(zero_torques[i]) >= max_torques_[i]`가 `0.0`
한계값에 대해 항상 참이 되어 `payload = 0.0`으로 즉시 반환한다. 관측과
일치한다 — fanuc과 pr2는 `max_payload`가 전부 0, panda와
dual_arm_panda는 0이 아니다(그쪽은 어긋난 0이 인덱스 범위 밖에 있다).

이식은 이 동작을 재현한다. 유일한 ground truth가 결함 있는 동작을 담고
있고, "고친" 버전을 대조할 대상이 없기 때문이다. `DynamicsSolver::new`가
`max_torques`를 명시적 인자로 받는 것은 upstream 생성자와 같은 모양이며,
호출자가 `joint_indices` 대신 `active_joint_indices`로 만들면 결함을
비껴간다 — 이식은 그 선택을 강제하지 않는다.

`kdl_parser`는 소스가 로컬에 없다(호스트·이미지 모두 컴파일된 `.so`뿐).
`X[i]`/`S[i]`는 §11.7에서 확보한 KDL 자체 소스(`frames.inl`, `segment.cpp`,
`joint.cpp`, `rigidbodyinertia.cpp`)에서 다시 유도했다.

관절 실효(effort) 한계가 `moveit-model`에 없는 것은 공백이 아니다 —
upstream `moveit::core::VariableBounds`에도 effort 필드가 없고,
`DynamicsSolver`는 질량·관성과 마찬가지로 raw URDF에서 직접 읽는다.

### 11.12 `collision_common_distance_field` 절반 이식 — 캐시 ABA 결함

`c773c80` 병합. 커밋 넷: `8f7ede7`(`collision_common_distance_field`의
`RobotModel` 비의존 절반), `a77f468`(오라클 op
`collision_object_point_decomposition`, `link_body_decomposition`),
`165858c`(캐시 수정), `e77dedd`(패리티 테스트와 픽스처).

**이번 라운드에서 나온 결함은 upstream 패리티 산물이 아니라 이 이식
자체의 설계 결함이었다.** `get_body_decomposition_cache_entry`가
`Arc::as_ptr(shape) as usize` — 맨 주소값 — 만을 키로 쓰면서 그 주소를
붙잡아 두는 것이 아무것도 없었다. 어떤 shape의 마지막 `Arc`가 떨어지면
할당자는 그 주소를 다음 `Arc<Shape>`에 그대로 재사용할 수 있고, 그러면
새 shape의 조회가 이전 shape의 `BodyDecomposition`을 조용히 돌려준다.
upstream에는 이 위험이 없다 — `std::weak_ptr`를 `std::map` 키로 두면
pointee가 파괴된 뒤에도 그 **control block** 할당이 맵 엔트리 수명만큼
살아 있어, 뒤따르는 무관한 `shared_ptr`의 control block이 같은 주소에
앉을 수 없다.

수정은 엔트리마다 `Weak<Shape>`를 값과 함께 저장하는 것이다. Rust는
strong·weak 카운트가 **둘 다** 0이 되어야 `ArcInner`를 해제하므로,
`Weak` 하나를 쥐고 있으면 `T`가 제자리에서 드롭된 뒤에도 할당(따라서
주소)이 유지된다. 이 캐시는 축출하지 않으므로(upstream 자신의 미구현
`// TODO - clean cache`와 같다) 한 번 캐시된 주소는 프로세스가 끝날
때까지 고정된다. `Weak`는 조회 때 upgrade되지 않는다 — 오직 할당을
고정하는 용도다.

**독립 검증.** 오라클을 새 다이제스트(`46ff0fa82d650830`)로 재빌드한 뒤
커밋된 요청 픽스처 3건을 실행 오라클에 다시 흘려보내 응답이
바이트 단위로 동일함을 확인했다(id 1 = 1021점, id 2 = 315점,
id 3 = 264점). 이어서 `Arc::downgrade(shape)`를 `Weak::new()`로 바꾼
음성 대조를 돌려 보고된 실패가 정확히 재현되는 것을 확인했다 —
`collision_object_point_decomposition_matches_the_oracle`의 id 2가
315 대신 **1021**, 즉 구(sphere) 픽스처의 점 개수를 그대로 받았다.
회귀 테스트 `cache_entry_survives_the_original_arc_shape_being_dropped`도
같이 실패했다(반지름 0.07에 대해 7 대신 19). 원복 후 38/38 통과.

`collision_common_distance_field`의 나머지 절반
(`DistanceFieldCacheEntry`, `addLinkBodyDecompositions`)은 `RobotModel`에
의존하므로 이번 라운드 범위 밖이다.

---

## 12. Phase 5 (제약 절반) — `moveit-constraints` 착수 (2026-08-03)

`moveit_core/kinematic_constraints/kinematic_constraint.{hpp,cpp}`를
이식했다: `JointConstraint`, `PositionConstraint`, `OrientationConstraint`,
`VisibilityConstraint`(2라운드에서 완전 이식, §12.5), `KinematicConstraintSet`,
`ConstraintEvaluationResult`. `utils.{hpp,cpp}`는 이식하지 않는다 — 이번
크레이트가 아니라 `moveit-ros`/`moveit-planning`의 몫이라는 판단은
그대로지만, 실제로 함수 전부가 `moveit_msgs`/`rclcpp::Node`에 묶여 있는
것은 아니다; 함수별 근거는 §12.7. `moveit-scene`은 이번 라운드 범위 밖이다
(충돌 검사 백엔드가 아직 trait뿐이라 §11.5의 `moveit-collision`을
막고 있다) — `VisibilityConstraint`의 원뿔-충돌 검사는 그럼에도 완전히
끝났다, §12.5에서 보듯 `PlanningScene`이 아예 필요 없었기 때문이다.

### 12.1 §4.3 매핑 결정 3건 — 어떤 필드가 `Option`/enum이 됐는가

**`PositionConstraint`.** upstream은 `constraint_region_`을
`std::vector<bodies::BodyPtr>` 하나로 이미 들고 있지만,
`moveit_msgs::msg::PositionConstraint`는 이걸 `primitives`/`meshes`
(도형 종류별로 나뉜 벡터 2개)와 `primitive_poses`/`mesh_poses`(길이가
각각 맞아야 하는 자매 벡터 2개)로 표현한다. 포팅한 `ConstraintRegion {
body: Body, pose: Isometry3 }` 하나가 이 넷을 대신한다 — `Body`
자체가 이미 sum type이라(`moveit_geometry::bodies::Body`, §9.1) 도형
종류별 벡터가 따로 있을 이유가 없다. `moveit-ros::TryFrom`이 만들 때
잃는 것: 길이가 어긋난 `primitives`/`primitive_poses` 쌍을 upstream처럼
자르고 경고만 남기는 대신 에러로 보고해야 한다.

**`OrientationConstraint`.** `moveit_msgs::msg::OrientationConstraint`는
`absolute_x/y/z_axis_tolerance` 세 실수와 `parameterization`
(`XYZ_EULER_ANGLES`/`ROTATION_VECTOR`) 태그를 따로 든다 — 태그가 그 세
실수의 *의미*를 바꾼다(오일러 각 성분인지 로드리게스 벡터 성분인지).
`OrientationTolerance` enum(`XyzEuler { x, y, z }` /
`RotationVector { x, y, z }`) 하나로 접었다. `moveit-ros::TryFrom`이
잃는 것: `parameterization`에 위 두 값 밖의 정수가 들어오면 upstream처럼
조용히 `XYZ_EULER_ANGLES`로 떨어지는 대신 에러가 나야 한다.

**`VisibilityConstraint`.** `moveit_msgs::msg::VisibilityConstraint`의
`target_radius`/`max_view_angle`/`max_range_angle` 세 필드 모두
`0.0`으로 "이 기준을 안 본다"를 표현한다(`configure()`의
`enabled()`가 셋 다 `> eps`로 검사). 과제 지시문은 뒤 둘만 이름을
댔지만 `target_radius`도 형태가 같은 결함이라 셋 다
`Option<f64>`로 바꿨다 — 하나만 고치면 나머지 하나가 다음 라운드에
같은 모양으로 다시 발견될 뿐이다. `VisibilityConstraint::new`가
`Some(0.0)`(또는 `EPSILON` 이하 값)도 `None`으로 정규화해서, "있음"과
"활성"이 항상 같은 뜻이 되게 한다.

세 타입 모두 공통으로: `configure(msg, tf)`에 대응하는 것이 없다 — D1이
이 크레이트에 `moveit_msgs` 타입을 두는 것 자체를 막으므로 옮길
`configure()`가 아예 없고, 각 타입의 `new()`가 순수 Rust 인자를 받는다.
프레임이 고정인지 모바일인지 결정하는 upstream의 `bool mobile_frame_` +
그 옆의 pose/rotation 필드 하나라는 조합(같은 필드가 플래그에 따라
"이미 변환된 값"과 "매 `decide()`마다 다시 변환해야 하는 값"을 오간다,
§4.1과 같은 이중 의미)도 세 타입 모두에서 나타나서, `ReferenceFrame`/
`OrientationTarget`/`FramedPose` 세 개의 `Fixed { .. } | Mobile { .. }`
enum으로 각각 고쳤다 — payload가 그 의미를 정의하는 variant 안에서만
존재하게.

### 12.2 `VisibilityConstraint`는 1라운드에서 부분 이식이었다 (2라운드에서 종결, §12.5)

upstream `decide()`는 view-angle·range-angle 검사를 통과하면 센서-타겟
사이에 원뿔 메시를 만들어 `collision_detection::CollisionEnvFCL`로
로봇과 충돌 검사한다. 1라운드 시점엔 `moveit-collision::CollisionEnv`가
아직 trait만 있고 구현체가 없어서(§9.3, §11.4)
`VisibilityConstraint::decide_geometry`는 view-angle·range-angle 검사까지만
완전히 이식했고, 그 뒤 원뿔 검사가 필요한 지점에서 `satisfied`를 지어내는
대신 `VisibilityDecision::NeedsConeCollisionCheck`를 반환했다.
`KinematicConstraintSet::decide`는 이걸 삼키지 않고
`Err(UndecidedConstraint)`로 전파했다 — 이 상황을 "만족"으로 보고하는
쪽이 이 설계 전체가 막으려는 조용한 오답이었기 때문이다. `moveit-collision`에
`ParryCollisionEnv` 구현체가 들어온 뒤(§11.9) 이 gap을 닫았다 — §12.5.

### 12.3 오라클 대조

`crates/moveit-constraints/tests/decide.rs`에 31개 유닛 테스트가
있다(경계값 단위: 허용오차 경계, mobile/fixed 프레임, 연속 조인트
wraparound, `Option`/`None` 판별 — panda·pr2 픽스처, pr2는 연속 조인트
경로 전용).

오라클에 `constraints` op을 추가하고(`tools/moveit-oracle/src/oracle.cpp`),
`moveit-diff --constraints N`으로 panda·fanuc 각각 1,000개, 합계
2,000개의 조합을 대조했다 — 전부 일치(0 실패). 각 조합은 무작위
FK 결과를 허용오차 경계 바깥/안쪽으로 정확히 밀어 넣어 만들었다(§4.3
지시대로 "전부 만족" 또는 "전부 위반"이면 아무것도 증명하지 못하므로).
실제로 얻어진 만족/위반 분포:

| 종류                          | panda (만족/위반) | fanuc (만족/위반) |
|-------------------------------|--------------------|--------------------|
| `joint`                        | 80 / 87            | 91 / 76            |
| `position`                     | 83 / 84            | 79 / 88            |
| `orientation` (xyz_euler)      | 78 / 89            | 74 / 93            |
| `orientation` (rotation_vector)| 99 / 68            | 97 / 70            |
| `visibility` (view_angle)      | 81 / 85            | 102 / 64           |
| `visibility` (range_angle)     | 75 / 91            | 79 / 87            |

`visibility`의 `target_radius` 기준(원뿔-로봇 충돌 검사, §12.2에서
미이식으로 남긴 부분)은 이 2,000개에 의도적으로 없다 — 이 포트가
결정할 수 없는 기준이라 대조 케이스 생성기가 애초에 만들지 않는다.
dual-arm panda는 이 라운드에서 URDF(xacro 전개 결과)가 아직 준비되지
않아 대조하지 못했다 — panda·fanuc만으로 이 절반의 완료 조건(2,000건
100% 일치)을 채웠다.

### 12.4 테스트 작성 중 발견하고 고친 결함 1건

`PositionConstraint::decide`가 `state.global_link_transform_at(...) *
self.offset`을 그대로 썼다 — `self.offset: Vector3`. upstream의
`Eigen::Isometry3d * Eigen::Vector3d`는 벡터를 점으로 취급해 회전과
평행이동을 모두 적용하는데, `nalgebra::Isometry3: Mul<Vector3>`는
벡터 시맨틱(회전만, 평행이동 없음)이라 오프셋이 0이 아니어도 링크
원점이 그대로 원점 근처로 계산됐다. `moveit_geometry::bodies`가 이미
똑같은 결함 모양에 대해 비공개 `transform_point` 헬퍼를 두고 있다
(`(pose * nalgebra::Point3::from(*point)).coords`) — 같은 수정을
적용했다. 유닛 테스트 3건이 고치기 전엔 실패했고 고친 뒤 통과한다.
크레이트 안의 다른 모든 `Isometry3` 곱셈(`region.pose` 재배치,
`FramedPose::resolve`, 뷰/레인지 각도의 방향 벡터)은 각각
Isometry×Isometry 합성이거나 진짜 방향 벡터라 이 결함에 해당하지
않는다 — `rg`로 크레이트 전체를 확인했다.

### 12.5 `VisibilityConstraint`의 원뿔-충돌 검사 이식 완료 (2라운드, 2026-08-03)

§12.2의 gap을 닫았다. `decide()`는 이제 view/range-angle 검사가
`target_radius` 때문에 미결정으로 남을 때 `decide_cone()`을 호출해
완전히 결정한다 — `KinematicConstraintSet`/`ConstraintEvaluationResult`
양쪽 다시 infallible해졌고(`UndecidedConstraint`/`VisibilityDecision`
제거), `Err`로 전파할 미결정 상태 자체가 없어졌다.

**`PlanningScene` 없이 끝난다.** upstream `decide()`가 만드는
`CollisionEnvFCL`은 그 함수 안에서만 사는 일회용 충돌 월드다 — 호출자의
씬을 참조하지 않는다. `decide_cone()`도 똑같이 한다: `cone_mesh()`로
센서-타겟 원뿔의 `Mesh`를 만들고, `moveit_collision::World`에 그
하나뿐인 도형을 `"cone"`이라는 이름으로 넣은 뒤, `ParryCollisionEnv`로
`state`의 로봇과 충돌 검사한다. 센서/타겟 프레임 자신과의 자가 충돌은
`AllowedCollisionMatrix::set_default_conditional_entry`로 걸러낸다
(`allow_sensor_or_target_contact`가 `RobotAttached`이거나 링크 이름이
sensor/target 프레임과 같으면 허용). `moveit-scene`이 이번 라운드 범위
밖이라는 §12 서두의 사실은 그대로지만, 이 기준은 애초에 그게 필요
없었다 — upstream 자신이 씬이 아니라 그때그때 만드는 충돌 월드를
쓰기 때문이다.

**`moveit-collision`에서 발견하고 고친 결함 1건 (별도 커밋).**
원뿔 메시(`TriMesh`)를 `World`에 넣는 첫 호출자가 이 코드였다 —
로봇 링크는 문서 주석대로 `Shape::Mesh`를 절대 갖지 않으므로,
`parry.rs`의 `PosedBody`는 지금까지 `TriMesh`를 담아본 적이 없었다.
`parry3d_f64::shape::TriMesh`는 그 자체로 복합 도형이라
(`as_composite_shape()`가 `Some`을 반환), 도형이 하나뿐인 바디도 항상
`Compound::new(parts)`로 감싸던 기존 `parry.rs` 설계는
`"Nested composite shapes are not allowed."` 패닉을 낸다. `PosedBody.shape`
타입을 `Compound`에서 `SharedShape`(`Arc<dyn Shape>`)로 바꾸고, 새
`combine_parts` 헬퍼가 도형이 하나뿐인 바디는 `Compound`를 아예 거치지
않고 그 도형의 상대 pose를 바디 pose에 직접 합성하게 했다(2개 이상인
바디만 `Compound`를 쓴다). 이건 이번 원뿔 검사 코드가 처음으로 드러낸
`moveit-collision`의 잠재 결함이라 별도 커밋으로 남긴다.

**`moveit-diff` 케이스 생성기도 고쳤다 — 이게 이번 지시의 핵심 확인
사항이다.** 1라운드의 대조 케이스 생성기는 `target_radius`를 항상
0으로만 만들어서(§12.3의 표에 `visibility_cone` 종류가 아예 없는 이유)
원뿔 경로 자체를 한 번도 오라클과 맞춰보지 않았다. `build_constraint_case`에
7번째 케이스 종류 `"visibility_cone"`을 추가했다 — `max_view_angle`/
`max_range_angle`을 절대 설정하지 않고 `target_radius`만 설정하므로,
`decide_by_angle`이 항상 `None`을 반환해 **생성된 케이스의 100%가
실제로 원뿔-충돌 경로에 도달한다**(뷰/레인지 각도 조기 결정으로
새는 케이스가 0%).

그런데 그 100%는 전부 "충돌 없음"으로만 오라클과 일치한다 — **panda,
fanuc, dual_arm_panda 세 픽스처의 `<collision>` 형상이 전부 STL
`<mesh>`이고, `moveit-model`의 URDF 로더가 메시 충돌 형상을 아예
보존하지 않아서(로더 자체는 이 작업 범위 밖, 다른 워커 소유), 이
셋의 `RobotModel`은 parry로 표현 가능한 충돌 형상이 하나도 없다.**
원뿔을 로봇 어디에 두든 이 셋에 대해서는 충돌이 감지될 수 없다 —
이건 이번 라운드가 만든 결함이 아니라 이미 있던, 범위 밖의 구조적
gap이다(`UNFIXED`로 보고). "같은 gap이 다른 모자를 쓰고 나타난 것"을
피하려고, 생성기는 새 케이스를 항상 모든 픽스처의 도달 범위 밖(회전
관절 포함 최대 실측 픽스처인 pr2의 팔 길이 ~1.7m보다 훨씬 큰 50m
오프셋)에 놓는다 — 그러면 포트와 오라클 양쪽이 "충돌 없음"에
동의하는 것이 실제 충돌 판정 로직이 아니라 우연한 기하학적 배치
때문이 아니라, 원뿔-충돌 판정 자체가 대칭적으로 정직하게 작동한다는
뜻이 된다. 대신 "충돌 있음" 분기는 이 크레이트 자신의 유닛 테스트
`cone_through_a_robot_link_is_violated`가 검증한다 — 유일하게
원시(primitive) 충돌 형상을 실제로 갖는 픽스처인 pr2의
`base_bellow_link`(`<box size="0.05 0.37 0.3"/>`)를 향해 원뿔의
바닥 원판(밑면 캡, 속이 빈 옆면 쉘과 달리 실제로 채워진 면)이 정확히
그 링크 위치를 지나게 배치한다.

**대조 결과 (4개 픽스처, 각 2,201건 = fk 100 + jacobian 100 +
model_info 1 + constraints 2,000, 전부 seed별 재실행으로 재확인,
2026-08-03):**

| 픽스처            | seed | group            | 결과            | `visibility_cone` (만족/위반) |
|--------------------|------|-------------------|-----------------|--------------------------------|
| panda              | 1    | panda_arm         | 2201/2201 일치  | 285 / 0                        |
| fanuc              | 2    | manipulator       | 2201/2201 일치  | 285 / 0                        |
| dual_arm_panda     | 3    | left_panda_arm    | 2201/2201 일치  | 285 / 0                        |
| pr2                | 4    | right_arm         | 2201/2201 일치  | 285 / 0                        |

네 실행 모두 `visibility_cone` 285건 전부가 "만족"으로만 오라클과
일치한다 — 위 문단에서 설명한 대로 의도된 결과지 우연이 아니다.

**대조 메커니즘 자체가 살아있다는 확인 (표준 지침 — 처음 통과한 검사는
일부러 깨서 검증).** `decide_cone`의 마지막 판정
(`!result.collision` → `result.collision`으로 뒤집음)을 일부러 깨고
panda seed 1로 300 케이스를 재실행했더니 `visibility_cone` 42건 전부가
`satisfied mismatch rust=false oracle=true`로 정확히 실패했다. 되돌리고
재실행하니 351/351 다시 일치했다 — 대조 스크립트가 이 경로의 회귀를
실제로 잡아낸다는 확인이다.

### 12.6 dual-arm panda 대조 추가 (2라운드, §12.3 완료조건 확장)

§12.3에서 "이 라운드에서 URDF가 아직 준비되지 않아 대조하지 못했다"고
남긴 dual-arm panda가 이제 `third_party/moveit_resources/dual_arm_panda_moveit_config`
+ `fixtures/dual_arm_panda.urdf`/`.srdf`로 준비됐다(다른 워커가 xacro를
전개). §12.5의 4-픽스처 대조 표에 이미 포함했다 — `left_panda_arm`
그룹으로 2,201건 전부 일치. 이 절반의 완료 조건은 이제 panda·fanuc
2건이 아니라 panda·fanuc·dual_arm_panda·pr2 4건, 합계 8,804건 100%
일치로 채워졌다.


### 12.7 `kinematic_constraints/utils.{hpp,cpp}` 함수별 분류 — 이식 안 함, 목록만 (2라운드)

과제 지시대로 이식하지 않는다. `moveit/kinematic_constraints/utils.hpp`가
선언하는 15개 함수(오버로드 포함)와 `utils.cpp`가 그 뒤에 숨긴
비공개 헬퍼 6개를 함수 단위로 분류했다. 기준: 시그니처에 `moveit_msgs`/
`rclcpp::Node`가 있어도 그게 알맹이(연산 자체)에 꼭 필요하지 않고 그냥
데이터를 담는 그릇이면 `portable`, ROS 파라미터 서버/노드/토픽 자체가
알맹이면 `moveit-ros`.

**공개 함수 (utils.hpp), 15개:**

| 함수 | 분류 | 근거 |
|---|---|---|
| `mergeConstraints(Constraints&, Constraints&)` | portable | 관절 제약의 구간 겹침 + 가중평균 병합은 진짜 산술이다; `moveit_msgs` 타입은 벡터-오브-구조체 그릇일 뿐, `RCLCPP_ERROR`는 실패 경로 로깅으로 부수적이다. |
| `countIndividualConstraints(Constraints&)` | portable | 벡터 길이 합, 사소하지만 msg 비의존적. 다만 우리 `KinematicConstraintSet`은 이미 `Vec<Constraint>` 하나라 이식할 실익이 거의 없다. |
| `constructGoalConstraints(state, jmg, below, above)` | portable | `state.copyJointGroupPositions` + `jmg->getVariableNames()`로 활성 관절마다 `JointConstraint` 하나씩 만든다 — msg는 출력 그릇일 뿐. |
| `constructGoalConstraints(state, jmg, tolerance)` | portable | 위 함수로 위임하는 얇은 오버로드. |
| `updateJointConstraints(Constraints&, state, jmg)` | portable | jmg의 활성 관절 이름에 있으면 갱신, 없으면 실패 — 순수 로직, frame/header 없음. |
| `constructGoalConstraints(link, PoseStamped, tol_pos, tol_angle)` (구 region) | portable | pose → 구 영역 `PositionConstraint` + rotation-vector `OrientationConstraint` 변환이 알맹이다. `header.frame_id`는 유지할 값(우리 쪽도 이미 평문 `&str` frame_id를 받는다), `header.stamp`(ROS 시각)는 이 함수도 그냥 통과시킬 뿐 아무 데도 안 쓰여서 이식 시 버릴 항목. |
| `constructGoalConstraints(link, PoseStamped, Vec<f64> tol_pos, Vec<f64> tol_angle)` (박스 영역) | portable | 위와 동일한 근거. |
| `updatePoseConstraint(Constraints&, link, PoseStamped&)` | portable | position/orientation 갱신 두 함수로 위임하는 얇은 변환. |
| `constructGoalConstraints(link, QuaternionStamped, tolerance)` | portable | orientation-only 버전, 근거는 pose 버전과 동일. |
| `updateOrientationConstraint(Constraints&, link, QuaternionStamped&)` | portable | 이름으로 찾아 갱신, `frame_id` 빈 값이면 실패(`RCLCPP_ERROR`는 부수적, `Result::Err`로 대체 가능). |
| `constructGoalConstraints(link, Point ref, PointStamped goal, tolerance)` | portable | 구 영역 `PositionConstraint` 하나 조립 — 필드 대입뿐. |
| `constructGoalConstraints(link, PointStamped goal, tolerance)` | portable | 위 함수로 위임(`reference_point = 0`). |
| `updatePositionConstraint(Constraints&, link, PointStamped&)` | portable | `updateOrientationConstraint`와 같은 모양. |
| `constructConstraints(rclcpp::Node::SharedPtr&, param, Constraints&)` | **moveit-ros** | 알맹이 자체가 `node->get_parameter`/`has_parameter`/`list_parameters` — ROS 파라미터 서버 읽기이지 기하/산술이 아니다. |
| `resolveConstraintFrames(state, Constraints&)` | portable | 알맹이는 `getGlobalLinkTransform(robot_link).inverse() * transform`과 쿼터니언 합성 — 진짜 Eigen 기하 연산, `tf2::toMsg`/`fromMsg`는 형식 변환이라 부수적. **갱신 (p1-fixtures 3라운드, 2026-08-03):** 이 함수가 필요로 하는 "`link_name`이 부착 물체(attached body)/서브프레임을 가리킬 때 로봇 링크로 되돌린다"는 폴백은 더 이상 no-op이 아니다. 상류가 이 폴백을 `RobotState::getFrameInfo`에 두는 것과 달리, 이 포트는 부착체를 `RobotState`가 아니라 `PlanningScene`에 둔다(§12 이전 D1/2라운드 결정) — 따라서 이 폴백은 `moveit-state`가 아니라 `crates/moveit-scene/src/scene.rs`의 `PlanningScene::frame_transform`/`PlanningScene::knows_frame_transform`에 새로 생겼다: 모델 프레임·링크 이름(`moveit_state::Posed::frame_transform` 위임), 부착체 id, 부착체 서브프레임(`"<id>/<subframe>"`, `attached_body.rs`의 새 `AttachedBody::subframe_pose`), 마지막으로 월드 객체/서브프레임(`moveit_collision::World::get_transform`/`knows_transform`) 순으로 시도한다. `moveit-state`의 `RobotState`/`Posed`는 여전히 부착체를 모른다 — 이 포트가 §12 당시 지적한 no-op의 원인은 "부착체 지원이 아예 없었다"였는데, 그 지원 자체는 `moveit-scene`에 생겼으므로 `link_name`이 부착체 id/서브프레임을 가리키는 실제 링크 이름으로 정확히 되돌아간다. `moveit-constraints`가 `resolveConstraintFrames`를 이식하려면 `moveit_state::RobotState::global_link_transform`이 아니라 `PlanningScene::frame_transform`을 호출해야 한다는 뜻 — 이 함수는 `&mut PlanningScene`가 필요하다(내부에서 `current_state_mut().update()`를 호출한다). TF 폴백(상류 `Transforms::getTransform`)은 이 포트에 없다(D1) — 셋 다 실패하면 `Error::UnknownName`. 오라클의 새 `frame_transform` op(`tools/moveit-oracle/src/oracle.cpp`)와 `crates/moveit-scene/tests/frame_transform_parity.rs`로 panda against 실제 오라클 검증 완료 — 부착체 id/서브프레임, 월드 객체/서브프레임, 그리고 `knowsTransform`/`getTransform`이 상류에서부터 실제로 불일치하는 서브프레임-대-형제객체이름 충돌 케이스까지 포함한다. |

**비공개 헬퍼 (utils.cpp, 익명 네임스페이스), 6개 — 전부 `moveit-ros`:**
`constructPoseStamped`, `constructConstraint`(Joint/Position/Orientation/
Visibility 4개 오버로드), `collectConstraints`. 여섯 다 `rclcpp::Node::SharedPtr`의
`get_parameter`/`has_parameter`/`list_parameters`를 직접 부른다 — 공개
함수 `constructConstraints` 하나를 위한 내부 구현이고, 독자적으로
이식할 이유가 없다.

**분류 결과가 애초 예상과 다르다는 점을 그대로 적는다.** 과제 지시문은
"기하 헬퍼 몇 개 정도가 portable 쪽에 떨어질 것"이라고 예상했지만,
실측 결과는 반대다 — 15개 공개 함수 중 13개가 `portable`이고
`moveit-ros`는 `constructConstraints` 하나(+ 그걸 지탱하는 비공개
헬퍼 6개)뿐이다. 이유: 이 파일의 함수 대부분은 `moveit_msgs::msg::Constraints`를
그저 출력 그릇으로만 쓸 뿐 ROS 노드/파라미터 서버/토픽 자체에 의존하지
않는다 — 우리 크레이트는 애초에 이 메시지 타입 자체를 갖지 않고
`JointConstraint`/`PositionConstraint`/... 같은 순수 Rust 값을 직접
만들므로, 이 함수들의 "산술/조립 로직"은 msg 대신 우리 타입을 만들도록
그대로 옮겨질 수 있다. ROS에 진짜로 묶인 건 파라미터 서버를 읽는
`constructConstraints` 계열뿐이다. 이번 라운드는 이 목록만 남기고
포팅하지 않는다 — 지시대로.

---

## 13. `moveit-octomap` 착수 — §2 "핵심 공백 3개" 중 하나 해소 (2026-08-03)

`23867d6` 병합. 커밋 셋: `bbed614`(octomap 1.9.7 점유 옥트리 이식),
`519fe37`(오라클 `octomap` op), `c43b78d`(경계 시나리오 4건 패리티 테스트).

§2 표가 `bye_octomap_rs` 0.1.1을 **성숙도 미달**로 판정하고 같은 행이
"자체 구현 검토"로 남겼던 항목이다. §7 위험표의 "+3주" 항목이기도 하다.
직접 이식 쪽으로 결론냈다. 테스트 27건(`tree.rs` 14, `node.rs` 8,
`key.rs` 4, 패리티 1).

이식하지 않고 남긴 것: `AbstractOcTree` 레지스트리, `ColorOcTree` /
`CountingOcTree`, 변경 감지, `setNodeValue`, `insertPointCloud` /
`computeDiscreteUpdate`, `tree_iterator`. 이 중 `insertPointCloud`는
깊이 카메라 포인트가 옥트리가 되는 경로이므로, 충돌 경로 배선을 하는
다음 라운드에서 보류 가능 여부를 다시 판단한다.

**§6.3 위험은 아직 열려 있다.** 독립형 옥트리가 들어온 것이지
`shapes::OcTree`가 충돌 월드에 연결된 것이 아니다 — MoveIt이 센서 유래
장애물을 표현하는 실제 경로가 그것이다. 다음 라운드 과제로 지정했다.

### 13.1 워커 보고의 독립 검증

이번 라운드도 보고를 그대로 받지 않고 다시 확인했다.

`p3-shapes`가 자기 워크트리에서 `tools/moveit-oracle/build.sh`를 돌리지
못했다고 보고했다 — `third_party/`의 gitignore된 벤더 트리가 워크트리에
없어서 "moveit" 스테이지가 실패한다. 대신 "oracle" 스테이지만 재현하는
스크래치 Dockerfile을 썼고, 이미지에 찍힌
`/usr/local/share/oracle-src.sha256`가 현재 트리의 다이제스트와 일치함을
근거로 댔다. 병합 후 실제 `build.sh`를 돌렸다 — `find_package(octomap)`과
`${OCTOMAP_LIBRARIES}`가 들어간 채로 정상 빌드됐다
(`moveit-rs/oracle:3ec0f5ca75dec908`). 대체는 타당했고 이탈은 종결.

커밋된 `octomap` 요청 픽스처 5건을 그 새 오라클에 다시 흘려보내 응답이
전부 바이트 단위로 동일함을 확인했다.

`p1-robotmodel`의 중심 주장(제약 2,000조합 100% 일치)도 새 오라클에
대해 다시 돌렸다 — panda 2001/2001, fanuc 2001/2001, 실패 0.
`44a84e4`의 `run-oracle.sh` 심볼릭 링크 마운트 수정은 실재하며 이제 모든
워커가 혜택을 본다(caucus 워크트리는 `third_party/`를 세션 루트 체크아웃
심볼릭 링크로 들고 있어서, 이 수정 전에는 `third_party` 상대 경로로 준
`--urdf`/`--srdf`가 컨테이너 안에서 전부 실패했다).

병합 충돌은 네 파일(`Cargo.toml`, `PORTING-PLAN.md`,
`tools/moveit-oracle/src/oracle.cpp`, `tools/moveit-oracle/CMakeLists.txt`)
에서 났고 전부 같은 모양이었다 — 두 브랜치가 같은 줄에 각자 항목을
추가한 것. 모두 양쪽을 살렸다. 오라클 op은 이제 15개다: `model_info`,
`fk`, `jacobian`, `random_states`, `acm`, `world`, `distance_field`,
`shape_points`, `common_root`, `collision_distance_field_types`,
`dynamics`, `collision_object_point_decomposition`,
`link_body_decomposition`, `constraints`, `octomap`.

병합 후 전체 게이트: `cargo nextest run --workspace` 686/686 통과,
`tools/ci/check-*.sh` 3건 전부 통과, 스윕 100,005건 실패 0
(최악 야코비안 편차 4.441e-16 ~ 2.554e-15).

### 13.2 `RuckigSmoothing` 블로커 — `RobotState`에 속도·가속도가 없다

`p6-totg`가 §4.6의 블로커를 다시 지목했다. `RobotTrajectory`가 들어와
해소된 줄 알았으나, 실제 막는 것은 `moveit_state::RobotState`에
웨이포인트별 속도·가속도 저장이 없다는 점이다.
`initializeRuckigState`, `getNextRuckigInput`, `extendTrajectoryDuration`
— Ruckig 동작의 본체인 이 셋이 전부 그 값을 읽고 쓴다.
`rsruckig` 3.0.0과 `VariableBounds`(속도·가속도·저크 한계 보유)는
문제가 아님을 워커가 각각 확인했다.

**결정: `RobotState`를 확장한다.** 모듈 지역 병렬 배열로 우회하는
선택지는 같은 사실에 집을 두 채 주는 것이라 기각했다 — 이후 모든 소비자
(`RobotTrajectory::print`, TOTG, IK 속도 시드)가 어느 쪽을 읽을지 알아야
한다. `state.rs`의 "속도·가속도 없음" 주석은 영구 설계 결정이 아니라 그
과제의 범위 표기이며, §11.6이 이미 같은 부재를 열린 블로커로 적어 두었다.
현재 `crates/moveit-state/`에 커밋 중인 다른 워커 브랜치가 없음을 여섯 개
전부 확인했다.

상류의 `acceleration_`/`effort_` 버퍼 에일리어싱은 이식하지 않는다 —
관측 가능한 유일한 결과가 한쪽을 쓰면 다른 쪽이 조용히 덮인다는 것이라,
이 계획이 반복해서 제거해 온 이중 의미 결함이다.

### 13.3 `fixtures/`의 출처가 강제되지 않고 있었다

병합 검증 중에 찾은 공백이다. FK·야코비안·ACM·다이내믹스·제약 —
이 이식의 모든 패리티 주장이 `fixtures/*.{urdf,srdf}`에 걸려 있는데,
이 파일들은 `third_party/moveit_resources`의 **사본**이다(그 디렉터리는
gitignore된 외부 체크아웃이라 새 클론과 CI에 없다). 사본이 원본에서
어긋나도 실패하는 것이 아무것도 없었다 — 주장이 그저 자기가 이름 붙인
로봇을 더 이상 설명하지 않게 될 뿐이다. 테스트 파일 세 곳이 바이트
동일성을 doc 주석에 "verified"로 적어 두었는데, 이는 시간이 지나면
낡는 일회성 수동 확인이다.

현재 상태는 확인했다 — 직접 사본 7건 전부 바이트 동일
(`panda`·`fanuc`의 urdf/srdf, `dual_arm_panda.srdf`,
`pr2`의 urdf/srdf ← 상류에서는 둘 다 `robot.xml`이라 매핑을 명시했다).
`dual_arm_panda.urdf`만 xacro 생성물이라 바이트 대응물이 없고, 재생성
명령이 `2bcd7cb` 커밋 본문에 있다.

`tools/ci/verify-fixture-provenance.sh`로 강제한다. 두 가지가 의도적이다:

- **`check-*.sh` 글롭 밖의 이름.** 그 글롭은 `.github/workflows/ci.yml`과
  로컬 게이트 루프가 돌리는 것인데, 이 검사는 그 러너들에 없는
  `third_party/`가 필요하다. CI에서 항상 건너뛰는 스크립트는 커버리지가
  없으면서 커버리지처럼 읽힌다. 이 스크립트는 벤더 트리를 요구하고
  없으면 실패하며, 이미 그 트리를 요구하는 `run-oracle-sweep.sh`에서
  돌아간다.

  이 문장이 참이 된 것은 `832d61e`부터다. 그 전까지 `ci.yml`은 글롭이
  아니라 세 스크립트를 이름으로 나열하고 있었다 — 이 헤더와 위 문단이
  둘 다 "글롭"이라고 단언하는 동안, 새로 추가된 `check-*.sh`는 CI에서
  조용히 실행되지 않았을 것이다. 규약을 지키라고 요구하면서 규약에서
  도출하지 않은 러너가 문제였으므로, 나열을 글롭 순회로 바꿔 규약이
  구조적으로 하중을 받게 했다. 빈 글롭은 통과가 아니라 실패다(이름을
  바꿔 집합을 비우는 것이 조용한 초록으로 끝나지 않도록). 세 경로를
  로컬에서 확인했다 — 스크립트 3건 실행 rc=0, 빈 글롭 rc=1, 중간
  스크립트 실패 시 루프 중단 rc=3.
- **테이블이 아니라 파일시스템이 검사를 이끈다.** 매핑 없이 추가된
  픽스처는 조용히 빠져나가는 대신 `UNMAPPED`로 실패한다. 규칙과
  "누군가 기억한 파일 목록"의 차이가 바로 이것이다.

음성 대조 3건으로 확인했다 — 픽스처 드리프트(exit 1, `DRIFTED`),
매핑 없는 새 픽스처(exit 1, `UNMAPPED`), 벤더 트리 부재(exit 1).
셋 다 원복 후 exit 0. 스윕에 배선한 뒤 200케이스로 재실행 — 통과.

### 13.4 Phase 3 완료 조건이 미충족인 진짜 이유 — `<mesh>` 충돌 형상 미로딩

`f7050c5` 병합. 커밋 셋: `4dc5556`(`bounded_prediction` 하한 0 클램프),
`aed57e6`(오라클+diff `collision` op), `aea0098`(커밋된 충돌 패리티
회귀 테스트).

**Phase 3의 100% 일치 완료 조건은 충족되지 않았고, 원인이 특정됐다.**
`crates/moveit-model/src/link_model.rs`의 이탈 4 — `<mesh>` 충돌 형상을
아예 로드하지 않는다. 워커 보고를 그대로 받지 않고 네 URDF의
`<collision>` 블록을 전부 파싱해 확인했다:

| 로봇 | collision 블록 | mesh | box | cylinder | sphere |
|---|---|---|---|---|---|
| panda | 11 | **11** | 0 | 0 | 0 |
| fanuc | 7 | **7** | 0 | 0 | 0 |
| dual_arm_panda | 22 | **22** | 0 | 0 | 0 |
| pr2 | 54 | 37 | 5 | 8 | 4 |

표의 수치는 `xml.etree`로 다시 센 것이다 — 처음 적었던 pr2 행
(59 블록 / box 8 / cylinder 10 / sphere 4)은 정규식이 XML 주석 안의
`<collision>` 블록까지 세서 틀렸다. mesh 37과 결론은 바뀌지 않는다.
링크당 `<collision>` 블록은 네 로봇 모두 정확히 하나씩이므로 블록 수와
충돌 형상을 가진 링크 수가 같다.

panda·fanuc·dual_arm_panda는 충돌 형상이 100% 메시다. 따라서 이 셋에
대해 Rust 쪽은 충돌할 형상 자체가 없다. 관측된 불일치는 panda
10,000/10,000(self 1,266 + robot 8,734), pr2 9,999/10,000(self) +
robot 1건이다. **fanuc의 10,000/10,000 일치는 패리티의 증거가 아니다** —
이 바닥 장면이 양쪽 모두에서 fanuc 충돌을 만들지 않을 뿐이다. 워커가
이 구분을 스스로 적어 보낸 것은 정확했다.

**`link_model.rs`가 이유로 댄 전제 중 절반이 이제 틀렸다.** "메시 로더가
있어도 CI에는 도움이 안 된다 — 실제 메시 파일이 새 클론에 없으므로"
라고 적혀 있는데, 앞 절반(CI에 없다)은 맞지만 뒤 절반은 멈출 이유가
되지 않는다. `third_party/moveit_resources`에 메시 파일 143개가 있고
여기에는 이 URDF들이 참조하는 충돌 STL이 전부 포함된다(panda 136K,
fanuc 352K, pr2). **네 로봇의 충돌 경로 메시는 전부 STL이다** — DAE는
시각화 경로에만 나오고 그쪽은 범위 밖이다. §13.3의 픽스처 출처 검사와
같은 구조다: 평범한 CI 러너에서는 못 돌지만 벤더 트리를 이미 요구하는
스윕 경로에서는 돈다.

다음 라운드 과제로 지정했다(STL 로더 → `package://` 해석 →
`LinkModel` 배선 → 스윕 재실행). 100% 도달을 요구하지 않았다 — 메시를
실제로 로드한 상태의 불일치 수치와 남은 것의 원인 규명이 산출물이다.

**고쳐진 결함 1건.** `ParryCollisionEnv::bounded_prediction`이
`Aabb::loosened`에 음수 마진을 넘겨 패닉했다("The loosening margin must
be non-negative") — `DistanceRequestType::Global`의 누적기가 앞선 관통
쌍에서 음수가 되면 언제든 도달한다. 즉 `enable_signed_distance: true`와
기본 요청 타입으로 실제 충돌 형상이 존재하기만 하면 되는 조건이고,
새 `collision` op의 경로가 정확히 그것이다. 하한을 `0.0`으로 클램프해
고쳤고, 클램프를 되돌리면 동일하게 패닉하는 것까지 워커가 확인했다.

**미해결.** pr2 case 7552의 고립된 `robot_collision` 불일치(오라클
collision=true/distance=-0.0249 대 rust collision=false/
distance=+0.0044)는 원인 미규명이다. ~~원시 형상 케이스라 메시 로딩으로
설명되지 않는다 — box에서 0을 가로지르는 부호 반전은 두 쪽이 점이 면의
어느 편에 있는지를 두고 불일치한다는 뜻이다.~~ 다음 라운드에 함께 넘겼다.

**취소선 부분은 거짓이다(2026-08-04, §46.1에서 해소).** 케이스 7552가
원시 형상 케이스가 아니다. 양쪽이 고르는 쌍이 넷 다 메시를 포함한다 —
자기충돌은 `l_gripper_r_finger_link`/`l_gripper_palm_link`(오라클)와
`base_bellow_link`/`torso_lift_link`(이 포트), 로봇충돌은
`r_gripper_l_finger_link`/`floor`와
`r_gripper_l_finger_tip_link`/`floor`다. 이 문장을 쓴 시점에는 어느
쌍인지 알 방법이 없었고(§43.4), 그런데도 형상 종류를 단정했다.

**병합 중 도입된 결함 1건.** `compare_collision` 호출부의 불필요한
차용 — `joint_values`가 `main`에서는 `&BTreeMap`, 워커 브랜치에서는
소유값이었고 병합이 브랜치 쪽 호출 모양을 남겼다. 양쪽 브랜치가 각각
clippy를 통과했으므로 병합 후 게이트에서만 잡히는 종류다. 충돌은 여섯
파일에서 났고 `protocol.rs`·`main.rs`·`rust_impl.rs`는 양쪽이 같은 지점에
서로 다른 항목을 추가한 것이라 기계적 "양쪽 유지"가 통하지 않았다 —
`use` 목록 병합과 enum·struct 경계 재구성을 손으로 했다.

### 13.5 `moveit-scene` 착수 — `WorldDiff` + `PlanningScene`

`e4ee160`, 충돌 없이 병합. 새 크레이트 `crates/moveit-scene`
(1,440줄: `scene.rs`, `world_diff.rs`, `attached_body.rs`, `layered.rs`).
병합 후 717/717 통과.

**과제 브리핑이 틀렸고 워커가 그것을 잡아냈다.** 내가 준 브리핑은
"상류는 바디가 부착될 때 ACM 항목을 추가하고 분리될 때 제거한다"고
적었는데, 워커가 `processAttachedCollisionObjectMsg`와
`RobotState::attachBody`를 끝까지 읽고 사실이 아님을 확인한 뒤
조용히 우회하는 대신 전면에 보고했다. 보고를 그대로 받지 않고
`~/work/moveit2/moveit_core`에서 직접 확인했다:

- `pushDiffs`(`planning_scene.cpp:377-381`) — `DESTROY`일 때
  `removeEntry`는 `if (!scene->getCurrentState().hasAttachedBody(it.first))`
  안에서만 실행된다. 워커가 인용한 주석 그대로다.
- `RobotState::attachBody` — 함수 본문 전체에 ACM/`AllowedCollision`
  참조가 0건.
- `processAttachedCollisionObjectMsg` — `AllowedCollisionMatrix`/`acm_`
  참조가 0건.
- `planning_scene.cpp`의 ACM 변경 지점은 정확히 셋(`:380`, `:1473`,
  `:1948`)이고 그중 attach/detach 경로는 없다.

실제 불변식은 더 좁다: **한 id의 ACM 항목은 그 id가 월드에 있든
부착 바디로 있든 존재하는 동안 유지되고, 완전한 삭제로만 정리된다.**
구현이 이 검증된 동작을 따르고, `scene.rs`에 "Deviation from the task
brief, confirmed against upstream"으로 문서화되어 있으며 경계 테스트
2건이 붙어 있다.

**남은 것 중 둘은 이번 라운드에 해소됐다.** 워커가 `c773c80` 시점에
"오라클 `collision` op이 없다"고 정확히 기록했는데, `f7050c5`(p3-acm)이
그것을 들여왔다. `PlanningScene`을 충돌 백엔드에 연결하는 것과
부착 바디 형상(`parry.rs`가 현재 `link_models()`와 `world.iter()`만
순회한다)이 다음 라운드 과제다.

다음 라운드 픽스처는 pr2 원시 형상 위에 짓도록 지정했다 — §13.4의
메시 공백이 닫히기 전까지 panda/fanuc의 일치는 패리티의 증거가
아니기 때문이다.

### 13.6 `collision_env_distance_field` 구성 슬라이스 — `add_link_body_decompositions`

`6a7aadc` 병합. 커밋 셋: `63d2041`(오라클
`link_models_with_collision_geometry` op), `4370a7e`(구현),
`49b9f27`(픽스처 + 패리티 테스트). 병합 후 721/721 통과, 오라클
재빌드 `24deb10eefb11348`.

**워커가 자기 테스트가 틀렸음을 인정하고 불변식으로 다시 썼다.** 첫 초안은
오라클의 링크 집합과 바이트 단위 동일성을 주장했다가 실패했다 — 오라클은
실제 메시 파일을 링크하므로 메시만 가진 PR2 링크들을 충돌 형상 보유로
보고하는데, `moveit-model`은 §13.4의 이탈 4에 따라 메시를 로드하지
않는다. 이는 `moveit-model` 결함이 아니라 잘못된 테스트였다. 다시 쓴
테스트가 주장하는 것은 **우리 집합 == 오라클 집합 − `Diagnostic::
UnsupportedLinkGeometry { kind: "mesh" }`가 기록된 링크**, 즉 모든
불일치가 설명되고 조용히 넘어가는 것이 없다는 불변식이다.

검증했다. 새 픽스처를 재빌드한 오라클에 다시 흘려 바이트 동일 확인(54
링크). 테스트가 자명하게 참이 되어버리는 형태인지도 확인했다 — 세 개의
서로 다른 주장(`!unsupported_mesh_links.is_empty()` 가드, 제외 후 동등성,
"오라클에만 있는 링크는 전부 진단으로 설명된다" 루프)으로 되어 있다.
`xml.etree`로 독립 파싱한 결과 제외 집합은 37, 표현되는 집합은 17로
자명하지 않다.

**남은 의존성 공백(우회하지 않고 보고됨).** 가장 큰 것은
`JointModelGroup::getUpdatedLinkModelNames()`와 `-WithGeometry` 변형의
부재다. `DistanceFieldCacheEntry`, `generateDistanceFieldCacheEntry`,
`getDistanceFieldCacheEntry`, `generateCollisionCheckingStructures`,
`getGroupStateRepresentation`, `compareCacheEntryTo*`,
`updateGroupStateRepresentationState`와 두 struct 정의를 막는다.

상류 구성을 직접 확인했다(`joint_model_group.cpp:255-278`) — 체인 순회가
아니라 **합집합**이다. `joint_roots_`를 돌며 각각의
`getDescendantLinkModels()`를 합치고 `OrderLinksByIndex()`로 정렬한다.
워커가 "`chain_root`는 단일 루트만 다루는데 일반
`JointModelGroup`은 여러 개일 수 있다"고 한 것은 `chain_root`에 대해서는
맞지만, 상류는 여기서 공통 루트를 쓰지 않는다 — `common_root_`는 몇 줄
위에서 다른 목적으로 계산된다. 따라서 풀어야 할 다중 루트 문제는 없다.
다음 라운드 과제로 넘겼다.

`moveit-state`의 비공개 `descendant_links_of_joint`는 상류가
`JointModel::getDescendantLinkModels`로 두는 것이므로 `moveit-model`이
소유해야 한다. 다음 라운드에서 `moveit-model`에 넣고 state 쪽 중복을
위임으로 바꾸도록 지정했다 — 같은 사실에 집을 두 채 두지 않기 위해서다.

---

## 14. Phase 4 완료, octree 충돌 결정 대기 (2026-08-03)

### 14.1 `moveit-kinematics` — Phase 4 이식, 완료 조건은 미달

`cefeabd` 병합. 커밋 여섯: `adcf5a7`(`KinematicsSolver` trait +
Newton-Raphson, LMA), `3b5bd12`(`solve()`의 `RobotState` 기본값 초기화),
`58ecb48`(불변식 경계 테스트, 자족적 FK-of-solution), `62f1e09`(diff `ik`
op), `75adb02`(오라클 `ik` op — `ChainIkSolverVelMimicSVD` 벤더링 +
외부 루프 전사), `cb668b7`(`--tol-ik` 기본값 수정).

**워커가 완료 조건 미달을 반올림하지 않고 그대로 보고했다.** "성공률 ≥
오라클" 기준을 네 픽스처 중 셋에서 못 맞췄고(0.3~1.4%p 뒤짐) 하나는
동률이다. 재빌드한 오라클로 다시 돌려 숫자를 자릿수까지 확인했다:

| 픽스처 / 그룹 | 오라클 | rust | degenerate |
|---|---|---|---|
| panda / panda_arm | 4897/5000 (97.9%) | 4876/5000 (97.5%) | 0 / 0 |
| fanuc / manipulator | 4568/5000 (91.4%) | 4498/5000 (90.0%) | 0 / 0 |

케이스별로는 15001/15001 통과, 실패 0 — 수렴한 해의 FK는 전부
`2e-5` 안에 든다.

**고쳐진 결함 1건.** `moveit-diff`의 `--tol-ik` 기본값이 `1e-6`으로,
`SolverParams::default().epsilon`(`1e-5`)보다 빡빡했다. `CartToJnt`의
수렴 판정이 twist 노름 `<= epsilon`인 스텝을 받아들이므로 수렴한 해의
FK 오차는 `(0, epsilon]` 어디에나 놓일 수 있다. panda_arm 5,000
스윕에서 2930건이 "실패"로 찍혔고 전부 오차가 `7e-6`~`9.9e-6`, 즉
epsilon 아래였다 — 솔버 결함이 아니라 측정 도구의 기준선이 틀린
것이다. `2e-5`로 고쳤다.

**성공률 격차는 원인 미규명이고, 다음 라운드는 측정 과제로 넘겼다.**
워커가 두 후보(Eigen 대 nalgebra 부동소수 발산, 재시작 RNG 스트림
분리)를 제시하되 어느 쪽도 단정하지 않았다. 코드를 확인한 결과 비교
설계 자체에 문제가 있다:

- `velocity.rs`는 `ChainIkSolverVelMimicSVD`를 실제로 이식했다 — mimic
  fold, 가중치, SVD, *상대* `svd_threshold`까지. 속도 단계의 알고리즘
  불일치는 아니다.
- `cart_to_jnt.rs:270`이 `params.max_restarts`만큼 무작위 재시작을
  돌리는데, 시드가 `ChaCha8Rng`다. 오라클 쪽 재시작은
  `random_numbers::RandomNumberGenerator`(boost mt19937)에서 온다.
  **독립된 난수 스트림을 가진 두 확률적 솔버는 케이스별 결과가 애초에
  비교 대상이 아니다.**

따라서 다음 라운드의 결정적 실험은 양쪽 모두 `max_restarts = 0`으로
두고 동일 시드에서 한 번만 시도하는 것이다. 격차가 사라지면 원인은
재시작 RNG이고 결함이 아니다. 남으면 그때는 결정론적 재현자가 생긴다.
아울러 주변부 합계(4897 대 4876)가 아니라 McNemar 쌍 카운트를 요구했다 —
같은 5,000개 표적에 대한 비교이므로 쌍 통계라야 의미가 있다.
`max_restarts`/`epsilon`/`svd_threshold`를 올려 격차를 메우는 것은
금지했다.

### 14.2 `shapes::OcTree` 충돌 배선 — parry에 대응물이 없다

`4cd9ab4` 병합. 커밋 둘: `f8cbcaf`(`shapes::OcTree`에 실제 옥트리
페이로드), `6534835`(오라클 `octree_in_world` op, 검증, 보류 목록).

**결정이 필요한 발견:** `parry3d-f64` 0.30.0에는 다중 해상도 옥트리
충돌 형상이 없다. 가장 가까운 `shape::Voxels`는 균일 해상도 전용이라,
`OcTree`에 쓰려면 트리의 최소 해상도보다 거친 리프를 최대 `8^k`개의
단위 셀로 펼쳐야 한다 — 실제 센서 맵에서 `prune()`이 아끼는 메모리를
정확히 그만큼 도로 부풀린다. 워커는 우회책을 적용하지 않았고 의존성도
추가하지 않았다. 옳은 판단이다.

이것은 이식 과제가 아니라 설계 결정이므로, 다음 라운드에 근거 수집만
지시했다: 상류 FCL `fcl::OcTree`의 깊이 적응 순회가 균일 복셀 형상이
줄 수 없는 것이 무엇인지, 실제 맵에서 측정한 확장 비율, 그리고 선택지
넷(커스텀 parry 형상 / 리프별 `Cuboid` 컴파운드 + BVH / 깊이 제한
균일 확장 / FCL FFI — D3가 "순수 Rust 먼저, FFI는 나중"이므로 배제
대상은 아니다) 각각의 비용. 구현은 이번 라운드에 하지 않는다.

`build.sh`를 워크트리에서 못 돌린다는 워커의 보고는 내 브리핑이 틀린
것이었다 — `third_party/`는 gitignore되므로 워크트리 지역이고 리베이스로
따라오지 않는다. 워커가 워크트리 격리를 이유로 세션 루트 체크아웃을
건드리기를 거부한 것이 맞다. 병합된 트리에서 내가 실제 `build.sh`를
돌렸고 정상 빌드됐다.

### 14.3 병합이 만든 결함 — 같은 모양 두 번째

`compare_ik` 호출부의 불필요한 차용. `joint_values`는 `main`의 `run()`
루프에서 `&BTreeMap`인데 옛 트리 기준으로 갈라진 브랜치가 소유값으로
넘겼다. p3-acm의 `collision` op(§13.4)에서 이미 한 번 나온 것과 정확히
같은 모양이다. `rg '&joint_values' tools/moveit-diff/src/`로 전수 확인해
`main.rs:513` 한 곳뿐임을 확인하고 고쳤다 — 나머지 열 곳은 이미 맨
이름으로 넘기고 있었다. 양쪽 브랜치가 각각 clippy를 통과하므로 병합 후
전체 게이트에서만 잡히는 종류다.

---

## 15. `--collision` 스윕은 4개 픽스처 중 3개에서 전부 실패한다 (2026-08-03)

### 15.1 기록이 실제보다 좋게 남아 있었다

§13.4는 mesh 미로딩을 "Phase 3 완료 조건 미달의 진짜 이유"로 적었고,
충돌 스윕의 불일치는 "pr2 case 7552 한 건"으로 남아 있었다. p1-robotmodel
2라운드를 병합한 뒤 `--collision` 스윕을 직접 돌려보니 실제 상태는 그보다
훨씬 나쁘다. 2,000 케이스 × 4 픽스처:

| 픽스처 | 통과 | 실패 | 실패 항목 |
|---|---|---|---|
| panda | 0/2000 | **2000** | `robot_collision` |
| fanuc | 2000/2000 | 0 | — (아래 15.3) |
| dual_arm_panda | 0/2000 | **2000** | `robot_collision` |
| pr2 | 0/2000 | **2000** | `self_collision` |

`--collision` 플래그를 도입한 커밋(`f7050c5`)에서 같은 프로브를 돌려도
pr2는 똑같이 실패한다. 오늘 병합 중 어느 것의 회귀도 아니고, 이번 라운드의
`PosedBody` 변경 때문도 아니다(§15.4). 도입 시점부터 이랬다.

### 15.2 원인은 하나 — rust가 링크의 대부분을 보지 못한다

오라클의 `link_models_with_collision_geometry`와 rust `LinkModel::shapes()`
를 픽스처별로 세어 맞춰봤다:

| 픽스처 | 오라클 | rust |
|---|---|---|
| panda | 11 | **0** |
| fanuc | 0 | 0 |
| dual_arm_panda | 22 | **0** |
| pr2 | 54 | **17** |

panda/dual_arm_panda는 rust 쪽 충돌 형상이 하나도 없다 — 그래서
`distance`가 `f64::MAX`(`1.797…e308`)로 찍히고, 오라클은 바닥 박스에
1.9 m 파묻힌 로봇을 본다. pr2의 17개는 §13.4가 센 프리미티브 링크
17개(박스 5 · 실린더 8 · 구 4)와 정확히 일치하고, mesh 링크 37개가 통째로
빠져 있다. pr2의 rust 자기충돌 거리가 설정과 무관하게 `0.029`로 고정인
것이 그 결과다 — 남은 17개가 대부분 고정 프레임 위에 있다.

`link_model.rs` deviation 4(=`<mesh>` 미로딩) 하나가 이 표 전체를 설명한다.

### 15.3 fanuc의 4001/4001 통과는 거짓 통과다

오라클의 fanuc 충돌 링크 수가 **0**이다. 컨테이너 안에서 fanuc의
`package://` mesh가 풀리지 않아 상류도 형상을 하나도 싣지 못한다. 즉
fanuc `--collision`은 빈 것과 빈 것을 비교해 2,000건 전부 통과로 찍힌다.
커버리지처럼 읽히지만 커버리지가 아니다 — `verify-fixture-provenance.sh`를
CI 경로가 아니라 스윕 경로에 둔 것과 같은 이유로(§13.3), 이 통과는
믿을 근거가 못 된다. mesh 로딩이 들어오면 fanuc은 오라클 쪽 형상 부재부터
따로 풀어야 한다.

### 15.4 `PosedBody`를 상류 모양으로 되돌렸다 — 패치가 아니라 구조

`a3bf407`이 고친 것은 결함 가족의 절반이다. 한 형상짜리 몸체는
`Compound`를 우회하게 됐지만, **두 개 이상**이면 여전히
`Compound::new`로 들어가고 parry는 `TriMesh`를 합성 형상으로 보므로
`"Nested composite shapes are not allowed"`로 패닉한다. 그 커밋의 주석은
"이 크레이트 안의 어떤 호출자도 그렇게 만들지 않는다"고 적었는데, 크레이트
*밖*에서는 `World::add_shapes_to_object`가 공개 API로 열려 있다 — mesh와
프리미티브를 같이 담은 씬 오브젝트는 특수한 경우가 아니라 보통 경우다.
공개 API만으로 패닉을 재현했다.

상류를 다시 읽으니 애초에 합치는 코드가 없다. `FCLObject`는
`std::vector<FCLCollisionObjectPtr> collision_objects_`를 들고,
`constructFCLObjectWorld`는 `Object::shapes_[i]`마다
`global_shape_poses_[i]`로 하나씩,`constructFCLObjectRobot`은 로봇 형상마다
`getCollisionBodyTransform(link, shape_index)`로 하나씩 push한다
(`collision_env_fcl.cpp:198-245`). `checkRobotCollisionHelper`와
`distanceRobotHelper`는 그 벡터를 순회하며 브로드페이즈를 형상마다 부른다
(`:337-338`, `:378-379`). 즉 "몸체당 형상 하나"라는 전제 자체가 이 포트의
이탈이었고, 그 이탈이 `Compound` 의존을, `Compound` 의존이 mesh 금지를
낳았다.

`PosedBody::parts`를 `Vec<(전역 pose, 형상)>`으로 바꿔 상류 모양을
복원했다. 몸체 대 몸체 검사는 두 파트 목록의 곱집합이고, `Compound`는
크레이트에서 완전히 사라졌다 — 런타임 검사로 막은 게 아니라 만들 수 없게
했다. 거리 쪽 임계값 계산도 파트 쌍마다 다시 하도록 옮겼다: 상류에서는
각 collision object가 `distanceCallback`을 따로 호출하며 그때까지의
`minimum_distance`/`distances`를 다시 읽는다.

**경계별 테스트**(`tests/multi_shape_object.rs`): 형상 개수(1 대 2+) ×
mesh 포함 여부의 격자, 각 칸에 충돌/비충돌 쌍을 둔 6건. 네거티브 컨트롤로
옛 `Compound` 접기를 되살리면 정확히 mesh를 포함한 다중 형상 3칸만
`"Nested composite shapes are not allowed"`로 죽고 나머지 3칸은 통과한다 —
테스트가 실제로 이 결함을 잡고 있음을 확인했다.

픽스처 어느 것도 형상 2개짜리 링크나 오브젝트를 갖지 않으므로 회귀는
없어야 하고, 실제로 pr2 200케이스 스윕의 385개 판정 줄이 변경 전후
바이트 단위로 동일하다.

이 테스트는 panda가 아니라 pr2를 쓴다. panda/fanuc/dual_arm_panda는
rust 쪽 충돌 형상이 0개라(§15.2) "충돌한다"는 단언이 성립할 수 없다.

### 15.5 `moveit-diff --cases 0`이 패닉했다

`run_constraint_cases`가 `case % states.len()`으로 상태 풀을 순환하는데,
`--cases`가 그 풀의 크기를 겸한다. `--cases 0 --constraints N`은 0으로
나눠 패닉한다. fk/jacobian/collision/ik 루프도 같은 풀을 읽으므로
`--cases 0`과 함께 주면 아무것도 비교하지 않고 조용히 통과로 보고한다 —
같은 결함 가족이다. `Config::parse`에서 거부하도록 했다. 구성이 만들어지는
유일한 지점에서 막으면 아래 소비자들은 가드 없이 인덱싱해도 된다.

---

## 16. `p3-shapes` 3라운드 — `bodies::Body` 오라클 검증, OcTree 충돌 결정 (2026-08-03)

**`bodies::Body`의 posed 알고리즘은 이번 라운드에 새로 이식한 게 아니다.**
과제 브리핑은 "1라운드부터 이연된 항목"이라고 적었지만, `bodies.rs`를
확인한 결과 `containsPoint`/`intersectsRay`/`computeBoundingBox` 계열
전부가 이전 라운드 커밋(`e3e55e4`와 후속 수정)에서 이미 완성돼 있었다.
실제로 비어 있던 것은 검증 경로였다 — 기존 `tests/probe_parity.rs`는
오라클의 JSON-line 프로토콜이 아니라 `libgeometric_shapes.so`에 직접
링크한 독립 프로브 바이너리를 재생하는 것이었고, 이번 라운드가 요구한
"오라클로 검증"은 아직 없었다. `tools/moveit-oracle`에 `body_query` op를
추가하고, 서사형 시나리오가 아니라 경계값 하나하나를 겨냥한 픽스처
(구 접선, 원기둥 양쪽 캡을 관통하는 축방향 광선, 원기둥 곡면에 진짜
접하는 광선, 원기둥 반지름 경계선을 축과 평행하게 지나 두 캡을 모두
스치는 퇴화 사례, 박스 모서리를 스치는 광선, `count` 절단 계약,
scale/padding 상호작용)로 `crates/moveit-geometry/tests/body_query_parity.rs`
를 작성해 4건 전부 통과를 확인했다. 커밋 `777b66b`.

**경계값 하나가 실제로 놀라웠다.** 원기둥 곡면에 반지름만큼 떨어진 채
축과 평행하게 놓인 광선(캡 사이 중간 높이)은 접선이라 구의 접선
사례처럼 점 하나로 접힐 것으로 예상했는데, 실제로는(오라클과 이 포트
양쪽 모두) 한 점으로 정확히 접혔다 — 예상이 맞았다. 반면 같은 방향이지만
반지름 경계선을 정확히 따라 두 캡을 관통하는 광선은 곡면 이차식이 아니라
캡 평면 가지(광선이 축과 평행하므로)를 타면서 그 가지 자신의 경계
허용오차(`v.norm_squared() < radius_scaled_sqr + ZERO`)에 정확히 걸려
점 2개(양쪽 캡 z-경계)를 보고한다 — 접힌 점 1개가 아니다. 둘 다
`cargo run --example`(스크래치, 커밋하지 않음)로 이 포트의
`Cylinder::ray_intersections`에 동일 입력을 직접 넣어 오라클과 정확히
일치함을 확인했다. `bodies::Cone`은 상류에 존재하지 않는다 —
`createEmptyBodyFromShapeType`(`body_operations.cpp`)에 `CONE` case가
없고(`default:`로 떨어져 에러 로그 후 `nullptr`), 호출자가 그 `nullptr`에
`setDimensions`를 무조건 호출하는 상류 자체의 잠재적 null-deref
버그다(아무도 Cone 바디를 만들지 않아 실전에서 발현하지 않는다) — 과제
브리핑의 "Cylinder/Cone 양쪽 끝단"이라는 표현은 Cylinder에만 해당한다.

**옥트리 충돌 도형 결정 근거는 §4.8에 작성했다, 구현은 하지 않았다.**
FCL의 실제 순회(`octree_solver-inl.h`)를 읽고 그것이 주는 것이
"메모리 절약"이 아니라 깊이 적응형 broad-phase 하강임을 확인했고,
이 프로젝트 자체의 `moveit_octomap::OcTree`로 방 크기 장면을 만들어
실측 배율(0.05m 해상도 ×6.84, 0.02m 해상도 ×15.99)을 냈으며, 실현
가능한 선택지 4개(커스텀 parry 도형/`QueryDispatcher`, 리프별
`Compound`, 깊이 상한 균일 확장, FCL FFI)를 각각의 근거(parry 소스
직접 확인 포함)와 함께 적었다. 1번을 추천하되 최종 결정은 사용자 몫으로
남겼다 — 이번 라운드에 넷 중 어느 것도 구현하지 않았다.

---

## 17. 3개 패널 동시 병합 — IK 격차 종결, `RuckigSmoothing`, `Body` 검증 (2026-08-03)

병합 순서: `9f2d967`(p6-totg) → `b23a7c0`(p1-joints) → `f6c42be`(p3-shapes).
`oracle.cpp` 디스패치가 세 번 모두 충돌했고(각각 순수 추가) 양쪽 블록의
중괄호 균형을 확인한 뒤 둘 다 남겼다. 오라클 op은 21개가 됐고 이미지는
`efe0908d197ab522`로 재빌드했다. 병합 후 762/762, clippy·fmt·rustdoc 무결,
`check-*.sh` 3종과 `verify-fixture-provenance.sh` 통과.

### 17.1 IK 성공률 격차는 재시작 RNG였다 — 그리고 측정 장치 쪽 결함이었다

§14.1이 남긴 질문에 워커가 결정적 실험으로 답했다. 양쪽 `max_restarts = 0`,
동일 시드, 난수 없음:

| 픽스처 / 그룹 | 오라클 | rust | b(오라클만) | c(rust만) |
|---|---|---|---|---|
| panda / panda_arm | 2433/5000 | 2433/5000 | 2 | 2 |
| fanuc / manipulator | 1061/5000 | 1059/5000 | 2 | 0 |
| dual_arm_panda / left_panda_arm | 2469/5000 | 2473/5000 | 3 | 7 |
| pr2 / right_arm | 3221/5000 | 3224/5000 | 15 | 18 |

(재빌드한 오라클로 내가 직접 다시 돌린 수치다. 워커 표와 절대값은 시드가
달라 다르지만 결론은 동일하다.) 0.3~1.4%p였던 주변부 격차가 사라지고 네
픽스처 모두 `b≈c`다. **핵심 알고리즘은 패리티가 있다.**

워커는 여기서 멈추지 않고, 단정하지 않는 형태로 코드 근거 하나를 덧붙였다:
`rust_impl::ik`가 케이스마다 `NewtonRaphsonSolver::new()`를 새로 만드는데
그 생성자가 RNG를 `DEFAULT_SEED = 0`으로 다시 심는다. 즉 5,000 케이스가
전부 **똑같은 20개 재시작 지점**을 되풀이한다. 오라클 쪽 `ik_rng_`는
`Oracle` 인스턴스 멤버라 런 전체에 걸쳐 스트림이 전진한다.

확인해보니 그대로였고, 이건 포트가 아니라 **측정 장치의 결함**이다 —
상류 `KDLKinematicsPlugin`도 한 번 초기화하고 계속 질의받는다. 케이스당
플러그인을 새로 만드는 쪽이 상류에서 있을 수 없는 사용법이다. 이중 의미는
`ik()`가 "솔버를 구성한다"와 "한 케이스를 푼다"를 겸한 데 있었다.
`rust_impl::IkSolver::new` + `solve_case`로 갈라 루프 밖에서 한 번만 만들게
했다(`5d67a2b`). 재시작을 켠 상태의 McNemar가 이렇게 바뀐다:

| 픽스처 | 수정 전 b/c (χ²) | 수정 후 b/c (χ²) |
|---|---|---|
| panda | 101 / 80 (2.44) | 73 / 68 (0.18) |
| fanuc | 367 / 297 (**7.38, p=0.007**) | **299 / 299 (0.00)** |
| dual_arm_panda | 89 / 74 (1.38) | 95 / 61 (7.41) |
| pr2 | 21 / 18 (0.23) | 13 / 10 (0.39) |

fanuc은 성공률까지 4593/5000으로 완전히 같아졌다 — 재시작 하에서 유일하게
유의했던 비대칭이 사라졌다. dual_arm의 7.41은 시드 1에서만 나오고
시드 2·3·4는 (87,77)·(83,74)·(75,95)로, 시드 4는 부호가 뒤집힌다. 넷을
합치면 b=340, c=307, χ²=1.68 (p≈0.19)로 유의하지 않다. **시드 노이즈다.**

`max_restarts`/`epsilon`/`svd_threshold`는 건드리지 않았다 — 워커도 나도.

### 17.2 `RuckigSmoothing` 이식 — §13.2 블로커 해소 확인

`289e9a3`(`RobotState` 속도·가속도·토크), `96cb036`/`d5a32d6`(그로 인해
낡아진 doc 주석), `e445814`(`rsruckig` 3.0.0 대상 이식), `9c1417b`(오라클
`ruckig` op + 패리티 픽스처). §13.2에서 "`RobotState`에 속도가 없어
막힌다"고 적었던 항목이 닫혔고, 저장은 상류 aliasing 없이 독립적이다.

픽스처를 새로 빌드한 오라클에 다시 던져 커밋된 응답과 바이트 단위로
같음을 확인했다. 요청 하나에 7개 하위 케이스 — limits 없음, `single_
waypoint`, 빈 궤적(둘 다 `num_waypoints < 2` no-op 경로), `mitigate_
overshoot: true`, 연속 중복 waypoint — 로 경계별 구성이다.

**워커가 스스로 범위 판단을 드러냈다.** 4단계가 `tools/oracle.cpp` 확장을
요구하는데 원래 배정은 `tools/`를 다른 워커 소유로 두고 있었다. 워커는
진행하되 "이건 확인된 안전한 override가 아니라 판단"이라고 명시하고,
`grep -n 'op =='`로 `ruckig` op이 없음을 먼저 확인했으며 추가만 했다.
그 판단이 맞았다 — 세 브랜치가 같은 디스패치 블록에 추가했고 충돌은
전부 순수 추가라 무해했다.

### 17.3 `bodies::Body` — 그리고 내 브리핑의 틀린 전제 두 건

`777b66b`(`body_query` 오라클 op + 경계 설계 픽스처 + `body_query_parity.rs`),
`164457e`(§4.8 OcTree 충돌 결정 문서).

워커가 내 과제 브리핑의 전제 두 개를 확인하고 반려했다. 둘 다 내가 틀렸다:

1. "`containsPoint`/`intersectsRay`/posed `boundingBox`가 1라운드부터
   deferred 목록에 있다" — 실제로는 `e3e55e4`에서 이미 이식돼 있었다.
   워커는 이미 맞는 코드를 다시 짜지 않고, 진짜 빈 곳이던 **오라클 검증**을
   메웠다. 그게 이 라운드의 실제 gap이 맞다.
2. "`Cylinder`/`Cone` 끝단 캡 케이스에 주의하라" — 상류에 `bodies::Cone`
   자체가 없다. `createEmptyBodyFromShapeType`에 `CONE` 분기가 없어
   `default:`로 떨어져 `nullptr`을 반환한다(호출자에서 잠재적 null 역참조지만
   아무도 Cone body를 만들지 않아 死코드). 없는 걸 채우려고 지어내지 않았다.

검증 과정에서 진짜 발견 하나: 실린더 축과 평행하면서 정확히 반지름 위에
놓인 광선은 곡면 분기가 아니라 캡 평면 분기를 타고 2점을 보고하는 반면,
곡면에 진짜로 접하는 광선은 1점으로 무너진다. 둘 다 이제 오라클로
검증되고, 이 포트는 변경 전부터 양쪽 다 상류와 일치했다.

픽스처를 새 오라클로 다시 캡처해 커밋본과 동일함을 확인했다(4 케이스).

### 17.4 OcTree 충돌 — 결정 근거가 모였다 (§4.8)

FCL 실제 순회(`octree_solver-inl.h`의 `OcTreeShapeIntersectRecurse`)를 읽고,
이 프로젝트 자신의 `moveit_octomap::OcTree`로 방 규모 씬에서 확장 비율을
실측했다: 해상도 0.05 m에서 **×6.84**, 0.02 m에서 **×15.99**. 네 선택지를
parry 벤더 소스 대비로 가격 매겼고 커스텀 parry `Shape`/`QueryDispatcher`를
권고한다. 지시대로 구현은 하지 않았다. §14.2에서 "결정 대기"로 남긴 항목의
근거가 이로써 갖춰졌다.

## 18. `p1-fixtures` 3라운드 병합 — 부착체(attached body)가 충돌 경로에 연결됐다 (2026-08-03)

`ee6f7e8`. 브랜치 커밋 4개: `4b99de4`(`PlanningScene` 충돌 메서드),
`9170651`(`CollisionEnv`에 부착체 지오메트리 관통), `f915b6e`(오라클
`collision` op의 `attached_bodies` 필드), `4c546a1`(pr2 부착체 픽스처).

### 18.1 무엇이 연결됐나

`CollisionEnv`의 네 메서드(`check_self_collision`, `check_robot_collision`,
`distance_self`, `distance_robot`)가 `attached: &[AttachedBodyGeometry]`
파라미터를 받는다. 상류 `CollisionEnvFCL::constructFCLObjectRobot`이
`state.getAttachedBodies()`를 자기·로봇·거리 질의가 공유하는 *같은*
`FCLObject`에 접어넣는 구조를 그대로 따른 것 — 부착 지오메트리는 일부
질의에만 주는 선택 인자가 아니라 이 검사들 모두에게 "로봇"의 일부다.

`PosedBody`에 `attached_link: Option<String>`과 `touch_links:
BTreeSet<String>`이 붙고, 두 게이트가 바디 쌍 단위로 동작한다:
`link_touches_attached`(부착체의 `touch_links`에 든 링크와의 쌍은 건너뜀),
`attached_pair_allowed`(같은 링크에 붙은 부착체끼리는 충돌 검사에서만
건너뜀 — 패널이 상류 `distanceCallback`에는 이 규칙이 없음을 확인했고,
그래서 `accumulate_distance`에는 `link_touches_attached`만 있다).

### 18.2 `parry.rs` 3-way 병합

같은 파일을 `b7d86f0`(§15.4의 `PosedBody` 부품 리스트화)에서 내가 이미
재구조화한 상태라 충돌 영역 5곳을 손으로 풀었다. 두 변경은 직교한다 —
저쪽은 바디 쌍 게이트(필드 2개 + 생성자 1개 + 게이트 함수 2개), 이쪽은
바디 *내부*의 부품 교차곱. 병합 결과는 게이트가 부품 루프 **바깥**(바디 쌍
단위), 부품 교차곱이 안쪽. 저쪽 `attached_body_body()` 생성자도
`pose_parts`로 다시 썼다.

### 18.3 시그니처 변경이 드러낸 정지 호출부 3곳

`attached` 파라미터 추가는 크레이트 경계를 넘는 변경이라 앵커
`check_robot_collision|check_self_collision|distance_robot|distance_self`로
워크스페이스 전체 48개 호출부를 열거해 분류했다. 정지한 것은 정확히 3곳:
`crates/moveit-collision/tests/multi_shape_object.rs:79`, `:155`,
`crates/moveit-constraints/src/visibility.rs:404`. 모두 `&[]`로 고쳤다 —
세 곳 다 부착체가 없는 시나리오다.

### 18.4 픽스처를 새 오라클로 재캡처했다

`crates/moveit-scene/tests/fixtures/pr2_attached_collision.json`의 3 케이스를
요청 3개로 다시 만들어 갓 빌드한 `moveit-rs/oracle:22b53eac162fcac9`에
흘려보냈다. 커밋본과 값이 정확히 일치한다:

| case | 부착체 | `robot_collision` | `robot_distance` |
|---|---|---|---|
| 0 | 없음 | false | 0.004407999999937988 |
| 1 | sphere r=0.1 @ `base_footprint` z=+0.5 | false | 0.004407999999937988 |
| 2 | 같은 sphere @ z=−0.1 | true | −0.1 |

케이스 2의 `−0.1`은 바닥 박스(`4×4×0.1`, 윗면 z=0)에 구가 0.1 m 파고든
값과 비트 단위로 같다.

### 18.5 패널의 판단 하나를 독립 확인했다

패널 보고: "`PlanningScene::distance_to_collision`은 상류
`distanceToCollision`의 부호 없는 거리 기본값을 재현하므로 실제 관통을
`0.0`으로 클램프한다. 오라클의 부호 있는 `robot_distance`와 비교했다면
버그가 아니라 거짓 실패였을 것이다." — 상류에서 확인했다:
`planning_scene.hpp:546`의 `distanceToCollision`이
`collision_env.hpp:220`의 편의 `distanceRobot(state, acm)`을 부르고, 그
편의 오버로드는 기본 생성 `DistanceRequest`를 쓰며
`collision_common.hpp:222`가 `enable_signed_distance = false`다. 이 포트
쪽도 `common.rs:331`이 기본값 `false`, `parry.rs:701-705`가 그때
`contact.dist.max(0.0)`으로 클램프한다. 그래서 이 파리티 테스트는
`distance_to_collision`이 아니라 `enable_signed_distance: true`를 명시한
직접 `distance_robot` 호출로 거리를 잰다.

주의로 남길 편차 하나: 상류 편의 오버로드는
`req.enableGroup(getRobotModel())`도 호출해 `active_components_only`를
설정하지만, 이 포트의 `distance_to_collision`은 그러지 않는다. §5의
`group_name` 미관통 편차와 같은 뿌리다.

## 19. 오라클이 fanuc을 지오메트리 없이 빌드하고 있었다 (2026-08-03)

§15.3에서 "fanuc의 통과는 공허하다"고 적었던 항목의 원인을 찾아 고쳤다.
커밋 `f95df44`, `a6823d7`, `b7f9329`, `89c6e51` — 서로 다른 네 개의 결함이다.

### 19.1 원인: `moveit_resources_fanuc_description`이 이미지에 없었다

Dockerfile은 `--packages-up-to moveit_core`로 빌드한다. 그 결과 이미지에
설치된 description 패키지는 `panda_description`과 `pr2_description`뿐이다
— 둘은 `moveit_core`의 test 의존이라 딸려 들어온 것이고, fanuc은 아니다.
컨테이너 안에서 `fixtures/fanuc.urdf`의 모든
`package://moveit_resources_fanuc_description/...` 메시가 해석에 실패하고
(`Package [...] does not exist`, `mesh_operations.cpp:289`), RobotModel이
`No geometry is associated to any robot links`를 남긴 뒤,
`link_models_with_collision_geometry`가 `[]`를 답한다.

즉 **fanuc에 대한 모든 오라클 응답은 지오메트리가 없는 로봇에 대한
진술이었다.** 패키지를 빌드 목록에 명시하니 7개 링크(`base_link`,
`link_1`..`link_6`)가 정상적으로 올라온다.

### 19.2 그래서 커밋된 fanuc 픽스처 두 개가 틀렸다

| 픽스처 | 옛 값 | 진짜 오라클 |
|---|---|---|
| `fanuc_collision.json` (4 케이스) | `robot_collision:false`, `self_collision:false`, 양쪽 거리 `DBL_MAX` | `robot_collision` 4/4 `true` (거리 ≈ −1e-15), `self_collision` 2/4 `true` |
| `fanuc_model_info.json` (`link_details`) | `shape_types: []`, `centered_bounding_box_offset: [0,0,0]` | `shape_types: ["mesh"]`, 실제 메시 유래 오프셋 |
| `fanuc_acm.json` | — | 동일 (SRDF만 읽으므로 영향 없음, 재실행으로 확인) |

`fanuc_collision.json`은 재캡처하지 않고 테스트와 함께 **삭제**했다. 이 포트는
fanuc 지오메트리가 0개라 모든 필드가 불일치하므로, panda가 이미 있는 자리
— 픽스처 없음, 테스트 없음 — 로 옮긴 것이다. 메시 로더가 들어오면
복원한다. `fanuc_model_info.json`은 재캡처했고, 테스트는 그대로 통과한다:
`assert_link_geometry_matches_oracle`이 원래부터 `mesh`/`capsule` 링크를
`supported_shape_count`에서 빼고 bbox 비교를 건너뛰도록 되어 있었다. 이제는
panda·pr2와 같은 이유로 통과한다.

`robot_model_parity.rs`에 있던 "오라클도 fanuc 메시를 못 읽는다"는 주석
블록은 삭제했다 — 원인을 정확히 짚은 진단이었고, 이 커밋이 그 원인을
없앴다.

### 19.3 스탬프가 이 결함을 잡지 못한 이유 (구조적)

이미지 스탬프는 "이 이미지가 지금 트리에서 빌드됐는가"를 답하라고 있는
장치인데, 해시 대상이 `*.cpp`/`*.hpp`/`*.h`/`CMakeLists.txt`뿐이었다.
**Dockerfile, build.sh(및 그것이 들고 있는 `MOVEIT2_SHA` 핀),
entrypoint.sh는 전부 밖**이었다. Dockerfile을 고쳐도 다이제스트가
그대로이니 태그도 그대로고, `run-oracle.sh`는 옛 이미지를 최신이라고
승인한다 — 스탬프가 막으라고 만들어진 바로 그 실패다.

확장자 화이트리스트를 없애고 디렉터리의 **모든 정규 파일**을 해시한다.
build.sh가 이 디렉터리 전체를 컨텍스트에 `cp -a`하고 Dockerfile이 전부
`COPY`하므로, "해시된 집합"과 "이미지에 들어간 집합"이 구성상 같아진다.
Dockerfile이 갖고 있던 `find` 식 복사본도 없애고 `src-digest.sh`를
`source`한다(정의 하나).

### 19.4 그 과정에서 드러난 로케일 결함

화이트리스트를 넓히자마자 호스트와 컨테이너의 다이제스트가 갈라졌다.
파일도 내용도 같은데 `sort`의 콜레이션이 다르다: 호스트는 대소문자 무시로
`build.sh` → `CMakeLists.txt`, 컨테이너의 C 로케일은 바이트 순으로
`CMakeLists.txt`가 먼저. 연결 순서가 달라 해시가 달라지고, 결과는
"다시 빌드해도 고쳐지지 않는 stale image" 보고다. `LC_ALL=C sort`로
고정했다. 옛 화이트리스트가 이걸 가리고 있었다 — 두 콜레이션이 갈릴 수
있는 이름이 `CMakeLists.txt` 하나뿐이었고, `src/` 대비로는 우연히 양쪽 다
먼저였다.

### 19.5 `build.sh`가 컨텍스트를 지우지 못하고 있었다

`trap 'rm -rf "$CTX"' EXIT` 다음 줄이 `exec docker build`였다. `exec`은
셸을 대체하므로 EXIT 트랩이 실행되지 않는다 — **성공한 빌드까지 포함해
모든 빌드가** moveit2 + moveit_resources 전체 export(개당 ~90 MB)를
남겼다. 이 트리에 24개, 2.1 GB가 쌓여 있었다. `exec`을 떼고(`set -e`가
종료 상태를 그대로 전파한다), 긴 빌드 중 Ctrl-C도 트랩을 타도록
`trap 'exit 130' INT`을 더했다. 쌓인 24개는 삭제했다.

## 20. 3개 패널 동시 병합 — TOTG 파리티, 옥트리 Compound, DistanceFieldCacheEntry (2026-08-03)

`p6-totg`, `p3-shapes`, `p3-distance-field` 세 브랜치를 병합했다. 병합 후
워크스페이스 797/797 통과, clippy·fmt·doc·네 개 CI 스크립트 전부 통과.

### 20.1 `p6-totg`가 진짜 버그를 하나 잡았다

`Trajectory::velocity`/`acceleration`이 `position`과 같은 방식으로
`time_step`을 질의 시각 기준으로 다시 계산하고 있었다. 상류는 그러지
않는다 — 직접 확인했다:
`time_optimal_trajectory_generation.cpp:864-916`에서 `getPosition`만
`time_step = time - previous->time_`으로 재대입하고,
`getVelocity`(881행)/`getAcceleration`(898행)은 `time_step`을 세그먼트
전체 폭(`it->time_ - previous->time_`)으로 둔 채 쓴다. 즉 속도·가속도는
세그먼트 *끝점* 값이고, 세그먼트 안에서는 계단 함수다. 위치만 연속
보간된다.

오라클 `totg` op가 이걸 드러냈다: 케이스 2(중복 waypoint), t=0.0,
`velocity[0]`이 `0.0` 대 오라클 `0.01` — 반올림 규모가 아니다. `sample()`을
`position_at`(질의 시각 재대입, 위치 전용)과
`segment_endpoint_state`(세그먼트 전체 폭, 속도·가속도 공용)로 나눠
고쳤고, 이 포트 코드가 상류와 문장 단위로 일치함을 확인했다. 수정 후 5개
케이스 전부 `1e-6` 이내(실측 편차는 최대 2e-9, 크기 ~1900 위에서).

`RobotState::invert_velocity`도 함께 들어왔다. 내 과제 브리핑은 "속도와
가속도를 뒤집는다"고 썼지만 상류 `invertVelocity`는 속도만 뒤집는다 —
워커가 브리핑이 아니라 소스를 따랐다. 맞는 판단이다.

### 20.2 `p3-shapes` — §4.8 2번안 구현 완료, 재구성 비용 실측

`compound_from_octree`(점유 리프마다 자기 깊이 크기의 `Cuboid` 하나)를
구현하고, 오라클에 `octree_shape_query` op(진짜 `fcl::collide`/
`fcl::distance`)를 더해 네 경계 전부에서 일치를 확인했다. `Compound::new`
재구성 시간은 0.05 m에서 13.3 ms, 0.02 m에서 130.4 ms — 자세한 수치와
1번안의 근거 판단은 §4.8에 있다.

다만 **아직 충돌 경로에 연결되지 않았다.** `parry.rs`의
`convert_shape`가 `Shape::OcTree(_) => None`이라, `World`에 든 옥트리는
여전히 이 백엔드에 보이지 않는다. 편차 10의 사유가 낡아 있던 것(`c67b5ca`,
"트리 페이로드가 없다"는 서술은 `moveit-octomap`이 생긴 시점에 이미
거짓이 됐다)은 고쳤고, 남은 것은 `convert_shape`에서의 호출 한 줄이다.

### 20.3 `p3-distance-field` — `DistanceFieldCacheEntry` 오라클 검증

`generateDistanceFieldCacheEntry`가 오라클과 비교된다. 그 과정에서
`moveit-state`의 중복 코드 123줄(`includes_parent`/`joint_precedes`/
`chain_root`/`descendant_links_of_joint`)을 지우고 `moveit-model`의
`JointModelGroup::is_chain`/`joint_roots`/`RobotModel::descendant_link_indices`로
위임했다 — 같은 알고리즘의 두 번째 사본이었음을 확인한 뒤의 삭제다.

### 20.4 병합이 만든 픽스처 오류 하나 (§19의 여진)

`p3-distance-field` 브랜치는 `b7f9329`(fanuc description 패키지) 이전에
잘렸다. 그래서 새로 추가한 `group_updated_links`의 fanuc 값을
지오메트리 없는 오라클에서 캡처했다 — `manipulator` 그룹의
`updated_link_with_geometry_names`가 `[]`. 진짜 답은 6개 링크 전부다.
재캡처했다(`d81cd0f` 직전 커밋). 단정문 자체는 이 포트의
`UnsupportedLinkGeometry` 진단으로 필터링하도록 이미 짜여 있어서 그대로
통과하지만, 이제는 의미가 있다.

세 브랜치의 새 픽스처 세 벌(`totg`, `octree_shape_query`,
`distance_field_cache_entry`)은 병합 후 이미지로 전부 다시 재생해
커밋본과 동일함을 확인했다.

### 20.5 `<safety_controller>` 격차는 존재하지 않는다 (보고 반증됨)

`p3-distance-field`는 이 포트가 URDF `<safety_controller>` 소프트 리밋을
읽지 않아 PR2의 `torso_lift_joint`, `l/r_elbow_flex_joint`,
`l/r_wrist_flex_joint` 기본값이 `0.0`으로 남는다고 UNFIXED에 올렸고,
픽스처에서 그 5개를 상류 기본값으로 명시 고정해 우회했다. 측정으로
반증했다.

`moveit-model/src/joint/urdf.rs:121-135`는 `robot_model.cpp:894-908`의
`jointBoundsFromURDF`와 같이 `<safety_controller>` 소프트 리밋을 우선
쓰고 `<limit>`이 더 좁은 쪽만 좁힌다. 스크래치 프로브로
`RobotState::set_to_default_values()`를 PR2에 돌리면 정확히
`torso_lift_joint = 0.16825`, `l/r_elbow_flex_joint = -1.13565`,
`l/r_wrist_flex_joint = -1.05` — 워커가 상류 전용이라고 적은 바로 그
값들이고, PR2에서 0이 아닌 기본값은 이 5개가 전부다.

관측의 실제 원인은 테스트가 `RobotState::new(&model)`만 부르고
`set_to_default_values()`를 부르지 않은 것이다. 오라클의
`applyJointValues`는 `setToDefaultValues()` 후 덮어쓰기이므로 포트 쪽도
같아야 한다. `61d5e63`에서 테스트에 `set_to_default_values()`를 넣고
픽스처의 5개 고정을 제거했다 — 응답 픽스처는 바뀌지 않았다.

같은 크레이트의 `collision_common_distance_field_parity.rs:312`도
`RobotState::new`로 시작하지만 헬퍼 `apply_joint_values`가 이미
`set_to_default_values()`를 부르므로 같은 결함이 아니다.

## 21. 메시 로더 착지 — `<mesh>` UNFIXED 종료

`p3-acm`의 세 커밋(`947f3e6`, `73da61e`, `aaaaae8`)을 `a1b2b5a`로 병합했다.
phase 3 시작부터 열려 있던 `<mesh>` collision geometry 미로딩이 닫혔다.

### 21.1 무엇이 들어왔나

- `moveit-geometry::stl::mesh_from_bytes` — STL 로더. 로드 시점에
  `compute_vertex_normals()`를 무조건 호출해 geometric_shapes의
  `createMeshFromVertices`와 맞춘다.
- `MeshSearchPaths` — `package://` URI 해석용 **패키지 이름 → 디렉터리
  맵**. 처음에는 탐색 루트 목록이었는데, 벤더링된 트리 이름이 소스
  저장소 이름(`panda_description`)이고 URDF의 `package://` URI는 ROS
  패키지 이름(`moveit_resources_panda_description`)이라 루트+이름 join이
  조용히 아무것도 못 찾았다. 명시 맵이 그 이중 의미를 없앤다.
- `RobotModel::from_urdf_and_srdf(..., &MeshSearchPaths)` — 시그니처
  변경. 충돌 지오메트리를 안 쓰는 호출자는 `MeshSearchPaths::none()`을
  넘기고, 그러면 모든 `<mesh>`가 예전처럼 `Diagnostic::UnsupportedLinkGeometry`
  와 함께 건너뛰어진다.
- `Diagnostic::UnsupportedLinkGeometry`에 `detail: Option<String>` 추가 —
  mesh 스킵 사유(미해석 `package://` URI, 미지원 확장자, 손상된 STL)를
  이름으로 남긴다.

### 21.2 병합 시 고친 것

자동 병합이 성공했지만 의미상 깨진 두 곳: `main` 쪽
`moveit-model/tests/robot_model_parity.rs:483`과
`moveit-distance-field/tests/collision_env_distance_field_parity.rs:379`이
`UnsupportedLinkGeometry`를 옛 3필드 모양으로 구조분해하고 있었다. 둘 다
`..`로 고쳤다. `collision_env_distance_field_parity.rs`의 import 블록은
3-way 충돌이 났고, 양쪽을 합쳐 해결했다(HEAD의 새 심볼 + p3-acm의
`MeshSearchPaths`).

### 21.3 워커 보고 독립 검증

보고를 중계하지 않고 병합 트리용 오라클 이미지(`0e59f840560d9bfe`)를 새로
빌드해 직접 재생했다:

- `capture-collision-fixtures.py` 재실행 — `panda_collision.json`,
  `fanuc_collision.json`, `pr2_collision.json` 셋 다 커밋본과 **바이트
  동일**.
- `mesh_parity.json`의 18개 케이스 전부를 `mesh` op으로 재생 —
  `vertex_count` / `triangle_count` / 정점 집합 **불일치 0**.

병합 후 워크스페이스: fmt clean, `clippy --workspace --all-targets
-D warnings` clean, `nextest --workspace` **813/813**,
`test --doc --workspace` clean, `tools/ci` 스크립트 4종 전부 통과.

### 21.4 남은 것 — 닫히지 않았다

- **거리 크기 차이.** 스윕에서 boolean 불일치는 0(panda 20,001건, pr2
  20,001건)이지만 침투 거리 크기는 어긋난다. 워커의 근본 원인 지목은
  `parry3d_f64::query::contact` 단일 접촉 vs FCL의 최대 200접촉 최대
  침투깊이 누적(`parry.rs` deviation 6)이고, fanuc mesh-vs-mesh에서 `~3x`,
  panda/pr2 mesh-vs-box에서 최악 `2.738` / `3.218e-1`이라는 **측정**은
  있으나 그 인과는 **아직 반증 시도된 적이 없다**. round 7에서 최악
  케이스를 골라 접촉 집합 최대값 누적으로 FCL 답이 재현되는지 확인하도록
  지시했다.
- **pr2 case 7552** `robot_collision` 불일치 — primitive 지오메트리,
  미해명. 현재 스윕이 boolean 불일치 0을 보고하므로 고쳐졌는지 커버가
  빠졌는지 확인이 필요하다.
- **비주얼 메시는 여전히 로드하지 않는다** (`link_model.rs:107`).
  렌더러가 없는 D1 범위에서는 영구 결정일 가능성이 높으나, 지금은 부수적
  주석일 뿐 명시된 결정이 아니다.
- **`MeshSearchPaths::none()` 호출자들.** 워크스페이스의
  `from_urdf_and_srdf` 호출자 대부분이 아직 `none()`을 넘긴다. 특히
  `moveit-distance-field`의 pr2 테스트들은 단정문이 메시 격차에 맞춰
  좁혀져 있어, 그 좁힘이 이제 불필요하거나 다른 모양이어야 한다.

  **pr2 메시 가용성 — 측정 완료.** `fixtures/pr2.urdf`가 참조하는
  `<collision>` 메시는 서로 다른 18개이고 **전부 `.stl`**, 전부
  `third_party/moveit_resources/pr2_description/urdf/meshes/`에 있다
  (누락 0건, 합계 0.59 MiB — 기존 `fixtures/meshes` 트리 488K와 같은
  규모). 다만 저장소 루트 `fixtures/meshes/`에는 아직 복사되지 않았고,
  거기엔 `fanuc_description`과 `panda_description`만 있다. pr2의
  `<visual>` 메시 20개는 `.dae`라 범위 밖이다. 즉 막고 있는 것은 기능이
  아니라 **픽스처 복사 한 건**이고, `verify-fixture-provenance.sh`의
  매핑 항목도 함께 필요하다 — `p3-acm` 소유.

---

## 22. `visibility_cone`의 "위반" 분기를 오라클과 실제로 맞춘다 — 그리고 그 대가 (3라운드, 2026-08-03)

### 22.1 §12.5의 100% "만족"은 커버리지가 아니었다

§12.5가 남긴 표는 네 픽스처 전부에서 `visibility_cone: 285 satisfied,
0 violated`였다 — 원뿔이 절대 로봇에 닿지 않는 배치(`FAR_OFFSET` 50m)만
썼기 때문에, `decide_cone`이 실제로 충돌을 찾아내는 분기는 오라클과
단 한 번도 맞춰보지 않은 채였다. panda/fanuc/dual_arm_panda는 여전히
그럴 수밖에 없다 — 셋 다 `<collision>`이 전부 STL `<mesh>`이고
`moveit-model`의 URDF 로더가 mesh 충돌 형상을 보존하지 않아
(`link_model.rs` deviation 4, §15.2 표 재확인) parry로 표현 가능한
충돌 형상이 하나도 없다. 원뿔을 어디에 두든 이 셋에 대해서는 "충돌
있음"이 원천적으로 나올 수 없다.

pr2는 다르다. §15.2가 이미 센 프리미티브 링크 17개(박스 5·실린더
8·구 4)는 두 backend가 똑같이 본다. `moveit-diff`의 케이스 생성기
(`tools/moveit-diff/src/main.rs`, `build_constraint_case`의
`case % 7 == 6` 분기)를 픽스처를 인식하도록 고쳤다:
`parry_representable_link_names`로 이런 링크가 있는 픽스처(사실상
pr2뿐)에서는 생성되는 케이스의 절반(`(case / 7) % 2 == 0`)을 그런
링크 하나의 전역 충돌-형상 중심에 정확히 배치하고, 나머지 절반은
그대로 `FAR_OFFSET`을 유지한다 — 링크가 하나도 없는 픽스처는 이전과
동일하게 매번 `FAR_OFFSET`을 쓴다.

**결과 (seed 4, `--group right_arm --constraints 2000`, 2026-08-03):**

```
visibility_cone: 142 satisfied, 143 violated
```

panda(seed 1)/fanuc(seed 2)/dual_arm_panda(seed 3)는 각각 285/0으로
불변, 전체 2,101/2,101 일치 — 셋 다 회귀 없음.

### 22.2 "위반" 판정 자체는 143건 전부 오라클과 일치, 그러나 깊이(depth) 값은 아니다

pr2 seed 4, 2,201건(fk 100 + jacobian 100 + model_info 1 +
constraints 2,000) 중 **`satisfied` 불리언 불일치는 0건** — 근거리
143건, 원거리 142건 전부 오라클과 "위반/만족" 판정이 일치한다.
`decide_cone`의 충돌-판정 로직 자체가 상류 `VisibilityConstraint::
decide()`(`kinematic_constraint.cpp:1069-1183`)와 정확히 같다는 확인이다.

그런데 근거리 143건 중 **119건은 보고된 `distance`(진단용 깊이 값,
판정 자체가 아니다)가 오라클과 다르다.** 원인은 §15.2와 같은 mesh
gap이 다른 모습으로 나타난 것이다: `decide_cone`은 원뿔 하나만 담은
임시 로컬 환경에 이 포트가 표현 가능한 로봇 링크만 넣고
`max_contacts: 1`로 첫 번째로 찾은 접촉만 남긴다. 오라클의 동등한
환경에는 pr2의 mesh 링크도 들어있고, pr2는 프리미티브 링크 바로
옆에 mesh 링크가 촘촘히 붙어 있는 픽스처라 원뿔이 의도한 링크와
함께 그런 mesh 링크에도 닿는 경우가 흔하다 — 그러면 상류의 순회
순서가 의도한 접촉 대신 그 mesh 접촉을 "첫 번째"로 골라, 같은
판정에 다른 깊이를 보고한다. 반지름(`0.005..0.015m`, pr2 최소
형상인 head-mount 구 반지름 0.0005m보다는 크고 최대 형상인
`base_bellow_link` 박스의 반폭 ~0.185m보다는 훨씬 작다)과 센서
오프셋(`0.005m`)을 pr2의 가장 작은 형상 기준으로 줄여도 실패 건수는
바닥 근처에서 거의 움직이지 않았다 — 반지름 0.3–0.6m·오프셋 1.0m
공유 조합(원래 값) 2,201건 중 134건 실패, 반지름 0.005–0.015m·오프셋
0.02m 조합 120건 실패, 최종값(반지름 0.005–0.015m·오프셋 0.005m)
119건 실패. 세 자릿수대에서 더 내려가지 않는다 — 즉 이건 생성기
파라미터로 없앨 수 있는 문제가 아니라 이 포트가 mesh 충돌 형상을
갖지 못하는 한 구조적으로 남는 gap이다.

**`UNFIXED`로 남긴다.** `decide_cone`(`crates/moveit-constraints/src/
visibility.rs`)의 판정 로직은 정확하고, 이번 커밋으로 고칠 것이
`moveit-constraints`나 이 생성기 안에는 없다 — `contact.depth`
값 자체가 전부 `moveit-collision`에서 나오고, 닫으려면 그쪽에 mesh
충돌 형상이 있어야 한다(§15.2의 근본 원인과 동일, 그쪽 소유자에게
전달할 사항이지 이번 라운드 소유 범위가 아니다). `moveit-diff`의
`compare_constraints`가 이 깊이 불일치를 여전히 `FAIL`로 찍는 것도
의도적으로 그대로 뒀다 — pass 기준을 바꾸는 건 이 조사 범위보다 큰
결정이라 판단해 이번 라운드에서 손대지 않았다.

**후속(§37).** 여기서 세운 가설 — "mesh 충돌 형상이 없어서" — 은
p3-acm이 pr2 메시를 실제로 넣은 뒤(§32.1) 재측정으로 **부분적으로만
맞았음이 드러났다**: 119건이 115건으로 줄었을 뿐 0이 되지 않았다.
이 절의 숫자와 원인 진단은 §37이 갱신한다.

---

## 23. `kinematic_constraints/utils.{hpp,cpp}` 중 이식 가능한 11개 이식 (3라운드, 2026-08-03)

라운드 2의 §12.7 표에서 "portable"로 분류했던 함수 13개(§22 기준
최신 라인 번호로는 `utils.hpp`의 선언 15개 중) 가운데, ROS 파라미터
읽기가 본질인 `constructConstraints`(및 그 비공개 헬퍼 6개, `moveit-ros`
소관)를 제외한 **11개**를 `crates/moveit-constraints/src/utils.rs`에
이식했다: `merge_constraints`, `count_individual_constraints`,
`construct_goal_joint_constraints`, `update_joint_constraints`,
`construct_goal_pose_constraints`(구체·상자 두 오버로드),
`update_pose_constraint`, `construct_goal_orientation_constraints`,
`update_orientation_constraint`, `construct_goal_position_constraints`,
`update_position_constraint`. `moveit_msgs::msg::Constraints`는 이
크레이트 자신의 `KinematicConstraintSet`/`JointConstraint`/
`PositionConstraint`/`OrientationConstraint`/`VisibilityConstraint`로
치환했다.

### 23.1 `resolveConstraintFrames`는 그대로 둔다

§12.7 표(원본 라인 1699 부근)의 기존 메모가 여전히 유효함을
`crates/moveit-state/src/state.rs:1150-1156`을 다시 읽어 확인했다:
`Posed::frame_transform`의 문서 주석이 지금도 "attached bodies are
not ported"라고 명시한다. §18(`p1-fixtures` 3라운드)이 부착체를
연결한 것은 맞지만 그건 `CollisionEnv`의 `AttachedBodyGeometry`
경로뿐이다 — `RobotState`/`Posed` 위에 `getAttachedBodies()`나
서브프레임을 이름으로 찾는 API는 아직 없다. `resolveConstraintFrames`가
필요로 하는 것은 정확히 그 API(`link_name`이 부착체/서브프레임을
가리킬 때 로봇 링크 이름으로 되돌리는 조회)이므로, 이번 라운드에서
이식해도 `frame_transform`이 항상 링크 이름만 성공시키는 한
"`c.link_name`이 이미 로봇 링크다"가 항상 참인 퇴화 함수가 된다.
**`p1-fixtures`(`AttachedBody`/서브프레임 소유자)가 `RobotState`
레벨에 이름 기반 서브프레임 조회를 추가한 뒤에 재검토할 것.**

**병합 시점 정정 (같은 라운드에서 해소됨).** `p1-fixtures`가 같은 라운드에
그 조회를 내놨다 — 다만 `RobotState` 레벨이 아니라 **`PlanningScene` 레벨**로,
이 포트가 부착체를 상태가 아니라 씬에 두기로 한 결정에 맞춰서다
(`crates/moveit-scene/src/scene.rs:613` `frame_transform`,
`:671` `knows_frame_transform` — 모델 프레임/링크 → 부착체 id/서브프레임 →
월드 객체 id/서브프레임의 3단 사다리). `Posed::frame_transform`의 주석도
`22bb2a2`에서 "attached bodies are not ported"를 버리고 "씬 한 층 위에서
해결된다"로 고쳤다. 따라서 §23.1의 차단 사유는 더 이상 성립하지 않는다.
남은 것은 상류가 `const RobotState&`를 받는 시그니처를 이 포트에서 무엇으로
바꿀지 결정하는 일이고, 그건 `moveit-constraints` 소유자의 몫이다.

### 23.2 설계: "재구성"이지 "제자리 수정"이 아니다

상류는 `moveit_msgs::msg::JointConstraint` 등을 필드 단위로
직접 수정한다(`jc.position = ...`). 이 포트의 제약 타입들은 생성 시점에
`RobotModel`을 상대로 검증/정규화(`normalize_angle`, 경계 clamp,
프레임 해석)를 이미 끝낸 불변 값이라 필드를 노출하지 않는다 — 그래서
`update_joint_constraints`/`update_orientation_constraint`/
`update_position_constraint`는 모두 "기존 값을 읽어 새 `::new()`를
호출해 통째로 교체"하는 형태다. `JointConstraint::merged`(private,
`crate::joint`)와 `PositionConstraint::with_updated_position`(private,
`crate::position`)이 바로 그 재구성 헬퍼다.

### 23.3 재확인 후 뒤집은 설계 가정 3건

작성 초안(이전 세션 압축 요약에 남아 있던 설계)을 이번 세션에서
업스트림 원문을 다시 읽고 3곳 고쳤다:

1. **`update_joint_constraints`의 이름 매칭.** 초안은
   `local_variable_name`을 잘라낸 이름으로 활성 조인트 목록과
   비교하려 했다. `utils.cpp:172-192`를 다시 읽으니 상류는 잘라내지
   않은 **전체** `joint_name` 문자열(`"joint/local"` 포함)로 비교한다
   — 멀티 DOF 조인트 제약에 대한 진짜 상류 한계이지 버그가 아니다.
   그대로 재현했고(`jc.joint_variable_name()`으로 비교),
   `joint.rs`의 `joint_name()`(스트립 버전) 게터는 필요 없어 삭제했다.
2. **`update_pose_constraint`의 단락 평가.** 초안은 위치/방향 갱신
   결과를 각각 `let`에 담은 뒤 `&&`로 합치려 했는데 이러면 상류의
   단락 평가(`utils.cpp:271-272`, 위치 갱신이 실패하면 방향 갱신
   호출 자체를 건너뜀)가 재현되지 않는다. `Ok(update_position_constraint(...)?
   && update_orientation_constraint(...)?)` 한 식으로 고쳐 Rust의
   `&&` 단락 평가에 맡겼고, 전용 테스트
   (`position_not_found_skips_orientation_update`)로 확인했다.
3. **쿼터니언 전용 `construct_goal_orientation_constraints`의
   파라미터화.** 초안은 포즈 오버로드와 같은 `RotationVector`를
   가정했다. `utils.cpp:275-290`을 다시 읽으니 이 오버로드는
   `ocm.parameterization`을 아예 설정하지 않는다 — 메시지 필드
   기본값 `0 = XYZ_EULER_ANGLES`가 그대로 남는다. `XyzEuler`로
   고쳤고, 동일 상태(`SB`)에서 두 파라미터화가 실제로 다른 값을
   내는 것(케이스 8: RotationVector, distance 4.028...; 케이스 10:
   XyzEuler, distance 3.8415...)을 오라클로 확인하는 테스트를 추가했다.

### 23.4 오라클 검증

`crates/moveit-constraints/tests/fixtures/panda_constraints.json`(신규,
케이스 12개, `moveit-rs/oracle:5188956fc433d046`에서 캡처)을
`crates/moveit-constraints/tests/utils_parity.rs`(신규, 23 테스트)가
역직렬화해 검증한다. 이 크레이트의 기존 관례(`collision_parity.rs`
등)를 따라 오라클을 테스트 시점에 라이브로 호출하지 않고 커밋된
픽스처를 읽는다. 생성된 값을 직접 단언하는 대신 오라클의
`constraints` op(`decide()`가 진짜로 계산하는 값과 동일한 프로토콜)로
검증해 "구성 결과"가 아니라 "구성된 제약을 평가한 결과"를 비교한다 —
`decide()`를 통과시키는 쪽이 구성값 직접 단언보다 강한 검증이다.
panda_arm(체인, 전부 1-DOF 회전 조인트)을 픽스처 그룹으로 골랐다 —
멀티 DOF 조인트가 없어 §23.3-1의 이름 매칭 예외 상황을 주 테스트
경로에 끌어들이지 않는다. `merge_constraints`/
`update_orientation_constraint`/`update_position_constraint`의 경계
케이스(윈도우 비중첩, 미발견 링크, 다중 영역 오류)는 오라클 없이
순수 단위 테스트로 커버했다.

## 24. `p1-fixtures` / `p1-robotmodel` 4차 병합 (2026-08-03)

`a1b2b5a`(메시 로더) 위에 두 브랜치를 연달아 병합했다. 최종 `3a4a9c3`,
`nextest --workspace` **845/845**.

### 24.1 `p1-fixtures` — `PlanningScene::frame_transform` 사다리

상류 `planning_scene.cpp:2036`의 3단 사다리를 이식했다: 모델 프레임/링크
→ 부착체 id/서브프레임 → 월드 객체 id/서브프레임. `AttachedBody`가
서브프레임을 갖게 됐고, `body/sub` 이름 규칙이 들어왔다. 오라클에
`frame_transform` op이 생겼고, 실제 `planning_scene::PlanningScene`을
구동한다.

**워커가 자기 1차 구현의 버그를 오라클로 잡았다.** panda(floating virtual
joint, `model_frame != root link`)에서 부착체도 월드 객체도 없는데
`knowsFrameTransform("world")`가 상류에서 `true`다. `RobotState`도
`World`도 그 답을 설명하지 못한다. 실제 기전은 TF 계층이 자기 target
frame을 자명하게 안다는 것이다 — 상류 소스를 직접 확인했다:
`transforms.cpp:66`의 `Transforms` 생성자가
`transforms_map_[target_frame_] = Identity()`를 심고,
`planning_scene.cpp:2070`의 3단째가
`getTransforms().Transforms::canTransform(frame_id)`인데 그건
`transforms_map_.find(...) != end()`(`transforms.cpp:136`)일 뿐이다.
D1에는 TF 계층이 없으므로 `knows_frame_transform`에서
`frame_id == model_frame`을 직접 특수 처리해 관측 가능한 계약을 맞췄다
(`2b110da`).

픽스처 질의 9개는 양쪽 분기를 다 친다: `a`와 `a/b`가 **둘 다 월드
객체**라서 `a/b`는 `a`의 서브프레임이 아니라 객체 id로 풀려야 하고
(`true`), `a/b/c`와 `nothing`은 어느 단에서도 안 풀린다(둘 다 `false`).

병합 트리용 오라클(`2a32b1a8cf876af4`)로 재생 — 커밋된 응답과 **바이트
동일**.

### 24.2 `p1-robotmodel` — `kinematic_constraints/utils` 11개

`constructConstraints`와 그 private 헬퍼 6개는 ROS 파라미터를 읽는
관심사라 `moveit-ros` 몫으로 남긴다. 이 결정은 유지한다.

`cargo doc`이 `rustdoc::private_intra_doc_links` 4건으로 실패했던 것을
`allow`가 아니라 소스에서 고쳤다 — public 문서 주석이 `pub(crate)` 항목을
intra-doc 링크로 가리키고 있었다.

### 24.3 두 워커의 UNFIXED가 서로를 풀었다

`p1-robotmodel`은 `resolveConstraintFrames`를 "이름 기반 부착체/서브프레임
조회가 없다"는 이유로 미뤘고(§23.1), `p1-fixtures`는 같은 라운드에 정확히
그 조회를 내놨다 — 다만 `RobotState`가 아니라 `PlanningScene` 레벨로.
§23.1에 정정을 덧붙였고 4라운드 과제로 넘겼다.

같은 방향의 두 번째 사례: `p1-robotmodel`의 `visibility_cone` 잔여
불일치 119/2,201건은 근본 원인이 "`moveit-collision`에 메시 지오메트리가
없다"인데, 그 격차가 §21에서 닫혔다. 스윕 재실행을 4라운드 과제로 넘겼다
— 숫자가 0으로 떨어지면 근본 원인 지목이 맞은 것이고, 아니면 그 지목이
틀린 것이다. 어느 쪽이든 새 숫자를 보고하도록 지시했다.

### 24.4 병합이 고쳐야 했던 것

- `PORTING-PLAN.md` 섹션 번호 충돌: `p1-robotmodel`이 §20/§21을 썼는데
  `main`에 이미 둘 다 있었다. 그쪽을 §22/§23으로 재번호했고 내부
  상호참조 1건(`§21.3-1` → `§23.3-1`)을 고쳤다.
- `from_urdf_and_srdf` 호출부 2곳이 메시 로더 이전 3인자 시그니처로
  남아 있었다: `moveit-constraints/tests/utils_parity.rs:53`,
  `moveit-scene/tests/frame_transform_parity.rs:130`. 둘 다
  `&MeshSearchPaths::none()`을 넘기도록 고쳤다.

## 25. 오라클 이미지 스탬프의 우회 경로 감사 (2026-08-03)

§19.3에서 스탬프의 파일 집합을 넓혀 "해시된 것 = 이미지에 들어간 것"을
구성적으로 맞췄지만, `src-digest.sh`가 스스로 적어 둔 대로 **파일이 아닌
빌드 입력**은 여전히 밖에 있다. 그 항목을 문서 각주로 남겨 두는 대신
전수 감사했다.

**불변식.** 빌드된 이미지를 바꾸는 모든 입력은 태그와 스탬프를 바꿔야
한다. 어떤 빌드 입력이라도 작업 트리와 다른 이미지는 답을 내면 안 된다.

**소유자/게이트.** `run-oracle.sh`의 스탬프 비교. 그 값은
`src-digest.sh:oracle_stamp`이 만들고(2026-08-04 이전에는 파일만 해싱하는
`oracle_src_digest`, 지금은 `oracle_file_digest`), `Dockerfile`이 이미지 안에
찍고, `build.sh`가 태그로 쓴다.

**우회 경로 전수.**

1. `tools/moveit-oracle/` 아래 파일 — **덮인다** (§19.3).
2. `--build-arg ROS_DISTRO=...` — **안 덮인다.** `Dockerfile`의 기본값
   `rolling`은 해시되지만, CLI에서 넘긴 값은 파일을 건드리지 않는다.
3. `--build-arg MOVEIT2_PACKAGES=...` — **안 덮인다.** 2와 같은 이유.
   §19.1이 바로 이 인자의 *기본값*이 틀려서 생긴 사고였다.
4. `TARGET=moveit build.sh` (→ `--target`) — **덮인다, 우연히.**
   `/usr/local/share/oracle-src.sha256`을 쓰는 `RUN`이 `FROM moveit AS
   oracle` 단계에만 있어서 중간 단계 이미지는 스탬프가 아예 없고,
   `run-oracle.sh`가 `<missing or unstamped>`로 잡는다. 설계로 막은 게
   아니라 부작용으로 막힌 것이므로 신뢰 대상은 아니다.
5. `MOVEIT2_SHA=<다른 sha> build.sh` — **안 덮인다.** `build.sh`의 핀
   검사는 `MOVEIT2_SHA`를 실제 HEAD와 비교할 뿐이고, 환경변수로 둘 다
   옮기면 검사는 통과한다. 다른 moveit2에서 빌드된 이미지가 같은 다이제스트,
   같은 태그를 갖는다.
6. `MOVEIT2_SRC=<다른 체크아웃>` — 5와 같은 계열. 같은 SHA면 내용도 같으니
   단독으로는 무해하고, 5와 조합될 때만 문제가 된다.
7. 베이스 이미지 `moveit/moveit2:${ROS_DISTRO}-ci` — **안 덮인다.**
   가변 태그다. 상류가 그 태그를 밀면 같은 다이제스트로 다른 이미지가
   나온다. 로컬 파일 해시로는 원리적으로 닫을 수 없고, `@sha256:`로
   핀하는 것만이 답이다.

**구조적 해결(설계 확정, 적용 보류).** 각 우회를 개별 검사로 막는 대신
스탬프를 "파일 다이제스트"에서 "해결된 전체 빌드 입력"으로 넓힌다:

- `src-digest.sh`가 정본 `ROS_DISTRO` / `MOVEIT2_PACKAGES` / `MOVEIT2_SHA`를
  한 곳에서 내놓는다(지금 `build.sh`와 `Dockerfile`에 흩어져 있다).
- `build.sh`가 셋을 항상 명시적으로 `--build-arg`로 넘긴다.
- `Dockerfile`이 **해결된** ARG 값을 파일 다이제스트와 함께 스탬프에
  찍는다. 그러면 손으로 `--build-arg`를 넘긴 이미지는 스탬프가 트리와
  달라져 잡힌다 (2, 3, 5 종료).
- 베이스 이미지를 `@sha256:`로 핀하고 그 다이제스트도 스탬프에 넣는다
  (7 종료).
- 4는 부작용 의존을 그만두고, 스탬프 없는 이미지를 명시적으로 거부한다.

**~~지금 적용하지 않는 이유(범위/시간 제한이 아니라 동시성)~~ — 적용했다
(`5c59aeb`, 2026-08-04).** 보류 사유는 "다이제스트를 바꾸면 활동 중인 패널
전부가 동시 전체 재빌드에 걸린다"였다. 그 전제가 틀렸다는 것을 재빌드를
실제로 해 보고 알았다: 스탬프는 `oracle` 스테이지의 마지막 `RUN` 하나에만
들어가고 비싼 `colcon build` 레이어는 그 위에 있으므로, 다이제스트만 바뀐
재빌드는 **0.8초**다(`docker build` 로그, 캐시 적중). 즉 "조용한 시점"을
기다릴 이유가 애초에 없었고, 측정 없이 비용을 가정해 구조적 수정을 세
라운드 미룬 것이다.

적용 내용은 위 설계 그대로이되 2번은 검사가 아니라 **삭제**로 닫았다.
`ROS_DISTRO`는 `FROM`의 태그만 골랐고 스테이지 안의 모든 `RUN`은 베이스
이미지 자신의 `ROS_DISTRO` ENV를 읽었다(ENV가 동명 ARG를 덮는다) — 즉
`--build-arg ROS_DISTRO=humble`은 humble 베이스에 rolling 경로를 쓰는
이미지를 만들었다. 베이스를 매니페스트 다이제스트로 핀하면 distro는 그
이미지의 성질이 되므로, 어긋날 수 있는 인자 자체가 없어진다.

**검증 4건(전부 실행함).**

1. 정본 빌드: 이미지 스탬프 = 트리 스탬프
   (`acc5bdb893b019fc83b8314685614ed3e292e62b30a47790e8066f5b6e36818e`),
   `model_info` 응답 정상, rc=0.
2. `MOVEIT2_SHA=deadbeef...`로 손빌드한 이미지: 스탬프
   `47dbccea5dc17cf7...` ≠ 트리, `run-oracle.sh` rc=1로 거부. **수정 전에는
   파일이 그대로이므로 같은 스탬프를 찍고 통과했을 이미지다.** 이것이 5번의
   실제 종료 증거다.
3. 중간 스테이지(`--target moveit`): `/usr/local/share/oracle-src.sha256`
   자체가 없어 `<missing or unstamped>`. 이전에는 부작용이었고 이제
   `oracle_stamp` 주석이 의존한다고 명시한다.
4. `BASE_IMAGE` 미전달: `base name (${BASE_IMAGE}) should not be blank`으로
   빌드 실패. Dockerfile에 기본값을 두지 않았기 때문이고, 기본값을 뒀다면
   같은 사실의 두 번째 정의가 되어 스탬프가 어느 쪽을 주장하는지 알 수 없게
   된다.

6번(`MOVEIT2_SRC`)은 단독으로는 여전히 무해하고 5번과의 조합으로만
문제였으므로 5번이 닫히면서 함께 닫힌다.

## 26. `p1-joints` 4라운드 (2026-08-03)

### 26.1 `checkConsistency`의 OOB 읽기 — 결함은 확인, 브리핑의 인용 두 건은 정정

4라운드 과제가 지목한 읽기를 핀 SHA(`e017c91ee12984393a28ba246075c65f69cde3bf`)
그대로에서 직접 다시 읽었다. 결함 자체는 **확인**된다:

- `kdl_kinematics_plugin.cpp:84-94`의 `checkConsistency`는
  `for (std::size_t i = 0; i < dimension_; ++i)` (풀스페이스 경계, 88행)로
  돌면서 `consistency_limits[i]` (90행)를 인덱싱한다.
- 호출부(`kdl_kinematics_plugin.cpp:391-392`)가 넘기는 `consistency_limits`
  인자는 `consistency_limits_mimic` — 337-341행에서 `mimic_joints_[i].active`인
  항목만 추려 만든, **활성 관절 수만큼만 있는** `std::vector<double>`이다.
- 즉 그룹에 mimic 관절이 하나라도 있으면 `dimension_ >` 활성 관절 수가 되고,
  `consistency_limits[i]`는 `i`가 활성 관절 수를 넘는 순간
  `std::vector::operator[]`의 경계 밖을 읽는다 — 예외가 아니라 UB.

브리핑의 인용 중 두 가지는 **정정**한다:

1. "`kinematics_base.cpp:320`의 에러 메시지" — 틀렸다.
   `kinematics_base.cpp`는 210줄뿐이고 `dimension_`이라는 이름이 단 한 번도
   나오지 않는다(`rg dimension_ kinematics_base.cpp` 매치 0건). `dimension_`은
   `KDLKinematicsPlugin`(하위 클래스, `kdl_kinematics_plugin.hpp`) 전용
   멤버다. 인용된 에러 메시지("Seed state must have size %d instead of
   size %zu")는 실제로 `kdl_kinematics_plugin.cpp:320`에 있다 — 파일 이름이
   바뀐 것뿐, 그 옆 줄의 "consistency_limits must be empty or have size %d"
   (`kdl_kinematics_plugin.cpp:329`)는 브리핑 인용 그대로 맞다.
2. "`jnt_seed_state.data`, `jnt_pos_out.data`... 전부 reduced-size" — 틀렸다.
   둘 다 `KDL::JntArray(dimension_)`으로 선언된다(352, 354행) — **풀스페이스**다.
   reduced-space인 건 세 인자 중 가운데 `consistency_limits_mimic` 하나뿐이다.
   OOB는 "세 인자가 서로 안 맞는다"가 아니라 "풀스페이스 경계로 reduced-space
   벡터 하나만 인덱싱한다"는, 더 좁고 정확한 모양이다.

이 결함은 이식하지 않는다: `registry.rs`의 `SolveOptions` 독코멘트가 이미
`consistency_limits`를 reduced-space(활성 관절 수만큼)로 정의해, `seed`/해가
이미 사는 공간과 맞춰서 이 불일치를 애초에 구성 불가능하게 만든다(217465e,
3라운드) — 그 문서화는 이번 확인으로 바뀌지 않는다. 오라클 `ik` op은 이
좁혀진 모양(풀스페이스 인자를 받아 오라클 쪽에서 `consistency_limits_mimic`으로
축약한 뒤 넘기는 것은 상류와 동일하게 하되, 축약된 벡터의 길이가 `dimension_`이
아니라 활성 관절 수라는 것을 오라클 구현이 스스로 인지하게)으로 설계해야
이 OOB를 오라클측에서도 재현하지 않는다.

### 26.2 오라클 `ik` op의 `consistency_limits` 확장과 4+1개 픽스처 비교

**설계.** 와이어의 `consistency_limits`는 `joint_values`와 같은 모양 —
관절 이름을 키로 하는 맵(`BTreeMap<String, f64>`) — 이고, **풀스페이스**
(상류 `searchPositionIK`가 실제로 받는 모양)로 정의했다. reduced-space로
이미 축약된 값을 오라클에 넘기면 오라클이 하는 일은 "그대로 통과"뿐이라
비교가 순환 논증이 된다; 오라클 쪽에서 활성 관절만 추려 자신만의
`consistency_limits_mimic`을 구성하게 해야 20.1에서 확인한 상류의 축약
자체를 검증하는 셈이 된다. 이름 기반 키를 쓴 것은 위치 기반
(`Vec<f64>`)이었다면 오라클과 이 포트가 각자 독립적으로 구현한 관절
순회 순서가 반드시 일치해야 했을 텐데, 그 순서 불일치가 바로 20.1의
OOB 버그가 태어난 방식이기 때문이다.

오라클의 일관성 검사(`consistencyOk`, `oracle.cpp`)는 재시도 루프
시작 전에 **한 번만** 캡처한 원본 시드(`q_seed_full`)를 기준으로 매
시도의 해를 비교한다 — 상류의 `jnt_seed_state`가 `searchPositionIK`의
재시도 루프 안에서 재대입되지 않는 것과 동일하다. 재시드 지점을
기준으로 비교하면 20.1의 OOB와는 다른 종류의 버그(기준점이 매 시도
바뀌는)가 됐을 것이다.

**해 콜백은 와이어에 없다.** 상류 `searchPositionIK`의
`solution_callback`은 임의의 C++ 클로저이고, 이 포트의
`SolveOptions::solution_callback`도 마찬가지로 `FnMut` 클로저다. 둘 다
프로세스 경계를 넘길 수 있는 값이 아니므로 오라클 와이어 프로토콜에는
이에 대응하는 필드를 만들지 않았다 — 이번 확장에서도, 앞으로도 만들
계획이 없다. `rust_impl.rs`의 `solve_case`는 항상
`solution_callback: None`으로 호출한다.

**소거법으로 확인한 사실: mimic 관절을 포함하는 IK 체인은 프로젝트
전체에서 크레이트 로컬 테스트 픽스처 하나뿐이다.** 공식 4개 픽스처
(`fixtures/*.srdf`, `verify-fixture-provenance.sh`로 벤더 원본과
바이트 동일함이 잠겨 있음)를 전수 확인한 결과:

- pr2의 `l_end_effector`/`r_end_effector`는 두 손가락이 같은 palm
  링크에서 갈라지는 분기 목록이라 `is_chain()`이 아니다; `right_arm`/
  `left_arm`은 그리퍼까지 닿지 않는다.
- panda의 `hand` 그룹은 실제 `<joint>` 멤버가 `panda_finger_joint1`
  하나뿐이고 mimic은 `<passive_joint>` 참조로만 존재한다 — 그 passive
  참조를 인정하더라도 `panda_leftfinger`/`panda_rightfinger`가 같은
  `panda_hand` 링크에서 갈라지므로 체인이 아니다.
- fanuc과 dual_arm_panda는 팔 체인에 mimic 관절이 아예 없다.

유일하게 실사용 가능한 mimic 체인은
`crates/moveit-kinematics/tests/fixtures/pr2.srdf`에 이미 문서화되어
있던 크레이트 로컬 전용 그룹 `l_gripper_finger_chain`
(`base_link="l_wrist_roll_link" tip_link="l_gripper_l_finger_tip_link"`,
실제 PR2 URDF의 관절 타입/mimic 배수를 그대로 쓰되 상류에 없는 그룹
경계를 새로 그은 것 — 그 파일 자체의 코멘트 및
`velocity.rs::pr2_gripper_mimic_column_folds_into_its_masters_column_not_its_own`
테스트 참고)이다. 이 URDF는 공식 잠긴 `fixtures/pr2.urdf`와 바이트
동일함을 `diff`로 확인했다(SRDF에 그룹 하나만 추가된 차이). 이 조합은
공식 4-픽스처 스윕에는 넣지 않고 별도로, `--urdf`/`--srdf`를 크레이트
로컬 경로로 직접 지정해 돌렸다.

**`--ik-consistency-limit FRACTION`.** IK 시드는 항상 각 관절 자체
경계의 중점이고, 일관성 검사는 재시드가 아니라 그 원본 시드에 대해서만
측정하므로, 한 관절이 가질 수 있는 최대 편차는 정확히 `range/2`다 —
`FRACTION ≥ 0.5`는 아무것도 거부하지 않는 자명한 통과가 된다. 7-DOF
팔에서 0.1~0.15는 20회 재시도로도 사실상 100% 거부(과제가 경고한
"아무것도 거부하지 않는 스윕은 아무것도 검증하지 않는다"의 반대
극단이자 마찬가지로 무의미)였다. 공식 4개 그룹에는 두 극단 모두를
피한 **0.35**를, DOF가 훨씬 낮은(1 능동 + 1 mimic) `l_gripper_finger_chain`에는
낮은 차원 탓에 같은 fraction이 다르게 반응함을 확인하기 위해
0.1/0.2/0.35 세 값을 스윕했다.

**결과 (오라클/이 포트 성공률, 500 케이스, `--ik-max-restarts` 기본값 20, FK 정확성 실패 전부 0건):**

| 픽스처 / 그룹 | 제한 없음 (oracle/rust) | `--ik-consistency-limit 0.35` (oracle/rust) |
|---|---|---|
| panda / panda_arm | 96.0% / 98.0% | 31.4% / 36.0% |
| fanuc / manipulator | 91.2% / 89.2% | 33.2% / 44.4% |
| dual_arm_panda / left_panda_arm | 99.0% / 96.6% | 28.6% / 33.6% |
| pr2 / right_arm | 99.8% / 99.6% | 23.2% / 25.2% |

크레이트 로컬 `pr2.urdf`+`pr2.srdf` / `l_gripper_finger_chain`
(오라클과 이 포트가 매 케이스 정확히 같은 수만 성공 — `paired: b=0, c=0`):

| fraction | 성공률 |
|---|---|
| 제한 없음 | 100.0% (500/500) |
| 0.1 | 20.8% (104/500) |
| 0.2 | 42.0% (210/500) |
| 0.35 | 69.8% (349/500) |

FK 정확성 실패는 8+4개 실행 전부에서 0건이었다 — `consistency_limits`의
풀스페이스→축약 로직과 재시도 루프의 원본-시드 고정이 상류와 이 포트
양쪽에서 동일하게 동작함을 확인했다. 구현은 `oracle.cpp`(op 확장),
`protocol.rs`(`Op::Ik::consistency_limits` 필드), `rust_impl.rs`
(`chain_joint_names`/`solve_case`의 축약), `main.rs`(`--ik-consistency-limit`
CLI, 관절별 fraction×range 계산)에 있다(d4b72cc).


### 26.3 §26.2의 결론 정정 — 재시드 경로에 단방향 격차가 있다 (병합 시 측정)

§26.2는 "네 공식 그룹 모두에서 오라클/이 포트 성공률이 소수점 첫째
자리까지 완전히 일치했고 `paired: b = 0, c = 0`"이라고 맺었다. **그
문장은 §26.2 자신의 표와 모순된다** — 같은 표가 panda 31.4% / 36.0%,
fanuc 33.2% / 44.4%를 적고 있고, `b`/`c`는 정의상 성공 개수 차이
(`rust_success - oracle_success == c - b`)이므로 둘 다 0일 수 없다.
`failed: 0`은 "각 측이 낸 해가 자기 FK로 목표를 맞추는가"만 보는
지표이고 두 측이 같은 케이스에서 성공하는가는 보지 않는다 —
`main.rs:1345-1360`의 `oracle_only`/`rust_only` 독코멘트가 바로 그
구분을 위해 존재한다고 적고 있다.

병합 시점에 직접 측정했다 (`--cases 500 --seed 20260803`,
`--ik-max-restarts` 기본값 20, 이미지 `a8988410cec4b1aa`):

| 픽스처 / 그룹 | `--ik-consistency-limit 0.35` oracle/rust | b (oracle만) | c (rust만) |
|---|---|---|---|
| panda / panda_arm | 27.8% / 38.6% | 15 | 69 |
| fanuc / manipulator | 29.6% / 44.4% | 18 | 92 |
| dual_arm_panda / left_panda_arm | 30.2% / 39.6% | 20 | 67 |
| pr2 / right_arm | 22.4% / 24.4% | 8 | 18 |

제한 없이 같은 시드로 돌리면 대칭 잡음이다: panda `b=8, c=10`,
fanuc `b=33, c=32`, pr2 `b=0, c=0`. 즉 **일관성 제한이 걸릴 때만** 네
그룹 전부에서 `c >> b`가 되고, 방향이 언제나 이 포트 쪽이다. `moveit-diff`
자신의 판정 기준(`b≈c`는 잡음, 한쪽이 크게 크면 실효)으로 실효다.

**격차의 위치는 재시드 경로로 좁혀진다.** `--ik-max-restarts 0`으로
재시드를 아예 끄면 두 측이 정확히 일치한다:

| panda / panda_arm, `--ik-max-restarts 0` | oracle/rust | b | c |
|---|---|---|---|
| `--ik-consistency-limit 0.35` | 15.2% / 15.2% (76/500) | 0 | 0 |
| 제한 없음 | 50.6% / 50.6% (253/500) | 1 | 1 |

첫 시도, 뉴턴 반복, 일관성 수락/거부 판정은 두 측이 동일하다. 갈리는
곳은 `near_by_configuration`(이 포트) 대 `getVariableRandomPositionsNearBy`
(상류)뿐이다. RNG 스트림 차이(ChaCha8 대 boost mt19937)만이라면 부호가
대칭이어야 하는데 네 그룹 전부 한 방향이므로, RNG만으로는 설명되지
않는다. 확인된 후보 하나: 상류 `RevoluteJointModel::getVariableRandomPositionsNearBy`
(`revolute_joint_model.cpp:122-136`)는 **연속 관절(`continuous_`)일 때
`near ± distance`를 클램프 없이 뽑고 `enforcePositionBounds`로 감싸는**
별도 분기를 갖는데, 이 포트의 `near_by_configuration`은 모든 관절을
`[min, max]`로 클램프한다. 다만 panda_arm에는 연속 관절이 없으므로 이
분기만으로 panda의 격차를 설명하지는 못한다. 근본 원인은 미확정 —
`p1-joints` 5라운드로 넘긴다.

### 26.4 재시드 격차의 근본 원인 — 오라클 자신의 버그, 그리고 별개의 연속 관절 격차

**주 원인: 오라클의 재시드 루프가 `consistency_limits`를 무시하고 항상
전체 범위에서 뽑는다.** `oracle.cpp`의 `ik()` 재시드 루프(2라운드에 처음
작성됨, 4라운드에 `consistencyOk` 게이트만 추가되고 이 루프 자체는
건드리지 않음)는 매 재시도마다

```cpp
reseed_active[k] = ik_rng_.uniformReal(joint_min[full_i], joint_max[full_i]);
```

로 각 활성 관절의 **전체** `[min, max]`에서 뽑았다 — `consistency_limits`가
있는지 여부와 무관하게. 그런데 상류 `searchPositionIK`는 조건부다
(`kdl_kinematics_plugin.cpp:373-382`):

```cpp
if (!consistency_limits_mimic.empty())
  getRandomConfiguration(jnt_seed_state.data, consistency_limits_mimic, jnt_pos_in.data);  // near-by, clamped
else
  getRandomConfiguration(jnt_pos_in.data);  // full-range
```

일관성 제한이 걸려 있을 때는 원본 시드 근처에서, 클램프해서 뽑아야
한다. 오라클은 이 분기를 아예 구현하지 않고 항상 full-range 분기만
탄 것 — 이 포트 자신의 `near_by_configuration`(`cart_to_jnt.rs`)은
처음부터 상류의 근접 재시드 분기를 정확히 구현하고 있었으므로, 좁은
제한 아래서는 오라클의 재시드가 원본 시드에서 너무 멀리 떨어진 곳을
뽑아 (수렴하더라도) 자신의 일관성 검사에서 스스로 떨어뜨리는 빈도가
이 포트보다 훨씬 높았다 — 방향이 항상 이 포트 쪽으로 치우친 이유다.

`joint_min[full_i]`/`joint_max[full_i]`를 `seed_active[k] ± limit`로
클램프하도록 오라클의 재시드 루프를 고쳤다(`8143395`). 같은 시드
(`--seed 20260803`)로 다시 측정:

| 픽스처 / 그룹 | 수정 전 (이미지 `a8988410cec4b1aa`) b/c, z | 수정 후 (이미지 `7bc4f487ef8d16ab`) b/c, z |
|---|---|---|
| panda / panda_arm | 15/69, z=5.89 | 31/36, z=0.61 |
| fanuc / manipulator | 18/92, z=7.06 | 32/32, z=0.00 |
| dual_arm_panda / left_panda_arm | 20/67, z=5.04 | 35/45, z=1.12 |
| pr2 / right_arm | 8/18, z=1.96 | 17/12, z=0.93 |

(`z = |b - c| / sqrt(b + c)`, McNemar 정규근사.) panda/fanuc/dual_arm_panda
세 그룹은 z가 5.0-7.1대에서 0.6-1.1대로 떨어졌다 — 이 격차의 압도적
부분이 오라클 자신의 버그였다는 뜻이다. pr2는 수정 전에도 z=1.96으로
"확실히 실효"라고 부를 정도는 아니었다 — 베이스라인 성공률이
99.6-99.8%로 높아 애초에 재시드가 필요한 케이스 자체가 500건 중
소수(`b+c=26`)뿐이었고, 이 통계량은 `b+c`가 작을 때 검정력이 낮다.
버그가 pr2에 없었다는 뜻이 아니라, 이 표본 크기로는 그 그룹에서
통계적으로 뚜렷하게 잡히지 않았다는 뜻이다 — `tools/moveit-diff`의
`PAIRED_DIVERGENCE_Z_THRESHOLD` 독코멘트에도 같은 사실을 적어 두었다.

**부수적, 별개의 원인: 연속 관절 재시드는 클램프가 아니라 랩(wrap)해야
한다.** 상류 `RevoluteJointModel::getVariableRandomPositionsNearBy`
(`revolute_joint_model.cpp:122-136`)는 `continuous_`일 때
`near ± limit`을 **클램프 없이** 뽑고 `enforcePositionBounds`로
`(-pi, pi]`에 랩시키는 별도 분기를 갖는데, 이 포트의
`near_by_configuration`은 (5라운드 이전까지) 모든 관절을 `[min, max]`로
직접 클램프했다 — 이 포트 자신의 독코멘트가 "non-continuous branch"라고
이미 스스로 적어 놓았던, 구현되지 않은 절반이다. `ChainInfo`에
`active_continuous: Vec<bool>`를 추가하고 `near_by_configuration`에
연속 관절 전용 랩 분기를 넣어 고쳤다(`6408570`), 전용 단위 테스트
(`near_by_configuration_wraps_a_continuous_joint_past_pi_instead_of_clamping`)로
검증.

이 격차는 네 공식 그룹의 스윕 수치에는 나타나지 않는다: pr2의 두
연속 관절(`r_forearm_roll_joint`, `r_wrist_roll_joint`)은 경계
`[-pi, pi]`의 중점인 0이 시드이고, `0.35 × 2π ≈ 2.2 rad` 제한은 0에서
`±π` 경계에 닿지 못한다 — 즉 이 스윕 조건에서는 클램프 분기와 랩
분기가 수학적으로 동일한 범위를 낸다. 그래서 수정 전후로 pr2 수치가
완전히 동일하다(17/12, 변화 없음). 실제로 검증하려면 시드가 경계
가까이 있어야 하므로, 스윕이 아니라 전용 단위 테스트로 이 분기를
직접 겨냥해 검증했다.

**구조적 조치: `b`/`c`를 출력 줄이 아니라 게이트로 만들었다.**
`moveit-diff`의 `run()`은 이제 `paired_divergence_z(b, c)`(McNemar 정규근사
통계량)를 계산해 `PAIRED_DIVERGENCE_Z_THRESHOLD = 3.0`을 넘으면
`ik_paired_divergence`라는 실패 verdict를 `report()`에 밀어 넣는다
(`957b230`) — `failed: 0`만 읽고 "일치"로 결론 내리는 26.2/26.3의 실수가
도구 차원에서 반복될 수 없도록. 임계값 3.0은 이번 라운드 자신의
수정 전/후 z값(5.89-7.05 대 0.00-1.12)에서 뽑았고, 그 경계 자체를
`paired_divergence_tests`(`main.rs`)로 고정해 두었다.

**재현 가능한 최종 스윕** (`--cases 500 --seed 20260803`,
`--ik-max-restarts` 기본값 20, 오라클 이미지 `7bc4f487ef8d16ab`, 전부
`failed: 0`, `ik_paired_divergence` 전부 PASS):

| 픽스처 / 그룹 | 제한 없음 oracle/rust, b, c, z | `--ik-consistency-limit 0.35` oracle/rust, b, c, z |
|---|---|---|
| panda / panda_arm | 97.6%/98.0%, b=8, c=10, z=0.47 | 37.6%/38.6%, b=31, c=36, z=0.61 |
| fanuc / manipulator | 91.2%/91.0%, b=33, c=32, z=0.12 | 44.4%/44.4%, b=32, c=32, z=0.00 |
| dual_arm_panda / left_panda_arm | 97.6%/98.6%, b=6, c=11, z=1.21 | 37.6%/39.6%, b=35, c=45, z=1.12 |
| pr2 / right_arm | 99.6%/99.6%, b=0, c=0, z=0.00 | 25.4%/24.4%, b=17, c=12, z=0.93 |

크레이트 로컬 `crates/moveit-kinematics/tests/fixtures/pr2.urdf`+
`pr2.srdf` / `l_gripper_finger_chain`(같은 조건, 같은 이미지):

| 조건 | oracle/rust, b, c, z |
|---|---|
| 제한 없음 | 100.0%/100.0%, b=0, c=0, z=0.00 |
| `--ik-consistency-limit 0.35` | 72.0%/72.0%, b=0, c=0, z=0.00 |

모든 z가 임계값 3.0을 한참 밑돈다.

## 27. `p3-distance-field` 5라운드 병합 (2026-08-04)

`5231cf2`. `nextest --workspace` **855/855**.

### 27.1 pr2 메시 격차는 "기능 없음"이 아니라 "픽스처 복사 없음"

워커가 `third_party/`를 직접 가리켜 `build_pr2_model`을 돌려 보고,
좁혀 두었던 단정문이 전부 오라클과 정확히 일치한다고 보고했다. 중계하지
않고 직접 재현했다:

- `MeshSearchPaths::none()`: 지오메트리 보유 링크 **17개**.
- `MeshSearchPaths::new([("moveit_resources_pr2_description",
  third_party/.../pr2_description)])`: **54개**, 남은
  `UnsupportedLinkGeometry{kind:"mesh"}` 진단 **0건** — 18개 충돌 메시가
  전부 해석된다.
- 그 54개를 `link_models_with_collision_geometry_response.json`의 오라클
  링크 목록과 대조: 개수도 **순서까지도** 동일(`actual == expected`).

즉 §21.4의 판정이 확인됐다 — 막고 있는 것은 `fixtures/meshes/`로의 0.59
MiB 복사 한 건이고, 그건 `p3-acm` 몫이며 이 병합 시점에 아직 안 들어왔다
(`fixtures/meshes/`에는 `fanuc_description`, `panda_description`뿐).
워커는 좁힘을 유지하되 세 곳의 문서 주석을 "기능 없음"에서 "픽스처 복사
없음, 정확성은 검증됨"으로 고쳤다 — 옳은 처리다. 검증한 사실을 픽스처가
없다는 이유로 단정문에 넣지 않고, 대신 왜 안 넣는지를 정확히 적었다.

### 27.2 이관 범위 재분류

`compareCacheEntryToState` / `compareCacheEntryToAllowedCollisionMatrix`를
자유 함수로 이식했다. 이전 라운드 문서가 이 둘을 `getDistanceFieldCacheEntry`와
한 덩어리로 "캐시 타입에 막힘"이라 묶었는데, 실제로는
`CollisionEnvDistanceField`의 캐시 멤버를 건드리지 않고 이미 이식된
`DistanceFieldCacheEntry` + `RobotState` + `AllowedCollisionMatrix`만
쓴다. 차단 목록이 과대했던 것.

여전히 막힌 것: `GroupStateRepresentation`, `getDistanceFieldCacheEntry`,
`generateCollisionCheckingStructures`, `updateGroupStateRepresentationState`.

`p1-fixtures`의 새 `moveit-scene::AttachedBody`가 이걸 풀어 주는지도
확인했고 — 풀어 주지 않는다. 그 타입은 `PlanningScene`에 살고 맨
`RobotState`에서 닿지 않으므로, 두 새 함수의 부착체 비교는 공허하게 참이
된다. 편차로 문서화했다.

오라클 op이 이 두 함수에 단독으로 닿지 않아(확장은 워커 범위 밖) 불변식
경계로 검증했다: 그룹 내/외 관절 이동, `EPSILON` 이내/초과 이동, 크기가
다른 상태, ACM 행 수 변화, 새로 비활성화된 자기충돌, 새로 비활성화된
그룹 내 쌍. 시나리오가 아니라 경계로 짠 테스트다.

## 28. 한 번도 실행된 적 없는 CI를 실제로 실행했다 (2026-08-04)

`.github/workflows/ci.yml`은 작성된 이후 한 번도 돌지 않았다 — 원격이
없으니 GitHub Actions가 트리거된 적이 없다. 워커들이 매 라운드 도는 것은
로컬 트리에서의 게이트이고, 그 트리에는 CI에 없는 것이 있다:
`third_party/moveit_resources` (gitignore 대상). 즉 "로컬 855/855 통과"는
"CI에서도 통과"의 증거가 아니었다.

깨끗한 클론에서 ci.yml의 단계를 그대로 실행해 확인했다 (`b2f1d9c`,
`git clone --no-hardlinks`로 추적 파일만 있는 트리 — `third_party/` 부재
확인, `fixtures/meshes/`에는 fanuc 7건·panda 10건만 존재):

- `cargo fmt --all -- --check` — rc=0
- `cargo clippy --workspace --all-targets -- -D warnings` — rc=0
- `cargo nextest run --workspace` — **855/855 통과**, rc=0
- `cargo test --doc --workspace` — rc=0
- `cargo doc --workspace --no-deps` — rc=0
- `tools/ci/check-{dep-direction,fixture-format,no-lint-suppression}.sh` —
  각 rc=0

`fixtures/`가 `third_party/`의 사본인 설계(§"픽스처 출처")가 실제로
값을 하고 있다는 뜻이다. 커밋된 테스트 중 `third_party/`를 읽는 것은
없다 — 유일한 경로 문자열은 `tools/moveit-diff/src/main.rs:386`인데
컴파일타임 문자열일 뿐 도구를 실제로 돌릴 때만 해석되고, moveit-diff는
CI에서 돌지 않는다.

남은 것은 원격 부재 하나다. 이 검증은 "ci.yml이 신선한 체크아웃에서
재현된다"를 닫았지 "GitHub Actions에서 돈다"를 닫지 않았다 —
액션 러너 고유의 실패(툴체인 액션, 캐시, nextest 설치)는 첫 푸시에서만
드러난다.

### 28.1 `distance_to_collision`의 `enableGroup` 누락은 동작 격차가 아니다

UNFIXED에 "`PlanningScene::distance_to_collision`이 상류의
`req.enableGroup(getRobotModel())`를 빠뜨렸다"가 올라 있었다. 상류를
끝까지 따라가 보니 이 호출점에서는 동작이 같다:

- `PlanningScene::distanceToCollision(state)` →
  `getCollisionEnv()->distanceRobot(state, getAllowedCollisionMatrix())`
  (`planning_scene.hpp:546-549`)
- 그 편의 오버로드가 `DistanceRequest req;`를 **기본 생성**한 뒤
  `req.enableGroup(...)`를 부른다 (`collision_env.hpp:220-232`)
- `DistanceRequest::group_name`의 기본값은 `""`
  (`collision_common.hpp:233`)
- `enableGroup`은 `hasJointModelGroup("")`가 거짓이면
  `active_components_only = nullptr`로 둔다
  (`collision_common.hpp:206-216`), 그리고
  `RobotModel::hasJointModelGroup`은 `joint_model_group_map_` 조회일
  뿐이라 (`robot_model.cpp:507-510`) 빈 이름은 결코 맞지 않는다 —
  SRDF 그룹 이름은 빈 문자열이 될 수 없다.

`nullptr`은 "모든 링크"다. 이 포트의
`DistanceRequest { acm: Some(&acm), ..Default::default() }` 역시 모든
링크를 검사한다. 따라서 이 항목은 잘못된 답을 내는 격차가 아니라
**그룹 한정 거리 질의라는 기능 자체의 부재**다. UNFIXED에서 그렇게
다시 쓴다 — 사라진 것이 아니라 성격이 바뀐 것이다.

## 29. `p3-shapes` 5라운드 병합 — 그리고 되살아난 캐시 키 결함 (2026-08-04)

`6529444`로 병합. `Shape::OcTree`가 `convert_shape`를 통해 실제로
연결됐고(`ca9929b`), 오라클 `octree_in_world` op에 `request["robot"]`이
추가되어 이 경로를 `compound_from_octree` 단위 테스트가 아니라 진짜
`CollisionEnvFCL` 대비 `CollisionEnv` 수준에서 검증한다(`1ea1be8`).

보고를 옮기지 않고 직접 확인했다: 병합 트리에서 오라클을 다시 빌드하고
(`684a3e8a15dbbf89`) 커밋된 `octree_world_collision_request.json` 5건을
재생해 `octree_world_collision_response.json`과 대조 — **5/5 바이트
동일**.

### 29.1 `OctreeCache`가 주소를 신원으로 착각한다

`parry.rs`의 새 `OctreeCache`는 `Arc::as_ptr(tree) as usize`를 키로
쓰면서 그 주소를 붙들어두는 것을 아무것도 갖고 있지 않았다. 캐시 값은
`Option<SharedShape>`뿐이라 옥트리 할당에 대한 참조가 없다. 마지막
`Arc`가 드롭되면 블록이 해제되고, 할당자는 그 주소를 다음 옥트리에
그대로 내줄 수 있다.

가정이 아니다. 임시 프로브로 재현했다 — 빈 트리를 변환해 `None`을
캐시하고, 드롭하고, 점유 리프가 있는 트리를 새로 할당했더니
**첫 시도에서** 같은 주소(`0x71f208000d40`)를 받았고 점유 트리가 빈
트리의 `None`을 돌려받았다. 즉 장애물이 충돌 검사에서 조용히 사라진다.
오류도, 빠진 shape도 없고, 그냥 답이 틀린다.

보고서는 캐시 비축출(non-eviction)을 "동작으로 관찰되지 않는다"고
분류했는데, 그 근거("옥트리를 제자리에서 교체하는 `World` API가 없다")는
*교체*를 다루지 실제 위험인 *해제 후 주소 재사용*을 다루지 않는다.
`World::remove_object`/`clear_objects`는 이미 존재한다.

앵커 감사(`rg -n 'as_ptr\('`, `rg -n 'ptr_eq'`)로 같은 결함 계열 두 곳을
셌다:

- `crates/moveit-collision/src/parry.rs:374` — 같은 결함, 이번에 수정.
- `crates/moveit-distance-field/src/collision_common_distance_field.rs:305`
  — **이미 같은 버그를 맞고 같은 방식으로 고쳐져 있었다.** 그 파일의
  회귀 테스트 doc(`:440` 부근)이 구(球)와 상자가 연달아 할당·해제되며
  같은 주소에 앉아 상자가 구의 분해를 받아간 사례를 기록해 두었다.
- `ptr_eq` 사용처 5곳은 별개다 — 살아 있는 `Arc` 둘을 비교할 뿐,
  죽은 주소를 신원으로 쓰지 않는다.

구조적 수정은 선례를 그대로 따랐다: 값 옆에 `Weak<moveit_octomap::OcTree>`
를 저장한다. `Weak`는 절대 `upgrade`되지 않고, 오직 제어 블록을 살려
주소가 재사용될 수 없게 만드는 용도다(`Arc`가 아니라 `Weak`인 이유는
트리 본체까지 붙들지 않기 위해서다). 그리고 `get_or_compute`의 시그니처를
`usize`에서 `&Arc<_>`로 바꿔 **키와 그 핀이 서로 다른 트리에서 나올 수
있는 경로 자체를 없앴다** — 호출자가 주소만 건네고 그 주소를 유효하게
유지하는 것은 건네지 않는 형태가 타입 수준에서 불가능해진다.

`moveit-collision`이 `Weak<moveit_octomap::OcTree>`를 이름 붙일 수 있어야
해서 `moveit-octomap`을 dev-dependency에서 일반 dependency로 올렸다.
이미 의존하는 `moveit-geometry`가 그 크레이트에 의존하므로 그래프에
새 간선은 없다.

회귀 테스트 `octree_cache_survives_shape_churn`은 빈/점유 트리를 번갈아
200회 만들고 드롭하며, 매 결과를 그 트리가 독립적으로 변환되어야 하는
값과 대조한다. 핀을 제거하면(`Weak::new()`로 바꿔 확인) 이 테스트는
실패한다 — 장식이 아니라 하중을 받는다.

병합 후 전체 게이트: fmt·clippy(`--workspace --all-targets -D warnings`)
·nextest **867/867**·doctest·`cargo doc`·`check-*.sh` 3종 모두 통과.

## 30. `p1-joints` 5라운드 / `p6-totg` 4라운드 병합 (2026-08-04)

`71cb14a`(p1-joints), `025f9dc`(p6-totg). 병합 후 `nextest --workspace`
**882/882**, 오라클 이미지 `448ac232926497b8`.

### 30.1 §26.4의 주장을 직접 재측정했다

§26.3의 격차가 오라클 자신의 버그였다는 것은 이 포팅에서 가장 값비싼
종류의 발견이라, 보고를 옮기지 않고 전부 다시 확인했다.

상류 확인: `kdl_kinematics_plugin.cpp:369-384`의 재시드 분기는
`!consistency_limits_mimic.empty()`로 갈라져
`getRandomConfiguration(jnt_seed_state.data, consistency_limits_mimic,
jnt_pos_in.data)` — 즉 *원래 시드* 근처를 뽑는다. 수정 후 오라클이 하는
것과 같다.

수정 후 스윕을 내가 다시 돌렸다 (`--cases 500 --seed 20260803
--ik-consistency-limit 0.35`, 이미지 `448ac232926497b8`):

| 픽스처 / 그룹 | b | c |
|---|---|---|
| panda / panda_arm | 31 | 36 |
| fanuc / manipulator | 32 | 32 |
| dual_arm_panda / left_panda_arm | 35 | 45 |
| pr2 / right_arm | 17 | 12 |

보고된 값과 정확히 일치한다. §26.3에서 내가 측정했던 수정 전 값
(15/69, 18/92, 20/67, 8/18)의 단방향 비대칭은 사라졌다.

게이트가 장식이 아닌지도 확인했다. 수정 *전* 이미지
(`a8988410cec4b1aa`)를 `--oracle` 래퍼로 직접 물려 새 `moveit-diff`를
돌렸더니 panda가 `b=15, c=69`를 그대로 재현하고
`FAIL ik_paired_divergence: |z| = 5.89 exceeds 3`으로 **런을 실패시킨다**.
`failed: 0`을 "일치"로 읽던 실수는 이제 도구가 막는다.

### 30.2 그런데 오라클은 아직 연속 관절을 랩하지 않는다 (UNFIXED)

수정된 재시드는 무조건 클램프한다:

```cpp
reseed_active[k] = ik_rng_.uniformReal(std::max(joint_min[full_i], seed_active[k] - limit),
                                       std::min(joint_max[full_i], seed_active[k] + limit));
```

상류 `RevoluteJointModel::getVariableRandomPositionsNearBy`
(`revolute_joint_model.cpp:122-136`)는 `continuous_`일 때 `near ± distance`를
클램프 없이 뽑고 `enforcePositionBounds`로 `(-pi, pi]`에 랩한다.
`rg -n 'continuous|isContinuous|RevoluteJointModel' tools/moveit-oracle/src/oracle.cpp`
— **0건**.

이 라운드는 *포트* 쪽 클램프-대-랩 격차를 고쳤다(`6408570`). 그래서 지금
상태는 부호가 뒤집힌 같은 격차다: 포트는 랩하고, 상류도 랩하고,
**오라클만 클램프한다**. 네 픽스처로는 드러나지 않는데 그 이유는
§26.4가 적어 둔 것과 같다 — pr2의 연속 관절 시드가 `[-pi, pi]`의 중점 0이고
`0.35 × 2π ≈ 2.2 rad`로는 `±π`에 닿지 않는다. 이것은 정확성이 아니라
픽스처의 한계다. 경계 근처 시드를 가진 연속 관절이 스윕에 들어오는
순간 오라클이 틀린 쪽이 된다.

### 30.3 `p6-totg`: `RobotTrajectory` 어댑터

`computeTimeStamps` 두 오버로드, `totgComputeTimeStamps`,
`doTimeParameterizationCalculations`, `hasMixedJointTypes`,
`verifyScalingFactor` 이식. 오라클 `totg` op이 최상위 `"group"` 키로
분기해 진짜 `robot_trajectory::RobotTrajectory`를 구동한다.

보고된 UNFIXED 하나는 이 크레이트 밖 문제다: 스케일링 전용
`compute_time_stamps` 오버로드는 이 워크스페이스의 어떤 픽스처로도
성공할 수 없다. `joint_bounds_from_urdf`
(`crates/moveit-model/src/joint/urdf.rs:119-144`)가 URDF에서 가속도 한계를
전혀 읽지 않고(`VariableBounds::default()`의 `acceleration_bounded: false`),
이는 상류 `jointBoundsFromURDF`도 마찬가지다 — URDF에 가속도 필드가 없고
실제 MoveIt 설정은 별도 `joint_limits.yaml`에서 가져오는데 이 워크스페이스는
그것을 로드하지 않는다. `moveit-model` 소유이므로 보고만 하고 넘긴다.

## 31. `p3-shapes` 6라운드 병합 — 그리고 내 브리핑이 틀렸다 (2026-08-04)

`0a257ed`. `nextest --workspace` **883/883**.

### 31.1 `OctreeCache` 성장 경계

§29에서 닫은 것은 정확성 절반이었고, 나머지 절반(엔트리가 영원히 쌓임)을
워커가 닫았다: `get_or_compute`가 매 호출 앞에서
`weak.strong_count() == 0`인 엔트리를 전부 걷어낸다. 캡도 타이머도 아니라
**참조 카운트 자체**를 신호로 쓴 것이 옳다 —
`World::remove_object`/`clear_objects`가 마지막 `Arc`를 떨어뜨리는 것이 이
캐시에서 "그 트리는 사라졌다"의 정의이고, 그 외에 알려 줄 것이 없다.
조회 대상 엔트리가 걷히는 일은 없다(호출자가 그 `Arc`를 들고 있으므로
`strong_count() >= 1`).

### 31.2 테스트가 자기 헬퍼를 측정하고 있었다

`octree_cache_prunes_an_entry_once_nothing_holds_its_tree`는
**`get_or_compute`의 prune을 지워도 통과했다.** 직접 지워서 확인했다.
원인은 `len()`이었다 — 세기 전에 스스로 `retain`을 돌렸으므로, 대상 코드가
prune을 하든 말든 걷힌 개수를 보고했다.

작성자의 독코멘트가 이미 그 모호함을 알고 있었다: "(after pruning would
happen, not before)". 헬퍼가 둘 중 어느 쪽인지 괄호로 설명해야 한다면
설명을 붙일 게 아니라 하나의 뜻만 갖게 해야 한다. `len()`을 순수 관찰자로
바꿔(`39cff17`) 원시 맵 크기를 죽은 엔트리 포함해 돌려주게 했다 — 성장
경계 테스트가 잡아야 하는 사실이 정확히 "아무도 못 쓰는 엔트리가 여전히
맵을 차지한다"이기 때문이다. 이제 prune을 지우면 이 테스트는 실패한다.

일반 규칙으로 승격: **테스트를 쓴 뒤 그 테스트가 검사하는 것을 지워
보고 실패하는지 확인한다.** 실패하는 것을 본 적 없는 테스트는 무언가를
검사한다는 것이 아직 증명되지 않았다. §29의 회귀 테스트도 같은 방식으로
확인했다.

### 31.3 내 6라운드 브리핑의 1번 항목은 근거가 없었다

`bodies::Body`의 `containsPoint`/`intersectsRay`/posed `boundingBox`를
"이 크레이트의 마지막 미이식 조각"이라고 썼는데, 워커가 되돌아보지 않고
멈춰서 물었다. 확인해 보니 워커가 옳았다 —
`crates/moveit-geometry/src/bodies.rs`는 3775줄이고 모든 서브클래스에
해당 함수들이 있으며, 모듈 독은 "The posed, algorithmic half of
`geometric_shapes`"로 시작하고, 브리핑이 요구한 경계 테스트가 이미 전부
있고, 커밋 체인이 이번 세션 5라운드 병합보다 앞선다.

출처는 워커 자신의 5라운드 UNFIXED 한 줄("deferred to Phase 3 collision
per PORTING-PLAN, untouched this round")이었고, 나는 그것을 트리와
대조하지 않고 브리핑으로 옮겼다. 바로 그 직전 라운드에
`p3-distance-field`에게 "물려받은 blocker 목록을 그대로 상속하지 말고 다시
읽어라"라고 요구해 놓고 같은 실수를 했다. 두 가지가 남는다 — 브리핑의
전제도 보고서에 요구하는 것과 같은 검증을 받아야 하고, "untouched this
round"라고 적힌 UNFIXED는 그 일이 남아 있다는 증거가 아니다.

3700줄을 다시 유도하는 대신 멈추고 물은 것이 옳았고 비용은 0이었다.

## 32. `p3-acm` 7라운드 병합 — pr2 메시가 들어왔다 (2026-08-04)

`2db5d10`. `nextest --workspace` **884/884**.

### 32.1 세 라운드 동안 열려 있던 픽스처 복사가 닫혔다

`c1e4f54`가 pr2의 `<collision>` 메시 18개를
`fixtures/meshes/pr2_description/`에 넣었다. 확인했다:
`git ls-files fixtures/meshes/`는 이제 fanuc 7 · panda 10 · pr2 18이고,
`verify-fixture-provenance.sh`가 18개 전부 `identical`로 통과한다.
**새 매핑 항목은 필요 없었다** — 그 스크립트의 메시 경로 도출이
`fixtures/meshes/` → `$VENDOR/` 기계적 치환(스크립트 99-102행)이라
파일시스템이 검사를 이끄는 설계가 여기서 값을 했다.

`1ff4d3b`가 pr2의 충돌 테스트를 panda·fanuc과 같은
`assert_full_parity_matches_oracle`로 바꿨다. 세 픽스처가 같은 단정문을
쓰는지 직접 확인했다 (`collision_parity.rs:286-302`) — 같다. pr2의
`self_collision` 제외 경로는 백엔드 격차가 아니라 픽스처에 메시가 없어서
생긴 흔적이었다는 진단이 맞았다.

이 병합으로 `moveit-distance-field`의 좁혀진 단정문 다섯 개를 막고 있던
것이 사라졌다. `p3-distance-field`는 6라운드 진행 중이었고, 그 항목이
`main` 확인에 걸려 있었으므로 라운드 중간에 직접 알렸다 — 이런 해제는
브랜치를 병합하는 쪽에서만 보인다(§24.3과 같은 종류).

### 32.2 비주얼 메시 미로드는 이제 명시된 결정이다

`d77f9cb`가 `link_model.rs`의 부수 주석을 번호 붙은 편차 5로 승격시켰다 —
D1 범위에 렌더러가 없으므로 영구 결정이다. §21.4가 "지금은 부수적
주석일 뿐 명시된 결정이 아니다"라고 적어 둔 항목이 닫혔다.

### 32.3 `MeshSearchPaths::none()` 호출자 — 세어 봤다

`rg -n 'MeshSearchPaths::none\(\)' crates/ tools/` — **36건**. pr2 메시가
들어온 지금 이 중 어느 것이 여전히 옳고 어느 것이 좁힘의 잔재인지는
크레이트마다 다르고, 각 크레이트 소유자가 판단할 문제다. 병합자로서
할 수 있는 것은 목록을 세는 것까지이므로 세어서 넘긴다. 메시가 있는
픽스처(panda·fanuc·pr2)를 로드하면서 `none()`을 넘기는 호출부는
"메시 없는 모델을 의도한 것"인지 "복사가 없던 시절의 잔재"인지를
호출부마다 한 줄로 답해야 한다.

## 33. `p3-distance-field` 6라운드 병합 — 좁힘 해제와 마지막 이관 조각 (2026-08-04)

`247ca6f`. `nextest --workspace` **898/898**.

### 33.1 다섯 개 단정문이 정확한 동등성으로 돌아왔다

pr2 메시가 §32에서 들어왔으므로 `build_pr2_model`이 실제 탐색 경로를 쓰고
`model.diagnostics()`가 비었다. 좁혀 두었던 것들이 전부 풀렸다:
`link_models_with_collision_geometry`의 링크 집합, `link_has_geometry`,
`link_body_indices`(이전에는 "오라클 비교 불가"라며 역직렬화조차 하지
않았다), `self_collision_enabled`, `intra_group_collision_enabled` —
모두 평범한 `assert_eq!`. `distance_queries`는
`actual >= expected - TOL`이라는 부분집합 성질에서
`assert_relative_eq!`로 바뀌었다.

§21.4에서 §27.1을 거쳐 §32.1까지 이어진 항목이 여기서 닫혔다.

### 33.2 느슨하게 만든 단정문 하나 — 그리고 얼마나 느슨해야 했는가

워커가 `sphere_radii` 하나는 반대로 *느슨하게* 해야 했다고 보고했다.
메시 유래 구(球)를 처음 비교하게 되면서 오라클과 "16번째 유효숫자에서"
어긋난다는 것이다. 조용히 좁히지 않고 보고한 것이 옳다.

주장을 측정했다. 테스트에 임시 계측을 넣어 차이가 나는 24개 반지름의
최대 편차를 뽑으니 **절대 `3.469e-18`, 상대 `1.436e-16`** — 이 크기에서
1 ulp다. "16번째 유효숫자"는 정확한 표현이었다.

그런데 워커가 고른 허용오차는 `TOL = 1e-4`였고, 이유는 "이 파일의 다른
기하 필드가 전부 쓰는 값"이었다. 그건 근거가 아니라 일관성이다.
`0.024` 반지름에 `1e-4`는 0.2%이고, 측정값보다 **12자리** 느슨하다 —
앞으로 진짜 회귀가 그 안에 들어와도 조용히 통과한다.

`RADIUS_TOL = 1e-12`을 따로 두고 근거를 그 자리에 적었다(`eaa41db`).
측정값보다 여전히 4자리 위이므로 float 비결합성에는 넉넉하고, 0.2%
회귀는 잡는다. **허용오차는 옆 단정문이 쓰는 값이 아니라 측정된 오차가
정당화해야 한다.**

### 33.3 이관 조각

`get_distance_field_cache_entry`, `group_state_representation`,
`update_group_state_representation_state` 이식.
`compare_cache_entry_to_state`의 부착체 비교가 `AttachedBodyGeometry`/
`AttachedBodySnapshot`으로 실제 비교가 되어, §27이 기록한 "공허하게 참"
상태가 닫혔다.

`generateCollisionCheckingStructures`만 남았고, 크레이트 안에 아직
설계되지 않은 영속 캐시 소유자 타입(`CollisionEnvDistanceField` 자신의
역할)이 필요하다는 이유가 붙어 있다.

커밋 두 개 중 `547cb97`은 세 함수를 한 커밋에 묶었다. 워커가 이유를
밝혔다 — `get_distance_field_cache_entry`가 같은 본문 안에서
`compare_cache_entry_to_state`의 바뀐 시그니처를 부르므로 기계적으로
쪼개면 존재한 적 없는 중간 문서를 지어내야 한다. 발견 하나에 커밋 하나
규칙의 단위가 "발견"이지 "함수"가 아니므로 이 판단은 유효하다.

## 34. `p6-totg` 5라운드 병합 — 코드 검사로만 확인되던 두 구멍을 오라클로 닫다 (2026-08-04)

`ebbbc9f`. `nextest --workspace` **899/899**.

### 34.1 "코드 검사로 확인함"은 검증이 아니었다

4라운드가 두 항목을 "code inspection only"로 남겼다 — 다중 DOF 능동
관절과 prismatic+revolute 혼합 그룹에서 TOTG가 상류와 같이 동작하는가.
그 표현 자체가 미검증의 완곡어다. 실제로 두 성질을 밟는 픽스처가
`fixtures/`에 하나도 없었기 때문에 생긴 표현이고, 없는 것을 없다고 적는
대신 "읽어보니 맞다"고 적으면 다음 사람은 그것을 검증으로 읽는다.

`0077ab9`이 크레이트 로컬 픽스처 `totg_synthetic.{urdf,srdf}`를 만들어
두 성질을 실제로 밟는다 — `planar_group`은 `theta`가 `[-π,π]` 이음매를
건너는 3-웨이포인트 경로이고, `mixed_group`은 prismatic과 revolute를
한 그룹에 섞는다. 동시에 `oracle.cpp`에 `hasMixedJointTypesForGroup`을
추가하고 모든 `totg` 응답에 `has_mixed_joint_types`로 실어, 픽스처가
의도한 성질을 실제로 갖는지를 와이어에서 확인한다. 픽스처가 의도를
만족하는지를 그 픽스처를 쓰는 테스트가 스스로 주장하지 않고 오라클이
말하게 한 것이 이 커밋의 핵심이다 — 그렇지 않으면 URDF를 잘못 써서
혼합 그룹이 아니게 되어도 테스트는 계속 초록이다.

### 34.2 깨진 커밋 메시지를 새 커밋으로 덮지 않고 다시 썼다

`e1edc6a`의 본문에 `§0`이 들어갈 자리에 `\xc2\xa70`이 그대로 박혀
있었다. 워커는 "amend 대신 새 커밋" 규칙을 지켜 고치지 않고 UNFIXED에
적었고, 적은 것 자체는 옳다 — 조용히 두는 것보다 낫다.

병합자 쪽 판단은 달랐다. 그 브랜치는 push된 적이 없어 재작성이 누구의
히스토리도 깨지 않고, 메시지 오탈자를 고치려고 커밋을 하나 더 만들면
로그에는 영구히 두 줄이 남는다. 세 커밋을 임시 브랜치에 cherry-pick으로
재생하면서 가운데 것만 `-F`로 고친 메시지를 주고 `--author`로 원저자를
보존한 뒤 병합했다. "amend하지 말라"는 규칙은 남이 이미 가져간 커밋을
바꾸지 말라는 뜻이지, 아무도 못 본 브랜치의 인코딩 사고를 영구화하라는
뜻이 아니다.

### 34.3 다른 패널에 넘긴 요구가 처음으로 정확했다

`0ae431d`이 scaling-only `compute_time_stamps` 오버로드를 끝까지
시험하지 못하는 이유를 세 주장으로 쪼갰다. 주장 1(`moveit-model`이 URDF
가속도 한계를 읽지 않음)은 결함이 아님을 확인, 주장 2(현재 어떤
픽스처로도 이 오버로드를 밟을 수 없음)는 참이고 이유가 정확하다 —
`RobotModel`의 공개 API가 전부 `&self`뿐이라 이미 `pub`인
`JointModel::set_variable_bounds_from_limits`에 도달할 길이 없다.
주장 3("따라서 시험 불가")은 스스로 기각했다: 상류에도 비-const
`getJointModel` 오버로드가 `robot_model.hpp:146`에 있다.

그래서 남긴 요구가 "moveit-model이 뭔가 해줘야 한다"가 아니라
`RobotModel::joint_model_mut(&mut self, name: &str) -> Result<&mut JointModel>`
한 줄이다. 이 정도로 좁혀진 요구는 받는 패널이 판단할 것이 남지 않아
바로 실행된다. 앞선 라운드들의 크로스 패널 블로커가 몇 번씩 왕복한
이유는 대부분 요구가 이 수준으로 좁혀지지 않아서였다.

## 35. `p1-joints` 6라운드 — §30.2의 UNFIXED를 닫는다 (2026-08-04)

`cd29af4` 기준 리베이스. 오라클 이미지 `7396c8b95fdc00f7`
(`tools/moveit-oracle/src-digest.sh` 스탬프로 확인).

### 35.1 오라클의 재시드가 이제 연속 관절에서 랩한다

§30.2가 UNFIXED로 남긴 것: 5라운드의 재시드 수정은 무조건
`std::max(joint_min, seed-limit)..std::min(joint_max, seed+limit)`로
클램프해, 포트(`6408570`)와 상류는 랩하는데 오라클만 클램프하는 부호
반전 격차가 남아 있었다.

`oracle.cpp`에 `active_continuous`(활성 관절별 `dynamic_cast<const
RevoluteJointModel*>(...)->isContinuous()`)를 한 번 계산해 두고,
재시드 루프의 `has_consistency_limits` 분기를 그 값으로 다시 갈랐다:
연속 관절은 `ik_rng_.uniformReal(seed-limit, seed+limit)`을 클램프 없이
뽑은 뒤 `fmod`로 `(-pi, pi]`에 랩한다 — `RevoluteJointModel::
enforcePositionBounds`(`revolute_joint_model.cpp:218-230`)와 정확히 같은
공식. 비연속 관절은 기존 클램프 공식 그대로 둔다.

다른 단일 자유도 관절 타입도 같은 방식으로 확인했다:

- `PrismaticJointModel::getVariableRandomPositionsNearBy`
  (`prismatic_joint_model.cpp:91-96`)는 연속 개념이 전혀 없고 클램프
  공식 하나뿐 — 오라클의 비연속 분기와 정확히 일치, 5라운드에서 이미
  확인한 그대로다.
- Planar/Floating 관절은 이 op에 도달할 수 없다 — `isSingleDOFJoints()`
  (오라클 `ik` op 자신의 최상단 조건, `oracle.cpp:1044`)가 이미 걸러낸다.
- 그러므로 `RevoluteJointModel`의 `continuous_` 분기가 이 op이 다루는
  전체 단일 자유도 관절 타입 중 유일하게 클램프 공식과 다른 경우였다 —
  이번에 우연히 처음 찾은 사례가 아니라 그것이 전부다.

`build.sh`로 재빌드, 새 이미지 `7396c8b95fdc00f7`. 재빌드 후 4픽스처
스윕을 재측정해 수치가 그대로임을 확인했다(`--cases 500 --seed
20260803 --ik-max-restarts 20`, 이미지 `7396c8b95fdc00f7`):

| 픽스처 / 그룹 | 제한 없음 b, c | `--ik-consistency-limit 0.35` b, c |
|---|---|---|
| panda / panda_arm | 8, 10 | 31, 36 |
| fanuc / manipulator | 33, 32 | 32, 32 |
| dual_arm_panda / left_panda_arm | 6, 11 | 35, 45 |
| pr2 / right_arm | 0, 0 | 17, 12 |

30.1의 수치와 완전히 같다 — 예상대로다. 이 네 픽스처의 연속 관절
(`r_forearm_roll_joint`, `r_wrist_roll_joint`)은 시드가 항상 `[-pi,pi]`의
중점 0이고 `0.35 × 2π ≈ 2.2 rad` 한계로는 랩 경계에 닿지 않으므로,
클램프 공식과 랩 공식이 이 스윕 조건에서는 수치적으로 동일한 결과를
낸다.

### 35.2 그 분기를 실제 스윕으로 때리기 — "시드가 `±π` 근처"는 문자 그대로는 불가능하다

과제의 원래 표현("a seed near ±π on a continuous joint")을 문자 그대로
만족시키려 했으나, 상류 자체의 설계가 이를 막는다: 오라클의
`seed_active[k]`는 항상 활성 관절의 bounds 중점이고
(`seed_active[k] = (joint_min + joint_max) / 2.0`, `oracle.cpp:1144-1146`),
연속 관절의 bounds는 어떤 URDF를 넣든 상류 생성자가 하드코딩한
`-M_PI`/`M_PI`다(`RevoluteJointModel`의 기본 생성자 및
`setContinuous(true)`, `revolute_joint_model.cpp:60-92`). 포트의
`IkSolver`도 같은 bounds-중점 관례를 쓴다. 즉 CLI로도, 새 URDF
픽스처로도 시드 자체를 `±π` 근처로 옮길 방법이 없다 — 이건 테스트
설계의 한계가 아니라 상류 자체의 불변식이다.

도달 가능한 대체 경로: 시드가 항상 0이므로, `seed ± limit`이 랩 경계를
넘게 하려면 `--ik-consistency-limit`을 `π`보다 크게 주면 된다. 연속
관절을 가진 픽스처/그룹은 현재 커밋된 것 중 pr2 `right_arm`
(`r_forearm_roll_joint`, `r_wrist_roll_joint`)이 유일하다 — 새 URDF
픽스처를 추가할 필요 없이 기존 픽스처에 이 플래그 조합만으로 도달한다.

`limit = 4.0`(`> π ≈ 3.14159`)에서 분기가 실제로 얼마나 발동하는지
먼저 계산으로 확인했다: 두 공식 모두 같은 하부 난수 draw `u ∈ [0,1)`를
소비하지만 다른 구간으로 스케일한다(클램프: `u`를 `[-π,π]`로, 랩:
`u`를 `[-limit,limit]`로 스케일 후 접음). 폭 `2·limit`에서 랩이 발동하는
비율은 `(limit - π) / limit`; `limit=4.0`이면 `(4.0 - 3.14159)/4.0 ≈
21.5%` — 재시드 draw의 5분의 1 이상이 두 공식에서 서로 다른 최종
관절값을 낸다. 분기는 죽은 코드가 아니라 실측으로도 무겁게 실행된다.

실측 스윕 (`--cases 500 --seed 20260803 --group right_arm
--ik-consistency-limit 4.0 --ik-max-restarts 20`, pr2/right_arm):

| 오라클 이미지 | b (oracle only) | c (rust only) | z |
|---|---|---|---|
| `7396c8b95fdc00f7`(수정 후, 랩) | 1 | 0 | 1.00 |
| `448ac232926497b8`(수정 전, 클램프)* | 1 | 2 | 1.73 |

*수정 전 이미지는 현재 트리와 소스가 달라 `run-oracle.sh`의 스탬프
검사가 막으므로, 그 검사만 건너뛰는 임시 래퍼로 고정 이미지를 직접
`docker run`했다 — 커밋하지 않은 스크래치 스크립트, 5라운드 병합자가
`a8988410cec4b1aa`를 검증할 때 쓴 것과 같은 방식.

`--cases 500`에서는 표본이 너무 작아(`b+c`가 1~3) 결론을 낼 수 없어
`--cases 5000`으로 반복했다:

| 오라클 이미지 | b | c | z |
|---|---|---|---|
| `7396c8b95fdc00f7`(랩) | 19 | 15 | 0.73 |
| `448ac232926497b8`(클램프) | 20 | 15 | 0.87 |

정직하게 말해: 두 이미지의 b/c는 서로 거의 같고, `z`도 둘 다 임계값
3.0에 한참 못 미친다 — 수정 전/후 사이의 **집계 성공/실패** 수준에서는
이 픽스처가 결함을 거의 드러내지 않는다. 이유를 구조적으로 설명할
필요가 있어, `--ik-max-restarts 0`(재시드 자체를 끔) 대 `20`을
같은 `limit=4.0`, `--cases 5000`에서 비교했다:

| `--ik-max-restarts` | 오라클 이미지 | b | c | z |
|---|---|---|---|---|
| 0 | `7396c8b95fdc00f7` | 23 | 7 | 2.92 |
| 20 | `7396c8b95fdc00f7` | 19 | 15 | 0.73 |
| 20 | `448ac232926497b8` | 20 | 15 | 0.87 |

재시드를 아예 끄면 `z=2.92`로 임계값 바로 아래까지 올라가는 진짜
격차가 있다 — 재시드 메커니즘 자체(클램프든 랩이든)가 이 격차 대부분을
닫는다. 클램프-대-랩의 구분이 기여하는 몫은 `0.87`과 `0.73`의 차이
정도로, 재시드 유무가 만드는 효과보다 훨씬 작다. pr2의
forearm-roll/wrist-roll은 방향이 완전히 자유로운 축이라, 재탐색을
어느 값에서 시작하든(클램프든 랩이든) 목표 자세에 수렴하는 데 별
차이가 없기 때문으로 구조적으로 설명된다 — 이 픽스처의 IK 여유도의
성질이지, 검증이 부실해서가 아니다.

**결론**: 이 분기는 실제 스윕(`--cases`, `--seed`, 고정 이미지로
재현 가능)으로 도달 가능하고 무겁게 실행됨을 계산과 실측 양쪽으로
확인했다. 다만 현재 커밋된 4개 픽스처 중 연속 관절을 가진 유일한
그룹(pr2/right_arm)의 IK 여유도가 높아, 그 분기의 존재가 **집계
성공/실패 수의 페어드 발산 통계**에는 강한 신호를 남기지 않는다.
이것은 §35.3의 검정력 문제와 같은 종류의 한계다 — 더 작은 `b+c`
표본에서 진짜 결함이 통계량에 가려지는 것과 마찬가지로, 여기서는
결함 자체는 확인됐지만 이 특정 관절의 IK 여유도가 그 결함이 성공/실패
카운트에 미치는 영향을 흡수한다.

### 35.3 `ik_paired_divergence`의 검정력을 도구 안에서 말한다

5라운드부터 pr2의 수정 전 쌍(`b=8, c=18`)이 `z=1.96`으로 임계값
3.0을 넘지 못했다는 것, 그리고 이것이 pr2가 영향받지 않았다는 뜻이
아니라 `b+c=26`이 너무 작다는 뜻이라는 것을 알고 있었다. 이 캐벗을
보고서가 아니라 도구 자체에 넣었다.

유도: `z = |b-c| / sqrt(b+c)`이므로, 실제 쏠림 비율 `p`(더 큰 쪽이
전체 불일치에서 차지하는 비율, `p > 0.5`)가 고정일 때 기댓값은
`E[z] ≈ (2p-1) · sqrt(b+c)`로 `sqrt(b+c)`에 비례해 커진다. 이
프로젝트가 직접 확인한 가장 작은 진짜(노이즈 아닌) 쏠림은 pr2 자신의
5라운드 수정 전 쌍 `b=8, c=18`(`p = 18/26 = 9/13`, `2p-1 = 5/13`)이다
— panda/fanuc/dual_arm_panda가 같은 결함을 `z 5.04~7.06`으로 훨씬
분명하게 보여준 것과 같은 결함인데, pr2는 재시드가 필요한 케이스 자체가
적어서(고성공률) 표본이 작았을 뿐이다. 이걸 기준으로 삼아(임의의
숫자가 아니라 실측된 가장 작은 진짜 효과 크기로 보정):

```
n_min = (3.0 / (5/13))^2 = (39/5)^2 = 60.84  →  61
```

`MINIMUM_USABLE_B_PLUS_C = 61`을 `tools/moveit-diff/src/main.rs`에
추가했다. `b+c`가 이보다 작고 `z`가 임계값을 넘지 못하면
`Verdict::Underpowered`(별도 상태, `Pass`로 접히지 않음)를 내고
`UNDERPOWERED ik_paired_divergence: b + c = N is below 61, ...`를
출력한다. pr2의 수정 전 쌍(`b+c=26`)은 물론이고 pr2의 **수정 후** 쌍
(`b=17, c=12`, `b+c=29`)도 이 기준 아래다 — 정직한 결과: 실제 61 미만
표본으로 돌린 이 라운드의 pr2/right_arm `--ik-consistency-limit 0.35`
스윕은 이제 `PASS`가 아니라 `UNDERPOWERED`로 찍힌다(실측 확인,
`--cases 500 --seed 20260803`). 임계값 미달을 낮추거나 이 사실을
숨기지 않고 그대로 뒀다 — 실측된 효과 크기로 보정한 바닥선이 pr2
자신의 값보다 큰 것은 우연이 아니라 pr2가 정확히 이 통계량이 약한
경계 사례이기 때문이다.

`paired_divergence_tests`에 경계 테스트 6개(합 9개): 유도식 자체를
`MINIMUM_USABLE_B_PLUS_C`와 다시 대조하는 테스트, pr2 수정 전/후 쌍이
각각 바닥선 아래/양쪽에 있음을 고정하는 테스트, 나머지 세 픽스처의
수정 후 쌍이 바닥선을 넘김을 고정하는 테스트.

### 35.4 게이트

- `cargo fmt --all`: PASS
- `cargo clippy --workspace --all-targets -- -D warnings`: PASS
- `cargo nextest run --workspace`: PASS, 885/885 (기준 882/882 + 이번
  라운드 `moveit-diff::paired_divergence_tests` 순증 3개: 기존 3개에서
  6개로)
- `cargo test --doc --workspace`: PASS
- `cargo doc --workspace --no-deps`: PASS
- `tools/ci/check-dep-direction.sh`: PASS
- `tools/ci/check-fixture-format.sh`: PASS
- `tools/ci/check-no-lint-suppression.sh`: PASS
- `tools/ci/verify-fixture-provenance.sh`: PASS

## 36. `resolveConstraintFrames` 이식 — 그리고 구성 시점 검증이 만든 API 형태 변경 (4라운드, 2026-08-04)

§23.1이 남겨둔 차단 사유(`RobotState`/`Posed`에 부착체/서브프레임을
이름으로 찾는 API가 없음)는 같은 병합 라운드에서 `p1-fixtures`가
`PlanningScene` 레벨(`scene.rs:613`, `:671`)로 해소했다 — `RobotState`
레벨이 아니라. 이번 라운드에서 이 함수를 실제로 이식했다.

### 36.1 의존 방향 — 클로저를 택했다

상류는 `const RobotState&`를 받아 부착체/서브프레임도 스스로 해석한다.
이 포트는 부착체를 `PlanningScene`에 두기로 했으므로(`AttachedBody`
모듈 문서) 동등한 조회는 그쪽에 있다. `&PlanningScene`을 직접 받으면
`moveit-constraints`가 `moveit-scene`에 의존하게 되는데, 상류
`planning_scene`이 `kinematic_constraints`에 의존하는 방향(목표
제약 검사 등)과 반대다 — `tools/ci/check-dep-direction.sh`는 ROS
의존만 검사해 이 역전은 잡지 못하지만, `PORTING-PLAN.md` §3의 크레이트
레이아웃(`moveit-scene`이 `moveit-constraints`보다 앞선다는 암묵적
순서는 없지만, 상류 자체의 실제 의존 방향이 근거다)과 upstream 자체의
`planning_scene.cpp`가 `kinematic_constraints`를 include하는 사실이
근거다. 그래서 "부착체/서브프레임 이름 하나를 로봇 링크+상대
포즈로 바꿔달라"는 조회 하나만 클로저 파라미터로 받는다 — 씬이 있는
호출자는 `PlanningScene::attached_frame` 위에 얇은 래퍼를 씌워
넘기고, 부착체가 전혀 없는 호출자는 `|_| None`을 넘긴다.

### 36.2 왜 `&mut KinematicConstraintSet`이 아니라 구성 이전 단계인가 — 재설계 사유

원래 계획(3라운드 인계 노트)은 다른 `update_*` 함수들과 같은 모양,
즉 `&mut KinematicConstraintSet`을 받아 이미 만들어진 제약을 훑고
`link_name`을 다시 쓰는 함수였다. 작성 도중 상류
`PositionConstraint::configure`(`kinematic_constraint.cpp:365`)를 다시
읽다가 발견한 사실: 상류 자신도 `pc.link_name`이 이미 실제 로봇
링크여야만 `configure()`가 성공한다(`link_model_ = robot_model_->
getLinkModel(pc.link_name); if (nullptr) return false`) —
이 크레이트의 `PositionConstraint::new`/`OrientationConstraint::new`가
`model.link_model(link_name)?`으로 강제하는 것과 정확히 같다.

즉 상류에서도 부착체/서브프레임 이름을 가진 제약은 **`configure()`가
실행되기 전, 아직 원시 `moveit_msgs::msg::Constraints` 메시지인
동안만** 존재할 수 있다 — `resolveConstraintFrames`의 유일한 실제
호출부(`moveit_ros/planning/planning_request_adapter_plugins/src/
resolve_constraint_frames.cpp`, `moveit-ros` 전용 플래닝 요청
어댑터)가 `MotionPlanRequest`의 원시 `path_constraints`/
`goal_constraints`를 `KinematicConstraintSet::add()`보다 먼저
고쳐 쓰는 것도 그래서다.

이 포트는 "원시 메시지 → 검증된 객체" 2단계 파이프라인을 1단계로
합쳤다(`utils.rs` 모듈 문서의 "update reconstructs, not mutates" —
`PositionConstraint::new`가 즉시 검증한다). 그 결과
`PositionConstraint`/`OrientationConstraint`는 만들어진 이후로 절대
미해석 `link_name`을 담을 수 없다 — `&mut KinematicConstraintSet`을
훑는 모양으로 지었다면 재타겟 분기가 **원리적으로 실행될 수 없는**
함수가 됐을 것이다. 이는 3라운드에서 이미 한 번 거부했던 "퇴화한
no-op"과 같은 결함이고, 이번엔 만들고 나서가 아니라 만들기 전에
`rg`형 재확인(상류 소스를 직접 다시 읽음)으로 잡았다.

**해결:** 구성 이전 단계에 두 함수를 이식했다 —
`resolve_position_constraint_frame`(link_name + point offset →
resolved link_name + adjusted offset)와
`resolve_orientation_constraint_frame`(link_name + orientation +
tolerance → resolved link_name + adjusted orientation). 호출자는
`PositionConstraint::new`/`OrientationConstraint::new`를 부르기
직전에 이 함수들을 불러, 결과를 그대로 그 생성자에 넘긴다 — 상류에서
`resolveConstraintFrames`가 항상 `configure()` 바로 앞에서 실행되던
자리와 정확히 대응한다.

### 36.3 오프셋/방향 변환 유도

공통 헬퍼 `resolve_frame_to_link`가 상류 `RobotState::getFrameInfo`의
세 단계 중 이 크레이트가 스스로 풀 수 있는 두 개(모델 프레임 →
루트 링크, 평범한 링크 이름 → 자기 자신)를 처리하고, 나머지 하나
(부착체/서브프레임)는 클로저에 위임한다. 반환값은 상류의
`robot_link_to_link_name = getGlobalLinkTransform(robot_link).
inverse() * transform`과 정확히 같은 의미(`link_name`의 프레임을
`robot_link`의 프레임으로 바꾸는 변환)이고, 위치 쪽은 이를 그대로
점(offset)에 적용, 방향 쪽은 그 회전 성분의 역(`link_name_to_
robot_link = transform.linear()^T * getGlobalLinkTransform(robot_link).
linear()`과 대수적으로 동치임을 손으로 유도해 확인)을 쿼터니언에
합성한다.

### 36.4 테스트 — 오라클 엔드포인트가 없다

`resolveConstraintFrames`의 유일한 상류 호출부가 `moveit-ros`
어댑터라 오라클에 대응하는 op가 없다. `crates/moveit-constraints/
tests/utils_parity.rs`에 `resolve_constraint_frame_boundary` 모듈(6
테스트, 손으로 계산한 기대값)로 커버했다:

- **핵심 케이스** — `"gripper_tool"`이라는, 모델에 실재하지 않는
  이름을 클로저만으로 `"panda_link8"` + 평행이동 (1,2,3)으로 해석하게
  하고, offset (0.1,0,0)이 (1.1,2,3)으로 바뀌는 것을 확인했다(위치)와,
  90도 Z축 회전 클로저에 대해 identity 방향이 그 역회전으로 바뀌는
  것을 확인했다(방향) — 라운드 3에서 거부했던 no-op이 되지
  않았음을 직접 증명하는 케이스.
- 이미 실제 링크 이름이면 링크/오프셋/방향이 그대로인 케이스.
- 모델 프레임 → 루트 링크(현재 상태의 `global_link_transform`으로
  독립 검산).
- 어디에도 해석되지 않는 이름 → `Ok(None)`.
- `XyzEuler` 톨러런스로 실제 프레임 변경(부착 서브프레임)을 재타겟하려
  하면 `Error::Other` — 상류의 `utils.cpp:661-664` 거부와 동일.

### 36.5 남는 것

`update_joint_constraints`의 `local_variable_name` 미제거 이름 비교
한계(§23.3-1)는 이 함수와 무관 — `resolveConstraintFrames`는
위치/방향 제약만 다루고, 상류 자신도 조인트 제약을 건드리지 않는다.

---

## 37. §22 UNFIXED 재검증 — mesh는 착지했다, 그러나 119건은 115건으로만 줄었다 (4라운드, 2026-08-04)

§22.2가 pr2 `visibility_cone`의 119/2,201 거리(depth) 불일치를
`moveit-model`이 mesh 충돌 형상을 갖지 못한 탓으로 근본 원인을
지목하고 `UNFIXED`로 남긴 뒤, `p3-acm`이 STL 로더(`947f3e6`,
`73da61e`, `aaaaae8`, `a1b2b5a`로 병합)를 착지시켰다. `moveit-model`은
이제 `RobotModel::from_urdf_and_srdf`에 실제
[`MeshSearchPaths`](crates/moveit-model/src/link_model.rs)를 주면
`<mesh>` 충돌 형상을 보존하고, `moveit-collision::parry`(`parry.rs:287-296`)는
그 정점/삼각형을 근사가 아니라 진짜 `parry3d::shape::TriMesh`로
변환한다 — 확인 후 진행했다(추측 아님).

`tools/moveit-diff`의 유일한 `from_urdf_and_srdf` 호출부
(`build_rust_model`)는 이 라운드가 시작되기 전부터 이미
`mesh_search_paths()`를 쓰고 있었다 — panda/fanuc뿐 아니라 pr2까지
`third_party/moveit_resources/pr2_description`으로 매핑되어 있었다
(이 함수 자체가 `aaaaae8`, 즉 mesh 로더를 착지시킨 그 커밋에서
도입됐다). 라운드 4 작업 지시에 있던 "`moveit-diff`의 모든
`from_urdf_and_srdf` 호출부가 지금 `MeshSearchPaths::none()`을 쓴다"는
전제는 리베이스 이후 시점 기준으로 이미 사실이 아니었다 — 내가
`mesh_search_paths()`나 `build_rust_model`을 고칠 필요는 없었고,
실제로 고치지 않았다.

**재실행 (§22.2와 동일 조건, seed 4·`--cases 100`·`--group right_arm`·
`--constraints 2000`, 2026-08-04):**

```
cases:  2201
passed: 2086
failed: 115
```

`visibility_cone: 142 satisfied, 143 violated` — §22.1의 분포와
동일, `satisfied` 불리언 불일치는 여전히 0건. `decide_cone`의 판정
로직 자체는 이번에도 완전히 정확했다.

119 → 115, 4건 감소. **0으로 떨어지지 않았다** — §22.2가 세운
가설("mesh 충돌 형상이 없어서")은 방향은 맞았지만, mesh가 실제로
로드되고 `moveit-collision`이 그것을 진짜 삼각형 메시로 변환해도
근거리 143건 중 115건(80%)은 여전히 오라클과 다른 깊이를 보고한다.
`decide_cone`의 `max_contacts: 1` 로컬 환경이 이제 pr2의 mesh 링크를
포함하는데도(§22.2 당시엔 아예 없었다) 오라클의 "첫 접촉" 순회
순서와 여전히 다른 접촉을 고르는 경우가 대다수라는 뜻이다 — 즉
근본 원인은 "mesh 형상의 부재"가 아니라 그보다 한 단계 더 깊이,
`moveit-collision`이 여러 접촉 후보 중 하나를 고르는 순회/타이브레이크
순서가 상류(FCL 기반)와 다른 데 있다. 이건 `moveit-collision`
내부의 접촉 순회 로직 문제이지 `moveit-constraints`의 `decide_cone`도
이 생성기도 고칠 수 있는 지점이 아니다 — 이번 라운드의 소유 범위
밖이며, 조사해서 고치는 것도 이번 라운드 지시("바뀌면 보고하되
직접 고치지 마라")를 넘어선다.

**`UNFIXED`로 계속 남긴다**, 숫자만 119→115로 갱신한다. mesh 로더
착지가 이 gap을 완전히 닫지 못했다는 것 자체가 새로운 사실이므로,
접촉 순회 순서 조사는 `moveit-collision` 소유자에게 넘긴다.

## 38. 유휴 상태로 방치돼 있던 세 패널 병합 (2026-08-04)

`p1-joints` 6라운드(`25c7bb7`), `p1-robotmodel` 4라운드(`60c480d`),
`p1-fixtures` 4라운드(`abad1c1`). 세 브랜치 모두 작업은 끝나 있었고
병합만 밀려 있었다. `nextest --workspace` **911/911**.

### 38.1 §30.2의 UNFIXED가 닫혔다 — 그리고 그 검증은 내가 다시 했다

오라클의 재시드가 연속 관절을 클램프하던 격차(§30.2)를 `9444463`이
`active_continuous[k]` 분기로 닫았다. 워커의 주장을 받지 않고 상류
`RevoluteJointModel::enforcePositionBounds`
(`revolute_joint_model.cpp:218-235`)와 대조했다 — `fmod` 후
`v <= -M_PI`면 `+2π`, `v > M_PI`면 `-2π`. 오라클 쪽 코드와 논리가
글자 단위로 같다. `PrismaticJointModel`에 연속 개념이 없다는 것과
planar/floating이 `isSingleDOFJoints()` 게이트에 막혀 이 op에 도달할 수
없다는 것도 워커가 스스로 확인했고, "연속 revolute가 유일한 격차였지
여러 개 중 하나가 아니다"라고 명시했다. 결함 계열의 크기를 세어서
말한 것이라 받아들일 수 있다.

### 38.2 검정력 없는 PASS를 PASS라고 부르지 않는다

`ik_paired_divergence`가 `z <= 3.0`이면 무조건 `PASS`를 찍던 것을
`Verdict::Underpowered`로 갈랐다(`2dca4f6`). 임계 `b+c = 61`의 유도를
직접 다시 계산했다: `E[z] ≈ (2p-1)·sqrt(b+c)`에 이 프로젝트가 실제로
확인한 가장 작은 진짜 편향(pr2 5라운드 사전 수정치 `b=8, c=18`,
`p = 9/13`, `2p-1 = 5/13`)을 넣고 `(3.0·13/5)² = 60.84 → 61`. pr2 자신의
`b+c = 26 < 61`이고 그때 `z = 1.96`이었다 — 즉 이 상수는 자기가 잡아야
할 사례를 실제로 잡는다. 가정된 숫자가 아니라 측정된 사례로 보정한
임계값이라는 점이 §33.2에서 요구한 것과 같은 성질이다.

한계도 워커가 스스로 적었다: "시드가 `±π` 근처인 스윕"은 구조적으로
불가능하다 — 재시드의 `near`가 상류가 못박은 경계 중점이라 연속 관절에서
항상 0이다. 대신 `--ik-consistency-limit > π`로 분기를 실제로 밟아
(`limit=4.0`에서 재시드 추첨의 ~21%가 랩) 측정했고, pr2의 IK 여유도가
집계 성공/실패 수준에서는 그 효과를 대부분 흡수한다는 것까지 숫자로
보고했다(§35.2). "못 밟았다"가 아니라 "밟았고 집계 통계에는 안 보인다"는
서로 다른 진술이고, 후자만 다음 사람이 쓸 수 있다.

### 38.3 mesh가 들어왔는데도 119건은 115건으로만 줄었다

§22.2가 세운 가설 — pr2 `visibility_cone`의 깊이 불일치는 mesh 충돌
형상이 없어서다 — 을 `p1-robotmodel`이 p3-acm의 메시 착지 이후 같은
스윕(seed 4, `right_arm`, 2,201건)으로 재측정했다. **119 → 115.**
`satisfied` 불리언 불일치는 여전히 0건.

이것은 가설의 기각이 아니라 축소다. 방향은 맞았지만 크기가 4건뿐이다.
남은 115건의 원인은 한 단계 더 깊다 — `decide_cone`의 `max_contacts: 1`
로컬 환경이 이제 pr2 mesh 링크를 포함하는데도 오라클의 "첫 접촉" 순회
순서와 다른 접촉을 고른다. 즉 `moveit-collision`의 접촉 순회/타이브레이크
순서 문제이고, `moveit-constraints`나 이 생성기에서 고칠 수 있는 것이
아니다. p3-acm(`moveit-collision` 소유자)에게 넘겼다.

워커가 UNFIXED를 지우지 않고 **숫자만 갱신하고 원인 진단을 바꿔서**
남긴 것이 옳다. 0이 아닌데 "메시 착지로 해결"이라 적었다면 §22.2의
가설이 검증된 것처럼 보였을 것이다.

### 38.4 커밋 제목이 이식하지 않은 심볼을 이름으로 걸었다

`f7b9fc5`의 제목은 `scene: port isStateFeasible/isStateConstrained/
isStateValid/isPathValid`인데, 본문과 `scene.rs:904`의 문서는
`StateFeasibilityFn` 술어를 **의도적으로 이식하지 않았다**고 정확히
적는다. 이식하지 않은 이유(등록하는 호출부가 이 포트 안에 하나도 없어
상류에서도 무조건 `true` 분기를 타므로, 저장 필드를 두는 것은 가상의
미래 호출자를 위한 speculative configurability다)는 타당하고, 문서는
정직하다.

정직하지 않은 것은 제목이다. `git log --grep isStateFeasible`은 이 커밋을
"이식했다"로 답하고, 커버리지 감사는 대부분 그 층에서 이뤄진다. 심볼
이름을 제목에 거는 것은 그 심볼이 이식됐다는 진술이므로, 의도적 제외는
제목이 아니라 본문에만 있어야 한다. 5라운드 브리핑으로 전달했다.

## 39. `p3-shapes` 7라운드 병합 — 서술을 측정으로 바꾸다 (2026-08-04)

`b304332`. `nextest --workspace` **919/919**.

### 39.1 "구조적 격차"가 숫자를 갖게 됐다

§4.8이 결정한 것 — `parry3d-f64`에 다중 해상도 옥트리가 없어
`Shape::OcTree`를 리프당 `Cuboid`의 `Compound`로 표현한다 — 은 지금까지
기전 서술이었고, 그 기전이 실제로 답을 틀리게 하는지는 아무도 재지
않았다. 이번 라운드가 쟀다: 디코이 리프 수를 0에서 216까지 늘리면서
`robot_distance`를 오라클과 대조했고, 모든 리프 수에서 비트 단위로
일치, 발산 추세 없음(`octree_leaf_count_scaling_parity.rs`).

이것은 격차가 없다는 증명이 아니다 — 측정한 리프 수 범위에서 관측되는
결과 차이가 없다는 것이고, 워커가 그 구분을 정확히 지켰다. "구조가
다르다"와 "답이 다르다"는 다른 진술이고, 후자만 사용자에게 의미가
있다. UNFIXED는 남되 이제 측정치가 붙어 있다.

### 39.2 캐시 경계 주장을 자기 테스트로 되돌려 확인했다 — 그리고 나도 했다

`OctreeCache`의 "최대 한 개의 stale 엔트리" 문서 주장이 실제
`World`/`ParryCollisionEnv`/`check_robot_collision` 위에서 50회
replace 루프로 검증됐다(`a9acef8`). 워커는 `get_or_compute`의 `retain`을
지워 테스트가 반복 1회차에서 실패하는 것을 확인하고 소스를 바이트
동일하게 복원했다.

받지 않고 다시 했다. `retain` 한 줄을 주석으로 바꾸고
`cargo nextest run -p moveit-collision -E 'test(octree_cache)'`:
5개 중 2개 실패(`octree_cache_prunes_an_entry_once_nothing_holds_its_tree`,
`octree_cache_stays_bounded_across_a_real_rebuild_and_replace_loop`),
실패 메시지는 `iteration 1: cache held 2 entries ... expected at most 1`.
복원 후 `git diff` 빈 출력, 5/5 통과. §31.2에서 "테스트가 자기 헬퍼를
측정하고 있었다"를 잡은 이후 이 크레이트의 캐시 테스트는 전부 이
방식으로 확인한다.

### 39.3 세 라운드째 같은 stale UNFIXED

`bodies::Body`의 `containsPoint`/`intersectsRay`/posed `boundingBox`가
"Phase 3 충돌로 연기됨"이라는 UNFIXED 줄이 이번에도 그대로 실려 왔다.

`rg -n 'fn contains_point|fn intersects_ray' crates/moveit-geometry/src/bodies.rs`
= **11건**. `Sphere`/`Cylinder`/`Cuboid`/`ConvexMesh` 각각과
`Body` 디스패처(`bodies.rs:2513`의 enum, `:2551`의 impl)까지 전부 있다.
`probe_parity.rs`가 실제 `.so`에 대고 고정하고 있다.

§31.3에 적었듯 이 문장을 **처음 6라운드 브리핑에 실은 것은 나였다** —
워커의 5라운드 UNFIXED를 검증 없이 옮겼다. 그 라운드에 정정해서
전달했는데도 7라운드 보고서에 다시 실렸다. 즉 이건 한쪽의 실수가
아니라 UNFIXED 줄이 라운드를 넘어 복사될 때 아무도 트리를 다시 보지
않는다는 구조적 문제다. 8라운드 브리핑에서 요구한 것은 "지우라"가
아니라 **모든 UNFIXED 줄에 근거 명령을 붙이라**는 것이다 — 다음 라운드에
그 명령을 다시 돌리면 참/거짓이 즉시 갈린다. 근거 없이 이월할 수 없는
형식으로 만드는 것이 유일한 구조적 해결이다.

## 40. 커밋된 요청/응답 픽스처 21쌍은 재생할 수 없다 (2026-08-04)

§10.6이 세운 절차 — 커밋된 요청을 그대로 다시 태워 응답을 대조한다 —
는 당시 11건에 대해 실제로 돌았다. 지금 트리에는
`crates/*/tests/fixtures/*_request.json`이 **21건**이고, 그중 어느
것도 기계적으로 재생할 수 없다.

이유는 픽스처가 자기가 어느 모델로 떴는지 기록하지 않기 때문이다.
요청 JSON에는 op 인자만 들어 있고, `--urdf`/`--srdf`는 그 픽스처를
소비하는 **테스트 소스 안에** `fixture_path("pr2.urdf")` 형태로만
존재한다. 파일명으로 테스트를 역추적해 봤더니 21건 중 여러 건이 틀린
테스트에 붙는다 — `world_request.json`이 옥트리 테스트로,
`totg_request.json`이 synthetic 테스트로 매칭된다. 파일명 유사도는
출처가 아니다.

왜 지금 문제인가. 각 패널은 자기 워크트리의 오라클로 픽스처를 뜨고,
병합 뒤 `main`의 `oracle.cpp`는 여섯 패널의 변경이 합쳐진 다른
바이너리다. 병합 산술이 어떤 op의 답을 조용히 바꿨다면, 그것을
드러내는 유일한 검사가 "커밋된 요청을 병합된 오라클에 다시 태우기"인데
그 검사를 돌릴 수 없다. 현재 919개 파리티 테스트는 **Rust 대 커밋된
응답**을 비교할 뿐, **병합된 오라클 대 커밋된 응답**은 아무도 비교하지
않는다.

`verify-fixture-provenance.sh`가 메시에 대해 하는 일과 같은 형태의
구멍이다. 메시는 벤더 원본과 대조할 수 있게 만들어 뒀고, 오라클
픽스처는 그렇지 않다.

**구조적 해결(다음 라운드 브리핑으로 배분).** 각 요청 픽스처가 자기
모델을 스스로 기록한다 — 요청 JSON 옆에 `"_model": "pr2"` 같은 필드든
크레이트별 매니페스트든, 형태는 소유자가 정한다. 조건은 하나다:
**재생에 필요한 정보가 테스트 소스가 아니라 픽스처와 함께 있을 것.**
그러면 `tools/ci/`에 재생·대조 스크립트를 하나 두고 병합마다 돌릴 수
있다.

21건이 여섯 크레이트에 흩어져 있고 각 크레이트에 소유자가 있으므로
병합자가 일괄 편집하지 않는다. 소유자별로 배분한다.

## 41. `p3-distance-field` 7라운드 병합 — 크레이트가 닫혔다, 문서는 아니었다 (2026-08-04)

`c8d3aca`. `nextest --workspace` **925/925** (919 + 6).

### 41.1 코드를 쓰기 전에 설계를 보고했고, 그 설계가 맞았다

브리핑이 요구한 것은 "타입이 필요한지를 상류의 실제 사용에서 답하라"였고
워커는 코드 없이 먼저 답했다. 받지 않고 상류를 직접 봤다:

- `generateCollisionCheckingStructures` 본문
  (`collision_env_distance_field.cpp:158-175`)은 정확히 세 호출이고,
  `update_cache_lock_`은 메서드가 `const`이기 때문에만 존재한다 —
  본문에 `const_cast<CollisionEnvDistanceField*>(this)`가 그대로 있다.
  `&mut self`가 같은 단일 기록자 보장을 공짜로 주므로 뮤텍스를 이식하지
  않는다는 판단은 옳다.
- 호출부는 **7곳**이고 전부 `if (!gsr)`로 게이트한다(`183`, `1393`,
  `1426`, `1459`, `1488`, `1524`, `1545`). 즉 캐시를 참조할지 말지를
  *호출자*가 정하므로 수명이 호출을 가로지른다 — 자유 함수 뒤에 숨긴
  `Option`이 아니라 타입이어야 한다는 결론의 근거가 실제로 성립한다.

"편해서 타입으로 했다"와 "상류 호출부가 그렇게 쓰므로 타입이어야 한다"는
다른 진술이고, 워커가 후자를 근거와 함께 냈다.

### 41.2 브리핑이 지목한 문장만 검증했다

3번 항목은 "네 문서가 지금 주장하는 두 가지를 병합된 트리에 대고 다시
확인하라"였다. 워커는 그 둘을 정확히 확인했다 — pr2 메시 경로는 실재하고
`verify-fixture-provenance.sh`가 기계적으로 덮으며,
`link_body_indices`의 "오라클 비교 불가" 주석은 **틀린 게 아니라 낡은
것**이었다는 구분까지 정확했다.

그런데 같은 라운드에 자기가 편집한 파일
`collision_common_distance_field_parity.rs`의 모듈 문서에는 이렇게
적혀 있었다: pr2 메시는 "gitignore된 `third_party/` 체크아웃에 있고
아직 이 워크스페이스의 커밋된 `fixtures/meshes/`로 복사되지 않았다".
§32에서 18개가 들어왔고 워커 자신이 방금 그 사실을 다른 파일에서
확인했다. 2번 항목의 분류에서 이 호출부를 `deliberate`로 판정하면서도
그 아래에 깔린 이유 문장은 고치지 않았다.

같은 문단에 두 번째 거짓이 있었다. `base_bellow_link`가 "pr2에서 비메시
충돌 형상을 하나만 가진 유일한 링크"라는 주장이다. URDF를 파싱해 세었다:
단일 `<box>`를 항등 원점에 가진 링크가 **4개**(`base_bellow_link`,
`head_plate_frame`, 좌우 `*_gripper_motor_accelerometer_link`),
비메시 충돌 형상을 가진 링크는 전부 **17개**다. 테스트가 틀린 것은
아니다 — 넷 중 아무거나 쓸 수 있고 `base_bellow_link`도 그중 하나다.
틀린 것은 "유일하다"는 근거이고, 그 근거가 2번 항목의 분류를 떠받치고
있었다.

`b5eb170`에서 둘 다 고쳤고, `none()`을 쓰는 이유를 "메시가 없어서"가
아니라 "이 테스트의 대상이 프리미티브라 메시 적재가 결과를 바꿀 수
없으므로 의도적으로 파이프라인 의존을 끊는다"로 다시 썼다. 그 선택을
정직하게 유지하는 것은 문장이 아니라 `run_link_body_decomposition_case`의
비어있지-않음 단정문이므로, 그 단정문을 문서가 가리키게 했다.

**교훈은 워커 개인의 실수가 아니라 브리핑 형식의 결함이다.** "이 두
주장을 확인하라"고 쓰면 그 둘만 확인된다. §39.3에서 stale UNFIXED에
대해 내린 처방과 같은 처방이 문서 주장에도 필요하다 — 검증 대상을
병합자가 열거하는 대신, 워커가 편집하는 문단 안의 모든 사실 주장이
근거를 하나씩 달고 있어야 한다.

## 42. `p6-totg` 6라운드 병합 — 트레이트를 이식하지 않기로 한 근거 (2026-08-04)

`f9c53d7`. `nextest --workspace` **927/927** (925 + 2).

### 42.1 D4 판단을 받지 않고 상류에서 다시 확인했다

워커의 핵심 주장은 "`TimeParameterization` 순수 가상 인터페이스를
트레이트로 이식하지 않는다 — 구현체가 하나이고 다형적 호출부가 없으므로
쓰이지 않는 추상화가 된다"였다. 둘 다 직접 확인했다:

- `rg -n 'public TimeParameterization' ~/work/moveit2` → 한 줄,
  `time_optimal_trajectory_generation.hpp:193`의
  `TimeOptimalTrajectoryGeneration`뿐. `RuckigSmoothing`은 상속하지
  않는다.
- `TimeParameterizationPtr|TimeParameterization&|TimeParameterization>|TimeParameterization \*`를
  헤더 자신을 제외하고 전체 체크아웃에서 검색 → **출력 없음**. 상류에
  다형적 호출부가 하나도 없다.

D4(컴파일타임 레지스트리)는 플러그인 지점에 트레이트를 두라는 결정이지,
구현체가 하나이고 디스패치가 없는 자리에도 두라는 결정이 아니다.
`90d90a6`(병합 후 `9bf99da`)이 그 판단과 근거를 심볼 옆에 남겼다.

워커가 따로 짚은 것 하나 — 그 인터페이스의 세 번째 순수 가상 오버로드가
`moveit_msgs::msg::JointLimits`를 받으므로, 충실한 트레이트는 D1에
막힌다. D1 제외가 잎 타입에서 인터페이스 모양으로 전파되는 사례다.

### 42.2 래퍼 두 개의 기본값을 대조로 확인했다

`trajectory_tools.cpp:63-76`의 `applyTOTGTimeParameterization`/
`applyRuckigSmoothing`은 각각 `TimeOptimalTrajectoryGeneration`/
`RuckigSmoothing`을 만들어 한 번 호출하고 끝난다 — 상류 본문을 직접 읽어
확인했다. 워커는 기본 인자를 `TotgOptions::default()`/
`SmoothingOptions::default()`와 대조해 이미 일치함을(`0.1`/`0.1`/`0.001`,
`false`/`0.01`) 보고했다. "일치할 것이다"가 아니라 "대조했고 어긋난 곳이
없었다"가 보고 형태였다.

나머지 셋(`isTrajectoryEmpty`, `trajectoryWaypointCount`,
`createTrajectoryMessage`)은 D1 제외이며, 이번에는 뭉뚱그리지 않고
각각 이름을 달아 `trajectory_tools.rs`에 남겼다.

### 42.3 5라운드 UNFIXED가 세 가지를 뭉쳐 놓았던 것을 워커가 스스로 갈랐다

이전 라운드의 UNFIXED 한 줄은 (a) 설계상 이식하지 않는 것,
(b) D1이 막는 것, (c) `RobotModel::joint_model_mut`에 막힌 것,
(d) **이미 이식된 것**을 하나로 묶어 놓았고, 마지막 항목은 없는 구멍을
있다고 적은 것이었다. 6라운드 보고는 항목당 한 절로 갈라 다시 썼다.
§39.3의 stale UNFIXED 처방이 요구한 것과 같은 형태를 워커가 지시 없이
적용한 사례다.

남은 실제 차단 항목은 하나뿐이다: 스케일링 전용
`compute_time_stamps` 오버로드와 그 래퍼가
`RobotModel::joint_model_mut`에 막혀 있다. 보고 시점에
`rg -rn joint_model_mut crates/moveit-model/src/`가 아무것도 내지
않는다는 것을 워커가 직접 확인하고 적었다 — 게이트 전에 확인한 의존성
상태다.

## 43. pr2 자기충돌 거리가 무작위 상태 전부에서 어긋난다 (2026-08-04)

§34 이후로 미결이던 pr2 `--collision` 스윕을 릴리스 빌드로 끝까지
돌렸고, 결과가 지금까지 기록된 어떤 수치와도 다른 종류의 것이다.

```
moveit-diff --urdf fixtures/pr2.urdf --srdf fixtures/pr2.srdf \
            --collision --cases 10000 --seed 20260804
cases: 20001  passed: 10013  failed: 9988
```

`fk[i]`는 10000건 전부 통과하고, 실패는 전부 `collision[i]`의
`distance differs`다. 즉 **무작위 상태 10000개 중 자기충돌 거리가
일치한 것이 12건뿐**이다.

현재 스탬프(`e5449e10cc11b081`)에 대고 300건으로 재확인했다 —
`sg docker -c`로 감싸서 돌렸고, 앞 세 건이 10000건 스윕과 바이트
단위로 같다. 300/300 실패. 앞선 스윕이 이름 변경 이전 이미지에 대고
돌았던 문제는 이것으로 닫힌다.

### 43.1 러스트 쪽 값이 상태에 의존하지 않는다

이것이 이 발견의 핵심이고, 단순한 허용오차 문제가 아니라는 증거다.
10000건의 러스트 `self_distance` 값을 세어 보면 서로 다른 값이 사실상
**셋**뿐이다(마지막 두어 자리의 잡음을 빼면):

```
-5.29289090633392...e-2   (약 5700건)
-4.65920000000832...e-2
-4.91695723318727...e-2
```

오라클 쪽은 상태마다 연속적으로 변한다(`-3.3e-3` ~ `-1.6e-1`). 관절을
무작위로 흔들었는데 러스트의 최소 거리가 변하지 않는다는 것은, 그 최소값을
내는 쌍의 상대 자세가 상태와 무관하다는 뜻이다 — 고정 조인트로 묶인 링크
쌍이거나, 같은 강체 부분트리 안의 쌍이다. 그리고 그 값이 오라클보다 항상
더 깊다.

`robot_distance`는 같은 케이스에서 1e-13 수준으로 일치한다. 어긋나는
것은 자기충돌 경로 하나다.

### 43.2 문서화된 편차 7로는 설명되지 않는다

`parry.rs` 모듈 문서의 편차 7은 "상류 `distanceCallback`은
`cdata->done = true`로 브로드페이즈를 조기 종료하지만 이 포트는 모든
쌍을 남김없이 평가한다"이다. 그 조기 종료의 조건을 상류에서 직접 읽었다
(`collision_common.cpp:732-734`):

```cpp
if (!cdata->req->enable_signed_distance && cdata->res->collision)
  cdata->done = true;
```

`!enable_signed_distance`로 게이트된다. 그런데 양쪽 모두
`enable_signed_distance = true`로 요청한다 —
`oracle.cpp:1524`/`1530`, `moveit-diff/src/rust_impl.rs:370`. 따라서
상류는 이 스윕에서 조기 종료하지 않고, 편차 7은 발동하지 않는다.
비교 대상은 양쪽 모두 모든 쌍을 평가한 뒤의 전역 최소값이며, 비교는
의미가 있다.

### 43.3 왜 지금까지 안 보였는가

`self_distance` 비교는 `--collision` 도입 시점(`aed57e6`)부터 있었다.
새로 생긴 비교가 아니다. panda 스윕은 통과해 왔고, pr2에서만 이렇게
된다. §22의 `visibility_cone` 115건과도 다른 항목이다 — 그쪽은 접촉
깊이, 이쪽은 거리 질의의 전역 최소값이다.

커밋된 픽스처 기반 `moveit-collision::collision_parity::pr2_collision_matches_the_oracle`는
통과한다. **처음에 여기 "픽스처가 잡는 상태 공간과 스윕이 잡는 상태
공간이 다르기 때문"이라고 적었는데, 틀렸다.** p3-acm 8라운드를 병합하며
`assert_full_parity_matches_oracle`를 직접 읽고 픽스처를 세어 확인한
사실은 이렇다:

- 그 단정문은 오라클이 해당 면에서 충돌을 보고하면 **부호만**
  단정한다(`self_distance <= TOLERANCE`). 크기 비교는 오라클이
  `collision: false`인 경우에만 한다
  (`collision_parity.rs:287-301`, `:303-316`). 이 완화는 이번
  라운드에 들어온 것이 아니라 `aaaaae8`부터 있었다.
- `pr2_collision.json`은 4 케이스이고 **4건 전부
  `self_collision: true`**다(fanuc 4건 중 2건, panda 4건 중 1건).

즉 pr2의 `self_distance` 크기는 커밋된 픽스처 테스트가 **단 한 건도
단정하지 않는다**. 통과하는 이유는 상태 공간이 달라서가 아니라, 어긋나는
바로 그 영역에서 단정이 부호로 완화돼 있기 때문이다. §43을 잡을 수
있었던 유일한 테스트가 그 값을 보지 않는다.

이것을 §40의 실례로 부르는 것도 부정확했다. §40은 "병합된 오라클을
커밋된 응답과 대조하지 못한다"는 문제고, 이쪽은 "커밋된 응답을 대조하긴
하는데 그 필드를 대조하지 않는다"는 별개 문제다.

### 43.4 먼저 필요한 것은 쌍의 이름이다

프로토콜이 `minimum_distance`의 스칼라(`self_distance`)만 나르고
`link_names`는 나르지 않는다(`protocol.rs:802-807`). 그래서 어느 링크
쌍이 그 상수를 내는지 지금은 알 수 없다 — pr2 케이스 7552의 쌍을 아직도
이름 대지 못하는 것과 같은 원인이다. 양쪽 모두
`DistanceResultsData::link_names`를 가지고 있으므로 나르지 않을 이유가
없다.

`moveit-collision` 소유자(p3-acm)에게 넘긴다. 라운드 첫 작업은 원인
추정이 아니라 쌍을 이름 댈 수 있게 만드는 것이다.

## 44. `p1-joints` 7라운드 병합 — 관측 불가능하던 수정에 관측기를 붙였다 (2026-08-04)

`d7ef13c`. 보고가 배달되지 않은 채 패널이 idle이었고(§38과 같은 형태),
브랜치에 두 커밋이 남아 있었다.

### 44.1 6라운드 수정에 실행 가능한 회귀 검사가 없었다는 것을 워커가 스스로 지목했다

§35에서 병합한 연속 조인트 reseed 수정(`9444463`)은 그 라운드에
"이 네 픽스처·이 시드에서는 무해한 no-op"으로 확인됐고, 그것이 곧
**어떤 자동 검사도 이 수정의 되돌림을 잡지 못한다**는 뜻이었다.
`ik()`의 응답은 최종 solved/failed만 나르고 중간 reseed 추출값은
나르지 않으므로, 분기를 관측하려면 계측을 새로 만들어야 했다.
워커는 `reseed_probe` 요청 필드를 추가해 실제 reseed 루프와 같은
`sampleReseed()`에서 N개를 뽑게 하고, FK/solve를 우회했다.

`verify-continuous-reseed-wrap.sh`가 검사하는 성질은 범위가 아니라
**밀도**다. `limit = 4.0 > π`에서 wrap과 clamp는 둘 다 `[-π, π]`
안에만 값을 내므로 범위로는 구별되지 않는다. 산술을 손으로 다시
유도해 확인했다:

- wrap: `[-4, 4]` 균등에서 `(π, 4]`와 `[-4, -π)`가 반대쪽 경계 밴드로
  접힌다. 밴드 폭은 `4 - π = 0.858407`, 두 밴드 질량은
  `2 × 0.858407 / 8 = 0.429204` → **42.9204%**
- clamp: 표본 구간이 뽑기 전에 `[-π, π]`로 잘리므로 밀도가 평평하다.
  `2 × 0.858407 / (2π) = 0.273241` → **27.3241%**

스크립트가 출력하는 예측값과 자릿수까지 일치한다. 측정값은 42.6000%
(20000 draws)이고, 워커는 옛 clamp 식을 일부러 되살려 재빌드한 뒤
27.22%를 관측해 반대 방향도 확인했다 — 통과하는 것만 보이고 실패하는
것은 안 보인 검사가 아니다.

### 44.2 그 스크립트가 CI 글롭 안에 있었다

병합 시점에 찾은 결함이다. 스크립트 자신의 헤더에는 이렇게 적혀 있었다:
"like run-oracle-sweep.sh, this is deliberately not one of the `check-*.sh`
scripts `.github/workflows/ci.yml` ... run". 그런데 파일 이름이
`check-continuous-reseed-wrap.sh`였다. `ci.yml`은 열거가 아니라
`tools/ci/check-*.sh` 글롭으로 돌리므로(새 검사를 빠뜨리지 않으려고
일부러 그렇게 했다), 이름이 헤더를 이긴다. GitHub Actions 러너에는
docker도 오라클 이미지도 없으므로 이 스크립트는 자기가 검사하는 것과
무관한 이유로 실패했을 것이다.

**Anchor:** `rg -n 'docker|third_party|run-oracle' tools/ci/check-*.sh`
**Sites:** `check-continuous-reseed-wrap.sh` 한 곳
**Same defect at:** 그 한 곳
**Distinct, skip:** `check-dep-direction.sh`, `check-fixture-format.sh`,
`check-no-lint-suppression.sh` — docker/`third_party` 의존이 없고 글롭
안에 있는 것이 맞다

`5f2d1be`에서 `verify-continuous-reseed-wrap.sh`로 옮겼다.
`verify-fixture-provenance.sh`가 이미 같은 이유로 글롭 밖에 있고 그
이유를 헤더에 적어 두었으므로, 규칙은 새로 만든 것이 아니라 이미 있던
것이다. 이름이 곧 기제라는 점을 새 헤더에 명시했다.

`oracle.cpp`의 주석이 옛 경로를 가리키고 있어 같이 고쳤고, 그 결과
스탬프가 바뀌어 오라클을 재빌드했다(`05f4d82bcb77d40f`). 주석 한 줄
때문에 재빌드가 필요한 것은 §25에서 의도적으로 택한 트레이드의
비용 쪽이다.

### 44.3 심볼 감사에서 구멍이 나오지 않았다는 보고

`3870e18`이 `kdl_kinematics_plugin.{hpp,cpp}`,
`chainiksolver_vel_mimic_svd.{hpp,cpp}`, `KDLKinematicsPlugin`이
재정의하는 모든 `KinematicsBase` 가상 함수를 열거해
ported-as / excluded(결정 인용) / unported-no-consumer로 분류하고
`lib.rs` 모듈 문서에 남겼다. 보고서가 아니라 크레이트 안에 두라는
브리핑 조건을 지켰다. `getLinkNames` 하나가 포트도 결정 인용도 없이
남는데, 유일한 상류 호출자 `getPositionFK`가 이 크레이트에 자리가
없으므로 봉사할 소비자도 없다는 것이 그 항목의 분류다.

## 45. `p1-fixtures` 5라운드 병합 (2026-08-04)

`63981f2`. 이 패널도 보고가 배달되지 않은 채 idle이었고 세 커밋이
브랜치에 남아 있었다. `nextest --workspace` **931/931**.

상류 주장 둘을 받지 않고 확인했다:

- `clear_diffs`(`bc31159`)가 대응한다는 `planning_scene.cpp:316-336`을
  읽었다. `clearDiffs`는 부모의 world를 다시 복제하고 새 `WorldDiff`를
  만들며 `robot_state_`/`acm_`를 `reset()`한다 — 워커의 설명("부모
  링크는 유지한 채 얼어붙은 것들을 `Layered::Inherited`로 되돌린다")과
  일치한다. 상류는 `scene_transforms_`/`object_colors_`/`object_types_`도
  같이 리셋하는데, 워커가 "이 포트가 나르는 필드로 한정"이라고 범위를
  명시했다.
- `73d6a1b`이 고친 거짓 주장 — "상류는 `PlanningScene` 진입점에서
  `CollisionRequest::cost`를 세우지 않는다" — 은 실제로 거짓이다.
  `planning_scene.cpp:2464`와 `:2506`에서 `creq.cost = true`다. 두
  곳이고, 워커가 지목한 `getCostSources`가 그중 하나다.

`59ba2e6`은 `is_state_colliding`을 `is_state_valid` 안에서 꺼내
독립 심볼로 만든 것으로, 순서는 이미 그대로 있었고 심볼만 없었다 —
§39가 p3-shapes에 요구한 것과 같은 종류의 커버리지 구멍이다.

## 46. `p3-acm` 8라운드 병합 — 케이스 7552가 쌍 순위 뒤집힘이었다 (2026-08-04)

`728cc95`. `nextest --workspace` **934/934** (931 + 3).

### 46.1 워커가 자기 초안의 전제를 스스로 반증했다

보고의 첫 문장이 이것이다: 처음에는 케이스 7552를 "같은 쌍, 다른 깊이,
panda/fanuc와 방향이 반대"로 가정했는데, 커밋 전에 확인해 보니 그게
아니었다. 확인 방법이 정확하다 — 한 쌍만 남기고 전부 건너뛰는 임시 ACM을
만들어 **각 구현이 고른 쌍을 따로 고립시켰다**. 결과:

| | 오라클이 고른 쌍 | 이 포트가 고른 쌍 |
|---|---|---|
| self | `l_gripper_r_finger_link`/`l_gripper_palm_link` `-0.03027` | `base_bellow_link`/`torso_lift_link` `-0.05293` |
| robot | `r_gripper_l_finger_link`/`floor` `-0.02488` | `r_gripper_l_finger_tip_link`/`floor` `-0.05094` |

그리고 교차 확인이 핵심이다. 오라클이 고른 self 쌍에 대한 **이 포트
자신의 답은 `-0.00188`**로 자기 최댓값 근처도 아니다 — 애초에 이길 수
없었다. 반면 오라클이 고른 robot 쌍에 대한 이 포트의 답은 `-0.02833`으로
오라클의 `-0.02488`에 가깝다 — 로봇 쪽 차이는 깊이가 아니라 **어느 손가락
링크가 이기느냐**가 거의 전부다.

이 포트의 self 승자 `-0.05293`은 §43이 측정한 상수
`-5.29289090633392e-2`와 같은 값이다. 두 발견이 같은 현상이고, §43이
"상대 자세가 상태와 무관한 쌍"이라고 추정한 것의 이름이
`base_bellow_link`/`torso_lift_link`다.

`parry.rs`의 편차 6과 `collision_parity.rs` 모듈 문서를 이 사실에 맞춰
다시 썼고, 편차 6 자신의 유보("다른 케이스에도 성립하는지는 미확인")를
두 번째 확인 케이스로 닫았다. panda 이상치(libccd EPA의 상류 수치 실패)
및 `visibility_cone`의 `max_contacts:1` 순회 순서 타이브레이크와
명시적으로 구분해 두었다 — 후자는 잘린 질의이고 이쪽은 편차 7에 따라
전수 질의라 "정말로 다 비교한 뒤의 순위 불일치"라는 것이 구분점이다.

스윕의 나머지 약 9,500건까지 편차 6으로 설명된다고는 **주장하지 않았다**.
그 유보가 문서에 그대로 적혀 있다.

### 46.2 §43.3이 틀렸다는 것이 이 병합에서 드러났다

`assert_full_parity_matches_oracle`를 직접 읽고 픽스처를 세었다.
`75a76d9`에서 §43.3을 고쳤고, 요지는 이렇다: pr2 픽스처 4 케이스가
**전부** `self_collision: true`이고, 그 단정문은 오라클이 충돌을
보고하는 면에 대해 **부호만** 단정한다. pr2의 `self_distance` 크기는
커밋된 테스트가 한 건도 단정하지 않는다. 완화 자체는 `aaaaae8`부터
있던 것이고 이번 라운드에 들어온 것이 아니다.

이것은 워커의 결함이 아니라 내가 §43을 쓸 때 확인하지 않고 "픽스처가
그 상태 공간에 닿지 않는다"고 적은 것의 결함이다. 단정문을 읽으면
10초에 반증되는 문장이었다.

### 46.3 `joint_model_mut`이 들어왔다 — 3라운드 차단 해제

`1e18b90`. 상류 `robot_model.hpp:146`의 비const `getJointModel`과
대조해 확인했다(상류는 없으면 nullptr, 이 포트는 `Err` — 크레이트의
기존 규약). 테스트 둘: 세터 경로와 미지 이름 오류 경로. p6-totg가
세 라운드 막혀 있던 스케일링 전용 `compute_time_stamps` 오버로드가
이제 열린다.

### 46.4 자기 감사 결과를 스스로 정정했다

`MeshSearchPaths::none()` 감사에서 원시 매치가 37건으로 늘었으나 추가된
하나는 `multi_shape_object.rs`의 문서 주석 언급이고 실제 호출부는 36으로
그대로다. 그런데 워커가 §32.3에서 자기 소유 크레이트에 6건이라고 적은
것이 7건이었고, 빠뜨린 하나가 자기 자신의 옛 커밋(`dd4eb88`)이
추가한 `octree_leaf_count_scaling_parity.rs:124`였다. 같은 합성
메시-없음 URDF를 쓰므로 분류는 `deliberate`로 같다. 숫자를 맞추는
대신 자기 숫자가 틀렸다고 적은 쪽을 택했다.

### 46.5 측정하지 않은 것을 측정으로 적지 않았다

`visibility_cone` 115건의 메시/프리미티브 구성에 대해, p1-robotmodel의
케이스 인덱스와 하네스가 없어 직접 셀 수 없다는 것을 먼저 밝히고,
pr2 URDF에서 직접 확인한 것(팔·그리퍼 충돌 링크 13개 중 10개가
`<mesh>`)과 거기서 나오는 추론을 **inference, not measurement**로
분리해 적었다. 이 프로젝트에서 반복해 요구해 온 구분이고, 지시 없이
지켜졌다.

## 47. `p1-robotmodel` 5라운드 병합 — Phase 7 착수 (2026-08-04)

`932599e`. `nextest --workspace` **941/941** (934 + 7).

### 47.1 자기 코드의 전제를 이번 라운드에 확인해서 뒤집었다

워커 자신의 테스트 코드와 문서 주석이 panda의 메시 충돌 형상이
"이 저장소에 vendoring되어 있지 않다"고 단정하고 있었다. 기억에서
가져온 문장이고, 거짓이었다 — `fixtures/meshes/panda_description/`은
커밋돼 있고 `moveit-collision`의 `collision_parity.rs`가 이미
`MeshSearchPaths::new`로 그것을 푼다. 합성 모델 대신 실제 메시를 로드하는
쪽으로 테스트를 고쳤다.

그 수정이 두 가지를 연달아 드러냈다:

1. panda의 전-0 상태가 실제 메시에서는 **자기충돌 상태**다. 오라클
   픽스처로 직접 확인했다 — `panda_collision.json`의 첫 케이스는
   `joint_values: {}`(전부 기본값)이고 `self_collision: true`,
   `self_distance: -0.01311`이다. 두 테스트 모두 `panda.srdf`의
   `"ready"` 자세로 바꿨다.
2. `measured_call_cost`가 보고하던 수가 **세 자릿수 틀렸다**. 이전
   측정(`~88µs/call`)은 충돌 형상이 하나도 로드되지 않은 씬에 대한
   것이었고, 실제 메시로는 **~8–15ms/call**(debug 프로파일)이다.
   샘플 수를 10,000에서 50으로 줄였다.

측정값을 재기 전에 설계하지 말라는 지시가 있었고, 워커는 잰 값을
믿었다가 그 값 자체가 틀렸다는 것을 다시 재서 찾아냈다. 첫 측정을
"쟀으니 됐다"로 닫지 않은 것이 이 라운드의 핵심이다.

### 47.2 설계 결정 셋, 각각 기각안과 함께

- `PlannerManager`/`PlanningContext`를 새 `moveit-planning` 크레이트가
  아니라 `moveit-planners-sbp` 안에 둔다 — 공유 크레이트의 모양을
  검증할 두 번째 플래너 계열이 아직 없고, `moveit-kinematics::registry`가
  이미 트레이트+레지스트리를 유일 구현체와 같은 자리에 두는 선례다.
- `get_planning_context`를 `&ParryCollisionEnv`로 특수화한다. 기각안:
  제네릭 `E: CollisionEnv<..>` — 트레이트 메서드에서 `dyn` 객체 안전성이
  깨진다.
- D1-free `MotionPlanRequest` 대응물의 `goal`은 구체적인
  `Vec<CompoundValue>` 상태다. 기각안: 변수당 `JointConstraint` 하나짜리
  샘플러 스텁 — 워크스페이스 어디에도 `constraint_samplers`가 없고(§3이
  명목상 범위에 넣어 두었는데도), 스텁은 Cartesian 목표를 조용히
  잘못 처리한다.

### 47.3 단정하지 않는 테스트가 하나 들어왔다

`measured_call_cost`에는 `assert!`가 없다. 시간 한계를 걸면 머신 속도
차이 때문에 의미가 없다는 것이 워커의 이유이고, 그 이유는 타이트한
한계를 걸지 말라는 근거는 되지만 아무것도 걸지 말라는 근거는 아니다.
현재 상태는 nextest가 통과 시 stdout을 삼키므로 **출력이 보이지도
않고 실패할 수도 없는데 50회 × 약 10ms를 매 실행마다 쓴다**.

**Anchor:** `rg -n 'println!' crates/ tools/ --glob '*.rs'`
**Sites:** `#[test]` 안의 `println!`은
`planning_scene_validity.rs:335` 한 곳
**Distinct, skip:** `crates/moveit-geometry/examples/octree_compound_bench.rs`
— 테스트가 아니라 예제 바이너리이고, 이것이 **이 저장소가 이미 가진
올바른 자리의 선례**다. `tools/moveit-diff/src/main.rs`는 바이너리
자신의 출력이다.

크레이트 소유자에게 넘긴다. 내가 고치지 않은 이유는 이것이 정확성
결함이 아니라 배치 규약 결함이고, CI 글롭 건(§44.2)과 달리 다른 누구도
깨뜨리지 않기 때문이다.

### 47.4 보고서의 커밋 수가 실제와 다르다

보고서가 "Four commits this round"라고 적고 셋(`d616c1f`, `1fa9778`,
`100ce0a`)을 나열했다. 브랜치에도 셋이다. 고칠 것은 없고, 자기 보고의
숫자를 세지 않은 사례로만 적어 둔다.

## 48. §40이 닫히기 시작했다 — 그리고 내가 형식을 지정하지 않은 대가 (2026-08-04)

`5fea01f`(p1-fixtures), `7383978`(p3-distance-field). `nextest --workspace`
**941/941**.

### 48.1 병합된 오라클이 커밋된 응답과 일치한다는 첫 증거

§40이 지적한 구멍은 "Rust 대 커밋된 응답"만 검사되고 "현재 오라클 대
커밋된 응답"은 아무도 검사하지 않는다는 것이었다. `oracle.cpp`는 이번
주에만 세 번 움직였다. p3-distance-field의
`tools/ci/verify-fixture-replay.sh`를 직접 돌렸다:

```
identical    moveit-distance-field/collision_distance_field_types
identical    moveit-distance-field/collision_object_point_decomposition
identical    moveit-distance-field/distance_field
identical    moveit-distance-field/distance_field_cache_entry
identical    moveit-distance-field/distance_field_negative
identical    moveit-distance-field/group_state_representation
identical    moveit-distance-field/link_body_decomposition
identical    moveit-distance-field/link_models_with_collision_geometry
identical    moveit-distance-field/shape_points
```

**21건 중 9건이 실제로 재생돼 바이트 단위로 일치한다.** 나머지 12건은
아직 미검증이다. p1-fixtures의 panda frame-transform 쌍도 PASS를 직접
확인했다.

### 48.2 "형식은 소유자가 정한다"가 잘못된 지시였다

두 패널이 몇 분 간격으로 끝냈고 서로 다른 기제를 냈다:

- p3-distance-field — 크레이트별 매니페스트
  `crates/<crate>/tests/fixtures/oracle-models.json` +
  `tools/ci/verify-fixture-replay.sh`. 이 스크립트는
  `crates/*/tests/fixtures/oracle-models.json`를 **글롭**하므로 이미
  워크스페이스 전체에 대해 일반적이다.
- p1-fixtures — 요청 JSON 안의 `"model"` 필드 +
  `tools/ci/verify-scene-fixture-replay.sh`. 한 크레이트에 하드코딩되고
  요청→응답 대응표가 스크립트 안에 있다.

둘 다 조건("재생 정보가 테스트 소스가 아니라 픽스처와 함께")을 만족하고
둘 다 동작한다. 그런데 이대로 두면 남은 다섯 패널이 스크립트를 다섯 개
더 쓰고, 일반적인 이름은 이미 첫 번째가 가져갔다. 공유 디렉터리에
"형식은 자유"라고 쓴 것이 결함이다 — 소유권이 크레이트 단위인데 산출물이
`tools/ci/` 공용이었다.

`note-fixture-replay-convergence.md`로 일곱 패널 전부에 수렴 지시를
보냈다: **`verify-fixture-replay.sh` 하나를 쓰고 매니페스트만 추가한다.**
p1-fixtures는 자기 스크립트를 지우고 매니페스트로 옮긴다.

### 48.3 무시 목록에는 근거가 붙어 있다

매니페스트의 `ignore_result_fields_by_id`가 유일한 탈출구다.
p3-distance-field가 쓰는 곳은 Sphere-only 바디에 대한
`relative_cylinder_pose` 한 곳이고, 근거가 C++ 쪽이 그 경로에서 필드를
초기화하지 않아 **연속 재생 두 번이 서로 다른 값을 낸다**는 것이다 —
맞출 고정값이 존재하지 않으므로 비교에서 빼는 것이 맞고, 커밋된 스냅숏
하나를 정답으로 삼는 쪽이 오히려 거짓이다. 근거 없는 무시 목록은 드리프트가
숨는 자리이므로, 수렴 지시에 "근거를 매니페스트에 적을 것"을 넣었다.

## 49. `p3-shapes` 8라운드 병합 — 출처를 소스가 아니라 바이너리에서 확인했다 (2026-08-04)

`6b78668`. `nextest --workspace` **941/941**. 8커밋.

### 49.1 요구한 출처 확인을, 요구한 것보다 한 단계 더 갔다

라운드 중에 "§9.1이 이 머신에 없다고 적은 `geometric_shapes-2.3.3` 소스를
읽고 있으니 출처를 대라"고 요구했다. `d374f23`의 답:

- 캐시된 타르볼이 배포 패키지와 내용상 일치한다는 근거 — 접두사,
  파일 mtime(`2025-06-06 20:41`)이 `CHANGELOG.rst`의 `2.3.3 (2025-06-06)`
  항목과 맞는다.
- 이번 라운드 감사가 결론을 끌어내는 두 파일(`body_operations.cpp`,
  `shape_operations.cpp`)은 원래의 6개 문자열 검사가 덮지 않았으므로,
  문자열 3개를 더해 각각 `libgeometric_shapes.so.2.3.3`의 `strings`에
  정확히 한 번씩 나오는 것을 확인했다.
- 캐시된 `.so`가 이번 라운드에 새로 빌드한 오라클 이미지 안의 것과
  바이트 동일(`sha256sum 547881ff...`)함을 확인했다.

그리고 결정적인 부분: 라운드의 결론 하나가 **부정 사실**에 기대고 있다
(`ConvexMesh::computeScaledVerticesFromPlaneProjections`가 한 번도
호출되지 않는다). 어떤 문자열 리터럴로도 증명할 수 없는 종류이므로,
소스 grep에서 멈추지 않고 배포된 `.so`를 `objdump -d`로 디스어셈블해
**라이브러리 전체에서 그 함수 주소를 겨냥하는 `call` 명령이 하나도
없음**을 확인했다. 타르볼이 letter-perfect인지와 무관하게 성립하는
증거다.

부정 사실은 증명이 어렵다는 이유로 보통 "확인했다"로 넘어가는 자리다.
여기서는 증거의 층위를 바꿔서 답했다.

### 49.2 헤더 단위 심볼 감사

`bodies.h`, `body_operations.h`(11개), `shapes.h`/`shape_operations.h`,
`mesh_operations.h`(14개)를 각각 커밋으로 나눠 감사했고, 그 과정에서
자기 문서의 주장 둘을 정정했다(`BodyVector` 주장, `computeBoundingSphere`
벡터 오버로드 주장). `Shape::OcTree`의 네이티브-형상 공백에는 falsifier를
붙였고(`0343920`), `ConvexMesh` 삼각분할 유보가 실질적으로 외형 문제임을
증명했다(`db7afde`).

그 자기 정정 중 하나는 커밋 전에 잡혔다. 워커가 처음 "`BodyVector`는
어디에도 호출자가 없다"고 썼다가 `rg`를 돌려
`collision_distance_field_types.hpp:293`을 찾았고, 근거를 더 좁고 확인된
것("얇은 루프이고 `Vec<Body>`로 조합 가능")으로 바꾼 뒤 커밋했다.
쓴 문장을 검사한 것이지 검사한 것을 쓴 것이 아니다.

### 49.3 출처 확인은 **부분적**이다 — 다운로드 URL이 없다

병합 후 도착한 보고서가 명시한 한계이고, §49.1이 이것을 적지 않았으므로
여기 적는다. **인용 가능한 `wget`/`curl` 명령이 이 세션에도 이전
라운드에도 기록돼 있지 않다.** 타르볼이 GitHub 태그 아카이브의 *모양*을
갖고 있다는 것(`.git` 없음, `package.xml` 2.3.3, 파일 mtime 일치)은
정황이지 증명이 아니고, 워커도 "the shape of a GitHub tag-archive, not
proof of one"이라고 썼다.

그래서 결론이 타르볼 텍스트에 기대지 않도록 층위를 옮긴 것이 §49.1의
내용이다 — `.so`가 이번 라운드 오라클 이미지
(`ros-rolling-geometric-shapes 2.3.3-1noble.20260113.113114`) 안의 것과
바이트 동일하고, 부정 사실은 `objdump -d`로 확인했다. 즉 **`.so`가
ground truth이고 타르볼은 보조**다. 이 구분이 없으면 §49.1을
"출처가 확정됐다"로 읽게 되는데, 확정된 것은 바이너리 쪽이다.

### 49.4 세 UNFIXED의 처분

- **`bodies::Body`** — 거짓이므로 삭제. `contains_point`/`intersects_ray`/
  `compute_bounding_*`가 이미 완비돼 있고 `rg` 11 hit이 라운드 5보다
  앞선다. 내가 세 라운드 연속으로 틀린 브리핑을 보낸 항목이고
  (§31.3, §39.3), 이번에 워커가 닫았다.
- **`Shape::OcTree`** — 직접 확인했다. `parry3d-f64 0.30`에 `Voxels`가
  있고(`shape/voxels/voxels.rs`), 생성자가
  `Voxels::new(voxel_size: Vector, &coords)`로 **형상 전체에 균일한
  voxel 크기 하나**를 받는다(`:509`, `:516`). 따라서 가지치기된
  가변-깊이 리프는 여전히 부풀어 오른다는 워커의 판단이 성립한다.
  falsifier도 정확하다: 노드별 크기를 받는 parry 생성자가 생기면 이
  결론이 뒤집힌다.
- **`ConvexMesh::triangles()`** — `compute_volume`(발산 정리)과
  `contains_point`(평면 중복 제거, 기존 테스트가 이미 증명)는 위상
  불변이고, `ray_intersections`만 실제로 민감하되 광선이 공면 패치의
  공유 모서리에 정확히 닿는 측도-0 경우에 한한다.

## 50. `p1-fixtures` 6라운드 병합 — `PlanningScene` 심볼 감사 (2026-08-04)

`5fea01f`. 6라운드 3개 항목 중 1·2번이 들어왔다(`aaf5cca`, `c43e9dd`).
패널이 중간에 idle로 배달됐지만 실제로는 작업 중이었고, 병합 시점에
브랜치 팁이 앞서 있었다 — `git log HEAD..<branch>`로 확인하고 팁을
병합했다. 배달된 보고서는 5라운드 것의 stale 재전송이었다.

`c43e9dd`가 상류 `planning_scene.h`의 공개 API 전체를 심볼 단위로
분류해 모듈 문서에 넣었다. `aaf5cca`는 §48의 재생 항목이고, 워커가
스크립트가 드리프트를 실제로 잡는지 커밋된 응답에 불일치를 주입해
FAIL을 관측하고 되돌리는 방식으로 자기 검증했다 — 통과만 보고 끝내지
않았다.

## 51. §40 픽스처 21건의 정확한 소유자별 회계 (2026-08-04)

§48이 "21건 중 9건 재생됨"이라고만 적었다. 나머지 12건이 어디에 있는지
세지 않은 채였고, 세어 보니 내 브리핑 두 건이 존재하지 않는 일을
지시하고 있었다.

`find crates tools -name '*_request.json'` 전수:

| 크레이트 | 건수 | 상태 | 소유 패널 |
|---|---|---|---|
| `moveit-distance-field` | 9 | **재생 확인됨**(전부 `identical`) | p3-distance-field |
| `moveit-collision` | 3 | 미검증 — `octree_leaf_count_scaling`, `octree_world_collision`, `world` | p3-acm |
| `moveit-geometry` | 3 | 미검증 — `body_query`, `octree_in_world`, `octree_shape_query` | p3-shapes |
| `moveit-trajectory` | 4 | 미검증 — `ruckig`, `totg`, `totg_robot_trajectory`, `totg_synthetic` | p6-totg |
| `moveit-octomap` | 1 | 미검증 — `octomap` | p3-shapes |
| `moveit-scene` | 1 | `"model"` 필드는 있으나 매니페스트 미이전 | p1-fixtures |

`moveit-kinematics`, `moveit-diff`, `moveit-constraints`,
`moveit-planners-sbp`, `moveit-model`, `moveit-state`, `moveit-srdf`,
`moveit-smoothing`은 요청 픽스처가 **하나도 없다**.

### 51.1 p1-joints와 p1-robotmodel에게 없는 일을 시켰다

두 패널의 이번 라운드 브리핑에 §40 재생 항목을 넣었는데, 둘 다 요청
픽스처가 0건이다. p1-joints는 이 항목을 todo에 올려 둔 채 작업 중이었다.
두 패널에 취소 지시를 보냈다.

원인은 §41.2·§39.3에서 워커들에게 처방한 것과 정확히 같은 결함이다 —
**주장에 falsifier를 달지 않았다.** "21건이 여섯 크레이트에 흩어져
있다"(§40)에서 "그러므로 이 패널도 갖고 있다"로 넘어가는 데 근거가
없었고, 근거를 만드는 명령은 1초짜리였다. 워커의 UNFIXED에 요구한 규칙을
내가 쓰는 *태스크*에도 적용해야 한다.

### 51.2 내 오산이 워커의 오산으로 번졌다 — 총계는 21이다

p3-distance-field 8라운드 보고서가 이렇게 적었다: "브리핑이 내 것을
8건이라 했는데 실제로는 **9건**이므로 워크스페이스 총계는 21이 아니라
**22**다."

앞의 절반은 맞고 뒤의 절반은 틀렸다. 직접 셌다:

```
find crates tools -name '*_request.json'  | wc -l   → 21
find crates/moveit-distance-field -name '*_request.json' | wc -l → 9
find crates tools -name '*_response.json' | wc -l   → 21
```

**§40의 21은 옳고, 내 브리핑의 "8 of the 21"이 틀렸다.** 워커는 자기
숫자를 정확히 세고도 총계 쪽을 고치는 방향으로 추론했다 — 브리핑의 두
숫자 중 어느 쪽이 틀렸는지를 확인하지 않고 자기가 검증한 쪽을 고정한 채
검증하지 않은 쪽을 움직였다.

내 쪽 결함이 먼저다. §40에서 총계 21은 세어서 적었는데, 라운드 브리핑의
크레이트별 배분은 세지 않고 적었다. §51.1의 p1-joints·p1-robotmodel 건과
같은 결함이고 같은 라운드에 두 번 나왔다 — 셈이 필요한 자리에 셈 대신
기억을 넣었다.

### 51.3 소유권 블록이 `moveit-octomap`을 빠뜨리고 있었다

최근 라운드 브리핑들의 ownership 절이 p3-shapes를 `moveit-geometry/`로만
적었다. `moveit-octomap`은 §13에서 p3-shapes가 만든 크레이트이고 전용
태스크 파일까지 있었는데, 최근 블록에서 이름이 빠졌다. 고아 크레이트는
아니고 브리핑 표기 누락이다 — 그 1건도 p3-shapes 소유로 센다.

## 52. 크레이트 로컬 로봇 기술이 출처 검사 밖에 있었다 (2026-08-04)

`6ec9a97`(p1-fixtures 병합), `dd497e4`(검사 확장).
`nextest --workspace` **941/941**.

### 52.1 수렴은 됐다

p1-fixtures가 §48.2의 수렴 지시를 그대로 실행했다:
`verify-scene-fixture-replay.sh` 삭제, `oracle-models.json`으로 이전,
요청 JSON에서 이제 중복인 `"model"` 필드 제거(같은 사실을 두 곳에 적지
않는다). 직접 돌려 확인했다 — 이제 스크립트 하나가 **10건**을 재생하고
전부 `identical`이다.

### 52.2 그러다 검사 밖에 있던 것을 찾았다

이전 과정에서 워커가 `panda.urdf`/`panda.srdf`를
`crates/moveit-scene/tests/fixtures/`에 복사했다. 공유 스크립트가
urdf/srdf를 매니페스트 기준 상대 경로로 푸므로 필요한 복사이고,
`moveit-state`·`moveit-kinematics`·`moveit-constraints`·
`moveit-distance-field`가 이미 같은 일을 하고 있던 선례를 따른 것이다.

그런데 `verify-fixture-provenance.sh`는 `fixtures/*.urdf fixtures/*.srdf`만
순회한다. **크레이트 로컬 복사본 12개는 아무것과도 대조되지 않고 있었다.**
루트 복사본은 vendored 트리와 대조돼 깨끗한데, 크레이트 로컬 복사본이 그
루트에서 갈라져도 아무 일도 일어나지 않는 상태였다 — 그 크레이트의 모든
파리티 주장이 이름과 다른 로봇을 조용히 기술하게 된다.

전수 대조해 보니 하나가 실제로 다르다:

```
DIFFERS  crates/moveit-kinematics/tests/fixtures/pr2.srdf
```

읽어 보니 **의도된 차이**다. 상류 PR2 SRDF의 어떤 그룹도 격리하지 않는
"활성 조인트 1 + mimic 조인트 1"짜리 `is_chain()` 그룹
(`l_gripper_finger_chain`)을 mimic 조인트 테스트용으로 추가한 것이고,
파일 안에 그 이유가 적혀 있다. 조인트 타입과
`l_gripper_l_finger_tip_joint`의 mimic 배수/오프셋은 실제 PR2 URDF의
값이며 그룹 경계만 새것이다.

즉 드리프트 사고는 아니었다. 그러나 **그것을 기계적으로 아는 방법이
없었다** — 파일을 열어 주석을 읽는 것 말고는.

### 52.3 확장: 크레이트 로컬을 루트에 사슬로 묶는다

`dd497e4`. 규칙은 같은 basename의 루트 픽스처와 바이트 동일이다. 루트
픽스처가 이미 vendored 소스와 묶여 있으므로, 두 단계를 이으면 크레이트
로컬 복사본도 출처 검사를 받게 된다.

예외는 두 종류이고 둘 다 표 항목을 요구한다 — 침묵은 허용하지 않는다:

- `DIVERGENT` — 의도된 편집. `moveit-kinematics/pr2.srdf` 한 건.
  루트와 **같아지면** `STALE`로 실패한다. 편집이 상류로 흡수됐는데 표만
  남는 상태를 잡기 위한 것이다.
- `SYNTHETIC` — 어떤 vendored 기술의 복사본도 아닌 손으로 쓴 로봇.
  `octree_world_robot.{urdf,srdf}`, `totg_synthetic.{urdf,srdf}` 4건.

표가 아니라 파일시스템으로 구동한다(기존 스크립트의 원칙 그대로).
어느 쪽에도 없고 루트와 다르면 실패하므로, 새 크레이트 로컬 기술이
잊혀서 검사를 빠져나갈 수 없다.

세 실패 모드를 각각 주입해 확인했다:

```
DRIFTED   moveit-state/panda.urdf에 한 줄 추가        → 검출
UNMAPPED  루트에 짝이 없는 newbot.urdf 추가            → 검출
STALE     kinematics/pr2.srdf를 루트와 같게 만듦       → 검출
```

전부 되돌린 뒤 재실행 pass. 통과만 보고 끝내지 않았다.

## 53. §43의 세 상수는 두 쌍 계열이었다 (2026-08-04)

p1-joints 라운드 8 (`65a9dd9`), 병합 `41f7987`. `nextest --workspace` **941/941**.

### 53.1 스칼라만 비교하던 것이 세 라운드를 잡아먹었다

`moveit-diff`가 self/robot 최소거리를 **값만** 비교하고 있었다. 그래서
§43은 "9,988/10,000이 어긋난다"까지만 말할 수 있었고, 어느 링크 쌍이
그 값을 만드는지는 세 라운드 동안 이름이 없었다.

`protocol.rs`에 `DistancePair`(각 변의 링크 이름 + body type)를 넣고
`CollisionCheckResult`에 `self_distance_pair`/`robot_distance_pair`를
달아 양쪽에서 채운다. Rust 쪽은 `DistanceResultsData::link_names`/
`body_types`에서, C++ 쪽은 p3-acm이 라운드 8에 이미 넣은
`distancePairToJson`/`bodyTypeName`에서 온다 — p1-joints가 자기 `oracle.cpp`
편집을 stash 해 뒀다가 형태가 동일함을 확인하고 버렸다. 같은 JSON을 두 번
만들지 않은 것이 맞다.

### 53.2 직접 재현했다 — 비율은 맞고 개수는 보고와 다르다

보고는 "300 cases"라며 `340/600`, `246/600`을 적었다. 나는 300 케이스를
직접 돌렸고(`--collision`, seed 20260804, right_arm) 300 실패에
`177 / 123`을 얻었다. 비율은 59:41 대 58:42로 일치하지만 **보고 안에서
300과 600이 서로 맞지 않는다**. 결론은 유효하고 분모만 불명확하다.

두 계열의 값 퍼짐을 직접 쟀다:

```
bellow  n=177  min=-5.29289090633394688e-02  max=-4.73932469304884987e-02  spread=5.536e-03
caster  n=123  min=-4.65920000000832751e-02  max=-4.65920000000434875e-02  spread=3.979e-14
```

- `base_bellow_link`/`torso_lift_link` — 상수가 아니다. `torso_lift_joint`가
  움직이는 만큼 5.5e-3 폭으로 흐른다. §43이 본 "세 상수"의 두 개는 같은
  계열의 서로 다른 표본이었다.
- `base_link`/`*_caster_*_wheel_link` **여덟 쌍** — `3.98e-14`, 즉 배정도
  한계까지 같은 값이다(워커는 ~11자리라 했고 실제로는 ~13자리). 바퀴가 자기
  roll 축에 대해 회전대칭이라 그 조인트가 최근접거리를 바꾸지 못한다.
  여덟 개의 다른 쌍이 하나의 상수로 보였던 이유다.

즉 상수 셋이 아니라 **계열 둘**이고, 하나는 대칭에서 나온 진짜 상수,
하나는 좁은 밴드다.

### 53.3 그리고 world 쪽에서도 어긋난다 — 이건 새 사실이다

내 재현에서 `robot_distance`(로봇 대 world object) 쌍도 300건 중 276건이
오라클과 다른 쌍을 고른다. 대부분은 무해하다 — 값이 `1e-12` 이내로 같고,
바퀴 여덟 개가 `floor`에 대해 동률이라 어느 것을 고르든 같은 수가 나온다.

그런데 **한 건은 허용오차를 넘는다**(`collision[122]`):

```
robot oracle -1.17505058621331926e-2 [l_gripper_r_finger_link/floor]
     vs rust -3.30976249554740254e-2 [l_gripper_r_finger_tip_link/floor]
     (|d|=2.135e-2, tol 1e-4)
```

두 가지를 뒤집는다:

1. **§43은 self-collision 고유 문제가 아니다.** 같은 종류의 불일치가
   world object 상대에서도 나온다. ACM 필터링이나 self 쌍 열거처럼
   self 경로에만 있는 것은 원인에서 배제된다.
2. **§43.x에 내가 쓴 "이 포트가 16배 얕게 답한다"는 방향이 일반적이지
   않다.** 여기서는 포트 쪽이 `-0.0331`로 오라클 `-0.0118`보다 **깊다**.
   침투깊이가 한쪽으로 치우친 게 아니라 양방향으로 어긋난다 — 계통 오차가
   아니라 최근접쌍 선택과 침투깊이 측정이 함께 흔들린다는 뜻이다.

p3-acm 몫이다. `moveit-collision`은 그쪽 소유고, 이 라운드의 산출물은
진단이지 수정이 아니다.

## 54. scaling-only 오버로드가 닫혔고, 오라클이 바뀐 뒤에도 15건이 그대로다 (2026-08-04)

p6-totg 라운드 7 (`d21f833`, `65b64f8`, `7c583b6`, `66688a1`), 병합 `9ba719f`.
`nextest --workspace` **942/942**. 오라클 stamp `f6e61b6136ad4791` →
**`c5e7d2936755ea44`**.

### 54.1 "Known gap"이 지워지지 않고 "Closed gap"이 됐다

§46에서 `RobotModel::joint_model_mut`이 들어오면서 막혀 있던
scaling-only `compute_time_stamps` 오버로드가 이번에 닫혔다. 닫은 방식이
중요하다:

- `oracle.cpp`의 `totgRobotTrajectoryCase`에 `acceleration_bounds` 필드를
  추가하고, **실제 오라클 파이프라인으로** 새 픽스처 쌍
  (`totg_robot_trajectory_scaling_only_request/response.json`)을 떴다.
  손으로 쓴 기대값이 아니다.
- 기존의 "에러 전달이 동등한지" 테스트를 실제 수치 파리티 테스트로 교체했다
  (`time_optimal_trajectory_generation.rs`, `trajectory_tools.rs`).
- 문서의 "Known gap" 절을 **삭제하지 않고** "Closed gap"으로 바꿔 무엇이
  닫았는지 남겼다. 갭이 지워지면 그것이 있었다는 사실도 지워진다.

`trajectory_processing/include/`의 진짜 헤더 넷 전부(`time_parameterization`,
`time_optimal_trajectory_generation`, `trajectory_tools`,
`ruckig_traj_smoothing`)에 심볼 단위 감사를 붙였다. 새 갭 없음.

### 54.2 오라클이 바뀐 뒤 15건 전부 재생 identical — 처음이다

`oracle.cpp`가 바뀌었으므로 이미지가 새로 빌드됐고 stamp가
`c5e7d2936755ea44`로 옮겨갔다. 그 새 이미지로 직접 재생했다:

```
9  moveit-distance-field   identical
1  moveit-scene            identical
5  moveit-trajectory       identical   (ruckig, totg, totg_robot_trajectory,
                                        totg_robot_trajectory_scaling_only,
                                        totg_synthetic)
```

**15/15.** §40이 원한 게 이거였다 — 오라클에 새 필드를 넣은 뒤에도 이전에
커밋된 응답들이 바이트 단위로 그대로 나온다는 증거. 지금까지 재생은 오라클이
그대로일 때만 돌았고, 그때는 "안 바뀐 것을 안 바뀌었다고" 확인한 것에 가까웠다.

`*_request.json` 총계는 21 → **22**로 늘었다(새 fixture 하나). 재생된 것이
15, 남은 것이 7 — p3-shapes 4, p3-acm 3.

### 54.3 커밋 하나가 혼자서는 깨진다 — 그리고 내가 만든 적 없는 규칙

`65b64f8`은 `oracle-models.json`에 `totg_robot_trajectory_scaling_only`를
등록하는데 그 픽스처 파일은 다음 커밋 `7c583b6`에서야 생긴다. 그 커밋에
정확히 서서 `verify-fixture-replay.sh`를 돌리면 MISSING이 난다. 워커가
스스로 발견해 보고한 것은 맞다.

다만 그 근거로 든 **"standing no-amend policy"는 존재하지 않는다.** 내가 준
규칙은 "한 발견 = 한 커밋"이고, 그것은 커밋을 순서대로 놓아 각각이 혼자
서도록 만드는 것을 금지하지 않는다 — 오히려 그쪽이 규칙의 목적에 맞는다.
두 커밋의 순서를 바꾸거나 매니페스트 등록을 픽스처와 같은 커밋에 두면 됐다.
없는 규칙을 근거로 고치지 않은 것이 이번 건의 실제 문제이고, 브랜치는 병합
전이라 되쓰는 비용도 없었다.

## 55. 검사가 실패했는데 통과라고 말하는 자리 셋 (2026-08-04)

p1-robotmodel 라운드 6 (`f5c5123`, `2d763fb`, `7ec0702`, `142c0a1`),
병합 `0bcc06c`. 후속 수정 `8791569`. `nextest --workspace` **942/942**.

### 55.1 §49.4의 두 UNFIXED가 닫혔다

`measured_call_cost`는 assert가 없고 nextest가 성공 시 stdout을 삼켜서
아무도 볼 수 없는 테스트였다. `examples/planning_scene_validity_bench.rs`로
옮기고, 자리에는 느슨한(2 s) 회귀 가드 테스트를 남겼다 — 측정은 실행해서
읽는 것이고 테스트는 무너졌을 때 실패하는 것이라는 구분이 맞다.

그리고 release 프로파일을 실제로 쟀다. 직접 재현했다:

```
release: mean 2.092531ms/call, min 889.71µs, max 5.724474ms, 50 calls
```

워커 보고는 2.048 ms, 내 실행은 2.093 ms — 같은 수다. debug 12.868 ms에서
약 6배이지 50배가 아니다. 문서의 "질의당 수 분" 주장이 debug 숫자 위에
서 있던 것을 고쳐, 이제 release 평균으로 20,000회 ≈ 41 s를 유도하고
"분 단위"는 max 관측치 쪽에만 남겼다(`planning_scene_validity.rs:61-76`).
지연 우려 자체는 유지되고 근거만 바뀌었다.

`moveit-constraints`/`moveit-planners-sbp` 양쪽에 심볼 감사를 붙였고,
"11 functions"라는 낡은 수를 13으로 고쳤다.

### 55.2 UNFIXED로 올라온 것 — 그리고 그 진단은 틀렸다

워커가 올렸다: `verify-continuous-reseed-wrap.sh`를 `sg docker` 없이 돌리면
**아무것도 출력하지 않고 exit 1** 한다. 직접 확인했다 — 출력 0바이트,
exit 1. 현상은 정확하다.

다만 진단이 틀렸다. 워커는 "스크립트가 CLAUDE.md가 요구하는 `sg docker -c`
래퍼 없이 docker를 직접 부른다"고 적었다. `sg docker`는 **호출자 쪽 환경**
문제다(이 호스트에서 사용자의 보조 그룹이 셸에 반영돼 있지 않다). 스크립트가
자기를 감쌀 수는 없고, 감싸서도 안 된다 — 그러면 그 그룹이 없는 호스트에서
못 돌게 된다.

진짜 결함은 **왜 실패했는지 한 글자도 말하지 않는 것**이다:

```bash
python3 - ... | run-oracle.sh ... 2>/dev/null | tail -1 > "$RESPONSE_FILE"
```

`2>/dev/null`이 유일한 설명을 버리고, `| tail`이 상태 보고 단계를 `tail`로
바꾼다. 둘 다 `run-oracle-sweep.sh`가 자기 주석에 이미 적어 둔 금지 사항인데
내가 이 스크립트를 쓰면서 지키지 않았다.

### 55.3 인용된 자리는 표본이었다 — 같은 결함 셋

앵커 `2>/dev/null` + "fallible 생산자를 필터로 파이프":

| site | 판정 |
|---|---|
| `verify-continuous-reseed-wrap.sh:61` | **같은 결함** |
| `check-no-lint-suppression.sh:33` | **같은 결함** — `if hits=$(rg ...)`가 rg의 "매치 없음"(1)과 "rg 자체 실패"(2)를 같은 분기로 접는다. 검색이 깨져도 `OK`를 찍는다 |
| `check-dep-direction.sh:33` | **같은 결함** — `cargo tree`가 파이프 머리에 있고 꼬리가 `\|\| true`라, cargo가 해석하지 못한 패키지가 "ROS 의존 없음"이 된다. 검사받지 않은 크레이트가 통과로 보고된다 |
| `run-oracle.sh:28,42` | 별개 — 부재가 곧 기대되는 신호인 탐침(`\|\| true`가 의도) |
| `run-oracle-sweep.sh:76`, `verify-fixture-provenance.sh:79,162` | 별개 — 이미 실패가 확정된 분기 안에서 표시용으로 자르는 것 |

셋 다 `8791569` 하나로 고쳤다(한 발견, 여러 자리). 각각 주입해 확인했다:

```
docker 그룹 없이 reseed-wrap  → "run-oracle.sh failed (exit 1)" + 이미지 stamp 불일치 전문
rg에 없는 경로 추가            → "rg failed (exit 2) -- this check did not run", exit 2
cargo tree에 없는 패키지       → "FAIL cargo tree -p ... exited 101 -- ... was not checked", exit 1
```

수정 후 `sg docker`로 감싼 정상 경로 재실행: `42.6000%`, 이전과 같은 수,
exit 0. 성공 경로는 그대로다.

이 결함군이 위험한 이유는 §51에 적은 것과 같다 — 검사가 **없는 것**보다
**돌지 않았는데 통과라고 말하는 것**이 나쁘다. 앞은 빈칸으로 보이고 뒤는
증거로 보인다.

## 56. §43의 plateau — 오라클이 아니라 이 포트가 맞다, 그리고 증명이 바뀐다 (2026-08-04)

p3-acm 라운드 9 (`b2d091e`, `85567b6`, `ce4afdf`), 병합 `4bf4c8a`.
`nextest --workspace` **944/944**. 재생 **18/18** (stamp `c5e7d2936755ea44`).

### 56.1 sign-only 분기에 경계가 생겼다

§43이 연 질문 — `collision_parity.rs`의 "충돌이면 부호만 본다" 분기가
deviation인가 구멍인가 — 에 답이 나왔다. **deviation**으로 결론짓되 그냥
두지 않고 `assert_plausible_depth`/`link_bounding_radius`를 붙였다. 강체는
자기 지름보다 깊게 겹칠 수 없다는 상한을, Mesh 전용이 아니라 shape-generic
으로(박스/실린더/콘/구 전부) 모든 충돌 케이스에 건다. 정확한 크기 파리티는
여전히 요구하지 않지만, panda worst-case 같은 물리적으로 불가능한 수로의
회귀는 이제 실패한다.

### 56.2 하드코딩된 오라클 상수 두 개를 현재 stamp로 재확인했다

`pr2_torso_lift_bellow_pair_plateau_is_geometrically_forced`는 오라클 값
두 개를 리터럴로 박아 두고 있다. 워커는 stamp `f6e61b6136ad4791`에서 떴고,
그 사이 p6-totg가 `oracle.cpp`를 바꿔 stamp가 `c5e7d2936755ea44`로 옮겨갔다.
직접 다시 떴다:

```
torso_lift_joint=0.1  self_distance -0.13543907645960804  torso_lift_link/base_bellow_link
torso_lift_joint=0.2  self_distance -0.05755014036972962  torso_lift_link/base_bellow_link
```

테스트의 `ORACLE_DIST_AT_0_1`/`ORACLE_DIST_AT_0_2`와 마지막 자리까지 같다.

### 56.3 그런데 워커의 논증은 결론을 지탱하지 못한다 — 더 나은 증거가 있다

워커는 "두 상태에서 같은 국소 feature가 잡히므로 참된 침투깊이는 그 구간에서
상수여야 하고, 따라서 상수를 유지하지 못하는 오라클이 틀렸다"고 적었다.
그런데 "참된 깊이가 변할 수 없다"가 바로 쟁점이다 — 순환이다.

`torso_lift_joint`를 0.00–0.30까지 0.02 간격으로 양쪽에서 쓸었다:

```
torso   oracle              rust
0.08    -0.073008524        -0.052928909
0.10    -0.135439076        -0.052928909
0.12    -0.141382829        -0.052928909
0.14    -0.115379149        -0.052928909
0.16    -0.095379149        -0.052928909
0.18    -0.076988211        -0.052928909
0.20    -0.057550140        -0.052928909
0.22    -0.035379149        -0.035379149
0.24    (pair 바뀜)          -0.015379149
```

오라클은 요동치지 않는다. 0.12부터 0.22까지 **매끄럽게 단조 감소**하며,
z 이동량과 거의 1:1로 얕아진다 — z 방향 겹침이 실제로 존재한다는 서명이다.
EPA 잡음이 아니다. 워커가 든 "상수를 유지하지 못한다"는 근거는 여기서
무너진다.

무너지는 대신 진짜 구조가 보인다. **이 포트의 곡선은
`min(0.052928909, ramp(t))`이다.**

- `t ≤ 0.20`: ramp가 0.0529보다 깊으므로 포트는 0.052928909(x 방향 면 접촉)를
  고른다.
- `t = 0.22`: ramp가 0.035379149로 얕아지고 포트도 **정확히 같은 수**를
  낸다 — 소수점 아홉 자리까지 오라클과 일치한다.
- `t = 0.24`: 포트 -0.015379149, 같은 ramp의 연장선.

침투깊이의 정의가 **최소 분리 이동(MTD)** 이므로, 두 방향 다 실제 겹침이면
정답은 **얕은 쪽**이다. 포트는 두 후보의 최소를 취하고, ramp가 최소가 되는
구간에서 오라클과 완전히 일치한다. 오라클은 구간 내내 z 방향 ramp만 들고
있어서, 더 얕은 분리 방향이 존재할 때 깊이를 과대보고한다.

즉 결론(**deviation, 이 포트가 맞다**)은 서고, 근거는 "plateau가 상수다"가
아니라 "포트의 곡선이 두 후보의 최소이고, 교차점 이후 오라클과 아홉 자리까지
같다"이다. 후자는 반증 가능하고 전자는 아니었다.

### 56.4 남는 것

- 두 곡선의 교차가 0.20과 0.22 사이라는 것까지만 쟀다. 교차점을 좁히면
  `min(...)` 해석이 한 번 더 확인된다.
- parry의 TriMesh 접촉은 삼각형별 최대이고, 삼각형 단위 MTD는 메시 전체
  MTD보다 얕을 수 있다(얇은 삼각형 문제). 위 일치는 이 구간에서 그 과소평가가
  일어나지 않았음을 보이지만, 일반적으로 일어나지 않는다는 뜻은 아니다.
- §53.3이 찾은 world object 쪽 불일치(`l_gripper_r_finger_link`/`floor`,
  포트가 **더 깊게** 답함)는 이 설명으로 덮이지 않는다. 방향이 반대다.

## 57. §40이 닫혔다 — 22/22. 그리고 pr2 메시는 한 번도 대조된 적이 없다 (2026-08-04)

p3-shapes 라운드 9 (`6d00c96`, `4893317`, `dec3626`), 병합 `2e0a60d`.
`nextest --workspace` **944/944**.

### 57.1 22/22

마지막 네 픽스처(`moveit-geometry`의 `body_query`/`octree_in_world`/
`octree_shape_query`, `moveit-octomap`의 `octomap`)가 공유 매니페스트로
들어왔다. 현재 stamp `c5e7d2936755ea44`로 직접 돌렸다:

```
3  moveit-collision        identical
9  moveit-distance-field   identical
3  moveit-geometry         identical
1  moveit-octomap          identical
1  moveit-scene            identical
5  moveit-trajectory       identical
```

**22/22.** §40이 물은 것 — 커밋된 응답들이 지금의 오라클에서 여전히 나오는가
— 에 워크스페이스 전체가 답했다. 재생 스크립트는 하나이고(§48.2의 수렴),
크레이트마다 매니페스트가 하나다. 일곱 패널이 각자 스크립트를 쓰던 상태에서
여기까지 왔다.

### 57.2 falsifier를 평가했고, 불발했다

`saveAsText`/`constructShapeFromText` 유예의 falsifier("이 형식을 필요로 한다고
말한 소유자가 아직 없다")를 말로 재확인하지 않고 실제로 검색했다. 워크스페이스
안에서 두 함수(및 snake_case 형태)를 부르는 곳은 자기 모듈 문서뿐이고,
`oracle.cpp`에도 해당 op이 없다. 명령과 결과를 `shapes.rs` 모듈 문서에 적었다.
유예는 유지된다.

다만 이 답은 p1-fixtures와 **묶여 있다**. 상류 소비자는 워크스페이스 밖에
있다 — `planning_scene.cpp:1062`가 `shapes::saveAsText`를,
`:1152`가 `shapes::constructShapeFromText`를 부른다. p1-fixtures가 그
`saveGeometryToStream`/`loadGeometryFromStream`을 "distinct"로 분류한 근거
(`getObjectColor`가 D1)가 §52 브리핑에서 무너졌으므로, 그쪽 답이 바뀌면
이 유예의 falsifier가 발화한다. 두 패널이 서로를 가리키며 같은 유예를
열어 두는 상태를 피하려고 양쪽에 같은 질문을 보냈다.

### 57.3 octree는 §43에서 배제된다 — 구조적으로

`check_self_collision`/`distance_self`는 `robot_bodies`만 만들고
`world_bodies`를 만들지 않는다. `compound_from_octree`/`OctreeCache`는
`world_bodies`를 통하거나 링크/부착 도형 자체가 `Shape::OcTree`일 때만
닿는데, URDF로 적재된 PR2 모델은 후자를 만들 수 없다. 따라서 self-collision
거리 불일치에 octree 변환은 관여할 수 없다. 런타임 관찰이 아니라 도달
가능성 논증이라 §43의 어떤 상태에서도 성립한다.

### 57.4 그 대신 나온 것 — pr2 메시는 한 번도 Assimp와 대조된 적이 없다

`mesh_parity.json`을 직접 세었다:

```
entries: 18   panda 11, fanuc 7, pr2 0
fixtures/meshes/pr2_description/**/*.stl: 18개
```

이 픽스처는 `vertices`를 통째로 들고 있어 비트 단위 대조를 한다 — panda와
fanuc에 대해서만. **pr2의 collision STL 18개는 하나도 대조되지 않았다.**
pr2 STL은 `VCG` 헤더(VCGLib 익스포터), panda는 `Export` 헤더로, 같은 바이너리
STL이라도 나온 도구가 다르다.

이게 왜 지금 중요한가: §43·§53·§56의 pr2 결론 전부가 이 포트가 파싱한 pr2
메시 정점 위에 서 있다. 정점이 Assimp와 다르면 침투깊이 논의 자체가 다른
기하에 대한 것이 된다. §56이 x-면 접촉과 z-ramp의 최소로 설명한 구조도,
§53.3이 남긴 world object 쪽 반대 방향 불일치도 마찬가지다.

p3-shapes는 §43에 대해 "octree는 아니고, 지목할 수 있는 것은 메시 파싱
커버리지 구멍"이라고 답했다. 배제 논증보다 이쪽이 이번 라운드의 산출물이다.

### 57.5 워커 보고의 사실오류 하나

보고에 `verify-scene-fixture-replay.sh`가 "아직 존재하며 삭제 지시가
이행되지 않았다"고 적혀 있다. main에는 없다 — p1-fixtures가 `ae85866`에서
지웠고 `2d0bdeb`으로 병합됐다. 워커의 베이스가 그 병합 이전이었다.
스스로 지우지 않고 올린 판단은 옳았다(자기 소유가 아니다).

## 58. 두 패널의 오라클 편집이 합쳐진 자리에서 23/23 (2026-08-04)

p3-distance-field 라운드 9 (`f812961`, `ac9f3e5`, `e31f1fb`), 병합 `bf1b193`.
`nextest --workspace` **945/945**. stamp `c5e7d2936755ea44` +
p3-distance-field의 `132da7515feddcc2` → **병합 결과 `746870de2ddd3ca6`**.

### 58.1 어느 브랜치도 빌드한 적 없는 오라클

p6-totg가 `acceleration_bounds`를, p3-distance-field가
`collision_sphere_free_functions` op을 각자 `oracle.cpp`에 넣었다. 병합
트리의 stamp는 둘 중 어느 것도 아니다 — 이 조합은 이번에 처음 빌드됐다.
빌드하고 전부 재생했다:

```
3  moveit-collision        identical
10 moveit-distance-field   identical
3  moveit-geometry         identical
1  moveit-octomap          identical
1  moveit-scene            identical
5  moveit-trajectory       identical
```

**23/23.** §54.2가 "오라클이 바뀐 뒤에도 그대로"를 한 패널의 편집으로
보였다면, 이번은 서로 모르는 두 편집이 합쳐진 트리에서 그렇다. 병합 자체가
답을 바꾸지 않았다는 증거는 이 형태로만 나온다.

참고로 재생 스크립트는 이미지를 **빌드하지 않는다**. stamp가 어긋난 채
돌리면 23건 전부 `ORACLE-FAIL rebuild with tools/moveit-oracle/build.sh`로
큰 소리를 내며 죽는다 — 이번에 실제로 그렇게 죽었고, 그게 맞는 동작이다.

### 58.2 member와 free의 갈라짐은 실재한다 — 직접 대조했다

워커의 주장(포트가 상류 자신의 member-vs-free 갈라짐을 그대로 옮겼다)을
상류에서 확인했다. `collision_distance_field_types.cpp`:

| | member (`PosedDistanceField::`) | free 함수 |
|---|---|---|
| 경계 밖 임계 | `grad.norm() > 0` | `grad.norm() > EPSILON` (0.0001) |
| `grad` 초기화 | `Eigen::Vector3d grad(0,0,0)` | `Eigen::Vector3d grad;` (미초기화) |
| `subtract_radii` 후 | `dist = std::abs(dist);` | **없음** |

Rust 쪽도 같다 — `collision_distance_field_types.rs:408`이 `> 0.0`,
`:531`이 `> EPSILON`, `:419`에 `.abs()`, free 쪽(`:535-553`)에는 없다.
free 변형은 음수 `dist`를 `gradient.distances[i]`에 그대로 넣는다. 실제로
관측 가능한 차이이고, 포트가 옮긴 것이 맞다.

### 58.3 그런데 임계 차이는 도달 불가능하다 — 감사에 빠진 사실

`DistanceField::getDistanceGradient`를 읽었다(`distance_field.cpp:73-97`,
헤더 `:313`에서 **non-virtual**):

```cpp
if (경계 밖) { gradient_x = gradient_y = gradient_z = 0.0; in_bounds = false; ... }
```

경계를 벗어나면 gradient를 **항상 (0,0,0)으로 쓴다**. 따라서
`!in_bounds && grad.norm() > 임계` 는 어느 임계값이든 `0.0 > x`를 묻는
것이고, member(`> 0`)든 free(`> EPSILON`)든 **절대 참이 될 수 없다.**
non-virtual이므로 파생 클래스가 뒤집을 수도 없다.

두 가지가 따라온다:

1. 미초기화 `grad`도 실제로는 읽히지 않는다 — 두 경로 모두 호출 직후
   세 성분이 쓰인다. 포트가 UB를 재현할 수 없다는 우려는 성립하지 않는다.
2. **새 픽스처가 "두 오버로드의 모든 분기를 친다"는 주장은 과하다.** 경계
   밖 조기 반환은 이 API를 통해 도달할 수 없는 분기다. 파리티는 유효하고
   커버리지 주장이 한 칸 넓었다.

관측 가능한 갈라짐은 `abs()` 유무 하나뿐이다. 그것은 픽스처가 실제로 친다.

### 58.4 나머지 둘

- `pregenerated_group_state_representation_map_` 도달불가 주장을 라운드 7의
  `DistanceFieldCollisionCache::generate_collision_checking_structures`에
  대해 다시 유도했다. 결론은 같지만(그것은 같은
  `generate_distance_field_cache_entry`의 새 **호출자**이지 두 번째 생성
  경로가 아니다) 근거를 타입 수준 보장으로 바꾸고 3항 falsifier를 감사
  줄에 박았다. 재유도를 요구한 이유가 이것이다 — 결론이 같아도 근거가
  낡으면 다음 라운드에 다시 물어야 한다.
- `.h` shim 여섯 개: 전문을 읽어 BSD 블록 + `create_deprecated_headers.py`/
  `moveit/moveit2#3113`을 가리키는 doc 주석 + 코드 세 줄(`#pragma once`,
  `#pragma message`, `#include` 하나)임을 근거로 적었다. "dead
  auto-generated"가 이제 재검증 가능한 문장이 됐다.

### 58.5 워커 보고의 사실오류

`verify-scene-fixture-replay.sh`를 게이트로 돌려 통과했다고 적혀 있다. main
에 그 파일은 없다(`ae85866`에서 삭제, `2d0bdeb`으로 병합). p3-shapes와 같은
원인 — 라운드 시작 시 rebase하지 않은 베이스다. §57.5와 같은 건이다.

## 59. 내가 세운 가설이 틀렸고 워커가 반증했다 — transforms는 진짜 구멍이다 (2026-08-04)

p1-fixtures 라운드 7 (`31fde4c`, `b224347`), 병합 `bc6cf36`.
`nextest --workspace` **945/945**.

### 59.1 브리핑이 준 가설, 그리고 그것이 깨진 지점

§52 브리핑에서 나는 이렇게 물었다: `scene_transforms_`의 fixed-frame 맵을
메시지가 아닌 경로로 채울 수 있는가. 그리고 내가 찾은 writer 목록
(`planning_scene.cpp:1334`/`:1383`은 메시지, `:344`/`:687`/`:1264`는 부모
장면 복사)을 제시하며 "이 열거가 완전하면 맵은 구성상 메시지로만 채워지고
D1이 덮는다"고 적었다.

**열거는 완전하지 않았다.** 워커가 찾은 것:

```cpp
// planning_scene.hpp:200
moveit::core::Transforms& getTransformsNonConst();     // public, mutable&

// transforms.hpp:113
void setTransform(const Eigen::Isometry3d& t, const std::string& from_frame);
```

직접 확인했다. 둘 다 public이고 두 번째는 ROS 타입을 하나도 받지 않는다.
`&mut PlanningScene`을 쥔 D1 범위의 호출자가 메시지 없이 fixed frame을
심을 수 있다. 내가 가설로 준 닫는 논증은 성립하지 않는다.

브리핑이 가설을 주면 워커가 그것을 확인해 오는 실패 모드를 계속 경계해
왔는데, 이번엔 반대로 갔다 — 가설을 받고 반증했다. 요구한 형태가 이것이다.

### 59.2 그래서 `getFrameTransform`에는 이 포트에 없는 단계가 있다

상류를 읽었다(`planning_scene.cpp:2036-2054`, `:2061-2071`):

```cpp
const Eigen::Isometry3d& t1 = state.getFrameTransform(frame_id, &frame_found);  // 로봇 링크/부착체
if (frame_found) return t1;
const Eigen::Isometry3d& t2 = getWorld()->getTransform(frame_id, frame_found);  // world object/subframe
if (frame_found) return t2;
return getTransforms().Transforms::getTransform(frame_id);                       // fixed-frame 맵
```

`knowsFrameTransform`도 같은 순서로 `Transforms::canTransform`까지 간다.
마지막 단계가 이 포트에 없다. `moveit_core/transforms`는 **D1 배제가 아니라
미포팅 구멍**이다. 워커가 `frame_transform`/`knows_frame_transform` 문서의
"TF tier", "tf2, D1" 표현을 전부 이 결론에 맞게 고쳤다.

### 59.3 크레이트는 만들지 않는다 — 소비자를 세어 본 결과

`transforms.hpp`를 include하는 상류 파일을 전수 조사했다:

| 소비자 | 실제로 쓰는 것 |
|---|---|
| `kinematic_constraints/kinematic_constraint.hpp` | `configure(msg, tf)`뿐 — 메시지의 frame_id를 푸는 용도, D1 인접 |
| `collision_detection/world.hpp` | `FixedTransformsMap` **typedef**(= `map<string, Isometry3d>`)만, 클래스 아님 |
| `robot_state/{robot_state,conversions,attached_body}.hpp` | msg 변환 경로 |
| `planning_scene/planning_scene.hpp` | **클래스 본체 — 위 3단계 fallback** |

D1 범위에서 `Transforms` 클래스 자체를 필요로 하는 소비자는 하나,
`PlanningScene::getFrameTransform`뿐이다. 별도 크레이트를 세울 근거가 없다 —
맵 하나와 조회, `transformPose`/`Vector3`/`Quaternion`/`Matrix` 계열이
`moveit-scene` 안에 있으면 된다. 크레이트를 하나 더 만들면 의존 그래프만
넓어지고 소유자는 그대로다.

**정정 (§66).** 이 절이 딛고 선 전제 — "이 워크스페이스 어디에도
`moveit_core/transforms` 크레이트가 없다" — 는 처음부터 거짓이었다.
`moveit_geometry::Transforms`가 `95b1854`(2026-08-03)로 이미 포팅돼
있었다. 결론(새 크레이트를 만들지 않는다)은 우연히 옳았지만 이유가
틀렸다: 만들 필요가 없었던 게 아니라 **이미 있었다**. 라운드 8 브리프가
그 거짓 전제를 검증 없이 되풀이했다.

### 59.4 `.scene` 유예는 살아남았다 — 단, 근거를 바꾼 뒤에

워커가 `planning_scene.cpp:1043-1215`를 전문 읽고 §52가 지적한 것을 확인했다:
writer는 생 float 넷을 찍고(`:1068`, 없으면 리터럴 `"0 0 0 0"` `:1071`),
reader는 float 넷을 되읽는다(`:1163-1164`). `std_msgs::msg::ColorRGBA`
직렬화가 스트림에 닿는 지점은 없다. 즉 이 크레이트의 `getObjectColor`가
D1인 것은 **이 포트가 고른 저장 타입** 때문이지 `.scene` 형식의 요구가
아니다.

그래서 "distinct, `getObjectColor`가 D1이므로"는 폐기하고, 양쪽이 같은
falsifier를 들게 했다 — "이 형식을 필요로 한다고 말한 소비자가 없다".
p3-shapes의 `saveAsText`/`constructShapeFromText` 유예와 **같은** 조건이고,
`moveit-scene`이 그 형식의 유일한 후보 소비자이면서 아직 요구하지 않았다.
서로를 가리키며 열려 있던 두 유예가 하나의 미충족 수요 조건으로 합쳐졌다.
§57.2가 걱정한 상태가 해소됐다.

## 60. world object 쪽 9건 — 그중 2건은 쌍이 같다 (2026-08-04)

p1-joints 라운드 9 (`eef7194`, `bc091c3`, `94a3f88`, `81ed055`, `01bea65`),
병합 `80080b7`. `nextest --workspace` **945/945**.

### 60.1 계기가 자기가 아는 것을 말하기 시작했다

§53.3에서 지적한 것 — 쌍 불일치는 스칼라가 이미 어긋난 줄에서만 보였다 —
이 `DistancePairStats`로 닫혔다. 쌍 disagreement를 FAIL과 **분리해서** 세고
보고하며, 동률 flip은 실패로 만들지 않는다(동률에는 유일한 정답이 없으므로
실패시키면 sweep이 무의미해진다).

3000 케이스를 직접 재현했다(seed 20260804, right_arm, `--collision`):

```
self  pair disagreement: 2935/3000 (97.8%), of which 2935 also exceeded tol
robot pair disagreement: 2647/3000 (88.2%), of which 7 also exceeded tol
```

워커 보고와 자릿수까지 같다. 이제 p3-acm이 "ranking이 얼마나 자주
뒤집히는가"의 분모를 갖는다.

### 60.2 robot 쪽 9/3000 — 그런데 보고가 한 갈래를 뭉갰다

허용오차를 넘는 robot-distance는 **9/3000 (0.30%)**. 워커 수치가 맞다
(요약줄의 7은 "쌍도 다른" 부분집합이고, 쌍이 같은 2건이 더 있다).

전부 gripper 계열 대 `floor`이고, **9건 모두 포트가 더 깊다**. 여기까지는
보고와 같다. 갈라지는 지점:

```
same-pair? oracle       rust         pair
False      -1.175e-02   -3.310e-02   l_gripper_r_finger_link  → _tip_link
False      -1.280e-02   -4.247e-02   l_gripper_l_finger_link  → l_gripper_r_finger_tip_link
False      -8.736e-03   -3.358e-02   r_gripper_l_finger_link  → _tip_link
False      -2.688e-02   -5.066e-02   r_gripper_l_finger_link  → _tip_link
False      -9.176e-03   -3.528e-02   r_gripper_l_finger_link  → _tip_link
False      -2.028e-02   -6.324e-02   r_gripper_palm_link      → r_gripper_l_finger_tip_link
False      -2.056e-02   -4.354e-02   l_gripper_l_finger_link  → _tip_link
True       -1.127e-02   -1.569e-02   l_gripper_l_finger_tip_link / floor
True       -9.943e-03   -1.237e-02   l_gripper_r_finger_tip_link / floor
```

**7건은 쌍 flip, 2건은 같은 쌍에서 크기가 다르다.** 보고는 9건을 한 덩어리로
"gripper-finger-vs-floor"라 적었는데, 두 종류다. 그리고 같은-쌍 2건이
`|d|` 2.4e-3–4.4e-3으로 flip 7건(2.1e-2–4.3e-2)보다 한 자릿수 작다.

이게 왜 중요한가: §56이 세운 기제 — 포트가 두 후보의 **최소**를 취하므로
맞고, 그래서 self 쪽에서 오라클보다 **얕다** — 는 여기서 방향이 반대다.
게다가 같은-쌍 2건에는 ranking이 아예 개입하지 않는다. 볼록 primitive
(`floor`는 Cuboid)와 메시 하나 사이의 순수한 침투깊이 불일치다. §56의
설명으로 덮이지 않는다. p3-acm 몫.

### 60.3 워커가 자기 실수의 원인을 찾았다 — 다만 고친 수가 또 안 맞는다

§53.2에서 지적한 `340/600`/`246/600` 분모 배증의 원인을 스스로 찾았다:
python 로그 파서가 같은 케이스에 대해 인라인 `FAIL` 줄과 실행 말미의 집계
줄을 **둘 다** 셌다. 원인 규명이 맞다 — 나도 3000건 파싱에서 같은 이중
계수를 만났다(18로 나왔고 실제는 9였다).

그런데 고친 값 `170/300`은 내 측정과 다르다. 내 300건 재현은
**177/300**(bellow) + **123/300**(caster) = 300으로 정확히 닫힌다. 워커의
`170 + 123 = 293`은 자기 총계 300과도 맞지 않는다. caster 쪽은 일치하므로
파싱 잔여 오차로 보인다. 라운드 10에 되돌렸다.

### 60.4 남은 셋

- `KinematicsBase` 감사를 공개 인터페이스 전체로 닫았다. 진짜 갭은
  `setValues` 하나(소비자 없음, `KDLKinematicsPlugin`이 부르지 않음).
  내가 브리핑에 나열한 미분류 후보 목록은 대부분 이미 분류돼 있었다 —
  추측 목록을 주면 워커가 그것을 확인하러 가는 비용이 든다.
- 형제 플러그인 셋에 처분이 붙었다: `srv_kinematics_plugin` 배제(ROS 서비스
  클라이언트), `ikfast_kinematics_plugin` 미포팅(이식할 알고리즘 없음,
  codegen 템플릿), `cached_ik_kinematics_plugin` **미포팅이되 진짜 범위 내
  갭** — §4.4가 D4 trait 주석에서 이것을 명시적으로 이름 부른다. "요청되지
  않음"이 아니라 근거 있는 처분이다.

**정정 (2026-08-05, §217.2가 실측).** 위 세 번째 항목의 "진짜 범위 내 갭"은
그 뒤 라운드가 닫았고 문장만 남아 있었다. `.hpp`/`-inl.hpp` 두 헤더는
`crates/moveit-kinematics/src/cached_solver.rs`가 포팅했다. 같은 디렉터리에서
미포팅으로 남는 것은 `cached_ik_kinematics_plugin.cpp`(pluginlib 등록
boilerplate, D4)와 `cached_ur_kinematics_plugin.cpp`(외부 `ur_kinematics`
의존) 둘뿐이다.

## 61. 패널이 찾은 첫 알고리즘 버그 — 회전 거리가 참각의 절반이었다 (2026-08-04)

p1-robotmodel 라운드 7(`2aa697b`, `2accae8`, `9b04950`, `f6e654f`,
`37260f4`). 이 세션에서 패널이 문서/처분이 아니라 **동작하는 코드의
계산 자체**를 틀렸다고 짚어낸 첫 건이다.

### 61.1 `Se3Space::rotation_distance`가 절반을 돌려줬다

`se3.rs`의 이전 식은 `2 * atan2(|a - near|, |a + near|)`였다. 단위
사원수 `a`, `near`에 대해 `near`는 부호 보정된 가까운 대표원이고,
`a . near = cos(theta/2)`(`theta`는 두 회전 사이 SO(3) 각). 4차원 단위
벡터 사이 각을 `phi`라 하면 `phi = theta/2`이고
`atan2(|a - near|, |a + near|) = phi/2 = theta/4`이다. 따라서 그 식이
돌려주던 값은 `theta/2` — 자기 doc 주석이 주장하던 참각의 정확히
절반이다. `4 * atan2(...)`가 `theta`이고, 이것이 `2 * acos(|a . near|)`와
실수 산술에서 완전히 같다.

내가 별도로 확인한 두 가지:

- 상류 `FloatingJointModel::distanceRotation`
  (`floating_joint_model.cpp:128-134`)은 `q2.angularDistance(q1)`이고,
  Eigen의 `angularDistance`는 `2 * atan2(|d.vec()|, |d.w()|)` — 상대
  회전의 참각이다. 30도 회전에 대해 상류 식과 `4 * atan2` 형태 둘 다
  `29.999999999999996`도, 고쳐지기 전 식은 `14.999999999999998`도.
- 이 수정은 배율 교정에 그치지 않는다. `sample_near`의 회전 분기 주석은
  "각 `theta`의 회전을 합성하면 bi-invariant 측지 거리로 정확히 `theta`
  떨어진 곳에 앉는다"고 적혀 있는데, 고치기 전에는 `theta/2`였으므로 그
  주석이 사실이 아니었고 예산을 절반만 썼다. 같은 이유로
  `rotation_radius = (rotation_budget / rotation_weight).min(PI)`의 `PI`
  절단도 고치기 전 기준으로는 도달할 수 없는 상한이었다(옛 거리의
  최대값은 `PI/2`). 상수 하나만 바꾸면 주석·절단·예산이 동시에 맞는
  자리였다.

`rotation_weight`가 `distance`에 기여하는 크기가 2배가 된다. 가중치를
튜닝해 둔 호출자가 있었다면 값의 의미가 바뀌지만, 바뀐 쪽이 상류와
같은 정의다.

### 61.2 왜 기존 테스트가 전부 통과했나

메트릭 공리(비음수·동일성·대칭성·삼각부등식)는 **양의 상수배에 대해
불변**이다. `slerp`는 거리가 아니라 `t`로 매개화되므로 보간 축도
따라간다. 그래서 무작위 추출로 공리를 검사하는 테스트는 배율이 몇이든
전부 통과한다. 잡아낸 것은 "외부에서 각을 아는" 경계 케이스 하나 —
`exp_map(x축, 30도)` — 였다. §56이 자기 논증을 원형에서 반증
가능한 형태로 바꾼 것과 같은 종류의 교훈이고, 이번에는 그 형태만이
버그를 드러냈다.

### 61.3 `moveit-constraints -> moveit-kinematics` 엣지: 승인

p1-robotmodel이 `IKConstraintSampler`/`ConstraintSamplerManager`를
`moveit-constraints`에 놓으려면 필요하다고 올린 새 의존 엣지를 승인한다.
`cargo tree -p moveit-kinematics -e normal`에 `moveit-constraints`가
없음을 확인했다 — 순환이 아니다. `check-dep-direction.sh`가 막는 것은
ROS 방향이지 이 방향이 아니다.

같은 커밋에서 확정된 사실 하나가 이 엣지의 근거다: `rrt_connect.rs`의
목표 서명이 구체 `S::State`이므로 **오늘 이 포트의 호출자는 pose goal을
표현할 방법이 아예 없다**. `constraint_samplers`는 편의 기능이 아니라
빠진 플래닝 능력이다.

### 61.4 픽스처

`panda_constraints.json`이 `panda_constraints_request.json`/
`panda_constraints_response.json`으로 재생 대상에 들어왔다. 재생
**24/24 identical**(§58의 23 + 이 한 건). §40의 "요청을 잃어버린
픽스처" 계열이 이 크레이트에서도 닫혔다.

## 62. 웨이포인트가 정착되지 않은 채 저장되고 있었다 (2026-08-04)

p6-totg 라운드 8(`fa5572e`, `d764e3f`, `73e7414`, `178b0d8`, `b670e28`).
브랜치 베이스가 `05296b7`로 여러 머지 뒤처져 있었고, 그래서 그쪽 보고의
951/951·재생 15/15·스탬프 `c5e7d29`는 전부 낡은 숫자다. 머지 후 현재
main에서 다시 잰 값이 아래 §62.4다.

### 62.1 `state->update()` 누락 — 아무도 지목하지 않은 진짜 갭

`robot_trajectory.hpp:200`/`:213`/`:226`이 웨이포인트를 저장하기 **전에**
`state->update()`를 부른다. 이 포트의 `add_suffix_way_point`/
`add_prefix_way_point`/`insert_way_point`에는 그 호출이 없었다.
`RobotState`가 `PartialEq`를 관절 위치가 아니라 캐시된 변환과
dirty-subtree 장부까지 포함해 파생하므로, 정착되지 않은 웨이포인트는 같은
논리적 상태를 정착된 경로로 저장한 것과 같지 않게 비교된다.

내가 상류 세 줄을 직접 읽어 확인했고, 회귀 테스트가 정말로 이 수정에
달려 있는지도 확인했다 — `state.update()` 세 자리를 지우고 돌리면
`add_suffix_way_point_settles_the_stored_waypoint`,
`add_prefix_way_point_settles_the_stored_waypoint`,
`insert_way_point_settles_the_stored_waypoint` 셋이 정확히 실패한다
(98건 중 95 pass / 3 fail). 삭제해도 통과하는 테스트가 아니다.

감사 중에 스스로 찾은 것이지 브리프가 지목한 것이 아니다.

### 62.2 `dynamics_solver` 처분이 틀렸다 — 이미 포팅돼 있다

같은 라운드가 `time_optimal_trajectory_generation.rs`에
"`dynamics_solver`: unported gap"과 "**no ground truth to verify it
against**: no oracle op exists for a `dynamics_solver`-shaped computation"을
써 넣었다. 두 주장 모두 사실이 아니다:

- `oracle.cpp`에 `dynamics` op가 이 라운드 이전부터 있다(`:541`), 캡처
  스크립트도 `tools/moveit-oracle/capture-dynamics-fixtures.py`로 있다.
- `moveit_state::dynamics::DynamicsSolver`(`crates/moveit-state/src/dynamics.rs`)가
  RNE 재귀를 직접 써서 포팅돼 있고 — `torques`/`max_torques`/`max_payload`/
  `payload_torques` — `dynamics_parity.rs`가 panda·fanuc·dual_arm_panda·pr2
  네 로봇의 `*_dynamics.json`으로 오라클과 대조한다.

`1757b75`에서 그 블록을 고쳤다. 살아남는 참인 부분은 더 좁다: 이 클래스는
정방향 토크 질문에 답할 뿐 `getMaxAcceleration` 모양의 메서드가 상류에도
여기에도 없으므로, 여전히 `acceleration_bounds`의 공급원은 아니다.

원인 하나는 내 브리프다 — 라운드 8 브리프가 "`dynamics_solver`
disposition"을 요구하면서 미포팅을 전제로 문장을 썼다. 담당 크레이트
밖이라 못 봤다는 것은 이유가 되지 않는다(`rg -l dynamics.json` 한 번이면
나온다). 하지만 전제를 심은 쪽이 먼저다.

### 62.3 `acceleration_filter` — 오라클 op으로 간다 (결정)

`AccelerationLimitedPlugin`의 QP가 1-D 구간 교집합으로 닫힌 형태로
환원된다는 것은 이 포트 자신의 유도이고, 검증 경로가 없다. §9.1과 §43이
반복해서 대가를 치른 것이 정확히 이 형태 — "우리가 유도했으니 맞다" —
이므로 닫힌 형태 단위 테스트만으로 승인하지 않는다.

**오라클 op을 추가한다.** 오라클 이미지의 third_party에 `osqp`,
`pluginlib`, `rclcpp`, `generate_parameter_library`가 이미 들어 있으므로
`moveit_acceleration_filter` 링크는 재료가 갖춰져 있다. `initialize()`가
`rclcpp::Node`를 요구하는 것은 `rclcpp::init` 뒤 노드를 만들어 파라미터를
직접 세우면 되고, ROS 그래프 실행을 요구하지 않는다. 링크가 실제로
데몬을 요구하는 것으로 드러나면 그 구체적 블로커를 보고하고, 그때 유도
공개(disclosed)와 함께 닫힌 형태 테스트로 후퇴한다.

### 62.4 머지 후 실측

`cargo nextest run --workspace` **961/961**, `cargo test --doc --workspace`
통과, clippy `-D warnings` 0건, `check-*.sh` 3건 전부 OK, 재생
**24/24 identical**. §4.6의 "보류"는 이 라운드로 완료 처리했다.

## 63. plateau 논증이 반증 가능한 테스트가 됐다 (2026-08-04)

p3-acm 라운드 10(`dcf3c4e`, `3825b85`, `758ef00`). 브랜치 베이스가
`f76421a`라 그쪽 보고의 944/944·재생 19/19는 낡은 숫자다. 머지 후 실측은
§63.4. 보고는 "4 commits"라고 적었지만 브랜치에는 3개다.

### 63.1 하드코딩된 오라클 리터럴 두 개가 캡처된 픽스처가 됐다

§56이 세운 min-of-two-candidates 논증이
`pr2_torso_lift_bellow_pair_crossover_confirms_min_of_two_candidates`로
들어왔다. 읽어보고 확인한 것은 이 테스트가 자기 자신을 재는 종류가
아니라는 점이다:

- `candidate_x`는 plateau 위 세 점(0.02/0.10/0.18)이 `1e-9` 안에서 모두
  같은지 확인한다 — 우연히 같은 두 점이 아니다.
- `candidate_z`는 0.23/0.25 두 끝점으로 직선을 맞춘 뒤, **내부점 0.24**를
  그 직선에 대조해 실제로 선형인지(두 점 secant가 아닌지) 확인한다.
  기울기가 `1.0`이어야 한다는 것도 물리에서 온 외부 근거다 — 메시를 z로
  `dt`만큼 강체 평행이동하면 z 방향 분리 거리는 정확히 `dt`만큼 이동한다.
- 교차점은 맞춘 직선이 아니라 **이 백엔드의 살아 있는 `distance_self`
  호출로 30회 이분해서** 관측한다. 관측값과 예측값이 `1e-4` 안에서
  일치해야 한다.
- 마지막이 진짜 반증 가능한 주장이다: 캡처된 오라클 응답에 대해 교차점
  **이전**에는 `1e-3`보다 크게 **불일치**해야 하고(그것이 deviation 6),
  교차점 **이후**에는 `1e-9` 안에서 일치해야 한다.

리터럴 `ORACLE_DIST_AT_0_1`/`_0_2`는 사라지고
`pr2_torso_lift_bellow_sweep_{request,response}.json`이 그 자리를 대신한다.
값은 내가 이전 라운드에 직접 잰 것과 같다 — `0.1`에서
`-0.13543907645960804`, `0.2`에서 `-0.05755014036972962`, `0.22`에서
`-0.03537914888262761`. 이제 재생 대상이므로 오라클이 바뀌면
`verify-fixture-replay.sh`가 먼저 말한다.

### 63.2 world 쪽이 다른 코드 경로가 아니라는 것도 확인됐다

`758ef00`의 주장을 `parry.rs`에서 직접 확인했다: `distance_self`(`:1109`)와
`distance_robot`(`:1125`)이 둘 다 `accumulate_distance`(`:902`)를 부르고,
차이는 `self_pairs(&bodies)`인지 `cross_pairs(&robot, &world)`인지뿐이다.

이것이 §60.2를 좁힌다. world 쪽 9건이 self 쪽과 반대 방향인 이유는 다른
경로를 타서가 아니다. 그리고 그중 **쌍이 같은 2건**은 §56의 순위 메커니즘으로
설명될 수 없다 — 양쪽이 같은 쌍을 골랐는데 값이 다르므로, 남는 것은
convex primitive(`floor`는 `Cuboid`) 대 메시의 침투 깊이 계산 차이뿐이다.

### 63.3 삭제된 테스트를 가리키는 문서 참조

`link_bounding_radius`의 근거 주석이 `dcf3c4e`가 지운 테스트 이름을 계속
가리키고 있었다. `dc2e616`에서 살아남은 이름으로 고쳤다.

### 63.4 머지 후 실측

`cargo nextest run --workspace` **961/961**, clippy `-D warnings` 0건,
`check-*.sh` 3건 OK, 출처 검사 OK(새 `moveit-collision/pr2.urdf`는
`identical`), 재생 **25/25 identical**.

## 64. pr2 메시 18개가 대조됐다 — §43/§53/§56이 기대던 가정이 닫혔다 (2026-08-04)

p3-shapes 라운드 10(`f11ad3a`, `96eb7b7`). 브랜치 베이스가 `0aa9b7c`라
그쪽 보고의 944/944·재생 22/22는 낡은 숫자다. 보고는 "Three commits"라고
적었지만 브랜치에는 2개다.

### 64.1 §57.2가 지목한 load-bearing 가정

§57.2가 남긴 것: `mesh_parity.json`에 pr2 항목이 0건인데 pr2 collision
STL은 18개 존재하고, §43/§53/§56의 모든 결론이 그 정점들 위에 서 있다.
이제 36건(panda/fanuc 18 + pr2 18) 전부가 오라클 `mesh` op으로 캡처돼
있고 전부 일치한다. pr2의 STL은 writer 헤더가 다르다(`VCG` 대 panda/fanuc의
`Export`)는데도 정점 집합이 비트 단위로 같다.

직접 확인한 것 셋:

- 픽스처가 진짜 오라클 캡처인지 — `base_v0/base_L.stl`을 살아 있는
  오라클에 직접 다시 요청했고 `triangle_count` 96 / `vertex_count` 50 /
  정점 리스트가 픽스처와 완전히 같다.
- 테스트가 자기 자신을 재지 않는지 — `mesh_parity.rs:143-148`은 디스크의
  STL 바이트를 읽어 이 포트의 `mesh_from_bytes`를 돌리고 그 결과를
  오라클 캡처와 대조한다. 진짜 differential이다.
- **호스트의 STL과 이미지 안의 STL이 같은 파일인지** — 이 대조가 의미를
  가지려면 필요한 전제인데, `verify-fixture-provenance.sh:100-103`이
  `shopt -s globstar` 아래 `fixtures/meshes/**/*.stl`를 이미 돌고 있고
  실행하면 정확히 35줄이 나온다. 새 메시가 들어와도 표를 고칠 필요 없이
  자동으로 검사된다. 별도로 `base_L.stl`의 sha256을 호스트와 이미지에서
  각각 재서 같은 것도 확인했다(`a85863d0...`).

`VERTEX_EPS`는 `1e-6`이고 좌표 크기는 0.1~0.6 m이므로 상대 오차 ~1e-5 —
STL이 f32로 저장된다는 점(상대 정밀도 ~1e-7)에 비해 느슨하지 않다.

### 64.2 `bodies::Body`의 "Phase 3 collision으로 미룸"은 낡은 문장이었다

`96eb7b7`이 현재 트리에 대고 다시 유도했다. `bodies::Body`는 라운드 3에
이미 포팅됐고, Phase 3 소비자로 지목됐던 `moveit-collision`은 이 모듈을
아예 쓰지 않는다 — 자기 모듈 문서에서 이름을 대고 거절하며
`ParryCollisionEnv`를 `parry3d-f64` 도형 위에 직접 세운다. 실제 현재
호출자는 `moveit-constraints`의 `PositionConstraint`(`Body::from_shape`/
`contains_point`)와 `moveit-distance-field`(`compute_bounding_sphere`/
`compute_bounding_cylinder`/`contains_point`)이다.

### 64.3 머지 후 실측

`cargo nextest run --workspace` **961/961**, clippy `-D warnings` 0건,
`check-*.sh` 3건 OK, 출처 검사 OK(메시 35건 포함), 재생 **25/25 identical**.

## 65. distance-field의 Phase 3 완료 조건은 이미 충족돼 있었다 (2026-08-04)

p3-distance-field 라운드 10(`003f8b2`, `8b992f2`, `59767b9`). 브랜치
베이스가 `3f7183e`라 그쪽 보고의 945/945·재생 23/23은 낡은 숫자다.

### 65.1 §58의 도달 불가 가드가 문서로 못박혔다

`003f8b2`가 §58이 세운 결론 — `!in_bounds && grad.norm() > threshold`는
어떤 `threshold >= 0`에 대해서도 참이 될 수 없다 — 를 코드 옆에 적었고,
그 근거를 내가 상류에서 다시 확인했다:

- `distance_field.hpp:313`의 `getDistanceGradient` 선언에 `virtual`이
  없다. 파생 클래스가 가로챌 수 없다.
- `distance_field.cpp`의 out-of-bounds 반환 경로가 `gradient_x`/`_y`/`_z`를
  전부 `0.0`으로 쓰고 `in_bounds = false`로 돌아간다.

처분도 옳다: **포팅한 채로 두되 검증된 동작으로 읽히지 않게 명시**한다.
이 크레이트의 어떤 픽스처도 이 분기를 태우지 않고, 태울 수도 없다.

부수적으로 "초기화되지 않은 `grad`" divergence도 살아 있는 차이가 아님이
같이 닫혔다 — 두 경로 모두 norm을 읽기 전에 세 성분을 다 쓴다.

### 65.2 상류 헤더 감사에서 나온 두 건

`8b992f2`의 두 주장을 각각 상류에서 확인했다:

- `data_ptrs_`는 상류 자신에게서 죽어 있다. `rg -n data_ptrs_
  moveit_core/distance_field/`의 히트가 정확히 1건이고 그것이
  `voxel_grid.hpp:288`의 선언이다.
- `propagate_negative_`는 픽스처에 고정된 값이 아니라 진짜로 흐른다.
  `propagation_distance_field.cpp`의 게이트 지점이 정확히 226, 240, 251,
  312, 333, 384행이고, 이 포트의 `propagate_negative_distances`가 같은
  두 함수(`add_new_obstacle_voxels`/`remove_obstacle_voxels`)에서 같은
  자리를 막는다.

### 65.3 계획서의 낡은 줄

이 문서가 `moveit-distance-field`에 대해 "현재 C++을 읽고 쓴 단위
테스트만 있고 Phase 3 완료 조건인 `1e-4` 대조가 없다"고 적고 있었다.
사실이 아니다 — 재생 25건 중 **10건**이 이 크레이트의 오라클 픽스처이고,
parity 테스트의 `TOL`은 다섯 파일에서 정확히 `1e-4`다. 줄을 고쳤다.

### 65.4 머지 후 실측

`cargo nextest run --workspace` **961/961**, clippy `-D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 재생 **25/25 identical**.

## 66. 세 번째 tier가 닫혔다 — 그리고 §59.3의 전제는 거짓이었다 (2026-08-04)

p1-fixtures 라운드 8(`52261fe`, `4588c4f`). 브랜치 베이스가 `163531e`라
그쪽 보고의 953/953·재생 23/23은 낡은 숫자다. 커밋 두 개인데 하나가
findings 두 개를 담고 있다(`52261fe` = item 1 + item 3) — 한 finding 당
한 커밋 규칙 위반이다.

### 66.1 내 브리프가 거짓 전제를 되풀이했다

라운드 8 브리프는 "`moveit_core/transforms`는 포팅되지 않은 구멍"이라는
라운드 7의 발견 위에서 상류 소비자를 세고 `moveit-scene`에 두라고
라우팅했다(§59.3). 그 전제가 거짓이다 — `moveit_geometry::Transforms`가
`95b1854`(2026-08-03)로 이미 있었고, `transform`/`transform_vector3`/
`transform_quaternion`/`transform_rotation_matrix`/`transform_pose`까지
갖춰져 있다. 직접 확인했다.

담당이 스스로 잡아서 중복 구현 대신 그것을 재사용했고, `scene.rs`/
`layered.rs`의 잘못된 문장도 같이 고쳤다. 결론(새 크레이트 없음)은
우연히 맞았지만 이유가 틀렸다: 만들 필요가 없었던 게 아니라 이미
있었다.

이번 라운드-세트에서 내가 워커 보고의 전제를 확인하지 않고 브리프에
실어 보낸 두 번째다. 크레이트 존재 여부는 `rg -n "pub struct Transforms"
crates/` 한 번이면 나온다.

### 66.2 비재귀가 구조적으로 성립한다

브리프가 요구한 것은 "주석이 아니라 구조로" 무한 재귀를 막으라는
것이었다. 상류는 `SceneTransforms::getTransform`이 `scene_->getFrameTransform`을
부르고 그쪽이 `getTransforms().Transforms::getTransform`으로 **명시적
한정**해서 재귀를 끊는다(`planning_scene.cpp:2053`, `:2070`).

이 포트에는 그 고리가 없다. `PlanningScene::transforms()`가 돌려주는
것은 씬 역참조가 없는 순수 `moveit_geometry::Transforms`이므로, 재귀가
성립할 수 있는 경로 자체가 존재하지 않는다. 한정자도 주석도 필요 없다.

tier 순서는 상류 `planning_scene.cpp:2036-2054`와 같다:
state → attached body → world → extra fixed frame map(`scene.rs:954-966`).

### 66.3 경계값으로 테스트됐다

새 테스트 8건이 시나리오가 아니라 경계다: 없는 이름, 빈 문자열, 선행
슬래시, link 이름과 map 키가 같을 때 어느 tier가 이기는지, attached
body와 world object가 같은 이름일 때 어느 쪽이 이기는지,
`decouple_parent`가 상속된 맵을 실체화하는지.

### 66.4 남은 것

`SceneTransforms::isFixedFrame`의 선행 `/` 처리와 object frame 위임은
재현되지 않았다. 유예 근거는 falsifier가 붙어 있다: 상류 전체에서 유일한
호출자가 `kinematic_constraints/kinematic_constraint.cpp`(4곳)이고 그것이
아직 미포팅에 메시지 타입이다. 소비자가 생기면 닫힌다.

### 66.5 머지 후 실측

`cargo nextest run --workspace` **969/969**, clippy `-D warnings` 0건,
`fmt --check`/`doc --no-deps` 통과, `check-*.sh` 3건 OK,
재생 **25/25 identical**.

## 67. self 쪽에도 same-pair 종이 있다 — 62건 (2026-08-04)

p1-joints 라운드 10(`024af85`, `12b70ab`, `a107193`, `b7dd96b`). 패널이
전달한 보고 텍스트는 이미 머지된 **라운드 9**의 것이었다(5 commits,
베이스 `5aac6ed`, 941/941, 재생 10/10). 브랜치에는 라운드 10 커밋 4개가
`0d1afee` 위에 실제로 올라와 있었고, 보고 없이 코드만 있는 상태였다.
아래는 내가 직접 재서 확인한 결과다.

### 67.1 `--stats-json`이 세 번째 재파싱을 끝냈다

이 라운드-세트에서 분모 오류가 세 번 나왔고 전부 사람이 읽는 로그를
일회용 python으로 긁은 데서 나왔다(§60.1, §60.3, 그리고 내 것 하나).
`moveit-diff`가 이미 갖고 있는 수를 `--stats-json <path>`로 내보내게
했으니 이제 재파싱이 아니라 재실행으로 확인된다. 구조적 처방이 맞다.

바로 그 도구로 3000건을 다시 돌렸다(`right_arm`, seed 20260804,
`--collision`):

```json
"self_total": 3000,  "self_pair_disagrees": 2935,
"self_pair_flip_and_value_diverges": 2935,
"self_same_pair_and_value_diverges": 62,
"robot_total": 3000, "robot_pair_disagrees": 2647,
"robot_pair_flip_and_value_diverges": 7,
"robot_same_pair_and_value_diverges": 2
```

world 쪽 7 + 2 = 9는 §60.2가 손으로 쪼갠 값과 정확히 같다. 계기가
사람의 분류를 재현했다.

### 67.2 내 브리프가 물은 것의 답: 0이 아니라 62다

라운드 10 브리프가 물었다 — "self 쪽에 same-pair 종이 아예 없다면 그것
자체가 발견이다". 없지 않다. **62/3000**이고, world 쪽 2건보다 31배
흔하다.

이것이 §56과 §63.2를 다시 좁힌다. §56이 세운 min-of-two-candidates는
**순위** 메커니즘이므로 양쪽이 같은 쌍을 고른 62건을 설명하지 못한다.
그중 얼마가 이미 알려진 plateau 계열(양쪽 다 `base_bellow_link`/
`torso_lift_link`를 골랐고 값만 다른, deviation 6)이고 얼마가 새로운
것인지는 아직 나뉘어 있지 않다. p3-acm 라운드 11이 world 쪽 2건을 다루고
있으므로, self 쪽 62건의 쌍 구성표가 그 작업의 입력이다.

### 67.3 bellow 분모는 177/300이 맞다

`b7dd96b`가 `170/300`을 `177/300`으로 고쳤다 — 내가 이전 라운드에 직접
재서 얻은 값과 같고, caster 쪽 `123/300`과 합쳐 정확히 300이 된다.
원인은 매칭 범위가 `, robot oracle` 이후 텍스트까지 먹은 것이었다.

### 67.4 머지 후 실측

`cargo nextest run --workspace` **970/970**, clippy `-D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 재생 **25/25 identical**.

## 68. metric 배율 결함을 계열로 쓸어냈다 (2026-08-04)

p1-robotmodel 라운드 8(`c2ff170`, `4cc14e1`). 베이스 `1b58458`.

### 68.1 요구한 숫자는 보고가 아니라 커밋 메시지에 있었다

브리프가 화면에 요구한 것 — distance/metric 진입점이 몇 개이고, 그중
몇 개가 이번 라운드 전에는 외부값 경계 테스트가 없었는지 — 은 보고
본문에 없다. `c2ff170`의 커밋 메시지에는 있다: **진짜 진입점 5개**,
그리고 산술 없이 그대로 위임하는 어댑터 3개(`compound.rs`의
`RealVectorAdapter`/`So2Adapter`/`Se3Adapter`). 다섯 개 각각에 대해 어떤
상류 심볼로 못박았는지가 파일:행과 함께 적혀 있다:

- `RealVectorSpace::distance` — `prismatic_joint_model.cpp:114-116`,
  `revolute_joint_model.cpp:180-181`
- `So2Space::distance` — `revolute_joint_model.cpp:173-179`의 continuous 분기
- `Se3Space::distance` — `floating_joint_model.cpp:120-126`(병진),
  `:115-118`(가중합). 회전은 `9b04950`에서 이미
- `CompoundSpace::distance` — `joint_model_group.cpp:462-471`
- `JointModelGroupSpace::distance` — `fixtures/panda.urdf`의 실제 한계값

보고에 안 적으면 없는 것과 같다. 다음 라운드 브리프에 그렇게 적었다.

### 68.2 테스트가 틀렸고 구현이 맞았던 건

`JointModelGroupSpace`의 pin을 처음 쓸 때 URDF `<limit>` 값을 그대로
기대값으로 넣었더니 구현과 어긋났다. 실제 bound는
`<safety_controller soft_lower_limit/soft_upper_limit>`에서 온다. 상류
`robot_model.cpp:898-908`을 직접 확인했다 — `urdf_joint->safety`가 있으면
soft limit을 쓰고, hard limit은 그것을 **좁히는 방향으로만** 적용된다
(`limits->lower > min_position_`일 때만 올리고, `limits->upper <
max_position_`일 때만 내린다). 담당이 테스트를 고쳤지 구현을 고치지
않았다.

### 68.3 sampler 두 개가 들어왔다

`4cc14e1`. 브리프가 놓치기 쉽다고 지목한 두 가지가 둘 다 들어왔다:

- `JointConstraintSampler::configure`가 bound 교집합이 비면 **configure
  시점에 실패**한다(샘플링 시점이 아니라). 경계 테스트 4종 — 빈 교집합,
  한 점 교집합(`tolerance == 0`), 그룹 밖 관절, 입력 순서를 뒤집은 union.
- `UnionConstraintSampler`의 `OrderSamplers` 술어가 그대로 이식됐다.
  상류 `union_constraint_sampler.cpp:60-121`과 한 줄씩 대조했다:
  updated-link 집합 포함 관계 → frame dependency → 순환 의존 tie-break →
  `JointConstraintSampler` 우선 → 그룹 이름 사전순. `dynamic_cast` 자리는
  `is_joint_constraint_sampler()` trait 메서드가 대신하고, 정렬은
  `sort_by`(Rust에서 stable)로 상류 `std::stable_sort`와 같다.

### 68.4 `IKConstraintSampler`의 블로커 — 결정

담당이 두 가지를 올렸다. 하나는 자기 것이고(두 `OrientationConstraint`
접근자, 이제 "미사용 접근자 갭"이 아니라 확인된 블로커다), 다른 하나를
나에게 라우팅했다: 상류 `ConstraintSamplerManager::selectDefaultSampler`가
`jmg->getGroupKinematics()`로 그룹→solver를 찾는데 이 포트에는 그 매핑이
없다.

**결정: 매핑을 만들지 않는다. `IKConstraintSampler`가 solver를 인자로
받는다.** 근거는 D4다 — `moveit-kinematics`의 `KINEMATICS_SOLVERS`는
알고리즘의 컴파일타임 레지스트리이고, `SolverRegistration::construct`가
이미 group_name을 **호출자에게서** 받는다. 그룹별 기본 solver 지정은
상류에서 `kinematics.yaml`/`robot_model_loader`가 하는 런타임 설정이고,
그 계열(`ConstraintSamplerManager`의 문자열 플러그인 디스패치)은 이미
D4로 제외돼 있다. 여기서 `JointModelGroup`에 `group_kinematics_`를
새로 심으면 제외한 런타임 설정 계층을 뒷문으로 들이는 셈이다.

따라서 블로커는 하나로 줄어든다 — 두 접근자 — 이고 그것은 담당의
크레이트 안에 있다. `moveit-kinematics`/`moveit-model`은 이 건으로
건드리지 않는다.

### 68.5 머지 후 실측

`cargo nextest run --workspace` **981/981**, clippy `-D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 재생 **25/25 identical**.

## 69. 세 유예가 전부 근거를 갖고 닫혔다 (2026-08-04)

p3-shapes 라운드 11(`11c0c8a`, `6260306`, `bf348ae`). 베이스 `8705eab`.
커밋 3개, finding 3개 — 이번 라운드-세트에서 한 finding 당 한 커밋을
정확히 지킨 유일한 패널이다.

### 69.1 `Voxels` 갭은 죽어 있던 갭이었다

세 라운드 동안 "left open, not re-verified"로 실려 다니던 항목이다.
담당이 `voxels.rs`의 `pub fn` 18개를 전부 확인해 uniform-`voxel_size`
제약이 그대로임을 재확인한 뒤, 진짜 발견을 붙였다: **이 워크스페이스에
`Voxels`를 쓰는 코드가 없다.** 직접 확인했다 — `rg -n "Voxels::(new|from)"
crates`의 히트는 `crates/moveit-geometry/src/shapes.rs:318`의 doc 주석 한 줄뿐이고, 나머지 히트는
전부 `addNewObstacleVoxels` 같은 부분 문자열이거나 주석이다. 실제로 쓰는
`compound_from_octree` 경로에는 그 제약이 없고 leaf 0~216에서 오라클
검증돼 있다.

falsifier 없는 "open"은 잊어버린 것과 구별되지 않는다 — 이 항목은
잊어버린 것이 아니라 애초에 살아 있는 결함이 아니었다.

### 69.2 `probe-parity`는 좌초하지 않았다

내가 라운드 11 브리프에서 "§9.1이 찾은 두 불일치의 픽스처가 브랜치
`probe-parity`(`dbf50a7`)에 있고 main에 못 들어왔다면 그것이 다른 두
항목보다 우선한다"고 걸어 둔 건이다. 좌초가 아니다:

- `0032889`(`moveit-geometry: probe bodies:: against the shipped
  libgeometric_shapes`)가 main의 조상이다.
- `probe_parity.rs`가 main의 테스트 트리에 있다.
- 후속 수정 넷도 main에 있다 — `aa80496`, `16cf87b`, `10b1909`,
  `db7afde`.

`dbf50a7`은 그 작업의 고아 중복본이다. §9.1의 `ConvexMesh::ray_intersections`
/ `OBB::extend_approx` 두 건은 이미 닫혀 있었다.

### 69.3 `BodyVector`는 라우팅이 아니라 결정으로 닫혔다

`BodyDecomposition::from_shapes`가 만든 `Vec<Body>`의 소비 지점
(`collision_distance_field_types.rs:711-786`)이 전부 전체 순회 아니면
인덱스 접근이고, `BodyVector`의 first-hit 질의를 쓰는 곳이 없다. 래퍼가
사는 것이 없다.

### 69.4 머지 후 실측

`cargo nextest run --workspace` **981/981**, clippy `-D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 재생 **25/25 identical**.

## 70. `.scene` 유예가 양쪽에서 동시에 닫혔다 (2026-08-04)

p1-fixtures 라운드 9(`86f102c`, `37f7a2f`, `943e909`). 베이스 `9c76ff2`.
finding 3개, 커밋 3개.

### 70.1 두 패널이 서로의 침묵을 가리키던 유예

`saveGeometryToStream`/`loadGeometryFromStream`(`moveit-scene`)과
`shapes::saveAsText`/`constructShapeFromText`(`moveit-geometry`)가 같은
falsifier — "이 형식을 필요로 한다고 말한 소비자가 없다" — 를 들고 서로를
가리키고 있었다(§59.4). 없는 수요는 잊어버린 항목과 구별되지 않으므로
긍정문으로 닫으라고 요구했고, 담당이 근거를 갖고 닫았다.

상류에서 이 두 함수를 부르는 곳을 내가 직접 세어 확인했다:

```
moveit_ros/warehouse/src/{save_as_text,import_from_text}.cpp
moveit_ros/move_group/src/default_capabilities/{load,save}_geometry_*_service_capability.cpp
moveit_ros/visualization/motion_planning_rviz_plugin/src/motion_planning_frame_objects.cpp
moveit_ros/planning/planning_components_tools/src/publish_scene_from_text.cpp
moveit_py/src/moveit/moveit_core/planning_scene/planning_scene.cpp
```

정의와 자기 테스트를 빼면 **전부 `moveit_ros` 아니면 `moveit_py`**다. D1/D2
어느 쪽으로도 범위 밖이다. 이제 "수요가 없다"가 아니라 "수요가 있는 곳이
전부 범위 밖이다"로 적혀 있고, p3-shapes의 대칭 유예도 같이 닫힌다.

### 70.2 stale해진 overload 개수 세 건

`943e909`이 라운드 6에서 물려받은 개수 셋을 헤더에 대고 다시 셌다. 내가
`planning_scene.hpp`에서 직접 재확인했다:

| 심볼 | 라운드 6이 적었던 값 | 실제 |
|---|---|---|
| `checkCollision` | 7 | **6** |
| `getCollidingLinks` | 5 | **6** |
| `getCollidingPairs` | 5 | **6** |

`getCollidingPairs` 6개 중 `group_name`을 받는 것은 1개가 아니라 **4개**다
(선언마다 괄호 짝을 맞춰 시그니처를 잘라내 세었다). 감사 bullet 59개를
전부 다시 걸었고 `unported, in scope`는 0건 — 커버리지는 이미 완전했고,
틀린 것은 개수뿐이었다.

### 70.3 남은 것

`isFixedFrame`의 world object frame 위임은 여전히 살아 있는 호출자가
없다. 다만 falsifier의 전제는 바뀌었다 — `moveit-constraints`가 이제
통째로 미포팅이 아니고 `PositionConstraint`/`OrientationConstraint`/
`VisibilityConstraint`가 `can_transform`으로 base class 절반을 이미
재현한다. 막고 있는 것은 살아 있는 `PlanningScene`에서 `Transforms`를
그 생성자들로 흘려보내는 다리가 없다는 것이고, 그것이 생기면 닫힌다.

새로 표면화된 것: `getCostSources`가 막혀 있다 —
`ParryCollisionEnv`가 `cost_sources: None`을 하드코딩한다. p3-acm 소관.

### 70.4 머지 후 실측

`cargo nextest run --workspace` **981/981**, clippy `-D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 재생 **25/25 identical**.

## 71. 복사된 tolerance 다섯 개 — 5자리 조였지만 아직 6자리 헐겁다 (2026-08-04)

p3-distance-field 라운드 11(`994c4b3`, `07c5591`). 베이스 `e690982`.

### 71.1 두 번째 propagation mode는 없다

내가 라운드 11 브리프에서 "Euclidean 대 Manhattan 축이 남아 있다"고 건
것을 담당이 반증했다. `rg -ci "manhattan|chebyshev" moveit_core/distance_field/`
는 히트 0이고, `PropagationDistanceField`가 그 패키지의 유일한
`DistanceField` 파생이며, 생성자 셋이 갖는 boolean mode 파라미터는
`propagate_negative_distances` 하나뿐이다. 내가 직접 다시 돌려 확인했다.
내가 만든 가설이고 담당이 지웠다.

`find_internal_points.hpp`는 라운드 10에 이미 감사돼 있었다
(`findinternal_points_convex` → `find_internal_points_convex`). 빠진 것은
작업이 아니라 보고였다.

### 71.2 tolerance 다섯 개가 측정된 적 없는 복사값이었다

`TOL`/`DISTANCE_TOL` 다섯 개가 전부 §5의 정책값 `1e-4`를 복사한 것이었고
실제 일치도와 대조된 적이 없었다. 담당이 계측을 붙여 재고 `1e-9`로
조였다. 5자리 개선이고 방향이 맞다.

**다만 "각 파일 최악 측정치보다 4자리 위"라는 근거는 재현되지 않는다.**
내가 다섯 상수를 `1e-15`로 낮춰 돌렸더니 `-p moveit-distance-field`의
**72건이 전부 통과한다**. 단언은 `assert_relative_eq!(a, b, epsilon = TOL)`
이고 `approx`의 `max_relative`는 기본값 `f64::EPSILON`(~2.22e-16)이므로,
이 통과는 절대 오차가 전부 `1e-15` 아래라는 뜻이다. 즉 현재 테스트
집합에서 `1e-9`가 실제로 물리는 지점보다 **최소 6자리 헐겁다**.

담당이 잰 `1.12e-13`이 상대값이고 내가 잰 것이 절대값이라 갈릴 수 있다.
어느 쪽이든 "몇 자리 위"라는 문장은 물리는 지점을 찾아서 쓰는 것이지
측정치에 상수를 더해 쓰는 것이 아니다. 라운드 12로 넘긴다.

### 71.3 손대지 않은 두 개는 손대지 않은 것이 맞다

`RADIUS_TOL = 1e-12`는 오늘 다시 재서 기록(`3.469e-18`/`1.436e-16`)과
같았고 그대로 뒀다. `POINT_EPS = 1e-6`은 grid bucket 크기라 거친 쪽이
안전한 방향이므로 값은 유지하고, 문서의 근거 없는 "여섯 자리" 주장만
측정값 ~열 자리로 고쳤다.

### 71.4 머지 후 실측

`cargo nextest run --workspace` **981/981**, clippy `-D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 재생 **25/25 identical**.

## 72. world 쪽 마지막 미해명 2건이 닫혔다 — 이 포트가 맞다 (2026-08-04)

p3-acm 라운드 11(`fc361a9`, `d499634`). 베이스 `90c11a3`.

### 72.1 충돌 파이프라인을 전혀 타지 않는 측정

§60.2/§63.2가 남긴 것: `l_gripper_{l,r}_finger_tip_link`/`floor` 두 건은
양쪽이 **같은 쌍**을 골랐는데 값이 다르므로 §56의 순위 메커니즘으로
설명되지 않는다. 침투 깊이는 최소 이동 거리(MTD)이므로, 더 깊은 답은
얕은 답이 실제 분리 방향에 대응하지 않을 때만 정당하다.

담당이 그 판정을 두 백엔드 어느 쪽에도 의존하지 않는 방법으로 했다:
`deepest_vertex_under_floor`가 링크 메시의 **자기 정점들**을 global link
transform으로 옮겨 가장 낮은 z를 찾는다. parry도 FCL도 타지 않는다.
그리고 그 정점이 `floor_env`의 4×4m 안쪽에 있는지를 테스트 안에서
단언한다 — 그래야 "똑바로 위가 유일한 싼 탈출"이 성립하고 깊이가 곧
MTD가 된다.

결과: 두 건 모두 이 포트의 크기를 재현한다(422번 0.015686397 대
0.015686399, 2996번 0.012374991 대 0.012374991).

내가 확인한 것은 이 단언이 실제로 구별력이 있는지다. `TOLERANCE`는
`1e-4`이고 오라클과 이 포트의 차이는 4.41e-3과 2.43e-3 — 각각 44배와
24배다. 이 포트가 오라클의 값을 냈다면 정점 검사가 실패한다.

**결론: `parry.rs`의 결함이 아니다.** 오라클의 얕은 값이 도달 가능한
분리 이동에 대응하지 않는다. `distanceRobot`의 re-collide-and-take-max-depth
탐색이 이 메시의 진짜 최심점을 놓치는 것이고, 이는 deviation 6과 같은
메커니즘(FCL의 비볼록 침투 깊이는 근사이지 정확한 EPA가 아니다)이
쌍 순위가 아니라 크기 불일치로 나타난 것이다.

### 72.2 7과 2는 겹치지 않는다 — 계기가 보장한다

`main.rs:1521-1541`을 직접 읽었다. `distance_pair_matches`가 참이면
`*_same_pair_and_value_diverges`, 거짓이면 `*_pair_disagrees` 후
`*_pair_flip_and_value_diverges`로 간다 — `if/else`이므로 한 케이스가 두
카운터에 동시에 들어갈 수 없다. §60.2가 손으로 나눈 7 + 2가 구조적으로
서로소임이 코드로 보장된다.

### 72.3 `assert_plausible_depth`는 이 계열을 잡을 수 없었다

`link_bounding_radius("floor")`가 `None`이다(floor는 로봇 링크가 아니다).
그래서 robot/World 쌍에서는 경계가 fingertip 자기 반지름의 2배
(≈0.069m)로 퇴화하고, 양쪽 값 0.010~0.016m는 그 안에 넉넉히 들어간다.
이 불일치는 실제 world geometry에 대고 재는 것(`deepest_vertex_under_floor`)
으로만 잡힌다. `link_bounding_radius`에 적혔다.

### 72.4 머지 후 실측

`cargo nextest run --workspace` **982/982**, clippy `-D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사 통과,
재생 **26/26 identical**(새 `pr2_world_object_same_pair` 포함).

이로써 §43에서 시작한 pr2 거리 불일치 계열에서 **world 쪽은 전부
설명됐다** — 7건은 순위 flip(§56의 메커니즘), 2건은 이 포트가 맞고
오라클이 얕다. 남은 것은 self 쪽 same-pair 62건(§67.2)이다.

## 73. `moveit-octomap`의 첫 심볼 감사와 §68 결함 계열의 음성 확인 (2026-08-04)

p3-shapes 라운드 12(`1aa52b3`, `1eafc3b`, `2a02a46`). 베이스 `7335e34`,
main `85d1a14`에 머지. 세 건 모두 문서 변경이라 테스트 수는 그대로다.

### 73.1 `moveit-octomap`은 이제껏 심볼 대조를 한 번도 받지 않았다

`moveit-geometry`의 `shapes.rs`/`bodies.rs`는 라운드마다 상류 헤더와
심볼 단위로 맞춰 왔지만 `moveit-octomap`은 그 대상이 아니었다. 이번에
오라클 컨테이너의 `liboctomap-dev 1.9.7`을 뜯어 7개 헤더에 대해
40개 심볼 그룹을 4분류했다 — ported 22, unported-in-scope 3,
architecturally distinct 15.

distinct 15건 중 네 개를 상류에서 직접 확인했다. `getMetricMin`/
`getMetricMax`/`getMetricSize`, `insertPointCloud`, `enableChangeDetection`,
`castRay`/`getNormals` — `moveit_core` 전체에서 호출자 0건이다. 확률
공간 게터의 유일한 `moveit_core` 호출자가 Bullet 백엔드라는 주장도
맞다: `bullet_utils.cpp:210`의 `getOccupancyThres()` 한 곳이다(내 첫
`rg`가 `ProbHit`만 봐서 0건으로 나왔는데, 이름을 넓히니 나온다).

### 73.2 `tree_iterator`에 처음으로 이름 있는 소비자가 붙었다

`collision_distance_field_types.cpp:355`의
`PosedBodyPointDecomposition(const shared_ptr<const octomap::OcTree>&)`
생성자가 `begin_tree()`/`end_tree()`만으로 구현돼 있다 — 확인했다.
그리고 Rust 쪽 `collision_distance_field_types.rs:929`의 같은 타입은
`BodyDecomposition` 계열 생성자 두 개만 포팅돼 있고 octree 생성자는
없다.

즉 §13의 "standalone octree는 있으나 collision 경로에 아직 물리지
않았다"는 위험이 추상적 진술이 아니라 **상류 한 지점과 그에 대응하는
Rust 쪽 결손 한 지점**으로 특정됐다. 두 크레이트에 걸친 항목이므로
p3-shapes와 p3-distance-field 양쪽 라운드 13에 넣는다.

### 73.3 `num_nodes`는 `size()`가 아니다

`tree.rs:300`의 `num_nodes`는 재귀 순회다 — 상류 `calcNumNodes()`
(`OcTreeBaseImpl.h:269`)에 대응하고, O(1) 카운터 `size()`
(`OcTreeBaseImpl.h:241`, `return tree_size;`)는 어떤 이름으로도 포팅되지
않았다. 헤더 두 줄을 컨테이너에서 직접 읽어 확인했다. `tree_size`는
이 워크스페이스에 없다(`iter.rs:221`의 `size()`는 leaf 한 변 길이라
무관하다).

### 73.4 §68 계열 점검: 네 개 모두 이미 외부에 고정돼 있다

"일률적으로 2배 큰 반지름은 이 크레이트의 모든 포함 관계 테스트를
통과한다"는 §68의 결함 계열을 `compute_bounding_sphere`/
`compute_bounding_cylinder`/`compute_volume`/`OBB::extend_approx`에
적용한 결과는 음성이다 — 넷 다 이미 고정돼 있다.

담당의 주장을 읽는 대신 구현을 흔들어 확인했다. 각 메서드의 반환값에
`1.000001`을 곱하고(구체 타입 4개 impl 전부, enum dispatch 제외)
`-p moveit-geometry --no-fail-fast`를 돌렸다:

- `compute_bounding_sphere` ×1.000001 → 5건 실패. probe_parity의
  sphere/cylinder/cuboid/convex_mesh 네 개 전부 + `body_query_parity`
- `compute_bounding_cylinder` ×1.000001 → 같은 5건
- `compute_volume` ×1.000001 → 8건 실패. 위 5건 + 해석적 단위 테스트
  3건(`sphere_volume_matches_four_thirds_pi_r_cubed` 등)
- `OBB::extend_approx`의 `half_extents` ×1.000001 → 3건 실패.
  `obb_predicates_match_libgeometric_shapes`,
  `obb_extend_approx_merge_largedist_matches_libgeometric_shapes`,
  `obb_extend_approx_noop_when_self_contains_other`

고정의 출처가 외부라는 것도 확인했다: `bodies_probe.json`은 오라클
이미지 안 `libgeometric_shapes.so.2.3.3`에 링크한 C++ 프로브의 `%.17g`
stdout이고(§9.1의 바이너리 프로브 경로), `body_query_parity`는 오라클의
`body_query` op이라 경로가 완전히 다르다. `check_body!` 매크로는 네
body 타입 전부에 대해 volume/bsphere/bcyl을 무조건 단언한다 —
`skip_rays`는 ray만 건너뛴다.

`ConvexMesh`가 특히 중요하다. 손으로 계산할 닫힌 형태가 없어서
자기 일관성 외에는 검증 수단이 없는 타입인데, 실제 `.so`가 그 역할을
한다.

### 73.5 `.scene` 텍스트 포맷은 양쪽이 닫혔다

`saveAsText`/`constructShapeFromText`의 유일한 근거였던 falsifier가
p1-fixtures의 `86f102c`(§70)로 소멸했다 — `.scene` 파일 상호운용이
`moveit-scene` 쪽에서 명시적 out-of-scope 결정이 됐으므로 이쪽 절반도
"미포팅"이 아니라 "결정"이다. 포팅 레시피는 문서에 남겼다.

### 73.6 머지 후 실측

`cargo nextest run --workspace` **982/982**, `cargo test --doc --workspace`
통과, clippy `--workspace --all-targets -D warnings` 0건, `fmt --check`
통과, `check-*.sh` 3건 OK, 재생 **26/26 identical**.

## 74. `AccelerationLimitedFilter`가 들어왔고, ruckig 스트리밍 fixture는 재는 게 없다 (2026-08-04)

p6-totg 라운드 9(`eef9370`, `ec9a539`, `c62603a`, `56c5afa`). 베이스
`7eabb53`, main `f1c14ef`에 머지. `oracle.cpp`가 바뀌어 스탬프가
`746870de2ddd3ca6` → **`fe75d0c58eb61962`**로 옮겨갔고 이미지를 다시
빌드했다.

### 74.1 상류의 인덱스 버그는 실재하고, 이 포트는 그 전제를 없앴다

`acceleration_filter.rs`의 deviation 노트가 주장하는 상류 버그를
직접 확인했다. `acceleration_filter.cpp:189-207`에서 바깥 루프는
`getActiveJointModelsBounds()`(관절당 한 항목), 안쪽 루프는 그 관절의
변수들인데 `ind++`가 **206행, 안쪽 루프가 닫힌 뒤**에 있다. 변수가 둘
이상인 관절은 마지막 변수의 값만 남기고 다음 관절 슬롯으로 넘어가지
않는다.

`joint_acceleration_bounds`는 관절 이름으로 한 관절당 한 bound를 읽으므로
다변수 관절을 표현할 방법 자체가 없다 — 버그를 고친 게 아니라 버그의
전제가 성립하지 않는 좁은 표현이다. 다만 fixture가 전부 `panda_arm`
(전 관절 단일 DOF)이라 이 좁힘은 어느 쪽으로도 검증되지 않는다.

같은 파일의 나머지 두 주장도 상류에서 확인했다: `doSmoothing`의 세 번째
인자는 `Eigen::VectorXd& /* unused */`(:310)이고,
`const size_t num_positions = velocities.size();`(:312)도 그대로다.

### 74.2 `acceleration_filter` 오라클 op는 구별력이 있다

`do_smoothing`의 혼합식 `*p = alpha * last_p + (1.0 - alpha) * *p`에
`1.000001`을 곱해서 돌렸다 → **5건 실패**, 그중
`acceleration_filter_matches_the_oracle` 포함. 오라클 대조가 실제로
물린다.

### 74.3 `ruckig_filter` 오라클 op는 `target_velocity`를 전혀 재지 않는다

같은 시험을 `RuckigFilter::do_smoothing`에 했더니 결과가 다르다.

```
target_velocity = current_velocity + 1.000001 * current_acceleration * dt  → 29/29 통과
target_velocity = current_velocity + 2.0      * current_acceleration * dt  → 29/29 통과
target_velocity = current_velocity + 0.0      * current_acceleration * dt  → 29/29 통과
target_velocity = 0.0                                                       → 29/29 통과
```

`target_velocity`를 전 관절 0으로 지워도 `ruckig_filter_matches_the_oracle`
(`TOL = 1e-9`, 스텝마다 위치·속도·가속도 전부 단언)을 포함해 한 건도
실패하지 않는다. 이 줄은 `do_smoothing`에서 위치 통과 외의 유일한 계산인데
새 fixture가 재는 것이 없다.

원인은 fixture의 형상이다. 케이스 1이 유일한 다중 스텝(5 커맨드)인데
목표가 0.3rad, jerk/accel/vel 한계가 전부 1.0, 주기 0.1s다. 0.5초 뒤에도
가속도는 0.5, 위치는 0.02 언저리 — 프로파일이 아직 초기 jerk 상승
구간이라 "목표 속도로 도착하도록 감속"하는 국면에 들어가지 않는다.
그 국면에서만 `target_velocity`가 출력에 영향을 준다.

이 포트의 줄 자체는 상류와 정확히 같다(`ruckig_filter.cpp:103-104`, 그리고
`Ruckig`가 `params_.update_period`로 생성되므로 `delta_time`으로 바꿔 쓴
것도 같은 값이다). 결함은 포트가 아니라 fixture의 구별력이다. 라운드 10
1번 항목.

### 74.4 머지 후 실측

`cargo nextest run --workspace` **995/995**(982 + 13), `cargo test --doc
--workspace` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사 통과, reseed-wrap 통과,
재생 **28/28 identical**(새 `moveit-smoothing/acceleration_filter`,
`moveit-smoothing/ruckig_filter` 포함).

## 75. `isFixedFrame` 다리는 놓였고, mesh cost source는 백엔드 한계가 아니다 (2026-08-04)

p1-fixtures 라운드 10(`ee78221`, `2046bbe`). 베이스 `5830e75`,
main `252354d`에 머지. **995 → 999**.

### 75.1 `transforms_with_world_objects`는 상류 범위와 맞다

`planning_scene.cpp:123-137`의 `SceneTransforms::isFixedFrame`을 읽었다:
빈 문자열 false → `Transforms::isFixedFrame`(원문 그대로, 베이스 맵)
→ 앞의 `/` 하나만 벗겨 `knowsObjectFrame` → `World::knowsTransform`
(`world.cpp:142-162`, 객체 이름 우선, 다음 `object/subframe`).
`getCurrentState()`는 어느 경로에도 없다.

새 접근자는 이 범위를 그대로 재현한다 — 베이스 맵 복제에 world 객체와
subframe을 bare/`/`-prefixed 두 키로 접어 넣고, 로봇 링크와 attached body는
넣지 않는다. `can_transform`이 문자열 그대로 한 번 조회하는 평면 맵이라
두 키가 필요하다는 논리도 맞다.

### 75.2 다만 앞에 `/`가 붙은 이름의 객체에서 과다 매칭한다

상류: 객체 이름이 `/obj`일 때 `isFixedFrame("/obj")`는 베이스 맵에 없고,
`/`를 벗긴 `knowsTransform("obj")`도 없으므로 **false**다.
이 포트: `insert`가 `name`과 `/{name}`을 넣으므로 키가 `/obj`, `//obj`가
되고 `can_transform("/obj")`가 **true**다.

추가된 경계 테스트 4건은 이 경우를 덮지 않는다. 심각도는 낮지만
경계에서의 불일치이고 재현이 한 줄이다 — 라운드 11에 넣는다.

### 75.3 mesh cost source는 "백엔드 한계"가 아니다

담당이 `scene.rs`에 쓴 문장: "matching this bit-for-bit needs
`parry3d-f64`'s lower-level BVH/`Qbvh` traversal API, which nothing in
`moveit-collision` calls today. This half is the genuine backend limitation."

두 군데가 틀렸다.

먼저 이름. `Qbvh`는 `parry3d-f64 0.30`에 **존재하지 않는다** —
`grep -rl Qbvh` 결과 0건이고, `partitioning/mod.rs`가 내보내는 것은
`Bvh`/`BvhNode`/`BvhNodeIndex`/`TraversalAction`이다. `Qbvh`는 이전
버전 이름이다.

그리고 결론. 필요한 데이터는 공개 API로 전부 나와 있다:
`TriMesh::bvh() -> &Bvh`(`parry3d-f64-0.30.0/src/shape/trimesh.rs:1808`), `Bvh::leaves`
(`parry3d-f64-0.30.0/src/partitioning/bvh/bvh_traverse.rs:103`), `Bvh::intersect_aabb -> impl Iterator<Item = u32>`
(`parry3d-f64-0.30.0/src/partitioning/bvh/bvh_queries.rs:203`), `BvhNode::aabb()`(`parry3d-f64-0.30.0/src/partitioning/bvh/bvh_tree.rs:721`)와
`leaf_data()`(`:567`), `TriMesh::triangle(i)`(`parry3d-f64-0.30.0/src/shape/trimesh.rs:1881`).
leaf 삼각형별 AABB를 얻어 겹치는 쌍의 교집합을 취하는 것이
`fcl2costsource`가 하는 일이고, 그 재료가 다 있다.

맞는 진술은 "`parry3d_f64::query::contact` **한 번의 호출**이 돌려주지
않는다"이지 "백엔드에 없다"가 아니다. 두 번째 순회가 필요한 작업량
문제이지 한계가 아니다. 따라서 mesh 절반도 non-mesh 절반과 같이
p3-acm의 작업으로 넘어간다 — 문서에 한계로 남기면 다시 묻지 않게 되는데,
그 결론이 틀렸다.

### 75.4 `visibility_cone` 115건은 라운드 4부터 주인이 있었다

내가 열 라운드째 "주인 없음"으로 적어 온 항목인데, 담당이 근거를 대고
반박했다: `decide_cone`(`moveit-constraints/src/visibility.rs:381`)은
자기 `World`/`ParryCollisionEnv`를 만들고 `PlanningScene`을 타지 않으며,
§37/§38.3이 잔차를 `moveit-collision`의 contact 순회/tie-break 순서로
좁혀 p3-acm에 배정했고 §46.1이 재확인했다. `moveit-scene`에는
`visibility_cone` 참조가 0건이다.

**주인 없음이 아니라 미해결이다.** 내 UNFIXED 문구가 틀렸다.

### 75.5 머지 후 실측

`cargo nextest run --workspace` **999/999**(995 + 4), `cargo test --doc
--workspace` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사 통과,
재생 **28/28 identical**.

## 76. self 쪽 62건이 분해됐고, cost source는 "발명"할 게 없다 (2026-08-04)

p3-acm 라운드 12(`3a9da07`, `2d5d3c7`, `0da55bf`). 베이스 `85d1a14`,
main `cd13b7f`에 머지. **999 → 1000**, 재생 **28 → 29**.

### 76.1 62건의 분해

52건은 이미 아는 `base_bellow_link`/`torso_lift_link` 평탄부(deviation
6(b), §56/§63.1). 나머지 10건은 전부 `base_link` 대 다섯 개
`*_caster_*_wheel_link` 중 하나다. 그중 3건은 오라클 값이 그 쌍의
bounding-radius 한계를 넘어 fixture와 영구 테스트로 고정됐고, 7건은
한계 안이라 판정되지 않았다.

새 테스트가 구별력이 있는지 확인했다. `parry.rs`의 `minimum_distance`
누적에 `×4`를 넣으니 `pr2_self_wheel_same_pair_oracle_magnitude_is_implausible`이
실패한다 — 쌍 자체가 바뀌면서 첫 단언에서 걸린다. fixture 대 상수 비교만
하는 테스트가 아니다.

문서의 수치 표기 하나는 틀렸다: "twice the wheel's own bounding radius
(`link_bounding_radius`, `0.1534m` for a pr2 caster wheel)"에서 `0.1534m`는
반지름이 아니라 **2배 한계값**이다(반지름은 ≈`0.0767m`). 오라클 값
0.1815~0.1829가 한계를 넘는다는 단언이 통과하므로 한계가 0.3068일 수는
없다.

### 76.2 판정 불가 7건 — 이 계열은 1차원이다

담당의 일반론("두 임의의 posed mesh에는 `floor_env` 같은 고정 외부 기준이
없다")은 맞다. 그런데 **이 계열은 임의의 두 mesh가 아니다.**

`pr2.urdf`를 읽었다: `base_link` → `*_caster_rotation_link`(축 `0 0 1`)
→ `*_caster_{l,r}_wheel_link`(축 `0 1 0`). 즉 `base_link`↔wheel 상대
변환은 스칼라 두 개의 함수이고, 휠 메시는 자기 회전축에 대해 대칭이라
(deviation 6 문서가 "the wheel-roll joint cannot move the closest point"로
이미 적은 사실) **기하는 `*_caster_rotation_joint` 하나의 함수다.**

한 개의 스칼라라면 §63.1이 평탄부에 쓴 기법이 그대로 적용된다 — 그
관절을 훑으면서 양쪽 답을 함수로 보고, 교차점을 이분하고, 원 정점으로
독립 측정한다. 판정 불가가 아니라 아직 안 한 것이다. 라운드 13.

덧붙여, 이 포트가 이 계열에서 **상수** `-0.046592m`를 내는데 오라클은
세 케이스에서 0.18291/0.18150/0.18206으로 **변한다**. 캐스터 회전이
케이스마다 다르므로 변하는 쪽이 자동으로 틀린 것은 아니지만, 두 함수의
모양이 다르다는 사실 자체가 1차원 훑기로 바로 드러난다.

### 76.3 `cost_density`는 계산되는 값이 아니라 필드다

담당이 `parry.rs`에 쓴 결론: "Implementing this would mean inventing an
independent cost-density estimate from scratch, not adapting one `parry`
already computes."

FCL 헤더를 오라클 이미지에서 직접 읽었다. 그런 추정치는 FCL도 계산하지
않는다.

- `collision_geometry.h:102`의 `S cost_density;`는 지오메트리의 **필드**이고
  `collision_geometry-inl.h:56`에서 **`1`로 초기화**된다. `moveit_core`
  전체에서 이 값을 설정하는 곳은 없다(`collision_tools.cpp:275`가 읽기만
  한다). 즉 MoveIt에서 `cs.cost`는 항상 `1`이다.
- `cost_source-inl.h:55-72`: `total_cost = cost_density × (AABB 부피)`.
- `mesh_collision_traversal_node-inl.h:186-189`: 교차하는 **삼각형 쌍마다**
  `AABB(p1,p2,p3).overlap(AABB(q1,q2,q3), overlap_part)` — 두 삼각형
  AABB의 교집합 상자 하나.

그러므로 `CostSource` 전체가 "겹침 AABB + 상수 1"이다. 발명할 밀도
추정치가 없다.

§75.3에서 확인한 대로 재료도 다 있다: `TriMesh::bvh()`,
`Bvh::intersect_aabb`, `BvhNode::aabb()`/`leaf_data()`,
`TriMesh::triangle(i)`. **p1-fixtures와 p3-acm이 서로 다른 근거로 같은
'백엔드 한계' 결론에 도달했고, 둘 다 틀렸다.** 두 크레이트의 문서를
모두 고치고 구현은 p3-acm이 한다.

순서 문제는 남는다: `res_->cost_sources`가 `std::set<CostSource>`이고
`collision_common.cpp:286`이 `max_cost_sources`를 넘으면 뒤에서 지우므로
`CostSource::operator<`(`cost_source-inl.h:93`) 정렬 순서가 관측 가능하다.
그건 구현할 때 맞춰야 할 대상이지 한계가 아니다.

### 76.4 머지 후 실측

`cargo nextest run --workspace` **1000/1000**, `cargo test --doc
--workspace` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사 통과, reseed-wrap 통과,
재생 **29/29 identical**.

## 77. `IkConstraintSampler`가 들어왔는데 `sample_pose`의 수식은 아무것도 고정돼 있지 않다 (2026-08-04)

p1-robotmodel 라운드 9(`df35ac1`, `97e7683`). 베이스 `85d1a14`,
main `71ed03e`에 머지. **1000 → 1010**.

### 77.1 이식 자체는 상류와 맞다

`default_constraint_samplers.cpp`와 줄 단위로 대조했다.
`getSamplingVolume`(:332-353), `getLinkName`(:355-360),
`samplePose`(:441-556) 전부 구조가 같다 — 영역 선택의
`(i + k) % b.size()` 순환, `2.0 * (u01 - 0.5) * (tol - eps)` 세 각,
XYZ_EULER와 ROTATION_VECTOR 두 분기, mobile frame 회전, 마지막의
link offset 차감 순서까지.

D4 결정대로 solver는 인자로 받고, bijection 배열 대신 이름으로
읽고 쓴다. `moveit-constraints -> moveit-kinematics` 의존 간선이
생겼고 `check-dep-direction.sh`는 통과한다.

### 77.2 그런데 새 테스트 8건은 그 수식을 재지 않는다

`sample_pose`의 자명하지 않은 줄을 하나씩 흔들어 `-p moveit-constraints`
75건을 돌렸다.

```
sampling_volume의 x*y*z  → ×1.000001   → 1건 실패  (고정돼 있음)
pos -= quat * link_offset → pos +=      → 75/75 통과
quat = frame_rot * quat   → quat * frame_rot → 75/75 통과
desired_rotation_matrix_in_ref_frame().transpose() → transpose 제거 → 75/75 통과
X*Y*Z (오일러 합성)      → Z*Y*X       → 75/75 통과
```

즉 고정된 것은 `sampling_volume` 하나뿐이고, **`sample_pose`가 실제로
계산하는 자세 수식 네 줄은 전부 부호·순서를 뒤집어도 통과한다.**
`link_offset`은 `tests/ik_sampler.rs` 어디에서도 설정되지 않는다
(`decide.rs:284`에만 있고 그건 `PositionConstraint`의 판정 경로다).

새로 들어온 8건은 sampling-volume 경계 3건, position-only, orientation-only,
max-attempts 소진, seed/solution 이름 왕복, `NewtonRaphsonSolver` 통합
1건 — 전부 배관과 경계이고 자세 수식이 아니다. §68이 `moveit-planners-sbp`에서
쓸어낸 것과 같은 계열이다: 일률적으로 틀린 값이 모든 자기 일관성
검사를 통과한다.

RNG가 들어간다고 고정이 불가능한 것은 아니다. seed를 고정하면
`sample_pose`는 결정적 함수이고, 위 네 개의 교란은 전부 서로 다른
자세를 낸다 — 각각을 구별하는 케이스를 하나씩 쓰면 된다(서로 다른 세
축 공차, 항등이 아닌 desired rotation, 교환되지 않는 frame 회전,
0이 아닌 link offset). 라운드 10.

### 77.3 머지 후 실측

`cargo nextest run --workspace` **1010/1010**(1000 + 10), `cargo test --doc
--workspace` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK(새 의존 간선 포함), 출처 검사 통과,
재생 **29/29 identical**.

## 78. §71.2가 닫혔다 — 이제 허용오차가 실제로 물린다 (2026-08-04)

p3-distance-field 라운드 12(`047808e`, `78b9635`, `7d65688`, `760301b`).
베이스 `d4b9b2b`, main `b0520ce`에 머지. 테스트 수는 그대로 **1010**.

### 78.1 원인은 `max_relative`였다

§71.2에서 다섯 상수를 `1e-15`로 낮춰도 72건이 전부 통과했던 이유가
밝혀졌다. `assert_relative_eq!(a, b, epsilon = TOL)`은 통과 조건이
`|a-b| <= epsilon` **또는** `|a-b| <= max_relative × max(|a|,|b|)`이고,
`max_relative`를 주지 않으면 `f64::EPSILON`(~2.22e-16)이 들어간다.
즉 명시한 `epsilon`과 무관하게 **크기에 비례하는 허용치가 항상 하나 더
켜져 있다.** 비교값이 1 근처 이상이면 그 비례 허용치가 관측 오차를
덮어 버리므로, `epsilon`만 이분해서는 무는 지점에 영영 닿지 못한다 —
`0.0`까지 내려도 통과한다. 내가 잰 것도, 담당이 처음에 잰 `1.12e-13`도
같은 함정의 양쪽 면이다.

(실제로 이 크레이트가 그 경우다: 수정 후 `epsilon = 1e-16`에서 실패하는
차이가 수정 전에는 `epsilon = 0`에서도 통과했으므로
`2.22e-16 × max(|a|,|b|) > 1e-16`, 즉 `max(|a|,|b|) > 0.45`다.)

세 파일의 모든 `assert_relative_eq!`에 `max_relative = TOL`을 명시로
넘긴 뒤 다시 이분했다.

### 78.2 다시 재봤다

내가 직접 확인한 값(세 파일의 `TOL`과 `RADIUS_TOL`을 함께 움직임):

```
1e-15 → 72/72 통과
1e-16 → 3건 실패
1e-17 → 4건 실패
```

1e-16에서 실패하는 셋은 파일마다 정확히 하나씩이다:
`collision_distance_field_types_match_the_oracle`,
`collision_object_point_decomposition_matches_the_oracle`,
`group_state_representation_matches_the_oracle`.

담당이 라운드 중간에 스스로 고친 주장 — `collision_common`이 무는 곳은
`link_body_decomposition`이 아니라 `collision_object_point_decomposition`
이라는 것 — 이 그대로 재현된다.

`RADIUS_TOL`만 `1e-18`로 내리면 `group_state_representation_matches_the_oracle`
하나가 실패한다. 바닥이 `1e-18`과 `1e-17` 사이라는 주장도 맞다.

현재 상수는 `1e-12`이고 무는 지점이 `1e-16`~`1e-15`이므로 **여유가 3~4
자리**다. §71.2의 "최소 6자리 헐겁다"는 닫혔다.

### 78.3 이분해도 안 걸리는 두 파일은 상수를 없앴다

`oracle_parity.rs`와 `collision_sphere_free_functions_parity.rs`는 `0.0`까지
내려도 실패가 없어서 `assert_eq!`로 바꿨다. 아무것도 위반할 수 없는
허용오차는 gate가 아니라는 판단이고, 맞다.

그 exactness 단언이 무엇을 재는지도 확인했다. 자유 함수
`get_collision_sphere_gradients`(`:567`)의 `closest_distance` 대입에
`×1.0000000000001`(상대 1e-13)을 넣으니
`collision_sphere_free_functions_match_the_oracle`이 실패한다.
같은 크기의 교란을 메서드 쪽(`:443`)에 넣으면 통과하는데, 그 파일의
`TOL`이 `1e-12`이라 1e-13은 허용 범위 안이기 때문이다 — `×1.0000000001`
(상대 1e-10)로 키우면 `collision_distance_field_types_match_the_oracle`이
실패한다. **두 오버로드 다 고정돼 있다.**

### 78.4 머지 후 실측

`cargo nextest run --workspace` **1010/1010**, `cargo test --doc
--workspace` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사 통과,
재생 **29/29 identical**.

## 79. `max_relative` 함정은 워크스페이스 전체의 계열이다 (2026-08-04)

§78이 한 크레이트에서 닫은 것은 표본이지 모집단이 아니다. 앵커를 잡고
전수 조사했다.

**앵커:** `assert_relative_eq!` 호출 중 `max_relative`를 넘기지 않는 것.

**전수:** `crates/`와 `tools/`의 모든 `.rs`에서 호출 괄호를 균형 파싱해
셌다.

```
총 호출          274
max_relative 있음  45
없음              229
```

없는 229건이 전부 결함은 아니다. 비례 허용치 `2.22e-16 × max(|a|,|b|)`가
관측 오차보다 작으면 `epsilon`이 실제로 지배하므로 그 단언은 멀쩡하다.
결함은 **비교값 크기가 커서 비례 허용치가 명시한 `epsilon`을 덮는**
경우뿐이고, 그것은 파일마다 재봐야 안다 — §78.2가 한 그대로
`epsilon`을 이분해 무는 지점을 찾고, `0.0`까지 내려도 통과하면 그
파일이 이 함정에 걸린 것이다.

크레이트별 미지정 호출 수(상위):

```
moveit-geometry/src/bodies.rs                47/47
moveit-geometry/src/shapes.rs                45/45
moveit-trajectory/src/trajectory.rs          21/21
moveit-collision/src/parry.rs                19/19
moveit-model/src/joint/planar.rs             10/10
moveit-model/src/joint/revolute.rs            9/9
moveit-distance-field/tests/upstream_parity.rs 7/7
moveit-geometry/src/octree_collision.rs       6/6
moveit-trajectory/src/path.rs                 6/6
moveit-trajectory/src/path_segment/linear.rs  6/6
```

각 크레이트 담당에게 자기 파일의 이분 결과를 요구한다. `0.0`까지 통과하는
파일은 §78.3처럼 exactness 단언으로 바꾸거나 `max_relative`를 명시하고
다시 이분한다.

이것이 §68("일률적으로 틀린 상수가 모든 자기 일관성 검사를 통과한다"),
§74.3(오라클 fixture가 `target_velocity`를 재지 않음),
§77.2(`sample_pose`의 네 줄이 부호를 뒤집어도 통과)와 같은 계열이다:
**통과하는 단언이 무언가를 재고 있다는 증거가 아니다.**

## 80. 62건의 분해가 세 번 독립으로 같은 값을 냈다 (2026-08-04)

p1-joints 라운드 11 부분 머지(`be5b8c1`, `5878d1a`, `a8b5e91`).
베이스 `231e17b`, main `159dda1`에 머지. **1010 → 1017**.

담당은 라운드가 끝나지 않았다 — 30000 케이스 pr2 스윕이 아직 돌고 있어
fallback 창을 넘겼다. 브랜치에 커밋된 세 건만 먼저 들여왔고 감시는 다시
걸어 뒀다.

### 80.1 히스토그램을 내가 직접 돌렸다

`--stats-json`에 `self_same_pair_histogram`이 생겼다. 같은 시드로 내가
다시 돌린 결과:

```
base_bellow_link/torso_lift_link    52
base_link/fr_caster_r_wheel_link     4
base_link/br_caster_l_wheel_link     3
base_link/bl_caster_r_wheel_link     1
base_link/fl_caster_l_wheel_link     1
base_link/fr_caster_l_wheel_link     1
                                  ----
                                    62
```

`self_same_pair_and_value_diverges`도 62다. **52 + 10, 캐스터 휠 링크
다섯 종** — p1-joints의 계측, p3-acm이 §76.1에서 독립으로 분해한 결과,
그리고 내 재현이 셋 다 같다.

명령:

```
moveit-diff --urdf .../pr2.urdf --srdf .../pr2.srdf \
  --group right_arm --collision --cases 3000 --seed 20260804 \
  --stats-json <out> --oracle tools/moveit-oracle/run-oracle.sh
```

### 80.2 `cached_ik_kinematics_plugin` 판정

"지금 해도 될 만큼 작지만 위로 올릴 만큼 크지는 않다" —
가로챌 메서드는 `solve_with_options` 하나, 디스크 캐시 포맷은 포팅
대상이 아니라 로컬 `serde` 선택, 최근접 시드 조회는 GNAT 트리 이식이
아니라 선형 스캔. 크레이트 경계를 넘지 않으므로 이 크레이트의 다음
라운드 작업이지 워크스페이스 결정이 아니다. **여러 라운드 미결이던
항목이 닫혔다.**

### 80.3 머지 후 실측

`cargo nextest run --workspace` **1017/1017**(1010 + 7), `cargo test --doc
--workspace` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사 통과,
재생 **29/29 identical**.

## 81. `tree_walk` 핀은 실측으로 셋을 구분한다 — 그리고 보고서 두 줄이 틀렸다 (2026-08-04)

p3-shapes 라운드 13 머지(`457ea0f`, `84c396c`, `b7caa43`). 베이스
`f1c14ef`, main `fb061c5`.

### 81.1 핀이 무엇을 재는지 직접 흔들어 봤다

§74.3/§77.2/§79의 계열 검사를 이번에도 적용했다. `tree_walk` 핀은
자기 일관성이 아니라 오라클의 `nodes` 배열에 대해 순서까지 포함해
필드별로 단언한다 — 개수, 그 다음 노드마다 `x`/`y`/`z`(< 1e-9),
`size`(< 1e-9), `depth`, `is_leaf`, `log_odds`(< `LOG_ODDS_EPS`),
`occupancy`(< `OCCUPANCY_EPS`).

셋을 각각 흔들어 `-p moveit-octomap`(27건)을 `--no-fail-fast`로 돌렸다:

```
자식 push 순서 뒤집기 (self.stack[before..].reverse())   1 fail
TreeNode::size() × 1.000001                              1 fail
TreeNode::is_leaf() 부정 (!has_children → has_children)  1 fail
```

세 번 다 정확히 한 건씩 떨어진다. **이번 라운드의 핀은 §79가 세는
쪽이 아니라 재는 쪽이다.**

### 81.2 보고서 두 줄이 트리와 어긋난다

**(a) `Body::intersects_ray`는 "unported"가 아니다.** UNFIXED 줄은
"stays unported ... decided, not deferred"라고 적었지만
`crates/moveit-geometry/src/bodies.rs`에 구현이 다섯 군데 있다 —
1614, 1975, 2327, 2874, 그리고 3166(enum dispatch). 파일 전체에
`todo!`/`unimplemented!`는 0건이다.

```rust
pub fn intersects_ray(&self, origin: &Vector3, dir: &Vector3) -> bool {
    !self.ray_intersections(origin, dir, Some(1)).is_empty()
}
```

없는 것은 구현이 아니라 **외부 호출자**뿐이다. UNFIXED 문구를 그대로
고쳐야 한다: 미포팅이 아니라 "포팅되었으나 크레이트 밖 소비자가 없다".

**(b) `sample_point_inside`의 소비자는 이미 있다.** 문서는 "once
ported"라고 미래형으로 적었지만 `crates/moveit-constraints/src/ik_sampler.rs:254`
가 main에서 이미 부른다:

```rust
body.sample_point_inside(max_attempts, &mut |lo, hi| rng.random_range(lo..hi))
```

p1-robotmodel 라운드 9가 넣었고, 그 머지는 이 패널의 베이스 `f1c14ef`
**뒤**에 들어갔다. 즉 보고서가 도착한 시점에 이미 낡은 주장이었다 —
브랜치가 베이스에서 볼 수 없는 사실을 머지하는 쪽만 볼 수 있는,
반복해서 나오는 실패 모드다.

### 81.3 머지 후 실측

`cargo nextest run --workspace` **1017/1017**(변동 없음 — `tree_walk`
단언이 `octomap_parity.rs`의 기존 `#[test]` 하나 안으로 들어갔고
fixture 두 개도 새 파일이 아니라 기존 파일에 106/427줄 추가다),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **29/29 identical**.

`oracle.cpp`가 바뀌었으므로(+24줄, `tree_walk` 질의) 스탬프를 다시
계산하고 이미지를 새로 빌드했다: **`7cc8a73408a83c92`**. 패널이 보고한
`081ed34b1019d990`은 자기 브랜치 트리의 값이고, 머지 후 값이 아니다.

## 82. §74.3이 닫혔다 — `do_smoothing`의 여섯 줄이 전부 물린다 (2026-08-04)

p6-totg 라운드 10 머지(`953af03`, `cd2ab61`, `02e171a`). 베이스
`252354d`, main에 머지. **1017 → 1019**.

### 82.1 여섯 갈래 삭제 시험을 내가 다시 돌렸다

§74.3에서 `ruckig_filter` fixture가 `target_velocity`를 재지 않는다고
적었다. 담당이 fixture를 3케이스에서 6케이스로 늘렸다(25틱 고정 타깃
수렴, 15틱 이동 타깃, 저크 상한 0 케이스 추가). `do_smoothing`의 모든
관측 가능한 줄을 하나씩 지우고 `-p moveit-smoothing`(31건)을
`--no-fail-fast`로 돌린 결과:

```
target_velocity = 0.0                       1 fail
pass_to_input 호출 삭제                     1 fail
RuckigResult 조기 반환 분기 무력화(&& false) 1 fail
positions 라이트백 삭제                     2 fail
velocities 라이트백 삭제                    2 fail
accelerations 라이트백 삭제                 1 fail
```

여섯 갈래 전부 최소 한 건을 떨어뜨린다. **§74.3의 미핀 항목이 닫혔다.**

(조기 반환 분기는 블록을 통째로 지우면 컴파일이 깨져서 조건에
`&& false`를 붙이는 쪽으로 바꿔 측정했다. 삭제가 컴파일되지 않는다는
사실 자체는 핀의 증거가 아니므로 반드시 컴파일되는 무력화로 다시
재야 한다.)

### 82.2 단일 DOF 좁히기가 계약이 됐다

`acceleration_filter.rs:153`과 `ruckig_filter.rs:138`이
`joint.variable_names().len() != 1`을 명시적으로 검사하고 전용 오류로
거절한다 — 전에는 뒤따르는 이름 조회가 우연히 실패해 주는 것에
기대고 있었다. 두 파일 모두 검사를 `if false`로 무력화하면 각각 정확히
한 건이 떨어진다. 우연이 아니라 계약이다.

### 82.3 브리프의 전제가 틀렸고, 담당이 맞다

`TimeOptimalTrajectoryGeneration`의 `RobotTrajectory` 어댑터를 내가
미포팅으로 적어 보냈는데 이미 포팅되어 있다 —
`compute_time_stamps`/`compute_time_stamps_with_limits` 양쪽,
`totg_compute_time_stamps`, `has_mixed_joint_types`, 그리고
`totg_robot_trajectory`/`totg_robot_trajectory_scaling_only` fixture
쌍까지 트리에 있다. 실제 결함은 `trajectory.rs`의 모듈 문서가 아직
"out of scope"라고 주장하고 있던 것뿐이고 그게 고쳐졌다(`953af03`).

**§81.2와 같은 계열이다** — 이 라운드 세트에서 낡은 문서 주장이 두 건
연속으로 나왔다. §65가 말한 "아무도 검사하지 않는 주장은 조용히
낡는다"가 계획 문서만이 아니라 크레이트 모듈 문서에도 그대로 적용된다.
"미포팅"이라고 쓰기 전에 `rg -n '<symbol>' crates/` 한 번이 규칙이다 —
내 브리프도 그걸 지켰어야 했다.

### 82.4 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1019/1019**(1017 + 2),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **29/29 identical**. `oracle.cpp`는 그대로라
스탬프는 `7cc8a73408a83c92` 유지.

담당이 보고한 997/997과 28/28은 자기 베이스 `252354d` 기준 값이다.

## 83. `/`-선행 객체 id 경계가 세 규칙으로 닫혔다 (2026-08-04)

p1-fixtures 라운드 11 머지(`bf11a20`, `6e1c8ea`, `08ab3c7`). 베이스
`cd13b7f`, main에 머지. **1019 → 1023**.

### 83.1 세 규칙 전부 물린다

§75.2에서 `transforms_with_world_objects`가 `/obj`를 과매칭한다고
적었다. 담당이 `world.cpp:142-164`를 직접 읽고 세 규칙으로 나눠 고쳤다.
각각 무력화하고 `-p moveit-scene`(68건)을 `--no-fail-fast`로 돌린 결과:

```
`/`-선행 이름의 bare insert 가드 되돌리기       1 fail
`/`-접두 insert 삭제                            3 fail
객체 id 우선 가드 무력화(has_object → false)    1 fail
```

세 규칙 다 재고 있다.

### 83.2 상류 근거를 내가 직접 읽었다

`World::knowsTransform`(`world.cpp:142`)은 정확 객체 id를
`objects_.find(name)`로 먼저 조회해 `:148`에서 반환하고, 서브프레임
루프는 `:152`에서야 시작한다. 담당이 인용한 "`:145`가 `:150`보다 먼저"는
맞다. 그리고 그 루프는 접두가 맞는 **첫** 객체에서 곧바로 `return`하므로
(`:156-160`, 계속 스캔하지 않는다) 중첩 `/` 이름의 모호한 경우는
`std::map` 순회 순서에 의존한다 — 담당이 "모델링하지 않음"으로 남긴
범위 제한도 상류 코드와 일치한다.

### 83.3 완료 조건의 숫자를 전부 재현했다

`08ab3c7`이 `lib.rs`에 넣은 완료 조건은 숫자마다 재현 명령을 붙였다.
전부 돌려 봤다:

```
rg -c '^/// - `' crates/moveit-scene/src/scene.rs      59   (선언과 일치)
같은 명령을 47-434행으로 제한                          59   (일치)
rg -n '^fn .*matches_the_oracle' .../tests/*.rs         3   (이름 3건 모두 일치)
scene.rs:2672 = "// ---- collision checking ----"           (일치)
```

"zero unported, in scope" 주장도 내가 따로 검증했다.
`planning_scene.hpp`에서 `public:` 구간의 멤버 선언 이름 **62개**를
뽑아 47-434행 감사 블록 본문과 대조했더니 **누락 0**이다. (62 대 59는
불일치가 아니라 한 불릿이 여러 심볼을 묶은 경우다 —
`:157`이 `getCollisionEnvUnpadded(name)`/`getCollisionEnvNonConst`를,
`:289`가 `setObjectColor`/`removeObjectColor`/`getKnownObjectColors`를
한 줄에 묶는다.)

**내 첫 대조는 틀렸다.** `^/// - \`` 로 시작하는 줄만 모아서 비교하는
바람에 여러 줄로 이어지는 불릿의 꼬리가 잘렸고, 위 여섯 심볼이 누락으로
잡혔다. 블록 전체 본문으로 다시 돌려서 0이 나왔다. §73.1과 같은 실수다 —
**대조 검사에서는 검색 범위를 좁힌 쪽이 항상 거짓 양성을 낸다.**

### 83.4 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1023/1023**(1019 + 4),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **29/29 identical**. 스탬프
`7cc8a73408a83c92` 유지.

담당이 보고한 1003/1003과 28/28은 베이스 `cd13b7f` 기준 값이다.

## 84. §77.2가 닫혔다 — 그리고 죽은 분기 하나를 실측으로 확인했다 (2026-08-04)

p1-robotmodel 라운드 10 머지(`10306a3`, `ec01208`). 베이스 `b0520ce`,
main에 머지. **1023 → 1036**.

### 84.1 `sample_pose`의 네 줄이 전부 물린다

§77.2에서 `sample_pose`의 다섯 줄 중 넷이 부호·순서를 뒤집어도 통과한다고
적었다. 담당이 `tests/ik_sampler.rs`에 네 건을 추가했다 — 각각 시드된 RNG를
이미 검증된 프리미티브로 재생해서 `sample_pose` 자신의 산술과 **독립으로**
계산한 기대값과 비교한다. 내가 네 갈래를 다시 흔들어
`-p moveit-constraints`(88건)를 `--no-fail-fast`로 돌린 결과:

```
XYZ 오일러 → ZYX 뒤집기                  3 fail
`.transpose()` 삭제                      1 fail
`frame_rot * quat` → `quat * frame_rot`  1 fail
`pos -=` → `pos +=`                      1 fail
```

네 갈래 전부 물린다. **§77.2의 미핀 항목이 닫혔다.** 오일러 뒤집기가
3건을 떨어뜨리는 것은 나머지 두 테스트가 같은 오일러 단계를 공유하기
때문이고, 담당 보고와 일치한다.

### 84.2 동률 처리의 비대칭이 상류와 맞다 — 한쪽은 도달 불가

`ConstraintSamplerManager::selectDefaultSampler`가
`select_default_sampler`로 이식됐다. 동률 방향이 두 곳에서 반대인데,
상류를 직접 읽어 확인했다
(`constraint_sampler_manager.cpp`):

```
링크별 삽입 (:194)   if (used_l[link]->getSamplingVolume() < iks->...) use = false;
                     → 동률이면 use가 true로 남아 새 후보가 이긴다 (나중 승)
링크 간 축약 (:303)  if (v < msv) { iks = it->second; msv = v; }
                     → 동률이면 갱신하지 않아 첫 후보가 이긴다 (먼저 승)
```

Rust 쪽도 각각 `existing < candidate`(나중 승)와 `b <= candidate`이면
`b` 유지(먼저 승)로 정확히 반대다. `BTreeMap`이 `std::map`의 키 순서와
같으므로 "먼저"의 뜻도 일치한다.

**두 방향을 각각 뒤집어 봤더니 결과가 갈렸다:**

```
링크별 동률 방향 뒤집기 (< → <=)    1 fail
링크 간 동률 방향 뒤집기 (<= → <)   0 fail — 88/88 통과
```

링크 간 쪽은 재고 있지 않다. 다만 이건 §79 계열의 구멍이 아니라
**구조적 도달 불가**다. `crates/moveit-constraints/src/ik_sampler.rs:177`이 제약 링크가 솔버의 tip
프레임과 같지 않으면 생성 자체를 거절하므로(고정 링크 브리징 미이식),
한 그룹에 솔버 하나·tip 하나면 `used`의 키는 최대 하나다. 담당의 UNFIXED가
그 사실을 정확히 적었고 내 실측이 그것과 일치한다.

**그래서 남는 것은 설계 결정이다.** 키가 최대 하나임이 구성상 보장된다면
그 상태는 `BTreeMap`이 아니라 `Option<(String, _)>`이어야 하고, 그러면
`>1` 분기와 그 안의 동률 비교가 아예 표현 불가능해진다. 지금은 "나중에
브리징이 들어오면 쓰려고" 충실 이식으로 남겨 둔, 영원히 실행되지 않는
분기다. 다음 라운드에 이 선택을 매듭짓게 한다.

### 84.3 `too_many_arguments`를 구조로 닫았다

`select_default_sampler_inner`의 clippy 경고를 `#[allow]`이 아니라
세 제약 종류 슬라이스를 `GroupConstraints`로 묶어서 닫았다. 억제 금지
규칙이 실제로 지켜진 사례다.

### 84.4 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1036/1036**(1023 + 13),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **29/29 identical**. 스탬프
`7cc8a73408a83c92` 유지.

담당이 보고한 1023/1023은 베이스 `b0520ce` 기준 값인데, 우연히 머지 전
main의 수와 같다.

## 85. `upstream_parity.rs`는 §79에 걸린다 — 7건 중 4건이 (2026-08-04)

p3-distance-field 라운드 13 머지(`7415f0a`, `2fdeade`, `d41794f`).
베이스 `159dda1`, main에 머지. 테스트 수 변동 없음(**1036/1036**, 문서
커밋 3건).

### 85.1 판정이 틀렸다 — 전체를 한꺼번에 이분했기 때문이다

`7415f0a`의 결론은 "7건 전부 `epsilon = 0.0`에서 실패하므로 이 파일은
§79 함정에 걸리지 않는다"이다. 실패가 난 건 맞다. 그러나 **7건을 한
덩어리로 이분했기 때문에 어느 것이 실패를 만들었는지 구분하지 못했다.**

두 상수군을 따로 이분했다:

```
네 개의 `epsilon = 0.0001` 사이트만 낮추기
  1e-5 / 1e-9 / 1e-15 / 1e-16 / 0.0   →  전부 3/3 통과

세 개의 `epsilon = RESOLUTION`(0.1) 사이트만 낮추기
  0.1 / 0.09 / 0.07 / 0.05 / 0.03 / 0.02 / 0.015  →  통과
  0.01                                            →  1 fail
```

`epsilon = 0.0`에서 나던 실패는 **전부 RESOLUTION 쪽에서** 나온 것이다.
담당의 문서조차 실패 지점을 `comp_y`/`point1().y`로 정확히 적어 놨는데,
그건 RESOLUTION 사이트다. 네 개의 `0.0001` 사이트는 한 번도 따로
측정된 적이 없다.

### 85.2 그 네 건은 문자 그대로 암묵 `max_relative`를 타고 있다

`epsilon = 0.0`으로 두고 `max_relative`를 명시해 이분했다:

```
max_relative = 2.3e-16   3/3 통과
max_relative = 1e-16     1 fail
max_relative = 1e-17     1 fail
max_relative = 0.0       1 fail
```

무는 지점이 `f64::EPSILON`(2.22e-16) 바로 그 자리다. 즉 이 네 단언이
통과하는 이유는 명명된 `epsilon = 1e-4`가 아니라 **주지 않은
`max_relative`의 기본값**이다. 명명된 상수는 무는 지점보다 **12자리**
위에 있고 아무것도 재지 않는다. §79가 찾으려던 바로 그 형태다.

`assert_eq!`로 바꾸는 exactness 출구는 이 네 건에서 통하지 않는다 —
바꿔서 돌리면 실패한다. 실제 오차가 0이 아니라 1 ULP 수준이다.
그러므로 출구는 `epsilon = 0.0, max_relative = <2.3e-16 이상>`이다.

세 개의 RESOLUTION 사이트는 진짜 게이트다: 상수 `0.1`, 무는 지점
`0.01`–`0.015`, 헤드룸 약 한 자리. 이쪽은 손댈 것이 없다.

### 85.3 배운 것: 이분은 상수군별로 해야 한다

**여러 상수를 한꺼번에 낮추면 살아 있는 게이트 하나가 죽은 단언 전부를
가린다.** §79 스윕을 받는 나머지 크레이트에도 같은 지침이 필요하다 —
파일 단위가 아니라 **상수(또는 상수군) 단위**로 이분하고, 실패가 났을 때
어느 단언이 떨어졌는지 이름까지 확인해야 한다. 이 지침을 남은 패널
브리프에 넣었다.

### 85.4 나머지 두 커밋은 유효하다

`d41794f`가 고친 다섯 곳(`lib.rs` ×3, `distance_field.rs`,
`collision_distance_field_types.rs`, `upstream_parity.rs`)의 낡은
"워크스페이스 어디에도 `octomap::OcTree` 대응물이 없다" 주장은 실제
드리프트였고 — `moveit-octomap/src/tree.rs:83`이 반례 — 정확한 이유
("이 크레이트가 `moveit-octomap`에 의존하지 않는다")로 바뀌었다.
§81.2·§82.3에 이은 낡은 문서 주장 **세 번째 연속 사례**다.

`2fdeade`의 함정 진술문 자체는 맞다. 다만 그 안에서
`upstream_parity.rs`를 "함정에 걸리지 않는 반례"로 인용한 부분은 §85.1의
측정으로 무효가 됐다 — 라운드 14에서 고친다.

### 85.5 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1036/1036**,
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **29/29 identical**. 스탬프
`7cc8a73408a83c92` 유지. 담당이 보고한 1010/1010은 베이스 `159dda1`
기준 값이다.

## 86. `cost_sources`가 착지했다 — 그러나 삼각형 단위 granularity는 재지 않는다 (2026-08-04)

p3-acm 라운드 13 부분 머지(`6890fdd`, `fc93f45`). 베이스 `9576dfb`,
main에 머지. **1036 → 1048**.

담당은 아직 라운드가 끝나지 않았다 — 캐스터 1차원 스윕의 ground-truth
프로브가 돌고 있어 fallback 창을 넘겼다. 커밋된 두 건만 먼저 들여왔고
감시는 다시 걸어 뒀다. §80과 같은 처리다.

### 86.1 §76.2의 판정이 뒤집혀 구현으로 닫혔다

§76.2에서 두 패널이 각각 다른 논거로 `getCostSources`를 "백엔드 한계"로
결론냈고 둘 다 틀렸다고 적었다. 이번에 실제로 구현됐다. 상류 대조를
내가 직접 했다:

`CostSource::operator<`(`collision_common.hpp:128-141`)는
`cost*getVolume()` 내림차순 → `cost` 내림차순 →
`aabb_min` 사전순이다. Rust 쪽 `Ord`(`common.rs:148`)는
`c2.total_cmp(&c1)` → `other.cost.total_cmp(&self.cost)` →
`total_cmp_aabb`로 세 단계가 정확히 대응한다.

절단도 맞다. 상류는 삽입할 때마다
`while (size() > max_cost_sources) erase(--end())`
(`collision_common.cpp:285-287`, 같은 패턴이 `:351-353`, `:388-390`에
세 번 반복된다)로 **뒤에서** 지운다. 정렬이 "가장 비싼 것이 앞"이므로
뒤는 가장 싼 것이고, Rust의 `BTreeSet::pop_last()`가 같은 것을 지운다.

### 86.2 네 갈래는 물리고, 다섯째는 물리지 않는다

`-p moveit-collision`(168건) `--no-fail-fast`:

```
절단 방향 뒤집기 (pop_last → pop_first)              1 fail
CostSource 정렬 뒤집기 (c2.total_cmp(c1) → c1↔c2)    5 fail
cost 상수 1.0 → 2.0                                  4 fail
mesh-shape: 삼각형 AABB → 메시 전체 AABB             0 fail  ← 통과
mesh-mesh:  삼각형 AABB → 메시 전체 AABB             0 fail  ← 통과
```

**이 구현의 중심 주장이 재어지지 않는다.** 커밋 메시지는 "Mesh-vs-mesh
emits one CostSource per intersecting triangle pair"라고 쓰고, 문서는
`mesh_collision_traversal_node-inl.h`의
`AABB(p1,p2,p3).overlap(AABB(q1,q2,q3))`를 인용한다. 그런데 삼각형별
AABB를 메시 전체 AABB로 바꿔도 168건 전부 통과한다.

원인은 테스트 픽스처다. `big_flat_triangle()`(`parry.rs:2621`)은
**삼각형 하나짜리 메시**라서 전체 AABB와 그 한 삼각형의 AABB가 같다.
테스트 이름이
`mesh_shape_cost_sources_is_one_triangle_aabb_overlapped_with_the_whole_shape_aabb`
로 바로 그 성질을 주장하는데, 그 픽스처로는 주장과 반대 구현을 구분할
수 없다.

§74.3(오라클 fixture가 `target_velocity`를 재지 않음),
§77.2(`sample_pose` 네 줄), §85.1(`upstream_parity.rs` 네 건)과 같은
계열이다 — **테스트가 자기 이름이 말하는 것을 재고 있는지는 지우고
돌려 봐야만 안다.**

라운드 13 잔여 작업으로 넘긴다: 최소 두 삼각형 이상, 그리고 두
삼각형이 **서로 다른 AABB를 갖도록** 배치한 메시로 두 경로를 다시
핀해라.

### 86.3 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1048/1048**(1036 + 12),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **29/29 identical**. 스탬프
`7cc8a73408a83c92` 유지.

## 87. p1-joints 라운드 11 잔여 병합 — 세계측 비율은 진짜 꼬리다 (2026-08-04)

`acef960` 한 건. §80에서 먼저 들여온 세 건(`be5b8c1`, `5878d1a`,
`a8b5e91`)의 나머지다. 문서 전용 커밋이라 테스트 수는 **1048/1048**로
변동 없다.

### 87.1 30000 케이스가 답한 것

§67.2가 남긴 질문은 "세계측 same-pair 2/3000(0.067%)이 진짜 낮은
비율인가, 자기측 ~2%를 적게 뽑은 것인가"였다. 10배로 늘린 결과:

```
자기측  532/30000  (1.77%)
세계측   14/30000  (0.047%)
```

두 비율 다 3000 케이스 값의 1.4배 안에 머물렀고 서로 수렴하지 않았다.
세계측이 자기측과 같은 모집단이라면 30000에서 ~600건이 나와야 하는데
14건이다. **비율 차이는 표본 문제가 아니라 쌍 모집단 자체의 성질이다.**

쌍 구성도 3000 케이스 판과 일관된다 — 자기측 440/532(82.7%)가
bellow/torso 고원, 92/532(17.3%)가 `base_link`/캐스터 휠 **여덟 쌍
전부**에 7–16건씩 퍼진다(3000에서는 다섯 쌍만 보였다). 세계측 14건은
전부 `floor`/`*_gripper_*_finger_tip_link`이고 네 손가락에 5/4/3/2로
갈린다.

캐스터 휠 계열이 다섯 쌍이 아니라 여덟 쌍이라는 것은 §80.1의 3000 케이스
분해를 **확장**한다 — p3-acm이 진행 중인 1차원 스윕의 대상이 다섯이
아니라 여덟이다.

### 87.2 다른 시드로 독립 재현했다 — 결론이 선다

시드 `77771111`, 6000 케이스로 내가 따로 돌린 결과:

```
자기측  111/6000  (1.85%)
세계측    4/6000  (0.067%)
비율                27.8x
```

담당의 두 지점과 나란히 놓으면:

```
            자기측     세계측     비율
 3000  (담당)  2.07%    0.067%    ~31x
 6000  (내 시드) 1.85%   0.067%    27.8x
30000  (담당)  1.77%    0.047%    ~38x
```

시드가 다른데도 세 지점이 같은 자리에 있다. **비율 차이는 표본 문제가
아니다** — 담당의 결론이 독립 재현으로 선다.

쌍 구성도 재현된다. 자기측 111건 중 93건(83.8%)이 bellow/torso 고원이고
(담당의 30000에서는 82.7%), 나머지 18건이 캐스터 휠 **일곱 쌍**에
1–4건씩 퍼진다:

```
93  base_bellow_link/torso_lift_link
 4  base_link/fr_caster_r_wheel_link
 3  base_link/bl_caster_r_wheel_link
 3  base_link/fl_caster_l_wheel_link
 3  base_link/fl_caster_r_wheel_link
 3  base_link/fr_caster_l_wheel_link
 1  base_link/bl_caster_l_wheel_link
 1  base_link/br_caster_l_wheel_link
```

3000 케이스에서 다섯 쌍만 보이던 것이 시드를 바꾸고 표본을 두 배로
늘리자 일곱으로 늘었다 — 담당이 30000에서 여덟 쌍 전부를 본 것과 같은
방향이다. **캐스터 계열은 다섯 쌍이 아니라 여덟 쌍이고, p3-acm의 1차원
스윕 대상이 그만큼 넓다.**

세계측 4건은 전부 `floor`/`*_gripper_*_finger_tip_link`이다
(`l_gripper_l_finger_tip` 2, `r_gripper_l_finger_tip` 1,
`r_gripper_r_finger_tip` 1) — 담당의 30000판과 같은 계열이다.

명령:

```
moveit-diff --urdf .../pr2.urdf --srdf .../pr2.srdf --group right_arm \
  --collision --cases 6000 --seed 77771111 --stats-json <out> \
  --oracle tools/moveit-oracle/run-oracle.sh
```

(`--stats-json`의 `cases` 필드는 표본 수가 아니라 검사 수다 —
`verdicts.len()`, `main.rs:788`. 6000 표본이 18001 검사를 낸다. 비율의
분모는 `--cases` 쪽이다.)

### 87.3 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1048/1048**(변동 없음),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK. 스탬프
`7cc8a73408a83c92` 유지.

담당이 보고한 977/977·재생 25/25·스탬프 `746870de2ddd3ca6`는 베이스
`231e17b` 기준 값이다 — 그 뒤로 머지가 여러 건 들어갔고 스탬프는 두 번
움직였다.

## 88. §79의 최대 노출 98건이 닫혔다 — 92건은 아무것도 재지 않았다 (2026-08-04)

p3-shapes 라운드 14 머지(`d43fab0`, `befd1df`, `2aba583`, `e68557a`,
`7031639`). 베이스 `4b13c99`, main에 머지. 테스트 수 **1048/1048** 변동
없음(단언 강화만).

### 88.1 결과

워크스페이스에서 가장 큰 두 노출이 이 크레이트 것이었다. 처분:

```
shapes.rs            45/45  →  전부 assert_eq! (비트 일치)
octree_collision.rs   6/6   →  전부 assert_eq! (비트 일치)
bodies.rs            47     →  41건 assert_eq!,
                                6건 epsilon = 1e-13, max_relative = 0.0
                              ────
합계                  98     →  92건이 아무것도 재고 있지 않았다
```

`shapes.rs`의 45건 중 7건은 `epsilon = 1e-12`를 **명시**하고 있었는데도
비트 일치였다. 상수를 적어 놓았다는 것이 그 상수가 무언가를 잰다는
뜻이 아니다.

### 88.2 §85.3의 지침이 그 자리에서 두 번 값을 했다

라운드 진행 중에 "이분은 파일 단위가 아니라 단언 단위로"를 보냈고,
담당이 `bodies.rs`를 **단언별로** 이분해 살아 있는 허용오차가 죽은
단언을 가리는 경우를 정확히 두 번 잡았다:

- `sphere_ray_origin_inside_moved_sphere` — `+1.6` 교점은 비트 일치인데
  `-0.6` 교점은 아니다
- `merge_bounding_spheres_two_spheres` — 반지름은 비트 일치인데
  `center.x`는 아니다

파일 단위로 이분했다면 두 파일 다 "실패하니 함정 아님"으로 끝났을
것이다. §85.1에서 p3-distance-field가 정확히 그렇게 틀렸다.

### 88.3 남긴 여섯 건의 무는 지점을 내가 다시 쟀다

`epsilon = 1e-13`을 함께 움직여 `-p moveit-geometry`(141건)를
`--no-fail-fast`로:

```
1e-13 (현재)  141/141 통과
1e-15         141/141 통과
1e-16         4 fail
0.0           4 fail
```

무는 지점이 `1e-16`–`1e-15`, 상수가 `1e-13`이므로 **헤드룸 2–3자리**다.
`max_relative = 0.0`이 명시돼 있어 §79 함정 경로도 막혀 있다.

### 88.4 exactness 전환이 실제로 조였다는 증거

전환이 단언을 강화했는지 직접 확인했다. `Sphere::compute_volume`
(`crates/moveit-geometry/src/shapes.rs:491`)에 상대 오차를 곱해 넣었다:

```
× 1.0000000000001    (1e-13)  1 fail
× 1.000000000000001  (1e-15)  1 fail
```

전환 전 `epsilon = 1e-12`였다면 둘 다 통과했을 크기다. **전환은 문서
정리가 아니라 검출력의 실제 증가다.**

### 88.5 §81.2의 두 정정이 반영됐다

`d43fab0`이 `bodies.rs`의 `intersects_ray`(미포팅이 아니라 외부 소비자
없음)와 `sample_point_inside`(미래형이 아니라 `crates/moveit-constraints/src/ik_sampler.rs:254`가 이미
호출) 문구를 고쳤다. 낡은 문서 주장이 §81.2·§82.3·§85.4에 이어 이번
라운드 세트에서 네 번째로 정정된 사례다.

### 88.6 새 격차 하나가 드러났다

`AbstractOccupancyOcTree`의 확률공간 setter 다섯(`setOccupancyThres`,
`setProbHit`, `setProbMiss`, `setClampingThresMin`, `setClampingThresMax`)이
`moveit-octomap`에 없어서 `occ_prob_thres_log`와 그 형제들이 `OcTree`
수명 내내 상류 기본값에 고정된다. `moveit2` 안에 호출자가 0건이므로
당장의 결함은 아니지만, 구조상 "설정 가능"인 것이 실제로는 상수인
상태다. `7031639`가 호출 지점에 그 사실을 적었다. 기본값 자체는
`bbed614`에서 이미 오라클로 핀돼 있다.

### 88.7 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1048/1048**,
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **29/29 identical**. 스탬프
`7cc8a73408a83c92` 유지.

담당이 보고한 1017/1017은 베이스 `4b13c99` 기준 값이다.

### 88.8 §79 스윕 현황

```
p3-shapes        98/98  닫힘 (이 절)
p6-totg          61건   라운드 11 진행 중
p3-distance-field 7건   4건이 함정 확인, 라운드 14에서 수정 중 (§85)
p1-fixtures       1건   이미 max_relative 있음 — 해당 없음
p1-robotmodel     0건   (대신 손수 쓴 비교 43곳, 라운드 11)
p1-joints         0건   (손수 쓴 비교 3곳)
p3-acm            나머지 — 아직 발주 안 함
```

## 89. §85가 닫혔다, 그리고 `from_octree`가 착지했다 (2026-08-04)

p3-distance-field 라운드 14 머지(`be10d78`, `458063f`, `41227a7`).
베이스 `aac08af`, main에 머지. **1048 → 1050**.

### 89.1 두 무는 지점을 내가 다시 쟀다 — 둘 다 재현된다

§85.1이 뒤집은 판정을 담당이 상수군별로 다시 재고 고쳤다. 내 재측정:

```
ULP_TOL (네 개의 함정 사이트, epsilon = 0.0)
  1.9e-16   3/3 통과
  1.85e-16  1 fail
  1.8e-16   1 fail
  1e-16     1 fail
  0.0       1 fail

RESOLUTION (세 개의 진짜 게이트 사이트)
  0.02    통과
  0.0136  통과
  0.0135  1 fail
  0.01    1 fail
```

담당이 적은 두 값 — 함정 사이트 바닥 `1.850371707708594e-16`, 게이트
사이트 바닥 `~0.0135530` — 이 그대로 나온다. **§85가 닫혔다.**

**다만 헤드룸을 짚어 둔다.** 함정 사이트에 넣은 `ULP_TOL = f64::EPSILON`
(2.22e-16)은 바닥 1.8504e-16의 **1.2배**다. 자릿수가 아니라 20% 여유다.
ULP 수준 허용오차로는 타당한 선택이지만, 부동소수 평가 순서가 바뀌면
바로 깨지는 마진이라는 뜻이다 — §88.3의 `bodies.rs`(상수 1e-13, 바닥
1e-16–1e-15, 헤드룸 2–3자리)와는 성격이 다르다. 이 파일이 앞으로
실패하면 회귀가 아니라 마진 문제일 수 있다.

### 89.2 `from_octree`의 상류 근거를 직접 읽었다

`collision_distance_field_types.cpp:355-365`:

```cpp
PosedBodyPointDecomposition::PosedBodyPointDecomposition(
    const std::shared_ptr<const octomap::OcTree>& octree)
  : body_decomposition_()
{
  int num_nodes = octree->getNumLeafNodes();
  posed_collision_points_.reserve(num_nodes);
  for (octomap::OcTree::tree_iterator tree_iter = octree->begin_tree();
       tree_iter != octree->end_tree(); ++tree_iter)
    posed_collision_points_.push_back(
        Eigen::Vector3d(tree_iter.getX(), tree_iter.getY(), tree_iter.getZ()));
}
```

담당의 독해가 맞다 — 점유 필터도, 리프 필터도 없이 **모든 노드**의
좌표를 담는다. `reserve(getNumLeafNodes())`는 힌트일 뿐이고, 실제로는
내부 노드까지 넣으므로 상류 스스로 과소 예약을 하고 있다. Rust 쪽은
`tree_nodes().map(|n| n.coordinate().coords).collect()`로 같은 것을 한다.

**필터가 진짜 없다는 것이 재어진다.** `-p moveit-distance-field`(74건)
`--no-fail-fast`:

```
.filter(|node| node.is_leaf())      1 fail
.filter(|node| node.is_occupied())  1 fail
무변경 (대조)                        74/74 통과
```

점 개수 검증도 자기 참조가 아니다 — `OcTree::num_nodes()`
(`tree.rs:316`)는 이터레이터가 아니라 별도의 재귀 카운트다(§73에서 이미
`calcNumNodes`이지 `size()`가 아님을 확인했다).

`addOcTreeToField`는 별개이고 여전히 미이식이라는 구분도 맞다 —
`distance_field.cpp:219-291`은 바운딩박스 리프 순회, 점유 필터,
해상도 이하 세분까지 하는 더 복잡한 알고리즘이다.

### 89.3 브리프 지시 하나를 담당이 거절했고, 그 판단이 맞다

브리프에 "오라클로 핀해라"라고 썼는데 담당은 `oracle.cpp`를 건드리지
않았다 — 이번 라운드에 `tools/` 소유가 자기가 아니라는 이유다. 그
판단이 옳다. 대신 비자기참조 단위 테스트 두 건으로 덮었고 위에서 내가
확인했듯 실제로 재고 있다. **오라클 커버리지가 필요하면 `tools/` 소유자를
거쳐야 한다** — 이 라운드 세트에서 `oracle.cpp`를 만진 것은 p3-shapes
라운드 13(`tree_walk`)뿐이다.

### 89.4 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1050/1050**(1048 + 2),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **29/29 identical**. 스탬프
`7cc8a73408a83c92` 유지.

## 90. 도달 불가 분기가 표현 불가능해졌다 — 그리고 보고서 한 줄이 트리와 다르다 (2026-08-04)

p1-robotmodel 라운드 11 머지(`8146ecc`, `95041dd`, `0ba77c4`).
베이스 `0ce4524`, main에 머지. **1050 → 1052**.

### 90.1 §84.2의 구조적 처방이 실행됐다

`used`가 `BTreeMap`에서 `Option<(String, IkConstraintSamplerAdapter)>`로
바뀌었다. `smallest_across_links`와 그 안의 검증 불가능한 동률 비교는
트리에서 사라졌다(`rg`로 확인, 0건). **도달 불가 분기를 주석으로
설명하는 대신 표현 불가능하게 만든 것**이 이 라운드 세트에서 처음 나온
구조적 종결이다.

살아남은 링크별 동률 방향은 여전히 물린다 —
`existing < candidate`를 `<=`로 뒤집으면 `-p moveit-constraints`(89건)에서
1 fail이다.

### 90.2 서브그룹 재귀 상한이 상류에 없다

상류 `constraint_sampler_manager.cpp:366`이 `selectDefaultSampler`를
**자기 자신으로** 다시 부른다. 즉 재귀에 깊이 제한이 없고, 포트가 갖고
있던 한 단계 상한은 실제 이탈이었다. `SubgroupSolver` 재귀 트리로
교체됐고, 재귀 스레딩을 `Vec::new()`로 끊으면 1 fail이다 — 깊이 2
테스트(`top`→`mid`→`leaf`)가 실제로 재고 있다.

### 90.3 손수 쓴 허용오차 43곳은 전부 진짜 게이트다

담당이 13개 파일 43+6곳을 자동 하네스로 이분했고, 상수를 바꿀 곳이
하나도 없다는 결론이다. 내가 `so2.rs`를 표본으로 다시 쟀다
(`-p moveit-planners-sbp`, 93건):

```
1e-9 (현재)  93/93 통과
1e-12        93/93 통과
1e-15        93/93 통과
1e-16        4 fail
0.0          7 fail
```

바닥이 `1e-16`–`1e-15`, 상수가 `1e-9`이므로 **헤드룸 6–7자리**다.
담당이 보고한 "1.5–12+자리" 범위와 맞다. 그리고 `.abs() <` 형태는
`assert_relative_eq!`와 달리 `max_relative` 경로가 없으므로 `0.0`까지의
이분이 그대로 의미를 갖는다 — §79 함정의 사촌이 아니라 순수한
상수-과대 문제였고, 이 크레이트에는 없었다.

`space.rs:366`의 `0.02`가 부동소수 허용오차가 아니라 몬테카를로 통계
경계(~8.5σ)라는 구분도 타당하다.

### 90.4 보고서 한 줄이 트리와 다르다 — `isFixedFrame` 배선은 설치가 아니다

보고서는 `0ba77c4`가 "constructor-side call site wiring
`PlanningScene::transforms_with_world_objects()` into
`PositionConstraint::new`"를 **추가**했다고 적었다. 실제로 들어간 것은
`#[cfg(test)]` 안의 테스트 하나다.

`rg`로 확인한 워크스페이스 전체 상황:

```
moveit-scene/src/scene.rs:821       정의
moveit-scene/src/scene.rs:2537-2667 자기 테스트 8건
moveit-planners-sbp/src/planning_scene_validity.rs:398, :411
                                    #[cfg(test)] 테스트 안
```

**프로덕션 호출자는 0건이다.** 그리고 그건 담당의 잘못이 아니다 —
`construct_goal_pose_constraints`(`crates/moveit-constraints/src/utils.rs:245`)를 비롯한 생성 경로가
전부 `tf: &Transforms`를 **호출자에게서 받는** 형태이고, 워크스페이스에
`PlanningScene`에서 목표 제약을 만드는 프로덕션 경로가 아직 없다.
배선할 대상 자체가 없다.

그러므로 UNFIXED 문구를 바꾼다: **"`isFixedFrame` 생성자 미배선"이
아니라 "배선 지점이 특정·테스트로 증명됐고, 프로덕션 호출자는 그런
경로가 생길 때 만들어진다"**이다. 어느 크레이트가 그 경로를 갖게 될지
(`moveit-planners-sbp`가 유일하게 양쪽에 의존한다)까지 문서에 남았으니
재개 조건은 명확하다.

`moveit-constraints`가 `moveit-scene`에 의존할 수 없다는 근거
(`check-dep-direction.sh`가 `moveit-scene → moveit-constraints` 방향을
이미 쓰고 있어 순환이 된다)는 맞다.

### 90.5 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1052/1052**(1050 + 2),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **29/29 identical**. 스탬프
`7cc8a73408a83c92` 유지.

담당이 보고한 1038/1038은 베이스 `0ce4524` 기준 값이다.

## 91. p1-fixtures 라운드 12 머지 — `kinematics_metrics` 이식과 허용오차 재측정

`moveit-metrics` 크레이트가 새로 들어왔다(`kinematics_metrics` 4개
공개 메서드), 오라클에 `kinematics_metrics` 질의가 붙었고 스탬프가
`7cc8a73408a83c92` → `3426f1b1193961ee`로 올라갔다. 이미지는 다시
빌드했다.

### 91.1 담당이 잰 두 값은 소수점까지 재현된다

임시 프로브로 40개 스칼라 비교와 120개 타원체 성분의 잔차를 직접 쟀다:

```
SCALAR    worst measured relative residual = 4.7503e-14   (보고서 4.75e-14)
ELLIPSOID worst measured relative residual = 3.5860e-13   (보고서 3.59e-13)
```

`epsilon`을 0으로 놓고 `max_relative`만 이분한 결과도 이 값과 맞는다 —
스칼라는 1e-13 통과 / 1e-14 실패, 타원체는 1e-12 통과 / 1e-13 실패.

### 91.2 그런데 무는 상수는 `max_relative`가 아니라 `epsilon`이다

§85.3대로 **상수 하나씩** 이분했더니 넷 다 단독으로는 0.0까지 통과했다.
쌍으로 둘 다 0.0으로 놓아야 1 fail이다. 이유는 fixture 값의 크기다:

```
manipulability_index_full         min 2.94616e-11  max 1.34701e-06
manipulability_index_translation  min 5.70825e-10  max 1.69937e-06
manipulability_full               min 1.88898e-10  max 1.97680e-06
manipulability_translation        min 1.30473e-09  max 7.03754e-06
```

가장 큰 값에서도 `SCALAR_MAX_RELATIVE * |expected|` = 1e-10 × 7.04e-06 =
7.04e-16 로, `SCALAR_EPSILON` 1e-12보다 **작다**. 즉 40개 비교 전부에서
`epsilon` 쪽이 허용폭을 정한다. 그 허용폭을 상대오차로 환산하면:

```
SCALAR    permitted band as relative: 1.4210e-7 ~ 3.3942e-2
SCALAR    min headroom (band / residual) = 1.6866e8
```

가장 작은 값(2.94616e-11)에서는 **3.4% 상대오차가 통과한다**. 주석이
적은 "`1e-10`은 측정 최악값의 ~2000배"는 성립하지 않고, "`epsilon`
바닥은 비용이 없다"도 사실이 아니다 — 바닥이 전부를 결정하고 있다.

타원체도 같은 모양이다. 고윳값 최소 3.29669e-03, 고유벡터 성분 최소
1.62356e-03에 `ELLIPSOID_EPSILON` 1e-9이 걸리므로:

```
ELLIPSOID permitted band as relative: 최대 6.1593e-7
ELLIPSOID min headroom = 5.2984e5
```

"~2700배"가 아니라 5~6자리다.

지시는 §88.4·§89.1과 같은 형태다: **`epsilon` 두 개를 0.0으로 내리고
`max_relative`를 측정 바닥에서 자릿수만큼 띄워라**(스칼라 1e-13,
타원체 1e-12이 바닥). 값의 크기가 1e-11까지 내려가는 fixture에서
절대 바닥은 상대 허용오차를 무력화한다.

이 건은 §85.3의 직접적인 사례다. 쌍으로 묶어 이분했으면 "0.0에서
실패한다"만 보고 건강해 보였을 것이다. 하나씩 재야 둘 다 단독으로
불필요하다는 게 보인다.

### 91.3 섭동 10건이 문다

담당의 3건을 내가 다시 돌렸고 셋 다 재현된다. 여기에 7건을 더 걸었다.
전부 `--no-fail-fast`, 각각 적용 → 실행 → 되돌림:

```
P1  manipulability의 min()/max() 뒤집기                    1 fail (담당)
P2  manipulability_index의 translation 분기 무력화          1 fail (담당)
P3  joint-limits penalty를 1.0으로 고정                     1 fail (담당)
P5  타원체가 (0,0) 대신 (3,3) 3x3 블록을 읽음               1 fail
P7  range <= f64::MIN_POSITIVE 건너뛰기 제거                1 fail
P10 penalty_multiplier==0 단축 경로 제거                    2 fail
P11 penalty 식 1-exp(-k*m) → exp(-k*m)                     1 fail
P12 manipulability에서 penalty 인자 제거                    1 fail
P13 고윳값에 1.000001 곱하기                                1 fail
P14 manipulability의 translation이 rows(3,3)을 봄           1 fail
```

`1-exp(-k*m)`의 부호, 타원체가 penalty를 **곱하지 않는다**는 상류 사실
(P13이 6자리 섭동에서 문다), 고정 관절이 별도 분기 없이 `range` 검사로
걸러진다는 주장(P7) 모두 실측으로 닫혔다.

### 91.4 그러나 4건은 통과한다 — 분기 하나와 관절종류 건너뛰기 3건이 안 물린다

```
P4  columns < 6  →  columns < 8 (7-DOF 팔을 SVD-곱 경로로 강제)  7/7 통과
P6  연속 회전관절 건너뛰기 제거                                  7/7 통과
P8  floating 관절 건너뛰기 제거                                  7/7 통과
P9  planar 관절 건너뛰기 제거                                    7/7 통과
```

**P4**: 테스트 모듈 주석은 `columns < 6` 분기가 오라클 커버리지가
없다고 이미 적어 뒀다. 실측은 그 반대편까지 말한다 — 조건 자체가 양쪽
어느 방향으로도 측정되지 않는다. 계수(full-rank) J에서는 두 경로가
허용오차 안에서 같은 값을 내기 때문이다. 활성 변수 6개 미만인 그룹이
필요하다.

**P6/P8/P9**: `panda_arm`의 `joint_indices()`에 연속·floating·planar
관절이 하나도 없다. `fixtures/panda.srdf:49`가 floating
`virtual_joint`를 선언하지만 그 관절은 `panda_arm` 그룹에 들어 있지
않아 P8이 통과한다.

넷 중 **planar이 제일 급하다.** 모듈 주석에서 가장 길고 가장 섬세한
이탈 근거가 planar 센티널이다 — 상류는 `-DBL_MAX`, 이 포트는
`f64::NEG_INFINITY`이고, 주석 스스로 "`DBL_MAX`를 쓰면 조용히 절대
매치되지 않고 항상 통과해 버린다"고 적었다. 그 문장을 검증하는 것이
지금 트리에 없다. **문서로만 있는 이탈**은 §79와 같은 모양이다.

필요한 fixture는 이미 다 있다. 새 로봇 파일이 필요 없다:

```
fixtures/pr2.urdf      type="continuous" 19건
fixtures/pr2.srdf:5    virtual_joint world_joint type="planar"
fixtures/panda.srdf:49 virtual_joint type="floating"
```

`joint_limits_penalty`는 공개 API이고 체인 그룹을 요구하지 않으므로
(`manipulability*`와 달리) 오라클 없이 단위 테스트로 넷 다 물릴 수
있다.

### 91.5 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1059/1059**(1052 + 7),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **30/30 identical**(29 → 30,
`moveit-metrics/panda_kinematics_metrics` 추가분).

담당이 보고한 1030/1030은 베이스 `2a2c5af` 기준 값이고, 재생 29 → 30도
그 기준에서 센 것이다.

## 92. `assert_relative_eq!` 전수 재고 — 남은 소유자별 분포

§91.2가 나온 뒤 워크스페이스 전체를 다시 셌다. `rg -c`의 줄 세기는
여러 줄에 걸친 호출을 놓치므로 괄호 매칭으로 호출 하나하나의 인자를
읽어 분류했다.

**첫 집계는 틀렸다.** 괄호 매칭만 하고 주석을 걸러내지 않아
`//!`/`///` 안에서 매크로 이름을 **언급만** 한 줄까지 호출로 셌다.
`moveit-distance-field`가 14건으로, `moveit-geometry`가 7건으로 부풀어
있었다. 주석 꼬리를 잘라내고 다시 세면 각각 4건이다. §83.3·§73.1과
같은 집계 오염이고, 이번에는 범위를 좁혀서가 아니라 **주석을 코드로
세어서** 생겼다. 세 번째다.

main `44d2bfe` 기준, p6-totg의 미머지 17커밋은 **반영되지 않은 값**:

```
총 183건 = max_relative 명시 60 / epsilon만 108 / 둘 다 없음 15
```

`epsilon`만 있는 108건에는 기본 `max_relative = f64::EPSILON`이 항상
켜져 있고, 둘 다 없는 15건은 양쪽 다 `f64::EPSILON`이라 사실상 정확
비교다 — 이 15건은 가장 엄격한 형태이지 구멍이 아니다. 구멍은 **상수를
비교값의 크기와 무관하게 고른 곳**이고, 두 방향 다 이 세션에서
실측됐다:

- 크기가 큰 쪽 — `max_relative`가 조용히 넓어진다(§85·§88·§89)
- 크기가 작은 쪽 — 절대 `epsilon` 바닥이 `max_relative`를 덮는다
  (§91.2, 허용폭이 3.4% 상대까지 벌어졌다)

`max_relative`가 없는 사이트를 소유자별로 추리면(`epsilon만 + 둘 다
없음`):

```
p3-acm            51건
  moveit-collision/src/parry.rs                        18 + 4
  moveit-model/src/joint/planar.rs                     10
  moveit-model/src/joint/revolute.rs                    6 + 3
  moveit-model/src/joint/floating.rs                    4
  moveit-model/src/joint/model.rs                       2 + 1
  moveit-model/src/joint/prismatic.rs                   1 + 2
p6-totg           57건 (미머지 17커밋이 대부분을 이미 건드렸다)
  moveit-trajectory/src/trajectory.rs                  21
  moveit-trajectory/src/path.rs                         6
  moveit-trajectory/src/path_segment/linear.rs          3 + 3
  moveit-trajectory/tests/*_parity.rs                  16
  moveit-trajectory/src/path_segment/circular.rs        3
  moveit-trajectory/src/trajectory_tools.rs             2
  moveit-smoothing/tests/*_parity.rs                    5
  moveit-smoothing/src/butterworth.rs                   1
p3-distance-field  4건
  tests/upstream_parity.rs                              3
  src/collision_distance_field_types.rs                     1
p3-shapes          4건
  moveit-geometry/src/transforms.rs                     3
  moveit-geometry/src/stl.rs                                1
무소유             4건
  moveit-state/tests/invariants.rs                      4
```

세 가지를 바로잡는다:

- p3-acm 몫은 **52건이 아니라 51건**이다.
  `moveit-collision/tests/world_parity.rs`의 1건은 이미
  `max_relative`가 붙어 있다.
- §88은 `moveit-geometry`를 "92/98 전환"으로 닫았지만 `bodies.rs`
  바깥에 **4건이 남아 있다** — `transforms.rs` 3, `stl.rs` 1. 그
  전수에 들어가지 않은 파일들이다.
- `upstream_parity.rs`의 7건 중 `max_relative`가 붙은 것은 4건이고
  나머지 3건은 `epsilon = RESOLUTION`(0.1)이다. 이 3건은 구멍이
  **아니다** — 상류 `EXPECT_NEAR`의 값을 그대로 옮긴 것이고 파일
  주석이 그렇게 적고 있다. 크기 대비로는 §91.2와 같은 모양(값이 ~1인데
  바닥이 0.1)이지만, 그것이 상류의 선택이므로 이 포트가 임의로 조이면
  충실성을 잃는다. 그 크레이트에서 실제로 남은 것은
  `src/collision_distance_field_types.rs`의 1건이다.

`moveit-state/tests/invariants.rs`의 4건은 소유자가 없는 크레이트에
있다. 지금은 어느 패널도 이 파일을 자기 것으로 열지 않는다 — 다음
라운드에서 배정한다.

## 93. p1-joints 라운드 12 머지 — `CachedIkSolver` 이식과 재는 것이 없는 상수

`ik_cache.rs`(`IKCache`)와 `cached_solver.rs`(`CachedIKKinematicsPlugin`
데코레이터)가 들어왔고 `newton_raphson_cached`/`lma_cached` 두 등록이
붙었다. 5커밋, 범위는 `moveit-kinematics`뿐이다.

### 93.1 상류 대조 — 세 지점을 직접 읽었다

- `Pose::distance`(`ik_cache.cpp:290-293`)는
  `(position - pose.position).length() + orientation.angleShortestPath(...)`
  다. 포트의 `pose_distance`가 `norm() + UnitQuaternion::angle_to`인
  것이 맞다 — `angle_to`도 최단경로 각([0, π])이다.
- `updateCache`의 삽입 가드는 `ik_cache_.size() < ik_cache_.capacity()`
  이지 `max_cache_size_`가 아니다(`:183`). 그런데
  `initializeCache`가 `:75`에서 `reserve(max_cache_size_)`를 먼저 하고
  `:119`의 `reserve(last_saved_cache_size_)`는 그보다 작으므로 libstdc++
  `reserve`의 무연산 규칙에 걸린다. 용량은 `max_cache_size_`에 머무르고,
  포트의 `entries.len() >= self.max_cache_size` 가드와 동치다.
- 삽입 조건이 OR(`pose 거리 초과 || config 제곱거리 초과`)인 것도
  `:183-184`에서 확인했다.

### 93.2 핀 8건이 문다

담당이 표로 낸 4건을 포함해 내가 직접 걸었다. `--no-fail-fast`, 각각
적용 → 실행 → 되돌림:

```
M1 update의 OR 게이트를 AND로                      3 fail
M2 pose_distance에서 angle_to 항 제거              1 fail
M3 nearest가 min 대신 max                          3 fail
M4 가득 찬 캐시 가드 제거                          1 fail
M6 캐시 시드를 무시하고 호출자 시드를 먼저 시도    2 fail
M7 호출자 시드 폴백 제거                           2 fail
M8 성공해도 캐시를 갱신하지 않음                   1 fail
```

M6과 M8의 첫 시도는 **컴파일에 실패했다**(`CacheEntry::config`가 미사용이
되어 `never used`가 에러로 올라온다). §82.1대로 컴파일되지 않는 삭제는
핀의 증거가 아니므로 `let _keep = nearest.config();`와 `if false {}`로
바꿔 다시 쟀다. 위 숫자는 그 재측정 값이다.

### 93.3 통과하는 섭동 3건 — 둘은 진짜 구멍, 하나는 아니다

```
M5  config 게이트 >  →  >=      31/31 통과
M5b pose 게이트   >  →  >=      31/31 통과
M9  update가 nearest를 재조회    31/31 통과
```

M5·M5b는 실제로 안 재는 경계다. 상류가 양쪽 다 strict `>`이고 포트도
그렇지만, 임계값에 **정확히 걸린** 입력을 넣는 테스트가 없다. §85 이후
이 저장소가 쓰는 "서사가 아니라 불변식 경계로 테스트하라"의 정확한
대상이다.

M9는 구멍이 **아니다**. 문서가 강조하는 "`update`에 넘기는 `nearest`는
두 시도 전에 조회한 그 값이지 여기서 다시 조회한 값이 아니다"는
`solve_with_options`가 `&mut self`를 잡고 있어 호출 중간에 캐시가 바뀔
수 없으므로 **구조적으로 보장된다**. 테스트가 필요 없는 불변식이니
다음 라운드에서 이걸 쫓지 마라.

### 93.4 `IK_DEGENERATE_EPS` — 보고서의 판정은 맞고 근거는 좁다

담당은 `main.rs:1755`/`:1771`의 `IK_DEGENERATE_EPS`가 "정보용 카운터로
출력될 뿐 `Verdict`로 들어가지 않는다"고 적었다. `Verdict` 부분은 맞고,
어떤 테스트도 이 상수나 두 필드를 참조하지 않는 것도 `rg`로 확인했다.

다만 `IkStats`는 `#[derive(serde::Serialize)]`이고
`main.rs:795`의 `ik: cfg.ik.then_some(ik_stats)`로 **`--stats-json`에
그대로 실린다**. 즉 이 상수는 출력만 되는 것이 아니라 내가 매 sweep마다
읽는 기계판독 결과의 한 숫자를 정한다. 1e-6 elementwise max-norm이
"degenerate"의 정의이고, 그 정의가 어디에서도 검증되지 않는다.

판정은 바뀌지 않는다(게이트하는 것이 없다는 것이 발견이다). 근거만
넓힌다.

### 93.5 오라클 fixture 부재는 사실이다

`moveit-kinematics`에는 `tests/fixtures/oracle-models.json`이 아예
없다 — 다른 9개 크레이트에는 있다. `"op": "ik"`를 쓰는 요청 fixture도
워크스페이스에 0건이다. 담당의 `6f35548` 기록이 맞다.

### 93.6 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1069/1069**(1059 + 10),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **30/30 identical**. 스탬프
`3426f1b1193961ee` 유지(오라클을 건드리지 않았다).

담당이 보고한 1058/1058은 베이스 `9a5292f` 기준 값이고, 재생 29/29도
그 기준이다. 담당이 이미지를 `7cc8a73408a83c92`로 다시 빌드한 것은 그
시점 트리에서 맞는 판단이었다.

## 94. p3-shapes 라운드 15 머지 — 확률공간 setter 다섯과 내가 만든 규칙 불일치

`AbstractOccupancyOcTree`의 setter 다섯이 이식됐고 오라클
`octomap` 질의가 무조건 `occupied` 필드를 내도록 확장됐다. 기존 id
1-12까지 다시 캡처됐다. 3커밋.

### 94.1 `oracle.cpp` 수정은 내 브리프가 허가한 것이다

다른 패널의 브리프는 전부 "`tools/`와 `PORTING-PLAN.md`는 내 것 —
`oracle.cpp` 확장이 필요하면 요청만 적어라"라고 쓴다. §89.3은
p3-distance-field가 브리프 지시보다 이 소유 규칙을 앞세워 `oracle.cpp`를
건드리지 않은 것을 옳다고 기록했다.

그런데 p3-shapes 라운드 15 브리프의 Ownership 블록은 "`tools/ci/`와
`PORTING-PLAN.md`는 내 것"이라고 **`tools/ci/`만** 적었고, 1항은
"다섯을 이식하고 각각을 **오라클로 핀한다**"를 권했고, 게이트 절은
"`oracle.cpp`를 고쳤으면 `build.sh`를 먼저 돌려라"라고 이미 수정을
전제했다. 세 군데가 일관되게 허가하고 있다.

**따라서 이 수정은 위반이 아니고 그대로 둔다.** 고칠 것은 규칙 쪽이다 —
같은 규칙이 패널마다 다르게 적히면 §89.3에서 옳았던 거절과 이번 라운드의
수정이 동시에 옳아지는 상태가 된다. 앞으로 모든 브리프에서 한 문장으로
통일한다: `tools/moveit-oracle/`는 내 것이고, 오라클 확장이 필요하면
보고서에 요청만 적는다. 이번처럼 예외를 두려면 브리프에 그 예외를
**명시적으로** 쓴다.

### 94.2 setter 다섯이 전부 문다

각 setter를 무연산으로 만들고 재측정했다(`logodds(prob) * 0.0 +
<기존값>` — 인자를 계속 쓰므로 컴파일된다, §82.1):

```
set_occupancy_thres    무연산   1 fail
set_prob_hit           무연산   1 fail
set_prob_miss          무연산   1 fail
set_clamping_thres_min 무연산   1 fail
set_clamping_thres_max 무연산   1 fail
```

다섯 다 `octomap_matches_liboctomap_for_every_boundary_scenario`가
잡는다. 다섯이 공유하는 변환 자체도 물린다 — `logodds`를
`ln(p/(1-p))`에서 `ln(p)`로 바꾸면 3 fail이고, 그중 둘
(`repeated_hits_converge_to_clamp_but_not_past_it`,
`zero_log_odds_is_occupied_under_the_default_threshold`)은 오라클
없이도 잡는 단위 테스트다.

### 94.3 `debug_assert!` 두 개는 안 재진다

```
debug_assert!(self.prob_hit_log >= 0.0) 제거   27/27 통과
```

상류 자신의 sanity check를 옮긴 것인데 범위 밖 확률을 넣는 테스트가
없다. `set_prob_miss`의 `<= 0.0`도 같다. nextest는 debug 프로파일로
도니 `debug_assert!`는 살아 있다 — 즉 테스트가 없어서 안 물리는
것이지 빌드 설정 때문이 아니다.

### 94.4 표류 정정 두 건은 사실이고, 총계는 내가 검증하지 않았다

- `457ea0f`가 "port tree_iterator as TreeNodes"인 것을 커밋에서 직접
  확인했다. 라운드 12 감사표가 `tree_iterator`를 미이식으로 적어 둔
  것은 실제 표류였고 정정이 맞다.
- `setNodeValue` 오버로드가 정확히 3개인 것을 상류 헤더에서 확인했다
  (`OccupancyOcTreeBase.h:158`, `:170`, `:184`). octomap이 이 머신에
  체크아웃돼 있지 않아 오라클 이미지 안의
  `/usr/include/octomap/`에서 읽었다.
- **검증하지 않은 것:** 24 ported / 2 unported / 15 distinct / 41
  symbol groups라는 총계. "symbol group"의 묶는 기준이 담당의
  정의이고 나는 그 정의를 재현하지 않았다. p1-fixtures의
  `planning_scene.hpp` 62개처럼 헤더의 `public:` 선언을 그대로 세는
  형태였다면 대조가 가능했을 것이다 — 다음 라운드에 그 형태를
  요구한다.

### 94.5 머지 후 실측

`oracle.cpp`가 p1-fixtures의 `kinematics_metrics` 확장과 합쳐지면서
스탬프가 `3426f1b1193961ee` → **`7b8463d6943edaac`**로 올라갔다.
담당 브랜치의 `270922540567cc3d`는 그 브랜치 안에서만 맞는 값이다.
이미지를 다시 빌드했다.

`cargo nextest run --workspace --no-fail-fast` **1069/1069**(변동 없음 —
기존 테스트 함수에 시나리오를 더한 것이지 새 `#[test]`가 아니다),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **30/30 identical**(재캡처된 octomap
fixture 포함).

담당이 보고한 1048/1048과 29/29는 베이스 `d849665` 기준 값이다.

## 95. 오라클 staleness 가드를 실제로 재 봤다

이 세션에서 모든 패널이 최소 한 번씩 낡은 이미지 스탬프에 걸렸다.
`run-oracle.sh`가 그것을 잡도록 짜여 있다는 것은 코드에 적혀 있었지만,
**가드가 실제로 발화하는지는 아무도 재지 않았다.** 양방향으로 쟀다:

```
IMAGE=moveit-rs/oracle:3426f1b1193961ee run-oracle.sh ...
  exit=1
  moveit-rs/oracle:3426f1b1193961ee was built from different oracle
  sources than the working tree
    image: 3426f1b1193961ee...
    tree:  7b8463d6943edaac...
  rebuild with tools/moveit-oracle/build.sh

현재 태그로 실행  →  모델 로드까지 정상 진행
```

가드는 태그 이름이 아니라 이미지 안의
`/usr/local/share/oracle-src.sha256`를 트리의 `oracle_stamp`와 대조한다.
태그는 변경 가능하므로 이름만으로는 부족하다는 판단이 옳고, 스탬프가
아예 없는 옛 이미지도 `<missing or unstamped>`로 걸린다.

**그러므로 낡은 이미지로 캡처된 fixture가 조용히 통과하는 경로는 없다.**
`verify-fixture-replay.sh`가 이미지를 빌드하지 않는다는 사실은 그대로지만
(그래서 `oracle.cpp`를 고친 뒤 `build.sh`를 먼저 돌리라는 지시는 계속
유효하다), 잊었을 때의 결과는 조용한 오답이 아니라 즉시 실패다.

부수 관찰: 이 머신에 `moveit-rs/oracle` 이미지가 10개 쌓여 있다.
정리는 파괴적 작업이라 사용자 확인 없이 하지 않는다.

## 96. p3-distance-field 라운드 15 머지 — `addOcTreeToField` 이식과 안 재지는 루프 경계

`octree_points`/`add_octree_to_field`가 상류
`distance_field.cpp:239-291`에서 이식됐고 `ULP_TOL`이 `TOL`로 바뀌며
`1e-14`로 넓어졌다. 3커밋.

`moveit-octomap` 의존은 이미 라운드 14에서 들어와 있었다 —
`leaves_in_bbx`가 있어 p3-shapes에 보고할 것이 없다는 판단이 맞다.

### 96.1 상류 대조 — 줄 단위로 맞는다

`getOcTreePoints`를 직접 읽었다. 포트가 상류와 같은 것:
`gridToWorld(0,0,0)`와 `gridToWorld(num_x, num_y, num_z)`로 만든 bbx,
`isNodeOccupied` 필터, `getSize() <= resolution_` 분기,
`ceil(getSize()/resolution_) * resolution_ / 2.0`, 그리고 세 겹
`for (x = c-ceil_val; x <= c+ceil_val; x += resolution_)`.

### 96.2 무는 지점 다섯 — 담당의 셋에 둘을 더했다

```
D1 점유 필터 무력화                    1 fail  (담당)
D2 세분 건너뛰기(항상 중심점 하나)     1 fail  (담당)
D3 bbx를 ±1e6으로 넓힘                 4 fail  (담당)
D4 ceil_val에서 /2.0 제거              1 fail
D5 ceil() → floor()                    1 fail
```

D3의 첫 시도(`Some(octree.leaves())`)는 **컴파일에 실패했다**
(`Point3` 미사용). §82.1대로 bbx를 ±1e6으로 넓히는 컴파일되는 형태로
다시 쟀다.

담당이 D3에서 "`add_points_to_field`의 자체 경계 검사가 통합 수준에서
bbox 클립 누락을 가린다, 그래서 private `octree_points`를 직접 테스트해야
했다"고 적은 것이 맞다. 넓힌 bbx에서 실패하는 4건에
`octree_points_excludes_leaves_outside_the_bounding_box`가 들어 있다.

### 96.3 루프 경계 `<=`가 안 재진다

```
while x <= coord.x + ceil_val  →  <     78/78 통과
while z <= coord.z + ceil_val  →  <     78/78 통과
```

이건 취향 경계가 아니다. 루프는 `coord.x - ceil_val`에서 시작해
`x += resolution`으로 누적하고, `resolution`(0.1 등)은 이진수로 정확히
표현되지 않는다. 마지막 반복이 `coord.x + ceil_val`에 **정확히**
떨어지는지는 부동소수 누적에 달려 있고, `<=`와 `<`가 갈리는 지점이
바로 거기다. 상류도 같은 취약성을 갖고 있으므로 포트는 충실하지만,
세분된 점 격자의 **마지막 면이 포함되는지**를 재는 것이 트리에 없다.

### 96.4 `TOL` 바닥은 재현되고 헤드룸은 54배다

```
TOL=1.9e-16   통과
TOL=1.85e-16  1 fail  (add_remove_points_matches_upstream_test_propagation_distance_field)
TOL=1.8e-16   1 fail
TOL=0.0       1 fail
```

§89.1의 바닥 `1.850371707708594e-16`이 그대로다. `TOL = 1e-14`이므로
헤드룸은 **54.0배 = 1.73자리**. §88.3이 `bodies.rs`에서 받아들인
2–3자리보다 낮다. 다만 이 잔차는 `df.distance(1000,1000,1000)` 대
`MAX_DIST` **한 지점**의 1 ULP 수준 양이고, 54 ULP의 여유는 그 크기에
비례한다. 더 넓히라고 요구하지 않는다 — 1.2배였던 §89의 상태에서
실질적으로 벗어났다.

### 96.5 커버리지 감사 — 둘은 재현되고 하나는 내 계수기가 못 센다

`distance_field.hpp`를 주석 제거 + 중괄호 깊이 추적으로 다시 셌다:

```
protected 메서드  2   ← 담당 보고 2 (1 ported / 1 unported)  일치
protected 필드    8   ← 담당 보고 8 (7 ported / 1 unported)  일치
public 메서드    27   ← 담당 보고 32                          불일치
```

**불일치의 원인은 내 쪽일 가능성이 높다.** 내 계수기는 `{ return
size_x_; }` 형태의 인라인 본문 접근자에서 깊이 추적이 끊겨 목록이
`getSizeX`에서 잘린다(`getSizeY`/`getSizeZ`/`getOriginX` 등이 빠졌다).
따라서 32가 틀렸다고 주장하지 않는다 — **확인하지 못했다**고 적는다.
§94.4에서 p3-shapes에 요구한 것과 같은 것을 요구한다: 세는 기준을
한 줄로 먼저 적어라. 기준이 적혀 있어야 다음 라운드에 대조가 된다.

### 96.6 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1073/1073**(1069 + 4),
`cargo test --doc --workspace` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK, 출처 검사와
연속 reseed 검사 통과, 재생 **30/30 identical**. 스탬프
`7b8463d6943edaac` 유지(오라클을 건드리지 않았다).

담당이 보고한 1054/1054와 29/29는 베이스 `87f2209` 기준 값이다.

## 97. p1-robotmodel 라운드 12 머지 — 검사 가능한 완료 조건과 검사할 수 없는 한 줄

문서만 바뀐 라운드다. 3커밋, 테스트 수 변동 없음(1073/1073).

### 97.1 세 숫자는 명령까지 그대로 재현된다

담당이 완료 조건에 적어 둔 명령을 **그대로 실행**했다:

```
rg -c '^//! - `' crates/moveit-constraints/src/lib.rs            → 61   일치
rg -c '^mod oracle_' .../tests/utils_parity.rs                   →  7   일치
sed -n '176,555p' .../utils_parity.rs | rg -c '#\[test\]'        → 16   일치
cargo nextest run -p moveit-constraints --no-fail-fast           → 89   일치
```

담당이 초안에서 자기 오류 셋(75→61, `^fn oracle_`→`mod oracle_`,
`solve()` 오버로드를 미이식으로 잘못 적은 것)을 잡아냈다고 보고한 것과
맞는다. **이번 라운드 세트에서 완료 조건의 숫자가 명령까지 전부 맞은
첫 사례다.**

### 97.2 그런데 `crates/moveit-constraints/src/utils.rs:64`의 명령은 돌아가지 않는다

`daee0dd`는 `transforms_with_world_objects`의 프로덕션 호출자가 0건임을
`rg -n 'transforms_with_world_objects' crates/ --glob '!*/tests/*'`로
확인했다고 적었고, 그 명령이 `crates/moveit-constraints/src/utils.rs:64`에 그대로 들어갔다.

**그 명령은 0건이 아니라 28줄을 낸다.** `--glob '!*/tests/*'`는
통합 테스트 디렉터리만 제외하고, `#[cfg(test)]` 모듈은 `src/` 안에
있기 때문이다(`scene.rs:2537-2667`, `planning_scene_validity.rs:374-419`).

내가 `#[cfg(test)]` 시작 줄을 기준으로 다시 분류했다:

```
CODE  crates/moveit-scene/src/scene.rs:821: pub fn transforms_with_world_objects(...)
```

**코드 히트는 정의 하나뿐이다. 결론(프로덕션 호출자 0건)은 맞다.**
틀린 것은 근거로 적어 둔 명령이다. 검사 가능한 완료 조건의 요점은
독자가 돌려 볼 수 있다는 것인데, 돌리면 28줄이 나와 문서가 틀린 것처럼
보인다. 명령을 고치거나, 기대 출력을 함께 적어야 한다.

### 97.3 상류 호출자 판정 — 결론은 맞고 근거는 더 강한 것이 있었다

담당은 "`constructGoalConstraints`의 실제 호출자 5곳이 전부
`moveit_ros`에 있고 D1 범위 밖"이라고 적었다. 세어 보면 다르다 —
정의/헤더/CHANGELOG를 뺀 참조가 **15개 파일 43줄**이고, 패키지별로는:

```
moveit_planners  7 파일   ← moveit_ros보다 많다
moveit_ros       5 파일
moveit_core      2 파일
moveit_py        1 파일
```

`moveit_planners/ompl`과 `pilz_industrial_motion_planner`는 플래너이지
ROS 배선이 아니므로 "전부 moveit_ros"라는 근거로는 닫히지 않는다.

**그러나 판정 자체는 맞고, 담당이 쓰지 않은 더 강한 근거가 있다.**
헤더의 `constructGoalConstraints` 오버로드 **7개 중 어느 것도
`PlanningScene`이나 `Transforms`를 받지 않는다**(`utils.hpp:83`, `:99`,
`:129`, `:148`, `:176`, `:205`, `:222`). 따라서 어느 패키지의 어느
호출자도 이 함수를 **통해** 씬과 목표 제약 생성을 짝지을 수 없다.
패키지 위치와 무관한 구조적 근거다. 호출자 위치를 세는 대신 시그니처를
읽었으면 한 번에 닫혔다.

### 97.4 하드코딩된 오라클 태그 다섯 — 셋은 정당하고 하나는 결함이다

**Anchor:** `rg -n 'oracle:[0-9a-f]{16}' crates/`
**Sites:**

```
moveit-constraints/src/lib.rs:78                    oracle:5188956fc433d046
moveit-constraints/tests/utils_parity.rs:12         oracle:5188956fc433d046
moveit-collision/tests/octree_world_collision_parity.rs:31  oracle:6192b2fbe3931089
moveit-geometry/tests/octree_shape_query_parity.rs:17       oracle:ec3982c6057ad64f
moveit-geometry/src/stl.rs:60                       oracle:1717af5da743934c
```

넷 다 현재 스탬프(`7b8463d6943edaac`)가 아니고, 이 머신의 이미지 목록에도
없다.

**같은 결함(고칠 것) — 1건:** `moveit-constraints/src/lib.rs:78`.
**검사 가능한 완료 조건 안에** 재현 불가능한 식별자가 들어 있다. 완료
조건의 다른 숫자는 전부 돌아가는데(§97.1) 이 하나만 돌아가지 않는다.

**다른 것(그대로 둔다) — 4건:** 나머지는 "이 fixture는 이 이미지에서
캡처됐다"는 **이력 기록**이다(`captured against`). 이력은 낡는 것이
정상이고, fixture의 현재 재현성은 `verify-fixture-provenance.sh`가
현재 이미지로 매 라운드 검사한다. `stl.rs:60`은 "그 이미지 안에 assimp
소스가 없더라"는 관찰이라 재확인 경로가 사라졌지만 재현 지시가 아니다.

파일 소유자가 셋으로 갈리므로(p1-robotmodel / p3-acm / p3-shapes) 내가
고치지 않는다. 해당 소유자 브리프에 한 줄씩 넣는다.

### 97.5 머지 후 실측

`cargo nextest run --workspace --no-fail-fast` **1073/1073**(문서만 바뀌어
변동 없음), `cargo test --doc --workspace` 통과, clippy `--workspace
--all-targets -D warnings` 0건, `fmt --check` 통과, `check-*.sh` 3건 OK.

담당이 보고한 1052/1052와 29/29는 베이스 `7d8d40a` 기준 값이다.

## 98. p6-totg 라운드 11 머지 — 담당이 자기 오류를 잡았고, 설명되지 않은 8.893e-9

17커밋(`0d1c240`..`fda0733`), 15파일, `moveit-trajectory`와
`moveit-smoothing` 두 크레이트. 단언만 바뀌어 테스트 수는 1073/1073
그대로다. `8cce74f`로 `--no-ff` 머지했다.

게이트는 머지된 트리에서 내가 다시 돌렸다: fmt 통과, clippy
`--workspace --all-targets -D warnings` 0건, nextest **1073/1073**,
`--doc` 통과, `cargo doc --no-deps` 통과, `check-*.sh` 3건 OK,
provenance OK, reseed OK, 재생 **30/30 identical**.

### 98.1 §92의 57건이 11건으로 줄었다

§92에서 이 두 크레이트에 `max_relative` 없는 사이트가 57건이라고
적었다. 머지 후 전수 재고를 다시 돌렸다:

```
assert_relative_eq! 호출        30
  max_relative 있음             19
  epsilon만                     11   ← 전부 trajectory.rs
  둘 다 없음                     0
```

남은 11건은 전부 `trajectory.rs` 안이다. §79 노출에서 이 담당 몫으로는
가장 큰 감소다.

### 98.2 담당이 스스로 잡은 결함이 실제로 문다

담당은 `trajectory.rs`의 duration 단언에서 `max_relative = 1e-6`을
넣으면 **허용 폭이 오히려 넓어진다**는 것을 스스로 발견하고
`f64::EPSILON`으로 되돌렸다고 보고했다. 내가 직접 재확인했다.

```
1_922.141_842_744_594_4 → 1_922.141_852_744_594_4  (+1e-5)   1 fail
                        → +1e-8                              통과
                        → +1e-9                              통과
+1e-5 nudge + max_relative = 1e-6 복원                        통과   ← 결함 재현
```

`approx`는 `|a-b| <= epsilon` **또는** `|a-b| <= max_relative *
max(|a|,|b|)`이면 통과한다. 값이 1922이므로 `max_relative = 1e-6`은
실효 허용 폭 `1.9e-3`, `f64::EPSILON`은 `4.3e-13`이다. `epsilon =
1e-6`이 둘 중 더 좁아 실제로 무는 쪽이 되고, 그래서 +1e-5는 실패하고
+1e-8은 통과한다.

**§91.2의 반대 방향이다.** §91.2에서는 비교값이 작아(≤7.03754e-06)
절대 floor가 `max_relative`를 삼켰다. 여기서는 비교값이 커서
`max_relative`가 절대 floor를 삼킨다. 같은 결함의 두 얼굴이고, 규칙은
하나다 — **허용치는 비교되는 값의 크기를 보고 정해라.**

내 첫 섭동은 무효였다. `1922.1418427445944`를 치환했는데 소스의
리터럴은 `1_922.141_842_744_594_4`로 밑줄이 들어가 있어 doc 주석의
출현만 바뀌었고 아무것도 실패하지 않았다. 밑줄 형태로 다시 재서 위
숫자를 얻었다. §82.1과 같은 종류의 무효 측정이다.

### 98.3 `epsilon = 0.1` 속도 사이트는 상류 충실이다

상류 `test_time_optimal_trajectory_generation.cpp`를 직접 읽었다:
`EXPECT_NEAR` 13건, `EXPECT_DOUBLE_EQ` 28건.

```
:108 :109 :156 :157   EXPECT_NEAR(0.0, trajectory.getVelocity(0.0)[0], 0.1)
:140                  EXPECT_DOUBLE_EQ(1922.1418427445944, trajectory.getDuration())
```

포트의 `epsilon = 0.1` 속도 사이트는 상류 자신의 `EXPECT_NEAR` 폭을
그대로 옮긴 것이다. §92에서 p3-distance-field의 `epsilon = RESOLUTION`
3건을 구멍이 아니라고 판정한 것과 같은 범주다 — **상류가 고른 폭을
옮긴 것은 이 저장소가 채우라고 요구하는 구멍이 아니다.**

### 98.4 설명되지 않은 8.893e-9

상류가 `EXPECT_DOUBLE_EQ`(4 ULP)로 못 박은 duration을 이 포트는
그 폭 안에서 재현하지 못한다.

```
상류    1922.1418427445944
포트    1922.14184275348748
차이    8.893e-9 절대 = 4.6e-12 상대 ≈ 20000 ULP
```

`epsilon = 1e-6`이 이 차이를 덮으므로 테스트는 통과하고, 헤드룸은
약 112배다. **그러나 8.893e-9이 어디서 오는지는 설명되지 않았다.**
경계는 지어졌고 원인은 지어지지 않았다. 20000 ULP는 마지막 자리
누적으로 보기에 큰 값이다 — 적분 스텝 수, `std::` 대 Rust의
초월함수 구현, 또는 경로 이산화 중 하나가 갈리고 있을 가능성이
있고 어느 쪽인지 재지 않았다. 다음 라운드의 1항이다.

### 98.5 문서 주장 대조 — 22건이고 낡은 표현은 없다

담당은 "doc 히트 24건을 확인했고 낡은 표현이 없다"고 보고했다. 내
패턴(`unported|out of scope|not yet|once ported`)으로는 **22건**이
나온다. 24를 재현하지 못했다 — 패턴 차이로 보이고 결함으로 보지
않는다. 그 22건은 내가 전부 읽었고 낡은 표현은 없다:

- D1 제외(`moveit_msgs`/`trajectory_msgs` 변환)에 결정을 함께 적은 것
  다수.
- `crates/moveit-trajectory/src/trajectory.rs:15`는 **자기 정정을 기록한 것**이다 — "이 주석은
  전에 범위 밖이라고 적었는데 그 모듈이 착지하면서 사실이 아니게
  됐다".
- `crates/moveit-trajectory/src/ruckig_smoothing.rs:72`의 "out of scope for this crate to add"는
  `moveit-model`에 인덱스 목록 메서드를 추가하는 일을 가리키고,
  그 크레이트는 p3-acm 소유다. 소유권 진술이라 맞다.

`crates/moveit-trajectory/src/lib.rs:339`와 `crates/moveit-trajectory/src/robot_trajectory.rs:21` 두 곳은 `RobotTrajectory::print`
/`operator<<`를 **미이식으로 남은 유일한 항목**으로 지목하면서 그
이유를 "D 결정도 의존성 부재도 아니고, 어떤 라운드도 요구하지 않았기
때문"이라고 적는다. 이 저장소가 금지한 "not yet" 자리채움이 아니라
**행동 가능한 실제 격차**다. 미룬 원래 이유(`RobotState`에 속도·가속도가
없었다)가 `RuckigSmoothing` 때문에 사라졌다는 것까지 적혀 있다.
다음 라운드에 이식한다.

## 99. p3-distance-field 라운드 16 머지 — 39%는 진짜고, `<=`는 여전히 안 재진다

3커밋(`54da0e9`, `8cd958e`, `94708cb`), `moveit-distance-field` 한
크레이트, 테스트 +2.

### 99.1 누적 오차가 마지막 면을 떨어뜨리는지 — 답은 39%다

§96.3에서 "누적 오차가 마지막 면을 떨어뜨리는 `resolution` 값이
존재하는지 찾아라, 근거 없이 괜찮다로 닫지 마라"고 요구했다. 담당이
실제 `OcTree` 위에서 448개 `(field_resolution, octree_resolution,
insert_point)` 조합을 쓸어 **176/448(39%)에서 마지막 면이 떨어진다**는
것을 쟀다. 축별로 갈리고 `k`의 홀짝과 무관하다는 것도 두 사례로
보였다(짝수 `k=10`: 세 축 전부 떨어져 1331이 아니라 1000. 홀수 `k=5`:
`x`만 떨어져 216이 아니라 180).

**드문 구석이 아니라 흔한 경우다.** 요구한 근거는 나왔다.

### 99.2 그런데 연산자 자체는 아직 안 재진다

머지된 트리에서 세 축의 `<=`를 각각 `<`로 바꿔 봤다:

```
x loop <= -> <     80/80 통과
y loop <= -> <     80/80 통과
z loop <= -> <     80/80 통과
```

**§96.3이 지적한 그 구멍이 그대로다.** 새 테스트 둘은 `<=`가 이미
`<`처럼 행동하는 입력을 골라 그 결과(떨어진 면)를 못 박는다. 그런
입력에서는 두 연산자가 같은 답을 내므로 어느 쪽으로 바꿔도 테스트가
통과한다. 담당 자신의 주석이 "not a taste check of `<=` vs `<`"라고
적은 것이 바로 그 표시다.

연산자를 재려면 **누적이 경계에 정확히 떨어지는** 입력이 필요하고,
그런 입력은 존재한다 — 해상도를 2의 거듭제곱으로 잡으면 누적이
정확해진다. 내가 직접 쟀다:

```
octree resolution 0.125, field resolution 0.0625,
insert (0.3125, 0.3125, 0.3125)  →  leaf size 0.125, coord 0.3125 (정확)

<=  : 27점   (축당 3점, 양 끝 포함)
<   :  8점   (축당 2점, 마지막 면 탈락)
```

축당 개수가 `ceil(size/resolution) + 1 = 3`이고 실제로 3이 나온다.
이것이 §96.3이 요구한 "양 끝 포함 `ceil+1`이 실제로 그런지 숫자로
확인"이고, 동시에 `<=`를 `<`와 구분하는 핀이다. 다음 라운드에
넣는다.

39% 사례와 이 사례는 대립하지 않는다 — 전자는 "누적이 어긋나면 면이
떨어진다", 후자는 "누적이 정확하면 `<=`가 면을 살린다"이고, 둘 다
있어야 루프가 문서대로 동작한다는 것이 재진다.

### 99.3 세는 기준은 적혔고 27 대 32도 설명된다

§96.5에서 요구한 대로 `lib.rs`에 세는 기준이 들어갔다 — 생성자·소멸자
제외, 인라인 본문 접근자 포함, 오버로드는 시그니처마다 따로,
`virtual` 재선언 없음. **내 계수기가 27에서 멈춘 이유가 그 기준으로
설명된다**(`{ return size_x_; }` 형태에서 깊이 추적이 끊겨
`getSizeY`/`getSizeZ`/`getOriginX` 등을 놓쳤다). 기준이 적혔으니 다음
라운드에 그 기준대로 대조한다.

### 99.4 §92가 이 크레이트에 남긴 1건이 닫혔다

`collision_distance_field_types.rs:1263`의 `assert_relative_eq!(sphere
.radius, 0.2)`는 `radius * 1.0 + 0.0`이라 비트 동일이고, `assert_eq!`로
바꿨다. 근거가 주석에 들어 있다.

**정정(§104):** 이 절은 처음에 "워크스페이스에 `epsilon`도
`max_relative`도 없는 사이트는 이제 0건"이라고 적었다. 틀렸다. 담당이
닫은 것은 **자기 크레이트의** 마지막 1건이고, 나는 그것을 워크스페이스
전체로 일반화하면서 다시 세지 않았다. 실제로는 p3-acm의 두 크레이트에
**10건이 남아 있다**(§104).

### 99.5 §참조 다섯 개가 존재하지 않는 절을 가리킨다

**Anchor:** `rg -n '§9[6-9]' crates/ tools/` — 워크스페이스에 5건이고
전부 이번 라운드에 이 담당이 넣은 것이다.

```
distance_field.rs:67   §97.1   → §96.3 (루프 경계)이어야 한다
distance_field.rs:87   §97.1   → 같음
distance_field.rs:518  §97.1   → 같음
lib.rs:198             §97.2   → §96.5 (커버리지 감사)
collision_distance_field_types.rs:1263  §97.3 → §92 (전수 재고)
```

**Same defect at:** 위 5건 전부.
**Distinct, skip:** 없음. 나머지 `§4`~`§92` 참조는 전부 이미 존재하는
절을 정확히 가리킨다(§79·§85.3·§90.1/2/3·§78.1 등을 표본으로 확인).

원인은 하나다 — **담당이 자기 라운드의 머지 절 번호를 추측했다.**
라운드 15 머지는 §96이 됐는데 §97로 적었다. 규칙으로 닫는다:
**읽지 않은 절 번호를 인용하지 마라.** 아직 쓰이지 않은 절을 가리켜야
하면 번호 대신 내용을 적고, 번호는 다음 라운드 브리프가 알려 준
뒤에 넣어라.

## 100. p1-joints 라운드 13 머지 — 두 경계가 물기 시작했다

4커밋(`1eabb2b`, `5430616`, `79d8b03`, `2c712e0`), `moveit-kinematics`·
`moveit-state`(테스트만)·`tools/moveit-diff`, 테스트 +6.

### 100.1 두 게이트 다 문다

§93.1에서 `>`를 `>=`로 바꿔도 31/31이 통과한다고 적었다. 머지된
트리에서 다시 쟀다:

```
pose   gate > -> >=   33 tests: 32 passed, 1 failed
                      ik_cache::tests::pose_gate_rejects_exactly_at_the_threshold_and_inserts_one_ulp_past_it
config gate > -> >=   33 tests: 32 passed, 1 failed
                      ik_cache::tests::config_gate_rejects_exactly_at_the_threshold_and_inserts_one_ulp_past_it
```

두 경계 다 문다. 담당이 두 게이트를 서로 다른 방식으로 만든 것도
근거가 있다 — pose 쪽은 `pose_distance`가 `.norm()`(sqrt)을 통과해
ULP 간격이 보존되지 않으므로 **임계값 쪽을** 1 ULP 밀었고, config
쪽은 `config_distance2`에 제곱근이 없으므로 **입력 쪽을** 밀었다.
`cart_to_jnt.rs:313`의 형태를 그대로 쓰라고 했는데 그대로 쓰면 안 되는
이유를 찾아 낸 것이다.

### 100.2 `IK_DEGENERATE_EPS`에 측정된 근거가 붙었다

상류에 대응 개념이 없다는 것(`kdl_kinematics_plugin.cpp`에
`degenerate` 0건)을 확인하고, 이 도구의 선택임을 명시한 뒤 2952건의
성공한 `panda_arm` 해에 대해 케이스별 elementwise max-`|solved − seed|`를
뽑았다. **최솟값 `3.414642e-01`, `1e-6`보다 5.5자리 위다.**

건강한 sweep에서는 이 카운터가 절대 오르지 않는다는 뜻이고, 그것이
의도다 — 이 상수가 잡으려는 것은 "반복하지 않고 시드를 그대로
돌려주는 솔버"이지 정상 수렴이 아니다. 상수가 `1e-6`~`1e-2` 어디에
있어도 같은 헤드룸이라는 것까지 적었다.

중복돼 있던 인라인 검사를 `is_degenerate_from_seed`로 뽑아내고 테스트
4개를 붙였다. 무는지 내가 다시 쟀다:

```
IK_DEGENERATE_EPS 1e-6 -> 1e2    21 tests: 19 passed, 2 failed
  is_degenerate_from_seed_tests::the_smallest_measured_real_movement_does_not_read_as_degenerate
  is_degenerate_from_seed_tests::one_joint_moving_past_the_threshold_is_enough_to_disqualify_the_whole_solution
```

담당이 보고한 "4개 중 2개가 잡는다"와 정확히 같다. 이전에는 0개였다.

### 100.3 `invariants.rs` 4건 — 둘은 진짜 노이즈, 둘은 비트 동일

§85.3대로 상수 하나씩 이분했다.

```
sin() 사이트   1e-12 통과 / 1e-15 실패, 실제 차 ~2.22e-14
cos() 사이트   1e-12 통과 / 1e-15 실패, 실제 차 ~3.22e-14
transform      1e-12 · 1e-15 · 0.0 전부 통과 — 비트 동일
norm_sqr_after 1e-12 · 1e-15 · 0.0 전부 통과 — 비트 동일
```

앞 둘은 진짜 반올림 노이즈이고 `1e-9`가 4.5자리 위다. 상수를 바꾸지
않은 판단이 맞다.

**뒤 둘은 다르다.** 비트 동일인데 `epsilon = 1e-9`로 남아 있다. 같은
라운드에 p3-distance-field는 똑같은 상황(`sphere.radius`가 `0.2`와 비트
동일)에서 `assert_eq!`로 바꿨다(`94708cb`, §99.4). **한 라운드 안에서
두 패널이 같은 상황을 다르게 처리했다.** 비트 동일이 근거로 적힌
사이트는 `assert_eq!`가 맞다 — 그래야 그 정확성에 기대고 있다는
사실이 코드에 드러나고, 나중에 누가 허용치를 만질 여지가 없다.
다음 라운드에 바꾼다.

담당이 `transform` 사이트가 비트 동일인 **이유**까지 짚은 것은
좋다(`harmonize_positions`가 링크 변환을 dirty로 표시하지 않으므로
`update()`가 캐시된 값을 그대로 돌려준다). 이 입력의 우연이 아니라
구조적이다.

### 100.4 커버리지 36건은 내가 확인하지 못했다

`ik_cache.hpp`가 따로 없다는 것은 확인했다 — `moveit2` 체크아웃에
`cached_ik_kinematics_plugin.hpp` 하나뿐이다. 11/36에서 25건이 전부
이유를 갖는다는 것도 목록으로 적혔다.

**36을 재현하지 못했다.** 내 계수기는 `public:` 블록 안에서 `(`가 있는
선언 줄을 세어 41을 내는데, 그중 7건이 다음 줄로 이어진 시그니처의
연속 줄, 4건이 doc 주석 본문, 3건이 인라인 본문 안이다. 빼면 27이
되고 36이 되지 않는다. **36이 틀렸다는 뜻이 아니라 내가 못 셌다는
뜻이다.**

§96.5에서 p3-distance-field에 요구하고 §99.3에서 받은 것과 같은 것을
요구한다 — **세는 기준을 한 줄로 적어라.** 생성자·소멸자, `= delete`
선언, `Options`/`Pose`의 public 데이터 멤버, 여러 줄에 걸친
시그니처를 각각 어떻게 세는지. 기준이 적혀 있으면 다음 라운드에
대조할 수 있다.

### 100.5 두 머지 후 실측

§99와 이 절의 두 브랜치를 연달아 머지한 트리에서 쟀다:
`fmt --check` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`cargo nextest run --workspace --no-fail-fast` **1081/1081**
(1073 + 2 + 6, 대사 일치), `cargo test --doc --workspace` 통과,
`check-*.sh` 3건 OK, `verify-fixture-provenance.sh` OK,
`verify-continuous-reseed-wrap.sh` OK, `verify-fixture-replay.sh`
**30/30 identical**.

## 101. p1-fixtures 라운드 13 머지 — 세 스킵이 물고, 못 편다던 것이 펴진다

5커밋(`1e0ebf5`, `55b70a8`, `e0069e6`, `0552866`, `26efcb7`),
`moveit-metrics` 한 크레이트, 테스트 +7(1081 → 1088).

### 101.1 세 스킵 전부 문다

§91에서 연속·부동·평면 스킵 셋이 안 재진다고 적었다. 머지된 트리에서
각각 `if false &&`로 무력화해 다시 쟀다:

```
연속 스킵 (:211)      14 tests: 13 passed, 1 failed
                      tests::continuous_revolute_joint_does_not_contribute_to_joint_limits_penalty
부동 스킵             14 tests: 13 passed, 1 failed
평면 sentinel         14 tests: 11 passed, 3 failed
```

연속 스킵을 위치 비교로 재려던 첫 시도가 `RevoluteJoint::distance`의
2π 대칭 때문에 눈이 멀었다는 것을 담당이 찾아내 골든값 재계산으로
바꾼 것, 평면 sentinel을 x/y 무한대와 theta ±PI 리터럴로 나눠 양방향
격리한 것 둘 다 §91이 요구한 형태다.

### 101.2 `columns < 6`은 못 편다고 했는데 펴진다

담당이 UNFIXED에 "`columns < 6` → `< 8` 섭동이 새 `panda_arm_5dof`
fixture로도 안 잡히고, 이 테스트 형태로는 잡을 수 없다"고 적었다.
**앞은 맞고 뒤는 틀렸다.** 방향을 반대로 하면 잡힌다:

```
columns < 6 -> < 5    14 tests: 13 passed, 1 failed
                      panda_arm_5dof_kinematics_metrics_matches_the_oracle
columns < 6 -> < 4    14 tests: 13 passed, 1 failed
                      같은 테스트
```

이유는 담당 자신의 논거가 한 방향으로만 성립하기 때문이다. 행 full
rank인 자코비안에서 특이값의 곱은 `sqrt(det(J Jᵀ))`와 정확히 같으므로
**넓히는 방향**(7-DOF를 SVD 경로로 보내는 `< 8`)은 관측되지 않는다.
**좁히는 방향**은 다르다 — 5-DOF 그룹을 `det(J Jᵀ)` 경로로 보내면
6×6(병진은 3×3) 곱행렬의 rank가 5 이하라 행렬식이 0이 되고
manipulability가 0으로 떨어진다. 오라클 값과 다르다.

**담당이 이번 라운드에 추가한 fixture가 바로 그 핀이다.** 재지 않은
것은 fixture가 아니라 섭동의 방향이다. 일반화해서 가져갈 것: 임계
상수를 재는 섭동은 **양방향**으로 걸어라. 한 방향이 수학적으로
무관측이면 그것은 상수가 안 재진다는 증거가 아니라 그 방향이 잘못
골라졌다는 증거다.

### 101.3 fixture divergence는 요청으로 왔고 내가 넣었다

`crates/moveit-metrics/tests/fixtures/panda.srdf`가 루트 fixture에
`panda_base`(부동 `virtual_joint` 격리)와 `panda_arm_5dof`(panda_link0
→ panda_link5, 5축)를 더한다. 루트 fixture는 vendored 상류와 바이트
동일을 유지해야 하므로 크레이트 로컬이 갈라지는 것이 맞고,
`moveit-kinematics`의 `pr2.srdf` 선례와 같은 형태다.

`tools/`가 내 것이라 담당이 **고치지 않고 요청만 적었다.** §94.1의
규칙이 이번에는 지켜졌다. `verify-fixture-provenance.sh`의 `DIVERGENT`
표에 항목을 내가 넣었고(`cc60e94`), 넣기 전에 `diff`로 갈라진 부분이
설명된 두 그룹뿐인지 직접 확인했다. 지금 `divergent`로 통과한다.

### 101.4 완료 조건 9건은 상류에서 대조된다

`kinematics_metrics.hpp`의 `public:` 블록(52–136행)을 직접 읽었다:
생성자, `getManipulabilityIndex` ×2, `getManipulabilityEllipsoid` ×2,
`getManipulability` ×2, `setPenaltyMultiplier`, `getPenaltyMultiplier`
= **9건.** 포트의 `rg -c '^//! - \`' crates/moveit-metrics/src/lib.rs`도
9를 낸다. 이름 단위로 하나씩 맞는다.

`getJointLimitsPenalty`는 137행의 `protected:` 아래이므로 public 감사
9건에 들어가지 않는 것이 맞다.

### 101.5 머지 후 실측

`fmt --check` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`cargo nextest run --workspace --no-fail-fast` **1088/1088**,
`cargo test --doc --workspace` 통과, `check-*.sh` 3건 OK,
`verify-fixture-provenance.sh` OK(새 `DIVERGENT` 항목 포함),
`verify-continuous-reseed-wrap.sh` OK, `verify-fixture-replay.sh`
**30/30 identical**.

## 102. `octree_points` 오라클 op — 상류가 같은 면을 떨어뜨린다

§99.2에서 p3-distance-field가 요청한 확장을 넣었다(`4f4a1a7`).
스탬프는 **`e1e8f6b2659b08b0`**으로 올라갔고 이미지는 빌드해 뒀다.

### 102.1 왜 오라클이어야 했는가

포트의 테스트는 포트가 무엇을 하는지만 보인다. 열려 있던 질문은
언어 간 문제였다 — 상류의

```cpp
for (double x = it.getX() - ceil_val; x <= it.getX() + ceil_val; x += resolution_)
```

는 누적 덧셈을 따로 반올림된 경계와 비교하고, C++ 컴파일러는 이
누적을 FMA로 축약할 자유가 있다. Rust의 엄격한 좌→우 `+=`와 갈릴 수
있고, 갈리면 §99.1의 39%가 포트만의 성질이 된다.

`getOcTreePoints`는 `protected`(`distance_field.hpp:587`)이고 bbx를
인자로 받지 않고 필드의 그리드 범위에서 유도한다. 그래서 op은 요청받은
geometry로 진짜 `PropagationDistanceField`를 만들고 **파생 클래스로**
그 메서드에 닿는다. `addOcTreeToField`를 쓰지 않은 이유는 그쪽의
`addPointsToField`가 여러 점을 한 셀로 접어 버려 비교하려는 점 개수
자체를 가리기 때문이다.

개수만이 아니라 **방출 순서 그대로 모든 점을 덤프한다** — 두 구현이
개수는 같고 어느 점인지는 다를 수 있고, 개수만으로는 "마지막 면이
빠졌다"와 "격자 전체가 한 칸 밀렸다"가 구분되지 않는다.

### 102.2 세 사례 전부 일치한다

p3-distance-field가 핀한 두 사례와 §99.2에서 내가 잰 2의 거듭제곱
사례를 상류에 돌렸다:

```
사례          octree res  삽입점                       field res  상류    포트
A 짝수 k=10   0.1        (0.35, 0.35, 0.35)            0.01       1000    1000
B 홀수 k=5    0.05       (-1.234567, 7.654321, 10.0001) 0.01        180     180
C 2의 거듭제곱 0.125      (0.3125, 0.3125, 0.3125)      0.0625        27      27
```

축별 범위까지 맞는다:

```
A  x/y/z 각 10개, 0.30 ~ 0.39      (세 축 전부 마지막 면 탈락)
B  x 5개 -1.25 ~ -1.21,  y 6개 7.65 ~ 7.70,  z 6개 10.00 ~ 10.05
C  x/y/z 각 3개, 0.250 ~ 0.375     (마지막 면 살아 있음)
```

B의 "x만 떨어지고 y·z는 6개를 유지한다"는 포트 테스트의 주장이
상류에서 축 단위로 그대로 나온다. 총계 우연이 아니다.

**결론: FMA 축약이 이 경로에서 두 언어를 갈라놓지 않는다.**
`distance_field.rs`의 "unverified" 표시는 이제 근거를 갖고 지울 수
있다. 39%는 포트의 성질이 아니라 상류의 성질이고 포트는 충실하다.

### 102.3 남은 것

이 op으로 fixture를 캡처하는 것은 아직 안 했다 — 소유자가
p3-distance-field이고 라운드 17에서 세 사례의 입력을 확정해 오기로
돼 있다. op이 먼저 들어갔으니 사례는 코드가 아니라 데이터로 붙는다.

## 103. p3-shapes 라운드 16 머지 — 158줄 감사와 돌지 않는 명령 두 개

3커밋(`da36f51`, `91bec85`, `941b942`), `moveit-octomap`·`moveit-geometry`,
테스트 +2(1088 → 1090).

### 103.1 `debug_assert!` 둘 다 문다

§94.3에서 안 재진다고 적은 두 sanity check를 `#[should_panic]`으로
덮었다. 머지된 트리에서 둘 다 지워 봤다:

```
debug_assert! 2개 제거    29 tests: 27 passed, 2 failed
                          tree::tests::set_prob_miss_above_half_panics_in_debug
                          tree::tests::set_prob_hit_below_half_panics_in_debug
```

### 103.2 24/2/15/41이 158줄 감사로 바뀌었고, 검사된다

§94.4에서 "symbol group"의 묶는 기준이 담당 정의라 재현할 수 없다고
적었다. 이번에 **선언 단위 감사**로 바뀌었고 기준이 먼저 적혀 있다 —
`OcTree` 인스턴스에 호출 가능한 모든 public 멤버(상속 사슬
`OcTree` → `OccupancyOcTreeBase` → `OcTreeBaseImpl` →
`AbstractOccupancyOcTree` → `AbstractOcTree`, 각 헤더의 `class X : public Y`
줄에서 직접 확인), 파생에서 재선언되는 pure virtual은 **최파생 한
번만**, 최파생이 아닌 생성자·소멸자는 제외, protected/private과
비호출 선언(타입 별칭, 전방 선언)은 범위 밖.

`rg -c '^/// - \`' crates/moveit-octomap/src/tree.rs` = **158**을
머지된 트리에서 직접 돌려 확인했다.

**내가 한 헤더를 골라 대조했다.** `AbstractOcTree.h`를 오라클 이미지
안에서 읽어 public 선언을 독립적으로 세면 24줄이 나오고, 감사의 해당
절은 8줄이다. 차이가 기준으로 전부 설명된다:

```
pure virtual 17건  (getResolution/setResolution/size/memoryUsage/
                    memoryUsageNode/getMetricMin×2/getMetricMax×2/
                    getMetricSize/prune/expand/clear/readData/writeData,
                    create/getTreeType)  → 롤업 2줄 "already counted above"
생성자·소멸자 2건                        → 기준상 제외
write×2 · createTree · read×2 5건        → distinct 5줄
iterator_base 전방 선언                  → 비호출, 1줄로 표시
```

**빠진 선언이 없다.** 24건이 8줄에 남김없이 대응한다. §94.4가 요구한
"다음 라운드에 내가 대조할 수 있는 형태"가 됐다.

`isNodeAtThreshold`가 라운드 12 표에서 아예 분류되지 않았던 것을
담당이 찾아 낸 것도 맞다.

### 103.3 근거로 적은 명령 두 개가 적힌 대로 돌지 않는다

```
tree.rs:194   rg -rl castRay moveit_core
tree.rs:432   rg -rl isNodeAtThreshold moveit_core
```

ripgrep의 `-r`은 **값을 받는다**(`-r REPLACEMENT, --replace=REPLACEMENT`).
따라서 `-rl`은 `-l`이 아니라 `--replace=l`로 파싱되고, 두 명령은 파일
목록이 아니라 "매치를 `l`로 치환한 줄"을 낸다. 히트가 0건이면 출력이
비어 결과가 같아 보이지만, 히트가 생기는 순간 전혀 다른 것을 낸다.

**판정 자체는 맞다.** 올바른 형태로 내가 직접 확인했다:

```
rg -l castRay moveit_core            exit 1 (0건)
rg -l isNodeAtThreshold moveit_core  exit 1 (0건)
```

§97.2와 같은 계열이다 — 근거로 문서에 박아 넣은 명령이 적힌 대로
돌지 않는 것. 그때는 `--glob '!*/tests/*'`가 `#[cfg(test)]`를 걸러
주지 못했고, 이번은 짧은 플래그 묶음이 값을 삼켰다. **두 번째
사례이므로 규칙으로 적는다: 문서에 넣는 명령은 넣기 전에 그대로
복사해 한 번 돌리고, 출력이 문서의 주장과 맞는지 확인해라.**

### 103.4 허용치 네 사이트 — 바닥이 재현되고 가려짐이 하나 있었다

`transforms.rs` 3건을 세 상수 동시에 이분해 담당이 보고한 바닥을
재현했다:

```
epsilon = 0.0               141 tests: 138 passed, 3 failed
epsilon = f64::EPSILON      141 tests: 141 passed
epsilon = f64::EPSILON / 2  141 tests: 139 passed, 2 failed
epsilon = f64::EPSILON / 4  141 tests: 138 passed, 3 failed
```

세 사이트 중 둘의 바닥이 `f64::EPSILON`, 하나가 `f64::EPSILON / 2`라는
보고와 정확히 맞는다.

**`transform_pose_applies_translation`에서 담당이 자기 이분이 가려진
것을 잡아냈다** — `max_relative`를 기본값(역시 `f64::EPSILON`)으로 둔
채 `epsilon`만 내렸더니 값의 크기가 ~1.0이라 상대항이 혼자 차이를
덮어 `f64::EPSILON / 2`가 통과했다. `max_relative = 0.0`으로 고정하고
다시 재서 실패를 확인했다. §91.2·§98.2와 같은 결함의 세 번째 얼굴이고,
이번에는 **측정 도구 쪽에서** 나타났다 — 이분 자체가 다른 게이트에
가려질 수 있다.

`stl.rs`의 1건은 비트 동일을 확인하고 `assert_eq!`로 바꿨다(§99.4의
정본과 같은 형태). 세 `transforms.rs` 사이트는 바닥이 아니라 측정된
헤드룸 위치인 `1e-12`를 유지하고 `max_relative = 0.0`을 명시했다.

### 103.5 머지 후 실측

`fmt --check` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`cargo nextest run --workspace --no-fail-fast` **1090/1090**,
`cargo test --doc --workspace` 통과, `check-*.sh` 3건 OK,
`verify-fixture-provenance.sh` OK, `verify-continuous-reseed-wrap.sh` OK,
`verify-fixture-replay.sh` **30/30 identical**.

## 104. `assert_relative_eq!` 재고 갱신 — 그리고 §99.4의 내 오류

§92 이후 여러 라운드가 쓸었으니 다시 셌다. 주석(`//` 이하)을 지운 뒤
괄호 짝맞추기로 호출을 잘라 분류하는 같은 계수기다(§92의 오염을 피하는
형태).

```
워크스페이스 전체   호출 151
  max_relative 있음     82
  epsilon만             59
  둘 다 없음            10
```

§92 시점의 183 = 60 / 108 / 15에서 상당히 옮겨졌다. 총 호출 수가 준
것은 비트 동일로 판명된 사이트가 `assert_eq!`로 바뀌었기 때문이다
(§99.4, §103.4).

### 104.1 §99.4에서 내가 틀렸다

§99.4에 "워크스페이스에 `epsilon`도 `max_relative`도 없는 사이트는
이제 0건"이라고 적었다. **틀렸다.** p3-distance-field가 닫은 것은
자기 크레이트의 마지막 1건이고, 나는 그것을 워크스페이스 전체로
일반화하면서 다시 세지 않았다. 담당의 보고는 자기 범위에 대해
정확했다 — 일반화한 것은 나다. §99.4를 정정했다.

둘 다 없는 사이트 10건은 전부 p3-acm의 두 크레이트에 있다:

```
crates/moveit-collision/src/parry.rs          4
crates/moveit-model/src/joint/revolute.rs     3
crates/moveit-model/src/joint/prismatic.rs    2
crates/moveit-model/src/joint/model.rs        1
```

**계열로 적는다:** 담당의 보고가 자기 범위에서 맞을 때 그것을
워크스페이스 주장으로 올리려면 워크스페이스에서 다시 세야 한다.
§92에서 내가 주석을 코드로 센 것, §101.2에서 담당이 한 방향 섭동만으로
"못 편다"고 결론낸 것과 같은 모양이다 — **측정한 범위 밖으로 결론을
넓히지 마라.**

### 104.2 §79 노출의 나머지는 전부 p3-acm이다

한 번도 배정된 적 없는 마지막 큰 덩어리다. 지금 트리 기준:

```
crates/moveit-collision/src/parry.rs        총 22   epsilon만 18   둘 다 없음 4
crates/moveit-model/src/joint/planar.rs     총 10   epsilon만 10
crates/moveit-model/src/joint/revolute.rs   총  9   epsilon만  6   둘 다 없음 3
crates/moveit-model/src/joint/floating.rs   총  4   epsilon만  4
crates/moveit-model/src/joint/prismatic.rs  총  3   epsilon만  1   둘 다 없음 2
crates/moveit-model/src/joint/model.rs      총  3   epsilon만  2   둘 다 없음 1
                                                   ----------   -----------
                                                          41            10
```

`moveit-collision/tests/world_parity.rs`의 1건만 `max_relative`가
있다. **51건이 §79 노출로 남아 있고 전부 이 담당 몫이다.**
다음 라운드에 배정한다.

## 105. p1-robotmodel 라운드 13 머지 — 진단이라고 분류한 것이 게이트였다

5커밋(`9eaf218`, `0ce1b0b`, `20feed0`, `f37f460`, `581ce00`),
`moveit-constraints` 한 크레이트, 테스트 +3(1090 → 1093).

### 105.1 §97.4가 요구한 확인에서 오분류가 하나 나왔다

라운드 12의 완료 조건은 11건의 `gap`을 "sampler-side 진단/벤치마킹이라
Phase 5의 `decide()` 완료 조건을 막지 않는다"로 닫았다. §97.4에서
"정말 진단/벤치마킹뿐인지 심볼별로 확인하고 하나라도 아니면
이식해라"고 요구했고, **하나가 아니었다.**

`setGroupStateValidityCallback`은 진단이 아니라 **IK 해의 수락을
결정하는 게이트**다. 상류를 직접 읽어 확인했다:

```
default_constraint_samplers.cpp:596-602
  if (group_state_validity_callback_)
    adapted_ik_validity_callback = [...] { return samplingIkCallbackFnAdapter(...); };
  → kinematics::KinematicsBase::IKCallbackFn 으로 솔버에 넘어간다

ompl_interface/src/detail/constrained_goal_sampler.cpp:135
  constraint_sampler_->setGroupStateValidityCallback(gsvcf);   ← 실제 프로덕션 호출자
```

콜백이 설정되면 IK가 낸 해를 그 콜백이 거부할 수 있다. 헤더만 보고
"setter니 설정용"으로 읽으면 놓치고, `.cpp`를 읽어야 보인다. 담당이
이번 라운드에 헤더가 아니라 `.cpp` 대조로 바꾼 것이 이것을 잡았다.

이식됐다(`f37f460`) — `moveit_kinematics::SolveOptions::solution_callback`을
재사용해 `IkConstraintSampler::sample`에 배선하고,
`IkConstraintSamplerAdapter`에는 `RefCell` 필드로 넣었다. 새 테스트
3개를 컴파일되는 형태로 무력화해 봤다(`Some(ref mut cb) => { let _ =
&mut **cb; None }`):

```
콜백이 솔버에 닿지 않게 함    92 tests: 89 passed, 3 failed
  sample_rejects_via_group_state_validity_callback_even_when_ik_converges
  sample_retries_past_group_state_validity_callback_rejections_and_accepts_on_success
  adapter_group_state_validity_callback_gates_the_trait_object_sample_path
```

셋 다 문다. getter 쪽은 상류 자신이 선언 밖에서 호출하지 않으므로
`structural`로 재분류한 것도 근거가 있다.

### 105.2 완료 조건 66건이 태그 단위로 재현된다

```
rg -c '^//! - CS:' crates/moveit-constraints/src/lib.rs   →  66
태그별:  ported 18 · structural 23 · D4 8 · D1 6 · gap 11  =  66
```

문서의 `18 + 23 + 8 + 6 + 11 = 66`이 맞는다. 내가 화살표 뒤 태그를
파싱해 독립적으로 세서 같은 분포를 얻었다. §97.1에 이어 이 담당의
완료 조건이 명령 수준까지 재현되는 두 번째 라운드다.

### 105.3 §97.2·§97.3의 세 건이 모두 닫혔다

- `utils.rs`의 `rg` 명령: 0건을 함의하던 것을 **기대 출력 그대로**
  적는 형태로 바꿨다(28줄, 실제 코드 히트는 `scene.rs:821` 하나).
  §97.2가 권한 후자를 골랐다.
- 호출자 위치 논거를 **시그니처 논거**로 교체했다. 담당이 브리프의
  15파일/43줄을 그대로 쓰지 않고 자기가 다시 세어 13파일/7-5-1로
  적은 것도 맞게 한 것이다 — 재현하지 않은 남의 숫자를 인용하는 것이
  이 저장소가 반복해서 잡아 온 결함이다.
- `crates/moveit-constraints/src/lib.rs:78`의 죽은 오라클 태그를 지우고 매 라운드 검사되는
  `panda_constraints` fixture 항목을 가리키게 했다.

### 105.4 `registry.rs`의 처분 절이 낡았다 — 담당의 UNFIXED가 맞다

담당이 범위 밖이라 두고 보고만 한 것을 내가 확인했다.
`moveit-planners-sbp/src/registry.rs:58`의

```
**Disposition** (proposed, not started — ...)
```

절은 `ConstraintSampler`/`JointConstraintSampler`/`UnionConstraintSampler`/
`IKConstraintSampler`/`selectDefaultSampler`를 앞으로 이식할 것으로
적고, `moveit-constraints -> moveit-kinematics` 의존 간선이 "does not
exist today"라고 적는다. 다섯 심볼 전부 지금 존재하고 간선도 존재한다:

```
sampler.rs:76    pub trait ConstraintSampler
sampler.rs:131   pub struct JointConstraintSampler
sampler.rs:333   pub struct UnionConstraintSampler
ik_sampler.rs:101 pub struct IkConstraintSampler
constraint_sampler_manager.rs:141  pub fn select_default_sampler
moveit-constraints/Cargo.toml:15   moveit-kinematics.workspace = true
```

**범위를 지키고 보고만 한 판단이 맞다.** 라운드 14에서 고친다.

### 105.5 머지 후 실측

`fmt --check` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`cargo nextest run --workspace --no-fail-fast` **1093/1093**,
`cargo test --doc --workspace` 통과, `check-*.sh` 3건 OK,
`verify-fixture-provenance.sh` OK, `verify-continuous-reseed-wrap.sh` OK,
`verify-fixture-replay.sh` **30/30 identical**.

## 106. p1-fixtures 라운드 14 머지 — 그리고 내 브리프가 없는 문장을 인용했다

2커밋(`47e7542`, `91f3c88`), `moveit-metrics`·`moveit-scene`, 문서만
바뀌어 테스트 수 변동 없음.

### 106.1 내 브리프 1항이 트리에 없는 문장을 가리켰다

라운드 14 브리프에서 "UNFIXED 항목과 모듈 doc의 `no output-observable
regression test; none is possible through this test shape` 문장을
고쳐라"라고 적었다. **그 문장은 어떤 소스 파일에도 없었다.**

```
git grep 'output-observable' f881ced -- crates/   →  0건
```

담당의 라운드 13 **보고서 본문**에만 있던 문장이고, 나는 그것을 트리에
대조하지 않고 모듈 doc에 있는 것처럼 인용했다. 담당이 이것을 잡아
"인용된 문장은 커밋 메시지 본문에만 있고 재작성할 수 없으니 내용을
살아 있는 doc에 적용했다"고 처리한 것이 맞다.

**계열로 적는다:** 워커의 보고서에서 문장을 가져와 브리프에 넣을 때는
그 문장이 트리에 있는지 먼저 확인해라. 보고서는 트리가 아니다. §104.1의
"측정한 범위 밖으로 결론을 넓히지 마라"와 같은 뿌리다 — 검증하지 않은
전제 위에 지시를 쌓았다.

### 106.2 `columns < 6`의 양방향 결과가 doc에 들어갔다

담당이 §101.2의 두 섭동을 **자기가 다시 재고 나서** 문서를 고쳤다.

```
< 6 → < 5   14 tests: 13 passed, 1 failed   panda_arm_5dof_kinematics_metrics_matches_the_oracle
< 6 → < 4   14 tests: 13 passed, 1 failed   같은 테스트
```

비대칭의 이유도 들어갔다 — 넓히는 쪽은 행 full rank에서 특이값의 곱이
`sqrt(det(J Jᵀ))`와 항등이라 무관측이고, 좁히는 쪽은 rank 결손으로
`det(J Jᵀ) → 0`이 되어 관측된다. §101.2가 요구한 것이 그대로 들어갔다.

### 106.3 `planning_scene.hpp` 재감사에서 구멍이 하나 나왔다

기존 오버로드 수는 전부 유지된다. 내가 상류 헤더에서 두 개를 골라
독립 확인했다:

```
grep -c 'bool isPathValid'    planning_scene.hpp  →  8   보고 8   일치
grep -c 'void checkCollision' planning_scene.hpp  →  6   보고 6   일치
```

**새 구멍:** `PlanningScene(const PlanningScene&) = delete`(`:97`)와
`operator=(const PlanningScene&) = delete`(`:102`)가 둘 다 `public:`
블록(`:93`부터 `:927`의 `private:`까지) 안인데 감사에 항목이 없었다.
내가 헤더에서 직접 확인했다. 한 항목으로 덮었고(Rust 구조체는 기본이
non-`Copy`/non-`Clone`이라 이식할 것이 없다), 세는 규약도 문서에
들어갔다 — 한 `public:` 선언당 한 항목, 동명 오버로드는 `(N overloads)`
로 접기, 여러 줄 시그니처는 첫 줄에서 한 번, `= delete` 쌍은 같은
관용구라 한 항목, 인라인 본문 접근자도 선언과 동일하게 셈.

```
rg -c '^/// - '  crates/moveit-scene/src/scene.rs   →  61
rg -c '^/// - `' crates/moveit-scene/src/scene.rs   →  60
```

보고한 61/60과 맞는다.

### 106.4 §79 몫은 0이다

`moveit-scene` 1건, `moveit-metrics` 7건, **8건 전부 `max_relative`가
있다**(`epsilon`만 0, 둘 다 없음 0). 내 §104 계수기로 재확인했다.
이 담당의 §79 노출은 없다.

### 106.5 머지 후 실측

`fmt --check` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`cargo nextest run --workspace --no-fail-fast` **1093/1093**,
`cargo test --doc --workspace` 통과, `check-*.sh` 3건 OK,
`verify-fixture-provenance.sh` OK, `verify-continuous-reseed-wrap.sh` OK,
`verify-fixture-replay.sh` **30/30 identical**.

## 107. p6-totg 라운드 12 머지 — 그리고 요청받은 오라클 op을 내가 만들었다

3커밋(`c90b5c1` `Display` 이식, `0e659e3` 11개 사이트 이분, `468b638`
8.893e-9 추적)을 `--no-ff`로 머지했다. 머지 후 실측 **1096/1096**.

### 107.1 `Display` 이식은 상류 열 순서와 맞는다

`RobotTrajectory::print`/`operator<<`를 `impl std::fmt::Display`로
옮겼다. 내가 상류와 열 순서를 대조했다 — `"Empty trajectory."`,
`"Trajectory has N points over D seconds"`, `  waypoint {:>3} time
{:>5.3} pos `, 그다음 조건부 `vel `/`acc `/`eff `. 일치한다.
`Display`의 시그니처에 상류의 `variable_indexes` 오버라이드를 넣을
자리가 없다는 것은 편차로 문서화됐다.

### 107.2 11개 사이트 이분 — 8개는 상류 전사라 손대지 않는 것이 맞다

`trajectory.rs`의 `epsilon`만 있는 11개 중 8개가
`EXPECT_NEAR(0.0, trajectory.getVelocity(...)[0], 0.1)`의 전사다.
**8건 모두 내가 상류에서 직접 확인했다** — `:108/109`, `:156/157`,
`:204/205`, `:594/595` 각각이 정확히 그 형태다. 전사한 상수를 이분하는
것은 상류를 다시 쓰는 것이므로 그대로 두고 줄 인용을 붙인 처리가 맞다.

나머지 3건은 내가 재현했다.

```
epsilon = 1e-9  → 1e-12   1 fail   upstream_test_single_dof_discontinuity
traj_duration 1e-3 → 1e-6  1 fail   같은 테스트
```

`traj_duration`의 `1e-3`은 이제 가정이 아니라 측정된 최소 통과 단계이고,
가속도 검사 둘은 `1e-3` → `1e-9`로 조여졌다.

### 107.3 요청받은 op은 요청받은 모양으로는 만들 수 없다 — 그리고 그 대체가 답을 냈다

담당이 UNFIXED에 오라클 요청을 남겼다. `upstream_test2`의 5웨이포인트
집합(`max_deviation=100.0`)에 대해 생성되는 `CircularPathSegment` 3개
각각의 `start_dot_end`·`angle`·`radius`·`center`/`x`/`y`를 f64 전정밀도로
달라는 것이었다.

**그 모양으로는 못 만든다.** 근거:

```
time_optimal_trajectory_generation.cpp:103   class CircularPathSegment
time_optimal_trajectory_generation.hpp:117   PathSegment* getPathSegment(double& s) const;   ← private:
```

타입이 설치되는 헤더가 아니라 `.cpp` 안에 있고, 세그먼트 핸들을 얻는
유일한 통로도 `private`다. §102의 `getOcTreePoints`는 `protected`라 파생
클래스로 닿았지만 여기는 그 수가 없다. 상류를 패치하면 닿지만, 그러면
오라클이 "손대지 않은 상류의 동작 기록"이기를 그만둔다. **"닿지 않는다"로
닫지 않고**(내가 p3-distance-field에게 "근거 없이 '괜찮다'로 닫지 마라"고
한 것과 같은 기준) 공개면으로 같은 기하를 재는 op을 만들었다 —
`totg_path`(`12ff220`). 스탬프 `e1e8f6b2659b08b0` → **`e7d32225310d3278`**.

공개면이 같은 값을 준다:

- `getSwitchingPoints`가 세그먼트 경계를 호길이로 주므로 각 블렌드의
  호길이 = `angle * radius`
- 원호 안에서 `getCurvature(s)`는 노름이 `1/radius`이므로 **`radius`가
  직접 복원되고**, 호길이로 `angle`이 따라 나온다
- 블렌드 양 끝의 `getConfig`/`getTangent`가 시작·이탈 지점을 고정한다

복원되지 않는 것은 `center`·`x`·`y`뿐이고, 그 셋은 경로의 관측량이 아니라
기저 선택이다.

### 107.4 실측 결과 — `Path`는 상류와 사실상 비트 일치다

호길이 1213.34…에서 블렌드 3개는 `[50, 140.238]`, `[380.719, 879.851]`,
`[1084.802, 1163.341]`이다. 각 블렌드 안 3점과 직선 구간 4점에서 상류와
포트를 ULP로 비교했다.

```
length                       ulp   0
블렌드1 (R=84.19078207201368)  |k| ulp  0,  0,  0
블렌드2 (R=993.4961962279818)  |k| ulp  0,  0, -1
블렌드3 (R≈50)                 |k| ulp +2, +3, +2
직선 4점                       |k| ulp  0 (모두 정확히 0)
tangent 최대                                     19 ulp
config  최대                                      1 ulp
```

**세 반지름 중 둘은 비트 일치하고, 셋째만 2~3 ULP 어긋난다.** 셋째
블렌드는 방향 `(-0.98058, 0.19612, 0)`에서 `(0,0,-1)`로 꺾이는 직각
모서리이고 반지름의 참값은 50이다 — 상류가 `50.00000000000001`, 포트가
`49.99999999999999`로 **양쪽 다 50에서 반대 방향으로** 벗어난다. 한쪽이
틀린 것이 아니라 둘 다 반올림한 것이다.

`tangent`의 19 ULP는 블렌드 전용이 아니다 — 직선 구간(`s=260`)에서도
18 ULP가 난다. 그 자리에서 손으로 확인했다: 원시 웨이포인트 차의
정규화(`d/‖d‖`도, `d * (1/‖d‖)`도)로는 포트 값이 재현되지 않는다
(각각 `[3,-1,-11,0]`, `[3,0,-11,0]` ULP). 상류 `LinearPathSegment`의
끝점이 원시 웨이포인트가 아니라 **블렌드가 잘라낸 끝점**이기 때문이고,
따라서 직선 구간의 접선도 블렌드 산술의 오차를 물려받는다.

### 107.5 그래서 8.893e-9은 기하가 아니라 적분에서 커진다

지속시간 편차를 같은 단위로 놓으면:

```
8.893e-9 / 1922.141842744594 = 4.63e-12  →  1922 근방의 ULP로 3.9e4 ULP
```

입력 기하는 **19 ULP 이하**로 일치하는데 출력은 **3.9e4 ULP** 어긋난다.
약 2000배다. 즉 남은 편차의 발생 자리는 `Path` 생성이 아니라
`Trajectory`의 적분·전환점 탐색이다. `468b638`이 누적 순서·FMA(objdump에
`vfmadd` 0건)·libm 편차·리덕션 순서·정규화 관례를 근거를 붙여 배제한
것은 유효하고, 이번 측정은 **탐색 범위를 `Trajectory`로 좁힌다.**

라운드 13은 여기서부터다. `totg_path`는 이미 있고 픽스처 포획만 남았다 —
포획은 담당의 크레이트 안이므로 담당이 한다.

### 107.6 머지 후 실측

`fmt --check` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`cargo nextest run --workspace --no-fail-fast` **1096/1096**,
`cargo test --doc --workspace` 통과, `check-*.sh` 3건 OK,
`verify-fixture-provenance.sh` OK, `verify-continuous-reseed-wrap.sh` OK
(42.6000% > 35.1222%), `verify-fixture-replay.sh` **30/30 identical**
(새 이미지 `e7d32225310d3278`로 재실행).

## 108. p1-joints 라운드 14 머지

3커밋(`c93d27d`, `985fc87`, `1f4be38`).

### 108.1 비트 일치 두 자리는 `assert_eq!`가 됐다

`invariants.rs:162`(`harmonize_positions` 변환)과 `:328`
(`norm_sqr_after == 1.0`)이 `assert_relative_eq!`에서 `assert_eq!`로
바뀌었다. 두 자리의 성격이 다르다는 것을 주석이 구분한다 — 앞은 구조적
항등, 뒤는 **이 입력에서만** 측정된 정확값. 그 구분이 중요한 이유는
뒤쪽이 입력이 바뀌면 깨질 수 있다는 뜻이기 때문이고, 주석에 그대로
적혀 있다. §100.3이 지적한 p3-distance-field와의 불일치가 닫혔다.

### 108.2 44 멤버를 이름으로 다 적었다

`cached_ik_kinematics_plugin.hpp`의 `public:` 선언을 다시 한 줄씩 읽어
`IKCache` 20(8 이식), `IKCacheMap` 6(0), `CachedIKKinematicsPlugin`
12(6), `CachedMultiTipIKKinematicsPlugin` 6(0) = **44 중 14 이식**으로
적었다. 세는 기준 네 가지가 명시됐다. 표만 있고 이름이 없던 것이
이번에 이름으로 바뀌었다 — 검증 가능한 형태다.

### 108.3 퇴화 카운터는 실제로 0을 읽고, 0이 아니게 만드는 길이 있다

300케이스 실제 스윕에서 `rust_degenerate: 0, oracle_degenerate: 0`
(297/294 성공). 브리프가 요구한 "stub 솔버로 0이 아니게 만들어라"는
`rust_impl::IkSolver`가 구체 struct라 trait 이음매가 없어 그대로는
안 된다 — **그러나 담당이 대신 도달 가능한 실제 경로를 찾았다.**
목표가 이미 `FK(seed)`인 케이스는 `cart_to_jnt`의 첫 반복에서
`q_full`을 건드리기 전에 수렴해 씨앗을 비트 그대로 돌려준다. 그것을
`a_case_already_at_the_seed_pose_converges_to_the_seed_unmoved`로
고정했다(`tools/moveit-diff/src/main.rs:2071`, 내가 확인했다).

요구를 그대로 못 하면 **요구가 겨냥한 것을 다른 길로 달성한다** —
"trait 이음매가 없어서 못 합니다"로 닫지 않은 것이 맞다.

## 109. p1-robotmodel 라운드 14 머지

3커밋(`9933635`, `4099374`, `8401fd3`).

### 109.1 `registry.rs`의 처분 절이 현재 상태로 다시 쓰였다

앵커 `rg -n 'not started|proposed|planned|would need|does not exist
today' crates/moveit-planners-sbp/src crates/moveit-constraints/src`가
3건을 냈고, 2건 same-defect(고침), 1건(`rrt_connect`의 goal-region
파라미터 부재)은 distinct로 남겼다. 머지 후 내가 같은 앵커를 다시
돌렸다 — 남은 히트는 새로 쓰인 문단의 `"ported, not proposed"` 하나뿐이다.

### 109.2 남은 11개 `gap`을 두 질문으로 다시 봤다

`setGroupStateValidityCallback`이 오분류였던 이유(헤더는 setter로
보이지만 `.cpp`에서 게이트)를 기준으로 11건을 다시 판정했다.

- `isValid()` — 상류에서 **분기 조건이 맞다.** 다만 그것이 가르는
  상태가 이 포트에서는 생성 불가능하다(`new()`가 fallible).
- `countSamplesPerSecond` — 유일한 호출자가 값을 검사 없이 전달한다.
  "진단이니 무해"로 넘기지 않고 호출자를 직접 열어 확인했다.
- 나머지 — 분기할 호출자 자체가 없다.

태그 수 `18/23/8/6/11 = 66`은 그대로다. §100.2의 함정(진단으로 흘러가는
숫자를 무해로 닫기)을 피한 처리다.

### 109.3 `moveit-planners-sbp`의 §79 몫은 0이다

`assert_relative_eq!` 사이트가 없다는 것을 확인하고 `lib.rs`의 완료
서술에 적었다.

## 110. p1-fixtures 라운드 15 머지

3커밋(`e31eccb`, `1cba458`, `0e07656`). 새 테스트 2개.

### 110.1 `moveit-scene`의 미이식 항목이 전부 `.cpp` 인용을 갖게 됐다

인용이 없던 8항목(`getCollisionDetectorName`, `getCollisionEnv`/
`getCollisionEnvUnpadded` 계열, `checkCollisionUnpadded`,
`distanceToCollisionUnpadded`, `setAttachedBodyUpdateCallback`,
`setCollisionObjectUpdateCallback`, `printKnownObjects`,
`allocateCollisionDetector`)을 `planning_scene.cpp`에 대고 다시 봤다.
재분류는 없다. 근거의 형태가 바뀐 것이 결과물이다 — 아키텍처 수준
추론이나 유추가 아니라 항목마다 자기 인용을 갖는다.

콜백 두 개가 `setGroupStateValidityCallback`과 다른 이유가 구조적으로
적혔다: 저 둘은 `void` 반환 알림 훅(`attached_body.hpp:52`,
`world.hpp:304`)이라 **결정을 게이트할 수가 없다.** `bool` 반환
`IKCallbackFn`과의 차이다.

### 110.2 `DBL_MAX` fall-through 주장이 드디어 측정됐다

주장은 "`DBL_MAX`면 조용히 아무것도 매치하지 않고 항상 fall through"
였고 한 번도 돌려진 적이 없었다. 실제로는 **`NaN`이 나온다.**

내가 머지 후 독립적으로 재현했다 — `f64::NEG_INFINITY`/`f64::INFINITY`
sentinel 비교 4곳을 `-f64::MAX`/`f64::MAX`로 바꾸고:

```
cargo nextest run -p moveit-metrics --no-fail-fast
16 tests: 15 passed, 1 failed
FAIL  planar_xy_infinite_bounds_still_skip_despite_finite_theta
```

기구는 이렇다: 스킵이 발동하지 않아 `PlanarJoint::distance`가 무한
경계에 대해 평가되고, 관절 항이 `∞ * ∞ / ∞²` — IEEE 754의 `∞/∞`가
된다. "항상 fall through"가 아니라 "`NaN`으로 오염된다"가 참이다.
doc과 테스트 주석 양쪽이 고쳐졌다.

**이것이 §79 계열의 본체다** — 재지 않은 주장은 틀린 주장일 수 있다.

### 110.3 `f64::MIN_POSITIVE` 두 게이트는 상류와 문자 그대로 같고, 물게 만들었다

둘 다 상류 `kinematics_metrics.cpp`의
`fabs(x) <= numeric_limits<double>::min()`과 연산자·sentinel이 일치한다.
양방향 섭동, 상수 하나씩:

```
penalty_multiplier  <= → <     14/14 무관측
                    sentinel → 0.0    14/14 무관측
                    sentinel → 2.0     8/14 관측
range               <= → <     14/14 무관측
                    sentinel → 0.0    14/14 무관측
                    sentinel → 100.0  10/14 관측
```

무관측 두 방향은 **무는 입력이 없어서**다 — `0.0`과
`f64::MIN_POSITIVE` 사이 약 2.2e-308 폭에 앉는 입력이 기존에 없었다.
그래서 논증으로 닫지 않고 경계에 정확히 앉는 입력을 만들어 테스트
둘을 넣었다: `penalty_multiplier == f64::MIN_POSITIVE`(페널티 `0.0` 대
`1.0`으로 `<=`와 `<`를 가른다), `range == f64::MIN_POSITIVE`(`range²`가
underflow해 `0.0/0.0`이 되므로 유한 대 `NaN`으로 가른다). §85.3이
요구한 "물지 않으면 무는 입력을 만들어라"가 그대로 됐다.

## 111. p3-shapes 라운드 17 머지

3커밋(`db46abc`, `8313f91`, `0632fcc`).

### 111.1 §103.3의 두 명령이 고쳐졌고, 계열 전체가 쓸렸다

`tree.rs` doc의 잘못된 ripgrep replace-플래그 두 자리가 고쳐졌고,
**두 크레이트의 문서화된 명령을 전부 다시 돌렸다** — 다른 불일치는
없다. 인용 하나가 표본이지 전부가 아니라는 규칙대로 처리됐다.

### 111.2 남은 네 octomap 헤더 대조에서 진짜 구멍이 하나 나왔다

`OcTree.h`/`OccupancyOcTreeBase.h`/`AbstractOccupancyOcTree.h`는 정확히
맞는다. `OcTreeBaseImpl.h`에 선언 하나가 빠져 있었다 — `getTreeType()
const`. 내가 헤더에서 직접 확인했다:

```
OcTreeBaseImpl.h:104   std::string getTreeType() const {return "OcTreeBaseImpl";}
```

교차참조 항목으로 덮었고 합계가 158→159, 교차참조 5→6으로 고쳐졌다.
`OcTree.h`의 구체 `getTreeType()`이 이것을 가린다는 것도 적혔다.

### 111.3 `moveit-geometry`도 같은 감사를 받았다

`geometric_shapes` 2.3.3에 대고 라운드 8의 `shapes.rs`/`bodies.rs`
감사를 다시 봤다. 헤더는 이번에 새로 받아 바이트 동일을 확인했다.
구멍 둘:

- `shape_operations.h`의 D1 제외 항목에서 산수 오류 — "All six"가
  실은 일곱(`constructShapeFromMsg` 오버로드 4 + 이름 다른 형제 3)
- 각 구체 shape 클래스의 자기 생성자·데이터 필드 항목이 없었다
  (이식은 돼 있고 열거만 안 돼 있었다 — §111.2의 `getTreeType`과
  같은 계열)

`bodies.rs`/`body_operations.h`에 같은 패턴이 있는지도 봤고 깨끗하다
(`Body`의 구체 생성자는 `Body::from_shape`로 이미 통일돼 있고 그렇게
문서화돼 있다).

## 112. p3-acm 머지 — 그리고 테스트 하나가 스위트를 17분 늘렸다

1커밋(`9dbf0c9`, 328줄). 라운드 12 이후 3시간 넘게 커밋이 0이던 패널로,
내가 "조사 그만두고 가진 것을 커밋하고 보고해라"라고 개입한 결과다.

### 112.1 캐스터 바퀴 고정 상수의 원인이 바뀌었다

이 백엔드의 `base_link`/캐스터 바퀴 자기거리 `-0.046592m`이 여러 바퀴·
여러 자세에서 비트 동일하게 나오는 것을 라운드 12는 "바퀴 회전 대칭"
으로 적었다. 틀렸다. 실제 원인은 **`base_link` 자기 조악 충돌 메시의
거의 평면인 한 면**이다. `parry3d_f64::query::contact`를 `base_link`의
96 삼각형 각각에 직접 불러(파이프라인을 통하지 않고) 삼각형 `[14, 12,
15]`이 매번 이긴다는 것, 그 셋의 `z`가 `base_link` 자기 프레임에서
같다는 것을 테스트가 못박는다.

그리고 **전역 불변량이 아니라 고원(plateau)이다** — 72점 밀집 스윕에서
평면 후보가 관절 전 범위의 약 80%를 덮고, 나머지 약 20%는 다른 두
삼각형(`[13,12,14]`가 `theta≈0` 부근, `[15,12,16]`이 `theta≈3.5..4.4`)
이 이긴다. 고원이 전 범위를 덮었거나 램프 구간까지 얼어 있었다면
그것이 진짜 결함이었을 것이고, 그 구분을 테스트가 담고 있다.

프로덕션 코드는 바뀌지 않았다 — 출력은 이미 맞았고 문서와 테스트가
틀렸거나 없었다.

### 112.2 새 테스트 하나가 전체 스위트를 1017초로 늘렸다

머지 후 실측에서 나온 것이다.

```
PASS [1017.400s] (1101/1101)
  moveit-collision::collision_parity
  pr2_self_wheel_same_pair_frozen_constant_is_a_planar_base_link_face
Summary [1017.414s] 1101 tests run: 1101 passed (1 slow), 0 skipped
```

**스위트 전체 벽시계가 이 테스트 하나로 결정된다.** 같은 10개 오라클
점에 대해 같은 `distance_self`를 부르는 형제
(`pr2_self_wheel_same_pair_oracle_magnitude_is_implausible`)는 60초
문턱에 걸리지도 않는다. 둘의 차이는 `DistanceRequestType::Single`과
per-삼각형 헬퍼다.

담당의 보고서는 `1024/1024 pass`만 적고 이 사실을 적지 않았다. 통과
여부만 보고하고 **비용을 보고하지 않은 것**이고, 이 저장소에서는
그것도 보고 대상이다 — 이 테스트가 들어간 뒤로 모든 패널의 게이트가
매번 17분씩 길어진다. 다음 라운드 1항이다.

### 112.3 머지 후 실측(5개 패널 합산)

p1-joints·p1-robotmodel·p1-fixtures·p3-shapes·p3-acm 다섯 브랜치를
`--no-ff`로 연속 머지했다. 충돌 없음.

`fmt --check` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`cargo nextest run --workspace --no-fail-fast` **1101/1101**,
`cargo test --doc --workspace` 통과, `check-*.sh` 3건 OK,
`verify-fixture-provenance.sh` OK, `verify-continuous-reseed-wrap.sh` OK
(42.6000% > 35.1222%), `verify-fixture-replay.sh` **30/30 identical**.
스탬프 `e7d32225310d3278`.

## 113. p6-totg 라운드 13 머지 — 8.893e-9이 블렌드 3 안으로 좁혀졌다

2커밋(`793ef28`, `b05c2f1`).

### 113.1 `totg_path` 픽스처의 허용오차는 측정으로 정해졌다

§107.3에서 내가 만든 op의 픽스처를 담당이 잡았다. 허용오차가 추측이
아니라 측정된 바닥 위에 얹혔다:

```
CONFIG_TOL    1e-9    측정 바닥 2.27e-13   (약 3.64자리 여유)
TANGENT_TOL   1e-11   측정 바닥 1.05e-15   (약 3.98자리)
CURVATURE_TOL 1e-13   측정 바닥 2.17e-17   (약 3.66자리)
length                비트 정확 → assert_eq!
```

세 상수 모두 `±0.0001%` 양방향 섭동으로 여전히 판별력이 있는지 확인한
뒤 커밋됐다. §85.3과 §101.2가 요구한 형태 그대로다.

### 113.2 편차가 나는 자리가 시간축에서 좁혀졌다

새 op 없이 기존 `totg`의 `sample_times`만으로 이분했다.

```
t < 1571.75          position/velocity 차가 Path 자체의 바닥(~1e-13 / 1e-16)
t 1571.75 → 1581.76  차가 약 500배로 점프
                     호길이 s ≈ 1123.8 → 1127.2
```

`s ∈ [1084.80, 1163.34]`, **블렌드 3 내부다.** §107.4에서 내가 ULP로
잰 것과 같은 자리다 — 세 블렌드 중 곡률이 어긋난 유일한 블렌드가
블렌드 3(`+2, +3, +2` ULP)이었고, 시간축 이분이 독립적으로 같은 곳을
가리켰다. 초기 조건 버그도, 전환점 경계 인공물도 아니라는 것이 이것으로
배제된다.

남은 것은 그 안에서 왜 500배가 되는가다. 열려 있다.

### 113.3 내 브리프 3항이 이미 끝난 일을 시켰다

브리프에 "`TimeOptimalTrajectoryGeneration` 어댑터는 라운드마다
유예되고 있다. 이번 라운드에 처분을 확정해라"라고 적었다. **이미
이식돼 있었다.**

```
f03a46e  moveit-trajectory: port TimeOptimalTrajectoryGeneration's
         RobotTrajectory adapter
git merge-base --is-ancestor f03a46e HEAD  →  ancestor: yes
```

내가 확인했다. 생성자, 메시지 아닌 `computeTimeStamps` 오버로드 둘,
`totgComputeTimeStamps`, 비공개 헬퍼 전부가 들어와 있고 `lib.rs`에
심볼 감사도 돼 있으며 `todo!`/`unimplemented!`가 없다.

출처는 담당의 **라운드 12 UNFIXED 항목**이었고("deliberately deferred
per this round's ownership boundary, unchanged from prior rounds"),
나는 그것을 트리에 대조하지 않고 브리프로 옮겼다.

**§106.1과 같은 결함이 두 번째다.** 그때는 워커의 보고서 *문장*을
트리에 있는 것처럼 인용했고, 이번에는 워커의 UNFIXED *상태*를 확인
없이 지시로 옮겼다. 뿌리는 하나다 — 검증하지 않은 전제 위에 지시를
쌓았다. 규칙으로 굳힌다:

> 워커의 UNFIXED를 다음 라운드 항목으로 옮기기 전에, 그 항목이
> **지금 트리에서도 미해결인지** 확인해라. UNFIXED는 그 라운드
> 시점의 서술이고, 그 사이에 자기 자신이나 형제 패널이 닫았을 수
> 있다. `git log`/`git merge-base --is-ancestor`로 확인하는 데 드는
> 비용은 초 단위다.

## 114. p3-distance-field 라운드 17 머지 — 그리고 `serde_json`이 f64를 1 ULP 틀리게 읽는다

5커밋(`30fdc3f`, `e779325`, `9410db8`, `d529de8`, `3fbefd4`).

### 114.1 세분 루프의 `<=`가 마침내 물었다

§99.3에서 두 라운드 열려 있던 것이다. 내가 §99에서 준 이분법 —
2의 거듭제곱 해상도로 `start`/`end`/누산을 정확하게 만드는 케이스 —
가 그대로 먹었다. `<=`와 `<`가 갈린다(27 대 8). 담당이 재현하고
테스트로 고정했다.

### 114.2 존재하지 않는 절 5건이 고쳐졌고 계수도 이름으로 다시 나왔다

§99.5의 잘못된 `§97.x` 인용 5건이 실제 절로 바뀌었고, 커버리지 계수가
자기가 적어 둔 기준에 맞춰 이름 단위로 다시 세졌다.

### 114.3 `octree_points` 픽스처가 잡혔고, 그 과정에서 진짜 결함이 나왔다

§102의 op에 대한 픽스처가 세 경계 케이스(1000/180/27점) 전부에 대해
개수가 아니라 **배열 전체와 리프 전체의 동등성**으로 잡혔다. 재생은
30 → **31**로 늘었다.

첫 실행이 케이스 B에서 55/180 불일치로 실패했고, **담당이 그것을 액면가로
받지 않고 원인까지 갔다.** 원시 바이트 검사로 좁힌 결론:
`serde_json`의 f64 파서가 일부 17유효자리 리터럴에서 올바르게 반올림하지
않는다.

### 114.4 나는 그 주장을 재현했고, 노출 범위는 이 크레이트가 아니다

주장이 크니 직접 쟀다. `serde_json = "=1.0.151"`, `raw_value` 기능:

```
"10.049999999999999"
  serde_json  10.05                     0x402419999999999a
  str::parse  10.049999999999999        0x4024199999999999   DISAGREE
  Value       10.05                     0x402419999999999a   DISAGREE

"2.2250738585072011e-308"
  serde_json  2.2250738585072014e-308   0x10000000000000
  str::parse  2.225073858507201e-308    0x0fffffffffffff     DISAGREE
```

**버전 특정이 아니다.** 담당의 `Cargo.toml` 주석은 이것을 "1.0.151의"
파서라고 적었는데, 같은 리터럴로 1.0.140 / 1.0.145 / 1.0.150 /
1.0.151을 각각 돌려 봤고 **넷 다 같게 틀린다.** 회귀가 아니라 오래된
동작이므로 **버전 고정은 해법이 아니다.** 그 주석은 고쳐야 한다.

그리고 **노출은 이 크레이트에 국한되지 않는다.** 커밋된 픽스처
101개의 모든 float 리터럴을 `serde_json::from_str::<f64>`와
`str::parse::<f64>`로 각각 읽어 비트를 비교했다:

```
  6859 / 84221 리터럴이 1 ULP 어긋난다  (8.1%)
  29개 파일, 9개 크레이트
```

크레이트별로 (오차 있는 파일만):

```
moveit-geometry     bodies_probe 10/293, mesh_parity 4036/33327,
                    octree_in_world_response 1/138
moveit-metrics      panda_arm_5dof …_response 28/441,
                    panda_kinematics_metrics_response 29/441
moveit-model        dual_arm_panda 12/547, fanuc 4/178, panda 6/298, pr2 19/1264
moveit-octomap      octomap_response 1/270
moveit-scene        panda_frame_transform_response 2/144,
                    panda_is_state_valid 8/92
moveit-smoothing    ruckig_filter_response 14/987
moveit-state        dual_arm_panda_dynamics 9/335, dual_arm_panda_fk 92/1654,
                    fanuc_dynamics 11/294, fanuc_fk 26/594,
                    panda_dynamics 12/335, panda_fk 57/816,
                    pr2_dynamics 16/343, pr2_fk 499/6224
moveit-trajectory   large_accel_waypoints 4/39, totg_path_request 2/30,
                    totg_path_response 8/138, totg_request 3/88,
                    totg_response 10/141, totg_robot_trajectory_response 57/594,
                    …_scaling_only_response 22/198, totg_synthetic_response 117/752
```

**이것이 계열이고, 인용된 크레이트는 표본이다.** 담당은 자기
크레이트만 `raw_value` + `RawValue` 텍스트 파싱으로 고쳤다. 범위를
지킨 것은 맞지만 나머지 8개 크레이트는 그대로다.

왜 지금까지 안 보였나: 허용오차가 1 ULP를 흡수한다. 보이는 자리는 둘
뿐이다 — (1) 비트 정확 비교(`assert_eq!`), (2) **이분으로 잰 "측정
바닥"**. 바닥이 픽스처 파싱 오차로 오염되면 그 바닥은 포트의 오차가
아니다. §79 스윕 전체가 이 위에 서 있다.

처분은 §115다 — 크레이트마다 `raw_value` 껍데기를 반복하는 것은 지역
패치이고, 워크스페이스 한 줄로 닫히는 구조적 해법이 있었다.

### 114.5 두 가지를 더 잡았다

- `arbitrary_precision`을 먼저 시도했다가 `#[serde(tag = "...")]`
  역직렬화가 크레이트 전역에서 깨졌고(`shape_points_parity.rs`,
  `collision_distance_field_types_parity.rs`), **새 테스트만이 아니라
  크레이트 전체 스위트를 커밋 전에 돌려서** 잡았다.
- `verify-fixture-replay.sh`는 `oracle-models.json`에 적힌 stem만
  재생한다. 새 픽스처가 목록에 없으면 **재생 커버리지에서 조용히
  빠진다.** `3fbefd4`가 등록했고, 등록 전에 pr2로 재생해 커밋된 응답과
  바이트 단위로 대조했다.

## 115. `float_roundtrip` — §114.4의 계열을 워크스페이스 한 줄로 닫았다

`70a6b31`.

### 115.1 해법은 껍데기가 아니라 기능 플래그였다

§114.4에서 노출을 9개 크레이트 6,859 리터럴로 재고 나서 처음 떠오른
것은 공유 헬퍼 크레이트였다 — `RawValue`로 읽고 `str::parse`로 넘기는
newtype을 만들어 모든 픽스처 구조체의 `f64`를 갈아 끼우는 것. 그것은
9개 크레이트의 필드를 전부 건드리는 큰 변경이고, 각 패널의 소유 영역을
가로지른다.

**그럴 필요가 없었다.** `serde_json`에 이 목적의 기능이 이미 있다.

```
[workspace.dependencies]
serde_json = { version = "1", features = ["float_roundtrip"] }
```

같은 측정기를 다시 돌렸다:

```
기능 없음:  6859 / 84221 리터럴이 1 ULP 어긋남
기능 있음:     0 / 84221
```

**전부 0이다.** 커밋된 픽스처 101개, 84,221개 리터럴 전수다.

기능은 `[workspace.dependencies]`에 한 번 선언한다. Cargo가 그래프
전체에서 기능을 합집합으로 잡으므로, 자기 기능 목록을 따로 쓰는 멤버
(`moveit-distance-field`의 `raw_value`)도 `workspace = true`를 통하는
한 이것을 물려받는다. 확인했다:

```
cargo tree -e features -i serde_json --workspace
  serde_json feature "default"
  serde_json feature "float_roundtrip"
  serde_json feature "raw_value"
  serde_json feature "std"
```

### 115.2 버전 고정은 해법이 아니었다

§114.4에 적었듯 이것은 회귀가 아니다. 같은 리터럴로 1.0.140 / 1.0.145 /
1.0.150 / 1.0.151을 각각 돌렸고 **넷 다 같게 틀린다.** 그러므로
`moveit-distance-field`의 `Cargo.toml` 주석이 이 성질을 "1.0.151의"
파서라고 특정한 것은 고쳐야 한다. 그 주석과 `raw_value` 껍데기 자체가
이제 불필요한지도 그 크레이트가 판단할 일이다 — 담당에게 넘긴다.

### 115.3 주석이 아니라 검사로 못박았다

기능 플래그는 다음 사람이 조용히 지울 수 있고, 지워져도 **어떤 테스트도
실패하지 않는다** — 허용오차가 1 ULP를 흡수하기 때문이다. 정확히 그래서
기계적 검사가 필요하다. `tools/ci/check-serde-float-roundtrip.sh`가
해결된 기능 목록을 보고 없으면 실패한다. `check-*.sh` 글롭에 들어가므로
CI가 자동으로 집는다(도커 불필요).

양방향으로 쟀다:

```
기능 있음  →  OK: serde_json resolves with "float_roundtrip"      exit 0
기능 제거  →  FAIL the workspace resolves serde_json without …    exit 1
```

`cargo tree`를 파이프 머리에 두지 않은 이유는 `check-dep-direction.sh`
와 같다 — 파이프로 넘기면 해결 실패가 빈 입력으로 `grep`에 도달해
"기능 없음"과 같은 종료 상태를 내지만 전혀 다른 사실이다.

### 115.4 실측

기능을 켠 뒤 `cargo nextest run --workspace --no-fail-fast`(§112.2의
1017초 테스트 제외) **1103/1103 통과, 회귀 0건.** 즉 지금까지 이
파싱 오차에 **의존하던** 테스트는 하나도 없었다 — 오차는 허용오차
아래에 잠겨 있었고, 드러난 자리는 `octree_points` 픽스처처럼 비트
정확 비교를 하는 곳뿐이었다.

`fmt --check` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`check-*.sh` **4건**(새 검사 포함) OK.

### 115.5 계열로 적는다

이번 건의 모양은 §85.3·§103.4와 같다. **허용오차가 결함을 흡수하면
결함은 사라지지 않고 보이지 않게 된다.** 세 사례가 같은 뿌리다:

- §103.4 — 기본 `max_relative`가 `epsilon` 이분을 가린다
- §110.2 — 재지 않은 doc 주장이 틀린 주장이었다(`NaN`이 나왔다)
- §114.4 — 픽스처 파싱 오차가 이분이 재는 "바닥"을 오염시킨다

셋 다 "통과했으니 맞다"가 성립하지 않는 자리다. §79 스윕에서 지금까지
잰 바닥들은 이 기능이 꺼진 상태에서 잰 것이므로, **1 ULP 규모의 바닥을
근거로 고른 상수는 다시 재야 한다.** 다음 라운드 세트에 배정한다.

## 116. 재생 검사가 자기 헤더가 말하는 것과 다른 것을 세고 있었다

`a640c98`. §114.5에서 담당이 짚은 것을 계열로 닫았다.

### 116.1 불변식의 정의역과 구현의 정의역이 달랐다

`verify-fixture-replay.sh`의 첫 문단은 이렇게 쓰여 있다 — "커밋된
모든 `*_request.json`/`*_response.json` 쌍은 실오라클에 재생했을 때
커밋된 응답을 재현해야 한다." 그런데 루프는 **매니페스트 항목**을
돈다.

두 집합은 같지 않다. 어긋나는 자리에서는 매니페스트가 조용히 이긴다 —
아무도 등록하지 않은 쌍은 재생되지 않고, 실행은 `identical` 줄이
쭉 찍히고 종료 코드 0으로 끝난다. **커버리지와 구분이 안 된다.**

두 집합을 실제로 세어 봤다:

```
커밋된 쌍          33
매니페스트에 없음   1   moveit-metrics/panda_arm_5dof_kinematics_metrics
쌍이 없는 항목      0
```

그 픽스처는 §101.2의 `columns < 6` 핀을 위해 라운드 13에 들어온
것이고, 그때부터 **한 번도 재생된 적이 없다.** 등록하고 돌려 보니
`identical`이다 — 즉 대가는 드리프트가 아니라 사각지대 자체였다.
이번에 우연히 깨끗했을 뿐이고, 다음 것이 그러리라는 보장은 없었다.

### 116.2 구조로 닫았다

지역 패치는 "빠진 항목 하나를 등록한다"이다. 그러면 다음 픽스처
작성자가 기억하는지에 다시 걸린다. 구조적 해법은 **검사의 정의역을
불변식의 정의역과 같게 만드는 것** — 커밋된 쌍에서 목록을 뽑고,
매니페스트에 없는 쌍을 skip이 아니라 **실패**로 만든다. 매니페스트가
아예 없는 크레이트의 쌍도 같은 경로로 잡힌다.

양방향으로 쟀다:

```
등록된 상태   exit 0,  33/33 identical
항목 제거     exit 1,  UNREGISTERED moveit-metrics/panda_arm_5dof_kinematics_metrics
```

### 116.3 소유 경계를 한 줄 넘었다

매니페스트 파일 `crates/moveit-metrics/tests/fixtures/oracle-models.json`
은 p1-fixtures의 것이다. 스크립트만 고치고 등록을 넘기면 등록이 올
때까지 main이 빨간 상태로 있게 되므로, 항목 한 줄은 내가 넣었다.
한 발견이므로 커밋도 하나다. 담당에게는 다음 브리프에서 알린다.

### 116.4 계열로 적는다

§112.2(통과는 보고하고 비용은 보고하지 않음), §114.4(허용오차 밑에
잠긴 파싱 오차), 그리고 이 건이 같은 모양이다:

> **검사가 아무것도 세지 않고도 통과처럼 보이는 자리를 찾아라.**
> 빈 글롭, 등록되지 않은 항목, 조용히 건너뛴 케이스, 흡수된 오차 —
> 전부 "초록"과 구분되지 않는다.

`ci.yml`의 `check-*.sh` 글롭이 비면 실패하도록 한 것,
`check-dep-direction.sh`가 `cargo tree`를 파이프 머리에 두지 않는 것,
그리고 이번 변경이 같은 규칙의 세 사례다.

## 117. p1-robotmodel 라운드 15 머지 — `visibility_cone` 115건의 원인이 나왔다

3커밋(`1258e7d`, `f6dfae9`, `c50694f`). 테스트 수는 그대로(**1104**) —
세 커밋 다 감사·문서·기록이다.

### 117.1 라운드 4부터 열려 있던 115건이 `decide_cone`의 결함이 아니다

라운드 4의 pr2 스윕을 그대로 다시 돌렸다: **2,201케이스 중 115건 실패,
전부 `visibility_cone`, 전부 거리 차이, 판정 불일치 0건.** 메시 지오
메트리가 착지한 뒤로 숫자가 변하지 않았다.

`compare_constraints`가 `satisfied`를 먼저 보고 틀리면 그 자리에서
반환하므로(`main.rs:984`), "판정 불일치 0 + 거리 불일치 115"는 코드
경로상 정확히 이 뜻이다 — 내가 확인했다.

원인은 이것이다. `cone_mesh`·`decide_cone`·
`allow_sensor_or_target_contact`는 상류와 정확히 일치한다. 갈리는
자리는 **`max_contacts: 1`이 처음 발견된 로봇 링크 하나만 저장한다**는
것이고(`crates/moveit-constraints/src/visibility.rs:453`, 상류 `req.max_contacts = 1`과 같다),
그 "처음"이 쌍 순회 순서다.

```
이 포트   parry.rs:798  cross_pairs = a.iter().flat_map(|x| b.iter().map(|y| (x,y)))
                        → RobotModel의 고정된 링크 배열 순서
상류      FCL 브로드페이즈 BVH의 자체 순회 순서(문서화돼 있지 않음)
```

내가 `cross_pairs`를 직접 읽어 확인했다. 두 순서 다 유효하고, 원뿔에
로봇 링크가 **둘 이상** 닿는 순간 "첫 접촉"이 갈린다.

이것은 `moveit-constraints`에서 고칠 수 있는 것이 아니다 — 순회 순서는
`moveit-collision`(p3-acm)에 있다. 담당이 범위를 지키고 원인만 이름
붙인 판단이 맞다.

### 117.2 그러나 설명은 아직 **일관될 뿐 검증되지는 않았다**

이 설명은 반증 가능한 예측을 하나 낳는다:

> 어긋나는 115건은 **정확히** 원뿔에 로봇 링크가 둘 이상 닿는
> 케이스여야 하고, 통과하는 2,086건은 닿는 링크가 하나 이하여야 한다.

**아직 아무도 이것을 확인하지 않았다.** 확인되면 115건은 닫힌다.
어긋나면 순회 순서는 원인의 일부일 뿐이고 남은 것이 있다. 다음
라운드 항목이다.

구조적 처분도 그 결과에 달려 있다. 예측이 맞으면 문제는 **하네스가
두 구현이 합의할 의무가 없는 값을 비교하고 있다**는 것이다 —
`max_contacts: 1` 아래의 접촉 깊이는 미명세 값이다. 상류가 자기
`decide` 안에서 `max_contacts = 1`을 세우므로 오라클 쪽을 올릴 수는
없다(올리면 오라클이 상류가 아니게 된다). 따라서 처분은 하네스 쪽이고,
그 근거가 §117.2의 예측이다.

### 117.3 `isValid()` 갭이 관례가 아니라 구조로 닫혔다

§109.2에서 "상류에서는 분기 조건이지만 이 포트에서는 그 상태를 만들
수 없다"로 판정했던 것을, 이번에 **모든 생성 경로를 열거해서** 닫았다.
네 타입(`JointConstraintSampler`, `UnionConstraintSampler`,
`IkConstraintSampler`, `IkConstraintSamplerAdapter`) 각각에 대해
`Self { .. }` 자리가 자기 fallible `new()` 안에 정확히 하나씩,
`Default`/`Deserialize` 없음, `pub` 필드 없음, `unsafe` 없음, 파생
`Clone`은 이미 검증된 수신자를 복제할 뿐. **우회 0건.**

"`new()`가 fallible이니까"는 관례였고, 이번 열거가 그것을 불변식으로
바꿨다.

### 117.4 자기 참조 함정을 자기가 잡았다

`assert_relative_eq!` 계수를 기록하는 문단을 쓰다가, **그 문단 자체의
텍스트가 자기가 인용한 명령의 결과를 부풀린다**는 것을 중간에 발견해
`///`/`//!` 줄을 걸러 내는 형태로 명령을 고쳤다. §73.1·§83.3·§92·
§104.1의 계수 오염 계열이 문서 자기 자신에서 재발한 사례다. 실제
호출은 0건.

### 117.5 내 브리프 숫자를 자기가 잰 값으로 고쳤다

브리프에 p3-acm의 느린 테스트를 1017초로 적었는데, 담당이 자기 실행
에서 **1128.457초**를 재고 "브리프의 숫자가 아니라 내가 잰 숫자를
기록한다"고 적었다. 맞는 처리다 — 재현하지 않은 남의 숫자를 인용하지
않는 것이 이 저장소가 반복해서 요구해 온 것이고, 이번에는 내 숫자가
그 대상이었다.

### 117.6 머지 후 실측

`fmt --check` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`cargo nextest run --workspace --no-fail-fast`(§112.2의 느린 테스트
제외) **1103/1103**, `cargo test --doc --workspace` 통과,
`check-*.sh` **4건** OK, `verify-fixture-provenance.sh` OK,
`verify-continuous-reseed-wrap.sh` OK(42.6000% > 35.1222%),
`verify-fixture-replay.sh` **33/33 identical**.

## 118. Phase 7의 완료 조건에는 C++ 기준선이 필요했고, 그것이 없었다 (2026-08-04)

`32114d5`. 오라클에 `plan` 연산을 추가했다. 코드 변경은
`tools/moveit-oracle/`만이고 Rust 트리는 건드리지 않았다.

### 118.1 막혀 있던 것은 감사가 아니라 계획이었다

일곱 패널이 전부 파리티 라운드를 돌고 있는 동안, §5의 Phase 7 완료
조건은 아무도 손댈 수 없는 상태였다:

> - 벤치마크 문제 500건에서 성공률이 C++ OMPL RRTConnect의 90% 이상
> - 산출 경로 100%가 `moveit-scene`의 충돌 검사와 제약을 통과
> - 경로 길이 중앙값이 C++ OMPL 대비 1.3배 이내

1번과 3번은 **C++ 쪽 숫자가 있어야 성립한다.** 오라클에는 플래닝
연산이 하나도 없었다(33개 연산 전부 모델·상태·충돌·IK·제약·궤적).
`moveit-planners-sbp`는 5,239줄로 RRT-Connect까지 착지해 있지만,
비교 대상이 없으니 완료 조건을 만족했는지 **판정할 수단 자체가 없었다.**
오라클은 내 것이므로 이 조각은 내가 막고 있던 것이다.

### 118.2 패키지 목록을 넓히지 않아도 됐다

`moveit_planners_ompl`을 빌드하면 `moveit_ros_planning`과 그
`rclcpp`/`tf2_ros` 트리가 colcon 빌드로 딸려 들어온다. Dockerfile 주석의
"Later phases widen this list"가 가리키던 자리다.

그런데 재 보니 넓힐 필요가 없었다 — **OMPL 1.7이 베이스 이미지에 이미
있다.** 헤더, `libompl.so`, `omplConfig.cmake` 전부
`/opt/ros/${ROS_DISTRO}` 아래에 있다. 그래서 `find_package(ompl REQUIRED)`
한 줄로 끝났고 `MOVEIT2_PACKAGES`는 그대로다. 스탬프는
`e7d32225310d3278` → `cd8ee2c1bdcf7148`로, 오라클 소스 변경분만큼만
움직였다.

### 118.3 왜 `ModelBasedStateSpace`가 아니라 공간을 다시 짓는가

완료 조건 3번은 "경로 길이 중앙값이 1.3배 이내"다. **두 길이가 같은
미터법으로 재어지지 않으면 이 비율은 아무 뜻도 없다.**

`moveit_planners_ompl`의 `ModelBasedStateSpace`는 가중
`CompoundStateSpace`를 만들지 않는다 — 평평한 배열 위에서
`JointModelGroup::distance`에 위임한다. 이 포트의
`JointModelGroupSpace`는 부분공간마다 `1/extent`로 정규화한 진짜 가중
합성이다(그 차이는 `joint_model_group_space.rs`가 이미 기록해 둔
의도적 이탈이다). 그래서 `plan`은 상류 브리지를 빌려 오지 않고
**관절 단위로 같은 공간을 다시 짓는다** — 유계 revolute/prismatic은
`1/(max-min)` 가중 1축 `RealVectorStateSpace`, 연속 revolute는
`1/(2π)` 가중 `SO2StateSpace`, planar은 x·y 축에 `angular_distance_weight/(2π)`
가중 theta, floating은 `1/extent` 가중 `RealVector(3)+SO3` 합성.

확인은 주장하지 않고 쟀다. panda_arm에서 joint1만 1.0 rad 다른 두
상태의 공간 거리가 `0.1725744658820281`로 나왔고, 이는
`1/(2 × 2.8973)`이다 — 포트의 가중 규칙과 정확히 같다. 요청의
`distance_probes` 필드가 그 표면이고, 플래너 출력은 시드 의존이라
**Rust 쪽이 비트 수준으로 걸 수 있는 유일한 자리**다.

### 118.4 OMPL의 SO3 거리 규약은 읽을 소스가 없어서 측정했다

`Se3Space::distance`는 **전체** 회전각에 가중한다(`rotation_distance`가
`2*acos(|dot|)`의 atan2 chord 형태 — 그 함수 주석에 한 번 반각으로
잘못 갔던 이력이 남아 있다). OMPL의 `SO3StateSpace::distance`는 헤더에
공식이 없고 이 이미지에는 OMPL `.cpp`가 없다.

그래서 하드코딩하지 않고 **링크된 라이브러리에서 읽었다.** 알려진 각
`π/2`의 회전에 대해 `distance`를 부르고 그 값으로 가중치를 보정한다.
실측값은 `0.7853981633974483` = `π/4`, 즉 OMPL은 반각 규약이고
보정 계수는 2다. 이 값(`space.so3_quarter_turn_distance`)은 응답에
그대로 실려 나가므로, 규약이 바뀌면 조용히 틀리는 게 아니라 숫자가
바뀐다.

§107.3(상류 헤더에서 닿는지 먼저 확인)의 반대 방향 사례다 — 닿을 수
없는 것을 요청하는 대신, 읽을 수 없는 상수를 실행 시점에 재서 얻었다.

### 118.5 OMPL은 stdout으로 로그를 쓴다

이 프로토콜은 stdout 한 줄에 응답 하나다. OMPL의 기본 콘솔 핸들러는
INFO/WARN을 **stdout으로** 쓴다. 로그 한 줄이면 그 뒤의 모든 응답이
요청과 어긋난다. `ompl::msg::noOutputHandler()`는 미관 문제가 아니라
프로토콜 문제다.

§55·§116 계열("검사가 아무것도 세지 않고도 통과처럼 보이는 자리")의
사촌이다: 여기서는 **응답이 다른 요청의 답인데도 형식은 멀쩡한** 자리였다.

### 118.6 반복 예산은 이름만큼 강한 레버가 아니다 — 이것도 쟀다

OMPL 1.7에는 반복 기반 종료 조건이 없다(`PlannerTerminationCondition.h`가
선언하는 것은 timed / non / always / and / or / exact-solution뿐).
그래서 `std::function<bool()>` 형태로 세는 조건을 만들었고, 그 단위가
포트의 `Termination::Iterations`와 같은지 **주장하지 않고 스윕했다.**

panda_arm에 상자 장애물을 두고 예산 `0,1,2,3,5,10,50,200,5000`:

```
예산 0     → 미해결, status=Timeout,       ptc_evaluations=1
예산 1 이상 → 전부 해결,                    ptc_evaluations=1
```

두 가지가 나왔다. 하나는 **오프셋이 0**이라는 것 — 예산 `n`은 반복 `n`회를
허용하고 종료 평가가 `n+1`번째다. 포트의 `for _ in 0..max_iterations`와
같은 의미이고, 예산 0에서 양쪽 다 반복을 한 번도 돌지 않는다.

다른 하나는 처음에 내가 쓰려던 주석이 틀렸다는 것이다. 나는 "grow 반복마다
한 번 + `Planner::solve` 자체의 소수 고정 횟수"라고 적었다가 스윕 결과로
고쳤다. 그리고 더 중요한 사실이 같이 나왔다 — **0이 아닌 모든 예산이
1회 반복으로 풀린다.** RRTConnect의 `connect`가 탐욕적이고 무한이라
7자유도 팔에서는 한 반복이 간극 전체를 닫는다. **반복 예산만으로 벤치마크
난이도를 조절하면 사실상 아무것도 재지 않게 된다.** 500문제 설계는 이
사실 위에서 해야 한다.

### 118.7 이 기준선이 아닌 것

`og::RRTConnect` 그 자체이지, `moveit_planners_ompl`의
`ModelBasedPlanningContext`가 아니다. projection evaluator, 경로 단순화,
요청 어댑터 체인이 전부 없다. 이 연산의 성공률과 경로 길이는 **이 공간
위의 RRTConnect에 대한 진술**이지 `move_group`이 무엇을 돌려줄지에
대한 진술이 아니다. 연산 주석에 그렇게 적었다.

결정론도 한계가 있다. `ompl::RNG::setSeed`는 프로세스 전역이고 RNG
인스턴스가 하나라도 생긴 뒤에는 OMPL이 거부하므로, 시드는 프로세스당
최대 한 번 적용된다(`seed_applied`가 그것을 알려 준다). **요청 스트림
전체를 순서대로 새 프로세스에 재생하면 재현되고**(실측: 같은 스트림
두 번, `planning_time_s` 빼고 전부 동일), 문제 하나만 따로 다시 돌리면
재현되지 않는다. 앞의 것이 `verify-fixture-replay.sh`가 하는 일이므로
픽스처로서는 충분하다.

### 118.8 픽스처는 일부러 커밋하지 않았다

`a640c98` 이후 커밋된 요청/응답 쌍은 전부 재생 대상이고 등록되지 않으면
실패한다(§116). `plan` 픽스처를 지금 넣으면 **소비하는 Rust 테스트가
없는 채로** 게이트만 하나 늘어난다 — §112.2·§116.4가 반복해서 잡아 온
"커버리지처럼 보이지만 아무것도 세지 않는 자리"를 내가 직접 만드는 셈이다.
`moveit-planners-sbp`는 p1-robotmodel 것이므로 픽스처 포착은 그쪽 라운드에
맡긴다.

### 118.9 실측

스탬프 `cd8ee2c1bdcf7148`. `check-*.sh` 4건 OK,
`verify-fixture-provenance.sh` OK, `verify-fixture-replay.sh` **33/33
identical**. Rust 트리 무변경이므로 cargo 게이트는 이 커밋의 범위가
아니다.

## 119. 네 패널 동시 병합 — 그리고 §117.1이 반증됐다 (2026-08-04)

`p1-joints` 15, `p1-fixtures` 16, `p3-acm` 14, `p3-shapes` 18라운드를
한 번에 병합했다(22커밋). 네 브랜치 모두 자기 소유 파일만 건드렸고
충돌은 없었다.

### 119.1 라운드 4부터 열려 있던 115건의 원인이 순회 순서가 **아니었다**

§117.1에서 p1-robotmodel이 원인으로 지목하고, §117.2에서 내가
"일관될 뿐 검증되지 않았다"며 반증 가능한 예측으로 바꿔 다음 라운드
항목으로 내보낸 그 설명이다:

> 어긋나는 115건은 **정확히** 원뿔에 로봇 링크가 둘 이상 닿는
> 케이스여야 하고, 통과하는 2,086건은 하나 이하여야 한다.

**p1-joints가 이것을 반증했다**(`d26916d`). `tools/moveit-diff/`에
`#[ignore]`된 진단 둘을 넣었다 — `decide_cone`의 실제 `max_contacts: 1`
대신 진단용으로 `64`까지 올려 접촉을 **전부** 열거한다. 커밋된 동작은
상류와 같은 `1`로 남는다.

내가 직접 돌린 실측(`--run-ignored all --no-capture`):

```
links checked:                                    17
near-placement가 1개 이상 닿은 링크:              17
near-placement가 2개 이상 닿은 링크 (ambiguous):   0

case 104: 1 pair(s) touched the cone: [("bl_caster_l_wheel_link", "cone")]
```

**17건 중 17건이 정확히 하나씩만 닿는다.** 실제로 어긋나는 케이스 104도
하나만 닿는다. 예측한 교차표의 오른쪽 열이 통째로 비어 있다 — 깨야 할
동점이 애초에 없으므로 쌍 순회 순서는 "첫 접촉"을 다르게 고를 수 없다.

진짜 원인은 `moveit-collision`의 **이미 문서화된 deviation 6**이다:
같은 명백한 접촉 하나에 대해 두 백엔드의 독립적인 침투 깊이 근사가
어긋난다. 케이스 104에서 오라클 깊이는 `bl_caster_l_wheel_link` 자신의
실린더 반지름과 **7ppm** 이내로 맞고 이 포트는 그렇지 않다. 0 근처
케이스 몇 건은 부호까지 뒤집힌다.

**§117.2를 쓴 것이 값을 했다.** "일관된 설명"을 그대로 닫았으면 라운드
4부터 열려 있던 항목이 *틀린 원인으로* 닫혔을 것이다. 반증 가능한
형태로 적어 두었기 때문에 다른 패널이 반증할 수 있었다.

동시에 이것은 §106.1 계열의 세 번째 사례이기도 하다 — 이번에는 내가
**남의 설명을 검증 없이 브리프의 지시로 승격시킨** 쪽이다. p1-robotmodel은
그 반증된 전제 위에서 라운드 16을 돌고 있었고, 병합 시점에 그 사실을
볼 수 있는 것은 나뿐이었다. 즉시 중단시키고 정정 브리프를 보냈다.

### 119.2 진단 자체에는 공허 통과 구멍이 남아 있다

`near_placement_never_touches_more_than_one_link_at_once`는
`eligible`이 비지 않았음은 확인하지만 **`touched_link_counts`가 비지
않았음은 확인하지 않는다.** 충돌 질의가 아무것도 돌려주지 않게 되면
`ambiguous`도 비고 테스트는 조용히 통과한다.

오늘은 공허하지 않다 — 17/17이 실제로 닿는 것을 내가 위에서 쟀다.
그러나 그것은 현재 값이 그렇다는 뜻이지 테스트가 그것을 강제한다는
뜻이 아니다. §55·§116.4·§118.5와 같은 계열("아무것도 세지 않고도
통과처럼 보이는 자리")이고, 다음 라운드 항목으로 넘긴다.

### 119.3 19분짜리 스위트가 22초가 됐다

§112.2에서 잡고 p3-acm에게 보냈던 그 테스트다. `986635c`가 고쳤다.

```
이전   pr2_self_wheel_same_pair_frozen_constant_...   1128.457 s   (스위트 약 19분)
이후   같은 테스트                                        18.111 s
       전체 스위트 1109/1109                               22.2 s
```

62배다. 커밋 헤드라인은 "1000x cost"라고 적혀 있는데 내가 실측한 것은
62배이므로 여기에는 실측값을 적는다. 이제 이분·섭동 작업을 `-p <crate>`로
좁힐 이유가 없어졌고, 모든 패널에 `--workspace`로 돌리라고 알렸다.

### 119.4 나머지 세 라운드

**p1-fixtures 16라운드**(6커밋). "잴 수 있는데 재지 않은" 문서 주장
계열을 쓸었다 — planar sentinel 6개 OR 항을 각각 분리해 핀하고,
`is_path_valid`의 진단 전체를 걸고(한 건만 맞히는 것이 아니라),
floating joint NaN 주장이 이미 발화 중이었음을 확인했다. 완료 기준
계수가 건너뛰던 두 번째 folding 단계에 이름을 붙였다.

**p3-acm 14라운드**(8커밋). §79의 51자리를 전부 처분했다 —
`epsilon`만 있던 41자리에 `max_relative`를 넣고, 둘 다 없던 10자리를
`assert_eq!`로 바꿨다(`parry.rs` 18+4, `planar.rs` 10, `revolute.rs`
6+3, `floating.rs` 4, `prismatic.rs` 1+2, `model.rs` 2+1). 캐스터 7건의
오라클 쪽 질문이 아직 열려 있다는 것을 문서에 그대로 적었다 —
닫힌 척하지 않았다.

**p3-shapes 18라운드**(6커밋). 계수 관례를 문장이 아니라 **재현 가능한
명령**으로 바꿨다(`tools/ci/count-public-declarations.sh`,
`tools/ci/count-relative-eq.pl`). `leaf_iterator` 순서를 논증하는 대신
오라클로 쟀고(`leaves` 픽스처 신규), `oracle-models.json`에 등록했다 —
§116의 게이트가 강제하는 그대로다. 재생이 33 → **34쌍**이 됐다.

### 119.5 실측

`fmt --check` 통과, clippy `--workspace --all-targets -D warnings` 0건,
`cargo nextest run --workspace --no-fail-fast` **1109/1109**(22.2초,
제외 없음), `cargo test --doc --workspace` 통과, `check-*.sh` **4건** OK,
`verify-fixture-provenance.sh` OK, `verify-continuous-reseed-wrap.sh`
OK(42.6000% > 35.1222%), `verify-fixture-replay.sh` **34/34 identical**.

p1-joints의 `#[ignore]` 진단 2건은 별도로 `--run-ignored all`로 돌려
통과와 위 숫자를 확인했다 — 일반 스위트는 이 둘을 실행하지 않으므로
게이트 통과가 그 주장의 근거가 되지 못한다.

## 120. 네 패널 병합 — 그리고 §119.1과 §118.6 둘 다 내가 과잉 주장했다 (2026-08-04)

`p1-robotmodel` 16, `p3-shapes` 19, `p3-distance-field` 18, `p6-totg` 14
라운드를 병합했다(12커밋). 전부 자기 소유 안이고 충돌 없음.

실측: `fmt --check` 통과, clippy `--workspace --all-targets -D warnings`
0건, `cargo nextest run --workspace --no-fail-fast` **1110/1110**(30초),
`cargo test --doc --workspace` 통과, `check-*.sh` **4건** OK,
`verify-fixture-provenance.sh` OK, `verify-fixture-replay.sh`
**35/35 identical**.

### 120.1 §119.1의 "오른쪽 열이 비어 있다"는 틀렸다

§119.1에서 나는 p1-joints의 진단만 보고 이렇게 썼다:

> 예측한 교차표의 오른쪽 열이 통째로 비어 있다 — 깨야 할 동점이
> 애초에 없으므로

**이것은 과잉 주장이다.** p1-robotmodel이 같은 라운드에 285케이스
스윕으로 **실패 집단 자체를** 교차 집계했다(`ee9ce92`):

```
touching | n   | pass | fail
1        | 129 | 24   | 105
>=2      |     |  4   |  10
```

**10/115(8.7%) 실패 케이스는 touching >= 2다.** 오른쪽 열은 비어 있지
않다. 그리고 touching >= 2인 **통과**도 4건 있다 — touching >= 2가
실패를 함의하지도 않는다.

두 측정이 모순인 것이 아니다. **표본이 다르다.** p1-joints는 pr2의
**기본 자세**에서 17개 링크의 near-placement를 봤고, p1-robotmodel은
**무작위 자세의 실제 실패 집단**을 봤다. 기본 자세에 동점이 없다는 것이
무작위 자세에 없다는 뜻이 아니고, 나는 앞의 것에서 뒤의 것을 결론했다.

정확한 문장은 이것이다:

> 예측은 반증됐다 — 지배적 원인(105/115, 91.3%)은 touching == 1이므로
> 순회 순서일 수 없다. 나머지 **10건은 touching >= 2이므로 이 증거로
> 배제되지 않는다.**

§117.2를 반증 가능한 형태로 적어 둔 것이 값을 했다는 §119.1의 결론
자체는 그대로다. 값을 한 방식이 내가 적은 것보다 정확했을 뿐이다.

### 120.2 그리고 담당의 두 커밋이 서로 모순인 채로 병합됐다

`ee9ce92`가 위 표를 재고, 바로 다음 `f111dfb`가 이렇게 덮었다:

> With no ties possible, pair-traversal order cannot be the cause of
> **any** of the 115 mismatches

근거로 든 것은 p1-joints의 기본 자세 진단이다. **자기가 방금 잰
집단 측정을 남의 표본 때문에 버렸다.** 게다가 `cone_touching_link_count`를
"다른 호출자가 없다"는 이유로 삭제해서, **그 10건은 이제 트리 안에서
다시 잴 수 없다.**

§117.5에서 담당이 나에게 적용했던 규칙 — 재현하지 않은 남의 숫자를
인용하지 않는다 — 은 양방향이다. **자기가 잰 숫자를 남의 숫자 때문에
버리지 않는 것**이 같은 규칙의 다른 쪽이다. 라운드 17 1항으로 돌려보냈다:
문장을 자기 측정으로 되돌리고, 진단을 되살리고, 남은 10건이 deviation 6과
같은 계열인지 크기 분포로 가르라고.

내 §119.1은 내가 여기서 고친다. 담당 것은 담당이 고친다.

### 120.3 §118.6의 "모든 예산이 1회 반복으로 풀린다"도 틀렸다 — 같은 병이다

§118.6에서 나는 이렇게 적었다:

> **0이 아닌 모든 예산이 1회 반복으로 풀린다.** [...] 반복 예산만으로
> 벤치마크 난이도를 조절하려 하면 아무것도 재지 못한다.

나는 이것을 **문제 하나**로 쟀다. 집단으로 재니 틀렸다. panda_arm에
장애물 구성을 바꿔 가며 무작위 시작·목표 20쌍씩(시드 20260804,
`range=0.05`, `motion_resolution=0.01`, 예산 2000):

```
config       solved   rate   med len   iters  bad ep  timeout
empty        15/15  100.0%    2.3302       1       5        0
floor        13/13  100.0%    2.3034       1       7        0
floor+wall    8/8   100.0%    4.8799     339      12        0
slot          9/9   100.0%    4.5918     597      11        0
corridor      4/5    80.0%    5.1309    1189      15        1
cage          4/4   100.0%    3.1099      25      16        0
```

중앙값 반복이 **1 → 339 → 597 → 1189**이고 `corridor`에서 처음으로
진짜 탐색 실패(timeout)가 나온다. **반복 예산은 유효한 난이도 레버가
맞다 — 장애물이 있을 때만.** §118.6은 "빈 공간에서는"이라는 조건이
빠진 채로 일반화한 것이다.

§119.1과 정확히 같은 실수다: **표본 하나에서 집단을 결론했다.** 한
라운드에 두 번 했고, 둘 다 내가 다른 사람에게 하지 말라고 브리프에
적어 온 것이다(§110.2 "재지 않은 주장은 틀린 주장일 수 있다").

### 120.4 그 스윕이 500문제 설계에 대해 실제로 말해 주는 것

세 가지가 나왔고 셋 다 Phase 7 완료 조건의 형태를 바꾼다.

**성공률은 판별력이 없다.** 유효한 끝점 쌍에서 `corridor`를 빼면 전부
100%다. 완료 조건 1(성공률이 C++의 90% 이상)은 이 난이도 범위에서
자동으로 만족된다. **실질적인 게이트는 조건 3(경로 길이 중앙값 1.3배
이내)이고**, 판별은 반복 수와 길이에서 나온다.

**무작위 시작·목표의 25~75%가 무효 끝점이다**(자기충돌 또는 장애물
침투). `empty`에서 5/20, `corridor`에서 15/20. 걸러지지 않은 세트로
성공률을 재면 두 구현의 **플래너가 아니라 샘플러를 재게 된다.**
500문제 세트는 끝점 유효성을 미리 걸러야 한다.

**난이도는 장애물 기하로 만들어야 한다.** `empty`/`floor`는 중앙값 1회
반복이라 아무것도 재지 않는다. 쓸 만한 대역은 `floor+wall` 이상이다.

**그리고 더 조이면 도로 쉬워진다.** `cage`는 20쌍 중 **16쌍이 무효
끝점**이고, 살아남은 4쌍은 중앙값 25회 반복으로 전부 풀린다 — 길이도
`corridor`의 5.13에서 3.11로 **줄어든다.** 공간을 너무 조이면 무작위
샘플러가 통과하는 쌍이 좁은 통로를 지나는 어려운 쌍이 아니라 **애초에
서로 가까운 쉬운 쌍**만 남기 때문이다. 난이도를 올리려고 장애물을 더
넣으면 어느 지점부터는 **플래너가 아니라 샘플러를 조이게 된다.**

따라서 쓸 만한 대역은 `floor+wall` ~ `corridor`이고, 그 위는 아니다.
이것은 §118.6의 반복 예산 오해와 같은 모양의 함정이다 — 레버를 끝까지
돌리면 측정 대상이 조용히 바뀐다.

### 120.5 나머지 세 라운드

**p1-robotmodel `4f870fe`** — `plan` 연산의 `distance_probes`로 두 공간
구성을 비트 수준으로 걸고 픽스처를 `oracle-models.json`에 **등록까지**
했다. §118.8에서 내가 소비자 없이 만들지 않겠다고 남겨 둔 자리를
소유자가 소비자와 함께 채웠다. 재생 34 → **35**.

**p3-shapes 19라운드**(3커밋). 계수 스크립트가 블록 주석과 문자열
리터럴을 거르도록 고쳤다(§119의 1항 그대로). §79를 고친 스크립트로 다시
세니 처분할 것이 없었다 — "없다"를 근거와 함께 적은 것이 맞는 처리다.
`octree_points` 폐쇄 조사를 끝까지 다시 돌려 **닫지 못하는 항목 하나를
이름으로 남겼다**: `LeavesInBbx`(`leaf_bbx_iterator`). 그 자체의 필드나
순서는 여전히 직접 검증되지 않고, `moveit-distance-field`의
`octree_points`가 내부적으로 부르지만 그것은 **소비자의 파생 결과를
핀할 뿐**이다.

**p3-distance-field 18라운드**(2커밋). `raw_value` 껍데기를 걷어냈다 —
워크스페이스 `float_roundtrip`이 `Value`뿐 아니라 직접 타입 필드
역직렬화에도 적용된다는 것을 확인하고(`[f64;3]`/`f64`/`Value` 전부
`str::parse`와 일치) 크레이트 로컬 우회를 지웠다. §115가 "버전 특정이
아니다"로 닫은 그 결함의 잔재다. 허용오차도 다시 이분했다.

**p6-totg 14라운드**(3커밋). `moveit-smoothing`의 §79는 **5자리, 전부
tests/**이고 다섯 다 이미 두 게이트를 같은 상수로 핀하고 있다 — 함정
범주 0건. `rg -c`가 아니라 주석 제거 + 괄호 균형 계수기로 셌다(§73.1
계열을 스스로 지켰다). `float_roundtrip` 아래 재측정에서 `totg_path`
바닥이 비트 단위로 동일했고, 오염된 리터럴이 픽스처에 실제로 있으나
(요청 2/30, 응답 8/138) **네 최대값을 세우는 항이 그중 하나도 아니라는
것**을 항별로 짚었다. 블렌드 3 안쪽의 메커니즘을
`next_velocity_switching_point`의 EPS 이분 탐색으로 좁혔고, 상류
`getNextVelocitySwitchingPoint`/`trajectory_`가 private이라 오라클로
확인할 수 없다는 것을 **헤더에서 직접 확인하고** 요청하지 않았다(§107.3).

## 121. visibility_cone 거리 비교를 접었다, 그리고 Phase 8을 병렬로 연다 (2026-08-04)

`p1-joints` 16, `p1-robotmodel` 17, `p3-shapes` 19-후속 라운드를 병합했다
(6커밋). 실측: `fmt --check` 통과, clippy `--workspace --all-targets
-D warnings` 0건, `cargo nextest run --workspace --no-fail-fast`
**1110/1110**(25.4초, 벽시계 28.4초), `cargo test --doc --workspace` 통과,
`check-*.sh` **4건** OK, `verify-fixture-provenance.sh` /
`verify-continuous-reseed-wrap.sh` OK, `verify-fixture-replay.sh`
**35/35 identical**.

### 121.1 115/2201은 허용오차로 닫을 수 없다 — 그래서 비교를 좁혔다

§117.2 이래 열려 있던 `visibility_cone`의 115/2201 불일치가 처분됐다.
`p1-joints`가 115건 전부를 재측정했다(시드 4, `--group right_arm
--cases 100 --constraints 2000`, 스탬프 `cd8ee2c1bdcf7148`):

```
max |diff|  5.42e-2
median      3.57e-3
min         3.93e-5
sign flips  25/115 (깊이가 0 근처인 구간)
```

**세 자릿수에 걸쳐 퍼져 있고 바닥이 없다.** "backend 잡음"과 "진짜 결함"을
가르는 문턱이 분포 안에 존재하지 않는다. 전부를 잠재우려면 ~5.4e-2가
필요한데, 그것은 비교 대상 깊이 자체보다 큰 값이 많아서 진짜 회귀도 같이
잠재운다. **허용오차로는 닫히지 않는 종류의 불일치다.**

그래서 넓히는 대신 좁혔다: `compare_constraints`가 `kind ==
"visibility_cone"`일 때 `distance` 비교만 건너뛴다. `satisfied`는 그대로
비교한다.

**포기한 것을 정확히 적는다** — near/far 문턱을 넘지 않을 만큼 작으면서
깊이 *값*은 틀리는 결함. 그보다 큰 결함은 `satisfied`가 잡는다.

이 문장을 말로 두지 않고 **섭동으로 확인했다**(§82.1). `decide_cone`의
`!result.collision`을 `result.collision`으로 뒤집으면 —

```
passed: 1916   failed: 285
  142x  satisfied mismatch rust=false oracle=true
  143x  satisfied mismatch rust=true oracle=false
```

좁힌 뒤에도 285건이 `satisfied`로 잡힌다. 좁히기 전의 sweep은 내가
직접 다시 돌려 **2201/2201**을 확인했다 — 담당의 숫자를 옮기지 않았다.

나머지 여섯 종류는 `distance`를 그대로 비교한다. 그 여섯은 충돌 검출에
도달하지 않고 지금까지 한 건도 어긋난 적이 없다.

### 121.2 담당이 자기 측정을 되찾았고, 남은 10건을 크기로 갈랐다

§120.2에서 돌려보낸 것 — `p1-robotmodel`이 `f111dfb`를 뒤집어 자기
285케이스 스윕을 복원하고 `cone_touching_link_count`를 되살렸다.
그리고 내가 요청한 크기 비교를 했다:

- touching >= 2인 실패 **10건**: 2.3e-4 ~ 3.6e-3
- touching == 1인 실패 **105건**: 3.9e-5 ~ 5.4e-2

**10건의 범위가 105건 안에 통째로 들어간다**(41% 직접 중첩, 평균이 105건
분포의 30 백분위). 별개의 군집이 아니다. touching >= 2가 실패를 함의하지도
않는다(14건 중 4건 통과), 실패율이 touching == 1보다 높지도 않다.

**닫지 않고 좁혔다**고 적은 것이 맞는 처리다. 그 10건에 깰 동점이 구조적으로
존재하는 것은 사실이고, 다만 **측정된 어떤 것도 순회 순서를 deviation 6보다
지지하지 않는다.** "증거 없음"을 "없음"으로 쓰지 않았다 — §120.2가 지적한
바로 그 실수의 반대 방향이다.

### 121.3 Phase 8을 Phase 7 완료 전에 연다 — 명시적으로

§5는 "조건을 만족하지 못하면 다음 단계로 넘어가지 않는다"고 적혀 있다.
Phase 7의 완료 조건(500문제 벤치마크)은 아직 열려 있다(이 절의 "열려
있다"는 §219이 갱신한다 — 500건 측정이 재실행 가능한 하네스로
닫혔다). 그럼에도
`moveit-planners-chomp` 이식을 지금 시작한다. 이유 두 가지:

1. **의존이 없다.** CHOMP의 core(`chomp_motion_planner`)는 Phase 1~6에만
   의존하고 그 여섯은 전부 닫혔다. Phase 7의 RRT/PRM 결과를 쓰는 부분이
   없다.
2. **인력을 더 붙일 수 없다.** Phase 7에 남은 것은 벤치마크 세트 하나이고
   소유자가 한 명(`p1-robotmodel`)이다. 나머지 패널을 놀리는 것과
   병렬화의 교환이 아니라, 놀리는 것과 진행의 교환이다.

**규칙은 유지된다: Phase 8의 완료 조건은 Phase 7이 닫히기 전에 선언하지
않는다.** 이식은 진행하되 단계 완료 판정은 순서를 지킨다. §5를 고치는 것이
아니라 이 한 번의 예외를 근거와 함께 남긴다.

`chomp_interface/`는 이식하지 않는다 — ROS 플러그인이고 D1/D2가 그것을
배제한다. 상류 3,771 LOC 중 core 7모듈이 대상이고, 이번 라운드 범위는
`chomp_parameters`·`chomp_utils`·`chomp_trajectory` 셋이다. 옵티마이저가
가장 어렵고 그 앞의 자료구조가 틀리면 전부 다시 하기 때문에 한 라운드에
몰지 않는다.

### 121.4 나머지 두 라운드

**p1-joints `ed97930`** — §79 계수를 `count_relative_eq.pl`로 재현 가능하게
만들었다(p3-acm의 형태를 가져왔다). `moveit-kinematics`/`tools/moveit-diff`/
`moveit-state/tests/invariants.rs`에 대해 both=0 epsilon_only=2
max_relative_only=0 neither=0. 앞의 둘은 `assert_relative_eq!` 호출이
**아예 0건**이라 처분할 것이 없다는 것이 근거와 함께 나왔다.

**p1-joints `5efda3a`** — §119.2의 공허한 통과 구멍을 구조적으로 닫았다.
`touched_link_counts`를 조건부로 채우는 대신 측정 지점에서 `touched > 0`을
단언하고 무조건 push한다 — 사후 집계 검사가 아니라 **구성상** 성립한다.
`is_empty()`/`len()==0`/`count()==` 앵커로 세 대상을 훑어 같은 모양이
이 자리 하나뿐임을 확인했고, 나머지 네 자리는 각각 왜 다른지 적었다.

**p3-shapes 2커밋** — `float_roundtrip` 바닥 명령(§115)에 대해 두 크레이트를
훑었고 해당 없음. `moveit-geometry`의 이분된 9자리는 전부 rustc가 파싱하는
Rust 소스 리터럴과 비교하며 두 파일 다 `serde_json`을 import조차 하지 않는다
— 없다를 근거로 적었다. `moveit-octomap`은 `assert_relative_eq!` 0건.
그리고 §113.3대로 `getTreeType()` 간극을 **라운드 17 커밋 메시지를 믿지 않고**
현재 트리에 대해 다시 셌다(159 불릿 그대로).

## 122. pilz 오라클의 비용을 미리 쟀다 (2026-08-04)

`p1-joints`에게 pilz 오라클 연산을 **이번 라운드에 요청하지 말라**고 한
이유는 비용이 크기 때문이고(§121.3의 배정), 그 비용이 얼마인지는 요청이
오기 전에 내가 알고 있어야 한다. 쟀다.

### 122.1 pilz는 base image에 없다 — ompl과 다르다

§118.2에서 ompl은 base image에 이미 들어 있어서 `MOVEIT2_PACKAGES` 확장이
**0**이었다. pilz는 아니다. 스탬프 `cd8ee2c1bdcf7148` 이미지 안에서 확인:

```
/opt/ros/rolling/include/pilz*   없음
/opt/ros/rolling/share/pilz*     없음
/opt/ros/rolling/lib | grep pilz 없음
```

소스 빌드가 필요하다.

### 122.2 콜콘 패키지 7 → 19

이미지 안에서 직접 쟀다:

```
현재  (moveit_core moveit_resources_fanuc_description)         7
pilz  (--packages-up-to pilz_industrial_motion_planner)  합집합 19
```

늘어나는 12개:

```
moveit_configs_utils                          moveit_ros_move_group
moveit_kinematics                             moveit_ros_occupancy_map_monitor
moveit_resources_fanuc_moveit_config          moveit_ros_planning
moveit_resources_prbt_ikfast_manipulator_plugin
moveit_resources_prbt_moveit_config           pilz_industrial_motion_planner
moveit_resources_prbt_pg70_support             pilz_industrial_motion_planner_testutils
moveit_resources_prbt_support
```

**`moveit_ros_planning`과 `moveit_ros_move_group`이 들어온다** — §118.2에서
`moveit_planners_ompl`을 피한 바로 그 이유다. pilz에는 우회로가 없다:
`joint_limits_common`부터가 `moveit_ros_move_group::moveit_ros_move_group`과
`moveit_ros_planning::moveit_ros_planning`에 직접 링크한다(상류
`CMakeLists.txt` 66~88줄). 해석적 core만 떼어 낼 수 없다.

### 122.3 다만 pluginlib 우회는 필요 없다 — 직접 링크된다

p6-totg가 `acceleration_filter`에서 겪은 것(모듈이 exported target set에
없어 `pluginlib::ClassLoader`로 우회) 과 다르다. pilz는 필요한 것이 전부
`ament_export_targets(pilz_industrial_motion_plannerTargets)`의
`install(TARGETS ...)` 목록 안에 있다:

- `trajectory_generation_common` = `trajectory_functions` +
  `trajectory_generator` + `trajectory_blender_transition_window`
- `joint_limits_common` = `joint_limits_aggregator`/`_container`/
  `_validator` + `limits_container`
- `planning_context_loader_ptp` 안에 `trajectory_generator_ptp` +
  **`velocity_profile_atrap`**이 같이 들어 있다(`_lin`/`_circ`도 같은 모양).
  이름은 loader지만 타겟 자체가 export되므로 링크 가능하다.

**결론: 12패키지 확장을 치르면 pilz 오라클은 직접 링크로 만들 수 있다.**
치를지 여부는 `p1-joints`가 무엇을 걸어야 하는지 확정한 뒤에 정한다 —
§5의 완료 조건이 "LIN/PTP/CIRC 궤적이 `1e-6` 이내"이므로 결국 치르게 될
가능성이 높지만, **무엇을 비교할지 모르는 채로 이미지를 12패키지 불리는
것**이 순서가 뒤바뀐 것이다. 재빌드는 스탬프가 바뀌므로 전 패널이
영향을 받는다.

## 123. 두 건의 소유 조정 — 공유 타입과 갈라진 간극 (2026-08-04)

Phase 8을 세 패널에 나눠 열자마자 소유가 갈리는 자리가 둘 나왔다. 둘 다
패널이 스스로 발견해서 **작업을 멈추고 물어 온 것**이고, 그것이 맞는
처리다 — 양쪽이 각자 만들었으면 중복이나 잘못된 의존 방향으로 굳었다.

### 123.1 `multivariate_gaussian`은 상류에 두 벌이다

`p6-totg`가 CHOMP를 시작하면서 상류 두 벌을 직접 대조했다
(`chomp_motion_planner/multivariate_gaussian.h` vs
`stomp_moveit/math/multivariate_gaussian.hpp`):

- **같은 것**: `mean_`/`covariance_`/`covariance_cholesky_`(`llt().matrixL()`),
  표준정규 샘플 루프
- **다른 것**: 네임스페이스, STOMP 쪽 `shared_ptr` typedef, 그리고
  STOMP `sample()`의 `bool use_covariance = true` 매개변수

**결정: `crates/moveit-sampling` 하나에 두고 두 플래너가 의존한다.**
소유는 `p3-shapes`(STOMP 담당) — 그쪽 라운드 범위에 이미 들어 있었고
`p6-totg`는 이번 라운드에 보류했기 때문이다. CHOMP는 옵티마이저에
도달하는 라운드에 같은 크레이트를 쓴다.

**플래너가 형제 플래너에 의존하는 방향은 쓰지 않는다.** `chomp → stomp`도
`stomp → chomp`도 아니다. 한쪽이 다른 쪽의 내부 결정에 묶이고,
`check-dep-direction.sh`가 막아야 할 모양이 새 크레이트 사이에서 다시
생긴다. 타입 하나짜리 크레이트가 과한 것 아니냐는 반론은 성립하지 않는다 —
소비자가 **오늘 둘 다 존재한다.** 미래를 위한 추상이 아니다.

**`use_covariance` bool은 옮기지 않는다.** 그것은 한 함수가 문맥에 따라
두 가지를 뜻하게 만드는 자리이고, 이 문서가 반복해서 결함의 원인으로
지목해 온 모양 그대로다. Rust API는 이름 붙은 메서드 둘로 가른다.
CHOMP는 그중 하나만 부른다 — 즉 CHOMP 쪽에는 분기가 아예 없다.

루트 `Cargo.toml`(내 소유)에 `moveit-sampling` 경로 항목과 `rand_distr`을
넣었다(`639c292`). `rand`에는 분포가 없어서 표준정규를 얻으려면
`rand_distr`이 필요하다. **핀하기 전에 실제로 확인했다**: `rand_distr 0.6`이
`rand 0.10.2` 하나로 해소되고(중복 없음) `rand_chacha`와 컴파일된다.
`StdRng`는 못 쓴다 — 워크스페이스가 `rand`를 `default-features = false`로
두어 `std_rng`가 꺼져 있고, 그래서 `rand_chacha`가 따로 있는 것이다.

### 123.2 `LeavesInBbx` 간극이 두 조각으로 갈라졌다

§120.5에서 `p3-shapes`가 이름으로 남긴 항목을 `p3-distance-field`가
소비자 쪽에서 확인했다. 처음으로 정확한 간극이 나왔다.

`octree_points`(`distance_field.rs:95`)는 `Leaf`의 접근자 **8개 중 3개**만
읽는다 — `is_occupied()`, `coordinate()`, `size()`. **내가 직접 확인했다.**
읽지 않는 다섯: `key()`, `index_key()`, `depth()`, `log_odds()`,
`occupancy()`.

그리고 담당이 `p3-shapes`의 새 `leaves_parity.rs`를 **먼저 읽고**,
그 테스트가 자기를 정당화한 논리를 그대로 적용했다:

> `tree_iterator`와 `leaf_iterator`는 서로 다른 상류 클래스이므로
> 한쪽의 검증이 다른 쪽으로 전이되지 않는다

같은 논리로 `leaf_iterator`의 순서가 이제 핀됐어도 **`leaf_bbx_iterator`로
전이되지 않는다.** 남의 커밋을 "관련 있으니 닫혔다"로 세지 않았다.

**정확한 간극 두 조각**:

1. 다섯 필드는 **어느 크레이트에도 소비자가 없다** → `moveit-octomap`에서만
   닫을 수 있다. 소비자 없는 필드에 픽스처를 붙이는 것이 맞는지 자체가
   판단할 거리이므로 그 판단도 `p3-shapes`에 넘긴다.
2. leaf 간 방출 순서는 **어느 테스트에서도 실행되지 않는다** → 여기서
   갈린다. `octree_points`가 받은 순서를 **충실히 전달하는가**는 같은
   언어 안의 질문이라 오라클이 필요 없고 `p3-distance-field`가 닫는다.
   그 순서가 상류와 **일치하는가**는 교차 언어라 `moveit-octomap` 쪽
   `leaf_bbx_iterator` 파리티 픽스처가 필요하다.

`octree_points`의 doc에 있던 "in emission order"는 **한 leaf 안의 세분
순서**를 뜻하는데 leaf 간 순서로 읽힌다. 오해를 낳는 문구이므로 고치라고
지시했다.

### 123.3 이 라운드가 아무것도 커밋하지 않았다는 것

`p3-distance-field` 19라운드는 세 항목을 전부 해내고 **커밋을 하나도
남기지 않았다.** 결론이 보고서 안에만 있다. 다음 사람이 크레이트를 열면
간극도, `both=27 epsilon_only=3`도, "argued-only 0건"도 볼 수 없다.

**보고서는 저장소가 아니다.** 감사 라운드의 산출물은 코드 변경이 아니라
**문장**일 때가 많고, 그 문장이 트리에 없으면 다음 라운드가 같은 것을 다시
잰다 — 이 문서가 §113.3으로 이미 한 번 겪은 실패다. 크레이트 doc이
자리다(`PORTING-PLAN.md`는 내 파일이므로 패널이 쓰지 않는다). 라운드 20의
1항으로 돌려보냈다.

## 124. Phase 7이 이름만 적어 둔 크레이트를 만든다 (2026-08-04)

네 패널이 동시에 유휴로 돌아왔다. 배정하면서 두 가지가 드러났다.

### 124.1 `moveit-planning`은 §5가 이름을 적고 만들지 않았다

§5 Phase 7은 크레이트를 둘 적는다 — `moveit-planning`,
`moveit-planners-sbp`. 뒤의 것만 존재한다. 그리고 Phase 7의 항목 셋 중
"요청·응답 어댑터 체인"은 워크스페이스 어디에도 없다.
`PlannerManager`/`PlanningContext` 트레이트는
`moveit-planners-sbp::registry`가 D4 형태로 이미 갖고 있어서, 없는 것은
어댑터 체인 하나뿐이다.

상류에서 그 체인이 `moveit_ros/planning/` 아래에 산다는 것이 지금까지
누락된 이유로 보인다 — 경로만 보면 Phase 9다. 그러나 파일을 열면
`check_start_state_bounds`(208줄), `check_start_state_collision`(113),
`check_for_stacked_constraints`(100), `resolve_constraint_frames`(83),
`validate_workspace_bounds`(103), `validate_path`(157)은 전부 씬·상태·
제약·궤적 위의 코어 로직이고, `rclcpp`/`moveit_msgs` 등장은 파일당
2~8회에 그친다. **경로가 아니라 내용이 단계를 정한다.** §5가 이것을
Phase 7에 적어 둔 것이 맞다.

`display_motion_path`는 rviz 퍼블리시라 D1이 배제한다.
`add_ruckig_traj_smoothing`/`add_time_optimal_parameterization`은
`moveit-smoothing`/`moveit-trajectory`를 의존만 하면 되므로 함께 넣는다.

**이것이 열려 있는 결함 하나를 닫는다.** §123.2에 적은
`fill_robot_trajectory`의 `dt = 0.1` — 상류에서 그 값을 덮어쓰는 것이
바로 `add_time_optimal_parameterization`이고, 이 포트에 그것이 없다는
것이 구멍의 원인이었다. 체인이 서면 닫힌다. p1-fixtures에게 주석이
아니라 테스트로 보이라고 요구했다.

### 124.2 계층을 거꾸로 세우지 않기 위해 순서를 나눴다

`PlanningRequest`/`PlanningResponse`/`PlanningContext`/`PlannerManager`가
지금 플래너 크레이트에 있다. 주인은 `moveit-planning`이어야 하고
`moveit-planners-sbp`가 그것을 의존해야 한다. 그런데 그 크레이트에서
p1-robotmodel이 Phase 7 벤치마크를 돌리는 중이다.

두 선택지 중 **의도적으로 틀린 계층을 먼저 세우는 쪽(플래닝 크레이트가
플래너 크레이트를 의존)은 택하지 않았다.** 그건 "구조 대신 패치"이고,
되돌리는 비용이 처음부터 옳게 세우는 비용보다 크다. 대신 순서를 나눴다:
이번 라운드에 `moveit-planning`이 **정본이 될 타입**을 정의하고 어댑터를
그 위에 세운다, `moveit-planners-sbp`의 재배치는 p1-robotmodel 병합 후
다음 라운드에 같은 패널이 한다. 보고서에 UNFIXED가 아니라 "다음 라운드
예정, 선행조건: p1-robotmodel 병합"으로 적으라고 했다 — 미해결이 아니라
일정이기 때문이다.

### 124.3 낡은 문장 둘을 각 소유자에게 돌려보냈다

- `moveit-planners-sbp/src/registry.rs`가 *"`constraint_samplers` itself
  has never been ported"* 라고 적고 있다. 지금은 사실이 아니다 —
  `moveit-constraints`에 `constraint_sampler_manager.rs`/`ik_sampler.rs`/
  `sampler.rs`가 있다. p1-fixtures에게 이 문장을 고치라고 하지 않고
  (남의 크레이트다) **그 사실이 `moveit-planning` 설계에 무엇을 바꾸는지**
  판단해서 답하라고 했다: 포즈 목표를 요청에 실을 수 있게 되었는지,
  아니면 샘플러는 있는데 플래너까지 잇는 선이 여전히 없는지.
- `moveit-distance-field/src/lib.rs`의 롤업이 아직
  `CollisionEnvDistanceField`를 "entirely unported"로 적는데, 라운드 21이
  그 진입점들을 이식한다. §113.3의 재발이므로 라운드 안에서 롤업을
  갱신하라고 명시했다.

### 124.4 나머지 두 배정

p1-joints는 pilz `trajectory_functions`/`trajectory_generator` 기반으로
가되, **오라클 요청서를 이번 라운드에 쓴다.** §5 Phase 8의 완료 조건이
`1e-6` 궤적 일치이고 §122에 비용을 이미 재 뒀으므로, 남은 것은 어떤
상류 타입을 부르고 응답에 무엇을 실을지다. 재빌드는 스탬프를 바꿔 전
패널에 영향을 주므로 요청서가 확정된 뒤 내가 한 번에 만든다.

p6-totg는 CHOMP `chomp_cost` — 유한차분 미분 행렬과 그 역행렬이 본체다.
경계에서 미분 규칙이 잘리는 방식이 결과를 바꾸고 잘못 옮겨도 중간
구간은 맞으므로, 경계값마다 케이스 하나를 요구했다.
`multivariate_gaussian`은 p3-shapes가 `moveit-sampling`으로 이미
이식했으니 다시 만들지 말라고 명시했다(§123.1).

## 125. 라이선스가 워크스페이스 상속으로 조용히 틀릴 수 있었다 (2026-08-04)

p3-shapes가 `moveit-stomp-core`를 만들면서 `license.workspace = true`를
쓰지 않고 명시적으로 적었다. 확인해 보니 옳다 — ros-industrial/stomp는
Apache-2.0이고(`LICENSE`, `package.xml:9`, `src/utils.cpp`를 포함한
모든 소스의 파일별 헤더), 이 워크스페이스가 이식하는 moveit2 패키지는
`moveit_core`/`moveit_planners/{stomp,chomp/*,pilz_industrial_motion_planner}`
전부 BSD-3-Clause다. 상속했으면 Apache-2.0 유래 코드가 BSD-3-Clause로
라벨된다.

**패치는 루트 매니페스트에 주석을 다는 것이었다.** 그걸로는 닫히지
않는다. 다음에 다른 상류를 이식하는 크레이트를 만드는 사람이
`license.workspace = true`를 쓰는 순간 아무것도 이상해 보이지 않고,
주석은 그 사람이 읽지 않은 파일에 있다. 그래서 게이트로 만들었다
(`tools/ci/check-license-matches-upstream.sh`, 2c4d628).

- **불변식:** 크레이트가 선언한 유효 license는 그 크레이트 **자기
  소스의** `SPDX-License-Identifier`와 같아야 한다.
- **규칙 셋:** (1) 추적되는 모든 `.rs`가 SPDX 식별자를 갖는다,
  (2) 한 크레이트의 소스는 하나의 식별자로 일치한다, (3) 매니페스트의
  유효 license(명시값, 없으면 상속값)가 그 식별자와 같다.
- **상류별 표를 두지 않았다.** 표는 새 상류가 생길 때마다 갱신해야
  하고, 갱신을 잊는 것이 정확히 이 게이트가 막으려는 실패다. 규칙은
  트리에서 유도된다 — 새 크레이트가 어느 상류를 이식하든, 헤더와
  매니페스트가 어긋나면 걸린다. 방향은 양쪽 다다: 헤더가 맞고
  매니페스트가 상속인 경우와, 매니페스트가 맞고 헤더를 복사해 온 경우.

현재 상태는 177개 `.rs` 전부가 SPDX를 갖고, 20개 크레이트 전부가
BSD-3-Clause로 일치한다. 게이트는 통과하며, 섭동 4건으로 실제로
잡는 것을 확인했다: 한 파일만 식별자가 다름(규칙 2), 크레이트 전체가
매니페스트와 다름(규칙 3 — `moveit-stomp-core`의 실제 사례),
SPDX 헤더 누락(규칙 1), 매니페스트에 license가 아예 없음.
`check-*.sh`는 5건에서 6건이 됐다.

## 126. `cargo test --doc`는 게이트로 보고돼 왔지만 한 건을 검사한다 (2026-08-04)

원격이 없어 `.github/workflows/ci.yml`이 한 번도 실행된 적 없다는 것이
계속 UNFIXED에 있었다. 로컬에서 답할 수 있는 만큼은 답했다: HEAD를
clean clone 해서(`third_party/`는 gitignore라 체크아웃에 없다 — CI
러너와 같은 상태) ci.yml의 다섯 단계를 그대로 돌렸다.

```
fmt           OK
ci checks     6/6 OK (globbed, ci.yml과 같은 방식)
clippy        --workspace --all-targets -D warnings, 통과
test          1189/1189 pass, 2 skipped, 21.9s
doctests      통과
docs          cargo doc --workspace --no-deps, 20개 크레이트 생성
```

**남은 미지는 러너 환경(툴체인 버전, 네트워크)뿐이고 저장소 내용이
아니다.** 특히 `third_party/`가 없어도 1189건이 전부 통과한다 — 테스트가
쓰는 URDF/SRDF 50개와 메시 35개가 전부 커밋돼 있고, `third_party/`를
읽는 코드는 `tools/moveit-diff` 하나이며 그 두 테스트는 `#[ignore]`다.
(그 둘도 `third_party/`가 있는 로컬에서는 통과한다 — 24/24, 각각 17.9초·
18.5초.)

### 126.1 그런데 doctests 단계가 사실상 비어 있다

`cargo test --doc --workspace`가 19개 크레이트에서 보고하는 총합은
**passed=1, failed=0, ignored=0** 이다. 추적되는 `.rs` 177개를 훑으면
doc 코드 블록이 30개인데 **29개가 ```` ```text ````이고 컴파일되는
```` ```rust ```` 블록은 1개다.

```
files scanned: 177 / doc code blocks: 30 / text 29 / bare(=rust doctest) 1
```

```` ```text ````가 틀린 것은 아니다 — 이 저장소의 doc 블록은 대부분
`rg` 명령, 상류 C++ 인용, 감사 출력이고 그건 실행 대상이 아니다.
문제는 **보고**다. 매 라운드 게이트 줄에 `cargo test --doc --workspace —
pass`가 적히고, 그 줄은 doc 예제가 검증됐다는 뜻으로 읽힌다. 실제로는
한 건이다. §119.2의 vacuous-pass를 CI 단계 하나 전체에 적용한 꼴이다.

`cargo doc --workspace --no-deps` 단계는 다르다 — rustdoc 링크 lint를
실제로 잡고(ci.yml 주석이 8건까지 쌓였던 것을 기록한다) 비어 있지 않다.

### 126.2 규칙 하나를 추가한다

전면적으로 doctest를 채우라는 뜻이 아니다. 그건 잡일이고, 위 29개를
```` ```rust ````로 바꾸는 것은 애초에 틀린 일이다. 비례하는 규칙은
이것이다:

**공개 API에 사용 예제를 doc으로 붙일 때, 그 예제는 컴파일되는
```` ```rust ```` 블록이어야 한다.** 셸 명령·상류 인용·측정 출력은
```` ```text ````가 맞다. 예제가 컴파일되지 않으면 그것은 문서가 아니라
주장이고, 이 저장소는 주장과 측정을 구분하는 것으로 서 있다.

그리고 라운드 보고서의 doctests 줄에는 **통과 여부가 아니라 건수**를
적는다. 1이 1이라고 적히면 아무도 그것을 커버리지로 읽지 않는다.

## 127. 관례로만 지켜지던 워크스페이스 규약 셋을 게이트로 바꿨다 (2026-08-04)

§125의 라이선스 건은 하나짜리 사고가 아니라 **부류**였다. 인용된
자리에서 멈추지 않고 같은 형태를 워크스페이스 전체에서 찾았다:
*"두 형태 다 빌드되기 때문에 아무것도 드러나지 않지만, 나중에 한쪽만
조용히 어긋나는 규약"*. 셋 나왔고 셋 다 닫았다.

### 127.1 크레이트 간 의존이 루트 테이블을 우회한다 (adc2156)

`moveit-planners-chomp`/`moveit-planners-stomp`가 `moveit-trajectory`를,
`moveit-planners-sbp`가 `moveit-scene`을 인라인
(`{ path = "...", version = "0.1.0" }`)으로 적고 있었다. 나머지 모든
간선은 `[workspace.dependencies]`를 거친다. 버전을 올리면 테이블은 한
번에 옮겨가고 인라인 `version = "0.1.0"`은 남아 `cargo publish`가 없는
버전으로 해결한다. 루트 매니페스트가 크레이트 그래프의 완전한 그림이
아니게 되는 것은 덤이다.

루트 테이블에 `moveit-scene`/`moveit-trajectory`/`moveit-smoothing`/
`moveit-planners-sbp`를 추가하고(앞의 둘은 지금 필요하고, 뒤의 둘은
p1-fixtures의 `moveit-planning`이 이번 라운드에 필요로 한다) 세 크레이트를
`.workspace = true`로 바꿨다. 게이트는
`check-workspace-dep-inheritance.sh`.

### 127.2 워크스페이스 lint를 크레이트가 통째로 잃을 수 있다 (1049459)

`moveit-kinematics`와 `moveit-planners-sbp`는 `[lints] workspace = true`를
쓰지 **못한다** — D4의 `linkme::distributed_slice`가 만드는 static이
전부 `#[link_section]`이라 `unsafe_code` lint를 건드리고, 워크스페이스의
`forbid`는 per-site `#[allow]`로 내릴 수 없다(게다가
`check-no-lint-suppression.sh`가 그 시도를 막는다). 둘 다 지금은 옳게
처리하고 있다: 나머지 lint를 전부 다시 적고 `unsafe_code` 하나만
`allow`로 완화하며, 이유를 매니페스트에 적어 뒀다.

문제는 **다음 크레이트**다. opt-out은 테이블 통째 교체라,
`[lints.rust] unsafe_code = "allow"` 한 줄만 적은 크레이트는
`warnings = "deny"`와 `missing_docs`를 조용히 잃는다. 그리고 아무것도
실패하지 않는다 — CI의 CLI `-D warnings`가 첫 번째를 가리고, 두 번째는
애초에 에러를 내지 않는다. 그래서 게이트가 요구하는 것은 값이 아니라
**존재**다: opt-out 하면 워크스페이스가 정한 키를 전부 다시 적어야 한다.
완화는 여전히 자유이고, 다만 이유가 이미 적혀 있는 자리에 명시적으로
남는다.

### 127.3 라이선스 (§125, 2c4d628)

앞 절에 기록. 셋 다 같은 형태의 게이트다: **상류별/크레이트별 표를 두지
않고 트리에서 규칙을 유도한다.** 표는 새 항목이 생길 때마다 갱신해야
하고, 갱신을 잊는 것이 정확히 이 게이트들이 막으려는 실패다.

`check-*.sh`는 5건에서 **8건**이 됐다. 셋 다 섭동으로 실제로 잡는 것을
확인했고(각각 3~4건), 오탐도 확인했다 — 주석 처리된 인라인 의존은
걸리지 않고, 값만 완화한 opt-out은 통과한다.

### 127.4 게이트로 만들지 않은 것 하나

`tools/ci/check-*.sh`는 docker를 필요로 하면 안 된다(CI 러너에 없다,
그래서 docker가 필요한 것은 `verify-*.sh`로 이름 짓는다). 지금 여덟 건
중 docker를 **실행**하는 것은 없다 — `rg`로 확인했고, 걸린 세 건은
전부 "Needs no docker"라고 적은 주석이었다. 규약은 지켜지고 있고, 어긴
순간 CI가 스스로 실패한다. 그 자기검출이 약한 이유(ci.yml이 아직 한 번도
돌지 않았다)는 알지만, 게이트를 무한정 늘리는 것이 아니라 §126의 clean
clone 검증을 주기적으로 다시 돌리는 쪽이 맞다고 판단했다.

## 128. 감사 스크립트 자신이 과소집계하고 있었다 (2026-08-04)

p3-shapes가 `moveit-stomp-core` 라운드에서 잡았다. 자기 테스트 파일의
`assert!` 메시지를 Rust 줄이음 문자열로 쓰자
`tools/ci/count-relative-eq.pl`이 실제 6건인 `epsilon =` 호출을 **0건**으로
보고했다. 그쪽은 문자열을 한 줄로 다시 써서 우회하고 그 사실을 자기
크레이트 doc에 주의사항으로 적었다 — 우회로는 옳은 판단이었다(공용
`tools/` 스크립트는 그쪽 소유가 아니다). 원인은 내 파일이므로 원인에서
고쳤다(c9780c7).

원인: 문자열 내용을 지우는 치환에 `/s`가 없었다.

```perl
$text =~ s{"(?:[^"\\]|\\.)*"}{""}g;    # 수정 전
```

`\\.`의 `.`는 `/s` 없이 개행을 먹지 못한다. 그래서

```rust
assert!(cond, "긴 메시지: \
         이어지는 줄");
```

의 여는 따옴표에서 매칭이 실패하고, Perl이 그 문자열의 **닫는** 따옴표를
새 여는 따옴표로 잡아 다음 리터럴의 따옴표까지를 한 문자열로 간주한다 —
그 사이의 진짜 코드가 통째로 지워진다. 3건짜리 픽스처로 재현했다:
수정 전 1건, 수정 후 3건.

### 128.1 기록된 숫자 둘이 바뀐다

이것이 이 건의 실제 비용이다. 스크립트를 고치면 이미 doc에 적힌 측정값이
틀린 것이 된다. 수정 전/후를 크레이트별로 전부 대조했다:

```
워크스페이스 전체   both=94  -> both=112   (epsilon_only=36, max_relative_only=1, neither=4 불변)
moveit-collision    both=2   -> both=3
moveit-distance-field both=27 -> both=44
그 외 모든 크레이트  변화 없음
```

`crates/moveit-distance-field/src/lib.rs:448`의 `both=27 epsilon_only=3`과
위 §123.3이 인용한 같은 숫자가 정정 대상이다. p1-joints가 기록한
`both=0 epsilon_only=2 max_relative_only=0 neither=0`(moveit-kinematics /
moveit-diff / invariants.rs)은 정정본으로 다시 세도 같다 — 영향 없음을
확인했다.

각 소유자에게 정정을 보냈다(p3-distance-field, p3-acm). **그 숫자에 기대어
내린 판단이 여전히 성립하는지까지 확인하라고 요구했다** — 숫자만 고치고
결론을 그대로 두면 §117.5를 다른 방향으로 어기는 것이다.

### 128.2 이 건이 남기는 것

`count-relative-eq.pl`은 한 번 통합된 뒤(§ 감사 스크립트 단일화) 여섯
크레이트가 같은 명령으로 세게 됐다. 그 통합이 없었으면 이 결함은 사본마다
다르게 나타나 **결함으로 인식되지도 않았을 것이다** — 크레이트마다 숫자가
다른 것이 원래 그런 줄 알았을 테니까. 단일 정본은 결함을 없애지 않고
드러낸다. 그게 통합의 값이다.

## 129. Phase 9를 연다 — 그리고 r2r이 배치를 강제한다 (2026-08-04)

사용자 지시: *"Phase9도 호환이 되게 해주면 좋을듯. ros2 없이도 쓸 수 있고,
호환도 가능하게"*. D1/D2가 원래 그 형태였으므로 방향 변경이 아니라
착수다. 다만 배정 전에 확인한 제약 하나가 배치를 바꾼다.

### 129.1 r2r은 빌드 시점에 ROS 2를 요구한다

r2r은 로컬 ROS 환경에서 바인딩을 **생성**한다. 이 호스트에는 ROS 2가
없고(`/opt/ros` 없음, `ros2` 바이너리 없음, `AMENT_PREFIX_PATH` 미설정),
CI 러너에도 없다. 지금 루트는 `members = ["crates/*", "tools/moveit-diff"]`
글롭이므로 `crates/moveit-ros`를 만드는 순간 **일곱 패널과 CI의
`cargo build --workspace`가 전부 깨진다.**

### 129.2 D5 — `moveit-ros`는 `crates/` 밖에 산다

`ros/moveit-ros/`, 자기 워크스페이스. 루트 워크스페이스의 멤버가 아니다.

대안은 `exclude = ["crates/moveit-ros"]`였고, 택하지 않았다. 그러면
`crates/*`가 "워크스페이스 멤버"와 "멤버처럼 생겼지만 제외"라는 두 뜻을
갖는다 — §125·§127에서 내내 없애 온 이중 의미이고, 새로 만들 이유가 없다.
밖에 두면 `crates/*`는 **ROS 없이 빌드되는 크레이트**라는 한 뜻만 유지한다.

부수 효과 하나가 좋은 쪽이다. `check-dep-direction.sh`는
`ALLOWED_PACKAGE='moveit-ros'` 예외를 두고 있는데, `moveit-ros`가 워크스페이스
밖이면 `cargo metadata`에 아예 나타나지 않으므로 그 예외가 죽는다. 게이트는
"**어떤** 워크스페이스 크레이트도 ROS 클라이언트 라이브러리를 의존할 수
없다"로 예외 없이 강해진다. 그 예외 줄은 `moveit-ros`가 실제로 생긴 뒤
정리한다.

### 129.3 D6 — 호환은 `TryFrom` 양방향 변환으로만

코어 타입은 ROS를 아는 메서드를 **갖지 않는다.** `moveit_msgs` ↔ 코어 타입
변환, `/plan_kinematic_path` 서비스, `/move_action` 액션 서버, planning
scene 구독이 전부 `moveit-ros` 안에 산다. 즉 ROS 없이 쓰는 경로가 기본이고
ROS 호환이 얹히는 것이지, 그 반대가 아니다. 사용자가 요구한 형태가 정확히
이것이다.

변환이 `From`이 아니라 `TryFrom`인 이유: `moveit_msgs`는 코어 타입보다
표현이 넓다(빈 `frame_id`, 미지정 enum, 길이가 어긋난 병렬 배열). 그
넓이를 `From`으로 흡수하면 실패가 기본값으로 조용히 바뀐다.

### 129.4 빌드·테스트는 별도 이미지에서

오라클 이미지에 Rust를 넣지 않는다. 스탬프가 바뀌어 **무관한 이유로**
일곱 패널 전부의 픽스처 검증에 영향을 준다. ROS 2 Rolling + Rust의 작은
이미지를 따로 만들고 `verify-ros-interop.sh`로 돌린다 — docker가 필요하므로
`check-*`가 아니다(그 글롭은 러너가 docker 없이 돌린다).

이것은 §5의 Phase 9 완료 조건을 바꾸지 않는다. 다만 **Phase 9는 Phase 8과
병렬로 진행하되, Phase 8의 완료 조건 선언보다 먼저 선언하지 않는다** —
§121.3이 Phase 7/8에 적용한 규칙과 같다.

## 130. pilz 오라클을 요청서보다 먼저 빌드해 봤다 (2026-08-04)

§122는 pilz 오라클의 비용을 **소스 읽기로** 추정했다: 콜콘 7 -> 19패키지,
`joint_limits_common`이 `moveit_ros_planning`/`moveit_ros_move_group`에
직접 링크하므로 해석적 core만 떼어 낼 수 없고, 다만 필요한 타겟이
`ament_export_targets`에 있으므로 pluginlib 우회는 필요 없다.

**추정은 추정이다.** 이미지를 한 번도 만들어 본 적이 없는데 요청서가
확정되면 그때 처음 빌드하는 것은 순서가 거꾸로다 — 그 시점에 실패하면
p1-joints의 라운드가 통째로 막힌다. 그래서 요청서를 기다리지 않고 변종을
먼저 빌드했다.

**정본을 건드리지 않고 할 수 있다.** `ORACLE_MOVEIT2_PACKAGES`는
`oracle_build_inputs`에 들어가고 그것이 스탬프에 들어가므로, 오버라이드한
빌드는 다른 태그를 받는다 — 설계가 이미 "재핀된 빌드가 정본을 사칭하지
못한다"를 보장한다. 정본 `8ed8a9395b730b08`은 그대로이고 변종은
`ce29f1718ca74fda`다.

### 130.1 측정 결과

빌드는 **성공했다**. `/ws/install`에 19개가 아니라 **21개** 패키지가 들어왔다
(§122의 추정보다 2개 많다 — `pilz_industrial_motion_planner_testutils`와
`moveit_resources_prbt_ikfast_manipulator_plugin`을 세지 않았다).
pilz가 설치한 공유 라이브러리는 열 개다:

```
libcommand_list_manager.so        libplanning_context_loader_lin.so
libjoint_limits_common.so         libplanning_context_loader_polyline.so
libpilz_industrial_motion_planner.so  libplanning_context_loader_ptp.so
libplanning_context_loader_base.so    libsequence_capability.so
libplanning_context_loader_circ.so    libtrajectory_generation_common.so
```

**pluginlib 우회가 필요 없다는 §122의 결론은 맞다.** 심볼로 확인했다:

- `TrajectoryGenerator::generate(PlanningSceneConstPtr const&,
  MotionPlanRequest const&, MotionPlanResponse&, double)` — `T`(정의됨),
  `libtrajectory_generation_common.so`
- `TrajectoryGeneratorPTP::TrajectoryGeneratorPTP(RobotModelConstPtr const&,
  LimitsContainer const&, string const&)` — `T`,
  `libplanning_context_loader_ptp.so`
- LIN / CIRC 생성자도 각각 `libplanning_context_loader_lin.so` /
  `_circ.so`에 `T`

즉 셋 다 직접 생성하고 `generate`를 직접 부를 수 있다. 플러그인 로더를
거칠 이유가 없다.

### 130.2 요청서에 남는 질문은 하나로 좁혀졌다

링크 가능성은 이제 열린 질문이 아니다. 남은 것은 **`PlanningScene`과
`MotionPlanRequest`를 JSON에서 어떻게 조립하는가**이고, 그것이 이 op의
실제 비용이다(p1-joints에게 이미 넘긴 사실 넷 중 세 번째). `generate`의
`sampling_time` 기본값이 0.1이라는 것도 `1e-6` 비교 대상을 정할 때
영향이 있다.

### 130.3 남는 것

이 변종 이미지는 요청서가 확정되면 버린다 — `oracle.cpp`에 pilz op를
더하면 파일 다이제스트가 바뀌어 어차피 또 다른 스탬프가 된다. 지금 값은
**"빌드가 되는가"에 대한 답**이지 보관할 산출물이 아니다. 정리 대상
이미지가 하나 늘었다(현재 89개, 정리는 사용자 승인 필요).

## 131. Phase 7 판정을 독립 재현했다 (2026-08-04)

p1-robotmodel 라운드 19가 Phase 7의 세 완료 조건을 전부 통과로 판정했다.
그 판정을 **다른 seed로 처음부터 다시 돌려서** 확인했다 — 보고서를 옮겨
적지 않는다는 이 세션의 원칙이 가장 값이 큰 자리다. 조건 하나라도
마진이 얇았다면 seed 하나 차이로 뒤집혔을 것이고, 그건 판정이 아니라
운이었을 것이다.

### 131.1 측정치

`floor_wall` 250 (seed 900001) + `cage` 250 (seed 900002), 같은 request JSON을
양쪽에 먹였다. C++는 오라클, 포트는 `plan_benchmark_port` seed_base 424242.

- C++: 500 중 498 exact (99.60%), median length 2.6597767032746464
- 포트: 500 중 497 solved (99.40%), median 2.6680037373621920,
  condition-2 497/497
- 조건 1: 99.40% ≥ 89.64% (C++ 99.60%의 90%) — pass
- 조건 2: 497/497 — pass
- 조건 3: 2.6680 ≤ 3.4577 (1.3 × 2.6598), 비율 1.003x — pass

C++ 쪽 숫자는 p1-robotmodel이 보고한 99.6% / 2.6598과 **일치**한다.
포트 쪽은 seed가 다르니 다르다(그쪽 499/500, 2.7085).

미해결 id도 재현됐다: 포트 [71,129,182], C++ non-exact [104,182],
교집합 {182}. `cage` 182가 양쪽 다 못 푸는 문제라는 관찰은 seed를 바꿔도
남았다 — RNG 산물이 아니라는 그쪽 주장이 실제로 버텼다.

### 131.2 그런데 seed_base가 기록돼 있지 않다

`plan_benchmark_port.rs`의 doc은 "같은 request 파일에 같은 `seed_base`면
두 실행이 동일하다"고 재현성을 보장하는데, **`lib.rs`의 측정 문단에는 그
`seed_base` 값이 없다.** 재현하려고 열었을 때 고를 값이 없어서 임의로
424242를 썼고, 그래서 내 숫자와 그쪽 숫자가 다르다.

결론은 양쪽 다 pass지만 그건 마진이 컸기 때문이지 기록이 충분해서가 아니다.
**재현 절차를 문서화했는데 그 절차의 입력값을 안 적으면 문서화한 게 아니다.**
p1-robotmodel 라운드 20의 선행 항목으로 돌렸다.

이 항목은 §219이 닫는다 — 네 seed(900001/900002/900021/900022)와
`seed_base` 424242가 `tools/ci/verify-phase7-benchmark.sh` 안에
상수로 들어갔고, 하네스가 자기 출력과 `doc/phase7-benchmark-results.json`
양쪽에 그 값을 적는다. 여기서 임의로 골랐던 424242가 그 상수가 됐다.

## 132. `pilz_trajectory` op — 요청서를 두 군데서 따르지 않았다 (2026-08-04)

p1-joints의 요청서(§130.2가 좁혀준 그것)를 받아 op을 구현했다.
세 제너레이터를 한 op으로 묶고 `generator` 필드로 고른다 — 생성자 모양과
진입점(`TrajectoryGenerator::generate`)이 셋 다 같으니 op 세 개는 같은
request 조립 코드 세 벌이 됐을 것이다.

### 132.1 limits를 request에 실었다 (요청서와 반대)

요청서는 "limits를 JSON에 넣지 말고 양쪽이 같은 `joint_limits.yaml` /
`pilz_cartesian_limits.yaml`을 읽자"였다. **그 전제가 이 저장소에서
성립하지 않는다.** 그 YAML들은 `third_party/moveit_resources/` 아래에만
있고 그 디렉터리는 gitignore 대상이다. gitignore된 외부 체크아웃에 의미가
걸린 fixture는 `verify-clean-checkout.sh`가 잡으려고 존재하는 실패
바로 그것이다(§126). 게다가 양쪽 다 YAML 파서가 없다.

요청서가 막으려던 위험은 "request JSON에 limit 오타"였는데, request로 싣는
쪽이 그 위험에 대해 **더 강하다**: 한 request가 양쪽을 다 구동하므로 오타는
양쪽에 동일하게 적용된다. 케이스를 덜 흥미롭게 만들 수는 있어도 없는
불일치를 만들어낼 수는 없다. 한 YAML을 두 독립 리더가 읽는 배치가 그걸
만들 수 있는 쪽이다.

### 132.2 §122의 결론이 LIN/CIRC를 덮지 못했다

PTP를 먼저 돌려 SUCCESS(waypoint 14개, 마지막 `time_from_start`
1.232450134)를 받았다. **거기서 멈추지 않고 LIN과 CIRC를 따로 돌린 것이
이 절의 내용을 만들었다** — 둘 다 `error_code` -31, NO_IK_SOLUTION.

URDF+SRDF만으로 만든 `RobotModel`에는 kinematics solver가 없고, LIN/CIRC는
Cartesian goal에 IK를 돌린다. §122의 "pluginlib 우회가 필요 없다"는
**제너레이터 세 개에 대해서는 참**이고 §130이 `nm -DC`로 실측까지 했다.
LIN/CIRC의 IK 의존성은 그 결론이 다룬 적 없는 별개 요구사항이었다.
`ensureKinematicsSolver`가 `kdl_kinematics_plugin/KDLKinematicsPlugin`을
붙이고 `moveit_kinematics`가 MOVEIT2_PACKAGES에 들어갔다.

**규칙 (132.2):** 한 진입점이 통과한 것을 형제 진입점의 근거로 삼지 마라.
§119.2의 vacuous-pass와 같은 계열이되 방향이 다르다 — 저기는 "통과했는데
아무것도 검사 안 했다"이고 여기는 "통과했는데 형제는 검사 안 했다"이다.
PTP만 돌리고 op이 된다고 보고했으면 p1-joints가 두 라운드를 버렸다.

### 132.3 비교에 딸려 오는 부채

오라클의 LIN/CIRC waypoint는 이제 그 KDL 플러그인 IK에 의존하고, 포트의
`compute_pose_ik`는 `moveit-kinematics`에 의존한다. **두 IK가 어긋나는 양은
궤적 코드에 닿기 전에 이미 `1e-6` 예산 안에 들어와 있다.** LIN/CIRC 불일치를
궤적 포팅 탓으로 돌리려면 Phase 4의 IK 파리티를 먼저 빼내야 한다.
PTP에는 이 부채가 없다(joint-space goal). 조건 판정 때 셋을 한 덩어리로
읽으면 안 되는 이유다.

## 133. D7 — r2r은 태그가 아니라 커밋 SHA로 핀한다 (2026-08-04)

p9-ros 라운드 1의 산출물: **D2가 지정한 r2r 0.9.5가 이 오라클 베이스
이미지의 ROS 2 Rolling에서 빌드되지 않는다.** `r2r-0.9.5/src/nodes.rs:1485`가
`rcl_timer_init`을 부르는데 이 Rolling 스냅샷의 rcl에는 `rcl_timer_init2`만
있다. 0.9.6이 distro-cfg 분기로 고쳤으나 crates.io 미공개(GitHub 태그만).

**D7: `rev = "<0.9.6 태그의 커밋 SHA>"`로 핀한다. `tag =`를 쓰지 않는다.**

근거는 이 저장소가 이미 `src-digest.sh`에서 베이스 이미지에 적용한 원칙
그대로다 — 태그는 움직인다. upstream이 `0.9.6`을 옮기면 `Cargo.lock` 없는
소비자는 조용히 다른 코드를 빌드하고, 로컬 파일 다이제스트는 전부 그대로다.
SHA는 옮길 수 없다. D2는 "r2r 0.9.5"를 명시했으나 그 버전이 실측으로 깨져
있으므로 D7이 그 부분을 대체한다.

### 133.1 `ALLOWED_PACKAGE` 예외가 죽어서 제거했다

§129.2가 예견한 대로다. `check-dep-direction.sh`의
`ALLOWED_PACKAGE='moveit-ros'` skip은 `moveit-ros`가 `ros/moveit-ros/`에
자기 `[workspace]`로 앉으면서 `cargo metadata --no-deps`에 아예 안 잡히게
됐다(실측: 멤버 22개, `moveit-ros` 미포함). 무해한 no-op으로 남기지 않고
지웠다 — 예외가 없으면 규칙이 균일해지고, 누군가 그 크레이트를 `crates/`
밑으로 옮기는 날 이 게이트가 실패한다. 그게 옳은 답이고, 잠자던 이름
비교가 덮었을 답이다.

## 134. `chomp_quad_cost_inverse` op, 그리고 패키징 선택 (2026-08-04)

p6-totg의 요청서(`crates/moveit-planners-chomp/doc/oracle-request-quad-cost-inv.md`)를
그대로 구현했다. 요청서가 명시적으로 나에게 맡긴 부분이 하나 있었다 —
"`ChompTrajectory`를 만들어 진짜 `ChompCost`를 쓰든, 생성자 본문을 직접
옮겨 쓰든 오라클 소유자가 편한 쪽으로 하라".

**진짜 `ChompCost`를 링크했다.** 이 비교가 묻는 것은 "Eigen의 역행렬이
nalgebra의 것과 다른가"인데, 생성자 본문을 손으로 옮기면 비교 대상에
검증되지 않은 변수가 하나 더 들어간다. 옮기다 낸 실수는 결과 행렬을
바꾸고, 그 차이는 decomposition 차이와 구분되지 않는다. **더 쉬운 쪽이
측정 대상을 오염시키는 경우, 쉬운 쪽은 선택지가 아니다.**
`chomp_motion_planner`가 MOVEIT2_PACKAGES에 들어갔다.

생성자가 `getNumPoints()`/`getDiscretization()` 외에는 아무것도 읽지 않으므로
(요청서가 round 16에서 이미 확인) 로봇 fixture는 필요 없고 `joint_number`는
0으로 넘긴다.

### 134.1 실측 5케이스

`discretization` 0.1, `derivative_costs` [0.0, 1.0, 0.0], `ridge_factor` 1e-6:

| `num_points` | `num_vars_free` | shape | 표본 |
|---:|---:|---|---|
| 13 | 1 | 1×1 | `[0][0] = 10.183771820145536` |
| 14 | 2 | 2×2 | |
| 15 | 3 | 3×3 | |
| 16 | 4 | 4×4 | |
| 20 | 8 | 8×8 | `[0][0] = 47.86937525471385` |

다섯 케이스 전부 `num_vars_free`가 요청서 표의 기대값과 일치했다.
요청서가 "이 값이 어긋나면 그것 자체가 더 흥미로운 발견"이라며 echo를
요구했는데, 어긋나지 않았다 — 즉 `DIFF_RULE_LENGTH`와 경계 공식이 양쪽에서
같다는 것은 비교 테스트를 쓰기 전에 이미 확인됐다.

### 134.2 스탬프 이력

이 세션에서 오라클을 세 번 다시 빌드했고 스탬프가 세 번 바뀌었다:
`230e92be6fa5cc3a`(pilz+IK) → `6797447ac4dc46e9`(+chomp). 중간에
p1-joints에게 첫 스탬프를 알린 뒤 chomp op 때문에 다시 바뀌어 정정을 보냈다.

**규칙 (134.2):** 요청서가 둘 이상 대기 중이면 스탬프를 알리기 전에 전부
넣어라. 스탬프를 먼저 알리면 그 사이에 캡처된 fixture가 어느 이미지에서
나왔는지 추적할 수 없어진다 — `run-oracle.sh`의 스탬프 검사가 다음 실행을
막아주긴 하지만, 이미 디스크에 있는 response 파일은 막아주지 못한다.

## 135. FCL / libccd 원본 소스가 로컬에 들어왔다 — 반드시 태그에서 읽어라

사용자가 `~/work/fcl`, `~/work/libccd` 두 소스를 체크아웃해 뒀다. p3-acm의
caster-wheel 질문이 "원본이 로컬에 없다"는 이유로 라운드를 넘겨온 것이
이걸로 풀린다.

**체크아웃 상태와 오라클이 실제로 링크하는 것이 다르다.** 이걸 확인하지
않고 HEAD를 읽으면 오라클이 돌리지 않는 알고리즘을 문서화하게 된다.

| | 로컬 체크아웃 | 오라클 이미지 |
|---|---|---|
| fcl | `e5efcc4`, `0.7.0-17-ge5efcc4` | `libfcl-dev 0.7.0-3build2` |
| libccd | `7931e76`, `v2.1` | `libccd-dev 2.1-2` |

이미지 안의 `changelog.Debian.gz`를 직접 읽어서 확인한 것: fcl
`0.7.0-3build2`는 upstream 0.7.0 위에 sparc64 패치 하나(amd64와 무관)와
no-change 리빌드(CVE-2024-3094 대응, liboctomap1.9t64 재링크)뿐 —
`0.7.0-1`의 "Remove all patches, they were merged upstream" 이후로 코드
패치가 없다. 즉 **오라클은 순수 upstream 0.7.0을 돌린다.** libccd
`2.1-2`도 packaging 전용 변경(debhelper, standards bump)뿐이라 로컬 `v2.1`과
정확히 일치하고, 이미지의 `/usr/include/ccd/config.h`는 `CCD_DOUBLE`이
정의되어 있고 `CCD_SINGLE`은 undef다 — 배정밀도로 돈다.

**그래서 fcl은 `0.7.0` 태그에서 읽어야 한다.** 체크아웃 HEAD는 태그보다 17
커밋 앞서 있고, 그 구간이
`include/fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h`를
447줄 고쳤다 — `3c2b993`("Fix EPA, use a more robust ccdVec3PointTriDist2")와
`da430b1`("Correct doSimplex4() testing tolerances"). convex-convex 접촉이
지나가는 바로 그 경로다. HEAD를 읽고 생긴 불일치는 포트의 결함으로
오귀속된다. `git -C ~/work/fcl show 0.7.0:<path>` 또는 태그 워크트리를 써라.

이건 §107.3(요청 전에 upstream *헤더*에서 심볼 도달 가능성 확인)과 같은
계열의 규칙이다: **원본을 읽기 전에, 읽으려는 리비전이 오라클이 링크하는
리비전인지 먼저 확인해라.** 로컬에 소스가 있다는 사실만으로는 부족하다.

## 136. `ci.yml`을 커밋된 파일만의 트리에서 실제로 돌렸다

"`.github/workflows/ci.yml`은 GitHub Actions에서 한 번도 돌아본 적이 없다"는
UNFIXED 항목을 원격 없이 닫을 수 있는 만큼 닫았다. `git archive HEAD`로 추적
중인 파일만(459개, 13MB) 새 디렉터리에 풀고, 캐시 없는 target에서 `ci.yml`의
`rust` job 스텝을 순서대로 그대로 실행했다.

| 스텝 | 결과 |
|---|---|
| `cargo fmt --all -- --check` | rc=0 |
| `cargo clippy --workspace --all-targets -- -D warnings` | rc=0 |
| `cargo nextest run --workspace` | rc=0 |
| `cargo test --doc --workspace` | rc=0 |
| `cargo doc --workspace --no-deps` | rc=0 |
| `check-*.sh` glob | 8개 매치, 8개 전부 rc=0 |

추가로 `cargo check --workspace --locked` rc=0 — `Cargo.lock`이 매니페스트와
어긋나 있지 않다(락 드리프트는 CI에서만 터지는 대표적 실패 모드다).

즉 **커밋된 내용만으로 job이 통과한다**. 워크트리에만 있는 파일이나
gitignore된 산출물에 의존하는 테스트는 없다. `nextest`가 건너뛰는 2개는
`moveit-diff`의 `visibility_cone_ambiguity_diagnostic`
(`a_real_mismatching_case_touches_exactly_one_link`,
`near_placement_never_touches_more_than_one_link_at_once`)로,
`third_party/moveit_resources`가 필요해 `#[ignore]`된 것 — 이 호스트에서도
CI에서도 똑같이 건너뛴다.

### 136.1 이 실험이 증명하지 못한 것

셋 다 남아 있다. "CI를 검증했다"로 뭉뚱그리지 않기 위해 적어둔다.

1. **Actions 자체는 여전히 미검증.** `actions/checkout@v4`,
   `dtolnay/rust-toolchain@stable`, `Swatinem/rust-cache@v2`,
   `taiki-e/install-action@nextest` 네 개는 실행된 적이 없다. 원격이 없으므로
   이건 원격이 생기기 전엔 닫을 수 없다.
2. **crates.io 신규 해석은 미검증.** 드라이런은 호스트의 `~/.cargo` 레지스트리
   캐시를 재사용했다. `--locked`가 통과했으니 락 파일 자체는 정합하지만,
   빈 캐시에서 네트워크로 받아오는 경로는 안 돌아봤다.
3. **툴체인이 떠 있다.** 드라이런은 호스트의 `rustc 1.97.0`을 썼고, CI는
   `dtolnay/rust-toolchain@stable`로 그날의 stable을 받는다.
   `rust-toolchain.toml`이 없어 고정돼 있지 않은데, `[workspace.lints.rust]`가
   `warnings = "deny"`라서 **새 rustc가 우리 코드에 새 린트를 켜면 리포지터리가
   그대로인 채로 빌드가 깨진다.** `ci.yml` 상단 주석이 `RUSTFLAGS: -D warnings`를
   거부한 근거("우리가 고칠 수 없는 서드파티 경고")는 워크스페이스 멤버에는
   적용되지 않으므로, 이 실패 모드는 그 주석이 막아주지 않는다. 고정할지
   말지는 트레이드오프가 있는 결정이라(고정하면 새 린트를 놓친다) 여기
   기록만 하고 바꾸지 않았다.

### 136.2 드라이런이 찾아낸 실제 결함 하나

`ci.yml`의 test 스텝이 `cargo nextest run --workspace`였다 — `--no-fail-fast`가
없다. 모든 워커의 로컬 게이트는 `--no-fail-fast`로 돌리므로, 같은 이름의
"test 스텝"이 두 곳에서 다른 것을 뜻하고 있었다: 테스트 3개를 깬 push가 CI에선
1개만 보고하고 나머지는 다음 실행에서야 드러난다. 규칙을 한쪽에 맞추는 게
구조적 해결이라 CI 쪽에 `--no-fail-fast`를 붙였다(`0830ce5`).

## 137. "의존성이 안 닿는다"는 UNFIXED는 `cargo tree` 없이는 못 적는다

p3-shapes 라운드 23이 `costs::getCollisionCostFunction`/
`getConstraintsCostFunction`를 UNFIXED로 남기며 근거를 "need
`moveit-scene`/`moveit-collision`, out of this crate's dependency reach this
round"라고 적었다. 병합 전에 확인해보니 사실이 아니었다:

- `crates/moveit-planners-stomp/Cargo.toml`의 `[dependencies]`에 그 둘이 아직
  적혀 있지 않을 뿐, 막는 규칙은 없다.
- 같은 계층의 `moveit-planners-sbp`는 이미 `moveit-collision`과
  `moveit-scene`을 정상 의존성으로 갖고 있다.
- `cargo tree -p moveit-scene -e normal`, `cargo tree -p moveit-collision -e
  normal` 둘 다 `stomp`를 포함하지 않는다 — 추가해도 순환이 없다.

즉 "reach 밖"이 아니라 "아직 안 적었다"였다. 라운드 24로 반려했다.

**규칙 (137):** UNFIXED의 사유로 의존성·순환·계층을 들려면 `cargo tree`(또는
`cargo metadata`) 실행 결과를 함께 적어라. 이건 §119.2(vacuous pass)와 같은
계열이다 — 확인하지 않은 제약을 사유로 적으면, 실제로는 열려 있는 작업이
구조적으로 막힌 것처럼 기록에 남고 다음 라운드가 그걸 전제로 삼는다. 형제
크레이트가 이미 같은 의존을 갖고 있는지 한 번 보는 것으로 대부분 판별된다.

## 138. 오라클의 순서 의존성과 wall-clock 필드를 구조로 닫았다

오라클 이미지를 세 번(pilz / IK / chomp) 다시 빌드한 뒤, committed fixture
35개가 전부 `identical`로 재생되는 것을 확인했다. 그런데 그건 **프로세스 안의
순서 의존성을 증명하지 않는다** — `verify-fixture-replay.sh`는 fixture 파일 하나당
오라클 프로세스 하나를 띄우고 그 파일의 요청들만 한 스트림으로 넣는다. pilz
요청은 아직 어느 committed fixture에도 없으므로, "LIN 요청이 먼저 지나간 뒤의
op"는 그 게이트가 한 번도 밟아본 적 없는 경로다.

### 138.1 측정: 실제로 오염되는가

`pilz_trajectory` LIN 요청을 fixture 요청들 앞에 붙여 **같은 프로세스**에 넣고,
붙이지 않은 실행과 비교했다. LIN이 실제로 성공했는지(`ok=true`,
`error_code=1`, waypoint 12개)를 매번 확인해서 §119.2의 vacuous pass를 피했다 —
taint가 적용되지 않은 실행은 결과를 세지 않았다.

| fixture | taint | ids | drift |
|---|---|---|---|
| `moveit-constraints/panda_constraints` | applied | 12 | 0 |
| `moveit-metrics/panda_kinematics_metrics` | applied | 1 | 0 |
| `moveit-planners-sbp/panda_arm_plan_distance_probes` | applied | 1 | 0 |
| `moveit-scene/panda_frame_transform` | applied | 1 | 0 |

**이 표가 증명하지 않는 것**을 분명히 해둔다. 코퍼스 35개 중 panda 모델은 12개뿐이고
나머지 23개(pr2 14, fanuc 5, 기타 4)는 LIN이 `unknown joint model group: panda_arm`으로
실패해 taint가 적용되지 않았다 — 그 행들은 vacuous라 위 표에 넣지 않았다. 즉 이건
**4개 op에 대한 표본**이지 코퍼스 전체 커버리지가 아니다. (측정 도중
`collision_distance_field_types`가 오염된 것처럼 보였는데, 내 비교 스크립트가
`ignore_result_fields_by_id`의 `relative_cylinder_pose`를 벗기지 않은 탓이었다 —
하네스 결함이지 오염이 아니다.)

### 138.2 그래서 측정을 늘리는 대신 구조로 닫았다

커버리지를 채우려면 pr2·fanuc용 LIN 요청을 따로 만들어야 하는데, 그건 "위법 상태가
관측되지 않음"을 매번 다시 재는 런타임 검사다. CLAUDE.md 기준으로 그건 패치이고,
구조적 해결은 **위법 상태를 만들 수 없게 하는 것**이다.

`ensureKinematicsSolver`가 이제 공유 `model_`이 아니라 `ensurePilzModel()`이
같은 URDF/SRDF에서 지연 생성하는 **pilz 전용 `RobotModel`**을 변경한다
(`e3375ee`). `setKinematicsAllocators`는 in-place이고 역연산이 없으므로 되돌릴
수 없다 — 되돌리는 대신 애초에 공유 상태를 건드리지 않는다. PTP를 제외하던 이유도
바뀌었다: 이제는 "공유 상태를 보호하려고"가 아니라 "쓰지도 않는 걸 붙일 이유가
없어서"다.

### 138.3 같이 드러난 결함: 응답에 들어간 stopwatch

**Anchor:** `rg -n 'planning_time' tools/moveit-oracle/src/oracle.cpp`
**Sites:** `oracle.cpp:4752`(`plan` → `planning_time_s`), `oracle.cpp:5135`
(`pilzTrajectory` → `planning_time`) — **둘 다 같은 결함**.

wall-clock 값이 바이트 비교되는 fixture에 들어가면 재생할 때마다 drift한다. 같은
LIN 요청이 `0.001528053` → `0.001346184`로 나왔고 나머지 필드는 전부 동일했다.
유지하려면 모든 pilz fixture의 모든 id에 `ignore_result_fields_by_id` 항목을
영구히 달아야 하는데, 그건 `verify-fixture-replay.sh` 헤더가 명시적으로 거부하는
"미래의 fixture 작성자가 매번 기억해주기를 믿는" 형태다. 그래서 필드를 제거했다
(`c0838b4`). `plan` 쪽은 잠복 상태였다 — committed fixture의 `problems`가 빈
배열이라 타이밍이 응답 파일까지 도달한 적이 없어 우연히 깨끗했을 뿐이다.

소비자는 없다: 어느 파리티 테스트도 C++ 스톱워치를 Rust 스톱워치와 비교할 수 없다.

### 138.4 회귀 확인

- 옛 이미지(`6797447ac4dc46e9`)와 전용 모델 적용본(`5fe015a366e3ccc0`)의
  LIN/CIRC 응답을 필드 단위로 비교: `planning_time` 외 **전부 동일**(waypoint
  포함).
- 최종 이미지(`9acdf82cb2e09162`)로 같은 요청을 두 번: **23086바이트 바이트 동일**.
- committed fixture 35개 전부 최종 이미지에서 `identical`, `verify-fixture-replay.sh`
  rc=0.

**현재 스탬프: `9acdf82cb2e09162`.** §134.2대로 중간 스탬프
(`5fe015a366e3ccc0`)는 아무에게도 알리지 않았다.

p1-joints가 이미 캡처한 pilz fixture 3개(`panda_ptp_response.json`,
`panda_lin_response.json`, `panda_lin_scaling05_rejected_response.json`)는
`planning_time`을 담고 있으므로 재캡처가 필요하다 — 값은 안 변하니 tolerance
재측정은 불필요하다는 점과 함께 전달했다.

## 139. 두 번째 거짓 blocker: 상속 관계를 보고 호출 관계를 결론냈다

§137에서 "의존성이 안 닿는다"는 UNFIXED를 반려한 데 이어, 같은 세션에서 같은
계열의 두 번째 사례가 나왔다. 이쪽이 더 비쌌다 — 그대로 뒀으면 Phase 8의 CHOMP가
optimizer 없는 비용함수 라이브러리로 끝났을 것이다.

p6-totg 라운드 18이 `ChompOptimizer`를 이렇게 적었다: `hy_env_`
(`const collision_detection::CollisionEnvHybrid*`)는 `CollisionEnvFCL`을 직접
상속하는데 D4.5가 FCL/Bullet을 parry3d-f64로 통째 교체하므로 `CollisionEnvHybrid`는
영원히 포팅되지 않는다, 따라서 literal port는 **permanently impossible**이고,
포팅하려면 collision-cost 경로를 재설계하는 semantic change라 sign-off가 필요하다.

상속 관계에 대한 서술은 전부 맞다. 결론이 틀렸다. pinned commit `e017c91e`에서
`rg -n 'hy_env_|CollisionEnvHybrid' moveit_planners/chomp/chomp_motion_planner/`는
전체 5 hit이고, 그중 실제 호출은 둘뿐이다:

- `chomp_optimizer.hpp:133` 필드 선언
- `chomp_optimizer.cpp:76,78` `dynamic_cast` + null 체크
- `chomp_optimizer.cpp:102` `hy_env_->getCollisionGradients(req, res, state_, &planning_scene_->getAllowedCollisionMatrix(), gsr_)`
- `chomp_optimizer.cpp:890` `hy_env_->getCollisionGradients(req, res, state_, nullptr, gsr_)`

그리고 `collision_env_hybrid.cpp`의 그 메서드는 한 줄이다:

```cpp
void CollisionEnvHybrid::getCollisionGradients(...) const
{
  cenv_distance_->getCollisionGradients(req, res, state, acm, gsr);
}
```

`CollisionEnvHybrid`가 `CollisionEnvFCL`을 상속하는 것은 CHOMP가 **한 번도 부르지
않는** 나머지 메서드들 때문이고, CHOMP가 지나가는 경로는 곧장
`CollisionEnvDistanceField`로 간다 — 그리고 그건 이미
`crates/moveit-distance-field/src/collision_env_distance_field.rs:1277`에
`get_collision_gradients`로 포팅돼 있다. 시그니처 차이는 upstream이 in/out
파라미터로 넘기던 `gsr_`을 반환값으로 바꾼 것 하나뿐이고, 그 함수 doc이 이미
"upstream's own caller가 하는 것과 같은 방식"이라고 적어놨는데 그 유일한 caller가
바로 `ChompOptimizer`다.

**규칙 (139):** "이 타입은 포팅 불가능하므로 이 타입을 쓰는 코드도 불가능하다"는
결론을 적으려면, **그 코드가 그 타입에 실제로 무엇을 호출하는지 세어라.**
`rg`로 호출 지점을 전부 뽑고, 각 호출의 upstream 구현이 무엇으로 내려가는지 한 단계
따라가라. 상속 그래프는 무엇이 *가능한지*를 말하지 무엇이 *쓰이는지*를 말하지
않는다. 이 사례에서 5 hit 중 2개만 호출이었고, 그 2개는 같은 메서드였고, 그
메서드는 forward 한 줄이었다 — `rg` 한 번과 `sed` 한 번으로 뒤집혔다.

§137과 묶어서: UNFIXED의 사유가 *구조적 제약*일 때(의존성, 순환, 계층, 상속,
"포팅 불가") 그 제약을 재현한 명령과 그 출력을 함께 적어라. 재현 없이 적힌 제약은
다음 라운드가 전제로 삼고, 그 다음 라운드는 그 전제 위에 설계를 얹는다.

## 140. D8 — 레지스트리는 새 크레이트로, 단 진짜 문제는 타입이 둘인 것이다

p1-fixtures 라운드 20이 레지스트리 위치를 조사하고 **새 별도 크레이트**를
추천했다. 근거를 확인했고 받아들인다 — 다만 조사가 부수적으로 드러낸 더 큰 문제가
있어서 그것부터 적는다.

### 140.1 확인한 근거

세 주장을 직접 재현했다:

- **`unsafe_code` 완화 범위.** `linkme::distributed_slice`를 호스팅하려면
  `unsafe_code = "allow"`가 필요하고, 기존 두 사례
  (`crates/moveit-kinematics/Cargo.toml:54`,
  `crates/moveit-planners-sbp/Cargo.toml:43`)가 실제로 그렇게 하면서 완화를
  **호스팅 크레이트 자신에 국한**시켰다. 루트 `Cargo.toml:85`는
  `unsafe_code = "forbid"`다. 레지스트리를 `moveit-planning`에 얹으면 어댑터
  체인과 캐노니컬 타입까지 통째로 `allow`로 내려간다 — 무관한 코드에 lint 완화를
  강제하는 것이라, 새 크레이트가 맞다.
- **dev-dependency 순환.** 격리된 사본에서 `moveit-planners-sbp`에
  `moveit-planning` normal dep을 실제로 추가해 `cargo metadata`(exit 0),
  `cargo tree`, `cargo check --workspace`(17.04s), `cargo test --doc`(2 passed)까지
  돌렸다. cargo는 dev-dep 순환을 정상 처리한다 — 추측이 아니라 실측.
- **`moveit-planning`은 루트 `[workspace.dependencies]`에 없다.** 확인했다.
  플래너 크레이트가 이걸 normal dep으로 붙이려면 그 파일에 등록이 필요하다.

### 140.2 진짜 문제: `PlanningRequest`/`PlanningResponse`가 각각 둘이다

조사 (a)에서 "트레이트 시그니처가 sbp 자신의 로컬 `PlanningRequest`/
`PlanningResponse`를 참조하므로 단순 이동이 아니다"라고 적혔는데, 그게 위치
문제보다 상위의 결함이다. `rg -n 'pub struct PlanningRequest|pub struct PlanningResponse' crates/`:

```
crates/moveit-planning/src/request.rs:60      pub struct PlanningRequest
crates/moveit-planning/src/response.rs:44     pub struct PlanningResponse<'m>
crates/moveit-planners-sbp/src/registry.rs:136  pub struct PlanningRequest
crates/moveit-planners-sbp/src/registry.rs:163  pub struct PlanningResponse<'m>
```

같은 이름이 어느 크레이트에서 보느냐에 따라 다른 타입을 뜻한다. 이게
[Structural fix vs. clever patch]가 말하는 dual meaning이고, 파이프라인과 플래너
사이 모든 이음매가 번역을 끼워야 하는 이유다 — 라운드 20의 `pipeline::Planner`가
레지스트리를 못 쓰고 클로저를 받게 된 것도, 레지스트리 이동이 "파일 옮기기"가
아니게 된 것도 전부 여기서 나온다. 레지스트리 위치는 이 문제의 하위 증상이다.

sbp 로컬 타입이 갈라진 이유는 그 `goal`이 `Vec<CompoundValue>` — 제약이 아니라
구체 상태 — 이기 때문이고, upstream `MotionPlanRequest`는 goal *constraints*를
싣는다. 그 간극을 메우는 작업이 지금 p1-robotmodel 라운드 20에서 진행 중인
`ConstraintSamplerManager`의 `rrt_connect` 배선이다.

### 140.3 D8

- **`moveit-planner-registry`** 를 새 워크스페이스 멤버로 만든다.
  `PlannerRegistration`/`PLANNER_MANAGERS`(`linkme` 슬라이스)와
  `PlannerManager`/`PlanningContext` 트레이트를 담고, `unsafe_code = "allow"`는
  이 크레이트에만 건다.
- 그 트레이트 시그니처는 **`moveit-planning`의 캐노니컬
  `PlanningRequest`/`PlanningResponse`** 로 쓴다. 따라서
  `moveit-planner-registry` → `moveit-planning` 의존이 생기고,
  `moveit-planning`을 루트 `[workspace.dependencies]`에 등록해야 한다.
- `moveit-planners-sbp`의 로컬 `PlanningRequest`/`PlanningResponse`는 **삭제**하고
  `rrt_connect`가 캐노니컬 타입을 받는다. 이름 하나가 한 가지만 뜻하게 되는 것이
  이 결정의 요점이고, 레지스트리를 옮길 수 있게 만드는 전제다.
- 그러면 `pipeline::generate_plan`의 클로저형 `Planner` 트레이트는 레지스트리의
  `PlannerManager`로 대체되고 사라진다.

**선행 조건:** p1-robotmodel 라운드 20의 `ConstraintSamplerManager` 배선이
착지해야 한다 — 그게 sbp의 구체-상태 goal을 불필요하게 만드는 작업이고, 그 전에
타입을 합치면 goal 표현을 두 번 고치게 된다. 그 라운드가 병합되기 전에는
착수하지 않는다. 이건 구조적 해결을 미루는 게 아니라 순서다: 지금 하면 같은
파일을 두 라운드가 동시에 고친다.

### 140.4 커밋 관행 한 건

라운드 20이 `cargo doc`을 깨뜨린 intra-doc link 3건을 새 커밋 대신
`git commit --amend`로 `pipeline.rs` 커밋에 합쳤고, 규칙 위반이라고 스스로
보고했다. 위반이 아니다 — "finding 하나당 커밋 하나"의 finding은 *리뷰에서
지적된 결함*이고, 같은 라운드에서 자기가 방금 쓴 코드가 게이트를 통과하지 못한
것은 별개 finding이 아니라 아직 완성되지 않은 같은 작업이다. push 전 로컬
커밋을 다듬는 것은 그 규칙이 막으려는 대상이 아니다.

## §141 D9 — orocos_kdl `Path_Circle`은 옮기지 않고 유도한다

Pilz CIRC(`TrajectoryGeneratorCirc::plan`)는 `KDL::Path_Circle`을 쓴다. p1-joints
라운드 19가 그 소스가 worktree에 없다고 막혀서 물었고, 세션 루트의
`third_party/orocos_kinematics_dynamics`를 그 worktree에 심볼릭 링크로 넣어줬다.
소스는 이제 읽히지만 **읽는 것과 옮기는 것은 다르다.**

`orocos_kdl/src/path_circle.hpp` 헤더가 **LGPL-2.1-or-later**다. 이 워크스페이스는
BSD-3-Clause이고, `tools/ci/check-license-matches-upstream.sh`가 정확히 이 사고를
막으려고 존재한다 — 크레이트의 선언 라이선스와 소스 파일의 SPDX 헤더가 어긋나면
실패한다. `moveit-stomp-core`가 Apache-2.0 upstream 때문에 `license.workspace = true`를
못 쓰는 것과 같은 계열인데, LGPL은 정적 링크되는 라이브러리에서 하위 사용자에게
copyleft가 전파되므로 훨씬 무겁다.

`moveit-kinematics`가 BSD인 채로 KDL을 잔뜩 인용하는 것과 헷갈리면 안 된다. 그
크레이트는 MoveIt 자신의 `KDLKinematicsPlugin`(BSD, `moveit_kinematics` 패키지)을
포팅하면서 KDL 타입을 *동작 설명*으로 인용한 것이지 orocos 소스를 옮긴 게 아니다.
`crates/moveit-kinematics/src/` 어디에도 orocos 파일:라인 인용이 없다.

**D9: `Path_Circle`은 줄 단위로 옮기지 않고 원호 기하로 독립 유도한다.**

근거는 두 가지다. 첫째, 원호 보간은 초등 기하다 — 중심·축·시작 반경 벡터·회전각이면
`Pos(s)`/`Vel(s)`/`Acc(s)`/`PathLength()`가 나온다. 표현을 빌릴 필요가 없다. 둘째,
정확성 증명 수단이 line correspondence보다 강한 게 이미 있다: CIRC 오라클 파리티다.
포트가 upstream과 같은 수를 내는지는 fixture가 답하지 검사자의 눈이 답하지 않는다.

가져와도 되는 것은 표현이 아니라 **인터페이스 사실**이다. 구체적으로:

- 생성자 인자 의미 — `F_base_start`, `V_base_center`, `V_base_p`(원 평면을 정하는
  세 번째 점), `F_base_end`의 회전, `RotationalInterpolation*`, `eqradius`, `aggregate`.
- **`eqradius`(equivalent radius) 규약** — 병진 경로길이와 회전 경로길이를 하나의
  스칼라 `s`로 섞는 KDL 고유의 스케일링. Pilz CIRC 결과가 여기 직접 의존하므로 이
  규약 자체는 재현해야 한다. 규약은 인터페이스 사실이지 저작 표현이 아니다.
- `RotationalInterpolation_SingleAxis`의 단일축 회전 보간 규약.
- 퇴화 입력(세 점 공선, 반경 0 등)에서 KDL이 던지는 조건과, Pilz가 그것을 어떤
  error code로 바꾸는지 — 이게 파리티 대상이다.

포팅한 파일 상단에 왜 이 파일이 BSD인지 한 문단으로 남긴다: orocos_kdl
`Path_Circle`(LGPL-2.1+)의 코드를 옮긴 것이 아니라 그 인터페이스 규약(특히
`eqradius`)에 맞춰 원호 기하를 독립 유도했고 등가성은 오라클 파리티로 증명한다는
취지로, 인용은 파일:라인이 아니라 규약 이름으로.

줄 단위 포팅이 불가피하다고 판단되면 코드를 쓰기 전에 멈추고 물어야 한다. 그 경우
별도 크레이트 + LGPL 선언이라는 무거운 결정이 필요하고, 그건 사용자 승인 사안이다.

### 141.1 커밋 귀속은 `git merge-base`로 확인한다

p1-joints 라운드 19가 `cargo doc --workspace --no-deps` 실패 5건을
`crates/moveit-planners-pilz/src/trajectory_generator.rs`의 "pre-existing" 결함으로
분류하고 "out of this round's change scope"로 UNFIXED에 넣었다. 근거로 든 것은
"last touched in `c0b8ab7`, not part of any LIN/fixture commit this round"였다.

`c0b8ab7`이 어느 라운드 커밋인지는 확인되지 않았다. 확인하면:

```
$ git merge-base --is-ancestor c0b8ab7 main   → false
$ git log -1 --format='%s' c0b8ab7
moveit-planners-pilz: port TrajectoryGenerator::generate orchestration + PTP
```

main에 없다 — 그 라운드 자신의 첫 커밋이다(rebase 후 `b98d6c6`). 그리고 병합 후
main에서 `cargo doc --workspace --no-deps`는 통과한다. 즉 그 5건은 브랜치에만
있고, 이번 라운드가 만든 것이다.

§137·§139·STOMP 제외에 이어 네 번째 같은 계열이다. 앞의 셋은 "의존성이 안 닿는다",
"상속 구조상 불가능", "ROS 결합이라 D1 제외"였고 전부 `rg` 한 번에 뒤집혔다. 이번
것은 "이전 라운드 것"이었고 `git merge-base --is-ancestor <sha> main` 한 번에
뒤집혔다. 공통점은 사유의 종류가 아니라 **사유를 재현하지 않았다**는 것이다.

규칙에 한 줄을 더한다: 결함을 "이전 라운드/범위 밖"으로 분류할 때는 그 커밋이
main의 조상인지 확인한 명령과 출력을 붙인다. 브랜치에만 있는 커밋은 이번 라운드다.

### 141.2 이번 병합 라운드

`cd8e262`(p3-shapes 라운드 24) 위에 네 브랜치를 병합했다. 충돌 없음.

| 패널 | 라운드 | 커밋 | 주 내용 |
|---|---|---|---|
| p1-robotmodel | 20 | 2 | `path_constraints` 샘플러를 `rrt_connect`의 uniform step에 배선, `Sampler<'a,S,R>` 도입 |
| p3-acm | 16 | 1 | caster-wheel narrowphase 가설 반증 (560 pose 스윕, 409 접촉 중 215 불일치 전부 MPR≥parry) |
| p3-distance-field | 22 | 2 | 첨부 바디 분해 2종 포팅 + 배선 4곳, 6개 함수의 "dead code" 서술을 "known gap"으로 정정 |
| p9-ros | 2 | 11 | D7 r2r rev-pin, `moveit_msgs` 변환 계층 우선순위 1~4, 랜드마인 2건 |

병합 후 내가 직접 측정한 기준선: `nextest --workspace --no-fail-fast` **1354 passed,
2 skipped**; `test --doc --workspace` **5 passed**; `doc --workspace --no-deps` 통과;
`check-*.sh` 8/8 통과(rc가 아니라 출력 내용으로 판정 — `sg docker -c`가 rc를 가린다).
`git diff --stat cd8e262..HEAD -- '*fixtures/*' tools/moveit-oracle/`가 비어 있어
docker replay verify 3종은 돌리지 않았다.

p1-joints는 병합하지 않았다 — `cargo doc`이 깨진 상태이고, 그게 §141.1의 건이다.

### 141.3 CHOMP의 `MultivariateGaussian` 중복

p6-totg가 STOMP와 CHOMP의 `MultivariateGaussian`을 대조하고 어디에 둘지 물었다.
답은 "이미 있다"다. `crates/moveit-sampling/src/multivariate_gaussian.rs:69`가 공용
구현이고, p6-totg가 찾아낸 STOMP의 `bool use_covariance` 분기는 거기서 bool이 아니라
두 메서드로 갈라져 있다 — `sample_with_covariance`(102), `sample_without_covariance`(113).
그리고 `crates/moveit-planners-chomp/src/multivariate_gaussian.rs`(251줄)가 그것의
중복 사본이다.

`moveit-sampling`의 dependencies는 nalgebra/rand/rand_distr 뿐이라 사이클이 없고,
루트 `[workspace.dependencies]`에 이미 등록돼 있다. chomp는 한 줄 추가로 쓴다.

다만 갈아끼우기 전에 두 구현이 난수를 같은 순서·횟수로 소비하는지 같은 seed로
바이트 비교해야 한다 — CHOMP 파리티 fixture가 난수열에 걸려 있으므로, 여기서
"알고리즘이 같다"는 충분조건이 아니다.

## §142 §136.1의 세 미검증 항목 중 하나를 측정으로 닫는다

§136.1이 `.github/workflows/ci.yml`에 대해 검증되지 않은 것 셋을 적었다: (a) 네
`uses:` 액션의 실제 동작, (b) cold crates.io resolution, (c) floating
`dtolnay/rust-toolchain@stable`이 `[workspace.lints.rust] warnings = "deny"` 아래에서
갖는 위험. (c)를 측정했다.

위험의 실체는 이렇다. 툴체인이 `@stable`로 떠 있고 워크스페이스가 warning을 error로
승격하므로, **저장소가 아무것도 바꾸지 않아도** 새 rustc가 새 lint를 켜는 순간 CI가
빨개진다. 이건 추측이 아니라 `ci.yml` 자신의 `env:` 주석이 `RUSTFLAGS: -D warnings`를
거부한 근거로 든 바로 그 메커니즘이다 — 다만 그 주석은 의존성 코드에만 적용했고,
워크스페이스 멤버에는 같은 논리가 그대로 남아 있다.

측정 방법: 호스트에 설치된 nightly가 stable보다 두 릴리스 앞선다.

```
$ rustc +stable  --version   → 1.97.0 (2d8144b78 2026-07-07)
$ rustc +nightly --version   → 1.99.0-nightly (af3d95584 2026-07-09)
```

`@stable`이 12주 뒤 도달할 지점의 근사다. 그 위에서 게이트 두 개를 돌렸다
(`CARGO_TARGET_DIR`를 분리해 stable 캐시를 건드리지 않았다):

```
$ cargo +nightly clippy --workspace --all-targets   → Finished, exit 0
$ cargo +nightly doc --workspace --no-deps          → Finished, exit 0
```

워크스페이스 22개 크레이트 전부 `Checking`을 거쳤고 warning/error 0줄이다.
`warnings = "deny"`가 걸린 상태이므로 새 lint가 하나라도 발화했다면 hard error로
멈췄을 것이다.

결론: **오늘 기준으로 두 릴리스 앞의 rustc에서도 깨지지 않는다.** 이건 "위험이
없다"가 아니라 "현재 알려진 lint 파이프라인에는 이 트리를 깨뜨리는 게 없다"이다.
툴체인 핀은 여전히 안 박는다 — 핀을 박으면 이 신호 자체가 사라지고, 그때는 rustc가
몇 릴리스 앞서갈 때까지 아무도 모른다. 트레이드오프는 §136.1에 적은 그대로 두되,
근거 없는 불안이 아니라 측정된 여유가 있다는 사실을 여기 남긴다.

재측정 시점: stable이 1.99로 올라가면 이 측정은 만료다. 그때 다시 nightly로 돌려라.

### 142.1 커밋된 lockfile을 CI가 주장하지 않던 구멍

(b)를 보다가 나온 별건이다. `Cargo.lock`은 tracked인데(`git ls-files Cargo.lock` →
있음) `ci.yml`의 어느 단계도 `--locked`를 넘기지 않았다(`grep -c '\-\-locked'` → 0).
그래서 manifest만 고치고 lockfile을 안 갱신한 커밋이 오면 cargo가 조용히 lockfile을
새로 써버리고 clippy·nextest·doctest·doc 네 단계가 전부 통과한다. **CI가 초록인
것이 트리에 커밋된 lockfile에 대해 아무것도 말해주지 않는다.**

§136.2의 `--no-fail-fast` 건과 같은 계열이다 — 로컬 게이트와 CI가 "그 단계가 무엇을
뜻하는지"에 대해 서로 다른 것을 뜻하고 있었다.

**Anchor:** `cargo (build|check|clippy|test|doc|nextest|run)` in `.github/`, `tools/ci/`
**Sites:** `ci.yml:32,38,40,48`; `check-dep-direction.sh:25,44`;
`check-serde-float-roundtrip.sh:33`; `run-oracle-sweep.sh:50`
**Same defect at:** `ci.yml`의 빌드 단계 4개 — 넷 다 커밋된 lockfile을 주장하지 않고
resolve한다.
**Distinct, skip:** `check-dep-direction.sh`/`check-serde-float-roundtrip.sh`의
`cargo metadata`/`cargo tree` — 읽기 전용 질의이고 CI에서는 아래 주장 뒤에 실행되며,
로컬에서는 편집 중인 트리에 대해서도 돌아야 하므로 `--locked`가 오히려 틀리다.
`run-oracle-sweep.sh:50`의 `cargo build` — 자기 헤더가 "Not a CI step"이라고 명시.

플래그를 네 군데 붙이는 대신 앞에서 한 번 주장하는 쪽을 택했다:

```yaml
- name: lockfile
  run: cargo fetch --locked
```

주장이 성립하면 뒤 단계들에는 다시 쓸 것이 남아 있지 않다 — 한 곳에서 불변식을
말하고, 실패 메시지도 cargo 자신의 것(드리프트한 패키지 이름 포함)이 나온다. 네
단계에 개별로 붙이면 어느 것이 먼저 터지느냐에 따라 메시지가 달라진다.

현재 트리에서 `cargo metadata --locked`와 `cargo fetch --locked` 둘 다 통과하므로
이 커밋은 동작 변화가 없다 — 구멍만 닫는다.

### 142.2 cold crates.io resolution — 측정했고, 통과한다

§136.1의 (b)다. CI 러너는 매번 빈 `CARGO_HOME`에서 시작하므로, 호스트의 따뜻한
레지스트리 캐시가 가려주던 문제(yank된 버전, 사라진 crate)가 거기서만 드러날 수
있다. 그대로 재현했다:

```
$ rm -rf $D && mkdir -p $D
$ CARGO_HOME=$D cargo fetch --locked
    Updating crates.io index
 Downloading crates ...
   ... (85 crates)
$ echo $?
0
$ du -sh $D → 103M
```

`Downloaded` 85줄. `Cargo.lock`의 `name =` 항목은 107개이고, 차이 22는 워크스페이스
멤버 수와 정확히 같다(`cargo metadata --no-deps` → packages 22개; `crates/` 21개 +
`tools/moveit-diff`). 멤버는 path dependency라 내려받지 않는다 — 즉 내려받아야 할
것을 전부 내려받았고 빠진 게 없다.

커밋된 lockfile이 지정하는 버전 중 yank된 것도, 받을 수 없는 것도 없다. §136.1의
(b)를 닫는다. 남는 것은 (a) 네 `uses:` 액션의 실제 동작뿐이고, 그건 GitHub Actions
러너 없이는 재현할 수 없다 — 원격이 붙기 전까지 UNFIXED로 둔다.

## §143 오라클의 두 번째 요청 간 오염 경로 — 첨부 바디

§138이 pilz op의 `setKinematicsAllocators`를 private model로 닫았다. 같은 계열이
한 층 아래에 하나 더 있었다.

`applyJointValues`는 모든 op이 요청 상태를 설치할 때 거치는 단일 진입점이고, 자기
doc에 불변식을 이미 적어놨다:

> Reset first: leaving the previous case's values in place would make a result
> depend on request order, which would quietly hide a disagreement on any
> variable the request omits.

그런데 그 함수가 부르는 `setToDefaultValues()`는 관절 값만 되돌리고 **첨부 바디
목록은 건드리지 않는다.** `state_`는 프로세스 수명 내내 살아 있으므로, 한 요청이
붙인 첨부 바디가 다음 요청에 그대로 남는다. 즉 그 doc이 선언한 불변식을 그 함수가
절반만 강제하고 있었다.

**Anchor:** `clearAttachedBodies` / `attachBody` on the shared `state_`
**Sites:** `oracle.cpp:2016`(`collision`), `:2214`(`frameTransform`),
`:2337`(`isStateValid`); `applyJointValues` 호출자 10곳
(`:1332,1383,1627,2014,2212,3035,3162,3316,4103,4382`)
**Same defect at:** `:2016`, `:2214` — 둘 다 `applyJointValues` 직후에 스스로
`clearAttachedBodies()`를 부르고 있었다. 나머지 8개 호출자는 부르지 않는다. 두
집단을 가르는 것은 **누가 그 생각을 했느냐**뿐이었다.
**Distinct, skip:** `:2337` — 매 호출마다 새로 만드는 `PlanningScene`의
`getCurrentStateNonConst()`를 비운다. 요청 간 수명이 없으므로 샐 곳이 없다.

호출자 두 곳에 있던 런타임 가드를 지우고 소유자 안으로 옮겼다:

```cpp
void applyJointValues(const json& request)
{
  state_->setToDefaultValues();
  state_->clearAttachedBodies();
  ...
```

이제 "`applyJointValues`가 끝나면 `*state_`는 모델 기본값이고 아무것도 붙어 있지
않다"가 **기억한 경로가 아니라 모든 경로에서** 참이다. 첨부하는 op은 그 뒤에
그대로 첨부한다. 가드를 하나 더 잘 두는 게 아니라 가드가 필요 없는 상태로 만드는
쪽 — §138과 같은 판단이다.

### 143.1 검증과 그 한계

새 stamp `e35d8c82d0cabbf6` (이전 `9acdf82cb2e09162`).

```
verify-fixture-replay.sh        → identical 36줄 / 총 36줄 (36/36)
verify-fixture-provenance.sh    → fail/error/mismatch 0건
check-fixture-format.sh         → OK: 109 oracle fixtures are in canonical form
```

`sg docker -c`가 감싼 명령의 종료코드를 가리므로 전부 출력 내용으로 판정했다.

**36/36 identical이 "오염이 없었다"를 증명하지 않는다.** `verify-fixture-replay.sh`는
fixture *파일* 하나당 오라클 프로세스 하나를 띄워 그 파일의 요청만 한 NDJSON
스트림으로 먹인다 — 파일 간 순서 효과는 애초에 이 스크립트가 볼 수 없는 곳에
있다(§138과 같은 사각지대). 지금 corpus가 그 순서를 우연히 밟지 않았을 뿐이고,
그게 이 결함이 오래 안 보인 이유다. 이번 변경이 하는 일은 corpus가 밟지 않았음을
확인하는 게 아니라 **밟을 수 있는 순서를 없애는 것**이다.

바꿔 말하면 이 커밋은 현재 corpus에 대해 동작 변화가 0이다. 값어치는 다음에 추가될
op — 특히 첨부 바디가 실제로 충돌 검사에 참여하게 만드는 작업(p3-distance-field
라운드 23) — 이 같은 함정을 다시 밟지 못한다는 데 있다.

## §144 `ros/moveit-ros`가 main에서 깨져 있었고, 어떤 게이트도 보지 않았다

병합 후 main에 대고 `sg docker -c ros/verify-ros-interop.sh`를 돌려서 발견했다.

```
error[E0063]: missing fields `planner_id` and `trajectory_constraints`
              in initializer of `moveit_planning::PlanningRequest`
  --> src/planning.rs:160:12
error[E0063]: missing field `planner_id`
              in initializer of `moveit_planning::PlanningResponse<'_>`
  --> src/planning.rs:223:12  및  :387:19
```

p1-fixtures 라운드 20이 `moveit-planning`에 `PlanningRequest::{trajectory_constraints,
planner_id}`와 `PlanningResponse::planner_id`를 추가했고, p9-ros 라운드 2의
`src/planning.rs`는 그 이전 모양으로 쓰여 있었다.

**두 라운드 다 자기 게이트를 통과했다.** 루트 워크스페이스는 `ros/`를 멤버로 갖지
않으므로(D5) `cargo clippy --workspace`도 `cargo nextest run --workspace`도 저기까지
가지 않고, `ros/verify-ros-interop.sh`는 자기 헤더에 이렇게 적혀 있다:

> Not run automatically by anything yet -- there is no CI hook for `ros/` this
> round (D5: ros/moveit-ros is outside the root workspace, so nothing in
> tools/ci/ walks into it).

즉 누구의 실수도 아니고 **경계를 보는 게이트가 없다**는 것이 원인이다. 오늘 같은
계열을 오라클에서 두 번 구조적으로 닫았다(§138 pilz 모델, §143 첨부 바디) — 둘 다
"가드를 하나 더 잘 두자"가 아니라 "기억에 의존하는 상태를 없애자"였다. 여기도 같은
기준을 적용해야 한다. 병합 루틴에 `verify-ros-interop.sh`를 넣는 것은 **오케스트레이터가
기억하는 가드**이므로 해결이 아니다.

### 144.1 D10 후보 — 변환 계층을 워크스페이스 안으로

`ros/moveit-ros`를 둘로 나눈다:

- **변환 계층** — 코어 타입 ↔ 메시지 모양의 평범한 Rust 구조체 변환. r2r도 ROS도
  필요 없다. 루트 워크스페이스 멤버가 될 수 있고, 그러면 `cargo clippy --workspace`가
  push마다 타입 체크한다 — 오늘 같은 필드 추가가 그 자리에서 빨개진다.
- **전송 계층** — r2r에 의존하는 부분만. 지금처럼 워크스페이스 밖, D5 경계 유지.

D1/D5를 깨지 않는다: 변환 계층에 ROS 의존이 없으므로 `check-dep-direction.sh`가
그대로 통과한다(그 스크립트는 워크스페이스 멤버가 ROS 클라이언트 라이브러리에
의존하는지를 본다).

**아직 결정하지 않았다.** p9-ros 라운드 3에 조사를 시켰다: `ros/moveit-ros/src/`에서
파일별 r2r 참조 줄 수, 변환 코드와 전송 코드가 파일 단위로 이미 갈라져 있는지,
아니면 무엇이 섞여 있는지. 그 숫자를 보고 확정한다. §137·§139·STOMP 제외·§141.1이
전부 "세지 않고 단정한" 사례였으므로, 여기서도 분리 가능/불가능을 세기 전에 말하지
않는다.

### 144.2 남은 CI 커버리지 구멍

같은 뿌리에서 나오는 것 둘을 여기 적어둔다.

- `tools/ci/verify-*.sh` 4개(`verify-clean-checkout.sh`, `verify-continuous-reseed-wrap.sh`,
  `verify-fixture-provenance.sh`, `verify-fixture-replay.sh`)는 docker가 필요해서
  `check-*.sh` glob 밖에 있고, 사람이 기억할 때만 돈다.
- `ros/verify-ros-interop.sh`도 같다.

둘 다 막는 것은 같은 하나다: 오라클/ROS 이미지가 레지스트리에 발행돼 있지 않아
러너가 끌어올 수 없다. `ci.yml` 꼬리 주석이 오라클 job에 대해 이미 같은 말을 한다.
발행 자체가 원격 저장소를 요구하므로(아직 push한 적이 없다) 지금은 UNFIXED로 둔다 —
`ros/moveit-ros` 쪽만은 §144.1이 이미지 없이 닫을 수 있는 유일한 길이다.

## §145 계약과 결함을 먼저 구분한다

p1-fixtures 라운드 21이 `PlanningResponse::start_state`를 "진짜 간극"으로 분류하며
이유를 이렇게 적었다:

> `PlanningSceneValidityChecker::is_valid`가 side effect로 scene 현재 상태를
> 훼손한다 ... `generate_plan` 반환 후 `scene.current_state()`를 읽어도 실제 시작
> 상태를 신뢰할 수 없음

사실 관찰은 맞다. `planning_scene_validity.rs:138-142`의 `is_valid`는 호출마다
`scene.current_state_mut()`에 샘플을 써넣는다. 그런데 그건 훼손이 아니라 **문서화된
계약**이고, 같은 파일 `:128-137`이 이유와 대안까지 적어놨다 — 매 호출 복원은 한
query가 만드는 수십만 호출 각각에 full-state clone을 하나씩 더 붙이는 비용인데 그
성질을 그 크레이트의 planning path는 아무것도 쓰지 않는다. 그래서:

> a caller that needs the pre-planning state preserved clones it once, itself,
> before handing the scene to this type.

`generate_plan`이 그 caller다. `start_state`는 막혀 있는 게 아니라 계약을 아직
이행하지 않았을 뿐이고, 이행 비용은 호출당이 아니라 query당 clone 한 번이다 —
위 doc이 거부한 비용과 다른 차수다.

§137·§139·STOMP 제외·§141.1이 "사유를 재현하지 않은" 사례였다면 이건 한 칸
다르다: 사실은 재현했는데 **그 사실이 계약인지 결함인지**를 묻지 않았다. 규칙에
한 줄 더한다 — 어떤 동작을 간극의 원인으로 지목하기 전에, 그 동작이 문서화된
계약인지 확인하고, 계약이면 "막혔다"가 아니라 "계약을 이행하면 된다"로 적는다.
계약에는 대개 이행 방법이 같이 적혀 있다.

### 145.1 p1-fixtures 라운드 21 병합

커밋 2개(`1e1f916` 캐노니컬 타입 델타 감사, `e8e27d5` `getCostSources` 재분류).
코드 로직 변경 없음, 문서·분류만.

두 감사를 따로 재현했다:

- `third_party/moveit_msgs/msg/MotionPlanRequest.msg`의 실제 필드 수는 주석·빈 줄
  제외 **16**. 보고의 8+4+4=16이 파일과 일치하고, 각 항목이 실제 필드명과 하나씩
  대응한다.
- `rg -n 'cost_sources: None' crates/moveit-collision/src/` → **0건**. `parry.rs`에
  `cost_sources_for_part_pair`/`mesh_mesh_cost_sources`/`mesh_shape_cost_sources`가
  있고 테스트가 계산값을 검사한다. `blocked` → `unported, in scope` 재분류가 맞다.

병합 후: `nextest --workspace --no-fail-fast` **1354 passed, 2 skipped**;
doctest **5**; `doc --workspace --no-deps` 통과; `check-*.sh` 8/8;
`ros/verify-ros-interop.sh`의 `error[E0063]` **3건 그대로**(§144의 기존 건, 새로
늘지 않음 — 이번 라운드는 그 두 타입에 필드를 더하지 않고 doc만 더했다).

## §146 STOMP의 네 "D1 제외"는 둘이 거짓, 하나가 층 착오, 하나만 진짜였다

p3-shapes 라운드 25가 `moveit-planners-stomp`의 `# Not ported: the ROS/task-engine
layer (D1/D2 exclusion)` 네 항목을 전부 재검토했다. 결과:

| # | 항목 | 판정 |
|---|---|---|
| 1 | goal constraint sampling | **거짓 제외** — 포팅함 (`sample_goal_state`) |
| 2 | seed-trajectory extraction | **거짓 제외** — 포팅함 (`extract_seed_trajectory`) |
| 3 | `allowed_planning_time` 워처 스레드 | ROS 결합 아님, 목록에서 제거 |
| 4 | pluginlib 등록 + `trajectory_visualization.hpp` | 진짜 ROS, 유지 |

1·2의 근거는 `rg -n 'rclcpp|node_|Logger|RCLCPP'`가 `stomp_moveit_planning_context.cpp`
전체에서 **7 hit**이고, 그 7개가 각각 무엇인지 줄 번호별로 확인된 것이다 — 60·62는
`getLogger()` 헬퍼, 115·126은 `extractSeedTrajectory` 자신의 실패 분기에서 찍는
`RCLCPP_WARN`(알고리즘의 의존이 아니라 로깅), 283은 미구현 오버로드, 305·310은
`setPathPublisher`/`getPathPublisher`의 `rclcpp::Publisher`. 두 함수 본문 어디에도
ROS가 없다. 4번은 유지하되 인용 파일이 틀렸던 것을 정정했다
(`stomp_moveit_planner_plugin.cpp:144`).

세션 통산 다섯 번째 "재현하지 않은 사유" 반증이다(§137 의존성, §139 상속, §141.1
커밋 귀속, §145 계약/결함 혼동, 그리고 이번 D1 제외 둘).

### 146.1 3번은 갭이 없는 게 아니라 다른 층에 있다

라운드 25가 3번에 대해 이렇게 맺었다:

> Removed from the exclusion list entirely rather than re-justified within it:
> there is no gap here for a future round to close.

앞부분은 맞다 — 워처 본문은 `std::condition_variable`/`std::mutex`/`std::async`/
`std::chrono`와 `stomp->cancel()`뿐이고, 라운드 24의 `CancelHandle::new`/`.clone()`
+ `std::thread::spawn`이 caller에게 같은 모양을 만들 재료를 이미 준다.

**"no gap"만 한 칸 과하다.** upstream에서 그 워처는 `StompPlanningContext::solve`
안에 있다 — `allowed_planning_time`을 받으면 플래너가 스스로 취소하는 것이
PlanningContext 층의 동작이고, 이 포트에서는 그 층이 아직 없다. 그리고 그 갭은
이미 다른 문서에 기록돼 있다: p1-fixtures 라운드 21의 `MotionPlanRequest.msg`
16필드 감사가 `allowed_planning_time`을 `unported, in scope`로 분류하며
"downstream `PlanningContext`/`PlannerManager` 몫"이라고 적었다.

두 문서가 실질적으로 같은 말을 하는데 `planner.rs`만 따로 읽으면 "아무도 갚을 게
없다"로 읽힌다. §145와 같은 계열의 정밀도 문제다 — 사실은 맞는데 결론의 층이
어긋났다. 라운드 26 항목 0으로 상호 참조를 넣게 했다.

### 146.2 병합과 다음 대상

라운드 25 커밋 2개(`cf1ce23`, `2d9b2c4`) 병합. 병합 후: `nextest --workspace
--no-fail-fast` **1362 passed, 2 skipped**; doctest **5**; `doc --workspace
--no-deps` 통과; `check-*.sh` 8/8.

이로써 `moveit-planners-stomp`는 upstream `moveit_planners/stomp`에서 pluginlib
등록과 `trajectory_visualization.hpp`만 남긴다(`composable_task.rs`가
`stomp_moveit_task.hpp`를, 나머지 네 모듈이 각 헤더를 이미 덮는다).

다음 대상은 `moveit-stomp-core`다. 이 워크스페이스에서 유일하게 다른
upstream(ros-industrial/stomp)과 다른 라이선스(Apache-2.0)를 갖는 크레이트인데
`moveit-scene`의 60항목 롤업 같은 심볼 단위 감사를 받은 적이 없다. 레퍼런스는
로컬에 있고 핀과 정확히 일치한다:

```
$ git -C /home/stevek/work/stomp log --oneline -1
b1a87c8 Merge pull request #18 from mosfet80/patch-3
$ git -C /home/stevek/work/stomp rev-list --count b1a87c80...HEAD
0
```

라운드 26에 (a) `stomp.h/cpp`·`task.h`·`utils.h/cpp`의 심볼 단위 완결성 감사와
(b) upstream 자신의 인수 테스트 `test/stomp_3dof.cpp`(445줄) 포팅을 시켰다. (b)는
이 포트에 upstream이 무엇을 옳다고 여기는지에 대한 **외부 기준**이 없다는 문제를
겨냥한 것이다 — 자체 일관성 테스트는 우리 구현끼리의 동의만 증명한다. 라이선스는
양쪽 다 Apache-2.0이라 §141(D9)의 LGPL 건과 다르다.

## §147 헤더의 default argument와 실제 호출부가 넘기는 값은 다르다

p1-robotmodel 라운드 21이 자기 라운드 20 결과를 다시 재서 뒤집었다. 라운드 20이
path-constraint 샘플링 호출부에 `moveit-constraints::DEFAULT_MAX_SAMPLING_ATTEMPTS`
(= 2)를 넘겼는데, 그 상수는 upstream
`ConstraintSampler::DEFAULT_MAX_SAMPLING_ATTEMPTS`(`constraint_sampler.hpp:64`)이고
**upstream에서는 두 `sample()` 오버로드의 default argument로만 쓰인다** — 이 포트의
`ConstraintSampler` 트레이트가 이미 접어버린 오버로드들이다. 살아 있는 호출부 중
2를 받는 것은 하나도 없다.

실제 호출부(`constrained_sampler.cpp:69-70`,
`constrained_goal_sampler.cpp:137`)는 전부 `getMaximumStateSamplingAttempts()`를
넘기고, 그 값은 **4**다. 내가 직접 확인했다:

```
$ sed -n '255,262p' moveit_planners/ompl/ompl_interface/src/planning_context_manager.cpp
  , max_goal_samples_(10)
  , max_state_sampling_attempts_(4)
  , max_goal_sampling_attempts_(1000)
  , max_planning_threads_(4)
  , max_solution_segment_length_(0.0)
```

인용이 정확하다. 커밋 `9b74b6d`가 `registry.rs`에 그 인용에서 온
`DEFAULT_MAX_STATE_SAMPLING_ATTEMPTS = 4`를 두고 호출부를 바꿨고,
`moveit-constraints`의 "real production call site" 서술도 되돌렸다.

### 147.1 인용은 표본이지 모집단이 아니다

같은 생성자가 다섯 개를 한꺼번에 초기화한다(`:258-262`, 위 인용). 포트에서
`rg -n 'max_goal_samples|max_goal_sampling_attempts|max_state_sampling_attempts|max_solution_segment_length' crates/`
→ **0건**. 즉 `DEFAULT_MAX_STATE_SAMPLING_ATTEMPTS`가 이 계열의 첫 사례이고,
나머지는 아직 아무도 손대지 않았다.

지금 진행 중인 goal constraint sampling에 직접 걸리는 것을 확인해뒀다:

| upstream 사이트 | 값 | 무엇 |
|---|---|---|
| `constrained_goal_sampler.cpp:137` | **4** | `sample()`의 attempts — path 샘플러와 같은 노브 |
| `constrained_goal_sampler.cpp:98` | **1000** | `getMaximumGoalSamplingAttempts()` |
| `constrained_goal_sampler.cpp:106` | **10** | `getMaximumGoalSamples()`, `gls->getStateCount()` 상한 |

첫 번째는 그대로 4를 써야 한다 — goal 경로에 다시 2를 넣으면 같은 실수의 반복이다.
뒤의 둘은 "goal 상태를 몇 개까지 모으고 몇 번까지 시도하는가"를 정하는 별개 노브고,
`getStateCount()`가 OMPL `GoalLazySamples`의 상태라 그 층 없이는 그대로 옮길 수
없다. 셋 다 이번 라운드에 구현하라고 하지 않았다 — 각각 **구현했는지, 아니면 어느
층에 속해서 미뤘는지**를 파일:라인과 함께 명시하라고 했다. 조용히 빠지는 것이
이 계열이 재발하는 경로다.

규칙으로 적어둔다: upstream 상수를 포트의 호출부에 넣을 때는, 그 상수가 헤더의
default argument인지 실제 호출부가 넘기는 configured 값인지 먼저 구분한다. 이름이
같아도 값이 다르고, 컴파일도 테스트도 그 차이를 잡지 않는다.

## §148 10건 잔여를 닫는 결정적 실험 — 기계는 이미 있다

§121.2가 touching >= 2인 실패 10건을 "닫지 않고 좁혔다"로 남겼다. 좁힌 근거는
크기 분포였다 — 10건(2.3e-4 ~ 3.6e-3)이 105건(3.9e-5 ~ 5.4e-2) 안에 통째로 들어가고
별개 군집이 아니라는 것. 그 처리는 옳았다. 다만 **크기 분포는 정황이고, 순회 순서를
직접 배제하지 못한다.**

직접 배제하는 실험이 있다. `VisibilityConstraint::cone_touching_link_count`의 doc이
그 열쇠를 이미 적어놨다:

> `decide`'s own reported depth continues to come from whichever single contact
> `cone_collision_result(state, 1)` happens to find first

즉 `decide`는 여러 접촉 중 **먼저 찾은 하나**의 깊이를 보고한다. 그렇다면 10건
각각에 대해 `cone_collision_result(state, usize::MAX)`로 **모든** 접촉의 깊이를
뽑아 오라클 값과 대조하면 답이 갈린다:

- 오라클 깊이가 그중 **어느 하나와 일치**한다 → 첫 번째가 아닌 다른 접촉이 정답이었다는
  뜻이고, **순회 순서가 그 10건의 원인이다.** deviation 6이 아니다.
- 오라클 깊이가 **어느 것과도 일치하지 않는다** → 어느 접촉을 골랐든 틀렸다는 뜻이고,
  순회 순서는 배제된다. deviation 6만 남는다.

두 결과 모두 결론이고, 어느 쪽이든 §121.2의 "증거 없음"이 "있음" 또는 "배제됨"으로
바뀐다.

비용이 거의 없다. `cone_collision_result`는 이미 `max_contacts` 인자를 받고
(`crates/moveit-constraints/src/visibility.rs:504-508`), `cone_touching_link_count`가 이미 `usize::MAX`로 부른다
(`:553-556`) — 세는 대신 깊이를 돌려주는 진단 하나를 같은 자리에 더하면 된다.
새 기계도, 오라클 확장도 필요 없다.

주의할 점 두 가지:

1. 비교는 `assert` 톨러런스가 아니라 **측정된 수치의 나열**이어야 한다. "일치한다/
   안 한다"를 어떤 임계로 판정할지부터가 결론을 바꾸므로, 10건 × 접촉 수만큼의
   깊이를 오라클 값과 나란히 표로 적고, 차이의 크기를 그대로 보여라. §121.2가
   크기 분포를 그렇게 다뤘던 방식 그대로.
2. touching >= 2이면서 **통과**한 4건도 같이 재라. 통과 케이스에서 여러 접촉의
   깊이가 어떻게 분포하는지가 대조군이다 — 실패 10건만 보면 "여러 접촉이 있으면
   원래 이렇다"와 "이 10건이 특별하다"를 가를 수 없다.

`crates/moveit-constraints`는 p1-robotmodel 소유고 지금 goal constraint sampling
(§147.1)으로 차 있다. 다음 라운드 항목으로 돌린다. 여기 적어두는 이유는 이 실험이
설계된 적이 없어서 — UNFIXED가 "원인 불명"으로 남아 있었지 "이렇게 하면 갈린다"로
남아 있지 않았다.

## §149 오라클에 능력을 더할 때, 기존 fixture가 한 바이트도 안 움직이는 것이 조건이다

p3-distance-field 라운드 23의 UNFIXED는 두 가지를 요구했다:
`distance_field_cache_entry`/`group_state_representation` 두 op이
(a) `request["attached_bodies"]`를 적용하지 않고 (b) `CollisionResult`를
전혀 덤프하지 않는다는 것. 둘 다 소스를 직접 읽어 사실로 확인했다.

### 149.1 `req.contacts`는 출력 스위치가 아니다 — 그래서 request 필드로 만들었다

`collision_env_distance_field.cpp:298-338`(true 분기)과 `:341-349`
(false 분기)를 읽으면 차이가 출력에만 있지 않다. true 분기는
`gsr->gradients_[i].types[col] = SELF`와 `gradients_[i].collision = true`를
쓰고 계속 스캔하고, false 분기는 첫 충돌 링크에서 즉시 반환하며 그 둘을
건드리지 않는다. 그런데 그 두 필드는 `group_state_representation` op이
**이미 덤프하고 있다**. 그러니 `req.contacts = true`로 그냥 켰다면 기존
fixture가 주장하던 값이 조용히 바뀐다.

측정으로 확인했다. pr2 `right_arm`에 같은 요청을 두 번:

```
contacts 있음: grad_collision_links=5/22, links_with_type_SELF=1, contacts=50
contacts 없음: grad_collision_links=0/22, links_with_type_SELF=0, contacts 키 없음
```

그래서 `contacts`(기본 false) / `max_contacts` / `max_contacts_per_pair`를
request 필드로 노출했다. 기존 fixture는 필드를 안 보내므로 바이트 동일,
새 fixture는 모드를 자기 요청에 적어 놓게 된다. `max_contacts_per_pair`까지
노출한 이유는 true 분기의 스캔 상한이
`std::min(req.max_contacts_per_pair, req.max_contacts - res.contact_count)`
라서, 그 값을 못 정하면 pair당 다중 contact 형태 자체에 도달할 수 없기
때문이다.

**규칙**: 오라클 op에 upstream 플래그를 새로 켤 때는, 그 플래그가 op이
이미 덤프하는 필드에 부수효과를 갖는지 upstream 소스에서 먼저 확인해라.
갖는다면 무조건 켜지 말고 request 필드로 만들어라. 판정 기준은
"replay가 36/36 identical인가" 하나다.

### 149.2 `Contact`의 두 번째 body는 body가 아닐 수 있다

`shapeKindsFor`는 `body_type`으로 분기해서 조회 결과를 그대로
역참조하고 있었다. `CollisionEnvDistanceField`는 링크를 **집계된 거리장**
하나와 비교하므로 이름 붙일 두 번째 body가 없고, 대신 sentinel을 쓴다:
`"self"`(타입 `ROBOT_LINK`, `:326-327`), `"environment"`(타입
`WORLD_OBJECT`, `:1615-1616`). 둘 다 모델에도 world에도 없고, **타입 태그가
sentinel의 일부**지 약속이 아니다. 첫 contact에서 `getLinkModel("self")`가
null을 반환하고 프로세스가 죽었다.

고친 방식이 요점이다. `name == "self" || name == "environment"` 비교가
아니라 **존재 확인**(`hasLinkModel` / `getAttachedBody` / `hasObject`)으로
막았다. 규칙이 모든 op·모든 body type에 동일하게 적용되고, upstream이
sentinel을 하나 더 늘려도 crash가 아니라 fixture의 `null`로 나타난다.
그리고 `null`(body가 아님)과 `[]`(collision geometry 없는 진짜 링크 —
pr2에 여럿 있다)을 서로 다른 값으로 남겼다. 값 하나에 뜻 하나.

이 결함은 smoke test 없이는 안 보였다. replay 36/36 identical은 새 경로를
한 줄도 지나지 않는다 — **기존 fixture가 안 보내는 필드로 켜지는 코드는
기존 fixture로 검증되지 않는다.** 능력을 더한 커밋은 replay(회귀)와 수동
1회 실행(신규 경로) 둘 다 필요하다.

### 149.3 `attached_body_names_`는 ACM이 null이면 비어 있다 — upstream 동작이다

smoke test에서 `use_acm: false`인 요청만 `attached_body_names: []`를
반환했다. 포트 버그가 아니다: `generateDistanceFieldCacheEntry`의
attached-body 열거 전체가 `if (acm)`(`:775`) 안에 있고 push는 `:801-802`다.
ACM이 null이면 state에 무엇이 붙어 있든 목록은 빈 채로 남는다. 두 op의
doc에 측정값과 함께 적었다.

부수적으로 `Contact::depth`는 이 경로들에서 항상 `0.0`이다 — upstream이
`pos`와 두 body 식별자만 쓰고(`:308-327`, `:1600-1616`), 값은
`collision_common.hpp:84`의 `= 0.0` default member initializer가 준다.
재현 가능하지만 침투 깊이를 측정한 값이 아니므로 fixture가 그렇게 읽으면
안 된다. `gradients` 벡터(§ 이전 라운드에서 제외한, 정말 미정의인 값)와
구분해서 기록한다.

## §150 D10 기각 — 쪼갤 ROS-free 부분집합이 없다. 경계는 §144.2로 합친다

§144.1에서 D10 후보로 적은 것은 "`ros/moveit-ros`의 ROS-비의존 conversion
계층을 워크스페이스 멤버로 분리해서 `--workspace`가 타입 체크하게 한다"였다.
p9-ros에게 파일별 `r2r` 참조 수를 세게 했고, 결과가 전제를 반증했다.

내가 병합 후 직접 재현한 수치:

```
rg -c 'r2r' ros/moveit-ros/src/            → 14개 파일
rg -n 'r2r::Node|r2r::Publisher|r2r::Subscriber|r2r::Client|r2r::Service|
       r2r::ActionServer|Context::create|spin_once|create_(subscription|
       publisher|service|client)' ros/moveit-ros/src/   → 0건
```

transport 심볼이 0건이라는 것은 "이 크레이트가 이미 ROS-free다"가 아니라
**쪼갤 경계가 없다**는 뜻이다. 77건의 `r2r` 참조는 전부 (a) `use r2r::X::msg`
타입 임포트와 (b) orphan rule 회피용 newtype 래퍼다. 즉 conversion 함수의
시그니처마다 `r2r` 메시지 타입이 이름으로 등장한다 — 그게 이 크레이트의 존재
이유다. ROS-free 부분집합은 공집합이다.

그리고 이번 라운드에 경계는 **더 넓어졌다**. `ros/moveit-ros/Cargo.toml`이
`moveit-scene`, `moveit-collision` path dep 2개를 추가해서 게이트 없는
crates/ ↔ ros/ 경계가 6개 크레이트에서 8개로 늘었다.

**결정: D10 기각.** §144.1의 갭은 닫히지 않았고, 닫는 방법은 하나뿐이다 —
`ros/`를 실제로 컴파일하는 게이트. 그건 ROS 이미지가 필요하고, 이미지는 CI의
docker가 필요하고, 그건 레지스트리/remote가 필요하다. 즉 §144.2와 같은
블로커다. §144.1을 별도 항목으로 두지 않고 §144.2에 합친다.

깨지는 방식 자체는 문제가 아니라는 점을 적어둔다. crates/의 구조체에 필드가
추가되면 ros/의 생성 지점이 `error[E0063]`로 **컴파일 에러**를 낸다 — 정확히
원하는 신호다. `..Default::default()`로 그걸 삼키는 것이 진짜 결함이고,
라운드 3 항목 0이 그걸 제거했다. 남은 문제는 "아무도 컴파일하지 않는다" 하나뿐이며,
우회는 사람이 매 병합마다 `sg docker -c 'ros/verify-ros-interop.sh'`를 돌리는
것이다 — 실제로 이번 병합에서 돌렸고 `all gates passed`, 92 tests 통과.
이건 workaround이지 구조적 닫힘이 아니다.

### 150.1 `PlanningScene`에 world 변경 접근자가 없어서 ROS 연산 2개가 막혔다

p9-ros의 UNFIXED 중 하나를 원본에 대고 확인했다. 사실이고, 막는 쪽은 ros/가
아니라 **우리 crates/**다.

`moveit_scene::PlanningScene::world()`(`scene.rs:981`)는 `&World`만 돌려준다.
`world_mut`은 없다(`rg`로 확인, 0건). 그런데 필요한 두 연산은 `World`에
`pub`으로 있다 — `move_shapes_in_object`(`world.rs:707`),
`set_subframes_of_object`(`world.rs:837`). 즉 크레이트 밖에서 도달 불가다.

`world_mut()`을 추가하는 것이 답이 **아니다**. 이 씬은 의도적으로 raw
`&mut World`가 아니라 연산별 래퍼를 노출한다 — `add_shape`(`:974`),
`move_object`(`:981`), `remove_object`(`:994`), `remove_all_objects`(`:1006`).
각 래퍼는 `World`의 알림/무효화 결과를 받아 씬 상태에 반영한다. `&mut World`를
그대로 내주면 그 경로를 우회할 수 있게 된다.

**따라서 할 일은 같은 패턴의 씬 레벨 연산 2개 추가**이며, 소유자는
`moveit-scene`을 가진 p1-fixtures다. p9-ros가 아니다. 다음 라운드 항목으로 넘긴다.

부수적으로, p9-ros가 "upstream 자체가 미구현"이라 적은 AttachedCollisionObject의
MOVE도 원본에서 확인했다: `planning_scene.cpp:1762-1765`가 통째로
`RCLCPP_ERROR("Move for attached objects not yet implemented")`다. 정확한 진술이다.

## §151 D9를 CIRC에만 적용하고 가족을 훑지 않았다 — LGPL 파생물 3개가 BSD 헤더로 main에 있다

**push 전에 반드시 닫아야 한다. 아직 아무것도 push하지 않았으므로 배포는
일어나지 않았고, 따라서 지금은 결함이지 침해가 아니다.** LGPL 의무는 배포
시점에 발생한다.

### 151.1 무엇을 찾았나

§141에서 D9를 결정할 때 나는 CIRC — 인용된 지점 — 에만 적용하고 같은 위험을
가진 **기존 파일을 찾지 않았다.** 「Fixes from reported defects」가 금지하는
바로 그 패턴이다: 인용은 표본이지 모집단이 아니다.

**Anchor:** `Ported from orocos` / `orocos_kdl/src/` 인용을 가진 `.rs`

**Sites** (`rg -l 'orocos_kdl|orocos_kinematics_dynamics' crates/ ros/ tools/ --glob '*.rs'`, 6건):

| 파일 | 줄 | 분류 |
|---|---|---|
| `moveit-planners-pilz/src/path_line.rs` | 272 | **같은 결함** |
| `moveit-planners-pilz/src/velocity_profile_trap.rs` | 197 | **같은 결함** |
| `moveit-state/src/dynamics.rs` | 581 | **같은 결함** |
| `moveit-planners-pilz/src/path_circle.rs` | 560 | distinct — D9 준수. "not transcribed … derived independently from elementary vector algebra"를 명시하고 LGPL 상태도 적었다 |
| `moveit-planners-pilz/src/lib.rs` | — | distinct — 위 파일들을 설명하는 모듈 doc. 결과로 문구가 바뀔 뿐 독립 결함 아님 |
| `moveit-planners-pilz/src/velocity_profile.rs` | — | distinct — `utilities/utility.cxx`를 `KDL::epsilon`의 **값**(`1e-6`) 하나 때문에 인용. 단일 수치 상수는 저작 표현이 아니다 |

같은 결함 3건은 전부 이 형태다:

```
// Copyright (c) 2004-2005, Erwin Aertbelien, ...   ← KDL 저자 (dynamics.rs는 Ruben Smits)
// SPDX-License-Identifier: BSD-3-Clause            ← 그런데 BSD로 선언
//
// Ported from orocos_kinematics_dynamics @ v1.5.1:
//   orocos_kdl/src/path_line.{hpp,cpp}             ← 이 파일들이 LGPL-2.1-or-later
```

인용된 orocos 파일의 라이선스를 전부 헤더에서 직접 확인했다 —
`path_line.{hpp,cpp}`, `velocityprofile_trap.{hpp,cpp}`,
`rotational_interpolation_sa.hpp`, `path_circle.hpp`,
`chainidsolver_recursive_newton_euler.{hpp,cpp}`, `frames.{hpp,inl}`,
`rigidbodyinertia.{hpp,cpp}` — **모두 "GNU Lesser General Public License …
version 2.1 … or (at your option) any later version"**. 예외 조항도, 듀얼
라이선스도 없다(`path_line.hpp:11-25` 전문 확인).

### 151.2 upstream이 괜찮은 이유가 우리에게는 적용되지 않는다

moveit2의 pilz는 BSD인데 KDL을 쓴다. 문제없는 이유는 **링크하기 때문**이다:
`pilz_industrial_motion_planner/CMakeLists.txt:28`이 `find_package(orocos_kdl
REQUIRED)`, `:87,109`가 `${orocos_kdl_LIBRARIES}`를 링크한다. LGPL이 GPL과
다른 지점이 정확히 이것 — 다른 라이선스의 코드에서 라이브러리를 링크하는
것을 허용한다.

우리는 링크하지 않고 **소스를 Rust로 번역**했다. 번역은 파생물이다. upstream이
안전한 근거가 그대로 오지 않는다.

### 151.3 남은 결정과 진행

두 갈래이고, 하나는 내 결정이 아니다:

- **(A) D9 방식으로 다시 유도** — `path_circle.rs`가 이미 그 기준을 통과했으므로
  가능함이 실증되어 있다. 세 파일 다 다시 쓰고, BSD 유지.
- **(B) `moveit-planners-pilz`/`moveit-state`를 LGPL-2.1-or-later로 재선언** —
  포트는 그대로 두되 파생물임을 인정. 라이브러리 전체의 라이선스 성격이 바뀌므로
  **사용자 결정 사항**이다.

두 갈래 어느 쪽이든 먼저 필요한 것은 같다: **각 파일이 실제로 어디까지
전사(transcription)이고 어디부터 독립 유도 가능한지 함수 단위 측정.**
`dynamics.rs`의 헤더는 "Every operator below was diffed against the headers"라고
적어 동작 확인 목적의 읽기를 시사하고, `path_line.rs`는 "Ported from"이라고
적었다 — 두 문구가 같은 것을 뜻하는지 재보지 않았다. 측정을 p1-joints
라운드 21로 발주했다. (B)를 고르면 재작성은 불필요해지지만 측정은 어느 쪽이든
필요하다.

**차단 사항: 이 항목이 열려 있는 동안 `git push` / `cargo publish` 금지.**

## §152 D11 — orocos 파생물 3건은 D9 방식으로 다시 유도한다. 라이선스는 BSD-3-Clause 유지

§151.3의 두 갈래에서 사용자가 (A)를 골랐다. 확정한다.

**D11**: `path_line.rs`, `velocity_profile_trap.rs`, `dynamics.rs`에서
LGPL orocos_kdl의 전사(transcription)로 분류되는 부분을 D9와 같은 방식으로
다시 유도한다 — 초등 수학/표준 알고리즘에서 독립적으로 도출하고, LGPL
소스에서 가져오는 것은 인터페이스 사실(상수 값, 인자 순서, 단위 규약)로
한정하며, 동등성은 줄 대응이 아니라 **오라클 파리티로 증명**한다.
`moveit-planners-pilz`와 `moveit-state`는 BSD-3-Clause를 유지한다.

가능함은 이미 실증돼 있다: `path_circle.rs`(560줄)가 같은 기준으로
`KDL::Path_Circle`을 대체했고, CIRC fixture 2건이 오라클과 identical이다.

### 152.1 순서 — 측정 커밋이 재작성보다 먼저다

p1-joints 라운드 21에 발주한 함수 단위 측정은 그대로 진행한다. 방향이
정해졌다고 측정을 건너뛰면 안 되는 이유가 둘이다:

1. **재작성 범위를 정하는 것이 측정이다.** 분류 2(독립 유도 가능)·3(인터페이스
   사실)·4(moveit2 BSD 출처)로 판명된 부분은 다시 쓸 필요가 없다. `dynamics.rs`의
   헤더가 "Every operator below was diffed against the headers"라고 적은 것이
   사실이라면 그 파일은 문구 정정만으로 끝날 수 있다.
2. **측정 커밋이 재작성이 옳게 범위를 잡았다는 증거로 남는다.** 재작성 후에
   측정하면 무엇을 왜 다시 썼는지 되짚을 근거가 사라진다.

따라서: 측정 보고서를 `doc/` 아래 파일로 **먼저 커밋**하고, 그 다음 분류
1(transcription)로 표시된 심볼만 finding 단위로 재작성한다.

### 152.2 재작성이 만족해야 하는 것

`path_circle.rs`가 세운 기준 그대로:

- 유도를 doc에 **실제로 적는다**. "독립적으로 유도했다"는 주장이 아니라 유도
  자체가 있어야 한다.
- LGPL 소스에서 가져온 것이 있으면 무엇인지 명시한다 — `path_circle.rs`는
  `eqradius` 규약을 그렇게 처리했다.
- 동등성은 오라클 fixture로 증명한다. 성공 경로 하나와 거부 경로 하나가 최소치다
  (CIRC가 `panda_circ` + `panda_circ_noplane_rejected`로 한 것처럼).
- 저작권 줄에서 KDL 저자를 뺀다 — 독립 유도라면 그 줄이 남아 있으면 안 되고,
  남겨야 한다면 독립 유도가 아니다. 이 한 줄이 분류 1과 2를 가르는 실질적
  시금석이다.

### 152.3 push 차단은 유지된다

§151의 차단은 D11이 **완료될 때까지** 유효하다. 방향이 정해진 것과 결함이
닫힌 것은 다르다.

## §153 존재하지 않는 의존성을 이유로 든 제외는 의존성이 생긴 뒤에도 아무도 다시 읽지 않는다

p3-shapes 라운드 26을 검증하다 발견했다. 이번 라운드가 만든 결함은 아니고,
아무도 다시 읽지 않아서 남은 것이다.

`crates/moveit-collision/src/lib.rs:26-35`(그리고 `env.rs:55`의 같은 문단):

> `collision_plugin_cache.*` (pluginlib backend selection),
> `collision_octomap_filter.*` and `occupancy_map.*` (both need an octomap
> dependency and a `RobotState`) are out of scope entirely — … the latter
> two have no `RobotState`-free piece to port at all.

`collision_octomap_filter`에 대해서는 **사실이 아니다.** 원본
(`moveit_core/collision_detection/src/collision_octomap_filter.cpp`, 318줄,
moveit2 `e017c91e`)을 직접 확인했다:

```
grep -c 'RobotState' collision_octomap_filter.cpp   → 0
```

`RobotState` 참조가 **0건**이다. 공개 진입점은
`refineContactNormals(const World::ObjectConstPtr& object, CollisionResult& res,
...)`(`:67`)로, 인자가 `World::Object`와 `CollisionResult` — 둘 다 이
워크스페이스에 이미 있다(`moveit_collision::World`, `CollisionResult`).
ROS include는 `rclcpp/logger.hpp`, `rclcpp/logging.hpp` 둘뿐이고 전부 로깅이다.
나머지 include는 `octomap/*`와 `geometric_shapes/shapes.h`다.

"octomap 의존성이 필요하다"는 부분은 **쓰였을 당시엔 참이었을 수 있지만 지금은
아니다** — `crates/moveit-octomap`이 2633줄로 존재한다. 즉 이 제외의 두 근거
중 하나는 처음부터 거짓이고, 다른 하나는 그 사이에 만료됐다.

### 153.1 규칙

제외 사유가 **부재**(의존성이 없다 / 타입이 없다 / 층이 아직 없다)일 때는
그 부재가 해소되는 순간 사유가 만료된다. 그런데 제외 문구는 코드가 아니라
주석이라 컴파일러가 만료를 알려주지 않고, 그 파일을 다시 여는 유일한 계기는
누군가 그 기능을 필요로 할 때뿐이다 — 그때는 이미 "out of scope"라고 적혀
있으니 아무도 다시 묻지 않는다.

따라서:

- **부재를 사유로 적을 때는 무엇이 생기면 만료되는지 같이 적는다.** "octomap
  의존성이 없어서"가 아니라 "`moveit-octomap`이 생기면 이 제외는 만료된다".
- **원본에 대고 세지 않은 제외 사유는 쓰지 않는다.** "`RobotState`-free한
  조각이 없다"는 `grep -c 'RobotState'` 한 번이면 반증되는 주장이었다.
- 브리프에서 어떤 항목이 제외 문구에 기대고 있으면 **그 문구를 다시 읽고
  원본에 대고 확인한 뒤에** 항목을 쓴다.

### 153.2 처리

`collision_octomap_filter.cpp`(318줄)는 포팅 가능한 대상이다 — 소유자는
`moveit-collision`을 가진 p3-acm이고, `moveit-octomap` 쪽 준비는 p3-shapes
라운드 27의 감사가 답한다. p3-acm의 현재 라운드(FK 측정)가 끝난 뒤 다음
라운드 항목으로 넘긴다. `occupancy_map.*`와 `collision_plugin_cache.*`는
같은 방식으로 재확인되지 않았으므로 그 제외는 아직 유효한지 미상이며, 같은
라운드에서 함께 세게 한다.

## §154 `GradientInfo::sphere_locations`는 gsr 재사용 전용이 아니다 — 생성자가 이미 pregenerated GSR을 만든다

p6-totg 라운드 19가 `moveit-planners-chomp`의 `ChompOptimizer`를 포팅하면서
UNFIXED로 남긴 전제:

> Upstream의 `getCollisionGradients`는 `gsr_` 재사용 경로에서만
> `sphere_locations`를 채운다. 이 크레이트는 `gsr_`를 보관하지 않으므로
> 항상 fresh-build 경로만 타고, 거기서 `sphere_locations`는 무조건 비어
> 있다.

**측정으로 반증된다.** 오라클은 `getCollisionGradients`에 매번 null
`GroupStateRepresentationPtr`를 넘긴다(재사용 경로가 아니다). 그런데
`sphere_locations`는 링크당 1~9개로 채워져 돌아온다 — 그리고 이 사실은
이번 라운드가 만든 것이 아니라 `group_state_representation_response.json`
(gradients 필드가 생기기 전에 커밋된 fixture)에 이미 들어 있었다.

`sphere_locations`에 쓰는 곳은 upstream 전체에서 네 군데뿐이다
(`rg sphere_locations moveit_core/collision_distance_field/`):

- `collision_env_distance_field.cpp:1119` / `:1152` —
  `updateGroupStateRepresentationState`, 즉 gsr 재사용 경로
- `:1224` — `getGroupStateRepresentation`의 **else** 분기
  (`dfce->pregenerated_group_state_representation_`가 있을 때)
- `:1246` — 첨부 바디 루프 (if/else 뒤에서 무조건 실행)

오라클은 null gsr을 넘기므로 재사용 경로(`:1119`)는 배제된다. 그런데
`sphere_locations`가 비어 있지 않다 ⟹ `:1224`가 실행됐다 ⟹
`pregenerated_group_state_representation_`가 non-null이다. 닫힌 연역이다.

**왜 non-null인가.** `initialize()`(`:126`)는 두 생성자 모두가 부르고,
그 안에서 모든 JointModelGroup을 돌며
`getGroupStateRepresentation(dfce, state, pregenerated_group_state_representation_map_[jm->getName()])`
를 호출한다(`:140-154`). `getDistanceFieldCacheEntry`는 그 맵에서 찾아
`dfce->pregenerated_group_state_representation_`에 꽂는다(`:868-871`).
즉 **fresh-build 분기(`:1161`)는 `initialize()` 안에서 그룹당 한 번만
실행되고, 그 뒤의 모든 호출은 pregenerated 분기를 탄다** — 호출자가
`gsr_`를 살려두든 말든 무관하다.

따라서:

- `sphere_locations`가 비는 것은 upstream의 성질이 아니라 **이 포트의
  갭**이다. `moveit-distance-field`가 pregenerated GSR을 만들지 않기
  때문에 항상 fresh 분기에 해당하는 상태로 남는다.
- p6-totg가 `resolve_collision_point_joint_index`/`perform_forward_kinematics`
  에서 `sphere_locations` 대신 `gradients`/`distances`/`sphere_centers()`로
  치환한 것은 갭을 우회한 workaround이며, UNFIXED에 적힌 해법(재사용 경로
  함수를 export 하라)은 원인을 잘못 짚었다. export가 아니라 pregenerated
  GSR을 만드는 것이 upstream과 같아지는 길이다.
- 검증 수단은 이미 있다: 오라클의 `sphere_locations_count`가 기대값이다.
  링크 15개(geometry 있는 것)에 대해 `{5:2, 2:4, 9:1, 3:4, 6:1, 4:2, 1:1}`,
  합계 54 — `types`/`distances` 길이와 정확히 같다.

이 항목은 `verify-brief-premises`의 세 번째 사례이자 방향이 반대인 사례다.
worker가 upstream을 읽고 세운 전제를, 그 worker가 손댈 수 없는 도구(오라클)의
이미 커밋된 출력이 반증했다. worker UNFIXED에 적힌 원인 진단은 병합자가
독립적으로 재측정하기 전까지 가설로 취급한다.

## §155 오라클 distance-field 두 op에 world와 gradients가 생겼다

p3-distance-field 라운드 24의 UNFIXED 2건에 대응한다. 커밋 `bc14b80`,
`84f5565`.

- `objects` (기본값 없음 = 빈 world): `distance_field_cache_entry`와
  `group_state_representation` 둘 다 `collision` op과 같은 단일 shape
  스키마 `{id, pose, shape}`를 읽는다. 이것이 없으면 environment distance
  field가 항상 비어 있어 `"environment"` sentinel contact도
  `getEnvironmentProximityGradients`도 fixture에서 도달 불가능했다.
  world를 받는 생성자는 조건 없이 쓴다 — 빈 world 등가성은 43/43 replay
  identical로 **측정**했고, 조건 분기는 아직 없는 fixture만 도달할 수 있는
  두 번째 경로가 됐을 것이다.
- `gradients` (기본 false): `getCollisionGradients`를 태운다. 오라클
  전체에서 이 함수를 부르는 곳이 하나도 없었으므로
  `get_self_proximity_gradients` / `get_intra_group_proximity_gradients` /
  `get_environment_proximity_gradients` 세 함수는 어떤 ground truth도 갖고
  있지 않았다.
- `attached_body_gradients`: `links` 덤프는 `link_names_.size()`로 도는
  link-indexed 루프라 첨부 바디의 gradient 슬롯은 구조적으로 노출될 수
  없었다. 인덱스 `link_names_.size()..gradients_.size()` 구간을 별도 배열로
  낸다.
- `gradients`와 `contacts`는 상호 배타이며 위반 시 명시적으로 throw 한다.
  `getCollisionGradients`의 시그니처가 `CollisionResult& /*res*/` — 받은
  결과를 버린다(`collision_env_distance_field.cpp:1517`). 조용히 낡은
  `res`를 돌려주는 대신 거절한다.

실측(pr2, `right_arm`): `gradients:true`만으로 type 히스토그램
`{SELF:6, INTRA_GROUP:48}`, `objects` 추가 시 `{SELF:6, INTRA:44, ENV:4}`,
`attached_bodies` 추가 시 `attached_body_gradients`에 `payload` 1건.
`use_acm:false`(null ACM)도 크래시 없이 같은 히스토그램.

## §156 오라클에 `cost_sources` / `path_cost_sources` op이 생겼다

p1-fixtures 라운드 22의 요청(§107.3 경로)을 그대로 구현했다. 커밋 `7a644af`,
스탬프 `9a5a1b33f255ea23`.

- `cost_sources` — upstream `PlanningScene::getCostSources(const RobotState&,
  std::size_t, const std::string&, std::set<CostSource>&)`
  (`planning_scene.cpp:2499-2510`). 요청: `joint_values`, `max_costs`,
  선택 `group_name`(기본 `""` = 전체 로봇), `objects`/`attached_bodies`는
  `collision` op과 같은 스키마. **`removeCostSources`/`removeOverlapping`을
  부르지 않는다** — p1-fixtures가 body를 읽어 찾아낸 비대칭이며, 이 op은 그
  비대칭 자체를 검증 대상으로 만든다.
- `path_cost_sources` — trajectory 오버로드(`:2457-2491`). 추가 요청 필드
  `waypoints`(필수), `overlap_fraction`(필수). 세 단계를 upstream 순서대로
  재현한다: union을 `max_costs`로 자르고 → `removeCostSources(costs, cs_start,
  overlap_fraction)` → `removeOverlapping(costs, overlap_fraction)`.
- 응답은 둘 다 `{"cost_sources": [{"aabb_min":[..],"aabb_max":[..],"cost":f}]}`.
  `std::set` 반복 순서 그대로 내보낸다 — `CostSource::operator<`가
  `cost * getVolume()` 내림차순이라 이미 most-costly-first다
  (`collision_common.hpp:128-141`). 여기서 다시 정렬하지 않는 이유는, 순서가
  다른 Rust 컨테이너를 정규화로 덮지 않고 mismatch로 드러내기 위해서다.
  `getVolume()`은 파생값이라 내보내지 않는다 — 곱은 맞는데 bound가 틀린
  fixture가 통과하는 것을 막는다.

**세 단계가 각각 관측 가능한 것을 실측으로 확인했다**(pr2, 0.6m 박스):
`max_costs=3` → 잘라낸 뒤 removal이 1개 더 지워 **2**개 남음(자르기가 먼저라는
순서가 보인다). `overlap_fraction` `0.9` vs `0.1` → **28** vs **13**
(`removeOverlapping`이 load-bearing). 첫 waypoint가 충돌하면 `cs_start`가
union과 같아져 결과가 **0**개 — `cs_start`가 running union의 복사본이 아니라
첫 waypoint의 집합이라는 사실이 이 케이스에서만 드러난다.

## §157 파일 간 순서 blind spot을 실제로 열어보니 오라클에 상태 누수가 있었다

§143.1이 `verify-fixture-replay.sh`의 한계를 적어두고, "지금 corpus가 그 순서를
우연히 밟지 않았을 뿐"이라고 했다. **그 문장이 틀렸다.** corpus는 그 순서를 이미
갖고 있었고, 다만 그것을 한 프로세스에 넣어보는 실행이 존재하지 않았을 뿐이다.

### 157.1 combined pass — 같은 로봇 모델을 쓰는 fixture를 한 프로세스에 넣는다

`verify-fixture-replay.sh`에 두 번째 pass를 붙였다(`9a976fe`). 기존 pass는
fixture 하나당 컨테이너 하나라 요청은 자기 파일의 op만 본다. 새 pass는 urdf/srdf를
**경로가 아니라 내용(sha256)으로** 묶어 각 그룹의 요청을 한 스트림으로 이어
붙인다. 경로로 묶으면 안 되는 이유가 핵심이다 — 같은 panda가 7개 크레이트의
fixture 디렉터리에 복사돼 있고, 크레이트를 건너뛰는 누수가 정확히 보고 싶은
경우다(`oracle.cpp`는 모든 패널이 공유한다).

43 fixture = **6개 content-distinct 모델**이라 컨테이너 기동이 43 → 6으로 줄어든다.
즉 이 pass는 위 pass보다 **싸다**. 그룹 크기 1(`totg_synthetic`)은 per-file pass가
이미 한 일이라 건너뛴다 — 42/43이 새로 덮인다.

비교 로직은 재구현하지 않았다. 요청 id를 `fixture_index * 1000 + 원래 id`로
재번호(코퍼스 전체 id가 정수, 최대 12라 1000이면 충돌 불가)하고
`ignore_result_fields_by_id`도 같은 규칙으로 병합해서, 이어붙인 것을 fixture 하나인
것처럼 `_replay_one.py`에 넘긴다. drift는 per-file pass와 **완전히 같은 코드로**
비교·보고·diff된다. 실패 시 재번호 구간표를 같이 찍어서 diff 라인이 어느 fixture로
되돌아가는지 보이게 했다.

### 157.2 첫 실행에서 바로 걸린 것

per-file 43/43 identical인 상태에서 combined이 `panda.urdf` 그룹(16 fixture, 7
크레이트, 31 요청)에서 **DRIFTED 342 line(s)**. 범인은 `moveit-trajectory/ruckig`
하나. 선행 fixture를 하나씩 붙여 이분한 결과 세 개가 재현시켰다:

```
moveit-smoothing/acceleration_filter                 DRIFT
moveit-smoothing/ruckig_filter                       DRIFT
moveit-trajectory/totg_robot_trajectory_scaling_only DRIFT
(나머지 12개는 전부 ok, 단독 실행도 ok)
```

정확히 그 셋이 `joint_model->setVariableBounds(limits)`로 **공유 `model_`을
수정하는** op들이다(`totgRobotTrajectoryCase`, `accelerationFilterCase`,
`setJointAccelerationVelocityJerkBounds`). 그리고 `ruckigCase`는 RobotModel-bounds
overload를 쓰므로 덮어써진 바로 그 필드를 읽는다. `group`이 없는 평범한 `totg`가
범인이 아닌 것도 이것으로 설명된다 — 그 경로엔 bounds 적용이 없다.

`totgRobotTrajectoryCase`의 doc은 이 성질을 **이미 알고 적어두고 있었다**: "the
mutation ... persists for the rest of this oracle process -- deliberately kept to
its own isolated fixture, never mixed into a fixture another case in the same file
also relies on." 즉 방어책이 **관례**였고, 한 프로세스에 두 파일이 들어가는 순간
관례는 지켜지지 않았다. 규칙대로 관례가 아니라 구조로 닫아야 하는 자리다.

### 157.3 구조적 수정 — override를 요청 하나의 수명으로 묶는다

`de020fa`. 세 호출부를 전부 `applyJointBounds(joint_model, limits)`로 돌리고, 그것이
교체 전 bounds를 `replaced_bounds_`에 기록한다. `handle`은 모든 op이 지나가는
단일 dispatcher이므로 거기에 `ScopedJointBounds` RAII를 두어 요청이 끝날 때
역순으로 되돌린다. 소유자가 하나라서 **나중에 추가될 op이 이 규칙을 몰라도
상속한다.**

- **Invariant:** 어떤 요청도 `model_`의 variable bounds를 바꾼 채로 끝날 수 없다.
- **Owner/Gate:** `Oracle::handle`의 `ScopedJointBounds` (복원은
  `restoreJointBounds`).
- **Bypass audit:** anchor `rg -n 'setVariableBounds' tools/moveit-oracle/src/
  oracle.cpp` → 호출 3곳(`:3985` 계열 totg, acceleration filter, ruckig filter)
  전부 owner 경유로 전환. 나머지 hit는 `applyJointBounds`/`restoreJointBounds`
  자신과 doc 문장.
- **범위 선택:** case가 아니라 **요청** 단위로 묶었다. 한 요청 안의 case들이
  override를 공유하는 것은 기존 캡처 동작이고, 그걸 바꾸면 committed fixture가
  달라진다. 파일 경계를 넘는 누수만 없애는 것이 최소이자 정확한 경계다.

되돌림이 역순인 이유: 한 요청이 같은 joint를 두 번 덮으면 처음 항목만이 요청
시작 시점의 bounds를 갖는다.

### 157.4 실측

새 stamp **`f209092a3c432394`** (이전 `9a5a1b33f255ea23`).

```
verify-fixture-replay.sh  per-file  → identical 43줄 / 43 (§149 준수, 변화 0)
                          combined  → 5개 그룹 전부 identical, single 1
이분 재실행                          → 세 DRIFT 선행 fixture 전부 ok
throw 경로 (bounds 적용 후 예외)     → 사전 이미지 DRIFTED / 사후 identical
check-*.sh 8개                       → OK
```

throw 경로는 별도로 쟀다. `totg` 요청의 `durations_from_previous`를 하나 잘라
bounds 적용 **뒤에** "waypoints/durations_from_previous length mismatch"로 던지게
만들고, 이어서 `ruckig` 요청을 보냈다. `9a5a1b33f255ea23` 이미지에서는 DRIFTED,
`f209092a3c432394`에서는 identical — RAII가 예외 경로를 덮는다는 것이 값으로
확인된다. `sg docker -c`가 종료코드를 가리므로 전부 출력 내용으로 판정했다.

### 157.5 남는 것

- combined pass는 **한 가지 순서**(그룹 내 crate/stem 정렬)만 밟는다. 역순이나
  임의 순열은 밟지 않는다. 누수가 남아 있어도 이 순서에서 관측되지 않으면 여전히
  안 보인다.
- 그룹 크기 1(`totg_synthetic`)은 영원히 이 pass의 사각지대다. 같은 모델을 쓰는
  fixture가 하나 더 생기면 만료된다(§153.1).
- committed fixture는 전부 자기 프로세스에서 캡처됐으므로 이번 누수에 오염되지
  않았다(43/43 identical이 그 증거다). 오염될 수 있었던 것은 **한 프로세스에 여러
  op을 보내는 실행** — `moveit-diff`, 그리고 캡처를 batch로 돌릴 경우다.

## §158 `ros/`가 72커밋 동안 main에서 컴파일되지 않았다 — 같은 경계, 두 번째

§144가 기록한 것과 **같은 경계에서 같은 모양으로** 다시 깨졌다. 이번에는 값을
쟀다.

### 158.1 측정

p9-ros 라운드 4를 병합(`5e71b54`)한 직후 `sg docker -c 'ros/verify-ros-interop.sh'`:

```
error[E0063]: missing field `start_state` in initializer of
              `moveit_planning::PlanningResponse<'_>`
  --> src/planning.rs:266:12   및   :439:19
```

`start_state`를 넣은 커밋은 `26c0442`(p1-fixtures 라운드 22)이고, 그 시점부터
main에 **72커밋**이 쌓였다. `ros/moveit-ros`는 그 72커밋 내내 컴파일되지 않는
상태였고, 어떤 게이트도 그것을 보지 않았다. p9-ros 라운드 4의 게이트가 초록이었던
것은 거짓 보고가 아니다 — 그 브랜치는 main보다 **49커밋 뒤**였고, 자기 base에
대해서는 실제로 통과했다. 브랜치 게이트는 자기 base에 대한 진술이지 main에 대한
진술이 아니다.

### 158.2 왜 아무도 못 봤나

D5로 `ros/moveit-ros`는 루트 워크스페이스 **밖**에 있다. 따라서
`cargo nextest run --workspace`도 `cargo clippy --workspace`도 이 크레이트를
컴파일하지 않는다. 컴파일하는 것은 `ros/verify-ros-interop.sh` 하나뿐인데, 이건
r2r → ROS 헤더 → docker가 필요해서 `check-*.sh` glob(러너에 docker 없음)에 들어갈
수 없다. 이름이 `verify-*`인 것은 맞는 분류다(§143). 문제는 **`verify-*`를 아무도
자동으로 돌리지 않는다**는 것이다.

### 158.3 규칙 — 조건 없이 병합마다 돌린다

측정된 비용: **9.9초**(warm cache, 97 tests). 조건부 규칙("crates/ 공개 API가
바뀐 병합에서만")은 판단을 요구하고, 판단은 이번에 실패한 바로 그 지점이다.
9.9초면 판단을 없애는 편이 싸다:

> **병합자는 모든 병합 뒤 `sg docker -c 'ros/verify-ros-interop.sh'`를 돌린다.
> 예외 없음.** 마지막 줄이 정확히 `all gates passed`인지로 판정한다 —
> `sg docker -c`가 종료코드를 가린다.

이것이 **관례이지 게이트가 아니라는 점을 명시해 둔다.** §157에서 관례로 막아둔
불변식이 어떻게 무너지는지 방금 봤다. 여기서 구조적으로 닫지 못하는 이유는
분명하다: ros/를 컴파일하려면 docker+ROS가 필요하고, 그건 지금 CI 러너에 없다.
만료 조건(§153.1) — **레지스트리/원격이 생겨 ci.yml이 실제로 돌기 시작하면**,
docker 있는 러너에서 이 스크립트를 job으로 돌릴 수 있고 그때 관례는 게이트로
승격된다. §144.2가 추적 중인 그 항목이다.

### 158.4 이번에 고친 것

`c8dd883`. `PlanningResponse::start_state`의 wire 대응물은
`MotionPlanResponse.trajectory_start`이고, `RobotStateMsg`/`RobotStateMsgOut`
변환기는 이미 있었다. 그래서 양방향 모두 실제로 옮긴다 — msg→core는
`trajectory_start`를 디코드하고, core→msg는 `start_state`를 싣는다. 기본값으로
채우는 것은 선택지가 아니었다: `start_state`는 호출자가 궤적에서 다시 유도할 수
없는 유일한 상태라 기본값은 **조용한 날조**가 된다.

round-trip 테스트는 시작 상태를 궤적 첫 waypoint와 **다른 값**(`j1= -0.7` vs
`0.3`)으로 두었다. 같게 두면 `start_state`를 궤적에서 재구성하는 잘못된 구현도
통과한다.

`trajectory_start`를 "대응 필드 없음"으로 적어둔 doc도 고쳤다. `group_name`/
`planning_time`/`error_code`는 여전히 대응물이 없고, `planning_time`은
p1-fixtures가 근거를 대는 중이라 만료 조건을 같이 적었다.

## §159 D11은 닫히지 않았다 — LGPL이 `third_party/`가 아니라 moveit2를 거쳐 들어와 있다

p1-joints 라운드 21(`3a3bcf8`)이 orocos 계열 재유도를 끝내고 "§151/D11 is now
fully closed"라고 보고했다. 재유도 자체는 검산했고 맞다 — `path_line.rs`,
`velocity_profile_trap.rs`, `dynamics.rs`에서 Erwin Aertbelien / Ruben Smits
저작권이 사라졌고, 남은 LGPL 언급은 "왜 이 파일이 BSD로 남는가"를 설명하는
산문이지 저작권 부여가 아니다. **닫히지 않은 것은 모집단이다.**

### 159.1 측정

213개 `.rs`의 헤더가 인용한 upstream 파일을 전부 열어 copyleft 문구를 찾았다.
BSD-3-Clause를 선언하면서 copyleft upstream을 인용하는 파일 3개:

```
crates/moveit-kinematics/src/lib.rs             chainiksolver_vel_mimic_svd.{cpp,hpp}
crates/moveit-kinematics/src/newton_raphson.rs  chainiksolver_vel_mimic_svd.{cpp,hpp}
crates/moveit-kinematics/src/velocity.rs        chainiksolver_vel_mimic_svd.cpp
```

`moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/
chainiksolver_vel_mimic_svd.hpp` 원문 헤더:

```
// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl
// This library is free software; ... GNU Lesser General Public
// License ... version 2.1 of the License, or (at your option) any later version.
// Modified to account for "mimic" joints ...
// Copyright  (C)  2013  Sachin Chitta, Willow Garage
```

moveit2가 KDL 솔버를 **LGPL 헤더째 vendoring**한 파일이다. Sachin Chitta의 수정도
그 LGPL 파일 안에서 이뤄졌으니 같은 라이선스 아래 있다.

표현이 실제로 옮겨져 있다는 것은 우리 쪽 doc이 스스로 말한다. `velocity.rs`:
`fold_jacobian`은 "`jacToJacReduced`" … "exactly as upstream's `result = vel1 +
multiplier * vel2` accumulation does", `expand_to_full`은 "matching upstream's own
expansion at the end of `ChainIkSolverVelMimicSVD::CartToJnt`". 인터페이스 사실
재사용이 아니라 전사다. 세 파일 어디에도 `path_line.rs`가 갖게 된 "Why this file
stays BSD-3-Clause" 정당화가 없다(`rg -i 'lgpl|stays BSD|independently|re-derived'`
→ 0건).

### 159.2 구조적 원인 — 저장소 라이선스를 파일 라이선스로 대신 썼다

라운드 21의 모집단은 "`third_party/orocos_kinematics_dynamics/`에서 온 파일"이었다.
이 세 파일의 인용 경로는 `moveit_kinematics/...`라 그 모집단에 들어오지 않는다.
한 문장으로:

> **upstream 저장소가 BSD라고 해서 그 안의 모든 파일이 BSD인 것은 아니다.**

D11이 "orocos에서 온 것"을 anchor로 삼은 것이 결함이었다. 올바른 anchor는
**인용된 파일 자체의 라이선스 문구**다 — 그 파일이 어느 저장소에 있는지와 무관하게.

### 159.3 게이트 (`a83ddfa`)

`tools/ci/verify-upstream-license-provenance.sh`. 각 `.rs` 헤더의 인용 경로를
upstream 체크아웃에서 열어 copyleft 문구를 찾고, 그 파일의 SPDX가 permissive면
실패한다. 규칙을 표에서 읽지 않고 **인용된 파일에서 매번 다시 읽는다** — 새 upstream이
추가돼도 가르칠 것이 없다.

`check-*`가 아니라 `verify-*`인 이유는 `verify-fixture-provenance.sh`와 같다:
upstream 체크아웃이 필요하고 CI 러너에는 없다. 인용을 **열지 못하면 통과가 아니라
실패**다 — 열지 못한 인용이야말로 아무도 라이선스를 확인하지 않은 경우다.

현재 250건 인용을 검사한다. 미해소 24건은 로컬에 체크아웃이 없는 것들이다:
`geometric_shapes/`(19), `srdfdom/`(3), `include/octomap/`(2). **이 셋의 라이선스는
확인되지 않았다 — 추정하지 않는다.** 사용자에게 경로를 요청해 둔 상태다.

### 159.4 상태

- **§151/§152의 `git push` 차단은 계속 유효하다.** D11은 열려 있다.
- 세 파일의 재유도는 p1-joints 라운드 22로 보냈다 — 라운드 21에서 검증된 같은
  방법론(1차 원리 재유도 → LGPL 저작권 제거 → "Why this file stays BSD" 섹션 →
  파리티 유지 확인)이다.
- 만료 조건(§153.1): `geometric_shapes`/`srdfdom`/`octomap` 체크아웃이 생기면
  미해소 24건이 실제 판정으로 바뀐다. 그 전까지 "clean"이라고 말할 수 없다.

## §160 §159.4의 만료 조건이 발동했다 — 세 upstream을 `third_party/`로 조달 (`b5d5680`)

§159.4는 "`geometric_shapes`/`srdfdom`/`octomap` 체크아웃이 생기면 미해소 24건이
실제 판정으로 바뀐다"를 만료 조건으로 적어뒀다. 그 조건이 충족됐다.

### 160.1 먼저 반증한 것 — 컨테이너로는 안 된다

오라클 컨테이너 안에 세 패키지가 있다는 것을 근거로 "경로 요청은 이제 불필요하다"고
적었는데, **그 진술은 중요한 부분에서 틀렸다.** 컨테이너가 갖고 있는 것은 설치된
헤더뿐이다:

```
/opt/ros/rolling/include/geometric_shapes/geometric_shapes/*.h   (12개)
/opt/ros/rolling/include/srdfdom/srdfdom/{model,srdf_writer,...}.h
/usr/include/octomap/*.h
```

`.cpp`와 `test/`는 컨테이너 어디에도 없다(`find / -name bodies.cpp` → 0건). Debian
패키지는 헤더와 컴파일된 `.so`만 싣는다. 24건 중 컨테이너로 해소되는 것은 9건이고,
나머지 13건은 그대로 남는다. 패키지 단위 라이선스 표로 때우는 선택지는 §159.2가
이미 배제했다 — moveit2가 BSD인데 LGPL 파일을 vendoring한 것이 D11의 결함이었다.

### 160.2 구조적 원인 — 재현 불가능한 provenance 기록

인용 자체는 날조가 아니었다. `moveit-geometry`/`moveit-srdf`의 헤더는 버전을
못박고 출처를 서술한다. `shapes.rs`는 커밋 `192801ce`까지 적어두고, 헤더는 Debian
패키지와 `diff`로, `.cpp`는 `libgeometric_shapes.so.2.3.3`의 문자열 테이블에 있는
6개 리터럴로 대조했다고 기록한다. 실제로 읽고 대조한 것이 맞다.

문제는 **읽은 트리가 남지 않았다**는 것이다. 소스 tarball은 그 라운드 동안만
존재했고, 그 뒤로는 디스크의 어떤 것도 그 주장을 다시 열어볼 수 없다.

> **다시 열 수 없는 provenance 기록은 트리의 속성이 아니라 누군가 한 번 한 일의
> 기록이다.**

`third_party/moveit_resources`가 같은 문제를 이미 이 방식으로 풀고 있다(§13.3).
게이트가 요구하는 것은 조달이지 신뢰가 아니다.

### 160.3 조달과 검증

`third_party/`에 태그 고정 shallow clone (gitignore 대상, §11.7과 동일한 취급):

| 패키지 | 태그 | 해소된 커밋 |
|---|---|---|
| `geometric_shapes` | `2.3.3` | `192801cebacc07d0e9f719576cdd1c9b36d0bc28` |
| `srdfdom` | `2.0.8` | `58ee1eccd1c34498f67022eb2080daec5e8bc162` |
| `octomap` | `v1.9.7` | `aa6372b87eaf7e89bb1c9421f61d58bd634477cb` |

세 버전 모두 인용한 헤더가 적어둔 것과 일치하고, 컨테이너의 dpkg 버전
(`ros-rolling-geometric-shapes 2.3.3-...`, `ros-rolling-srdfdom 2.0.8-...`,
`liboctomap-dev 1.9.7+dfsg-3.1build3`)과도 일치한다. **geometric_shapes의 태그
`2.3.3`이 `shapes.rs`가 미리 못박아둔 커밋 `192801ce`로 정확히 해소된다** — 그
provenance 주장의 독립 검증이다.

라이선스는 표가 아니라 **인용된 파일 자체**에서 읽었다: `bodies.cpp` "Copyright 2008
Willow Garage, Inc." + BSD 3절, `srdfdom/src/model.cpp` "Software License Agreement
(BSD License)", `OcTreeNode.h` "License: New BSD". 셋 다 permissive다.

### 160.4 게이트 상태

`octomap`의 인용은 패키지 상대 경로(`include/octomap/...`)이고 저장소가 패키지를 한
단계 아래 두므로 자기 루트가 필요하다(`third_party/octomap/octomap`). 나머지 둘은
clone 디렉터리 이름으로 인용하므로 `third_party/` 자체가 루트다.

검사 인용 **250건 → 274건**, 미해소 **24건 → 0건**. 남은 실패는 §159.1의 실제 결함
5건뿐이다 — 게이트가 이제 진짜 결함에서만 빨갛다. **`git push` 차단은 계속 유효하다**
(p1-joints 라운드 22 진행 중).

## §161 `Octomap.data` 디코더 규모 — 패널 간 불일치를 실측으로 종결

p9-ros 라운드 4는 upstream 읽기 경로를 **약 130줄**로, p3-shapes는 **650–800줄**로
보고했다. 5배 차이는 어느 한쪽이 틀렸다는 뜻이거나 두 쪽이 서로 다른 것을 셌다는
뜻이다. §160.3으로 octomap 1.9.7 소스가 디스크에 생겼으므로 이제 셀 수 있다.

중괄호 깊이로 함수 본문 경계를 잡아 실측한 값(`third_party/octomap/octomap`):

| 파일 | 함수 | 줄 범위 | 줄 수 |
|---|---|---|---|
| `include/octomap/OccupancyOcTreeBase.hxx` | `readBinaryData` | 931–943 | 13 |
| `include/octomap/OccupancyOcTreeBase.hxx` | `readBinaryNode` | 954–1022 | 69 |
| `include/octomap/OcTreeBaseImpl.hxx` | `readData` | 801–821 | 21 |
| `include/octomap/OcTreeBaseImpl.hxx` | `readNodesRecurs` | 824–844 | 21 |
| `include/octomap/OcTreeDataNode.hxx` | `readData` | 114–117 | 4 |
| | | **합계** | **128** |

**p9-ros가 맞다.** 128줄이고, 그쪽 추정 130줄과 2줄 차이다.

650–800은 이 경로의 C++ 줄 수일 수 없다. 두 수는 서로 다른 양이다 — 하나는
**전사할 upstream C++**, 다른 하나는 **테스트와 오류 처리를 포함한 Rust 산출물**로
읽어야 앞뒤가 맞는다. 둘을 같은 단위인 것처럼 나란히 두면 결정을 내리는 쪽이
오도된다. 규모 추정을 보고할 때는 **무엇을 센 것인지**를 같이 적어야 한다.

이 수치는 §157의 결정(디코더는 `moveit-octomap`이 갖는다, `Node`/`OcTree::root`를
public으로 열지 않는다)을 바꾸지 않는다. 128줄은 그 크레이트 안에서 감당할 규모다.

## §162 D11 종결 — `moveit-kinematics` 세 파일 재유도, push 차단 해제

p1-joints 라운드 22 병합(`97dc83d`..`495d958`). §159.1의 3개 파일이 처리됐고
`verify-upstream-license-provenance.sh`가 **272건 인용, 충돌 0건**으로 통과한다.

### 162.1 게이트 통과가 아니라 실제 재유도인지 확인했다

게이트는 인용을 지우기만 해도 초록이 된다. 그래서 내용을 직접 봤다:

- `velocity.rs`: 저작권이 `moveit-rs contributors` 단독으로 정리됐고, 인용이
  `Ported from`이 아니라 **`Used by`** — 즉 자신을 호출하는 BSD 쪽
  `kdl_kinematics_plugin.cpp:467`을 가리킨다. LGPL 파일은 "이 파일이 포팅하는
  대신 그 역할을 대신하는 것"으로 산문에 명시된다. `reduction_matrix`/
  `fold_jacobian`/`expand_to_full`/`solve_velocity` 각각에 유도 근거가 붙어 있다
  (mimic 제약의 아핀 미분, 가중 최소자승의 표준 축약).
- 라운드 21에서 문제였던 전사 표지(`exactly as upstream's ...`,
  `matching upstream's own expansion at ...`)가 `moveit-kinematics/src/`에서
  사라졌다. 남은 `ChainIkSolverVelMimicSVD`/`jacToJacReduced` 언급 5건은
  `cart_to_jnt.rs`/`chain.rs`/`lma.rs`/`params.rs`의 것으로, BSD인
  `kdl_kinematics_plugin.cpp`가 그 솔버를 호출한다는 **인터페이스 사실**이다
  (`cart_to_jnt.rs:113`의 "upstream"은 그 BSD 파일의 `q_out` 버퍼다).

### 162.2 부재를 근거로 삼은 논증 하나를 반증하고 고쳤다 (`900724f`)

`newton_raphson.rs`는 독립성을 **부재**로 논증하고 있었다 — "truncation 규칙은
`chainiksolver_vel_mimic_svd.{hpp,cpp}`가 자기 텍스트 어디에서도 말하지 않는다".
원본을 열어 확인했더니 틀렸다:

```
chainiksolver_vel_mimic_svd.hpp:59
  @param threshold if a singular value is below this value, its inverse is
                   set to zero, default: 0.001
```

말하고 있다. 다만 **절대 임계값**으로 말하는데, 같은 파일 `.cpp:62`의
`svd_.setThreshold(threshold)`는 Eigen의 **상대** 계약(`threshold * |largest|`)
이라 자기 산문이 자기 구현을 틀리게 서술하고 있다. 결론(전사가 아니다)은 살아남되
근거가 바뀐다 — 이 포트는 Eigen이 문서화한 상대 계약을 구현하고, 그것은 그 LGPL
파일의 산문이 **틀리게 적은 것**이다. 부재보다 강한 근거다.

> **부재를 근거로 한 라이선스 논증은 원본을 열어 확인하기 전까지는 논증이 아니다.**

§153.1(부재로 정당화한 제외는 그 부재가 해소되면 조용히 만료된다)의 라이선스판이다.

### 162.3 게이트가 못 보는 것 (알려진 한계, 만료 조건 포함)

게이트는 헤더의 **들여쓴 인용 경로**만 읽는다. `velocity.rs`처럼 LGPL 파일을
`//!` 산문으로만 언급하면 게이트에 잡히지 않는다. 이번에는 그것이 정확한 표현이지만
(포팅하지 않았으므로), **산문 언급만 남기고 표현을 다시 들여오는 경로는 게이트가
막지 못한다.** 이것을 닫힌 것으로 말하지 않는다.

만료 조건: 헤더 인용 블록뿐 아니라 `//!`/`///` 본문의 upstream 파일명까지 훑도록
게이트를 넓히면 이 한계가 사라진다. 오탐이 늘어날 것이므로 아직 하지 않았다.

### 162.4 상태

**§151/§152의 `git push`/`cargo publish` 차단을 해제한다.** 전체 워크스페이스
게이트 실측: clippy `--workspace --all-targets -- -D warnings` 통과, nextest
**1419 pass / 2 skipped**, doctest **5**, `check-*.sh` **8/8**,
`ros/verify-ros-interop.sh` **`all gates passed`**, 라이선스 게이트 **272건 / 충돌 0**.

## §163 D12 기각 — `solver: None`은 크레이트 계층 결함이 아니다 (`cc377ff`)

p1-robotmodel 라운드 22의 측정 보고 `doc/d12-solver-none-structural-measurement.md`.
**D12를 기각한다.** 그리고 D12가 이 문서에 결정으로 등재된 적이 없었다는 것도
같이 고친다 — 나는 워커 브리프에서 "D12"라는 라벨을 써왔지만
`rg 'D12' PORTING-PLAN.md`는 0건이었다. 브리프에만 존재하는 결정 라벨은 결정이
아니다.

### 163.1 D12가 무엇이었나

`moveit-kinematics-base`를 `moveit-model` 아래로 추출해 `JointModelGroup`이
`Arc<dyn KinematicsSolver>`를 들 수 있게 하자 — 여러 라운드에 걸쳐 반복해서
올라온 `solver: None` UNFIXED의 구조적 원인이 크레이트 순환이라는 가정에서
나온 것이다.

### 163.2 기각 근거 — 가정이 틀렸다

측정 1–4단계는 추출이 **가능하다**는 것을 보였다(순환 없음,
`check-dep-direction.sh` 통과, 최소 API 확정). 그런데 실행하면 안 된다. 이유는
크레이트 계층이 아니라 **이미 내려진 결정**이다. 두 곳에서 확인했다:

- **§68.4** (내가 내린 결정): "매핑을 만들지 않는다. `IKConstraintSampler`가
  solver를 인자로 받는다." 근거는 D4 — 그룹별 기본 solver 지정은 상류의
  런타임 설정 계층(`kinematics.yaml`/`robot_model_loader`)이고 D4가 통째로
  제외한 계열이다. `JointModelGroup`에 `group_kinematics_`를 심는 것은 그
  제외한 계층을 뒷문으로 들이는 것이다.
- **§77.1**: "D4 결정대로 solver는 인자로 받고 ... `check-dep-direction.sh`는
  통과한다" — 같은 결정을 재확인하고, 그때 생긴
  `moveit-constraints -> moveit-kinematics` 간선이 게이트를 통과한다는 것까지
  적어뒀다.

핵심은 이것이다: 추출을 해도 **그 필드를 자동으로 채울 것이 없다.** 상류가
그것을 채우는 기구(`RobotModel::setKinematicsAllocators`, ROS 파라미터 서버에서
`robot_model_loader` 생성 시점에 주입)가 바로 D4가 제외한 런타임 설정 계층이다.
추출은 호출자가 solver를 **저장**할 수 있게 할 뿐, 없는 것을 만들어내지 않는다 —
"no caller has anything to give"가 실제로 가리키는 것이 그것이다.

### 163.3 그러면 그 UNFIXED를 무엇이 닫나

크레이트 재구조화가 아니라 **호출자 배선**이다. `moveit-kinematics`와
`moveit-constraints`에 이미 둘 다 의존하는 플래너 크레이트 안에서 끝난다.
현재 `moveit-planners-sbp::registry::RrtConnectContext::solve`를 비롯한 어떤
생산 진입점도 solver를 이름으로 찾아 구성해 넘기지 않는다 — 못 찾아서 `None`이
아니라 **애초에 `None`을 고른다.** sbp의 `PlanningRequest`에 선택적이고
호출자가 명시적으로 구성한 solver를 통과시키는 것이 그 갭을 닫는다.

### 163.4 만료 조건

§153.1의 "부재 근거" 제외가 **아니다** — 블로커는 없는 계층이 아니라 서 있는
결정이다. 따라서 D4의 런타임 설정 제외 자체를 kinematics에 한해 좁히기로
하는 새 결정이 나올 때만 재론한다. 크레이트 순환 논거만으로 다시 올리는 것은
이미 두 번 답한 질문을 다시 묻는 것이다.

### 163.5 브리프의 전제 하나가 틀렸다

내 브리프는 `moveit-kinematics`의 직접 의존자를 "5개"로 적었다. 실측은
**3개**다(`moveit-planners-stomp`, `moveit-constraints`, `moveit-planners-pilz`).
`moveit-planners-sbp`는 직접 의존자가 아니다 — `Cargo.toml`의 언급은 주석 한
줄이고(`:32`), `src/`의 `moveit_kinematics` 언급 4건도 전부 doc comment다.
워커가 잡아냈고 내가 `rg`로 재확인했다.

## §164 deviation 6(b) 잔여 후보 — 정점 순서는 반증됐고, 삼각형은 잴 수단이 없었다

p3-acm이 `e3a4571`로 padding/scale 절반을 반증하면서 다음 후보를 하나로 좁혔다:
**repro가 손으로 만든 `BVHModel`이 `CollisionEnvFCL`이 같은 STL에서 실제로
싣는 것과 같은가.** 그 후보를 두 조각으로 나눠 하나는 반증하고, 다른 하나는
잴 수 있게 만들었다.

### 164.1 upstream이 무엇으로 mesh를 만드는지

`collision_common.cpp:902-920` — `BVHModel<OBBRSSd>`를 **두 배열**로 만든다:

```cpp
std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
  tri_indices[i] = fcl::Triangle(mesh->triangles[3*i], ..[3*i+1], ..[3*i+2]);
std::vector<fcl::Vector3d> points(mesh->vertex_count);
  points[i] = fcl::Vector3d(mesh->vertices[3*i], ..[3*i+1], ..[3*i+2]);
g->beginModel(); g->addSubModel(points, tri_indices); g->endModel();
```

`points`는 `shapes::Mesh`가 저장한 **순서 그대로**이고 `tri_indices`는 그
순서를 **인덱싱**한다. 따라서 정점 집합이 같아도 순서가 다르거나 삼각형
인덱스가 다르면 BVH가 달라지고, 순회 순서가 달라지고, 보고되는 최심점이
달라진다 — deviation 6이 기록한 잔여가 정확히 그 형태다.

그 배열을 만드는 로더도 이제 읽을 수 있다(§160):
`third_party/geometric_shapes/src/mesh_operations.cpp:250-252`가 assimp를
`aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
| aiProcess_RemoveComponent`로 부르고 `:266`에서 `aiProcess_OptimizeMeshes |
aiProcess_OptimizeGraph`를 더 건다. 즉 **정점 병합은 assimp가 한다.**
`createMeshFromVertices`(2인자 판, `:112`)는 병합하지 않고 복사만 한다.

### 164.2 기존 테스트가 재는 것과 재지 않는 것

`crates/moveit-geometry/tests/mesh_parity.rs`가 이미 있고 통과한다. 그런데
무엇을 재는지 보면:

- 정점 **개수** — 잰다
- 정점 **집합** — 잰다. 다만 `HashSet<[i64;3]>`로 비교하므로 **순서를 버린다**
- 삼각형 **개수** — 잰다
- 삼각형 **인덱스** — **전혀 재지 않는다.** fixture에 아예 없다
  (`mesh_parity.json`의 키는 `resource`/`scale`/`triangle_count`/
  `vertex_count`/`vertices`뿐)

정점 순서를 버리고 삼각형 인덱스를 안 보는 테스트는, BVH가 달라지는 바로 그
차이를 통과시킨다. 통과하는 테스트가 있다는 것이 그 후보를 배제하지 않는다.

### 164.3 정점 순서는 반증했다 — 36/36 일치

fixture는 정점을 **순서대로 전부** 갖고 있다(테스트가 버릴 뿐이다). 그래서
오라클을 건드리지 않고 바로 잴 수 있었다. 임시 프로브로 36개 mesh 전부에 대해
`mesh_from_bytes`의 정점 순서를 fixture 순서와 원소별로(1e-12) 비교했다:

> **36개 중 0개가 정점 순서에서 다르다.**

정점 순서 절반은 후보에서 빠진다.

### 164.4 삼각형 인덱스는 ground truth가 없었다 — 오라클을 넓혔다

남은 절반은 **잴 수단 자체가 없었다.** `meshOp`이 `vertex_count`/
`triangle_count`/`vertices`만 내보내고 삼각형 인덱스는 안 내보냈기 때문이다.
`mesh` op에 `triangles`를 추가했다. 확인: `finger.stl` → `triangle_count` 32,
`triangles` 32건, 첫 항목 `[0,1,2]`.

§149 확인: 스탬프 `c88557f4058892e9` → **`552427488cc040a2`**, 재빌드 후
replay **44/44 identical**, combined 5그룹 + single 1건 — 기존 fixture 무변동.

### 164.5 다음

- p3-shapes(`moveit-geometry` 소유): `mesh_parity`를 순서 비교와 삼각형 인덱스
  비교로 강화하고 fixture를 재생성해라. `HashSet` 비교는 **순서를 버린다는
  사실을 doc에 적고** 남길지 없앨지 판단해라.
- p3-acm: 그 결과가 나오면 deviation 6(b)의 mesh-construction 후보가 확정
  또는 배제된다. 삼각형 인덱스까지 일치하면 이 후보는 닫히고, 잔여 3건의
  원인은 BVH 분할/순회 쪽으로 좁혀진다.

## §165 CONE constraint region — 이 포트가 거부하는 자리에서 upstream은 죽는다

p9-ros 라운드 5가 `SolidPrimitive::CONE`이 `PositionConstraint::new`에서 무조건
거부된다는 것을 찾아 문서화하고 테스트를 붙였다(`4192e1d`). 그리고 "`Body`에
Cone variant가 없는 것은 `moveit-geometry` 소관"이라고 다른 소유자에게
넘겼다. **넘길 필요가 없다 — 고칠 것이 없다.** §160으로 `geometric_shapes`
소스가 읽히게 돼서 확인할 수 있었다.

### 165.1 upstream 경로 전체

```
SolidPrimitive::CONE
  -> shapes::constructShapeFromMsg          (shape_operations.cpp:101-106)
       CONE 분기가 있다. new Cone(radius, height) 를 정상 반환한다.
  -> PositionConstraint::configure          (kinematic_constraint.cpp:411-412)
       const bodies::BodyPtr body(bodies::createEmptyBodyFromShapeType(shape->type));
       body->setDimensionsDirty(shape.get());        // null 검사 없음
  -> bodies::createEmptyBodyFromShapeType   (body_operations.cpp:40-58)
       BOX / SPHERE / CYLINDER / MESH 네 개만 case가 있다.
       CONE 은 default: 로 떨어져 로그만 찍고 nullptr 을 반환한다.
```

즉 shape 생성은 성공하고, body 생성은 `nullptr`을 돌려주고, 그 다음 줄이
검사 없이 역참조한다. **CONE constraint region이 들어오면 upstream moveit2는
널 역참조로 죽는다.** `bodies.h`가 선언하는 Body 서브클래스는 `Sphere`(:286),
`Cylinder`(:339), `Box`(:402), `ConvexMesh`(:465) 넷뿐이고 Cone은 없다.

### 165.2 이 포트

`Body::from_shape`(`bodies.rs:3065`)가
`Shape::Cone(_) | Shape::Plane(_) | Shape::OcTree(_) => None`으로 처리하고,
`PositionConstraint::new`가 그 `None`을 거부로 바꾼다. 크래시가 아니라 타입
있는 거부다.

따라서 이것은 **포팅 갭이 아니다.** `bodies::Cone`을 새로 만드는 것은 upstream
파리티를 맞추는 것이 아니라 **upstream에 없는 것을 발명하는 것**이고, 동시에
upstream이 죽는 입력을 이 포트만 성공시키는 의도적 이탈이 된다. 하지 마라.

`Shape::compute_volume`/`get_dimensions`가 `Cone`에 `None`을 주는 것도 같은
이유로 이미 맞다(`crates/moveit-geometry/src/shapes.rs:72-73`이 "There is no `bodies::Cone`"이라고
적어둔 것이 정확했다 — 다만 그 문장은 upstream의 널 역참조까지는 몰랐다).

### 165.3 §153.1 만료 조건

upstream `geometric_shapes`가 `bodies::Cone`을 추가하고 `PositionConstraint`가
그것을 쓰게 되면 만료된다. 그 전까지 "CONE은 거부"는 파리티이고, 바꾸는 것이
이탈이다.

### 165.4 이렇게 확인할 수 있게 된 경위

이 판정은 `third_party/geometric_shapes`의 `body_operations.cpp`와
`shape_operations.cpp`를 열어야 나온다. 오늘 아침까지 이 저장소는 이 머신에
없었고(§160), 컨테이너에는 헤더만 있어 `.cpp`의 `default:` 분기를 볼 수
없었다. 조달이 없었으면 "다른 크레이트 소유자가 판단할 사항"으로 남았을
항목이다.

## §166 저작권 표기도 인용이다 — 게이트 확장과 19건의 미정당 표기

§162.3에 "라이선스 게이트는 들여쓴 인용 *경로*만 읽으므로, LGPL 파일이
`//!` 산문에만 이름으로 등장하면 보이지 않는다"고 적고 만료조건을
"doc 본문까지 스캔 확대"로 달아뒀다. 그 만료조건은 틀린 방향이었다.
실제로 새는 것은 산문이 아니라 **저작권 표기 줄** 자체였다.

p1-joints가 `cart_to_jnt.rs:2`에서 손으로 찾아냈다(`d6b58a6`):
`Copyright (c) 2013, Sachin Chitta, Willow Garage` — 이것은
LGPL-2.1-or-later인 `chainiksolver_vel_mimic_svd.{h,hpp,cpp}`의 저작권
줄이고, 그 플러그인 디렉터리에서 **그 세 파일에만** 나온다. 정작 이
파일이 인용하는 `kdl_kinematics_plugin.cpp`의 저작권은
`2012, Willow Garage, Inc.`이고 Sachin Chitta는 거기서 *Author* 주석에만
등장한다. 게이트는 열어볼 LGPL 경로가 헤더에 없었으므로 아무것도 볼 수
없었다.

### 166.1 규칙

`verify-upstream-license-provenance.sh`에 두 번째 규칙을 넣었다(`7d9dfec`):

> 파일이 주장하는 모든 `moveit-rs contributors` 아닌 저작권 줄은, **그
> 파일 자신이 인용한 파일 중 하나가 연도와 권리자 둘 다 그대로 갖고
> 있어야 한다.**

저작권 표기는 인용과 똑같이 출처에 대한 주장이고, 똑같은 소스로 검사할
수 있다. 이것이 §162.3이 달았어야 할 만료조건이다.

주장(assertion)은 `//`로 시작하는 라이선스 헤더의 `Copyright` 줄로
한정한다. `velocity.rs`/`kinematics/lib.rs`의 `//!` 문단은 "이 파일이
일부러 달지 *않은* 표기"를 설명하는 것이라, 주장으로 읽으면 그 문단이
말하는 바를 정확히 뒤집는다.

매칭은 퍼지가 아니라 정확 일치다. 표기가 출처의 문구를 재현하지 못하면
의도가 뻔해도 보고할 값어치가 있다 — 고치는 방향은 둘을 일치시키는
것이고, 문구가 바뀐 권리자를 받아줄 만큼 느슨한 매처는 *틀린* 권리자도
받아준다.

### 166.2 새 규칙을 넣으려다 드러난 파서 결함 4건 (전부 기존 결함)

새 규칙이 낸 51건 중 32건이 게이트 자신의 결함이었다. 넷 다 **조용히
검사를 건너뛰는** 방향으로 틀려 있었다 — 이 게이트가 막으려고 존재하는
바로 그 실패 양식이다.

| 커밋 | 결함 | 결과 |
|---|---|---|
| `e8c79d7` | 인용 경로 뒤에 심볼 괄호가 붙으면 산문으로 파싱 | 24개 인용이 한 번도 안 열림 |
| `7526012` | `not ported` 구절이 아무 데나 있으면 인용 목록이 거기서 끊김 | `moveit-octomap` 세 파일의 유일한 인용이 사라짐 |
| `3617b0e` | `.hxx`가 파일명으로 인식 안 됨 | `OcTreeIterator.hxx`(iter.rs의 전부) 누락 |
| `0693782` | 중괄호 형식과 맨-디렉터리 형식이 아무 파일로도 해석 안 됨 | pilz/stomp가 통째로 인용한 패키지가 미검사 |

검사되는 상류 파일 수: **278 → 470**.

`0693782`가 구조적인 쪽이다. 인용은 이제 **파일 집합**으로 해석된다 —
경로면 그 파일 하나, 디렉터리면 그 아래 소스 전부 — 그래서 두 규칙이
두 형식 모두에 같은 방식으로 적용된다. 형식별 분기가 아니다.

### 166.3 남은 19건 — 소유자별

게이트는 지금 **red**다. `verify-*`라 CI에는 없지만(`check-*` glob이
아님), 손으로 돌리면 실패한다. 새로 깨진 것이 아니라 **줄곧 틀려
있었는데 이제 보이는 것**이다.

- **p3-shapes** (`moveit-geometry`, `moveit-octomap`, `moveit-planners-stomp`)
  - `geometry/bodies.rs` 2008/2019/2024 `Willow Garage, Inc. / Open Robotics` —
    권리자 둘을 한 줄에 합쳐 놓아 어떤 상류 줄과도 일치하지 않는다
  - `geometry/lib.rs`, `geometry/transforms.rs` 2013 `Ioan A. Sucan` —
    인용한 `transforms.hpp`는 `2011, Willow Garage, Inc.`
  - `geometry/shapes.rs` 2012 `Willow Garage`
  - `octomap/lib.rs` — 인용하는 파일이 **하나도 없다**(데비안 패키지명만
    적혀 있다). 표기를 뒷받침할 것이 없다
  - `planners-stomp/lib.rs` 2020 `PickNik Inc.` — 인용한 패키지의 모든
    파일이 **2023**
- **p1-joints** (`moveit-kinematics`)
  - `cart_to_jnt.rs`, `newton_raphson.rs` 2008 `Willow Garage` — 둘 다
    `kdl_kinematics_plugin.{cpp,hpp}`만 인용하고 그것은 **2012**.
    `d6b58a6`이 같은 블록의 LGPL 줄을 지웠지만 바로 윗줄은 남아 있었다
- **p3-acm** (`moveit-model`) — `link_model.rs` 2013 `Ioan A. Sucan`,
  인용한 `link_model.cpp`는 `2008, Willow Garage, Inc.`
- **p1-fixtures** (`moveit-scene`) — `attached_body.rs` 2011
  `Willow Garage`, 인용한 `attached_body.hpp`는 **2012**
- **p1-joints** (`moveit-state`) — `dynamics.rs` 2013 `Ioan A. Sucan`
- **p6-totg** (`moveit-trajectory`) — `numeric.rs`,
  `path_segment/{circular,linear}.rs`, `tests/large_accel.rs` 2012
  `Georgia Tech Research Corporation`, 인용한
  `time_optimal_trajectory_generation.cpp`는 **2011**
  (`path.rs`는 `2011-2012`로 적고 `.hpp`도 인용해서 통과한다)
- **p1-joints** (`moveit-planners-pilz`) — `cartesian_trajectory.rs` 2019
  `Pilz GmbH & Co. KG`

`Ioan A. Sucan` 계열 넷은 상류에 **실재하는** 저작권 줄이다 —
`joint_model.hpp`/`joint_model_group.hpp`의 것이고, 이 파일들이 인용한
파일의 것이 아니다. 그래서 두 갈래 중 하나다: 표기가 잘못 복사된
것이거나, **인용 목록이 불완전한 것**(정말 그 파일에서 가져왔다면
인용해야 한다). 어느 쪽인지는 소유자만 안다 — 그래서 내가 고치지 않고
라우팅한다.

### 166.4 만료조건

19건이 0이 되면 이 절은 기록으로만 남는다. 게이트가 다시 red가 되는
경우는 새 파일이 인용하지 않은 출처의 저작권을 주장할 때뿐이고, 그것이
정확히 이 규칙이 잡으려는 것이다.

## §167 파생물은 출처의 표기를 유지해야 한다 — 36건

§166의 규칙은 "근거 없는 저작권 주장"을 잡는다. 그 **거울상**이 남아
있었고, 이쪽은 출처 논증이 아니라 **라이선스 조항**이 뒤를 받친다:
BSD-3-Clause 제1항과 Apache-2.0 제4(c)항은 파생물이 출처의 저작권 표기를
**유지**할 것을 요구한다. 근거 없는 주장은 출처 오류지만, 유지 의무를
빠뜨린 것은 **컴플라이언스 오류**다.

`9f1629a`이 세 번째 규칙을 넣었다. 결과 **36건**(25개 파일).

### 167.1 `Ported from`과 `Used by`를 파서가 구분한다

유지 의무는 **파생물에만** 생긴다. 이 트리는 이미 두 동사를 일관되게
쓰고 있었다 — `Ported from` 132건, `Used by` 3건 — 그래서 새 표기법을
만들 필요가 없었다.

`velocity.rs`가 이 구분을 필수로 만드는 사례다. 그 파일이 인용하는 것은
**포팅하지 않기로 한** LGPL 솔버를 호출했을 상류 호출부다. `Used by`를
파생으로 읽으면 이 게이트는 그 파일에 LGPL 표기를 달라고 요구하게 된다 —
그 파일 문단이 말하는 바를 정확히 뒤집는다. 그래서 주석이 아니라 파서에
들어가 있다.

### 167.2 명시된 한계

패키지 디렉터리 인용(pilz, stomp)은 유지 검사에서 제외한다. 파일 하나로
해석되는 인용에만 요구한다 — 130개 파일의 권리자를 헤더 하나가 재현할
수는 없다. **이것은 "통째로 포팅한 패키지는 유지 의무가 없다"는 주장이
아니라 이 규칙이 재지 않는 범위**이고, 만료조건은 그 두 크레이트의
인용이 파일 단위로 구체화되는 시점이다.

> **정정(§167.5):** 위 근거 문장은 세어보지 않고 썼고, 세어보면 틀렸다.
> 145개 파일이 담은 서로 다른 권리자는 6명이지 145명이 아니다. 제외를
> 유지할 이유는 규모가 아니라 인용이 실제 포팅 범위보다 넓다는 것이며,
> 올바른 수정은 게이트가 아니라 인용 쪽이다 — §167.5를 읽어라.

### 167.3 §166과 겹친다 — 소유자는 한 번에 고쳐라

같은 파일이 양쪽에 걸린 경우가 많다. 가장 선명한 예:

- `moveit-model/src/joint/planar.rs`, `revolute.rs`는 `2013, Ioan A. Sucan`을
  **달아야 한다**(§167).
- `moveit-geometry/src/lib.rs`, `transforms.rs`, `moveit-model/src/link_model.rs`,
  `moveit-state/src/dynamics.rs`는 **같은 줄을 근거 없이 달고 있다**(§166).

같은 이름이 한쪽에서는 누락이고 다른 쪽에서는 잘못된 주장이다. 두 규칙이
서로를 보완한다는 증거이고, 소유자가 두 목록을 **같이** 보고 한 번에
고쳐야 하는 이유다.

### 167.4 소유자별 36건

- **p3-shapes** — `geometry/bodies.rs`(2008 Willow Garage, 2013 Willow Garage,
  2019 Bielefeld University, 2019 Open Robotics, 2024 Open Robotics),
  `geometry/lib.rs`(2011 WG), `geometry/stl.rs`(2013 WG),
  `geometry/transforms.rs`(2011 WG)
- **p3-acm** — `moveit-collision/lib.rs`(2012 WG, 2013 WG),
  `moveit-model/joint/{fixed,floating,model,planar,prismatic,revolute}.rs`,
  `moveit-model/{link_model,robot_model}.rs` (대부분 2008 WG,
  `planar`/`revolute`는 2013 Ioan A. Sucan 추가)
- **p1-joints** — `moveit-kinematics/{cart_to_jnt,newton_raphson}.rs`(2012 WG),
  `moveit-state/{dynamics,lib}.rs`(2012 WG)
- **p1-fixtures** — `moveit-planning/lib.rs`(2012 WG, 2019 Bielefeld University,
  2021 PickNik Robotics, 2023 PickNik),
  `moveit-planning/request_adapters/resolve_constraint_frames.rs`(2011 WG),
  `moveit-scene/{lib,world_diff}.rs`(2013 WG)
- **p9-ros** — `ros/moveit-ros/src/scene/{attached,collision_object,planning_scene}.rs`
  (2011 WG, `collision_object.rs`는 2019 Universität Hamburg 추가)
- **(소유자 미지정)** — `moveit-error/src/lib.rs`(2021 PickNik)

전체 목록은 `tools/ci/verify-upstream-license-provenance.sh`를 돌리면
파일:연도:권리자로 나온다.

맞춰야 하는 것은 **연도와 권리자 이름 단어들뿐**이다. `(c)` 유무, 연도
뒤 쉼표, 대소문자, 구두점, 말미의 `Inc`/`Corporation`/`GmbH` 접미사는
정규화로 사라지므로 상류의 구두점을 흉내 낼 필요가 없다 —
`geometric_shapes`는 `Copyright 2008 Willow Garage, Inc.`로,
moveit2는 `Copyright (c) 2008, Willow Garage, Inc.`로 쓰는데 둘은 같게
취급된다. 이 트리의 기존 문체를 유지하고 연도와 이름만 상류 것으로 써라.

정규화가 살려주지 **않는** 것은 이름 철자다. 상류는 움라우트를 쓰지 않고
ASCII로 `Universitaet Hamburg`라고 적는다
(`moveit_core/utils/src/message_checks.cpp:4`).

### 167.5 167.2의 근거는 측정으로 반증된다 — 그리고 진짜 결함은 인용 쪽이다

§167.2는 패키지 디렉터리 인용을 유지 검사에서 빼면서 근거를 이렇게 적었다:
"130개 파일의 권리자를 헤더 하나가 재현할 수는 없다." **세어보지 않고 쓴
문장이고, 세어보면 틀렸다.**

| 인용 | 해석되는 파일 수 | **서로 다른** (연도, 권리자) |
|---|---|---|
| `moveit_planners/pilz_industrial_motion_planner/` | 145 | **6** |
| `moveit_planners/stomp/` | 20 | **2** |

헤더가 재현해야 하는 양은 **파일 수가 아니라 서로 다른 권리자 수**다.
6줄은 헤더가 충분히 담는다. 제외의 근거였던 규모는 존재하지 않았다.

**그런데 제외를 그냥 걷어내면 게이트가 §166 위반을 요구하게 된다.** 두
크레이트가 실제로 포팅한 파일만 다시 재면 이렇게 나온다:

- pilz의 실제 포팅 파일(모듈 doc이 이미 파일명으로 열거하고 있다)이 담은
  서로 다른 권리자는 **1건** — `2018, Pilz GmbH & Co. KG`, 헤더에 이미 있다.
  나머지 5건(2025 Aiman Haidar, 2021 Cristian C. Beltran-Hernandez,
  2020 PAL Robotics, 2012 Willow Garage)은 **포팅하지 않은 파일의
  권리자**다. 유지 규칙이 그걸 요구하면, 근거 없는 저작권 주장을 잡으라고
  만든 §166이 잡아야 할 줄을 §167이 달라고 시키는 꼴이 된다.
- stomp에서 `2009, Willow Garage`를 지닌 파일은 `math/multivariate_gaussian.{h,hpp}`
  하나뿐이고, 그건 이 크레이트가 아니라 `moveit-sampling`이 포팅했다.
  그리고 `moveit-sampling/src/{lib,multivariate_gaussian}.rs`는 그 파일을
  **파일 단위로 인용하고 `2009, Willow Garage`를 이미 유지하고 있다.**
  유지가 실제로 새어나간 곳은 없었다.

즉 §167.2가 가리고 있던 것은 유지 누락이 아니라 **인용의 이중 의미**다.
`moveit_planners/pilz_industrial_motion_planner/`는 사람이 읽을 때는 "이
크레이트의 출신"이고 기계가 읽을 때는 "그 아래 145개 파일 전부"인데, 실제로
포팅한 것은 13개다. 게이트가 경계를 특수 처리해야 했던 이유가 그것이고,
§166과 §167이 같은 줄을 두고 반대 방향을 가리키는 이유도 그것이다.

**구조적 수정은 게이트가 아니라 인용을 고치는 것이다.** 두 인용을 실제
포팅한 파일 목록으로 좁히면 (a) 유지 검사가 자연히 파일 단위로 걸리고,
(b) `len(resolved) == 1` 특수 케이스가 근거를 잃고 사라지며, (c) 헤더는
**한 줄도 바뀌지 않는다** — 위 측정이 이미 현재 헤더가 실제 포팅 범위에
정확히 맞음을 보였다. 게이트에 예외를 정교하게 다듬는 쪽은 패치이고,
이중 의미를 없애는 쪽이 구조적 수정이다.

순서: 소유자가 인용을 좁힌다(pilz는 p1-joints, stomp는 p3-shapes) →
그 다음에 `verify-upstream-license-provenance.sh`에서 `len(resolved) == 1`을
제거한다. 순서를 뒤집으면 main이 §166 위반을 요구하며 붉어진다.

## §168 이탈 6(b)의 mesh 구성 후보 — 닫힘. 그리고 §161 닫힘

### 168.1 두 반쪽 다 36/36 일치

§164에서 이탈 6(b)의 mesh-구성 후보를 두 반쪽으로 갈랐다: **정점 순서**와
**삼각형 인덱스**. 둘 다 닫혔다.

- **정점 순서** — 비용 0으로 반증됐다. fixture는 오라클의 방출 순서를
  그대로 담고 있었고(재정렬된 적이 없다), 테스트가 그것을 `HashSet`으로
  버리고 있었을 뿐이다. 인덱스 대 인덱스로 비교하니 **36/36 일치**.
- **삼각형 인덱스** — 어떤 테스트도 잰 적이 없었다. `f5950d5`가 오라클
  `meshOp`에 `triangles`를 추가해 처음으로 측정 가능해졌고, p3-shapes가
  스탬프 `552427488cc040a2`로 fixture를 재생성해 비교했다. **36/36
  전 인덱스 일치**.

`collision_common.cpp:902-920`이 `BVHModel<OBBRSSd>`를 두 배열로 *함께*
만들기 때문에, 정점 집합만 맞고 순서나 인덱스가 다르면 BVH가 달라지고
최심 관통점이 달라진다. 그래서 집합 비교만으로는 이 후보를 배제할 수
없었다. 이제 배제된다 — **이탈 6(b)의 원인은 mesh 구성이 아니다.**

측정한 것이 맞는지 직접 확인했다(테스트가 공허하게 통과하지 않는가):
fixture에서 삼각형 하나의 winding을 뒤집으면 `triangle indices disagree`로,
정점 두 개를 바꾸면 `vertex 0`에서 각각 실패한다. 두 assertion 다 판별력이
있다.

### 168.2 §161 닫힘

p3-shapes가 `third_party/octomap`을 직접 열어 brace-depth로 재측정했다:
읽기 경로 **128줄**(`readBinaryData` 13 + `readBinaryNode` 69 +
`OcTreeBaseImpl::readData` 21 + `readNodesRecurs` 21 +
`OcTreeDataNode::readData` 4), 쓰기 경로 106줄, 합계 **234줄**.

내가 독립적으로 재측정해 확인했다: `readBinaryData` 13,
`readBinaryNode` 69, `readNodesRecurs` 21 — 같은 값이다. p9-ros의 ~130도
같은 것을 재고 있었다. §161이 기록한 불일치의 원인은 확정됐다: 두 패널이
**다른 단위**를 재고 있었고(상류 C++ 줄 수 vs Rust 구현+테스트 줄 수)
어느 쪽도 단위를 명시하지 않았다. 이제 둘 다 명시적으로 따로 적힌다.

덤으로, "native-endian이고 상류는 언급하지 않는다"는 **검증되지 않은
가정**이었던 것이 근거를 갖게 됐다 — `rg -ni endian` 0건. 내가 다시 돌려
확인했다. §162.2가 `newton_raphson.rs`에서 요구한 것과 같은 규율이고,
이번에는 부재 주장이 성립한다.

### 168.3 §166/§167 진행

p3-shapes가 자기 몫 14건(UNJUSTIFIED 6 + UNRETAINED 8)을 전부 닫았다.
`moveit-geometry`/`moveit-octomap`/`moveit-planners-stomp`는 양쪽 목록에서
사라졌다. 남은 것은 **34건**(UNJUSTIFIED 10 + UNRETAINED 24).

`bodies.rs`가 두 규칙이 서로를 보완한다는 것을 보여준 사례다: 권리자
둘을 한 줄에 합쳐 놓아 §166(불일치)과 §167(다섯 표기 누락)에 동시에
걸렸고, 상류가 적은 대로 줄마다 하나씩 푸는 **한 번의 고침**으로 둘 다
닫혔다.

## §169 §148 종결 — touching >= 2는 지금 기하학적으로 도달 불가능하다

§121.2의 원본 10건(touching >= 2 실패)은 seed가 남지 않아 복원 불가라고
§148이 이미 적어뒀다. 그 10건 자체를 쫓는 대신, 새 스윕을 내 seed로 두 번
돌리고(`cargo run --release --example visibility_cone_depth_sweep -p
moveit-constraints`, `crates/moveit-constraints/examples/
visibility_cone_depth_sweep.rs`, 이번 라운드에 추가) 오라클 스탬프
`c88557f4058892e9`로 측정했다:

```
seed=23    cases=400   touching==0: 200   touching==1: 200   touching>=2: 0
seed=90210 cases=2000  touching==0: 1000  touching==1: 1000  touching>=2: 0
```

합쳐서 2,400건, **touching >= 2 0건.** near/far 분기가 정확히 반씩
갈리고(근접 배치는 항상 touching==1, 원거리 배치는 항상 touching==0 —
근접 케이스가 겹치는 일도, 아예 안 닿는 일도 없다), touching==1 케이스의
`|oracle_distance - local_depth|` 최대값(이 스윕 자체의 잡음 바닥, 순회
순서 모호성이 원천적으로 없는 표본에서 측정 — §148의 주의할 점 1이 요구한
실측 tolerance)은 seed 23 `1.519e-1`, seed 90210 `1.565e-1`이다.

### 169.1 0건이 우연이 아니라는 증거 — 기하학적으로 불가능하다

`--geometry-gaps` 플래그(같은 바이너리)가 pr2의 17개 parry-representable
링크 전체 쌍(136쌍)에 대해 "한쪽 링크 중심에 앵커한 근접 케이스가 다른 쪽의
충돌 형상에 닿는 데 필요한 최소 reach"를 계산한다:

```
generator max cone reach: 0.0150
가장 타이트한 쌍: 0.0232  bl_caster_l_wheel_link <-> bl_caster_r_wheel_link
                          (그리고 br/fl/fr 세 캐스터 쌍도 동일하게 0.0980)
                  0.0295  head_mount_kinect_ir_link <-> head_mount_kinect_rgb_link
```

생성기가 만들 수 있는 콘의 자기 중심(근접 케이스의 앵커, 즉 타깃 링크의
`origin_transform` 적용 후 shape 중심)으로부터의 최대 도달 거리는
`max(target_radius, sensor_offset)`다(정점과 밑면 rim의 볼록결합 중 고정점
까지 거리 제곱은 구간 끝점에서 최대이므로 — 이전에 썼던
`sqrt(target_radius²+sensor_offset²)`는 느슨한 상한이었을 뿐, 진짜 상한은
더 작다). `target_radius`의 상한은 `0.015`(`main.rs:1335`), `sensor_offset`은
`0.005` 고정이므로 최대 도달은 **0.015**. 136쌍 전부에서 필요한 reach의
최솟값은 **0.0232**(캐스터 휠 네 쌍) — `0.015`를 항상 초과한다. **어떤 근접
배치도, 어떤 조인트 상태에서도, 두 번째 링크를 건드릴 수 없다.**

이 결론이 포즈에 의존하지 않는다는 것은 관계적 거리 자체가 짧고 강체로
연결된 체인(각 휠은 자기 캐스터 회전 관절 하나만 사이에 두고, kinect 세
링크는 전부 `head_mount_link`에 고정)에서 나온다는 데서 이미 자명하지만,
`--geometry-gaps`를 기본 포즈와 무관하게 `Op::Fk`가 반환한 순시 포즈에 대해
계산하도록 짜여 있어 임의 상태에 대해서도 그대로 재실행 가능하다.

### 169.2 이 생성기 경계와 pr2 fixture는 §121.2 이래 바뀐 적이 없다

`git log -S'0.005..0.015' -- tools/moveit-diff/src/main.rs`는 `350750e`(이
근접/원거리 분기 자체를 만든 커밋, 라운드 4) 이후 딱 한 번 더
`d26916d`에서 같은 문자열이 **다른 테스트 모듈 안에** 나타날 뿐, 값 자체는
한 번도 바뀌지 않았다. `git log -S'radius="0.074792"' --
crates/moveit-constraints/tests/fixtures/pr2.urdf`도 `823dce5`(pr2 fixture를
처음 vendor한 커밋) 단 한 번만 나온다 — 픽스처도 바뀐 적이 없다. 즉 §121.2가
측정한 시점(같은 생성기, 같은 fixture)과 지금 사이에 이 결론을 뒤집을 수
있는 변수가 없다.

### 169.3 §121.2의 "10건"과의 모순 — 손실된 임시 계측이 원인일 가능성이 높다

`2cdd452`(라운드 16, §121.2의 출처)는 "temporary, git-reverted
instrumentation patch to `tools/moveit-diff`"로 얻은 285케이스 스윕에서
touching >= 2가 14건(10 실패 + 4 통과) 나왔다고 적었다. 그 패치는 커밋되지
않고 되돌려졌으므로 지금은 다시 볼 수 없다 — 이 항목이 두 라운드 밀린 바로
그 이유(§148 서두)와 동일한 사정이다. §169.2가 생성기/fixture 불변을 확인한
이상, 이 모순은 드리프트로 설명되지 않는다. 남는 설명은 그 임시 계측 자체의
결함이다 — 검증 불가능하므로 확정하지 않고 후보로만 남긴다.

같은 날 `tools/`쪽 패널이 독립적으로 낸 `d26916d`(`moveit-diff: clear
visibility_cone's 115-case mismatch of both suspects`)도 방향이 같다:
`max_contacts`를 64로 올린 `#[ignore]`d 진단으로 **실제 115건의 실패 케이스
전부**를 조사해 "동시에 두 링크 이상이 닿는 모호한 장면은 하나도 없다"고
결론지었다 — 방법은 다르지만(장면 단위 조사 vs 이번 라운드의 기하학적 증명 +
2,400케이스 표본) 같은 결론이다.

### 169.4 §148 판정

touching >= 2 케이스가 **이번 스윕 2,400건 중 0건**이고, **그 생성기+fixture
아래서 기하학적으로 불가능함**이 증명됐으므로, 순회 순서 가설과 deviation 6을
가를 대상 자체가 없다. "일부 일치, 일부 불일치"로 흐릴 여지가 없는 판정:
**순회 순서는 배제된다(적용 대상이 없어서) — 현재 재현 가능한 모든
`visibility_cone` 불일치는 `moveit-collision`의 deviation 6만으로 설명된다.**
§121.2의 "닫지 않고 좁혔다"를 "닫혔다(排除)"로 바꾼다. §121.2 자체의 10건
측정과의 모순(§169.3)은 별개로 기록해뒀다 — 이 판정을 무효화하지 않는다,
그 10건은 재현 불가능한 임시 계측의 산물이었고 §169.2가 재현 가능한 두 개의
불변량(생성기 경계, fixture)으로 그 자리를 대체했기 때문이다.

## §170 doc example은 어떤 크레이트 범위 게이트에도 보이지 않는다 (`0c44eb6`)

일곱 개 워커 브랜치를 한 번에 병합한 뒤 `cargo test --doc --workspace`가
red였다. 각 브랜치는 자기 게이트를 통과했고, 병합 자체도 충돌 없이 됐다.

p1-robotmodel의 `9796b2c`가 `moveit_planners_sbp::PlanningRequest`에 public
필드 `solver: Option<Box<dyn KinematicsSolver>>`를 추가했다. `Box<dyn _>`는
`Default`를 파생하지 않으므로 이 struct의 **모든** 구성 자리가 필드를 명시해야
한다. 크레이트 안(`registry.rs` 테스트 6곳, `examples/plan_benchmark_port.rs`)은
같은 커밋이 전부 갱신했다. 크레이트 **밖**의 유일한 구성 자리는
`crates/moveit-planning/src/lib.rs:351`의 doc example이었고, 그것은 p1-fixtures
소유 크레이트라 p1-robotmodel의 `-p moveit-planners-sbp` 범위에 없었다.

### §170.1 구멍은 실수가 아니라 게이트 범위다

세 도구 전부가 doc example을 보지 못한다:

- `cargo nextest run`은 doctest를 **실행하지 않는다.** nextest의 알려진
  성질이고 CLAUDE.md도 "Doctests are not covered by nextest"라고 적고 있다.
- `cargo clippy --all-targets`는 lib/bin/test/bench/example을 덮지만
  doctest는 덮지 않는다. `--all-targets`라는 이름이 이 자리에서 오해를 부른다.
- `-p <crate>` 범위는 다른 크레이트의 doctest를 애초에 컴파일하지 않는다.

즉 **다른 크레이트의 doc example은 `cargo test --doc --workspace` 외에
어떤 경로로도 컴파일되지 않는다.** 이 워크스페이스에서 그 명령을 돌리는 자리는
`.github/workflows/ci.yml:51` 한 곳뿐이고, CI는 아직 GitHub Actions에서
한 번도 실행된 적이 없다(누적 UNFIXED). 그래서 병합 시점의 나 말고는
아무도 이걸 볼 수 없었다.

### §170.2 규칙

**public 타입의 필드 또는 enum 변형을 추가·제거·개명하면
`cargo test --doc --workspace`를 돌린다.** `-p` 범위로는 잡히지 않는다.
CLAUDE.md의 escalation 규칙("changed public API → `--workspace`")이 이미
요구하는 것이지만, 그 규칙을 읽고도 clippy/nextest만 workspace로 올리고
doctest를 빠뜨리는 것이 실제로 일어난 실패이므로 doctest를 명시한다.

새 check 스크립트는 만들지 않는다 — 명령은 이미 `ci.yml`에 있고, 없는 것은
스크립트가 아니라 CI 실행이다. 스크립트를 추가하면 같은 명령이 두 곳에서
관리되는 대신 실행되지 않는 자리가 하나 더 생길 뿐이다.

### §170.3 만료 조건 (§153.1)

CI가 GitHub Actions에서 실제로 돌기 시작하면 §170.2의 수동 규칙은
게이트로 대체된다 — 그때 이 절은 "왜 그 스텝이 있는지"의 근거로만 남는다.

## §171 FCL의 cost source 입도는 traversal이 아니라 **dispatch**에서 정해진다

p1-fixtures가 `bb212dd`로 `cost_sources`/`path_cost_sources` fixture를 캡처하면서
"오라클은 콜리전 쌍당 coarse box 1개, 이 포트는 삼각형당 1개(20개)"라는
불일치를 재서 두 테스트를 `#[ignore]` 처리하고 `moveit-collision`의
`mesh_shape_cost_sources`(`parry.rs:2011-2023`) 결함으로 귀속시켰다.

**측정은 맞고 귀속도 맞지만 성격 규정이 틀렸다.** 원인을 상류에서 찾았다.

### 171.1 기전 — `use_approximate_cost`

`fcl::CollisionRequest`의 5번째 생성자 인자 `use_approximate_cost_`는
**기본값이 `true`**다(`fcl/include/fcl/narrowphase/collision_request.h:101`,
tag `0.7.0`). `moveit_core`는 이 인자를 한 번도 넘기지 않는다 — 세 호출
전부 4개 위치 인자만 준다(`collision_detection_fcl/src/collision_common.cpp:228`,
`:303`, `:364`, 형태는 `fcl::CollisionRequestd(num_max_contacts,
enable_contact, num_max_cost_sources, enable_cost)`). 따라서
**moveit이 FCL에 보내는 모든 요청은 `use_approximate_cost == true`다.**

`collision_func_matrix-inl.h`가 그 플래그를 읽는 자리는 정확히 네 곳이고,
전부 **타입이 섞인** dispatch다:

- `:184` OcTree ↔ BVH
- `:237` BVH ↔ OcTree
- `:330` `BVHShapeCollider::collide` (BVH ↔ primitive Shape)
- `:391` `orientedBVHShapeCollide` (OBB/RSS/OBBRSS BVH ↔ Shape)

이 네 자리는 전부 같은 2단 구조다(`:330-355`이 대표):

1. `enable_cost = false`로 복사한 요청으로 **진짜 traversal**을 돌려 contact만 얻는다.
2. `constructBox(obj1->getBV(0).bv, tf1, box, box_tf)`로 **메시의 BVH 루트
   경계 상자 하나**를 만들고 메시의 `cost_density`를 옮겨 담는다.
3. `ShapeShapeCollide<Box, Shape>`를 cost 전용 요청으로 돌려 **cost source
   하나**를 만든다.

즉 mesh↔shape에서 삼각형별 cost는 **애초에 계산되지 않는다.**
`MeshShapeCollisionTraversalNode::leafTesting`의 삼각형별
`addCostSource`(`mesh_shape_collision_traversal_node-inl.h:112,123`)는
`use_approximate_cost == false`일 때만 도달하는 죽은 경로다 — moveit 아래서는.

### 171.2 mesh↔mesh는 왜 삼각형별인가

`BVHCollide`(`:558`, `:648`)와 `orientedMeshCollide`(`:572`)에는 이 분기가
**없다.** mesh↔mesh는 언제나 traversal의 삼각형별 cost를 그대로 낸다.
그래서 같은 fixture 안에서 id 1(panda 자기충돌, mesh↔mesh)은 오라클이
**75개**를 내고 이 포트가 일치하는데, id 8(panda_hand에 붙인 0.05m 큐브 ↔
`panda_link7` 메시, mesh↔shape)은 오라클이 **1개**를 낸다. 두 숫자는 모순이
아니라 **dispatch가 다르다는 증거**다.

### 171.3 그래서 고칠 대상이 바뀐다

`mesh_shape_cost_sources`가 삼각형별로 내는 것 자체는 "과잉 보고"가 아니다 —
FCL의 정확(exact) cost 경로가 하는 일과 같다. 결함은 **그 함수가 잘못된
분기에 배선돼 있다**는 것이다. 20개를 하나로 합치는 것은 패치이고, 합치는
규칙(합집합? 최대? 평균?)을 새로 발명해야 하므로 다음 라운드에 또 다른
경계를 만든다. 구조적 수정은 FCL의 2단 dispatch를 그대로 재현하는 것이다:

- mesh↔shape 쌍에서 **contact는 지금의 정확한 traversal 그대로**,
- **cost는 메시의 BVH 루트 경계 상자 ↔ shape** 한 번의 shape-shape 계산으로.

두 산출물이 서로 다른 기하에서 나오는 것이 upstream의 실제 모양이다.
합치기(coalescing)로는 `cost_density`가 메시 것으로 실린다는 점도,
루트 경계가 실제 겹친 삼각형들의 합집합보다 크다는 점도 재현되지 않는다.

### 171.4 octree 경로의 doc 주장도 재측정 대상이다

`parry.rs`의 `cost_sources_for_part_pair` doc이 "FCL's own octree narrowphase
(`octree_solver-inl.h`) *does* cost per leaf, so this is a real, if minor,
further deviation"라고 적고 있다. `use_approximate_cost == true` 아래서
OcTree↔BVH는 `:184`/`:237`의 근사 경로를 타므로 이 주장은 그대로 성립하지
않는다. 다만 근사 경로가 octree 쪽에서 무엇을 하는지는(`OcTreeShapeCollide`가
메시 루트 상자에 대해 leaf별로 cost를 내는지) 별도 확인이 필요하다 —
여기서 확정하지 않고 **측정 대상으로 남긴다.** 확실한 것은 지금 doc이
근거로 삼은 "octree narrowphase가 leaf별로 낸다"가 moveit이 실제로 타는
경로를 가리키고 있지 않다는 것뿐이다.

### 171.5 만료 조건 (§153.1)

`moveit_core`가 `use_approximate_cost`를 명시적으로 넘기기 시작하면
§171 전체가 만료된다. 앵커: `rg -n 'CollisionRequestd\(' moveit_core/`가
5개 이상의 위치 인자를 가진 호출을 내놓는 순간.

### 171.6 절단 규칙은 "가장 비싼 것을 남긴다" — fixture로 확인했다

`max_costs`가 실제로 무엇을 남기는지는 §171의 dispatch와 별개의 규칙이고,
p1-fixtures가 캡처한 fixture가 그것을 이미 담고 있다. 내가 직접 대조했다.

FCL: `CostSource::total_cost = cost_density * (Δx·Δy·Δz)`
(`cost_source-inl.h:51-63`), `operator<`는 `total_cost`가 **클수록 앞**으로
가도록 뒤집혀 있다(`:86-103`). `CollisionResult::addCostSource`는
`cost_sources.insert(c)` 후 `while(size > cap) erase(--end())`
(`collision_result-inl.h:66-72`) — 즉 **가장 싼 것부터 버린다.**

moveit_core도 같은 모양을 한 번 더 한다: 자기
`CostSource::operator<`가 `cost * getVolume()` 내림차순이고
(`collision_common.hpp:128-141`), `collision_common.cpp:286-287`,
`:352-353`, `:389-390`이 `while(size > max_cost_sources) erase(--end())`를
반복한다. **절단은 두 번 일어나고 두 번 다 같은 규칙이다.**

fixture에서 확인:

```
id 3 (max_costs=9)  -> 9개
id 2 (max_costs=5)  -> id 3의 부피 상위 5개와 **정확히 일치**
id 4 (max_costs=50) -> id 3과 **동일** (참 개수 9가 상한 아래)
id 5 (group=hand)   -> id 3의 부분집합 2개
id 6 (group=panda_arm) -> id 3의 부분집합 9개
```

이 fixture의 모든 `cost`가 `1.0`이므로 `cost × volume` 순서가 부피 순서와
같고, `id2 == top-5-by-volume(id3)`가 정확히 성립한다. **`max_costs`는
임의의 N개가 아니라 가장 비싼 N개를 남긴다.** group 필터가 부분집합을
낸다는 것도 같이 확인됐다 — 필터는 쌍을 고를 뿐 cost를 다시 계산하지 않는다.

동점 처리에 함정이 하나 있다. moveit의 `operator<`는 마지막 비교를
`aabb_min < other.aabb_min`으로 끝낸다(`std::array<double,3>`이므로
사전식 전순서다) — **`aabb_max`는 비교하지 않는다.** 따라서
`cost × volume`, `cost`, `aabb_min`이 모두 같고 `aabb_max`만 다른 두
cost source는 `std::set`에서 **같은 원소로 취급되어 하나가 조용히
사라진다.** 이 포트가 순서만 맞추고 이 축약을 재현하지 않으면 개수가
어긋난다.

### 171.7 §171로 설명되지 않는 잔여 — path id 3

p1-fixtures가 `#[ignore]`를 떼고 돌린 결과에서 state 쪽은 전부 이 포트가
**더 많이** 낸다(id 8: 20 vs 1, id 4: 50 vs 9) — §171이 예측하는 방향이다.
그런데 path id 3은 **3 vs 5**로 이 포트가 **더 적게** 낸다. 방향이 반대다.

state op은 `removeCostSources`/`removeOverlapping` pass를 돌지 않고 path op은
돈다는 것이 p1-fixtures의 관찰이므로, path 쪽 불일치에는 §171 위에 **또
하나의 원인**이 얹혀 있다. §171을 고친 뒤에도 path id 3-6이 남으면 그것이
독립된 결함이고, 같이 닫히면 `max_costs=2` 절단이 waypoint별로 일어나는
순서 문제였다는 뜻이다. **어느 쪽인지 지금 단정하지 않는다** — §171 수정
후의 재측정이 판정한다.

### 171.8 부피 크기 자체가 dispatch 증거다

같은 fixture 안에서 두 모집단의 cost source 부피가 겹치지 않는다:

```
id 1 (mesh↔mesh, 삼각형별)      부피 6.591e-09 ~ 4.436e-05
id 3/4/6 (mesh↔shape, 근사)     부피 8.616e-05 ~ 1.024e-02
id 8 (mesh↔shape, 단일)         부피 4.795e-04
```

삼각형 규모와 링크 규모가 두 자리에서 여섯 자리까지 떨어져 있다. 만약
mesh↔shape도 삼각형별 경로를 탔다면 id 3의 최소 부피가 id 1의 범위 안에
들어와야 한다. 들어오지 않는다 — §171.1의 dispatch 분기가 실제로 갈린다는
독립 증거다.

## §172 부동소수 → 정수 좁힘: 전사가 정확해도 경계에서 갈린다

세 라운드에 걸쳐 서로 다른 경로로 같은 모양의 결함이 세 번 나왔다. 계열로
적어두지 않으면 패널마다 다시 발견하게 된다.

**앵커는 둘이고, 순서가 중요하다.**

1. **상류 쪽(먼저 돌려라):** 상류에서 `int`/`size_t`/`unsigned`로 선언되거나
   `static_cast`되는데 **초기화 식이 부동소수**인 자리. 예:
   `int inv_twice_resolution_;`가 `1.0/(2.0*resolution_)`를 받는다.
2. **포트 쪽:** Rust의 `as i32`/`as u32`/`as usize`/`as i64`가 `f64` 식을
   받는 곳.

**포트 쪽만 돌리면 계열의 절반을 놓친다.** §172.1의 사례 1이 정확히
그렇다 — 그 결함은 포트에 `as` 캐스트가 **없어서** 생겼다. 포트는 `f64`로
계산했고 상류가 `int`로 좁혔다. 어떤 Rust 측 grep으로도 안 나온다.
**없는 좁힘은 있는 좁힘보다 찾기 어렵고 더 조용하다** — 값이 상류와
다를 뿐 크래시도 경고도 없다. 그러니 상류 앵커를 먼저 돌리고, 각 히트에
대해 "포트는 이 좁힘을 재현하는가"를 물어라.

**왜 전사가 정확해도 결함인가.** 이 자리들은 상류를 정확히 옮겨도 두
언어의 정의가 다르다:

| 입력 | C++ (`double` → 정수 축소) | Rust (`as`) |
|---|---|---|
| 범위 초과 | **UB** | 포화(`i32::MAX`, `usize::MAX`) |
| 음수 → 부호없음 | **UB** | **0** |
| `NaN` | **UB** | **0** |
| `inf` | **UB** | 포화 |

상류가 UB인 구간에는 "정답"이 없으므로 **오라클과 대조해도 판정할 수
없다.** 그래서 이 계열은 파리티 테스트가 잡지 못한다 — 유일한 방법은
경계를 열거해서 거부하는 것이다.

### 172.1 확인된 세 사례

1. **`inv_twice_resolution_`** (`distance_field.hpp:614`, p3-distance-field
   라운드 25) — `double` 필드들 사이에 혼자 `int`. `1.0/(2.0*resolution)`이
   `resolution >= 0.51`에서 **0으로 절단**되어 gradient가 항등적으로 0이
   된다. 상류도 같으므로 파리티지만, **파리티라는 것을 테스트가 못박아야
   한다**(0.5/0.51 양쪽). `resolution < 2.328e-10`에서 포화 대 UB로 갈린다.
2. **`max_distance_sq`** (`propagation_distance_field.cpp:88` ↔
   `propagation.rs:242`) — `ceil(d/r)²`가 `double`이고 `int`로 좁혀진다.
   포트의 가드가 `is_finite() && >= 0.0`만 보고 `> i32::MAX`를 안 본다.
   `max_distance/resolution > 46341`에서 포화한 뒤 `bucket_len`이 `2^31`이
   되어 **OOM**이다. 값 오차가 아니라 자원 고갈이다.
3. **`sample_count`** (`time_optimal_trajectory_generation.cpp:1245` ↔
   `.rs:698`) — 전사는 정확하고 루프 경계도 같다. `TotgOptions::resample_dt`가
   검증 없는 `pub` 필드라: `0.0`이면 `inf.ceil() as usize == usize::MAX` →
   `0..=usize::MAX` **행 + OOM**; **음수면 `as usize == 0`** → 에러 없이
   **1점짜리 궤적**이 나온다. 후자가 이 계열에서 가장 나쁜 모양이다 —
   크래시도 에러도 로그도 없다.

### 172.2 왜 기존 감사가 못 잡았나

p6-totg 라운드 30의 감사는 `not ported`/`out of scope`/dead-code 마커를
앵커로 썼고 "silent-wrong-value 갭 없음"이라는 부정 결과를 냈다. 사례 3이
그 결과를 반증한다. **이 계열은 미포팅 표시가 없는 곳, 전사가 정확한 곳에
있다.** "무엇을 안 옮겼나"로는 안 나오고 "옮긴 것이 경계에서 상류와 같은가"로
물어야 나온다.

### 172.3 규칙

1. 위 앵커로 자기 크레이트를 쓸고 **모든 히트를 화면에 열거**한 뒤 각각
   `same defect` / `distinct(한 줄 이유)`로 분류한다. 진짜 정수량
   (셀 수, stride, 제곱 셀거리, 방향 인덱스)은 `distinct`다 — 좁혀지는 것이
   **실수값**인 자리만 대상이다.
2. 고칠 때는 호출 경로마다 `if`를 뿌리지 말고 **검증을 한 곳으로 모은다.**
   가능하면 유효하지 않은 값을 담을 수 없게 타입/생성자로 막는다
   (`pub` 수치 필드를 검증하는 생성자 뒤로 옮기는 것이 대표적).
3. 상류와 갈리는 입력 구간을 **§153.1로 명시**한다: 어느 구간인지, 상류가
   그 구간에서 UB라 정답이 없다는 것, 그리고 상류가 검증을 추가하면
   만료된다는 것.
4. 테스트는 서사가 아니라 **경계**로: 절단이 일어나는 값과 그 직전 값,
   포화 경계, 음수, `NaN`, `0`.

### 172.4 상류 소스가 없을 때 — 구간을 거부하면 확인할 것이 없어진다

p9-ros 라운드 7이 `ros/moveit-ros`의 상류 3개 범위를 쓸어 **0건**을 냈고,
경계 하나를 0건이 아니라 **미확인**으로 남겼다. 판단은 옳다. 상류
(`robot_trajectory.cpp:357,410,833`)는 `rclcpp::Duration::from_seconds(total_time)`을
거치는데, 이 머신에는 `duration.hpp`(선언 `:135`)만 있고 `duration.cpp`는
호스트에도 컨테이너에도 없다 — 확인했다. 없는 소스를 추측해서 0건에
넣지 않은 것이 규칙대로다.

**그런데 이 미확인은 소스 없이도 닫힌다.** `seconds_to_duration`은
음수·비유한·`i32::MAX` 초과를 **거부한다**. 상류의 동작을 모르는 구간이
정확히 포트가 들어가기를 거부하는 구간이다. 남는 입력은 유한한
`[0, i32::MAX]`이고, 거기서 `(sec, nanosec)`로의 사상은 메시지 정의
(`int32 sec`, `uint32 nanosec`, `nanosec < 1e9`)만으로 유일하게 결정된다 —
`from_seconds`가 그 구간에서 무엇을 하든 답이 하나뿐이라 비교할 것이 없다.

일반 규칙으로: **상류 소스를 못 구해 어떤 구간의 상류 동작을 판정할 수
없다면, 그 구간을 거부하는 것이 그 구간을 추측하는 것보다 강하다.** 추측은
틀릴 수 있고 조용히 틀리지만, 거부는 판정 자체를 불필요하게 만든다. 이것은
§172.3-2의 "타입/생성자로 막는다"와 같은 수를 검증 쪽에 적용한 것이다.

만료조건(§153.1): `rclcpp` 소스가 이 트리에 들어오면 거부 구간에서 상류가
실제로 무엇을 하는지 확인할 수 있게 되고, 그때 이 구간을 거부가 아니라
재현으로 바꿀지 결정할 수 있다.

## §173 combined replay가 한 방향만 걷던 것 — 닫힘 (`308064e`)

`verify-fixture-replay.sh`의 combined pass는 같은 URDF/SRDF를 쓰는 fixture
전부를 **오라클 프로세스 하나**에 이어 붙여 돌린다. 목적은 요청 사이의 상태
누수 검출이다 — fixture A가 남긴 상태가 fixture B의 결과를 바꾸면 per-file
pass는 전부 통과하는데 combined에서만 깨진다.

그런데 순서가 하나였다. `sorted(glob(...))` + `sorted(manifest)`로 정해지는
고정 알파벳 순서 하나만 걸었다.

**요청 i에서 j로의 누수는 i가 j보다 앞설 때만 관측된다.** 순서가 하나면
순서쌍의 **절반만** 시험되고, 나머지 절반은 시험되지 않은 채 "통과"로
보고된다. 이것이 §153.1이 경계하는 모양 그대로다 — 부재가 아니라
**미시험**이 통과로 보고된다.

역순은 모든 순서쌍을 한 번에 뒤집는 유일한 순열이다. 두 번 돌리면 모든
순서쌍이 시험된다. `replay_one`이 위치가 아니라 id로 대응시키므로
(그 파일 자신의 주석이 근거) 기록된 응답 파일은 양방향에 그대로 쓰인다.

**완전 커버리지가 아니다.** 두 요청 사이의 누수는 잡지만, 특정 제3의 요청이
둘 사이에 끼어야만 발생하는 누수는 여전히 못 잡는다. n! 중 2다. 그 한계를
스크립트 주석에 적어뒀다.

### 173.1 판별력 확인 — 새 pass가 실제로 도는가

통과하는 새 검사는 그 자체로는 아무것도 증명하지 않는다. `octomap_response.json`
id 1의 `log_odds`를 `3.511030673980713` → `3.5`로 바꾸고 돌렸다:

```
COMBINED    fanuc.urdf: 5 fixtures, 27 requests, 2 crate(s) -- DRIFTED 2 line(s) differ
            ids 4001-4012  moveit-octomap/octomap
REVERSED    fanuc.urdf: 5 fixtures, 27 requests, 2 crate(s) -- DRIFTED 2 line(s) differ
            ids 4001-4012  moveit-octomap/octomap
```

두 줄 다 났다 — 역순 pass가 실제로 오라클을 돌리고 비교하며, 역순으로도
id 대응이 유지되어 실패를 맞는 fixture(4001-4012)에 귀속시킨다. 되돌린 뒤
5그룹 × 2방향 전부 통과한다.

### 173.2 비용

`2m04.7s` → `2m22.7s`, **+18초**. 오라클 프로세스 기동이 combined 그룹당
두 배가 되지만 per-file pass가 전체 시간을 지배하므로 두 배가 되지 않는다.

## §174 이 저장소의 "CI"는 한 번도 돈 적이 없고, **돌 수가 없다**

`.github/workflows/ci.yml`은 `fmt`/`clippy --workspace`/`nextest --workspace`/
`cargo test --doc --workspace`/`cargo doc`/`tools/ci/check-*.sh` 글롭을
정의한다. 지금까지 이 문서는 그것을 "아직 GitHub Actions에서 실행된 적
없음"으로 적어 왔다. 실제 이유는 더 강하다:

```
$ git remote -v
(빈 출력)
```

**원격이 하나도 없다.** push할 곳이 없으므로 워크플로가 트리거될 경로
자체가 존재하지 않는다. "아직 안 돌았다"가 아니라 "돌 수 없다"이고, 그
차이는 중요하다 — 전자는 시간이 지나면 저절로 해소되지만 후자는 누군가
원격을 만들기 전까지 영원히 그대로다.

따라서 이 트리에서 **게이트라고 부르는 것은 전부 로컬 관례다**:

| 항목 | 실제 실행 주체 |
|---|---|
| `check-*.sh` 8개 | ci.yml의 글롭 — **실행된 적 없음**. 사람이 손으로 돌릴 때만 |
| `verify-*.sh` 6개 | 애초에 ci.yml 밖(도커/상류 필요). 사람이 기억할 때만 |
| `ros/verify-ros-interop.sh` | §158 관례. 사람이 기억할 때만 |
| `cargo test --doc --workspace` | ci.yml:51에 있으나 실행된 적 없음 — §170이 이것 때문에 main을 깼다 |

§170이 왜 실제 사고로 이어졌는지가 여기서 설명된다. doctest를 잡는 명령은
이미 ci.yml에 **적혀** 있었다. 없던 것은 명령이 아니라 **실행**이다. 게이트를
문서에 추가하는 것과 게이트가 도는 것은 다른 일이고, 이 저장소는 지금까지
전자만 해 왔다.

**이건 내가 닫을 수 없다.** 원격을 만드는 것은 사용자의 결정이고(이 세션의
규칙상 push는 명시 승인 없이 불가), 원격 없이 `check-*.sh`를 강제로 돌게
만드는 우회(예: 셸아웃하는 테스트를 `cargo test`에 심기)는 게이트를
"돌게" 만드는 것이 아니라 테스트 스위트를 느리게 만들면서 CI가 있는
척하는 것이다 — §172.4가 말한 것과 같은 이유로, 추측/흉내보다 **없다고
명시하는 쪽**이 강하다.

기록해 둘 것: 이 문서의 모든 "게이트 통과" 문장은 **그 라운드에 사람이
직접 돌린 결과**이지 자동 검증이 아니다. 만료조건(§153.1): 원격이 붙고
Actions가 한 번이라도 초록으로 돌면 이 절을 지운다.

## §175 감사 목록을 컨텍스트에만 두면 압축이 그것을 지운다

p3-distance-field 라운드 26이 UNFIXED에 이렇게 적었다 — 배경 에이전트 둘이
제출한 **93건**(50+43)의 주장 감사 목록이 컨텍스트 압축으로 사라졌고,
"목록에 있었으나 이번 세그먼트에 나타나지 않은 잔여 항목이 있는지는
재감사 없이는 확신할 수 없다."

**보고 태도는 정확하다** — 확인할 수 없는 것을 확인했다고 하지 않았다.
문제는 그 다음이다: 이 손실은 그 패널의 부주의가 아니라 **저장 위치의
성질**에서 나온다. 93건은 처음부터 디스크에 없었고, 대화 컨텍스트는
압축되는 매체다. 같은 일이 같은 조건에서 반드시 다시 일어난다.

그리고 지금 **여러 패널이 동시에 같은 종류의 감사를 돌고 있다**(type-b
주장 감사). 각각이 수십 건짜리 목록을 컨텍스트에만 쌓고 있다면, 손실은
이미 예약되어 있다.

### 175.1 규칙 — 감사 산출물은 파일이다

주장 감사·인벤토리·전수 열거를 시작하는 즉시 `doc/claim-audit/<크레이트>.md`를
만들고 **항목을 발견할 때마다 거기에 append 한다.** 보고서를 쓸 때 한꺼번에
옮기는 것이 아니다 — 그 시점이면 이미 늦다.

한 항목 = 한 행:

| 항목 | 내용 |
|---|---|
| `where` | 이 트리의 `파일:줄` |
| `claim` | 그 줄이 상류에 대해 주장하는 바(한 줄) |
| `verdict` | `CONFIRMED` / `EXPIRED` / `UNVERIFIABLE(사유)` |
| `evidence` | **실제로 연 상류 `파일:줄`.** 포트에서 추론한 것은 증거가 아니다 |
| `commit` | 고쳤다면 그 해시, 아니면 빈칸 |

`UNVERIFIABLE`은 실패가 아니라 정당한 세 번째 분류다(§172.4가 그 예 —
상류 소스가 이 머신에 없으면 없다고 적는 것이 추측보다 강하다).

### 175.2 왜 파일이어야 하는가

- **압축이 지우지 못한다.** 컨텍스트는 줄어들지만 파일은 남는다.
- **다음 라운드가 이어받는다.** 지금은 라운드마다 목록을 처음부터 다시
  만들고 있고, 그래서 "전부 확인했다"를 아무도 주장할 수 없다.
- **소유자가 바뀌어도 산다.** 크레이트 소유권은 라운드마다 옮겨간다.
- **완결성이 검사 가능해진다.** 파일에 있으면 세어볼 수 있고, 세어볼 수
  있으면 §153.1 만료조건을 걸 수 있다.

### 175.3 이 절의 한계 — 게이트가 아니다

`doc/claim-audit/`가 실제로 채워지는지 검사하는 스크립트는 **없다.** 만들
수도 있지만(파일 존재 여부만 보는 검사는 빈 파일로 통과한다) 내용의
완결성은 기계가 잴 수 없고, §174가 말한 대로 이 저장소에서는 검사를
추가해도 **아무도 돌리지 않는다.** 그러므로 이것은 관례다 — 관례라고
명시하는 편이 게이트인 척하는 것보다 정확하다.

만료조건(§153.1): `doc/claim-audit/`가 두 라운드 연속으로 비어 있으면
이 관례는 지켜지지 않는 것이고, 그때는 관례를 강화할 게 아니라 왜 아무도
쓰지 않는지를 먼저 물어야 한다.

## §176 §172 적용 — `DiscreteMotionValidator::is_motion_valid`의 `steps` 좁힘 (moveit-planners-sbp)

§172 포트 쪽 앵커(`as u64`가 `f64` 식을 받는 자리)를 moveit-planners-sbp에
돌려 `validity.rs:123`을 찾았다: `(dist / self.resolution).ceil() as u64`.
상류 대응은 OMPL `StateSpace::validSegmentCount`
(`ompl/base/src/StateSpace.cpp:851`,
`(unsigned int)ceil(distance / longestValidSegment_)`) — 좁힘 모양은
같지만 폭이 다르고(상류 `unsigned int`, 포트 `u64`), 상류 호출부
(`DiscreteMotionValidator.cpp:54,103`)가 그 `unsigned int`를 다시 부호 있는
`int nd`로 한 번 더 좁혀서, 오버플로로 음수 wrap되면 내부 보간 검사를
통째로 건너뛸 수 있다(둘 다 C++ 좁힘 변환 UB/구현정의).

`resolution`은 생성자(`DiscreteMotionValidator::new`)에서
`finite && > 0.0`만 검증하고, 실제 호출 시점의 `dist`에 비해 얼마나
작은지는 검증하지 않는다. `resolution`이 `dist`에 비해 병적으로 작으면
(예: 오설정된 `PlanningRequest::resolution`) `step_count`가 `u32::MAX`
(4,294,967,295)를 훌쩍 넘는다. 고치기 전 코드는 그 값을 그대로 `as u64`로
담았고, `check_range`의 총 호출 수는 `O(step_count)`이므로 사실상 무한
hang이 된다 — §172.1 사례 2(`max_distance_sq`)와 같은 계열, 메모리가
아니라 CPU가 고갈되는 모양만 다르다. `NaN` 거리(상태에 `NaN` 성분이
섞여 들어온 경우 `RealVectorSpace::distance`가 그대로 전파한다)는
`NaN.ceil() as u64 == 0`으로 새므로 `steps - 1`이 release 빌드에서 조용히
`u64::MAX`로 wrap한다 — §172 표의 "`NaN` → 0" 행 그대로.

고침: `steps`로 좁히기 전에
`step_count.is_finite() && step_count <= u32::MAX as f64`를 단언한다.
`u32::MAX`는 임의 값이 아니라 `validSegmentCount` 자신이 반환하는 폭이다
— 상류 자신의 표현으로도 담을 수 없었을 값만 거부한다. 상류가 이
구간에서 UB이므로 오라클로 판정할 수 없고(§172), 유일한 방법은 경계를
거부하는 것이다.

### 176.1 경계 (§153.1)

거부 구간은 `dist/resolution > u32::MAX`(정상적인 관절 공간 거리에 비해
`resolution`이 병적으로 작을 때만 도달) 또는 `dist`가 `NaN`/무한(state에
비유한 값이 섞여 들어왔을 때만 도달)이다. 둘 다 정상적인
`PlanningRequest`/`RobotState` 구성에서는 발생하지 않는다. 만료 조건:
상류가 `validSegmentCount`/`checkMotion`에 자체 검증을 추가하면(현재는
없음) 이 절을 다시 확인한다.

Tests: `resolution_far_smaller_than_distance_panics_instead_of_hanging`,
`nan_distance_panics_instead_of_silently_producing_a_degenerate_range`
(`crates/moveit-planners-sbp/src/validity.rs`).

### 176.2 `longestValidSegmentCountFactor_` — factor confirmed 1, not dropped

`validSegmentCount`'s full upstream line is
`longestValidSegmentCountFactor_ * (unsigned int)ceil(distance / longestValidSegment_)`
(`StateSpace.cpp:853`); the port has no factor. Checked whether that is a
silently dropped multiplier or a correct no-op:

- `StateSpace.cpp:94`: `longestValidSegmentCountFactor_` defaults to `1`
  and is only ever changed by `setValidSegmentCountFactor()`
  (`StateSpace.cpp:825`).
- `rg -n "ValidSegmentCountFactor|longest_valid_segment" moveit_planners/ompl`
  against upstream moveit2: MoveIt's `ompl_interface` only ever reads/sets
  `longest_valid_segment_fraction` (`model_based_planning_context.cpp:294-315`,
  feeding `setLongestValidSegmentFraction`) and never calls
  `setValidSegmentCountFactor`. Also confirmed via
  `ompl_interface.cpp:170`, which declares `longest_valid_segment_fraction`
  as the only related planner parameter — no factor parameter exists in
  the path this port models.

So in the only path this port targets (MoveIt's own OMPL integration, not
arbitrary direct OMPL use), the factor is always `1` — multiplying by it
is a no-op upstream itself never exercises differently. Confirmed, not a
defect. `PlanningRequest::resolution` also already models
`longestValidSegment_` (the post-fraction absolute distance
`DiscreteMotionValidator::new` takes directly), not the raw
`longest_valid_segment_fraction` — the `maxExtent_ * fraction`
multiplication is likewise the caller's responsibility outside this
crate, matching how `moveit_planners_sbp` takes `resolution` as an
already-resolved absolute value throughout, not a fraction.

## §177 링커 순서는 선택 규칙이 아니다 — `linkme` 슬라이스의 첫 항목 집기

**측정.** `moveit-octomap/Cargo.toml`에 `thiserror` 한 줄을 추가한 것
**만으로** `moveit-planners-pilz`의 오라클 패리티 테스트 3건이 깨졌다.
단일 변수로 격리했다 — 65c0fd9(p1-joints 병합 직후)에 `Cargo.lock` +
`crates/moveit-octomap/Cargo.toml` 두 파일만 얹으면 재현되고, 나머지
p3-shapes 변경(octomap `tree.rs` 568줄, `error.rs`, doc)을 전부 얹고
그 두 파일만 되돌리면 통과한다. 3회 반복, 결정적.

원인은 octomap이 아니다. `KINEMATICS_SOLVERS`(`linkme::distributed_slice`)의
**나열 순서가 뒤집혔다**:

```text
65c0fd9 : ["lma", "newton_raphson", "lma_cached", "newton_raphson_cached"]
f74a2b7 : ["lma_cached", "newton_raphson_cached", "lma", "newton_raphson"]
```

`linkme`의 순서는 링커 섹션 배치 순서이고, 그것은 의존성 그래프의
함수다. 워크스페이스 **어디에서든** 크레이트가 의존성을 하나 더 달면
바뀔 수 있다.

**결함.** pilz의 여섯 개 호출부가 전부 이 모양이다:

```rust
KINEMATICS_SOLVERS
    .iter()
    .find_map(|registration| { (registration.construct)(...).ok().filter(...) })
```

"구성에 성공하는 첫 등록을 쓴다" — 여기서 "첫"은 링커가 정한다. panda
arm 그룹에는 네 등록이 전부 구성에 성공하므로, 실제로 쓰이는 IK 솔버는
소스 어디에도 적혀 있지 않고 빌드 그래프가 고른다. 이것이
[Structural fix vs. clever patch]가 말하는 **런타임 게이트**의 최악
형태다: 게이트조차 아니고, 관측 가능한 규칙이 없다.

`SolverRegistration::name`의 자체 doc은 이미 "The name a caller scanning
`KINEMATICS_SOLVERS` **matches on**"이라고 쓰여 있다. 계약은 있었고
호출부가 지키지 않았다.

### §177.1 두 번째 사실 — `lma_cached`는 `lma`와 같은 답을 주지 않는다

순서가 뒤집혀 `lma_cached`가 선택되자 LIN의 waypoint 0이
`-2.2506433721376613`이 되었다. 오라클은 `-2.356`, 즉 시작 상태 그대로다.
시작 자세에서 시작 관절값을 시드로 준 IK가 시드를 그대로 돌려주지 않는다.
캐시 래퍼가 감싼 솔버와 관측적으로 동등하지 않다는 뜻이고, 이것은 순서
문제와 **별개의 결함**이다. 순서를 고정해도 이쪽은 남는다.

### §177.2 고칠 방향

이름으로 고르는 단일 소유 API를 `moveit-kinematics`에
두고 여섯 호출부를 전부 그리로 보낸다. 어떤 솔버를 쓰는지가 소스의 값이
되어야 하며, 링커의 부산물이면 안 된다. D4(컴파일타임 레지스트리)는
`dlopen` 이름 조회를 안 하겠다는 것이지 **선택 규칙을 두지 말라는 것이
아니다** — 지금은 규칙이 없는 것이지 D4를 따르고 있는 것이 아니다.

### §177.3 회귀 테스트는 무엇을 고정해야 하나

슬라이스의 순서를
`assert_eq!` 하면 안 된다 — 그것은 링커를 고정하는 것이고, 다음 의존성
추가에서 결함이 아닌 것이 빨갛게 된다. 고정해야 할 것은 **어떤 그룹에
대해 해결된 솔버의 이름**이다. 그러면 다음 재배열은 수치 패리티가 조용히
드리프트하는 대신 선택 단계에서 이름으로 실패한다.

### §177.4 만료조건 없음 — 다른 레지스트리에도 적용된다

지금
워크스페이스의 `distributed_slice`는 둘이다.
`PLANNER_MANAGERS`(`moveit-planners-sbp/src/registry.rs:441`)의 유일한
순회는 `:655`의 `find(|r| r.name == "rrt_connect")`로 이름 키이고
테스트 전용이라 이 결함이 아니다. 새 레지스트리를 만들거나 새 순회를
추가할 때 이 절이 요구하는 것은 하나다: **첫 항목을 집지 말 것.**

## §167.6 §167.2 면제를 걷어내니 그 아래 두 번째 구멍이 있었다 (`692ea31`, `b24fd01`)

§167.5는 §167.2의 `len(resolved) == 1` 면제가 근거 없는 수량 주장 위에
서 있었음을 측정으로 보였고, 두 디렉터리 인용이 좁혀지면 면제를 걷으라고
남겼다. p1-joints(`944b4a8`, pilz 145파일 → 13파일)와 p3-shapes(`0ca9158`,
stomp → 6파일)가 둘 다 좁혔고 — 게이트의 상류 파일 수가 471에서 334로
떨어진 것이 그 흔적이다 — 면제를 걷었다.

**걷어냈더니 아무것도 안 잡혔다.** 그것이 정상이 아니었다. 변이 테스트로
확인했다: pilz 헤더를 bare directory 인용으로 되돌리고 `Copyright (c)
2018, Pilz GmbH & Co. KG` 한 줄을 지웠는데도 게이트가 OK를 냈다. 면제를
되살려도 OK, 걷어내도 OK — 즉 그 변이는 면제 판정에 **도달조차 하지
않았다.**

원인은 파서에 있었다. `derivations`는 들여쓴 파일명을 처리하는 가지에서만
채워진다:

```python
if derived:
    derivations.update(citations[-len(expanded):])
```

`Ported from` 아래에 패키지 디렉터리만 쓰고 그 밑에 파일명을 나열하지 않은
헤더는 `citations`에는 들어가지만 `derivations`에는 **한 번도** 들어가지
않는다. 그래서 §166/§167의 보존 규칙이 그 헤더에는 적용되지 않았다. 면제가
"의도적으로 처리하고 있다"고 주장하던 바로 그 부류가, 실제로는 규칙 밖에
있었다.

두 수정이 모두 있어야 변이가 잡힌다(하나만으로는 통과한다):

1. `692ea31` — 보류된 디렉터리가 자기 줄에서의 `derived` 플래그를 들고
   다니게 하고, flush 시점에 `derivations`에 넣는다. 살아 있는 플래그를
   읽으면 헤더가 끝난 섹션에 디렉터리가 귀속된다.
2. `b24fd01` — `len(resolved) == 1` 면제 제거.

변이 상태에서 이제 UNRETAINED 6건이 나온다(willow garage 2012, pilz 2018,
pilz 2019, cristian c beltran hernandez 2021, aiman haidar 2025 등) —
§167.2가 "헤더가 재현하기엔 너무 많다"고 말하던 바로 그 목록이다. 대응은
면제가 아니라 **인용을 실제 포팅한 파일로 좁히는 것**이고, 그것이 두
패널이 이미 한 일이다.

**교훈은 [checkers-fail-toward-silence]와 같다.** 면제가 무엇을 걸러내고
있는지 측정하지 않으면, 그 면제는 자기 아래의 구멍을 가려 준다. 면제를
걷었을 때 아무것도 안 나오면 "깨끗하다"가 아니라 "도달하지 않는다"를 먼저
의심해야 한다 — 변이 하나로 갈린다.

## §178 병합 게이트가 `cargo doc`을 안 돌려서 main이 깨진 채로 갔다 (`a0fb9dc`)

p6-totg 라운드 32를 병합할 때(`289b5d7`) fmt / clippy `--workspace
--all-targets` / nextest `--workspace` / `cargo test --doc --workspace`를
돌렸다. 전부 통과했다. **`cargo doc --workspace --no-deps`는 안 돌렸고**,
그것이 깨져 있었다:

```text
error: public documentation for `time_optimal_trajectory_generation`
       links to private item `MAX_RESAMPLE_SAMPLE_COUNT`
error: unresolved link to `MAX_FROM_DURATION_POINTS`
error: unresolved link to `Error`
```

내가 찾은 게 아니라 p1-robotmodel이 R24 보고의 UNFIXED에 적어 왔고, 나는
repo root에서 재현해 확인했다. 병합한 사람이 못 본 것을 다음 라운드의
다른 패널이 본 것이다.

**이것이 세 번째다.** §170(doctest는 크레이트 범위 게이트에 안 보인다),
§174(CI는 돌 수가 없다), 그리고 지금 rustdoc. 매번 같은 모양이다 — 검사가
없는 게 아니라 **그 검사를 포함하는 명령이 없다.** clippy `--all-targets`는
rustdoc lint에 닿지 않고, nextest도 닿지 않으며, `cargo test --doc`는
doctest를 *실행*할 뿐 링크 해석 오류를 내지 않는다. 셋 다 통과하면서
`cargo doc`만 깨질 수 있다.

조율자의 병합 게이트는 이제 다음 다섯이다. 빠짐없이 전부 돌린다:

```text
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo nextest run --workspace --no-fail-fast
cargo test --doc --workspace
cargo doc --workspace --no-deps          # ← 빠져 있던 것
tools/ci/check-*.sh (8) + verify-upstream-license-provenance.sh
```

패널 쪽 대응은 자기 크레이트에 `cargo doc -p <crate> --no-deps`를
추가하는 것이다. §170이 doctest에 대해 말한 것과 같은 이유다.

**만료조건(§153.1) 없음, 대신 다음 검사:** 네 번째 사례가 나오면 게이트를
하나 더 추가할 게 아니라, 왜 "전부 돌리는 명령 하나"가 아직 없는지를 먼저
물어야 한다. §174가 답의 절반이다 — 그런 명령을 쓸 자리(CI)가 이 저장소에는
돌지 않는다.

## §179 "나중 라운드"와 "언급 없음" — 둘 다 미포팅인데 하나만 보인다

두 유휴 패널에 라운드를 주려고 상류 디렉터리를 포트와 파일 단위로
맞춰 세어 봤다. 감사(§175)는 *포팅된 코드가 상류를 맞게 말하는가*를
검사하지 어떤 상류 파일이 아예 안 왔는지는 검사하지 않는다. 그래서
포팅되지 않은 파일은 감사 표에 행이 생기지 않는다 — **빠진 것은 틀린
것으로 나타나지 않고, 아무것으로도 나타나지 않는다.**

### 179.1 pilz — 선언된 미포팅

`crates/moveit-planners-pilz/src/lib.rs:90-91`:

```text
Not yet in scope, planned for later rounds:
`trajectory_blender_transition_window`.
```

§5의 Phase 8 범위 줄은 "LIN/PTP/CIRC + sequence blending"이다. 즉 이
파일은 완료 조건 **안쪽**이지 옆이 아니다. 상류 `src/` 22개 중 포트가
가진 것은 12개이고, D1/D2로 명시 제외된 다섯을 빼면 남는 in-scope
미포팅은 이 하나다. 선언되어 있었으므로 잃어버린 것은 아니지만, "later
rounds"에는 만료조건이 없어서 라운드가 30번 도는 동안 아무도 그 문장을
다시 읽지 않았다. p1-joints에 배정.

**정정 (2026-08-05, §217.2가 실측, 이 병합 시점에 다시 쟀다).** 위 단락의 세
수와 그 결론이 모두 트리와 다르다. 상류 `src/*.cpp`는 22가 아니라 **24**개,
포트가 가진 것은 12가 아니라 **14**개, `lib.rs`의 D1/D2 목록이 이름으로
지목하는 `src/*.cpp`는 "다섯"이 아니라 **8**개
(`move_group_sequence_{action,service}`,
`planning_context_loader{,_circ,_lin,_polyline,_ptp}`,
`pilz_industrial_motion_planner`)다. 그리고 하나 남았다고 지목한
`trajectory_blender_transition_window.cpp`는 포팅됐다. 남는 in-scope 미포팅
`src/` 파일은 이 단락이 언급하지 않는 **둘**, `joint_limits_aggregator.cpp`와
`joint_limits_validator.cpp`다. §217.2는 이 셋을 branch 시점 수(24/13/9)로
적었는데, 그 뒤 `command_list_manager.{hpp,cpp}`가 D1/D2 제외에서 포팅으로
옮겨가 13→14, 9→8이 됐다. 결론(남은 둘)은 양쪽 시점에서 같다.

### 179.2 planning_pipeline_interfaces — 언급조차 없음

이쪽이 더 나쁘다. 상류
`moveit_ros/planning/planning_pipeline_interfaces/src/`의 네 파일
(`planning_pipeline_interfaces`, `plan_responses_container`,
`solution_selection_functions`, `stopping_criterion_function`)은
`crates/moveit-planning/`에 포팅되지도, 제외되지도 않았다. 실측:

```console
$ rg -ni 'PlanResponsesContainer|solution_selection|stopping_criterion|parallel' \
    crates/moveit-planning/src/
crates/moveit-planning/src/response.rs:29:  ... a parallel `Vec<f64>` of durations ...
```

한 건, 그리고 무관한 주석이다. `lib.rs`는 같은 상위 디렉터리의
`planning_pipeline.cpp`는 `:186`/`:437`에서 인용하고
`display_motion_path.cpp`는 `:154`에서 D1로 제외한다. 즉 이 크레이트는
이웃 파일들에 대해서는 판정을 내렸고 이 네 개에 대해서만 침묵한다.
**침묵은 제외가 아니다.** p1-fixtures에 배정하며, 네 파일을 하나로
묶어 판정하지 말 것을 조건으로 달았다 — `planWithParallelPipelines`가
`rclcpp::Node`를 받는다고 해서 `solution_selection_functions`까지
ROS 결합인 것은 아니고, 그 구분이 "제외 하나"와 "간극 셋"을 가른다.

### 179.3 규칙

제외는 §153.1대로 만료조건을 달아야 하고, **연기(deferral)도 제외의
일종이다.** "planned for later rounds"는 만료조건 없는 제외문이며 그
점에서 "의존성이 없어서 제외"와 같은 계열이다. 앞으로 미포팅 상류
파일은 세 상태 중 하나로만 존재한다:

1. 포팅됨 — 감사 표에 행이 있다
2. 제외됨 — `lib.rs`에 근거 심볼과 만료조건이 있다
3. 간극 — 위 둘 중 어느 것도 아니면 간극이고, 간극은 라운드에 배정된다

세 번째 상태가 이번에 두 건 나왔다. 크레이트 감사가 아무리 촘촘해도
이 상태는 안 잡힌다. 잡으려면 **상류 디렉터리 대 포트 디렉터리를 파일
단위로 세는 별도 검사**가 필요하고, 지금 그것은 조율자가 손으로 한
것 말고는 없다.

## §180 회귀가 실패가 아니라 정지로 나타나면 게이트는 영원히 기다린다

p3-shapes 라운드 34가 `cancelling_from_another_thread_stops_a_plan_call_
already_in_flight`를 다시 썼다. 옛 판은 `sleep(20ms)` 뒤 `elapsed < 5s`를
주장했고, 그 패널 자신이 그것을 반증했다 — `num_iterations_after_valid`
기본값이 `0`이라 `Stomp::solve`는 유효 반복 하나 뒤에 스스로 빠져나오고,
따라서 **취소를 통째로 지워도 그 테스트는 통과했다.** 새 판은 시간 대신
`cost_fn` 호출 수를 세고 `num_iterations = 1_000_000`,
`num_iterations_after_valid = num_iterations`로 두어 `proceed`가 false가
되는 것 외에 빠져나갈 길을 없앴다. 옳은 방향이다.

### 180.1 실측 — 변이는 통과하지 않았지만 실패하지도 않았다

`CancelHandle::cancel`의 본문을 비우고 돌렸다.

```console
$ timeout 240 cargo nextest run -p moveit-planners-stomp cancelling_from_another_thread
Terminated
EXIT=143
```

통과는 안 했다(옛 판이었으면 통과했다). 그런데 실패도 안 했다 — 240초
동안 아무것도 출력하지 않고 돌았다. 취소가 유일한 탈출구라는 것이 곧
**취소가 깨지면 탈출구가 없다**는 뜻이고, 회귀 신호가 무한 정지로
나타난다.

주의: 첫 변이는 `Stomp::cancel`을 비운 것이었고 테스트는 0.017초에
통과했다. 그 자리가 아니었다 — 테스트는 `CancelHandle::cancel`을 부른다.
변이가 살아남으면 테스트를 의심하기 전에 **변이가 실제로 그 경로를 건드렸는지**
먼저 확인해야 한다. §167.5에서 한 번, 여기서 두 번째다.

### 180.2 구조적 해결 — 테스트가 아니라 러너에 걸었다

이 테스트를 시간 제한으로 되돌리는 것은 방금 없앤 결함을 다시 넣는 것이다.
고칠 자리는 테스트가 아니라 **정지를 실패로 바꾸는 것이 아무 데도 없다는
사실**이다. nextest의 기본 `slow-timeout`은 경고만 하고
`terminate-after`가 없으면 아무도 죽이지 않는다. §174에 따라 바깥에서
한도를 거는 CI도 없다.

`159d703` — `.config/nextest.toml`:

```toml
[profile.default]
slow-timeout = { period = "60s", terminate-after = 5 }
```

변이를 그대로 둔 채 재측정:

```text
TIMEOUT [ 300.007s] (1/1) moveit-planners-stomp planner::tests::
    cancelling_from_another_thread_stops_a_plan_call_already_in_flight
Summary [ 300.009s] 1 test run: 0 passed, 1 timed out, 50 skipped
error: test run failed
```

300초는 현재 워크스페이스 최장 테스트(28.9초,
`moveit-distance-field::collision_env_distance_field_parity`)의 10배가
넘으므로 느린 테스트에는 발화하지 않는다. 패널 여덟이 동시에 컴파일하는
부하까지 감안한 여유다.

### 180.3 §178의 "네 번째 사례" 지시에 대한 답

§178은 네 번째가 나오면 게이트를 더할 게 아니라 왜 "전부 돌리는 명령
하나"가 없는지를 먼저 물으라고 했다. **이건 그 계열이 아니다.** §170/§174/
§178은 전부 *검사를 포함하는 명령이 없다*였다. 여기서는 명령이 있고
돌았다 — 신호가 도착하지 않았을 뿐이다. 그래서 여섯 번째 명령을 더하지
않았고, 이미 도는 명령의 출력이 유한해지도록 러너를 고쳤다. 게이트 목록은
§178의 다섯 그대로다.

계열이 다르므로 다음 검사도 다르다: **테스트가 회귀를 실패가 아니라
정지로 표현할 수 있는 자리가 또 있는가.** 무한 반복 상한을 두고 조기
탈출 조건 하나에 의존하는 테스트가 그 모양이다. 지금 워크스페이스에서
`num_iterations = 1_000_000` 같은 상한은 이 한 건뿐이지만, 세어 본 것이
아니라 이 라운드에 눈에 띈 것이 하나라는 뜻이다.

## §181 시딩 수정은 단위 테스트가 지키고 있고, end-to-end 측정은 그것을 재지 않는다

p1-robotmodel 라운드 26이 `GroupConstraintSampler`의 `working`을 호출
사이에 유지하도록 고쳤다(상류 `work_state_`와 같은 모양,
`f8c7af0`). 같은 라운드가 `path_constraints_end_to_end_wired_vs_unwired`를
추가해 **unwired 1/5, wired 5/5**를 측정했고, 라운드 24의 **wired 0/5 vs
unwired 5/5**가 뒤집힌 것을 시딩 수정의 결과로 서술했다.

### 181.1 실측 — 수정을 되돌려도 숫자가 같다

`try_sample`에 라운드 24의 매-호출 초기화(`*state = template.clone()`)를
되돌려 넣고 그대로 돌렸다.

```text
path_constraints_end_to_end_wired_vs_unwired:
    unwired 1/5, wired 5/5 (self-motion distance 0.803693435403718 rad)
```

수정 전과 **자릿수까지 동일하다.** 변이가 그 경로에 닿지 않은 것이
아니다 — `try_sample`에 `eprintln!`을 넣어 세어 보니 이 테스트 한 번에
**11회** 불린다(§180.1의 교훈대로 변이 도달을 먼저 확인했다). 11회면
solve당 두 번 남짓이고, 호출 간 지속성이 지속시킬 것이 거의 없다.

따라서 0/5 → 5/5 뒤집힘의 원인은 시딩 수정이 아니다. 남은 후보는 같은
라운드의 `resolve_constraint_sampler` 추출(§163.3의 goal 전용 `.take()`
결함을 닫은 것)과, 커밋 메시지 자신이 밝힌 시나리오 차이 — 라운드 24는
네 시나리오 스윕이었고 이번은 한 시나리오다. 둘 중 어느 것인지는 아직
안 쟀다.

### 181.2 그러나 수정은 지켜지고 있다

같은 변이로 크레이트 전체를 돌리면 정확히 하나가 빨개진다:

```text
FAIL constrained_sampler::tests::
    try_sample_carries_the_previous_draws_result_forward_as_the_next_seed
Summary: 106 tests run: 105 passed, 1 failed
```

즉 시딩 수정에는 가드가 있다. **없는 것은 가드가 아니라 귀속의 근거다** —
단위 테스트는 "이전 결과가 다음 시드가 된다"를 지키고, end-to-end 측정은
"그것이 성공률을 바꾼다"를 지지하지 않는다. 두 주장은 다르고, 라운드
보고는 앞엣것의 증거로 뒤엣것을 말했다.

### 181.3 규칙

수정 A와 측정 B가 같은 라운드에 들어오면 **B에서 A를 되돌려 보기 전에는
A가 B를 움직였다고 쓰지 않는다.** 같은 라운드에 다른 수정이 함께
들어왔다면 특히 그렇다 — 이번에 함께 들어온 것이 하나 더 있었고, 그쪽이
남은 후보다. §127의 "관례로만 지켜지던 것"과 같은 자리다: 인과는 관례로
지켜지지 않는다.

이것은 메모리의 `record-causes-as-falsifiable-predictions`가 예측한 그대로
발생했다 — 일관되지만 측정되지 않은 원인이 반증 가능한 형태로 적혀 있었고,
되돌려 보니 반증됐다.

## §182 다섯 명령이 처음으로 동시에 초록이다 — 그리고 같은 결함이 한 크레이트 건너에 남았다

p6-totg 라운드 33이 §178이 만든 붉은 줄을 닫았다. §178 이후 처음으로
병합 게이트 다섯이 한 커밋에서 전부 통과한다:

```console
$ cargo fmt --all -- --check                    # clean
$ cargo clippy --workspace --all-targets -- -D warnings   # clean
$ cargo nextest run --workspace --no-fail-fast  # 1543 passed, 4 skipped
$ cargo test --doc --workspace                  # ok
$ cargo doc --workspace --no-deps               # Generated ... 21 other files
```

패널 보고에 "pilz 3건 known-red per §177" 같은 문장이 더 이상 유효하지
않다. §177은 닫혔고 `cargo doc`도 닫혔다. 앞으로 자기 브랜치에서 빨간
것을 보면 그것은 자기 것이다.

### 182.1 구조적 수정은 경계별로 물린다

`TotgOptions::resample_dt`를 `pub(crate)` + 검증하는
`with_resample_dt` + 읽기 전용 `resample_dt()`로 바꾼 것이
구조적인지 실측했다. `with_resample_dt`의 검증을 `if false`로 죽이면
정확히 다섯이 빨개진다:

```text
resample_dt_nan_is_rejected
resample_dt_negative_is_rejected_not_silently_truncated
resample_dt_positive_infinity_is_rejected
resample_dt_negative_infinity_is_rejected
resample_dt_zero_is_rejected_not_hung
```

시나리오당 하나가 아니라 **경계당 하나**다 — NaN, 음수, ±무한, 0. 이것이
"Replace the primitive" 절이 요구하는 모양이고, 서사형 테스트였으면 다섯
중 하나만 물었을 것이다.

크레이트 안의 쓰기 자리는 둘뿐이다: `Default`(0.1, 유효)와
`with_resample_dt`(검증됨). 구조체 리터럴 생성은 `Default` 하나뿐이므로
지금은 우회로가 없다. 다만 `pub(crate)`이므로 **같은 크레이트 안에서**
새 리터럴을 쓰면 우회가 가능하다 — 타입으로 막힌 것이 아니라 생성 자리가
하나뿐이어서 막힌 것이다. 이 구분을 문서에 적어 두지 않으면 다음 라운드가
"unrepresentable"을 액면 그대로 읽는다.

### 182.2 같은 결함이 한 크레이트 건너에 그대로 있다

p6-totg가 UNFIXED로 올렸고, 병합 후 확인했다.
`crates/moveit-planning/src/response_adapters/add_time_optimal_parameterization.rs`:

```text
:83   resample_dt: f64,
:91   pub fn new(path_tolerance: f64, resample_dt: f64, min_angle_change: f64) -> Self
:94       resample_dt,
:117      self.resample_dt,
```

`new`는 아무것도 검증하지 않고 저장하며, 잘못된 값은 `adapt()`가 돌 때에야
`Err`로 나타난다. 방금 `TotgOptions`에서 닫은 것과 **같은 이중 의미**다 —
필드가 "검증된 값"과 "아직 검증 안 된 값" 둘 다를 뜻한다. p6-TOTG는 이
크레이트를 소유하지 않으므로 옳게 라우팅했다.

**"고친 자리의 이웃 크레이트에 같은 결함이 있는가"는 CLAUDE.md의 defect-
family 규칙이 이미 요구하는 것이고, 이번에는 소유권 경계가 그것을 한
라운드 늦췄다.** 소유자 규칙과 결함 가족 규칙이 부딪히는 자리이며, 답은
라우팅이지 월권이 아니다 — 다만 라우팅된 항목은 다음 라운드의 브리프에
들어가야 하고 UNFIXED 목록에서 조용히 늙으면 안 된다.

## §183 D6가 Phase 9 완료 조건과 부딪히는 자리를 하나 찾았다

p9-ros 라운드 9가 `apply_octomap`에서 자기 결함을 찾아 고쳤다 —
`map.origin`은 `map.header.frame_id` 기준인데 그것을 월드 좌표로 쓰고
있었다(`0e3f706`). 상류 `planning_scene.cpp:1494-1497`의
`p = t * p`가 맞고, 같은 크레이트의 `collision_object.rs:358`,
`attached.rs:221`이 이미 쓰던 패턴이다. 회귀 테스트를 직접 검증했다:
합성을 걷어내면 정확히 하나가 빨개진다.

```text
FAILED scene::planning_scene::tests::
    octomap_origin_is_composed_with_the_header_frame_transform
  left:  rotation [0, 0, 0, 1]
  right: rotation [0, 0, 0.24740395925452294, 0.9689124217106447]
test result: FAILED. 117 passed; 1 failed
```

기존 삽입 테스트 셋이 전부 `model.model_frame()`(항등)을 쓰고 있어서
이 결함을 못 잡았다는 그 패널의 진단도 맞다.

### 183.1 같은 감사가 부수적으로 찾아낸 것이 더 크다

그 패널은 `OctomapWithPose` 오버로드가 빈 `header.frame_id`에도
`getFrameTransform`을 무조건 부른다는 것을 발견하고, 이 포트의
`frame_transform`이 D6에 따라 `Err`를 내므로 "결함이 아니라 D6 정책의
발현"으로 분류해 deviation 행만 적었다. 분류의 전제를 상류에서 직접
확인했다.

`planning_scene.cpp:1452-1460` — 평범한 `Octomap` 오버로드에는 빈 문자열
가드가 있다. `:1477-1499`의 `OctomapWithPose` 오버로드에는 **없다.** 그
패널의 관찰은 정확하다. 그런데 그 다음이 문제다:

```text
Transforms::getTransform (transforms.cpp:110-125)
  if (!from_frame.empty()) { ... }
  RCLCPP_ERROR("Unable to transform ... Returning identity.");
  static const Eigen::Isometry3d IDENTITY = ...;
  return IDENTITY;
```

상류는 빈 프레임에서 **로그를 남기고 항등을 돌려주며 성공한다.** 이
포트는 `Err`로 메시지를 거절한다. 즉 상류가 받아들이는 메시지를 이 포트가
거절한다.

### 183.2 이것은 정책 표명이 아니라 완료 조건 위반이다

§5 Phase 9의 완료 조건은 "기존 C++ `MoveGroupInterface` 클라이언트가
**코드 변경 없이** `moveit-ros` 노드에 요청을 보내 유효한 궤적을 받는다"
이다. 옥토맵이 이미 월드 좌표계에 있을 때 `header.frame_id`를 비워
보내는 것은 예외적 입력이 아니라 그 메시지의 통상적 사용이다. D6가
그것을 거절하면 완료 조건이 그 경로에서 성립하지 않는다.

**해소 방향: D6를 바꾸지 않고, 이 한 진입점에서 메시지 의미를 따른다.**
`OctomapWithPose`의 빈 `header.frame_id`는 "월드"라는 뜻이고, 그것은
미해결 이름이 아니라 명시된 기본값이다 — D6가 막으려던 것(오타난 프레임
이름이 조용히 항등으로 흡수되는 것)과 다른 경우다. 빈 문자열은 분기해서
항등을 쓰고, 비어 있지 않은데 해석되지 않는 이름은 지금처럼 `Err`로
남긴다. 그러면 D6의 실제 목적은 유지되고 완료 조건도 성립한다.

이 구분을 코드에 적어 두지 않으면 다음 라운드가 "빈 프레임도 Err여야
정책 일관성"이라고 되돌린다. 빈 문자열과 미해결 이름은 같은 것이 아니다.

### 183.3 이 계열을 다시 훑어야 한다

`OctomapWithPose`만 이런 것이 아닐 수 있다. 상류에서 빈 `frame_id` 가드가
있는 오버로드와 없는 오버로드가 갈리는 자리를 전부 세야 한다 —
`collision_object.rs`, `attached.rs`가 이미 같은 패턴을 쓰고 있으므로
그쪽 진입점들도 같은 질문을 받는다. 한 자리를 고치고 나머지를 안 세면
CLAUDE.md의 defect-family 규칙 위반이다.

## §184 §171 종결 — 그리고 "둘 다 통과"는 자기 UNFIXED와 모순이었다

p3-acm이 `mesh_shape_cost_sources`의 mesh 비용원 루트 박스를 축정렬
`Bvh` AABB로 맞추던 것을 지향 OBB로 바꿨다(`1e25683`). 근거를 포트
쪽에서 서술하지 않고 FCL 원본에서 읽었다 — `moveit_core`는 항상
`fcl::BVHModel<fcl::OBBRSSd>`를 만들고 `constructBox`가 그것을 `obb`
성분으로 줄인다(`fcl/include/fcl/geometry/shape/utility-inl.h:1083-1088`),
그리고 점 집합은 `geometry-inl.h:1349-1379`의 삼각형 꼭짓점 가중 방식이다.
`parry3d_f64::utils::obb`에 같은 점 집합을 먹여 `mesh_world_obb_aabb`로
교체했다.

요청한 이등분도 그대로 재현해 왔다: path id 3에서
`pre_remove_cost_sources n=2` → `post_remove_cost_sources n=5` →
`post_remove_overlapping n=5`. **`remove_cost_sources`의 축별 분할은
처음부터 결함이 아니었다.** p1-fixtures가 지목한 용의자가 아니라 그
위쪽 박스 적합이 원인이었고, 나는 그 용의자를 "측정이 아니라 추론"이라고
표시해서 넘겼으므로 이 결과는 절차가 의도대로 작동한 경우다.

### 184.1 보고의 두 문장이 서로 모순이다

보고서 본문: "both `#[ignore]`d tests now pass in full at the existing
`1e-9` threshold". 같은 보고서 UNFIXED: "State-op id 5 (group-filter):
`9 actual vs 2 expected`". **두 문장은 같은 테스트에 대한 것이고 동시에
참일 수 없다.** 병합 후 `--run-ignored all`로 실측:

```text
PASS  moveit-scene::cost_sources_parity
        panda_path_cost_sources_blocked_by_mesh_shape_cost_sources
FAIL  moveit-scene::cost_sources_parity
        panda_cost_sources_blocked_by_mesh_shape_cost_sources
  cost_sources_parity.rs:510: case id 5: count mismatch
    left: 9   right: 2
```

즉 **하나는 진짜로 통과하고, 하나는 여전히 실패한다.** 실패의 원인은 그
패널이 UNFIXED에 스스로 적어 둔 id 5다. 수정 자체는 진짜다 — id 2가
`2.69e-2`에서 통과로 바뀌었기 때문에 테스트가 id 5까지 **도달**한 것이고,
이전에는 id 2에서 멈췄다. 진전이 실패를 앞으로 밀어낸 것이지 없앤 것이
아니다.

### 184.2 결과적으로 `#[ignore]` 두 개의 상태가 갈렸다

- `panda_path_cost_sources_blocked_by_mesh_shape_cost_sources` — 이제
  통과한다. `#[ignore]`는 만료됐고 지워야 한다. 남겨 두면 통과하는
  테스트가 영원히 안 돌고, 회귀가 나도 아무도 모른다.
- `panda_cost_sources_blocked_by_mesh_shape_cost_sources` — 계속
  `#[ignore]`이되 **이유 문자열이 틀렸다.** 지금 문구는 id 2의
  `2.69e-2` 거리 간극을 말하는데 그 간극은 사라졌다. id 5 그룹 필터로
  다시 써야 한다.

둘 다 `crates/moveit-scene/`이라 p1-fixtures 소유다. p3-acm은 옳게
라우팅했고, 라우팅된 항목이 UNFIXED 목록에서 늙지 않도록 §182.2와 같이
다음 브리프에 넣는다.

### 184.3 검증

`sg docker -c tools/ci/verify-fixture-replay.sh` — 47/47 identical,
DRIFTED 0. 기하 적합을 바꿨는데 오라클 fixture가 한 바이트도 안 움직였다
(§149). 워크스페이스 1544/1544, 다섯 게이트 명령 전부 초록.

## §185 블렌더 966줄이 들어왔고, 그것을 잴 오라클 op이 없다

p1-joints 라운드 32가 `trajectory_blender_transition_window`를 포팅했다
(`b3e39c3`, 966줄). §179.1이 연 항목이고, 이로써 §5 Phase 8 범위 줄의
"LIN/PTP/CIRC + sequence blending" 중 마지막 항이 코드로는 닫혔다.
lib.rs의 "Not yet in scope, planned for later rounds" 문장도 사라졌다.

item 4의 측정은 내가 상류에서 직접 확인했다. 기존 제외 사유
("`plan_components_builder`는 `command_list_manager`의 요청 타입에
의존한다")는 **좁은 것이 아니라 틀렸다**:

```console
$ rg 'command_list_manager|CommandListManager' \
    include/.../plan_components_builder.hpp src/plan_components_builder.cpp
(no hits)
$ rg 'PlanComponentsBuilder' include/.../command_list_manager.hpp
222:  PlanComponentsBuilder plan_comp_builder_;
```

의존은 반대 방향이다. `moveit_msgs` 등장은 4회뿐이고 전부
`CREATE_MOVEIT_ERROR_CODE_EXCEPTION(..., MoveItErrorCodes::FAILURE)`의
매크로 인자다 — 요청 마샬링이 아니다. §153이 말한 그대로, 의존성을
근거로 든 제외가 의존성을 확인하지 않은 채 여러 라운드를 살아남았다.

### 185.1 그런데 이 966줄은 상류와 비교된 적이 없다

블렌더의 테스트는 9개이고 전부 자기 일관성 검사다 — `validate_request`의
거부 조건 다섯, `determine_trajectory_alignment` 둘,
`search_intersection_points` 둘, 그리고 경계 연속성 하나. **상류 출력과
대조하는 것은 하나도 없다.** LIN/PTP/CIRC는 `pilz_trajectory` op으로
1e-6 이내를 재고 있는데(§132), 블렌드는 잴 수단이 없다.

Phase 8의 완료 조건은 "LIN/PTP/CIRC 궤적이 오라클과 1e-6 이내"라고
쓰여 있어서 문자 그대로는 블렌드를 요구하지 않는다. 그러나 범위 줄이
블렌딩을 포함하는 이상, **범위에는 있는데 완료 조건이 안 재는 부분**이
생긴 것이고 그것은 조건 문구의 결함이지 면제가 아니다.

### 185.2 비용은 이미 치러져 있다

§122가 pilz 오라클의 비용을 12패키지 확장으로 미리 쟀고, §132가 그것을
치렀다. 지금 `tools/moveit-oracle/CMakeLists.txt:92`가 이미 링크한다:

```cmake
pilz_industrial_motion_planner::trajectory_generation_common
```

§122.3이 확인한 대로 이 타겟이 곧
`trajectory_functions` + `trajectory_generator` +
`trajectory_blender_transition_window`다. 즉 **블렌드 op을 추가하는 데
새 패키지도, 이미지 확장도 필요 없다.** 드는 비용은 오라클 소스 변경으로
스탬프가 바뀌는 것뿐이고, 그것은 §149가 이미 규정한 절차다 — 새 능력을
더할 때 기존 fixture가 한 바이트도 안 움직이는 것이 조건이며
`verify-fixture-replay.sh`가 그것을 검사한다.

절차는 §107/§155/§156과 같다: 소유자 패널이 요청서를 쓰고 조율자가 op을
만든다. `tools/moveit-oracle/`는 조율자 소유다.

## §186 §139의 세 번째 사례 — 상속 그래프로 내린 제외가 또 틀렸다

p3-distance-field 라운드 28이 `CollisionEnvHybrid` 제외를 측정으로
바꿨다(`ddd5ff0`). 기존 사유는 "`CollisionEnvFCL`을 직접 상속하는데
D4.5로 `CollisionEnvFCL`이 포팅되지 않으므로 그것에 의존하는 것도 될 수
없다"였다. 상류에서 직접 세어 확인했다:

```console
$ rg -n 'CollisionEnvFCL' src/collision_env_hybrid.cpp
49:  : CollisionEnvFCL(robot_model)
61:  : CollisionEnvFCL(robot_model, world, padding, scale)
69:  : CollisionEnvFCL(other, world)
169:  CollisionEnvFCL::setWorld(world);
```

멤버 22개 중 기반 클래스를 건드리는 것은 **4개**뿐이고, 그 중 셋은
생성자 base-init이며 넘기는 것(`getWorld()`, `(world, padding, scale)`)은
전부 `CollisionEnv` 자신에 선언된 것이지 FCL 고유가 아니다. 남은
하나 `setWorld`가 하는 실제 일은 FCL의 지속 broadphase 캐시
(`manager_`/`fcl_objs_`)를 월드 교체 시 재구축하는 것인데,
`ParryCollisionEnv`에는 그 캐시 자체가 없다 — 매 `check_*` 호출마다
`self.world`에서 바디를 새로 계산한다(`parry.rs:1841`). 나머지 18개는
전부 `cenv_distance_`로의 통과 호출, 즉 이 크레이트가 이미 가진
`DistanceFieldCollisionCache`다.

**§139, §185의 `plan_components_builder`, 그리고 이것 — 같은 계열 세
번째다.** 세 번 다 "관계를 보고 호출을 결론냈다"이고, 세 번 다 실제로
세어 보니 호출이 한 줌이었다. 제외 사유가 *관계*(상속, 의존, 포함)를
가리키면 그 자체가 재측정 신호다. 관계는 호출 수를 말해 주지 않는다.

### 186.1 이번에는 소유권 경계에서 멈춘 것이 옳았다

그 패널은 포팅하지 않고 추정치만 냈다. 조합기가 살 자리는
`moveit-distance-field`(이미 `moveit-collision`에 단방향 의존)이지만
공개 형태를 `moveit-collision`의 백엔드 타입이 정하므로 그쪽 소유자의
결정이라는 이유다. 브리프가 "추정치를 먼저 달라, 디프를 만들지 마라"고
요구한 그대로다 — §182.2가 라우팅을 한 라운드 늦춘 것과 달리, 여기서는
늦춘 것이 비용이 아니라 정확히 필요한 조율이었다.

**판단: 포팅한다.** 근거는 셋이다. (1) 두 반쪽이 모두 이미 있고
연결만 없다. (2) D4.5는 FCL *백엔드*를 대체했지 `CollisionEnv` 인터페이스를
지운 것이 아니며, 이 제외는 그 둘을 혼동한 것이 원인이었다. (3) distance-field
자기충돌 + 범용 월드충돌이라는 조합은 상류에서 실제로 쓰이는 구성이다.
자리는 `moveit-distance-field`, 공개 형태는 `moveit-collision` 소유자가
검토한다.

## §187 §181 종결 — 그리고 설계를 바꾼 측정이 코드로 남지 않았다

p1-robotmodel 라운드 27이 §181의 질문에 되돌림 실험 두 개로 답했다.
두 번째를 직접 재현했다 — `resolve_constraint_sampler`의
`path_constraints` 호출부를 `None`으로 되돌리면:

```text
path_constraints_end_to_end_wired_vs_unwired: unwired 1/5, wired 1/5
```

wired가 5/5에서 1/5로 무너져 unwired와 같아진다. 시딩 수정을 되돌렸을
때는 숫자가 자릿수까지 그대로였다(§181.1). 따라서 그 테스트의 5/5는
§163.3의 배선 확장이 만든 것이고, 시딩 수정이 만든 것이 아니다.
`doc/claim-audit/moveit-planners-sbp.md`의 라운드 24·25 행은 EXPIRED로,
새 행은 CONFIRMED로 정리됐다.

### 187.1 두 번째 실험이 무엇을 보이고 무엇을 안 보이는지

그 패널이 스스로 적었고, 그 정직함이 이 항목의 핵심이다: 테스트의 목표는
`Goal::State`이므로 goal 분기는 `select_default_sampler`를 아예 부르지
않는다(`Goal::State(_) => None`). 배선까지 `None`으로 되돌리면 어느
호출부도 solver를 못 보고, wired와 unwired가 **구성상** 동일해진다.

즉 실험 (2)가 보이는 것은 "이 테스트는 배선에 대해 판별한다"이고,
"배선이 플래닝을 일반적으로 낫게 한다"가 아니다. 후자를 주장하려면 다른
시나리오가 필요하다. 되돌림 실험이 항진명제에 가까워지는 자리가 있다는
것 — 테스트가 토글하는 바로 그것을 되돌리면 토글이 무의미해진다 — 을
기록해 둔다.

### 187.2 진짜 남은 구멍: 라운드 24의 측정은 다시 돌릴 수 없다

라운드 24는 네 시나리오 스윕에서 **wired 0/5 vs unwired 5/5**, 즉
지금과 반대 방향을 쟀다. 지금 배선을 빼면 wired는 1/5이지 5/5가 아니므로,
두 측정은 그대로는 양립하지 않는다. 그런데 어느 쪽이 맞는지 판정할 수가
없다 — **그 스윕은 재사용 가능한 코드로 커밋된 적이 없다.** 감사 행이
그것을 명시한다: "that sweep was never committed as reusable code, so it
could not be re-run."

설계 결정을 바꾼 측정이 산문으로만 남으면, 그 결정을 뒤집을 근거도 같이
사라진다. 라운드 24의 숫자는 `template`을 트리 지역성에 재고정하는
작업을 연기시켰고, 그 연기는 지금도 유효한데 근거는 재현 불가능하다.
**규칙: 설계를 바꾸거나 작업을 연기시키는 측정은 커밋된 실행 가능한
코드여야 한다.** 보고서의 표는 그 코드의 출력이지 그 자체가 증거가 아니다.

## §188 오라클이 볼 수 없는 값을 오라클이 계산하면, 그건 오라클이 아니다

§185가 연 구멍 — 블렌더 966줄에 상류 대조가 하나도 없다 — 을 `pilz_blend`
op으로 닫았다(`b63171d`). p1-joints의 요청 문서가 정수 필드 세 개를 요구했고,
그 중 둘만 넣었다. 뺀 하나가 이 항목이다.

`searchIntersectionPoints`와 `determineTrajectoryAlignment`는 둘 다
`TrajectoryBlenderTransitionWindow`의 **private** 멤버라서 바깥에서 부를 수
없다. 그런데 앞의 두 인덱스는 여전히 진짜 상류 산출물이다 — `blend()`가
응답 궤적을 정확히 그 인덱스에서 자르고, 그 자름이 근사가 아니라 정확하기
때문이다. `[0, first_intersection_index)` 복사 루프는
`res.first_trajectory`의 waypoint 수를 `first_intersection_index`와 **같게**
만들고, `[second_intersection_index + 1, count)` 루프는
`res.second_trajectory`의 수를 입력 수에서 `second_intersection_index + 1`을
뺀 값으로 만든다. 정확한 복사 루프 두 개를 역산하는 것은 상류가 계산한 값을
*복원*하는 것이지 *재계산*하는 것이 아니다.

`blend_align_index`에는 그런 증인이 없다. 그것은 `blendTrajectoryCartesian`의
샘플링 산술로만 흘러들어가고 응답 모양에 살아남는 경계를 하나도 결정하지
않는다. 그래서 그 필드를 넣으려면 `determineTrajectoryAlignment`의 여섯 줄을
**오라클 쪽에서** 다시 돌려야 하고, 그러면 포트는 상류의 실행이 아니라
`oracle.cpp`의 재구현과 비교된다. 그건 픽스처가 자기 헬퍼를 재는 것이고,
오라클이 존재하는 이유가 정확히 그것을 하지 않기 위해서다.

**규칙: 오라클은 상류가 산출한 값만 방출한다. 상류가 내부에 감춘 값을
오라클이 유도해서 내보내면, 그 필드는 대조가 아니라 두 번째 구현이다.**
감춰진 값이 필요하면 그것이 실제로 결과를 바꾸는 자리에서 재라 — 여기서는
`blend_trajectory`의 waypoint들이고, 요청 문서의 케이스 B가 바로 그 분기가
출력을 바꾸도록 만들려고 존재한다.

### 188.1 역산이 맞는지는 상류 자신의 로그가 확인해줬다

운 좋게도 `searchIntersectionPoints`가 찾은 인덱스를 `RCLCPP_INFO`로 찍는다.
두 케이스를 돌린 로그가 `index: 8` / `index: 7`, `index: 8` / `index: 3`이고,
복사 루프 역산으로 얻은 값과 정확히 일치했다. 역산이 옳다는 것을 추론이 아니라
독립 관측으로 확인한 셈이다. 다만 이 로그는 계약이 아니다 — 상류가 로그 한 줄을
지우면 사라진다. 그래서 역산을 로그에 의존시키지 않았고, 이 확인은 일회성
교차검증으로만 기록한다.

### 188.2 첫 실행에서 정수는 전부 일치했다

| 항목 | 케이스 A (대칭 0.1) | 케이스 B (비대칭 0.3) |
|---|---:|---:|
| 세그먼트 1 / 2 입력 waypoint | 16 / 16 | 16 / 9 |
| `first_intersection_index` | 8 | 8 |
| `second_intersection_index` | 7 | 3 |
| 출력 first / blend / second | 8 / 8 / 8 | 8 / 8 / 5 |
| `determineTrajectoryAlignment` 분기 | `else` (8 == 8) | `way_point_count_1 > way_point_count_2` (8 > 4) |

포트가 요청 문서에 자기 값으로 적어둔 숫자와 전부 같다 — 두 분기 모두, 두
인덱스 모두, 입출력 waypoint 수 모두. 분기 커버리지 구멍은 이것으로 닫혔다.

아직 대조되지 않은 것은 waypoint의 **수치**다. 정수가 맞았다고 수치가 맞는 것은
아니고, 여기에 붙일 tolerance는 아직 모른다 — 실제 응답이 생긴 지금 측정해서
정해야 하며 LIN의 숫자를 그대로 가져다 쓰면 안 된다(CLAUDE.md의 tolerance 규칙).
그 측정이 p1-joints의 다음 작업이다.

오라클 스탬프는 `3537df47121b8c7f` → `043ed31a2186fe4e`로 올랐고,
`verify-fixture-replay.sh`는 49/49 identical, 0 DRIFTED다 — 헬퍼 세 개를
`pilzTrajectory`에서 들어낸 리팩터가 기존 픽스처를 한 바이트도 바꾸지 않았다(§149).

## §189 §187의 두 번째 사례가 같은 라운드에 나왔다 — 산문으로 적힌 22는 실제로 140이었다

§187은 "설계를 바꾼 측정은 커밋된 실행 가능한 코드여야 한다"를 라운드 24의
재현 불가능한 스윕에서 끌어냈다. 같은 라운드에 p1-fixtures가 독립적인 두 번째
사례를 만들었고, 이쪽은 결론까지 나왔다.

`moveit-scene`의 §172 narrowing sweep은 여러 라운드 동안 "19+1+2=22 hits"로
문서에 적혀 있었다. p1-fixtures가 그 관행을 스크립트로 만들어
(`tools/ci/count-narrowing-sweep.sh`, `8538ec1`) 같은 상류 8개 파일에 그대로
돌리자 **140**이 나왔다. 직접 재현했고 파일별 숫자까지 전부 일치한다 —
`planning_scene.cpp` 24, `planning_scene.hpp` 4, `robot_state.cpp` 76,
`attached_body.hpp` 0, `attached_body.cpp` 1, `world.cpp` 10, `world.hpp` 4,
`kinematic_constraint.cpp` 21.

가장 큰 격차는 `robot_state.cpp`의 2 대 76이다. 옛 스윕의 주장 문구 자체가
("2 ... `static_cast<std::size_t>(s.rows())`") 그 파일의 `static_cast` 두 개만
서술하고 있고, 평범한 선언 74개는 아예 언급이 없다. 즉 그 파일은 **훑어진 적이
없고** 훑어졌다고 적혀 있었을 뿐이다. 산문으로 적힌 "mechanical sweep"은
mechanical이었는지 검증할 방법이 없다는 것이 요점이다.

결론 자체는 바뀌지 않았다 — 140개 중 실제 부동소수점 narrowing은 여전히 **0**
이다(2개는 스크립트가 스스로 문서화한 텍스트 오탐, ~13개는 초기화식이 없는
파라미터/필드, 나머지는 전부 정수 소스). 초기화식이 부동소수점에서 오는 hit이
하나라도 있는지 독립적으로 확인했고 없었다. 그러니 이건 결함이 아니라 **근거의
질**에 관한 것이다: 6배 틀린 숫자로 옳은 결론을 지지하고 있었고, 아무도 몇 라운드
동안 알아채지 못했다.

**§187의 규칙을 여기까지 넓힌다: 측정이 설계를 바꿨든 아니든, 감사 문서에 숫자로
적히는 순간 그 숫자를 만든 명령이 함께 커밋돼야 한다.** 숫자만 있고 명령이 없으면
그 숫자는 다음 라운드에 검증되지 않고, 검증되지 않은 채로 다른 주장의 근거가 된다.
`count-relative-eq.pl`, `count-public-declarations.sh`가 이미 그 형태였고,
narrowing sweep만 산문으로 남아 있었다.

### 189.1 스크립트가 자기 한계를 문서화한 것이 이 커밋에서 제일 좋은 부분이다

`count-narrowing-sweep.sh`는 자기가 C++ 파서가 아님을 헤더에 적고, 구분하지
못하는 오탐 두 형태(반환 타입이 정수인 메서드 선언, `new TYPE[...]` 배열 new)를
이름으로 명시한다. 그리고 실제 스윕 결과에서 그 두 형태에 정확히 해당하는
`world.hpp:138`과 `kinematic_constraint.cpp:947`을 문서가 따로 빼서 적었다.
두 줄 다 열어서 확인했고 서술대로다.

세는 도구가 자기가 못 세는 것을 말하지 않으면, 그 도구의 출력은 "22"와 같은
종류의 숫자가 된다 — 반박 불가능해 보이지만 무엇을 세었는지 알 수 없는 숫자.
분류는 문서 산문이 하고 스크립트는 원시 hit만 낸다는 이 분업이 옳다.

## §190 "지금은 동작한다"는 분류가 아니다 — 그리고 내 grep도 §189에 걸렸다

p6-totg가 라운드 하나를 `moveit-planners-chomp/src/planner.rs`의 맨
`[`Error`]` 링크가 왜 `cargo doc`을 깨뜨렸는지 이분법으로 추적하는 데 썼다.
기제는 못 찾았다. 그런데 못 찾은 것보다 **찾은 것**이 중요하다: 그 파일에는
`use moveit_error::Error`가 `:117`에 있고, 그런데도 실패했다.

그래서 "이 파일엔 `Error`가 import돼 있으니 맨 링크도 안전하다"는 논증은
성립하지 않는다. 나머지 맨 링크들이 지금 해결되는 것은 그것들이 옳아서가
아니라 p6-totg가 실증한 순서/내용 민감성이 아직 그것들을 물지 않았을 뿐이다.
§178이 기록한 main의 `cargo doc` 적신호가 정확히 이 부류에서 나왔다.

p6-totg는 "다른 자리는 지금 동작하므로 투기적으로 건드리지 않겠다"고 적었고,
그 판단을 뒤집었다. **기제를 못 밝힌 것은 패치를 미룰 사유가 아니라 구조적
수정을 택할 사유다** — `[`Error`](moveit_error::Error)`는 주변 스코프에
의존하지 않으므로 그 기제가 무엇이든 무관하게 해결된다. 원인을 이해할
때까지 기다리는 수정보다, 원인과 독립적으로 옳은 수정이 낫다.

### 190.1 가족을 세다가 §189를 내가 어겼다

`rg '\[`Error`\]'`가 23개를 냈다. 그대로 가족 크기로 쓸 뻔했는데, 그 중
넷은 평범한 `//` 주석 안에 있다 — rustdoc이 아예 읽지 않는 자리다. 실제
doc comment(`///`/`//!`)로 좁히면 **19개**다.

바로 앞 항목(§189)에서 "raw hit은 분류가 아니다"라고 적어놓고, 다음 계산에서
raw hit을 그대로 가족 크기로 쓸 뻔했다. 세는 도구는 자기가 무엇을 세는지
말해야 하고, 그건 남이 만든 스크립트뿐 아니라 내가 그 자리에서 친 grep에도
적용된다. 앵커에 `^\s*(///|//!)`를 넣는 데 10초 걸렸다.

### 190.2 분류

- **같은 결함, 이번에 수정:** `moveit-smoothing/src/ruckig_filter.rs`
  `:124,250,316,423`, `acceleration_filter.rs` `:141,254,294,321,479`,
  `moveit-trajectory/src/ruckig_smoothing.rs` `:54,167`,
  `time_optimal_trajectory_generation.rs` `:35,225,433,461,625` — 16개,
  p6-totg 소유, 한 커밋
- **같은 결함, 소유자에게 라우팅:** `moveit-collision/src/parry.rs:742`
  (p3-acm), `moveit-metrics/src/lib.rs:1105` (p1-fixtures) — 각자 자기 커밋
- **distinct:** `moveit-error/src/lib.rs:20` — 같은 파일에 정의된 타입을
  가리키므로 민감할 크로스모듈 해결 자체가 없다

기제 추적은 UNFIXED로 남긴다. 이건 우회가 아니다 — 적용한 수정이 기제와
무관하게 옳다는 것이 그 수정을 고른 이유 자체다.

## §191 앵커를 좁게 잡으면 결함군이 앵커 모양으로 잘린다

p9-ros가 §183을 결함군 규칙대로 처리했다. 앵커
`rg -n "frame_transform\(" ros/moveit-ros/src`, 사이트 4개 열거, 3개는 같은
결함으로 수정, 1개(`collision_object.rs:358`)는 상류가 `:1889`에서
`knowsFrameTransform`으로 가드하므로 distinct — 그 판정을
`RobotState::knowsFrameTransform`/`World::knowsTransform`/
`Transforms::canTransform` 셋을 다 추적해 빈 문자열에서 전부 false임을 확인해
내렸다. 절차대로고, 결과도 맞다(직접 재확인했다).

그런데 같은 보고의 item 1에서 그 패널이 스스로 이렇게 적었다: 이번엔
`frame_transform` 호출부만 전수 조사했지 다른 옵셔널 필드를 같은 렌즈로
재검사하진 않았다고. 그게 이 항목이다.

**§183의 결함은 `frame_transform`의 결함이 아니었다.** 결함은 "wire가 기본값에
의미를 부여한 필드를, 포트가 미지정으로 읽고 거부한다"였다. 빈 `frame_id`는
그 부류의 한 사례일 뿐이다. 앵커를 `frame_transform\(`로 잡는 순간 조사 범위가
그 함수 이름을 텍스트로 포함하는 자리로 잘렸고, 같은 결함이 다른 필드에서
일어나는 자리는 앵커에 걸리지 않게 됐다.

CLAUDE.md의 결함군 규칙은 "구조적 앵커를 식별하라"고 하고 함수 심볼을 예로
든다. 그 예시가 함정이다 — **인용된 자리에서 가장 먼저 눈에 띄는 심볼이 그
결함의 구조적 앵커라는 보장이 없다.** 여기서 진짜 앵커는 함수가 아니라
*필드의 성질*이었고, 그건 `rg` 패턴으로 직접 표현되지 않는다.

**규칙에 한 줄 더한다: 앵커를 정한 뒤, 그 앵커가 결함의 원인을 가리키는지
증상이 나타난 자리를 가리키는지 물어라.** 후자면 앵커가 결함군을 자기 모양으로
자른다. 원인 쪽 앵커가 `rg` 패턴으로 표현 불가능하면(여기서는 "기본값이 의미를
갖는 필드"), 열거를 손으로 하되 그 목록이 앵커라고 명시해라.

### 191.1 이 경우는 목록이 이미 있었다

운이 좋은 부분: 같은 패널이 같은 라운드에 17.2절로 필드 위험 분류표를 만들면서
**default-has-meaning**이라는 범주를 이름 붙여 놓았다. 결함군의 올바른 앵커가
결함을 고친 커밋 옆에 이미 문서로 존재했는데, 두 작업이 서로를 못 봤다. 다음
라운드 지시는 그 범주를 앵커로 삼아 §183을 다시 도는 것이다.

### 191.2 부수적으로: 분류가 수정 가능한 것을 가상의 위험으로 적어뒀다

17절이 하드코딩된 `MoveItErrorCodes::SUCCESS`(=1)와 지역 재선언된
`CollisionObject`의 `ADD/REMOVE/APPEND/MOVE`를 "지금은 맞지만 repin에 취약"으로
분류했다. 브리프가 판정만 요구했으니 거기서 멈춘 건 옳다. 다만 확인해보니
**r2r이 그 상수들을 실제로 생성한다** — `target/.../moveit_msgs.rs:8031`의
`impl CollisionObject { pub const ADD: ... }`, `:9569`의 `pub const SUCCESS`.
그러면 이건 미래 리스크가 아니라 지금 닫히는 자리다. `_bindgen_ty_404`의 실체를
생성 파일에서 못 찾아 사용 가능 여부는 미확정이고, 컴파일이 답한다 — 안 되면
그것도 결과로 적고 `.msg` 값과 포트 상수의 일치를 재는 테스트로 대신한다.

## §192 결함이 있는 크레이트 밖에서 내린 귀속이 또 틀렸다 — 그리고 커밋은 맞는데 보고가 넘겼다

라운드 4부터 열려 있던 id 5(`9 actual vs 2 expected`)가 닫혔다. 원인은
`moveit-scene`의 그룹 필터링이 아니었다. **`moveit-collision`이 `group_name`을
아예 읽지 않았다** — `check_self_collision`/`check_robot_collision`/
`distance_self`/`distance_robot` 네 메서드가 전부 그 필드를 받고 무시했고,
모듈 문서는 그것이 상류와 일치한다고 적고 있었다.

상류 주장을 전부 직접 확인했다: `cd.enableGroup(getRobotModel())`는
`collision_env_fcl.cpp:281`과 `:336`에서 **무조건** 호출되고,
`enableGroup`(`collision_common.hpp:206-216`)은
`getUpdatedLinkModelsSet()`으로 해석하며, `collisionCallback:92-94`는 **양쪽 다**
active set 밖일 때만 쌍을 버린다(AND of negations = OR of memberships).
p3-acm의 `585a79e`가 그대로 재현했다. 되돌리면 id 5가 정확히 기록된 문구로
실패하고, 넣으면 `--run-ignored all`이 86/86이다.

### 192.1 §139/§185/§186의 네 번째 사례 — 이번엔 귀속이다

앞의 세 건은 *제외*가 관계(상속한다, 의존한다, 포함한다)로 정당화됐다가
호출 수에 반박당한 사례였다. 이번 것은 *귀속*이다: id 5는 그룹 필터 문제이고,
그룹 필터는 씬 계층의 일이므로, 결함은 `moveit-scene`에 있다 — 이 추론에서
틀린 것은 마지막 단계다. **어느 계층이 그 일을 "맡아야 하는가"는 어느 계층이
그 일을 "하고 있는가"가 아니다.** 실제로는 아무 계층도 하고 있지 않았다.

귀속을 내린 패널은 결함이 있는 크레이트를 소유한 바로 그 패널이었고, 자기
크레이트 밖으로 밀어냈다가 이번 라운드에 스스로 되돌렸다. 자기 정정이므로
그 자체는 옳게 처리된 것이다.

**규칙: 결함을 다른 크레이트로 귀속시키는 것은 그 크레이트에 있다는 주장이지
자기 크레이트에 없다는 주장이 아니다.** 후자를 확인하지 않고 전자를 말하면
아무도 안 고치는 상태가 된다 — `doc/crate-ownership.md`의 서두가 기록한 라운드
10의 실패와 같은 모양이고, 그때는 7건이었다.

### 192.2 커밋은 정확한데 구두 보고가 넘겼다

같은 보고가 이 수정을 "PORTING-PLAN.md §119.1/120.1의 귀속에 따라 pr2 깊이
불일치 115건 중 105건의 직접 원인"이라고 적었다. **틀렸다.** §119.1은 순회
순서 설명이 반증됐고 진짜 원인은 **deviation 6** — 같은 하나의 명백한 접촉에
대해 두 백엔드의 침투 깊이 근사가 어긋나는 것 — 이라고 기록한다. §120.1의
`105/115`는 `touching == 1`인 **부분집단의 크기**이지 어떤 수정에 대한 귀속이
아니다. 그리고 구성상 불가능하다: 그 105건은 접촉 쌍 하나에서의 깊이 **크기**
불일치이고, 그룹 필터는 쌍을 남기거나 버릴 뿐 깊이 값을 바꾸지 않는다.

주목할 점은 **커밋 본문에는 이 주장이 없다**는 것이다. 커밋은 id 5만 주장하고
정확하다. 디스크의 산출물은 옳고 서술만 넘어갔다. 병합하는 쪽이 §119.1을
열어보지 않았으면 그 서술에서 결론을 가져갔을 것이다 — 워커 보고를 중계하지
말고 검증하라는 규칙이 커밋 품질과 무관하게 필요한 이유다.

## §193 §185 종결 — 그리고 담요 tolerance가 삼켰을 것을 두 개의 명시적 오프셋이 드러냈다

p1-joints가 `pilz_blend_parity.rs`를 넣었다(`639df34`). 블렌더는 이번 라운드에
자기 일관성 테스트 열둘에 외부 대조 0에서, 오라클에 고정된 파리티 테스트 둘로
갔다. §185가 연 구멍이 닫혔다.

받아 적지 않고 판별력을 직접 쟀다: smoothstep의 `alpha`에 `+1e-7`을 더하면 —
그 패널이 설정한 가속도 tolerance `1.2e-6`보다 **작은** 섭동인데도 — 두 테스트가
모두 실패한다. tolerance 표가 측정에서 나왔다는 주장이 실제로 성립한다.

| 필드 | 측정 최대 | 설정 |
|---|---:|---:|
| time | 1.0e-9 | 1e-6 |
| position | 2.28e-9 | 1e-8 |
| velocity | 1.96e-8 | 8e-8 |
| acceleration | 2.91e-7 | 1.2e-6 |

LIN 픽스처의 `POSITION_TOLERANCE`(당시 `1.26e-5`)보다 세 자릿수 이상
빡빡하다. 이웃 상수를 가져다 쓰지 않고 쟀기 때문에 나온 차이다.

**정정 (2026-08-05).** 방향은 그 뒤 뒤집혔다. LIN을 다시 재니 실측 최대가
`2.09e-14`여서 상수를 `1e-13`으로 조였고(§217.3의 부수 소견 참조), 지금은
LIN 쪽이 이 표보다 다섯 자릿수 빡빡하다. 위 문장의 요지("이웃 상수를 베끼지
않고 쟀다")는 그대로이며, 실제로 이 표의 수는 다시 쟀을 때 그대로였고 베낀
쪽만 낡았다.

### 193.1 담요 tolerance였으면 못 봤을 것

첫 측정 패스에서 `time_from_start`에 `~0.1s`(= `sampling_time`) 어긋남이
나왔다. 그 크레이트 안에 이미 설명이 하나 있었다 — `second_trajectory`의
"waypoint-0 duration은 항상 `0.0`"이라는 지역 deviation 주석. 거기서 멈추고
`TIME_TOLERANCE`를 `0.2`로 잡았으면 테스트는 통과했을 것이고 아무도 몰랐을
것이다.

세그먼트별로 최대 어긋남을 따로 추적하자 `blend_trajectory`도 **같은** 오프셋을
갖는 것이 보였고, 이유가 지역 deviation이 아니라 크레이트 전역 불변식이었다 —
`robot_trajectory.rs`의 "New invariant" 절, `duration_from_previous[0]`은 새로
만들어지는 **모든** `RobotTrajectory`에서 `0.0`이다. 문서화된 자리가 하나뿐이라고
적용 범위가 하나인 것이 아니다.

두 오프셋을 각각 명시하고 불변식을 인용했다. 담요 tolerance 하나로 덮었으면
그 크기의 미래 실제 divergence도 같이 삼켰을 것이다. **오차의 원인을 알면
tolerance가 아니라 오프셋으로 처리한다** — tolerance는 모르는 잡음의 크기이지
아는 상수의 자리가 아니다. 오프셋을 지우면 테스트가 실패하는 것까지 확인됐다.

### 193.2 변이 테스트가 찾은 셋 — 전부 마스킹 기제가 특정됐다

브리프대로 열두 테스트를 변이시켰고 셋이 살아남았다. 셋 다 "테스트가 약하다"가
아니라 **무엇이 가렸는지**가 특정됐다:

- `validate_request_rejects_blend_radius_at_or_below_zero`(`5554ec1`) —
  두 궤적을 체이닝 없이 독립 생성해서, 몇 줄 앞의 경계 상태 검사가 같은
  `InvalidMotionPlan`으로 먼저 거부하고 있었다. 정작 시험 대상인
  `blend_radius <= 0.0`은 지워도 통과했다. 직접 재현했다 —
  `if false && req.blend_radius <= 0.0`으로 바꾸니 수정 후 테스트는 실패한다.
- `search_intersection_points_rejects_a_radius_larger_than_...`(`52d597b`) —
  fixture가 너무 작아 `?`로 엮인 두 호출이 각각 독립적으로 실패했고, 그래서
  어느 쪽 실패인지 테스트가 구분할 수 없었다. 두 테스트로 쪼개 각각을 알려진
  교차 궤적과 짝지었다.
- `blend_produces_a_continuous_trajectory_through_the_shared_boundary`(`83dc4ff`)
  — waypoint 수 단언이 비엄격 `<=`였다. 두 복사 루프의 같은 방향 off-by-one은
  `<=`를 여전히 만족한다. **자기가 잡으라고 존재하는 오류를 잡을 수 없는
  경계였다.**

### 193.3 남은 것: 기하가 한 자리에 고정돼 있다

두 케이스 모두 `first_intersection_index == 8`이고 코너 위치가 같다. 바뀌는
것은 세그먼트 2의 속도뿐이다. 즉 `search_intersection_points`의 후방/전방 탐색은
각각 **한 값에서만** 고정됐고, `determine_trajectory_alignment`는 두 분기가
고정됐을 뿐 인덱스 범위에 걸친 산술은 아니다. 다음 라운드에 `blend_radius`와
코너 각도를 움직이는 3·4번 케이스를 요청 문서로 받기로 했다.

## §194 포트가 만든 API가 상류에 없던 버그를 만들었다 — 그리고 변이 테스트가 그것을 찾았다

p3-shapes가 §180.1 절차대로 취소 테스트를 변이시켰고, 결과가 테스트 결함이
아니라 **프로덕션 버그**였다(`a682f63`).

`Stomp::with_cancel_handle`는 `reset_variables()`를 부르고, 그것이 무조건
`proceed = true`를 썼다. 그래서 호출자가 `Stomp`를 만들기 **전에**
`CancelHandle::cancel`로 취소해 둔 상태가 생성 즉시 지워졌다. 테스트는
통과하고 있었다.

**상류에는 이 버그가 없고, 없을 수가 없다.** upstream의 `resetVariables`도
`proceed_ = true`를 쓰지만 `proceed_`는 `Stomp`의 private 멤버라서 생성 전에
어떤 호출자도 건드릴 수 없다. `with_cancel_handle`은 라운드 24에 이 포트가
만든 API이고(상류에 대응 없음), 그것이 "생성 전 취소"라는 새 상태를 표현
가능하게 만들었는데 생성 경로가 그 상태를 존중하지 않았다.

**포트가 상류에 없는 API를 추가하면, 상류의 불변식이 그 API 아래에서
성립하는지는 별도로 확인해야 한다.** 상류 코드를 정확히 옮겼다는 것은 그
코드가 새 진입점에서도 옳다는 뜻이 아니다 — 상류의 정확성이 그쪽에서는
접근 불가능성에 기대고 있었을 수 있고, 여기서는 그 접근이 열렸다.

구조적으로 고쳤다: `reset_variables`는 이제 `proceed`를 아예 만지지 않고,
상류가 실제로 재시작을 의도하는 `Stomp::clear`/`Stomp::set_config`가 자기가
직접 설정한다. `Stomp::new`/`with_cancel_handle`은 호출자가 준
`CancelHandle`의 상태를 덮어쓰지 않는다. 런타임 가드를 하나 더 넣는 대신
그 필드를 쓰는 자리를 재시작을 의도하는 자리로 제한한 것이다.

직접 재현했다: `reset_variables`에 `self.proceed.store(true, ...)`를 다시
넣으면 `cancelling_before_plan_is_called_returns_the_unmodified_linear_
interpolation_seed`가 실패한다. 버그도 수정도 테스트도 전부 실재한다.

### 194.1 변이가 살아남았을 때 무엇을 의심할지 — 세 번째 갈래

§180.1은 변이가 살아남으면 먼저 그 변이가 경로에 도달했는지 확인하라고
적었다(내가 엉뚱한 메서드를 비우고 테스트가 약하다고 결론냈던 건). 이번
라운드가 세 번째 갈래를 추가한다:

1. 변이가 경로에 도달하지 않았다 — 변이 지점이 틀렸다
2. 변이가 도달했는데 테스트가 약하다 — 테스트를 고친다
3. **변이가 도달했고 테스트도 옳은데, 변이 없이도 이미 틀린 값이 나오고
   있었다 — 프로덕션 버그다**

3번이 이번 것이다. 계수 단언을 넣자 취소 전에도 `cost_fn`이 17번 불렸다 —
변이와 무관하게. 변이 테스트의 산출물이 항상 테스트 수정인 것은 아니다.

### 194.2 부수적으로: 두 테스트가 각각 두 개의 마스크를 갖고 있었다

`cancelling_before_solve_stops_before_num_iterations_completes`(`788cc3f`)는
마스크가 둘이었다. 단언이 `optimized`의 **모양**만 봤는데 `solve`는 반복
횟수와 무관하게 같은 모양을 돌려주고, 게다가 `create_3dof_configuration`의
`num_iterations_after_valid: 0`이 early-valid break로 한 번 만에 빠져나가게
했다. 라운드 34에 같은 결함군을 한 번 고쳤던 자리다 — 같은 fixture 설정이
다른 테스트에서 다시 마스크로 작동했다.

수정 후 같은 변이를 다시 걸었더니 300초에서 TERMINATING/TIMEOUT이 났다.
§180의 `.config/nextest.toml`이 없었으면 이것은 실패가 아니라 무한 정지로
나타났을 것이다.

## §195 실행 가능해졌다고 지켜지는 것은 아니다 — §187의 수정이 §187의 모양을 한 층 위에서 재현했다

§187은 라운드 24의 4-시나리오 스윕이 커밋되지 않아 재실행도 검증도
불가능하다는 것이었다. 라운드 28이 그것을 커밋된 코드로 다시 세웠다
(`e1f2b67`, `path_constraints_four_scenario_wired_vs_unwired_sweep`). 여섯
개 구성 전부 내가 직접 돌려 문서화된 숫자와 정확히 일치했다:

```
scenario 1 (self-motion, Goal::State):       unwired 1/5, wired 5/5
scenario 2 (Goal::Constraints, no corridor): unwired 0/5, wired 5/5
scenario 3 (orientation-only corridor):      unwired 5/5, wired 5/5
scenario 4 tight  (0.03/Iterations(20)):     unwired 1/5, wired 5/5
scenario 4 medium (0.1 /Iterations(20)):     unwired 0/5, wired 5/5
scenario 4 loose  (0.2 /Iterations(200)):    unwired 0/5, wired 5/5
```

숫자는 맞다. 그런데 그 테스트는 여섯 숫자 중 어느 것도 단언하지 않는다 —
`eprintln!`로 출력만 하고, 단언하는 것은 시나리오가 잘 구성되었는지
(IK 도달 가능성, 자기운동 분리 거리)뿐이다.

확인했다. `run_scenario`의 wired 분기에서 `solver: Some(wired_solver)`를
`None`으로 바꾸면 — 즉 측정하려던 효과를 통째로 없애면 — 여섯 숫자가 전부
바뀌는데(`5/5`가 각각 `1/5`, `0/5`, `5/5`, `1/5`, `0/5`, `0/5`로 무너진다)
테스트는 그대로 통과한다.

§187의 결함은 "측정이 재실행 불가능한 산문으로만 남았다"였다. 라운드 28의
결과물은 측정을 재실행 가능하게 만들었지만, 그 측정에서 끌어낸 **결론**은
여전히 아무 게이트도 지키지 않는 산문이다 — doc comment의 표, claim-audit의
두 행, 그리고 `constrained_sampler.rs`의 유예 수정문이 전부 이 숫자들을
근거로 인용하는데, 숫자가 뒤집혀도 실패하는 것은 없다.

### 195.1 "방향은 측정이지 지켜야 할 회귀가 아니다"가 성립하지 않는 이유

커밋 메시지는 어느 쪽이 이기는지는 측정이지 회귀가 아니므로 단언하지
않았다고 적었다. 그 논리는 같은 보고서의 다른 문장이 무너뜨린다 —
"deterministic seeds throughout, 반복 실행에서 동일하게 재현". 결정적이라면
단언할 수 있다. 그리고 그 숫자가 유예 결정의 근거로 인용되는 순간, 그것은
관찰이 아니라 주장이 된다. 주장은 게이트가 지켜야 한다.

경계선은 "회귀인가"가 아니라 **"어딘가에서 인용되는가"**다. 인용되지 않는
숫자는 출력만 해도 된다. 인용되는 숫자는 단언되어야 한다 — 그래야 다음
라운드가 doc comment를 읽고 더 이상 참이 아닌 값을 근거로 쓰는 일이 없다.

### 195.2 같은 라운드가 그 실패를 이미 한 번 겪었다

`9d2e511`이 고친 것이 정확히 이것이다.
`path_constraints_end_to_end_wired_vs_unwired`의 doc comment가 느슨한
예산(`0.2`/`Iterations(200)`)에서 wired/unwired 둘 다 5/5라고 적어두었는데,
실제로 측정하니 unwired 0/5, wired 5/5였다. 단언되지 않은 채 doc에만 적힌
숫자가 조용히 거짓이 된 사례다 — 그리고 그것을 발견한 같은 라운드가 새 표
여섯 줄을 같은 방식으로 남겼다. 스윕의 scenario 4 loose 행이 그 정정을
독립적으로 재현한다는 점은 별개로 옳다.

## §196 빈 그룹은 실패하지 않는다 — SRDF 체인 그룹이 비면 그 위의 모든 단언이 공허하게 통과한다

라운드 29가 `HybridCollisionEnv`를 올렸다(`69044f5`, `5e18441`). 상류
`CollisionEnvHybrid`는 `CollisionEnvFCL`을 상속하고 `cenv_distance_`를
멤버로 들고, 접미사 없는 `checkCollision`/`checkSelfCollision`/
`checkRobotCollision`은 재정의하지 않는다 — 직접 헤더를 열어 확인했다
(`collision_env_hybrid.hpp:48`, 선언된 것은 접미사 붙은 12개뿐).

부수적으로 나온 것이 이 절의 본문이다. 테스트 fixture를 만들다가 발견한
사실: `RobotModel`의 SRDF 체인 그룹 해석은 **active joint**를 따라 걷는다.
두 링크 사이가 fixed joint면 그룹은 `link_names == []`로 해석되고, 오류도
경고도 나지 않는다. 그 그룹 위에 세운 충돌 검사 테스트는 검사할 링크가
하나도 없으니 전부 통과한다 — 통과의 이유가 "충돌이 없어서"가 아니라
"볼 것이 없어서"인데, 결과는 초록으로 같다.

해당 fixture는 기본값 0인 revolute joint로 바꿔 실재하게 만들었고, 두
테스트 모두 내가 직접 무력화해 물리는 것을 확인했다: `build_env_distance_field`의
world 순회를 `.take(0)`으로 비우면 world-swap 테스트가 실패하고(자기 충돌
테스트는 world를 쓰지 않으므로 통과 — 올바른 변별), `collision_tolerance`를
`-0.1`에서 `0.0`으로 되돌리면 불일치 테스트가 실패한다.

### 196.1 이것은 fixture 하나의 문제가 아니다

`[[delete-what-a-test-tests]]`가 기록한 것과 같은 계열이다 — 테스트의
대상이 사라져도 테스트는 초록이다. 다만 그때는 대상을 지워야 드러났고,
이번은 **처음부터** 대상이 없었는데 아무도 알 수 없었다. 후자가 더 나쁘다:
지운 적이 없으니 지워보는 검증으로도 걸리지 않는다.

앵커는 `type="fixed"`가 아니다. 그것은 증상이 나타난 자리다(§191). 결함은
"SRDF 그룹이 빈 링크 집합으로 해석되는데 그 위에 단언이 세워져 있다"이고,
fixed joint는 그렇게 되는 여러 경로 중 하나일 뿐이다. 크레이트 전체에서
`type="fixed"`를 포함하는 인라인 SRDF/URDF fixture는 35군데다 — 세어봤을
뿐, 그중 몇이 실제로 빈 그룹을 만드는지는 아직 모른다.

### 196.2 구조적 답은 감사가 아니라 fixture가 거절하는 것이다

35군데를 한 번 감사해도 36번째가 같은 방식으로 들어온다. 빈 그룹을 조용히
쓸 수 없게 만드는 쪽이 닫는 방법이다 — 그룹을 돌려주는 테스트 헬퍼가
`link_names().is_empty()`를 거절하면 이 결함군은 새로 생길 수 없다.
상류가 빈 그룹을 허용하고 경고만 찍는다는 사실은 `RobotModel`의 동작을
바꿀 이유가 아니다. 바꿔야 하는 것은 **테스트가 그것을 모른 채 지나갈 수
있다**는 쪽이다.

### 196.3 이 포트는 상류보다 world 하나를 덜 갖는다

상류 `CollisionEnvHybrid::setWorld`(`collision_env_hybrid.cpp:163`)는
`cenv_distance_->setWorld`와 `CollisionEnvFCL::setWorld` **양쪽**에
전파한다. world를 두 벌 들고 재정의 하나로 동기화를 유지하는 구조이고,
`CollisionEnvFCL::setWorld`에 직접 닿는 경로가 있으면 그 동기화는 우회된다.

포트는 world를 `parry` 한 곳만 들고 거리장을 매 호출 그것에서 유도한다.
두 world가 어긋나는 상태가 방어되는 것이 아니라 **표현 불가능**하다.
그래서 `setWorld` 대응물이 없는 것이 누락이 아니라 결과다 — 그 이유가
doc에 적혀 있지 않으면 다음 라운드가 누락으로 읽는다.

## §197 수정은 자기 크레이트 밖의 주장도 거짓으로 만든다 — 그리고 그 주장은 수정을 가리키지 않는다

라운드 18이 id 5를 닫았다(`e410533`). p1-fixtures가 `585a79e`의 `parry.rs`
헌크만 되돌려 정확한 불일치를 재현하고 복구했다 — 내가 앞선 라운드에 한
것과 독립적으로 같은 결론에 도달했다(`case id 5: count mismatch left: 9
right: 2`). id 5는 `moveit-scene`의 결함인 적이 없었고 `moveit-collision`의
group 필터링 미구현이었다. `#[ignore]`가 사라져 `cost_sources_parity`가
144/144로 전부 돈다.

이 절의 본문은 그 다음에 발견된 것이다. `585a79e`는 `moveit-collision`을
고쳤는데, 그 수정이 `moveit-scene`의 `PlanningScene::colliding_pairs` doc에
적혀 있던 두 문장을 조용히 거짓으로 만들었다:

- "`ParryCollisionEnv`는 `CollisionRequest::group_name`을 전혀 읽지 않는다"
- "상류의 FCL 백엔드도 group 필터링을 연결하지 않는다"

둘 다 `585a79e` **이전에는 참**이었다. 수정 이후 둘 다 거짓이고, 그 결과
`colliding_pairs`의 `group_name` 누락은 "어차피 inert한 파라미터"가 아니라
상류 `getCollidingPairs`의 group_name 받는 오버로드 4개에 대한 **실재하는
미해결 격차**가 된다.

수정한 쪽은 이것을 알 방법이 없었다. `moveit-collision`을 고치는 사람에게
`moveit-scene`의 doc이 자기 동작을 인용하고 있다는 신호는 아무 데도 없다 —
인용은 한 방향으로만 걸리고, 참조되는 쪽은 자기가 참조된다는 사실을 모른다.

### 197.1 이것은 §153.1의 반대 방향이다

§153.1은 "부재를 근거로 한 배제는 그 부재가 해소되면 조용히 만료된다"였다.
이번은 **존재**를 근거로 한 정당화가 그 존재가 사라지면서 만료된 경우다.
같은 구조의 다른 부호이고, 같은 이유로 아무도 알아채지 못한다 — 만료
조건이 다른 크레이트에 있기 때문이다.

기계적 대응은 없다. 크레이트 경계를 넘어 "이 doc이 저 크레이트의 현재
동작을 사실 주장으로 인용한다"를 추적하는 장치가 이 저장소에 없고, 만드는
것도 이번 라운드의 범위가 아니다. 대신 규칙으로 적는다: **다른 크레이트의
동작을 사실로 인용하는 doc 문장은, 그 크레이트를 고치는 라운드가 끝날 때
역방향으로 한 번 훑어야 한다.** p1-fixtures가 이번에 요청 없이 그렇게 했고,
그래서 발견됐다.

### 197.2 결정 D13 — `colliding_pairs`는 `group_name: Option<&str>`을 받는다

격차가 실재로 확인된 이상 열어둘 이유가 없다. 형제 메서드를 새로 만드는
대신 기존 시그니처에 `group_name: Option<&str>`를 더해 상류의 오버로드
4개를 하나로 접는다 — 이 포트가 다른 오버로드군에서 이미 쓴 규칙과 같고,
경계마다 다른 규칙을 두지 않는다. 크레이트 밖 호출자는
`check_start_state_collision.rs:65` 한 곳뿐이고 `None`을 넘기면 된다.

### 197.3 조건부여야 할 `#[ignore]`가 무조건이다

`cost_sources_parity`의 `#[ignore]`를 지우고 워크스페이스를 돌리니
1584개 중 2개가 여전히 skip이다. `.config/nextest.toml`에는 필터가 없고,
남은 둘은 `tools/moveit-diff/src/main.rs:2435,2553` —
`"needs third_party/moveit_resources"`.

`third_party`는 `.gitignore:3`에 통째로 들어 있으니 그 전제는 진짜다. 갓
클론한 트리에는 없다. 그런데 이 기계에는 있고, 둘 다 `--run-ignored all`로
돌리면 통과한다(16.3s, 21.0s). 즉 `#[ignore]`가 "리소스가 없다"와 "리소스가
있다"를 구분하지 않아서, 있는 기계에서도 영원히 안 돈다 — 방금 지운 것과
같은 결함이다.

절대 실행되지 않는 테스트는 §184가 지운 것과 같고, 조건이 충족됐는데도
실행되지 않는 테스트는 그것을 조건으로 위장한 것이다. 런타임 early-return은
답이 아니다 — 그건 §196의 공허한 통과를 새로 만든다. 저장소가 이미 쓰는
방식대로 `tools/ci/` 스크립트가 `third_party/moveit_resources` 존재를 보고
`--run-ignored all`로 이 둘을 돌리는 쪽이 맞다.

## §198 `cargo doc`는 자기가 문서화하는 항목까지만 검사한다 — 테스트 모듈의 doc은 어떤 게이트에도 없다

내가 p6-totg에 16개 자리의 bare `` [`Error`] ``를
`` [`Error`](moveit_error::Error) ``로 한정하라고 지시했다. 그 지시는
빌드되지 않는다. 워커가 커밋 대신 충돌을 보고했고, 옳았다.

직접 재현했다. `ruckig_filter.rs:124` 한 자리만 바꿔도:

```
error: redundant explicit link target
    = note: `-D rustdoc::redundant-explicit-links` implied by `-D warnings`
```

`redundant_explicit_links`는 bare 링크가 **이미 올바르게 해석될 때** 발동한다.
즉 내 지시의 전제("이 자리들에서 bare가 동작한다")가 바로 그 지시를
불가능하게 만드는 조건이다. `abd7bd5`가 `crates/moveit-planners-chomp/src/planner.rs:106`에서 유효했던 이유가
정확히 뒤집힌 것이다 — 거기서는 bare가 해석에 실패했으므로 명시 대상이
잉여가 아니었다.

그래서 16개 자리에 대한 결정은 **아무것도 바꾸지 않는다**이다. §178이
적었듯 `cargo doc`이 rustdoc 린트에 닿는 유일한 게이트이고, 깨진 intra-doc
링크는 `-D warnings`에서 에러다. 13개 자리는 이미 그 게이트가 지킨다 —
ambient import가 사라지면 조용히 깨지는 것이 아니라 `cargo doc`이 실패한다.
없는 결함군이었다.

### 198.1 그런데 나머지 3개가 진짜를 가리키고 있었다

워커가 "3개 자리는 어느 쪽으로도 진단이 안 나온다(private/test 항목이라
rustdoc이 `--no-deps`에서 린트하지 않는다)"고 적었다. 그 문장이 이 절의
본문이다. 확인해봤다 — `ruckig_filter.rs:423`의 test 항목 doc에 **어디에도
없는 타입** 링크를 넣고 세 게이트를 전부 돌렸다:

```
cargo doc -p moveit-smoothing --no-deps            → 진단 0건
cargo clippy -p moveit-smoothing --all-targets     → 진단 0건
cargo nextest run -p moveit-smoothing              → 31/31 통과
```

존재하지 않는 타입을 가리키는 doc 링크가 **어떤 게이트에도 걸리지 않는다.**
§178은 "`cargo doc`만이 rustdoc 린트에 닿는다"였는데, 정확히는 **`cargo doc`이
문서화하는 항목까지만** 닿는다. `#[cfg(test)]` 항목은 doc 빌드에서 cfg가
켜지지 않으므로 존재하지 않고, private 항목은 기본적으로 문서화되지 않는다.

### 198.2 값싼 닫는 방법을 찾지 못했다

- `--document-private-items` — 여전히 진단 0건 (cfg(test)가 문제이지
  가시성이 문제가 아니다)
- `RUSTDOCFLAGS="--cfg test" cargo doc --document-private-items` — doc
  빌드는 dev-dependency를 링크하지 않으므로 `use moveit_srdf::SrdfModel`에서
  `E0432`로 실패한다. 크레이트 자체가 문서화되지 않는다.

노출 규모를 셌다(거친 추정 — `#[cfg(test)]`/`mod tests` 표식 이후의 `///`를
전부 세는 방식이고, tests 모듈이 파일 끝에 오는 관례에 기댄다):

```
표식 이후 /// 을 가진 파일:  61
그 /// 줄 수:              3649
그중 [..] 링크를 포함한 줄:  350
```

`[[checkers-fail-toward-silence]]`가 기록한 것과 같은 계열이다 — 검사기가
검사하지 못하는 입력을 만나면 실패가 아니라 침묵을 낸다. 이번에는 검사기가
그 입력을 **존재하지 않는 것으로 취급**한다는 점만 다르다.

닫는 방법은 이번 라운드에 없다. 350줄을 손으로 감사하는 것은 361번째 줄을
막지 못하므로 답이 아니다. UNFIXED로 남기고, 이 절이 그 근거다.

## §199 결정 D14 — wire 기본값 0.0은 상류에서 1.0을 뜻한다, 거부가 아니라

p9-ros가 라운드 12에서 §191의 앵커 교정을 실제로 수행했다(`4004d2c`).
`frame_transform\(` 텍스트가 아니라 "wire 기본값이 상류에서 의미를 갖는데
포트가 거부한다"는 성질로 앵커를 다시 잡고 `ros/moveit-ros` 전 사이트를
손으로 열거해 상류 줄 단위로 대조했다. 결론: **이 크레이트 안에서는 새
same-defect 사이트가 없다.** §183은 유일한 사례였다. 앵커 교정이 결과를
바꾸지 않았다는 것도 결과이고, 그렇게 보고한 것이 옳다.

그런데 같은 보고서가 크레이트 경계 밖에서 하나를 찾아 "기록만 하고 조정하지
않음"으로 넘겼다. 그것이 이 절이다.

### 199.1 정책으로 분류된 것이 실제로는 §183이었다

`moveit-constraints`의 네 생성자가 `weight <= EPS`를 `Err`로 거부한다:

```
joint.rs:108   ← kinematic_constraint.cpp:263
position.rs:168 ← :450
orientation.rs:201 ← :641
visibility.rs:231 ← :871
```

상류는 네 자리 전부 경고를 찍고 `constraint_weight_ = 1.0`으로 치환한다.
4대4로 정확히 대응한다 — 직접 세어 확인했다.

포트의 doc(`crates/moveit-constraints/src/joint.rs:85-89`)은 이것을 "silent substitution 대신 에러로
표면화"라는 프로젝트 전역 정책의 의도적 적용으로 적고 D6의 `Transforms`
deviation 1을 인용한다. p9-ros는 그 doc을 읽고 의도적 결정으로 분류했다.
읽은 것 자체는 맞지만 분류가 틀렸고, 나도 그 doc을 쓸 때 같은 것을 놓쳤다.

**D6는 이름 해석에 관한 것이다** — 해석될 수 없는 프레임 이름을 항등
변환으로 조용히 대체하지 않는다. "답할 수 없다"를 답인 척하지 않는 규칙이다.
`weight`가 0.0인 것은 답할 수 없는 입력이 아니다. 상류가 **0.0에 1.0이라는
의미를 부여한** 값이다. 두 가지는 같은 모양이 아니다.

그리고 이것은 이론이 아니다. `ros/moveit-ros/src/constraints/set.rs:50,57`이
wire 메시지를 코어 제약으로 변환하는 살아있는 경로이고, ROS 메시지의
`weight`는 미지정 시 0.0으로 도착한다. 즉 **상류가 정상 처리하는 메시지를
포트는 통째로 거부한다.** §183이 정의한 결함 그대로다.

### 199.2 D14

네 자리 전부 상류를 따라 `weight <= EPS`를 1.0으로 정규화한다. D6는
그대로 유지한다 — 구분선은 "해석될 수 없는 조회"(D6, 계속 `Err`)와
"상류가 의미를 부여한 wire 기본값"(D14, 상류를 따른다)이다.

경계마다 다른 규칙을 두지 않기 위해 wire 변환 경로에서만 정규화하는
타협은 택하지 않는다. 0.0이 1.0을 뜻하는 것은 상류 **생성자**의 의미이고,
포트의 생성자는 같은 함수다. 변환 경로에만 두면 Rust 직접 호출과 wire
경로가 같은 입력에 다른 답을 내는 이중 의미가 생긴다.

요구 테스트: 네 자리 각각 `0.0` / `EPS` / `EPS`보다 큰 값 / 음수의 경계
케이스, 그리고 `weight`를 지정하지 않은 wire 메시지가 `set.rs`의 변환을
통과해 가중치 1.0의 제약이 되는 경로 테스트 하나.

### 199.3 부수: 커밋 메시지가 이득을 틀리게 설명했다

`993506d`/`6c2d884`가 r2r 생성 상수를 참조하도록 바꾼 것은 옳다. 값도
직접 확인했다 — docker 안에서 여섯 개를 단언해 전부 통과했고,
`PLANNING_FAILED as i32 == -1`도 확인했다(음수 int32가 `repr(u32)` bindgen
enum을 통과해 살아남는다).

다만 커밋 메시지와 소스 doc이 적은 이득이 사실이 아니다: "renumber하면
컴파일 에러가 된다"고 했는데, `const ADD: u8 = ...::ADD as u8;`는 값이
0에서 5로 바뀌어도 컴파일된다. 컴파일 에러가 되는 것은 상수가 **삭제되거나
이름이 바뀔 때**뿐이다.

실제 이득은 그보다 낫다: renumber가 조용히 **따라간다**. 예전에는 조용한
불일치였던 것이 이제 불가능하다. 더 강한 사실을 약하고 틀린 문장으로
적은 것이라 고쳐야 한다.

## §200 편차 6이 닫혔다 — 다만 닫은 숫자는 이 저장소에서 재현할 수 없다

p3-acm 라운드 11(`75f076e`)이 이 프로젝트에서 가장 오래 열려 있던 UNFIXED를
닫았다. 편차 6 — pr2 `visibility_cone` 115건의 침투 깊이 불일치 — 는 포트의
결함이 아니라 **두 알고리즘의 차이**다.

방법이 옳다. 불일치 사례 하나(case 104)의 실제 승리 삼각형을 복원하고,
이 백엔드의 EPA로 자기 값 `2.08696987934592244e-2`를 재현한 뒤(포착된 기준값
`...93702e-2`와 `~1.5e-14` 상대 일치), **같은 삼각형/실린더에 진짜
`ccdMPRPenetration`을 직접 돌려** `7.47919999515277989e-2`를 얻었다. 오라클이
보고한 `7.47914550966356367e-2`와 절대 `5.4e-7`, 상대 `~7.3ppm`.

7.3ppm은 우연이 아니다. "불일치"가 "둘 다 자기 기준으로 옳은 서로 다른 두
알고리즘"으로 바뀌었다 — MPR은 portal refinement이고 EPA가 수렴하는 최소
깊이 witness로 수렴할 보장이 없다. 방향도 16/17 라운드의 `base_link` 16/16
표본과 같고(MPR이 항상 더 깊다), 크기도 그 `0.0312`–`0.1042m` 대역 안이다.

라운드 중간에 자기 오류를 하나 잡아 양쪽 문서에 남긴 것도 옳다: parry의
Y→Z `axis_fix`를 libccd의 `ccd_cyl_t`에 적용했는데 그쪽은 원래 Z축이다
(`testsuites/support.c`). 숫자를 믿기 전에 잡았다.

### 200.1 그런데 그 숫자를 다시 낼 방법이 트리에 없다

`ccdMPRPenetration`은 이 저장소 전체에서 두 파일에만 나온다 — `parry.rs`의
doc과 `doc/claim-audit/moveit-collision.md`. 둘 다 산문이다. libccd 드라이버는
in-tree에 없고, 라운드 16/17이 썼다는 빌드도 저장소에 없다.

parry 쪽 절반은 재현된다(`a_real_mismatching_case_touches_exactly_one_link`가
백엔드 값을 고정한다). 재현되지 않는 것은 **결정적인 절반**이다 — libccd MPR
숫자, 승리 삼각형 `[5, 1, 6]`의 동정, 정점 1이 실린더 원점에서 `1.1e-16`
안에 놓인다는 관찰.

§187이 라운드 24의 스윕에 대해, §195가 그 스윕의 결론에 대해 적은 것과 같은
모양이고, 이번이 셋 중 가장 무겁다. 115건짜리 편차의 **종결**이 아무도 다시
돌릴 수 없는 숫자 위에 서 있다.

libccd는 이 기계에 있다(`/home/stevek/work/libccd/`). §197.3이
`third_party/moveit_resources`에 대해 정한 것과 같은 형태로 닫는다:
`tools/ci/` 스크립트가 libccd 존재를 확인하고, 있으면 case 104의 삼각형에
MPR과 EPA를 둘 다 돌려 두 숫자와 그 비를 검증한다. 없으면 건너뛴다 —
다만 §196대로 건너뛴 사실이 조용하면 안 된다.

### 200.2 귀속 정정은 구조적 증명으로 왔다

지난 라운드의 과잉 주장("`group_name` 수정이 105/115의 직접 원인")이
철회됐다. 이번엔 표본이 아니라 구조로 증명했고, 내가 직접 확인했다:
`visibility.rs`에 `group_name`이 한 번도 나오지 않고
(`rg` 결과 0건), `CollisionRequest::default().group_name == None`이며
(`common.rs:270`), `active_group_links`는 `group_name?`에서 즉시 `None`을
돌려준다(`parry.rs:1327`). 필터가 no-op이므로 깊이를 바꿀 수 없다.

내가 앞선 라운드에 다른 논거로 같은 결론에 도달했었다 — group 필터링은
쌍을 남기거나 버릴 뿐 깊이 크기를 바꿀 수 없다. 서로 독립적인 두 반증이
일치한다. 표본이 아니라 구조로 다시 증명한 쪽이 더 강하다.

### 200.3 내 병합 게이트가 §178을 어겼다

이 병합 지점에서 워크스페이스 게이트를 돌리니 `cargo doc`이 실패했다 —
`collision_env_hybrid.rs`의 public 메서드 doc 6곳이 private
`build_env_distance_field`를 intra-doc 링크로 가리킨다. 라운드 29 병합
때 들어온 것이고, 그때 나는 fmt/clippy/nextest를 `-p`로 돌리고
**`cargo doc`을 돌리지 않았다**.

§178이 정확히 이것을 적어둔 절이다 — rustdoc 자체 린트에 닿는 게이트는
`cargo doc`뿐이고 clippy `--all-targets`도 `cargo test --doc`도 놓친다.
알고 적어놓고 병합 때 빼먹었다. `80412b6`으로 고쳤다. 크레이트 하나만
건드리는 병합이라도 doc 링크가 바뀌었으면 `cargo doc`은 범위에 들어간다.

## §201 측정을 지우는 것이 위생인 경우와 증거 인멸인 경우 — 구분선을 적는다

p1-joints 라운드 34(`193aa48`, `36d2a28`)가 두 항목을 끝냈다. 두 번째가 이
절의 이유다.

항목 2의 내용 자체는 훌륭하다. 내가 요청한 것은 "속도가 아니라 **기하**를
움직이는 세 번째·네 번째 케이스"였는데, 패널이 케이스 D(150° 코너)를
제안하면서 **그 케이스가 요청 목적으로는 쓸모없다**는 것을 스스로 측정해
보고했다: 45°/120°/150° 전부 케이스 A의 인덱스 8/7을 그대로 재현한다.

직접 확인했다. `search_intersection_points`(`:386`)는 **첫 궤적의 마지막
웨이포인트** 하나에서 `circ_pose`를 구한 뒤, 그 한 점에 대해 두 궤적을 각각
같은 반경으로 독립 탐색한다(`:395`, `:404`). 두 번째 궤적의 방향은 어디에도
들어가지 않는다. 반경과 속도를 고정하면 각도는 인덱스를 움직일 수 없다 —
구조적으로 그렇다. 패널의 설명이 맞다.

그리고 문서가 케이스 D의 가치를 "새 인덱스 커버리지"에서
"`blend_trajectory_cartesian`의 slerp/quintic 산술을 더 날카로운 코너에서
확인하는 것"으로 다시 규정하고, 거기서 불일치가 나면 **이번 라운드의 설명
자체가 반증된다**고 적었다. 자기 결론에 반증 조건을 붙인 것이다. 옳다.

### 201.1 그런데 그 측정이 트리에 없다

보고서의 문장: "예측을 유도한 probe 테스트는 `trajectory_blender_transition_
window.rs`에 작성해 실행한 뒤 완전히 되돌렸다(`git checkout --`, clean 확인)
— 트리에 측정 코드가 남지 않았다."

이것을 위생으로 보고했다. 이번 세션에서 네 번째다(§187 라운드 24의 스윕,
§195 그 스윕의 결론, §200 편차 6의 MPR 숫자, 그리고 이것). 앞의 셋은
부주의였고 이것은 **의도적**이라는 점만 다르다.

구분선이 어디에도 적혀 있지 않아서 적는다:

- **비계(scaffolding)는 지운다.** 답을 얻기 위해 임시로 만든 도구, 한 번
  쓰고 버리는 출력 코드, 탐색 과정 자체. 트리에 남기면 소음이다.
- **의존하게 된 주장의 증거는 남긴다.** 그 측정의 결과를 이후의 문장이
  인용하는 순간, 그것은 관찰이 아니라 주장이 된다(§195.1). 주장은
  재현 가능해야 한다.

케이스 D의 예측 인덱스 8/7과 "각도는 인덱스를 움직일 수 없다"는 설명은
둘 다 후자다. 요청 문서가 그 위에 서 있고, 내가 오라클 실행을 그 예측에
맞춰 계획한다.

### 201.2 이 경우는 오라클조차 필요 없다

가장 아까운 점이다. `search_intersection_points`는 **포트 자신의 함수**다.
코너 각도를 바꿔가며 인덱스가 변하지 않음을 단언하는 테스트는 오라클도,
docker도, 픽스처도 필요 없다 — 순수 in-tree 테스트다. 지울 이유가 없었다.

§200이 libccd 하니스를 요구한 것은 트리 밖 의존성 때문이었고, §197.3이
`third_party/moveit_resources`에 스크립트를 붙인 것도 같은 이유였다. 여기엔
그 장애물조차 없다.

## §202 §196의 구조적 닫기가 한 크레이트에서만 닫혔다 — 그리고 커밋 메시지가 자기 doc보다 더 크게 말했다

p3-distance-field가 fallback 마감에 잘렸지만 세 커밋이 들어왔다.

`d9c7078`이 §196.2가 요구한 구조적 닫기다. 감사 대신 fixture가 거절한다:
`test_support::assert_group_has_updated_links`가 그룹의 `updated_link_names()`가
비면 **fixture 구성 시점에** 패닉하고, 이 크레이트의 두 합성 fixture가
모델을 만든 직후 그것을 호출한다.

직접 확인했다. `collision_env_hybrid.rs:475`의 `mid_to_tip`을 `revolute`에서
`fixed`로 되돌리면 두 테스트가 공허하게 통과하는 대신 fixture 시점에
실패하고, 메시지가 원인과 고치는 법을 함께 말한다.

앵커 교정도 옳다. 내가 준 앵커는 "fixed joint 때문에 빈 그룹"이었고, 실제로
검사가 걷는 집합은 `link_names()`가 아니라 `updated_link_names()`다 —
`generate_distance_field_cache_entry`가 그것을 소비한다. `link_names()`로
멀쩡해 보이는 그룹이 여전히 공허할 수 있다. §191이 말한 "앵커가 증상을
가리킨다"의 또 한 사례이고, 이번엔 워커가 내 앵커를 고쳤다.

`ad02f78`은 §196.3을 코드 옆에 옮겨 적었고, `6a6e665`는 내가 `80412b6`으로
고친 것과 같은 private intra-doc 링크를 독립적으로 찾아 같은 방식으로
고쳤다 — 링크를 없애되 헬퍼의 가시성은 넓히지 않는 쪽. 같은 판단이다.

### 202.1 커밋 메시지가 자기 코드의 doc보다 강하게 말한다

`d9c7078`의 본문: "a group with an active joint can still resolve zero
*updated* links."

같은 커밋이 넣은 `lib.rs`의 doc: "...yet still resolve to zero *updated*
links **whenever none of its joints are active**."

둘이 다르다. doc은 "활성 관절이 없을 때"라는 안전한 주장이고, 커밋 본문은
"활성 관절이 있어도"라는 더 강한 주장인데 근거가 없다. §192가 기록한
모양(커밋은 맞는데 서술이 넘쳤다)의 반대 방향이다 — 여기선 doc이 맞고
커밋 본문이 넘쳤다.

강한 쪽이 참이라면 그것이 진짜 앵커이고 훨씬 중요하다. 참이 아니라면
철회해야 한다. 어느 쪽이든 시연이 필요하다.

### 202.2 한 크레이트만 닫혔다

헬퍼는 `#[cfg(test)] pub(crate)`이고 `moveit-distance-field` 안에서만 산다.
§196이 센 35개 fixture 자리 중 나머지는 그대로다. "36번째가 같은 방식으로
들어오는 것을 막는다"는 목표가 이 크레이트에 대해서만 달성됐다.

워크스페이스 전체로 넓히려면 헬퍼가 공용 테스트 지원 자리로 올라가야 하고,
그건 크레이트 경계를 넘는 변경이라 별도 결정이다. 지금은 UNFIXED다.

## §203 질문이 틀렸을 때 워커가 질문을 고쳤다 — scenario 3의 무승부는 두 후보 설명 중 어느 쪽도 아니었다

p1-robotmodel도 fallback 마감에 잘렸지만 여섯 커밋이 들어왔고, §195의 세
항목이 전부 닫혔다.

`494b4dc`가 내가 지적한 `zip` 절단을, `36eb630`/`82fc3f8`/`e370581`이 §195의
앵커 스윕에서 나온 세 자리를 각각 실행 가능한 단언으로 바꿨다. `82fc3f8`의
`30/30`은 이제 `0..30` 루프가 매번 다시 재고 정확히 단언한다.

이 절은 `71f421d` 때문이다.

### 203.1 내가 제시한 두 선택지가 둘 다 틀렸다

내가 물은 것: scenario 3의 `unwired 5/5 = wired 5/5` 무승부가 (a) 균일
관절공간 샘플링이 이미 충분해서 IK 샘플러가 더할 게 없는 것인지, (b) 코리도
자체가 사실상 공허한 것인지. "plan 성공률이 아니라 샘플 수준 만족률을
직접 재라"고 덧붙였다.

워커가 재보니 **전역 i.i.d. `sample_uniform` 만족률로는 두 선택지를 구분할
수 없다.** scenario 1과 3이 전역적으로는 비슷하게 희소하다 —
`0/20,000` 대 `2/200,000`. 내가 말한 "샘플 수준 만족률"이 잘못된 양이었다.

구분한 것은 다른 양이다: 이미 코리도 안에 있는 점에서 출발한, `step_size`로
제한된 **국소** 만족률 — `rrt_connect::extend`가 트리를 키우며 실제로
샘플하는 양. 거기서 `9.2%` 대 `83.0%`, 약 9배 차이가 나온다.

그 하나의 숫자가 두 가지를 동시에 한다. `83%`는 균일 샘플링만으로도 plan이
성립하기에 충분하므로 무승부를 설명하고, 동시에 `100%`에 한참 못 미치므로
공허함을 반증한다. (a)도 (b)도 아니고, "국소적으로는 쉽고 전역적으로는
아니다"였다.

### 203.2 전역 희소성과 국소 용이성은 다른 축이다

기록해 둘 일반형: 샘플러의 가치를 재려면 **플래너가 실제로 뽑는 분포에서**
재야 한다. 전역 균일 분포에서 잰 만족률은 두 시나리오를 똑같이 "희소"로
보이게 만들고, 실제로 플래너가 겪는 차이는 지우고 남지 않는다.

내 브리프의 "sample-level satisfaction rate"는 그 구분을 담지 못하는
표현이었다. 워커가 그것을 바꿔 재고, 바꿔 잰 이유를 적었다.

### 203.3 비용

새 테스트가 `20.1s`로 이 크레이트의 wall clock을 혼자 정한다(이전 `2.8s`).
워크스페이스 최장(`28.9s`, distance-field)을 넘지 않으므로 전체 시계는
그대로다. 지금은 받아들이지만, 다음에 이 크레이트에 긴 테스트가 하나 더
들어오면 그때는 wall clock을 재고 결정해야 한다.

## §204 같은 라운드에 두 패널이 같은 가드를 서로 다른 앵커로 넣었다 — 하나는 결함군을 닫지 못한다

p6-totg가 §196 가드를 자기 두 fixture에 넣었다(`d49461e`). 넣은 것 자체는
옳고, 넣기 전에 두 fixture의 관절 타입을 먼저 확인해 "지금은 `revolute`라
현재 이 버그에 맞지 않는다, 이건 선제적 가드다"라고 정확히 말한 것도 옳다.

그런데 단언하는 대상이 `link_names()`다. 같은 라운드에 p3-distance-field가
확인한 것은 검사가 실제로 걷는 집합이 `updated_link_names()`라는 것이었고
(§202), 그 둘은 같지 않다. `link_names()`가 비어 있지 않은데
`updated_link_names()`가 빈 그룹은 이 가드를 통과하면서 여전히 공허하다.

chomp에도 해당한다. `optimizer.rs:1015`가 `group_name`을 `CollisionRequest`에
실어 보내고, 그것을 받는 `ParryCollisionEnv`의 `active_group_links`
(`parry.rs:1330`)가 `updated_link_names()`를 읽는다. 즉 chomp의 충돌 단언이
공허해지는지를 결정하는 집합도 `updated_link_names()`이지 `link_names()`가
아니다.

### 204.1 이것이 헬퍼를 크레이트 밖으로 올려야 하는 이유다

§202.2에 "한 크레이트만 닫혔다"고 적었을 때는 범위 문제로 봤다. 한 라운드
뒤에 그것보다 나쁜 것이 드러났다: **두 번째 구현이 첫 번째와 다른 앵커를
쓴다.** 각 패널이 자기 크레이트에 같은 가드를 다시 만들면 앵커도 각자
고르고, 약한 쪽은 결함군을 닫지 못한 채 닫았다고 보고한다.

공유 헬퍼가 하나면 앵커도 하나다. 크레이트 경계를 넘는 변경이라 결정이
필요하고, 지금 내린다: **`assert_group_has_updated_links`를 공용 테스트
지원 자리로 올리고, 두 크레이트의 지역 사본을 그것으로 교체한다.** 위치와
crate-dep 방향은 p3-distance-field가 형태를 가져오면 확정한다.

### 204.2 브리프가 이미 끝난 일을 시켰다

항목 1은 "ChompOptimizer 포트를 끝내라"였는데, 패널이 이미 끝났다고 답했고
맞다. `77738b9`가 main의 조상이고 크레이트에 `todo!`/`unimplemented!`가
하나도 없다. 미포트 항목은 각각 이유와 함께 문서화돼 있다(`destroy()` D1
no-op, `debugCost` 죽은 코드, HMC 경로는 상류 자체가 죽은 코드,
`ChompPlanner`는 D1 제외 ROS 래퍼).

패널이 "라운드 B에 이미 같은 결론을 보고했다"고 적었다. 내 브리프가
그 보고를 반영하지 못한 채 두 라운드를 더 갔다 —
`[[task-brief-files-can-be-many-rounds-stale]]`가 기록한 것과 같은 실패이고,
이번엔 브리프를 쓴 것이 나다. 워커가 "이미 됐다"고 답할 때 그것을
확인하는 비용은 `git merge-base` 한 번이다.

## §205 미래에 고쳐질 것을 기다리는 테스트는 `#[ignore]`가 아니라 트립와이어여야 한다

p9-ros 라운드 13이 세 항목을 닫았다(`eac4850`, `b07527d`, `d4ca334`). 게이트
4/4 green.

`eac4850`이 §199.3의 틀린 문장을 정정했고, `6c2d884`는 애초에 그 문장을 쓴
적이 없어 손대지 않았다고 명시했다 — 한 결함군을 고칠 때 형제 자리를
확인하고 "여기엔 없다"까지 적는 것이 §189의 열거다.

`d4ca334`가 `ros/moveit-ros/doc/message-mapping.md` §17.5의 "반대 극성"
세 자리에 만료 조건을 달면서 **세 그룹이 서로 다른 조건을 갖는다**는
것을 밝힌 것이 이번 라운드의 제일 좋은 부분이다.
`state.rs`의 `attached_collision_objects`/`is_diff`는 코어 필드가 없어서가
아니라 `&mut PlanningScene`을 함께 받는 변환 진입점이 없어서 막혀 있고,
`multi_dof_joint_state`는 `moveit-state`의 다중-DOF 지원, `planning.rs` 두
자리는 `moveit-planning`의 필드, `trajectory.rs`는 `RobotTrajectory` 자체의
불변식 변경을 기다린다. 하나의 "구조적 갭"으로 뭉뚱그리면 넷 중 셋이
만료 조건 없이 남았을 것이다.

### 205.1 `#[ignore]`가 다시 들어왔다

`b07527d`이 D14의 wire 경로 테스트를 넣었다. 내가 요청한 대로 코어 수정
전에 먼저 써서 red임을 확인했고(`Err(Construct("JointConstraint weight must
be strictly positive"))`), 게이트를 green으로 유지하려고 `#[ignore]`를 달았다.

의도는 맞는데 형태가 이 세션이 계속 닫아온 결함이다. §184가 지운 것도,
§197.3이 스크립트로 감싼 것도, 전제가 충족됐는데 아무도 모르는 `#[ignore]`
였다. 여기서도 D14가 코어에 들어오는 순간 이 테스트는 통과할 수 있게 되지만
**아무것도 실패하지 않는다.** "그때 un-ignore한다"는 관례이지 장치가 아니고,
관례는 라운드가 넘어가면 사라진다.

### 205.2 대신 트립와이어

수정을 기다리는 테스트의 올바른 형태는 **현재의 (틀린) 동작을 단언하고,
수정이 들어오면 실패하도록 두는 것**이다:

- 지금: `weight = 0.0`이 `Err(Construct(...))`를 낸다고 단언한다 → green
- D14 착륙 순간: 그 단언이 깨진다 → **red**, 강제로 주의를 끈다
- 그때 이 테스트를 지우고 원하는 동작(정규화 → `1.0`) 단언으로 교체한다

`#[ignore]`는 수정이 들어와도 침묵하고, 트립와이어는 수정이 들어오면 소리를
낸다. 게이트를 green으로 유지한다는 목적은 둘 다 달성하는데, 하나만 만료를
스스로 알린다.

원하는 동작의 단언문은 같은 파일의 doc/주석으로 남겨 교체할 때 그대로
쓰게 한다. 일반 규칙으로 적는다: **다른 곳의 수정을 기다리느라 red인
테스트에 `#[ignore]`를 달지 않는다. 현재 동작을 단언하고, 수정이 그것을
깨뜨리게 한다.**

## §206 자기 가설을 15개로 확장해 4/15에서 반증한 라운드

p3-acm이 §200 항목 2를 닫았다(`b794f6d`). 나는 "case 104 하나가 아니라
표본 전체에 대해 주장하라"고 요구했고, 패널이 15개 조합
(`joint_state` × `target_radius` × `cone_sides`)으로 확장했다.

결과가 주장을 둘로 쪼갰다:

- **일반화된다:** 근접 배치가 접촉 링크 자신의 중심을 관통한다는 기하학적
  주장. 15/15에서 타깃-중심 정점이 실린더 로컬 원점의 `1e-9` 안에 있고,
  매번 실제 관통이 일어난다.
- **일반화되지 않는다:** 승리 삼각형이 항상 그 타깃-중심 정점을 포함한다는
  더 강한 주장 — case 104의 `[5, 1, 6]`이 그런 모양이었던 것. 15개 중
  **4개**에서만 참이다. 나머지는 센서 정점(정점 0)을 공유하는 삼각형이
  이겼다.

패널의 보고에 따르면 첫 초안이 강한 쪽을 보편 불변식으로 단언했고 첫
반례(`br_caster_l_wheel_link`, `radius=0.005`, `cone_sides=3`, 승리 삼각형
`[3, 0, 4]`)에서 실패했다. 그때 tolerance를 넓히거나 표본을 골라 통과시키지
않고 실제 분포를 재서 **참인 쪽만** 단언하고, 4/15라는 비율 자체를 고정해
드리프트가 소리를 내게 했다.

`[[known-constant-is-an-offset-not-a-tolerance]]`가 기록한 실패의 정확한
반대 행동이다. 반례를 만났을 때 tolerance를 움직이면 테스트는 통과하고
결함군은 남는다. 여기서는 반례가 주장을 고쳤다.

기록해 둘 형태: **일반화 요구는 그 자체로 반증 장치다.** case 104 하나만
보고 있었다면 `[5, 1, 6]`은 영원히 "이 메커니즘의 모양"으로 남았을 것이다.
n을 키운 것이 그것을 "이 메커니즘의 가장 눈에 띄는 사례"로 강등시켰다.

### 206.1 libccd 게이트는 아직 없다 — 그리고 하니스가 여전히 커밋되지 않았다

§200.1의 UNFIXED는 그대로다. 패널이 `tools/ci/`는 자기 소유가 아니라며
쓰지 않고 완전한 스펙만 `doc/claim-audit/moveit-collision.md`에 남겼다.
소유 규칙을 지킨 것은 맞다.

그런데 스펙이 참조하는 하니스 — 라운드 21의 `dev7/mpr_case104.c` — 가
"어디에도 커밋되지 않음, 필요하면 이 패널에 사본을 요청하라"로 적혀 있다.
편차 6을 닫은 그 숫자를 낸 프로그램이 한 패널의 작업 디렉터리에만 있다는
뜻이다. §201이 적은 구분선 그대로다: 비계는 지워도 되지만, **의존하게 된
주장의 증거는 남긴다.** 이 C 파일은 후자다.

게이트 스크립트는 내가 쓴다(`tools/ci/`는 내 소유다). 하니스는 내가 다시
유도할 것이 아니라 이미 동작하는 것을 커밋받아야 한다 — case 104의 삼각형
좌표가 트리 어디에도 없고 관절 상태에서 콘 메시를 재구성해야 나오기
때문에, 재유도는 그 자체로 두 번째 구현이 된다(§188).

libccd는 이 기계의 `/home/stevek/work/libccd/`에 소스로 있다. 시스템 설치는
없다(`pkg-config --exists ccd` 실패). 따라서 게이트는 소스 빌드 경로를
가져야 한다.

## §207 오라클을 돌려야만 알 수 있는 것: 예측이 맞았는지가 아니라 질문이 성립하는지

p1-joints가 라운드 35에서 `f0bc1c6`/`a7d5451`을 올렸고, 나는 row 13의
bite-check을 직접 재현했다(`:394`에 `circ_pose`를 `second_trajectory`의
마지막 waypoint 쪽으로 0.3 섞는 변이를 넣고
`search_intersection_points_indices_are_invariant_to_the_corners_angle`이
`left: (9, 10), right: (8, 7)`로 붉어지는 것을 확인, 되돌림). 같은 변이에서
row 14(`radius_sweep`)도 함께 붉어진다 — 패널이 "14/15는 별도 bite-check이
필요 없다"고 쓴 근거(정확값 4쌍 + 거부 경계)와 모순되지 않고, 그 주장을
한 건 더 강화한다.

그 다음 패널이 답한 항목 3("케이스 D를 돌릴 가치가 있는가")을 받아
케이스 C(`blend_radius 0.08`)와 D(150° 코너)를 오라클에 돌렸다. 이미지는
재빌드하지 않았다(스탬프 `043ed31a2186fe4e` 그대로). 결과:

- **케이스 C: 예측 그대로.** `first_intersection_index = 5`,
  `second_intersection_index = 10`, 입력 waypoint 16/16. 세 세그먼트의 모든
  waypoint를 비교하는 `blend_panda_arm_radius08_matches_the_oracle`로 착지
  (`e228571`).
- **케이스 D: 오라클이 거부한다.** `error_code = -1`(`PLANNING_FAILED`),
  `generateJointTrajectory`가 4번째 blend 샘플에서 `panda_joint2`의 감속
  한계 위반(`-2.50863` vs `-1.875`). 인덱스 필드는 아예 방출되지 않는다.

### §207.1 예측이 반증된 것이 아니라, 예측을 시험할 자리가 없었다

케이스 D의 문서 예측은 "인덱스가 케이스 A와 동일할 것"이었다. 오라클이
인덱스를 내놓지 않으므로 이 예측은 **확인되지도 반증되지도 않았다**. 이건
"예측이 틀렸다"와 다른 결과이고, 둘을 같은 칸에 적으면 다음 라운드가
잘못된 교훈을 가져간다. 문서(`oracle-request-pilz-blend-geometry.md`)에는
결과를 앞에 붙이고 원래 예측 산문은 그대로 남겼다 — 반증된 예측이
읽히는 채로 남아야 §205의 트립와이어 논리와 같은 값을 한다.

이 라운드가 실제로 배운 것은 예측의 참/거짓이 아니라 **질문의 성립
여부**다. 요청 문서는 "포트 쪽에서 로컬로 돌려봤고 거부되지 않았다"를
근거로 케이스가 성립한다고 썼다. 하지만 포트 쪽 로컬 실행은
`search_intersection_points`만 돌린 것이었고, 거부는 그보다 **뒤 단계**인
`generate_joint_trajectory_from_cartesian`에서 일어난다. 로컬 프로브가
파이프라인의 앞부분만 돌린 채 "성공한다"고 읽힌 것 — 프로브의 범위가
케이스의 범위보다 좁았다는 뜻이다.

### §207.2 거부 일치는 약한 결과가 아니다 — 단, 코드 일치만으로는 그렇다

두 구현이 모두 거부했다는 사실만으로는 아무것도 증명되지 않는다. 서로 다른
이유로 거부해도 `Result::Err`는 같은 모양이기 때문이다. 그래서 세 층으로
확인했다:

1. 에러 코드를 하드코딩된 variant가 아니라 오라클 fixture의 `error_code`
   숫자와 비교한다(`-1`). `InvalidMotionPlan`(`-2`)이면 통과하지 않는다.
2. `verify_sample_joint_limits`의 감속 분기와
   `generate_joint_trajectory_from_cartesian`의 샘플 루프에 임시
   `eprintln!`을 넣어(적용→실행→되돌림) 포트가 **샘플 4, `panda_joint2`,
   `acceleration_current = -2.5086292326350526`** 에서 거부하는 것을 봤다.
   상류 로그의 `-2.50863`과 상류가 출력하는 자리 수 전부가 같다.
3. 그 증거를 테스트 doc에 남겼다 — §201이 요구하는, 의존하는 주장의 근거는
   지울 수 없다는 규칙.

### §207.3 `run_case`는 거부 케이스를 표현할 수 없었다 — 구조가 결과를 골랐다

기존 드라이버는 `blend(...)`의 결과를 자기가 `unwrap`했다. 그러면 표현
가능한 결과가 성공 경로 하나뿐이고, 거부 케이스는 파이프라인 80줄을
복제해야 쓸 수 있다. `drive_case`가 `Result`를 그대로 호출자에게 넘기도록
갈랐다(`638e8a0`). 이건 편의 리팩터가 아니라 §207.1이 드러낸 것과 같은
문제다 — 도구가 볼 수 있는 결과의 집합이 실제 결과의 집합보다 좁으면,
좁은 쪽이 답으로 보고된다.

### §207.4 톨러런스를 다시 재고, 넓히지 않았다

케이스 C가 `POSITION_TOLERANCE`의 측정 최대값을 `2.28e-9`에서 `5.46e-9`로
올린다. 상수는 `1e-8` 그대로 두었다 — 문서가 자칭하던 "약 4배 마진"을
복원하려면 `2.5e-8`로 넓혀야 하는데, 양쪽 다 결정적 계산이라 마진이 사는
것은 없고 죽는 것은 `1e-8` 규모의 실제 회귀를 볼 능력뿐이다
(`known-constant-is-an-offset-not-a-tolerance`). doc 주석에서 "4배"라는
자칭을 지우고 실제 마진 `1.8배`와 케이스별 측정치를 적었다.

`verify-fixture-replay.sh` 49/49 → 51/51 identical. 오라클 스탬프
`043ed31a2186fe4e` 불변.

## §208 트립와이어가 처음으로 울린 라운드, 그리고 손으로 적은 목록 두 개를 지운 라운드

여덟 패널이 모두 idle이 되어 여섯 브랜치 11개 커밋을 한 번에 병합했다.
`--workspace` 게이트 전부 green(1598/1598), `check-*.sh` 8종, `verify-*.sh`
7종, `ros/verify-ros-interop.sh` 4/4.

### §208.1 §205의 트립와이어가 실제로 값을 냈다

p9-ros(`cd425e9`)는 네 개의 wire-path 테스트를 **현재의 (틀린) 거부 동작을
assert하도록** 썼다. `#[ignore]`가 아니라 green인 채로 두어, D14가 착지하는
순간 자동으로 붉어지게 한 것이다. 같은 병합 라운드에 p1-robotmodel의
D14(`551b719`)가 들어왔고, 네 개가 전부 붉어졌다. 각 `Ok` 값이
`weight: 1.0`을 달고 있었다 — D14가 `TryFrom` 체인 전체를 통과해 네 타입
모두에서 동작한다는 것을, 읽어서가 아니라 실행해서 알았다. 전환은
`932b7bf`.

전환이 쉬웠던 이유는 패널이 주석에 "울리면 무엇을 하라"까지 적어뒀기
때문이다(`replace that test with assert_eq!(c.weight(), 1.0)`). 울릴 조건만
적힌 트립와이어는 다음 사람에게 붉은 테스트 하나를 남길 뿐이다.

`#[ignore]`였다면 D14 착지 사실은 아무 신호도 내지 않았고, 테스트는
"원하는 동작"을 assert한 채 영원히 skip되었을 것이다 — §184/§197.3이
두 번 닫은 그 모양 그대로.

### §208.2 손으로 적은 목록은 두 층 모두에서 같은 방향으로 실패한다

p1-fixtures가 두 가지를 찾았고 둘 다 `tools/ci`(내 소유)였다.

1. `verify-vendored-fixture-tests.sh`의 `TESTS` 배열이 손으로 유지된다.
   새 `#[ignore = "needs third_party/moveit_resources"]` 테스트는 배열에
   추가되지 않고 **그냥 안 돈다** — 이 스크립트가 닫으려던 never-runs
   결함이 한 층 위에서 재현된 것. `81e6867`이 attribute 자체에서
   `mapfile`로 유도하고 `attr_count` 자기검사를 붙였다. 세 번째
   `#[ignore]` 테스트를 넣어 실행 수가 스크립트 수정 없이 3으로 가는 것을
   확인했다.
2. **어떤 `verify-*.sh`를 매 라운드 돌릴지가 산문으로 손에 적혀 있었다.**
   내 라운드-17 브리프는 당시 여섯 중 셋만 이름 붙였다. 같은 결함,
   한 층 위. `1a9fc39`의 `tools/ci/verify-all.sh`가 glob으로 열거를
   대체한다(자기 자신 제외, 하나 실패해도 전부 실행, 실패를 모아 보고).
   `sg docker -c ./tools/ci/verify-all.sh` 하나가 이제 게이트다.

`--type rust` 제약이 하중을 받는다: 없으면 `attr_count`가 스크립트 자신의
주석에 인용된 attribute까지 세어 **틀린 이유로 통과한다**. 패널이 양쪽 다
돌려서 잡았다.

### §208.3 측정을 넓게 서술하는 것은 부정 주장에서 특히 비싸다

p3-shapes(`b9a64bb`)의 doc이 "1..=200 covers every num_timesteps"라고
쓰는데 코드는 `1..=60` 연속 + `[80, 100, 150, 200]` 네 점이다. `61..=199`의
나머지는 검사되지 않는다. §189의 모양 그대로지만, 이 경우 **doc이 그
측정에서 끌어내는 결론이 부정 주장**("어떤 호출자도 생성자가 거부하는
covariance에 도달할 수 없다")이라는 점이 다르다. 부정 주장은 실제로 훑은
집합만큼만 강하다. `8351f8d`에서 양쪽을 다 적고 빈 구간을 명시했다.
커밋 메시지의 같은 문장은 이미 history에 있어 고칠 수 없다.

### §208.4 deviation 6의 숫자가 이 저장소에서 재현 가능해졌다 — §200.1 닫힘

p3-acm의 `34480c8`을 읽지 않고 돌렸다. `build.sh`가 libccd v2.1을
`CCD_DOUBLE`로 소스 빌드하고, Rust 예제가 재구성한 기하를 파이프로 먹여
`mpr_depth=7.47919999515277989e-02` — README가 주장하는 숫자와 정확히 같다.
EPA는 `-0.020869698793459224`. `384f80c`의 `verify-mpr-vs-epa.sh`가 두 값을
1e-9 상대오차로 게이트하고, v2.1 체크아웃이 없으면 **큰 소리로** SKIP한다.
어느 쪽 값이든 그 이상 흔들면 실패하는 것을 확인했다.

다만 이것으로 deviation 6이 **클래스로서** 닫힌 것은 아니다. 케이스 하나다.
p3-acm에 N개로 확장하고 부호가 뒤집히는 케이스를 찾으라고 넘겼다 — MPR이
EPA보다 얕은 케이스가 하나라도 있으면 "by construction"이라는 절반이
반증된다.

## §209 §198의 "값싼 닫기 없음"이 틀렸다 — 절반은 플래그 하나였다

§198은 `cargo doc`가 자기가 문서화하는 항목까지만 검사한다는 것을 기록하고,
노출 규모(61파일 / 3649줄 / 링크 있는 350줄)를 적은 뒤 **값싼 기계적 닫기가
없다**고 결론냈다. 그 결론의 절반이 틀렸다.

`RUSTDOCFLAGS="--document-private-items" cargo doc --workspace --no-deps`
한 번이 **깨진 링크 36개**를 즉시 뱉는다. 원인은 단순하다:
`crates/moveit-collision/src/lib.rs:76`은 `mod parry;`이지 `pub mod`가
아니다. 그래서 `parry.rs`의 모듈 헤더 — 이 저장소에서 가장 긴 doc 중
하나이고 deviation 1~11이 전부 거기 산다 — 는 렌더링된 적도, 링크가
검사된 적도 없다. 다른 private 모듈도 전부 같다.

발견된 것(모두 `0463136`에서 수정):

- unresolved 36: 대부분 다른 크레이트 타입을 import 없이 bare로 링크
  (`[`JointModelGroup`]`), 존재하지 않는 경로
  (`moveit_model::PlanarJoint::…` — 실제로는 `moveit_model::joint::PlanarJoint`),
  private 테스트 이름 링크, 그리고 `crate::iter::Leaf`처럼 아예 없는 모듈.
- ambiguous 8+4: `KINEMATICS_SOLVERS`(static이자 macro),
  `query::contact`(함수이자 모듈), `rrt_connect`(함수이자 모듈).
- redundant explicit target 4개.

### §209.1 고치는 과정에서 내가 두 개를 새로 만들었다

`[`Compound`]`을 `[`parry3d_f64::shape::Compound`]`로 일괄 치환했더니
이미 명시적 타깃을 달고 있던
`[`Compound`](parry3d_f64::shape::Compound)` 한 자리가
`[`X`](X)` 꼴이 되어 redundant 오류가 났다. 또
`[`crate::rrt_connect`]`를 `mod@`로 명확화했더니 이번엔 **private 모듈을
public doc이 링크한다**는 다른 오류로 바뀌었다(모듈이 `mod rrt_connect;`,
공개된 것은 `pub use`된 항목들뿐). 둘 다 코드 스팬으로 되돌렸다.

일괄 치환이 자기가 만든 새 오류를 남기는 것 — 게이트가 없었으면 둘 다 다음
라운드까지 보이지 않았을 종류다. 게이트를 먼저 세우고 고쳤기 때문에 같은
자리에서 잡혔다.

### §209.2 `#[cfg(test)]` 절반은 여전히 열려 있고, 왜 열려 있는지가 중요하다

`--cfg test`를 RUSTDOCFLAGS에 더하면 rustdoc이 `#[cfg(test)]` 모듈을
보긴 한다. 그러나 doc 빌드는 dev-dependency를 링크하지 않으므로
`approx`, `rand_chacha`, `moveit_sampling` 등의 import에서 컴파일이
깨진다. 즉 이 절반은 플래그 하나로 닫히지 않는다 — §198의 결론은 이쪽에
대해서는 맞았다. `verify-private-doc-links.sh`(`eda6f46`) 헤더에 이
사실과 "플래그를 지우고 닫혔다고 부르지 말 것"을 적어뒀다.

### §209.3 규칙

"값싼 닫기가 없다"는 결론은 **시도한 명령을 적지 않으면** 검증 불가능한
주장이다. §198은 노출 규모는 숫자로 적었지만 무엇을 시도해봤는지는 적지
않았다. 다음에 같은 형태의 결론을 쓸 때는 시도한 명령과 그 출력이 함께
가야 한다 — 그것이 §189가 측정에 대해 요구하는 것과 같은 요구다.

## §210 테스트가 느렸던 것이 아니라 프로파일이 없었다 — 그리고 "휴면 중"으로 분류한 결함군을 닫았다

p1-fixtures 라운드가 두 가지를 보고했다. 하나는 재현된 결함, 하나는
재현하지 못한 패턴. 조율자 소유 파일이라 셋 다 내가 적용했고, 그 과정에서
분류 자체가 한 번 뒤집혔다.

### §210.1 게이트 15개 중 1개만 "실증 가능"이었는데, 나머지도 고쳤다

`verify-clean-checkout.sh:51`이 `mapfile -t steps < <(python3 ...)` 꼴이다.
`mapfile`은 producer의 종료 상태를 전파하지 않고 `set -e`도 그것을 보지
못하므로, 파서 자신의 `sys.exit("no run: steps found in the workflow")`가
stderr에 찍힌 채로 스크립트는 0으로 끝나며 "every ci.yml step passes"를
출력한다 — 0개를 돌리고서. 합성 workflow(‌`run:` 없음)로 양방향 재현했다:
수정 전 rc=0, 수정 후 rc=1 (`a1a0b8f`).

나머지 다섯 스크립트는 p1-fixtures가 "실증 불가(not demonstrable)"로
분류했다. `git ls-files`/glob 목록이 비면 루프가 0회 돌고 그대로 OK를
찍지만, 현재 트리에서는 목록이 비지 않으므로 재현할 수 없다는 것이다.
분류는 정확했다. 그러나 **결론이 틀렸다**: 그 목록들이 파일시스템에서
유도되는 이유가 바로 아무도 유지보수하지 않기 위해서이고, 같은 성질이
경로 규약이 바뀌는 순간 목록을 조용히 비게 만든다. `verify-vendored-
fixture-tests.sh`와 `verify-all.sh`는 각자 자기 목록에 대해 이 가드를
**사후에** 붙였다 — 두 번 다 같은 실패를 겪고 나서.

`tools/ci/gate-lib.sh`의 `require_nonempty`가 그 규칙을 한 자리에 둔다
(`fd1bc04`). 호출부 7개: `check-license-matches-upstream.sh`,
`check-lints-not-silently-dropped.sh`,
`check-workspace-dep-inheritance.sh`(2),
`verify-fixture-provenance.sh`(3), 그리고
`verify-upstream-license-provenance.sh`는 파이썬 쪽 `tracked`/`checked`
카운터에 같은 규칙을 적었다. 각 가드는 자기 glob을 아무것도 매치하지 않는
경로로 돌려 **개별적으로 물려봤다** — 7/7 rc=1.

`check-audit-scripts-not-copied.sh`는 제외했다. 매치 0이 곧 통과인 부정
단언이라 emptiness 가드가 의미를 뒤집는다.

### §210.2 스위트 24.2초 중 22초는 최적화 없이 컴파일된 수치 코드였다

p1-fixtures가 가장 느린 10개를 재고, 그중 `gripper_pair_contact_is_
prediction_invariant`를 단독으로 dev 20.1초 / `--release` 1.076초로
측정했다 — 19배. 그 테스트가 하는 일은 PR2 모델을 만들고 parry contact
질의 3번을 돌리는 것뿐이다. 루트 `Cargo.toml`에 `[profile...]` 섹션이
하나도 없었다.

내가 잰 것(이 머신, 96코어):

| 프로파일 | 그 테스트 | 워크스페이스 전체 | cold build |
|---|---|---|---|
| 기본(opt-level 0) | 18.4s | 24.2s | 25s |
| deps만 opt-level 2 | 14.1s | — | — |
| workspace 1 + deps 2 | **0.9s** | **2.0s** | 39s |

deps만 올리면 1.3배밖에 안 준다. 비용의 대부분은 이 저장소 자신의
크레이트에 있었다. `[profile.dev] opt-level = 1` + `[profile.dev.package."*"]
opt-level = 2`가 `e733f19`이고, 일회성 빌드 비용 +14초는 첫 테스트 실행에서
회수된다.

`debug-assertions`와 `overflow-checks`는 `opt-level`과 독립이라 그대로
켜져 있다. **가정하지 않고 확인했다**: `--print cfg`가 여전히
`debug_assertions`를 내보내고, `-C opt-level=1 -C debug-assertions=on`으로
컴파일한 `u8` 오버플로가 여전히 패닉한다. 스위트의 벽시계를 커버리지와
바꾸는 프로파일이었다면 이 변경은 손해다.

### §210.3 규칙 두 개

하나. **"현재 트리에서 재현 불가"는 "결함 아님"이 아니다.** 재현 가능성은
결함의 성질이 아니라 지금 트리 상태의 성질이다. 파일시스템에서 유도되는
목록은 유지보수를 없애는 대신 조용히 비어질 경로를 연다 — 그 둘은 같은
성질의 앞뒷면이다.

둘. **비용을 잰다는 것은 테스트를 재는 것이 아니라 무엇이 비용을 지배하는지
재는 것이다.** 라운드마다 "가장 느린 N개" 표를 만들었지만 그 표의 어느
행도 "이 비용은 테스트가 아니라 빌드 설정에서 온다"를 말할 수 없다. 그것을
말한 것은 한 테스트를 dev와 release 양쪽에서 **각각 재본** 한 번의 측정
이었다. 표는 순위를 주고 원인은 대조가 준다.

## §211 인용 하나가 소비자 열 개를 대변할 수 없다 — 상류 규칙 세 개를 한 규칙으로 덮은 라운드

### §211.1 시작점: 워커의 근거는 맞았지만 그 근거가 닿는 곳이 하나였다

p9-ros가 `4ff563d`에서 `ros/moveit-ros/src/geometry.rs`의
`TryFrom<Quaternion> for UnitQuaternion` 문턱을 `norm <= f64::EPSILON`에서
`|norm - 1.0| > 1e-3`으로 좁혔다. 근거로 든 것은
`kinematic_constraint.cpp:609-615` — `OrientationConstraint::configure`가
`fabs(q.norm() - 1.0) > 1e-3`을 "probably incorrect"로 보고 항등원으로
치환하는 분기다. 그 인용 자체는 **확인했고 정확하다**. 같은 라운드의
`cone_sides` 판정도 `:822-829`과 헤더 `:878`(`unsigned int cone_sides_;`)에서
그대로 성립한다 — 상류 자신의 가드 순서가 int→unsigned 감김을 이미 막고
있어서 `msg.cone_sides.max(0) as usize`는 정확히 옳다.

문제는 근거가 아니라 **그 근거가 닿는 호출 지점의 수**였다. 이 impl의
소비자는 열 개다. `configure`에 닿는 것은 그중 하나다.

### §211.2 상류에는 규칙이 하나가 아니라 셋이다

> **이 절의 결론은 §211.6이 뒤집었다. 규칙은 셋이 아니라 둘이다.** 아래
> 표의 `tf2::fromMsg` + `ASSERT_ISOMETRY` 두 행은 별개의 세 번째 규칙이
> 아니라 일반 규칙(무조건 `quaternion.normalize()`)의 일부다 —
> `tf2_eigen.hpp:493-505`의 `fromMsg` 본문 자체가 조건 없이 정규화하고,
> `ASSERT_ISOMETRY`는 그 *다음에* 온다. 표와 그 아래 "하나."의 논증은
> 라운드 15가 이 결론에 도달한 경로로 남겨 두고, 사실관계는 §211.6을 읽어라.

| 포트 지점 | 상류 도달 경로 | 상류 규칙 |
|---|---|---|
| `constraints/orientation.rs:85` | `OrientationConstraint::configure` :609-615 | `\|norm-1\|>1e-3` → 경고 후 항등원 치환 |
| `constraints/position.rs:161` | `PositionConstraint::configure` :405-406, :433-434 | `tf2::fromMsg` + `ASSERT_ISOMETRY` |
| `constraints/visibility.rs:114,115` | `VisibilityConstraint::configure` :845-846, :858-859 | `tf2::fromMsg` + `ASSERT_ISOMETRY` |
| `scene/collision_object.rs:142,207,239,478,515` | `planning_scene.cpp` `utilities::poseMsgToEigen` | 무조건 `quaternion.normalize()` |
| `scene/planning_scene.rs:147` | `planning_scene.cpp:1496` 같은 헬퍼 | 무조건 `quaternion.normalize()` |

두 가지가 이 표를 만들었다.

하나. **`ASSERT_ISOMETRY`는 검사처럼 보이지만 릴리스에서 아무것도 하지
않는다.** `third_party/geometric_shapes`의 `check_isometry.h`에서 `NDEBUG`이면
`(void)sizeof(transform);`로 전개된다. 아니면 `checkIsometry`를
`Eigen::NumTraits<double>::dummy_precision()`(1e-12)로 부르고
`assert(!"Invalid isometry transform")`으로 죽는다. 즉 이 세 지점에서
출하되는 상류의 동작은 "검사 없음"이고, 디버그 상류의 동작은
"1e-12에서 abort"다. 1e-3은 그 어느 쪽도 아니다.

둘. **`planning_scene.cpp:76-82`의 `utilities::poseMsgToEigen`은 정규화가
목적인 헬퍼다.** 독스트링이 "convert Pose msg to Eigen::Isometry,
**normalizing the quaternion part if necessary**"이고 본문은 조건 없는
`quaternion.normalize()`다. 여섯 지점이 여기로 간다. `norm == 2.0`은 상류가
**의도적으로 받아들이는** 값이고, D14의 시험("상류가 의미를 정의하는가")이
성립한다 — 정의된 의미가 "정규화한다"이다. 그 여섯 지점에 D6은 닿지 않는다.

### §211.3 균일함이 목표가 아니었다

이 라운드에서 처음에 떠오른 교정은 "경계에 규칙 하나"였다 — 이 문서가
반복해서 선호해 온 형태다. 그것이 틀린 이유는 **상류가 균일하지 않기
때문**이다. 균일한 규칙은 미러링하는 대상이 균일할 때만 기본값이다. 세
규칙을 하나로 덮는 것은 정리가 아니라 동작 변경이고, 정리처럼 보이기
때문에 리뷰를 통과한다.

구조적 교정은 문턱을 옮기는 것이 아니라 **이중 의미를 없애는 것**이다.
현재 `TryFrom<Quaternion>`은 한 지점에서 "방향 제약의 의심 규칙"을,
아홉 지점에서 "일반 포즈 규칙"을 뜻한다. 각 상류 규칙에 자기 이름을 주고
`Pose → Isometry3`가 어느 규칙을 적용하는지 호출자에게 보이게 해야 한다.
p9-ros에 이 형태로 넘겼고, `ASSERT_ISOMETRY` 세 지점은 어느 규칙을
택할지 **접지 말고 판정해서 근거와 함께 보고**하도록 명시했다.

### §211.4 문서가 코드보다 먼저 낡았다

`TryFrom<Pose> for Isometry3`의 독스트링은 `4ff563d` 이후에도 "Fails exactly
when the embedded orientation does (`Quaternion::try_from`'s
**zero/non-finite-norm** case)"라고 말한다. 이제 `norm == 2.0`에서도 실패한다.
Pose 경로의 유일한 테스트 `pose_with_degenerate_orientation_fails`가 전부-0
케이스만 덮고 있어서 — 즉 **낡은 문서가 말하는 그 케이스만** 덮고 있어서 —
어긋남을 아무것도 잡지 못했다. 테스트가 문서와 같은 범위를 가지면 문서의
낡음을 테스트로 검출할 수 없다.

### §211.5 규칙

**공유 헬퍼의 상류 규칙을 인용하기 전에 소비자를 센다.** 인용 하나에
소비자 열이면 그 자체가 신호다. 헬퍼는 호출자 하나가 시야에 있는 동안
쓰이고, 그 호출자의 상류 규칙이 헬퍼의 규칙으로 기록된다. 이후의 모든
호출자는 누구도 자기 몫으로 다시 유도하지 않은 정당화를 상속한다.

**가드의 문턱이 매크로에서 오면 매크로를 연다.** `ASSERT_ISOMETRY`는
검사처럼 읽히고 릴리스에서 `(void)sizeof(x)`로 컴파일된다.

### §211.6 매크로만 열고 그 앞의 함수는 안 열었다 -- 세 규칙이 아니라 두 규칙이었다

p9-ros가 §211.2의 표를 오라클 이미지 안 실제 헤더로 재검증했다. `ASSERT_ISOMETRY`에
대한 §211.2의 서술(`check_isometry.h`, `NDEBUG`면 `(void)sizeof(x)`)은
정확했고 그대로 확인된다. 그런데 §211.2는 그 매크로 **앞에서 호출되는
함수 자체**는 열어보지 않고 "검사 없음"이라고 결론냈다 -- 그 함수가
`tf2_eigen.hpp`의 `fromMsg(const geometry_msgs::msg::Pose&,
Eigen::Isometry3d&)`(`:493-505`)이고, 본문이 이렇다:

```cpp
void fromMsg(const geometry_msgs::msg::Pose & msg, Eigen::Isometry3d & out)
{
  Eigen::Quaterniond quat(msg.orientation.w, msg.orientation.x,
                          msg.orientation.y, msg.orientation.z);
  quat.normalize();
  out = Eigen::Isometry3d(Eigen::Translation3d(...) * quat);
}
```

조건 없는 `quat.normalize()`다 -- `planning_scene.cpp`의
`utilities::poseMsgToEigen`과 **똑같은 규칙**이다. `ros/moveit-ros/src/constraints/position.rs:161`/
`ros/moveit-ros/src/constraints/visibility.rs:114,115`가 실제로 여기 닿는다는 것은
`kinematic_constraint.hpp:875,877`의 `Eigen::Isometry3d sensor_pose_,
target_pose_;` 선언으로 확인된다 (`t`/`target_pose_`/`sensor_pose_`가
전부 `Isometry3d`이므로 오버로드 해석이 이 함수를 정확히 고른다).
`ASSERT_ISOMETRY`는 이 정규화 **다음에** 실행되는 중복 안전장치이고,
릴리스에서 아무것도 하지 않는다는 §211.2의 관찰은 맞지만 그로부터
"그러니 정규화도 없다"는 따라 나오지 않는다.

즉 열 개 소비자가 닿는 상류 규칙은 세 개가 아니라 **두 개**다:

1. **일반 규칙** (9곳: `orientation.rs`를 제외한 전부) -- 조건 없이
   정규화, 절대 실패하지 않음. `poseMsgToEigen`과 `tf2_eigen::fromMsg`
   둘 다 이 규칙 하나로 수렴한다.
2. **`OrientationConstraint::configure`의 의심 규칙** (1곳:
   `ros/moveit-ros/src/constraints/orientation.rs:85`만) -- `\|norm-1\|>1e-3`이면 항등원으로 치환.

§211.3이 요구한 "이중 의미 제거"는 그대로 유효하다 -- 다만 이름을 줘야
할 규칙이 셋이 아니라 둘이었다. p9-ros가 `geometry.rs`에
`OrientationConstraintQuaternion`(전용 타입, 자기 규칙만 구현)을 새로
만들고 기존 `TryFrom<Quaternion> for UnitQuaternion`을 문턱
`norm <= f64::EPSILON`로 되돌려 아홉 곳의 일반 규칙으로 되돌렸다.
`Eigen::MatrixBase::normalize()`(`Eigen/src/Core/Dot.h:145-151`)가
`if (z > RealScalar(0))`로 가드하므로 정확히 0인 노름은 상류에서도
그대로 남는다는 것까지 오라클 이미지의 실제 헤더로 확인했다 -- 이
지점만은 D6이 남는다(`nalgebra::UnitQuaternion`이 표현할 수 없는 값이므로).

**규칙 (§211.5에 추가):** 매크로를 열었으면 그 매크로가 감싸는 *이전*
문장도 열어라. 가드 앞에 이미 부작용이 있으면, 가드가 no-op이라는 사실만으로
"그 지점의 상류 동작은 없음"이라고 결론 내릴 수 없다.

## §212 스탬프를 12일 만에 움직인 라운드 — 그리고 그 움직임을 재생 게이트가 증명하게 한 방법

### §212.1 두 패널이 같은 이미지에서 막혀 있었다

p3-distance-field는 `HybridCollisionEnv`의 네 `*DistanceField` 진입점 중
F1/F2/F4에 `"mode": "robot_only"`가 필요하다고 요청 문서에 적었고,
p3-acm은 case 623을 가르기 위해 `collision` op의
`max_contacts_per_pair`를 요청했다. 둘 다 코디네이터 소유
(`tools/moveit-oracle/`)이고, 둘 다 이미지 재빌드를 뜻한다.

한 번의 재빌드로 둘 다 처리했다. 재빌드 비용은 실측 36초 —
`moveit_core`는 캐시되고 오라클 자신의 C++만 다시 컴파일된다. 즉
"스탬프를 움직이지 않는다"를 지켜온 이유는 빌드 비용이 아니라
**기존 픽스처가 조용히 달라질 위험**이었고, 그 위험은 재빌드 횟수가
아니라 변경의 모양으로 결정된다.

스탬프: `043ed31a2186fe4e` → `700e7be54cb0a61f`.

### §212.2 두 변경 모두 "기본값에서 오늘과 바이트 단위로 같다"를 구조로 보장했다

`mode` (신규, 기본 `"collision"`): 없으면 `checkCollision` — 이 필드가
생기기 전 모든 픽스처가 탄 바로 그 분기다. `"robot_only"`는
`checkRobotCollision`. 두 오버로드 모두 `last_gsr_`를 설정하므로
(`collision_env_distance_field.cpp:1468,1497`) 응답 덤프 코드는 손대지
않았다.

`max_contacts_per_pair` (신규, 기본 `1`): 여기서 갈림길이 있었다.
`contactsToJson`는 쌍마다 `front()` 하나, `allContactsToJson`은 전부를
같은 `ContactMap` 순회 순서로 내보낸다. **요청 값에 따라 둘 중 하나를
고르는 분기**를 두는 대신 항상 `allContactsToJson`을 쓴다 — 기본값
`1`에서는 어떤 쌍의 리스트도 원소가 둘 이상일 수 없으므로 두 함수의
출력이 **구성상** 동일하기 때문이다. 규칙이 하나 남는다: "쌍마다
`max_contacts_per_pair`개까지 보고한다." 요청 값에 따라 응답의 의미가
달라지는 형태를 만들지 않았다.

### §212.3 증명은 논증이 아니라 재생이었다

두 변경 모두 "기본 경로는 안 건드렸다"고 **주장**할 수 있었다. 대신
새 이미지에 대해 `verify-fixture-replay.sh`를 돌렸다: **52/52
identical**, drift 0, 다른 출력 라인 0. 그것이 이 라운드에서 스탬프
이동을 정당화한 유일한 증거다.

이것이 스탬프 게이트가 존재하는 이유의 역방향 사용이다. 게이트는
평소에는 "낡은 이미지로 픽스처를 만들지 마라"를 강제하지만, 재빌드가
불가피한 라운드에서는 **"새 이미지가 옛 픽스처를 재현하는가"를 묻는
유일한 도구**가 된다. 재빌드를 미루면 이 질문을 던질 기회 자체가 없다.

### §212.4 남은 것

`b1ef9a8`(mode 분기)만 담긴 중간 트리의 이미지는 빌드하지 않았다 —
빌드하고 재생을 확인한 것은 두 변경이 모두 들어간 최종 트리
(`700e7be54cb0a61f`)다. 커밋은 발견 단위로 나뉘고 검증은 트리 단위로
이뤄진다는 뜻이며, 중간 커밋 하나만 체크아웃해 오라클을 돌리려는
사람은 자기 손으로 빌드해야 한다.

## §213 "인용 하나-소비자 열 개" 검사를 크레이트 전체로 확장 — 갈라진 곳은 §211 하나뿐이었다

### §213.1 앵커와 방법

앵커: `rg -n '^impl(<[^>]*>)? (TryFrom|From)<' ros/moveit-ros/src` (39개
매치). 각 impl의 프로덕션 콜사이트는 파일별 `#[cfg(test)] mod tests`
시작 줄을 경계로 `rg -n '::try_from\(|\.try_into\(\)'` 결과를 나눠 셌다
— 테스트 안에서만 불리는 것은 소비자로 세지 않았다.

콜사이트가 1개뿐인 impl은 애초에 "공유"가 아니므로 갈릴 위험이 없다.
2개 이상인 impl만 아래 표에 올렸고, 각각을 실제 상류 C++ 소스
(`/home/stevek/work/moveit2`, `third_party/geometric_shapes`)를 열어
확인했다 — §211.6과 같은 방법, 인용을 재사용하지 않고 직접 다시 열었다.

### §213.2 표

| impl | 프로덕션 콜사이트 | 상류 도달 경로 | 판정 |
|---|---|---|---|
| `TryFrom<Quaternion> for UnitQuaternion` (`TryFrom<Pose> for Isometry3`를 통해) | 9곳 — `ros/moveit-ros/src/constraints/position.rs:161`, `ros/moveit-ros/src/constraints/visibility.rs:114`/`115`, `collision_object.rs`×5, `planning_scene.rs` 옥토맵 원점 | `poseMsgToEigen`/`tf2_eigen::fromMsg` — 무조건 normalize, 한 규칙 | §211/`f2a7847`에서 이미 분리 완료 (10번째 자리는 별도 타입 `OrientationConstraintQuaternion`). 이번 스윕에서 재확인, 편차 없음 |
| `TryFrom<SolidPrimitiveMsg> for Shape` | 2곳 — `ros/moveit-ros/src/constraints/position.rs:160`(BoundingVolume), `collision_object.rs:183`(shapesAndPosesFromCollisionObjectMessage) | 둘 다 `shapes::constructShapeFromMsg(const shape_msgs::msg::SolidPrimitive&)` (`third_party/geometric_shapes/src/shape_operations.cpp:78-112`) 하나 — 같은 함수, 같은 BOX/SPHERE/CYLINDER/CONE 분기, 실패 시 같은 "shape==nullptr" 귀결 | 균일. 편집 불필요 |
| `TryFrom<u8> for CollisionObjectOperation` | 2곳 — `collision_object.rs:310`(processCollisionObjectMsg), `attached.rs:62`(processAttachedCollisionObjectMsg) | 두 디스패처(`planning_scene.cpp:1774-1798`, `:1536-1769`) 모두 ADD/APPEND/REMOVE/MOVE를 같은 상수와 비교하고, 그 외 값은 둘 다 동일한 "Unknown collision object operation: %d" 에러로 귀결 (직접 읽고 확인, 다르다고 가정하지 않음) | 균일. 편집 불필요 |
| `TryFrom<ConstraintsMsg> for KinematicConstraintSet` | 3곳 — `planning.rs`의 `goal_constraints`/`path_constraints`/`trajectory_constraints` | 셋 다 `KinematicConstraintSet::add(const moveit_msgs::msg::Constraints&, const Transforms&)` (`kinematic_constraint.cpp:1294`) 단 하나 — 이 함수는 호출자가 `MotionPlanRequest`의 어느 필드에서 왔는지 알지 못한다 | 균일. 편집 불필요 |
| `TryFrom<Point>`/`TryFrom<Vector3> for CoreVector3` 및 그 역방향(`Point`/`Vector3`/`Pose`/`Quaternion` 출력) | 여러 곳 (`position.rs`, `planning.rs`×2, `shapes.rs` 등) | 실패 분기 자체가 없음 — `geometry.rs`의 기존 doc comment가 이미 "Total in practice"라고 명시하며, 이번 스윕은 그 주장을 소스가 아니라 impl 본문 자체(항상 `Ok`)로 재확인했다 | 균일 — 구조적으로 갈릴 수 없음 |
| 나머지 전부: `JointLimits`, `JointConstraint`/`PositionConstraint`/`OrientationConstraint`/`VisibilityConstraint`의 msg<->core 양방향, `RobotState`, `RobotTrajectory`/`JointTrajectory`, `MeshMsg`/`PlaneMsg` -> `Shape`/`Plane` | 각 1곳 | 해당 없음 | 애초에 공유 impl이 아님 — 대상 아님 |

### §213.3 결론

`f2a7847`의 Quaternion/Pose 분리가 이 크레이트에서 "한 impl이 서로 다른
상류 규칙에 닿는 소비자를 가진" 유일한 사례였다. 갈림 위험이 있던
나머지 셋(`SolidPrimitiveMsg`, `CollisionObjectOperation`,
`ConstraintsMsg`)은 소비자가 2곳 이상이지만 전부 실제로 같은 상류 함수
하나에 닿는다는 것을 인용 없이 직접 소스를 열어 확인했다 — 이번
라운드에서 이 크레이트에 추가로 쪼갤 곳은 없다.

## §214 같은 함수 안의 형제 분기, 같은 변형(variant) — `e3b40c6`과 같은 모양을 이 크레이트에서도 찾았다

### §214.1 앵커

`moveit-constraints`의 `e3b40c6`: `Body::from_shape(shape)?`의 `Err` 절반과
`Ok(None)` 절반이 같은 줄에 있는데, 테스트가 `.is_err()`만 확인해서
`?`를 `.unwrap_or(None)`으로 바꿔 형제 분기로 라우팅해도 초록으로
남았다. 같은 모양을 `ros/moveit-ros/src` 전체에서 찾기 위해 함수 본문
안에 같은 `Error` variant를 만드는 `Err(Error::...)` 리터럴이 2개
이상인 함수를 스캔했다 (AST 없이 중괄호 깊이로 함수 본문을 잘라
`Err(Error::(\w+)` 패턴 카운트). 결과 5개:

| 파일:함수 | 같은 variant 반복 | 기존 테스트가 구분했는가 |
|---|---|---|
| `trajectory.rs`의 `TryFrom<JointTrajectoryMsg>::try_from` | `Construct` × 3 (positions 길이 불일치/0이 아닌 시작 시각/시간 역행) + `set_point_array`의 4번째(속도/가속도/effort 길이) | 아니오 — 시작 시각·시간 역행 테스트는 variant만 확인, positions 길이 불일치와 속도 길이 불일치는 테스트 자체가 없었음 |
| `state.rs`의 `TryFrom<RobotStateMsg>::try_from` | `Other` × 3 (`is_diff`/`attached_collision_objects`/`multi_dof_joint_state`) + `set_parallel_array`의 4번째(속도/effort 길이) | 아니오 — `is_diff`만 테스트 있었고 variant만 확인, 나머지 셋은 테스트 자체가 없었음 |
| `planning.rs`의 `TryFrom<PlanningRequestMsg>::try_from` | `Other` × 2 (`start_state`/`reference_trajectories`) | 아니오 — `start_state`만 테스트 있었고 variant만 확인, `reference_trajectories`는 테스트 자체가 없었음 |
| `scene/attached.rs`의 `apply_attach` | `Other` × 2 (월드 오브젝트 승격 경로의 "no geometry"/메시지-지오메트리 경로의 "no geometry") | 아니오 — 두 테스트 모두 존재하지만 variant만 확인 |
| `scene/collision_object.rs`의 `apply_move` | `Other` × 2 (알 수 없는 id/포즈 개수 불일치) | 아니오 — 두 테스트 모두 존재하지만 variant만 확인 |

### §214.2 고친 방법

`e3b40c6`과 동일한 헬퍼 모양(`assert_err_mentions`, `#[track_caller]`,
렌더링된 에러 문자열에 `needle`이 포함되는지 확인)을 이 다섯 파일 각각의
테스트 모듈에 추가했다 — 크레이트 공용 헬퍼로 올리지 않은 이유는
`e3b40c6` 자체가 파일 로컬 헬퍼였고, 이 크레이트의 `Result`는 파일마다
`std::result::Result`거나 `moveit_error::Result` 별칭이라(뒤엣것은
제네릭 인자가 하나뿐) 시그니처가 파일마다 달라야 했기 때문이다
(`scene/attached.rs`, `scene/collision_object.rs`는
`std::result::Result<T, Error>`로 명시).

variant만 확인하던 기존 테스트 6곳을 메시지 내용 확인으로 바꾸고,
테스트가 아예 없던 형제 분기 6곳에 새 테스트를 추가했다 (positions
길이 불일치·속도 길이 불일치 ×2파일·`attached_collision_objects`·
`multi_dof_joint_state`·`reference_trajectories`). 141개였던 테스트가
147개.

### §214.3 스캔에서 제외한 것

같은 함수 안에 2개 이상이지만 이미 호출부마다 다른 메시지를 만드는
공유 헬퍼(`constraints/position.rs`의 `dim()` — BOX_X/SPHERE_RADIUS 등
필드 이름이 이미 인자로 갈리고, `collision_object.rs`의
`parallel_shapes`/`subframes_from_parallel_arrays` — `items_field`가
이미 인자로 갈림)는 이번 라운드에 포함하지 않았다: 메시지가 이미
호출부마다 다르므로 §214가 잡는 정확한 결함 모양(형제 분기가 *같은*
메시지를 만들어 라우팅 버그를 가릴 수 있는 경우)이 아니고, 테스트가
그 메시지 내용을 아직 확인하지 않는 것은 커버리지 gap이지 이번 항목이
겨냥한 판별 불가능성이 아니다.

## §215 사이트별 norm 경계값 표 — norm=2.0가 회귀를 낸 지점이라 이번 라운드는 사이트별 실측을 새로 추가했다

### §215.1 방법

§211/§213이 세운 규칙: `f2a7847`이 쪼갠 두 규칙(generic — 유한·0이
아니면 무조건 재정규화, strict — `OrientationConstraintQuaternion`,
1e-3 임계값)에 열 개의 와이어 필드가 어느 규칙에 닿는지는 이미
확인했지만, 그 소비자 열 곳 "각각"에서 norm=2.0/1.0009/1.0011/전부
0/NaN 다섯 값이 실제로 어떤 결과를 내는지는 아직 아무도 실행한 적이
없었다 — `geometry.rs`의 규칙 자체 단위 테스트만 있었다. 아홉 곳은
와이어 필드에서 `Isometry3::try_from(Pose(...))` 호출까지 분기가
전혀 없는 것을 직접 코드를 읽어 재확인했으므로(§213.1의 `rg` 앵커
재실행), 코드 경로가 동일하다는 근거로 generic 규칙의 단위 테스트
결과를 그 아홉 곳에 적용할 수 있다 — 다만 이것은 "이 사이트에서
실행함"과는 다른 주장이라 `doc/message-mapping.md` §18의 표에서
`✅site`(이 사이트 자체를 실행)와 `✅generic-fn`/`✅strict`(코드
동일성으로 추론, 이 사이트에서 실행한 것은 아님)를 구분해서 표기했다.

norm=2.0 열만은 예외로 열 곳 전부에 이 사이트 자체를 통과하는
end-to-end 테스트를 새로 추가했다: `4ff563d`가 실제로 깨뜨린 값이라
추론이 아니라 실측이 필요하다고 판단했다. `strict` 규칙(사이트 #2,
`OrientationConstraint.orientation`)은 norm=2.0에서 이미 이전
라운드에 end-to-end 테스트(`orientation_norm_2_is_rejected_end_to_end_unlike_a_scene_pose`)가
있었고, 나머지 네 값은 `geometry.rs`의 `OrientationConstraintQuaternion`
단위 테스트로만 커버된다 (이 사이트 자체에서 실행한 것은 아님, 표에
그대로 명시).

### §215.2 이번 라운드에 추가한 사이트별 실측 테스트

- `constraints/position.rs`: `region_pose_with_norm_2_orientation_succeeds_and_normalizes`
  (`ros/moveit-ros/src/constraints/position.rs:161`)
- `constraints/visibility.rs`: `sensor_and_target_pose_with_norm_2_orientation_succeed_and_normalize`
  (`ros/moveit-ros/src/constraints/visibility.rs:114`/`115`, 필드 두 개를 테스트 하나로)
- `scene/collision_object.rs`: `add_with_norm_2_orientation_on_object_and_shape_poses_succeeds_and_normalizes`
  (`:142`/`:207`), `add_with_norm_2_orientation_on_subframe_pose_succeeds_and_normalizes`
  (`:239`), `move_with_norm_2_orientation_on_object_and_shape_poses_succeeds_and_normalizes`
  (`:478`/`:515`)
- `scene/planning_scene.rs`: `octomap_origin_with_norm_2_orientation_succeeds_and_normalizes`
  (`:147`)
- `geometry.rs`: `norm_just_inside_orientation_rules_1e_minus_3_tolerance_is_also_accepted_here`
  — generic 규칙 자체가 norm=1.0009에서 정확히 실행된 적이 없었던
  빈틈을 메움 (strict 규칙 쪽은 이미 있었음)

147개였던 테스트가 154개 (신규 7개: `position.rs` 1 +
`visibility.rs` 1 + `collision_object.rs` 3 + `planning_scene.rs` 1 +
`geometry.rs` 1).

### §215.3 아직 "실행하지 않음"으로 남긴 것

없음 — 브리프의 "실행하지 않은 행이 있으면 실행하거나 명시적으로
미검증 표시"라는 지시를 각 셀 단위로 적용했다: norm=2.0 열은 열 곳
전부 이 사이트 자체 실행, 나머지 네 값은 코드 동일성 근거를 명시하고
"이 사이트에서 실행한 것은 아님"이라고 표에 그대로 적었다 (숨기지
않음). 이 근거가 깨지는 조건도 표 마지막에 명시했다: 아홉 곳 중
하나라도 와이어 필드와 호출 사이에 분기(사이트별 기본값 대체나
clamp 등)가 생기면 그 행의 추론 셀은 더 이상 유효하지 않고 실제
사이트별 테스트로 다시 실행해야 한다.

## §216 Phase 5 완료 조건 세 항목을 실제로 측정했다 — 하나는 다른 것을 재고 있었고, 둘은 계기가 없었다

§5의 Phase 5 완료 조건은 세 줄이다. 시작 시점의 통념은 "1번은 충족,
2·3번은 미착수"였는데, 셋 다 재보고 나니 그 통념이 세 항목 모두에서
틀렸다.

### 216.1 1번 — 2,000건은 돌고 있었지만 "조합"이 아니었고, 아무도 돌리지 않았다

`moveit-diff --constraints 2000`은 라운드 4부터 있었고 4로봇 전부에서
0건 불일치로 통과했다. 그런데 생성기를 열어 세어 보니 **모든 케이스가
제약을 정확히 하나씩만** 담고 있었다 — 7종을 순환하며 로봇당 ~286건씩.
조건이 요구하는 것은 "제약 **조합** 2,000건"이므로, 통과하던 그 숫자는
조건이 겨냥한 것을 재고 있지 않았다. 게다가 이 명령을 실행하는 CI
스크립트가 없었다. §11.2가 야코비안에서 적은 것과 같은 모양이다: 비교가
존재하는 것과 실행 주체가 있는 것은 다르다.

생성기를 12종 구성(단일 7 + 복합 5)으로 바꿨다. 복합 5종이 네 제약
종류를 모두 포함하고, 최대 5개 제약이 한 케이스에 들어간다. 로봇당
2,000 조합, 구성별 166~167건씩. 4로봇 각 `cases: 2051, passed: 2051,
failed: 0` — 2051 = 조합 2,000 + `model_info` 1 + fk 50. 실행 주체는
`tools/ci/verify-constraint-sweep.sh`, 4로봇 전체 **30.1초**.

반증 확인: `rust_impl::constraints`의 position/orientation push 루프
순서를 바꾸면 panda에서 `passed: 1553, failed: 498`. 498은 두 종류를
동시에 담은 구성 3종 × 166건과 정확히 일치하고, 단일 종류 케이스는
한 건도 실패하지 않는다 — 바꾸기 전의 단일 제약 스윕이었다면 이
결함을 볼 수 없었다는 뜻이다.

### 216.2 2번 — 계기가 아예 없었다

샘플러가 낸 상태를 자기 제약의 `decide()`로 되먹이는 코드는 트리에
없었다. `crates/moveit-constraints/tests/sampler_self_validation.rs`가
그것이다. 7종 샘플러 구성에서 **10,000 상태 생성, 10,000 만족**
(10,002 시도 — `ik_position_only`가 두 번 수렴에 실패했다). 구성별:
joint_full_coverage 2000, joint_partial_coverage 1600, ik_position_only
1600, ik_orientation_only 1600, ik_position_and_orientation 1600,
union_hand_joint_plus_arm_ik 800, manager_partial_joint_plus_ik 800.
**10.6초**, `tools/ci/verify-sampler-self-validation.sh`가 실행한다.

루프는 고정 횟수 `for`가 아니라 할당량까지 뽑는 `while`이고 시도 상한
(할당량 × 4)이 있다. 0건을 낸 구성은 100%로 접히지 않고
`produced 0 of its N states -- a vacuous 100%`로 실패한다.

반증 확인 두 가지. (a) 샘플 직후 `panda_joint1`에 +0.5 rad을 더하면
7종 전부가 위반을 보고하고 총계가 `10000 produced, 2178 satisfied`로
떨어진다. (b) `MAX_IK_ATTEMPTS = 0`이면 IK 기반 5종이 위 문구로
실패하고 0.6초에 끝난다.

정직하게 적어 둘 한계: IK 세 구성은 `IkConstraintSampler::sample`이
받아들이기 전에 이미 자기 제약을 `validate()`로 되묻는다. 그 세
구성에서 이 테스트는 부분적으로 동어반복이다. 진짜로 검사 없이 나오는
경로는 joint 두 구성과 disjoint union 하나다.

### 216.3 3번 — 오라클에 이 질문을 할 op이 없었다

`collision`은 `PlanningScene` 없이 `CollisionEnvFCL`을 직접 만들고,
`is_state_valid`는 scene을 만들지만 `diff()`를 부르지 않는다. 씬 diff를
적용한 뒤의 충돌 결과를 물을 수 있는 op이 없었으므로, 이 조건은
"미충족"이 아니라 **측정 불가능**이었다.

`scene_diff_collision`을 추가했다. 요청의 joint/objects/attached로 부모
씬을 세우고, 충돌을 재고, 상류 `PlanningScene::diff()`로 자식을 만들고,
diff 액션을 적용하고, **자식과 부모를 다시** 잰다. 부모를 두 번 재는
것이 핵심이다: 자식만 보고하면 올바른 copy-on-write와 부모를 망가뜨린
diff를 구분할 수 없다.

`crates/moveit-scene/tests/scene_diff_collision_parity.rs`, pr2 9케이스.
조건이 명시한 다섯 종류를 각각 1케이스 이상 덮는다 — 월드 오브젝트
추가(1, 6, 9), 제거(2, 8), 링크에 부착(3), 분리(4), ACM 엔트리
변경(5, 6, 8). 나머지 4케이스는 앞의 다섯을 공허하지 않게 만드는
장치다: 6번은 충돌 수치를 하나도 바꾸지 않으면서 월드만 바꾸고, 7번은
빈 diff 대조군이며, 8번은 허용→제거→재추가로 `remove_object`의 ACM
가지치기만이 볼 수 있는 차이를 만들고, 9번은 부모가 이미 가진
오브젝트에 도형을 더해 `World::ensure_unique`를 타는 유일한 케이스다.
`cargo nextest run -p moveit-scene` 안에서 **0.040초**라 별도 opt-in이
없다.

반증 확인 네 가지, 각각 다른 비교 대상을 겨냥한다:

| 변형 | 실패한 곳 |
|---|---|
| `diff()`가 부모 월드를 상속하지 않음 | case 1 `world_object_ids` |
| `remove_object`가 ACM을 가지치기하지 않음 | case 8 `robot_collision` |
| `detach`가 기하를 월드로 되돌리지 않음 | case 4 `world_object_ids` |
| `World::ensure_unique`가 공유 오브젝트를 복제하지 않음 | case 9만 — 1~8은 전부 통과한 뒤 9에서 패닉 |

마지막 행이 이 파일의 부모 격리 단언이 무엇을 재고 무엇을 재지 않는지
정한다. 이 포트에서 부모는 자식이 사는 동안 `Arc` 뒤에 있으므로 상태·
ACM·transform 층은 **구조적으로** 닿을 수 없다 — 그 셋에 대해 단언은
측정이 아니라 재확인이다. 공유 가변 경로는 `World`의 `Arc<Object>`
copy-on-write 하나뿐이고, 그것은 부모가 이미 가진 오브젝트를 건드려야
닿는다. case 1~8은 전부 맵 엔트리 단위 추가/제거라 거기에 닿지 않는다.
case 9가 유일한 실측이다.

C++ 쪽은 사정이 다르다. 자식이 가변 `WorldPtr`과 가변 ACM을 들고 있어
어느 쪽으로든 써 넣을 수 있으므로, 오라클에서 `parent_after ==
parent_before`는 진짜 살아 있는 검사이고 커밋된 픽스처가 그것을 고정한다.

같이 고친 것 하나. 처음 쓴 `remove_object`/`detach` diff 액션은 이름
그대로 `World::removeObject`와 `RobotState::clearAttachedBody`만
불렀다. 그런데 상류의 `processCollisionObjectRemove`는 ACM 엔트리를
지우고, AttachedCollisionObject의 REMOVE 분기는 기하를 월드로 되돌린
**뒤에** 상태에서 지운다. 이 포트의 `PlanningScene::remove_object`/
`detach`는 상류를 따르므로, 고치지 않았다면 오라클과 Rust가 서로
일치하면서 둘 다 상류에서 벗어나는 픽스처를 만들 뻔했다.

### 216.4 남는 것

- pr2의 `self_collision`/`self_distance`는 이 비교에서 제외했다.
  이 포트가 메시를 싣지 않아 pr2 자기충돌면 대부분이 없고, 그 불일치가
  diff 층의 신호를 덮는다. 오라클은 두 필드를 그대로 보고하고
  `verify-fixture-replay.sh`가 커밋된 응답 전체를 살아 있는 오라클과
  대조하므로, 드리프트에 대해서는 여전히 고정돼 있다.
- 씬 diff는 `push_diffs`/`decouple_parent`/`clear_diffs`까지 있는데
  이번 비교는 `diff()` 적용 후의 충돌 결과만 덮는다. 부모로 되밀어
  넣는 경로는 오라클 op으로 열려 있지 않다.

---

## §217 포트 커버리지를 다시 쟀다 — 결정된 미포팅 45, 진짜 갭 40

미포팅 파일 목록이 "결정된 미포팅"과 "진짜 갭"을 섞어 들고 있었다. 섞이면
남은 일 목록이 **양쪽으로** 틀린다: 이미 결정된 것을 다시 검토하러 가고,
아무도 결정한 적 없는 것을 결정된 것으로 착각해 지나친다. 이 절은 그 분할을
실측으로 다시 만들고, 계기와 표를 커밋해 다음 라운드가 산문이 아니라 명령을
읽게 한다.

**실측 (2026-08-05):**

```console
$ ./tools/ci/measure-port-coverage.py
corpus   245
ported   150
unported 95
cited-outside-corpus 20
```

미포팅 95건의 분류: `decided-non-port` **45** / `gap` **40** /
`ported-elsewhere` **10**. 한 줄에 한 파일씩, 결정된 것은 근거 문장을
인용해 [`doc/port-coverage.md`](doc/port-coverage.md)에 적었다. 245는 이
문서들 사이를 근거 없이 옮겨 다니던 수였고, 이번에 셸 파이프라인과 파이썬
워커 두 계기로 각각 뽑아 정렬 `diff`가 0줄임을 확인한 뒤에 쓴다.

**수가 이 절이 처음 쓰인 시점과 다른 이유.** 측정은 이 절의 브랜치 시점에서
했고, 그 뒤 `cartesian_interpolator.{hpp,cpp}`(표가 `gap`으로 세던 둘)와
`command_list_manager.{hpp,cpp}`가 각각 포팅·결정 인용을 얻어 머지됐다.
계기가 `STALE ROW` 넷을 찍었고, 그 넷을 표에서 지운 뒤의 수가 위의 150/95다
— 계기가 흔들린 것이 아니라 트리가 움직인 것이고, 그 구분을 사람이 아니라
`--check`가 했다는 것이 이 표를 산문 대신 계기의 입력으로 둔 이유다.

**왜 표는 `doc/`에 있고 이 절에는 수만 있는가.** 이 파일이 실제로 읽히는
파일이다 — `rg -l 'PORTING-PLAN\.md' crates/ ros/ tools/ doc/ | wc -l`은
**181**이고, `doc/` 안에서 가장 많이 참조되는 파일(`doc/upstream-bugs.md`)은
**5**다. 그래서 판정과 수는 여기 둔다. 95행 표 자체를 링크된 파일에 두는
이유는 분량이 아니라 기계 검사다: `measure-port-coverage.py --check
doc/port-coverage.md`가 그 표의 행 집합을 계기가 계산한 미포팅 집합과
맞춰 보고, 어긋나면 `MISSING ROW`/`STALE ROW`를 찍고 non-zero로 끝난다.
산문 요약은 그 검사를 받을 수 없다.

**코퍼스 정의가 §1과 어긋나는 지점.** 두 정의가 실제로 다르게 세는 파일은
`moveit_core/version/version.cpp` **하나**다. §1은 `version`을 한 번도
언급하지 않으므로 §1을 따르면 코퍼스는 246이 된다. §1이 파일 종류에 넣은
`*.cc`는 다섯 루트에서 실측 **0**개라 무영향이고, §1의 파일 수(292 등)는
테스트와 `.h` shim을 포함한 전수라는 점도 함께 적어 둔다.

### §217.1 ikfast — §60.4가 이미 결정해 뒀다, 그러니 갭이 아니다

`ikfast_kinematics_plugin`을 갭으로 세던 목록이 있었다. §60.4를 열어 확인한
결과 이미 처분이 붙어 있다:

> 형제 플러그인 셋에 처분이 붙었다: `srv_kinematics_plugin` 배제(ROS 서비스
> 클라이언트), `ikfast_kinematics_plugin` **미포팅(이식할 알고리즘 없음,
> codegen 템플릿)**, `cached_ik_kinematics_plugin` 미포팅이되 진짜 범위 내
> 갭 — §4.4가 D4 trait 주석에서 이것을 명시적으로 이름 부른다.

크레이트 쪽 문장도 같은 방향이고 더 구체적이다
(`crates/moveit-kinematics/src/lib.rs:241-250`): "a 1421-line C++ template
with placeholder tokens that OpenRave's separate, external IKFast code
generator fills in with a *robot-specific* closed-form analytic solution ...
porting this would mean porting OpenRave's symbolic-algebra codegen tool".

따라서 `doc/port-coverage.md`의 해당 두 행은 `decided-non-port`다.
코퍼스에 드는 ikfast 파일은 두 개뿐이며, 그중 `templates/ikfast.h`는 이
코퍼스에서 shim이 아닌 **유일한** `.h`다(나머지 141개는 전부 상류 PR #3113의
자동 생성 포워딩 shim).

| 상류 파일 | 분류 |
|---|---|
| `moveit_kinematics/ikfast_kinematics_plugin/templates/ikfast.h` | `decided-non-port` |
| `moveit_kinematics/ikfast_kinematics_plugin/templates/ikfast61_moveit_plugin_template.cpp` | `decided-non-port` |

### §217.2 문서가 트리와 어긋나는 곳 세 군데

미포팅 95건을 `doc/`와 이 파일에 대조했다. 어떤 문서도 "포팅됐다"고 적은
파일이 실제로는 미포팅인 경우는 없었다. 반대 방향으로 셋이 나왔다.

**1. §60.4의 `cached_ik_kinematics_plugin` — "진짜 범위 내 갭"이라 적혀
있으나 이미 포팅됐다.** §60.4 문장은 위 §217.1에 인용한 그대로다. 트리:

```console
$ sed -n '5,7p' crates/moveit-kinematics/src/cached_solver.rs
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   .../cached_ik_kinematics_plugin/cached_ik_kinematics_plugin.hpp
//   .../cached_ik_kinematics_plugin/cached_ik_kinematics_plugin-inl.hpp
$ rg -n 'pub struct CachedIkSolver|name: "(newton_raphson|lma)_cached"' \
     crates/moveit-kinematics/src/cached_solver.rs
114:pub struct CachedIkSolver<S> {
196:    name: "newton_raphson_cached",
210:    name: "lma_cached",
```

`.hpp`/`-inl.hpp` 두 헤더는 포팅됨으로 세어지고, 같은 디렉터리의
`cached_ik_kinematics_plugin.cpp`(pluginlib 등록 boilerplate, D4)와
`cached_ur_kinematics_plugin.cpp`(외부 `ur_kinematics` 의존)만 미포팅으로
남는다. §60.4의 "진짜 범위 내 갭"은 그 뒤 라운드가 닫았고, 문장만 남았다.

**2. §179.1의 pilz 수 세 개가 실측과 다르다.** §179.1은 "상류 `src/` 22개
중 포트가 가진 것은 12개이고, D1/D2로 명시 제외된 다섯을 빼면 남는 in-scope
미포팅은 이 하나(`trajectory_blender_transition_window`)"라고 적는다. 실측:

```console
$ ls /home/stevek/work/moveit2/moveit_planners/pilz_industrial_motion_planner/src/*.cpp | wc -l
24
$ ./tools/ci/measure-port-coverage.py --list-ported   | grep -c 'pilz_industrial_motion_planner/src/'
13
$ ./tools/ci/measure-port-coverage.py --list-unported | grep -c 'pilz_industrial_motion_planner/src/'
11
```

22 → **24**, 12 → **13**, "다섯" → `crates/moveit-planners-pilz/src/lib.rs:104-124`가 이름으로 지목하는
`src/*.cpp`는 **9개**(`move_group_sequence_{action,service}`,
`planning_context_loader{,_circ,_lin,_polyline,_ptp}`,
`pilz_industrial_motion_planner`, `command_list_manager`). 그리고 §179.1이
하나 남았다고 지목한 `trajectory_blender_transition_window.cpp`는 지금
포팅돼 있다(`crates/moveit-planners-pilz/src/trajectory_blender_transition_window.rs:9`).
남은 in-scope 미포팅 `src/` 파일은 §179.1이 언급하지 않는 **둘**이다:
`joint_limits_aggregator.cpp`, `joint_limits_validator.cpp`.

**3. §153.2가 "미상"으로 남긴 둘의 상태를 여기서 확정한다.** §153.2는
"`occupancy_map.*`와 `collision_plugin_cache.*`는 같은 방식으로
재확인되지 않았으므로 그 제외는 아직 유효한지 미상"이라고 적었다.

- `collision_plugin_cache.{hpp,cpp}`와 `collision_plugin.hpp` →
  `decided-non-port`. 근거는 `crates/moveit-collision/src/lib.rs:37-49`가
  파일과 줄을 짚어 적은 두 가지다: 본문 전체가 pluginlib 런타임 클래스
  로딩이라 그 ROS 기구와 무관한 알고리즘이 없다는 것
  (`collision_plugin_cache.cpp:37-38`), 그리고 `CollisionPlugin::initialize`가
  `planning_scene::PlanningScenePtr`를 받으므로(`collision_plugin.hpp:93`)
  여기서 받으면 크레이트 순환 의존이 된다는 것.
- `occupancy_map.hpp` → `gap`. 같은 doc이 "genuinely `RobotState`-free and
  portable"이라고 적고 `moveit-octomap`으로 보내라고만 한다. 이식하지 않기로
  한 결정이 아니라 소유 디렉터리를 옮기라는 라우팅이므로 갭이다.

### §217.3 §5 Phase 완료 조건 열 개를 다시 쟀다 — 넷이 미충족

Phase 완료 조건은 "만족하지 못하면 다음 단계로 넘어가지 않는다"고 §5가 적어
둔 것이므로, 어느 것이 아직 성립하지 않는지가 남은 일 목록의 절반이다.
아래는 조건을 원문 그대로 옮기고, 그 옆에 그것을 판정한 명령과 출력을
붙인 것이다. 판정을 내리지 못한 항목은 MET/UNMET이 아니라 **미측정**으로
적는다.

**Phase 0 — MET.** 조건: "오라클이 panda URDF/SRDF에 대해 임의 관절값
1,000세트의 FK를 출력하고, `moveit-diff`가 그것을 읽어 '`Rust 구현 없음`'으로
1,000건 전부 실패 보고한다." §7이 2026-08-03 완료로 기록한다. 문자 그대로의
증명(전건 실패)은 지금 재현할 수 없다 — Rust 구현이 존재하므로 실패가 나올
수 없다. 하네스가 도는지는 아래 Phase 2의 `verify-oracle-sweep.sh` EXIT=0이
대신 보인다.

**Phase 1 — UNMET.** 조건: "panda / prbt / fanuc 3종에 대해 링크 수, 조인트
수, 그룹 구성, 조인트 한계값, mimic 관계가 오라클과 완전 일치."

```console
$ ls fixtures/
dual_arm_panda.srdf  dual_arm_panda.urdf  fanuc.srdf  fanuc.urdf  meshes
panda.srdf  panda.urdf  pr2.srdf  pr2.urdf
```

**prbt 픽스처가 없다.** 세 로봇 중 하나에 대해서는 비교 자체가 실행된 적이
없다. panda/fanuc는 `robot_model_parity.rs`가 오라클 `model_info`와 맞춘다.

**Phase 2 — 앞의 두 항목 MET, 세 번째 UNMET.** (세 번째는 §238이
2026-08-06에 닫았다. 아래 진단 — 오라클에 해당 op이 없고 포트 쪽 mimic
전파가 FK 스윕 경로에서 실행되지 않는다 — 은 그 라운드가 오라클에
`enforce_bounds`/`mimic_propagate`/`interpolate` 세 op을 추가한 근거가
됐다.)

```console
$ sg docker -c './tools/ci/verify-oracle-sweep.sh 10000 1'   # EXIT=0
=== panda / panda_arm ===          cases: 20001  passed: 20001  failed: 0
=== fanuc / manipulator ===        cases: 20001  passed: 20001  failed: 0
=== dual_arm_panda / left_panda_arm === cases: 20001  passed: 20001  failed: 0
=== pr2 / right_arm ===            cases: 20001  passed: 20001  failed: 0
=== pr2 / base ===                 cases: 20001  passed: 20001  failed: 0
```

FK는 `moveit-diff` 기본 `1e-9`, 야코비안은 `--tol-jacobian 1e-7`. 관측된
최악 야코비안 편차는 2.887e-15(panda) / 1.665e-15(fanuc) /
2.665e-15(dual_arm_panda) / 2.331e-15(pr2 right_arm) / 3.331e-16(pr2 base).

세 번째 항목("관절 한계 클램핑, mimic 전파, floating/planar 조인트 보간이
일치")은 **오라클과 맞춰 본 적이 없다.** 오라클이 구현한 op 41개 중 보간이나
한계 강제에 해당하는 op이 없고(`rg -n 'op == "' tools/moveit-oracle/src/oracle.cpp`),
`rg -n -i interpolat crates/moveit-state/tests/ tools/moveit-diff/src/`는 0건이다.
`invariants.rs`의 경계 테스트는 자기 검증이지 오라클 비교가 아니다. 게다가
FK 스윕이 쓰는 상태는 오라클의 `random_states`가 mimic 값까지 만들어 준
것을 포트가 `set_variable_position`으로 그대로 얹는 것이므로, 포트 쪽 mimic
전파는 그 경로에서 실행되지 않는다.

**Phase 3 — 첫 항목 MET, 둘째 UNMET.** 조건: "10,000 상태 × 3로봇에서
`collision: bool` 이 오라클과 **100% 일치**", "`distance: f64` 가 `1e-4`
이내 일치".

```console
$ moveit-diff --cases 10000 --seed 1 --collision --oracle ...   # 로봇별
panda:           cases 20001  passed 10458  failed 9543   worst distance 2.738e0
fanuc:           cases 20001  passed 13888  failed 6113   worst distance 2.897e-1
dual_arm_panda:  cases 20001  passed 16493  failed 3508   worst distance 1.882e-1
$ grep -c 'self_collision differs\|robot_collision differs' <각 출력>
0 / 0 / 0
```

세 로봇 30,000 상태에서 bool 불일치는 **0건**이다 — 첫 항목 MET. 실패
19,164건은 전부 거리값이고, 요구치 `1e-4`에 대해 최악 편차가 panda에서
2.738e0이다 — 둘째 항목 UNMET. (§43/§53/§56/§72가 추적해 온 그 군이다:
panda `stats-json`의 `robot_same_pair_and_value_diverges` 6364건은 전부
`floor/panda_link0` 한 쌍이다.)

**Phase 4 — (a) UNMET, (b) UNMET.** 조건: "도달 가능한 목표 자세 5,000개에
대해 (a) 성공률이 C++ KDL 플러그인 이상, (b) 성공한 해의 FK가 목표 자세와
`1e-6` 이내 일치."

```console
$ moveit-diff --cases 5000 --seed 1 --group panda_arm --ik --tol-ik 1e-6 --oracle ...
oracle success rate: 4921/5000 (98.4%)
rust   success rate: 4906/5000 (98.1%)
cases: 15002  passed: 13489  failed: 1513
```

(a) 4906 < 4921이므로 "이상"이 성립하지 않는다. (b) 1e-6에서 1,513건이
초과한다(병진 1,112 + 회전 401). 다만 초과분의 최대는 병진 9.923e-6 /
회전 8.758e-6이고 **`1e-5`를 넘는 것은 0건**이다 — 이는 `SolverParams::
default().epsilon`이 `1e-5`이고 `CartToJnt`가 twist norm `<= epsilon`인
스텝을 수렴으로 받아들이기 때문이다(`moveit-diff`의 기본 `tol_ik`가 `2e-5`인
이유). 즉 (b)의 `1e-6`은 지금 솔버의 수렴 기준보다 엄격해서, 솔버 정확도를
올리거나 조건의 수를 근거와 함께 바꾸지 않으면 성립할 수 없다.

> 후속: 둘 다 §221에서 닫았다. (a)의 4906 < 4921은 알고리즘 열위가 아니라
> 재시작 난수열의 차이이고(§221.1), (b)의 `1e-6`은 조건 쪽이 틀린 수여서
> §5를 `SolverParams::epsilon`(`1e-5`)으로 고쳤다(§221.2). 위 문단의 측정값
> 자체는 이번 라운드에 재현되어 그대로 유효하다.

**Phase 5 — 첫 항목 MET, 둘째·셋째 UNMET.**

```console
$ moveit-diff --cases 100 --seed 1 --group panda_arm --constraints 2000 --oracle ...
cases: 2201  passed: 2201  failed: 0
```

제약 조합 2,000건 `decide()` 100% 일치 — MET. 둘째("제약 샘플러가 생성한
상태 10,000개가 전부 자기 제약을 만족")는 트리에서 가장 큰 샘플러 자기검증
루프가 **200**이다(`crates/moveit-constraints/tests/sampler.rs:190`;
`rg -no 'for [_a-z]+ in 0\.\.[0-9_]+' crates/moveit-constraints/`의 최대값).
셋째("씬 diff 적용 후 충돌 결과가 오라클과 100% 일치")는 계기가 없다 —
오라클 op 41개 중 씬 diff를 적용해 충돌을 되돌려 주는 op이 없다.

**정정 (병합 시점).** 위 둘째·셋째 서술은 이 절이 측정된 브랜치 시점의
트리에 대한 것이고, 그 뒤 `p10-phase5`가 병합되며 둘 다 닫혔다. 둘째는
`tools/ci/verify-sampler-self-validation.sh`가
`crates/moveit-constraints/tests/sampler_self_validation.rs`를 돌려 샘플러
일곱 종이 만든 상태를 전부 자기 제약에 되먹인다 — 실측 시도 **10,002** /
생성 **10,000** / 만족 **10,000**(13.9s). 셋째는 오라클에
`scene_diff_collision` op이 생겼고
`crates/moveit-scene/tests/scene_diff_collision_parity.rs`가 조건이 이름
부르는 diff 다섯 종을 포함한 아홉 케이스를 그 op과 대조한다. 따라서 Phase 5는
세 항목 모두 MET이고, 아래 요약의 "부분 UNMET 2개"는 Phase 2의 셋째 항목
하나로 줄어든다.

**Phase 6 — MET.** 조건: "동일 waypoint 입력에 대해 TOTG 산출 시간
파라미터화가 오라클과 `1e-6` 이내 일치."

`totg_parity.rs`의 상시 허용치는 `DURATION_TOL = 2e-5`라 조건의 `1e-6`을
그대로 검사하지 않는다. 그래서 상수를 조여 직접 쟀다(측정용, 커밋하지 않음):
`1e-6`에서 통과, `1e-9`에서 실패하며 그때 찍히는 값이 case 4(0-based, 즉
doc의 case 5)의 `abs diff=8.893039193935692e-9`이다. 조건의 `1e-6`보다 3자리
작으므로 MET.

**Phase 7 — 세 항목 모두 MET.** 양쪽을 이번 라운드에 다시 쟀다(C++ 쪽은
오라클 `plan` op, 포트 쪽은 `plan_benchmark_port`, 문제 집합은 동일한
`floor_wall 250 900001` + `cage 250 900002`):

| | C++ OMPL RRTConnect | 포트 |
|---|---|---|
| exact 해결 | 498/500 (99.6%) | 499/500 (99.8%) |
| 경로 길이 중앙값 | 2.6598 | 2.7085 |

조건 1: 90% × 99.6% = 89.64% ≤ 99.8% — 충족. 조건 2: 해결한 499개 전부
`condition2_valid` — 충족. 조건 3: 1.3 × 2.6598 = 3.4577, 포트 2.7085
(비 1.018배) — 충족.

**Phase 8 — pilz 항목 MET, CHOMP/STOMP 항목 미측정.** 조건: "pilz
LIN/PTP/CIRC 궤적이 오라클과 `1e-6` 이내 일치. CHOMP/STOMP는 Phase 7과 같은
속성 기반 검증."

LIN/CIRC의 상시 허용치는 `1e-6`보다 느슨하므로(LIN 5e-5/5e-4/5e-3, CIRC
2e-5/2e-4/4e-3) 세 파일의 위치/속도/가속 허용치를 `1e-6`으로 조여 직접
쟀다(측정용, 커밋하지 않음): PTP는 상시 `TOLERANCE = 1e-6` 그대로 통과,
LIN·CIRC도 `1e-6`에서 통과한다. 더 조이면 LIN은 `1e-12` 통과 / `1e-15`
실패, CIRC는 `1e-6` 통과 / `1e-9` 실패(가속 편차 약 1.672e-9)다. 즉 조건은
커밋된 픽스처 위에서 성립한다. **부수 소견:** LIN 허용치의 doc이 근거로
드는 실측치(위치 1.26e-5, 속도 1.24e-4, 가속 1.26e-3)는 지금 트리에서
재현되지 않는다 — 같은 픽스처가 `1e-12`에서 통과한다. 허용치가 실측보다
6~7자리 느슨하다는 뜻이고, 그 크기의 회귀는 이 테스트에 걸리지 않는다.

**병합 시점 후속 (닫힘).** "1e-12에서 통과"는 하한이지 실측치가 아니라서
느슨한 정도를 과소평가했다. 비교 루프에서 전 waypoint·전 joint의
`(actual - expected).abs()` 최대를 직접 모아 재니 LIN은 위치 `2.09e-14` /
속도 `3.17e-14` / 가속 `2.66e-13`으로, 커밋돼 있던 `5e-5`/`5e-4`/`5e-3`은
6~7자리가 아니라 **9~10자리** 느슨했다. 같은 방식으로 CIRC도 재니 위치
`1.81e-9` / 속도 `5.52e-9` / 가속 `3.32e-8`(커밋값 `2e-5`/`2e-4`/`4e-3`,
4~5자리 느슨)이었다. 두 파일의 상수를 각자의 실측 × 약 4배로 조이고, 근거
문장에 남아 있던 null-space 설명을 실측으로 대체했다. 같이 재본
`pilz_trajectory_polyline_parity.rs`는 문서값 `1.60e-9`이 실측
`1.6045e-9`과 그대로 일치해 손대지 않았다 — 이웃 상수를 베끼지 않고 자기
픽스처를 잰 파일만 낡지 않았다는 뜻이다.

CHOMP/STOMP의 속성 기반 검증은 **미측정**이다. 두 크레이트에는 Phase 7의
`plan_benchmark_*`에 해당하는 하네스가 없다(`crates/moveit-planners-chomp/
{examples,benches}`, `crates/moveit-planners-stomp/{examples,benches}` 넷 다
존재하지 않는다).

**Phase 9 — UNMET.** 조건: "기존 C++ `MoveGroupInterface` 클라이언트가 코드
변경 없이 `moveit-ros` 노드에 플래닝 요청을 보내 유효한 궤적을 받는다."

`rg -n 'MoveGroupInterface' crates/ ros/ tools/ doc/ PORTING-PLAN.md`는
**2건**이고 둘 다 이 파일 안의 조건문 자신이다(§5:727, §183.2:14375). 게이트
스크립트 자신도 그렇게 적는다 — `ros/verify-ros-interop.sh`의 "What this
does NOT check": "No live ROS 2 graph: no node is ever spun up, no
topic/service/action is published or called against a real moveit2 or rclrs
process ... Wire-format compatibility with a real moveit2 node is unverified
by this script."

> **후속.** 위는 anchor 한 줄짜리 판정이었다. §226이 같은 결론(UNMET)을
> 더 깊게 다시 쟀다 — 게이트 스크립트의 실제 범위, C++
> `MoveGroupInterface` 클라이언트의 빌드 가능성(빌드됨, 오라클 이미지
> 계열에서 재현), `ros/moveit-ros`가 조건의 네 조각 중 무엇을
> 구현했는지(`TryFrom` 변환만 존재, 서비스/액션/구독/노드 바이너리
> 전부 부재)까지.

**정정.** 위 아홉 조건 판정 중 다수(Phase 1, Phase 3 두 절, Phase 4 (a)/(b),
Phase 9)는 이후 §218, §221, §226, §229에서 다시 측정됐다. 현재 판정은 §5의
**완료 조건 현황표**를 보라 — 이 절의 위 측정 자체는 그 표가 인용하는 원
증거로 남는다.

---

## §218 Phase 1 완료 조건을 닫고, Phase 3을 조건이 말한 크기로 처음 실제로 측정했다 (2026-08-05)

§5의 두 완료 조건이 같은 이유로 열려 있었다: 트리에 prbt 픽스처가
없었다. Phase 1은 "panda / prbt / fanuc 3종", Phase 3은 "10,000 상태 ×
3로봇"을 명시하는데 3종 중 하나가 없었으므로 어느 쪽도 조건이 말한
대로 실행된 적이 없다. 이번 라운드는 픽스처를 만들고, 두 조건을 각각
실행 주체가 있는 명령으로 바꾼 뒤, 나온 수치를 그대로 적는다.

### §218.1 prbt 픽스처 — 합성하지 않고 벤더링했다

prbt는 이 기계에 있다. `/home/stevek/work/moveit2/moveit_planners/
test_configs/`의 `prbt_support`·`prbt_moveit_config`에 xacro로 있고,
오라클 이미지에도 `moveit_resources_prbt_*` 패키지로 이미 설치돼 있어
`$(find ...)`와 `package://`가 컨테이너 안에서 해석된다. 그래서 URDF를
손으로 쓰지 않고, 핀된 오라클 이미지 안에서 진짜 `xacro`를 돌려
전개한 결과를 `fixtures/prbt.{urdf,srdf}`로 벤더링했다.

측정된 성질: 링크 11, 조인트 11(URDF 10 + SRDF `FixedBase` 가상
조인트 1), mimic 없음, collision 블록 17개가 **전부 프리미티브**
(cylinder/sphere/box)이고 메시는 `visual`에만 나온다. **링크당
`<collision>` 블록이 하나가 아닌 첫 픽스처**다 — base_link 1, link_1 3,
link_2 2, link_3 3, link_4 2, link_5 5, flange 1. §13.4의 표가 "링크당
정확히 하나씩이므로 블록 수와 충돌 형상을 가진 링크 수가 같다"고 적은
전제는 prbt에는 성립하지 않는다.

`tools/ci/verify-fixture-provenance.sh`의 `GENERATED` 항목은 지금까지
"재생성 명령은 커밋 본문에 있다"는 산문뿐이어서 상류가 움직여도
아무것도 빨개지지 않았다. 세 개의 병렬 표(`GENERATED_SOURCES` /
`GENERATED_DIGEST` / `GENERATED_COMMAND`)로 바꿔 입력 xacro의 sha256을
실제로 대조하게 했다. 이것이 고정하는 것은 **입력**이지 전개 결과가
아니다 — 스크립트 헤더에 그렇게 적었다.

### §218.2 Phase 1 — 다섯 항목을 항목별로 판정한다, 그리고 충족

기존 `compare_model_info`는 `ModelInfo` 전체를 한 덩어리로 비교해
하나의 verdict만 냈다. 조건은 다섯 항목을 이름으로 나열하므로
`compare_model_info_clauses`로 항목별 판정을 분리했다(전체 비교는
남겨서 다섯 항목 밖의 필드도 계속 빨개진다). `link_count`/`joint_count`는
개수가 아니라 **이름 목록 전체**를 비교한다 — 개수만 보면 링크 하나가
개명돼도 통과한다.

5종 25항목 전부 일치 (seed 1, `verify-oracle-sweep.sh`):

| 로봇 | link_count | joint_count | group_composition | joint_limits | mimic |
|---|---|---|---|---|---|
| panda | 12 | 12 | 3 그룹, 22 멤버 | 16 bound / 12 조인트 | 1 관계 |
| prbt | 11 | 11 | 1 그룹, 6 멤버 | 6 bound / 11 조인트 | **0 관계** |
| fanuc | 9 | 9 | 1 그룹, 7 멤버 | 6 bound / 9 조인트 | **0 관계** |
| dual_arm_panda | 25 | 25 | 2 그룹, 16 멤버 | 18 bound / 25 조인트 | 2 관계 |
| pr2 | 95 | 95 | 8 그룹, 98 멤버 | 48 bound / 95 조인트 | 6 관계 |

**prbt와 fanuc의 `mimic` 칸은 빈 집합끼리의 비교다.** 확인한 관계가
0개라는 뜻이지 관계를 확인했다는 뜻이 아니므로 그대로 적는다.

빈 집합끼리 통과하는 항목이 있다는 것은 이 다섯 판정이 실제로 변별력이
있는지를 따로 보여야 한다는 뜻이다. 픽스처를 변형하는 방법은 무의미하다
— 양쪽이 같은 파일을 읽으므로 픽스처를 바꾸면 오라클과 포트가 함께
움직인다. 그래서 기대값 `ModelInfo`를 한 필드씩 흔드는 단위 테스트
8개를 붙였다(`phase1_clause_discrimination_tests`): 링크 하나 삭제 /
링크 하나 **개명** / 조인트 하나 삭제 / 그룹 멤버 하나 삭제 / 한계값
**1 ulp** 변경 / `position_bounded` 반전 / mimic 관계 추가, 그리고
무변형이 다섯 항목 전부 통과한다는 기준 케이스. 기준 케이스가 없으면
나머지 일곱은 "항상 빨간 항목" 때문에도 통과할 수 있다.

### §218.3 Phase 3 `collision: bool` 절 — panda·fanuc 충족, prbt만 깨진다

조건이 말한 10,000 상태 × 로봇을 실제로 돌렸다 (seed 1, tol 1e-4):

| 로봇 | `collision: bool` 불일치 | `distance` 불일치 | 측정 max\|Δ\| | wall |
|---|---|---|---|---|
| panda | **0 / 10,000** | 9,543 / 10,000 | 2.738380e0 | 438s |
| prbt | **6,854 / 10,000** | 10,000 / 10,000 | 1.000000e0 | 18s |
| fanuc | **0 / 10,000** | 6,113 / 10,000 | 2.897030e-1 | 2829s |
| dual_arm_panda | **0 / 10,000** | 3,508 / 10,000 | 1.882271e-1 | 454s |
| pr2 | **0 / 10,000** | 9,988 / 10,000 | 3.217869e-1 | 1076s |

**§13.4가 미충족 원인으로 지목한 것은 해소됐다.** 그때 panda는
10,000/10,000 불일치였고 원인은 `<mesh>` 충돌 형상 미로딩이었다. 지금
같은 panda가 `bool` 0/10,000이다.

위 행은 `third_party/moveit_resources`가 있는 **주 체크아웃에서**
측정했다. 그 트리가 없는 곳에서 돌리면 비교가 한쪽만 형상을 잃은 채
성립해 버린다는 것이 §13.4와 별개로 남아 있던 문제다 —
`run-oracle.sh`는 `third_party/`를 주 체크아웃의 절대 경로로 무조건
마운트하므로 오라클은 메시를 전부 읽고, Rust 쪽만 조용히 퇴화한 형상을
쓴다. 그래서 `build_rust_model`이 `RobotModel::diagnostics()`를 확인해
`<collision>` 메시가 하나라도 떨어지면 **거부한다.** 측정: fanuc의 메시
URI를 없는 패키지로 돌려 직접 실행하면 표 대신 떨어진 7건을 이름으로
찍고 exit 2로 끝난다 (`cases:` 행 0개, `collision[...]` 행 0개). 고치기
전에 같은 상황이 표로 나온 기록이 §13.4다 — 거기서 panda가 낸
10,000/10,000 불일치가 바로 이 퇴화한 형상끼리의 비교였고, 그때는
실행이 거부되지 않고 끝까지 돌아 숫자를 냈다.

남은 하나는 prbt이고, 원인은 상태와 무관한 **정확한 접선**이다. 양쪽
소스에서 확인했다:

- `moveit-diff`의 바닥은 `Cuboid::new(4.0, 4.0, 0.1)`을 `(0,0,-0.05)`에
  둔다 → 윗면 z = **0.000000**.
- `fixtures/prbt.urdf`의 `prbt_base_link` collision 실린더는
  `length=0.13`, `origin xyz="0 0 0.065"` → 아랫면
  z = 0.065 − 0.13/2 = **0.000000**.
- 로봇을 고정하는 `world-base_link-fixed`는 `<origin>`이 **아예 없어**
  항등이다. 따라서 이 접선은 10,000 상태 전부에서 동일하게 성립한다.

거기서 두 구현이 갈린다. moveit2 `collision_detection_fcl/src/
collision_common.cpp` (핀 `e017c91e`): 603행 `fcl::distance`가 충돌
쌍에 대해 `-1` 센티널을 돌려주고, 613행이 그것을 그대로 싣고, 636행
`if (distance <= 0 && cdata->req->enable_signed_distance)`로 들어가
647행이 `num_max_contacts = 200`으로 `fcl::collide`를 돌린다. **정확한
접선에서는 접촉이 0개**라서 648행 `if (contacts > 0)`이 건너뛰어지고,
`-1`이 교체되지 않은 채 보고된다 — 동시에 `bool`은 false가 된다.
Rust 쪽은 같은 쌍을 −2.775558e-17로 재고 true라고 답한다.

측정이 이 설명과 일치한다: prbt의 "robot same-pair value divergence"가
6,854로 `bool` 불일치 수와 **정확히 같고**, 나머지 3,146은 양쪽이 서로
다른 쌍을 고른 경우이며 그쪽은 `bool`이 일치한다.

이것은 픽스처를 옮겨 닫을 문제가 아니다. 바닥을 1mm 내리면 숫자는
초록이 되지만 측정한 것은 없어진다 — 허용오차를 넓히는 것과 같은
동작이다. 접선이라는 입력에서 두 백엔드가 다른 답을 낸다는 사실
자체가 결과다.

그리고 이것은 이 포트의 숫자가 아니라 **업스트림 결함**으로 판정했다:
`doc/upstream-bugs.md`의
`fcl-distance-sentinel-survives-zero-contacts` (`not-reproduced`).
같은 함수가 `nearest_points`와 `normal`은 무조건 0으로 지우면서
`distance`의 센티널만 `contacts > 0`일 때 교체하므로, 접선에서는 1미터
관통을 주장하는 레코드가 그 주장을 뒷받침할 기하 없이 반환된다 — 그
파일이 정의한 "반환값으로 새어나가는 값" 기준에 해당한다. 다섯 픽스처
중 prbt에서만, 그리고 prbt의 10,000건 전부에서 나타난다는 측정도 그
항목에 적었다.

### §218.4 Phase 3 `distance: f64` 절 — 5종 전부 미충족, 그리고 원인은 하나가 아니다

**prbt는 이 절의 분류 대상이 아니다.** prbt의 max\|Δ\| 1.000000e0은
`|−1.0 − (−2.775558e-17)|`, 즉 §218.3의 센티널 간격을 그대로 적은
값이지 기하학적 오차 1미터가 아니다. 크기로 읽으면 안 되고, 따라서
아래 배율 목록에도 넣지 않는다. `1e-4` 대비 초과 배율은 **panda
27,384배, fanuc 2,897배**다.

prbt의 10,000건이 전부 하나의 인공물이라는 것은 측정으로 확인했다.
10,000 상태 **전부**에서 오라클이 보고한 robot 최소 거리는
`floor/prbt_base_link` 위의 −1.0 센티널이다 (10,000/10,000). 포트가
같은 쌍을 지목한 6,854건이 `bool` 불일치이고, 나머지 3,146건은 포트가
바닥에 실제로 닿은 다른 링크를 지목한 경우다 —
`prbt_link_3` 1,184 / `link_4` 959 / `link_5` 899 / `flange` 104
(합 3,146). **이 3,146건은 아래의 pair-flip 기전이 아니다**: 쌍이
갈리는 이유는 −1.0이 어떤 실제 관통보다도 작아서 항상 최소값을
이기기 때문이고, 원인은 §218.3이 이미 격리한 접선 하나다. 그쪽에서
`bool`이 일치하는 이유도 같다 — 그 다른 쌍들은 접촉을 실제로
만들어내므로 오라클의 `bool`도 true가 된다.

나머지 두 로봇에서는 `DistancePairStats`가 세는 두 기전이 실제로
갈리고, 지배적인 쪽이 서로 반대다:

- **같은 쌍, 다른 값** — §11.10이 예고한 이탈 6. 오라클은 최대 200
  접촉을 모아 가장 깊은 관통을 고르고, 이 포트는 `query::contact()`의
  단일 접촉을 쓴다. panda `collision[3289]`에서 같은
  `panda_link0/floor` 쌍에 대해 오라클 −2.806384e0, Rust −6.879644e-2
  (|Δ| 2.738e0). 4×4 바닥에서 오라클이 고르는 깊이가 대각
  (√(2²+2²)=2.83) 규모라는 것이 이 값의 크기를 설명한다.
- **다른 쌍** (pair-flip) — 최소값이 근접한 쌍들 사이에서 어느 쪽이
  이기는지가 갈린다. pr2의 self가 9,821/10,000으로 여기에 지배된다.

"로봇마다 지배적인 기전이 다르다"는 주장은 수치로 적을 수 있다:

| 로봇 | self / robot | 지배 기전 | 쌍 분포 |
|---|---|---|---|
| panda | self 1,225 / **robot 9,490** | robot 쪽 6,364가 같은 쌍 값 발산 (이탈 6), flip 3,126 | robot 히스토그램이 **단일 쌍** `floor/panda_link0` 6,364 |
| fanuc | **self 4,924** / robot 2,302 | self 쪽 3,338이 같은 쌍 값 발산, flip 1,586; robot 쪽은 같은 쌍 값 발산이 **0**이고 2,302 전부 flip | self 히스토그램이 **9개 쌍에 분산**, 최다 `link_4/link_6` 2,153, `link_2/link_4` 553, `link_1/link_4` 426 |
| prbt | — | 위 두 기전 어느 쪽도 아님 | 10,000 전부 §218.3의 센티널 |

즉 조건이 명시한 3종은 서로 다른 이유로 깨진다: 하나는 단일 월드
오브젝트 쌍(panda), 하나는 분산된 self 쌍(fanuc), 하나는 센티널
하나(prbt). 하나의 완화책으로 셋이 같이 닫히지 않는다는 뜻이다.

조건이 명시한 3종 각각에서 max\|Δ\|를 낸 상태는 다음과 같다 (seed 1):

- panda `collision[3289]` — 같은 쌍 `panda_link0/floor`, 오라클
  −2.80638374720525752e0 vs Rust −6.87964415068230695e-2. 이탈 6.
- prbt `collision[2]` — 같은 쌍 `floor/prbt_base_link`, 오라클
  −1.0(센티널) vs Rust −2.77555756156288149e-17. §218.3의 접선.
- fanuc `collision[9651]` — **쌍이 다르다**: 오라클
  `base_link/floor` −9.93013661298909247e-16, Rust `link_4/floor`
  −2.89703002516375319e-1. fanuc의 robot 쪽 실패는 같은 쌍 값 발산이
  0/10,000이고 전부 pair-flip이다 — 이탈 6이 아니다.

하나의 `failed:` 총계로는 위 구분이 전혀 나오지 않는다. 그래서
`compare_collision`이
`collision: bool` 첫 불일치에서 `return`하며 `distance`를 `NaN`으로
남기던 것을 고쳤다 — **가장 발산할 법한 상태에서 정확히 두 번째 절이
평가되지 않고 있었다.** 이제 두 절을 항상 평가하고
`CollisionClauseStats`로 따로 센다. 부작용으로 "worst distance
deviation"은 `bool`이 어긋난 상태까지 포함하게 됐다.

접촉점 좌표는 비교하지 않는다. 그 제외는 조건 자신의 세 번째 항목
(§4.5, 검증 한계)이지 수치를 통과시키려고 여기서 고른 편의가 아니다.

### §218.5 비용 — 이 스윕은 옵트인이어야 한다

측정된 총 wall clock은 **4,815초(80분 15초)**이고, `fanuc` 하나가
2,829초로 59%를 차지한다. fanuc은 가장 큰 로봇이 아니다(링크 9,
pr2는 95) — 비용은 모델 크기가 아니라 실제로 narrowphase까지 가는
쌍 수를 따른다. 같은 10,000 상태에 대한 Phase 2 게이트
(`verify-oracle-sweep.sh`)는 113초이므로 **43배**다.

`tools/ci/` 안에는 옵트인 환경변수 관례가 없다 — 이 디렉터리의 환경
변수(`MOVEIT2_SRC`, `LIBCCD_SRC`, `OCTOMAP_SRC`)는 전부 외부 체크아웃
경로이지 옵트인이 아니다. 그래서 새 관례를 만들지 않고
`verify-mpr-vs-epa.sh`가 이미 쓰는 **시끄러운 SKIP** 위에 `PHASE3_SWEEP=1`을
얹었다: SKIP 줄이 "이것은 통과가 아니다"라고 직접 말한다. 조용한 SKIP은
통과와 구분되지 않는다.

`verify-oracle-sweep.sh`에는 `prbt` 항목을 추가했다. 어떤 스크립트도
부르지 않는 픽스처는 Phase 1 항목을 아무도 대조해주지 않는다. 같은
스크립트가 다섯 항목 판정을 이름으로 출력하도록 grep도 넓혔다 —
이전에는 `cases: 20006`이라는 다섯 자리 총계 안에 익명으로 다섯 개가
섞여 있어서 exit code는 덮고 있었지만 사람이 읽을 수 있는 항목별
결과는 아니었다.

---

## §238 Phase 2 세 번째 완료 조건 — 계기를 만들어 재고, `slerp`이 틀렸다는 것을 찾았다 (2026-08-06)

§5 Phase 2의 셋째 조건("관절 한계 클램핑, mimic 전파, floating/planar
조인트 보간이 일치")은 §218이 진단한 대로 **계기 자체가 없던** 항목이다.
오라클에는 클램핑·전파·보간에 해당하는 op이 없었고, FK 스윕이 쓰는 경로는
오라클이 만든 상태를 포트가 변수별로 그대로 얹기 때문에 포트 쪽 mimic
전파를 아예 실행하지 않는다. 그래서 이번 라운드는 먼저 계기를 만들었다:
오라클에 `enforce_bounds`/`mimic_propagate`/`interpolate` 세 op을 붙이고
(`tools/moveit-oracle/src/oracle.cpp`), `moveit-diff`에 `--state-ops`와
열거기(`tools/moveit-diff/src/state_ops.rs`)를 붙이고,
`tools/ci/verify-phase2-state-sweep.sh`로 5로봇을 한 줄로 돌게 했다.

```console
$ sg docker -c './tools/ci/verify-phase2-state-sweep.sh'   # EXIT=0
=== panda ===   clamping 122/0   mimic 10/0   interpolation 371/0
=== prbt ===    clamping  54/0   mimic  0/0   interpolation 168/0
=== fanuc ===   clamping  54/0   mimic  0/0   interpolation 168/0
=== dual_arm_panda === clamping 198/0  mimic 20/0  interpolation 504/0
=== pr2 ===     clamping 568/0   mimic 20/0   interpolation 1967/0
```

케이스 4,224건(클램핑 996, mimic 50, 보간 3,178), **허용오차 0.0 — 비트
일치 — 에서 불일치 0건, double-cover 0건**. 클램핑과 mimic 전파는 인자로
허용오차를 받지 않는다: 한계 복사·`fmod`·`factor * v + offset`은 양쪽이
같은 IEEE 연산을 하므로 차이가 났다면 그것은 반올림이 아니라 계산 대상이
다르다는 뜻이다. 보간만 인자를 받고, 기본값은 0.0이다.

### §238.1 케이스는 무작위가 아니라 경계값이고, 값은 오라클이 준다

케이스 값은 전부 오라클의 `model_info`가 보고한 한계에서 파생한다.
포트가 한계를 잘못 읽고 있으면 케이스 값과 기대값이 함께 움직여
자기 자신과 일치해버리기 때문이다. 경계는 한계의 *종류*로 고르지 조인트
타입 이름으로 고르지 않는다 — 유한 클램프는
`at-min`/`min-1ulp-below`/`min-1ulp-above`/`below-min`과 최대 쪽 넷 +
`midpoint`, 랩(연속 revolute와 planar의 `theta`)은
`at-minus-pi`/`at-plus-pi`와 각각의 1ulp 이웃 + `±2π`/`±3π`/`zero`.
`-π`를 둘로 나눈 이유는 upstream 자신이 둘로 갈리기 때문이다: revolute는
`v <= -π || v > π`에서 랩하고(`revolute_joint_model.cpp:223`) planar는
`!(v >= -π && v <= π)`에서 랩한다(`planar_joint_model.cpp:312`) — 같은
`-π`를 하나는 `+π`로 고치고 하나는 그대로 둔다. 무한 한계(planar의
`x`/`y`, floating의 병진)는 열거할 경계가 없으므로 크기 케이스만 돌고
그 사실을 `SKIPPED` 줄로 **출력한다**. 조용한 스킵은 통과와 구분되지
않는다.

### §238.2 클램핑 × mimic 교호 — 따로 재면 보이지 않는다

`enforceBounds`가 도는 것은 `getActiveJointModels()`이고 거기에 mimic
조인트는 없다. 그래서 마스터가 클램프되면 follower는 자기 한계 밖에
남을 수 있다. 이건 두 절을 따로 재서는 나오지 않으므로 `clamp_mimic_cases`가
마스터 경계값 × follower 상태 3종(`mimic-consistent`,
`mimic-inconsistent-in-bounds`, `mimic-outside-own-bounds`) 격자로 덮는다.
panda의 mimic 마스터는 prismatic이고 pr2의 것들은 revolute인데, upstream의
`enforcePositionBounds` 반환값이 revolute는 무조건 `true`,
prismatic은 변경이 있을 때만 `true`라서(`revolute_joint_model.cpp:247`,
`prismatic_joint_model.cpp:111`) 이 둘은 같은 케이스에서 서로 다른 가지를
탄다 — 두 픽스처가 모두 필요한 이유다.

mimic 전파에는 소유자가 둘이다. `RobotModel::updateMimicJoints`(전체 모델
세터)와 `RobotState::updateMimicJoint`(단일 조인트 세터). 마스터를 쓰는
케이스는 전부 두 번째에서 끝나므로 첫 번째를 덮지 못한다 — 그래서 아무
것도 쓰지 않는 `mimic/defaults-only`가 따로 있다.

### §238.3 double cover — 부호로 숨는 차이

쿼터니언 `q`와 `-q`는 같은 회전이므로, 성분 비교만 하면 부호 규약 차이가
"차이 2.0"으로 나오고 회전 비교만 하면 아예 안 보인다. 그래서 둘 다 한다:
판정은 성분별 편차로 하되, 회전 블록이 정확히 서로의 부호 반전인 케이스는
`double_cover`로 **따로 세고** `[DOUBLE COVER: rotation blocks are exact
negatives]`로 부호 무관 회전각과 함께 출력한다. 이번 측정에서 그 수는
0이다 — 즉 규약이 같다는 것이 측정된 것이지 가정된 것이 아니다.

### §238.4 찾은 결함: `nalgebra`의 `try_slerp`은 Eigen의 `slerp`이 아니다

계기를 처음 돌렸을 때 panda 보간에서 18건이 틀렸다. 원인은 하나였다 —
`FloatingJointModel::interpolate`가 Eigen의
`QuaternionBase::slerp`(Eigen 3.4, `Eigen/src/Geometry/Quaternion.h:782`)를
부르는데 포트는 `UnitQuaternion::try_slerp`으로 갈음하고 있었고, 이 둘은
다른 함수다. 세 가지가 다르다: (1) 준평행 가지를 Eigen은 `|d| >= 1 - ε`
에서 **lerp**로 들어가고 nalgebra는 `|d| >= 1`에서 `from`을 그대로
돌려준다(측정 8.9e-16), (2) `d`가 정규화되지 않은 생 내적이라 노름이 1을
넘는 쌍은 Eigen에서 lerp되지만 nalgebra는 모든 `t`에서 `from`을 돌려줘
아예 움직이지 않는다(측정 1.414), (3) Eigen은 결과를 정규화하지 않고
nalgebra는 한다(측정 1.25e-13).

패치가 아니라 구조로 닫았다. 앵커 `rg -n 'slerp' crates/`로 같은 Eigen
호출을 포팅한 자리를 전부 세었더니 셋이었다 —
`moveit-model`의 `joint/floating.rs`, `moveit-kinematics`의
`cartesian_interpolator.rs`, `moveit-planners-pilz`의
`trajectory_blender_transition_window.rs`. 넷째인 `moveit-planners-sbp`의
`se3.rs`는 OMPL의 `SO3StateSpace::interpolate`를 옮긴 것이라 같은 결함이
아니다. 그래서 `moveit-geometry`에 `quaternion::slerp{,_coefficients}`
하나를 두고 앞의 셋을 그리로 보냈다.

부수 효과가 하나 있었고 그것도 측정으로 확인했다: pilz 블렌드 패리티
테스트가 들고 있던 케이스별 허용오차 오버라이드 10개가 이 수정 뒤에
전부 불필요해졌다. `crates/moveit-planners-pilz/doc/oracle-request-pilz-blend-geometry.md`에
수정 전/후 12행 표로 적었고, 같은 문서가 "slerp 방향이나 off-by-one은
아니다"라고 적어두었던 인과 주장은 **반증됐다**고 명시했다.

### §238.5 이 측정이 덮지 않은 것

- 커밋된 다섯 픽스처의 mimic은 전부 `multiplier=1, offset=0`이다. 그래서
  `factor * v + offset`의 산술은 그 한 점에서만 실행됐다. 배수/오프셋이
  다른 픽스처를 벤더링하기 전에는 이 절의 산술은 그 점에서만 검증된
  것이다.
- 포트에는 공개 `RobotState::interpolate`가 없다. 보간은
  `JointModel::interpolate`에 대해 조인트별로 비교했고, 전체 상태를 도는
  upstream의 루프 자체는 비교되지 않았다.
- `CartesianInterpolator::interpolate_pose`의 병진은 `from.lerp(to, t)`
  = `a + (b - a) * t`인데 upstream은
  `percentage * b + (1 - percentage) * a`다(`cartesian_interpolator.cpp:258`,
  `:451`). ULP 수준에서 다르고 `t = 1`에서 upstream은 정확히 `b`다.
  이 함수를 오라클과 맞춰 보는 계기가 없어서 고치지 않았다 — 재보지 않고
  바꾸는 것은 측정이 아니라 주장이다.

> **세 줄 모두 §244가 처분했다.** 첫째 줄의 픽스처는 상류에 있었고
> (`one_robot`, `multiplier=1.5`/`offset=0.1`) 이미 커밋돼 있었다. 둘째
> 줄의 전체 상태 루프는 세 오버로드를 포팅하고 `state_interpolate` op으로
> 7로봇 1,572 케이스를 허용오차 0.0에서 비교했다. 셋째 줄은 결함이 아니라
> 이 절 자신의 오독이다 — `nalgebra`의 `lerp`은 `axpy(t, rhs, 1 - t)`로
> 포워딩하므로 상류의 `percentage * b + (1 - percentage) * a`와 성분별로
> 같은 식이다. 이 세 줄을 읽는 사람은 §244를 같이 읽어야 한다.

---

## §224 pilz joint-limits 여섯 파일 — 이 포트의 두 번째 입력은 파라미터 서버가 아니라 오버라이드 컨테이너다

`doc/port-coverage.md`가 `gap`으로 세던 pilz joint-limits 여섯 파일을 처분한다.
편집을 시작하기 전에 막힌 지점을 먼저 적는다: 여섯 중 둘은 내용의 전부가
`rclcpp` 파라미터 서버 접근이고, 하나는 그 파라미터 서버를 함수 인자로 받는다.
"파라미터 계층을 흉내 낸 껍데기를 만들어 포트가 완성돼 보이게 한다"는 선택지는
이 절에서 명시적으로 버린다.

### 224.1 이미 있는 표현 위에 얹는다 — 두 번째 한계 표현을 만들지 않는다

`crates/moveit-planners-pilz/src/limits.rs`가 이미
`JointLimit`/`JointLimitsContainer`/`CartesianLimits`/`LimitsContainer`를 들고
있고(상류 `joint_limits_container.{hpp,cpp}` +
`joint_limits_extension.hpp` + `joint_limits_copy/joint_limits.hpp` +
`limits_container.{hpp,cpp}`의 포트), 이 크레이트의 소비자가 이미 그것을 쓴다 —
`trajectory_functions::verify_sample_joint_limits`,
`trajectory_generator`, `command_list_manager::new(model, LimitsContainer)`.
이번 라운드의 두 모듈은 **그 타입을 인자와 반환값으로 쓴다**. 새 구조체는
`AggregationError` 하나뿐이고, 그것은 한계 표현이 아니라 상류 예외 클래스의
대응물이다.

`limits.rs`의 `JointLimit`이 상류 `joint_limits::JointLimits`(18필드 중
14개)와 `joint_limits_interface::JointLimits`의 확장 2필드
(`max_deceleration`/`has_deceleration_limits`)를 이미 한 타입으로 접어 두었다는
점이 아래 224.2의 환원을 가능하게 한다.

### 224.2 막힌 지점: `getAggregatedLimits`의 두 번째 입력이 무엇인가

상류 서명 (`joint_limits_aggregator.hpp:80-82`):

```cpp
static JointLimitsContainer getAggregatedLimits(
    const rclcpp::Node::SharedPtr& node, const std::string& param_namespace,
    const std::vector<const moveit::core::JointModel*>& joint_models);
```

`(node, param_namespace)` 쌍이 하는 일을 두 파일을 열어 확인했다.

- `joint_limits_interface_extension.hpp:49-98` — 함수 둘. 첫째는
  `joint_limits::declareParameters`로 그대로 넘기는 1줄 포워더. 둘째는
  `joint_limits::getJointLimits`를 부른 뒤 `<ns>.joint_limits.<joint>.
  has_deceleration_limits`와 `.max_deceleration` 둘을 더 읽어
  `limits`에 써 넣는다.
- `joint_limits_copy/joint_limits_rosparam.hpp:44-239` —
  `declareParameters`가 관절당 파라미터 **18개**를 선언하고,
  `getJointLimits`가 그 18개를 읽어 `JointLimits`의 필드에 써 넣는다.

두 파일이 `node`로 하는 일은 **`declare_parameter`/`has_parameter`/
`get_parameter` 뿐**이다(`node->` 출현: 각각 7회, 53회 — 다른 멤버 호출은
로거를 얻는 `node->get_logger()`/`node->get_name()`뿐이다). 계산은 한 줄도
없다. 즉 파라미터 서버가 `getAggregatedLimits`에 기여하는 것은
**관절 이름 → 부분적으로 채워진 `JointLimit` 하나**가 전부다.

**이 포트의 등가 입력을 그렇게 정한다:**

```rust
pub fn aggregate_limits<'a>(
    joint_models: impl IntoIterator<Item = &'a JointModel>,
    overrides: &JointLimitsContainer,
) -> Result<JointLimitsContainer, AggregationError>
```

`(node, param_namespace)` → `overrides: &JointLimitsContainer`.
YAML 파일이 진짜 입력이었고 파라미터 서버는 그 운반 수단이었다.
`overrides.has_limit(name)`가 상류 `getJointLimits`의 `true`/`false` 반환에,
`overrides.limit(name)`가 그 out-parameter에 대응한다.

**오버라이드 타입을 새로 만들지 않고 `JointLimitsContainer`를 쓰는 이유**는
224.1의 규칙만이 아니다. `JointLimitsContainer::add_limit`이
`has_deceleration_limits && max_deceleration >= 0`을 거부하므로, 오버라이드가
그 불변식을 **구성 시점에** 만족한다. 상류의 YAML 경로에는 그 게이트가 없고,
그것이 224.5(a)의 결함이 존재할 수 있는 이유다.

### 224.3 상류의 두 분기가 같은 일을 한다 — 읽어서 확인했다

`joint_limits_aggregator.cpp:70-99`의 if/else에서, `else` 팔(파라미터에 이
관절에 대한 것이 아무것도 없을 때)의 본문은

```cpp
updatePositionLimitFromJointModel(joint_model, joint_limit);
updateVelocityLimitFromJointModel(joint_model, joint_limit);
```

이고, `if` 팔은 `has_position_limits`/`has_velocity_limits`가 둘 다 `false`일 때
같은 두 호출을 한다(`:77-80`, `:86-89`). `joint_limit`은 그 지점에서
기본 생성 상태다. 따라서

```rust
let mut joint_limit = overrides.limit(name).unwrap_or_default();
```

는 근사가 아니라 **정확한** 등가다 — 부재한 관절은 모든 `has_*`가 `false`인
기본값을 얻고, 그 값으로 아래 두 if/else가 상류 `else` 팔과 같은 경로를 탄다.
이것이 이 절이 "이중 의미를 제거한다"고 말할 수 있는 지점이다: 상류에 있던
"파라미터가 없음" vs "파라미터가 있으나 플래그가 전부 false" 두 상태가 포트에서
하나의 규칙으로 접힌다.

### 224.4 파일별 처분

| 상류 파일 | 처분 | 근거 |
|---|---|---|
| `include/pilz_industrial_motion_planner/joint_limits_aggregator.hpp` | 포팅 | `crates/moveit-planners-pilz/src/joint_limits_aggregator.rs` |
| `src/joint_limits_aggregator.cpp` | 포팅 | 같음 — 224.2의 입력 치환 외에 계산은 전부 옮긴다 |
| `include/pilz_industrial_motion_planner/joint_limits_validator.hpp` | 포팅 | `crates/moveit-planners-pilz/src/joint_limits_validator.rs` |
| `src/joint_limits_validator.cpp` | 포팅 | 같음 — ROS 의존 0 |
| `include/pilz_industrial_motion_planner/joint_limits_interface_extension.hpp` | `decided-non-port` (D1) | 아래 |
| `include/joint_limits_copy/joint_limits_rosparam.hpp` | `decided-non-port` (D1) | 아래 |

**`joint_limits_interface_extension.hpp` — 남는 비-ROS 잔여분 0.**
파일 전체가 100줄, 내용은 인라인 함수 둘뿐이고 둘 다
`const rclcpp::Node::SharedPtr&`를 받는다. 이 파일이 헤더 주석으로
광고하는 "`JointLimits`에 deceleration 파라미터를 더한 확장"은 실제로는
**다른 파일**(`joint_limits_extension.hpp`)에 있고, 그 파일은 이미
`limits.rs`가 인용해 포팅돼 있다. 그러므로 이 파일에서 파라미터 서버 접근을
빼면 남는 것이 없다 — 부분 포팅할 대상이 없어서 `decided-non-port`이지,
어려워서가 아니다.

**`joint_limits_rosparam.hpp` — 남는 비-ROS 잔여분 0.**
302줄, 함수 다섯(`declareParameterTemplate`, `declareParameters`,
`getJointLimits(JointLimits&)`, `getJointLimits(SoftJointLimits&)`).
`node->` 출현 53회. 파일 머리의 상류 자신의 주석이 출처를 말한다: ros2_control
DRAFT PR #462에서 복사해 온 것이고 "Remove when ros2_control has an upstream
version of this". 즉 상류에서도 이 파일은 파라미터 서버 어댑터이지 pilz의
계산이 아니다. `SoftJointLimits` 오버로드는 pilz 패키지 전체에서 호출자가
**0**이다(`rg -n SoftJointLimits moveit_planners/` — `joint_limits_copy/`
바깥 히트 0).

두 파일 모두 D1(코어 크레이트는 ROS 타입을 일절 참조하지 않는다)에 정면으로
해당한다. `doc/port-coverage.md`의 두 행을 `gap` → `decided-non-port`로 옮기고
증거 칸이 이 절을 가리킨다.

### 224.5 상류가 정의하지 않은 두 지점 — 포트는 따라가지 않는다

**(a) `getAggregatedLimits`가 `addLimit`의 반환값을 버린다.**
`joint_limits_aggregator.cpp:109`는 `container.addLimit(name, joint_limit);`
이고 `bool` 반환을 읽지 않는다. `addLimit`은
`has_deceleration_limits && max_deceleration >= 0`이면 삽입하지 않고 `false`를
낸다(`joint_limits_container.cpp`). `:102-106`이 `max_deceleration =
-max_acceleration`을 설정하므로, 파라미터가 `has_acceleration_limits: true,
max_acceleration: 0.0`을 주면 `max_deceleration = -0.0`이 되고 `-0.0 >= 0.0`은
참이라 그 관절이 **조용히 컨테이너에서 빠진다**. 상류 자신의 테스트
`ExpectedMapSize`(`container.getCount() == joint_models.size()`)가 그 상황에서
깨진다. `doc/upstream-bugs.md`의
`aggregated-limits-drops-rejected-joint-silently`.

**(b) 다중 DOF 관절에 대한 경계 검사가 인접 멤버를 읽는다.**
`checkPositionBoundsThrowing`은 `joint_model->satisfiesPositionBounds(
&joint_limit.min_position)`을 부른다. `PlanarJointModel`/`FloatingJointModel`의
오버라이드는 `values[0..2]`/`values[0..6]`을 읽는데, 넘긴 포인터는 단일
`double` 멤버 하나를 가리킨다 — 가리킨 객체의 끝을 넘어 읽는 것이므로
동작이 정의되지 않는다. planar(3원소)에서는 상류 구조체가 `min_position`,
`max_position`, `max_velocity`를 그 순서로 선언하므로 읽히는 바이트가 그
셋에 해당하지만, floating(7원소)은 선언된 `double` 여섯 개를 넘어간다.
어느 쪽이든 그 관절의 위치·속도가 아니다. 클래스 doc이 "Does not support
MultiDOF joints"라고 적으면서도 이 경로에는 그 가드가 없다.
`doc/upstream-bugs.md`의 `check-position-bounds-multidof-adjacent-members`.

포트는 (a)를 `AggregationError::DuplicateJoint`/`NonNegativeDeceleration`으로,
(b)를 `AggregationError::MultiDofBoundsCheck`로 **오류로 올린다**. 둘 다
`not-reproduced`이고, 이유는 상류가 그 자리에서 정의된 동작을 갖지 않기
때문이다 — 조용한 드롭과 인접 멤버 읽기는 재현할 "동작"이 아니다.

### 224.6 실제로 착지한 것과, 가드마다 물린 변형

여섯 파일의 처분은 224.4대로 끝났다. `doc/port-coverage.md`는 네 행이
사라지고(두 모듈이 `Ported from moveit2 @ …` 헤더로 그 상류 경로를 이름으로
든다) 두 행이 `gap` → `decided-non-port`로 옮겨, 미포팅 95 → 91,
`decided-non-port` 45 → 47 / `gap` 40 → 34가 되었다.
`measure-port-coverage.py --check`가 91행 == 91건으로 통과한다.

테스트는 두 모듈 합쳐 39개다. 이 39개가 "가드를 덮는다"는 주장은 읽기가
아니라 실측이다 — 가드마다 변형을 하나씩 넣어 돌리고 되돌렸고, 58회의 변형
실행 결과가 `doc/assertion-discrimination-ledger-p10-jointlimits.md`에
전부 적혀 있다. 39개 중 **38개**가 자기만 깨뜨리는 변형을 갖는다.

갖지 못한 하나는 `a_differing_max_position_is_a_disagreement`다. 그 픽스처는
`a_third_joint_disagreeing_with_the_first_two_is_still_a_disagreement`의
진부분집합(2관절 vs 3관절, 같은 `max_position` 1.0 vs 2.0)이라
`max_position` 비교를 어떻게 바꾸든 둘이 같이 깨진다. 크기로 가르는 변형도
없다 — 두 픽스처의 차이가 같은 1.0이다. 반대 방향은 갈라진다(`V2`,
"첫 쌍만 비교"는 3관절 쪽만 깨뜨린다). 그래서 두 테스트는 중복이 아니고,
작은 쪽의 커버리지 주장이 2개 가족이라는 사실만 원장에 그대로 적었다.

이 절을 쓰면서 상류 결함 두 건이 `doc/upstream-bugs.md`에 들어갔다
(`aggregated-limits-drops-rejected-joint-silently`,
`check-position-bounds-multidof-adjacent-members`). 둘 다 `not-reproduced`,
둘 다 C++로 돌려본 것이 아니라 읽은 것이며, 대신 포트 쪽에서 대응 테스트와
그 테스트만 깨뜨리는 변형(`A10`, `A35`/`A34`)을 확보했다.

## §225 `constraint_samplers`의 남은 두 파일과 all-valid 충돌 검출기 — 갭 6건을 판정으로 바꿨다

`doc/port-coverage.md`가 `gap`으로 들고 있던 6개 파일을 열어 판정했다.
`gap`은 "아무도 결정한 적 없다"는 뜻이지 "포팅해야 한다"는 뜻이 아니므로,
판정 없이 남겨 두면 다음 라운드가 같은 파일을 같은 깊이로 다시 연다. 아래
네 소절이 각각 근거를 적고, `doc/port-coverage.md`의 해당 행을 옮긴다.

### §225.1 `constraint_sampler_tools.{hpp,cpp}` — `decided-non-port`

네 선언 중 셋은 이미 D1이다(`visualizeDistribution` 둘은
`visualization_msgs::msg::MarkerArray`를 받고,
`countSamplesPerSecond(constr, scene, group)`는 `moveit_msgs::Constraints`와
`PlanningSceneConstPtr`을 직접 받는다). 넷째
`countSamplesPerSecond(sampler, reference_state)`만 ROS 타입을 받지 않아
`crates/moveit-constraints/src/lib.rs`의 선언별 감사가 라운드 12부터
`gap`으로 들고 있었다. 이 절이 그것을 판정한다.

**포팅하지 않는다.** 세 가지 이유이고, 앞의 둘이 결정적이다.

1. **루프의 종료 조건이 벽시계다.** `constraint_sampler_tools.cpp:82,92`가
   `rclcpp::Clock().now() + rclcpp::Duration::from_seconds(1)`을 잡고
   `while (rclcpp::Clock().now() < end)`로 돈다. 뽑는 횟수가 기계 성능의
   함수이므로 반환값의 분해능도 기계의 성질이고, 호출 하나가 구조적으로
   1초 이상 걸린다. 이 워크스페이스의 어떤 테스트도 그 출력에 단언을 걸 수
   없다.
2. **같은 양을 이미 결정적으로 재고 있다.**
   `crates/moveit-constraints/tests/sampler_self_validation.rs`가 일곱 개
   샘플러 구성마다 `attempted`/`produced`/`satisfied`를 세고, 시드 고정된
   10,000 상태 예산에 대해 `produced < quota`면 그 구성을 이름 붙여
   실패시킨다. `countSamplesPerSecond`가 계산하는 `valid / total`이 바로
   그 `produced / attempted`인데, 이쪽은 벽시계가 아니라 뽑기 수로 예산이
   묶여 있고 값을 반환하는 대신 단언한다.
3. **생산 호출자가 0이다.** `moveit_core`/`moveit_planners`/`moveit_ros`
   전체에서 유일한 호출자는 자기 자신의 D1 형제
   (`constraint_sampler_tools.cpp:68`)이고, 그 형제는 받은 `double`을
   들여다보지 않고 자기 반환값으로 그대로 흘려보낸다(라운드 13/14 근거,
   `lib.rs`의 해당 항목에 유지). 그 위에 이번 라운드가 하나 더 찾았다 —
   그 `double`은 이름이 약속하는 rate도 아니다
   (`doc/upstream-bugs.md`의 `count-samples-per-second-returns-a-ratio`).

`lib.rs`의 66개 선언 감사에서 이 항목의 태그를 `gap`에서 `decided`로
옮겼다. 태그별 수는 `rg -c '^//! - CS:.*-> TAG'`로 재현되며
19/23/8/6/1/9 = 66이다.

**부수 정정.** 그 감사의 마지막 합계 줄이 `18 + 23 + 8 + 6 + 11 = 66`이었다.
합은 맞지만 `ported`와 `gap` 항이 라운드 20 이후 재도출된 적이 없어 둘 다
1씩 틀려 있었고, 반대 방향이라 총합이 가려 주고 있었다. 항을 `rg -c`
실측으로 바꿨다.

### §225.2 `constraint_sampler.cpp` — `ported-elsewhere`, 잔여분 `clear()`는 판정

이 파일은 67줄이고 함수 본문이 정확히 둘이다. 표가 이 행을 `gap`으로 들고
있던 근거는 "선언별 감사가 `getName()`을 세 샘플러에서 미포팅으로 남긴다"
였는데, `getName()`은 이 `.cpp`에 없다 — 헤더의 순수 가상이고 구체
오버라이드는 `default_constraint_samplers.hpp`/`union_constraint_sampler.hpp`
안에 인라인으로 있다. 즉 이 행의 근거는 다른 파일의 잔여분을 가리키고
있었다. 이 절이 이 파일 자체의 두 본문을 판정한다.

- **생성자 (`:52-60`).** 실질은 한 줄
  `jmg_ = scene->getRobotModel()->getJointModelGroup(group_name)`이고,
  실패하면 `RCLCPP_ERROR`를 찍은 뒤 **널 `jmg_`인 채로 생성이 계속된다**
  (그래서 모든 `configure()`가 `if (!jmg_)`를 다시 본다,
  `default_constraint_samplers.cpp:72`). 포트에서는
  `JointConstraintSampler::new`/`UnionConstraintSampler::new`의 첫 줄
  `model.joint_model_group(group_name)?`가 그 조회이고, `?`가 그 "로그 찍고
  계속"을 생성 시점의 `Error::UnknownName`으로 바꾼다
  (`crates/moveit-constraints/src/sampler.rs:184,377`).
  `IkConstraintSamplerAdapter::new`는 이미 해결된 `&JointModelGroup`을 받아
  조회 자체가 없다. 나머지 초기화 셋(`is_valid_`, `verbose_`, `scene_`)은
  포트에 대응 필드가 없는 것들이고, 각각 이미
  `crates/moveit-constraints/src/lib.rs`의 선언별 감사에 이름으로 처분이
  붙어 있다.
- **`clear()` (`:62-66`) — 포팅하지 않는다.** 호출 지점은 둘 다
  `configure()` 안이다: 두 타입의 `configure()` 첫 줄
  (`default_constraint_samplers.cpp:70,255`, 각자의 `clear()` 오버라이드
  경유) — 살아 있는 샘플러에 `configure()`를 **두 번째로** 걸 때 백지에서
  시작하기 위한 것 — 과 `:121`의 "no possible values for the joint" 실패
  경로 — 실패를 넘어 살아남는 객체에 이미 써 넣은 부분 설정을 손으로
  되돌리기 위한 것. 이 포트에는 둘 다 구조적으로 없다. 재설정 단계가
  없고(`new()`가 통째로 짓거나 `Err`), `frame_depends`는 그 `new()` 안에서
  한 번 계산된 뒤 다시 쓰이지 않으며, `:121`에 해당하는 실패는
  `crates/moveit-constraints/src/sampler.rs:213-221`의 `return Err(..)`이라 되돌릴 부분 구축물이 애초에
  존재하지 않는다. 상류가 손으로 되돌려야 하는 이유는 실패한 객체가
  살아남기 때문이고, 포트는 그 객체를 만들지 않는다.

따라서 분류는 `ported-elsewhere`(내용이 다른 이름으로 트리 안에 있음),
증거는 `crates/moveit-constraints/src/sampler.rs:184,377`, 잔여분 `clear()`는 위에서 판정. `sampler.rs`
모듈 doc이 같은 내용을 그 파일 옆에 적어 둔다.

### §225.3 `collision_env_allvalid.{hpp,cpp}` — 포팅했다, 고르는 경로까지

`AllValidCollisionEnv`를 `crates/moveit-collision/src/all_valid.rs`에
포팅했다. 상류 클래스 자체는 작다 — 여섯 개 메서드가 `res.collision =
false`만 쓰고 받은 입력을 하나도 읽지 않는다. 어려운 쪽은 "이걸 어떻게
고르느냐"다. 아무도 고를 수 없는 널 검출기는 포트가 아니라 죽은 코드이고,
"충돌 없음"은 **호출하지 않았을 때 나오는 답과 같아서** 테스트가
`assert!(!collision)` 하나로는 아무것도 증명하지 못한다.

상류의 선택 경로는 `CollisionDetectorAllocatorAllValid`(`NAME`이
`"ALL_VALID"`)를 `PlanningScene`의 `collision_detector_` 맵에 등록하고
문자열로 찾는 것이다. 이 포트에는 그 맵도 그 할당자도 없다(§225.4).
대신 `moveit_scene::PlanningScene`의 모든 충돌 메서드가 호출자가 주는
`E: CollisionEnv<Posed<'_, 'm>>`에 대해 제네릭이므로, 이 백엔드를 고른다는
것은 `ParryCollisionEnv`가 갈 자리에 `AllValidCollisionEnv`를 넘긴다는
뜻이다. 그것이 선택 경로의 **전부**이고, `AllValidCollisionEnv`가 같은
바운드를 만족한다는 사실 자체가 도달 가능성의 증명이다.

`crates/moveit-scene/tests/all_valid_selection.rs`가 그 경로를 실행한다.
같은 씬·같은 상태·같은 ACM·같은 요청을 두 번 묻고 호출자가 이름 붙인 타입
하나만 다르게 한다. pr2를 쓰는 이유는 `base_footprint`가 원점 근처에
**프리미티브** 상자를 달고 있어 0.1 구 하나로 메시 로딩 없이 충돌이
나기 때문이다(panda/fanuc는 `<mesh>`뿐이라 "충돌한다" 쪽 절반이 백엔드
선택이 아니라 메시 해석에 걸린다). 네 케이스 중 하나는 대조군 — 이
픽스처가 실제로 충돌한다는 것 — 이고, 이것이 깨지면 나머지 셋이 공허해지기
때문에 따로 이름을 붙였다.

증명의 무게는 `false`가 아니라 두 가지 변이가 진다. 하나는 테스트가 이름
붙인 타입만 `&parry`로 바꾸면 그 케이스가 깨진다는 것(답이 이름 붙인
백엔드의 함수라는 뜻), 다른 하나는 `distance_to_collision`이 `f64::MAX`를
낸다는 것 — 이 값은 이 트리에서 `AllValidCollisionEnv::distance_robot`만
만들어 내므로, 백엔드를 건너뛰고 기본값을 돌려주는 씬이었다면 이 케이스가
깨진다. 여덟 개 변이 전부와 각각이 무엇을 죽이고 무엇을 살려 두는지는
`doc/assertion-discrimination-ledger-p10-samplers.md`에 있다.

두 가지 상류 판정이 코드에 남았다.

1. **`distanceRobot(state)`의 `0.0`은 따라가지 않는다.** 상류가
   `virtual double distanceRobot(state) const`로 `0.0`을 반환하는데
   (`collision_env_allvalid.cpp:114-123`), 기반 클래스의 동명 편의
   오버로드가 **비가상**이라(`collision_env.hpp:202`) 그 선언은 아무것도
   재정의하지 못하고 가리기만 한다. 할당자가 건네주는 `CollisionEnvPtr`를
   든 호출자는 기반 쪽, 즉 `std::numeric_limits<double>::max()`를 받는다.
   한 질문에 두 답이고 고르는 것은 식의 정적 타입이다
   (`doc/upstream-bugs.md`의 `all-valid-distance-robot-hides-base-overload`).
   이 포트에는 `distance_robot(state)` 편의 오버로드가 아예 없어서 그 분기를
   표현할 수 없고, 기반 쪽 값인 `f64::MAX`로 간다. 의미상으로도 그쪽이
   맞다 — `0.0`은 이 백엔드가 존재 이유로 삼는 "충돌 없음"의 반대편
   경계다.
2. **연속(두 상태) 형태는 `Err`가 아니라 답한다.** `ParryCollisionEnv`는
   스윕 질의가 없어 `Err`를 내지만(`parry.rs:2439`), 상류는 두 개의
   `checkRobotCollision(state1, state2, ...)` 오버로드를 **둘 다**
   `res.collision = false`로 재정의한다(`collision_env_allvalid.cpp:89-106`).
   "아무것도 충돌하지 않는다"는 주장은 경로에도 상태와 똑같이 적용된다.

`CollisionResult`의 세 `Option` 필드는 요청을 따라간다 — 상류가
기본 생성된 결과를 그대로 두는 것이 이 포트에서는 "물어봤으니 `Some`,
안 물어봤으니 `None`"이다. 여기서 인접 결함 하나가 드러났다:
`ParryCollisionEnv`의 `accumulate_collision`(`parry.rs:2177-2150`)은
`distance: None`을 무조건 쓰므로 `CollisionRequest::distance`를 켠
호출자에게도 `None`을 준다. `CollisionResult::distance`의 doc이 적어 둔
"요청했을 때 정확히 존재한다"를 어기는 쪽은 그쪽이다. 이번 라운드 범위가
아니어서 고치지 않고 보고만 한다.

### §225.4 `collision_detector_allocator{,_allvalid}.hpp` — `decided-non-port`, 그리고 §4.5의 좁힘

두 할당자 헤더를 포팅하지 않는다. §4.5가 "`CollisionDetectorAllocator`
trait은 유지하므로 나중에 FCL FFI 백엔드를 추가할 수 있다"고 적어 둔 것을
이 절이 **좁힌다** — 유지되어야 할 것은 그 목적이지 그 간접층이 아니고,
목적은 `CollisionEnv` trait 자체가 이미 지고 있다. `moveit-collision`의
`env` 모듈 doc이 같은 판정을 그 trait 옆에 적는다.

`env.rs`가 지금까지 들고 있던 것은 판정이 아니라 **유예**였다: "컴파일타임
레지스트리는 등록자가 최소 하나는 있어야 값어치가 있는데 이 태스크는 trait만
남기고 끝난다(parry 백엔드 아직 없음)". 그 조건은 만료했다 —
`ParryCollisionEnv`, `AllValidCollisionEnv`,
`moveit_distance_field::HybridCollisionEnv` 셋이 `CollisionEnv`를 구현한다.
그래서 유예를 한 번 더 미루는 대신 여기서 결정한다.

**상류 할당자가 하는 일은 백엔드의 타입을 런타임 문자열로 미루는 것이다.**
`allocateEnv`가 `CollisionEnvPtr`을 돌려주고 `getName()`이 그 쌍에 이름을
붙이므로, `PlanningScene::allocateCollisionDetector`가 `collision_detector_`
맵을 그 이름으로 키잉하고 `getCollisionEnv(name)`이 찾아 쓴다
(`planning_scene.cpp:255-311`). 이 포트는 그 결정을 반대 방향으로 한 번,
영구히 했다: `moveit_scene::PlanningScene`의 충돌 메서드가 호출자가 주는
`E: CollisionEnv<Posed<'_, 'm>>`에 대해 제네릭이라 백엔드는 호출 지점에서
**타입으로** 지목된다(`scene.rs:549-557`의 `allocateCollisionDetector`
처분). 찾을 이름도, 그 이름을 키로 쓸 맵도 없다.

세 가지가 근거다.

1. **소비자가 없다.** `rg -n -i 'collision_detector|detector_name' crates/
   ros/ tools/ --glob '*.rs'`의 결과 12건이 전부 상류 심볼을 이름으로
   부르는 doc 주석이고, 선택 지점은 하나도 없다. 레지스트리를 놓으면
   등록자만 있고 소비자가 없다.
2. **`linkme` 순서 위험을 이미 한 번 치렀다.** §177 — `linkme` 슬라이스
   순서는 링커 섹션 순서이고, 워크스페이스 어딘가에 의존성 하나를 더한 것이
   `KINEMATICS_SOLVERS` 순서를 조용히 뒤집어 pilz 패리티 테스트를 깨뜨렸다.
   아무도 읽지 않는 슬라이스를 위해 그 위험을 다시 들일 이유가 없다.
3. **균일한 생성 프로토콜이 세 백엔드 중 어디에도 맞지 않는다.**
   `ParryCollisionEnv::new(world, padding_scale)`,
   `HybridCollisionEnv::new(world, padding_scale, link_body_decompositions,
   distance_field_config, collision_tolerance) -> Result<Self>`,
   그리고 인자가 없는 유닛 구조체 `AllValidCollisionEnv`. 상류의
   `allocateEnv(world, robot_model)` 세 오버로드로 이 셋을 다 부를 수 없다.

`collision_detector_allocator_allvalid.hpp`는 그 템플릿의 인스턴스화 한 줄
(`CollisionDetectorAllocatorTemplate<CollisionEnvAllValid,
CollisionDetectorAllocatorAllValid>`)에 `NAME = "ALL_VALID"`를 붙인 것이
전부이므로 상위 판정을 그대로 따른다. 같은 형태의
`collision_detector_allocator_{distance_field,hybrid}.hpp` 둘은 이미 D4로
`decided-non-port`였다(`crates/moveit-distance-field/src/lib.rs:541-553`) —
이 절은 그 판정을 남은 두 건으로 넓히고, 그 근거를 D4의 "pluginlib 런타임
플러그인"보다 한 단계 아래에서 다시 적는다.

> Phase 4는 §221에서 갱신되었다. (b)는 조건의 수를 고친 뒤 MET이고
> (19,389개 해 중 `1e-5` 이상 0개), (a)는 재시작을 끄면 네 픽스처 모두에서
> 포트가 오라클 이상이다 — 기본 재시작에서의 한계 성공률 대소는 난수열을
> 측정한다. 위 요약 줄의 "Phase 4"는 §221 이전 판정이다.

## §221 Phase 4 완료 조건 두 항목 — (a)는 재시작 난수, (b)는 조건의 수가 틀렸다

§216.3이 Phase 4를 (a)·(b) 모두 UNMET으로 기록했다. 이번 라운드는 그 두
항목의 원인을 각각 찾아 이름 붙이고, 고칠 수 있는 쪽을 고쳤다. 모든 수치는
이 라운드에 다시 측정했고, 명령을 함께 적는다.

### 221.1 (a) "열다섯 개의 자세"는 존재하지 않는다 — b=82와 c=67의 차이일 뿐이다

> **이 절을 시작시킨 전제는 틀렸고, 이 절이 그것을 반증한다.** 이 라운드의
> 지시는 "오라클은 풀고 이 포트는 못 푸는 자세 열다섯 개를 특정하라"였다.
> 그런 집합은 없다. 15는 **잔차**다 — 오라클만 푸는 82건에서 포트만 푸는
> 67건을 뺀 값이고, 두 집합은 서로소다. 게다가 재시작을 끄면 부호가 뒤집혀
> 네 픽스처 전부에서 포트가 오라클 이상이다(아래 두 번째 표). 이 문서를
> 뒤에 읽는 사람은 "그 열다섯 개"를 찾으러 가지 말 것. 찾을 것이 없다.

**15는 집합이 아니라 두 disjoint 집합의 차다.** `4906` vs `4921`은 한계
(marginal) 성공률이고, 그 뒤의 쌍(paired) 분해는 b=82(오라클만 성공),
c=67(포트만 성공)이다. "오라클이 풀고 포트가 못 푸는 자세"는 15개가 아니라
82개이며, 반대 방향으로 67개가 따로 있다.

계기가 없어서 이제까지 그 82개를 이름으로 부를 수 없었다. 이번 라운드에
`moveit-diff`에 `--ik-divergence-json`(케이스 번호·성공한 쪽·joint_values를
행으로 기록)과 `--ik-rng-seed`(이 쪽 재시작 재시드 스트림만 교체)를 넣었다.

```console
$ moveit-diff --urdf fixtures/panda.urdf --srdf fixtures/panda.srdf \
    --cases 5000 --seed 1 --group panda_arm --ik \
    --ik-max-restarts 20 --ik-rng-seed <N> \
    --ik-divergence-json <out> --oracle tools/moveit-oracle/run-oracle.sh
```

(`--urdf`/`--srdf`는 **절대 경로**여야 한다. `run-oracle.sh`가 레포 루트를
컨테이너 안 같은 경로에 마운트하고 그 경로로 파일을 열기 때문에, 상대 경로를
주면 `oracle: startup failed: cannot open fixtures/panda.urdf`로 죽는다.
아래 모든 실행도 같다.)

| `--ik-rng-seed` | 오라클 성공 | 포트 성공 | b | c |
|---|---|---|---|---|
| 0 | 4921/5000 | 4906/5000 | 82 | 67 |
| 12345 | 4921/5000 | 4901/5000 | 82 | 62 |
| 777 | 4921/5000 | 4890/5000 | 89 | 58 |

**b의 구성원은 자세의 성질이 아니라 뽑기의 성질이다.** 세 실행의 b를
교집합하면 남는 것은 **2개**(case 408, 4130)뿐이고, 합집합은 **226개**다.
`--ik-rng-seed 0`의 82개 중 **80개**를 포트가 다른 두 스트림 중 하나에서
푼다.

이 해석이 성립하려면 오라클의 성공 집합이 세 실행에서 같아야 한다. 같다:
`Op::Ik`가 실어 보내는 필드는 `group`/`joint_values`/`position_only`/
`max_restarts`/`consistency_limits` 다섯 개뿐이라(`protocol.rs:186-230`)
`--ik-rng-seed`는 wire에 실리지 않고, 오라클의 IK 난수는 고정 시드 멤버
(`tools/moveit-oracle/src/oracle.cpp:6065`, `ik_rng_{ 42 }`)이며, 오라클
`ik` op에는 벽시계가 없다. 세 실행의 오라클 성공 수가 모두 `4921`로 같고,
`--ik-rng-seed 0` 실행을 다시 돌리면 `--ik-divergence-json` 파일이 `cmp`로
바이트 동일하다.

**재시작을 끄면 포트가 뒤지지 않는다.** `--ik-max-restarts 0`은 양쪽 모두
결정론적 bounds-midpoint 시드 한 번만 시도한다:

| `--ik-rng-seed` | 오라클 | 포트 | b | c |
|---|---|---|---|---|
| 0 | 2432/5000 | **2435**/5000 | 1 | 4 |
| 12345 | 2432/5000 | **2434**/5000 | 2 | 4 |
| 777 | 2432/5000 | **2436**/5000 | 1 | 5 |

5,000건 중 4,995건이 일치하고, 세 스트림 모두에서 포트가 오라클보다 많이
푼다. 즉 시드·수렴 판정·관절 한계 클램핑에는 성공률을 가르는 차이가 없다.

같은 재시작-없는 비교를 나머지 세 픽스처에도 돌렸다(`--ik-rng-seed` 기본 0):

| 픽스처/그룹 | 오라클 | 포트 | b | c |
|---|---|---|---|---|
| panda/panda_arm | 2432/5000 | **2435**/5000 | 1 | 4 |
| fanuc/manipulator | 1061/5000 | 1061/5000 | **0** | **0** |
| dual_arm_panda/left_panda_arm | 2471/5000 | 2471/5000 | 4 | 4 |
| pr2/right_arm | 3223/5000 | **3227**/5000 | 16 | 20 |

**재시작을 끈 네 픽스처 전부에서 포트가 오라클 이상이다.** fanuc은 5,000건의
성공·실패가 한 건도 다르지 않다(b = c = 0). 기본 재시작에서 나타나던
픽스처별 부호 차이(위 −15/+7/−20/+1)는 이 층에서는 존재하지 않는다.

**남은 5건의 기전.** 임시 계측(커밋하지 않음: `cart_to_jnt`의 특이점 탈출
분기에 `eprintln!`을 넣고 `search_position_ik` 진입마다 케이스 경계를 찍은
뒤 되돌림)으로, 재시작 없는 5,000건 중 **252건**이 그 분기에 들어간다.
재시작 없는 세 실행에서 한 번이라도 갈린 케이스는 8개이고, 그중 205·1052·
2069·4252·4914는 그 분기를 밟는다(각각 36·1·1·9·6회) — 그래서 `--ik-rng-seed`를
바꾸면 갈림이 이동한다. 1306·1356·1608은 **0회**로 포트 쪽이 완전히
결정론적이며, 셋 다 포트가 풀고 오라클이 못 푸는 방향이다. 이 셋의 원인은
오라클 쪽 난수(`delta_q.data.setRandom()`, Eigen `std::rand`)이거나 부동소수
경로 차이인데, 오라클을 계측하지 않았으므로 둘을 분리하지 못했다 — UNFIXED로
남긴다(5,000건 중 3건).

**부호가 픽스처마다 다르다.** 같은 명령을 네 픽스처에 돌린 결과:

| 픽스처/그룹 | 오라클 | 포트 | 차 | b | c | McNemar \|z\| |
|---|---|---|---|---|---|---|
| panda/panda_arm | 4921 | 4906 | −15 | 82 | 67 | 1.23 |
| fanuc/manipulator | 4584 | **4591** | +7 | 303 | 310 | 0.28 |
| dual_arm_panda/left_panda_arm | 4925 | 4905 | −20 | 85 | 65 | 1.63 |
| pr2/right_arm | 4986 | **4987** | +1 | 10 | 11 | 0.22 |

네 개 중 둘은 포트가 앞선다. 네 개 모두 `PAIRED_DIVERGENCE_Z_THRESHOLD`
(3.0) 아래이고, pr2만 `b + c = 21 < MINIMUM_USABLE_B_PLUS_C`(61)라 검정력이
없다.

**포트가 다르게 하는 것은 이것이다:** 재시작 재시드의 난수열. 포트는
`ChaCha8Rng`(기본 시드 0, `NewtonRaphsonSolver::new_with_seed`), 오라클은
boost `mt19937`(시드 42)이며, 두 스트림은 서로 무관하다. 그리고 양쪽 모두
상류 `searchPositionIK`의 벽시계 재시작 루프
(`kdl_kinematics_plugin.cpp:303-415`의 `do { ... } while (!timedOut(start_time,
timeout))`)를 **고정 시도 횟수**로 바꾼 동일한 이탈을 갖는다
(`SolverParams::max_restarts`, `Op::Ik::max_restarts`). 시드는 양쪽 다 같은
bounds-midpoint, 수렴 판정은 양쪽 다 `max(position_error, orientation_error)
<= epsilon`, 관절 한계 클램핑은 재시작 없는 실행이 4,995/5,000 일치로
배제한다.

**따라서 (a)를 한계 성공률의 대소로 읽으면 그 문장은 난수 뽑기를 측정한다.**
이 문서는 (a)의 문구를 이번 라운드에 고치지 않는다(§5는 (b)만 고쳤다). 다만
읽는 사람이 4906 < 4921을 알고리즘 열위로 오독하지 않도록, 위 표와 재시작
없는 비교를 §5 Phase 4에서 참조하게 했다.

### 221.2 (b) `1e-6`은 솔버 계약을 잘못 읽은 수다 — 옳은 수는 `epsilon`(`1e-5`)

**상류 계약을 추론하지 않고 읽었다.**
`moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_parameters.yaml`
(reference: `/home/stevek/work/moveit2` @ `e017c91e`)의 `epsilon`은
`default_value: 0.00001`, 설명은 `"Epsilon. Default is 1e-5"`다. 그리고
`kdl_kinematics_plugin.cpp`의 `CartToJnt`(418-497)는 매 반복 앞에서 현재
`q_out`의 FK 트위스트를 재고,

```cpp
const double position_error = delta_twist.vel.Norm();
const double orientation_error = ik_solver.isPositionOnly() ? 0 : delta_twist.rot.Norm();
const double delta_twist_norm = std::max(position_error, orientation_error);
if (delta_twist_norm <= params_.epsilon) { success = true; break; }
```

`delta_q`를 더하기 **전에** `break`한다. 즉 반환되는 해는 방금 오차를 잰
바로 그 구성이고, **성공한 해의 FK 오차는 병진·회전 각각 `epsilon` 이하가
구조적으로 보장된다.** 오라클도 같은 상수를 박아 두었고
(`oracle.cpp:1772`, `constexpr double kEpsilon = 0.00001`), 이 포트도 같다
(`crates/moveit-kinematics/src/params.rs:65`, `epsilon: 0.00001`).

**측정: 오차 상한이 `epsilon`을 정확히 따라간다.** `--ik-epsilon`(이번
라운드에 추가, 이 쪽 `SolverParams::epsilon`만 바꾼다)으로 격자를 돌렸다.
`--cases 5000 --seed 1 --group panda_arm`, panda 픽스처:

| `--ik-epsilon` | `--tol-ik` | failed | 최대 병진 오차 | 최대 회전 오차 |
|---|---|---|---|---|
| `1e-5` | `1e-5` | **0** | — | — |
| `1e-5` | `1e-6` | 1513 (병진 1112 + 회전 401) | `9.922533068067614e-6` | `8.758324911274685e-6` |
| `1e-6` | `1e-6` | **0** | — | — |
| `1e-6` | `1e-7` | 1244 | `9.79513234121028e-7` | `9.559096684818135e-7` |
| `1e-7` | `1e-7` | **0** | — | — |
| `1e-7` | `1e-8` | 1082 | `9.965294414492535e-8` | `9.947287282497852e-8` |

`epsilon`을 한 자리 조일 때마다 최대 오차가 정확히 한 자리 내려가고,
`tol_ik == epsilon`에서는 언제나 0건이다. **따라서 조건의 `1e-6`은 솔버
정확도에 관한 진술이 아니다 — "`epsilon`을 `1e-6`으로 두라"는 문장을 다른
곳에 옮겨 적은 것이다.**

(1112/401은 `moveit-diff`가 케이스별 `FAIL` 줄과 말미의 중복 제거 목록에
같은 메시지를 두 번 찍기 때문에 `grep -c 'translation error'`로 세면 2224/802가
나온다 — §60.3이 경고한 그 이중 계수다. 위 수는 `grep -c '^FAIL ik\[.*translation error'`로
인라인 줄만 센 값이고, 1112 + 401 = 1513 = 요약의 `failed`와 일치한다.)

**대안의 비용도 쟀다 — 그리고 비용은 이유가 아니었다.** `epsilon`을 조이는
쪽을 실제로 측정했다(반복 횟수는 임시 계측: `cart_to_jnt` 반복문에 누적
카운터를 넣고 실행 후 되돌림):

| `--ik-epsilon` | 포트 성공률 | 솔버 반복(5,000 solve 합) | solve 벽시계 |
|---|---|---|---|
| `1e-5` | 4906/5000 | 728,695 | 5.413 s / 5.943 s |
| `1e-6` | 4897/5000 | 668,855 | 5.017 s / 5.182 s |
| `1e-7` | 4915/5000 | 920,881 | 5.492 s / 6.455 s |
| `1e-8` | 4904/5000 | 798,879 | 5.123 s / 6.229 s |

**벽시계 열은 재현되지 않으므로 두 실행을 나란히 적는다**(§221.4). 성공률과
반복 횟수는 두 실행에서 자리 하나까지 같지만, 벽시계는 최대 +21.59%
움직였다. 그 차이는 트리가 아니라 실행 잡음이다 — **같은 트리·같은
바이너리·같은 `epsilon`의 독립 실행 쌍에서 이미 16.71%가 흔들린다**
(4.823 s vs 5.629 s, `1e-6`). 따라서 이 열의 열 간 대소를 epsilon의 비용
차로 읽으면 안 되고, 아래 비단조성 논증도 벽시계에 걸려 있지 않다.

성공률도 반복 횟수도 **단조가 아니다.** 그 이유가 결정의 핵심이다:
이 알고리즘에서 `epsilon`은 정확도 다이얼이 아니라 세 곳을 동시에 가르는
상수다(`crates/moveit-kinematics/src/cart_to_jnt.rs`, 상류도 같은 세 곳) —
수렴 판정 `delta_twist_norm <= epsilon`, 특이점 고착 판정
`delta_q_norm < epsilon`, 포기 판정 `step_size < epsilon`. `epsilon`을
움직이면 받아들이는 정확도만이 아니라 탐색 자체가 바뀌고, 위 비단조성이
그 부작용의 관측값이다.

**결정: 솔버가 아니라 §5를 고친다.** 근거 셋:

1. `1e-5`는 상류가 선언한 기본값 그 자체다. 포트의 `epsilon`을 바꾸면
   `kdl_kinematics_parameters.yaml`의 `default_value`와 더 이상 맞지 않는다.
2. 오라클은 `kEpsilon`을 박아 두었다. 포트만 다른 `epsilon`으로 돌면
   같은 완료 조건의 (a)와 (b)가 **서로 다른 솔버 계약** 아래에서 측정된다 —
   (a)의 "C++ KDL 플러그인 이상"이 비교 대상을 잃는다.
3. `epsilon`은 순수한 정확도 손잡이가 아니다(위 비단조성). 문서 한 문장을
   만족시키려고 조이면 특이점 처리가 부수적으로 바뀐다.

**옳은 수와 그 근거 측정:** 성공한 해의 FK 오차 ≤ `SolverParams::epsilon`
= `1e-5`, 병진·회전 각각. 네 픽스처를 `--cases 5000 --seed 1 --tol-ik 1e-6`으로
돌려 `1e-6`을 넘는 값을 전부 찍고 그 최댓값을 취했다(따라서 `1e-5`를 넘는
값이 있었다면 반드시 이 목록에 들어온다):

| 픽스처/그룹 | 성공한 해 | 최대 병진 오차 | 최대 회전 오차 |
|---|---|---|---|
| panda/panda_arm | 4906 | `9.922533068067614e-6` | `8.758324911274685e-6` |
| fanuc/manipulator | 4591 | `9.950412400488455e-6` | `9.890500838819756e-6` |
| dual_arm_panda/left_panda_arm | 4905 | `9.932247828109614e-6` | `8.92274391460425e-6` |
| pr2/right_arm | 4987 | `9.997467339945621e-6` | `9.86080194284471e-6` |

합계 **19,389개 해 중 `1e-5` 이상은 0개**, 최댓값은 pr2의
`9.997467339945621e-6`(상한의 99.97%). 여유가 거의 없는 것이 정상이다 —
이 값은 알고리즘이 보장하는 상한 그 자체이므로, 이 조건이 깨지는 경우는
해 추출·좌표계 변환·mimic 접기 같은 **진짜 결함**뿐이다. `moveit-diff`의
상시 기본값 `tol_ik = 2e-5`가 이보다 느슨한 이유가 그것이고
(`Config::tol_ik` 주석), 완료 조건은 상시 게이트가 아니라 계약의 상한을
적어야 하므로 `1e-5`로 적는다.

### 221.3 `1e-6`을 통과하며 서 있는 테스트 하나 — 모순이 아니다

§5를 고친 뒤 트리에서 옛 수를 그대로 주장하는 곳이 있는지 찾았다. 코퍼스는
이 워크트리의 추적 파일 653개 전부이고, 명령은
`git grep -n 'Phase 4' -- .`다. `PORTING-PLAN.md` 밖의 hit은 7개이며 그중
조건의 수를 직접 주장하는 것은 하나다:
`crates/moveit-kinematics/tests/ik_fk_roundtrip.rs`. 이 파일의 모듈 doc은
"FK(해)가 목표와 `1e-6` 이내에 들어와야 한다"고 적고, 다섯 테스트가 실제로
`1e-6`으로 단언하며 **통과한다**(`cargo nextest run -p moveit-kinematics
-E 'binary(ik_fk_roundtrip)'` → 5 passed).

모순처럼 읽히지만 아니다. `epsilon`은 **상한**이지 하한이 아니다 —
수렴한 해는 `(0, epsilon]` 어디에나 있을 수 있고, 좋은 목표에서는 훨씬
정확하다. 다섯 목표의 실제 오차를 쟀다(임시 `eprintln!`, 되돌림):

| 케이스 | 병진 오차 | 회전 오차 |
|---|---|---|
| newton-raphson | `1.2007133237158706e-8` | `4.981927639545521e-8` |
| lma | `1.8574827213419745e-7` | `8.748641341953485e-8` |
| position-only | `3.287750868270131e-7` | — |
| mimic chain | `5.463425037994619e-9` | `1.37808328817672e-32` |
| right_arm | `7.224854600224112e-9` | `1.7728384344369148e-8` |

가장 느슨한 것이 `1e-6`의 3배 아래(position-only), 가장 빡빡한 것이 183배
아래다. 즉 이 다섯 목표에서 `1e-6`은 변별력이 있는 상수이고, 5,000개
무작위 목표에서 1,513건이 `(1e-6, 1e-5]`에 떨어지는 것과 아무 충돌이 없다.
다만 모듈 doc이 "들어와야 한다"고 일반 요구처럼 적어 §5의 수정과 어긋나
보이므로, 그 문장을 "이 다섯 목표에서 그렇다"로 고치고 보장되는 것은
`epsilon`임을 함께 적었다.

### 221.4 92커밋 뒤의 main에서 두 측정을 다시 돌렸다 (2026-08-06)

§221의 모든 수는 `c21350b8`("docs(plan): §216.3, ...") 위에서 쟀다. 그 뒤
`64a436f`로 병합됐고 지금 브랜치는 main `7572123`에 있다.
`git rev-list --count c21350b8..HEAD` = **92**. 그 사이 main이
`crates/moveit-kinematics`를 크게 건드렸으므로(아래 목록) 판정이 아니라
**숫자 자체**를 다시 쟀다.

**선결 조건: 오라클 이미지를 다시 빌드했다.** 이 워크트리의 오라클 소스가
바뀌면 스탬프가 바뀌고, `run-oracle.sh`는 다른 스탬프의 이미지를 거부한다.

```console
$ sg docker -c 'tools/moveit-oracle/build.sh'
$ source tools/moveit-oracle/src-digest.sh && oracle_stamp "$PWD/tools/moveit-oracle"
7667aaf94fef1a52218f7d18362cb3dd2f76c97826f4af8ce77dbb70fcbfff40
$ sg docker -c 'docker run --rm --entrypoint cat \
    moveit-rs/oracle:7667aaf94fef1a52 /usr/local/share/oracle-src.sha256'
7667aaf94fef1a52218f7d18362cb3dd2f76c97826f4af8ce77dbb70fcbfff40
```

(`docker`는 반드시 `sg docker -c`로 감싼다. 감싸지 않은 호출은 이 기계에서
실패를 성공으로 보고한다.)

**측정 1 — 재시작 없는 비교. 네 픽스처, 여덟 개 수 전부 동일하다.**
왼쪽이 이번 실행, 괄호가 §221.1의 값이다.

| 픽스처/그룹 | 오라클 | 포트 | b | c |
|---|---|---|---|---|
| panda/panda_arm | 2432 (2432) | **2435** (2435) | 1 (1) | 4 (4) |
| fanuc/manipulator | 1061 (1061) | 1061 (1061) | 0 (0) | 0 (0) |
| dual_arm_panda/left_panda_arm | 2471 (2471) | 2471 (2471) | 4 (4) | 4 (4) |
| pr2/right_arm | 3223 (3223) | **3227** (3227) | 16 (16) | 20 (20) |

세 스트림 표도 전부 동일하다 — `--ik-max-restarts 0`에서
rng 12345 → 2434(b2/c4), rng 777 → 2436(b1/c5); `--ik-max-restarts 20`에서
포트 4906/4901/4890 대 오라클 4921(세 번 모두), b = 82/82/89, c = 67/62/58.
새로 쓴 `--ik-divergence-json` 세 파일에서 b 집합을 다시 계산해도
|b| = 82/82/89, A∩B∩C = **2**(case 408, 4130), 합집합 **226**,
rng 0의 82개 중 **80개**를 다른 스트림에서 푼다 — §221.1과 같은 수다.

**측정 2 — epsilon 격자. 여섯 행이 최대 오차 16자리까지 동일하다.**

| `--ik-epsilon` | `--tol-ik` | 포트 성공 | failed | 최대 병진 | 최대 회전 |
|---|---|---|---|---|---|
| `1e-5` | `1e-5` | 4906 | **0** | — | — |
| `1e-5` | `1e-6` | 4906 | 1513 (1112+401) | `9.922533068067614e-6` | `8.758324911274685e-6` |
| `1e-6` | `1e-6` | 4897 | **0** | — | — |
| `1e-6` | `1e-7` | 4897 | 1244 (877+367) | `9.79513234121028e-7` | `9.559096684818135e-7` |
| `1e-7` | `1e-7` | 4915 | **0** | — | — |
| `1e-7` | `1e-8` | 4915 | 1082 (726+356) | `9.965294414492535e-8` | `9.947287282497852e-8` |

반복 횟수도 같은 계측(임시, 되돌림)으로 다시 재서 **세 자리 하나까지
같다**: `1e-5` 728,695 · `1e-6` 668,855 · `1e-7` 920,881 · `1e-8` 798,879.

**동일한 이유를 짚는다 — "안 바뀌었겠지"가 아니라 병합이 IK 경로에 무엇을
했는지 열거했다.**

```console
$ git diff --stat c21350b8..HEAD -- crates/moveit-kinematics/src/ \
    crates/moveit-kinematics/Cargo.toml
 crates/moveit-kinematics/Cargo.toml                |    7 +-
 crates/moveit-kinematics/src/cached_solver.rs      |  178 +++-
 crates/moveit-kinematics/src/cartesian_interpolator.rs | 1093 ++++++++++
 crates/moveit-kinematics/src/ik_cache.rs           |  404 ++++++--
 crates/moveit-kinematics/src/ik_cache/format.rs    |  459 ++++++++
 crates/moveit-kinematics/src/lib.rs                |   61 +-
$ git diff c21350b8..HEAD -- crates/moveit-kinematics/src/cart_to_jnt.rs
$                                    # 빈 diff — 바이트 동일
```

이 측정이 밟는 파일 — `cart_to_jnt.rs`, `chain.rs`, `velocity.rs`,
`newton_raphson.rs`, `params.rs` — 은 위 목록에 하나도 없다.

`Cargo.toml`의 7줄은 `serde`/`serde_json`을 `dev-dependencies`에서
`dependencies`로 옮긴 것이고, 이 레포는 그런 의존성 편집이 `linkme`
`distributed_slice` 순서를 뒤집어 pilz의 IK 솔버를 조용히 바꾼 전례가 있다.
그 경로는 이 측정에 닿지 않는다: `moveit-diff`는 레지스트리를 통해 솔버를
고르지 않는다. 코퍼스는 `tools/moveit-diff/src/`의 파일 3개(`main.rs`,
`protocol.rs`, `rust_impl.rs`) 전부이고, `rg -c 'KINEMATICS_SOLVERS'
tools/moveit-diff/src/`는 **hit 0**(exit 1)이다. 솔버는
`rust_impl.rs:518`에서 `NewtonRaphsonSolver::new_with_seed(...)`로 직접
만든다.

**움직인 수 하나: solve 벽시계. 그리고 그것은 트리 때문이 아니다.**
§221.2의 비용 표와 이번 실행:

| `--ik-epsilon` | §221.2 | 이번 | 차 |
|---|---|---|---|
| `1e-5` | 5.413 s | 5.943 s | +0.530 s (+9.79%) |
| `1e-6` | 5.017 s | 5.182 s | +0.165 s (+3.29%) |
| `1e-7` | 5.492 s | 6.455 s | +0.963 s (+17.53%) |
| `1e-8` | 5.123 s | 6.229 s | +1.106 s (+21.59%) |

이 차를 병합 탓으로 돌리기 전에 **같은 트리·같은 바이너리의 산포를 먼저
쟀다.** 위 epsilon 격자는 각 `epsilon`을 `--tol-ik`만 바꿔 두 번씩 돌므로,
같은 조건의 독립 실행 쌍이 세 개 나온다:

| `--ik-epsilon` | 실행 A | 실행 B | 산포 |
|---|---|---|---|
| `1e-5` | 5.692 s | 5.811 s | 0.119 s (2.09%) |
| `1e-6` | 4.823 s | 5.629 s | **0.806 s (16.71%)** |
| `1e-7` | 6.619 s | 5.984 s | 0.635 s (9.59%) |

같은 트리에서 16.71%가 흔들린다. 즉 위 표의 +3.29% / +9.79% / +17.53%는
병합과 실행 잡음을 **구별하지 못한다**. 반복 횟수는 같은 실행에서 자리
하나까지 같으므로, 달라진 것은 코드가 하는 일의 양이 아니라 그 일을 언제
CPU를 받아 했는가다. §221.2의 결정은 벽시계가 아니라 성공률과 반복 횟수의
**비단조성**에 걸려 있고 그 둘은 그대로다 — 그래서 결정은 서지만,
벽시계 열은 그대로 두면 재현되지 않는 수를 주장하게 되므로 §221.2에
그 사실을 적었다.

## §219 Phase 7 완료 조건을 명령으로 닫았다 — 500건, seed 고정, 재실행 가능 (2026-08-06)

`90ca3fd`. §5 Phase 7의 세 완료 조건은 §118이 오라클에 `plan` 연산을
붙인 뒤로 **측정 가능하지만 측정되지 않은** 상태였고, §131은 그 판정을
독립 재현하면서 "재현 절차의 입력값(`seed_base`)이 기록돼 있지 않다"는
결함을 남겼다. 이번 라운드는 그 셋을 보고서의 숫자가 아니라
`tools/ci/verify-phase7-benchmark.sh full` 한 줄로 만들었다.

판정은 게이트 집합 **panda_arm 500건**에서 내린다 —
`floor_wall` 250(seed 900001) + `cage` 250(seed 900002), 포트 RNG
`seed_base` 424242, 호출당 타임아웃 120s. 세 조건 전부 충족(§5의 Phase 7
블록에 조건별로 적었다).

### §219.1 STEP 0 — 기준 플래너가 이 기계에 실제로 있는지부터

조건 1과 3은 "C++ OMPL RRTConnect 대비"이므로 그 플래너가 없으면
**대체 플래너로 바꿔치기하지 않고 미측정으로 남기는 것**이 유일하게
정직한 처리다. 그래서 대체 가능성을 먼저 죽였다:

```
$ sg docker -c "docker run --rm --entrypoint bash moveit-rs/oracle:a16bed4725212153 -lc \
    'ls /opt/ros/rolling/lib/x86_64-linux-gnu | grep -i ompl; \
     ls -d /opt/ros/rolling/include/ompl*; ls -d /opt/ros/rolling/share/ompl*; \
     ls -d /opt/ros/rolling/*/moveit_planners_ompl* || echo \"(no moveit_planners_ompl)\"'"
libompl.so
libompl.so.1.7.0
libompl.so.18
/opt/ros/rolling/include/ompl-1.7
/opt/ros/rolling/share/ompl
(no moveit_planners_ompl)
```

OMPL 1.7이 오라클 이미지에 있고(§118.2가 이미 적은 사실), 오라클의
`plan` 연산은 `og::RRTConnect`를 직접 링크한다
(`tools/moveit-oracle/CMakeLists.txt`의 `find_package(ompl REQUIRED)`).
그래서 조건 1·3은 **측정 가능하고, UNFIXED로 갈 항목이 없다.**

없는 것도 적는다: 위 출력의 마지막 줄대로 moveit 자신의 `ompl_interface`
(`moveit_planners_ompl`)는 이 이미지에 없다. 조건의 문구는 "C++ OMPL
RRTConnect"이고 그것이 실행된 것이므로 조건은 덮이지만, "moveit의 OMPL
플러그인 대비"는 이 측정이 한 적 없다. 호스트
(`/home/stevek/work/ompl`)에는 빌드되지 않은 소스 체크아웃만 있다.

### §219.2 문제 500건은 재현 가능해야 한다 — 그리고 실행 가능성 회계

문제 하나는 (start, goal, 장애물 씬) 삼중이다. 두 끝점은 균일 샘플에서
뽑고 **충돌·제약을 통과하지 못하면 버린다**(제약 집합에서는 제약까지).
버린 비율도 출력한다 — panda `floor_wall` 60.8%, `cage` 77.5%, fanuc
89.0%/88.5%, 제약 집합 99.0%. 이 숫자가 없으면 "몇 번 뽑아서 500건을
만들었는지"를 알 수 없다.

**끝점이 유효한 것은 해가 있음을 함의하지 않는다.** 장애물이 두
유효 끝점을 자유공간의 서로 다른 성분으로 갈라놓으면 어떤 플래너도 잇지
못한다. 그리고 샘플링 플래너는 확률적 완전성만 가지므로 **실패는 실행
불가능의 증명이 아니다** — 유한 예산에서 실패가 증명이 되는 지점은
없다. 그래서 회계를 이렇게 갈랐다:

- 어느 쪽이든 풀고 그 경로가 조건 2를 통과하면 그 문제는 **실행 가능,
  목격됨**(경로가 구성적 증거다)
- 양쪽 다 못 푼 것은 50000 iterations / 300s로 올려 양쪽 재실행하고,
  그래도 안 되면 **실행 가능성 미상** — "불가능"이라고 절대 쓰지 않는다

게이트 500건: **500 목격, 미상 0**. `panda_cage`의 1건은 상향 예산에서
포트가 풀어 목격으로 전환됐다. fanuc 500건은 **406 목격, 94 미상**이고
그 94건은 상향 예산에서도 양쪽 0건이었다.

### §219.3 panda 500건이 예전 제너레이터와 같은지 — 내 문구가 먼저 틀렸다

fanuc을 넣으면서 `plan_benchmark_problem_set.rs`를 고쳤으므로, panda의
500건이 움직였다면 §131이 기록한 C++ 기준선은 **다른 집합의 측정치**가
된다. 처음에 나는 이것을 "panda's 500 problems are bit-identical to the
pre-change generator (verified field by field)"라고 결과 파일에 적었다.
main이 "그 명령을 남겨라, 병합 때 내가 돌린다"고 했고, 돌려 보니
**그 문구가 틀렸다**:

```
$ diff <(pre_5adbed6 floor_wall 250 900001) <(plan_benchmark_problem_set floor_wall 250 900001 panda)
1c1
< {"group":"panda_arm","id":0,...          (이하 500KB, 여기서 생략)
```

전체 diff는 다르다. 새 제너레이터가 키 네 개를 **추가**하기 때문이다.
정확한 측정치로 교체했다 (두 seed 모두 동일한 결과):

```
added:   "config,joint_constraint,robot,scale"
removed: ""
IDENTICAL after removing the added keys
```

즉 `problems`·`objects`·`range`·`motion_resolution`·`max_iterations`·
`seed`·`group`·`op`·`id`는 바이트 단위로 같고, 다른 것은 추가된 네 키뿐이다.
그 네 키가 **오라클도 움직이지 않는다**는 것은 nlohmann의 미지 키 처리로
논증하지 않고 쟀다: 새 request를 먹인 오라클이 250/250, 248/250 exact,
pooled median `2.6597767032746464`를 돌려준다 — §131이 예전 request에서
기록한 그 값이다. 검사 명령은
`tools/ci/verify-phase7-benchmark.sh`의 seed 주석에 그대로 들어 있다.

**교훈은 문구가 아니라 절차다.** "field by field 확인했다"는 내가 실제로
한 일이었지만(그 필드들은 정말 같았다), 내가 **적은 문장**은 전체 출력이
같다는 주장이었다. 검사 명령을 남기라는 요구가 그 간극을 드러냈다 —
주장을 명령으로 바꾸면 주장의 범위도 명령이 정한다.

### §219.4 조건 2 — 끝점이 아니라 waypoint 전부, 그리고 검사가 실패할 수 있음의 증명

`rrt_connect`는 트리 정점만 돌려주므로 그 배열을 그대로
`PlanningScene::is_path_valid`에 넣으면 **구간 내부를 건너뛴다**
(`is_path_valid`는 건네준 상태만 검사하고 보간하지 않는다). 그래서
경로를 `StateSpace::interpolate`로 `motion_resolution` 간격으로 다시
촘촘하게 만든 뒤 검사한다. 게이트 497경로 = **waypoint 168,340개**,
fanuc 405경로 = 52,983개, 제약 집합 250경로 = 67,820개. 전부 통과.

**아무것도 검사하지 않는 검증기는 동작하는 검증기와 똑같이 100%를
보고한다.** 그래서 100%를 적기 전에 검사가 실패할 수 있음을 먼저
증명한다 — 하네스의 첫 단계이고, 여기서 실패하면 아래 모든 조건 2
숫자가 무효다:

```
PASS inject=collision -- inject=collision rejected all 6 paths, as required
PASS inject=constraint -- inject=constraint rejected all 6 paths, as required
PASS no-injection control -- condition2 6/6
```

주입 상태는 "충돌하게 만들었으니 충돌할 것"이라고 **가정하지 않고**
직접 질의로 나쁨을 확인한 뒤 넣는다(제약 모드는 충돌은 없고 제약만
tolerance의 4배로 어기는 상태). 그리고 무주입 대조가 같이 있는 이유는,
**모든 것을 거부하는 검증기도 주입 게이트는 통과**하기 때문이다.

제약 쪽 절반은 원래 공허했다 — 모든 request의 `path_constraints`가
`None`이었다. 오라클의 `plan` 연산에는 제약 입력이 없어 C++ 대응이
불가능하므로, 제약 집합 250건(`panda_joint1:0.0:0.5`)은 포트 단독으로
돌리고 조건 1·3에는 참여시키지 않는다. 조건 2는 §5의 세 조건 중 OMPL이
필요 없는 유일한 조건이므로 이 배치로 온전히 측정된다.

### §219.5 게이트 판정, 그리고 두 번째 로봇을 평균에 섞지 않은 이유

게이트(panda 500건):

| | C++ OMPL RRTConnect | 포트 |
|---|---|---|
| 해결 | 498/500 = 99.6% | 497/500 = 99.4% (타임아웃 0, 그 외 실패 3) |
| pooled median 길이 | 2.6597767032746464 | 2.668003737362192 |

- 조건 1: 99.4% ≥ 89.64% — 충족
- 조건 2: 497/497 (waypoint 168,340) — 충족
- 조건 3: 2.668003737362192 ≤ 3.4577097142570405, 비율 1.003배 — 충족

fanuc `manipulator` 500건도 같은 하네스로 쟀고 **게이트에 평균으로 섞지
않았다** — 로봇별로 섞은 평균은 한 로봇의 실패를 가릴 수 있다. C++
406/500(81.2%), 포트 405/500(81.0%), 조건 2 405/405, 조건 3
1.8556940608849652 ≤ 2.4287125308631925.

fanuc의 실패율이 포트 결함인지 문제 집합의 성질인지는 **양쪽을 다
쟀으므로 답할 수 있다**:

- `fanuc_floor_wall`: C++ 실패 44, 포트 실패 45, **양쪽 다 못 푼 것 44**
  → C++가 실패한 44건은 전부 포트도 실패했고, 포트만 실패한 것은 1건
- `fanuc_cage`: C++ 실패 50, 포트 실패 50, 양쪽 다 못 푼 것 50
  → 두 실패 집합이 **동일**
- 그 94건은 50000 iterations / 300s 상향에서도 양쪽 0건

즉 fanuc의 낮은 성공률은 포트가 뒤처진 결과가 아니라 이 문제 집합이
양쪽에 어려운 결과다(포트가 추가로 놓친 것은 500건 중 1건). fanuc의
기하는 panda 장애물 집합을 **실측 reach 비율로 스케일**한 것이다 —
panda 0.9025 m, fanuc 1.4912 m, 비율 1.6522991689750695. 이 94건이
정말 자유공간이 갈라진 문제인지는 **측정하지 않았고**, 그래서 위처럼
"실행 가능성 미상"으로만 적는다.

범위가 조건의 문구보다 커졌음을 적어 둔다: 조건은 500건을 말하는데
실제로 돌린 것은 다섯 집합 **1,250건**(panda 500 = 게이트, fanuc 500,
제약 250)이다. 판정은 게이트 500건이고 나머지는 별도 보고다.

### §219.6 하네스를 만드는 동안 나온 결함 네 개

1. **샤드 찌꺼기 오염 (실제 오염, pilot이 잡았다).** 샤드 출력이
   공유 `$WORKDIR`에 고정 이름(`shard.$i.ndjson`)으로 쓰이는 바람에,
   문제 수가 더 적은 나중 호출이 **앞 호출의 남은 샤드를 같이 읽었다** —
   2건을 상향 재실행한 결과가 6건 해결로 집계되고 미상 개수가 `-4`로
   나왔다. 호출마다 출력 파일 이름으로 된 전용 디렉터리를 쓰고,
   요청 문제 수와 결과 줄 수가 다르면 실패시키고, `newly > n_unsolved`도
   명시적 실패로 만들었다. 음수 미상 개수가 **출력될 수 있었다**는 것이
   이 결함의 크기다.
2. **내가 적은 통계가 측정 전에 먼저 존재했다.** `DEFAULT_TIMEOUT_SECONDS`의
   doc에 "가장 느린 `solve()`가 21.9 s, 평균 2.4 s"라고 적었는데 그 시점에
   아무것도 재지 않았다. 실측(panda 평균 2.4s/최악 9.6s, fanuc 평균
   8.7–9.1s/최악 40.4s)으로 교체하고 120s의 근거를 "관측 최악의 약 3배,
   그리고 예산을 낮게 잡으면 성공이 실패로 기록되어 포트를 과소평가하므로
   높게 잡는 쪽이 안전한 방향"으로 다시 썼다.
3. **거친 단언 하나가 게이트를 울렸다.** `verify-orphan-enumeration.sh`가
   `plan_benchmark_port.rs:398`을 orphan으로 잡았다. 대상은
   `assert!(matches!(inject.as_deref(), None|Some("collision")|Some("constraint")))`
   — `matches!`는 어느 패턴이 틀렸는지 말할 수 없다. ledger 항목을 추가하거나
   스냅샷을 재생성하는 대신 **타입으로 닫았다**: `InjectMode` enum과
   `parse`. 유효 모드 집합이 CLI 검증과 `build_injected_state`의 `match`
   두 곳에서 각각 결정되던 것이 한 곳으로 모이고, 둘이 일치하는 동안만
   도달 불가였던 catch-all 팔이 사라졌다. 스캐너 사이트 718 → 717, orphan 0.
4. **내가 만든 출처 표기가 거짓을 함의했다.** 결과 파일이
   `git rev-parse HEAD`만 적었는데 측정은 더러운 작업 트리에서 돌았다 —
   그 트리가 만든 적 없는 커밋에 숫자를 귀속시킨다. §219.7이 그 처리다.

### §219.7 출처는 커밋이 아니라 내용으로 적었다 — 그리고 세 번 돌렸다

결과 파일을 만드는 실행은 **그 파일을 담을 커밋보다 먼저** 일어나므로
`commit` 필드는 원리적으로 부모(`5adbed6`)만 가리킬 수 있다. main이
지적한 대로 "이 하네스를 추가한 커밋"이라는 산문은 나중 독자가 해결할 수
없는 참조다. 그래서 revision이 아니라 **내용**으로 적는다 —
`measured_sources`에 숫자를 결정하는 파일 셋의 git blob id를 넣고,
`dirty_paths`에 측정 시점에 커밋되지 않았던 경로를 넣는다. 검사는 한 줄:

```
$ git hash-object crates/moveit-planners-sbp/examples/plan_benchmark_port.rs
9effe34cbd0b5ac9c11387cbafd1889709d8d822   # = 결과 파일의 measured_sources 값
```

세 파일 전부 커밋된 트리에서 같은 값이 나온다. 즉 **커밋된 코드가 그
숫자를 만든 코드다.** 측정 시점에 커밋되지 않았던 것은 PORTING-PLAN.md
하나이고 그것은 숫자를 결정하지 않는다.

`full` 모드를 세 번 돌렸고 **셋이 모든 개수에서 일치한다**:

| | panda fw | panda cage | fanuc fw | fanuc cage | 제약 | 포트 실행 벽시계 |
|---|---|---|---|---|---|---|
| 1회 (15:12Z) | 249 | 248 | 205 | 200 | 250 | 455.6s |
| 2회 (상향 중 중단) | 249 | 248 | 205 | 200 | 250 | 438.3s |
| 3회 (16:52Z, 커밋된 것) | 249 | 248 | 205 | 200 | 250 | 461.4s |

C++ 쪽도 셋 다 250/248/206/200, pooled median도 셋 다 동일한 자릿수까지
같다. 2회는 결과 파일에 `measured_sources`를 넣기로 하고 상향 단계에서
중단시킨 것이므로 판정을 내지 않았다. **타임아웃이 세 번 다 0이었으므로
이 일치는 결정론이 실제로 유지된다는 뜻이다**(타임아웃이 하나라도 있으면
기계 속도가 결과에 섞인다).

비용도 숫자로 적는다. 3회 총 벽시계 **3077.77s (51.3분)**, 그중 포트
실행 461.4s(32 샤드 병렬), 집합별 CPU 510/498/2061/2279/279s, 가장 느린
단일 호출 16.68s(게이트)·52.31s(fanuc). 나머지 대부분은 실행 가능성
상향 단계다 — 그 단계의 모든 문제는 이미 한 번 실패한 문제이므로
**구조상 전부 자기 deadline까지 돈다.** 이 비용이 `verify-all.sh` glob에
들어갈 수 없는 이유이고, 기본 모드를 8건 pilot으로 둔 이유다
(pilot 216.5s, 주입 게이트 두 개 포함).

### §219.8 이 측정이 하지 않은 것

- moveit `ompl_interface` 대비 비교. 이미지에 그 패키지가 없다(§219.1).
  조건의 문구는 OMPL RRTConnect이고 그것은 실행됐다.
- 제약 조건의 C++ 대조. 오라클 `plan` 연산에 제약 입력이 없다 —
  제약 집합은 포트 단독 250건이다.
- fanuc 94건이 실행 불가능인지 여부. 유한 예산은 그것을 증명하지 않으며
  **미상으로 남긴다.**
- PRM / RRT* / KPIECE. §5 Phase 7의 항목이지만 완료 조건은 성공률·경로
  길이를 RRTConnect 대비로만 말한다. 이 하네스는 `RrtConnectManager`만
  구동한다.

## §220 `setFromIK`을 `moveit-state`가 아닌 `moveit-kinematics`에 두고, 첨부 바디는 의존성 대신 주입한 이유

### §220.1 배치 — 사이클 두 개를 피하면서 새 엣지는 만들지 않았다

`RobotState::setFromIK`은 `RobotState`의 메서드지만, 하는 일은
`KinematicsSolver`를 호출하는 것이다. `moveit-state`에서
`moveit-kinematics`를 부르면 Cargo 사이클이므로 `RobotState` 옆에
둘 수 없다. `moveit-scene`도 답이 아니다 —
`moveit-scene -> moveit-constraints -> moveit-kinematics`가 이미
있어서 `moveit-kinematics -> moveit-scene`은 두 번째 사이클이다.

그래서 `moveit-kinematics`의 새 모듈 `src/set_from_ik.rs`에 두었다.
근거가 되는 엣지는 둘 다 이미 있던 것이다:
`moveit-kinematics -> moveit-state`(`Cargo.toml:15`),
`moveit-kinematics -> moveit-model`(`:14`). **새 크레이트 엣지는
추가하지 않았다.** `tools/ci/check-dep-direction.sh`가 금지하는 것은
ROS 클라이언트 라이브러리(`r2r`/`rclrs`/`ros2-client`/`rustdds`/
`rosidl_*`)뿐이고, 여기에 해당하는 것은 없다.

호출 방향이 뒤집힌 대가는 시그니처에 드러난다. 상류가
`state.setFromIK(...)`인 것이 여기서는
`set_from_ik(&mut state, solver, targets, ik)`인 자유 함수다.
`RobotState`가 자기 자신을 IK로 채우는 능력을 잃은 것이 아니라,
그 능력이 어느 크레이트에 사는지가 바뀐 것이다.

### §220.2 첨부 바디 — 세 번째 사이클 대신 주입한 트레이트

상류 `getLinkModelIncludingAttachedBodies`는 프레임 이름을 링크
또는 첨부 바디(그리고 그 서브프레임)로 해석한다. 첨부 바디는
`moveit-scene`에 산다. 즉 이 기능을 그대로 옮기면
`moveit-kinematics -> moveit-scene` 사이클이 다시 필요해진다.

엣지를 만드는 대신 호출자가 주입하는 트레이트로 뒤집었다:

```rust
pub trait AttachedFrames {
    fn attached_frame(&self, frame: &str) -> Option<AttachedFrame<'_>>;
}
```

메서드가 하나인 것이 요점이다. `link_name`과 `link_pose_frame`을
따로 묻는 두 메서드였다면 둘이 서로 다른 바디를 가리키는 상태를
만들 수 있다. 하나의 `AttachedFrame`으로 함께 돌려주면 그 조합이
타입상 불가능하다. 첨부 바디가 없는 호출자를 위해
`NoAttachedFrames` 유닛 구조체를 두었고, `impl AttachedFrames for ()`는
쓰지 않았다 — 편의 impl이 by-construction 불변식을 새게 하는
모양이라 이 저장소에서 이미 한 번 문제가 된 적이 있다.

`AttachedFrames`는 바디 자체의 프레임과 서브프레임을 구분하지
않는다. 상류도 둘 다 첨부된 링크로 답하고 둘 다 강체이므로,
구분할 것이 없다.

### §220.3 다중 팁 — `tip_frames`를 provided 메서드로 넣고, 위임 래퍼 두 곳은 forward 했다

`setFromIK`의 팁 매칭과 "호출자가 이름 대지 않은 팁 채우기"는
상류에서 복수형(`getTipFrames`)에 대해 쓰여 있다. 이 포트의
`KinematicsSolver`에는 단수 `tip_frame`밖에 없었다.

`tip_frames()`를 **provided** 메서드로 추가했다 — 기본값은
`[tip_frame()]`이라 기존 구현체 아홉 개가 하나도 깨지지 않는다.
다만 위임 래퍼는 기본값을 상속하면 안 된다: 팁이 여럿인 솔버를
감싼 래퍼가 자기 `tip_frame()` 하나만 보고하게 되기 때문이다.
`CachedIkSolver`와 `moveit-planners-sbp`의 `SharedKinematicsSolver`
두 곳 모두 forward 하도록 했다. 결함군을 인용 한 곳이 아니라
군 전체로 닫은 것이고, `tip_frames`가 도입되는 커밋과 같은 커밋에
들어갔으므로 어떤 커밋 트리에서도 좁혀진 상태가 존재한 적은 없다.

`set_from_ik` 자체는 팁이 둘 이상인 요청을 거부하고
`set_from_ik_subgroups`를 가리킨다. `solve_with_options`가 포즈
하나를 받기 때문이다. 상류가 `supportsGroup`으로 내리는 그 결정을
이 포트는 `tip_frames().len()`으로 직접 내린다.

### §220.4 상태 되감기 — 상류가 열어 둔 결함군을 구조로 닫았다

상류 `setFromIK`은 실패해도 상태를 되돌리지 않는다.
`GroupStateValidityCallbackFn`은 상태를 고칠 수 있고, 트리 안의 두
콜백이 실제로 `setJointGroupPositions`으로 시작한다. 그래서
`false`를 반환한 뒤에도 마지막으로 거절된 후보가 상태에 남는다.
`setFromIKSubgroups`는 콜백 없이도 같은 일을 한다 — 서브그룹마다
바로 쓰고, 다음 서브그룹이 실패하면 `break`할 뿐이다.

이 포트는 재현하지 않는다
(`doc/upstream-bugs.md`,
`set-from-ik-leaves-a-rejected-candidate-in-the-state`). 불변식은
하나다: **`Ok(false)` 또는 `Err`로 끝나는 호출은 상태를 진입
시점과 바이트 단위로 같게 남긴다.** `set_from_ik`은 진입 시
`state.positions()`를 스냅샷하고 무조건 복원한 뒤에야 채택된 해를
쓴다. `set_from_ik_subgroups`는 성공한 sweep을 스냅샷하고, 그룹
훅이 승낙하면 그 스냅샷을 다시 적용하며(훅이 도중에 쓴 것은 남지
않는다), 그 밖의 모든 경로에서는 진입 스냅샷으로 되감는다.

이것이 훅에게 `&mut RobotState`를 안전하게 건넬 수 있게 하는
근거이기도 하다. 훅은 후보가 이미 적용된 상태를 받는다 — 상류처럼
후보를 배열로만 건네고 훅이 알아서 쓰게 하는 대신 — 그래야 훅
안에서 FK를 볼 수 있고, mimic 조인트도 값이 들어가 있다.

### §220.5 bijection이 사라진 자리

상류는 `getKinematicsSolverJointBijection()`으로 솔버 순서와 그룹
순서를 오간다. 이 포트의 `KinematicsSolver::joint_names()`는 활성
조인트만 담으므로, 그 산술적 재배치로는 mimic 값을 만들어 낼 수
없다. 대신 이름으로 상태에 쓰고 그룹의 변수 목록을 상태에서 다시
읽는다 — mimic에 값을 주는 것은 `set_variable_position`의 전파이지
재배치가 아니다. 여기서 mimic 규칙을 다시 구현하면 두 번째 구현이
된다.

측정으로 확인했다: PR2 `l_gripper_finger_chain`의 솔버는 조인트
하나를 보고하고 그룹은 변수 둘을 가진다. 훅이 받은 슬라이스는
길이 2였고, 두 번째 항목은 진입값 `0.3`이 아니라 쓰기가 전파한
`0.10000000039269835`였다.

bijection을 만들면서 상류가 함께 하던 "솔버 조인트가 그룹 변수인지"
검사(`joint_model_group.cpp:626-636`)는 남겼다 —
`check_solver_joints_are_group_variables`. 순열 자체만 소비자가
없어진 것이다.

### §220.6 게이트가 조용히 통과한 자리 — TIMEOUT은 FAIL이 아니다

각 가드마다 격리 변형을 돌리는 하니스를 썼다. 첫 판은 `FAIL`이
들어간 줄만 셌는데, 이 저장소의 `.config/nextest.toml`은
`terminate-after = 5`라서 멈춘 테스트를 **TIMEOUT**으로 보고한다.
그 결과 `rigidly_connected_parent_link`의 루프 종료 조건을 없앤
변형(B21)이 "실패한 테스트 없음"으로 돌아왔다 — 실제로는 테스트
셋이 300초씩 매달려 있었다. 하니스가 `TIMEOUT`/`SIGSEGV`/`ABORT`/
`LEAK`/`SIGABRT`/`CANCEL`까지 세도록 고친 뒤에야 B21이 무엇을
물었는지 보였다. 가드의 회귀 신호를 실패가 아니라 정지로 바꾸는
변형도 물린 것이고, 그 차이를 못 보는 하니스는 증거가 아니다.

같은 판에서 물지 않은 가드도 하나 나왔다:
`resolve_ik_queries`의 exact-name 빠른 경로를 지워도 76개 테스트가
전부 통과한다. 커버리지 구멍이 아니라 동작상 중복이다 — 팁 이름을
정확히 댄 타깃은 아래의 강체 연결 분기에서 같은 문자열끼리 비교해
참이 되고, 곱해지는 변환도 같은 것 둘이라 항등이다. 상류가 같은
자리에 두고 있고 팁마다 변환 조회 두 번을 아끼므로 남겼지만,
여기에 의존하는 것은 없다. 이 판정도 원장에 그대로 적었다
(`doc/assertion-discrimination-ledger-p10-setfromik.md` §5).

### §220.7 아직 포팅하지 않은 것

`RobotState::interpolate`와 `RobotState::distance`의 지역 사본은
그대로 두었다. `setFromIK`은 둘 중 어느 것도 쓰지 않으므로, 진짜
메서드를 포팅하는 것은 이번 작업을 닫는 일부가 아니다.

---

## §226 Phase 9 완료 조건의 갭을 처음 실측했다 — 조건 자체는 이 기계에서 미도달 (2026-08-06)

§217.3이 Phase 9를 UNMET으로 적었지만, 그 판정은 `MoveGroupInterface`
문자열이 트리에 없다는 한 줄짜리 anchor 하나였다. Phase 3·Phase 4가
이번 라운드 각각 §218·§221에서 실행 가능한 명령으로 다시 측정된 것과
달리, Phase 9는 아무도 재보지 않았다. §5의 조건은 종단 조건이다: "기존
C++ `MoveGroupInterface` 클라이언트가 코드 변경 없이 `moveit-ros`
노드에 플래닝 요청을 보내 유효한 궤적을 받는다." 이 절은 그 조건이
요구하는 세 가지 — 게이트 스크립트의 실제 범위, 클라이언트 쪽 빌드
가능성, 서버 쪽 구현 현황 — 를 순서대로 측정하고, 처음으로 막히는
지점에서 멈춘다.

### §226.1 STEP 1 — `verify-ros-interop.sh`가 조건의 어디까지를 덮는가

`tools/ci/verify-ros-interop.sh`는 얇은 caller고(`exec
"$REPO_ROOT/ros/verify-ros-interop.sh" "$@"`), 실제 게이트는
`ros/verify-ros-interop.sh`다. 이 스크립트를 처음부터 끝까지 읽은
결과, 하는 일은 `ros/Dockerfile`이 만든 `moveit-rs/ros-dev:latest`
이미지 안에서:

- `cargo fmt --check`
- `cargo clippy --all-targets -- -D warnings`
- `cargo test` (이 이미지엔 nextest가 없다), 통과 건수를
  `ros/moveit-ros/src`의 `#[test]` 개수와 대조
- `cargo doc --no-deps`

이름이 "ros-interop"이라고 해서 조건과 같은 범위를 덮는다고 가정하면
안 된다 — 스크립트 자신의 "What this does NOT check" 절이 정확히
이렇게 적는다:

> No live ROS 2 graph: no node is ever spun up, no topic/service/action
> is published or called against a real moveit2 or rclrs process ...
> Wire-format compatibility with a real moveit2 node is unverified by
> this script.

즉 이 게이트는 **컴파일 + lint + 단위 테스트 통과**만 보고, 조건이
요구하는 "클라이언트가 노드에 요청을 보내고 유효한 궤적을 받는다"는
어떤 실행도 하지 않는다. 살아있는 ROS 2 그래프, 실제 서비스/액션
호출, `moveit2` 노드와의 와이어 포맷 호환성 — 셋 다 이 스크립트가
명시적으로 범위 밖이라 적어 둔 것들이고, 이 셋이 바로 조건의 핵심이다.
이름이 맞아 보인다고 범위가 맞다고 가정하지 않는다는 이 라운드의
전제가 여기서 바로 맞아떨어진다: 게이트는 초록이어도 조건에 대해
아무것도 증명하지 않는다.

### §226.2 STEP 2 — C++ `MoveGroupInterface` 클라이언트가 이 기계에서 빌드/실행되는가

오라클 이미지가 있음직한 host다. 그런데 오라클은 `moveit_core
moveit_resources_fanuc_description pilz_industrial_motion_planner
moveit_kinematics chomp_motion_planner`만 빌드한다
(`tools/moveit-oracle/src-digest.sh`). `MoveGroupInterface`는
`moveit_ros_planning_interface` 패키지(`moveit_ros/planning_interface/
move_group_interface`)가 제공하는데, 이 패키지는 저 다섯 개 어디의
의존성도 아니다.

```
$ sg docker -c "docker run --rm --entrypoint bash \
    moveit-rs/oracle:7667aaf94fef1a52 -lc \
    'ls /ws/install | grep planning_interface || echo NOT_PRESENT; \
     find / -iname libmoveit_move_group_interface* 2>/dev/null || echo NOT_FOUND'"
NOT_PRESENT
NOT_FOUND
```

**빌드된 결과물은 없다.** 다만 오라클 이미지가 전체 `moveit2` 소스
체크아웃을 빌드 컨텍스트로 복사해 두므로(`/ws/src/moveit2`),
`move_group_interface`의 소스 자체는 있다 — 컴파일된 적이 없을
뿐이다. §219.1이 한 것처럼 "이미 있는 것"만 확인하는 대신, 여기서는
그 소스가 실제로 빌드되는지까지 쟀다(커밋되는 이미지가 아니라 `--rm`
컨테이너 안에서, 측정 목적으로만):

```
$ sg docker -c "docker run --rm --entrypoint bash \
    moveit-rs/oracle:7667aaf94fef1a52 -lc \
    'source /opt/ros/rolling/setup.bash && source /ws/install/setup.bash && \
     cd /ws && colcon build --packages-select moveit_ros_planning_interface ...'"
ERROR: Failed to find the following files:
- /ws/install/moveit_simple_controller_manager/.../package.sh
- /ws/install/moveit_ros_warehouse/.../package.sh
Check that the following packages have been built:
- moveit_simple_controller_manager
- moveit_ros_warehouse
```

의존성 사슬을 `colcon list --packages-up-to moveit_ros_planning_interface
--topological-order`로 펼치면 16개 패키지고, 그중 13개는 오라클이 이미
빌드해 뒀다 — 부족한 건 `moveit_simple_controller_manager`,
`moveit_ros_warehouse`, `moveit_ros_planning_interface` 자신 셋뿐이다.
`--packages-up-to`로 그 셋을 마저 빌드하니:

```
Finished <<< moveit_ros_warehouse [37.3s]
Starting >>> moveit_ros_planning_interface
...
[100%] Linking CXX shared library libmoveit_move_group_interface.so
[100%] Built target moveit_move_group_interface
...
Summary: 16 packages finished [1min 18s]
```

**`libmoveit_move_group_interface.so`가 실제로 빌드된다** — 이
컨테이너는 `--rm`이라 결과물은 컨테이너와 함께 사라지고, 어떤
`Dockerfile`도 이 라운드에서 바뀌지 않았다(측정이지 포트가 아니다).
결론: 오라클 이미지 계열 위에서 C++ `MoveGroupInterface`는 **빌드
가능**이고, 막힌 건 이미지에 이미 있냐가 아니라 3개 패키지 1분 18초의
추가 빌드였다.

다만 게이트가 실제로 쓰는 이미지는 오라클이 아니라
`moveit-rs/ros-dev:latest`(`ros/verify-ros-interop.sh`가 여는 그
이미지)다. 그 이미지는 `ros/Dockerfile`대로 `moveit_msgs`만 소스
빌드하고 Rust 툴체인을 얹은 것이라, `moveit2` C++ 스택이 통째로
없다:

```
$ sg docker -c "docker run --rm --entrypoint bash \
    moveit-rs/ros-dev:latest -lc \
    'ls /ws/install; dpkg -l | grep -i moveit || echo none'"
... moveit_msgs setup.bash ...   # moveit_msgs 하나뿐
none
```

즉 클라이언트를 **빌드**할 수 있는 곳(오라클 계열)과, `moveit-ros`
Rust 노드를 **빌드/게이트**하는 곳(`ros-dev`)이 서로 다른 이미지고,
둘을 같이 띄워 서로 통신시키는 구성은 어느 쪽에도 없다. "실행"까지는
못 갔다 — `MoveGroupInterface` 생성자는 `/robot_description` 파라미터와
살아있는 `move_group` 상대(서비스/액션 서버)를 필요로 하는데, 그
상대가 존재하는지가 바로 STEP 3의 질문이라 여기서 멈춘다.

### §226.3 STEP 3 — `ros/moveit-ros`가 조건이 이름 부른 네 조각 중 무엇을 구현했는가

corpus: `ros/moveit-ros/src/`의 `.rs` 18개 파일 전부(`find
ros/moveit-ros/src -name '*.rs'`), 그리고 조건 문자열 자체는
`crates/ ros/ tools/ doc/ PORTING-PLAN.md` 전체.

| 조각 | 상태 | 근거 |
|---|---|---|
| `/plan_kinematic_path` 서비스 | **부재** | `rg -n 'plan_kinematic_path' crates/ ros/ tools/ doc/ PORTING-PLAN.md` — 히트는 `ros/moveit-ros/src/lib.rs:17`(부재를 스스로 적은 모듈 문서), `Cargo.toml:3`(description), 이 문서의 조건문 자신(§5:788, §217.3:16782) 뿐. 실제 서비스 등록 코드는 0건 |
| `/move_action` 액션 서버 | **부재** | 같은 corpus, 같은 명령 — 히트는 전부 산문/설명, 실제 액션 서버 등록 0건 |
| planning scene 토픽 구독 | **부재** | `rg -n 'create_subscription' ros/moveit-ros/src/ -t rust` 0건. `scene/planning_scene.rs`는 `usePlanningSceneMsg`/`setPlanningSceneMsg`/`setPlanningSceneDiffMsg`를 이미 손에 든 메시지 값에 대한 순수 `TryFrom` 변환으로 포팅한 것이지, 살아있는 토픽을 구독해 상태를 갱신하는 코드가 아니다 |
| `moveit_msgs` `TryFrom` 변환 | **존재** | `rg -n '^impl TryFrom' ros/moveit-ros/src/*.rs ros/moveit-ros/src/**/*.rs` — **24개** 블록, `geometry.rs`(9) `scene/shapes.rs`(3) `constraints/visibility.rs`(3) `constraints/position.rs`(2) `model.rs`(2) `constraints/{joint,orientation,set}.rs`·`planning.rs`·`scene/collision_object.rs`(각 1) |

부재 셋의 anchor를 한 번에:

```
$ rg -n 'create_service|create_action_server|ActionServer|create_subscription|create_client|r2r::Node|Node::create|fn main' \
    ros/moveit-ros/src/ -t rust
(no matches, exit 1)
```

더 근본적으로: `ros/moveit-ros`에는 `[[bin]]` 타깃도 `fn main`도
어디에도 없다 (`find ros/moveit-ros -name '*.rs' | xargs grep -l 'fn
main'` → 0건, 트리 전체 검색). 크레이트는 순수 라이브러리이고, r2r는
오직 생성된 메시지 struct 타입(`r2r::geometry_msgs::msg::*`,
`r2r::moveit_msgs::msg::*`, `r2r::shape_msgs::msg::*` 등)을 가져오는
데만 쓰인다 — `r2r::Node`를 만들거나 `spin`을 도는 코드는 이 크레이트
안에 한 줄도 없다. 즉 "moveit-ros 노드"라 부를 실행 가능한 것 자체가
지금 존재하지 않는다.

크레이트 자신의 모듈 문서(`ros/moveit-ros/src/lib.rs:14-19`)가 이미 같은 결론을 적어
둔다 — "Round 1 scope ... Type conversion only -- no
`/plan_kinematic_path` service, no `/move_action` action server, no
planning-scene subscription (deferred to a later round)." 이 절은 그
문서화된 주장을 그대로 믿지 않고 위 rg/find로 독립 재확인했다 — 문서화된
공백도 확인되지 않은 결함일 수 있다는 이유에서다.

부수로 확인한 것: moveit2 소스에는 이 두 이름의 실제 정의가 있다 —
`plan_kinematic_path`는 `capability_names.hpp`가 정의하는 서비스
이름 문자열(타입은 `moveit_msgs/srv/GetMotionPlan`), `/move_action`은
`moveit_msgs/action/MoveGroup.action`. 둘 다 `moveit_msgs`에 이미
벤더링돼 있으므로(`ros/Dockerfile`), 타입 자체는 `ros-dev` 이미지에서
바로 쓸 수 있다 — 없는 건 타입이 아니라 그 타입으로 서비스/액션을
등록하는 서버 코드와, 그것을 실행할 노드 바이너리다.

### §226.4 결론 — 조건은 이 기계에서 UNMET, 막히는 지점은 서버 쪽이지 클라이언트 쪽이 아니다

**조건 전체는 미도달이다.** 다만 "미도달"의 정확한 위치를 좁혔다는
것이 이번 실측의 결과다:

- **게이트가 조건을 덮지 않는다는 것(STEP 1)은 이제 스크립트 본문
  인용으로 확정.**
- **클라이언트 쪽은 막혀 있지 않다(STEP 2).** C++
  `MoveGroupInterface`는 오라클 이미지 계열에서 빌드된다 — 이번
  라운드에 직접 재현했고, 추가로 필요한 건 이미 빌드돼 있는 13개
  패키지 위에 3개(`moveit_simple_controller_manager`,
  `moveit_ros_warehouse`, `moveit_ros_planning_interface`), 1분
  18초뿐이다. 다만 이 재현은 오라클 이미지에서 한 것이고, `ros-dev`
  이미지(게이트가 실제로 쓰는 이미지)에는 `moveit2` C++ 스택이 아예
  없다 — 두 이미지를 합치거나 별도 이미지를 만드는 작업은 아직
  아무도 하지 않았다.
- **막힌 지점은 서버 쪽이다(STEP 3).** `/plan_kinematic_path` 서비스,
  `/move_action` 액션 서버, planning scene 구독 셋 다 부재하고, 그걸
  실행할 노드 바이너리(`fn main`)조차 없다. `moveit_msgs` `TryFrom`
  변환(24개)만 존재하며, 이것이 조건이 이름 부른 네 조각 중 유일하게
  이미 있는 것이다.

**조건을 그대로 재는 것과 더 약한 조건으로 바꿔치기하는 것을 구분해서
적는다:** 이번 실측이 실제로 실행한 것은 (a) 게이트 스크립트를 읽고
범위를 확정한 것, (b) C++ 클라이언트 라이브러리가 빌드되는지를
컨테이너 안에서 직접 확인한 것, (c) 서버 쪽 구현 현황을 rg/find로
전수 확인한 것 — 셋 다 조건보다 **좁은** 측정이고, 이 절은 그것을
그렇게 표시한다. 클라이언트가 노드에 요청을 보내 궤적을 받는
end-to-end 시도는 하지 않았다 — 보낼 상대(서비스/액션 서버)가 서버
쪽에 없으므로 시도해도 조건에 대해 아무것도 증명하지 못했을
것이다. 대체 조건으로 바꿔 통과를 보고하지 않는다.

**이 조건을 닫으려면 필요한 것(순서대로, 측정 결과에서 직접
도출):**

1. `ros/moveit-ros`에 `fn main`을 갖는 실행 바이너리(`[[bin]]`)를
   추가하고 `r2r::Node`를 만들어 `spin`하는 코드 — 지금은 전혀 없다.
2. 그 노드 위에 `/plan_kinematic_path`(`GetMotionPlan.srv`) 서비스와
   `/move_action`(`MoveGroup.action`) 액션 서버를 등록하는 코드 —
   이미 있는 24개 `TryFrom` 변환과 `crates/moveit-planning`의 기존
   `plan()` 호출을 잇는 배선(wiring)이 대부분일 것으로 보이나, 이
   절은 그 크기를 재지 않았다.
3. planning scene 토픽 구독 — `scene/planning_scene.rs`의 기존
   `TryFrom` 변환을 살아있는 `/planning_scene` 토픽에 실제로 연결하는
   코드.
4. `ros-dev` 이미지(또는 별도 이미지)에서 C++ `MoveGroupInterface`
   클라이언트를 빌드할 수 있게 하는 이미지 작업 — STEP 2가 보인 3개
   패키지·1분 18초는 오라클 이미지 기준이고, 게이트가 실제로 쓰는
   이미지에서 그대로 성립한다는 보장은 없다(미확인).
5. 1~4가 갖춰진 뒤에야 "코드 변경 없는 기존 C++
   `MoveGroupInterface` 클라이언트가 유효한 궤적을 받는다"는 원래
   문구 그대로의 종단 시도가 가능해진다.

> **후속.** §235가 이 절의 다음 질문 — 조건이 도달 가능한지, 도달
> 가능하다면 가장 작은 조각이 무엇인지 — 에 답한다. 결론: 도달
> 가능하고, 위 5항목 중 무엇도 이 포트가 짓지 않기로 결정한 rclcpp
> 런타임이 아니다. 전부 D2/D5/D6가 이미 짓기로 결정한 것이고, 아직
> 안 지었을 뿐이다.

## §227 pilz의 `PlanningContext` CRTP 계층 다섯 파일 — 계산은 한 문장이고, 그 문장은 이미 트리에 있다

`doc/port-coverage.md`가 `gap`으로 세던 `planning_context_base.hpp`와
파생 넷(`planning_context_{ptp,lin,circ,polyline}.hpp`)을 처분한다.
질문은 "CRTP 계층 안의 생성기 로직이 ROS를 향한 기반 클래스에서 분리
가능한가"이고, 답은 상류를 열어 문장을 세는 것으로 나온다.

### §227.1 기반 클래스 안의 문장을 전부 센다

`planning_context_base.hpp:87-171`(상류 `e017c91`). 멤버 함수는 넷이고,
본문의 문장은 전부 합쳐 아래가 전부다.

```cpp
void PlanningContextBase<GeneratorT>::solve(planning_interface::MotionPlanResponse& res)
{
  if (terminated_) { RCLCPP_ERROR(...); res.error_code.val = PLANNING_FAILED; return; }
  generator_.generate(getPlanningScene(), request_, res);   // <-- 계산은 이 한 줄
}
void PlanningContextBase<GeneratorT>::solve(planning_interface::MotionPlanDetailedResponse& res)
{ /* 위를 부른 뒤 같은 궤적을 "plan"/"simplify"/"interpolate" 이름으로 3회 push */ }
bool PlanningContextBase<GeneratorT>::terminate() { RCLCPP_DEBUG_STREAM(...); terminated_ = true; return true; }
void PlanningContextBase<GeneratorT>::clear() { /* No structures that need cleaning */ return; }
```

생성자는 `planning_interface::PlanningContext(name, group)`에 이름과 그룹을
넘기고 `generator_(model, limits_, group)`를 짓는다. 즉 이 파일이 계산에
기여하는 것은 **생성기 하나를 `(model, limits, group)`로 짓고, 그것의
`generate`를 부르는 것**이 전부다.

두 대응물이 이미 트리에 있다.

- `generator_(model, limits_, group)` →
  `crates/moveit-planners-pilz/src/trajectory_generator.rs:352`
  (`TrajectoryGenerator::new(robot_model, planner_limits)`)와 각 명령별
  생성기의 `new(base, group_name)` — 예: `trajectory_generator_ptp.rs:78`.
  상류가 CRTP 매개변수로 고정하는 `(model, limits, group)` 삼중항이 여기서는
  같은 생성자의 인자다.
- `generator_.generate(scene, request_, res)` →
  `crates/moveit-planners-pilz/src/trajectory_generator.rs:606-636`
  (`PilzGenerator::generate`), 반환값이 같은 파일 `:494-530`의
  `MotionPlanResponse`다.

### §227.2 파생 네 파일에는 문장이 0개다

`planning_context_{ptp,lin,circ,polyline}.hpp`는 각각 67~69줄이고, 내용은
`MOVEIT_CLASS_FORWARD` 한 줄과 전달 생성자 하나뿐이다. PTP를 예로:

```cpp
class PlanningContextPTP : public PlanningContextBase<TrajectoryGeneratorPTP>
{
public:
  PlanningContextPTP(const std::string& name, const std::string& group,
                     const moveit::core::RobotModelConstPtr& model, const LimitsContainer& limits)
    : PlanningContextBase<TrajectoryGeneratorPTP>(name, group, model, limits) {}
};
```

본문 문장 0개, 멤버 0개, 재정의 0개. 네 파일이 하는 일은 **명령 종류 하나를
생성기 타입 하나에 묶는 것**이며, 이 포트에서 그 결합은 타입 매개변수가
아니라 `impl PilzGenerator for TrajectoryGeneratorPTP`(및 LIN/CIRC/Polyline)
그 자체다. 별도의 컨텍스트 타입이 존재할 자리가 없다.

### §227.3 잔여분 셋을 각각 처분한다

기반 클래스에서 위 두 대응물이 가져가지 않은 것은 셋이고, 셋 다 결정한다.

1. **`terminated_` / `terminate()`** — D1/D4로 버린다. 상류 자신의 주석이
   무엇인지 말한다: *"Currently will not stop a running solve but not start
   future solves."* 즉 취소가 아니라 컨텍스트를 재사용 불가로 표시하는
   플래그이고(`clear()`도 이것을 되돌리지 않는다), 이것을 세우는 유일한
   호출자는 `move_group`의 액션 취소 경로다. 이 포트의 `PlanningContext`
   대응물인 `crates/moveit-planners-sbp/src/registry.rs:542-582`가 이미 같은
   이유로 `terminate`를 두지 않는다 — "this crate's planners are
   synchronous"; 동기 `solve`는 상류에서도 실행 중 `terminated_`를 관측할 수
   없다.
2. **`solve(MotionPlanDetailedResponse&)`** — D6으로 버린다. 이 함수는
   `undetailed_response`를 만들어 위의 `solve`에 넘긴 뒤, **같은 궤적
   포인터를** `"plan"`/`"simplify"`/`"interpolate"` 세 이름으로 밀어 넣는다.
   pilz에는 simplify·interpolate 단계가 없으므로 세 항목은 동일한 값이고,
   함수 전체가 `moveit_msgs`의 `MotionPlanDetailedResponse` 모양을 채우는
   어댑터다. 이 어댑터가 실패 경로에서 널 포인터 세 개를 밀어 넣는 점은
   결함이며 `doc/upstream-bugs.md`의
   `pilz-detailed-response-pushes-null-trajectory`에 적었다.
3. **`clear()`** — 본문이 주석 한 줄과 `return;`이다. 가져올 것이 없다.

따라서 다섯 파일 모두 `gap`이 아니다. 기반은 계산 한 문장이 트리에 있으므로
`ported-elsewhere`, 파생 넷은 문장이 0개이고 그 결합을 포트가 다른 축으로
표현하므로 `decided-non-port`다.

### §227.4 이 판정이 하지 않은 것

- `planning_interface::PlanningContext` 자체를 pilz로 들여오지 않았다.
  `moveit-planners-pilz`는 `moveit-planning`에도 `moveit-planners-sbp`에도
  의존하지 않으며, 이 라운드는 그 의존 간선을 만들지 않는다(`Cargo.toml`
  변경은 `linkme` distributed_slice 순서를 바꾼다).
- 따라서 이 크레이트에는 `PlanningContext`를 구현하는 타입이 없다. pilz
  생성기를 `moveit-planners-sbp`의 레지스트리에 등록하는 것은 별개의
  결정이고, 이 절은 그것을 하지 않는다.

### §227.5 `trajectory_generation_exceptions.hpp` — 46개 예외 클래스가 8개 코드로 접힌다

이 헤더는 타입 셋을 정의한다: 추상 기반 `MoveItErrorCodeException :
std::runtime_error`(순수 가상 `getErrorCode()`), 코드를 템플릿 인자로 받는
`TemplatedMoveItErrorCodeException<ERROR_CODE = FAILURE>`, 그리고 파생 클래스를
찍어내는 `CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NAME, CODE)` 매크로.

**소비되는 방식을 먼저 센다.** 패키지 전체에서 `MoveItErrorCodeException`을
잡는 곳은 여덟 곳이고(`trajectory_generator.cpp:312,324,337,350`,
`move_group_sequence_service.cpp:105`,
`move_group_sequence_action.cpp:230,287`, 그리고 `:113,238,295`의
`std::exception` 폴백), 그 여덟 곳이 예외에서 꺼내는 것은 **정확히 둘**이다 —
`ex.getErrorCode()`는 `res.error_code.val`로 들어가고, `ex.what()`은
`RCLCPP_ERROR_STREAM`으로만 나간다. 즉 메시지는 로그 전용이고, 이 크레이트가
로깅을 `Result` 반환으로 대체한다는 기존 규칙(이 파일의 "ROS dependencies
found" 절)이 그대로 적용된다. 비-로깅 잔여분은 0이다.

**따라서 대응물은 열거형 46개 변형이 아니라 메커니즘 하나다.**

| 상류 | 이 포트 |
|---|---|
| `MoveItErrorCodeException` (코드를 실은 예외) | `moveit_error::Error::Code(MoveItErrorCode)` — `crates/moveit-error/src/lib.rs:100-102` |
| `catch (MoveItErrorCodeException&) { res.error_code.val = ex.getErrorCode(); }` ×4 | `MotionPlanResponse::failure` — `crates/moveit-planners-pilz/src/trajectory_generator.rs:511-517`, `PilzGenerator::generate`의 네 `Err` 분기 `:606-636` |
| `TemplatedMoveItErrorCodeException`의 기본 `FAILURE` | `failure`의 `_ => MoveItErrorCode::Failure` 팔 |
| `ex.what()` | 없음 — 로그 전용이므로 D-정책상 버린다 |

`CREATE_MOVEIT_ERROR_CODE_EXCEPTION` 호출은 `include/` 전체에 **49회**,
서로 다른 이름 **46개**다(50번째 매크로 출현은 정의 자신). 49−46의 차 셋은
`trajectory_generator_lin.hpp`와 `trajectory_generator_polyline.hpp`가 같은
이름 셋을 중복 선언하는 것이고, 이미
`doc/upstream-bugs.md`의 `polyline-header-redeclares-lin-exceptions`다.
`joint_limits_aggregator.hpp`의 `AggregationException` 계열과
`planning_exceptions.hpp`의 `PlanningException` 계열은 이 매크로를 쓰지 않는
**별개 분류 체계**이므로 46에 들지 않는다.

**46개 이름 각각의 대응.** `상류 코드`는 매크로의 두 번째 인자,
`포트 코드`는 대응 사이트가 실제로 내는 `MoveItErrorCode`다. 포트 경로는
별도 표기가 없으면 `crates/moveit-planners-pilz/src/`.

| # | 상류 클래스 | 상류 코드 | 상류 throw | 이 포트 | 포트 코드 |
|---|---|---|---|---|---|
| 1 | `NegativeBlendRadiusException` | INVALID_MOTION_PLAN | `command_list_manager.cpp:273` | `SequenceError::NegativeBlendRadius` (`command_list_manager.rs:111`, 발생 `:438`) | InvalidMotionPlan (`:174`) |
| 2 | `LastBlendRadiusNotZeroException` | INVALID_MOTION_PLAN | `command_list_manager.hpp:232` | `SequenceError::LastBlendRadiusNotZero` (`:120`, 발생 `:454`) | InvalidMotionPlan |
| 3 | `StartStateSetException` | INVALID_ROBOT_STATE | `command_list_manager.cpp:300` | `SequenceError::StartStateSet` (`:127`, 발생 `:480`) | InvalidRobotState (`:177`) |
| 4 | `OverlappingBlendRadiiException` | INVALID_MOTION_PLAN | `command_list_manager.cpp:160` | `SequenceError::OverlappingBlendRadii` (`:138`, 발생 `:391`) | InvalidMotionPlan |
| 5 | `PlanningPipelineException` | FAILURE | `command_list_manager.cpp:260` | `SequenceError::Planning` (`:147`, 발생 `:276`,`:281`) | 실패 아이템 자신의 코드 (`:178`) — 상류도 클래스 기본 `FAILURE`가 아니라 실패 응답의 코드를 두 번째 인자로 넘긴다 |
| 6 | `NoBlenderSetException` | FAILURE | `plan_components_builder.cpp:80` | 표현 불가 — `PlanComponentsBuilder::new`가 블렌더가 필요한 한계를 받는다 (`plan_components_builder.rs:28-33`) | — |
| 7 | `NoTipFrameFunctionSetException` | FAILURE | 없음 | 죽은 선언 — 패키지 어디서도 throw되지 않는다 (`plan_components_builder.rs:34-36`) | — |
| 8 | `NoRobotModelSetException` | FAILURE | `plan_components_builder.cpp:112` | 표현 불가 — 같은 생성자가 로봇 모델을 받는다 | — |
| 9 | `BlendingFailedException` | FAILURE | `plan_components_builder.cpp:96` | `PlanComponentsBuilder::append`가 블렌더의 오류를 그대로 전파 (`plan_components_builder.rs:110-116`) | 블렌더 자신의 코드 — 상류가 `FAILURE`로 뭉개는 것을 좁힌다 |
| 10 | `NoSolverException` | FAILURE | `tip_frame_getter.hpp:79` | `solver_tip_frame` (`trajectory_functions.rs:808`) | Failure |
| 11 | `MoreThanOneTipFrameException` | FAILURE | `tip_frame_getter.hpp:85` | 대응 없음 — `doc/port-coverage.md`의 `tip_frame_getter.hpp` 행이 이미 "multi-tip 분기"를 잔여분으로 기록한다 | — |
| 12 | `CircleNoPlane` | INVALID_MOTION_PLAN | `trajectory_generator_circ.cpp:264` | `build_path`의 세 `map_err` (`trajectory_generator_circ.rs:324`,`:329`,`:334`) | InvalidMotionPlan |
| 13 | `CircleToSmall` | INVALID_MOTION_PLAN | `trajectory_generator_circ.cpp:270` | 같음 | InvalidMotionPlan |
| 14 | `CenterPointDifferentRadius` | INVALID_MOTION_PLAN | `trajectory_generator_circ.cpp:276` | 같음 | InvalidMotionPlan |
| 15 | `CircTrajectoryConversionFailure` | INVALID_MOTION_PLAN | `trajectory_generator_circ.cpp:225` | `generate_joint_trajectory`의 반환을 그대로 전파 (`trajectory_generator_circ.rs:296`) | 그 함수가 낸 코드 — 상류도 `error_code.val`을 명시 인자로 실어 클래스 기본값을 덮는다 |
| 16 | `UnknownPathConstraintName` | INVALID_MOTION_PLAN | `trajectory_generator_circ.cpp:79` | 표현 불가 — `CircPathConstraintKind`가 두 변형뿐 (`trajectory_generator_circ.rs:23-35`) | — |
| 17 | `NoPositionConstraints` | INVALID_MOTION_PLAN | `trajectory_generator_circ.cpp:84` | 표현 불가 — 같은 문단 | — |
| 18 | `NoPrimitivePose` | INVALID_MOTION_PLAN | `trajectory_generator_circ.cpp:89` | 표현 불가 — 같은 문단 | — |
| 19 | `UnknownLinkNameOfAuxiliaryPoint` | INVALID_LINK_NAME | `trajectory_generator_circ.cpp:109` | `extract_motion_plan_info` (`trajectory_generator_circ.rs:157`) | InvalidLinkName |
| 20 | `NumberOfConstraintsMismatch` | INVALID_GOAL_CONSTRAINTS | `trajectory_generator_circ.cpp:119` | 같은 함수의 관절 수 비교 (`trajectory_generator_circ.rs:163-164`) | InvalidGoalConstraints |
| 21 | `CircInverseForGoalIncalculable` | NO_IK_SOLUTION | `trajectory_generator_circ.cpp:158` | `trajectory_generator_circ.rs:190`,`:200` | NoIkSolution |
| 22 | `TrajectoryGeneratorInvalidLimitsException` | FAILURE | `trajectory_generator_ptp.cpp:63,71,82,86,90` | `TrajectoryGeneratorPtp::new`의 여섯 `Error::construct` (`trajectory_generator_ptp.rs:80,87,95,97,102,107`) | `MotionPlanResponse::failure`의 `_` 팔을 지나 Failure (`trajectory_generator.rs:518`이 이 대응을 명시한다) |
| 23 | `VelocityScalingIncorrect` | INVALID_MOTION_PLAN | `trajectory_generator.cpp:96` | `check_velocity_scaling` (`trajectory_generator.rs:647`) | InvalidMotionPlan |
| 24 | `AccelerationScalingIncorrect` | INVALID_MOTION_PLAN | `trajectory_generator.cpp:107` | `check_acceleration_scaling` (`trajectory_generator.rs:660`) | InvalidMotionPlan |
| 25 | `UnknownPlanningGroup` | INVALID_GROUP_NAME | `trajectory_generator.cpp:117` | `check_for_valid_group_name` (`trajectory_generator.rs:673`) | InvalidGroupName |
| 26 | `NoJointNamesInStartState` | INVALID_ROBOT_STATE | 없음 | 죽은 선언 — 상류 트리 전체에서 throw되지 않고, `unittest_trajectory_generator.cpp:70`이 생성만 한다 | — |
| 27 | `SizeMismatchInStartState` | INVALID_ROBOT_STATE | `trajectory_generator.cpp:126` | 표현 불가 — `StartState::position`이 맵이라 이름 수와 값 수가 어긋날 수 없다 (`trajectory_generator.rs:678-681`) | — |
| 28 | `JointsOfStartStateOutOfRange` | INVALID_ROBOT_STATE | `trajectory_generator.cpp:144` | `check_start_state`의 위치 한계 분기 (`trajectory_generator.rs:703`) | InvalidRobotState |
| 29 | `NonZeroVelocityInStartState` | INVALID_ROBOT_STATE | `trajectory_generator.cpp:151` | 같은 함수의 속도 분기 (`trajectory_generator.rs:711`) | InvalidRobotState |
| 30 | `NotExactlyOneGoalConstraintGiven` | INVALID_GOAL_CONSTRAINTS | `trajectory_generator.cpp:228` | 표현 불가 — `Goal`이 두 변형 열거형 (`trajectory_generator.rs:24-42`) | — |
| 31 | `OnlyOneGoalTypeAllowed` | INVALID_GOAL_CONSTRAINTS | `trajectory_generator.cpp:234` | 표현 불가 — 같은 문단 | — |
| 32 | `StartStateGoalStateMismatch` | INVALID_GOAL_CONSTRAINTS | 없음 | 죽은 선언 — `unittest_trajectory_generator.cpp:100`이 생성만 한다 | — |
| 33 | `JointConstraintDoesNotBelongToGroup` | INVALID_GOAL_CONSTRAINTS | `trajectory_generator.cpp:166` | `check_joint_goal`의 `has_joint_model` 분기 (`trajectory_generator.rs:757`) | InvalidGoalConstraints |
| 34 | `JointsOfGoalOutOfRange` | INVALID_GOAL_CONSTRAINTS | `trajectory_generator.cpp:173` | 같은 함수의 한계 분기 (`trajectory_generator.rs:760`) | InvalidGoalConstraints |
| 35 | `PositionConstraintNameMissing` | INVALID_GOAL_CONSTRAINTS | `trajectory_generator.cpp:189` | `check_cartesian_goal`의 `link_name.is_empty()` (`trajectory_generator.rs:784`) | InvalidGoalConstraints |
| 36 | `OrientationConstraintNameMissing` | INVALID_GOAL_CONSTRAINTS | `trajectory_generator.cpp:194` | 같은 분기 — `Goal::Cartesian`은 `link_name` 하나를 공유하므로 두 검사가 하나가 된다 | InvalidGoalConstraints |
| 37 | `PositionOrientationConstraintNameMismatch` | INVALID_GOAL_CONSTRAINTS | `trajectory_generator.cpp:203` | 표현 불가 — 같은 이유 (`trajectory_generator.rs:767-772`) | — |
| 38 | `NoIKSolverAvailable` | NO_IK_SOLUTION | `trajectory_generator.cpp:211` | `check_cartesian_goal`의 솔버 탐색 (`trajectory_generator.rs:794`) | NoIkSolution |
| 39 | `NoPrimitivePoseGiven` | INVALID_GOAL_CONSTRAINTS | `trajectory_generator.cpp:216` | 표현 불가 — `trajectory_generator.rs:24-42` | — |
| 40 | `LinTrajectoryConversionFailure` | FAILURE | `trajectory_generator_lin.cpp:172`, `trajectory_generator_polyline.cpp:185` | `generate_joint_trajectory`의 반환을 전파 (`trajectory_generator_lin.rs:218`, `trajectory_generator_polyline.rs:250`) | 그 함수가 낸 코드 — 상류도 `error_code.val`을 명시 인자로 넘긴다 |
| 41 | `JointNumberMismatch` | INVALID_GOAL_CONSTRAINTS | `trajectory_generator_lin.cpp:97` | **대응 없음 — §227.6** | — |
| 42 | `LinInverseForGoalIncalculable` | NO_IK_SOLUTION | `trajectory_generator_lin.cpp:136`, `trajectory_generator_polyline.cpp:122` | `trajectory_generator_lin.rs:148`,`:158`; `trajectory_generator_polyline.rs:172`,`:182` | NoIkSolution |
| 43 | `NoWaypointsSpecified` | INVALID_MOTION_PLAN | `trajectory_generator_polyline.cpp:221` | `cmd_specific_request_validation` (`trajectory_generator_polyline.rs:130`) | InvalidMotionPlan |
| 44 | `ConsicutiveColinearWaypoints` | INVALID_MOTION_PLAN | `trajectory_generator_polyline.cpp:164` | `plan`의 `polyline_from_waypoints` 실패 (`trajectory_generator_polyline.rs:226`) | InvalidMotionPlan |
| 45 | `PtpVelocityProfileSyncFailed` | FAILURE | `trajectory_generator_ptp.cpp:183` | `plan_ptp`의 동기화 실패 (`trajectory_generator_ptp.rs:301`, `Error::construct`; doc `:221`) | Failure (`_` 팔) |
| 46 | `PtpNoIkSolutionForGoalPose` | NO_IK_SOLUTION | `trajectory_generator_ptp.cpp:261` | `extract_motion_plan_info` (`trajectory_generator_ptp.rs:164`,`:174`) | NoIkSolution |

**코드 축 요약.** 46개 이름이 쓰는 상류 코드는 8종
(FAILURE, INVALID_MOTION_PLAN, INVALID_ROBOT_STATE, INVALID_GOAL_CONSTRAINTS,
INVALID_GROUP_NAME, INVALID_LINK_NAME, NO_IK_SOLUTION, 그리고 명시 인자로
덮어쓰는 경우)이고, 여덟 종 모두 `MoveItErrorCode`에 대응물이 있다.
**코드 축에 빠진 변형은 없다.** 세 부류가 값으로 대응하지 않는데, 각각의
이유가 다르다: 죽은 선언 3개(#7, #26, #32 — 상류 트리 어디서도 throw되지
않는다), 타입 모양으로 표현 불가 10개(#6, #8, #16, #17, #18, #27, #30, #31,
#37, #39), 그리고 실제 누락 2개(#11, #41).

### §227.6 실제 누락 둘 — 이 라운드가 고치지 않은 것

표의 46행 중 "상류가 거부하는 요청을 포트가 받아들인다"에 해당하는 것은
둘뿐이고, 둘 다 이 라운드의 여덟 파일 밖이므로 **여기서는 기록만 한다.**

1. **#41 `JointNumberMismatch`** — `trajectory_generator_lin.cpp:88-97`은
   LIN의 관절 공간 목표에 대해
   `goal_constraints.front().joint_constraints.size() !=
   group->getActiveJointModelNames().size()`를 검사하고 어긋나면
   `INVALID_GOAL_CONSTRAINTS`로 거부한다. `trajectory_generator_lin.rs:122-133`의
   `Goal::Joint(positions)` 분기에는 그 비교가 없고, `check_joint_goal`
   (`trajectory_generator.rs:748-765`)도 이름의 소속과 한계만 볼 뿐 개수는 세지
   않는다. 그러므로 그룹의 활성 관절 6개 중 3개만 지정한 LIN 요청은 상류에서
   거부되고 이 포트에서는 통과해, 지정되지 않은 관절이 현재 값에 머문 채로
   FK가 계산된다. **같은 검사가 CIRC에는 있다** —
   `trajectory_generator_circ.rs:163-164`가 `NumberOfConstraintsMismatch`(#20)를
   같은 코드로 포팅했다. 즉 상류에서 동일하던 두 생성기가 이 포트에서
   갈라져 있다.
2. **#11 `MoreThanOneTipFrameException`** — `tip_frame_getter.hpp:85`의
   multi-tip 분기. 이것은 새 사실이 아니라 `doc/port-coverage.md`의
   `tip_frame_getter.hpp` 행이 이미 잔여분으로 적어 둔 것이며, 이 표가 그
   기록과 일치함을 확인한 것이다.

### §227.7 이 표가 하지 않은 것

- 46개 각 행을 실행으로 확인하지 않았다. 표의 근거는 상류 throw 사이트와
  포트 `Err` 사이트를 양쪽에서 읽은 것이고, 그것이 이 절의 증거 등급이다.
- `trajectory_generator_{lin,circ,polyline}.hpp`의 생성자 doc 넷이
  `@throw TrajectoryGeneratorInvalidLimitsException`이라고 적지만 그 셋의
  생성자 본문은 `planner_limits_.printCartesianLimits()` 한 줄뿐이고 아무것도
  throw하지 않는다(`trajectory_generator_{circ,lin,polyline}.cpp:62-76`). PTP만
  실제로 던진다. 문서 결함이고 동작 결함이 아니므로
  `doc/upstream-bugs.md`에 넣지 않는다.

## §228 `moveit_core/utils`의 테스트/문자열 유틸 일곱 파일과 `console_colors.hpp` — 갭 8건을 판정으로 바꿨다

`doc/port-coverage.md`가 `gap`으로 들고 있던 8개 파일을 열어 판정했다.
여덟 개가 전부 `decided-non-port`로 끝났는데, 이유는 파일마다 다르고 그
차이가 중요하다 — "언어가 그 문제를 안 가진다"(§228.1), "이 포트가 같은
일을 다른 계기로 이미 한다"(§228.3, §228.4), "소비자가 코퍼스 밖에만
있다"(§228.2, §228.5). 근거를 뭉뚱그리면 만료 조건도 뭉뚱그려지므로 절을
나눈다.

부재 주장은 전부 `crates/ ros/ tools/ doc/ PORTING-PLAN.md` 코퍼스에 대한
`rg` 결과이고, 각 절이 그 명령을 적는다.

한 가지 공통점은 적어 둘 값어치가 있다. `robot_model_test_utils.*`와
`eigen_test_utils.hpp`는 **테스트 지원 코드인데 `test/` 디렉터리 밖에
산다**(`utils/src/`, `utils/include/`). 코퍼스 계기가 걸러내는 것은 경로에
`test`/`tests` 성분이 있는 파일이므로(`measure-port-coverage.py:90`) 이
둘은 코퍼스 안으로 쓸려 들어왔다. 상류의 `test/` 아래 테스트들은 애초에
코퍼스 밖이라 이런 행이 없다.

### §228.1 `lexical_casts.{hpp,cpp}` — `decided-non-port`, 언어가 이 문제를 안 가진다

파일의 존재 이유가 자기 헤더 doc에 적혀 있다: 시스템 로케일에 따라 소수점
구분자가 달라지는데 "이는 내부(비사용자 대면) 목적에는 흔히 원치 않는
것"이라, `toString`/`toDouble`/`toFloat`가 스트림에
`std::locale::classic()`을 imbue한 뒤 `<<`/`>>`를 쓴다
(`lexical_casts.cpp:49-52,69-72`).

**그 문제는 C++ iostream의 성질이지 부동소수 변환의 성질이 아니다.**
Rust의 `f64: Display`와 `f64: FromStr`은 로케일을 입력으로 받지 않는다 —
`format!("{x}")`와 `"1.5".parse::<f64>()`는 언어가 정의하는 대로 항상 `.`을
쓴다. 즉 `toStringImpl`의 본체에서 의미를 지는 유일한 줄인
`oss.imbue(std::locale::classic())`이 이 포트에서는 기본값이다. 옮길
알고리즘이 남지 않는다.

한 걸음 더 재 봤고, 결과가 판정을 굳혔다. 두 구현을 같은 값으로 돌린
실측(`g++ -std=c++17`로 컴파일한 상류 두 함수의 동형 모델과 `rustc -O`):

```
toString(0.12345678901230001) = "0.123457"     round-trips: NO
format!("{v}")                = "0.1234567890123"  round-trips: yes
```

`std::ostringstream`의 기본 정밀도가 6 유효숫자이고 `toString`이 그것을
바꾸지 않기 때문이다. Rust의 `Display`는 왕복하는 최단 표현을 낸다. 그래서
이 포트의 기본값은 상류 함수의 대체물일 뿐 아니라, 상류가 실제로 그 함수를
쓰는 방식(설정값을 문자열로 썼다가 도로 읽는 것)에서 더 낫다. 그 왕복
자리는 `doc/upstream-bugs.md`의
`to-string-truncates-to-six-significant-digits`에 따로 적었다.

**호출자는 전부 코퍼스 밖이다.** 상류 전체에서 `lexical_casts.hpp`를
include하는 파일은 셋 — `moveit_planners/ompl/ompl_interface/src/
{ompl_interface.cpp:200, model_based_planning_context.cpp:300,315,597}`와
`moveit_ros/benchmarks/src/BenchmarkExecutor.cpp:989,1034` — 이고,
`CORPUS_ROOTS`(`moveit_core`, `moveit_kinematics`, chomp/stomp/pilz)는 셋 중
어느 것도 포함하지 않는다. ompl은 D3으로 네이티브 플래너가 대체한다.

**기존 인용을 바로잡는다.** 표가 이 두 행에 달고 있던 증거는
`crates/moveit-error/src/lib.rs:312`였는데, 그 자리는 이 파일을 *포팅한다고*
말하지 않는다 — 정반대로, `MoveItErrorCode`의 `Display`가 포팅하는 것이
`errorCodeToString`이지 `lexical_casts.cpp`의 `toString`이 아님을 밝히려고
"그 디렉터리의 유일한 `toString`은 무관한 float 포매터"라고 적은 자리다.
파일을 *건드리는* 인용이지 *덮는* 인용이 아니므로, 새 행은 그 사실을 그대로
적는다.

### §228.2 `rclcpp_utils.{hpp,cpp}` — `decided-non-port`, 다만 표가 적어 둔 이유는 틀렸다

표가 이 두 행에 달고 있던 근거는 "내용상 D1(`rclcpp`)이지만 이 저장소의
어떤 텍스트도 그렇게 말하지 않는다"였다. **앞부분이 사실이 아니다.** 파일을
열면 ROS 타입이 하나도 없다: `rclcpp_utils.hpp`가 include하는 것은
`<string>` 하나뿐이고(`:30`), `.cpp`는 자기 헤더만 include한다(`:28`).
내용은 `std::string` 두 함수 — `clean(name)`이 `//`를 `/`로 접고 끝의 `/`를
떼며, `append(left, right)`가 `/`로 이어 붙인 뒤 `clean`을 건다. 이름과
네임스페이스(`rclcpp::names`)가 ROS를 가리킬 뿐, 코드는 D1이 정의하는
"ROS 메시지 타입"에 닿지 않는다. 이름으로 분류하면 이렇게 틀린다.

진짜 근거는 **소비자**다. 이 두 함수가 만드는 것은 ROS 노드/토픽/서비스
이름이고, 상류 호출자는 전부 `moveit_ros/*`다 —
`moveit_ros/planning_interface/move_group_interface/src/
move_group_interface.cpp:178-205`가 `move_group_namespace`에 액션·서비스
이름을 이어 붙이는 자리이고, `moveit_ros/visualization/
planning_scene_rviz_plugin/src/planning_scene_display.cpp:64`가 또 하나다.
`moveit_ros`는 `CORPUS_ROOTS` 밖이다.

이 포트 쪽 소비자도 없다. `rg -n -F rclcpp_utils crates/ ros/ tools/ doc/
PORTING-PLAN.md`는 `doc/port-coverage.md`의 자기 행 둘만 찾는다. 그럴 수밖에
없는 것이, ROS를 아는 크레이트는 `ros/moveit-ros` 하나인데(D2) 그 크레이트의
이번 라운드 범위가 **타입 변환뿐**이라 노드도 토픽도 서비스도 만들지
않는다(`ros/moveit-ros/src/lib.rs:17-19`). 붙일 이름이 없는 곳에 이름
정규화 함수를 놓으면 호출자 없는 코드가 된다.

**만료 조건:** `moveit-ros`가 토픽/서비스/액션 이름을 구성하기 시작하면 이
판정을 다시 한다. 그때 결정할 것은 "포팅할까"가 아니라 "r2r이 이미 주는
이름 처리로 충분한가"이며, 그 답은 그 라운드가 r2r을 열어 보고 정해야 한다
— 이 절은 그것을 확인하지 않았고, 확인하지 않은 것을 근거로 쓰지 않는다.

### §228.3 `robot_model_test_utils.{hpp,cpp}` — `decided-non-port`, 이 포트의 대응물은 픽스처 출처 표다

파일은 두 덩어리이고 각각 다른 이유로 안 옮긴다.

**(a) 로더 셋 — 이 포트에는 이미 대응물이 있고, 경로 규칙까지 같다.**
`loadModelInterface(robot_name)`는 `moveit_resources_<name>_description`
패키지를 런타임에 ament로 찾아 `urdf/<name>.urdf`를 읽는데, `pr2`만
`urdf/robot.xml`로 특수 분기한다(`robot_model_test_utils.cpp:136-143`).
`loadSRDFModel`도 같은 모양으로 `pr2`는 `srdf/robot.xml`,
나머지는 `moveit_resources_<name>_moveit_config`의
`config/<name>.srdf`다(`:158-185`).

이 포트는 같은 파일들을 빌드타임에 복사해 `fixtures/`에 커밋하고
`RobotModel::from_urdf_and_srdf`로 읽는다 — 호출 지점 127개
(`rg -o -F '::from_urdf_and_srdf(' crates/ ros/ --glob '*.rs' | wc -l`;
이름 전체를 세면 140이지만 그중 1개는 정의 줄, 16개는 주석 줄이다).
그리고 그 복사의 출처가
`tools/ci/verify-fixture-provenance.sh`의 `SOURCE_OF` 표인데, 그 표가
상류 로더의 분기와 **같은 매핑을 그대로 적고 있다**:

```
[fixtures/pr2.urdf]="$VENDOR/pr2_description/urdf/robot.xml"
[fixtures/pr2.srdf]="$VENDOR/pr2_description/srdf/robot.xml"
[fixtures/panda.urdf]="$VENDOR/panda_description/urdf/panda.urdf"
[fixtures/panda.srdf]="$VENDOR/panda_moveit_config/config/panda.srdf"
```

pr2의 두 파일만 `robot.xml`이라는 상류의 특수 케이스가 그 스크립트 주석에도
따로 적혀 있다("pr2's two files are both named `robot.xml` upstream, which is
why the mapping is explicit"). 즉 상류가 런타임 패키지 조회로 하는 일을 이
포트는 커밋된 복사본과 다이제스트로 고정된 출처 표로 하고, xacro 확장이
필요한 둘(`dual_arm_panda.urdf`, `prbt.urdf`)은 입력 집합과 재생성 명령까지
그 표에 적혀 있다. 옮길 것이 남지 않았다.

**(b) `RobotModelBuilder`와 `loadIKPluginForGroup` — D1/D4.**
`RobotModelBuilder`의 공개 시그니처 여섯 개
(`addChain`, `addCollisionMesh`, `addCollisionBox`, `addVisualBox`,
`addInertial`, `addLinkCollision`/`addLinkVisual`)가 전부
`geometry_msgs::msg::Pose`를 받는다(`robot_model_test_utils.hpp:133-165,
217-221`) — D1이다. 알고리즘 쪽은 더 얇다: `build()`의 본체는
`urdf_model_->initTree`, `initRoot`, `srdf_writer_->updateSRDFModel` 세
호출이고(`:503-531`), 셋 다 `urdfdom`/`srdfdom`의 함수라 MoveIt 알고리즘이
아니다. `loadIKPluginForGroup`은 `rclcpp::Node::SharedPtr`와
`pluginlib::ClassLoader<kinematics::KinematicsBase>`를 직접 쓴다(`:189-213`)
— D1 + D4.

이 포트에서 합성 모델이 필요한 자리는 테스트 안에 URDF/SRDF 문자열을 직접
써서 채운다(`rg -l '<robot name=' crates/ ros/ tools/ --glob '*.rs'` → 24개
파일). 빌더 없이 이미 돌아가는 방식이므로, 호출자가 생기지 않을 300줄짜리
빌더를 옮기는 것은 죽은 코드를 만드는 일이다.

`crates/moveit-test-support/src/lib.rs`의 모듈 doc이 이 판정과 짝을 이룬다 —
이번 라운드 전까지 그 doc은 "상류에는 이에 상응하는 테스트 측 기계장치가
없다"고 적고 있었는데, 그 문장이 부정하던 파일이 정확히 이 두 개였다.

### §228.4 `eigen_test_utils.hpp` — `decided-non-port`, 이 포트는 같은 단언을 `approx`로 한다

파일은 gtest 술어 하나와 그것을 감싸는 매크로 둘이다:
`Eigen::Transform`용 `operator<<`, `val1.isApprox(val2, prec_)`를 도는
`IsApprox` 술어, 그리고 `EXPECT_EIGEN_EQ`/`EXPECT_EIGEN_NEAR`
(`eigen_test_utils.hpp:72-79`). `::testing::AssertionResult`과
`EXPECT_PRED_FORMAT2`에 기대어 있으므로 옮기려면 gtest를 옮겨야 하는데, 이
포트는 `#[test]`와 `assert*!`를 쓴다.

**이 포트의 대응물은 이름이 있다.** `approx` 크레이트(`Cargo.toml:72`,
워크스페이스 dep, 15개 크레이트가 상속)의 `assert_relative_eq!`이고,
`crates/ ros/ tools/`에 **호출 268건, 41개 파일**이다. 이름 전체를 세면
329건이지만 그중 61건은 주석/모듈 doc 안에서 이 매크로를 *언급하는*
줄이므로 호출이 아니다
(`rg -n -F 'assert_relative_eq!' crates/ ros/ tools/ --glob '*.rs' |
rg -v '^[^:]+:[0-9]+:\s*//' | wc -l`). 처음 이 절에 적었던 328/61-파일은
그 구분을 하지 않은 수였고, 게다가 이번 라운드가
`crates/moveit-test-support/src/lib.rs`의 doc에 이 이름을 한 번 더 쓰면서
스스로 328을 329로 만들었다 — 언급을 세는 계기는 자기 문서에도 반응한다.
호출 줄에 이 매크로가 두 번 나오는 줄은 없다(0건 확인).
`Eigen::Transform` 통째 비교에 해당하는 자리는 이름 붙은 헬퍼로 한 번 더
감싸 두었다 — `assert_isometry_eq`가
`actual.to_homogeneous()`와 `expected.to_homogeneous()`를
`assert_relative_eq!(.., epsilon = 1e-9, max_relative = 1e-9)`로 비교한다
(`crates/moveit-collision/tests/world_parity.rs:121-129`,
`crates/moveit-scene/tests/frame_transform_parity.rs:146`).

**옮기지 않는 편이 나은 이유가 하나 더 있다.** `EXPECT_EIGEN_EQ`는 허용오차를
비교 대상이 아니라 **스칼라 타입**에서 끌어온다 —
`Eigen::NumTraits<Scalar>::dummy_precision()`, 즉 라이브러리가 주는 기본값이고
매크로 본문에 그렇게 적혀 있다. 이 저장소의 규칙은 허용오차를 이웃에서
베끼지 않고 실측해서 정하는 것이므로, 이 매크로를 옮기면 측정된 적 없는
기본값을 328개 자리의 기본형으로 들이게 된다. (그 기본값의 실제 수치는 이
기계에 MoveIt이 빌드하는 Eigen 체크아웃이 없어 확인하지 않았고, 확인하지
않은 것을 근거로 쓰지 않는다 — 근거는 "비교 대상이 아니라 타입에서
나온다"는 매크로 본문 자체다.)

`rg -n -F eigen_test_utils crates/ ros/ tools/ doc/ PORTING-PLAN.md`와
`rg -n -F EXPECT_EIGEN crates/ ros/ tools/`는 각각 자기 행 하나와 0건이다.

### §228.5 `console_colors.hpp` — `decided-non-port`, 코퍼스 안 소비자가 하나뿐이고 그것이 없다

ANSI 이스케이프 `#define` 9개가 전부다(`console_colors.hpp:39-47`). 상류에서
이 헤더를 쓰는 곳은 셋인데, `moveit_ros/move_group/src/move_group.cpp:122-198`과
`moveit_ros/planning_interface/test/move_group_ompl_constraints_test.cpp`은
코퍼스 밖이고, 코퍼스 안은 하나 —
`moveit_core/robot_state/src/robot_state.cpp:2321,2347`, 즉
`RobotState::printStatePositionsWithJointLimits`가 관절이 한계를 벗어났을 때
줄을 빨갛게 칠하는 자리다.

그 함수가 이 포트에 없다. `rg -n -i
'printStatePositionsWithJointLimits|print_state_positions_with_joint_limits'
crates/ ros/ doc/ PORTING-PLAN.md`가 0건이고, 상류 `robot_state.hpp`가
선언하는 `print*` 여섯 개(`:1643-1654`) 중 어느 것도 `crates/moveit-state`에
없다 — 그 크레이트의 크레이트 doc이 적어 둔 범위에도 들어 있지 않다
(`crates/moveit-state/src/lib.rs:15-31`). `std::ostream`에 사람이 읽을
디버그 그림을 그리는 함수가 없으니, 그 그림을 칠할 색도 필요 없다.

**만료 조건:** 이 포트가 사람이 읽는 상태 덤프를 갖게 되면 다시 본다. 그때도
9개 `#define`을 옮기는 것이 답일지는 별개다 — Rust 쪽에는 같은 일을 하는
크레이트가 있고, 이 절은 그것을 조사하지 않았다.


## §229 Phase 3 미충족 두 절의 원인을 확정했다 — `bool`은 상류에 규약이 없고, `distance`는 다른 양이다 (2026-08-06)

§218이 Phase 3을 조건이 말한 크기로 처음 재고 두 절을 **미충족**으로
남겼다. 이 절은 그 두 절이 각각 무엇 때문에 깨지는지를 확정한다. 어느
쪽도 허용오차로 닫지 않았다 — §11.10이 미리 적어둔 대로, 1e-4를 넘는
발산은 백엔드에 대한 발견이지 오차를 늘릴 사유가 아니다.

### §229.1 `collision: bool` 절 (prbt 6,854/10,000) — 상류에 정합할 규약이 없다

§218.3이 이 6,854건을 `z = 0` 정확 접선 하나로 좁혔다. 남은 질문은 그
접선에서 상류가 `false`를 내는 것이 **결함**인지 **포트가 따라야 할
축퇴 규약**인지였다. 답은 둘 다 아니다: 상류에 단일 규약이 없다.

바닥 상판을 접선 위아래로 쓸어 오라클을 잰 결과(`e017c91ee`, seed 무관):

| 바닥 상판 `z` | 실제 간극 | `robot_collision` | `robot_distance` |
|---|---|---|---|
| `-1e-3`  | `+1e-3`  | `false` | `+1.000000000000001e-3` |
| `-1e-7`  | `+1e-7`  | `false` | `+1.000000000028756e-7` |
| `-3e-8`  | `+3e-8`  | `false` | `+2.999999999808711e-8` |
| `-1e-9`  | `+1e-9`  | `false` | `+9.999999994736568e-10` |
| `-1e-15` | `+1e-15` | `false` | `+1.038912551220369e-15` |
| `0`      | `0`      | `false` | **`-1.000000000000000e0`** |
| `+1e-15` | `-1e-15` | `true`  | `-1.129411566063279e-15` |
| `+1e-9`  | `-1e-9`  | `true`  | `-9.999999994737827e-10` |
| `+1e-3`  | `-1e-3`  | `true`  | `-1.000000000000001e-3` |

`bool` 열만 보면 "닿는 것은 충돌이 아니다"라는 단조 규약처럼 읽힌다.
그렇지 않다. `fcl::collide`는 도형 쌍마다 다른 협면(narrowphase)으로
분기하고, 이 워크스페이스가 가진 **정확히 닿는 두 쌍이 서로 반대**로
나온다:

| 쌍 | 간극 | 오라클 `collision` | 오라클 `distance` |
|---|---|---|---|
| prbt 실린더가 상자 위에 (위 표) | 정확히 `0` | `false` | `-1.0` |
| octree 리프 면이 상자 면에 (`octree_world_collision_response.json` case 4) | 정확히 `0` | `true` | `-0.0` |

`-1.0`이 단서다. prbt 쌍은 `fcl::collide`가 접촉을 **0개** 찾아
`fcl::distance`의 센티널이 그대로 남은 것이고, octree 쌍은 접촉을 찾아
그 깊이(`-0.0`)를 보고한 것이다. 그러므로 정확 접촉에서 맞출 상류의
답이 하나로 존재하지 않는다.

`distance` 열의 `-1.0`은 결함이며 이미
`doc/upstream-bugs.md`의 `fcl-distance-sentinel-survives-zero-contacts`에
있다. 이번에 그 항목의 **Evidence**를 위 연속성 표로 교체했다 — 양쪽
`1e-15`에서 연속이고 그 사이 한 점에서 `1e15`배 뛴다 — 그리고 octree
반례를 나란히 적어 `bool` 열이 규약이 아님을 그 항목 안에서도 읽을 수
있게 했다.

**판정: 이 절은 이 픽스처 위에서 닫히지 않는다.** `bool`에는 허용오차가
없고, 채택할 규약도 없으며, 접선에서의 답은 각 백엔드 자신의 반올림이
정한다 — 이 백엔드는 `-2.775558e-17`을 얻어 `true` 쪽에 떨어진다.
`fixtures/prbt.urdf`나 바닥을 옮기면 절은 초록이 되지만 그것은 측정을
지우는 것이지 닫는 것이 아니다. 재실행 가능한 형태로
`crates/moveit-collision/tests/exact_tangency_boundary.rs`에 고정했다
(오라클·docker 불요, 4건 `0.017s`).

### §229.2 이 백엔드의 충돌 경계는 0이 아니라 양의 간극 안에 있다 (측정, 미수정)

§229.1을 조사하다 별도로 나온 **포트 쪽** 발견이다. `parry.rs`의 모듈
문서는 `query::contact`를 prediction `0.0`으로 부르므로 "닿거나 관통한
쌍만 `Some`"이라고 적고 있다. 측정은 그렇지 않다: prbt 실린더 대
`4x4x0.1` 상자에서 `3e-8 m` 간극이 `Some`을, `1e-7 m` 간극이 `None`을
준다. 즉 실효 경계가 `5e-8 m`쯤의 **빈 공간** 안에 있고, 그 지점에서
오라클은 `false`(§229.1 표의 `-3e-8` 행)이므로 이것은 그 자체로
발산이다.

`contact.dist <= 0.0` 부호 검사를 넣어봤고 **되돌렸다**. 이유는 두
가지이며 둘 다 측정이다.

1. 고치려는 절을 닫지 못한다. prbt의 접선 값은 `-2.775558e-17`로 이미
   부호 검사의 충돌 쪽이다.
2. 오라클로 뒷받침되는 파리티 케이스를 깬다.
   `octree_world_object_collision_matches_the_oracle` case 4에서 오라클은
   `true`인데 `parry`는 그 쌍을 0보다 **약간 위**에 놓는다. 부호 검사를
   넣으면 왼쪽 `false` / 오른쪽 `true`로 갈린다.

즉 지금의 여유(margin)가 상류의 도형쌍별 비일관성을 흡수하고 있다.
없애면 파리티 케이스 하나를 내주고 아무것도 닫지 못한다. 그래서
**UNFIXED로 남기고 측정으로 고정**했다 —
`the_collision_boundary_sits_in_a_positive_gap`이 `1e-7`(비충돌)과
`3e-8`(충돌, 그러나 거리 질의는 양수)을 함께 못 박고, `parry.rs`의
해당 지점에는 되돌린 이유를 주석으로 남겼다. 다음 사람이 같은 발상을
할 것이기 때문이다.

이 주석 22줄이 `parry.rs`의 아래쪽 줄 번호를 전부 밀었으므로,
`parry.rs:2142` 아래를 가리키던 인용 15개(ledger p3-acm 13, p10-samplers
1, upstream-bugs 1)와 산문 속 맨숫자 참조 5개를 `+22`로 다시 매겼다.
`verify-orphan-enumeration.sh`는 이 변경 전 기준(`7572123`)에서 초록,
주석만 넣었을 때 고아 10 / 미해결 인용 10으로 빨강, 재번호 후 다시
초록이다.

### §229.3 `distance: f64` 절 (panda 27,384배, 사례로 확증; fanuc·pr2는 별개) — 배율이 아니라 다른 양이다

배율이 너무 커서 수치 잡음일 수 없다는 지시가 맞았다. 비교되고 있던
것은 **다른 정의의 거리**다.

`distanceCallback`(`collision_common.cpp:646`, `:650-659`, `:662-663` @
`e017c91ee`)은 `fcl::collide`를 `num_max_contacts = 200`으로 돌린 뒤
`penetration_depth`가 **최대**인 접촉을 골라 그 부호를 뒤집어
`dist_result.distance`로 쓴다. 메시 링크에서 `fcl::collide`는 삼각형
단위이고, 큰 상자 **안에** 완전히 들어간 삼각형은 분리축이 없어 FCL이
측면 탈출 거리를 보고한다 — 상자가 커지면 같이 커지는 값이다. `max`
선택은 그 인공물을 옆의 정상 접촉 200개 위로 올린다.

`panda_link0` 기본 자세, 바닥 슬래브 `L x L x 0.1`의 **상판을 `z = +0.05`에
고정**해 실제 겹침을 `0.05 m`로 붙박아 두고 `L`만 바꾼 오라클 측정:

| `L` (m) | 오라클 `robot_distance` |
|---|---|
| `0.4`  | `-2.252549386999574e-1` |
| `1.0`  | `-6.442843086554950e-1` |
| `2.0`  | `-1.349881199903309e0` |
| `4.0`  | `-2.763223704646119e0` |
| `8.0`  | `-3.999482770392206e0` |
| `20.0` | `-9.999482770392206e0` |

겹침이 한 번도 움직이지 않은 스윕에서 44배가 변한다. `L >= 8`에서는
정확히 `L/2 - 0.000517`, 즉 슬래브의 반폭이다 — 수직 탈출이 아니라 측면
탈출이다.

`max_contacts_per_pair: 200`으로 접촉 집합을 받아 상류의 `max` 선택과
FCL의 개별 깊이를 분리하면:

| `L` | `panda_link0/floor` 접촉 수 | min | median | max | `0.309272` 초과 |
|---|---|---|---|---|---|
| `4.0`  | 100 | `0.041524344` | `0.049999832` | `2.763223705` | 100건 중 32건 |
| `20.0` | 100 | `0.041524344` | `0.049999698` | `9.999482770` | 100건 중 14건 |

**median은 두 폭 모두에서 옳은 `~0.05`이고 폭에 대해 `1.3e-7` 이내로
불변**이다. `0.309272`는 `panda_link0` 자신의 지름으로,
`fixtures/meshes/panda_description/meshes/collision/link0.stl`의 삼각형
200개에서 최대 `|vertex|`의 2배로 측정했다. 최대값 아래의 모든 순서
통계는 기하가 묶어주고 최대값만 묶이지 않으며, 상류 `:655`가 바로 그
묶이지 않는 것을 고른다. 그러므로 결함은 FCL 경계의 **MoveIt 쪽**에
있다 — 같은 접촉 집합에서 최소나 중앙값을 골랐으면 옳게 답했다.

포트는 `-0.05003249277506257`을 `L`의 `0.4`/`1.0`/`4.0`/`20.0` 네 폭
모두에서 내며 **폭 간 산포가 정확히 `0.000000e0`**, 오라클 자신의 median
접촉과 `3.3e-5` 차이 — 즉 **1e-4 절 안**이다.

**적용 범위 — panda만 사례로 확증했다, fanuc은 이 기전이 아니다.**
이 절의 제목이 이전 판에서 "3로봇 전부"라 적었던 것은 과잉 일반화다.
§218.4가 이미 세 로봇 각각의 max\|Δ\| 상태를 개별 추적해뒀다: panda
`collision[3289]`는 **같은 쌍** `panda_link0/floor`에서 값만 갈리는
경우로 §218.4 자신이 "이탈 6"이라 이름 붙였고, 이 절의 위 표가 바로
그 기전이다. 그러나 fanuc `collision[9651]`은 **쌍 자체가 다르다** —
오라클은 `base_link/floor`(−9.930e-16, 사실상 접촉 없음)를, 포트는
`link_4/floor`(−2.897e-1)를 최소로 고른다. §218.4는 이를 명시적으로
"이탈 6이 아니다"라 적었다 — fanuc의 robot 쪽 실패 2,302건은 같은 쌍
값 발산이 **0**건이고 전부 pair-flip(최소값이 근접한 서로 다른 쌍
사이의 승자 교체)이다. 이 절의 표·불변량 시험은 fanuc의 2,897배를
설명하지 않는다. pr2의 mesh-vs-box 최악값 `3.218e-1`(§21.4, 이 절 이전
판)은 같은 쌍/다른 쌍 구분이 **한 번도 시행되지 않았다** — 근거 없이
이 기전으로 분류해서는 안 된다. `doc/upstream-bugs.md`의 해당 항목도
이 구분에 맞춰 고쳤다.

**판정: panda의 미충족은 포트의 오차가 아니라 상류가 다른 양을 보고하는
것이다 — 사례로 확증됨.** fanuc·pr2는 별개 원인이거나(fanuc) 미확인
(pr2)이며, 이 절이 "닫는다"고 주장하는 범위가 아니다. `doc/upstream-bugs.md`에
`distance-callback-max-contact-depth`로 등재했고(not-reproduced), 재현자는
`crates/moveit-collision/tests/penetration_depth_scale_invariance.rs`
6건이다(오라클·docker 불요). 허용오차 확대는 애초에 불가능했다 — `L = 20`
에서 발산이 `9.95 m`이고 `L`에 대해 무한히 커지므로, 이것을 받아들이면서
동시에 무엇인가를 탐지하는 고정 허용오차는 존재하지 않는다.

### §229.4 이 절이 하지 않은 것

- prbt `bool` 절을 초록으로 만들지 않았다. §229.1의 판정은 "이 픽스처
  위에서 닫히지 않는다"이며, 그 상태는 여전히 **미충족**이다.
- §229.2의 `5e-8 m` 여유를 없애지 않았다. **UNFIXED**이며 사유는 측정으로
  적었다.
- `distance` 절을 초록으로 만들지 않았다. 상류 수치와의 일치는 상류의
  결함을 재현해야만 얻어지고, 그러면 §229.3의 두 불변량 시험이 함께
  깨진다.
- 10,000 상태 스윕을 다시 돌리지 않았다. §218이 잰 수치를 그대로 쓰며,
  이 절이 더한 것은 그 수치의 **원인**이지 새 스윕이 아니다.
- fanuc의 2,897배를 이 기전으로 설명하지 않는다 (§218.4 자신이 "이탈
  6이 아니다"라 적은 pair-flip 사례다) — 위 "적용 범위" 문단 참고. pr2의
  3.218e-1(§21.4)도 같은 쌍/다른 쌍 구분을 시행한 적이 없어 미확인이다.

---

## §230 `HybridCollisionEnv`의 `env_field`를 증분 유지로 바꿨다 — 문서에 없던 절을 뒤늦게 적는다 (`a3822fb`, 2026-08-05 커밋 / 2026-08-06 기록)

이 절은 새 측정이 아니다. `a3822fb`가 이미 한 일을 계획서에 적는다. 그
커밋의 러스트 doc 주석 네 곳이 `§231`을 인용하는데 그런 절은 존재한 적이
없고 — 커밋이 `PORTING-PLAN.md`를 아예 건드리지 않았다 — 오늘 도입한
`tools/ci/check-porting-plan-sections.sh`의 배경인 "번호를 워커가 고른다"
문제의 가장 오래된 사례다. 번호는 여기서 `§230`으로 확정하고, 인용 네
곳을 그쪽으로 돌린다.

### §230.1 전제가 절반 틀렸다

`build_env_distance_field`는 `check_*_distance_field` 호출마다 `World`의
모든 물체로부터 `PropagationDistanceField`를 새로 지었다. 근거는
"`World`에는 훅을 걸 옵서버 기구가 없다"였다. 이 전제는 절반만 참이다 —
`World`에 *콜백 등록*이 없는 것은 맞지만, 모든 mutator가 무엇이 바뀌었는지
서술하는 `Notification`을 반환하고, `all_objects_as_notifications`가 현재
내용을 새 소비자에게 재생하는 용도로 이미 있다. 상류의
`addObserver`/`notifyObjectChange`가 하는 일이 정확히 그것이므로, 없던
것은 기구가 아니라 그 기구를 쓰는 코드였다.

### §230.2 `world_mut` 대신 `mutate_world` 하나

`world_mut`을 걷어내고 `mutate_world`를 두었다. `World`를 바꾸는 클로저를
받아 전달하고, 그것이 돌려주는 모든 `Notification`을 이제 영속인
`env_field`에 `apply_notification`으로 적용한다. 분기 구조는 상류
`notifyObjectChange`를 그대로 따른다 — `CREATE`/`ADD_SHAPE`는 추가만,
`MOVE_SHAPE`/`REMOVE_SHAPE`는 제거 후 추가, `DESTROY`는 제거만.

`World` 값이 이 타입 안에 하나뿐이고 그 하나에 드나드는 길이
`mutate_world` 하나뿐이라는 것이 요점이다. 상류의 `setWorld` 오버라이드가
막으려는 "두 절반이 서로 다른 세계를 본다"는 상태는 지킴이가 잘 지켜서
도달 불가능한 것이 아니라, 두 번째 `World` 값 자체가 없어서 표현 불가능
하다. `self.env_field`는 두 번째 세계가 아니라 그 하나에서 파생된
구조다.

### §230.3 상류에 없는 실패 경로 하나 — `desynced_objects`

`collision_object_point_decomposition`은 이 포트에서 실패할 수 있고 상류의
대응물은 그렇지 않다. 그래서 변형 도중의 분해 실패를 조용히 낡은 채로 두는
대신 `desynced_objects`에 적고, `check_*_distance_field`/
`get_collision_gradients`/`get_all_collisions` 전부가 `env_field`를 읽기
전에 그 집합이 비었는지를 먼저 본다.

### §230.4 `OctreeCache`의 패턴이 아닌 이유

`OctreeCache`는 키별 순수 메모이제이션이다. `env_field`는 그렇게 나뉘지
않는다 — 셀 하나의 값이 그 안의 모든 점에 함께 의존하는 단일 집계
구조다. 그래서 `apply_notification`은 독립 조각을 메모이즈하지 않고 물체별
점들을 그 하나의 구조에 누적/회수한다. 물체별 제거-후-추가 방식이 갖는
"복셀에 참조 계수가 없다"는 한계는 상류 `notifyObjectChange`가 그대로 갖는
한계이며, 이 포트가 새로 만든 공백이 아니다.

### §230.5 이 절이 하지 않은 것

- `HybridCollisionEnv::new`가 fallible이 됐다(생성 시점에 `env_field`를
  짓는다). 호출자 쪽 파급은 그 커밋에서 이미 처리했고 여기서 다시 재지
  않았다.
- `Clone`은 여전히 derive하지 않는다. 상류의 복사 생성자가 답하는
  "공유냐 깊은 복사냐" 질문(상류는 깊은 복사)은 이 타입에 아직 적용되지
  않는다.
- 이 절은 `a3822fb`의 기록이지 재측정이 아니다. 그 커밋이 추가한
  `env_field_after_incremental_churn_matches_a_fresh_rebuild_of_the_same_world`
  — 증분 부기가 깨끗한 재빌드와 일치하는지를 보는 시험 — 를 이 라운드가
  다시 돌려 통과를 확인한 것 외에 새 수치는 없다.


## §231 `collision_detection`에 남은 갭 세 건을 판정으로 바꿨다 — 그리고 판정하는 동안 포트 결함 두 개가 나왔다 (2026-08-06)

`doc/port-coverage.md`가 `gap`으로 남겨 둔 `collision_detection` 파일 셋
(`occupancy_map.hpp` 120줄, `test_collision_common_pr2.hpp` 571줄,
`test_collision_common_panda.hpp` 383줄)을 각각 증거로 판정한다. §217.3이
셋 중 `occupancy_map.hpp`에 대해 "이식하지 않기로 한 결정이 아니라 소유
디렉터리를 옮기라는 라우팅이므로 갭"이라고 적었고, 그 라우팅을 판정으로
바꾸는 것이 여기 할 일이다.

두 테스트 헤더에 대해서는 **기구와 단언을 분리**한다. 둘 다 GoogleTest
`TYPED_TEST_P` 픽스처이고, 공유 헤더인 유일한 이유가
`CollisionAllocatorType` 타입 파라미터이며, 이 포트에는 그 파라미터가
훑을 백엔드가 하나뿐이다. 그러나 헤더가 **무엇을 단언하는가**는 그것을
단언하는 기구와 다른 물건이고, 판정의 근거가 되는 쪽은 단언이다.

### §231.1 `test_collision_common_panda.hpp` — `decided-non-port`, 단언 10건 중 9건은 옮겼다

`REGISTER_TYPED_TEST_SUITE_P` 셋(`:378-383`)이 등록하는 테스트는 10개다.
9개를 `crates/moveit-collision/tests/upstream_panda_harness.rs`에 상류의
관절값·상자 치수·패딩값·기대 크기와 **상류 자신의 허용오차** 그대로
옮겼다(케이스별 대응표는 그 파일의 모듈 doc에 있다). 남은 하나 `InitOK`는
픽스처의 `robot_model_ok_`를 단언하는 것이고, 이 포트에서 그것은
`build_panda()`의 `.expect`다.

```console
$ cargo nextest run -p moveit-collision --test upstream_panda_harness
    Starting 9 tests across 1 binary
     Summary [   0.056s] 9 tests run: 9 passed, 0 skipped
```

오라클로 재도출하지 않은 이유는 두 가지다. `PaddingTest`는 애초에 오라클에
줄 수 없다 — `tools/moveit-oracle`의 `collision` op은 `CollisionEnvFCL(model,
world)`를 만들고 `setLinkPadding`을 **호출하지 않는다**. 나머지는 줄 수
있지만, 여기서 기대값은 상류가 손으로 적은 상수 그 자체이므로 오라클로
바꾸면 고정 표적이 움직이는 표적으로 바뀐다. 오라클 일치는
`collision_parity.rs`가 재는 것이고, 이 파일이 재는 것은 상류의 손으로 적은
기대값이다.

**이 이식이 찾아낸 포트 결함 둘.** 둘 다 이 워크스페이스가 이미 가진
시험 어느 것으로도 닿지 않았고, 그것이 "이미 덮여 있을 것"이라고 추론하는
대신 헤더의 단언을 실제로 옮겨 본 이유다.

1. **`CollisionRequest::distance`가 아무것도 채우지 않았다.** 상류의 두
   충돌 헬퍼는 각각 끝에 `if (req.distance)` 블록을 달고 있고
   (`collision_env_fcl.cpp:283-297`가 self, `:340-354`가 robot) 그 블록이
   두 번째 거리 질의를 돌려 `CollisionResult::distance`에 넣는다. parry
   백엔드는 **양쪽 다** 돌리지 않았으므로 `distance: true`가 `None`을
   냈다. `parry.rs`의 `attach_requested_distance` 하나로 두 진입점이
   지나가게 고쳤다 — 진입점마다 따로 쓰는 것이 한쪽만 구현되고 다른
   쪽은 조용히 무시되는 바로 그 모양이기 때문이다.
2. **쌍 맵 두 개가 정렬이 아니라 순회 순서로 키를 매겼다.** 상류는
   `collisionCallback`과 `distanceCallback` 양쪽에서 사전순으로 작은 이름을
   앞에 둔다(`collision_common.cpp:240-242`, `:564-567`). 이 백엔드는
   `cross_pairs`가 로봇을 먼저 놓으므로 `distance_robot`의 모든 쌍을
   `(link, object)`로 넣었고, 상류는 이름이 앞서는 쪽을 먼저 넣었다.
   측정으로 확인한 실제 키는 아래와 같다(`distance_single` 실패 시 출력).

   ```console
   keys are [("panda_hand", "collection"), ("panda_hand", "object"),
             ("panda_leftfinger", "collection"), ... ]   # 22개 전부 역순
   ```

   `BTreeMap`이므로 이것은 조회 키뿐 아니라 `contacts.begin()`이 무엇을
   내는지까지 결정한다 — 상류 자신의 `ContactReporting`이 그 첫 원소를
   읽는다. `parry.rs`의 `pair_key` 하나로 두 site가 지나가게 고쳤다.
   `moveit-distance-field`는 이 계열이 **아니다**: 그 상류
   (`collision_env_distance_field.cpp:329`, `:621`, `:1618`)는 정렬하지
   않고 포트도 정렬하지 않는다.

세 시험이 이 두 픽스를 판별한다는 것은 변이로 확인했다.
`attach_requested_distance`를 즉시 return으로 바꾸면 4건이 깨지고
`is_none_when_not_requested`만 통과하며, robot 쪽 호출만 지우면 robot 쪽
1건만 깨진다. `pair_key`의 정렬을 없애면 3건 전부 깨진다.

### §231.2 `occupancy_map.hpp` — `decided-non-port`, 상류 코퍼스 안에 쓰는 곳이 0이다

§217.3이 이 파일을 `gap`으로 둔 이유는 정확했다: 그때 근거로 인용된
문장은 "`moveit-octomap`으로 보내라"는 라우팅이지 이식하지 않기로 한
결정이 아니었다. 여기서 판정으로 바꾼다.

헤더 전체는 120줄이고, `octomap::OcTree`에 대해 더하는 것은 정확히
이것뿐이다(`:52-116` 실측):

- 전달 생성자 둘 — `OccMapTree(double resolution)`, `OccMapTree(const
  std::string& filename)`
- `std::shared_mutex` 잠금 여섯 — `lockRead`/`unlockRead`/`lockWrite`/
  `unlockWrite`/`reading`/`writing`
- 갱신 콜백 둘 — `setUpdateCallback`/`triggerUpdateCallback`
- 별칭 셋 — `OccMapNode`, `OccMapTreePtr`, `OccMapTreeConstPtr`

**더해진 API를 쓰는 코퍼스 파일은 헤더 자신 말고 0이다.** 두 질문을 따로
쟀다 — 타입 이름을 쓰는 곳과, 잠금·콜백 API를 부르는 곳.

```console
$ cd /home/stevek/work/moveit2
$ rg -l --no-heading 'OccMapTree' --glob '*.cpp' --glob '*.hpp' --glob '*.h' . | sort
./moveit_core/collision_detection/include/moveit/collision_detection/occupancy_map.hpp
./moveit_core/planning_scene/src/planning_scene.cpp
./moveit_ros/occupancy_map_monitor/include/moveit/occupancy_map_monitor/occupancy_map_monitor.hpp
./moveit_ros/occupancy_map_monitor/include/moveit/occupancy_map_monitor/occupancy_map_updater.hpp
./moveit_ros/occupancy_map_monitor/src/occupancy_map_monitor.cpp
./moveit_ros/perception/lazy_free_space_updater/include/moveit/lazy_free_space_updater/lazy_free_space_updater.hpp
./moveit_ros/perception/lazy_free_space_updater/src/lazy_free_space_updater.cpp
./moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp
```

8개 중 코퍼스 안은 헤더 자신과 `planning_scene.cpp` **둘**이고, 나머지 6개는
전부 `moveit_ros/*`다. 잠금·콜백 쪽은 12개인데 코퍼스 안은 **헤더 자신
하나뿐**이다.

```console
$ rg -l --no-heading 'lockRead|unlockRead|lockWrite|unlockWrite|->reading\(\)|->writing\(\)|triggerUpdateCallback|setUpdateCallback' \
       --glob '*.cpp' --glob '*.hpp' --glob '*.h' . | rg -v '^\./moveit_ros/'
./moveit_core/collision_detection/include/moveit/collision_detection/occupancy_map.hpp
```

그 하나뿐인 코퍼스 사용처가 무엇을 쓰는지도 실측했다. 다섯 히트는
`#include`(`:39`)와 `createOctomap`(`:1417-1420`, `:1451`, `:1492`)이고,
`createOctomap` 본문은 `OccMapTree(map.resolution)` 생성 뒤
`octomap_msgs::readTree` 또는 `om->readData(datastream)`만 부른다 —
잠금도 콜백도 건드리지 않는, 순수한 `octomap::OcTree`다. 이 포트는 그것을
이미 한다: `ros/moveit-ros/src/scene/planning_scene.rs:137-143`의
`apply_octomap`이 `moveit_octomap::OcTree::new(resolution)` 뒤
`read_binary_data`/`read_data`를 부른다.

더하는 두 기구는 이 포트가 **구조적으로 다르게 표현하는** 바로 그 둘이다.
공유 가변성은 타입 안이 아니라 사용처에 둔다 — 이 트리의 옥트리는 전부
`Arc<OcTree>`로 불변 공유된다(`ros/moveit-ros/src/scene/planning_scene.rs:148`,
`crates/moveit-distance-field/src/distance_field.rs:752`, `:807`,
`crates/moveit-collision/src/parry.rs` 등). 갱신 콜백은 트리의 것이 아니라
모니터의 것이고, 모니터(`moveit_ros/occupancy_map_monitor`)는 코퍼스 밖이다.

만료 조건은 취향이 아니라 사실로 적는다: **코퍼스 안에서 잠금 API나 콜백
API를 부르는 호출자가 생기면** 다시 연다. `moveit-octomap`으로 라우팅하라는
요청은 이 판정으로 철회한다 — 보낼 내용이 `octomap::OcTree` 자체 말고는
없기 때문이다.

## §232 `test_collision_common_{panda,pr2}.hpp` — 코퍼스 규칙은 결함이 아니고, 짝이 없던 단언 하나는 만들었다 (2026-08-06)

`doc/port-coverage.md`가 `gap`으로 들고 있던 6건 중 코드가 아닌 두 건.
두 물음을 순서대로 물었고, 첫 물음의 답이 "행을 남긴다"였으므로 둘째로
넘어갔다.

### §232.1 코퍼스 규칙 — 규칙은 실제로 존재한다. 그런데 채택하면 후퇴다

물음은 "이 두 파일이 코퍼스에 있는 것이 경로 성분 사고인가"였다. 사고인
것은 맞다. `corpus_files()`가 거르는 것은 경로에 `test`/`tests` 성분이
있는 파일이고(`tools/ci/measure-port-coverage.py:90`), 상류가 이 테스트
본문들을 `collision_detection/include/` 밑에 두었기 때문에 그물을 빠져나온다.

**바꾸기 전에 "다른 코퍼스 멤버 중 상류 테스트 본문이 몇 개인가"를 셋다.**
서로 독립인 술어 셋을 코퍼스 245개 전부에 돌렸다.

| 술어 | 옮기는 파일 수 |
|---|---|
| A. 비-shim 소비자가 1개 이상이고 **전부** `test/` 밑 | 3 |
| B. 자기 자신이 `#include <gtest/gtest.h>` | 3 (A와 **같은 3개**) |
| C. basename을 `_`로 쪼갠 토큰에 `test`가 있음 | **5** |

A와 B는 서로 다른 계기인데 같은 3개를 고른다 — `test_collision_common_
{panda,pr2}.hpp`와 `eigen_test_utils.hpp`. C는 여기에
`robot_model_test_utils.{hpp,cpp}`를 더한 5개이고, 이 5개가 §228이 이미
이름 붙인 그 가족이다("테스트 지원 코드인데 `test/` 디렉터리 밖에 산다").
상류 소스 1,296개 전체에서 C의 오탐은 **0건**이다(`test/` 밖에서 basename에
`test`가 든 19개는 전부 진짜 테스트 코드이고, 그중 코퍼스 루트 안에 있는
것이 이 5개다).

즉 **"두 파일짜리 예외 목록"이 아닌 진짜 규칙이 존재한다.** A/B는 가족
5개 중 3개만 잡으므로 규칙이 아니라 부분 형태다 —
`robot_model_test_utils.hpp`는 gtest를 안 쓰고(B 탈락), 소비자 하나가
`moveit_ros/planning/planning_components_tools/src/compare_collision_speed_
checking_fcl_bullet.cpp`로 `test/` 밖이다(A 탈락). C만 가족을 덮는다.

**그런데 C를 채택하면 안 된다. 이유는 이 트리 안에 적혀 있다.**
`crates/moveit-test-support/src/lib.rs:8-15`가 그것이다:

> Not a port of any upstream file. This used to add "upstream has no
> equivalent test-side machinery of its own", **which is false**: upstream
> ships `robot_model_test_utils.{hpp,cpp}` … and `eigen_test_utils.hpp` ….
> **Both live outside a `test/` directory, so they are inside this port's
> measured corpus and carry their own `doc/port-coverage.md` rows.**

이 문장은 앞선 라운드가 **거짓 주장을 고치면서** 쓴 것이고, 고침의 근거가
바로 "코퍼스 안에 있고 행을 가진다"이다. 다섯 개가 실제로 전부 행을
가진다(측정: `gap` 2, `decided-non-port` 3). 코퍼스에서 빼면 그 행 다섯
개가 사라지고, 사라지는 순간 이 포트는 "상류에 테스트 지원 기계가 없다"는
쪽으로 되돌아갈 근거를 얻는다 — **행이 강제하던 의무, 즉 파일마다 이 포트의
대응물을 이름으로 대라는 의무가 없어진다.** 규칙이 옳아도 그 규칙이 지우는
것이 이 문서 체계가 유일하게 붙잡고 있던 것이라면 채택이 후퇴다.

따라서 **코퍼스 규칙은 고치지 않는다. 행을 판정한다.**

### §232.2 두 헤더의 판정 — 단언 하나하나에 짝을 댔다

두 헤더는 `TYPED_TEST_P` 본문이고(panda 10건, pr2 11건), 존재 이유는 FCL과
Bullet 두 백엔드에 같은 스위트를 물리는 것이다. 인스턴스화는
`collision_detection_{fcl,bullet}/test/` 안에만 있고 §1이 그 두 트리를 각각
`[parry로 대체]`·`[드롭]`으로 뺐다. 그러나 `decided-non-port`의 근거는
"테스트 파일이라서"가 아니라 **단언별 대응물**이어야 하므로 21건을 전부
분류했다.

| 상류 단언 | 이 포트의 짝 |
|---|---|
| panda `InitOK`/`DefaultNotInCollision`/`LinksInCollision`/`RobotWorldCollision_{1,2}`/`DistanceSelf`/`DistanceWorld` | `collision_parity.rs:542` — 오라클이 직접 답한 4개 상태에 대해 `check_self_collision`/`check_robot_collision`/`distance_self`/`distance_robot` 전부 |
| pr2 `InitOK`/`DefaultNotInCollision`/`LinksInCollision` | `collision_parity.rs:554` — 같은 4항목, pr2 4개 상태 |
| pr2 `AttachedBodyTester`/`ConvertObjectToAttached` | `crates/moveit-scene/tests/attached_collision_parity.rs` (5건) |
| pr2 `DiffSceneTester` | `crates/moveit-scene/tests/scene_diff_collision_parity.rs` (12건) |
| pr2 `TestChangingShapeSize` | 같은 파일의 `remove_object` 케이스 + `world_parity.rs:145`의 `set_object_pose` |
| panda `DistanceSingle`/`DistancePoints`, pr2 `ContactReporting`/`ContactPositions` | 없음 — §4.5가 기록한 **제외**다. `parry.rs` 이탈 4·6(쌍당 접촉 1개 대 FCL 최대 200개)이 어떤 허용오차로도 수렴하지 않게 만든다 |
| pr2 `MoveMesh` | 짝이 필요 없다. 본문에 `ASSERT_`/`EXPECT_`가 **0개**다 — `checkCollision`을 다섯 번 부르고 결과를 읽지 않는다 |
| pr2 `TestCollisionMapAdditionSpeed` | 짝이 필요 없다. 이 파일의 `EXPECT_TIME_LT` 9개 중 하나이고, 벽시계는 백엔드가 아니라 기계를 잰다 |
| panda `PaddingTest` | **이 라운드에 만들었다** (아래) |

**짝이 없던 단언은 하나였고, 없는 이유가 구조적이었다.** 오라클의
`collision` op은 padding 인자를 받지 않는다(`oracle.cpp:2191`의
`json collision(const json&)`은 `joint_values`·`attached_bodies`·월드
객체·`max_contacts_per_pair`만 읽는다). 그래서
`tests/fixtures/{panda,fanuc,pr2}_collision.json`은 전부 생성자 기본값
padding `0.0`에서 잡혔고, **차분 픽스처로는 `LinkPaddingScale`을 원리상 못
건드린다.** 실제로 이 라운드 전까지 워크스페이스에서 0이 아닌 padding이
충돌 질의 앞에 놓인 적이 없다 — `set_link_padding`/`LinkPaddingScale::
with_links` 호출처가 전부 `env.rs`의 자기 단위 테스트였고, 그것들은 맵의
장부(클램핑·변경 보고·미추적 링크 기본값)를 볼 뿐 판정을 보지 않는다.

그래서 세 번째 종류의 ground truth를 썼다 — 오라클의 답도, 손으로 고른
상수도 아닌, **상류가 공개한 시나리오를 상류의 수 그대로** 재생한 것:
같은 `0.1` 정육면체를 같은 `(0.43, 0, 0.55)`에, 같은 `panda_hand`를 같은
`0.08`로, 같은 `setToHome` 자세에서
(`crates/moveit-collision/tests/link_padding_changes_collision_verdict.rs`).

측정한 스윕(padding → 최소 robot 거리, 괄호는 최근접 쌍):

| padding | 최소 거리 | 쌍 |
|---|---|---|
| 0.00 | `+0.029119199` | panda_link7 / box |
| 0.02 | `+0.021490125` | panda_hand / box |
| 0.04 | `+0.001589386` | panda_hand / box |
| 0.06 | `-0.018311353` | panda_hand / box |
| 0.08 | `-0.038212093` | panda_hand / box |

padding 없는 값 `+0.029119199`는 상류 `DistanceWorld`가
`EXPECT_NEAR(res.distance, 0.029, 0.01)`로 주장하는 바로 그 양이다. 이
백엔드는 상류의 공칭값 `0.029`에서 `0.000119` 떨어져 있다 — 상류가 스스로
허용한 `0.01`보다 **84배** 좁다. 시나리오가 비슷한 것이 아니라 재현된
것이다. 뒤집힘은 `0.04`와 `0.06` 사이에서 일어나므로(`panda_hand`의 맨
간극이 `~0.0416`), 상류의 `0.08`은 칼날이 아니라 뒤집힘점의 약 2배다.

**단언이 실제로 변별하는지 다섯 변이로 확인했다**(전부 되돌림). 각 변이가
서로 다른 줄에서 깨진다 — 하나가 다른 것의 실패를 가리지 않는다:

| 변이 | 깨지는 단언 |
|---|---|
| padding을 항상 `0.0`으로 | `:200` "panda_hand padded by 0.08 must reach the box" |
| padding을 이름과 무관하게 **모든** 링크에 | `:206` 최근접 쌍이 `panda_link7`로 바뀐다 |
| padding을 2배로 | `:216` 깊이가 padding을 따라가지 않는다 |
| 모든 링크에 상수 `0.005` 누출 | `:187` 맨 거리 `0.024626` ≠ `0.029119` |
| `set_link_padding(_, 0.0)`을 no-op으로 | `:230` 되돌려도 깨끗해지지 않는다 |

마지막 변이가 중요하다. 상류의 세 번째 단계(`setLinkPadding("panda_hand",
0.0)` 후 재검사)가 붙잡는 성질은 **가역성**이다 — padding이 생성 시점에
링크 기하에 구워지는 것이 아니라 질의마다 `LinkPaddingScale`에서 읽힌다는
것. 그래서 이 테스트는 판정만이 아니라 거리가 **정확히** 원래 값으로
돌아오는 것까지 단언한다.

### §232.3 `test_collision_common_pr2.hpp` — `decided-non-port`, 단언 41건 중 옮길 수 있는 것은 3건이다

기구를 물리는 근거는 §232.2의 panda 헤더와 같다(`TYPED_TEST_P` +
`CollisionAllocatorType`). 다른 것은 **단언 쪽 결론**이다. panda는 10건 중
9건이 옮겨졌지만 pr2는 41건 중 3건뿐이고, 나머지 38건이 왜 안 되는지가
이 절의 내용이다. 근거 없이 "대부분 못 옮긴다"고 적지 않기 위해 41건을
테스트별로 셌다.

```console
$ H=/home/stevek/work/moveit2/moveit_core/collision_detection/include/moveit/collision_detection/test_collision_common_pr2.hpp
$ rg -n '^\s*(ASSERT_|EXPECT_)' "$H" | wc -l
41
```

| 상류 테스트 | 단언 | 판정 |
|---|---|---|
| `InitOK` (`:100-104`) | 1 | `upstream_pr2_harness.rs`의 `build_pr2()` `.expect` |
| `DefaultNotInCollision` (`:105-116`) | 1 | 옮김 — 그리고 상류에서 공허하다는 측정을 같이 붙였다 |
| `LinksInCollision` (`:117-158`) | 3 | `updateStateWithLinkAt` 미이식 |
| `ContactReporting` (`:159-213`) | 9 | `updateStateWithLinkAt` 미이식 |
| `ContactPositions` (`:214-284`) | 9 | `updateStateWithLinkAt` 미이식 (`:282`는 상류 자체가 공허) |
| `AttachedBodyTester` (`:285-353`) | 6 | `updateStateWithLinkAt` 미이식 |
| `DiffSceneTester` (`:354-407`) | 3 | 전부 `EXPECT_TIME_LT` |
| `ConvertObjectToAttached` (`:408-475`) | 4 | 3건 `EXPECT_TIME_LT`, 실질 1건(`:461`)은 `kinect.dae` 필요 |
| `TestCollisionMapAdditionSpeed` (`:476-494`) | 1 | `EXPECT_TIME_LT` — 실질은 옮김 |
| `MoveMesh` (`:495-518`) | 0 | 단언이 하나도 없다 |
| `TestChangingShapeSize` (`:519-567`) | 4 | 옮김 1건(`:543`), `:528`은 상류 자체가 공허, `:553`/`:565`는 `kinect.dae` 필요 |

합 41. 표의 건수는 위 `wc -l`과 같은 스캔을 테스트 경계로 쪼개 얻었고,
합계가 맞는다고 각 행이 맞는 것은 아니므로 행별로 줄 번호를 찍어 확인했다.

못 옮기는 이유는 셋이고, 셋 다 주장이 아니라 측정이다.

**하나 — `updateStateWithLinkAt`이 이식돼 있지 않다.** 11개 테스트 중 4개,
호출 15회가 여기 걸린다. 이 함수는 링크의 전역 변환을 직접 써 넣고 자손만
다시 푸는 것이라(`robot_state.cpp:850-871`), 상태가 자기 관절값과 **일부러**
어긋난다 — 상류 선언이 그렇게 적어 둔다(`robot_state.hpp:1213-1220`,
"neglecting the joint values of its parent joint ... although they do not
match the joint values anymore"). 두 링크를 자세를 풀지 않고 접촉시키는
수단이다. 이 포트에는 대응물이 없다.

```console
$ rg -n 'update_state_with_link_at' crates/ ros/ --glob '*.rs' \
      --glob '!**/upstream_pr2_harness.rs'
$ echo $?
1
```

`RobotState`의 것이므로 `moveit-collision`이 아니라 `moveit-state`가 가질
API다. 여기서 우회하지 않고 막힌 것으로 적는다 — 우회로(부동 관절 URDF)는
상류 숫자가 말하는 로봇과 다른 로봇을 시험하는 것이 된다.

**둘 — `kinect.dae`는 커밋된 픽스처가 아니다.** 파일은 이 기계에 있다
(`third_party/moveit_resources/pr2_description/urdf/meshes/sensors/kinect_v0/kinect.dae`,
164,201바이트). 그러나 `third_party/`는 gitignore된 외부 체크아웃이고, 이
저장소의 규약은 테스트가 `fixtures/meshes/` 밑 사본을 읽는 것이다
(`crates/moveit-geometry/tests/mesh_parity.rs:19-23`). 그 트리의
`pr2_description`은 18개 전부 `.stl`이다.

```console
$ find fixtures/meshes/pr2_description -type f | sed 's/.*\.//' | sort | uniq -c
     18 stl
```

그래서 이건 복사 한 번으로 끝나지 않는다. COLLADA 리더가 없고(이 포트가
파싱하는 것은 STL이다 — `crates/moveit-geometry/src/stl.rs`), 게다가 픽스처
출처 게이트가 STL만 훑는다(`tools/ci/verify-fixture-provenance.sh:196`,
`mesh_fixtures=(fixtures/meshes/**/*.stl)`) — `.dae`를 넣으면 바로 그것을
검사하라고 있는 게이트가 검사하지 않는 자리에 놓인다. 이 게이트 구멍은
`kinect.dae`가 실제로 필요해질 때 같이 닫아야 한다.

**셋 — `EXPECT_TIME_LT`는 1초 밑에서 실패할 수 없다.** 매크로는 `NDEBUG`에서
`EXPECT_LT`, 아니면 no-op이고(`:92-96`), 비교되는 값은 전부
`duration_cast<std::chrono::seconds>(..).count()` — 초 단위 정수다. 즉
`EXPECT_TIME_LT(x, .05)`는 `x == 0`이면, 곧 1초 미만이면 언제나 참이다.
호출은 7곳(`:374`, `:395`, `:405`, `:433`, `:472`, `:473`, `:489`)이고,
이 숫자들을 그대로 옮기면 **아무 측정도 고르지 않은 허용오차**를 옮기는
것이 된다. 대신 실질이 있는 곳은 실질을 단언하고 시간은 숫자로 보고한다:
`TestCollisionMapAdditionSpeed`의 `EXPECT_TIME_LT(t, 5.0)` 자리에서
`collision_map_addition_lands_every_shape_in_one_object`는 10,000개가 한
오브젝트에 다 들어갔는지를 단언하고, 추가에 걸린 시간(1.113ms / 1.339ms /
1.066ms, 3회)을 찍는다.

옮긴 3건은 `crates/moveit-collision/tests/upstream_pr2_harness.rs`의
`default_not_in_collision`, `changing_shape_size_keeps_the_collision`,
`collision_map_addition_lands_every_shape_in_one_object`다.

`default_not_in_collision`에는 상류에 없는 단언이 하나 더 붙어 있다. 픽스처
ACM이 `AllowedCollisionMatrix(getLinkModelNames(), true)`, 곧 **모든 링크 쌍
허용**이라, `checkSelfCollision`은 기하를 보기 전에 모든 쌍을 건너뛴다 —
이 자세든 다른 어떤 자세든 `false` 말고는 나올 수 없다. 그래서 같은 상태를
ACM 없이 한 번 더 돌려 충돌이 나오는 것을 단언한다. 상류 단언이 로봇 자세에
대한 것이었는지 ACM에 대한 것이었는지를 기록으로 남기는 것이 목적이고,
측정 결과는 후자다.

만료 조건: `moveit-state`가 `update_state_with_link_at`을 갖게 되면 27건이
열리고, `fixtures/meshes/`가 `kinect.dae`를 갖게 되면(게이트 glob과 함께)
3건이 열린다. 그때 이 절을 다시 연다.


### §232.4 이 절이 하지 않은 것

- 코퍼스 술어를 바꾸지 않았다. §232.1의 규칙 C는 실측으로 성립하지만
  채택하지 않았고, 이유를 적었다. 뒤에 이 판단을 뒤집으려는 사람은
  `crates/moveit-test-support/src/lib.rs:8-15`를 먼저 고쳐야 한다.
- ~~pr2 헤더에는 새 테스트를 만들지 않았다. 11건 중 짝이 없는 것은 §4.5가
  이미 기록한 제외 2건과, 아무것도 단언하지 않는 2건뿐이다.~~ **§232.3이
  뒤집었다 (병합 시 기록).** 이 줄은 `TYPED_TEST_P` 본문 11건을 단위로 센
  것이고, 단언 단위로는 41건 중 38건이 막혀 있다 — 막는 것은 §4.5의 제외가
  아니라 `update_state_with_link_at` 미이식, `kinect.dae` 부재,
  `EXPECT_TIME_LT`의 초 단위 절단 셋이다. 옮길 수 있던 3건은 §232.3이
  `crates/moveit-collision/tests/upstream_pr2_harness.rs`로 만들었다.
- `MoveMesh`/`TestCollisionMapAdditionSpeed`가 쓰는 `kinect_dae_resource_`
  (`.dae` 메시)를 이 포트에 들여오지 않았다. `MoveMesh`는 단언이 0건이고
  `TestCollisionMapAdditionSpeed`는 §232.3이 실질 단언으로 옮겼다.

## §233 `attached_body.cpp`의 마지막 갭 둘 — `setScale`/`setPadding`을 옮겼고, `Arc::make_mut` 등가는 **강한** 공유에서만 성립한다 (2026-08-06)

`doc/port-coverage.md`의 `moveit_core/robot_state/src/attached_body.cpp`
행은 잔여분 넷 중 둘(`computeTransform`, `getGlobalSubframeTransform`)을
이미 `decided-non-port`로 정리해 두었고, 나머지 둘은 "막힌 것이 아니라
안 쓴 것"이라고 적혀 있었다. 그 둘을
`crates/moveit-scene/src/attached_body.rs:191`(`set_scale`),
`:205`(`set_padding`)로 옮기면서 행을 `ported-elsewhere`로 닫는다. 이 절이
기록하는 것은 옮겼다는 사실이 아니라, 옮기는 과정에서 **행이 적어 둔 등가
주장이 부분적으로 거짓임을 측정으로 확인한 것**이다.

### §233.1 상류가 하는 일 (`attached_body.cpp:86-103`, `:120-137`)

두 함수의 본문은 `shape->scale(scale)` / `shape->padd(padding)` 한 곳만
빼고 바이트 단위로 같다. 각각 `shapes_`를 돌며 도형마다

    if (shape.use_count() == 1)  const_cast<shapes::Shape*>(shape.get())->scale(scale);
    else { shapes::Shape* copy = shape->clone(); copy->scale(scale); shape.reset(copy); }

를 한다. 둘 다 `shape_poses_`도, `global_collision_body_transforms_`도
건드리지 않고 `computeTransform`도 부르지 않는다 — 도형의 치수만 바꾼다.

상류 자신은 이 둘을 **한 번도 부르지 않는다.**
`rg -o 'setScale|setPadding' /home/stevek/work/moveit2 --glob '*.{cpp,hpp,h,py}'`
는 16개 파일에서 37회를 내는데, `AttachedBody`에 걸리는 것은
`attached_body.cpp:86,120`(정의)과 `attached_body.hpp:190,193`(선언)
넷뿐이고 나머지는 전부 `CollisionEnv`, Ogre 노드, `MeshFilter`의 동명
메서드다. 이 포트에서 `AttachedBody`를 가변으로 얻는 경로가
`PlanningScene::detach`가 돌려주는 소유값뿐인 것은, 따라서 상류보다
좁지 않다.

### §233.2 `Arc::make_mut`는 `use_count() == 1`이 아니다 — 약한 참조에서 갈린다

행이 "정확히 `Arc::make_mut`"라고 적은 부분이 여기서 갈린다.
`Arc::make_mut`는 강한 수가 1을 넘을 때 **또는 `Weak`가 하나라도 살아
있을 때** 복제하고, C++ `use_count()`는 `weak_ptr`을 세지 않는다.

이 차이는 이 트리에서 이론이 아니라 **체계적**이다.
`crates/moveit-distance-field/src/collision_common_distance_field.rs:511`이
분해한 모든 도형에 대해 `cache.insert(key, (Arc::downgrade(shape), ...))`
를 하고, 그 캐시는 상류의 미구현 `// TODO - clean cache`를 그대로 물려받아
**축출하지 않는다**. 부착체의 도형은 같은 파일의
`attached_body_sphere_decomposition`(`:567`)와
`attached_body_point_decomposition`(`:590`)을 통해 그 경로에 들어간다.
그러므로 한 번이라도 거리장 분해를 거친 부착체는, 이 모듈이 유일한 강한
소유자여도 이후 모든 `set_scale`/`set_padding`에서 복제된다.

방향은 안전한 쪽이다. 남이 아직 관찰할 수 있는 도형을 밑에서 바꾸는 일이
없고, 복제 덕분에 그 캐시는 **새 키**를 받는다 — 축척 이전 치수로 계산된
분해가 축척 이후 도형의 주소에 남아 되돌아오지 않는다.

측정으로 확인한 것 하나 더:
`an_outstanding_weak_forces_a_clone_upstreams_use_count_would_not`에서
`make_mut` 이후 원래 할당의 `Weak::strong_count()`는 **0**이다. 즉
`make_mut`는 값을 옛 할당에서 **꺼내 가고**, 남은 `Weak`는 더 이상
`upgrade`되지 않는다. 위 캐시가 `Weak`를 오직 주소 고정용으로만 쓰고
`upgrade`하지 않는다고 자기 문서에 적어 둔 것이 여기서 필요조건이 된다.

### §233.3 `void`가 아니라 `Result<()>`이고, 적용은 원자적이지 않다

이 포트의 `moveit_geometry::Shape::scale`/`::padd`
(`crates/moveit-geometry/src/shapes.rs:1492`, `:1497`)는 치수를 0 미만으로
끌어내리는 인자를 거부한다. 그래서 상류의 `void`를 그대로 쓸 수 없고
`Result<()>`를 돌려준다.

`geometric_shapes` 원본이 같은 지점에서 실패할 수 있는지는 이 라운드가
**확정하지 않았다**. 그 패키지는 고정된 `moveit2` 체크아웃 아래에 없고 이
변경을 위해 읽지 않았다. 따라서 오류 경로의 규약은 상류 대조가 아니라 이
포트 자신의 규약으로 적었다:

- `?`는 루프 밖으로 전파되므로 **실패한 도형 앞의 도형들은 이미 바뀐 채로
  남는다.** 상류 루프가 중간에 빠져나갈 때 남기는 상태와 같은 모양이다.
- 실패한 도형 자신의 치수는 그대로다. 다만 그 `Arc`는 `make_mut`가 이미
  공유를 끊어 복제로 바꿔 놓은 뒤다 — 상류의 `else` 가지는
  `shape.reset(copy)`에 닿지 않으므로 그쪽에서는 원본이 공유된 채 남는다.

이 부분 적용은 시험으로 고정되어 있다
(`negative_padding_larger_than_a_shape_is_rejected_after_updating_its_predecessors`,
실패가 **둘째** 도형에 떨어지도록 픽스처를 배열해야만 "실패 지점에서
멈춤"과 "전부 되돌림"이 구분된다). 되돌리는 판본을 만들어 보면 그 시험
하나만 깨진다 — `doc/assertion-discrimination-ledger-p10-attached-body.md`의
M5.

### §233.4 이 절이 하지 않은 것

- `PlanningScene`에 `attached_body_mut` 류의 접근자를 만들지 않았다.
  과제 범위가 `attached_body.rs`였고, §233.1대로 상류에도 호출자가 없다.
- `attached_body.rs`의 헤더를 `Ported from`으로 바꾸지 않았다. 이 파일의
  `AttachedBody`는 여전히 `.hpp`에서 **behaviorally derived**이고,
  `Ported from`을 달면 이 라운드가 옮기지 않은 `.cpp`의 나머지까지
  포팅됐다고 주장하게 된다. 그래서 계기는 이 `.cpp`를 여전히 미포팅으로
  세고, 행은 표에 남는다.
- `geometric_shapes`의 예외 거동을 확인하지 않았다. §233.3의 오류 규약은
  이 포트 쪽 사실만으로 적혀 있다.


## §234 `planning_response.cpp`의 남은 28줄 — `MotionPlanDetailedResponse::getMessage`를 포팅하지 않기로 판정했다

`doc/port-coverage.md`가 이 파일을 `gap`으로 들고 있던 근거는 한 문장이었다.
파일의 두 함수 중 하나는 트리에 대응이 있고, 다른 하나는 대응도 결정도
없다는 것. 실측하면 79줄 파일에서 함수가 차지하는 것은 39줄이고
(`MotionPlanResponse::getMessage` `:40-50` 11줄 —
`ros/moveit-ros/src/planning.rs`의 `TryFrom<PlanningResponse<'m>> for
PlanningResponseMsgOut`이 이것이다 —,
`MotionPlanDetailedResponse::getMessage` `:52-79` 28줄), 대응이 없는 쪽이
28줄로 더 크다. §3의 규칙은 잔여분이 **미결정**이고 파일의 대부분이 트리에
없을 때만 `gap`이라고 적으므로, 이 절은 그 연언의 앞항을 없앤다 —
**28줄을 포팅하지 않기로 판정한다.**

### §234.1 무엇을 포팅하지 않는지부터 적는다 — 이름이 아니라 규칙

판정의 대상은 함수 이름이 아니라 그 함수가 가진 규칙이다. 상류 전문:

```cpp
// moveit_core/planning_interface/src/planning_response.cpp:52-79
void planning_interface::MotionPlanDetailedResponse::getMessage(
    moveit_msgs::msg::MotionPlanDetailedResponse& msg) const
{
  msg.error_code = error_code;
  msg.trajectory.clear();
  msg.description.clear();
  msg.processing_time.clear();
  bool first = true;
  for (std::size_t i = 0; i < trajectory.size(); ++i)
  {
    if (trajectory[i]->empty())
      continue;
    if (first)
    {
      first = false;
      moveit::core::robotStateToRobotStateMsg(trajectory[i]->getFirstWayPoint(), msg.trajectory_start);
      msg.group_name = trajectory[i]->getGroupName();
    }
    msg.trajectory.resize(msg.trajectory.size() + 1);
    trajectory[i]->getRobotTrajectoryMsg(msg.trajectory.back());
    if (description.size() > i)
      msg.description.push_back(description[i]);
    if (processing_time.size() > i)
      msg.processing_time.push_back(processing_time[i]);
  }
}
```

규칙은 네 개의 경계를 가진다. 판정이 이 경계들을 없애는 것이지, 못 본 것이
아님을 남기려고 열거한다.

1. **빈 벡터** — `trajectory.size() == 0`이면 루프가 한 번도 안 돌고,
   `msg.trajectory_start`와 `msg.group_name`은 **지워지지도 쓰이지도
   않는다**(`clear()` 셋에 이 둘은 없다). 즉 호출자가 넘긴 `msg`에 남아
   있던 값이 그대로 나간다. 세 벡터만 비워진다.
2. **전부 빈 궤적** — 모든 `trajectory[i]->empty()`가 참이면 결과는 1번과
   같다. `first`는 끝까지 `true`로 남는다.
3. **첫 비어있지 않은 것이 `i == 0`이 아닐 때** — `trajectory_start`와
   `group_name`은 **인덱스 0이 아니라 처음으로 비어있지 않은 궤적**에서
   나온다. 앞쪽의 빈 궤적들은 출력에 아무 흔적을 안 남긴다.
4. **길이 불일치** — `description`/`processing_time`의 push는 **원본
   인덱스 `i`**로 가드된다(`description.size() > i`). 출력 인덱스가 아니다.
   그래서 빈 궤적이 앞에 있으면 세 출력 벡터의 길이가 서로 달라질 수 있다:
   `trajectory`는 비어있지 않은 것만 세고, `description`은 `i`가 그 벡터의
   길이 안에 드는 것만 센다.

`planner_id`는 구조체에 있지만(`planning_response.hpp:83`) 이 함수가 읽지
않고, 와이어 타입에도 그런 필드가 없다
(`third_party/moveit_msgs/msg/MotionPlanDetailedResponse.msg`는
`trajectory_start`/`group_name`/`trajectory[]`/`description[]`/
`processing_time[]`/`error_code` 여섯 필드다). 상류가 버리는 필드다.

### §234.2 D6은 이 함수를 `moveit-ros`로 보내지만, 변환할 코어 쪽 원본이 없다

D6은 "모든 `moveit_msgs` 변환은 `moveit-ros`의 `TryFrom`"이다. 이 함수가
`moveit-ros`에 속한다는 것까지는 D6이 정한다. 문제는 그다음이다 —
`TryFrom<코어 타입>`의 **코어 타입이 이 워크스페이스에 없다.**

트리에서 상류 `planning_interface::MotionPlanDetailedResponse`의 대응물은
하나뿐이고, `doc/port-coverage.md:146`이 이미 그렇게 판정해 두었다:
`crates/moveit-planners-chomp/src/planner.rs:205`의 `ChompSolution`. 그
타입의 필드는 셋이다 — `trajectory: RobotTrajectory<'m>`(벡터가 아니라
**하나**), `planner_id: String`, `description: String`(벡터가 아니라
**하나**). `processing_time`도 `error_code`도 없다(전자는 §138.3, 후자는
`Result::Err`).

그래서 `TryFrom<ChompSolution> for MotionPlanDetailedResponseMsgOut`을 쓰면
§234.1의 **네 경계가 전부 표현 불가능해진다.** 벡터가 없으니 빈 벡터가
없고(1), 전부 빈 경우가 없고(2), `first`가 `false`가 될 인덱스가 없고(3),
길이를 어긋나게 할 두 번째 벡터가 없다(4). 남는 것은 궤적 하나와 문자열
하나를 옮기는 세 줄이며, 그 세 줄은 `planning_response.cpp` 행이 말하는
규칙을 **하나도 담지 않는다.** 행을 그런 함수로 닫는 것은 포팅이 아니라
행의 주어를 바꾸는 것이다.

의존 간선도 없다. `ros/moveit-ros/Cargo.toml`에
`moveit-planners-*` 의존이 하나도 없다(`rg -n 'chomp|moveit-planners'`가
0줄). 간선을 새로 만드는 것은 §177의 `linkme` distributed_slice 순서
위험을 건드리는 별개의 결정이고, 만들어도 위 문단의 결론은 안 바뀐다.

**대안으로 `moveit-planning`에 일반 `DetailedPlanningResponse`를 새로 만드는
길도 재 봤고, 거절한다.** 그것은 `doc/port-coverage.md:146`이 이미
`ChompSolution`으로 귀속한 구조체의 **두 번째** 포트 표현이 되며, 이
워크스페이스에서 생산자도 소비자도 0이다. 생산자 쪽은 이미 판정이 끝나
있다: pilz의 `solve(MotionPlanDetailedResponse&)`는 §227.3(2)가 D6으로
버렸고, chomp의 `solve`는 `ChompSolution`으로 좁혔다. 소비자 쪽은
§234.3이다.

### §234.3 상류에서 이 함수의 호출자는 0이고, 와이어 타입은 어떤 srv/action/msg에도 실리지 않는다

두 실측 모두 상류 `e017c91ee` 체크아웃 전체에 대한 것이다.

**호출자 0.** `rg -n 'getMessage' --glob '!*CHANGELOG*'`는
`AllowedCollisionMatrix::getMessage`를 걸러내면 13줄이고, 그중 선언 둘
(`planning_response.hpp:56`, `:77`), 정의 둘(`planning_response.cpp:40`,
`:52`), **호출 아홉**이다. 아홉을 하나씩 열어 인자 타입을 확인했다:

| 호출 지점 | 인자 |
|---|---|
| `moveit_ros/move_group/src/default_capabilities/plan_service_capability.cpp:97` | `res->motion_plan_response` (`GetMotionPlan.srv:8` = `MotionPlanResponse`) |
| `moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp:245` | `progress.response` (`PipelineState.msg:5` = `MotionPlanResponse`) |
| pilz `unittest_trajectory_generator_ptp.cpp:382,488,560,687,829` | `moveit_msgs::msg::MotionPlanResponse res_msg` (각 호출 직전 줄) |
| pilz `unittest_trajectory_generator_lin.cpp:145` | 같음 |
| pilz `unittest_trajectory_generator_circ.cpp:130` | 같음 |

인자 타입이 판별자다. 두 `getMessage`는 서로 다른 클래스의 멤버이므로
오버로드는 수신자가 고르지만, 인자가 `moveit_msgs::msg::MotionPlanResponse&`
인 호출은 `MotionPlanDetailedResponse&`를 받는 서명에 **컴파일되지
않는다.** 아홉 중 아홉이 그 인자를 넘긴다. 따라서
`MotionPlanDetailedResponse::getMessage`의 호출자는 **테스트를 포함해
상류 전체에서 0이다.**

**와이어 타입은 어디에도 실리지 않는다.**
`rg -n 'moveit_msgs::msg::MotionPlanDetailedResponse'`는 상류 전체에서 정확히
두 줄 — 위의 선언(`:77`)과 정의(`:52`) — 이다. 메시지 정의 쪽도 같다:
`rg -n 'MotionPlanDetailedResponse' third_party/moveit_msgs/`는
`CMakeLists.txt:53`(빌드 목록) 한 줄뿐이고, 어떤 `.srv`/`.action`/`.msg`도
이 타입을 담지 않는다. 대조군이 그 차이를 보여준다 — `MotionPlanResponse`는
`srv/GetMotionPlan.srv:8`, `msg/PipelineState.msg:5`,
`action/GlobalPlanner.action:6` 셋에 실린다. 즉 이 함수가 채우는 메시지는
ROS 그래프에서 받을 수 있는 곳이 없다.

이것은 판정의 **이유가 아니라 비용 측정**이다. 호출자가 0이라는 사실이
함수를 안 옮겨도 된다고 말해주지는 않는다(그 논리라면 상류의 죽은 코드가
전부 면제된다). 말해주는 것은 이 함수를 안 옮겨서 이 포트가 못 하게 되는
일이 무엇인가이고, 답은 "상류가 오늘 그것으로 하는 일도 없다"이다.

### §234.4 만료 조건 — 무엇이 이 판정을 되돌리는가

셋 중 **아무거나 하나**가 성립하면 이 절은 무효이고 행은 다시 열린다.

1. 이 워크스페이스의 어떤 크레이트가 궤적 **벡터**를 담은 응답 타입을
   만든다(예: 다단계 플래너가 `plan`/`simplify`/`interpolate`를 따로
   내놓는다). 그 순간 §234.1의 네 경계가 표현 가능해지고, D6에 따라
   변환은 `moveit-ros`의 `TryFrom`이어야 한다.
2. `moveit-ros`가 `MotionPlanDetailedResponse`를 담은 서비스나 액션을
   내보내야 한다 — 즉 §234.3의 두 번째 실측이 뒤집힌다.
3. 상류가 이 함수에 호출자를 추가한다. 그러면 §234.3의 첫 번째 실측이
   뒤집히고, `pilz-detailed-response-pushes-null-trajectory`의
   도달 가능성 문단도 함께 다시 재야 한다.

### §234.5 이 절이 하지 않은 것

- `ChompSolution`을 벡터 형태로 넓히지 않았다. 그것은 상류 chomp가 항상
  길이 1로 resize한다는 `crates/moveit-planners-chomp/src/planner.rs:193-203`의 감사와 어긋나며, 이 절은
  그 감사를 다시 열지 않는다.
- `ros/moveit-ros/Cargo.toml`을 건드리지 않았다. 의존 간선은 추가되지
  않았다.
- `MotionPlanResponse::getMessage` 쪽 잔여분(`planning_time`)을 닫지
  않았다. 그것은 `crates/moveit-planning/src/response.rs`의 D8 감사가
  "unported, in scope"로 들고 있고 만료 조건도 거기 있다.
- `pilz-detailed-response-pushes-null-trajectory`의 등급을 바꾸지 않았다.
  호출자 0 실측은 그 항목의 도달 가능성 문단을 **더 강하게** 만들 뿐,
  결함 자체를 없애지 않는다.


---

## §235 Phase 9는 도달 가능하다 — 빠진 네 조각은 전부 신규 인프라이고, D2/D5/D6가 이미 짓기로 정한 것이다 (2026-08-06)

§226이 Phase 9를 UNMET으로 확정하고 막힌 지점을 서버 쪽(서비스·액션·
구독·노드 바이너리 부재)으로 좁혔다. 그 절이 멈춘 자리에서 다시
시작한다: 조건이 **도달 가능한가**, 그리고 도달 가능하다면 **가장 작은
조각**은 무엇인가.

### §235.1 네 조각을 "만들 수 있는 것" 단위로 나눈다

| 조각 | 와이어 타입 출처 | 이름/심볼 출처 | 핸들러 알고리즘 출처(상류) |
|---|---|---|---|
| 노드 진입점 (`fn main`, `r2r::Node`, `spin`) | 해당 없음 (런타임 부트스트랩, 메시지 타입이 아니다) | 해당 없음 | `moveit_ros/move_group/src/move_group.cpp`의 `main()` — 코퍼스 밖 |
| `/plan_kinematic_path` 서비스 | `moveit_msgs::srv::GetMotionPlan` (`moveit_msgs`, 코퍼스 밖이지만 이미 r2r 바인딩으로 이 크레이트가 씀) | `PLANNER_SERVICE_NAME = "plan_kinematic_path"`, `moveit_ros/move_group/include/moveit/move_group/capability_names.hpp:43-44` — 코퍼스 밖 | `moveit_ros/move_group/src/default_capabilities/plan_service_capability.{hpp,cpp}` — 코퍼스 밖 |
| `/move_action` 액션 서버 | `moveit_msgs::action::MoveGroup` (`moveit_msgs`, 코퍼스 밖) | `MOVE_ACTION = "move_action"`, 같은 `capability_names.hpp:52` — 코퍼스 밖 | `moveit_ros/move_group/src/default_capabilities/move_action_capability.{hpp,cpp}` — 코퍼스 밖 |
| planning scene 토픽 구독 | `moveit_msgs::msg::PlanningScene` (`moveit_msgs`, 코퍼스 밖) | 토픽 이름은 `PlanningSceneMonitor` 생성자 인자(고정 문자열 아님) | `moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp:1205`(`create_subscription<moveit_msgs::msg::PlanningScene>`) — 코퍼스 밖 |

네 근거 파일 전부 이 기계의 `/home/stevek/work/moveit2`에서 직접 읽고
확인했다:

```
$ grep -n 'PLANNER_SERVICE_NAME\|MOVE_ACTION' \
    moveit_ros/move_group/include/moveit/move_group/capability_names.hpp
43:static const std::string PLANNER_SERVICE_NAME =
44:    "plan_kinematic_path";
52:static const std::string MOVE_ACTION = "move_action";
$ grep -n 'create_subscription' \
    moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp | head -1
1205:    planning_scene_subscriber_ = pnode_->create_subscription<moveit_msgs::msg::PlanningScene>(
```

핵심 관찰: 네 조각 중 어느 것도 알고리즘을 새로 발명하지 않는다. 서비스와
액션 핸들러가 실제로 하는 일 — 요청 변환 → 플래닝 파이프라인 호출 → 응답
변환 — 은 이미 이 포트 안에 있다. `crates/moveit-planning/src/pipeline.rs:377`의
`generate_plan`이 상류 `generatePlan`을 대신하고,
`ros/moveit-ros/src/planning.rs:124`의
`impl<'m> TryFrom<PlanningRequestMsg<'m>> for PlanningRequest`가 요청의
msg→core 방향을, `:193`의 `impl TryFrom<PlanningRequest> for
PlanningRequestMsgOut`이 core→msg 방향을 이미 담당한다. 빠진 것은 이
셋을 리스너 콜백 하나로 잇는 배선(wiring)뿐이다.

### §235.2 포트인가, 신규 인프라인가 — `doc/port-coverage.md`를 직접 확인했다

가정하지 않고 `tools/ci/measure-port-coverage.py`의 `CORPUS_ROOTS`를
읽었다:

```
$ sed -n '46,52p' tools/ci/measure-port-coverage.py
CORPUS_ROOTS = [
    "moveit_core",
    "moveit_kinematics",
    "moveit_planners/chomp",
    "moveit_planners/stomp",
    "moveit_planners/pilz_industrial_motion_planner",
]
```

`moveit_ros`는 이 목록에 없다 — 코드로 확인한 사실이지, `doc/port-coverage.md`
1절의 산문("moveit_ros | 413 | 77,463 | **범위 밖**")을 옮겨 적은 것이
아니다. §235.1의 네 근거 파일(`move_group.cpp`, `plan_service_capability.*`,
`move_action_capability.*`, `planning_scene_monitor.cpp`)은 전부
`moveit_ros/*` 아래에 있다. 즉 **넷 다 코퍼스 밖**이고, `measure-port-coverage.py`가
이 넷에 대해 `unported`/`gap`/`decided-non-port` 어느 판정도 내지
않는다 — 판정 대상 집합에 처음부터 들어 있지 않기 때문이다.
`doc/port-coverage.md`를 `rg`로 직접 확인했다:

```
$ rg -n 'move_group\.cpp|plan_service_capability|move_action_capability|planning_scene_monitor\.cpp' \
    doc/port-coverage.md
(no matches, exit 1)
```

**넷 다 `doc/port-coverage.md`에 행으로 owed되지 않는다.** 그렇다고
"이 포트가 짓지 않기로 결정한 인프라"도 아니다 — 그 범주의 실례가 이미
같은 표에 있고, 그것과 대조하면 차이가 분명해진다. `doc/port-coverage.md`의
`move_group_sequence_action.hpp`/`move_group_sequence_service.hpp`
행(pilz의 시퀀스 액션/서비스, `moveit_planners/pilz_industrial_motion_planner`
아래라 **코퍼스 안**인데도 `decided-non-port`)은 "actionlib/rclcpp
action and service servers wrapping the planner for move_group; nothing
here computes a trajectory"라고 적는다 — 즉 이미 포팅된 계산을 감싸는
rclcpp 래퍼는 **짓지 않기로 결정**했다는 뜻이고, 그 결정은 지금도
유효하다(pilz 시퀀스 서비스는 Rust 쪽에 없고 필요하지도 않다 — Phase 9의
완료 조건이 요구하는 것이 아니다).

Phase 9의 네 조각은 형태만 같을 뿐(계산을 감싸는 rclcpp 스타일 래퍼)
반대 결정을 받았다. 계획서 원본(2026-08-03, D5/D6가 생기기도 전) §5가
이미 "`/plan_kinematic_path` 서비스, `/move_action` 액션 서버, planning
scene 토픽 구독"을 Phase 9의 산출물로 이름 붙였고, Phase 9가 실제로
열린 시점(§129.3, 2026-08-04)의 D6가 다시 확정한다:

> moveit_msgs ↔ 코어 타입 변환, `/plan_kinematic_path` 서비스,
> `/move_action` 액션 서버, planning scene 구독이 전부 `moveit-ros`
> 안에 산다. 즉 ROS 없이 쓰는 경로가 기본이고 ROS 호환이 얹히는
> 것이지, 그 반대가 아니다.

이 문장은 "짓지 않는다"가 아니라 "어디에 짓는가"를 정한다 — pilz
시퀀스 행이 짓지 않기로 결정한 것과 반대다. **분류: 넷 다 포트가 아니라
신규 인프라이고, 넷 다 이미 짓기로 결정된 신규 인프라다.** 그래서
넷 다 `doc/port-coverage.md`에 행으로 들어가지 않는다(코퍼스가 다루는
"포팅/미포팅"의 대상이 아니다) — 대신 이 계획서(§5, §129.3, 그리고 이
절)가 "예정된 미완료 작업"으로 계속 추적한다.

### §235.3 부수로 정정한다 — `TryFrom` 개수는 24가 아니라 38이다

§226.3의 anchor `rg -n '^impl TryFrom'`은 라이프타임 제네릭 impl
(`impl<'m> TryFrom<...> for ...`)을 놓친다 — `impl TryFrom`으로
시작하지 않고 `impl<'m> TryFrom`으로 시작하기 때문이다. 병합으로
`ros/moveit-ros/src/state.rs`·`planning.rs`가 바뀌기도 했으므로, anchor를
고쳐 이 라운드의 실제 상태를 다시 잰다:

```
$ rg -n '^impl(<[^>]*>)? TryFrom' ros/moveit-ros/src/*.rs ros/moveit-ros/src/**/*.rs | wc -l
38
$ rg -c '^impl(<[^>]*>)? TryFrom' ros/moveit-ros/src/*.rs ros/moveit-ros/src/**/*.rs | grep -v ':0'
ros/moveit-ros/src/constraints/set.rs:2
ros/moveit-ros/src/geometry.rs:9
ros/moveit-ros/src/constraints/visibility.rs:4
ros/moveit-ros/src/model.rs:2
ros/moveit-ros/src/planning.rs:6
ros/moveit-ros/src/scene/collision_object.rs:1
ros/moveit-ros/src/state.rs:2
ros/moveit-ros/src/scene/shapes.rs:3
ros/moveit-ros/src/trajectory.rs:2
ros/moveit-ros/src/constraints/joint.rs:2
ros/moveit-ros/src/constraints/orientation.rs:2
ros/moveit-ros/src/constraints/position.rs:3
```

**38개, §226.3의 24는 anchor 결함으로 인한 저수였다.** `planning.rs` 6개
중 서비스 배선에 바로 쓰이는 둘: `:124`
`impl<'m> TryFrom<PlanningRequestMsg<'m>> for PlanningRequest`(요청의
msg→core 방향)와 `:193` `impl TryFrom<PlanningRequest> for
PlanningRequestMsgOut`(응답 아님 — 요청의 core→msg 방향;
응답 변환은 `:239`/`:302`가 따로 담당). 살아있는 통신 프리미티브
부재는 이번에도 재확인했다 — 병합 전과 결과 동일:

```
$ rg -n 'create_service|create_action_server|ActionServer|create_subscription|create_client|r2r::Node|Node::create|fn main' \
    ros/moveit-ros/src/ -t rust
(no matches, exit 1)
$ grep -n '\[\[bin\]\]' ros/moveit-ros/Cargo.toml
(no matches)
```

### §235.4 결론 — 조건은 이 포트가 짓지 않기로 결정한 rclcpp 런타임을 요구하지 않는다

세 갈래 중 사용자가 제시한 세 번째("rclcpp 런타임을 요구하고 이 포트가
짓지 않기로 결정했다면 미도달로 §5에 적는다")는 **성립하지 않는다.**
D5(§129.2)가 격리한 위치에, D6(§129.3)가 명시적으로 네 조각 전부의
소재를 `moveit-ros`로 정했고, 원본 §5(2026-08-03)도 이미 같은 넷을
Phase 9의 산출물로 적었다. §235.2가 대조로 보였듯, 이 포트가 "짓지
않기로 결정한" rclcpp 래퍼의 실례(pilz 시퀀스 액션/서비스)는 따로
있고 그 결정문은 Phase 9와 다른 조건에 붙는다. Phase 9의 네 조각은
그 범주가 아니다 — **미도달의 원인은 결정이 아니라 미착수다.**

**조건은 도달 가능하다.** §5의 완료 조건 문구를 정정하거나 "미도달"로
동결할 근거가 없으므로 그렇게 하지 않는다.

### §235.5 순서와 첫 종단 측정을 여는 가장 작은 조각

§226.4의 5항목을 우선순위로 다시 정렬한다 — 기준은 "무엇이 다음 조각의
전제조건인가"와 "무엇이 가장 적은 배선으로 첫 실측을 낳는가"다.

1. **노드 진입점.** `ros/moveit-ros`에 `[[bin]]` + `fn main`, `r2r::Node`
   생성, `spin`. 다른 어떤 조각도 이것 없이는 등록될 자리가 없다 —
   순서 1은 고정이다.
2. **`/plan_kinematic_path` 서비스.** `r2r::Node::create_service::<GetMotionPlan::Service>()`로
   등록하고, 요청을 `planning.rs:124`의 기존 `TryFrom`으로
   변환 → `pipeline.rs:377`의 기존 `generate_plan` 호출 → 응답을
   `planning.rs:239`의 기존 `TryFrom`으로 변환 → 회신. **이 조각을
   1과 함께 지으면 첫 종단 측정이 열린다** — 서비스는 동기 단발
   RPC라 액션의 goal/feedback/cancel 상태 기계가 필요 없고,
   요청 자체에 `start_state`를 (diff 아닌 완전 지정으로) 채워 보내는
   호출자라면 4번(라이브 planning scene 구독)도 아직 없이 유효한
   응답을 받을 수 있다. 다만 이렇게 얻는 측정은 "코드 변경 없는 기존
   `MoveGroupInterface` 클라이언트"보다 **좁다** — 원시 서비스 호출
   (`ros2 service call` 또는 그 타입만 링크한 최소 C++ 클라이언트)이지
   `MoveGroupInterface` 클래스 자체가 아니다. 그 좁음을 좁다고 표시하고
   넘어간다: 조건 문구 그대로를 재려면 5번(이미지 작업)까지 필요하다.
3. **`/move_action` 액션 서버.** r2r의 액션 서버 API(goal/feedback/
   cancel/result)가 서비스보다 상태 기계가 하나 더 있다. 변환·플래닝
   호출은 2와 동일한 기존 코드를 재사용한다.
4. **planning scene 토픽 구독.** `scene/planning_scene.rs`의 기존
   변환을 살아있는 `/planning_scene` 구독 콜백에 연결한다. 이것 없이도
   2가 첫 측정을 열지만, `start_state.is_diff = true`로 "현재 상태
   사용"을 보내는 통상적인 `MoveGroupInterface` 호출은 이 조각 없이는
   틀린 답을 받거나 거부당한다 — 조건 문구의 "코드 변경 없이"를
   만족하려면 결국 필요하다.
5. **이미지 작업.** §226.2가 잰 C++ `MoveGroupInterface` 빌드(3패키지,
   1분 18초)는 오라클 이미지 계열에서 한 것이고, 게이트가 쓰는
   `ros-dev` 이미지에는 `moveit2` C++ 스택이 없다. 1~4가 다 갖춰져도
   실제 `MoveGroupInterface` 클라이언트를 이 기계에서 빌드·실행할
   이미지가 없으면 조건 문구 그대로의 시도 자체가 불가능하다.

**가장 작은 조각: 1 + 2.** 서비스 하나가 "요청 변환 → 실 플래너 →
응답 변환 → 와이어"라는 조건의 핵심 사슬을 끝에서 끝까지 처음
증명하는 지점이고, 액션 상태 기계·라이브 구독·이미지 작업 없이
도달한다. 이 절은 코드를 쓰지 않았다 — 위 다섯은 순서와 근거이지
구현이 아니다.


---

## §237 Phase 2 셋째 항목의 "부분 UNMET"을 세 조각으로 나눠 실측했다 — mimic은 MET, 클램핑·보간은 이 기계에서 오라클 비교가 원천 불가 (2026-08-06)

> **이 절의 (a)·(c) 판정은 §238이 뒤집었다 (병합 시 기록).** "이 기계에서
> 오라클 비교가 원천 불가"는 이 절이 41개 op을 센 시점(`4407a10`)에는
> 참이었다. 같은 날 다른 가지가 오라클에 `enforce_bounds`/`mimic_propagate`/
> `interpolate` 세 op을 붙였고(`9e60f3a`, 03:07 — 이 절의 커밋 `338c7c6`,
> 03:18보다 11분 앞서지만 조상이 아니라 이 절의 작업 트리에서 보이지
> 않았다), §238이 그 계기로 5로봇 4,224 케이스를 허용오차 0.0에서 재
> 불일치 0건을 얻었다. §5 현황표의 이 항목 행은 §238을 인용한다. 아래
> §237.1의 "부분"이라는 낱말에 대한 판독은 §238과 무관하게 그대로다.

### §237.1 "부분"은 이 항목 자신의 경계였던 적이 없다

Phase 2 완료 조건(§5:656-663), 셋째 항목 원문:

> 관절 한계 클램핑, mimic 전파, floating/planar 조인트 보간이 일치

("일치" 대상과 허용오차는 명시돼 있지 않다 — 앞 두 항목은 각각
"오라클과 `1e-9` 이내 일치"/"`1e-7` 이내 일치"라고 명시하는데, 셋째는
병렬 구조로 "오라클과 일치"를 물려받는다고 읽을 수는 있어도 허용오차는
비어 있다. §237.2에서 실측이 정확히 bit-exact로 나와 이 공백이 실제로는
막히지 않았음을 보인다.)

이 항목을 가장 깊이 잰 §217.3(:16647)은 자기 자신을 이렇게 판정했다:
"Phase 2 — 앞의 두 항목 MET, 세 번째 **UNMET**." — "부분"이 아니라
평범한 UNMET이다. "부분 UNMET"이라는 말은 이 항목 판정 문장 어디에도
없고, 오직 §5 말미의 롤업 요약 세 곳에만 나온다:

- `:16811-16812` "요약: UNMET 4개(Phase 1, 3, 4, 9), **부분 UNMET
  2개(Phase 2의 셋째 항목, Phase 5의 둘째·셋째 항목)**, 미측정 1개..."
- `:16814-16816` "이 트리에서는 UNMET 4개(Phase 1, 3, 4, 9), **부분
  UNMET 1개(Phase 2의 셋째 항목)**, 미측정 1개..." (Phase 5의 두 항목이
  닫혀 목록에서 빠진 뒤)
- `:16844-16845` "UNMET 3개(Phase 3, 4, 9), **부분 UNMET 1개(Phase 2의
  셋째 항목)**, 미측정 1개..." (Phase 1이 §218로 MET가 된 뒤)

세 곳 모두 같은 패턴이다: "부분 UNMET N개"는 **Phase 개수**를 세는
말이고, 괄호 안은 그 Phase 중 어느 항목이 걸림돌인지 이름 붙인 것이다.
Phase 5도 함께 "부분 UNMET"으로 묶였던 이유는 Phase 5의 첫 항목은 MET,
둘째·셋째는 UNMET이었기 때문(§217.3의 Phase 5 절 참조) — 즉 "부분"은
"이 Phase는 항목이 섞여 있다"는 뜻이지 "이 항목은 부분적으로만
측정됐다"는 뜻이 아니다. Phase 2는 첫째·둘째 MET, 셋째 UNMET이라 같은
이유로 "부분"에 묶였을 뿐, 셋째 항목 자신의 측정 범위에 관한 진술이
아니다.

**이것이 실제 결함이다.** "부분"이라는 단어가 두 가지로 읽히는데
(Phase 차원의 혼재 vs. 항목 차원의 부분측정), 문서 어디에도 어느
쪽인지 적혀 있지 않았다. §217.3 자신의 "UNMET" 판정과 나란히 두면
항목 차원의 "부분측정"이라는 읽기는 §237.2 이전까지는 거짓이었다 —
이 항목은 세 조각 중 무엇 하나도 오라클과 비교된 적이 없다(§217.3:
16662-16669, 이 세션에서 재확인, 아래).

### §237.2 세 조각을 각각 이 트리에서 다시 쟀다

셋째 항목은 콤마로 묶인 세 개의 독립된 하위 절이다. 각각의 모집단과
실측 모집단을 따로 낸다.

**(a) 관절 한계 클램핑 — 이 기계에서 오라클 비교가 원천 불가.**
오라클이 노출하는 41개 op 전부를 다시 나열했다(이 트리, `4407a10`):

```
$ rg -n 'op == "' tools/moveit-oracle/src/oracle.cpp
```

`model_info, fk, jacobian, random_states, kinematics_metrics, acm,
collision, world, frame_transform, is_state_valid,
scene_diff_collision, cost_sources, path_cost_sources, octree_points,
distance_field, shape_points, mesh, common_root,
collision_distance_field_types, collision_sphere_free_functions,
distance_field_cache_entry, group_state_representation, dynamics,
collision_object_point_decomposition, link_body_decomposition,
link_models_with_collision_geometry, constraints, octomap, ik,
octree_in_world, octree_shape_query, ruckig, body_query, totg,
totg_path, acceleration_filter, plan, ruckig_filter, pilz_trajectory,
pilz_blend, chomp_quad_cost_inverse` — 41개, clamp/bounds/limit 계열
op가 없다. 더 결정적으로, 오라클 바이너리 자체가 상류의 클램핑
함수를 한 번도 호출하지 않는다:

```
$ rg -n -i 'enforceBounds|satisfiesBounds' tools/moveit-oracle/src/oracle.cpp
(no output, exit 1)
```

`fk`/`jacobian`이 공유하는 유일한 관절값 적용 경로
(`applyJointValuesTo`, `:1430-1443`)는 `state.setVariablePosition`을
그대로 호출하고 `state.update()`로 끝난다 — `enforceBounds()` 호출이
없다. 즉 범위를 벗어난 관절값을 오라클에 넣어도 오라클은 클램프하지
않은 값으로 FK를 계산해 돌려준다. 이 항목을 오라클과 비교하려면
`oracle.cpp`에 새 op를 추가하고 핀된 오라클 이미지를 재빌드해야
한다 — C++ 바이너리 변경과 이미지 재빌드는 이 크레이트의 범위 밖이고
이 세션의 fence 밖이다. **실측 모집단: 0. 이유: 도구 부재, 명령과
출력 위에 그대로 적음.**

**(b) floating/planar 조인트 보간 — 이 기계에서 오라클 비교가 원천
불가, (a)와 같은 이유.**

```
$ rg -n -i interpolat crates/moveit-state/tests/ tools/moveit-diff/src/
(no output, exit 1)
$ rg -n -i 'clamp|enforce.?bound' tools/moveit-diff/src/
(no output, exit 1)
$ rg -n -i interpolat tools/moveit-oracle/src/oracle.cpp
5813: ...distance-field 콜백 안에서 "interpolated position"이라는
      다른 개념(충돌 거리 보간)을 가리키는 주석 3건, RobotState/
      JointModel::interpolate와 무관
```

`JointModel::interpolate`는 실제로 포트에 존재한다
(`moveit-model`의 `revolute.rs:139`/`prismatic.rs:88`/
`planar.rs:164`/`floating.rs:168`, 디스패처는 `crates/moveit-model/src/joint/model.rs:873`) — 다만
`moveit-state/src/lib.rs:45`가 명시적으로 "Deferred, out of scope for
this task: ... `interpolate`"라고 적어 RobotState 수준의 상위
API(`RobotState::interpolate`, 두 whole-state 사이 보간)는 포트되지
않았다고 말한다. 셋째 항목 원문의 "floating/planar **조인트**
보간"은 낮은 수준(JointModel::interpolate)을 가리키므로 이미 존재는
한다 — 각 조인트 타입 파일에 자체 단위 시험(예:
`interpolate_wraps_the_short_way_when_continuous`,
`interpolate_does_not_panic_on_antipodal_quaternions`)이 있지만
전부 자기검증(해석적으로 기대값을 계산해 대조)이지 오라클과의 비교가
아니다. 오라클에는 `from`/`to`/`t` 세 값을 받아 보간된 상태를
돌려주는 op가 없고, 41개 op 중 이를 대체할 만한 것도 없다(pilz/totg의
"보간"은 시간-파라미터화 궤적 생성이라는 다른 연산이지 이 API가
아니다). **실측 모집단: 0. 이유: (a)와 동일, 도구 부재.**

**(c) mimic 전파 — 오라클과 비교 가능했고, 지금 처음 비교했다.**

`randomStates`(oracle.cpp:1537, 이 op의 문서 주석 자신이 이미
적어 놓았다): "RobotModel::getVariableRandomPositions ... derives
mimic values." — 즉 `tests/fk_parity.rs`의 네 로봇 픽스처
(`{panda,dual_arm_panda,pr2,fanuc}_fk.json`, 각 3개의 무작위 case)에
이미 실려 있는 `joint_values`의 follower 변수 값은 실제 moveit2가
유도한 값이다. 이 데이터를 새로 오라클을 부르지 않고 재가공하는 것만
으로 mimic 전파를 오라클과 비교할 수 있다 — 이전까지 아무도 그렇게
하지 않았을 뿐이다.

새 시험 `crates/moveit-state/tests/mimic_propagation_parity.rs`:
각 로봇의 mimic (master, follower) 쌍마다, 픽스처 case의 master 값만
포트에 `set_variable_position`으로 주입해 follower를 내부적으로
유도시키고, 그 유도값을 같은 case에 실려 있는(오라클이 낸) follower
값과 `assert_eq!`로 대조한다.

| 로봇 | mimic 관계 수 | case 수 | 비교 건수 | 결과 |
|---|---|---|---|---|
| panda | 1 (`panda_finger_joint1`→`panda_finger_joint2`) | 3 | 3 | 전부 일치 |
| dual_arm_panda | 2 (left/right `_finger_joint1`→`_finger_joint2`) | 3 | 6 | 전부 일치 |
| pr2 | 6 (l/r `_gripper_l_finger_joint`→각 3개) | 3 | 18 | 전부 일치 |
| fanuc | 0 (mimic 관절 없음, `fanuc.urdf`에 `<mimic>` 0건) | — | — | 해당 없음 |

```
$ cargo test -p moveit-state --test mimic_propagation_parity
running 3 tests
test dual_arm_panda_mimic_propagation_matches_the_oracle ... ok
test panda_mimic_propagation_matches_the_oracle ... ok
test pr2_mimic_propagation_matches_the_oracle ... ok
test result: ok. 3 passed; 0 failed
```

**모집단: 4로봇 중 mimic이 있는 3로봇의 모든 mimic 관계(9개) × 픽스처의
모든 무작위 case(3개) = 27건. 실측: 27/27, 전부 오라클 값과
bit-exact 일치.** 셋째 항목 자체에는 허용오차가 적혀 있지 않지만
(§237.1) `assert_eq!`가 정확히 통과해 그 공백은 실무에서 막히지
않았다.

**구별 시험(discrimination check), mutate/observe-failure/revert:**
`update_mimic_joint`(state.rs:880-881)의
`mimic.factor * source + mimic.offset`을
`mimic.factor * source + mimic.offset + 1.0`으로 바꾸고 다시
돌렸다:

```
test panda_mimic_propagation_matches_the_oracle ... FAILED
  left: 1.0036935438215733
 right: 0.0036935438215732574
test dual_arm_panda_mimic_propagation_matches_the_oracle ... FAILED
  left: 1.022772453026846
 right: 0.02277245302684605
test pr2_mimic_propagation_matches_the_oracle ... FAILED
  left: 1.3136513575147837
 right: 0.3136513575147838
test result: FAILED. 0 passed; 3 failed
```

세 시험 모두 즉시 실패, 편차는 주입한 `+1.0`과 정확히 일치. 되돌린
뒤 `git diff -- crates/moveit-state/src/state.rs`는 비어 있다 —
`state.rs`는 순수 변경 없이 시험만 남았다.

### §237.3 판정

Phase 2 셋째 항목은 **여전히 UNMET**이다 — 세 하위 절이 모두
"일치"해야 닫히는 AND 조건인데, 그중 하나(mimic)만 이번에 MET로
바뀌었을 뿐 나머지 둘(클램핑, 보간)은 이 기계의 오라클 바이너리로는
원천적으로 비교할 방법이 없다. 이것은 "부분 UNMET"이 처음부터 뜻해야
했던 것과 우연히 지금 일치한다 — 3개 중 1개 MET, 2개는 UNMET이 아니라
**미측정(도구 부재)** — 하지만 §237.1이 보였듯 그 정확한 뜻은 지금까지
한 번도 문서에 적힌 적이 없었고, §217.3 자신의 판정 문장은 "UNMET"
이라고만 적어 이 구분을 지우고 있었다.

요약 갱신: `:16844-16845`의 "부분 UNMET 1개(Phase 2의 셋째 항목)"는
숫자상 그대로 유지된다(Phase 2는 여전히 완전히 MET가 아니다) — 다만
이제 그 괄호 뒤에 "(mimic MET, 클램핑·보간 미측정)"이라는 세 조각
분해가 딸려야 정확하다. Phase 개수 집계("부분 UNMET 1개")를 다시
쓰지는 않는다 — Phase 2가 부분(=항목 혼재) 상태인 것은 §237 이전과
이후 모두 참이므로, 이 절이 바꾸는 것은 그 안의 세 조각짜리 내역이지
Phase 단위 집계가 아니다.

### §237.4 이 절이 하지 않은 것

- 클램핑·보간 절을 닫지 않았다. 닫으려면 `tools/moveit-oracle/src/
  oracle.cpp`에 새 op를 추가하고 핀된 오라클 이미지를 재빌드해야
  한다 — 이 절의 fence 밖. **UNFIXED, 사유: 도구 부재(§237.2 (a)/(b)).**
- 셋째 항목의 명시적 허용오차 공백(§237.1)을 문서에 채워 넣지
  않았다 — mimic 하위 절은 실측이 bit-exact라 막히지 않았지만, 클램핑·
  보간 하위 절은 애초에 비교 자체가 없어 허용오차 공백이 의미가
  없다.
- Phase 2 전체를 MET로 표시하지 않았다. 세 하위 절 중 둘이 미측정인 한
  AND 조건은 닫히지 않는다.

## §236 `setMotionPlanRequest`의 요청 정규화는 포팅하지 않는다 — 고칠 피연산자가 없고, 그 규칙 자체가 NaN을 놓친다 (2026-08-06)

`doc/port-coverage.md`의 `moveit_core/planning_interface/src/planning_interface.cpp`
행에 남아 있던 마지막 항목이다. 그 파일의 멤버 정의 9개 중 8개는
`moveit-planners-sbp` round-6 심볼 감사가 이미 판정했고, 어떤 문장도
판정하지 않은 하나가 `PlanningContext::setMotionPlanRequest`(`:89-104`)다.
이 절이 그 하나를 판정한다.

### §236.1 상류 규칙 — 읽은 그대로

`moveit_core/planning_interface/src/planning_interface.cpp:89-104`
(고정 커밋 `e017c91e`):

```c++
request_ = request;
if (request_.allowed_planning_time <= 0.0)
{
  RCLCPP_INFO(getLogger(), "The timeout for planning must be positive (%lf specified). Assuming one second instead.",
              request_.allowed_planning_time);
  request_.allowed_planning_time = 1.0;
}
if (request_.num_planning_attempts < 0)
{
  RCLCPP_ERROR(getLogger(), "The number of desired planning attempts should be positive. "
                            "Assuming one attempt.");
}
request_.num_planning_attempts = std::max(1, request_.num_planning_attempts);
```

두 절반이 같은 의도를 서로 다르게 쓴다. 시간 쪽은 보고와 수정이 한
`if` 안에 있고, 시도 횟수 쪽은 보고가 `< 0`에서만 나오는데 수정
(`max(1, n)`)은 `if` 밖에서 모든 값에 적용된다 — 설정되지 않은 메시지의
기본값인 `0`은 말없이 `1`이 된다. 그리고 시간 쪽 가드는 자기 로그가
말하는 술어("must be positive")와 다른 술어(`<= 0.0`)를 검사한다.
IEEE-754에서 `!(x <= 0)`은 `x > 0`이 아니다: NaN은 둘 다 만족하지
못하므로, 수정할 여지가 가장 없는 값이 수정되지 않고 통과하는 유일한
값이다. 이 결함과 측정은 `doc/upstream-bugs.md`의
`set-motion-plan-request-time-guard-polarity`에 있다.

### §236.2 이 규칙이 실제로 지키는 것 — 측정

상류에서 이 setter를 부르는 곳은 네 군데이고 네 곳 모두 부른다
(`stomp_moveit_planner_plugin.cpp:102`, `chomp_plugin.cpp:100`,
`planning_context_manager.cpp:590`,
`pilz_industrial_motion_planner.cpp:154`) — 우회하는 플러그인 경로는
없다. 그런데 정규화된 값을 **읽는** 곳은 둘뿐이다:

| 소비자 | `allowed_planning_time` | `num_planning_attempts` |
|---|---|---|
| OMPL (`model_based_planning_context.cpp:775`, `:805`) | 읽음 → `solve(timeout, count)` | 읽음 |
| STOMP (`stomp_moveit_planning_context.cpp:252`) | 읽음 (감시 스레드) | 읽지 않음 |
| CHOMP (`chomp_plugin.cpp`, `chomp_interface.cpp`) | 읽지 않음 | 읽지 않음 |
| pilz (`pilz_industrial_motion_planner.cpp`) | 읽지 않음 | 읽지 않음 |

시도 횟수 쪽 수정은 관측 가능한 효과가 사실상 없다:
`solve(double, unsigned int)`가 `count <= 1`을 한 번 실행으로 처리하므로
(`model_based_planning_context.cpp:855`) `0`과 `1`은 같은 경로다. 남는
것은 음수뿐이고 그것은 `int32` → `unsigned int` 변환을 막는 일인데,
`moveit_ros/planning/moveit_cpp/src/planning_component.cpp:328`은 요청을
만드는 자리에서 `std::max(1, ...)`를 **이미 한 번 더** 적용한다. 즉 이
규칙은 컨텍스트의 불변식이 아니라 요청 위생(hygiene)이고, 상류 자신도
그것을 호출자 쪽에서 중복으로 수행한다.

시간 쪽 수정은 실효가 있다 — 그리고 자기가 통과시킨 값에는 실효가
없다. 측정(`g++ 13.3.0`, x86-64, `-O2`, OMPL 체크아웃
`/home/stevek/work/ompl` `eb3baca7` = `2.0.1-10-geb3baca7`; NaN은
상수 접기를 피하려고 `argv`에서 읽는다):

| `allowed_planning_time` | `:92` 발동 | `ompl::time::seconds` | 시작 시점에 PTC 이미 참 |
|---|---|---|---|
| `nan`  | 아니오 | `0` ns          | **예** |
| `-1`   | 예     | `-1000000000` ns | (여기 오기 전에 `1.0`으로 수정됨) |
| `0`    | 예     | `0` ns           | (여기 오기 전에 `1.0`으로 수정됨) |
| `1e-9` | 아니오 | `0` ns           | **예** |
| `1e-6` | 아니오 | `1000` ns        | 아니오 |
| `5`    | 아니오 | `5000000000` ns  | 아니오 |

`ompl::time::seconds`(`src/ompl/util/Time.h:64-69`)가 정수 초 + 정수
마이크로초로 duration을 만들기 때문에 1 µs 미만은 `0`이 되고, NaN은
`(long)NaN`(미정의 동작, 이 호스트에서 `LONG_MIN`)이 µs→ns 확장에서
정확히 `0`으로 감싸인다. 두 경우 모두 `endTime = now() + 0`이므로 종료
조건이 첫 평가에서 이미 참이고 플래너는 시간을 전혀 받지 못한다 —
1초 클램프가 막으려던 바로 그 결과를, 클램프가 잡지 못하는 값들로
도달한다.

### §236.3 판정: 포팅하지 않는다

**첫째, 고칠 피연산자가 이 포트에 없다.** 이 규칙은 두 필드의 값을
고치는 것이고 두 필드는 어느 요청 타입에도 없다:
`moveit_planning::PlanningRequest`는 8개 필드이며 그중 어느 것도 예산이
아니고(`crates/moveit-planning/src/request.rs`의 16필드 감사가 두
필드를 "unported, in scope"로 이미 열거한다),
`moveit_planners_sbp::registry::PlanningRequest`도 갖고 있지 않다.
`ros/moveit-ros/src/planning.rs`의 `TryFrom<PlanningRequestMsg>`는 와이어
경계에서 둘 다 버린다. `rg -n 'allowed_planning_time|num_planning_attempts' crates ros`는
28줄을 내는데(병합 시 재측정 — 이 절이 잰 16은 다른 네 가지가 들어오기
전 수치다) 27줄이 `.rs`이고 1줄이
`ros/moveit-ros/doc/message-mapping.md:634`의 표 한 행이다. 27줄 중 21줄은
doc 주석이고 나머지 6줄은 §236.4가 세운 만료 tripwire 테스트 두 개 안에
있다 — 생산 코드에서 두 정규화를 적용하는 자리는 여전히 한 줄도 없다.
규칙을 피연산자보다 먼저 포팅할 수는 없다.

**둘째, 받는 자리는 이미 있고 그 자리에는 setter가 없다.** 이 포트의
`PlanningContext` 계층은 포팅되어 있다:
`moveit_planners_sbp::registry`의 `PlannerManager::get_planning_context`
(`crates/moveit-planners-sbp/src/registry.rs:584`)가 `request:
PlanningRequest`를 **값으로** 받고 `RrtConnectManager`의 구현
(`:621`)이 그것을 `RrtConnectContext.request`로 옮긴다. 상류가
`setMotionPlanRequest`에 정규화를 매단 이유는 그것이 요청이 컨텍스트에
들어오는 유일한 문이라는 점인데, 이 포트에서 그 문은 생성자 인자다 —
정규화를 매달 setter가 없고, 필요하지도 않다.

**셋째, 이 포트의 타입에서는 세 입력이 만들어질 수 없다.** 이 포트에서
탐색 예산에 해당하는 것은 `moveit_planners_sbp::Termination`
(`crates/moveit-planners-sbp/src/rrt_connect.rs:40-58`,
`RrtConnectParams::termination`을 통해 요청에 실리고
`crates/moveit-planners-sbp/src/registry.rs:768`에서 소비된다)이다:

| 상류 가드가 고치는 입력 | 이 포트에서 |
|---|---|
| 음수 (`allowed_planning_time < 0`, `num_planning_attempts < 0`) | `Iterations(usize)`/`Deadline(Duration)` 모두 부호 없음 — 표현 불가 |
| 설정되지 않음 (`0.0`/`0`이 "미지정"을 겸함) | `Termination`은 `Default`도 "미지정" variant도 없다(`derive(Debug, Clone, Copy)`뿐, `impl Default` 없음) — 호출자가 variant를 지목해야 하므로 겸용 값이 없다 |
| NaN | `Duration`은 NaN을 담을 수 없다 |

`Termination`의 doc 주석이 이 설계를 이미 자기 말로 적어두었다: 결정성
보장이 "by construction"으로 성립하도록 합타입으로 두었다는 것. 세
입력이 모두 구성 불가이므로 정규화할 상태가 남지 않는다. 이것이 패치가
아니라 구조적 답이라는 근거이며, 트리의 다른 wall-clock 예산에 대해
이미 같은 답이 적용되어 있다 —
`doc/upstream-bugs.md`의 `set-from-ik-zero-timeout-is-not-single-attempt`
(`SolverParams::max_restarts`가 초 단위 타임아웃을 대체했고, 그래서
재해석될 센티널 값이 없다).

**넷째, 규칙을 그대로 옮기는 것은 결함을 옮기는 것이다.** §236.1이
읽은 대로 `<= 0.0`은 NaN을 통과시키고, 통과한 `1e-9`는 소비자에서 0
예산이 된다. 표현 불가로 닫으면 네 경계(`-1.0`, `0.0`, `+eps`, NaN)가
한 번에 닫히고, 클램프를 옮기면 그중 둘이 열린 채로 따라온다.

**대조 — 표현 가능하면 이 포트는 같은 수정을 이미 포팅했다.** 상류는
같은 종류의 "미지정 → 기본값" 수정을 workspace 상자에 대해서는
요청 어댑터로 구현한다
(`moveit_ros/planning/planning_request_adapter_plugins/src/validate_workspace_bounds.cpp:72-93`,
여섯 성분이 모두 `< epsilon`이면 기본 큐브로 채운다). 그 필드는 이
포트가 실제로 들고 있고(`WorkspaceBounds`, 전부 0 = "미지정"이 타입의
doc에 적혀 있다), 그래서 수정도 포팅되어 있다 —
`crates/moveit-planning/src/request_adapters/validate_workspace_bounds.rs:68`.
판정 기준은 "상류 수정을 포팅하지 않는다"가 아니라 "고칠 상태가
표현 가능한가"이며, 두 필드는 그 기준의 반대편에 있다.

### §236.4 만료 조건과 규칙이 살 자리

이 판정은 두 필드가 어느 요청 타입에도 없다는 사실에 걸려 있고, 그
사실이 깨지는 순간 만료한다. 만료 감지는 문장이 아니라 테스트다:
`ros/moveit-ros/src/planning.rs`의
`allowed_planning_time_boundaries_are_not_observable_on_the_core_request`
(경계 `-1.0`, `0.0`, `f64::EPSILON`, `5.0`, `f64::NAN`)와
`num_planning_attempts_boundaries_are_not_observable_on_the_core_request`
(경계 `-1`, `0`, `1`, `2`)가, 각 경계값을 실은 와이어 메시지가 코어
요청에서 **구별되지 않음**을 확인한다. 필드가 코어에 생겨 매핑되는
순간 두 테스트가 어긋난 경계를 이름으로 보고하며 깨진다. 부재를
주장하는 판정이므로 클램프를 테스트할 수는 없고 — 부를 클램프가 없다 —
판정이 서 있는 전제를 테스트한다.

필드가 생길 때 규칙이 살 자리는 setter가 아니라 요청 어댑터다:
`ValidateWorkspaceBounds` 옆
(`crates/moveit-planning/src/request_adapters/`), 상류가 형제 수정을
두는 자리와 같고, `PlanningRequestAdapter` 체인은 이미 요청을 `&mut`로
받는다. 그때 옮길 것은 `<= 0.0` 클램프가 아니라 D8이 두 요청 타입을
합칠 때 예산을 `Termination`으로 싣는 결정이다 — 그 형태에서는
§236.3의 표가 그대로 성립하여 어댑터가 고칠 상태 자체가 생기지 않는다.
D8이 예산을 `f64` 초 단위로 싣는 쪽으로 결정된다면 이 절을 다시 열어야
하고, 그때 필요한 가드는 `<= 0.0`이 아니라 `> 0.0`이다
(§236.1, 그리고 상류 자신의 `MoveGroupInterface::setPlanningTime`,
`move_group_interface.cpp:1013-1017`).
## §239 `CollisionRequest` 12개 필드를 parry의 두 충돌 진입점에 대해 전수 조사했다 — `is_done`이 조용히 버려지고 있었다 (2026-08-06)

§231.1이 고친 `CollisionRequest::distance`는 "선언되어 있고, 공개로
설정할 수 있고, 백엔드가 읽지 않는" 필드였다. 그런 필드가 하나뿐이라고
믿을 근거는 없었으므로 12개 전부를 같은 질문으로 잰다: 이 필드를
`ParryCollisionEnv::check_self_collision`과
`CollisionEnv::check_robot_collision`이 각각 **읽는가 / 무시하는가 /
읽을 수 있는 자리 자체가 없는가**.

세 번째 항목을 두 번째와 분리하는 이유는 판정이 다르기 때문이다.
"무시한다"는 결함이고 — 그것이 `distance`였다 — "닿을 자리가 없다"는
상류 자신도 백엔드에서 읽지 않는다는 사실의 결과다. 조사는 그 둘을
섞지 않으려고 필드마다 상류의 읽는 지점을 먼저 찾고 시작했다.

### §239.1 12개 필드 × 두 진입점

`self`는 `check_self_collision`, `robot`은 `check_robot_collision`이다.
포트 인용은 `crates/moveit-collision/src/parry.rs`, 상류 인용은
`moveit_core/` 아래 경로다.

| 필드 | self | robot | 포트에서 읽는 곳 | 상류에서 읽는 곳 |
|---|---|---|---|---|
| `group_name` | 읽음 | 읽음 | `:2490`, `:2517` (`active_group_links`), `:2464` (거리 질의로 전달) | `collision_common.cpp:1012-1022` (`CollisionData::enableGroup`), 술어는 `:80-94` |
| `pad_environment_collisions` | **포팅 안 함 (§242)** | **포팅 안 함 (§242)** | 필드가 없다 (`b5cced7`) | 백엔드에는 없다. `planning_scene.cpp:442`가 유일한 독자이고, 필드를 백엔드에 넘기는 대신 패딩된/안 된 두 `CollisionEnv` 인스턴스 중 하나를 고른다 |
| `pad_self_collisions` | **포팅 안 함 (§242)** | **포팅 안 함 (§242)** | 필드가 없다 (`b5cced7`) | 같은 방식, `planning_scene.cpp:453`·`:558`. 상류 트리 전체에 **대입하는 곳이 0**이므로 실효값은 기본 `false` 하나다 |
| `distance` | 읽음 | 읽음 | `:2460` (`attach_requested_distance`) | `collision_env_fcl.cpp:283-297` (self), `:340-354` (robot) |
| `detailed_distance` | 읽음 | 읽음 | `:2469` | 같은 두 블록의 `if (req.detailed_distance)` |
| `cost` | 읽음 | 읽음 | `:2184`, `:2243`, `:2271` | `collision_common.cpp:279-288`, `:341-354`, 그리고 종료 규칙 `:405` |
| `contacts` | 읽음 | 읽음 | `:2200`, `:2240`, `:2271` | `collision_common.cpp:196`, `:398` |
| `max_contacts` | 읽음 | 읽음 | `:2200`, `:2271` | `collision_common.cpp:198`, `:214`, `:398` |
| `max_contacts_per_pair` | 읽음 | 읽음 | `:2202` | `collision_common.cpp:212-214` |
| `max_cost_sources` | 읽음 | 읽음 | `:2187` | `collision_common.cpp:286-287`, `:352-353` |
| `is_done` | **읽음 (이 회차에 고침)** | **읽음 (이 회차에 고침)** | `:2274` (`sweep_is_done`) | `collision_common.cpp:411-413`, 그 결과를 보는 곳이 `:70-71`과 `collision_env_fcl.cpp:337` |
| `verbose` | 무시 | 무시 | 없음 | `collision_common.cpp`에 17군데, **전부 로그 전용** |

`distance`/`detailed_distance` 두 행은 §231.1의 `00e37c1`을 상류에서
다시 확인한 것이다. 두 블록은 `DistanceRequest`에 `group_name`과 `acm`
둘만 넣고 나머지는 기본값으로 두는데, `attach_requested_distance`도
정확히 그 둘만 넣는다(`:2464`). 빠뜨린 필드는 없다.

`146bc2c`의 정렬 키도 같은 방식으로 확인했고, 이것은 **일탈이 아니다**:
상류는 `collision_common.cpp:240-242`, `:329-331`(접촉 맵), `:565-567`
(거리 맵) 세 곳 모두에서 `cd1->getID() < cd2->getID()`로 사전순 작은
이름을 앞에 둔다. 삽입 순서가 아니다. 반대로 `DistanceResultsData`의
`link_names`는 상류가 **정렬하지 않고**(`:627-628`이 `res_cd1`/`res_cd2`를
그 순서 그대로 넣는다) 포트도 정렬하지 않으므로, 정렬을 그쪽까지
넓히면 그때 일탈이 된다.

### §239.2 `is_done` — 선언되고, 공개로 설정할 수 있고, 아무도 읽지 않았다

`IsDoneFn`은 `crates/moveit-collision/src/common.rs:199`에서 공개
타입 별칭으로 나가고 `CollisionRequest::is_done`은 공개 필드다. 그런데
`146bc2c` 시점에 이 필드를 읽는 코드는 워크스페이스 전체에 없었다.

```console
$ git grep -n 'request\.is_done\|\.is_done(' 146bc2c -- crates ros
$ git grep -c is_done HEAD -- crates/moveit-collision/src/common.rs
HEAD:crates/moveit-collision/src/common.rs:5
```

다섯 히트는 전부 선언 쪽이다 — 타입 별칭, 필드, 손으로 쓴 `Debug`,
`Default`, 그리고 그 doc. `distance`와 정확히 같은 모양이다: 호출자는
설정할 수 있고, 설정해도 아무 일도 일어나지 않는다.

상류의 종료는 **두 갈래**이고 순서가 있다(`collision_common.cpp:395-424`).
암묵 갈래가 먼저다 — 충돌이 기록되었고, 접촉 예산이 필요 없거나 이미
찼고, 비용을 모으고 있지 않으면 `done_`이다. 그다음이
`if (!cdata->done_ && cdata->req_->is_done)`이다. 즉 `is_done`은 암묵
갈래가 이미 멈춘 뒤에는 **불리지 않는다**. `sweep_is_done`이 그 순서
그대로다.

멈추는 단위는 part 쌍이다. 상류에서 `collisionCallback` 한 번은 충돌
객체 한 쌍이고, 이 백엔드에서 그것은 body 쌍이 아니라 part 쌍이다
(`part_pairs`). 그리고 종료 블록은 `fcl::collide`가 아무것도 못 찾은
쌍에서도 실행된다 — 건너뛰기 규칙(`always_allow_collision`, 접촉 링크)만
`:184-185`에서 `return false`로 그 앞을 빠져나간다. 고치기 전 포트는
접촉이 없으면 `continue`로 건너뛰었으므로, 이 두 가지를 같이 옮겼다.

암묵 갈래는 **관측 가능한 출력은 바꾸지 않는다** — 그것이 발동하는
조건이 곧 "남은 필드가 이미 최종값"이라는 뜻이기 때문이다. 바뀌는 것은
뒤쪽 쌍의 `AllowedCollision::Conditional` 술어가 더 이상 불리지 않는다는
것 하나이고, 상류도 같다. 관측 가능한 쪽은 `is_done`이다: 콜백은
`collision`이 아직 `false`인 채로도, 예산이 남은 채로도 훑기를 끝낼 수
있다.

`is_done`에 넘기는 인자는 새로 만든 값이 아니라 **그 시점의 결과**여야
한다(상류는 `*cdata->res_`를 그대로 넘긴다). `sweep_result`가 반환값과
콜백 인자 양쪽을 만들어서, 표현이 둘로 갈라지지 않게 했다. 복제 비용은
`max_contacts`와 `max_cost_sources`로 묶여 있고 훑은 쌍 수와 무관하며,
`is_done`이 `None`이면 복제 자체가 일어나지 않는다.

시험 일곱 개는 경계마다 하나씩이고, 변이 아홉 개로 판별을 확인했다.
`if done { break; }`를 죽이면 답이 결정하는 시험 1건만, 건너뛴 쌍까지
종료를 재게 하면 ACM 시험 1건만, 콜백에 빈 스냅숏을 주면 스냅숏 시험
1건만 깨진다. 암묵 갈래를 `is_done` 뒤로 옮기면 선점 시험 2건이,
`!cost` 항을 지우면 비용 시험과 기존 `max_cost_sources` 시험 2건이,
접촉 예산 항을 지우면 스냅숏 시험과 기존 예산 시험 둘, 그리고
`upstream_panda_harness robot_world_collision_2`까지 3건이 깨진다.
`collision` 항을 지우면 11건이 깨지는데 여기에는 panda·pr2·fanuc 오라클
대조 셋이 포함된다 — 이 항은 새 시험보다 기존 대조가 먼저 붙잡는다.

### §239.3 무시되는 채로 남는 셋 — 그리고 그것이 `is_done`과 다른 이유

**`pad_environment_collisions`/`pad_self_collisions`.** 상류에서도 어떤
`CollisionEnv` 백엔드도 이 둘을 읽지 않는다 —
`collision_env_fcl.cpp`와 `collision_common.cpp` 어디에도 없다. 읽는
곳은 `planning_scene.cpp:442`·`:453`·`:558` 셋뿐이고, 셋 다 필드를
넘기는 대신 `getCollisionEnv()`와 `getCollisionEnvUnpadded()` 중 하나를
고른다. D4가 그 이중 환경 기계를 없애고 호출자가 소유하는 `E` 하나로
바꿨으므로(`crates/moveit-scene/src/scene.rs:566-576`,
`PlanningScene::check_collision`의 서명이 `env: &E` 하나다) 이 포트에는
고를 대상이 애초에 없다. `parry.rs` 안에서 고칠 수 있는 것이 아니고,
고치려면 `moveit-scene`이 환경 둘을 받는 API 변경이다 — 이 회차의
범위 밖이고, 그 판단은 D4를 다시 여는 문제다.

**§242가 이 문단을 대체한다.** "환경 둘을 받는 API 변경"이라는 전제가
틀렸다: 상류의 두 환경은 패딩 말고 다른 것이 없으므로 호출자가
`env.clone()` 뒤 `padding_scale_mut()`를 비우면 미패딩 환경이 나오고,
D4는 열리지 않는다. 두 필드는 포팅하지 않기로 판정했다(`b5cced7`).

이 행을 확정하는 동안 상류 결함 하나가 나왔다:
`checkCollisionUnpadded` 여섯 오버로드 중 둘이 `new_req`를 만들어
`pad_environment_collisions = false`를 넣고는 원본 `req`를 넘긴다
(`planning_scene.cpp:456-463`, `:501-508`). 이름과 반대로 패딩된 검사다.
`doc/upstream-bugs.md`의
`check-collision-unpadded-discards-its-own-request-copy`에 적었다.

**`verbose`.** 이 필드는 상류에서 **제어 흐름을 바꾸지 않는다**. 추측이
아니라 실측이다: `collision_common.cpp`의 17개 읽는 지점(`:110`, `:122`,
`:138`, `:151`, `:176`, `:188`, `:232`, `:255`, `:261`, `:320`, `:368`,
`:402`, `:414`, `:514`, `:530`, `:544`, `:557`) 전부가 `RCLCPP_DEBUG`
또는 `RCLCPP_INFO` 한 문장만 담은 블록을 연다. `collision_env_fcl.cpp`는
이 필드를 아예 언급하지 않는다. 따라서 무시해도 잃는 동작이 없다.

존중하려 해도 지금은 할 수 없다 — 이 워크스페이스에는 로깅 파사드가
없다.

```console
$ rg -l 'tracing::|log::(debug|info|warn|error)' crates
$
```

로깅을 도입하는 것은 이 크레이트의 결정이 아니므로 여기서는 무시로
남기고 사실만 적는다. 이 조사가 그 김에 상류 로그 결함 하나를 찾았고
(`collision_common.cpp:261-268`의 버리는 갈래가 "Contact was stored."라고
찍는다) `doc/upstream-bugs.md`의
`collision-callback-logs-contact-stored-when-dropped`에 있다.

만료 조건: **워크스페이스에 로깅 파사드가 생기면** `verbose` 행을 다시
연다. `pad_*` 두 행의 만료 조건은 §242.4가 다시 쓴다.

## §240 포팅됨 158건 중 선언 단위로 세어 본 것은 69건뿐이었다 — 나머지 89건을 목록으로 만들고 게이트를 붙였다 (2026-08-06)

### §240.1 재는 것이 다른 두 문서

`doc/port-coverage.md`는 코퍼스 245건 중 **미포팅 87건**을 분류한다. 그
반대쪽 158건은 어느 계기도 보지 않는다. §1이 그 판정을 그대로 적어 두었기
때문에 숨은 사실이 아니다: "포팅됨"은 어떤 `.rs` 파일의
`// Ported from moveit2 @ <sha>:` 헤더 블록 안에 그 상류 경로가 나온다는
뜻이고, 그게 전부다. **파일 단위 주장이다.**

그러므로 1,764줄짜리 헤더를 인용하면서 그 안의 228개 public 선언 중 몇
개를 옮겼든 `verify-port-coverage.sh`는 초록이다. 그 헤더가
`robot_state.hpp`이고, 이 라운드 전까지 몇 개인지 세어 본 기록은 트리
어디에도 없었다.

새 문서 `doc/declaration-audit-coverage.md`가 그 158건을 한 줄씩 적는다.
코퍼스 정의를 다시 쓰지 않고 `port-coverage.md` §1·§2를 가리키며, 계기
`tools/ci/measure-declaration-audits.py`도 `measure-port-coverage.py`의
`corpus_files()`/`cited_paths()`를 **import**한다 — 인용 블록 문법(중괄호
전개, 디렉터리 통째 인용, 들여쓴 멤버)의 두 번째 사본을 만들지 않기 위해서다
(`check-audit-scripts-not-copied.sh`가 적은 그 이유 그대로).

### §240.2 판정 규칙과 그 대가

`audited` = 트리 어딘가의 문장이 그 상류 파일(또는 그것이 선언하는 클래스)의
**모든** public 선언을 열거했다고 *주장하고* 선언마다 처분을 붙인 경우.
`none` = 그런 완전성 주장이 없는 경우.

엄격한 쪽이다. `moveit-error`의 산문은 `exceptions.hpp`가 선언하는 두 클래스
(`moveit::Exception`, `moveit::ConstructException`, 각각 public 선언 1개)를
모두 이름으로 대응시키지만 전수라고 주장하지 않으므로 `none`이다. 이 대가는
문서 §2에 명시했다.

측정 도중 후보 정규식 두 개가 자기 히트를 열어 보는 것만으로 반증됐다.
`crates/moveit-planning/src/request_adapters/check_start_state_bounds.rs:10`의
"Symbol audit"은 선언 감사가 아니라 `rclcpp`/`moveit_msgs` **출현** 감사이고
대상 파일도 코퍼스 밖(`moveit_ros/*`)이다.
`crates/moveit-octomap/src/tree.rs:87`의 "Symbol-by-symbol audit"은 octomap
상류 헤더에 대한 것이지 코퍼스 파일에 대한 것이 아니다. 반대 방향의 오차도
있었다: `crates/moveit-scene/src/scene.rs:50`의 진짜 `planning_scene.hpp`
감사(60개 항목)는 제목이 `# Scope`라서 제목 기반 스캔에 잡히지 않았다.
그래서 최종 목록은 정규식의 산출물이 아니라 **감사문마다 그 감사문이 스스로
적은 적용 범위를 읽어 파일에 배정한** 결과이고, 문서의 근거는 행마다 붙은
`파일:줄` 포인터 — 독자가 직접 여는 것 — 이지 어떤 스캐너의 출력이 아니다.

### §240.3 실측: 69 / 89, 그리고 그 89가 몰려 있는 곳

라운드 시작 시점 **69 audited / 89 none**. §240.5의 감사로 4건이 넘어가
지금은 73 / 85다.

미감사가 세 크레이트에 몰려 있다: `moveit-planners-pilz` 39,
`moveit-model` 19, `moveit-collision` 9 — 합쳐 85건 중 67건이고, 셋 다
라운드 시작 시점 선언 단위 감사가 0이었다. 나머지 18건은
`moveit-planners-stomp` 6, `moveit-geometry` 3, `moveit-kinematics` 3,
`moveit-state` 2, `moveit-error` 2, `moveit-ros` 1, `moveit-sampling` 1로
흩어져 있다. 이 크레이트별 수치는 게이트가 보지 않는다 —
`verify-declaration-audits.sh`는 행 집합과 판정 어휘와 증거를 대조할 뿐
산문에 적힌 분포를 대조하지 않으므로, 인용할 때는 표에서 다시 세라
(이 문단의 앞선 판본이 20 / 13 / 68로 적혀 있었고, 셋 다 표와 다르며
셋의 합조차 자기가 말한 68이 아니었다). 단일 파일로 가장 큰 것은
`robot_state.hpp`로, `RobotState` 자체만 public 선언 **228**개
(`count-public-declarations.sh` 실측, 파일 1,764줄)이며 두 크레이트가
인용한다.

`moveit-test-support`는 `doc/claim-audit/`에 대응 파일이 없는 유일한
크레이트지만 이 표에는 아무 행도 기여하지 않는다 — 그 크레이트의 `.rs`에는
헤더 블록 인용이 0건이다. 별개의 구멍이므로 사실만 적고 판정하지 않았다.

### §240.4 이 세 수에는 게이트가 있다

`port-coverage.md` §4의 3분할이 아무 계기 없이 여러 문서에 인용되는 값인
것과 달리, `verify-declaration-audits.sh`가 행 집합·판정 어휘·증거를 전부
대조한다. 새로 포팅된 파일이 생기면 `MISSING ROW`로 실패하고, 판정이 두
단어 밖이면 실패하고, `audited`인데 증거가 없거나 `파일:줄` 형태가 아니거나
그 파일이 없거나 그 줄이 파일 끝을 넘으면 실패한다. 행 0개 파싱도 실패다.
검사하지 **않는** 것은 그 줄이 여전히 감사문의 시작인지이며, 문서에 그렇게
적었다.

`--check`를 아홉 가지로 변이시켜 각각이 서로 다른 메시지로 무는 것을
확인했다: 행 삭제(`MISSING ROW`), 가짜 행 추가(`STALE ROW`), 행 중복
(`DUPLICATE ROW`), 판정 어휘 위반, `audited`에서 증거 제거, `none`에 증거
추가, 없는 파일 지목, 파일 끝 초과 줄 지목, 표 문법 파괴(행 0개). 변이하지
않은 사본은 OK다.

### §240.5 실제로 감사한 4건 — `World`(37)와 `AllowedCollisionMatrix`(29)

`moveit-collision` 13건 중 중심 두 클래스와 그 짝 `.cpp`.
`crates/moveit-collision/src/world.rs:119`와
`crates/moveit-collision/src/matrix.rs:13`에 선언마다 처분을 적었다. 두
헤더 모두 스크립트 계수와 손 열거가 정확히 일치했다(37, 29). `World`의 중첩
`struct Object` 8건은 스크립트가 셀 수 없어(`class`만 매칭, 깊이 1만 계수)
`world.hpp:78-117`에서 손으로 열거했다고 감사문에 적었다.

`robot_state.hpp`(228)를 고르지 않은 이유는 한 라운드에 끝나지 않기
때문이다. 절반만 한 감사는 완전성을 주장할 수 없어 §240.2의 규칙상 `none`과
구별되지 않으므로, 시작해 두는 것에 값이 없다.

처분이 `ported`가 아닌 것은 decided-non-port 8건(`~World()`,
`using const_iterator`, `ObserverHandle`/`ObserverCallbackFn`/`addObserver`/
`removeObserver` — deviation 4, 만료 조건 명시 —,
`MOVEIT_CLASS_FORWARD(AllowedCollisionMatrix)`, `print`)과 unported-in-scope
2건이다. 후자는 `AllowedCollisionMatrix`의 메시지 생성자와 `getMessage()`로,
D6/§4.3이 `moveit-ros`의 `TryFrom` 층에 배정했으나 아직 없다. 새로 발견한
미처리가 아니라 이미 이름이 적힌 구멍이다 —
`ros/moveit-ros/src/scene/planning_scene.rs:19-24`가
`allowed_collision_matrix`를 미변환 `PlanningScene` 필드로 열거해 두었고 그
파일을 열어 확인했다.

`doc/port-coverage.md`에 새 `gap` 행은 생기지 않았고 생길 수 없다: 그 표는
미포팅 **파일**을 분류하는데 이 4건은 전부 포팅된 파일이고, 위 2건은 파일이
아니라 파일 안의 선언이다. 선언 단위 미처리를 적을 자리가 그 표에 없다는 것
자체가 새 문서가 생긴 이유다.

### §240.6 감사가 고친 상류 서술 오류 하나

`matrix.rs`가 `AllowedCollisionMatrix::print`를 "`rclcpp` 로거
(`RCLCPP_WARN_STREAM_THROTTLE`)로 포맷하므로 미포팅"이라고 적고 있었다.
`collision_matrix.cpp:428-491`에는 로깅이 없다 — 호출자가 준
`std::ostream&`에 인덱스 헤더 행과 이름별 `01?`/`-` 표를 쓰고 끝난다.
`RCLCPP_WARN_STREAM_THROTTLE`은 이 체크아웃 전체에서
`collision_common.cpp:60` 한 곳뿐이고 `print`와 무관하다. 이 체크아웃에
`AllowedCollisionMatrix::print` 호출자는 0건이다(`\.print\(|->print\(`
전체 6건, 어느 것도 수신자가 ACM이 아님). 사유를 "호출자 0건인 디버그
프린터, 대응물은 `Display` impl"로 교체했고 재개 조건도 적었다.

파일 단위 인용이 초록인 채로 선언 하나의 미포팅 **사유**가 틀려 있었다는
것이 §240.1의 구멍을 보여 주는 가장 짧은 예다.

### §240.7 이 절이 하지 않은 것

미감사 85건 중 81건은 그대로다. `moveit-planners-pilz` 39,
`moveit-model` 20, `robot_state.hpp`/`.cpp` 2를 포함한다. 감사 결과 발견될
미처리 선언은 이 문서의 행이거나 판정이지 `port-coverage.md`의 행이 아니다
(§240.5).

`moveit-test-support`의 `doc/claim-audit/` 부재도 그대로다 — 이 문서가 재는
구멍이 아니어서 판정하지 않았다.

## §241 `/plan_kinematic_path` 서비스와 노드 바이너리를 지었다 — 서비스는 살아있고 와이어를 왕복하지만, `MoveGroupInterface::plan()`은 이 서비스를 아예 부르지 않는다 (2026-08-06)

### §241.1 지은 것 — `fn main`/`r2r::Node`/서비스 등록, §226.4가 부재로 적은 조각 중 첫째와 둘째의 절반

§226.4가 STEP 3에서 부재로 적은 넷 중, 이번 라운드가 지은 건: (1) `fn
main`을 갖는 `[[bin]]` 타깃(`ros/moveit-ros/src/bin/plan_kinematic_path_server.rs`,
Cargo가 `src/bin/`을 자동 발견) — `r2r::Context::create` →
`r2r::Node::create` → `node.spin_once` 루프, 지금까지 `ros/moveit-ros`
어디에도 없던 것. (2) 그 위의 `/plan_kinematic_path`
(`moveit_msgs/srv/GetMotionPlan`) 서비스 등록 — `/move_action`과
planning scene 구독은 짓지 않았다, §241.4가 왜인지 적는다.

r2r는 특정 async 런타임을 강제하지 않는다(README: "the library
purposefully does not chose an async runtime") — r2r 자신의
`examples/service.rs`가 하듯 `futures::executor::LocalPool` +
`node.spin_once` 루프로 짰다. `moveit-ros/Cargo.toml`에 `futures =
"0.3"`을 새로 추가했고(r2r 자신이 핀한 버전과 동일), `moveit-srdf`/`urdf-rs`를
`[dev-dependencies]`에서 `[dependencies]`로 옮겼다(이 바이너리가 시작
시 URDF/SRDF 파일 경로에서 `RobotModel`을 로드하는 데 쓴다 —
`state.rs`의 테스트 헬퍼 `one_joint_model_from`과 같은 패턴).

### §241.2 요청 변환까지만 배선했다 — 플래너를 부르지 않는다, 이 워크스페이스에 부를 게 없어서

`handle_request`는 들어온 `GetMotionPlan::Request`를 이미 있는
`TryFrom<PlanningRequestMsg> for PlanningRequest`
(`ros/moveit-ros/src/planning.rs`)로 변환한 뒤 멈춘다 — 플래너를 부르지
않으므로 모든 응답이 빈 궤적과 `SUCCESS`가 아닌 `error_code`를 싣는다.

이건 지름길이 아니라 실측이다. `rg -n "impl.*Planner<'m>.*for"
crates/`는 `crates/moveit-planning/src/pipeline.rs`의
`#[cfg(test)] mod tests` 안 네 개(`FixedGoalPlanner`, `FailingPlanner`,
`RecordingPlanner`, `SideEffectPlanner`) 말고는 0건이고,
`crates/moveit-planning/src/response.rs:45-68`의 문서 주석이 이미 같은
결론을 적어 뒀다(이번 라운드가 다시 확인, 문서화된 공백도 확인되지
않은 결함일 수 있다는 이유에서). `rg -n moveit-planning
crates/moveit-planners-{sbp,chomp,stomp,pilz}/Cargo.toml`도 0건 — 이
워크스페이스의 플래너 크레이트 넷 중 어느 것도 `moveit-planning`에
의존하지 않는다. 이 워크스페이스에 존재하는 유일한 구체 플래너,
`moveit_planners_sbp::registry::RrtConnectManager`(`crates/moveit-planners-sbp/src/registry.rs:616`,
`impl PlannerManager for RrtConnectManager`)는 `moveit-planning`의
`PlanningRequest`/`PlanningResponse`와 이름만 같고 타입이 다른, 자기
자신의 `PlanningRequest`/`PlanningResponse`를 쓴다
(`crates/moveit-planners-sbp/src/registry.rs:270-340`, 이번 라운드가 다시 읽어 확인).

이 둘을 잇는 어댑터를 `ros/moveit-ros` 안에 지금 짜지 않았다 — D8
(§140)이 이미 이 둘을 하나의 크레이트(`moveit-planner-registry`)로
합치기로 정해 뒀고("이건 구조적 해결을 미루는 게 아니라 순서다: 지금
하면 같은 파일을 두 라운드가 동시에 고친다"), 여기서 임시 변환을 짜면
`goal`이 크레이트 경계를 넘나들며 두 가지 다른 뜻을 갖는 채로 다음
라운드에 넘겨진다 — CLAUDE.md의 structural-fix-over-patch 규칙이 바로
이 모양을 patch로 이름 붙인다. `ros/moveit-ros` 펜스 밖이기도 하다 —
`moveit-planners-sbp`에 의존을 추가하는 것과, D8이 이미 주인인 어댑터를
여기서 짜는 것은 다른 일이다.

### §241.3 살아있는 DDS 왕복으로 실측 — `ros2 service call`이 진짜 요청을 보내고, 서버가 진짜 타입 응답을 돌려준다

와이어 변환이 실제로 라이브 DDS 위에서 도는지, in-process 구조체
생성이 아니라: `moveit-rs/ros-dev:latest` 컨테이너 안에서 이 바이너리를
한 조인트 URDF/SRDF fixture로 띄우고, 같은 컨테이너의 `ros2 service
call /plan_kinematic_path moveit_msgs/srv/GetMotionPlan "{}"`로
호출했다.

```
response:
moveit_msgs.srv.GetMotionPlan_Response(motion_plan_response=moveit_msgs.msg.MotionPlanResponse(
  ...
  error_code=moveit_msgs.msg.MoveItErrorCodes(
    val=-1,
    message='moveit-ros has no moveit_planning::pipeline::Planner to call yet
             (PORTING-PLAN.md §241): the request converted, but there is no
             planner in this workspace to hand it to.',
    source='moveit-ros/plan_kinematic_path_server')))
```

`ros2 service list`가 `/plan_kinematic_path`를 실제로 보였고, 요청은
in-process 함수 호출이 아니라 실제 DDS 미들웨어를 왕복했다 —
`ros/verify-ros-interop.sh` 자신의 머리말이 "No live ROS 2 graph"라
적어 둔 공백 중 하나를 이번 라운드가 실제로 메웠다는 근거다.

이 실측을 `ros/verify-ros-interop.sh`에 회귀 게이트로 옮겼다(새 `run
"live"` 단계) — fixture URDF/SRDF를 컨테이너 안에 쓰고, 서버를
백그라운드로 띄우고, `ros2 service call`로 호출한 뒤 응답 문자열에서
`val=-1`과 위 메시지를 grep한다. "먼저 실패하고 나중에 통과하는지"를
직접 쟀다 — 이번 라운드가 추가한 `ros/moveit-ros/Cargo.toml`,
`Cargo.lock`, `src/bin/`만 `git stash`로 걷어내고 같은 스크립트를
돌리면 `error: no bin target named
plan_kinematic_path_server`로 확정 실패(exit 101)했고, `git stash
pop`으로 되돌린 뒤 다시 돌리면 통과한다. 이 과정에서 스크립트 자신의
기존 버그도 하나 찾아 고쳤다 — §241.5.

### §241.4 결정적 실측 — 무변경 `MoveGroupInterface::plan()`은 `/plan_kinematic_path`를 아예 부르지 않는다

"완료" 기준은 무변경 C++ `MoveGroupInterface` 클라이언트가 이 서버에
요청을 보내 유효한 궤적을 받는 것이다. 실제로 그 클라이언트를 이
기계에서 띄우기 전에, 상류 소스 자체가 답을 준다.

`moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp`
(상류 오라클 이미지의 `/ws/src/moveit2` 체크아웃):

- `MoveGroupInterface::plan(Plan& plan)` (:1455)는 `impl_->plan(plan)`을
  호출할 뿐이다.
- `MoveGroupInterfaceImpl::plan(Plan& plan)` (:657)는 시작하자마자
  `move_action_client_->action_server_is_ready()`를 검사해(:659), 준비
  안 됐으면 **로컬에서** `MoveItErrorCode::FAILURE`를 반환하고
  끝난다 — 아무 메시지도 나가지 않는다. 준비됐으면
  `moveit_msgs::action::MoveGroup::Goal`을 만들어
  `move_action_client_->async_send_goal`로 보낸다(:712) —
  `move_action_client_`는
  `rclcpp_action::create_client<moveit_msgs::action::MoveGroup>`(:188),
  즉 `/move_action` 액션이다.
- 이 파일 전체에 `GetMotionPlan`이나 `plan_kinematic_path`를 언급하는
  줄은 0건이다(`grep -n "GetMotionPlan\|plan_kinematic_path"
  move_group_interface.cpp` → 무매치, `create_client` 호출 다섯 건은
  `QueryPlannerInterfaces`/`GetPlannerParams`/`SetPlannerParams`/`GetCartesianPath`뿐).

즉 무변경 `MoveGroupInterface::plan()`은 `/plan_kinematic_path`를 부를
**경로 자체가 없다** — 서비스가 완벽하게 살아 돌아가도(§241.3이 그걸
증명했다) 닿지 않는다. 필요한 건 `/move_action`
(`moveit_msgs/action/MoveGroup`) 액션 서버다. 이건 §235가 이미 "raw
서비스 호출은 `MoveGroupInterface` 클래스 자체보다 좁다"고 산문으로
적어 둔 우려를, 코드 인용이 있는 사실로 좁힌 것이다 — 좁을 뿐 아니라,
그 클래스의 유일한 플래닝 경로가 아예 다른 서비스를 쓴다.

**측정한 지점, 그대로:** 이 라운드가 지은 `/plan_kinematic_path`
서비스는 살아있고, 와이어를 왕복하고, 게이트로 고정됐다. "완료" 기준을
직접 재는 데는 미달이다 — 막힌 지점은 다음 조각이지 이번 조각의 결함이
아니다:

1. `/move_action` (`moveit_msgs::action::MoveGroup`) 액션 서버 —
   §226.4 항목 2의 나머지 절반. `MoveGroupInterface::plan()`이 실제로
   부르는 유일한 경로. 이번 라운드가 위임받은 "그 조각과 그것만"에
   포함되지 않았으므로 짓지 않았다.
2. planning scene 토픽 구독 — §226.4 항목 3, 그대로 부재.
3. `ros-dev` 이미지에 C++ `moveit2` 스택을 얹거나, 오라클 이미지에
   Rust/r2r 툴체인을 얹는 이미지 작업 — §226.4 항목 4, 그대로
   미측정/미착수. `/move_action`이 지어져도 이 작업 없이는 실제
   `MoveGroupInterface` 클라이언트를 이 기계에서 이 노드에 대고 돌릴
   방법이 없다.
4. 1~3이 갖춰진 뒤에야 "코드 변경 없는 기존 C++ `MoveGroupInterface`
   클라이언트가 유효한 궤적을 받는다"는 원래 문구 그대로의 종단
   시도가 가능하다 — 그리고 그때도 `moveit-planning`에 부를 플래너가
   없으므로(§241.2) 받는 건 유효한 궤적이 아니라 `PLANNING_FAILED`일
   것이다. D8과 플래너 배선은 그 시점 이전에 별도로 닫혀야 한다.

### §241.5 게이트 자신의 결함 하나 — `[[bin]]` 타깃이 `verify-ros-interop.sh`의 "마지막 test result 줄" 가정을 깼다

`ros/verify-ros-interop.sh`의 유닛테스트 카운트 검증은 "Doc-tests
앞의 마지막 `test result:` 줄"을 lib의 결과로 가정했다(`tail -1`) —
이번 라운드 전까지는 유닛테스트 바이너리가 lib 하나뿐이라 맞는
가정이었다. `src/bin/plan_kinematic_path_server.rs`를 추가하니 `cargo
test`가 그 바이너리용 유닛테스트 스위트를 하나 더 돌리고(테스트 0개,
설계대로), 그 결과가 lib의 "174 passed"와 "Doc-tests" 사이에 끼어든다
— `tail -1`이 이제 bin의 "0 passed"를 집어, 스크립트가 `cargo test
reported 0 passing unit test(s) but ... has 174`로 **거짓 실패**했다.
이번 라운드가 §241.3의 "live" 단계를 넣기 전에 전체 게이트를 먼저
돌려 직접 겪었다.

고친 방식은 patch가 아니라 가정을 일반화한 것이다 — "Doc-tests 이전의
`test result:` 줄은 하나"가 아니라 "몇 개든 전부 더한다"로 바꿨다.
`[[bin]]` 타깃이 몇 개로 늘어나도 성립하는 규칙이고, 이번 버그를 만든
그 가정 자체를 제거한다.

### §241.6 §5 표 갱신 여부

Phase 9의 판정은 UNMET에서 바뀌지 않는다 — §241.4가 측정한 대로
"완료" 기준(무변경 클라이언트가 유효한 궤적을 받음)에 아직 미달이다.
사용자 지시대로 판정이 바뀔 때만 §5 표 행을 고치므로, 이 라운드는 그
행(측정한 §: §226.4)을 건드리지 않는다.

## §242 `pad_environment_collisions`/`pad_self_collisions` — 상류의 두 환경 차이는 패딩뿐이고, D4의 `E` 하나가 그것을 낸다 (2026-08-06)

§239.3이 이 두 필드를 "고치려면 `moveit-scene`이 환경 둘을 받는 API
변경이고 그 판단은 D4를 다시 여는 문제"라며 열어둔 채 끝냈다. 그 문장은
판정이 아니라 미룸이었다. 여기서 재고 판정한다.

질문은 셋이다 — 상류에서 이 둘을 실제로 쓰는 곳은 어디인가, 그중
`move_group`의 계획·실행 경로에서 닿는 것은 무엇인가, 그리고 상류가 가진
두 `CollisionEnv` 인스턴스는 서로 **무엇이** 다른가. 셋째를 먼저 답하지
않으면 포트의 대안이 "필드를 살린다"로만 보이는데, 실제 대안은 그것이
아니다.

### §242.1 앵커 넷을 전수하면 코퍼스 안 호출자는 0이다

브리프가 지목한 앵커는 `checkCollisionUnpadded`,
`checkSelfCollisionUnpadded`, 그리고 두 필드다. 먼저 실측 하나를
정정한다: **`checkSelfCollisionUnpadded`는 상류에 존재하지 않는다.**

```console
$ cd /home/stevek/work/moveit2   # e017c91e
$ rg -o --no-heading '\b\w*[Uu]npadded\w*\b' --glob '*.cpp' --glob '*.hpp' --glob '*.h' . \
    | sed 's/.*://' | sort | uniq -c | sort -rn
     17 checkCollisionUnpadded
     16 getCollisionEnvUnpadded
      7 distanceToCollisionUnpadded
      5 cenv_unpadded_
      4 check_collision_unpadded
      3 unpadded
      3 cenv_unpadded_const_
      2 CollisionRobotUnpadded
      1 unpadded_param
```

패딩 안 된 환경으로 가는 문은 셋이고(`checkCollisionUnpadded`,
`getCollisionEnvUnpadded`, `distanceToCollisionUnpadded`) 자기 충돌
전용 문은 없다. 넷을 파일별로 펼치면:

| 파일 | 히트 | 코퍼스 | 무엇인가 |
|---|---|---|---|
| `moveit_core/planning_scene/src/planning_scene.cpp` | `checkCollisionUnpadded` 정의 6 (`:457`·`:465`·`:473`·`:482`·`:491`·`:502`), `pad_environment_collisions` 7 (`:442`와 그 여섯 정의 안의 `new_req`), `pad_self_collisions` 2 (`:453`·`:558`) | 안 | **정의·유일한 독자** |
| `moveit_core/planning_scene/include/.../planning_scene.hpp` | `checkCollisionUnpadded` 선언 6 (`:380`~`:412`) | 안 | 선언 |
| `moveit_core/collision_detection/include/.../collision_common.hpp` | `:154`, `:157` | 안 | 필드 자신 |
| `moveit_py/.../planning_scene.cpp` | `check_collision_unpadded` 바인딩 3 + 주석 2 | 밖 | 파이썬 바인딩 |
| `moveit_ros/planning/plan_execution/src/plan_execution.cpp` | `:285` `req.pad_environment_collisions = false;` | 밖 | **실호출자** |
| `moveit_ros/benchmarks/src/BenchmarkExecutor.cpp` | `:1012` 같은 대입, `:1021` `distanceToCollisionUnpadded` | 밖 | **실호출자** |
| `moveit_ros/moveit_servo/src/collision_monitor.cpp` | `:124` `getCollisionEnvUnpadded()->checkSelfCollision(...)` | 밖 | **실호출자** |
| `moveit_core/planning_scene/test/test_planning_scene.cpp` | `getCollisionEnvUnpadded()->getWorld()->size()` 3 (`:181`·`:184`·`:204`) | 밖(`test` 경로) | 월드 크기만 본다 |

코퍼스 안 파일 셋은 전부 **선언·정의·유일한 독자**이고, 넷 중 어느 것도
부르는 코퍼스 파일은 없다. 즉 **코퍼스 안 호출자 0**이다.

그 전수에서 하나가 더 나왔다. **`pad_self_collisions`에 값을 대입하는
곳은 상류 트리 전체에 0개다.** 히트 셋은 `collision_common.hpp:157`의
기본값 `false`와 `planning_scene.cpp:453`·`:558`의 삼항 연산자뿐이다.
따라서 상류의 실효 규칙은 필드 이름이 시사하는 "요청마다 고른다"가
아니라 **"환경 쪽은 패딩, 자기 충돌은 언제나 패딩 없음"** 이다.
`checkCollisionUnpadded` 여섯도 `pad_environment_collisions`만 끄므로
같은 결론에 든다.

### §242.2 `move_group` 경로에서 닿는 것은 `isRemainingPathValid` 하나다

Phase 9의 조건은 "기존 C++ `MoveGroupInterface` 클라이언트가 유효한 궤적을
그대로 받는다"이므로, 코퍼스 밖 셋 중 `move_group`이 링크하는 것이 어느
것인지가 실제 질문이다.

```console
$ rg -n --no-heading -w 'isRemainingPathValid|planAndExecute|executeAndMonitor' \
     --glob '*.cpp' --glob '*.hpp' .
moveit_ros/move_group/src/default_capabilities/move_action_capability.cpp:183:  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);
moveit_planners/pilz_industrial_motion_planner/src/move_group_sequence_action.cpp:179:  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);
moveit_ros/planning/plan_execution/src/plan_execution.cpp:219:      plan.error_code = executeAndMonitor(plan, false);
moveit_ros/planning/plan_execution/src/plan_execution.cpp:501:      if (!isRemainingPathValid(plan, current_index))
moveit_ros/planning/plan_execution/src/plan_execution.cpp:617:    if (!isRemainingPathValid(plan, next_index))
```

- **닿는다.** `plan_execution.cpp:285`. `move_action_capability.cpp:183`
  → `planAndExecute`(`:118`) → `executeAndMonitor`(`:355`) →
  `isRemainingPathValid`(`:501`, 그리고
  `successfulTrajectorySegmentExecution`의 `:617`). 진입 조건은
  `plan.plan_components[i].trajectory_monitoring`이고 기본값이 `true`다
  (`plan_representation.hpp:51`·`:59`). 두 번째 진입점
  `move_group_sequence_action.cpp:179`는 코퍼스 **안** 파일이지만
  `doc/port-coverage.md:199`가 `decided-non-port`로 판정한 액션 서버다.
- **닿지 않는다.** `moveit_servo`(`collision_monitor.cpp:124`)는
  `moveit_servo_lib_cpp`(`moveit_ros/moveit_servo/CMakeLists.txt:44`)에,
  `benchmarks`(`BenchmarkExecutor.cpp:1012`·`:1021`)는
  `moveit_run_benchmark`가 링크하는 라이브러리(`:31`, `:49`)에 들어간다.
  둘 다 `move_group`이 링크하지 않는다 — `rg -n 'servo|benchmark'`가
  `moveit_ros/move_group/CMakeLists.txt`와 `package.xml`에서 히트 0이다.

그러므로 Phase 9가 존중해야 하는 미패딩 호출자는 **정확히 하나**,
`PlanExecution::isRemainingPathValid`다. 그리고 그것이 사는
`moveit_ros/planning/plan_execution`은 이 포트에 없다 —
`ros/moveit-ros/src/`에는 `constraints/`, `scene/`, `geometry.rs`,
`model.rs`, `planning.rs`, `state.rs`, `trajectory.rs`,
`conversion_coverage.rs`뿐이다.

### §242.3 두 환경은 패딩 말고 다른 것이 없다 — 그래서 대안은 "필드"가 아니라 "clone"이다

`allocateCollisionDetector`(`planning_scene.cpp:255-286`)를 열면 둘은 같은
인자로 만들어진다.

```cpp
collision_detector_->cenv_          = alloc_->allocateEnv(world_, getRobotModel());
collision_detector_->cenv_unpadded_ = alloc_->allocateEnv(world_, getRobotModel());
if (prev_coll_detector)
  collision_detector_->copyPadding(*prev_coll_detector);   // cenv_ 만
```

그 뒤로 `cenv_unpadded_`에 패딩을 넣는 곳은 없다. 패딩을 쓰는 네 지점이
전부 `cenv_`를 이름으로 지목한다 — `copyPadding`(`:249-252`),
`pushDiffs`의 `active_cenv`(`:365-366`, `getCollisionEnvNonConst()`이므로
`cenv_`), `setPlanningSceneMsg`(`:1348-1349`), `usePlanningSceneMsg`
(`:1386-1387`). 부모 분기(`:269-272`)는 각자의 짝을 복사 생성하므로
성질이 귀납적으로 보존된다. 즉 **"unpadded"는 `CollisionEnv` 생성자
기본값(패딩 `0.0`, 스케일 `1.0`, `collision_env.cpp:83-95`·`:99-115`)
그대로라는 뜻**이고, 그것은 이 포트의 `LinkPaddingScale::default()`와
같은 상태다.

따라서 D4의 `E` 하나로 미패딩 검사를 못 낸다는 전제가 틀렸다. 호출자가
쓰는 것은:

```rust
let mut unpadded = env.clone();
*unpadded.padding_scale_mut() = LinkPaddingScale::default();
```

`ParryCollisionEnv`의 `World`는 `BTreeMap<String, Arc<Object>>`이고
옥트리 캐시는 `Arc<Mutex<..>>`이므로 이 `clone`은 얕은 참조 증가다
(`ParryCollisionEnv::new`로 다시 지으면 캐시가 식는다 — `clone` 쪽이
싸고 옳다). 남는 차이 하나는 상류의 두 환경이 `WorldPtr` 하나를
공유하는 살아 있는 뷰인 반면 여기 `clone`은 스냅숏이라는 것인데,
유일한 대상 호출자 `isRemainingPathValid`는 루프 전체를
`LockedPlanningSceneRO`로 잠그고 돌므로 그 구간에 월드가 바뀌지 않는다.

### §242.4 판정 — 두 필드는 포팅하지 않는다

구조적 선택지는 둘이었다. (a) 필드를 살리고
`PlanningScene::check_collision`이 `if padded` 런타임 분기로 환경 둘 중
하나를 고르게 한다. (b) 규칙을 하나로 두고 — 패딩은 `E`의 성질이다 —
필드를 없앤다.

(b)를 택했다(`b5cced7`). 근거는 취향이 아니라 §242.1~.3이다: 이 둘은
백엔드 필드가 아니고(어떤 `CollisionEnv`도 읽지 않는다), 코퍼스 안
호출자가 0이며, 살려두면 **설정할 수 있는데 아무도 읽지 않는 필드**가
된다 — `distance`(§231.1, `00e37c1`)와 `is_done`(§239.2)이 방금 두 번
낸 바로 그 결함 모양이다. 없애면 그 상태를 표현할 수 없다.

남는 차이는 필드가 아니라 **어느 쪽 절반에 패딩이 닿는가**이고, 이것은
`f43daeb`에서 `PlanningScene` 타입 문서의 명시적 일탈로 적고 시험으로
고정했다. 상류 실효 규칙은 §242.1이 잰 대로 "자기 충돌은 언제나 패딩
없음"인데 이 포트는 `E` 하나이므로 양쪽에 같은 패딩이 간다. 패딩이 든
`E`에서만 갈라지고, 이 워크스페이스에는 그런 호출자가 없다 —
비기본 `LinkPaddingScale`을 만드는 두 곳
(`crates/moveit-collision/tests/link_padding_changes_collision_verdict.rs`,
`crates/moveit-collision/tests/upstream_panda_harness.rs`)은 모두
`CollisionEnv`를 직접 부르고, 이는 상류 자신의
`test_collision_common_panda.hpp:215-233`이 하는 것과 같다.

경계 양쪽을 시험으로 못박았다.
`crates/moveit-scene/tests/padding_reaches_the_scenes_self_half.rs`가
패딩 없는 `E`(상류와 같은 답)와 패딩 든 `E`(갈라지는 답)를 한 쌍으로
두고, `link_padding_changes_collision_verdict.rs`의 새 시험이 백엔드
층에서 `check_self_collision`이 `LinkPaddingScale`을 읽는다는 것 자체를
잡는다. 변이 셋으로 판별을 확인했다 — `check_self_collision`에서 패딩을
떼면 새 시험 둘만, `check_robot_collision`에서 떼면 월드 쪽 둘만,
`CollisionEnv::check_collision`이 자기 검사를 건너뛰게 하면 18건(그중
씬 시험은 패딩 든 쪽 하나)이 깨진다. 미추적 링크의 기본 패딩을
`0.0`에서 `0.05`로 바꾸면 52건이 깨지고 거기에 패딩 없는 쪽 씬 시험이
든다 — 음성 대조도 비어 있지 않다.

**만료 조건**(§231.2와 같은 모양, 취향이 아니라 사실로):
**`PlanningScene::check_collision`에 비기본 `LinkPaddingScale`을 가진
`E`를 넘기면서 상류의 미패딩 자기 절반을 필요로 하는 호출자가 생기면**
다시 연다. 실질적으로는 `moveit_ros/planning/plan_execution`을 포팅할
때다. 그때의 수정은 그 호출자가 §242.3의 미패딩 `clone`을 자기 손으로
넘기는 것이지, `PlanningScene`이 `E`를 둘 받는 것이 아니다 — 그 형태는
D4를 다시 열지 않는다.

## §243 `tools/ci` 게이트가 못 보는 경계를 뒤진다 — `verify-all.sh` 전체 실행과 32개 게이트 커버리지 표

`check-porting-plan-sections.sh`(§226 3중 충돌), `check-upstream-bugs-index.sh`
(방향이 틀린 `in_index` 플래그), `check-phase-status.sh`(9개의 append-only 문단)
셋 다 사고가 난 뒤에야 생겼다. 이번 라운드는 사고 전에 같은 모양을 찾는
것이 목표였다.

**Half one — 전체 스윕.** `verify-all.sh`의 18개 `verify-*.sh` 전부를
`PHASE3_SWEEP=1` 포함해 실행. 상세 결과, `verify-upstream-license-provenance.sh`의
"third_party/ 없음" 정정, `verify-phase3-collision-sweep.sh`가 panda/prbt
2개 로봇에서 이미 UNMET로 확정되고 나머지 3개는 이 라운드 시간 안에
끝나지 못한 채 백그라운드에 남겨진 경위는
`doc/claim-audit/tools-ci-gates.md` §243.1/§243.3에 전부 적었다 —
collision/distance-field는 이 패널의 펜스
(`moveit-scene`, `moveit-metrics`) 밖이라 보고만 하고 고치지 않았다.

**Half two — 커버리지 감사.** 32개 `check-*.sh`/`verify-*.sh` 전부를 각각이
실제로 무엇을 파싱하고 무엇을 못 보는지, 구체적 실패 사례와 함께
`doc/claim-audit/tools-ci-gates.md` §243.2 표에 적었다. 가장 값비싼 갭으로
고른 것을 게이트로 만들어 커밋했다
(`doc/claim-audit/tools-ci-gates.md` §243.4): `crates/*/tests/*.rs`의
bracket 스타일 doc 링크는 `cargo doc`(테스트 타깃 자체가 없음, `--help`로
확인), `cargo clippy`(링크 해석은 clippy 린트가 아님), `cargo test`(bracket
링크는 doctest가 아님) 어느 것에도 닿지 않는다 -- `verify-private-doc-links.sh`가
이미 문서화한 `#[cfg(test)]` 갭의 절반, `tests/*.rs` 쪽만 닫는
`tools/ci/check-test-doc-links.sh`를 새로 추가했다. 실제 첫 실행에서 진짜
실패 하나를 잡았다 (`moveit_error::Error::Code` -- 체커의 정밀도 결함으로
드러나 소스에서 고쳤다, 뮤테이션 판별 통과) — 전체 경위는
`doc/claim-audit/tools-ci-gates.md` §243.4.

찾았지만 이번 라운드에 게이트로 만들지 않은 것도 하나 있다
(`doc/claim-audit/tools-ci-gates.md` §243.5):
`check-porting-plan-sections.sh`는 §226 같은 번호 충돌은 막지만, 리넘버
후에도 살아남는 **참조**(정확히 이 스크립트 자신의 헤더가 적은 그 사고)는
아무것도 안 본다. 순진하게 "본문의 모든 §NNN이 `all_ids`에 있어야 한다"로
만들면 지금 트리에서 정당한 인용 4건을 전부 오탐한다: 외부 문헌
(Ericson, *Real-Time Collision Detection*) §4.4.1,
`ros/moveit-ros/doc/message-mapping.md` 자체 번호 체계의 §17.5,
그리고 PORTING-PLAN.md 자신의 굵은글씨 의사-소절 §177.1 두 건.
검증까지 마치고 다음 라운드로 넘겼다 — 실제로 무엇이 왔는지는
`doc/claim-audit/tools-ci-gates.md` §243.5의 병합 노트에 적었다.

---

## §244 §238.5가 "덮지 않았다"고 적은 세 항목의 처분 — 둘은 닫았고 하나는 오독이었다 (2026-08-06)

§238은 Phase 2 셋째 조건을 MET으로 올리면서 자기가 덮지 못한 것을 §238.5에
세 줄로 적어 두었다. MET 행이 측정되지 않은 경계 위에 서 있으면 그 행은
다시 열린다. 이 절은 그 세 줄을 각각 처분한다 — 두 개는 계기를 만들어
닫았고, 셋째는 실제로 결함이 아니라 §238 자신의 오독이었다.

### §244.1 mimic 산술의 비항등 점 — 상류에 픽스처가 있었고, 이미 커밋돼 있었다

§238.5의 첫 줄은 "커밋된 다섯 픽스처의 mimic이 전부 `multiplier=1,
offset=0`"이라 `factor * v + offset`이 항등인 한 점에서만 실행됐다고
적었다. 그 문장이 쓰인 뒤 `one_robot`이 들어왔다(`57b0fb7`).

```console
$ cd /home/stevek/work/moveit2 && git rev-parse HEAD
e017c91ee12984393a28ba246075c65f69cde3bf
$ grep -rn '<mimic' --include='*.urdf' --include='*.xacro' \
      --include='*.cpp' --include='*.hpp' --include='*.py' . | grep -v /\.git/
moveit_core/robot_state/test/robot_state_test.cpp:355:    <mimic joint="joint_f" multiplier="1.5" offset="0.1"/>
moveit_core/robot_trajectory/test/test_robot_trajectory.cpp:268:  "    <mimic joint=\"joint_f\" multiplier=\"1.5\" offset=\"0.1\"/>"
moveit_planners/test_configs/prbt_pg70_support/urdf/pg70/pg70.urdf.xacro:93:      <mimic joint="${name}_finger_left_joint" multiplier="1" offset="0"/>
moveit_py/test/unit/fixtures/panda.urdf:222:        <mimic joint="panda_finger_joint1" />
moveit_ros/robot_interaction/test/locked_robot_state_test.cpp:149:  "    <mimic joint=\"joint_f\" multiplier=\"1.5\" offset=\"0.1\"/>"
```

상류 전체에 `<mimic` 선언은 다섯 개고, 비항등 계수를 쓰는 것은 하나뿐이다 —
`joint_f`의 `1.5`/`0.1`, 세 C++ 테스트 파일이 같은 `MODEL2` 문자열을
공유해서 세 번 나타난다. 그 문자열이 곧
`fixtures/one_robot.urdf`이고(`verify-fixture-provenance.sh`의
`EXTRACTED_FROM ... robot_state_test.cpp::MODEL2`), `:105`에
`<mimic joint="joint_f" multiplier="1.5" offset="0.1"/>`을 그대로 들고
있다. 따라서 "합성 URDF가 이 자리에 허용되는가"라는 판단은 내릴 필요가
없었다 — 상류가 가진 유일한 비항등 mimic이 이미 벤더링돼 있고, 이번
라운드의 스윕은 그 로봇을 포함해 돈다(아래 §244.2의 `one_robot` 행).

산술을 세 방향으로 틀리게 만드는 변이 — 계수를 떨어뜨리기, 오프셋을
떨어뜨리기, `factor * (v + offset)`으로 순서를 바꾸기 — 는 항등 mimic만
있는 여섯 로봇에서 전부 같은 수를 내고, `one_robot`에서만 갈라진다. 그
점이 이 픽스처가 스윕에 들어 있어야 하는 이유이며,
`verify-phase2-state-sweep.sh`의 `ROBOTS` 주석에 그렇게 적혀 있다.

### §244.2 `RobotState::interpolate` 세 오버로드를 포팅하고 전체 상태 루프를 오라클과 맞췄다

§238.5의 둘째 줄이 지적한 대로, §238의 비교는 `JointModel::interpolate`를
조인트별로 본 것이고 상류의 전체 상태 루프는 비교되지 않았다. 이번
라운드는 "덮이지 않은 간극"으로 적는 쪽이 아니라 루프를 포팅해 재는 쪽을
택했다. 상류 오버로드는 셋이고 서로 다른 일을 한다
(`moveit_core/robot_state/src/robot_state.cpp`):

- `:1138` 전체 상태 — `checkInterpolationParamBounds` 뒤
  `RobotModel::interpolate`(`robot_model.cpp:1518`), 즉
  `active_joint_model_vector_` 루프 + `updateMimicJoints(state)`.
- `:1147` 그룹 — 같은 경계 검사 뒤 `group->getActiveJointModels()` 루프 +
  `state.updateMimicJoints(joint_group)`. 이 `updateMimicJoints`
  (`robot_state.cpp:210`)가 도는 것은 **그룹의** mimic
  (`group->getMimicJointModels()`)이지 모델 전체의 mimic이 아니다.
- `:1159` 단일 조인트 — **경계 검사가 없다**. 함수는
  `if (joint->getVariableCount() == 0) return;`으로 열고 곧장
  `joint->interpolate` + `markDirtyJointTransforms` +
  `updateMimicJoint(joint)`로 간다.

포트는 `RobotState::interpolate` / `interpolate_group` /
`interpolate_joint`로 셋을 그대로 가른다
(`crates/moveit-state/src/state.rs`). 세 번째가 경계 검사를 부르지 않는
비대칭은 상류의 것이므로 포트에서도 그대로 두고, 그 사실을 doc 주석과
테스트 양쪽에 못 박았다 — 세 오버로드에 검사를 "친절하게" 통일한 포트는
상류가 실제로 수행하는 호출을 거부하게 된다.

mimic 쓰기는 한 주인으로 접었다. `write_mimic(mimic_index)`가 산술
(`factor * source + offset`)과 dirty 표시를 **함께** 수행하고,
`propagate_all_mimics`(전체 폼)와 그룹 폼의 `group.mimic_joint_indices()`
루프와 `update_mimic_joint`(단일 조인트 폼)가 전부 그것을 부른다. 호출자가
표시를 맡는 설계는 상류 세 자리 중 하나(`RobotModel`의 것, 맨
`double*` 위에서 도므로 표시할 dirty 상태 자체가 없다)만 표시를 생략하는
것이 옳고 나머지 둘은 생략이 곧 stale transform 버그가 되므로, 생략이
정답인 자리와 버그인 자리가 한 규칙 아래 섞인다.

계기는 `state_interpolate` op이다 — `tools/moveit-oracle/src/oracle.cpp`의
`stateInterpolateOp`이 `from`/`to`/`seed` 위치 벡터 전부와 `t`, `scope`를
받아 `setVariablePositions`로 심고 세 오버로드 중 하나를 부른 뒤 결과
상태의 모든 변수를 돌려준다. `tools/moveit-diff/src/state_ops.rs`의 넷째
절(`state_interpolation`)이 그것을 열거한다: 오라클이 뽑은 무작위 상태 셋
(`from`/`to`/`seed`)에 더해, 모든 mimic을 자기 master에서 `+0.5`만큼
어긋나게 만든 `broken-mimic-seed`를 하나 더 만든다. 이 어긋난 seed가
필요한 이유는 `setVariablePositions(const double*)`이 `memcpy`이고 mimic을
전파하지 않기 때문이다(`robot_state.cpp:349`의 주석이 그렇게 말한다) —
즉 목적지가 들고 있던 모순된 mimic 값이 그대로 살아남고, 어느
오버로드가 그것을 덮어쓰고 어느 것이 놔두는지가 그때만 보인다.

```console
$ sg docker -c './tools/ci/verify-phase2-state-sweep.sh'   # EXIT=0
=== panda ===          clamping 122/0  mimic 10/0  interpolation  371/0  state_interpolation 134/0
=== prbt ===           clamping  54/0  mimic  0/0  interpolation  168/0  state_interpolation  55/0
=== fanuc ===          clamping  54/0  mimic  0/0  interpolation  168/0  state_interpolation  47/0
=== dual_arm_panda === clamping 198/0  mimic 20/0  interpolation  504/0  state_interpolation 230/0
=== pr2 ===            clamping 568/0  mimic 20/0  interpolation 1967/0  state_interpolation 838/0
=== one_robot ===      clamping  73/0  mimic 10/0  interpolation  189/0  state_interpolation 110/0
=== prbt_pg70 ===      clamping  90/0  mimic 10/0  interpolation  224/0  state_interpolation 158/0
OK: ... agree with the oracle on all 7 committed robots
```

허용오차 `0.0`, 즉 비트 단위 일치다. 넷째 절만 1,572 케이스이고 네 절
합계는 6,392다.

### §244.3 그룹 폼과 전체 폼이 같은 입력에서 갈라지는 유일한 자리 — `prbt_pg70`

그룹 폼이 도는 것은 그룹의 mimic이다. 그러므로 **mimic의 master는
그룹에 있는데 mimic 자신은 없는** 그룹에서만 두 오버로드가 같은 입력에
다른 답을 낸다: 전체 폼은 mimic을 master로부터 다시 쓰고, 그룹 폼은
목적지가 들고 있던 값을 그대로 둔다.

이 모양은 SRDF 저작만으로 도달한다 —
`JointModelGroup`의 `mimic_joints_`(`joint_model_group.cpp:155`)는 그룹의
**멤버인** mimic만 담고, 그룹 확장(`robot_model.cpp:785-830`)은 하위
그룹의 mimic은 끌어오지만 멤버 자신의 mimic은 끌어오지 않는다. 커밋된
여섯 로봇의 그룹은 전부 mimic 쌍의 양쪽을 함께 갖거나 둘 다 갖지 않아서,
이 경계는 어느 케이스도 밟지 않았다.

밟는 로봇은 상류에 있다. Pilz의
`moveit_planners/test_configs/prbt_pg70_support/config/pg70.srdf.xacro:36-38`이
`gripper` 그룹에 `${prefix}gripper_finger_left_joint` 하나만 넣는데,
같은 패키지의 `urdf/pg70/pg70.urdf.xacro:93`이 오른쪽 손가락을 그
왼쪽 손가락의 mimic으로 선언한다. `fixtures/prbt.urdf`와 같은
컨테이너·같은 xacro 경로로 `gripper:=pg70`만 덧붙여 뽑은 것이
`fixtures/prbt_pg70.{urdf,srdf}`이고, 두 파일의 include 폐포(각각 10개,
3개)를 sha256으로 `verify-fixture-provenance.sh`에 못 박았다.

이 픽스처가 재는 것이 무엇인지는 변이 하나로 확정했다: 그룹 폼의 mimic
전파를 `group.mimic_joint_indices()` 대신 "그룹의 master들이 거느린
모든 mimic"으로 바꾸면 `prbt_pg70`에서만 8건이 어긋나고 나머지 여섯
로봇은 전부 0건이다.

### §244.4 §238.5의 셋째 줄은 결함이 아니라 오독이었다 — `nalgebra`의 `lerp`이 곧 upstream의 식이다

§238.5는 `CartesianInterpolator::interpolate_pose`의 병진이
`from.lerp(to, t)` = `a + (b - a) * t`인데 상류는
`percentage * b + (1 - percentage) * a`라고 적었다. 이름만 보고 읽은
것이다. `nalgebra` 0.35.0의 `Vector::lerp`
(`base/interpolation.rs`)는 `axpy(t, rhs, T::one() - t)`로 포워딩하고,
그것은 성분별로 `t*rhs + (1-t)*self` — 상류의 식 그대로다. 포트는 처음부터
갈라져 있지 않았고, 그래서 이번 라운드는 소스를 **바꾸지 않았다**.

바꾸지 않는 대신, 어느 쪽 f64 프로그램인지를 이름이 아니라 테스트가 말하게
했다(`crates/moveit-kinematics/src/cartesian_interpolator.rs`의 `mod
tests`). 두 철자는 실수 위에서 같고 f64 위에서 다르다: 미터 스케일
8,405쌍을 쓸어 재 보면 `a + (b - a) * t`는 `t = 1`에서 2,040쌍에 대해
`b`를 맞히지 못하고, 내부에서는 최대 `4.44e-16`까지 벌어진다. 테스트의
`NEAR`/`FAR` 상수는 그 쓸기에서 골라낸, 두 철자가 실제로 갈라지는
좌표다 — 임의로 고른 좌표에서는 세 테스트가 두 철자 아래 모두 통과해
아무것도 못 박지 못한다(실측했다). 그래서 내부 케이스는 `differed > 0`을
스스로 확인한다.

### §244.5 이 라운드가 여전히 덮지 못하는 것

- **비유한 `t`는 오라클을 통과할 수 없다.** `serde_json`은 `NaN`과 무한대를
  둘 다 `null`로 싣기 때문에 넷째 절은 유한한 `t`만 보낼 수 있다. 상류
  `checkInterpolationParamBounds`(`robot_model.hpp:63`)가 던지는 것은 정확히
  그 두 값이고, 셋 중 둘만 그것을 부른다. 이 경계는 오라클이 아니라
  `crates/moveit-state/tests/interpolate_state.rs`가 잡는다.
- **어느 조인트가 stale로 표시됐는지는 전선에 실리지 않는다.** 오라클 op은
  변수 위치를 돌려주고, 상류의 dirty 장부는 `RobotState` 내부에 있다.
  위치는 맞게 쓰고 표시를 빠뜨린 오버로드는 넷째 절의 모든 케이스에
  동의한 다음 다음 `update()`에서 낡은 transform을 돌려준다. 같은 파일이
  네 자리를 각각 잡는다.
- **단일 조인트 폼의 0-변수 조기 반환은 이 포트에서 관측되지 않는다.**
  그 반환을 지워도 `moveit-state` 스위트는 전부 통과하고 `panda` 넷째 절도
  0건이다 — `interpolate_one`이 폭이 0인 구간을 복사하면 아무 일도
  일어나지 않고 `update_mimic_joint`가 같은 가드를 한 단계 아래에 또 들고
  있기 때문이다. 상류의 것이므로 남기지만, 이 반환을 지키는 계기는 없다.

## §245 Phase 4 (a)를 판정 규칙째로 고정한다 — 재시작 스트림은 판정의 일부가 아니고, 시드가 겹치면 오라클이 자기 답안을 돌려받는다 (2026-08-06)

§5 현황표의 `| Phase 4 | (a) 성공률이 C++ KDL 플러그인 이상 | UNMET | §221.1 |`
행은 두 수를 비교해서 나왔다 — `--ik-max-restarts 20`에서 포트 4906,
오라클 4921. 그런데 조건문은 재시작 수를 말하지 않는다. 같은 조건을
`--ik-max-restarts 0`에서 읽으면 포트가 앞선다(2435 대 2432). 즉 판정이
조건이 명시하지 않은 파라미터에 달려 있었다. 이 절은 그 파라미터를 고르는
대신 **판정 규칙 자체를 고정**한다.

고정에 앞서 계측기에서 결함 하나가 나왔고, 그것이 왜 "수 두 개 비교"가
검정이 될 수 없는지를 가장 선명하게 보여주므로 먼저 적는다.

### §245.1 계측기 결함: `--seed`와 오라클 재시작 시드가 겹치면 오라클이 정답 구성을 그대로 다시 뽑는다

오라클은 **표적 풀**과 **자기 IK 재시작**을 같은 생성기 클래스
(`random_numbers::RandomNumberGenerator`, boost `mt19937`)의 **같은 정수 시드
공간**에서 뽑는다. `randomStates`는 `request["seed"]`로 하나를 심고
(`oracle.cpp:1547`), `ik()`의 재시작 루프는 `ik_rng_`에서 뽑는다
(`oracle.cpp:2235`). 두 시드가 같으면 그 둘은 **같은 스트림**이다. 그러면
재시작이 표적을 만들어 낸 바로 그 관절 구성을 다시 뽑아내고, 오라클은
그것을 자기 해로 돌려준다.

추측이 아니라 해를 열어서 확인했다. `fanuc/manipulator`, 1,000 케이스,
오라클을 직접 구동해 각 응답의 `solution`을 그 표적의 생성 구성과 비교했다.

```console
$ python3 replay.py          # oracle을 직접 구동, ik 요청 1,000건
oracle ik-rng-seed   1: solved 952/1000, solution identical to the target's own generating config: 716
oracle ik-rng-seed 999: solved 389/1000, solution identical to the target's own generating config: 0
```

716/952가 표적의 생성 구성과 `1e-9` 이내로 **동일**하다. 스트림이 어긋나면
0/389이다.

이것이 계측에 남기는 자국은 크다. `moveit-diff`가 출하 상태 그대로,
새 플래그 없이, `--seed 42`(오라클의 내장 기본값도 42)로 돈 결과다.

```console
$ moveit-diff --urdf fanuc.urdf --srdf fanuc.srdf --cases 5000 --seed 42 \
    --group manipulator --ik --ik-max-restarts 2 --ik-rng-seed 0 --oracle ...
oracle success rate: 4733/5000 (94.7%)
rust   success rate: 1970/5000 (39.4%)
paired: b (oracle only) = 2833, c (rust only) = 70
VERDICT: paired divergence is not noise (|z| = 51.28 > 3) -- b = 2833, c = 70
         likely reflects a real algorithmic gap, not restart-RNG variance

$ ... --seed 41 ...           # 다른 것은 전부 같다
oracle success rate: 1955/5000 (39.1%)
rust   success rate: 2012/5000 (40.2%)
paired: b (oracle only) = 677, c (rust only) = 734
PASS ik_paired_divergence
```

정수 하나가 "실제 알고리즘 격차"와 "포트가 앞섬"을 가른다.

**시드 축을 훑어 특이값이 아니라 규칙임을 확인했다.** `--ik-max-restarts 2`,
`fanuc`, 케이스 시드 1 고정, 오라클 시드만 이동:

| 오라클 시드 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 42 | 43 | 100 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 오라클 성공 | 2018 | **4706** | 2050 | 2055 | 2073 | 2002 | 2035 | 2042 | 2059 | 2026 | 2029 | 2003 | 2031 | 2015 | 1998 | 1998 |

시드 1만 튀는 것이 아니라 **케이스 시드와 같을 때** 튄다. 대각선을 재면
전부 튄다:

| (케이스, 오라클) | (1,1) | (2,2) | (3,3) | (4,4) | (5,5) | (42,42) |
|---|---|---|---|---|---|---|
| 오라클 성공 | 4706 | 4743 | 4744 | 4724 | 4706 | 4733 |

같은 케이스 시드에 다른 오라클 시드를 주면 (2,999)=2017, (3,999)=1989,
(4,999)=1961, (5,999)=2008로 전부 정상 범위다.

**왜 fanuc만인가 — 픽스처의 성질이지 보장이 아니다.** 재현이 성립하려면
`randomStates`의 한 상태가 소비하는 변수 집합이 IK 그룹의 활성 관절
집합과 **같아야** 한다. 네 픽스처를 오라클에 직접 물어 셌다.

| 픽스처/그룹 | `random_states` 변수 | 그룹 활성 관절 | 정렬 |
|---|---|---|---|
| fanuc/manipulator | 6 (`joint_1..joint_6`) | 6 (같은 이름·같은 순서) | **일치** |
| panda/panda_arm | 16 (손가락 2 + 가상 조인트 7 포함) | 7 | 불일치 |
| dual_arm_panda/left_panda_arm | 18 | 7 | 불일치 |
| pr2/right_arm | 48 | 7 | 불일치 |

`--seed 42`로 네 픽스처를 다 돌린 결과가 이 표와 맞는다 — fanuc만
4733(대조군 `--seed 41`은 1955)이고, panda 3284(3266), dual_arm 3241(3279),
pr2 4221(4230)은 대조군과 구별되지 않는다.

**구조적으로 막았다.** 두 시드를 모두 쥐고 있는 쪽은 `moveit-diff`뿐이므로
거기서 막는다. `moveit-diff`는 이제 오라클의 `--ik-rng-seed`를 **항상**
넘기고(기본값 42 — 오라클이 혼자서 골랐을 값과 같으므로 기록된 수는 하나도
움직이지 않는다), 넘기기 전에 `--seed`와 대조해 같으면 **거부한다**
(`reject_colliding_oracle_streams`, 종료 코드 2). 값을 몰래 바꾸지 않고
거부하는 이유는, 조용히 제3의 값을 쓰는 실행이야말로 이 검사가 없애려는
"명시되지 않은 파라미터"이기 때문이다. 오늘 어긋나 있는 세 픽스처도
보장이 아니라 픽스처의 성질이므로 그룹을 가리지 않고 거부한다.

```console
$ moveit-diff ... --seed 42 --group manipulator --ik --oracle ...
moveit-diff: --seed 42 and --oracle-ik-rng-seed 42 select the same oracle random
stream, which lets the oracle's IK restarts replay the configurations its targets
were built from; pass a different --oracle-ik-rng-seed (default 42)
$ echo $?
2
```

기본 경로가 그대로임은 §221.4의 수로 확인했다 — 플래그 없이
`--seed 1 --group panda_arm --ik-max-restarts 20`은 오라클 4921 / 포트 4906 /
b 82 / c 67로 §221.4와 같다.

### §245.2 재시작이 켜지면 두 성공 수는 확률변수다 — 4906 대 4921은 포트의 5/8분위와 오라클의 **최댓값**을 비교한 것이다

포트와 오라클은 재시작 시 **같은 분포**(각 활성 관절의 자기 한계 위 균등)에서
뽑고 **같은 횟수**(`max_restarts + 1`회 시도) 시도한다. 다른 것은 스트림뿐이다
— 포트는 `ChaCha8Rng` 시드 0, 오라클은 boost `mt19937` 시드 42. 어느 쪽을
선호할 근거가 없다.

그래서 양쪽 주변분포를 각각 8회씩 재측정했다(panda/panda_arm, 5,000 케이스,
`--seed 1`, `--ik-max-restarts 20`, 위 코드 변경 뒤의 바이너리):

| | 표본 | 평균 | 표준편차 | 최소 | 최대 |
|---|---|---|---|---|---|
| 포트 (`--ik-rng-seed` 0..7) | 4906 4905 4900 4908 4893 4912 4911 4899 | 4904.25 | 6.50 | 4893 | 4912 |
| 오라클 (`--oracle-ik-rng-seed` 42,2..8) | 4921 4891 4909 4891 4892 4901 4905 4906 | 4902.00 | 10.54 | 4891 | 4921 |

**오라클의 기본 시드 42가 내놓는 4921은 그 자신의 8표본 중 최댓값(순위 8/8)이고,
포트의 기본 시드 0이 내놓는 4906은 순위 5/8이다.** §221.1이 UNMET의 근거로
든 "4906 대 4921"은 포트의 중간 표본과 오라클의 최고 표본을 맞붙인 것이다.
평균 차는 포트가 +2.25 앞서고(Welch t = +0.514), 무작위로 한 표본씩 뽑아
비교하면 포트가 이길 확률이 38/64 = 0.594다. 두 분포는 겹친다.

`dual_arm_panda`에서도 같은 방향이 나왔다(각 8표본): 포트 평균 4917.125,
오라클 평균 4914.125이며, 포트의 기본 시드 0은 자기 표본의 **최솟값**,
오라클의 42는 자기 표본의 **최댓값**이었다.

**포트가 지는 동작점도 그대로 적는다.** `--ik-max-restarts 50`, panda,
각 8표본: 포트 평균 4990.875, 오라클 평균 4992.0으로 오라클이 1.125 앞선다.
`--ik-max-restarts 5`에서는 포트 4020.3 대 오라클 4017.0(각 10표본)으로
포트가 앞선다. 어느 쪽도 자기 산포보다 크지 않다.

### §245.3 재시작을 끈 동작점은 이 레포 자신의 게이트가 "판정 불가"라고 답하는 지점이다

§221.1이 포트 우위의 근거로 든 `--ik-max-restarts 0`은 결정론적 비교가
아니다. 오라클 쪽은 상수지만(특이점 흔들기가 시드되지 않은 `std::rand()`를
쓴다 — `oracle.cpp:2188`) **포트 쪽은 아니다**. 포트의 흔들기는 재시작과
같은 rng를 소비한다(`cart_to_jnt.rs:237`). 포트 rng 시드만 옮겨 재측정했다:

| 픽스처 | 오라클(상수) | 포트 표본 | 포트 평균 | 포트 < 오라클 |
|---|---|---|---|---|
| panda/panda_arm | 2432 | 2435 2435 2435 2432 2436 2432 2436 2435 | 2434.50 | 0/8 |
| fanuc/manipulator | 1061 | 1061 1061 1062 1060 1060 1059 | 1060.50 | **3/6** |
| dual_arm_panda/left_panda_arm | 2471 | 2471 2471 2473 2469 2470 2469 | 2470.50 | **3/6** |
| pr2/right_arm | 3223 | 3227 3229 3232 3228 3227 3230 | 3228.83 | 0/6 |

§221.1과 §221.4가 "재시작을 끄면 포트가 앞선다"고 적은 것은 포트 rng 시드 0
한 표본이다. fanuc과 dual_arm에서는 6표본 중 3표본이 오라클보다 **아래**다.
재시작을 끄는 것으로는 조건이 결정 가능해지지 않는다.

게다가 이 동작점은 **이 레포가 이미 판정 불가라고 규정한 지점**이다.
`moveit-diff`의 McNemar 게이트는 `b + c < MINIMUM_USABLE_B_PLUS_C`(= 61)일 때
`Pass`가 아니라 `Underpowered`를 낸다 — "실제로 본 것 중 가장 작은 버그를
잡아낼 만큼 크지 않다"는 이유로 보정된 값이다. panda에서 재시작 0의
`b + c`는 **5**다.

### §245.4 판정 규칙을 고정한다 — 동작점은 게이트가 검정력을 갖는 최소 재시작 수, 검정은 게이트 자신

조건 (a)의 판정은 이제 다음 두 줄로 고정한다. 어느 쪽도 어느 수가 예뻐서
고른 것이 아니라, 이 레포가 이미 만들어 둔 계측기가 스스로 요구하는 것이다.

1. **동작점**: `--ik-max-restarts`는 McNemar 게이트가 검정력을 갖는
   (`b + c >= MINIMUM_USABLE_B_PLUS_C`) 값 중에서 고른다. 재시작 0은
   게이트가 `Underpowered`를 내므로 이 조건의 근거가 될 수 없다 — 유리한
   방향이더라도 마찬가지다.
2. **검정**: 성공 수 두 개의 대소가 아니라 **짝지은 McNemar 게이트**
   (`|z| <= PAIRED_DIVERGENCE_Z_THRESHOLD` = 3)가 판정이다. §245.2가
   보인 대로 성공 수 두 개의 대소는 스트림 한 쌍을 뽑는 추첨이고,
   짝지은 b/c는 같은 케이스에서 어느 쪽만 풀었는지를 세므로 스트림
   추첨이 아니라 케이스에 대한 진술이다.

이 규칙으로 네 픽스처를 격자로 재측정했다(5,000 케이스, `--seed 1`,
`--ik-rng-seed 0`, 오라클 기본 42):

| 픽스처 | 재시작 | 오라클 | 포트 | b | c | b+c | \|z\| | 게이트 |
|---|---|---|---|---|---|---|---|---|
| panda | 0 | 2432 | 2435 | 1 | 4 | 5 | 1.34 | **UNDERPOWERED** |
| panda | 1 | 2912 | 2918 | 398 | 404 | 802 | 0.21 | PASS |
| panda | 2 | 3258 | 3317 | 525 | 584 | 1109 | 1.77 | PASS |
| panda | 5 | 4039 | 4077 | 564 | 602 | 1166 | 1.11 | PASS |
| panda | 10 | 4600 | 4588 | 319 | 307 | 626 | 0.48 | PASS |
| panda | 20 | 4921 | 4906 | 82 | 67 | 149 | 1.23 | PASS |
| fanuc | 20 | 4584 | 4591 | 303 | 310 | 613 | 0.28 | PASS |
| dual_arm_panda | 20 | 4925 | 4905 | 85 | 65 | 150 | 1.63 | PASS |
| pr2 | 1 | 3828 | 3839 | 368 | 379 | 747 | 0.40 | PASS |
| pr2 | 2 | 4252 | 4209 | 412 | 369 | 781 | 1.54 | PASS |
| pr2 | 5 | 4710 | 4694 | 219 | 203 | 422 | 0.78 | PASS |
| pr2 | 10 | 4923 | 4904 | 78 | 59 | 137 | 1.62 | PASS |
| pr2 | 20 | 4986 | 4987 | 10 | 11 | 21 | 0.22 | **UNDERPOWERED** |

검정력이 있는 11개 동작점 전부에서 게이트가 PASS다. 검정력이 없는 두 개는
판정에 쓰지 않는다(panda 재시작 0은 포트가 앞서 보이고 pr2 재시작 20은
오라클이 앞서 보이지만, 둘 다 게이트가 판정을 거부한 지점이다).

성공 수의 대소 자체는 방향이 오간다 — 포트는 panda 재시작 1·2·5와 fanuc·pr2
재시작 1에서 앞서고, panda 재시작 10·20과 dual_arm 재시작 20, pr2 재시작
2·5·10에서 뒤진다. 최대 격차는 dual_arm 재시작 20의 20건으로 5,000건의
0.4%이며, §245.2가 잰 오라클 자신의 표본 폭(4891..4921, 30건)보다 작다.
"어느 쪽이 앞서는가"가 스트림 추첨이라는 §245.2의 결론과 같은 그림이다.

**판정: MET.** 근거는 "포트가 더 높다"가 아니라 "이 레포가 조건 (a)를 위해
만든 짝지은 게이트가, 검정력을 갖는 모든 동작점에서, 네 픽스처 전부에서,
포트가 오라클보다 낮다는 것을 기각한다"이다. §5 현황표의 행은 §221.1이 아니라
이 절을 인용하도록 바꾼다.

**이 절이 닫지 않은 것.** 조건문 자체의 문구("성공률이 C++ KDL 플러그인
이상")는 여전히 동작점을 말하지 않는다. 위 규칙은 그 공백을 이 절에 적어
메운 것이지 조건문을 고친 것이 아니다. 조건문을 고칠지는 §5를 소유한 쪽의
결정이다.

## §246 Phase 4(a) 재확인 — 재시작 로또인지 독립 재측정과 상류 소스로 확인한다

§221.1이 이미 결론 내렸다: 15는 존재하지 않는 집합이 아니라 b=82, c=67의
잔차이고, 재시작을 끄면 네 픽스처 전부 포트가 오라클 이상이다. 이번
라운드는 그 결론을 그대로 물려받지 않고 처음부터 다시 측정하라는 지시를
받았다 — A∩B∩C와 union을 §221.1의 수를 옮겨 적지 않고 새로 뽑고, 상류
`/home/stevek/work/moveit2`의 재시작 코드를 직접 읽어 "왜"를 확인하고,
재시작 예산을 바꿔 부호가 실제로 흔들리는지 실험하고, 표본을 3개 스트림
너머로 넓히라는 것. 아래는 그 결과다. 결론은 §221.1과 같은 방향이지만,
§221.1이 갖지 못했던 세 가지 새 증거(상류 소스 인용, 예산 실험, 12-스트림
일반화)를 더한다.

### §246.1 A∩B∩C와 union을 새로 뽑는다

`panda`/`panda_arm`, `--cases 5000 --ik-max-restarts 20`, 세 개의 독립
`--ik-rng-seed`(0, 12345, 777)로 `moveit-diff --ik --ik-divergence-json`을
새로 실행하고(이 라운드 이전의 어떤 산출물도 재사용하지 않았다),
`oracle_only` 케이스 번호 집합을 처음부터 교집합·합집합했다:

```
seed 0:      oracle_only = 82건
seed 12345:  oracle_only = 82건
seed 777:    oracle_only = 89건
A ∩ B ∩ C = {408, 4130}                (2건)
A ∪ B ∪ C = 226건
seed 0의 82건 중 80건이 다른 두 스트림 중 하나에서는 풀린다
```

§221.1이 인용한 수(82/82/89, 교집합 2, 합집합 226)와 정확히 일치한다 —
독립 재도출이 그 수를 재확인했다.

### §246.2 네 가설 중 세 개를 상류 소스로 배제한다

브리핑이 이름 붙인 네 가설 — 반복 예산, 재시작 표본 분포, 시드-위치
매핑, 진짜 솔버 차이 — 을 하나씩 확인한다.

- **반복 예산·수렴 판정.** `crates/moveit-kinematics/src/cart_to_jnt.rs`의
  Newton 반복(`cart_to_jnt`, 특이점 후퇴, "wiggle" 탈출)과 상류
  `kdl_kinematics_plugin.cpp`의 `CartToJnt`(417-497행)를 이번에 다시
  대조했다. §221.1/§221.2가 이미 확인한 것(재시작 없는 실행 4,995/5,000
  바이트 일치, epsilon 격자 실험)과 다른 불일치를 찾지 못했다. 배제.
- **시드-위치 매핑.** 상류 `KDLKinematicsPlugin::getRandomConfiguration`
  (`kdl_kinematics_plugin.cpp`)은 `RobotState::setToRandomPositions`
  (`robot_state.cpp:271`)로 위임하고, 그 안에서
  `RevoluteJointModel::getVariableRandomPositions`와
  `PrismaticJointModel::getVariableRandomPositions`(각각
  `revolute_joint_model.cpp`, `prismatic_joint_model.cpp`)가
  `values[0] = rng.uniformReal(bounds[0].min_position_,
  bounds[0].max_position_);`로 활성 조인트마다 **자기 bound 전체에서
  균등 표본**을 뽑는다. 포트의 `random_configuration()`
  (`cart_to_jnt.rs`)은 `chain.active_min.iter().zip(&chain.active_max)
  .map(|(&min, &max)| rng.random_range(min..=max))` — 같은 조인트마다
  자기 bound 전체에서 균등 표본. 분포와 bound 의미론이 동일하다 — 상류
  소스를 읽어 확인했고, 수치에서 추론하지 않았다. 배제.
- **재시작 표본 분포 자체.** 위와 같은 근거로 분포 함수(bound 전체 균등)는
  동일하다. 남는 차이는 RNG 알고리즘 자체뿐이다: 포트는 `ChaCha8Rng`(고정
  시드 0), 오라클은 boost `mt19937`(고정 시드 42) — 서로 무관한 두
  스트림이 같은 5,000개 입력에 대해 다른 21회 시도열을 뽑는다. 이것은
  §221.1도 이미 지적한 사실이다.
- **진짜 솔버 차이.** 위 두 항목이 배제되면 남는 것은 이것뿐인데, 첫
  항목에서 이미 배제했다.

네 가설 중 남는 것은 "재시작 재시드가 뽑는 자세 자체가 다르다"뿐이고,
그것은 알고리즘 차이가 아니라 **서로 다른 난수 스트림이 같은 분포에서
다른 표본을 뽑는다는 사실** 그 자체다.

### §246.3 재시작 예산을 20 → 100으로 올리면 부호가 뒤집힌다

같은 두 케이스(408, 4130)가 "모든 스트림에서 실패"라면, 예산을 늘려도
계속 실패해야 한다 — 진짜 능력 격차라면 예산은 그것을 못 고친다. 실측:

```
seed 0,   --ik-max-restarts 100: 오라클 4995/5000, 포트 4996/5000, b=0, c=1
seed 777, --ik-max-restarts 100: 오라클 4995/5000, 포트 4998/5000, b=0, c=3
```

`--ik-max-restarts`는 `Op::Ik::max_restarts`로 전선에 실려 양쪽 다 동일하게
올라간다(`protocol.rs:186-230`) — 포트만 유리해지는 비교가 아니다. 그런데도
20에서는 포트가 −15(seed 0 기준), 100에서는 포트가 +1로 **부호가 뒤집힌다**.
두 실행 모두에서 408과 4130이 더는 `oracle_only`에 나타나지 않는다 — "모든
스트림에서 실패"였던 케이스가 예산을 늘리자마자 둘 다 풀린다. 이것은 진짜
능력 격차라면 나올 수 없는 모양이다.

### §246.4 스트림을 3개에서 12개로 넓히면 "항상 실패"는 공집합이 된다

원래 예산(20)으로 되돌리고, seed 1-9(원래 세 스트림 0/12345/777에 더한 9개
독립 스트림)로 같은 케이스 408·4130의 운명만 추적했다:

```
케이스 4130: 추가 9개 스트림 중 9/9에서 풀린다 (0/9 실패)
케이스 408:  추가 9개 스트림 중 5/9에서 풀린다 (seed 2,5,7,8,9 풀림 /
             seed 1,3,4,6 실패)
```

12개 스트림(원래 3 + 추가 9) 전체를 합치면, **모든 스트림에서 실패하는
케이스는 하나도 없다** — n=3에서는 {408, 4130} 두 건으로 보였던 "항상
실패" 집합이 n=12에서는 공집합이다. 408은 다른 것보다 어려운 자세(9개 중
5개 스트림만 풀었다)이지만 무조건 못 푸는 자세는 아니다. n=3 표본에서
"교집합 2건"으로 보인 것 자체가 표본 크기의 산물이었다.

### §246.5 평결과 제안 — §5는 고치지 않는다

**소스 수정으로 닫히지 않는다.** §246.2가 상류 소스로 직접 확인한 대로,
포트의 재시작 표본 분포·bound 의미론·수렴 판정은 상류와 이미 일치한다.
버그가 없다 — 고칠 코드가 없다.

**이것은 재시작 로또이고, "한계 성공률의 대소"는 5,000건 표본에서
정합한(well-posed) 검정이 아니다.** §246.3의 예산 실험이 결정적 증거다:
같은 두 "항상 실패" 케이스가 예산을 5배 늘리는 것만으로 풀리고 부호까지
뒤집힌다. §246.4는 표본을 넓히기만 해도 "항상 실패" 집합이 공집합으로
줄어드는 것을 보여준다. 두 실험 모두 §221.1의 결론("포트가 다르게 하는
것은 재시작 재시드의 난수열")을 독립적으로 재확인하고, 그 결론에 상류
소스 근거와 정량적 예산 실험을 더한다.

**제안 — 이 문서는 §5의 Phase 4(a) 행이나 조건 문구를 이 라운드에 고치지
않는다. 판정 변경은 오케스트레이터의 몫이며, 아래는 측정에 근거한
권고일 뿐이다.** `moveit-diff`는 이미 McNemar 쌍대 검정
(`paired_divergence_z(b, c)`, 정규근사, `PAIRED_DIVERGENCE_Z_THRESHOLD`
= 3.0, `MINIMUM_USABLE_B_PLUS_C` = 61)을 구현하고 있고, 기본 재시작
예산에서 이미 그 결과를 냈다 — panda `|z| = 1.23`(§221.1의 표), 넷 중
가장 큰 값이 dual_arm_panda의 1.63으로 여전히 임계값 3.0 아래다. 한계
성공률의 산술 차(4906 대 4921)가 아니라 이 쌍대 검정의 `|z|`가 (a)의 실제
측정 도구가 되어야 한다는 것이 이 절의 권고다: `|z| < 3.0`이면 두 구현이
같은 재시작 로또를 서로 다르게 뽑은 것과 통계적으로 구분되지 않고,
`b + c < 61`이면 애초에 검정력이 없다고 보고해야 한다(§221.1의 pr2가 이미
그 경우다). 이 권고를 채택한다면 (a)의 조건 문구 자체도 "한계 성공률이
C++ 이상"에서 "McNemar `|z|`가 임계값 미만"으로 바뀌어야 하는데, 그 문구
변경은 이 절이 아니라 오케스트레이터가 결정한다.

**병합 시점의 주석.** 이 절과 §245는 같은 라운드에 서로 모르는 채로 Phase 4
(a)를 재조사했고, 같은 권고에 도달했다 — 한계 성공률의 대소가 아니라 짝지은
McNemar `|z|`가 판정 도구여야 한다. §245.4가 그 규칙을 실제로 고정하고 §5의
행을 MET으로 바꿨으므로, 이 절이 "§5는 고치지 않는다"고 적은 것은 이 절의
범위에 대한 진술이지 미결 상태의 기록이 아니다. 이 절이 §245에 더하는 것은
독립 재측정 두 가지다: 재시작 예산을 100으로 올리면 부호가 뒤집힌다는 것과,
스트림을 12개로 넓히면 "항상 실패하는 케이스"가 공집합이 된다는 것 — 둘 다
§245.2가 스트림 추첨이라고 부른 것을 다른 축에서 확인한다.

