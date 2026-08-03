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

## 13. `moveit-octomap` 착수 — §1.3 "핵심 공백 3개" 중 하나 해소 (2026-08-03)

`23867d6` 병합. 커밋 셋: `bbed614`(octomap 1.9.7 점유 옥트리 이식),
`519fe37`(오라클 `octomap` op), `c43b78d`(경계 시나리오 4건 패리티 테스트).

§1.2 표가 `bye_octomap_rs` 0.1.1을 **성숙도 미달**로 판정하고 §1.3이
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
(`crates/moveit-scene/src/scene.rs:583` `frame_transform`,
`:641` `knows_frame_transform` — 모델 프레임/링크 → 부착체 id/서브프레임 →
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
||||||| 3a4a9c3

## 36. `resolveConstraintFrames` 이식 — 그리고 구성 시점 검증이 만든 API 형태 변경 (4라운드, 2026-08-04)

§23.1이 남겨둔 차단 사유(`RobotState`/`Posed`에 부착체/서브프레임을
이름으로 찾는 API가 없음)는 같은 병합 라운드에서 `p1-fixtures`가
`PlanningScene` 레벨(`scene.rs:583`, `:641`)로 해소했다 — `RobotState`
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
isStateValid/isPathValid`인데, 본문과 `scene.rs:874`의 문서는
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

### 51.2 소유권 블록이 `moveit-octomap`을 빠뜨리고 있었다

최근 라운드 브리핑들의 ownership 절이 p3-shapes를 `moveit-geometry/`로만
적었다. `moveit-octomap`은 §13에서 p3-shapes가 만든 크레이트이고 전용
태스크 파일까지 있었는데, 최근 블록에서 이름이 빠졌다. 고아 크레이트는
아니고 브리핑 표기 누락이다 — 그 1건도 p3-shapes 소유로 센다.
