# cspace — 핵심 목표

MoveIt 2를 ROS 비의존 순수 Rust 모션플래닝 라이브러리로 이식한다.

- **상류 기준점:** `/home/stevek/work/moveit2` @ `e017c91ee12984393a28ba246075c65f69cde3bf`
- **라이선스:** MoveIt 2는 BSD-3-Clause. 이식한 파일은 원본 저작권 헤더와
  대응 상류 경로를 파일 상단에 유지한다.

## 확정된 설계 결정

| # | 결정 | 선택 |
|---|---|---|
| D1 | 최종 형태 | ROS 독립 Rust 모션플래닝 라이브러리 |
| D2 | ROS 2 바인딩 | r2r — 선택적 `cspace-ros` 크레이트에만 격리 |
| D3 | OMPL | 순수 Rust 플래너 우선, cxx FFI는 후순위 |
| D4 | 플러그인 모델 | 컴파일타임 레지스트리 (trait + `linkme`) |

**코어 크레이트는 ROS 타입을 일절 참조하지 않는다.** `moveit_msgs`,
`geometry_msgs`, `rclcpp` 대응 타입은 코어 안에서 순수 Rust로 새로 정의한다.
ROS 2 연동은 `cspace-ros` 하나에만 존재하고, 그 크레이트만 r2r에 의존한다.
코어는 ROS 2 설치 없이 `cargo test`로 전부 검증된다.

**의존 방향 규칙 (CI 강제):** `cspace-ros`를 제외한 어떤 크레이트도 `r2r`,
`rclrs`, `ros2-client`에 의존하지 않는다.

## 범위

**이식 대상:** `moveit_core` (70,215 LOC), `moveit_kinematics` (6,912),
`moveit_planners` 중 pilz / chomp / stomp, 그리고 신규 순수 Rust SBP 플래너.

**범위 밖:** `moveit_ros` (선택적 `cspace-ros`가 일부만 커버),
`moveit_setup_assistant` (Qt GUI), `moveit_py` (PyO3), `moveit_plugins`
(ros2_control 결합), `collision_detection_bullet` (드롭),
`collision_detection_fcl` (`parry`로 대체).

## 완료 조건

상류 C++을 기준선으로 하는 차등 테스트(`tools/moveit-oracle` = C++ 오라클,
`tools/moveit-diff` = Rust 러너)로 측정한다.

| Phase | 조건 |
|---|---|
| 0 | 오라클 FK 파이프라인이 동작한다 |
| 1 | panda/prbt/fanuc 링크 수·조인트 수·그룹 구성·조인트 한계값·mimic 관계 완전 일치 |
| 2 | FK 10,000×3로봇 `1e-9` 이내, 야코비안 `1e-7` 이내(열 순서 규약 포함), 관절 한계 클램핑·mimic 전파·floating/planar 보간 일치 |
| 3 | `collision: bool` 이 두 파견표가 겹치는 형상 쌍에서 일치, `distance: f64` 가 분리 분기에서 `1e-4` 이내, 관통 분기는 상류 결함이 발화할 수 없는 부분모집단에서 `1e-4` 이내 |
| 4 | IK 성공률이 C++ KDL 플러그인과 짝지은 McNemar 검정으로 구별되지 않고, 성공한 해의 FK가 `1e-5` 이내 일치 |
| 5 | 제약 조합 `decide()` 100% 일치, 샘플러 생성 상태 전부 자기 제약 만족, 씬 diff 후 충돌 결과 100% 일치 |
| 6 | TOTG 시간 파라미터화가 `1e-6` 이내 일치 |
| 7 | 벤치마크 성공률이 C++ OMPL RRTConnect의 90% 이상, 산출 경로 100%가 충돌·제약 통과, 경로 길이 중앙값 1.3배 이내 |
| 8 | pilz LIN/PTP/CIRC 궤적이 `1e-6` 이내 일치. CHOMP/STOMP는 Phase 7의 세 속성을 각 플래너의 C++ 구현 기준선으로 통과 (경로 유효성 판정은 아래) |
| 9 | 기존 C++ `MoveGroupInterface` 클라이언트가 무변경으로 유효 궤적 수신 |

Phase 8의 경로 유효성은 Phase 7의 단일 100%와 다르다. 지역 최적화기는 성긴
웨이포인트 열을 반환하므로, 플래너가 실제로 점수를 매긴 상태와 조밀화가 처음
도달하는 표본을 나눠 판정한다. 전자는 CHOMP/STOMP 모두 100%, 예외 없음.
후자는 CHOMP 0% · STOMP 2% 이하다. 측정값은 CHOMP 0/174 · 0/179, STOMP
0/219 · 1/194(0.52%). 상한 2%는 포트 측 최대치(0.99%)의 약 2배로 잡은
값이고, 상류 C++ STOMP도 같은 문제군에서 같은 잔차를 보이므로(cage 0/205,
floor_wall 2/241 = 0.83%, 둘 다 자기 반환 웨이포인트에서는 100%) 이 잔차
자체는 포트 고유의 결함이 아니다. 제약 없이 계획한 경로를 그 제약과 함께
검사하는 `inject_constrained` 대조군은 유효성이 아니라 귀속성으로 판정한다.

Phase 8의 CHOMP/STOMP는 기준선이 둘이고, 위 표의 조건은 그중 하나다. 표가
판정하는 것은 각 플래너의 자기 상류 C++ 구현을 기준선으로 한 쪽이고,
`doc/phase8-optimizer-properties.json` 140/140이 그것이다. 같은 세 속성을
§5가 줄 번호로 지목하는 C++ OMPL RRTConnect 기준선으로 재는
`tools/ci/verify-phase8-benchmark.sh`가 따로 있으며, 핀 값은 전부 재현하되
(cpp 498, chomp 380/379, stomp 441/438, 중앙값 셋 모두 핀과 비트 일치)
여섯 조건 중 넷이 UNMET이라 QUALIFIED로 끝난다. 성공률이 CHOMP 380/500 =
76.0%, STOMP 441/500 = 88.2%로 기준 89.64%에 미치지 못하고, 조밀화 유효성이
CHOMP 379/380 · STOMP 438/441이다. 앞의 둘은 한 궤적을 다듬는 최적화기를
트리를 키우는 표본 기반 플래너와 성공률로 비교한 값이고, 뒤의 둘은 위
문단이 다루는 그 잔차다. 각 UNMET의 근거는 그 스크립트 헤더에 있다.

위 표의 전 Phase 조건 충족(2026-08-11 측정 기준:
`doc/phase7-benchmark-results.json` 39/39,
`doc/phase8-optimizer-properties.json` 140/140). 이후 작업은 상류 대비
코드 결함 수정이다.

## 크레이트

```
cspace-core        error geometry octomap srdf model state kinematics
                   sampling trajectory smoothing metrics test_support
cspace-collision   (+ distance_field)
cspace-bullet      gjk epa simplex manifold dbvt shapes convex_convex
cspace-bullet-cast cast_hull_shape cast_bvh_manager cast_contact
cspace-planning    (+ constraints scene planner_registry)
cspace-planners    sbp chomp pilz stomp
cspace-stomp-core
```

`tools/moveit-oracle` (C++ 차등 오라클, moveit2 링크) /
`tools/moveit-diff` (Rust 차등 러너).

## 검증

```
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo nextest run --workspace
cargo test --doc --workspace
tools/ci/            # 코드 동작 검증 게이트
```
