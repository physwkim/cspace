# moveit-rs — 핵심 목표

MoveIt 2를 ROS 비의존 순수 Rust 모션플래닝 라이브러리로 이식한다.

- **상류 기준점:** `/home/stevek/work/moveit2` @ `e017c91ee12984393a28ba246075c65f69cde3bf`
- **라이선스:** MoveIt 2는 BSD-3-Clause. 이식한 파일은 원본 저작권 헤더와
  대응 상류 경로를 파일 상단에 유지한다.

## 확정된 설계 결정

| # | 결정 | 선택 |
|---|---|---|
| D1 | 최종 형태 | ROS 독립 Rust 모션플래닝 라이브러리 |
| D2 | ROS 2 바인딩 | r2r — 선택적 `moveit-ros` 크레이트에만 격리 |
| D3 | OMPL | 순수 Rust 플래너 우선, cxx FFI는 후순위 |
| D4 | 플러그인 모델 | 컴파일타임 레지스트리 (trait + `linkme`) |

**코어 크레이트는 ROS 타입을 일절 참조하지 않는다.** `moveit_msgs`,
`geometry_msgs`, `rclcpp` 대응 타입은 코어 안에서 순수 Rust로 새로 정의한다.
ROS 2 연동은 `moveit-ros` 하나에만 존재하고, 그 크레이트만 r2r에 의존한다.
코어는 ROS 2 설치 없이 `cargo test`로 전부 검증된다.

**의존 방향 규칙 (CI 강제):** `moveit-ros`를 제외한 어떤 크레이트도 `r2r`,
`rclrs`, `ros2-client`에 의존하지 않는다.

## 범위

**이식 대상:** `moveit_core` (70,215 LOC), `moveit_kinematics` (6,912),
`moveit_planners` 중 pilz / chomp / stomp, 그리고 신규 순수 Rust SBP 플래너.

**범위 밖:** `moveit_ros` (선택적 `moveit-ros`가 일부만 커버),
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
| 8 | pilz LIN/PTP/CIRC 궤적이 `1e-6` 이내 일치. CHOMP/STOMP는 Phase 7의 세 속성을 각 플래너의 C++ 구현 기준선으로 통과 |
| 9 | 기존 C++ `MoveGroupInterface` 클라이언트가 무변경으로 유효 궤적 수신 |

전 Phase 조건 충족(2026-08-06 측정 기준). 이후 작업은 상류 대비 코드 결함
수정이다.

## 크레이트

```
moveit-error          moveit-geometry       moveit-srdf
moveit-model          moveit-state          moveit-metrics
moveit-collision      moveit-distance-field moveit-octomap
moveit-scene          moveit-constraints    moveit-sampling
moveit-trajectory     moveit-smoothing      moveit-kinematics
moveit-planning       moveit-planner-registry
moveit-planners-sbp   moveit-planners-chomp
moveit-planners-stomp moveit-stomp-core     moveit-planners-pilz
moveit-test-support
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
