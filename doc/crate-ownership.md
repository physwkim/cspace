# 크레이트 소유권 — 스냅샷 2026-08-04

`PORTING-PLAN.md` §175는 "감사 목록을 컨텍스트에만 두면 압축이 그것을
지운다"를 워커들에게 요구한다. **이 파일은 그 규칙을 나 자신에게 적용한
결과다.** 소유권 맵은 지금까지 조율자(main 패널)의 대화 컨텍스트에만
있었고, 그 결과가 이미 두 번 나왔다:

1. `p1-fixtures` 라운드 10이 `moveit-planning`의 감사 발견 7건을 "내
   것이 아님(not mine)"으로 분류했다 — **실제로는 그 패널 소유다.**
   자기 소유 크레이트의 결함을 남의 것으로 넘긴 것이고, 아무도 안 고치는
   결과로 이어질 뻔했다.
2. 이 파일을 쓰면서 크레이트를 세어 보니 **`moveit-srdf`는 어떤
   패널에게도 배정된 적이 없다.** 컨텍스트 안의 맵에는 20개가 있었고
   트리에는 21개가 있다. 세어보기 전에는 몰랐다.

두 번째가 이 파일의 존재 이유를 그대로 보여준다 — 맵이 파일이면 셀 수
있고, 셀 수 있으면 빠진 것이 드러난다.

## 배정

| 소유자 | 크레이트/경로 |
|---|---|
| `p3-shapes` | `moveit-geometry`, `moveit-octomap`, `moveit-planners-stomp`, `moveit-sampling`, `moveit-stomp-core` |
| `p3-acm` | `moveit-model`, `moveit-collision` |
| `p3-distance-field` | `moveit-distance-field` |
| `p1-fixtures` | `moveit-scene`, `moveit-metrics`, **`moveit-planning`** |
| `p1-joints` | `moveit-kinematics`, `moveit-planners-pilz`, `moveit-state`, `tools/moveit-diff` |
| `p1-robotmodel` | `moveit-constraints`, `moveit-planners-sbp` |
| `p6-totg` | `moveit-trajectory`, `moveit-smoothing`, `moveit-planners-chomp` |
| `p9-ros` | `ros/` 전체 |
| 조율자(main) | `tools/moveit-oracle/`, `tools/ci/`, `PORTING-PLAN.md`, `doc/`, `.github/`, 루트 `Cargo.toml` |
| 조율자(main) — 크레이트 | `moveit-error`, `moveit-srdf` |

21개 크레이트 전부가 위 표에 나온다. 새 크레이트를 만들면 여기에
줄을 추가하는 것이 그 라운드의 일부다.

## 미배정 두 건 — 해소됨

미배정 크레이트에 대한 규칙은 "조율자가 직접 처리하거나 다음 라운드에
소유자를 지정한다"였다. 둘 다 조율자 소유로 확정했다. "미배정"은 상태이지
면제가 아니었고, 이제 상태도 아니다.

- `moveit-error` — 워크스페이스 전체가 의존하는 에러 타입이라 어느 한
  패널에 주기 애매했다. 지금까지 문제가 되지 않은 이유는 아무도 바꿀 일이
  없었기 때문이지, 안전해서가 아니다. 조율자 소유.
- `moveit-srdf` — **이 파일을 쓰다가 발견했다.** 배정된 적이 없고,
  따라서 주장 감사(§175)도 §172 스윕도 이 크레이트에는 한 번도 돌지
  않았다. 다른 크레이트들이 라운드마다 결함을 내놓고 있는데 이쪽만
  깨끗할 이유가 없다 — **깨끗한 것이 아니라 안 본 것이다.**
  조율자가 직접 처리했다: `doc/claim-audit/moveit-srdf.md`에 상류 주장
  12건 검증(12 CONFIRMED)과 §172 양방향 스윕(0/0, 상류에 float→int 좁힘이
  존재하지 않는다는 근거 포함)을 기록했다. 그 파일의 "Not exhaustive"
  항목이 남은 구멍을 명시한다 — 행동을 주장하지 않는 구조적 인용 약 48건은
  아직 안 봤고, 통과가 아니라 미감사다.

## 교차 소유 편집

원칙은 소유자만 자기 크레이트를 고친다. 예외는 명시적 1회 허가로만
생긴다(예: §171 수정을 위해 `p3-acm`에게 `cost_sources_parity.rs`
직접 수정을 허가한 건). 허가는 그 커밋에 한정되고 다음 라운드로
넘어가지 않는다.

`PORTING-PLAN.md`는 조율자 소유다. 새 절 번호가 필요하면 append 하지
말고 다음 번호를 물어라 — `p1-robotmodel` 라운드 25가 §174를 붙였을 때
main에는 이미 다른 §174와 §175가 있어서 병합 충돌이 났다(그 절은
§176으로 재번호했다).

## 만료조건 (§153.1)

이 표는 스냅샷이다. 패널이 추가·제거되거나 크레이트가 생기면 그 라운드에
갱신한다. 갱신되지 않은 채 남으면 위 1번 사고가 그대로 재발한다 —
소유자는 자기 것이 아니라고 믿고, 조율자는 배정했다고 믿는다.
