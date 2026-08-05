# 포트 커버리지 — 상류 코퍼스의 포팅/미포팅 분할과 미포팅 99건의 분류

`PORTING-PLAN.md` §216이 이 파일을 가리킨다. 여기 있는 모든 수는
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
ported   146
unported 99
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
  만료 조건이 이미 충족된 배제(예: `collision_detector_allocator.hpp`)도
  여기 들어간다.
- **`ported-elsewhere`** — 내용이 다른 이름으로 트리 안에 있는 경우.
  증거 칸에 `.rs` 파일과 심볼을 적는다. 잔여분이 있으면 비고에 남긴다 —
  잔여분이 결정되지 않았고 파일의 대부분이 트리에 없으면 `gap`이다.

부재 주장은 전부 `crates/ ros/ tools/ doc/ PORTING-PLAN.md` 코퍼스에
대한 `rg` 결과이고, 비고 칸에 그 명령을 적었다.

