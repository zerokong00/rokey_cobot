# pipe_robot_v6 + restroom_pipeR150 테스트 기록 (2026-08-07, 잠시 중단 — 다음에 이어서)

`real_map_demo.py`(로봇: `robot_bellows.usda`)와는 별개로, 새 로봇 후보
`pipe_robot_v6.usda`를 새 맵(`restroom_pipeR150.stp`)에서 GUI로 라이브
테스트하는 중. **안착(휠-벽 밀착) 자체는 확인됐다 — 이제 실제 주행(`go()`)
테스트가 다음 단계.** 사용자 요청으로 여기서 일시 중단, 아래는 지금까지
진행 상황 기록.

---

## 배경 — 왜 이 로봇/맵인가

- `real_map_demo.py`가 쓰는 로봇은 `robot_bellows.usda`(12휠, 2세그먼트, 벨로우즈 4관절, 로컬 -Z가 전방).
- 처음엔 `robot_from_bot_welder_art_v2.usda`(더 복잡한 후보: 12휠+12피스톤+D6 벨로우즈+통합 토치)를 검토했으나, `real_map_demo.py`의 거의 모든 로봇 종속 코드(휠/피스톤/벨로우즈 이름 매칭, 카메라 마운트, 토치 조준)를 새로 짜야 해서 보류.
- 사용자가 대신 **`pipe_robot_v6.usda`**(순수 주행/조향 전용, 토치 없음, 12휠·3세그먼트: `Body`+`DiscF`+`DiscR`, 양 끝 Roll+Bend 조향)로 방향 전환. 같은 폴더에 이미 전용 컨트롤러 `robot_final/final_script.py`가 있음(GUI Script Editor에 붙여넣어 라이브로 돌리는 방식, `real_map_demo.py`처럼 독립 실행 스크립트가 아님).
- 맵은 `restroom_pipeR150.stp`(원본 STEP CAD, 미변환) 사용하기로 함 — `real_map_demo.py`가 쓰는 `restroom_pipe150_final_fixed.usd`와 같은 계열(floor1/floor2/aisle 포함)이지만 확증은 안 됨.

## 지금까지 한 일

1. **STEP → USD 변환**: 기존 도구 `tools/step_to_usd.py`로 `maps/restroom_pipeR150.stp` → `maps/restroom_pipeR150.usd` 변환 성공(22메시, 36,803삼각형, 783KB).
2. **`robot_final/gui_smoke_test.py` 작성** — 맵+로봇을 올리고 GUI를 띄우는 셋업 스크립트(미션 로직 없음). 핵심 설계:
   - `real_map_demo.py`의 `CenterLine` 클래스와 `FLOORS["floor2"]` 코스 좌표를 그대로 재사용(같은 CAD 계열이라 좌표 공유 가정).
   - 배치는 `real_map_demo.py`와 **동일한 방식**: 부모 Xform(articulation root)에 변환을 걸지 않고, `PATH.frame(s)`로 구한 위치+접선을 각 **RigidBody 자식의 로컬 변환에 개별적으로 굽는다**(`local * PLACE`).
   - `pipe_robot_v6`은 `final_script.py`가 로컬 **+X축을 진행방향**으로 읽는다(로컬 -Z가 전방인 robot_bellows와 다른 축 규약) — PLACE 행렬 row0에 코스 접선을 넣어 반영.
3. **크래시 해결**: 처음엔 raw `omni.usd` 스테이지 + GUI Play 버튼 조합으로 만들었는데, Play를 누르면(또는 몇 초 뒤 자동으로) RigidBody 29개 **전부**에 `Invalid PhysX transform` 경고가 뜨고 프로세스가 죽었다. `real_map_demo.py`를 다시 보니 raw 스테이지가 아니라 **`isaacsim.core.api.World`로 만든 스테이지에, 배치를 다 구운 뒤 `world.reset()`을 호출**하는 순서였다 — 이 패턴으로 바꾸니 크래시 사라짐(`Invalid PhysX transform` 재발 없음, 137초+ 안정 실행 확인).
4. **위치/방향 조정**(사용자 실시간 GUI 피드백 반영):
   - "z축으로 좀 더 내려가면 될 것 같은데" → `START_S_MM` 35→70→120mm로 증가(코스가 이 구간에서 순수 -Z 하강이라 s 증가 = z 하강).
   - "로봇을 z축 방향으로 뒤집고" → 진행방향(`_fwd`)과 위(`_yc`) 축을 반전(`_fwd = -_t_start`, `_yc = -_u_start`), 오른손 좌표계는 유지.
5. **안착(휠-벽 밀착) 진단 추가** — `real_map_demo.py::radial_r()`와 같은 방식으로 휠 12개의 중심선 기준 반경(mm)을 찍어서 파이프 벽에 고르게 걸쳐있는지 확인:
   - `Body`(6휠)·`DiscF`(3휠)는 꾸준히 37~41mm로 고름(정상으로 보임, 설계 내반경 50mm에서 휠 반경만큼 뺀 범위).
   - `DiscR`(뒤쪽 3휠)만 비정상: `23.6 / 50.9 / 50.0`mm — 그것도 체크포인트마다 오락가락(진동처럼 보였음).
6. **가설 1 — "뒤쪽 조향이 능동 제어 없이 안 잡혀서 그렇다"**: `final_script.py`의 `REAR_FREE=True` 설계상 `reset()`(`_configure_rear()`가 -10°×sign 바이어스를 건다)을 호출하면 잡힐 거라 예상 → **틀렸다.** `reset()` 호출 후 오히려 `31.7 / 60.2 / 31.0`mm로 더 나빠짐(단, 진동은 멈추고 그 상태로 고정됨). `robot_final/check_rear_settle.py`(새 스크립트, `final_script.py`의 `def`/`class`만 로드하고 자동실행부는 잘라내서 `reset()`만 수동 호출)로 확인.
7. **USD 원본 직접 대조**: `pipe_robot_v6.usda`의 `SteerJoints`(RollF/BendF/RollR/RollR)를 직접 읽어보니 F/R이 **완벽히 대칭**(같은 damping/stiffness/maxForce, 대칭 localPos) — 에셋 자체의 설계 결함은 아님. 그래서 원인이 내 배치 코드나 물리 안정성 쪽일 가능성으로 좁혀지던 중,
8. **가설 2(사용자 지적) — "로봇이 너무 길어서 관 굽는 지점에 걸쳐있다"**: `Body`↔`SphF`↔`DiscF` 기하를 계산하면 `DiscF`/`DiscR` 원점이 `Body`에서 각각 약 **±114mm**(총 ~228mm) 떨어져 있는데, 코스의 첫 직선(라이저) 구간은 겨우 **185mm**(굽음 시작 전까지). `Body`를 s=120mm에 놓으면 반대쪽 끝은 이미 굽은 구간(또는 그 너머)에 걸쳐 있는데, 안착 진단은 **로봇 전체에 Body 기준 s 하나만** 써서 반경을 쟀으니 굽은 구간에 걸친 쪽은 애초에 잘못된 중심선 기준점으로 측정된 셈이었다.
9. **확인됨 — 가설 2가 맞았다.** `check_rear_settle.py`에 `real_map_demo.py::CenterLine.project()`(전체 중심선 탐색, 굽은 구간 포함)를 그대로 이식해서 **휠 12개 각각을 자기 위치에서 따로 투영**하도록 고치고 재실행:
   ```
   s(mm)   Body[179 178 175 143 142 139]  DiscF[47 44 46]  DiscR[264 294 268]
   r(mm)   Body[40.8 40.2 40.7 40.7 40.5 40.6]  DiscF[40.8 40.8 40.6]  DiscR[40.6 30.2 40.4]
   ```
   `DiscR`이 s=264~294mm — 굽음 시작(185mm)을 한참 지나 곡관 안쪽 깊숙이 있는 게 실측으로 확인됐다. 휠별로 제대로 투영하니 반경도 거의 정상(40.6/30.2/40.4 — 이전의 `31.7/60.2/31.0`은 순전히 측정 오류였다). 남은 30.2mm 휠 하나는 약간 낮지만 이전처럼 "벽을 뚫고 들어간" 수준(60mm대)은 전혀 아니다 — **안착 자체는 정상으로 판단.**

## 다음에 이어서 할 일 (사용자 요청으로 여기서 중단)

1. **주행 테스트** — `final_script.py`의 `go()`(실제 주행+분기조향)는 아직 한 번도 안 불러봤다. `check_rear_settle.py`는 의도적으로 `reset()`까지만 호출한다(안착만 보려고). 다음엔 `go()`까지 실행해서 실제로 굴러가는지, 곡관을 타고 넘는지 확인 필요.
2. `PIPE`/`JUNCTION` 상수 정리 — 지금은 `JUNCTION = Body 시작 위치`로 임시로 넣어놨다(진짜 분기점 아님, 안착 확인용 placeholder). floor2는 실제 T분기가 없어서(`real_map_demo.py` 기준 "disconnect"로 끝남) 분기조향 자체를 보려면 floor1 좌표(730,850mm 부근)가 필요할 수 있음.
3. 로봇 길이(~228mm+)가 직선 구간(185mm)보다 길다는 게 확인됐으니, `START_S_MM`을 어떻게 잡아도 한쪽 끝은 항상 곡관에 걸린다는 전제로 이후 테스트를 설계할 것(문제가 아니라 이 로봇/코스의 정상적인 조건으로 보임).
4. 한 번 정도 Isaac Sim 프로세스가 스크립트 코드 실행 전(앱 시작 직후) 원인불명으로 죽은 적이 있었다(`check_rear2`) — 재시도(`check_rear3`)하니 바로 정상 작동했다. 재현되면 원인 더 볼 것, 한 번뿐이면 일시적 문제로 무시 가능.

## 만든 파일

- `maps/restroom_pipeR150.usd` — STEP 변환 결과물(신규)
- `robot_final/gui_smoke_test.py` — 맵+로봇 GUI 셋업(신규, 미션 로직 없음)
- `robot_final/check_rear_settle.py` — 안착 진단 전용, `final_script.py`의 함수만 로드해서 `reset()`까지만 호출(신규, `go()`/주행은 아직 안 부름)
- `robot_final/PIPE_ROBOT_V6_TEST_LOG.md` — 이 문서
