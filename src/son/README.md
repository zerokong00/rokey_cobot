# src/son — 배관 점검 로봇 (협동3)

기능별로 나눠 둔다. **디렉터리 하나 = 관심사 하나.**

```
son/
├─ spec/          치수·한계값 단일 출처 (parts_meta.json)
├─ tools/         자산 생성 도구
├─ robot/         로봇 본체 — 형상 + 물리(articulation)
├─ camera/        카메라 센서 — 부착·발행·진단
├─ pipe/          배관 환경 — 형상 + 주행 시험
├─ condition/     관 상태 판정 + 시각 오도메트리 (인지, ROS 노드)
├─ localization/  추측 항법 — 도면 없이 지나온 길 기록 (ROS 노드)
├─ driver/        주행 제어 — FSM + 속도 법칙 (ROS 노드)
├─ welder/        용접 모듈 — 토치 + 시퀀스 + 3겹 검증
├─ test_code/     오프라인 검증 (강제 입력·강제 자세)
├─ training/      YOLO 결함 검출 학습
└─ HANDOFF.md     Isaac Sim 담당자 인수인계
```

## 각 디렉터리

| 디렉터리 | 내용 | 실행 환경 |
|---|---|---|
| `spec/` | `parts_meta.json` — 암 기준각·스트로크 한계·세그먼트 간격. 스크립트가 하드코딩 대신 이 파일을 읽는다 | — |
| `tools/` | `build_parts.py` — robot·camera·pipe STL 전부 생성 | python3 |
| `robot/` | `meshes/` + `articulate.py`(링크14/조인트13) + `state_bridge.py`(휠·관절·**IMU** 발행) | **Isaac Sim** |
| `camera/` | `meshes/camera_housing.stl` + `config/camera.yaml` + `rig.py` (부착·발행·`--save`) + `depth_probe.py` (깊이 반환 진단) | **Isaac Sim** |
| `pipe/` | `meshes/` (직관·SR곡관, **균열 있는 판 2종**, 결함·비드 각 2종) + `curve_demo.py` | **Isaac Sim** |
| `condition/` | `detector.py` (판정) + `odometry.py` (시각 오도메트리) + `node.py` + `depth_probe_ros.py` + `config/` | rclpy |
| `localization/` | `deadreckon.py`(거리 융합·곡관 판정·경로) + `node.py` + `config/` | rclpy |
| `driver/` | `control.py` (FSM·속도 법칙, 순수) + `node.py` + `config/` | rclpy |
| `welder/` | `weld.py`(시퀀스·3겹 검증) + `audit.py`(복귀 감사) + `articulate.py`(17링크, **Isaac Sim**) + `meshes/torch_*` | python3 / Isaac |
| `test_code/` | `robot/`, `condition/`, `driver/`, `welder/`, `localization/` — 합성 입력·강제 자세로 로직만 검증 | python3 |

## 의존 방향

```
spec/parts_meta.json
      ▲ ▲ ▲
      │ │ └──────── pipe/curve_demo.py
      │ └────────── robot/articulate.py
      └──────────── test_code/robot/preview_assembly.py

tools/build_parts.py ──> robot/meshes/  camera/meshes/  pipe/meshes/  spec/
camera/rig.py ──────────> robot/robot_2seg_cam.usd
condition/detector.py <── condition/node.py, test_code/condition/*
condition/node.py ──/condition, /visual_speed──> driver/node.py ──> /cmd_vel
```

`spec/parts_meta.json` 이 단일 출처다. 치수를 바꾸려면 `tools/build_parts.py` 의
파라미터를 고치고 다시 돌린다. 나머지는 전부 따라온다.

## 실행 순서

```bash
# 자산
python3 tools/build_parts.py

# 오프라인 검증 (Isaac Sim 불필요)
python3 test_code/robot/preview_assembly.py
python3 test_code/condition/make_scenes.py && python3 test_code/condition/test_detector.py

# Isaac Sim  (USD 파일 없음 — 주행 스크립트가 실행 중에 씬으로 직접 조립한다)
PYTHONUNBUFFERED=1 isaac_python camera/depth_probe.py            # ① 진단 먼저, GUI 필수
PYTHONUNBUFFERED=1 isaac_python robot/articulate.py --headless   # 조립 자체검증(선택)
PYTHONUNBUFFERED=1 isaac_python welder/articulate.py --headless  # 수리기 17링크(선택)
PYTHONUNBUFFERED=1 isaac_python pipe/curve_demo.py --headless --steps 4000
PYTHONUNBUFFERED=1 isaac_python pipe/curve_demo.py --cameras     # 영상까지, GUI 필수

# 로봇 상태 발행 (PC1, Isaac Sim)
PYTHONUNBUFFERED=1 isaac_python robot/state_bridge.py

# 인지·항법·주행 노드 (PC2)
python3 condition/node.py --ros-args --params-file condition/config/pipe_condition.yaml
python3 localization/node.py --ros-args --params-file localization/config/localization.yaml
python3 driver/node.py --ros-args --params-file driver/config/driver.yaml
```

**①을 건너뛰면 안 된다.** 빈 공간 깊이 반환 방식을 모르면 단절 판정이 조용히
작동하지 않는다. 자세한 내용은 `HANDOFF.md`.

## 파일 배너

| 배너 | 인터프리터 | 뜻 |
|---|---|---|
| `[Isaac 3.11]` | Isaac Sim 내장 python | `pxr`/`omni`/`isaacsim` 을 쓴다 |
| `[ROS 3.10]` | 시스템 python3 | `rclpy`/`pipe_msgs` 를 쓴다 |
| `[공용]` | 둘 다 | numpy·cv2 만. ROS·Isaac 비의존 |
| `[오프라인]` | 3.10 | 강제 입력·강제 자세. `test_code/` 전용 |
| `[자산생성]` | 3.10 | STL 등 빌드 도구 |

배너는 장식이 아니다. 해당 파일 맨 위의 `pyver.require_isaac()` /
`require_ros()` 가 **틀린 인터프리터로 띄우면 즉시 막는다.** 지금 환경이 어느
쪽인지는 `python3 pyver.py` 로 본다.

## 파이썬이 왜 두 개인가

**Isaac Sim 5.1 은 Python 3.11 전용이고 ROS 2 Humble 은 3.10 이다.** 확장 모듈
ABI 가 달라 서로의 라이브러리를 못 읽는다. **Isaac Sim 을 띄우는 터미널에서
`source /opt/ros/humble/setup.bash` 를 하면 안 된다.**

그래도 통신은 된다 — **DDS 가 파이썬 버전과 무관하게 데이터를 나르기 때문**이다.
3.11 쪽이 발행한 `/scout/front/image_raw` 를 3.10 쪽 노드가 그대로 받는다.

단, `camera/rig.py` 와 `robot/state_bridge.py` 는 **rclpy 로 직접 발행**하므로
Isaac Sim 쪽에 3.11 용 ROS 2 빌드가 한 번 필요하다 (`HANDOFF.md` 참조).
**커스텀 메시지 `pipe_msgs` 는 3.10 쪽 노드끼리만 쓴다** — Isaac 쪽이 발행하는
것은 전부 표준 메시지다.

## 검증 상태

| 항목 | 상태 |
|---|---|
| 형상·간섭·곡관 통과 기하 | ✅ 오프라인 검증 |
| 관 상태 판정 로직 (합성 11종) | ✅ 전수 통과, 오프셋 오차 0.14mm |
| 주행 FSM·속도 법칙 (19항목) | ✅ 전수 통과 |
| 시각 오도메트리 | ✅ 정확도 ±0.25mm |
| ROS 왕복 (판정 → 주행 → cmd_vel) | ✅ 확인 |
| 추측 항법 (5항목) | ✅ 전수 통과 · 슬립 구간 오차 21.1%→2.1% |
| 용접 3겹 검증 + 복귀 감사 + 링 간섭 (34항목) | ✅ 전수 통과 |
| 결함·비드 기하 (파임 2.2mm / 돌출 0.8mm) | ✅ 실측 일치 |
| 토치 링 배치 (수납 40 / 도달 48mm) | ✅ 곡관·관벽 여유 확인 |
| Isaac Sim 전부 (물리·카메라·주행) | ❌ **미실행** — 이 노트북 사양 미달 |

자세한 근거는 `test_code/README.md`, 인수인계는 `HANDOFF.md`.

## 🚨 조립은 `robot/assemble.py` 한 벌뿐이다 (2026-08-04 변경)

예전에는 `articulate.py` 가 `robot_2seg.usd` 를, `rig.py --save` 가
`robot_2seg_cam.usd` 를 구워 두고 주행 쪽이 그것을 불러 썼다. **그 파일 왕복이
사고의 근원이라 없앴다.**

- `.usd` 는 `.gitignore` 대상이라 레포에 없다 → 읽는 쪽은 그 파일이 지금 소스에서
  나온 것인지 알 수 없었다. **세대가 다른 STL 로 만든 로봇을 조용히 읽는 사고**가
  실제로 났다
- 형상·상수를 고쳐도 낡은 USD 를 계속 읽어 "수정이 반영 안 된다" 오진이 났고,
  그 우회로 "다른 디렉터리에서 만들어 덮어쓰기" 가 생겼다
- Isaac 기동이 2회(조립+주행) 필요했다 → **1회**

지금은 `assemble.build(stage, root, origin)` 가 씬에 직접 짓는다. 시작 위치는
**각 링크 변환에 반영**한다 — articulation root 프림에 변환 op 을 걸면 PhysX 가
어긋난다. `articulate.py` 는 자유공간 자체검증만 남았고 산출물이 없다.

### 🚨 휠 충돌체는 실린더 프리미티브여야 한다

**convexHull 로 바꾸면 관내 주행이 0mm 가 된다.** 메시 hull 은 트레드 둘레가
다각형이라 날카로운 에지가 관벽을 물고, 각속도는 나오는데 관절 위치가 안 쌓이는
스틱슬립 채터가 된다. 시각 메시는 크라운 R50 을 그대로 쓰고 충돌체만 원통이다
(설계 v3 §13.4 "휠은 실린더 프리미티브").

### 진단 노브 (기본값 = 설계값, env 로만 변경)

`WHEEL_TORQUE_FRACTION` `ARM_DR_MM` `STAGGER_MM` (assemble.py) /
`PHYSICS_HZ` `GRAVITY_OFF` (curve_demo.py). 설계값에서 벗어나면 `[진단]` 을 찍는다.
⚠ `test_traction.py` 가 소스를 정규식으로 읽으므로 **설계값은 리터럴로 남길 것.**
