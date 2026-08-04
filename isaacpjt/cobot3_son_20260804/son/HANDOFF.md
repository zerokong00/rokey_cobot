# Isaac Sim 담당자 인수인계 (2026-08-04)

**STL만으로는 아무것도 동작하지 않는다.** STL은 형상일 뿐이고 관절·물리·카메라·
IMU·ROS 발행은 전부 스크립트가 실행 중에 만든다.

**압축을 풀고 아래 순서대로 실행하면 된다. ①을 건너뛰지 말 것.**

---

## 5분 요약

| | |
|---|---|
| 로봇 | 2단 1관절, 6륜, 전장 169mm, DN100 관 내부 |
| 수리기 | 정찰기 + **본체를 감싸는 회전 링에 용접 토치** (17링크) |
| 카메라 | 어안 140°, 앞뒤 2대. **RealSense 아님** (최소 측정거리 52cm로 적용 불가) |
| 로직 | 관 상태 판정 / 추측 항법 / 주행 제어 / 용접 — 전부 오프라인 검증 끝 |
| **당신이 할 일** | Isaac Sim 스크립트 6개를 **처음** 돌려보는 것 |

내 노트북이 Isaac Sim 최소 사양 미달(RTX 3050 Ti 4GB)이라 **시뮬 쪽은 한 번도
실행되지 않았다.** 나머지는 전부 실제로 돌려서 확인했다.

---

## ⚠ 먼저 — 파이썬이 두 개다

**Isaac Sim 5.1 은 Python 3.11, 우리 ROS 2 Humble 은 3.10 이다.**
**Isaac Sim 을 띄우는 터미널에서 `source /opt/ros/humble/setup.bash` 를
하지 말 것.** 3.10 라이브러리가 앞에 잡혀 심볼이 충돌한다.

| 어디서 도는가 | 파이썬 | 파일 |
|---|---|---|
| Isaac Sim 안 | **3.11** | `camera/` `robot/` `pipe/curve_demo.py` `welder/articulate.py` |
| ROS 2 노드 | **3.10** | `condition/node.py` `driver/node.py` `localization/node.py` |
| 둘 다 | 아무거나 | `*/detector.py` `control.py` `deadreckon.py` `weld.py` `audit.py` |

**통신은 문제없다** — DDS 가 파이썬 버전과 무관하게 데이터를 나른다. 3.11 쪽이
발행한 영상을 3.10 쪽 노드가 그대로 받는다.

### 딱 하나 준비가 필요하다

`camera/rig.py` 와 `robot/state_bridge.py` 가 **rclpy 로 직접 발행**한다.
Isaac Sim 의 3.11 에는 rclpy 가 없으므로 한 번만 빌드해 둔다:

```bash
git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces
cd IsaacSim-ros_workspaces && ./build_ros.sh -d humble -v 22.04
source build_ws/humble/humble_ws/install/local_setup.bash
source build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash
~/isaacsim/isaac-sim.sh        # 같은 터미널에서
```

**커스텀 메시지 `pipe_msgs` 는 이 빌드에 안 넣어도 된다.** Isaac 쪽이 발행하는
것은 전부 표준 메시지(`sensor_msgs`, `std_msgs`)고, `pipe_msgs` 는 3.10 쪽
노드끼리만 주고받는다.

파일을 틀린 쪽에서 띄우면 스크립트가 **바로 막고 어느 쪽인지 알려준다.**
지금 터미널이 어느 쪽인지는 `python3 pyver.py` 로 확인한다.

---

## 실행 순서

```bash
# 0. 형상 생성 (이미 들어 있다. 치수를 바꿀 때만)
python3 tools/build_parts.py

# ① 깊이 반환 방식 진단  ← 가장 먼저. GUI 필수
PYTHONUNBUFFERED=1 isaac_python camera/depth_probe.py

# ② 정찰기 물리 조립 (링크 14 / DOF 13)
PYTHONUNBUFFERED=1 isaac_python robot/articulate.py --headless
#    → robot/robot_2seg.usd

# ③ 수리기 물리 조립 (링크 17 / DOF 15, 토치 링 포함)
PYTHONUNBUFFERED=1 isaac_python welder/articulate.py --headless
#    → welder/welder_2seg.usd

# ④ 카메라 부착 후 저장  ← GUI 필수
PYTHONUNBUFFERED=1 isaac_python camera/rig.py --save
#    → robot/robot_2seg_cam.usd   (최종 자산)

# ⑤ 로봇 상태 발행 확인 (휠·관절·IMU)
PYTHONUNBUFFERED=1 isaac_python robot/state_bridge.py

# ⑥ 곡관 주행 + 영상 발행  ← GUI 필수
PYTHONUNBUFFERED=1 isaac_python pipe/curve_demo.py --cameras
```

### ①을 건너뛰면 안 되는 이유

빈 공간의 깊이를 시뮬레이터가 `0` / `inf` / `NaN` / 최대거리 중 무엇으로
돌려주는지에 따라 판정식이 달라진다. **틀리면 관 단절 판정이 조용히 작동하지
않는다** — 에러도 안 나고 그냥 아무것도 감지하지 못한다.

결과가 `camera/camera_probe_result.json`으로 떨어진다. 그 `invalid_mode` 값을
`condition/config/pipe_condition.yaml`에 넣어야 한다.
같은 스크립트가 **근접 벽면(25mm) 측정 가능 여부**도 같이 본다.

---

## GUI가 반드시 필요한 것

**headless에서는 카메라 프레임이 발행되지 않는다.** `ros2 topic list`에 토픽이
보여도 프레임 수는 0이다(팀 실측, 40초 확인).

| 스크립트 | GUI |
|---|---|
| `camera/depth_probe.py` | **필수** |
| `camera/rig.py --save` | **필수** — 프레임 0이면 저장을 거부하도록 해뒀다 |
| `pipe/curve_demo.py --cameras` | **필수** |
| `robot/articulate.py`, `welder/articulate.py` | 불필요 |
| `robot/state_bridge.py` | 불필요 (카메라를 안 쓴다) |

---

## 디렉터리

```
son/
├─ spec/parts_meta.json     치수·한계값 단일 출처. 전 스크립트가 읽는다
├─ tools/build_parts.py     STL 전부 생성
├─ robot/                   meshes + articulate.py + state_bridge.py
├─ camera/                  meshes + config + rig.py + depth_probe.py
├─ pipe/                    meshes(직관·SR곡관·**균열판**·결함·비드) + curve_demo.py
├─ welder/                  meshes(링·로드·팁) + weld.py + audit.py + articulate.py
├─ condition/               관 상태 판정 + 시각 오도메트리   (PC2)
├─ localization/            추측 항법                        (PC2)
├─ driver/                  주행 제어                        (PC2)
└─ test_code/               오프라인 검증 + 그림
```

**`spec/parts_meta.json`을 빼면 안 된다.** 암 기준각·스트로크 한계·링 위치가
전부 여기 있고 스크립트가 하드코딩 대신 이 파일을 읽는다.

---

## 배관 메시 6종

| 파일 | 용도 |
|---|---|
| `pipe_straight.stl` | 정상 직관 600mm |
| `pipe_elbow_sr.stl` | 정상 SR곡관 90° (R=100mm) |
| **`pipe_straight_crack.stl`** | **균열 있는 직관** — 중앙(x=0)에 깊이 2.2mm |
| **`pipe_elbow_crack.stl`** | **균열 있는 곡관** — 호 중간(78.5mm)에 깊이 2.2mm |
| `defect_crack/hole` + `bead_crack/hole` | 개구부에 끼우는 **패치 쌍**. 가시성 전환으로 수리 표현 |

앞의 넷은 **관 자체에 균열이 박힌 통짜 메시**다 — 그냥 얹으면 끝. 수리 표현이
필요 없는 용도(YOLO 학습 영상, 판정기 시험, 눈 확인)에 쓴다.

패치 쌍은 용접 시뮬용이다. 결함 프림을 숨기고 비드 프림을 보이면 형상이 실제로
바뀐다(색상 덮어쓰기가 아니라 Depth 로도 확인되는 방식).

균열 위치는 생성 함수의 `site_x` / `site_deg` / `site_frac` 인자로 바꾼다.

## 그림부터 보면 빠르다

```
test_code/welder/overview.png        전체 현황 한 장
test_code/welder/welder_layout.png   수리기 배치 6패널
test_code/welder/welder_camera.png   카메라 시야 + 결함/비드 단면
test_code/welder/welder_ring.gif     토치 링 회전
test_code/robot/curve_pass.gif       곡관 통과
test_code/robot/preview_*.png        정찰기 직관/곡관 자세
test_code/condition/scenes/*.png     판정 시험 장면 (RGB + Depth)
```

---

## 오프라인 검증 — 그쪽에서도 바로 돌아간다

`python3`만 있으면 된다 (numpy / scipy / opencv / trimesh / matplotlib).

```bash
python3 test_code/robot/preview_assembly.py          # 형상·곡관 간섭
python3 test_code/condition/make_scenes.py           # 시험 장면 생성 (.npy 는 압축에서 뺐다)
python3 test_code/condition/test_detector.py         # 관 상태 판정 11장면
python3 test_code/driver/test_control.py             # 주행 FSM 19항목
python3 test_code/driver/test_odometry.py            # 시각 오도메트리
python3 test_code/localization/test_deadreckon.py    # 추측 항법 5항목
python3 test_code/welder/test_weld.py                # 용접 3겹 10항목
python3 test_code/welder/test_audit.py               # 복귀 감사 12항목
python3 test_code/welder/test_ring_clearance.py      # 링 간섭 12항목
```

`test_detector.py`는 `make_scenes.py`를 먼저 돌려야 한다(장면 `.npy`를 용량 때문에
뺐다).

| 항목 | 결과 |
|---|---|
| 형상·곡관 통과 기하 | 관절 44.9°(한계 55), 스트로크 ±4.02mm(한계 6) |
| 관 상태 판정 | 11장면 전수, 오프셋 오차 0.14mm |
| 시각 오도메트리 | ±0.25mm |
| 추측 항법 | 슬립 구간 거리오차 21.1% → 2.1% |
| 주행 FSM | 19/19 |
| 용접 3겹 + 복귀 감사 + 링 간섭 | 34/34 |
| ROS 왕복 | 판정→주행→`cmd_vel`, 항법 경로 재구성 확인 |

---

## 토픽

```
Isaac Sim (PC1)
  robot/state_bridge.py  → /wheel_speed /wheel_speeds /joint_angle
                           /imu_roll /imu_yaw_rate /suspension
  camera/rig.py          → /front/rgb /front/depth /front/camera_info
                           /rear/rgb  /rear/depth  /rear/camera_info
                              ↓ DDS
PC2
  condition/node.py      ← depth, rgb, joint_angle
                         → /condition  /visual_speed
  localization/node.py   ← wheel_speed, visual_speed, joint_angle, imu_*
                         → /pose /path /defect_marks
  driver/node.py         ← condition, wheel_speed, visual_speed
                         → /cmd_vel (linear.x 만)
```

⚠️ **아직 안 이어진 것** — `driver`가 내는 `/cmd_vel`을 Isaac Sim 쪽에서 받아
바퀴를 돌리는 부분이 없다. `pipe/curve_demo.py`는 지금 자체적으로 속도를 준다.
이걸 잇는 게 다음 할 일이다.

---

## 알려진 함정

| 항목 | 값 / 조치 | 안 지키면 |
|---|---|---|
| 이미지 QoS | `BEST_EFFORT` | 토픽이 아예 수신되지 않는다 |
| `contactOffset` | `0.0005` | 휠 반경 10mm 스케일에서 로봇이 뜬 것처럼 거동 |
| 배관 콜라이더 | `approximation="none"` | 관 안쪽이 꽉 찬 덩어리가 되어 못 들어간다 |
| 조명 intensity | `3e7` | 3e5면 밝기 중앙값 3/255 |
| near clip | `0.005` | 관벽 25mm가 잘려 화면이 검정 |
| `EnabledSelfCollisions` | `False` | 암이 본체와 겹쳐 발산 |
| `set_joint_positions` | 호출 후 drive target 재설정 | target이 덮어써진다 |
| 각도 드라이브 단위 | N·m/rad에 π/180 | USD는 도(degree) 기준으로 해석 |

**`fisheyePolynomial`은 쓰지 말 것.** Isaac Sim 5.0부터 폐기됐고 설정해도
무시된다. `set_opencv_fisheye_properties()`를 쓴다(반영돼 있음).
**`set_horizontal_fov`라는 세터는 없다** — 초점거리 + 센서폭으로 화각이 정해진다.

---

## 처음 돌릴 때 알려줬으면 하는 것 4가지

1. **IMU 프레임 키** — `state_bridge.py`가 첫 프레임에 `IMU 프레임 키: [...]`를
   찍는다. 문서에 반환 키가 안 나와 있어 `lin_acc`/`linear_acceleration` 양쪽을
   받게 해뒀다. 실제 키를 알려 달라
2. **롤 부호** — 로봇을 굴렸을 때 `/imu_roll`이 어느 방향으로 늘어나는지.
   IMU 좌표계가 프림 로컬인지 월드인지에 따라 뒤집힐 수 있다.
   **이게 틀리면 토치 링을 반대로 돌린다**
3. **`camera_probe_result.json`의 `invalid_mode`** — 이 값이 있어야 단절 판정이 산다
4. **`set_opencv_fisheye_properties`가 먹는지** — 실패하면 경고를 찍고 핀홀로
   떨어진다. 그 경우 `condition/config`의 `projection`도 바꿔야 한다

---

## 문제가 생기면 보는 순서

1. 화면이 검정 → near clip(0.005), 조명(3e7), GUI 여부
2. 토픽은 있는데 프레임 0 → headless로 돌렸는지
3. 토픽 자체가 안 옴 → 구독자 QoS, `ROS_LOCALHOST_ONLY=0`
4. 로봇이 공중에 뜸 → `contactOffset`
5. 관에 안 들어감 → 배관 콜라이더 `approximation`
6. 조인트 발산 → `EnabledSelfCollisions`, 질량, 게인
7. 단절 감지 안 됨 → ① 진단을 안 돌렸거나 `invalid_mode`가 틀림
8. 링크/DOF 개수 불일치 → 각 `articulate.py`가 기대값과 대조해 찍는다

---

## 설계 문서와 다른 점 (이미 `.docx`에 반영함)

| 항목 | 설계 원안 | 현재 |
|---|---|---|
| 센서 | RealSense 전/후 | **광각 보어스코프형 어안** (D455 최소거리 52cm로 적용 불가) |
| 토치 | 축 근처 2자유도, J2 35mm | **본체 감싸는 회전 링**, J2 8mm |
| 위치 추정 | 도면 기반 위상 매칭(5.3) | **로봇은 도면 없음**. 상대 경로만 내고 도면 매칭은 관리자 쪽 |
| 슬립 감지 | 휠 6개 편차 | + **시각 오도메트리** (6륜 동시 슬립은 편차로 못 잡음) |
| 용접 검증 | 검출기 미검출 | **3겹** (정렬 + Depth 프로파일 + 검출기) |

마지막 3개는 `.docx`에 아직 안 넣었다. 필요하면 말해 달라.
