# ROS 2 통신 파트 — 남은 일 (2026-08-06)

브랜치 **`ros2-comm`**. 아직 **커밋 안 함.**

이 문서 하나만 보고 이어서 할 수 있게 적었다. 순서는 값어치 순이다.

---

## 0. 5분 요약 — 지금 어디까지 왔나

Isaac Sim(3.11) → ROS 2(3.10) 통신이 **실기로 돈다.** 카메라·주행상태·사건이
전부 도착하는 것을 확인했다. 남은 것은 ① 내가 낸 버그 하나 ② 값의 **의미**
검증 ③ 팀의 진짜 소비자와 붙이기다.

| 토픽 | 상태 | 근거 |
|---|---|---|
| `rgb/compressed` | ✅ | 640×360 JPEG, 평균밝기 133, 브라우저로 눈 확인 |
| `depth/compressed` | ✅ | 유효 100%, min 0.042 / 중앙 0.068 / max 0.31 m |
| `camera_info` | ✅ | fx 261.9 = `(640/2)/tan⁻¹(70°)` 정확 일치 |
| `drive_state` | ✅ | 상태·s·이탈·방향 |
| `event` | ✅ | START / DEFECT / WELD_BEGIN / WELD_DONE |
| `moving` | ✅ | 전환 시에만 발행 |
| `odom` | △ | 값은 오는데 **호길이가 맞는지 미검증** → §2 |
| `imu` | ❌ | **틀린 물리량을 싣고 있다** → §1 |
| `joint_states` | △ | 발행만 하고 아무도 안 본다 → §6 |
| `rear/*` | ❌ | 테스트 브리지가 전방만 보낸다 → §5 |
| `depth` (32FC1 raw) | ❌ | 선택 항목, 안 보냄 → §3 |
| `cmd_vel` | ❌ | 아무도 안 보내고 안 받는다 → §7 |
| `mission` | ✅/△ | `integration_test/real_map_demo.py` 는 **동작을 바꾼다**(2026-08-08). `repair_demo`/`src/son/real_map_demo` 는 아직 로그만 → §4 |
| `repair_target` | ❌ | 아예 안 쓰인다 → §7 |

---

## 1. 🚨 IMU 롤이 틀린 값이다 — 제일 먼저

### 무엇이 틀렸나

`src/son/repair_demo.py:2001`

```python
roll_deg=math.degrees(math.atan2(_p[2], _p[1] - IN_Y)),
```

`_p` 는 `wpos(_seg1)` 즉 seg1 의 **월드 위치**다. 그래서 이 값은

> 로봇이 관 단면에서 **어느 쪽으로 치우쳐 있는가**(중심 이탈 방향)

이지, 규약이 요구하는

> 로봇이 제 축을 중심으로 **얼마나 굴렀는가**(롤)

가 아니다. **완전히 다른 물리량이다.**

### 왜 이게 중요한가

롤이 곧 **결함의 시계각 기준**이고, 그 값으로 **토치 링 J1 을 어디로 돌릴지**
정한다. 틀리면 **엉뚱한 곳을 용접한다. 에러는 안 난다.**
규격서(`pipe_comm/ROS2_통신규격.md` §6.5)가 "이 시스템에서 가장 조용히
망가지는 자리"로 꼽은 바로 그 값이다.

### 증상 (이미 로그에 있었다)

로봇이 SETTLE/INSPECT 로 **멈춰 있는데** 롤이 계속 움직였다:

```
롤 -77.6°  →  -96.4°  →  -101.3°  →  -101.8°
```

진짜 롤이면 정지 중에는 안 변한다. 피스톤이 자리를 잡으며 중심 이탈 **방향**이
도는 것을 롤이라고 실은 것이다.

### 고치는 법

seg1 의 **월드 회전**에서 관 축(X) 성분을 뽑는다. `map_test_demo.py` 용으로
써 두었던 것과 같은 방식이다:

```python
def wroll(prim):
    """세그먼트의 월드 롤(도). 관 축이 X 이므로 X 회전이 시계각 기준이다."""
    _XC.Clear()
    m = _XC.GetLocalToWorldTransform(prim)
    e = m.ExtractRotation().Decompose(Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0),
                                      Gf.Vec3d(0, 0, 1))
    return float(e[0])
```

그리고 `roll_deg=wroll(_seg1)` 로 바꾼다.

⚠ **이것도 임시판이다.** 본판(Isaac 담당자)은 `IMUSensor` 를 붙여
`lin_acc` 에서 유도해야 한다 — 규격서 §6.5 의 부호 함정(`atan2(gy, gz)`,
정립에서 +9.81)은 프림 변환 경로로는 검증되지 않는다.

### 통과 기준

로봇을 관에 정립으로 넣었을 때 **롤이 0° 근처**여야 한다. 그리고 정지 중에는
**안 변해야** 한다. 규격서 §6.5 실측표(0° / +30° / −45° / +90°)가 대조표다.

```bash
ros2 topic echo /robot/imu --field orientation
```

---

## 2. `odom.x` 가 진짜 호길이인가

### 왜 보나

규약(§6.4)은 `pose.pose.position.x` 에 **관 중심선을 따라 잰 진행거리**를
싣기로 했다. 직선거리를 실으면 **곡관에서 값이 줄어든다** — 위치 추정이
뒤로 가는 것처럼 보인다.

`repair_demo.py` 의 `pipe_s()` 가 그 계산인데, 곡관 구간을
`t = atan2(x, -y)` 로 잡는다. 이 식이 실제 기하와 맞는지는 **아직 안 봤다.**

### 확인법

곡관을 지나는 동안 **단조증가**해야 한다. 한 번이라도 줄면 틀린 것이다.

```bash
ros2 topic echo /robot/odom --field pose.pose.position.x
```

로봇이 s ≈ 350mm(직관 끝) → 586mm(곡관 끝) 구간을 지날 때를 보면 된다.
코스 총길이는 **935.6mm**(직관 350 + 호 235.6 + 직관 350).

### 참고 — 코스 기하 (실측)

`src/son/pipe/pipe_elbow_lr150.usda`, `metersPerUnit 1` (이미 미터):

```
입구 직관  (−0.350, −0.150, 0) → (0, −0.150, 0)   +X 진행,  0.350
곡관       중심 (0,0), R=0.150, 90°               호 0.2356
           P(t) = (R·sin t, −R·cos t, 0),  t: 0 → π/2
출구 직관  (0.150, 0, 0) → (0.150, 0.350, 0)      +Y 진행,  0.350
```

`repair_demo.py` 의 상수와 일치한다: `IN_Y=-0.150`, `ARC_R=0.150`,
`OUT_X=0.150`, `PIPE_IR=0.050`.

---

## 3. depth 압축이 얼마나 깎아먹는가

규약은 `depth/compressed`(16UC1 PNG, mm)를 주 경로로 쓰고, 원본 정밀도가
필요하면 `depth`(32FC1)를 따로 받는다. 그런데 **지금 브리지는 raw 를 안
보낸다.** 둘 다 켜서 대조해야 mm 반올림 오차가 실제로 예상 수준(≤0.5mm)인지
확인된다.

### 할 일

`src/dongmin/isaac_bridge/ros_bridge.py` 의 `publish_camera()` 에 32FC1 발행을 추가한다
(`Topics.DEPTH_RAW`, `sensor_msgs/Image`, `encoding="32FC1"`).
`camera_monitor` 는 **이미 `depth_raw` 를 구독하고 있어서** 붙이면 바로
`depth32` 줄이 같이 찍힌다.

### 통과 기준

`camera_monitor` 출력에서 `depth` 와 `depth32` 의 min/중앙/max 차이가
**0.5mm 이내**.

⚠ WAN 을 건널 일이 생기면 raw 는 못 쓴다(1280×720 10Hz ≈ 300 Mbps).
시연은 한 PC 라 상관없다.

---

## 4. `mission` 지령이 실제로 동작을 바꾸는가

**(2026-08-08 — 관제 패널이 쓰는 시연에서는 끝냈다.)**
`src/dongyeon/integration_test/real_map_demo.py` (= `--ros` 로 `course` 를
발행하는, web_panel 3D 맵이 보는 그 시연)이 여섯 지령을 전부 구현한다:

| 지령 | 동작 |
|---|---|
| START | `_hold` 해제 — 주행 재개 |
| STOP | `_hold` — 주행 상태로 돌아오는 순간 `drive(0.0)` 로 선다 |
| RECALL | 그 자리에서 `RETURN` 진입(방향 반전). JUNCTION 이면 예압을 푼다 |
| FORWARD | 복귀 취소 — `CRUISE` 로 돌려세워 다시 전진 점검. **결함 플래그(`inspected`/`rechecked`)를 되돌린다** — 안 그러면 RECHECK 안의 용접봉 잔량 검사를 건너뛰고 바로 용접한다 |
| RETRY | `RECOVER` 진입 — 후진 후 재진입 (웹 버튼은 뺐다. mission_cli 로는 그대로 됨) |
| SPEED | `TARGET_SPEED_MPS` **와 `SPIN_DEG_S` 를 같이** 갈고 5mm/s ~ contactOffset 상한으로 자른다 |
| ESTOP | 어느 상태에서든 즉시 언다(아크도 끈다) |
| (해제) | `START` + `reason=ESTOP_RELEASE` — 웹의 `🔓 비상정지 해제` 버튼. 풀면 HOLD 로 돌아오고 ▶ START 를 한 번 더 눌러야 움직인다 |

🚨 **용접 시퀀스 중에는 ESTOP 말고는 안 걸린다.** 토치가 뻗고 아크가 붙은
상태로 세우면 재개 경로가 없다. 그래서 지령을 거절하지 않고 **미뤘다가**
주행 상태(`_HOLD_STATES`)로 돌아오는 순간 건다. 서 있는 동안 `drive_state` 는
`HOLD`(ESTOP 은 `DEAD`)로 나가고 `reason` 에 `HOLD(CRUISE)` 처럼 원래 FSM
상태를 남긴다 — 안 그러면 화면에는 "주행 중"인데 실제로는 멈춰 있다.

남은 것: `repair_demo.py` 와 `src/son/real_map_demo.py` 는 아직 **로그만 찍고
무시한다** — 시퀀스를 끝까지 돌리는 시연이라 일부러 그렇게 뒀던 것이다
(`📥 지령` 출력). 필요해지면 위 구현을 그대로 옮기면 된다(상태 이름만 다르다).

수신 경로가 살아 있는 것은 확인됐지만 **STOP 이 로봇을 세우는 것은 확인
안 됐다.**

### 할 일

`repair_demo.py` 의 지령 처리부에서 최소한 `ESTOP`/`STOP` 은 실제로
`drive(0.0)` 을 걸게 한다. `map_test_demo.py` 에 그 처리를 써 뒀던 것이
참고가 된다(START/STOP/RECALL/RETRY/ESTOP).

### 통과 기준

```bash
ros2 run pipe_comm mission_cli -- STOP
```
→ `drive_monitor` 에 `■ 정지` 가 뜨고 `s` 가 멈춘다.

---

## 5. 후방 카메라

규약에 `rear/rgb/compressed`, `rear/camera_info` 가 있는데 테스트 브리지가
**전방만** 보낸다(`ros_bridge.py` 머리말에 그렇게 적어 뒀다).

`repair_demo.py` 는 `rigs` 에 `back_camera` 도 들고 있으므로
`use_annotators()` 를 한 번 더 부르고 발행 대상을 늘리면 된다.
복귀 주행·시각 오도메트리에 쓸 값이다.

---

## 6. `joint_states` 를 보는 쪽이 없다

발행은 하는데(휠 12개 위치·속도) 아무도 안 본다. 원래 용도는 **휠 개별 속도의
편차로 슬립을 보는 것**이다(설계 5.2). 6륜 동시 슬립은 편차로 못 잡아서
시각 오도메트리를 같이 쓴다는 게 설계 변경점이었다.

`drive_monitor` 에 휠 속도 편차 한 줄을 추가하면 바로 보인다.

---

## 7. 아직 아무도 안 쓰는 것

| 토픽 | 사정 |
|---|---|
| `cmd_vel` | **HANDOFF.md 가 "아직 안 이어진 것" 으로 남긴 부분.** 주행 제어를 ROS 쪽(`son/driver/node.py`)으로 넘길 때 붙인다. 지금 시연은 자체적으로 속도를 준다 |
| `repair_target` | 결함 위치(호길이 + 시계각)를 ROS 쪽에서 정해 Isaac 으로 보내는 경로. `dongyeon/pipe_coordinator_node` 가 낼 값이다 |

---

## 8. 🎯 최종 관문 — 팀의 진짜 소비자와 붙이기

지금까지 확인은 **전부 내가 만든 모니터로만** 했다. 팀의 실제 검출 노드와는
**한 번도 안 붙여봤다.**

`src/dongyeon/pipe_inspect_demo/pipe_inspect_demo/pipe_vision_node.py` 가
받는 것:

```
/rgb/compressed      CompressedImage  JPEG
/depth/compressed    CompressedImage  16UC1 PNG (mm)
/camera_info         CameraInfo
/odom                nav_msgs/Odometry
/imu                 sensor_msgs/Imu
```

우리 규약과 **타입·형식이 같다**(그쪽 `ros_image_utils.compressed_to_depth_m`
가 16UC1 PNG 만 받는 것에 맞춰 규약을 정했다). 다만 **네임스페이스가 없다** —
그쪽은 `/rgb/compressed`, 우리는 `/robot/rgb/compressed` 다.

### 붙이는 법

리매핑으로 이어 본다:

```bash
ros2 run pipe_inspect_demo pipe_vision \
  --ros-args \
  -r /rgb/compressed:=/robot/rgb/compressed \
  -r /depth/compressed:=/robot/depth/compressed \
  -r /camera_info:=/robot/camera_info \
  -r /odom:=/robot/odom \
  -r /imu:=/robot/imu
```

또는 그쪽 노드의 파라미터로 토픽 이름을 바꾼다(`rgb_topic` 등 전부
파라미터로 열려 있다).

### 통과 기준

`/defect/report_json` 에 결함 JSON 이 나오면 끝까지 이어진 것이다.

⚠ YOLO 모델(`yolov8n_seg_best.pt`)이 필요하고 `.gitignore` 의 `*.pt` 때문에
git 으로 안 따라간다. 없으면 그쪽 노드가 `FileNotFoundError` 로 죽는다.

---

## 9. 실행 방법 (집에서 다시 띄울 때)

### 터미널 A — 받는 쪽

```bash
ros_set
ros2 launch pipe_comm monitor.launch.py
```

### 터미널 B — 보내는 쪽 (Isaac)

```bash
isaac_ros                       # 🚨 ros_set 을 같이 하면 안 된다
cd ~/cobot3_ws/src/son
isaac_python repair_demo.py --ros --hold
```

### 터미널 C — 브라우저로 카메라 보기

```bash
ros_set
ros2 run pipe_comm web_view --ros-args -p ns:=robot
```
→ `ssh -L 8080:localhost:8080 ubuntu@<서버>` 후 `http://localhost:8080`

### 터미널 D — 지령 (선택)

```bash
ros2 run pipe_comm mission_cli -- STOP
```

---

## 10. 오늘 걸린 함정 (다시 밟지 말 것)

| 증상 | 원인 | 조치 |
|---|---|---|
| `rclpy 없음` → **세그폴트 + 코어덤프** | `isaac_ros` 를 안 했는데 `--ros` 를 줬다. Isaac 이 다 뜬 뒤 `SystemExit` 을 던지면 종료 경로에서 터진다 | `isaac_ros` 먼저. (검사를 SimulationApp **앞**으로 옮기는 게 근본 해결 — 아직 안 함) |
| 카메라 프레임 0 | `--headless` 는 `world.step(render=False)` 라 렌더가 안 돈다 | `--headless` 를 빼면 된다. X 없이도 `isaac_python` 이 스트리밍을 켜서 렌더가 돈다 |
| 시퀀스가 끝나면 발행이 멈춤 | 주 루프의 `if state == "DONE": ... continue` 가 나머지를 건너뛴다. 발행 블록이 그 **뒤**에 있었다 | **고침** — 발행 블록을 DONE 검사 **앞**으로 옮겼다 |
| WebRTC 에 아무것도 안 뜸 | `livestream.py` 가 `src/dongmin/graphic_file/scripts/` 에 있었는데 그 디렉터리를 비우면서 같이 지워졌다. git 에도 없었다 | **고침** — `tools/isaac_autostream/livestream.py` 로 다시 만들었고, `sitecustomize.py` 가 **자기 옆을 먼저** 보게 했다 |
| 스트리밍 켜면 세그폴트 | `isaacsim.exp.full` 이 `isaacsim.ros2.bridge` 를 물고 오는데 그 확장이 rclpy 를 로드하는 순간 죽는다. PYTHONPATH 와 무관 | **고침** — `isaacsim.exp.base` 로 바꿨다(UI 있음 / ros2 bridge 없음) |
| 브라우저로 WebRTC 접속 안 됨 | `omni.kit.livestream.webrtc` 는 **백엔드 전용**. HTML 클라이언트가 번들에 없고 49100 은 WebSocket 시그널링이다 | **Isaac Sim WebRTC Streaming Client** 앱 필요. 카메라만 볼 거면 `web_view` 가 낫다 |
| `rqt` 가 안 뜸 | 디스플레이가 없다 | `web_view` 또는 `camera_monitor -p save_dir:=...` |
| pytest 가 수집 단계에서 죽음 | ROS 소싱 셸에서 ament 플러그인 훅이 새 pytest 와 안 맞는다 | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q` |

---

## 11. 오늘 바뀐 파일

```
 M .gitignore                                   .pytest_cache/ 추가
 M src/dongmin/pipe_comm/README.md              설치·실행 절차
 M src/dongmin/pipe_comm/pipe_comm/contract.py  상대경로, check_env()
 M src/dongmin/pipe_comm/pipe_comm/camera_monitor.py  두절/낡음 구분
 M src/dongmin/pipe_comm/pipe_comm/drive_monitor.py   log_env
 M src/son/repair_demo.py                       --ros (기본 꺼짐)
 M tools/isaac_autostream/sitecustomize.py      livestream 탐색 경로

?? src/dongmin/pipe_comm/ROS2_통신규격.md        팀 전달용 규격서
?? src/dongmin/pipe_comm/pipe_comm/web_view.py   브라우저 카메라 뷰
?? src/dongmin/pipe_comm/tools/export_isaac.py   isaacpjt 내보내기
?? src/dongmin/TODO.md                           이 문서
?? src/dongmin/isaac_bridge/ros_bridge.py       Isaac 쪽 발행자(테스트판)
?? isaacpjt/dongmin/pipe_comm/                   내보낸 사본(생성물)
?? tools/isaac_autostream/livestream.py          되살린 WebRTC 설정
```

🚨 **`docs/` 는 `.gitignore` 대상이다.** 그래서 규격서를 `docs/` 가 아니라
`src/dongmin/pipe_comm/` 에 뒀다. 새 문서를 `docs/` 에 만들면 안 올라간다.

---

## 12. 팀에 넘길 것

- **Isaac 담당자** → `src/dongmin/pipe_comm/ROS2_통신규격.md`
  (13절, 필드 단위까지. 이것만 보고 본판 발행자를 구현할 수 있다)
- **PC1 (통신 담당 PC)** → `src/dongmin` 만 pull 하면 된다.
  `colcon build --packages-select pipe_comm` 하나로 끝난다
- **규약 사본** → `isaacpjt/dongmin/pipe_comm/` (생성물, 손대지 말 것.
  `python3 src/dongmin/pipe_comm/tools/export_isaac.py --write` 로 다시 굽는다)
