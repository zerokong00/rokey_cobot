# pipe_comm — 배관 점검 로봇 ROS 2 통신

Isaac Sim 쪽(발행)과 ROS 2 노드 쪽(수신·지령)이 주고받는 **규약과 감시 도구**.

```
pipe_comm/
├─ contract.py        🔑 토픽 이름·JSON 스키마 **단일 출처**. 양쪽이 이것만 읽는다
├─ image_codec.py     영상 인코딩/디코딩 (cv_bridge 없이)
├─ camera_monitor.py  카메라가 오는지 검증하는 노드
├─ drive_monitor.py   주행/정지 알림을 보는 노드 (로봇 여러 대 동시)
└─ mission_cli.py     출발·정지·복귀 지령을 쏘는 CLI
```

---

## 5분 요약

| | |
|---|---|
| 도메인 | **`ROS_DOMAIN_ID=143`** — 다르면 토픽이 아예 안 보인다 |
| 미들웨어 | `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (양쪽 같아야 한다) |
| 시연 구성 | **RTX 5080 MSI PC 한 대**에서 Isaac 과 ROS 노드를 같이 돌린다 |
| 메시지 | 경계는 **표준 메시지만**. 커스텀 `.msg` 없음 → 받는 쪽 빌드 불필요 |
| 구조체 | `std_msgs/String` 에 JSON. 스키마는 `contract.py` |
| 영상 QoS | **BEST_EFFORT** — RELIABLE 로 구독하면 프레임이 0 이다 |

---

## 🚨 PC 번호로 부르지 않는다

팀 관례는 `PC1 = 카메라·검출 쪽`, `PC2 = Isaac Sim` 인데 번호는 읽는 사람마다
뒤집힌다(이 규약을 처음 쓸 때 실제로 반대로 적었다). 그래서 문서·코드 전부
**역할 이름**만 쓴다 — **`Isaac 쪽`(3.11)** 과 **`ROS 노드 쪽`(3.10)**.

---

## 왜 커스텀 msg 를 안 쓰는가

**Isaac Sim 은 Python 3.11, 우리 ROS 2 Humble 은 3.10 이다.** 커스텀 msg 는
rosidl 이 만든 3.10 용 `.so` 에 묶여 있어 Isaac 쪽에서 못 쓴다. Isaac 5.1 의
내장 rclpy 는 `isaacsim.ros2.bridge/humble` 에 박제된 3.11 ABI 라 우리
워크스페이스를 그대로 얹을 수 없다.

표준 메시지는 그 번들에 **이미 들어 있으므로 빌드가 0 이다.** 구조가 필요한
값은 `std_msgs/String` 에 JSON 으로 싣는다. 부수 효과로 받는 쪽이 이 패키지를
빌드하지 않아도 `ros2 topic echo` 로 바로 볼 수 있다.

`pipe_msgs` 는 레포에 없고 앞으로도 만들지 않는다.

---

## 토픽 지도

로봇 한 대 = 네임스페이스 하나. `map_test_demo.py` 는 코스마다 로봇이 한
대씩이라 `/elbow_v` `/elbow_h` `/tee` 로 갈라지고, `repair_demo.py` /
`real_map_demo.py` 는 한 대라 `/robot` 하나다.

### Isaac → ROS 노드 (주는 데이터)

| 토픽 | 타입 | 내용 | 주기 |
|---|---|---|---|
| `rgb/compressed` | `CompressedImage` jpeg | 전방 카메라 | 10Hz |
| `depth/compressed` | `CompressedImage` **16UC1 PNG(mm)** | 전방 깊이 | 10Hz |
| `depth` | `Image` 32FC1(m) | 원본 정밀도. **선택** | 10Hz |
| `camera_info` | `CameraInfo` | 어안 f 를 `K[0]` 에 | 10Hz |
| `rear/rgb/compressed` | `CompressedImage` | 후방 카메라 | 10Hz |
| `odom` | `Odometry` | **`pose.position.x` = 중심선 호길이** | 10Hz |
| `imu` | `Imu` | 롤(결함 시계각 기준)·요 각속도 | 50Hz |
| `joint_states` | `JointState` | 휠 12 + 피스톤 6 | 10Hz |
| `moving` | `Bool` | 주행 중인가 — **주행/정지 알림** | 전환 시 |
| `drive_state` | `String` JSON | FSM 상태 전체 | 10Hz |
| `event` | `String` JSON | 1회성 사건 | 사건 시 |

### ROS 노드 → Isaac (받는 데이터)

| 토픽 | 타입 | 내용 |
|---|---|---|
| `cmd_vel` | `Twist` | `linear.x` 만 (m/s). 음수 = 후진 |
| `mission` | `String` JSON | `START` `STOP` `RECALL` `RETRY` `SPEED` `ESTOP` |
| `repair_target` | `String` JSON | 결함 위치(호길이 + 시계각) → 용접 정렬 |

### JSON 스키마

```jsonc
// drive_state — 10Hz
{"stamp":1.23,"robot":"elbow_v","step":2963,"state":"RUN","moving":true,
 "dir":1,"speed_mps":0.1,"s_mm":431.2,"s_total_mm":935.6,"off_mm":3.44,
 "lap":2,"stuck":0,"reason":""}

// event — 사건 때만. alert=true 면 사람이 바로 봐야 한다
{"stamp":1.23,"robot":"tee","event":"STUCK","s_mm":431.2,
 "detail":"방향 전환 → 후진","alert":true}

// mission
{"stamp":1.23,"cmd":"SPEED","reason":"곡관 진입","mps":0.05}

// repair_target — clock_deg 0°=천장, 180°=바닥(중력)
{"stamp":1.23,"id":"crack_0001","s_mm":1130.0,"clock_deg":180.0,
 "width_mm":2.2,"depth_mm":2.2,"confidence":0.87}
```

상태값 / 사건 / 지령의 **전체 목록은 `contract.py` 의 상수**다. 문자열을 손으로
적지 말 것 — 모르는 값을 주면 `ValueError` 로 바로 걸리게 해 뒀다.

---

## 규약이 정하는 것들 (틀리면 조용히 망가지는 자리)

| 항목 | 규약 | 안 지키면 |
|---|---|---|
| Depth 압축 | **16UC1 PNG, mm, 0=무효** | JPEG 는 손실이라 깊이가 훼손된다 |
| Depth 무효 | 캐스팅 **전에** 0 으로 | `inf`/`NaN` 이 쓰레기 정수가 되어 관 단절이 "가까운 벽"이 된다 |
| Depth 상한 | 65m 초과는 무효 | uint16 wrap — 먼 곳이 코앞으로 보인다 |
| RGB 채널 | cv2 에 넘길 때 BGR | R/B 가 바뀐 채로 YOLO 에 들어간다(검출률만 떨어짐) |
| RGB 알파 | RGBA 4채널을 3으로 자름 | Isaac annotator 는 RGBA 를 준다 |
| `odom.x` | **중심선 호길이** | 직선거리를 쓰면 곡관에서 위치가 뒤로 간다 |
| `K[0]` | 등거리 어안 f (r=f·θ) | 핀홀로 역산하면 입사각이 틀어진다 |
| 시계각 | 0°=천장, 180°=바닥 | 부호가 뒤집히면 **토치를 정반대로 돌린다** |
| 영상 QoS | BEST_EFFORT | 토픽은 보이는데 프레임이 0 |
| 지령 QoS | RELIABLE | 정지 지령을 놓친다 |

---

## 실행

```bash
ros_set                                   # ROS 2 소싱 (python 3.10)
export ROS_DOMAIN_ID=143

# 카메라가 오는지
ros2 run pipe_comm camera_monitor --ros-args -p ns:=elbow_v

# 주행/정지 알림 — 로봇 3대를 한 노드에서
ros2 run pipe_comm drive_monitor --ros-args -p ns:="[elbow_v,elbow_h,tee]"

# 둘 다 한 번에
ros2 launch pipe_comm monitor.launch.py ns:=all

# 지령
ros2 run pipe_comm mission_cli -- STOP --ns elbow_v
ros2 run pipe_comm mission_cli -- SPEED --mps 0.05 --ns all
```

`mission_cli` 는 발행 뒤 **구독자 수를 확인하고 0 이면 0 이 아닌 종료코드**를
낸다 — RELIABLE 이라도 구독자가 없으면 지령은 그냥 사라지기 때문이다(latch 가
아니다). 스크립트에서 `&&` 로 이어 쓸 수 있다.

---

## 오프라인 검증

ROS 없이 `python3 -m pytest test/` 로 돈다 (18항목).

```
test_contract.py     토픽 이름·JSON 스키마·모르는 값 거부
test_image_codec.py  RGB 채널 순서, 알파 제거, 깊이 무효/오버플로 처리
```

🔑 규약이 깨지는 것은 **런타임에 침묵으로** 나타난다(토픽 오타 → 아무도 못
받음, 깊이 무효 처리 실패 → 판정이 조용히 뒤집힘). 그래서 여기서 미리 건다.

---

## 문제가 생기면 보는 순서

1. `ros2 topic list` 에 아무것도 없음 → `ROS_DOMAIN_ID` 143 인가, 양쪽
   `RMW_IMPLEMENTATION` 이 같은가
2. 토픽은 있는데 프레임 0 → 구독 QoS 가 BEST_EFFORT 인가, Isaac 이 **Play**
   상태인가
3. 영상이 검음 → 관 내부는 로봇 조명이 유일한 광원이다. 조명 intensity 확인
4. Depth 유효율이 낮음 → near clip 이 관벽(25mm)을 자르는가, 카메라가 본체에
   파묻혔는가
5. `drive_monitor` 가 **⛔ 두절** → Isaac 프로세스가 살아 있는가
6. `mission_cli` 가 **구독자 0** → Isaac 쪽 시연이 떠 있는가, `--ns` 가 맞는가

---

## 이전 패키지에서 바뀐 것

`src/dongmin` 의 `M0609` / `pipe_inspect` 를 지우고 이 패키지로 다시 만들었다
(2026-08-06). 옛 `pipe_inspect/camera_check.py` 는 `/rgb` `/depth` 를 **raw
Image** 로 받았는데, 같은 팀의 `dongyeon/pipe_vision_node` 는
`/rgb/compressed` `/depth/compressed` 를, `son/camera/rig.py` 는 `front/rgb`
`front/depth`(32FC1) 를 써서 **세 갈래로 갈려 있었다.** 어느 둘을 붙여도 안
맞는 상태였다. `contract.py` 하나로 합친 것이 이 패키지의 존재 이유다.
