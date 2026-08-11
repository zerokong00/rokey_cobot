# v1_3 통합 — 동민 작업 안내 (2026-08-11)

받는 사람: 동민 (`pipe_comm` · `web_panel` 담당)
보내는 쪽: 손영빈 (`src/son`, Isaac 시연 본판)

**요약 한 줄**: 시연 본판이 `real_map_demo_v1_3.py` 로 바뀌었고, **로봇 2대가
네임스페이스 `/floor1`·`/floor2` 로 동시에 발행**한다. 규약(`contract.py`)은
**고치지 않았다** — 토픽 이름도 스키마도 그대로다. 웹 쪽에서 할 일은
**네임스페이스 2개를 띄우는 것**과 **카메라 역할 매핑 한 줄**뿐이다.

---

## 1. 지금 git 에 있는 것과 무엇이 달라졌나

| | 오늘 pull 한 상태 | v1_3 (지금) |
|---|---|---|
| 시연 스크립트 | `real_map_demo_v1_2.py` (한 번에 한 층) | **`real_map_demo_v1_3.py`** (층 개별 + 동시) |
| 로봇 | 1대 | **2대 동시 가능** (`/floor1`, `/floor2`) |
| ROS 발행 | 없었음 | **`isaac_bridge/ros_bridge.py` 그대로 사용** |
| 카메라 | front/back | **로봇당 1대** (아래 2절) |
| 도메인 | 문서 143 / 코드 50 혼재 | **143 으로 통일** |

`ros_bridge.py` 와 `contract.py` 는 **네 코드를 그대로 import 해서 쓴다.**
내가 고친 곳은 없다.

---

## 2. 카메라 — 여기만 매핑을 맞춰 주면 된다 🔑

렌더 비용 때문에 **로봇당 카메라 1대**만 둔다. 층마다 역할이 다르다:

| 로봇 | 카메라 | 발행 토픽 | 비고 |
|---|---|---|---|
| `/floor1` | 전방 | **`/floor1/rgb/compressed`** + `/floor1/camera_info` | 깊이도 여기(`/floor1/depth/compressed`) |
| `/floor2` | 용접부 | **`/floor2/torch/rgb/compressed`** + `/floor2/torch/camera_info` | 결함·용접봉이 보이는 근접 시점 |

- **`rear/…` 는 더 이상 발행되지 않는다** (후방 카메라 폐지). 웹 카메라 화면이
  3분할이면 그 칸은 비게 되므로, floor1=전방 / floor2=용접 **2분할**을 권한다.
- **둘 다 10Hz 로 항상 발행한다.** 상황에 따라 껐다 켜지 않는다
  (실측 확인: 20초 구독에 각 94건 수신).
- `drive_state` JSON 에 `cam` 필드가 있다 — 그 로봇이 지금 내보내는 역할
  이름(`"front"` / `"torch"`)이다. 화면 라벨에 그대로 쓰면 된다.

---

## 3. 웹 패널 띄우는 법 (층마다 하나씩)

```bash
ros_set
export ROS_DOMAIN_ID=143              # 🚨 규격서 값. 50 아님
ros2 run pipe_comm web_panel --ros-args -p ns:=floor1 -p port:=8080
ros2 run pipe_comm web_panel --ros-args -p ns:=floor2 -p port:=8081
```

→ `http://<IP>:8080` (아래층) · `http://<IP>:8081` (윗층)

한 화면에 둘을 같이 보고 싶다면 `web_panel` 이 네임스페이스 하나만 받는
구조라 **패널을 두 개 띄우는 것이 지금으로선 가장 빠른 길**이다. 한 패널에서
층을 전환하고 싶으면 `ns` 를 런타임 파라미터로 바꿀 수 있게 해 주면 된다
(그쪽 판단에 맡긴다).

---

## 4. 그대로 쓰는 것 (확인만)

전부 `contract.py` 규약대로 나간다. 스키마 변경 없음.

- `drive_state` (10Hz, RELIABLE) — 규약 필드 + **추가 필드 2개**
  - `pos_m`: `[x, y, z]` 월드 좌표(m). T 분기 맵에서 s 만으로는 가지를 못
    가리므로 넣었다. 규약의 추가 필드라 스키마 위반이 아니다.
  - `cam`: 지금 내보내는 카메라 역할 이름.
- `odom` / `imu` / `joint_states`(바퀴 12개) / `moving`
- `course` — 기동 직후 1회 latched. 표본을 60점 안팎으로 늘렸다(전엔 4점이라
  튜브가 각졌다).
- `mesh` — `restroom_final0807.webmesh` 1.4MB 1회 latched.
- `event` — 규약 상수 그대로. **용접 사건 3종이 실제로 나간다**:
  - `DEFECT` (결함 앞 정지) · `WELD_BEGIN` (아크 점화) · `WELD_DONE` (비드 형성)
  - 셋 다 추가 필드 `clock_deg`(결함의 시계 방향)와 `s_mm` 을 싣는다.
- 구독: `mission`(`start`/`stop`/`resume`/`estop`) · `cmd_vel`
  - `mission` 은 실제로 물려 있다 — 정지 지령을 주면 그 로봇이 그 자리에서
    선다. 웹 버튼으로 시험해 봐 주면 좋겠다.

---

## 5. 상태값 매핑 (규약 STATE_* 그대로)

| 상황 | 보내는 값 |
|---|---|
| 투입 직후 안착 | `SETTLE` |
| 주행 | `RUN` |
| 웹 정지 지령 | `HOLD` |
| 복귀 주행 | `RETURN` |
| 결함 앞 정지·정렬 | `INSPECT` |
| 아크 용접 중 | `REPAIR` |
| 이탈 정지 | `DEAD` |

---

## 6. Isaac 쪽 실행 (참고용 — 그쪽에서 띄울 일은 없음)

```bash
cd ~/cobot3_ws/src/son
./run_v13.sh floor1     # 아래층만
./run_v13.sh floor2     # 윗층만 (결함 2곳 용접)
./run_v13.sh both       # 두 층 동시
```

스크립트가 도메인 143 · Isaac 내장 rclpy 경로를 다 넣는다.

🚨 **Isaac 을 headless 로 돌리면 카메라 프레임이 안 나온다**(기록된 함정).
카메라 화면을 봐야 하는 시험은 반드시 GUI 로 띄운 것과 붙을 것.

---

## 7. 요청 사항 정리

1. **카메라 페이지 매핑**: floor1 → `rgb`, floor2 → `torch/rgb` (2절 표).
   `rear` 구독은 빼도 된다.
2. **패널 2개 기동** 확인 (`ns:=floor1 port:=8080`, `ns:=floor2 port:=8081`).
3. **도메인 143** 로 통일 — 혹시 50 으로 맞춰 둔 곳이 있으면 고쳐 주기.
4. 여유가 되면: `mission` 정지/재개 버튼과 용접 사건 3종(`DEFECT`/
   `WELD_BEGIN`/`WELD_DONE`) 표시가 잘 뜨는지 확인.

막히는 것 있으면 알려줘. 규약을 건드려야 하는 상황이면 **그쪽이 정본**이니
먼저 합의하고 내가 맞추겠다.
