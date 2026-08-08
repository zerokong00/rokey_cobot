# 용접봉 예산 알고리즘 + 카메라 스트리밍 정리 (2026-08-07)

`real_map_demo.py`(floor1/floor2 통합 미션) 기준. 관련 모듈:
`driver/rod_budget.py`, `welder/weld.py`, `tools/view_active_cam.py`.

---

## 1. 결함 크기 계산

두 가지 경로가 있고, **실제로 쓰는 건 area_px 경로뿐**이다(이유는 아래).

### 1-1. `find_wall_hole()` — real_map_demo.py:1441

전방 카메라 RGB에서 결함(어두운 구멍)을 찾는다.

1. 밝기 임계값(`g < thr`, 결함이 보여야 할 자리 주변 window의 5~90퍼센타일 분포로 결정)보다 어두운 픽셀을 `cv2.connectedComponentsWithStats`로 묶는다.
2. 관 저 끝(먼 배경)과 진짜 결함을 가르기 위해 **Depth 테두리 거리**로 필터링한다 — 결함 테두리는 근접 벽(≈0.1m), 관 저 끝은 먼 벽(0.3~0.5m).
3. 여러 후보 덩어리 중 **결함이 있어야 할 화소 위치(`expect_px`, 기하로 미리 계산)에 가장 가까운 것**을 고른다(가장 큰 덩어리 X — 관 저 끝이 항상 더 크다).
4. 결과: `area_px`(연결된 어두운 덩어리의 전체 픽셀 수), `cx/cy`(중심 좌표), `r_eq_px = sqrt(area_px/π)`.

**중요**: "안쪽 작은 원 vs 바깥쪽 큰 원" 같은 구분은 없다 — 밝기 임계값 아래로 **연결된 영역 전체**가 하나의 결함으로 잡힌다. 두 개의 링(예: 진짜 구멍 + 변색 테두리)이 밝기상 이어져 있으면 바깥쪽까지, 끊겨 있으면 안쪽만 잡힌다.

### 1-2. 등가 직경 환산 — 두 경로

- **`defect_diameter_from_area_px()`** (weld.py:225) — `find_wall_hole()`의 `area_px`를 mm²로 환산. **관통 구멍은 이 경로를 쓴다.**
- **`defect_diameter_mm()`** (weld.py:186) — Depth 프로파일(파인 화소 적분)로 계산. 크랙류에는 유효하지만 **관통 구멍에는 못 쓴다** — 구멍 화소의 depth 값은 결함까지의 거리가 아니라 "구멍 뒤 배경"까지의 거리라, 카메라 각도만으로 값이 수십~수백 배 튄다(실측: 같은 ø38.1mm 결함인데 결함1 875mm vs 결함2 5mm).

두 경로 모두 같은 공식을 쓴다 — 픽셀 하나가 덮는 실공간 크기를 **Depth가 아니라 기하학적 기대 거리**(`bore_r_mm / sin(theta)`, "이 화소가 정상 관벽 위치였다면"의 거리)로 계산해서 곱한다. 이래야 관통구멍의 배경-거리 오염을 안 받는다.

```
theta = 화소가 광축에서 벗어난 각도(카메라 내부파라미터로 계산)
d_bore_mm = bore_r_mm / sin(theta)      # 그 화소의 "정상 벽" 거리
px_w_mm = d_bore_mm / f_px               # 화소 1칸의 실공간 폭(mm)
area_mm2 = area_px × px_w_mm²
diam_mm = sqrt(4 × area_mm2 / π)         # 등가 직경
```

### 1-3. 재확인(RECHECK) — 최댓값 채택

INSPECT(120mm 거리)와 RECHECK(90mm 거리) 두 번 독립 측정하고, **평균이 아니라 최댓값**을 채택한다. 두 측정 차이가 랜덤 잡음이 아니라 거리에 따라 **체계적으로** 벌어지는 게 실측으로 확인됐고(예: 764mm → 1435mm, 재현됨), §8.3 자체가 "과소평가하면 안 된다"는 보수적 설계라 더 큰 쪽을 따른다.

---

## 2. 용접봉 소모량 계산 (§8.1 L_req)

`driver/rod_budget.py::required_length_mm()` — 결함 체적에서 소모 길이(mm)를 역산.

```
L_req = (V_defect × α) / A_rod
```

- `A_rod = π × (2.0mm / 2)²` — 용접봉 단면적(지름 2.0mm 고정)
- `α = 1.2` — 스패터 손실·덧살 여유(VOLUME_MARGIN_FACTOR)
- 관통구멍(`hole`): `V_defect = π × (diameter_mm/2)² × wall_mm × 1.3` (HOLE_VOLUME_FACTOR, 구멍 체적 보정)
- 크랙(`crack`): `V_defect = length_mm × width_mm × depth_mm`

치수를 못 재면(크기 계측 실패) `default_repair_mm = 100.0mm`로 근사한다(§8.1 예시: 구멍 d8/t4 ≈ 100mm 근거).

**호출 지점**: `weld.py::rod_precheck()` — `find_wall_hole()`의 `area_px`가 있으면 위 1-2 경로로 실측 직경을 구해 `required_length_mm("hole", ...)`를 계산하고, 없으면 `default_repair_mm`으로 물러난다.

---

## 3. 용접봉 잔량 확인 (소진 판정)

`driver/rod_budget.py::RodBudget` 클래스.

| 값 | 기본값 | 의미 |
|---|---|---|
| `total_mm` | 1880.0mm | 코일 수용 한계(§8.2). `ROD_TOTAL_MM` 환경변수로 덮어쓸 수 있음(테스트용) |
| `reserve_mm` | 10.0mm | 소진 판정 예비량 — **쓰기 전에** 이 아래로 내려가면 소진 |
| `default_repair_mm` | 100.0mm | 치수 미계측 결함의 예상 소모량 |

```
remaining_mm = max(0, total_mm - used_mm)
can_repair(need) = (remaining_mm - need) >= reserve_mm
```

### 판정 시점 — **용접 시도 전에** 먼저 묻는다 (2026-08-07 재배선)

처음엔 "용접 다 하고 나서(VERIFY) 잔량 확인" 순서였는데, 이러면 "다음 결함을 발견했지만 감당 못 해서 시도하지 않고 복귀"가 구조적으로 불가능했다(용접 다 하고 나서 확인하니 항상 늦음). 지금은 **INSPECT 직후**(결함을 막 발견한 시점)에 확인한다:

1. **INSPECT** — 결함 등록 직후 `sequencer.rod_precheck()`로 예상 소요량 계산 → `rod_budget.mark_exhausted_if_needed(req)`. 감당 못 하면 이 결함은 **용접 시도조차 안 하고** 바로 `RETURN`.
2. **RECHECK** — 재측정값(최댓값 채택)으로 다시 확인. 여기서도 감당 못 하면 `RETURN`.
3. **VERIFY**(정렬 성패와 무관, 아크가 이미 일어났으므로) — `rod_budget.consume(rod_required_mm)`으로 실제 잔량을 깎는다.

소진 사유는 **관 단절**과 **용접봉 소진** 둘뿐이다(임무 규칙 8) — 기존 `RETURN` 상태(원래 관 단절 전용)를 그대로 재사용하고 `_return_cause`로 실제 사유를 구분해 보고 메시지에 남긴다.

---

## 4. 실행 명령어

### 4-1. Isaac Sim PC — floor2 전체 미션

```bash
isaac_ros    # LD_LIBRARY_PATH 설정 (ROS2 스트리밍 쓰려면 필수, ~/.bashrc alias)
cd ~/cobot3_ws/rokey_cobot/src/dongyeon/integration_test
PYTHONUNBUFFERED=1 isaac_python real_map_demo.py --shots
```

- `--shots`(선택, ROS2 스트리밍과 무관): INSPECT/RECHECK/VERIFY 등 단계마다 front/back 카메라 PNG를 `out/real_map_floor2/`에 저장(torch_camera는 제외). ROS2로만 실시간 확인할 거면 안 붙여도 된다 — 발행은 `--shots` 여부와 상관없이 항상 켜진다(rclpy 초기화만 되면).
- floor2가 기본값 (floor1은 `--floor1` 추가)
- GUI 모드 필수 — headless는 카메라 프레임이 발행되지 않음
- 부팅 ~15초, 첫 용접 도달까지 총 2~3분

**자주 쓰는 추가 옵션**:
- `ROD_TOTAL_MM=5000` — 예산을 넉넉히 줘서 무조건 용접까지 도달하게 강제(기본값 1880mm으로는 예산 부족 판정으로 용접 없이 복귀할 수도 있음)
- `--no-ros` — ROS2 스트리밍 끄기
- `CAM_DIAG=1` — 카메라 설정 직후 진단 로그 찍고 즉시 종료(빠른 방향 확인용)
- `ARC_CHECK=1` — 용접(ARC) 상태 중간 지점에서 스냅샷 찍고 자동 종료

### 4-2. ROS2 PC(또는 같은 PC의 별도 터미널) — 영상 확인

**반드시 Isaac Sim을 띄운 터미널과는 별개의 새 터미널**에서 실행할 것(같은 터미널에서 ROS Humble을 source하면 Python 3.10/3.11 심볼 충돌).

```bash
source /opt/ros/humble/setup.bash
cd ~/cobot3_ws/rokey_cobot/src/dongyeon/integration_test
python3 tools/view_active_cam.py
```

- `q` 또는 `ESC`로 종료
- 창 좌상단에 현재 활성 카메라 이름 + 수신 프레임 번호 오버레이
- CLI로 빠르게만 확인하려면:
  ```bash
  ros2 topic list | grep repair_robot
  ros2 topic echo /repair_robot/active_cam/which
  ros2 topic hz /repair_robot/active_cam/rgb/compressed
  ros2 run rqt_image_view rqt_image_view   # GUI로 직접 보고 싶으면
  ```

### 4-3. YOLO 2차검증 PC(PC2, 선택) — active_cam_vision

`active_cam` 뷰어와는 별개로, `/repair_robot/active_cam/{rgb/compressed,which}`를 그대로 구독해 YOLO 추론을 돌리고 싶을 때만 켠다. 안 켜도 미션은 정상 진행되고(§5-2), 로그에 "의견 없음"만 찍힌다.

```bash
source /opt/ros/humble/setup.bash
cd ~/cobot3_ws
colcon build --packages-select pipe_inspect_demo   # 최초 1회 (워크스페이스 루트는 rokey_cobot이 아니라 그 상위 ~/cobot3_ws)
source install/setup.bash
ros2 run pipe_inspect_demo active_cam_vision
```

- 출력: `/defect/detected`(Bool), `/defect/report_json`(String, JSON), `/defect/debug_image/compressed`(`yolo_viewer`용) — `real_map_demo.py`의 `ActiveCamBridge`가 앞의 둘을 구독해 §5-2 로그를 만든다.
- Isaac Sim PC와 분리된 별도 PC 권장(GPU 컨텐션 회피) — 같은 PC에서 돌려도 동작은 하지만 두 프로세스가 GPU를 나눠 써서 느려질 수 있다.
- 예전엔 `pipe_vision`(front_camera 고정, Depth/Odom/IMU 필요)을 썼는데, 한 로봇이 정찰+수리를 동시에 하도록 구조가 바뀌면서 축 위치 추적(Odom/IMU)이 필요 없어져 `active_cam_vision`으로 교체했다(2026-08-07) — **`pipe_vision`과 동시에 띄우지 말 것**, 둘 다 같은 `/defect/detected`,`/defect/report_json`을 발행해서 충돌한다.

---

## 5. 통신 인터페이스 (ROS2)

발행 노드: `real_map_demo.py`의 `ActiveCamBridge` (node name `repair_robot_active_cam`).
구독 예시: `tools/view_active_cam.py`의 `ActiveCamViewer` (node name `active_cam_viewer`).

QoS(양쪽 동일하게 맞춰야 함): `BEST_EFFORT` / `KEEP_LAST` / `depth=1`.

| 토픽 이름 | 메시지 타입 | 내용 | 주기 |
|---|---|---|---|
| `/repair_robot/active_cam/rgb/compressed` | `sensor_msgs/CompressedImage` | 그때그때 **활성 카메라 하나**의 JPEG(품질 85). `format="jpeg"`, `header.frame_id`에 카메라 이름(`front_camera`/`back_camera`/`torch_camera`) | 10Hz 목표(실측 ~8.5Hz, 물리 스텝 부하에 따라 변동) |
| `/repair_robot/active_cam/which` | `std_msgs/String` | 지금 발행 중인 카메라 이름 그 자체(`data` 필드) — 위 이미지가 어느 카메라 것인지 판별용 | 이미지와 같은 주기로 같이 발행 |

### 5-1. active_cam 에 얹은 YOLO 2차검증 (2026-08-07, `PipeVisionBridge`에서 교체)

처음엔 `pipe_vision_node`(front_camera 고정, RGB/Depth/CameraInfo/Odom/IMU 5-토픽 동기화, Odom/IMU 기반 축 위치 추적)를 위해 별도 `PipeVisionBridge`를 만들었는데, 정찰+수리를 **한 로봇**이 동시에 하는 구조로 바뀌면서 축 위치를 YOLO 쪽에서 따로 추적할 이유가 없어졌다(이미 OpenCV 쪽 코스 s좌표로 확정됨) — 그래서 **Depth·Odom·IMU가 아예 필요 없는** 새 노드 `active_cam_vision`(`pipe_inspect_demo/pipe_inspect_demo/active_cam_vision_node.py`)으로 교체했다. `PipeVisionBridge` 클래스와 `/rgb`,`/depth`,`/camera_info`,`/odom`,`/imu`,`/inspection/reset` 토픽은 real_map_demo.py에서 완전히 제거됐다.

`active_cam_vision`은 **이미 발행 중인 `/repair_robot/active_cam/{rgb/compressed,which}`를 그대로 구독**한다 — Isaac 쪽에 새 발행자가 필요 없다(§5 표의 두 토픽 재사용). 카메라가 front/back/torch 뭐로 바뀌든 상관없이 들어오는 프레임에 YOLO Seg를 돌린다. YOLO 2차검증 의견(`/defect/detected`,`/defect/report_json`) 구독은 `real_map_demo.py`의 `ActiveCamBridge`가 맡는다(`yolo_opinion()` 메서드, node는 그대로 `repair_robot_active_cam` 하나).

| 토픽 이름 | 메시지 타입 | 내용 | 방향 |
|---|---|---|---|
| `/defect/detected` | `std_msgs/Bool` | 이번 프레임에 결함(등록 문턱 0.8 이상)을 봤는지 | active_cam_vision → Isaac |
| `/defect/report_json` | `std_msgs/String` | JSON — `camera`(어느 액티브캠), `class`,`confidence`,`area_px`,`center_pixel`,`detections_count`. **축 위치 필드 없음**(Odom 없이 계산 불가) — `_log_yolo_opinion`이 `try/except KeyError`로 이미 방어돼 있어 축위치 비교만 생략되고 에러는 안 남 | active_cam_vision → Isaac |
| `/defect/debug_image/compressed` | `sensor_msgs/CompressedImage` | YOLO 박스/마스크 오버레이 — `yolo_viewer`가 구독. 결함이 화면 밖으로 나가도 마지막 검출을 계속 그린다(고정 위치라 실제 화면과 안 맞을 수 있음, 시각 참고용) | active_cam_vision → (yolo_viewer) |

Depth 없이 추론하는 경로(`YoloSegDetector.infer_rgb_only()`)는 물리 치수(길이/폭 mm)를 측정하지 않는다 — 애초에 YOLO는 참고용 2차 의견이라 크기는 OpenCV 쪽(§1) 것만 쓴다.

**알려진 한계**: active_cam 세 카메라(front/back/torch) 모두 어안 140°인데 YOLO 모델은 640×640 핀홀로 학습됐다 — 화면 가장자리·극단적 근접(용접 중 torch_camera처럼)일수록 검출 정확도가 떨어질 수 있다. 특히 ARC(용접) 중엔 토치 팁/로드와 아크 스파크(시각 전용 효과)가 결함 자리를 실제로 가리므로, 그 순간 미검출은 학습 문제라기보다 원래 안 보이는 장면일 가능성이 크다(OpenCV 쪽도 같은 이유로 VERIFY를 ARC 도중이 아니라 COOL/REPOSITION 이후에 함).

### 5-2. OpenCV 1차 · YOLO 2차검증 (2026-08-07 추가)

배경: 기존 `pipe_vision_node`는 정찰 로봇 + 수리 로봇 2대 구조를 전제로 만들어졌는데, 지금은 한 로봇이 정찰과 수리를 동시에 한다. 그래서 **OpenCV(`find_wall_hole`/`find_weld_bead`)가 1차·확정 판정**을 그대로 유지하고, **YOLO는 2차·참고용 의견**으로만 나란히 로그에 남긴다 — YOLO가 없거나(PC2 미실행) 느려도 미션은 절대 안 죽는다(`yolo_opinion(max_age_s=3.0)`이 3초 넘게 소식 없으면 `None`="의견 없음"으로 명확히 구분, "결함 없다고 봄"과 혼동하지 않음).

| 시점 | 로그 태그 | 비교 대상 |
|---|---|---|
| INSPECT (결함 등록 직후) | `[YOLO 2차검증·INSPECT]` | OpenCV 검출 여부 + 축위치(mm) 비교 |
| RECHECK (재확인 촬영 후) | `[YOLO 2차검증·RECHECK]` | 위와 동일 |
| VERIFY (용접 완료 판정 직후) | `[YOLO 용접완료·2차검증]` | 비드(색) 판정(`ok`)과 YOLO의 "여전히 크랙 보이는가"(반전 비교) — 일치/불일치만 로그, **판정 자체는 그대로 비드가 함** |

절대 YOLO 의견으로 OpenCV/비드 판정을 대체하지 않는다 — `weld.py` 자체 문서가 경고하는 "자기충족적 검출기" 함정을 피하기 위한 설계.

2026-08-07 실측 확인(`--floor2 --glass --hold`, PC2/YOLO 미실행 상태 — 당시엔 `PipeVisionBridge` 기준으로 확인했고 이후 `active_cam_vision`으로 교체했지만 "구독자 없으면 None" 방어 로직은 동일하게 옮겨졌다): INSPECT/RECHECK/VERIFY 세 지점 모두 에러·Traceback 없이 "의견 없음" 로그가 정상 출력됨 — YOLO 부재 시 우아한 저하(graceful degradation) 확인됨.

### 활성 카메라 전환 규칙 — `active_camera_name(state)`

FSM 상태별로 어느 카메라를 내보낼지 결정한다(real_map_demo.py):

| FSM 상태 | 활성 카메라 |
|---|---|
| `RETURN`, `RECOVER` | `back_camera` (후진) |
| `ALIGN`, `EXTEND`, `ARC` | `torch_camera` (정렬~용접) |
| 그 외(`COOL`, `REPOSITION`, `VERIFY`, `SETTLE`, `CRUISE`, `INSPECT`, `RECHECK`, `JUNCTION`, `RESUME`, `DISCONNECTED`, `DONE` 등) | `front_camera` (전진/기본) |

용접(ARC)이 끝나면 COOL/REPOSITION/VERIFY(식히고 물러나 검증 촬영하는 사후 단계)부터는 바로 `front_camera`로 돌아온다 — torch_camera는 실제 정렬~용접 구간에만 쓴다(2026-08-07 변경).

### 인코딩 규약 (색상 채널 순서 — 헷갈리기 쉬운 부분)

- Isaac 쪽 annotator "rgb"는 RGB 순서로 나온다.
- `ActiveCamBridge.publish()`가 `a[:, :, ::-1]`로 **BGR로 뒤집은 뒤** `cv2.imencode(".jpg", ...)` — `camera/rig.py`의 기존 `CameraBridge`와 같은 관례.
- 받는 쪽(`view_active_cam.py`)은 `cv2.imdecode(..., cv2.IMREAD_COLOR)`로 디코딩하면 **그대로 BGR**이 나오고, `cv2.imshow`가 기대하는 순서와 일치 — 별도 반전 불필요.

### rclpy 부트스트랩 (Isaac Sim 쪽, Python 3.11)

`import rclpy`만 하면 `ModuleNotFoundError` — 반드시 이 순서:

```python
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<isaac설치>/exts/isaacsim.ros2.bridge/humble/lib   # 셸에서, isaac_ros alias
```
```python
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")   # import rclpy 전에 반드시 먼저
import rclpy
```

Isaac Sim에 Humble/Python 3.11용 rclpy가 이미 번들되어 있다(`exts/isaacsim.ros2.bridge/humble/rclpy`, `rclpy-3.3.17-py3.11`) — `pyver.py` 문서의 "IsaacSim-ros_workspaces로 직접 빌드해야 한다"는 안내는 이 설치 기준으로는 불필요(낡은 정보로 확인됨, 2026-08-07).

`rclpy` 로드 실패 시(LD_LIBRARY_PATH 안 잡혔거나 등) `real_map_demo.py`는 죽지 않고 `NO_ROS=True`로 강등해 ROS2 발행만 끈 채 나머지 미션(주행·용접·카메라 추적)은 그대로 진행한다.

---

## 6. 빠른 참조 — 실행 명령어만

### Isaac Sim PC

```bash
isaac_ros
cd ~/cobot3_ws/rokey_cobot/src/dongyeon/integration_test
PYTHONUNBUFFERED=1 isaac_python real_map_demo.py
```

`--shots`는 안 붙여도 ROS2 영상은 그대로 나온다 — PNG 파일도 같이 남기고 싶을 때만 끝에 추가.

### ROS 노드 PC (별도 터미널) — 영상 확인

```bash
source /opt/ros/humble/setup.bash
cd ~/cobot3_ws/rokey_cobot/src/dongyeon/integration_test
python3 tools/view_active_cam.py
```

### YOLO 2차검증 PC (선택, 별도 터미널/PC)

```bash
source /opt/ros/humble/setup.bash
source ~/cobot3_ws/install/setup.bash
ros2 run pipe_inspect_demo active_cam_vision
```
