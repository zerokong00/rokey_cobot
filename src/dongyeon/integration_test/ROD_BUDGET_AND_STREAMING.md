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

---

## 5. 통신 인터페이스 (ROS2)

발행 노드: `real_map_demo.py`의 `ActiveCamBridge` (node name `repair_robot_active_cam`).
구독 예시: `tools/view_active_cam.py`의 `ActiveCamViewer` (node name `active_cam_viewer`).

QoS(양쪽 동일하게 맞춰야 함): `BEST_EFFORT` / `KEEP_LAST` / `depth=1`.

| 토픽 이름 | 메시지 타입 | 내용 | 주기 |
|---|---|---|---|
| `/repair_robot/active_cam/rgb/compressed` | `sensor_msgs/CompressedImage` | 그때그때 **활성 카메라 하나**의 JPEG(품질 85). `format="jpeg"`, `header.frame_id`에 카메라 이름(`front_camera`/`back_camera`/`torch_camera`) | 10Hz 목표(실측 ~8.5Hz, 물리 스텝 부하에 따라 변동) |
| `/repair_robot/active_cam/which` | `std_msgs/String` | 지금 발행 중인 카메라 이름 그 자체(`data` 필드) — 위 이미지가 어느 카메라 것인지 판별용 | 이미지와 같은 주기로 같이 발행 |

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

### ROS 노드 PC (별도 터미널)

```bash
source /opt/ros/humble/setup.bash
cd ~/cobot3_ws/rokey_cobot/src/dongyeon/integration_test
python3 tools/view_active_cam.py
```
