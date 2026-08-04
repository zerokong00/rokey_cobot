# Isaac Sim 실행 검증 보고서 (2026-08-04)

`HANDOFF.md` 를 받아 Isaac Sim 이 있는 장비에서 실제로 실행한 결과다.
작성자 노트북에서 한 번도 안 돌았던 시뮬 쪽 6개 스크립트를 처음 돌린 기록이다.

- 장비 : Ubuntu 22.04.5 / RTX 5080 Laptop 16GB / Isaac Sim 5.1 (Python 3.11.13)
- 방식 : 물리 검증은 headless, 카메라 진단은 GUI (headless 는 카메라 프레임 0)
- 범위 : **PC 간 ROS 통신은 검증 대상에서 제외.** Isaac Sim 내부에서
  구현이 제대로 도는지만 확인했다

---

## 1. 요약

| | 항목 | 결과 |
|---|---|---|
| ① | `camera/depth_probe.py` | ⚠ → ✅  씬 결함 수정 후 통과. 빈 공간 = `inf` |
| ② | `robot/articulate.py` | ✅ 링크 14 / DOF 13, 총질량 500g, 예압 9.0N |
| ③ | `welder/articulate.py` | ⚠ → ✅  즉시 크래시. 수정 후 링크 17 / DOF 15 통과 |
| ④ | `camera/rig.py --save` | ⏸ 미실행 (rclpy 필요) |
| ⑤ | `robot/state_bridge.py` | ⏸ 미실행 (rclpy 필요) |
| ⑥ | `pipe/curve_demo.py` | ❌ **주행 실패. 구동부가 동작하지 않는다** |

**형상·조립·조인트 구성은 설계대로 정확하게 만들어져 있다.
반면 주행 구동부는 관 안에서 전혀 동작하지 않는다.**

`articulate.py` 의 자체 검증은 **자유공간** 기준이라 전부 통과한다.
관 안에 넣는 순간 드러나는 결함이라 그 검증만으로는 잡히지 않았다.

---

## 2. 발견된 결함

### 결함 1 — `welder/articulate.py` 가 실행 즉시 크래시 [치명]

```
File "son/welder/articulate.py", line 64, in part_path
    return SON / _CATEGORY[name] / "meshes" / f"{name}.stl"
KeyError: 'torch_ring'
```

`_CATEGORY` 딕셔너리에 토치 3종(`torch_ring` / `torch_rod` / `torch_tip`)이
없다. 정찰기 `robot/articulate.py` 에서 파일을 복사해 오면서 그쪽에 없던
토치 항목만 추가되지 않은 것으로 보인다.

- 영향 : HANDOFF 실행 순서 ③번이 링크 하나도 못 만들고 죽는다
- 상태 : **수정함** (3-1 참조)

---

### 결함 2 — `camera/depth_probe.py` 가 쓸 수 없는 답을 낸다 [치명]

HANDOFF 가 "①을 건너뛰지 말 것" 이라고 못박은 진단인데, 실행 결과가
`판정불가` 로 나온다.

```
① 빈 공간 (화면 중앙 120x120, 관이 끊긴 방향)
   픽셀 576,000  0=0  NaN=0  inf=0  최대값부근=0  유효=576,000
   유효값 0.2150 ~ 0.2179 m
   → invalid_mode: 판정불가
     판정식: 중앙이 비어 있지 않다 — 씬 배치 확인
```

원인은 진단 씬이다. `build_scene()` 이 관을 `UsdGeom.Cylinder` 로 만드는데
그것은 **양 끝에 뚜껑이 달린 속이 꽉 찬 원기둥**이다. 카메라를 그 안에 두면
화면 중앙에 끝면(0.215m)이 그대로 잡힌다. 빈 공간을 잰 것이 아니라 벽을 잰
것이라 진단이 성립할 수 없다.

- 영향 : `invalid_mode` 를 확정할 수 없다 → `pipe_condition.yaml` 을 못 채운다
  → **관 단절 판정이 조용히 작동하지 않는다.** HANDOFF 가 경계한 바로 그 형태
- 상태 : **수정함** (3-2 참조)

---

### 결함 3 — 곡관 주행이 동작하지 않는다 [치명, 미해결]

원본 그대로 실행한 결과:

```
     step     x(mm)     z(mm)     관절(°)     암 min~max(°)
        0    -544.6      -0.4        1.73    -1.20~  -0.02
      160    -591.5     -15.2       39.02    -7.70~  -0.23
      320    -604.7     -23.5       55.00    -5.50~  -0.23
     ...     (이후 3999 스텝까지 완전 정지)
  최대 관절각    55.00°  (한계 55°)
  곡관 진입      아니오
  [FAIL] 직관에서 전진하지 못했다
```

전진은커녕 **뒤로 105mm 밀리고**, 관절이 한계 55°까지 접혀(잭나이프) 굳는다.
암 각도가 −7.70 ~ −0.02° 로, `articulate.py` 자유공간 검증에서 6개 전부
+13.40°(상한)까지 벌어지던 것과 대조된다. **예압이 사라진 상태다.**

원인을 가르려고 한 번에 한 조건씩 6회 실험했다.

| 실험 | 조작 | 결과 |
|---|---|---|
| 원본 | — | 뒤로 105mm, 잭나이프 55°, 암 −7.70~−0.02 |
| A | 구동 부호만 반전 | **앞으로** 22mm 이동 후 정지 |
| B | 부호 + 암 드라이브 타깃 재설정 | 관 중앙에 정확히 안착(z −0.1mm, 관절 −0.48°, 암 6개 대칭) 후 1.5mm만 이동 |
| 진단 | 휠 각속도 로깅 | **36스텝 이후 6륜 전부 정확히 `+0.0` deg/s** (목표 −286) |
| C | `world.reset()` 전에 목표 설정 | 변화 없음 |
| D | 휠 `maxForce` ×100 | **출력 숫자가 한 글자도 안 변함** |
| F | `apply_action()` 으로 지령 | 바퀴는 돎. 단 **−100 ~ +250 deg/s 로 서로 반대 방향**, 로봇은 제자리 고정 |

여기서 확인된 원인은 세 가지다.

#### 3-1) 구동 부호가 반대다

`TARGET_SPEED_MPS = +0.05` 를 주면 로봇이 **−X**(곡관 반대 방향)로 간다.

암 링크가 `rx(-phi)` 로 배치되어 휠 회전축이 원주 접선의 **음(−)** 방향이
된다. 6륜 전부 일관되게 같은 방향이므로 부호 하나만 뒤집으면 해결된다.
실험 A 에서 부호만 반전하자 진행 방향이 실제로 바뀌었다.

#### 3-2) `set_joint_positions` 가 예압을 지운다

`pipe/curve_demo.py:196` 부근

```python
start = art.get_joint_positions()
for k in arm_idx:
    start[k] = 0.0
art.set_joint_positions(start)      # ← 여기서 런타임 드라이브 타깃이 날아간다

for k in wheel_idx:                  # 바퀴만 재설정하고 암은 빠뜨렸다
    ...GetTargetVelocityAttr().Set(SPIN_DEG_S)
```

주의할 점은 **USD 속성값은 멀쩡하다**는 것이다. 호출 전후 모두 19.402° 로
읽힌다. 지워지는 것은 PhysX 런타임 타깃이다. 그래서 파일을 읽는 것만으로는
발견되지 않고, 같은 값을 **다시 `Set` 해주면 복구된다.**

실험 B 에서 재설정하자 암이 −0.02° → +13.44° 로 벌어지고 로봇이 관 중앙에
정확히 안착했다(z −0.1mm, 관절 −0.48°).

이 함정은 `HANDOFF.md` 의 "알려진 함정" 표에 작성자 본인이 적어 둔 항목인데,
정작 `curve_demo.py` 가 바퀴만 지키고 암을 빠뜨렸다.

#### 3-3) USD 드라이브 속성으로는 바퀴에 지령이 전달되지 않는다

실험 D 가 결정적이다. 휠 `maxForce` 를 **100배**로 올렸는데 출력 숫자가
완전히 동일했다. 드라이브가 아예 작용하지 않는다는 뜻이다.
실험 C 처럼 `world.reset()` **전에** 걸어도 마찬가지였다.

`apply_action(ArticulationAction(joint_velocities=...))` 으로 바꾸자
비로소 바퀴가 돌았다(실험 F). 다만 그 상태에서도 6륜이 −100 ~ +250 deg/s 로
**서로 반대 방향으로 돌며** 로봇은 x = −542.6mm 에 고정되었다.
이 지점부터는 구현팀의 판단이 필요하다.

- 상태 : **미해결.** 원인만 특정했고 원본은 수정하지 않았다

---

### 결함 4 — 빈 공간 `inf` 가 판정 노드의 단위 변환을 깨뜨린다 [치명, 미해결]

수정된 ① 이 내놓은 실측값은 **빈 공간 = `inf`** 였다(중앙 576,000 픽셀 전부).

```
① 빈 공간   픽셀 576,000   inf=576,000   유효=0   → invalid = ~np.isfinite(depth)
```

설정 자체는 바꿀 것이 없다. `pipe_condition.yaml` 의 현재
`invalid_mode: "empty_is_zero"` 가 `~isfinite | <=0` 으로 판정하므로 `inf` 를
정확히 잡는다(이름만 오해를 부를 뿐 동작은 맞다).

**문제는 다른 곳에서 터진다.** `condition/node.py:152`

```python
img = np.asarray(img, dtype=np.float64)
if img.dtype != np.float64 or np.nanmax(img) > 100.0:
    img = img / 1000.0        # mm → m 자동 변환
```

`np.nanmax` 는 NaN 만 무시하고 `inf` 는 그대로 돌려준다. 실측 확인:

```
nanmax(inf 포함) = inf  →  > 100.0 == True  →  전 화소를 1000 으로 나눈다
0.05 m → 0.00005 m
```

**단절 픽셀이 한 개라도 들어오는 순간 Depth 전체가 1/1000 이 된다.**
그러면 `offset_mm` 도 1/1000 이 되어 무슨 일이 있어도 `NORMAL` 로 나온다.
에러도 안 나고 화면상 이상도 없다.

- 영향 : 어긋남 판정이 상시 무력화된다. 가장 발견하기 어려운 형태의 결함
- 상태 : **미해결.** PC2 코드라 이번 실행 범위 밖이어서 손대지 않았다

---

### 결함 5 — 근접 벽면 경고는 오탐이다 [경미]

① 이 매번 아래 경고를 낸다.

```
② 근접 벽면 (25mm)
   전체 유효 35,142,560  최소 45.73 mm  35mm 미만 0
   [경고] 근접 벽면 Depth 가 없다. ... DISCONNECTED 오탐이 발생한다
```

`camera.yaml` 의 `near_wall_mm: 25.0` 을 기준으로 35mm 미만 픽셀을 세는데,
카메라는 관 중심축에 있고(`camera.yaml` front/rear 모두 `offset_mm: [0,0,0]`)
DN100 내반경은 50mm 다. **축 위 카메라에서 관벽은 원래 50mm 이므로
35mm 미만 픽셀은 기하학적으로 존재할 수 없다.**

설계의 "관 내벽까지 25mm" 는 **본체 표면**에서 관벽까지의 거리다
(본체 폭 50mm → 반폭 25mm, 관벽 50mm). 카메라 기준이 아니다.

실측 최소 45.73mm 로 근접 측정은 정상 동작한다. near clip 0.005 도 충분하다.

- 영향 : 없음. 다만 매번 경고가 떠서 진짜 문제를 가린다
- 상태 : 미수정 (판정 기준을 바꿀지는 설계 결정 사항)

---

### 결함 6 — rclpy 전제가 이 환경과 맞지 않는다 [설계 판단 필요]

`camera/rig.py`, `robot/state_bridge.py`, `pipe/curve_demo.py` 세 파일이
`require_isaac(needs_rclpy=True)` 로 rclpy 를 요구한다. 이 장비의 Isaac Sim
Python 3.11.13 에는 rclpy 가 없어 세 파일 모두 실행 자체가 거부된다.

HANDOFF 는 `IsaacSim-ros_workspaces` 를 3.11 로 빌드하라고 안내하는데,
확인해 보니 두 가지가 어긋난다.

- **main 브랜치는 Python 3.12 만 만든다.** dockerfile 3종이 전부
  `..._python_312_minimal.dockerfile` 이다. 3.11 과 ABI 가 달라 `import rclpy`
  가 실패한다. **`IsaacSim-5.1.0` 태그에 3.11 dockerfile 이 있다** —
  빌드한다면 이쪽이어야 한다
- 빌드에 docker 가 필수인데 이 장비에는 설치되어 있지 않다

추가로 `pipe/curve_demo.py` 는 `import rclpy` 가 `if CAMERAS:` 블록 안에만
있는데 가드는 **무조건** 걸려 있었다. 카메라 없이 주행만 볼 때도 ROS 빌드를
요구하는 셈이라 과도한 제약이다.

- 상태 : `curve_demo.py` 의 가드만 완화함 (3-3 참조). 나머지는 미결

---

## 3. 수정 내역

원본 파일에 남긴 변경은 **3건**이다. 주행 실험(A~F)은 전부 임시 사본으로
돌렸고 원본은 건드리지 않았다. 임시 파일은 모두 삭제했다.

### 3-1) `welder/articulate.py:54` — 토치 카테고리 누락

**원인** : `_CATEGORY` 에 토치 3종이 없어 `part_path("torch_ring")` 이
`KeyError` 를 낸다.

**내용**

```python
 _CATEGORY = {
     "body_rear": "robot", "body_front": "robot", "bellows": "robot",
     "arm": "robot", "wheel": "robot",
     "camera_housing": "camera",
     "pipe_straight": "pipe", "pipe_elbow_sr": "pipe",
+    "torch_ring": "welder", "torch_rod": "welder", "torch_tip": "welder",
 }
```

**이유** : STL 은 `welder/meshes/` 에 이미 정상적으로 존재한다. 경로를 찾는
표에만 등록이 빠진 것이라 표를 채우는 것이 최소 수정이다. 다른 방식(경로
하드코딩 등)은 단일 출처 구조를 깨뜨린다.

**결과** : 통과.

```
  링크 17 (기대 17)   DOF 15 (기대 15)
    관절 1 / 서스펜션 6 / 바퀴 6 / 토치 2
  토치  링 x 63mm, 안지름 26 / 수납 40mm / 도달 48mm (관벽 50)
    J1 목표   +90.0° → 실제   +90.0°  오차  0.00°  OK
    J1 목표   -90.0° → 실제   -90.0°  오차  0.00°  OK
    J2 목표     8.0mm → 실제    8.0mm  오차  0.00mm  OK
저장 완료: son/welder/welder_2seg.usd
```

---

### 3-2) `camera/depth_probe.py:77` — 진단 씬을 열린 관으로 교체

**원인** : `UsdGeom.Cylinder` 는 뚜껑이 달린 속 찬 원기둥이라 관 내부에서
보면 화면 중앙에 끝면이 잡힌다. 빈 공간이 만들어지지 않는다.

**내용** : `UsdGeom.Cylinder` 대신 실제 배관 메시(`pipe/meshes/pipe_straight.stl`,
양끝이 열린 600mm 직관)를 읽어 배치한다. 관의 앞끝을 `HOLE_X` 에 맞춰
그 너머가 진짜로 비게 만들었다. STL 로더(`load_stl`)와 `import struct` 를
같이 추가했다 — 다른 스크립트들이 쓰는 것과 동일한 구현이다.

**이유** : 원래 의도("카메라 앞 HOLE_X 까지만 관이 있고 그 너머는 빈 공간")를
그대로 살리면서, 프로젝트가 이미 쓰고 있는 실제 자산으로 바꾸는 것이 가장
작은 변경이다. Isaac Sim 에는 불리언 컷이 없으므로 관을 잘라 개구부를 만드는
접근 자체는 유지했다.

**결과** : 진단이 성립한다.

```
① 빈 공간   픽셀 576,000   0=0  NaN=0  inf=576,000  유효=0
   → invalid_mode: empty_is_zero
     판정식: invalid = ~np.isfinite(depth)
```

`pipe_condition.yaml` 의 현재 값이 이미 `empty_is_zero` 이므로 **설정 변경은
불필요**하다. 대신 이 결과로 결함 4(판정 노드의 `inf` 단위 변환)를 찾아냈다.

---

### 3-3) `pipe/curve_demo.py:27` — rclpy 가드 완화

**원인** : `import rclpy` 는 `if CAMERAS:` 안에만 있는데 가드는 무조건
`needs_rclpy=True` 였다. 카메라를 쓰지 않는 주행 시험까지 막혔다.

**내용**

```python
-require_isaac(__file__, needs_rclpy=True)
+# rclpy 는 --cameras 일 때만 import 한다(아래 CAMERAS 분기). 카메라 없이
+# 주행만 볼 때까지 ROS 빌드를 요구할 이유가 없다.
+require_isaac(__file__, needs_rclpy="--cameras" in sys.argv)
```

**이유** : 가드가 실제 import 조건과 일치하지 않았다. 조건을 실제와 맞추는
것이 원저자 의도를 보존하는 최소 수정이다. `--cameras` 를 주면 종전대로
rclpy 를 요구한다.

**결과** : ROS 빌드 없이 곡관 주행 시험을 실행할 수 있게 되었고,
그 덕분에 결함 3(주행 구동부 미동작)을 발견했다.

---

### 생성된 산출물

```
son/robot/robot_2seg.usd            정찰기 물리 (링크 14 / DOF 13)
son/welder/welder_2seg.usd          수리기 물리 (링크 17 / DOF 15)
son/camera/camera_probe_result.json Depth 진단 결과 (invalid_mode)
```

---

## 4. 개선이 필요한 부분

### 4-1) 최우선 — 주행 구동부 재작성

결함 3 이 해결되지 않으면 그 뒤의 모든 시나리오(정찰·수리·복귀)가 성립하지
않는다. 세 가지를 함께 손봐야 한다.

1. 구동 부호를 반전한다 (또는 휠 조인트 축 부호를 바꾼다)
2. `set_joint_positions` 뒤에 **암 드라이브 타깃도** 재설정한다
3. USD 드라이브 속성 직접 쓰기를 **`apply_action`** 으로 교체한다.
   `maxForce` ×100 이 무영향이었다는 것이 이 경로가 죽어 있다는 증거다
4. 그 위에서 6륜이 같은 방향으로 도는지 다시 확인한다.
   실험 F 에서 −100 ~ +250 deg/s 로 갈렸다

권장 : 곡관 이전에 **직관 600mm 직진**만 먼저 통과시키고, 그 다음 곡관으로
넘어가는 것이 원인 분리에 유리하다.

### 4-2) 검증 씬을 자유공간이 아니라 관 안에서 잡을 것

`articulate.py` 는 자유공간에서 검증하므로 "암 6개가 상한까지 벌어짐 / 바퀴
6개 회전"이 전부 통과한다. 그러나 관에 넣으면 예압도 구동도 죽는다.
**자유공간 통과가 관 내부 동작을 보장하지 않는다.** 조립 검증 단계에
"직관에 넣고 N mm 전진" 항목을 추가할 것을 권한다.

### 4-3) `inf` 안전성 점검 (결함 4)

`condition/node.py` 의 단위 자동 변환을 `inf` 안전하게 고쳐야 한다.

```python
finite = img[np.isfinite(img)]
if finite.size and finite.max() > 100.0:
    img = img / 1000.0
```

같은 패턴이 다른 곳에도 있는지 훑어볼 것. 빈 공간이 `inf` 로 온다는 사실이
이번에 확정되었으므로, `nanmax` / `max` / 정규화 계산 전반이 점검 대상이다.

### 4-4) 근접 벽면 판정 기준 재정의 (결함 5)

`camera.yaml` 의 `near_wall_mm` 이 카메라 기준인지 본체 표면 기준인지
명확히 하고, 축 위 카메라 기준이면 50mm 로 바꿀 것. 지금은 절대 만족할 수
없는 조건이라 경고가 상시 발생한다.

### 4-5) rclpy 의존 축소

`camera/rig.py` 와 `robot/state_bridge.py` 는 본체(카메라 프림 생성·annotator·
프레임 획득 / IMU·엔코더 읽기)가 ROS 와 무관한데 `import rclpy` 가 모듈
최상단에 있어 통째로 막힌다. **지연 import 로 바꾸면 ROS 빌드 없이도 핵심
동작을 검증할 수 있다.** `rig.py --save` 경로는 애초에 발행을 하지 않는다.

ROS 발행 방식 자체도 결정이 필요하다.

- (가) `IsaacSim-5.1.0` 태그로 ROS 2 를 Python 3.11 빌드 (docker 필요)
- (나) OmniGraph 브릿지 노드로 전환 (빌드 불필요, 발행부 재작성 필요)

본 프로젝트 규약은 (나)를 채택하고 있으므로 방침 정리가 선행되어야 한다.

### 4-6) 문서 최신화

`robot/README.md`, `condition/README.md` 와 여러 스크립트의 docstring 이
재구성 이전 파일명을 쓰고 있다.

| 문서 기재 | 실제 |
|---|---|
| `build_robot_parts.py` | `tools/build_parts.py` |
| `robot_articulated.py` | `robot/articulate.py` |
| `pipe_curve_demo.py` | `pipe/curve_demo.py` |
| `pipe_condition_node.py` | `condition/node.py` |
| `parts/` | `robot/meshes/`, `pipe/meshes/` 등 |
| `../test_code/pipe_condition/` | `test_code/condition/` |

실행할 때는 README 가 아니라 `HANDOFF.md` 를 따라야 한다.

### 4-7) 그 밖에

- **수리기 총질량 563g** (설계 500g). 토치 63g 이 그대로 얹혀 있다.
  유체 계산(견인력 6.5N / 4.2N)이 500g 기준이므로 재확인 필요
- `test_code/condition/scenes/` 에 `.npy` 가 3개뿐이라 `test_detector.py` 는
  `make_scenes.py` 를 먼저 돌려야 한다 (HANDOFF 에 명시되어 있음)
- 곡관 각도 규약이 직관과 반대다 (`crack_inject.py:361`). 곡관은
  `site_deg=0` 이 **안쪽**(r=50), 180 이 바깥쪽이다. 균열 위치 지정 시 혼동 주의

---

## 5. HANDOFF "처음 돌릴 때 알려줬으면 하는 것" 에 대한 회신

| 질문 | 답 |
|---|---|
| 1. IMU 프레임 키 | **미확인** — `state_bridge.py` 가 rclpy 때문에 실행되지 않음 |
| 2. 롤 부호 | **미확인** — 동일 사유 |
| 3. `invalid_mode` | **`inf`** (빈 공간). 현재 설정 `empty_is_zero` 가 맞게 동작함. 단 결함 4 참조 |
| 4. `set_opencv_fisheye_properties` | **동작함.** 경고 없이 어안 f=523.8px / HFOV 140° 로 진단 완료 |
