# pipe_robot — 2단 1관절 정찰기 (설계확정본 v3 구현)

설계확정본 v3의 2단 1관절 형상을 Isaac Sim articulation으로 만드는 자산·스크립트.
기존 `body_new1.stl`(단일 강체 beta)과 별개이며 그쪽은 건드리지 않음.

**Isaac Sim 담당자에게 넘길 때는 `HANDOFF.md` 를 먼저 볼 것.**

## 파일

| 파일 | 역할 | 실행 환경 |
|---|---|---|
| `build_robot_parts.py` | 링크별 STL + 배관 STL 생성, `parts_meta.json` 출력 | 일반 python3 (trimesh) |
| `robot_articulated.py` | articulation 구성, `robot_2seg.usd` 저장 | **Isaac Sim** |
| `pipe_curve_demo.py` | SR 곡관 통과 주행 시험 | **Isaac Sim** |
| `camera_depth_probe.py` | **빈 공간 Depth 반환값 + 근접 벽면 진단** | **Isaac Sim** |
| `camera_rig.py` | 카메라 2대·조명 부착, `--save` 로 자산 저장 / 없으면 ROS 발행 | **Isaac Sim** |
| `config/camera.yaml` | 카메라·조명 파라미터 전부 | — |

오프라인 검증(강제 자세·합성 입력)은 전부 **`../test_code/pipe_robot/`** 에 있다.

## 실행 순서

```bash
python3 build_robot_parts.py          # parts/ 생성
python3 ../test_code/pipe_robot/preview_assembly.py   # 오프라인 검증

PYTHONUNBUFFERED=1 isaac_python camera_depth_probe.py      # ① 진단 먼저 (GUI)
PYTHONUNBUFFERED=1 isaac_python robot_articulated.py --headless
PYTHONUNBUFFERED=1 isaac_python camera_rig.py --save       # ② 카메라 붙여 저장 (GUI)
PYTHONUNBUFFERED=1 isaac_python pipe_curve_demo.py --cameras
```

### 자산 관계

```
build_robot_parts.py  →  parts/*.stl
robot_articulated.py  →  robot_2seg.usd        (물리)
camera_rig.py --save  →  robot_2seg_cam.usd    (물리 + 카메라)  ← 최종 자산
pipe_curve_demo.py    →  그것을 불러 주행 (--cameras 면 영상도 발행)
```

**카메라는 파일이 아니다.** USD 의 카메라는 조명처럼 프림 타입 하나이며
메시가 없다. `camera_rig.py` 가 실행 중에 링크의 자식으로 만들어 넣는다.
자식이므로 몸통이 꺾이면 카메라도 따라 꺾인다. STL 인 것은 외형 하우징뿐이고
콜라이더도 없다.

```
/World/Robot
  └ body_front                 ← 링크(강체)
      └ front_camera
          ├ housing/mesh       ← camera_housing.stl (시각 전용)
          ├ light_0, light_1
          └ sensor             ← 카메라 프림
```

**`--save` 가 담는 것과 안 담는 것** — 카메라·조명·하우징 프림은 USD 에
저장되지만 annotator 와 render product 는 런타임 자원이라 저장되지 않는다.
불러 쓰는 쪽에서 `camera_rig.attach_existing()` 을 호출해야 영상이 나온다
(`pipe_curve_demo.py --cameras` 가 그렇게 한다).

### parts/ 구성

```
parts/
├─ robot/    body_rear  body_front  bellows  arm  wheel
├─ camera/   camera_housing        ← 외형만. 광학과 무관, 콜라이더 없음
├─ pipe/     pipe_straight  pipe_elbow_sr
└─ parts_meta.json                 ← 치수·각도·한계값. 스크립트가 읽는다
```

`parts_meta.json` 은 빼면 안 된다. 암 기준각·스트로크 한계·세그먼트 간격이
전부 여기 있고 스크립트가 하드코딩 대신 이 파일을 읽는다.

`parts/**/*.stl` 은 `.gitignore` 대상이 아니므로 생성물이 커밋될 수 있음.
스크립트로 재생성되는 파일이라 커밋 불필요.

## 시각화 산출물

`../test_code/pipe_robot/` 에 있다 — `curve_pass.gif`, `preview_straight.png`,
`preview_bent.png`. **전부 강제 자세로 만든 기하 해석이지 물리 시뮬이 아니다.**
자세한 내용과 수치 근거는 `../test_code/README.md`.

## 카메라 — RealSense 폐기, 어안 Camera prim

**RealSense 는 스펙상 적용 불가다.**

| 항목 | D455 | 본 프로젝트 |
|---|---|---|
| 최소 측정 거리 | **52cm** | **25mm** (20배 차이) |
| 본체 치수 | 124×25×29mm | 세그먼트 62mm 내부 |

Isaac Sim 의 다른 depth 카메라 에셋(D555, Orbbec, Leopard, Sensing)도 전부
자율주행·서비스 로봇용이라 같은 문제가 있다. **이 스케일에 맞는 상용 에셋은 없다.**

실제 배관 점검 장비도 스테레오가 아니라 **광각 보어스코프**를 쓴다. 그래서
일반 `Camera` prim + 어안 투영으로 전환했고, 이쪽이 현실에도 더 부합한다.
**외형만 RealSense 형태**의 하우징(`parts/camera_housing.stl`, 44×15×13mm,
렌즈 3개)을 붙인다. 시각 전용이며 콜라이더 없음.

| 항목 | 값 | 근거 |
|---|---|---|
| 투영 | `fisheyePolynomial`, HFOV 140° | 관 내부는 원통, 화각이 넓을수록 관벽을 많이 봄 |
| 클리핑 | 0.005 ~ 5.0 m | 관벽 25mm 보다 near 가 작아야 함 |
| 해상도 | 1280×720 | YOLO 입력 640 대비 과하지 않게 |
| 주기 | 10 Hz | 주행 0.1 m/s 에서 프레임당 10mm |
| 어노테이터 | `distance_to_camera` | 관 내부는 방사형 — 광축 투영 거리가 아니라 실제 거리 |
| 조명 | SphereLight ×2/카메라, 3e7 | 관 내부는 로봇 조명이 유일한 광원 |

### 반드시 먼저 실행

```bash
PYTHONUNBUFFERED=1 isaac_python camera_depth_probe.py    # GUI 필수
```

빈 공간의 `distance_to_camera` 가 0 / inf / NaN / 최대값 중 무엇인지 확인한다.
**이 확인 없이 구현하면 관 단절 판정이 전혀 작동하지 않는다.** 결과는
`camera_probe_result.json` 으로 떨어지며, 그 `invalid_mode` 를
`pipe_condition/config/pipe_condition.yaml` 에 넣는다.

근접 벽면(25mm) 측정 가능 여부도 같이 본다. 안 나오면 무효 픽셀 비율이 상시
높아져 DISCONNECTED 오탐이 발생한다.

### CameraInfo 주의

`ROS2PublishCameraInfo` writer 를 rgb/depth 와 **같은 render product 에 붙이면
파이프라인이 깨진다**(팀 실측). 그래서 `camera_rig.py` 는 CameraInfo 를 발행하지
않고 내부 파라미터를 `camera_intrinsics.json` 으로 떨어뜨린다. 별도 노드가 그걸
읽어 발행해야 한다.

어안이라 `K` 는 엄밀한 핀홀 행렬이 아니다. **`K[0]` 에 등거리 어안의 `f`
(r = f·θ)를 싣는 규약**을 쓴다. 판정기가 그 값으로 입사각을 역산한다.

## 구조

- 링크 14개 — body 2 + arm 6 + wheel 6
- 조인트 13개 — 관절 revolute 1 + 서스펜션 revolute 6 + 바퀴 revolute 6
- 벨로우즈는 `body_rear`의 시각 전용 자식. 콜라이더 없음, 굽힘 시 변형 안 함
- 축 규약 — **로봇 길이 방향 = X**, 관절 회전축 = Y(피치), 원주 각도는 +Z에서 +Y 방향

## 원본 대비 변경점

기존 `협동3_로봇_디자인/build_robot_stl.py`에서 바뀐 것 3가지.

1. **단일 메시 → 링크별 파일.** `concatenate` 제거, 부품 단위 export
2. **굽힘·암 각도를 메시에서 제거.** 기준 자세(굽힘 0°, 암 기준각)로만 출력하고
   움직임은 조인트가 담당
3. **암 기준각 44.4° → 43.30°** (아래 참조)

## 검증 결과 (2026-08-03, 오프라인)

**형상은 통과.** 필요 관절각 35.79°(한계 ±55°), 본체·암 관벽 여유 8.01mm,
필요 서스펜션 스트로크 ±4.59mm(한계 ±6mm). 휠 접촉 반경 50.00mm.

설계 문서의 곡관 산출식(±16.93 / ±2.46mm)을 독립 재현해 소수점까지 일치 확인.
**단 실제 필요 스트로크는 ±2.46mm 가 아니라 ±4.59mm** — 암 스태거와 암이 뒤로
뻗은 배치 때문. 여유가 3.5mm 가 아니라 1.4mm 다.

측정 방법·전수 결과·분리도는 **`../test_code/README.md`** 참조.

## 발견된 문제

### 1. 암 기준각 44.4°는 휠 림이 관벽을 0.55mm 파고듦 — 수정함

폭 15mm 평면 휠은 중심면이 아니라 **양 모서리(y=±7.5)가 먼저 벽에 닿음.**
벽이 원주 방향으로 휘어 모서리 위치의 벽 반경이 √(50²−7.5²)=49.43mm이기 때문.
문서값 44.4°는 나이프 에지 접촉을 가정한 값.

접촉 조건에서 역산한 **43.30°**를 기본값으로 사용. 문서값 유지는
`--keep-doc-angle` 플래그.

### 2. 전장이 설계 150mm를 초과 (169mm)

암이 피벗에서 뒤로 29mm 뻗어 후방 휠이 본체 뒤 끝(−75mm)보다 뒤인 −83mm에 위치.
휠 폭까지 더하면 −93mm. 원본 생성기부터 있던 특성이며 이번에 만든 것 아님.
직관 주행·곡관 통과에는 영향 없으나 **카탈로그 전장은 150mm가 아니라 169mm.**

### 3. 관절 1자유도의 방향 제약

관절 축이 Y(피치) 하나뿐이라 **곡관이 로봇의 피치 평면에 있을 때만 통과 가능.**
다른 방향 곡관은 로봇이 롤로 자세를 맞춰야 하는데, 관절이 수동이라 그 롤이
자동으로 일어나는지 미검증. `pipe_curve_demo.py`가 곡관을 XZ 평면에 둔 이유.

설계 문서도 관절 엔코더 1개로 적고 있어 문서와는 일치. 실제 배관망 대응은
2자유도(유니버설)가 필요할 수 있음 — 팀 논의 필요.

## 미검증 사항

**Isaac Sim을 이 노트북에서 못 돌림**(RTX 3050 Ti 4GB, 최소 사양 미달).
따라서 아래는 코드만 작성되고 실행되지 않음.

- `robot_articulated.py` — 문법·배치 변환식·STL 로더는 오프라인 검증했으나
  **articulation 생성·스프링·마찰·솔버 거동 일체 미실행**
- `pipe_curve_demo.py` — **전부 미실행.** 주행 성공 여부 알 수 없음
- 오프라인 검증은 **기하만** 확인. 물리(접촉·마찰·구동토크)는 포함 안 됨

## 물리 파라미터 근거

| 값 | 근거 |
|---|---|
| 총질량 500g | 설계 건조 질량. body 190g×2 + arm 12g×6 + wheel 8g×6 |
| 암 예압 9N/휠 | 설계값. 토크로 환산 = 9N × 암 모멘트암 29.11mm |
| 암 스프링 K | 자유장을 상한 +6° 바깥에 두고 기준자세에서 예압 9N이 되도록 역산 |
| 관절 센터링 0.01 N·m/rad | 설계값. 액추에이터 없음(수동) |
| 관절 한계 ±55° | 설계값(기계식 스토퍼) |
| 바퀴 최대토크 | 마찰 한계 = μ0.7 × 9N × 반경 10mm = 63 mN·m |
| `contactOffset` 0.0005 | 팀 실측 확인값. 기본 0.02는 휠 반경 10mm 스케일에서 부양 거동 |
| `EnabledSelfCollisions` False | 암이 본체 소켓과 겹쳐 켜면 발산 |

## 구현 주의

- **각도 드라이브 단위.** USD 물리는 각도 드라이브의 stiffness/damping/target을
  **도(degree) 기준**으로 해석. N·m/rad로 설계한 값에 π/180을 곱해 넣음
- **`set_joint_positions`가 drive target을 덮어씀.** 호출 후 target 재설정 필요
  (`pipe_curve_demo.py`에서 암 초기자세 설정 시 해당)
- **STL을 USD로 변환하지 않고 `UsdGeom.Mesh`로 직접 저작.** 변환기 의존이 없고
  정점을 읽을 때 m 단위로 바꾸므로 변환 체인에 scale op이 생기지 않음
- **배관 콜라이더는 `approximation="none"`.** convex hull이면 관 안쪽이 막힘
- `preview_bent.png`의 점선은 **직관** 기준선이라 곡관 자세에서는 의미 없음

## 남은 작업

- Isaac Sim 있는 장비에서 `robot_articulated.py --headless` 실행 및 검증 통과 확인
- `pipe_curve_demo.py`로 곡관 통과 실증
- 관절 자유도 1 vs 2 팀 결정
- 전장 169mm를 설계 문서에 반영할지, 암 배치를 바꿔 150mm에 맞출지 결정
