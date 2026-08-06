# 배관 점검 로봇 — Isaac Sim 작업 기록

배관 내부를 주행하며 결함을 카메라로 탐지하는 로봇의 Isaac Sim 구현.

🔑 **현역은 `zigzag/` 다 (2026-08-06).** 아래 「로봇 구성」부터의 기록은
`graphic_file/` 의 3다리 9륜 로봇에 대한 것으로, 2026-08-01 에 끝난 이전
단계다. 지금 굴리는 것은 지그재그 5m 코스 + 벨로우즈 12륜 로봇이고 그쪽
기록은 `zigzag/README.md` 에 따로 있다.

## 폴더 구조

```
isaacpjt/dongmin/
├── README.md                이 파일 (작업 기록)
├── zigzag/                  ★ 현역 — 지그재그 R100 점검·수리 시연
│   ├── README.md                  실측·주의사항 (여기부터 읽는다)
│   ├── TODO_WATER.md              물 채우기 + 누수 가시화 (진행 중)
│   ├── course.py                  코스 기하 단일 출처
│   ├── zigzag_demo.py             주 시연 (주행 + 용접)
│   ├── water_demo.py              로봇 없이 물만
│   ├── meshes/                    [산출물] build_defect_w56.py 가 생성
│   └── test_code/                 코스 기하 대조 시험
├── M0609/                   이전 실습 사본 (참고용 패턴 코드)
├── basic/                   최소 standalone 예제
└── graphic_file/            이전 단계 (3다리 9륜) 형상·스크립트
    ├── pipe/                배관 형상 (STL 원본 + USD 변환본)
    │   ├── pipe.stl/.usd            직관 (내경 100mm 스케일로 사용)
    │   └── pipe_levelup.stl/.usd    가변관 (내경 190 → 테이퍼 → 95mm)
    └── robot/               로봇 형상 + 스크립트 + 산출물
        ├── body/leg/wheel.stl/.usd  파츠 (CATIA 원본 STL, 수동 USD 변환)
        ├── robot_img.png            조립 형상 참고 이미지
        ├── robot_assembled.usd      [산출물] robot_articulated.py 가 생성
        └── inspect_out/             [산출물] pipe_inspect_demo.py 결과
```

`[산출물]` 표시 파일은 스크립트 재실행으로 재생성 가능 — 저장소에 올릴 필요 없음.

🚨 `zigzag/` 가 쓰는 배관·로봇 usda 두 개는 **저장소 루트**에 있다
(`test_pipe_zigzag_long_R100.usda`, `robot_from_bot_welder_art_v2.usda`).
`scene.py` 가 절대경로로 읽으므로 옮기면 깨진다.

## 로봇 구성 (2026-08-01 확정)

- 링크 13개: body 1 + leg 3(120° 간격) + wheel 9(다리당 3개)
- 조인트 12개: prismatic 3(다리 반경방향, 스프링 역할) + revolute 9(바퀴 구동)
- 다리 스트로크 90~101.3mm → 파지 가능 내경 약 190~210mm 스케일 환산 전 기준
  (배관 데모는 STL을 스케일해서 내경을 맞춤)
- 파츠 배치값은 STL 정점을 직접 파싱한 실측값 (CATIA 어셈블리 좌표계, mm)

구조상 핵심 (isaacsim_kb/NOTES/04 §5):
- Joint 의 Body0 는 articulation 트리의 **부모**여야 함
- rigid body 중첩 금지 → 전 링크를 `/World/Robot` 바로 아래 형제로 평탄화
- scale 은 rigid body 변환 체인이 아니라 **메시 리프에만** 적용

## 스크립트 (검증 순서대로)

모두 `PYTHONUNBUFFERED=1 isaac_python <스크립트>` 로 실행. `--headless` 는 숫자 검증만.

| 순서 | 스크립트 | 무엇을 검증하나 |
|---|---|---|
| 1 | `assemble_robot.py` | 파츠 배치 (형상만, 물리 없음) |
| 2 | `robot_articulated.py` | 조인트 구성 → `robot_assembled.usd` 저장 |
| 3 | `drop_demo.py` | 평면 낙하·착지 (articulation 이 충격에 터지지 않는가) |
| 4 | `spring_demo.py` | 다리 스프링 동작 (관경 300↔342mm 왕복 재현, 중력 OFF) |
| 5 | `pipe_drive_demo.py` | 직관 내부 주행, **슬립률**(실주행/이론주행) 측정 |
| 6 | `pipe_levelup_demo.py` | 가변관에서 다리가 내경을 따라 신축하는가 |
| 7 | `pipe_inspect_demo.py` | 결함(크랙 2, 구멍 1) 탐지 → `inspect_out/` 기록 |
| 8 | `pipe_chain_ros_demo.py` | 배관 체인 주행 + `/rgb` `/depth` ROS2 발행 (**GUI 필수**) |

## 실측으로 확인된 사실 (2026-08-01)

추측이 아니라 시뮬레이션을 돌려 재현한 결과다.

1. **가변관 설계 한계**: 바닥에 얹힌 채 테이퍼를 오르면 몸이 경사각(5.7°)만큼
   들려서 95mm 구멍(여유 0.3mm)에 기하학적으로 진입 불가
   (피치 +7.3°로 입구에 쐐기 고정, X=-24mm 정지 실측).
   → 이 구조(강체 레일)는 **파지 상태로만** 급격한 축관 통과 가능.
   `pipe_levelup_demo.py` 는 그래서 파지 유지 구간만 왕복한다.
2. **배관 체인 주행 방향**: 내경 95→100mm 경계는 다리가 "바깥으로" 신장하는
   방향이라 턱에 안 걸림. 반대 방향(100→95)은 2.5mm 단차를 기어올라야 해서 위험.
3. **결함 탐지 임계값**: 주행 초반 프레임의 실측 통계(중앙값)로 산출 (C8).
   fallback 없음 — 탐지 실패는 FAIL 로 드러난다.
4. **ROS2 카메라 발행**: replicator writer(`ROS2PublishImage`)를 render product 에
   attach 하는 Python 방식만 동작 (KB NOTES/05 §5).
   - `ROS2CameraHelper`(OmniGraph) 방식은 이 구성에서 **빈 프레임만 발행됨**
   - `/camera_info` writer 는 이미지 파이프라인을 깨서 **발행하지 않음**.
     내부 파라미터(fx=763.5 등)는 시뮬 시작 로그에 출력만 한다.
5. **탐지 레코드 포맷** (C7): (프레임 번호, 시뮬 시각, 로봇 X 오도메트리, blob 정보)
   + 프레임 PNG. 정답은 `ground_truth.csv`, 결과는 `detections.csv`.

## 연동 대상

PC B 측 수신·검증 노드는 `src/dongmin/pipe_inspect/` (ROS2 패키지, 해당 README 참조).

## 다음 작업 후보 (2026-08-01 시점 — 아래 둘은 `zigzag/` 에서 이미 답이 나왔다)

- ~~곡관/분기관 형상 추가 및 주행 검증~~ → `zigzag/` 가 R100 굽힘 7개로 한다
- ~~결함 표현 확대 (텍스처 기반 크랙, 누수 마커)~~ → `zigzag/` 는 관벽을
  실제로 뚫는 방식(`defect.py`)으로 갔고, 누수는 `TODO_WATER.md` 진행 중
- PC B 쪽 결함 판정 노드 (`6_pick_place_color.py` 의 OBSERVE 패턴 재사용)
- 탐지 성능 정량 평가 (`detections.csv` vs `ground_truth.csv` 매칭)

🚨 위 본문의 C1~C9 / E1~E5 는 `CLAUDE.md` 의 항목 번호인데 **그 파일은
저장소에 없다.** 번호로 되짚을 수 없으니 본문 설명만 믿을 것.
