# 단독 기능확인판 (STANDALONE)

**이 zip 은 Isaac Sim 안에서 기능이 도는지만 확인하는 판이다.**
PC1↔PC2 ROS 통신은 이 판으로 검증할 수 없다.

## 원본과 다른 점 — 딱 하나

`son/standalone/` 에 rclpy 대역이 들어 있고 `pyver.py` 가 그것을 sys.path 에
붙인다. **애플리케이션 코드는 원본과 한 글자도 다르지 않다.**

    발행 대신 → 값을 찍고 이미지를 저장한다 (son/out/)

**`build_ros.sh` 도, docker 도 필요 없다.** 여섯 스크립트가 전부 돈다.

## 실행

```bash
python3 pyver.py                       # 지금 인터프리터 확인

PYTHONUNBUFFERED=1 isaac_python camera/depth_probe.py      # ① GUI 필수
PYTHONUNBUFFERED=1 isaac_python robot/articulate.py        # ②
PYTHONUNBUFFERED=1 isaac_python welder/articulate.py       # ③
PYTHONUNBUFFERED=1 isaac_python camera/rig.py --save       # ④ GUI 필수
PYTHONUNBUFFERED=1 isaac_python robot/state_bridge.py      # ⑤
PYTHONUNBUFFERED=1 isaac_python pipe/curve_demo.py         # ⑥ GUI 필수
```

## Depth 는 그림이 아니라 숫자다

`④⑥` 을 돌리면 이런 줄이 나온다. **이 숫자가 물리적으로 말이 되는지 봐야
한다.** 토픽이 나왔다는 것만으로는 정상 동작 확인이 아니다.

```
front/depth  min 0.0240  max 4.8700  중앙값 0.0610 m  무효 0.3%
```

| 나온 값 | 뜻 |
|---|---|
| `min 0.024` | 정상 — 관벽이 잡힌다 |
| 유효 픽셀 0 | annotator 미부착이거나 headless 로 띄웠다 |
| `min 25.0` | 단위가 mm 다 (1000배 틀림) |
| `max 5.0` 만 가득 | 아무것도 안 맞았다 |
| 무효 30% | 판정기가 못 쓴다 (허용 2%) |

관 내반경이 50mm 이므로 **가장 가까운 관벽은 0.045~0.050 근처**여야 한다.

## 오프라인 시험 — Isaac Sim 없이도 돈다

```bash
cd test_code/condition && python3 test_detector.py
cd test_code/camera    && python3 test_probe_scene.py
cd test_code/driver    && python3 test_traction.py
```

`test_code/` 전부 시스템 python3(3.10) 로 돈다. `test_code/README.md` 참조.

## 통신까지 확인하려면

원본 판이 필요하다. `HANDOFF.md` 의 rclpy 절을 볼 것 —
`IsaacSim-5.1.0` **태그**로 빌드해야 한다(main 은 Python 3.12 만 만든다).
