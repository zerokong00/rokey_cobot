# legacy — son 1세대 플랫폼 (보관본)

**여기 있는 것은 삭제·수정 대상이 아니다.** 관내 주행이 실제로 확인된 조립이라
원본으로 보존한다. 2026-08-04 플랫폼을 벨로우즈 12륜으로 교체하면서 현역 경로에서
빠졌을 뿐이다.

## 무엇이 들어 있나

| 파일 | 원래 위치 |
|---|---|
| `assemble.py` | `robot/assemble.py` — 조립 구현 한 벌(정찰기·수리기 공용) |
| `articulate.py` | `robot/articulate.py` — 자유공간 자체검증 |
| `welder_articulate.py` | `welder/articulate.py` — 수리기 자체검증 |
| `curve_demo.py` | `pipe/curve_demo.py` — SR 곡관 주행 시연 |
| `robot_README.md` | `robot/README.md` |
| `meshes/` | son 로봇 5종(`body_*`,`bellows`,`arm`,`wheel`) + 코스 배관 4종(`pipe_straight`,`pipe_elbow_sr`,`*_crack`) |

## 옮기면서 고친 것 — 자산 경로 앵커뿐

물리·치수 상수는 한 글자도 안 건드렸다. 바뀐 것은 이것뿐이다.

- `part_path()` 에서 `robot` / `pipe` 카테고리를 `legacy/meshes/` 로 돌린다.
  `welder` / `camera` 는 현역이라 son 루트 아래를 그대로 본다.
- `import assemble` 의 `sys.path` 가 `son/robot` → `son/legacy` 를 가리킨다.

`SON = HERE.parent` 는 그대로 `src/son` 을 가리킨다(이 디렉터리를 son 바로 아래
둔 이유가 그것이다) — `spec/parts_meta.json`·`pyver.py` 는 계속 정상 해석된다.

⚠ `tools/build_parts.py` 는 여전히 **`robot/meshes/`·`pipe/meshes/` 로** STL 을
쓴다. 보관본 메시를 다시 만들 일이 있으면 생성 후 `legacy/meshes/` 로 옮길 것.

## 왜 교체했나 (실측 근거, 2026-08-04)

같은 배관·같은 물리 조건(1/240)에서:

| 로봇 | LR 곡관 R=150 |
|---|---|
| 벨로우즈 12륜 | ✅ 완주 |
| son 6륜 (이 보관본) | ❌ 수평 2% / 수직 55% 에서 정지 |

son 조립은 **더 완만한 LR 곡관에서도 못 넘었다.** 곡관 반경만 단일 변수로
바꾼 대조(SR R=100 → LR R=150)에서 진입 깊이가 26%→55% 로 늘었을 뿐 통과는
못 했다. 정지 시점의 양상이 일관된다 — 암이 신장 상한(13.78°)에 붙고 중앙
관절은 여유가 남는다(-32.8° / 한계 ±55). **관절 굽힘이 아니라 서스펜션
스트로크 총량이 부족**하다.

직관 주행 자체는 정상이었다(슬립 0.94~0.96, 밀착 6/6, 예압 87~113%).

## 살아 있는 성과 — 여기서 나온 것들

교체 후에도 유효한 실측·판단이 많다. 버리지 말 것.

- **휠 충돌체는 실린더 프리미티브**여야 한다(메시 convexHull 은 스틱슬립 채터로
  주행 0mm). 단 **배관·로봇 조합에 따라 다르다** — 벨로우즈 12륜 + LR 배관에서는
  원본 convexHull 이 정상 주행하고 실린더로 바꾸면 오히려 죽었다.
- `contactOffset 0.0005`(CPU 전용), 이미지 QoS `BEST_EFFORT`,
  `TRANSIENT_LOCAL` 정답 토픽 등 v3 §13 파라미터
- 토치 3겹 검증·복귀 감사, 관 상태 판정(Depth 무효 비율), 시각 오도메트리,
  추측 항법 — 전부 현역 디렉터리에 그대로 있다
