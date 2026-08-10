# web/ — 관제 패널 웹 소스

`pipe_comm/web_panel` (FastAPI) 이 서빙하는 페이지다. ROS 파이썬 패키지와
수명·도구가 달라 워크스페이스 최상위로 분리해 둔다.

```
index.html            셸 — 사이드바 + 본문 틀 (__NS__ 는 서버가 치환)
panel.css             스타일 (다크. 관제실 화면이라 카메라 영상 대비가 산다)
app.js                라우터 + 웹소켓 + 공유 store + 지령 — ES 모듈
views/home.js         Home        대시보드 요약
views/handling.js     Robot Handling  조종석 — 지령 + 카메라 + 상태 + 사건 로그
views/camera.js       (사이드바에 없음) 카메라 한 칸 — `mountCam()` 을 조종석이 쓴다
views/events.js       (사이드바에 없음) 사건 로그 — `mountLog()` 을 조종석이 쓴다
views/map3d.js        3D Map      three.js 배관 맵
views/detect.js       Detect List 결함 리포트 표
three.module.min.js   three@0.160.1 벤더 사본
OrbitControls.js      three examples/jsm 사본 (importmap 으로 'three' 해석)
```

## 페이지와 주소

주소는 진짜다 — `<IP>:8080/handling` 을 직접 치거나 새로고침해도 그 페이지가
뜬다. 다만 **라우팅은 브라우저(History API)가 하고 서버는 어느 경로로 들어와도
같은 셸을 준다**. 웹소켓 하나를 페이지 전환 내내 공유하기 위해서다 — 서버
라우팅으로 가르면 전환마다 재접속이라 카메라가 끊기고 지난 결함·사건을 다시
받아야 한다.

🔑 경로 목록은 두 곳에 있다: `app.js` 의 `ROUTES` 와 `web_panel.py` 의 라우트
목록. **위험한 방향은 한쪽뿐이다** — `ROUTES` 에만 있고 서버에 없으면 그 주소는
새로고침에서만 404 가 나서 진단이 늦다. 페이지를 추가하면 반드시 둘 다 고칠 것.
반대로 서버에만 남은 주소(`/camera`, `/events` — 2026-08-08 에 조종석으로
합치면서 사이드바에서 뺐다)는 무해하다: 셸이 뜨고 브라우저가 `/home` 으로
돌려보낸다. 옛 북마크가 404 대신 홈으로 가므로 일부러 남겼다.

## 새 페이지 추가

`views/foo.js` 에 `export function mount(el)` 을 두고(정리할 게 있으면
정리 함수를 돌려준다), `app.js` 의 `ROUTES` 에 한 줄, `web_panel.py` 의
경로 목록에 한 줄 넣는다.

```js
import {store, bus} from '/static/app.js';
export function mount(el) {
  el.innerHTML = `<div class="card wide">…</div>`;
  const draw = () => { /* store 를 읽어 그린다 */ };
  draw();
  return bus.on('state', draw);      // 돌려준 함수가 unmount 때 불린다
}
```

**화면에 없는 페이지도 상태는 계속 쌓인다** — `store` 가 웹소켓을 받아 두고,
페이지는 mount 때 `store` 를 통째로 읽어 그린 뒤 `bus` 를 구독한다. 그래서
Event Log 를 나갔다 와도 로그가 비지 않는다.

## 왜 CDN 을 안 쓰나

현장망은 인터넷이 없다. three.js 갱신은 파일 교체로 한다 — 버전을 올리면
`index.html` 주석의 버전 표기도 같이 고칠 것.

## 고치고 확인하는 법

`web_panel` 은 **이 디렉터리를 설치본보다 먼저** 잡는다(`find_web()`).
즉 여기 파일을 고치면 **colcon build 없이 브라우저 새로고침만** 하면 된다.
(`.py` 를 고쳤을 때만 재빌드가 필요하다 — 이 패키지는 --symlink-install
로도 파일이 복사된다.)

```bash
ros2 run pipe_comm web_panel --ros-args -p ns:=robot -p port:=8080
```

`colcon build` 는 이 디렉터리를 `share/pipe_comm/web/` 으로 복사한다
(`src/dongmin/pipe_comm/setup.py` 의 `data_files`). 소스 트리 없이 설치본만
배포하는 경우의 폴백이다 — **파일을 새로 추가하면 재빌드해야** 설치본에도
들어간다.

## 데이터가 어디서 오나

| 화면 | 출처 |
|---|---|
| 카메라 3대 | `rgb/compressed`, `rear/rgb/compressed`, `torch/rgb/compressed` → WS 바이너리 채널 1/2/3 |
| 지금 켜진 카메라 | `drive_state` 의 `cam` (`front`/`rear`/`torch`) |
| 지령 이력 | `POST /cmd` 를 서버가 기록 → WS `{"type":"cmd"}` (지금은 그리는 화면이 없다. `store.cmds` 에 쌓이고 서버 로그에도 남는다) |
| 코스 중심선(관 튜브) | `course` 토픽(latched) → WS `{"type":"course"}` |
| CAD 메시(전체 맵) | `GET /mesh` — **`mesh` 토픽(latched)으로 Isaac 이 넘긴 것**이 1순위, 없으면 로컬 `.webmesh` 파일 |
| 로봇 빨간 점 | `drive_state` 의 `pos_m` (없으면 `s_mm`) |
| 결함 목록·✕ 마커 | `/defect/report_json` |
| 사건 로그·수리 스티커 | `event` (WELD_BEGIN/DONE) |
| 지나온 초록선 | `drive_state` 의 `max_s_mm` (서버가 누적) |
| 지령 버튼 | `POST /cmd` → `mission` 토픽 |

## 버튼 구성

| 버튼 | 지령 | 비고 |
|---|---|---|
| ▶ 시작 / ⏸ 정지 | `START` / `STOP` | **한 버튼 토글**. 글자는 로봇이 보내온 상태(`HOLD`/`DEAD` = 서 있음)를 따라간다 |
| ⏩ 전진 점검 | `FORWARD` | 복귀를 접고 다시 전진하며 결함을 찾는다 |
| ↩ 복귀 | `RECALL` | 방향을 뒤집어 출발점으로 |
| ⛔ 비상정지 / 🔓 해제 | `ESTOP` / `START`+`ESTOP_RELEASE` | 아래 참고 |

🔑 토글은 **누른 순간 뒤집지 않는다.** 시연이 지령을 미룰 수 있어서(바로 아래),
누르자마자 글자를 바꾸면 버튼이 거짓말을 한다. 대신 상태가 따라올 때까지
`⏸ 정지 대기…` 로 보여 준다 — 아무 표시도 없으면 안 먹은 줄 알고 계속 누른다.

🚨 `START` 는 **방향을 안 바꾼다.** 복귀 중에 ▶ 시작을 눌러도 계속 뒤로 간다.
앞으로 돌려세우는 것은 `FORWARD` 다(그래서 규약에 지령을 따로 뒀다).

## 지령 버튼은 **미뤄질 수 있다**

시연은 용접 시퀀스(ALIGN·EXTEND·ARC·COOL) 중에 STOP/RECALL/FORWARD 를 듣지
않는다 — 토치가 뻗고 아크가 붙은 상태로 세우면 재개 경로가 없다. 지령은
버려지지 않고 **주행 상태로 돌아오는 순간** 걸린다(거절보다 지연이 조작하는
사람에게 예측 가능하다). ESTOP 만 그 자리에서 즉시 언다.

서 있는 동안 시연은 상태를 `HOLD` / `ESTOP` 은 `DEAD` 로 싣고, `reason` 에
`HOLD(CRUISE)` 처럼 원래 FSM 상태를 괄호로 남긴다 — Robot Handling 의 "사유"
칸이 그것이다.

**비상정지 해제**: 비상정지 중에는 START 를 포함해 어떤 지령도 안 먹는다.
`🔓 비상정지 해제` 버튼만 듣는데, 이것은 규약에 지령을 새로 만들지 않고
**`START` 에 `reason=ESTOP_RELEASE` 를 실어** 보내는 것이다 — 실수로 ▶ 시작을
눌러서 비상정지가 풀리면 안 되기 때문이다. 해제해도 **바로 안 간다**: 정지
(HOLD)로 돌아오고, ▶ 시작을 한 번 더 눌러야 움직인다.

## 재시작(`run`) — 초록선이 다시 0 부터

Isaac GUI 에서 Stop→Play 로 시연을 다시 돌리면 `drive_state.step` 이 뒤로
간다. 서버(`web_panel.py` 의 `Store.set_state`)가 그것을 **재시작으로 읽어**
답파거리를 0 으로 되감고 `run` 번호를 올린다. 페이지는 `run` 이 바뀌면
지나온 초록선과 수리 마커를 지운다(`app.js` 의 `onState` → `bus` 의 `reset`).

🔑 시연 쪽에 새 토픽을 요구하지 않는다 — 이미 오는 `step` 으로 알아챈다.
사건 로그는 지우지 않는다(로그는 지난 판까지 포함해 읽는 것이다). 다만
**새로 붙는 브라우저**에는 지난 판의 사건을 다시 보내지 않는다 — 그러지
않으면 새로고침 한 번에 지난 판의 용접 스티커가 되살아난다.

🔑 **코스 좌표를 여기에 하드코딩하지 않는다.** 기하의 단일 출처는 시연
(`real_map_demo.py` 의 `CenterLine`) 이고, 맵이 바뀌면 그쪽만 고치면 된다.

## CAD 메시는 Isaac 이 굽고 토픽으로 넘긴다

**Isaac PC 와 웹 PC 가 다른 자리에 있다.** 그래서 `.webmesh` 파일을 여기서
읽을 수가 없다 — 시연이 기동할 때 굽고 `mesh` 토픽(latched, 0.77MB 1회)으로
넘긴다. 조각내기는 RTPS 가 알아서 한다(실측: latched 로 통과, 발행 1ms).

🚨 **굽는 쪽이 Isaac 인 이유는 맵 z 오프셋 때문이다.** 활성 층의 수평망을
월드 z=0 으로 올리는 값이 층마다 다른데(floor2 +250 / floor1 +2740.23),
그걸 아는 것은 맵을 배치하는 시연뿐이다. 받는 쪽이 파라미터로 다시 말해 주던
구조에서는 `--floor1` 로 돌리는 순간 **건물이 2.49m 어긋난 채로 그려졌다** —
에러 없이.

`web_panel` 도 굽는 손잡이(`-p bake_mesh:=true`)를 갖고 있지만 **기본은 꺼져
있다.** 둘 다 구우면 서로 다른 z 로 구워 나중에 실행한 쪽이 이기는 경합이
된다. 그 손잡이는 "시연 없이 웹만 띄워 볼 때" 의 탈출구다.

🚨 화면이 비어 있으면 **시연을 `--ros` 로 띄웠는지부터** 볼 것 — 카메라·코스·
로봇 위치가 전부 그 플래그에 달려 있다.

## Robot Handling 은 조종석이다

위에서부터 **임무 지령 → 카메라 → (현재 상태 | 사건 로그)** 한 화면이다.
카메라 칸과 로그 칸은 각 페이지의 모듈을 그대로 끼운다 —
`camera.js` 의 `mountCam(el, {bare:true})`, `events.js` 의 `mountLog(el, {limit})`.
🔑 사건 줄·카메라 칸을 두 벌 만들지 말 것. 한쪽만 고쳐지면 같은 사건이 두
화면에 다르게 보인다.

속도 슬라이더·지령 이력·지령 규약 카드는 뺐다(2026-08-08, 조종석을 비우려고).
`SPEED` 지령 자체는 규약·시연에 그대로 있으니 필요하면
`ros2 run pipe_comm mission_cli -- SPEED --mps 0.05` 로 보낼 수 있다.

## 카메라는 한 번에 한 대만 켜진다

| 언제 | 켜지는 카메라 |
|---|---|
| 전진 주행 | `front` |
| 후진·복귀 (`RETURN`/`RECOVER`) | `rear` |
| 정렬~아크 (`ALIGN`/`EXTEND`/`ARC`) | `torch` |

분기 규칙의 단일 출처는 시연의 `active_camera_name()` 이고, 시연은 그 한 대만
발행한다(안 쓰는 카메라를 안 굽는 만큼 JPEG 인코딩 비용이 빠진다). 지금 켜진
역할은 `drive_state.cam` 으로 온다.

🔑 **화면도 한 칸만 만든다.** 예전에는 3칸을 나란히 두고 둘은 늘 "대기" 였는데,
안 쓰는 칸이 마지막 프레임을 **디코딩된 채로** 붙들고 있었다(브라우저는
`<img>` 에 걸린 그림을 화면 밖이라고 버리지 않는다). 지금은 분기가 바뀌면
그 칸을 비우고(`objectURL` 회수 + `src` 제거) 새 역할로 갈아 끼우며,
`app.js` 의 `dropOtherCams()` 가 store 에 남은 다른 역할의 Blob 도 놓는다.
Home 의 카메라 칸도 켜진 것을 따라 바뀐다.

🚨 그래서 "후방 카메라가 안 나온다" 는 대개 고장이 아니라 이 분기다. 켜져
있는데 끊긴 경우에만 "N초째 프레임 없음" 으로 적는다.

## Detect List 는 보고서 파일이 아니다

주행이 끝난 뒤 올라오는 파일이 아니라, dongyeon 검출 노드(`pipe_vision_node`)가
**결함을 볼 때마다 실시간으로** 흘리는 `/defect/report_json` 을 표로 쌓은
것이다. 그래서 검출 노드를 안 띄우면 **끝까지 비어 있다**(시연만으로는 안
찬다). 파일로 남기는 것은 별개 노드(`pipe_report_node`)가 디스크에 한다.
