"""웹 관제 패널 — 브라우저에서 로봇을 **보고 조종한다** (FastAPI).

`web_view` 가 "카메라가 오는지" 보는 진단 도구라면, 이것은 시연용 관제
화면이다. 한 페이지에서:

    ▶ 시작 / ⏸ 정지 / ↩ 복귀 / ⛔ 비상정지     → `mission` 토픽 발행
    속도 슬라이더                               → `mission` SPEED
    3D 배관 맵 + 현재 위치 + 지나간 구간         ← `course` + `drive_state`
    전방 카메라                                  ← `rgb/compressed`
    결함 목록                                    ← `/defect/report_json` (dongyeon)
    사건 로그                                    ← `event`

의존성: `pip install fastapi uvicorn` (ROS 노드 쪽 3.10 전용 — Isaac 쪽과
무관하다). `web_view` 와 달리 외부 패키지를 쓰는 이유는 **양방향**이 되면서
웹소켓·REST·JSON 검증을 손으로 다 짜는 비용이 재사용 이득을 넘어섰기 때문.
zero-dep 진단 경로가 필요하면 `web_view` 가 그대로 있다.

━━ 페이지 파일 — 워크스페이스의 `web/` (한 파일 원칙을 버렸다) ━━━━━━

HTML/CSS/JS 는 이 패키지 안이 아니라 **`$COBOT3_WS/web/`** 에 있다
(index.html, panel.css, panel.js, three.js 벤더 사본). 3D 맵을 three.js 로
옮기면서 페이지가 파이썬 문자열로 들고 있기엔 너무 커졌고, 웹 소스는
ROS 파이썬 패키지와 수명·도구가 달라 따로 두는 편이 낫다. three.js 는
CDN 이 아니라 벤더 사본이다 — 현장망(오프라인)에서도 떠야 한다.

🔑 서빙 우선순위: **소스 트리**($COBOT3_WS/web) > 설치본
   (share/pipe_comm/web, setup.py 의 data_files 가 복사). 이 패키지는
   --symlink-install 로도 파일이 복사되어 .py 수정마다 재빌드해야 하는데,
   소스를 먼저 잡으면 HTML/CSS/JS 는 **재빌드 없이 새로고침**으로 반영된다.

━━ 3D 배관 맵 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

코스 기하는 하드코딩하지 않는다 — 시연(real_map_demo --ros)이 기동할 때
`course` 토픽(latched)으로 중심선 표본을 발행하고(단일 출처는 그쪽
CenterLine), 이 노드가 받아 웹소켓 {"type":"course"} 로 페이지에 넘긴다.
맵이 바뀌면 시연 쪽만 고치면 된다. 로봇 위치(pos_m)·결함(s, 시계각)은
그 좌표계(월드 m) 그대로 그린다.

CAD 메시: `tools/usd_to_webmesh.py` 로 구운 `.webmesh` 가 있으면 `/mesh` 로
서빙하고, 페이지가 전체 맵(floor2+floor1+aisle)을 반투명 메시로 그린다.
파일이 없으면 코스 튜브만으로 동작한다 — 메시는 항상 **덤**이다.
경로: `-p mesh:=<파일>` 지정 > $COBOT3_WS/src/dongyeon/integration_test/
maps/*.webmesh 중 최신 것.

━━ 지령 경로 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

버튼 → POST /cmd → `contract.mission()` 검증 → `mission` 토픽 발행.
mission_cli 와 같은 원칙으로 **발행 뒤 구독자 수를 확인해** 0 이면 응답에
경고를 실어 화면에 띄운다 — RELIABLE 이라도 구독자가 없으면 지령은
소리 없이 사라지기 때문이다(latch 가 아니다).

실행:
  ros2 run pipe_comm web_panel --ros-args -p ns:=robot -p port:=8080
  → http://<서버IP>:8080   (web_view 와 포트가 겹치니 둘 중 하나만 띄울 것)

🚨 인증이 없다. 공인망에 열지 말 것 — 버튼이 있으니 web_view 보다 위험하다.
   보안그룹은 본인 IP /32 로만, 또는 SSH 터널을 쓸 것.
"""

import json
import os
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, UInt8MultiArray

from pipe_comm import contract
from pipe_comm.contract import Topics
from pipe_comm.drive_monitor import _quat_to_roll

# dongyeon 검출 노드의 출력. 네임스페이스가 없는 절대 토픽이다.
DEFECT_REPORT_TOPIC = "/defect/report_json"

# 웹소켓 바이너리 첫 바이트 = 채널 (web/app.js 의 CH 와 맞출 것)
CH_RGB = b"\x01"       # 전방 카메라
CH_REAR = b"\x02"      # 후방 카메라
CH_TORCH = b"\x03"     # 토치 카메라 (용접부 근접)
CAM_CH = {"front": CH_RGB, "rear": CH_REAR, "torch": CH_TORCH}


class Store:
    """최신 상태 보관소. ROS 스레드가 쓰고 웹소켓 태스크들이 읽는다.

    사건·결함은 **덧붙이기만 하는 목록**이다 — 웹소켓마다 어디까지 보냈는지
    (인덱스)를 따로 들고 가므로, 늦게 접속한 브라우저도 지난 결함을 다 받는다.
    """

    MAX_LOG = 300          # 사건 로그 상한. 넘치면 앞을 버린다

    def __init__(self):
        self.lock = threading.Lock()
        self.state = None            # drive_state dict + roll/odom 보강
        self.state_seq = 0
        self.course = None           # course dict — latched 라 마지막 것만
        self.course_seq = 0
        # 카메라 3대 — {역할: JPEG 바이트}, 역할별 시퀀스 번호.
        # 웹소켓마다 어디까지 보냈는지를 이 번호로 따라간다.
        self.cam = {"front": None, "rear": None, "torch": None}
        self.cam_seq = {"front": 0, "rear": 0, "torch": 0}
        # 지나간 최대 호길이 — 지도는 이 구간까지만 그린다("촬영한 만큼").
        # 서버가 들고 있어야 브라우저를 새로고침해도 지도가 안 사라진다.
        self.max_s_mm = 0.0
        # 시연 판(run) 번호. Isaac GUI 에서 Stop→Play 로 **재시작**하면 올라간다.
        # 웹은 이 번호가 바뀌는 것만 보고 지나온 초록선·수리 마커를 지운다.
        self.run = 0
        self.last_step = -1
        self.events = []             # event dict 목록
        self.events_dropped = 0      # MAX_LOG 로 버린 수 (인덱스 보정용)
        # 이 패널이 **보낸** 지령의 이력. 사건(event)과 달리 로봇이 아니라
        # 여기서 생기는 기록이라 따로 둔다 — "누가 뭘 눌렀나" 는 로봇 상태만
        # 봐서는 못 읽는다(구독자 0 이면 로봇에는 아예 안 갔을 수도 있다).
        self.cmds = []
        self.cmds_dropped = 0
        self.defects = []            # report dict 목록 (갱신도 새 항목으로)
        self.roll_deg = None
        self.odom_v = None
        # 🔑 Isaac 쪽에서 latched 로 넘어온 CAD 메시(.webmesh 바이트).
        #    **PC 가 갈릴 때의 주 경로다** — 파일은 저쪽 디스크에만 있다.
        #    이게 오면 로컬 파일보다 **먼저** 쓴다: 시연이 실제로 쓴 맵과
        #    z 오프셋으로 구워진 것이라 언제나 더 옳다.
        self.mesh = None

    def set_state(self, d):
        with self.lock:
            if self.roll_deg is not None:
                d["roll_deg"] = round(self.roll_deg, 1)
            if self.odom_v is not None:
                d["odom_v_mps"] = round(self.odom_v, 4)
            # 🔑 재시작은 **step 이 뒤로 가는 것**으로 드러난다 (시연이 Stop→Play
            #    에서 step 을 0 으로 되돌린다). 시연 쪽에 새 토픽·새 필드를
            #    요구하지 않고 이미 오는 값으로 알아채는 것이 요점이다.
            step = int(d.get("step") or 0)
            if step < self.last_step:
                self.run += 1
                self.max_s_mm = 0.0
                # 지난 판의 사건은 **다음에 붙는 브라우저에 다시 보내지 않는다** —
                # 그러지 않으면 새로고침한 화면에 지난 판의 용접 마커가 되살아난다.
                # events_dropped 는 "앞에서 버린 수" 라 소켓 인덱스는 그대로 맞는다.
                self.events_dropped += len(self.events)
                self.events.clear()
            self.last_step = step
            d["run"] = self.run
            s = float(d.get("s_mm") or 0.0)
            if s > self.max_s_mm:
                self.max_s_mm = s
            d["max_s_mm"] = round(self.max_s_mm, 1)
            self.state = d
            self.state_seq += 1

    def set_course(self, d):
        with self.lock:
            self.course = d
            self.course_seq += 1

    def set_cam(self, role, data):
        with self.lock:
            self.cam[role] = data
            self.cam_seq[role] += 1

    def add_event(self, d):
        with self.lock:
            self.events.append(d)
            if len(self.events) > self.MAX_LOG:
                cut = len(self.events) - self.MAX_LOG
                del self.events[:cut]
                self.events_dropped += cut

    def add_cmd(self, d):
        with self.lock:
            self.cmds.append(d)
            if len(self.cmds) > self.MAX_LOG:
                cut = len(self.cmds) - self.MAX_LOG
                del self.cmds[:cut]
                self.cmds_dropped += cut

    def add_defect(self, d):
        with self.lock:
            self.defects.append(d)


class PanelNode(Node):

    def __init__(self, store):
        super().__init__("web_panel")
        self.declare_parameter("ns", contract.DEFAULT_NS)
        self.declare_parameter("port", 8080)
        self.declare_parameter("mesh", "")   # .webmesh 경로 (빈 값 = 자동 탐색)
        # 🔑 `.webmesh` 가 없거나 낡았으면 기동 때 **직접 굽는다**(0.5초쯤).
        #    map_usd 를 비우면 maps/ 의 가장 최근 `.usd` 를 고른다.
        #    🚨 z_shift 는 **층마다 다르다** — real_map_demo floor2 가 250 이다.
        #       틀리면 건물이 로봇보다 위/아래로 통째로 어긋나 보인다.
        self.declare_parameter("map_usd", "")
        self.declare_parameter("z_shift_mm", 250.0)
        # 🚨 **기본은 끈다.** 굽는 쪽은 Isaac 이다 — 맵 배치(층별 z)를 아는 것이
        #    거기뿐이라, 여기서도 구우면 둘이 다른 z 로 구워 **나중에 실행한
        #    쪽이 이기는 경합**이 된다. 이 손잡이는 "시연 없이 웹만 띄워 볼 때"
        #    의 탈출구다. Isaac 쪽은 `mesh` 토픽으로 넘긴다.
        self.declare_parameter("bake_mesh", False)

        self.store = store
        self.ns = str(self.get_parameter("ns").value)
        t = Topics(self.ns)
        sq, cq = contract.sensor_qos(), contract.command_qos()

        self.pub_mission = self.create_publisher(String, t.mission, cq)

        self.create_subscription(String, t.drive_state, self._on_state, cq)
        self.create_subscription(String, t.event, self._on_event, cq)
        # 코스 기하 — latched. 시연이 이 노드보다 먼저 떠 있어도 받는다.
        self.create_subscription(String, t.course, self._on_course,
                                 contract.latched_qos())
        # CAD 메시 — 역시 latched. Isaac PC 가 따로 있을 때 이쪽으로 온다.
        self.create_subscription(UInt8MultiArray, t.mesh, self._on_mesh,
                                 contract.latched_qos())
        # 카메라 3대 — 페이지 Camera 화면이 셋을 나란히 본다
        self.create_subscription(CompressedImage, t.rgb,
                                 lambda m: store.set_cam("front",
                                                         bytes(m.data)), sq)
        self.create_subscription(CompressedImage, t.rear_rgb,
                                 lambda m: store.set_cam("rear",
                                                         bytes(m.data)), sq)
        self.create_subscription(CompressedImage, t.torch_rgb,
                                 lambda m: store.set_cam("torch",
                                                         bytes(m.data)), sq)
        self.create_subscription(Odometry, t.odom, self._on_odom, sq)
        self.create_subscription(Imu, t.imu, self._on_imu, sq)
        # 결함 리포트 — dongyeon 쪽은 기본 QoS(RELIABLE, depth 10)로 발행한다
        self.create_subscription(String, DEFECT_REPORT_TOPIC,
                                 self._on_defect, 10)
        contract.log_env(self.get_logger())

    def _on_state(self, msg):
        d = contract.parse(msg.data)
        if d is not None:
            self.store.set_state(d)

    def _on_event(self, msg):
        d = contract.parse(msg.data)
        if d is not None:
            self.store.add_event(d)

    def _on_course(self, msg):
        d = contract.parse(msg.data)
        if d is not None and d.get("pts"):
            self.store.set_course(d)
            self.get_logger().info(
                f"코스 수신 — 표본 {len(d['pts'])}개, "
                f"총 {d.get('s_total_m', 0) * 1000:.0f}mm")

    def _on_odom(self, msg):
        self.store.odom_v = float(msg.twist.twist.linear.x)

    def _on_imu(self, msg):
        self.store.roll_deg = _quat_to_roll(msg.orientation)

    def _on_mesh(self, msg):
        """Isaac 쪽 CAD 메시. **로컬 파일보다 이게 옳다** — 시연이 실제로 쓴
        맵과 z 오프셋으로 구워진 것이다."""
        data = bytes(msg.data)
        if not data:
            return
        self.store.mesh = data
        self.get_logger().info(
            f"CAD 메시 수신 — {len(data) / 1e6:.2f} MB (Isaac 쪽에서 latched). "
            f"이제부터 /mesh 는 이것을 준다")

    def _on_defect(self, msg):
        d = contract.parse(msg.data)
        if d is not None:
            self.store.add_defect(d)

    # ── 지령 ────────────────────────────────────────────────────
    def send_mission(self, cmd, mps=None, reason=""):
        """지령을 발행하고 (구독자 수) 를 돌려준다. 검증은 contract 가 한다."""
        payload = contract.mission(cmd, mps=mps, reason=reason)
        self.pub_mission.publish(String(data=contract.dumps(payload)))
        n = self.pub_mission.get_subscription_count()
        self.get_logger().info(f"지령 {cmd}"
                               + (f" mps={mps}" if mps is not None else "")
                               + f" → 구독자 {n}")
        return n


def _ws() -> Path:
    """워크스페이스 루트. 설치본(install/)에서 돌므로 __file__ 상대 경로는 못
    쓴다 — COBOT3_WS(~/.bashrc 가 export)로 잡고, 없으면 ~/cobot3_ws 로 짐작."""
    return Path(os.environ.get("COBOT3_WS", str(Path.home() / "cobot3_ws")))


def bake_mesh(usd: Path, z_shift_mm: float, logger) -> Path | None:
    """맵 USD 를 `.webmesh` 로 굽는다. 실패하면 None — **메시는 항상 덤이다.**

    🔑 `tools/usd_to_webmesh.py` 를 **경로로 읽어** 그 안의 `convert()` 를
       부른다. tools/ 는 파이썬 패키지가 아니라 colcon 이 설치하지 않으므로
       import 로는 못 잡는다. 규칙(포맷·좌표계·법선 재계산)은 그 파일 한 곳에만
       둔다 — 여기에 베껴 적으면 반드시 갈라진다.

    🚨 이 노드는 python 3.10 이고 USD 바인딩(`pxr`)은 `usd-core` pip 패키지로
       들어와 있다(실측 0.26.8). 다른 PC 에는 없을 수 있으므로 **없어도 그냥
       진행한다** — 3D 맵은 코스 튜브만으로 동작한다.
    """
    import importlib.util

    tool = _ws() / "src/dongmin/pipe_comm/tools/usd_to_webmesh.py"
    if not tool.is_file():
        logger.warning(f"메시 변환기가 없다: {tool}")
        return None
    try:
        spec = importlib.util.spec_from_file_location("_usd_to_webmesh", tool)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        logger.info(f"CAD 메시 굽는 중 — {usd.name} (z {z_shift_mm:+.0f}mm)…")
        return mod.convert(usd, z_shift_mm)
    except Exception as exc:                     # pxr 없음·USD 깨짐 등 전부
        logger.warning(f"CAD 메시를 못 구웠다({exc.__class__.__name__}: {exc}) "
                       f"— 3D 맵은 코스 튜브만 그린다")
        return None


def find_mesh(explicit: str, logger, map_usd: str = "",
              z_shift_mm: float = 250.0, bake: bool = True) -> bytes | None:
    """`.webmesh` 를 찾아 통째로 읽는다. 없으면 None — 페이지는 코스만 그린다.

    🔑 **없거나 낡았으면 그 자리에서 굽는다**(0.5초쯤). 손으로 한 번 굽는 것을
       잊어 "3D 맵에 건물이 안 보인다" 가 되는 일이 잦아서다. `.usd` 보다
       오래된 `.webmesh` 도 다시 굽는다 — **낡은 메시는 없는 것보다 나쁘다**
       (맵이 바뀐 줄 모르고 옛 건물 위에 로봇을 그린다).

    map_usd     구울 원본. 비우면 maps/ 에서 가장 최근 `.usd` 를 고른다.
    z_shift_mm  맵 z 오프셋. real_map_demo floor2 기준 250 (floor1 은 다르다).
    bake        False 면 옛 동작(찾기만 한다).
    """
    if explicit:
        cands = [Path(explicit)]
    else:
        cands = sorted(
            _ws().glob("src/dongyeon/integration_test/maps/*.webmesh"),
            key=lambda p: p.stat().st_mtime, reverse=True)

    # 구울 원본을 정한다 — 지정이 없으면 maps/ 의 가장 최근 .usd.
    usd = Path(map_usd) if map_usd else None
    if bake and usd is None:
        usds = sorted(_ws().glob("src/dongyeon/integration_test/maps/*.usd"),
                      key=lambda p: p.stat().st_mtime, reverse=True)
        usd = usds[0] if usds else None

    if bake and usd is not None and usd.is_file():
        out = usd.with_suffix(".webmesh")
        # 낡음 판정: 메시가 없거나 원본보다 오래됐다
        stale = (not out.is_file()
                 or out.stat().st_mtime < usd.stat().st_mtime)
        if stale:
            if out.is_file():
                logger.info(f"CAD 메시가 낡았다 — {usd.name} 이 더 새롭다. 다시 굽는다")
            baked = bake_mesh(usd, z_shift_mm, logger)
            if baked is not None:
                cands = [baked] + [p for p in cands if p != baked]
        elif out not in cands:
            cands = [out] + cands

    for p in cands:
        if p.is_file():
            data = p.read_bytes()
            logger.info(f"CAD 메시 서빙: {p} ({len(data) / 1e6:.2f} MB)")
            return data
    logger.info("CAD 메시 없음 — 3D 맵은 코스 튜브만 그린다. "
                "tools/usd_to_webmesh.py 로 구울 것")
    return None


def find_web(logger) -> Path:
    """페이지 소스 디렉터리(`web/`). **소스 트리를 설치본보다 먼저** 잡는다.

    이 패키지는 --symlink-install 로도 파일이 복사되어 .py 수정마다 재빌드가
    필요한데(기록된 함정), 소스를 먼저 보면 HTML/CSS/JS 수정은 재빌드 없이
    **새로고침만으로** 반영된다. 설치본(share/pipe_comm/web)은 소스 트리 없이
    배포된 경우의 폴백이다 — setup.py 의 data_files 가 거기로 복사한다.
    """
    ws = Path(os.environ.get("COBOT3_WS", str(Path.home() / "cobot3_ws")))
    cands = [ws / "web"]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(Path(get_package_share_directory("pipe_comm")) / "web")
    except Exception:
        pass
    for i, p in enumerate(cands):
        if (p / "index.html").is_file():
            logger.info(f"웹 소스: {p}"
                        + (" (소스 트리 — 수정이 새로고침으로 반영된다)"
                           if i == 0 else " (설치본)"))
            return p
    raise SystemExit(f"web/index.html 이 없다 — 찾은 곳: "
                     f"{[str(c) for c in cands]}. 워크스페이스가 다른 자리면 "
                     f"COBOT3_WS 를 export 하고, 설치본을 쓸 거면 colcon "
                     f"build 를 다시 할 것")


def build_app(store, node, mesh_bytes=None, web_dir=None):
    """FastAPI 앱. import 를 함수 안에 두어 fastapi 미설치 환경에서도
    모듈 자체는 읽히게 한다(오프라인 테스트가 Store 만 쓸 수 있게)."""
    import asyncio

    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse, JSONResponse, Response
    from fastapi.staticfiles import StaticFiles

    app = FastAPI(title="pipe_comm web_panel")
    app.mount("/static", StaticFiles(directory=str(web_dir)), name="static")

    def page():
        # 매 요청마다 읽는다 — 소스 서빙일 때 수정이 새로고침으로 반영된다.
        html = (web_dir / "index.html").read_text(encoding="utf-8")
        return HTMLResponse(html.replace("__NS__", node.ns))

    # 🔑 페이지는 하나(셸)이고 갈라지는 것은 브라우저 쪽 라우터다. 서버는 어느
    #    주소로 들어와도 같은 셸을 준다 — 그래야 `/camera` 를 **직접 치거나
    #    새로고침해도** 404 가 아니다(웹소켓 하나를 페이지 전환 내내 공유하는
    #    구조라 서버 라우팅으로 가르면 전환마다 재접속이 된다).
    #    경로 목록은 web/app.js 의 ROUTES 와 맞출 것.
    for _p in ("/", "/home", "/camera", "/handling", "/map", "/detect",
               "/events"):
        app.get(_p)(page)

    @app.get("/mesh")
    async def mesh():
        # 🔑 **Isaac 쪽에서 온 것이 언제나 우선이다.** 시연이 실제로 쓴 맵과
        #    z 오프셋으로 구워진 것이라, 이쪽 디스크의 파일보다 옳다(층이
        #    바뀌면 로컬 파일은 2.49m 어긋난 채로 남아 있을 수 있다).
        #    브라우저가 페이지를 열 때 한 번 가져가므로, 늦게 도착해도
        #    새로고침하면 새 것을 받는다.
        data = store.mesh if store.mesh is not None else mesh_bytes
        if data is None:
            return JSONResponse({"error": "no mesh"}, status_code=404)
        return Response(content=data,
                        media_type="application/octet-stream")

    @app.post("/cmd")
    async def cmd(body: dict):
        import time as _t
        c = str(body.get("cmd", "")).upper()
        mps = body.get("mps")
        reason = str(body.get("reason", "web_panel"))
        rec = {"stamp": round(_t.time(), 3), "cmd": c, "reason": reason}
        if mps is not None:
            rec["mps"] = mps
        try:
            n = node.send_mission(c, mps=mps, reason=reason)
        except ValueError as exc:
            # 🔑 실패한 지령도 이력에 남긴다 — 왜 아무 일도 안 일어났는지가
            #    바로 이 줄에 있다(오타·미지원 지령).
            rec.update(ok=False, subscribers=0, error=str(exc))
            store.add_cmd(rec)
            return JSONResponse({"ok": False, "error": str(exc)},
                                status_code=400)
        # 구독자 0 = 지령이 아무 데도 안 갔다. 화면이 이 사실을 띄워야 한다.
        rec.update(ok=n > 0, subscribers=n)
        store.add_cmd(rec)
        return {"ok": n > 0, "subscribers": n,
                "warn": None if n > 0 else
                "구독자 0 — Isaac 쪽 시연(repair_demo --ros)이 떠 있는가?"}

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        state_seq = course_seq = -1
        cam_seq = {r: -1 for r in CAM_CH}
        ev_i = df_i = cm_i = 0
        try:
            while True:
                sent = False
                with store.lock:
                    state, s_seq = store.state, store.state_seq
                    course, c_seq = store.course, store.course_seq
                    cams = {r: (store.cam[r], store.cam_seq[r])
                            for r in CAM_CH}
                    events = store.events[ev_i - store.events_dropped:]
                    ev_next = len(store.events) + store.events_dropped
                    cmds = store.cmds[cm_i - store.cmds_dropped:]
                    cm_next = len(store.cmds) + store.cmds_dropped
                    defects = store.defects[df_i:]
                    df_next = len(store.defects)
                # 코스를 상태보다 먼저 — 페이지가 코스 없이는 점을 못 찍는다
                if course is not None and c_seq != course_seq:
                    await sock.send_text(json.dumps(
                        {"type": "course", "data": course},
                        ensure_ascii=False))
                    course_seq, sent = c_seq, True
                if state is not None and s_seq != state_seq:
                    await sock.send_text(json.dumps(
                        {"type": "state", "data": state}, ensure_ascii=False))
                    state_seq, sent = s_seq, True
                for e in events:
                    await sock.send_text(json.dumps(
                        {"type": "event", "data": e}, ensure_ascii=False))
                    sent = True
                ev_i = ev_next
                for m in cmds:
                    await sock.send_text(json.dumps(
                        {"type": "cmd", "data": m}, ensure_ascii=False))
                    sent = True
                cm_i = cm_next
                for d in defects:
                    await sock.send_text(json.dumps(
                        {"type": "defect", "data": d}, ensure_ascii=False))
                    sent = True
                df_i = df_next
                for role, ch in CAM_CH.items():
                    data, seq = cams[role]
                    if data is not None and seq != cam_seq[role]:
                        await sock.send_bytes(ch + data)
                        cam_seq[role], sent = seq, True
                if not sent:
                    await asyncio.sleep(0.03)
        except (WebSocketDisconnect, ConnectionResetError):
            pass

    return app


def main(args=None):
    try:
        import uvicorn
    except ImportError:
        raise SystemExit("uvicorn/fastapi 가 없다 — pip install fastapi uvicorn")

    rclpy.init(args=args)
    store = Store()
    node = PanelNode(store)
    port = int(node.get_parameter("port").value)
    mesh_bytes = find_mesh(
        str(node.get_parameter("mesh").value), node.get_logger(),
        map_usd=str(node.get_parameter("map_usd").value),
        z_shift_mm=float(node.get_parameter("z_shift_mm").value),
        bake=bool(node.get_parameter("bake_mesh").value))
    web_dir = find_web(node.get_logger())

    # rclpy 는 뒷단 스레드에서 돌리고, uvicorn 이 주 스레드를 가진다.
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    node.get_logger().info(
        f"관제 패널 시작 [{node.ns}] — http://<서버IP>:{port}\n"
        f"  🚨 지령 버튼이 있는 페이지다. 공인망에 열지 말 것 (본인 IP /32 "
        f"또는 SSH 터널)")
    try:
        uvicorn.run(build_app(store, node, mesh_bytes, web_dir),
                    host="0.0.0.0", port=port,
                    log_level="warning")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
