
# ══════════════════════════════════════════════════════════════
#  STA — Pick & Place + ROS 색상 감지  (PC A / Isaac Sim 측)
#
#   1. 파란/초록 큐브 2개가 공중에서 대기
#   2. 그 중 하나가 랜덤으로 pick 영역에 낙하
#   3. Wrist Camera 가 /rgb 발행  (USD 내장 ROS2 Camera Graph)
#   4. PC B(color_detector)가 /color_id 발행  (파랑=1 / 초록=2)
#   5. 감지된 색상과 같은 색 마커 위치에 Place
# ══════════════════════════════════════════════════════════════

import argparse
import sys as _sys

# ── 실행 옵션 ─────────────────────────────────────────────────
#   (옵션 없이 실행하면 GUI 로 뜨고 ▶ Play 를 누를 때까지 대기)
_parser = argparse.ArgumentParser(description="M0609 색상 인식 Pick & Place")
_parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
_parser.add_argument("--autoplay", action="store_true", help="Play 자동 시작")
_parser.add_argument("--rounds", type=int, default=0, help="반복 횟수 (0=무한)")
_args, _ = _parser.parse_known_args(_sys.argv[1:])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": _args.headless})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import sys
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

# ── ROS2 수신은 OmniGraph(ROS2 Bridge)로 처리 ─────────────────
#   ※ Isaac Sim 5.x 는 Python 3.11, ROS2 Humble 의 rclpy 는 3.10 빌드라
#     isaac_python 안에서는 rclpy 를 import 할 수 없다.
#     → ROS2Subscriber OmniGraph 노드로 /color_id 를 구독한다.
import omni.graph.core as og

_THIS_DIR = Path(__file__).resolve().parent

# rmpflow 인프라 폴더 경로 등록
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_pick_place_controller import PickPlaceController
from m0609_rmpflow_controller import RMPFlowController

from isaacsim.core.utils.rotations import euler_angles_to_quat

# 손목 카메라가 아래(테이블)를 향하는 EE 자세
EE_DOWN_QUAT = euler_angles_to_quat(np.array([0, np.pi, 0]))

# ╔══════════════════════════════════════════════════════════════╗
# ║  A. Task 파라미터                                             ║
# ╚══════════════════════════════════════════════════════════════╝
USD_PATH        = str(_THIS_DIR / "Collected_m0609_camera/m0609_camera.usd")
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
GRIPPER_JOINTS  = ["finger_joint", "right_inner_knuckle_joint"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

GRIPPER_OPEN    = [0.0, 0.0]
GRIPPER_CLOSE   = [0.5, 0.5]
GRIPPER_DELTA   = [-0.5, -0.5]

FINGER_STATIC   = 1.8
FINGER_DYNAMIC  = 1.4
CUBE_STATIC     = 1.2
CUBE_DYNAMIC    = 1.0

# ╔══════════════════════════════════════════════════════════════╗
# ║  B. 색상 시나리오 파라미터 (★ 이번 장에서 새로 추가)            ║
# ╚══════════════════════════════════════════════════════════════╝
CUBE_SIZE   = 0.05
CUBE_Z      = CUBE_SIZE / 2.0 + 0.0015          # 테이블 위 안착 높이

# ── B-1. 공중 대기 위치 (물리 OFF 상태로 떠 있음) ──────────────
STANDBY_POS = {
    1: np.array([0.10,  0.55, 0.55]),           # 파란 큐브
    2: np.array([-0.10, 0.55, 0.55]),           # 초록 큐브
}

# ── B-2. 랜덤 Pick 영역 (여기 안에서 낙하 지점 추첨) ───────────
PICK_X_RANGE = (0.33, 0.47)
PICK_Y_RANGE = (0.28, 0.45)

# ── B-3. 색상별 Place 마커 위치 ────────────────────────────────
PLACE_POS = {
    1: np.array([0.55, -0.28, 0.0]),            # color_id=1 → 파란 마커
    2: np.array([0.38, -0.46, 0.0]),            # color_id=2 → 초록 마커
}

COLOR_RGB = {
    1: np.array([0.0, 0.1, 1.0]),               # 파랑
    2: np.array([0.0, 1.0, 0.1]),               # 초록
}
COLOR_NAME = {0: "없음", 1: "파랑", 2: "초록"}

# ── B-4. Pick & Place 동작 파라미터 ───────────────────────────
EE_OFFSET     = np.array([0.0, 0.0, 0.2])       # 접근 높이
SETTLE_STEPS  = 90                              # 낙하 안정화 스텝
DETECT_HOLD   = 5                               # 같은 값 연속 N회 → 확정
COLOR_LOCK_EVENT = 5                            # event 5 이후 색상 확정(변경 금지)

# 관측 자세 : 큐브 바로 위 이 높이로 EE(손목 카메라)를 보낸다.
#   색상을 받을 때까지 Pick 을 시작하지 않으므로,
#   PC B 의 감지 노드가 없으면 로봇은 여기서 계속 대기한다.
OBSERVE_OFFSET = np.array([0.0, 0.0, 0.28])

# 손목 카메라가 큐브 바로 위에 있을 때만 색상 판정을 신뢰한다.
#   (홈 자세에서는 Place 마커가 화면에 잡혀 오판할 수 있음)
DETECT_EE_RADIUS = 0.15                         # EE-큐브 수평 거리 임계 [m]
ROUNDS        = _args.rounds                    # 0 = 무한 반복 (--rounds N)

# ── B-5. ROS 토픽 / OmniGraph 경로 ────────────────────────────
COLOR_ID_TOPIC   = "color_id"                    # → /color_id
COLOR_GRAPH_PATH = "/World/ColorIdGraph"
COLOR_SUB_NODE   = COLOR_GRAPH_PATH + "/color_id_sub"

# ── B-6. 10단계 타이밍 (작을수록 빠름) ────────────────────────
EVENTS_DT = [
    0.008,   # 0. 접근 이동
    0.005,   # 1. 하강
    0.02,    # 2. 그리퍼 닫기 대기
    0.1,     # 3. 그리퍼 닫힘 유지
    0.0025,  # 4. 들어올리기
    0.01,    # 5. Place 위치로 이동
    0.0025,  # 6. 하강
    1,       # 7. 그리퍼 열기 대기
    0.008,   # 8. 상승
    0.08,    # 9. 복귀
]

# ── B-7. 인프라 파일 경로 (RMPFlow가 참조) ────────────────────
M0609_URDF_PATH           = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")


# ============================================================
# 유틸
# ============================================================
def find_prim_path_by_name(root_path: str, name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def initialize_robot(robot, world):
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    robot.set_joint_positions(np.zeros(robot.num_dof))


# ============================================================
# ROS2 — /color_id 구독 (OmniGraph ROS2Subscriber)
# ============================================================
def build_color_id_graph():
    """/color_id (std_msgs/Int32) 를 구독하는 OmniGraph 를 만든다.

    Isaac Sim(3.11)에서는 rclpy(3.10)를 쓸 수 없으므로
    ROS2 Bridge 의 범용 Subscriber 노드로 메시지를 받는다.
    """
    og.Controller.edit(
        {"graph_path": COLOR_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("color_id_sub", "isaacsim.ros2.bridge.ROS2Subscriber"),
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "color_id_sub.inputs:execIn"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("color_id_sub.inputs:topicName", COLOR_ID_TOPIC),
                ("color_id_sub.inputs:messagePackage", "std_msgs"),
                ("color_id_sub.inputs:messageSubfolder", "msg"),
                ("color_id_sub.inputs:messageName", "Int32"),
                ("color_id_sub.inputs:queueSize", 1),
            ],
        },
    )
    print(f"  [OK] ROS2Subscriber graph: {COLOR_SUB_NODE} → /{COLOR_ID_TOPIC}")


class ColorIdReader:
    """OmniGraph 구독 노드의 outputs:data 를 매 스텝 읽어 판정한다."""

    def __init__(self):
        self._attr = None
        self._latest = 0        # 마지막 수신값
        self._stable = 0        # 연속 확정값
        self._prev = 0          # 직전 수신값
        self._count = 0         # 연속 횟수
        self._rx = 0            # 유효값(0 아님) 폴링 횟수

    def _resolve_attr(self):
        """메시지 타입이 확정된 뒤에야 outputs:data 가 생성된다."""
        if self._attr is not None:
            return self._attr
        try:
            node = og.Controller.node(COLOR_SUB_NODE)
            if node is None or not node.is_valid():
                return None
            if not node.get_attribute_exists("outputs:data"):
                return None
            self._attr = node.get_attribute("outputs:data")
        except Exception:
            return None
        return self._attr

    def poll(self):
        """매 시뮬레이션 스텝 호출 — 구독값을 읽어 상태 갱신."""
        attr = self._resolve_attr()
        if attr is None:
            return
        try:
            value = int(og.Controller.get(attr))
        except Exception:
            return

        self._latest = value
        if value != 0:
            self._rx += 1

        # 같은 값이 DETECT_HOLD 회 연속이면 확정
        self._count = self._count + 1 if value == self._prev else 1
        self._prev = value
        if value != 0 and self._count >= DETECT_HOLD:
            self._stable = value

    # ── 외부 인터페이스 ──────────────────────────────────────
    @property
    def latest(self):
        return self._latest

    @property
    def stable(self):
        return self._stable

    @property
    def rx_count(self):
        return self._rx

    @property
    def connected(self):
        return self._attr is not None

    def clear(self):
        """새 라운드 시작 시 이전 판정 초기화."""
        self._latest = 0
        self._stable = 0
        self._count = 0
        self._prev = 0


# ============================================================
# Task — 큐브 2개 + 색상 마커 2개
# ============================================================
class M0609ColorTask(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self.cubes = {}         # {color_id: DynamicCuboid}

    # ── 씬 구성 5단계 ────────────────────────────────────────
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)
        self._create_scene(scene)
        print("\n  [완료] 씬 구성 성공!\n")

    def _load_usd(self):
        print("\n" + "=" * 60)
        print("[1.LOAD] USD 로드")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()
        print(f"  [OK] {USD_PATH}")

    def _discover_links(self):
        print("\n" + "=" * 60)
        print("[2.DISCOVER] 링크 경로 탐색")
        print("=" * 60)
        self._ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found")
        print(f"  EE ({EE_LINK_NAME}) = {self._ee_path}")
        for jn in GRIPPER_JOINTS:
            print(f"  {jn:<35} = {find_prim_path_by_name(ROBOT_PRIM_PATH, jn)}")

    def _setup_physics(self):
        print("\n" + "=" * 60)
        print("[3.PHYSICS] 물리 설정")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()

        drive_count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    drive_count += 1
        print(f"  [OK] drive updated: {drive_count}")

    def _register_robot(self, scene):
        print("\n" + "=" * 60)
        print("[4.REGISTER] 로봇 등록")
        print("=" * 60)
        gripper = ParallelGripper(
            end_effector_prim_path=self._ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array(GRIPPER_OPEN),
            joint_closed_positions=np.array(GRIPPER_CLOSE),
            action_deltas=np.array(GRIPPER_DELTA),
        )
        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=self._ee_path,
                gripper=gripper,
            )
        )
        print(f"  [OK] SingleManipulator: {ROBOT_PRIM_PATH}")

    def _create_scene(self, scene):
        print("\n" + "=" * 60)
        print("[5.SCENE] 작업 환경 구성 (큐브 2개 + 마커 2개)")
        print("=" * 60)

        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC,
            dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )

        # ── 파란/초록 큐브를 공중 대기 위치에 스폰 ────────────
        for cid, tag in [(1, "blue"), (2, "green")]:
            cube = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/cube_{tag}",
                    name=f"cube_{tag}",
                    position=STANDBY_POS[cid],
                    scale=np.array([CUBE_SIZE] * 3),
                    color=COLOR_RGB[cid],
                    mass=0.05,
                    physics_material=cube_material,
                )
            )
            self.cubes[cid] = cube
            print(f"  [OK] cube_{tag} (color_id={cid}) @ {STANDBY_POS[cid]} (공중 대기)")

        # ── 색상별 Place 마커 ─────────────────────────────────
        for cid, tag in [(1, "blue"), (2, "green")]:
            scene.add(
                VisualCuboid(
                    prim_path=f"/World/marker_{tag}",
                    name=f"marker_{tag}",
                    position=PLACE_POS[cid],
                    scale=np.array([0.07, 0.07, 0.001]),
                    color=COLOR_RGB[cid],
                )
            )
            print(f"  [OK] marker_{tag} @ {PLACE_POS[cid]}")

        # ── 그리퍼 손가락 마찰 ────────────────────────────────
        finger_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/finger_material",
            static_friction=FINGER_STATIC,
            dynamic_friction=FINGER_DYNAMIC,
            restitution=0.0,
        )
        for link_name in ["left_inner_finger", "right_inner_finger"]:
            link_path = find_prim_path_by_name(ROBOT_PRIM_PATH, link_name)
            if link_path:
                SingleGeometryPrim(
                    prim_path=link_path,
                    name=f"{link_name}_geom",
                ).apply_physics_material(finger_material)
                print(f"  [OK] friction: {link_path}")

    # ── 큐브 상태 제어 ───────────────────────────────────────
    @staticmethod
    def _stop(cube):
        """속도 0으로 정지 (물리 뷰 미초기화 시엔 조용히 통과).

        ※ 물리가 꺼진(eDISABLE_SIMULATION) 상태에서 호출하면
          PhysX 가 에러를 뱉으므로 반드시 켜진 상태에서만 부를 것.
        """
        try:
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))
        except Exception:
            pass

    def park_all_cubes(self):
        """두 큐브 모두 공중 대기 상태(물리 OFF)로 되돌린다."""
        for cid, cube in self.cubes.items():
            cube.set_world_pose(position=STANDBY_POS[cid])
            cube.disable_rigid_body_physics()

    def drop_cube(self, cid, position):
        """선택된 큐브를 pick 영역에 놓고 물리를 켠다."""
        cube = self.cubes[cid]
        cube.enable_rigid_body_physics()
        cube.set_world_pose(position=position)
        self._stop(cube)        # 물리를 켠 뒤에 잔여 속도 제거

    def get_cube_position(self, cid):
        pos, _ = self.cubes[cid].get_world_pose()
        return pos

    def get_observations(self):
        obs = {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            }
        }
        for cid, cube in self.cubes.items():
            pos, _ = cube.get_world_pose()
            obs[cube.name] = {"position": pos, "color_id": cid}
        return obs

    def post_reset(self):
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )
        self.park_all_cubes()


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 메인 — 랜덤 스폰 → 색상 감지 → 색상별 Place               ║
# ╚══════════════════════════════════════════════════════════════╝

def color_sequence(rng):
    """스폰할 색상을 무한히 내놓는 제너레이터.

    순수 랜덤(rng.choice)은 같은 색이 10번 넘게 연속으로 나올 수 있어
    시연 중 "한 색만 나온다"처럼 보인다.
    → [파랑, 초록] 한 쌍을 섞어서 내보내 균등 + 최대 2연속으로 제한한다.
    """
    while True:
        pair = [1, 2]
        rng.shuffle(pair)
        for cid in pair:
            yield cid


def ee_over_cube(robot, task, cid):
    """손목 카메라(EE)가 대상 큐브 바로 위에 있는가?

    홈 자세나 Place 마커 위에서는 마커 색이 화면을 채워 오판할 수 있으므로,
    EE 가 큐브 수평 반경 안에 들어왔을 때의 판정만 신뢰한다.
    """
    ee_pos, _ = robot.end_effector.get_world_pose()
    cube_pos = task.get_cube_position(cid)
    return np.linalg.norm(ee_pos[:2] - cube_pos[:2]) < DETECT_EE_RADIUS


def random_pick_position(rng):
    return np.array([
        rng.uniform(*PICK_X_RANGE),
        rng.uniform(*PICK_Y_RANGE),
        CUBE_Z,
    ])


def main():
    rng = np.random.default_rng()
    color_picker = color_sequence(rng)   # 파랑/초록 균등 스폰

    # ── C-1. World + Task ─────────────────────────────────────
    my_world = World(stage_units_in_meters=1.0)
    task = M0609ColorTask(name="m0609_color_task")
    my_world.add_task(task)
    my_world.reset()

    # ── C-2. /color_id 구독 그래프 생성 ───────────────────────
    print("\n" + "=" * 60)
    print("[C-2] ROS2 색상 구독 그래프 생성")
    print("=" * 60)
    build_color_id_graph()
    color_node = ColorIdReader()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)
    task.park_all_cubes()

    for _ in range(30):
        my_world.step(render=True)

    # ── C-3. Controller 생성 ──────────────────────────────────
    print("\n" + "=" * 60)
    print("[C-3] PickPlaceController 생성")
    print("=" * 60)
    controller = PickPlaceController(
        name="m0609_pick_place_controller",
        gripper=robot.gripper,
        robot_articulation=robot,
        end_effector_initial_height=0.30,
        events_dt=EVENTS_DT,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )
    print("  [OK] PickPlaceController 생성 완료")

    # 관측 자세 이동용 (Pick 전에 손목 카메라를 큐브 위로 보낸다)
    obs_controller = RMPFlowController(
        name="m0609_observe_controller",
        robot_articulation=robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )
    print("  [OK] 관측용 RMPFlowController 생성 완료")

    print("\n" + "=" * 60)
    if _args.autoplay:
        print("[준비 완료] --autoplay : 자동으로 시작합니다.")
        my_world.play()
    else:
        print("[준비 완료] Isaac Sim 의 ▶ Play 를 누르면 시작합니다.")
    print(f"  - PC B 에서 'ros2 run M0609 m0609_color_detector' 실행")
    print(f"  - 수신 토픽 : /{COLOR_ID_TOPIC} (1=파랑 / 2=초록)")
    print("=" * 60 + "\n")

    # ── C-4. 상태 머신 루프 ───────────────────────────────────
    SPAWN, SETTLE, OBSERVE, PICK, FINISH = "SPAWN", "SETTLE", "OBSERVE", "PICK", "FINISH"

    was_playing = False
    state = SPAWN
    timer = 0
    round_no = 0
    target_cid = 0          # 실제로 놓인 큐브 (정답)
    detected_cid = 0        # ROS 로 받은 판정값
    place_pos = PLACE_POS[1]
    color_locked = False

    while simulation_app.is_running():
        my_world.step(render=True)
        color_node.poll()          # OmniGraph 구독값 읽기
        time.sleep(0.005)

        is_playing = my_world.is_playing()

        # ── Play 시작 감지 → 전체 리셋 ────────────────────────
        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)
            controller.reset()
            obs_controller.reset()
            task.park_all_cubes()
            color_node.clear()
            state, timer, round_no = SPAWN, 0, 0
            print("\n[RESET] Play 감지 — 시나리오 시작\n")

        was_playing = is_playing
        if not is_playing:
            continue

        # ── 1) SPAWN : 큐브 하나를 랜덤 위치에 낙하 ───────────
        if state == SPAWN:
            round_no += 1
            if ROUNDS and round_no > ROUNDS:
                print(f"\n[종료] {ROUNDS} 라운드 완료")
                my_world.pause()
                continue

            task.park_all_cubes()
            target_cid = next(color_picker)
            spawn_pos = random_pick_position(rng)
            task.drop_cube(target_cid, spawn_pos)

            controller.reset()
            obs_controller.reset()
            color_node.clear()
            detected_cid = 0
            color_locked = False
            timer = 0
            state = SETTLE

            print("\n" + "=" * 60)
            print(f"[ROUND {round_no}] 스폰 : {COLOR_NAME[target_cid]} 큐브 "
                  f"@ ({spawn_pos[0]:.3f}, {spawn_pos[1]:.3f})")
            print("=" * 60)

        # ── 2) SETTLE : 낙하 안정화 대기 ──────────────────────
        elif state == SETTLE:
            timer += 1
            if timer >= SETTLE_STEPS:
                timer = 0
                state = OBSERVE
                print("  [SETTLE] 안정화 완료 → 관측 자세로 이동")

        # ── 3) OBSERVE : 큐브 위로 이동해 색상 확정 ────────────
        #      /color_id 를 받을 때까지 Pick 을 시작하지 않는다.
        #      (PC B 의 감지 노드가 없으면 여기서 계속 대기)
        elif state == OBSERVE:
            timer += 1
            cube_position = task.get_cube_position(target_cid)
            look_at = cube_position + OBSERVE_OFFSET

            actions = obs_controller.forward(
                target_end_effector_position=look_at,
                target_end_effector_orientation=EE_DOWN_QUAT,
            )
            robot.apply_action(actions)

            over_cube = ee_over_cube(robot, task, target_cid)

            if color_node.stable != 0 and over_cube:
                detected_cid = color_node.stable
                place_pos = PLACE_POS[detected_cid]
                state = PICK
                timer = 0
                print(f"  [DETECT] color_id={detected_cid} ({COLOR_NAME[detected_cid]}) "
                      f"→ Place {place_pos}")
            elif timer % 150 == 0:
                link = "연결됨" if color_node.connected else "미연결"
                print(f"  [WAIT] 색상 대기 중 … (구독 {link}, "
                      f"수신값={color_node.latest}, EE가 큐브 위={over_cube})")
                if not color_node.connected or color_node.rx_count == 0:
                    print("         → PC B 에서 'ros2 run M0609 m0609_color_detector' "
                          "실행 여부 / ROS_DOMAIN_ID 를 확인하세요.")

        # ── 4) PICK : Pick & Place 실행 ───────────────────────
        elif state == PICK:
            cube_position = task.get_cube_position(target_cid)
            current_joints = robot.get_joint_positions()
            event = controller.get_current_event()

            # event 5(Place 이동) 이전까지는 색상 판정을 계속 갱신
            if not color_locked:
                if event >= COLOR_LOCK_EVENT:
                    if detected_cid == 0:
                        # OBSERVE 를 통과했으면 여기 올 수 없다. 오면 버그.
                        print("\n  [ERROR] 색상 미확정 상태로 Place 단계 진입 — 중단합니다.")
                        my_world.pause()
                        continue
                    color_locked = True
                    print(f"  [LOCK] event={event} 색상 확정 : "
                          f"{COLOR_NAME[detected_cid]} → {place_pos}")
                elif (color_node.stable != 0
                      and color_node.stable != detected_cid
                      and ee_over_cube(robot, task, target_cid)):
                    detected_cid = color_node.stable
                    place_pos = PLACE_POS[detected_cid]
                    print(f"  [DETECT] 갱신 color_id={detected_cid} "
                          f"({COLOR_NAME[detected_cid]})")

            actions = controller.forward(
                picking_position=cube_position,
                placing_position=place_pos,
                current_joint_positions=current_joints,
                end_effector_offset=EE_OFFSET,
            )
            robot.apply_action(actions)

            if timer % 30 == 0:
                ee_pos, _ = robot.end_effector.get_world_pose()
                print(f"  [event={event}] cube_z={cube_position[2]:.4f}  "
                      f"ee_z={ee_pos[2]:.4f}  color_id={color_node.latest}")
            timer += 1

            if controller.is_done():
                state = FINISH
                timer = 0

        # ── 5) FINISH : 결과 판정 후 다음 라운드 ──────────────
        elif state == FINISH:
            timer += 1
            if timer == 1:
                cube_pos = task.get_cube_position(target_cid)
                err = np.linalg.norm(cube_pos[:2] - place_pos[:2])
                ok = (detected_cid == target_cid) and (err < 0.08)
                print("-" * 60)
                print(f"  실제 색상 : {COLOR_NAME[target_cid]}")
                print(f"  감지 색상 : {COLOR_NAME[detected_cid]}")
                print(f"  Place 오차: {err:.3f} m")
                print(f"  결과      : {'성공 ✅' if ok else '실패 ❌'}")
                print("-" * 60)
            if timer >= 60:
                state = SPAWN

    simulation_app.close()


if __name__ == "__main__":
    main()
