"""[Isaac 3.11] 로봇 상태 발행 — 휠 속도 · 관절 각도 · IMU 를 ROS 2 로 내보낸다.

추측 항법(localization/deadreckon.py)이 요구하는 신호를 여기서 만든다.
설계 5.2 의 신호 구성 그대로다.

    바퀴 엔코더 6개   →  /wheel_speed (평균)  /wheel_speeds (개별 6)
    관절 엔코더       →  /joint_angle          곡관 진입·이탈과 회전 방향
    IMU 중력          →  /imu_roll             결함의 원주 위치. 링 J1 을 어디로
    IMU 요 각속도     →  /imu_yaw_rate         회전량 적분, 관절 각도 교차 검증
    서스펜션 스트로크 →  /suspension           관경 변화·이물 (보조)

**IMU 가 없으면 롤을 모르고, 롤을 모르면 결함이 관의 어느 시계 방향에 있는지
모른다.** 그러면 토치 링 J1 을 어디로 돌릴지 정할 수 없다.

기준 세그먼트는 후방으로 통일한다(설계 5.1). 로봇이 어떻게 투입되었든 중력은
변하지 않으므로 정찰기와 수리기가 같은 롤 기준을 공유한다.

카메라와 같은 방식으로 쓴다 — 시뮬 루프를 도는 쪽에서 import 해 attach 한다.

실행:
  PYTHONUNBUFFERED=1 isaac_python state_bridge.py          # 단독 확인
  (보통은 pipe/curve_demo.py --state 가 import 해서 쓴다)
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pyver import require_isaac

require_isaac(__file__, needs_rclpy=True)

HEADLESS = "--headless" in sys.argv
ROBOT = "/World/Robot"
NS = ""
if "--robot" in sys.argv:
    ROBOT = sys.argv[sys.argv.index("--robot") + 1]
if "--ns" in sys.argv:
    NS = sys.argv[sys.argv.index("--ns") + 1].rstrip("/")

from isaacsim import SimulationApp

# 다른 스크립트가 import 할 때는 그쪽이 이미 SimulationApp 을 만든 상태다.
simulation_app = None
if __name__ == "__main__":
    simulation_app = SimulationApp({"headless": HEADLESS})

import json
import math
import time
from pathlib import Path

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.sensors.physics import IMUSensor

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

HERE = Path(__file__).resolve().parent
SON = HERE.parent
META = json.loads((SON / "spec" / "parts_meta.json").read_text())

MM = 0.001
WHEEL_R_M = META["wheel_r"] * MM
ARM_LEN_MM = META["arm_len"]
PSI0 = META["arm_angle_nominal"]

IMU_PRIM = "imu"
IMU_LINK = "body_rear"          # 설계 5.1 — 기준 세그먼트는 후방으로 통일


def attach_imu(robot_path=ROBOT, link=IMU_LINK, frequency=200):
    """후방 세그먼트에 IMU 를 붙인다. 없으면 롤을 못 구한다."""
    imu = IMUSensor(prim_path=f"{robot_path}/{link}/{IMU_PRIM}",
                    name="pipe_imu",
                    frequency=frequency,
                    translation=np.array([0.0, 0.0, 0.0]))
    imu.initialize()
    return imu


def roll_from_gravity(lin_acc):
    """중력 방향으로 절대 롤을 구한다.

    로봇 로컬 X 가 관 축이므로 중력의 YZ 성분이 '아래쪽'을 가리킨다.
    로봇이 어떻게 투입되었든 중력은 변하지 않아 기준이 흔들리지 않는다.

    🚨 부호 규약 확정(2026-08-04 실측). Isaac Sim 의 `lin_acc` 는 **고유가속도**
    규약이라 정지 상태에서 위쪽으로 +9.81 을 읽는다 — 중력 벡터가 아니다.
    롤 θ 로 굴리면 lin_acc = (0, g·sinθ, g·cosθ) 이므로 θ = atan2(gy, gz) 다.
    z 에 음수를 붙이면 180°−θ 가 되어 **정립 상태에서 180° 를 보고한다.**
    그러면 토치 링 J1 을 정반대로 돌려 엉뚱한 곳을 용접한다.

    월드 고정 조인트로 정지시켜 실측한 값(자유낙하에서는 가속도계가 0 을 읽어
    측정 자체가 안 된다):

        적용 롤   lin_acc                atan2(gy,-gz) 옛   atan2(gy, gz) 지금
          0°     (0,  0.000, +9.810)        +180.00°            0.00°
        +30°     (0, +4.905, +8.496)        +150.00°          +30.00°
        -45°     (0, -6.937, +6.937)        -135.00°          -45.00°
        +90°     (0, +9.810,  0.000)         +90.00°          +90.00°
    """
    a = np.asarray(lin_acc, dtype=float).reshape(-1)
    if a.size < 3:
        return 0.0
    gy, gz = float(a[1]), float(a[2])
    if abs(gy) < 1e-6 and abs(gz) < 1e-6:
        return 0.0
    return math.degrees(math.atan2(gy, gz))


class RobotStateBridge(Node):
    def __init__(self, art, imu=None, hz=50.0):
        super().__init__("robot_state_bridge")
        self.art = art
        self.imu = imu
        self.hz = float(hz)
        pre = f"{NS}/" if NS else ""
        self.pub_wheel = self.create_publisher(Float32, f"{pre}wheel_speed", 10)
        self.pub_wheels = self.create_publisher(Float32MultiArray,
                                                f"{pre}wheel_speeds", 10)
        self.pub_joint = self.create_publisher(Float32, f"{pre}joint_angle", 10)
        self.pub_roll = self.create_publisher(Float32, f"{pre}imu_roll", 10)
        self.pub_yaw = self.create_publisher(Float32, f"{pre}imu_yaw_rate", 10)
        self.pub_susp = self.create_publisher(Float32MultiArray,
                                              f"{pre}suspension", 10)

        dof = list(art.dof_names or [])
        self.i_wheel = [k for k, n in enumerate(dof)
                        if n.startswith("joint_wheel_")]
        self.i_arm = [k for k, n in enumerate(dof)
                      if n.startswith("joint_arm_")]
        self.i_waist = [k for k, n in enumerate(dof)
                        if n.startswith("joint_waist")]
        self.n = 0
        self._logged = False
        self.get_logger().info(
            f"상태 발행 — 휠 {len(self.i_wheel)} / 암 {len(self.i_arm)} / "
            f"관절 {len(self.i_waist)}, IMU {'있음' if imu else '없음'}")
        if len(self.i_wheel) != 6:
            self.get_logger().warn(
                f"휠 조인트가 {len(self.i_wheel)}개다 — 6개여야 한다")

    def publish(self):
        vel = self.art.get_joint_velocities()
        pos = self.art.get_joint_positions()

        # 휠 각속도(rad/s) × 반경 = 접지 속도. 6개 평균이 주 신호,
        # 개별 값은 편차로 슬립을 보는 데 쓴다(설계 5.2).
        ws = [float(vel[k]) * WHEEL_R_M for k in self.i_wheel]
        if ws:
            self.pub_wheel.publish(Float32(data=float(np.mean(ws))))
            self.pub_wheels.publish(Float32MultiArray(data=ws))

        if self.i_waist:
            self.pub_joint.publish(Float32(
                data=float(math.degrees(pos[self.i_waist[0]]))))

        # 암 각도 → 반경 방향 스트로크(mm). 관경 변화·이물 감지용 보조 신호.
        if self.i_arm:
            st = [ARM_LEN_MM * (math.sin(math.radians(
                PSI0 + math.degrees(float(pos[k])))) -
                math.sin(math.radians(PSI0))) for k in self.i_arm]
            self.pub_susp.publish(Float32MultiArray(data=st))

        if self.imu is not None:
            fr = self.imu.get_current_frame(read_gravity=True)
            if not self._logged:
                self.get_logger().info(f"IMU 프레임 키: {sorted(fr.keys())}")
                self._logged = True
            acc = fr.get("lin_acc")
            if acc is None:
                acc = fr.get("linear_acceleration")
            ang = fr.get("ang_vel")
            if ang is None:
                ang = fr.get("angular_velocity")
            if acc is not None:
                self.pub_roll.publish(Float32(
                    data=float(roll_from_gravity(acc))))
            if ang is not None:
                a = np.asarray(ang, dtype=float).reshape(-1)
                # 관 축이 X 이므로 요(진행 방향 전환)는 월드 Z 회전이다.
                if a.size >= 3:
                    self.pub_yaw.publish(Float32(
                        data=float(math.degrees(a[2]))))
        self.n += 1


def attach(art, hz=50.0, robot_path=ROBOT, with_imu=True, imu=None):
    """시뮬 루프를 도는 쪽에서 호출한다. rclpy 는 호출자가 init 해 둘 것.

    🚨 `imu` 는 **world.reset() 전에** 만들어 여기로 넘겨야 한다. reset 뒤에
    붙이면 센서가 한 번도 갱신되지 않는다 — physics_step 이 0 에 머물고
    lin_acc 가 [0,0,0] 이라 /imu_roll, /imu_yaw_rate 가 항상 0 으로 나간다.

        부착 시점        physics_step   lin_acc
        reset 뒤 (옛)        0          [0,0,0]
        reset 앞 (지금)     52          값이 나온다

    롤을 모르면 결함의 원주 위치를 모르고 토치 링 J1 을 어디로 돌릴지 정할 수
    없다. 2026-08-04 실측으로 확인됐다.
    """
    if imu is None and with_imu:
        print("[경고] reset() 이후에 IMU 를 만들면 갱신되지 않는다. "
              "reset() 전에 attach_imu() 를 호출해 imu= 로 넘길 것")
        try:
            imu = attach_imu(robot_path)
        except Exception as exc:
            print(f"[경고] IMU 생성 실패 ({exc}) — 롤·요를 발행하지 않는다")
    return RobotStateBridge(art, imu, hz)


def main():
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    usd = SON / "robot" / "robot_2seg.usd"
    if not usd.is_file():
        print(f"[중단] {usd} 가 없다. robot/articulate.py 를 먼저 실행할 것.")
        simulation_app.close()
        return 1

    from pxr import UsdGeom
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, ROBOT).GetPrim().GetReferences() \
        .AddReference(str(usd), "/World/Robot")

    art = SingleArticulation(prim_path=ROBOT, name="pipe_robot_state")
    world.scene.add(art)

    # IMU 는 reset() 전에 붙여야 갱신된다 (attach() 주석 참조)
    imu = None
    try:
        imu = attach_imu(ROBOT)
    except Exception as exc:
        print(f"[경고] IMU 생성 실패 ({exc}) — 롤·요를 발행하지 않는다")

    world.reset()
    for _ in range(30):
        world.step(render=not HEADLESS)

    rclpy.init()
    bridge = attach(art, imu=imu)

    # 바퀴를 돌려 발행값이 실제로 움직이는지 본다.
    from pxr import UsdPhysics
    dof = list(art.dof_names or [])
    for k in bridge.i_wheel:
        UsdPhysics.DriveAPI.Get(
            stage.GetPrimAtPath(f"{ROBOT}/{dof[k]}"), "angular"
        ).GetTargetVelocityAttr().Set(180.0)

    print("\n" + "=" * 74)
    print("로봇 상태 발행 확인")
    print("=" * 74)
    period = 1.0 / bridge.hz
    last = 0.0
    t0 = time.time()
    try:
        while simulation_app.is_running() and time.time() - t0 < 8.0:
            world.step(render=not HEADLESS)
            now = time.time()
            if now - last >= period:
                last = now
                bridge.publish()
                rclpy.spin_once(bridge, timeout_sec=0.0)
                if bridge.n % 25 == 0:
                    vel = art.get_joint_velocities()
                    ws = [float(vel[k]) * WHEEL_R_M for k in bridge.i_wheel]
                    pos = art.get_joint_positions()
                    j = (math.degrees(pos[bridge.i_waist[0]])
                         if bridge.i_waist else 0.0)
                    line = (f"  휠 평균 {np.mean(ws):+.4f} m/s  "
                            f"편차 {np.std(ws):.4f}  관절 {j:+.2f}°")
                    if bridge.imu is not None:
                        fr = bridge.imu.get_current_frame(read_gravity=True)
                        acc = fr.get("lin_acc", fr.get("linear_acceleration"))
                        if acc is not None:
                            line += f"  롤 {roll_from_gravity(acc):+.1f}°"
                    print(line)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n  총 {bridge.n} 회 발행")
        print("  → /wheel_speed /wheel_speeds /joint_angle "
              "/imu_roll /imu_yaw_rate /suspension")
        print("=" * 74)
        bridge.destroy_node()
        rclpy.shutdown()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
