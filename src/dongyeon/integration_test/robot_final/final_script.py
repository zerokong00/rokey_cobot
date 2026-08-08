"""
=====================================================================
 PIPE ROBOT  -  junction steering, attitude-independent
=====================================================================

WHAT CHANGED FROM THE VERSION THAT WORKED

  Only the Roll command.  Everything about the bend schedule - the front
  release delay, the rear soft bias, the widths, LAG - is untouched.

  Before, Roll was worked out once in _setup() from the robot root's AUTHORED
  transform.  That is where you put the prim on the stage, not where the robot
  actually is.  So the bend plane was locked to the pose you happened to drop
  it in, and if the robot rolled inside the pipe it steered off to the side.
  Turning up instead of down meant physically rotating both pipe and robot
  180 deg about X and flipping BRANCH_DIR.

  Now Roll is recomputed every physics step from the BODY's LIVE orientation:

      bl   = BRANCH_DIR expressed in the body's own frame
      Roll = atan2(bl.y, -bl.z)

  Roll a body 30 deg and the command moves 30 deg to cancel it, so the bend
  plane keeps pointing at the world direction you asked for.  BRANCH_DIR is
  now a pure WORLD vector - the robot's attitude no longer enters into it.

  The tracking stops as soon as the turn actually starts (see ROLL_LOCK_AT).
  It has to: once the body pitches into the branch, "the body's frame" is
  swinging through 90 deg, and chasing it would spin the bend plane round
  mid-manoeuvre.  So it tracks while straight, locks, then executes.

USE
  1. Load robot + tee, place the robot in the main pipe, press PLAY.
  2. Fix ROBOT / PIPE and BRANCH_DIR below, then Run.
  3. selftest()      -> BendF snaps to 40 deg.  Nothing kinking on screen means
                        the writes are not reaching physics.
     reset()         -> straighten.
     go()            -> starts the wheels and runs the whole manoeuvre.
     where()         -> position, targets and the live roll.  Changes nothing.
     set_branch(0,0,1) -> aim at a different world direction, any time.
     stop()          -> wheels off, joints straight.
=====================================================================
"""

import math
import omni.usd
import omni.physx
from pxr import Usd, UsdGeom, Gf

# ========================= CONFIG ====================================
ROBOT      = "/World/pipe_robot_v6"
PIPE       = "/World/test_pipe_tee_ID100"
JUNCTION   = None                      # or a world (x,y,z) in metres

# WORLD direction the branch runs.  This is the one you change to pick where
# the robot goes, and it no longer depends on how the robot is lying:
#     (0,0,-1) down      (0,0,+1) up
#     (0,+1,0) one side  (0,-1,0) the other
#     (0, 0.7, -0.7) diagonal - any unit-ish vector perpendicular to the pipe
BRANCH_DIR = Gf.Vec3d(0.0, 0.0, -1.0)

# 로봇 진행방향에 수직인 단면 평면
# 0° = RIGHT, +90° = UP, -90° = DOWN
PLANE_UP    = Gf.Vec3d(0.0,  0.0, 1.0)
PLANE_RIGHT = Gf.Vec3d(0.0, -1.0, 0.0)

# Roll tracking.  ROLL_TRACK keeps the bend plane aimed at BRANCH_DIR while
# the robot is still running straight, correcting for however it has rolled.
# ROLL_LOCK_AT is the BendF angle at which tracking freezes and the roll is
# held for the rest of the manoeuvre - past this the body itself is turning,
# and following its frame would swing the bend plane around.
ROLL_TRACK   = True
ROLL_LOCK_AT = 3.0     # deg of BendF

WHEEL_CMD   = 600.0
SPEED_SCALE = 0.90

LAG_MM     = 48.0
TURN_FRONT = 56.0
TURN_BODY  = 56.0
TURN_REAR  = 90.0

FRONT_RELEASE_DELAY_MM = 10.0

REAR_FREE       = True
REAR_BIAS_DEG   = 10.0
REAR_BIAS_K     = 1.50
REAR_DAMP       = 0.30
REAR_MF         = 1.00

TOTAL_BEND = 90.0
BEND_MF    = 2.5
ROLL_MF    = 2.0
VERBOSE    = True
# =====================================================================

WHEEL_R = 0.008
_sub, _S = None, {}


def _ss(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def _angles(s):
    """Axis angle of each of the three segments, in degrees."""
    L = LAG_MM / 1000.0

    def A(x, w_mm):
        w = w_mm / 1000.0
        return TOTAL_BEND * _ss((x + w / 2.0) / w)
    return (A(s, TURN_FRONT), A(s - L, TURN_BODY), A(s - 2 * L, TURN_REAR))


def _targets(s):
    tF, tB, tR = _angles(s)
    delay = FRONT_RELEASE_DELAY_MM / 1000.0
    _, tB_front, _ = _angles(s - delay)
    return [tF - tB_front, -(tB - tR)]


# ---------------- live pose / attitude -------------------------------
def _view(expr, name):
    for mod, cls in (("isaacsim.core.prims", "RigidPrim"),
                     ("omni.isaac.core.prims", "RigidPrimView")):
        try:
            m = __import__(mod, fromlist=[cls])
            v = getattr(m, cls)(prim_paths_expr=expr, name=name)
            try:
                v.initialize()
            except Exception:
                pass
            p, q = v.get_world_poses()
            if p is not None and len(p):
                return v, "%s.%s" % (mod, cls)
        except Exception:
            pass
    return None, None


def _make_readers():
    """head position, and body position+rotation, from the physics view."""
    hv, hl = _view("%s/Robot/DiscF" % ROBOT, "v8_head")
    bv, bl = _view("%s/Robot/Body" % ROBOT, "v8_body")

    head = None
    if hv is not None:
        def head(v=hv):
            p, _ = v.get_world_poses()
            return Gf.Vec3d(float(p[0][0]), float(p[0][1]), float(p[0][2]))

    body = None
    if bv is not None:
        def body(v=bv):
            p, q = v.get_world_poses()
            w, x, y, z = (float(q[0][0]), float(q[0][1]),
                          float(q[0][2]), float(q[0][3]))
            rot = Gf.Matrix3d(Gf.Rotation(Gf.Quatd(w, Gf.Vec3d(x, y, z))))
            return Gf.Vec3d(float(p[0][0]), float(p[0][1]), float(p[0][2])), rot

    if head is None:
        try:
            from usdrt import Usd as RtUsd
            rt = RtUsd.Stage.Attach(omni.usd.get_context().get_stage_id())
            pr = rt.GetPrimAtPath("%s/Robot/DiscF" % ROBOT)
            if pr.IsValid():
                def head(p=pr):
                    v = p.GetAttribute("_worldPosition").Get()
                    return Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]))
                hl = "usdrt fabric"
        except Exception:
            pass
    return head, hl, body, bl


def _body_axes():
    """Body local axes in world.  Falls back to the authored transform when
    there is no live source - in that case roll tracking is static."""
    if _S.get("bodyread") is not None:
        try:
            _, rot = _S["bodyread"]()
            return [Gf.Vec3d(rot[i][0], rot[i][1], rot[i][2]).GetNormalized()
                    for i in range(3)]
        except Exception:
            pass
    return _S["ax0"]


def _roll_target():
    """현재 BODY 자세에서 BRANCH_DIR을 향하도록 Roll과 Bend 부호를 계산한다.

    핵심:
      - Roll은 항상 -90 ~ +90 deg 범위로 제한
      - 반대편 반원은 180 deg Roll 대신 Bend 방향(sign)을 뒤집어서 처리

    따라서 기본 자세 기준:
      -90 deg (DOWN) -> roll 0, sign +1
      +90 deg (UP)   -> roll 0, sign -1
    """
    ax = _body_axes()
    b = Gf.Vec3d(*BRANCH_DIR).GetNormalized()

    bl = Gf.Vec3d(
        Gf.Dot(b, ax[0]),
        Gf.Dot(b, ax[1]),
        Gf.Dot(b, ax[2])
    )

    if math.hypot(bl[1], bl[2]) < 1e-6:
        return _S.get("roll", 0.0), _S.get("sign", 1.0)

    roll = math.degrees(math.atan2(bl[1], -bl[2]))
    sign = 1.0

    if roll > 90.0:
        roll -= 180.0
        sign = -1.0
    elif roll < -90.0:
        roll += 180.0
        sign = -1.0

    return roll, sign


def _setup():
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(ROBOT)
    if not root.IsValid():
        raise RuntimeError("ROBOT not found: %s" % ROBOT)

    if JUNCTION is not None:
        jp = Gf.Vec3d(*JUNCTION)
    else:
        pp = stage.GetPrimAtPath(PIPE)
        if not pp.IsValid():
            raise RuntimeError("PIPE not found: %s (or set JUNCTION)" % PIPE)
        jp = UsdGeom.Xformable(pp).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()

    rot = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractRotationMatrix()
    ax0 = [Gf.Vec3d(rot[i][0], rot[i][1], rot[i][2]).GetNormalized()
           for i in range(3)]
    travel = ax0[0]

    bends, rolls = [], []
    for tag in ("F", "R"):
        bp = stage.GetPrimAtPath("%s/Robot/SteerJoints/Bend%s" % (ROBOT, tag))
        rp = stage.GetPrimAtPath("%s/Robot/SteerJoints/Roll%s" % (ROBOT, tag))
        if not bp.IsValid() or not rp.IsValid():
            raise RuntimeError("steer joint %s missing" % tag)
        bends.append(bp)
        rolls.append(rp)

    wheels = [p for p in stage.Traverse()
              if str(p.GetPath()).startswith("%s/Robot/DriveJoints/" % ROBOT)]
    if not wheels:
        raise RuntimeError("no drive joints found")

    head = stage.GetPrimAtPath("%s/Robot/DiscF" % ROBOT)
    hp0 = UsdGeom.Xformable(head).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()
    s0 = Gf.Dot(hp0 - jp, travel)

    hread, hlabel, bread, blabel = _make_readers()
    v = WHEEL_CMD * math.pi / 180.0 * WHEEL_R * SPEED_SCALE
    k0 = bends[1].GetAttribute("drive:angular:physics:stiffness").Get() or 25.0
    d0 = bends[1].GetAttribute("drive:angular:physics:damping").Get() or 3.0

    _S.clear()
    _S.update(jp=jp, travel=travel, ax0=ax0, bends=bends, rolls=rolls,
              wheels=wheels, sign=1.0, v=v, s0=s0, t=None, last=-1.0,
              shift=0.0, reader=hread, label=hlabel, bodyread=bread,
              bodylabel=blabel, mode="pending", p0=None, arc=None, prev=None,
              done=False, bend_k=k0, bend_d=d0, roll=0.0, roll_locked=False)
    _S["roll"], _S["sign"] = _roll_target()

    if VERBOSE:
        print("[v8] head pose  :", hlabel or "NONE (odometry only)")
        print("[v8] body attitude:", blabel or "NONE - roll tracking is STATIC")
        print("[v8] head starts %+.0f mm from the junction" % (s0 * 1000))
        print("[v8] BRANCH_DIR (world) %s -> Roll %.1f deg, bend sign %+d"
              % ([round(q, 2) for q in BRANCH_DIR], _S["roll"], int(_S["sign"])))
        print("[v8] roll tracking %s, locks once BendF passes %.0f deg"
              % ("ON" if ROLL_TRACK else "OFF", ROLL_LOCK_AT))
        pf = max(abs(_targets(q / 1000.0)[0]) for q in range(-150, 400))
        print("[v8] peak BendF %.1f deg | front release delay %.0f mm"
              % (pf, FRONT_RELEASE_DELAY_MM))
        print("[v8] ready.  selftest() -> reset() -> go()")


def _set_bend(prim, deg, mf=None):
    a = prim.GetAttribute("drive:angular:physics:targetPosition")
    if a.IsValid():
        a.Set(float(deg))
    if mf is not None:
        f = prim.GetAttribute("drive:angular:physics:maxForce")
        if f.IsValid():
            f.Set(float(mf))


def _drive(prim, stiffness, damping, maxforce):
    for n, val in (("stiffness", stiffness), ("damping", damping),
                   ("maxForce", maxforce)):
        a = prim.GetAttribute("drive:angular:physics:%s" % n)
        if a.IsValid():
            a.Set(float(val))


def _configure_rear():
    r = _S["bends"][1]
    if REAR_FREE:
        _set_bend(r, -REAR_BIAS_DEG * _S["sign"])
        _drive(r, REAR_BIAS_K, REAR_DAMP, REAR_MF)
    else:
        _drive(r, _S["bend_k"], _S["bend_d"], BEND_MF)


def _apply_roll():
    for p in _S["rolls"]:
        _set_bend(p, _S["roll"], mf=ROLL_MF)


def _set_wheels(vel):
    for p in _S["wheels"]:
        a = p.GetAttribute("drive:angular:physics:targetVelocity")
        if a.IsValid():
            a.Set(float(vel))


def _on_step(dt):
    if not _S or _S.get("t") is None or _S.get("done"):
        return
    _S["t"] += dt
    s = _S["s0"] + _S["v"] * _S["t"] + _S["shift"]
    if _S["mode"] == "live":
        try:
            now = _S["reader"]()
            if _S["arc"] is None:
                _S["arc"] = Gf.Dot(now - _S["jp"], _S["travel"])
            else:
                _S["arc"] += (now - _S["prev"]).GetLength()
            _S["prev"] = now
            s = _S["arc"] + _S["shift"]
        except Exception:
            _S["mode"] = "odometry"
    elif _S["mode"] == "pending":
        if _S["reader"] is None:
            _S["mode"] = "odometry"
            print("[v8] no live pose -> odometry")
        elif _S["p0"] is None:
            _S["p0"] = _S["reader"]()
        elif _S["t"] > 1.0:
            moved = (_S["reader"]() - _S["p0"]).GetLength()
            _S["mode"] = "live" if moved > 0.005 else "odometry"
            print("[v8] poses moved %.0f mm in 1 s -> %s"
                  % (moved * 1000, _S["mode"]))

    tg = _targets(s)

    # Keep the bend plane aimed at BRANCH_DIR while still running straight.
    # Freeze it once the turn starts - past that the body's own frame is
    # swinging through 90 deg and chasing it would spin the plane.
    if ROLL_TRACK and not _S["roll_locked"]:
        if abs(tg[0]) >= ROLL_LOCK_AT:
            _S["roll_locked"] = True
            print("[v8] roll locked at %.1f deg (BendF reached %.1f)"
                  % (_S["roll"], tg[0]))
        else:
            _S["roll"], _S["sign"] = _roll_target()
    _apply_roll()

    _set_bend(_S["bends"][0], tg[0] * _S["sign"])
    if not REAR_FREE:
        _set_bend(_S["bends"][1], tg[1] * _S["sign"])

    if VERBOSE and _S["t"] - _S["last"] >= 0.5:
        _S["last"] = _S["t"]
        rear = "free" if REAR_FREE else "%+6.1f" % tg[1]
        print("[v8] %-9s t=%4.1fs s=%+6.0fmm  BendF %+6.1f  BendR %s  Roll %+6.1f  sign %+d%s"
              % (_S["mode"], _S["t"], s * 1000,
                 tg[0] * _S["sign"], rear, _S["roll"], int(_S["sign"]),
                 " (locked)" if _S["roll_locked"] else ""))

    if s > (2 * LAG_MM + max(TURN_FRONT, TURN_BODY, TURN_REAR)) / 1000.0:
        _S["done"] = True
        _set_bend(_S["bends"][0], 0.0)
        if not REAR_FREE:
            _set_bend(_S["bends"][1], 0.0)
        print("[v8] through the branch - bends released, still driving")


# ------------------------------------------------------------------ API
def set_branch(x, y, z):
    """WORLD 좌표계 기준 분기 방향 설정."""
    global BRANCH_DIR
    BRANCH_DIR = Gf.Vec3d(float(x), float(y), float(z))
    _S["roll"], _S["sign"] = _roll_target()
    _apply_roll()
    print("[v8] branch -> %s (world) | roll %.1f deg, bend sign %+d"
          % ([round(q, 2) for q in BRANCH_DIR],
             _S["roll"], int(_S["sign"])))


def set_branch_xy(x, y):
    """로봇 진행방향에 수직인 단면 좌표로 분기 방향 지정.
       +x = RIGHT, +y = UP
    """
    v = (Gf.Vec3d(*PLANE_RIGHT) * float(x) +
         Gf.Vec3d(*PLANE_UP) * float(y))

    if v.GetLength() < 1e-9:
        print("[v8] zero direction - ignored")
        return

    set_branch(v[0], v[1], v[2])


def set_branch_angle(deg):
    """단면 기준 각도 지정.
       0 = RIGHT, +90 = UP, -90 = DOWN, 180 = LEFT
    """
    a = math.radians(float(deg))
    set_branch_xy(math.cos(a), math.sin(a))
    print("[v8] cross-section angle %.1f deg" % deg)


def plane_check():
    """현재 단면 좌표축과 각도 방향 확인."""
    print("[v8] travel (world)      %s"
          % [round(q, 3) for q in _S["travel"]])
    print("[v8] PLANE_RIGHT (+x)    %s"
          % [round(q, 3) for q in PLANE_RIGHT])
    print("[v8] PLANE_UP    (+y)    %s"
          % [round(q, 3) for q in PLANE_UP])
    print("[v8] BRANCH_DIR now      %s"
          % [round(q, 3) for q in BRANCH_DIR])

    for nm, d in (("right", 0), ("up", 90),
                  ("left", 180), ("down", -90)):
        a = math.radians(d)
        v = (Gf.Vec3d(*PLANE_RIGHT) * math.cos(a) +
             Gf.Vec3d(*PLANE_UP) * math.sin(a))
        print("[v8]   %-5s (%4d deg) -> world %s"
              % (nm, d, [round(q, 2) for q in v]))


def roll_track(on=True):
    """Turn live roll tracking on or off."""
    global ROLL_TRACK
    ROLL_TRACK = bool(on)
    _S["roll_locked"] = False
    print("[v8] roll tracking -> %s" % ("ON" if ROLL_TRACK else "OFF"))


def roll_now():
    """현재 자세에서 필요한 Roll과 Bend 부호만 출력."""
    ax = _body_axes()
    b = Gf.Vec3d(*BRANCH_DIR).GetNormalized()
    bl = Gf.Vec3d(
        Gf.Dot(b, ax[0]),
        Gf.Dot(b, ax[1]),
        Gf.Dot(b, ax[2])
    )
    r, sg = _roll_target()

    print("[v8] body axis (world) %s"
          % [round(q, 3) for q in ax[0]])
    print("[v8] branch in body frame %s -> roll %.1f deg, bend sign %+d"
          % ([round(q, 3) for q in bl], r, int(sg)))


def selftest():
    _S["t"] = None
    _apply_roll()
    _set_bend(_S["bends"][0], 0.0, mf=BEND_MF)
    _configure_rear()
    _set_bend(_S["bends"][0], 40.0 * _S["sign"])
    print("[v8] BendF -> %.0f deg.  LOOK AT THE ROBOT.  reset() when done." % (40.0 * _S["sign"]))


def reset():
    _S["t"] = None
    _S["done"] = False
    _S["roll_locked"] = False
    _S["roll"], _S["sign"] = _roll_target()
    _apply_roll()
    _set_bend(_S["bends"][0], 0.0, mf=BEND_MF)
    _configure_rear()
    print("[v8] straight | roll %.1f deg, bend sign %+d (tracking %s) | rear %s"
          % (_S["roll"], int(_S["sign"]),
             "ON" if ROLL_TRACK else "OFF",
             "SOFT-BIASED" if REAR_FREE else "driven"))


def where():
    if _S.get("reader") is None:
        print("[v8] no live reader")
        return
    cur = Gf.Dot(_S["reader"]() - _S["jp"], _S["travel"])
    s = (_S["arc"] if _S["arc"] is not None else cur) + _S.get("shift", 0.0)
    tg = _targets(s)
    print("[v8] head proj %+.0f mm | schedule s %+.0f mm | mode %s"
          % (cur * 1000, s * 1000, _S.get("mode")))
    print("[v8] BendF %+.1f  BendR %s  Roll %+.1f  sign %+d%s"
          % (tg[0] * _S["sign"],
             "free" if REAR_FREE else "%+.1f" % (tg[1] * _S["sign"]),
             _S["roll"], int(_S["sign"]),
             " (locked)" if _S["roll_locked"] else ""))
    print("[v8] segment angles  front %.0f  body %.0f  rear %.0f deg" % _angles(s))
    roll_now()


def nudge(mm):
    _S["shift"] = _S.get("shift", 0.0) + mm / 1000.0
    print("[v8] shift %+0.0f mm (total %+0.0f)" % (mm, _S["shift"] * 1000))


def go():
    reset()
    _S["t"] = 0.0
    _S["last"] = -1.0
    _S["shift"] = 0.0
    _S["mode"] = "pending"
    _S["p0"] = None
    _S["arc"] = None
    _S["done"] = False
    _set_wheels(WHEEL_CMD)
    print("[v8] wheels %.0f deg/s, head %+.0f mm out, schedule armed"
          % (WHEEL_CMD, _S["s0"] * 1000))


def stop():
    _set_wheels(0.0)
    _S["t"] = None
    _set_bend(_S["bends"][0], 0.0)
    if not REAR_FREE:
        _set_bend(_S["bends"][1], 0.0)
    print("[v8] stopped")


def v8_stop():
    global _sub
    if _sub is not None:
        try:
            _sub.unsubscribe()
        except Exception:
            pass
        _sub = None
        print("[v8] callback removed")


def v8_start():
    global _sub
    v8_stop()
    _setup()
    _sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)


v8_start()
plane_check()

# +90° = 위쪽
set_branch_angle(90.0)
go()
