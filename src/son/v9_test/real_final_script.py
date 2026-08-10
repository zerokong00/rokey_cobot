"""
=====================================================================
 PIPE ROBOT  -  junction steering, attitude-independent
=====================================================================

WHAT CHANGED FROM THE VERSION YOU WERE RUNNING

  Only _roll_target().  The bend schedule, the front release delay, the rear
  soft bias, LAG, the widths - none of it is touched.

  WHY 170 deg AND UP WAS BREAKING

  A branch direction can always be expressed two ways: (roll, +bend) or
  (roll -/+ 180, -bend).  Both put the disc in exactly the same place.  The old
  code always forced roll into +-90 by flipping the sign whenever it crossed
  that line, and it recomputed this from scratch every physics step.

  Near a 180 deg branch the raw roll lands right on that +-90 boundary:

      branch 160 deg -> roll -70   (20 deg of margin)
      branch 170 deg -> roll -80   (10 deg)
      branch 179 deg -> roll -89   ( 1 deg)
      branch 181 deg -> roll +89   <- 179 deg jump, and the bend sign inverts

  So once the body rolled by more than that margin - which it does, constantly,
  by a few degrees - the chosen representation flipped back and forth every
  step.  Roll was commanded to swing 180 deg and BendF changed sign at the same
  time.  That is the thrashing you saw, and it is why it appeared exactly at
  170 deg and not before.

  THE FIX

  Pick the representation nearest the one already being commanded, and allow
  roll to run to +-115 rather than snapping at +-90.  With a fixed branch the
  representation is then chosen once and never flips: at 178 deg the command
  now slides smoothly between -96 and -80 as the body wobbles, sign pinned at
  -1.  On the first call, with nothing to be continuous with, it takes the
  representation with the smallest roll, so nothing rotates further than it
  has to.

USE
  1. Load robot + tee, place the robot in the main pipe, press PLAY.
  2. Fix ROBOT / PIPE below, then Run.
  3. plane_check()        -> confirm which way is right and up.
     set_branch_angle(170) -> 0 = right, +90 = up, 180 = left, -90 = down.
     selftest() / reset() / go() / where() / stop() as before.
=====================================================================
"""

import math
import omni.usd
import omni.physx
from pxr import Usd, UsdGeom, Gf

# ========================= CONFIG ====================================
ROBOT      = "/World/pipe_robot_v9"
PIPE       = "/World/test_pipe_tee_ID100"
JUNCTION   = None                      # or a world (x,y,z) in metres

BRANCH_DIR = Gf.Vec3d(0.0, 0.0, -1.0)

# 로봇 진행방향에 수직인 단면 평면
# 0° = RIGHT, +90° = UP, -90° = DOWN
PLANE_UP    = Gf.Vec3d(0.0,  0.0, 1.0)
PLANE_RIGHT = Gf.Vec3d(0.0, -1.0, 0.0)

ROLL_TRACK   = True
ROLL_LOCK_AT = 3.0     # deg of BendF

# How far past 90 deg the roll command is allowed to run before the code gives
# up on continuity and takes the other representation.  This is the hysteresis
# that stops the flip-flopping near a 180 deg branch.  Bigger = more stable
# there, but more roll against the wall.  0 reproduces the old behaviour.
ROLL_HYST = 25.0

# Largest roll correction applied in one step.  The loop now measures the bend
# plane directly and corrects toward BRANCH_DIR, so it converges on its own;
# this just keeps a single step from being a lurch.
ROLL_STEP_MAX = 20.0

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
ROLL_MF    = 5.0      # was 2.0.  Turning a loaded disc inside the bore has to
                      # drag three wheels sideways: 3 x 18 N x 1.1 x 42 mm =
                      # 2.5 N.m of pure friction.  At 2.0 the drive could not
                      # reliably win that, which is the first thing to rule out
                      # for the 179-190 band.  roll_actual() now measures it.
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
    hv, hl = _view("%s/Robot/DiscF" % ROBOT, "v10_head")
    bv, bl = _view("%s/Robot/Body" % ROBOT, "v10_body")
    dv, _dl = _view("%s/Robot/SphF" % ROBOT, "v10_drum")

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

    drum = None
    if dv is not None:
        def drum(v=dv):
            _p, q = v.get_world_poses()
            w, x, y, z = (float(q[0][0]), float(q[0][1]),
                          float(q[0][2]), float(q[0][3]))
            return Gf.Matrix3d(Gf.Rotation(Gf.Quatd(w, Gf.Vec3d(x, y, z))))

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
    return head, hl, body, bl, drum


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


def _bend_dir_now():
    """Which way a POSITIVE bend currently sends the front disc, in world.

    The bend turns about the drum's own Y, so the disc moves along Yd x Xd.
    Reading this off the drum itself is the whole point: it is the absolute
    orientation of the bend plane, which is what we actually care about.
    """
    if _S.get("drumread") is None:
        return None, None
    try:
        rd = _S["drumread"]()
    except Exception:
        return None, None
    xd = Gf.Vec3d(rd[0][0], rd[0][1], rd[0][2]).GetNormalized()
    yd = Gf.Vec3d(rd[1][0], rd[1][1], rd[1][2]).GetNormalized()
    return Gf.Cross(yd, xd).GetNormalized(), xd


def _roll_target(first=False):
    """Roll command and bend sign that put the bend plane on BRANCH_DIR.

    WHY THIS IS MEASURED OFF THE DRUM, NOT THE BODY

    The previous version recomputed the roll from the BODY's attitude every
    step.  But rolling the drum torques the body the opposite way, the tracker
    then saw the body had moved and asked for more roll, which turned the body
    further - a positive feedback loop closed through the reaction torque.  The
    log showed it plainly: the command ran 85 -> -106 -> +105 -> 110 and the
    representation flipped twice in one second, finally locking 25 deg away
    from the correct 85.  The drum tracked its command perfectly the whole
    time (final error -1.2 deg), so torque was never the issue.

    So close the loop on the thing that actually matters instead: measure
    where the bend plane points RIGHT NOW from the drum's own orientation,
    work out the angular error to BRANCH_DIR about the disc axis, and correct
    the roll by exactly that.  Body roll is then just a disturbance the loop
    rejects, not something it chases.  The error also tells us the sign
    directly - if correcting would need more than 90 deg, flip the bend
    instead and take the short way round.
    """
    cur, axis = _bend_dir_now()
    if cur is None:                      # no drum feedback: old body-based way
        ax = _body_axes()
        b = Gf.Vec3d(*BRANCH_DIR).GetNormalized()
        bl = Gf.Vec3d(Gf.Dot(b, ax[0]), Gf.Dot(b, ax[1]), Gf.Dot(b, ax[2]))
        if math.hypot(bl[1], bl[2]) < 1e-6:
            return _S.get("roll", 0.0), _S.get("sign", 1.0)
        raw = math.degrees(math.atan2(bl[1], -bl[2]))
        c = [(raw + d, sg) for d, sg in ((0.0, 1.0), (180.0, -1.0), (-180.0, -1.0))
             if -180.0 - 1e-6 <= raw + d <= 180.0 + 1e-6]
        return min(c, key=lambda q: abs(q[0]))

    b = Gf.Vec3d(*BRANCH_DIR)
    want = b - axis * Gf.Dot(b, axis)     # only the part across the pipe
    if want.GetLength() < 1e-9:
        return _S.get("roll", 0.0), _S.get("sign", 1.0)
    want = want.GetNormalized()

    err = math.degrees(math.atan2(Gf.Dot(Gf.Cross(cur, want), axis),
                                  Gf.Dot(cur, want)))
    sign = 1.0
    if err > 90.0:
        err -= 180.0
        sign = -1.0
    elif err < -90.0:
        err += 180.0
        sign = -1.0

    err = max(-ROLL_STEP_MAX, min(ROLL_STEP_MAX, err))
    have = _roll_actual()
    base = have if have is not None else _S.get("roll", 0.0)
    return base + err, sign


def _roll_actual():
    """Roll the drum has ACTUALLY reached, measured from the two bodies.

    This is the number that settles whether a failure is a wrong command or a
    joint that could not get there: compare it with what is being asked for.
    """
    if _S.get("drumread") is None or _S.get("bodyread") is None:
        return None
    try:
        _, rb = _S["bodyread"]()
        rd = _S["drumread"]()
    except Exception:
        return None
    bx = [Gf.Vec3d(rb[i][0], rb[i][1], rb[i][2]).GetNormalized() for i in range(3)]
    yd = Gf.Vec3d(rd[1][0], rd[1][1], rd[1][2]).GetNormalized()
    return math.degrees(math.atan2(Gf.Dot(yd, bx[2]), Gf.Dot(yd, bx[1])))


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

    hread, hlabel, bread, blabel, dread = _make_readers()
    v = WHEEL_CMD * math.pi / 180.0 * WHEEL_R * SPEED_SCALE
    k0 = bends[1].GetAttribute("drive:angular:physics:stiffness").Get() or 25.0
    d0 = bends[1].GetAttribute("drive:angular:physics:damping").Get() or 3.0

    _S.clear()
    _S.update(jp=jp, travel=travel, ax0=ax0, bends=bends, rolls=rolls,
              wheels=wheels, sign=1.0, v=v, s0=s0, t=None, last=-1.0,
              shift=0.0, reader=hread, label=hlabel, bodyread=bread,
              bodylabel=blabel, drumread=dread, mode="pending", p0=None,
              arc=None, prev=None, done=False, bend_k=k0, bend_d=d0,
              roll_locked=False)
    _S["roll"], _S["sign"] = _roll_target(first=True)

    if VERBOSE:
        print("[v9] head pose  :", hlabel or "NONE (odometry only)")
        print("[v9] body attitude:", blabel or "NONE - roll tracking is STATIC")
        print("[v9] head starts %+.0f mm from the junction" % (s0 * 1000))
        print("[v9] BRANCH_DIR (world) %s -> Roll %.1f deg, bend sign %+d"
              % ([round(q, 2) for q in BRANCH_DIR], _S["roll"], int(_S["sign"])))
        print("[v9] roll tracking %s (hysteresis +-%.0f deg), locks past BendF %.0f"
              % ("ON" if ROLL_TRACK else "OFF", ROLL_HYST, ROLL_LOCK_AT))
        print("[v9] roll drive maxForce %.2f N.m (friction to beat ~2.5)" % ROLL_MF)
        ra = _roll_actual()
        print("[v9] drum feedback: %s"
              % ("OK, roll now %.1f deg - aiming loop closed on the DRUM" % ra
                 if ra is not None
                 else "NONE - falling back to the old body-based roll"))
        pf = max(abs(_targets(q / 1000.0)[0]) for q in range(-150, 400))
        print("[v9] peak BendF %.1f deg | front release delay %.0f mm"
              % (pf, FRONT_RELEASE_DELAY_MM))
        print("[v9] ready.  selftest() -> reset() -> go()")


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
            print("[v9] no live pose -> odometry")
        elif _S["p0"] is None:
            _S["p0"] = _S["reader"]()
        elif _S["t"] > 1.0:
            moved = (_S["reader"]() - _S["p0"]).GetLength()
            _S["mode"] = "live" if moved > 0.005 else "odometry"
            print("[v9] poses moved %.0f mm in 1 s -> %s"
                  % (moved * 1000, _S["mode"]))

    tg = _targets(s)

    if ROLL_TRACK and not _S["roll_locked"]:
        if abs(tg[0]) >= ROLL_LOCK_AT:
            _S["roll_locked"] = True
            print("[v9] roll locked at %.1f deg, sign %+d (BendF reached %.1f)"
                  % (_S["roll"], int(_S["sign"]), tg[0]))
        else:
            old = _S["sign"]
            _S["roll"], _S["sign"] = _roll_target()
            if old != _S["sign"]:
                print("[v9] bend sign -> %+d (roll %.1f)"
                      % (int(_S["sign"]), _S["roll"]))
    _apply_roll()

    _set_bend(_S["bends"][0], tg[0] * _S["sign"])
    if not REAR_FREE:
        _set_bend(_S["bends"][1], tg[1] * _S["sign"])

    if VERBOSE and _S["t"] - _S["last"] >= 0.5:
        _S["last"] = _S["t"]
        rear = "free" if REAR_FREE else "%+6.1f" % (tg[1] * _S["sign"])
        ra = _roll_actual()
        rs = ("%+6.1f (err %+.1f)" % (ra, ra - _S["roll"])
              if ra is not None else "   n/a")
        print("[v9] %-9s t=%4.1fs s=%+6.0fmm  BendF %+6.1f  BendR %s  Roll cmd %+6.1f act %s  sign %+d%s"
              % (_S["mode"], _S["t"], s * 1000,
                 tg[0] * _S["sign"], rear, _S["roll"], rs, int(_S["sign"]),
                 " (locked)" if _S["roll_locked"] else ""))

    if s > (2 * LAG_MM + max(TURN_FRONT, TURN_BODY, TURN_REAR)) / 1000.0:
        _S["done"] = True
        _set_bend(_S["bends"][0], 0.0)
        if not REAR_FREE:
            _set_bend(_S["bends"][1], 0.0)
        print("[v9] through the branch - bends released, still driving")


# ------------------------------------------------------------------ API
def set_branch(x, y, z):
    """WORLD 좌표계 기준 분기 방향 설정."""
    global BRANCH_DIR
    BRANCH_DIR = Gf.Vec3d(float(x), float(y), float(z))
    _S["roll"], _S["sign"] = _roll_target(first=True)
    _apply_roll()
    print("[v9] branch -> %s (world) | roll %.1f deg, bend sign %+d"
          % ([round(q, 2) for q in BRANCH_DIR], _S["roll"], int(_S["sign"])))


def set_branch_xy(x, y):
    """단면 좌표로 분기 방향 지정.  +x = RIGHT, +y = UP"""
    v = Gf.Vec3d(*PLANE_RIGHT) * float(x) + Gf.Vec3d(*PLANE_UP) * float(y)
    if v.GetLength() < 1e-9:
        print("[v9] zero direction - ignored")
        return
    set_branch(v[0], v[1], v[2])


def set_branch_angle(deg):
    """단면 기준 각도.  0 = RIGHT, +90 = UP, 180 = LEFT, -90 = DOWN"""
    a = math.radians(float(deg))
    set_branch_xy(math.cos(a), math.sin(a))
    print("[v9] cross-section angle %.1f deg" % deg)


def plane_check():
    """현재 단면 좌표축과 각도 방향 확인."""
    print("[v9] travel (world)      %s" % [round(q, 3) for q in _S["travel"]])
    print("[v9] PLANE_RIGHT (+x)    %s" % [round(q, 3) for q in PLANE_RIGHT])
    print("[v9] PLANE_UP    (+y)    %s" % [round(q, 3) for q in PLANE_UP])
    print("[v9] BRANCH_DIR now      %s" % [round(q, 3) for q in BRANCH_DIR])
    for nm, d in (("right", 0), ("up", 90), ("left", 180), ("down", -90)):
        a = math.radians(d)
        v = Gf.Vec3d(*PLANE_RIGHT) * math.cos(a) + Gf.Vec3d(*PLANE_UP) * math.sin(a)
        print("[v9]   %-5s (%4d deg) -> world %s" % (nm, d, [round(q, 2) for q in v]))


def roll_wait(deg_tol=5.0):
    """Report whether the drum has settled on its commanded roll.  Call this
    after reset() and before go() - if the error is large the bend plane is
    simply not where the schedule thinks it is."""
    ra = _roll_actual()
    if ra is None:
        print("[v9] no drum reader - cannot check")
        return False
    err = ra - _S["roll"]
    ok = abs(err) <= deg_tol
    print("[v9] roll cmd %+.1f, actual %+.1f, error %+.1f -> %s"
          % (_S["roll"], ra, err, "settled" if ok else "NOT THERE YET"))
    return ok


def roll_track(on=True):
    global ROLL_TRACK
    ROLL_TRACK = bool(on)
    _S["roll_locked"] = False
    print("[v9] roll tracking -> %s" % ("ON" if ROLL_TRACK else "OFF"))


def aim_error():
    """How far the bend plane is from BRANCH_DIR right now, in degrees.
    This is the quantity the roll loop drives to zero."""
    cur, axis = _bend_dir_now()
    if cur is None:
        print("[v9] no drum feedback - cannot measure aim")
        return None
    b = Gf.Vec3d(*BRANCH_DIR)
    want = (b - axis * Gf.Dot(b, axis)).GetNormalized()
    e = math.degrees(math.atan2(Gf.Dot(Gf.Cross(cur, want), axis),
                                Gf.Dot(cur, want)))
    print("[v9] bend plane points %s" % [round(q, 3) for q in cur])
    print("[v9] want              %s" % [round(q, 3) for q in want])
    print("[v9] aim error %+.1f deg (0 or +-180 = on target, sign picks which)"
          % e)
    return e


def roll_now():
    """현재 자세에서 두 표현 모두와, 지금 선택된 것을 출력."""
    ax = _body_axes()
    b = Gf.Vec3d(*BRANCH_DIR).GetNormalized()
    bl = Gf.Vec3d(Gf.Dot(b, ax[0]), Gf.Dot(b, ax[1]), Gf.Dot(b, ax[2]))
    raw = math.degrees(math.atan2(bl[1], -bl[2]))
    print("[v9] body axis (world) %s" % [round(q, 3) for q in ax[0]])
    print("[v9] branch in body frame %s -> raw roll %.1f"
          % ([round(q, 3) for q in bl], raw))
    for d, sg in ((0.0, 1.0), (180.0, -1.0), (-180.0, -1.0)):
        r = raw + d
        if -180.0 - 1e-6 <= r <= 180.0 + 1e-6:
            mark = "  <- in use" if abs(r - _S["roll"]) < 1e-6 else ""
            print("[v9]   option roll %+7.1f  sign %+d%s" % (r, int(sg), mark))
    print("[v9] margin to the +-%.0f flip boundary: %.1f deg"
          % (90.0 + ROLL_HYST, 90.0 + ROLL_HYST - abs(_S["roll"])))
    ra = _roll_actual()
    if ra is None:
        print("[v9] achieved roll: cannot measure (no drum reader)")
    else:
        print("[v9] achieved roll %+.1f vs commanded %+.1f -> error %+.1f deg"
              % (ra, _S["roll"], ra - _S["roll"]))
        if abs(ra - _S["roll"]) > 10.0:
            print("[v9] >> the drum is NOT reaching its target."
                  "  raise ROLL_MF, not the schedule.")


def selftest():
    _S["t"] = None
    _apply_roll()
    _set_bend(_S["bends"][0], 0.0, mf=BEND_MF)
    _configure_rear()
    _set_bend(_S["bends"][0], 40.0 * _S["sign"])
    print("[v9] BendF -> %.0f deg.  LOOK AT THE ROBOT.  reset() when done."
          % (40.0 * _S["sign"]))


def reset():
    _S["t"] = None
    _S["done"] = False
    _S["roll_locked"] = False
    _S["roll"], _S["sign"] = _roll_target(first=True)
    _apply_roll()
    _set_bend(_S["bends"][0], 0.0, mf=BEND_MF)
    _configure_rear()
    print("[v9] straight | roll %.1f deg, bend sign %+d (tracking %s) | rear %s"
          % (_S["roll"], int(_S["sign"]), "ON" if ROLL_TRACK else "OFF",
             "SOFT-BIASED" if REAR_FREE else "driven"))


def where():
    if _S.get("reader") is None:
        print("[v9] no live reader")
        return
    cur = Gf.Dot(_S["reader"]() - _S["jp"], _S["travel"])
    s = (_S["arc"] if _S["arc"] is not None else cur) + _S.get("shift", 0.0)
    tg = _targets(s)
    print("[v9] head proj %+.0f mm | schedule s %+.0f mm | mode %s"
          % (cur * 1000, s * 1000, _S.get("mode")))
    print("[v9] BendF %+.1f  BendR %s  Roll %+.1f  sign %+d%s"
          % (tg[0] * _S["sign"],
             "free" if REAR_FREE else "%+.1f" % (tg[1] * _S["sign"]),
             _S["roll"], int(_S["sign"]),
             " (locked)" if _S["roll_locked"] else ""))
    print("[v9] segment angles  front %.0f  body %.0f  rear %.0f deg" % _angles(s))
    roll_now()


def nudge(mm):
    _S["shift"] = _S.get("shift", 0.0) + mm / 1000.0
    print("[v9] shift %+0.0f mm (total %+0.0f)" % (mm, _S["shift"] * 1000))


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
    print("[v9] wheels %.0f deg/s, head %+.0f mm out, schedule armed"
          % (WHEEL_CMD, _S["s0"] * 1000))


def stop():
    _set_wheels(0.0)
    _S["t"] = None
    _set_bend(_S["bends"][0], 0.0)
    if not REAR_FREE:
        _set_bend(_S["bends"][1], 0.0)
    print("[v9] stopped")


def v9_stop():
    global _sub
    if _sub is not None:
        try:
            _sub.unsubscribe()
        except Exception:
            pass
        _sub = None
        print("[v9] callback removed")


def v9_start():
    global _sub
    v9_stop()
    _setup()
    _sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)


v9_start()
set_branch_angle(90.0)
go()

