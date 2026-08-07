"""[단독판] rclpy.node 대역 — 발행을 진단 출력으로 바꾼다."""

import sys
import time
from pathlib import Path

import numpy as np

from . import OUT_DIR

_pubs = []
SAVE_EVERY = 30          # 몇 번째 프레임마다 이미지를 저장하나
REPORT_EVERY = 20        # 그 뒤로는 몇 번째마다 숫자를 찍나
REPORT_FIRST = 3         # 처음 몇 프레임은 무조건 찍는다 — 여기서 대부분 드러난다


class _Stamp:
    def __init__(self):
        self.sec = int(time.time())
        self.nanosec = 0


class _Now:
    def __init__(self):
        self.nanoseconds = int(time.time() * 1e9)

    def to_msg(self):
        return _Stamp()


class _Clock:
    def now(self):
        return _Now()


class _Logger:
    def __init__(self, name):
        self.name = name

    def info(self, m):
        print(f"[{self.name}] {m}")

    def warn(self, m):
        print(f"[{self.name}] 경고: {m}")

    def error(self, m):
        print(f"[{self.name}] 오류: {m}", file=sys.stderr)

    def debug(self, m):
        pass


def _depth_report(topic, a):
    """Depth 는 그림이 아니라 숫자다. 물리적으로 말이 되는지 여기서 본다."""
    fin = np.isfinite(a) & (a > 0)
    n = a.size
    bad = n - int(fin.sum())
    if not fin.any():
        w = "유효 픽셀 0 — annotator 미부착이거나 headless 다"
        return f"{topic}  {w}", w
    v = a[fin]
    note = ""
    if v.min() > 10.0:
        note = "  ← 단위가 mm 인 듯하다 (1000배 확인)"
    elif v.max() < 0.01:
        note = "  ← 값이 너무 작다"
    if bad / n > 0.02:
        note += f"  ← 무효 {bad / n * 100:.0f}% 는 판정 허용치(2%) 초과"
    return (f"{topic}  min {v.min():.4f}  max {v.max():.4f}  "
            f"중앙값 {np.median(v):.4f} m  무효 {bad / n * 100:.1f}%{note}",
            note.strip())


class _Publisher:
    def __init__(self, msg_type, topic, name):
        self.type = getattr(msg_type, "__name__", str(msg_type))
        self.topic = topic
        self.node = name
        self.n = 0
        self.last = None
        self._seen = set()
        _pubs.append(self)

    # ── 종류별 처리 ────────────────────────────────────────────────
    def _image(self, msg):
        h, w = int(msg.height), int(msg.width)
        enc = getattr(msg, "encoding", "")
        if enc == "32FC1":
            a = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            line, warn = _depth_report(self.topic, a.astype(np.float64))
            if self._should_report(warn):
                print("  " + line)
            if self.n % SAVE_EVERY == 0:
                self._save_depth(a)
        else:
            a = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)
            med = int(np.median(a))
            warn = "어둡다 — 조명 intensity 확인" if med < 10 else ""
            if self._should_report(warn):
                print(f"  {self.topic}  {w}x{h} {enc}  밝기 중앙값 {med}/255"
                      + (f"  ← {warn}" if warn else ""))
            if self.n % SAVE_EVERY == 0:
                self._save_png(a, f"{self._stem()}_{self.n:05d}.png")

    def _save_depth(self, a):
        """정규화해서 눈으로 볼 수 있게 저장한다. 원본 값은 .npy 로 같이."""
        np.save(OUT_DIR / f"{self._stem()}_{self.n:05d}.npy", a)
        v = np.where(np.isfinite(a) & (a > 0), a, np.nan)
        lo, hi = np.nanmin(v), np.nanmax(v)
        g = np.zeros(a.shape, np.uint8) if hi <= lo else \
            np.nan_to_num((v - lo) / (hi - lo) * 255).astype(np.uint8)
        self._save_png(np.dstack([g, g, g]), f"{self._stem()}_{self.n:05d}.png")

    def _save_png(self, arr, name):
        try:
            import png_writer
        except ImportError:
            from . import _png as png_writer
        png_writer.write(OUT_DIR / name, arr)

    def _compressed(self, msg):
        if self.n % SAVE_EVERY == 0:
            p = OUT_DIR / f"{self._stem()}_{self.n:05d}.jpg"
            p.write_bytes(bytes(msg.data))
        if self.n % REPORT_EVERY == 0:
            print(f"  {self.topic}  jpeg {len(msg.data) / 1024:.0f} KB")

    def _camera_info(self, msg):
        if self.n == 0:
            k = list(msg.k)
            print(f"  {self.topic}  {msg.width}x{msg.height}  "
                  f"f={k[0]:.1f}px  pp=({k[2]:.1f},{k[5]:.1f})  "
                  f"모델={getattr(msg, 'distortion_model', '?')}")

    def _scalar(self, msg):
        d = getattr(msg, "data", None)
        self.last = d
        if self._should_report():
            if isinstance(d, (list, tuple, np.ndarray)):
                s = " ".join(f"{float(x):+.3f}" for x in list(d)[:8])
                print(f"  {self.topic}  [{s}]")
            else:
                print(f"  {self.topic}  {d}")

    def _should_report(self, warn=""):
        """처음 몇 프레임 + 주기 + **새로운 이상 징후**는 반드시 찍는다.

        주기만 쓰면 두 번째 프레임부터 나타난 문제가 통째로 묻힌다.
        같은 경고는 한 번만 낸다 — 매 프레임 도배되면 못 읽는다.
        """
        if warn and warn not in self._seen:
            self._seen.add(warn)
            return True
        return self.n < REPORT_FIRST or self.n % REPORT_EVERY == 0

    def _stem(self):
        return self.topic.strip("/").replace("/", "_") or "topic"

    def publish(self, msg):
        t = type(msg).__name__
        try:
            if t == "Image":
                self._image(msg)
            elif t == "CompressedImage":
                self._compressed(msg)
            elif t == "CameraInfo":
                self._camera_info(msg)
            else:
                self._scalar(msg)
        except Exception as exc:                      # 진단이 실행을 막으면 안 된다
            print(f"  [단독판] {self.topic} 처리 실패: {exc}")
        self.n += 1


class Node:
    def __init__(self, name, *a, **kw):
        self._name = name
        self._timers = []
        self._params = {}

    def create_publisher(self, msg_type, topic, qos=None, **kw):
        return _Publisher(msg_type, topic, self._name)

    def create_subscription(self, msg_type, topic, cb, qos=None, **kw):
        print(f"[단독판] 구독 {topic} — 단독판에는 발행자가 없어 콜백이 안 온다")
        return None

    def create_timer(self, period, cb):
        self._timers.append([period, cb, 0.0])
        return None

    def _run_timers(self):
        now = time.time()
        for t in self._timers:
            if now - t[2] >= t[0]:
                t[2] = now
                t[1]()

    def declare_parameter(self, name, value=None, *a, **kw):
        self._params[name] = value
        return _Param(value)

    def get_parameter(self, name):
        return _Param(self._params.get(name))

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Logger(self._name)

    def destroy_node(self):
        pass


class _Param:
    def __init__(self, value):
        self.value = value


def summary():
    print("=" * 74)
    print("[단독판] 발행 요약 — 실제로는 저장·출력만 했다")
    print(f"  {'토픽':<34}{'타입':<18}{'횟수':>8}")
    for p in _pubs:
        print(f"  {p.topic:<34}{p.type:<18}{p.n:>8}")
    if not _pubs:
        print("  발행 없음 — 퍼블리셔가 하나도 안 만들어졌다")
    print(f"  산출물: {OUT_DIR}")
    print("=" * 74)
