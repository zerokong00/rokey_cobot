"""[자산생성] 단독 기능확인판 zip 생성 — 원본은 건드리지 않는다.

## 왜 두 판인가

원본은 **PC1↔PC2 ROS 통신**을 전제로 rclpy 로 발행한다. 방향은 맞지만,
Isaac Sim 담당자가 지금 하려는 것은 통신 검증이 아니라 **"구현한 것이
돌아가는가"** 확인이다. 그 확인을 하려고 ROS 2 를 Python 3.11 로 다시
빌드(docker 필요)해야 하는 것은 순서가 뒤집힌 것이다.

그래서 이 스크립트가 **사본에만** 손을 대서 rclpy 의존을 걷어낸 판을 만든다.

    tools/standalone_stub/   rclpy / sensor_msgs / std_msgs / geometry_msgs 대역
                             발행 대신 값을 찍고 이미지를 저장한다

`son/standalone/` 으로 들어가고, 사본의 `pyver.py` 가 그 경로를 sys.path 앞에
붙인다. **애플리케이션 코드는 한 줄도 안 고친다** — import 되는 것만 다르다.

⚠ 이 판으로는 **ROS 통신을 검증할 수 없다.** 그것은 원본 판으로 따로 한다.

## 두 가지 출력

    --out  <zip>    압축 파일 (예전 방식. 직접 전달할 때)
    --dest <디렉터리>  통합본 자리에 바로 생성 (git 방식. 지금 쓰는 것)

## 한 방향만 자동이다

    src/son  ──생성──▶  isaacpjt/integration/son  ──push──▶  git
       ▲                                                     │
       └──────── 사람이 판단해서 반영 (자동 아님) ◀──────pull──┘

**`src/son` 은 읽기만 한다.** 통합본에서 무슨 일이 나든 원본에 쓰지 않는다.

다만 통합본은 Isaac Sim 담당자도 직접 고친다. 그대로 재생성하면 그 수정이
조용히 사라지므로, 생성 직전에 **무엇이 날아가는지 알려준다.** 가져오지는
않는다 — 판단은 사람이 한다.

판별은 `.generated_manifest.json` 으로 한다. 지난번 생성물의 해시를 들고
있다가

    해시가 다르다   → **저쪽이 고친 것** (경고하고 확인을 받는다)
    매니페스트에 없다 → **저쪽이 추가한 것** (마찬가지)
    내 쪽만 바뀐 것  → 그냥 목록으로 보여준다

실행:
  python3 tools/make_standalone.py --dest ../../isaacpjt/integration/son
  python3 tools/make_standalone.py --out ~/Downloads/이름.zip
"""

import argparse
import hashlib
import os
import json
import shutil
import subprocess
import sys
import tempfile
import zipfile
from datetime import date
from pathlib import Path

HERE = Path(__file__).resolve().parent
SON = HERE.parent
STUB = HERE / "standalone_stub"

# 통째로 빼는 것 — 학습 데이터(268MB), 관리자 쪽(파킹), 다른 과제 잔재
DROP_DIRS = {"training", "monitor", "color_detector", "color_interfaces",
             "__pycache__", "standalone_stub"}
# 시험 장면은 make_scenes.py 로 재생성 가능하다. 대표 몇 개만 넣는다.
DROP_FILES = {".gitignore", ".generated_manifest.json"}
SCENE_DIR = "test_code/condition/scenes"
SCENE_KEEP = 6
SCENE_META = "scenes_meta.json"

PYVER_ANCHOR = "def require_isaac(who=\"\", needs_rclpy=False):"
PYVER_PATCH = '''# ── 단독 기능확인판 ──────────────────────────────────────────────
# 이 줄부터는 **zip 사본에만** 있는 코드다. tools/make_standalone.py 가
# 넣는다. 원본 src/son/pyver.py 에는 없다.
#
# rclpy 자리에 standalone/ 의 대역 모듈이 들어간다. 발행 대신 값을 찍고
# 이미지를 저장한다. 애플리케이션 코드는 한 줄도 안 고쳤다.
_STANDALONE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "standalone")
if os.path.isdir(_STANDALONE) and _STANDALONE not in sys.path:
    sys.path.insert(0, _STANDALONE)
STANDALONE = os.path.isdir(_STANDALONE)


'''
# 단독판에서는 rclpy 유무를 따지지 않는다 — 대역이 항상 있다.
NEEDS_RCLPY_PATCH = ('    if needs_rclpy:',
                     '    if needs_rclpy and not STANDALONE:')

NOTE = """# 단독 기능확인판 (STANDALONE)

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
"""


def build(dst_root):
    son = dst_root / "son"
    son.mkdir(parents=True)

    for item in sorted(SON.iterdir()):
        if item.name in DROP_DIRS or item.name.endswith(".zip"):
            continue
        # .gitignore 는 **원본 저장소 사정**이다. 통합본에 딸려가면 거기서
        # 시험 장면(.npy)을 가려 저쪽이 test_detector.py 를 못 돌린다.
        if item.name in DROP_FILES:
            continue
        if item.is_dir():
            shutil.copytree(item, son / item.name,
                            ignore=shutil.ignore_patterns(
                                "__pycache__", "*.pyc", "*.zip"))
        else:
            shutil.copy2(item, son / item.name)

    # 시험 장면은 대표 몇 개만
    scenes = son / SCENE_DIR
    if scenes.is_dir():
        # scenes_meta.json 은 **반드시** 남긴다. 이름순으로 자르면 뒤로 밀려
        # 잘려나가고, 그러면 test_detector.py 가 FileNotFoundError 로 죽는다.
        keep = {SCENE_META} | set(
            sorted(q.name for q in scenes.iterdir()
                   if q.name != SCENE_META)[:SCENE_KEEP])
        for q in scenes.iterdir():
            if q.name not in keep:
                q.unlink()

    # 대역 모듈을 얹는다
    shutil.copytree(STUB, son / "standalone",
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))

    # 사본의 pyver.py 만 고친다
    pv = son / "pyver.py"
    src = pv.read_text()
    if PYVER_ANCHOR not in src:
        raise SystemExit(f"[중단] pyver.py 에서 기준점을 못 찾았다: {PYVER_ANCHOR}")
    src = src.replace(PYVER_ANCHOR, PYVER_PATCH + PYVER_ANCHOR, 1)
    if NEEDS_RCLPY_PATCH[0] not in src:
        raise SystemExit("[중단] pyver.py 의 needs_rclpy 분기를 못 찾았다")
    src = src.replace(*NEEDS_RCLPY_PATCH, 1)
    pv.write_text(src)

    (son / "STANDALONE.md").write_text(NOTE)
    return son


def verify(son):
    """사본이 실제로 rclpy 없이 import 되는지 확인한다."""
    checks = []
    r = subprocess.run(
        [sys.executable, "-c",
         f"import sys; sys.path.insert(0, {str(son)!r}); "
         "import pyver; print('STANDALONE', pyver.STANDALONE); "
         "import rclpy; from rclpy.node import Node; "
         "from sensor_msgs.msg import Image, CameraInfo; "
         "from std_msgs.msg import Float32, Float32MultiArray; "
         "print('대역 import OK')"],
        capture_output=True, text=True)
    checks.append(("대역 모듈이 import 된다", r.returncode == 0,
                   (r.stdout + r.stderr).strip().splitlines()[-1:]))

    r2 = subprocess.run(
        [sys.executable, "-m", "compileall", "-q", str(son)],
        capture_output=True, text=True)
    checks.append(("전체 컴파일", r2.returncode == 0, []))

    src = (son / "pyver.py").read_text()
    checks.append(("pyver 가 standalone 을 sys.path 에 붙인다",
                   "STANDALONE = os.path.isdir" in src, []))
    checks.append(("단독판에서 rclpy 요구를 건너뛴다",
                   "needs_rclpy and not STANDALONE" in src, []))
    return checks


MANIFEST = ".generated_manifest.json"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]


def manifest_of(root):
    return {str(p.relative_to(root)): sha(p)
            for p in sorted(root.rglob("*"))
            if p.is_file() and p.name != MANIFEST}


def survey(dest):
    """통합본에 이미 있는 것 중 **우리가 만들지 않은 변경**을 찾는다.

    가져오지 않는다. 무엇이 덮이는지 알려주기만 한다.
    """
    if not dest.is_dir():
        return None, [], []
    mf_path = dest / MANIFEST
    if not mf_path.is_file():
        return "매니페스트 없음", [], []
    old = json.loads(mf_path.read_text())
    now = manifest_of(dest)
    changed = sorted(f for f, h in now.items()
                     if f in old and old[f] != h)
    added = sorted(f for f in now if f not in old)
    return None, changed, added


def report_overwrite(dest, assume_yes):
    note, changed, added = survey(dest)
    if note == "매니페스트 없음":
        print(f"  [주의] {dest} 에 매니페스트가 없다 — 이 스크립트가 만든 것이")
        print("         아니거나 첫 생성이다. 무엇이 덮이는지 판별할 수 없다.")
    if not changed and not added:
        return True
    print("\n" + "!" * 74)
    print("  통합본에만 있는 변경 — 재생성하면 사라진다")
    print("!" * 74)
    for f in changed:
        print(f"    수정됨  {f}")
    for f in added:
        print(f"    추가됨  {f}")
    print("\n  자동으로 가져오지 않는다. src/son 에 반영할지는 사람이 정한다.")
    print("  확인 방법:  diff src/son/<파일>  <통합본>/<파일>")
    if assume_yes:
        print("  --yes 가 주어져 그대로 진행한다.")
        return True
    try:
        ans = input("\n  덮어쓰고 계속할까? [y/N] ").strip().lower()
    except EOFError:
        ans = ""
    return ans == "y"


def main():
    ap = argparse.ArgumentParser(description="단독 기능확인판 생성")
    ap.add_argument("--dest", default=None,
                    help="통합본 디렉터리에 바로 생성 (예: "
                         "../../isaacpjt/integration/son)")
    ap.add_argument("--yes", action="store_true",
                    help="덮어쓰기 확인을 묻지 않는다")
    ap.add_argument("--out", default=str(
        Path.home() / "Downloads" /
        f"협동3_son_단독확인판_{date.today():%Y%m%d}.zip"))
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    dest = Path(a.dest).expanduser().resolve() if a.dest else None

    if dest and not report_overwrite(dest, a.yes):
        print("  중단했다. 통합본은 그대로다.")
        return 1

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        son = build(root)

        print("=" * 74)
        print("단독 기능확인판 — 사본 검사")
        print("=" * 74)
        allok = True
        for name, good, extra in verify(son):
            allok = allok and good
            print(f"  {'OK  ' if good else 'FAIL'} {name}")
            for line in extra:
                print(f"       {line}")
        if not allok:
            raise SystemExit("[중단] 사본이 검사를 통과하지 못했다")

        for p in son.rglob("__pycache__"):
            shutil.rmtree(p, ignore_errors=True)

        if dest:
            # 통합본 자리에 통째로 갈아끼운다. src/son 에는 쓰지 않는다.
            if dest.is_dir():
                shutil.rmtree(dest)
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(son, dest)
            (dest / MANIFEST).write_text(
                json.dumps(manifest_of(dest), indent=2, ensure_ascii=False))
            n = sum(1 for q in dest.rglob("*") if q.is_file())
            size = sum(q.stat().st_size for q in dest.rglob("*") if q.is_file())
            target, unit = dest, f"{size / 1e6:.1f} MB"
        else:
            out.parent.mkdir(parents=True, exist_ok=True)
            if out.exists():
                out.unlink()
            n = 0
            with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
                for q in sorted(son.rglob("*")):
                    if q.is_file():
                        z.write(q, q.relative_to(root))
                        n += 1
            target, unit = out, f"{out.stat().st_size / 1e6:.1f} MB"

    print("=" * 74)
    print(f"  {target}")
    print(f"  {unit}   파일 {n}개")
    print("  ⚠ 이 판으로는 PC1↔PC2 ROS 통신을 검증할 수 없다")
    if dest:
        rel = os.path.relpath(dest, Path.cwd())
        print(f"\n  다음: git add {rel} && git diff --staged 로 확인 후 커밋")
        print("  src/son 은 건드리지 않았다")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    sys.exit(main())
