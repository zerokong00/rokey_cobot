"""규약을 Isaac 프로젝트 자리(`isaacpjt/dongmin/pipe_comm/`)로 내보낸다.

    src/dongmin/pipe_comm/pipe_comm  ──생성──▶  isaacpjt/dongmin/pipe_comm
              (원본, 단일 출처)                        (사본, 읽기 전용)

🚨 **사본은 손으로 고치지 말 것.** 규약이 두 벌이 되면 반드시 갈라진다 —
   팀 안에서 이미 `/rgb` `front/rgb` `/rgb/compressed` 세 갈래로 갈린 적이
   있고, `pipe_comm` 은 그걸 합치려고 만든 패키지다.

   그래서 이 도구는 `src/son` → `isaacpjt/integration/son` 을 굽는
   `src/son/tools/make_standalone.py` 와 같은 방식을 쓴다: 지난번 생성물의
   해시를 `.generated_manifest.json` 에 들고 있다가, **사본이 손으로 바뀌었으면
   덮어쓰기 전에 알려 준다.** 가져오지는 않는다 — 판단은 사람이 한다.

왜 사본이 필요한가
  Isaac 쪽(Python 3.11)은 colcon 설치본을 못 쓴다(3.10 ABI). 소스를 경로로
  읽어야 하는데, Isaac 담당자가 워크스페이스 전체가 아니라 `isaacpjt/` 만
  받는 경우가 있어 그 안에도 규약이 있어야 한다.

  ⚠ **워크스페이스를 통째로 받는 사람은 이 사본이 필요 없다.**
    `src/dongmin/pipe_comm` 을 직접 `sys.path` 에 넣는 쪽이 언제나 낫다
    (`src/dongmin/isaac_bridge/ros_bridge.py` 가 그렇게 한다).

내보내는 것은 **import 되는 모듈뿐**이다. 수신 노드(camera_monitor 등)는
ROS 쪽 전용이라 Isaac 에 갈 이유가 없다.

실행:
  python3 src/dongmin/pipe_comm/tools/export_isaac.py           # 확인만
  python3 src/dongmin/pipe_comm/tools/export_isaac.py --write   # 실제로 쓴다
"""

import hashlib
import json
import shutil
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]          # src/dongmin/pipe_comm
WS = PKG.parents[2]                                # 워크스페이스
DEST = WS / "isaacpjt" / "dongmin" / "pipe_comm"
MANIFEST = DEST / ".generated_manifest.json"

# Isaac 쪽이 import 하는 것만. 노드는 안 보낸다.
FILES = ["__init__.py", "contract.py", "image_codec.py"]

HEADER = """# 이 디렉터리는 **생성물이다. 손으로 고치지 말 것.**
#
#   원본:   src/dongmin/pipe_comm/pipe_comm/
#   생성:   python3 src/dongmin/pipe_comm/tools/export_isaac.py --write
#
# 규약이 두 벌이 되면 갈라진다. 고칠 것이 있으면 **원본을 고치고 다시 굽는다.**
# 워크스페이스를 통째로 받았다면 이 사본 대신 원본을 sys.path 에 넣을 것.
"""


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def main():
    write = "--write" in sys.argv
    src = PKG / "pipe_comm"
    old = json.loads(MANIFEST.read_text()) if MANIFEST.is_file() else {}

    # 🚨 먼저 **저쪽이 손으로 바뀌었는지** 본다. 덮어쓰고 나면 못 안다.
    edited = []
    for name in FILES:
        d = DEST / name
        if d.is_file() and name in old and sha(d) != old[name]:
            edited.append(name)
    if edited:
        print("🚨 사본이 손으로 바뀌었다 — 덮어쓰면 그 수정이 사라진다:")
        for n in edited:
            print(f"     isaacpjt/dongmin/pipe_comm/{n}")
        print("   내용을 확인해 원본에 반영한 뒤 다시 돌릴 것.")
        if write:
            print("   (--write 를 줬지만 중단한다)")
            return 2

    changed, manifest = [], {}
    for name in FILES:
        s, d = src / name, DEST / name
        if not s.is_file():
            print(f"[중단] 원본이 없다: {s}")
            return 1
        h = sha(s)
        manifest[name] = h
        if not d.is_file() or sha(d) != h:
            changed.append(name)
        if write:
            DEST.mkdir(parents=True, exist_ok=True)
            shutil.copy2(s, d)

    if write:
        (DEST / "README.md").write_text(HEADER)
        MANIFEST.write_text(json.dumps(manifest, indent=2, sort_keys=True))
        print(f"내보냄 → {DEST}")
        for n in FILES:
            print(f"  {n}{'  (변경)' if n in changed else ''}")
    else:
        print(f"대상 {DEST}")
        print("변경될 파일: " + (", ".join(changed) if changed else "없음"))
        print("실제로 쓰려면 --write")
    return 0


if __name__ == "__main__":
    sys.exit(main())
