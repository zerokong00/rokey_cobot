"""[오프라인] 부품 경로 표 검사 — Isaac Sim 없이 `KeyError` 를 미리 잡는다.

2026-08-04 실기에서 `welder/articulate.py` 가 실행 즉시 죽었다.

    KeyError: 'torch_ring'

STL 은 `welder/meshes/` 에 멀쩡히 있었고 `_CATEGORY` 표에만 등록이 빠져
있었다. 정찰기 파일을 복사해 오면서 토치 3종만 안 넣은 것이다.

**이 결함은 Isaac Sim 없이도 100% 잡을 수 있었다.** 스크립트가
`part_path("torch_ring")` 을 부르는데 표에 그 키가 없다는 것은 파일만
읽어도 알 수 있다. 그래서 여기서 검사한다.

`articulate.py` 들은 맨 위에서 `require_isaac()` 을 부르므로 3.10 에서
import 할 수 없다. **소스를 `ast` 로 읽는다** — 실행하지 않는다.

실행:  python3 test_part_paths.py
"""

import ast
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]

TARGETS = ["robot/articulate.py", "welder/articulate.py"]


def category_table(tree):
    """모듈 최상단의 _CATEGORY 딕셔너리 리터럴을 읽는다."""
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for t in node.targets:
            if isinstance(t, ast.Name) and t.id == "_CATEGORY":
                return ast.literal_eval(node.value)
    return None


def _literal(node):
    return (node.value if isinstance(node, ast.Constant)
            and isinstance(node.value, str) else None)


# 부품 이름이 실제로 넘어가는 자리. (함수명, 몇 번째 위치인수)
# part_path 는 직접 호출, make_link 는 3번째 인수가 STL 이름이다.
CALL_SITES = {"part_path": 0, "make_link": 2}


def requested_parts(tree):
    """소스가 실제로 요청하는 부품 이름을 모은다.

    ⚠ `part_path("...")` 만 보면 안 된다 — 실제 호출은 대부분
    `make_link(경로, 행렬, "wheel", ...)` 처럼 한 겹 감싸여 있다.
    처음에 part_path 만 세다가 0종이 나와 시험이 공허하게 통과했다.
    """
    out = set()
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)):
            continue
        pos = CALL_SITES.get(node.func.id)
        if pos is None or len(node.args) <= pos:
            continue
        name = _literal(node.args[pos])
        if name:
            out.add(name)
    return out


def main():
    ok = True
    print("=" * 78)
    print("부품 경로 표 검사 — 실행 없이 KeyError 를 잡는다")
    print("=" * 78)

    for rel in TARGETS:
        path = SON / rel
        tree = ast.parse(path.read_text(), filename=str(path))
        cat = category_table(tree)
        want = requested_parts(tree)

        print(f"\n{rel}")
        if cat is None:
            print("  [FAIL] _CATEGORY 를 못 찾았다")
            ok = False
            continue
        print(f"  표에 {len(cat)}종 등록, 소스에서 {len(want)}종 요청")

        missing = sorted(want - set(cat))
        print(f"  표에 없는 요청 : {missing if missing else '없음'}"
              f"  {'FAIL' if missing else 'OK'}")
        ok = ok and not missing

        # 표에 있는데 STL 이 실제로 없는 것 — 오타나 자산 누락
        gone = sorted(n for n, c in cat.items()
                      if not (SON / c / "meshes" / f"{n}.stl").is_file())
        print(f"  STL 이 없는 항목: {gone if gone else '없음'}"
              f"  {'FAIL' if gone else 'OK'}")
        ok = ok and not gone

        # 요청된 부품의 STL 이 실제로 열리는지까지 본다
        bad = []
        for n in sorted(want & set(cat)):
            f = SON / cat[n] / "meshes" / f"{n}.stl"
            if not f.is_file() or f.stat().st_size < 84:
                bad.append(n)
        print(f"  못 읽는 STL   : {bad if bad else '없음'}"
              f"  {'FAIL' if bad else 'OK'}")
        ok = ok and not bad

    # 어느 스크립트도 안 쓰는 STL — 실패는 아니고 알려만 준다
    print("\n" + "-" * 78)
    used = set()
    for rel in TARGETS:
        used |= requested_parts(ast.parse((SON / rel).read_text()))
    have = {p.stem for p in SON.glob("*/meshes/*.stl")}
    idle = sorted(have - used)
    print(f"  articulate 가 안 쓰는 STL {len(idle)}개: {idle}")
    print("  (배관·결함 메시는 씬 쪽에서 쓰므로 정상이다)")

    print("=" * 78)
    print("전체 판정:", "통과" if ok else "실패")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
