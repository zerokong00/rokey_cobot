#!/usr/bin/env python3
"""STEP(.stp) → USD 변환 — **GUI 없이** 돈다.  [Isaac 3.11]

CATIA 에서 받은 `.stp` 를 Isaac 이 읽는 `.usd` 로 굽는다. Isaac 의 CAD 변환기
(HOOPS)는 확장으로만 제공돼 파일 메뉴에서 부르는 것이 보통이지만, 확장을 직접
켜면 SimulationApp 안에서 헤드리스로 부를 수 있다.

  isaac_python tools/step_to_usd.py <입력.stp> [출력.usd]

출력 기본값은 `src/son/maps/<이름>.usd` 다.

🔑 **STL 로 우회하지 말 것.** STL 은 단위·상방향 정보가 없어 Isaac 에서 눕고
   스케일이 1000배 틀어진다. STEP 변환본은 `upAxis Z`, `metersPerUnit 0.001`
   이 박혀 나온다.
🚨 `file_format_args` 는 **`dict[str, str]`** 이다. HoopsOptions 객체를 넘기면
   TypeError 가 난다.
🚨 변환본을 Isaac GUI 에서 **저장(Ctrl+S)하지 말 것** — 형상 없는 껍데기가
   원본을 덮어쓴다(실측으로 한 번 날렸다). 남기려면 Save As.
"""

import sys
from pathlib import Path

SON = Path(__file__).resolve().parent.parent
MAPS = SON / "maps"


def main(src: str, dst: str) -> int:
    from isaacsim import SimulationApp                      # noqa: F401
    app = SimulationApp({"headless": True})

    import omni.kit.app
    mgr = omni.kit.app.get_app().get_extension_manager()
    for ext in ("omni.kit.converter.common", "omni.kit.converter.hoops_core"):
        mgr.set_extension_enabled_immediate(ext, True)

    import omni.kit.converter.hoops_core as hoops
    conv = hoops.get_instance()
    if conv is None:
        print("[변환] ✗ HOOPS 변환기를 못 얻었다 — 확장이 안 켜졌다")
        app.close()
        return 1

    # 🚨 Kit 이 stdout 을 가로채 `print` 가 종료 때 통째로 사라진다(실측:
    #    변환 로그가 한 줄도 안 남았다). **stderr 에 즉시 flush** 로 찍는다.
    def say(msg):
        sys.stderr.write(msg + "\n")
        sys.stderr.flush()

    say(f"[변환] {src}\n     → {dst}")
    # 🚨 `create_converter_task` 는 **코루틴을 돌려준다**(내부 `create_import_task`
    #    가 async). 그냥 부르면 아무 일도 안 하고 빈 경로가 나온다 — 실측으로
    #    `path=''` 만 받고 파일이 안 생겼다. Kit 의 루프에 얹고 **`app.update()`
    #    로 펌프**해야 실제로 변환된다.
    from omni.kit.async_engine import run_coroutine
    task = run_coroutine(conv.create_converter_task(src, dst, {}))
    for _ in range(60000):
        if task.done():
            break
        app.update()
    if not task.done():
        say("[변환] ✗ 시간 초과")
        app.close()
        return 1
    ret = task.result()
    path = ret[0] if isinstance(ret, tuple) else getattr(ret, "path", "")
    status = ret[1] if isinstance(ret, tuple) else getattr(ret, "status", None)
    ok = bool(path) and Path(dst).is_file()
    say(f"[변환] {'✓' if ok else '✗'} path={path!r} status={status!r}")
    if ok:
        say(f"[변환] {Path(dst).stat().st_size / 1024:.0f}KB 저장")
    app.close()
    return 0 if ok else 1


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    _src = Path(sys.argv[1]).expanduser().resolve()
    if not _src.is_file():
        raise SystemExit(f"[중단] 입력이 없다: {_src}")
    _dst = (Path(sys.argv[2]).expanduser().resolve() if len(sys.argv) > 2
            else MAPS / (_src.stem + ".usd"))
    _dst.parent.mkdir(parents=True, exist_ok=True)
    raise SystemExit(main(str(_src), str(_dst)))
