"""[자산생성] 설계확정본 .docx → 텍스트 추출.

설계 문서가 치수의 **정본**이다. `spec/parts_meta.json` 은 여기서 파생된
사본일 뿐이고, 둘이 어긋나면 문서가 옳다.

크라운(휠 Ø20 x 폭 15mm, **크라운 반경 50mm**)이 parts_meta 로 옮겨지지
않아 2026-08-04 까지 휠이 원통으로 만들어졌다. 그런 누락을 잡으려면 문서를
기계가 읽을 수 있어야 한다. 그래서 텍스트로 떠 둔다.

실행:
  python3 tools/extract_design.py [--docx 경로]
"""

import argparse
import re
import sys
import zipfile
from pathlib import Path

SON = Path(__file__).resolve().parent.parent
DEFAULT = Path.home() / "Downloads" / "배관점검로봇_설계확정본_v3.docx"
OUT = SON / "spec" / "design_v3_extract.txt"


def extract(docx):
    x = zipfile.ZipFile(docx).read("word/document.xml").decode()
    t = re.sub(r"</w:p>", "\n", x)
    t = re.sub(r"<[^>]+>", "", t)
    return [l.strip() for l in t.split("\n") if l.strip()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--docx", default=str(DEFAULT))
    a = ap.parse_args()
    p = Path(a.docx).expanduser()
    if not p.is_file():
        print(f"[중단] {p} 가 없다")
        return 1
    lines = extract(p)
    OUT.write_text(f"# {p.name} 본문 추출\n"
                   "# 재생성: tools/extract_design.py\n"
                   "# 이 파일이 치수의 원천이다. parts_meta.json 은 여기서 파생된다.\n\n"
                   + "\n".join(lines))
    print(f"{OUT}  {len(lines)}줄")
    return 0


if __name__ == "__main__":
    sys.exit(main())
