"""[단독판] 최소 PNG 저장기 — 의존성 없이 zlib 만 쓴다.

Isaac Sim 에 PIL/cv2 가 있으리라 가정하지 않는다. 진단 이미지를 저장하려고
설치를 요구하면 단독판의 취지가 무너진다.
"""

import struct
import zlib


def write(path, arr):
    """arr: (H, W, 3) uint8 또는 (H, W) uint8"""
    import numpy as np
    a = np.asarray(arr, dtype=np.uint8)
    if a.ndim == 2:
        a = np.dstack([a, a, a])
    h, w = a.shape[0], a.shape[1]
    raw = b"".join(b"\x00" + a[y, :, :3].tobytes() for y in range(h))

    def chunk(tag, data):
        c = tag + data
        return struct.pack(">I", len(data)) + c + struct.pack(
            ">I", zlib.crc32(c) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))
