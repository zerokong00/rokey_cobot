"""[단독판] std_msgs.msg 대역."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from _msgbase import Msg


class Float32(Msg):
    _fields = ("data",)


class Float64(Msg):
    _fields = ("data",)


class Float32MultiArray(Msg):
    _fields = ("data", "layout")


class Bool(Msg):
    _fields = ("data",)


class String(Msg):
    _fields = ("data",)
