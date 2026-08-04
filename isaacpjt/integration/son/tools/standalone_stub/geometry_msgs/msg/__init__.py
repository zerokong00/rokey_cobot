"""[단독판] geometry_msgs.msg 대역."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from _msgbase import Msg


class Vector3(Msg):
    _fields = ("x", "y", "z")


class Twist(Msg):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.linear = Vector3(x=0.0, y=0.0, z=0.0)
        self.angular = Vector3(x=0.0, y=0.0, z=0.0)
