"""[단독판] sensor_msgs.msg 대역."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from _msgbase import Msg


class Image(Msg):
    _fields = ("height", "width", "encoding", "step", "data",
               "is_bigendian")


class CompressedImage(Msg):
    _fields = ("format", "data")


class CameraInfo(Msg):
    _fields = ("height", "width", "distortion_model", "d", "k", "r", "p")


class Imu(Msg):
    _fields = ("orientation", "angular_velocity", "linear_acceleration")


class JointState(Msg):
    _fields = ("name", "position", "velocity", "effort")
