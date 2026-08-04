"""[단독판] rclpy.qos 대역 — 값만 들고 있는다. 단독판에는 DDS 가 없다."""


class _Enum:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self.name


class ReliabilityPolicy:
    BEST_EFFORT = _Enum("BEST_EFFORT")
    RELIABLE = _Enum("RELIABLE")
    SYSTEM_DEFAULT = _Enum("SYSTEM_DEFAULT")


class HistoryPolicy:
    KEEP_LAST = _Enum("KEEP_LAST")
    KEEP_ALL = _Enum("KEEP_ALL")
    SYSTEM_DEFAULT = _Enum("SYSTEM_DEFAULT")


class DurabilityPolicy:
    VOLATILE = _Enum("VOLATILE")
    TRANSIENT_LOCAL = _Enum("TRANSIENT_LOCAL")


class QoSProfile:
    def __init__(self, depth=10, reliability=None, history=None,
                 durability=None, **kw):
        self.depth = depth
        self.reliability = reliability
        self.history = history
        self.durability = durability
