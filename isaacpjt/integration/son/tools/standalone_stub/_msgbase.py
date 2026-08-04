"""[단독판] 메시지 대역의 공통 뼈대. 속성을 자유롭게 넣고 뺄 수 있게만 한다."""


class Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class Msg:
    _fields = ()

    def __init__(self, **kw):
        self.header = Header()
        for f in self._fields:
            setattr(self, f, None)
        for k, v in kw.items():
            setattr(self, k, v)
