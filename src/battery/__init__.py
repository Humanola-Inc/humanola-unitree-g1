from humanola import robo
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_

from g1_native import G1Subscriber


class G1Battery:
    def __init__(self):
        self.sub = G1Subscriber("rt/lf/bmsstate", BmsState_)

    def get_battery(self) -> robo.Battery:
        msg = self.sub.get_msg()
        if msg is None:
            return robo.Battery()
        else:
            return robo.Battery().attach("Main Battery", int(msg.soc.real))


__all__ = ["G1Battery"]
