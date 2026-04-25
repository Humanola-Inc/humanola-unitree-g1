from dataclasses import dataclass
from queue import Empty
from typing import List, Tuple, Union

from humanola import controllers, robo
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from .arms_v1 import G1Arm, G1ArmConfig
from .loco import LocoConfig, LocoController, WalkMode


class CtrlBeg:
    pass


class CtrlEnd:
    pass


class Ctrl:
    prev: bytes
    cur: bytes

    def to_dev(self) -> Tuple[controllers.Device, controllers.Device]:
        return controllers.Device.proto_decode(
            self.prev
        ), controllers.Device.proto_decode(self.cur)


@dataclass
class XrFullConfig:
    loco: LocoConfig
    arms: G1ArmConfig
    others: List["robo.ControllerStream"]


class XrFull:
    def __init__(self, config: XrFullConfig):
        self.config = config
        self.loco = LocoController(self.config.loco)
        self.walk_mode = WalkMode.NORMAL
        self.arms = G1Arm(self.config.arms)

    def ctrl_beg(self):
        self.loco.ctrl_beg()
        for c in self.config.others:
            c.ctrl_beg()
        self.arms.ctrl_beg()
        self.walk_mode = WalkMode.NORMAL

    def ctrl(self, prev: controllers.Device, cur: controllers.Device):
        prev_mode_btn = prev.get(
            controllers.Query()
            .name(self.config.loco.mode_switch_btn)
            .kind(controllers.SensorKind.Btn)
        )
        cur_mode_btn = cur.get(
            controllers.Query()
            .name(self.config.loco.mode_switch_btn)
            .kind(controllers.SensorKind.Btn)
        )
        if (
            prev_mode_btn is not None
            and cur_mode_btn is not None
            and prev_mode_btn.as_btn().pressed
            and not cur_mode_btn.as_btn().pressed
        ):
            if self.walk_mode == WalkMode.NORMAL:
                self.walk_mode = WalkMode.AI
                self.arms.ctrl_end()
            else:
                self.walk_mode = WalkMode.NORMAL
                self.arms.ctrl_beg()
        self.loco.ctrl(prev, cur)
        if self.walk_mode == WalkMode.NORMAL:
            self.arms.ctrl(prev, cur)
        for c in self.config.others:
            c.ctrl(prev, cur)

    def ctrl_end(self):
        self.loco.ctrl_end()
        if self.walk_mode == WalkMode.NORMAL:
            self.arms.ctrl_end()
        for c in self.config.others:
            c.ctrl_end()

    def desc(self):
        return robo.LoopDesc(
            name="XR Teloperation",
            desc="Full XR Teleoperation, Hands and Foot",
            frame_rate=60,
        )

    def open(self):
        return self
