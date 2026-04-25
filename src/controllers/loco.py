from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from humanola import controllers, robo
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class WalkMode(Enum):
    AI = 0
    NORMAL = 1


@dataclass
class LocoConfig:
    translation_joy: str
    rotation_joy: str
    mode_switch_btn: str
    max_forward: float = 1.0
    max_lateral: float = 0.5
    max_angular: float = 0.7

    @staticmethod
    def xr_pure() -> LocoConfig:
        return LocoConfig(
            translation_joy="xr.left.joy",
            rotation_joy="xr.right.joy",
            mode_switch_btn="xr.right.primary",
        )

    @staticmethod
    def xr_with_hands() -> LocoConfig:
        return LocoConfig(
            translation_joy="xr.left.joy",
            rotation_joy="xr.right.joy",
            mode_switch_btn="xr.right.joy",
        )

    @staticmethod
    def dpad() -> LocoConfig:
        return LocoConfig(
            translation_joy="dpad.left.joy",
            rotation_joy="dpad.right.joy",
            mode_switch_btn="dpad.square",
        )


@dataclass
class MoveCommand:
    x: float
    y: float
    w: float


class LocoController:
    def __init__(self, config: LocoConfig):
        self.config = config
        self.loco = None
        self.walk_mode = WalkMode.NORMAL
        self.rot_query = (
            controllers.Query()
            .kind(controllers.SensorKind.Joy)
            .name(self.config.rotation_joy)
        )
        self.trans_query = (
            controllers.Query()
            .kind(controllers.SensorKind.Joy)
            .name(self.config.translation_joy)
        )
        self.mode_switch_btn_query = (
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name(self.config.mode_switch_btn)
        )

    def ctrl_beg(self):
        self.loco = LocoClient()
        self.loco.Init()
        self.loco.SetFsmId(500)
        self.walk_mode = WalkMode.NORMAL

    def to_move_command(self, cur: controllers.Device) -> MoveCommand:
        left_joy = cur.get(self.trans_query)
        right_joy = cur.get(self.rot_query)
        cmd = MoveCommand(x=0, y=0, w=0)
        if left_joy is not None:
            left_joy = left_joy.as_joy()
            cmd.y = -left_joy.y * self.config.max_forward
            cmd.x = -left_joy.x * self.config.max_lateral
        if right_joy is not None:
            right_joy = right_joy.as_joy()
            cmd.w = -right_joy.x * self.config.max_angular
        return cmd

    def should_change_walk_mode(
        self, prev: controllers.Device, cur: controllers.Device
    ):
        prev_sq_btn = prev.get(self.mode_switch_btn_query)
        cur_sq_btn = cur.get(self.mode_switch_btn_query)
        return (
            prev_sq_btn is not None
            and cur_sq_btn is not None
            and prev_sq_btn.as_btn().pressed
            and not cur_sq_btn.as_btn().pressed
        )

    def should_do_wave(self, prev: controllers.Device, cur: controllers.Device):
        prev_trig_btn = prev.get(
            controllers.Query().kind(controllers.SensorKind.Btn).name("dpad.triangle")
        )
        cur_trig_btn = cur.get(
            controllers.Query().kind(controllers.SensorKind.Btn).name("dpad.triangle")
        )
        return (
            prev_trig_btn is not None
            and cur_trig_btn is not None
            and prev_trig_btn.as_btn().pressed
            and not cur_trig_btn.as_btn().pressed
        )

    def ctrl(self, prev: controllers.Device, cur: controllers.Device):
        assert self.loco is not None
        loco_cmd = self.to_move_command(cur)
        if not (loco_cmd.y == 0 and loco_cmd.x == 0 and loco_cmd.w == 0):
            self.loco.Move(loco_cmd.y, loco_cmd.x, loco_cmd.w)
        if self.should_change_walk_mode(prev, cur):
            if self.walk_mode == WalkMode.NORMAL:
                self.walk_mode = WalkMode.AI
                self.loco.SetFsmId(801)
            else:
                self.walk_mode = WalkMode.NORMAL
                self.loco.SetFsmId(500)
        # if self.should_do_wave(prev, cur):
        #     self.loco.WaveHand()

    def ctrl_end(self):
        assert self.loco is not None
        self.loco = None

    def desc(self) -> robo.LoopDesc:
        return robo.LoopDesc(
            name="PS4 Control",
            desc="Control the G1 unit with a ps4 stick",
            frame_rate=60,
        )

    def open(self):
        return self
