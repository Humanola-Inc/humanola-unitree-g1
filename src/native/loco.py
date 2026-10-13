from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Protocol

from humanola import robo


class WalkMode(IntEnum):
    # these are the loco fsm ids
    NORMAL = 500
    AI = 801


class Loco(Protocol):
    def start(self):
        pass

    def set_walk_mode(self, mode: WalkMode):
        pass

    def move(self, x: float, y: float, w: float):
        pass

    def close(self):
        pass


class LocoSim:
    def __init__(self):
        self.walk_mode = WalkMode.NORMAL
        self.velocity = (0.0, 0.0, 0.0)

    def start(self):
        pass

    def set_walk_mode(self, mode: WalkMode):
        self.walk_mode = mode

    def move(self, x: float, y: float, w: float):
        self.velocity = (x, y, w)

    def close(self):
        pass


class LocoNotSim:
    def __init__(self):
        # imported here so --sim runs without the unitree sdk installed
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

        self.loco = LocoClient()

    def start(self):
        self.loco.Init()

    def set_walk_mode(self, mode: WalkMode):
        self.loco.SetFsmId(mode.value)

    def move(self, x: float, y: float, w: float):
        self.loco.Move(x, y, w)

    def close(self):
        pass


@dataclass
class LocoConfig:
    translation_joy: robo.Query
    rotation_joy: robo.Query
    walk_mode_switch: robo.Query
    max_forward: float = 1.0
    max_lateral: float = 0.5
    max_angular: float = 0.7

    @staticmethod
    def xr() -> LocoConfig:
        return LocoConfig(
            translation_joy=robo.Query(kind=robo.SensorKind.Joy, name="xr.left.joy"),
            rotation_joy=robo.Query(kind=robo.SensorKind.Joy, name="xr.right.joy"),
            walk_mode_switch=robo.Query(kind=robo.SensorKind.Btn, name="xr.right.joy"),
        )

    @staticmethod
    def gamepad() -> LocoConfig:
        return LocoConfig(
            translation_joy=robo.Query(kind=robo.SensorKind.Joy, name="dpad.left.joy"),
            rotation_joy=robo.Query(kind=robo.SensorKind.Joy, name="dpad.right.joy"),
            walk_mode_switch=robo.Query(kind=robo.SensorKind.Btn, name="dpad.square"),
        )


class Xr2Loco:
    def __init__(self, config: LocoConfig, sim: bool = False):
        self.config = config
        self.loco: Loco = LocoSim() if sim else LocoNotSim()
        self.walk_mode = WalkMode.NORMAL

    def open(self):
        self.loco.start()
        self.walk_mode = WalkMode.NORMAL
        self.loco.set_walk_mode(self.walk_mode)
        return self

    def close_stream(self):
        self.loco.close()

    def should_switch_walk_mode(self, prev: robo.Device, cur: robo.Device) -> bool:
        prev_btn = prev.get(self.config.walk_mode_switch)
        cur_btn = cur.get(self.config.walk_mode_switch)
        return (
            prev_btn is not None
            and cur_btn is not None
            and prev_btn.as_btn().pressed
            and not cur_btn.as_btn().pressed
        )

    def recv_delta(self, prev: robo.Device, cur: robo.Device):
        x, y, w = 0.0, 0.0, 0.0
        translation_joy = cur.get(self.config.translation_joy)
        if translation_joy is not None:
            translation_joy = translation_joy.as_joy()
            x = -translation_joy.y * self.config.max_forward
            y = -translation_joy.x * self.config.max_lateral
        rotation_joy = cur.get(self.config.rotation_joy)
        if rotation_joy is not None:
            w = -rotation_joy.as_joy().x * self.config.max_angular
        if not (x == 0 and y == 0 and w == 0):
            self.loco.move(x, y, w)
        if self.should_switch_walk_mode(prev, cur):
            if self.walk_mode == WalkMode.NORMAL:
                self.walk_mode = WalkMode.AI
            else:
                self.walk_mode = WalkMode.NORMAL
            self.loco.set_walk_mode(self.walk_mode)
