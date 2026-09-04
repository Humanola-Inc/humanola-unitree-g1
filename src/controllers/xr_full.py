import logging
from dataclasses import dataclass
from typing import List

from humanola import robo as controllers

from .arms_v1 import G1Arm, G1ArmConfig
from .loco import LocoConfig, LocoController, WalkMode


@dataclass
class XrFullConfig:
    loco: LocoConfig
    arms: G1ArmConfig
    others: List[object]


class XrFull:
    def __init__(self, config: XrFullConfig):
        self.config = config
        self.loco = LocoController(self.config.loco)
        self.walk_mode = WalkMode.NORMAL
        self.arms = G1Arm(self.config.arms)
        self.other_skip_ids = []

    def open(self):
        self.loco.open()
        for id, c in enumerate(self.config.others):
            try:
                c.open()
            except Exception as e:
                self.other_skip_ids.append(id)
                logging.warning(f"[WARN] cannot start: {e}")
        self.arms.open()
        self.walk_mode = WalkMode.NORMAL
        return self

    def recv_delta(self, prev: controllers.Device, cur: controllers.Device):
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
                self.arms.close()
            else:
                self.walk_mode = WalkMode.NORMAL
                self.arms.open()
        self.loco.recv_delta(prev, cur)
        if self.walk_mode == WalkMode.NORMAL:
            self.arms.recv_delta(prev, cur)
        for id, c in enumerate(self.config.others):
            if id not in self.other_skip_ids:
                c.recv_delta(prev, cur)

    def close(self):
        self.loco.close()
        if self.walk_mode == WalkMode.NORMAL:
            self.arms.close()
        for id, c in enumerate(self.config.others):
            if id not in self.other_skip_ids:
                c.close()
