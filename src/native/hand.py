from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Literal, Protocol

import numpy as np
import numpy.typing as npt
from humanola import robo

FINGER_JOINTS = [
    # (proximal, tip) for pinky, ring, middle, index
    ("pinky-finger-phalanx-proximal", "pinky-finger-tip"),
    ("ring-finger-phalanx-proximal", "ring-finger-tip"),
    ("middle-finger-phalanx-proximal", "middle-finger-tip"),
    ("index-finger-phalanx-proximal", "index-finger-tip"),
]


def find_angle(
    origin: npt.NDArray[np.float64],
    x1: npt.NDArray[np.float64],
    x2: npt.NDArray[np.float64],
) -> float:
    line1 = x1 - origin
    line2 = x2 - origin
    cos = np.dot(line1, line2) / (np.linalg.norm(line1) * np.linalg.norm(line2))
    return math.acos(np.clip(cos, -1, 1))


class Hand(Protocol):
    def start(self):
        pass

    def set_angle(self, angles: npt.NDArray[np.int32]):
        pass

    def open_hand(self):
        pass

    def close_hand(self):
        pass

    def close(self):
        pass


class InspireSim:
    def __init__(self, ip: str, port: int):
        self.angles = np.zeros(6, dtype=np.int32)

    def start(self):
        pass

    def set_angle(self, angles: npt.NDArray[np.int32]):
        self.angles = angles

    def open_hand(self):
        self.angles = np.zeros(6, dtype=np.int32)

    def close_hand(self):
        self.angles = np.full(6, 1000, dtype=np.int32)

    def close(self):
        pass


class InspireNotSim:
    def __init__(self, ip: str, port: int):
        # imported here so --sim runs without the inspire sdk installed
        from inspire_demos import InspireHandModbus

        self.hand = InspireHandModbus(ip=ip, port=port)

    def start(self):
        self.hand.connect()
        self.hand.set_angle(np.zeros(6, dtype=np.int32))

    def set_angle(self, angles: npt.NDArray[np.int32]):
        self.hand.set_angle(angles)

    def open_hand(self):
        self.hand.perform_open()

    def close_hand(self):
        self.hand.perform_close()

    def close(self):
        self.hand.disconnect()


@dataclass
class HandConfig:
    hand: Literal["left", "right"]
    ip: str
    # buttons that open the hand when not using hand tracking
    buttons: List[robo.Query]
    port: int = 6000

    @staticmethod
    def right() -> HandConfig:
        return HandConfig(
            hand="right",
            ip="192.168.123.211",
            buttons=[
                robo.Query(kind=robo.SensorKind.Btn, name="xr.right.aim"),
                robo.Query(kind=robo.SensorKind.Btn, name="dpad.r2"),
            ],
        )


class Xr2Hand:
    def __init__(self, config: HandConfig, sim: bool = False):
        self.config = config
        self.hand: Hand = (
            InspireSim(config.ip, config.port)
            if sim
            else InspireNotSim(config.ip, config.port)
        )

    def open(self):
        self.hand.start()
        return self

    def close_stream(self):
        self.hand.close()

    def get_pos(self, cur: robo.Device, joint: str) -> npt.NDArray[np.float64] | None:
        pos = cur.get(
            robo.Query(kind=robo.SensorKind.Pos, name=f"hand.{self.config.hand}.{joint}")
        )
        if pos is None:
            return None
        pos = pos.as_pos()
        return np.array([pos.x, pos.y, pos.z])

    def recv_tracking(self, cur: robo.Device):
        angles = np.zeros(6, dtype=np.float64)
        wrist = self.get_pos(cur, "bod")
        if wrist is not None:
            for i, (prox_joint, tip_joint) in enumerate(FINGER_JOINTS):
                prox = self.get_pos(cur, prox_joint)
                tip = self.get_pos(cur, tip_joint)
                if prox is not None and tip is not None:
                    angles[i] = find_angle(prox, wrist, tip)
            # thumb bend and rotate
            index_prox = self.get_pos(cur, "index-finger-phalanx-proximal")
            thumb_prox = self.get_pos(cur, "thumb-phalanx-proximal")
            thumb_tip = self.get_pos(cur, "thumb-tip")
            if index_prox is not None and thumb_prox is not None and thumb_tip is not None:
                angles[4] = find_angle(thumb_prox, index_prox, thumb_tip)
                angles[5] = find_angle(thumb_prox, wrist, thumb_tip)
        self.hand.set_angle((angles / 3.14 * 1000).astype(np.int32))

    def recv_buttons(self, cur: robo.Device):
        for query in self.config.buttons:
            btn = cur.get(query)
            if btn is not None and btn.as_btn().pressed:
                self.hand.open_hand()
                return
        self.hand.close_hand()

    def recv_delta(self, prev: robo.Device, cur: robo.Device):
        if prev.kind == "xr-hand" and cur.kind == "xr-hand":
            self.recv_tracking(cur)
        else:
            self.recv_buttons(cur)
