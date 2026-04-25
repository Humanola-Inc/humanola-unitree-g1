import math
from dataclasses import dataclass
from typing import List, Literal

import numpy as np
from humanola import controllers, robo
from inspire_demos import InspireHandModbus


def find_angle(origin: np.ndarray, x1: np.ndarray, x2: np.ndarray):
    line1 = x1 - origin
    line2 = x2 - origin
    angle = np.dot(line1, line2) / (np.linalg.norm(line1) * np.linalg.norm(line2))
    return math.acos(angle)


@dataclass
class InspireHandConfig:
    hand: Literal["left", "right"]
    ip: str
    port: int = 6000


@dataclass
class InspireHandButtonConfig:
    buttons: List[str]
    ip: str
    port: int = 6000


@dataclass
class InspireHandGenericConfig:
    hand: Literal["left", "right"]
    ip: str
    buttons: List[str]
    port: int = 6000


def hand_ctrl(
    handedness: Literal["left", "right"],
    cur: controllers.Device,
    hand: InspireHandModbus,
):
    wrist_pos = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.bod")
    )
    thumb_prox = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.thumb-phalanx-proximal")
    )
    thumb_tip = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.thumb-tip")
    )
    index_prox = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.index-finger-phalanx-proximal")
    )
    index_tip = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.index-finger-tip")
    )
    middle_prox = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.middle-finger-phalanx-proximal")
    )
    middle_tip = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.middle-finger-tip")
    )
    ring_prox = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.ring-finger-phalanx-proximal")
    )
    ring_tip = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.ring-finger-tip")
    )
    pinky_prox = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.pinky-finger-phalanx-proximal")
    )
    pinky_tip = cur.get(
        controllers.Query()
        .kind(controllers.SensorKind.Pos)
        .name(f"hand.{handedness}.pinky-finger-tip")
    )

    angles = np.zeros(6, dtype=np.float32)

    wrist = None
    if wrist_pos is not None:
        wrist_pos = wrist_pos.as_pos()
        wrist = np.array([wrist_pos.x, wrist_pos.y, wrist_pos.z])
    if index_prox is not None:
        index_prox = index_prox.as_pos()
        index_prox = np.array([index_prox.x, index_prox.y, index_prox.z])

    # pinky (little)
    if wrist is not None and pinky_prox is not None and pinky_tip is not None:
        pinky_prox = pinky_prox.as_pos()
        pinky_tip = pinky_tip.as_pos()
        angles[0] = find_angle(
            np.array([pinky_prox.x, pinky_prox.y, pinky_prox.z]),
            wrist,
            np.array([pinky_tip.x, pinky_tip.y, pinky_tip.z]),
        )

    # ring
    if wrist is not None and ring_prox is not None and ring_tip is not None:
        ring_prox = ring_prox.as_pos()
        ring_tip = ring_tip.as_pos()
        angles[1] = find_angle(
            np.array([ring_prox.x, ring_prox.y, ring_prox.z]),
            wrist,
            np.array([ring_tip.x, ring_tip.y, ring_tip.z]),
        )

    # middle
    if wrist is not None and middle_prox is not None and middle_tip is not None:
        middle_prox = middle_prox.as_pos()
        middle_tip = middle_tip.as_pos()
        angles[2] = find_angle(
            np.array([middle_prox.x, middle_prox.y, middle_prox.z]),
            wrist,
            np.array([middle_tip.x, middle_tip.y, middle_tip.z]),
        )

    # index
    if wrist is not None and index_prox is not None and index_tip is not None:
        index_tip = index_tip.as_pos()
        angles[3] = find_angle(
            index_prox,
            wrist,
            np.array([index_tip.x, index_tip.y, index_tip.z]),
        )

    # thumb bend and rotate
    if (
        wrist is not None
        and thumb_prox is not None
        and thumb_tip is not None
        and index_prox is not None
    ):
        thumb_prox = thumb_prox.as_pos()
        thumb_tip = thumb_tip.as_pos()

        thumb_prox = np.array([thumb_prox.x, thumb_prox.y, thumb_prox.z])
        thumb_tip = np.array([thumb_tip.x, thumb_tip.y, thumb_tip.z])

        angles[4] = find_angle(
            thumb_prox,
            index_prox,
            thumb_tip,
        )
        angles[5] = find_angle(thumb_prox, wrist, thumb_tip)
    hand.set_angle(
        (angles / 3.14 * 1000).astype(np.int32),
    )


class InspireHandController:
    def __init__(self, config: InspireHandConfig):
        self.config = config
        self.hand = InspireHandModbus(ip=self.config.ip, port=self.config.port)

    def ctrl_beg(self):
        self.hand.connect()
        self.hand.set_angle(np.array([0, 0, 0, 0, 0, 0], dtype=np.int32))

    def ctrl(self, prev: controllers.Device, cur: controllers.Device):
        hand_ctrl(self.config.hand, cur, self.hand)

    def ctrl_end(self):
        self.hand.disconnect()

    def desc(self) -> robo.LoopDesc:
        return robo.LoopDesc(
            name="Inspire Hands",
            desc="Controller for Inspire Hands",
            frame_rate=60,
        )

    def open(self):
        return self


def button_ctrl(
    cur: controllers.Device,
    hand: InspireHandModbus,
    qs: List[controllers.Query],
):
    for q in qs:
        cur_btn = cur.get(q)
        if cur_btn is None:
            continue
        cur_btn = cur_btn.as_btn()
        if cur_btn.pressed:
            hand.perform_open()
            return
    hand.perform_close()


class InspireHandButtonController:
    def __init__(self, config: InspireHandButtonConfig):
        self.config = config
        self.hand = InspireHandModbus(ip=self.config.ip, port=self.config.port)
        self.qs = [
            controllers.Query().kind(controllers.SensorKind.Btn).name(name)
            for name in config.buttons
        ]

    def ctrl_beg(self):
        self.hand.connect()
        self.hand.set_angle(np.array([0, 0, 0, 0, 0, 0], dtype=np.int32))

    def ctrl(self, prev: controllers.Device, cur: controllers.Device):
        button_ctrl(cur, self.hand, self.qs)

    def ctrl_end(self):
        self.hand.disconnect()

    def desc(self) -> robo.LoopDesc:
        return robo.LoopDesc(
            name="Inspire Hands",
            desc="Controller for Inspire Hands",
            frame_rate=60,
        )

    def open(self):
        return self


class InspireHandGenericController:
    def __init__(self, config: InspireHandGenericConfig):
        self.config = config
        self.hand = InspireHandModbus(ip=self.config.ip, port=self.config.port)
        self.qs = [
            controllers.Query().kind(controllers.SensorKind.Btn).name(name)
            for name in config.buttons
        ]

    def ctrl_beg(self):
        self.hand.connect()
        self.hand.set_angle(np.array([0, 0, 0, 0, 0, 0], dtype=np.int32))

    def ctrl(self, prev: controllers.Device, cur: controllers.Device):
        if prev.kind == "xr-hand" and cur.kind == "xr-hand":
            hand_ctrl(self.config.hand, cur, self.hand)
        else:
            button_ctrl(cur, self.hand, self.qs)

    def ctrl_end(self):
        self.hand.disconnect()

    def desc(self) -> robo.LoopDesc:
        return robo.LoopDesc(
            name="Inspire Hands",
            desc="Controller for Inspire Hands",
            frame_rate=60,
        )

    def open(self):
        return self
