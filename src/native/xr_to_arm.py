from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
from humanola import robo

from .controller import ArmController

RELAX_LEFT_JOINTS = np.array(
    [0.2861, 0.2043, -0.051, 0.9818, 0.0631, 0.0657, -0.0017], dtype=np.float64
)
RELAX_RIGHT_JOINTS = np.array(
    [0.2912, -0.2207, 0.0177, 0.9886, -0.1403, 0.025, 0.0118], dtype=np.float64
)
HOLD_JOINTS = np.zeros(7, dtype=np.float64)


def quat_pos_to_pose(
    quat: npt.NDArray[np.float64], pos: npt.NDArray[np.float64]
) -> npt.NDArray[np.float64]:
    x, y, z, w = quat / np.linalg.norm(quat)  # normalize to guard against drift
    pose = np.eye(4)
    pose[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]
    pose[:3, 3] = pos
    return pose


@dataclass
class XrConfig:
    ef_act: robo.Query
    ef_pos: robo.Query
    ef_rot: robo.Query
    arm_relax: robo.Query
    arm_hold: robo.Query

    @staticmethod
    def left() -> XrConfig:
        return XrConfig(
            ef_act=robo.Query(kind=robo.SensorKind.Btn, name="xr.left.squeeze"),
            ef_pos=robo.Query(kind=robo.SensorKind.Pos, name="xr.left.bod"),
            ef_rot=robo.Query(kind=robo.SensorKind.Rot, name="xr.left.bod"),
            arm_relax=robo.Query(kind=robo.SensorKind.Btn, name="xr.left.secondary"),
            arm_hold=robo.Query(kind=robo.SensorKind.Btn, name="xr.left.primary"),
        )

    @staticmethod
    def right() -> XrConfig:
        return XrConfig(
            ef_act=robo.Query(kind=robo.SensorKind.Btn, name="xr.right.squeeze"),
            ef_pos=robo.Query(kind=robo.SensorKind.Pos, name="xr.right.bod"),
            ef_rot=robo.Query(kind=robo.SensorKind.Rot, name="xr.right.bod"),
            arm_relax=robo.Query(kind=robo.SensorKind.Btn, name="xr.right.secondary"),
            arm_hold=robo.Query(kind=robo.SensorKind.Btn, name="xr.right.primary"),
        )


def is_released(prev: robo.Device, cur: robo.Device, query: robo.Query) -> bool:
    prev_btn = prev.get(query)
    cur_btn = cur.get(query)
    return (
        prev_btn is not None
        and cur_btn is not None
        and prev_btn.as_btn().pressed
        and not cur_btn.as_btn().pressed
    )


class Xr2Arm:
    # OpenXR -> Unitree URDF basis change, the controllers already follow the
    # URDF orientation so only the basis needs to change
    XR_TO_ROBOT = np.array(
        [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=np.float64
    )
    ROBOT_TO_XR = XR_TO_ROBOT.T

    def __init__(
        self,
        arm: ArmController,
        left: XrConfig,
        right: XrConfig,
        scale: float = 1.0,
    ):
        self.arm = arm
        self.left = left
        self.right = right
        self.scale = scale

    def open(self):
        self.arm.open()

    def close_stream(self):
        self.arm.close()

    def ef_delta(
        self, prev: robo.Device, cur: robo.Device, config: XrConfig
    ) -> npt.NDArray[np.float64] | None:
        cur_act_btn = cur.get(config.ef_act)
        if cur_act_btn is None or not cur_act_btn.as_btn().pressed:
            return None
        prev_position = prev.get(config.ef_pos)
        prev_rotation = prev.get(config.ef_rot)
        cur_position = cur.get(config.ef_pos)
        cur_rotation = cur.get(config.ef_rot)
        if (
            prev_position is None
            or prev_rotation is None
            or cur_position is None
            or cur_rotation is None
        ):
            return None
        prev_position = prev_position.as_pos()
        prev_rotation = prev_rotation.as_rot()
        cur_position = cur_position.as_pos()
        cur_rotation = cur_rotation.as_rot()
        prev_pose = (
            Xr2Arm.XR_TO_ROBOT
            @ quat_pos_to_pose(
                np.array(
                    [prev_rotation.x, prev_rotation.y, prev_rotation.z, prev_rotation.w]
                ),
                np.array([prev_position.x, prev_position.y, prev_position.z]),
            )
            @ Xr2Arm.ROBOT_TO_XR
        )
        cur_pose = (
            Xr2Arm.XR_TO_ROBOT
            @ quat_pos_to_pose(
                np.array(
                    [cur_rotation.x, cur_rotation.y, cur_rotation.z, cur_rotation.w]
                ),
                np.array([cur_position.x, cur_position.y, cur_position.z]),
            )
            @ Xr2Arm.ROBOT_TO_XR
        )
        delta_pose = np.linalg.inv(prev_pose) @ cur_pose
        delta_pose[:3, 3] *= self.scale
        return delta_pose

    def recv_delta(self, prev: robo.Device, cur: robo.Device):
        # relax
        if is_released(prev, cur, self.left.arm_relax):
            self.arm.state.set_left_joints(RELAX_LEFT_JOINTS)
            self.arm.update()
            return
        if is_released(prev, cur, self.right.arm_relax):
            self.arm.state.set_right_joints(RELAX_RIGHT_JOINTS)
            self.arm.update()
            return
        # hold
        if is_released(prev, cur, self.left.arm_hold):
            self.arm.state.set_left_joints(HOLD_JOINTS)
            self.arm.update()
            return
        if is_released(prev, cur, self.right.arm_hold):
            self.arm.state.set_right_joints(HOLD_JOINTS)
            self.arm.update()
            return
        # position and rotation, both arms share one solve as they share the torso
        left_delta = self.ef_delta(prev, cur, self.left)
        right_delta = self.ef_delta(prev, cur, self.right)
        if left_delta is None and right_delta is None:
            return
        self.arm.state.set_with_ef_delta(
            left_delta if left_delta is not None else np.eye(4),
            right_delta if right_delta is not None else np.eye(4),
        )
        self.arm.update()
