from typing import Protocol

import numpy as np
import numpy.typing as npt

from .state import ArmState
from .subscriber import G1Subscriber

JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


class Body(Protocol):
    def start(self):
        pass

    def get_joints(self) -> npt.NDArray[np.float64]:
        pass

    def get_battery(self) -> int | None:
        pass

    def close(self):
        pass


class G1BodySim:
    def __init__(self, arm: ArmState):
        self.arm = arm

    def start(self):
        pass

    def get_joints(self) -> npt.NDArray[np.float64]:
        joints = np.zeros(len(JOINT_NAMES))
        joints[15:29] = self.arm.joints
        return joints

    def get_battery(self) -> int | None:
        return 100

    def close(self):
        pass


class G1BodyNotSim:
    def __init__(self):
        # imported here so --sim runs without the unitree sdk installed
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_

        self.low_state = G1Subscriber("rt/lowstate", LowState_, auto_start=False)
        self.bms_state = G1Subscriber("rt/lf/bmsstate", BmsState_, auto_start=False)
        self.joints = np.zeros(len(JOINT_NAMES))

    def start(self):
        self.low_state.start()
        self.bms_state.start()

    def get_joints(self) -> npt.NDArray[np.float64]:
        msg = self.low_state.get_msg_nowait()
        if msg is not None:
            self.joints = np.array(
                [msg.motor_state[i].q for i in range(len(JOINT_NAMES))]
            )
        return self.joints

    def get_battery(self) -> int | None:
        msg = self.bms_state.get_msg_nowait()
        if msg is None:
            return None
        return int(msg.soc)

    def close(self):
        self.low_state.stop()
        self.bms_state.stop()
