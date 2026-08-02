import math
from typing import Sequence

from humanola import robo, transport
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, MotorState_

from g1_native import G1Subscriber

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

RAD_TO_DEG = 180 / math.pi

JOINT_FIELDS = [
    robo.Field.tensor(name, robo.TDType.Angle, RAD_TO_DEG, "rad", [1])
    for name in JOINT_NAMES
]


class JointSource:
    def __init__(self):
        self.sub = G1Subscriber("rt/lowstate", LowState_, auto_start=False)
        self.q = [0.0] * len(JOINT_NAMES)

    def src(self) -> transport.RawPacket:
        msg = self.sub.get_msg_nowait()
        if msg is not None:
            motor_states: Sequence[MotorState_] = msg.motor_state  # type: ignore
            self.q = [motor_states[i].q for i in range(len(JOINT_NAMES))]
        row = robo.Row()
        for q in self.q:
            row.add([q])
        return transport.RawPacket(row.proto_encode())

    def open(self):
        self.sub.start()
        return self
