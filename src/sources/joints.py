import math
from typing import Sequence

from g1_native import G1Subscriber
from humanola import data, robo, streaming
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, MotorState_

RAD_TO_MDEG = 180_000 / math.pi


class JointSource:
    def __init__(self):
        self.sub = G1Subscriber("rt/lowstate", LowState_, auto_start=False)

    def src_beg(self):
        self.sub.start()

    def src(self):
        msg = self.sub.get_msg_nowait()
        if msg is None:
            return streaming.DataFrame.binary(data.Frame().proto_encode())
        frame = data.Frame()
        motor_states: Sequence[MotorState_] = msg.motor_state  # type: ignore
        # Left leg
        frame.add_entry(
            "left_hip_pitch_joint",
            data.Entry.rot(int(motor_states[0].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "left_hip_roll_joint",
            data.Entry.rot(int(motor_states[1].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "left_hip_yaw_joint",
            data.Entry.rot(int(motor_states[2].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "left_knee_joint", data.Entry.rot(int(motor_states[3].q * RAD_TO_MDEG))
        )
        frame.add_entry(
            "left_ankle_pitch_joint",
            data.Entry.rot(int(motor_states[4].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "left_ankle_roll_joint",
            data.Entry.rot(int(motor_states[5].q * RAD_TO_MDEG)),
        )

        # Right leg
        frame.add_entry(
            "right_hip_pitch_joint",
            data.Entry.rot(int(motor_states[6].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "right_hip_roll_joint",
            data.Entry.rot(int(motor_states[7].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "right_hip_yaw_joint",
            data.Entry.rot(int(motor_states[8].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "right_knee_joint", data.Entry.rot(int(motor_states[9].q * RAD_TO_MDEG))
        )
        frame.add_entry(
            "right_ankle_pitch_joint",
            data.Entry.rot(int(motor_states[10].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "right_ankle_roll_joint",
            data.Entry.rot(int(motor_states[11].q * RAD_TO_MDEG)),
        )

        # Torso
        frame.add_entry(
            "waist_yaw_joint", data.Entry.rot(int(motor_states[12].q * RAD_TO_MDEG))
        )
        frame.add_entry(
            "waist_roll_joint", data.Entry.rot(int(motor_states[13].q * RAD_TO_MDEG))
        )
        frame.add_entry(
            "waist_pitch_joint",
            data.Entry.rot(int(motor_states[14].q * RAD_TO_MDEG)),
        )

        # Left arm
        frame.add_entry(
            "left_shoulder_pitch_joint",
            data.Entry.rot(int(motor_states[15].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "left_shoulder_roll_joint",
            data.Entry.rot(int(motor_states[16].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "left_shoulder_yaw_joint",
            data.Entry.rot(int(motor_states[17].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "left_elbow_joint", data.Entry.rot(int(motor_states[18].q * RAD_TO_MDEG))
        )
        frame.add_entry(
            "left_wrist_roll_joint",
            data.Entry.rot(int(motor_states[19].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "left_wrist_pitch_joint",
            data.Entry.rot(int(motor_states[20].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "left_wrist_yaw_joint",
            data.Entry.rot(int(motor_states[21].q * RAD_TO_MDEG)),
        )

        # Right arm
        frame.add_entry(
            "right_shoulder_pitch_joint",
            data.Entry.rot(int(motor_states[22].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "right_shoulder_roll_joint",
            data.Entry.rot(int(motor_states[23].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "right_shoulder_yaw_joint",
            data.Entry.rot(int(motor_states[24].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "right_elbow_joint",
            data.Entry.rot(int(motor_states[25].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "right_wrist_roll_joint",
            data.Entry.rot(int(motor_states[26].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "right_wrist_pitch_joint",
            data.Entry.rot(int(motor_states[27].q * RAD_TO_MDEG)),
        )
        frame.add_entry(
            "right_wrist_yaw_joint",
            data.Entry.rot(int(motor_states[28].q * RAD_TO_MDEG)),
        )
        return streaming.DataFrame.binary(frame.proto_encode())

    def src_end(self):
        self.sub.stop()

    def desc(self):
        return robo.LoopDesc(
            name="Unitree G1 Joint Data",
            desc="The joint data for unitree G1",
            frame_rate=60,
        )

    def open(self):
        return JointSource()
