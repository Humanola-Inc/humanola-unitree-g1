from __future__ import annotations

import multiprocessing
import pathlib
import sys
import time
from dataclasses import dataclass
from enum import IntEnum
from queue import Empty
from typing import List, Literal, Tuple

import casadi
import numpy as np
import pinocchio as pin
from g1_native import G1Subscriber, convert_to_robot_convention, pose2transform
from humanola import controllers, robo
from pinocchio import casadi as cpin
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


class G1_29_JointIndex(IntEnum):
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11
    kWaistYaw = 12
    kWaistRoll = 13
    kWaistPitch = 14
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34

    @staticmethod
    def joint_arm_indexes() -> List[G1_29_JointIndex]:
        return [
            G1_29_JointIndex.kLeftShoulderPitch,
            G1_29_JointIndex.kLeftShoulderRoll,
            G1_29_JointIndex.kLeftShoulderYaw,
            G1_29_JointIndex.kLeftElbow,
            G1_29_JointIndex.kLeftWristRoll,
            G1_29_JointIndex.kLeftWristPitch,
            G1_29_JointIndex.kLeftWristyaw,
            G1_29_JointIndex.kRightShoulderPitch,
            G1_29_JointIndex.kRightShoulderRoll,
            G1_29_JointIndex.kRightShoulderYaw,
            G1_29_JointIndex.kRightElbow,
            G1_29_JointIndex.kRightWristRoll,
            G1_29_JointIndex.kRightWristPitch,
            G1_29_JointIndex.kRightWristYaw,
        ]

    @staticmethod
    def weak_motor_indexes() -> List[G1_29_JointIndex]:
        return [
            G1_29_JointIndex.kLeftAnklePitch,
            G1_29_JointIndex.kRightAnklePitch,
            G1_29_JointIndex.kLeftShoulderPitch,
            G1_29_JointIndex.kLeftShoulderRoll,
            G1_29_JointIndex.kLeftShoulderYaw,
            G1_29_JointIndex.kLeftElbow,
            G1_29_JointIndex.kRightShoulderPitch,
            G1_29_JointIndex.kRightShoulderRoll,
            G1_29_JointIndex.kRightShoulderYaw,
            G1_29_JointIndex.kRightElbow,
        ]

    @staticmethod
    def wrist_motor_indexes() -> List[G1_29_JointIndex]:
        return [
            G1_29_JointIndex.kLeftWristRoll,
            G1_29_JointIndex.kLeftWristPitch,
            G1_29_JointIndex.kLeftWristyaw,
            G1_29_JointIndex.kRightWristRoll,
            G1_29_JointIndex.kRightWristPitch,
            G1_29_JointIndex.kRightWristYaw,
        ]

    @staticmethod
    def waist_motor_indexes() -> List[G1_29_JointIndex]:
        return [
            G1_29_JointIndex.kWaistPitch,
            G1_29_JointIndex.kWaistRoll,
            G1_29_JointIndex.kWaistYaw,
        ]


JOINT_ARM_INDEXES = G1_29_JointIndex.joint_arm_indexes()
WEAK_MOTOR_INDEXES = G1_29_JointIndex.weak_motor_indexes()
WRIST_MOTOR_INDEXES = G1_29_JointIndex.wrist_motor_indexes()
WAIST_MOTOR_INDEXES = G1_29_JointIndex.waist_motor_indexes()
JOINT_LOCK_ID = [
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
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
]
URDF_PATH = pathlib.Path("/app/URDF/g1_body29_hand14.urdf")
MESH_PATH = pathlib.Path("/app/URDF")


class WeightedMovingFilter:
    def __init__(self, weights: List[float], data_size: int = 14):
        self._window_size = len(weights)
        self._weights = np.array(weights)
        assert np.isclose(np.sum(self._weights), 1.0), (
            "[WeightedMovingFilter] the sum of weights list must be 1.0!"
        )
        self._data_size = data_size
        self._filtered_data = np.zeros(self._data_size)
        self._data_queue = []

    def _apply_filter(self):
        if len(self._data_queue) < self._window_size:
            return self._data_queue[-1]

        data_array = np.array(self._data_queue)
        temp_filtered_data = np.zeros(self._data_size)
        for i in range(self._data_size):
            temp_filtered_data[i] = np.convolve(
                data_array[:, i], self._weights, mode="valid"
            )[-1]

        return temp_filtered_data

    def add_data(self, new_data: np.ndarray):
        assert len(new_data) == self._data_size

        if len(self._data_queue) > 0 and np.array_equal(new_data, self._data_queue[-1]):
            return  # skip duplicate data

        if len(self._data_queue) >= self._window_size:
            self._data_queue.pop(0)

        self._data_queue.append(new_data)
        self._filtered_data = self._apply_filter()

    @property
    def filtered_data(self):
        return self._filtered_data


class ArmSolver:
    def __init__(self):
        robot = pin.RobotWrapper.BuildFromURDF(str(URDF_PATH), str(MESH_PATH))
        self.robot = robot.buildReducedRobot(
            list_of_joints_to_lock=JOINT_LOCK_ID,
            reference_configuration=np.zeros(robot.model.nq),
        )
        # End Effector frame, i.e. this makes it easier to control the end effector
        self.left_frame_id = self.robot.model.addFrame(
            pin.Frame(
                "L_ee",
                self.robot.model.getJointId("left_wrist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.05, 0, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )
        # End Effector frame, i.e. this makes it easier to control the end effector
        self.right_frame_id = self.robot.model.addFrame(
            pin.Frame(
                "R_ee",
                self.robot.model.getJointId("right_wrist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.05, 0, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )
        self.cmodel = cpin.Model(self.robot.model)
        self.cdata = self.cmodel.createData()
        # 14 joints
        self.cq = casadi.SX.sym("q", 14, 1)
        self.tf_l = casadi.SX.sym("tf_l", 4, 4)
        self.tf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        self.opti = casadi.Opti()
        # 14 joints on the hands
        self.opti_var_q = self.opti.variable(14)
        self.opti_var_prev_q = self.opti.parameter(14)
        self.opti_par_tf_l = self.opti.parameter(4, 4)
        self.opti_par_tf_r = self.opti.parameter(4, 4)

        # This technically becomes
        self.opti.minimize(
            # 60 positional errors
            60
            * casadi.sumsqr(
                casadi.Function(
                    "pos_err",
                    [self.cq, self.tf_l, self.tf_r],
                    [
                        casadi.vertcat(
                            self.cdata.oMf[self.left_frame_id].translation
                            - self.tf_l[:3, 3],
                            self.cdata.oMf[self.right_frame_id].translation
                            - self.tf_r[:3, 3],
                        )
                    ],
                )(self.opti_var_q, self.opti_par_tf_l, self.opti_par_tf_r)
            )
            +
            # 1.5 rotational log3 errors
            1.5
            * casadi.sumsqr(
                casadi.Function(
                    "rot_err",
                    [self.cq, self.tf_l, self.tf_r],
                    [
                        casadi.vertcat(
                            cpin.log3(
                                self.cdata.oMf[self.left_frame_id].rotation
                                @ self.tf_l[:3, :3].T
                            ),
                            cpin.log3(
                                self.cdata.oMf[self.right_frame_id].rotation
                                @ self.tf_r[:3, :3].T
                            ),
                        )
                    ],
                )(self.opti_var_q, self.opti_par_tf_l, self.opti_par_tf_r)
            )
            # no idea what this is with the angles (make angles small ?)
            + 0.01 * casadi.sumsqr(self.opti_var_q)
            # no idea what this is (make the changes as little as possible ?)
            + 0.2 * casadi.sumsqr(self.opti_var_q - self.opti_var_prev_q),
        )
        # This binds the angle to some limit
        self.opti.subject_to(
            self.opti.bounded(
                self.robot.model.lowerPositionLimit,
                self.opti_var_q,
                self.robot.model.upperPositionLimit,
            )
        )
        self.opti.solver(
            "ipopt",
            {
                "ipopt": {"print_level": 0, "max_iter": 100, "tol": 1e-7},
                "print_time": False,
                "calc_lam_p": False,
            },
        )
        self.smooth_filter = WeightedMovingFilter([0.4, 0.3, 0.2, 0.1], 14)

    def inverse(
        self, prev_joints: np.ndarray, left_ef: np.ndarray, right_ef: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        # set params for previous value before we solve
        self.opti.set_value(self.opti_var_prev_q, prev_joints)
        self.opti.set_value(self.opti_par_tf_l, left_ef)
        self.opti.set_value(self.opti_par_tf_r, right_ef)
        self.opti.set_initial(self.opti_var_q, prev_joints)

        # now we solve
        self.opti.solve()
        joints = self.opti.value(self.opti_var_q)
        joints = np.array([joint for joint in joints])
        self.smooth_filter.add_data(joints)
        joints = self.smooth_filter.filtered_data
        # Solve for tauff
        tauff = pin.rnea(
            self.robot.model,
            self.robot.data,
            joints,
            np.zeros(14),
            np.zeros(14),
        )
        return (np.array([joint for joint in joints]), np.array([tau for tau in tauff]))

    def forward(self, joints: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        data = self.robot.model.createData()
        pin.forwardKinematics(self.robot.model, data, joints)
        pin.updateFramePlacements(self.robot.model, data)
        return data.oMf[self.left_frame_id].homogeneous, data.oMf[
            self.right_frame_id
        ].homogeneous


@dataclass
class ArmConfig:
    kp_low: float = 80.0
    kd_low: float = 4.0
    kp_high: float = 300.0
    kd_high: float = 3.0
    kp_wrist: float = 60.0
    kd_wrist: float = 2.0
    arm_velocity_limit: float = 40.0
    follow_rate: float = 1.0
    control_freq: int = 250


class ArmState:
    def __init__(
        self,
        joints: np.ndarray,
        solver: ArmSolver,
        queue: multiprocessing.Queue[Tuple[np.ndarray, np.ndarray]],
    ):
        assert len(joints) == 14
        self.joints = joints
        self.tauff = np.zeros(14)
        self.left_ef, self.right_ef = solver.forward(self.joints)
        self.solver = solver
        self.queue = queue

    def set_joints_full(self, joints: np.ndarray):
        self.joints = joints
        self.left_ef, self.right_ef = self.solver.forward(self.joints)
        self.tauff = np.zeros(14)
        self.send_state()

    def set_joints(self, left_joint: np.ndarray | None, right_joint: np.ndarray | None):
        if left_joint is not None:
            self.joints[:7] = left_joint
        if right_joint is not None:
            self.joints[7:] = right_joint
        self.left_ef, self.right_ef = self.solver.forward(self.joints)
        self.tauff = np.zeros(14)
        self.send_state()

    def set_with_ef(self, left_ef: np.ndarray, right_ef: np.ndarray):
        self.joints, self.tauff = self.solver.inverse(self.joints, left_ef, right_ef)
        self.left_ef, self.right_ef = left_ef, right_ef
        self.send_state()

    def send_state(self):
        self.queue.put_nowait((self.joints, self.tauff))

    def clear(self):
        try:
            while True:
                self.queue.get_nowait()
        except Exception as _:
            pass


def arm_control_loop(
    queue: multiprocessing.Queue[Tuple[np.ndarray, np.ndarray]],
    config: ArmConfig,
    ev: multiprocessing.Event,
):
    ChannelFactoryInitialize(0)
    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    sub = G1Subscriber("rt/lowstate", LowState_)
    crc = CRC()
    state = sub.get_msg()
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_machine = state.mode_machine
    cmd.mode_pr = 0
    ctrl_dt = 1 / config.control_freq
    for joint in G1_29_JointIndex:
        cmd.motor_cmd[joint.value].mode = 1
        if joint in WRIST_MOTOR_INDEXES:
            cmd.motor_cmd[joint.value].kp = config.kp_wrist
            cmd.motor_cmd[joint.value].kd = config.kd_wrist
        elif joint in WEAK_MOTOR_INDEXES or joint in JOINT_ARM_INDEXES:
            cmd.motor_cmd[joint.value].kp = config.kp_low
            cmd.motor_cmd[joint.value].kd = config.kd_low
        else:
            cmd.motor_cmd[joint.value].kp = config.kp_high
            cmd.motor_cmd[joint.value].kd = config.kd_high
        if joint not in WAIST_MOTOR_INDEXES:
            cmd.motor_cmd[joint.value].q = state.motor_state[joint.value].q
        else:
            cmd.motor_cmd[joint.value].q = 0
    target_joints, tauff = queue.get()
    while ev.is_set():
        start_time = time.perf_counter_ns()
        msg = sub.get_msg()
        cur_joints = np.array([msg.motor_state[id.value].q for id in JOINT_ARM_INDEXES])
        try:
            target_joints, tauff = queue.get_nowait()
        except Empty:
            pass
        delta = target_joints - cur_joints
        motion_scale = np.max(np.abs(delta)) / (config.arm_velocity_limit * ctrl_dt)
        cur_joints = cur_joints + delta / max(motion_scale, 1.0)
        cmd.motor_cmd[G1_29_JointIndex.kNotUsedJoint0.value].mode = 1
        cmd.motor_cmd[G1_29_JointIndex.kNotUsedJoint0.value].q = 1
        for i, joint in enumerate(JOINT_ARM_INDEXES):
            # TODO: Adaptive Low Gains
            cmd.motor_cmd[joint.value].q = cur_joints[i]
            cmd.motor_cmd[joint.value].dq = 0
            cmd.motor_cmd[joint.value].tau = tauff[i]
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)
        duration = (time.perf_counter_ns() - start_time) / 1e9
        if duration < ctrl_dt:
            time.sleep(ctrl_dt - duration)
        else:
            print(f"Control Loop Slows: {round(duration / 1e6, 8)}ms", file=sys.stderr)
    cmd.motor_cmd[G1_29_JointIndex.kNotUsedJoint0.value].mode = 1
    cmd.motor_cmd[G1_29_JointIndex.kNotUsedJoint0.value].q = 0
    pub.Write(cmd)
    pub.Close()


class ArmController:
    def __init__(
        self,
        config: ArmConfig | None = None,
        solver: ArmSolver | None = None,
    ):
        self.solver = solver if solver is not None else ArmSolver()
        self.config = config if config is not None else ArmConfig()
        # Solvers
        self.sub = G1Subscriber("rt/lowstate", LowState_)
        # These are for arm state locking
        self.mp_ctx = multiprocessing.get_context("spawn")
        self.run_loop = self.mp_ctx.Event()
        msg = self.sub.get_msg()
        self.queue = self.mp_ctx.Queue()
        self.state = ArmState(
            np.array(
                [msg.motor_state[id.value].q for id in JOINT_ARM_INDEXES],
                dtype=np.float32,
            ),
            self.solver,
            self.queue,
        )
        self.proc = None

    def beg(self):
        msg = self.sub.get_msg()
        self.state.clear()
        self.state.set_joints_full(
            np.array(
                [msg.motor_state[id.value].q for id in JOINT_ARM_INDEXES],
                dtype=np.float32,
            )
        )
        self.run_loop.set()
        self.proc = self.mp_ctx.Process(
            target=arm_control_loop,
            args=(self.queue, self.config, self.run_loop),
        )
        self.proc.start()

    def end(self):
        self.run_loop.clear()
        if self.proc is not None:
            self.proc.join()

    def move_arms_rel(self, rel_left_tf: np.ndarray, rel_right_tf: np.ndarray):
        prev_left_ef, prev_right_ef = self.state.left_ef, self.state.right_ef
        new_left_ef = prev_left_ef @ rel_left_tf
        new_right_ef = prev_right_ef @ rel_right_tf
        new_left_ef = (
            self.config.follow_rate * new_left_ef
            + (1 - self.config.follow_rate) * prev_left_ef
        )
        new_right_ef = (
            self.config.follow_rate * new_right_ef
            + (1 - self.config.follow_rate) * new_right_ef
        )
        self.state.set_with_ef(new_left_ef, new_right_ef)

    def move_arms_ef(self, left_ef: np.ndarray, right_ef: np.ndarray):
        prev_left_ef, prev_right_ef = self.state.left_ef, self.state.right_ef
        left_ef = (
            self.config.follow_rate * left_ef
            + (1 - self.config.follow_rate) * prev_left_ef
        )
        right_ef = (
            self.config.follow_rate * right_ef
            + (1 - self.config.follow_rate) * prev_right_ef
        )
        self.state.set_with_ef(left_ef, right_ef)

    def move_arms_with_joint(
        self, left_joints: np.ndarray | None, right_joints: np.ndarray | None
    ):
        self.state.set_joints(left_joints, right_joints)

    # These are controls for specific poses

    def move_arms_to_hold(self, h: Literal["left", "right", "both"]):
        if h == "both":
            self.move_arms_with_joint(np.zeros(7), np.zeros(7))
        elif h == "left":
            self.move_arms_with_joint(np.zeros(7), None)
        else:
            self.move_arms_with_joint(None, np.zeros(7))

    def move_arms_to_relax(self, h: Literal["left", "right", "both"]):
        RELAX_LEFT = np.array(
            [
                0.2861,  # Left shoulder pitch (home)
                0.2043,  # Left shoulder roll (home - open arm outward)
                -0.051,  # Left shoulder yaw (home)
                0.9818,  # Left elbow (home - 60% extension)
                0.0631,  # Left wrist roll (home)
                0.0657,  # Left wrist pitch (home)
                -0.0017,  # Left wrist yaw (home)
            ]
        )
        RELAX_RIGHT = np.array(
            [
                0.2912,  # Right shoulder pitch (home)
                -0.2207,  # Right shoulder roll (home - open arm outward)
                0.0177,  # Right shoulder yaw (home)
                0.9886,  # Right elbow (home - 60% extension)
                -0.1403,  # Right wrist roll (home)
                0.025,  # Right wrist pitch (home)
                0.0118,  # Right wrist yaw (home)
            ]
        )
        if h == "both":
            self.move_arms_with_joint(RELAX_LEFT, RELAX_RIGHT)
        elif h == "left":
            self.move_arms_with_joint(RELAX_LEFT, None)
        else:
            self.move_arms_with_joint(None, RELAX_RIGHT)


@dataclass
class G1ArmConfig:
    scale: float = 2.0


class G1Arm:
    def __init__(self, config: G1ArmConfig = G1ArmConfig()):
        self.config = config
        self.left_xr_pos_query = (
            controllers.Query().kind(controllers.SensorKind.Pos).name("xr.left.bod")
        )
        self.left_xr_rot_query = (
            controllers.Query().kind(controllers.SensorKind.Rot).name("xr.left.bod")
        )
        self.right_xr_pos_query = (
            controllers.Query().kind(controllers.SensorKind.Pos).name("xr.right.bod")
        )
        self.right_xr_rot_query = (
            controllers.Query().kind(controllers.SensorKind.Rot).name("xr.right.bod")
        )
        self.left_xr_squeeze_query = (
            controllers.Query().kind(controllers.SensorKind.Btn).name("xr.left.squeeze")
        )
        self.right_xr_squeeze_query = (
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name("xr.right.squeeze")
        )
        self.left_xr_reset_query = (
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name("xr.left.secondary")
        )
        self.right_xr_reset_query = (
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name("xr.right.secondary")
        )
        self.left_xr_hold_query = (
            controllers.Query().kind(controllers.SensorKind.Btn).name("xr.left.primary")
        )
        self.right_xr_hold_query = (
            controllers.Query()
            .kind(controllers.SensorKind.Btn)
            .name("xr.right.primary")
        )
        # JOWI's Version
        self.controller = ArmController()
        pass

    def ctrl_beg(self):
        self.controller.beg()

    def ctrl_reset(self, prev: controllers.Device, cur: controllers.Device):
        prev_left_res_btn = prev.get(self.left_xr_reset_query)
        left_res_btn = cur.get(self.left_xr_reset_query)
        prev_right_res_btn = prev.get(self.right_xr_reset_query)
        right_res_btn = cur.get(self.right_xr_reset_query)
        if (
            prev_left_res_btn is not None
            and prev_left_res_btn.as_btn().pressed
            and left_res_btn is not None
            and not left_res_btn.as_btn().pressed
        ):
            self.controller.move_arms_to_relax("left")
        if (
            prev_right_res_btn is not None
            and prev_right_res_btn.as_btn().pressed
            and right_res_btn is not None
            and not right_res_btn.as_btn().pressed
        ):
            self.controller.move_arms_to_relax("right")

    def ctrl_hold(self, prev: controllers.Device, cur: controllers.Device):
        prev_left_hold_btn = prev.get(self.left_xr_hold_query)
        left_hold_btn = cur.get(self.left_xr_hold_query)
        prev_right_hold_btn = prev.get(self.right_xr_hold_query)
        right_hold_btn = cur.get(self.right_xr_hold_query)
        if (
            prev_left_hold_btn is not None
            and prev_left_hold_btn.as_btn().pressed
            and left_hold_btn is not None
            and not left_hold_btn.as_btn().pressed
        ):
            self.controller.move_arms_to_hold("left")
        if (
            prev_right_hold_btn is not None
            and prev_right_hold_btn.as_btn().pressed
            and right_hold_btn is not None
            and not right_hold_btn.as_btn().pressed
        ):
            self.controller.move_arms_to_hold("right")

    def ctrl(self, prev: controllers.Device, cur: controllers.Device):
        self.ctrl_reset(prev, cur)
        self.ctrl_hold(prev, cur)
        prev_left_pos = prev.get(self.left_xr_pos_query)
        cur_left_pos = cur.get(self.left_xr_pos_query)
        prev_left_rot = prev.get(self.left_xr_rot_query)
        cur_left_rot = cur.get(self.left_xr_rot_query)
        prev_right_pos = prev.get(self.right_xr_pos_query)
        cur_right_pos = cur.get(self.right_xr_pos_query)
        prev_right_rot = prev.get(self.right_xr_rot_query)
        cur_right_rot = cur.get(self.right_xr_rot_query)
        right_sq = cur.get(self.right_xr_squeeze_query)
        left_sq = cur.get(self.left_xr_squeeze_query)
        left_rel_tf = np.identity(4, dtype=np.float64)
        right_rel_tf = np.identity(4, dtype=np.float64)
        should_run = False
        if (
            prev_left_pos is not None
            and cur_left_pos is not None
            and prev_left_rot is not None
            and cur_left_rot is not None
            and left_sq is not None
            and left_sq.as_btn().pressed
        ):
            prev_left_pos = prev_left_pos.as_pos()
            cur_left_pos = cur_left_pos.as_pos()
            prev_left_rot = prev_left_rot.as_rot()
            cur_left_rot = cur_left_rot.as_rot()
            prev_tf = convert_to_robot_convention(
                pose2transform(prev_left_pos, prev_left_rot)
            )
            tf = convert_to_robot_convention(pose2transform(cur_left_pos, cur_left_rot))
            left_rel_tf = np.linalg.inv(prev_tf) @ tf * self.config.scale
            should_run = True
        if (
            prev_right_pos is not None
            and cur_right_pos is not None
            and prev_right_rot is not None
            and cur_right_rot is not None
            and right_sq is not None
            and right_sq.as_btn().pressed
        ):
            prev_right_pos = prev_right_pos.as_pos()
            cur_right_pos = cur_right_pos.as_pos()
            prev_right_rot = prev_right_rot.as_rot()
            cur_right_rot = cur_right_rot.as_rot()
            prev_tf = convert_to_robot_convention(
                pose2transform(prev_right_pos, prev_right_rot)
            )
            tf = convert_to_robot_convention(
                pose2transform(cur_right_pos, cur_right_rot)
            )
            right_rel_tf = np.linalg.inv(prev_tf) @ tf * self.config.scale
            should_run = True
        if should_run:
            self.controller.move_arms_rel(left_rel_tf, right_rel_tf)

    def ctrl_end(self):
        self.controller.end()

    def open(self):
        return self

    def desc(self):
        return robo.LoopDesc(name="G1 Arm", desc="G1 Arm controller", frame_rate=60)
