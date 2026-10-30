import pathlib
from typing import List, Protocol, Tuple

import casadi
import mink
import mujoco
import numpy as np
import numpy.typing as npt
import pinocchio as pin
import pinocchio.casadi as cpin

# Everything except the 14 arm joints is locked for the arm IK
LOCKED_JOINTS = [
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


class WeightedMovingFilter:
    def __init__(self, weights: List[float], data_size: int):
        assert np.isclose(np.sum(weights), 1.0), "filter weights must sum to 1.0"
        self.weights = np.array(weights)
        self.data_size = data_size
        self.queue: List[npt.NDArray[np.float64]] = []
        self.filtered = np.zeros(data_size)

    def add(self, data: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        if len(self.queue) > 0 and np.array_equal(data, self.queue[-1]):
            return self.filtered
        if len(self.queue) >= len(self.weights):
            self.queue.pop(0)
        self.queue.append(data)
        if len(self.queue) < len(self.weights):
            self.filtered = data
        else:
            window = np.array(self.queue)
            self.filtered = np.array(
                [
                    np.convolve(window[:, i], self.weights, mode="valid")[-1]
                    for i in range(self.data_size)
                ]
            )
        return self.filtered


class ArmSolver(Protocol):
    def forward(
        self, joints: npt.NDArray[np.float64]
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]: ...

    def backward(
        self,
        left_pose: npt.NDArray[np.float64],
        right_pose: npt.NDArray[np.float64],
        prev_joints: npt.NDArray[np.float64] | None = None,
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]: ...


class CasadiSolver:
    def __init__(self, urdf_file: pathlib.Path, compile: bool = True):
        self.model = pin.RobotWrapper.BuildFromURDF(
            str(urdf_file), package_dirs=str(urdf_file.parent)
        )
        self.model = self.model.buildReducedRobot(
            list_of_joints_to_lock=LOCKED_JOINTS,
            reference_configuration=np.zeros(self.model.model.nq),
        )
        # End Effector Frames, we will protrude them a little from the
        # wrists (for hands)
        self.left_ef_frame_id = self.model.model.addFrame(
            pin.Frame(
                "left_ee",
                self.model.model.getJointId("left_wrist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.05, 0, 0])),
                pin.FrameType.OP_FRAME,
            )
        )
        self.right_ef_frame_id = self.model.model.addFrame(
            pin.Frame(
                "right_ee",
                self.model.model.getJointId("right_wrist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.05, 0, 0])),
                pin.FrameType.OP_FRAME,
            )
        )
        self.data = self.model.model.createData()
        self.cmodel = cpin.Model(self.model.model)
        self.cdata = self.cmodel.createData()

        # Forwards and Inverse
        self.cq = casadi.SX.sym("q", 14, 1)  # 14 DOF (7 per arm)
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)  # Left end effector transform
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)  # Right end effector transform
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # End effector errors
        left_ef = self.cdata.oMf[self.left_ef_frame_id]
        right_ef = self.cdata.oMf[self.right_ef_frame_id]
        ef_pos_error = casadi.Function(
            "pos_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    left_ef.translation - self.cTf_l[:3, 3],
                    right_ef.translation - self.cTf_r[:3, 3],
                )
            ],
        )
        ef_rot_error = casadi.Function(
            "rot_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(left_ef.rotation @ self.cTf_l[:3, :3].T),
                    cpin.log3(right_ef.rotation @ self.cTf_r[:3, :3].T),
                )
            ],
        )

        # optimisation
        opti = casadi.Opti()
        var_q = opti.variable(14)
        prev_var_q = opti.parameter(14)
        param_tf_l = opti.parameter(4, 4)
        param_tf_r = opti.parameter(4, 4)

        # optimiser error
        opti.subject_to(
            opti.bounded(
                self.model.model.lowerPositionLimit,
                var_q,
                self.model.model.upperPositionLimit,
            )
        )
        opti.minimize(
            60 * casadi.sumsqr(ef_pos_error(var_q, param_tf_l, param_tf_r))
            + 1.5 * casadi.sumsqr(ef_rot_error(var_q, param_tf_l, param_tf_r))
            + 0.2 * casadi.sumsqr(var_q - prev_var_q)
            + 0.01 * casadi.sumsqr(var_q)
        )
        opti.solver(
            "ipopt",
            {
                "ipopt": {"print_level": 0, "max_iter": 100, "tol": 1e-7},
                "print_time": False,
                "calc_lam_p": False,
                "jit": compile,
                "compiler": "shell",
                "jit_options": {
                    "flags": ["-Ofast", "-march=native"],
                    "compiler": "gcc",
                },
                "jit_cleanup": True,
                "expand": True,
            },
        )
        self.solve_fn = opti.to_function(
            "ik",
            [var_q, param_tf_l, param_tf_r, prev_var_q],
            [var_q],
            ["q_init", "tf_l", "tf_r", "prev_q"],
            ["q_sol"],
        )
        self.smooth_filter = WeightedMovingFilter([0.4, 0.3, 0.2, 0.1], 14)

        # Warm-up: forces JIT compilation now instead of on the first real
        # backward(). Skips the smoothing filter so it doesn't see a bogus sample.
        left_pose, right_pose = self.forward(np.zeros(14))
        self.solve_fn(np.zeros(14), left_pose, right_pose, np.zeros(14))

    def forward(
        self, joints: npt.NDArray[np.float64]
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        data = self.model.model.createData()
        pin.forwardKinematics(self.model.model, data, joints)
        pin.updateFramePlacements(self.model.model, data)
        return (
            data.oMf[self.left_ef_frame_id].homogeneous,
            data.oMf[self.right_ef_frame_id].homogeneous,
        )

    def backward(
        self,
        left_pose: npt.NDArray[np.float64],
        right_pose: npt.NDArray[np.float64],
        prev_joints: npt.NDArray[np.float64] | None = None,
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        if prev_joints is None:
            prev_joints = np.zeros(14)
        joints = self.solve_fn(prev_joints, left_pose, right_pose, prev_joints)
        joints = self.smooth_filter.add(np.array(joints).flatten())
        # feed forward torque to hold the arms against gravity
        tauff = pin.rnea(
            self.model.model, self.data, joints, np.zeros(14), np.zeros(14)
        )
        return joints, np.array(tauff)


class MujocoSolver:
    def __init__(self, urdf_file: pathlib.Path, max_iters: int = 20):
        spec = mujoco.MjSpec.from_file(str(urdf_file))
        for name in LOCKED_JOINTS:
            spec.delete(spec.joint(name))
        # only kinematics and inertia are needed, skip loading the meshes
        for geom in [g for g in spec.geoms if g.type == mujoco.mjtGeom.mjGEOM_MESH]:
            spec.delete(geom)
        for mesh in list(spec.meshes):
            spec.delete(mesh)
        # End Effector Frames, we will protrude them a little from the
        # wrists (for hands)
        spec.joint("left_wrist_yaw_joint").parent.add_site(
            name="left_ee", pos=[0.05, 0, 0]
        )
        spec.joint("right_wrist_yaw_joint").parent.add_site(
            name="right_ee", pos=[0.05, 0, 0]
        )
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.configuration = mink.Configuration(self.model)
        self.limits = [mink.ConfigurationLimit(self.model)]
        self.max_iters = max_iters

        # position matters more than orientation, with a light pull towards
        # zero to keep away from odd configurations
        self.left_ef_task = mink.FrameTask(
            frame_name="left_ee",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=0.16,
            lm_damping=1.0,
        )
        self.right_ef_task = mink.FrameTask(
            frame_name="right_ee",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=0.16,
            lm_damping=1.0,
        )
        self.posture_task = mink.PostureTask(self.model, cost=0.003)
        self.posture_task.set_target(np.zeros(self.model.nq))
        self.smooth_filter = WeightedMovingFilter([0.4, 0.3, 0.2, 0.1], 14)

    def forward(
        self, joints: npt.NDArray[np.float64]
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        self.configuration.update(joints)
        return (
            self.configuration.get_transform_frame_to_world(
                "left_ee", "site"
            ).as_matrix(),
            self.configuration.get_transform_frame_to_world(
                "right_ee", "site"
            ).as_matrix(),
        )

    def backward(
        self,
        left_pose: npt.NDArray[np.float64],
        right_pose: npt.NDArray[np.float64],
        prev_joints: npt.NDArray[np.float64] | None = None,
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        if prev_joints is None:
            prev_joints = np.zeros(14)
        self.configuration.update(prev_joints)
        self.left_ef_task.set_target(mink.SE3.from_matrix(left_pose))
        self.right_ef_task.set_target(mink.SE3.from_matrix(right_pose))
        tasks = [self.left_ef_task, self.right_ef_task, self.posture_task]
        for _ in range(self.max_iters):
            # dt = 1 so each step is a full joint displacement
            vel = mink.solve_ik(
                self.configuration,
                tasks,
                1.0,
                "quadprog",
                damping=1e-3,
                limits=self.limits,
            )
            self.configuration.integrate_inplace(vel, 1.0)
            if np.linalg.norm(vel) < 1e-4:
                break
        joints = self.smooth_filter.add(self.configuration.q.copy())
        # feed forward torque to hold the arms against gravity, qfrc_bias is
        # just gravity when the arms are not moving
        self.data.qpos[:] = joints
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        return joints, self.data.qfrc_bias.copy()
