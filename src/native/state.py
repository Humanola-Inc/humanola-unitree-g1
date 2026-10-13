import numpy as np
import numpy.typing as npt

from .solver import ArmSolver


class ArmState:
    def __init__(self, joints: npt.NDArray[np.float64], solver: ArmSolver):
        assert len(joints) == 14
        self.joints = joints
        self.tauff = np.zeros(14)
        self.solver = solver
        self.left_pose, self.right_pose = solver.forward(self.joints)

    def set_with_ef_delta(
        self,
        left_ef_delta: npt.NDArray[np.float64],
        right_ef_delta: npt.NDArray[np.float64],
    ):
        next_left_pose = self.left_pose @ left_ef_delta
        next_right_pose = self.right_pose @ right_ef_delta
        self.joints, self.tauff = self.solver.backward(
            next_left_pose, next_right_pose, self.joints
        )
        self.left_pose, self.right_pose = next_left_pose, next_right_pose

    def set_with_joints(self, joints: npt.NDArray[np.float64]):
        self.left_pose, self.right_pose = self.solver.forward(joints)
        self.joints = joints.copy()
        self.tauff = np.zeros(14)

    def set_left_joints(self, joints: npt.NDArray[np.float64]):
        next_joints = self.joints.copy()
        next_joints[:7] = joints
        self.set_with_joints(next_joints)

    def set_right_joints(self, joints: npt.NDArray[np.float64]):
        next_joints = self.joints.copy()
        next_joints[7:] = joints
        self.set_with_joints(next_joints)
