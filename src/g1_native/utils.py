import numpy as np
from humanola import controllers


def pose2transform(
    pos: controllers.PositionState, rot: controllers.RotationState
) -> np.ndarray:
    """Convert position and quaternion to a 4x4 homogeneous transformation matrix."""
    x, y, z = pos.x, pos.y, pos.z
    qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w

    R = np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ]
    )

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]

    return T


def convert_to_robot_convention(transform_vr: np.ndarray) -> np.ndarray:
    """Convert VR controller pose to robot convention.

    Note: VR controllers already follow Unitree URDF convention for orientation,
    so we only need to change the basis (coordinate system), not the pose itself.
    This matches the xr_teleoperate implementation for controller mode.
    """
    # Basis conversion matrices (OpenXR <-> Robot convention)
    T_ROBOT_OPENXR = np.array(
        [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
    )

    T_OPENXR_ROBOT = np.array(
        [[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]
    )

    # # Only change basis, no additional rotation needed for controllers
    # # This prevents swapping wristRoll and wristYaw axes
    transform_robot = T_ROBOT_OPENXR @ transform_vr @ T_OPENXR_ROBOT
    return transform_robot
