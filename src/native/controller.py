import multiprocessing as mp
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from multiprocessing.connection import Connection
from multiprocessing.synchronize import Event
from typing import Protocol

import numpy as np
import numpy.typing as npt

from .state import ArmState
from .subscriber import G1Subscriber

# DDS does not survive a fork, and neither do the humanola runtime (grpc) threads
mp_ctx = mp.get_context("spawn")


class G1JointIndex(IntEnum):
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28
    # index 29 is the arm_sdk weight (0 = released, 1 = arm_sdk in control)
    NotUsed0 = 29
    NotUsed1 = 30
    NotUsed2 = 31
    NotUsed3 = 32
    NotUsed4 = 33
    NotUsed5 = 34


ARM_JOINTS = [G1JointIndex(i) for i in range(15, 29)]
WEAK_JOINTS = [
    G1JointIndex.LeftAnklePitch,
    G1JointIndex.RightAnklePitch,
    G1JointIndex.LeftShoulderPitch,
    G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,
    G1JointIndex.LeftElbow,
    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
]
WRIST_JOINTS = [
    G1JointIndex.LeftWristRoll,
    G1JointIndex.LeftWristPitch,
    G1JointIndex.LeftWristYaw,
    G1JointIndex.RightWristRoll,
    G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
]
WAIST_JOINTS = [
    G1JointIndex.WaistYaw,
    G1JointIndex.WaistRoll,
    G1JointIndex.WaistPitch,
]


@dataclass
class ArmGains:
    kp_low: float = 80.0
    kd_low: float = 4.0
    kp_high: float = 300.0
    kd_high: float = 3.0
    kp_wrist: float = 60.0
    kd_wrist: float = 2.0
    velocity_limit: float = 40.0
    control_freq: int = 250


class Arm(Protocol):
    def __init__(self, gains: ArmGains):
        pass

    def start(self):
        pass

    def update(self, joints: npt.NDArray[np.float64], tauff: npt.NDArray[np.float64]):
        pass

    def get_joints(self) -> npt.NDArray[np.float64]:
        pass

    def close(self):
        pass


class G1ArmSim:
    def __init__(self, gains: ArmGains):
        self.gains = gains
        self.joints = np.zeros(14)

    def start(self):
        pass

    def update(self, joints: npt.NDArray[np.float64], tauff: npt.NDArray[np.float64]):
        self.joints = joints

    def get_joints(self) -> npt.NDArray[np.float64]:
        return self.joints

    def close(self):
        pass


class G1ArmNotSim:
    def __init__(self, gains: ArmGains):
        self.gains = gains

    def start(self):
        # imported here so --sim runs without the unitree sdk installed
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
        )
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        ChannelFactoryInitialize(0)
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()
        self.sub = G1Subscriber("rt/lowstate", LowState_)
        self.crc = CRC()
        state = self.sub.get_msg()
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.cmd.mode_machine = state.mode_machine
        self.cmd.mode_pr = 0
        for joint in G1JointIndex:
            self.cmd.motor_cmd[joint].mode = 1
            if joint in WRIST_JOINTS:
                self.cmd.motor_cmd[joint].kp = self.gains.kp_wrist
                self.cmd.motor_cmd[joint].kd = self.gains.kd_wrist
            elif joint in WEAK_JOINTS or joint in ARM_JOINTS:
                self.cmd.motor_cmd[joint].kp = self.gains.kp_low
                self.cmd.motor_cmd[joint].kd = self.gains.kd_low
            else:
                self.cmd.motor_cmd[joint].kp = self.gains.kp_high
                self.cmd.motor_cmd[joint].kd = self.gains.kd_high
            if joint in WAIST_JOINTS:
                self.cmd.motor_cmd[joint].q = 0
            else:
                self.cmd.motor_cmd[joint].q = state.motor_state[joint].q
        self.target_lock = threading.Lock()
        self.target_joints = np.array([state.motor_state[j].q for j in ARM_JOINTS])
        self.target_tauff = np.zeros(14)
        self.is_closed = threading.Event()
        # arm_sdk has to be streamed to continuously, so it gets its own thread
        self.ctrl_thread = threading.Thread(target=self.ctrl_loop, daemon=True)
        self.ctrl_thread.start()

    def ctrl_loop(self):
        ctrl_dt = 1 / self.gains.control_freq
        while not self.is_closed.is_set():
            start = time.perf_counter()
            with self.target_lock:
                target_joints, tauff = self.target_joints, self.target_tauff
            cur_joints = self.get_joints()
            # limit how far the arms can move in a single tick
            delta = target_joints - cur_joints
            motion_scale = np.max(np.abs(delta)) / (
                self.gains.velocity_limit * ctrl_dt
            )
            cur_joints = cur_joints + delta / max(motion_scale, 1.0)
            self.cmd.motor_cmd[G1JointIndex.NotUsed0].q = 1
            for i, joint in enumerate(ARM_JOINTS):
                self.cmd.motor_cmd[joint].q = cur_joints[i]
                self.cmd.motor_cmd[joint].dq = 0
                self.cmd.motor_cmd[joint].tau = tauff[i]
            self.cmd.crc = self.crc.Crc(self.cmd)
            self.pub.Write(self.cmd)
            elapsed = time.perf_counter() - start
            if elapsed < ctrl_dt:
                time.sleep(ctrl_dt - elapsed)

    def update(self, joints: npt.NDArray[np.float64], tauff: npt.NDArray[np.float64]):
        with self.target_lock:
            self.target_joints, self.target_tauff = joints, tauff

    def get_joints(self) -> npt.NDArray[np.float64]:
        msg = self.sub.get_msg()
        return np.array([msg.motor_state[j].q for j in ARM_JOINTS])

    def close(self):
        self.is_closed.set()
        self.ctrl_thread.join()
        # hand the arms back to the locomotion controller
        self.cmd.motor_cmd[G1JointIndex.NotUsed0].q = 0
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)
        self.pub.Close()
        self.sub.stop()


@dataclass
class JointCtrl:
    joints: npt.NDArray[np.float64]
    tauff: npt.NDArray[np.float64]


class GetJoint:
    pass


class Close:
    pass


def offshore_controller(
    gains: ArmGains,
    sim: bool,
    is_ready: Event,
    ctrl_rx: Connection,
    joints_rx: Connection,
) -> None:
    # the ik holds the gil for a long time, keeping arm_sdk in its own process
    # stops it from missing its control deadline
    arm: Arm = G1ArmSim(gains) if sim else G1ArmNotSim(gains)
    arm.start()

    def joints_loop():
        while True:
            ev = joints_rx.recv()
            if isinstance(ev, Close):
                break
            elif isinstance(ev, GetJoint):
                joints_rx.send(arm.get_joints())

    def ctrl_loop():
        while True:
            ev = ctrl_rx.recv()
            if isinstance(ev, Close):
                break
            elif isinstance(ev, JointCtrl):
                arm.update(ev.joints, ev.tauff)

    joints_thread = threading.Thread(target=joints_loop, daemon=True)
    ctrl_thread = threading.Thread(target=ctrl_loop, daemon=True)
    joints_thread.start()
    ctrl_thread.start()
    is_ready.set()
    try:
        joints_thread.join()
        ctrl_thread.join()
    finally:
        arm.close()


class ArmController:
    def __init__(
        self,
        state: ArmState,
        gains: ArmGains | None = None,
        sim: bool = False,
    ):
        self.state = state
        self.gains = gains if gains is not None else ArmGains()
        self.sim = sim
        self.process = None

    def __start_process(self):
        self.is_ready = mp_ctx.Event()
        ctrl_rx, self.ctrl_tx = mp_ctx.Pipe(duplex=False)
        self.joints_tx, joints_rx = mp_ctx.Pipe()
        self.process = mp_ctx.Process(
            target=offshore_controller,
            args=(
                self.gains,
                self.sim,
                self.is_ready,
                ctrl_rx,
                joints_rx,
            ),
        )
        self.process.start()
        self.is_ready.wait()

    def open(self):
        if self.process is not None:
            return
        self.__start_process()
        self.state.set_with_joints(self.get_joints())
        self.update()

    def update(self):
        if self.process is None:
            return
        self.ctrl_tx.send(JointCtrl(self.state.joints, self.state.tauff))

    def get_joints(self) -> npt.NDArray[np.float64]:
        self.joints_tx.send(GetJoint())
        return self.joints_tx.recv()

    def close(self):
        if self.process is None:
            return
        self.joints_tx.send(Close())
        self.ctrl_tx.send(Close())
        self.process.join()
        self.process = None
