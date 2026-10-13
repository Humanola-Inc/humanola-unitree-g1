import argparse
import logging
import pathlib

import numpy as np
from humanola import constants, dataset, robo
from native import (
    JOINT_NAMES,
    ArmController,
    ArmSolver,
    ArmState,
    G1BodyNotSim,
    G1BodySim,
    HandConfig,
    LocoConfig,
    WalkMode,
    Xr2Arm,
    Xr2Hand,
    Xr2Loco,
    XrConfig,
)
from native.body import Body


class XrCtrl:
    def __init__(self, arm: Xr2Arm, loco: Xr2Loco, hand: Xr2Hand):
        self.arm = arm
        self.loco = loco
        self.hand = hand
        self.has_hand = False

    def open(self):
        self.loco.open()
        self.arm.open()
        # the robot is still usable without its hand
        try:
            self.hand.open()
            self.has_hand = True
        except Exception as e:
            logging.warning(f"cannot connect to hand: {e}")
            self.has_hand = False
        return self

    def recv_delta(self, prev: robo.Device, cur: robo.Device):
        prev_walk_mode = self.loco.walk_mode
        self.loco.recv_delta(prev, cur)
        # AI walking drives the arms itself, so release them while it is on
        if self.loco.walk_mode != prev_walk_mode:
            if self.loco.walk_mode == WalkMode.AI:
                self.arm.close_stream()
            else:
                self.arm.open()
        if self.loco.walk_mode == WalkMode.NORMAL:
            self.arm.recv_delta(prev, cur)
        if self.has_hand:
            self.hand.recv_delta(prev, cur)

    def close_stream(self):
        self.loco.close_stream()
        self.arm.close_stream()
        if self.has_hand:
            self.hand.close_stream()


class JointData:
    def __init__(self, body: Body):
        self.body = body

    def open(self):
        return self

    def get_data(self):
        frame = robo.DataFrame()
        for joint in self.body.get_joints():
            frame.attach(dataset.Num(joint))
        return frame

    def close_stream(self):
        pass


class G1Battery:
    def __init__(self, body: Body):
        self.body = body

    def get_battery(self) -> robo.Battery:
        soc = self.body.get_battery()
        if soc is None:
            return robo.Battery()
        return robo.Battery().attach("Main Battery", soc)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    args = parser.parse_args()
    urdf_path = pathlib.Path(__file__).parent.parent / "URDF/g1_body29_hand14.urdf"

    if not args.sim:
        # imported here so --sim runs without the unitree sdk installed
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(0)

    arm = ArmController(
        state=ArmState(
            joints=np.zeros(14),
            solver=ArmSolver(urdf_path),
        ),
        sim=args.sim,
    )
    body = G1BodySim(arm.state) if args.sim else G1BodyNotSim()
    body.start()

    channel, runtime = (
        robo.RoboConfig(
            api_url="https://grpc.humanola.com",
            api_key="<YOUR_API_KEY>",
            robo_id="<YOUR_ROBO_ID>",
        )
        .attach_device_subscriber(
            topic=constants.DEV_XR_CONTROLLER_TOPIC,
            name="XR Teleoperation",
            desc="Full XR teleoperation, arms, hand and walking",
            rate=60,
            v=XrCtrl(
                arm=Xr2Arm(arm, XrConfig.left(), XrConfig.right(), scale=1.0),
                loco=Xr2Loco(LocoConfig.xr(), sim=args.sim),
                hand=Xr2Hand(HandConfig.right(), sim=args.sim),
            ),
        )
        .attach_device_subscriber(
            topic=constants.DEV_XR_HAND_TOPIC,
            name="Inspire Hands",
            desc="Controls the inspire hand with hand tracking",
            rate=60,
            v=Xr2Hand(HandConfig.right(), sim=args.sim),
        )
        .attach_device_subscriber(
            topic=constants.DEV_GAMEPAD_TOPIC,
            name="Gamepad Control",
            desc="Walks the G1 with a gamepad",
            rate=60,
            v=Xr2Loco(LocoConfig.gamepad(), sim=args.sim),
        )
        .attach_battery(G1Battery(body))
        .attach_data(
            topic=constants.SRC_DATA,
            name="Unitree G1 Joints",
            desc="Records the joint position of the G1",
            rate=60,
            v=JointData(body),
            fields=[
                dataset.Field(
                    name=name,
                    dtype=dataset.AngleType(unit=dataset.AngleUnit.RAD, shape=[1]),
                )
                for name in JOINT_NAMES
            ],
        )
        .run()
    )
    try:
        runtime.wait_for_interrupt()
    finally:
        arm.close()
        body.close()
