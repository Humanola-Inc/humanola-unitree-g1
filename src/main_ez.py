import os

from humanola import constants, robo
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from battery import G1Battery
from controllers import (
    # G1Arm,
    G1ArmConfig,
    InspireHandGenericConfig,
    InspireHandGenericController,
    LocoConfig,
    LocoController,
    XrFull,
    XrFullConfig,
)
from main import ROBO_ID, attach_cameras
from sources import JOINT_FIELDS, JointSource

if __name__ == "__main__":
    ChannelFactoryInitialize(0)
    config = attach_cameras(
        robo.RoboConfig(
            os.environ["HUMANOLA_API_ENDPOINT"],
            os.environ["HUMANOLA_API_KEY"],
            os.environ.get("HUMANOLA_ROBO_ID", ROBO_ID),
        )
        .attach_data(
            constants.SRC_DATA,
            JointSource(),
            60,
            JOINT_FIELDS,
            name="Unitree G1 Joint Data",
            desc="The joint data for unitree G1",
        )
        .attach_device_subscriber(
            constants.DEV_XR_HAND_TOPIC,
            InspireHandGenericController(
                config=InspireHandGenericConfig(
                    ip="192.168.123.211",
                    buttons=["xr.right.aim", "dpad.r2"],
                    hand="right",
                )
            ),
            60,
            name="Inspire Hands",
            desc="Inspire hands control",
        )
        .attach_device_subscriber(
            constants.DEV_XR_CONTROLLER_TOPIC,
            XrFull(
                config=XrFullConfig(
                    loco=LocoConfig.xr_with_hands(),
                    arms=G1ArmConfig(scale=1.0),
                    others=[
                        InspireHandGenericController(
                            config=InspireHandGenericConfig(
                                ip="192.168.123.211",
                                buttons=["xr.right.aim", "dpad.r2"],
                                hand="right",
                            )
                        )
                    ],
                )
            ),
            60,
            name="XR Teloperation",
            desc="Full XR Teleoperation, Hands and Foot",
        )
        .attach_device_subscriber(
            constants.DEV_GAMEPAD_TOPIC,
            LocoController(config=LocoConfig.dpad()),
            60,
            name="PS4 Control",
            desc="Control the G1 unit with a ps4 stick",
        )
        .attach_battery(G1Battery())
    )
    channel, runtime = config.run()
    runtime.wait_for_interrupt()
