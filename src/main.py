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
from sources import JOINT_FIELDS, JointSource

if __name__ == "__main__":
    ChannelFactoryInitialize(0)
    channel, runtime = (
        robo.Robo(
            api_url="https://grpc.humanola.com",
            api_key="<YOUR_API_KEY>",
        )
        .attach_source(
            robo.LoopDesc(
                name="Unitree G1 Joint Data",
                desc="The joint data for unitree G1",
                rate=60,
                topic=constants.SRC_DATA,
                fields=JOINT_FIELDS,
            ),
            JointSource(),
        )
        .attach_controller(
            robo.LoopDesc(
                name="Inspire Hands",
                desc="Inspire hands control",
                rate=60,
                topic=constants.DEV_XR_HAND_TOPIC,
            ),
            InspireHandGenericController(
                config=InspireHandGenericConfig(
                    ip="192.168.123.211",
                    buttons=["xr.right.aim", "dpad.r2"],
                    hand="right",
                )
            ),
        )
        .attach_controller(
            robo.LoopDesc(
                name="XR Teloperation",
                desc="Full XR Teleoperation, Hands and Foot",
                rate=60,
                topic=constants.DEV_XR_CONTROLLER_TOPIC,
            ),
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
        )
        .attach_controller(
            robo.LoopDesc(
                name="PS4 Control",
                desc="Control the G1 unit with a ps4 stick",
                rate=60,
                topic=constants.DEV_GAMEPAD_TOPIC,
            ),
            LocoController(config=LocoConfig.dpad()),
        )
        .attach_battery(G1Battery())
        .auto_discover_cameras()
        .verbose()
        .run()
    )
    runtime.wait_for_interrupt()
