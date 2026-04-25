import sys

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
from humanola import robo
from sources import JointSource
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

if __name__ == "__main__":
    ChannelFactoryInitialize(0)
    channel, runtime = (
        robo.Robo(
            url="https://grpc.humanola.com",
            api_key="<YOUR_API_KEY>",
            robo_id="<YOUR_ROBO_ID>",
        )
        .add_source("data", JointSource())
        .add_controller(
            "controller",
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
        .add_controller("controller", LocoController(config=LocoConfig.dpad()))
        # .add_controller("controller", G1Arm(config=G1ArmConfig(scale=1.0)))
        # .add_controller(
        #     "controller",
        #     InspireHandGenericController(
        #         config=InspireHandGenericConfig(
        #             ip="192.168.123.211",
        #             buttons=["xr.right.aim", "dpad.r2"],
        #             hand="right",
        #         )
        #     ),
        # )
        .add_camera(
            "vr-cam",
            robo.UsbCameraSpec.new_auto(
                kind=robo.CameraKind.Stereo,
                width=3840,
                height=1920,
                frame_rate=30,
                name="VR Cam",
                id=0,
            ),
        )
        .add_camera(
            "intel-realsense",
            robo.UsbCameraSpec.new_auto(
                name="Intel Realsense Camera",
                kind=robo.CameraKind.Mono,
                width=1920,
                height=1080,
                frame_rate=30,
                id=6,
            ),
        )
        .set_battery(G1Battery())
        .verbose()
        .run(on_error=lambda x: print(str(x), file=sys.stderr))
    )
    runtime.wait_for_interrupt()
