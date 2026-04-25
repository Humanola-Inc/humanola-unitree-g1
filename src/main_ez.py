import sys

from humanola import robo
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
from sources import JointSource

if __name__ == "__main__":
    ChannelFactoryInitialize(0)

    # detect cameras
    picked_camera = None
    cameras = robo.list_cameras()
    for id, camera in cameras:
        desc = camera.desc()
        if desc.width == 1920 and desc.height == 1080 and desc.frame_rate == 30:
            picked_camera = camera

    unitree_g1 = (
        robo.Robo.new_default()
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
        .set_battery(G1Battery())
    )
    if picked_camera is not None:
        unitree_g1 = unitree_g1.add_camera("unitree-camera", picked_camera)
    channel, runtime = unitree_g1.run(on_error=lambda x: print(str(x), file=sys.stderr))
    runtime.wait_for_interrupt()
