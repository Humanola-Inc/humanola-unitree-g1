import sys
from dataclasses import dataclass

from humanola import robo
from typing_extensions import Dict
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


@dataclass
class CameraSpec:
    id: int
    desc: robo.CameraDesc
    cam: robo.CameraSpec


if __name__ == "__main__":
    ChannelFactoryInitialize(0)

    # detect cameras
    picked_camera = None
    cameras = robo.list_cameras()
    cam_ids: Dict[int, CameraSpec] = {}

    for id, camera in cameras:
        desc = camera.desc()
        if id not in cam_ids:
            cam_ids[id] = CameraSpec(id=id, desc=desc, cam=camera)
        elif (
            id in cam_ids
            and desc.width > cam_ids[id].desc.width
            and desc.height > cam_ids[id].desc.height
            and desc.frame_rate > cam_ids[id].desc.frame_rate
        ):
            cam_ids[id] = CameraSpec(id=id, desc=desc, cam=camera)

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

    for id, spec in cam_ids.items():
        unitree_g1 = unitree_g1.add_camera(spec.desc.name, spec.cam)
    channel, runtime = unitree_g1.run(on_error=lambda x: print(str(x), file=sys.stderr))
    runtime.wait_for_interrupt()
