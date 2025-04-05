# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

# URDF import, configuration and simulation sample
kit = SimulationApp({"renderer": "RaytracedLighting", "headless": False})
import omni.kit.commands
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.extensions import get_extension_path_from_name
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics

# Setting up import configuration:
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = True
import_config.distance_scale = 1.0

# Import URDF, prim_path contains the path the path to the usd prim in the stage.
dest_path = "/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/direct/bionic_arm/assets/usd/bionic_arm.usd"
status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path ="/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/direct/bionic_arm/assets/urdf/EZARM_SLDASM/urdf/EZARM_SLDASM.urdf",
    import_config=import_config,
    get_articulation_root=False,
    dest_path = dest_path
)

