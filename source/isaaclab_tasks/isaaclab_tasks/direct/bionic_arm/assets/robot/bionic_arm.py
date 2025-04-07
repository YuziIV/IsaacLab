import isaaclab.sim as sim_utils
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg


##
# Configuration
##
usd_path = "/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/direct/bionic_arm/assets/usd/bionic_arm.usd"

BIONIC_ARM_CFG = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path = usd_path,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            retain_accelerations=True,
            max_depenetration_velocity=1000.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.0005,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force"),
        fixed_tendons_props=sim_utils.FixedTendonPropertiesCfg(limit_stiffness=30.0, damping=0.1),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, -0.15, 0.5),
        rot=(0.7071068, 0.7071068, 0.0, 0.0),
        joint_pos={".*": 0.0},
    ),
    actuators={
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=[
                "palm_joint",
                "pinky_joint",
                "ring_joint", 
                "middle_joint",
                "pointer_joint",
                "thumb_swivel_joint",
                "thumb_joint"
            ],
            effort_limit={
                "palm_joint": 5.0,
                "pinky_joint": 2.0,
                "ring_joint": 2.0,
                "middle_joint": 2.0,
                "pointer_joint": 2.0,
                "thumb_swivel_joint": 2.0,
                "thumb_joint": 2.0
            },
            stiffness={
                ".*": 1.0
            },
            damping={
                ".*": 0.1
            },
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Bionic Arm robot."""
