from isaaclab.envs import DirectRLEnv
from isaaclab.utils.configclass import configclass
from isaaclab.sim.spawners.from_files import spawn_ground_plane, GroundPlaneCfg, UsdFileCfg
from isaaclab.sim.spawners.lights import DomeLightCfg
from isaaclab.sim import SimulationCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg, Articulation
from isaaclab_tasks.direct.bionic_arm.urdf_importer import ImportURDF
import os

urdf_path = "/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/direct/bionic_arm/assets/urdf/EZARM_SLDASM/urdf/EZARM_SLDASM.urdf"
usd_path = "/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/direct/bionic_arm/assets/usd/bionic_arm.usd"

class EmptyEnv(DirectRLEnv):
    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

    def _setup_scene(self):
        # Add ground plane
        spawn_ground_plane("/World/ground", GroundPlaneCfg())

        # Import the robot URDF to USD (only once)
        if not os.path.exists(usd_path):
            urdf_importer = ImportURDF(urdf_path, usd_path)
            status, prim_path = urdf_importer.import_urdf_to_usd()
            print("URDF import status:", status, "Prim path:", prim_path)

        # Spawn the robot into the stage
        robot_cfg = ArticulationCfg(
            prim_path="/World/Robot",
            spawn=UsdFileCfg(usd_path=usd_path),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, 0.0),
                rot=(1.0, 0.0, 0.0, 0.0),
                joint_pos={".*": 0.0},
            ),
        )
        self.robot = Articulation(cfg=self.cfg.robot_cfg)

        self.scene.articulations["robot"] = self.robot

        # Add light
        light_cfg = DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

        # Set up environment replication (this still works even with 1 env)
        self.scene.clone_environments(copy_from_source=False)

    def _get_observations(self):
        # Return dummy observations
        obs = {"policy": self.scene.env_origins.clone()}
        return obs

    def _get_rewards(self):
        return 0.0 * self.scene.env_origins[:, 0]

    def _get_dones(self):
        return self.reset_buf.clone(), self.reset_buf.clone()

    def _pre_physics_step(self, actions):
        self.actions = actions.clone()  # dummy buffer

    def _apply_action(self):
        # No-op to satisfy the environment loop
        pass
