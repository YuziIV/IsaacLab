# convert_bionic_urdf.py

from pathlib import Path
import os
from isaaclab_tasks.direct.bionic_arm.urdf_importer import ImportURDF
from isaaclab.app import AppLauncher

# Launch Isaac Sim app in non-headless mode
app_launcher = AppLauncher(headless=False)
simulation_app = app_launcher.app

def main():
    # Load URDF and convert to USD if not already done
    urdf_path = "/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/direct/bionic_arm/assets/urdf/EZARM_SLDASM/urdf/EZARM_SLDASM.urdf"
    usd_path = "/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/direct/bionic_arm/assets/usd/bionic_arm.usd"

    if not Path(usd_path).exists():
        print("[INFO] Converting URDF to USD...")
        urdf_importer = ImportURDF(urdf_path, usd_path)
        status, prim_path = urdf_importer.import_urdf_to_usd()
        print("URDF import status:", status, "Prim path:", prim_path)
    else:
        print("[INFO] USD already exists. Skipping import.")

    simulation_app.update()
    simulation_app.run()

if __name__ == "__main__":
    main()
    simulation_app.close()
