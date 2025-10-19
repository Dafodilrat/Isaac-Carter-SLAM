from omni.isaac.kit import SimulationApp
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",
    "exts": {
        "isaacsim.ros2.bridge": {"enabled": True}
    }
})

# These imports must come AFTER SimulationApp creation
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage, get_current_stage
from pxr import Usd
import omni.timeline
import time

# Path to your scene
scene_path = "/isaac_slam/usda/slam.usda"

print(f"[INFO] Opening stage: {scene_path}")
open_stage(scene_path)

# --- Wait until the stage is actually loaded ---
stage = get_current_stage()
timeout = 10.0   # seconds
elapsed = 0.0

while (stage is None or not stage.HasDefaultPrim()) and elapsed < timeout:
    simulation_app.update()
    time.sleep(0.1)
    elapsed += 0.1
    stage = get_current_stage()

if stage is None or not stage.HasDefaultPrim():
    raise RuntimeError("[ERROR] Stage failed to load within timeout.")

print("[INFO] Stage fully loaded.")

# --- Initialize world after stage load ---
world = World()
world.reset()

# --- Start timeline safely ---
timeline = omni.timeline.get_timeline_interface()
timeline.play()
print("[INFO] Timeline started — OmniGraph nodes should now execute.")

# --- Run main loop ---
try:
    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.01)
except KeyboardInterrupt:
    print("\n[INFO] Interrupted by user.")

timeline.stop()
simulation_app.close()
