import os
import sys
import numpy as np
from isaacsim import SimulationApp

asset = os.path.abspath(sys.argv[1])
app = SimulationApp({"headless": False})
from isaacsim.core.utils.stage import open_stage
if not open_stage(asset):
    raise RuntimeError(asset)
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

# PhysX joint states are only evaluated after a simulation reset.  Do that once
# before the first viewport frame, then pause: the opened scene is already home.
world = World(stage_units_in_meters=1.0)
robot = Articulation("/fairino3_v6_robot")
world.reset()
robot.initialize()
robot.set_joint_positions(np.array([1.2, -1.2, 1.0, -1.8, -1.57, 0.0]), joint_indices=np.arange(6))
world.step(render=False)
world.pause()
app.update()
print(f"Opened USD: {asset}", flush=True)
while app.is_running():
    app.update()
app.close()
