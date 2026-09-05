"""Bake the verified FR3 home pose into the existing USD link transforms."""
import argparse
import os

import numpy as np
from isaacsim import SimulationApp


HOME = np.array([1.2, -1.2, 1.0, -1.8, -1.57, 0.0])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", required=True)
    args = parser.parse_args()
    app = SimulationApp({"headless": True})
    try:
        import omni
        from pxr import Gf, UsdGeom, UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.stage import open_stage

        asset = os.path.abspath(args.asset)
        if not open_stage(asset):
            raise RuntimeError(f"failed to open {asset}")
        stage = omni.usd.get_context().get_stage()
        robot_path = "/fairino3_v6_robot"
        world = World(stage_units_in_meters=1.0)
        robot = Articulation(robot_path)
        world.reset()
        robot.initialize()
        robot.set_joint_positions(HOME, joint_indices=np.arange(6))
        world.step(render=False)

        bodies = [prim for prim in stage.Traverse()
                  if str(prim.GetPath()).startswith(robot_path + "/Geometry/")
                  and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        cache = UsdGeom.XformCache()
        world_matrices = {prim.GetPath(): cache.GetLocalToWorldTransform(prim) for prim in bodies}
        for prim in sorted(bodies, key=lambda item: len(str(item.GetPath()))):
            parent = prim.GetParent()
            while parent and parent.GetPath() not in world_matrices:
                parent = parent.GetParent()
            local = world_matrices[prim.GetPath()]
            if parent:
                # Gf matrices use row-vector multiplication: local * parent = world.
                local = local * world_matrices[parent.GetPath()].GetInverse()
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTransformOp().Set(Gf.Matrix4d(local))

        stage.GetRootLayer().Save()
        print(f"baked {len(bodies)} rigid link transforms into {asset}")
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
