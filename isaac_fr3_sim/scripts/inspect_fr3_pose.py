"""Print the authored and simulated FR3 pose without modifying the USD."""
import argparse
import os

from isaacsim import SimulationApp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", required=True)
    parser.add_argument("--frames", type=int, default=120)
    args = parser.parse_args()
    app = SimulationApp({"headless": True})
    try:
        import omni
        from pxr import UsdGeom, UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.stage import open_stage

        asset = os.path.abspath(args.asset)
        if not open_stage(asset):
            raise RuntimeError(f"failed to open {asset}")
        stage = omni.usd.get_context().get_stage()
        robot_path = "/fairino3_v6_robot"
        base_path = robot_path + "/Geometry/base_link"
        wrist_path = base_path + "/shoulder_link/upperarm_link/forearm_link/wrist1_link/wrist2_link/wrist3_link"
        robot_prim = stage.GetPrimAtPath(robot_path)
        def base_translation():
            return tuple(round(float(v), 4) for v in UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(base_path)).ExtractTranslation())
        def wrist_translation():
            return tuple(round(float(v), 4) for v in UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(wrist_path)).ExtractTranslation())
        print("robot xform ops:", [(op.GetOpName(), op.Get()) for op in UsdGeom.Xformable(robot_prim).GetOrderedXformOps()])
        print("base xform ops:", [(op.GetOpName(), op.Get()) for op in UsdGeom.Xformable(stage.GetPrimAtPath(base_path)).GetOrderedXformOps()])
        print("authored wrist translation:", wrist_translation())
        print("robot APIs:", stage.GetPrimAtPath(robot_path).GetAppliedSchemas())
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):
                continue
            name = prim.GetName()
            if name in {"j1", "j2", "j3", "j4", "j5", "j6", "root_joint"}:
                print(name, "path=", prim.GetPath(), "body0=", prim.GetRelationship("physics:body0").GetTargets(),
                      "body1=", prim.GetRelationship("physics:body1").GetTargets(),
                      "target=", prim.GetAttribute("drive:angular:physics:targetPosition").Get(),
                      "state=", prim.GetAttribute("state:angular:physics:position").Get())
        world = World(stage_units_in_meters=1.0)
        robot = Articulation(robot_path)
        world.reset()
        robot.initialize()
        initial = robot.get_joint_positions()[0]
        print("initial pose:", initial.round(4))
        print("initial gripper pose:", initial[6:].round(4))
        print("initial base translation:", base_translation())
        print("initial wrist translation:", wrist_translation())
        for _ in range(args.frames):
            world.step(render=False)
        after = robot.get_joint_positions()[0]
        print("after pose:", after.round(4))
        print("after gripper pose:", after[6:].round(4))
        print("after base translation:", base_translation())
        print("after wrist translation:", wrist_translation())
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
