"""Patch the existing complete USD without rebuilding from the source asset."""
import argparse
import os
from isaacsim import SimulationApp


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--asset", required=True)
    args = p.parse_args()
    app = SimulationApp({"headless": True})
    try:
        import omni
        from pxr import Sdf, Usd, UsdGeom, UsdPhysics
        path = os.path.abspath(args.asset)
        stage = Usd.Stage.Open(path)
        if stage is None:
            raise RuntimeError(f"cannot open {path}")
        robot = "/fairino3_v6_robot"
        UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        # Keep user-authored scene/physics and only add missing calibration opinions.
        robot_xform = UsdGeom.Xformable(stage.GetPrimAtPath(robot))
        for op in robot_xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                op.Set((0.0, 0.0, 0.0))
        base = stage.GetPrimAtPath(robot + "/Geometry/base_link")
        UsdPhysics.RigidBodyAPI.Apply(base)
        base.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(False)
        root_joint = stage.GetPrimAtPath(robot + "/Physics/root_joint")
        # Restore the importer-authored root-joint topology.  Alternative
        # world/anchor constraints made this articulation invalid or unstable.
        if root_joint:
            root_joint.GetRelationship("physics:body0").SetTargets([stage.GetPrimAtPath(robot).GetPath()])
            root_joint.GetRelationship("physics:body1").SetTargets([base.GetPath()])
            enabled = root_joint.GetAttribute("physics:jointEnabled")
            if enabled:
                enabled.Set(True)
        # Per-link visual hulls overlap at joints; they must not collide with other
        # links in this same articulation.
        PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(robot)).GetEnabledSelfCollisionsAttr().Set(False)
        # Keep only the sensor camera; remove optional visual housing geometry.
        for child in ("housing", "lens"):
            prim = stage.GetPrimAtPath(robot + "/Geometry/base_link/shoulder_link/upperarm_link/forearm_link/wrist1_link/wrist2_link/wrist3_link/wrist_camera/" + child)
            if prim:
                stage.RemovePrim(prim.GetPath())
        # Values mirror MoveIt initial_positions.yaml; PhysX angular values are degrees.
        for name, degrees in (("j1", 68.754935), ("j2", -68.754935), ("j3", 57.295780),
                              ("j4", -103.132403), ("j5", -89.954374), ("j6", 0.0)):
            joint = stage.GetPrimAtPath(f"{robot}/Physics/{name}")
            if joint:
                target = joint.GetAttribute("drive:angular:physics:targetPosition")
                if not target:
                    target = joint.CreateAttribute("drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float)
                target.Set(degrees)
                attr = joint.GetAttribute("state:angular:physics:position")
                if not attr:
                    attr = joint.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float)
                attr.Set(degrees)
                for drive_name, drive_value in (("drive:angular:physics:stiffness", 500.0),
                                                ("drive:angular:physics:damping", 50.0),
                                                ("drive:angular:physics:maxForce", 150.0)):
                    drive = joint.GetAttribute(drive_name)
                    if not drive:
                        drive = joint.CreateAttribute(drive_name, Sdf.ValueTypeNames.Float)
                    drive.Set(drive_value)
        # Do not place colliders on imported visual meshes.  The per-link visual
        # hulls overlap at joints and destabilize this articulation; dedicated
        # collision proxies will be added in a later, separately validated step.
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(robot + "/Geometry/"):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
                prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        for prim in stage.Traverse():
            if prim.GetName() in {"robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
                                  "robotiq_85_left_finger_tip_joint", "robotiq_85_right_finger_tip_joint"}:
                joint = prim
                for attr_name, value in (("physics:lowerLimit", -45.8366), ("physics:upperLimit", 45.8366)):
                    attr = joint.GetAttribute(attr_name)
                    if not attr:
                        attr = joint.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float)
                    attr.Set(value)
        for scene_path in ("/World/Ground", "/World/Worktable", "/World/Banana"):
            UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(scene_path))
        banana = stage.GetPrimAtPath("/World/Banana")
        UsdPhysics.RigidBodyAPI.Apply(banana)
        mass = UsdPhysics.MassAPI.Apply(banana).GetMassAttr()
        mass.Set(0.12)
        stage.GetRootLayer().Save()
        print(f"calibrated existing USD: {path}")
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
