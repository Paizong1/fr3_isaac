#!/usr/bin/env python3
"""Make the converted FR3 USD portable and add the minimum sensor anchor."""
import argparse
import os
import shutil


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--source", required=True, help="Original fairino3_robotiq.usd")
    p.add_argument("--output", required=True, help="Portable USD to create")
    p.add_argument("--payload-root", required=True, help="Directory containing fairino3_v6_2/payloads")
    args = p.parse_args()

    source = os.path.abspath(args.source)
    output = os.path.abspath(args.output)
    payload_root = os.path.abspath(args.payload_root)
    if not os.path.isfile(source):
        p.error(f"source USD does not exist: {source}")
    if not os.path.isdir(payload_root):
        p.error(f"payload root does not exist: {payload_root}")
    os.makedirs(os.path.dirname(output), exist_ok=True)
    shutil.copy2(source, output)

    # PXR is exposed after Isaac Sim initializes its application environment.
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})
    from pxr import Sdf, Usd, UsdGeom

    stage = Usd.Stage.Open(output)
    if stage is None:
        raise RuntimeError(f"cannot open copied USD: {output}")
    root = stage.GetRootLayer()
    replacements = {
        "../ROS_JIXIEBI/FR3/src/fairino3_v6_robotiq_2/payloads/Physics/mujoco.usda": os.path.join(payload_root, "payloads", "Physics", "mujoco.usda"),
        "../ROS_JIXIEBI/FR3/src/fairino3_v6_robotiq_2/payloads/Physics/physics.usda": os.path.join(payload_root, "payloads", "Physics", "physics.usda"),
        "../ROS_JIXIEBI/FR3/src/fairino3_v6_robotiq_2/payloads/Physics/physx.usda": os.path.join(payload_root, "payloads", "Physics", "physx.usda"),
        "../ROS_JIXIEBI/FR3/src/fairino3_v6_robotiq_2/payloads/base.usda": os.path.join(payload_root, "payloads", "base.usda"),
    }
    for old, new in replacements.items():
        if not os.path.isfile(new):
            raise RuntimeError(f"missing payload: {new}")
        if old in root.GetExternalReferences():
            # Keep the generated USD relocatable with the payload directory.
            relative = os.path.relpath(new, os.path.dirname(output)).replace(os.sep, "/")
            root.UpdateExternalReference(old, relative)

    robot = stage.GetPrimAtPath("/fairino3_v6_robot")
    if not robot:
        raise RuntimeError("/fairino3_v6_robot not found")
    camera_path = "/fairino3_v6_robot/Geometry/base_link/shoulder_link/upperarm_link/forearm_link/wrist1_link/wrist2_link/wrist3_link/wrist_camera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    try:
        camera.GetPrim().ApplyAPI("IsaacCamera")
    except Exception:
        pass
    camera.CreateFocalLengthAttr(24.0)
    camera.CreateHorizontalApertureAttr(36.0)
    camera.CreateClippingRangeAttr((0.02, 5.0))
    camera.GetPrim().CreateAttribute("isaac:camera:width", Sdf.ValueTypeNames.Int).Set(640)
    camera.GetPrim().CreateAttribute("isaac:camera:height", Sdf.ValueTypeNames.Int).Set(480)
    camera.GetPrim().CreateAttribute("isaac:camera:update_rate", Sdf.ValueTypeNames.Double).Set(30.0)
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.AddTranslateOp().Set((0.12, 0.0, 0.06))
    xform.AddRotateXYZOp().Set((0.0, -90.0, 180.0))
    # Visible camera housing: the Isaac Camera prim is a sensor and has no mesh.
    housing = UsdGeom.Cube.Define(stage, camera_path + "/housing")
    housing.AddTranslateOp().Set((0.0, 0.0, 0.0))
    housing.AddScaleOp().Set((0.0075, 0.0075, 0.005))
    housing.CreateDisplayColorAttr([(0.03, 0.03, 0.03)])
    lens = UsdGeom.Cylinder.Define(stage, camera_path + "/lens")
    lens.AddTranslateOp().Set((0.009, 0.0, 0.0))
    lens.AddRotateXYZOp().Set((0.0, 90.0, 0.0))
    lens.AddScaleOp().Set((0.0035, 0.0035, 0.0015))
    lens.CreateDisplayColorAttr([(0.01, 0.01, 0.01)])
    # Minimal Isaac-native scene stand-ins for the Gazebo room assets.
    UsdGeom.Xform.Define(stage, "/World")
    table = UsdGeom.Cube.Define(stage, "/World/Worktable")
    table.AddTranslateOp().Set((0.45, 0.0, 0.15))
    table.AddScaleOp().Set((0.40, 0.40, 0.15))
    table.CreateDisplayColorAttr([(0.25, 0.25, 0.28)])
    floor = UsdGeom.Cube.Define(stage, "/World/Ground")
    floor.AddTranslateOp().Set((0.0, 0.0, -0.02))
    floor.AddScaleOp().Set((2.0, 2.0, 0.02))
    floor.CreateDisplayColorAttr([(0.12, 0.12, 0.12)])
    banana = UsdGeom.Sphere.Define(stage, "/World/Banana")
    banana.AddTranslateOp().Set((0.45, 0.0, 0.42))
    banana.AddScaleOp().Set((0.06, 0.025, 0.025))
    banana.CreateDisplayColorAttr([(0.95, 0.75, 0.05)])
    # Upright FR3 display pose (degrees, matching PhysX angular drive units).
    for name, degrees in (("j1", 0.0), ("j2", -45.0), ("j3", 90.0),
                          ("j4", -45.0), ("j5", 0.0), ("j6", 0.0)):
        joint = stage.GetPrimAtPath(f"/fairino3_v6_robot/Physics/{name}")
        if joint:
            attr = joint.GetAttribute("drive:angular:physics:targetPosition")
            if not attr:
                attr = joint.CreateAttribute("drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float)
            attr.Set(degrees)
    stage.SetDefaultPrim(robot)
    root.Save()
    print(f"created {output}")
    print("prepared portable references and /fairino3_v6_robot/wrist_camera")
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
