"""Create a configured Windows Isaac Sim stage from the user-provided combined USD."""
import argparse
import os

from isaacsim import SimulationApp


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--source", default=os.path.join(os.path.dirname(__file__), "..", "..", "..", "fairino3_robotiq.usd"))
    p.add_argument("--output", default=os.path.join(os.path.dirname(__file__), "..", "assets", "fairino3_robotiq_complete.usd"))
    args = p.parse_args()
    app = SimulationApp({"headless": True})
    try:
        import omni
        from pxr import Gf, Sdf, UsdGeom
        from isaacsim.core.utils.stage import open_stage
        source = os.path.abspath(args.source)
        output = os.path.abspath(args.output)
        if not open_stage(source):
            raise RuntimeError(f"failed to open {source}")
        app.update()
        stage = omni.usd.get_context().get_stage()
        robot = "/fairino3_v6_robot"
        if not stage.GetPrimAtPath(robot):
            raise RuntimeError(f"missing {robot}")
        # Add the wrist RGB-D camera mount and stable optical-frame metadata.
        camera_path = robot + "/wrist_camera"
        camera = UsdGeom.Camera.Define(stage, camera_path)
        UsdGeom.XformCommonAPI(camera).SetTranslate(Gf.Vec3d(0.12, 0.0, 0.06))
        UsdGeom.XformCommonAPI(camera).SetRotate((0.0, -90.0, 180.0))
        camera.GetFocalLengthAttr().Set(24.0)
        camera.CreateAttribute("isaac:ros2:rgb_topic", Sdf.ValueTypeNames.String).Set("wrist_camera/image_raw")
        camera.CreateAttribute("isaac:ros2:depth_topic", Sdf.ValueTypeNames.String).Set("wrist_camera/depth/image_raw")
        camera.CreateAttribute("isaac:ros2:frame_id", Sdf.ValueTypeNames.String).Set("wrist_camera_optical_frame")
        # Persist finite limits required by NewtonMimic follower joints.
        for name in ("robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
                     "robotiq_85_left_finger_tip_joint", "robotiq_85_right_finger_tip_joint"):
            prim = stage.GetPrimAtPath(f"{robot}/Physics/{name}")
            if prim:
                for attr_name, value in (("physics:lowerLimit", -45.8366), ("physics:upperLimit", 45.8366)):
                    attr = prim.GetAttribute(attr_name) or prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Double)
                    attr.Set(value)
        stage.GetRootLayer().Export(output)
        print(f"Created complete USD: {output}")
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
