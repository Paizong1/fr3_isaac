"""Store TCP and wrist-camera calibration on existing USD prims."""
import argparse
import os

from isaacsim import SimulationApp


ROBOT = "/fairino3_v6_robot/Geometry/base_link"
WRIST = ROBOT + "/shoulder_link/upperarm_link/forearm_link/wrist1_link/wrist2_link/wrist3_link"
GRIPPER_BASE = WRIST + "/robotiq_85_base_link"
CAMERA = WRIST + "/wrist_camera"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    app = SimulationApp({"headless": True})
    try:
        from pxr import Gf, Sdf, Usd, UsdGeom

        stage = Usd.Stage.Open(os.path.abspath(args.asset))
        if stage is None:
            raise RuntimeError(f"cannot open {args.asset}")
        for path in (WRIST, GRIPPER_BASE, CAMERA):
            prim = stage.GetPrimAtPath(path)
            if not prim:
                raise RuntimeError(f"missing calibration parent: {path}")
            print(path, UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation())
        if args.dry_run:
            for path, names in (
                (WRIST, ("isaac:ros2:frame_id",)),
                (GRIPPER_BASE, ("isaac:calibration:tcp_frame_id", "isaac:calibration:tcp_translation_m")),
                (CAMERA, ("isaac:ros2:frame_id", "isaac:calibration:optical_rpy_deg")),
            ):
                prim = stage.GetPrimAtPath(path)
                for name in names:
                    print(name, prim.GetAttribute(name).Get())
            return 0
        def set_attr(prim, name, type_name, value):
            prim.CreateAttribute(name, type_name).Set(value)

        wrist = stage.GetPrimAtPath(WRIST)
        gripper = stage.GetPrimAtPath(GRIPPER_BASE)
        camera = stage.GetPrimAtPath(CAMERA)
        # Geometry hierarchy is instanceable; store calibration on its existing
        # Prim rather than creating non-authorable children below it.
        set_attr(wrist, "isaac:ros2:frame_id", Sdf.ValueTypeNames.String, "tool0")
        set_attr(wrist, "isaac:calibration:mount_note", Sdf.ValueTypeNames.String,
                 "USD mount verified: robotiq_85_base_link coincides with wrist3_link")
        set_attr(gripper, "isaac:ros2:frame_id", Sdf.ValueTypeNames.String, "robotiq_85_base_link")
        set_attr(gripper, "isaac:calibration:tcp_frame_id", Sdf.ValueTypeNames.String, "gripper_tcp")
        set_attr(gripper, "isaac:calibration:tcp_translation_m", Sdf.ValueTypeNames.Vector3d, Gf.Vec3d(0.000532, 0.000127, 0.000953))
        set_attr(gripper, "isaac:calibration:tcp_rpy_deg", Sdf.ValueTypeNames.Vector3f, Gf.Vec3f(0.0014, -0.0097, -0.0450))
        set_attr(camera, "isaac:ros2:frame_id", Sdf.ValueTypeNames.String, "wrist_camera_optical_frame")
        set_attr(camera, "isaac:calibration:link_frame_id", Sdf.ValueTypeNames.String, "wrist_camera_link")
        set_attr(camera, "isaac:calibration:optical_frame_id", Sdf.ValueTypeNames.String, "wrist_camera_optical_frame")
        set_attr(camera, "isaac:calibration:optical_rpy_deg", Sdf.ValueTypeNames.Vector3f, Gf.Vec3f(-90.0, 0.0, -90.0))
        stage.GetRootLayer().Save()
        print("stored tool0, gripper_tcp, wrist_camera_link, and wrist_camera_optical_frame calibration")
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
