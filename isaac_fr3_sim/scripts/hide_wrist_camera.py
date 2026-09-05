"""Keep the wrist camera functional while hiding it from the viewport."""
import argparse
import os

from isaacsim import SimulationApp


CAMERA_PATH = (
    "/fairino3_v6_robot/Geometry/base_link/shoulder_link/upperarm_link/"
    "forearm_link/wrist1_link/wrist2_link/wrist3_link/wrist_camera"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", required=True)
    args = parser.parse_args()
    app = SimulationApp({"headless": True})
    try:
        from pxr import Usd, UsdGeom

        path = os.path.abspath(args.asset)
        stage = Usd.Stage.Open(path)
        if stage is None:
            raise RuntimeError(f"cannot open {path}")
        camera = stage.GetPrimAtPath(CAMERA_PATH)
        if not camera or not camera.IsA(UsdGeom.Camera):
            raise RuntimeError(f"wrist camera missing or invalid: {CAMERA_PATH}")
        UsdGeom.Imageable(camera).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        stage.GetRootLayer().Save()
        print(f"hid viewport display for camera: {CAMERA_PATH}")
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
