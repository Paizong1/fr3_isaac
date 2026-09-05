"""Calibrate USD joint limits and drive caps from the bundled FR3/Robotiq URDF."""
import argparse
import math
import os

from isaacsim import SimulationApp


ROBOT = "/fairino3_v6_robot/Physics/"
ARM = (
    ("j1", 150.0, 3.15), ("j2", 150.0, 3.15), ("j3", 150.0, 3.15),
    ("j4", 28.0, 3.20), ("j5", 28.0, 3.20), ("j6", 28.0, 3.20),
)
MIMICS = (
    "robotiq_85_right_knuckle_joint", "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_right_inner_knuckle_joint", "robotiq_85_left_finger_tip_joint",
    "robotiq_85_right_finger_tip_joint",
)


def set_float(prim, name, value, sdf):
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, sdf.ValueTypeNames.Float)
    attr.Set(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", required=True)
    args = parser.parse_args()
    app = SimulationApp({"headless": True})
    try:
        from pxr import Sdf, Usd

        path = os.path.abspath(args.asset)
        stage = Usd.Stage.Open(path)
        if stage is None:
            raise RuntimeError(f"cannot open {path}")
        for name, force, velocity in ARM:
            joint = stage.GetPrimAtPath(ROBOT + name)
            if not joint:
                raise RuntimeError(f"missing arm joint: {name}")
            set_float(joint, "drive:angular:physics:targetVelocity", 0.0, Sdf)
            set_float(joint, "drive:angular:physics:stiffness", 500.0, Sdf)
            set_float(joint, "drive:angular:physics:damping", 50.0, Sdf)
            set_float(joint, "drive:angular:physics:maxForce", force, Sdf)
            set_float(joint, "physxJoint:maxJointVelocity", math.degrees(velocity), Sdf)

        master = stage.GetPrimAtPath(ROBOT + "robotiq_85_left_knuckle_joint")
        if not master:
            raise RuntimeError("missing Robotiq master joint")
        set_float(master, "physics:lowerLimit", 0.0, Sdf)
        set_float(master, "physics:upperLimit", math.degrees(0.8), Sdf)
        set_float(master, "drive:angular:physics:targetVelocity", 0.0, Sdf)
        set_float(master, "drive:angular:physics:stiffness", 1000.0, Sdf)
        set_float(master, "drive:angular:physics:damping", 100.0, Sdf)
        set_float(master, "drive:angular:physics:maxForce", 50.0, Sdf)
        set_float(master, "physxJoint:maxJointVelocity", math.degrees(0.5), Sdf)
        for name in MIMICS:
            joint = stage.GetPrimAtPath(ROBOT + name)
            if not joint:
                raise RuntimeError(f"missing Robotiq mimic joint: {name}")
            set_float(joint, "drive:angular:physics:maxForce", 50.0, Sdf)
            set_float(joint, "physxJoint:maxJointVelocity", math.degrees(0.5), Sdf)
        stage.GetRootLayer().Save()
        print(f"calibrated joint caps in {path}")
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
