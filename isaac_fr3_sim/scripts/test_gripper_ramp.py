"""Validate a bounded, gradual Robotiq close command without saving changes."""
import argparse
import math
import os

from isaacsim import SimulationApp


MASTER = "/fairino3_v6_robot/Physics/robotiq_85_left_knuckle_joint"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", required=True)
    parser.add_argument("--target-radians", type=float, default=0.6)
    parser.add_argument("--ramp-frames", type=int, default=240)
    parser.add_argument("--settle-frames", type=int, default=120)
    args = parser.parse_args()
    app = SimulationApp({"headless": True})
    try:
        import omni
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.stage import open_stage

        if not open_stage(os.path.abspath(args.asset)):
            raise RuntimeError(f"cannot open {args.asset}")
        stage = omni.usd.get_context().get_stage()
        target = stage.GetPrimAtPath(MASTER).GetAttribute("drive:angular:physics:targetPosition")
        world = World(stage_units_in_meters=1.0)
        robot = Articulation("/fairino3_v6_robot")
        world.reset()
        robot.initialize()
        for frame in range(args.ramp_frames + args.settle_frames):
            progress = min(1.0, (frame + 1) / args.ramp_frames)
            target.Set(math.degrees(args.target_radians * progress))
            world.step(render=False)
        positions = robot.get_joint_positions()[0]
        print("final arm pose:", positions[:6].round(4))
        print("final gripper pose:", positions[6:].round(4))
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
