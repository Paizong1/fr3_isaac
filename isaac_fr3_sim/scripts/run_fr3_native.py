#!/usr/bin/env python3
"""Windows-only FR3 simulation; no ROS 2, Ubuntu, or MoveIt required."""
import argparse
import os


def repair_gripper_limits(stage) -> None:
    """Give NewtonMimic follower joints finite revolute limits at runtime."""
    from pxr import Sdf
    for name in (
        "robotiq_85_left_inner_knuckle_joint",
        "robotiq_85_right_inner_knuckle_joint",
        "robotiq_85_left_finger_tip_joint",
        "robotiq_85_right_finger_tip_joint",
    ):
        prim = stage.GetPrimAtPath(f"/fairino3_v6_robot/Physics/{name}")
        if not prim or not prim.IsValid():
            continue
        for attr_name, value in (("physics:lowerLimit", -45.8366), ("physics:upperLimit", 45.8366)):
            attr = prim.GetAttribute(attr_name)
            if not attr:
                attr = prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Double)
            attr.Set(value)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--asset", default=os.path.join(os.path.dirname(__file__), "..", "assets", "fairino3_robotiq_complete.usd"))
    p.add_argument("--headless", action="store_true")
    p.add_argument("--test-frames", type=int, default=0)
    p.add_argument("--cycle-frames", type=int, default=240)
    args = p.parse_args()
    asset = os.path.abspath(args.asset)
    if not os.path.isfile(asset):
        p.error(f"USD asset does not exist: {asset}")

    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": args.headless})
    try:
        import numpy as np
        import omni
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        from isaacsim.core.utils.stage import open_stage
        from isaacsim.core.utils.types import ArticulationAction

        if not open_stage(asset):
            raise RuntimeError(f"failed to open USD stage: {asset}")
        stage = omni.usd.get_context().get_stage()
        repair_gripper_limits(stage)
        robot_path = "/fairino3_v6_robot"
        robot_prim = stage.GetPrimAtPath(robot_path)
        if not robot_prim:
            raise RuntimeError(f"robot prim not found: {robot_path}")

        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()
        world.scene.add(FixedCuboid(
            prim_path="/World/Worktable", name="worktable",
            position=np.array([0.45, 0.0, 0.15]), scale=np.array([0.8, 0.8, 0.3]),
            color=np.array([0.25, 0.25, 0.28]),
        ))
        world.scene.add(DynamicCuboid(
            prim_path="/World/TargetCube", name="target_cube",
            position=np.array([0.45, 0.0, 0.42]), size=0.08,
            color=np.array([0.1, 0.65, 0.95]),
        ))
        robot = Articulation(robot_path)
        world.reset()
        robot.initialize()
        names = list(robot.dof_names)
        print("FR3 DOFs:", names)
        targets = {"j1": 1.2, "j2": -1.2, "j3": 1.0, "j4": -1.8, "j5": -1.57, "j6": 0.0, "robotiq_85_left_knuckle_joint": 0.02}
        indices = np.array([names.index(n) for n in targets if n in names], dtype=np.int32)
        positions = np.array([targets[n] for n in targets if n in names], dtype=np.float64)
        home_positions = positions.copy()
        task_positions = positions.copy()
        if "robotiq_85_left_knuckle_joint" in names:
            grip_i = int(np.where(indices == names.index("robotiq_85_left_knuckle_joint"))[0][0])
            task_positions[grip_i] = 0.72
        if len(task_positions) >= 6:
            task_positions[:6] = np.array([0.6, -0.9, 1.25, -1.7, -1.2, 0.0])

        frames = 0
        while simulation_app.is_running():
            phase = (frames % max(2, args.cycle_frames)) / max(2, args.cycle_frames)
            alpha = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
            command = home_positions + alpha * (task_positions - home_positions)
            robot.apply_action(ArticulationAction(joint_positions=command, joint_indices=indices))
            world.step(render=not args.headless)
            frames += 1
            if args.test_frames and frames >= args.test_frames:
                break
    finally:
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
