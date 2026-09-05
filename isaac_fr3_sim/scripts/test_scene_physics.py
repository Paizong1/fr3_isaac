"""Minimal gravity test for the existing ground, table, and banana colliders."""
import argparse
import os

from isaacsim import SimulationApp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", required=True)
    parser.add_argument("--frames", type=int, default=240)
    args = parser.parse_args()
    app = SimulationApp({"headless": True})
    try:
        import omni
        from pxr import UsdGeom
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import open_stage

        if not open_stage(os.path.abspath(args.asset)):
            raise RuntimeError(f"failed to open {args.asset}")
        stage = omni.usd.get_context().get_stage()
        banana = stage.GetPrimAtPath("/World/Banana")
        def height() -> float:
            return float(UsdGeom.XformCache().GetLocalToWorldTransform(banana).ExtractTranslation()[2])
        world = World(stage_units_in_meters=1.0)
        world.reset()
        before = height()
        for _ in range(args.frames):
            world.step(render=False)
        after = height()
        print(f"banana z: {before:.4f} -> {after:.4f}")
        if not 0.30 <= after <= 0.36:
            raise RuntimeError("banana did not settle on the worktable")
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
