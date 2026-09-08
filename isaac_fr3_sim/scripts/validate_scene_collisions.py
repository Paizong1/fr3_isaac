"""Run a read-only collision sanity check for the FR3 scene."""
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
        from pxr import Usd, UsdGeom
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import open_stage

        if not open_stage(os.path.abspath(args.asset)):
            raise RuntimeError(f"failed to open {args.asset}")
        stage = omni.usd.get_context().get_stage()
        banana = stage.GetPrimAtPath("/World/Banana")
        ground = stage.GetPrimAtPath("/World/Ground")
        if not banana or not ground:
            raise RuntimeError("scene must contain /World/Banana and /World/Ground")
        def z(prim) -> float:
            return float(UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation()[2])
        ground_top = float(UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
        ).ComputeWorldBound(ground).ComputeAlignedBox().GetMax()[2])
        world = World(stage_units_in_meters=1.0)
        world.reset()
        before = z(banana)
        for _ in range(args.frames):
            world.step(render=False)
        after = z(banana)
        for _ in range(30):
            world.step(render=False)
        settled = z(banana)
        banana_bounds = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
        ).ComputeWorldBound(banana).ComputeAlignedBox()
        print(f"ground top z: {ground_top:.4f}")
        print(f"banana z: {before:.4f} -> {after:.4f} -> {settled:.4f}")
        print(
            "banana bounds z: "
            f"{float(banana_bounds.GetMin()[2]):.4f} .. "
            f"{float(banana_bounds.GetMax()[2]):.4f}"
        )
        if settled < ground_top - 0.005:
            raise RuntimeError("banana passed through the ground")
        if abs(settled - after) > 0.01:
            raise RuntimeError("banana has not settled after the collision test")
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
