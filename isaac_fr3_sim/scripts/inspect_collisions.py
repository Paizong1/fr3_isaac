"""Report collision coverage of the existing FR3 scene without changing it."""
import argparse
import os

from isaacsim import SimulationApp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", required=True)
    args = parser.parse_args()
    app = SimulationApp({"headless": True})
    try:
        from collections import Counter
        from pxr import Usd, UsdGeom, UsdPhysics

        stage = Usd.Stage.Open(os.path.abspath(args.asset))
        if stage is None:
            raise RuntimeError(f"cannot open {args.asset}")
        collision_paths = [str(prim.GetPath()) for prim in stage.Traverse()
                           if prim.HasAPI(UsdPhysics.CollisionAPI)]
        proxy_collision_paths = [str(prim.GetPath()) for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
                                 if prim.HasAPI(UsdPhysics.CollisionAPI)]
        rigid_paths = [str(prim.GetPath()) for prim in stage.Traverse()
                       if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        print("collision count:", len(collision_paths))
        print("collision count with proxies:", len(proxy_collision_paths))
        print("direct collision paths:", collision_paths)
        print("rigid-body count:", len(rigid_paths))
        for path in ("/World/Ground", "/World/Worktable", "/World/Banana"):
            prim = stage.GetPrimAtPath(path)
            print(path, "valid=", bool(prim), "collision=", prim.HasAPI(UsdPhysics.CollisionAPI) if prim else False,
                  "rigid=", prim.HasAPI(UsdPhysics.RigidBodyAPI) if prim else False)
        banana = stage.GetPrimAtPath("/World/Banana")
        if banana:
            bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(banana)
            box = bbox.ComputeAlignedBox()
            print("banana world bbox:", tuple(box.GetMin()), tuple(box.GetMax()), tuple(box.GetSize()))
            print("banana xform ops:", [
                (str(op.GetOpType()), str(op.Get()))
                for op in UsdGeom.Xformable(banana).GetOrderedXformOps()
            ])
        print("robot collision count:", sum(path.startswith("/fairino3_v6_robot/") for path in proxy_collision_paths))
        print("robot geometry types:", dict(Counter(
            prim.GetTypeName() for prim in stage.Traverse()
            if str(prim.GetPath()).startswith("/fairino3_v6_robot/Geometry/") and prim.GetTypeName()
        )))
        print("robot geometry types with proxies:", dict(Counter(
            prim.GetTypeName() for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
            if str(prim.GetPath()).startswith("/fairino3_v6_robot/Geometry/") and prim.GetTypeName()
        )))
        print("robot instance roots:", [str(prim.GetPath()) for prim in stage.Traverse()
              if str(prim.GetPath()).startswith("/fairino3_v6_robot/Geometry/") and prim.IsInstance()])
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
