"""Attach the imported FR3 root joint directly to the physics world."""
import argparse
import os

from isaacsim import SimulationApp


ROOT_JOINT = "/fairino3_v6_robot/Physics/root_joint"
BASE = "/fairino3_v6_robot/Geometry/base_link"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", required=True)
    args = parser.parse_args()
    app = SimulationApp({"headless": True})
    try:
        from pxr import Gf, Sdf, Usd, UsdGeom

        stage = Usd.Stage.Open(os.path.abspath(args.asset))
        if stage is None:
            raise RuntimeError(f"cannot open {args.asset}")
        joint = stage.GetPrimAtPath(ROOT_JOINT)
        if not joint:
            raise RuntimeError(f"missing root joint: {ROOT_JOINT}")
        body0 = joint.GetRelationship("physics:body0")
        body1 = joint.GetRelationship("physics:body1")
        body0.SetTargets([])  # Empty body0 means the physics world.
        body1.SetTargets([Sdf.Path(BASE)])
        # The standing pose was baked into base_link after URDF import.  The
        # importer left both root anchors at zero, so PhysX sees incompatible
        # frames once the articulation starts.  Preserve the current base pose
        # as the world-side fixed-joint frame and use base_link's origin on the
        # body side.
        base_world = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(BASE))
        joint.GetAttribute("physics:localPos0").Set(Gf.Vec3f(base_world.ExtractTranslation()))
        joint.GetAttribute("physics:localRot0").Set(Gf.Quatf(base_world.ExtractRotationQuat()))
        joint.GetAttribute("physics:localPos1").Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.GetAttribute("physics:localRot1").Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        enabled = joint.GetAttribute("physics:jointEnabled")
        if enabled:
            enabled.Set(True)
        stage.GetRootLayer().Save()
        print(f"root joint attached to world: {ROOT_JOINT}")
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
