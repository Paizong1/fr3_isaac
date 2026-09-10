#!/usr/bin/env python3
"""Minimal Isaac Sim runner for the converted FR3 USD asset."""
import argparse
import math
import os
import sys
import time
import traceback


ARM_JOINTS = ("j1", "j2", "j3", "j4", "j5", "j6")
MAX_SAFE_ARM_ABS_RAD = 2.0 * math.pi
ACTUAL_LIMIT_GRACE_RAD = 0.1
J2_COMPENSATION_DEADBAND_RAD = 0.01
JOINT_TRAJECTORY_TOPIC = "/fairino3_controller/joint_trajectory"
GRIPPER_MASTER_JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_COMMAND_TOPIC = "/robotiq_gripper_controller/position_command"
GRIPPER_STATE_TOPIC = "/robotiq_gripper_controller/position_state"
GRIPPER_MASTER_DRIVE = (20.0, 300.0, 50.0)
GRIPPER_FRICTION_CLOSE_DRIVE = (8.0, 120.0, 80.0)
GRIPPER_HOLD_DRIVE = (8.0, 100.0, 100.0)
GRIPPER_MAX_VELOCITY_RAD_S = 0.18
GRIPPER_COMMAND_VELOCITY_RAD_S = 0.12
GRIPPER_LIMIT_DEG = math.degrees(0.8)
BILATERAL_CONTACT_DISTANCE_M = 0.065
FRICTION_STATIC_FRICTION = 1.2
FRICTION_DYNAMIC_FRICTION = 1.0
FRICTION_HOLD_FORCE_N = 5.0
FRICTION_GRASP_SOLVER_POSITION_ITERATIONS = 16
FRICTION_GRASP_SOLVER_VELOCITY_ITERATIONS = 16
TRACKING_COMPENSATION_GAIN = 0.5
TRACKING_HOLD_COMPENSATION_GAIN = 0.5
TRACKING_COMPENSATION_LIMIT_RAD = 0.02
TRACKING_COMPENSATION_RESPONSE_SEC = 0.45
TRACKING_COMPENSATION_INDICES = (ARM_JOINTS.index("j2"), ARM_JOINTS.index("j3"))
LIFT_TRACKING_REPORT_SEC = 0.5
STARTUP_TRACKING_REPORT_SEC = 0.5

# PhysX revolute-drive gains.  The converted USD has angular DriveAPI schemas
# but no stiffness/damping, so position targets otherwise produce no torque.
ARM_DRIVES = {
    # The shoulder links sagged by about 0.02 rad under gravity with the
    # original gains, exceeding FollowJointTrajectory's final tolerance.
    "j1": (250.0, 5_000.0, 400.0),
    # Extra damping suppresses the measured lift-start vertical rebound.
    "j2": (1_000.0, 34_000.0, 1_760.0),
    "j3": (250.0, 9_000.0, 550.0),
    "j4": (120.0, 7_000.0, 400.0),
    "j5": (300.0, 12_000.0, 750.0),
    "j6": (120.0, 4_000.0, 300.0),
}


class TrajectoryExecutor:
    """Validate and interpolate one position-only JointTrajectory at a time."""

    def __init__(self, joint_limits):
        self.joint_limits = joint_limits
        self.start_time = None
        self.start_positions = None
        self.points = ()
        self.j2_compensation = 0.0
        self.new_trajectory = False

    def submit(self, message, now, current_positions, append=False):
        names = tuple(message.joint_names)
        if len(names) != len(ARM_JOINTS) or set(names) != set(ARM_JOINTS):
            return "joint_names must contain exactly j1 through j6 (no gripper joints)"
        if len(set(names)) != len(names):
            return "joint_names must not contain duplicates"
        if not message.points:
            return "trajectory has no points"

        queue_active = append and self.points and now < self.start_time + self.points[-1][0]
        self.new_trajectory = not queue_active
        if not queue_active:
            if self.points:
                previous_j2 = self.points[-1][1][ARM_JOINTS.index("j2")]
                actual_j2 = float(current_positions[ARM_JOINTS.index("j2")])
                correction = previous_j2 - actual_j2
                self.j2_compensation = (
                    max(-0.03, min(0.03, correction))
                    if abs(correction) >= J2_COMPENSATION_DEADBAND_RAD else 0.0
                )
            else:
                self.j2_compensation = 0.0

        order = [names.index(name) for name in ARM_JOINTS]
        points = []
        previous_time = -1.0
        for point in message.points:
            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if not math.isfinite(point_time) or point_time < 0.1 or point_time <= previous_time:
                return "point time_from_start values must be finite, at least 0.1 s, and strictly increasing"
            if len(point.positions) != len(ARM_JOINTS):
                return "each point must contain six position targets"
            positions = [float(point.positions[index]) for index in order]
            positions[ARM_JOINTS.index("j2")] += self.j2_compensation
            positions = tuple(positions)
            if not all(math.isfinite(position) for position in positions):
                return "position targets must be finite"
            for name, position in zip(ARM_JOINTS, positions):
                lower, upper = self.joint_limits[name]
                if position < lower or position > upper:
                    return f"{name} target {position:.6f} is outside [{lower:.6f}, {upper:.6f}]"
                if abs(position) > MAX_SAFE_ARM_ABS_RAD:
                    return f"{name} target {position:.6f} exceeds the {MAX_SAFE_ARM_ABS_RAD:.3f} rad safety limit"
            points.append((point_time, positions))
            previous_time = point_time

        if queue_active:
            offset = self.points[-1][0]
            self.points += tuple((offset + point_time, positions) for point_time, positions in points)
        else:
            self.start_time = now
            self.start_positions = tuple(float(position) for position in current_positions)
            self.points = tuple(points)
        return None

    def target_at(self, now):
        if not self.points:
            return None
        elapsed = max(0.0, now - self.start_time)
        knots = ((0.0, self.start_positions),) + self.points
        for index, ((left_time, left_positions), (right_time, right_positions)) in enumerate(
            zip(knots, knots[1:])
        ):
            if elapsed <= right_time:
                previous = knots[index - 1] if index else None
                following = knots[index + 2] if index + 2 < len(knots) else None
                duration = right_time - left_time
                return self._interpolate(
                    left_positions,
                    right_positions,
                    self._knot_velocity(previous, (left_time, left_positions), (right_time, right_positions)),
                    self._knot_velocity((left_time, left_positions), (right_time, right_positions), following),
                    duration,
                    (elapsed - left_time) / duration,
                )
        return self.points[-1][1]

    @staticmethod
    def _knot_velocity(previous, current, following):
        if previous is None or following is None:
            return (0.0,) * len(current[1])
        previous_time, previous_positions = previous
        current_time, current_positions = current
        following_time, following_positions = following
        left_duration = current_time - previous_time
        right_duration = following_time - current_time
        velocities = []
        for before, position, after in zip(previous_positions, current_positions, following_positions):
            left_slope = (position - before) / left_duration
            right_slope = (after - position) / right_duration
            if left_slope * right_slope <= 0.0:
                velocities.append(0.0)
                continue
            left_weight = 2.0 * right_duration + left_duration
            right_weight = right_duration + 2.0 * left_duration
            velocities.append(
                (left_weight + right_weight) /
                (left_weight / left_slope + right_weight / right_slope)
            )
        return tuple(velocities)

    @staticmethod
    def _interpolate(left, right, left_velocity, right_velocity, duration, ratio):
        ratio = max(0.0, min(1.0, ratio))
        ratio2 = ratio * ratio
        ratio3 = ratio2 * ratio
        h00 = 2.0 * ratio3 - 3.0 * ratio2 + 1.0
        h10 = ratio3 - 2.0 * ratio2 + ratio
        h01 = -2.0 * ratio3 + 3.0 * ratio2
        h11 = ratio3 - ratio2
        return tuple(
            h00 * a + h10 * duration * va + h01 * b + h11 * duration * vb
            for a, b, va, vb in zip(left, right, left_velocity, right_velocity)
        )


class GripperRamp:
    """Rate-limit the Robotiq master joint so commands never jump in one frame."""

    def __init__(self, lower=0.0, upper=0.8, max_velocity=0.1):
        self.lower = lower
        self.upper = upper
        self.max_velocity = max_velocity
        self.target = None
        self.commanded = None

    def set_target(self, target):
        if not math.isfinite(target) or not self.lower <= target <= self.upper:
            return f"target must be within [{self.lower:.3f}, {self.upper:.3f}] rad"
        self.target = target
        return None

    def step(self, current, dt):
        if self.target is None:
            return None
        if self.commanded is None:
            self.commanded = current
        delta = self.target - self.commanded
        self.commanded += max(-self.max_velocity * dt, min(self.max_velocity * dt, delta))
        return self.commanded


def closest_joint_angle(position, target, lower=None, upper=None):
    """Use an equivalent revolute angle inside the joint limits when possible."""
    candidates = (
        position + 2.0 * math.pi * turns
        for turns in range(-64, 65)
        if lower is None or lower - ACTUAL_LIMIT_GRACE_RAD <= position + 2.0 * math.pi * turns <= upper + ACTUAL_LIMIT_GRACE_RAD
    )
    candidates = tuple(candidates)
    if candidates:
        return min(candidates, key=lambda value: abs(value - target))
    return position + 2.0 * math.pi * round((target - position) / (2.0 * math.pi))


def update_tracking_bias(target, actual, bias, dt, gain=TRACKING_COMPENSATION_GAIN):
    alpha = min(1.0, max(0.0, dt) / TRACKING_COMPENSATION_RESPONSE_SEC)
    updated = list(bias)
    for index in TRACKING_COMPENSATION_INDICES:
        desired = max(
            -TRACKING_COMPENSATION_LIMIT_RAD,
            min(TRACKING_COMPENSATION_LIMIT_RAD, gain * (target[index] - actual[index])),
        )
        updated[index] += alpha * (desired - updated[index])
    return tuple(updated)


def position_relative_to_midpoint(position, left, right):
    return tuple(value - (left[index] + right[index]) / 2.0 for index, value in enumerate(position))


def should_update_friction_bias(smoke_target, friction_bias_locked):
    return smoke_target is None and not friction_bias_locked


def run_trajectory_self_test() -> None:
    class Duration:
        def __init__(self, seconds):
            self.sec, self.nanosec = divmod(round(seconds * 1_000_000_000), 1_000_000_000)

    class Point:
        def __init__(self, positions, seconds):
            self.positions, self.time_from_start = positions, Duration(seconds)

    class Message:
        joint_names = list(ARM_JOINTS)
        points = [Point([1, 2, 3, 4, 5, 6], 1.0), Point([2, 3, 4, 5, 6, 6], 2.0)]

    executor = TrajectoryExecutor({name: (-10.0, 10.0) for name in ARM_JOINTS})
    assert executor.submit(Message(), 10.0, [0.0] * 6) is None
    assert all(math.isclose(actual, expected) for actual, expected in zip(
        executor.target_at(10.5), (0.375, 5.0 / 6.0, 1.3125, 1.8, 2.2916666666666665, 3.0)
    ))
    assert executor.target_at(11.0) == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert executor.target_at(12.0) == (2.0, 3.0, 4.0, 5.0, 6.0, 6.0)
    assert TrajectoryExecutor._knot_velocity((0.0, (0.0,)), (1.0, (1.0,)), (2.0, (2.0,))) == (1.0,)
    assert TrajectoryExecutor._knot_velocity((0.0, (0.0,)), (1.0, (1.0,)), (2.0, (0.0,))) == (0.0,)
    class SinglePointMessage:
        joint_names = list(ARM_JOINTS)

        def __init__(self, positions):
            self.points = [Point(positions, 1.0)]

    queued = TrajectoryExecutor({name: (-10.0, 10.0) for name in ARM_JOINTS})
    assert queued.submit(SinglePointMessage([1] * 6), 20.0, [0.0] * 6) is None
    assert queued.submit(SinglePointMessage([2] * 6), 20.0, [0.0] * 6, append=True) is None
    assert queued.target_at(21.5) == (1.625,) * 6
    assert queued.submit(SinglePointMessage([2] * 6), 23.0, [2.0, 2.012, 2.0, 2.0, 2.0, 2.0], append=True) is None
    assert math.isclose(queued.points[-1][1][1], 1.988, abs_tol=1e-12)
    assert queued.submit(SinglePointMessage([2] * 6), 25.0, [2.0, 1.996, 2.0, 2.0, 2.0, 2.0], append=True) is None
    assert queued.points[-1][1][1] == 2.0
    Message.joint_names = [*ARM_JOINTS[:-1], "robotiq_85_left_knuckle_joint"]
    assert executor.submit(Message(), 12.0, [0.0] * 6) is not None
    gripper = GripperRamp(max_velocity=GRIPPER_COMMAND_VELOCITY_RAD_S)
    assert gripper.set_target(0.6) is None
    assert math.isclose(gripper.step(0.0, 0.1), 0.012, abs_tol=1e-12)
    assert gripper.set_target(0.9) is not None
    assert math.isclose(closest_joint_angle(-4.109, -1.571, -3.1, 3.1), 2.174185307179586, abs_tol=1e-6)
    assert math.isclose(closest_joint_angle(27.753784, -1.552, -3.1, 3.1), 2.6210428, abs_tol=1e-5)
    bias = update_tracking_bias((0.0, 1.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.98, 0.0, 0.0, 0.0, 0.0), (0.0,) * 6, 0.45)
    assert math.isclose(bias[1], 0.01, abs_tol=1e-12)
    assert position_relative_to_midpoint((2.0, 3.0, 4.0), (0.0, 0.0, 0.0), (4.0, 2.0, 2.0)) == (0.0, 2.0, 3.0)
    assert should_update_friction_bias(None, False)
    assert not should_update_friction_bias((0.0,) * 6, False)
    assert not should_update_friction_bias(None, True)
    print("[bridge] trajectory self-test passed", flush=True)


def repair_gripper_limits(stage) -> None:
    from pxr import Sdf
    master = stage.GetPrimAtPath(f"/fairino3_v6_robot/Physics/{GRIPPER_MASTER_JOINT}")
    if not master or not master.IsValid():
        raise RuntimeError("Robotiq master joint not found")
    for attr_name, value in (("physics:lowerLimit", 0.0), ("physics:upperLimit", GRIPPER_LIMIT_DEG)):
        attr = master.GetAttribute(attr_name)
        if not attr:
            attr = master.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float)
        attr.Set(value)
    velocity = master.GetAttribute("physxJoint:maxJointVelocity")
    if not velocity:
        velocity = master.CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float)
    velocity.Set(math.degrees(GRIPPER_MAX_VELOCITY_RAD_S))
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
        velocity = prim.GetAttribute("physxJoint:maxJointVelocity")
        if not velocity:
            velocity = prim.CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float)
        velocity.Set(math.degrees(GRIPPER_MAX_VELOCITY_RAD_S))


def configure_position_drives(stage) -> None:
    """Add the missing PhysX PD gains without saving changes to the user's USD."""
    from pxr import UsdPhysics

    drives = ARM_DRIVES
    for name, (max_force, stiffness, damping) in drives.items():
        joint = stage.GetPrimAtPath(f"/fairino3_v6_robot/Physics/{name}")
        if not joint or not joint.IsValid():
            raise RuntimeError(f"position-drive joint not found: {name}")
        drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateMaxForceAttr(max_force).Set(max_force)
        drive.CreateStiffnessAttr(stiffness).Set(stiffness)
        drive.CreateDampingAttr(damping).Set(damping)
        print(
            f"[bridge] position drive {name}: force={max_force:.1f}, "
            f"stiffness={stiffness:.1f}, damping={damping:.1f}",
            flush=True,
        )
    gripper_joint = stage.GetPrimAtPath(f"/fairino3_v6_robot/Physics/{GRIPPER_MASTER_JOINT}")
    drive = UsdPhysics.DriveAPI.Apply(gripper_joint, "angular")
    max_force, stiffness, damping = GRIPPER_MASTER_DRIVE
    drive.CreateTypeAttr("force")
    drive.CreateMaxForceAttr(max_force).Set(max_force)
    drive.CreateStiffnessAttr(stiffness).Set(stiffness)
    drive.CreateDampingAttr(damping).Set(damping)
    print(
        f"[bridge] gripper drive {GRIPPER_MASTER_JOINT}: force={max_force:.1f}, "
        f"stiffness={stiffness:.1f}, damping={damping:.1f}, limit=0.800 rad",
        flush=True,
    )


BANANA_CALIBRATION_POSITION = (-0.13779715872850662, -0.5833656024637526, 0.025)


def configure_scene_collisions(
    stage, freeze_banana: bool, enable_collisions: bool = True, banana_contact_proxy: bool = False,
    stable_grasp_demo: bool = False, bilateral_grasp_demo: bool = False
) -> None:
    """Configure table collisions and the banana's physical or virtual mode."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    if enable_collisions:
        counts = {}
        for root_path in ("/World/Ground", "/World/Worktable", "/World/Banana"):
            root = stage.GetPrimAtPath(root_path)
            if not root or not root.IsValid():
                counts[root_path] = "missing"
                continue
            if root_path == "/World/Banana" and banana_contact_proxy:
                disabled = 0
                rigid_bodies_disabled = 0
                for prim in Usd.PrimRange(root):
                    if prim.IsA(UsdGeom.Gprim):
                        collider = UsdPhysics.CollisionAPI.Apply(prim)
                        collider.CreateCollisionEnabledAttr(False).Set(False)
                        disabled += 1
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False).Set(False)
                        rigid_bodies_disabled += 1
                counts[root_path] = f"visual-only mesh ({disabled} colliders, {rigid_bodies_disabled} bodies disabled)"
                continue
            count = 0
            try:
                for prim in Usd.PrimRange(root):
                    if not prim.IsA(UsdGeom.Gprim):
                        continue
                    collider = UsdPhysics.CollisionAPI.Apply(prim)
                    collider.CreateCollisionEnabledAttr(True).Set(True)
                    if root_path == "/World/Banana" and prim.IsA(UsdGeom.Mesh):
                        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull").Set("convexHull")
                    count += 1
                counts[root_path] = count
            except Exception as exc:
                counts[root_path] = f"error: {exc}"
        if banana_contact_proxy:
            banana = stage.GetPrimAtPath("/World/Banana")
            if not banana or not banana.IsValid():
                raise RuntimeError("Banana prim not found for contact proxy")
            proxy = UsdGeom.Capsule.Define(stage, "/World/BananaContactProxy")
            proxy.CreateAxisAttr(UsdGeom.Tokens.x)
            proxy.CreateRadiusAttr(0.025)
            proxy.CreateHeightAttr(0.12)
            proxy.AddTranslateOp().Set(Gf.Vec3d(*BANANA_CALIBRATION_POSITION))
            UsdGeom.Imageable(proxy.GetPrim()).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            UsdPhysics.CollisionAPI.Apply(proxy.GetPrim()).CreateCollisionEnabledAttr(
                True
            ).Set(not (freeze_banana or stable_grasp_demo or bilateral_grasp_demo))
            if stable_grasp_demo or bilateral_grasp_demo:
                UsdPhysics.RigidBodyAPI.Apply(proxy.GetPrim()).CreateKinematicEnabledAttr(True).Set(True)
            elif not freeze_banana:
                UsdPhysics.RigidBodyAPI.Apply(proxy.GetPrim()).CreateKinematicEnabledAttr(True).Set(False)
                UsdPhysics.MassAPI.Apply(proxy.GetPrim()).CreateMassAttr(0.12).Set(0.12)
            counts["/World/BananaContactProxy"] = (
                "virtual-attach capsule" if (stable_grasp_demo or bilateral_grasp_demo) else
                ("visual-only frozen capsule" if freeze_banana else
                 "capsule 0.120m x 0.050m")
            )
        print(f"[bridge] scene colliders enabled: {counts}", flush=True)
    else:
        disabled = {}
        for root_path in ("/World/Ground", "/World/Worktable", "/World/Banana"):
            root = stage.GetPrimAtPath(root_path)
            if not root or not root.IsValid():
                disabled[root_path] = "missing"
                continue
            count = 0
            for prim in Usd.PrimRange(root):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False).Set(False)
                    count += 1
            disabled[root_path] = count
        print(f"[bridge] scene colliders disabled for this diagnostic run: {disabled}", flush=True)
    if freeze_banana or stable_grasp_demo or bilateral_grasp_demo:
        banana = stage.GetPrimAtPath("/World/Banana")
        banana_xform = UsdGeom.Xformable(banana)
        translate_ops = [
            op for op in banana_xform.GetOrderedXformOps()
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
        ]
        translate_op = translate_ops[-1] if translate_ops else banana_xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*BANANA_CALIBRATION_POSITION))
        body = UsdPhysics.RigidBodyAPI.Apply(banana)
        body.CreateKinematicEnabledAttr(True).Set(True)
        world_position = banana_xform.ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        ).ExtractTranslation()
        print(
            f"[bridge] banana reset and configured "
            f"{'virtual grasp' if (stable_grasp_demo or bilateral_grasp_demo) else 'TCP calibration'}: "
            f"world={tuple(world_position)}",
            flush=True,
        )


def configure_bilateral_grasp_pads(stage):
    """Create small kinematic contact pads at the two finger tips.

    The imported finger meshes have no PhysX colliders.  These pads give the
    dynamic banana a real, symmetric contact signal without turning the whole
    gripper into a high-cost mesh collider.
    """
    from pxr import UsdGeom, UsdPhysics

    pads = {}
    for side in ("left", "right"):
        sphere = UsdGeom.Sphere.Define(stage, f"/World/GraspContactPads/{side}")
        sphere.CreateRadiusAttr(0.008)
        translate = sphere.AddTranslateOp()
        prim = sphere.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True).Set(False)
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(True).Set(True)
        pads[side] = (prim, translate)
    print("[bridge] bilateral grasp pads enabled at Robotiq finger tips", flush=True)
    return pads


def configure_friction_grasp_pads(stage, finger_tips, banana_proxy):
    """Add collider-only pads below the articulated finger-tip links."""
    from isaacsim.sensors.experimental.physics import Contact
    from pxr import UsdGeom, UsdPhysics, UsdShade

    material = UsdShade.Material.Define(stage, "/World/FrictionGraspMaterial")
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api.CreateStaticFrictionAttr(FRICTION_STATIC_FRICTION)
    material_api.CreateDynamicFrictionAttr(FRICTION_DYNAMIC_FRICTION)
    UsdShade.MaterialBindingAPI.Apply(banana_proxy).Bind(material)
    sensor_paths = {}
    for side, tip in finger_tips.items():
        pad = UsdGeom.Sphere.Define(stage, f"{tip.GetPath()}/friction_pad")
        pad.CreateRadiusAttr(0.008)
        prim = pad.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True).Set(True)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
        sensor_path = f"{tip.GetPath()}/friction_pad/pressure_sensor"
        Contact.create(
            sensor_path,
            translations=[[0.0, 0.0, 0.0]],
            min_threshold=0.0,
            max_threshold=100.0,
            radius=-1.0,
        )
        sensor_paths[side] = sensor_path
    print("[bridge] friction grasp pads attached to Robotiq finger-tip links", flush=True)
    return sensor_paths


def configure_friction_grasp_solver(stage):
    """Increase only the two bodies participating in the friction contact."""
    from pxr import PhysxSchema

    for path, api_type in (
        ("/fairino3_v6_robot", PhysxSchema.PhysxArticulationAPI),
        ("/World/BananaContactProxy", PhysxSchema.PhysxRigidBodyAPI),
    ):
        api = api_type.Apply(stage.GetPrimAtPath(path))
        api.CreateSolverPositionIterationCountAttr().Set(FRICTION_GRASP_SOLVER_POSITION_ITERATIONS)
        api.CreateSolverVelocityIterationCountAttr().Set(FRICTION_GRASP_SOLVER_VELOCITY_ITERATIONS)
    print(
        "[bridge] friction grasp solver: "
        f"position_iterations={FRICTION_GRASP_SOLVER_POSITION_ITERATIONS}, "
        f"velocity_iterations={FRICTION_GRASP_SOLVER_VELOCITY_ITERATIONS}",
        flush=True,
    )


def reduce_detection_shadows(stage) -> None:
    """Disable existing light shadows for the camera session; do not save the USD."""
    disabled = 0
    for prim in stage.Traverse():
        shadow_enable = prim.GetAttribute("inputs:shadow:enable")
        if shadow_enable and shadow_enable.Get() is not False:
            shadow_enable.Set(False)
            disabled += 1
    print(f"[bridge] disabled shadows on {disabled} light(s) for YOLO", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asset", help="Path to fairino3_robotiq.usd")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--rgb-from-active-viewport",
        action="store_true",
        help="Publish RGB from the visible Isaac viewport render product (requires windowed mode).",
    )
    parser.add_argument("--domain-id", type=int, default=None)
    parser.add_argument("--test-frames", type=int, default=0)
    parser.add_argument("--trajectory-topic", default=JOINT_TRAJECTORY_TOPIC)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-tick-rate", type=float, default=1.0)
    parser.add_argument("--sim-rate", type=float, default=30.0)
    parser.add_argument("--self-test-trajectories", action="store_true")
    parser.add_argument("--drive-smoke-test", action="store_true",
                        help="hold the arm, move j6 by 0.1 rad, and verify PhysX tracking")
    parser.add_argument("--startup-tracking", action="store_true",
                        help="Print arm angles and startup-hold errors before any arm trajectory arrives.")
    parser.add_argument("--keep-light-shadows", action="store_true")
    parser.add_argument("--freeze-banana", action="store_true",
                        help="Keep the banana kinematic during TCP-only calibration.")
    parser.add_argument("--stable-grasp-demo", action="store_true",
                        help="Kinematically attach the banana to the gripper after a completed close command.")
    parser.add_argument("--bilateral-grasp-demo", action="store_true",
                        help="Use virtual bilateral contact and attach after both finger tips are near.")
    parser.add_argument("--friction-grasp", action="store_true",
                        help="Grip the dynamic banana capsule through physical finger-pad friction.")
    parser.add_argument("--disable-scene-collisions", action="store_true",
                        help="Diagnostic only: do not enable Ground, Worktable, or Banana PhysX colliders.")
    parser.add_argument("--banana-contact-proxy", action="store_true",
                        help="Diagnostic only: replace Banana's scaled mesh collider with a fixed capsule proxy.")
    args = parser.parse_args()
    if args.self_test_trajectories:
        run_trajectory_self_test()
        return 0
    if args.stable_grasp_demo and not args.banana_contact_proxy:
        parser.error("--stable-grasp-demo requires --banana-contact-proxy")
    if args.stable_grasp_demo and args.freeze_banana:
        parser.error("--stable-grasp-demo cannot be combined with --freeze-banana")
    if args.bilateral_grasp_demo and not args.banana_contact_proxy:
        parser.error("--bilateral-grasp-demo requires --banana-contact-proxy")
    if args.bilateral_grasp_demo and (args.freeze_banana or args.stable_grasp_demo):
        parser.error("--bilateral-grasp-demo cannot be combined with frozen or stable-grasp modes")
    if args.friction_grasp and not args.banana_contact_proxy:
        parser.error("--friction-grasp requires --banana-contact-proxy")
    if args.friction_grasp and (args.freeze_banana or args.stable_grasp_demo or args.bilateral_grasp_demo):
        parser.error("--friction-grasp cannot be combined with frozen or virtual-attach modes")
    print(f"[bridge] args parsed: {args}", flush=True)
    print(f"[bridge] script: {os.path.abspath(__file__)}", flush=True)
    asset_arg = args.asset or os.path.join(os.path.dirname(__file__), "..", "..", "..", "fairino3_robotiq.usd")
    asset = os.path.abspath(asset_arg)
    if not os.path.isfile(asset):
        parser.error(f"USD asset does not exist: {asset}")
    ros_domain_id = args.domain_id if args.domain_id is not None else int(os.environ.get("ROS_DOMAIN_ID", "42"))
    if not 0 <= ros_domain_id <= 232:
        parser.error("--domain-id must be between 0 and 232")
    os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")

    from isaacsim import SimulationApp

    print("[bridge] starting SimulationApp", flush=True)
    simulation_app = SimulationApp({"headless": args.headless})
    print("[bridge] SimulationApp started", flush=True)
    try:
        import omni
        print("[bridge] omni imported", flush=True)
        import omni.graph.core as og
        print("[bridge] graph imported", flush=True)
        from isaacsim.core.experimental.utils import app as app_utils
        print("[bridge] app utils imported", flush=True)
        from isaacsim.core.utils.stage import open_stage
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

        # Isaac Sim 6.0.1 uses the isaacsim.* ROS 2 extension namespace.
        app_utils.enable_extension("isaacsim.ros2.bridge")
        if args.friction_grasp:
            app_utils.enable_extension("isaacsim.sensors.experimental.physics")
        simulation_app.update()
        print(f"[bridge] opening asset: {asset}", flush=True)
        if not open_stage(asset):
            raise RuntimeError(f"failed to open USD stage: {asset}")
        stage = omni.usd.get_context().get_stage()
        robot_path = "/fairino3_v6_robot"
        articulation_root_path = robot_path + "/Geometry/base_link"
        repair_gripper_limits(stage)
        configure_position_drives(stage)
        configure_scene_collisions(
            stage, args.freeze_banana, not args.disable_scene_collisions, args.banana_contact_proxy,
            args.stable_grasp_demo, args.bilateral_grasp_demo
        )
        if args.friction_grasp:
            configure_friction_grasp_solver(stage)
        if not args.keep_light_shadows:
            reduce_detection_shadows(stage)
        robot_prim = stage.GetPrimAtPath(robot_path)
        if not robot_prim:
            raise RuntimeError(f"robot prim not found: {robot_path}")
        print(f"[bridge] robot found: {robot_path}", flush=True)

        camera_path = robot_path + "/Geometry/base_link/shoulder_link/upperarm_link/forearm_link/wrist1_link/wrist2_link/wrist3_link/wrist_camera"
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera_prim = camera.GetPrim()
        camera_prim.ApplyAPI("OmniSensorAPI")
        camera_prim.GetAttribute("omni:sensor:tickRate").Set(args.camera_tick_rate)

        from usdrt import Sdf as UsdrtSdf

        keys = og.Controller.Keys
        graph, _, _, _ = og.Controller.edit(
            {"graph_path": "/ROS_FR3", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("SensorDataQoS", "isaacsim.ros2.bridge.ROS2QoSProfile"),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("ComputeTransformTree", "isaacsim.core.nodes.IsaacComputeTransformTree"),
                    ("PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("StaticTool0", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("StaticGripperTcp", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("CreateCameraRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("PublishRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.SET_VALUES: [
                    ("Context.inputs:domain_id", ros_domain_id),
                    ("Context.inputs:useDomainIDEnvVar", False),
                    ("PublishClock.inputs:topicName", "/clock"),
                    ("ComputeTransformTree.inputs:parentPrim", UsdrtSdf.Path("/World")),
                    # Include the camera Prim so TF comes from the saved USD transform,
                    # rather than a duplicated hand-authored camera mount transform.
                    ("ComputeTransformTree.inputs:targetPrims", [UsdrtSdf.Path(articulation_root_path), UsdrtSdf.Path(camera_path)]),
                    ("PublishTransformTree.inputs:topicName", "/tf"),
                    ("StaticTool0.inputs:parentFrameId", "wrist3_link"),
                    ("StaticTool0.inputs:childFrameId", "tool0"),
                    ("StaticGripperTcp.inputs:parentFrameId", "robotiq_85_base_link"),
                    ("StaticGripperTcp.inputs:childFrameId", "gripper_tcp"),
                    ("StaticGripperTcp.inputs:translation", [0.000532, 0.000127, 0.000953]),
                    ("CreateCameraRenderProduct.inputs:cameraPrim", [UsdrtSdf.Path(camera_path)]),
                    ("CreateCameraRenderProduct.inputs:width", args.camera_width),
                    ("CreateCameraRenderProduct.inputs:height", args.camera_height),
                    ("SensorDataQoS.inputs:createProfile", "Sensor Data"),
                    ("PublishRgb.inputs:type", "rgb"),
                    ("PublishRgb.inputs:topicName", "/wrist_camera/image_raw"),
                    ("PublishRgb.inputs:frameId", "wrist_camera"),
                    ("PublishDepth.inputs:type", "depth"),
                    ("PublishDepth.inputs:topicName", "/wrist_camera/depth/image_raw"),
                    ("PublishDepth.inputs:frameId", "wrist_camera"),
                    ("PublishCameraInfo.inputs:topicName", "/wrist_camera/depth/camera_info"),
                    ("PublishCameraInfo.inputs:frameId", "wrist_camera"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    ("ReadSimulationTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                    ("OnPlaybackTick.outputs:tick", "ComputeTransformTree.inputs:execIn"),
                    ("ComputeTransformTree.outputs:execOut", "PublishTransformTree.inputs:execIn"),
                    ("Context.outputs:context", "PublishTransformTree.inputs:context"),
                    ("ReadSimulationTime.outputs:simulationTime", "PublishTransformTree.inputs:timeStamp"),
                    ("ComputeTransformTree.outputs:parentFrames", "PublishTransformTree.inputs:parentFrames"),
                    ("ComputeTransformTree.outputs:childFrames", "PublishTransformTree.inputs:childFrames"),
                    ("ComputeTransformTree.outputs:translations", "PublishTransformTree.inputs:translations"),
                    ("ComputeTransformTree.outputs:orientations", "PublishTransformTree.inputs:orientations"),
                    ("OnPlaybackTick.outputs:tick", "CreateCameraRenderProduct.inputs:execIn"),
                    ("CreateCameraRenderProduct.outputs:execOut", "PublishRgb.inputs:execIn"),
                    ("CreateCameraRenderProduct.outputs:renderProductPath", "PublishRgb.inputs:renderProductPath"),
                    ("CreateCameraRenderProduct.outputs:execOut", "PublishDepth.inputs:execIn"),
                    ("CreateCameraRenderProduct.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
                    ("CreateCameraRenderProduct.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
                    ("CreateCameraRenderProduct.outputs:renderProductPath", "PublishCameraInfo.inputs:renderProductPath"),
                    ("SensorDataQoS.outputs:qosProfile", "PublishRgb.inputs:qosProfile"),
                    ("SensorDataQoS.outputs:qosProfile", "PublishDepth.inputs:qosProfile"),
                    ("SensorDataQoS.outputs:qosProfile", "PublishCameraInfo.inputs:qosProfile"),
                ],
            },
        )
        for node in ("StaticTool0", "StaticGripperTcp"):
            og.Controller.set(og.Controller.attribute(f"/ROS_FR3/{node}.inputs:topicName"), "/tf_static")
            og.Controller.set(og.Controller.attribute(f"/ROS_FR3/{node}.inputs:staticPublisher"), True)
            og.Controller.connect(
                og.Controller.attribute("/ROS_FR3/OnPlaybackTick.outputs:tick"),
                og.Controller.attribute(f"/ROS_FR3/{node}.inputs:execIn"),
            )
            og.Controller.connect(
                og.Controller.attribute("/ROS_FR3/Context.outputs:context"),
                og.Controller.attribute(f"/ROS_FR3/{node}.inputs:context"),
            )
            og.Controller.connect(
                og.Controller.attribute("/ROS_FR3/ReadSimulationTime.outputs:simulationTime"),
                og.Controller.attribute(f"/ROS_FR3/{node}.inputs:timeStamp"),
            )
        for node in ("PublishRgb", "PublishDepth", "PublishCameraInfo"):
            og.Controller.connect(
                og.Controller.attribute("/ROS_FR3/Context.outputs:context"),
                og.Controller.attribute(f"/ROS_FR3/{node}.inputs:context"),
            )
        og.Controller.evaluate_sync(graph)
        print("[bridge] native ROS 2 joint-state graph created: /ROS_FR3", flush=True)
        print("[bridge] native clock publisher ready: /clock", flush=True)

        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        joint_names = ["j1", "j2", "j3", "j4", "j5", "j6",
                       "robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint",
                       "robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
                       "robotiq_85_left_finger_tip_joint", "robotiq_85_right_finger_tip_joint"]
        # Articulation tensors are only available after a physics reset.  Do
        # not publish a plausible-looking all-zero fallback if this fails.
        world = World(stage_units_in_meters=1.0)
        banana_visual = stage.GetPrimAtPath("/World/Banana") if args.banana_contact_proxy and not args.freeze_banana else None
        banana_proxy = stage.GetPrimAtPath("/World/BananaContactProxy") if banana_visual else None
        banana_visual_xform = UsdGeom.Xformable(banana_visual) if banana_visual else None
        banana_proxy_xform = UsdGeom.Xformable(banana_proxy) if banana_proxy else None
        banana_visual_translate = None
        banana_visual_orient = None
        if banana_visual_xform:
            ops = banana_visual_xform.GetOrderedXformOps()
            translate_ops = [op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
            orient_ops = [op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeOrient]
            banana_visual_translate = translate_ops[-1] if translate_ops else banana_visual_xform.AddTranslateOp()
            banana_visual_orient = orient_ops[-1] if orient_ops else banana_visual_xform.AddOrientOp()
            proxy_world = banana_proxy_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            banana_visual_translate.Set(proxy_world.ExtractTranslation())
            rotation = proxy_world.ExtractRotationQuat()
            banana_visual_orient.Set(
                Gf.Quatf(float(rotation.GetReal()), Gf.Vec3f(*rotation.GetImaginary()))
            )
        banana_proxy_translate = None
        if banana_proxy_xform:
            ops = [op for op in banana_proxy_xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
            banana_proxy_translate = ops[-1] if ops else banana_proxy_xform.AddTranslateOp()
        gripper_base_xform = None
        finger_tips = {}
        if args.stable_grasp_demo or args.bilateral_grasp_demo or args.friction_grasp:
            gripper_base = next(
                (prim for prim in stage.Traverse() if prim.GetName() == "robotiq_85_base_link"), None
            )
            if not gripper_base or not gripper_base.IsValid():
                raise RuntimeError("Robotiq base link not found for stable grasp demo")
            gripper_base_xform = UsdGeom.Xformable(gripper_base)
        if args.bilateral_grasp_demo or args.friction_grasp:
            for side in ("left", "right"):
                tip = next(
                    (prim for prim in stage.Traverse()
                     if prim.GetName() == f"robotiq_85_{side}_finger_tip_link"), None
                )
                if not tip or not tip.IsValid():
                    raise RuntimeError(f"Robotiq {side} finger tip link not found for bilateral grasp demo")
                finger_tips[side] = tip
        bilateral_pads = configure_bilateral_grasp_pads(stage) if args.bilateral_grasp_demo else {}
        pressure_sensor_paths = {}
        if args.friction_grasp:
            pressure_sensor_paths = configure_friction_grasp_pads(stage, finger_tips, banana_proxy)
        banana_attached = False
        banana_gripper_offset = None
        articulation = Articulation(robot_path)
        gripper_drive = UsdPhysics.DriveAPI.Apply(
            stage.GetPrimAtPath(f"{robot_path}/Physics/{GRIPPER_MASTER_JOINT}"), "angular"
        )
        gripper_close_drive = GRIPPER_FRICTION_CLOSE_DRIVE if args.friction_grasp else GRIPPER_MASTER_DRIVE
        if args.friction_grasp:
            max_force, stiffness, damping = gripper_close_drive
            gripper_drive.CreateMaxForceAttr(max_force).Set(max_force)
            gripper_drive.CreateStiffnessAttr(stiffness).Set(stiffness)
            gripper_drive.CreateDampingAttr(damping).Set(damping)
        world.reset()
        articulation.initialize()
        pressure_sensors = {}
        if args.friction_grasp:
            from isaacsim.sensors.experimental.physics import ContactSensor
            pressure_sensors = {
                side: ContactSensor(path) for side, path in pressure_sensor_paths.items()
            }
            print(
                f"[bridge] friction pressure control enabled: threshold={FRICTION_HOLD_FORCE_N:.2f}N",
                flush=True,
            )
        articulation_names = list(articulation.dof_names)
        missing = set(joint_names) - set(articulation_names)
        if missing:
            raise RuntimeError(f"articulation is missing expected DOFs: {sorted(missing)}")
        arm_indices = [articulation_names.index(name) for name in ARM_JOINTS]
        arm_limits = {}
        for name in ARM_JOINTS:
            joint = stage.GetPrimAtPath(f"{robot_path}/Physics/{name}")
            lower = joint.GetAttribute("physics:lowerLimit").Get()
            upper = joint.GetAttribute("physics:upperLimit").Get()
            if lower is None or upper is None:
                raise RuntimeError(f"{name} has no finite joint limits")
            arm_limits[name] = (math.radians(float(lower)), math.radians(float(upper)))
        print(f"[bridge] articulation state ready: {len(articulation_names)} DOF", flush=True)
        print(
            "[bridge] arm limits: "
            + ", ".join(f"{name}=[{lower:.4f}, {upper:.4f}]" for name, (lower, upper) in arm_limits.items()),
            flush=True,
        )
        startup_hold_positions = tuple(
            float(position) for position in articulation.get_joint_positions()[0, arm_indices]
        )
        articulation.set_joint_position_targets(startup_hold_positions, joint_indices=arm_indices)
        print("[bridge] startup hold: arm targets initialized from current positions", flush=True)
        smoke_target = None
        smoke_error = None
        if args.drive_smoke_test:
            if not args.test_frames or args.test_frames < 90:
                parser.error("--drive-smoke-test requires --test-frames >= 90")
            smoke_target = articulation.get_joint_positions()[0, arm_indices].copy()
            j6_index = ARM_JOINTS.index("j6")
            lower, upper = arm_limits["j6"]
            smoke_target[j6_index] = min(upper, max(lower, smoke_target[j6_index] + 0.1))
            print("[bridge] drive smoke test: holding arm and moving j6 by 0.1 rad", flush=True)

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image, JointState
        from std_msgs.msg import Float64
        from trajectory_msgs.msg import JointTrajectory

        executor = TrajectoryExecutor(arm_limits)
        gripper_ramp = GripperRamp(max_velocity=GRIPPER_COMMAND_VELOCITY_RAD_S)
        friction_hold_position = None
        friction_hold_relative_position = None
        friction_hold_finger_midpoint = None
        friction_hold_gripper_base = None
        friction_bias_locked = False
        last_friction_pressure_report = -float("inf")
        lift_tracking_until = -float("inf")
        last_lift_tracking_report = -float("inf")
        lift_tracking_active = False
        lift_gripper_xy_min = [0.0, 0.0]
        lift_gripper_xy_max = [0.0, 0.0]
        lift_gripper_z_min = 0.0
        lift_gripper_base_z_min = 0.0
        lift_gripper_z_min_time = 0.0
        lift_gripper_z_min_error = None
        bilateral_contacts = {"left": False, "right": False}
        bilateral_wait_reported = False

        master_index = articulation_names.index(GRIPPER_MASTER_JOINT)
        rclpy.init(args=None)
        trajectory_node = rclpy.create_node("fr3_joint_trajectory_subscriber")
        gripper_node = rclpy.create_node("fr3_gripper_command_subscriber")

        def receive_trajectory(message) -> None:
            nonlocal friction_bias_locked, lift_tracking_active, lift_tracking_until, last_lift_tracking_report
            nonlocal lift_gripper_xy_min, lift_gripper_xy_max
            nonlocal lift_gripper_z_min, lift_gripper_z_min_time, lift_gripper_z_min_error
            nonlocal lift_gripper_base_z_min
            try:
                positions = articulation.get_joint_positions()
                current_positions = (
                    positions[0, arm_indices] if positions is not None else last_safe_arm_positions
                )
                error = executor.submit(
                    message, world.current_time, current_positions, append=len(message.points) == 1
                )
                if error:
                    print(f"[bridge] rejected {args.trajectory_topic}: {error}", flush=True)
                elif executor.new_trajectory:
                    if abs(executor.j2_compensation) >= 1e-6:
                        print(
                            f"[bridge] j2 tracking compensation: {executor.j2_compensation:+.4f} rad",
                            flush=True,
                        )
                    final_target = ", ".join(
                        f"{name}={position:.4f}"
                        for name, position in zip(ARM_JOINTS, executor.points[-1][1])
                    )
                    print(
                        f"[bridge] accepted {args.trajectory_topic}: {len(message.points)} point(s), "
                        f"{executor.points[-1][0]:.3f}s, final_target=[{final_target}]",
                        flush=True,
                    )
                    if args.friction_grasp and friction_hold_position is not None:
                        first_error = [
                            target - actual
                            for target, actual in zip(executor.points[0][1], current_positions)
                        ]
                        print(
                            "[bridge] lift start error: "
                            + ", ".join(
                                f"{name}={error:+.4f}" for name, error in zip(ARM_JOINTS, first_error)
                            ),
                            flush=True,
                        )
                        friction_bias_locked = True
                        lift_tracking_active = True
                        last_lift_tracking_report = -float("inf")
                        lift_gripper_xy_min = [0.0, 0.0]
                        lift_gripper_xy_max = [0.0, 0.0]
                        lift_gripper_z_min = 0.0
                        lift_gripper_base_z_min = 0.0
                        lift_gripper_z_min_time = 0.0
                        lift_gripper_z_min_error = None
                    if lift_tracking_active:
                        lift_tracking_until = executor.start_time + executor.points[-1][0] + 0.5
                elif lift_tracking_active:
                    lift_tracking_until = executor.start_time + executor.points[-1][0] + 0.5
            except Exception as error:
                print(f"[bridge] rejected {args.trajectory_topic}: callback failed: {error}", flush=True)

        trajectory_node.create_subscription(
            JointTrajectory,
            args.trajectory_topic,
            receive_trajectory,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        def receive_gripper_target(message) -> None:
            nonlocal friction_bias_locked, friction_hold_finger_midpoint, friction_hold_gripper_base
            nonlocal friction_hold_position, friction_hold_relative_position
            target = float(message.data)
            if args.friction_grasp and friction_hold_position is not None and target >= friction_hold_position:
                return
            if args.friction_grasp and friction_hold_position is not None:
                max_force, stiffness, damping = gripper_close_drive
                gripper_drive.CreateMaxForceAttr(max_force).Set(max_force)
                gripper_drive.CreateStiffnessAttr(stiffness).Set(stiffness)
                gripper_drive.CreateDampingAttr(damping).Set(damping)
            friction_hold_position = None
            friction_hold_relative_position = None
            friction_hold_finger_midpoint = None
            friction_hold_gripper_base = None
            friction_bias_locked = False
            error = gripper_ramp.set_target(target)
            if error:
                print(f"[bridge] rejected {GRIPPER_COMMAND_TOPIC}: {error}", flush=True)

        gripper_node.create_subscription(Float64, GRIPPER_COMMAND_TOPIC, receive_gripper_target, 1)
        gripper_state_publisher = gripper_node.create_publisher(Float64, GRIPPER_STATE_TOPIC, 10)
        joint_state_publisher = trajectory_node.create_publisher(JointState, "/joint_states", 10)
        direct_rgb_publisher = trajectory_node.create_publisher(
            Image, "/wrist_camera/image_raw_direct", qos_profile_sensor_data
        )
        direct_depth_publisher = trajectory_node.create_publisher(
            Image, "/wrist_camera/depth/image_raw_direct", qos_profile_sensor_data
        )
        direct_depth_info_publisher = trajectory_node.create_publisher(
            CameraInfo, "/wrist_camera/depth/camera_info_direct", qos_profile_sensor_data
        )
        ros_executor = SingleThreadedExecutor()
        ros_executor.add_node(trajectory_node)
        ros_executor.add_node(gripper_node)
        print("[bridge] joint-state publisher ready: /joint_states", flush=True)
        print(f"[bridge] trajectory subscriber ready: {args.trajectory_topic}", flush=True)
        print(f"[bridge] gripper bridge ready: {GRIPPER_COMMAND_TOPIC}", flush=True)

        timeline = omni.timeline.get_timeline_interface()
        print(
            "[bridge] camera publishers configured: "
            "/wrist_camera/image_raw, /wrist_camera/depth/image_raw, "
            "/wrist_camera/depth/camera_info",
            flush=True,
        )
        stage.SetEndTimeCode(1_000_000.0)
        timeline.set_end_time(1_000_000.0)
        simulation_app.update()
        print(f"[bridge] timeline end time: {timeline.get_end_time():.1f} s", flush=True)
        timeline.play()
        viewport_rgb_annotator = None
        viewport_depth_annotator = None
        if args.rgb_from_active_viewport:
            if args.headless:
                raise RuntimeError(
                    "--rgb-from-active-viewport requires Isaac windowed mode; remove --headless."
                )
            # The GUI viewport is demonstrably rendering the banana correctly.
            # Reuse that exact render product instead of creating an off-screen
            # product, which is black in this Isaac/WSL configuration.
            from omni.kit.viewport.utility import get_active_viewport
            import omni.replicator.core as rep
            import numpy as np

            simulation_app.update()
            viewport = get_active_viewport()
            if viewport is None:
                raise RuntimeError("Isaac active viewport is unavailable for RGB detection.")
            viewport.camera_path = camera_path
            viewport.set_texture_resolution([args.camera_width, args.camera_height])
            simulation_app.update()
            # Attach directly to the render product displayed by the GUI.
            # Camera.get_rgba() cannot read this Hydra texture reliably in 6.0.1.
            viewport_rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            viewport_rgb_annotator.attach(viewport.get_render_product_path())
            viewport_depth_annotator = rep.AnnotatorRegistry.get_annotator(
                "distance_to_image_plane"
            )
            viewport_depth_annotator.attach(viewport.get_render_product_path())
            print(
                "[bridge] GUI viewport RGB-D publishers ready: "
                "/wrist_camera/image_raw_direct, /wrist_camera/depth/image_raw_direct",
                flush=True,
            )
        frames = 0
        previous_sim_time = world.current_time
        last_direct_rgb_time = -float("inf")
        camera_debug = os.environ.get("FR3_CAMERA_DEBUG") == "1"
        last_camera_heartbeat = time.monotonic()
        direct_rgb_frames = 0
        direct_rgb_status = "not_attempted"
        last_safe_arm_positions = tuple(
            float(position) for position in articulation.get_joint_positions()[0, arm_indices]
        )
        tracking_bias = (0.0,) * len(ARM_JOINTS)
        last_startup_tracking_report = -float("inf")

        def gripper_base_world_position():
            if not gripper_base_xform:
                return None
            return tuple(
                gripper_base_xform.ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ).ExtractTranslation()
            )

        def finger_midpoint_world_position():
            if not finger_tips:
                return None
            tip_positions = [
                UsdGeom.Xformable(finger_tips[side]).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ).ExtractTranslation()
                for side in ("left", "right")
            ]
            return tuple((left + right) / 2.0 for left, right in zip(*tip_positions))

        def banana_relative_to_finger_midpoint():
            midpoint = finger_midpoint_world_position()
            if not banana_proxy_xform or midpoint is None:
                return None
            banana_position = banana_proxy_xform.ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            ).ExtractTranslation()
            return tuple(value - reference for value, reference in zip(banana_position, midpoint))

        # In headless smoke tests Kit may report not-running immediately after
        # a world reset.  An explicit finite test still advances the requested
        # physics frames and therefore exercises the native publisher node.
        # Kit's windowed is_running flag becomes false during this scene's
        # first renderer update even though physics remains usable.  Keep the
        # bridge alive until Ctrl+C, a safety stop, or an explicit test limit.
        while not args.test_frames or frames < args.test_frames:
            step_started = time.monotonic()
            if camera_debug and step_started - last_camera_heartbeat >= 5.0:
                print(
                    f"[bridge] camera heartbeat frames={frames} sim_time={world.current_time:.3f} "
                    f"direct_rgb_frames={direct_rgb_frames} direct_rgb={direct_rgb_status}",
                    flush=True,
                )
                last_camera_heartbeat = step_started
            # A small wait lets Fast DDS dispatch an arriving command without
            # busy-spinning the simulation loop while the controller is idle.
            ros_executor.spin_once(timeout_sec=0.001)
            raw_arm_positions = tuple(
                float(position) for position in articulation.get_joint_positions()[0, arm_indices]
            )
            target = executor.target_at(world.current_time)
            if smoke_target is not None:
                target = smoke_target
            arm_positions = (
                tuple(
                    closest_joint_angle(position, goal, *arm_limits[name])
                    for name, position, goal in zip(ARM_JOINTS, raw_arm_positions, target)
                )
                if target is not None else raw_arm_positions
            )
            if (
                args.startup_tracking and target is None and
                world.current_time - last_startup_tracking_report >= STARTUP_TRACKING_REPORT_SEC
            ):
                print(
                    "[bridge] startup tracking: actual=["
                    + ", ".join(
                        f"{name}={position:+.4f}"
                        for name, position in zip(ARM_JOINTS, arm_positions)
                    )
                    + "] hold_error=["
                    + ", ".join(
                        f"{name}={goal - position:+.4f}"
                        for name, goal, position in zip(ARM_JOINTS, startup_hold_positions, arm_positions)
                    )
                    + "]",
                    flush=True,
                )
                last_startup_tracking_report = world.current_time
            unsafe = [
                (name, position)
                for name, position in zip(ARM_JOINTS, arm_positions)
                if target is not None and (
                    not math.isfinite(position)
                    or abs(position) > MAX_SAFE_ARM_ABS_RAD
                    or position < arm_limits[name][0] - ACTUAL_LIMIT_GRACE_RAD
                    or position > arm_limits[name][1] + ACTUAL_LIMIT_GRACE_RAD
                )
            ]
            if unsafe:
                print(
                    "[bridge] SAFETY STOP: unsafe arm state "
                    + ", ".join(f"{name}={position!r}" for name, position in unsafe)
                    + "; raw_arm_state="
                    + ", ".join(f"{name}={position!r}" for name, position in zip(ARM_JOINTS, raw_arm_positions))
                    + (f"; active_target={tuple(float(value) for value in target)!r}" if target is not None else ""),
                    flush=True,
                )
                articulation.set_joint_position_targets(last_safe_arm_positions, joint_indices=arm_indices)
                timeline.stop()
                break
            last_safe_arm_positions = arm_positions
            if target is not None:
                friction_support_active = (
                    args.friction_grasp and (
                        friction_hold_position is not None or
                        (gripper_ramp.target is not None and gripper_ramp.target >= 0.70)
                    )
                )
                if smoke_target is None and friction_support_active:
                    if should_update_friction_bias(smoke_target, friction_bias_locked):
                        tracking_bias = update_tracking_bias(
                            target,
                            arm_positions,
                            tracking_bias,
                            world.current_time - previous_sim_time,
                            TRACKING_HOLD_COMPENSATION_GAIN,
                        )
                    control_target = tuple(
                        max(arm_limits[name][0], min(arm_limits[name][1], goal + bias))
                        for name, goal, bias in zip(ARM_JOINTS, target, tracking_bias)
                    )
                else:
                    tracking_bias = (0.0,) * len(ARM_JOINTS)
                    control_target = target
                articulation.set_joint_position_targets(control_target, joint_indices=arm_indices)
            else:
                tracking_bias = update_tracking_bias(
                    startup_hold_positions,
                    arm_positions,
                    tracking_bias,
                    world.current_time - previous_sim_time,
                )
                control_target = tuple(
                    max(arm_limits[name][0], min(arm_limits[name][1], goal + bias))
                    for name, goal, bias in zip(ARM_JOINTS, startup_hold_positions, tracking_bias)
                )
                articulation.set_joint_position_targets(control_target, joint_indices=arm_indices)
            current_gripper_position = float(articulation.get_joint_positions()[0, master_index])
            gripper_target = gripper_ramp.step(
                current_gripper_position,
                max(0.0, world.current_time - previous_sim_time),
            )
            if gripper_target is not None:
                articulation.set_joint_position_targets((gripper_target,), joint_indices=[master_index])
            if args.bilateral_grasp_demo and not banana_attached:
                banana_position = banana_proxy_xform.ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ).ExtractTranslation()
                pad_separations = {}
                for side, (_, translate) in bilateral_pads.items():
                    pad_position = UsdGeom.Xformable(finger_tips[side]).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    ).ExtractTranslation()
                    if not all(math.isfinite(float(value)) for value in pad_position):
                        pad_separations[side] = float("inf")
                        continue
                    translate.Set(pad_position)
                    closest_x = max(-0.06, min(0.06, pad_position[0] - banana_position[0]))
                    pad_separations[side] = math.sqrt(
                        (pad_position[0] - banana_position[0] - closest_x) ** 2
                        + (pad_position[1] - banana_position[1]) ** 2
                        + (pad_position[2] - banana_position[2]) ** 2
                    )
                close_active = gripper_ramp.target is not None and gripper_ramp.target >= 0.70
                both_near = close_active and all(
                    pad_separations.get(side, float("inf")) <= BILATERAL_CONTACT_DISTANCE_M
                    for side in bilateral_pads
                )
                if close_active and not both_near and not bilateral_wait_reported:
                    print(
                        "[bridge] bilateral attach waiting: "
                        f"left={pad_separations.get('left', float('inf')):.4f}m "
                        f"right={pad_separations.get('right', float('inf')):.4f}m "
                        f"threshold={BILATERAL_CONTACT_DISTANCE_M:.4f}m",
                        flush=True,
                    )
                    bilateral_wait_reported = True
                for side, (pad, _) in bilateral_pads.items():
                    UsdPhysics.CollisionAPI(pad).CreateCollisionEnabledAttr(True).Set(False)
                    if both_near:
                        bilateral_contacts[side] = True
            if (
                args.stable_grasp_demo and not banana_attached and
                gripper_ramp.target is not None and gripper_ramp.target >= 0.70 and
                banana_proxy_xform and gripper_base_xform
            ):
                banana_position = banana_proxy_xform.ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ).ExtractTranslation()
                gripper_position = gripper_base_xform.ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ).ExtractTranslation()
                banana_gripper_offset = banana_position - gripper_position
                banana_attached = True
                print("[bridge] banana attached to gripper for stable demo", flush=True)
            if (
                args.bilateral_grasp_demo and not banana_attached and
                bilateral_contacts["left"] and bilateral_contacts["right"] and
                banana_proxy_xform and gripper_base_xform
            ):
                banana_position = banana_proxy_xform.ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ).ExtractTranslation()
                gripper_position = gripper_base_xform.ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ).ExtractTranslation()
                banana_gripper_offset = banana_position - gripper_position
                UsdPhysics.CollisionAPI(banana_proxy).CreateCollisionEnabledAttr(True).Set(False)
                UsdPhysics.RigidBodyAPI(banana_proxy).CreateKinematicEnabledAttr(False).Set(True)
                # Keep the commanded slow close running.  Snapping its target
                # to the measured position here creates a velocity step.
                banana_attached = True
                print("[bridge] bilateral finger contact; banana attached", flush=True)
            gripper_state_publisher.publish(Float64(data=current_gripper_position))
            state = JointState()
            state.header.stamp.sec = int(world.current_time)
            state.header.stamp.nanosec = int((world.current_time % 1.0) * 1_000_000_000)
            state.name = articulation_names
            state_positions = [float(position) for position in articulation.get_joint_positions()[0]]
            if target is not None:
                for index, position in zip(arm_indices, arm_positions):
                    state_positions[index] = position
            state.position = state_positions
            joint_state_publisher.publish(state)
            previous_sim_time = world.current_time
            # The ROS camera helper reads an off-screen render product; without
            # rendering it publishes valid rgb8 messages filled with black.
            world.step(render=True)
            finger_midpoint = finger_midpoint_world_position() if lift_tracking_active else None
            finger_midpoint_delta = (
                tuple(
                    value - reference
                    for value, reference in zip(finger_midpoint, friction_hold_finger_midpoint)
                )
                if finger_midpoint is not None and friction_hold_finger_midpoint is not None
                else None
            )
            gripper_base = gripper_base_world_position() if lift_tracking_active else None
            gripper_base_delta = (
                tuple(
                    value - reference
                    for value, reference in zip(gripper_base, friction_hold_gripper_base)
                )
                if gripper_base is not None and friction_hold_gripper_base is not None
                else None
            )
            if gripper_base_delta is not None:
                lift_gripper_base_z_min = min(lift_gripper_base_z_min, gripper_base_delta[2])
            if finger_midpoint_delta is not None:
                for axis in range(2):
                    lift_gripper_xy_min[axis] = min(lift_gripper_xy_min[axis], finger_midpoint_delta[axis])
                    lift_gripper_xy_max[axis] = max(lift_gripper_xy_max[axis], finger_midpoint_delta[axis])
                if finger_midpoint_delta[2] < lift_gripper_z_min:
                    lift_gripper_z_min = finger_midpoint_delta[2]
                    lift_gripper_z_min_time = world.current_time - executor.start_time
                    desired_at_min = executor.target_at(world.current_time)
                    actual_at_min = articulation.get_joint_positions()[0, arm_indices]
                    lift_gripper_z_min_error = tuple(
                        desired - actual for desired, actual in zip(desired_at_min, actual_at_min)
                    )
            if (
                world.current_time <= lift_tracking_until and
                world.current_time - last_lift_tracking_report >= LIFT_TRACKING_REPORT_SEC
            ):
                desired = executor.target_at(world.current_time)
                actual = articulation.get_joint_positions()[0, arm_indices]
                remaining = max(0.0, executor.start_time + executor.points[-1][0] - world.current_time)
                forces = {
                    side: sensor.get_sensor_reading().value
                    for side, sensor in pressure_sensors.items()
                }
                try:
                    measured_efforts = articulation.get_measured_joint_efforts()[0]
                    lift_efforts = {
                        name: float(measured_efforts[arm_indices[ARM_JOINTS.index(name)]])
                        for name in ("j2", "j3", "j4", "j5")
                    }
                except Exception:
                    lift_efforts = None
                banana_relative_position = banana_relative_to_finger_midpoint()
                banana_relative_delta = (
                    tuple(
                        value - reference
                        for value, reference in zip(
                            banana_relative_position, friction_hold_relative_position
                        )
                    )
                    if banana_relative_position is not None and friction_hold_relative_position is not None
                    else None
                )
                print(
                    "[bridge] lift tracking: "
                    f"remaining={remaining:.2f}s, error=["
                    + ", ".join(
                        f"{name}={target - position:+.4f}"
                        for name, target, position in zip(ARM_JOINTS, desired, actual)
                    )
                    + "] compensation=["
                    + ", ".join(
                        f"{ARM_JOINTS[index]}={tracking_bias[index]:+.4f}"
                        for index in TRACKING_COMPENSATION_INDICES
                    )
                    + "] pressure=["
                    + ", ".join(f"{side}={force:.2f}N" for side, force in forces.items())
                    + "] banana_relative=["
                    + (", ".join(f"{value:+.4f}" for value in banana_relative_position)
                       if banana_relative_position is not None else "unavailable")
                    + "] banana_delta=["
                    + (", ".join(f"{value:+.4f}" for value in banana_relative_delta)
                       if banana_relative_delta is not None else "unavailable")
                    + "] gripper_delta=["
                    + (", ".join(f"{value:+.4f}" for value in finger_midpoint_delta)
                       if finger_midpoint_delta is not None else "unavailable")
                    + "] gripper_base_delta=["
                    + (", ".join(f"{value:+.4f}" for value in gripper_base_delta)
                       if gripper_base_delta is not None else "unavailable")
                    + f"] gripper_base_z_min={lift_gripper_base_z_min:+.4f}"
                    + " gripper_xy_peak=["
                    + ", ".join(
                        f"{axis}={max(abs(low), abs(high)):.4f}"
                        for axis, low, high in zip("xy", lift_gripper_xy_min, lift_gripper_xy_max)
                    )
                    + "] gripper_xy_p2p=["
                    + ", ".join(
                        f"{axis}={high - low:.4f}"
                        for axis, low, high in zip("xy", lift_gripper_xy_min, lift_gripper_xy_max)
                    )
                    + f"] gripper_z_min={lift_gripper_z_min:+.4f}@{lift_gripper_z_min_time:.3f}s"
                    + " z_min_error=["
                    + (", ".join(
                        f"{name}={lift_gripper_z_min_error[ARM_JOINTS.index(name)]:+.4f}"
                        for name in ("j2", "j3")
                    ) if lift_gripper_z_min_error is not None else "unavailable")
                    + "] effort=["
                    + (", ".join(
                        f"{name}={lift_efforts[name]:+.1f}/{ARM_DRIVES[name][0]:.1f} "
                        f"({abs(lift_efforts[name]) / ARM_DRIVES[name][0]:.0%})"
                        for name in ("j2", "j3", "j4", "j5")
                    ) if lift_efforts is not None else "unavailable")
                    + "]",
                    flush=True,
                )
                last_lift_tracking_report = world.current_time
            elif lift_tracking_active and world.current_time > lift_tracking_until:
                lift_tracking_active = False
            if (
                args.friction_grasp and friction_hold_position is None and
                gripper_ramp.target is not None
            ):
                forces = {}
                sensor_valid = {}
                for side, sensor in pressure_sensors.items():
                    reading = sensor.get_sensor_reading()
                    sensor_valid[side] = reading.is_valid
                    forces[side] = reading.value if reading.is_valid and reading.in_contact else 0.0
                if world.current_time - last_friction_pressure_report >= 0.5:
                    print(
                        "[bridge] friction pressure: "
                        + ", ".join(
                            f"{side}={force:.2f}N({'ok' if sensor_valid[side] else 'invalid'})"
                            for side, force in forces.items()
                        ) + f", target={gripper_ramp.target:.3f} rad",
                        flush=True,
                    )
                    last_friction_pressure_report = world.current_time
                current_position = float(articulation.get_joint_positions()[0, master_index])
                closing = gripper_ramp.target > current_position + 0.001
                if closing and all(force >= FRICTION_HOLD_FORCE_N for force in forces.values()):
                    friction_hold_position = current_position
                    friction_hold_relative_position = banana_relative_to_finger_midpoint()
                    friction_hold_finger_midpoint = finger_midpoint_world_position()
                    friction_hold_gripper_base = gripper_base_world_position()
                    gripper_ramp.target = friction_hold_position
                    gripper_ramp.commanded = friction_hold_position
                    max_force, stiffness, damping = GRIPPER_HOLD_DRIVE
                    gripper_drive.CreateMaxForceAttr(max_force).Set(max_force)
                    gripper_drive.CreateStiffnessAttr(stiffness).Set(stiffness)
                    gripper_drive.CreateDampingAttr(damping).Set(damping)
                    print(
                        "[bridge] friction hold: "
                        + ", ".join(f"{side}={force:.2f}N" for side, force in forces.items())
                        + f", position={current_position:.3f} rad, target={friction_hold_position:.3f} rad, "
                        f"drive={GRIPPER_HOLD_DRIVE}, relative=["
                        + (", ".join(f"{value:+.4f}" for value in friction_hold_relative_position)
                           if friction_hold_relative_position is not None else "unavailable")
                        + "]",
                        flush=True,
                    )
            if banana_attached and banana_proxy_translate and gripper_base_xform:
                gripper_position = gripper_base_xform.ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ).ExtractTranslation()
                banana_proxy_translate.Set(gripper_position + banana_gripper_offset)
            if banana_visual_translate and banana_proxy_xform:
                proxy_world = banana_proxy_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                banana_visual_translate.Set(proxy_world.ExtractTranslation())
                rotation = proxy_world.ExtractRotationQuat()
                banana_visual_orient.Set(
                    Gf.Quatf(float(rotation.GetReal()), Gf.Vec3f(*rotation.GetImaginary()))
                )
            simulation_app.update()
            if (
                viewport_rgb_annotator is not None
                and viewport_depth_annotator is not None
                and world.current_time - last_direct_rgb_time >= 1.0 / max(0.1, args.camera_tick_rate)
            ):
                rgba = viewport_rgb_annotator.get_data()
                depth = viewport_depth_annotator.get_data()
                if rgba is not None and rgba.size and depth is not None and depth.size:
                    direct_rgb_status = f"{rgba.shape}/{rgba.dtype}"
                    rgb = rgba[:, :, :3]
                    if rgb.dtype != np.uint8:
                        rgb = np.clip(rgb * 255.0, 0.0, 255.0).astype(np.uint8)
                    image = Image()
                    image.header.stamp.sec = int(world.current_time)
                    image.header.stamp.nanosec = int(
                        (world.current_time % 1.0) * 1_000_000_000
                    )
                    image.header.frame_id = "wrist_camera"
                    image.height, image.width = rgb.shape[:2]
                    image.encoding = "rgb8"
                    image.is_bigendian = 0
                    image.step = image.width * 3
                    image.data = rgb.tobytes()
                    direct_rgb_publisher.publish(image)
                    depth = np.asarray(depth, dtype=np.float32).squeeze()
                    depth_image = Image()
                    depth_image.header = image.header
                    depth_image.height, depth_image.width = depth.shape[:2]
                    depth_image.encoding = "32FC1"
                    depth_image.is_bigendian = 0
                    depth_image.step = depth_image.width * 4
                    depth_image.data = depth.tobytes()
                    direct_depth_publisher.publish(depth_image)
                    depth_info = CameraInfo()
                    depth_info.header = image.header
                    depth_info.height = image.height
                    depth_info.width = image.width
                    fx = args.camera_width * (2.0 / 3.0)
                    fy = args.camera_height * (8.0 / 9.0)
                    cx = args.camera_width / 2.0
                    cy = args.camera_height / 2.0
                    depth_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
                    depth_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
                    depth_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                    depth_info.distortion_model = "plumb_bob"
                    direct_depth_info_publisher.publish(depth_info)
                    last_direct_rgb_time = world.current_time
                    direct_rgb_frames += 1
                else:
                    direct_rgb_status = "empty"
            frames += 1
            if args.test_frames and frames >= args.test_frames:
                break
            sleep_sec = 1.0 / max(1.0, args.sim_rate) - (time.monotonic() - step_started)
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
        if smoke_target is not None:
            actual = articulation.get_joint_positions()[0, arm_indices]
            smoke_error = abs(float(actual[ARM_JOINTS.index("j6")] - smoke_target[ARM_JOINTS.index("j6")]))
            print(f"[bridge] drive smoke test j6 error: {smoke_error:.4f} rad", flush=True)
        timeline.stop()
        render_product = og.Controller.attribute(
            "/ROS_FR3/CreateCameraRenderProduct.outputs:renderProductPath"
        ).get()
        if not render_product:
            raise RuntimeError("wrist camera render product was not created")
        print(
            "[bridge] camera publishers ready: "
            "/wrist_camera/image_raw, /wrist_camera/depth/image_raw, "
            "/wrist_camera/depth/camera_info",
            flush=True,
        )
        print(f"[bridge] completed {frames} simulation frames", flush=True)
        ros_executor.shutdown()
        trajectory_node.destroy_node()
        gripper_node.destroy_node()
        if viewport_rgb_annotator is not None:
            viewport_rgb_annotator.detach()
        rclpy.shutdown()
        if smoke_error is not None and smoke_error > 0.03:
            raise RuntimeError(f"position drive did not track j6 target (error {smoke_error:.4f} rad)")
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
