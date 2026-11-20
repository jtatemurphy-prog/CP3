# Copyright (c) 2026, ai_pd_tasks project.
# SPDX-License-Identifier: BSD-3-Clause

"""Author the car's physical connections as PhysX joints (call after the part prims exist).

* ``stud``  -> fixed joint with a break force (stud clutch), enabled when the part is pressed home;
* ``cross`` -> fixed joint (cross-axle friction fit);
* ``hinge`` -> revolute joint about the axle (axle turning in a round Technic hole);
* ``motor`` -> revolute joint with an angular velocity drive that reproduces the DC-motor line
  ``tau = tau_stall (1 - w / w0)`` (damping = tau_stall / w0, target = w0, max force = tau_stall);
* gear meshes -> real tooth contact (analytic tooth hulls); PhysX gear joints are optional and off
  by default (they did not hold the tooth ratio on CPU D6 joints in testing).

Joints are authored in the *design* relative pose, so enabling a joint pulls a part the last fraction of a
millimetre home, like a stud clicking in. Collisions are filtered between mated parts only (their collision
hulls overlap by design: studs are visual, cross holes are not modelled in the gear/wheel hulls).
"""

from __future__ import annotations

import math

from . import car_design as cd

JOINT_SCOPE = "Joints"


def _qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # v' = q v q*
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def _rel_pose(parent: cd.PartInstance, child: cd.PartInstance):
    """Child origin pose (position, quaternion xyzw) in the parent frame."""
    qp, qc = cd.part_quat_xyzw(parent), cd.part_quat_xyzw(child)
    d = tuple(c - p for c, p in zip(child.pos, parent.pos))
    return _qrot(_qconj(qp), d), cd._qmul(_qconj(qp), qc)


def _quatf(q_xyzw):
    from pxr import Gf  # noqa: PLC0415

    x, y, z, w = q_xyzw
    return Gf.Quatf(w, Gf.Vec3f(x, y, z))


def joint_path(env_path: str, conn: cd.Connection) -> str:
    return f"{env_path}/{JOINT_SCOPE}/{conn.kind}_{conn.child}__{conn.parent}"


def author_connection(stage, env_path: str, part_path, conn: cd.Connection, enabled: bool):
    """Create the joint for ``conn``. ``part_path(name)`` maps a part name to its prim path."""
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics  # noqa: PLC0415

    parent, child = cd.PART_BY_NAME[conn.parent], cd.PART_BY_NAME[conn.child]
    local_pos, local_quat = _rel_pose(parent, child)
    path = joint_path(env_path, conn)
    if conn.kind in ("stud", "cross"):
        joint = UsdPhysics.FixedJoint.Define(stage, path)
    else:
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        joint.CreateAxisAttr("X")  # axles, and the motor shaft, run along their local x
    joint.CreateBody0Rel().SetTargets([Sdf.Path(part_path(conn.parent))])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(part_path(conn.child))])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos))
    joint.CreateLocalRot0Attr().Set(_quatf(local_quat))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateJointEnabledAttr(enabled)
    joint.CreateCollisionEnabledAttr(False)
    if conn.kind == "stud":
        joint.CreateBreakForceAttr(cd.STUD_CLUTCH_N_PER_STUD * max(conn.studs, 1))
    if conn.kind == "hinge":  # axle friction in the hole (see cd.HINGE_DAMPING_NM_S_PER_RAD)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(cd.HINGE_DAMPING_NM_S_PER_RAD * math.pi / 180.0)  # USD: per degree/s
        drive.CreateMaxForceAttr(0.01)
        drive.CreateTargetVelocityAttr(0.0)
    if conn.kind == "motor":
        p = cd.motor_drive_params()
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(p["damping_nm_s_per_deg"])
        drive.CreateMaxForceAttr(p["max_force_nm"])
        drive.CreateTargetVelocityAttr(0.0)  # motor off until launch
    PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
    return joint


def set_motor_velocity(stage, env_path: str, fraction: float) -> None:
    """Switch the motor: ``fraction`` of the no-load speed (1 = full forward, 0 = off, -1 = reverse)."""
    from pxr import UsdPhysics  # noqa: PLC0415

    conn = next(c for c in cd.CONNECTIONS if c.kind == "motor")
    drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(joint_path(env_path, conn)), "angular")
    drive.GetTargetVelocityAttr().Set(fraction * cd.motor_drive_params()["target_velocity_deg_s"])


def hinge_of_shaft(shaft: str) -> cd.Connection:
    return next(c for c in cd.CONNECTIONS if c.child == shaft and c.kind in ("hinge", "motor"))


def author_gear(stage, env_path: str, part_path, mesh: cd.GearMesh, enabled: bool):
    from pxr import PhysxSchema, Sdf  # noqa: PLC0415

    path = f"{env_path}/{JOINT_SCOPE}/gear_{mesh.gear_a}__{mesh.gear_b}"
    gear = PhysxSchema.PhysxPhysicsGearJoint.Define(stage, path)
    gear.CreateBody0Rel().SetTargets([Sdf.Path(part_path(mesh.shaft_a))])
    gear.CreateBody1Rel().SetTargets([Sdf.Path(part_path(mesh.shaft_b))])
    gear.CreateHinge0Rel().SetTargets([Sdf.Path(joint_path(env_path, hinge_of_shaft(mesh.shaft_a)))])
    gear.CreateHinge1Rel().SetTargets([Sdf.Path(joint_path(env_path, hinge_of_shaft(mesh.shaft_b)))])
    gear.CreateGearRatioAttr(mesh.teeth_a / mesh.teeth_b)
    gear.CreateJointEnabledAttr(enabled)
    return gear


def mated_pairs() -> list[tuple[str, str]]:
    """Part pairs whose collisions are filtered.

    Filtered: every stud fit, the cross fits and the motor output. Studs are visual only and the hulls of a seated
    part and its parent are 0.04-0.1 mm apart: flat face on flat face. In PhysX (TGS, 32 iterations) that contact
    now and then threw the pressed part off as it came within the contact offset (~0.8 mm above its seat) -- decks
    and 1x4 plates, at 9-70 m/s. Filtered, a pressed part would sink as far as the press goes, so presses go only
    0.1 mm past the seat (``LegoSkills.place_on_studs``), well inside the 0.6 mm click window. The cross holes of
    gears, wheels and the motor are not cut into their hulls, so those pairs overlap by design. *Not* filtered:
    axles in round Technic holes -- the holes are exact in the collision geometry (0.25 mm radial clearance), so an
    axle is carried by the holes while it is being inserted, before its hinge joint is enabled.
    """
    import os  # noqa: PLC0415

    extra = {tuple(p.split(":")) for p in os.environ.get("LEGO_FILTER_EXTRA", "").split(",") if ":" in p}  # debug
    return sorted({(c.child, c.parent) for c in cd.CONNECTIONS if c.kind != "hinge"} | extra)


def author_collision_filters(stage, part_path) -> None:
    from pxr import Sdf, UsdPhysics  # noqa: PLC0415

    for a, b in mated_pairs():
        api = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(part_path(a)))
        api.CreateFilteredPairsRel().AddTarget(Sdf.Path(part_path(b)))


def author_all(stage, env_path: str, part_path, enabled: bool, filters: bool = True, gear_joints: bool = False) -> None:
    """Author every connection. Gears mesh through tooth contact unless ``gear_joints`` (PhysX gear joints)."""
    for conn in cd.CONNECTIONS:
        author_connection(stage, env_path, part_path, conn, enabled)
    if gear_joints:
        for mesh in cd.GEAR_MESHES:
            author_gear(stage, env_path, part_path, mesh, enabled)
    if filters:
        author_collision_filters(stage, part_path)
