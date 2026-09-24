"""Export the corrected USD asset as a URDF for the controller's Pinocchio model.

Pinocchio cannot read USD, and the reference URDF lacks the fixes that
fix_robinion_usd.py applies to the USD (ankle CoM offset, back-thigh mass,
negative foot inertia). This script writes the kinematic tree and inertial
parameters exactly as PhysX sees them, so the controller and the simulator
share one model:

  links   : every rigid body (mass, CoM, full inertia tensor from
            diagonalInertia/principalAxes), plus imu_link and cam_link as
            massless frames. No meshes -- the controller needs no geometry.
  joints  : revolute and fixed joints from the physics joint frames
            (localPos/localRot), axis, limits, effort, velocity, damping.
  skipped : the parallelogram loop-closure joints (a URDF is a tree); the
            controller closes the loops with the linear coupling q = G q_a.

The root is lower_body_link, the articulation root in the USD. The written file
is then loaded back with Pinocchio and checked against the USD at the zero pose
(body placements, total mass, whole-body CoM).

Usage:
    uv run python tools/asset/export_controller_urdf.py assets/robonionv2.usd assets/robonionv2_controller.urdf
"""

import argparse
import math
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation as Rot

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("usd")
parser.add_argument("urdf")
args = parser.parse_args()

FRAME_LINKS = ("imu_link", "cam_link")
AXES = {"X": np.array([1.0, 0.0, 0.0]), "Y": np.array([0.0, 1.0, 0.0]), "Z": np.array([0.0, 0.0, 1.0])}

stage = Usd.Stage.Open(args.usd)
assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0 and UsdPhysics.GetStageKilogramsPerUnit(stage) == 1.0
xc = UsdGeom.XformCache()


def attr(prim, name):
    a = prim.GetAttribute(name)
    return a.Get() if a and a.HasAuthoredValue() else None


def quat_to_R(q):
    return Rot.from_quat([*q.GetImaginary(), q.GetReal()]).as_matrix()


def world(prim):
    m = np.array(xc.GetLocalToWorldTransform(prim)).T
    R = m[:3, :3]
    return m[:3, 3], R / np.linalg.norm(R, axis=0)


def fmt(v):
    return " ".join(f"{x:.9g}" for x in v)


roots = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
assert len(roots) == 1, roots
root = roots[0]
bodies = {p.GetName(): p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
all_joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
tree_joints = [j for j in all_joints if not attr(j, "physics:excludeFromArticulation")]
skipped = [j.GetName() for j in all_joints if attr(j, "physics:excludeFromArticulation")]

robot = ET.Element("robot", name=stage.GetDefaultPrim().GetName())

for name, body in bodies.items():
    link = ET.SubElement(robot, "link", name=name)
    mass = attr(body, "physics:mass")
    com = np.array(attr(body, "physics:centerOfMass"))
    I = quat_to_R(attr(body, "physics:principalAxes")) @ np.diag(attr(body, "physics:diagonalInertia"))
    I = I @ quat_to_R(attr(body, "physics:principalAxes")).T
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz=fmt(com), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.9g}")
    ET.SubElement(
        inertial, "inertia",
        ixx=f"{I[0, 0]:.9g}", ixy=f"{I[0, 1]:.9g}", ixz=f"{I[0, 2]:.9g}",
        iyy=f"{I[1, 1]:.9g}", iyz=f"{I[1, 2]:.9g}", izz=f"{I[2, 2]:.9g}",
    )

for j in tree_joints:
    body0 = stage.GetPrimAtPath(j.GetRelationship("physics:body0").GetTargets()[0])
    body1 = stage.GetPrimAtPath(j.GetRelationship("physics:body1").GetTargets()[0])
    R0, R1 = quat_to_R(attr(j, "physics:localRot0")), quat_to_R(attr(j, "physics:localRot1"))
    p0, p1 = np.array(attr(j, "physics:localPos0")), np.array(attr(j, "physics:localPos1"))
    # URDF puts the child frame on the joint; the USD child body frame sits at (p1, R1) in joint space.
    R = R0 @ R1.T
    t = p0 - R @ p1
    kind = {"PhysicsRevoluteJoint": "revolute", "PhysicsFixedJoint": "fixed"}[j.GetTypeName()]
    joint = ET.SubElement(robot, "joint", name=j.GetName(), type=kind)
    ET.SubElement(joint, "origin", xyz=fmt(t), rpy=fmt(Rot.from_matrix(R).as_euler("xyz")))
    ET.SubElement(joint, "parent", link=body0.GetName())
    ET.SubElement(joint, "child", link=body1.GetName())
    if kind == "revolute":
        ET.SubElement(joint, "axis", xyz=fmt(R1 @ AXES[attr(j, "physics:axis")]))
        effort = attr(j, "drive:angular:physics:maxForce") or attr(j, "urdf:limit:effort")
        velocity = math.radians(attr(j, "physxJoint:maxJointVelocity") or attr(j, "newton:velocityLimit"))
        ET.SubElement(
            joint, "limit",
            lower=f"{math.radians(attr(j, 'physics:lowerLimit')):.9g}",
            upper=f"{math.radians(attr(j, 'physics:upperLimit')):.9g}",
            effort=f"{effort:.9g}", velocity=f"{velocity:.9g}",
        )
        damping = attr(j, "newton:damping")
        if damping is not None:
            ET.SubElement(joint, "dynamics", damping=f"{math.degrees(damping):.9g}", friction="0")

for name in FRAME_LINKS:
    prim = next(p for p in stage.Traverse() if p.GetName() == name)
    parent = prim.GetParent()
    m = np.array(xc.ComputeRelativeTransform(prim, parent)[0]).T
    R = m[:3, :3] / np.linalg.norm(m[:3, :3], axis=0)
    ET.SubElement(robot, "link", name=name)
    joint = ET.SubElement(robot, "joint", name=f"{name}_fixed_joint", type="fixed")
    ET.SubElement(joint, "origin", xyz=fmt(m[:3, 3]), rpy=fmt(Rot.from_matrix(R).as_euler("xyz")))
    ET.SubElement(joint, "parent", link=parent.GetName())
    ET.SubElement(joint, "child", link=name)

ET.indent(robot)
comment = ET.Comment(f" Generated from {args.usd} by tools/asset/export_controller_urdf.py -- do not edit ")
with open(args.urdf, "w") as f:
    f.write('<?xml version="1.0"?>\n' + ET.tostring(comment, encoding="unicode") + "\n")
    f.write(ET.tostring(robot, encoding="unicode") + "\n")
print(f"wrote {args.urdf}: {len(bodies)} bodies, {len(tree_joints)} joints, frames {FRAME_LINKS}")
print(f"skipped loop closures: {skipped}")

# Round-trip check against the USD at the zero pose.
model = pin.buildModelFromUrdf(args.urdf, pin.JointModelFreeFlyer())
data = model.createData()
t_root, R_root = world(root)
q = pin.neutral(model)
q[:3] = t_root
q[3:7] = Rot.from_matrix(R_root).as_quat()
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

worst = 0.0
for name, body in bodies.items():
    t, R = world(body)
    M = data.oMf[model.getFrameId(name)]
    worst = max(worst, np.abs(M.translation - t).max(), np.abs(M.rotation - R).max())

usd_mass = sum(attr(b, "physics:mass") for b in bodies.values())
usd_com = sum(
    attr(b, "physics:mass") * (world(b)[1] @ np.array(attr(b, "physics:centerOfMass")) + world(b)[0])
    for b in bodies.values()
) / usd_mass
pin_com = pin.centerOfMass(model, data, q)
revolute = sum(1 for j in tree_joints if j.GetTypeName() == "PhysicsRevoluteJoint")

print(f"pinocchio: nq={model.nq} nv={model.nv} (free-flyer 7/6 + {revolute} revolute)")
print(f"body placement max diff at zero pose: {worst:.2e}")
print(f"total mass usd={usd_mass:.6f} pinocchio={sum(i.mass for i in model.inertias):.6f}")
print(f"CoM usd={usd_com.round(6)} pinocchio={pin_com.round(6)} diff={np.abs(usd_com - pin_com).max():.2e}")
ok = model.nv == 6 + revolute and worst < 1e-6 and np.abs(usd_com - pin_com).max() < 1e-6
print("OK" if ok else "MISMATCH")
