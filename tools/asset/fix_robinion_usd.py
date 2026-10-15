"""Author the hand-made physics fixes on top of a converted Robinion USD.

`urdf_usd_converter` can only reproduce what the URDF expresses: a *tree* of
links and joints. The three fixes below carry information the URDF cannot,
so they are written as overrides in the root layer (robinion.usda). The Payload/
folder stays exactly as the converter produced it, which keeps
"converter output" and "our decisions" separable (diff the two files).

  1. Loop-closure joints (spherical) for the two parallelogram stages of each leg.
  2. Self-collision off on the articulation root (pelvis), plus a portable
     UsdPhysics.CollisionGroup self-filter covering every collider prim
     (including loop-jointed bodies the PhysX-attribute-only fix may miss).
  3. Floating base: deactivate the world->pelvis fixed joint.

Inertials and limits are taken from the URDF as they are (upstream 55ac2c2 fixed
the foot inertia, the back-thigh mass and the ankle CoM that earlier versions of
this script patched).

The script is idempotent: every fix checks the current state first, so it can
be run on a fresh conversion or on a file already partly edited in the GUI.

Usage:
    uv run python tools/asset/fix_robinion_usd.py assets/robonionv2/robinion.usda
"""

import argparse

from pxr import Gf, Sdf, Usd, UsdPhysics

PELVIS = "/robinion/Geometry/base_link/lower_body_link"
PHYSICS_SCOPE = "/robinion/Physics"

# Leg geometry from the URDF. Both bars of a stage are 0.2 m long (along -Z of
# the bar frame) and the back pivot sits 0.04125 m behind (-X) the front pivot,
# on the ground link as well as on the coupler. That is what makes each stage a
# parallelogram, so the distal end of the back bar must be pinned to the coupler
# at (-0.04125, 0, 0) in the coupler frame.
BAR_LENGTH = 0.2
PIVOT_SPACING = 0.04125

RIGHT = "right_hip_yaw_link/right_hip_roll_pitch_link"
LEFT = "left_hip_yaw_link/left_hip_roll_pitch_link"
LOOP_JOINTS = {
    # name: (body0 = back bar, body1 = coupler link), paths relative to the pelvis
    "right_back_thigh_loop_joint": (
        f"{RIGHT}/right_back_thigh_pitch_link",
        f"{RIGHT}/right_front_thigh_pitch_link/right_knee_pitch_link",
    ),
    "right_back_shin_loop_joint": (
        f"{RIGHT}/right_front_thigh_pitch_link/right_knee_pitch_link/right_back_shin_pitch_link",
        f"{RIGHT}/right_front_thigh_pitch_link/right_knee_pitch_link/right_front_shin_pitch_link/right_ankle_roll_pitch_link",
    ),
    "left_back_thigh_loop_joint": (
        f"{LEFT}/left_back_thigh_pitch_link",
        f"{LEFT}/left_front_thigh_pitch_link/left_knee_pitch_link",
    ),
    "left_back_shin_loop_joint": (
        f"{LEFT}/left_front_thigh_pitch_link/left_knee_pitch_link/left_back_shin_pitch_link",
        f"{LEFT}/left_front_thigh_pitch_link/left_knee_pitch_link/left_front_shin_pitch_link/left_ankle_roll_pitch_link",
    ),
}


def fix_loop_joints(stage):
    # The closure is spherical, not revolute. The three tree joints
    # of each parallelogram stage are already parallel revolutes, so the closure
    # only has to pin one point: a spherical joint adds the 3 positional
    # constraints that does, whereas a revolute would add 5 and leave the planar
    # 4-bar over-constrained by 3 (Gruebler in 3D: 6*3 - 4*5 = -2). PhysX can
    # tolerate that redundancy on some builds but not others: with omni.physx
    # 110.1.x (Isaac Sim 6.0.1, the GUI install) a revolute closure makes the
    # whole articulation explode to ~1e14 m on first ground contact, while the
    # spherical closure is stable on both 110.1.x and 110.3.x (see
    # a standalone drop test, since removed). NVIDIA's own loop-closure examples use spherical
    # or D6 joints for the same reason.
    for name, (body0_rel, body1_rel) in LOOP_JOINTS.items():
        body0, body1 = f"{PELVIS}/{body0_rel}", f"{PELVIS}/{body1_rel}"
        for b in (body0, body1):
            assert stage.GetPrimAtPath(b), f"link not found: {b}"
        path = f"{PHYSICS_SCOPE}/{name}"
        existing = stage.GetPrimAtPath(path)
        if existing and existing.IsA(UsdPhysics.SphericalJoint):
            print(f"[1] {name}: already present, skipped")
            continue
        if existing:
            # An older run authored this closure as a revolute joint.
            old_type = existing.GetTypeName()
            stage.RemovePrim(path)
            print(f"[1] {name}: replacing {old_type} with SphericalJoint")
        j = UsdPhysics.SphericalJoint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([body0])
        j.CreateBody1Rel().SetTargets([body1])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, -BAR_LENGTH))  # distal end of the back bar
        j.CreateLocalPos1Attr(Gf.Vec3f(-PIVOT_SPACING, 0.0, 0.0))  # back pivot on the coupler
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))  # all leg frames are parallel at q = 0
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        # PhysX articulations must be trees; a loop joint is accepted only as an
        # ordinary constraint between two articulation links.
        j.CreateExcludeFromArticulationAttr(True)
        print(f"[1] {name}: created")


def fix_self_collision(stage):
    pelvis = stage.GetPrimAtPath(PELVIS)
    assert pelvis.HasAPI(UsdPhysics.ArticulationRootAPI), "converter did not mark the pelvis as articulation root"
    pelvis.AddAppliedSchema("PhysxArticulationAPI")
    # PhysX reads the physxArticulation attribute; the Newton schema the
    # converter applies has its own. Author both so every backend agrees.
    # These are authored as plain custom attributes (not through the real
    # PhysxSchema.PhysxArticulationAPI class) because that class only exists
    # inside the Kit/Isaac Sim runtime, not in this project's standalone pxr
    # package -- so there is no way from here to confirm PhysX actually reads
    # them, or that they cover bodies linked in only via a loop-closure joint
    # (excludeFromArticulation) rather than the reduced-coordinate tree.
    pelvis.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    pelvis.CreateAttribute("newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)
    print("[2] lower_body_link: PhysxArticulationAPI, self-collision off")

    # Also disable self-collision through UsdPhysics.CollisionGroup.
    # a vanilla (non-PhysX-specific) schema that any physics backend must honor and that
    # this script can construct and verify without the Kit runtime. A group that lists
    # itself in filteredGroups (invertFilteredGroups=False, the default) does not collide
    # with its own members -- the standard USD idiom for "self-collision off". Every prim
    # with UsdPhysics.CollisionAPI is added, including bodies only reachable via a
    # loop-closure joint, which the two attribute-only fixes above cannot guarantee cover.
    group = UsdPhysics.CollisionGroup.Define(stage, f"{PHYSICS_SCOPE}/RobinionSelfCollisionGroup")
    group.CreateInvertFilteredGroupsAttr(False)
    group.CreateFilteredGroupsRel().SetTargets([group.GetPath()])
    members = [p.GetPath() for p in Usd.PrimRange(stage.GetPrimAtPath(PELVIS)) if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert members, "no prims with UsdPhysics.CollisionAPI found under the pelvis"
    collection = group.GetCollidersCollectionAPI()
    collection.CreateIncludesRel().SetTargets(members)
    print(f"[2] RobinionSelfCollisionGroup: {len(members)} collider prims, self-filtered")


def fix_floating_base(stage):
    # The converter turns `base_link -> lower_body_link` into a fixed joint whose
    # body0 is the asset root, i.e. the pelvis is welded to the world. A walking
    # robot needs a free pelvis, so the joint is deactivated (not deleted: it is
    # defined in the Payload layer, an override can only switch it off).
    joint = stage.GetPrimAtPath(f"{PHYSICS_SCOPE}/base_to_lower_body_fixed_joint")
    if not joint or not joint.IsActive():
        print("[3] base_to_lower_body_fixed_joint: already inactive/absent, skipped")
        return
    joint.SetActive(False)
    print("[3] base_to_lower_body_fixed_joint: deactivated (floating base)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("usd_path")
    args = parser.parse_args()

    stage = Usd.Stage.Open(args.usd_path)
    stage.SetEditTarget(stage.GetRootLayer())  # every edit lands in the root layer, never in Payload/

    fix_loop_joints(stage)
    fix_self_collision(stage)
    fix_floating_base(stage)

    stage.GetRootLayer().Save()
    print("saved", args.usd_path)


if __name__ == "__main__":
    main()
