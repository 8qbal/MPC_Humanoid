"""Author the hand-made physics fixes on top of a converted Robinion USD.

`urdf_usd_converter` can only reproduce what the URDF expresses: a *tree* of
links and joints. The six fixes below carry information the URDF cannot (or
gets wrong), so they are written as overrides in the root layer (robinion.usda). The Payload/
folder stays exactly as the converter produced it, which keeps
"converter output" and "our decisions" separable (diff the two files).

  1. Loop-closure joints (spherical) for the two parallelogram stages of each leg.
  2. Self-collision off on the articulation root (pelvis), plus a portable
     UsdPhysics.CollisionGroup self-filter covering every collider prim
     (including loop-jointed bodies the PhysX-attribute-only fix may miss).
  3. Floating base: deactivate the world->pelvis fixed joint.
  4. Negative principal inertia moments replaced with a physically valid
     positive value (URDF foot has izz < 0; PhysX requires izz > 0 for
     articulation links, so clamping to 0 does not work).
  5. left_back_thigh_pitch_link mass 0.078 -> 0.06 to match the right side.
  6. Ankle centre of mass shifted by the visual-mesh offset the URDF forgot to
     apply to the inertial origin (both ankle_roll_pitch links, 3.5 cm error).

The script is idempotent: every fix checks the current state first, so it can
be run on a fresh conversion or on a file already partly edited in the GUI.

Usage:
    uv run python tools/asset/fix_robinion_usd.py assets/robonionv2/robinion.usda
    uv run python tools/asset/fix_robinion_usd.py assets/robonionv2/robinion.usda --no-mass-fix
"""

import argparse

from pxr import Gf, Sdf, Usd, UsdPhysics, Vt

PELVIS = "/robinion/Geometry/base_link/lower_body_link"
PHYSICS_SCOPE = "/robinion/Physics"

# Leg geometry from the URDF. Both bars of a stage are 0.2 m long (along -Z of
# the bar frame) and the back pivot sits 0.04125 m behind (-X) the front pivot,
# on the ground link as well as on the coupler. That is what makes each stage a
# parallelogram, so the distal end of the back bar must be pinned to the coupler
# at (-0.04125, 0, 0) in the coupler frame.
BAR_LENGTH = 0.2
PIVOT_SPACING = 0.04125

# The URDF foot inertia (izz=-2.3e-5) is not just a sign error. The triangle
# inequality every rigid body's principal moments must satisfy (|Ixx-Iyy| <=
# Izz <= Ixx+Iyy, with Ixx=5.57e-4, Iyy=8.1e-5 for the foot) puts the physically
# valid range at [4.76e-4, 6.38e-4] -- nowhere near 0. A uniform-box estimate
# from the foot mesh's bounding box (0.175 x 0.067 x 0.069 m, mass 0.229 kg)
# gives Izz ~= 6.70e-4, near the top of that range, corroborating it. 5.0e-4
# is used: inside the valid range, conservative relative to the box estimate.
NEGATIVE_INERTIA_REPLACEMENT = 5.0e-4  # N*m*s^2 (kg*m^2), replaces clamp-to-0

# The two ankle_roll_pitch links are the only leg links whose visual mesh is
# placed with a non-zero <visual><origin> (0.0245, -0.0245, 0), yet their
# <inertial><origin> is the raw mesh centroid, i.e. it was never moved into the
# link frame (check_urdf_vs_usd.py / the mesh centroid coincide with the URDF
# CoM only *before* the offset, and the hand-authored collision box centre
# (-0.022, -0.0245, 0.0155) sits where CoM + offset lands). Every other leg link
# has CoM == mesh centroid in the link frame. The inertia tensor was taken about
# the mesh centroid, so it stays valid about the shifted CoM; only the point moves.
ANKLE_COM_OFFSET = Gf.Vec3f(0.0245, -0.0245, 0.0)  # m, = the URDF <visual><origin xyz>
ANKLE_LINKS = ("right_ankle_roll_pitch_link", "left_ankle_roll_pitch_link")

RIGHT = "right_hip_yaw_link/right_hip_roll_pitch_link"
LEFT = "left_hip_yaw_link/left_hip_roll_pitch_link"
LOOP_JOINTS = {
    # name: (body0 = back bar, body1 = coupler link), paths relative to the pelvis
    "right_back_thigh_loop_joint": (f"{RIGHT}/right_back_thigh_pitch_link",
                                    f"{RIGHT}/right_front_thigh_pitch_link/right_knee_pitch_link"),
    "right_back_shin_loop_joint": (f"{RIGHT}/right_front_thigh_pitch_link/right_knee_pitch_link/right_back_shin_pitch_link",
                                   f"{RIGHT}/right_front_thigh_pitch_link/right_knee_pitch_link/right_front_shin_pitch_link/right_ankle_roll_pitch_link"),
    "left_back_thigh_loop_joint": (f"{LEFT}/left_back_thigh_pitch_link",
                                   f"{LEFT}/left_front_thigh_pitch_link/left_knee_pitch_link"),
    "left_back_shin_loop_joint": (f"{LEFT}/left_front_thigh_pitch_link/left_knee_pitch_link/left_back_shin_pitch_link",
                                  f"{LEFT}/left_front_thigh_pitch_link/left_knee_pitch_link/left_front_shin_pitch_link/left_ankle_roll_pitch_link"),
}



def find_by_name(stage, name):
    hits = [p for p in stage.Traverse() if p.GetName() == name]
    assert len(hits) == 1, f"expected exactly one prim named {name}, found {len(hits)}"
    return hits[0]


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
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, -BAR_LENGTH))     # distal end of the back bar
        j.CreateLocalPos1Attr(Gf.Vec3f(-PIVOT_SPACING, 0.0, 0.0))  # back pivot on the coupler
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))                 # all leg frames are parallel at q = 0
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


def fix_negative_inertia(stage):
    # The URDF foot inertia is ixx=5.57e-4 iyy=8.1e-5 izz=-2.3e-5 and the
    # converter carries the negative moment through to both diagonalInertia and
    # newton:inertia. PhysX rejects this outright for articulation links
    # ("PxRigidBody::setMassSpaceInertiaTensor(): components must be > 0"), so
    # clamping to exactly 0 does not work either. The negative moment is
    # replaced with NEGATIVE_INERTIA_REPLACEMENT (see derivation above),
    # everything else is kept.
    # PhysX requires every principal moment to be strictly > 0 for an
    # articulation link, so both a negative moment (raw URDF) and an already
    # clamped-to-0 one (an older run of this fix) need replacing.
    fixed = 0
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        diag = prim.GetAttribute("physics:diagonalInertia")
        if not diag or not diag.HasAuthoredValue() or min(diag.Get()) > 0:
            continue
        d = diag.Get()
        diag.Set(Gf.Vec3f(*(v if v > 0 else NEGATIVE_INERTIA_REPLACEMENT for v in d)))
        full = prim.GetAttribute("newton:inertia")  # [ixx, iyy, izz, ixy, ixz, iyz]
        if full and full.HasAuthoredValue():
            f = list(full.Get())
            f[:3] = [v if v > 0 else NEGATIVE_INERTIA_REPLACEMENT for v in f[:3]]
            full.Set(Vt.DoubleArray(f))
        prim.SetCustomDataByKey(
            "inertia_source",
            f"URDF value, negative moment replaced with {NEGATIVE_INERTIA_REPLACEMENT} "
            f"(was {tuple(d)}; see NEGATIVE_INERTIA_REPLACEMENT derivation in this script)",
        )
        print(f"[4] {prim.GetName()}: diagonalInertia {tuple(d)} -> {tuple(diag.Get())}")
        fixed += 1
    if not fixed:
        print("[4] no negative principal inertia found, skipped")


def fix_mass_symmetry(stage):
    link = find_by_name(stage, "left_back_thigh_pitch_link")
    link.GetAttribute("physics:mass").Set(0.06)
    print("[5] left_back_thigh_pitch_link: mass -> 0.06 (matches right side)")


def fix_ankle_com(stage):
    for name in ANKLE_LINKS:
        link = find_by_name(stage, name)
        attr = link.GetAttribute("physics:centerOfMass")
        assert attr and attr.HasAuthoredValue(), f"{name}: converter did not author physics:centerOfMass"
        if link.GetCustomDataByKey("com_source"):
            print(f"[6] {name}: CoM already shifted, skipped")
            continue
        raw = attr.Get()
        attr.Set(raw + ANKLE_COM_OFFSET)
        link.SetCustomDataByKey(
            "com_source",
            f"URDF inertial origin {tuple(raw)} + visual mesh offset {tuple(ANKLE_COM_OFFSET)} "
            "(URDF forgot to move the mesh-centroid CoM into the link frame; see ANKLE_COM_OFFSET in this script)",
        )
        print(f"[6] {name}: centerOfMass {tuple(raw)} -> {tuple(attr.Get())}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("usd_path")
    parser.add_argument("--no-mass-fix", action="store_true", help="skip fix 5")
    args = parser.parse_args()

    stage = Usd.Stage.Open(args.usd_path)
    stage.SetEditTarget(stage.GetRootLayer())  # every edit lands in the root layer, never in Payload/

    fix_loop_joints(stage)
    fix_self_collision(stage)
    fix_floating_base(stage)
    fix_negative_inertia(stage)
    if not args.no_mass_fix:
        fix_mass_symmetry(stage)
    fix_ankle_com(stage)

    stage.GetRootLayer().Save()
    print("saved", args.usd_path)


if __name__ == "__main__":
    main()
