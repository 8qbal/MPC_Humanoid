"""Author the hand-made physics fixes on top of a converted Robinion USD.

`urdf_usd_converter` can only reproduce what the URDF expresses: a *tree* of
links and joints. The five fixes below carry information the URDF cannot, so
they are written as overrides in the root layer (robinion.usda). The Payload/
folder stays exactly as the converter produced it, which keeps
"converter output" and "our decisions" separable (diff the two files).

  1. Loop-closure joints for the two parallelogram stages of each leg.
  2. Self-collision off on the articulation root (pelvis).
  3. Floating base: deactivate the world->pelvis fixed joint.
  4. Foot inertia recomputed from the foot STL (URDF value has izz < 0).
  5. left_back_thigh_pitch_link mass 0.078 -> 0.06 to match the right side.

The script is idempotent: every fix checks the current state first, so it can
be run on a fresh conversion or on a file already partly edited in the GUI.

Usage:
    uv run python scripts/fix_robinion_usd.py assets/robonionv2/robinion.usda
    uv run python scripts/fix_robinion_usd.py assets/robonionv2/robinion.usda --no-mass-fix
"""

import argparse
import struct

import numpy as np
from pxr import Gf, Sdf, Usd, UsdPhysics
from scipy.spatial.transform import Rotation as Rot

PELVIS = "/robinion/Geometry/base_link/lower_body_link"
PHYSICS_SCOPE = "/robinion/Physics"

# Leg geometry from the URDF: both bars of a stage are 0.2 m long (along -Z of
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
    "right_back_thigh_loop_joint": (f"{RIGHT}/right_back_thigh_pitch_link",
                                    f"{RIGHT}/right_front_thigh_pitch_link/right_knee_pitch_link"),
    "right_back_shin_loop_joint": (f"{RIGHT}/right_front_thigh_pitch_link/right_knee_pitch_link/right_back_shin_pitch_link",
                                   f"{RIGHT}/right_front_thigh_pitch_link/right_knee_pitch_link/right_front_shin_pitch_link/right_ankle_roll_pitch_link"),
    "left_back_thigh_loop_joint": (f"{LEFT}/left_back_thigh_pitch_link",
                                   f"{LEFT}/left_front_thigh_pitch_link/left_knee_pitch_link"),
    "left_back_shin_loop_joint": (f"{LEFT}/left_front_thigh_pitch_link/left_knee_pitch_link/left_back_shin_pitch_link",
                                  f"{LEFT}/left_front_thigh_pitch_link/left_knee_pitch_link/left_front_shin_pitch_link/left_ankle_roll_pitch_link"),
}


def read_stl(path):
    """Binary STL -> (N, 3, 3) triangle vertices."""
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    return np.frombuffer(data[84:84 + n * 50], dtype=rec)["v"].reshape(-1, 3, 3).astype(np.float64)


def solid_inertia(tris, mass):
    """Inertia of a closed mesh treated as a uniform solid of the given mass.

    Every triangle plus the origin forms a tetrahedron; summing the signed
    volume moments of all tetrahedra gives the exact volume integrals.
    Returns (centroid, inertia tensor about the centroid, volume).
    """
    canon = np.array([[1 / 60, 1 / 120, 1 / 120], [1 / 120, 1 / 60, 1 / 120], [1 / 120, 1 / 120, 1 / 60]])
    vol, first, second = 0.0, np.zeros(3), np.zeros((3, 3))
    for a, b, c in tris:
        A = np.array([a, b, c])
        det = np.linalg.det(A)
        vol += det / 6
        first += det / 24 * (a + b + c)
        second += det * A.T @ canon @ A
    com = first / vol
    cov = (mass / vol) * second - mass * np.outer(com, com)
    return com, np.trace(cov) * np.eye(3) - cov, vol


def find_by_name(stage, name):
    hits = [p for p in stage.Traverse() if p.GetName() == name]
    assert len(hits) == 1, f"expected exactly one prim named {name}, found {len(hits)}"
    return hits[0]


def fix_loop_joints(stage):
    for name, (body0_rel, body1_rel) in LOOP_JOINTS.items():
        body0, body1 = f"{PELVIS}/{body0_rel}", f"{PELVIS}/{body1_rel}"
        for b in (body0, body1):
            assert stage.GetPrimAtPath(b), f"link not found: {b}"
        existing = [p for p in stage.Traverse() if p.GetName() == name]
        if existing:
            print(f"[1] {name}: already present at {existing[0].GetPath()}, skipped")
            continue
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{PHYSICS_SCOPE}/{name}")
        j.CreateBody0Rel().SetTargets([body0])
        j.CreateBody1Rel().SetTargets([body1])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, -BAR_LENGTH))     # distal end of the back bar
        j.CreateLocalPos1Attr(Gf.Vec3f(-PIVOT_SPACING, 0.0, 0.0))  # back pivot on the coupler
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))                 # all leg frames are parallel at q = 0
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateAxisAttr("Y")                                        # pitch axis, like every leg joint
        # PhysX articulations must be trees: a loop joint is only accepted as an
        # ordinary constraint between two articulation links.
        j.CreateExcludeFromArticulationAttr(True)
        print(f"[1] {name}: created")


def fix_self_collision(stage):
    pelvis = stage.GetPrimAtPath(PELVIS)
    assert pelvis.HasAPI(UsdPhysics.ArticulationRootAPI), "converter did not mark the pelvis as articulation root"
    pelvis.AddAppliedSchema("PhysxArticulationAPI")
    # PhysX reads the physxArticulation attribute; the Newton schema the
    # converter applies has its own. Author both so every backend agrees.
    pelvis.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    pelvis.CreateAttribute("newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)
    print("[2] lower_body_link: PhysxArticulationAPI, self-collision off")


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


def fix_foot_inertia(stage, stl_path, mass):
    # URDF: ixx=5.57e-4 iyy=8.1e-5 izz=-2.3e-5 -> a negative principal moment is
    # not a physical inertia. Recompute from the STL as a uniform solid.
    com, inertia, vol = solid_inertia(read_stl(stl_path), mass)
    w, V = np.linalg.eigh(inertia)            # inertia = V diag(w) V^T
    if np.linalg.det(V) < 0:
        V[:, 0] *= -1                          # keep V a proper rotation
    x, y, z, qw = Rot.from_matrix(V).as_quat()
    print(f"[4] foot mesh: {vol * 1e6:.1f} cm^3, implied density {mass / vol:.0f} kg/m^3, centroid {com.round(4)}")
    print(f"    principal moments {w}")
    for side in ("right", "left"):
        foot = find_by_name(stage, f"{side}_foot_roll_link")
        usd_com = np.array(foot.GetAttribute("physics:centerOfMass").Get())
        assert np.abs(usd_com - com).max() < 1e-3, f"URDF CoM {usd_com} != mesh centroid {com}: wrong STL?"
        foot.GetAttribute("physics:diagonalInertia").Set(Gf.Vec3f(*map(float, w)))
        foot.GetAttribute("physics:principalAxes").Set(Gf.Quatf(float(qw), Gf.Vec3f(float(x), float(y), float(z))))
        foot.SetCustomDataByKey("inertia_source", f"solid mesh {stl_path}, m={mass} kg")
        print(f"    {side}_foot_roll_link updated")


def fix_mass_symmetry(stage):
    link = find_by_name(stage, "left_back_thigh_pitch_link")
    link.GetAttribute("physics:mass").Set(0.06)
    print("[5] left_back_thigh_pitch_link: mass -> 0.06 (matches right side)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("usd_path")
    parser.add_argument("--foot-stl", default="references/robinion_description/meshes/right_foot_visual.stl")
    parser.add_argument("--foot-mass", type=float, default=0.229, help="URDF mass of *_foot_roll_link [kg]")
    parser.add_argument("--no-mass-fix", action="store_true", help="skip fix 5")
    args = parser.parse_args()

    stage = Usd.Stage.Open(args.usd_path)
    stage.SetEditTarget(stage.GetRootLayer())  # every edit lands in the root layer, never in Payload/

    fix_loop_joints(stage)
    fix_self_collision(stage)
    fix_floating_base(stage)
    fix_foot_inertia(stage, args.foot_stl, args.foot_mass)
    if not args.no_mass_fix:
        fix_mass_symmetry(stage)

    stage.GetRootLayer().Save()
    print("saved", args.usd_path)


if __name__ == "__main__":
    main()
