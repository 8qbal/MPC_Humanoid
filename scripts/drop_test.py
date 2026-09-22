"""Headless drop test for assets/robonionv2.usd: spawn over a ground plane, step
physics with no drives, and report the first step at which the pelvis position
becomes non-finite or leaves a sane bounding box (i.e. the articulation explodes).

Usage:
    uv run python scripts/drop_test.py                       # default asset, 5 s
    uv run python scripts/drop_test.py --usd path.usd --seconds 8 --height 0.6
    uv run python scripts/drop_test.py --no-loop-joints      # deactivate the 4 loop joints
    uv run python scripts/drop_test.py --spherical-loops     # swap loop joints to spherical
"""

import argparse
import math
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--usd", default="assets/robonionv2.usd")
parser.add_argument("--seconds", type=float, default=5.0)
parser.add_argument("--height", type=float, default=0.6)
parser.add_argument("--dt", type=float, default=1.0 / 200.0)
parser.add_argument("--no-loop-joints", action="store_true")
parser.add_argument("--spherical-loops", action="store_true")
parser.add_argument("--pos-iters", type=int, default=None)
parser.add_argument("--vel-iters", type=int, default=None)
parser.add_argument("--print-every", type=float, default=0.25, help="seconds between status lines")
parser.add_argument("--gpu", action="store_true", help="enable GPU dynamics + GPU broadphase on the physics scene")
parser.add_argument("--no-ground", action="store_true")
parser.add_argument("--open-direct", action="store_true", help="open the usd as the stage instead of referencing it")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

ROBOT = "/robinion" if args.open_direct else "/World/Robot"
PELVIS = f"{ROBOT}/Geometry/base_link/lower_body_link"
LOOP_JOINTS = [
    f"{ROBOT}/Physics/right_back_thigh_loop_joint",
    f"{ROBOT}/Physics/right_back_shin_loop_joint",
    f"{ROBOT}/Physics/left_back_thigh_loop_joint",
    f"{ROBOT}/Physics/left_back_shin_loop_joint",
]

if args.open_direct:
    omni.usd.get_context().open_stage(args.usd)
world = World(stage_units_in_meters=1.0, physics_dt=args.dt, rendering_dt=args.dt)
if args.gpu:
    pc = world.get_physics_context()
    pc.enable_gpu_dynamics(True)
    pc.set_broadphase_type("GPU")
    print("[drop_test] GPU dynamics + GPU broadphase ENABLED")
if not args.no_ground:
    world.scene.add_default_ground_plane()
if not args.open_direct:
    add_reference_to_stage(usd_path=args.usd, prim_path=ROBOT)
stage = omni.usd.get_context().get_stage()

robot = stage.GetPrimAtPath(ROBOT)
xf = UsdGeom.Xformable(robot)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, args.height))

pelvis = stage.GetPrimAtPath(PELVIS)
assert pelvis, f"pelvis not found at {PELVIS}"

if args.no_loop_joints:
    for p in LOOP_JOINTS:
        stage.GetPrimAtPath(p).SetActive(False)
    print("[drop_test] loop joints DEACTIVATED")

if args.spherical_loops:
    # Rebuild each loop joint as a spherical joint with the same anchors.
    for p in LOOP_JOINTS:
        old = stage.GetPrimAtPath(p)
        b0 = old.GetRelationship("physics:body0").GetTargets()
        b1 = old.GetRelationship("physics:body1").GetTargets()
        lp0 = old.GetAttribute("physics:localPos0").Get()
        lp1 = old.GetAttribute("physics:localPos1").Get()
        stage.RemovePrim(p)
        j = UsdPhysics.SphericalJoint.Define(stage, p)
        j.CreateBody0Rel().SetTargets(b0)
        j.CreateBody1Rel().SetTargets(b1)
        j.CreateLocalPos0Attr(lp0)
        j.CreateLocalPos1Attr(lp1)
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
    print("[drop_test] loop joints rebuilt as SPHERICAL")

if args.pos_iters is not None:
    pelvis.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(args.pos_iters)
if args.vel_iters is not None:
    pelvis.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(args.vel_iters)

world.reset()

xc = UsdGeom.XformCache()
n_steps = int(round(args.seconds / args.dt))
print_every = max(1, int(round(args.print_every / args.dt)))
exploded_at = None
max_r = 0.0
for i in range(n_steps):
    world.step(render=False)
    xc.Clear()
    m = xc.GetLocalToWorldTransform(pelvis)
    pos = np.array(m.ExtractTranslation())
    r = float(np.linalg.norm(pos))
    max_r = max(max_r, r) if math.isfinite(r) else float("inf")
    t = (i + 1) * args.dt
    if not math.isfinite(r) or r > 50.0:
        exploded_at = t
        print(f"[drop_test] EXPLODED at t={t:.3f}s  pelvis={pos}")
        break
    if (i + 1) % print_every == 0:
        print(f"[drop_test] t={t:5.2f}s  pelvis z={pos[2]: .3f}  |pos|={r:.3f}")

if exploded_at is None:
    print(f"[drop_test] OK: no explosion in {args.seconds}s (max |pos| = {max_r:.3f})")
    # Verify that the two anchors of each loop joint still coincide.
    xc.Clear()
    for p in LOOP_JOINTS:
        j = stage.GetPrimAtPath(p)
        if not j or not j.IsActive():
            continue
        b0 = stage.GetPrimAtPath(j.GetRelationship("physics:body0").GetTargets()[0])
        b1 = stage.GetPrimAtPath(j.GetRelationship("physics:body1").GetTargets()[0])
        W0 = np.array(xc.GetLocalToWorldTransform(b0)).T
        W1 = np.array(xc.GetLocalToWorldTransform(b1)).T
        a0 = W0[:3, :3] @ np.array(j.GetAttribute("physics:localPos0").Get()) + W0[:3, 3]
        a1 = W1[:3, :3] @ np.array(j.GetAttribute("physics:localPos1").Get()) + W1[:3, 3]
        print(f"[drop_test] {j.GetName()}: anchor gap = {np.linalg.norm(a0 - a1) * 1000:.2f} mm")

app.close()
sys.exit(1 if exploded_at is not None else 0)
