"""Dump the physics parameters stored inside a USD robot asset.

Prints the articulation-root settings, every rigid body's mass properties,
and every joint's limits / drive / PhysX parameters, so the values baked into
the USD can be compared against the ArticulationCfg that overrides them.

Usage:
    uv run python scripts/inspect_usd.py assets/robonionv2.usd
    uv run python scripts/inspect_usd.py assets/robonionv2.usd --raw   # also dump every authored attribute
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Inspect physics parameters of a USD asset.")
parser.add_argument("usd_path", type=str, help="Path to the .usd/.usda/.usdc file.")
parser.add_argument("--raw", action="store_true", help="Also dump every authored attribute per prim.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)


def get(prim, name):
    attr = prim.GetAttribute(name)
    return attr.Get() if attr and attr.HasAuthoredValue() else None


def dump_raw(prim):
    for attr in prim.GetAttributes():
        if attr.HasAuthoredValue():
            print(f"      {attr.GetName()} = {fmt(attr.Get())}")


def main():
    stage = Usd.Stage.Open(args_cli.usd_path)
    print(f"\n=== {args_cli.usd_path} ===")
    print(f"metersPerUnit={UsdGeom.GetStageMetersPerUnit(stage)}  upAxis={UsdGeom.GetStageUpAxis(stage)}")

    bodies, joints = [], []
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"\n--- ArticulationRoot: {prim.GetPath()} ---")
            for k in (
                "physxArticulation:enabledSelfCollisions",
                "physxArticulation:solverPositionIterationCount",
                "physxArticulation:solverVelocityIterationCount",
                "physxArticulation:sleepThreshold",
                "physxArticulation:stabilizationThreshold",
            ):
                print(f"  {k} = {fmt(get(prim, k))}")
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies.append(prim)
        if prim.IsA(UsdPhysics.Joint):
            joints.append(prim)

    print(f"\n--- Rigid bodies ({len(bodies)}) ---")
    hdr = f"{'link':34s} {'mass':>9s} {'com':>28s} {'diagInertia':>32s} {'dens':>6s} collision"
    print(hdr)
    total_mass = 0.0
    for prim in bodies:
        mass = get(prim, "physics:mass")
        total_mass += mass or 0.0
        com = get(prim, "physics:centerOfMass")
        inertia = get(prim, "physics:diagonalInertia")
        dens = get(prim, "physics:density")
        n_col = sum(
            1 for c in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()) if c.HasAPI(UsdPhysics.CollisionAPI)
        )
        com_s = "-" if com is None else "(" + ", ".join(f"{x:.4f}" for x in com) + ")"
        in_s = "-" if inertia is None else "(" + ", ".join(f"{x:.3e}" for x in inertia) + ")"
        print(f"{prim.GetName():34s} {fmt(mass):>9s} {com_s:>28s} {in_s:>32s} {fmt(dens):>6s} {n_col}")
        if args_cli.raw:
            dump_raw(prim)
    print(f"{'TOTAL (authored mass only)':34s} {total_mass:9.4f}")

    print(f"\n--- Joints ({len(joints)}) ---")
    print(
        f"{'joint':34s} {'type':9s} {'lower':>8s} {'upper':>8s} {'stiff':>9s} {'damp':>9s} "
        f"{'maxF':>8s} {'fric':>6s} {'armat':>6s} {'maxVel':>7s} drive"
    )
    for prim in joints:
        jtype = prim.GetTypeName().replace("Physics", "").replace("Joint", "")
        lower = get(prim, "physics:lowerLimit")
        upper = get(prim, "physics:upperLimit")
        drive = "angular" if prim.HasAPI(UsdPhysics.DriveAPI, "angular") else (
            "linear" if prim.HasAPI(UsdPhysics.DriveAPI, "linear") else "-"
        )
        stiff = get(prim, f"drive:{drive}:physics:stiffness") if drive != "-" else None
        damp = get(prim, f"drive:{drive}:physics:damping") if drive != "-" else None
        maxf = get(prim, f"drive:{drive}:physics:maxForce") if drive != "-" else None
        dtype = get(prim, f"drive:{drive}:physics:type") if drive != "-" else None
        fric = get(prim, "physxJoint:jointFriction")
        armat = get(prim, "physxJoint:armature")
        maxvel = get(prim, "physxJoint:maxJointVelocity")
        excl = get(prim, "physics:excludeFromArticulation")
        print(
            f"{prim.GetName():34s} {jtype:9s} {fmt(lower):>8s} {fmt(upper):>8s} {fmt(stiff):>9s} {fmt(damp):>9s} "
            f"{fmt(maxf):>8s} {fmt(fric):>6s} {fmt(armat):>6s} {fmt(maxvel):>7s} {drive}/{fmt(dtype)}"
            + ("  [excludeFromArticulation]" if excl else "")
        )
        if args_cli.raw:
            dump_raw(prim)

    mats = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.MaterialAPI)]
    if mats:
        print(f"\n--- Physics materials ({len(mats)}) ---")
        for prim in mats:
            print(
                f"{str(prim.GetPath()):50s} staticFric={fmt(get(prim, 'physics:staticFriction'))} "
                f"dynFric={fmt(get(prim, 'physics:dynamicFriction'))} rest={fmt(get(prim, 'physics:restitution'))}"
            )


if __name__ == "__main__":
    main()
    simulation_app.close()
