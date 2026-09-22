"""Compare a URDF (+ its STL meshes) against a USD asset, link by link.

For every URDF link/joint the matching USD prim is located by name and the
following is compared:
  links   : parent prim, local transform vs joint <origin>, mass, centre of
            mass, full inertia tensor (USD principalAxes/diagonalInertia is
            rebuilt as R diag R^T), visual mesh (triangle count, bbox, area,
            centroid, visual origin) vs the STL, collision shapes
  joints  : type, body0/body1, joint frame, axis direction incl. sign, limits,
            effort (drive maxForce), velocity (maxJointVelocity), damping
  extras  : loop-closure joints present only in the USD are checked by forward
            kinematics: both anchors must coincide at the zero pose
The inertia check also reports whether the *inverse* principalAxes would have
matched, which is the signature of the URDF USD Converter <= 0.1.3 bug.

Usage:
    uv run python scripts/check_urdf_vs_usd.py assets/robonionv2.usd
    uv run python scripts/check_urdf_vs_usd.py assets/robonionv2/robinion.usda --urdf path/to.urdf
"""

import argparse
import math
import struct
import xml.etree.ElementTree as ET
from collections import Counter

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation as Rot

parser = argparse.ArgumentParser()
parser.add_argument("usd")
parser.add_argument("--urdf", default="references/robinion_description/robinion2.urdf")
parser.add_argument("--meshes", default="references/robinion_description/meshes/")
args = parser.parse_args()

# Keep these values synchronized with fix_robinion_usd.py.
NEGATIVE_INERTIA_REPLACEMENT = 5.0e-4
ANKLE_COM_OFFSET = np.array([0.0245, -0.0245, 0.0])
ANKLE_LINKS = ("right_ankle_roll_pitch_link", "left_ankle_roll_pitch_link")

# URDF parsing.
def v3(s):
    return np.array([float(x) for x in s.split()])


def rpy2R(rpy):
    return Rot.from_euler("xyz", rpy).as_matrix()


root = ET.parse(args.urdf).getroot()
links, joints = {}, {}
for l in root.findall("link"):
    d = {"name": l.get("name"), "visual": [], "collision": [], "inertial": None}
    for vis in l.findall("visual"):
        o, g = vis.find("origin"), vis.find("geometry")
        mesh = g.find("mesh")
        d["visual"].append({
            "xyz": v3(o.get("xyz")) if o is not None else np.zeros(3),
            "rpy": v3(o.get("rpy")) if o is not None else np.zeros(3),
            "mesh": mesh.get("filename").split("/")[-1] if mesh is not None else None,
        })
    for col in l.findall("collision"):
        o, g = col.find("origin"), col.find("geometry")[0]
        d["collision"].append({
            "xyz": v3(o.get("xyz")) if o is not None else np.zeros(3),
            "rpy": v3(o.get("rpy")) if o is not None else np.zeros(3),
            "type": g.tag, "attr": dict(g.attrib),
        })
    ine = l.find("inertial")
    if ine is not None:
        o, I = ine.find("origin"), ine.find("inertia").attrib
        d["inertial"] = {
            "mass": float(ine.find("mass").get("value")),
            "xyz": v3(o.get("xyz")) if o is not None else np.zeros(3),
            "rpy": v3(o.get("rpy")) if o is not None else np.zeros(3),
            "I": np.array([[float(I["ixx"]), float(I["ixy"]), float(I["ixz"])],
                           [float(I["ixy"]), float(I["iyy"]), float(I["iyz"])],
                           [float(I["ixz"]), float(I["iyz"]), float(I["izz"])]]),
        }
    links[d["name"]] = d
for j in root.findall("joint"):
    o, ax, lim, dyn = j.find("origin"), j.find("axis"), j.find("limit"), j.find("dynamics")
    joints[j.get("name")] = {
        "name": j.get("name"), "type": j.get("type"),
        "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
        "xyz": v3(o.get("xyz")) if o is not None else np.zeros(3),
        "rpy": v3(o.get("rpy")) if o is not None else np.zeros(3),
        "axis": v3(ax.get("xyz")) if ax is not None else None,
        "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
        "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
        "effort": float(lim.get("effort")) if lim is not None else None,
        "velocity": float(lim.get("velocity")) if lim is not None else None,
        "damping": float(dyn.get("damping")) if dyn is not None else None,
    }
child_of = {j["child"]: j for j in joints.values()}


# STL parsing.
def read_stl(p):
    d = open(p, "rb").read()
    if d[:5] == b"solid" and b"facet" in d[:400]:
        pts = [[float(x) for x in s.split()[1:4]] for s in d.decode(errors="ignore").splitlines() if s.strip().startswith("vertex")]
        return np.array(pts).reshape(-1, 3, 3)
    n = struct.unpack("<I", d[80:84])[0]
    rec = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    return np.frombuffer(d[84:84 + n * 50], dtype=rec)["v"].reshape(-1, 3, 3).astype(np.float64)


def area_centroid(T):
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    a = np.linalg.norm(n, axis=1) / 2
    return a.sum(), (a[:, None] * T.mean(1)).sum(0) / a.sum()


# USD helpers.
stage = Usd.Stage.Open(args.usd)
print(f"USD: {args.usd}  metersPerUnit={UsdGeom.GetStageMetersPerUnit(stage)} "
      f"kgPerUnit={UsdPhysics.GetStageKilogramsPerUnit(stage)} up={UsdGeom.GetStageUpAxis(stage)}")
xc = UsdGeom.XformCache()
by_name = {}
for p in stage.Traverse(Usd.TraverseInstanceProxies()):
    by_name.setdefault(p.GetName(), []).append(p)


def A(prim, name):
    a = prim.GetAttribute(name)
    return a.Get() if a and a.HasAuthoredValue() else None


def q2R(q):
    """Gf.Quat -> rotation matrix; None if the quaternion is degenerate."""
    v = np.array([*q.GetImaginary(), q.GetReal()])
    if np.linalg.norm(v) < 1e-6:
        return None
    return Rot.from_quat(v).as_matrix()


def local_xf(prim):
    m = np.array(UsdGeom.Xformable(prim).GetLocalTransformation()).T
    R = m[:3, :3]
    return m[:3, 3], R / np.linalg.norm(R, axis=0)  # strip scale


issues = []


def issue(sev, what):
    issues.append((sev, what))
    print(f"   [{sev}] {what}")


def find_link(name):
    for cand in (name, name.replace("_link", "")):
        ps = [p for p in by_name.get(cand, []) if "/Geometry" in str(p.GetPath())]
        if ps:
            return ps[0]
    return None


# Link validation.
print("\n================ LINKS ================")
usd_mass = 0.0
for name, L in links.items():
    p = find_link(name)
    if p is None:
        issue("ERR" if L["inertial"] else "INFO", f"link {name} missing in USD" + ("" if L["inertial"] else " (massless frame)"))
        continue
    print(f"\n{name}  ->  {p.GetPath()}")
    if name in child_of:
        j = child_of[name]
        t, R = local_xf(p)
        if p.GetParent().GetName() != j["parent"]:
            issue("ERR", f"USD parent {p.GetParent().GetName()} != URDF parent {j['parent']}")
        dt, dR = np.abs(t - j["xyz"]).max(), np.abs(R - rpy2R(j["rpy"])).max()
        ok = dt < 1e-6 and dR < 1e-5
        print(f"   xform vs '{j['name']}' origin: dt={dt:.1e} dR={dR:.1e} {'OK' if ok else 'MISMATCH'}")
        if not ok:
            issue("ERR", f"{name}: local transform mismatch")
    if L["inertial"]:
        u = L["inertial"]
        m, com, di, pa = A(p, "physics:mass"), A(p, "physics:centerOfMass"), A(p, "physics:diagonalInertia"), A(p, "physics:principalAxes")
        if not p.HasAPI(UsdPhysics.RigidBodyAPI):
            issue("ERR", f"{name}: no RigidBodyAPI")
        if m is None:
            issue("ERR", f"{name}: no physics:mass")
            continue
        usd_mass += m
        print(f"   mass urdf={u['mass']:.6f} usd={m:.6f} {'OK' if abs(m - u['mass']) < 1e-6 else 'MISMATCH'}")
        if abs(m - u["mass"]) > 1e-6:
            issue("WARN", f"{name}: mass urdf {u['mass']} vs usd {m}")
        com_expected = u["xyz"]
        if name in ANKLE_LINKS:
            com_expected = u["xyz"] + ANKLE_COM_OFFSET
            issue("INFO", f"{name}: URDF inertial origin ignores the visual mesh offset, expecting CoM shifted by {ANKLE_COM_OFFSET} in USD")
        dcom = np.abs(np.array(com) - com_expected).max() if com is not None else np.inf
        print(f"   com  d={dcom:.1e} {'OK' if dcom < 1e-6 else 'MISMATCH'}")
        if dcom > 1e-6:
            issue("ERR", f"{name}: CoM mismatch")
        if di is not None and pa is not None and q2R(pa) is None:
            issue("ERR", f"{name}: principalAxes {tuple(pa.GetImaginary())}, w={pa.GetReal()} has zero norm (identity is w=1)")
        elif di is not None and pa is not None:
            Rp, D = q2R(pa), np.diag(np.array(di))
            I_usd, I_inv = Rp @ D @ Rp.T, Rp.T @ D @ Rp
            I_urdf = rpy2R(u["rpy"]) @ u["I"] @ rpy2R(u["rpy"]).T
            eig_raw = np.linalg.eigvalsh(I_urdf)
            if (eig_raw < 0).any():
                # fix_robinion_usd.py replaces negative principal moments with
                # NEGATIVE_INERTIA_REPLACEMENT (PhysX requires > 0 for articulation
                # links): compare against that instead of the raw URDF value.
                w, V = np.linalg.eigh(I_urdf)
                I_urdf = V @ np.diag(np.where(w < 0, NEGATIVE_INERTIA_REPLACEMENT, w)) @ V.T
                issue(
                    "INFO",
                    f"{name}: URDF inertia has negative principal value {np.sort(eig_raw)}, "
                    f"expecting it replaced with {NEGATIVE_INERTIA_REPLACEMENT} in USD",
                )
            scale = max(np.abs(I_urdf).max(), 1e-12)
            rel, rel_inv = np.abs(I_usd - I_urdf).max() / scale, np.abs(I_inv - I_urdf).max() / scale
            eig = np.sort(np.linalg.eigvalsh(I_urdf))
            print(f"   inertia: urdf eig={eig.round(8)} usd diag={np.sort(np.array(di)).round(8)} tensor rel diff={rel:.2%} {'OK' if rel < 0.01 else 'MISMATCH'}")
            if rel >= 0.01 and rel_inv < 0.01:
                issue("ERR", f"{name}: principalAxes is INVERTED (converter <=0.1.3 bug)")
            elif rel >= 0.01:
                issue("ERR", f"{name}: inertia tensor mismatch rel {rel:.1%}\n      urdf=\n{I_urdf}\n      usd=\n{I_usd}")
        else:
            issue("WARN", f"{name}: no diagonalInertia/principalAxes authored")
    elif p.HasAPI(UsdPhysics.RigidBodyAPI):
        issue("WARN", f"{name}: URDF massless but USD has RigidBodyAPI")

    vis_prims = [c for c in p.GetChildren() if c.GetName().endswith("_visual")]
    for v in L["visual"]:
        if v["mesh"] is None:
            continue
        stem = v["mesh"].replace(".stl", "")
        cand = [c for c in vis_prims if c.GetName() == stem]
        if not cand:
            issue("ERR", f"{name}: visual {v['mesh']} not found (usd visuals: {[c.GetName() for c in vis_prims]})")
            continue
        vp = cand[0]
        t, R = local_xf(vp)
        dt, dR = np.abs(t - v["xyz"]).max(), np.abs(R - rpy2R(v["rpy"])).max()
        meshes = [c for c in Usd.PrimRange(vp, Usd.TraverseInstanceProxies()) if c.IsA(UsdGeom.Mesh)]
        if not meshes:
            issue("ERR", f"{name}: visual {stem} has no Mesh prim")
            continue
        um = UsdGeom.Mesh(meshes[0])
        pts = np.array(um.GetPointsAttr().Get())
        fvc, fvi = np.array(um.GetFaceVertexCountsAttr().Get()), np.array(um.GetFaceVertexIndicesAttr().Get())
        mxf = np.array(xc.ComputeRelativeTransform(meshes[0], vp)[0]).T
        pts = (mxf[:3, :3] @ pts.T).T + mxf[:3, 3]
        tri, off = [], 0
        for c in fvc:
            idx = fvi[off:off + c]
            off += c
            for k in range(1, c - 1):
                tri.append([pts[idx[0]], pts[idx[k]], pts[idx[k + 1]]])
        tri = np.array(tri)
        stl = read_stl(args.meshes + v["mesh"])
        bb = np.abs(np.vstack([stl.reshape(-1, 3).min(0), stl.reshape(-1, 3).max(0)]) - np.vstack([pts.min(0), pts.max(0)])).max()
        a_s, c_s = area_centroid(stl)
        a_u, c_u = area_centroid(tri)
        ok = dt < 1e-6 and dR < 1e-5 and bb < 1e-5 and abs(a_s - a_u) / a_s < 1e-3 and np.abs(c_s - c_u).max() < 1e-5 and len(stl) == len(tri)
        print(f"   visual {v['mesh']:34s} origin dt={dt:.1e} | tris {len(stl)}/{len(tri)} | bbox d={bb:.1e} | area d={(a_s - a_u) / a_s:.1e} | centroid d={np.abs(c_s - c_u).max():.1e} {'OK' if ok else 'MISMATCH'}")
        if not ok:
            issue("ERR", f"{name}: visual mesh {v['mesh']} differs from USD")

    def owner(c):
        q = c.GetParent()
        while q and q != p:
            if q.GetName() in links or q.GetName() + "_link" in links:
                return q
            q = q.GetParent()
        return p

    cols = [c for c in Usd.PrimRange(p, Usd.TraverseInstanceProxies()) if c.HasAPI(UsdPhysics.CollisionAPI) and owner(c) == p]
    if len(cols) != len(L["collision"]):
        issue("ERR", f"{name}: collision count urdf={len(L['collision'])} usd={len(cols)}")
    for uc in L["collision"]:
        if uc["type"] == "box":
            size = v3(uc["attr"]["size"])
            hit = [c for c in cols if c.GetTypeName() == "Cube"
                   and np.allclose(np.array(A(c, "xformOp:scale")), size, atol=1e-6)
                   and np.allclose(local_xf(c)[0], uc["xyz"], atol=1e-6)
                   and np.allclose(local_xf(c)[1], rpy2R(uc["rpy"]), atol=1e-5)]
            print(f"   collision box {size} @ {uc['xyz']} {'OK' if hit else 'MISMATCH'}")
            if not hit:
                issue("ERR", f"{name}: box collision {size} @ {uc['xyz']} not matched")
        elif uc["type"] == "mesh":
            hit = [c for c in cols if c.IsA(UsdGeom.Mesh)]
            print(f"   collision mesh approximation={A(hit[0], 'physics:approximation') if hit else None} {'OK' if hit else 'MISMATCH'}")
            if not hit:
                issue("ERR", f"{name}: mesh collision not found")

print(f"\nTOTAL mass urdf={sum(l['inertial']['mass'] for l in links.values() if l['inertial']):.6f}  usd={usd_mass:.6f}")

# Joint validation.
print("\n================ JOINTS ================")
AX = {"X": np.array([1.0, 0, 0]), "Y": np.array([0, 1.0, 0]), "Z": np.array([0, 0, 1.0])}
usd_joints = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
expect = {"revolute": "PhysicsRevoluteJoint", "continuous": "PhysicsRevoluteJoint", "fixed": "PhysicsFixedJoint", "prismatic": "PhysicsPrismaticJoint"}
for name, J in joints.items():
    p = usd_joints.get(name)
    if p is None:
        issue("ERR" if J["type"] != "fixed" else "INFO", f"joint {name} ({J['type']}) missing/inactive in USD")
        continue
    b0 = [t.name for t in p.GetRelationship("physics:body0").GetTargets()]
    b1 = [t.name for t in p.GetRelationship("physics:body1").GetTargets()]
    print(f"\n{name}: {p.GetTypeName()} body0={b0} body1={b1}")
    if p.GetTypeName() != expect[J["type"]]:
        issue("ERR", f"{name}: type {p.GetTypeName()} != {expect[J['type']]}")
    if b0 and b0[0] not in (J["parent"], J["parent"].replace("_link", "")):
        issue("ERR", f"{name}: body0 {b0} != parent {J['parent']}")
    if b1 and b1[0] not in (J["child"], J["child"].replace("_link", "")):
        issue("ERR", f"{name}: body1 {b1} != child {J['child']}")
    lp0, lp1 = np.array(A(p, "physics:localPos0") or (0, 0, 0)), np.array(A(p, "physics:localPos1") or (0, 0, 0))
    lr0, lr1 = A(p, "physics:localRot0"), A(p, "physics:localRot1")
    R0, R1 = (q2R(lr0) if lr0 else np.eye(3)), (q2R(lr1) if lr1 else np.eye(3))
    Rpc = R0 @ R1.T
    tpc = lp0 - Rpc @ lp1
    dt, dR = np.abs(tpc - J["xyz"]).max(), np.abs(Rpc - rpy2R(J["rpy"])).max()
    print(f"   frame dt={dt:.1e} dR={dR:.1e} {'OK' if dt < 1e-6 and dR < 1e-5 else 'MISMATCH'}")
    if not (dt < 1e-6 and dR < 1e-5):
        issue("ERR", f"{name}: joint frame mismatch")
    if J["type"] in ("revolute", "continuous"):
        ax = R1 @ AX[A(p, "physics:axis") or "X"]
        dot = float(np.dot(ax, J["axis"]))
        print(f"   axis usd={ax.round(4)} urdf={J['axis']} dot={dot:+.4f} {'OK' if abs(dot - 1) < 1e-5 else 'MISMATCH'}")
        if abs(dot - 1) > 1e-5:
            issue("ERR", f"{name}: axis mismatch (dot {dot:+.3f})")
        lo, hi = A(p, "physics:lowerLimit"), A(p, "physics:upperLimit")
        lo_u, hi_u = math.degrees(J["lower"]), math.degrees(J["upper"])
        ok = lo is not None and hi is not None and abs(lo - lo_u) < 1e-3 and abs(hi - hi_u) < 1e-3
        print(f"   limits(deg) usd=[{lo}, {hi}] urdf=[{lo_u:.3f}, {hi_u:.3f}] {'OK' if ok else 'MISMATCH'}")
        if not ok:
            issue("ERR", f"{name}: limits mismatch")
        # Effort / velocity / damping live in different attributes depending on
        # the converter: <=0.1.3 authors a PhysX drive (maxForce, damping in
        # N*m*s/rad) + physxJoint:maxJointVelocity; 0.3.x authors
        # urdf:limit:effort + newton:velocityLimit + newton:damping (per degree)
        # and leaves the PhysX drive to the ArticulationCfg.
        effort = A(p, "drive:angular:physics:maxForce")
        effort_src = "drive:maxForce"
        if effort is None:
            effort, effort_src = A(p, "urdf:limit:effort"), "urdf:limit:effort"
        vel = A(p, "physxJoint:maxJointVelocity")
        vel_src = "physxJoint:maxJointVelocity"
        if vel is None:
            vel, vel_src = A(p, "newton:velocityLimit"), "newton:velocityLimit"
        damp = A(p, "drive:angular:physics:damping")
        damp_src = "drive:damping"
        if damp is None and A(p, "newton:damping") is not None:
            damp, damp_src = math.degrees(A(p, "newton:damping")), "newton:damping (deg->rad)"
        vel_u = math.degrees(J["velocity"])
        print(f"   effort={effort} [{effort_src}] (urdf {J['effort']}) | velocity={vel} [{vel_src}] (urdf {vel_u:.2f} deg/s) | damping={damp} [{damp_src}] (urdf {J['damping']})")
        if effort is None:
            issue("WARN", f"{name}: no effort limit authored (urdf {J['effort']})")
        elif abs(effort - J["effort"]) > 1e-4:
            issue("WARN", f"{name}: effort {effort} != urdf {J['effort']}")
        if vel is None:
            issue("WARN", f"{name}: no velocity limit authored (urdf {vel_u:.2f} deg/s)")
        elif abs(vel - vel_u) > 1e-2:
            issue("WARN", f"{name}: velocity limit {vel} != urdf {vel_u:.2f}")
        if damp is not None and J["damping"] is not None and abs(damp - J["damping"]) > 1e-4:
            issue("WARN", f"{name}: damping {damp:.4g} != urdf {J['damping']}")

# Loop-closure validation.
print("\n================ EXTRA USD JOINTS (loop closures) ================")
for n, p in usd_joints.items():
    if n in joints:
        continue
    b0 = stage.GetPrimAtPath(p.GetRelationship("physics:body0").GetTargets()[0])
    b1 = stage.GetPrimAtPath(p.GetRelationship("physics:body1").GetTargets()[0])
    W0, W1 = np.array(xc.GetLocalToWorldTransform(b0)).T, np.array(xc.GetLocalToWorldTransform(b1)).T
    a0 = W0[:3, :3] @ np.array(A(p, "physics:localPos0")) + W0[:3, 3]
    a1 = W1[:3, :3] @ np.array(A(p, "physics:localPos1")) + W1[:3, 3]
    gap = np.linalg.norm(a0 - a1)
    print(f"{n}: {b0.GetName()} <-> {b1.GetName()} excludeFromArticulation={A(p, 'physics:excludeFromArticulation')} anchor gap={gap * 1000:.3f} mm {'OK' if gap < 1e-4 else 'MISMATCH'}")
    if gap >= 1e-4:
        issue("ERR", f"{n}: loop anchors do not coincide at zero pose")

print("\n================ ARTICULATION ================")
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"root: {p.GetPath()}  schemas={p.GetAppliedSchemas()}  selfCollisions={A(p, 'physxArticulation:enabledSelfCollisions')}")
print("physics scenes:", [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)])

print("\n================ SUMMARY ================")
print(Counter(s for s, _ in issues))
for s, w in issues:
    print(f"[{s}] {w.splitlines()[0]}")
