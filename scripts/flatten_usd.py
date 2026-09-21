"""Flatten a layered USD asset into one standalone .usd file.

The converter writes an "atomic component": a thin robinion.usda that payloads
Payload/Contents.usda, which in turn sublayers Geometry/Physics and references
GeometryLibrary.usdc. That is fine for editing, but the asset then only works
while the whole folder stays in place. Flattening resolves every
payload/reference/sublayer into a single layer with the meshes embedded, so the
result can be copied anywhere on its own.

Only the default prim (the robot) is kept. Everything else at the root is
session clutter that Isaac Sim / the converter leave behind: the converter's
PhysicsScene (Isaac Lab creates its own), viewport cameras (OmniverseKit_*),
render settings, and any environment prim (e.g. a grid floor) that happened to
be loaded in the GUI when the file was saved.

Usage:
    uv run python scripts/flatten_usd.py assets/robonionv2/robinion.usda assets/robonionv2.usd
"""

import argparse
import os

from pxr import Usd

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("src")
parser.add_argument("dst")
args = parser.parse_args()

stage = Usd.Stage.Open(args.src)
default_prim = stage.GetRootLayer().defaultPrim
assert default_prim, f"{args.src} has no defaultPrim"

flat = stage.Flatten()  # one Sdf.Layer with everything composed
for name in [p.name for p in flat.rootPrims if p.name != default_prim]:
    del flat.pseudoRoot.nameChildren[name]
    print("dropped /" + name)

flat.defaultPrim = default_prim
flat.customLayerData = {
    "creator": stage.GetRootLayer().customLayerData.get("creator", ""),
    "flattened_from": os.path.relpath(args.src),
}
flat.Export(args.dst)
print(f"wrote {args.dst} ({os.path.getsize(args.dst) / 1e6:.1f} MB)")

check = Usd.Stage.Open(args.dst)
print("root prims :", [p.GetName() for p in check.GetPseudoRoot().GetChildren()])
print("layers used:", [os.path.relpath(l.realPath) for l in check.GetUsedLayers() if l.realPath])
