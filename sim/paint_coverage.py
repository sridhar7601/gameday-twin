"""
paint_coverage.py — bake a measured coverage result onto the venue in 3D.

Takes a coverage_*.json and the venue it was measured on, and writes a new USD
where the floor is tiled with the result: red for floor no camera can see,
blue shading for 1/2/3 cameras. Camera positions are dropped in as markers with
a post down to the floor.

The point is that the thing you look at in the viewer IS the finding — the blind
area is visible as red floor, not just a number in a report.

Pure USD authoring: no Kit, no GPU, no renderer. Run it with usdpy.sh so it does
not fight the streaming GUI for the cache lock.

  bash sim/usdpy.sh sim/paint_coverage.py \
      --venue results/venue_before.usd \
      --coverage results/coverage_A.json \
      --out results/venue_coverage_A.usd
"""
import argparse
import json
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ap = argparse.ArgumentParser()
ap.add_argument("--venue", required=True, help="venue .usd to build on")
ap.add_argument("--coverage", required=True, help="coverage_*.json from coverage.py")
ap.add_argument("--out", required=True)
ap.add_argument("--tile-z", type=float, default=0.02, help="height above the floor (m)")
args = ap.parse_args()

cov = json.load(open(args.coverage))
grid = cov["visibility_grid"]           # rows = y, cols = x
cell = cov["grid_m"]
ny, nx = len(grid), len(grid[0])

# Reference the existing venue so geometry stays in one place and any rebuild of
# the venue flows through to this overlay.
stage = Usd.Stage.CreateNew(args.out)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())
venue_ref = stage.OverridePrim("/World/Venue")
venue_path = os.path.abspath(args.venue)
venue_ref.GetReferences().AddReference(venue_path, "/World/Venue")

# Colours mirror the report: red = unseen, then light->dark blue for 1/2/3.
PALETTE = {
    0: (0.816, 0.231, 0.231),
    1: (0.525, 0.714, 0.937),
    2: (0.165, 0.470, 0.839),
    3: (0.063, 0.259, 0.506),
}

# One PointInstancer per coverage level: 2,400 tiles as individual prims would
# crawl in the viewport, whereas instanced quads stay interactive.
tiles = UsdGeom.Scope.Define(stage, "/World/Coverage")
counts = {}
for level, colour in PALETTE.items():
    pos = [Gf.Vec3f((i + 0.5) * cell, (j + 0.5) * cell, args.tile_z)
           for j in range(ny) for i in range(nx) if grid[j][i] == level]
    counts[level] = len(pos)
    if not pos:
        continue
    proto_scope = UsdGeom.Scope.Define(stage, f"/World/Coverage/L{level}_proto")
    quad = UsdGeom.Mesh.Define(stage, f"/World/Coverage/L{level}_proto/tile")
    h = cell * 0.47                       # small gap so the grid stays readable
    quad.CreatePointsAttr([(-h, -h, 0), (h, -h, 0), (h, h, 0), (-h, h, 0)])
    quad.CreateFaceVertexCountsAttr([4])
    quad.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    quad.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
    quad.CreateDisplayOpacityAttr([0.55 if level else 0.85])
    quad.CreateDoubleSidedAttr(True)

    inst = UsdGeom.PointInstancer.Define(stage, f"/World/Coverage/L{level}")
    inst.CreatePositionsAttr(pos)
    inst.CreateProtoIndicesAttr([0] * len(pos))
    inst.CreatePrototypesRel().SetTargets([quad.GetPath()])

# Camera markers: a sphere at the mount point and a post to the floor.
cams = UsdGeom.Scope.Define(stage, "/World/Cameras")
for c in cov["per_camera"]:
    x, y, z = c["pos"]
    weak = c["coverage_pct"] < 20
    col = (0.816, 0.231, 0.231) if weak else (0.10, 0.10, 0.10)
    ball = UsdGeom.Sphere.Define(stage, f"/World/Cameras/{c['id']}")
    ball.CreateRadiusAttr(0.45)
    UsdGeom.Xformable(ball).AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    ball.CreateDisplayColorAttr([Gf.Vec3f(*col)])
    post = UsdGeom.Cylinder.Define(stage, f"/World/Cameras/{c['id']}_post")
    post.CreateRadiusAttr(0.06)
    post.CreateHeightAttr(z)
    post.CreateAxisAttr("Z")
    UsdGeom.Xformable(post).AddTranslateOp().Set(Gf.Vec3d(x, y, z / 2))
    post.CreateDisplayColorAttr([Gf.Vec3f(*col)])

light = UsdLux.DomeLight.Define(stage, "/World/CoverageLight")
light.CreateIntensityAttr(900.0)

stage.GetRootLayer().Save()
total = sum(counts.values())
print(f"wrote {args.out}")
print(f"  tiles: " + ", ".join(f"{k} cam={v}" for k, v in sorted(counts.items())) + f"  (total {total})")
print(f"  unseen floor painted red: {counts.get(0,0) * cell * cell:.0f} m2")
print(f"  cameras: {[c['id'] + ' ' + str(c['coverage_pct']) + '%' for c in cov['per_camera']]}")
