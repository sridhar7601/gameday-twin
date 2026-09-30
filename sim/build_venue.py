"""
build_venue.py — procedural "stadium-section proxy" venue as OpenUSD.

This is NOT a real venue. It is a deliberately simple, fully-disclosed proxy:
a 60 m x 40 m concourse, three stand blocks along the north edge, a barrier row
with gates separating the stands from the south concourse, four structural
pillars, three exits in the south wall, and one extra exit (E4) that is closed
by default and opened in the "after" scenario.

Every solid gets a PhysX collider so PhysX raycasts (coverage.py) and agent
navigation (surge) see the same geometry.

Runs inside Isaac Sim's python:  /opt/IsaacSim/python.sh build_venue.py --out venue.usd
"""
import argparse
import json
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from venue_def import VENUE, resolve, segments as _wall_segments  # noqa: E402


def _box(stage, path, center, size, color=(0.6, 0.6, 0.6), collide=True):
    """Axis-aligned box as a UsdGeom.Cube scaled to `size`, centred at `center`."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def build(stage, venue, open_gate_ids=(), open_exit_ids=()):
    """Author the venue under /World/Venue. Returns the resolved venue dict."""
    v = resolve(open_gate_ids, open_exit_ids)

    W, H = v["size"]["x"], v["size"]["y"]
    wh, wt = v["wall_height"], v["wall_thickness"]

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    root = UsdGeom.Xform.Define(stage, "/World/Venue")
    # customData cannot hold lists of dictionaries, so the manifest travels as
    # a JSON string; the .venue.json sidecar remains the machine-readable copy.
    root.GetPrim().SetCustomDataByKey("venue_json", json.dumps(v))
    root.GetPrim().SetCustomDataByKey("venue_name", v["name"])

    # Floor
    _box(stage, "/World/Venue/Floor", (W / 2, H / 2, -0.05), (W, H, 0.1), (0.35, 0.35, 0.38))

    # Perimeter walls. South wall carries the exits.
    south = _wall_segments(0.0, W, list(v["exits"].values()))
    for i, (x0, x1) in enumerate(south):
        _box(stage, f"/World/Venue/Wall_S{i}", ((x0 + x1) / 2, wt / 2, wh / 2), (x1 - x0, wt, wh), (0.5, 0.5, 0.55))
    _box(stage, "/World/Venue/Wall_N", (W / 2, H - wt / 2, wh / 2), (W, wt, wh), (0.5, 0.5, 0.55))
    _box(stage, "/World/Venue/Wall_W", (wt / 2, H / 2, wh / 2), (wt, H, wh), (0.5, 0.5, 0.55))
    _box(stage, "/World/Venue/Wall_E", (W - wt / 2, H / 2, wh / 2), (wt, H, wh), (0.5, 0.5, 0.55))

    # Barrier row with gates
    b = v["barrier"]
    for i, (x0, x1) in enumerate(_wall_segments(0.0, W, list(v["gates"].values()))):
        _box(stage, f"/World/Venue/Barrier_{i}", ((x0 + x1) / 2, b["y"], b["height"] / 2), (x1 - x0, b["thickness"], b["height"]), (0.8, 0.55, 0.1))

    # Stands (tiered solid blocks)
    tiers, y0, depth, rise = v["stand_tiers"], v["stand_y0"], v["stand_depth"], v["stand_rise"]
    for s in v["stands"]:
        for t in range(tiers):
            ty0 = y0 + depth * t / tiers
            ty1 = y0 + depth * (t + 1) / tiers
            h = rise * (t + 1) / tiers
            _box(stage, f"/World/Venue/Stand_{s['id']}_T{t}", ((s["x0"] + s["x1"]) / 2, (ty0 + ty1) / 2, h / 2), (s["x1"] - s["x0"], ty1 - ty0, h), (0.25, 0.3, 0.45))

    # Pillars
    for p in v["pillars"]:
        _box(stage, f"/World/Venue/Pillar_{p['id']}", (p["x"], p["y"], wh / 2), (1.0, 1.0, wh), (0.6, 0.6, 0.6))

    # Named points used by the surge scripts (not rendered): spawn rows and exit targets.
    pts = UsdGeom.Scope.Define(stage, "/World/Venue/Points")
    for s in v["stands"]:
        pr = UsdGeom.Xform.Define(stage, f"/World/Venue/Points/Spawn_{s['id']}")
        pr.AddTranslateOp().Set(Gf.Vec3d((s["x0"] + s["x1"]) / 2, y0 - 1.5, 0.0))
        pr.GetPrim().SetCustomDataByKey("half_width", (s["x1"] - s["x0"]) / 2 - 0.5)
    for eid, e in v["exits"].items():
        pr = UsdGeom.Xform.Define(stage, f"/World/Venue/Points/Exit_{eid}")
        pr.AddTranslateOp().Set(Gf.Vec3d((e["x0"] + e["x1"]) / 2, -1.0, 0.0))
        pr.GetPrim().SetCustomDataByKey("open", bool(e["open"]))
    for gid, g in v["gates"].items():
        pr = UsdGeom.Xform.Define(stage, f"/World/Venue/Points/Gate_{gid}")
        pr.AddTranslateOp().Set(Gf.Vec3d((g["x0"] + g["x1"]) / 2, b["y"], 0.0))
        pr.GetPrim().SetCustomDataByKey("open", bool(g["open"]))

    # A neutral dome light so RTX stills are not black.
    from pxr import UsdLux
    light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    light.CreateIntensityAttr(1000.0)

    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="venue.usd")
    ap.add_argument("--open-gates", default="", help="comma list, e.g. G3 (the 'after' fix)")
    ap.add_argument("--open-exits", default="", help="comma list of extra exits to open")
    args = ap.parse_args()

    stage = Usd.Stage.CreateNew(args.out)
    v = build(
        stage,
        VENUE,
        open_gate_ids=[g for g in args.open_gates.split(",") if g],
        open_exit_ids=[e for e in args.open_exits.split(",") if e],
    )
    stage.GetRootLayer().Save()
    manifest = os.path.splitext(args.out)[0] + ".venue.json"
    with open(manifest, "w") as f:
        json.dump(v, f, indent=2)
    print(f"wrote {args.out} and {manifest}")
    print("open gates:", [g for g, d in v["gates"].items() if d["open"]],
          "open exits:", [e for e, d in v["exits"].items() if d["open"]])


if __name__ == "__main__":
    main()
