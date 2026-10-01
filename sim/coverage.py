"""
coverage.py — geometric camera-coverage certificate via PhysX raycasts.

For a camera set (A or B in cameras.json) and the proxy venue:
  * lay a 1 m grid over the floor,
  * for every cell, test a target point at TARGET_Z (torso height) — is it inside the
    camera frustum AND is the straight line from the camera to it unobstructed?
    (PhysX raycast_closest with max distance = distance to the target; a hit means blocked)
  * report per-camera coverage %, union coverage %, uncovered m², and the per-cell
    visibility count (0..N cameras), plus one RTX still per camera for the deck.

Headless. Run inside Isaac Sim python:
  /opt/IsaacSim/python.sh coverage.py --venue venue.usd --set A --out results/coverage_A.json
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

# --- Isaac Sim app must be created before any omni/pxr-runtime import ---------------
try:
    from isaacsim.simulation_app import SimulationApp  # Isaac Sim 5/6
except ImportError:  # older layout
    from isaacsim import SimulationApp  # type: ignore

ap = argparse.ArgumentParser()
ap.add_argument("--venue", required=True)
ap.add_argument("--cameras", default=os.path.join(os.path.dirname(__file__), "cameras.json"))
ap.add_argument("--set", default="A")
ap.add_argument("--out", required=True)
ap.add_argument("--grid", type=float, default=1.0, help="cell size in metres")
ap.add_argument("--target-z", type=float, default=1.0, help="test-point height above the walking surface (m)")
ap.add_argument("--target-mode", choices=["terrain", "floor"], default="terrain",
                help="terrain: test point rides the stand tiers, so seated areas are measured "
                     "where people actually are. floor: 1 m above z=0 everywhere -- which puts "
                     "the point INSIDE the solid stand blocks and counts them blind by construction.")
ap.add_argument("--stills", action="store_true", help="also render one RGB still per camera")
ap.add_argument("--still-res", default="1280x720")
args = ap.parse_args()

simulation_app = SimulationApp({"headless": True})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402


def _git_hash():
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=os.path.dirname(__file__), stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "n/a"


def _instance_type():
    try:
        import urllib.request
        tok = urllib.request.urlopen(urllib.request.Request("http://169.254.169.254/latest/api/token", method="PUT", headers={"X-aws-ec2-metadata-token-ttl-seconds": "60"}), timeout=1).read().decode()
        return urllib.request.urlopen(urllib.request.Request("http://169.254.169.254/latest/meta-data/instance-type", headers={"X-aws-ec2-metadata-token": tok}), timeout=1).read().decode()
    except Exception:
        return "unknown"


def _isaac_version():
    for p in ("/opt/IsaacSim/VERSION", os.path.join(os.path.dirname(sys.executable), "..", "VERSION")):
        if os.path.exists(p):
            return open(p).read().strip()
    try:
        import isaacsim
        return getattr(isaacsim, "__version__", "unknown")
    except Exception:
        return "unknown"


def _basis(pos, look_at):
    """Camera forward/right/up unit vectors (z-up world)."""
    f = np.array(look_at, float) - np.array(pos, float)
    f /= np.linalg.norm(f)
    up_w = np.array([0.0, 0.0, 1.0])
    r = np.cross(f, up_w)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return f, r, u


def in_frustum(f, r, u, d, hfov, vfov):
    """d = vector camera→target. True when inside the pyramid frustum."""
    z = d @ f
    if z <= 0:
        return False
    x, y = d @ r, d @ u
    return abs(x) <= z * math.tan(hfov / 2) and abs(y) <= z * math.tan(vfov / 2)


# ---------------------------------------------------------------------------- stage
ctx = omni.usd.get_context()
ctx.open_stage(args.venue)
stage = ctx.get_stage()
# The sidecar manifest is the machine-readable copy of the layout (USD
# customData cannot hold lists of dictionaries).
venue = json.load(open(os.path.splitext(args.venue)[0] + ".venue.json"))
W, H = venue["size"]["x"], venue["size"]["y"]

# PhysX scene queries need the physics scene initialised: play and step a few frames.
omni.timeline.get_timeline_interface().play()
for _ in range(10):
    simulation_app.update()

cams_cfg = json.load(open(args.cameras))
hfov = math.radians(cams_cfg["hfov_deg"])
vfov = 2 * math.atan(math.tan(hfov / 2) / cams_cfg["aspect"])
cams = cams_cfg["sets"][args.set]

nx, ny = int(round(W / args.grid)), int(round(H / args.grid))
xs = (np.arange(nx) + 0.5) * args.grid
ys = (np.arange(ny) + 0.5) * args.grid


def surface_z(x, y):
    """Height of the walking surface at (x, y): a stand tier, or the floor.

    People on a stand are on top of its tiers, not inside the block. With
    --target-mode floor the test point sits inside the solid block and every
    seating cell is counted blind by construction, which is meaningless.
    """
    if args.target_mode == "floor":
        return 0.0
    y0, depth, rise, tiers = venue["stand_y0"], venue["stand_depth"], venue["stand_rise"], venue["stand_tiers"]
    for s in venue["stands"]:
        if s["x0"] <= x < s["x1"] and y0 <= y < y0 + depth:
            tier = min(tiers - 1, int((y - y0) / depth * tiers))
            return rise * (tier + 1) / tiers
    return 0.0


def region_of(x, y):
    """concourse | stands | excluded (pillar footprints are not walkable)."""
    for p in venue["pillars"]:
        if abs(x - p["x"]) < 0.5 and abs(y - p["y"]) < 0.5:
            return "excluded"
    y0, depth = venue["stand_y0"], venue["stand_depth"]
    for s in venue["stands"]:
        if s["x0"] <= x < s["x1"] and y0 <= y < y0 + depth:
            return "stands"
    return "concourse"


region = np.array([[region_of(x, y) for x in xs] for y in ys])
measured = region != "excluded"
vis = np.zeros((ny, nx), dtype=np.int16)  # how many cameras see each cell
per_cam = []
sq = get_physx_scene_query_interface()

t0 = time.time()
for cam in cams:
    pos = np.array(cam["pos"], float)
    f, r, u = _basis(cam["pos"], cam["look_at"])
    seen = np.zeros((ny, nx), dtype=bool)
    rays = blocked = out_of_view = 0
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            if not measured[j, i]:
                continue
            tgt = np.array([x, y, surface_z(x, y) + args.target_z])
            d = tgt - pos
            if not in_frustum(f, r, u, d, hfov, vfov):
                out_of_view += 1
                continue
            dist = float(np.linalg.norm(d))
            dirn = d / dist
            rays += 1
            # Stop the ray 5 cm before the target so the surface under the target never counts.
            hit = sq.raycast_closest(carb.Float3(*pos), carb.Float3(*dirn), dist - 0.05)
            if hit["hit"]:
                blocked += 1
            else:
                seen[j, i] = True
    vis += seen
    n_meas = int(measured.sum())
    per_cam.append({
        "id": cam["id"], "pos": cam["pos"], "look_at": cam["look_at"],
        "coverage_pct": round(100 * seen.sum() / n_meas, 2),
        "concourse_pct": round(100 * seen[region == "concourse"].mean(), 2),
        "stands_pct": round(100 * seen[region == "stands"].mean(), 2),
        "cells_visible": int(seen.sum()), "cells_in_frustum": rays,
        "cells_blocked_by_geometry": blocked, "cells_out_of_view": out_of_view,
    })
    print(f"[{cam['id']}] coverage {per_cam[-1]['coverage_pct']:.1f}%  "
          f"(concourse {per_cam[-1]['concourse_pct']:.1f}%, stands {per_cam[-1]['stands_pct']:.1f}%)  "
          f"in-frustum {rays}  blocked {blocked}")

union = vis > 0
grid_out = np.where(measured, vis, -1)     # -1 marks cells that are not walkable (pillars)


def _region_stats(name):
    m = region == name
    return {"m2": float(m.sum() * args.grid ** 2),
            "union_coverage_pct": round(100 * float(union[m].mean()), 2),
            "uncovered_m2": round(float((~union & m).sum()) * args.grid ** 2, 1),
            "covered_by_2plus_pct": round(100 * float((vis[m] >= 2).mean()), 2)}


result = {
    "venue": venue["name"], "camera_set": args.set, "grid_m": args.grid,
    "target_z_m": args.target_z, "target_mode": args.target_mode,
    "hfov_deg": cams_cfg["hfov_deg"],
    "floor_m2": float(measured.sum() * args.grid ** 2), "cells": int(measured.sum()),
    "excluded_m2": float((~measured).sum() * args.grid ** 2),
    "union_coverage_pct": round(100 * float(union[measured].mean()), 2),
    "uncovered_m2": round(float((~union & measured).sum()) * args.grid ** 2, 1),
    "covered_by_2plus_pct": round(100 * float((vis[measured] >= 2).mean()), 2),
    "regions": {"concourse": _region_stats("concourse"), "stands": _region_stats("stands")},
    "per_camera": per_cam,
    "visibility_grid": grid_out.tolist(),   # rows = y (south→north), cols = x (west→east); -1 = not walkable
    "provenance": {
        "isaac_sim_version": _isaac_version(), "instance_type": _instance_type(),
        "git_hash": _git_hash(),
        "method": ("PhysX raycast_closest, camera→target at (walking surface + target_z); "
                   "target rides the stand tiers in terrain mode; blocked if any collider hit before target; "
                   "pillar footprints excluded"),
        "elapsed_s": round(time.time() - t0, 1), "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    },
}
os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
with open(args.out, "w") as fh:
    json.dump(result, fh, indent=1)
print(f"union {result['union_coverage_pct']}%  uncovered {result['uncovered_m2']} m²  -> {args.out}")

# ------------------------------------------------------------------- optional stills
if args.stills:
    try:
        import omni.replicator.core as rep
        from PIL import Image
        w, h = (int(v) for v in args.still_res.split("x"))
        out_dir = os.path.splitext(args.out)[0] + "_stills"
        os.makedirs(out_dir, exist_ok=True)
        for cam in cams:
            cam_prim = rep.create.camera(position=tuple(cam["pos"]), look_at=tuple(cam["look_at"]), focal_length=12.0)
            rp = rep.create.render_product(cam_prim, (w, h))
            ann = rep.AnnotatorRegistry.get_annotator("rgb")
            ann.attach([rp])
            for _ in range(8):           # let RTX converge a little
                rep.orchestrator.step(rt_subframes=8)
            img = ann.get_data()
            Image.fromarray(np.asarray(img)[..., :3]).save(os.path.join(out_dir, f"{cam['id']}.png"))
            ann.detach()
            rp.destroy()
        print(f"stills -> {out_dir}")
    except Exception as e:  # numbers above are already saved; stills are nice-to-have
        print(f"[warn] stills failed: {e!r}")

simulation_app.close()
