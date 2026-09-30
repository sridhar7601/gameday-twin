"""
render_views.py — RTX renders of the proxy venue for the deck.

Produces a top-down plan view plus one frame from each camera position, so the
deck can show what a given camera actually sees (and what the stand blocks hide
from it).

Kept separate from coverage.py because the measurement must not depend on the
renderer: coverage numbers come from PhysX ray-casts and are already saved
before anything is drawn.

omni.replicator.core is enabled explicitly -- in a bare SimulationApp the
extension is not started, and `rep.orchestrator` is then None, which is what
made the inline stills in coverage.py fail.

  /isaac-sim/python.sh render_views.py --venue results/venue_before.usd --out results/renders
"""
import argparse
import json
import os

try:
    from isaacsim.simulation_app import SimulationApp
except ImportError:
    from isaacsim import SimulationApp  # type: ignore

ap = argparse.ArgumentParser()
ap.add_argument("--venue", required=True)
ap.add_argument("--cameras", default=os.path.join(os.path.dirname(__file__), "cameras.json"))
ap.add_argument("--set", default="A")
ap.add_argument("--out", required=True)
ap.add_argument("--res", default="1600x900")
ap.add_argument("--subframes", type=int, default=32, help="RTX accumulation per frame")
args = ap.parse_args()

W, H = (int(v) for v in args.res.split("x"))
simulation_app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": W, "height": H})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.replicator.core")
simulation_app.update()

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image  # noqa: E402

omni.usd.get_context().open_stage(args.venue)
for _ in range(60):                     # let the stage and materials settle
    simulation_app.update()

venue = json.load(open(os.path.splitext(args.venue)[0] + ".venue.json"))
VW, VH = venue["size"]["x"], venue["size"]["y"]
cams = json.load(open(args.cameras))["sets"][args.set]

views = [("plan", (VW / 2, VH / 2 - 0.01, 70.0), (VW / 2, VH / 2, 0.0), 18.0),
         ("oblique", (-14.0, -18.0, 26.0), (VW / 2, VH / 2, 0.0), 20.0)]
views += [(c["id"], tuple(c["pos"]), tuple(c["look_at"]), 12.0) for c in cams]

os.makedirs(args.out, exist_ok=True)
written = []
for name, pos, look, flen in views:
    cam = rep.create.camera(position=pos, look_at=look, focal_length=flen)
    rp = rep.create.render_product(cam, (W, H))
    ann = rep.AnnotatorRegistry.get_annotator("rgb")
    ann.attach([rp])
    rep.orchestrator.step(rt_subframes=args.subframes)
    for _ in range(4):                  # drain so the annotator holds a full frame
        simulation_app.update()
    arr = np.asarray(ann.get_data())
    if arr.size:
        path = os.path.join(args.out, f"{name}.png")
        Image.fromarray(arr[..., :3]).save(path)
        written.append(path)
        print(f"wrote {path}")
    else:
        print(f"[warn] empty frame for {name}")
    ann.detach()
    rp.destroy()

print(f"{len(written)} renders -> {args.out}")
simulation_app.close()
