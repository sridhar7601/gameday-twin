"""
density.py — surge scenario: agents leave the stands and head for the exits;
we measure crowd density per 1 m² cell over time, find the hottest cell, and
report time-to-clear. Run once with the "before" venue (gate G3 closed) and once
with the "after" venue (G3 open) using the SAME seed.

Agent model (disclosed on every result screen as "simplified agent model"):
  * goal-seeking along an A* path on a 0.5 m occupancy grid built from the venue,
  * separation force from neighbours within 0.8 m,
  * wall/pillar repulsion, speed capped at 1.3 m/s, dt = 0.1 s.
  It is NOT a validated pedestrian model. It is enough to show where a layout
  concentrates people and how a layout change shifts that concentration.

Agents are also written into the USD stage as capsules so RTX stills show the
crowd. When run inside Isaac Sim's python the scene is opened headless and a
top-down still is rendered at the peak-density moment; when run with plain
python (no Isaac Sim) it still produces the numbers.

  /opt/IsaacSim/python.sh density.py --venue results/venue_before.usd --out results/surge_before.json --seed 42
"""
import argparse
import heapq
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from venue_def import segments as _segments  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--venue", required=True, help="venue .usd (its .venue.json sidecar is read)")
ap.add_argument("--out", required=True)
ap.add_argument("--seed", type=int, default=42)
ap.add_argument("--agents-per-stand", type=int, default=400)
ap.add_argument("--dt", type=float, default=0.1)
ap.add_argument("--max-t", type=float, default=600.0)
ap.add_argument("--grid", type=float, default=1.0, help="density cell size (m)")
ap.add_argument("--threshold", type=float, default=4.0, help="persons/m² flagged as over-threshold (source to be cited on slide)")
ap.add_argument("--sample-every", type=float, default=1.0, help="seconds between density samples")
ap.add_argument("--no-isaac", action="store_true", help="numbers only, skip Isaac Sim / stills")
ap.add_argument("--body-radius", type=float, default=0.20, help="agent body radius (m); sets the packing limit")
ap.add_argument("--relax-iters", type=int, default=4, help="position-based non-overlap relaxation passes per step")
args = ap.parse_args()

rng = np.random.default_rng(args.seed)
_mf = args.venue if args.venue.endswith(".venue.json") else os.path.splitext(args.venue)[0] + ".venue.json"
venue = json.load(open(_mf))
W, H = venue["size"]["x"], venue["size"]["y"]

# ------------------------------------------------------------------ obstacles
# Rectangles (x0,y0,x1,y1) that agents may not enter. Same construction as build_venue.py.
wt, b = venue["wall_thickness"], venue["barrier"]
rects = []
for x0, x1 in _segments(0, W, venue["exits"].values()):
    rects.append((x0, 0.0, x1, wt))                              # south wall (with exits)
rects += [(0, H - wt, W, H), (0, 0, wt, H), (W - wt, 0, W, H)]   # N, W, E walls
for x0, x1 in _segments(0, W, venue["gates"].values()):
    rects.append((x0, b["y"] - b["thickness"] / 2, x1, b["y"] + b["thickness"] / 2))
# NOTE: the stand blocks are seating terraces, i.e. where the crowd starts and
# walks down from. They are solid geometry for the cameras (coverage.py) but
# walkable for the crowd, so they are deliberately not added as obstacles here.
for p in venue["pillars"]:
    rects.append((p["x"] - 0.5, p["y"] - 0.5, p["x"] + 0.5, p["y"] + 0.5))

# ------------------------------------------------------------------ routing grid (0.5 m)
RES = 0.5
gx, gy = int(W / RES), int(H / RES)
blocked = np.zeros((gy, gx), dtype=bool)
for (x0, y0, x1, y1) in rects:
    i0, i1 = max(0, int((x0 - 0.3) / RES)), min(gx, int(math.ceil((x1 + 0.3) / RES)))
    j0, j1 = max(0, int((y0 - 0.3) / RES)), min(gy, int(math.ceil((y1 + 0.3) / RES)))
    blocked[j0:j1, i0:i1] = True

exit_cells = []
for eid, e in venue["exits"].items():
    if e["open"]:
        cx = (e["x0"] + e["x1"]) / 2
        exit_cells.append((int(cx / RES), 0))
        blocked[0:2, int(e["x0"] / RES):int(e["x1"] / RES)] = False

def dijkstra_from_exits():
    """Distance-to-nearest-exit field over the routing grid (multi-source)."""
    dist = np.full((gy, gx), np.inf)
    pq = []
    for (i, j) in exit_cells:
        dist[j, i] = 0.0
        heapq.heappush(pq, (0.0, i, j))
    nbrs = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (-1, -1, 1.4142)]
    while pq:
        d, i, j = heapq.heappop(pq)
        if d > dist[j, i]:
            continue
        for di, dj, c in nbrs:
            ni, nj = i + di, j + dj
            if 0 <= ni < gx and 0 <= nj < gy and not blocked[nj, ni]:
                if di and dj and (blocked[j, ni] or blocked[nj, i]):
                    continue  # no corner cutting
                nd = d + c
                if nd < dist[nj, ni]:
                    dist[nj, ni] = nd
                    heapq.heappush(pq, (nd, ni, nj))
    return dist

dist_field = dijkstra_from_exits()


def pushout_field():
    """For every blocked cell, the unit vector toward the nearest free cell.

    Needed because the non-overlap relaxation can shove an agent into a wall;
    without a way back out the agent is reverted and stays overlapping, which
    is what let density climb past the physical packing limit.
    """
    from collections import deque
    src = np.full((gy, gx, 2), -1, dtype=np.int32)   # nearest free cell index
    dq = deque()
    for j in range(gy):
        for i in range(gx):
            if not blocked[j, i]:
                src[j, i] = (i, j)
                dq.append((i, j))
    while dq:
        i, j = dq.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < gx and 0 <= nj < gy and src[nj, ni, 0] < 0:
                src[nj, ni] = src[j, i]
                dq.append((ni, nj))
    ii, jj = np.meshgrid(np.arange(gx), np.arange(gy))
    vec = np.stack([src[..., 0] - ii, src[..., 1] - jj], -1).astype(float)
    n = np.linalg.norm(vec, axis=-1, keepdims=True)
    return np.divide(vec, n, out=np.zeros_like(vec), where=n > 0)


PUSHOUT = pushout_field()
# Descent direction per cell = toward the neighbour with the smallest distance.
flow = np.zeros((gy, gx, 2))
for j in range(gy):
    for i in range(gx):
        if blocked[j, i] or not np.isfinite(dist_field[j, i]):
            continue
        best, bd = None, dist_field[j, i]
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                ni, nj = i + di, j + dj
                if (di or dj) and 0 <= ni < gx and 0 <= nj < gy and dist_field[nj, ni] < bd:
                    bd, best = dist_field[nj, ni], (di, dj)
        if best:
            v = np.array(best, float)
            flow[j, i] = v / np.linalg.norm(v)

# ------------------------------------------------------------------ agents
# Agents start spread across the seating terrace on a jittered lattice whose
# spacing is >= the body diameter, so the initial state already satisfies the
# non-overlap constraint. Spawning them into a narrow strip in front of the
# stand put ~8 people per m² on frame 0 -- above the packing ceiling -- and the
# "peak density" then just measured that bad initial condition.
SPAWN_PITCH = max(0.60, 2.2 * args.body_radius)
P = []
for s in venue["stands"]:
    n = args.agents_per_stand
    x0, x1 = s["x0"] + 0.5, s["x1"] - 0.5
    y0, y1 = venue["stand_y0"] + 0.5, venue["stand_y0"] + venue["stand_depth"] - 0.5
    ncols = max(1, int((x1 - x0) / SPAWN_PITCH))
    nrows = int(math.ceil(n / ncols))
    if nrows * SPAWN_PITCH > (y1 - y0):
        raise SystemExit(f"stand {s['id']} cannot hold {n} agents at {SPAWN_PITCH:.2f} m pitch; "
                         f"need {nrows * SPAWN_PITCH:.1f} m of depth, have {y1 - y0:.1f} m")
    cx, cy = np.meshgrid(np.arange(ncols), np.arange(nrows))
    pts = np.stack([x0 + cx.ravel() * SPAWN_PITCH, y0 + cy.ravel() * SPAWN_PITCH], 1)[:n]
    pts += rng.uniform(-0.12, 0.12, pts.shape)   # break the lattice, stay clear of contact
    P.append(pts)
P = np.concatenate(P)
N = len(P)
V = np.zeros_like(P)
alive = np.ones(N, dtype=bool)
pref_speed = rng.normal(1.3, 0.15, N).clip(0.8, 1.7)

def cell_of(p):
    i = np.clip((p[:, 0] / RES).astype(int), 0, gx - 1)
    j = np.clip((p[:, 1] / RES).astype(int), 0, gy - 1)
    return i, j

# density bookkeeping
nx, ny = int(W / args.grid), int(H / args.grid)
peak = {"density": 0.0, "cell": None, "t": None}
worst_min_dist = np.inf   # tightest spacing seen over the whole run
over_thr_frames = 0
series = []           # (t, remaining, max_density)
snapshots = {}        # t -> density grid at peak (kept small: only the peak)
t, next_sample = 0.0, 0.0
t90 = t100 = None
start = time.time()
CHUNK = 512
SOCIAL_R = 0.75          # comfort distance for the soft repulsion term
# Theoretical packing ceiling for discs of this radius (hexagonal): used as a
# sanity bound printed with the result, not as a clamp.
PACK_MAX = 1.0 / (2.0 * math.sqrt(3.0) * args.body_radius ** 2)

while t < args.max_t and alive.any():
    idx = np.where(alive)[0]
    p = P[idx]
    i, j = cell_of(p)
    goal = flow[j, i] * pref_speed[idx, None]

    # soft social repulsion within SOCIAL_R (comfort distance), chunked for memory
    sep = np.zeros_like(p)
    for k0 in range(0, len(p), CHUNK):
        d = p[k0:k0 + CHUNK, None, :] - p[None, :, :]
        r2 = (d ** 2).sum(-1)
        m = (r2 < SOCIAL_R ** 2) & (r2 > 1e-9)
        r = np.sqrt(r2) + 1e-9
        f = np.where(m, (SOCIAL_R - r) / r, 0.0)
        sep[k0:k0 + CHUNK] = (d * f[..., None]).sum(1) * 1.2

    # obstacle repulsion: push away from blocked neighbour cells
    obs = np.zeros_like(p)
    for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        ni, nj = np.clip(i + di, 0, gx - 1), np.clip(j + dj, 0, gy - 1)
        hitm = blocked[nj, ni]
        obs[hitm] -= np.array([di, dj], float) * 1.5

    V[idx] = 0.5 * V[idx] + 0.5 * (goal + sep + obs)
    sp = np.linalg.norm(V[idx], axis=1) + 1e-9
    V[idx] *= np.minimum(1.0, 1.7 / sp)[:, None]
    newp = p + V[idx] * args.dt

    # --- hard non-overlap: position-based relaxation -------------------------
    # Bodies are discs of radius args.body_radius. Without this, agents pass
    # through each other and density rises past what is physically possible.
    # Each pass pushes overlapping pairs apart by half the overlap. This caps
    # attainable density near hexagonal packing of the discs.
    MIN_D = 2.0 * args.body_radius
    for _it in range(args.relax_iters):
        # Only genuinely close pairs matter, so a KD-tree replaces the O(N^2)
        # sweep. That makes enough iterations affordable for the solve to
        # actually converge inside a dense queue, which the all-pairs version
        # never managed.
        pairs = cKDTree(newp).query_pairs(MIN_D, output_type="ndarray")
        if len(pairs) == 0:
            break
        a, bq = pairs[:, 0], pairs[:, 1]
        d = newp[a] - newp[bq]
        r = np.linalg.norm(d, axis=1)
        deg = r < 1e-9                      # exactly coincident: shove apart randomly
        if deg.any():
            d[deg] = rng.normal(0, 1, (deg.sum(), 2))
            r[deg] = np.linalg.norm(d[deg], axis=1)
        half = (MIN_D - r) / r * 0.5
        shift = d * half[:, None]
        corr = np.zeros_like(newp)
        np.add.at(corr, a, shift)
        np.add.at(corr, bq, -shift)
        cn = np.linalg.norm(corr, axis=1, keepdims=True)
        corr *= np.minimum(1.0, MIN_D / np.maximum(cn, 1e-9))   # no explosions
        newp += corr
        # anyone shoved into geometry is walked back out, otherwise the
        # separation just applied is silently lost on the revert below
        for _ in range(4):
            ci, cj = cell_of(newp)
            insidewall = blocked[cj, ci]
            if not insidewall.any():
                break
            newp[insidewall] += PUSHOUT[cj[insidewall], ci[insidewall]] * RES

    # final guard: if still inside geometry, keep the previous position
    ni, nj = cell_of(newp)
    ok = ~blocked[nj, ni]
    P[idx[ok]] = newp[ok]
    V[idx[~ok]] *= 0.0

    # exit: crossing y < 0.5 through an open exit
    gone = P[idx, 1] < 0.6
    alive[idx[gone]] = False
    t += args.dt

    remaining = int(alive.sum())
    if t90 is None and remaining <= 0.1 * N:
        t90 = round(t, 1)
    if t100 is None and remaining == 0:
        t100 = round(t, 1)

    if t >= next_sample:
        next_sample += args.sample_every
        live = P[alive]
        grid = np.zeros((ny, nx))
        if len(live):
            ci = np.clip((live[:, 0] / args.grid).astype(int), 0, nx - 1)
            cj = np.clip((live[:, 1] / args.grid).astype(int), 0, ny - 1)
            np.add.at(grid, (cj, ci), 1)
        grid /= args.grid ** 2
        mx = float(grid.max())
        over_thr_frames += int((grid >= args.threshold).sum())
        series.append([round(t, 1), remaining, round(mx, 2)])
        # verification, every sample: the smallest centre-to-centre distance
        # actually present. If it drops below 2*body_radius the non-overlap
        # constraint failed and no density figure from this run is trustworthy.
        if len(live) > 1:
            dd, _ = cKDTree(live).query(live, k=2)
            mind = float(dd[:, 1].min())
            worst_min_dist = min(worst_min_dist, mind)
        else:
            mind = float("inf")

        if mx > peak["density"]:
            jj, ii = np.unravel_index(grid.argmax(), grid.shape)
            peak = {"density": round(mx, 2), "cell": [int(ii), int(jj)],
                    "cell_centre_m": [float((ii + 0.5) * args.grid), float((jj + 0.5) * args.grid)],
                    "t": round(t, 1), "min_pair_distance_m": round(mind, 3)}
            snapshots = {"t": round(t, 1), "grid": grid.round(2).tolist(), "positions": live.round(2).tolist()}

def _git_hash():
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=os.path.dirname(os.path.abspath(__file__)), stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "n/a"

result = {
    "venue": venue["name"], "scenario": "surge_to_exits", "seed": args.seed, "agents": int(N),
    "open_gates": [g for g, d in venue["gates"].items() if d["open"]],
    "open_exits": [e for e, d in venue["exits"].items() if d["open"]],
    "grid_m": args.grid, "threshold_per_m2": args.threshold, "dt_s": args.dt, "sample_every_s": args.sample_every,
    "peak_density_per_m2": peak["density"], "peak_cell": peak["cell"], "peak_cell_centre_m": peak.get("cell_centre_m"), "peak_t_s": peak["t"],
    "min_pair_distance_at_peak_m": peak.get("min_pair_distance_m"),
    "worst_min_pair_distance_m": round(float(worst_min_dist), 3),
    "nonoverlap_constraint_held": bool(worst_min_dist >= 2 * args.body_radius - 0.02),
    "cell_samples_over_threshold": over_thr_frames,
    "t90_s": t90, "t100_s": t100, "remaining_at_end": int(alive.sum()), "sim_end_t_s": round(t, 1),
    "series": series, "peak_snapshot": snapshots,
    "body_radius_m": args.body_radius,
    "packing_ceiling_per_m2": round(PACK_MAX, 2),
    "agent_model": ("simplified: distance-field goal-seeking + soft social repulsion + "
                    "position-based hard non-overlap (disc radius %.2f m); "
                    "not a validated pedestrian model" % args.body_radius),
    "provenance": {"git_hash": _git_hash(), "elapsed_s": round(time.time() - start, 1), "python": sys.version.split()[0],
                   "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())},
}
os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
json.dump(result, open(args.out, "w"), indent=1)
print(f"agents {N}  peak {peak['density']} /m² at {peak.get('cell_centre_m')} t={peak['t']}s  t90={t90}s  t100={t100}s  over-thr samples={over_thr_frames}")
print(f"  tightest spacing over run {result['worst_min_pair_distance_m']} m "
      f"(need >= {2*args.body_radius:.2f}); packing ceiling {PACK_MAX:.1f}/m²; "
      f"constraint held: {result['nonoverlap_constraint_held']}")
print(f"-> {args.out}")

# ------------------------------------------------------------------ Isaac Sim still (optional)
if not args.no_isaac and snapshots:
    try:
        try:
            from isaacsim.simulation_app import SimulationApp
        except ImportError:
            from isaacsim import SimulationApp  # type: ignore
        app = SimulationApp({"headless": True})
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image
        from pxr import Gf, UsdGeom
        ctx = omni.usd.get_context(); ctx.open_stage(args.venue); stage = ctx.get_stage()
        scope = UsdGeom.Scope.Define(stage, "/World/Agents")
        for k, (x, y) in enumerate(snapshots["positions"]):
            c = UsdGeom.Capsule.Define(stage, f"/World/Agents/a{k}")
            c.GetRadiusAttr().Set(0.22); c.GetHeightAttr().Set(1.3); c.GetAxisAttr().Set("Z")
            UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.87))
            c.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.25, 0.2)])
        out_dir = os.path.splitext(args.out)[0] + "_stills"; os.makedirs(out_dir, exist_ok=True)
        for name, pos, look in (("topdown", (W / 2, H / 2, 55.0), (W / 2, H / 2 + 0.01, 0.0)), ("gate_view", (30.0, 39.0, 6.0), (30.0, 18.0, 1.0))):
            cam = rep.create.camera(position=pos, look_at=look, focal_length=14.0)
            rp = rep.create.render_product(cam, (1280, 720))
            ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach([rp])
            for _ in range(8):
                rep.orchestrator.step(rt_subframes=8)
            Image.fromarray(np.asarray(ann.get_data())[..., :3]).save(os.path.join(out_dir, f"{name}_t{snapshots['t']}.png"))
            ann.detach(); rp.destroy()
        print(f"stills -> {out_dir}")
        app.close()
    except Exception as e:
        print(f"[warn] Isaac Sim still skipped: {e!r}")
