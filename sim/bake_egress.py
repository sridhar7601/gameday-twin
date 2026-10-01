"""
bake_egress.py — animated egress baked into USD as time samples.

Every person walks the shortest route from their seat to their nearest open
exit, computed with Dijkstra over a 0.5 m grid of the venue. Departures are
staggered so people leave in waves rather than all at once. Positions are
written as USD time samples on a PointInstancer, so pressing Play in Isaac Sim
animates the whole crowd, and the timeline can be scrubbed.

WHAT THIS IS, AND IS NOT
------------------------
It is: routing and throughput. Which way people go, which gate they use, and
how many clear each gate and exit per second. Those come out of the geometry
and the route lengths, and are honest.

It is NOT crowd dynamics. People do not push, and are not stopped by the person
in front, so they can pass through each other in a queue. Therefore this script
deliberately reports NO density figure -- a density measured on overlapping
bodies is meaningless, which is exactly what sank the earlier attempt. Use the
throughput numbers, never a persons-per-square-metre number from this.

  bash sim/usdpy.sh sim/bake_egress.py \
      --venue results/venue_before.usd --out results/egress_before.usd \
      --report results/egress_before.json --seed 42
"""
import argparse
import heapq
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from venue_def import segments as _segments  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--venue", required=True)
ap.add_argument("--out", help="output .usd; omit to skip USD authoring (no pxr needed)")
ap.add_argument("--report", required=True)
ap.add_argument("--tracks", help="also write compact per-person routes as JSON "
                                 "(path + start time + speed), for replay outside Isaac Sim")
ap.add_argument("--people", type=int, default=450)
ap.add_argument("--seed", type=int, default=42)
ap.add_argument("--fps", type=float, default=12.0, help="time samples per second")
ap.add_argument("--max-t", type=float, default=90.0)
ap.add_argument("--speed", type=float, default=1.25, help="mean walking speed m/s")
ap.add_argument("--stagger", type=float, default=25.0, help="seconds over which people set off")
args = ap.parse_args()

rng = random.Random(args.seed)
venue = json.load(open(os.path.splitext(args.venue)[0] + ".venue.json"))
W, H = venue["size"]["x"], venue["size"]["y"]
RES = 0.5
gx, gy = int(W / RES), int(H / RES)

# ---------------------------------------------------------------- obstacles
wt, bar = venue["wall_thickness"], venue["barrier"]
rects = []
for x0, x1 in _segments(0, W, venue["exits"].values()):
    rects.append((x0, 0.0, x1, wt))
rects += [(0, H - wt, W, H), (0, 0, wt, H), (W - wt, 0, W, H)]
for x0, x1 in _segments(0, W, venue["gates"].values()):
    rects.append((x0, bar["y"] - bar["thickness"] / 2, x1, bar["y"] + bar["thickness"] / 2))
for p in venue["pillars"]:
    rects.append((p["x"] - 0.5, p["y"] - 0.5, p["x"] + 0.5, p["y"] + 0.5))
# Stands are seating terraces: walkable, so they are not obstacles here.

blocked = [[False] * gx for _ in range(gy)]
for (x0, y0, x1, y1) in rects:
    for j in range(max(0, int((y0 - 0.25) / RES)), min(gy, int(math.ceil((y1 + 0.25) / RES)))):
        for i in range(max(0, int((x0 - 0.25) / RES)), min(gx, int(math.ceil((x1 + 0.25) / RES)))):
            blocked[j][i] = True

exit_cells, exit_of = [], {}
for eid, e in venue["exits"].items():
    if not e["open"]:
        continue
    for i in range(int(e["x0"] / RES), int(e["x1"] / RES)):
        blocked[0][i] = blocked[1][i] = False
        exit_cells.append((i, 0))
        exit_of[(i, 0)] = eid

# ------------------------------------------------- distance field to exits
INF = float("inf")
dist = [[INF] * gx for _ in range(gy)]
owner = [[None] * gx for _ in range(gy)]
pq = []
for (i, j) in exit_cells:
    dist[j][i] = 0.0
    owner[j][i] = exit_of[(i, j)]
    heapq.heappush(pq, (0.0, i, j))
NB = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
      (1, 1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (-1, -1, 1.4142)]
while pq:
    d, i, j = heapq.heappop(pq)
    if d > dist[j][i]:
        continue
    for di, dj, c in NB:
        ni, nj = i + di, j + dj
        if 0 <= ni < gx and 0 <= nj < gy and not blocked[nj][ni]:
            if di and dj and (blocked[j][ni] or blocked[nj][i]):
                continue
            nd = d + c
            if nd < dist[nj][ni] - 1e-9:
                dist[nj][ni] = nd
                owner[nj][ni] = owner[j][i]
                heapq.heappush(pq, (nd, ni, nj))

def route(i, j):
    """Steepest descent to an exit; returns a list of (x, y) in metres."""
    path = []
    steps = 0
    while dist[j][i] > 0 and steps < 4000:
        path.append(((i + 0.5) * RES, (j + 0.5) * RES))
        best, bd = None, dist[j][i]
        for di, dj, _ in NB:
            ni, nj = i + di, j + dj
            if 0 <= ni < gx and 0 <= nj < gy and dist[nj][ni] < bd:
                bd, best = dist[nj][ni], (ni, nj)
        if best is None:
            break
        i, j = best
        steps += 1
    path.append(((i + 0.5) * RES, -1.2))     # step out through the doorway
    return path, owner[j][i]

# ------------------------------------------------------------ seat the crowd
seats = []
for s in venue["stands"]:
    x0, x1 = s["x0"] + 1.0, s["x1"] - 1.0
    y0, y1 = venue["stand_y0"] + 1.0, venue["stand_y0"] + venue["stand_depth"] - 1.0
    nx_ = max(1, int((x1 - x0) / 1.1))
    ny_ = max(1, int((y1 - y0) / 1.1))
    for a in range(nx_):
        for b in range(ny_):
            seats.append((x0 + a * 1.1, y0 + b * 1.1))
rng.shuffle(seats)
seats = seats[:args.people]

people = []
for (sx, sy) in seats:
    i, j = min(int(sx / RES), gx - 1), min(int(sy / RES), gy - 1)
    if blocked[j][i]:
        continue
    path, eid = route(i, j)
    if len(path) < 2:
        continue
    seglen = [math.dist(path[k], path[k + 1]) for k in range(len(path) - 1)]
    people.append({
        "path": path, "seglen": seglen, "total": sum(seglen),
        "speed": rng.gauss(args.speed, 0.14),
        "t0": rng.uniform(0, args.stagger),
        "exit": eid,
    })

# --------------------------------------------------- walk + collect throughput
def pos_at(p, t):
    """Position at time t, or None once the person is out."""
    if t < p["t0"]:
        return p["path"][0]
    travelled = (t - p["t0"]) * max(p["speed"], 0.3)
    if travelled >= p["total"]:
        return None
    acc = 0.0
    for k, L in enumerate(p["seglen"]):
        if acc + L >= travelled:
            f = (travelled - acc) / L if L else 0.0
            (x0, y0), (x1, y1) = p["path"][k], p["path"][k + 1]
            return (x0 + (x1 - x0) * f, y0 + (y1 - y0) * f)
        acc += L
    return None

nframes = int(args.max_t * args.fps) + 1
gate_y = bar["y"]
prev_side, prev_out = {}, {}
gate_counts = {g: 0 for g in venue["gates"] if venue["gates"][g]["open"]}
exit_counts = {e: 0 for e in venue["exits"] if venue["exits"][e]["open"]}
series = []
positions_per_frame = []
cleared_t = {}

for f in range(nframes):
    t = f / args.fps
    frame, still_in = [], 0
    for idx, p in enumerate(people):
        q = pos_at(p, t)
        if q is None:
            if idx not in prev_out:
                prev_out[idx] = t
                exit_counts[p["exit"]] = exit_counts.get(p["exit"], 0) + 1
                cleared_t[idx] = t
            continue
        still_in += 1
        frame.append(q)
        # gate crossing: north side -> south side of the barrier row
        side = q[1] > gate_y
        if idx in prev_side and prev_side[idx] and not side:
            for gid, g in venue["gates"].items():
                if g["open"] and g["x0"] - 0.6 <= q[0] <= g["x1"] + 0.6:
                    gate_counts[gid] = gate_counts.get(gid, 0) + 1
                    break
        prev_side[idx] = side
    positions_per_frame.append(frame)
    if f % int(args.fps) == 0:
        series.append([round(t, 1), still_in, len(prev_out)])
    if still_in == 0 and f > args.fps:
        nframes = f + 1
        break

N = len(people)
times = sorted(cleared_t.values())
t50 = round(times[int(.5 * N)], 1) if len(times) > N * .5 else None
t90 = round(times[int(.9 * N)], 1) if len(times) > N * .9 else None
t100 = round(times[-1], 1) if len(times) == N else None

# ------------------------------------------------------------------ write USD
if args.out:
    # Imported here, not at the top, so the routing and the tracks export run on
    # any machine with plain Python -- pxr only exists inside Isaac Sim.
    from pxr import Gf, Usd, UsdGeom, UsdLux

    stage = Usd.Stage.CreateNew(args.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(nframes - 1)
    stage.SetTimeCodesPerSecond(args.fps)
    stage.SetFramesPerSecond(args.fps)

    ov = stage.OverridePrim("/World/Venue")
    ov.GetReferences().AddReference(os.path.abspath(args.venue), "/World/Venue")

    proto = UsdGeom.Scope.Define(stage, "/World/Crowd/proto")
    body = UsdGeom.Capsule.Define(stage, "/World/Crowd/proto/person")
    body.CreateRadiusAttr(0.22)
    body.CreateHeightAttr(1.25)
    body.CreateAxisAttr("Z")
    body.CreateDisplayColorAttr([Gf.Vec3f(0.90, 0.32, 0.20)])
    UsdGeom.Xformable(body).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.85))

    inst = UsdGeom.PointInstancer.Define(stage, "/World/Crowd")
    inst.CreatePrototypesRel().SetTargets([body.GetPath()])
    pos_attr = inst.CreatePositionsAttr()
    idx_attr = inst.CreateProtoIndicesAttr()
    maxn = max(len(f) for f in positions_per_frame)
    for f, frame in enumerate(positions_per_frame):
        pts = [Gf.Vec3f(x, y, 0.0) for (x, y) in frame]
        pts += [Gf.Vec3f(0, 0, -50.0)] * (maxn - len(pts))   # park the departed below the floor
        pos_attr.Set(pts, Usd.TimeCode(f))
        idx_attr.Set([0] * maxn, Usd.TimeCode(f))

    UsdLux.DomeLight.Define(stage, "/World/EgressLight").CreateIntensityAttr(900.0)
    stage.GetRootLayer().Save()

# ------------------------------------------------------ compact tracks (replay)
if args.tracks:
    # Route polylines are tiny compared with per-frame positions, and a player can
    # interpolate along them -- this is what the cinematic page consumes.
    tracks = [{"p": [[round(x, 2), round(y, 2)] for (x, y) in p["path"]],
               "t0": round(p["t0"], 2), "v": round(p["speed"], 3), "exit": p["exit"]}
              for p in people]
    with open(args.tracks, "w") as fh:
        json.dump({"venue": venue["name"], "seed": args.seed, "people": N,
                   "gates_open": sorted(gate_counts), "tracks": tracks}, fh, separators=(",", ":"))

report = {
    "venue": venue["name"], "scenario": "egress_routing", "seed": args.seed,
    "people": N, "open_gates": sorted(gate_counts), "open_exits": sorted(exit_counts),
    "gate_throughput_total": gate_counts, "exit_throughput_total": exit_counts,
    "t50_s": t50, "t90_s": t90, "t100_s": t100,
    "sim_span_s": round((nframes - 1) / args.fps, 1), "fps": args.fps,
    "walk_speed_mean_ms": args.speed, "stagger_s": args.stagger,
    "remaining_series_t_in_out": series,
    "model": ("shortest-path egress routing with staggered departures; positions baked as USD "
              "time samples. Routing and throughput only -- people do not collide, so NO "
              "density figure is derived or claimed from this run."),
}
json.dump(report, open(args.report, "w"), indent=1)

print(f"wrote {args.out or '(no usd)'}  ({nframes} frames @ {args.fps} fps, {N} people)"
      + (f"  tracks -> {args.tracks}" if args.tracks else ""))
print(f"  gate throughput: {gate_counts}")
print(f"  exit throughput: {exit_counts}")
print(f"  cleared: 50% {t50}s | 90% {t90}s | 100% {t100}s")
print(f"  report -> {args.report}")
