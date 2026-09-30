"""
venue_def.py — the single source of truth for the proxy venue's dimensions.

Deliberately free of any USD / Isaac Sim import so that the crowd simulation
(density.py) and the geometry builder (build_venue.py) agree on the layout and
so the manifest can be produced on any machine.

This is NOT a real venue: a 60 x 40 m concourse with three stand blocks, a
barrier row with gates, four pillars and three exits. Always labelled
"stadium-section proxy".
"""
import json

VENUE = {
    "name": "stadium_section_proxy_v1",
    "size": {"x": 60.0, "y": 40.0},
    "wall_height": 4.0,
    "wall_thickness": 0.4,
    # Barrier row at y=20 with gates. Agents from the stands must pass a gate.
    "barrier": {"y": 20.0, "height": 1.2, "thickness": 0.3},
    "gates": {
        "G1": {"x0": 18.0, "x1": 21.0, "open": True},
        "G2": {"x0": 39.0, "x1": 42.0, "open": True},
        # G3 is the "fix": closed in the before-run, opened in the after-run.
        "G3": {"x0": 28.5, "x1": 31.5, "open": False},
    },
    # Exits are gaps in the south wall (y=0).
    "exits": {
        "E1": {"x0": 8.0, "x1": 11.0, "open": True},
        "E2": {"x0": 28.5, "x1": 31.5, "open": True},
        "E3": {"x0": 49.0, "x1": 52.0, "open": True},
    },
    # Stand blocks: tiered boxes rising toward the north wall.
    "stands": [
        {"id": "S1", "x0": 2.0, "x1": 18.0},
        {"id": "S2", "x0": 22.0, "x1": 38.0},
        {"id": "S3", "x0": 42.0, "x1": 58.0},
    ],
    "stand_tiers": 5,
    "stand_y0": 24.0,          # front row
    "stand_depth": 16.0,       # to the north wall
    "stand_rise": 5.0,         # total height
    "pillars": [               # 1 m square columns, full wall height
        {"id": "P1", "x": 15.0, "y": 10.0},
        {"id": "P2", "x": 30.0, "y": 10.0},
        {"id": "P3", "x": 45.0, "y": 10.0},
        {"id": "P4", "x": 30.0, "y": 26.0},
    ],
}


def resolve(open_gate_ids=(), open_exit_ids=()):
    """Deep copy of VENUE with the named gates/exits forced open."""
    v = json.loads(json.dumps(VENUE))
    for gid in open_gate_ids:
        v["gates"][gid]["open"] = True
    for eid in open_exit_ids:
        v["exits"][eid]["open"] = True
    return v


def segments(x_lo, x_hi, gaps):
    """Split [x_lo, x_hi] into solid wall segments around the open gaps."""
    open_gaps = sorted((g["x0"], g["x1"]) for g in gaps if g["open"])
    segs, cur = [], x_lo
    for g0, g1 in open_gaps:
        if g0 > cur:
            segs.append((cur, g0))
        cur = max(cur, g1)
    if cur < x_hi:
        segs.append((cur, x_hi))
    return segs


if __name__ == "__main__":  # write a manifest without needing USD
    import argparse
    ap = argparse.ArgumentParser(description="emit the venue manifest json only")
    ap.add_argument("--out", required=True)
    ap.add_argument("--open-gates", default="")
    ap.add_argument("--open-exits", default="")
    a = ap.parse_args()
    v = resolve([g for g in a.open_gates.split(",") if g], [e for e in a.open_exits.split(",") if e])
    with open(a.out, "w") as f:
        json.dump(v, f, indent=2)
    print(f"wrote {a.out}; open gates "
          f"{[g for g, d in v['gates'].items() if d['open']]}, "
          f"open exits {[e for e, d in v['exits'].items() if d['open']]}")
