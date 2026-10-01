# GameDayTwin — a rehearsal engine for venue safety

**Presidio Innovation Sprint 2026 · LA Olympics '28 Challenge · Team Bankai**

> You cannot rehearse the Olympics. You cannot evacuate 70,000 people for practice, stage a
> midnight surge, or stand in a venue that hasn't been built yet. GameDayTwin builds the venue
> in simulation, runs the crowd and the cameras through it, and measures what no survey can:
> **where people will be that nobody is watching.**

Built on **NVIDIA Isaac Sim 6.0.1 / Omniverse** (OpenUSD twin, PhysX ray-casts, routed crowds,
RTX render), run on **AWS** (g5.2xlarge, A10G), fitting the camera network **Cisco** supplies
to LA28 as Official Network Equipment Partner.

---

## What it answers today — measured

Everything in this section was computed on a real Isaac Sim run. Nothing is illustrative.

| Question | Answer on the proxy venue |
|---|---|
| Which floor can no camera see? | **226 m²** — all of it on the concourse, where people walk. Seating is 100 % covered. |
| Why? | Camera C3 sees only **8 %** of the concourse: the seating blocks sit in its line of sight. |
| Does moving it help? | Yes. One camera moved to mid north wall → C3 **68 %** of the concourse, unwatched floor **188 m²**, union coverage **90.6 → 92.2 %**. |
| How does the crowd leave? | 428 people routed to exits: 90 % clear in **56.6 s**, gate load **226 / 202**. Opening a third gate balances it to **144 / 144 / 140**. |
| Where is the exposure? | *Unwatched person-seconds* — every second anyone spends on floor no camera sees — fuses the two. It ranks fixes by human exposure, not square metres. |

Full numbers: [`results/coverage_A_terrain.json`](results/coverage_A_terrain.json),
[`results/coverage_B_terrain.json`](results/coverage_B_terrain.json),
[`results/egress_before.json`](results/egress_before.json), [`results/egress_after.json`](results/egress_after.json).

## What is real, what is proxy, what is roadmap

| | Status |
|---|---|
| Venue | **Procedural proxy** — a 60 × 40 m stadium section: three stand blocks, a barrier with gates, four pillars, three exits. Not any real venue. |
| Camera coverage | **Measured.** PhysX `raycast_closest` from each camera to a 1 m grid, 1 m above the walking surface, following the stand tiers. Pillar footprints excluded. |
| Crowd egress | **Measured routing and throughput.** Shortest-path routes, staggered departures. People do **not** collide, so **no density figure is derived or claimed** — a density measured on overlapping bodies is meaningless. |
| Unwatched person-seconds | Derived from the two measurements above. Only as good as the crowd model. |
| Perception-AI test (camera sees the person, the model misses them) | **Roadmap.** Not built. Isaac Sim Replicator gives the ground-truth frames; the detector run is the next module. |
| Volumetric smoke (Omniverse Flow) | **Roadmap.** `omni.flowusd` is present in the container; not run. |
| Mounting feasibility | **Input, not output.** The engine ranks among mounting points the venue supplies. It cannot know whether a bracket fits. |

## The films and the certificate

All three open from disk in any browser. No server, no network, nothing to install.

| File | Length | What it is |
|---|---|---|
| [`app/cinematic.html`](app/cinematic.html) | 70 s | **Measured film.** The venue draws itself, 428 real routes play, the floor lights up by camera count, one camera moves. Every number on screen comes from `results/`. |
| [`app/vision.html`](app/vision.html) | 46 s | **Concept reel**, stamped *not a measurement*: the same engine on a transit platform, steward posts, exit-sign visibility and a temporary venue. No figures shown, by design. |
| [`app/index.html`](app/index.html) | — | **Readiness certificate.** Before/after toggle, coverage map, per-camera table with concourse/seating split, provenance. |

Controls in the films: `Space` play/pause · `R` restart · `← →` skip 5 s · `F` fullscreen · `H` hide bar.

## How it works

```
sim/venue_def.py       the venue as data (no USD dependency) — single source of truth
sim/build_venue.py     → OpenUSD stage with PhysX colliders            (usdpy.sh, no GPU)
sim/coverage.py        → ray-cast coverage per camera                   (Isaac Sim, GPU)
sim/bake_egress.py     → routed crowd as USD time samples + JSON tracks  (plain Python)
sim/paint_coverage.py  → coverage painted onto the 3D floor as instanced tiles
sim/remeasure.sh       → both camera sets in a throw-away container
app/*.html             → self-contained pages; data embedded at build time
```

Two primitives do all the work — **ray-cast visibility** (what can see what, through real
geometry) and **navmesh routing + throughput** (how people move and how fast a space
clears). Change the building file and the same code answers the same questions for a Metro
platform, a fan zone or a temporary overlay.

## Run it yourself

```bash
# routing + tracks need only Python 3 (no Isaac Sim)
python3 sim/venue_def.py --out results/venue_before.venue.json
python3 sim/bake_egress.py --venue results/venue_before.usd --report results/egress_before.json \
        --tracks results/tracks_before.json --seed 42

# geometry + coverage need Isaac Sim 6.0.1 (container nvcr.io/nvidia/isaac-sim:6.0.1)
bash sim/usdpy.sh sim/build_venue.py --out results/venue_before.usd
bash sim/remeasure.sh          # both camera sets, terrain mode
```

Setting Isaac Sim up on EC2 cost us a night of debugging; every hurdle and its fix is in
[`docs/ISAAC_SIM_ON_EC2.md`](docs/ISAAC_SIM_ON_EC2.md) — read it before you start a box.

## Why simulation, and not cameras or footage

An AI camera tells you what it saw. Nothing tells you what it missed — a missed person
produces no alert, no log, no error. In the real world there is no ground truth, so the
failure is invisible by construction. Recorded footage has the same problem, contains none
of the rare events, and does not exist for a venue that has not been built.

In a twin you know where every person is, you can re-run the identical scenario after a fix,
you can stage the dangerous case safely, and you can rehearse a building that only exists in
simulation. It also needs **no real video**: no footage of real people, no PII, no
data-sharing agreement.

*Cars have crash tests. Buildings have fire drills. Safety AI has nothing — yet.*

## Honesty notes

- The first coverage run reported 850 m² unseen. 624 m² of that was the *inside* of the
  solid seating blocks — the test point sat inside the geometry. That number is superseded
  and should not be quoted. The concourse figure was identical in both runs (226 m²).
- C1 ≡ C2 and C3 ≡ C4 in the original layout because the venue and the rig are symmetric
  about the centre line — a consistency check, not duplicated data.
- `coverage.py` aborts inside `simulation_app.close()` *after* writing its results. Cosmetic.
- The egress film and the 3D scenes use capsules, not human models: NVIDIA's crowd
  extension is not in the 6.0.1 container image and the People assets were not reachable.

## Scalability and legacy

The engine is content-agnostic. Venues, Metro stations and platforms (LA28's no-parking
mandate puts every visitor through transit), temporary overlays, LAX, fan zones — same
pipeline, different USD. After the Games, Los Angeles keeps the twins: wildfire evacuation
rehearsal, airport operations, Metro planning. Readiness is a subscription by nature: every
camera moved, model updated or layout changed re-runs the suite.

---

Sridhar Suresh · Team Bankai · 2026
