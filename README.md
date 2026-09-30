# GameDayTwin — venue rehearsal engine (finals build, 2026-09-30/10-01)

A fresh, minimal build that produces **real measured numbers** on NVIDIA Isaac Sim for two
questions a venue safety team asks before an event:

1. **Where does a surge concentrate people, and does a layout change fix it?**
   (`sim/density.py` — crowd density per 1 m² cell over time, before vs after, same seed)
2. **Which floor area can the cameras not see?**
   (`sim/coverage.py` — PhysX ray-cast coverage certificate per camera, before vs after a camera move)

## What is real, what is proxy, what is simplified — say this on stage

| Item | Status |
|---|---|
| Venue geometry | **Procedural proxy** ("stadium-section proxy"): 60 × 40 m concourse, 3 stand blocks, barrier row with gates, 4 pillars, 3 exits. Not any real venue. |
| Coverage numbers | ✅ **Real measurement** — PhysX `raycast_closest` from each camera to a 1 m grid at 1 m height, on EC2 g5.2xlarge (A10G) in the Isaac Sim 6.0.1 container. |
| Crowd density numbers | ❌ **Do not use.** `density.py` runs, but fails its own non-overlap check (tightest agent spacing 0.028 m against a 0.40 m body diameter), so the densities it reports exceed the physical packing ceiling of 7.2 persons/m². Treated as roadmap, not a result. |
| Density threshold | Configurable (`--threshold`, default 4 persons/m²). **Cite a source before putting the number on a slide.** |
| Renders | Not produced in this build — `coverage.py --stills` exists but was not run. |
| Smoke (Flow), ML perception tests | **Roadmap** — not run. (`omni.flowusd` is confirmed present in the container.) |

## Measured result (2026-09-30)

Camera set A, four cameras at 90° HFOV on the perimeter at 5 m:

| Metric | Value |
|---|---|
| Floor seen by ≥1 camera | **64.58 %** of 2,400 m² |
| Floor no camera sees | **850 m²** (35.4 %) |
| Floor seen by ≥2 cameras | 45.0 % |
| C1 / C2 | 54.0 % each |
| C3 / C4 | **5.46 %** each — 2,046 cells blocked by the stand blocks |

C1/C2 and C3/C4 match exactly because the venue and camera rig are symmetric about the
hall centre line — a consistency check, not duplicated data. Ray-cast time 0.9 s.

Open `app/index.html` in any browser (no server, no network) to see it.

### Known gaps in this run
- `coverage.py` crashes in `simulation_app.close()` *after* writing results — cosmetic, the JSON is complete.
- `provenance.isaac_sim_version` reads "unknown" and `git_hash` "n/a" (the version file and git dir are not visible inside the container).
- Camera set B was not run, so there is no before/after camera-move comparison yet.

## Layout
```
sim/build_venue.py   procedural venue → venue.usd + venue.venue.json (before: gate G3 closed; --open-gates G3 for after)
sim/cameras.json     camera set A and set B (B moves C3 to look along the gates)
sim/coverage.py      coverage certificate → results/coverage_<set>.json (+ stills)
sim/density.py       surge scenario → results/surge_<before|after>.json (+ stills)
sim/run_all.sh       exact command order on the GPU box
results/             committed evidence (json, png)
app/index.html       single-file viewer of results/ — opens offline
```

## Run (GPU box, Isaac Sim 6.0.1 at /opt/IsaacSim)
```bash
cd ~/gameday-twin && bash sim/run_all.sh
```
`density.py` also runs with plain python + numpy (`--no-isaac`) for the numbers alone.

## Provenance
Every result JSON carries seed, git hash, Isaac Sim version (where applicable), EC2 instance
type and timestamp. The viewer shows them.
