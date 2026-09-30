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
| Coverage numbers | **Real geometric measurement**: PhysX `raycast_closest` from each camera to a 1 m grid at 1 m height. |
| Crowd agents | **Simplified agent model** (distance-field goal-seeking + separation + obstacle repulsion). Not a validated pedestrian model. Isaac Sim Replicator Agent (IRA) characters are the roadmap replacement. |
| Density threshold | Configurable (`--threshold`, default 4 persons/m²). **Cite a source before putting the number on a slide.** |
| Renders | Isaac Sim RTX stills of the proxy venue and agent capsules. |
| Smoke (Flow), ML perception tests | **Roadmap** — not run in this build. |

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
