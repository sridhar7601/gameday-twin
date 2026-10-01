# Running Isaac Sim on EC2 — what actually bit us

Everything here was hit and solved on 2026-09-30 with Isaac Sim 6.0.1 on a g5.2xlarge.
Read this before starting a box; it is roughly four hours of debugging compressed.

## 1. The one that cost the most: container cache permissions

**Symptom.** GUI connects but the viewport is black, `FPS: 0.00`, RTX renders never
finish, the app feels broken. Logs show `PermissionError: Permission denied:
'/isaac-sim/.cache/warp'`, `Failed to create texture cache`, `Failed to acquire
exclusive lock to data store`.

**Cause.** The container runs as **uid 1234** (`isaac-sim`). Host directories mounted
into it are owned by `ubuntu` (uid 1000), so Isaac Sim cannot write warp, shader or
texture caches — and without a writable shader cache nothing renders.

**Fix.** Before starting the container:
```bash
sudo chown -R 1234:1234 ~/docker/isaac-sim
sudo chown -R 1234:1234 <any host dir the container writes to>   # e.g. results/
```
After this the viewport went to 40–60 FPS immediately. **Check this first whenever the
viewport is black.**

## 2. Only one Kit process at a time

A render script (`SimulationApp`) and the streaming GUI both want the same cache lock.
Run them together and the second one hangs at "app ready" with the GPU at 0% — it looks
like a slow render, it is a deadlock. Stop the GUI before batch rendering, or give the
batch job its own cache directory.

## 3. `pxr` is not importable from `python.sh`

Authoring USD needs no GPU and no Kit, but `./python.sh -c "from pxr import Usd"` fails
with `ModuleNotFoundError`, then with `libusd_tf.so: cannot open shared object file`.

The bindings live in an extension cache directory that is not on the default paths. Use
[`sim/usdpy.sh`](../sim/usdpy.sh), which sets:
```bash
U=$(ls -d /isaac-sim/extscache/omni.usd.libs-* | head -1)
export PYTHONPATH="$U:$PYTHONPATH"
export LD_LIBRARY_PATH="$U/bin:$LD_LIBRARY_PATH"
exec /isaac-sim/kit/python/bin/python3 "$@"
```

## 4. USD `customData` cannot hold lists of dictionaries

`SetCustomDataByKey("venue", {...with a list of dicts...})` raises
`ValueError: ... is not a valid scene description datatype`. Store JSON as a **string**
(`json.dumps`) and keep a `.venue.json` sidecar as the machine-readable copy.

## 5. Replicator's orchestrator is `None` in a bare SimulationApp

`rep.orchestrator.step()` fails with `AttributeError: 'NoneType' object has no attribute
'status'` because the extension never started. Enable it explicitly:
```python
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.replicator.core")
simulation_app.update()
import omni.replicator.core as rep
```

## 6. PhysX scene queries need physics to have stepped

`raycast_closest` returns nothing until the timeline has played and a few frames have
elapsed. Do `omni.timeline.get_timeline_interface().play()` then ~10
`simulation_app.update()` calls before querying.

## 7. `simulation_app.close()` can abort after a successful run

Scripts print their results, write their JSON, then die with `Fatal Python error:
Aborted` inside `close()`. Cosmetic — the output is already on disk. Do not let it make
you think the run failed.

## 8. Viewing the GUI

There is **no web UI**. Browsing to the instance IP times out because nothing is
listening on 80/443. The GUI reaches you only through the **Isaac Sim WebRTC Streaming
Client** desktop app:
```bash
/isaac-sim/isaac-sim.streaming.sh --allow-root
```
then connect the client to the public IP. Ports: **49100/TCP**, **47995–48012/UDP**
(plus 22 for SSH). `carb.windowing-glfw.plugin` warnings are normal headless.

`--exec <script>` did **not** fire for us as a way to auto-open a stage; open the file
from the GUI instead (`File > Open`, or the Content panel path box).

## 9. What is and is not in the 6.0.1 container

| Extension | Present |
|---|---|
| `omni.anim.navigation.core` (navmesh) | ✅ |
| `omni.flowusd` (smoke/fire) | ✅ |
| `omni.replicator.core` | ✅ (enable explicitly) |
| `isaacsim.replicator.agent` (IRA crowd) | ❌ |
| `omni.anim.people` | ❌ |

So NVIDIA's crowd-animation system is **not** available in this image. Crowds must be
authored yourself — see [`sim/bake_egress.py`](../sim/bake_egress.py), which writes agent
positions as USD time samples so the timeline plays them back.

NVIDIA's People assets **are** reachable — but not at guessed paths. Ask Isaac Sim for its
own root instead of guessing (`sim/probe_people.py`):
```python
from isaacsim.storage.native import get_assets_root_path
root = get_assets_root_path()   # .../Assets/Isaac/6.0 for this build
```
Under `{root}/Isaac/People/` there are `Characters/` (business, medical, police,
construction; `F_Business_02/F_Business_02.usd` is a Z-up SkelRoot with a 101-joint
skeleton), `DH_Characters/` (23 digital humans) and `Animations/` (walk loops such as
`stand_walk_loop_in_place.skelanim.usd`, 81 joints, 30 fps, Y-up, centimetres). They are
plain UsdSkel assets, so they can be referenced and driven without `omni.anim.people`.
Not yet done on this project: binding the walk cycle and moving characters along the
baked egress tracks.

## 10. AWS specifics

- An org SCP restricted GPU instances to **g5.2xlarge** and **g6e.xlarge** only; g6e had
  no capacity in either us-east-1 AZ, so g5.2xlarge (A10G 24 GB, 8 vCPU, 30 GB RAM) it was.
  Decode a refusal with `aws sts decode-authorization-message`.
- AMI: **Deep Learning Base OSS Nvidia Driver GPU AMI (Ubuntu 22.04)** — driver 595.91.07,
  Docker and the NVIDIA container toolkit preinstalled, no Marketplace subscription.
  Isaac Sim 6.0.1 wants driver ≥ 595.58.03, so this clears it.
- `docker pull nvcr.io/nvidia/isaac-sim:6.0.1` needs no NGC login. ~32 GB on disk; give
  the root volume 250 GB.
- A **stopped** instance keeps its disk but gets a **new public IP** on restart — re-add
  it to the security group each time.

## 11. Cost

g5.2xlarge on-demand is roughly **$1.2/hour**; a 250 GB gp3 root volume about **$16/month**
and it keeps billing while the instance is stopped. Stop the instance between sessions;
terminate it when the project ends.
