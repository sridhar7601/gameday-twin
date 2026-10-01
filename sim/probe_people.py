"""Probe: where are NVIDIA's People character assets for this Isaac Sim build?

Boots a headless Kit, asks Isaac Sim for its own asset root (instead of guessing
S3 paths), then lists what is under Isaac/People. Prints one line per finding so
it can be grepped from the container log.
"""
import sys
from isaacsim.simulation_app import SimulationApp

app = SimulationApp({"headless": True})
try:
    import omni.client
    from isaacsim.storage.native import get_assets_root_path

    root = get_assets_root_path()
    print(f"[P] assets_root = {root}")
    if not root:
        print("[P] no assets root resolved")
        sys.exit(0)

    def ls(path, depth=0, max_depth=2):
        res, entries = omni.client.list(path)
        if res != omni.client.Result.OK:
            print(f"[P] list {path} -> {res}")
            return
        for e in entries:
            name = e.relative_path
            is_dir = bool(e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN)
            print(f"[P] {'  ' * depth}{name}{'/' if is_dir else ''}  ({path})")
            if is_dir and depth < max_depth:
                ls(f"{path}/{name}", depth + 1, max_depth)

    for sub in ("Isaac/People", "Isaac/People/Characters", "Isaac/Props/People", "Isaac/Environments"):
        print(f"[P] --- {sub} ---")
        ls(f"{root}/{sub}", max_depth=1)
finally:
    app.close()
