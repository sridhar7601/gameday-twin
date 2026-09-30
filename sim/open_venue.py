"""
open_venue.py — startup hook for the streaming GUI.

Passed to Kit as `--exec`, this opens the proxy venue stage and frames it, so a
viewer connecting over WebRTC sees the venue immediately instead of an empty
black viewport (and does not need the menu bar to load a file).
"""
import carb
import omni.usd

STAGE = carb.settings.get_settings().get("/exts/gamedaytwin/stage") or \
    "/workspace/results/venue_before.usd"


def _frame():
    """Select the venue root and frame it in the active viewport."""
    try:
        import omni.kit.commands
        from omni.kit.viewport.utility import get_active_viewport
        ctx = omni.usd.get_context()
        ctx.get_selection().set_selected_prim_paths(["/World/Venue"], True)
        vp = get_active_viewport()
        if vp is not None:
            import omni.kit.viewport.utility.camera_state  # noqa: F401
            omni.kit.commands.execute("FramePrimsCommand",
                                      prim_to_move=None,
                                      prims_to_frame=["/World/Venue"],
                                      usd_context_name=ctx.get_name())
    except Exception as exc:  # framing is cosmetic; never block startup
        carb.log_warn(f"[gamedaytwin] could not frame venue: {exc!r}")


def _on_open(result, err):
    if result:
        carb.log_warn(f"[gamedaytwin] opened {STAGE}")
        _frame()
    else:
        carb.log_error(f"[gamedaytwin] failed to open {STAGE}: {err}")


omni.usd.get_context().open_stage_with_callback(STAGE, _on_open)
