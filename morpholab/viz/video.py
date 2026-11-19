"""Episode video: replay a rollout offscreen and write frames to a file.

Recording is a *replay*, not a second simulation: the rollout is re-run with a frame hook,
which is deterministic and therefore shows exactly the trial that produced the numbers.
Rendering during the original run would be wasteful in a sweep, where 99.9 % of trials are
never watched.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np

from ..core.logging import get_logger
from ..core.types import Phase
from .render import CameraSpec, Renderer, renderer_available, tile

log = get_logger(__name__)

__all__ = ["record_episode", "write_video", "side_by_side"]


def write_video(frames: Sequence[np.ndarray], path: str | Path, *, fps: float = 30.0) -> Path:
    """Write RGB frames to mp4 (or gif, chosen by the file extension)."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if not frames:
        raise ValueError("no frames to write")
    import imageio.v3 as iio

    if p.suffix.lower() == ".gif":
        iio.imwrite(p, np.asarray(frames), duration=1000 / fps, loop=0)
    else:
        iio.imwrite(p, np.asarray(frames), fps=fps, codec="libx264", quality=8)
    return p


def record_episode(
    rollout: Any,
    task: Any,
    *,
    path: str | Path,
    camera: CameraSpec | str = "grasp_front",
    width: int = 800,
    height: int = 600,
    fps: float = 30.0,
    slow_motion: float = 1.0,
    overlay_contacts: bool = True,
) -> Path:
    """Re-run ``rollout`` under ``task``, capturing frames, and write a video.

    ``slow_motion`` above 1 stretches time by duplicating simulated frames rather than by
    slowing the physics, so the recorded motion is exactly the simulated motion.
    """
    import mujoco

    if not renderer_available():
        raise RuntimeError("no working GL backend; video export is unavailable")

    model, data = rollout.model, rollout.data
    dt = float(model.opt.timestep)
    stride = max(1, int(round(1.0 / (fps * dt))))
    frames: list[np.ndarray] = []

    with Renderer(model, width, height) as renderer:
        renderer.set_camera(camera)
        if overlay_contacts:
            renderer.set_flags(contactpoint=True, contactforce=True)

        mujoco.mj_resetData(model, data)
        rollout._apply_initial_state()
        rollout.controller.reset(rollout.ctx, data)
        rollout._apply_preshape_posture()
        mujoco.mj_forward(model, data)

        step = 0
        for stage in task.stages:
            base = rollout._current_wrist()
            n = max(1, int(round(stage.duration / dt)))
            for i in range(n):
                rollout._set_wrist(stage.wrist.pose_at(i * dt, base, stage.duration))
                data.ctrl[:] = rollout.controller.act(rollout.ctx, data, float(data.time), stage.grip)
                mujoco.mj_step(model, data)
                step += 1
                if step % stride == 0:
                    frame = renderer.frame(data)
                    frames.extend([frame] * max(1, int(round(slow_motion))))
                if not np.all(np.isfinite(data.qpos)):
                    break

    log.debug("recorded %d frames", len(frames))
    return write_video(frames, path, fps=fps)


def side_by_side(
    rollouts: Sequence[tuple[Any, Any]],
    *,
    path: str | Path,
    labels: Sequence[str] = (),
    camera: CameraSpec | str = "grasp_front",
    width: int = 480,
    height: int = 380,
    fps: float = 30.0,
) -> Path:
    """Record several (rollout, task) pairs on a shared clock and tile them.

    Comparison on a shared clock is the point: two grasps shown at their own paces are not
    comparable, and the moment one fails is exactly what the viewer needs to see relative
    to the other.
    """
    import mujoco

    tracks: list[list[np.ndarray]] = []
    for rollout, task in rollouts:
        frames: list[np.ndarray] = []
        model, data = rollout.model, rollout.data
        dt = float(model.opt.timestep)
        stride = max(1, int(round(1.0 / (fps * dt))))
        with Renderer(model, width, height) as renderer:
            renderer.set_camera(camera)
            mujoco.mj_resetData(model, data)
            rollout._apply_initial_state()
            rollout.controller.reset(rollout.ctx, data)
            rollout._apply_preshape_posture()
            mujoco.mj_forward(model, data)
            step = 0
            for stage in task.stages:
                base = rollout._current_wrist()
                for i in range(max(1, int(round(stage.duration / dt)))):
                    rollout._set_wrist(stage.wrist.pose_at(i * dt, base, stage.duration))
                    data.ctrl[:] = rollout.controller.act(rollout.ctx, data, float(data.time), stage.grip)
                    mujoco.mj_step(model, data)
                    step += 1
                    if step % stride == 0:
                        frames.append(renderer.frame(data))
        tracks.append(frames)

    n = max(len(t) for t in tracks)
    # Hold the last frame of a short track rather than truncating the long one: a trial
    # that ended early ended early, and cutting the others would hide why.
    padded = [t + [t[-1]] * (n - len(t)) for t in tracks if t]
    composed = [tile([track[i] for track in padded], columns=len(padded)) for i in range(n)]
    return write_video(composed, path, fps=fps)
