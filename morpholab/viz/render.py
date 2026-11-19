"""Offscreen rendering.

Headless is the default everywhere in the platform, so this module never assumes a window
exists.  It probes GL backends once per process and caches the working one; if none works,
:func:`renderer_available` returns ``False`` and callers degrade gracefully rather than
crashing a sweep that only wanted physics.

Overlay layers (contacts, forces, friction cones, frames) live in
:mod:`morpholab.viz.overlays` and are applied to the scene this module produces.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Sequence

import numpy as np

from ..core.logging import get_logger

if TYPE_CHECKING:  # pragma: no cover
    import mujoco

log = get_logger(__name__)

__all__ = ["CameraSpec", "Renderer", "renderer_available", "render_frame", "save_png"]

_BACKENDS = ("egl", "osmesa", "glfw")
_backend_probe: str | None | bool = False  # False = not probed yet


@dataclass(frozen=True)
class CameraSpec:
    """A camera, either by name from the model or as a free tracking camera."""

    name: str | None = None
    distance: float = 0.55
    azimuth: float = 135.0
    elevation: float = -22.0
    lookat: tuple[float, float, float] = (0.0, 0.0, 0.05)
    track_body: str | None = None

    @staticmethod
    def presets() -> dict[str, "CameraSpec"]:
        """Standard viewpoints, so figures across a study frame their subject identically."""
        return {
            "default": CameraSpec(),
            "front": CameraSpec(azimuth=90.0, elevation=-18.0, distance=0.5),
            "side": CameraSpec(azimuth=0.0, elevation=-12.0, distance=0.5),
            "top": CameraSpec(azimuth=90.0, elevation=-80.0, distance=0.45),
            "grasp": CameraSpec(azimuth=125.0, elevation=-28.0, distance=0.36,
                                lookat=(0.0, 0.0, 0.08)),
            "wide": CameraSpec(azimuth=135.0, elevation=-20.0, distance=0.9),
        }


def renderer_available() -> bool:
    """Whether any GL backend in this process can produce a frame."""
    return _probe() is not None


def _probe() -> str | None:
    """Find and cache a working GL backend."""
    global _backend_probe
    if _backend_probe is not False:
        return _backend_probe  # type: ignore[return-value]

    import mujoco

    xml = "<mujoco><worldbody><light pos='0 0 2'/><geom type='plane' size='1 1 .1'/></worldbody></mujoco>"
    chosen = os.environ.get("MUJOCO_GL")
    order = (chosen, *_BACKENDS) if chosen else _BACKENDS
    for backend in order:
        if not backend:
            continue
        os.environ["MUJOCO_GL"] = backend
        try:
            model = mujoco.MjModel.from_xml_string(xml)
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            r = mujoco.Renderer(model, 64, 64)
            r.update_scene(data)
            r.render()
            r.close()
            _backend_probe = backend
            log.debug("render backend: %s", backend)
            return backend
        except Exception as exc:  # noqa: BLE001 - probing absent backends is expected
            log.debug("render backend %s unavailable: %s", backend, exc)
    _backend_probe = None
    log.warning("no working GL backend; rendering disabled (physics is unaffected)")
    return None


class Renderer:
    """A reusable offscreen renderer bound to one compiled model.

    Construction is expensive (it allocates a GL context), so callers should keep one per
    model rather than per frame.  Used as a context manager it always releases the context.
    """

    def __init__(self, model: "mujoco.MjModel", width: int = 960, height: int = 720) -> None:
        import mujoco

        if _probe() is None:
            raise RuntimeError(
                "no working GL backend; install libEGL/OSMesa or run with MUJOCO_GL=osmesa"
            )
        self.model = model
        self.width, self.height = int(width), int(height)
        self._renderer = mujoco.Renderer(model, self.height, self.width)
        self._camera = mujoco.MjvCamera()
        self._options = mujoco.MjvOption()

    # -- configuration ----------------------------------------------------------------

    def set_camera(self, camera: CameraSpec | str | None) -> None:
        import mujoco

        if isinstance(camera, str):
            camera = CameraSpec.presets().get(camera, CameraSpec(name=camera))
        if camera is None:
            camera = CameraSpec()

        if camera.name is not None:
            cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera.name)
            if cam_id >= 0:
                self._camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
                self._camera.fixedcamid = cam_id
                return
            log.debug("no camera named %r in the model; using a free camera", camera.name)

        self._camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self._camera.distance = camera.distance
        self._camera.azimuth = camera.azimuth
        self._camera.elevation = camera.elevation
        self._camera.lookat[:] = camera.lookat
        if camera.track_body:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, camera.track_body)
            if bid >= 0:
                self._camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                self._camera.trackbodyid = bid

    def set_flags(self, **flags: bool) -> None:
        """Toggle MuJoCo visualization flags by name, e.g. ``contactforce=True``.

        Accepts the ``mjVIS_*`` and ``mjRND_*`` short names without their prefixes, so
        callers write ``transparent=True`` rather than looking up an enum.
        """
        import mujoco

        for key, value in flags.items():
            vis = getattr(mujoco.mjtVisFlag, f"mjVIS_{key.upper()}", None)
            if vis is not None:
                self._options.flags[vis] = bool(value)
                continue
            rnd = getattr(mujoco.mjtRndFlag, f"mjRND_{key.upper()}", None)
            if rnd is not None:
                self._renderer.scene.flags[rnd] = bool(value)
                continue
            raise ValueError(f"unknown visualization flag {key!r}")

    def set_geom_groups(self, groups: Sequence[int]) -> None:
        """Show only the listed geom groups (2 = structure, 3 = pads, 4 = sites)."""
        for g in range(len(self._options.geomgroup)):
            self._options.geomgroup[g] = 1 if g in groups else 0

    # -- rendering ---------------------------------------------------------------------

    def frame(
        self,
        data: "mujoco.MjData",
        *,
        camera: CameraSpec | str | None = None,
        decorate: Any = None,
    ) -> np.ndarray:
        """Render one RGB frame as ``uint8[H, W, 3]``.

        ``decorate`` is an optional callable ``(scene, model, data) -> None`` that appends
        decorative geoms; this is the hook the overlay layer uses.
        """
        if camera is not None:
            self.set_camera(camera)
        self._renderer.update_scene(data, self._camera, self._options)
        if decorate is not None:
            decorate(self._renderer.scene, self.model, data)
        return self._renderer.render().copy()

    def close(self) -> None:
        try:
            self._renderer.close()
        except Exception:  # noqa: BLE001 - teardown races in some GL drivers
            pass

    def __enter__(self) -> "Renderer":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def render_frame(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    *,
    camera: CameraSpec | str | None = "default",
    width: int = 960,
    height: int = 720,
    decorate: Any = None,
) -> np.ndarray:
    """One-shot render. Convenient, but allocates a GL context each call -- use
    :class:`Renderer` for sequences."""
    with Renderer(model, width, height) as r:
        return r.frame(data, camera=camera, decorate=decorate)


def save_png(image: np.ndarray, path: str | Path) -> Path:
    """Write an RGB array to a PNG."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    try:
        import imageio.v3 as iio

        iio.imwrite(p, np.ascontiguousarray(image))
    except ImportError:
        from matplotlib import pyplot as plt

        plt.imsave(p, np.ascontiguousarray(image))
    return p


def tile(images: Sequence[np.ndarray], columns: int | None = None, pad: int = 4) -> np.ndarray:
    """Arrange frames into a grid, for comparison figures.

    Frames of differing sizes are padded rather than resized, so nothing is distorted.
    """
    if not images:
        raise ValueError("no images to tile")
    n = len(images)
    cols = columns or int(np.ceil(np.sqrt(n)))
    rows = int(np.ceil(n / cols))
    h = max(im.shape[0] for im in images)
    w = max(im.shape[1] for im in images)
    canvas = np.full((rows * (h + pad) + pad, cols * (w + pad) + pad, 3), 20, np.uint8)
    for i, im in enumerate(images):
        r, c = divmod(i, cols)
        y = pad + r * (h + pad)
        x = pad + c * (w + pad)
        canvas[y : y + im.shape[0], x : x + im.shape[1]] = im
    return canvas
