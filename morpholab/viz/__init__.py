"""Visualization: offscreen rendering, overlays, video, publication figures.

Nothing here is imported by the simulation or analysis layers, and no module in this
package is imported at platform start-up -- headless runs never pay for it.
"""

from __future__ import annotations

from .render import CameraSpec, Renderer, render_frame, renderer_available, save_png, tile

__all__ = ["Renderer", "CameraSpec", "render_frame", "renderer_available", "save_png", "tile"]
