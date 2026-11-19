"""``morpholab doctor`` -- verify the environment before a long run fails on job 9,999."""

from __future__ import annotations

import argparse
import importlib
import os
import platform
import sys
import time
from typing import Any

from ..core.paths import asset_root, default_runs_root, project_root

OK, WARN, BAD = "  ok  ", " warn ", " FAIL "

REQUIRED = ["mujoco", "numpy", "scipy", "yaml"]
OPTIONAL = {
    "pandas": "analysis (dataframes, tables)",
    "sklearn": "sensitivity analysis, Bayesian optimization",
    "matplotlib": "figures",
    "imageio": "video export",
    "cma": "CMA-ES optimizer",
    "pytest": "test suite",
}


def add_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--bench", action="store_true", help="benchmark simulation throughput")


def _row(status: str, name: str, detail: str = "") -> None:
    print(f"[{status}] {name:<22} {detail}")


def run(args: argparse.Namespace) -> int:
    failures = 0
    print(f"morpholab environment check\n{'=' * 60}")

    print("\npython")
    _row(OK, "version", sys.version.split()[0])
    _row(OK, "executable", sys.executable)
    _row(OK, "platform", f"{platform.system()} {platform.release()} {platform.machine()}")
    _row(OK, "cpu count", str(os.cpu_count()))

    print("\nrequired packages")
    for name in REQUIRED:
        try:
            mod = importlib.import_module(name)
            _row(OK, name, getattr(mod, "__version__", "?"))
        except ImportError:
            _row(BAD, name, "missing -- pip install morpholab")
            failures += 1

    print("\noptional packages")
    for name, purpose in OPTIONAL.items():
        try:
            mod = importlib.import_module(name)
            _row(OK, name, f"{getattr(mod, '__version__', '?')}  ({purpose})")
        except ImportError:
            _row(WARN, name, f"missing -- disables {purpose}")

    print("\nsimulation")
    try:
        import mujoco

        model = mujoco.MjModel.from_xml_string(_PROBE_XML)
        data = mujoco.MjData(model)
        for _ in range(100):
            mujoco.mj_step(model, data)
        _row(OK, "mujoco physics", f"stepped a {model.nbody}-body probe model")
    except Exception as exc:  # noqa: BLE001
        _row(BAD, "mujoco physics", str(exc)[:80])
        failures += 1
        return _finish(failures)

    backend = _probe_render()
    if backend:
        _row(OK, "render backend", f"{backend} (headless video and figures available)")
    else:
        _row(WARN, "render backend", "none working -- headless rendering disabled, physics unaffected")

    print("\npaths")
    _row(OK, "project root", str(project_root()))
    runs = default_runs_root()
    _row(OK, "runs root", f"{runs} ({'exists' if runs.exists() else 'will be created'})")
    assets = asset_root()
    n_mesh = len(list(assets.glob("*.stl"))) + len(list(assets.glob("*.obj"))) if assets.exists() else 0
    _row(OK if assets.exists() else WARN, "mesh assets", f"{assets} ({n_mesh} meshes)")

    if args.bench:
        print("\nbenchmark")
        for label, xml in (("free sphere", _PROBE_XML), ("contact-rich", _CONTACT_XML)):
            rate = _bench(xml)
            _row(OK, label, f"{rate:,.0f} steps/s  ({rate * 0.002:,.0f}x realtime at dt=2ms)")

    return _finish(failures)


def _finish(failures: int) -> int:
    print(f"\n{'=' * 60}")
    if failures:
        print(f"{failures} required check(s) failed.")
        return 1
    print("environment ready.")
    return 0


def _probe_render() -> str | None:
    """Try each GL backend; return the first that renders a frame."""
    import mujoco

    for backend in ("egl", "osmesa", "glfw"):
        os.environ["MUJOCO_GL"] = backend
        try:
            model = mujoco.MjModel.from_xml_string(_PROBE_XML)
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            renderer = mujoco.Renderer(model, 64, 64)
            renderer.update_scene(data)
            renderer.render()
            renderer.close()
            return backend
        except Exception:  # noqa: BLE001 - probing is expected to fail for absent backends
            continue
    os.environ.pop("MUJOCO_GL", None)
    return None


def _bench(xml: str, steps: int = 20_000) -> float:
    import mujoco

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    for _ in range(200):  # warm up: first steps allocate and populate the contact cache
        mujoco.mj_step(model, data)
    t0 = time.perf_counter()
    for _ in range(steps):
        mujoco.mj_step(model, data)
    return steps / max(time.perf_counter() - t0, 1e-9)


_PROBE_XML = """
<mujoco>
  <option timestep="0.002" integrator="implicitfast"/>
  <worldbody>
    <light pos="0 0 2"/>
    <geom name="floor" type="plane" size="1 1 .1"/>
    <body pos="0 0 .3"><freejoint/><geom type="sphere" size=".05"/></body>
  </worldbody>
</mujoco>
"""

_CONTACT_XML = """
<mujoco>
  <option timestep="0.002" integrator="implicitfast"/>
  <worldbody>
    <light pos="0 0 2"/>
    <geom name="floor" type="plane" size="1 1 .1"/>
    <body pos="0 0 0.06"><freejoint/><geom type="box" size=".05 .05 .05"/></body>
    <body pos="0.02 0 0.17"><freejoint/><geom type="box" size=".05 .05 .05"/></body>
    <body pos="-0.02 0 0.28"><freejoint/><geom type="box" size=".05 .05 .05"/></body>
    <body pos="0 0.02 0.39"><freejoint/><geom type="box" size=".05 .05 .05"/></body>
  </worldbody>
</mujoco>
"""
