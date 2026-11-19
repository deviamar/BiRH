"""Episode traces: the time series, events and snapshots a rollout produces.

Design constraints that shape this module:

* A sweep runs tens of thousands of trials, so a trace must be **cheap** -- samples are
  decimated and stored as float32, and traces are optional to persist at all.
* Metrics must be computable **later**, without re-simulating.  That is what
  :class:`Snapshot` is for: it captures complete simulation state at the moment of grasp,
  and both the analytic grasp metrics and the whole perturbation battery re-enter the
  simulator from it (``docs/01-ARCHITECTURE.md`` §4.3).
* Failures must be **explicable**, so events carry the time and the reason rather than a
  bare boolean.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Sequence

import numpy as np

from ..core.types import Array, FailureMode, Outcome, Phase, Pose

if TYPE_CHECKING:  # pragma: no cover
    import mujoco

__all__ = ["Snapshot", "Event", "EpisodeTrace", "TraceRecorder"]


@dataclass
class Snapshot:
    """Complete simulation state, restorable exactly.

    ``qacc_warmstart`` is included deliberately: MuJoCo's solver warm-starts from the
    previous acceleration, so restoring without it gives a slightly different first step
    and makes a "reset" probe non-reproducible -- the kind of small non-determinism that
    quietly ruins a bisection search.
    """

    time: float
    qpos: Array
    qvel: Array
    act: Array | None
    ctrl: Array
    mocap_pos: Array | None
    mocap_quat: Array | None
    qacc_warmstart: Array
    phase: str = ""
    label: str = ""

    @staticmethod
    def capture(model: "mujoco.MjModel", data: "mujoco.MjData", *, phase: str = "", label: str = "") -> "Snapshot":
        return Snapshot(
            time=float(data.time),
            qpos=data.qpos.copy(),
            qvel=data.qvel.copy(),
            act=data.act.copy() if model.na > 0 else None,
            ctrl=data.ctrl.copy(),
            mocap_pos=data.mocap_pos.copy() if model.nmocap > 0 else None,
            mocap_quat=data.mocap_quat.copy() if model.nmocap > 0 else None,
            qacc_warmstart=data.qacc_warmstart.copy(),
            phase=phase,
            label=label,
        )

    def restore(self, model: "mujoco.MjModel", data: "mujoco.MjData") -> None:
        """Put the simulator back exactly where this snapshot was taken."""
        import mujoco

        data.time = self.time
        data.qpos[:] = self.qpos
        data.qvel[:] = self.qvel
        if self.act is not None and model.na > 0:
            data.act[:] = self.act
        data.ctrl[:] = self.ctrl
        if self.mocap_pos is not None and model.nmocap > 0:
            data.mocap_pos[:] = self.mocap_pos
            data.mocap_quat[:] = self.mocap_quat
        data.qacc_warmstart[:] = self.qacc_warmstart
        # Residual external forces from a previous probe would silently contaminate the
        # next one (``docs/00-REVIEW.md`` §3.3).
        data.xfrc_applied[:] = 0.0
        data.qfrc_applied[:] = 0.0
        data.qacc[:] = 0.0
        mujoco.mj_forward(model, data)


@dataclass(frozen=True)
class Event:
    """Something noteworthy that happened at a moment in time."""

    time: float
    kind: str
    detail: dict[str, Any] = field(default_factory=dict)

    def __str__(self) -> str:
        extra = ", ".join(f"{k}={v}" for k, v in self.detail.items())
        return f"[{self.time:6.3f}s] {self.kind}" + (f" ({extra})" if extra else "")


@dataclass
class EpisodeTrace:
    """One rollout's recorded history."""

    meta: dict[str, Any] = field(default_factory=dict)
    t: Array = field(default_factory=lambda: np.zeros(0, np.float32))
    series: dict[str, Array] = field(default_factory=dict)
    events: list[Event] = field(default_factory=list)
    snapshots: dict[str, Snapshot] = field(default_factory=dict)
    contact_sets: dict[str, Any] = field(default_factory=dict)
    phase_times: dict[str, tuple[float, float]] = field(default_factory=dict)
    outcome: str = Outcome.FAILURE
    failure_mode: str = FailureMode.NONE
    failure_detail: str = ""
    duration: float = 0.0
    steps: int = 0
    wall_time: float = 0.0

    # -- access ------------------------------------------------------------------------

    def get(self, key: str) -> Array:
        try:
            return self.series[key]
        except KeyError:
            raise KeyError(
                f"trace has no series {key!r}; recorded: {sorted(self.series)}"
            ) from None

    def at_phase(self, key: str, phase: Phase | str) -> Array:
        """A series restricted to one phase."""
        name = str(phase)
        if name not in self.phase_times:
            return np.zeros(0)
        lo, hi = self.phase_times[name]
        mask = (self.t >= lo) & (self.t <= hi)
        return self.get(key)[mask]

    def final(self, key: str, default: float = 0.0) -> float:
        arr = self.series.get(key)
        if arr is None or len(arr) == 0:
            return default
        return float(arr[-1])

    def peak(self, key: str, default: float = 0.0) -> float:
        arr = self.series.get(key)
        if arr is None or len(arr) == 0:
            return default
        return float(np.max(np.abs(arr)))

    def events_of(self, kind: str) -> list[Event]:
        return [e for e in self.events if e.kind == kind]

    def first_event_time(self, kind: str) -> float | None:
        for e in self.events:
            if e.kind == kind:
                return e.time
        return None

    @property
    def success(self) -> bool:
        return self.outcome == Outcome.SUCCESS

    # -- persistence -------------------------------------------------------------------

    def save(self, path: str | Path) -> Path:
        """Write to a compressed ``.npz``.

        Snapshots are stored too, so a saved trace can be re-probed with new metrics
        months later without re-running the rollout.
        """
        import json

        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        payload: dict[str, Any] = {"__t": self.t}
        for key, arr in self.series.items():
            payload[f"series__{key}"] = arr
        for name, snap in self.snapshots.items():
            for field_name in ("qpos", "qvel", "ctrl", "qacc_warmstart"):
                payload[f"snap__{name}__{field_name}"] = getattr(snap, field_name)
            for field_name in ("act", "mocap_pos", "mocap_quat"):
                value = getattr(snap, field_name)
                if value is not None:
                    payload[f"snap__{name}__{field_name}"] = value
            payload[f"snapmeta__{name}"] = np.array(
                json.dumps({"time": snap.time, "phase": snap.phase, "label": snap.label})
            )
        payload["__meta"] = np.array(
            json.dumps(
                {
                    "meta": _jsonable(self.meta),
                    "events": [{"time": e.time, "kind": e.kind, "detail": _jsonable(e.detail)} for e in self.events],
                    "phase_times": self.phase_times,
                    "outcome": str(self.outcome),
                    "failure_mode": str(self.failure_mode),
                    "failure_detail": self.failure_detail,
                    "duration": self.duration,
                    "steps": self.steps,
                    "wall_time": self.wall_time,
                }
            )
        )
        np.savez_compressed(p, **payload)
        return p

    @staticmethod
    def load(path: str | Path) -> "EpisodeTrace":
        import json

        raw = np.load(Path(path), allow_pickle=False)
        info = json.loads(str(raw["__meta"]))
        trace = EpisodeTrace(
            meta=info["meta"],
            t=raw["__t"],
            series={k[len("series__"):]: raw[k] for k in raw.files if k.startswith("series__")},
            events=[Event(e["time"], e["kind"], e["detail"]) for e in info["events"]],
            phase_times={k: tuple(v) for k, v in info["phase_times"].items()},
            outcome=info["outcome"],
            failure_mode=info["failure_mode"],
            failure_detail=info.get("failure_detail", ""),
            duration=info["duration"],
            steps=info["steps"],
            wall_time=info.get("wall_time", 0.0),
        )
        names = {k.split("__")[1] for k in raw.files if k.startswith("snap__")}
        for name in names:
            meta = json.loads(str(raw[f"snapmeta__{name}"]))
            trace.snapshots[name] = Snapshot(
                time=meta["time"], phase=meta["phase"], label=meta["label"],
                qpos=raw[f"snap__{name}__qpos"], qvel=raw[f"snap__{name}__qvel"],
                ctrl=raw[f"snap__{name}__ctrl"],
                qacc_warmstart=raw[f"snap__{name}__qacc_warmstart"],
                act=raw.get(f"snap__{name}__act"),
                mocap_pos=raw.get(f"snap__{name}__mocap_pos"),
                mocap_quat=raw.get(f"snap__{name}__mocap_quat"),
            )
        return trace

    def describe(self) -> str:
        lines = [
            f"{self.meta.get('hand', '?')} x {self.meta.get('object', '?')} "
            f"x {self.meta.get('task', '?')}: {self.outcome}"
            + (f" ({self.failure_mode})" if self.failure_mode != FailureMode.NONE else ""),
            f"  {self.steps} steps, {self.duration:.2f} s simulated, {self.wall_time * 1000:.0f} ms wall",
        ]
        if self.failure_detail:
            lines.append(f"  {self.failure_detail}")
        for e in self.events:
            lines.append(f"  {e}")
        return "\n".join(lines)


def _jsonable(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    if isinstance(obj, Pose):
        return obj.to_dict()
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    return str(obj)


class TraceRecorder:
    """Accumulates a trace during a rollout.

    Samples are decimated: at 2 ms steps a 5 s trial is 2,500 steps, and recording every
    one of ~20 series for 20,000 trials would be gigabytes for information no analysis
    uses.  Decimation to ~200 Hz keeps every feature that matters at grasp timescales.
    """

    def __init__(self, *, sample_hz: float = 200.0, timestep: float = 0.002, capacity: int = 4096):
        self.stride = max(1, int(round(1.0 / (sample_hz * timestep))))
        self._t: list[float] = []
        self._series: dict[str, list[float]] = {}
        self.events: list[Event] = []
        self.snapshots: dict[str, Snapshot] = {}
        self.contact_sets: dict[str, Any] = {}
        self.phase_times: dict[str, list[float]] = {}
        self._step = 0

    def should_sample(self) -> bool:
        return self._step % self.stride == 0

    def tick(self) -> None:
        self._step += 1

    def record(self, t: float, values: dict[str, float]) -> None:
        self._t.append(float(t))
        for key, value in values.items():
            series = self._series.setdefault(key, [])
            # Backfill so all series stay the same length even if one starts late.
            if len(series) < len(self._t) - 1:
                series.extend([series[-1] if series else 0.0] * (len(self._t) - 1 - len(series)))
            series.append(float(value))

    def event(self, t: float, kind: str, **detail: Any) -> None:
        self.events.append(Event(float(t), kind, detail))

    def snapshot(self, name: str, snap: Snapshot) -> None:
        self.snapshots[name] = snap

    def enter_phase(self, phase: str, t: float) -> None:
        entry = self.phase_times.setdefault(phase, [t, t])
        entry[0] = min(entry[0], t)

    def exit_phase(self, phase: str, t: float) -> None:
        self.phase_times.setdefault(phase, [t, t])[1] = t

    def finish(self, **kwargs: Any) -> EpisodeTrace:
        n = len(self._t)
        series = {}
        for key, values in self._series.items():
            if len(values) < n:
                values = values + [values[-1] if values else 0.0] * (n - len(values))
            series[key] = np.asarray(values[:n], np.float32)
        return EpisodeTrace(
            t=np.asarray(self._t, np.float32),
            series=series,
            events=list(self.events),
            snapshots=dict(self.snapshots),
            contact_sets=dict(self.contact_sets),
            phase_times={k: (float(v[0]), float(v[1])) for k, v in self.phase_times.items()},
            steps=self._step,
            **kwargs,
        )
