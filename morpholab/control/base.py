"""Controller interface.

A controller sees a :class:`ControlContext` -- pre-resolved MuJoCo ids, grouped by
semantic role -- and returns a control vector.  It never string-formats a joint name
(``docs/00-REVIEW.md`` §3.7), and it never inspects the task beyond the ``grip`` word the
stage supplies, so the same controller runs under every task and a learned policy can
replace a scripted one without touching anything else.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol, Sequence, runtime_checkable

import numpy as np

from ..core.registry import Registry
from ..mjcf.model import NameIndex

if TYPE_CHECKING:  # pragma: no cover
    import mujoco

__all__ = ["ControlContext", "Controller", "controllers", "get_controller", "DigitHandles"]

controllers: Registry["Controller"] = Registry("controller")


def get_controller(name: str, **kwargs) -> "Controller":
    return controllers.create(name, **kwargs)


@dataclass(frozen=True)
class DigitHandles:
    """Resolved ids for one digit, so a controller can work per-digit without lookups."""

    name: str
    joint_ids: tuple[int, ...]
    qpos_adr: tuple[int, ...]
    dof_adr: tuple[int, ...]
    pad_geom_ids: tuple[int, ...]
    link_geom_ids: tuple[int, ...]
    actuator_ids: tuple[int, ...]
    """Actuators that drive this digit's flexion. One per joint under per-joint actuation,
    one shared tendon actuator under tendon coupling, and the hand-wide actuator under a
    synergy -- so a controller written against this list works in all three modes."""

    abduction_actuator: int | None = None
    is_thumb: bool = False

    @property
    def n_joints(self) -> int:
        return len(self.joint_ids)


@dataclass
class ControlContext:
    """Everything a controller needs, resolved once when the model is compiled."""

    model: Any
    index: NameIndex
    digits: tuple[DigitHandles, ...]
    flexion_actuator_ids: tuple[int, ...]
    abduction_actuator_ids: tuple[int, ...]
    all_actuator_ids: tuple[int, ...]
    object_geom_ids: frozenset[int]
    pad_geom_ids: frozenset[int]
    hand_geom_ids: frozenset[int]
    palm_body_id: int
    object_body_id: int
    mocap_id: int
    ctrl_open: np.ndarray
    """Control vector for the fully open hand -- per actuator, from its ctrlrange."""

    ctrl_closed: np.ndarray
    timestep: float
    meta: dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def build(model: Any, index: NameIndex, hand_spec: Any) -> "ControlContext":
        """Resolve every name in ``index`` against a compiled model."""
        import mujoco

        def ids(names: Sequence[str], kind: int) -> tuple[int, ...]:
            out = []
            for n in names:
                i = mujoco.mj_name2id(model, kind, n)
                if i < 0:
                    raise KeyError(f"compiled model has no {kind!r} named {n!r}")
                out.append(int(i))
            return tuple(out)

        JOINT = mujoco.mjtObj.mjOBJ_JOINT
        GEOM = mujoco.mjtObj.mjOBJ_GEOM
        ACT = mujoco.mjtObj.mjOBJ_ACTUATOR
        BODY = mujoco.mjtObj.mjOBJ_BODY

        hand_index: NameIndex = index.groups and index  # readability only
        digits: list[DigitHandles] = []
        flexion_acts: list[int] = []
        abduction_acts: list[int] = []

        for digit in hand_spec.digits:
            joint_names = index.group(f"hand.{digit.name}.joints") or index.group(f"{digit.name}.joints")
            pad_names = index.group(f"hand.{digit.name}.pads") or index.group(f"{digit.name}.pads")
            link_names = index.group(f"hand.{digit.name}.links") or index.group(f"{digit.name}.links")
            joint_ids = ids(joint_names, JOINT)

            act_role = _actuator_roles(index, digit, hand_spec)
            act_ids = ids(act_role["flexion"], ACT)
            flexion_acts += [a for a in act_ids if a not in flexion_acts]
            abd_id = ids(act_role["abduction"], ACT)[0] if act_role["abduction"] else None
            if abd_id is not None:
                abduction_acts.append(abd_id)

            digits.append(
                DigitHandles(
                    name=digit.name,
                    joint_ids=joint_ids,
                    qpos_adr=tuple(int(model.jnt_qposadr[j]) for j in joint_ids),
                    dof_adr=tuple(int(model.jnt_dofadr[j]) for j in joint_ids),
                    pad_geom_ids=ids(pad_names, GEOM),
                    link_geom_ids=ids(link_names, GEOM),
                    actuator_ids=act_ids,
                    abduction_actuator=abd_id,
                    is_thumb=digit.is_thumb,
                )
            )

        object_geoms = frozenset(ids(index.group("object_geoms"), GEOM))
        pad_geoms = frozenset(ids(index.group("pads") or index.group("hand.pads"), GEOM))
        hand_geoms = frozenset(ids(index.group("hand_geoms"), GEOM)) if index.group("hand_geoms") else pad_geoms

        palm_name = index.bodies.get("palm") or index.bodies.get("hand.palm")
        obj_name = index.bodies.get("object") or index.bodies.get("object.root")
        palm_id = int(mujoco.mj_name2id(model, BODY, palm_name))
        obj_id = int(mujoco.mj_name2id(model, BODY, obj_name))
        target_id = int(mujoco.mj_name2id(model, BODY, "wrist_target"))
        mocap_id = int(model.body_mocapid[target_id]) if target_id >= 0 else -1

        lo = model.actuator_ctrlrange[:, 0].copy()
        hi = model.actuator_ctrlrange[:, 1].copy()
        # "Open" is the extreme nearest zero flexion, which is the low end for flexion
        # actuators and the *centre* for abduction (whose range straddles zero).
        ctrl_open = lo.copy()
        ctrl_closed = hi.copy()
        for a in abduction_acts:
            ctrl_open[a] = 0.0
            ctrl_closed[a] = 0.0

        return ControlContext(
            model=model, index=index, digits=tuple(digits),
            flexion_actuator_ids=tuple(sorted(set(flexion_acts))),
            abduction_actuator_ids=tuple(sorted(set(abduction_acts))),
            all_actuator_ids=tuple(range(model.nu)),
            object_geom_ids=object_geoms, pad_geom_ids=pad_geoms, hand_geom_ids=hand_geoms,
            palm_body_id=palm_id, object_body_id=obj_id, mocap_id=mocap_id,
            ctrl_open=ctrl_open, ctrl_closed=ctrl_closed,
            timestep=float(model.opt.timestep),
        )

    # -- convenience -------------------------------------------------------------------

    def digit(self, name: str) -> DigitHandles:
        for d in self.digits:
            if d.name == name:
                return d
        raise KeyError(f"no digit named {name!r}")

    @property
    def n_actuators(self) -> int:
        return int(self.model.nu)


def _actuator_roles(index: NameIndex, digit: Any, hand_spec: Any) -> dict[str, list[str]]:
    """Which actuators drive a digit, across all three actuation modes."""
    acts = index.actuators
    flexion: list[str] = []
    abduction: list[str] = []

    for key, name in acts.items():
        base = key.split("hand.", 1)[-1]
        if base == digit.name:                       # tendon-coupled: one per digit
            flexion.append(name)
        elif base == "hand":                          # synergy: one for everything
            flexion.append(name)
        elif base.startswith(f"{digit.name}.") and base.endswith(".flex"):
            flexion.append(name)                      # per-joint
        elif base == f"{digit.name}.abduction":
            abduction.append(name)
    return {"flexion": flexion, "abduction": abduction}


@runtime_checkable
class Controller(Protocol):
    """Produces a control vector each step.

    ``grip`` is the word the current task stage supplies (``open``/``close``/``hold``/
    ``release``).  A controller is free to ignore it -- a learned policy would -- but the
    scripted ones use it as their mode.
    """

    name: str

    def reset(self, ctx: ControlContext, data: Any) -> None: ...

    def act(self, ctx: ControlContext, data: Any, t: float, grip: str) -> np.ndarray: ...

    def state(self) -> dict[str, Any]: ...
