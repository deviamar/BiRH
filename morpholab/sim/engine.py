"""The rollout engine.

Executes a task's stages against a scene, records a trace, and classifies the outcome.
The engine is **task-agnostic and morphology-agnostic**: it reads stage durations, grip
words and wrist motions from the task, and resolves everything else through the scene's
name index.

Object pose error is measured **in the palm frame** throughout (``docs/02-MATH.md`` §5.1).
Measuring in the world frame cannot distinguish "the hand carried the object 12 cm
upward" from "the object slipped 12 cm", and measuring only the palm-to-object *distance*
cannot see an object sliding tangentially around the palm at constant radius -- the flaw
in the original slip test (``docs/00-REVIEW.md`` §3.3).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Sequence

import numpy as np

from ..control.base import ControlContext, Controller
from ..core.logging import get_logger
from ..core.types import Array, FailureMode, Outcome, Phase, Pose
from ..mjcf.transforms import geodesic_angle, matrix_from_quat, quat_conjugate, quat_multiply
from ..tasks.base import Stage, Task
from .contacts import ContactSet, build_geom_lookup, extract_contacts, self_collision_force
from .scene import SceneBuild
from .trace import EpisodeTrace, Snapshot, TraceRecorder

if TYPE_CHECKING:  # pragma: no cover
    import mujoco

log = get_logger(__name__)

__all__ = ["Rollout", "RolloutOptions", "run_episode"]


@dataclass(frozen=True)
class RolloutOptions:
    """Engine settings that are not part of the task or the scene."""

    sample_hz: float = 200.0
    contact_check_hz: float = 100.0
    """How often contacts are extracted. Contact extraction dominates the per-step cost
    of a rollout, and grasp state does not change meaningfully faster than this."""

    lost_grace: float = 0.25
    """Seconds without contact before a stage marked ``abort_if_lost`` gives up. A brief
    contact gap during a lift is normal; a quarter second of it is not."""

    escape_distance: float = 0.35
    """Object distance from the palm at which the trial ends regardless of stage."""

    capture_video: bool = False
    frame_hz: float = 30.0
    max_wall_seconds: float = 60.0
    warn_on_divergence: bool = True


@dataclass
class Rollout:
    """A configured, reusable rollout of one (scene, task, controller) combination.

    The compiled model is kept alive between runs, because compiling a hand costs about
    the same as simulating one trial -- which is why the experiment runner groups jobs by
    morphology before dispatching them (``docs/01-ARCHITECTURE.md`` §8).
    """

    scene: SceneBuild
    task: Task
    controller: Controller
    options: RolloutOptions = field(default_factory=RolloutOptions)

    model: Any = field(default=None, init=False, repr=False)
    data: Any = field(default=None, init=False, repr=False)
    ctx: ControlContext = field(default=None, init=False, repr=False)  # type: ignore[assignment]
    _geom_lookup: dict = field(default_factory=dict, init=False, repr=False)
    _warnings: list[str] = field(default_factory=list, init=False, repr=False)

    def __post_init__(self) -> None:
        import mujoco

        self.model = self.scene.compile()
        self.data = mujoco.MjData(self.model)
        self.ctx = ControlContext.build(self.model, self.scene.index, self.scene.hand)
        self.ctx.meta["preshape"] = float(self.scene.meta.get("preshape", 0.0))
        self.ctx.meta["grasp_depth"] = float(self.scene.meta.get("approach_plan", {}).get("grasp_depth", 0.0))
        self._geom_lookup = build_geom_lookup(self.scene.index, self.scene.hand, self.model)

    # -- main loop ---------------------------------------------------------------------

    def run(self, *, seed_ctx: Any = None, frames: list | None = None) -> EpisodeTrace:
        """Execute the task once and return the trace."""
        import mujoco

        wall_start = time.perf_counter()
        model, data = self.model, self.data
        mujoco.mj_resetData(model, data)
        self._apply_initial_state()
        self.controller.reset(self.ctx, data)
        self._apply_preshape_posture()
        mujoco.mj_forward(model, data)

        rec = TraceRecorder(sample_hz=self.options.sample_hz, timestep=self.ctx.timestep)
        contact_stride = max(1, int(round(1.0 / (self.options.contact_check_hz * self.ctx.timestep))))
        frame_stride = max(1, int(round(1.0 / (self.options.frame_hz * self.ctx.timestep))))

        reference = self._palm_frame_pose()
        object_start_z = float(data.xipos[self.ctx.object_body_id][2])
        lift_reference_z = object_start_z

        contacts = self._contacts()
        state = _RunState(reference=reference, object_start_z=object_start_z)
        aborted = ""
        step_index = 0

        for stage in self.task.stages:
            base_wrist = self._current_wrist()
            rec.enter_phase(str(stage.phase), float(data.time))
            n_steps = max(1, int(round(stage.duration / self.ctx.timestep)))
            stage_t = 0.0

            for _ in range(n_steps):
                self._set_wrist(stage.wrist.pose_at(stage_t, base_wrist, stage.duration))
                data.ctrl[:] = self.controller.act(self.ctx, data, float(data.time), stage.grip)
                mujoco.mj_step(model, data)
                stage_t += self.ctx.timestep
                step_index += 1
                rec.tick()

                if step_index % contact_stride == 0:
                    contacts = self._contacts()
                    state.update_contacts(contacts, float(data.time), rec)

                if rec.should_sample():
                    self._sample(rec, contacts, state, reference, lift_reference_z)

                if self.options.capture_video and frames is not None and step_index % frame_stride == 0:
                    frames.append(float(data.time))

                bad = self._check_divergence()
                if bad:
                    aborted = bad
                    break

                if stage.abort_if_lost and state.lost_for(float(data.time)) > self.options.lost_grace:
                    aborted = "lost"
                    rec.event(float(data.time), "object_lost", phase=str(stage.phase))
                    break

                if self._object_escaped(reference):
                    aborted = "escaped"
                    rec.event(float(data.time), "object_escaped", phase=str(stage.phase))
                    break

                if time.perf_counter() - wall_start > self.options.max_wall_seconds:
                    aborted = "timeout"
                    break

            rec.exit_phase(str(stage.phase), float(data.time))
            if stage.snapshot:
                rec.snapshot(_snapshot_name(stage), Snapshot.capture(
                    model, data, phase=str(stage.phase), label=stage.label))
                rec.contact_sets[_snapshot_name(stage)] = self._contacts()
                state.grasp_contacts = rec.contact_sets[_snapshot_name(stage)]
                if stage.phase == Phase.LIFT or stage.phase == Phase.SETTLE_GRASP:
                    lift_reference_z = float(data.xipos[self.ctx.object_body_id][2])
                rec.event(float(data.time), "grasp_captured",
                          contacts=state.grasp_contacts.count,
                          digits=state.grasp_contacts.n_digits)

            if stage.phase == Phase.SETTLE:
                # Re-reference after settling: the object may have rolled, and lift height
                # must be measured from where it actually was, not where it was dropped.
                lift_reference_z = float(data.xipos[self.ctx.object_body_id][2])
                state.object_start_z = lift_reference_z

            if aborted:
                break

        trace = rec.finish(
            meta=self._meta(),
            duration=float(data.time),
            wall_time=time.perf_counter() - wall_start,
        )
        self._classify(trace, state, aborted, lift_reference_z)
        return trace

    # -- setup -------------------------------------------------------------------------

    def _apply_initial_state(self) -> None:
        """Place the object and the wrist target at the poses the scene was built with."""
        import mujoco

        data = self.data
        if self.ctx.mocap_id >= 0:
            pose = self.scene.initial_wrist
            data.mocap_pos[self.ctx.mocap_id] = pose.pos
            data.mocap_quat[self.ctx.mocap_id] = pose.quat
        mujoco.mj_forward(self.model, data)

    def _apply_preshape_posture(self) -> None:
        """Start the digits at the planned pre-shape rather than letting them snap to it.

        Commanding a pre-shape without setting the matching joint angles would make every
        trial begin with a whip-crack that launches the object -- a simulation artifact
        that would be recorded as a grasp failure.
        """
        import mujoco

        preshape = float(self.ctx.meta.get("preshape", 0.0))
        if preshape <= 0.0:
            return
        for digit_spec, handles in zip(self.scene.hand.digits, self.ctx.digits):
            for joint, adr in zip(digit_spec.flexion_joints, handles.qpos_adr):
                lo, hi = joint.range
                self.data.qpos[adr] = lo + preshape * (hi - lo)
        self.data.ctrl[:] = self.controller.act(self.ctx, self.data, 0.0, "hold")
        mujoco.mj_forward(self.model, self.data)

    def _current_wrist(self) -> Pose:
        if self.ctx.mocap_id < 0:
            return self.scene.initial_wrist
        return Pose.from_arrays(
            self.data.mocap_pos[self.ctx.mocap_id], self.data.mocap_quat[self.ctx.mocap_id]
        )

    def _set_wrist(self, pose: Pose) -> None:
        if self.ctx.mocap_id < 0:
            return
        self.data.mocap_pos[self.ctx.mocap_id] = pose.pos
        self.data.mocap_quat[self.ctx.mocap_id] = pose.quat

    # -- observation --------------------------------------------------------------------

    def _contacts(self) -> ContactSet:
        return extract_contacts(
            self.model, self.data,
            object_geoms=self.ctx.object_geom_ids,
            hand_geoms=self.ctx.hand_geom_ids,
            digit_of_geom=self._geom_lookup,
            object_body=self.ctx.object_body_id,
            palm_body=self.ctx.palm_body_id,
        )

    def _palm_frame_pose(self) -> tuple[Array, Array]:
        """Object pose expressed in the palm frame: ``(position, quaternion)``."""
        data = self.data
        palm_pos = np.asarray(data.xpos[self.ctx.palm_body_id], float)
        palm_quat = np.asarray(data.xquat[self.ctx.palm_body_id], float)
        obj_pos = np.asarray(data.xipos[self.ctx.object_body_id], float)
        obj_quat = np.asarray(data.xquat[self.ctx.object_body_id], float)
        R = matrix_from_quat(palm_quat)
        rel_pos = R.T @ (obj_pos - palm_pos)
        rel_quat = np.asarray(quat_multiply(quat_conjugate(palm_quat), obj_quat), float)
        return rel_pos, rel_quat

    def _palm_frame_error(self, reference: tuple[Array, Array]) -> tuple[float, float]:
        pos, quat = self._palm_frame_pose()
        return (
            float(np.linalg.norm(pos - reference[0])),
            float(geodesic_angle(reference[1], quat)),
        )

    def _object_escaped(self, reference: tuple[Array, Array]) -> bool:
        pos, _ = self._palm_frame_pose()
        return bool(np.linalg.norm(pos) > self.options.escape_distance)

    def _check_divergence(self) -> str:
        data = self.data
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            return "divergence"
        if float(np.max(np.abs(data.qvel))) > 500.0:
            return "divergence"
        return ""

    def _sample(
        self,
        rec: TraceRecorder,
        contacts: ContactSet,
        state: "_RunState",
        reference: tuple[Array, Array],
        lift_reference_z: float,
    ) -> None:
        data = self.data
        slip, rot = self._palm_frame_error(reference)
        obj_pos = np.asarray(data.xipos[self.ctx.object_body_id], float)
        actuator_force = np.abs(data.actuator_force)
        joint_vel = np.array(
            [data.qvel[a] for d in self.ctx.digits for a in d.dof_adr], float
        ) if self.ctx.digits else np.zeros(0)
        joint_torque = np.array(
            [data.qfrc_actuator[a] for d in self.ctx.digits for a in d.dof_adr], float
        ) if self.ctx.digits else np.zeros(0)

        rec.record(
            float(data.time),
            {
                "object_z": float(obj_pos[2]),
                "lift_height": float(obj_pos[2] - lift_reference_z),
                "slip_distance": slip,
                "slip_rotation": rot,
                "contact_count": float(contacts.count),
                "contact_digits": float(contacts.n_digits),
                "normal_force": contacts.total_normal_force,
                "palm_force_fraction": contacts.palm_force_fraction,
                "min_slip_margin": contacts.min_slip_margin,
                "actuator_force_max": float(actuator_force.max()) if actuator_force.size else 0.0,
                "actuator_force_sum": float(actuator_force.sum()),
                "joint_torque_max": float(np.abs(joint_torque).max()) if joint_torque.size else 0.0,
                "mech_power": float(np.abs(joint_torque * joint_vel).sum()) if joint_torque.size else 0.0,
                "object_speed": float(np.linalg.norm(data.cvel[self.ctx.object_body_id, 3:])),
                "self_collision_force": self_collision_force(
                    self.model, data, self.ctx.hand_geom_ids
                ),
            },
        )

    def _meta(self) -> dict[str, Any]:
        return {
            "hand": self.scene.hand.name,
            "morphology_id": self.scene.hand.morphology_id,
            "object": self.scene.obj.name,
            "object_id": self.scene.obj.object_id,
            "task": self.task.name,
            "grasp_type": self.task.grasp_type,
            "controller": getattr(self.controller, "name", type(self.controller).__name__),
            "admissible": self.scene.meta.get("admissible", True),
            "aperture": self.scene.meta.get("aperture"),
            "object_width": self.scene.meta.get("object_width"),
            "timestep": self.ctx.timestep,
            "n_actuators": self.scene.hand.n_actuators,
            "n_joints": self.scene.hand.n_joints,
        }

    # -- outcome -------------------------------------------------------------------------

    def _classify(
        self, trace: EpisodeTrace, state: "_RunState", aborted: str, lift_reference_z: float
    ) -> None:
        """Assign an outcome and, on failure, exactly one taxonomy label.

        Classes are checked in priority order (``docs/02-MATH.md`` §5.4) so failure
        fractions sum to ``1 - success rate`` and can be stacked in a single bar.
        """
        criteria = self.task.criteria
        final_contacts = self._contacts()
        lift = trace.final("lift_height")
        slip = trace.final("slip_distance")
        rot = trace.final("slip_rotation")
        speed = trace.final("object_speed")

        trace.contact_sets.setdefault("final", final_contacts)

        if not self.scene.meta.get("admissible", True):
            trace.outcome = Outcome.INADMISSIBLE
            trace.failure_mode = FailureMode.UNREACHABLE
            trace.failure_detail = (
                f"object width {self.scene.meta['object_width'] * 100:.1f} cm exceeds "
                f"90% of the hand aperture ({self.scene.meta['aperture'] * 100:.1f} cm)"
            )
            return

        if aborted in ("divergence", "timeout"):
            trace.outcome = Outcome.ERROR
            trace.failure_mode = FailureMode.SIM_DIVERGENCE
            trace.failure_detail = f"simulation aborted: {aborted}"
            return

        ok = (
            lift >= criteria.min_lift_height
            and slip <= criteria.max_slip_distance
            and rot <= criteria.max_slip_rotation
            and speed <= criteria.max_object_speed
            and (
                not criteria.require_final_contact
                or (
                    final_contacts.count >= criteria.min_contacts
                    and final_contacts.n_digits >= criteria.min_contact_digits
                )
            )
        )
        if ok:
            trace.outcome = Outcome.SUCCESS
            trace.failure_mode = FailureMode.NONE
            return

        trace.outcome = Outcome.FAILURE
        trace.failure_mode, trace.failure_detail = self._failure_mode(
            trace, state, aborted, final_contacts, lift, slip, rot
        )

    def _failure_mode(
        self,
        trace: EpisodeTrace,
        state: "_RunState",
        aborted: str,
        final: ContactSet,
        lift: float,
        slip: float,
        rot: float,
    ) -> tuple[str, str]:
        criteria = self.task.criteria
        grasp = state.grasp_contacts

        if state.max_contacts <= 1:
            return FailureMode.NO_CONTACT, "the digits never made more than one contact"
        if state.max_contact_digits < criteria.min_contact_digits:
            return (
                FailureMode.INSUFFICIENT_DIGITS,
                f"only {state.max_contact_digits} digit(s) ever contacted the object; "
                f"{criteria.min_contact_digits} required",
            )
        if trace.peak("self_collision_force") > max(trace.peak("normal_force"), 1e-6):
            return (
                FailureMode.SELF_COLLISION,
                "digit-digit contact forces exceeded digit-object forces: the digits "
                "obstruct each other before reaching the object",
            )
        if state.saturated_fraction > 0.9 and (grasp is None or grasp.count < criteria.min_contacts):
            return FailureMode.ACTUATOR_SATURATION, "actuators saturated without achieving a grasp"

        lift_window = trace.at_phase("contact_count", Phase.LIFT)
        if lift_window.size and float(lift_window[-1]) < 1 and lift < criteria.min_lift_height:
            return FailureMode.EJECT_ON_LIFT, "contact was lost during the lift"
        if rot > criteria.max_slip_rotation:
            return (
                FailureMode.ROTATE_OUT,
                f"object rotated {math.degrees(rot):.0f}° in the palm frame "
                f"(limit {math.degrees(criteria.max_slip_rotation):.0f}°)",
            )
        if slip > criteria.max_slip_distance:
            return (
                FailureMode.SLIP_OUT,
                f"object translated {slip * 100:.1f} cm in the palm frame "
                f"(limit {criteria.max_slip_distance * 100:.1f} cm)",
            )
        if any(s.phase == Phase.PERTURB for s in self.task.stages):
            return FailureMode.DISTURBANCE_FAILURE, "grasp survived the lift but failed under disturbance"
        if lift < criteria.min_lift_height:
            return (
                FailureMode.EJECT_ON_LIFT,
                f"object rose only {lift * 100:.1f} cm (needed {criteria.min_lift_height * 100:.1f} cm)",
            )
        return FailureMode.SLIP_OUT, "grasp criteria not met at the end of the trial"


@dataclass
class _RunState:
    """Running aggregates the classifier needs but the trace does not store directly."""

    reference: tuple[Array, Array]
    object_start_z: float
    max_contacts: int = 0
    max_contact_digits: int = 0
    grasp_contacts: ContactSet | None = None
    last_contact_time: float = 0.0
    saturated_steps: int = 0
    total_steps: int = 0

    def update_contacts(self, contacts: ContactSet, t: float, rec: TraceRecorder) -> None:
        if contacts.count > self.max_contacts:
            self.max_contacts = contacts.count
        if contacts.n_digits > self.max_contact_digits:
            self.max_contact_digits = contacts.n_digits
            rec.event(t, "digit_contact", digits=contacts.n_digits, count=contacts.count)
        if contacts.count > 0:
            self.last_contact_time = t

    def lost_for(self, t: float) -> float:
        return t - self.last_contact_time

    @property
    def saturated_fraction(self) -> float:
        return self.saturated_steps / max(self.total_steps, 1)


def _snapshot_name(stage: Stage) -> str:
    return "grasp" if stage.phase in (Phase.SETTLE_GRASP, Phase.CLOSE) else str(stage.phase)


def run_episode(
    scene: SceneBuild,
    task: Task,
    controller: Controller,
    *,
    options: RolloutOptions | None = None,
) -> EpisodeTrace:
    """Convenience wrapper: build a rollout, run it once, return the trace."""
    return Rollout(scene, task, controller, options or RolloutOptions()).run()
