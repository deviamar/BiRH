"""Tests for objects, scenes, tasks, controllers and the rollout engine."""

from __future__ import annotations

import math

import numpy as np
import pytest

from morpholab.control import get_controller
from morpholab.core.types import FailureMode, Outcome, Phase, Pose
from morpholab.morphology import get_preset
from morpholab.objects import build_object, get_object, get_object_set, objects
from morpholab.objects.spec import ObjectSpec, Part
from morpholab.sim import Rollout, build_scene
from morpholab.sim.approach import digit_arc, plan_approach
from morpholab.sim.trace import EpisodeTrace, Snapshot
from morpholab.tasks import Orbit, Shake, Translate, get_task, tasks

mujoco = pytest.importorskip("mujoco")


# --------------------------------------------------------------------------------------
# objects
# --------------------------------------------------------------------------------------

@pytest.mark.parametrize("name", objects.names())
def test_every_object_compiles_and_settles(name):
    spec = get_object(name)
    model = build_object(spec, standalone=True).model.compile()
    data = mujoco.MjData(model)
    for _ in range(800):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))


def test_object_mass_is_independent_of_geometry():
    """Part densities are derived so the parts sum to the declared mass -- mass must stay a
    sweepable variable rather than an accident of how many parts an object has."""
    for name in ("mug", "hammer", "tape_roll", "sphere_medium"):
        spec = get_object(name)
        model = build_object(spec, standalone=True).model.compile()
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "obj_root")
        assert model.body_mass[body] == pytest.approx(spec.mass, rel=0.05)


def test_aabb_bounds_rotated_parts():
    upright = ObjectSpec("u", (Part("cylinder", (0.02, 0.05)),))
    assert upright.extents == pytest.approx([0.04, 0.04, 0.10], abs=1e-9)

    from morpholab.mjcf.transforms import quat_from_axis_angle

    lying = ObjectSpec("l", (Part("cylinder", (0.02, 0.05), quat=quat_from_axis_angle((0, 1, 0), math.pi / 2)),))
    assert sorted(lying.extents) == pytest.approx([0.04, 0.04, 0.10], abs=1e-6)


def test_graspable_width_is_the_second_smallest_extent_by_default():
    """For an elongated object the digits must span its diameter, not its length."""
    rod = ObjectSpec("rod", (Part("box", (0.10, 0.01, 0.02)),))
    assert rod.width == pytest.approx(0.04)   # 0.02, 0.04, 0.20 -> second smallest


def test_scaling_preserves_density_by_default():
    base = get_object("sphere_medium")
    big = base.scaled(2.0)
    assert big.mass == pytest.approx(base.mass * 8)
    assert big.width == pytest.approx(base.width * 2)
    same_mass = base.scaled(2.0, keep_density=False)
    assert same_mass.mass == pytest.approx(base.mass)


def test_object_sets_are_disjoint_where_they_claim_to_be():
    train = {o.name for o in get_object_set("train")}
    test = {o.name for o in get_object_set("test")}
    assert not (train & test), "train/test overlap would invalidate held-out reporting"


def test_scaled_object_name_suffix_parses():
    spec = get_object("cube_medium@1.5")
    assert spec.width == pytest.approx(get_object("cube_medium").width * 1.5)


# --------------------------------------------------------------------------------------
# approach planning
# --------------------------------------------------------------------------------------

def test_digit_arc_starts_extended_and_curls_inward():
    hand = get_preset("birh_v1")
    digit = hand.digits[0]
    f, radial, depth = digit_arc(digit, hand.palm.radius)
    assert radial[0] == pytest.approx(hand.palm.radius + digit.total_length, abs=1e-9)
    assert depth[0] == pytest.approx(0.0, abs=1e-12)
    assert radial.min() < radial[0]
    assert depth.max() > 0.0


def test_approach_places_the_object_at_the_convergence_depth():
    hand = get_preset("birh_v1")
    obj = get_object("sphere_medium")
    plan = plan_approach(hand, obj)
    palm_surface = plan.pose.pos[2] - hand.palm.thickness / 2.0
    object_centre = obj.rest_height  # sphere: frame centre is its centre
    assert palm_surface - object_centre == pytest.approx(plan.grasp_depth, abs=1e-6)


def test_approach_preshapes_when_the_closing_arc_would_hit_the_support():
    """A long-fingered hand grasping a flat object on a table must start partly closed, or
    its fingertips sweep below the table before converging."""
    hand = get_preset("birh_v1")
    flat = get_object("phone")
    tall = get_object("bottle")
    assert plan_approach(hand, flat).preshape > 0.3
    assert plan_approach(hand, tall).preshape == pytest.approx(0.0)


def test_approach_scales_with_finger_length():
    """The whole reason the planner exists: a fixed standoff would compare a grasp against
    a near-miss when finger length varies."""
    from dataclasses import replace

    short = get_preset("birh_v1")
    short = replace(short, digits=tuple(replace(d, total_length=0.08) for d in short.digits))
    long_ = get_preset("birh_v1")
    obj = get_object("sphere_medium")
    assert plan_approach(long_, obj).pose.pos[2] > plan_approach(short, obj).pose.pos[2]


def test_top_approach_points_the_palm_downward():
    from morpholab.mjcf.transforms import matrix_from_quat

    plan = plan_approach(get_preset("birh_v1"), get_object("sphere_medium"), direction="top")
    palm_normal = matrix_from_quat(plan.pose.quat)[:, 2]
    assert palm_normal[2] < -0.99


def test_side_approach_points_the_palm_horizontally_inward():
    from morpholab.mjcf.transforms import matrix_from_quat

    plan = plan_approach(get_preset("birh_v1"), get_object("bottle"), direction="side")
    palm_normal = matrix_from_quat(plan.pose.quat)[:, 2]
    assert abs(palm_normal[2]) < 1e-6
    # points back toward the object at the origin
    assert np.dot(palm_normal[:2], np.array(plan.pose.pos[:2])) < 0


# --------------------------------------------------------------------------------------
# scenes
# --------------------------------------------------------------------------------------

def test_scene_compiles_with_a_welded_mocap_wrist():
    scene = build_scene(get_preset("birh_v1"), get_object("sphere_medium"))
    model = scene.compile()
    assert model.neq == 1 and model.nmocap == 1
    data = mujoco.MjData(model)
    for _ in range(400):
        mujoco.mj_step(model, data)
    palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand_palm")
    target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wrist_target")
    assert np.linalg.norm(data.xpos[palm] - data.xpos[target]) < 0.002


def test_scene_name_index_resolves_every_role():
    scene = build_scene(get_preset("birh_v2"), get_object("mug"))
    model = scene.compile()
    for group in ("hand_geoms", "object_geoms", "pads"):
        names = scene.index.group(group)
        assert names, f"group {group} is empty"
        for n in names:
            assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) >= 0


def test_object_too_wide_for_the_hand_is_inadmissible_not_failed():
    """A hopeless pairing must not drag down a morphology's success rate for a reason that
    has nothing to do with its design (``docs/02-MATH.md`` §1.3)."""
    huge = get_object("sphere_large").scaled(3.0)
    scene = build_scene(get_preset("two_finger"), huge)
    assert scene.meta["admissible"] is False
    trace = Rollout(scene, get_task("close_only"), get_controller("ramp")).run()
    assert trace.outcome == Outcome.INADMISSIBLE
    assert trace.failure_mode == FailureMode.UNREACHABLE


# --------------------------------------------------------------------------------------
# tasks
# --------------------------------------------------------------------------------------

@pytest.mark.parametrize("name", tasks.names())
def test_every_task_is_well_formed(name):
    task = get_task(name)
    assert task.duration > 0
    assert task.stages[0].phase == Phase.SETTLE
    assert any(s.snapshot for s in task.stages), "a task must capture a grasp snapshot"
    assert task.to_dict()["name"] == name


def test_translate_motion_has_zero_velocity_at_both_ends():
    """A step command through a compliant weld shakes the object loose on its own."""
    motion = Translate((0.0, 0.0, 0.1))
    base = Pose()
    start = motion.pose_at(0.0, base, 1.0).pos[2]
    early = motion.pose_at(0.02, base, 1.0).pos[2]
    late = motion.pose_at(0.98, base, 1.0).pos[2]
    end = motion.pose_at(1.0, base, 1.0).pos[2]
    assert end == pytest.approx(0.1)
    assert (early - start) < 0.1 * 0.02      # slower than linear at the start
    assert (end - late) < 0.1 * 0.02          # and at the end


def test_orbit_motion_reaches_the_commanded_angle():
    from morpholab.mjcf.transforms import geodesic_angle

    motion = Orbit((1.0, 0.0, 0.0), math.pi)
    final = motion.pose_at(1.0, Pose(), 1.0)
    assert geodesic_angle((1, 0, 0, 0), final.quat) == pytest.approx(math.pi, abs=1e-6)


def test_shake_reports_its_peak_acceleration():
    shake = Shake(amplitude=0.02, frequency=6.0)
    assert shake.peak_acceleration() == pytest.approx(0.02 * (2 * math.pi * 6) ** 2 / 9.81)


def test_randomization_is_reproducible_and_off_by_default():
    task = get_task("lift")
    assert not task.randomize.active
    hand, obj = get_preset("birh_v1"), get_object("sphere_medium")
    a = task.initial_poses(hand, obj, np.random.default_rng(0))
    b = task.initial_poses(hand, obj, np.random.default_rng(0))
    assert a[0].pos == b[0].pos

    randomized = get_task("lift_randomized")
    c = randomized.initial_poses(hand, obj, np.random.default_rng(1))
    d = randomized.initial_poses(hand, obj, np.random.default_rng(2))
    assert c[0].pos != d[0].pos


def test_object_filter_pairs_grasp_types_with_objects_that_afford_them():
    """Scoring a hand on pinching a bottle would be a measurement of the pairing, not the
    hand."""
    pinch = get_task("pinch")
    assert pinch.applies_to(get_object("marker"))
    assert not pinch.applies_to(get_object("bottle"))
    assert get_task("lift").applies_to(get_object("bottle"))


# --------------------------------------------------------------------------------------
# controllers
# --------------------------------------------------------------------------------------

def test_controllers_stay_within_actuator_limits():
    scene = build_scene(get_preset("birh_v1"), get_object("sphere_medium"))
    for name in ("position", "ramp", "sequential", "firm", "gentle"):
        rollout = Rollout(scene, get_task("close_only"), get_controller(name))
        trace = rollout.run()
        lo = rollout.model.actuator_ctrlrange[:, 0]
        hi = rollout.model.actuator_ctrlrange[:, 1]
        assert np.all(rollout.data.ctrl >= lo - 1e-9)
        assert np.all(rollout.data.ctrl <= hi + 1e-9)
        assert trace.steps > 0


def test_ramp_controller_closes_to_a_force_target():
    """Closing to a force rather than to a contact event is what makes the controller
    comparable across morphologies."""
    scene = build_scene(get_preset("birh_v1"), get_object("sphere_medium"))
    gentle = Rollout(scene, get_task("close_only"), get_controller("gentle")).run()
    firm = Rollout(scene, get_task("close_only"), get_controller("firm")).run()
    assert firm.final("normal_force") > gentle.final("normal_force")


# --------------------------------------------------------------------------------------
# engine
# --------------------------------------------------------------------------------------

def test_rollout_is_deterministic():
    scene = build_scene(get_preset("birh_v1"), get_object("sphere_medium"))
    rollout = Rollout(scene, get_task("lift"), get_controller("ramp"))
    a, b = rollout.run(), rollout.run()
    assert a.outcome == b.outcome
    assert a.steps == b.steps
    assert np.allclose(a.get("lift_height"), b.get("lift_height"))


def test_successful_lift_raises_the_object_and_keeps_contact():
    scene = build_scene(get_preset("birh_v1"), get_object("sphere_medium"))
    trace = Rollout(scene, get_task("lift"), get_controller("ramp")).run()
    assert trace.outcome == Outcome.SUCCESS, trace.failure_detail
    assert trace.final("lift_height") > 0.05
    assert trace.final("contact_digits") >= 2


def test_trace_records_phases_snapshots_and_events():
    scene = build_scene(get_preset("birh_v1"), get_object("sphere_medium"))
    trace = Rollout(scene, get_task("lift"), get_controller("ramp")).run()
    assert {str(Phase.SETTLE), str(Phase.CLOSE), str(Phase.LIFT)} <= set(trace.phase_times)
    assert "grasp" in trace.snapshots
    assert trace.events_of("grasp_captured")
    assert trace.at_phase("lift_height", Phase.LIFT).size > 0


def test_snapshot_restores_state_exactly():
    """The property the whole re-probing design depends on: metrics and the disturbance
    battery re-enter the simulator from a snapshot and must land in the same place."""
    scene = build_scene(get_preset("birh_v1"), get_object("sphere_medium"))
    rollout = Rollout(scene, get_task("lift"), get_controller("ramp"))
    trace = rollout.run()
    snap = trace.snapshots["grasp"]

    snap.restore(rollout.model, rollout.data)
    for _ in range(200):
        mujoco.mj_step(rollout.model, rollout.data)
    first = rollout.data.qpos.copy()

    snap.restore(rollout.model, rollout.data)
    for _ in range(200):
        mujoco.mj_step(rollout.model, rollout.data)
    assert np.allclose(first, rollout.data.qpos)


def test_trace_round_trips_through_disk(tmp_path):
    scene = build_scene(get_preset("birh_v1"), get_object("sphere_medium"))
    trace = Rollout(scene, get_task("lift"), get_controller("ramp")).run()
    loaded = EpisodeTrace.load(trace.save(tmp_path / "t.npz"))
    assert loaded.outcome == trace.outcome
    assert np.allclose(loaded.get("lift_height"), trace.get("lift_height"))
    assert np.allclose(loaded.snapshots["grasp"].qpos, trace.snapshots["grasp"].qpos)


def test_slip_is_measured_in_the_palm_frame_not_the_world():
    """A 12 cm lift must not register as 12 cm of slip (``docs/00-REVIEW.md`` §3.3)."""
    scene = build_scene(get_preset("birh_v1"), get_object("sphere_medium"))
    trace = Rollout(scene, get_task("lift"), get_controller("ramp")).run()
    assert trace.final("lift_height") > 0.05
    assert trace.final("slip_distance") < 0.03


def test_failure_modes_are_specific_and_mutually_exclusive():
    scene = build_scene(get_preset("two_finger"), get_object("plate"))
    trace = Rollout(scene, get_task("lift"), get_controller("ramp")).run()
    assert trace.outcome in (Outcome.FAILURE, Outcome.INADMISSIBLE)
    assert trace.failure_mode != FailureMode.NONE
    assert trace.failure_detail


def test_underactuation_outperforms_independent_joints_on_the_core_set():
    """A platform-level regression on the project's own central question: the tendon hand
    and its per-joint twin differ *only* in actuation, so the comparison is clean."""
    tendon, direct = get_preset("birh_v1"), get_preset("birh_v1_direct")
    task, controller = get_task("lift"), get_controller("ramp")
    scores = {}
    for hand in (tendon, direct):
        wins = 0
        for obj in get_object_set("quick"):
            scene = build_scene(hand, obj)
            wins += Rollout(scene, task, controller).run().success
        scores[hand.name] = wins
    assert scores["birh_v1"] >= scores["birh_v1_direct"]
