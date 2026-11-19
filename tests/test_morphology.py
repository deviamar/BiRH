"""Tests for morphology specs, generation, search space and validation.

The generator tests assert *behaviour* -- do the fingers converge when flexed, does the
tendon close proximal joints first -- rather than XML text, because the XML is generated
and its exact form is not the contract.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from morpholab.morphology import (
    ActuationMode,
    ActuationSpec,
    DigitSpec,
    HandSpec,
    JointSpec,
    MountSpec,
    PalmSpec,
    build_hand,
    canonicalize_layout,
    get_preset,
    get_space,
    preset_names,
    stick_breaking,
    validate_hand,
)
from morpholab.morphology.space import inverse_stick_breaking

mujoco = pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def v1() -> HandSpec:
    return get_preset("birh_v1")


def _simulate(spec: HandSpec, ctrl_fraction: float, steps: int = 4000):
    """Compile, drive every actuator to a fraction of its range, and settle."""
    build = build_hand(spec, standalone=True)
    model = build.model.compile()
    data = mujoco.MjData(model)
    lo, hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
    data.ctrl[:] = lo + ctrl_fraction * (hi - lo)
    for _ in range(steps):
        mujoco.mj_step(model, data)
    return build, model, data


def _joint_angles(build, model, data, names) -> np.ndarray:
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
    return np.array([data.qpos[model.jnt_qposadr[i]] for i in ids])


def _tip_positions(build, model, data) -> np.ndarray:
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, s)
           for s in build.index.group("fingertip_sites")]
    return np.array([data.site_xpos[i] for i in ids])


# --------------------------------------------------------------------------------------
# spec
# --------------------------------------------------------------------------------------

def test_length_ratios_are_normalized_on_construction():
    d = DigitSpec(name="d", mount=MountSpec(azimuth=0.0), total_length=0.1,
                  length_ratios=(2.0, 1.0, 1.0))
    assert np.isclose(sum(d.length_ratios), 1.0)
    assert np.allclose(d.phalanx_lengths, [0.05, 0.025, 0.025])


def test_digit_rejects_impossible_geometry():
    with pytest.raises(ValueError, match="positive"):
        DigitSpec(name="d", mount=MountSpec(azimuth=0.0), total_length=0.1,
                  length_ratios=(0.5, -0.5, 1.0))
    with pytest.raises(ValueError, match="total_length"):
        DigitSpec(name="d", mount=MountSpec(azimuth=0.0), total_length=0.0)


def test_phalanx_radius_tapers_linearly():
    d = DigitSpec(name="d", mount=MountSpec(azimuth=0.0), total_length=0.1,
                  base_radius=0.012, tip_radius=0.006, length_ratios=(0.4, 0.3, 0.3))
    assert np.isclose(d.phalanx_radius(0), 0.012)
    assert np.isclose(d.phalanx_radius(2), 0.006)
    assert np.isclose(d.phalanx_radius(1), 0.009)


def test_actuator_counts_reflect_the_actuation_mode(v1):
    assert v1.actuation.mode == ActuationMode.TENDON_COUPLED
    assert v1.n_joints == 9 and v1.n_actuators == 3

    direct = get_preset("birh_v1_direct")
    assert direct.n_joints == 9 and direct.n_actuators == 9

    synergy = get_preset("synergy_three")
    assert synergy.n_actuators == 1

    # Abduction always costs its own actuator, even in tendon modes.
    v2 = get_preset("birh_v2")
    assert v2.n_abduction == 1 and v2.n_actuators == v2.n_joints


def test_morphology_id_ignores_name_and_meta_but_not_geometry(v1):
    assert v1.renamed("something_else").morphology_id == v1.morphology_id
    assert v1.with_meta(generation=7).morphology_id == v1.morphology_id

    from dataclasses import replace
    wider = replace(v1, palm=replace(v1.palm, radius=v1.palm.radius + 0.001))
    assert wider.morphology_id != v1.morphology_id


def test_spec_round_trips_through_a_dict(v1):
    assert HandSpec.from_dict(v1.to_dict()).morphology_id == v1.morphology_id


def test_material_overrides_take_precedence_over_the_library(v1):
    from morpholab.core.types import MaterialSpec

    custom = MaterialSpec("pad_rubber", friction=(2.0, 0.02, 0.001))
    hand = v1.with_material("pad_rubber", custom)
    assert hand.material("pad_rubber").friction[0] == 2.0
    assert v1.material("pad_rubber").friction[0] != 2.0


def test_unknown_material_names_are_reported_with_the_options(v1):
    with pytest.raises(KeyError, match="library has"):
        v1.material("unobtainium")


def test_hand_rejects_duplicate_digit_names():
    d = DigitSpec(name="a", mount=MountSpec(azimuth=0.0), total_length=0.1)
    with pytest.raises(ValueError, match="duplicate digit names"):
        HandSpec(name="h", digits=(d, d))


# --------------------------------------------------------------------------------------
# generation
# --------------------------------------------------------------------------------------

@pytest.mark.parametrize("name", preset_names())
def test_every_preset_compiles_and_steps(name):
    spec = get_preset(name)
    build = build_hand(spec, standalone=True)
    assert build.model.validate() == []
    model = build.model.compile()
    data = mujoco.MjData(model)
    for _ in range(500):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))


@pytest.mark.parametrize("name", preset_names())
def test_generated_models_have_no_contacts_at_rest(name):
    """Self-intersecting geometry at the home posture would corrupt every contact metric
    computed from it (``docs/00-REVIEW.md`` §3.6)."""
    build = build_hand(get_preset(name), standalone=True)
    model = build.model.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    contacts = [
        (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
         mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2))
        for c in data.contact[: data.ncon]
    ]
    assert contacts == []


def test_extended_fingertips_lie_on_the_expected_circle(v1):
    build, model, data = _simulate(v1, ctrl_fraction=0.0, steps=1)
    mujoco.mj_forward(model, data)
    tips = _tip_positions(build, model, data)
    expected = v1.palm.radius + v1.digits[0].total_length
    radii = np.hypot(tips[:, 0], tips[:, 1])
    assert np.allclose(radii, expected, atol=1e-3)
    # Three identical digits at 120 degrees: the tips form an equilateral triangle.
    angles = np.sort(np.degrees(np.arctan2(tips[:, 1], tips[:, 0])) % 360)
    assert np.allclose(np.diff(angles), 120.0, atol=1e-3)


def test_flexion_converges_fingertips_toward_the_palm_axis(v1):
    """The behavioural statement of the frame convention: closing must bring the tips
    *inward* and *up*, for every digit, at every azimuth."""
    build, model, data = _simulate(v1, ctrl_fraction=0.0, steps=1)
    mujoco.mj_forward(model, data)
    open_tips = _tip_positions(build, model, data)

    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
           for n in build.index.group("flexion_joints")]
    for i in ids:
        data.qpos[model.jnt_qposadr[i]] = math.radians(90.0)
    mujoco.mj_forward(model, data)
    closed_tips = _tip_positions(build, model, data)

    assert np.all(np.hypot(closed_tips[:, 0], closed_tips[:, 1])
                  < np.hypot(open_tips[:, 0], open_tips[:, 1]))
    assert np.all(closed_tips[:, 2] > open_tips[:, 2])


def test_tendon_coupling_closes_proximal_joints_first(v1):
    """The v1 design notes wanted proximal-before-distal closure and approximated it with
    differential screw friction; the moment-arm ratio produces it exactly
    (``docs/02-MATH.md`` §6.2)."""
    build, model, data = _simulate(v1, ctrl_fraction=0.4)
    angles = _joint_angles(build, model, data, build.index.group(f"{v1.digits[0].name}.joints"))
    limit = v1.digits[0].flexion_range[1]
    assert np.all(angles < 0.95 * limit), "sequencing is only observable before joints saturate"
    assert angles[0] > angles[1] > angles[2] > 0.0
    # The ratio is set by the moment arms, so it is a manufacturable quantity (pulley radii)
    # rather than a tuning parameter (``docs/02-MATH.md`` §6.2).
    expected = v1.actuation.tendon_ratio
    assert angles[1] / angles[0] == pytest.approx(expected, rel=0.25)


def test_identical_digits_close_identically(v1):
    build, model, data = _simulate(v1, ctrl_fraction=0.6)
    per_digit = [
        _joint_angles(build, model, data, build.index.group(f"{d.name}.joints"))
        for d in v1.digits
    ]
    assert np.allclose(per_digit[0], per_digit[1], atol=1e-2)
    assert np.allclose(per_digit[0], per_digit[2], atol=1e-2)


def test_synergy_hand_closes_all_digits_from_one_actuator():
    """A single tendon must not privilege one digit; the moment-arm series restarts per
    digit precisely so that it does not."""
    spec = get_preset("synergy_three")
    build, model, data = _simulate(spec, ctrl_fraction=1.0)
    assert model.nu == 1
    per_digit = [
        _joint_angles(build, model, data, build.index.group(f"{d.name}.joints"))
        for d in spec.digits
    ]
    assert np.allclose(per_digit[0], per_digit[1], atol=2e-2)
    assert np.all(per_digit[0] > math.radians(20.0))


def test_name_index_covers_every_digit_and_joint(v1):
    build = build_hand(v1)
    assert len(build.index.group("flexion_joints")) == v1.n_joints
    assert len(build.index.group("pads")) == v1.n_phalanges
    assert len(build.index.group("actuators")) == v1.n_actuators
    for d in v1.digits:
        assert len(build.index.group(f"{d.name}.joints")) == d.n_phalanges
        assert len(build.index.group(f"{d.name}.pads")) == d.n_phalanges


def test_prefix_lets_two_hands_share_one_scene(v1):
    a = build_hand(v1, prefix="a_")
    b = build_hand(v1, prefix="b_")
    a.model.merge(b.model)  # raises on name collision
    assert a.model.compile().nbody == 2 * 14 - 1


# --------------------------------------------------------------------------------------
# parameter space
# --------------------------------------------------------------------------------------

def test_stick_breaking_maps_the_cube_onto_the_simplex():
    rng = np.random.default_rng(0)
    for _ in range(200):
        u = rng.uniform(0.01, 0.99, size=2)
        r = stick_breaking(u)
        assert len(r) == 3 and np.isclose(sum(r), 1.0) and all(v > 0 for v in r)


def test_stick_breaking_is_invertible():
    for ratios in [(0.44, 0.33, 0.23), (0.6, 0.2, 0.2), (0.34, 0.33, 0.33)]:
        assert np.allclose(stick_breaking(inverse_stick_breaking(ratios)), ratios, atol=1e-9)


def test_space_encode_decode_round_trips_a_preset():
    space = get_space("three_finger_v1")
    base = get_preset("birh_v1")
    x = space.encode(base)
    assert x.shape == (space.dim,) and np.all((x >= 0) & (x <= 1))
    hand = space.decode(x)
    assert np.allclose(space.encode(hand), x, atol=1e-9)
    assert np.isclose(hand.palm.radius, base.palm.radius)
    assert np.allclose(hand.digits[0].length_ratios, base.digits[0].length_ratios, atol=1e-9)


def test_space_decode_is_deterministic():
    space = get_space("three_finger_v1")
    x = np.linspace(0.1, 0.9, space.dim)
    assert space.decode(x).morphology_id == space.decode(x).morphology_id


def test_space_rejects_wrong_length_vectors():
    space = get_space("three_finger_v1")
    with pytest.raises(ValueError, match="expects"):
        space.decode([0.5, 0.5])


def test_sampled_hands_are_feasible_and_compile():
    space = get_space("three_finger_v1")
    rng = np.random.default_rng(3)
    for spec in space.sample_many(rng, 25):
        assert validate_hand(spec).ok
        build_hand(spec).compile()


def test_boolean_parameter_decodes_both_levels():
    space = get_space("three_finger_v1")
    idx = space.names().index("thumb_abduction")
    lo = np.full(space.dim, 0.5)
    lo[idx] = 0.1
    hi = lo.copy()
    hi[idx] = 0.9
    assert space.decode(lo).n_abduction == 0
    assert space.decode(hi).n_abduction == 1


def test_log_scaled_parameters_span_their_decades():
    space = get_space("three_finger_v1")
    p = space.param("joint_stiffness")
    assert np.isclose(p.decode(0.0), p.low) and np.isclose(p.decode(1.0), p.high)
    assert p.decode(0.5) < (p.low + p.high) / 2  # geometric, not arithmetic, midpoint


def test_layout_space_is_two_dimensional_and_matches_the_full_space():
    small = get_space("three_finger_layout")
    assert small.dim == 2
    hand = small.decode([0.5, 0.5])
    assert hand.n_digits == 3


# --------------------------------------------------------------------------------------
# canonicalization
# --------------------------------------------------------------------------------------

def _three_finger(azimuths_deg) -> HandSpec:
    return HandSpec(
        name="t",
        palm=PalmSpec(radius=0.05),
        digits=tuple(
            DigitSpec(name=f"d{i}", mount=MountSpec(azimuth=math.radians(a)), total_length=0.11)
            for i, a in enumerate(azimuths_deg)
        ),
    )


def test_canonicalization_collapses_the_rotation_and_reflection_group():
    """A 3-finger layout has 6 symmetric encodings; without quotienting, five of every six
    optimizer evaluations rediscover an already-evaluated hand
    (``docs/02-MATH.md`` §1.2)."""
    base = [90.0, 190.0, 300.0]
    variants = []
    for rotation in (0.0, 37.0, 180.0, 305.0):
        rotated = [(a + rotation) % 360 for a in base]
        variants.append(rotated)
        variants.append([(180.0 - a) % 360 for a in rotated])  # mirrored

    ids = {canonicalize_layout(_three_finger(v)).morphology_id for v in variants}
    assert len(ids) == 1


def test_canonicalization_is_idempotent():
    hand = canonicalize_layout(_three_finger([10.0, 130.0, 260.0]))
    assert canonicalize_layout(hand).morphology_id == hand.morphology_id


def test_canonicalization_places_the_reference_digit_at_ninety_degrees():
    hand = canonicalize_layout(_three_finger([10.0, 130.0, 260.0]))
    assert np.isclose(math.degrees(hand.digits[0].mount.azimuth), 90.0, atol=1e-6)


def test_canonicalization_does_not_mirror_a_hand_with_a_thumb():
    """A reflected thumb is a genuinely different hand, so the mirror step is skipped."""
    hand = get_preset("birh_v2")
    mirrored = HandSpec(
        name="m", palm=hand.palm, actuation=hand.actuation,
        digits=tuple(
            d.__class__(**{**d.to_dict(), "mount": MountSpec(
                azimuth=math.radians(180.0 - math.degrees(d.mount.azimuth)),
                radius_frac=d.mount.radius_frac, elevation=d.mount.elevation, roll=d.mount.roll,
            )}) if False else d
            for d in hand.digits
        ),
    )
    assert canonicalize_layout(hand).n_thumbs == 1
    assert canonicalize_layout(mirrored).digits[0].is_thumb


# --------------------------------------------------------------------------------------
# validation
# --------------------------------------------------------------------------------------

@pytest.mark.parametrize("name", preset_names())
def test_presets_are_valid(name):
    report = validate_hand(get_preset(name))
    assert report.ok, report.describe()


def test_validation_catches_overlapping_digit_bases():
    hand = _three_finger([90.0, 91.0, 270.0])
    report = validate_hand(hand)
    assert not report.ok and "digit_overlap" in report.codes


def test_validation_catches_a_digit_that_cannot_reach_the_palm_axis():
    hand = HandSpec(
        name="short",
        palm=PalmSpec(radius=0.08),
        digits=(DigitSpec(name="d", mount=MountSpec(azimuth=0.0), total_length=0.03),),
    )
    assert "unopposable_digit" in validate_hand(hand).codes


def test_validation_warns_when_gravity_beats_the_extension_spring():
    """The failure that inverted the closure sequence during development: springs weaker
    than the digit's own weight put resting posture under gravity's control."""
    base = get_preset("birh_v1")
    from dataclasses import replace

    floppy = replace(base, actuation=replace(base.actuation, extension_stiffness=0.001))
    assert "weak_extension_spring" in validate_hand(floppy).codes
    assert validate_hand(floppy).ok  # a warning, not an error


def test_validation_rejects_frictionless_pads():
    from morpholab.core.types import MaterialSpec

    hand = get_preset("birh_v1").with_material(
        "pad_rubber", MaterialSpec("pad_rubber", friction=(0.0, 0.0, 0.0))
    )
    report = validate_hand(hand)
    assert not report.ok and "zero_friction" in report.codes
