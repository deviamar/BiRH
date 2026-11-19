"""Feasibility checks: every preset assesses, and each check fires on the design that
should trip it."""

from __future__ import annotations

import subprocess
import sys
import pytest

from morpholab.feasibility import ACTUATORS, MATERIALS, assess, feasibility_matrix
from morpholab.morphology import get_preset
from morpholab.morphology.presets import names
from morpholab.morphology.spec import ActuationSpec, DigitSpec, HandSpec, MountSpec, PalmSpec


def _hand(**kw) -> HandSpec:
    base = dict(radius=0.05, thickness=0.022)
    digit_kw = dict(total_length=0.11, length_ratios=(0.44, 0.33, 0.23), base_radius=0.011, tip_radius=0.008)
    digit_kw.update(kw.pop("digit", {}))
    act = kw.pop("actuation", ActuationSpec(mode="per_joint", kp=6.0, force_limit=0.8))
    palm = PalmSpec(**{**base, **kw.pop("palm", {})})
    digits = tuple(DigitSpec(name=f"d{i}", mount=MountSpec(azimuth=i * 2.094), **digit_kw) for i in range(3))
    return HandSpec(name="t", palm=palm, digits=digits, actuation=act)


@pytest.mark.parametrize("preset", names())
def test_every_preset_assesses_with_every_pairing(preset):
    spec = get_preset(preset)
    for m in MATERIALS:
        for a in ACTUATORS:
            rep = assess(spec, material=m, actuator=a)
            assert 0.0 <= rep.score <= 1.0
            assert rep.checks and all(c.status in ("pass", "warn", "fail") for c in rep.checks)
            assert rep.estimates["mass_kg"] > 0


def test_reference_hand_is_buildable_from_pla_with_a_strong_tendon_spool():
    rep = assess(get_preset("birh_v1"), material="pla", actuator="xc330_spool")
    fails = [c.name for c in rep.checks if c.status == "fail"]
    assert not fails, fails
    # ... and the weaker XL330 spool cannot reach the 1.2 N.m the reference hand simulates.
    weak = assess(get_preset("birh_v1"), material="pla", actuator="xl330_spool")
    assert [c.name for c in weak.checks if c.status == "fail"] == ["actuator_torque"]


def test_weak_servo_fails_torque_and_strong_one_passes():
    spec = _hand()
    weak = assess(spec, material="pla", actuator="sg90")
    strong = assess(spec, material="pla", actuator="mg996r")
    assert any(c.name == "actuator_torque" and c.status == "fail" for c in weak.checks)
    assert next(c for c in strong.checks if c.name == "actuator_torque").status == "pass"


def test_thin_fingers_fail_min_feature_and_bending():
    thin = _hand(digit=dict(base_radius=0.0025, tip_radius=0.0015))
    rep = assess(thin, material="pla", actuator="xl330_m288")
    names_failed = {c.name for c in rep.checks if c.status == "fail"}
    assert "tip_radius" in names_failed
    assert "bending_stress" in names_failed


def test_many_actuators_overflow_a_small_palm():
    crowded = _hand(palm=dict(radius=0.025, thickness=0.012),
                    actuation=ActuationSpec(mode="per_joint", kp=6.0, force_limit=0.3))
    rep = assess(crowded, material="pla", actuator="mg996r")
    assert next(c for c in rep.checks if c.name == "palm_volume").status == "fail"


def test_soft_material_switches_to_flexure_checks():
    soft = _hand(digit=dict(stiffness=0.05))
    rep = assess(soft, material="tpu_95a", actuator="xl330_spool")
    kinds = {c.name.split("[")[0] for c in rep.checks}
    assert "flexure_thickness" in kinds and "flexure_strain" in kinds
    assert "bending_stress" not in kinds
    # A silicone finger with the same stiffness needs a flexure thicker than the link allows.
    rep2 = assess(_hand(digit=dict(stiffness=2.0)), material="silicone_ecoflex30", actuator="pneumatic_chamber")
    assert any(c.name.startswith("flexure_thickness") and c.status == "fail" for c in rep2.checks)


def test_pneumatic_actuator_requires_soft_body():
    rep = assess(_hand(), material="pla", actuator="pneumatic_chamber")
    assert any(c.name == "pneumatic_needs_soft" and c.status == "fail" for c in rep.checks)


def test_matrix_is_sorted_by_score_and_covers_all_pairings():
    rows = feasibility_matrix(get_preset("two_finger"))
    assert len(rows) == len(MATERIALS) * len(ACTUATORS)
    scores = [r["score"] for r in rows]
    assert scores == sorted(scores, reverse=True)


def test_cli_runs_on_a_preset(tmp_path):
    out = tmp_path / "f.csv"
    proc = subprocess.run([sys.executable, "-m", "morpholab.feasibility", "birh_v1", "birh_v2",
                           "--actuator", "xl330_spool", "--csv", str(out)], capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, proc.stderr
    assert "birh_v1" in proc.stdout and out.exists()
    assert len(out.read_text().splitlines()) == 3
