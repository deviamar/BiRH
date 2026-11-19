"""Tests for the foundation layer: registry, ids, seeding, config, types."""

from __future__ import annotations

import numpy as np
import pytest

from morpholab.core.config import Config, ConfigError, config_from_dict, deep_merge, load_config, parse_overrides
from morpholab.core.ids import canonical_json, content_hash, short_id
from morpholab.core.registry import Registry, RegistryError
from morpholab.core.rng import RandomContext, fibonacci_sphere, seed_from, spherical_directions
from morpholab.core.types import FailureMode, MaterialSpec, Phase, SolverSpec


# --------------------------------------------------------------------------------------
# registry
# --------------------------------------------------------------------------------------

def test_registry_register_and_create():
    reg = Registry[str]("thing")

    @reg.register("a", doc="First thing", tags=["x"])
    def _a(n: int = 1) -> str:
        return "a" * n

    assert reg.names() == ["a"]
    assert reg.create("a", n=3) == "aaa"
    assert "a" in reg and len(reg) == 1
    assert reg.get("a").summary == "First thing"
    assert reg.names(tag="x") == ["a"] and reg.names(tag="y") == []


def test_registry_rejects_duplicates_unless_allowed():
    reg = Registry[int]("thing")
    reg.add("a", lambda: 1)
    with pytest.raises(RegistryError, match="already registered"):
        reg.add("a", lambda: 2)

    loose = Registry[int]("thing", allow_override=True)
    loose.add("a", lambda: 1)
    loose.add("a", lambda: 2)
    assert loose.create("a") == 2


def test_registry_suggests_close_names():
    reg = Registry[int]("thing")
    reg.add("sphere_medium", lambda: 1)
    with pytest.raises(RegistryError, match="did you mean 'sphere_medium'"):
        reg.create("sphere_mediun")


# --------------------------------------------------------------------------------------
# content ids
# --------------------------------------------------------------------------------------

def test_hash_is_key_order_independent():
    assert content_hash({"a": 1, "b": 2}) == content_hash({"b": 2, "a": 1})


def test_hash_distinguishes_values_and_collapses_negative_zero():
    assert content_hash({"a": 1.0}) != content_hash({"a": 1.0000001})
    assert content_hash(-0.0) == content_hash(0.0)
    assert content_hash([1, 2]) == content_hash((1, 2))  # tuple/list are the same content


def test_hash_handles_numpy_and_nested_structures():
    a = content_hash({"v": np.array([1.0, 2.0]), "n": np.float64(3.0)})
    b = content_hash({"v": [1.0, 2.0], "n": 3.0})
    assert a == b


def test_hash_ignores_float_noise_below_precision():
    # Values differing far below FLOAT_DIGITS must not change the id, or YAML round-trips
    # would silently invalidate resume.
    assert content_hash(0.1 + 0.2) == content_hash(0.3)


def test_canonical_json_is_deterministic_text():
    payload = {"b": [3, 1], "a": {"z": 1, "y": 2}}
    assert canonical_json(payload) == '{"a":{"y":2,"z":1},"b":[3,1]}'
    assert len(short_id(payload)) == 8


# --------------------------------------------------------------------------------------
# seeding
# --------------------------------------------------------------------------------------

def test_seed_is_stable_across_calls():
    assert seed_from("exp", 3, "sphere") == seed_from("exp", 3, "sphere")
    assert seed_from("exp", 3) != seed_from("exp", 4)


def test_random_context_streams_are_reproducible_and_independent():
    a = RandomContext(["exp", 7]).stream("init_pose").normal(size=8)
    b = RandomContext(["exp", 7]).stream("init_pose").normal(size=8)
    assert np.allclose(a, b)

    ctx = RandomContext(["exp", 7])
    assert not np.allclose(ctx.stream("init_pose").normal(size=8), ctx.stream("domain").normal(size=8))


def test_stream_values_do_not_depend_on_creation_order():
    """The property that lets the framework gain new randomized elements without
    invalidating existing runs."""
    first = RandomContext(["e"])
    a_then_b = (first.stream("a").normal(size=4), first.stream("b").normal(size=4))

    second = RandomContext(["e"])
    b_val = second.stream("b").normal(size=4)
    a_val = second.stream("a").normal(size=4)

    assert np.allclose(a_then_b[0], a_val)
    assert np.allclose(a_then_b[1], b_val)


def test_fibonacci_sphere_is_unit_and_well_spread():
    pts = fibonacci_sphere(200)
    assert pts.shape == (200, 3)
    assert np.allclose(np.linalg.norm(pts, axis=1), 1.0)
    # A well-spread set has a near-zero resultant; clustered samples would not.
    assert np.linalg.norm(pts.mean(axis=0)) < 0.02


def test_spherical_directions_in_six_dimensions_requires_a_seed():
    rng = np.random.default_rng(0)
    dirs = spherical_directions(32, dim=6, rng=rng)
    assert dirs.shape == (32, 6)
    assert np.allclose(np.linalg.norm(dirs, axis=1), 1.0)
    with pytest.raises(ValueError, match="seeded rng"):
        spherical_directions(4, dim=6)


# --------------------------------------------------------------------------------------
# config
# --------------------------------------------------------------------------------------

def test_deep_merge_merges_maps_and_replaces_lists():
    base = {"a": {"x": 1, "y": 2}, "l": [1, 2, 3]}
    over = {"a": {"y": 9}, "l": [7]}
    assert deep_merge(base, over) == {"a": {"x": 1, "y": 9}, "l": [7]}


def test_parse_overrides_builds_nested_typed_values():
    out = parse_overrides(["sim.timestep=0.001", "n=5", "flag=true", "objs=[a,b]"])
    assert out == {"sim": {"timestep": 0.001}, "n": 5, "flag": True, "objs": ["a", "b"]}
    with pytest.raises(ConfigError):
        parse_overrides(["nope"])


def test_config_interpolation_preserves_type_and_supports_embedding():
    cfg = config_from_dict({"a": 0.5, "b": "${a}", "c": "value is ${a}"})
    assert cfg.get("b") == 0.5 and isinstance(cfg.get("b"), float)
    assert cfg.get("c") == "value is 0.5"


def test_config_interpolation_detects_cycles():
    with pytest.raises(ConfigError, match="did not converge|cannot interpolate"):
        config_from_dict({"a": "${b}", "b": "${a}"})


def test_config_missing_key_raises_with_suggestions():
    cfg = config_from_dict({"sim": {"timestep": 1, "integrator": "implicitfast"}})
    with pytest.raises(ConfigError, match="available near"):
        cfg.get("sim.tiemstep")
    assert cfg.get("sim.tiemstep", 42) == 42
    assert "sim.timestep" in cfg and "sim.nope" not in cfg


def test_config_hash_depends_only_on_content():
    a = config_from_dict({"x": 1, "y": {"z": 2}})
    b = config_from_dict({"y": {"z": 2}, "x": 1})
    c = config_from_dict({"x": 1, "y": {"z": 3}})
    assert a.hash == b.hash != c.hash


def test_config_extends_chain_and_cycle_detection(tmp_path):
    (tmp_path / "base.yaml").write_text("sim:\n  timestep: 0.002\n  integrator: implicitfast\n")
    (tmp_path / "study.yaml").write_text("extends: base.yaml\nsim:\n  timestep: 0.001\nname: s\n")
    cfg = load_config(tmp_path / "study.yaml")
    assert cfg.get("sim.timestep") == 0.001
    assert cfg.get("sim.integrator") == "implicitfast"  # inherited
    assert cfg.get("name") == "s"

    (tmp_path / "loop_a.yaml").write_text("extends: loop_b.yaml\n")
    (tmp_path / "loop_b.yaml").write_text("extends: loop_a.yaml\n")
    with pytest.raises(ConfigError, match="circular"):
        load_config(tmp_path / "loop_a.yaml")


def test_config_round_trips_through_yaml(tmp_path):
    cfg = config_from_dict({"sim": {"timestep": 0.002}, "objects": ["a", "b"]})
    path = cfg.save(tmp_path / "c.yaml")
    assert load_config(path).hash == cfg.hash


# --------------------------------------------------------------------------------------
# types
# --------------------------------------------------------------------------------------

def test_solver_spec_rejects_timestep_that_would_destabilize_contact():
    spec = SolverSpec(time_const=0.002)
    assert spec.check_stable(0.0005)[0] is True
    ok, msg = spec.check_stable(0.005)
    assert ok is False and "below 2*timestep" in msg


def test_solver_spec_emits_valid_mjcf_attribute_strings():
    spec = SolverSpec.compliant()
    assert len(spec.solref.split()) == 2
    assert len(spec.solimp.split()) == 5


def test_failure_mode_priority_covers_every_real_mode():
    ordered = FailureMode.priority()
    assert FailureMode.NONE not in ordered
    assert set(ordered) == set(FailureMode) - {FailureMode.NONE}
    assert len(ordered) == len(set(ordered))


def test_material_with_friction_is_a_copy():
    mat = MaterialSpec("pad", (1.0, 0.01, 0.001))
    hi = mat.with_friction(1.5)
    assert hi.friction[0] == 1.5 and mat.friction[0] == 1.0
    assert hi.friction[1:] == mat.friction[1:]


def test_phase_and_outcome_serialize_as_strings():
    assert Phase.CLOSE == "close"
    assert canonical_json({"p": Phase.LIFT}) == '{"p":"lift"}'
