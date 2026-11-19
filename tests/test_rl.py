"""Policy, controller, ES, environment and co-design plumbing."""

from __future__ import annotations

import numpy as np
import pytest

from morpholab.rl import (
    ACT_DIM, OBS_DIM, ESConfig, MLPPolicy, PolicyController, PolicyEvalConfig, RewardWeights,
    episode_reward, evaluate_population, rollout_policy, train_policy,
)
from morpholab.rl.es import centered_ranks

pytest.importorskip("mujoco")


def test_policy_shapes_and_roundtrip():
    pol = MLPPolicy(OBS_DIM, ACT_DIM, (16,)).init_random(np.random.default_rng(0))
    a = pol(np.zeros(OBS_DIM))
    assert a.shape == (ACT_DIM,) and np.all(np.abs(a) <= 1.0)
    pol2 = MLPPolicy.from_dict(pol.to_dict())
    np.testing.assert_allclose(pol2(np.ones(OBS_DIM)), pol(np.ones(OBS_DIM)))
    with pytest.raises(ValueError):
        pol.set_flat(np.zeros(3))


def test_centered_ranks_are_scale_free():
    r = centered_ranks(np.array([10.0, -3.0, 4.0]))
    assert r.tolist() == [0.5, -0.5, 0.0]
    np.testing.assert_allclose(centered_ranks(np.array([1e6, -1e6, 0.0])), r)


def test_episode_reward_orders_outcomes():
    good = {"success": True, "lift_height": 0.12, "contact_digits": 3, "n_digits": 3, "grasp_epsilon": 0.3, "slip_distance": 0.0, "energy": 1.0}
    meh = {"success": False, "lift_height": 0.02, "contact_digits": 2, "n_digits": 3, "grasp_epsilon": 0.05, "slip_distance": 0.02, "energy": 1.0}
    bad = {"outcome": "error"}
    assert episode_reward(good) > episode_reward(meh) > episode_reward(bad)


def test_zero_policy_in_residual_mode_grasps_like_the_scripted_baseline():
    """At initialization the residual controller must behave like a closure ramp, so ES
    starts from a grasp rather than from nothing."""
    from morpholab.morphology import get_preset

    hand = get_preset("birh_v1")
    zero = np.zeros(MLPPolicy(OBS_DIM, ACT_DIM, (32,)).n_params)
    res = rollout_policy(hand, "sphere_medium", "lift", [zero], PolicyEvalConfig(), keep_rows=True)
    reward, row = res[0]
    assert row is not None and row["contact_digits"] >= 2
    assert reward > 0.3


def test_policy_controller_reset_and_act_shapes():
    from morpholab.morphology import get_preset
    from morpholab.objects import get_object
    from morpholab.sim import Rollout, build_scene
    from morpholab.tasks import get_task

    hand = get_preset("birh_v2")  # has an abduction actuator
    task = get_task("close_only")
    scene = build_scene(hand, get_object("sphere_medium"), approach=task.approach)
    ctl = PolicyController(MLPPolicy(OBS_DIM, ACT_DIM, (8,)).init_random(np.random.default_rng(1)))
    rollout = Rollout(scene, task, ctl)
    trace = rollout.run()
    assert trace.steps > 0
    u = ctl.act(rollout.ctx, rollout.data, 0.0, "close")
    assert u.shape == (rollout.model.nu,)
    lo, hi = rollout.model.actuator_ctrlrange[:, 0], rollout.model.actuator_ctrlrange[:, 1]
    assert np.all(u >= lo - 1e-9) and np.all(u <= hi + 1e-9)


def test_evaluate_population_parallel_matches_serial():
    from morpholab.morphology import get_preset

    hand = get_preset("birh_v1")
    rng = np.random.default_rng(0)
    n = MLPPolicy(OBS_DIM, ACT_DIM, (8,)).n_params
    pop = [rng.standard_normal(n) * 0.05 for _ in range(3)]
    cfg = PolicyEvalConfig(hidden=(8,))
    serial = evaluate_population(hand, pop, ["sphere_medium"], ["close_only"], cfg, workers=1)
    parallel = evaluate_population(hand, pop, ["sphere_medium"], ["close_only"], cfg, workers=2, shard_size=2)
    assert serial.shape == (3,)
    np.testing.assert_allclose(serial, parallel)


def test_es_improves_a_synthetic_objective_and_checkpoints(tmp_path):
    from morpholab.morphology import get_preset

    hand = get_preset("birh_v1")
    n = MLPPolicy(OBS_DIM, ACT_DIM, (4,)).n_params
    target = np.random.default_rng(7).standard_normal(n) * 0.1

    def synthetic(pop):
        return np.array([-np.sum((np.asarray(p) - target) ** 2) for p in pop])

    cfg = ESConfig(popsize=16, sigma=0.05, lr=0.01, iterations=6, seed=0, eval_every=2)
    ev = PolicyEvalConfig(hidden=(4,))
    r1 = train_policy(hand, [], [], cfg, ev, evaluate=synthetic, checkpoint_dir=tmp_path)
    first, last = r1.history[0]["pop_mean"], r1.history[-1]["pop_mean"]
    assert last > first
    assert (tmp_path / "es_state.npz").exists() and (tmp_path / "best_policy.json").exists()
    r2 = train_policy(hand, [], [], ESConfig(**{**cfg.to_dict(), "iterations": 8}), ev, evaluate=synthetic, checkpoint_dir=tmp_path)
    assert [h["iteration"] for h in r2.history] == [6, 7]


def test_grasp_env_steps_to_the_end_of_the_task():
    from morpholab.rl import make_env

    env = make_env("birh_v1", "sphere_medium", "close_only", frame_skip=20)
    obs, info = env.reset(seed=0)
    assert obs.shape == (OBS_DIM,) and np.isfinite(obs).all()
    total, steps, truncated = 0.0, 0, False
    while not truncated and steps < env.max_steps + 5:
        obs, r, terminated, truncated, info = env.step(np.zeros(ACT_DIM))
        total += r
        steps += 1
        assert np.isfinite(obs).all()
        if terminated:
            break
    assert truncated and "contact_digits" in info
    env.close()


def test_codesign_runs_one_tiny_generation(tmp_path):
    from morpholab.core.paths import RunDirs
    from morpholab.morphology import get_space
    from morpholab.optimize import SimpleES
    from morpholab.rl import CodesignConfig, codesign

    space = get_space("three_finger_layout")
    dirs = RunDirs.create("test-codesign", "deadbeef", runs_root=tmp_path)
    cfg = CodesignConfig(objects=["sphere_medium"], tasks=["close_only"], generations=1,
                         inner=ESConfig(iterations=1, popsize=2, eval_every=1),
                         eval=PolicyEvalConfig(hidden=(4,)), workers=1)
    summary = codesign(space, SimpleES(dim=space.dim, popsize=2, seed=0), cfg, dirs)
    assert summary["n_hands"] >= 1
    assert (dirs.root / "history.jsonl").exists() and (dirs.root / "best_policy.json").exists()
