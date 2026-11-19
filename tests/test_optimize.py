"""Optimizers on synthetic objectives, then the real objective on a tiny problem."""

from __future__ import annotations

import json

import numpy as np
import pytest

from morpholab.optimize import (
    CMAES, NSGA2, GeneticAlgorithm, GraspObjective, ObjectiveWeights, RandomSearch, SimpleES,
    aggregate_rows, get_optimizer, optimize, optimizers, pareto_mask,
)
from morpholab.optimize.algorithms import crowding_distance, non_dominated_sort


def sphere(X: np.ndarray, center: float = 0.7) -> np.ndarray:
    return ((np.asarray(X) - center) ** 2).sum(axis=1)


def _run(opt, f, gens):
    for _ in range(gens):
        X = opt.ask()
        opt.tell(X, f(X))
    return opt.best()


def test_registry_lists_every_algorithm():
    assert {"random", "es", "cmaes", "ga", "nsga2"} <= set(optimizers.names())


@pytest.mark.parametrize("name,target", [("es", 0.02), ("cmaes", 0.02), ("ga", 0.1)])
def test_single_objective_optimizers_beat_random_search(name, target):
    dim, gens = 6, 25
    try:
        opt = get_optimizer(name, dim=dim, popsize=12, seed=1)
    except ImportError:
        pytest.skip("cma not installed")
    rnd = RandomSearch(dim, popsize=12, seed=1)
    _, f_opt = _run(opt, sphere, gens)
    _, f_rnd = _run(rnd, sphere, gens)
    assert f_opt[0] < f_rnd[0]
    assert f_opt[0] < target


def test_optimizers_stay_in_the_unit_cube():
    for name in ("es", "ga", "nsga2"):
        opt = get_optimizer(name, dim=4, popsize=10, seed=3, **({"n_obj": 2} if name == "nsga2" else {}))
        for _ in range(5):
            X = opt.ask()
            assert X.shape == (10, 4) and X.min() >= 0.0 and X.max() <= 1.0
            F = np.stack([sphere(X), sphere(X, 0.2)], axis=1) if name == "nsga2" else sphere(X)
            opt.tell(X, F)


def test_state_roundtrip_reproduces_the_next_generation():
    for name in ("random", "es", "ga", "nsga2", "cmaes"):
        try:
            a = get_optimizer(name, dim=3, popsize=8, seed=5, **({"n_obj": 2} if name == "nsga2" else {}))
        except ImportError:
            continue
        X = a.ask()
        F = np.stack([sphere(X), sphere(X, 0.2)], axis=1) if name == "nsga2" else sphere(X)
        a.tell(X, F)
        state = json.loads(json.dumps(a.state()))
        b = get_optimizer(name, dim=3, popsize=8, seed=99, **({"n_obj": 2} if name == "nsga2" else {}))
        b.load_state(state)
        np.testing.assert_allclose(a.ask(), b.ask())


def test_non_dominated_sort_and_crowding():
    F = np.array([[0, 1], [1, 0], [0.5, 0.5], [1, 1], [2, 2]])
    fronts = non_dominated_sort(F)
    assert sorted(fronts[0]) == [0, 1, 2]
    assert fronts[1] == [3] and fronts[2] == [4]
    cd = crowding_distance(F, fronts[0])
    assert np.isinf(cd[0]) and np.isinf(cd[1]) and np.isfinite(cd[2])
    assert pareto_mask(F).tolist() == [True, True, True, False, False]


def test_nsga2_hypervolume_is_monotone_on_a_convex_front():
    def two(X):
        X = np.asarray(X)
        return np.stack([X[:, 0], 1 - np.sqrt(np.clip(X[:, 0], 0, 1)) + ((X[:, 1:] - 0.5) ** 2).sum(axis=1)], axis=1)

    opt = NSGA2(dim=3, popsize=20, seed=2)
    hv = []
    for _ in range(15):
        X = opt.ask()
        opt.tell(X, two(X))
        hv.append(opt.hypervolume(ref=np.array([1.5, 1.5])))
    assert all(b >= a - 1e-12 for a, b in zip(hv, hv[1:]))
    fx, ff = opt.front()
    assert len(ff) >= 5 and np.all(np.diff(ff[:, 0]) >= 0)


def test_aggregate_rows_weights_objects_equally():
    rows = [{"object": "a", "success": True, "grasp_epsilon": 0.3}] * 9 + [{"object": "b", "success": False, "grasp_epsilon": 0.0}]
    agg = aggregate_rows(rows, ObjectiveWeights())
    assert agg["success"] == pytest.approx(0.5)
    assert agg["n_objects"] == 2


# --------------------------------------------------------------------------------------
# the real objective, kept tiny
# --------------------------------------------------------------------------------------

@pytest.fixture
def run_dirs(tmp_path):
    from morpholab.core.paths import RunDirs

    return RunDirs.create("test-optimize", "abcd1234", runs_root=tmp_path)


def test_grasp_objective_evaluates_and_resumes(run_dirs):
    pytest.importorskip("mujoco")
    from morpholab.experiments import ResultStore
    from morpholab.morphology import get_preset, get_space

    space = get_space("three_finger_v1")
    store = ResultStore(run_dirs.results)
    obj = GraspObjective(space=space, store=store, objects=["sphere_medium"], tasks=["close_only"], trials=1,
                         experiment="t", weights=ObjectiveWeights(complexity=0.1))
    x = space.encode(get_preset("birh_v1"))
    F, infos = obj.evaluate(np.array([x]))
    assert F.shape == (1, 1) and np.isfinite(F).all()
    assert infos[0]["valid"] and infos[0]["n_rows"] == 1
    n_before = store.count()
    F2, _ = obj.evaluate(np.array([x]))  # cached: no new trials
    assert store.count() == n_before and F2[0, 0] == pytest.approx(F[0, 0])
    assert store.morphology_ids()


def test_optimize_loop_checkpoints_and_resumes(run_dirs):
    pytest.importorskip("mujoco")
    from morpholab.experiments import ResultStore
    from morpholab.morphology import get_space

    space = get_space("three_finger_layout")
    store = ResultStore(run_dirs.results)
    objective = GraspObjective(space=space, store=store, objects=["sphere_medium"], tasks=["close_only"], trials=1, experiment="t")
    opt = SimpleES(dim=space.dim, popsize=2, seed=0)
    r1 = optimize(space, objective, opt, generations=1, run_dirs=run_dirs)
    assert r1.n_evaluated == 2 and (run_dirs.root / "best_hand.json").exists()
    opt2 = SimpleES(dim=space.dim, popsize=2, seed=0)
    r2 = optimize(space, objective, opt2, generations=2, run_dirs=run_dirs)
    assert r2.n_evaluated == 4  # resumed: one more generation, not two
    hist = (run_dirs.root / "history.jsonl").read_text().splitlines()
    assert len(hist) == 4


def test_multi_objective_mismatch_is_refused(run_dirs):
    from morpholab.experiments import ResultStore
    from morpholab.morphology import get_space

    space = get_space("three_finger_layout")
    objective = GraspObjective(space=space, store=ResultStore(run_dirs.results), multi_objective=True)
    with pytest.raises(ValueError):
        optimize(space, objective, SimpleES(dim=space.dim), generations=1, run_dirs=run_dirs)
