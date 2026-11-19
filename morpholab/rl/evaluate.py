"""Evaluate policies on real grasp trials, in parallel.

A policy candidate is a flat parameter vector.  Evaluating a population means running
every candidate on every (object, task) pair; the work is sharded so that each worker
compiles one scene and runs every candidate in its shard against it, because compiling a
scene costs about as much as a trial (``docs/01-ARCHITECTURE.md`` §8).

The reward is computed from the same result row a sweep would write, so a learned
controller is scored by exactly the metrics that score the scripted ones.
"""

from __future__ import annotations

import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

from ..core.logging import get_logger
from ..experiments.sweep import _resolve_objects, _resolve_tasks
from ..morphology.spec import HandSpec

log = get_logger(__name__)

__all__ = ["RewardWeights", "episode_reward", "rollout_policy", "evaluate_population", "PolicyEvalConfig"]


@dataclass(frozen=True)
class RewardWeights:
    """Shaped episode reward from a result row. Success dominates; the rest breaks ties
    and gives the search a gradient before the first success."""

    success: float = 1.0
    lift: float = 0.5
    contact_digits: float = 0.15
    epsilon: float = 0.3
    slip: float = -0.3
    energy: float = -0.02
    eps_ref: float = 0.3
    lift_ref: float = 0.12
    energy_ref: float = 3.0

    def to_dict(self) -> dict[str, float]:
        return dict(vars(self))


def episode_reward(row: dict[str, Any], w: RewardWeights = RewardWeights()) -> float:
    if row.get("outcome") == "error":
        return -1.0
    n_digits = max(int(row.get("n_digits") or 1), 1)
    lift = float(row.get("lift_height") or 0.0)
    return (
        w.success * float(bool(row.get("success")))
        + w.lift * float(np.clip(lift / w.lift_ref, 0.0, 1.0))
        + w.contact_digits * min(float(row.get("contact_digits") or 0.0) / n_digits, 1.0)
        + w.epsilon * min(float(row.get("grasp_epsilon") or 0.0) / w.eps_ref, 1.0)
        + w.slip * min(float(row.get("slip_distance") or 0.0) / 0.05, 1.0)
        + w.energy * min(float(row.get("energy") or 0.0) / w.energy_ref, 1.0)
    )


@dataclass(frozen=True)
class PolicyEvalConfig:
    hidden: tuple[int, ...] = (32,)
    residual: bool = True
    rate: float = 1.5
    trials: int = 1
    scene_options: dict[str, Any] = field(default_factory=dict)
    reward: RewardWeights = field(default_factory=RewardWeights)

    def to_dict(self) -> dict[str, Any]:
        return {"hidden": list(self.hidden), "residual": self.residual, "rate": self.rate,
                "trials": self.trials, "scene_options": dict(self.scene_options), "reward": self.reward.to_dict()}


def _make_controller(params: np.ndarray, cfg: PolicyEvalConfig):
    from .controller import ACT_DIM, OBS_DIM, PolicyController
    from .policy import MLPPolicy

    policy = MLPPolicy(OBS_DIM, ACT_DIM, cfg.hidden).set_flat(params)
    return PolicyController(policy, rate=cfg.rate, residual=cfg.residual)


def rollout_policy(
    hand: HandSpec, obj_name: str, task_name: str, candidates: Sequence[np.ndarray], cfg: PolicyEvalConfig,
    *, seed: int = 0, keep_rows: bool = False,
) -> list[tuple[float, dict[str, Any] | None]]:
    """Run every candidate on one (object, task) scene. Returns ``(reward, row)`` pairs."""
    from ..core.rng import RandomContext
    from ..experiments.runner import _apply_poses
    from ..metrics import MetricContext, base_row, evaluate_metrics
    from ..objects import get_object
    from ..sim import Rollout, build_scene
    from ..sim.scene import SceneOptions
    from ..tasks import get_task

    obj = get_object(obj_name)
    task = get_task(task_name)
    scene = build_scene(hand, obj, options=SceneOptions(**cfg.scene_options) if cfg.scene_options else None,
                        approach=task.approach, standoff=task.standoff)
    rollout = Rollout(scene, task, _make_controller(np.asarray(candidates[0], float), cfg))
    out: list[tuple[float, dict[str, Any] | None]] = []
    for params in candidates:
        rewards, last_row = [], None
        for trial in range(cfg.trials):
            try:
                rollout.controller = _make_controller(np.asarray(params, float), cfg)
                if task.randomize.active or trial > 0:
                    rng = RandomContext([seed, hand.morphology_id, obj_name, task_name, trial])
                    object_pose, wrist_pose = task.initial_poses(hand, obj, rng.stream("init_pose"))
                    _apply_poses(rollout, object_pose, wrist_pose)
                trace = rollout.run()
                ctx = MetricContext(trace=trace, hand=hand, obj=obj, task=task, scene=scene)
                row = {**base_row(ctx), **evaluate_metrics(ctx, include_sim=False)}
                row["trial"] = trial
                rewards.append(episode_reward(row, cfg.reward))
                last_row = row
            except Exception as exc:  # noqa: BLE001 - one bad rollout must not lose the population
                log.warning("policy rollout failed on %s/%s: %s", obj_name, task_name, exc)
                rewards.append(-1.0)
        out.append((float(np.mean(rewards)), last_row if keep_rows else None))
    return out


def _shard_worker(payload: tuple) -> tuple[list[int], str, str, list[float]]:
    hand_dict, obj_name, task_name, idx, params_list, cfg_dict, seed = payload
    hand = HandSpec.from_dict(hand_dict)
    cfg = PolicyEvalConfig(
        hidden=tuple(cfg_dict["hidden"]), residual=cfg_dict["residual"], rate=cfg_dict["rate"],
        trials=cfg_dict["trials"], scene_options=cfg_dict["scene_options"],
        reward=RewardWeights(**cfg_dict["reward"]),
    )
    res = rollout_policy(hand, obj_name, task_name, [np.array(p) for p in params_list], cfg, seed=seed)
    return idx, obj_name, task_name, [r for r, _ in res]


def evaluate_population(
    hand: HandSpec,
    population: Sequence[np.ndarray],
    objects: Sequence[str],
    tasks: Sequence[str],
    cfg: PolicyEvalConfig,
    *,
    workers: int = 1,
    seed: int = 0,
    shard_size: int = 8,
) -> np.ndarray:
    """Mean reward of each candidate over every (object, task) pair. Shape ``(n,)``."""
    obj_names = _resolve_objects(list(objects))
    task_names = _resolve_tasks(list(tasks))
    n = len(population)
    rewards = np.zeros((n, len(obj_names) * len(task_names)))
    pairs = [(o, t) for o in obj_names for t in task_names]
    pair_index = {p: k for k, p in enumerate(pairs)}
    shards = [list(range(i, min(i + shard_size, n))) for i in range(0, n, shard_size)]
    hand_dict, cfg_dict = hand.to_dict(), cfg.to_dict()
    payloads = [
        (hand_dict, o, t, idx, [np.asarray(population[i]).tolist() for i in idx], cfg_dict, seed)
        for (o, t) in pairs for idx in shards
    ]
    started = time.perf_counter()
    if workers <= 1:
        results = [_shard_worker(p) for p in payloads]
    else:
        results = []
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = [pool.submit(_shard_worker, p) for p in payloads]
            for fut in as_completed(futures):
                results.append(fut.result())
    for idx, o, t, rs in results:
        k = pair_index[(o, t)]
        for i, r in zip(idx, rs):
            rewards[i, k] = r
    log.debug("evaluated %d policies x %d scenes in %.1fs", n, len(pairs), time.perf_counter() - started)
    return rewards.mean(axis=1)
