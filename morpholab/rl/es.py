"""Evolution strategies for policy search (Salimans et al. 2017 style).

Why ES rather than PPO for this platform:

* a trial is 2-4 s of simulation and the reward is mostly terminal (did it lift?), which
  is the regime where ES is competitive with policy-gradient methods and far simpler;
* evaluation parallelizes over the existing process pool with no shared model, no
  vectorized environment and no GPU;
* the policy is a flat vector, so it checkpoints, resumes and inherits across
  morphologies in co-design without any framework.

Antithetic sampling, centered-rank fitness shaping and an Adam step: the standard recipe.
A Gymnasium environment for gradient-based RL is in ``morpholab.rl.env`` for when a
harder task (in-hand reorientation) justifies the extra machinery.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np

from ..core.logging import get_logger
from ..morphology.spec import HandSpec
from .controller import ACT_DIM, OBS_DIM
from .evaluate import PolicyEvalConfig, evaluate_population
from .policy import MLPPolicy

log = get_logger(__name__)

__all__ = ["ESConfig", "ESResult", "train_policy", "centered_ranks"]


@dataclass(frozen=True)
class ESConfig:
    popsize: int = 16
    """Perturbations per iteration; antithetic doubles the evaluations."""

    sigma: float = 0.05
    lr: float = 0.01
    """Adam step. Per-coordinate, so keep it well below ``sigma``; 0.05 diverges."""
    iterations: int = 50
    antithetic: bool = True
    weight_decay: float = 0.005
    init_scale: float = 0.05
    seed: int = 0
    eval_every: int = 5
    """Evaluate the mean policy itself every k iterations (it is what gets deployed)."""

    def to_dict(self) -> dict[str, Any]:
        return dict(vars(self))


@dataclass
class ESResult:
    params: np.ndarray
    """The mean policy at the end of training."""

    best_params: np.ndarray
    best_reward: float
    history: list[dict[str, Any]] = field(default_factory=list)
    policy_dict: dict[str, Any] = field(default_factory=dict)


def centered_ranks(x: np.ndarray) -> np.ndarray:
    """Map rewards to utilities in ``[-0.5, 0.5]`` by rank: robust to reward scale."""
    ranks = np.empty(len(x), float)
    ranks[np.argsort(x)] = np.arange(len(x))
    return ranks / max(len(x) - 1, 1) - 0.5


class _Adam:
    def __init__(self, n: int, lr: float, b1: float = 0.9, b2: float = 0.999, eps: float = 1e-8) -> None:
        self.m, self.v, self.t = np.zeros(n), np.zeros(n), 0
        self.lr, self.b1, self.b2, self.eps = lr, b1, b2, eps

    def step(self, grad: np.ndarray) -> np.ndarray:
        self.t += 1
        self.m = self.b1 * self.m + (1 - self.b1) * grad
        self.v = self.b2 * self.v + (1 - self.b2) * grad * grad
        mh = self.m / (1 - self.b1 ** self.t)
        vh = self.v / (1 - self.b2 ** self.t)
        return self.lr * mh / (np.sqrt(vh) + self.eps)


def train_policy(
    hand: HandSpec,
    objects: Sequence[str],
    tasks: Sequence[str],
    cfg: ESConfig = ESConfig(),
    eval_cfg: PolicyEvalConfig = PolicyEvalConfig(),
    *,
    workers: int = 1,
    init_params: np.ndarray | None = None,
    checkpoint_dir: Path | None = None,
    on_iteration: Callable[[int, dict[str, Any]], None] | None = None,
    evaluate: Callable[[Sequence[np.ndarray]], np.ndarray] | None = None,
) -> ESResult:
    """Train a :class:`MLPPolicy` for ``hand`` by ES. ``evaluate`` can replace the
    simulator (used by the tests with a synthetic objective)."""
    rng = np.random.default_rng(cfg.seed)
    policy = MLPPolicy(OBS_DIM, ACT_DIM, eval_cfg.hidden)
    theta = (policy.init_random(rng, cfg.init_scale).get_flat() if init_params is None
             else np.asarray(init_params, float).copy())
    if theta.size != policy.n_params:
        raise ValueError(f"init_params has {theta.size} entries, policy needs {policy.n_params}")
    adam = _Adam(theta.size, cfg.lr)

    def _eval(pop: Sequence[np.ndarray]) -> np.ndarray:
        if evaluate is not None:
            return np.asarray(evaluate(pop), float)
        return evaluate_population(hand, pop, objects, tasks, eval_cfg, workers=workers, seed=cfg.seed)

    history: list[dict[str, Any]] = []
    best_params, best_reward = theta.copy(), -np.inf
    start_iter = 0
    if checkpoint_dir is not None:
        checkpoint_dir.mkdir(parents=True, exist_ok=True)
        ck = checkpoint_dir / "es_state.npz"
        if ck.exists():
            saved = np.load(ck, allow_pickle=False)
            theta, best_params = saved["theta"], saved["best_params"]
            best_reward, start_iter = float(saved["best_reward"]), int(saved["iteration"])
            adam.m, adam.v, adam.t = saved["adam_m"], saved["adam_v"], int(saved["adam_t"])
            rng = np.random.default_rng(cfg.seed + start_iter)
            log.info("resuming ES from iteration %d (best %.3f)", start_iter, best_reward)

    started = time.perf_counter()
    for it in range(start_iter, cfg.iterations):
        t0 = time.perf_counter()
        eps = rng.standard_normal((cfg.popsize, theta.size))
        if cfg.antithetic:
            eps = np.vstack([eps, -eps])
        pop = [theta + cfg.sigma * e for e in eps]
        if it % cfg.eval_every == 0:
            pop.append(theta.copy())
        rewards = _eval(pop)
        mean_reward = float(rewards[-1]) if it % cfg.eval_every == 0 else float("nan")
        r_pop = rewards[: len(eps)]
        utils = centered_ranks(r_pop)
        grad = (utils[:, None] * eps).sum(axis=0) / (len(eps) * cfg.sigma)
        theta = theta + adam.step(grad) - cfg.lr * cfg.weight_decay * theta

        i_best = int(np.argmax(rewards))
        if rewards[i_best] > best_reward:
            best_reward, best_params = float(rewards[i_best]), np.asarray(pop[i_best]).copy()
        rec = {"iteration": it, "pop_mean": float(r_pop.mean()), "pop_max": float(r_pop.max()),
               "mean_policy": mean_reward, "best": best_reward, "seconds": time.perf_counter() - t0}
        history.append(rec)
        log.info("es it %d/%d  pop mean %.3f  max %.3f  mean-policy %s  best %.3f  (%.0fs)",
                 it + 1, cfg.iterations, rec["pop_mean"], rec["pop_max"],
                 "-" if np.isnan(mean_reward) else f"{mean_reward:.3f}", best_reward, rec["seconds"])
        if on_iteration:
            on_iteration(it, rec)
        if checkpoint_dir is not None:
            np.savez(checkpoint_dir / "es_state.npz", theta=theta, best_params=best_params,
                     best_reward=best_reward, iteration=it + 1, adam_m=adam.m, adam_v=adam.v, adam_t=adam.t)
            with (checkpoint_dir / "es_history.jsonl").open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(rec) + "\n")
            (checkpoint_dir / "best_policy.json").write_text(
                json.dumps(policy.set_flat(best_params).to_dict()), encoding="utf-8")

    log.info("ES finished: best reward %.3f in %.0fs", best_reward, time.perf_counter() - started)
    return ESResult(params=theta, best_params=best_params, best_reward=best_reward, history=history,
                    policy_dict=policy.set_flat(best_params).to_dict())
