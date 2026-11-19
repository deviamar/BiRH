"""Learned control: a policy as a Controller, ES training, a Gymnasium environment, and
morphology-control co-design (``docs/06-OPTIMIZATION-AND-RL.md`` §3-4)."""

from __future__ import annotations

from .codesign import CodesignConfig, codesign
from .controller import ACT_DIM, OBS_DIM, PolicyController, observation
from .env import GraspEnv, make_env
from .es import ESConfig, ESResult, train_policy
from .evaluate import PolicyEvalConfig, RewardWeights, episode_reward, evaluate_population, rollout_policy
from .policy import MLPPolicy

__all__ = [
    "MLPPolicy", "PolicyController", "observation", "OBS_DIM", "ACT_DIM",
    "PolicyEvalConfig", "RewardWeights", "episode_reward", "evaluate_population", "rollout_policy",
    "ESConfig", "ESResult", "train_policy", "GraspEnv", "make_env", "CodesignConfig", "codesign",
]
