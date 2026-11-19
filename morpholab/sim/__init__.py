"""Simulation: scene assembly, rollout engine, traces, perturbations."""

from __future__ import annotations

from .contacts import Contact, ContactSet, extract_contacts
from .engine import Rollout, RolloutOptions, run_episode
from .scene import SceneBuild, SceneOptions, approach_pose, build_scene
from .trace import EpisodeTrace, Event, Snapshot, TraceRecorder

__all__ = [
    "build_scene", "SceneBuild", "SceneOptions", "approach_pose",
    "Rollout", "RolloutOptions", "run_episode",
    "EpisodeTrace", "Snapshot", "Event", "TraceRecorder",
    "Contact", "ContactSet", "extract_contacts",
]
