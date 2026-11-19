# Implementation Roadmap

Each milestone ends in a working, tested, usable state. Nothing is a stub waiting for a later
milestone to make it real. The ordering principle is **the platform before the science, and
usability before optimizer count** — a lab that can run one experiment well is worth more than one
that can nearly run six.

---

## M0 — Foundations
`morpholab/core/`, `pyproject.toml`, `tests/`

Registry, deterministic RNG derivation, content-hash IDs, config loading with deep merge and
override syntax, run-directory conventions, worker-safe logging, CLI skeleton, `morpholab doctor`.

**Done when:** `morpholab doctor` reports MuJoCo version, render backend, core count, and package
versions; `pytest` passes; two identical configs hash identically and one differing in any field
does not.

---

## M1 — Morphology generation
`morpholab/mjcf/`, `morpholab/morphology/`

Declarative MJCF element tree; transforms with tested frame conventions; `HandSpec` tree;
frame-based builder producing hand + actuators + tendons + contact classes + `HandIndex`;
`ParamSpace` with encode/decode, canonicalization and feasibility; presets `birh_v1`, `birh_v2`,
`parallel_jaw`.

**Done when:** every preset and 500 random points in the v1 space compile in MuJoCo without warning;
frame tests pass; `encode(decode(x)) == x`; canonicalization maps all `2n_d` symmetric variants of a
layout to one representative; `morpholab morphology show birh_v1` prints the spec and renders a PNG.

---

## M2 — Objects, scenes, tasks
`morpholab/objects/`, `morpholab/sim/scene.py`, `morpholab/tasks/`

Object library: primitives, composites (bottle, mug, hammer, phone, marker, tape, scissors…), YCB
mesh wrappers over the existing `assets/`, categories and named object sets (`core15`, `train`,
`test`). Scene assembly with mocap-welded wrist. Tasks: `lift`, `hold_disturbance`, `pinch`,
`reorient`, plus the AHAP grasp-type suite.

**Done when:** every object compiles, rests stably on the ground under gravity, and reports a
consistent mass/extent; `morpholab objects list` tabulates the library; a scene with any preset hand
and any object compiles and steps.

---

## M3 — Control and rollout
`morpholab/control/`, `morpholab/sim/engine.py`, `trace.py`, `contacts.py`

Controller interface; position, PD, sequential-closure, synergy, tendon controllers. Phase machine,
trace recording with decimation, snapshots, contact extraction, early termination, divergence
guards.

**Done when:** `morpholab run --hand birh_v1 --object sphere_medium --task lift` succeeds, writes a
trace, and is bit-identical across repeated runs with the same seed and across worker counts.

---

## M4 — Metrics
`morpholab/metrics/`

Grasp matrix, friction-cone linearization, GWS hull, ε and volume metrics, isotropy, contact
distribution, disturbance-tolerance bisection, escape probability, failure taxonomy, effort,
complexity cost model.

**Done when:** ε is verified against closed-form cases (an ideal antipodal 2-contact grasp, a
symmetric 3-contact grasp on a sphere), disturbance tolerance is invariant to timestep within its
CI, and every failed trial receives exactly one taxonomy label.

---

## M5 — Experiment management
`morpholab/experiments/`

Job construction and hashing, sweep expansion, serial and multiprocessing runners with per-job
timeouts and isolation, JSONL store with resume, provenance capture, morphology-grouped dispatch.

**Done when:** a 2,000-job sweep runs on 20 workers, is killed halfway, resumes with zero duplicated
work, and produces results independent of worker count.

---

## M6 — Analysis and statistics
`morpholab/analysis/`

Tidy loading, Wilson/bootstrap CIs, paired and unpaired tests with Holm correction and effect sizes,
sensitivity (regression, permutation, Morris, Sobol), Pareto tools, publication tables.

**Done when:** statistical functions match reference values on textbook examples; `morpholab analyze
<run>` emits a table set from stored results alone.

---

## M7 — Visualization
`morpholab/viz/`

Offscreen renderer, overlay stack (contacts, normals, forces, friction cones, COM, frames,
workspace), video with slow motion and captions, side-by-side comparison, figure registry, style,
interactive viewer.

**Done when:** a trial renders to mp4 headlessly with overlays; a success/failure pair renders
side-by-side on a shared clock; every figure regenerates from `results.jsonl` with no simulation.

---

## M8 — Optimization
`morpholab/optimize/`

Optimizer protocol, objective wrapper over the runner, grid, random, CMA-ES, GA, NSGA-II, PSO,
Bayesian; common random numbers, progressive trial counts, per-generation checkpoint and resume.

**Done when:** every optimizer solves a synthetic test objective; CMA-ES beats matched-budget random
search on the real objective; NSGA-II produces a Pareto front with monotonically increasing
hypervolume; a killed optimization resumes from its last generation.

---

## M9 — Reporting and research workflows
`morpholab/report/`, `configs/studies/`

Report generator with morphology cards, metric tables, figures and failure analysis; cross-run trend
summarization; and ready-to-run configs for the five studies in `00-REVIEW.md §7`.

**Done when:** `morpholab report <run>` produces a self-contained document; the Study 1 calibration
config runs end to end and its report answers whether the simulator is trustworthy.

---

## M10 — Expansion (post-platform)

Only after M0–M9 are solid, and each item independently:

- Morphology space v2: tendon routing geometry, compliant palm fields, palm curvature, 4+ digits,
  non-hinge joints, differential mechanisms, per-phalanx tapering.
- RL: Gymnasium-compatible environment over the existing scene/task/controller stack; policy as a
  `Controller`; morphology–control co-optimization (Study 5).
- Arm: re-attach the UR10e for reach-and-manipulate tasks.
- Sim-to-real: parameter identification against the physical prototype; report the transfer gap.

---

## Cross-cutting requirements, enforced from M0

- **Tests alongside code**, not after. Numerical claims get numerical tests.
- **No absolute paths, ever** (review §3.2).
- **No hardcoded joint or geom names outside the builders** (review §3.7).
- **Docstrings cite `02-MATH.md` section numbers** for every non-obvious formula.
- **Every new random draw goes through the seeding scheme**, never `np.random` directly.
- **Headless-first**: no module imports a renderer at import time.
