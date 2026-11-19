# Implementation Status and Known Limitations

*Written against the working tree, not against intentions.  A platform whose limits are
undocumented is a platform that produces confident wrong answers.*

---

## 1. What is built and tested

| Milestone | State | Evidence |
|---|---|---|
| **M0** Foundations | complete | `morpholab doctor`; registry, config, seeding, ids, paths, logging; 26 tests |
| **M1** Morphology generation | complete | frame-based builder, 9 presets, 12-parameter search space, canonicalization, validation; 60 tests |
| **M2** Objects, scenes, tasks | complete | 35 objects, 6 named sets, 12 tasks, mocap-welded wrist, FK approach planner; 45 tests |
| **M3** Control and rollout | complete | 5 controllers, phase machine, traces, snapshots, failure taxonomy; 40 tests |
| **M4** Metrics | complete | grasp matrix, Ferrari–Canny ε, GWS volume, isotropy, contact distribution, disturbance bisection, effort, complexity cost model |
| **M5** Experiment management | complete | content-hashed jobs, sweep expansion, parallel runner, JSONL store, resume, provenance; 30 tests |
| **M6** Analysis | core complete | Wilson, cluster bootstrap, paired/unpaired tests, Holm, effect sizes, Pareto front with knee, failure breakdown, power analysis; 20 tests |
| **M7** Visualization | partial | offscreen renderer, camera presets, tiling, episode video, side-by-side. **Not built:** overlay layer (friction cones, force vectors, workspace clouds), figure registry, interactive viewer |
| **M8** Optimization | core complete | random, `es`, CMA-ES, GA, NSGA-II over any `ParamSpace`; objective through the runner and store; per-generation checkpoint and resume; held-out evaluation; `morpholab optimize morphology`. **Not built:** PSO, Bayesian, progressive trial counts |
| **M8b** Learned control | core complete | `PolicyController` (numpy MLP, residual on a closure ramp), OpenAI-style ES training over the process pool, Gymnasium-compatible `GraspEnv`, morphology-control co-design with policy inheritance and fixed-vs-learned rank correlation. **Not built:** PPO/SAC (bring `stable-baselines3`), controller registry entry for `policy` |
| **M8c** Feasibility | complete | material x actuator manufacturability checks (`morpholab.feasibility`), usable as an optimization penalty or a filter |
| **M9** Reporting | **not built** | the analysis functions a report would assemble exist; the generator does not |

237 tests pass. ~13,600 lines of Python.

Run `morpholab doctor` first; `morpholab list` enumerates every registered component.

---

## 2. Known limitations

These are real and they bound what the platform can currently be used to claim.

**2.1 Opposed and anthropomorphic hands underperform, and the approach planner is a
suspect.**
On the `core15` set with a top-down `lift`, `birh_v1` (3 symmetric fingers) reaches ~86 %
while `birh_v2` (2 fingers + thumb) reaches ~25 % and `anthropomorphic_ref` 0 %.  Part of
that is real — closely-spaced digits self-collide, which the failure taxonomy reports —
but the approach planner (`morpholab/sim/approach.py`) models the hand as digits
converging on the palm axis, which describes a symmetric hand well and an opposed hand
badly.  A side approach helps only marginally.

*Consequence:* **do not compare symmetric against opposed morphologies on these numbers.**
Comparisons *within* a family (v1 vs v1-direct, v2 vs v2-without-abduction) pair correctly
and are trustworthy.  Fixing this means a grasp planner that reasons about opposition
rather than convergence, and it is the highest-value next piece of work.

**2.2 Contact forces can exceed what the actuators could produce.**
When an object reacts against the support or wedges inside a concavity, contact forces
reach hundreds of newtons from a 1.2 N·m actuator.  The `force_plausibility` metric
detects this and flags the row (`force_implausible`), so the condition is filterable
rather than invisible — but it is detected, not prevented.

**2.3 The gravity-multiplier metric saturates.**
Most successful grasps hit the 12 g ceiling, so the number does not discriminate.  Raise
`max_multiplier`, or prefer `disturbance_worst`, which does discriminate.

**2.4 The simulator is uncalibrated against hardware.**
The timestep-convergence check has now been run (`configs/overnight/A-convergence-*.yaml`,
2026-09-03): success rates and epsilon for `birh_v1`, `birh_v1_direct` and `two_finger` on
`core15` are unchanged within their confidence intervals across 1, 2 and 4 ms timesteps,
so the default 2 ms is numerically adequate.  Contact parameters, friction coefficients
and actuator limits remain engineering estimates, not measurements; every result is a
statement about this model rather than about hardware.

**2.5 Mesh objects use convex-hull collision.**
YCB wrappers are marked `convex_hull_collision=True` and reported as such.  The composite
objects (mug, tape, scissors, wire) are faithful; the meshes are not.

**2.6 Statistical power is not yet there.**
The pilot sweeps in this repository use 1–2 trials per cell.  `required_n(0.1)` says 393
per group for a 10-point difference at 80 % power.  The ablation configs specify 20–30
trials, which detects roughly a 25-point difference — adequate for large effects, not for
the marginal ones the project cares most about.

**2.7 Single controller.**
Every result uses a scripted closure controller.  Per `docs/00-REVIEW.md` §8, morphology
rankings obtained under one fixed controller are known to be unreliable; Study 5
(co-optimization) exists precisely to quantify how unreliable, and has not been run.

---

## 3. Bugs found and fixed during construction

Recorded because each was a *plausible-output* failure rather than a crash — the class of
bug that quietly corrupts results:

| Bug | Symptom | Fix |
|---|---|---|
| `ResultStore.__len__` made an empty store falsy | `if store:` discarded it; a completed sweep wrote nothing | explicit `__bool__`; `is not None` checks |
| Parallel chunking flattened distinct scene groups | every job in a chunk evaluated against the first job's hand *and object* — a full, plausible, wrong result set | re-group inside the worker |
| Palm placed inside tall objects | object pressed into the table through the wrist weld; 1000 N contact forces | clamp the palm above the object's top |
| Tendon actuator gain was an arbitrary ×200 | grip strength was an artifact of units | dimensionally correct `kp/c²` conversion |
| Actuator torque limit 8 N·m | ~7× a realistic hand-scale drive | 1.2 N·m, documented |
| Moment-arm series decayed across the whole hand in synergy mode | one finger closed, the rest stood still | restart the series per digit |
| `phase_times` evaluated its RHS before `setdefault` | `KeyError` on the first phase | split the statement |
| Fixed 3 cm standoff | a hand with 15 cm fingers and one with 7 cm were compared at completely different grasp geometries | FK-based approach planner with pre-shaping |

---

## 4. The shortest path forward

1. **Run Study 1** (`configs/studies/01-calibration.yaml`). Add the timestep-convergence
   check. Until this exists, nothing else is worth reporting.
2. **Fix the approach planner for opposed hands** (§2.1) — it currently gates the entire
   thumb line of inquiry, which is the project's own central question.
3. **Validate the optimizers on `three_finger_layout`** (two dimensions, grid-searchable)
   before trusting them on the 12-parameter space.
4. **Build `three_finger_v2`** per `docs/05-DESIGN-SPACE.md` §4 -- coupling ratios,
   thumb roll, palm curvature, a fingernail -- which is where hands different in kind
   come from.
5. **Run co-design** on the Pareto front to measure how much the fixed-controller ranking
   moves (`morpholab optimize codesign`).
6. **Build M9** so a run explains itself without anyone writing analysis code.
