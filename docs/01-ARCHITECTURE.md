# Morphology Lab — Software Architecture

*The computational laboratory that serves the research agenda in `00-REVIEW.md`.*

---

## 1. Design principles

1. **The morphology is data, not code.** A hand is a serializable `HandSpec`. Nothing downstream may
   hardcode a joint name, a digit count, or a geometry.
2. **One direction of dependency.** `core → mjcf → {morphology, objects} → sim → metrics →
   experiments → {optimize, analysis} → {viz, report} → cli`. No upward imports, ever. This is what
   makes components replaceable.
3. **Every component is a registry entry.** Controllers, objects, tasks, metrics, optimizers,
   perturbations and figures are all looked up by string name from a registry, so a config file can
   name any of them and a user can add one without touching the framework.
4. **Headless is the default; visualization is an optional decorator.** No module imports a renderer
   at module scope. A rollout that produces video is the *same* rollout with a recorder attached.
5. **Determinism is structural, not aspirational.** Seeds derive from content hashes, so a job's
   result depends only on its configuration — never on execution order, worker count, or wall time.
6. **A result is a row.** Every trial emits one flat, self-describing record. Analysis is then plain
   dataframe work, and any experiment ever run remains queryable.
7. **Fail loudly at build time.** Morphology validation, XML compilation, and config parsing raise
   before a single physics step is taken. A 10,000-job sweep must not discover a typo on job 9,999.

---

## 2. Layer map

```
┌────────────────────────────────────────────────────────────────────────────┐
│  cli/            morpholab run | sweep | optimize | analyze | render | ... │
├────────────────────────────────────────────────────────────────────────────┤
│  report/         markdown+html reports, figure embedding, trend summaries  │
│  viz/            offscreen render, overlays, video, publication figures    │
│  analysis/       stats, CIs, significance, sensitivity, Pareto, tables     │
│  optimize/       grid | random | CMA-ES | GA | NSGA-II | PSO | BO          │
├────────────────────────────────────────────────────────────────────────────┤
│  experiments/    job graph, parallel runner, checkpoint/resume, store      │
├────────────────────────────────────────────────────────────────────────────┤
│  metrics/        grasp quality | contact | robustness | effort | complexity│
├────────────────────────────────────────────────────────────────────────────┤
│  sim/            scene assembly, rollout engine, phases, traces, perturb   │
│  control/        Controller interface + implementations                    │
│  tasks/          Task interface: staging, success criteria, randomization  │
├────────────────────────────────────────────────────────────────────────────┤
│  morphology/     HandSpec, ParamSpace, builder, presets, validation        │
│  objects/        ObjectSpec, library, builder, mesh import                 │
├────────────────────────────────────────────────────────────────────────────┤
│  mjcf/           declarative MJCF element tree, transforms, quaternions    │
├────────────────────────────────────────────────────────────────────────────┤
│  core/           registry, rng, ids, config, paths, logging, types         │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Package layout

```
morpholab/
  core/
    registry.py      Registry[T]: decorator-based name → factory, with listing/help
    rng.py           SeedSequence-based deterministic RNG derivation
    ids.py           canonical JSON hashing → stable content IDs
    config.py        dataclass ⟷ YAML/JSON, deep-merge, ${var} interpolation, overrides
    paths.py         run directory conventions
    logging.py       structured logging, worker-safe
    types.py         shared type aliases and small value objects
  mjcf/
    element.py       MJCF element tree (attrs, children, merge, to_xml)
    model.py         MjcfModel: root + named-entity index + compile()
    transforms.py    quaternions, rotation matrices, frames, axis-angle
  morphology/
    spec.py          HandSpec / PalmSpec / DigitSpec / PhalanxSpec / JointSpec / ActuationSpec
    space.py         ParamSpace: bounds, categoricals, encode/decode, canonicalization
    builder.py       HandSpec → MjcfModel (+ HandIndex: names of joints/acts/pads/tips)
    presets.py       birh_v1, birh_v2, parallel_jaw baseline, human_ref
    validate.py      geometric/physical feasibility checks
  objects/
    spec.py          ObjectSpec: geometry tree, physical props, metadata
    library.py       primitives, composites, YCB mesh wrappers, categories, sets
    builder.py       ObjectSpec → MJCF body
  tasks/
    base.py          Task interface, TaskStage, SuccessCriteria
    grasp.py         Lift, HoldUnderDisturbance, Pinch, Reorient, PickAndPlace
    ahap.py          AHAP-derived grasp-type suite
  control/
    base.py          Controller interface
    position.py      PositionController, PDController
    closure.py       SequentialClosureController, SynergyClosureController
    tendon.py        TendonClosureController (underactuated)
    scheduled.py     phase-scheduled wrapper
  sim/
    scene.py         Scene assembly: world + hand + object + mocap wrist + sensors
    engine.py        Rollout: phase machine, stepping, callbacks, recorders
    trace.py         EpisodeTrace: time series + events + snapshots
    perturb.py       perturbation battery, bisection search, randomization
    contacts.py      contact extraction into a stable, name-resolved structure
  metrics/
    base.py          Metric interface + registry
    grasp.py         grasp matrix, friction cone, GWS, ε-metric, volume, isotropy
    contact.py       count, spread, opposition, palm involvement
    robustness.py    disturbance tolerance, escape probability, failure taxonomy
    effort.py        energy, peak torque, joint utilization, saturation
    complexity.py    actuator/joint/mechanism cost model
    aggregate.py     per-trial → per-cell aggregation with bootstrap CIs
  experiments/
    job.py           Job: (morphology, object, task, controller, seed, overrides) → id
    runner.py        serial + multiprocessing execution, retries, timeouts
    store.py         JSONL append store, parquet/CSV export, resume by id set
    sweep.py         config → job list (grid, list, zip, sampled)
  optimize/
    base.py          Optimizer ask/tell, Objective, Problem
    grid.py random_search.py cmaes.py ga.py nsga2.py pso.py bayes.py
    objective.py     morphology → fitness via the experiment runner
  analysis/
    load.py          store → dataframe, tidy schema
    stats.py         Wilson, bootstrap BCa, Mann-Whitney, Welch, Holm, effect sizes
    sensitivity.py   standardized regression, permutation importance, Morris, Sobol
    pareto.py        non-dominated sorting, hypervolume, front extraction
    tables.py        publication tables (markdown/latex/csv)
  viz/
    style.py         a single figure style; colors, fonts, sizes
    render.py        offscreen renderer + overlay layers
    overlays.py      contacts, normals, forces, friction cones, COM, frames, workspace
    video.py         frame recorder → mp4/gif, slow motion, side-by-side
    figures.py       registered figure builders (curves, boxes, corr, Pareto, importance)
    compare.py       morphology comparison grids, success-vs-failure pairs
    interactive.py   passive viewer with live overlays and frame stepping
  report/
    generator.py     experiment → markdown/html report with embedded figures
    trends.py        cross-experiment trend summarization
  cli.py             argparse-based entry point, subcommands
```

---

## 4. Core data model

### 4.1 `HandSpec`

```python
HandSpec
├── name: str
├── palm: PalmSpec
│     radius, thickness, curvature, mass,
│     material: MaterialSpec (friction, solref, solimp)  ← palm compliance lives here
├── digits: list[DigitSpec]
│     ├── name, is_thumb
│     ├── mount: MountSpec (azimuth, radius_frac, elevation, roll)   ← the frame, see §5
│     ├── total_length, length_ratios (simplex), radius_profile
│     ├── joints: list[JointSpec] (type, axis_local, range, stiffness, damping, armature)
│     ├── abduction: JointSpec | None                                ← the disputed DOF
│     └── pad: PadSpec (thickness, coverage, material)
├── actuation: ActuationSpec
│     mode ∈ {per_joint, tendon_coupled, synergy}
│     gains, force limits, coupling coefficients
└── meta: dict (provenance, parent genome id, generation, …)
```

`HandSpec` is a frozen dataclass tree, round-trips through JSON/YAML, and hashes to a stable
`morphology_id`. A hand is thus *citable*: any figure, video, or result row can name the exact hand
that produced it, and that hand can be rebuilt byte-identically.

### 4.2 `ParamSpace` — the search space

`ParamSpace` is the bridge between a `HandSpec` tree and the flat vectors optimizers want.

- A list of `Param(name, low, high, kind ∈ {continuous, integer, categorical, boolean}, log)`.
- `encode(spec) → x ∈ [0,1]^n`, `decode(x) → HandSpec` — via explicit getter/setter paths into the
  spec tree, so adding a searchable parameter is one line.
- **Canonicalization** (`canonicalize(spec)`): removes symmetry-induced degeneracy — rotate digit
  azimuths so digit 0 is at 0, sort the rest, renormalize length ratios onto the simplex. Without
  this, a 3-finger layout has a 6-fold redundancy (3 rotations × reflection) and every optimizer
  wastes its budget rediscovering the same hand.
- **Constraints**: `feasible(spec) → (bool, reasons)`. Adjacent digits must not interpenetrate at
  the base; total finger length must exceed palm radius by enough to oppose; ratios must be positive.
  Infeasible points are rejected before simulation and reported to the optimizer as a penalty, never
  silently clipped.

The **v1 space is deliberately small**, per the project constraint:

| Parameter | Range | Why it is in v1 |
|---|---|---|
| `digit_azimuth[1]`, `digit_azimuth[2]` | 40°–180° | The original open question |
| `palm_radius` | 0.03–0.08 m | Sets opposition geometry |
| `finger_length` | 0.07–0.16 m | Object-size matching |
| `ratio_proximal`, `ratio_middle` | simplex | Tests the human-proportion assumption (A3/A4) |
| `flexion_range_max` | 60°–140° | Enclosure capability |
| `joint_stiffness`, `joint_damping` | log scale | Passive compliance |
| `pad_friction` | 0.4–1.6 | Cheapest real design lever |
| `thumb_abduction` | bool + range | v2's central claim (A6) |

Deferred to v2 of the space, once v1 is validated: tendon routing geometry, per-phalanx radii, palm
curvature and compliance fields, 4+ digits, non-hinge joints, differential mechanisms.

### 4.3 `EpisodeTrace`

One rollout produces one trace:

```
EpisodeTrace
├── meta: job id, morphology id, object, task, controller, seeds, git sha, versions
├── time series (float32 arrays, decimated): t, qpos, qvel, ctrl, actuator_force,
│      object_pose (palm frame), palm_pose, contact_count, total_normal_force, energy
├── events: [(t, kind, payload)]  — contact_made, phase_change, slip, saturation, failure
├── snapshots: {phase_name → full sim state}  — enables replay, re-probing, video without re-running
└── contacts_at: {phase_name → list[Contact]}  — for analytic grasp metrics
```

Traces are optional to persist (they are large); metrics are always persisted. Snapshots are the key
design decision: **the analytic grasp metrics and the perturbation battery both re-enter the sim
from the `grasp` snapshot**, so evaluation is separable from the rollout and can be re-run later
with new metrics without re-simulating the approach.

### 4.4 `TrialResult` — the row

Flat dict: identifiers + all scalar metrics + failure classification + timing. Appended as one JSONL
line. This is the only durable output that analysis reads.

---

## 5. The frame convention (the fix for §3.1 of the review)

All morphology geometry derives from **one** rotation per digit. The palm is a cylinder about `+z`;
fingers curl toward `+z`.

For a digit mounted at azimuth θ, elevation φ, roll ψ:

```
R_mount = Rz(θ) · Ry(−φ) · Rx(ψ)
```

giving the mount frame basis (columns of `R_mount`):

- `x̂` — radially outward: the direction the digit extends
- `ŷ` — tangential: the flexion axis (negated so positive joint angle curls toward the palm normal)
- `ẑ` — palm normal at ψ=φ=0: the direction fingertips converge

Inside that frame **every phalanx is identical and trivial**:

```xml
<body pos="L_prev 0 0">
  <joint type="hinge" axis="0 -1 0" range="..."/>
  <geom type="capsule" fromto="0 0 0  L 0 0" size="r"/>
  <geom name="...pad" type="box" pos="L/2 0 r" size="L/2 r t"/>
</body>
```

There is no per-part trigonometry. Abduction, when present, is a joint with `axis="0 0 1"` inserted
before the first flexion joint. `transforms.py` is unit-tested against known frames (θ=0 gives the
identity; θ=90° maps `x̂→ŷ`; a flexion of +90° maps a fingertip from the palm plane onto the palm
normal), so a sign error fails a test rather than silently producing a hand that cannot close.

---

## 6. Simulation

### 6.1 Scene assembly

`Scene = world ⊕ hand ⊕ object ⊕ wrist ⊕ sensors`, composed as MJCF element trees and compiled once.

- **Wrist**: the palm carries a free joint and is welded to a **mocap body**. Commanding the mocap
  pose moves the hand dynamically (through the weld's compliance) rather than kinematically, so lift
  and shake tests exert real forces on the grasp. No arm is required for morphology studies; a
  `UR10eMount` is a drop-in alternative for manipulation tasks later.
- **Contact filtering**: digit-internal adjacent-body pairs are excluded via `contype/conaffinity`
  classes; digit-to-digit and digit-to-object collisions are kept (self-collision *is* a
  morphological failure mode and must remain observable).
- **Naming discipline**: `HandIndex` returned by the builder maps semantic roles → MuJoCo names and
  ids (`index.pads[digit][phalanx]`, `index.flexion_joints`, `index.object_geoms`). Nothing outside
  the builder ever string-formats a MuJoCo name.

### 6.2 Rollout phase machine

```
SETTLE → APPROACH → CLOSE → SETTLE_GRASP → LIFT → HOLD → PERTURB → done
```

Each phase has a duration, a controller mode, an abort predicate, and an on-exit hook (snapshot,
contact capture, metric probe). Tasks configure the phase list; the engine is task-agnostic. Early
termination on unrecoverable failure (object ejected, sim divergence) saves the bulk of the compute
in a large sweep, because most bad morphologies fail in the first second.

### 6.3 Determinism

```
seed(job) = SHA256(experiment_id ‖ morphology_id ‖ object ‖ task ‖ controller ‖ trial_index)
```

fed into `numpy.random.SeedSequence`, which spawns independent child streams for *initial pose*,
*domain randomization*, *perturbation directions*, and *actuator noise*. Streams are independent, so
adding a new randomized element does not perturb existing streams — a run from last month remains
byte-reproducible after the framework gains a feature.

---

## 7. Metrics

Two families, always reported together (per §8 of the review — their disagreement is informative):

**Analytic** (from the `grasp` snapshot, no extra simulation): grasp matrix G, friction-cone
primitive wrenches, GWS convex hull, Ferrari–Canny ε, GWS volume, force-closure boolean, grasp
isotropy σ_min/σ_max, contact count, contact spread, opposition index, palm involvement.

**Empirical** (extra simulation from the snapshot): disturbance tolerance via bisection on constant
wrenches over a Fibonacci-sampled direction set; escape probability under randomized perturbation;
maximum survived impulse; shake-test amplitude; gravity multiplier at failure; energy, peak torque,
joint utilization; task success.

**Structural** (from the spec alone, free): DOF count, actuator count, joint count, digit count,
mechanism penalties → a scalar `complexity` with a documented, editable cost model.

Every metric is a registry entry with declared inputs (`spec` / `trace` / `snapshot` / `sim`), so the
engine knows which metrics need a live simulator and can batch them.

---

## 8. Experiments

```
config.yaml  →  SweepSpec  →  [Job, …]  →  Runner  →  Store(JSONL)  →  DataFrame
                                  │
                                  └── job_id = hash(job content)
```

- **Resume** is set difference: read completed `job_id`s from the store, skip them. Killing and
  restarting a 20,000-job sweep is free and safe.
- **Parallelism** via `multiprocessing` with a worker pool; each worker compiles its own MuJoCo model
  and reuses it across jobs with the same morphology (compilation dominates short trials, so jobs are
  **grouped by morphology** before dispatch — a ~3× throughput win).
- **Isolation**: a crashed or diverged job records a failure row and never takes down the sweep.
- **Provenance**: every run directory captures the resolved config, git SHA, dirty-tree diff, package
  versions, host, and CPU count.

Run directory:

```
runs/<experiment_name>/<timestamp>-<config_hash>/
  config.resolved.yaml   provenance.json
  results.jsonl          morphologies/<morphology_id>.json
  traces/<job_id>.npz    media/<job_id>.mp4
  figures/               report.md  report.html
```

---

## 9. Optimization

```python
class Optimizer(Protocol):
    def ask(self, n: int) -> list[np.ndarray]: ...
    def tell(self, xs, fitnesses, infos) -> None: ...
    def state_dict(self) / load_state_dict(...)   # checkpointable
```

Optimizers see only `x ∈ [0,1]^n`; `ParamSpace` handles decoding, and `Objective` handles evaluating
a decoded hand across (object × task × seed) via the same `Runner` used by sweeps. The consequence
is that **an optimizer run and a sweep produce identical result rows**, so all analysis and reporting
machinery works on both without special cases.

Provided: `grid`, `random`, `cmaes` (via `cma`), `ga`, `nsga2` (non-dominated sort + crowding
distance, for performance-vs-complexity), `pso`, `bayes` (GP + expected improvement).
Every optimizer checkpoints every generation and resumes.

**Multi-objective is the default framing**, not an add-on: the project's question is explicitly about
trading capability against complexity, and a scalarized `score − λ·complexity` hides exactly the
trade-off curve the project wants to see.

---

## 10. Analysis and statistics

Tidy dataframe in, tables and figures out.

- Proportions: Wilson score intervals (never normal approximation — success rates near 0 or 1 are
  common here and the normal interval is badly wrong there).
- Means: BCa bootstrap CIs; trials within a morphology are correlated, so the bootstrap resamples at
  the **object level**, not the trial level.
- Comparisons: Welch's t and Mann–Whitney U, paired where the design is paired (same object, same
  seed, different morphology — which is much more powerful and is how ablations are run), with
  Holm–Bonferroni across families and Cliff's δ / Cohen's d reported alongside every p-value.
- Sensitivity: standardized regression coefficients, permutation importance on a gradient-boosted
  surrogate, Morris elementary effects, optional Sobol indices.
- Pareto: non-dominated sorting, hypervolume with a fixed reference point, knee-point identification.

---

## 11. Visualization

Layered, and independent of whether a window exists.

- `render.py` — offscreen `mujoco.Renderer` (EGL), fixed camera presets, deterministic framing.
- `overlays.py` — geometry injected into the `MjvScene` as decorative geoms: contact points, contact
  normals, force vectors scaled by magnitude, friction cones, COM markers, digit frames, joint-limit
  indicators, workspace point clouds, wrench-space projections.
- `video.py` — frame recorder with slow-motion, phase captions, and side-by-side composition
  (morphology A vs B, success vs failure) on a shared clock.
- `figures.py` — every figure is a registered builder taking a dataframe and returning a `Figure`, so
  reports assemble themselves and any figure can be regenerated from stored results alone.
- `interactive.py` — passive viewer with the same overlay stack, plus pause/step/scrub.

One `style.py` sets fonts, sizes, colors, and DPI so every figure in the project is visually
consistent and publication-ready without per-figure fiddling.

---

## 12. Reporting

`report/generator.py` turns a run directory into a self-contained document: configuration,
morphology cards (parameters + rendered image + complexity breakdown), object and task tables,
metric tables with CIs, the standard figure set, failure-mode breakdown, embedded video links, and
an automatically generated findings section (largest effects, significant differences, Pareto knee,
top sensitivity parameters). `report/trends.py` reads across runs to summarize what has been learned
so far.

---

## 13. Extension points (the usability test for this architecture)

| To add… | You write | You touch nothing else |
|---|---|---|
| A new object | one `@objects.register` function returning an `ObjectSpec` | ✔ |
| A new task | a `Task` subclass with stages + success criteria | ✔ |
| A new controller | a `Controller` subclass with `reset`/`act` | ✔ |
| A new metric | a function with a declared input kind | ✔ |
| A new perturbation | a `Perturbation` subclass | ✔ |
| A new optimizer | `ask`/`tell` | ✔ |
| A new figure | a dataframe → Figure builder | ✔ |
| A new searchable morphology parameter | one `Param` entry with getter/setter paths | ✔ |
| A new digit topology | `DigitSpec` field + builder branch | builder only |

If any of these ever requires editing the engine, the architecture has failed and should be fixed
rather than worked around.
