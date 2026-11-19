# Review of the Existing BiRH Project, and a Revised Research Agenda

*Written as the opening document of the Morphology Lab redesign. It evaluates the project as it
stands (commit `e932d41`), separates what should be preserved from what should be discarded, and
proposes a research agenda that the new platform is built to serve.*

---

## 1. What BiRH currently is

BiRH is a personal research project asking: *how is the human hand so effective at grasping a wide
range of objects, and how can we design a robotic hand that is simpler but still effective?*

The artifacts are:

| Artifact | Content |
|---|---|
| `design/v1-initial-concept/` | CAD (STEP/STL) of a 3-finger, 3-phalanx, radially symmetric hand; tendon holes for one-motor-per-finger underactuation with spring return; design notes |
| `design/v2-improved-design/` | CAD of a 2-finger + thumb hand with thumb abduction/adduction; an AHAP-inspired configuration comparison table |
| `simulation/mujoco-models/` | `hand.xml` (prototype 2, hand-authored), `hand_ur10e.xml` (hand mounted on a UR10e), `scene.xml`, `static_hand_demo.xml` (presentation-only), YCB meshes |
| `simulation/python-scripts/` | `hand_design.py` (f-string XML generation parameterized by 3 finger angles), `contact.py` (sequential closure + a disturbance test), `motor_control.py`, `viewer.py` |
| `visuals/` | Photos, presentation PDF |

Total live simulation code: **486 lines of Python, 609 lines of XML**, of which roughly 250 lines are
the vendored UR10e model from MuJoCo Menagerie.

---

## 2. Strengths — what is genuinely good and must be preserved

These are not politeness points. Each of them is a real research asset that the new platform
inherits directly.

**2.1 The research question is well-posed and unusual.**
Most hand projects ask "how do we build a more capable hand?" BiRH asks "how much capability
survives aggressive simplification, and which simplifications cost the most?" That is a *design
optimization* framing, and it is the correct framing — it has a measurable objective and an
interpretable answer. It is also under-served in the literature: there is a large body of work on
grasp synthesis for *given* hands and a much smaller body on comparative morphology under an
explicit complexity budget.

**2.2 The three-finger, force-closure-under-rotation argument is sound.**
`design-notes.md` justifies three fingers by force closure being maintained as the hand rotates
about the palm axis. This is correct and non-trivial: with a radially symmetric 3-finger layout the
grasp wrench space has approximate C₃ symmetry about the palm normal, so grasp quality is far less
sensitive to approach azimuth than for a 2-finger gripper. This is a *testable hypothesis*, and the
new platform should test it explicitly rather than assume it.

**2.3 AHAP adoption is the single best methodological decision in the project.**
Llop-Harillo et al. (2019) gives a standardized, published, citable benchmark of grasp types drawn
from activities of daily living. Adopting an external benchmark instead of inventing a private one
is what separates a portfolio project from a research project. The v2 configuration comparison table
is, structurally, a real experiment: three morphologies × nine grasp types, with an ordinal outcome.

**2.4 The "constrained human hand as a physical simulator" trick is clever and cheap.**
Using constrained motions of one's own hand to emulate a candidate morphology before committing to
CAD is an excellent low-cost screening method. It is exactly the right instinct — *evaluate the
morphology before you build it* — and it is the instinct the software platform now automates.

**2.5 The insight that emerged from v1 testing is the actual research finding.**
Two findings are recorded and both are substantive:
- *Missing abduction/adduction eliminates entire grasp categories* (lateral pinch, diagonal volar) —
  a discontinuity in capability, not a gradual degradation. This is a claim about the **structure**
  of the design space, and it is precisely the kind of claim morphology optimization should recover
  automatically.
- *Closure sequencing matters*: the desire for proximal-before-distal contact, approximated
  physically by differential screw friction. This is the underactuation problem, stated correctly
  from first principles by someone who had not yet read the underactuation literature.

**2.6 `hand_design.py` already contains the right idea.**
It is 181 messy lines, but the concept — *the hand is a function of parameters, and the XML is
generated, not written* — is the entire thesis of the new platform. The project had already
converged on procedural morphology generation. It just needed the idea taken seriously.

**2.7 `contact.py` already contains the right evaluation idea.**
`max_force()` applies disturbance forces along sampled unit vectors and ramps until slip. That is a
crude approximation of the **disturbance-tolerance / grasp-robustness metric**, which is a better
grasp quality measure than the analytic ε-metric alone because it is computed under the actual
contact dynamics. Again: the right instinct, needing rigor.

---

## 3. Weaknesses — technical

Stated bluntly, because these are the things the rewrite fixes.

**3.1 The morphology generator is not actually a morphology generator.**
`hand_design.py` exposes exactly three parameters (finger azimuths) and hardcodes everything else:
palm radius `0.05`, phalanx lengths `0.075`/`0.05`, damping `5`, range `0..1.571`, two phalanges,
pad sizes, contact solver parameters. Worse, the three azimuths are not fully honored:

```python
quat.append(euler_to_quat(a1, 0, 0, True))   # a1 is already radians; degrees=True re-converts
quat.append(euler_to_quat(60, 0, 0, True))   # hardcoded, ignores a2
quat.append(euler_to_quat(-60, 0, 0, True))  # hardcoded, ignores a3
```

`a1` is in radians (line 18) but passed with `degrees=True`, so it is converted a second time; and
fingers 2 and 3 get literal ±60° pad rotations regardless of the requested angles. Similarly `axis[]`
uses the `axes()` helper for finger 1 but the `xyz()` helper for fingers 2 and 3, so finger 1's hinge
axis is tangential (correct) while fingers 2 and 3 have **radial hinge axes** — they cannot flex
toward the palm centre at all. There are also two definitions of `euler_to_quat` with different
signatures and different component orders (the first returns `w,x,y,z`, the second returns a string
in `x,y,z,w` order; MuJoCo expects `w,x,y,z`). A stray `quat="..."` fragment sits outside a tag on
the `mp1` geom line.

The root cause is not carelessness — it is that **per-part trigonometry is the wrong abstraction**.
Every geom's position, orientation, and axis is computed independently from the azimuth, so each is
an independent opportunity for a sign or unit error, and errors are invisible in a viewer that still
renders something hand-shaped. The fix is structural: build a **mount frame per digit** once, and
express every phalanx in that frame with trivial local coordinates (`fromto="0 0 0  L 0 0"`,
`axis="0 -1 0"`). Then there is exactly one place where trigonometry happens, and it is unit-testable
against known frames.

**3.2 Absolute paths pin the code to one machine.**
`/Users/stellaluna/Documents/GitHub/BiRH/...` appears in `contact.py`, `motor_control.py`,
`viewer.py`, and `/Users/stellaluna/apple.stl` in `hand.xml`. None of these scripts run on any other
computer, including the one this platform is being written on. This alone makes the current results
unreproducible by anyone else.

**3.3 The disturbance test is physically wrong in three ways.**

```python
while not (linear_slip(...) or rot_slip(...)):
    data.xfrc_applied[obj_id, :3] = v * n
    mujoco.mj_step(model, data)
    n += .1
```

- *The force ramps continuously while the object accelerates.* The reported "maximum force" is
  therefore a function of the ramp rate and timestep, not a property of the grasp. Halving `dt`
  halves the reported tolerance. The correct procedure applies a **constant** wrench for a **fixed
  settling duration** and asks whether the object escaped, then **bisects** on magnitude.
- *There is no upper bound.* If the grasp never fails, the loop never terminates.
- *`restore_keyframe` is called with `grasp` captured after the loop began*, and `grasp` is only
  assigned inside the `phase == "closing"` branch, so `max_force` can be called with an unbound or
  stale snapshot. The `testing` branch is commented out, so this never actually ran.

Additionally `linear_slip` measures the palm→object centre distance, which is invariant to the
object *sliding tangentially around* the palm at constant radius — a real failure mode it cannot
detect. Displacement should be measured in the **palm frame** as a full 6-D pose error.

**3.4 Grasp evaluation is binary and contact-agnostic.**
The only notions of success are "a pad touches the object" and "the object moved more than 1 cm /
5°". There is no force closure test, no grasp wrench space, no contact distribution, no measure of
how *close* a marginal grasp was to failing. Two grasps that both "succeed" are indistinguishable,
which makes optimization over morphology impossible — a search needs a **graded** signal.

**3.5 Solver settings are unexamined and expensive.**
`integrator="RK4" timestep="0.0001"` on a contact-rich model: RK4 is a poor choice for
contact-dominated dynamics in MuJoCo (the recommended integrators are `implicitfast` or `implicit`,
which are stable at far larger timesteps and handle damping implicitly), and 0.1 ms means 10,000
steps per simulated second. A grasp trial of ~4 s costs 40,000 steps. At the scale this project now
needs — thousands of morphologies × dozens of objects × tens of trials — that is the difference
between an experiment finishing overnight and finishing next month. Meanwhile `hand.xml` sets
`range="0 90"` on `mpj1` with `angle="radian"`, i.e. a 5157° joint limit.

**3.6 Contact parameters are copy-pasted magic numbers.**
`solref="0.0005 1.6" solimp="0.99 0.999 0.0003 0.5 2"` appears identically on nine geoms. `solref`'s
first element is a time constant of 0.5 ms — *five times smaller than the 0.1 ms timestep is
required for stability*, and this is borderline. More importantly, contact stiffness/compliance is
one of the research variables the project explicitly wants to study ("how much compliance is
actually beneficial?"), so it must be a **named, swept parameter**, not a literal repeated nine
times.

**3.7 The closure controller conflates control with evaluation.**
`grasp_step` advances a per-finger joint index when a pad touches the object — a reasonable
sequential-closure heuristic — but it is fused to the evaluation loop, to specific joint names
(`mpj1`, `pipj1`, …), and to a specific object geom name. It cannot be swapped for a PD controller,
a synergy controller, or a learned policy. `all_fingers_done` additionally mutates `finger['current']`
inside what reads as a predicate, and returns `False` on the first incomplete finger, so it can
advance one finger and skip checking the others.

**3.8 No determinism, no logging, no persistence.**
`random_unit_vectors(n, rng=0)` seeds correctly, but nothing else does; nothing is written to disk;
the outcome of a run is a `print` in a terminal. There is no way to compare a run today with a run
last month, and no record of which model version produced which photograph.

**3.9 No tests, no packaging, no dependency declaration.**
There is no `pyproject.toml`, no `requirements.txt`, no test directory. The repository does not
declare that it needs MuJoCo.

---

## 4. Weaknesses — methodological

These matter more than the code defects, because they limit what the project can *claim*.

**4.1 n = 1 per cell, and the outcome scale is subjective.**
The v2 comparison table has one qualitative rating per (configuration, grasp type) cell, produced by
one person constraining their own hand. `⚠️ Less stable` is not measurable and not comparable across
rows. No repetition means no variance estimate, and no variance estimate means no claim can be
distinguished from noise. This is the single largest methodological gap: **the project's central
result rests on 27 unrepeated subjective judgments.**

**4.2 The experimenter and the apparatus are the same person.**
Using one's own constrained hand to emulate a morphology is a great screening tool, but the person
performing the grasp knows which configuration is "supposed" to win and controls every unmeasured
variable (approach angle, wrist pose, applied force, retry count). This is an uncontrolled and
unblinded protocol. It is fine for generating hypotheses and unsuitable for confirming them.

**4.3 Confounded comparisons.**
Prototype 1 (3 symmetric fingers) versus Config 1 (2 fingers + thumb) differs in **finger count,
finger placement, symmetry, and opposition geometry simultaneously**. When Config 1 grips cylinders
better, no attribution to a specific feature is possible. The v1→v2 decision to move to 2+thumb was
therefore made on a confounded comparison. It may well be the right decision — but the evidence does
not establish it. Proper ablation requires changing **one** morphological factor at a time, which in
turn requires a parameterized generator, which is why 3.1 and 4.3 are the same problem.

**4.4 Simulation and physical prototype have drifted apart.**
`hand.xml` (2 phalanges, 2 fingers + thumb, direct joint actuators, palm radius 0.1 m) does not
correspond to `hand_design.py` (2 phalanges, 3 fingers, palm radius 0.05 m) and neither corresponds
to v1 CAD (3 phalanges, 3 fingers, tendon-driven). `design-notes.md` records the reason for the last
divergence — the tendon model would not move, so it was replaced by direct joint actuators on advice
— which was pragmatically correct but silently changed the object of study. **Underactuation is the
defining feature of the physical hand and it was removed from the model.** Any simulated conclusion
about closure sequencing therefore does not transfer to the prototype.

*(The tendon failure has a known cause, incidentally: a MuJoCo `spatial` tendon only produces joint
torque if it is routed through sites that give it a nonzero **moment arm** about each joint axis;
sites placed along the phalanx centreline, or with the tendon slack at the reference configuration,
produce zero torque no matter how large the actuator gain. The new platform models underactuation
with `fixed` tendons — an explicit linear coupling Σᵢ cᵢqᵢ with `cᵢ` as the moment arms — which is
both numerically robust and a more honest abstraction of a routed tendon.)*

**4.5 The object set is not sampled from anything.**
Grasps were tested against wire, paper, tape, an apple mesh, a cylinder. There is no statement of
what population these represent, so "works across various shapes" has no scope. A defensible object
set is drawn from a published benchmark (YCB — already partly vendored in `assets/`) and stratified
by the properties expected to matter (size relative to palm, aspect ratio, mass, surface friction).

**4.6 No baseline.**
Nothing is compared against a published hand. Without a reference point — a parallel-jaw gripper at
minimum, ideally an Allegro/Barrett-class hand — "effective" has no scale.

**4.7 The complexity axis is asserted, never measured.**
The whole premise is *simpler but still effective*, yet "simplicity" is never quantified. Is a
2-finger+thumb hand with thumb abduction (7 DOF as modelled, 7 actuators) simpler than a 3-finger
symmetric hand (6 DOF, 3 actuators when tendon-driven)? By actuator count, **no** — v2 is *more*
complex than v1 despite being presented as the refinement. This inversion is invisible without an
explicit complexity metric, and it means the project may have been optimizing the wrong direction.

---

## 5. Assumptions worth making explicit (and testing)

The project rests on assumptions that are reasonable, unstated, and empirically checkable. The
platform's job is to convert each into an experiment.

| # | Implicit assumption | Status | How the platform tests it |
|---|---|---|---|
| A1 | Three fingers is the right constraint | Asserted from force-closure symmetry | Sweep n ∈ {2,3,4,5} under matched complexity budget; measure marginal return per finger |
| A2 | Radial symmetry is good | Inherited from v1 | Compare symmetric vs. optimized-asymmetric layouts on the same object set |
| A3 | Human phalanx proportions transfer | Scaled from the designer's forefinger | Treat length ratios as free parameters on the 2-simplex; see whether the optimum lands near human ratios |
| A4 | Distal-decreasing length is correct | Stated in v1 notes | Falls out of A3 |
| A5 | Proximal-first closure is desirable | Stated in v1 notes | Compare closure schedules (simultaneous / proximal-first / tendon-differential) as a controller factor |
| A6 | Thumb abduction is worth its cost | v2's central claim, n=1 | Ablation: identical hand ± abduction DOF, matched trials, paired statistics |
| A7 | 2 phalanges suffice | Chosen for prototyping ease | Sweep phalanx count 2 vs 3 at matched total finger length |
| A8 | Pads improve grasping | Present in every model, never varied | Sweep pad friction and compliance; measure effect size |
| A9 | A rigid palm is adequate | Never questioned in code, questioned in notes | Sweep palm compliance (`solref`/`solimp`) and palm curvature |
| A10 | Grasp quality generalizes across objects | Implicit in "works across shapes" | Report per-category scores and a cross-category generalization index, never a single mean |

---

## 6. Unanswered research questions the project already raised

From `design-notes.md`, verbatim in substance — these are good questions and the platform is
designed to answer exactly these:

1. How would an extra DOF rotating each finger about the palm (perpendicular to the current hinge)
   improve grasping?
2. What finger placement positions around the palm are best?
3. Would a deformable palm improve stability?
4. How would a more complex thumb mechanism improve grasping?
5. How can a thumb mechanism authentically capture human thumb motion?

To which the review adds:

6. Where do diminishing returns in DOF count begin?
7. Is symmetric-3 or opposed-2+thumb better *at matched actuator count*?
8. Does underactuation (tendon coupling) beat independent joint control at matched actuator count?
9. How much of grasp performance is morphology and how much is control? (A bad controller makes a
   good hand look bad; this confound must be broken by optimizing control per morphology.)
10. Which morphological parameters does performance depend on *most steeply* — i.e. where should
    manufacturing tolerance be tight?

---

## 7. Revised research agenda

The agenda is organized as five studies of increasing ambition. Each is a concrete, publishable
question with a defined design, and each is implementable on the platform described in
`01-ARCHITECTURE.md`. Numbers of trials are indicative and set by power analysis on pilot variance.

### Study 1 — Baseline characterization and instrument validation
*Goal: establish that the simulator produces stable, reproducible, well-calibrated grasp outcomes
before any claim is made with it.*

- Fix a reference morphology (v1: 3 symmetric fingers, 2 phalanges, hinge joints).
- Run every object in the library × every task × 30 seeded trials.
- Report: success rate with Wilson intervals, ε-metric distribution, disturbance tolerance,
  wall-clock cost per trial, and **seed-to-seed variance** vs. **domain-randomization variance**.
- Validate solver settings: sweep timestep and integrator, confirm metrics are invariant to
  numerical parameters within tolerance. *A metric that changes when the timestep changes is not a
  metric.*
- Deliverable: a calibration report. This is the step whose absence most limits the current project.

### Study 2 — Factorial ablation of morphological features
*Goal: attribute performance to individual features, breaking the confounds of §4.3.*

A 2-level factorial over the features the project actually disputes:

| Factor | Levels |
|---|---|
| Digit count | 2 + thumb / 3 symmetric |
| Thumb abduction | absent / present |
| Phalanx count | 2 / 3 |
| Actuation | independent joints / tendon-coupled (underactuated) |
| Palm compliance | rigid / compliant |
| Pad friction | low / high |

Full factorial = 64 cells × 15 objects × 20 seeds = 19,200 trials. At ~0.4 s/trial across 20 workers
this is well under an hour — the point being that *the factorial that was previously infeasible by
hand is now cheap*. Analysis: main effects and two-way interactions with Holm-corrected tests and
effect sizes; the headline output is a **ranked list of which DOF are worth their complexity**,
which is the project's central question stated in answerable form.

### Study 3 — Continuous morphology optimization under a complexity budget
*Goal: find the best 3-finger hand, and characterize the shape of the objective landscape.*

- Search space (deliberately small, per the constraint to start narrow): finger azimuths (2 free
  after canonicalization), palm radius, total finger length, two length-ratio parameters on the
  simplex, flexion range, joint stiffness, joint damping, pad friction, optional thumb-abduction
  range. ~11 continuous dimensions.
- Optimizers: random search baseline → CMA-ES → NSGA-II for the (performance, complexity) Pareto
  front. Random search is not a formality; it is the **control condition** that says whether the
  fancy optimizer earned its compute.
- Fitness = mean task score over a *training* object set, with a held-out *test* object set never
  seen by the optimizer. Reporting test-set performance is what distinguishes morphology
  optimization from morphology overfitting, and it is routinely omitted in this literature.
- Deliverable: a Pareto front of hands, sensitivity/importance ranking over parameters, and the
  answer to "where do diminishing returns begin".

### Study 4 — Generalization and robustness
*Goal: determine which morphology generalizes, not which scores highest on average.*

- Cross-category evaluation: train on one object category, test on all others; a full
  train-category × test-category matrix.
- Domain randomization at test time: friction, mass, object pose, actuator noise, sensor noise.
- Report a **generalization score** (worst-category performance, or the mean minus the standard
  deviation across categories) alongside the mean. Optimizing the mean and optimizing the worst case
  select different hands, and which one you report is a scientific choice that should be made
  explicitly.
- Failure-mode taxonomy: every failed trial is automatically classified (never contacted / slipped
  out / rotated out / ejected on lift / failed under disturbance / self-collision / actuator
  saturation), so failures become data rather than absence of data.

### Study 5 — Morphology–control co-optimization
*Goal: break the morphology/control confound of question 9.*

- Inner loop: optimize control (initially closure-schedule and gain parameters via CMA-ES; later an
  RL policy) for a given morphology.
- Outer loop: evolve morphology on the inner-loop-optimized score.
- Compare against the naive single-loop result to **quantify how much of Study 3's ranking was an
  artifact of a fixed controller**. This is the most interesting result the project can produce, and
  it is only reachable once the platform exists.

### Cross-cutting: AHAP as the external yardstick
Every study reports an AHAP-style score alongside internal metrics, so results connect to a
published benchmark and to the v2 hand-testing table already in the repository — which becomes a
validation target: *does the simulator reproduce the qualitative ordering that was observed by
hand?* If it does, that is evidence the simulator is trustworthy. If it does not, that discrepancy
is itself a finding worth chasing.

---

## 8. Positioning relative to current literature

Briefly, so the agenda is not re-deriving solved problems:

- **Morphology optimization / co-design.** Evolving body and brain together goes back to Sims (1994);
  the modern line runs through Ha's *Reinforcement Learning for Improving Agent Design* (2019),
  Schaff et al. (2019), and the graph-based approaches of Wang et al. (*Neural Graph Evolution*,
  2019) and Zhao et al. (*RoboGrammar*, 2020). The consistent lesson is that **morphology search
  with a fixed, mediocre controller reliably ranks morphologies wrong**, which is why Study 5 exists
  and why Study 3 explicitly reports its controller as a limitation.
- **Hand design under a complexity budget.** Underactuated and compliant hands — the iHY hand
  (Odhner et al., 2014), the Yale OpenHand series, and the RBO Hand line — all argue that mechanical
  intelligence substitutes for actuator count. BiRH's v1 tendon design is squarely in this tradition
  and the project should not have abandoned it in simulation.
- **Grasp quality metrics.** Ferrari & Canny (1992) for the ε and volume metrics; Roa & Suárez
  (2015) for the survey and, importantly, for the finding that analytic metrics correlate only
  moderately with physical success. Hence this platform computes analytic ε **and** simulated
  disturbance tolerance and reports both, treating their disagreement as informative.
- **Benchmarks.** YCB (Calli et al., 2015) for objects — already partly present in `assets/`; AHAP
  (Llop-Harillo et al., 2019) for grasp-type coverage.
- **What is under-explored, and where BiRH can contribute.** Most co-design work optimizes locomotors
  or simple grippers on a single objective. Comparatively little work reports (a) a
  performance-versus-*manufacturability* Pareto front for hands, (b) held-out object generalization
  for optimized morphologies, or (c) whether an *evolved* hand recovers known biological proportions.
  All three are within reach here, and all three are more interesting than "our optimizer found a
  good hand".

---

## 9. What carries forward, in one table

| From the current project | Disposition |
|---|---|
| Research question and "simplicity as an axis" framing | **Keep, formalize** — complexity becomes a measured objective |
| 3-finger symmetric baseline | **Keep as a preset**, `birh_v1`, and as a control condition |
| 2-finger + thumb design | **Keep as a preset**, `birh_v2`, no longer privileged as "the improvement" |
| AHAP methodology | **Keep, automate** — becomes a task suite |
| Procedural XML generation (`hand_design.py`) | **Idea kept, implementation rewritten** as frame-based construction |
| Disturbance-tolerance evaluation (`contact.py`) | **Idea kept, implementation rewritten** as bisection on constant wrenches |
| Sequential contact-driven closure (`grasp_step`) | **Idea kept**, becomes one `Controller` among several |
| Tendon underactuation (v1 CAD, abandoned in sim) | **Restored** via `fixed` tendons — it is the hand's defining feature |
| YCB meshes in `assets/` | **Keep**, wrapped by the object library |
| Hand-authored `hand.xml`, `hand_ur10e.xml` | **Archive** — reference only, never the source of truth |
| Absolute machine paths | **Delete** |
| RK4 @ 1e-4 s | **Replace** with `implicitfast`, timestep validated by Study 1 |
| UR10e mount | **Defer** — a mocap-welded floating wrist is sufficient and far cheaper for morphology studies; the arm returns for manipulation tasks |

---

## 10. The one-sentence summary

BiRH had the right question, the right benchmark, and the right two implementation ideas, evaluated
with n=1 subjective trials on confounded comparisons using a model that had silently diverged from
the hardware — so the rewrite changes almost none of the thinking and almost all of the machinery.
