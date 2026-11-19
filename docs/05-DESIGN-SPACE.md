# What to Vary: A Design-Space Review for a Genuinely Interesting Hand

*A literature-grounded answer to "which parameters should the search be allowed to
change?", written after the AHAP framing was dropped and before the second morphology
space is built. It is opinionated on purpose. Every claim that matters is tied to a
published result, and every recommendation says whether the platform can already do it.*

---

## 1. The short version

The current space (`three_finger_v1`, 12 parameters) varies **where** three identical
hinge-jointed fingers sit and **how big** they are. Those are the parameters a search can
tune, but they are not the ones that make one hand *different in kind* from another. Forty
years of hand design say the discontinuities in grasping capability come from five places,
in roughly this order of leverage:

1. **The contact interface** — friction, compliance, area, and adhesion. This is where
   Cutkosky's group has spent most of its effort, and it changes what force closure even
   requires.
2. **Opposition geometry** — whether and how digits can push *against* each other, which is
   what a thumb is for. Not "how many fingers" but "how many independent opposition axes".
3. **Compliance and underactuation** — how the hand conforms without being told to. The
   iHY, SDM, RBO and SoftHand results all show mechanical intelligence substituting for
   actuators.
4. **Coupling structure** — which joints move together and in what ratio: tendon routing,
   differentials, synergies. This is a design variable the current space holds fixed.
5. **Geometric affordances** — nails, edges, palm cupping, fingertip shape. Small features
   that decide whether a phone, a key or a plate is graspable at all.

Only after those come the things the current space already varies: placement, length,
proportions. The recommendation in §4 is a second space built around 1–5, with feasibility
(§5) as a co-objective so the search cannot escape into hands that cannot be built.

---

## 2. What the literature says, by theme

### 2.1 Grasp choice and the role of the task (Cutkosky)

Cutkosky's 1989 grasp taxonomy (*On grasp choice, grasp models, and the design of hands for
manufacturing tasks*, IEEE T-RA) organized human grasps along two axes — **power versus
precision**, and the object geometry that drives the choice — and made the point that a
hand design should be derived from the *task requirements* (forces to resist, motions to
produce, uncertainty to tolerate), not from anatomy. Two consequences for this project:

- The evaluation must include tasks that separate power from precision. Lifting a sphere
  rewards enclosure; reorienting a marker rewards fingertip control. A hand optimized on
  one will lose the other. The platform already has both kinds of task; the *object set*
  needs to be stratified so both are represented (§4.3).
- "How many fingers" is the wrong primary question. The taxonomy's precision grasps
  need two or three *opposed* contacts; its power grasps need a palm and wrap. A three-finger
  radial hand is a power-grasp specialist that improvises precision; a two-finger-plus-thumb
  hand is the reverse. Comparing them on a single mean success rate hides that.

Feix, Romero, Schmidt, Kragic and Dollar (2016, *The GRASP taxonomy of human grasp types*,
IEEE T-HMS) consolidated 33 grasp types and is the current standard; it is also a
reasonable replacement for the AHAP grasp list if you want an external yardstick that
is not AHAP.

### 2.2 Contact and adhesion: the interface is the design (Cutkosky lab, 2010–2022)

The Stanford BDML line — Hawkes, Cutkosky and colleagues on gecko-inspired microwedge
adhesives (e.g. Hawkes et al. 2015, *J. R. Soc. Interface*; Jiang et al. 2017 *Science
Robotics* on the space gripper) — established that a **shear-activated, controllable
adhesive** changes the grasping problem: contacts can resist *pull-off* and tangential
load without normal force, so force closure no longer requires wrapping. Ruotolo, Brouwer
and Cutkosky (2021, *From grasping to manipulation with gecko-inspired adhesives on a
multifinger gripper*, *Science Robotics*) put that on a multi-finger hand (the "farmHand")
and showed **large, delicate objects handled with low squeeze force**, plus in-hand
manipulation that a friction-only hand cannot do.

What this means for a design search:

- `pad_friction` is in the current space, but friction is a *symmetric* parameter. An
  adhesive is **anisotropic**: high shear capacity in one direction, low in the other,
  and it needs a preload and a peel angle. Modelling it as "high μ" over-credits it in
  directions where it does nothing. The honest simulation is a contact model with
  direction-dependent friction and a small tensile capacity; MuJoCo cannot do the tensile
  part natively, so this is a research contribution, not a parameter.
- Pad **area and compliance** matter more than μ once μ > ~0.8. The classic result
  (Cutkosky & Wright 1986, *Friction, stability and the design of robotic fingers*, IJRR)
  is that soft fingertips resist torsion about the contact normal in proportion to the
  contact patch, which is what stops a pinched object spinning. The platform models this
  with torsional friction on pads; it is set per material but not searched.

The person whose name you could not recall is likely **Hannah Stuart** (UC Berkeley, a
Cutkosky PhD): her work on the Ocean One hand (Stuart et al. 2017, *The Ocean One hands:
An adaptive design for improved grasping*, IJRR), on suction and adhesion in unstructured
grasping, and more recently on **compliant palms and tendon-driven underactuated hands**
is close to what this project is trying to do. If Dr. Cutkosky's suggestion was along
these lines, the theme is: *make the surface and the compliance do the work, and keep the
actuator count low.* That is exactly the "simpler but effective" thesis, argued from
mechanism rather than from finger count.

### 2.3 Underactuation and compliance

- **Dollar & Howe** (2010, *The highly adaptive SDM Hand*, IJRR): four fingers, one
  actuator, elastomer flexure joints, shape-deposition manufactured. Grasped a wide range
  of objects *because* it could not be controlled precisely. Key design variables: **joint
  stiffness ratios** along the finger and the **tendon moment-arm ratio**, which set the
  closure sequence passively.
- **Odhner et al.** (2014, *A compliant, underactuated hand for robust manipulation*,
  IJRR; the iHY hand): three fingers, one of which is a re-orientable thumb; flexure
  joints; demonstrated that a simple hand with the right compliance can do precision and
  power grasps *and* some in-hand manipulation.
- **Birglen, Laliberté & Gosselin** (2008, *Underactuated Robotic Hands*, Springer): the
  theory. Underactuated fingers have a **grasp stability condition** on the ratio of
  transmission gains to phalanx lengths; some geometries eject the object as they close.
  This is precisely the closure-sequence problem the v1 design notes raised and tried to
  solve with differential screw friction. The right variables are the transmission
  ratios (`tendon_coefficients`), the extension-spring stiffnesses, and the phalanx
  length ratios — *jointly*.
- **Catalano et al.** (2014, *Adaptive synergies for the design and control of the
  Pisa/IIT SoftHand*, IJRR): 19 joints, one motor, driven by the first postural synergy of
  the human hand (Santello, Flanders & Soechting 1998). The design question becomes
  "which one-dimensional motion should the hand be built to make?", and the answer is
  learned from human data rather than guessed.
- **Deimel & Brock** (2016, *A novel type of compliant and underactuated robotic hand for
  dexterous grasping*, IJRR; RBO Hand 2): pneumatic soft fingers with a **soft palm**;
  argued for "morphological computation" — the shape and compliance perform part of the
  control. Grasped 31 objects with 13 grasp types.

The consistent lesson: **compliance and coupling are design parameters, and they interact
with geometry**. A search that varies geometry with compliance held fixed will find the
best hand for that compliance, which is a much less interesting object.

### 2.4 Opposition and the thumb

The human thumb's distinguishing property is not that it exists but that its
carpometacarpal joint gives it a **movable opposition axis** (Kapandji's opposition
scale; Napier 1956 on prehensile movements). Robotic results:

- Xu & Todorov (2016, *Design of a highly biomimetic anthropomorphic robotic hand*, ICRA)
  reproduced the thumb CMC saddle joint and showed it is the single feature most
  responsible for grasp versatility in their hand.
- Kontoudis et al. (2019, *Open-source, anthropomorphic, underactuated robot hands with a
  selectively lockable differential mechanism*, IEEE RA-M): a lockable differential lets
  one actuator produce several grasp types; the *lock state* is a discrete design variable.

For this project: the abduction DOF in `birh_v2` is a first step but only moves the thumb
in one plane. The parameters that actually vary opposition are **thumb base roll**
(the `roll` field in `MountSpec`, present in `anthropomorphic_ref`, absent from the
search space), **thumb elevation**, and a **palm cup angle** that lets the two fingers
converge. These are three parameters, all already representable, none searched.

### 2.5 Geometric affordances nobody optimizes

The failure breakdown in this repository's own pilot is instructive: flat objects
(phone, key, plate, disc) defeat every preset, and they do so in the same way — nothing
can get *under* the object. The literature's answer is not more DOF:

- **Fingernails / distal edges**: a thin stiff lip at the fingertip that slides under a
  flat object against the table (the "scooping" affordance). Humans use it constantly;
  Stuart's Ocean One hand and several agricultural grippers have it; almost no
  optimization study parameterizes it. A single parameter — nail length — would change
  the achievable set of objects, which is a bigger effect than any continuous tuning.
- **Palm cupping / curvature**: a concave palm turns a three-point power grasp into an
  enveloping one and improves torque resistance. `PalmSpec.curvature` exists in the spec
  and is unused by the space.
- **Fingertip shape**: spherical tips maximize approach tolerance; cylindrical or
  flattened tips maximize contact area; rolling (Hillberry) contacts change the kinematics.
  The platform models tips as capsules with a fixed radius ratio.
- **Phalanx cross-section and tapering**: tapered links reach into concavities (the mug
  handle) and reduce self-collision between closely spaced digits, which is the dominant
  failure of the opposed presets.

### 2.6 What optimization studies of hands have actually found

- Ha (2019, *Reinforcement learning for improving agent design*) and Schaff et al. (2019,
  *Jointly learning to construct and control agents*): body optimization with a **fixed
  or under-trained controller reliably picks the wrong body**. Every ranking in this
  repository is currently produced that way (`docs/04-STATUS.md` §2.7).
- Zhao et al. (2020, *RoboGrammar*): searching over *structures* via a graph grammar finds
  bodies a parameter sweep cannot express. Applied to hands, the grammar's productions
  would be "add a digit at azimuth θ", "split a phalanx", "couple joints i and j with
  ratio r", "add an abduction axis", "add a nail". This is the honest way to "let it
  evolve" and is beyond a numeric parameter space.
- Hazard, Pollard et al. (2018, *Automated design of manipulators for in-hand tasks*):
  fingertip placement and count optimized for in-hand rolling, found that **two or three
  well-placed fingers outperform four badly placed ones**, and that optimal placement is
  task-dependent.
- Kodnongbua et al. (2023, *Computational design of passive grippers*) and the fin-ray /
  passive-gripper literature: with the right compliance and geometry the actuator count
  goes to zero for some tasks.

---

## 3. Feedback on the current search space

| Parameter | Verdict | Why |
|---|---|---|
| `gap_1`, `gap_2` (digit azimuths) | keep | The only true "layout" freedom; cheap; canonicalized. |
| `palm_radius` | keep | Sets aperture; interacts with everything. |
| `finger_length` | keep | But it should scale the *object set*, not compete against it (§4.3). |
| `ratio_u1`, `ratio_u2` (phalanx proportions) | keep, expect small effect | Birglen: proportions matter *jointly with* transmission ratios; alone they are weak. |
| `flexion_max` | keep | Cheap; decides wrap-around. |
| `joint_stiffness`, `joint_damping` | keep, and add per-joint ratios | A single stiffness for all joints cannot express proximal-first closure. |
| `pad_friction` | keep, but low ceiling | Above ~1.0 the sim over-credits it; compliance/area matter more. |
| `thumb_abduction`, `abduction_range` | keep, but it is not "a thumb" | Without base roll and elevation it is a finger that wags. |

Missing, in order of the leverage I would expect:

1. **Transmission ratios** (`tendon_coefficients` per joint) and **extension-spring
   ratios** — the underactuation design variables. Representable today; not searched.
2. **Thumb base roll and elevation** — the opposition axis. Representable today.
3. **Palm curvature** — representable today.
4. **A fingernail / distal lip** — needs a builder change (one extra geom on the distal
   phalanx). One parameter; likely the largest single effect on the flat-object failures.
5. **Pad compliance** (solref/solimp of pad material) and **pad coverage/thickness** —
   representable today via `MaterialSpec`.
6. **Per-digit length and radius asymmetry** — a thumb is usually shorter and thicker.
7. **Digit count 2–4** — an integer parameter; the builder handles it, the space does not.
8. **Joint type**: pin vs flexure — matters for feasibility more than for the sim; treat
   as a material choice (§5).

What I would *not* add yet: anything the simulator cannot model honestly. Adhesion
(tensile contact), variable stiffness, jamming, and soft-body deformation of the fingers
themselves all fall in that category. They are real design options, but a search over
them in this simulator would be a search over artifacts.

---

## 4. A concrete second space

### 4.1 `three_finger_v2` (≈ 22 parameters)

All of `v1`, plus:

| Parameter | Range | Notes |
|---|---|---|
| `coupling_2`, `coupling_3` | [0.3, 1.2] | tendon moment-arm ratio of joint 2 and 3 relative to joint 1 |
| `spring_ratio_2`, `spring_ratio_3` | [0.3, 3.0] log | extension stiffness of joint 2, 3 relative to joint 1 |
| `thumb_roll` | [−80°, 0°] | base roll of digit 3, active only when `thumb_abduction` |
| `thumb_elevation` | [0°, 30°] | |
| `thumb_length_ratio` | [0.7, 1.1] | digit 3 length relative to the others |
| `palm_curvature` | [0, 0.4] | |
| `pad_compliance` | [0.01, 0.06] s log | `SolverSpec.time_const` for the pad material |
| `pad_coverage` | [0.5, 1.0] | fraction of phalanx wrapped |
| `nail_length` | [0, 8 mm] | needs the builder addition |
| `radius_scale` | [0.07, 0.13] | link thickness relative to length (currently fixed at 0.095) |

Dimensionality 22 is well inside CMA-ES's comfortable range and small enough that a random
search of a few thousand hands (one overnight) gives a useful baseline.

### 4.2 `graph_space` (later)

A grammar over `HandSpec` with productions: add/remove digit; add abduction; add roll;
split/merge phalanx; set coupling; add nail. Searched by a GA over production sequences.
This is where hands the designer did not think of come from; it is also where feasibility
becomes essential, because grammars find degenerate structures fast.

### 4.3 The object set is part of the design space

"Wide variety" has to be stratified, or the optimizer tunes to whatever dominates the
set. Recommended: 4 shape classes (round, cylindrical, flat, non-convex) × 3 sizes
relative to the palm (0.5, 1.0, 1.8) = 12 cells, two objects per cell, drawn from the
existing library plus YCB. Train on one object per cell, test on the other. Report
**per-cell** success and the **worst cell**, not just the mean (`docs/00-REVIEW.md`, Study 4).

---

## 5. Material and actuator feasibility

`morpholab.feasibility` scores a `HandSpec` against a named material, actuator and pad
material. The checks are first-order but each maps to a real failure mode:

| Check | Catches |
|---|---|
| `actuator_torque` | the sim's `force_limit` exceeds what the servo / tendon spool can deliver |
| `actuator_oversized` | the actuator is 3× what the sim uses (weight for nothing) |
| `bending_stress`, `tip_deflection` | rigid links too thin for the load (cantilever at full torque) |
| `flexure_thickness`, `flexure_strain` | soft links: the simulated joint stiffness is not achievable as a living hinge, or the hinge would tear at full flexion |
| `tip_radius` | distal link cannot house a pin, a tendon channel and two walls |
| `pad_thickness` | pad thinner than the pad material can be cast/printed |
| `palm_volume`, `actuators_in_phalanges`, `tendon_exits` | the actuators do not fit |
| `chamber_radius` | pneumatic finger too thin for a chamber |
| `grip_to_weight`, `hand_mass` | the hand cannot lift itself / exceeds a wrist budget |

Two things it does deliberately:

- **Soft is a different physics, not a different number.** Choosing TPU or silicone turns
  every hinge into a flexure; the checks switch to strain and thickness, and a hand with
  high simulated joint stiffness becomes *infeasible* in silicone because the flexure
  would have to be thicker than the finger. That is the correct answer and the sim would
  never tell you.
- **Actuator kind determines packaging.** Rotary servos must fit in the links or go to
  the palm with tendons; spool motors live in the palm and are limited by tendon tension
  × moment arm; pneumatic chambers need soft bodies. `--matrix` scores every pairing so a
  design can be asked "what could this be made of?"

Use it as a **penalty** in optimization (`--w-feasibility`), as a **filter** on a
finished run (`python -m morpholab.feasibility runs/<run>/morphologies --csv`), or as a
design conversation (`--matrix` on one hand).

What it does not do: dynamics of tendons (friction, stretch, backlash), thermal limits,
gear wear, cable routing around joints, or cost. Those are the next layer if the numbers
start to matter.

---

## 6. What I would actually run first

1. Tonight's random sweep (600 hands, `v1` space) as the baseline.
2. `morpholab optimize morphology --algorithm cmaes` on `v1`, 30 generations, with
   `--w-feasibility 0.3 --material pla --actuator xl330_spool`, to see whether CMA-ES
   beats random at matched budget and whether feasibility changes the winner.
3. Build `three_finger_v2` (§4.1, about 150 lines in `space.py` plus the nail geom) and
   run NSGA-II over (performance, complexity) for the Pareto front.
4. `morpholab optimize codesign` on 20 hands from the front to measure how much the
   fixed-controller ranking moves. If the Spearman correlation is below ~0.6, every
   earlier ranking needs the caveat; if above ~0.8, the cheap loop is trustworthy.
5. Only then the grammar space.

---

## References

- Birglen, L., Laliberté, T., Gosselin, C. (2008). *Underactuated Robotic Hands*. Springer.
- Catalano, M. G., et al. (2014). Adaptive synergies for the design and control of the Pisa/IIT SoftHand. *IJRR* 33(5).
- Cutkosky, M. R. (1989). On grasp choice, grasp models, and the design of hands for manufacturing tasks. *IEEE T-RA* 5(3).
- Cutkosky, M. R., Wright, P. K. (1986). Friction, stability and the design of robotic fingers. *IJRR* 5(4).
- Deimel, R., Brock, O. (2016). A novel type of compliant and underactuated robotic hand for dexterous grasping. *IJRR* 35(1–3).
- Dollar, A. M., Howe, R. D. (2010). The highly adaptive SDM Hand: design and performance evaluation. *IJRR* 29(5).
- Feix, T., et al. (2016). The GRASP taxonomy of human grasp types. *IEEE T-HMS* 46(1).
- Ha, D. (2019). Reinforcement learning for improving agent design. *Artificial Life* 25(4).
- Hawkes, E. W., et al. (2015). Grasping without squeezing: shear adhesion gripper with fibrillar thin film. *ICRA* / *J. R. Soc. Interface*.
- Hazard, C., Pollard, N., et al. (2018). Automated design of manipulators for in-hand tasks. *Humanoids*.
- Jiang, H., et al. (2017). A robotic device using gecko-inspired adhesives can grasp and manipulate large objects in microgravity. *Science Robotics* 2(7).
- Kontoudis, G. P., et al. (2019). Open-source anthropomorphic underactuated robot hands with a selectively lockable differential mechanism. *IEEE RA-M*.
- Odhner, L. U., et al. (2014). A compliant, underactuated hand for robust manipulation. *IJRR* 33(5).
- Ruotolo, W., Brouwer, D., Cutkosky, M. R. (2021). From grasping to manipulation with gecko-inspired adhesives on a multifinger gripper. *Science Robotics* 6(61).
- Santello, M., Flanders, M., Soechting, J. F. (1998). Postural hand synergies for tool use. *J. Neurosci.* 18(23).
- Schaff, C., et al. (2019). Jointly learning to construct and control agents using deep reinforcement learning. *ICRA*.
- Stuart, H., et al. (2017). The Ocean One hands: An adaptive design for improved grasping. *IJRR* 36(2).
- Xu, Z., Todorov, E. (2016). Design of a highly biomimetic anthropomorphic robotic hand towards artificial limb regeneration. *ICRA*.
- Zhao, A., et al. (2020). RoboGrammar: graph grammar for terrain-optimized robot design. *ACM TOG* 39(6).

*Citations are from memory and should be checked against the originals before they go in
anything you submit; years and venues are correct to the best of my knowledge, page
numbers are omitted for that reason.*
