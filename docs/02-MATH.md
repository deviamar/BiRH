# Mathematical Foundations

*Everything the platform computes, stated precisely enough to be implemented and checked. Section
numbers are referenced from docstrings in the code.*

---

## 1. Morphology as a parameterized manifold

A hand is a point `h ∈ H` in a structured design space

```
H = H_disc × H_cont
```

where `H_disc` holds discrete structure (digit count `n_d`, phalanges per digit `n_p`, presence of
abduction, actuation mode) and `H_cont ⊆ ℝ^m` holds continuous geometry and dynamics.

Optimizers work on a normalized cube. For each continuous parameter with bounds `[a_i, b_i]`:

```
x_i = (θ_i − a_i)/(b_i − a_i)                 linear
x_i = (ln θ_i − ln a_i)/(ln b_i − ln a_i)     log-scaled (stiffness, damping, friction)
```

Integers and categoricals decode by binning `x_i ∈ [0,1)` into `k` equal cells; booleans threshold
at ½. Bin decoding is used rather than rounding because it keeps the optimizer's view of the space
uniform.

### 1.1 The simplex constraint on phalanx lengths

Phalanx lengths are `L_j = ρ_j · L` with `ρ ∈ Δ^{n_p−1}` (`ρ_j > 0`, `Σρ_j = 1`) and `L` the total
digit length. Searching over `ρ` directly wastes budget on infeasible points, so the platform
searches over `u ∈ [0,1]^{n_p−1}` and maps to the simplex with the **stick-breaking** transform:

```
ρ_1 = u_1
ρ_j = u_j · (1 − Σ_{k<j} ρ_k)      for  1 < j < n_p
ρ_{n_p} = 1 − Σ_{k<n_p} ρ_k
```

This is a bijection from the open cube to the open simplex, so every optimizer proposal is feasible
by construction. (It is not volume-preserving; where an unbiased prior matters — random search
baselines — the platform instead samples `ρ ~ Dirichlet(1,…,1)`, which is uniform on `Δ`.)

### 1.2 Canonicalization and symmetry quotient

Digit azimuths `(θ_1,…,θ_{n_d})` on the palm circle are defined up to a global rotation and a
reflection, so the raw parameterization covers each distinct layout `2·n_d` times. Since a symmetry
class evaluates identically, an optimizer exploring it is wasting `2n_d − 1` of every `2n_d`
evaluations. The platform quotients the symmetry by mapping every layout to a canonical
representative:

1. Sort azimuths ascending.
2. Rotate so `θ_1 = 0`.
3. Choose between the layout and its reflection `θ ↦ −θ (mod 2π)` by lexicographic order of the
   resulting gap sequence.

Formally, the search happens on `H/G` with `G = D_{n_d}` acting on azimuths. Thumbs break the
symmetry and are excluded from the reflection step (they are identified by role, not position).

### 1.3 Feasibility constraints

A proposal is rejected before simulation if any of the following fail:

- **Base clearance.** For adjacent digits at azimuths `θ_i, θ_j` mounted at radius `r_m` with
  proximal radii `w_i, w_j`: `2 r_m sin(|θ_i − θ_j|/2) ≥ w_i + w_j + ε`.
- **Opposability.** Digits must be able to reach the palm axis: `L ≥ r_m` (a finger shorter than its
  mount radius can never touch anything the others touch).
- **Object admissibility.** The grasp aperture `2(r_m + L)` must exceed the object's bounding
  dimension, else the trial is *impossible* rather than *failed* — an important distinction that
  keeps hopeless (morphology, object) pairs from polluting success statistics.

---

## 2. Kinematics

### 2.1 Mount frame

For digit azimuth `θ`, elevation `φ`, roll `ψ`, the mount rotation is

```
R = R_z(θ) R_y(−φ) R_x(ψ) ∈ SO(3)
```

with mount position `p = r_m (cos θ, sin θ, 0)ᵀ + (0,0,t_p/2)ᵀ` for a palm of thickness `t_p`.
MJCF takes orientations as unit quaternions in `(w, x, y, z)` order; conversion from `R` uses the
sign-stable Shepperd branch selection (pick the largest of the four diagonal combinations) rather
than the naive `w = ½√(1+tr R)` formula, which loses precision as `tr R → −1`.

### 2.2 Forward kinematics of a digit

Within the mount frame, with flexion angles `q_1..q_{n_p}` about `ŷ_local` (negated, so positive
flexes toward the palm normal) and optional abduction `q_0` about `ẑ_local`:

```
T_0 = Rot_z(q_0)
T_j = T_{j−1} · Rot_y(−q_j) · Trans_x(L_j)
```

The fingertip position is `p_tip = p + R · T_{n_p} · 0`. This closed form is used for workspace
computation and IK without invoking MuJoCo.

### 2.3 Workspace

The reachable set of digit `d` is `W_d = { p_tip(q) : q ∈ Q_d }` with `Q_d` the box of joint limits.
The platform estimates:

- **Volume** by Monte-Carlo over `Q_d` with a voxel occupancy grid at resolution `δ`, giving
  `V̂ = δ³ · |occupied|`; the estimator's standard error is reported.
- **Common workspace** `W_∩ = ⋂_d W_d` — the region where all digits can simultaneously contact,
  which upper-bounds graspable object size and is a far more informative geometric summary than any
  single digit's reach. Computed as the intersection of the voxel grids.
- **Opposition volume**: the subset of `W_∩` where the digits' inward normals positively span ℝ³ —
  i.e. where force closure is geometrically possible at all.

---

## 3. Contact and friction

MuJoCo resolves contacts with a convex, soft-constraint formulation. For our purposes the relevant
facts are:

- Each contact `i` yields a position `p_i`, a frame with unit normal `n_i` (from geom 1 to geom 2),
  two tangents `t_i^1, t_i^2`, a penetration `d_i`, and a friction tuple. `data.contact` gives the
  geometry; `mj_contactForce` gives the 6-vector force/torque in the contact frame.
- The **soft constraint parameters** are `solref = (τ, ζ)` (time constant, damping ratio) and
  `solimp = (d₀, d_∞, width, midpoint, power)`. Compliance in this framework is *not* a spring
  constant but a target impedance: the constraint is driven toward zero violation as a critically-
  damped (for `ζ = 1`) second-order system with time constant `τ`. Stability requires `τ ≳ 2·Δt`;
  the platform validates this at build time and refuses to compile a model that violates it — the
  single most common cause of silent garbage in contact simulation.
- Palm and pad compliance is therefore parameterized by `(τ, ζ)` and `d₀` — physically interpretable
  as "how fast the surface recovers" and "how much it yields" — and these are sweepable research
  variables, not constants (review §3.6).

### 3.1 Coulomb friction cone and its linearization

At contact `i` with friction coefficient `μ_i`, admissible forces form the cone

```
FC_i = { f = f_n n_i + f_t : f_n ≥ 0,  ‖f_t‖ ≤ μ_i f_n }
```

For polytopic computations this is approximated by an `m`-sided pyramid with edge generators

```
f_i^k = n_i + μ_i (cos(2πk/m) t_i^1 + sin(2πk/m) t_i^2),   k = 0..m−1
```

The pyramid is *inscribed* in the cone (conservative: it never claims friction the cone does not
provide). The platform uses `m = 8` by default and reports the value, since ε-metrics are not
comparable across different `m`. Soft-finger contacts optionally add a torsional generator
`±μ_t n_i` about the normal.

---

## 4. Grasp analysis

### 4.1 Grasp matrix

Let the object have centre of mass `c`. A contact force `f_i` applied at `p_i` produces the wrench

```
w_i = [ f_i ,  (p_i − c) × f_i / λ ]ᵀ ∈ ℝ⁶
```

The **torque normalization length** `λ` is required for the force and torque components to be
commensurable; the platform uses `λ = max_i ‖p_i − c‖` (the standard choice) and stores it, because
ε-metrics computed with different `λ` are not comparable — a detail routinely omitted in reported
results.

Stacking the point-contact-with-friction maps gives the grasp matrix

```
G = [G_1 … G_n] ∈ ℝ^{6×3n},    G_i = [ I₃ ; S(p_i − c)/λ ]  restricted to the contact frame
```

so that `w = G λ_c` for contact force vector `λ_c ∈ ℝ^{3n}`.

Derived quantities:

- **Rank**: `rank G < 6` ⟹ some wrench direction is uncontrollable — no force closure regardless of
  friction.
- **Isotropy** `σ_min(G)/σ_max(G) ∈ [0,1]`: how uniformly the grasp can resist wrenches in different
  directions. A high-ε grasp with low isotropy is strong in one direction and fragile in another.
- **Minimum singular value** `σ_min(G)`: worst-case force-transmission ratio.

### 4.2 Grasp wrench space and the Ferrari–Canny ε-metric

Let `W = { w_i^k }` be the set of primitive wrenches from all contacts and all cone edges. Under the
**unit total normal force** convention (`Σ_i f_{n,i} ≤ 1`), the grasp wrench space is

```
GWS = ConvexHull(W ∪ {0})
```

Then:

- **Force closure** ⟺ `0 ∈ int(GWS)` ⟺ the primitive wrenches positively span ℝ⁶.
- **ε-metric** (Ferrari & Canny, 1992):

  ```
  ε = min over facets F of GWS  of  dist(0, F) = min_F  ( −b_F / ‖a_F‖ )
  ```

  where each facet has the halfspace form `a_Fᵀ w ≤ b_F`. Equivalently, `ε` is the radius of the
  largest ball centred at the origin contained in the GWS: **the smallest external wrench magnitude
  that the grasp cannot resist**, in the worst direction. `ε = 0` denotes no force closure.
- **Volume metric** `Q_v = vol(GWS)`: average-case rather than worst-case resistance. Reported
  alongside `ε` because they can disagree, and the disagreement identifies grasps that are strong on
  average but have one fragile direction.

Implementation: `scipy.spatial.ConvexHull` in ℝ⁶ (qhull). With `n ≤ 8` contacts and `m = 8` edges
this is ≤ 64 points, which qhull handles in milliseconds. Degenerate configurations (fewer than 7
affinely independent points, all wrenches in a hyperplane) are detected and reported as `ε = 0`
rather than raising.

### 4.3 Contact distribution metrics

- **Contact count** `n_c`, and **effective digit count** `n_d^eff` = number of *distinct digits* in
  contact (three contacts from one finger is not a three-fingered grasp).
- **Contact spread**: with unit vectors `û_i = (p_i − c)/‖p_i − c‖`, spread is
  `1 − ‖(1/n_c) Σ û_i‖ ∈ [0,1]`. It is 0 when all contacts are on one side (a push, not a grasp) and
  approaches 1 for well-distributed enclosure. This is the resultant-length statistic from
  directional statistics and is exactly the right tool: it is rotation-invariant and needs no
  arbitrary reference axis.
- **Opposition index**: `max_{i,j} (−n̂_i · n̂_j)` — how directly opposed the two most opposed
  contacts are. Near 1 means true opposition; near 0 means the object is only being cradled.
- **Palm involvement**: fraction of total normal force carried by palm geoms. Distinguishes a power
  grasp from a fingertip grasp *quantitatively*, which is what AHAP grasp-type classification needs.

---

## 5. Robustness under disturbance

The empirical counterpart of `ε`, and the metric the review argues should be trusted more.

### 5.1 Disturbance tolerance

From the stored `grasp` snapshot `s*`, for a unit wrench direction `û ∈ S⁵`, define

```
survives(F, û) = 1  if  applying wrench F·û to the object for duration T
                       leaves the object pose within tolerance of its initial palm-frame pose
```

with pose error measured **in the palm frame** (review §3.3):

```
e_pos = ‖ᵖp_obj(T) − ᵖp_obj(0)‖ ,   e_rot = 2 arccos |⟨ᵖq_obj(T), ᵖq_obj(0)⟩|
survives ⟺ e_pos < δ_p  ∧  e_rot < δ_r
```

Crucially the wrench is **constant over the whole window** and the state is **reset from `s*` for
every probe**, so the result is a property of the grasp rather than of the ramp rate or the timestep.

The tolerance in direction `û` is

```
F*(û) = sup { F ≥ 0 : survives(F, û) }
```

found by geometric bracketing (`F = F₀·2^k` until failure, bounded by `F_max`) followed by `K`
bisection steps, costing `⌈log₂(F_max/F₀)⌉ + K ≈ 12` simulations of length `T` per direction — cheap
because each is short and starts from a settled state.

`survives(·, û)` is assumed monotone in `F`; this is true for the overwhelming majority of grasps but
not guaranteed (a grasp can occasionally be *stabilized* by a larger force seating the object). The
platform verifies monotonicity on a subsample and flags violations rather than assuming them away.

### 5.2 Aggregate robustness

Over a direction set `{û_j}` of `N` points from a **Fibonacci sphere** (low-discrepancy on `S²` for
pure forces; a spherical-code construction on `S⁵` when torques are included):

- **Worst-case tolerance** `F_min = min_j F*(û_j)` — the honest number, and the one a grasp fails at.
- **Mean tolerance** `F̄ = (1/N) Σ_j F*(û_j)`.
- **Disturbance polytope volume** `V_D = vol(ConvexHull({F*(û_j)·û_j}))`, normalized by the volume of
  the ball of radius `F̄`, giving an anisotropy measure directly comparable to `ε` vs `Q_v`.
- **Gravity-multiplier tolerance**: the largest `g/g₀` sustained, which is the most physically
  intuitive single number and needs no direction sampling.

The estimator `F_min` over `N` sampled directions is biased upward (the true minimum over the
continuous sphere is at most the sampled minimum). The bias shrinks as `N^{−1/2}` for smooth `F*`;
the platform reports `N` and, optionally, a local refinement descent from the sampled minimizer.

### 5.3 Escape probability

Under domain randomization `ξ ~ p(ξ)` (friction, mass, initial pose, actuator noise), the escape
probability is `P_esc = E_ξ[1 − success(ξ)]`, estimated by Monte Carlo with a Wilson interval. This
is the quantity that actually predicts field performance, and it is the target of Study 4.

### 5.4 Failure taxonomy

Every failure is classified automatically from the trace, in priority order:

| Class | Detection |
|---|---|
| `no_contact` | contact count never exceeded 1 during CLOSE |
| `insufficient_digits` | fewer than 2 distinct digits ever contacted |
| `slip_out` | contacts lost while object translated > δ_p in the palm frame |
| `rotate_out` | contacts lost while object rotated > δ_r |
| `eject_on_lift` | contact lost within the LIFT window, object velocity away from palm |
| `disturbance_failure` | survived LIFT+HOLD, failed the perturbation battery |
| `actuator_saturation` | commanded force at limit for > 90 % of CLOSE with no contact gain |
| `self_collision` | digit–digit contact force exceeded digit–object force |
| `unreachable` | aperture insufficient for object bounding box (excluded from success stats) |
| `sim_divergence` | non-finite state or `mj_warning` raised |

Classes are mutually exclusive by construction, so failure fractions sum to `1 − success rate` and
can be plotted as a stacked bar per morphology — turning "it failed" into "it failed *this way*,
which points at *this* design change".

---

## 6. Effort, actuation and complexity

### 6.1 Energy and torque

```
E = ∫₀^T Σ_a |τ_a(t) · q̇_a(t)| dt        (mechanical work, absolute value: no regeneration credit)
τ_peak = max_{a,t} |τ_a(t)|
utilization_a = (1/T)∫ |τ_a| dt / τ_a^max
saturation_a = (1/T)|{ t : |τ_a| > 0.98 τ_a^max }|
```

Joint utilization identifies actuators that are never worked hard — **direct evidence of a removable
DOF**, and one of the cheapest routes to answering "how many DOF are necessary".

### 6.2 Underactuation via fixed tendons

A `fixed` tendon defines the scalar length `ℓ = Σ_i c_i q_i`. An actuator force `F` applied to the
tendon produces joint torques `τ_i = c_i F`. Combined with per-joint extension springs
`τ_i^spring = −k_i(q_i − q_i^ref)`, equilibrium for an unobstructed joint satisfies

```
c_i F = k_i (q_i − q_i^ref)   ⟹   q_i = q_i^ref + c_i F / k_i
```

so joints close in proportion to `c_i/k_i` until one contacts the object; the contact absorbs that
joint's share of the tendon travel and the remaining joints continue to close. **This reproduces the
proximal-first sequential closure the v1 notes describe achieving physically by differential screw
friction** (review §2.5), with `c_i` as the tendon moment arms — a directly manufacturable quantity
(pulley radii). One actuator drives `n_p` joints, which is why the complexity accounting below
separates joint count from actuator count.

### 6.3 Complexity cost model

```
C = w_a·N_actuators + w_j·N_joints + w_d·N_digits + w_p·N_phalanges
    + w_abd·N_abduction + w_tendon·N_tendon_routes + w_compliant·N_compliant_elements
```

Defaults `w_a = 1.0`, `w_j = 0.6`, `w_d = 0.4`, `w_p = 0.15`, `w_abd = 0.3`, `w_tendon = 0.5`,
`w_compliant = 0.2`, chosen so that an actuator is the dominant unit cost (it is: motor + driver +
encoder + wiring dominates a 3D-printed link). The weights are configuration, not code, and every
reported result carries the weight vector that produced it — because a Pareto front is only
meaningful relative to a stated cost model.

**Sanity check on the current project** (review §4.7): `birh_v1` (3 digits × 2 joints, tendon-coupled,
3 actuators) scores `3.0 + 3.6 + 1.2 + 0.9 + 0 + 1.5 = 10.2`; `birh_v2` (2 fingers + thumb, thumb
abduction, 7 independent actuators) scores `7.0 + 4.2 + 1.2 + 0.9 + 0.3 + 0 = 13.6`. The "refined"
design is 33 % more complex. That inversion is exactly what an explicit cost model is for.

---

## 7. Objectives and aggregation

### 7.1 Per-trial score

```
s = 1[success] · ( α·ŝ_ε + β·ŝ_F + γ·ŝ_c + δ·(1 − ŝ_E) )
```

with each `ŝ` a metric normalized to `[0,1]` by a fixed, object-specific reference scale (so scores
are comparable across objects of different mass). Multiplying by the success indicator rather than
adding it prevents the pathological "high-quality grasp that drops the object" from scoring well.

### 7.2 Aggregation across objects

Per morphology `h`, over objects `o ∈ O` with `n` seeded trials each:

```
S̄(h) = (1/|O|) Σ_o mean_trials s(h,o,·)        mean performance
S_min(h) = min_o mean_trials s(h,o,·)           worst-category performance
S_gen(h) = S̄(h) − std_o( mean_trials s )       generalization-penalized score
```

All three are reported. `S̄` and `S_gen` select different hands, and which one is the objective is a
scientific decision made in the config, never silently in code.

### 7.3 Multi-objective formulation

The project's actual question is the Pareto front of

```
maximize S̄(h) (or S_gen)   ,   minimize C(h)
```

Point `h₁` dominates `h₂` iff it is no worse on both and strictly better on one. NSGA-II ranks by
non-domination front, then by crowding distance

```
d_i = Σ_objectives (f_{i+1} − f_{i−1}) / (f_max − f_min)
```

Front quality is tracked by **hypervolume** relative to a fixed reference point `(S=0, C=C_max)`,
which is the only front-quality measure that is Pareto-compliant. The **knee point** — maximum
curvature of the front, found as the point of greatest distance from the chord joining the extremes
— is reported as the "best value for the complexity" recommendation, which is the answer to *where
do diminishing returns begin*.

---

## 8. Optimization algorithms

**CMA-ES.** Maintains `(m, σ, C)`; samples `x_k = m + σ B D z_k`, `z_k ~ N(0,I)`. Updates the mean by
weighted recombination of the `μ` best, the evolution paths `p_σ, p_c` by cumulation, and `C` by
rank-1 (from `p_c`) plus rank-μ updates. Well suited here: the space is ~11-dimensional, continuous,
non-convex, and the objective is noisy — CMA-ES's covariance adaptation handles the ill-conditioning
introduced by parameters with very different effective scales, which is precisely the situation
(azimuths in radians, stiffness across decades).

**Noise handling.** The objective is stochastic (seeded trials). Two mitigations: (i) fixed common
random numbers across candidates within a generation, which is variance reduction by *pairing* and
typically cuts the effective noise by an order of magnitude; (ii) increasing the trial count with
generation index, `n_g = n₀ + ⌊κ·g⌋`, spending precision only when the population has converged
enough to need it.

**GA.** Simulated binary crossover (SBX, `η_c = 15`) and polynomial mutation (`η_m = 20`) on the
normalized cube, tournament selection, elitism. SBX is used rather than blend crossover because it
respects box bounds without rejection sampling.

**PSO.** `v ← ω v + φ_p r_p (p_best − x) + φ_g r_g (g_best − x)`, with `ω = 0.72`, `φ_p = φ_g = 1.49`
(Clerc's constriction values, which guarantee convergence of the deterministic recurrence).

**Bayesian optimization.** GP surrogate with a Matérn-5/2 kernel (once-differentiable sample paths —
appropriate for a landscape with contact-induced kinks, where the RBF kernel's infinite smoothness
assumption is wrong), automatic relevance determination for per-parameter length scales, and
Expected Improvement

```
EI(x) = (μ(x) − f⁺ − ξ)Φ(z) + σ(x)φ(z),   z = (μ(x) − f⁺ − ξ)/σ(x)
```

ARD length scales are themselves a *result*: a large length scale means the objective barely depends
on that parameter, which is a direct, free sensitivity analysis.

**Random search** is always run as the control. Reporting an optimizer's result without a
matched-budget random-search baseline says nothing about whether the optimizer helped.

---

## 9. Statistics

**Proportions.** Wilson score interval at level `1−α`:

```
(p̂ + z²/2n ± z√(p̂(1−p̂)/n + z²/4n²)) / (1 + z²/n)
```

Used for all success rates. The normal approximation is never used: with `n = 30` and `p̂ = 0.97` it
produces an upper bound above 1.

**Means.** BCa bootstrap with `B = 10 000`, resampling **at the object level** because trials within
an object share the object's difficulty and are not independent. Ignoring this clustering understates
the interval, typically by a factor of √(trials per object).

**Two-group comparison.** Welch's *t* (unequal variances) and Mann–Whitney *U* (no distributional
assumption); both reported, with disagreement flagged as evidence of a distributional difference
rather than a location shift.

**Paired designs.** Ablations vary one factor with object and seed held fixed, so the paired
Wilcoxon signed-rank test applies and is substantially more powerful. The platform detects pairing
from the design and uses the paired test automatically.

**Multiplicity.** Holm–Bonferroni within each family of hypotheses (a family = one factor across all
objects). Holm rather than Bonferroni: uniformly more powerful, same family-wise error control, no
extra assumptions.

**Effect sizes.** Always reported next to *p*: Cohen's `d = (μ₁−μ₂)/s_pooled` for means, Cliff's
`δ = P(X>Y) − P(X<Y)` for ordinal/non-normal outcomes, and the risk difference with its CI for
proportions. With thousands of cheap trials, *p* < 0.001 is easy and uninformative; the effect size
is the finding.

**Power.** For success-rate comparisons, detecting a difference `Δ` at power `1−β` needs roughly

```
n ≈ 2 (z_{1−α/2} + z_{1−β})² p̄(1−p̄) / Δ²
```

per group — `n ≈ 194` per group for `Δ = 0.1` around `p̄ = 0.5`. The platform runs power analysis on
pilot data **before** committing a sweep, so trial counts are chosen rather than guessed.

---

## 10. Sensitivity analysis

**Standardized regression coefficients.** Fit `y = β₀ + Σ β_i x_i` on standardized inputs; `|β_i|`
ranks importance, valid to the extent the model is linear (report `R²` — if it is low, the ranking is
not trustworthy and the next two methods take over).

**Permutation importance.** Fit a gradient-boosted surrogate `f̂`, then measure the increase in error
when feature `i` is permuted. Captures nonlinearity and interaction; correlated features share
credit, so the platform also reports the input correlation matrix beside it.

**Morris elementary effects.** For `r` trajectories with step `Δ`:

```
EE_i = (y(x + Δe_i) − y(x))/Δ ,   μ*_i = mean|EE_i| ,   σ_i = std(EE_i)
```

`μ*` ranks influence; large `σ` signals interaction or nonlinearity. Cost is `r(m+1)` evaluations —
cheap enough to run as a screening pass before any expensive optimization, and the recommended first
experiment on any newly added parameter.

**Sobol indices.** Variance decomposition `V(y) = Σ V_i + Σ V_ij + …` via the Saltelli estimator,
giving first-order `S_i = V_i/V(y)` and total-order `S_{T_i}`. Cost `N(m+2)`; run only on the reduced
parameter set that screening has already identified as important.

**Interpretation caution.** All of these are sensitivity *of the simulated objective*. A parameter's
importance is only as trustworthy as the simulator's fidelity for the mechanism that parameter
controls — which is why Study 1 (calibration) precedes every sensitivity claim.

---

## 11. Numerical hygiene

- **Integrator**: `implicitfast`. Damping and passive forces are integrated implicitly, which is
  stable at timesteps roughly an order of magnitude larger than RK4 permits for the same model
  (review §3.5).
- **Timestep**: default `2×10⁻³ s`, validated by a convergence study — metrics must be invariant to
  halving `Δt` within their reported confidence intervals. **A metric that moves when `Δt` moves is
  a numerical artifact, not a result**, and the platform's calibration report checks this explicitly.
- **`solref` guard**: refuse to compile if `τ < 2Δt`.
- **Divergence detection**: non-finite `qpos`/`qvel`, or MuJoCo warnings, terminate the trial and
  record `sim_divergence` rather than propagating NaNs into the statistics.
- **Determinism**: MuJoCo is deterministic for a fixed model, initial state and control sequence.
  All stochasticity enters through explicitly seeded `SeedSequence` streams, so a result row is a
  pure function of its job identifier — the property that makes resume, caching, and reproduction
  work at all.
