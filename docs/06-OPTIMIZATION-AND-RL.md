# Optimization and Learned Control

*How to search the morphology space, train a controller, and run the two together. Every
command below is runnable today; every number in an example is a suggestion sized for a
20-core machine.*

---

## 1. The objective

An optimizer proposes points in `[0,1]^n`; the objective decodes each into a `HandSpec`,
runs it through the normal experiment runner, and returns a loss. Because it goes through
the runner, an optimization run is also a sweep: every hand tried is in
`runs/<run>/morphologies/`, every trial in `results.jsonl`, and re-visiting a point costs
nothing.

```
loss = −( w_success · success + w_quality · min(ε/ε_ref, 1) + w_lift · lift )
       + w_complexity · complexity / complexity_ref
       + w_feasibility · (1 − feasibility_score)
```

- **success** is averaged *per object first*, then across objects, so one object with many
  trials cannot dominate.
- **ε** (Ferrari–Canny) grades successful grasps; without it two successes are
  indistinguishable and the search has no gradient once it reaches 100 %.
- **complexity** uses the cost model in `docs/02-MATH.md` §6.3; `complexity_ref` is
  `birh_v1`'s value, so the penalty reads as "multiples of the reference hand".
- **feasibility** comes from `morpholab.feasibility` for a chosen material and actuator.
- Designs the validator rejects (self-intersecting digits, impossible proportions) get a
  fixed loss and are never simulated.

Multi-objective mode returns `[−performance, complexity]` and NSGA-II returns the Pareto
front instead of a winner.

---

## 2. Morphology optimization

```bash
# random search: the control condition
morpholab optimize morphology --algorithm random --popsize 32 --generations 20 --objects train -j 20

# CMA-ES, warm-started from the reference hand, with a feasibility penalty
morpholab optimize morphology --algorithm cmaes --generations 30 --popsize 16 \
    --warm-start birh_v1 --w-feasibility 0.3 --material pla --actuator xl330_spool -j 20

# the Pareto front over (performance, complexity)
morpholab optimize morphology --algorithm nsga2 --multi --popsize 32 --generations 40 -j 20
```

Each run directory holds `history.jsonl` (one line per hand), `best_hand.json`,
`pareto.json` (multi-objective), `checkpoints/optimizer.json`, and the usual
`results.jsonl` + `morphologies/`. Re-running the same command resumes from the last
generation. At the end the best hand is evaluated on the held-out `test` object set and
the train/test gap is printed; **that gap is the number to report**.

| Algorithm | Use when | Notes |
|---|---|---|
| `random` | always, first | the baseline every other algorithm must beat at matched budget |
| `cmaes` | continuous spaces, 5–50 dims | needs `pip install cma`; `es` is the dependency-free fallback |
| `ga` | spaces with booleans/categoricals | SBX + polynomial mutation; population is inspectable |
| `nsga2` | you want the front, not a point | `--multi`; hypervolume logged per generation |

Budget arithmetic: at ~0.3 s per trial, `popsize 16 × 15 objects × 2 trials` is 480 trials
per generation, about 10 s on 20 workers. Thirty generations is five minutes. The
expensive part is never the optimizer.

**Validate an optimizer before trusting it.** `three_finger_layout` is two-dimensional, so
a 40×40 grid is ground truth; run CMA-ES on it and check it lands on the grid optimum.

---

## 3. Learned control by evolution strategies

A policy is a small numpy MLP (`morpholab.rl.MLPPolicy`) wrapped as a `Controller`
(`PolicyController`), so it runs under every task and produces the same result rows as
the scripted controllers. Observation: joint positions and velocities (padded to a fixed
layout), per-digit contact force, object position and velocity in the palm frame, the
task's grip word, phase progress, and the current actuator targets. Action: a per-actuator
target *rate*.

**Residual mode** (default) adds the policy to a scripted closure ramp, so a zero policy
already grasps and the search refines rather than discovers. `--no-residual` removes the
crutch.

```bash
morpholab optimize policy --hand birh_v1 --objects quick --tasks lift --iterations 60 --popsize 16 -j 20
morpholab optimize policy --hand runs/optimize-cmaes/<run>/best_hand.json --objects train --iterations 100 -j 20
```

Training uses OpenAI-style ES: antithetic Gaussian perturbations, centered-rank fitness
shaping, an Adam step. Each iteration evaluates `2 × popsize` policies on every
(object, task) pair; with `popsize 16` and four objects that is 128 rollouts, about 3 s on
20 cores. Checkpoints (`es_state.npz`, `best_policy.json`) land in the run's
`checkpoints/` and resume automatically.

Why ES rather than PPO here: trials are short, the reward is mostly terminal, the
simulator is not differentiable, and evaluation parallelizes over processes with no
shared model. For harder tasks (in-hand reorientation) `morpholab.rl.GraspEnv` is a
Gymnasium-compatible environment:

```python
from morpholab.rl import make_env
env = make_env("birh_v1", "marker", "reorient", frame_skip=10)
obs, info = env.reset(seed=0)
obs, r, terminated, truncated, info = env.step(env.action_space.sample())
```

With `gymnasium` and `stable-baselines3` installed, `PPO("MlpPolicy", env)` works
directly. Neither is a dependency of the platform.

---

## 4. Co-design

```bash
morpholab optimize codesign --generations 10 --popsize 8 --inner-iterations 12 --objects quick -j 20
```

Outer loop: CMA-ES over the morphology space. Inner loop: ES trains a policy for each hand,
**warm-started from the best policy so far** (policy inheritance, which is what makes the
inner loop affordable). Each hand is scored twice — under the fixed `ramp` controller and
under its trained policy — and the run's summary reports the **Spearman rank correlation**
between the two. That number is the answer to "how much of the fixed-controller ranking
was an artifact of the controller?" (`docs/00-REVIEW.md` §7, Study 5). Below ~0.6, prior
rankings need a caveat; above ~0.8, the cheap loop is trustworthy.

Cost: `popsize 8 × 12 iterations × 32 rollouts × 4 objects` ≈ 12,000 rollouts per outer
generation, a few minutes on 20 cores. Ten generations is under an hour.

---

## 5. Feasibility

```bash
python -m morpholab.feasibility birh_v1 --material pla --actuator xl330_spool
python -m morpholab.feasibility birh_v2 --matrix                # every material x actuator
python -m morpholab.feasibility runs/<run>/morphologies --csv feasibility.csv
```

See `docs/05-DESIGN-SPACE.md` §5 for what each check means. In an optimization,
`--w-feasibility 0.3` turns the score into a penalty; in analysis, join the CSV on
`morphology_id`.

---

## 6. Caveats that still apply

- Everything here inherits the simulator's calibration status (`docs/04-STATUS.md` §2.4).
  An optimizer will exploit any artifact it can find; the timestep-convergence check and
  the force-plausibility flag are the first defence.
- The approach planner still places opposed hands badly (§2.1 there). Until it is fixed a
  search will learn to avoid thumbs for the wrong reason.
- Seeds derive from job content, so two hands never share random numbers. Comparisons
  between hands are therefore unpaired at the trial level; use enough trials.
- `PolicyController` is not registered in the controller registry by default, so
  `morpholab run --controller policy` does not work yet; load a saved policy in Python via
  `MLPPolicy.from_dict` and `rollout_policy` instead. Registering it is a one-line change
  in `morpholab/control/__init__.py`, deferred while a sweep is running against that module.
