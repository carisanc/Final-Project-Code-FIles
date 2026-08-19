# f1tenth_sim

Standalone kinematic pure pursuit simulation for F1TENTH — the
**parametric-instability** counterpart to `moby_sim` (structural instability)
in the comparative project *"Structural and Parametric Instability in
Trajectory-Tracking Control: MOBY vs. F1TENTH."*

## Why this exists

Before porting pure pursuit onto the full `f1tenth_gym_ros` ROS2 stack (which
needs the SLAM map, tuned AMCL localization, and the minimum-curvature raceline
already built for this project), this package validates the control law itself
— in isolation, at low computational cost — against a path built to match the
curvature profile of MOBY's own test trajectory (Badia Torres et al., 2024,
Figure 10). This gives a same-scenario, curvature-based point of comparison
between the two platforms without waiting on the full closed-loop stack.

## Source equations

- **Coulter (1992)** — pure pursuit goal-point geometry and curvature:
  `kappa_pp = 1/r = 2 * lateral_offset / l**2`
- **Evans et al. (2024)** — F1TENTH steering-angle formula:
  `delta = arctan(L * sin(phi) / l_d)`
- Kinematic bicycle model: `x_dot=v*cos(theta)`, `y_dot=v*sin(theta)`,
  `theta_dot=(v/L)*tan(delta)`

## A real subtlety this package documents (not a bug)

Coulter's raw curvature formula and the Evans et al. steering formula are
**not the same definition of curvature**. For small heading angles, the
curvature implied by `steering_angle` (`tan(delta)/L`) is approximately
**half** of Coulter's `pure_pursuit_curvature`. Both are used deliberately in
this package for different purposes:

- `pure_pursuit_curvature` — closed-form geometric check against a known path
  curvature (`tests/test_model.py::test_curvature_matches_circle_for_small_lookahead`).
- `steering_angle` — the actual control law driving the bicycle model, i.e.
  what the ROS2 node will use in Phase 3.

`tests/test_model.py::test_steering_law_uses_half_coulter_curvature` pins this
ratio down numerically so it doesn't silently drift if either formula changes.

## Notation

Matches the disambiguated table used in the project report:

| Symbol | Meaning |
|---|---|
| `(x_g, y_g)` | goal point in the vehicle's **local** frame (x_g = forward, y_g = lateral) — **not** Coulter's own axis convention, see `model.py` docstring |
| `l`, `l_d` | look-ahead distance |
| `r` | radius of the pure-pursuit arc |
| `kappa_pp` | curvature of the pure-pursuit arc |
| `phi` | heading angle to the goal point |
| `delta` | front steering angle (control input) |
| `L` | wheelbase |

## What is (and is not) simulated here

**Simulated:** a constant-speed kinematic bicycle model with the pure pursuit
control law, run open-loop against a known reference path (no sensor noise,
no localization error, no tire slip).

**Not simulated:** the full `f1tenth_gym_ros` dynamic single-track model,
real LiDAR-based localization (AMCL), or the actual minimum-curvature raceline
built for this project (that closed-loop validation is Phase 3's remaining
task, see the project report).

## Files

```
f1tenth_sim/
  model.py       bicycle model, pure pursuit law, curvature formulas (+ the factor-of-2 note)
  path.py        reference path generators (MOBY-Fig.-10-style arc path, circle path)
  simulate.py     closed-loop pure pursuit simulation loop
  plots.py        figure generation (matplotlib)
  run_all.py      reproduces every figure/CSV used in the report
  tests/
    test_model.py  validation tests (zero-steering, curvature match, factor-of-2, error trend)
  outputs/        generated figures + CSV (created by run_all.py, not tracked in git)
```

## How to run

```bash
cd <repo_root>
python3 -m f1tenth_sim.run_all
```

Validate first (recommended):

```bash
python3 f1tenth_sim/tests/test_model.py
```

Result: 4/4 checks pass (zero steering on a straight-ahead goal, curvature
matches 1/R for a known circle, the Coulter/Evans curvature ratio is ~2,
mean tracking error increases monotonically with l_d).

## Parameters used

- `L = 0.33 m` — approximate F1TENTH wheelbase
- `v = 2.0 m/s` — constant speed (simplified constant-speed pass; the real
  gym stack uses the dynamic single-track model instead)
- `l_d in {0.35, 0.8, 1.6} m` — look-ahead sweep

## Dependencies

```
numpy
matplotlib
```
