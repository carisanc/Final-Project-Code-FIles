# moby_sim

Independent numerical verification of the **offset-differential (MOBY) push/pull
instability**, for the comparative project *"Structural and Parametric Instability
in Trajectory-Tracking Control: MOBY vs. F1TENTH."*

## Why this exists

The MOBY side of this project is a literature-based paper critique (Badia Torres
et al., 2024) — there is no physical offset-differential robot available to test.
That left an asymmetry: F1TENTH would have a real ROS2/gym simulation, while MOBY
would only have "we read the equations." This package closes that gap by turning
the paper's own **closed-form internal-dynamics equations** (Eq. 17–25) into a
runnable numerical simulation, so both platforms in the comparison are backed by
an independently produced simulation, not just one of them.

## Source equations (Badia Torres, Perez Gracia, Domenech-Mestres, 2024)

- **Eq. 20** — natural (uncompensated) dynamics: `gamma_dot = -(sin(gamma)/d1) * c`
- **Eq. 21** — instantaneous turn rate: `1/R_turn = -sin(gamma)/d1`
- **Eq. 22** — target gamma for a circular path of angular speed `omega`:
  `gamma_goal = pi - arcsin(d1*omega/c)`
- **Eq. 23–25** — velocity-mirroring compensation with aggressiveness `k`:
  `k=0` no correction, `k=1` freezes the drift, `k>1` restores toward `gamma_goal`.

`gamma = 0` is the stable **pure push** equilibrium (wheels trail the center);
`gamma = pi` is the unstable **pure pull** equilibrium (wheels lead the center).

## What is (and is not) reproduced here

**Reproduced:** the scalar internal-dynamics model the paper itself uses to
*explain* the instability (Section 4), including a working implementation of the
compensation law's documented behavior for `k`.

**Not reproduced:** the full multibody/Lagrangian model (augmented Jacobian,
caster-wheel dynamics, torque calculations, Section 6 onward). That is a
separate, much larger modeling effort and is out of scope for this
comparative-analysis project.

**Compensation law caveat:** `model.commanded_direction` is a **single-angle
reformulation** of the paper's 2D velocity-mirroring algorithm (Eq. 23–25),
derived independently for this project — it is not a line-by-line
re-implementation of the rotation-matrix form. It was validated by checking that
it reproduces the paper's own *documented qualitative behavior* for `k` (see
"Validation" below), not by comparing against the paper's numerical results
(which are not published in machine-readable form).

## Validation

`tests/test_model.py` checks the implementation against this description from
the paper (text between Eq. 24 and Eq. 25):

> "k determines the aggressiveness of the compensation, ranging from 0 (no
> compensation at all) through to 1 (only stopping the progress of the switch
> but not returning to the desired configuration) and then returning to the
> desired configuration faster with a higher value of k."

Run the tests:

```bash
python -m pytest moby_sim/tests/   # or: python moby_sim/tests/test_model.py
```

Result: all 4 checks pass (k=0 diverges to push, k=1 freezes near the
disturbed value, k>1 restores to pull, higher k converges faster).

## Files

```
moby_sim/
  model.py       core equations (natural dynamics, compensation law, curvature feasibility)
  simulate.py    forward-Euler time integration -> gamma(t), (x,y,theta_c)(t)
  plots.py       figure generation (matplotlib)
  run_all.py     reproduces every figure/CSV used in the report
  tests/
    test_model.py  behavioral validation of the compensation law
  outputs/       generated figures + CSV (created by run_all.py, not tracked in git)
```

## How to run

```bash
cd <repo_root>
python -m moby_sim.run_all
```

This writes to `moby_sim/outputs/`:

- `moby_gamma_traces.png` — gamma(t) for k=0/1/2, plus turn-rate sensitivity to d1
- `moby_trajectory_birdseye.png` — 2D trajectory with chassis-orientation arrows
- `moby_curvature_feasibility.png` — minimum feasible turn radius (R = d1)
- `gamma_traces.csv`, `curvature_feasibility.csv` — raw numerical results

## Parameters used

- `d1 = 0.145 m` — real MOBY robot offset (Badia Torres et al., 2024)
- `c = 1.2 m/s` — cruise speed used in the paper's own simulations (Section 7.1)

## Dependencies

```
numpy
matplotlib
```
