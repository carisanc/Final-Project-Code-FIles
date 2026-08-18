"""
simulate.py
-----------
Forward-Euler time integration of the MOBY internal-dynamics model
(model.py), producing both the internal angle trace gamma(t) and a full
2D trajectory (x, y, theta_c) suitable for a bird's-eye-view plot
comparable to Figures 6/7/9 in Badia Torres et al. (2024).
"""

import numpy as np
from . import model


def run_straight_line(
    d1=0.145,
    c=1.2,
    gamma0=np.pi + 0.3,
    gamma_goal=np.pi,
    k=0.0,
    psi_input=0.0,
    T=6.0,
    dt=1e-3,
):
    """
    Simulate the chassis while the robot is commanded to travel along a
    fixed global direction `psi_input`, starting with an internal chassis
    angle `gamma0` (a disturbance away from `gamma_goal`).

    Returns a dict with time series: t, gamma, theta_c, x, y, psi_commanded.
    """
    n = int(T / dt)
    t = np.linspace(0.0, T, n)

    theta_c = np.zeros(n)
    gamma = np.zeros(n)
    x = np.zeros(n)
    y = np.zeros(n)
    psi_cmd = np.zeros(n)

    theta_c[0] = psi_input + gamma0
    gamma[0] = gamma0

    for i in range(1, n):
        gamma_measured = model.wrap(theta_c[i - 1] - psi_input)
        psi_c = model.commanded_direction(psi_input, gamma_measured, gamma_goal, k)
        gamma_physical = model.wrap(theta_c[i - 1] - psi_c)

        theta_dot = model.natural_heading_rate(gamma_physical, d1, c)

        theta_c[i] = theta_c[i - 1] + theta_dot * dt
        x[i] = x[i - 1] + c * np.cos(psi_c) * dt
        y[i] = y[i - 1] + c * np.sin(psi_c) * dt
        gamma[i] = theta_c[i] - psi_input  # continuous (unwrapped) for correct plotting
        psi_cmd[i] = psi_c

    return {
        "t": t,
        "gamma": gamma,
        "theta_c": theta_c,
        "x": x,
        "y": y,
        "psi_commanded": psi_cmd,
        "params": {"d1": d1, "c": c, "gamma0": gamma0, "gamma_goal": gamma_goal, "k": k},
    }


def curvature_feasibility_sweep(d1=0.145, c=1.2, radii=None):
    """
    For a range of target path radii R (m), compute whether a pull-side
    gamma_goal exists (Eq. 22) and return the feasibility table.

    This is the MOBY-side counterpart to the F1TENTH look-ahead-vs-curvature
    sweep: instead of varying a tunable controller parameter, here the
    "test input" is the commanded path curvature itself, and the output is
    a hard feasibility boundary at R = d1 (Badia Torres et al., 2024,
    Section 5, design rule 2).
    """
    if radii is None:
        radii = np.linspace(0.05, 1.0, 200)

    omega = c / radii  # rad/s implied by driving a circle of radius R at speed c
    gamma_goals = []
    feasible = []
    for w in omega:
        gg = model.gamma_goal_for_circular_path(d1, c, w)
        feasible.append(gg is not None)
        gamma_goals.append(gg if gg is not None else np.nan)

    return {
        "R": radii,
        "omega": omega,
        "gamma_goal": np.array(gamma_goals),
        "feasible": np.array(feasible),
        "R_min": model.min_feasible_turn_radius(d1),
        "params": {"d1": d1, "c": c},
    }
