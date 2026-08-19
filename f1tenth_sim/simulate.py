"""
simulate.py
-----------
Closed-loop simulation of the pure pursuit controller (model.py) tracking
a reference path (path.py) with the kinematic bicycle model.
"""

import numpy as np
from . import model


def pure_pursuit_run(path, l_d, v=2.0, L=0.33, dt=0.02, steps=2500, start_offset=(-0.05, 0.0)):
    """
    Run pure pursuit on `path` with look-ahead distance `l_d`.

    Parameters
    ----------
    path : (N, 2) ndarray
        Reference waypoints (global frame).
    l_d : float
        Look-ahead distance [m].
    v : float
        Constant longitudinal speed [m/s].
    L : float
        Vehicle wheelbase [m] (approximate F1TENTH value: 0.33 m).
    dt : float
        Integration step [s].
    steps : int
        Maximum number of steps.
    start_offset : (float, float)
        Initial (x, y) offset from path[0], to avoid a degenerate zero
        look-ahead search at t=0.

    Returns
    -------
    dict with trajectory (x, y, theta), lateral error, and the goal-point
    curvature kappa_pp actually commanded at each step.
    """
    x = path[0, 0] + start_offset[0]
    y = path[0, 1] + start_offset[1]
    theta = 0.0

    traj_x, traj_y, traj_theta = [x], [y], [theta]
    errs = []
    kappas = []
    idx = 0

    for _ in range(steps):
        d = np.hypot(path[idx:, 0] - x, path[idx:, 1] - y)
        cand = np.where(d >= l_d)[0]
        if len(cand) == 0:
            break
        idx = idx + cand[0]
        goal = path[idx]

        x_g, y_g, phi = model.goal_point_local_frame((x, y), theta, goal)
        delta = model.steering_angle(phi, L, l_d)
        # kappa_pp uses the ACTUAL chord distance to the selected goal point
        # (which can differ slightly from the nominal l_d due to the
        # discrete path search), matching Coulter's exact definition of l.
        actual_l = np.hypot(x_g, y_g)
        kappa = model.pure_pursuit_curvature(y_g, actual_l)

        x, y, theta = model.bicycle_step(x, y, theta, v, delta, L, dt)

        traj_x.append(x)
        traj_y.append(y)
        traj_theta.append(theta)
        errs.append(np.min(np.hypot(path[:, 0] - x, path[:, 1] - y)))
        kappas.append(kappa)

        if idx >= len(path) - 2:
            break

    return {
        "x": np.array(traj_x),
        "y": np.array(traj_y),
        "theta": np.array(traj_theta),
        "lateral_error": np.array(errs),
        "kappa_pp": np.array(kappas),
        "params": {"l_d": l_d, "v": v, "L": L, "dt": dt},
    }


def lookahead_sweep(path, lookaheads, **kwargs):
    """Run pure_pursuit_run for each look-ahead distance in `lookaheads`."""
    return {l_d: pure_pursuit_run(path, l_d, **kwargs) for l_d in lookaheads}
