"""
model.py
--------
Closed-form equations for the offset-differential (MOBY) push/pull instability,
reproduced from:

    J. Badia Torres, A. Perez Gracia, and C. Domenech-Mestres, "Driving
    Strategies for Omnidirectional Mobile Robots with Offset Differential
    Wheels," Robotics, vol. 13, no. 19, 2024.  (Eq. 17-25)

Scope and simplifications (read this before using the results in a report):

- This module implements the SCALAR internal-dynamics model the paper itself
  uses to explain and characterize the push/pull instability (Section 4,
  building on Yun & Yamamoto, 1997). It does NOT reproduce the full
  multibody/Lagrangian model (augmented Jacobian, caster wheel dynamics,
  torque calculations) from Section 6 onward of the paper -- that is a
  separate, much larger modeling effort outside the scope of this
  comparative-analysis project.
- The compensation law implemented in `compensated_heading_rate` is a
  single-angle reformulation of the paper's velocity-mirroring algorithm
  (Eq. 23-25), not a re-implementation of the 2D rotation-matrix form. It was
  derived independently for this project and its correctness is checked
  against the paper's own qualitative description of the aggressiveness
  parameter k (see `README.md`, "Validation" section, and
  `tests/test_model.py`).
"""

import numpy as np


def wrap(angle):
    """Wrap an angle (rad) to (-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def natural_heading_rate(gamma, d1, c):
    """
    Uncompensated internal dynamics of the chassis angle gamma.

    Reproduces Eq. (20) of Badia Torres et al. (2024):
        gamma_dot = -(sin(gamma) / d1) * c

    gamma = 0   -> pure push (stable equilibrium)
    gamma = pi  -> pure pull (unstable equilibrium)

    Parameters
    ----------
    gamma : float or ndarray
        Angle [rad] between the pure-push direction and the desired velocity.
    d1 : float
        Fixed offset between the actuated-wheel axis and the vertical
        rotation axis [m]. Structural parameter of the robot.
    c : float
        Robot speed [m/s].
    """
    return -(np.sin(gamma) / d1) * c


def turn_rate(gamma, d1):
    """
    Instantaneous turn rate 1/R_turn as a function of gamma, Eq. (21):
        1/R_turn = -sin(gamma) / d1
    """
    return -np.sin(gamma) / d1


def gamma_goal_for_circular_path(d1, c, omega):
    """
    Target gamma for tracking a circular path of angular speed omega,
    Eq. (22):  gamma_goal = pi - arcsin(d1 * omega / c)

    Returns None if the requested curvature is not achievable while
    remaining near the pull-side equilibrium (i.e., |d1*omega/c| > 1),
    which is the closed-form version of the paper's design rule
    "avoid turns with a radius smaller than d1" (Section 5).
    """
    arg = d1 * omega / c
    if abs(arg) > 1.0:
        return None
    return np.pi - np.arcsin(arg)


def min_feasible_turn_radius(d1):
    """
    Smallest path radius R = c/omega for which a pull-side equilibrium
    exists, from the feasibility condition |d1*omega/c| <= 1  =>  R >= d1.
    This is the closed-form counterpart to Badia Torres et al. (2024),
    Section 5, design rule 2 ("turns with a radius smaller than d1 ...
    should be avoided").
    """
    return d1


def commanded_direction(psi_input, gamma_measured, gamma_goal, k):
    """
    Velocity-mirroring compensation, single-angle form.

    The paper (Eq. 23-25) corrects the *commanded* velocity direction by
    mirroring/rotating the operator's input velocity as a function of the
    tracking error gamma_err = gamma - gamma_goal, with an aggressiveness
    parameter k in [0, 1] -> [1, inf):
        k = 0 : no correction (open-loop)
        k = 1 : freezes the drift (Eq. 25 special case) without returning
                to gamma_goal
        k > 1 : actively restores gamma toward gamma_goal

    This single-angle form was derived for this project as:
        psi_commanded = psi_input + k * gamma_err

    and reproduces exactly the qualitative k=0 / k=1 / k>1 behavior
    described in the paper (see README "Validation").

    Parameters
    ----------
    psi_input : float
        Heading [rad, global frame] the operator/planner originally wants
        to travel along (the uncorrected target direction).
    gamma_measured : float
        Current gamma measured relative to psi_input.
    gamma_goal : float
        Desired gamma (e.g., pi to remain in the pull region).
    k : float
        Compensation aggressiveness.

    Returns
    -------
    psi_commanded : float
        The actual direction sent to the low-level wheel controller.
    """
    gamma_err = wrap(gamma_measured - gamma_goal)
    return psi_input + k * gamma_err
