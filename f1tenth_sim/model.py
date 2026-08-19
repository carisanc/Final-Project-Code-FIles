"""
model.py
--------
Kinematic single-track (bicycle) model and pure pursuit control law for
F1TENTH, following:

    R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking
    Algorithm," Technical Report CMU-RI-TR-92-01, Carnegie Mellon
    University, 1992.  (goal-point geometry / curvature derivation)

    B. D. Evans, R. Trumpp, M. Caccamo, F. Jahncke, J. Betz, H. W.
    Jordaan, and H. A. Engelbrecht, "Unifying F1TENTH Autonomous Racing:
    Survey, Methods and Benchmarks," arXiv:2402.18558, 2024.
    (F1TENTH-specific steering-angle formula)

Notation matches the disambiguated table used in the project report:
    (x_g, y_g)  goal point in the vehicle's LOCAL frame (not the same
                (x, y) as the vehicle's GLOBAL position used elsewhere)
    l, l_d      look-ahead distance (same quantity, l is Coulter's
                original name, l_d is the F1TENTH-literature name)
    r           radius of the pure-pursuit arc (not MOBY's wheel radius R)
    kappa_pp    curvature of the pure-pursuit arc, kappa_pp = 1/r
    phi         heading angle between the vehicle and the goal point
    delta       front steering angle (Ackermann control input)
    L           vehicle wheelbase

Scope and simplifications:
    This is a constant-speed KINEMATIC simulation (no tire slip, no load
    transfer, no actuator dynamics). It is meant as a fast, independent
    check of the pure pursuit law prior to porting it onto the full
    f1tenth_gym_ros ROS2 stack (dynamic single-track model + real LiDAR
    localization), not a replacement for that closed-loop validation.
"""

import numpy as np


def wrap(angle):
    """Wrap an angle (rad) to (-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def pure_pursuit_curvature(lateral_offset, l):
    """
    Curvature of the pure-pursuit tracking arc, Coulter (1992):
        kappa_pp = 1/r = 2*x / l**2

    IMPORTANT convention note: Coulter's original paper defines its local
    x-axis ALONG the rear axle (i.e., LATERAL to the direction of travel),
    so his "x" is a lateral offset, not a forward distance. This project's
    own local-frame convention (see report Notation table and
    `goal_point_local_frame` below) instead uses the common robotics
    convention x_g = forward, y_g = lateral. To apply Coulter's formula
    correctly under that convention, this function must be called with
    the LATERAL coordinate (y_g), not x_g:

        kappa = pure_pursuit_curvature(y_g, l_d)   # correct
        kappa = pure_pursuit_curvature(x_g, l_d)   # WRONG (mixes conventions)

    lateral_offset : the goal point's lateral coordinate (Coulter's "x",
        this project's y_g).
    l : look-ahead distance (chord length to the goal point).
    """
    return 2.0 * lateral_offset / l**2


def steering_angle(phi, L, l_d):
    """
    F1TENTH steering-angle formula (Evans et al., 2024):
        delta = arctan(L * sin(phi) / l_d)

    phi   : angle [rad] between the vehicle heading and the goal point
    L     : wheelbase [m]
    l_d   : look-ahead distance [m]

    NOTE on a real (not a bug) discrepancy between sources: this formula
    implies an effective tracking curvature of tan(delta)/L = sin(phi)/l_d,
    which for a goal point at chord length l (sin(phi) = lateral_offset/l)
    is approximately lateral_offset/(l_d*l) -- about HALF of Coulter's own
    curvature formula kappa_pp = 2*lateral_offset/l**2 for l ~ l_d. The two
    papers are defining the "curvature to command" slightly differently
    (Coulter derives the curvature of the unique circular arc through the
    goal point; the Evans et al. formula is the commonly-implemented
    steering law in the F1TENTH codebase). Both are used in this project:
    `pure_pursuit_curvature` for the closed-form geometric check against a
    known path curvature (see tests/test_model.py), and `steering_angle`
    (this function) for actually driving the bicycle model, matching what
    the ROS2 pure pursuit node will use in Phase 3.
    """
    return np.arctan2(L * np.sin(phi), l_d)


def goal_point_local_frame(vehicle_xy, vehicle_theta, goal_xy):
    """
    Transform a global goal point into the vehicle's local frame and
    return (x_g, y_g, phi), where phi = atan2(y_g, x_g).
    """
    dx = goal_xy[0] - vehicle_xy[0]
    dy = goal_xy[1] - vehicle_xy[1]
    x_g = np.cos(-vehicle_theta) * dx - np.sin(-vehicle_theta) * dy
    y_g = np.sin(-vehicle_theta) * dx + np.cos(-vehicle_theta) * dy
    phi = np.arctan2(y_g, x_g)
    return x_g, y_g, phi


def bicycle_step(x, y, theta, v, delta, L, dt):
    """
    One forward-Euler step of the kinematic bicycle model:
        x_dot     = v * cos(theta)
        y_dot     = v * sin(theta)
        theta_dot = (v / L) * tan(delta)
    """
    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + (v / L) * np.tan(delta) * dt
    return x_new, y_new, theta_new
