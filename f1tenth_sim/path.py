"""
path.py
-------
Reference-path generators for the F1TENTH pure pursuit checks.

`arc_path` mirrors the curvature profile of the test trajectory used in
Badia Torres et al. (2024), Figure 10, for the MOBY robot (a 60-degree arc
of radius 5 m followed by a 90-degree arc of radius 8 m of opposite
curvature), so the F1TENTH results can be read against the same
curvature-change scenario used on the MOBY side of this project.
"""

import numpy as np


def _arc_segment(pts, x, y, theta, radius, angle_deg, n):
    """
    Append n points of a constant-curvature arc to `pts`, starting at
    (x, y, theta). `radius` sign sets turn direction: positive = CCW
    (left turn), negative = CW (right turn). Uses forward Euler
    arc-length integration (robust to the sign of `radius`, unlike a
    naive closed-form rotation formula).
    """
    total_dtheta = np.radians(angle_deg) * np.sign(radius)
    dtheta = total_dtheta / n
    step_len = abs(radius) * abs(dtheta)
    for _ in range(n):
        x += step_len * np.cos(theta + dtheta / 2)
        y += step_len * np.sin(theta + dtheta / 2)
        theta += dtheta
        pts.append((x, y))
    return x, y, theta


def arc_path(ds=0.02, straight_len=5.0):
    """
    Build the MOBY-Fig.-10-style test path: a 60 deg arc (r=5 m), then a
    90 deg arc of opposite curvature (r=8 m), then a straight segment.
    Returns an (N, 2) array of waypoints.
    """
    pts = [(0.0, 0.0)]
    x, y, theta = 0.0, 0.0, 0.0

    n1 = int(np.radians(60) * 5.0 / ds)
    x, y, theta = _arc_segment(pts, x, y, theta, 5.0, 60, n1)

    n2 = int(np.radians(90) * 8.0 / ds)
    x, y, theta = _arc_segment(pts, x, y, theta, -8.0, 90, n2)

    n3 = int(straight_len / ds)
    for _ in range(n3):
        x += ds * np.cos(theta)
        y += ds * np.sin(theta)
        pts.append((x, y))

    return np.array(pts)


def circle_path(radius=5.0, angle_deg=180.0, ds=0.02):
    """A single constant-curvature arc, used for the closed-form
    curvature-matching validation test (kappa_pp should equal 1/radius)."""
    pts = [(0.0, 0.0)]
    n = int(np.radians(angle_deg) * radius / ds)
    _arc_segment(pts, 0.0, 0.0, 0.0, radius, angle_deg, n)
    return np.array(pts)


def local_curvature(path):
    """Discrete curvature of a path (for diagnostic/error-vs-curvature plots)."""
    dx = np.gradient(path[:, 0])
    dy = np.gradient(path[:, 1])
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = (dx**2 + dy**2) ** 1.5
    denom[denom < 1e-9] = 1e-9
    return (dx * ddy - dy * ddx) / denom
