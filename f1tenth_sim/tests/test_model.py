"""
test_model.py
-------------
Validation tests for the pure pursuit law and kinematic bicycle model,
matching the "Validation and Initial Testing" checks described in the
project report:

  1. Straight-line test: goal point directly ahead (phi=0) -> delta=0.
  2. Constant-curvature test: on a circular path, the curvature implied
     by the goal-point geometry (kappa_pp = 2*x_g/l**2, Coulter, 1992)
     should match the path's actual curvature 1/R for a small enough
     look-ahead distance.
  3. Regression check: mean tracking error should increase with l_d on
     the curvature-varying path (the corner-cutting trend reported in
     Coulter, 1992 and reproduced in the project report).
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from f1tenth_sim import model, path, simulate  # noqa: E402


def test_straight_ahead_goal_gives_zero_steering():
    x_g, y_g, phi = model.goal_point_local_frame((0.0, 0.0), 0.0, (1.0, 0.0))
    delta = model.steering_angle(phi, L=0.33, l_d=1.0)
    assert abs(delta) < 1e-9, f"expected delta=0 for phi=0, got {delta}"


def test_curvature_matches_circle_for_small_lookahead():
    """
    Checks Coulter's raw geometric formula (kappa_pp = 2*lateral/l**2) in
    isolation, using only the FIRST sample (before the vehicle has moved
    at all). This isolates the geometric formula itself from the
    steering law's own tracking bias (see
    test_steering_law_uses_half_coulter_curvature below).
    """
    R = 5.0
    l_d = 0.15
    circ = path.circle_path(radius=R, angle_deg=90.0)
    res = simulate.pure_pursuit_run(circ, l_d, v=1.0, L=0.33, dt=0.01, steps=1,
                                     start_offset=(0.0, 0.0))
    kappa_est = res["kappa_pp"][0]
    kappa_true = 1.0 / R
    rel_err = abs(kappa_est - kappa_true) / kappa_true
    assert rel_err < 0.01, f"expected kappa_pp close to 1/R={kappa_true:.3f}, got {kappa_est:.3f} (rel. err {rel_err:.2%})"


def test_steering_law_uses_half_coulter_curvature():
    """
    Documents a real (not a bug) discrepancy between the two source
    papers: the Evans et al. (2024) steering formula
    delta=arctan(L*sin(phi)/l_d) implements an effective curvature
    tan(delta)/L that is approximately HALF of Coulter's (1992) raw
    kappa_pp = 2*lateral/l**2 for small heading angles. This test pins
    that ratio down numerically so a future change to either formula is
    caught.
    """
    R = 5.0
    l_d = 0.15
    circ = path.circle_path(radius=R, angle_deg=90.0)
    res = simulate.pure_pursuit_run(circ, l_d, v=1.0, L=0.33, dt=0.01, steps=1,
                                     start_offset=(0.0, 0.0))
    x_g, y_g, phi = model.goal_point_local_frame((0.0, 0.0), 0.0, circ[
        np.where(np.hypot(circ[:, 0], circ[:, 1]) >= l_d)[0][0]])
    delta = model.steering_angle(phi, L=0.33, l_d=l_d)
    effective_kappa = np.tan(delta) / 0.33
    ratio = res["kappa_pp"][0] / effective_kappa
    assert 1.8 < ratio < 2.2, f"expected Coulter/Evans curvature ratio near 2, got {ratio:.2f}"


def test_error_increases_with_lookahead():
    ref_path = path.arc_path()
    lookaheads = [0.35, 0.8, 1.6]
    results = simulate.lookahead_sweep(ref_path, lookaheads)
    means = [results[l_d]["lateral_error"].mean() for l_d in lookaheads]
    assert means[0] < means[1] < means[2], f"expected increasing error with l_d, got {means}"


if __name__ == "__main__":
    test_straight_ahead_goal_gives_zero_steering()
    test_curvature_matches_circle_for_small_lookahead()
    test_steering_law_uses_half_coulter_curvature()
    test_error_increases_with_lookahead()
    print("All tests passed.")
