"""
test_model.py
-------------
Validates the single-angle compensation law (model.commanded_direction)
against the *qualitative* behavior Badia Torres et al. (2024) describe in
text for the aggressiveness parameter k (between Eq. 24 and Eq. 25):

    "... k determines the aggressiveness of the compensation, ranging
    from 0 (no compensation at all) through to 1 (only stopping the
    progress of the switch but not returning to the desired
    configuration) and then returning to the desired configuration
    faster with a higher value of k."

These are behavioral/regression tests for this project's simplified
model, not a validation against the paper's own numerical results
(which are not published in machine-readable form).
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from moby_sim import simulate  # noqa: E402


def final_gamma_deg(k, T=6.0):
    res = simulate.run_straight_line(gamma0=np.pi + 0.3, k=k, T=T)
    return np.degrees(res["gamma"][-1]) % 360


def test_k0_diverges_to_push():
    """k=0: no correction -> gamma should drift away from pull (180 deg)
    toward push (0/360 deg)."""
    g_final = final_gamma_deg(k=0.0)
    assert abs(g_final - 180) > 90, f"expected drift away from 180 deg, got {g_final:.1f}"


def test_k1_freezes_drift():
    """k=1: should stop the drift close to its starting point (gamma0 =
    180+ ~17 deg), not return fully to 180 and not diverge to push."""
    g_final = final_gamma_deg(k=1.0)
    assert 185 < g_final < 220, f"expected drift frozen near {180+17:.0f} deg, got {g_final:.1f}"


def test_k_greater_than_1_restores_to_pull():
    """k>1: should actively return close to the pull equilibrium (180 deg)."""
    g_final = final_gamma_deg(k=2.0)
    assert abs(g_final - 180) < 2.0, f"expected near 180 deg, got {g_final:.1f}"


def test_higher_k_converges_faster():
    """Higher k should reach a small error from gamma_goal sooner."""
    def time_to_converge(k, tol_deg=1.0):
        res = simulate.run_straight_line(gamma0=np.pi + 0.3, k=k, T=6.0)
        err = np.abs(np.degrees(res["gamma"]) - 180)
        idx = np.argmax(err < tol_deg)
        return res["t"][idx] if err[idx] < tol_deg else np.inf

    t_k2 = time_to_converge(2.0)
    t_k4 = time_to_converge(4.0)
    assert t_k4 <= t_k2, f"expected k=4 to converge at least as fast as k=2 ({t_k4} vs {t_k2})"


if __name__ == "__main__":
    test_k0_diverges_to_push()
    test_k1_freezes_drift()
    test_k_greater_than_1_restores_to_pull()
    test_higher_k_converges_faster()
    print("All tests passed.")
