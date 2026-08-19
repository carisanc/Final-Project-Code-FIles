"""
plots.py
--------
Figure-generation functions for the F1TENTH independent pure pursuit checks.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plot_trajectory_and_error(path, results_by_ld, save_path, dt=0.02, v=2.0):
    """
    Left panel: reference path + tracked trajectories for each look-ahead.
    Right panel: lateral tracking error vs. distance traveled.
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    colors = ["#d62728", "#2ca02c", "#1f77b4", "#9467bd", "#8c564b"]

    ax = axes[0]
    ax.plot(path[:, 0], path[:, 1], "--", color="gray", lw=1.6,
            label="Reference path\n(same curvature profile as MOBY Fig. 10)")
    for (l_d, res), c in zip(results_by_ld.items(), colors):
        ax.plot(res["x"], res["y"], color=c, lw=1.8, label=f"pure pursuit, l_d={l_d} m")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("F1TENTH: pure pursuit tracking\non a curvature-varying path")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax2 = axes[1]
    for (l_d, res), c in zip(results_by_ld.items(), colors):
        errs = res["lateral_error"]
        dist = np.arange(len(errs)) * dt * v
        ax2.plot(dist, errs, color=c, lw=1.6,
                 label=f"l_d={l_d} m  (mean={errs.mean():.3f} m, max={errs.max():.3f} m)")
    ax2.set_xlabel("distance traveled [m]")
    ax2.set_ylabel("lateral tracking error [m]")
    ax2.set_title("Tracking error vs. look-ahead distance\nacross the curvature-varying segments")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=200, facecolor="white")
    plt.close(fig)
