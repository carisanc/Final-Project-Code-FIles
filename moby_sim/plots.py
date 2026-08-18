"""
plots.py
--------
Figure-generation functions for the MOBY independent numerical verification.
Kept separate from simulate.py so the numerical results can be reused
(e.g., exported to CSV) without requiring matplotlib.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plot_gamma_traces(results_by_label, d1_values_for_sensitivity, save_path):
    """
    Left panel: gamma(t) [deg] for each labeled run (e.g., k=0, k=1, k=2).
    Right panel: structural sensitivity 1/R_turn = -sin(gamma)/d1 for a few
    values of d1, to show that this is a fixed design parameter, not a
    runtime control knob.
    """
    from . import model

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))

    ax = axes[0]
    colors = ["#d62728", "#ff7f0e", "#2ca02c", "#1f77b4"]
    for (label, res), color in zip(results_by_label.items(), colors):
        ax.plot(res["t"], np.degrees(res["gamma"]), color=color, lw=2, label=label)
    ax.axhline(180, color="gray", ls="--", lw=1, label=r"Pure pull ($\gamma=\pi$)")
    ax.axhline(0, color="gray", ls=":", lw=1, label=r"Pure push ($\gamma=0$)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"$\gamma$ [deg]")
    ax.set_title("MOBY: independent numerical reproduction\nof the push/pull instability (Eq. 20, 23-25)")
    ax.legend(fontsize=8, loc="center right")
    ax.grid(alpha=0.3)

    ax2 = axes[1]
    gam = np.linspace(0.01, 2 * np.pi - 0.01, 400)
    styles = ["--", "-", ":"]
    for d1_test, style in zip(d1_values_for_sensitivity, styles):
        ax2.plot(np.degrees(gam), model.turn_rate(gam, d1_test), style, label=f"d1 = {d1_test} m")
    ax2.axvline(180, color="gray", lw=0.8)
    ax2.axvline(0, color="gray", lw=0.8)
    ax2.set_xlabel(r"$\gamma$ [deg]")
    ax2.set_ylabel(r"$1/R_{turn} = -\sin(\gamma)/d_1$  [1/m]")
    ax2.set_title("Structural sensitivity to d1\n(fixed by design, not tunable)")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=200, facecolor="white")
    plt.close(fig)


def _draw_chassis(ax, x, y, theta_c, arrow_len, color="#888888"):
    """Small arrow indicating chassis orientation (length scaled to the
    plot's own axis range so it stays legible regardless of the very
    different magnitudes of x-travel vs. lateral deviation)."""
    dx = arrow_len * np.cos(theta_c)
    dy = arrow_len * np.sin(theta_c)
    ax.annotate("", xy=(x + dx, y + dy), xytext=(x, y),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6),
                zorder=5)
    ax.plot(x, y, "o", color=color, markersize=3.5, zorder=6)


def plot_trajectory_birdseye(results_by_label, save_path, n_icons=12):
    """
    Bird's-eye-view trajectory plot with chassis-orientation arrows along
    the path, comparable in spirit to Figures 6/7/9 of Badia Torres et al.
    (2024) (full-scale chassis icons are not legible here because the
    lateral deviation is only a few cm while the travel distance is
    meters; orientation is instead shown with short arrows).
    """
    fig, ax = plt.subplots(figsize=(9, 3.6))
    colors = ["#d62728", "#2ca02c"]

    y_all = np.concatenate([res["y"] for res in results_by_label.values()])
    x_all = np.concatenate([res["x"] for res in results_by_label.values()])
    y_pad = max(0.03, 0.5 * (y_all.max() - y_all.min() + 1e-6))
    arrow_len = 0.03 * (x_all.max() - x_all.min())

    for (label, res), color in zip(results_by_label.items(), colors):
        ax.plot(res["x"], res["y"], color=color, lw=1.8, label=label, zorder=2)
        idxs = np.linspace(0, len(res["x"]) - 1, n_icons).astype(int)
        for i in idxs:
            _draw_chassis(ax, res["x"][i], res["y"][i], res["theta_c"][i], arrow_len, color=color)

    ax.set_ylim(y_all.min() - y_pad, y_all.max() + y_pad)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]  (exaggerated vs. x)")
    ax.set_title("MOBY: chassis trajectory and orientation\n(bird's-eye view, cf. Badia Torres et al. 2024, Fig. 6-7)")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=200, facecolor="white")
    plt.close(fig)


def plot_curvature_feasibility(sweep, save_path):
    """
    Plot gamma_goal vs. target path radius R, shading the infeasible
    region (R < d1) where no pull-side equilibrium exists for the
    commanded curvature -- the closed-form counterpart to Badia Torres
    et al. (2024), Section 5, design rule 2.
    """
    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    R = sweep["R"]
    gg = np.degrees(sweep["gamma_goal"])
    ax.plot(R, gg, color="#1f77b4", lw=2)
    ax.axvline(sweep["R_min"], color="#d62728", ls="--", lw=1.5,
               label=f"R = d1 = {sweep['R_min']} m (feasibility limit)")
    ax.axvspan(R.min(), sweep["R_min"], color="#d62728", alpha=0.08)
    ax.text((R.min() + sweep["R_min"]) / 2, 90, "infeasible\n(R < d1)",
            ha="center", fontsize=9, color="#d62728")
    ax.set_xlabel("target path radius R [m]")
    ax.set_ylabel(r"$\gamma_{goal}$ [deg]")
    ax.set_title("MOBY: curvature feasibility for pull-region tracking\n(Eq. 22)")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=200, facecolor="white")
    plt.close(fig)
