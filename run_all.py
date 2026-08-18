"""
run_all.py
----------
Reproduces all MOBY independent-verification results used in the project
report:

  1. gamma(t) for k = 0 (uncompensated), k = 1 (freezes drift), k = 2
     (actively restores toward the pull equilibrium) -- plus the
     structural sensitivity of turn rate to d1.
  2. Bird's-eye-view trajectory (x, y, theta_c) for the uncompensated vs.
     compensated (k=2) cases.
  3. Curvature-feasibility sweep: minimum path radius for which a
     pull-region equilibrium exists (closed-form counterpart to the
     F1TENTH look-ahead-vs-curvature sweep).

Run with:  python -m moby_sim.run_all
Outputs are written to ./outputs/ (created if it does not exist).
"""

import os
import numpy as np

from . import simulate
from . import plots

OUT_DIR = os.path.join(os.path.dirname(__file__), "outputs")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    d1 = 0.145   # m, real MOBY robot (Badia Torres et al., 2024)
    c = 1.2      # m/s, cruise speed used in the paper's own simulations
    gamma0 = np.pi + 0.3

    # --- Experiment 1: gamma(t) for k = 0, 1, 2 -----------------------
    results = {}
    for k, label in [(0.0, "k=0 (uncompensated)"),
                      (1.0, "k=1 (freezes drift)"),
                      (2.0, "k=2 (restores to pull)")]:
        results[label] = simulate.run_straight_line(d1=d1, c=c, gamma0=gamma0, k=k)

    plots.plot_gamma_traces(
        results,
        d1_values_for_sensitivity=[0.10, 0.145, 0.25],
        save_path=os.path.join(OUT_DIR, "moby_gamma_traces.png"),
    )

    final_gammas = {label: np.degrees(res["gamma"][-1]) % 360 for label, res in results.items()}
    print("Final gamma [deg] after simulated run:")
    for label, g in final_gammas.items():
        print(f"  {label}: {g:.2f}")

    # --- Experiment 2: bird's-eye trajectory (uncompensated vs k=2) ---
    traj_results = {
        "Uncompensated (k=0)": simulate.run_straight_line(d1=d1, c=c, gamma0=gamma0, k=0.0, T=2.0),
        "Compensated (k=2)": simulate.run_straight_line(d1=d1, c=c, gamma0=gamma0, k=2.0, T=2.0),
    }
    plots.plot_trajectory_birdseye(
        traj_results,
        save_path=os.path.join(OUT_DIR, "moby_trajectory_birdseye.png"),
    )

    # --- Experiment 3: curvature feasibility sweep ---------------------
    sweep = simulate.curvature_feasibility_sweep(d1=d1, c=c)
    plots.plot_curvature_feasibility(sweep, save_path=os.path.join(OUT_DIR, "moby_curvature_feasibility.png"))
    print(f"\nMinimum feasible turn radius for pull-region tracking: R = d1 = {sweep['R_min']} m")

    # --- Save raw numerical results as CSV for the report / appendix ---
    import csv
    with open(os.path.join(OUT_DIR, "gamma_traces.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_s"] + list(results.keys()))
        t = results["k=0 (uncompensated)"]["t"]
        rows = zip(t, *[np.degrees(res["gamma"]) for res in results.values()])
        writer.writerows(rows)

    with open(os.path.join(OUT_DIR, "curvature_feasibility.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["R_m", "gamma_goal_deg", "feasible"])
        for R, gg, feas in zip(sweep["R"], np.degrees(sweep["gamma_goal"]), sweep["feasible"]):
            writer.writerow([R, gg if feas else "", feas])

    print(f"\nAll figures and CSV results written to: {OUT_DIR}")


if __name__ == "__main__":
    main()
