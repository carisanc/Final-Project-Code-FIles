"""
run_all.py
----------
Reproduces the F1TENTH independent pure pursuit results used in the
project report: a look-ahead-distance sweep on a curvature-varying path
built to match the curvature profile of MOBY's own test trajectory
(Badia Torres et al., 2024, Figure 10).

Run with:  python3 -m f1tenth_sim.run_all
Outputs are written to ./outputs/ (created if it does not exist).
"""

import os
import csv
import numpy as np

from . import path as path_mod
from . import simulate
from . import plots

OUT_DIR = os.path.join(os.path.dirname(__file__), "outputs")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    ref_path = path_mod.arc_path()
    lookaheads = [0.35, 0.8, 1.6]

    results = simulate.lookahead_sweep(ref_path, lookaheads)

    plots.plot_trajectory_and_error(
        ref_path, results, save_path=os.path.join(OUT_DIR, "f1tenth_pp_curvature_sim.png")
    )

    print("Look-ahead sweep results on the curvature-varying path:")
    for l_d, res in results.items():
        e = res["lateral_error"]
        print(f"  l_d={l_d} m: mean err={e.mean():.4f} m, max err={e.max():.4f} m, n_steps={len(res['x'])}")

    with open(os.path.join(OUT_DIR, "lookahead_sweep_summary.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["l_d_m", "mean_error_m", "max_error_m", "n_steps"])
        for l_d, res in results.items():
            e = res["lateral_error"]
            writer.writerow([l_d, e.mean(), e.max(), len(res["x"])])

    print(f"\nAll figures and CSV results written to: {OUT_DIR}")


if __name__ == "__main__":
    main()
