#!/usr/bin/env python3
import argparse
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_run(path):
    """
    Quote-aware CSV reader (run_label can contain commas, e.g.
    "lookahead_k=0.4, ss=0.6" -- np.genfromtxt/np.loadtxt do NOT respect
    CSV quoting and will misparse that field; Python's csv module does).
    Returns a dict of numpy arrays, one per numeric column, plus the
    run_label string.
    """
    t, x, y, yaw, steer, speed, err, idx = ([] for _ in range(8))
    label = ""
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['t_s']))
            x.append(float(row['x']))
            y.append(float(row['y']))
            yaw.append(float(row['yaw']))
            steer.append(float(row['steering_cmd_rad']))
            speed.append(float(row['speed_cmd_mps']))
            err.append(float(row['lateral_error_m']))
            idx.append(int(row['nearest_idx']))
            label = row['run_label']
    return {
        't': np.array(t), 'x': np.array(x), 'y': np.array(y), 'yaw': np.array(yaw),
        'steering_cmd_rad': np.array(steer), 'speed_cmd_mps': np.array(speed),
        'lateral_error_m': np.array(err), 'nearest_idx': np.array(idx),
        'run_label': label,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+', help='CSV files from pp_run_logger.py')
    parser.add_argument('--raceline', required=True, help='Reference raceline CSV (x,y,...)')
    parser.add_argument('--out', default='real_pp_lookahead_comparison.png')
    args = parser.parse_args()

    ref = np.loadtxt(args.raceline, delimiter=',', skiprows=1)
    ref_x, ref_y = ref[:, 0], ref[:, 1]

    colors = ["#d62728", "#2ca02c", "#1f77b4", "#9467bd", "#8c564b"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    ax.plot(ref_x, ref_y, "--", color="gray", lw=1.2, label="Raceline (reference)")

    ax2 = axes[1]
    summary_lines = []

    for path, c in zip(args.runs, colors):
        d = load_run(path)
        label = d['run_label'] if d['run_label'] else path
        ax.plot(d['x'], d['y'], color=c, lw=1.0, alpha=0.8, label=label)

        dist = np.cumsum(np.r_[0, np.hypot(np.diff(d['x']), np.diff(d['y']))])
        err = d['lateral_error_m']
        ax2.plot(dist, err, color=c, lw=0.8, alpha=0.8,
                 label=f"{label}  (mean={err.mean():.3f} m, max={err.max():.3f} m)")
        summary_lines.append(
            f"{label}: mean={err.mean():.4f} m, max={err.max():.4f} m, "
            f"p95={np.percentile(err, 95):.4f} m, n={len(err)}, duration={d['t'][-1]:.1f} s"
        )

    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Real ROS2/gym runs on São Paulo circuit\n(pure_pursuit_node, ground-truth raceline)')
    ax.legend(fontsize=7, loc='best')
    ax.grid(alpha=0.3)

    ax2.set_xlabel('distance traveled [m]')
    ax2.set_ylabel('lateral tracking error [m]')
    ax2.set_title('Real tracking error vs. distance\n(compare against Figure 7, synthetic sweep)')
    ax2.legend(fontsize=7)
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(args.out, dpi=200, facecolor='white')
    print(f"Saved: {args.out}\n")
    print("Summary:")
    for line in summary_lines:
        print(" ", line)


if __name__ == '__main__':
    main()
