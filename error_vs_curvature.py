#!/usr/bin/env python3

import argparse
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_run(path):
    t, err, idx = [], [], []
    label = ""
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['t_s']))
            err.append(float(row['lateral_error_m']))
            idx.append(int(row['nearest_idx']))
            label = row['run_label']
    return {'t': np.array(t), 'lateral_error_m': np.array(err),
            'nearest_idx': np.array(idx), 'run_label': label}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--raceline', required=True)
    parser.add_argument('--out', default='error_vs_curvature.png')
    parser.add_argument('--spike_threshold', type=float, default=0.5,
                         help='Error [m] above which a sample counts as a "spike" for the report table.')
    args = parser.parse_args()

    race = np.loadtxt(args.raceline, delimiter=',', skiprows=1)
    race_x, race_y, race_kappa = race[:, 0], race[:, 1], race[:, 3]
    race_radius = np.where(np.abs(race_kappa) > 1e-6, 1.0 / np.abs(race_kappa), np.inf)
    n_race = len(race_x)

    colors = ["#d62728", "#2ca02c", "#1f77b4"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    ax2 = axes[1]

    print(f"{'run':<28} {'spike_frac':<12} {'min_R_at_spike':<16} {'kappa_at_spike':<14}")

    spike_summaries = []
    for path, c in zip(args.runs, colors):
        d = load_run(path)
        label = d['run_label'] if d['run_label'] else path
        err = d['lateral_error_m']
        idx = d['nearest_idx'] % n_race
        kappa_at_sample = race_kappa[idx]
        radius_at_sample = race_radius[idx]

        ax.scatter(np.abs(kappa_at_sample), err, s=3, alpha=0.15, color=c, label=label)

        # bin by |kappa| to get a mean-error-vs-curvature curve (less noisy than raw scatter)
        bins = np.linspace(0, np.abs(race_kappa).max(), 25)
        bin_idx = np.digitize(np.abs(kappa_at_sample), bins)
        bin_centers, bin_means = [], []
        for b in range(1, len(bins)):
            mask = bin_idx == b
            if mask.sum() > 5:
                bin_centers.append((bins[b - 1] + bins[b]) / 2)
                bin_means.append(err[mask].mean())
        ax2.plot(bin_centers, bin_means, color=c, lw=2, marker='o', markersize=3, label=label)

        spike_mask = err > args.spike_threshold
        spike_frac = spike_mask.mean()
        if spike_mask.sum() > 0:
            min_R = radius_at_sample[spike_mask].min()
            kappa_at_worst = np.abs(kappa_at_sample[spike_mask]).max()
        else:
            min_R, kappa_at_worst = np.nan, np.nan
        print(f"{label:<28} {spike_frac:<12.3%} {min_R:<16.2f} {kappa_at_worst:<14.3f}")
        spike_summaries.append((label, spike_frac, min_R, kappa_at_worst))

    ax.set_xlabel('|local raceline curvature| [1/m]')
    ax.set_ylabel('lateral tracking error [m]')
    ax.set_title('Every sample: error vs. local curvature\nat the nearest raceline point')
    ax.legend(fontsize=8, markerscale=4)
    ax.grid(alpha=0.3)

    ax2.set_xlabel('|local raceline curvature| [1/m]  (binned)')
    ax2.set_ylabel('mean lateral tracking error [m]')
    ax2.set_title('Binned mean error vs. curvature\n(cf. MOBY\u2019s R = d1 feasibility limit)')
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(args.out, dpi=200, facecolor='white')
    print(f"\nSaved: {args.out}")

    print("\nSpike table (error > {:.2f} m):".format(args.spike_threshold))
    print(f"{'run':<28} {'% samples spiking':<20} {'min radius at spike [m]':<26} {'max |kappa| at spike':<20}")
    for label, frac, minR, kap in spike_summaries:
        print(f"{label:<28} {frac:<20.2%} {minR:<26.2f} {kap:<20.3f}")


if __name__ == '__main__':
    main()
