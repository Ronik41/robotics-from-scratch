"""Render recorded numerical evidence; never synthesizes robot measurements."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('acceptance', type=Path)
    args = parser.parse_args()
    fig, axes = plt.subplots(3, 2, figsize=(11, 11), constrained_layout=True)
    for col, mode in enumerate(('mapping', 'localization')):
        samples = json.loads((args.acceptance/mode/'trajectory.json').read_text())
        times = [s['time'] for s in samples]
        ax = axes[0, col]
        for key, label, color in [('truth_in_map', 'Gazebo oracle (test only)', '#202020'),
                                   ('estimate', 'SLAM' if mode == 'mapping' else 'AMCL', '#007f82'),
                                   ('odom', 'Wheel odometry', '#d77b10')]:
            ax.plot([s[key][0] for s in samples], [s[key][1] for s in samples], label=label, color=color)
        ax.set(xlabel='map x [m]', ylabel='map y [m]', title=mode.capitalize())
        ax.axis('equal'); ax.legend(fontsize=8); ax.grid(alpha=.2)
        for row, key, units, budget in [(1, 'position_error_m', 'Position error [m]', .15),
                                       (2, 'yaw_error_rad', 'Absolute yaw error [rad]', .12)]:
            ax = axes[row, col]
            ax.plot(times, [s['odom_'+key] for s in samples], color='#d77b10', label='Wheel odometry')
            ax.plot(times, [s[key] for s in samples], color='#007f82', label='SLAM' if mode == 'mapping' else 'AMCL')
            ax.axhline(budget, color='#a03030', linestyle='--', linewidth=1, label='Final-window budget')
            ax.set(xlabel='Simulation time [s]', ylabel=units); ax.grid(alpha=.2)
            ax.legend(fontsize=8)
    fig.suptitle('M6 — measured map-frame trajectories and error\nFixed initial alignment; no fitted registration', fontsize=14)
    fig.savefig(args.acceptance/'trajectory-errors.png', dpi=150)


if __name__ == '__main__':
    main()
