"""Top-view plot of a finished tag mission: driven path, tag map, visiting order.

Saved as PNG (plus the path as CSV) by tag_mission when the mission ends, e.g.
  ~/golf_ws/mission_logs/mission_20261029_153012.png
The path is the robot's own estimate (speed command / odometry + IMU heading, corrected
on each tag it reaches), the same pose the mission navigated with.
"""

import csv
import math
import os

import matplotlib
matplotlib.use('Agg')  # no window needed: render straight to a file
import matplotlib.pyplot as plt  # noqa: E402


def save_csv(path, trail):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'x_m', 'y_m', 'yaw_deg', 'state'])
        for t, x, y, yaw, state in trail:
            w.writerow([f'{t:.2f}', f'{x:.3f}', f'{y:.3f}', f'{math.degrees(yaw):.1f}', state])


def save_plot(path, trail, tags, order, visited, missed=(), unconfirmed=(), arrivals=(),
              true_trail=None, true_tags=None, standoff=0.0, title=''):
    """trail: [(t, x, y, yaw, state)]; tags: {id: (x, y)} (map); order: planned tour ids;
    visited/missed/unconfirmed: id lists; arrivals: [(id, x, y)] robot pose on arrival."""
    fig, ax = plt.subplots(figsize=(9, 9), dpi=110)
    colors = {'SCAN': '#9e9e9e', 'ALIGN': '#ff9800', 'NAVIGATE': '#1e88e5',
              'ARRIVED': '#43a047', 'SEARCH': '#8e24aa'}

    if true_trail:  # simulation only: where the robot really went
        ax.plot([p[0] for p in true_trail], [p[1] for p in true_trail], '-', color='#e53935',
                lw=1.0, alpha=0.6, label='true path (simulation)')
    if true_tags:
        ax.scatter([p[0] for p in true_tags.values()], [p[1] for p in true_tags.values()],
                   marker='x', s=60, color='#e53935', label='true tag (simulation)', zorder=4)

    # driven path, coloured by mission state
    for i in range(1, len(trail)):
        a, b = trail[i - 1], trail[i]
        if math.hypot(b[1] - a[1], b[2] - a[2]) > 0.3:   # relocalization jump: dashed
            ax.plot([a[1], b[1]], [a[2], b[2]], ':', color='#757575', lw=1)
            continue
        ax.plot([a[1], b[1]], [a[2], b[2]], '-', color=colors.get(b[4], '#1e88e5'), lw=2.2)
    for state, c in colors.items():
        if any(p[4] == state for p in trail):
            ax.plot([], [], '-', color=c, lw=2.2, label=f'path: {state}')

    # planned tour (straight lines between tags, from the scan position)
    if order and trail:
        start = next(((p[1], p[2]) for p in trail if p[4] != 'SCAN'), (trail[-1][1], trail[-1][2]))
        pts = [start] + [tags[i] for i in order if i in tags]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], '--', color='#bdbdbd', lw=1,
                label='planned order (after the scan)', zorder=1)

    # tags from the map
    for tid, (x, y) in tags.items():
        if tid in unconfirmed:
            c, m = '#fb8c00', 's'
        elif tid in visited:
            c, m = '#2e7d32', 's'
        elif tid in missed:
            c, m = '#c62828', 's'
        else:
            c, m = '#616161', 's'
        ax.scatter([x], [y], marker=m, s=140, color=c, edgecolor='black', zorder=5)
        if standoff > 0:
            ax.add_patch(plt.Circle((x, y), standoff, fill=False, ls=':', color=c, alpha=0.5))
        n = f' #{visited.index(tid) + 1}' if tid in visited else ''
        ax.annotate(f'tag {tid}{n}', (x, y), xytext=(8, 8), textcoords='offset points',
                    fontsize=10, weight='bold', color=c)
    ax.scatter([], [], marker='s', color='#2e7d32', label='tag visited (#order)')
    if missed:
        ax.scatter([], [], marker='s', color='#c62828', label='tag missed')
    if unconfirmed:
        ax.scatter([], [], marker='s', color='#fb8c00', label='reached, not seen on arrival')

    for tid, x, y in arrivals:
        ax.scatter([x], [y], marker='o', s=30, color='#2e7d32', zorder=6)

    # start and end pose with heading arrows
    if trail:
        for p, label, c in ((trail[0], 'start', '#000000'), (trail[-1], 'end', '#6a1b9a')):
            ax.scatter([p[1]], [p[2]], marker='o', s=90, color=c, zorder=7,
                       label='start (360 deg scan here)' if label == 'start' else label)
            ax.arrow(p[1], p[2], 0.25 * math.cos(p[3]), 0.25 * math.sin(p[3]), width=0.02,
                     color=c, zorder=7, length_includes_head=True)

    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x (m)  - forward at start')
    ax.set_ylabel('y (m)  - left at start')
    ax.set_title(title, fontsize=10)
    ax.legend(loc='best', fontsize=8, framealpha=0.85)
    fig.tight_layout()
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path
