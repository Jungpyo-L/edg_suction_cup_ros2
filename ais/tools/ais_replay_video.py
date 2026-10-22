#!/usr/bin/env python3
"""Render trials saved by ais_trial_runner.py (robot, or the simulator stand-in of test_runner_loop.py) as mp4.

  python3 ais/tools/ais_replay_video.py ~/EDG_Experiment/ais_robot_demo --trials 0-5 [--ref DIR] [--out DIR]
Left: top view in the object frame (as-built polygon, 30 mm boundary, cup chambers coloured by vacuum, path).
Right: chamber vacuum (simulator order) with the success threshold, and the applied body-frame action.
--ref DIR overlays the path of the same trial number from another results folder (e.g. simulation vs robot).
Needs matplotlib and ffmpeg only (system Python on the robot PC works).
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
from matplotlib.patches import Circle, PathPatch, Wedge  # noqa: E402
from matplotlib.path import Path as MplPath  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

CONTROL_HZ = 50.0
P_MAX_PA = 64400.0
BOUNDARY_MM = 30.0
NAMES = {'ppo': 'PPO', 'greedy': 'greedy', 'yaw_momentum': 'yaw-momentum', 'yaw': 'yaw', 'sac': 'SAC', 'momentum': 'momentum'}


def parse_trials(spec):
    out = set()
    for part in spec.split(','):
        lo, _, hi = part.partition('-')
        out.update(range(int(lo), int(hi or lo) + 1))
    return out


def polygon_patch(geojson, **kw):
    verts, codes = [], []
    for ring in geojson['coordinates']:
        ring = np.asarray(ring, float)
        verts += ring.tolist() + [ring[0].tolist()]
        codes += [MplPath.MOVETO] + [MplPath.LINETO] * (len(ring) - 1) + [MplPath.CLOSEPOLY]
    return PathPatch(MplPath(verts, codes), **kw)


def object_frame(poses, frame):
    """UR poses (N x 6) -> object-frame xy (mm) and simulator body yaw (deg)."""
    poses = np.asarray(poses, float)
    th = np.radians(frame['calibration_rotation_deg'])
    rot = np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])     # R(-theta)
    xy = (poses[:, :2] - np.asarray(frame['object_origin_robot_m'])) @ rot.T * 1000.
    x_axis = Rotation.from_rotvec(poses[:, 3:6]).as_matrix()[:, :, 0]
    psi = np.degrees(np.arctan2(x_axis[:, 1], x_axis[:, 0]))
    return xy, psi + frame['body_offset_deg'] - frame['calibration_rotation_deg']


def load(results, trial):
    j = results / ('trial_%04d.json' % trial)
    if not j.exists():
        return None
    return json.loads(j.read_text()), dict(np.load(results / ('trial_%04d.npz' % trial)))


def render(summary, arrays, polygon, out, ref=None, lip_mm=5.0, fps=25, label=''):
    frame = summary['frame']
    n = frame['chambers']
    xy, yaw = object_frame(arrays['actual'], frame)
    vac = np.asarray(arrays['vacuum_sim_pa'], float)
    act = np.asarray(arrays['action'], float)
    t = np.arange(len(xy)) / CONTROL_HZ
    seal_pa = summary['seal_frac'] * summary['pmax_kpa'] * 1000.
    ref_xy = object_frame(ref[1]['actual'], ref[0]['frame'])[0] if ref else None

    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1], hspace=0.35, wspace=0.22, left=0.05, right=0.97, top=0.88, bottom=0.08)
    ax = fig.add_subplot(gs[:, 0])
    ax_p = fig.add_subplot(gs[0, 1])
    ax_a = fig.add_subplot(gs[1, 1], sharex=ax_p)
    row = summary['row']
    result = {'sealed': 'SEALED', 'timeout': 'timeout (10 s)', 'boundary': 'left the 30 mm boundary',
              'tracking_fault': 'tracking fault'}.get(summary['reason'], summary['reason'])
    fig.suptitle('%strial %s   %dch %s   object %s (%s)   start %s   ->  %s in %.2f s' % (
        label, row['trial'], n, NAMES.get(row['controller'], row['controller']), row['object_no'], row['object'],
        row['case_id'].split(':')[-1], result, summary['ticks'] / CONTROL_HZ), fontsize=12)

    ax.add_patch(polygon_patch(polygon, facecolor='#d9d9d9', edgecolor='#555555', lw=1.0))
    ax.add_patch(Circle((0, 0), BOUNDARY_MM, fill=False, ls=':', color='#999999'))
    ax.plot(0, 0, '+', color='#999999')
    if ref_xy is not None:
        ax.plot(ref_xy[:, 0], ref_xy[:, 1], '--', color='#e07b00', lw=1.2, label='reference')
    ax.plot(xy[0, 0], xy[0, 1], 'o', color='k', ms=4)
    trail, = ax.plot([], [], '-', color='#1f5fbf', lw=1.6, label='cup centre')
    cmap = plt.get_cmap('viridis')
    wedges = [Wedge((0, 0), lip_mm, 0, 1, facecolor=cmap(0.), edgecolor='k', lw=0.6) for _ in range(n)]
    for w in wedges:
        ax.add_patch(w)
    tick_mark, = ax.plot([], [], '-', color='r', lw=1.5, label='cup body x-axis')
    lim = max(BOUNDARY_MM, np.abs(xy).max() + lip_mm + 2)
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_aspect('equal')
    ax.set_xlabel('object x (mm)'); ax.set_ylabel('object y (mm)')
    ax.legend(loc='lower right', fontsize=8)
    clock = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', family='monospace')

    colors = plt.get_cmap('tab10')
    for i in range(n):
        ax_p.plot(t, vac[:, i] / 1000., color=colors(i), lw=1.2, label='ch %d' % i)
    ax_p.axhline(seal_pa / 1000., color='k', ls='--', lw=0.8)
    ax_p.text(1.0, seal_pa / 1000., '%.0f %% P_max ' % (100 * summary['seal_frac']), va='top', ha='right', fontsize=8,
              transform=ax_p.get_yaxis_transform())
    # log axis like the policy observation (log1p(p / 10 Pa)): the small pre-seal differences are what the controllers use
    ax_p.set_yscale('symlog', linthresh=0.01, linscale=0.3)
    ax_p.set_ylim(0, P_MAX_PA / 1000. * 1.5)
    ax_p.set_yticks([0, 0.01, 0.1, 1, 10, 64.4]); ax_p.set_yticklabels(['0', '0.01', '0.1', '1', '10', '64.4'])
    ax_p.set_ylabel('chamber vacuum (kPa, log)')
    ax_p.legend(loc='lower left', bbox_to_anchor=(0, 1.0), fontsize=7, ncol=n, frameon=False)
    for k, (lab, c) in enumerate((('v_x body', '#1f77b4'), ('v_y body', '#ff7f0e'), ('yaw rate', '#2ca02c'))):
        ax_a.plot(t, act[:, k], color=c, lw=1.2, label=lab)
    ax_a.set_ylim(-1.1, 1.1); ax_a.set_ylabel('action (x 10 mm/s, x 10 deg/s)'); ax_a.set_xlabel('time (s)')
    ax_a.legend(loc='lower left', bbox_to_anchor=(0, 1.0), fontsize=7, ncol=3, frameon=False)
    ax_p.set_xlim(0, max(t[-1] if len(t) else 1, 1.0))
    cursors = [a.axvline(0, color='r', lw=0.8) for a in (ax_p, ax_a)]

    def draw(k):
        trail.set_data(xy[:k + 1, 0], xy[:k + 1, 1])
        for i, w in enumerate(wedges):
            centre = yaw[k] + 180. / n + 360. * i / n
            w.set_center(xy[k]); w.set_theta1(centre - 180. / n); w.set_theta2(centre + 180. / n)
            w.set_facecolor(cmap(np.clip(np.log1p(vac[k, i] / 10.) / np.log1p(P_MAX_PA / 10.), 0, 1)))
        d = np.radians(yaw[k])
        tick_mark.set_data([xy[k, 0], xy[k, 0] + 1.6 * lip_mm * np.cos(d)], [xy[k, 1], xy[k, 1] + 1.6 * lip_mm * np.sin(d)])
        clock.set_text('t = %5.2f s\nyaw %6.1f deg' % (t[k], (yaw[k] + 180.) % 360. - 180.))
        for c in cursors:
            c.set_xdata([t[k], t[k]])

    step = max(1, int(round(CONTROL_HZ / fps)))
    frames = list(range(0, len(xy), step)) + [len(xy) - 1] * fps   # hold the last frame for 1 s
    writer = FFMpegWriter(fps=fps, bitrate=2500)
    with writer.saving(fig, str(out), dpi=100):
        for k in frames:
            draw(k)
            writer.grab_frame()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('results', help='folder with trial_XXXX.json/.npz')
    ap.add_argument('--trials', default=None, help='e.g. 0-5,12 (default: all in the folder)')
    ap.add_argument('--ref', default=None, help='folder whose same-numbered trial path is overlaid')
    ap.add_argument('--out', default=None, help='default <results>/videos')
    ap.add_argument('--repo', default='~/projects/2D_Haptic_Search')
    ap.add_argument('--label', default='', help='prefix for the title, e.g. "simulation " or "robot "')
    ap.add_argument('--fps', type=int, default=25)
    a = ap.parse_args()
    results = Path(a.results).expanduser()
    out = Path(a.out).expanduser() if a.out else results / 'videos'
    out.mkdir(parents=True, exist_ok=True)
    trials = sorted(parse_trials(a.trials)) if a.trials else sorted(int(p.stem[6:]) for p in results.glob('trial_*.json'))
    poly_dir = Path(a.repo).expanduser() / 'data' / 'robot_as_built_K10' / 'objects'
    for trial in trials:
        got = load(results, trial)
        if got is None:
            print('trial %d: not in %s' % (trial, results)); continue
        summary, arrays = got
        if len(arrays['actual']) < 2:
            print('trial %d: too short to render' % trial); continue
        polygon = json.loads((poly_dir / ('%s.json' % summary['row']['object'])).read_text())['polygon']
        ref = load(Path(a.ref).expanduser(), trial) if a.ref else None
        path = out / ('trial_%04d_%dch_%s_%s.mp4' % (trial, summary['frame']['chambers'], summary['row']['controller'], summary['reason']))
        render(summary, arrays, polygon, path, ref, fps=a.fps, label=a.label)
        print('wrote %s' % path)


if __name__ == '__main__':
    sys.exit(main())
