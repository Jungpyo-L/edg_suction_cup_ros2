#!/usr/bin/env python3
"""Map simulator body-frame commands and chamber order onto the UR base frame (numpy only, no ROS).

Frames (all seen from above, angles counter-clockwise about the UR `base` +z, which points up):
- body yaw theta: the simulator cup yaw. Simulator chamber i points at theta + 180/n + 360 i/n.
  fixture_calibration.Calibration.trial_pose() returns the start position and theta in this frame.
- tool yaw psi: direction of the tool x-axis projected on the table. The tool points down, so the UR pose is
  Rz(psi) Rx(pi) (same as Rotation.from_euler("ZXY", [psi, pi, 0]) in the lateral scripts).
- the cup is rigid on the tool: theta = psi + body_offset_deg. Rotating the tool about base +z by w rotates the cup
  by w, so the simulator yaw rate needs no sign change when it is applied about base +z.
- chamber_map[i] = sensor channel that is simulator chamber i. A cup whose channels are numbered clockwise seen
  from above, or wired in any order, is just another permutation; body_offset_deg and chamber_map come from
  fit_chamber_directions().
"""
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .policy import CONTROL_HZ, SPEED_MM_S, YAW_RATE_DEG_S


def wrap_deg(a):
    return (np.asarray(a, dtype=float) + 180.) % 360. - 180.


def tool_pose(xyz_m, psi_deg):
    """UR pose [x, y, z, rx, ry, rz] with the tool pointing down and its x-axis at psi (deg)."""
    rotvec = Rotation.from_euler("ZXY", [np.radians(psi_deg), np.pi, 0.]).as_rotvec()
    return [float(v) for v in np.r_[np.asarray(xyz_m, dtype=float), rotvec]]


def tool_yaw_deg(ur_pose):
    """psi of a UR pose [x, y, z, rx, ry, rz]: angle of the tool x-axis projected on the table."""
    x_axis = Rotation.from_rotvec(ur_pose[3:6]).as_matrix()[:, 0]
    return float(np.degrees(np.arctan2(x_axis[1], x_axis[0])))


def tool_tilt_deg(ur_pose):
    """Angle between the tool z-axis and straight down."""
    z_axis = Rotation.from_rotvec(ur_pose[3:6]).as_matrix()[:, 2]
    return float(np.degrees(np.arccos(np.clip(-z_axis[2], -1., 1.))))


class CupConfig:
    """One cup from ais/config/cups.json."""

    def __init__(self, name, entry):
        self.name = name
        self.chambers = int(entry['chambers'])
        self.tcp_offset = [float(v) for v in entry['tcp_offset']]
        self.lip_radius_mm = float(entry.get('lip_radius_mm', 5.0))
        self.chamber_map = entry.get('chamber_map')
        self.body_offset_deg = entry.get('body_offset_deg')
        self.calibrated = bool(entry.get('calibrated', False))
        if self.chamber_map is not None and sorted(self.chamber_map) != list(range(self.chambers)):
            raise ValueError(f'{name}: chamber_map must be a permutation of 0..{self.chambers - 1}')

    def require_calibrated(self):
        if not self.calibrated or self.chamber_map is None or self.body_offset_deg is None:
            raise RuntimeError(f'Cup {self.name}: chamber_map / body_offset_deg are not measured yet. '
                               'Run ais_chamber_check.py first (or pass --dry-run).')

    def sim_vacuum(self, sensor_pa):
        """Offset-corrected sensor reading (Pa, vacuum negative, channel order) -> simulator order, vacuum positive."""
        p = np.asarray(sensor_pa, dtype=float)
        if p.shape != (self.chambers,):
            raise ValueError(f'Expected {self.chambers} channels, got {p.shape}')
        return -p[np.asarray(self.chamber_map)]

    def psi_for_body_yaw(self, theta_deg):
        return float(wrap_deg(theta_deg - self.body_offset_deg))

    def body_yaw_for_psi(self, psi_deg):
        return float(wrap_deg(psi_deg + self.body_offset_deg))


def load_cups(path):
    data = json.loads(Path(path).read_text())
    return {k: CupConfig(k, v) for k, v in data['cups'].items()}


class BodyCommandIntegrator:
    """Integrates applied body-frame actions into servoL targets, exactly like IdealContinuousEnv.step:
    position += R(theta) a[:2] * SPEED * dt with theta from the start of the tick, then theta += a[2] * YAW_RATE * dt.

    Targets are integrated from the commanded state, not the measured pose, so robot tracking error does not feed back
    into the commanded path. xy in m (UR base), theta in deg (body yaw). z is held at the contact height.
    """

    def __init__(self, xy_m, z_m, theta_deg, cup, dt=1. / CONTROL_HZ):
        self.xy = np.asarray(xy_m, dtype=float).copy()
        self.z = float(z_m)
        self.theta = float(theta_deg)
        self.cup, self.dt = cup, dt
        self.path_mm = 0.
        self.yaw_path_deg = 0.

    def step(self, applied):
        a = np.asarray(applied, dtype=float)
        c, s = np.cos(np.radians(self.theta)), np.sin(np.radians(self.theta))
        delta_mm = np.array([[c, -s], [s, c]]) @ a[:2] * SPEED_MM_S * self.dt
        self.xy = self.xy + delta_mm / 1000.
        self.theta += float(a[2]) * YAW_RATE_DEG_S * self.dt
        self.path_mm += float(np.linalg.norm(delta_mm))
        self.yaw_path_deg += abs(float(a[2]) * YAW_RATE_DEG_S * self.dt)
        return self.target()

    def target(self):
        return tool_pose(np.r_[self.xy, self.z], self.cup.psi_for_body_yaw(self.theta))


def chamber_angle_sim_deg(n, i):
    return 180. / n + 360. * i / n


def fit_chamber_directions(open_dir_deg):
    """Fit chamber_map and body_offset_deg from measured chamber directions.

    open_dir_deg[j]: angle, relative to the tool x-axis (seen from above, CCW), at which sensor channel j points,
    e.g. from a rotation scan over a straight edge: (outward edge normal) - psi at the channel's weakest vacuum.
    Tries every assignment (any wiring order). Returns (chamber_map, body_offset_deg, rms_residual_deg).
    """
    alpha = np.asarray(open_dir_deg, dtype=float)
    n = len(alpha)
    best = None
    for chamber_map in itertools.permutations(range(n)):
        chamber_map = list(chamber_map)
        # channel chamber_map[i] should point at body_offset + 180/n + 360 i/n
        diffs = np.array([alpha[chamber_map[i]] - chamber_angle_sim_deg(n, i) for i in range(n)])
        offset = float(np.degrees(np.angle(np.mean(np.exp(1j * np.radians(diffs))))))
        rms = float(np.sqrt(np.mean(wrap_deg(diffs - offset) ** 2)))
        if best is None or rms < best[2]:
            best = (chamber_map, float(wrap_deg(offset)), rms)
    return best
