#!/usr/bin/env python3
"""Unit checks for ais_haptic/robot_frame.py (system Python, no robot, no simulator).

  python3 ais/tools/test_robot_frame.py
The integrator is also checked against the simulator in ais/tools/test_policy_parity.py.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # ais/: ais_haptic
from ais_haptic import robot_frame as rf  # noqa: E402

rng = np.random.default_rng(0)
ok = True


def check(name, cond, detail=''):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'} {name} {detail}")


# tool pose: psi round trip, pointing straight down, same convention as the lateral scripts
psis = rng.uniform(-180, 180, 200)
err = max(abs(rf.wrap_deg(rf.tool_yaw_deg(rf.tool_pose([0.5, 0.1, 0.02], p)) - p)) for p in psis)
tilt = max(rf.tool_tilt_deg(rf.tool_pose([0, 0, 0], p)) for p in psis)
same = max(np.abs(Rotation.from_rotvec(rf.tool_pose([0, 0, 0], p)[3:]).as_matrix()
                  - Rotation.from_euler("ZXY", [np.radians(p), np.pi, 0]).as_matrix()).max() for p in psis)
check('tool yaw round trip', err < 1e-9, f'max {err:.1e} deg')
check('tool points down', tilt < 1e-5, f'max tilt {tilt:.1e} deg')
check('matches from_euler("ZXY", [psi, pi, 0])', same < 1e-12)
print(f'     note: the fixed rotvec [2.263, -2.179, 0] of the old scripts is psi = {rf.tool_yaw_deg([0, 0, 0, 2.263, -2.179, 0]):.1f} deg')

# rotating the tool about base +z by w changes psi (and so the body yaw) by +w
p0 = rf.tool_pose([0, 0, 0], 30.)
R1 = Rotation.from_euler('z', 7., degrees=True) * Rotation.from_rotvec(p0[3:])
check('base +z rotation adds to psi', abs(rf.tool_yaw_deg([0, 0, 0, *R1.as_rotvec()]) - 37.) < 1e-9)

# chamber fit: any wiring order, any offset, 3 deg measurement noise. The simulator cup is n-fold symmetric, so
# (map shifted by one chamber, offset - 360/n) is the same physical answer: compare the predicted channel directions.
worst = 0.
for n in (3, 6):
    for _ in range(50):
        perm = list(rng.permutation(n)); off = rng.uniform(-180, 180)
        alpha = np.zeros(n)
        for i in range(n):
            alpha[perm[i]] = off + rf.chamber_angle_sim_deg(n, i) + rng.normal(0, 3)
        cmap, est, rms = rf.fit_chamber_directions(rf.wrap_deg(alpha))
        true_dir = {perm[i]: off + rf.chamber_angle_sim_deg(n, i) for i in range(n)}
        fit_dir = {cmap[i]: est + rf.chamber_angle_sim_deg(n, i) for i in range(n)}
        e = max(abs(rf.wrap_deg(fit_dir[j] - true_dir[j])) for j in range(n))
        worst = max(worst, e)
        if e > 5:
            check(f'chamber fit {n}ch', False, f'perm {perm} -> {cmap}, offset {off:.1f} -> {est:.1f}')
check('chamber fit recovers every channel direction', ok, f'worst error {worst:.2f} deg')

# pressure reorder and sign
cup = rf.CupConfig('t', dict(chambers=3, tcp_offset=[0] * 6, chamber_map=[1, 2, 0], body_offset_deg=10., calibrated=True))
check('sim_vacuum order and sign', np.allclose(cup.sim_vacuum([-1., -2., -3.]), [2., 3., 1.]))
check('psi/body yaw inverse', abs(cup.body_yaw_for_psi(cup.psi_for_body_yaw(123.)) - 123.) < 1e-9)

# integrator: pure rotation keeps xy; a body +x step at theta 90 moves base +y
it = rf.BodyCommandIntegrator([0.5, 0.0], 0.02, 90., cup)
it.step([1., 0., 0.])
check('body +x at theta 90 -> base +y', np.allclose(it.xy, [0.5, 0.0002]), f'{it.xy}')
it = rf.BodyCommandIntegrator([0.5, 0.0], 0.02, 0., cup)
for _ in range(50):
    it.step([0., 0., 1.])
check('yaw 1 s at full rate = 10 deg', abs(it.theta - 10.) < 1e-9 and np.allclose(it.xy, [0.5, 0.]))

print('ALL PASS' if ok else 'SOME CHECKS FAILED')
sys.exit(0 if ok else 1)
