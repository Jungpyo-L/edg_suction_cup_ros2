#!/usr/bin/env python3
"""Check haptic_chamber_check.analyze_scan on scans synthesized with the simulator pressure model (pressure_v2).

  cd ~/projects/2D_Haptic_Search && ~/miniconda3/envs/suctionRL/bin/python ~/ros2_ws/src/suction_cup/tools/test_chamber_check.py
A cup of radius 5 mm straddles the rim of a 23.4 mm bore; channels are wired in a random order with a random
cup-to-tool yaw offset and a random robot-frame direction; 6 Pa noise. The fit must recover every channel direction.
"""
import importlib.util
import sys
import types
from pathlib import Path

import numpy as np
from shapely.affinity import affine_transform
from shapely.geometry import Point, box

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
sys.path.insert(0, str(Path.cwd()))
from haptic_search.physics import pressure_v2  # noqa: E402
from helperFunction import haptic_robot_frame as rf  # noqa: E402

for name in ('rclpy', 'helperFunction.haptic_ros', 'helperFunction.rtde_helper'):   # the analysis needs no ROS
    sys.modules[name] = types.ModuleType(name)
sys.modules['helperFunction.haptic_ros'].HapticRosIO = object
sys.modules['helperFunction.rtde_helper'].rtdeHelp = object
spec = importlib.util.spec_from_file_location('chamber_check', PACKAGE / 'src' / 'haptic_chamber_check.py')
cc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cc)

rng = np.random.default_rng(3)
ok, worst = True, 0.
for n in (3, 6):
    templates = pressure_v2.chamber_templates(n)
    areas = np.array([t.area for t in templates])
    for trial in range(10):
        perm = list(rng.permutation(n)); body_offset = rng.uniform(-180, 180); open_dir = rng.uniform(-180, 180)
        overlap_mm = rng.uniform(0.5, 1.5)       # how far the 5 mm lip reaches past the bore rim (script default 1)
        bore = Point(0, 0).buffer(11.7, 256)
        surface = box(-40, -40, 40, 40).difference(bore)
        # cup centre at distance r from the bore centre, bore lying along open_dir as seen from the cup
        r = 11.7 + 5 - overlap_mm
        centre = -r * np.array([np.cos(np.radians(open_dir)), np.sin(np.radians(open_dir))])
        psis = np.arange(-180., 180., 15.)
        vac = []
        for psi in psis:
            theta = np.radians(psi + body_offset)
            c, s = np.cos(theta), np.sin(theta)
            covered = [affine_transform(t, [c, -s, s, c, *centre]).intersection(surface).area for t in templates]
            p_sim = pressure_v2.chamber_pressures(np.clip(areas - covered, 0, None), n)
            p_chan = np.zeros(n)
            p_chan[perm] = p_sim                   # channel perm[i] carries simulator chamber i
            vac.append(p_chan + rng.normal(0, 6 / np.sqrt(7), n))
        report, cmap, off, rms, weak = cc.analyze_scan(psis, np.array(vac), open_dir, verbose=False)
        true_dir = {perm[i]: body_offset + rf.chamber_angle_sim_deg(n, i) for i in range(n)}
        fit_dir = {cmap[i]: off + rf.chamber_angle_sim_deg(n, i) for i in range(n)}
        e = max(abs(rf.wrap_deg(fit_dir[j] - true_dir[j])) for j in range(n))
        worst = max(worst, e)
        if e > 5 or weak:
            ok = False
            print(f'FAIL {n}ch: perm {perm} off {body_offset:.1f} -> {cmap} {off:.1f}, err {e:.1f}, weak {weak}')
print(f"{'PASS' if ok else 'FAIL'} chamber check recovers channel directions (worst {worst:.2f} deg)")
sys.exit(0 if ok else 1)
