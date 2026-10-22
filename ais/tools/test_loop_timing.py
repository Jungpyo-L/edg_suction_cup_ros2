#!/usr/bin/env python3
"""Real-time check of the runner's 50 Hz loop (system Python, no robot, no simulator; takes ~11 s).

  python3 ais/tools/test_loop_timing.py
A perfect fake UR and a constant pressure; the yaw heuristic turns at full rate and never seals, so a correct loop
runs 500 ticks in 10 s and turns the cup by 100 deg (10 deg/s), as in the simulator.
"""
import importlib.util
import sys
from argparse import Namespace
from pathlib import Path

import numpy as np

AIS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(AIS))
from ais_haptic import robot_frame as rf  # noqa: E402

spec = importlib.util.spec_from_file_location('runner', AIS / 'nodes' / 'ais_trial_runner.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
runner.HOLD_AFTER_SEAL_S = runner.RELEASE_WAIT_S = 0.0


class UR:
    def __init__(self):
        self.pose, self.q6 = rf.tool_pose([0.6, 0.0, 0.1], 0.0), 0.0
        self.rtde_c = self.rtde_r = self

    def isPoseWithinSafetyLimits(self, p): return True
    def isJointsWithinSafetyLimits(self, q): return True
    def getActualTCPPose(self): return list(self.pose)
    def getActualQ(self): return [0.0] * 5 + [self.q6]
    def getInverseKinematics(self, p, qnear): self.ik = list(p); return list(qnear)
    def moveJ(self, q, *a): self.pose = self.ik
    def moveL(self, p, *a): self.pose = list(p)
    def servoL(self, p, *a): self.pose = list(p)
    def servoStop(self): pass


class IO:
    def pressure(self): return -np.full(3, 50.0)
    def __getattr__(self, name): return lambda *a: (np.zeros(0), np.zeros(0), np.zeros((0, 3))) if name == 'stop_raw_recording' else None


class Cal:
    theta = 0.0
    def trial_pose(self, row): return np.array([0.6, 0.0, 0.02]), 0.0
    def object_pose(self, obj): return np.array([0.6, 0.0, 0.02]), 0.0


cup = rf.CupConfig('t', dict(chambers=3, tcp_offset=[0] * 6, chamber_map=[0, 1, 2], body_offset_deg=0., calibrated=True))
row = dict(trial='0', controller='yaw', policy='', object='x', object_no='1', case_id='x:0',
           start_x_object='0', start_y_object='0', start_yaw_deg='0')
args = Namespace(press_mm=5.0, settle_s=0.0, seal_frac=0.8, pmax_kpa=64.4, lookahead=0.1, gain=300., q6_home_deg=0.)
s, arrays = runner.run_trial(args, cup, runner.Plan(row, Cal(), cup), UR(), IO(), runner.make_controller('yaw', 3))
turn = arrays['theta_cmd'][-1] - arrays['theta_cmd'][0] + 0.2       # first logged theta is after one step
ok = s['ticks'] == 500 and abs(s['time_s'] - 10.0) < 0.15 and abs(s['loop_hz'] - 50.0) < 0.5 and abs(turn - 100.0) < 1e-6
print(f"{'PASS' if ok else 'FAIL'} {s['ticks']} ticks in {s['time_s']:.3f} s, loop {s['loop_hz']:.2f} Hz, "
      f"{s['loop_overruns']} overruns, cup turned {turn:.1f} deg (expected 500, 10 s, 50 Hz, 100 deg)")
sys.exit(0 if ok else 1)
