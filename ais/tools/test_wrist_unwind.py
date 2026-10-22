#!/usr/bin/env python3
"""Wrist 3 over a whole cup block, with and without the unwinding of ais_haptic.motion (simulator stand-in for the UR).

  cd ~/projects/2D_Haptic_Search && ~/miniconda3/envs/suctionRL/bin/python ~/ros2_ws/src/suction_cup/ais/tools/test_wrist_unwind.py
One fake UR runs every trial of a block in schedule order, so wrist rotations accumulate as on the robot.
'without' replaces the unwinding moveJ by a moveL (shortest rotation from the current wrist), the old behaviour.
Pass: with unwinding, wrist 3 stays inside the UR limit (+-360 deg) with margin, and every trial starts within 180 deg of home.
"""
import csv
import json
import sys
from argparse import Namespace
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_runner_loop as T  # noqa: E402
from ais_haptic import robot_frame as rf  # noqa: E402

runner = T.runner


def block(spec):
    ch, unwind = spec
    from shapely.geometry import shape
    from tools.evaluate_v2_final import FinalEnvV2, final_inputs
    from tools.fixture_calibration import Calibration, solve
    from tools.train_continuous_rl import case_noise_seed
    if not unwind:
        runner.move_unwound = lambda rtde, pose, home=0.0, **kw: rtde.rtde_c.moveL(pose)
    lay = json.loads(Path('robot/fixture_final_subset_K10/fixture_layout.json').read_text())
    pts = {n: tuple(np.r_[np.array(p) + [400., -100.], 20.0 - lay['calibration']['bore_depth']] / 1000)
           for n, p in lay['calibration']['bores'].items()}
    cal = Calibration(solve(pts), lay)
    cup = rf.CupConfig('fake', dict(chambers=ch, tcp_offset=[0] * 6, chamber_map=list(range(ch)), body_offset_deg=30., calibrated=True))
    args = Namespace(press_mm=5.0, settle_s=0.0, seal_frac=0.8, pmax_kpa=64.4, lookahead=0.1, gain=300., q6_home_deg=0.)
    rows = [r for r in csv.DictReader(open('robot/fixture_final_subset_K10/trial_schedule.csv')) if int(r['chambers']) == ch]
    cases = {c['id']: c for c in final_inputs()[1]}
    ur = T.FakeUR(None, cal, cup)
    ur.pose = rf.tool_pose([0.6, 0.0, 0.1], 0.0)
    q6_all, starts, reasons = [], [], {}
    for row in rows:
        c = cases[row['case_id']]
        env = FinalEnvV2(chambers=ch, variant='mlp_h5s5_log', pressure_model='v2', shaping='log')
        env.objects = dict(env.objects)
        poly = json.loads((Path('data/robot_as_built_K10/objects') / f"{c['object_id']}.json").read_text())['polygon']
        env.objects[c['object']] = (shape(poly), env.objects[c['object']][1])
        env.reset(options=dict(case=c, noise_seed=case_noise_seed(c['id']), model_params={}))
        ur.env, ur.obj = env, row['object']
        ctrl, _ = runner.controller_for(row, cup, T.PACKAGE / 'policies')
        summary, _ = runner.run_trial(args, cup, runner.Plan(row, cal, cup), ur, T.FakeIO(env, cup), ctrl)
        w = summary['wrist3_deg']
        q6_all += [w['min'], w['max'], w['before_unwind']]
        starts.append(w['start'])
        reasons[summary['reason']] = reasons.get(summary['reason'], 0) + 1
    return ch, unwind, min(q6_all), max(q6_all), min(starts), max(starts), reasons


def main():
    ok = True
    with ProcessPoolExecutor(max_workers=4) as pool:
        for ch, unwind, lo, hi, s_lo, s_hi, reasons in pool.map(block, [(3, False), (3, True), (6, False), (6, True)]):
            line = (f"{ch}ch {'with' if unwind else 'without':7s} unwinding: wrist 3 over the block {lo:8.1f} .. {hi:8.1f} deg, "
                    f"trial starts {s_lo:7.1f} .. {s_hi:7.1f} deg, {reasons}")
            if unwind:
                good = max(abs(lo), abs(hi)) < 360 - 10 and max(abs(s_lo), abs(s_hi)) <= 180 + 1e-6 and 'joint_limit' not in reasons
                ok &= good
                print(('PASS ' if good else 'FAIL ') + line)
            else:
                print('     ' + line)
    print('ALL PASS' if ok else 'SOME CHECKS FAILED')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
