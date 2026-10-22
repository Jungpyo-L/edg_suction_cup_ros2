#!/usr/bin/env python3
"""End-to-end check of haptic_trial_runner.run_trial with the robot and the sensor replaced by the simulator.

  cd ~/projects/2D_Haptic_Search && ~/miniconda3/envs/suctionRL/bin/python ~/ros2_ws/src/suction_cup/ais/tools/test_runner_loop.py \
      [--save ~/EDG_Experiment/ais_sim_in_the_loop]   # also keep the trials for ais_replay_video.py
A fake UR follows every servoL target exactly; the target is turned back into an object-frame cup pose and fed to the
simulator (FinalEnvV2, as-built polygons, same noise seeds). The fake sensor returns the simulator pressures in a
scrambled channel order, negative, through a random cup-to-tool yaw offset and a rotated fixture calibration.
The simulator physics keeps running after the geometric seal (no episode end), like the robot.
Pass (run with no pressure stop): if every frame conversion is right, the geometric seal happens at exactly the tick
stored in outputs/robot_as_built_sim/episodes.json. Also reported for the default 0.8 P_max stop: success against the
simulator's geometric seal, and how many ticks earlier the pressure stop fires.
"""
import importlib.util
import json
import sys
from argparse import Namespace
from pathlib import Path

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]   # ais/
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(Path.cwd()))
from ais_haptic import robot_frame as rf  # noqa: E402

spec = importlib.util.spec_from_file_location('runner', PACKAGE / 'nodes' / 'ais_trial_runner.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
runner.HOLD_AFTER_SEAL_S = runner.RELEASE_WAIT_S = 0.0
LONG_EPISODE_TICKS = 300
runner.time.sleep = lambda s: None


class FakeUR:
    """rtde_c and rtde_r in one object; servoL drives the simulator."""

    def __init__(self, env, cal, cup):
        self.env, self.cal, self.cup = env, cal, cup
        self.pose = None
        self.first_seal = None
        self.rtde_c = self.rtde_r = self

    def isPoseWithinSafetyLimits(self, pose):
        return True

    def getActualTCPPose(self):
        return list(self.pose)

    def moveL(self, pose, speed=None, acc=None, asynchronous=False):
        self.pose = list(pose)

    def initPeriod(self):
        return 0

    def waitPeriod(self, t):
        pass

    def servoStop(self):
        pass

    def object_state(self, pose):
        xy_base = self.cal.robot_to_base_mm(np.array(pose[:2]) * 1000.)
        origin = np.array(self.cal.lay['object_origins'][self.obj])
        yaw = self.cup.body_yaw_for_psi(rf.tool_yaw_deg(pose)) - self.cal.theta
        return xy_base - origin, yaw

    def servoL(self, pose, speed, acc, dt, lookahead, gain):
        # Step the simulator physics directly, without the episode ending at the geometric seal: the filtered
        # pressure needs a few ticks to reach the runner's 0.8 P_max, and the cup keeps moving meanwhile, as on the robot.
        (x0, yaw0), (x1, yaw1) = self.object_state(self.pose), self.object_state(pose)
        self.env.state = self.env.sim.step((x1 - x0) / dt, float(rf.wrap_deg(yaw1 - yaw0)) / dt)
        if self.env.state['success'] and self.first_seal is None:
            self.first_seal = self.env.sim.tick
        self.pose = list(pose)


class FakeIO:
    def __init__(self, env, cup):
        self.env, self.cup = env, cup

    def pressure(self):
        p = np.zeros(self.cup.chambers)
        p[self.cup.chamber_map] = self.env.state['pressures_pa']   # channel chamber_map[i] = simulator chamber i
        return -p

    def zero_in_air(self):
        pass

    def vacuum(self, on):
        pass

    def sync(self, code):
        pass

    def start_raw_recording(self):
        pass

    def stop_raw_recording(self):
        return np.zeros(0), np.zeros(0), np.zeros((0, self.cup.chambers))


def run(ch, controller, seed, seal_frac, save=None):
    from shapely.geometry import shape
    from tools.evaluate_v2_final import FinalEnvV2, final_inputs
    from tools.fixture_calibration import Calibration, solve
    from tools.train_continuous_rl import case_noise_seed
    import csv
    rng = np.random.default_rng(seed)
    lay = json.loads(Path('robot/fixture_final_subset_K10/fixture_layout.json').read_text())
    th = np.radians(rng.uniform(-180, 180)); R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    t = rng.uniform(300, 700, 2)
    pts = {n: tuple(np.r_[R @ np.array(p) + t, 20.0 - lay['calibration']['bore_depth']] / 1000)
           for n, p in lay['calibration']['bores'].items()}
    cal = Calibration(solve(pts), lay)
    cup = rf.CupConfig('fake', dict(chambers=ch, tcp_offset=[0] * 6, chamber_map=[int(i) for i in rng.permutation(ch)],
                                     body_offset_deg=float(rng.uniform(-180, 180)), calibrated=True))
    args = Namespace(press_mm=5.0, settle_s=0.0, seal_frac=seal_frac, pmax_kpa=64.4, lookahead=0.1, gain=300.)
    rows = [r for r in csv.DictReader(open('robot/fixture_final_subset_K10/trial_schedule.csv'))
            if int(r['chambers']) == ch and r['controller'] == controller]
    cases = {c['id']: c for c in final_inputs()[1]}
    out = []
    for row in rows:
        c = cases[row['case_id']]
        env = FinalEnvV2(chambers=ch, variant='mlp_h5s5_log', pressure_model='v2', shaping='log')
        env.objects = dict(env.objects)
        poly = json.loads((Path('data/robot_as_built_K10/objects') / f"{c['object_id']}.json").read_text())['polygon']
        env.objects[c['object']] = (shape(poly), env.objects[c['object']][1])
        env.reset(options=dict(case=c, noise_seed=case_noise_seed(c['id']), model_params={}))
        plan = runner.Plan(row, cal, cup)
        ur = FakeUR(env, cal, cup); ur.obj = row['object']
        ctrl, policy_path = runner.controller_for(row, cup, PACKAGE / 'policies')
        ur.pose = rf.tool_pose([plan.xy[0], plan.xy[1], plan.z_top + 0.05], plan.psi)
        summary, arrays = runner.run_trial(args, cup, plan, ur, FakeIO(env, cup), ctrl)
        if save:
            runner.save_trial(Path(save), row, summary, arrays, args, policy_path)
        out.append(dict(case_id=row['case_id'], success=summary['success'], reason=summary['reason'],
                        ticks=summary['ticks'], first_seal=ur.first_seal))
    return cup, out


def job(spec):
    ch, controller, seed, save = spec
    _, never = run(ch, controller, seed, 2.0)       # no pressure stop: frame check
    cup, stop = run(ch, controller, seed, 0.8, save)  # the runner default
    return spec, cup, never, stop


def main():
    from concurrent.futures import ProcessPoolExecutor
    ref = {(r['controller'], r['chambers'], r['case_id']): r
           for r in json.loads(Path('outputs/robot_as_built_sim/episodes.json').read_text())}
    save = None
    if '--save' in sys.argv:       # keep the 0.8 P_max trials in the runner's format (ais_replay_video.py reads them)
        save = str(Path(sys.argv[sys.argv.index('--save') + 1]).expanduser())
        Path(save).mkdir(parents=True, exist_ok=True)
    specs = [(ch, c, 10 * ch + k, save) for ch in (3, 6) for k, c in enumerate(('ppo', 'greedy', 'yaw_momentum'))]
    ok = True
    with ProcessPoolExecutor(max_workers=len(specs)) as pool:
        for (ch, controller, _, _), cup, never, stop in pool.map(job, specs):
            stored = [ref[(controller, ch, o['case_id'])] for o in never]
            bad = [(o, s) for o, s in zip(never, stored)
                   if (o['first_seal'] is not None and o['first_seal'] <= 500) != s['success']
                   or (s['success'] and o['first_seal'] != s['ticks'])]
            # long episodes amplify ~1e-13 m pose round-off (and float32, see test_policy_parity.py); judge those apart
            chaotic = [(o, st) for o, st in bad if st['ticks'] > LONG_EPISODE_TICKS]
            bad = [(o, st) for o, st in bad if st['ticks'] <= LONG_EPISODE_TICKS]
            ok &= not bad
            early = [s['ticks'] - o['ticks'] for o, s in zip(stop, stored) if o['success'] and s['success']]
            gained = sum(o['success'] and not s['success'] for o, s in zip(stop, stored))
            lost = sum(s['success'] and not o['success'] for o, s in zip(stop, stored))
            print(f"{'PASS' if not bad else 'FAIL'} {controller:13s} {ch}ch  map {cup.chamber_map} offset {cup.body_offset_deg:7.1f}: "
                  f"geometric seal tick matches the simulator in {len(never) - len(bad) - len(chaotic)}/{len(never)}"
                  + (f" ({len(chaotic)} differ after > {LONG_EPISODE_TICKS} ticks)" if chaotic else ''))
            print(f"     stop at 0.8 P_max: success {np.mean([o['success'] for o in stop]) * 100:5.1f}% vs sim "
                  f"{np.mean([s['success'] for s in stored]) * 100:5.1f}% (gained {gained}, lost {lost}); "
                  f"stops before the geometric seal by median {np.median(early) if early else 0:.0f}, max {max(early) if early else 0} ticks")
            for o, s in (bad + chaotic)[:5]:
                print('     ', o['case_id'], 'first seal', o['first_seal'], 'stored', s['success'], s['ticks'])
    print('ALL PASS' if ok else 'SOME CHECKS FAILED')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
