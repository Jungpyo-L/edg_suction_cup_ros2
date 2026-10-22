#!/usr/bin/env python3
"""Check ais_haptic/policy against the 2D_Haptic_Search simulator, tick by tick.

Run from the simulator repo in the suctionRL env:
  cd ~/projects/2D_Haptic_Search && taskset -c 0-3,5-15,17-31 \
    ~/miniconda3/envs/suctionRL/bin/python ~/ros2_ws/src/suction_cup/ais/tools/test_policy_parity.py
Replays the robot-experiment cases on the as-built polygons (as tools/analyze_robot_plate_layout.py --as-built),
but every action comes from ais_haptic.policy. At each tick it compares
  - policies: our observation with the env observation, and our action with limit(model.predict(obs));
  - heuristics: our action (fed yaw + a constant offset) with the ContinuousHeuristic conversion of the script.
The robot-side integrator (robot_frame.BodyCommandIntegrator) must track the simulator cup position and yaw.
Each episode's success must equal outputs/robot_as_built_sim/episodes.json. Episode length is only reported:
float32 rounding (~1e-6 per action) can shift a long episode's seal by a tick or two (seen once, PPO 3ch, 480 vs 481).
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]   # ais/
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(Path.cwd()))
from ais_haptic import policy as hp  # noqa: E402
from ais_haptic import robot_frame as rf  # noqa: E402

POLICIES = {('ppo', 3): ('ppo_ch3_s2_ch3_s5', 'outputs/v2_s234_20261019/runs/s2_ch3_s5/final.zip'),
            ('ppo', 6): ('ppo_ch6_s2_ch6_s1', 'outputs/v2_s234_20261019/runs/s2_ch6_s1/final.zip'),
            ('sac', 3): ('sac_ch3_ch3_sac_log_s3', 'outputs/v2_s1_confirm_20261019/runs/ch3_sac_log_s3/final.zip')}
# robot controllers (PPO, greedy, yaw-momentum) first; the rest stay as extra checks of the same code
JOBS = [(c, ch) for ch in (3, 6) for c in ('ppo', 'greedy', 'yaw_momentum')] + \
       [('yaw', 3), ('momentum', 3), ('sac', 3), ('yaw', 6)]
YAW_OFFSET_DEG = 37.  # heuristics must not depend on the absolute yaw reference
OBS_TOL, ACT_TOL = 1e-6, 5e-5  # float32 forward pass vs torch
POSE_TOL_MM = 1e-6
IDENTITY_CUP = {ch: rf.CupConfig(f'id{ch}', dict(chambers=ch, tcp_offset=[0] * 6, chamber_map=list(range(ch)),
                                                body_offset_deg=0., calibrated=True)) for ch in (3, 6)}


def run(job):
    import torch
    torch.set_num_threads(1)
    from shapely.geometry import shape
    from stable_baselines3 import PPO, SAC
    from haptic_search.continuous.controllers import ContinuousHeuristic
    from tools.evaluate_v2_final import FinalEnvV2, final_inputs
    from tools.train_continuous_rl import case_noise_seed
    ctrl_name, ch = job
    objects = json.loads(Path('robot/laser_cut_final_subset_K10/robot_subset_K10.json').read_text())['objects']
    model = None
    if ctrl_name in ('ppo', 'sac'):
        name, zip_path = POLICIES[(ctrl_name, ch)]
        model = (PPO if ctrl_name == 'ppo' else SAC).load(zip_path, device='cpu')
        ours = hp.make_controller(ctrl_name, ch, PACKAGE / 'policies' / f'{name}.npz')
    else:
        ours = hp.make_controller(ctrl_name, ch)
    _, cases = final_inputs()
    rows, worst_obs, worst_act, worst_pose = [], 0., 0., 0.
    for c in [c for c in cases if c['object_id'] in objects]:
        env = FinalEnvV2(chambers=ch, variant='mlp_h5s5_log', pressure_model='v2', shaping='log')
        env.objects = dict(env.objects)
        poly = json.loads((Path('data/robot_as_built_K10/objects') / f"{c['object_id']}.json").read_text())['polygon']
        env.objects[c['object']] = (shape(poly), env.objects[c['object']][1])
        o, _ = env.reset(options=dict(case=c, noise_seed=case_noise_seed(c['id']), model_params={}))
        ours.reset()
        ref_h = ContinuousHeuristic(ctrl_name, ch, 10., 10., .5) if model is None else None
        integ = rf.BodyCommandIntegrator(np.array([env.state['x_mm'], env.state['y_mm']]) / 1000., 0., env.state['yaw_deg'],
                                         IDENTITY_CUP[ch])
        te = False
        for tick in range(500):
            p, yaw = env.state['pressures_pa'], env.state['yaw_deg']
            a = ours.act(p, yaw + YAW_OFFSET_DEG, 1 / env.hz)
            if model is not None:
                worst_obs = max(worst_obs, float(np.abs(ours.last_obs - o).max()))
                ref = hp.limit_action(model.predict(o, deterministic=True)[0])
            else:
                v, w = ref_h.command(p, yaw, 1 / env.hz)
                ref = hp.limit_action(np.r_[hp.rotation(-yaw) @ v / env.speed, w / env.yaw_rate])
            worst_act = max(worst_act, float(np.abs(a - ref).max()))
            o, _, te, tr, _ = env.step(a)
            integ.step(a)
            worst_pose = max(worst_pose, float(np.abs(integ.xy * 1000. - [env.state['x_mm'], env.state['y_mm']]).max()),
                             abs(integ.theta - env.state['yaw_deg']))
            if te or tr:
                break
        rows.append(dict(controller=ctrl_name, chambers=ch, case_id=c['id'], success=bool(te), ticks=tick + 1))
    return rows, worst_obs, worst_act, worst_pose


def main():
    ref = {(r['controller'], r['chambers'], r['case_id']): r
           for r in json.loads(Path('outputs/robot_as_built_sim/episodes.json').read_text())}
    ok = True
    with ProcessPoolExecutor(max_workers=len(JOBS)) as pool:
        for job, (rows, wo, wa, wp) in zip(JOBS, pool.map(run, JOBS)):
            stored = [ref.get((r['controller'], r['chambers'], r['case_id'])) for r in rows]
            have = [(r, s) for r, s in zip(rows, stored) if s is not None]
            mismatch = sum(r['success'] != s['success'] for r, s in have)
            shifted = sum(r['ticks'] != s['ticks'] for r, s in have)
            rate = np.mean([r['success'] for r in rows])
            good = wo <= OBS_TOL and wa <= ACT_TOL and wp <= POSE_TOL_MM and mismatch == 0
            ok &= good
            print(f"{'PASS' if good else 'FAIL'} {job[0]:13s} {job[1]}ch  n={len(rows)}  success={rate * 100:5.1f}%  "
                  f"max|dobs|={wo:.1e}  max|dact|={wa:.1e}  max|dpose|={wp:.1e}  vs stored: {len(have)} compared, {mismatch} outcomes differ, {shifted} lengths differ")
    print('ALL PASS' if ok else 'SOME CHECKS FAILED')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
