#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Run haptic-search trials from 2D_Haptic_Search/robot/fixture_final_subset_K10/trial_schedule.csv on the UR10.
#
#   ros2 run suction_cup haptic_trial_runner.py --cup ch3 --dry-run            # plan only: no ROS, no robot
#   ros2 run suction_cup haptic_trial_runner.py --cup ch3 --trials 0-5 --step  # pilot, Enter before each trial
#   ros2 run suction_cup haptic_trial_runner.py --cup ch3                      # the whole 3-chamber block, resumable
#
# Per trial: hover above the start (tool yaw from the simulator start yaw) -> zero sensors in air -> descend to the
# contact height minus press_mm -> vacuum on, settle -> 50 Hz loop (controller -> servoL) until every chamber reaches
# seal_frac * P_max (then hold 1 s), 10 s, the 30 mm object boundary, or a tracking fault -> vacuum off -> lift.
# Matches the simulator: 10 mm/s, 10 deg/s, body-frame actions (helperFunction/haptic_policy, haptic_robot_frame).
# Each trial is saved as <results>/trial_XXXX.npz (per tick + raw sensor stream) and .json (summary); trials with a
# .json are skipped, so rerunning the same command resumes. Ctrl+C stops safely; that trial is saved as aborted and
# rerun next time.

import argparse
import csv
import hashlib
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from helperFunction.haptic_policy import CONTROL_HZ, DURATION_S, P_MAX_PA, make_controller
from helperFunction.haptic_robot_frame import (BodyCommandIntegrator, load_cups, tool_pose, tool_tilt_deg,
                                               tool_yaw_deg, wrap_deg)

HOVER_MM = 10.0
MOVE_SPEED, MOVE_ACC = 0.05, 0.3
TOUCH_SPEED, TOUCH_ACC = 0.005, 0.1
BOUNDARY_MM = 30.0
TRACKING_LIMIT_MM = 3.0
HOLD_AFTER_SEAL_S = 1.0
RELEASE_WAIT_S = 0.5
POLICY_FILES = {('ppo', 3): 'ppo_ch3_s2_ch3_s5.npz', ('ppo', 6): 'ppo_ch6_s2_ch6_s1.npz',
                ('sac', 3): 'sac_ch3_ch3_sac_log_s3.npz'}
POLICY_NAMES = {'ppo_ch3_s2_ch3_s5.npz': 's2_ch3_s5', 'ppo_ch6_s2_ch6_s1.npz': 's2_ch6_s1',
                'sac_ch3_ch3_sac_log_s3.npz': 'ch3_sac_log_s3'}


def parse_trials(spec):
    if not spec:
        return None
    out = set()
    for part in spec.split(','):
        lo, _, hi = part.partition('-')
        out.update(range(int(lo), int(hi or lo) + 1))
    return out


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Plan:
    """Robot-frame start pose of one schedule row."""

    def __init__(self, row, cal, cup):
        self.row = row
        xyz, theta = cal.trial_pose(row)
        self.xy, self.z_top, self.theta = np.asarray(xyz[:2]), float(xyz[2]), float(theta)
        self.psi = cup.psi_for_body_yaw(self.theta)
        self.origin_xy = cal.object_pose(row['object'])[0][:2]   # object-frame origin, robot frame (m)

    def object_distance_mm(self, xy_m):
        return float(np.linalg.norm(np.asarray(xy_m) - self.origin_xy) * 1000.)


def load_rows(args, cup):
    rows = list(csv.DictReader(open(Path(args.schedule).expanduser())))
    keep = parse_trials(args.trials)
    controllers = set(args.controllers.split(',')) if args.controllers else None
    rows = [r for r in rows if int(r['chambers']) == cup.chambers
            and (keep is None or int(r['trial']) in keep)
            and (controllers is None or r['controller'] in controllers)]
    return rows


def controller_for(row, cup, policy_dir):
    name = row['controller']
    path = None
    if name in ('ppo', 'sac'):
        path = Path(policy_dir) / POLICY_FILES[(name, cup.chambers)]
        if POLICY_NAMES[path.name] != row['policy']:
            raise ValueError('Schedule asks for policy %s, %s holds %s' % (row['policy'], path.name, POLICY_NAMES[path.name]))
    return make_controller(name, cup.chambers, path), path


def dry_run(args, cup, cal, rows, policy_dir):
    print('cup %s: %d chambers, TCP %s, chamber_map %s, body_offset %s, calibrated %s' % (
        cup.name, cup.chambers, cup.tcp_offset, cup.chamber_map, cup.body_offset_deg, cup.calibrated))
    if not cup.calibrated:
        print('  (uncalibrated cup: tool yaw below uses body_offset 0 and is NOT what the robot will use)')
        cup.body_offset_deg, cup.chamber_map = 0.0, list(range(cup.chambers))
    print('calibration: rotation %.2f deg, xy rms %.3f mm, tilt %.3f deg' % (cal.theta, cal.d['xy_rms_mm'], cal.d['tilt_deg']))
    print('\n trial ctrl          obj  case                  x_mm     y_mm   ztop_mm  body_yaw  tool_yaw')
    zero = np.zeros(cup.chambers)
    for row in rows:
        p = Plan(row, cal, cup)
        ctrl, _ = controller_for(row, cup, policy_dir)
        for _ in range(3):
            ctrl.act(zero, p.theta)
        print('%5s %-13s %3s  %-20s %8.2f %8.2f %8.2f %8.1f %8.1f' % (
            row['trial'], row['controller'], row['object_no'], row['case_id'], p.xy[0] * 1000, p.xy[1] * 1000,
            p.z_top * 1000, p.theta, p.psi))
    print('\n%d trials planned; controllers load and run on zero pressure.' % len(rows))


def run_trial(args, cup, plan, rtde, io, ctrl):
    """One trial on the robot. Returns (summary dict, per-tick arrays dict)."""
    dt = 1.0 / CONTROL_HZ
    p_max = args.pmax_kpa * 1000.0
    seal_pa = args.seal_frac * p_max
    hover = tool_pose([plan.xy[0], plan.xy[1], plan.z_top + HOVER_MM / 1000.0], plan.psi)
    touch = tool_pose([plan.xy[0], plan.xy[1], plan.z_top + 0.001], plan.psi)
    contact = tool_pose([plan.xy[0], plan.xy[1], plan.z_top - args.press_mm / 1000.0], plan.psi)
    for pose in (hover, contact):
        if not rtde.rtde_c.isPoseWithinSafetyLimits(pose):
            raise RuntimeError('Start pose outside the robot safety limits: %s' % np.round(pose, 4))

    # approach in air, zero, press, vacuum on
    current = list(rtde.rtde_r.getActualTCPPose())
    if current[2] < plan.z_top + HOVER_MM / 1000.0:
        current[2] = plan.z_top + HOVER_MM / 1000.0
        rtde.rtde_c.moveL(current, MOVE_SPEED, MOVE_ACC)
    rtde.rtde_c.moveL(hover, MOVE_SPEED, MOVE_ACC)
    time.sleep(0.3)
    io.zero_in_air()
    rtde.rtde_c.moveL(touch, MOVE_SPEED / 2, MOVE_ACC)
    rtde.rtde_c.moveL(contact, TOUCH_SPEED, TOUCH_ACC)
    ctrl.reset()
    integ = BodyCommandIntegrator(plan.xy, plan.z_top - args.press_mm / 1000.0, plan.theta, cup, dt)
    io.start_raw_recording()
    io.vacuum(True)
    time.sleep(args.settle_s)

    log = {k: [] for k in ('t', 'sensor_pa', 'vacuum_sim_pa', 'action', 'target', 'actual', 'theta_cmd', 'obs')}
    reason, t0, seal_tick = 'timeout', time.monotonic(), None
    io.sync(1)
    try:
        for tick in range(int(round(DURATION_S * CONTROL_HZ))):
            t_start = rtde.rtde_c.initPeriod()
            sensor = io.pressure()
            vac = cup.sim_vacuum(sensor)
            actual = list(rtde.rtde_r.getActualTCPPose())
            if np.all(vac >= seal_pa):
                reason, seal_tick = 'sealed', tick
                break
            a = ctrl.act(vac, integ.theta, dt)
            target = integ.step(a)
            if plan.object_distance_mm(integ.xy) > BOUNDARY_MM:
                reason = 'boundary'
                break
            err_mm = np.linalg.norm(np.array(actual[:3]) - log['target'][-1][:3]) * 1000 if log['target'] else 0.0
            if err_mm > TRACKING_LIMIT_MM:
                reason = 'tracking_fault'
                break
            rtde.rtde_c.servoL(target, 0.0, 0.0, dt, args.lookahead, args.gain)
            log['t'].append(time.monotonic() - t0)
            log['sensor_pa'].append(sensor)
            log['vacuum_sim_pa'].append(vac)
            log['action'].append(a)
            log['target'].append(target)
            log['actual'].append(actual)
            log['theta_cmd'].append(integ.theta)
            log['obs'].append(getattr(ctrl, 'last_obs', None) if getattr(ctrl, 'last_obs', None) is not None
                              else np.zeros(0))
            rtde.rtde_c.waitPeriod(t_start)
    finally:
        rtde.rtde_c.servoStop()
        elapsed = time.monotonic() - t0
        io.sync(2)
    hold = []
    if reason == 'sealed':
        t_hold = time.monotonic() + HOLD_AFTER_SEAL_S
        while time.monotonic() < t_hold:
            hold.append(cup.sim_vacuum(io.pressure()))
            time.sleep(0.01)
    io.vacuum(False)
    time.sleep(RELEASE_WAIT_S)
    raw_t, raw_stamp, raw_pa = io.stop_raw_recording()
    end_pose = list(rtde.rtde_r.getActualTCPPose())
    lift = list(end_pose)
    lift[2] = plan.z_top + HOVER_MM / 1000.0
    rtde.rtde_c.moveL(lift, MOVE_SPEED / 2, MOVE_ACC)

    actual = np.array(log['actual']) if log['actual'] else np.zeros((0, 6))
    path_actual = float(np.sum(np.linalg.norm(np.diff(actual[:, :2], axis=0), axis=1)) * 1000) if len(actual) > 1 else 0.
    summary = dict(trial=int(plan.row['trial']), success=reason == 'sealed', reason=reason, time_s=float(elapsed),
                   ticks=len(log['t']), seal_tick=seal_tick, path_cmd_mm=integ.path_mm, path_actual_mm=path_actual,
                   yaw_path_deg=integ.yaw_path_deg,
                   end_object_distance_mm=plan.object_distance_mm(end_pose[:2]),
                   hold_min_vacuum_pa=(np.min(hold, axis=0).tolist() if hold else None),
                   start=dict(x_m=plan.xy[0], y_m=plan.xy[1], z_top_m=plan.z_top, body_yaw_deg=plan.theta, tool_yaw_deg=plan.psi),
                   raw_rate_hz=float((len(raw_t) - 1) / (raw_t[-1] - raw_t[0])) if len(raw_t) > 1 else None)
    arrays = {k: np.array(v) for k, v in log.items() if k != 'obs'}
    arrays['obs'] = np.array(log['obs']) if log['obs'] and len(log['obs'][0]) else np.zeros((0,))
    arrays.update(hold_vacuum_pa=np.array(hold), raw_receive_s=raw_t - t0 if len(raw_t) else raw_t,
                  raw_stamp_s=raw_stamp, raw_pa=raw_pa)
    return summary, arrays


def main(args):
    sys.path.insert(0, str(Path(args.repo).expanduser()))
    from tools.fixture_calibration import load_calibration

    cups_path = Path(args.cups).expanduser()
    cup = load_cups(cups_path)[args.cup]
    cal_path = Path(args.calibration).expanduser() if args.calibration else \
        Path(args.repo).expanduser() / 'robot' / 'calibration' / ('fixture_calibration_%s.json' % args.cup)
    policy_dir = Path(args.policies).expanduser()
    rows = load_rows(args, cup)
    if not rows:
        raise SystemExit('No schedule rows match --cup/--trials/--controllers.')
    if args.dry_run:
        if not cal_path.exists():
            raise SystemExit('No calibration at %s. Run haptic_fixture_calibration.py first.' % cal_path)
        return dry_run(args, cup, load_calibration(cal_path), rows, policy_dir)
    cup.require_calibrated()
    cal = load_calibration(cal_path)

    results = Path(args.results).expanduser()
    results.mkdir(parents=True, exist_ok=True)
    todo = [r for r in rows if not (results / ('trial_%04d.json' % int(r['trial']))).exists()]
    print('%d of %d selected trials still to run -> %s' % (len(todo), len(rows), results))
    if not todo:
        return
    session = dict(started=datetime.now().isoformat(timespec='seconds'), cup=vars(cup), calibration=str(cal_path),
                   calibration_sha256=sha256(cal_path), cups_sha256=sha256(cups_path),
                   schedule=str(Path(args.schedule).expanduser()), schedule_sha256=sha256(Path(args.schedule).expanduser()),
                   args=vars(args), policies={p: sha256(policy_dir / p) for p in POLICY_FILES.values() if (policy_dir / p).exists()})
    (results / ('session_%s.json' % datetime.now().strftime('%Y%m%d_%H%M%S'))).write_text(json.dumps(session, indent=1, default=str))

    import rclpy
    from helperFunction.haptic_ros import HapticRosIO
    from helperFunction.rtde_helper import rtdeHelp

    rclpy.init()
    node = rclpy.create_node('haptic_trial_runner')
    io = HapticRosIO(node, cup.chambers)
    rtde = None
    try:
        io.vacuum(False)
        rtde = rtdeHelp(125, node=node)
        rtde.setTCPoffset(cup.tcp_offset)
        time.sleep(0.2)
        io.wait_for_pressure()
        pose = list(rtde.rtde_r.getActualTCPPose())
        if tool_tilt_deg(pose) > 1.0:
            raise RuntimeError('Tool is not pointing down (tilt %.2f deg)' % tool_tilt_deg(pose))
        for k, row in enumerate(todo):
            plan = Plan(row, cal, cup)
            ctrl, policy_path = controller_for(row, cup, policy_dir)
            print('\n[%d/%d] trial %s  %s  object %s  %s  start (%.1f, %.1f) mm obj, yaw %.1f deg' % (
                k + 1, len(todo), row['trial'], row['controller'], row['object_no'], row['case_id'],
                float(row['start_x_object']), float(row['start_y_object']), float(row['start_yaw_deg'])))
            if args.step:
                input('  Enter to run (Ctrl+C to stop) ')
            summary, arrays = run_trial(args, cup, plan, rtde, io, ctrl)
            summary.update(row=row, controller=row['controller'], policy=row['policy'],
                           policy_file=str(policy_path) if policy_path else None,
                           seal_frac=args.seal_frac, pmax_kpa=args.pmax_kpa, press_mm=args.press_mm,
                           finished=datetime.now().isoformat(timespec='seconds'))
            np.savez_compressed(results / ('trial_%04d.npz' % int(row['trial'])), **arrays)
            (results / ('trial_%04d.json' % int(row['trial']))).write_text(json.dumps(summary, indent=1, default=float))
            print('  -> %s in %.2f s, path %.1f mm (cmd), %.1f mm from the object origin at the end' % (
                summary['reason'], summary['time_s'], summary['path_cmd_mm'], summary['end_object_distance_mm']))
    except KeyboardInterrupt:
        print('\nStopped by the user. The interrupted trial is not saved and will run again next time.')
    finally:
        io.vacuum(False)
        if rtde is not None:
            try:
                rtde.rtde_c.servoStop()
                pose = list(rtde.rtde_r.getActualTCPPose())
                if 'plan' in locals():
                    time.sleep(RELEASE_WAIT_S)
                    pose[2] = max(pose[2], plan.z_top + HOVER_MM / 1000.0)
                    rtde.rtde_c.moveL(pose, MOVE_SPEED / 2, MOVE_ACC)
            except Exception as exc:
                print('Could not lift after stopping: %s' % exc)
            rtde.disconnect()
        io.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    repo = '~/projects/2D_Haptic_Search'
    parser = argparse.ArgumentParser()
    parser.add_argument('--cup', required=True, help='cup in config/haptic_cups.json (ch3, ch6); selects that block')
    parser.add_argument('--trials', default=None, help='trial numbers, e.g. 0-5,12')
    parser.add_argument('--controllers', default=None, help='comma list, e.g. ppo,yaw_momentum')
    parser.add_argument('--dry-run', action='store_true', help='plan only: print start poses, load controllers; no ROS/robot')
    parser.add_argument('--step', action='store_true', help='wait for Enter before each trial')
    parser.add_argument('--press_mm', type=float, default=5.0, help='press depth below the contact height (simulator: 5)')
    parser.add_argument('--settle_s', type=float, default=1.0, help='vacuum on, cup still, before the controller starts')
    parser.add_argument('--seal_frac', type=float, default=0.8, help='success: every chamber >= seal_frac * P_max')
    parser.add_argument('--pmax_kpa', type=float, default=P_MAX_PA / 1000, help='sealed vacuum measured this session')
    parser.add_argument('--lookahead', type=float, default=0.1, help='servoL lookahead time (s), 0.03-0.2')
    parser.add_argument('--gain', type=float, default=300.0, help='servoL gain, 100-2000')
    parser.add_argument('--schedule', default=repo + '/robot/fixture_final_subset_K10/trial_schedule.csv')
    parser.add_argument('--calibration', default=None, help='default <repo>/robot/calibration/fixture_calibration_<cup>.json')
    parser.add_argument('--results', default='~/EDG_Experiment/ais_robot_demo')
    parser.add_argument('--repo', default=repo)
    parser.add_argument('--cups', default='~/ros2_ws/src/suction_cup/config/haptic_cups.json')
    parser.add_argument('--policies', default='~/ros2_ws/src/suction_cup/policies')
    main(parser.parse_args())
