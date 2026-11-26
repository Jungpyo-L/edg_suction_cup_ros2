#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Run haptic-search trials from 2D_Haptic_Search/robot/fixture_final_subset_K10/trial_schedule.csv on the UR10.
#
#   ros2 run suction_cup ais_trial_runner.py --cup ch3 --dry-run            # plan only: no ROS, no robot
#   ros2 run suction_cup ais_trial_runner.py --cup ch3 --trials 0-5 --step  # pilot, Enter before each trial
#   ros2 run suction_cup ais_trial_runner.py --cup ch3                      # the whole 3-chamber block, resumable
#
# Per trial: lift to the travel height, moveJ above the start with wrist 3 unwound (ais_haptic.motion) ->
# hover above the start (tool yaw from the simulator start yaw) -> zero sensors in air -> descend to the
# contact height minus press_mm -> vacuum on, settle -> 50 Hz loop (controller -> servoL) until every chamber reaches
# seal_frac * P_max, 10 s, the 30 mm object boundary, or a tracking fault -> hold 1 s -> vacuum off -> lift.
# Static pressure for the pressure-model parity plot: mean/std of the chamber vacuum over the last 0.5 s of the still
# period before the controller starts and of the hold after it stops, with the cup pose in the object frame
# (summary keys static_start / static_end).
# Matches the simulator: 10 mm/s, 10 deg/s, body-frame actions (ais_haptic.policy, ais_haptic.robot_frame).
# Each trial is saved as <results>/trial_XXXX.npz (per tick + raw sensor stream) and .json (summary), one row of
# <results>/results.csv (cup, object, start pose, controller, success...), and, by default, a data_logger .mat
# (/SensorPacket, /endEffectorPose, /netft_data, /sync, plus the same trial info) in <results>/datalog/<yymmdd>/.
# Trials with a .json are skipped, so rerunning the same command resumes. Ctrl+C stops safely; that trial is saved as aborted and
# rerun next time.

import sys
from pathlib import Path

# With colcon --symlink-install this file runs from ais/nodes/: put ais/ (ais_haptic) and src/ (helperFunction)
# on the path. A copied install has both next to the script already.
_NODES = Path(__file__).resolve().parent
sys.path[:0] = [str(_NODES.parent), str(_NODES.parent.parent / "src")]

import argparse
import csv
import hashlib
import os
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from ais_haptic.motion import WRIST3_LIMIT_DEG, move_unwound, wrist3_deg
from ais_haptic.policy import CONTROL_HZ, DURATION_S, P_MAX_PA, make_controller
from ais_haptic.robot_frame import (BodyCommandIntegrator, load_cups, tool_pose, tool_tilt_deg,
                                               tool_yaw_deg, wrap_deg)

HOVER_MM = 10.0
TRAVEL_MM = 30.0           # free-space height for the joint move that unwinds the wrist
WRIST3_MARGIN_DEG = 10.0   # stop a trial this close to the wrist 3 limit
MOVE_SPEED, MOVE_ACC = 0.05, 0.3
TOUCH_SPEED, TOUCH_ACC = 0.005, 0.1
BOUNDARY_MM = 30.0
TRACKING_LIMIT_MM = 3.0
HOLD_AFTER_STOP_S = 1.0     # still, vacuum on, after every trial end (sealed or not)
STATIC_WINDOW_S = 0.5       # static-pressure statistics use the last part of a still period
RELEASE_WAIT_S = 0.5
POLICY_FILES = {('ppo', 3): 'ppo_ch3_v3_s2_ch3_s4.npz', ('ppo', 6): 'ppo_ch6_v3_s2_ch6_s4.npz',
                ('ppo', 5): 'ppo_ch5_v3_s2_ch5_s4.npz', ('ppo', 4): 'ppo_ch4_v3_s2_ch4_s1.npz'}
POLICY_NAMES = {f: f.split('_v3_')[1][:-4] for f in POLICY_FILES.values()}   # v3 refit runs, e.g. s2_ch3_s4


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
        self.frame = dict(object_origin_robot_m=self.origin_xy.tolist(), calibration_rotation_deg=float(cal.theta),
                          body_offset_deg=cup.body_offset_deg, chamber_map=cup.chamber_map, chambers=cup.chambers)

    def object_distance_mm(self, xy_m):
        return float(np.linalg.norm(np.asarray(xy_m) - self.origin_xy) * 1000.)

    def object_pose(self, ur_pose, cup):
        """UR pose -> cup centre (mm) and simulator body yaw (deg) in the object frame."""
        th = np.radians(self.frame['calibration_rotation_deg'])
        d = (np.asarray(ur_pose[:2]) - self.origin_xy) * 1000.
        x, y = np.cos(th) * d[0] + np.sin(th) * d[1], -np.sin(th) * d[0] + np.cos(th) * d[1]
        yaw = wrap_deg(cup.body_yaw_for_psi(tool_yaw_deg(ur_pose)) - self.frame['calibration_rotation_deg'])
        return float(x), float(y), float(yaw)


def static_pressure(io, cup, plan, rtde, seconds, window_s=STATIC_WINDOW_S):
    """Hold still for `seconds` with the vacuum on; statistics of the 7-point-average chamber vacuum (simulator order,
    Pa) over the last `window_s`, with the cup pose in the object frame. For the measured-vs-model parity plot."""
    t, samples = [], []
    t_end = time.monotonic() + seconds
    while True:
        t.append(time.monotonic())
        samples.append(cup.sim_vacuum(io.pressure()))
        if time.monotonic() >= t_end:
            break
        time.sleep(0.01)
    t, samples = np.array(t), np.array(samples)
    last = samples[t >= t[-1] - window_s]
    x, y, yaw = plan.object_pose(list(rtde.rtde_r.getActualTCPPose()), cup)
    return dict(mean_pa=last.mean(axis=0).tolist(), std_pa=last.std(axis=0).tolist(), n=int(len(last)),
                window_s=window_s, x_obj_mm=x, y_obj_mm=y, yaw_obj_deg=yaw), samples


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


def vacuum_scale(args):
    """Factor applied to the measured vacuum before it goes to a controller (1 unless --normalize_vacuum)."""
    return P_MAX_PA / (args.pmax_kpa * 1000.0) if getattr(args, 'normalize_vacuum', False) else 1.0


def seal_threshold_pa(args):
    """Success threshold (Pa, every chamber): --seal_kpa when given, else seal_frac * pmax_kpa."""
    kpa = getattr(args, 'seal_kpa', None)
    return kpa * 1000.0 if kpa is not None else args.seal_frac * args.pmax_kpa * 1000.0


def run_trial(args, cup, plan, rtde, io, ctrl):
    """One trial on the robot. Returns (summary dict, per-tick arrays dict)."""
    dt = 1.0 / CONTROL_HZ                       # simulator time per control tick: the controllers always see this
    time_scale = float(getattr(args, 'time_scale', 1.0))
    period = dt * time_scale                    # real time per tick: time_scale 4 = same path per tick at 1/4 speed
    vac_scale = vacuum_scale(args)
    seal_pa = seal_threshold_pa(args)
    hover = tool_pose([plan.xy[0], plan.xy[1], plan.z_top + HOVER_MM / 1000.0], plan.psi)
    touch = tool_pose([plan.xy[0], plan.xy[1], plan.z_top + 0.001], plan.psi)
    contact = tool_pose([plan.xy[0], plan.xy[1], plan.z_top - args.press_mm / 1000.0], plan.psi)
    for pose in (hover, contact):
        if not rtde.rtde_c.isPoseWithinSafetyLimits(pose):
            raise RuntimeError('Start pose outside the robot safety limits: %s' % np.round(pose, 4))

    # approach in air with the wrist unwound, zero, press, vacuum on
    travel_z = plan.z_top + TRAVEL_MM / 1000.0
    current = list(rtde.rtde_r.getActualTCPPose())
    if current[2] < travel_z:
        current[2] = travel_z
        rtde.rtde_c.moveL(current, MOVE_SPEED, MOVE_ACC)
    q6_before = wrist3_deg(rtde)
    move_unwound(rtde, tool_pose([plan.xy[0], plan.xy[1], travel_z], plan.psi), args.q6_home_deg)
    q6_start = wrist3_deg(rtde)
    rtde.rtde_c.moveL(hover, MOVE_SPEED, MOVE_ACC)
    time.sleep(0.3)
    if hasattr(io, 'require_fresh_pressure'):
        io.require_fresh_pressure()            # before any contact: a dead sensor stream stops the run here
    io.zero_in_air()
    if getattr(args, 'datalog', False):
        io.datalog(True)
    rtde.rtde_c.moveL(touch, MOVE_SPEED / 2, MOVE_ACC)
    rtde.rtde_c.moveL(contact, TOUCH_SPEED, TOUCH_ACC)
    ctrl.reset()
    integ = BodyCommandIntegrator(plan.xy, plan.z_top - args.press_mm / 1000.0, plan.theta, cup, dt)
    io.start_raw_recording()
    io.vacuum(True)
    static_start, _ = static_pressure(io, cup, plan, rtde, args.settle_s)   # still at the start pose, before control

    log = {k: [] for k in ('t', 'sensor_pa', 'vacuum_sim_pa', 'action', 'target', 'actual', 'theta_cmd', 'obs')}
    reason, t0, seal_tick = 'timeout', time.monotonic(), None
    next_tick, overruns = time.monotonic(), 0
    q6_range = [q6_start, q6_start]
    io.sync(1)
    try:
        for tick in range(int(round(DURATION_S * CONTROL_HZ))):
            sensor = io.pressure()
            age = io.pressure_age_s() if hasattr(io, 'pressure_age_s') else None
            if isinstance(age, float) and age > 0.5:
                reason = 'sensor_lost'
                break
            vac = cup.sim_vacuum(sensor)
            actual = list(rtde.rtde_r.getActualTCPPose())
            q6 = wrist3_deg(rtde)
            q6_range[0], q6_range[1] = min(q6_range[0], q6), max(q6_range[1], q6)
            if abs(q6) > WRIST3_LIMIT_DEG - WRIST3_MARGIN_DEG:
                reason = 'joint_limit'
                break
            if np.all(vac >= seal_pa):
                reason, seal_tick = 'sealed', tick
                break
            # The controllers were fixed for a sealed vacuum of P_MAX_PA; with --normalize_vacuum the measured vacuum
            # is rescaled so that this session's sealed vacuum (--pmax_kpa) maps onto it. The stop test above uses
            # the measured values.
            a = ctrl.act(vac * vac_scale, integ.theta, dt)
            target = integ.step(a)
            if plan.object_distance_mm(integ.xy) > BOUNDARY_MM:
                reason = 'boundary'
                break
            err_mm = np.linalg.norm(np.array(actual[:3]) - log['target'][-1][:3]) * 1000 if log['target'] else 0.0
            if err_mm > TRACKING_LIMIT_MM:
                reason = 'tracking_fault'
                break
            rtde.rtde_c.servoL(target, 0.0, 0.0, period, args.lookahead, args.gain)
            log['t'].append(time.monotonic() - t0)
            log['sensor_pa'].append(sensor)
            log['vacuum_sim_pa'].append(vac)
            log['action'].append(a)
            log['target'].append(target)
            log['actual'].append(actual)
            log['theta_cmd'].append(integ.theta)
            log['obs'].append(getattr(ctrl, 'last_obs', None) if getattr(ctrl, 'last_obs', None) is not None
                              else np.zeros(0))
            # Our own 50 Hz clock: rtde_c.waitPeriod() waits one RTDE period (1/125 s), which would run this loop
            # at 125 Hz while each target advances 1/50 s of motion, i.e. 2.5x the simulator speed.
            next_tick += period
            slack = next_tick - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                overruns += 1
    finally:
        rtde.rtde_c.servoStop()
        elapsed = time.monotonic() - t0
        io.sync(2)
    static_end, hold = static_pressure(io, cup, plan, rtde, HOLD_AFTER_STOP_S)   # still at the end pose
    io.vacuum(False)
    time.sleep(RELEASE_WAIT_S)
    raw_t, raw_stamp, raw_pa = io.stop_raw_recording()
    end_pose = list(rtde.rtde_r.getActualTCPPose())
    lift = list(end_pose)
    lift[2] = plan.z_top + HOVER_MM / 1000.0
    rtde.rtde_c.moveL(lift, MOVE_SPEED / 2, MOVE_ACC)
    if getattr(args, 'datalog', False):
        io.datalog(False)

    actual = np.array(log['actual']) if log['actual'] else np.zeros((0, 6))
    path_actual = float(np.sum(np.linalg.norm(np.diff(actual[:, :2], axis=0), axis=1)) * 1000) if len(actual) > 1 else 0.
    summary = dict(trial=int(plan.row['trial']), success=reason == 'sealed', reason=reason, time_s=float(elapsed),
                   ticks=len(log['t']), seal_tick=seal_tick, path_cmd_mm=integ.path_mm, path_actual_mm=path_actual,
                   yaw_path_deg=integ.yaw_path_deg,
                   end_object_distance_mm=plan.object_distance_mm(end_pose[:2]),
                   hold_min_vacuum_pa=np.min(hold, axis=0).tolist(),
                   static_start=static_start, static_end=static_end,
                   start=dict(x_m=plan.xy[0], y_m=plan.xy[1], z_top_m=plan.z_top, body_yaw_deg=plan.theta, tool_yaw_deg=plan.psi),
                   frame=plan.frame,
                   wrist3_deg=dict(before_unwind=q6_before, start=q6_start, min=q6_range[0], max=q6_range[1]),
                   raw_rate_hz=float((len(raw_t) - 1) / (raw_t[-1] - raw_t[0])) if len(raw_t) > 1 else None,
                   loop_hz=float((len(log['t']) - 1) / (log['t'][-1] - log['t'][0])) if len(log['t']) > 1 else None,
                   loop_overruns=overruns)
    arrays = {k: np.array(v) for k, v in log.items() if k != 'obs'}
    arrays['obs'] = np.array(log['obs']) if log['obs'] and len(log['obs'][0]) else np.zeros((0,))
    arrays.update(hold_vacuum_pa=np.array(hold), raw_receive_s=raw_t - t0 if len(raw_t) else raw_t,
                  raw_stamp_s=raw_stamp, raw_pa=raw_pa)
    return summary, arrays


RESULT_COLUMNS = ['trial', 'label', 'chambers', 'controller', 'policy', 'object_no', 'object', 'start_index', 'success',
                  'reason', 'time_s', 'path_cmd_mm', 'path_actual_mm', 'end_object_distance_mm', 'loop_hz',
                  'datalog_mat', 'finished']


def trial_info(row):
    """Readable identity of a schedule row: cup, object number, start pose number (0-9 per object), controller."""
    start = int(row['case_id'].split(':')[-1])
    label = 'ch%s_obj%02d_pose%d_%s' % (row['chambers'], int(row['object_no']), start, row['controller'])
    return dict(label=label, chambers=int(row['chambers']), object_no=int(row['object_no']), object=row['object'],
                start_index=start, controller=row['controller'], policy=row['policy'])


def save_trial(results, row, summary, arrays, args, policy_path):
    """<results>/trial_XXXX.npz (per tick, raw stream) and .json (summary), plus a row of <results>/results.csv.
    The .json marks the trial as done."""
    summary.update(trial_info(row), row=row, policy_file=str(policy_path) if policy_path else None,
                   seal_frac=args.seal_frac, pmax_kpa=args.pmax_kpa, seal_kpa=getattr(args, 'seal_kpa', None), seal_threshold_pa=seal_threshold_pa(args),
                   time_scale=float(getattr(args, 'time_scale', 1.0)), vacuum_scale=vacuum_scale(args), press_mm=args.press_mm,
                   finished=datetime.now().isoformat(timespec='seconds'))
    summary.setdefault('datalog_mat', None)
    np.savez_compressed(results / ('trial_%04d.npz' % int(row['trial'])), **arrays)
    (results / ('trial_%04d.json' % int(row['trial']))).write_text(json.dumps(summary, indent=1, default=float))
    table = results / 'results.csv'
    new = not table.exists()
    with open(table, 'a', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=RESULT_COLUMNS, extrasaction='ignore')
        if new:
            w.writeheader()
        w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in summary.items() if k in RESULT_COLUMNS})


def save_datalog(file_help, row, summary, cup):
    """data_logger CSVs -> .mat (fileSaveHelp.saveDataParams), with the trial identity and outcome as variables."""
    info = trial_info(row)
    meta = argparse.Namespace(**info, trial=int(row['trial']), case_id=row['case_id'],
                              start_x_object_mm=float(row['start_x_object']), start_y_object_mm=float(row['start_y_object']),
                              start_yaw_deg=float(row['start_yaw_deg']), success=int(summary['reason'] == 'sealed'),
                              reason=summary['reason'], time_s=summary['time_s'], path_cmd_mm=summary['path_cmd_mm'],
                              cup=cup.name, chamber_map=list(cup.chamber_map), body_offset_deg=cup.body_offset_deg,
                              trial_npz='trial_%04d.npz' % int(row['trial']))
    outcome = 'success' if summary['reason'] == 'sealed' else 'fail_' + summary['reason']
    folder, name = file_help.saveDataParams(meta, appendTxt='AIS_%s_trial%04d_%s' % (info['label'], int(row['trial']), outcome))
    return str(Path(folder) / ('%s_AIS_%s_trial%04d_%s.mat' % (name, info['label'], int(row['trial']), outcome)))


def main(args):
    sys.path.insert(0, str(Path(args.repo).expanduser()))
    from tools.fixture_calibration import load_calibration

    cups_path = Path(args.cups).expanduser()
    cup = load_cups(cups_path)[args.cup]
    cal_path = Path(args.calibration).expanduser() if args.calibration else \
        Path(args.repo).expanduser() / 'robot' / 'calibration' / ('fixture_calibration_%s.json' % args.cup)
    policy_dir = Path(args.policies).expanduser()
    if args.schedule is None:   # 3ch and 6ch: trial_schedule.csv; 4ch and 5ch (added 2026-11-26): trial_schedule_ch45.csv
        args.schedule = '%s/robot/fixture_final_subset_K10/trial_schedule%s.csv' % (args.repo, '_ch45' if cup.chambers in (4, 5) else '')
    rows = load_rows(args, cup)
    if not rows:
        raise SystemExit('No schedule rows match --cup/--trials/--controllers.')
    if args.dry_run:
        if not cal_path.exists():
            raise SystemExit('No calibration at %s. Run ais_fixture_calibration.py first.' % cal_path)
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
    from ais_haptic.ros_io import HapticRosIO
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
        file_help = None
        if args.datalog:
            from helperFunction.fileSaveHelper import fileSaveHelp
            io.wait_for_datalogger()
            io.datalog(False)
            datalog_dir = results / 'datalog'
            file_help = fileSaveHelp(os.path.relpath(datalog_dir, Path.home()))
            print('data_logger .mat files -> %s' % file_help.ResultSavingDirectory)
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
            if file_help is not None:
                file_help.clearTmpFolder()
            summary, arrays = run_trial(args, cup, plan, rtde, io, ctrl)
            if summary['reason'] == 'sensor_lost':
                raise RuntimeError('pressure stream lost during trial %s' % row['trial'])   # not saved: runs again
            if file_help is not None:
                try:
                    summary['datalog_mat'] = save_datalog(file_help, row, summary, cup)
                except Exception as exc:     # the trial itself is in the .npz; do not lose it to a logger problem
                    print('  WARNING: no data_logger .mat for this trial (%s)' % exc)
            save_trial(results, row, summary, arrays, args, policy_path)
            print('  -> %s: %s in %.2f s, path %.1f mm (cmd), %.1f mm from the object origin at the end' % (
                summary['label'], 'SUCCESS' if summary['success'] else 'FAIL (%s)' % summary['reason'],
                summary['time_s'], summary['path_cmd_mm'], summary['end_object_distance_mm']))
    except KeyboardInterrupt:
        print('\nStopped by the user. The interrupted trial is not saved and will run again next time.')
    except RuntimeError as exc:
        print('\nSTOPPED: %s\nThe interrupted trial is not saved and will run again next time.' % exc)
        exit_code = 2
    finally:
        io.vacuum(False)
        if args.datalog:
            try:
                io.datalog(False)
            except Exception as exc:
                print('Could not stop data_logger: %s' % exc)
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
    if locals().get('exit_code'):
        sys.exit(exit_code)      # so that "runner && next_tool" does not start the next tool after a failure


if __name__ == '__main__':
    repo = '~/projects/2D_Haptic_Search'
    parser = argparse.ArgumentParser()
    parser.add_argument('--cup', required=True, help='cup in ais/config/cups.json (ch3, ch4, ch5, ch6); selects that block')
    parser.add_argument('--trials', default=None, help='trial numbers, e.g. 0-5,12')
    parser.add_argument('--controllers', default=None, help='comma list, e.g. ppo,yaw_momentum')
    parser.add_argument('--dry-run', action='store_true', help='plan only: print start poses, load controllers; no ROS/robot')
    parser.add_argument('--step', action='store_true', help='wait for Enter before each trial')
    parser.add_argument('--press_mm', type=float, default=5.0, help='press depth below the contact height (simulator: 5)')
    parser.add_argument('--settle_s', type=float, default=1.0, help='vacuum on, cup still, before the controller starts')
    parser.add_argument('--seal_frac', type=float, default=0.8, help='success: every chamber >= seal_frac * P_max')
    parser.add_argument('--time_scale', type=float, default=1.0,
                        help='real time per control tick / simulator tick (0.02 s). 4 = the same path per tick at a '
                             'quarter of the speed (2.5 mm/s, 2.5 deg/s, 40 s limit), so that the vacuum (about 0.4 s lag) '
                             'follows the cup as the quasi-static simulator assumes')
    parser.add_argument('--normalize_vacuum', action='store_true',
                        help='feed the controllers measured vacuum x (simulator P_max / --pmax_kpa)')
    parser.add_argument('--seal_kpa', type=float, default=None,
                        help='success: every chamber >= this vacuum (kPa); overrides seal_frac * pmax_kpa')
    parser.add_argument('--pmax_kpa', type=float, default=P_MAX_PA / 1000, help='sealed vacuum measured this session')
    parser.add_argument('--lookahead', type=float, default=0.1, help='servoL lookahead time (s), 0.03-0.2')
    parser.add_argument('--gain', type=float, default=300.0, help='servoL gain, 100-2000')
    parser.add_argument('--datalog', action=argparse.BooleanOptionalAction, default=True,
                        help='also record data_logger.py topics to a .mat per trial (default on; --no-datalog to skip)')
    parser.add_argument('--q6_home_deg', type=float, default=0.0, help='wrist 3 is unwound to within 180 deg of this before each trial')
    parser.add_argument('--schedule', default=None, help='default: trial_schedule.csv (3ch, 6ch) or trial_schedule_ch45.csv (4ch, 5ch) of the fixture')
    parser.add_argument('--calibration', default=None, help='default <repo>/robot/calibration/fixture_calibration_<cup>.json')
    parser.add_argument('--results', default='~/EDG_Experiment/ais_robot_demo')
    parser.add_argument('--repo', default=repo)
    parser.add_argument('--cups', default='~/ros2_ws/src/suction_cup/ais/config/cups.json')
    parser.add_argument('--policies', default='~/ros2_ws/src/suction_cup/ais/policies')
    main(parser.parse_args())
