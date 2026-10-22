#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Measure which sensor channel is which simulator chamber, and the cup yaw relative to the tool (chamber_map,
# body_offset_deg in ais/config/cups.json). Needs the fixture calibration of the same cup.
#
#   ros2 run suction_cup ais_chamber_check.py --cup ch3 --bore C0 [--write]
#
# The cup lip reaches overlap_mm past the rim of a calibration bore (centre bore radius + lip radius - overlap_mm
# from the bore centre along dir_deg in the base-plate frame, default along the block diagonal). All chambers share
# one supply, so a small overlap gives the clearest per-channel difference (simulator: ~1 mm). It is pressed, read, lifted and turned in yaw steps through a
# full turn with vacuum on. A channel's vacuum is weakest when its chamber faces the bore centre, whose direction in the
# robot frame is known from the calibration, so each channel's direction relative to the tool follows without
# assuming the yaw sign. --write stores the fit in cups.json (calibrated = true).

import sys
from pathlib import Path

# With colcon --symlink-install this file runs from ais/nodes/: put ais/ (ais_haptic) and src/ (helperFunction)
# on the path. A copied install has both next to the script already.
_NODES = Path(__file__).resolve().parent
sys.path[:0] = [str(_NODES.parent), str(_NODES.parent.parent / "src")]

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy

from ais_haptic.motion import move_unwound, wrist3_deg
from ais_haptic.robot_frame import (fit_chamber_directions, load_cups, tool_pose, tool_yaw_deg,
                                               wrap_deg)
from ais_haptic.ros_io import HapticRosIO
from helperFunction.rtde_helper import rtdeHelp

HOVER_MM = 10.0
TRAVEL_MM = 30.0
MOVE_SPEED, MOVE_ACC = 0.03, 0.2
TOUCH_SPEED = 0.005
MIN_SWING_PA = 200.0   # a channel that faces the bore at some yaw loses far more vacuum than this


def fit_weakest_direction(psi_deg, vacuum):
    """Fit vacuum = c0 + c1 cos psi + c2 sin psi; return (psi at the minimum, modulation amplitude, rms residual)."""
    psi = np.radians(psi_deg)
    A = np.c_[np.ones_like(psi), np.cos(psi), np.sin(psi)]
    c, *_ = np.linalg.lstsq(A, vacuum, rcond=None)
    resid = vacuum - A @ c
    return float(np.degrees(np.arctan2(-c[2], -c[1]))), float(np.hypot(c[1], c[2])), float(np.sqrt(np.mean(resid ** 2)))


def analyze_scan(psi, vac, open_dir, verbose=True):
    """psi (deg, tool yaw per step), vac (steps x channels, vacuum positive), open_dir (deg, robot frame, cup -> bore).
    Returns (per-channel report, chamber_map, body_offset_deg, fit rms deg, weakly modulated channels)."""
    alpha, report = [], []
    for j in range(vac.shape[1]):
        psi_min, amp, rms = fit_weakest_direction(psi, vac[:, j])
        alpha.append(float(wrap_deg(open_dir - psi_min)))
        report.append(dict(channel=j, weakest_at_psi=psi_min, direction_in_tool_deg=alpha[-1], modulation_pa=amp,
                           fit_rms_pa=rms, min_pa=float(vac[:, j].min()), max_pa=float(vac[:, j].max())))
        if verbose:
            print("channel %d: weakest at psi %.1f deg -> points %.1f deg from tool x; modulation %.0f Pa (fit rms %.0f)" % (
                j, psi_min, alpha[-1], amp, rms))
    chamber_map, offset, rms = fit_chamber_directions(alpha)
    # the response is not sinusoidal (it saturates when a chamber seals), so judge the swing, not the fit residual
    weak = [r["channel"] for r in report if r["max_pa"] - r["min_pa"] < MIN_SWING_PA]
    if verbose:
        spacing = np.sort(wrap_deg(np.array(alpha) - alpha[0]) % 360)
        print("\nchamber_map %s, body_offset_deg %.1f, fit rms %.1f deg (channel spacing %s)" % (
            chamber_map, offset, rms, np.round(np.diff(np.r_[spacing, 360]), 1)))
        if weak:
            print("WARNING: channels %s barely change with yaw; the lip may not reach the rim (try --overlap_mm)." % weak)
        if rms > 10:
            print("WARNING: channel directions are not evenly spaced (rms %.1f deg); do not trust this fit." % rms)
    return report, chamber_map, offset, rms, weak


def main(args):
    import sys
    sys.path.insert(0, str(Path(args.repo).expanduser()))
    from tools.fixture_calibration import load_calibration

    cups_path = Path(args.cups).expanduser()
    cup = load_cups(cups_path)[args.cup]
    cal_path = Path(args.repo).expanduser() / "robot" / "calibration" / ("fixture_calibration_%s.json" % args.cup)
    cal = load_calibration(cal_path)
    bores = cal.lay["calibration"]["bores"]
    bore_base = np.array(bores[args.bore], float)
    d = np.radians(args.dir_deg)
    offset_mm = args.offset_mm if args.offset_mm is not None else \
        cal.lay["calibration"]["bore_diameter"] / 2 + cup.lip_radius_mm - args.overlap_mm
    cup_base = bore_base + offset_mm * np.array([np.cos(d), np.sin(d)])
    bore_xy = cal.base_to_robot_mm(bore_base) / 1000.0
    cup_xy = cal.base_to_robot_mm(cup_base) / 1000.0
    z_top = cal.top_z_mm(cup_base) / 1000.0
    open_dir = float(np.degrees(np.arctan2(*(bore_xy - cup_xy)[::-1])))
    print("cup %s, bore %s: cup centre %.1f mm from the bore centre; bore lies at %.1f deg (robot frame)" % (
        cup.name, args.bore, offset_mm, open_dir))

    rclpy.init()
    node = rclpy.create_node("haptic_chamber_check")
    io = HapticRosIO(node, cup.chambers)
    rtde = None
    try:
        io.vacuum(False)
        rtde = rtdeHelp(125, node=node)
        rtde.setTCPoffset(cup.tcp_offset)
        time.sleep(0.2)
        io.wait_for_pressure()
        psi0 = tool_yaw_deg(list(rtde.rtde_r.getActualTCPPose()))
        hover = lambda psi: tool_pose([cup_xy[0], cup_xy[1], z_top + HOVER_MM / 1000.0], psi)
        input("Enter: lift, then move %.0f mm above the contact point (tool yaw %.1f deg, wrist 3 unwound) " % (HOVER_MM, psi0))
        current = list(rtde.rtde_r.getActualTCPPose())
        current[2] = max(current[2], z_top + TRAVEL_MM / 1000.0)
        rtde.rtde_c.moveL(current, MOVE_SPEED, MOVE_ACC)
        move_unwound(rtde, tool_pose([cup_xy[0], cup_xy[1], z_top + TRAVEL_MM / 1000.0], psi0), args.q6_home_deg)
        rtde.rtde_c.moveL(hover(psi0), MOVE_SPEED, MOVE_ACC)
        print("wrist 3 at %.1f deg; the scan turns it by about +-180 deg" % wrist3_deg(rtde))
        print("zeroing the sensors in air ...")
        io.zero_in_air()
        io.vacuum(True)
        time.sleep(1.0)
        # never a 180 deg step, whose direction would be undefined: start half a step past -180 and turn one way
        psis = psi0 + np.arange(-180.0 + args.step_deg / 2, 180.0, args.step_deg)
        rows = []
        for k, psi in enumerate(psis):
            rtde.rtde_c.moveL(tool_pose([cup_xy[0], cup_xy[1], z_top + 0.002], psi), MOVE_SPEED, MOVE_ACC)
            rtde.rtde_c.moveL(tool_pose([cup_xy[0], cup_xy[1], z_top - args.press_mm / 1000.0], psi), TOUCH_SPEED, MOVE_ACC)
            time.sleep(args.settle_s)
            p = io.mean_pressure(args.read_s)
            pose = list(rtde.rtde_r.getActualTCPPose())
            rows.append(dict(psi_cmd=float(psi), psi=tool_yaw_deg(pose), pose=pose, pressure_pa=p.tolist()))
            print("  %2d/%d  psi %7.1f  %s Pa" % (k + 1, len(psis), psi, np.array2string(p, precision=0)))
            rtde.rtde_c.moveL(tool_pose([cup_xy[0], cup_xy[1], z_top + 0.002], psi), TOUCH_SPEED * 2, MOVE_ACC)
        io.vacuum(False)
        rtde.rtde_c.moveL(hover(psis[-1]), MOVE_SPEED, MOVE_ACC)
        rtde.rtde_c.moveL(hover(psi0), MOVE_SPEED, MOVE_ACC)
    except KeyboardInterrupt:
        print("\nInterrupted: vacuum off, lifting.")
        rows = None
        if rtde is not None:
            p = list(rtde.rtde_r.getActualTCPPose())
            p[2] = max(p[2], z_top + HOVER_MM / 1000.0)
            rtde.rtde_c.moveL(p, MOVE_SPEED, MOVE_ACC)
    finally:
        io.vacuum(False)
        io.close()
        if rtde is not None:
            rtde.disconnect()
        node.destroy_node()
        rclpy.shutdown()
    if not rows:
        return

    psi = np.array([r["psi"] for r in rows])
    vac = -np.array([r["pressure_pa"] for r in rows])     # vacuum positive
    report, chamber_map, offset, rms, weak = analyze_scan(psi, vac, open_dir)

    out_dir = Path.home() / "EDG_Experiment" / datetime.now().strftime("%y%m%d")
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / ("chamber_check_%s_%s.json" % (cup.name, datetime.now().strftime("%H%M%S")))
    result = dict(cup=cup.name, bore=args.bore, offset_mm=offset_mm, dir_deg=args.dir_deg, press_mm=args.press_mm,
                  open_dir_robot_deg=open_dir, channels=report, chamber_map=chamber_map, body_offset_deg=offset,
                  fit_rms_deg=rms, scan=rows, calibration=str(cal_path))
    out.write_text(json.dumps(result, indent=1))
    print("saved %s" % out)

    if args.write:
        if weak or rms > 10:
            print("Not writing cups.json: fix the warnings first.")
            return
        data = json.loads(cups_path.read_text())
        data["cups"][cup.name].update(chamber_map=chamber_map, body_offset_deg=round(offset, 2), calibrated=True,
                                      chamber_check=str(out))
        cups_path.write_text(json.dumps(data, indent=1))
        print("updated %s" % cups_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cup", required=True, help="cup name in ais/config/cups.json (ch3, ch6)")
    parser.add_argument("--bore", default="C0")
    parser.add_argument("--dir_deg", type=float, default=45.0, help="straddle direction from the bore centre, base-plate frame")
    parser.add_argument("--overlap_mm", type=float, default=1.0, help="how far the cup lip reaches past the bore rim")
    parser.add_argument("--offset_mm", type=float, default=None, help="cup centre distance from the bore centre (overrides --overlap_mm)")
    parser.add_argument("--press_mm", type=float, default=5.0)
    parser.add_argument("--step_deg", type=float, default=15.0)
    parser.add_argument("--settle_s", type=float, default=0.5)
    parser.add_argument("--read_s", type=float, default=0.5)
    parser.add_argument("--write", action="store_true", help="store the fit in cups.json")
    parser.add_argument("--q6_home_deg", type=float, default=0.0, help="wrist 3 is unwound near this before the scan")
    parser.add_argument("--repo", default="~/projects/2D_Haptic_Search")
    parser.add_argument("--cups", default="~/ros2_ws/src/suction_cup/ais/config/cups.json")
    main(parser.parse_args())
