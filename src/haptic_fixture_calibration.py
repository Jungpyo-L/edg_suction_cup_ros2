#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Record the four fixture calibration bores (C0..C3) with the cup and save the fixture calibration for that cup.
# ROS2 port of 2D_Haptic_Search/robot/ros/calibrate_fixture.py; the solve, checks and file format stay in
# 2D_Haptic_Search/tools/fixture_calibration.py (numpy only), imported from --repo.
#
#   ros2 run suction_cup haptic_fixture_calibration.py --cup ch3 --repeats 3
#
# Vacuum stays off. For each bore, move the cup into it until the lip rests lightly on the bore floor, then press
# Enter. Moving: keyboard jog in the UR base frame with the tool orientation held fixed (default), or --freedrive
# (teach mode, by hand). The TCP is set from config/haptic_cups.json, so record with the cup that the trials use.
# Output: <repo>/robot/calibration/fixture_calibration_<cup>.json (+ time-stamped copy and raw csv).

import argparse
import csv
import json
import select
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np
import rclpy
from std_msgs.msg import Int8

from helperFunction.haptic_robot_frame import load_cups, tool_tilt_deg, tool_yaw_deg
from helperFunction.rtde_helper import rtdeHelp

JOG_KEYS = {"d": (1, 0, 0), "a": (-1, 0, 0), "w": (0, 1, 0), "s": (0, -1, 0), "r": (0, 0, 1), "f": (0, 0, -1)}
STEPS_MM = {"6": 50.0, "5": 10.0, "1": 2.0, "2": 0.5, "3": 0.1, "4": 0.02}
JOG_SPEED, JOG_ACC = 0.01, 0.1   # m/s, m/s^2: slow enough to stop on a key release
MAX_Z_DROP_M = 0.060             # refuse to jog more than 60 mm below the starting height


def read_key(timeout=0.1):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else None


def actual_pose(rtde):
    return list(rtde.rtde_r.getActualTCPPose())


def jog_to_bore(rtde, label, z_floor):
    """Keyboard jog with fixed orientation. Returns when Enter is pressed; 'q' aborts."""
    print("\n[%s] jog:  a/d -x/+x   s/w -y/+y   f/r down/up (base frame)   step 6:50 5:10 1:2 2:0.5 3:0.1 4:0.02 mm" % label)
    print("        Enter: record this pose    q: abort")
    step = 0.5
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            key = read_key()
            if key is None:
                continue
            if key in ("\n", "\r"):
                return
            if key == "q":
                raise KeyboardInterrupt
            if key in STEPS_MM:
                step = STEPS_MM[key]
                print("  step %.2f mm" % step)
                continue
            if key not in JOG_KEYS:
                continue
            pose = actual_pose(rtde)
            target = list(pose)
            for i, d in enumerate(JOG_KEYS[key]):
                target[i] += d * step / 1000.0
            if target[2] < z_floor:
                print("  refused: more than %.0f mm below the start height" % (MAX_Z_DROP_M * 1000))
                continue
            rtde.rtde_c.moveL(target, JOG_SPEED, JOG_ACC)
            p = actual_pose(rtde)
            print("  x %.2f  y %.2f  z %.2f mm" % (p[0] * 1000, p[1] * 1000, p[2] * 1000))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def predict_bore_xy(recorded, bores_mm, target):
    """Robot-frame xy (m) of a bore from the bores recorded so far (rigid fit, needs two), else None."""
    names = [n for n in recorded if n in bores_mm]
    if len(names) < 2:
        return None
    P = np.array([bores_mm[n] for n in names], float)
    Q = np.array([recorded[n][:2] for n in names], float) * 1000.0
    pc, qc = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - pc).T @ (Q - qc))
    R = (U @ Vt).T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = (U @ Vt).T
    return (R @ (np.array(bores_mm[target], float) - pc) + qc) / 1000.0


def main(args):
    sys.path.insert(0, str(Path(args.repo).expanduser()))
    from tools.fixture_calibration import DEFAULT_LAYOUT, report, save, solve
    bores_mm = json.loads(Path(DEFAULT_LAYOUT).read_text())["calibration"]["bores"]

    cup = load_cups(Path(args.cups).expanduser())[args.cup]
    out = Path(args.out) if args.out else Path(args.repo).expanduser() / "robot" / "calibration" / (
        "fixture_calibration_%s.json" % args.cup)

    rclpy.init()
    node = rclpy.create_node("haptic_fixture_calibration")
    rtde = None
    pwm_pub = node.create_publisher(Int8, "pwm", 1)
    try:
        pwm_pub.publish(Int8(data=0))
        rtde = rtdeHelp(125, node=node)
        rtde.setTCPoffset(cup.tcp_offset)
        time.sleep(0.2)
        start = actual_pose(rtde)
        print("TCP offset %s (cup %s); start pose x %.1f y %.1f z %.1f mm, tool yaw %.1f deg, tilt %.2f deg" % (
            cup.tcp_offset, cup.name, start[0] * 1000, start[1] * 1000, start[2] * 1000,
            tool_yaw_deg(start), tool_tilt_deg(start)))
        if tool_tilt_deg(start) > 1.0:
            raise RuntimeError("The tool is not pointing straight down (tilt %.2f deg). Fix it on the pendant first."
                               % tool_tilt_deg(start))
        z_floor = start[2] - MAX_Z_DROP_M

        pts, raw = {}, []
        for bore in args.bores.split(","):
            acc = []
            guess = predict_bore_xy(pts, bores_mm, bore)
            if guess is not None and not args.freedrive:
                hover = actual_pose(rtde)
                hover[2] = max(hover[2], start[2])
                answer = input("\n[%s] move above the predicted bore centre (x %.1f, y %.1f mm) at the start height? [y/N] "
                               % (bore, guess[0] * 1000, guess[1] * 1000))
                if answer.strip().lower() == "y":
                    rtde.rtde_c.moveL(hover, 0.05, 0.2)
                    rtde.rtde_c.moveL([guess[0], guess[1]] + hover[2:], 0.05, 0.2)
            for k in range(args.repeats):
                label = "%s %d/%d" % (bore, k + 1, args.repeats)
                if args.freedrive:
                    input("\n[%s] Enter to start freedrive, guide the cup into bore %s ..." % (label, bore))
                    rtde.rtde_c.teachMode()
                    input("  cup resting on the bore floor? Enter to record ")
                    rtde.rtde_c.endTeachMode()
                    time.sleep(0.2)
                else:
                    jog_to_bore(rtde, label, z_floor)
                pose = actual_pose(rtde)
                tilt, yaw = tool_tilt_deg(pose), tool_yaw_deg(pose)
                if tilt > 0.5 or abs((yaw - tool_yaw_deg(start) + 180) % 360 - 180) > 2.0:
                    print("  WARNING: orientation changed (tilt %.2f deg, yaw %.1f vs %.1f deg)" % (
                        tilt, yaw, tool_yaw_deg(start)))
                acc.append(pose[:3])
                raw.append(dict(bore=bore, x=pose[0], y=pose[1], z=pose[2], tool_yaw_deg=yaw, tilt_deg=tilt))
                print("  %s: x=%.2f y=%.2f z=%.2f mm" % (bore, pose[0] * 1000, pose[1] * 1000, pose[2] * 1000))
                if k < args.repeats - 1 or bore != args.bores.split(",")[-1]:
                    print("  lift the cup out of the bore (r key / freedrive) before moving on")
                    if not args.freedrive:
                        lifted = list(pose)
                        lifted[2] += args.lift_mm / 1000.0
                        rtde.rtde_c.moveL(lifted, JOG_SPEED * 2, JOG_ACC)
            pts[bore] = tuple(np.mean(acc, axis=0))
            if args.repeats > 1:
                spread = max(np.linalg.norm(np.array(v[:2]) - pts[bore][:2]) for v in acc) * 1000
                print("  %s: repeatability (max xy distance from mean) %.3f mm" % (bore, spread))

        d = solve(pts)
        d.update(cup=cup.name, tcp_offset=cup.tcp_offset)
        report(d)
        out, stamped = save(d, out)
        with open(stamped.with_suffix(".csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(raw[0]))
            w.writeheader()
            w.writerows(raw)
        print("saved %s (latest) and %s + raw csv" % (out, stamped))
    except KeyboardInterrupt:
        print("\nAborted; nothing saved.")
    finally:
        pwm_pub.publish(Int8(data=0))
        if rtde is not None:
            try:
                rtde.rtde_c.endTeachMode()
            except Exception:
                pass
            rtde.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cup", required=True, help="cup name in config/haptic_cups.json (ch3, ch6)")
    parser.add_argument("--repo", default="~/projects/2D_Haptic_Search")
    # the source tree, not the installed copy: measured cup values are written there without a rebuild
    parser.add_argument("--cups", default="~/ros2_ws/src/suction_cup/config/haptic_cups.json")
    parser.add_argument("--out", default=None)
    parser.add_argument("--bores", default="C0,C1,C2,C3")
    parser.add_argument("--repeats", type=int, default=3, help="insertions per bore (averaged)")
    parser.add_argument("--lift_mm", type=float, default=10.0, help="automatic lift after each recording (jog mode)")
    parser.add_argument("--freedrive", action="store_true", help="guide the cup by hand (teach mode) instead of jogging")
    main(parser.parse_args())
