#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Record the four fixture calibration bores (C0..C3) with the cup and save the fixture calibration for that cup.
# ROS2 port of 2D_Haptic_Search/robot/ros/calibrate_fixture.py; the solve, checks and file format stay in
# 2D_Haptic_Search/tools/fixture_calibration.py (numpy only), imported from --repo.
#
#   ros2 run suction_cup ais_fixture_calibration.py --cup ch3 --repeats 3
#
# Vacuum stays off. 1) For each bore, put the cup into it and press Enter: this gives x, y and the plate rotation
# (the depth in the bore does not matter). 2) Contact height: touch the top of objects --top_objects (default 1, 5,
# 10, 6) at a flat spot, lip just touching, Enter; the contact plane is fitted through these points.
# --touch lowers the cup by force sensing (--touch_n) instead of by eye. Moving: keyboard jog in the UR base frame with the tool orientation held fixed (default), or --freedrive
# (teach mode, by hand). The TCP is set from ais/config/cups.json, so record with the cup that the trials use.
# Output: <repo>/robot/calibration/fixture_calibration_<cup>.json (+ time-stamped copy and raw csv).

import sys
from pathlib import Path

# With colcon --symlink-install this file runs from ais/nodes/: put ais/ (ais_haptic) and src/ (helperFunction)
# on the path. A copied install has both next to the script already.
_NODES = Path(__file__).resolve().parent
sys.path[:0] = [str(_NODES.parent), str(_NODES.parent.parent / "src")]

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
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int8

from ais_haptic.robot_frame import load_cups, tool_tilt_deg, tool_yaw_deg
from helperFunction.keyboard_jog import jog
from helperFunction.rtde_helper import rtdeHelp

JOG_SPEED, JOG_ACC = 0.01, 0.1   # m/s, m/s^2: automatic lift out of a bore


def read_key(timeout=0.1):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else None


def actual_pose(rtde):
    return list(rtde.rtde_r.getActualTCPPose())


def jog_to_bore(rtde, label, z_floor):
    """Keyboard jog with fixed orientation (helperFunction/keyboard_jog.py: step-dependent speed, t = freedrive,
    control script restored after pendant use). Returns when Enter is pressed; q aborts."""
    jog(rtde, label, z_floor=z_floor)


class ForceReader:
    """Fz of the wrist force sensor (netft_data), read by spinning the node here (nothing else spins it)."""

    def __init__(self, node):
        self.node, self.fz, self.count = node, None, 0
        node.create_subscription(WrenchStamped, "netft_data", self._cb, 50)

    def _cb(self, msg):
        self.fz, self.count = msg.wrench.force.z, self.count + 1

    def mean(self, seconds):
        vals, n0, t0 = [], self.count, time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.count != n0:
                n0 = self.count
                vals.append(self.fz)
        if len(vals) < 5:
            raise RuntimeError("No force data on netft_data (is suction_experiment.launch.py running?). "
                               "Use --no-touch to set the height by eye instead.")
        return float(np.mean(vals)), float(np.std(vals))


def touch_down(rtde, force, args):
    """Lower the cup straight down at --touch_speed until the wrist force changes by --touch_n (the lip is on the bore
    floor), stop, and return (found, force change in N, distance moved in mm). The height no longer has to be judged
    by eye; the same force at every bore gives the same lip compression, so the fitted plane is consistent."""
    bias, noise = force.mean(0.5)
    thr = max(args.touch_n, 6 * noise)
    z0, hits, df = actual_pose(rtde)[2], 0, 0.0
    try:
        while True:
            rtde.rtde_c.speedL([0, 0, -args.touch_speed / 1000.0, 0, 0, 0], 0.05, 0.02)
            n0 = force.count
            rclpy.spin_once(force.node, timeout_sec=0.008)
            if force.count != n0:
                df = force.fz - bias
                hits = hits + 1 if abs(df) > thr else 0
            moved = (z0 - actual_pose(rtde)[2]) * 1000
            if hits >= 3:                                   # three samples in a row: not a single noisy reading
                return True, df, moved
            if moved > args.touch_max_mm:
                return False, df, moved
    finally:
        rtde.rtde_c.speedStop()
        time.sleep(0.3)


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
    from tools.fixture_calibration import DEFAULT_LAYOUT, Calibration, report, save, solve
    layout = json.loads(Path(DEFAULT_LAYOUT).read_text())
    bores_mm = layout["calibration"]["bores"]

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
        z_floor = start[2] - args.max_drop_mm / 1000.0
        force = None
        if args.touch:
            force = ForceReader(node)
            _, noise = force.mean(1.0)
            print("force sensor ok (Fz noise %.3f N); height by touch-down at %.1f N" % (noise, args.touch_n))

        pts, raw = {}, []
        wanted_tops = args.reuse_tops or [v for v in args.top_objects.split(",") if v.strip()]
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
                    input("  cup in the bore? Enter to record ")
                    rtde.rtde_c.endTeachMode()
                    time.sleep(0.2)
                else:
                    jog_to_bore(rtde, label, z_floor)
                touch = {}
                if force is not None and not wanted_tops:   # bore height only matters without object-top points
                    while True:
                        before = actual_pose(rtde)
                        found, df, moved = touch_down(rtde, force, args)
                        print("  touch-down: %s after %.2f mm (force change %+.2f N)" % (
                            "contact" if found else "NO CONTACT", moved, df))
                        if found:
                            touch = dict(touch_force_n=round(df, 3), touch_travel_mm=round(moved, 3))
                            if moved < 0.3:
                                print("  WARNING: contact almost immediately - the cup was already pressed or is rubbing "
                                      "on the bore wall; lift a little and repeat if unsure")
                            break
                        rtde.rtde_c.moveL(before, JOG_SPEED, JOG_ACC)
                        print("  no contact within %.0f mm: put the cup deeper into the bore and press Enter again"
                              % args.touch_max_mm)
                        jog_to_bore(rtde, label, z_floor)
                pose = actual_pose(rtde)
                tilt, yaw = tool_tilt_deg(pose), tool_yaw_deg(pose)
                if tilt > 0.5 or abs((yaw - tool_yaw_deg(start) + 180) % 360 - 180) > 2.0:
                    print("  WARNING: orientation changed (tilt %.2f deg, yaw %.1f vs %.1f deg)" % (
                        tilt, yaw, tool_yaw_deg(start)))
                acc.append(pose[:3])
                raw.append(dict(bore=bore, x=pose[0], y=pose[1], z=pose[2], tool_yaw_deg=yaw, tilt_deg=tilt,
                                touch_force_n=touch.get('touch_force_n', ''), touch_travel_mm=touch.get('touch_travel_mm', '')))
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

        # contact height: touch object tops (the lip is easy to see there); the bore heights are then not used
        tops = []
        wanted = [int(v) for v in args.top_objects.split(",") if v.strip()]
        if args.reuse_tops:
            tops = json.loads(out.read_text())["top_points_m"]
            if not tops:
                raise RuntimeError("%s has no object-top points to reuse" % out)
            print("\nreusing %d object-top points from %s (valid only if the plate and cup have not moved)" % (len(tops), out))
        elif wanted:
            xy_cal = Calibration(solve(pts), layout)
            by_no = {int(n): o for o, n in layout["object_numbers"].items()}
            print("\nContact height: touch the top of object(s) %s. Pick a flat spot where the whole lip is on the surface\n"
                  "(not over a hole or the edge), lower until the lip just touches, Enter." % wanted)
            for no in wanted:
                label = "object %d top" % no
                origin = xy_cal.base_to_robot_mm(layout["object_origins"][by_no[no]]) / 1000.0
                hover = actual_pose(rtde)
                hover[2] = max(hover[2], start[2])
                if input("\n[%s] move above object %d (x %.1f, y %.1f mm) at the start height? [y/N] " % (
                        label, no, origin[0] * 1000, origin[1] * 1000)).strip().lower() == "y":
                    rtde.rtde_c.moveL(hover, 0.05, 0.2)
                    rtde.rtde_c.moveL([origin[0], origin[1]] + hover[2:], 0.05, 0.2)
                jog_to_bore(rtde, label, z_floor)
                if force is not None:
                    found, df, moved = touch_down(rtde, force, args)
                    print("  touch-down: %s after %.2f mm (force change %+.2f N)" % (
                        "contact" if found else "NO CONTACT - this point is recorded as it is", moved, df))
                pose = actual_pose(rtde)
                tops.append(pose[:3])
                raw.append(dict(bore="top_obj%02d" % no, x=pose[0], y=pose[1], z=pose[2], tool_yaw_deg=tool_yaw_deg(pose),
                                tilt_deg=tool_tilt_deg(pose), touch_force_n="", touch_travel_mm=""))
                print("  %s: x=%.2f y=%.2f z=%.2f mm" % (label, pose[0] * 1000, pose[1] * 1000, pose[2] * 1000))
                lifted = list(pose)
                lifted[2] += args.lift_mm / 1000.0
                rtde.rtde_c.moveL(lifted, JOG_SPEED * 2, JOG_ACC)

        d = solve(pts, top_points_m=tops or None)
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
    parser.add_argument("--cup", required=True, help="cup name in ais/config/cups.json (ch3, ch6)")
    parser.add_argument("--repo", default="~/projects/2D_Haptic_Search")
    # the source tree, not the installed copy: measured cup values are written there without a rebuild
    parser.add_argument("--cups", default="~/ros2_ws/src/suction_cup/ais/config/cups.json")
    parser.add_argument("--out", default=None)
    parser.add_argument("--bores", default="C0,C1,C2,C3")
    parser.add_argument("--repeats", type=int, default=3, help="insertions per bore (averaged)")
    parser.add_argument("--lift_mm", type=float, default=10.0, help="automatic lift after each recording (jog mode)")
    parser.add_argument("--top_objects", default="1,5,10,6",
                        help="object numbers whose top is touched for the contact height (3 or more spread out: plane; "
                             "1-2: height only, slope from the bores; empty: use the bore floors)")
    parser.add_argument("--reuse_tops", action="store_true",
                        help="redo the bores only: keep the object-top points of the existing calibration file")
    parser.add_argument("--touch", action="store_true",
                        help="after Enter, lower the cup at --touch_speed until the force sensor feels contact, and "
                             "record there (object tops; bore floors when --top_objects is empty)")
    parser.add_argument("--touch_n", type=float, default=1.0, help="force change that counts as contact (N)")
    parser.add_argument("--touch_speed", type=float, default=1.0, help="touch-down speed (mm/s)")
    parser.add_argument("--touch_max_mm", type=float, default=10.0, help="give up after this much descent")
    parser.add_argument("--max_drop_mm", type=float, default=500.0,
                        help="jog refuses to go further than this below the height at start")
    parser.add_argument("--freedrive", action="store_true", help="guide the cup by hand (teach mode) instead of jogging")
    main(parser.parse_args())
