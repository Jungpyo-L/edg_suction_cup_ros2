#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Seal check for the lateral characterization setup: press the cup at a few x offsets / yaws, vacuum on, and print the
# steady chamber vacuum. Tells a leaking object, a leaking cup or a weak vacuum supply apart.
#
#   ros2 run suction_cup seal_check.py --ch 3 --corner 270 --x -10 -15 --yaw 0 90 180
#   ros2 run suction_cup seal_check.py --ch 3 --corner 180 --x -10
#   ros2 run suction_cup seal_check.py --ch 3 --corner 180 --pose_x 0.60 --pose_y -0.30   # any flat area (z: --pose_z)
#
# Same start point, orientation, press depth and zeroing as JP_characterization_lateral_positioning.py (recorded
# location of --corner unless --pose_* is given). Needs ESP32_Pressure, ESP32_PWM and netft running (the launch file).
# A sealed cup on a flat surface reads about 64.4 kPa in every chamber.

import argparse
import copy
import time
from math import pi

import numpy as np
import rclpy
from std_msgs.msg import Int8

import JP_characterization_lateral_positioning as lateral
from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp

APPROACH_MM = 30.0


def spin_for(node, seconds, p_help=None):
    """Spin for `seconds`; returns (times since start, raw pressures) of every new sample."""
    t0 = time.time()
    times, raw, last = [], [], None
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.01)
        if p_help is not None and p_help.thisPres is not None and p_help.thisPres is not last:
            last = p_help.thisPres
            times.append(time.time() - t0)
            raw.append(np.array(last, dtype=float))
    return np.array(times), np.array(raw)


def main(args):
    tcp_offset, start = lateral.resolve_hardware(args)
    default_yaw = lateral.default_yaw_for_channels(args.ch) - np.radians(args.axis_deg)   # turn with the bisector
    rclpy.init()
    node = rclpy.create_node("seal_check")
    pwm = node.create_publisher(Int8, "pwm", 1)
    rtde_help = None
    rows = []
    try:
        ft_help = FT_CallbackHelp(node)
        p_help = P_CallbackHelp(node)
        rtde_help = rtdeHelp(125, node=node)
        rtde_help.setTCPoffset(tcp_offset)
        pwm.publish(Int8(data=0))
        spin_for(node, 1.0)
        p_help.startSampling()
        spin_for(node, 0.5)
        print("start point %s mm, press %.1f mm, TCP %s" % (np.round(np.array(start) * 1000, 2), args.deformation, tcp_offset))

        current = list(rtde_help.rtde_r.getActualTCPPose())
        travel_z = max(current[2], start[2] + APPROACH_MM / 1000.0)
        rtde_help.rtde_c.moveL(current[:2] + [travel_z] + current[3:], 0.05, 0.2)
        input("Enter: move above the start point and run %d checks " % (len(args.x) * len(args.yaw)))
        for x in args.x:
            for yaw in args.yaw:
                orientation = lateral.yaw_quat(default_yaw - yaw * pi / 180)
                contact = copy.deepcopy(list(start))
                lateral.shift_along_axis(contact, x, args.axis_deg)    # along the recorded bisector, as the sweep
                above = contact[:2] + [travel_z]
                press = contact[:2] + [start[2] - args.deformation / 1000.0]
                rtde_help.goToPose(rtde_help.getPoseObj(above, orientation))
                rtde_help.goToPose(rtde_help.getPoseObj(contact, orientation))
                ft_help.setNowAsBias()
                lateral.zero_pressure(node, p_help)                       # vacuum off, lip touching
                rtde_help.goToPose(rtde_help.getPoseObj(press, orientation), speed=0.02, acc=0.1)
                pwm.publish(Int8(data=100))
                times, raw = spin_for(node, args.hold_s, p_help)
                pwm.publish(Int8(data=0))
                vac_t = (p_help.PressureOffset - raw) / 1000.0           # kPa, every sample since vacuum on
                vacuum = vac_t[times > times[-1] - 0.5].mean(axis=0)     # last 0.5 s
                marks = [t for t in (0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0) if t <= args.hold_s]
                timeline = "  ".join("%gs:%.2f" % (t, vac_t[np.argmin(abs(times - t))].mean()) for t in marks)
                fz = getattr(ft_help, "averageFz_noOffset", float("nan"))
                z = list(rtde_help.rtde_r.getActualTCPPose())[2]
                rows.append((x, yaw, vacuum, fz, z))
                print("x %+5.1f mm  theta %5.1f deg : vacuum %s kPa (last 0.5 s)   Fz %6.2f N   tcp z %.2f mm" % (
                    x, yaw, np.round(vacuum, 2), fz, z * 1000))
                print("      mean vacuum over time (kPa): %s" % timeline)
                spin_for(node, 0.3)
                rtde_help.goToPose(rtde_help.getPoseObj(above, orientation))
        p_help.stopSampling()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        pwm.publish(Int8(data=0))
        if rtde_help is not None:
            try:
                p = list(rtde_help.rtde_r.getActualTCPPose())
                p[2] = max(p[2], start[2] + APPROACH_MM / 1000.0)
                rtde_help.rtde_c.moveL(p, 0.05, 0.2)
            except Exception as exc:
                print("could not lift: %s" % exc)
            rtde_help.disconnect()
        node.destroy_node()
        rclpy.shutdown()
    if rows:
        best = max(rows, key=lambda r: r[2].min())
        print("\nhighest all-chamber vacuum: %.2f kPa at x %+.1f, theta %.1f (sealed flat surface: ~64.4)" % (
            best[2].min(), best[0], best[1]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ch", type=int, default=3)
    parser.add_argument("--corner", type=int, default=270)
    parser.add_argument("--setup", default=lateral.ACTIVE_SETUP)
    parser.add_argument("--x", type=float, nargs="+", default=[-10.0], help="x offsets (mm) from the start point")
    parser.add_argument("--yaw", type=float, nargs="+", default=[0.0], help="theta (deg) as in the sweep")
    parser.add_argument("--deformation", type=float, default=5.0, help="press depth below the contact point (mm)")
    parser.add_argument("--hold_s", type=float, default=5.0, help="vacuum-on time per check (s)")
    for k in ("tcp_x", "tcp_y", "tcp_z", "pose_x", "pose_y", "pose_z"):
        parser.add_argument("--" + k, type=float, default=None)
    main(parser.parse_args())
