#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Measure the pressure stream that the haptic-search controllers will see, and compare it with the simulator's
# sensor model (144 Hz raw samples, sigma 6 Pa, 7-point moving average). No robot motion; run with the cup in air,
# vacuum on or off (--note says which), while ESP32_Pressure.py is running:
#   ros2 run suction_cup haptic_pressure_check.py --seconds 20 --ch 3 --note "air, vacuum on"
# Saves the raw samples to ~/EDG_Experiment/<yymmdd>/pressure_check_<time>.npz.

import argparse
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy

from suction_cup.msg import SensorPacket

SIM_RATE_HZ = 144.0
SIM_SIGMA_PA = 6.0
MA_POINTS = 7


def main(args):
    rclpy.init()
    node = rclpy.create_node("haptic_pressure_check")
    stamps, wall, values = [], [], []

    def callback(msg):
        wall.append(time.monotonic())
        stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        values.append(list(msg.data))

    node.create_subscription(SensorPacket, "SensorPacket", callback, 100)
    try:
        print("Waiting for /SensorPacket ...")
        while not values and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
        values.clear(); stamps.clear(); wall.clear()
        print("Recording %.0f s" % args.seconds)
        t_end = time.monotonic() + args.seconds
        while time.monotonic() < t_end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    widths = {len(v) for v in values}
    if len(widths) != 1:
        raise SystemExit("Channel count changed during the recording: %s" % sorted(widths))
    P = np.array(values, dtype=float)
    n_ch = P.shape[1]
    if args.ch is not None and n_ch != args.ch:
        print("WARNING: received %d channels, expected %d" % (n_ch, args.ch))
    w, s = np.array(wall), np.array(stamps)
    rate = (len(w) - 1) / (w[-1] - w[0])
    dw = np.diff(w) * 1000
    ds = np.diff(s) * 1000
    # 7-point moving average as P_CallbackHelp does, per message
    kernel = np.ones(MA_POINTS) / MA_POINTS
    filt = np.array([np.convolve(P[:, i], kernel, mode="valid") for i in range(n_ch)]).T

    print("\n========== pressure stream ==========")
    print("samples: %d over %.2f s, %d channels" % (len(P), w[-1] - w[0], n_ch))
    print("receive rate: %.1f Hz (simulator %.0f Hz, P_CallbackHelp assumes 166 Hz)" % (rate, SIM_RATE_HZ))
    print("inter-arrival ms (receive): median %.2f, p5 %.2f, p95 %.2f, max %.2f" % (
        np.median(dw), np.percentile(dw, 5), np.percentile(dw, 95), dw.max()))
    print("inter-arrival ms (header stamp): median %.2f, zero gaps %.0f %%" % (np.median(ds), 100 * np.mean(ds == 0)))
    if abs(rate - 100) < 3:
        print("NOTE: ~100 Hz is the old one-line-per-10-ms reader limit; check that the drained reader is running.")
    print("7-point average spans %.1f ms at this rate (simulator %.1f ms)" % (
        1000 * (MA_POINTS - 1) / rate, 1000 * (MA_POINTS - 1) / SIM_RATE_HZ))
    print("\nper channel (Pa): mean, raw std (sim %.1f), 7-pt std (sim %.2f)" % (SIM_SIGMA_PA, SIM_SIGMA_PA / np.sqrt(MA_POINTS)))
    for i in range(n_ch):
        print("  ch%d: %10.1f  %6.2f  %6.2f" % (i, P[:, i].mean(), P[:, i].std(), filt[:, i].std()))

    out_dir = Path.home() / "EDG_Experiment" / datetime.now().strftime("%y%m%d")
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / ("pressure_check_%s.npz" % datetime.now().strftime("%H%M%S"))
    np.savez(out, pressure_pa=P, receive_time_s=w, header_stamp_s=s, note=args.note, rate_hz=rate)
    print("\nsaved %s" % out)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--ch", type=int, default=None, help="expected channel count")
    parser.add_argument("--note", type=str, default="", help="conditions, e.g. 'air, vacuum on'")
    main(parser.parse_args())
