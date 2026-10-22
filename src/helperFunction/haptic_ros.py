#!/usr/bin/env python3
"""ROS2 side of the haptic-search robot demo: pressure, vacuum, sync and data-logger handles on one node that is
spun in a background thread, so a control loop can read the latest pressure without spinning by hand.
"""
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int8

from suction_cup.msg import SensorPacket

from .SuctionP_callback_helper import P_CallbackHelp

VACUUM_ON, VACUUM_OFF = 100, 0
SYNC_RESET, SYNC_START, SYNC_STOP = 0, 1, 2


class HapticRosIO:
    def __init__(self, node, chambers):
        self.node, self.chambers = node, chambers
        self.p_help = P_CallbackHelp(node)
        self.pwm_pub = node.create_publisher(Int8, "pwm", 1)
        self.sync_pub = node.create_publisher(Int8, "sync", 1)
        self.raw_lock = threading.Lock()
        self.raw = None          # list of (receive time, header stamp, values) while recording
        node.create_subscription(SensorPacket, "SensorPacket", self._raw_callback, 100)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)
        self._stop = False
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        while not self._stop and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.01)

    def _raw_callback(self, msg):
        with self.raw_lock:
            if self.raw is not None:
                self.raw.append((time.monotonic(), msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec, list(msg.data)))

    def start_raw_recording(self):
        with self.raw_lock:
            self.raw = []

    def stop_raw_recording(self):
        """Returns (receive time s (monotonic), header stamp s, raw values Pa) since start_raw_recording()."""
        with self.raw_lock:
            raw, self.raw = self.raw or [], None
        if not raw:
            return np.zeros(0), np.zeros(0), np.zeros((0, self.chambers))
        t, s, v = zip(*raw)
        return np.array(t), np.array(s), np.array(v, dtype=float)

    def vacuum(self, on):
        self.pwm_pub.publish(Int8(data=VACUUM_ON if on else VACUUM_OFF))

    def sync(self, code):
        self.sync_pub.publish(Int8(data=int(code)))

    def wait_for_pressure(self, timeout_s=5.0):
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            if self.p_help.four_pressure is not None:
                if len(self.p_help.four_pressure) != self.chambers:
                    raise RuntimeError("Pressure stream has %d channels, the cup has %d"
                                       % (len(self.p_help.four_pressure), self.chambers))
                return
            time.sleep(0.05)
        raise RuntimeError("No /SensorPacket within %.0f s. Is ESP32_Pressure.py running?" % timeout_s)

    def zero_in_air(self):
        """Zero the sensors; only call with the cup in air (handoff: never zero on a surface)."""
        self.p_help.setNowAsOffset()

    def pressure(self):
        """Latest offset-corrected 7-point average, Pa, vacuum negative, channel order."""
        p = self.p_help.four_pressure
        return None if p is None else np.array(p, dtype=float)

    def mean_pressure(self, seconds):
        samples, t_end = [], time.monotonic() + seconds
        while time.monotonic() < t_end:
            samples.append(self.pressure())
            time.sleep(0.005)
        return np.mean(samples, axis=0)

    def close(self):
        self._stop = True
        self.thread.join(timeout=1.0)
        self.p_help.shutdown()
