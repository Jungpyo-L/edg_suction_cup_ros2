#!/usr/bin/env python3

# Live view of the ATI FT sensor. Subscribes to netft_data and plots the raw
# wrench so the sensor can be verified by hand before any experiment relies on
# it. Also reports the idle noise band on Fz, which is the number that sets the
# contact-detection threshold for a force-guided descent.

import argparse
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node

from helperFunction.FT_callback_helper import FT_CallbackHelp


class FTMonitor(Node):
    def __init__(self, window_sec):
        super().__init__("ft_monitor")
        self.window_sec = window_sec
        self.lock = threading.Lock()
        self.t = deque()
        self.forces = deque()
        self.torques = deque()
        self.count = 0
        self.t0 = time.time()
        self.force_offset = np.zeros(3)
        self.torque_offset = np.zeros(3)

        # Raw subscription, deliberately unaveraged: a hand press should show up
        # with no smoothing so the true noise band is visible.
        self.create_subscription(WrenchStamped, "netft_data", self.callback, 10)
        # The helper the experiments actually use. Constructing it here means
        # this script exercises that path too, not just the bare topic.
        self.ft_help = FT_CallbackHelp(self)

    def callback(self, msg):
        now = time.time() - self.t0
        f = msg.wrench.force
        tq = msg.wrench.torque
        with self.lock:
            self.count += 1
            self.t.append(now)
            self.forces.append((f.x, f.y, f.z))
            self.torques.append((tq.x, tq.y, tq.z))
            while self.t and now - self.t[0] > self.window_sec:
                self.t.popleft()
                self.forces.popleft()
                self.torques.popleft()

    def snapshot(self):
        """Return (t, forces, torques) as arrays with the current bias removed."""
        with self.lock:
            t = np.asarray(self.t)
            forces = np.asarray(self.forces).reshape(-1, 3)
            torques = np.asarray(self.torques).reshape(-1, 3)
        return t, forces - self.force_offset, torques - self.torque_offset

    def zero(self):
        """Zero the plot against the current window, and bias the helper too."""
        _, forces, torques = self.snapshot()
        if forces.shape[0] == 0:
            return
        self.force_offset = self.force_offset + forces.mean(axis=0)
        self.torque_offset = self.torque_offset + torques.mean(axis=0)
        try:
            self.ft_help.setNowAsBias()
        except AttributeError:
            # setNowAsBias reads averageTx/Ty/Tz, which only exist after the
            # helper's buffer has filled. Harmless here; the plot still zeroes.
            pass


def wait_for_data(node, timeout_sec=10.0):
    deadline = time.time() + timeout_sec
    while node.count == 0:
        if time.time() > deadline:
            raise RuntimeError(
                "No messages on /netft_data after %.0f s. Check that netft_node "
                "is running and that the ATI sensor IP is reachable." % timeout_sec
            )
        time.sleep(0.1)


def run_text(node, rate_hz):
    """Terminal readout, for when there is no display available."""
    period = 1.0 / rate_hz
    print("Fx, Fy, Fz in N; p2p is the peak-to-peak of Fz over the window.")
    while rclpy.ok():
        t, forces, _ = node.snapshot()
        if forces.shape[0]:
            fz = forces[:, 2]
            print(
                "\rFx=%+7.3f  Fy=%+7.3f  Fz=%+7.3f   Fz p2p=%6.3f  N=%d   "
                % (forces[-1, 0], forces[-1, 1], forces[-1, 2], fz.max() - fz.min(),
                   forces.shape[0]),
                end="",
                flush=True,
            )
        time.sleep(period)


def run_plot(node, window_sec, force_limit, torque_limit):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, (ax_f, ax_t) = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
    fig.canvas.manager.set_window_title("ATI FT monitor - netft_data")

    force_lines = [ax_f.plot([], [], label=name)[0] for name in ("Fx", "Fy", "Fz")]
    torque_lines = [ax_t.plot([], [], label=name)[0] for name in ("Tx", "Ty", "Tz")]

    ax_f.set_ylabel("force (N)")
    ax_t.set_ylabel("torque (Nm)")
    ax_t.set_xlabel("time (s)")
    for ax in (ax_f, ax_t):
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left")
        ax.axhline(0.0, color="k", lw=0.8, alpha=0.5)

    readout = ax_f.text(
        0.99, 0.03, "", transform=ax_f.transAxes, ha="right", va="bottom",
        family="monospace", fontsize=9,
    )
    fig.suptitle("press 'b' to zero, 'q' to quit - push on the sensor by hand")

    def on_key(event):
        if event.key == "b":
            node.zero()

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        t, forces, torques = node.snapshot()
        if t.shape[0] == 0:
            return force_lines + torque_lines + [readout]

        for i, line in enumerate(force_lines):
            line.set_data(t, forces[:, i])
        for i, line in enumerate(torque_lines):
            line.set_data(t, torques[:, i])

        ax_f.set_xlim(max(0.0, t[-1] - window_sec), max(window_sec, t[-1]))
        # Autoscale to the data but never below the limit floor, so small noise
        # is not blown up to look like a large signal.
        f_span = max(force_limit, np.abs(forces).max() * 1.2)
        t_span = max(torque_limit, np.abs(torques).max() * 1.2)
        ax_f.set_ylim(-f_span, f_span)
        ax_t.set_ylim(-t_span, t_span)

        fz = forces[:, 2]
        readout.set_text(
            "Fz = %+7.3f N\nFz p2p = %6.3f N   (over %.0f s)\nrate = %5.1f Hz"
            % (fz[-1], fz.max() - fz.min(), window_sec,
               t.shape[0] / max(t[-1] - t[0], 1e-6))
        )
        return force_lines + torque_lines + [readout]

    # cache_frame_data=False keeps FuncAnimation from retaining every frame of a
    # stream that never ends.
    _anim = FuncAnimation(fig, update, interval=50, blit=False,
                          cache_frame_data=False)
    plt.show()


def main(args):
    rclpy.init()
    node = FTMonitor(args.window)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        wait_for_data(node, args.timeout)
        print("Receiving /netft_data.")
        if args.no_plot:
            run_text(node, args.rate)
        else:
            run_plot(node, args.window, args.force_limit, args.torque_limit)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--window", type=float, default=10.0,
                        help="seconds of history shown")
    parser.add_argument("--force-limit", type=float, default=2.0,
                        help="minimum half-height of the force axis (N)")
    parser.add_argument("--torque-limit", type=float, default=0.2,
                        help="minimum half-height of the torque axis (Nm)")
    parser.add_argument("--no-plot", action="store_true",
                        help="print to the terminal instead of plotting")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="terminal refresh rate in --no-plot mode (Hz)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="seconds to wait for the first message")
    main(parser.parse_args())
