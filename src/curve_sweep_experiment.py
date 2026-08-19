#!/usr/bin/env python3

# Sweeps the suction cup along a curved test surface of known equation, so the
# curvature and surface tilt at every probe point are known analytically rather
# than estimated from data.
#
# The profile varies along one axis only (a curved extrusion, not a dome), and
# the tool keeps a fixed orientation throughout - so tilt is a recorded feature,
# not something compensated for. That is deliberate: it is what a gripper
# without pose compensation actually experiences.
#
# Registration is by apex: the detector's highest point is taken as the profile
# vertex, and --axis says which base-frame direction the profile runs along.
# Mount the part accordingly.

import argparse
import csv
import math
import os
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int8

from suction_cup.srv import Enable
from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.ros2_helpers import call_enable_service
from helperFunction.rtde_helper import rtdeHelp

from scipy.spatial.transform import Rotation as R

# Same fixed tool rotation and parked pose as sphere_sweep_experiment.py.
ROTVEC_DEFAULT = [2.263, -2.179, 0.000]  # rad
HOME_POSITION = [0.558369, -0.051576, 0.404641]
HOME_QUAT = [-0.722349, 0.691528, 0.000403, 0.001165]

# Phase codes on /sync, identical to sphere_sweep_experiment.py so one analysis
# path reads both experiments.
EVENT_TRAVEL = 0
EVENT_DESCEND = 1
EVENT_CONTACT = 2
EVENT_PRELOAD = 3
EVENT_DWELL_END = 4
EVENT_DESCEND_STEP = 5
EVENT_PRELOAD_STEP = 6
MAX_WAYPOINTS = 12

APEX_BUFFER = 5000


class Profile:
    """A curve z(s) with its first two derivatives, vertex at s = 0.

    s is the signed distance from the vertex along the sweep axis, and z is
    height relative to the vertex, so z(0) = 0 and z'(0) = 0.
    """

    def __init__(self, name, a, b):
        self.name = name
        self.a = a
        self.b = b

    def z(self, s):
        if self.name == "parabola":
            return -self.a * s * s
        u = self._u(s)
        return self.b * (math.sqrt(u) - 1.0)

    def dz(self, s):
        if self.name == "parabola":
            return -2.0 * self.a * s
        u = self._u(s)
        return -self.b * s / (self.a * self.a * math.sqrt(u))

    def ddz(self, s):
        if self.name == "parabola":
            return -2.0 * self.a
        u = self._u(s)
        return -self.b / (self.a * self.a * u ** 1.5)

    def _u(self, s):
        u = 1.0 - (s / self.a) ** 2
        if u <= 1e-9:
            raise ValueError(
                "Offset %.4f m reaches the semi-axis a=%.4f m, where the ellipse "
                "turns vertical. Reduce --extent." % (abs(s), self.a)
            )
        return u

    def curvature(self, s):
        """Signed-magnitude curvature, 1/m. Equals 1/R for a circle of radius R."""
        return abs(self.ddz(s)) / (1.0 + self.dz(s) ** 2) ** 1.5

    def tilt_deg(self, s):
        """Angle between the surface normal and vertical, degrees."""
        return math.degrees(math.atan(abs(self.dz(s))))


def build_profile(args):
    if args.profile == "parabola":
        if args.a <= 0.0:
            raise ValueError("--a must be positive for a parabola (z = -a*s^2)")
        return Profile("parabola", args.a, 0.0)
    if args.a <= 0.0 or args.b <= 0.0:
        raise ValueError("--a and --b must both be positive for an ellipse")
    return Profile("ellipse", args.a, args.b)


def offsets_by_spacing(profile, args):
    """Signed offsets from the vertex, either evenly in s or evenly in curvature.

    Even-s spacing clusters samples where curvature barely changes. Even-curvature
    spacing gives uniform coverage of the variable actually under study, which is
    usually what you want.
    """
    half = (args.num_points - 1) // 2
    if half == 0:
        return [0.0]

    if args.spacing == "x":
        step = args.extent / half
        magnitudes = [step * i for i in range(1, half + 1)]
    else:
        # Curvature is monotonic in |s| for both profiles, so invert it by
        # interpolating a dense table rather than solving analytically.
        grid = np.linspace(0.0, args.extent, 2001)
        kappa = np.array([profile.curvature(s) for s in grid])
        order = np.argsort(kappa)
        targets = np.linspace(kappa[0], kappa[-1], half + 1)[1:]
        magnitudes = list(np.interp(targets, kappa[order], grid[order]))

    offsets = [0.0]
    for magnitude in magnitudes:
        offsets.extend([-magnitude, magnitude])
    return offsets


class ApexListener:
    """Collects sphere_apex messages so the sweep can start from a settled estimate."""

    def __init__(self, node, topic="sphere_apex"):
        self.node = node
        self.samples = deque(maxlen=APEX_BUFFER)
        self.frame_id = "?"
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = node.create_subscription(PointStamped, topic, self.callback, qos)

    def callback(self, msg):
        self.frame_id = msg.header.frame_id
        self.samples.append([msg.point.x, msg.point.y, msg.point.z])

    def wait_for_apex(self, num_samples=20, timeout_sec=20.0):
        self.samples.clear()
        deadline = time.time() + timeout_sec
        while len(self.samples) < num_samples:
            if time.time() > deadline:
                raise RuntimeError(
                    "Only received %d sphere_apex messages in %.0f s. Check that "
                    "realsense_sphere_detector.py is running and that the crop box "
                    "contains the curved part." % (len(self.samples), timeout_sec)
                )
            rclpy.spin_once(self.node, timeout_sec=0.1)

        samples = np.asarray(self.samples)
        spread = samples.max(axis=0) - samples.min(axis=0)
        self.node.get_logger().info("Apex sample spread (m): %s" % np.round(spread, 4))
        return samples.mean(axis=0)


def read_avg_fz(node, ft_help, n_spins=5, timeout_sec=0.05):
    """Spin briefly so the FT callback runs, then return |Fz| (bias-subtracted, N)."""
    for _ in range(n_spins):
        rclpy.spin_once(node, timeout_sec=timeout_sec)
    return abs(getattr(ft_help, "averageFz_noOffset", 0.0))


def bias_ft_sensor(node, ft_help, n_spins=60, timeout_sec=0.05):
    """Fill the FT averaging buffer off the surface, then zero the sensor."""
    for _ in range(n_spins):
        rclpy.spin_once(node, timeout_sec=timeout_sec)
        if getattr(ft_help, "startAverage", False):
            break
    if not getattr(ft_help, "startAverage", False):
        raise RuntimeError(
            "No /netft_data received - cannot bias the FT sensor. Check that "
            "netft_node is running and the ATI sensor IP is reachable."
        )
    ft_help.setNowAsBias()


def descend_to_contact(node, rtde_help, ft_help, xy, z_start, orientation, args,
                       on_event=None):
    """Step down until |Fz| crosses the contact threshold, then press preload_depth.

    Same routine as the sphere sweep: the surface height is measured by touch, so
    the profile equation only has to be good enough to place the hover pose.
    """
    x, y = xy

    def move_to(z):
        rtde_help.goToPose(
            rtde_help.getPoseObj([x, y, z], orientation),
            speed=args.descend_speed,
            acc=args.descend_acc,
        )
        time.sleep(args.settle)
        return read_avg_fz(node, ft_help)

    z = z_start
    fz = read_avg_fz(node, ft_help)
    z_contact = None
    while (z_start - z) < args.max_search:
        if fz >= args.contact_force:
            z_contact = z
            break
        z -= args.descend_step
        fz = move_to(z)
        if on_event is not None:
            on_event(EVENT_DESCEND_STEP)
    if z_contact is None and fz >= args.contact_force:
        z_contact = z
    if z_contact is None:
        raise RuntimeError(
            "Searched %.1f mm below the hover pose without reaching the %.2f N "
            "contact threshold (|Fz|=%.2f N at z=%.4f). The profile or its "
            "registration is wrong, or the threshold is below the noise floor."
            % (args.max_search * 1e3, args.contact_force, fz, z)
        )
    if on_event is not None:
        on_event(EVENT_CONTACT)
    node.get_logger().info("contact at z=%.4f m, |Fz|=%.2f N" % (z_contact, fz))

    target = z_contact - args.preload_depth
    while z > target + 1e-9:
        z = max(target, z - args.descend_step)
        fz = move_to(z)
        if on_event is not None:
            on_event(EVENT_PRELOAD_STEP)
        if fz >= args.force_ceiling:
            node.get_logger().warn(
                "Force ceiling %.1f N reached at z=%.4f m, %.1f mm into a %.1f mm "
                "preload; stopping here."
                % (args.force_ceiling, z, (z_contact - z) * 1e3,
                   args.preload_depth * 1e3)
            )
            break
    if on_event is not None:
        on_event(EVENT_PRELOAD)
    return z_contact, z, fz


def validate_args(args):
    """Reject values that would make the descent loops non-terminating."""
    positive = {
        "--descend-step": args.descend_step,
        "--max-search": args.max_search,
        "--contact-force": args.contact_force,
        "--descend-speed": args.descend_speed,
        "--descend-acc": args.descend_acc,
        "--apex-samples": args.apex_samples,
        "--apex-timeout": args.apex_timeout,
        "--extent": args.extent,
        "--num-points": args.num_points,
    }
    for name, value in positive.items():
        if value <= 0:
            raise ValueError("%s must be greater than zero, got %s" % (name, value))

    non_negative = {
        "--preload-depth": args.preload_depth,
        "--hover-height": args.hover_height,
        "--dwell": args.dwell,
        "--settle": args.settle,
        "--travel-height": args.travel_height,
    }
    for name, value in non_negative.items():
        if value < 0:
            raise ValueError("%s cannot be negative, got %s" % (name, value))

    if args.num_points % 2 == 0:
        raise ValueError(
            "--num-points must be odd so the vertex itself is probed, got %d"
            % args.num_points
        )
    if args.num_points > MAX_WAYPOINTS:
        raise ValueError(
            "%d waypoints exceeds the %d that fit in the Int8 /sync phase code."
            % (args.num_points, MAX_WAYPOINTS)
        )
    if args.force_ceiling <= args.contact_force:
        raise ValueError(
            "--force-ceiling (%.2f N) must exceed --contact-force (%.2f N)."
            % (args.force_ceiling, args.contact_force)
        )
    if args.travel_height <= args.hover_height:
        raise ValueError(
            "--travel-height (%.3f m) must exceed --hover-height (%.3f m)."
            % (args.travel_height, args.hover_height)
        )


def write_geometry(directory, args, rows):
    """Record each waypoint's analytic geometry, keyed to its /sync index.

    These are the features for later analysis: the pressure and force traces say
    what happened, this says where on the curve it happened.
    """
    stamp = datetime.now().strftime("%y%m%d_%H%M%S")
    path = os.path.join(directory, "curve_sweep_geometry_%s.csv" % stamp)
    fields = ["waypoint", "offset_s", "profile", "a", "b", "surface_dz",
              "curvature_1_per_m", "radius_equiv_m", "tilt_deg", "x", "y"]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


def wait_for_data_logger(node, client):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the data_logging service...")


def main(args):
    np.set_printoptions(precision=4)
    validate_args(args)
    profile = build_profile(args)
    offsets = offsets_by_spacing(profile, args)

    print("Profile %s (a=%.4f, b=%.4f), %d points over +/-%.3f m, %s spacing."
          % (profile.name, args.a, args.b, len(offsets), args.extent, args.spacing))
    print("Waypoint geometry:")
    for index, s in enumerate(offsets, start=1):
        kappa = profile.curvature(s)
        print("  %2d  s=%+.4f m  dz=%+.4f m  kappa=%7.2f 1/m  (R=%.4f m)  tilt=%5.1f deg"
              % (index, s, profile.z(s), kappa, 1.0 / kappa if kappa > 0 else float("inf"),
                 profile.tilt_deg(s)))

    rclpy.init()
    node = rclpy.create_node("curve_sweep_experiment")
    try:
        apex_listener = ApexListener(node, args.apex_topic)
        ft_help = file_help = rtde_help = None
        sync_pub = data_logger_client = None

        # A dry run needs neither the robot nor the logger, and skipping rtdeHelp
        # avoids the slow RTDEControlInterface handshake.
        if not args.dry_run:
            ft_help = FT_CallbackHelp(node)
            time.sleep(0.5)
            file_help = fileSaveHelp()
            time.sleep(0.5)
            rtde_help = rtdeHelp(125, node=node)
            time.sleep(0.5)

            sync_pub = node.create_publisher(Int8, "sync", 1)
            data_logger_client = node.create_client(Enable, "data_logging")
            wait_for_data_logger(node, data_logger_client)
            call_enable_service(node, data_logger_client, False)
            time.sleep(1)
            file_help.clearTmpFolder()

        print("Waiting for a settled apex estimate...")
        apex = apex_listener.wait_for_apex(args.apex_samples, args.apex_timeout)
        apex_frame = apex_listener.frame_id
        print("Vertex (%s): x=%.5f y=%.5f z=%.5f" % ((apex_frame,) + tuple(apex)))

        orientation_fixed = R.from_rotvec(ROTVEC_DEFAULT).as_quat()
        axis = np.array([1.0, 0.0, 0.0]) if args.axis == "x" else np.array([0.0, 1.0, 0.0])

        waypoints = []
        geometry = []
        for index, s in enumerate(offsets, start=1):
            # Predicted surface point: along the axis by s, down by the profile.
            point = apex + axis * s + np.array([0.0, 0.0, profile.z(s)])
            waypoints.append(point)
            kappa = profile.curvature(s)
            geometry.append({
                "waypoint": index,
                "offset_s": "%.5f" % s,
                "profile": profile.name,
                "a": args.a,
                "b": args.b,
                "surface_dz": "%.5f" % profile.z(s),
                "curvature_1_per_m": "%.4f" % kappa,
                "radius_equiv_m": "%.5f" % (1.0 / kappa) if kappa > 0 else "inf",
                "tilt_deg": "%.3f" % profile.tilt_deg(s),
                "x": "%.5f" % point[0],
                "y": "%.5f" % point[1],
            })

        print("Predicted surface points (%s); actual touch z is found by the FT "
              "sensor:" % apex_frame)
        for index, point in enumerate(waypoints, start=1):
            print("  %2d -> %s" % (index, np.round(point, 5)))
        if args.dry_run:
            print("Dry run: no robot motion commanded.")
            return

        geometry_path = write_geometry(file_help.ResultSavingDirectory, args, geometry)
        print("Waypoint geometry written to %s" % geometry_path)

        logging_response = call_enable_service(node, data_logger_client, True)
        if not logging_response.output_file_name.strip():
            raise RuntimeError(
                "Data logger did not create any CSV files. Check that topics in "
                "config/TopicsList.txt are currently published before recording."
            )

        travel_z = apex[2] + args.travel_height
        print("Travel plane at z=%.5f (%.1f mm above the vertex)."
              % (travel_z, args.travel_height * 1e3))

        def gate(prompt):
            if args.unattended:
                print("%s [unattended]" % prompt)
                return
            input(prompt)

        def publish_event(index, event):
            sync_pub.publish(Int8(data=index * 10 + event))
            rclpy.spin_once(node, timeout_sec=0.0)

        for index, point in enumerate(waypoints, start=1):
            hover_xyz = point + np.array([0.0, 0.0, args.hover_height])
            hover = rtde_help.getPoseObj(list(hover_xyz), orientation_fixed)
            travel = rtde_help.getPoseObj(
                [point[0], point[1], travel_z], orientation_fixed
            )

            print("--- waypoint %d (s=%+.4f m, kappa=%.2f 1/m, tilt=%.1f deg) ---"
                  % (index, offsets[index - 1], profile.curvature(offsets[index - 1]),
                     profile.tilt_deg(offsets[index - 1])))
            gate("Press <Enter> to cycle to next hover pose")
            publish_event(index, EVENT_TRAVEL)
            rtde_help.goToPose(travel)
            rtde_help.goToPose(hover)

            bias_ft_sensor(node, ft_help)
            gate("Press <Enter> to descend to contact")
            publish_event(index, EVENT_DESCEND)
            z_contact, z_final, fz_final = descend_to_contact(
                node, rtde_help, ft_help, (point[0], point[1]),
                hover_xyz[2], orientation_fixed, args,
                on_event=lambda event, i=index: publish_event(i, event),
            )
            print("  contact z=%.5f, final z=%.5f (%.1f mm preload), |Fz|=%.2f N, "
                  "predicted z=%.5f"
                  % (z_contact, z_final, (z_contact - z_final) * 1e3, fz_final,
                     point[2]))

            time.sleep(args.dwell)
            publish_event(index, EVENT_DWELL_END)
            rtde_help.goToPose(travel)
            time.sleep(0.1)

        call_enable_service(node, data_logger_client, False)
        time.sleep(0.2)

        if not args.no_home:
            print("Returning to the parked pose %s." % np.round(HOME_POSITION, 4))
            rtde_help.goToPose(rtde_help.getPoseObj(HOME_POSITION, HOME_QUAT))

        file_help.saveDataParams(
            args,
            appendTxt="Curve_sweep_%s_a%s_n%d" % (profile.name, args.a, len(offsets)),
        )
        file_help.clearTmpFolder()
        print("============ Curve sweep complete!")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=["parabola", "ellipse"],
                        default="parabola",
                        help="parabola: z = -a*s^2. ellipse: semi-axes a along "
                        "the sweep and b vertical")
    parser.add_argument("--a", type=float, default=20.0,
                        help="parabola coefficient (1/m), or the ellipse's "
                        "horizontal semi-axis (m)")
    parser.add_argument("--b", type=float, default=0.0,
                        help="ellipse vertical semi-axis (m); unused for a parabola")
    parser.add_argument("--extent", type=float, default=0.030,
                        help="furthest offset from the vertex, each way (m)")
    parser.add_argument("--num-points", type=int, default=7,
                        help="total probe points, odd so the vertex is included")
    parser.add_argument("--spacing", choices=["curvature", "x"], default="curvature",
                        help="space points evenly in curvature or in offset")
    parser.add_argument("--axis", choices=["x", "y"], default="x",
                        help="base-frame axis the profile varies along")

    parser.add_argument("--contact-force", type=float, default=0.2,
                        help="|Fz| that counts as first contact (N)")
    parser.add_argument("--preload-depth", type=float, default=0.003,
                        help="distance to descend past first contact (m)")
    parser.add_argument("--descend-step", type=float, default=3e-4,
                        help="downward z increment per step during descent (m)")
    parser.add_argument("--max-search", type=float, default=0.040,
                        help="max descent below hover before giving up (m)")
    parser.add_argument("--force-ceiling", type=float, default=8.0,
                        help="abort the preload if |Fz| reaches this (N)")
    parser.add_argument("--settle", type=float, default=0.05,
                        help="seconds to settle after each descent step")
    parser.add_argument("--descend-speed", type=float, default=0.01)
    parser.add_argument("--descend-acc", type=float, default=0.1)

    parser.add_argument("--hover-height", type=float, default=0.030,
                        help="hover height above the predicted surface (m)")
    parser.add_argument("--travel-height", type=float, default=0.080,
                        help="height above the vertex for all lateral moves (m)")
    parser.add_argument("--dwell", type=float, default=1.0,
                        help="seconds to hold while the seal settles")
    parser.add_argument("--unattended", action="store_true",
                        help="do not wait for <Enter> between moves")
    parser.add_argument("--no-home", action="store_true",
                        help="stay at the travel plane instead of returning home")

    parser.add_argument("--apex-topic", type=str, default="sphere_apex")
    parser.add_argument("--apex-samples", type=int, default=20)
    parser.add_argument("--apex-timeout", type=float, default=20.0)
    parser.add_argument("--dry-run", action="store_true",
                        help="print the planned poses and geometry without moving")
    parser.add_argument("--author", type=str, default="EDG")
    main(parser.parse_args())
