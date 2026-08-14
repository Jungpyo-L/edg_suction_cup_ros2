#!/usr/bin/env python3

# Sweeps the suction cup across a test sphere whose apex is detected from the
# RealSense point cloud by realsense_sphere_detector.py. Waypoints are given as
# offsets relative to the detected apex, so the same offsets work wherever the
# sphere is placed in the workspace.

import argparse
import math
import time
from collections import deque

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

# Fixed tool rotation for the whole sweep, same value used by simple_experiment.py.
ROTVEC_DEFAULT = [2.263, -2.179, 0.000]  # rad

# Cap on retained apex samples. The listener keeps receiving for the whole run,
# since read_avg_fz spins the node on every descent step, so an unbounded list
# would grow for as long as the experiment lasts.
APEX_BUFFER = 5000


def parse_offsets(text):
    """Parse "dx,dy,dz;dx,dy,dz" into a list of (dx, dy, dz) tuples in meters."""
    offsets = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [float(value) for value in chunk.split(",")]
        if len(parts) != 3:
            raise ValueError("Each waypoint needs exactly three values: dx,dy,dz")
        offsets.append(tuple(parts))
    if not offsets:
        raise ValueError("No waypoints given")
    return offsets


def compass_offsets(radius):
    """The apex itself plus north/west/south/east waypoints at radius/2 from it.

    North is +y and east is +x in the target frame, i.e. the compass is read
    looking down at the sphere from above. dz is zero: every waypoint keeps the
    apex height, which leaves the cup clear of the sphere since the surface
    curves away below the apex.
    """
    step = radius / 2.0
    return (
        ["center", "north", "west", "south", "east"],
        [
            (0.0, 0.0, 0.0),
            (0.0, step, 0.0),
            (-step, 0.0, 0.0),
            (0.0, -step, 0.0),
            (step, 0.0, 0.0),
        ],
    )


class ApexListener:
    """Collects sphere_apex messages so the sweep can start from a settled estimate."""

    def __init__(self, node, topic="sphere_apex"):
        self.node = node
        self.samples = deque(maxlen=APEX_BUFFER)
        # Whatever frame the detector is publishing in; reported rather than
        # assumed, since the detector's target_frame is a runtime parameter.
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
        """Average the next num_samples apex estimates. Raises on timeout."""
        self.samples.clear()
        deadline = time.time() + timeout_sec
        while len(self.samples) < num_samples:
            if time.time() > deadline:
                raise RuntimeError(
                    "Only received %d sphere_apex messages in %.0f s. Check that "
                    "realsense_sphere_detector.py is running and that the crop box "
                    "actually contains the sphere." % (len(self.samples), timeout_sec)
                )
            rclpy.spin_once(self.node, timeout_sec=0.1)

        samples = np.asarray(self.samples)
        spread = samples.max(axis=0) - samples.min(axis=0)
        self.node.get_logger().info("Apex sample spread (m): %s" % np.round(spread, 4))
        return samples.mean(axis=0)


def surface_drop(radius, dx, dy):
    """How far the sphere surface sits below the apex at horizontal offset (dx, dy).

    Zero when no radius is given, so offset-mode waypoints keep the old behavior
    of staying at apex height.
    """
    if radius <= 0.0:
        return 0.0
    d = math.hypot(dx, dy)
    if d >= radius:
        raise ValueError(
            "Waypoint is %.4f m from the apex, at or beyond the sphere radius "
            "%.4f m - there is no surface there." % (d, radius)
        )
    return radius - math.sqrt(radius * radius - d * d)


def read_avg_fz(node, ft_help, n_spins=5, timeout_sec=0.05):
    """Spin briefly so the FT callback runs, then return |Fz| (bias-subtracted, N)."""
    for _ in range(n_spins):
        rclpy.spin_once(node, timeout_sec=timeout_sec)
    return abs(getattr(ft_help, "averageFz_noOffset", 0.0))


def bias_ft_sensor(node, ft_help, n_spins=60, timeout_sec=0.05):
    """Fill the FT averaging buffer off the surface, then zero the sensor.

    Waits until FT data is actually flowing before biasing: setNowAsBias reads
    averageTx/Ty/Tz, which only exist after the first averaged callback, so a
    silent /netft_data would otherwise raise AttributeError.
    """
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


def descend_to_contact(node, rtde_help, ft_help, xy, z_start, orientation, args):
    """Step down from z_start until |Fz| crosses the contact threshold, then a
    further preload_depth. Returns (z_contact, z_final, fz_final)."""
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
    # The loop re-tests its distance bound before its force check, so a contact
    # made on the very last step would otherwise be thrown away.
    if z_contact is None and fz >= args.contact_force:
        z_contact = z
    if z_contact is None:
        raise RuntimeError(
            "Searched %.1f mm below the hover pose without reaching the %.2f N "
            "contact threshold (|Fz|=%.2f N at z=%.4f). The predicted surface is "
            "wrong, or the threshold is below the sensor noise floor."
            % (args.max_search * 1e3, args.contact_force, fz, z)
        )
    node.get_logger().info("contact at z=%.4f m, |Fz|=%.2f N" % (z_contact, fz))

    # Preload past contact. Stepped rather than a single move so the force
    # ceiling can stop it partway if the cup loads faster than expected.
    target = z_contact - args.preload_depth
    while z > target + 1e-9:
        z = max(target, z - args.descend_step)
        fz = move_to(z)
        if fz >= args.force_ceiling:
            node.get_logger().warn(
                "Force ceiling %.1f N reached at z=%.4f m, %.1f mm into a %.1f mm "
                "preload; stopping here."
                % (args.force_ceiling, z, (z_contact - z) * 1e3,
                   args.preload_depth * 1e3)
            )
            break
    return z_contact, z, fz


def validate_args(args):
    """Reject argument values that would make the descent loops non-terminating.

    Both loops in descend_to_contact advance by descend_step and stop on a
    distance bound, so a zero or negative step never satisfies its exit
    condition: the search loop would issue moveL commands forever, and a
    negative one would drive the tool upward without limit. A zero speed makes
    moveL itself block indefinitely, since it returns on motion completion.
    """
    positive = {
        "--descend-step": args.descend_step,
        "--max-search": args.max_search,
        "--contact-force": args.contact_force,
        "--descend-speed": args.descend_speed,
        "--descend-acc": args.descend_acc,
        "--apex-samples": args.apex_samples,
        "--apex-timeout": args.apex_timeout,
    }
    for name, value in positive.items():
        if value <= 0:
            raise ValueError("%s must be greater than zero, got %s" % (name, value))

    non_negative = {
        "--radius": args.radius,
        "--standoff": args.standoff,
        "--preload-depth": args.preload_depth,
        "--hover-height": args.hover_height,
        "--dwell": args.dwell,
        "--settle": args.settle,
    }
    for name, value in non_negative.items():
        if value < 0:
            raise ValueError("%s cannot be negative, got %s" % (name, value))

    if args.force_ceiling <= args.contact_force:
        raise ValueError(
            "--force-ceiling (%.2f N) must exceed --contact-force (%.2f N), or the "
            "preload aborts on the same reading that triggered contact."
            % (args.force_ceiling, args.contact_force)
        )
    if args.apex_samples > APEX_BUFFER:
        raise ValueError(
            "--apex-samples cannot exceed %d, the listener's buffer size."
            % APEX_BUFFER
        )


def wait_for_data_logger(node, client):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the data_logging service...")


def main(args):
    np.set_printoptions(precision=4)
    if args.radius > 0.0:
        labels, offsets = compass_offsets(args.radius)
        print(
            "Sphere radius %.4f m -> five waypoints: the apex, then %.4f m N, W, S, E "
            "of it. --offsets is ignored." % (args.radius, args.radius / 2.0)
        )
    else:
        offsets = parse_offsets(args.offsets)
        labels = ["waypoint %d" % i for i in range(1, len(offsets) + 1)]

    validate_args(args)
    offset_mode = args.mode == "offset"
    if offset_mode:
        print("Offset mode: holding %.1f mm above the apex plane, no contact."
              % (args.standoff * 1e3))
    else:
        print("Force mode: descending to %.2f N contact, then %.1f mm preload."
              % (args.contact_force, args.preload_depth * 1e3))

    rclpy.init()
    node = rclpy.create_node("sphere_sweep_experiment")
    try:
        apex_listener = ApexListener(node, args.apex_topic)
        ft_help = file_help = rtde_help = None
        sync_pub = data_logger_client = None

        # A dry run only needs the apex and some arithmetic. Skipping the rest
        # matters for more than speed: rtdeHelp opens an RTDEControlInterface,
        # which uploads a control program to the UR and blocks for minutes while
        # it contends with ur_control.launch.py for the controller's program
        # slot. Skipping it also lets a dry run work with just the camera and
        # detector, no robot or suction stack.
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

        print("Waiting for a settled sphere apex estimate...")
        apex = apex_listener.wait_for_apex(args.apex_samples, args.apex_timeout)
        apex_frame = apex_listener.frame_id
        print("Sphere apex (%s): x=%.5f y=%.5f z=%.5f" % ((apex_frame,) + tuple(apex)))

        orientation_fixed = R.from_rotvec(ROTVEC_DEFAULT).as_quat()

        waypoints = []
        for label, (dx, dy, dz) in zip(labels, offsets):
            # In descend mode this is the predicted surface height, used only to
            # place the hover pose; the real touch z comes from the FT sensor.
            if offset_mode:
                dz_final = dz + args.standoff
            else:
                dz_final = dz - surface_drop(args.radius, dx, dy)
            touch_xyz = apex + np.array([dx, dy, dz_final])
            # Tolerance so a waypoint sitting exactly on the limit is not rejected
            # by floating-point error in the norm.
            if np.linalg.norm(touch_xyz - apex) > args.max_offset + 1e-9:
                raise ValueError(
                    "Waypoint %s (%.3f, %.3f, %.3f) is %.3f m from the apex, beyond "
                    "the --max-offset safety limit of %.3f m."
                    % (label, dx, dy, dz, np.linalg.norm(touch_xyz - apex), args.max_offset)
                )
            waypoints.append(touch_xyz)

        if offset_mode:
            print("Planned poses (%s), all %.1f mm above the apex plane:"
                  % (apex_frame, args.standoff * 1e3))
        else:
            print("Predicted surface points (%s); actual touch z is found by the "
                  "FT sensor:" % apex_frame)
        for label, offset, touch_xyz in zip(labels, offsets, waypoints):
            print("  %-6s offset %s -> %s"
                  % (label, np.array(offset), np.round(touch_xyz, 5)))
        if args.dry_run:
            print("Dry run: no robot motion commanded.")
            return

        logging_response = call_enable_service(node, data_logger_client, True)
        if not logging_response.output_file_name.strip():
            raise RuntimeError(
                "Data logger did not create any CSV files. Check that topics in "
                "config/TopicsList.txt are currently published before recording."
            )

        try:
            ft_help.setNowAsBias()
            time.sleep(0.1)
        except Exception:
            print("set now as offset failed, but it is okay")

        for label, touch_xyz in zip(labels, waypoints):
            hover_xyz = touch_xyz + np.array([0.0, 0.0, args.hover_height])
            hover = rtde_help.getPoseObj(list(hover_xyz), orientation_fixed)

            print("--- %s ---" % label)
            input("Press <Enter> to cycle to next hover pose")  # inputs handle timing
            rtde_help.goToPose(hover)

            if offset_mode:
                input("Press <Enter> to cycle to next touch pose")
                sync_pub.publish(Int8(data=1))
                rclpy.spin_once(node, timeout_sec=0.0)
                rtde_help.goToPose(
                    rtde_help.getPoseObj(list(touch_xyz), orientation_fixed)
                )
            else:
                # Bias while clear of the surface so the descent stops on real
                # contact rather than on any resting sensor offset.
                bias_ft_sensor(node, ft_help)
                input("Press <Enter> to descend to contact")
                sync_pub.publish(Int8(data=1))
                rclpy.spin_once(node, timeout_sec=0.0)
                z_contact, z_final, fz_final = descend_to_contact(
                    node, rtde_help, ft_help, (touch_xyz[0], touch_xyz[1]),
                    hover_xyz[2], orientation_fixed, args,
                )
                print("  %-6s contact z=%.5f, final z=%.5f (%.1f mm preload), "
                      "|Fz|=%.2f N, vision predicted z=%.5f"
                      % (label, z_contact, z_final, (z_contact - z_final) * 1e3,
                         fz_final, touch_xyz[2]))

            time.sleep(args.dwell)
            if args.retract:
                rtde_help.goToPose(hover)
                time.sleep(0.1)

        call_enable_service(node, data_logger_client, False)
        time.sleep(0.2)

        if args.radius > 0.0:
            plan_txt = "radius_%s" % args.radius
        else:
            plan_txt = "offsets_%s" % args.offsets.replace(";", "__").replace(",", "_")
        file_help.saveDataParams(
            args,
            appendTxt="Sphere_sweep_%s_%s" % (args.mode, plan_txt),
        )
        file_help.clearTmpFolder()
        print("============ Sphere sweep complete!")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--offsets",
        type=str,
        default="-0.030,0,-0.004; 0,0,0; 0.030,0,-0.004",
        help="waypoints relative to the sphere apex as 'dx,dy,dz; dx,dy,dz' in meters "
        "(default: 30 mm left, apex, 30 mm right)",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=0.0,
        help="radius of curvature of the test sphere (m). When given, --offsets is "
        "ignored and five waypoints are generated: the apex itself, then radius/2 "
        "north, west, south and east of it, all at the apex height",
    )
    parser.add_argument("--hover-height", type=float, default=0.005,
                        help="hover height above each touch pose (m)")
    parser.add_argument("--dwell", type=float, default=0.5,
                        help="seconds to hold at each touch pose")
    parser.add_argument("--retract", action="store_true",
                        help="return to the hover pose after each touch")
    parser.add_argument(
        "--mode",
        choices=["offset", "force"],
        default="offset",
        help="offset: move in x-y only, holding every waypoint --standoff above "
        "the apex plane, and never touch the sphere. force: descend at each "
        "waypoint until the FT sensor senses contact, then press "
        "--preload-depth further. Defaults to offset, the safe one",
    )
    parser.add_argument("--standoff", type=float, default=0.010,
                        help="height above the apex plane held in offset mode (m)")
    parser.add_argument("--contact-force", type=float, default=0.3,
                        help="|Fz| that counts as first contact (N)")
    parser.add_argument("--preload-depth", type=float, default=0.003,
                        help="distance to descend past first contact (m)")
    parser.add_argument("--descend-step", type=float, default=3e-4,
                        help="downward z increment per step during descent (m)")
    parser.add_argument("--max-search", type=float, default=0.015,
                        help="max descent below hover before giving up (m)")
    parser.add_argument("--force-ceiling", type=float, default=8.0,
                        help="abort the preload if |Fz| reaches this (N)")
    parser.add_argument("--settle", type=float, default=0.05,
                        help="seconds to settle after each descent step")
    parser.add_argument("--descend-speed", type=float, default=0.01)
    parser.add_argument("--descend-acc", type=float, default=0.1)
    parser.add_argument("--max-offset", type=float, default=0.10,
                        help="reject waypoints farther than this from the apex (m)")
    parser.add_argument("--apex-topic", type=str, default="sphere_apex")
    parser.add_argument("--apex-samples", type=int, default=20,
                        help="apex messages averaged before planning")
    parser.add_argument("--apex-timeout", type=float, default=20.0)
    parser.add_argument("--dry-run", action="store_true",
                        help="print the planned poses without moving the robot")
    parser.add_argument("--author", type=str, default="EDG")
    main(parser.parse_args())
