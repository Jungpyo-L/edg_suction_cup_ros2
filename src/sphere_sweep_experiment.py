#!/usr/bin/env python3

# Sweeps the suction cup across a test sphere whose apex is detected from the
# RealSense point cloud by realsense_sphere_detector.py. Waypoints are given as
# offsets relative to the detected apex, so the same offsets work wherever the
# sphere is placed in the workspace.

import argparse
import time

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


class ApexListener:
    """Collects sphere_apex messages so the sweep can start from a settled estimate."""

    def __init__(self, node, topic="sphere_apex"):
        self.node = node
        self.samples = []
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = node.create_subscription(PointStamped, topic, self.callback, qos)

    def callback(self, msg):
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


def wait_for_data_logger(node, client):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the data_logging service...")


def main(args):
    np.set_printoptions(precision=4)
    offsets = parse_offsets(args.offsets)

    rclpy.init()
    node = rclpy.create_node("sphere_sweep_experiment")
    try:
        ft_help = FT_CallbackHelp(node)
        time.sleep(0.5)
        file_help = fileSaveHelp()
        time.sleep(0.5)
        rtde_help = rtdeHelp(125, node=node)
        time.sleep(0.5)
        apex_listener = ApexListener(node, args.apex_topic)

        sync_pub = node.create_publisher(Int8, "sync", 1)
        data_logger_client = node.create_client(Enable, "data_logging")
        wait_for_data_logger(node, data_logger_client)
        call_enable_service(node, data_logger_client, False)
        time.sleep(1)
        file_help.clearTmpFolder()

        print("Waiting for a settled sphere apex estimate...")
        apex = apex_listener.wait_for_apex(args.apex_samples, args.apex_timeout)
        print("Sphere apex (base_link): x=%.5f y=%.5f z=%.5f" % tuple(apex))

        orientation_fixed = R.from_rotvec(ROTVEC_DEFAULT).as_quat()

        waypoints = []
        for dx, dy, dz in offsets:
            touch_xyz = apex + np.array([dx, dy, dz - args.press_depth])
            if np.linalg.norm(touch_xyz - apex) > args.max_offset:
                raise ValueError(
                    "Waypoint (%.3f, %.3f, %.3f) is %.3f m from the apex, beyond the "
                    "--max-offset safety limit of %.3f m."
                    % (dx, dy, dz, np.linalg.norm(touch_xyz - apex), args.max_offset)
                )
            waypoints.append(touch_xyz)

        print("Planned touch poses (base_link):")
        for offset, touch_xyz in zip(offsets, waypoints):
            print("  offset %s -> %s" % (np.array(offset), np.round(touch_xyz, 5)))
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

        for touch_xyz in waypoints:
            hover_xyz = touch_xyz + np.array([0.0, 0.0, args.hover_height])
            hover = rtde_help.getPoseObj(list(hover_xyz), orientation_fixed)
            touch = rtde_help.getPoseObj(list(touch_xyz), orientation_fixed)

            input("Press <Enter> to cycle to next hover pose")  # inputs handle timing
            rtde_help.goToPose(hover)
            input("Press <Enter> to cycle to next touch pose")
            sync_pub.publish(Int8(data=1))
            rclpy.spin_once(node, timeout_sec=0.0)
            rtde_help.goToPose(touch)
            time.sleep(args.dwell)
            if args.retract:
                rtde_help.goToPose(hover)
                time.sleep(0.1)

        call_enable_service(node, data_logger_client, False)
        time.sleep(0.2)

        file_help.saveDataParams(
            args,
            appendTxt="Sphere_sweep_press_%s_offsets_%s"
            % (args.press_depth, args.offsets.replace(";", "__").replace(",", "_")),
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
    parser.add_argument("--press-depth", type=float, default=0.0,
                        help="extra downward press applied to every touch pose (m)")
    parser.add_argument("--hover-height", type=float, default=0.005,
                        help="hover height above each touch pose (m)")
    parser.add_argument("--dwell", type=float, default=0.5,
                        help="seconds to hold at each touch pose")
    parser.add_argument("--retract", action="store_true",
                        help="return to the hover pose after each touch")
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
