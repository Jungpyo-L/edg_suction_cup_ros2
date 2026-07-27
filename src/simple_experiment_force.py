#!/usr/bin/env python3

# Waypoint experiment for the UR10e: at each waypoint the tool descends straight
# down until the ATI FT sensor registers a target contact force, then stops.
# Same waypoints as simple_experiment.py; the touch z is found by the FT sensor.

import argparse
import time

import numpy as np
import rclpy
from std_msgs.msg import Int8

from suction_cup.srv import Enable
from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.ros2_helpers import call_enable_service
from helperFunction.rtde_helper import rtdeHelp

from scipy.spatial.transform import Rotation as R


def wait_for_data_logger(node, client):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the data_logging service...")


def read_avg_fz(node, ft_help, n_spins=5, timeout_sec=0.05):
    """Spin briefly so the FT callback runs, then return |Fz| (bias-subtracted, N)."""
    for _ in range(n_spins):
        rclpy.spin_once(node, timeout_sec=timeout_sec)
    return abs(getattr(ft_help, "averageFz_noOffset", 0.0))


def bias_ft_sensor(node, ft_help, n_spins=60, timeout_sec=0.05):
    """Fill the FT averaging buffer in free space, then zero the sensor.

    Waits until FT data is actually flowing before biasing. Without this,
    setNowAsBias() reads averageTx/Ty/Tz, which only exist after the first
    averaged callback, so a silent /netft_data would raise AttributeError.
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


def descend_until_force(node, rtde_help, ft_help, hover_pose, target_force,
                        step=3e-4, max_depth=15e-3, settle=0.05,
                        speed=0.01, acc=0.1):
    """Step straight down (base -z) from hover_pose until |Fz| >= target_force,
    or until max_depth is exceeded (safety cap). Returns (final_pose, achieved_force)."""
    x = hover_pose.pose.position.x
    y = hover_pose.pose.position.y
    z0 = hover_pose.pose.position.z
    orientation = [hover_pose.pose.orientation.x, hover_pose.pose.orientation.y,
                   hover_pose.pose.orientation.z, hover_pose.pose.orientation.w]

    z = z0
    fz = read_avg_fz(node, ft_help)
    while (z0 - z) < max_depth:
        if fz >= target_force:
            node.get_logger().info("Target force reached: Fz=%.2f N" % fz)
            break
        z -= step
        rtde_help.goToPose(rtde_help.getPoseObj([x, y, z], orientation), speed=speed, acc=acc)
        time.sleep(settle)
        fz = read_avg_fz(node, ft_help)
        node.get_logger().info("descent z=%.4f m, Fz=%.2f N" % (z, fz))
    else:
        node.get_logger().warn(
            "Hit max_depth (%.1f mm) before reaching target force; stopping. Fz=%.2f N"
            % (max_depth * 1e3, fz))
    return rtde_help.getCurrentPose(), fz


def main(args):
    np.set_printoptions(precision=4)

    rclpy.init()
    node = rclpy.create_node("edg_experiment")
    try:
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

        HOVER_HEIGHT_M = 0.005

        # Fixed rotation for all waypoints
        ROTVEC_DEFAULT = [2.263, -2.179, 0.000] # in rad
        orientation_fixed = R.from_rotvec(ROTVEC_DEFAULT).as_quat()

        # Same waypoints as simple_experiment.py. depth is now the hover-start
        # height; the touch z is found by the FT sensor. Home is travel-only.
        waypoints = [
            {"xy": (0.53060, -0.13720), "depth_cm": 0.23464, "descend": False}, # home (travel only)
            {"xy": (0.55165, -0.13716), "depth_cm": args.depth1, "descend": True}, # left
            {"xy": (0.56850, -0.13719), "depth_cm": args.depth2, "descend": True}, # middle
            {"xy": (0.58554, -0.13716), "depth_cm": args.depth3, "descend": True},] # right

        def build_poses(xy, z_touch):
            x, y = xy
            hover  = rtde_help.getPoseObj([x, y, z_touch + HOVER_HEIGHT_M], orientation_fixed)
            touch  = rtde_help.getPoseObj([x, y, z_touch], orientation_fixed)
            return hover, touch

        logging_response = call_enable_service(node, data_logger_client, True)
        if not logging_response.output_file_name.strip():
            raise RuntimeError(
                "Data logger did not create any CSV files. Check that topics in "
                "config/TopicsList.txt are currently published before recording."
            )

        for wp in waypoints:
            hover, _ = build_poses(wp["xy"], wp["depth_cm"])

            input("Press <Enter> to move to hover pose") # inputs handle timing
            rtde_help.goToPose(hover)

            if not wp.get("descend", True):
                continue  # travel-only waypoint (e.g. home); no force descent

            # zero the FT sensor while off the surface, so the descent stops on
            # the actual contact force rather than any resting sensor bias.
            bias_ft_sensor(node, ft_help)

            input("Press <Enter> to descend until target force")
            final_pose, achieved_force = descend_until_force(
                node, rtde_help, ft_help, hover,
                target_force=args.force,
                step=args.force_step,
                max_depth=args.max_depth,
            )
            node.get_logger().info(
                "Contact at z=%.4f m with Fz=%.2f N"
                % (final_pose.pose.position.z, achieved_force)
            )

            # to retract after each touch, uncomment:
            # rtde_help.goToPose(hover)
            # time.sleep(0.1)

        call_enable_service(node, data_logger_client, False)
        time.sleep(0.2)

        file_help.saveDataParams(
            args,
            appendTxt="Waypoint_force_experiment_force_" + str(args.force),
        )

        file_help.clearTmpFolder()
        print("============ Python UR_Interface demo complete!")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# currently only assuming three waypoints + home
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # depths set the hover-start height (m) above which the force-guided descent
    # begins; the actual touch z is found by the FT sensor.
    parser.add_argument("--depth1", type=float, help="hover-start height for side [left] waypoint (m)", default=0.05600)
    parser.add_argument("--depth2", type=float, help="hover-start height for middle waypoint (m)", default=0.05600)
    parser.add_argument("--depth3", type=float, help="hover-start height for side [right] waypoint (m)", default=0.05600)
    parser.add_argument("--force", type=float, help="target contact force |Fz| to stop the descent (N)", default=3.0)
    parser.add_argument("--force_step", type=float, help="downward z increment per step during descent (m)", default=3e-4)
    parser.add_argument("--max_depth", type=float, help="max descent from hover before aborting (m)", default=15e-3)
    parser.add_argument("--author", type=str, help="argument for str type", default="EDG")
    parser.add_argument("--cycle", type=int, help="the number of cycle to apply", default=1)
    main(parser.parse_args())
