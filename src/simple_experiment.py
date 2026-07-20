#!/usr/bin/env python3

# Basic experiment demo for UR10e robot motion while logging data.

import argparse
import time

import numpy as np
import rclpy
from std_msgs.msg import Int8

from suction_cup.srv import Enable
from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.ros2_helpers import call_enable_service, quaternion_from_euler
from helperFunction.rtde_helper import rtdeHelp

from scipy.spatial.transform import Rotation as R

def wait_for_data_logger(node, client):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the data_logging service...")


def main(args):
    deg2rad = np.pi / 180.0
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

        waypoints = [
            {"xy": (0.53060, -0.13720), "depth_cm": 0.23464}, # home position
            {"xy": (0.55165, -0.13716), "depth_cm": args.depth1}, # left 
            {"xy": (0.56850, -0.13719), "depth_cm": args.depth2}, # middle
            {"xy": (0.58554, -0.13716), "depth_cm": args.depth3},] # right

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
            hover, touch = build_poses(wp["xy"], wp["depth_cm"])

            input("Press <Enter> to cycle to next hover pose") # inputs handle timing
            rtde_help.goToPose(hover)
            input("Press <Enter> to cycle to next touch pose")
            rtde_help.goToPose(touch)

            # include the two lines below if you only want the touch to be for a brief instant
            # rtde_help.goToPose(hover)
            # time.sleep(0.1)


        # position_a = [0.580, -0.098, 0.223 - args.depth * 1e-2]
        # orientation_a = quaternion_from_euler(np.pi, 0, -np.pi / 2, "sxyz")
        # pose_a = rtde_help.getPoseObj(position_a, orientation_a)

        # orientation_b = quaternion_from_euler(np.pi + 45 * deg2rad, 0, -np.pi / 2, "sxyz")
        # pose_b = rtde_help.getPoseObj(position_a, orientation_b)

        # input("Press <Enter> to go start pose")
        # rtde_help.goToPose(pose_a)

        # input("Press <Enter> to go start experiment")
        # try:
        #     ft_help.setNowAsBias()
        #     time.sleep(0.1)
        # except Exception:
        #     print("set now as offset failed, but it is okay")

        # for _ in range(5):
        #     sync_pub.publish(Int8(data=1))
        #     rclpy.spin_once(node, timeout_sec=0.0)
        #     time.sleep(0.1)

        # logging_response = call_enable_service(node, data_logger_client, True)
        # if not logging_response.output_file_name.strip():
        #     raise RuntimeError(
        #         "Data logger did not create any CSV files. Check that topics in "
        #         "config/TopicsList.txt are currently published before recording."
        #     )
        # time.sleep(0.2)

        # for _ in range(args.cycle):
        #     rclpy.spin_once(node, timeout_sec=0.0)
        #     sync_pub.publish(Int8(data=1))
        #     rtde_help.goToPose(pose_b)
        #     time.sleep(0.1)
        #     rtde_help.goToPose(pose_a)
        #     time.sleep(0.1)

        call_enable_service(node, data_logger_client, False)
        time.sleep(0.2)

        file_help.saveDataParams(
            args,
            appendTxt="Waypoint_touch_experiment_depth1_" + str(args.depth1) + "_depth2_" + str(args.depth2) + "_depth3_" + str(args.depth3),
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
    parser.add_argument("--depth1", type=float, help="touch height for side [left] waypoint (m)", default=0.05600)
    parser.add_argument("--depth2", type=float, help="touch height for middle waypoint (m)", default=0.05600)
    parser.add_argument("--depth3", type=float, help="touch height for side [right] waypoint (m)", default=0.05600)
    parser.add_argument("--author", type=str, help="argument for str type", default="EDG")
    parser.add_argument("--cycle", type=int, help="the number of cycle to apply", default=1)
    main(parser.parse_args())
