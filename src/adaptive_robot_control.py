#!/usr/bin/env python3

# Adaptive robot control example for the UR10e.

import argparse
import time

import numpy as np
import rclpy

from suction_cup.srv import Enable
from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.ros2_helpers import call_enable_service, quaternion_from_euler
from helperFunction.rtde_helper import rtdeHelp


def wait_for_data_logger(node, client):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the data_logging service...")


def main(args):
    np.set_printoptions(precision=4)

    rclpy.init()
    node = rclpy.create_node("edg_experiment")
    try:
        ft_help = FT_CallbackHelp(node)
        time.sleep(0.5)
        rtde_help = rtdeHelp(125, node=node)
        time.sleep(0.5)
        file_help = fileSaveHelp()
        adpt_help = adaptMotionHelp(d_w=1, d_lat=10e-3, d_z=5e-3)

        data_logger_client = node.create_client(Enable, "data_logging")
        wait_for_data_logger(node, data_logger_client)
        call_enable_service(node, data_logger_client, False)
        time.sleep(1)
        file_help.clearTmpFolder()

        start_position = [0.520, -0.200, 0.40]
        start_orientation = quaternion_from_euler(np.pi, 0, -np.pi / 2, "sxyz")
        start_pose = rtde_help.getPoseObj(start_position, start_orientation)

        input("Press <Enter> to go to startPose")
        rtde_help.goToPose(start_pose)
        time.sleep(1)

        ft_help.setNowAsBias()

        input("Press <Enter> to adaptive robot control")
        start_time = time.time()
        while (time.time() - start_time) < args.timeLimit:
            rclpy.spin_once(node, timeout_sec=0.0)
            t_normal = adpt_help.get_Tmat_axialMove(ft_help.averageFz_noOffset, args.normalForce)
            t_rotation = adpt_help.get_Tmat_RotateInX(direction=1)
            target_transform = t_normal @ t_rotation
            current_pose = rtde_help.getCurrentPose()
            target_pose = adpt_help.get_PoseStamped_from_T_initPose(target_transform, current_pose)
            rtde_help.goToPoseAdaptive(target_pose, time=2)

        print("============ Python UR_Interface demo complete!")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeLimit", type=float, help="time limit for the adaptive motion", default=5)
    parser.add_argument("--pathlLimit", type=float, help="path-length limit for the adaptive motion (m)", default=0.1)
    parser.add_argument("--normalForce", type=float, help="normal force threshold", default=0.5)
    main(parser.parse_args())
