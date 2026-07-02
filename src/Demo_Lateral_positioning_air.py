#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Description: Simple lateral positioning controller test without force checking or vacuum success detection

import argparse
import time
from math import pi

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int8

from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.hapticSearch2D import hapticSearch2DHelp
from helperFunction.ros2_helpers import call_enable_service
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from suction_cup.srv import Enable


def wait_for_data_logger(node, client):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the data_logging service...")


def main(args):
    DUTYCYCLE_100 = 100
    DUTYCYCLE_0 = 0

    SYNC_RESET = 0

    np.set_printoptions(precision=4)

    rclpy.init()
    node = rclpy.create_node("suction_cup_demo_lateral")

    try:
        P_help = P_CallbackHelp(node)
        time.sleep(0.5)
        rtde_help = rtdeHelp(125, node=node)
        time.sleep(0.5)
        file_help = fileSaveHelp()
        adaptMotionHelp(dw=0.5, d_lat=2e-3, d_z=0.1e-3)
        search_help = hapticSearch2DHelp(
            d_lat=5e-3,
            d_yaw=1.5,
            n_ch=args.ch,
            p_reverse=args.reverse,
        )

        target_pwm_pub = node.create_publisher(Int8, "pwm", 1)
        target_pwm_pub.publish(Int8(data=DUTYCYCLE_0))

        sync_pub = node.create_publisher(Int8, "sync", 1)
        sync_pub.publish(Int8(data=SYNC_RESET))

        data_logger_client = node.create_client(Enable, "data_logging")
        wait_for_data_logger(node, data_logger_client)
        call_enable_service(node, data_logger_client, False)
        time.sleep(1)
        file_help.clearTmpFolder()

        disengage_position_init = [0.581, -0.206, 0.245]
        if args.ch == 3:
            default_yaw = pi / 2 - 60 * pi / 180
        elif args.ch == 4:
            default_yaw = pi / 2 - 45 * pi / 180
        elif args.ch == 5:
            default_yaw = pi / 2 - 90 * pi / 180
        elif args.ch == 6:
            default_yaw = pi / 2 - 60 * pi / 180
        else:
            default_yaw = pi / 2 - 45 * pi / 180

        set_orientation = Rotation.from_euler("ZXY", [default_yaw, pi, 0]).as_quat()
        disengage_pose = rtde_help.getPoseObj(disengage_position_init, set_orientation)

        time_limit = 20

        input("Press <Enter> to go disEngagePose")
        rtde_help.goToPose(disengage_pose)
        time.sleep(0.1)

        print("Start sampling")
        P_help.startSampling()
        time.sleep(0.5)
        P_help.setNowAsOffset()

        input("Press <Enter> to start lateral search")
        target_pwm_pub.publish(Int8(data=DUTYCYCLE_100))
        call_enable_service(node, data_logger_client, True)
        start_time = time.time()

        while time.time() - start_time <= time_limit:
            rclpy.spin_once(node, timeout_sec=0.0)
            P_array = P_help.four_pressure
            if P_array is None:
                time.sleep(0.05)
                continue

            T_later, T_yaw, T_align = search_help.get_Tmats_from_controller(
                P_array, args.controller
            )
            T_move = T_later @ T_yaw @ T_align

            measured_curr_pose = rtde_help.getCurrentPose()
            curr_pose = search_help.get_PoseStamped_from_T_initPose(
                T_move, measured_curr_pose
            )
            rtde_help.goToPoseAdaptive(curr_pose)
            time.sleep(0.05)

        args.timeOverFlag = True
        args.elapsedTime = time.time() - start_time
        print("Controller testing completed after %.2f seconds" % args.elapsedTime)

        rtde_help.stopAtCurrPoseAdaptive()
        target_pwm_pub.publish(Int8(data=DUTYCYCLE_0))
        time.sleep(0.1)

        print("============ Stopping data logger ...")
        call_enable_service(node, data_logger_client, False)
        P_help.stopSampling()
        time.sleep(0.3)

        file_help.saveDataParams(
            args,
            appendTxt="demo_lateral_positioning_controller_"
            + str(getattr(args, "controller", "greedy")),
        )
        file_help.clearTmpFolder()
        print("============ Python UR_Interface demo complete!")
    except KeyboardInterrupt:
        pass
    finally:
        if "P_help" in locals():
            P_help.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ch", type=int, help="number of channel", default=4)
    parser.add_argument(
        "--reverse", type=bool, help="when we use reverse airflow", default=False
    )
    parser.add_argument(
        "--controller",
        type=str,
        help="2D haptic controllers (greedy, yaw, momentum, yaw_momentum)",
        default="greedy",
    )
    main(parser.parse_args())
