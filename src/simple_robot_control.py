#!/usr/bin/env python3

# Basic robot control example for the UR10e.

import numpy as np
import rclpy
import time
from rclpy.node import Node

from helperFunction.rtde_helper import rtdeHelp
from helperFunction.ros2_helpers import quaternion_from_euler


def run_demo(node: Node):
    deg2rad = np.pi / 180.0
    np.set_printoptions(precision=4)

    rtde_help = rtdeHelp(125, node=node)

    position_a = [0.520, -0.200, 0.40]
    orientation_a = quaternion_from_euler(np.pi, 0, -np.pi / 2, "sxyz")
    pose_a = rtde_help.getPoseObj(position_a, orientation_a)

    position_b = [0.620, -0.100, 0.50]
    pose_b = rtde_help.getPoseObj(position_b, orientation_a)

    orientation_b = quaternion_from_euler(np.pi + 45 * deg2rad, 0, -np.pi / 2, "sxyz")
    pose_c = rtde_help.getPoseObj(position_b, orientation_b)

    input("Press <Enter> to go to pose A")
    rtde_help.goToPose(pose_a)
    time.sleep(1)
    print("poseA: ", rtde_help.getCurrentPose())

    input("Press <Enter> to go to pose B")
    rtde_help.goToPose(pose_b)
    time.sleep(1)
    print("poseB: ", rtde_help.getCurrentPose())

    input("Press <Enter> to go to pose C")
    rtde_help.goToPose(pose_c)
    time.sleep(1)
    print("poseC: ", rtde_help.getCurrentPose())

    print("============ Python UR_Interface demo complete!")


def main():
    rclpy.init()
    node = rclpy.create_node("edg_experiment")
    try:
        run_demo(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
