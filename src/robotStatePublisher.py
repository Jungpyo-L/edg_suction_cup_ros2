#!/usr/bin/env python3
# Publishes the UR TCP pose reported by RTDE on the endEffectorPose topic.

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

import rtde_receive

from helperFunction.transformation_matrix import getPoseObj


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__("robot_state_publisher")
        self.declare_parameter("robot_ip", "10.0.0.1")
        self.declare_parameter("rtde_frequency", 125)
        self.declare_parameter("publish_rate", 30.0)

        robot_ip = self.get_parameter("robot_ip").value
        rtde_frequency = self.get_parameter("rtde_frequency").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.publisher = self.create_publisher(PoseStamped, "endEffectorPose", 10)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip, rtde_frequency)

    def publish_pose(self):
        tcp_pose = self.rtde_r.getActualTCPPose()
        position = [tcp_pose[0], tcp_pose[1], tcp_pose[2]]
        rotation = R.from_rotvec(np.array([tcp_pose[3], tcp_pose[4], tcp_pose[5]]))
        pose = getPoseObj(position, rotation.as_quat(), self)
        self.publisher.publish(pose)


def main():
    rclpy.init()
    node = RobotStatePublisher()
    sleep_time = 1.0 / node.publish_rate
    try:
        while rclpy.ok():
            node.publish_pose()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
