#!/usr/bin/env python3

import rclpy
from scipy.spatial.transform import Rotation

from suction_cup.srv import Enable


def quaternion_from_euler(roll, pitch, yaw, axes='sxyz'):
    if axes != 'sxyz':
        raise ValueError("Only static xyz Euler angles ('sxyz') are supported.")
    return Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()


def call_enable_service(node, client, enabled):
    request = Enable.Request()
    request.enable_data_logging = bool(enabled)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.exception() is not None:
        raise future.exception()
    return future.result()
