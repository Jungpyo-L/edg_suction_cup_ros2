#!/usr/bin/env python3

import rclpy
from scipy.spatial.transform import Rotation

from suction_cup.srv import Enable


def quaternion_from_euler(roll, pitch, yaw, axes='sxyz'):
    if axes != 'sxyz':
        raise ValueError("Only static xyz Euler angles ('sxyz') are supported.")
    return Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()


def call_enable_service(node, client, enabled, timeout_sec=15.0):
    """Toggle data logging, refusing to wait forever for an answer.

    Without a timeout this blocks silently if the logger accepts the request
    and never responds, which looks identical to the script having hung.
    """
    request = Enable.Request()
    request.enable_data_logging = bool(enabled)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        future.cancel()
        raise RuntimeError(
            "The data_logging service did not respond within %.0f s. Check that "
            "data_logger.py is still running." % timeout_sec
        )
    if future.exception() is not None:
        raise future.exception()
    return future.result()
