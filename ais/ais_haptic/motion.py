#!/usr/bin/env python3
"""Wrist unwinding for the haptic-search robot demo (needs only an rtdeHelp-like object, no ROS).

The yaw controllers turn the tool the same way trial after trial, and moveL to the next start takes the shortest
rotation from the current joints, so wrist 3 keeps winding until it reaches its joint limit. Before each trial the
start pose is therefore reached with moveJ to an IK solution whose wrist 3 is moved by whole turns to lie closest to
a home angle; the tool pose is identical, only the wrist is unwound.
"""
import numpy as np

WRIST3_LIMIT_DEG = 360.0


def wrap_rad(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def unwound_joints(rtde, pose, q6_home_deg=0.0):
    """Joint target for `pose` on the current arm branch, with wrist 3 within 180 deg of q6_home_deg."""
    q = list(rtde.rtde_c.getInverseKinematics(list(pose), list(rtde.rtde_r.getActualQ())))
    home = np.radians(q6_home_deg)
    q[5] = float(home + wrap_rad(q[5] - home))
    if not rtde.rtde_c.isJointsWithinSafetyLimits(q):
        raise RuntimeError('Unwound joint target is outside the safety limits: %s deg' % np.round(np.degrees(q), 1))
    return q


def move_unwound(rtde, pose, q6_home_deg=0.0, speed=0.5, acc=0.5):
    """moveJ (rad/s, rad/s^2) to `pose` with wrist 3 unwound. Call it in free space: a joint move is not a straight line."""
    q = unwound_joints(rtde, pose, q6_home_deg)
    rtde.rtde_c.moveJ(q, speed, acc)
    return q


def wrist3_deg(rtde):
    return float(np.degrees(rtde.rtde_r.getActualQ()[5]))
