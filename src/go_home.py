#!/usr/bin/env python3
"""Send the arm to the parked pose the experiments start and end at.

Lifts straight up first, never sideways, so a cup left on or near a sphere is
not dragged across it, then crosses to the parked pose - the same retreat an
aborted sweep does.

    ros2 run suction_cup go_home.py
    ros2 run suction_cup go_home.py --lift 0.10
"""

import argparse

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lift", type=float, default=0.05,
                        help="how far to rise straight up before crossing (m)")
    args = parser.parse_args()

    import rclpy
    from scipy.spatial.transform import Rotation as R

    from helperFunction.rtde_helper import rtdeHelp
    import sphere_sweep_experiment as sweep

    rclpy.init()
    node = rclpy.create_node("go_home")
    try:
        print("Opening the RTDE control interface ...")
        rtde_help = rtdeHelp(125, node=node)
        pose = rtde_help.rtde_r.getActualTCPPose()
        # Lift in whatever yaw the tool is in, so the cup does not turn while
        # it may still be touching something.
        current = R.from_rotvec(pose[3:6]).as_quat()
        print("Tool at %s" % np.round(pose[:3], 4))
        # Never come down on the way: lift to at least the parked height.
        sweep.retreat_to_home(rtde_help, max(pose[2] + args.lift, sweep.HOME_POSITION[2]),
                              current)
        print("Done.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
