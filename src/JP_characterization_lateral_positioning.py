#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Data collection by sweeping yaw and lateral (x) offset.

import argparse
import copy
import pickle
import time
from math import floor, pi

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int8

from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.ros2_helpers import call_enable_service
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from suction_cup.srv import Enable


# =============================================================================
# HARDWARE SETUP — edit this block when swapping prototypes / cups / fixtures
#
# Pick which named setup to use with ACTIVE_SETUP, or pass --setup <name>.
# CLI overrides (--tcp_z, --pose_x/y/z) win over the values below.
#
# tcp_offset: UR set_tcp pose [x, y, z, rx, ry, rz] in m / rad
#   z is usually flange -> cup lip (tool length). Change this per prototype.
# disengage_position: hover pose above the workpiece [x, y, z] in m
#   If set (not None), it is used for every --corner.
# disengage_by_corner: fallback hover poses keyed by --corner (90 / 180 / 270)
# =============================================================================
ACTIVE_SETUP = "default"

SETUPS = {
    # Old cell values kept as a starting point — measure and replace for new HW.
    "default": {
        "tcp_offset": [0.0, 0.0, 0.150, 0.0, 0.0, 0.0],
        "tcp_offset_by_ch": {
            # Extra length for taller 5/6-ch cups on the old mount.
            5: [0.0, 0.0, 0.150 + 0.02 - 0.0008, 0.0, 0.0, 0.0],
            6: [0.0, 0.0, 0.150 + 0.02 - 0.0008, 0.0, 0.0, 0.0],
        },
        "disengage_position": None,
        "disengage_by_corner": {
            90: [0.6165, -0.2258, 0.0170],
            180: [0.6092, -0.275, 0.0180],
            270: [0.555, 0.100, 0.0170],
        },
    },
    # Copy / rename this entry for each new prototype. Fill in measured numbers.
    "tensr_proto_v1": {
        "tcp_offset": [0.0, 0.0, 0.170, 0.0, 0.0, 0.0],  # TODO: measure tool length
        "tcp_offset_by_ch": {},
        "disengage_position": [0.581, -0.206, 0.245],  # TODO: measure hover pose
        "disengage_by_corner": {},
    },
}


def str2bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("1", "true", "t", "yes", "y")


def wait_for_data_logger(node, client):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the data_logging service...")


def spin_briefly(node, n=5, timeout_sec=0.05):
    for _ in range(n):
        rclpy.spin_once(node, timeout_sec=timeout_sec)


def yaw_quat(yaw_rad):
    # Matches the old tf.transformations.quaternion_from_euler(..., 'szxy') usage.
    return Rotation.from_euler("ZXY", [yaw_rad, pi, 0]).as_quat()


def default_yaw_for_channels(n_ch):
    if n_ch == 3:
        return pi / 2 - 60 * pi / 180
    if n_ch == 4:
        return pi / 2 - 45 * pi / 180
    if n_ch == 5:
        return pi / 2 - 90 * pi / 180
    if n_ch == 6:
        return pi / 2 - 60 * pi / 180
    return pi / 2 - 45 * pi / 180


def resolve_hardware(args):
    """Return (tcp_offset, disengage_position) from SETUP + CLI overrides."""
    if args.setup not in SETUPS:
        raise ValueError(
            "Unknown setup '%s'. Available: %s" % (args.setup, ", ".join(SETUPS))
        )
    setup = SETUPS[args.setup]

    tcp_offset = list(setup["tcp_offset_by_ch"].get(args.ch, setup["tcp_offset"]))
    if args.tcp_z is not None:
        tcp_offset[2] = args.tcp_z
    if args.tcp_x is not None:
        tcp_offset[0] = args.tcp_x
    if args.tcp_y is not None:
        tcp_offset[1] = args.tcp_y

    if setup.get("disengage_position") is not None:
        disengage = list(setup["disengage_position"])
    else:
        by_corner = setup.get("disengage_by_corner") or {}
        if args.corner not in by_corner:
            raise ValueError(
                "Setup '%s' has no disengage pose for --corner %s. "
                "Set disengage_position in SETUPS, add disengage_by_corner[%s], "
                "or pass --pose_x/y/z."
                % (args.setup, args.corner, args.corner)
            )
        disengage = list(by_corner[args.corner])

    if args.pose_x is not None:
        disengage[0] = args.pose_x
    if args.pose_y is not None:
        disengage[1] = args.pose_y
    if args.pose_z is not None:
        disengage[2] = args.pose_z

    return tcp_offset, disengage


def main(args):
    DUTYCYCLE_100 = 100
    DUTYCYCLE_0 = 0

    SYNC_RESET = 0
    SYNC_START = 1
    SYNC_STOP = 2

    F_normal_thres = [args.normalForce, args.normalForce + 0.5]
    args.normalForce_thres = F_normal_thres

    tcp_offset, disengage_position_init = resolve_hardware(args)
    args.tcp_offset = tcp_offset
    args.disengagePosition_init = disengage_position_init
    args.setup_name = args.setup

    print("========== HARDWARE ==========")
    print("setup:              ", args.setup)
    print("tcp_offset [m,rad]: ", tcp_offset)
    print("disengage [m]:      ", disengage_position_init)
    print("ch / corner:        ", args.ch, "/", args.corner)
    print("==============================")

    np.set_printoptions(precision=4)

    rclpy.init()
    node = rclpy.create_node("suction_cup_jp_lateral")
    rtde_help = None
    p_help = None

    try:
        ft_help = FT_CallbackHelp(node)
        time.sleep(0.5)
        p_help = P_CallbackHelp(node)
        time.sleep(0.5)
        rtde_help = rtdeHelp(125, node=node)
        time.sleep(0.5)
        file_help = fileSaveHelp()
        adpt_help = adaptMotionHelp(d_w=0.5, d_lat=0.5e-3, d_z=0.1e-3)

        time.sleep(0.5)
        rtde_help.setTCPoffset(tcp_offset)
        time.sleep(0.2)

        target_pwm_pub = node.create_publisher(Int8, "pwm", 1)
        target_pwm_pub.publish(Int8(data=DUTYCYCLE_0))

        sync_pub = node.create_publisher(Int8, "sync", 1)
        sync_pub.publish(Int8(data=SYNC_RESET))

        data_logger_client = node.create_client(Enable, "data_logging")
        wait_for_data_logger(node, data_logger_client)
        call_enable_service(node, data_logger_client, False)
        time.sleep(1)
        file_help.clearTmpFolder()

        xoffset = args.xoffset

        default_yaw = default_yaw_for_channels(args.ch)
        set_orientation = yaw_quat(pi / 2)
        disengage_pose = rtde_help.getPoseObj(disengage_position_init, set_orientation)

        input("Press <Enter> to go disEngagePose")
        rtde_help.goToPose(disengage_pose)
        time.sleep(0.1)

        p_help.startSampling()
        time.sleep(1)
        spin_briefly(node, n=20)
        ft_help.setNowAsBias()
        p_help.setNowAsOffset()

        input("Press <Enter> to go normal to get engage point")

        if args.zHeight:
            engage_z = disengage_position_init[2] - args.deformation * 1e-3
        else:
            print("move along normal")
            target_pose = rtde_help.getCurrentPose()
            far_flag = True
            spin_briefly(node)
            f_normal = ft_help.averageFz_noOffset
            target_pwm_pub.publish(Int8(data=DUTYCYCLE_0))

            while far_flag:
                spin_briefly(node)
                f_normal = ft_help.averageFz_noOffset

                if f_normal > -F_normal_thres[0]:
                    t_move = adpt_help.get_Tmat_TranlateInZ(direction=1)
                    target_pose = adpt_help.get_PoseStamped_from_T_initPose(
                        t_move, target_pose
                    )
                    rtde_help.goToPoseAdaptive(target_pose, time=0.1)
                elif f_normal < -F_normal_thres[1]:
                    t_move = adpt_help.get_Tmat_TranlateInZ(direction=-1)
                    target_pose = adpt_help.get_PoseStamped_from_T_initPose(
                        t_move, target_pose
                    )
                    rtde_help.goToPoseAdaptive(target_pose, time=0.1)
                else:
                    far_flag = False
                    rtde_help.stopAtCurrPoseAdaptive()
                    print("reached threshhold normal force: ", f_normal)
                    args.normalForceUsed = f_normal
                    time.sleep(0.1)

            target_pose_engaged = rtde_help.getCurrentPose()
            engage_z = target_pose_engaged.pose.position.z
            rtde_help.goToPose(disengage_pose)
            time.sleep(0.1)

            with open(file_help.ResultSavingDirectory + "/engage_z.p", "wb") as engage_z_file:
                pickle.dump(engage_z, engage_z_file)

        input("Press <Enter> to start to data collection")
        start_angle_flag = True
        xoffsets = np.arange(xoffset, 5, 1)
        suction_flag = False

        for j in xoffsets:
            print("Move to the updated disengage point")
            args.xoffset = int(j)
            disengage_position = copy.deepcopy(disengage_position_init)
            print("disengagePosition: ", disengage_position)
            disengage_position[0] += j * 0.001
            print("disengagePosition: ", disengage_position)
            engage_position = copy.deepcopy(disengage_position)
            engage_position[2] = engage_z

            for i in range(round(args.angle / 5) + 1):
                print("offset: ", j)
                print("Pose Idx: ", i)
                args.theta = round((pi / 36 * i) * 180 / pi)
                print("Theta =", args.theta)

                if args.startAngle > args.theta and start_angle_flag:
                    continue
                start_angle_flag = False

                target_orientation = yaw_quat(default_yaw - 5 * pi / 180 * i)
                target_pose = rtde_help.getPoseObj(disengage_position, target_orientation)
                target_pose_init = target_pose
                rtde_help.goToPose(target_pose)
                target_pwm_pub.publish(Int8(data=DUTYCYCLE_0))
                sync_pub.publish(Int8(data=SYNC_RESET))
                time.sleep(0.1)

                p_help.startSampling()
                time.sleep(0.3)
                spin_briefly(node, n=10)
                p_help.setNowAsOffset()

                target_orientation = yaw_quat(default_yaw - 5 * pi / 180 * i)
                target_pose = rtde_help.getPoseObj(engage_position, target_orientation)
                rtde_help.goToPose(target_pose)
                target_pwm_pub.publish(Int8(data=DUTYCYCLE_100))

                print("Start to record data")
                call_enable_service(node, data_logger_client, True)
                time.sleep(0.2)
                sync_pub.publish(Int8(data=SYNC_START))
                time.sleep(1)
                spin_briefly(node, n=10)

                p_init = p_help.four_pressure
                f_normal = ft_help.averageFz_noOffset
                args.normalForceActual = f_normal
                args.pressure_avg = p_init
                p_vac = p_help.P_vac

                if (
                    p_init is not None
                    and all(np.array(p_init) < p_vac)
                    and i == 0
                ):
                    print("Suction Engage Succeed from initial touch")
                    suction_flag = True
                else:
                    suction_flag = False

                print("Stop to record data")
                sync_pub.publish(Int8(data=SYNC_STOP))
                time.sleep(0.1)
                target_pwm_pub.publish(Int8(data=DUTYCYCLE_0))
                rtde_help.goToPose(target_pose_init)

                time.sleep(0.1)
                call_enable_service(node, data_logger_client, False)
                time.sleep(0.1)

                file_help.saveDataParams(
                    args,
                    appendTxt=(
                        "jp_lateral_"
                        + "setup_"
                        + str(args.setup)
                        + "_corner_"
                        + str(args.corner)
                        + "_xoffset_"
                        + str(args.xoffset)
                        + "_theta_"
                        + str(args.theta)
                        + "_material_"
                        + str(args.material)
                    ),
                )
                file_help.clearTmpFolder()
                p_help.stopSampling()
                time.sleep(0.1)

            if not suction_flag:
                for k in range(floor(args.angle / 90)):
                    target_orientation = yaw_quat(default_yaw + pi / 2 * (k + 1))
                    target_pose = rtde_help.getPoseObj(
                        disengage_position, target_orientation
                    )
                    rtde_help.goToPose(target_pose)
                    time.sleep(0.1)

        print("Go to disengage point")
        set_orientation = yaw_quat(pi / 2)
        disengage_pose = rtde_help.getPoseObj(disengage_position_init, set_orientation)
        rtde_help.goToPose(disengage_pose)
        time.sleep(0.3)

        print("============ Stopping data logger ...")
        call_enable_service(node, data_logger_client, False)
        p_help.stopSampling()
        print("============ Python UR_Interface demo complete!")

    except KeyboardInterrupt:
        pass
    finally:
        try:
            target_pwm_pub.publish(Int8(data=DUTYCYCLE_0))
        except Exception:
            pass
        if p_help is not None:
            p_help.shutdown()
        if rtde_help is not None:
            rtde_help.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Yaw x lateral offset characterization. "
            "Edit SETUPS / ACTIVE_SETUP at the top for each prototype, "
            "or override pose/TCP on the command line."
        )
    )
    parser.add_argument(
        "--setup",
        type=str,
        help="named hardware setup from SETUPS at top of file",
        default=ACTIVE_SETUP,
        choices=sorted(SETUPS.keys()),
    )
    parser.add_argument(
        "--tcp_x",
        type=float,
        default=None,
        help="override TCP offset x (m)",
    )
    parser.add_argument(
        "--tcp_y",
        type=float,
        default=None,
        help="override TCP offset y (m)",
    )
    parser.add_argument(
        "--tcp_z",
        type=float,
        default=None,
        help="override TCP offset z / tool length (m)",
    )
    parser.add_argument(
        "--pose_x",
        type=float,
        default=None,
        help="override disengage / hover pose x (m)",
    )
    parser.add_argument(
        "--pose_y",
        type=float,
        default=None,
        help="override disengage / hover pose y (m)",
    )
    parser.add_argument(
        "--pose_z",
        type=float,
        default=None,
        help="override disengage / hover pose z (m)",
    )
    parser.add_argument(
        "--useStoredData",
        type=str2bool,
        help="take image or use existingFile",
        default=False,
    )
    parser.add_argument(
        "--storedDataDirectory",
        type=str,
        help="location of target saved File",
        default="",
    )
    parser.add_argument(
        "--startIdx", type=int, help="startIndex Of the pose List", default=0
    )
    parser.add_argument(
        "--xoffset", type=int, help="x direction offset (mm)", default=-4
    )
    parser.add_argument(
        "--angle", type=int, help="angles of exploration (deg)", default=360
    )
    parser.add_argument(
        "--startAngle", type=int, help="start angle of exploration (deg)", default=0
    )
    parser.add_argument(
        "--primitives",
        type=str,
        help="types of primitives (edge, corner, etc.)",
        default="edge",
    )
    parser.add_argument(
        "--normalForce", type=float, help="normal force", default=1.5
    )
    parser.add_argument(
        "--deformation",
        type=float,
        help="normal deformation during data collection (mm)",
        default=4.0,
    )
    parser.add_argument(
        "--zHeight",
        type=str2bool,
        help="use preset height mode? (rather than normal force)",
        default=True,
    )
    parser.add_argument("--ch", type=int, help="number of channel", default=4)
    parser.add_argument(
        "--newCup",
        type=str2bool,
        help="whether we use new suction cup (ver2) or not",
        default=False,
    )
    parser.add_argument(
        "--corner", type=int, help="corner angle of the object", default=180
    )
    parser.add_argument(
        "--material",
        type=int,
        help="0: Mold max 40, 1: Elastic 50A (formlab), 2: Agilus 30 (Objet)",
        default=0,
    )
    parser.add_argument(
        "--disk_curvature",
        type=int,
        help="curvature or radius of disk",
        default=0,
    )

    main(parser.parse_args())
