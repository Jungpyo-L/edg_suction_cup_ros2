#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Record the default location of a characterization object (edge / corner) for JP_characterization_lateral_positioning.py.
#
#   ros2 run suction_cup record_corner_location.py --ch 3 --corner 270
#
# Vacuum stays off. Sets the cup TCP of the setup, points the tool straight down (the script's disengage orientation),
# optionally moves 30 mm above the currently stored location, then keyboard jog (helperFunction/keyboard_jog.py) until
# the lip just touches the top surface with the cup centred on the edge / corner vertex (x offset 0). Enter saves the
# point to config/lateral_corner_locations.json under (setup, ch, corner); the characterization script then uses it
# instead of the hard-coded disengage_by_corner value.

import argparse
import json
import time
from datetime import datetime
from math import pi

import numpy as np
import rclpy
from std_msgs.msg import Int8

import JP_characterization_lateral_positioning as lateral
from helperFunction.keyboard_jog import jog
from helperFunction.rtde_helper import rtdeHelp

APPROACH_MM = 30.0
MOVE_SPEED, MOVE_ACC = 0.05, 0.2


def main(args):
    tcp_offset, stored = lateral.resolve_hardware(args)
    print("setup %s, ch %d, corner %d: TCP %s, stored location %s mm" % (
        args.setup, args.ch, args.corner, tcp_offset, np.round(np.array(stored) * 1000, 2)))

    rclpy.init()
    node = rclpy.create_node("record_corner_location")
    pwm_pub = node.create_publisher(Int8, "pwm", 1)
    rtde_help = None
    try:
        pwm_pub.publish(Int8(data=0))
        rtde_help = rtdeHelp(125, node=node)
        rtde_help.setTCPoffset(tcp_offset)
        time.sleep(0.2)
        orientation = lateral.yaw_quat(pi / 2)      # same as the script's disengage pose

        current = list(rtde_help.rtde_r.getActualTCPPose())
        print("current TCP: x %.1f y %.1f z %.1f mm" % tuple(np.array(current[:3]) * 1000))
        if input("Move %.0f mm above the stored location, tool straight down? [y/N] " % APPROACH_MM).strip().lower() == "y":
            above = list(stored)
            above[2] += APPROACH_MM / 1000.0
            up = list(current[:3])
            up[2] = max(up[2], above[2])
            rtde_help.goToPose(rtde_help.getPoseObj(up, orientation), speed=MOVE_SPEED, acc=MOVE_ACC)
            rtde_help.goToPose(rtde_help.getPoseObj(above, orientation), speed=MOVE_SPEED, acc=MOVE_ACC)
        else:
            here = list(current[:3])
            rtde_help.goToPose(rtde_help.getPoseObj(here, orientation), speed=MOVE_SPEED, acc=MOVE_ACC)

        start_z = list(rtde_help.rtde_r.getActualTCPPose())[2]
        pose = jog(rtde_help, "corner %d: lip just touching, cup centred on the edge / vertex" % args.corner,
                   z_floor=start_z - args.max_drop_mm / 1000.0)
        position = [round(v, 5) for v in pose[:3]]
        print("recorded: x %.2f y %.2f z %.2f mm (moved %.2f / %.2f / %.2f mm from the stored location)" % (
            *(np.array(position) * 1000), *((np.array(position) - np.array(stored)) * 1000)))

        data = json.loads(lateral.LOCATIONS_FILE.read_text()) if lateral.LOCATIONS_FILE.exists() else {}
        entry = data.setdefault(args.setup, {}).setdefault("ch%d" % args.ch, {})
        if str(args.corner) in entry:
            print("replacing previous: %s" % entry[str(args.corner)])
        entry[str(args.corner)] = dict(position=position, tcp_offset=tcp_offset,
                                       recorded=datetime.now().isoformat(timespec="seconds"), note=args.note)
        lateral.LOCATIONS_FILE.parent.mkdir(parents=True, exist_ok=True)
        lateral.LOCATIONS_FILE.write_text(json.dumps(data, indent=1))
        print("saved to %s" % lateral.LOCATIONS_FILE)

        lift = list(rtde_help.rtde_r.getActualTCPPose())
        lift[2] += args.lift_mm / 1000.0
        rtde_help.rtde_c.moveL(lift, MOVE_SPEED / 2, MOVE_ACC)
    except KeyboardInterrupt:
        print("\nAborted; nothing saved.")
    finally:
        pwm_pub.publish(Int8(data=0))
        if rtde_help is not None:
            rtde_help.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--corner", type=int, required=True, help="90, 180 or 270")
    parser.add_argument("--ch", type=int, default=3)
    parser.add_argument("--setup", default=lateral.ACTIVE_SETUP)
    parser.add_argument("--tcp_x", type=float, default=None)
    parser.add_argument("--tcp_y", type=float, default=None)
    parser.add_argument("--tcp_z", type=float, default=None)
    parser.add_argument("--pose_x", type=float, default=None)
    parser.add_argument("--pose_y", type=float, default=None)
    parser.add_argument("--pose_z", type=float, default=None)
    parser.add_argument("--max_drop_mm", type=float, default=60.0, help="jog floor below the start height")
    parser.add_argument("--lift_mm", type=float, default=10.0, help="lift after saving")
    parser.add_argument("--note", default="")
    main(parser.parse_args())
