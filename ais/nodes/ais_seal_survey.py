#!/usr/bin/env python3

# Authors: Jungpyo Lee
# Seal survey for the AIS haptic-search robot demo: press the cup at the most interior point of each object top (where
# the whole lip is on the surface with the most margin) and read the chamber vacuum. Answers "what does a sealed cup
# read on these objects?" before the success threshold of ais_trial_runner.py is chosen.
#
#   ros2 run suction_cup ais_seal_survey.py --cup ch3                            # all 10 objects, 5 mm press
#   ros2 run suction_cup ais_seal_survey.py --cup ch3 --objects 10,7 --press_mm 2 3 4 5 6
#
# Points: 2D_Haptic_Search/robot/fixture_final_subset_K10/seal_points.json (object frame; clearance = distance to the
# nearest edge or hole of the as-built outline). Per point and depth: hover, zero in air, press, vacuum on, hold
# --hold_s, vacuum off, lift. Same approach, zeroing and press as a trial. Needs the fixture calibration of the cup.
# Output: table on screen and ~/EDG_Experiment/ais_robot_demo/seal_survey_<time>.json (with the vacuum time course).

import sys
from pathlib import Path

_NODES = Path(__file__).resolve().parent
sys.path[:0] = [str(_NODES.parent), str(_NODES.parent.parent / "src")]

import argparse
import json
import time
from datetime import datetime

import numpy as np

from ais_haptic.motion import move_unwound
from ais_haptic.robot_frame import load_cups, tool_pose, tool_yaw_deg

HOVER_MM, TRAVEL_MM = 10.0, 30.0
MOVE_SPEED, MOVE_ACC = 0.05, 0.3
TOUCH_SPEED, TOUCH_ACC = 0.005, 0.1


def main(args):
    repo = Path(args.repo).expanduser()
    sys.path.insert(0, str(repo))
    from tools.fixture_calibration import load_calibration

    cup = load_cups(Path(args.cups).expanduser())[args.cup]
    cal = load_calibration(repo / "robot" / "calibration" / ("fixture_calibration_%s.json" % args.cup))
    points = json.loads((repo / "robot" / "fixture_final_subset_K10" / "seal_points.json").read_text())["points"]
    by_no = {v["object_no"]: (oid, v) for oid, v in points.items()}
    wanted = [int(v) for v in args.objects.split(",")] if args.objects else sorted(by_no)

    import rclpy
    from ais_haptic.ros_io import HapticRosIO
    from helperFunction.rtde_helper import rtdeHelp

    rclpy.init()
    node = rclpy.create_node("haptic_seal_survey")
    io = HapticRosIO(node, cup.chambers)
    rtde, rows = None, []
    try:
        io.vacuum(False)
        rtde = rtdeHelp(125, node=node)
        rtde.setTCPoffset(cup.tcp_offset)
        time.sleep(0.2)
        io.wait_for_pressure()
        psi = tool_yaw_deg(list(rtde.rtde_r.getActualTCPPose())) if args.psi_deg is None else args.psi_deg
        print("cup %s, tool yaw %.1f deg, hold %.1f s, press %s mm" % (cup.name, psi, args.hold_s, args.press_mm))
        for no in wanted:
            oid, pt = by_no[no]
            xyz, _ = cal.object_pose(oid, (pt["x_obj_mm"], pt["y_obj_mm"]))
            x, y, z_top = (float(v) for v in xyz)
            if args.step:
                input("Enter: object %d (%s), point (%.1f, %.1f) mm, clearance %.1f mm " % (
                    no, oid, pt["x_obj_mm"], pt["y_obj_mm"], pt["clearance_mm"]))
            current = list(rtde.rtde_r.getActualTCPPose())
            if current[2] < z_top + TRAVEL_MM / 1000.0:
                current[2] = z_top + TRAVEL_MM / 1000.0
                rtde.rtde_c.moveL(current, MOVE_SPEED, MOVE_ACC)
            move_unwound(rtde, tool_pose([x, y, z_top + TRAVEL_MM / 1000.0], psi), args.q6_home_deg)
            for depth in args.press_mm:
                rtde.rtde_c.moveL(tool_pose([x, y, z_top + HOVER_MM / 1000.0], psi), MOVE_SPEED, MOVE_ACC)
                time.sleep(0.3)
                io.zero_in_air()
                rtde.rtde_c.moveL(tool_pose([x, y, z_top + 0.001], psi), MOVE_SPEED / 2, MOVE_ACC)
                rtde.rtde_c.moveL(tool_pose([x, y, z_top - depth / 1000.0], psi), TOUCH_SPEED, TOUCH_ACC)
                io.vacuum(True)
                t0, t, vac = time.monotonic(), [], []
                while time.monotonic() - t0 < args.hold_s:
                    t.append(time.monotonic() - t0)
                    vac.append(-io.pressure() / 1000.0)                    # kPa, vacuum positive, sensor order
                    time.sleep(0.01)
                io.vacuum(False)
                t, vac = np.array(t), np.array(vac)
                final = vac[t > t[-1] - 0.5].mean(axis=0)
                reach = [float(t[np.argmax(vac[:, j] >= 0.9 * final[j])]) if final[j] > 1.0 else None for j in range(vac.shape[1])]
                rows.append(dict(object_no=no, object=oid, x_obj_mm=pt["x_obj_mm"], y_obj_mm=pt["y_obj_mm"],
                                 clearance_mm=pt["clearance_mm"], press_mm=depth, vacuum_kpa=final.round(2).tolist(),
                                 t90_s=reach, t=t.round(3).tolist(), vacuum_trace_kpa=vac.round(2).tolist()))
                print("object %2d  clearance %4.1f mm  press %.1f mm : vacuum %s kPa (sensors 1-%d, last 0.5 s)   90%% after %s s" % (
                    no, pt["clearance_mm"], depth, np.array2string(final, precision=1), vac.shape[1],
                    ["%.2f" % r if r is not None else "-" for r in reach]))
                time.sleep(args.release_s)
                rtde.rtde_c.moveL(tool_pose([x, y, z_top + HOVER_MM / 1000.0], psi), MOVE_SPEED, MOVE_ACC)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        io.vacuum(False)
        if rtde is not None:
            try:
                p = list(rtde.rtde_r.getActualTCPPose())
                p[2] += TRAVEL_MM / 1000.0
                rtde.rtde_c.moveL(p, MOVE_SPEED, MOVE_ACC)
            except Exception as exc:
                print("could not lift: %s" % exc)
            rtde.disconnect()
        io.close()
        node.destroy_node()
        rclpy.shutdown()
    if rows:
        out = Path(args.results).expanduser()
        out.mkdir(parents=True, exist_ok=True)
        path = out / ("seal_survey_%s_%s.json" % (cup.name, datetime.now().strftime("%Y%m%d_%H%M%S")))
        path.write_text(json.dumps(dict(cup=cup.name, hold_s=args.hold_s, rows=rows), indent=1))
        low = np.array([min(r["vacuum_kpa"]) for r in rows])
        print("\nlowest chamber per point: min %.1f, median %.1f, max %.1f kPa -> %s" % (low.min(), np.median(low), low.max(), path))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cup", required=True)
    parser.add_argument("--objects", default=None, help="object numbers, e.g. 10,7,9 (default: all)")
    parser.add_argument("--press_mm", type=float, nargs="+", default=[5.0], help="press depth(s) below the contact height")
    parser.add_argument("--hold_s", type=float, default=3.0, help="vacuum-on time per point")
    parser.add_argument("--release_s", type=float, default=0.5)
    parser.add_argument("--psi_deg", type=float, default=None, help="tool yaw (default: keep the current one)")
    parser.add_argument("--step", action="store_true", help="wait for Enter before each object")
    parser.add_argument("--q6_home_deg", type=float, default=0.0)
    parser.add_argument("--repo", default="~/projects/2D_Haptic_Search")
    parser.add_argument("--cups", default="~/ros2_ws/src/suction_cup/ais/config/cups.json")
    parser.add_argument("--results", default="~/EDG_Experiment/ais_robot_demo")
    main(parser.parse_args())
