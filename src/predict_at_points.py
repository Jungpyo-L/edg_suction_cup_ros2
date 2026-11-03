#!/usr/bin/env python3
"""Visit fixed x-y points, find the surface under each, and say which sphere it is.

At every point the arm comes over from --start-z, searches straight down at
--search-speed until the cup touches (never below --floor-z), then taps from
--hover-height above that contact exactly as predict_curvature.py does - the
training run's settings - and prints the prediction for that point before
moving on, so each one can be checked by eye.

The points are given in the robot's base frame, or taken from the apexes the
sweep saved for each sphere, in which case the true size is known and every
prediction is marked right or wrong:

    ros2 run suction_cup predict_at_points.py --model ~/....joblib \
        --points "0.53,-0.14; 0.58,-0.14" --start-z 0.15 --floor-z 0.06
    ros2 run suction_cup predict_at_points.py --model ~/....joblib \
        --apexes 0.020,0.025,0.030,0.040 --taps 3

Each point is logged and saved as its own .mat, so a prediction can be
rerun later with predict_curvature.py --mat.
"""

import argparse
import os
import sys
import time

import numpy as np

import predict_curvature as pc


def parse_points(text):
    """'x,y; x,y' as a list of (x, y) in metres."""
    points = []
    for piece in text.split(";"):
        if not piece.strip():
            continue
        values = [float(v) for v in piece.split(",")]
        if len(values) != 2:
            raise ValueError("each --points entry is 'x,y', got %r" % piece.strip())
        points.append(tuple(values))
    if not points:
        raise ValueError("--points was given nothing to visit.")
    return points


def plan_points(args, sweep):
    """(x, y, true radius or None) per point, and the heights to work between."""
    if args.apexes:
        radii = sweep.parse_radii(args.apexes)
        apexes = []
        for radius in radii:
            apex = sweep.stored_apex(radius)
            if apex is None:
                sys.exit("No apex saved for the %s m sphere in %s."
                         % (sweep.apex_key(radius), sweep.APEX_STORE))
            apexes.append(apex)
        points = [(a[0], a[1], r) for a, r in zip(apexes, radii)]
        tops = [a[2] for a in apexes]
        # Close above the highest sphere, so the search is short, and never
        # deeper than the indentation limit below the lowest.
        start_z = args.start_z if args.start_z is not None else max(tops) + args.travel_height
        floor_z = args.floor_z if args.floor_z is not None else min(tops) - args.max_indent
    else:
        if args.start_z is None or args.floor_z is None:
            sys.exit("--points needs --start-z and --floor-z: nothing else says "
                     "how high the surfaces are.")
        xy = parse_points(args.points)
        truths = ([float(t) for t in args.truths.split(",")] if args.truths
                  else [None] * len(xy))
        if len(truths) != len(xy):
            sys.exit("--truths has %d entries for %d points." % (len(truths), len(xy)))
        points = [(x, y, t) for (x, y), t in zip(xy, truths)]
        start_z, floor_z = args.start_z, args.floor_z
    if floor_z >= start_z:
        sys.exit("--floor-z %.4f must be below --start-z %.4f." % (floor_z, start_z))
    return points, start_z, floor_z


def run_points(args, bundle):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from scipy.spatial.transform import Rotation as R
    from std_msgs.msg import Int16

    from suction_cup.srv import Enable
    from helperFunction.fileSaveHelper import fileSaveHelp
    from helperFunction.ros2_helpers import call_enable_service
    from helperFunction.rtde_helper import rtdeHelp
    import sphere_sweep_experiment as sweep

    points, start_z, floor_z = plan_points(args, sweep)
    yaws = pc.tap_yaws(args.taps, args.yaw_span, args.yaw_start)
    print("%d points, searched from z=%.4f down to at most z=%.4f:"
          % (len(points), start_z, floor_z))
    for index, (x, y, truth) in enumerate(points, start=1):
        print("  point %d: x=%.4f y=%.4f%s" % (index, x, y, "" if truth is None
                                                else "  (truth R %.0f mm)" % (truth * 1e3)))
    input("Press <Enter> to start, Ctrl-C to abort.")

    # The search is a tap of its own, only lighter and allowed the whole drop.
    search = argparse.Namespace(**vars(args))
    search.tap_speed = args.search_speed
    search.tap_force = args.search_force
    search.max_search = start_z - floor_z

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("predict_at_points")
    rtde_help = watch = client = None
    logging_response = None
    base_rotation = R.from_rotvec(sweep.ROTVEC_DEFAULT)
    upright = base_rotation.as_quat()
    results = []

    def turned(degrees):
        return (base_rotation * R.from_euler("z", degrees, degrees=True)).as_quat()

    try:
        watch = sweep.TapWatcher(args.contact_force)
        args.ft_rate = watch.measure_rate()
        sweep.check_tap_speed(args, args.ft_rate)
        sweep.check_tap_speed(search, args.ft_rate)
        file_help = fileSaveHelp()
        rtde_help = rtdeHelp(125, node=node)
        time.sleep(0.5)

        sync_pub = node.create_publisher(Int16, "sync", 1)
        client = node.create_client(Enable, "data_logging")
        sweep.wait_for_data_logger(node, client)
        call_enable_service(node, client, False)
        time.sleep(1)
        file_help.clearTmpFolder()

        here = np.array(rtde_help.rtde_r.getActualTCPPose()[:3])
        rtde_help.goToPose(rtde_help.getPoseObj(
            [here[0], here[1], max(here[2], start_z)], upright))

        for number, (x, y, truth) in enumerate(points, start=1):
            print()
            print("============ point %d of %d: x=%.4f y=%.4f" % (number, len(points), x, y))
            # Across at the start height, never sideways near a surface.
            rtde_help.goToPose(rtde_help.getPoseObj([x, y, start_z], upright))
            z_contact, _, _, reason = sweep.tap_surface(
                rtde_help, watch, (x, y), start_z, upright, search, on_event=lambda event: None)
            if z_contact is None:
                print("  nothing touched down to z=%.4f (%s) - skipping this point."
                      % (floor_z, reason))
                results.append((number, truth, None))
                continue
            print("  surface found at z=%.5f" % z_contact)

            hover_z = z_contact + args.hover_height
            spot_z = z_contact + args.travel_height
            # On args so they land in the .mat, and so curvature_classifier
            # reads the file as a tap run with every waypoint wanted.
            args.mode = "tap"
            args.repeat = args.taps
            args.radius = truth or 0.0
            args.tap_radius = np.full(args.taps, args.radius)
            args.tap_plan = np.array([[0.0, 0.0, yaw] for yaw in yaws])
            args.tap_results = np.zeros((0, 5))
            args.touch_point = np.array([x, y, z_contact])

            logging_response = call_enable_service(node, client, True)
            for index, yaw in enumerate(yaws, start=1):
                orientation = turned(yaw)
                sync_pub.publish(Int16(data=index * 10 + sweep.EVENT_TRAVEL))
                rtde_help.goToPose(rtde_help.getPoseObj([x, y, spot_z], orientation))
                rtde_help.goToPose(rtde_help.getPoseObj([x, y, hover_z], orientation))
                tz_contact, z_stop, peak, why = sweep.tap_surface(
                    rtde_help, watch, (x, y), hover_z, orientation, args,
                    on_event=lambda event, i=index: sync_pub.publish(Int16(data=i * 10 + event)))
                sweep.report_tap("tap %d" % index, tz_contact, z_stop, peak, why, args.tap_force)
                args.tap_results = np.vstack([args.tap_results, [
                    index, np.nan if tz_contact is None else tz_contact, z_stop, peak,
                    sweep.TAP_REASONS.index(why)]])
                rtde_help.goToPose(rtde_help.getPoseObj([x, y, spot_z], orientation))
                time.sleep(0.1)
            call_enable_service(node, client, False)
            logged, logging_response = logging_response.output_file_name, None
            time.sleep(0.2)
            sweep.trim_to_taps(logged)
            rtde_help.goToPose(rtde_help.getPoseObj([x, y, start_z], upright))

            # Deliberately not "Sphere_sweep_tap": curvature_classifier
            # collects training runs by that name.
            append = "Curvature_points_tap_p%d_%dx" % (number, args.taps)
            folder, stem = file_help.saveDataParams(args, appendTxt=append)
            file_help.clearTmpFolder()
            path = os.path.join(folder, "%s_%s.mat" % (stem, append))
            try:
                verdict, detail, _ = pc.classify(path, bundle)
            except SystemExit as exc:
                print("  could not classify point %d: %s" % (number, exc))
                results.append((number, truth, None))
                continue
            line = "============ point %d: %s (%s)" % (number, verdict, detail)
            if truth:
                actual = class_label(truth)
                line += " - actually %s: %s" % (actual, "correct" if verdict == actual else "WRONG")
            print(line)
            results.append((number, truth, verdict))
            if number < len(points) and not args.unattended:
                input("Press <Enter> for the next point.")

        sweep.retreat_to_home(rtde_help, start_z, upright, go_home=not args.no_home)
    except BaseException:
        print()
        print("============ Stopping. Recovering the arm; do not press Ctrl-C again.")
        if rtde_help is not None:
            sweep.retreat_to_home(rtde_help, start_z, upright, go_home=not args.no_home)
        if client is not None and logging_response is not None:
            try:
                call_enable_service(node, client, False)
            except Exception as exc:
                print("  could not stop the data logger: %s" % exc)
        raise
    finally:
        if rtde_help is not None:
            rtde_help.disconnect()
        if watch is not None:
            watch.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return results


def class_label(radius):
    import curvature_classifier as clf
    return clf.class_name(round(1.0 / radius, 6))


def main(args):
    if args.taps < 1:
        sys.exit("--taps must be at least 1.")
    if not args.points and not args.apexes:
        sys.exit("Give --points or --apexes.")
    bundle = pc.load_model(args.model, pc.find_ml_dir(args.ml_dir))
    force_max = bundle["config"].get("force_max", 0.0)
    if bundle["config"].get("representation") == "force" and args.tap_force <= force_max:
        sys.exit("--tap-force %.1f N must exceed the %.1f N the model reads each "
                 "tap up to." % (args.tap_force, force_max))
    print("Model %s: one of %s" % (os.path.basename(args.model),
                                   ", ".join(str(c) for c in bundle["classes"])))

    results = run_points(args, bundle)
    print()
    print("============ Summary")
    judged = 0
    right = 0
    for number, truth, verdict in results:
        line = "  point %d: %s" % (number, verdict or "no prediction")
        if truth and verdict:
            judged += 1
            right += verdict == class_label(truth)
            line += "  (actually %s)" % class_label(truth)
        print(line)
    if judged:
        print("  %d of %d right" % (right, judged))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True,
                        help="a .joblib saved by ml/curvature_classifier.py")
    parser.add_argument("--points", default=None,
                        help="x-y points in the base frame (m), as 'x,y; x,y'")
    parser.add_argument("--truths", default=None,
                        help="--points: the real radius (m) under each point, "
                        "as '0.020,0.030', to mark predictions right or wrong. "
                        "Never given to the model")
    parser.add_argument("--apexes", default=None,
                        help="visit the saved apexes of these spheres instead, "
                        "as radii (m), e.g. '0.020,0.030'; the radius is then "
                        "the truth")
    parser.add_argument("--start-z", type=float, default=None,
                        help="height every search starts from and every move "
                        "between points is made at (m). Must clear every "
                        "surface. With --apexes, defaults to the highest apex "
                        "plus --travel-height")
    parser.add_argument("--floor-z", type=float, default=None,
                        help="the search never goes below this height (m). With "
                        "--apexes, defaults to the lowest apex less --max-indent")
    parser.add_argument("--search-speed", type=float, default=0.010,
                        help="speed of the downward search for the surface (m/s)")
    parser.add_argument("--search-force", type=float, default=1.0,
                        help="|Fz| that ends the search (N); light, so the "
                        "search only touches")
    parser.add_argument("--taps", type=int, default=1,
                        help="taps at each point, each at its own yaw within "
                        "--yaw-span; with more than one the verdict combines them")
    parser.add_argument("--unattended", action="store_true",
                        help="go straight on to the next point without waiting")
    parser.add_argument("--ml-dir", default=None,
                        help="folder holding curvature_classifier.py, if it is "
                        "not found beside src/")
    # Defaults are the four-sphere training run's settings.
    parser.add_argument("--tap-speed", type=float, default=0.010,
                        help="descent and retract speed of the measured taps (m/s)")
    parser.add_argument("--tap-force", type=float, default=2.5,
                        help="stop each measured tap at this |Fz| (N)")
    parser.add_argument("--tap-acc", type=float, default=0.1)
    parser.add_argument("--hover-height", type=float, default=0.003,
                        help="height above the found surface each tap starts from (m)")
    parser.add_argument("--travel-height", type=float, default=0.020,
                        help="height above the surface where the cup turns between taps (m)")
    parser.add_argument("--max-indent", type=float, default=0.006,
                        help="safety stop this far past contact (m)")
    parser.add_argument("--max-search", type=float, default=0.020,
                        help="safety stop this far below hover, for the measured taps (m)")
    parser.add_argument("--contact-force", type=float, default=0.2,
                        help="|Fz| that counts as first contact (N)")
    parser.add_argument("--yaw-span", type=float, default=30.0)
    parser.add_argument("--yaw-start", type=float, default=-15.0)
    parser.add_argument("--no-home", action="store_true")
    main(parser.parse_args())
