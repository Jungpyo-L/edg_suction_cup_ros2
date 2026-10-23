#!/usr/bin/env python3
"""Tap a sphere you have placed anywhere and say which trained size it is.

Jog the cup onto any point of the sphere, press Enter, and the arm taps that
spot exactly the way sphere_sweep_experiment.py --mode tap does: it rises to
--hover-height above the point you accepted, descends at --tap-speed until
--tap-force, and comes straight back up. The defaults are the settings the
four-sphere training run used, so a tap here is the same measurement as a tap
in the training data.

The taps are logged and saved like any run, then cut into the model's input by
the same code that built the training set - ml/curvature_classifier.py - so
the two cannot drift apart. The model chooses its own input: if it was trained
on pressure only, the indentation logged here is simply not passed to it.

    ros2 run suction_cup predict_curvature.py --model ~/curvature_force_32pt_minirocket.joblib
    ros2 run suction_cup predict_curvature.py --model ... --start 0.025 --truth 0.025 --taps 3

--start begins the jog above a sphere's saved apex rather than wherever the arm
is, which saves a long walk down from the parked pose. --truth only labels the
saved file and reports right or wrong; it is never shown to the model.

Without a robot, classify a file that is already saved:

    python3 src/predict_curvature.py --model ... --mat DataLog_..._Curvature_predict_tap_1x.mat
"""

import argparse
import glob
import os
import sys
import time
from collections import Counter

import numpy as np


def find_ml_dir(given):
    """Where curvature_classifier.py and minirocket.py live.

    The installed script is a symlink into the source tree under
    --symlink-install, so the real path finds ml/ beside src/. A plain install
    copies the script away from it, hence the workspace fallback.
    """
    here = os.path.dirname(os.path.realpath(__file__))
    candidates = [given] if given else [
        os.path.join(here, "..", "ml"),
        os.path.expanduser("~/ros2_ws/src/suction_cup/ml"),
    ]
    for path in candidates:
        if path and os.path.isfile(os.path.join(path, "curvature_classifier.py")):
            return os.path.abspath(path)
    sys.exit("Cannot find ml/curvature_classifier.py; pass --ml-dir.")


def load_model(path, ml_dir):
    import joblib
    # A saved MiniRocket pipeline names its module, which is only importable
    # with ml/ on the path.
    sys.path.insert(0, ml_dir)
    bundle = joblib.load(os.path.expanduser(path))
    config = bundle["config"]
    if config.get("representation") != "force":
        print("!! This model reads taps on a %r grid. The force grid is the one "
              "that does not depend on tap speed." % config.get("representation"))
    return bundle


def tap_yaws(taps, span, start):
    """One yaw per tap, centred in equal slices of the span, so a single tap
    sits at the middle of the trained range rather than at its edge."""
    return [start + span * (i + 0.5) / taps for i in range(taps)]


def classify(path, bundle):
    """Predict the sphere for every usable tap in a saved file."""
    import curvature_classifier as clf

    config = argparse.Namespace(**bundle["config"])
    run, problem = clf.load_run(path, config.channels)
    if run is None:
        sys.exit("Cannot read %s: %s" % (path, problem))
    samples, skipped = clf.extract_taps(run, config)
    for waypoint, reason in skipped:
        print("  tap %d not usable: %s" % (waypoint, reason))
    if not samples:
        sys.exit("No usable taps to classify.")

    # Picked by name, so a model trained without indentation is given exactly
    # the pressure channels, in the order it learned them.
    logged = clf.channel_names(config, "z" in run)
    wanted = [str(name) for name in bundle["channels"]]
    missing = [name for name in wanted if name not in logged]
    if missing:
        sys.exit("The model needs %s, which this file does not have (it has %s)."
                 % (", ".join(missing), ", ".join(logged)))
    rows = [logged.index(name) for name in wanted]
    waypoints = [waypoint for waypoint, _ in samples]
    features = np.array([array[rows] for _, array in samples]).reshape(len(samples), -1)

    model = bundle["model"]
    predicted = model.predict(features)
    proba = model.predict_proba(features) if hasattr(model, "predict_proba") else None

    print()
    print("Model input: %s" % ", ".join(wanted))
    for i, (waypoint, label) in enumerate(zip(waypoints, predicted)):
        peak, stopped = clf.tap_outcome(run, waypoint)
        line = "  tap %d: %-9s peak %.2f N, stopped on %s" % (waypoint, label, peak, stopped)
        if proba is not None:
            line += "   " + "  ".join("%s %.0f%%" % (c, p * 1e2)
                                      for c, p in zip(model.classes_, proba[i]))
        print(line)

    if proba is not None:
        # Averaged rather than voted, so one confident tap outweighs two
        # hesitant ones.
        mean = proba.mean(axis=0)
        verdict = model.classes_[int(np.argmax(mean))]
        detail = "mean probability %.0f%%" % (mean.max() * 1e2)
    else:
        # RidgeClassifierCV, behind MiniRocket, gives scores but no
        # probabilities, so taps are counted instead.
        verdict, votes = Counter(predicted).most_common(1)[0]
        detail = "%d of %d taps" % (votes, len(predicted))
    return verdict, detail, len(samples)


def tap_and_save(args):
    """Jog, tap, log and save. Returns the path of the saved .mat."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from scipy.spatial.transform import Rotation as R
    from std_msgs.msg import Int16

    from suction_cup.srv import Enable
    from helperFunction.fileSaveHelper import fileSaveHelp
    from helperFunction.ros2_helpers import call_enable_service
    from helperFunction.rtde_helper import rtdeHelp
    import sphere_sweep_experiment as sweep

    yaws = tap_yaws(args.taps, args.yaw_span, args.yaw_start)
    # On args so they land in the .mat, and so curvature_classifier reads the
    # file as a tap run with every waypoint wanted.
    args.mode = "tap"
    args.repeat = args.taps
    args.radius = args.truth or 0.0
    args.tap_radius = np.full(args.taps, args.radius)
    args.tap_plan = np.array([[0.0, 0.0, yaw] for yaw in yaws])
    args.tap_results = np.zeros((0, 5))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("predict_curvature")
    rtde_help = watch = client = None
    logging_response = None
    base_rotation = R.from_rotvec(sweep.ROTVEC_DEFAULT)
    upright = base_rotation.as_quat()
    clear_z = None

    def turned(degrees):
        return (base_rotation * R.from_euler("z", degrees, degrees=True)).as_quat()

    try:
        watch = sweep.TapWatcher(args.contact_force)
        args.ft_rate = watch.measure_rate()
        sweep.check_tap_speed(args, args.ft_rate)
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
        if args.start == "here":
            start = here
        else:
            start = sweep.stored_apex(float(args.start))
            if start is None:
                raise RuntimeError("No apex saved for the %s m sphere in %s."
                                   % (sweep.apex_key(float(args.start)), sweep.APEX_STORE))
            # Over the saved apex from above, never sideways at sphere height.
            # The jog then opens --jog-height above it.
            high = max(here[2], start[2] + args.clear_height)
            rtde_help.goToPose(rtde_help.getPoseObj([here[0], here[1], high], upright))
            rtde_help.goToPose(rtde_help.getPoseObj([start[0], start[1], high], upright))

        print()
        print("Jog onto the point to tap - anywhere on the sphere - so the cup "
              "just touches it, then press Enter.")
        touch = sweep.jog_to_apex(rtde_help, start, upright, args)
        clear_z = touch[2] + args.clear_height
        args.touch_point = touch

        hover_z = touch[2] + args.hover_height
        spot_z = touch[2] + args.travel_height
        logging_response = call_enable_service(node, client, True)

        for index, yaw in enumerate(yaws, start=1):
            orientation = turned(yaw)
            print("--- tap %d of %d, yaw %+.1f deg ---" % (index, args.taps, yaw))
            # Up clear of the surface, turn there, then down to hover: the cup
            # never turns or moves sideways while it is near the sphere.
            sync_pub.publish(Int16(data=index * 10 + sweep.EVENT_TRAVEL))
            rtde_help.goToPose(rtde_help.getPoseObj([touch[0], touch[1], spot_z], orientation))
            rtde_help.goToPose(rtde_help.getPoseObj([touch[0], touch[1], hover_z], orientation))
            z_contact, z_stop, peak, reason = sweep.tap_surface(
                rtde_help, watch, (touch[0], touch[1]), hover_z, orientation, args,
                on_event=lambda event, i=index: sync_pub.publish(Int16(data=i * 10 + event)))
            sweep.report_tap("tap %d" % index, z_contact, z_stop, peak, reason, args.tap_force)
            args.tap_results = np.vstack([args.tap_results, [
                index, np.nan if z_contact is None else z_contact, z_stop, peak,
                sweep.TAP_REASONS.index(reason)]])
            rtde_help.goToPose(rtde_help.getPoseObj([touch[0], touch[1], spot_z], orientation))
            time.sleep(0.1)

        call_enable_service(node, client, False)
        time.sleep(0.2)
        sweep.trim_to_taps(logging_response.output_file_name)
        sweep.retreat_to_home(rtde_help, clear_z, upright, go_home=not args.no_home)

        # Deliberately not "Sphere_sweep_tap": curvature_classifier collects
        # training runs by that name, and a test tap must never become one.
        append = "Curvature_predict_tap_%dx" % args.taps
        folder, stem = file_help.saveDataParams(args, appendTxt=append)
        file_help.clearTmpFolder()
        return os.path.join(folder, "%s_%s.mat" % (stem, append))
    except BaseException:
        print()
        print("============ Stopping. Recovering the arm; do not press Ctrl-C again.")
        if rtde_help is not None:
            sweep.retreat_to_home(rtde_help, clear_z, upright, go_home=not args.no_home)
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


def main(args):
    if args.taps < 1:
        sys.exit("--taps must be at least 1.")
    bundle = load_model(args.model, find_ml_dir(args.ml_dir))
    force_max = bundle["config"].get("force_max", 0.0)
    if bundle["config"].get("representation") == "force" and args.tap_force <= force_max:
        sys.exit("--tap-force %.1f N must exceed the %.1f N the model reads each "
                 "tap up to." % (args.tap_force, force_max))
    print("Model %s: one of %s" % (os.path.basename(args.model),
                                   ", ".join(str(c) for c in bundle["classes"])))

    if args.mat:
        matches = sorted(glob.glob(os.path.expanduser(args.mat)))
        if not matches:
            sys.exit("No file matches %s" % args.mat)
        path = matches[-1]
    else:
        path = tap_and_save(args)
    print("Taps saved in %s" % path)

    verdict, detail, used = classify(path, bundle)
    print()
    print("============ Prediction: %s (%s)" % (verdict, detail))
    if args.truth:
        import curvature_classifier as clf
        actual = clf.class_name(round(1.0 / args.truth, 6))
        print("============ Actually %s: %s" % (actual, "correct" if verdict == actual else "WRONG"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True,
                        help="a .joblib saved by ml/curvature_classifier.py")
    parser.add_argument("--mat", default=None,
                        help="classify this saved file instead of tapping; a "
                        "glob takes the newest match")
    parser.add_argument("--truth", type=float, default=None,
                        help="the sphere's real radius (m), to report right or "
                        "wrong. Never given to the model")
    parser.add_argument("--taps", type=int, default=1,
                        help="taps at the jogged point, each at its own yaw "
                        "within --yaw-span; with more than one the verdict "
                        "combines them")
    parser.add_argument("--start", default="here",
                        help="where the jog begins: 'here', or the radius (m) of "
                        "a sphere whose saved apex to start above, e.g. 0.025")
    parser.add_argument("--ml-dir", default=None,
                        help="folder holding curvature_classifier.py, if it is "
                        "not found beside src/")
    # Defaults are the four-sphere training run's settings.
    parser.add_argument("--tap-speed", type=float, default=0.010,
                        help="descent and retract speed (m/s)")
    parser.add_argument("--tap-force", type=float, default=2.5,
                        help="stop the descent at this |Fz| (N)")
    parser.add_argument("--tap-acc", type=float, default=0.1)
    parser.add_argument("--hover-height", type=float, default=0.003,
                        help="height above the jogged point each tap starts from (m)")
    parser.add_argument("--travel-height", type=float, default=0.020,
                        help="height above the point where the cup turns between taps (m)")
    parser.add_argument("--max-indent", type=float, default=0.006,
                        help="safety stop this far past contact (m)")
    parser.add_argument("--max-search", type=float, default=0.020,
                        help="safety stop this far below hover (m)")
    parser.add_argument("--contact-force", type=float, default=0.2,
                        help="|Fz| that counts as first contact (N)")
    parser.add_argument("--yaw-span", type=float, default=30.0)
    parser.add_argument("--yaw-start", type=float, default=-15.0)
    parser.add_argument("--clear-height", type=float, default=0.080,
                        help="height above the sphere to lift to before any "
                        "sideways move to or from it (m)")
    parser.add_argument("--jog-height", type=float, default=0.020,
                        help="height above the starting point the jog opens at (m)")
    parser.add_argument("--jog-step", type=float, default=0.001)
    parser.add_argument("--jog-coarse", type=float, default=0.010)
    parser.add_argument("--jog-range", type=float, default=0.30)
    parser.add_argument("--max-offset", type=float, default=0.10)
    parser.add_argument("--no-home", action="store_true")
    main(parser.parse_args())
