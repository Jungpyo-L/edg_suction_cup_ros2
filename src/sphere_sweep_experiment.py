#!/usr/bin/env python3

# Sweeps the suction cup across a test sphere whose apex is detected from the
# RealSense point cloud by realsense_sphere_detector.py. Waypoints are given as
# offsets relative to the detected apex, so the same offsets work wherever the
# sphere is placed in the workspace.

import argparse
import math
import os
import signal
import sys
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int8

from suction_cup.srv import Enable
from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.ros2_helpers import call_enable_service
from helperFunction.rtde_helper import rtdeHelp

from scipy.spatial.transform import Rotation as R

# Fixed tool rotation for the whole sweep, same value used by simple_experiment.py.
ROTVEC_DEFAULT = [2.263, -2.179, 0.000]  # rad

# Parked pose the arm returns to when the sweep finishes, recorded from the
# robot in base. Its orientation is within 0.4 deg of ROTVEC_DEFAULT, so the
# return is effectively a pure translation.
HOME_POSITION = [0.558369, -0.051576, 0.404641]
HOME_QUAT = [-0.722349, 0.691528, 0.000403, 0.001165]

# Where an accepted jog leaves its correction so the next run can reuse it.
# Under ~/.ros rather than beside the results, because it describes the camera
# setup rather than any one experiment, and it should survive clearing results.
APEX_OFFSET_FILE = os.path.expanduser("~/.ros/suction_cup_apex_offset.txt")


def write_apex_offset(offset):
    """Record a jog correction for later runs. Best effort: failing to save it
    costs a re-jog, which is not worth aborting a sweep over."""
    try:
        os.makedirs(os.path.dirname(APEX_OFFSET_FILE), exist_ok=True)
        with open(APEX_OFFSET_FILE, "w") as handle:
            handle.write("%.6f,%.6f,%.6f" % tuple(offset))
        print("Saved to %s. Reuse it with --apex-offset last." % APEX_OFFSET_FILE)
    except OSError as exc:
        print("Could not save the offset (%s). Pass it by hand next time." % exc)


def read_apex_offset():
    """The offset written by the last accepted jog."""
    try:
        with open(APEX_OFFSET_FILE) as handle:
            text = handle.read().strip()
    except OSError as exc:
        raise RuntimeError(
            "--apex-offset last, but no saved offset in %s (%s). Run once with "
            "--jog-apex first." % (APEX_OFFSET_FILE, exc)
        )
    return text


# Jog directions in base, the frame the UR controller works in. w and s run
# along x, a and d along y, r and f along z.
JOG_KEYS = {
    "w": (1.0, 0.0, 0.0), "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0), "d": (0.0, -1.0, 0.0),
    "r": (0.0, 0.0, 1.0), "f": (0.0, 0.0, -1.0),
}


def read_key():
    """One keypress, no Enter. Restores the terminal before returning so that
    everything printed around it behaves normally."""
    import termios
    import tty

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def jog_to_apex(rtde_help, apex, orientation, args):
    """Drive the cup onto the true apex by hand, starting from the vision guess.

    Returns the corrected apex. Only x and y are taken from the arm: the tool
    tip sits below the TCP by the whole tool length, which is not measured, so
    the arm cannot report the apex height. z is left as vision found it, which
    costs nothing in force mode because the descent searches for the surface.

    The point of this is that the vision error is a fixed offset, not noise -
    it comes from the camera's calibration and its viewing angle, so it barely
    changes between runs. Correct it once here and the printed --apex-offset
    replays it on later runs without jogging again.
    """
    start = np.asarray(apex, dtype=float)
    target = start + np.array([0.0, 0.0, args.hover_height])
    step = args.jog_step

    print()
    print("Jogging to the apex. The cup is %.0f mm above the vision estimate."
          % (args.hover_height * 1e3))
    print("  w/s  +x/-x     a/d  +y/-y     r/f  +z/-z")
    print("  [ ]  step down/up (now %.1f mm)" % (step * 1e3))
    print("  <Enter> accept, x abort")
    print("Lateral moves happen at whatever height you are at, so lift before")
    print("crossing the sphere if you have come down onto it.")

    rtde_help.goToPose(rtde_help.getPoseObj(list(target), orientation))

    while True:
        key = read_key()

        if key in ("\r", "\n"):
            pose = rtde_help.rtde_r.getActualTCPPose()
            corrected = np.array([pose[0], pose[1], start[2]])
            delta = corrected - start
            print()
            print("Accepted apex x=%.5f y=%.5f z=%.5f" % tuple(corrected))
            print("Jogged dx=%+.1f mm dy=%+.1f mm from where it started."
                  % (delta[0] * 1e3, delta[1] * 1e3))
            return corrected

        if key in ("x", "\x03"):
            raise KeyboardInterrupt

        if key == "[":
            step = max(0.0001, step / 2.0)
            print("  step %.1f mm" % (step * 1e3))
            continue
        if key == "]":
            step = min(0.010, step * 2.0)
            print("  step %.1f mm" % (step * 1e3))
            continue

        direction = JOG_KEYS.get(key.lower())
        if direction is None:
            continue

        moved = target + np.asarray(direction) * step
        lateral = float(np.linalg.norm((moved - start)[:2]))
        if lateral > args.max_offset:
            print("  refusing to jog more than %.0f mm sideways from the vision "
                  "estimate; it is %.0f mm out already."
                  % (args.max_offset * 1e3, lateral * 1e3))
            continue

        target = moved
        rtde_help.goToPose(rtde_help.getPoseObj(list(target), orientation),
                           speed=0.02, acc=0.1)
        offset = target - start
        print("  x=%.5f y=%.5f z=%.5f   offset dx=%+.1f dy=%+.1f dz=%+.1f mm"
              % (target[0], target[1], target[2],
                 offset[0] * 1e3, offset[1] * 1e3, offset[2] * 1e3))


def retreat_to_home(rtde_help, travel_z, orientation, go_home=True):
    """Bring the tool somewhere safe after an interrupted run.

    Straight up to the travel plane first, never sideways: an aborted run can
    leave the cup preloaded against the sphere, and a lateral move from there
    drags the lip across the surface. Only once it is clear does it cross to the
    parked pose.

    Every step is guarded on its own. A retreat that fails halfway is still
    better than one that gives up at the first exception, and the reason is
    printed rather than swallowed so a stuck robot is visible.
    """
    try:
        # Cancel whatever moveL was in flight, otherwise the next command queues
        # behind a motion that is still running toward the old target.
        rtde_help.rtde_c.stopL(1.0)
        time.sleep(0.2)
    except Exception as exc:
        print("  could not stop the current move: %s" % exc)

    try:
        pose = rtde_help.rtde_r.getActualTCPPose()
        # travel_z is unknown if the interrupt landed before the apex was read,
        # in which case the tool has not descended anywhere and a small nominal
        # lift is enough to clear the surface.
        lift_z = pose[2] + 0.05 if travel_z is None else max(travel_z, pose[2])
        print("  lifting straight up to z=%.4f" % lift_z)
        rtde_help.goToPose(
            rtde_help.getPoseObj([pose[0], pose[1], lift_z], orientation)
        )
    except Exception as exc:
        print("  lift failed: %s" % exc)
        return

    if not go_home:
        print("  staying at the travel plane (--no-home).")
        return

    try:
        print("  returning to the parked pose %s." % np.round(HOME_POSITION, 4))
        rtde_help.goToPose(rtde_help.getPoseObj(HOME_POSITION, HOME_QUAT))
    except Exception as exc:
        print("  return to home failed: %s" % exc)


# Phase codes published on /sync so the continuous logs can be cut into phases
# afterwards. The published value is waypoint_index * 10 + event, so a reader
# recovers both with divmod(code, 10): code 21 is waypoint 2, event 1.
#
# Event 0 means the tool is travelling and the sample should be ignored. The
# others bound the phases of one probe. Int8 caps the value at 127, hence the
# waypoint limit in validate_args.
EVENT_TRAVEL = 0
EVENT_DESCEND = 1
EVENT_CONTACT = 2
EVENT_PRELOAD = 3
EVENT_DWELL_END = 4
# Emitted once per completed step, so every step boundary is its own row in the
# sync log. The code repeats rather than counting: the step index is the row's
# position within the waypoint, which keeps the value inside Int8 no matter how
# many steps a descent takes.
EVENT_DESCEND_STEP = 5
EVENT_PRELOAD_STEP = 6
MAX_WAYPOINTS = 12

# Cap on retained apex samples. The listener keeps receiving for the whole run,
# since read_avg_fz spins the node on every descent step, so an unbounded list
# would grow for as long as the experiment lasts.
APEX_BUFFER = 5000


def parse_offsets(text):
    """Parse "dx,dy,dz;dx,dy,dz" into a list of (dx, dy, dz) tuples in meters."""
    offsets = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [float(value) for value in chunk.split(",")]
        if len(parts) != 3:
            raise ValueError("Each waypoint needs exactly three values: dx,dy,dz")
        offsets.append(tuple(parts))
    if not offsets:
        raise ValueError("No waypoints given")
    return offsets


def compass_offsets(radius):
    """The apex itself plus north/west/south/east waypoints at radius/2 from it.

    North is +y and east is +x in the target frame, i.e. the compass is read
    looking down at the sphere from above. dz is zero: every waypoint keeps the
    apex height, which leaves the cup clear of the sphere since the surface
    curves away below the apex.
    """
    step = radius / 2.0
    return (
        ["center", "north", "west", "south", "east"],
        [
            (0.0, 0.0, 0.0),
            (0.0, step, 0.0),
            (-step, 0.0, 0.0),
            (0.0, -step, 0.0),
            (step, 0.0, 0.0),
        ],
    )


class ApexListener:
    """Collects sphere_apex messages so the sweep can start from a settled estimate."""

    def __init__(self, node, topic="sphere_apex"):
        self.node = node
        self.samples = deque(maxlen=APEX_BUFFER)
        # Whatever frame the detector is publishing in; reported rather than
        # assumed, since the detector's target_frame is a runtime parameter.
        self.frame_id = "?"
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = node.create_subscription(PointStamped, topic, self.callback, qos)

    def callback(self, msg):
        self.frame_id = msg.header.frame_id
        self.samples.append([msg.point.x, msg.point.y, msg.point.z])

    def wait_for_apex(self, num_samples=20, timeout_sec=20.0):
        """Average the next num_samples apex estimates. Raises on timeout."""
        self.samples.clear()
        deadline = time.time() + timeout_sec
        while len(self.samples) < num_samples:
            if time.time() > deadline:
                raise RuntimeError(
                    "Only received %d sphere_apex messages in %.0f s. Check that "
                    "realsense_sphere_detector.py is running and that the crop box "
                    "actually contains the sphere." % (len(self.samples), timeout_sec)
                )
            rclpy.spin_once(self.node, timeout_sec=0.1)

        samples = np.asarray(self.samples)
        spread = samples.max(axis=0) - samples.min(axis=0)
        self.node.get_logger().info("Apex sample spread (m): %s" % np.round(spread, 4))
        return samples.mean(axis=0)


def surface_drop(radius, dx, dy):
    """How far the sphere surface sits below the apex at horizontal offset (dx, dy).

    Zero when no radius is given, so offset-mode waypoints keep the old behavior
    of staying at apex height.
    """
    if radius <= 0.0:
        return 0.0
    d = math.hypot(dx, dy)
    if d >= radius:
        raise ValueError(
            "Waypoint is %.4f m from the apex, at or beyond the sphere radius "
            "%.4f m - there is no surface there." % (d, radius)
        )
    return radius - math.sqrt(radius * radius - d * d)


def read_avg_fz(node, ft_help, n_spins=5, timeout_sec=0.05):
    """Spin briefly so the FT callback runs, then return |Fz| (bias-subtracted, N)."""
    for _ in range(n_spins):
        rclpy.spin_once(node, timeout_sec=timeout_sec)
    return abs(getattr(ft_help, "averageFz_noOffset", 0.0))


def bias_ft_sensor(node, ft_help, n_spins=60, timeout_sec=0.05):
    """Fill the FT averaging buffer off the surface, then zero the sensor.

    Waits until FT data is actually flowing before biasing: setNowAsBias reads
    averageTx/Ty/Tz, which only exist after the first averaged callback, so a
    silent /netft_data would otherwise raise AttributeError.
    """
    for _ in range(n_spins):
        rclpy.spin_once(node, timeout_sec=timeout_sec)
        if getattr(ft_help, "startAverage", False):
            break
    if not getattr(ft_help, "startAverage", False):
        raise RuntimeError(
            "No /netft_data received - cannot bias the FT sensor. Check that "
            "netft_node is running and the ATI sensor IP is reachable."
        )
    ft_help.setNowAsBias()


def descend_to_contact(node, rtde_help, ft_help, xy, z_start, orientation, args,
                       on_event=None):
    """Step down from z_start until |Fz| crosses the contact threshold, then a
    further preload_depth. Returns (z_contact, z_final, fz_final)."""
    x, y = xy

    def move_to(z):
        rtde_help.goToPose(
            rtde_help.getPoseObj([x, y, z], orientation),
            speed=args.descend_speed,
            acc=args.descend_acc,
        )
        time.sleep(args.settle)
        return read_avg_fz(node, ft_help)

    z = z_start
    fz = read_avg_fz(node, ft_help)
    z_contact = None
    while (z_start - z) < args.max_search:
        if fz >= args.contact_force:
            z_contact = z
            break
        z -= args.descend_step
        fz = move_to(z)
        if on_event is not None:
            on_event(EVENT_DESCEND_STEP)
    # The loop re-tests its distance bound before its force check, so a contact
    # made on the very last step would otherwise be thrown away.
    if z_contact is None and fz >= args.contact_force:
        z_contact = z
    if z_contact is None:
        raise RuntimeError(
            "Searched %.1f mm below the hover pose without reaching the %.2f N "
            "contact threshold (|Fz|=%.2f N at z=%.4f). The predicted surface is "
            "wrong, or the threshold is below the sensor noise floor."
            % (args.max_search * 1e3, args.contact_force, fz, z)
        )
    # Marked here rather than re-derived in analysis, so the recorded instant is
    # the one the robot actually acted on.
    if on_event is not None:
        on_event(EVENT_CONTACT)
    node.get_logger().info("contact at z=%.4f m, |Fz|=%.2f N" % (z_contact, fz))

    # Preload past contact. Stepped rather than a single move so the force
    # ceiling can stop it partway if the cup loads faster than expected.
    target = z_contact - args.preload_depth
    while z > target + 1e-9:
        z = max(target, z - args.descend_step)
        fz = move_to(z)
        if on_event is not None:
            on_event(EVENT_PRELOAD_STEP)
        if fz >= args.force_ceiling:
            node.get_logger().warn(
                "Force ceiling %.1f N reached at z=%.4f m, %.1f mm into a %.1f mm "
                "preload; stopping here."
                % (args.force_ceiling, z, (z_contact - z) * 1e3,
                   args.preload_depth * 1e3)
            )
            break
    if on_event is not None:
        on_event(EVENT_PRELOAD)
    return z_contact, z, fz


def validate_args(args):
    """Reject argument values that would make the descent loops non-terminating.

    Both loops in descend_to_contact advance by descend_step and stop on a
    distance bound, so a zero or negative step never satisfies its exit
    condition: the search loop would issue moveL commands forever, and a
    negative one would drive the tool upward without limit. A zero speed makes
    moveL itself block indefinitely, since it returns on motion completion.
    """
    positive = {
        "--descend-step": args.descend_step,
        "--max-search": args.max_search,
        "--contact-force": args.contact_force,
        "--descend-speed": args.descend_speed,
        "--descend-acc": args.descend_acc,
        "--apex-samples": args.apex_samples,
        "--apex-timeout": args.apex_timeout,
    }
    for name, value in positive.items():
        if value <= 0:
            raise ValueError("%s must be greater than zero, got %s" % (name, value))

    non_negative = {
        "--radius": args.radius,
        "--standoff": args.standoff,
        "--preload-depth": args.preload_depth,
        "--hover-height": args.hover_height,
        "--dwell": args.dwell,
        "--settle": args.settle,
        "--travel-height": args.travel_height,
    }
    for name, value in non_negative.items():
        if value < 0:
            raise ValueError("%s cannot be negative, got %s" % (name, value))

    # The travel plane has to clear whatever the waypoints themselves sit at, or
    # "lifting" to it would drive the tool down into the sphere.
    lowest_clearance = args.standoff if args.mode == "offset" else args.hover_height
    if args.travel_height <= lowest_clearance:
        raise ValueError(
            "--travel-height (%.3f m) must exceed %s (%.3f m), otherwise moving to "
            "the travel plane lowers the tool instead of raising it."
            % (args.travel_height,
               "--standoff" if args.mode == "offset" else "--hover-height",
               lowest_clearance)
        )

    if args.force_ceiling <= args.contact_force:
        raise ValueError(
            "--force-ceiling (%.2f N) must exceed --contact-force (%.2f N), or the "
            "preload aborts on the same reading that triggered contact."
            % (args.force_ceiling, args.contact_force)
        )
    count = 5 if args.radius > 0.0 else len(parse_offsets(args.offsets))
    if count > MAX_WAYPOINTS:
        raise ValueError(
            "%d waypoints exceeds the %d that fit in the Int8 /sync phase code "
            "(waypoint * 10 + event)." % (count, MAX_WAYPOINTS)
        )
    if args.apex_samples > APEX_BUFFER:
        raise ValueError(
            "--apex-samples cannot exceed %d, the listener's buffer size."
            % APEX_BUFFER
        )


def wait_for_data_logger(node, client):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the data_logging service...")


def main(args):
    np.set_printoptions(precision=4)
    if args.radius > 0.0:
        labels, offsets = compass_offsets(args.radius)
        print(
            "Sphere radius %.4f m -> five waypoints: the apex, then %.4f m N, W, S, E "
            "of it. --offsets is ignored." % (args.radius, args.radius / 2.0)
        )
    else:
        offsets = parse_offsets(args.offsets)
        labels = ["waypoint %d" % i for i in range(1, len(offsets) + 1)]

    validate_args(args)
    offset_mode = args.mode == "offset"
    if offset_mode:
        print("Offset mode: holding %.1f mm above the apex plane, no contact."
              % (args.standoff * 1e3))
    else:
        print("Force mode: descending to %.2f N contact, then %.1f mm preload."
              % (args.contact_force, args.preload_depth * 1e3))

    rclpy.init()
    node = rclpy.create_node("sphere_sweep_experiment")
    ft_help = file_help = rtde_help = None
    sync_pub = data_logger_client = None
    # Held outside the try so an interrupt anywhere below still knows where the
    # safe plane is and which way the tool is pointing.
    travel_z = None
    orientation_fixed = R.from_rotvec(ROTVEC_DEFAULT).as_quat()
    try:
        apex_listener = ApexListener(node, args.apex_topic)

        # A dry run only needs the apex and some arithmetic. Skipping the rest
        # matters for more than speed: rtdeHelp opens an RTDEControlInterface,
        # which uploads a control program to the UR and blocks for minutes while
        # it contends with ur_control.launch.py for the controller's program
        # slot. Skipping it also lets a dry run work with just the camera and
        # detector, no robot or suction stack.
        if not args.dry_run:
            ft_help = FT_CallbackHelp(node)
            time.sleep(0.5)
            file_help = fileSaveHelp()
            time.sleep(0.5)
            rtde_help = rtdeHelp(125, node=node)
            time.sleep(0.5)

            sync_pub = node.create_publisher(Int8, "sync", 1)
            data_logger_client = node.create_client(Enable, "data_logging")
            wait_for_data_logger(node, data_logger_client)
            call_enable_service(node, data_logger_client, False)
            time.sleep(1)
            file_help.clearTmpFolder()

        if args.apex is not None:
            # Vision bypassed entirely. Some targets return no depth at all -
            # a smooth, glossy, dark sphere gives the stereo matcher nothing to
            # match, and the IR projector reflects off it rather than scattering
            # back - so the detector never sees them however the crop is set.
            apex = np.asarray(parse_offsets(args.apex)[0], dtype=float)
            apex_frame = "manual"
            print("Using the apex given on the command line: x=%.5f y=%.5f z=%.5f"
                  % tuple(apex))
            print("Vision is not consulted, so realsense_sphere_detector.py does "
                  "not need to be running.")
        else:
            print("Waiting for a settled sphere apex estimate...")
            apex = apex_listener.wait_for_apex(args.apex_samples, args.apex_timeout)
            apex_frame = apex_listener.frame_id
            print("Sphere apex (%s): x=%.5f y=%.5f z=%.5f"
                  % ((apex_frame,) + tuple(apex)))

        apex_vision = np.asarray(apex, dtype=float)
        args.apex_source = apex_frame

        offset_text = args.apex_offset.strip()
        if args.apex is not None and offset_text not in ("", "0,0,0"):
            # The saved offset corrects the camera, and there is no camera in
            # this path. Applying it to a hand-entered apex would move the arm
            # away from the number that was typed.
            print("Ignoring --apex-offset: it corrects the vision estimate, and "
                  "--apex replaces it.")
            offset_text = "0,0,0"
        if offset_text == "last":
            offset_text = read_apex_offset()
            print("Loaded saved apex offset %s from %s"
                  % (offset_text, APEX_OFFSET_FILE))
        apex_offset = np.asarray(parse_offsets(offset_text)[0], dtype=float)
        if np.any(apex_offset):
            apex = apex_vision + apex_offset
            print("Applied apex offset %s -> x=%.5f y=%.5f z=%.5f"
                  % (np.round(apex_offset, 5), apex[0], apex[1], apex[2]))

        if args.jog_apex:
            if args.dry_run:
                print("--jog-apex needs the robot, and --dry-run does not start "
                      "it. Skipping the jog.")
            else:
                apex = jog_to_apex(rtde_help, apex, orientation_fixed, args)
                # Save the total correction from the vision estimate, not just
                # the part added by hand, so --apex-offset last reproduces this
                # run's apex from a fresh vision reading.
                write_apex_offset(np.asarray(apex, dtype=float) - apex_vision)

        # Recorded onto args so they land in the .mat alongside everything else:
        # a sweep is not interpretable later without knowing which apex it
        # actually probed, and a jogged apex exists nowhere else.
        args.apex_vision = [float(v) for v in apex_vision]
        args.apex_used = [float(v) for v in np.asarray(apex, dtype=float)]
        args.apex_correction = [float(v) for v in
                                (np.asarray(apex, dtype=float) - apex_vision)]


        waypoints = []
        for label, (dx, dy, dz) in zip(labels, offsets):
            # In descend mode this is the predicted surface height, used only to
            # place the hover pose; the real touch z comes from the FT sensor.
            if offset_mode:
                dz_final = dz + args.standoff
            else:
                dz_final = dz - surface_drop(args.radius, dx, dy)
            touch_xyz = apex + np.array([dx, dy, dz_final])
            # Tolerance so a waypoint sitting exactly on the limit is not rejected
            # by floating-point error in the norm.
            if np.linalg.norm(touch_xyz - apex) > args.max_offset + 1e-9:
                raise ValueError(
                    "Waypoint %s (%.3f, %.3f, %.3f) is %.3f m from the apex, beyond "
                    "the --max-offset safety limit of %.3f m."
                    % (label, dx, dy, dz, np.linalg.norm(touch_xyz - apex), args.max_offset)
                )
            waypoints.append(touch_xyz)

        if offset_mode:
            print("Planned poses (%s), all %.1f mm above the apex plane:"
                  % (apex_frame, args.standoff * 1e3))
        else:
            print("Predicted surface points (%s); actual touch z is found by the "
                  "FT sensor:" % apex_frame)
        for label, offset, touch_xyz in zip(labels, offsets, waypoints):
            print("  %-6s offset %s -> %s"
                  % (label, np.array(offset), np.round(touch_xyz, 5)))
        if args.dry_run:
            print("Dry run: no robot motion commanded.")
            return

        logging_response = call_enable_service(node, data_logger_client, True)
        if not logging_response.output_file_name.strip():
            raise RuntimeError(
                "Data logger did not create any CSV files. Check that topics in "
                "config/TopicsList.txt are currently published before recording."
            )

        try:
            ft_help.setNowAsBias()
            time.sleep(0.1)
        except Exception:
            print("set now as offset failed, but it is okay")

        # One plane above the sphere that every lateral move happens in, so the
        # tool never traverses near the surface. Fixed relative to the apex
        # rather than to each waypoint, so it clears the sphere's top too.
        travel_z = apex[2] + args.travel_height
        print("Travel plane at z=%.5f (%.1f mm above the apex)."
              % (travel_z, args.travel_height * 1e3))

        def gate(prompt):
            """Wait for <Enter>, or just announce the step when unattended."""
            if args.unattended:
                print("%s [unattended]" % prompt)
                return
            input(prompt)

        def publish_event(index, event):
            sync_pub.publish(Int8(data=index * 10 + event))
            rclpy.spin_once(node, timeout_sec=0.0)

        for index, (label, touch_xyz) in enumerate(zip(labels, waypoints), start=1):
            hover_xyz = touch_xyz + np.array([0.0, 0.0, args.hover_height])
            hover = rtde_help.getPoseObj(list(hover_xyz), orientation_fixed)
            travel = rtde_help.getPoseObj(
                [touch_xyz[0], touch_xyz[1], travel_z], orientation_fixed
            )

            print("--- %s ---" % label)
            gate("Press <Enter> to cycle to next hover pose")  # gates handle timing
            # Cross to this waypoint in the travel plane first, then come
            # straight down. The previous waypoint left the tool up here, so
            # this move is lateral only.
            publish_event(index, EVENT_TRAVEL)
            rtde_help.goToPose(travel)
            rtde_help.goToPose(hover)

            if offset_mode:
                gate("Press <Enter> to cycle to next touch pose")
                publish_event(index, EVENT_DESCEND)
                rtde_help.goToPose(
                    rtde_help.getPoseObj(list(touch_xyz), orientation_fixed)
                )
            else:
                # Bias while clear of the surface so the descent stops on real
                # contact rather than on any resting sensor offset.
                bias_ft_sensor(node, ft_help)
                gate("Press <Enter> to descend to contact")
                publish_event(index, EVENT_DESCEND)
                z_contact, z_final, fz_final = descend_to_contact(
                    node, rtde_help, ft_help, (touch_xyz[0], touch_xyz[1]),
                    hover_xyz[2], orientation_fixed, args,
                    on_event=lambda event, i=index: publish_event(i, event),
                )
                print("  %-6s contact z=%.5f, final z=%.5f (%.1f mm preload), "
                      "|Fz|=%.2f N, vision predicted z=%.5f"
                      % (label, z_contact, z_final, (z_contact - z_final) * 1e3,
                         fz_final, touch_xyz[2]))

            time.sleep(args.dwell)
            publish_event(index, EVENT_DWELL_END)
            # Lift straight back to the travel plane before the next waypoint,
            # so the cup is never in contact while moving laterally.
            rtde_help.goToPose(travel)
            time.sleep(0.1)

        call_enable_service(node, data_logger_client, False)
        time.sleep(0.2)

        if not args.no_home:
            # Safe as a single move: the last waypoint left the tool in the
            # travel plane, and home is well above it.
            print("Returning to the parked pose %s." % np.round(HOME_POSITION, 4))
            rtde_help.goToPose(rtde_help.getPoseObj(HOME_POSITION, HOME_QUAT))

        if args.radius > 0.0:
            plan_txt = "radius_%s" % args.radius
        else:
            plan_txt = "offsets_%s" % args.offsets.replace(";", "__").replace(",", "_")
        file_help.saveDataParams(
            args,
            appendTxt="Sphere_sweep_%s_%s" % (args.mode, plan_txt),
        )
        file_help.clearTmpFolder()
        print("============ Sphere sweep complete!")
    except KeyboardInterrupt:
        print()
        print("============ Interrupted. Recovering the arm; do not press Ctrl-C again.")
        # Deafen the process to further interrupts for the duration of the
        # retreat. A second Ctrl-C here would abort the recovery and leave the
        # cup pressed against the sphere, which is the exact situation this
        # handler exists to prevent.
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if data_logger_client is not None:
                try:
                    call_enable_service(node, data_logger_client, False)
                    time.sleep(0.2)
                except Exception as exc:
                    print("  could not stop the data logger: %s" % exc)

            if rtde_help is not None:
                retreat_to_home(rtde_help, travel_z, orientation_fixed,
                                go_home=not args.no_home)

            # Save whatever was recorded before the interrupt. A partial sweep
            # is still data, and it is the whole point of being able to stop
            # one part way through.
            if file_help is not None:
                try:
                    file_help.saveDataParams(
                        args, appendTxt="Sphere_sweep_%s_interrupted" % args.mode
                    )
                    file_help.clearTmpFolder()
                    print("  partial run saved.")
                except Exception as exc:
                    print("  nothing saved: %s" % exc)
        finally:
            signal.signal(signal.SIGINT, previous_handler)
        print("============ Recovered.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--offsets",
        type=str,
        default="-0.030,0,-0.004; 0,0,0; 0.030,0,-0.004",
        help="waypoints relative to the sphere apex as 'dx,dy,dz; dx,dy,dz' in meters "
        "(default: 30 mm left, apex, 30 mm right)",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=0.0,
        help="radius of curvature of the test sphere (m). When given, --offsets is "
        "ignored and five waypoints are generated: the apex itself, then radius/2 "
        "north, west, south and east of it, all at the apex height",
    )
    parser.add_argument("--hover-height", type=float, default=0.005,
                        help="hover height above each touch pose (m)")
    parser.add_argument("--dwell", type=float, default=1.0,
                        help="seconds to hold at each touch pose while the seal "
                        "settles and the pressure plateaus")
    parser.add_argument("--unattended", action="store_true",
                        help="do not wait for <Enter> between moves, so a run can "
                        "proceed without someone at the keyboard")
    parser.add_argument("--no-home", action="store_true",
                        help="stay at the travel plane instead of returning to "
                        "the parked pose when the sweep finishes")
    # 80 mm rather than something tighter because the clearance that matters is
    # at the cup lip, which sits below the TCP by the whole tool length.
    parser.add_argument("--travel-height", type=float, default=0.080,
                        help="height above the apex of the plane used for all "
                        "lateral moves between waypoints, and the height lifted "
                        "to after each probe (m)")
    parser.add_argument(
        "--mode",
        choices=["offset", "force"],
        default="offset",
        help="offset: move in x-y only, holding every waypoint --standoff above "
        "the apex plane, and never touch the sphere. force: descend at each "
        "waypoint until the FT sensor senses contact, then press "
        "--preload-depth further. Defaults to offset, the safe one",
    )
    parser.add_argument("--standoff", type=float, default=0.010,
                        help="height above the apex plane held in offset mode (m)")
    # 0.2 N is about 3x the idle noise measured on this Axia80 (0.063 N
    # peak-to-peak worst case over a minute, usually lower).
    parser.add_argument("--contact-force", type=float, default=0.2,
                        help="|Fz| that counts as first contact (N)")
    parser.add_argument("--preload-depth", type=float, default=0.003,
                        help="distance to descend past first contact (m)")
    parser.add_argument("--descend-step", type=float, default=3e-4,
                        help="downward z increment per step during descent (m)")
    parser.add_argument("--max-search", type=float, default=0.015,
                        help="max descent below hover before giving up (m)")
    parser.add_argument("--force-ceiling", type=float, default=8.0,
                        help="abort the preload if |Fz| reaches this (N)")
    parser.add_argument("--settle", type=float, default=0.05,
                        help="seconds to settle after each descent step")
    parser.add_argument("--descend-speed", type=float, default=0.01)
    parser.add_argument("--descend-acc", type=float, default=0.1)
    parser.add_argument("--max-offset", type=float, default=0.10,
                        help="reject waypoints farther than this from the apex (m)")
    parser.add_argument("--apex-topic", type=str, default="sphere_apex")
    parser.add_argument("--apex", type=str, default=None,
                        help="use this apex as 'x,y,z' in meters instead of "
                        "waiting for the detector. For spheres the camera cannot "
                        "see at all - dark, glossy or too small to return depth. "
                        "Combine with --jog-apex to correct a rough guess by hand")
    parser.add_argument("--jog-apex", action="store_true",
                        help="after reading the vision apex, hover above it and "
                        "let the keyboard drive the cup onto the real apex. Only "
                        "x and y are taken from the arm; the tool length is not "
                        "known, so z stays as vision found it")
    parser.add_argument("--jog-step", type=float, default=0.001,
                        help="starting jog increment (m), halved with [ and "
                        "doubled with ]")
    parser.add_argument("--apex-offset", type=str, default="0,0,0",
                        help="correction added to the vision apex as 'dx,dy,dz' "
                        "in meters, or the word 'last' to reuse the offset saved "
                        "by the most recent --jog-apex. A jog only has to be done "
                        "once per camera setup")
    parser.add_argument("--apex-samples", type=int, default=20,
                        help="apex messages averaged before planning")
    parser.add_argument("--apex-timeout", type=float, default=20.0)
    parser.add_argument("--dry-run", action="store_true",
                        help="print the planned poses without moving the robot")
    parser.add_argument("--author", type=str, default="EDG")
    main(parser.parse_args())
