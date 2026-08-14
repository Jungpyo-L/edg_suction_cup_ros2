#!/usr/bin/env python3

# PARKED REFERENCE - NOT WIRED INTO ANYTHING, NOT INSTALLED BY CMakeLists.
#
# Contact-quality labelling and pull-off force measurement, written for
# sphere_sweep_experiment.py and then backed out of it. Kept here so it can be
# dropped back in without rewriting. Nothing imports this file.
#
# Three pieces, independent of each other:
#
#   1. prompt_label        - asks for a subjective good/bad/partial call at each
#                            waypoint, meant to be called while the cup is still
#                            down and the contact is visible.
#   2. SummaryWriter       - one CSV row per waypoint, flushed immediately so an
#                            aborted run keeps the waypoints it finished.
#   3. retract_measuring_  - steps the first few mm of the lift and returns the
#      pulloff               peak force resisting separation.
#
# To reinstate, see the "call sites" note at the bottom.

import csv
import os
import time
from datetime import datetime

import rclpy


LABEL_CHOICES = {"g": "good", "b": "bad", "p": "partial", "s": "skip"}


def prompt_label(waypoint):
    """Ask for a subjective contact-quality call while the cup is still down."""
    prompt = "  %s contact - [g]ood / [b]ad / [p]artial / [s]kip: " % waypoint
    while True:
        answer = input(prompt).strip().lower()
        if answer in LABEL_CHOICES:
            return LABEL_CHOICES[answer]
        # Accept the full word too, since it is easy to type it out by reflex.
        if answer in LABEL_CHOICES.values():
            return answer
        print("    Enter one of g, b, p, s.")


class SummaryWriter:
    """Appends one row per waypoint, flushed immediately.

    Written incrementally rather than at the end so an aborted run still leaves
    the waypoints it did complete.
    """

    FIELDS = ["waypoint", "mode", "x", "y", "contact_z", "final_z",
              "preload_mm", "fz_final", "pull_off_n", "label", "note"]

    def __init__(self, directory, mode):
        stamp = datetime.now().strftime("%y%m%d_%H%M%S")
        self.path = os.path.join(directory, "sphere_sweep_%s_%s.csv" % (mode, stamp))
        self.handle = open(self.path, "w", newline="")
        self.writer = csv.DictWriter(self.handle, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.handle.flush()

    def add(self, **row):
        self.writer.writerow({key: row.get(key, "") for key in self.FIELDS})
        self.handle.flush()

    def close(self):
        if self.handle is not None:
            self.handle.close()
            self.handle = None


def read_signed_fz(node, ft_help, n_spins=5, timeout_sec=0.05):
    """Spin briefly so the FT callback runs, then return Fz (bias-subtracted, N).

    The live script keeps only the magnitude version. The pull-off measurement
    needs the sign, to tell pressing from pulling.
    """
    for _ in range(n_spins):
        rclpy.spin_once(node, timeout_sec=timeout_sec)
    return float(getattr(ft_help, "averageFz_noOffset", 0.0))


def retract_measuring_pulloff(node, rtde_help, ft_help, xy, z_from, orientation,
                              press_sign, args):
    """Lift in small steps, returning the peak force resisting separation (N).

    press_sign is the sign Fz took while pressing, so the pull-off is read as
    force in the opposite direction. Taking it that way avoids depending on the
    sensor's z convention. Only the first few millimetres are stepped: the cup
    releases within that, and stepping the whole way to the travel plane would
    add a hundred moves per waypoint.

    Verified terminating: step=3e-4 over distance=0.005 gives 17 steps and lands
    exactly on z_from + distance.
    """
    x, y = xy
    peak = 0.0
    z = z_from
    z_end = z_from + args.pulloff_distance
    while z < z_end - 1e-9:
        z = min(z_end, z + args.retract_step)
        rtde_help.goToPose(
            rtde_help.getPoseObj([x, y, z], orientation),
            speed=args.retract_speed,
            acc=args.descend_acc,
        )
        time.sleep(args.settle)
        peak = max(peak, -press_sign * read_signed_fz(node, ft_help))
    return peak


# ---------------------------------------------------------------------------
# Call sites, as they were in sphere_sweep_experiment.py's main().
#
# Arguments that were added:
#
#   --no-label           store_true, skip the prompt
#   --no-pulloff         store_true, skip the stepped retract
#   --pulloff-distance   float, default 0.005, metres lifted while measuring
#   --retract-step       float, default 3e-4, metres per step during the lift
#   --retract-speed      float, default 0.005, m/s during the lift
#
# validate_args treated --pulloff-distance, --retract-step and --retract-speed
# as strictly positive, for the same non-termination reason as --descend-step.
#
# Created once, after the travel plane is computed and before the waypoint loop:
#
#   summary = SummaryWriter(file_help.ResultSavingDirectory, args.mode)
#   print("Per-waypoint summary: %s" % summary.path)
#
# Inside the waypoint loop, replacing the plain `time.sleep(args.dwell)` that
# precedes the lift back to the travel plane:
#
#   time.sleep(args.dwell)
#
#   label_value = "" if args.no_label else prompt_label(label)
#
#   pull_off = ""
#   if not offset_mode and not args.no_pulloff:
#       press_sign = 1.0 if read_signed_fz(node, ft_help) >= 0.0 else -1.0
#       pull_off = retract_measuring_pulloff(
#           node, rtde_help, ft_help, (touch_xyz[0], touch_xyz[1]),
#           z_final, orientation_fixed, press_sign, args,
#       )
#       print("  %-6s pull-off peak %.2f N" % (label, pull_off))
#
#   summary.add(
#       waypoint=label, mode=args.mode,
#       x="%.5f" % touch_xyz[0], y="%.5f" % touch_xyz[1],
#       contact_z="" if offset_mode else "%.5f" % z_contact,
#       final_z="%.5f" % (touch_xyz[2] if offset_mode else z_final),
#       preload_mm="" if offset_mode else "%.2f" % ((z_contact - z_final) * 1e3),
#       fz_final="" if offset_mode else "%.3f" % fz_final,
#       pull_off_n="" if pull_off == "" else "%.3f" % pull_off,
#       label=label_value,
#   )
#
# And after the loop, before disabling the data logger:
#
#   summary.close()
# ---------------------------------------------------------------------------
