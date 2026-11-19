#!/usr/bin/env python3
"""Run several sphere sweeps one after another, unattended.

The queue is a text file with one sweep per line, written exactly as the
arguments to sphere_sweep_experiment.py. Blank lines and anything after a #
are ignored. For example:

    # jog once, then reuse the saved points for every later run
    --spheres 0.020,0.025,0.030,0.040 --mode tap --tap-speed 0.010 ... --jog-apex --jog-range 0.30
    --spheres 0.020,0.025,0.030,0.040 --mode tap --tap-speed 0.015 ...
    --spheres 0 --mode tap --tap-speed 0.010 ...

    ros2 run suction_cup sweep_queue.py ~/queue.txt
    ros2 run suction_cup sweep_queue.py ~/queue.txt --dry-run

Every line is parsed and checked before the robot moves, so a typo in the last
run is caught at the start, not hours in. Only the first run may jog
(--jog-apex needs someone at the keyboard); later runs reuse the saved points,
and every surface they tap must already be saved or jogged by an earlier run.
--unattended is added to any line without it.

Each run saves its own .mat as usual. A run that fails stops the queue (the
sweep has already lifted the cup and parked the arm); --keep-going carries on
with the next run instead. A run that finished and saved but crashed while
exiting counts as done. Ctrl-C lets the current sweep recover the arm, then
stops the queue. A log of every run, its exit status and the files it saved is
written to ~/EDG_Experiment/queue_<date>_<time>.log.
"""

import argparse
import copy
import glob
import os
import shlex
import subprocess
import sys
import time
from datetime import datetime

SWEEP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "sphere_sweep_experiment.py")
DATA_DIR = os.path.expanduser("~/EDG_Experiment")


def read_queue(path):
    """The argument list of each run, in order, with its line number."""
    runs = []
    with open(os.path.expanduser(path)) as handle:
        for number, line in enumerate(handle, 1):
            words = shlex.split(line, comments=True)
            if words:
                runs.append((number, words))
    return runs


def check_queue(runs):
    """Parse and validate every run, and check each one's surfaces will have a
    saved point by the time it starts. Raises ValueError naming the line."""
    import sphere_sweep_experiment as sweep

    parser = sweep.build_parser()
    jogged = set()
    for index, (number, words) in enumerate(runs):
        where = "queue line %d" % number
        if "--jog-apex" in words and index > 0:
            raise ValueError(
                "%s: only the first run may use --jog-apex - a jog waits at "
                "the keyboard, which would hold the queue until someone came "
                "back. Jog in the first run and let later runs reuse the "
                "saved points." % where)
        if "--dry-run" in words:
            raise ValueError("%s: --dry-run in a queue line; use "
                             "sweep_queue.py --dry-run instead." % where)
        if "--unattended" not in words:
            words.append("--unattended")
        try:
            args = parser.parse_args(words)
        except SystemExit:
            raise ValueError("%s: the arguments above were not accepted." % where)
        try:
            sweep.validate_args(args)
            print("\n[run %d, line %d]" % (index + 1, number))
            sweep.plan_offsets(copy.copy(args))
        except ValueError as exc:
            raise ValueError("%s: %s" % (where, exc))
        if not args.spheres:
            continue
        for surf in sweep.surfaces(sweep.parse_radii(args.spheres), args):
            if args.jog_apex:
                jogged.add(surf.key)
            elif surf.key not in jogged and sweep.stored_point(surf.key) is None:
                raise ValueError(
                    "%s: nothing saved for the %s and no earlier run jogs it. "
                    "Add --jog-apex --jog-range 0.30 to the first run."
                    % (where, surf.name))


def saved_files():
    return set(glob.glob(os.path.join(DATA_DIR, "*", "*.mat")))


def run_one(words, done_file):
    """Run one sweep to the end. Returns its exit code, or None when Ctrl-C
    stopped it (after letting it recover the arm). The sweep writes
    `done_file` once it has finished and saved."""
    env = dict(os.environ, SWEEP_DONE_FILE=done_file)
    child = subprocess.Popen([sys.executable, SWEEP] + words, env=env)
    try:
        return child.wait()
    except KeyboardInterrupt:
        # The sweep got the same Ctrl-C and is lifting the cup and parking the
        # arm. Wait for that to finish before the queue exits.
        print("\n[queue] Ctrl-C: waiting for the sweep to recover the arm ...")
        while True:
            try:
                child.wait()
                return None
            except KeyboardInterrupt:
                print("[queue] still recovering - please wait.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("queue", help="text file, one sweep's arguments per line")
    parser.add_argument("--dry-run", action="store_true",
                        help="check the queue and print each run's plan, then stop")
    parser.add_argument("--keep-going", action="store_true",
                        help="after a failed run, carry on with the next one")
    parser.add_argument("--pause", type=float, default=10.0,
                        help="seconds to wait between runs")
    args = parser.parse_args()
    # In order with the sweeps' own output, also when piped through tee.
    sys.stdout.reconfigure(line_buffering=True)

    runs = read_queue(args.queue)
    if not runs:
        sys.exit("No runs in %s." % args.queue)
    try:
        check_queue(runs)
    except ValueError as exc:
        sys.exit("\n[queue] Not started: %s" % exc)
    print("\n[queue] %d runs checked." % len(runs))
    if args.dry_run:
        return

    os.makedirs(DATA_DIR, exist_ok=True)
    log_path = os.path.join(DATA_DIR, "queue_%s.log" % datetime.now().strftime("%y%m%d_%H%M%S"))
    results = []
    with open(log_path, "w") as log:
        log.write("queue %s, %d runs\n" % (os.path.abspath(os.path.expanduser(args.queue)),
                                           len(runs)))
        for index, (number, words) in enumerate(runs):
            if index > 0:
                print("\n[queue] next run in %.0f s (Ctrl-C to stop the queue) ..."
                      % args.pause)
                try:
                    time.sleep(args.pause)
                except KeyboardInterrupt:
                    print("\n[queue] Stopped before run %d." % (index + 1))
                    break
            print("\n[queue] ===== run %d of %d (line %d) =====" % (index + 1, len(runs), number))
            before, started = saved_files(), time.monotonic()
            stamp = datetime.now().strftime("%H:%M:%S")
            done_file = log_path + ".run%d.done" % (index + 1)
            code = run_one(words, done_file)
            finished = os.path.exists(done_file)
            if finished:
                os.remove(done_file)
            minutes = (time.monotonic() - started) / 60.0
            new = sorted(saved_files() - before)
            if code is None:
                status = "stopped"
            elif code == 0:
                status = "ok"
            elif finished:
                # Finished and saved, then died on the way out: the data is
                # complete, so the queue carries on.
                status = "ok (finished and saved, then exited with %d)" % code
                code = 0
            else:
                status = "failed (exit %d)" % code
            results.append((index + 1, status, minutes, new))
            log.write("\nrun %d (line %d) started %s, %.1f min, %s\n  args: %s\n"
                      % (index + 1, number, stamp, minutes, status, " ".join(words)))
            for path in new:
                log.write("  saved: %s\n" % path)
            log.flush()
            if code is None:
                break
            if code != 0 and not args.keep_going:
                print("\n[queue] Run %d failed; stopping. --keep-going would carry on."
                      % (index + 1))
                break

    print("\n[queue] ===== summary =====")
    for run, status, minutes, new in results:
        print("run %d: %s, %.1f min" % (run, status, minutes))
        for path in new:
            print("    %s" % os.path.basename(path))
    skipped = len(runs) - len(results)
    if skipped:
        print("%d run(s) not started." % skipped)
    print("Log: %s" % log_path)


if __name__ == "__main__":
    main()
