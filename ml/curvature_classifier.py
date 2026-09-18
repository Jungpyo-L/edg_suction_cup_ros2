#!/usr/bin/env python3
"""Classify surface curvature from suction-cup taps.

Reads the .mat files saved by sphere_sweep_experiment.py --mode tap, cuts one
sample out of each tap, and trains a classifier whose label is the curvature
of the surface tapped: 1/R for a sphere of radius R, 0 for the flat plate. The
label is read from each run's own --radius, so nothing has to be annotated by
hand - but check it with --list first, because a sphere run given --offsets
instead of --radius records radius 0 and would be labelled flat.

    python3 ml/curvature_classifier.py --list
    python3 ml/curvature_classifier.py
    python3 ml/curvature_classifier.py --representation time

Every tap becomes an array of channels x points: the pressure change in each
chamber, the contact force, and the indentation past first contact. How the
points are placed is the --representation:

  force  (default) the channels read off at fixed force levels on the way up
         to the stop. Pressure at 1 N means the same thing however fast the
         cup got there, so tap speed is divided out.
  time   the channels on a fixed time grid around contact. Keeps speed in the
         signal, so a model can lean on it. Comparing the two representations
         is the check that accuracy comes from geometry rather than speed.

Accuracy is estimated leaving out one whole run at a time. Taps from one run
share a session, a cup condition and a mounting, so splitting taps at random
would test on near-copies of the training data and overstate the result.
"""

import argparse
import fnmatch
import glob
import os
import re
import sys

import numpy as np
from scipy.io import loadmat

# Phase codes published on /sync by sphere_sweep_experiment.py, as
# waypoint * 10 + event.
EVENT_DESCEND = 1
EVENT_CONTACT = 2
EVENT_STOP = 3

# Pre-contact samples are the baseline each channel is measured from. The last
# stretch before contact is left out of it, since the cup is already loading.
BASELINE_GUARD = 0.02
MIN_BASELINE_SAMPLES = 3


def read_table(mat, topic):
    """(column names, float array) for one logged topic, or None if absent.

    saveDataParams stores each CSV as <topic>_columnName and <topic>_data,
    where <topic> is the file's topic with its underscores removed.
    """
    if topic + "_data" not in mat or topic + "_columnName" not in mat:
        return None
    columns = [str(c).strip() for c in np.atleast_1d(mat[topic + "_columnName"])]
    data = np.atleast_2d(np.asarray(mat[topic + "_data"], dtype=object))
    if data.shape[1] != len(columns) and data.shape[0] == len(columns):
        data = data.T
    try:
        data = data.astype(float)
    except (TypeError, ValueError):
        numeric = np.full(data.shape, np.nan)
        for index, value in np.ndenumerate(data):
            try:
                numeric[index] = float(value)
            except (TypeError, ValueError):
                pass
        data = numeric
    if data.size == 0 or data.shape[1] != len(columns):
        return None
    order = np.argsort(data[:, 0], kind="stable")
    return columns, data[order]


def column(table, predicate, what):
    columns, data = table
    for index, name in enumerate(columns):
        if predicate(name):
            return data[:, index]
    raise KeyError("no %s column among %s" % (what, columns))


def load_run(path, channels):
    """Everything extract_taps needs from one run, or a reason it is unusable."""
    try:
        mat = loadmat(path, squeeze_me=True)
    except Exception as exc:
        return None, "unreadable (%s)" % exc

    mode = str(mat.get("mode", "")).strip()
    if mode != "tap":
        return None, "mode is %r, not tap" % mode
    radius = float(mat.get("radius", 0.0))

    sync = read_table(mat, "sync")
    ft = read_table(mat, "netftdata")
    pressure = read_table(mat, "SensorPacket")
    pose = read_table(mat, "endEffectorPose")
    missing = [name for name, table in (("/sync", sync), ("/netft_data", ft),
                                        ("/SensorPacket", pressure))
               if table is None]
    if missing:
        return None, "no %s logged" % ", ".join(missing)

    try:
        run = {
            "path": path,
            "radius": radius,
            "sync_t": sync[1][:, 0],
            "sync_code": np.rint(column(sync, lambda n: n != "ROStimestamp",
                                        "/sync")).astype(int),
            "ft_t": ft[1][:, 0],
            "fz": column(ft, lambda n: n.endswith("force.z"), "force z"),
            "p_t": pressure[1][:, 0],
            "pressure": [column(pressure,
                                lambda n, c=c: re.search(r"data\[%d\]$" % c, n),
                                "chamber %d" % c)
                         for c in channels],
        }
    except KeyError as exc:
        return None, str(exc)
    if pose is not None:
        try:
            run["z_t"] = pose[1][:, 0]
            run["z"] = column(pose, lambda n: n.endswith("position.z"), "z")
        except KeyError:
            pass
    return run, None


def first_time(run, code):
    hits = run["sync_t"][run["sync_code"] == code]
    return float(hits[0]) if hits.size else None


def extract_taps(run, args):
    """One (waypoint, array) per usable tap in the run, plus the reasons for
    any that were skipped."""
    samples, skipped = [], []
    waypoints = sorted({code // 10 for code in run["sync_code"]
                        if code % 10 == EVENT_DESCEND})
    has_z = "z" in run
    for waypoint in waypoints:
        if args.waypoints and waypoint not in args.waypoints:
            continue
        t_desc = first_time(run, waypoint * 10 + EVENT_DESCEND)
        t_contact = first_time(run, waypoint * 10 + EVENT_CONTACT)
        t_stop = first_time(run, waypoint * 10 + EVENT_STOP)
        if t_contact is None:
            skipped.append((waypoint, "no contact"))
            continue
        if t_stop is None:
            skipped.append((waypoint, "unfinished"))
            continue

        def baseline(t, values):
            inside = (t >= t_desc) & (t <= t_contact - BASELINE_GUARD)
            return float(np.median(values[inside])) if inside.sum() >= MIN_BASELINE_SAMPLES else None

        f_base = baseline(run["ft_t"], run["fz"])
        p_bases = [baseline(run["p_t"], p) for p in run["pressure"]]
        if f_base is None or any(b is None for b in p_bases):
            skipped.append((waypoint, "no pre-contact baseline - hover too low?"))
            continue

        force = run["fz"] - f_base
        # Compression reads negative or positive depending on how the sensor is
        # mounted. Orient it so the load at the stop is positive.
        at_stop = np.interp(t_stop, run["ft_t"], force)
        if at_stop < 0:
            force = -force
        pressures = [p - b for p, b in zip(run["pressure"], p_bases)]
        if has_z:
            depth = np.interp(t_contact, run["z_t"], run["z"]) - run["z"]

        if args.representation == "force":
            loading = (run["ft_t"] >= t_contact - 0.05) & (run["ft_t"] <= t_stop)
            t_load, f_load = run["ft_t"][loading], force[loading]
            if t_load.size < 3:
                skipped.append((waypoint, "too few force samples while loading"))
                continue
            # First time each force level was reached. The running maximum
            # makes force monotone so every level maps to one instant.
            f_mono = np.maximum.accumulate(f_load)
            if f_mono[-1] < args.force_max:
                skipped.append((waypoint, "peak %.2f N never reached %.2f N - "
                                          "stopped on a distance limit?"
                                % (f_mono[-1], args.force_max)))
                continue
            levels, first = np.unique(f_mono, return_index=True)
            grid = np.linspace(args.force_min, args.force_max, args.points)
            t_eval = np.interp(grid, levels, t_load[first])
        else:
            grid = np.linspace(-args.pre, args.post, args.points)
            # Clamped to the tap: past the stop the logged data is either cut
            # away or is the retract, and neither belongs to the touch.
            t_eval = np.clip(t_contact + grid, t_desc, t_stop)

        rows = [np.interp(t_eval, run["p_t"], p) for p in pressures]
        if args.representation == "time":
            rows.append(np.interp(t_eval, run["ft_t"], force))
        if has_z:
            rows.append(np.interp(t_eval, run["z_t"], depth))
        samples.append((waypoint, np.vstack(rows)))
    return samples, skipped


def channel_names(args, has_z):
    names = ["dP chamber %d" % c for c in args.channels]
    if args.representation == "time":
        names.append("force")
    if has_z:
        names.append("indentation")
    return names


def class_name(kappa):
    return "flat" if kappa == 0 else "R %.0f mm" % (1e3 / kappa)


def build_dataset(args):
    paths = sorted(set(glob.glob(os.path.join(os.path.expanduser(args.dir), "**",
                                              "DataLog_*Sphere_sweep_tap*.mat"),
                                 recursive=True)))
    for pattern in args.exclude:
        paths = [p for p in paths if not fnmatch.fnmatch(os.path.basename(p), pattern)]

    runs, report = [], []
    for path in paths:
        run, problem = load_run(path, args.channels)
        if run is None:
            report.append((path, None, 0, [], problem))
            continue
        samples, skipped = extract_taps(run, args)
        runs.append((run, samples))
        report.append((path, run["radius"], len(samples), skipped, None))

    # A channel has to exist in every run, or the arrays do not line up.
    has_z = bool(runs) and all("z" in run for run, _ in runs)
    X, y, groups, meta = [], [], [], []
    for group, (run, samples) in enumerate(runs):
        kappa = 1.0 / run["radius"] if run["radius"] > 0 else 0.0
        for waypoint, array in samples:
            if not has_z and "z" in run:
                array = array[:-1]
            X.append(array)
            y.append(round(kappa, 6))
            groups.append(group)
            meta.append((os.path.basename(run["path"]), waypoint))
    return (np.array(X), np.array(y), np.array(groups), meta, report,
            channel_names(args, has_z))


def print_report(report):
    print("%-58s %8s %5s  %s" % ("run", "label", "taps", "notes"))
    for path, radius, kept, skipped, problem in report:
        name = os.path.basename(path)
        if problem is not None:
            print("%-58s %8s %5s  skipped: %s" % (name[:58], "-", "-", problem))
            continue
        kappa = 1.0 / radius if radius > 0 else 0.0
        notes = "; ".join("wp%d %s" % item for item in skipped)
        print("%-58s %8s %5d  %s" % (name[:58], class_name(kappa), kept, notes))


def models(n_channels):
    from sklearn.ensemble import RandomForestClassifier
    from sklearn.linear_model import LogisticRegression, RidgeClassifierCV
    from sklearn.pipeline import make_pipeline
    from sklearn.preprocessing import StandardScaler
    from minirocket import MiniRocket
    return {
        "logreg": make_pipeline(StandardScaler(),
                                LogisticRegression(C=1.0, max_iter=5000)),
        "forest": RandomForestClassifier(n_estimators=500, random_state=0),
        # Ridge with its penalty chosen by internal cross-validation over the
        # same range as the MiniRocket paper.
        "minirocket": make_pipeline(MiniRocket(n_channels=n_channels),
                                    StandardScaler(),
                                    RidgeClassifierCV(alphas=np.logspace(-3, 3, 10))),
    }


def evaluate(name, model, features, labels, groups, classes):
    from sklearn.base import clone
    from sklearn.metrics import balanced_accuracy_score, confusion_matrix
    from sklearn.model_selection import LeaveOneGroupOut

    predicted = np.empty_like(labels)
    for train, test in LeaveOneGroupOut().split(features, labels, groups):
        if len(np.unique(labels[train])) < 2:
            # Holding this run out left a single class to train on, which
            # some models refuse to fit. Predicting that class is all any of
            # them could do.
            predicted[test] = labels[train][0]
            continue
        fitted = clone(model).fit(features[train], labels[train])
        predicted[test] = fitted.predict(features[test])

    accuracy = float(np.mean(predicted == labels))
    balanced = float(balanced_accuracy_score(labels, predicted))
    print()
    print("%s: accuracy %.1f%%, balanced accuracy %.1f%%, leaving one run out "
          "at a time" % (name, accuracy * 1e2, balanced * 1e2))
    matrix = confusion_matrix(labels, predicted, labels=classes)
    width = max(len(class_name(k)) for k in classes) + 2
    print(" " * width + "predicted ->")
    print(" " * width + "".join("%*s" % (width, class_name(k)) for k in classes))
    for kappa, row in zip(classes, matrix):
        print("%*s" % (width, class_name(kappa)) + "".join("%*d" % (width, n) for n in row))


def main(args):
    X, y, groups, meta, report, names = build_dataset(args)
    print_report(report)
    if args.list:
        return
    if X.size == 0:
        sys.exit("\nNo usable taps found under %s." % args.dir)

    classes = np.unique(y)
    print()
    print("%d taps from %d runs, %d classes; each tap is %d channels x %d "
          "points (%s): %s"
          % (len(y), len(np.unique(groups)), len(classes), X.shape[1], X.shape[2],
             args.representation, ", ".join(names)))
    print("%-10s %8s %6s %6s" % ("class", "1/R", "runs", "taps"))
    thin = []
    for kappa in classes:
        in_class = y == kappa
        n_runs = len(np.unique(groups[in_class]))
        print("%-10s %8.1f %6d %6d" % (class_name(kappa), kappa, n_runs, in_class.sum()))
        if n_runs < 2:
            thin.append(class_name(kappa))
    if len(classes) < 2:
        sys.exit("\nOnly one class - nothing to tell apart yet.")
    if thin:
        print()
        print("!! %s: only one run, so with it held out there is no training "
              "example of that class left and every one of its taps is "
              "predicted wrong by construction. Record at least two runs per "
              "class." % ", ".join(thin))

    features = X.reshape(len(X), -1)
    candidates = models(X.shape[1])
    for name, model in candidates.items():
        evaluate(name, model, features, y, groups, classes)

    # Fitted on every tap for use on new data. Its accuracy is the
    # leave-one-run-out figure above, not anything measured on this fit.
    os.makedirs(os.path.expanduser(args.out), exist_ok=True)
    # Named by representation and length, so a model always sits beside the
    # dataset it was trained on: rerunning with another --points would
    # otherwise replace the dataset under models built at the old length.
    out = os.path.join(os.path.expanduser(args.out), "curvature_%s_%dpt"
                       % (args.representation, args.points))
    np.savez(out + "_dataset.npz", X=X, y=y, groups=groups,
             channels=np.array(names), files=np.array([m[0] for m in meta]),
             waypoints=np.array([m[1] for m in meta]))
    import joblib
    final = candidates[args.model].fit(features, y)
    joblib.dump({"model": final, "classes": classes, "channels": names,
                 "config": vars(args)}, out + "_%s.joblib" % args.model)
    print()
    print("Dataset: %s_dataset.npz" % out)
    print("Model (%s, fitted on all %d taps): %s_%s.joblib"
          % (args.model, len(y), out, args.model))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dir", default="~/EDG_Experiment",
                        help="searched recursively for tap-mode .mat files")
    parser.add_argument("--exclude", nargs="*", default=[],
                        help="filename patterns to leave out, e.g. '*260918*'")
    parser.add_argument("--list", action="store_true",
                        help="show each run, its label and its taps, then stop")
    parser.add_argument("--representation", choices=["force", "time"],
                        default="force")
    parser.add_argument("--waypoints", type=int, nargs="*", default=[1],
                        help="which waypoints to use (default: 1, the centre). "
                        "The outer points always sit R/2 from the apex, which "
                        "puts them on a 30 degree slope on every sphere and on "
                        "none on the plate - include them and the model can "
                        "tell the plate apart by tilt, not curvature. Pass "
                        "nothing after the flag to use every waypoint")
    parser.add_argument("--channels", type=int, nargs="*", default=[0, 1, 2],
                        help="which /SensorPacket data indices are real chambers")
    parser.add_argument("--points", type=int, default=32,
                        help="points per channel")
    parser.add_argument("--force-min", type=float, default=0.3,
                        help="force representation: lowest level sampled (N)")
    parser.add_argument("--force-max", type=float, default=1.9,
                        help="force representation: highest level sampled (N). "
                        "Taps whose peak never reached it are skipped")
    parser.add_argument("--pre", type=float, default=0.1,
                        help="time representation: seconds before contact")
    parser.add_argument("--post", type=float, default=0.3,
                        help="time representation: seconds after contact")
    parser.add_argument("--model", choices=["logreg", "forest", "minirocket"],
                        default="logreg",
                        help="which of the evaluated models to save. MiniRocket "
                        "reads the shape of each curve and wants longer series "
                        "than the default --points 32 gives it room for; try "
                        "--points 128 alongside it")
    parser.add_argument("--out", default="~/EDG_Experiment/ml")
    main(parser.parse_args())
