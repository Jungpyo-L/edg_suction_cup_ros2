#!/usr/bin/env python3

# Plots the pressure traces from a sphere sweep run, using the /sync phase codes
# to label what was happening. Analysis tool - run it with python3 directly, it
# is not a ROS node and is not installed by CMakeLists.
#
#   python3 plot_sweep.py                  # panels, one per probe point
#   python3 plot_sweep.py --timeline       # one continuous trace instead
#   python3 plot_sweep.py --list           # show runs found
#
# By default it reads the most recent run left in /tmp by data_logger.py. After
# fileSaveHelp.saveDataParams has run, those files have been moved, so point at
# them with --dir /tmp/processed_csv.

import argparse
import glob
import os
import re

import matplotlib.pyplot as plt
import pandas as pd

# Must match the constants in src/sphere_sweep_experiment.py.
EVENT_NAMES = {0: "travel", 1: "descend", 2: "contact", 3: "preload done",
               4: "dwell end", 5: "descend step", 6: "preload step"}
STEP_EVENTS = (5, 6)
# Only meaningful for --radius runs, which always produce these five in this
# order. An --offsets run has arbitrary points, so it falls back to the index.
WAYPOINT_NAMES = {1: "center", 2: "north", 3: "west", 4: "south", 5: "east"}


def waypoint_label(index, total):
    if total == len(WAYPOINT_NAMES):
        return WAYPOINT_NAMES.get(index, "waypoint %d" % index)
    return "waypoint %d" % index

# Phases worth shading, as (start event, end event, colour, label).
PHASES = [
    (1, 2, "#fdd", "descent"),
    (2, 3, "#dfd", "preload"),
    (3, 4, "#ddf", "dwell"),
]


def find_runs(directory):
    """Group the logger's CSVs by run, newest first.

    Filenames look like dataLog_2026_0814_143312__SensorPacket.csv, so the run
    key is everything between the prefix and the topic name.
    """
    runs = {}
    for path in glob.glob(os.path.join(directory, "dataLog_*.csv")):
        match = re.match(r"dataLog_(\d{4}_\d{4}_\d{6})_(.+)\.csv$",
                         os.path.basename(path))
        if match:
            runs.setdefault(match.group(1), {})[match.group(2).lstrip("_")] = path
    return dict(sorted(runs.items(), reverse=True))


def load_topic(paths, topic):
    """Read one topic's CSV, renaming its timestamp column to `t`."""
    if topic not in paths:
        return None
    frame = pd.read_csv(paths[topic])
    if frame.empty:
        return None
    return frame.rename(columns={"ROStimestamp": "t"}).sort_values("t")


def pressure_columns(frame, channels):
    """Columns holding the per-chamber values, in channel order."""
    found = []
    for column in frame.columns:
        match = re.search(r"\.?_?data\[(\d+)\]$", column)
        if match and (channels is None or int(match.group(1)) in channels):
            found.append((int(match.group(1)), column))
    if not found:
        raise SystemExit(
            "No /SensorPacket.data[..] columns found. Columns present: %s"
            % list(frame.columns)
        )
    return [column for _, column in sorted(found)]


def tag(frame, sync):
    """Attach the most recent phase code to every row.

    The code is a step function, only changing when the experiment publishes, so
    a backward as-of join reproduces it exactly on any sampling rate.
    """
    code_column = [c for c in sync.columns if c.endswith(".data") or c.endswith("._data")][0]
    codes = sync[["t", code_column]].rename(columns={code_column: "code"})
    merged = pd.merge_asof(frame, codes, on="t", direction="backward")
    merged["code"] = merged["code"].fillna(-1).astype(int)
    # Codes are waypoint * 10 + event, so divmod recovers both.
    merged["waypoint"] = merged["code"] // 10
    merged["event"] = merged["code"] % 10
    merged.loc[merged["code"] < 0, ["waypoint", "event"]] = -1
    return merged


def event_times(sync, code_column):
    """Map (waypoint, event) -> timestamp of the first time it was published.

    Step events repeat, so only their first occurrence lands here; use
    step_times for the full sequence.
    """
    times = {}
    for _, row in sync.iterrows():
        code = int(row[code_column])
        key = (code // 10, code % 10)
        times.setdefault(key, row["t"])
    return times


def step_times(sync, code_column, waypoint):
    """Every step-boundary timestamp for one waypoint, in order.

    Each row is one completed step, so the index in this list is the step
    number.
    """
    codes = sync[code_column].astype(int)
    rows = sync[(codes // 10 == waypoint) & (codes % 10).isin(STEP_EVENTS)]
    return rows["t"].tolist()


def shade_phases(axis, times, waypoint, t0):
    for start_event, end_event, colour, name in PHASES:
        start = times.get((waypoint, start_event))
        end = times.get((waypoint, end_event))
        if start is None or end is None:
            continue
        axis.axvspan(start - t0, end - t0, color=colour, alpha=0.6,
                     label=name, zorder=0)
    contact = times.get((waypoint, 2))
    if contact is not None:
        axis.axvline(contact - t0, color="k", lw=1.0, ls="--", zorder=3)


def plot_panels(pressure, columns, sync, code_column, args):
    """One panel per probe point, each with time measured from its contact."""
    times = event_times(sync, code_column)
    waypoints = sorted(w for w in pressure["waypoint"].unique() if w > 0)
    if not waypoints:
        raise SystemExit("No waypoint codes in the sync log - was this an old run?")

    figure, axes = plt.subplots(len(waypoints), 1, sharey=True,
                                figsize=(9, 2.4 * len(waypoints)))
    if len(waypoints) == 1:
        axes = [axes]

    for axis, waypoint in zip(axes, waypoints):
        rows = pressure[pressure["waypoint"] == waypoint]
        # Zero on contact so the panels line up; fall back to the descent start
        # when there was no contact, as in offset mode.
        t0 = times.get((waypoint, 2)) or times.get((waypoint, 1)) or rows["t"].iloc[0]
        shade_phases(axis, times, waypoint, t0)
        if args.steps:
            # Rug along the bottom: one tick per completed 0.3 mm step.
            for step_t in step_times(sync, code_column, waypoint):
                axis.axvline(step_t - t0, color="k", lw=0.4, alpha=0.25,
                             ymax=0.06, zorder=1)
        for index, column in enumerate(columns):
            axis.plot(rows["t"] - t0, rows[column], lw=1.0,
                      label="chamber %d" % index)
        axis.set_ylabel("pressure")
        axis.set_title("%s (waypoint %d)"
                       % (waypoint_label(waypoint, len(waypoints)), waypoint),
                       loc="left", fontsize=10)
        axis.grid(True, alpha=0.3)

    axes[-1].set_xlabel("time from contact (s)")
    # One legend for the figure: repeating it per panel wastes the space.
    handles, labels = axes[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    figure.legend(unique.values(), unique.keys(), loc="upper right", ncol=3)
    figure.suptitle("Chamber pressure per probe point", x=0.01, ha="left")
    figure.tight_layout()
    return figure


def plot_timeline(pressure, columns, sync, code_column, args):
    """The whole run as one continuous trace, with every phase shaded."""
    times = event_times(sync, code_column)
    t0 = pressure["t"].iloc[0]

    figure, axis = plt.subplots(figsize=(13, 5))
    for waypoint in sorted(w for w in pressure["waypoint"].unique() if w > 0):
        shade_phases(axis, times, waypoint, t0)
        start = times.get((waypoint, 1))
        if start is not None:
            axis.text(start - t0, 1.01, waypoint_label(waypoint, 0),
                      transform=axis.get_xaxis_transform(), fontsize=8, rotation=90)

    for index, column in enumerate(columns):
        axis.plot(pressure["t"] - t0, pressure[column], lw=0.9,
                  label="chamber %d" % index)

    axis.set_xlabel("time (s)")
    axis.set_ylabel("pressure")
    axis.grid(True, alpha=0.3)
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), loc="upper right", ncol=4)
    axis.set_title("Chamber pressure over the run")
    figure.tight_layout()
    return figure


def main(args):
    runs = find_runs(args.dir)
    if not runs:
        raise SystemExit("No dataLog_*.csv files in %s" % args.dir)

    if args.list:
        for key, paths in runs.items():
            print("%s  %s" % (key, ", ".join(sorted(paths))))
        return

    key = args.run or next(iter(runs))
    if key not in runs:
        raise SystemExit("Run %s not found. Use --list to see what is there." % key)
    paths = runs[key]
    print("Run %s: %s" % (key, ", ".join(sorted(paths))))

    pressure = load_topic(paths, "SensorPacket")
    sync = load_topic(paths, "sync")
    if pressure is None:
        raise SystemExit("No SensorPacket data in run %s." % key)
    if sync is None:
        raise SystemExit("No sync data in run %s - nothing to label with." % key)

    code_column = [c for c in sync.columns if c.endswith(".data") or c.endswith("._data")][0]
    columns = pressure_columns(pressure, args.channels)
    print("Plotting %d chambers: %s" % (len(columns), columns))

    tagged = tag(pressure, sync)
    if args.timeline:
        figure = plot_timeline(tagged, columns, sync, code_column, args)
    else:
        figure = plot_panels(tagged, columns, sync, code_column, args)

    if args.save:
        figure.savefig(args.save, dpi=150)
        print("Wrote %s" % args.save)
    else:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default="/tmp",
                        help="where the dataLog_*.csv files are")
    parser.add_argument("--run", default=None,
                        help="run timestamp, e.g. 2026_0814_143312 (default: newest)")
    parser.add_argument("--list", action="store_true",
                        help="list the runs found and exit")
    parser.add_argument("--channels", type=int, nargs="*", default=[0, 1, 2],
                        help="which SensorPacket data indices are real chambers")
    parser.add_argument("--steps", action="store_true",
                        help="mark every step boundary as a tick along the bottom")
    parser.add_argument("--timeline", action="store_true",
                        help="one continuous trace instead of per-point panels")
    parser.add_argument("--save", default=None,
                        help="write to this image file instead of showing a window")
    main(parser.parse_args())
