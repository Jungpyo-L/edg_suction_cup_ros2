#!/usr/bin/env python3
"""Look at what a tap run actually recorded.

Two views, from the .mat files sphere_sweep_experiment.py --mode tap saves:

    python3 ml/inspect_run.py --run latest     one run: every tap, and whether
                                               the taps themselves behaved
    python3 ml/inspect_run.py                  all runs: each curvature's mean
                                               pressure curve against the others

The second is the question the whole experiment asks - do the curvatures
separate - so it is the default. Both print a text digest as well as writing a
PNG, since the numbers paste into a conversation and the picture does not
always travel.

Pressure is plotted against force rather than time: the cup reaches 1 N at a
different moment on every tap, and comparing curves at the same force is what
the classifier does too.
"""

import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from curvature_classifier import (EVENT_DESCEND, class_name, curvature_of,
                                  load_run, tap_signals)

# From the validated reference palette. Chambers are an identity, so they take
# categorical slots 1-3; curvature is a magnitude, so the classes take one hue
# stepped light to dark, in curvature order.
CHAMBER_COLOURS = ["#2a78d6", "#eb6834", "#1baf7a"]
CURVATURE_RAMP = ["#86b6ef", "#3987e5", "#1c5cab", "#0d366b"]
SURFACE = "#fcfcfb"
INK = "#1a1a19"
MUTED = "#6b6a63"


def tap_curves(run, channels):
    """One dict per tap, holding the whole curve rather than a fixed grid:
    force and each channel from contact to the stop, plus where the tap was
    and how it ended."""
    curves, skipped = [], []
    waypoints = sorted({c // 10 for c in run["sync_code"] if c % 10 == EVENT_DESCEND})
    for waypoint in waypoints:
        signals, problem = tap_signals(run, waypoint)
        if problem is not None:
            skipped.append((waypoint, problem))
            continue
        t_contact, t_stop = signals["t_contact"], signals["t_stop"]
        times = run["ft_t"]
        window = (times >= t_contact) & (times <= t_stop)
        if window.sum() < 3:
            skipped.append((waypoint, "too few force samples"))
            continue

        plan = run.get("plan")
        row = plan[waypoint - 1] if plan is not None and waypoint - 1 < len(plan) else None
        force = signals["force"][window]
        curve = {
            "waypoint": waypoint,
            "kappa": curvature_of(run, waypoint),
            "t": times[window] - t_contact,
            # Monotone, so each force level maps to one moment of the tap.
            "force": np.maximum.accumulate(force),
            "raw_force": force,
            "yaw": float(row[2]) if row is not None else float("nan"),
            "offset": float(np.hypot(*row[:2])) if row is not None else float("nan"),
            "duration": t_stop - t_contact,
            "peak": float(np.max(force)),
        }
        for index, pressure in enumerate(signals["pressures"]):
            curve["dp%d" % index] = np.interp(times[window], run["p_t"], pressure)
        if "depth" in signals:
            curve["depth"] = np.interp(times[window], run["z_t"], signals["depth"])
        curves.append(curve)
    return curves, skipped


def on_force_grid(curve, key, grid):
    """`key` read off at each force level, or NaN past where the tap stopped."""
    force, values = curve["force"], curve.get(key)
    if values is None:
        return np.full(grid.shape, np.nan)
    levels, first = np.unique(force, return_index=True)
    out = np.interp(grid, levels, values[first])
    return np.where(grid <= force[-1], out, np.nan)


def digest(path, curves, skipped, channels):
    print("%s" % os.path.basename(path))
    print("  %d taps kept%s"
          % (len(curves),
             ", %d skipped (%s)" % (len(skipped), "; ".join("wp%d %s" % s for s in skipped))
             if skipped else ""))
    # One run can cover several spheres bolted down together, and pooling their
    # numbers would hide the very difference being looked for.
    for kappa in sorted({c["kappa"] for c in curves}):
        digest_surface(kappa, [c for c in curves if c["kappa"] == kappa], channels)


def digest_surface(kappa, curves, channels):
    print("  %s, %d taps" % (class_name(kappa), len(curves)))
    peaks = np.array([c["peak"] for c in curves])
    durations = np.array([c["duration"] for c in curves]) * 1e3
    print("  peak force   %.2f N median, %.2f to %.2f" % (np.median(peaks), peaks.min(), peaks.max()))
    print("  contact to stop  %.0f ms median, %.0f to %.0f" % (np.median(durations), durations.min(), durations.max()))
    if "depth" in curves[0]:
        depths = np.array([c["depth"][-1] for c in curves]) * 1e3
        print("  indentation  %.2f mm median, %.2f to %.2f" % (np.median(depths), depths.min(), depths.max()))
    for index in range(len(channels)):
        finals = np.array([c["dp%d" % index][-1] for c in curves]) / 1e3
        print("  chamber %d dP at the stop  %+.2f kPa median, %+.2f to %+.2f"
              % (channels[index], np.median(finals), finals.min(), finals.max()))
    apex = [c for c in curves if c["offset"] == 0.0]
    if len(apex) >= 2:
        order = np.argsort([c["waypoint"] for c in apex])
        first_tap, last_tap = apex[order[0]], apex[order[-1]]
        drift = (last_tap["dp0"][-1] - first_tap["dp0"][-1]) / 1e3
        print("  drift across the run  chamber %d at the apex moved %+.2f kPa from "
              "the first apex tap to the last" % (channels[0], drift))


def style(axis):
    axis.set_facecolor(SURFACE)
    axis.grid(True, alpha=0.25, lw=0.6)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        axis.spines[side].set_color(MUTED)
    axis.tick_params(colors=MUTED, labelsize=8)


def plot_one_run(path, curves, channels, grid, args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 3, figsize=(13, 4.2), facecolor=SURFACE)
    surfaces = ", ".join(class_name(k) for k in sorted({c["kappa"] for c in curves}))

    axis = axes[0]
    for index in range(len(channels)):
        colour = CHAMBER_COLOURS[index % len(CHAMBER_COLOURS)]
        for curve in curves:
            axis.plot(curve["force"], curve["dp%d" % index] / 1e3, lw=0.8,
                      color=colour, alpha=0.35, solid_capstyle="round")
        stack = np.vstack([on_force_grid(c, "dp%d" % index, grid) for c in curves]) / 1e3
        mean = np.nanmean(stack, axis=0)
        axis.plot(grid, mean, lw=2.0, color=colour, solid_capstyle="round",
                  label="chamber %d" % channels[index])
        axis.annotate(" %d" % channels[index], (grid[-1], mean[-1]), color=colour,
                      fontsize=8, va="center")
    axis.set_xlim(grid[0], grid[-1] + 0.12 * (grid[-1] - grid[0]))
    axis.set_title("pressure change per chamber", loc="left", fontsize=10, color=INK)
    axis.set_xlabel("force (N)", fontsize=9, color=MUTED)
    axis.set_ylabel("dP (kPa)", fontsize=9, color=MUTED)
    axis.legend(frameon=False, fontsize=8, labelcolor=MUTED)
    style(axis)

    axis = axes[1]
    for curve in curves:
        axis.plot(curve["t"] * 1e3, curve["raw_force"], lw=0.8, color=CHAMBER_COLOURS[0],
                  alpha=0.35, solid_capstyle="round")
    axis.axhline(args.tap_force, color=MUTED, lw=1.0, ls="--")
    axis.annotate(" stop at %.1f N" % args.tap_force, (0, args.tap_force), color=MUTED,
                  fontsize=8, va="bottom")
    axis.set_title("force through each tap", loc="left", fontsize=10, color=INK)
    axis.set_xlabel("ms from contact", fontsize=9, color=MUTED)
    axis.set_ylabel("force (N)", fontsize=9, color=MUTED)
    style(axis)

    axis = axes[2]
    if "depth" in curves[0]:
        for curve in curves:
            axis.plot(curve["force"], curve["depth"] * 1e3, lw=0.8,
                      color=CHAMBER_COLOURS[2], alpha=0.35, solid_capstyle="round")
        axis.set_ylabel("indentation (mm)", fontsize=9, color=MUTED)
    axis.set_title("how far the cup pressed in", loc="left", fontsize=10, color=INK)
    axis.set_xlabel("force (N)", fontsize=9, color=MUTED)
    style(axis)

    figure.suptitle("%s - %s, %d taps" % (os.path.basename(path), surfaces, len(curves)),
                    x=0.01, ha="left", fontsize=11, color=INK)
    figure.tight_layout(rect=(0, 0, 1, 0.94))
    return figure


def plot_classes(by_class, channels, grid):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    panels = len(channels) + 1
    figure, axes = plt.subplots(1, panels, figsize=(3.6 * panels, 4.2), facecolor=SURFACE)
    # The chamber panels share a y axis: the whole point is to compare the same
    # pressure change between chambers and classes, which separate scales would
    # quietly undo. Indentation keeps its own, being a different quantity.
    for axis in axes[1:len(channels)]:
        axis.sharey(axes[0])
    classes = sorted(by_class)
    for slot, kappa in enumerate(classes):
        colour = CURVATURE_RAMP[slot % len(CURVATURE_RAMP)]
        curves = by_class[kappa]
        for index in range(len(channels)):
            stack = np.vstack([on_force_grid(c, "dp%d" % index, grid) for c in curves]) / 1e3
            mean, spread = np.nanmean(stack, axis=0), np.nanstd(stack, axis=0)
            axis = axes[index]
            axis.fill_between(grid, mean - spread, mean + spread, color=colour,
                              alpha=0.15, lw=0)
            axis.plot(grid, mean, lw=2.0, color=colour, solid_capstyle="round",
                      label="%s (%d taps)" % (class_name(kappa), len(curves)))
            if index == len(channels) - 1:
                axis.annotate(" " + class_name(kappa), (grid[-1], mean[-1]),
                              color=colour, fontsize=8, va="center")
        stack = np.vstack([on_force_grid(c, "depth", grid) for c in curves]) * 1e3
        axes[-1].plot(grid, np.nanmean(stack, axis=0), lw=2.0, color=colour,
                      solid_capstyle="round")
        axes[-1].annotate(" " + class_name(kappa), (grid[-1], np.nanmean(stack, axis=0)[-1]),
                          color=colour, fontsize=8, va="center")

    for index in range(len(channels)):
        axes[index].set_title("chamber %d" % channels[index], loc="left", fontsize=10, color=INK)
        axes[index].set_xlabel("force (N)", fontsize=9, color=MUTED)
        axes[index].set_ylabel("dP (kPa)", fontsize=9, color=MUTED)
        style(axes[index])
    # Room at the right for the direct labels, so they sit inside the panel
    # rather than overhanging the next one.
    for axis in axes:
        axis.set_xlim(grid[0], grid[-1] + 0.18 * (grid[-1] - grid[0]))
    axes[-1].set_title("indentation", loc="left", fontsize=10, color=INK)
    axes[-1].set_xlabel("force (N)", fontsize=9, color=MUTED)
    axes[-1].set_ylabel("mm", fontsize=9, color=MUTED)
    style(axes[-1])
    axes[0].legend(frameon=False, fontsize=8, labelcolor=MUTED)
    figure.suptitle("Mean of each surface, shaded one standard deviation",
                    x=0.01, ha="left", fontsize=11, color=INK)
    figure.tight_layout(rect=(0, 0, 1, 0.94))
    return figure


def main(args):
    paths = sorted(glob.glob(os.path.join(os.path.expanduser(args.dir), "**",
                                          "DataLog_*Sphere_sweep_tap*.mat"),
                             recursive=True))
    if args.run:
        paths = paths[-1:] if args.run == "latest" else [os.path.expanduser(args.run)]
    if not paths:
        sys.exit("No tap runs found under %s" % args.dir)

    grid = np.linspace(args.force_min, args.force_max, args.points)
    by_class, loaded = {}, []
    for path in paths:
        run, problem = load_run(path, args.channels)
        if run is None:
            print("%s\n  skipped: %s" % (os.path.basename(path), problem))
            continue
        curves, skipped = tap_curves(run, args.channels)
        digest(path, curves, skipped, args.channels)
        if curves:
            for curve in curves:
                by_class.setdefault(curve["kappa"], []).append(curve)
            loaded.append((path, curves))
    if not loaded:
        sys.exit("Nothing to plot.")

    if args.run:
        path, curves = loaded[-1]
        figure = plot_one_run(path, curves, args.channels, grid, args)
    else:
        figure = plot_classes(by_class, args.channels, grid)

    out = os.path.expanduser(args.save)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    figure.savefig(out, dpi=150, facecolor=SURFACE)
    print()
    print("Wrote %s" % out)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dir", default="~/EDG_Experiment")
    parser.add_argument("--run", default=None,
                        help="one run to look at in detail: a path, or 'latest'")
    parser.add_argument("--channels", type=int, nargs="*", default=[0, 1, 2])
    parser.add_argument("--force-min", type=float, default=0.3)
    parser.add_argument("--force-max", type=float, default=1.9)
    parser.add_argument("--points", type=int, default=64)
    parser.add_argument("--tap-force", type=float, default=2.0,
                        help="only drawn as a reference line")
    parser.add_argument("--save", default="~/EDG_Experiment/ml/inspect.png")
    main(parser.parse_args())
