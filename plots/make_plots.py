"""Plot suction-cup sweep runs straight from the .mat files."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.io import loadmat

UP = r"C:\Users\ninad\.claude\uploads\54553782-a560-42c2-818b-07d940c217b3"
OUT = r"C:\Users\ninad\Desktop\EDG\edg_suction_cup_ros2\plots"

RUNS = [
    ("25 mm sphere, compass at radius/2 (30 deg)", "sphere_25mm",
     "7fff050d-DataLog_2026_0819_164916_Sphere_sweep_force_radius_0.025.mat"),
    ("20 mm sphere, compass at radius/2 (30 deg)", "sphere_20mm",
     "0336c873-DataLog_2026_0819_165600_Sphere_sweep_force_radius_0.02.mat"),
]

SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]
SURFACE, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#8a8983"
# Phase shading, keyed off the /sync event codes.
PHASES = [(1, 2, "#e9e9e6", "descent"), (2, 3, "#dbe9f7", "preload"),
          (3, 4, "#e4f2ec", "dwell")]
NAMES = {1: "center", 2: "north", 3: "west", 4: "south", 5: "east"}

scalar = np.vectorize(lambda x: float(np.ravel(x)[0]))


def load(path):
    m = loadmat(path)
    d = np.array(m["SensorPacket_data"])
    t = scalar(d[:, 0])
    p = np.stack([scalar(d[:, i]) for i in (3, 4, 5)], 1)
    s = np.array(m["sync_data"])
    st = scalar(s[:, 0])
    sc = np.array([int(np.ravel(r)[0]) for r in s[:, 2]])
    return t, p, st, sc


def event(st, sc, code):
    hits = [st[j] for j in range(len(sc)) if sc[j] == code]
    return hits[0] if hits else None


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=INK2, labelsize=9, length=3, width=0.8)
    ax.grid(True, color=MUTED, alpha=0.25, lw=0.6)
    ax.set_axisbelow(True)


def plot_run(title, slug, path):
    t, p, st, sc = load(path)
    waypoints = sorted({c // 10 for c in sc if c // 10 > 0})

    fig, axes = plt.subplots(len(waypoints), 1, figsize=(9.5, 2.05 * len(waypoints)),
                             sharex=True, sharey=True)
    fig.patch.set_facecolor(SURFACE)
    axes = np.atleast_1d(axes)

    for ax, w in zip(axes, waypoints):
        style(ax)
        descend = event(st, sc, w * 10 + 1)
        contact = event(st, sc, w * 10 + 2)
        t0 = contact if contact is not None else descend
        base = p[(t < descend) & (t > descend - 1.0)].mean(0)

        for a, b, colour, label in PHASES:
            ta, tb = event(st, sc, w * 10 + a), event(st, sc, w * 10 + b)
            if ta is not None and tb is not None and tb > ta:
                ax.axvspan(ta - t0, tb - t0, color=colour, zorder=0, label=label)
        if contact is not None:
            ax.axvline(0, color=INK2, lw=1.0, ls=(0, (4, 3)), zorder=3)

        window = (t > t0 - 2.0) & (t < t0 + 5.0)
        for i, colour in enumerate(SERIES):
            y = (p[:, i] - base[i]) / 1000.0
            ax.plot(t[window] - t0, y[window], color=colour, lw=1.6,
                    solid_capstyle="round", zorder=4, label="chamber %d" % i)

        dwell_end = event(st, sc, w * 10 + 4)
        lo, hi = (t0 + 0.3, dwell_end) if dwell_end else (t0 + 0.3, t0 + 2.5)
        drop = (p[(t > lo) & (t < hi)].mean(0) - base).mean() / 1000.0
        search = (contact - descend) if contact is not None else float("nan")
        ax.set_title("%s   |   descent %.2f s, mean drop %.1f kPa"
                     % (NAMES.get(w, "waypoint %d" % w), search, drop),
                     loc="left", fontsize=10, color=INK, pad=6)
        ax.set_ylabel("dP (kPa)", fontsize=9, color=INK2)

    axes[-1].set_xlabel("time from contact (s)", fontsize=9, color=INK2)
    handles, labels = axes[0].get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    fig.legend(uniq.values(), uniq.keys(), loc="upper right", ncol=6,
               frameon=False, fontsize=9, labelcolor=INK2,
               bbox_to_anchor=(0.99, 0.962))
    fig.suptitle(title, x=0.012, y=0.995, ha="left", fontsize=13, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.935))
    dest = os.path.join(OUT, "%s.png" % slug)
    fig.savefig(dest, dpi=160, facecolor=SURFACE)
    plt.close(fig)
    print("wrote", dest)

    return [(NAMES.get(w, str(w)),
             (event(st, sc, w * 10 + 2) or 0) - (event(st, sc, w * 10 + 1) or 0),
             ((p[(t > (event(st, sc, w * 10 + 2) or event(st, sc, w * 10 + 1)) + 0.3)
                 & (t < (event(st, sc, w * 10 + 2) or 0) + 2.5)].mean(0)
               - p[(t < event(st, sc, w * 10 + 1))
                   & (t > event(st, sc, w * 10 + 1) - 1.0)].mean(0)).mean() / 1000.0))
            for w in waypoints]


summary = []
for title, slug, name in RUNS:
    summary.append((title, plot_run(title, slug, os.path.join(UP, name))))

# Table view - the relief rule for the low-contrast slot, and handy on its own.
table = os.path.join(OUT, "sweep_summary.csv")
with open(table, "w") as fh:
    fh.write("run,waypoint,descent_s,mean_drop_kPa" + chr(10))
    for title, rows in summary:
        for name, search, drop in rows:
            fh.write("\"%s\",%s,%.2f,%.2f" % (title, name, search, drop) + chr(10))
print("wrote", table)
