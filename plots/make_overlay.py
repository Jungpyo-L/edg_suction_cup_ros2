"""Overlay the two sphere runs so they can be compared waypoint by waypoint.

Colour = sphere, line style = chamber.
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from scipy.io import loadmat

UP = r"C:\Users\ninad\.claude\uploads\54553782-a560-42c2-818b-07d940c217b3"
OUT = r"C:\Users\ninad\Desktop\EDG\edg_suction_cup_ros2\plots"

RUNS = [
    ("25 mm sphere", "#e34948",
     "7fff050d-DataLog_2026_0819_164916_Sphere_sweep_force_radius_0.025.mat"),
    ("20 mm sphere", "#2a78d6",
     "0336c873-DataLog_2026_0819_165600_Sphere_sweep_force_radius_0.02.mat"),
]
STYLES = ["-", (0, (5, 2)), (0, (1, 1.6))]   # chamber 0, 1, 2
NAMES = {1: "center", 2: "north", 3: "west", 4: "south", 5: "east"}
SURFACE, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#8a8983"

scalar = np.vectorize(lambda x: float(np.ravel(x)[0]))


def load(path):
    m = loadmat(path)
    d = np.array(m["SensorPacket_data"])
    t = scalar(d[:, 0])
    p = np.stack([scalar(d[:, i]) for i in (3, 4, 5)], 1)
    s = np.array(m["sync_data"])
    return t, p, scalar(s[:, 0]), np.array([int(np.ravel(r)[0]) for r in s[:, 2]])


def event(st, sc, code):
    hits = [st[j] for j in range(len(sc)) if sc[j] == code]
    return hits[0] if hits else None


data = [(label, colour, load(os.path.join(UP, name))) for label, colour, name in RUNS]

fig, axes = plt.subplots(5, 1, figsize=(10, 12.5), sharex=True, sharey=True)
fig.patch.set_facecolor(SURFACE)

for w, ax in zip(range(1, 6), axes):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=INK2, labelsize=9, length=3, width=0.8)
    ax.grid(True, color=MUTED, alpha=0.22, lw=0.6)
    ax.set_axisbelow(True)
    ax.axvline(0, color=INK2, lw=1.0, ls=(0, (4, 3)), zorder=2)
    ax.axhline(0, color=MUTED, lw=0.8, zorder=1)

    notes = []
    for label, colour, (t, p, st, sc) in data:
        descend, contact = event(st, sc, w * 10 + 1), event(st, sc, w * 10 + 2)
        if descend is None:
            continue
        t0 = contact if contact is not None else descend
        base = p[(t < descend) & (t > descend - 1.0)].mean(0)
        window = (t > t0 - 1.5) & (t < t0 + 4.0)
        for i, style in enumerate(STYLES):
            ax.plot(t[window] - t0, (p[window, i] - base[i]) / 1000.0,
                    color=colour, ls=style, lw=1.5, zorder=4)
        end = event(st, sc, w * 10 + 4) or (t0 + 2.5)
        drop = (p[(t > t0 + 0.3) & (t < end)].mean(0) - base).mean() / 1000.0
        notes.append("%s: %.1f kPa" % (label.split()[0], drop))

    ax.set_title("%s      %s" % (NAMES[w], "     ".join(notes)),
                 loc="left", fontsize=10.5, color=INK, pad=6)
    ax.set_ylabel("dP (kPa)", fontsize=9, color=INK2)

axes[-1].set_xlabel("time from contact (s)", fontsize=9, color=INK2)

handles = [Line2D([], [], color=c, lw=1.6, label=l) for l, c, _ in RUNS]
handles += [Line2D([], [], color=INK2, lw=1.4, ls=s, label="chamber %d" % i)
            for i, s in enumerate(STYLES)]
fig.legend(handles=handles, loc="upper right", ncol=5, frameon=False,
           fontsize=9, labelcolor=INK2, bbox_to_anchor=(0.99, 0.966))
fig.suptitle("Chamber pressure vs baseline: 25 mm vs 20 mm sphere",
             x=0.012, y=0.995, ha="left", fontsize=13, color=INK)
fig.tight_layout(rect=(0, 0, 1, 0.945))
dest = os.path.join(OUT, "sphere_overlay.png")
fig.savefig(dest, dpi=160, facecolor=SURFACE)
print("wrote", dest)
