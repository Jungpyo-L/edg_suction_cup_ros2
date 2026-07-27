# Caveats — dome sampling for a curvature / pressure model

Context for `src/simple_experiment.py`: sampling a curved geometry (a dome,
R ≈ 50 mm) at a few points with a multi-chamber suction cup, pressing straight
down, to relate per-channel pressure to the contact condition. This document
records the caveats identified while reviewing the method and the data pipeline.

> **Headline:** with a single dome you can build a *tilt / surface-normal* vs
> pressure model, **not** a *curvature* vs pressure model. A curvature model
> needs multiple radii (or cylinders). See Section E.

---

## A. Experimental design & geometry

1. **A single dome has constant curvature.** R = 50 mm → κ = 1/R = 20 m⁻¹
   *everywhere* on the surface. Sampling different points on one dome does **not**
   vary curvature. A curvature model needs **multiple radii** (or a flat plate +
   several domes/cylinders).
2. **What you actually sweep is contact tilt, not curvature.** Pressing straight
   down at points off the apex changes the angle between the cup axis and the
   local surface normal (θ = arcsin(d/R)), where d is the horizontal offset from
   the apex — not the curvature.
3. **Only ~2 distinct conditions.** The three touch points are apex (0°) and
   ±17 mm ≈ ±20°. Left/right are mirror images → effectively **0° and 20°**, at a
   single curvature. Very sparse.
4. **Fixed tool orientation confounds the off-apex points.** At the sides the cup
   lands tilted relative to the surface, engaging on one edge and compressing
   asymmetrically. Intended for a tilt study, but it is not a clean symmetric press.
5. **Home is not a data point** (handled in code: `descend: False`, travel only).

## B. Multi-chamber physics

> **This cup has 3 chambers** (arranged 120° apart, per `calculate_unit_vectors`
> for `n=3`). Note the code's `four_pressure` name and `n_ch=4` default are
> misleading — channel count is auto-detected from `data.ch` at runtime.

6. **Apex, axis-aligned = axisymmetric → no inter-channel variation.** A sphere
   pressed square-on drives all three chambers equally; you get only a common-mode
   change. This is why one dome frustrates a per-channel model.
7. **Inter-channel variation on the dome = tilt (a "dipole").** The chamber
   differences encode the **surface-normal direction**, not curvature.
8. **Curvature needs directional curvature (a "quadrupole").** Only a surface
   curved more in one direction than another (a **cylinder**) makes the chambers
   differ by curvature. A sphere cannot produce this.
9. **The existing haptic-search vector (Σ P·û) is blind to symmetric curvature** —
   it extracts the dipole/tilt only.
10. **With only 3 chambers you cannot separate curvature from tilt.** Three scalar
    readings fully decompose into mean (1 DOF) + 2D dipole/tilt (2 DOF) = 3 DOF —
    every sensor is used up. There is **no remaining degree of freedom for a
    quadrupole**, and a quadrupole (curvature anisotropy) *aliases into the dipole*
    with only 3 samples around the circle. So a 3-chamber cup measures:
    - mean → mean curvature + preload,
    - dipole (2D) → tilt / surface normal (with any curvature anisotropy aliased in).

    Cleanly resolving a curvature-anisotropy signal would need ≥4 chambers.

## C. Force-guided descent method

11. **Bias assumes hover is truly off-surface.** If hover is not clear of the
    dome, the tare captures contact and the stop threshold is wrong.
12. **Stepped descent can overshoot.** Force is only checked *between* steps, so a
    stiff contact can jump past the target within one `force_step`. Use a small
    step (0.1–0.3 mm) on stiff surfaces.
13. **Force reading lags.** `spin_once` pulls the oldest queued `/netft_data`
    message, plus a 7-sample moving average → ~100 ms latency → more overshoot.
    Tolerable at the slow step rate, not zero.
14. **Equal vertical Fz ≠ equal contact / normal force across tilt.** The descent
    stops on `|Fz|` (vertical), but at a tilted point the contact force acts along
    the tilted surface normal. For a target Fz, the true contact force ≈ Fz / cos θ
    (~6% higher at 20°), plus an ignored lateral component (Fx ≈ Fz·tan θ) that can
    make the cup slide. So "same Fz" does **not** mean "same preload / engagement"
    at different points — a residual confound even for the tilt model.
15. **`max_depth` is the only crash guard.** If a hover/depth guess is off, the
    robot descends blindly up to `max_depth` (default 15 mm) before aborting. Keep
    hover safely above the surface.
16. **Descent is base-frame straight down, not surface-normal.** At off-apex points
    the approach is oblique to the surface.
17. **One bias per waypoint.** Sensor drift over a run is small but nonzero.
18. **Dwell is human-timed** (`input()`), so the settle / averaging window per touch
    is inconsistent.

## D. Data actually collected

19. **Contact force is not logged.** `netft_data` is not in
    `config/TopicsList.txt`, so the variable now being controlled (force) exists
    only in the console — you cannot verify equal preload across points afterward.
    **Biggest data gap.**
20. **Only raw pressure is saved.** `/SensorCallback` (filtered) is not produced by
    this script (`P_CallbackHelp` is never instantiated); filter raw
    `/SensorPacket` in post.
21. **No segmentation markers.** `/sync` is advertised but never published, so its
    CSV is empty. Reconstruct per-waypoint contact windows from `/endEffectorPose`.
22. **No pressure tare in the data.** Use each waypoint's hover segment as the
    per-channel baseline in post-processing.
23. **Dome apex xy and radius are not recorded.** They are needed to convert logged
    xy → tilt angle; add them as CLI args so they are saved with the run.
24. **Two asynchronous streams** (~100 Hz pressure, 30 Hz pose) joined by timestamp —
    nearest-match / interpolate in post.
25. **`depth_cm` is mislabeled** — the values are in meters.

## E. Bottom line for modelling

- **This rig can support:** per-channel (and dipole) pressure **vs. contact tilt
  angle**, at fixed curvature and roughly fixed vertical force.
- **It cannot support:** pressure **vs. curvature** — that needs multiple radii /
  cylinders.
- **Even the tilt model** carries caveat #14 (equal Fz ≠ equal normal preload) and
  #19 (force not logged), which are worth addressing before trusting fitted
  parameters.

### Suggested next steps

- Close the cheap data gaps: log `netft_data` (#19) and record apex xy + radius (#23).
- If the goal is truly curvature: add cylinders / multiple radii and press at the
  crest at controlled force (#1, #8). Note (#10) that a 3-chamber cup cannot
  resolve curvature *direction* independently of tilt — separating curvature
  anisotropy would need ≥4 chambers.
