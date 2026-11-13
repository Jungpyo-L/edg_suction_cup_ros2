# Tap experiments: runbook

Everything needed to collect tap data on the sphere bed and flat panel, test a
trained model live, and train one. Commands are run on the lab machine
(`edg3090`) unless marked otherwise. Run one command at a time.

## 1. Get the code onto the lab machine

The lab checkout tracks `sensorized_testing`. If `git pull` ever says "There is
no tracking information for the current branch", link it once:

```bash
cd ~/ros2_ws/src/suction_cup && git fetch origin && git branch --set-upstream-to=origin/sensorized_testing sensorized_testing
```

Pull and rebuild after every change (the build is needed for new scripts, and
`ros2 run` runs the installed copy):

```bash
cd ~/ros2_ws/src/suction_cup && git pull && chmod +x src/*.py
```
```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select suction_cup && source install/setup.bash
```

Check what is checked out with `git log --oneline -1`.

## 2. Start the robot and sensors (three terminals)

Before starting: on the teach pendant the robot is powered on, brakes
released, no protective stop, and in **Remote Control** mode. If any of these
is off, every script hangs at `Opening the RTDE control interface ...`.

Terminal 1, robot driver:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```
```bash
ros2 launch suction_cup ur_control.launch.py
```

Terminal 2, sensors and data logger:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```
```bash
ros2 launch suction_cup suction_experiment.launch.py
```

Terminal 3, check both sensors, then run experiments from here:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```
```bash
ros2 topic hz /netft_data
```
```bash
ros2 topic hz /SensorPacket
```

Expect about 500 Hz for the force sensor and about 100 Hz for pressure.
`/SensorPacket` can take a few seconds to appear after launch. `ros2 topic echo
/SensorPacket` shows `ch: 4` with the 4-chamber cup.

## 3. Collect taps: `sphere_sweep_experiment.py`

### What one run does

1. **Jog** (with `--jog-apex`): for each surface in `--spheres`, you drive the
   cup onto its top by keyboard and press Enter. Each accepted point is saved in
   `~/.ros/suction_cup_apexes.json` and reused by later runs without
   `--jog-apex`.
2. **Plan**: for each surface, `--repeat` points scattered around its jogged
   point, each tapped at `--yaws` rotations. The run cycles through the
   surfaces a point at a time, so drift spreads evenly across all of them.
3. **Tap**: from `--hover-height` above the surface, down at `--tap-speed`
   until `--tap-force`, then straight back up.
4. **Save**: one `.mat` in `~/EDG_Experiment/<YYMMDD>/`, e.g. `261102/`. With `--log-tap-only`
   it keeps only each tap from descent start until the cup is back at hover
   (the press and the rise).

### The standard run (4 spheres)

```bash
ros2 run suction_cup sphere_sweep_experiment.py --spheres 0.020,0.025,0.030,0.040 --mode tap --hover-height 0.003 --travel-height 0.020 --tap-speed 0.010 --tap-acc 0.1 --tap-force 2.5 --max-indent 0.006 --max-search 0.020 --log-tap-only --unattended --repeat 12 --yaws 5 --yaw-span 30 --yaw-start -12 --yaw-jitter 1.5 --jitter-fraction 0.5 --jog-apex --jog-range 0.30
```

240 taps (60 per sphere), about 20-25 minutes. Leave out `--jog-apex
--jog-range 0.30` to reuse the saved apexes when nothing has moved.

### Adding the 35 mm sphere

`--spheres` takes **radii** in metres. The 35 mm radius sphere is `0.035`:

```bash
ros2 run suction_cup sphere_sweep_experiment.py --spheres 0.020,0.025,0.030,0.035,0.040 --mode tap --hover-height 0.003 --travel-height 0.020 --tap-speed 0.010 --tap-acc 0.1 --tap-force 2.5 --max-indent 0.006 --max-search 0.020 --log-tap-only --unattended --repeat 12 --yaws 5 --yaw-span 30 --yaw-start -12 --yaw-jitter 1.5 --jitter-fraction 0.5 --jog-apex --jog-range 0.30
```

### Adding the flat panel (with its 4 holes)

A `0` in `--spheres` is the flat panel. It is jogged at **five** places, in
this order: the panel middle, then the centre of hole 1, 2, 3 and 4. Number
the holes in any order but keep it the same every run.

- Middle taps land within `--center-jitter` of the middle (default 2 mm).
- Hole taps land within `--hole-jitter` of the hole centre (default 2 mm).
- The panel only allows about 2 mm either way, so leave both at the default
  unless the panel changes. (The two 2026-11-05 panel runs used 20 mm middle /
  7 mm hole jitter, before this limit was set.)
- Each of the five gets `--repeat` points of its own, so the panel gets 5x the
  taps of a sphere.

All spheres plus the panel (600 taps):

```bash
ros2 run suction_cup sphere_sweep_experiment.py --spheres 0.020,0.025,0.030,0.035,0.040,0 --mode tap --hover-height 0.003 --travel-height 0.020 --tap-speed 0.010 --tap-acc 0.1 --tap-force 2.5 --max-indent 0.006 --max-search 0.020 --log-tap-only --unattended --repeat 12 --yaws 5 --yaw-span 30 --yaw-start -12 --yaw-jitter 1.5 --jitter-fraction 0.5 --jog-apex --jog-range 0.30
```

The flat panel alone (300 taps; one class only, so not trainable on its own):

```bash
ros2 run suction_cup sphere_sweep_experiment.py --spheres 0 --mode tap --hover-height 0.003 --travel-height 0.020 --tap-speed 0.010 --tap-acc 0.1 --tap-force 2.5 --max-indent 0.006 --max-search 0.020 --log-tap-only --unattended --repeat 12 --yaws 5 --yaw-span 30 --yaw-start -12 --yaw-jitter 1.5 --jog-apex --jog-range 0.30
```

The saved points are keyed `0.0000` (middle) and `0.0000_hole1` ...
`0.0000_hole4` in the apex store.

### Options worth knowing

| Option | Meaning |
|---|---|
| `--tap-speed` | descent speed, m/s (0.010, 0.0125, 0.015 used so far). No upper limit: the force overshoots `--tap-force` more the faster it goes, and above ~3 mm/s raise `--tap-acc` too, or the cup is still speeding up when it reaches the surface from 3 mm hover (needs v^2/2a under 3 mm: 0.5 m/s^2 is enough up to ~50 mm/s) |
| `--repeat`, `--yaws` | points per surface, rotations per point |
| `--yaw-start`, `--yaw-span` | the rotations run from start to start + span, in deg |
| `--yaw-jitter` | each rotation dithered by up to this many deg |
| `--jitter-fraction` | sphere taps land within this fraction of the radius from the apex |
| `--center-jitter`, `--hole-jitter` | flat panel ranges, m (default 0.002 each) |
| `--seed` | fixes the random tap positions; a new random one is used and saved if omitted |
| `--dry-run` | print the plan without moving (needs saved points) |
| `--no-home` | stay at the travel plane at the end instead of parking |

### What the `.mat` records

All options, plus `tap_plan` (one row per tap: dx, dy, yaw, site, where site
is 0 for a sphere or the panel middle and n for hole n), `tap_radius` (0 for
flat), `tap_results` (contact z, stop z, peak force, why it stopped),
`apex_used` (one row per jogged point) and `surface_names` (which surface each
row of `apex_used` is).

### Several runs in a row: `sweep_queue.py`

Write the runs into a text file, one per line, each line being exactly the
arguments you would give `sphere_sweep_experiment.py`. `#` starts a comment.
Only the **first** line may jog (`--jog-apex`); the rest reuse the saved
points, so after the jog you can leave. `--unattended` is added where missing.

```text
# queue.txt - jog the spheres once, then three speeds (0.1 m/s^2 as in earlier runs; 25 mm/s needs 0.5)
--spheres 0.020,0.025,0.030,0.040 --mode tap --hover-height 0.003 --travel-height 0.020 --tap-speed 0.010 --tap-acc 0.1 --tap-force 2.5 --max-indent 0.006 --max-search 0.020 --log-tap-only --repeat 12 --yaws 5 --yaw-span 30 --yaw-start -12 --yaw-jitter 1.5 --jitter-fraction 0.5 --jog-apex --jog-range 0.30
--spheres 0.020,0.025,0.030,0.040 --mode tap --hover-height 0.003 --travel-height 0.020 --tap-speed 0.015 --tap-acc 0.1 --tap-force 2.5 --max-indent 0.006 --max-search 0.020 --log-tap-only --repeat 12 --yaws 5 --yaw-span 30 --yaw-start -12 --yaw-jitter 1.5 --jitter-fraction 0.5
--spheres 0.020,0.025,0.030,0.040 --mode tap --hover-height 0.003 --travel-height 0.020 --tap-speed 0.025 --tap-acc 0.5 --tap-force 2.5 --max-indent 0.006 --max-search 0.020 --log-tap-only --repeat 12 --yaws 5 --yaw-span 30 --yaw-start -12 --yaw-jitter 1.5 --jitter-fraction 0.5
```

Make the file on the lab machine (paste the lines in, save with Ctrl-O, exit
with Ctrl-X):

```bash
nano ~/queue.txt
```

Check every line and print each run's plan without moving the robot:

```bash
ros2 run suction_cup sweep_queue.py ~/queue.txt --dry-run
```

Run it:

```bash
ros2 run suction_cup sweep_queue.py ~/queue.txt
```

- Every line is checked before the robot moves: a typo, a bad value, a jog on
  a later line, or a surface that is neither saved nor jogged by an earlier
  line stops it at the start. A surface added on a later line (e.g. the flat
  panel) must be jogged in the first line too, or already saved.
- A failed run (protective stop, sensor dropout, ...) has already lifted the
  cup and parked the arm; the queue then stops. `--keep-going` carries on with
  the next run instead. `--pause` sets the wait between runs (default 10 s).
- Ctrl-C lets the current run recover the arm, then stops the queue.
- Each run saves its own `.mat` as usual. A log of every run (start time,
  minutes, ok/failed, files saved) goes to `~/EDG_Experiment/queue_<date>_<time>.log`,
  and a summary prints at the end.
- Rough time per run: about 21-23 min for 12 x 5 on four spheres (240 taps),
  25-27 min for the flat panel alone (300 taps).

## 4. Jogging

| Key | Action |
|---|---|
| `w` / `s` | +x / -x |
| `a` / `d` | +y / -y |
| `r` / `f` | up / down |
| `[` / `]` | halve / double the fine step (starts at 1 mm; 0.1-10 mm) |
| Shift + move key | big step: 10 mm, or 100 mm after pressing `m` |
| `m` | switch the Shift step between 10 mm and 100 mm |
| Enter | accept this pose as the point |
| `x` | abort (lifts straight up and parks) |

- When a jog opens, the arm **never moves down by itself**: it lifts if
  needed, crosses, and stops above the last saved point. Every downward move
  is a key you press.
- Between surfaces it crosses 20 mm above the tallest saved point.
- The 100 mm step can go down: make sure there is room below before pressing
  Shift+F in 100 mm mode. Come down the last few mm with plain `f` and `[`.
- A jog may move at most `--jog-range` sideways from where it opened.
- After moving spheres, always run with `--jog-apex`: a run without it taps
  the old saved positions.

## 5. Sending the arm home: `go_home.py`

Lifts straight up (keeping the current rotation) to at least the parked
height, then moves to the parked pose:

```bash
ros2 run suction_cup go_home.py
```

`--lift 0.10` lifts further before crossing.

## 6. Testing a trained model live

Tap a sphere and print the prediction. `--at` taps a saved apex without
jogging; `--start` begins the jog above a saved apex:

```bash
ros2 run suction_cup predict_curvature.py --model ~/EDG_Experiment/ml/<model>.joblib --at 0.030 --truth 0.030 --taps 3
```

Visit the saved apexes one by one, searching down for the surface at each:

```bash
ros2 run suction_cup predict_at_points.py --model ~/EDG_Experiment/ml/<model>.joblib --apexes 0.020,0.025,0.030,0.040 --taps 1
```

Or give base-frame x-y points with a start and floor height:

```bash
ros2 run suction_cup predict_at_points.py --model ~/EDG_Experiment/ml/<model>.joblib --points "0.53,-0.14; 0.58,-0.14" --start-z 0.15 --floor-z 0.06
```

Both wait for Enter after each point unless `--unattended`. Use the same
`--tap-speed` as the training data. Keep a hand near the stop button the
first time.

## 7. Training: `ml/curvature_classifier.py`

```bash
python3 ~/ros2_ws/src/suction_cup/ml/curvature_classifier.py --dir ~/EDG_Experiment/261102 --speeds 0.010 --channels 0 1 2 3 --no-indentation --evaluate minirocket --test-size 0.3 --model minirocket --points 128 --out ~/EDG_Experiment/ml/261102_10mm_pressure
```

| Option | Meaning |
|---|---|
| `--dir` | folder searched (recursively) for sweep `.mat` files |
| `--speeds` | keep only runs at these tap speeds, m/s |
| `--channels` | pressure chambers: `0 1 2` (3-chamber cup), `0 1 2 3` (4-chamber) |
| `--no-indentation` | pressure only, leave out the arm's indentation |
| `--representation` | `force` (read at 0.3-1.9 N) or `time` (-100 to +300 ms around contact) |
| `--evaluate` | which models to score: `logreg`, `forest`, `minirocket` |
| `--test-size 0.3` | score on a random 30% held out; without it, leave one run out |
| `--model`, `--out` | the model to fit on every tap and save |
| `--list` | just list the runs found |

The saved model is fitted on every tap; the score printed is from the split.

## 8. Data and figures (laptop)

- Runs copied to the laptop go under `data/` (gitignored), e.g. `data/11-04/`.
- Figures go in `data/ml/figs/<YYYY-MM-DD>/`. Each gets an entry in
  `data/ml/figs/README.md` (runs, settings, results) and a one-line
  description in `data/ml/figs/FIGURES.txt`.
- Run file names carry neither speed nor sphere position; read `tap_speed`
  etc. from the `.mat`, or keep the mapping in the run log below.

## 9. Run log

| File (`DataLog_2026_...`) | Cup | Surfaces | Speed | Notes |
|---|---|---|---|---|
| `1022_173345` | 3-ch | R20/25/30/40 | 10 mm/s | 10 pts x 6 yaws, ends at the stop (no rise) |
| `1022_175827` | 3-ch | R20/25/30/40 | 10 mm/s | 12 pts x 10 yaws, ends at the stop (no rise) |
| `1102_203126` | 4-ch | R20/25/30/40 | 10 mm/s | first positions, yaw start -15 |
| `1102_205625` | 4-ch | R20/25/30/40 | 12.5 mm/s | first positions, yaw start -15 |
| `1102_212100` | 4-ch | R20/25/30/40 | 15 mm/s | first positions, yaw start -15 |
| `1103_175008` | 4-ch | R20/25/30/40 | 15 mm/s | spheres moved, yaw start -12 |
| `1103_181400` | 4-ch | R20/25/30/40 | 10 mm/s | spheres moved, yaw start -12 |
| `1105_??????` | 4-ch | flat panel only (middle + 4 holes) | 10 mm/s | 300 taps, 20 mm middle / 7 mm hole jitter; file name to fill in |
| `1105_??????` | 4-ch | flat panel only (middle + 4 holes) | 15 mm/s | 300 taps, 20 mm middle / 7 mm hole jitter; file name to fill in |
| `1112_172507` | 3-ch | flat panel only (middle + 4 holes) | 10 mm/s | 300 taps, 2 mm jitter, jogged |
| `1112_175420` | 3-ch | flat panel only (middle + 4 holes) | 15 mm/s | 300 taps, 2 mm jitter, reused 172507 points |
| `1112_183045` | 3-ch | R20/25/30/40 | 15 mm/s | 12 pts x 10 yaws, yaw start -15, new sphere arrangement; pairs with `1022_175827` (10 mm/s) |

Still to do: 12.5 mm/s with the spheres moved.

## 10. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Hangs at `Opening the RTDE control interface ...` | pendant: clear the protective stop, power on, brakes off, Remote Control mode |
| `git pull`: no tracking information | see section 1 |
| `topic [/SensorPacket] does not appear to be published yet` | wait a few seconds after launch |
| Run taps the old sphere positions | spheres were moved: run with `--jog-apex` |
| Pressure in the 4-chamber runs changes ~0.6 s after contact | under investigation (cup vs sensor link); train on the press + rise, not the press alone |
