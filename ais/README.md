# AIS robot demo (haptic search on the UR10)

Everything specific to the AIS paper's robot demonstration lives in this folder; the rest of the package is shared
lab code. Starting point: `~/projects/2D_Haptic_Search/docs/handoff/ROBOT_DEMO_HANDOFF.md`.

## Layout

| Path | Contents |
|---|---|
| `nodes/ais_*.py` | ROS2 nodes, installed flat: `ros2 run suction_cup ais_<name>.py` |
| `ais_haptic/` | Python package used by the nodes: `policy` (controllers), `robot_frame` (frames, cup config), `ros_io` (pressure / vacuum / sync) |
| `config/cups.json` | per-cup TCP, lip radius, chamber map, body offset |
| `policies/*.npz` | exported PPO / SAC actors |
| `tools/` | policy export, tests, `ais_replay_video.py` |

Shared lab code it uses unchanged: `src/helperFunction/rtde_helper.py`, `SuctionP_callback_helper.py`, and the node
`src/ESP32_Pressure.py` (its serial reader was fixed for this demo, see below).
Old ROS1 reference (read-only clone): `~/projects/ref/edg_suction_cup_ros1` (GitHub `Jungpyo-L/edg_suction_cup`, branch `2D_haptic_search`).

## Environment (2026-10-21)

- Robot runs from the `jungpyo` account on edg-3090 only (earlier runs used `edg3090`). Never control the UR from both accounts at once.
- ur_rtde 1.6.5: `python3 -m pip install --user --break-system-packages ur_rtde` (system Python 3.12, the one ROS2 jazzy uses).
- The `suctionRL` conda env is Python 3.10 and cannot import `rclpy`. It is used only for exporting and checking policies.

## Controller set (decided 2026-10-21)

3ch and 6ch cups, each with PPO (`s2_ch3_s5` / `s2_ch6_s1`), greedy, yaw–momentum: 600 trials (2 cups x 3 controllers x 100 starts).
yaw and SAC were dropped; greedy was added (it is the earlier controller and gives a three-level ranking in simulation).
The schedule was regenerated with the same seed (only `trial_schedule.csv` changed; fixture files are byte-identical).
As-built simulation of the same 100 starts: PPO 94/94 %, yaw–momentum 82/78 %, greedy 67/73 % (3ch/6ch).
3ch PPO vs yaw–momentum has low power at 100 starts (0.39–0.74; 2D_Haptic_Search/docs/ROBOT_EXPERIMENT_PLAN.md §2). Decision: run 100 per combination anyway and state the low power in advance.

## Step 1: controllers matching the simulator (done)

- `ais_haptic/policy.py` (numpy only): `PolicyController` (rl_env `mlp_h5s5_log` observation history + exported MLP), `HeuristicController` (greedy / yaw / momentum / yaw_momentum, 10 mm/s, 10 deg/s, tau 0.5 s), `limit_action`, `make_controller`.
  Input: chamber vacuum in Pa, positive, in simulator chamber order. Output: applied body-frame action in [-1, 1]^3.
  The heuristics only need yaw relative to the trial start (constant yaw offsets cancel).
- `policies/*.npz`: deterministic actors exported by `ais/tools/export_policy_npz.py --all` (suctionRL env). Each file stores the SHA-256 of its source `final.zip`.
- `ais/tools/test_policy_parity.py`: replays the 10 robot objects (as-built polygons) x 10 starts, every action from `ais_haptic.policy`.
  Result 2026-10-21: all pass. Observations identical; action difference ≤ 3e-11 (heuristics), ≤ 3e-6 (PPO), ≤ 1.8e-5 (SAC, float32).
  Success matches `outputs/robot_as_built_sim/episodes.json` for all 600 stored episodes; one PPO 3ch episode seals one tick later (481 vs 480) from float32 rounding.
- Cost: about 60 µs per tick in system Python.

## Steps 2-5: robot-side tools (written 2026-10-21, not yet run on the robot)

All new nodes are in `CMakeLists.txt`; `colcon build --packages-select suction_cup`, then `ros2 run suction_cup <node>`.
Run order on the robot day:

| # | Command | What it does |
|---|---|---|
| 1 | `ESP32_Pressure.py --ch 3` | now drains every serial line per 10 ms tick (it used to read one, capping the stream at 100 Hz and letting a faster stream lag) |
| 2 | `ais_pressure_check.py --ch 3 --note "air, vacuum off"` | receive rate, jitter, per-channel noise vs the simulator sensor model (144 Hz, sigma 6 Pa, 7-point MA) |
| 3 | `ais_fixture_calibration.py --cup ch3 --repeats 3` | vacuum off; keyboard jog (or `--freedrive`) into C0..C3, predicts the remaining bores after two; saves `2D_Haptic_Search/robot/calibration/fixture_calibration_ch3.json` |
| 4 | `ais_chamber_check.py --cup ch3 --bore C0 --write` | cup lip ~1 mm over a bore rim, full yaw scan; fits `chamber_map` and `body_offset_deg` in `ais/config/cups.json` |
| 5 | `ais_trial_runner.py --cup ch3 --dry-run` | plan only (no ROS): start poses, controllers load |
| 6 | `ais_trial_runner.py --cup ch3 --trials 0-5 --step` | pilot trials, Enter before each |
| 7 | `ais_trial_runner.py --cup ch3` | the 300-trial block; rerun the same command to resume |

Repeat 3-7 with `--cup ch6` after swapping the cup (calibration and chamber check are per cup: TCP length differs).

Files: `ais_haptic/robot_frame.py` (frames, cup config, chamber fit, servoL target integrator),
`ais_haptic/ros_io.py` (pressure / vacuum / sync on a background-spun node, raw-stream recording),
`ais/config/cups.json` (TCP, lip radius, chamber map; `calibrated: false` until step 4).
Results in `~/EDG_Experiment/ais_robot_demo/`:

| File | Contents |
|---|---|
| `results.csv` | one row per trial: `label` (e.g. `ch3_obj01_pose1_ppo`), cup, controller, policy, object number and id, start pose index 0-9, success, reason, time, path, loop rate, .mat path |
| `trial_XXXX.json` | summary: the above plus start pose (robot and object frame), wrist 3, settings |
| `trial_XXXX.npz` | per 50 Hz tick: sensor, simulator-order vacuum, policy observation, action, servoL target, actual TCP pose, body yaw; 144 Hz raw sensor stream; 1 s hold after a seal |
| `datalog/<yymmdd>/DataLog_<time>_AIS_<label>_trialXXXX_<success or fail_reason>.mat` | data_logger.py topics (`/SensorPacket`, `/SensorCallback`, `/endEffectorPose`, `/sync`, `/netft_data`) as before, plus the trial identity and outcome as variables (`label`, `success`, `start_index`, `object_no`, ...) |
| `session_*.json` | hashes of calibration, cup config, schedule, policies |

The .mat is on by default: start `data_logger.py` (and `robotStatePublisher.py`, netft) before the runner, or pass
`--no-datalog`. A logger failure only skips that trial's .mat; the .npz is always written.

Frames: everything is done in the UR `base` frame seen from above. Tool yaw psi is the tool x-axis on the table;
body yaw = psi + body_offset_deg; rotating about base +z adds to both, so no yaw-sign guess is needed. The chamber check
measures each channel's direction against a known bore, so the old ROS1 hints ([1, 2, 0], -yaw - 90) are not used.

### Verification (no robot)

- `ais/tools/test_robot_frame.py` (system Python): pose conventions, chamber fit for any wiring, integrator. All pass.
- `ais/tools/test_chamber_check.py` (suctionRL, simulator pressure model): bore-rim scans with random wiring/offset; directions recovered within 0.2 deg.
- `ais/tools/test_runner_loop.py` (suctionRL): `run_trial` with the UR and sensor replaced by the simulator, random wiring,
  cup offset and fixture rotation. Geometric seal tick equals the stored simulation in 598/600 trials; the 2 others are
  PPO 3ch episodes > 370 ticks that also diverge from float32 round-off alone.

### Loop timing

The runner keeps its own 50 Hz clock. `rtde_c.waitPeriod()` waits one RTDE period (1/125 s with `rtdeHelp(125)`), so
the first version would have run the loop at 125 Hz while each servoL target advanced 1/50 s of motion: 2.5x the
simulator speed and a 4 s time limit. Fixed before any robot run; `tools/test_loop_timing.py` (real time, ~11 s)
checks 500 ticks in 10 s and 100 deg of turn. Each trial logs `loop_hz` and `loop_overruns`. The UR still servos at its
own 125 Hz between our 50 Hz targets (servoL time = 0.02 s, lookahead smoothing).

### Wrist 3 wind-up

The yaw controllers turn the tool the same way every trial, and a moveL to the next start takes the shortest rotation
from the current wrist, so wrist 3 used to wind up until the joint limit. Now (`ais_haptic/motion.py`) each trial
starts with: lift to 30 mm above the object tops -> moveJ to the start pose with wrist 3 moved by whole turns to within
180 deg of `--q6_home_deg` (default 0) -> moveL down. During a trial the runner stops with reason `joint_limit` if
wrist 3 comes within 10 deg of +-360 deg; every trial logs wrist 3 before/after unwinding and its range.
The chamber check unwinds the same way and scans in 15 deg steps from -172.5 to +172.5 deg (never a 180 deg step).
`tools/test_wrist_unwind.py`, one fake UR through a whole 300-trial block: without unwinding wrist 3 reached -1251 deg
(3ch) / -1487 deg (6ch) and 284 / 236 trials hit the limit; with it, -266 .. +208 deg and none.
On the robot, check that the joint move at the travel height clears the fixture (watch the first few trials).

### Watching trials

`tools/ais_replay_video.py <results dir> --trials ...` renders a trial as mp4 from the runner's files: top view in the
object frame (as-built polygon, chambers coloured by vacuum, path, 30 mm boundary), chamber vacuum on a log axis with
the success threshold, and the applied action. `--ref <dir>` overlays the same trial from another folder (simulation
vs robot). `tools/test_runner_loop.py --save <dir>` keeps the simulator-in-the-loop trials in the same format;
12 examples (3ch/6ch x PPO/greedy/yaw-momentum on two starts each) are in `~/EDG_Experiment/ais_sim_in_the_loop/videos/`.

### Finding: the success criterion differs from the simulator's

The runner stops when every chamber reaches `seal_frac` x P_max (default 0.8); the simulator counts a seal when the open
mouth area is < 1e-3 mm^2. Replaying the robot cases (test_runner_loop.py, 0.8): usually the stop is 0-1 tick after the
geometric seal (sensor lag), but a nearly closed cup can pass 0.8 P_max long before it seals (up to 336 ticks early),
and 6ch yaw-momentum gains 4/100 successes (82 vs 78 %). For sim-vs-robot comparison the simulation reference should use
the same pressure criterion; the per-tick logs allow any threshold 0.6-0.9 afterwards (see ROBOT_EXPERIMENT_PLAN.md §5).

## Still to do

- Robot day: steps 1-7 above, plus Phase 0 checks (sliding distortion, latency) from ROBOT_EXPERIMENT_PLAN.md §6.
- Decide: success threshold (`--seal_frac`), press depth (`--press_mm`, simulator 5 mm), servoL lookahead/gain (tracking is logged).
- Simulation reference with the pressure criterion for the sim-vs-robot comparison.
