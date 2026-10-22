# AIS robot demo: progress and decisions

Starting point: `~/projects/2D_Haptic_Search/docs/handoff/ROBOT_DEMO_HANDOFF.md`.
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

- `src/helperFunction/haptic_policy.py` (numpy only): `PolicyController` (rl_env `mlp_h5s5_log` observation history + exported MLP), `HeuristicController` (greedy / yaw / momentum / yaw_momentum, 10 mm/s, 10 deg/s, tau 0.5 s), `limit_action`, `make_controller`.
  Input: chamber vacuum in Pa, positive, in simulator chamber order. Output: applied body-frame action in [-1, 1]^3.
  The heuristics only need yaw relative to the trial start (constant yaw offsets cancel).
- `policies/*.npz`: deterministic actors exported by `tools/export_policy_npz.py --all` (suctionRL env). Each file stores the SHA-256 of its source `final.zip`.
- `tools/test_policy_parity.py`: replays the 10 robot objects (as-built polygons) x 10 starts, every action from `haptic_policy`.
  Result 2026-10-21: all pass. Observations identical; action difference ≤ 3e-11 (heuristics), ≤ 3e-6 (PPO), ≤ 1.8e-5 (SAC, float32).
  Success matches `outputs/robot_as_built_sim/episodes.json` for all 600 stored episodes; one PPO 3ch episode seals one tick later (481 vs 480) from float32 rounding.
- Cost: about 60 µs per tick in system Python.

## Still to do

2. Measure the pressure sample rate; map physical chambers to simulator order and check the yaw sign (cup is upside down).
3. Body-frame action → tool-frame servoL target (mapping set by step 2).
4. Calibration recording node (port `robot/ros/calibrate_fixture.py` to ROS2).
5. Single-trial node, then the `trial_schedule.csv` scheduler.
