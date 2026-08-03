# Startup Instructions

Working startup sequence for the RealSense point cloud pipeline, plus the failure
modes hit while bringing it up.

## Point cloud in `base_link`

Five terminals. Every one of them starts with:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```

| # | Purpose | Command |
|---|---------|---------|
| T1 | Robot driver — the only source of `base_link` | `ros2 launch suction_cup ur_control.launch.py` |
| T2 | RealSense camera | `ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true` |
| T3 | Hand-eye extrinsic — bridges the two TF trees | `ros2 launch cluttered_grasping_tactile handeye_publish.launch.py` |
| T4 | Detector | `ros2 run suction_cup realsense_sphere_detector.py` |
| T5 | RViz | `ros2 run rviz2 rviz2` |

Order matters only in that T1–T3 must be up before T4, since the detector needs
both TF trees connected.

RViz setup: Fixed Frame `base_link`, Add → By topic → `/filtered_points`. Size
~0.005 and Color Transformer → AxisColor make the sphere's curvature obvious.

T4 logs `filtered_points: N points in base_link` every 5 seconds when it is
working. Silence means no cloud is arriving; a repeating warning means TF.

### Height filtering

The crop box defaults leave x and y unbounded (`±100 m`) and filter on height
alone, because the sphere apex is the topmost point in the scene. Only `crop_max`'s
z normally needs tuning:

```bash
ros2 run suction_cup realsense_sphere_detector.py --ros-args \
  -p crop_min:="[-100.0, -100.0, 0.0]" -p crop_max:="[100.0, 100.0, 0.25]"
```

Set that upper z snugly above the sphere. With it too high, the robot arm is the
highest thing in the cloud and the apex silently jumps to the arm — the topic keeps
publishing plausible-looking numbers, so watch `/sphere_apex` in RViz rather than
trusting it.

`/sphere_apex` is a `geometry_msgs/PoseStamped` with identity orientation: it is a
position, and the tool rotation belongs to the experiment.

### Verification gates

Run these in a spare terminal, in order. Each isolates one layer.

```bash
ros2 topic hz /endEffectorPose                                  # T1 alive
ros2 topic info /camera/camera/depth/color/points -v            # T2: want Publisher count: 1
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame  # T3: must print numbers
ros2 topic hz /filtered_points                                  # T4 publishing
```

The `tf2_echo` gate is the important one — everything downstream depends on it.
Allow a few seconds; it complains until the first `/tf_static` message lands.

## Adding the suction stack

Only needed for experiments, not for viewing the cloud. Independent of T1–T4, so
it can go in any terminal at any point.

```bash
ros2 launch suction_cup suction_experiment.launch.py
# check: ros2 topic hz /netft_data ; ros2 topic hz /SensorPacket
```

Outstanding edit: the `esp32_pressure` node in `launch/suction_experiment.launch.py`
needs `arguments=["--ch", "3"]`. The hardware has 3 chambers; the code defaults to 4.

## Experiments

```bash
# vision-guided, needs T1-T4
ros2 run suction_cup sphere_sweep_experiment.py --dry-run     # always first
ros2 run suction_cup sphere_sweep_experiment.py \
  --offsets "-0.030,0,-0.004; 0,0,0; 0.030,0,-0.004" --press-depth 0.001

# camera-independent, need only T1 + the suction stack
ros2 run suction_cup simple_experiment.py --depth1 0.056 --depth2 0.056 --depth3 0.056
ros2 run suction_cup simple_experiment_force.py --force 3.0
ros2 run suction_cup simple_experiment_penetration.py --penetration 3e-3
```

`--dry-run` prints every planned pose without moving. Compare its z values against
a known-good fixed-z run before allowing motion; disagreement of more than a few
millimeters means the extrinsic is off.

Do not run the sweep against a passthrough crop box. With no crop, "highest point"
is the ceiling or the robot arm, and the printed plan will look reasonable while
being wrong.

## Do not run `realsense_sphere.launch.py`

It starts its own camera *and* its own placeholder static transform. That gives
`camera_link` two parents, and TF resolves a two-parent frame unpredictably — the
cloud lands somewhere wrong while `tf2_echo` still appears to succeed. T2/T3/T4
replace it. Keep it only as a tape-measure fallback if the hand-eye calibration is
ever unavailable.

## Troubleshooting

Failure modes actually encountered, with their distinguishing signature.

**`No executable found` even though the file is installed.**
`--symlink-install` symlinks `install/` back to `src/`, and a symlink inherits
permissions from its target — so CMake never chmods a copy. Files created on
Windows or checked out without the exec bit are not runnable.
```bash
chmod +x ~/ros2_ws/src/suction_cup/src/realsense_sphere_detector.py
```
Git tracks the bit once set. `sphere_sweep_experiment.py` needs the same.

**Camera fails with `Cannot open '/dev/video*'`.**
Another process is holding the device, usually a camera node left running from a
previous attempt.
```bash
pgrep -af realsense
```
Kill it and relaunch T2.

**`Publisher count: 0` on the camera topic.**
The topic exists only because subscribers asked for it; nothing is being sent. The
camera node is not running, or is running without `pointcloud.enable:=true`. Live
fix without a relaunch:
```bash
ros2 param set /camera/camera pointcloud.enable true
```

**`... are not part of the same tree`.**
`base_link` and the camera frames both exist but nothing links them — T3 is not
running. Distinct from `frame does not exist`, which means one of the two trees is
absent entirely (usually T1 or T2 down).

**Detector logs nothing at all.**
No cloud is arriving, so the callback never fires. Every other failure logs
something within 5 seconds. Check the topic name matches `input_topic`.

**Detector logs nothing, but the camera is confirmed publishing.**
The crop box is rejecting everything. To rule it out, pass a box larger than any
real measurement and disable the expensive stage:
```bash
ros2 run suction_cup realsense_sphere_detector.py --ros-args \
  -p crop_min:="[-100.0, -100.0, -100.0]" -p crop_max:="[100.0, 100.0, 100.0]" \
  -p outlier_neighbors:=0 -p voxel_leaf:=0.01
```
Every value needs an explicit decimal point — `-100` parses as an integer array and
the node rejects it as the wrong parameter type.

**RViz empty but `ros2 topic hz` shows a rate.**
Fixed Frame. It defaults to `map`, which does not exist here; set `base_link`.

**Wrong workspace.**
`ros2 pkg prefix suction_cup` shows which install tree is winning. Pick one
workspace and source only that one in every terminal — mixing them means debugging
a stale binary while reading fresh source.

## QoS

- Raw camera topic is **best-effort** (driver's choice). Inspecting it needs
  `--qos-reliability best_effort`, and RViz needs Reliability → Best Effort.
  `ros2 topic hz` does not accept QoS flags at all; only `echo` does.
- `/filtered_points` is **reliable**, matching the CLI and RViz defaults. No flags.
- `/sphere_apex` is **reliable + transient-local**, so a script started later still
  receives the most recent estimate immediately.
