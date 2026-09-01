# Force Experiment

## Commands

Rebuild first. The scripts are copied into `install/`, so a pull alone does
nothing. Source only `/opt/ros/jazzy/setup.bash` in this terminal:

```bash
cd ~/ros2_ws && colcon build --packages-select suction_cup
```

Every terminal below starts with:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```

Robot driver:

```bash
ros2 launch suction_cup ur_control.launch.py
```

Camera:

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

Hand-eye extrinsic:

```bash
ros2 launch cluttered_grasping_tactile handeye_publish.launch.py
```

Detector:

```bash
ros2 run suction_cup realsense_sphere_detector.py --ros-args -p target_frame:=base -p crop_min:="[-100.0, -0.15, 0.0]" -p crop_max:="[100.0, 0.08, 0.12]"
```

Suction stack — FT sensor, pressure, data logger:

```bash
ros2 launch suction_cup suction_experiment.launch.py
```

Dry run:

```bash
ros2 run suction_cup sphere_sweep_experiment.py --radius 0.025 --dry-run
```

Offset mode, no contact:

```bash
ros2 run suction_cup sphere_sweep_experiment.py --radius 0.025
```

Force mode:

```bash
ros2 run suction_cup sphere_sweep_experiment.py --radius 0.025 --mode force --hover-height 0.030 --max-search 0.040
```

Substitute your sphere radius for `0.025`.

## Checks

```bash
ros2 run tf2_ros tf2_echo base camera_depth_optical_frame
ros2 topic hz /filtered_points
ros2 topic hz /netft_data
ros2 service list | grep data_logging
```

In the dry-run output: center pose equals the apex, all five z identical, and
x, y near x 0.53-0.59 / y -0.06. Right magnitudes with both signs flipped means
the detector is in `base_link` instead of `base`.

## Notes

`--hover-height 0.030` is not optional in force mode. The FT sensor is biased at
the hover pose, so if the cup is already touching there the contact force gets
zeroed out and the descent presses far harder before triggering. A 10 mm
standoff was already found to be in contact.

Wait for the detector to log `filtered_points: N points in base` before running
the sweep.

Do not start `netft_node` separately — the suction stack starts its own.

Contact threshold is 0.2 N, about 3x the measured idle noise of 0.063 N p2p.

---

# Debugging notes

Everything below actually went wrong on 2026-08-14.

**RealSense wedges, four different-looking symptoms, one cause.** Stale
`/dev/video0` descriptor; `Failed to resolve the request: Z16 848x480`;
`xioctl(VIDIOC_QBUF) failed`; `UVCIOC_CTRL_QUERY ... Connection timed out`. All
USB. Software restarts do not clear it — unplug the cable, wait five seconds,
replug the same port, then:

```bash
rs-enumerate-devices | grep -i "usb type"
```

Wants `3.2`. Four failures in an hour points at a marginal USB 3 cable or a hub.

**Script prints one line then goes silent for minutes.** That is
`RTDEControlInterface` uploading its control program while contending with
`ur_control.launch.py`. Real work, not a hang, and it only happens once per run.
Check the pendant is in Remote Control with External Control stopped. Dry runs no
longer open this connection, so they return in seconds.

**Silence rules out the data logger.** That branch logs once a second. To test it
directly:

```bash
ros2 service call /data_logging suction_cup/srv/Enable "{enable_data_logging: false}"
```

**Build succeeded but behaviour did not change.** `install(PROGRAMS ...)` copies
the scripts, so `ros2 run` executes the installed copy. Confirm which one:

```bash
grep -n 'declare_parameter("target_frame"' ~/ros2_ws/install/suction_cup/lib/suction_cup/realsense_sphere_detector.py
```

Wants `"base"`. If it says `"base_link"` the build did not take.

**colcon fails on `ament_cmake_python` symlink.** Left over from an earlier
`--symlink-install` build:

```bash
rm -rf ~/ros2_ws/build/suction_cup
```

**`/endEffectorPose` missing.** The suction stack is down. Note
`robotStatePublisher.py` is *also* named `robot_state_publisher`, colliding with
the UR driver's URDF node, so seeing that name in `ros2 node list` does not mean
yours is running.

**Robot moves to the mirrored point.** RTDE never reads TF; it works in the
controller's `base`, which is rotated 180° about Z from ROS `base_link`. RViz
renders either one happily, which is why this is easy to miss.

**Apex looks plausible but is wrong.** If the crop box clips the top of the
sphere, the detector reports the highest surviving point rather than failing.
Same trap if `crop_max` z is too high — the arm becomes the highest thing in the
cloud.

**`/SensorPacket` has six channels for three chambers.** `--ch` on
`ESP32_Pressure.py` is validation only and cannot change the count, which is
auto-detected from the serial stream. Identify which three indices respond by
venting one chamber at a time and record the mapping with the data.

Labelling, per-waypoint summary CSV and pull-off force measurement are written
but **not wired in** — they sit in `reference_contact_labeling.py`.
