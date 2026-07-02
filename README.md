# Suction Cup ROS 2 Package

This package is the ROS 2 suction cup experiment stack for the EDG UR10 setup. It supports UR10 robot control, TCP pose publishing, ATI force/torque streaming, ESP32 pressure and PWM hardware, live PlotJuggler visualization, and experiment data logging.

## Supported Platform

- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- Universal Robots ROS 2 driver stack
- ATI NetFT sensor support through the ROS 2 `edg_netft_ros2` package
- ESP32 pressure sensor and PWM hardware over serial

The ROS package name is `suction_cup`. This is an `ament_cmake` package that installs Python nodes as ROS 2 executables.

## Workspace Dependencies

Clone these packages into the same ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Jungpyo-L/edg_suction_cup_ros2.git suction_cup
git clone https://github.com/Jungpyo-L/edg_netft_ros2.git
```

You also need the UR ROS 2 driver stack and PlotJuggler:

```bash
sudo apt install ros-jazzy-ur ros-jazzy-plotjuggler-ros python3-serial
```

## Build

From the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select suction_cup edg_netft_ros2
source install/setup.bash
```

If launch files or config files were recently changed, rebuild and source again:

```bash
colcon build --packages-select suction_cup
source install/setup.bash
```

## Launch Files

Use two terminals when both robot control and suction cup sensing are needed.

### Robot Control, MoveIt, and RViz

`ur_control.launch.py` starts the Universal Robots ROS 2 driver and MoveIt. RViz is enabled by default.

```bash
ros2 launch suction_cup ur_control.launch.py
```

Default arguments:

```text
robot_ip:=10.0.0.1
ur_type:=ur10
launch_rviz:=true
headless_mode:=true
```

`headless_mode:=true` tells the UR ROS 2 driver to send the External Control program to the robot automatically. If this does not work with the robot controller version, run the External Control program manually on the teach pendant.

### Suction Cup Experiment Stack

For suction cup work, use `suction_experiment.launch.py`. It starts:

- `robotStatePublisher.py`
- `netft_node`
- `ESP32_Pressure.py`
- `ESP32_PWM.py`
- `data_logger.py`
- PlotJuggler with the suction cup layout

```bash
ros2 launch suction_cup suction_experiment.launch.py
```

Default arguments:

```text
ati_ip:=192.168.1.42
robot_ip:=10.0.0.1
launch_plot:=true
plotjuggler_layout:=<suction_cup share>/config/PlotJuggler_suctioncup_layout.xml
```

Examples:

```bash
ros2 launch suction_cup suction_experiment.launch.py ati_ip:=192.168.1.42
ros2 launch suction_cup suction_experiment.launch.py launch_plot:=false
ros2 launch suction_cup suction_experiment.launch.py plotjuggler_layout:=/path/to/my_preset.xml
```

Note: `ur_experiment.launch.py` is the older generic experiment launch file and does not include the ESP32 pressure or PWM nodes. For suction cup experiments, prefer `suction_experiment.launch.py`.

## ESP32 Hardware Nodes

### Pressure sensor

`ESP32_Pressure.py` reads chamber pressure data from the ESP32 over serial and publishes `SensorPacket`.

Default serial device:

```text
/dev/ttyPressure
```

Behavior:

- Automatically sends the UART start command when the node starts
- Automatically detects the number of chambers from the first valid serial line
- Still listens to `cmdPacket` for stop commands from experiment scripts
- Prints warnings and errors only, not live sensor values

Run manually if needed:

```bash
ros2 run suction_cup ESP32_Pressure.py
ros2 run suction_cup ESP32_Pressure.py --ch 4
ros2 run suction_cup ESP32_Pressure.py --no-auto-start
```

### PWM control

`ESP32_PWM.py` subscribes to `/pwm` and writes PWM duty cycle values to the ESP32 over serial.

Default serial device:

```text
/dev/ttyPWM
```

Run manually if needed:

```bash
ros2 run suction_cup ESP32_PWM.py
```

Publish PWM from another node or terminal:

```bash
ros2 topic pub /pwm std_msgs/msg/Int8 "{data: 100}" --once
```

## Important Topics

Robot TCP pose:

```text
/endEffectorPose
```

ATI force/torque data:

```text
/netft_data
/netft_ready
/diagnostics
```

Suction cup pressure data:

```text
/SensorPacket
/SensorCallback
/cmdPacket
/pwm
/sync
```

Check that nodes and topics are alive with:

```bash
ros2 node list
ros2 topic list | grep -E 'Sensor|netft|pwm|endEffector'
```

## PlotJuggler

PlotJuggler is launched automatically by `suction_experiment.launch.py` when `launch_plot:=true`.

Default layout:

```text
config/PlotJuggler_suctioncup_layout.xml
```

For live streaming:

1. Open PlotJuggler
2. Choose **Streaming** → **Start: ROS 2 Topic Subscriber**
3. Select topics such as `/SensorPacket`, `/SensorCallback`, and `/netft_data`

You can also open the layout manually:

```bash
ros2 run plotjuggler plotjuggler --layout ~/ros2_ws/src/suction_cup/config/PlotJuggler_suctioncup_layout.xml
```

## Data Logging

`data_logger.py` records the topics listed in:

```text
config/TopicsList.txt
```

Current default list:

```text
/SensorPacket
/SensorCallback
/endEffectorPose
/sync
```

Enable and disable logging through the `data_logging` service from experiment scripts. Logged CSV files can be converted to `.mat` files using `fileSaveHelper.py`.

For ROS 2 bag recording:

```bash
ros2 bag record /SensorPacket /SensorCallback /endEffectorPose /netft_data /sync
```

## Example Nodes

Basic UR10 examples:

```bash
ros2 run suction_cup simple_robot_control.py
ros2 run suction_cup simple_data_log.py
ros2 run suction_cup simple_experiment.py
```

Main suction cup lateral positioning demo:

```bash
ros2 run suction_cup Demo_Lateral_positioning_air.py --ch 4 --controller greedy
```

Typical workflow:

```bash
# Terminal 1
ros2 launch suction_cup ur_control.launch.py

# Terminal 2
ros2 launch suction_cup suction_experiment.launch.py

# Terminal 3
ros2 run suction_cup Demo_Lateral_positioning_air.py --ch 4
```

Before running robot motion examples, make sure `ur_control.launch.py` is running and the UR driver controller is active.

## Helper Modules

The helper modules are installed from `src/helperFunction`.

- `rtde_helper.py` contains UR RTDE helper functions for robot motion and TCP pose access.
- `SuctionP_callback_helper.py` subscribes to `/SensorPacket` and provides filtered pressure data.
- `hapticSearch2D.py` contains 2D haptic search controller logic.
- `FT_callback_helper.py` subscribes to `/netft_data` and provides force/torque filtering and offset handling.
- `transformation_matrix.py` contains pose and transformation matrix helpers.
- `fileSaveHelper.py` contains experiment file saving helpers, including `.mat` export.
- `utils.py` contains math utilities used by the robot control scripts.

## Troubleshooting

If `/SensorPacket` is not visible, check the ESP32 pressure node and serial device:

```bash
ros2 node list | grep esp32
ls -l /dev/ttyPressure
```

If `netft_node` does not publish `/netft_data`, verify the ATI sensor IP and that the node is still running:

```bash
ros2 node list | grep netft
ros2 topic list | grep netft
```

If ROS cannot write logs because of permissions:

```bash
export ROS_LOG_DIR=~/ros2_ws/log/ros
```

If deleted packages still appear after tab completion, remove that package's `build/`, `install/`, and `log/` artifacts, then rebuild and source the workspace again.

## Author

Please contact the author with questions.

Jungpyo Lee

- Email: jungpyolee@berkeley.edu
- GitHub: [@Jungpyo-L](https://github.com/Jungpyo-L)
