cmake_minimum_required(VERSION 3.8)
project(suction_cup)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorPacket.msg"
  "msg/MarkerPacket.msg"
  "msg/CmdPacket.msg"
  "srv/Enable.srv"
  DEPENDENCIES std_msgs
)

  install(PROGRAMS
  src/adaptive_robot_control.py
  src/curve_sweep_experiment.py
  src/data_logger.py
  src/Demo_Lateral_positioning_air.py
  src/JP_characterization_lateral_positioning.py
  src/ESP32_PWM.py
  src/ESP32_Pressure.py
  src/ft_monitor.py
  src/realsense_sphere_detector.py
  src/robotStatePublisher.py
  src/sphere_sweep_experiment.py
  src/simple_data_log.py
  src/simple_experiment.py
  src/simple_experiment_force.py
  src/simple_experiment_penetration.py
  src/simple_robot_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  src/helperFunction
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  src/helperFunction
  DESTINATION lib/${PROJECT_NAME}
)

# AIS haptic-search robot demo (ais/README.md): nodes run as `ros2 run suction_cup ais_<name>.py`
install(PROGRAMS
  ais/nodes/ais_chamber_check.py
  ais/nodes/ais_fixture_calibration.py
  ais/nodes/ais_pressure_check.py
  ais/nodes/ais_trial_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ais/ais_haptic
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()