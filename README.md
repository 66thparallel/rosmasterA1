# ROSMASTER A1 C++ ROS 2 Port

`rosmaster_a1` is a C++17 ROS 2 package that ports the Sparky route-planning,
Pure Pursuit, kinematic simulation, and telemetry workflow. It is a simulator-first
application for ROS 2 Humble on Ubuntu 22.04 and is not yet a direct hardware driver.

## Requirements

- Ubuntu 22.04 with ROS 2 Humble
- C++17 compiler and `colcon`
- ROS packages: `ackermann_msgs`, `diagnostic_msgs`, `geometry_msgs`, `nav_msgs`,
  `tf2`, `tf2_geometry_msgs`, `tf2_ros`, `robot_state_publisher`, `rviz2`, and
  `ament_cmake_gtest`

## Build And Test

From the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rosmaster_a1 --symlink-install
source install/setup.bash
colcon test --packages-select rosmaster_a1 --event-handlers console_direct+
colcon test-result --verbose
```

## Run The Simulator

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rosmaster_a1 simulator.launch.py
```

The launch is headless by default. Start RViz with:

```bash
ros2 launch rosmaster_a1 simulator.launch.py enable_rviz:=true
```

Use a different route/configuration file or log location with:

```bash
ros2 launch rosmaster_a1 simulator.launch.py \
  route_config:=/absolute/path/to/route.yaml \
  metrics_log_dir:=/absolute/path/to/metrics_logs
```

## ROS Interfaces

| Topic | Type | Role |
| --- | --- | --- |
| `/path` | `nav_msgs/msg/Path` | Static route published by `path_planner_node` |
| `/cmd_drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Controller output consumed by the simulator |
| `/odom` | `nav_msgs/msg/Odometry` | Simulated state published by `vehicle_simulator_node` |
| `/metrics/planner` | `diagnostic_msgs/msg/DiagnosticArray` | Route publication telemetry |
| `/metrics/controller` | `diagnostic_msgs/msg/DiagnosticArray` | Pure Pursuit telemetry |

`drive.speed` is metres per second and `drive.steering_angle` is radians. The
simulator publishes `map -> odom -> base_link`; its URDF is only a visualization
placeholder and is not ROSMASTER A1 geometry.

## Configuration

[config/default_route.yaml](config/default_route.yaml) contains parameters for
all four nodes. Important controller parameters are `wheelbase`,
`lookahead_distance`, `max_speed`, `minimum_speed`, and
`max_steering_angle`. `minimum_speed` lets the Ackermann simulator make a forward
turn when a route segment requires a large heading correction. Missing, stale, or
completed route inputs still produce a zero-speed command.

## Physical Robot Integration

Do not connect `/cmd_drive` directly to the ROSMASTER A1 until the Yahboom Humble
driver contract is verified on the Jetson Orin Nano. Before adding a chassis
adapter, inspect the installed vendor workspace and record:

```bash
ros2 topic list -t
ros2 node list
ros2 node info <vendor_driver_node>
ros2 interface show <vendor_command_type>
```

Confirm command units, steering and speed limits, serial-device permissions,
watchdog/emergency-stop behavior, odometry authority, and frame conventions on a
raised-wheel bench. The physical adapter must be launched separately from the
simulator so it cannot create competing command or odometry publishers.

## Validation Status

The package has been built, unit tested, and launched as a simulator with ROS 2
Jazzy. It still requires the same validation on Ubuntu 22.04 with ROS 2 Humble
and on the Jetson Orin Nano before deployment.