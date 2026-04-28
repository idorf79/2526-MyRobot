# Prototype Laser Scan Filtering

This document describes what's needed to filter the Lidar Scan information.
This could become handy, when there are (fixed) obsticles blocking the Lidars rays.

Sources used:

- <https://wiki.ros.org/laser_filters#scan_to_scan_filter_chain>

For this prototype the following ROS2 packages are needed:

```bash
sudo apt install ros-jazzy-laser-filters
```

## Starting the REAL robot

## Using a simulated robot

Start Gazebo:

```bash
ros2 launch linorobot2_gazebo gazebo.launch.py use_sim_time:=true
```

Run the laser filtering node:

```bash
ros2 run laser_filters scan_to_scan_filter_chain \
  --ros-args -p scan_topic:=scan \
  -p target_frame:=base_link \
  --params-file /home/fste/projects/2526-MyRobot/config/laserscan_filter/laser_filters.yaml
```

Start SLAM with "own" config:

```bash
ros2 launch linorobot2_navigation slam.launch.py rviz:=true sim:=true params_file:=/home/fste/projects/2526-MyRobot/config/laserscan_filter/slam.yaml
```

Start teleoperator:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

TODO: Test Navigation with filtered lidar.
