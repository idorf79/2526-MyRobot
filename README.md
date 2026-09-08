# MyREnSRobot

This repository contains all the information for my REnS robot.

## Installation

[Installation](how-to-docs/Installation.md)

## Prototyping ESP32
Since the robot will contain an ESP32 at some point, and I like to experiment, some prototyping (sub)projects can be found here:

[ESP32 Prototyping](prototypes/esp32/prototyping/README.md)


## Open Manipulator

[Open Manipulator](prototypes/ros2_ws/prototype_openmanipulator.md)


## Run REnS in Gazebo

### Start Gazebo
```bash
ros2 launch linorobot2_gazebo gazebo.launch.py
```

### Start Slam

```bash
ros2 launch linorobot2_navigation slam.launch.py sim:=true rviz:=true
```

Drive arround, make a map

Save the map

```bash
ros2 run nav2_map_server mapa_saver_cli -f my_map --ros-args -p save_map_timeout:=10000.
```

### Start Navigation using the saved map

```bash
ros2 launch linorobot2_navigation navigation.launch.py sim:=true rviz:=true map:=/tmp/my_map.yaml
```

### Navigate to original starting point

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose 'pose:
  header:
    frame_id: map
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
' --feedback
```