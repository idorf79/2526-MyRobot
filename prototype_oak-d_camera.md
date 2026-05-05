# ROS2 OAK-D Pro prototype

Source: <https://docs.luxonis.com/software-v3/depthai/ros/>

## Install the drivers

```bash
sudo apt install ros-jazzy-depthai-ros-v3
```

## Run the camera

```bash
ros2 launch depthai_ros_driver_v3 driver.launch.py
```

ros2 launch depthai_ros_driver_v3 driver.launch.py params_file:=/home/rens/rens_hardware/config/oak-d/oak_test.yaml
