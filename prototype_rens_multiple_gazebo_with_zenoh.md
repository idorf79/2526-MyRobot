# Multiple gazebos

On Robot01:

export ROS_DOMAIN_ID=2
export GZ_PARTITION=2
ros2 launch linorobot2_gazebo gazebo.launch.py


## Start Zenoh client on Robot:

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
zenoh-bridge-ros2dds -e tcp/192.168.3.20:6400 -n /robot01 peer


# On server:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
zenoh-bridge-ros2dds router -l tcp/192.168.3.20:6400
