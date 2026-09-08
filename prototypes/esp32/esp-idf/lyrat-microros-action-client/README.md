# Prototyping a ROS2 action server and client

## Server

```bash
cd <>/MyREnSRobot/prototypes
mkdir -p action_server_ws/src
cd actions_server_ws/src

git clone -b jazzy git@github.com:ros2/rclc.git

cd -
cd action_server_ws

source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install

docker run -it --rm --name action_server_example --network ros-network -v .:/mnt -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros:jazzy-ros-base bash
```

Then, in the 'docker' bash:

```bash 
apt update
apt-get install ros-$ROS_DISTRO-osrf-testing-tools-cpp ros-$ROS_DISTRO-test-msgs ros-$ROS_DISTRO-example-interfaces

cd /mnt

colcon build

source install/setup.bash

ros2 run rclc_examples example_action_server
```

## Client

```bash
cd action_server_ws

docker run -it --rm --name action_client_example --network ros-network -v .:/mnt -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros:jazzy-ros-base bash
```

Then, in the 'docker' bash:

```bash 
apt update
apt-get install ros-$ROS_DISTRO-osrf-testing-tools-cpp ros-$ROS_DISTRO-test-msgs ros-$ROS_DISTRO-example-interfaces

cd /mnt

# Build not needed, as this has been done at server?
# colcon build

    source install/setup.bash

    ros2 run rclc_examples example_action_client
```

using micro-ROS on ESP:

```bash
docker run -it -p 8888:8888/udp --rm --net=ros-network -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp microros/micro-ros-agent:jazzy udp4 --port 8888 -v6
```


### Issues

If you run into issues with the actions and microROS (server doesn't get any request).

This might be because there are not enough resources allocated. Xheck 'colcon.meta' in the micro-ros component.
Check the settings below:

```file
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_XML_BUFFER_LENGTH=400",
                "-DRMW_UXRCE_TRANSPORT=udp",
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=5",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=5",
                "-DRMW_UXRCE_MAX_SERVICES=5",
                "-DRMW_UXRCE_MAX_CLIENTS=5",
                "-DRMW_UXRCE_MAX_HISTORY=1",
                "-DRMW_UXRCE_MAX_ACTION_CLIENTS=2",
                "-DRMW_UXRCE_MAX_ACTION_SERVERS=1"
            ]
        },
```

Also check the number of handles given to the executor (in your client's code):

```file
  rclc_executor_init(&executor, &support.context, 6, &allocator);
```
