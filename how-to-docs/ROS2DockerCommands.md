# ROS2 Docker commands

## Build "own" ROS2 image

```bash
cd <...>/dockerfiles

docker build -f dockerfile_ros2_zenoh . -t ros2_jazzy_zenoh
```


## Run "own" ROS2 image

```bash
docker run -it --rm --name ros2_zenoh_client --network ros-network -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2_jazzy_zenoh:latest
```