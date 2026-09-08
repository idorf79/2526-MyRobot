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

### Macbook

Install ZenohBridge
```
brew install eclipse-zenoh/homebrew-zenoh/zenoh-bridge-ros2dds
```

Start a Zenoh bridge

```bash
zenoh-bridge-ros2dds
```

New terminal, start docker container
```bash
docker run -it --rm --name ros2_zenoh_bridge  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2_jazzy_zenoh:latest szenoh-bridge-ros2dds -e tcp/host.docker.internal:7447
```

Another terminal, start a bash in the bridge container

```bash
docker exec -it ros2_zenoh_bridge bash
source /opt/ros2/jazzy/setup.bash
```