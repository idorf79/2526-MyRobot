# Installing RenS Robot Software

Start with an up-to-date Ubuntu Server 24.04 version
(sudo apt update && sudo apt upgrade)

1. Disable automatic upgrade

```bash
sudo systemctl disable unattended-upgrades.service
```

1. Disable cloud-init

```bash
sudo touch /etc/cloud/cloud-init.disabled
```

1. Install ROS

The version mentioned next, was detected by using:

```bash
curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}'
```

```bash
export ROS_APT_SOURCE_VERSION=1.2.0
```

1. Get Ubuntu version specific ROS debian package:

```bash
# curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" 

# sudo apt install /tmp/ros2-apt-source.deb 

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

1. Update APT with the new ROS repository:

```bash
sudo apt update
```

1. Install colcon and ROS for the robot:

```bash
sudo apt install -y \
net-tools \
mc \
clang-tidy \
flex \
bison \
build-essential \
python3-vcstool \
python3-venv \
python3-semver \
python3-colcon-common-extensions \
python3-colcon-core \
python3-rosdep \
libpython3-dev \
python3-pip \
ros-jazzy-ros-base \
ros-dev-tools \
ros-jazzy-gazebo-* \
ros-jazzy-cartographer \
ros-jazzy-cartographer-ros \
ros-jazzy-navigation2 \
ros-jazzy-nav2-bringup \
ros-jazzy-laser-filters \
ros-jazzy-moveit \
ros-jazzy-controller-manager \
ros-jazzy-joint-trajectory-controller \
ros-jazzy-gripper-controllers \
ros-jazzy-gz-ros2-control \
ros-jazzy-depthai-ros \
ros-jazzy-joint-state-publisher-gui \
ros-jazzy-realsense2-description \
ros-jazzy-imu-tools \
ros-jazzy-joy-linux \
ros-jazzy-teleop-twist-joy \
ros-jazzy-teleop-twist-keyboard \
ros-jazzy-depthimage-to-laserscan \
ros-jazzy-rqt-image-view \
ros-jazzy-ros2-control \
ros-jazzy-ros2-controllers \
ros-jazzy-dynamixel-hardware-interface \
ros-jazzy-warehouse-ros-sqlite \
ros-jazzy-open-manipulator-gui

sudo apt install -y ros-jazzy-camera-calibration ros-jazzy-camera-calibration-parsers ros-jazzy-camera-info-manager ros-jazzy-compressed-depth-image-transport ros-jazzy-compressed-image-transport ros-jazzy-depth-image-proc ros-jazzy-depthai ros-jazzy-depthai-bridge ros-jazzy-depthai-descriptions ros-jazzy-depthai-examples ros-jazzy-depthai-filters ros-jazzy-depthai-ros-driver ros-jazzy-depthai-ros-msgs ros-jazzy-ffmpeg-image-transport-msgs ros-jazzy-foxglove-msgs ros-jazzy-image-geometry ros-jazzy-image-pipeline ros-jazzy-image-proc ros-jazzy-image-publisher  ros-jazzy-image-rotate ros-jazzy-image-transport-plugins ros-jazzy-image-view ros-jazzy-rviz-imu-plugin ros-jazzy-stereo-image-proc ros-jazzy-theora-image-transport ros-jazzy-tracetools-image-pipeline ros-jazzy-zstd-image-transport
```

1. Initialize ROS dependencies

```bash
sudo rosdep init
rosdep update
```

1. Add ROS environment to bash shell by default

```bash
echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
```

If you want to continue in the same terminal, source your `~/.bashrc` :)

1. Install the RenS software packages

```bash
mkdir -p ~/rens_tmp
cd ~/rens_tmp
wget https://github.com/AvansTi/rens/raw/refs/heads/jazzy_rens/install_linorobot2.bash
bash install_linorobot2.bash 2wd a1 oakdpro


sudo reboot
```

1. Rebuild all the linorobot packages:

```bash
colcon build --symlink-install --packages-skip micro_ros_msgs drive_base_msgs
```

A way to check if were on a Raspberry Pi. Might come in handy:

```bash
cat /sys/firmware/devicetree/base/model
```

1. Add user to dialout groep

```bash
sudo usermod -a -G dialout rens
```

1. Disable auto update/upgrade

TODO: Something with 'sudo systemctl disable unattended-upgrades.service'

TOOD: this needs fixing/updating: Teensy not used lateron

1. Install the RenS Hardware SW

```bash
cd ~/rens_tmp
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py

echo "export PATH=\${PATH}:~/.platformio/penv/bin" >> $HOME/.bashrc
source ~/.bashrc

cd ~
git clone https://github.com/AvansTi/rens_hardware.git rens_hardware

sudo cp ~/rens_hardware/config/udev_rules.d/rens_hw.rules /etc/udev/rules.d

cd ~/rens_hardware/firmware

pio run -e rens
pio run -e pico2
```

1. Upload firmware to MCU

```
cd ~/rens_ws/src/rens_hardware/firmware

pio run -e teensy40 -t upload

```

1. Install the 'Shutdown button'

TODO:
