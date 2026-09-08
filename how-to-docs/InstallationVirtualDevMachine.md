# Installing RenS Virtual Development Machine

TODO: This is a copy of the REnS Robot Installation. Needs to be updated for Development


1. Create Virtual Machine with VirtualBox

Create a new Virtual Machine.
Use 'ubuntu-24.04.4-desktop-amd64.iso' as ISO.
Don't use "Unattended Installation".

Virtual hardware:
- "Base Memory" -> 8Gb
- "Number of CPUs" -> 4

Virtual hard disk:
- New Virtual Hard Disk
- "Disk Size" -> 125GB

After creation of the virtual machine, set:
- "Video Memory" -> 128MB

Start the Virtual machine and install Ubuntu:
- "Language" -> English
- "Keyboard layout" -> English (US)
- "Connect to internet" -> Use wired connection
- Choose "Interactive installation"
- Choose "Extended selection"
- "Create your account"
  - "Your name" -> "REnS Developer"
  - "Computer's name" -> "rensdevelvm0x"
  - "username" -> "rens"
  - "Password" -> "rens2627"


After reboot:
- Skip Ubuntu Pro
- Don't share system data

Run update & upgrade

```bash
sudo apt update && sudo apt upgrade
```

1. Disable automatic upgrade

```bash
sudo systemctl stop unattended-upgrades.service
sudo systemctl disable unattended-upgrades.service
```

1. Install VirtualBox Guest Additions

Mount the Guest Additions CD image.

Run installer

```bash
sudo /media/rens/VBox_GA_..../VBoxLinuxAdditions.run
```

Afterwards a reboot might be needed.
Enable the "Shared Clipboard"


1. Install extra tools

```bash
sudo apt install terminator screen curl
```

1. Install ROS

The version mentioned next, was detected by using:

```bash
export ROS_APT_SOURCE_VERSION=`curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}'
`
```

1. Get Ubuntu version specific ROS debian package:

```bash
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
teensy-loader-cli \
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


```


1. Upgrade the system

```bash
sudo apt update
sudo apt upgrade
```

1. Add ROS environment to bash shell by default

```bash
echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
```

If you want to continue in the same terminal, source your `~/.bashrc` :)

1. Remove older packages. Use with care!

```bash
rm -rf ~/uros_ws
rm -rf ~/rens_ws
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
```

1. Install the RenS software packages

```bash
mkdir -p ~/rens_tmp
cd ~/rens_tmp
wget https://github.com/AvansTi/rens/raw/refs/heads/jazzy_rens/install_linorobot2.bash
bash install_linorobot2.bash 2wd

cd ~

rm -rf ~/rens_tmp

sudo reboot
```


1. Enable Gazebo build

```bash
rm ~/rens_ws/src/rens/linorobot2_gazebo/COLCON_IGNORE
cd ~/rens_ws/
colcon build --symlink-install

source install/setup.bash
```

1. Install Zenoh

```bash
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg


echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null

sudo apt update

sudo apt install zenoh-bridge-ros2dds ros-jazzy-rmw-cyclonedds-cpp

```

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $HOME/.bashrc
```

Everything up till here has been released in: '260908_REnS_VirtualBox.ova'

## TODO

This section describes what still needs to be done.

## Releasing the VirtualBox

1. Make the disks as small as possible.

Inside the Virtual Machine:

```bash
sudo dd if=/dev/zero of=/zerofile bs=1M status=progress
sudo rm -f /zerofile

sudo shutdown -h now
```

1. Compact the Virtual Box disk.

On the host machine:

```powershell
cd C:\VM\2627-REnSDev
& "C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" modifymedium disk "C:\VM\2627-REnSDev\2627-REnSDev.vdi" --compact
```

1. Export the Virtual Machine as Appliance

In VirtualBox: File -> Export Appliance
Select the correct VirtualMachine ("2627-REnSDev"). Select the correct "File" in "Format Settings", for example: "C:\VM\260908_REnS_VirtualBox.ova".
Under "Appliance Settings", set a version. For example: "260809".
Click "Finish"

You now should have an Appliance which you can share :) 


TODO:

- add 'rens' to group 'dailout'
- install Visual Studio Code
- install Arduino IDE
- generate SSH-keys (?)
- install Docker (?)
- configure GIT (user & email)
