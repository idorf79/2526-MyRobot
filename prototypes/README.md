# Prototypes

## Development environment for ESP32 controllers

### The ¨OLD¨ way

Use Visual Studio Code to install the ESP-IDF extension (ID: espressif.esp-idf-extension)

Make sure to have an `esp` directory in your "Home": ```mkdir ~/esp/```

During setting up the ESP-IDF Extension

1. Select `Advanced`
Use the following settings:

Download server: Github
Select ESP-IDF version: v5.5(release version)
ESP-IDF container directory: /home/<username>/esp
ESP-IDF Tools directory: /home/<username>/.espressif
Python version: /usr/bin/python3

2. Press "Configure Tools"

This will install ESP-IDF and the needed tools. Also it will create a virtual python environment for building the ESP code.

It will take a while to download and install.

3. Press "Download Tools¨

### Using the ¨new" way: EIM

Used version of ESP-IDF Extension: 2.2.0

Before installing via EIM, install the following packages:

```bash
sudo apt install ccache gperf dfu-utils python3-pip python3-venv
```

Start the ESP-iDF Installation Manager (EIM) via Visual Studio Code.
Make sure to uncheck the "Allow sending usage statistics", if you don´t want to share info with Espressif ;)

Click "Start new Installation" -> "Start installation wizard" in "Custom Installation".

- Select Targets: esp32

- Select IDF version: v5.5.5

- Select Features: "core"

- Select Tools: add the optional "cmake"

## Micro ROS ESP-IDF

in the "components" directory of your ESP-IDF project (create if it doesn´t exist), get a clone of "git@github.com:micro-ROS/micro_ros_espidf_component.git":

```bash
git clone -b jazzy git@github.com:micro-ROS/micro_ros_espidf_component.git
```

There's an issue with a 'log4cxx' package, so as a workaround, remove the line

'    touch src/rcl_logging/rcl_logging_log4cxx/COLCON_IGNORE; \'

from the 'libmicroros.mk' file in the just cloned repository

Afterwards build the needed tools, in a terminal where ROS2 is NOT sourced!

```bash
cd <>/components/micro_ros_espidf_component

make -f libmicrosros ${PWD}/micro_ros_dev/install
```

Also make sure RMW_IMPLEMENTATION is not set, or set to 'rmw_microxrcedds'
Then you can build the ESP MicroROS component.

## External Libraries from Uncle Rus

For the ESP32 some external libraries are used from https://github.com/UncleRus/esp-idf-lib.

These can be "installed" by:

```
cd ~/esp/
git clone -b 0.9.4 https://github.com/UncleRus/esp-idf-lib.git
```

Note: A specific version is used here!

Make sure the `CMakeLists.txt` on the project level reflects the correct path to the libraries. For example:

```
set(EXTRA_COMPONENT_DIRS /home/fste/esp/esp-idf-lib/components)
```
=======
# Prototype folder

This folder contains some prototypes made during the project.
