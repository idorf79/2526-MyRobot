# TinyTronics 29x5 Prototypes.

This folder contains prototypes made in the Arduino IDE.

In Arduino IDE, set Board to: "LoLin S3 Mini"

Versions for libraries etc. are (sometimes) mentioned in the `.ino` files.

Needed libraries: 

```
// IMPORTANT NOTE: Use ESP32 Core V3.3.10
#include <Adafruit_GFX.h>        // Tested with V1.12.6 (With BusIO v 1.17.4)
#include <Adafruit_NeoMatrix.h>  // Tested with V1.3.3
#include <Adafruit_NeoPixel.h>   // Tested with V1.15.5
#include <Fonts/TomThumb.h>      // Included in Adafruit_GFX library

#include <Wire.h>   // Included in ESP32 core
#include <AHT20.h>  // Tested with V1.0.2. -> dvarrel
```

## Using serial port for MicroROS communication

Make sure you set 'USB CDC On Boot' to "Enabled". Otherwise the Serial port will not be recognized.


## MicroROS library (incl. workaround)

To "install" the microROS Arduino library see: https://github.com/micro-ROS/micro_ros_arduino
Make sure to select the correct ROS version (Jazzy now).

For the ESP32S3 MCU, the uROS library seems to fail during the linking stage.
As workaround:

* Goto library installation path (for example `/home/<user>/Arduino/libraries/micro_ros_arduino/src`)
* Create a link so ESP32S3 uses the same static library as ESP32: `ln -s esp32 esp32s3

## MicroROS Agent


If you're using serial communication for MicroROS, use this as MicroROS agent:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v 6
```

Using Wifi:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 -v 6
```

Docker command to run on macbook:

```bash
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:jazzy udp4 -p 8888 -v 6
```

Docker command to run on Linux:
```bash
docker run -it --net=host microros/micro-ros-agent:jazzy udp4 -p 8888 -v 6
```