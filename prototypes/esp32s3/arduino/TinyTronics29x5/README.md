# TinyTronics 29x5 Prototypes.

This folder contains prototypes made in the Arduino IDE.

Versions for libraries etc. are (sometimes) mentioned in the `.ino` files.

Needed libraries: 

```
#include <Adafruit_GFX.h>        // Tested with V1.12.1 (With BusIO v 1.17.2))
#include <Adafruit_NeoMatrix.h>  // Tested with V1.3.3
#include <Adafruit_NeoPixel.h>   // Tested with V1.15.1
#include <Fonts/TomThumb.h>      // Included in Adafruit_GFX library

#include <Wire.h>   // Included in ESP32 core
#include <AHT20.h>  // Tested with V1.0.1. -> dvarrel
```


## MicroROS library (incl. workaround)

To "install" the microROS Arduino library see: https://github.com/micro-ROS/micro_ros_arduino
Make sure to select the correct ROS version (Jazzy now).

For the ESP32S3 MCU, the uROS library seems to fail during the linking stage.
As workaround:

* Goto library installation path (for example `/home/fste/Arduino/libraries/micro_ros_arduino/src`)
* Create a link so ESP32S3 uses the same static library as ESP32: `ln -s esp32 esp32s3`