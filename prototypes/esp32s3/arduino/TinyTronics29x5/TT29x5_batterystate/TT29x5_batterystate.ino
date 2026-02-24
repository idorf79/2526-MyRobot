#include <micro_ros_arduino.h>

// Example code for TinyTronics Smart Home RGB LED Matrix 29x5, based on the Adafruit NeoMatrix Library and AHT20 library.

// MIT License

// Copyright (c) 2024 TinyTronics B.V.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*************************************************************************************/
// IMPORTANT NOTE: Use ESP32 Core V2.0.17 (newer versions might result in a boot loop)
#include <Adafruit_GFX.h>       // Tested with V1.12.1 (With BusIO v 1.15.0)
#include <Adafruit_NeoMatrix.h> // Tested with V1.3.3
#include <Adafruit_NeoPixel.h>  // Tested with V1.15.1
#include <Fonts/TomThumb.h>     // Included in Adafruit_GFX library

#include <Wire.h>  // Included in ESP32 core
#include <AHT20.h> // Tested with V1.0.1. -> dvarrel

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/battery_state.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

/*************************************************************************************/

// Set Board to: LoLin S3 Mini
// Docker command to run on macbook:
// docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 -p 8888 -v 6

// Docker command to run on Linux:
// docker run -it --net=host microros/micro-ros-agent:jazzy udp4 -p 8888 -v 6

// ROS2 command to publish a topic to "battery_state" (which takes a Float32):
// ros2 topic pub /battery_state std_msgs/msg/Float32 "data: 12.3" --once

AHT20 aht20;

#define LEDSTRIP_DIN 42 // This is the Data in pin for the LEDs, and should not be changed

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(29, 5, LEDSTRIP_DIN,
                                               NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
                                               NEO_GRB + NEO_KHZ800);

float battery_voltage = 0.0;

bool showTemperature = true;
bool toggleModeActive = true;

uint16_t connectionDot = matrix.Color(0, 0, 255);
uint16_t commandDot = matrix.Color(255, 0, 255);

const uint16_t updateDotOnColor = matrix.Color(255, 255, 255);
const uint16_t updateDotOffColor = matrix.Color(0, 0, 0);
uint16_t updateDot = updateDotOnColor;
bool updateDotOnState = true;

rcl_subscription_t subscriber;

sensor_msgs__msg__BatteryState sensorMsgBattery;
std_msgs__msg__Int32 msgInt32;
std_msgs__msg__Float32 msgFloat32;

rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;

bool microRosConnected = false;

void error_loop()
{
  while (1)
  {
    //    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    connectionDot = matrix.Color(255, 0, 0);
    delay(500);
    connectionDot = matrix.Color(0, 0, 0);
    delay(500);
  }
}

void vInitMicroROS()
{

  set_microros_wifi_transports("robot-lan", "robot-lan-2024!", "10.10.45.40", 8888);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_tinyt_29x5_node", "", &support));

  // create subscriber
  const char *topic_name = "battery_state";

  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),      
      //ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      topic_name));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, 
  //&msgInt32, 
  //&sensorMsgBattery,
  &msgFloat32,
  &subscription_callback, ON_NEW_DATA));
  printf("Subscriber initialized");
}

void vTaskupdateMatrix(void *pvParameters)
{

  TickType_t xLastWakeTime;

  matrix.begin();
  matrix.setBrightness(50);  // Turn down brightness to about 50%
  matrix.setFont(&TomThumb); // TomThumb font (3x5 pixels)

  xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    matrix.fillScreen(0);   // Erase pixel status
    matrix.setCursor(3, 5); // Set start position
    matrix.setTextColor(matrix.Color(255, 255, 255));

    matrix.print(battery_voltage, 2);

    matrix.print(" V");
    matrix.drawPixel(28, 0, commandDot);
    matrix.drawPixel(28, 1, updateDot);
    matrix.drawPixel(28, 2, connectionDot);

    matrix.show(); // Update matrix
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(40));
  }
}

void vTaskRosConnectionCheck(void *pvParameters)
{

  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(3000));
    if (rmw_uros_ping_agent(100, 3) == RMW_RET_OK)
    {
      connectionDot = matrix.Color(0, 255, 0);
      microRosConnected = true;
    }
    else
    {
      connectionDot = matrix.Color(255, 0, 0);
      microRosConnected = false;
    }
  }
}

// Subscriber message cb
//  This callback will probably run in the current "loop"-task
void subscription_callback(const void *msgin)
{
  //const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  //const sensor_msgs__msg__BatteryState *msg = (const sensor_msgs__msg__BatteryState *)msgin;
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  
  // printf("Received voltage: %d\n", msg->data);
  //printf("Received voltage: %f\n", msg->voltage);

  //battery_voltage = msg->voltage;
  battery_voltage = msg->data;
  
  if (updateDotOnState)
    updateDot = updateDotOnColor;
  else
    updateDot = updateDotOffColor;

  updateDotOnState = !updateDotOnState;
}

void vInitSubscriber()
{

}

void setup()
{

  xTaskCreatePinnedToCore(vTaskupdateMatrix, "updateMatrix", 5000, NULL, 10, NULL, 1);


  vInitMicroROS();

  vInitSubscriber();

  xTaskCreatePinnedToCore(vTaskRosConnectionCheck, "uRosAlivePublisher", 5000, NULL, 10, NULL, 0);
}

void loop()
{
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
