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
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include <Fonts/TomThumb.h>

#include <Wire.h>
#include <AHT20.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/temperature.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop(); \
    } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
    } \
  }

/*************************************************************************************/
// ROS2 command to publish a topic to "command" (which takes a Int32):
// ros2 topic pub /command std_msgs/msg/Int32 "data: 1" --once

AHT20 aht20;

#define LEDSTRIP_DIN 42  // This is the Data in pin for the LEDs, and should not be changed

#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(29, 5, LEDSTRIP_DIN,
                                               NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
                                               NEO_GRB + NEO_KHZ800);

float humidity = 0.0;
float temperature = -14.9;
uint32_t command = 0;

bool showTemperature = true;
bool toggleModeActive = true;
bool rosErrorReset = false;

uint16_t connectionDot = matrix.Color(0, 0, 255);
uint16_t commandDot = matrix.Color(255, 0, 255);
uint16_t errorDot = matrix.Color(0, 0, 0);

const uint16_t updateDotOnColor = matrix.Color(255, 255, 255);
const uint16_t updateDotOffColor = matrix.Color(0, 0, 0);
uint16_t updateDot = updateDotOnColor;
bool updateDotOnState = true;

rcl_publisher_t temp_publisher, hum_publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 hum_msg, command_msg;
sensor_msgs__msg__Temperature sensorMsgTemp;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
TaskHandle_t rosTaskPublisher;


bool microRosConnected = false;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void error_loop() {
  while (!rosErrorReset) {
    //    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    errorDot = matrix.Color(255, 0, 0);
    delay(500);
    errorDot = matrix.Color(0, 0, 0);
    delay(500);
  }
}

bool vCreateMicroRosEntities() {

  rosErrorReset = false;

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_tinyt_29x5_node", "", &support));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &temp_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
    "temperature"));

  RCCHECK(rclc_publisher_init_best_effort(
    &hum_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "humidity"));

  // create subscriber
  const char *topic_name = "command";

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    topic_name));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &command_msg, &subscription_callback, ON_NEW_DATA));

  return true;
}

void vDestroyMicroRosEntities() {

  vTaskDelete(rosTaskPublisher);

  rcl_publisher_fini(&temp_publisher, &node);
  rcl_publisher_fini(&hum_publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  rosErrorReset = true;
}

void vTaskupdateTemperatureAndHumidity(void *pvParameters) {

  TickType_t xLastWakeTime;
  Wire.begin(6, 7);  // Join I2C bus for AHT20 (SDA 6 and SCL 7)

  xLastWakeTime = xTaskGetTickCount();
  while (true) {
    temperature = aht20.getTemperature();
    humidity = aht20.getHumidity();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
    if (toggleModeActive)
      showTemperature = !showTemperature;

    if (updateDotOnState)
      updateDot = updateDotOnColor;
    else
      updateDot = updateDotOffColor;

    updateDotOnState = !updateDotOnState;
  }
}

void vTaskupdateMatrix(void *pvParameters) {

  TickType_t xLastWakeTime;

  matrix.begin();
  matrix.setBrightness(20);   // Turn down brightness to about 50%
  matrix.setFont(&TomThumb);  // TomThumb font (3x5 pixels)

  xLastWakeTime = xTaskGetTickCount();
  while (true) {
    matrix.fillScreen(0);    // Erase pixel status
    matrix.setCursor(3, 5);  // Set start position
    matrix.setTextColor(matrix.Color(255, 255, 255));

    if (showTemperature == true) {
      if (temperature > 0.0) {
        matrix.print(" ");  // add space. Temperatures lower than  0.0 will add a "-"
      }
      if (temperature < 0.0) {
        matrix.setTextColor(matrix.Color(0, 0, 255));
      } else if (temperature < 25.0) {
        matrix.setTextColor(matrix.Color(0, 255, 0));
      } else if (temperature >= 25.0) {
        matrix.setTextColor(matrix.Color(255, 0, 0));
      }
      matrix.print(temperature, 1);

      matrix.setTextColor(matrix.Color(255, 255, 255));
      matrix.print(" C");
    } else {
      matrix.print(humidity, 1);
      matrix.print(" % ");
    }

    matrix.drawPixel(28, 0, connectionDot);
    matrix.drawPixel(28, 1, commandDot);
    matrix.drawPixel(28, 2, updateDot);
    matrix.drawPixel(28, 4, errorDot);

    matrix.show();  // Update matrix
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(150));
  }
}

void vTaskRosPublisher(void *pvParameters) {

  TickType_t xLastWakeTime;

  hum_msg.data = 0;

  xLastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
    if (state == AGENT_CONNECTED) {
      hum_msg.data = humidity * 100;

      sensorMsgTemp.temperature = temperature;

      // Publish to the topic here
      RCSOFTCHECK(rcl_publish(&temp_publisher, &sensorMsgTemp, NULL));
      RCSOFTCHECK(rcl_publish(&hum_publisher, &hum_msg, NULL));
    }
  }
}

void vTaskRos(void *pvParameters) {

  TickType_t xLastWakeTime;


  xLastWakeTime = xTaskGetTickCount();

  while (true) {
    switch (state) {
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        connectionDot = matrix.Color(255, 255, 0);
        break;
      case AGENT_AVAILABLE:
        state = (true == vCreateMicroRosEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
          vDestroyMicroRosEntities();
        };
        connectionDot = matrix.Color(0, 255, 255);

        break;
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
          connectionDot = matrix.Color(0, 255, 0);
        }      
        break;
      case AGENT_DISCONNECTED:
        vDestroyMicroRosEntities();
        state = WAITING_AGENT;
        connectionDot = matrix.Color(255, 0, 0);
        break;
      default:
        break;
    }
  }
}

// Subscriber message cb
//  This callback will probably run in the current "loop"-task
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  printf("Received command: %d\n", msg->data);

  command = msg->data;
  switch (command) {
    case 0:
      commandDot = matrix.Color(255, 0, 255);
      toggleModeActive = true;
      rosErrorReset = true;
      break;
    case 1:
      commandDot = matrix.Color(255, 255, 0);
      toggleModeActive = false;
      showTemperature = true;
      break;
    case 2:
      commandDot = matrix.Color(0, 255, 255);
      toggleModeActive = false;
      showTemperature = false;
      break;
    default:
      commandDot = matrix.Color(255, 0, 255);
      toggleModeActive = true;
      break;
  }
}

void setup() {

  xTaskCreatePinnedToCore(vTaskupdateTemperatureAndHumidity, "getSensorData", 2048, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(vTaskupdateMatrix, "updateMatrix", 4096, NULL, 10, NULL, 1);


  set_microros_wifi_transports("robot-lan", "robot-lan-2024!", "10.2.50.40", 8888);
  //set_microros_transports();

  state = WAITING_AGENT;
  
  xTaskCreatePinnedToCore(vTaskRos, "uRos", 4096, NULL, 10, NULL, 0);

  xTaskCreatePinnedToCore(vTaskRosPublisher, "uPublisher", 4096, NULL, 10, &rosTaskPublisher, 0);

}

void loop() {
}
