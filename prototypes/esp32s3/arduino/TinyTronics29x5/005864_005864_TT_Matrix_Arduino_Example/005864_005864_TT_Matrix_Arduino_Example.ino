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
#include <Adafruit_GFX.h>        // Tested with V1.12.1 (With BusIO v 1.17.2)
#include <Adafruit_NeoMatrix.h>  // Tested with V1.3.3
#include <Adafruit_NeoPixel.h>   // Tested with V1.15.1
#include <Fonts/TomThumb.h>      // Included in Adafruit_GFX library

#include <Wire.h>   // Included in ESP32 core
#include <AHT20.h>  // Tested with V1.0.1. -> dvarrel
/*************************************************************************************/

// Set Board to: LoLin S3 Mini

AHT20 aht20;

#define LED_DIN 42  // This is the Data in pin for the LEDs, and should not be changed

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(29, 5, LED_DIN,
                                               NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
                                               NEO_GRB + NEO_KHZ800);

void setup() {
  Wire.begin(6, 7);  //Join I2C bus for AHT20 (SDA 6 and SCL 7)
  matrix.begin();
  matrix.setBrightness(25);     // Turn down brightness to about 50%
  matrix.setFont(&TomThumb);    // TomThumb font (3x5 pixels)
  matrix.Color(255, 255, 255);  // Set pixel colour to white
}

void loop() {
  matrix.fillScreen(0);  // Erase pixel status
  matrix.show();         // Update matrix


  float temperature = aht20.getTemperature();
  float humidity = aht20.getHumidity();

  matrix.setCursor(3, 5);  // Set start position,
  matrix.print(temperature, 2);
  matrix.print(" C");
  matrix.show();           // Update matrix
  delay(5000);             // Wait 5 seconds before showing humidity
  matrix.fillScreen(0);    // Erase pixel status
  matrix.setCursor(3, 5);  // Set start position,
  matrix.print(humidity, 0);
  matrix.print(" RH %");
  matrix.show();  // Update matrix
  delay(5000);    // Wait 5 seconds before showing temperature
}
