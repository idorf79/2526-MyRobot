/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <Wire.h>
#include <Adafruit_GFX.h>      // Version 1.12.6
#include <Adafruit_SSD1306.h>  // Version 2.5.16

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int counter = 0;
bool direction = true;
char strCounter[10];

void setup() {
  Wire.begin(5, 4);
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  delay(2000);
  display.invertDisplay(false);
  display.clearDisplay();
}

void loop() {
  delay(1);
  //display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  // Display counter text
  // sprintf(strCounter, "%d", counter);
  // display.println(strCounter);

  display.drawCircle(92, 32, counter, SSD1306_WHITE);
  display.drawCircle(32, 32, 30 - counter, SSD1306_WHITE);

  display.display();
  if (direction) {
    counter++;
    if (counter >= 30) {
      direction = false;
    }
  } else {
    counter--;
    if (counter <= 0) {
      direction = true;
    }
  }
}