/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "neotrellis.h"

static const char *TAG = "neotrellis-app";

static bool led_on[3] = {false, false, false};

void toggleLed(uint8_t led)
{
    struct rgb_color lc;

    lc.red = 0x00;
    lc.green = 0x00;
    lc.blue = 0x00;
    if (led_on[led])
    {
        if (led == 0)
            lc.red = 0xFF;
        if (led == 1)
            lc.green = 0xFF;
        if (led == 2)
            lc.blue = 0xFF;
        led_on[led] = false;
    }
    else
    {
        if (led == 0)
            lc.red = 0x50;
        if (led == 1)
            lc.green = 0x50;
        if (led == 2)
            lc.blue = 0x50;

        led_on[led] = true;
    }
    neotrellis_set_led(led, &lc);
}

void button0Func()
{
    ESP_LOGI(TAG, "Button0");
    toggleLed(0);
}

void button1Func()
{
    ESP_LOGI(TAG, "Button1");
    toggleLed(1);
}

void button2Func()
{
    ESP_LOGI(TAG, "Button2");
    toggleLed(2);
}

static void led_task(void *pArgs)
{
    struct rgb_color lc;
    lc.red = 0x00;
    lc.green = 0x00;
    lc.blue = 0x00;

    toggleLed(0);
    toggleLed(1);
    toggleLed(2);

    while (1)
    {
        for (int led = 4; led < neotrellis_get_number_of_leds(); led++)
        {
            if (neotrellis_is_button_pressed(0))
                lc.red = 0x20;
            if (neotrellis_is_button_pressed(1))
                lc.green = 0x20;
            if (neotrellis_is_button_pressed(2))
                lc.blue = 0x20;
            neotrellis_set_led(led, &lc);
            vTaskDelay(100 / portTICK_PERIOD_MS);

            lc.red = 0x00;
            lc.green = 0x00;
            lc.blue = 0x00;
            neotrellis_set_led(led, &lc);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_ERROR); // set all components to ERROR level
    esp_log_level_set(TAG, ESP_LOG_INFO);  // set all components to ERROR level
    ESP_LOGI(TAG, "Starting");

    neotrellis_init();

    neotrellis_set_button_callback(0, button0Func);
    neotrellis_set_button_callback(1, button1Func);
    neotrellis_set_button_callback(2, button2Func);

    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}