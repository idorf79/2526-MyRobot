// Example usage of led_matrix_gfx for a 29x5 NeoPixel matrix on GPIO 42.
// Drop this in as your project's main/main.c (and add
// led_matrix_gfx as a component, see README.md).

#include "led_matrix_gfx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MATRIX_WIDTH  29
#define MATRIX_HEIGHT 5
#define MATRIX_GPIO   42

void app_main(void)
{
    led_matrix_config_t config = {
        .gpio_num = MATRIX_GPIO,
        .width = MATRIX_WIDTH,
        .height = MATRIX_HEIGHT,
        // Most single-strip matrices are wired in a zigzag. If your text
        // looks shredded/interleaved, flip this to LM_LAYOUT_ROW_MAJOR,
        // and/or flip first_row_reversed below. See README.md.
        .layout = LM_LAYOUT_SERPENTINE,
        .first_row_reversed = false,
        .brightness = 40, // keep this low to start with -- 145 LEDs at full
                           // white brightness draws a *lot* of current
    };

    led_matrix_t *matrix = led_matrix_init(&config);
    if (!matrix) {
        return; // check the "led_matrix_gfx" log tag for the reason
    }

    // --- 1. static shapes ---------------------------------------------
    lm_clear(matrix);
    lm_draw_rect(matrix, 0, 0, MATRIX_WIDTH, MATRIX_HEIGHT, LM_RED);
    lm_draw_line(matrix, 0, 0, MATRIX_WIDTH - 1, MATRIX_HEIGHT - 1, LM_GREEN);
    lm_fill_circle(matrix, 14, 2, 2, LM_BLUE);
    lm_show(matrix);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // --- 2. static text --------------------------------------------
    lm_clear(matrix);
    lm_draw_text(matrix, 0, 0, "HI", LM_WHITE);
    lm_show(matrix);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // --- 3. scrolling text -------------------------------------------
    const char *message = "HELLO FROM ESP32S3 - PURE ESP-IDF NEOPIXEL MATRIX";
    int text_w = lm_text_width(message);

    for (int offset = MATRIX_WIDTH; offset > -text_w; offset--) {
        lm_clear(matrix);
        lm_draw_text(matrix, offset, 0, message, LM_WHITE);
        lm_show(matrix);
        vTaskDelay(pdMS_TO_TICKS(60)); // scroll speed
    }

    lm_clear(matrix);
    lm_show(matrix);
}
