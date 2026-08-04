// Example usage of led_matrix_gfx for a 29x5 NeoPixel matrix on GPIO 42.
// Drop this in as your project's main/main.c (and add
// led_matrix_gfx as a component, see README.md).

#include "led_matrix_gfx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MATRIX_WIDTH  29
#define MATRIX_HEIGHT 5
#define MATRIX_GPIO   42

#include "esp_log.h"
static const char *EX_TAG = "wiring_test";

// Cycles through all 8 (layout x first_line_reversed) combinations,
// drawing "HI" on each and printing the combo to the log. Watch the
// panel and match it to the log line where "HI" reads correctly and
// left-to-right (not mirrored). Use that combo in your real config.
static void wiring_self_test(led_matrix_t *unused)
{
    (void)unused;
    struct { lm_layout_t layout; const char *name; } layouts[] = {
        { LM_LAYOUT_ROWS,               "LM_LAYOUT_ROWS" },
        { LM_LAYOUT_ROWS_SERPENTINE,    "LM_LAYOUT_ROWS_SERPENTINE" },
        { LM_LAYOUT_COLUMNS,            "LM_LAYOUT_COLUMNS" },
        { LM_LAYOUT_COLUMNS_SERPENTINE, "LM_LAYOUT_COLUMNS_SERPENTINE" },
    };

    for (int li = 0; li < 4; li++) {
        for (int rev = 0; rev < 2; rev++) {
            led_matrix_config_t config = {
                .gpio_num = MATRIX_GPIO,
                .width = MATRIX_WIDTH,
                .height = MATRIX_HEIGHT,
                .layout = layouts[li].layout,
                .first_line_reversed = (bool)rev,
                .brightness = 40,
            };
            led_matrix_t *m = led_matrix_init(&config);
            if (!m) continue;

            ESP_LOGI(EX_TAG, "layout=%s first_line_reversed=%d",
                     layouts[li].name, rev);

            lm_clear(m);
            lm_draw_text(m, 0, 0, "HI", LM_WHITE);
            lm_show(m);
            vTaskDelay(pdMS_TO_TICKS(2500));

            led_matrix_deinit(m);
        }
    }
}

void app_main(void)
{
    led_matrix_config_t config = {
        .gpio_num = MATRIX_GPIO,
        .width = MATRIX_WIDTH,
        .height = MATRIX_HEIGHT,
        // If text still looks wrong, try the other 3 combinations of
        // layout / first_line_reversed -- see wiring_self_test() below
        // and README.md "Wiring / layout" section.
        .layout = LM_LAYOUT_COLUMNS_SERPENTINE,
        .first_line_reversed = false,
        .brightness = 60, // keep this low to start with -- 145 LEDs at full
                           // white brightness draws a *lot* of current
    };

    led_matrix_t *matrix = led_matrix_init(&config);
    if (!matrix) {
        return; // check the "led_matrix_gfx" log tag for the reason
    }

    // Uncomment this to walk through all 8 layout combinations one at a
    // time and find the one that matches your physical wiring, instead of
    // guessing. See its definition below.
    // wiring_self_test(matrix);

    const int blink_time = 500;

    while (true){
        // --- 1. static shapes ---------------------------------------------
        lm_clear(matrix);
        lm_fill_rect(matrix, 0, 0, MATRIX_WIDTH, MATRIX_HEIGHT, LM_WHITE);
        lm_draw_rect(matrix, 0, 0, MATRIX_WIDTH, MATRIX_HEIGHT, LM_RED);
        lm_draw_line(matrix, 0, 0, MATRIX_WIDTH - 1, MATRIX_HEIGHT - 1, LM_GREEN);
        lm_fill_circle(matrix, 14, 2, 2, LM_BLUE);
        lm_show(matrix);
        vTaskDelay(pdMS_TO_TICKS(3000));

        // --- 2. static text --------------------------------------------
        lm_clear(matrix);
        lm_draw_text(matrix, 0, 0, "HI", LM_DARK_ORANGE);
        lm_show(matrix);
        vTaskDelay(pdMS_TO_TICKS(3000));

        // --- 3. scrolling text -------------------------------------------
        char *message = "HELLO FROM ESP32S3 - PURE ESP-IDF NEOPIXEL MATRIX";
        int text_w = lm_text_width(message);

        for (int offset = MATRIX_WIDTH; offset > -text_w; offset--) {
            lm_clear(matrix);
            lm_draw_text(matrix, offset, 0, message, LM_WHITE);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(60)); // scroll speed
        }

        // --- 4. scrolling text -------------------------------------------
        const char *message01 = "abcdefghijklmnopqrstuvwxyz";
        text_w = lm_text_width(message01);

        for (int offset = MATRIX_WIDTH; offset > -text_w; offset--) {
            lm_clear(matrix);
            lm_draw_text(matrix, offset, 0, message01, LM_YELLOW);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(60)); // scroll speed
        }    
        
        // --- 5. scrolling text -------------------------------------------
        const char *message02 = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
        text_w = lm_text_width(message02);

        for (int offset = MATRIX_WIDTH; offset > -text_w; offset--) {
            lm_clear(matrix);
            lm_draw_text(matrix, offset, 0, message02, LM_WHITE);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(60)); // scroll speed
        }

        // --- 5. scrolling text -------------------------------------------
        const char *message03 = "0123456789";
        text_w = lm_text_width(message03);

        for (int offset = MATRIX_WIDTH; offset > -text_w; offset--) {
            lm_clear(matrix);
            lm_draw_text(matrix, offset, 0, message03, LM_ORANGE);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(60)); // scroll speed
        }

        // Audi
        // Blink left
        for (int blinkCounter = 0; blinkCounter <= 5; blinkCounter++){
            lm_clear(matrix);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(blink_time));
            for (int width = 0; width <= (MATRIX_WIDTH / 2); width++)
            {
                lm_fill_rect(matrix, (MATRIX_WIDTH / 2) - width, 0, width , MATRIX_HEIGHT, LM_DARK_ORANGE);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(blink_time / (MATRIX_WIDTH / 2)));
            }
        }
        // Blink Right
        for (int blinkCounter = 0; blinkCounter <= 5; blinkCounter++){
            lm_clear(matrix);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(blink_time));
            for (int width = 0; width <= (MATRIX_WIDTH / 2); width++)
            {
                lm_fill_rect(matrix, (MATRIX_WIDTH / 2), 0, width, MATRIX_HEIGHT, LM_DARK_ORANGE);
                lm_show(matrix);
                vTaskDelay(pdMS_TO_TICKS(blink_time / (MATRIX_WIDTH / 2)));
            }
        }

        // Normal Blink
        // Blink left
        for (int blinkCounter = 0; blinkCounter <= 5; blinkCounter++){
            lm_clear(matrix);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(blink_time));
            lm_fill_rect(matrix, 0, 0, MATRIX_WIDTH / 2, MATRIX_HEIGHT, LM_DARK_ORANGE);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(blink_time));
        }

        // Blink Right
        for (int blinkCounter = 0; blinkCounter <= 5; blinkCounter++){
            lm_clear(matrix);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(blink_time));
            lm_fill_rect(matrix, MATRIX_WIDTH / 2, 0, MATRIX_WIDTH, MATRIX_HEIGHT, LM_DARK_ORANGE);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(blink_time));
        }
        lm_clear(matrix);

        // --- static text --------------------------------------------
        lm_draw_text(matrix, (MATRIX_WIDTH / 2), 0, "3", LM_RED);
        lm_show(matrix);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- static text --------------------------------------------
        lm_draw_text(matrix, (MATRIX_WIDTH / 4), 0, "2", LM_BLUE);
        lm_show(matrix);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- static text --------------------------------------------
        lm_draw_text(matrix, (MATRIX_WIDTH / 4) * 3, 0, "1", LM_GREEN);
        lm_show(matrix);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- dynamic text --------------------------------------------
        for (int counter = 1; counter <= 10; counter ++){
            char text[3];
            itoa(counter, text, 10);
            lm_clear(matrix);
            lm_draw_text(matrix, (MATRIX_WIDTH / 4) * 2, 0, text, LM_MAGENTA);
            lm_show(matrix);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(2500));

        lm_clear(matrix);
        lm_show(matrix);
    }
}
