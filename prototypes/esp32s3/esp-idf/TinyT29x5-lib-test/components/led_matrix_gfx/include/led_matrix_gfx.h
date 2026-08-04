#pragma once
// led_matrix_gfx: a tiny GFX layer (pixels/lines/shapes/text) for a WS2812
// NeoPixel matrix wired as a single strip, built on Espressif's official
// `led_strip` RMT component for pure ESP-IDF projects (no Arduino needed).

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t r, g, b;
} lm_color_t;

#define LM_COLOR(R, G, B) ((lm_color_t){ .r = (R), .g = (G), .b = (B) })
#define LM_BLACK   LM_COLOR(0, 0, 0)
#define LM_WHITE   LM_COLOR(255, 255, 255)
#define LM_RED     LM_COLOR(255, 0, 0)
#define LM_GREEN   LM_COLOR(0, 255, 0)
#define LM_BLUE    LM_COLOR(0, 0, 255)

// How the single LED strip is physically wired into the width x height grid.
typedef enum {
    // Every row is wired in the same direction (left->right), i.e. you
    // physically start each new row back on the left edge.
    LM_LAYOUT_ROW_MAJOR,
    // Rows alternate direction (boustrophedon / "zigzag"). This is the
    // usual wiring when a single continuous strip snakes through a panel,
    // e.g. row 0 goes left->right, row 1 right->left, row 2 left->right...
    LM_LAYOUT_SERPENTINE,
} lm_layout_t;

typedef struct {
    int gpio_num;              // GPIO driving the strip's data line (e.g. 42)
    int width;                 // matrix width in pixels (e.g. 29)
    int height;                // matrix height in pixels (e.g. 5)
    lm_layout_t layout;        // physical wiring layout
    bool first_row_reversed;   // if true, row 0 runs right->left instead of left->right
                                // (only relevant for LM_LAYOUT_SERPENTINE; for
                                // LM_LAYOUT_ROW_MAJOR every row uses this same direction)
    uint8_t brightness;        // global brightness scale 0-255, applied at lm_show()
} led_matrix_config_t;

typedef struct led_matrix_s led_matrix_t;

// Create and initialize the matrix (allocates the framebuffer, sets up the
// RMT-backed led_strip driver). Returns NULL on failure.
led_matrix_t *led_matrix_init(const led_matrix_config_t *config);

// Free all resources.
void led_matrix_deinit(led_matrix_t *m);

// --- framebuffer control -----------------------------------------------

void lm_clear(led_matrix_t *m);                              // set framebuffer to black
void lm_fill(led_matrix_t *m, lm_color_t c);                  // set every pixel to c
void lm_set_pixel(led_matrix_t *m, int x, int y, lm_color_t c); // out-of-bounds is a no-op
lm_color_t lm_get_pixel(led_matrix_t *m, int x, int y);

// Push the framebuffer out to the physical LEDs (applies brightness scaling
// and the physical layout mapping). Nothing appears on the hardware until
// you call this.
esp_err_t lm_show(led_matrix_t *m);

// --- shapes ---------------------------------------------------------------

void lm_draw_line(led_matrix_t *m, int x0, int y0, int x1, int y1, lm_color_t c);
void lm_draw_rect(led_matrix_t *m, int x, int y, int w, int h, lm_color_t c);       // outline
void lm_fill_rect(led_matrix_t *m, int x, int y, int w, int h, lm_color_t c);       // filled
void lm_draw_circle(led_matrix_t *m, int x0, int y0, int radius, lm_color_t c);     // outline
void lm_fill_circle(led_matrix_t *m, int x0, int y0, int radius, lm_color_t c);     // filled

// --- text (built-in 3x5 font; covers A-Z, 0-9, space, basic punctuation) --

// Draws one glyph with its top-left corner at (x, y). Returns the x-advance
// in pixels (glyph width + 1px spacing) so you can chain calls.
int lm_draw_char(led_matrix_t *m, int x, int y, char c, lm_color_t color);

// Draws a string starting at (x, y) (top-left of the first glyph). Handles
// '\n' by moving to the next text line (y += 6). Returns the pixel width of
// the longest line drawn, so you can e.g. compute a scroll offset.
int lm_draw_text(led_matrix_t *m, int x, int y, const char *text, lm_color_t color);

// Pixel width the given text would occupy if drawn with lm_draw_text
// (single line only; does not account for '\n').
int lm_text_width(const char *text);

#ifdef __cplusplus
}
#endif
