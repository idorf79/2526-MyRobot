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
#define LM_BLACK        LM_COLOR(0, 0, 0)
#define LM_WHITE        LM_COLOR(255, 255, 255)
#define LM_RED          LM_COLOR(255, 0, 0)
#define LM_GREEN        LM_COLOR(0, 255, 0)
#define LM_LIGHT_GREEN  LM_COLOR(144, 238, 144)
#define LM_DARK_GREEN   LM_COLOR(0, 100, 0)
#define LM_BLUE         LM_COLOR(0, 0, 255)
#define LM_LIGHT_BLUE   LM_COLOR(173, 216, 230)
#define LM_DARK_BLUE    LM_COLOR(0, 0, 139)
#define LM_YELLOW       LM_COLOR(255, 255, 0)
#define LM_ORANGE       LM_COLOR(255, 165, 0)
#define LM_DARK_ORANGE  LM_COLOR(255, 140, 0)
#define LM_MAGENTA      LM_COLOR(255, 0, 255)

// How the single LED strip is physically wired into the width x height grid.
// The strip either runs in complete ROWS (each traversal is `width` pixels
// long, then jumps to the next row) or complete COLUMNS (each traversal is
// `height` pixels long, then jumps to the next column) -- and within that,
// either every traversal goes the same direction, or direction alternates
// (serpentine / boustrophedon), which is by far the most common wiring for
// a hand-built single-strip panel.
typedef enum {
    LM_LAYOUT_ROWS,               // rows, all same direction
    LM_LAYOUT_ROWS_SERPENTINE,    // rows, alternating direction (zigzag)
    LM_LAYOUT_COLUMNS,            // columns, all same direction
    LM_LAYOUT_COLUMNS_SERPENTINE, // columns, alternating direction (zigzag)
} lm_layout_t;

typedef struct {
    int gpio_num;              // GPIO driving the strip's data line (e.g. 42)
    int width;                 // matrix width in pixels (e.g. 29)
    int height;                // matrix height in pixels (e.g. 5)
    lm_layout_t layout;        // physical wiring layout
    bool first_line_reversed;  // if true, the first row (ROWS* layouts) or
                                // first column (COLUMNS* layouts) runs in
                                // the "reverse" direction (right->left, or
                                // bottom->top) instead of the default
                                // forward direction (left->right / top->bottom)
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
