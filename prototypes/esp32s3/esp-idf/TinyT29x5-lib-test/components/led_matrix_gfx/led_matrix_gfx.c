#include "led_matrix_gfx.h"
#include "font3x5.h"

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "led_strip.h"

static const char *TAG = "led_matrix_gfx";

struct led_matrix_s {
    int width;
    int height;
    lm_layout_t layout;
    bool first_line_reversed;
    uint8_t brightness;
    lm_color_t *fb;            // width*height framebuffer, row-major (y*width + x)
    led_strip_handle_t strip;  // physical driver
};

// --- physical pixel index mapping ------------------------------------------
//
// (x, y) are always logical drawing coordinates: x in [0, width), y in
// [0, height), origin top-left, regardless of physical wiring. This
// function is the only place that needs to know how the strip actually
// snakes through the panel.

static inline int xy_to_index(const led_matrix_t *m, int x, int y)
{
    switch (m->layout) {
    case LM_LAYOUT_ROWS: {
        int col = m->first_line_reversed ? (m->width - 1 - x) : x;
        return y * m->width + col;
    }
    case LM_LAYOUT_ROWS_SERPENTINE: {
        bool reversed = (y % 2 == 0) ? m->first_line_reversed : !m->first_line_reversed;
        int col = reversed ? (m->width - 1 - x) : x;
        return y * m->width + col;
    }
    case LM_LAYOUT_COLUMNS: {
        int row = m->first_line_reversed ? (m->height - 1 - y) : y;
        return x * m->height + row;
    }
    case LM_LAYOUT_COLUMNS_SERPENTINE:
    default: {
        bool reversed = (x % 2 == 0) ? m->first_line_reversed : !m->first_line_reversed;
        int row = reversed ? (m->height - 1 - y) : y;
        return x * m->height + row;
    }
    }
}

// --- init / deinit ----------------------------------------------------------

led_matrix_t *led_matrix_init(const led_matrix_config_t *config)
{
    if (!config || config->width <= 0 || config->height <= 0) {
        ESP_LOGE(TAG, "invalid config");
        return NULL;
    }

    led_matrix_t *m = calloc(1, sizeof(led_matrix_t));
    if (!m) {
        return NULL;
    }

    m->width = config->width;
    m->height = config->height;
    m->layout = config->layout;
    m->first_line_reversed = config->first_line_reversed;
    m->brightness = config->brightness ? config->brightness : 255;

    m->fb = calloc((size_t)m->width * m->height, sizeof(lm_color_t));
    if (!m->fb) {
        ESP_LOGE(TAG, "framebuffer allocation failed");
        free(m);
        return NULL;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = config->gpio_num,
        .max_leds = (uint32_t)(m->width * m->height),
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &m->strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed: %s", esp_err_to_name(err));
        free(m->fb);
        free(m);
        return NULL;
    }

    led_strip_clear(m->strip);
    return m;
}

void led_matrix_deinit(led_matrix_t *m)
{
    if (!m) return;
    if (m->strip) {
        led_strip_clear(m->strip);
        led_strip_del(m->strip);
    }
    free(m->fb);
    free(m);
}

// --- framebuffer control -----------------------------------------------

void lm_clear(led_matrix_t *m)
{
    if (!m) return;
    memset(m->fb, 0, (size_t)m->width * m->height * sizeof(lm_color_t));
}

void lm_fill(led_matrix_t *m, lm_color_t c)
{
    if (!m) return;
    for (int i = 0; i < m->width * m->height; i++) {
        m->fb[i] = c;
    }
}

void lm_set_pixel(led_matrix_t *m, int x, int y, lm_color_t c)
{
    if (!m || x < 0 || x >= m->width || y < 0 || y >= m->height) return;
    m->fb[y * m->width + x] = c;
}

lm_color_t lm_get_pixel(led_matrix_t *m, int x, int y)
{
    if (!m || x < 0 || x >= m->width || y < 0 || y >= m->height) {
        return LM_BLACK;
    }
    return m->fb[y * m->width + x];
}

static inline uint8_t scale8(uint8_t v, uint8_t brightness)
{
    return (uint16_t)v * brightness / 255;
}

esp_err_t lm_show(led_matrix_t *m)
{
    if (!m) return ESP_ERR_INVALID_ARG;

    for (int y = 0; y < m->height; y++) {
        for (int x = 0; x < m->width; x++) {
            lm_color_t c = m->fb[y * m->width + x];
            int idx = xy_to_index(m, x, y);
            led_strip_set_pixel(m->strip, idx,
                                 scale8(c.r, m->brightness),
                                 scale8(c.g, m->brightness),
                                 scale8(c.b, m->brightness));
        }
    }
    return led_strip_refresh(m->strip);
}

// --- shapes ---------------------------------------------------------------

void lm_draw_line(led_matrix_t *m, int x0, int y0, int x1, int y1, lm_color_t c)
{
    // Bresenham's line algorithm
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        lm_set_pixel(m, x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void lm_draw_rect(led_matrix_t *m, int x, int y, int w, int h, lm_color_t c)
{
    if (w <= 0 || h <= 0) return;
    lm_draw_line(m, x, y, x + w - 1, y, c);
    lm_draw_line(m, x, y + h - 1, x + w - 1, y + h - 1, c);
    lm_draw_line(m, x, y, x, y + h - 1, c);
    lm_draw_line(m, x + w - 1, y, x + w - 1, y + h - 1, c);
}

void lm_fill_rect(led_matrix_t *m, int x, int y, int w, int h, lm_color_t c)
{
    for (int j = y; j < y + h; j++) {
        for (int i = x; i < x + w; i++) {
            lm_set_pixel(m, i, j, c);
        }
    }
}

void lm_draw_circle(led_matrix_t *m, int x0, int y0, int radius, lm_color_t c)
{
    // Midpoint circle algorithm
    int x = radius, y = 0, err = 0;
    while (x >= y) {
        lm_set_pixel(m, x0 + x, y0 + y, c);
        lm_set_pixel(m, x0 + y, y0 + x, c);
        lm_set_pixel(m, x0 - y, y0 + x, c);
        lm_set_pixel(m, x0 - x, y0 + y, c);
        lm_set_pixel(m, x0 - x, y0 - y, c);
        lm_set_pixel(m, x0 - y, y0 - x, c);
        lm_set_pixel(m, x0 + y, y0 - x, c);
        lm_set_pixel(m, x0 + x, y0 - y, c);
        y++;
        if (err <= 0) {
            err += 2 * y + 1;
        }
        if (err > 0) {
            x--;
            err -= 2 * x + 1;
        }
    }
}

void lm_fill_circle(led_matrix_t *m, int x0, int y0, int radius, lm_color_t c)
{
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x * x + y * y <= radius * radius) {
                lm_set_pixel(m, x0 + x, y0 + y, c);
            }
        }
    }
}

// --- text -------------------------------------------------------------

int lm_draw_char(led_matrix_t *m, int x, int y, char c, lm_color_t color)
{
    uint8_t ascii = (uint8_t)c;
    if (ascii >= 128) ascii = ' ';

    for (int row = 0; row < LM_FONT_GLYPH_HEIGHT; row++) {
        uint8_t bits = font3x5[ascii][row];
        for (int col = 0; col < LM_FONT_GLYPH_WIDTH; col++) {
            if (bits & (1 << (LM_FONT_GLYPH_WIDTH - 1 - col))) {
                lm_set_pixel(m, x + col, y + row, color);
            }
        }
    }
    return LM_FONT_GLYPH_WIDTH + 1; // +1px spacing between characters
}

int lm_draw_text(led_matrix_t *m, int x, int y, const char *text, lm_color_t color)
{
    int cursor_x = x, cursor_y = y;
    int max_width = 0;

    for (const char *p = text; *p; p++) {
        if (*p == '\n') {
            if (cursor_x - x > max_width) max_width = cursor_x - x;
            cursor_x = x;
            cursor_y += LM_FONT_GLYPH_HEIGHT + 1;
            continue;
        }
        cursor_x += lm_draw_char(m, cursor_x, cursor_y, *p, color);
    }
    if (cursor_x - x > max_width) max_width = cursor_x - x;
    return max_width > 0 ? max_width - 1 : 0; // drop trailing inter-char spacing
}

int lm_text_width(const char *text)
{
    int len = 0;
    for (const char *p = text; *p; p++) {
        len += LM_FONT_GLYPH_WIDTH + 1;
    }
    return len > 0 ? len - 1 : 0;
}
