# led_matrix_gfx

A tiny GFX layer — pixels, lines, rectangles, circles, and text — for a
NeoPixel (WS2812) matrix wired as one single strip, for **pure ESP-IDF**
(no Arduino). It's built on top of Espressif's official `led_strip`
managed component, which drives the strip over RMT.

Written for your board: ESP32-S3, 29x5 matrix (145 LEDs), data on GPIO 42 —
but `width`/`height`/`gpio_num` are all configurable.

## Files

```
led_matrix_gfx/
├── CMakeLists.txt          component build file
├── idf_component.yml       declares the dependency on espressif/led_strip
├── include/
│   └── led_matrix_gfx.h    public API
├── led_matrix_gfx.c        implementation
├── font3x5.h               generated 3x5 pixel font (see font_gen/)
├── font_gen/
│   └── make_font.py        script used to design/verify the font
└── example_main.c          example app_main() using the library
```

## Installing into your project

1. Copy the `led_matrix_gfx/` folder into your project's `components/`
   directory (create that directory if it doesn't exist yet):

   ```
   your_project/
   ├── CMakeLists.txt
   ├── main/
   └── components/
       └── led_matrix_gfx/   <- this folder
   ```

2. `idf_component.yml` declares a dependency on `espressif/led_strip`.
   The first time you build (with network access), ESP-IDF's component
   manager will fetch it automatically:

   ```
   idf.py set-target esp32s3
   idf.py build
   ```

   If you'd rather pull it explicitly first:
   ```
   idf.py add-dependency "espressif/led_strip^2.5.0"
   ```

3. In your own `main/main.c`, `#include "led_matrix_gfx.h"` and make sure
   your `main/CMakeLists.txt`'s `idf_component_register(... REQUIRES ...)`
   includes `led_matrix_gfx` (or just `PRIV_REQUIRES led_matrix_gfx`).

   `example_main.c` in this folder is a working `app_main()` you can drop
   straight into `main/main.c` to see it running.

## Wiring / layout — the one thing you must get right

Your 145 LEDs are one continuous strip snaking through a 29x5 grid. The
library needs to know *how* it snakes so it can translate your (x, y)
drawing coordinates to the correct physical LED index. Two things control
this in `led_matrix_config_t`:

- `layout`:
  - `LM_LAYOUT_SERPENTINE` — rows alternate direction (row 0 goes
    left→right, row 1 right→left, row 2 left→right, ...). This is the
    standard wiring for a single strip folded into a panel, and is the
    default in the example.
  - `LM_LAYOUT_ROW_MAJOR` — every row runs the same direction (you'd
    physically jump the strip back to the start edge for each new row).
- `first_row_reversed` — set this `true` if row 0 (y = 0) starts on the
  *right* edge instead of the left.

**If you're not sure which way your panel is wired:** run the example,
watch `lm_draw_text(matrix, 0, 0, "HI", LM_WHITE)`. If it's legible,
you guessed right. If it looks shredded/interleaved (alternating rows
scrambled), toggle `layout`. If it's mirrored left-right, toggle
`first_row_reversed`.

If your physical layout is more unusual than a simple row/serpentine grid
(e.g. it's wired column-by-column, or in tiles), the mapping lives entirely
in `xy_to_index()` in `led_matrix_gfx.c` — it's a ~10 line function, easy to
adapt.

## API overview

```c
led_matrix_config_t config = {
    .gpio_num = 42,
    .width = 29,
    .height = 5,
    .layout = LM_LAYOUT_SERPENTINE,
    .first_row_reversed = false,
    .brightness = 40,   // 0-255, global scale applied on lm_show()
};
led_matrix_t *m = led_matrix_init(&config);

lm_clear(m);
lm_set_pixel(m, 3, 2, LM_RED);
lm_draw_line(m, 0, 0, 28, 4, LM_GREEN);
lm_draw_rect(m, 0, 0, 29, 5, LM_BLUE);   // outline
lm_fill_rect(m, 5, 1, 4, 3, LM_WHITE);
lm_draw_circle(m, 14, 2, 2, LM_RED);
lm_fill_circle(m, 14, 2, 2, LM_RED);
lm_draw_text(m, 0, 0, "HI", LM_WHITE);   // built-in 3x5 font
lm_show(m);                              // nothing appears until you call this
```

Text is a compact **3 pixels wide x 5 pixels tall** font (uppercase
A-Z, digits 0-9, space, and basic punctuation `. , ! ? : - '`), sized to
exactly fill your 5-row-tall panel. At 3px + 1px spacing per character,
you get about 7 characters visible at once on a 29-wide panel — the
`example_main.c` scrolling demo is the practical way to show longer
messages.

## Notes

- **Power**: 145 WS2812 LEDs at full white and full brightness can draw
  close to 8-9A. Start with a low `brightness` value (as in the example)
  until you've confirmed your power supply/injection can handle it, and
  add a large capacitor + series resistor on the data line per the usual
  NeoPixel wiring guidance if you haven't already.
- **Thread safety**: the framebuffer isn't mutex-protected. If you'll be
  drawing from more than one task, add your own lock around calls, or
  only ever touch the matrix from one task/queue.
- **Extending the font**: `font_gen/make_font.py` defines every glyph as
  5 rows of `#`/`.` and includes an ASCII-art preview printed at generation
  time — edit it, rerun (`python3 font_gen/make_font.py`), and it
  regenerates `font3x5.h`.
