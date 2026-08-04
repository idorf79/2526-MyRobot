#!/usr/bin/env python3
"""
Generates a compact 3-wide x 5-tall bitmap font for tiny LED matrices
(fits exactly in a 5-row-tall panel) and dumps it as a C header.

Each glyph is 5 rows. Each row is given as a 3-character string of
'#'/'.' (left column first). We render an ASCII preview of every glyph
so mistakes are obvious before they end up on hardware.
"""

FONT = {
    ' ': ["...", "...", "...", "...", "..."],
    '!': [".#.", ".#.", ".#.", "...", ".#."],
    '?': ["##.", "..#", ".#.", "...", ".#."],
    '.': ["...", "...", "...", "...", ".#."],
    ',': ["...", "...", "...", ".#.", "#.."],
    ':': [".#.", "...", "...", ".#.", "..."],
    '-': ["...", "...", "###", "...", "..."],
    "'": [".#.", ".#.", "...", "...", "..."],
    '0': [".#.", "#.#", "#.#", "#.#", ".#."],
    '1': [".#.", "##.", ".#.", ".#.", "###"],
    '2': ["##.", "..#", ".#.", "#..", "###"],
    '3': ["##.", "..#", ".#.", "..#", "##."],
    '4': ["#.#", "#.#", "###", "..#", "..#"],
    '5': ["###", "#..", "##.", "..#", "##."],
    '6': [".##", "#..", "##.", "#.#", ".#."],
    '7': ["###", "..#", ".#.", "#..", "#.."],
    '8': [".#.", "#.#", ".#.", "#.#", ".#."],
    '9': [".#.", "#.#", ".##", "..#", "##."],
    'A': [".#.", "#.#", "###", "#.#", "#.#"],
    'B': ["##.", "#.#", "##.", "#.#", "##."],
    'C': [".##", "#..", "#..", "#..", ".##"],
    'D': ["##.", "#.#", "#.#", "#.#", "##."],
    'E': ["###", "#..", "##.", "#..", "###"],
    'F': ["###", "#..", "##.", "#..", "#.."],
    'G': [".##", "#..", "#.#", "#.#", ".##"],
    'H': ["#.#", "#.#", "###", "#.#", "#.#"],
    'I': ["###", ".#.", ".#.", ".#.", "###"],
    'J': ["..#", "..#", "..#", "#.#", ".#."],
    'K': ["#.#", "#.#", "##.", "#.#", "#.#"],
    'L': ["#..", "#..", "#..", "#..", "###"],
    'M': ["#.#", "###", "###", "#.#", "#.#"],
    'N': ["#.#", "##.", "#.#", ".##", "#.#"],
    'O': [".#.", "#.#", "#.#", "#.#", ".#."],
    'P': ["##.", "#.#", "##.", "#..", "#.."],
    'Q': [".#.", "#.#", "#.#", ".#.", "..#"],
    'R': ["##.", "#.#", "##.", "#.#", "#.#"],
    'S': [".##", "#..", ".#.", "..#", "##."],
    'T': ["###", ".#.", ".#.", ".#.", ".#."],
    'U': ["#.#", "#.#", "#.#", "#.#", ".#."],
    'V': ["#.#", "#.#", "#.#", "#.#", ".#."],
    'W': ["#.#", "#.#", "###", "###", "#.#"],
    'X': ["#.#", "#.#", ".#.", "#.#", "#.#"],
    'Y': ["#.#", "#.#", ".#.", ".#.", ".#."],
    'Z': ["###", "..#", ".#.", "#..", "###"],
}

def preview(ch, rows):
    print(f"'{ch}':")
    for r in rows:
        print("  " + r.replace('#', '#').replace('.', ' '))
    print()

def rows_to_bits(rows):
    # bit2 = leftmost column, bit0 = rightmost column
    out = []
    for r in rows:
        assert len(r) == 3, f"bad row length in glyph: {r!r}"
        v = 0
        for i, ch in enumerate(r):
            if ch == '#':
                v |= (1 << (2 - i))
        out.append(v)
    return out

def main():
    # sanity check + preview everything
    for ch, rows in FONT.items():
        assert len(rows) == 5, f"glyph {ch!r} must have 5 rows"
        preview(ch, rows)

    lines = []
    lines.append("// Auto-generated 3x5 pixel font. See font_gen/make_font.py.")
    lines.append("#pragma once")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append("#define LM_FONT_GLYPH_WIDTH  3")
    lines.append("#define LM_FONT_GLYPH_HEIGHT 5")
    lines.append("")
    lines.append("// Row-major bitmap: font3x5[ascii_index][row], bits 2..0 = columns left..right")
    lines.append("static const uint8_t font3x5[128][LM_FONT_GLYPH_HEIGHT] = {")
    table = ["{0,0,0,0,0}"] * 128
    for ch, rows in FONT.items():
        bits = rows_to_bits(rows)
        idx = ord(ch)
        table[idx] = "{" + ",".join(str(b) for b in bits) + "}"
    for i in range(128):
        comment = ""
        if 32 <= i < 127:
            comment = f"  // 0x{i:02X} '{chr(i)}'"
        else:
            comment = f"  // 0x{i:02X}"
        lines.append(f"    {table[i]},{comment}")
    lines.append("};")
    lines.append("")

    with open("/home/claude/led_matrix_gfx/font3x5.h", "w") as f:
        f.write("\n".join(lines))
    print("Wrote font3x5.h with", len(FONT), "glyphs defined (others default to blank).")

if __name__ == "__main__":
    main()
