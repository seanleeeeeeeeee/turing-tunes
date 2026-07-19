#pragma once
#include <stdint.h>

#define LCD_W 128
#define LCD_H 160

/* RGB565 helpers */
#define RGB(r,g,b)  ((uint16_t)((((r)&0xF8)<<8) | (((g)&0xFC)<<3) | ((b)>>3)))
#define C_BLACK  RGB(0,0,0)
#define C_WHITE  RGB(255,255,255)
#define C_GREY   RGB(110,110,110)
#define C_CYAN   RGB(0,200,220)
#define C_YELLOW RGB(250,210,0)
#define C_RED    RGB(230,40,40)
#define C_GREEN  RGB(40,200,80)
#define C_NAVY   RGB(10,20,60)

void lcd_init(void);
void lcd_clear(uint16_t color);
void lcd_fill_rect(int x, int y, int w, int h, uint16_t color);
void lcd_text(int x, int y, const char *s, uint16_t fg, uint16_t bg, int scale);
void lcd_flush(void);                 /* push framebuffer to panel */
void lcd_backlight(bool on);
void lcd_draw_map(int x, int y, int w, int h, int scale, int tile);
void lcd_flush_rows(int y0, int y1);