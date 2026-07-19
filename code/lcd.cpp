#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lcd.h"
#include "simulation.h"

static const char *TAG = "lcd";

#define PIN_LCD_SCK   GPIO_NUM_12
#define PIN_LCD_MOSI  GPIO_NUM_11
#define PIN_LCD_CS    GPIO_NUM_10
#define PIN_LCD_DC    GPIO_NUM_13
#define PIN_LCD_RST   GPIO_NUM_14
#define PIN_LCD_BLK   GPIO_NUM_21

#define LCD_SPI_HOST  SPI2_HOST
#define LCD_SPI_HZ    (20 * 1000 * 1000)   /* ST7735S is spec'd ~15 MHz writes;
                                              20 MHz works on virtually all
                                              modules, 26 MHz on most. Drop to
                                              15 MHz if you see glitches. */

/* Some ST7735S 128x160 panels are addressed with a small offset.
 * Standard "green tab" 1.8" modules are (0,0); if your image is
 * shifted with a line of noise, try (2,1) or (1,2). */
#define COL_OFFSET 0
#define ROW_OFFSET 0

static esp_lcd_panel_io_handle_t s_io;
static uint16_t *s_fb;                     /* 128*160*2 = 40 KB, DMA-capable */
// extern float **u;
extern volatile float    ui_level;
extern volatile float	 ui_freq1;
int oct;
/* ---------------- low-level helpers ---------------- */
static void cmd(uint8_t c, const uint8_t *data, int len)
{
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_io, c, data, len));
}

/* ---------------- panel init ---------------- */
static void st7735s_init_seq(void)
{
    /* hardware reset */
    gpio_set_level(PIN_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    cmd(0x01, NULL, 0);                       /* SWRESET */
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(0x11, NULL, 0);                       /* SLPOUT  */
    vTaskDelay(pdMS_TO_TICKS(120));

    /* Frame rate: ~60 Hz in normal/idle/partial mode */
    cmd(0xB1, (uint8_t[]){0x01, 0x2C, 0x2D}, 3);   /* FRMCTR1 */
    cmd(0xB2, (uint8_t[]){0x01, 0x2C, 0x2D}, 3);   /* FRMCTR2 */
    cmd(0xB3, (uint8_t[]){0x01, 0x2C, 0x2D,
                          0x01, 0x2C, 0x2D}, 6);   /* FRMCTR3 */
    cmd(0xB4, (uint8_t[]){0x07}, 1);               /* INVCTR: no inversion */

    /* Power sequence */
    cmd(0xC0, (uint8_t[]){0xA2, 0x02, 0x84}, 3);   /* PWCTR1 */
    cmd(0xC1, (uint8_t[]){0xC5}, 1);               /* PWCTR2 */
    cmd(0xC2, (uint8_t[]){0x0A, 0x00}, 2);         /* PWCTR3 */
    cmd(0xC3, (uint8_t[]){0x8A, 0x2A}, 2);         /* PWCTR4 */
    cmd(0xC4, (uint8_t[]){0x8A, 0xEE}, 2);         /* PWCTR5 */
    cmd(0xC5, (uint8_t[]){0x0E}, 1);               /* VMCTR1 */

    cmd(0x20, NULL, 0);                            /* INVOFF */

    /* MADCTL: row/col order + RGB/BGR.
     * 0xC0 = portrait, connector at bottom, RGB order.
     * If red and blue are swapped on your module, use 0xC8 (BGR bit). */
    cmd(0x36, (uint8_t[]){0xC0}, 1);

    cmd(0x3A, (uint8_t[]){0x05}, 1);               /* COLMOD: 16-bit RGB565 */

    /* Gamma (the usual ST7735S "green tab" tables) */
    cmd(0xE0, (uint8_t[]){0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,
                          0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10}, 16);
    cmd(0xE1, (uint8_t[]){0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,
                          0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10}, 16);

    cmd(0x13, NULL, 0);                            /* NORON */
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd(0x29, NULL, 0);                            /* DISPON */
    vTaskDelay(pdMS_TO_TICKS(10));
}

void lcd_init(void)
{
    /* GPIOs for reset + backlight */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LCD_RST) | (1ULL << PIN_LCD_BLK),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_LCD_BLK, 0);       /* keep dark during init */

    /* SPI bus — dedicated to the LCD */
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_LCD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * LCD_H * 2 + 16,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num       = PIN_LCD_CS,
        .dc_gpio_num       = PIN_LCD_DC,
        .spi_mode          = 0,
        .pclk_hz           = (25 * 1000 * 1000),
        .trans_queue_depth = 4,
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_cfg, &s_io));

    /* Framebuffer must be internal + DMA-capable (not PSRAM) */
    s_fb = (uint16_t *)heap_caps_malloc(LCD_W * LCD_H * 2, MALLOC_CAP_DMA);
    configASSERT(s_fb);

    st7735s_init_seq();
    lcd_clear(C_BLACK);
    lcd_flush();
    gpio_set_level(PIN_LCD_BLK, 1);
    ESP_LOGI(TAG, "ST7735S ready (%dx%d @ %d MHz)", LCD_W, LCD_H, LCD_SPI_HZ/1000000);
}

void lcd_backlight(bool on) { gpio_set_level(PIN_LCD_BLK, on); }

/* ---------------- framebuffer drawing ----------------
 * The panel expects big-endian RGB565; we store bytes pre-swapped
 * in the framebuffer so the flush is one straight DMA blast.       */
static inline uint16_t swap16(uint16_t v) { return (v >> 8) | (v << 8); }

void lcd_clear(uint16_t color)
{
    uint16_t c = swap16(color);
    for (int i = 0; i < LCD_W * LCD_H; ++i) s_fb[i] = c;
}

void lcd_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_W) w = LCD_W - x;
    if (y + h > LCD_H) h = LCD_H - y;
    if (w <= 0 || h <= 0) return;
    uint16_t c = swap16(color);
    for (int j = 0; j < h; ++j) {
        uint16_t *row = &s_fb[(y + j) * LCD_W + x];
        for (int i = 0; i < w; ++i) row[i] = c;
    }
}

static uint8_t hot_r, hot_g, hot_b;
static uint8_t warm_r, warm_g, warm_b;

static uint16_t warm_col;
static uint16_t gradient(float x) {
    // Clamp x to [0, 1]
//     if (x < 0.0) x = 0.0;
//     if (x > 1.0) x = 1.0;
    uint8_t r, g, b;
    // Yellow -> Orange -> Red -> Purple -> Indigo/Dark Blue
//     x = x * 1.
//     float min = (1 - ui_level) * 0.3f; // lvl = 0, min = 0.5 (no yellow) lvl = 1, min = 0 (all colors)
//     x = x * (1.0f - min) + min;	
//     float max = ui_level * -0.5f + 1.0f; // lvl = 0, max = 1 (all) lvl = 1, max = 0.5 (no dark)
//     x = x * max;
    
    if (x < 0.3f) {
        // Yellow (255,255,0) to Orange (255,140,0)
        float t = (x - 0.15f) / 0.15f;
        r = hot_r + ((warm_r - hot_r) * t);
        g = hot_g + ((warm_g - hot_g) * t);
        b = hot_b + ((warm_b - hot_b) * t);
//         r = hot_r;
//         g = hot_g;
//         b = hot_b;
//         r = (uint8_t)(0 - t * (0 - 255));
//         g = (uint8_t)(255 - t * (255 - 140));
//         b = (uint8_t)(139 - t * (139 - 0));
//     	ESP_LOGI(TAG, "x=%.2f lvl=%.2f rgb= %d %d %d", x, ui_level, r, g, b);
    }
    else if (x < 0.5f) {
        // Orange (255,140,0) to Red (200,30,30)
        float t = (x - 0.3f) / 0.2f;
        r = (warm_r - t * (warm_r - 200));
        g = (warm_g - t * (warm_g - 30));
        b = (warm_b - (t * (warm_b - 30)));
    }
    else if (x < 0.75f) {
        // Red (200,30,30) to Purple (120,40,150)
        float t = (x - 0.5f) / 0.25f;
        r = (uint8_t)(200 - t * (200 - 120));
        g = (uint8_t)(30 + t * (40 - 30));
        b = (uint8_t)(30 + t * (150 - 30));
    }
    else {
        // Purple (120,40,150) to Dark Indigo (40,20,90)
        float t = (x - 0.75f) / 0.25f;
        r = (uint8_t)(120 - t * (120 - 0));
        g = (uint8_t)(40 - t * (40 - 0));
        b = (uint8_t)(150 - t * (150 - 0));
    }
    return RGB(r, g, b);
}
static inline uint16_t mapDoubleToRGB565(float x) {
    // Clamp the input to the 0.0 to 1.0 range
//     if (x < 0.0f) x = 0.0f;
//     if (x > 1.0f) x = 1.0f;

    // Scale double to 8-bit intensity (0-255)
    uint8_t intensity = (uint8_t)(x * 255.0f);

    // Map to 5-6-5 bits
    uint16_t r = (intensity >> 3) & 0x1F;  // 5 bits
    uint16_t g = (intensity >> 2) & 0x3F;  // 6 bits
    uint16_t b = (intensity >> 3) & 0x1F;  // 5 bits

    // Pack into a uint16_t
    return (r << 11) | (g << 5) | b;
}

static void color_wheel(float hue, uint8_t* r, uint8_t* g, uint8_t* b) {
	float s = 100.0f; float v = 50.0f;
    float c = v * s;
    float hPrime = hue / 60.0f;
    float x = c * (1 - abs(fmod(hPrime, 2.0f) - 1));
    float m = v - c;
    float rPrime = 0.0f; float gPrime = 0.0f; float bPrime = 0.0f;
    
    if (hPrime >= 0 && hPrime < 1)      { rPrime = c; gPrime = x; bPrime = 0; } 
    else if (hPrime >= 1 && hPrime < 2) { rPrime = x; gPrime = c; bPrime = 0; } 
    else if (hPrime >= 2 && hPrime < 3) { rPrime = 0; gPrime = c; bPrime = x; } 
    else if (hPrime >= 3 && hPrime < 4) { rPrime = 0; gPrime = x; bPrime = c; } 
    else if (hPrime >= 4 && hPrime < 5) { rPrime = x; gPrime = 0; bPrime = c; } 
    else if (hPrime >= 5 && hPrime < 6) { rPrime = c; gPrime = 0; bPrime = x; }
    
    *r = round((rPrime + m) * 255.0f);
	*g = round((gPrime + m) * 255.0f);
	*b = round((bPrime + m) * 255.0f);
}

void lcd_draw_map(int x, int y, int w, int h, int scale, int tile)
{
	if (ui_level > threshold) {
		float note = ui_freq1;
		oct = 0;
		while (note > 120.0f) {
			note /= 2.0f;
			oct++;
		}
		note_pos = (int)note - 40;
		float h1, h2, s, v;
		h1 = note * 6.0f - 360.0f;
		h2 = (h1 < 330.0f) ? (h1 + 30.0f) : (h1 - 330.0f);
		color_wheel(h1, &hot_r, &hot_g, &hot_b);
		color_wheel(h2, &warm_r, &warm_g, &warm_b);
		ESP_LOGI(TAG, "note=%.2f/%d h=%.2f (%d %d %d) h=%.2f (%d %d %d)", note, oct, h1, hot_r, hot_g, hot_b, h2, warm_r, warm_g, warm_b);
	} else {
// 		hot_r = 255; hot_g = 255, hot_b = 0;
// 		warm_r =255; warm_g= 140; warm_b= 0;
		hot_r = 50; hot_g = 50, hot_b = 50;
		warm_r =100; warm_g= 100; warm_b= 100;
	}
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_W) w = LCD_W - x;
    if (y + h > LCD_H) h = LCD_H - y;
    if (w <= 0 || h <= 0) return;

    int tile_size = w / tile;        // pixel size of one repeat period
    int cells     = tile_size / scale; // map cells per period

    static uint16_t rowbuf[LCD_W];   // one fully built row, reused
    int cached_cy = -1;              // which map-row is currently in rowbuf

    for (int j = 0; j < h; ++j) {
        int cy = (j % tile_size) / scale;

        if (cy != cached_cy) {
            // Build ONE period of the row (cells values, each expanded to `scale` px)
            for (int cx = 0; cx < cells; ++cx) {
                uint16_t c = swap16(gradient(u[cx][cy]));
                uint16_t *dst = &rowbuf[cx * scale];
                for (int k = 0; k < scale; ++k) dst[k] = c;
            }
            // Replicate that period across the rest of the tiles
            for (int t = 1; t < tile; ++t)
                memcpy(&rowbuf[t * tile_size], rowbuf, tile_size * sizeof(uint16_t));

            // Handle any leftover pixels if w isn't an exact multiple of tile_size
            int done = tile * tile_size;
            if (done < w)
                memcpy(&rowbuf[done], rowbuf, (w - done) * sizeof(uint16_t));

            cached_cy = cy;
        }

        uint16_t *row = &s_fb[(y + j) * LCD_W + x];
        memcpy(row, rowbuf, w * sizeof(uint16_t));
    }
}
// void lcd_draw_map(int x, int y, int w, int h, int scale, int tile)
// {
//     if (x < 0) { w += x; x = 0; }
//     if (y < 0) { h += y; y = 0; }
//     if (x + w > LCD_W) w = LCD_W - x;
//     if (y + h > LCD_H) h = LCD_H - y;
//     if (w <= 0 || h <= 0) return;
//     for (int j = 0; j < h; ++j) {
//         uint16_t *row = &s_fb[(y + j) * LCD_W + x];
//         for (int i = 0; i < w; ++i) {
//         	uint16_t c = swap16(gradient(u[(i % (w/tile))/scale][(j % (w/tile))/scale]));
//         	row[i] = c;
//         }
//     }
// }
// void lcd_draw_map(int x, int y, int w, int h, int scale, int tile)
// {
//     for (int j = 0; j < HEIGHT; ++j) {    0 = 0,1,64,65  1 = 2,3,66,67 2 = 4,5,68,69
//     	u[i][j]
//         uint16_t *row = &s_fb[(y + j * 2) * LCD_W + x];
//         for (int i = 0; i < WIDTH; ++i) {
//         	uint16_t c = swap16(gradient(u[i][j]));
//         	row[i*2] = c;
//         	row[i*2+1] = c;
//         	row[i*2+64] = c;
//         	row[i*2+65] = c;
//         }
//         row = &s_fb[(y + j * 2 + 1) * LCD_W + x];
//         row = &s_fb[(y + j * 2 + 64) * LCD_W + x];
//         row = &s_fb[(y + j * 2 + 65) * LCD_W + x];
// 
// 
//     }	
//     for (int j = 0; j < h; ++j) {
//         uint16_t *row = &s_fb[(y + j) * LCD_W + x];
//         for (int i = 0; i < w; ++i) {
//         	uint16_t c = swap16(gradient(u[(i % (w/tile))/scale][(j % (w/tile))/scale]));
//         	row[i] = c;
//         }
//     }
// }

/* Classic 5x7 font, ASCII 32..126, column-major bits */
static const uint8_t font5x7[95][5] = {
 {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
 {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
 {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
 {0x00,0x41,0x22,0x1C,0x00},{0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
 {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
 {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
 {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
 {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
 {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
 {0x00,0x56,0x36,0x00,0x00},{0x00,0x08,0x14,0x22,0x41},{0x14,0x14,0x14,0x14,0x14},
 {0x41,0x22,0x14,0x08,0x00},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
 {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
 {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
 {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
 {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
 {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
 {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
 {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
 {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
 {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
 {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},
 {0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
 {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
 {0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
 {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
 {0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
 {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
 {0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
 {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},
 {0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
 {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},
 {0x00,0x41,0x36,0x08,0x00},{0x08,0x04,0x08,0x10,0x08},
};

void lcd_text(int x, int y, const char *s, uint16_t fg, uint16_t bg, int scale)
{
    uint16_t f = swap16(fg), b = swap16(bg);
    for (; *s; ++s) {
        char c = (*s < 32 || *s > 126) ? '?' : *s;
        const uint8_t *g = font5x7[c - 32];
        for (int col = 0; col < 6; ++col) {              /* 5 cols + 1 spacing */
            uint8_t bits = (col < 5) ? g[col] : 0;
            for (int row = 0; row < 8; ++row) {
                uint16_t px = (bits & (1 << row)) ? f : b;
                for (int sy = 0; sy < scale; ++sy)
                    for (int sx = 0; sx < scale; ++sx) {
                        int px_x = x + col * scale + sx;
                        int px_y = y + row * scale + sy;
                        if (px_x >= 0 && px_x < LCD_W && px_y >= 0 && px_y < LCD_H)
                            s_fb[px_y * LCD_W + px_x] = px;
                    }
            }
        }
        x += 6 * scale;
    }
}

void lcd_flush(void)
{
    /* CASET / RASET: full window, then RAMWR the whole framebuffer */
    uint8_t ca[] = {0, COL_OFFSET, 0, COL_OFFSET + LCD_W - 1};
    uint8_t ra[] = {0, ROW_OFFSET, 0, ROW_OFFSET + LCD_H - 1};
    cmd(0x2A, ca, 4);
    cmd(0x2B, ra, 4);
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(s_io, 0x2C, s_fb, LCD_W * LCD_H * 2));
}

void lcd_flush_rows(int y0, int y1)  /* inclusive range */
{
    uint8_t ca[] = {0, (uint8_t)COL_OFFSET, 0, (uint8_t)(COL_OFFSET + LCD_W - 1)};
    uint8_t ra[] = {0, (uint8_t)(ROW_OFFSET + y0), 0, (uint8_t)(ROW_OFFSET + y1)};
    cmd(0x2A, ca, 4);
    cmd(0x2B, ra, 4);
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(
        s_io, 0x2C,
        s_fb + y0 * LCD_W,               /* if s_fb is uint16_t* */
        (y1 - y0 + 1) * LCD_W * 2));
}