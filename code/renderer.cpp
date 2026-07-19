#include <math.h>
#include <string.h>
#include "esp_err.h"
#include "renderer.h"
#include "lcd.h"
#include "controls.h"

extern Panel s_panels[];
extern int s_panel_count;

extern volatile int      ui_panel;
extern volatile int      ui_sel;
extern volatile uint32_t ui_ver;
extern volatile float    ui_level;
extern volatile float ui_rms;
extern volatile uint32_t ui_snr_req;
extern volatile uint32_t ui_chl_req;
extern bool ui_menu;
extern bool ui_viz;

extern Panel menu_p;

#define MENU_COUNT 6

enum { P_GAIN, P_FUZZ, P_CHOR, P_FLAN, P_VIB, P_TREM, P_DELAY, P_VERB, P_CRUSH, P_COUNT };

void draw_screen_body(void)
{
    int p = ui_panel, sel = ui_sel;
    Panel &pan = s_panels[p];

    lcd_clear(C_BLACK);

    lcd_fill_rect(0, 0, LCD_W, 18, C_NAVY);
    lcd_text(4, 2, pan.name(), C_CYAN, C_NAVY, 2);
    char page[16];
    snprintf(page, sizeof page, "%d/%d", p + 1, P_COUNT);
    lcd_text(LCD_W - 4 - 6*4, 6, page, C_GREY, C_NAVY, 1);

    /* --- dial list with value bars --- */
    int y = 24;
    if (!ui_menu)
    { //normal settings
		for (int d = 0; d < pan.count(); ++d) {
			bool hot = (d == sel);
			uint16_t fg = hot ? C_YELLOW : C_WHITE;
	
			if (hot) lcd_text(0, y, ">", C_YELLOW, C_BLACK, 1);
			lcd_text(8, y, pan.label(d), fg, C_BLACK, 1);
	
			char val[12];
			float v = pan.dial(d);
			snprintf(val, sizeof val, "%.2f", v);
			lcd_text(LCD_W - 6 * (int)strlen(val) - 2, y, val, fg, C_BLACK, 1);
	
			/* bar showing position within min..max */
			/* (add tiny getters min(d)/max(d) to Panel, shown below) */
			float lo = pan.dmin(d), hi = pan.dmax(d);
			int bw = (int)((v - lo) / (hi - lo) * (LCD_W - 12));
			lcd_fill_rect(8, y + 9, LCD_W - 12, 3, C_GREY);
			lcd_fill_rect(8, y + 9, bw, 3, hot ? C_YELLOW : C_CYAN);
	
			y += 15;
		}
	} else
	{
		lcd_fill_rect(0, 0, LCD_W, 18, C_NAVY);
		lcd_text(4, 2, menu_p.name(), C_CYAN, C_NAVY, 2);
		for (int d = 0; d < MENU_COUNT; ++d) {
			bool hot = (d == sel);
			uint16_t fg = hot ? C_YELLOW : C_WHITE;
	
			if (hot) lcd_text(0, y, ">", C_YELLOW, C_BLACK, 1);
			lcd_text(8, y, menu_p.label(d), fg, C_BLACK, 1);
			
			char val[12];
			float v = menu_p.dial(d);
			snprintf(val, sizeof val, "%.3f", v);
			if (d == 5) {
				switch ((int)v){
					case 0: snprintf(val, sizeof val, "off"); break;
					case 1: snprintf(val, sizeof val, "CORAL"); break;	//coral
					case 2: snprintf(val, sizeof val, "PARTICLES1"); break;	//particles
					case 3: snprintf(val, sizeof val, "PARTICLES2"); break;	//particles
					case 4: snprintf(val, sizeof val, "GLIDERS"); break;	//gliders
					case 5: snprintf(val, sizeof val, "WAVES_SM"); break;	//small waves
					case 6: snprintf(val, sizeof val, "WAVES_LG"); break;	//big waves
					case 7: snprintf(val, sizeof val, "CHAOS1"); break;	//chaos
					case 8: snprintf(val, sizeof val, "CHAOS2"); break;	//chaos
					case 9: snprintf(val, sizeof val, "WORMS"); break;
				}
			}
			lcd_text(LCD_W - 6 * (int)strlen(val) - 2, y, val, fg, C_BLACK, 1);
	
			/* bar showing position within min..max */
			/* (add tiny getters min(d)/max(d) to Panel, shown below) */
			float lo = menu_p.dmin(d), hi = menu_p.dmax(d);
			int bw = (int)((v - lo) / (hi - lo) * (LCD_W - 12));
			lcd_fill_rect(8, y + 9, LCD_W - 12, 3, C_GREY);
			lcd_fill_rect(8, y + 9, bw, 3, hot ? C_YELLOW : C_CYAN);
	
			y += 15;
		}
	}
}
 
void draw_level_meter(void)
{
	float lv = ui_level;
    int mw = (int)(lv * (LCD_W - 8));
    lcd_fill_rect(4, LCD_H - 8, LCD_W - 8, 5, C_GREY);
    lcd_fill_rect(4, LCD_H - 8, mw, 5, lv > 0.9f ? C_RED : C_GREEN);
}

void draw_snr(snr_state_t st, float noise_rms, float snr_db, float snr_peak)
{
    lcd_fill_rect(0, LCD_H - 34, LCD_W, 34, C_NAVY);

    if (st == SNR_CAL) {
        lcd_text(4, LCD_H - 30, "MEASURING NOISE", C_YELLOW, C_NAVY, 1);
        lcd_text(4, LCD_H - 20, "DON'T PLAY...", C_WHITE, C_NAVY, 1);
        return;
    }

    char line[24];
    float noise_db = 20.0f * log10f(noise_rms + 1e-7f);   /* dBFS */

    snprintf(line, sizeof line, "SNR %5.1f dB", snr_db);
    lcd_text(4, LCD_H - 32, line, C_CYAN, C_NAVY, 1);
    snprintf(line, sizeof line, "PK  %5.1f  NF %5.1f", snr_peak, noise_db);
    lcd_text(4, LCD_H - 22, line, C_GREY, C_NAVY, 1);

    /* bar: 0..96 dB scale */
    int bw = (int)(snr_db / 96.0f * (LCD_W - 8));
    if (bw < 0) bw = 0;
    lcd_fill_rect(4, LCD_H - 10, LCD_W - 8, 5, C_GREY);
    lcd_fill_rect(4, LCD_H - 10, bw, 5,
                  snr_db > 60 ? C_GREEN : snr_db > 40 ? C_YELLOW : C_RED);
}
