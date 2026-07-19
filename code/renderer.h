#pragma once
#include "controls.h"

enum snr_state_t { SNR_OFF, SNR_CAL, SNR_SHOW };
// typedef struct {
// 	enum 
// } ui_params_t;
void draw_screen_body(void);
void draw_level_meter(void);
void draw_snr(snr_state_t st, float noise_rms, float snr_db, float snr_peak);
