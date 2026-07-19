#pragma once
#include "esp_err.h"

void      settings_load(int *out_panel);   /* call once at boot, before apply_all() */
esp_err_t settings_save(int cur_panel);    /* call from the control task */