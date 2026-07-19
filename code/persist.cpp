#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "controls.h"
#include "persist.h"

static const char *TAG = "persist";

#define NVS_NAMESPACE  "fxpedal"
#define NVS_KEY        "dials"

/* Bump SETTINGS_MAGIC whenever you add/remove/reorder panels or dials.
 * Old blobs are then rejected and defaults are used instead. */
#define SETTINGS_MAGIC 0x46585031u   /* "FXP1" */
#define MAX_DIALS      64

/* main.cpp */
extern Panel s_panels[];
extern int s_panel_count;

typedef struct {
    uint32_t magic;
    uint16_t total;          /* total dial count — layout sanity check */
    uint16_t panel;          /* last-selected page */
    float    vals[MAX_DIALS];
} settings_blob_t;

static int total_dials(void)
{
    int t = 0;
    for (int p = 0; p < s_panel_count; ++p) t += s_panels[p].count();
    return t;
}

esp_err_t settings_save(int cur_panel)
{
    settings_blob_t b = {};
    b.magic = SETTINGS_MAGIC;
    b.total = (uint16_t)total_dials();
    b.panel = (uint16_t)cur_panel;

    int k = 0;
    for (int p = 0; p < s_panel_count; ++p)
        for (int d = 0; d < s_panels[p].count(); ++d)
            b.vals[k++] = s_panels[p].dial(d);

    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;

    /* only the used portion of the blob */
    size_t sz = offsetof(settings_blob_t, vals) + k * sizeof(float);
    e = nvs_set_blob(h, NVS_KEY, &b, sz);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);

    if (e == ESP_OK) ESP_LOGI(TAG, "saved %d dials", k);
    else             ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(e));
    return e;
}

void settings_load(int *out_panel)
{
    *out_panel = 0;

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no saved settings (first boot)");
        return;
    }

    settings_blob_t b = {};
    size_t sz = sizeof(b);
    esp_err_t e = nvs_get_blob(h, NVS_KEY, &b, &sz);
    nvs_close(h);

    if (e != ESP_OK) {
        ESP_LOGI(TAG, "no saved settings (%s)", esp_err_to_name(e));
        return;
    }
    if (b.magic != SETTINGS_MAGIC || b.total != (uint16_t)total_dials()) {
        ESP_LOGW(TAG, "saved layout mismatch (old firmware?) — using defaults");
        return;
    }

    int k = 0;
    for (int p = 0; p < s_panel_count; ++p)
        for (int d = 0; d < s_panels[p].count(); ++d)
            s_panels[p].set(d, b.vals[k++]);   /* clamped setter, see below */

    *out_panel = (b.panel < s_panel_count) ? b.panel : 0;
    ESP_LOGI(TAG, "restored %d dials, page %d", k, *out_panel);
}