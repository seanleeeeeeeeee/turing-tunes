#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"

#include "stick.h"
#include "controls.h"
#include "lcd.h"
#include "persist.h"
#include "renderer.h"
#include "simulation.h"


static const char *TAG = "wm8960";

#define PIN_I2C_SDA   GPIO_NUM_8
#define PIN_I2C_SCL   GPIO_NUM_9

#define PIN_I2S_MCLK  GPIO_NUM_15
#define PIN_I2S_BCLK  GPIO_NUM_16
#define PIN_I2S_WS    GPIO_NUM_17
#define PIN_I2S_DOUT  GPIO_NUM_18
#define PIN_I2S_DIN   GPIO_NUM_4

#define WM8960_I2C_ADDR  0x1a
#define SAMPLE_RATE_HZ   44100
#define FRAMES_PER_BUF   256

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_wm8960;
static i2s_chan_handle_t       s_tx;
static i2s_chan_handle_t       s_rx;

volatile int      ui_panel = 0;
volatile int      ui_sel = 0;
volatile uint32_t ui_ver = 0;
volatile float    ui_level = 0.0f;
volatile float ui_rms = 0.0f;
volatile uint32_t ui_snr_req = 0;
volatile uint32_t ui_chl_req = 0;
volatile bool ui_menu = false;
volatile bool ui_viz = false;
volatile int32_t	ui_gs_req = 0;

// float **u = NULL;
float F = 0.015;
float k = 0.048;
int steps = 20;
int rad = 2;

static void i2c_setup(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = PIN_I2C_SDA,
        .scl_io_num        = PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 0,
        .flags = { .enable_internal_pullup = true },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = WM8960_I2C_ADDR,
        .scl_speed_hz    = 1000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_wm8960));
}

static void i2s_setup(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx, &s_rx));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE_HZ,
            .clk_src        = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = PIN_I2S_MCLK,
            .bclk = PIN_I2S_BCLK,
            .ws   = PIN_I2S_WS,
            .dout = PIN_I2S_DOUT,
            .din  = PIN_I2S_DIN,
            .invert_flags = {0},
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx));
}

static esp_err_t wm8960_write(uint8_t reg, uint16_t value)
{
    uint8_t buf[2] = {
        (uint8_t)((reg << 1) | ((value >> 8) & 0x01)),
        (uint8_t)(value & 0xFF),
    };
    esp_err_t e = ESP_FAIL;
    for (int attempt = 0; attempt < 10; ++attempt) {
        e = i2c_master_transmit(s_wm8960, buf, 2, 100);
        if (e == ESP_OK) {
        	if (attempt > 0) {
        		ESP_LOGW(TAG, "  (took %d retries)", attempt);
        	}
        	return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return e;
}

#define WM_W(reg, val) do {                                                    \
    uint16_t _v = (uint16_t)(val);                                             \
    esp_err_t _e = wm8960_write((reg), _v);                                    \
    if (_e != ESP_OK) {                                                        \
        ESP_LOGE(TAG, "reg 0x%02X <- 0x%03X  FAIL  (%s)",                      \
                 (reg), _v, esp_err_to_name(_e));                              \
        return _e;                                                             \
    }                                                                          \
    ESP_LOGI(TAG, "reg 0x%02X <- 0x%03X  ok", (reg), _v);					\
} while (0)

static esp_err_t codec_set_pga(float dB)
{
    if (dB < -17.25f) dB = -17.25f;
    if (dB >  30.0f)  dB =  30.0f;
    uint8_t code = (uint8_t)lroundf((dB + 17.25f) / 0.75f);
    WM_W(0x01, (1<<8) | (1<<6) | code);
    return ESP_OK;
}
static esp_err_t codec_set_adc(float dB)
{
    if (dB < -97.0f) dB = -97.0f;
    if (dB >  30.0f) dB =  30.0f;
    uint8_t code = (uint8_t)(lroundf(dB / 0.5f) + 195);
    WM_W(0x16, 0x100 | code);
    return ESP_OK;
}
static esp_err_t codec_set_dac(float dB)
{
    if (dB < -127.0f) dB = -127.0f;
    if (dB >    0.0f) dB =    0.0f;
    uint8_t code = (uint8_t)(lroundf(dB / 0.5f) + 255);
    WM_W(0x0A, 0x100 | code);
    WM_W(0x0B, 0x100 | code);
    return ESP_OK;
}
static esp_err_t codec_set_hp(float dB)
{
    if (dB < -73.0f) dB = -73.0f;
    if (dB >   6.0f) dB =   6.0f;
    uint8_t code = (uint8_t)(lroundf(dB) + 121);
    WM_W(0x02, 0x100 | code);
    WM_W(0x03, 0x100 | code);
    return ESP_OK;
}
static esp_err_t codec_set_micboost(int boost)
{
    if (boost < 0) boost = 0;
    if (boost > 3) boost = 3;
    WM_W(0x21, (1<<8) | (boost<<4) | (1<<3));
    return ESP_OK;
}

static esp_err_t wm8960_setup(float pga, float adc, float dac, float hp, int micboost)
{
    WM_W(0x0F, 0x000);                              /* software reset */
    vTaskDelay(pdMS_TO_TICKS(10));

    WM_W(0x19, (1<<8)|(1<<7)|(1<<6)|(1<<4)|(1<<2)); /* VMID charge */
    vTaskDelay(pdMS_TO_TICKS(100));
    WM_W(0x19, (1<<7)|(1<<6)|(1<<4)|(1<<2));        /* normal */

    WM_W(0x1A, (1<<8)|(1<<7)|(1<<6));               /* DACL/R + OUT1 */
    WM_W(0x2F, (1<<4)|(1<<3));                      /* RMIC + mixers */
    WM_W(0x04, 0x000);
    WM_W(0x05, 0x000);
    WM_W(0x07, 0x002);                              /* I2S 16-bit slave */

    ESP_ERROR_CHECK(codec_set_micboost(micboost));
    ESP_ERROR_CHECK(codec_set_pga(pga));
    ESP_ERROR_CHECK(codec_set_adc(adc));
    ESP_ERROR_CHECK(codec_set_dac(dac));

    WM_W(0x22, (1<<8));                             /* LD2LO */
    WM_W(0x25, (1<<8));                             /* RD2RO */

    ESP_ERROR_CHECK(codec_set_hp(hp));

    WM_W(0x2C, (7<<4));
    WM_W(0x18, (1<<6));                             /* ADCLRC = DACLRC */
    WM_W(0x30, 0x009);

    ESP_LOGI(TAG, "WM8960 setup complete");
    return ESP_OK;
}

/* ================================================================
 *  Effect parameters — written by control task, read by DSP task.
 *  32-bit aligned loads/stores are atomic on ESP32, so plain
 *  volatile fields are safe for single-writer/single-reader use.
 * ================================================================ */
typedef struct {
    volatile bool  fuzz_on;   volatile float fuzz_drive, fuzz_level;
    volatile bool  chor_on;   volatile float chor_rate,  chor_depth_ms, chor_mix;
    volatile bool  flan_on;   volatile float flan_rate,  flan_depth_ms, flan_fb, flan_mix;
    volatile bool  vib_on;    volatile float vib_rate,   vib_depth_ms;
    volatile bool  trem_on;   volatile float trem_rate,  trem_depth;
    volatile bool  delay_on;  volatile int   delay_samps; volatile float delay_fb, delay_mix;
    volatile bool  verb_on;   volatile float verb_size,  verb_damp, verb_mix;
    volatile bool  crush_on;  volatile float crush_q;    volatile int   crush_ds;
} fx_params_t;

static fx_params_t fx;

/* ================================================================
 *  DSP state
 * ================================================================ */
#define DELAY_MAX_SAMPS  (SAMPLE_RATE_HZ)          /* up to 1 s of delay */
#define CHOR_BUF   2048
#define FLAN_BUF    512
#define VIB_BUF     512
#define LFO_N       512

static int16_t *s_delay_buf;                        /* big -> heap/PSRAM */
static int      s_delay_w;

static float s_chor_buf[CHOR_BUF]; static int s_chor_w;
static float s_flan_buf[FLAN_BUF]; static int s_flan_w;
static float s_vib_buf [VIB_BUF];  static int s_vib_w;
static float s_chor_ph, s_flan_ph, s_vib_ph, s_trem_ph;

static float s_sin[LFO_N];

/* Freeverb-style reverb (mono): 4 damped combs + 2 allpasses */
static const int s_comb_len[4] = {1116, 1188, 1277, 1356};
static const int s_ap_len[2]   = {556, 225};
static float *s_comb[4]; static int s_comb_i[4]; static float s_comb_lp[4];
static float *s_ap[2];   static int s_ap_i[2];

/* Bitcrusher */
static int   s_crush_cnt;
static float s_crush_held;

static void fx_init(void)
{
    s_delay_buf = (int16_t *)heap_caps_calloc(DELAY_MAX_SAMPS, sizeof(int16_t),
                                              MALLOC_CAP_SPIRAM);
    if (!s_delay_buf)
        s_delay_buf = (int16_t *)heap_caps_calloc(DELAY_MAX_SAMPS, sizeof(int16_t),
                                                  MALLOC_CAP_8BIT);
    configASSERT(s_delay_buf);

    for (int c = 0; c < 4; ++c) {
        s_comb[c] = (float *)heap_caps_calloc(s_comb_len[c], sizeof(float), MALLOC_CAP_8BIT);
        configASSERT(s_comb[c]);
    }
    for (int a = 0; a < 2; ++a) {
        s_ap[a] = (float *)heap_caps_calloc(s_ap_len[a], sizeof(float), MALLOC_CAP_8BIT);
        configASSERT(s_ap[a]);
    }
    for (int i = 0; i < LFO_N; ++i)
        s_sin[i] = sinf(2.0f * (float)M_PI * i / LFO_N);
}

static inline float lfo_tick(float *phase, float rate_hz)
{
    *phase += rate_hz / SAMPLE_RATE_HZ;
    if (*phase >= 1.0f) *phase -= 1.0f;
    return s_sin[(int)(*phase * LFO_N) & (LFO_N - 1)];
}

/* interpolated read `d` samples behind write index `w` in circular buf */
static inline float frac_read(const float *b, int size, int w, float d)
{
    float rp = (float)w - d;
    while (rp < 0) rp += size;
    int i0 = (int)rp;
    float fr = rp - (float)i0;
    int i1 = i0 + 1; if (i1 >= size) i1 = 0;
    return b[i0] + (b[i1] - b[i0]) * fr;
}

/* ---- Full chain: fuzz -> chorus -> flanger -> vibrato -> tremolo
 *                  -> delay -> reverb -> bitcrusher              ---- */
static inline float fx_process(float x)
{
    /* 1. Fuzz / distortion (soft clip) */
    if (fx.fuzz_on) {
        float d = x * fx.fuzz_drive;
        d = d / (1.0f + fabsf(d));               /* fast soft saturation */
        x = d * fx.fuzz_level;
    }

    /* 2. Chorus */
    s_chor_buf[s_chor_w] = x;
    if (fx.chor_on) {
        float dep = fx.chor_depth_ms * SAMPLE_RATE_HZ / 1000.0f;
        float d = dep + 1.0f + dep * lfo_tick(&s_chor_ph, fx.chor_rate);
        float wet = frac_read(s_chor_buf, CHOR_BUF, s_chor_w, d);
        x = x + wet * fx.chor_mix;
    }
    if (++s_chor_w >= CHOR_BUF) s_chor_w = 0;

    /* 3. Flanger (short delay, feedback) */
    {
        float wet = 0.0f;
        if (fx.flan_on) {
            float dep = fx.flan_depth_ms * SAMPLE_RATE_HZ / 1000.0f;
            float d = dep + 1.0f + dep * lfo_tick(&s_flan_ph, fx.flan_rate);
            wet = frac_read(s_flan_buf, FLAN_BUF, s_flan_w, d);
        }
        s_flan_buf[s_flan_w] = x + wet * fx.flan_fb;
        if (++s_flan_w >= FLAN_BUF) s_flan_w = 0;
        if (fx.flan_on) x = x + wet * fx.flan_mix;
    }

    /* 4. Vibrato (100% wet modulated delay = pitch wobble) */
    s_vib_buf[s_vib_w] = x;
    if (fx.vib_on) {
        float dep = fx.vib_depth_ms * SAMPLE_RATE_HZ / 1000.0f;
        float d = dep + 1.0f + dep * lfo_tick(&s_vib_ph, fx.vib_rate);
        x = frac_read(s_vib_buf, VIB_BUF, s_vib_w, d);
    }
    if (++s_vib_w >= VIB_BUF) s_vib_w = 0;

    /* 5. Tremolo */
    if (fx.trem_on) {
        float g = 1.0f - fx.trem_depth * (0.5f + 0.5f * lfo_tick(&s_trem_ph, fx.trem_rate));
        x *= g;
    }

    /* 6. Delay / echo */
    {
        int ds = fx.delay_samps;
        if (ds < 1) ds = 1;
        if (ds > DELAY_MAX_SAMPS - 1) ds = DELAY_MAX_SAMPS - 1;
        int r = s_delay_w - ds; if (r < 0) r += DELAY_MAX_SAMPS;
        float delayed = s_delay_buf[r] / 32768.0f;
        float wr = x + delayed * fx.delay_fb;
        if (wr >  0.999f) wr =  0.999f;
        if (wr < -0.999f) wr = -0.999f;
        s_delay_buf[s_delay_w] = (int16_t)(wr * 32767.0f);
        if (++s_delay_w >= DELAY_MAX_SAMPS) s_delay_w = 0;
        if (fx.delay_on) x = x + delayed * fx.delay_mix;
    }

    /* 7. Reverb */
    if (fx.verb_on) {
        float in = x * 0.3f;
        float wet = 0.0f;
        float size = fx.verb_size, damp = fx.verb_damp;
        for (int c = 0; c < 4; ++c) {
            float y = s_comb[c][s_comb_i[c]];
            s_comb_lp[c] = y + (s_comb_lp[c] - y) * damp;
            s_comb[c][s_comb_i[c]] = in + s_comb_lp[c] * size;
            if (++s_comb_i[c] >= s_comb_len[c]) s_comb_i[c] = 0;
            wet += y;
        }
        wet *= 0.25f;
        for (int a = 0; a < 2; ++a) {
            float d = s_ap[a][s_ap_i[a]];
            s_ap[a][s_ap_i[a]] = wet + d * 0.5f;
            wet = d - wet * 0.5f;
            if (++s_ap_i[a] >= s_ap_len[a]) s_ap_i[a] = 0;
        }
        x = x + wet * fx.verb_mix;
    }

    /* 8. Bitcrusher / lo-fi */
    if (fx.crush_on) {
        if (--s_crush_cnt <= 0) {
            s_crush_cnt = fx.crush_ds;
            float q = fx.crush_q;                /* precomputed 2^(bits-1) */
            s_crush_held = floorf(x * q) / q;
        }
        x = s_crush_held;
    }

    return x;
}
/* ---- autocorrelation pitch estimator ---------------------------------- */
#define AC_DECIM     4                        /* 48k -> 12k analysis rate  */
#define AC_SR        (SAMPLE_RATE_HZ / AC_DECIM)
#define AC_N         512                      /* ~43 ms window @ 12 kHz    */
#define AC_LAG_MIN   (AC_SR / 1200)           /* highest freq ~1.2 kHz     */
#define AC_LAG_MAX   (AC_SR / 60)             /* lowest  freq ~60 Hz       */

volatile float ui_freq1 = 0.0f;
volatile float ui_freq2 = 0.0f;

static float ac_buf[AC_N];
static int   ac_pos, ac_dec;

static void ac_analyze(void)
{
    static float r[AC_LAG_MAX + 1];

    /* remove DC */
    float mean = 0;
    for (int i = 0; i < AC_N; i++) mean += ac_buf[i];
    mean /= AC_N;

    float r0 = 0;
    for (int i = 0; i < AC_N; i++) {
        ac_buf[i] -= mean;
        r0 += ac_buf[i] * ac_buf[i];
    }
    if (r0 < 1e-6f) { ui_freq1 = ui_freq2 = 0; return; }

    /* autocorrelation over the lag range of interest */
    for (int lag = AC_LAG_MIN; lag <= AC_LAG_MAX; lag++) {
        float s = 0;
        for (int i = 0; i < AC_N - lag; i++)
            s += ac_buf[i] * ac_buf[i + lag];
        r[lag] = s;
    }

    /* pass 1: strongest local peak */
    int l1 = 0; float p1 = 0.20f * r0;        /* min confidence threshold */
    for (int lag = AC_LAG_MIN + 1; lag < AC_LAG_MAX; lag++)
        if (r[lag] > p1 && r[lag] >= r[lag-1] && r[lag] >= r[lag+1]) {
            p1 = r[lag]; l1 = lag;
        }

    /* pass 2: strongest peak not near the first (±20%) */
    int l2 = 0; float p2 = 0.15f * r0;
    for (int lag = AC_LAG_MIN + 1; lag < AC_LAG_MAX; lag++) {
        if (l1 && abs(lag - l1) < l1 / 5) continue;
        if (r[lag] > p2 && r[lag] >= r[lag-1] && r[lag] >= r[lag+1]) {
            p2 = r[lag]; l2 = lag;
        }
    }

    ui_freq1 = std::max(60.0f, l1 ? (float)AC_SR / (float)l1 : 0);
    ui_freq2 = l2 ? (float)AC_SR / (float)l2 : 0;
}
/* ----------------------------------------------------------------------- */
static void dsp_task(void *arg)
{
    static int16_t buf[FRAMES_PER_BUF * 2];
    size_t br, bw;
	//bool freq = false;
    while (1) {
        i2s_channel_read(s_rx, buf, sizeof(buf), &br, portMAX_DELAY);
        int n = br / sizeof(int16_t);
        int frames = n / 2;
        float peak = 0;
        float acc  = 0;
        for (int i = 0; i < n; i += 2) {
            float x = buf[i + 1] / 32768.0f;      /* right ch = RINPUT1 */
                        /* feed decimated dry input to the analyzer */            
            if (++ac_dec >= AC_DECIM) {
            	ac_dec = 0;
            	if (ac_pos < AC_N) ac_buf[ac_pos++] = x;
            }
            float a = fabsf(x);
            if (a > peak) peak = a;
            //acc += x * x;
            x = fx_process(x);
            if (x >  0.999f) x =  0.999f;
            if (x < -0.999f) x = -0.999f;
            int16_t o = (int16_t)(x * 32767.0f);
            buf[i] = buf[i + 1] = o;
        }
        ui_level = (peak > ui_level) ? (peak) : (ui_level * 0.95f);
//      float rms = sqrtf(acc / (float)frames);
//         ui_rms = ui_rms * 0.85f + rms * 0.15f;
		if (ac_pos >= AC_N) {
           /* window full -> analyze & restart */
            ac_analyze();
            ac_pos = 0;
        }
        i2s_channel_write(s_tx, buf, br, &bw, portMAX_DELAY);
        //freq = !freq;
    }
}

/* ================================================================
 *  Panels — one page per effect
 * ================================================================ */
enum { P_GAIN, P_FUZZ, P_CHOR, P_FLAN, P_VIB, P_TREM, P_DELAY, P_VERB, P_CRUSH, P_COUNT };

static Dial gain_d[] = {
    {10.0f, -17.25f, 30.0f, 0.75f, "PGA dB"},
    { 0.0f, -97.0f,  30.0f, 1.0f,  "ADC dB"},
    { 0.0f, -127.0f,  0.0f, 1.0f,  "DAC dB"},
    {-10.0f, -73.0f,  6.0f, 1.0f,  "HP dB"},
    { 3.0f,   0.0f,   3.0f, 1.0f,  "MICBOOST"},
};
static Dial fuzz_d[] = {
    {0, 0, 1, 1, "ON"},
    {20.0f, 1.0f, 80.0f, 2.0f, "DRIVE"},
    {0.6f, 0.05f, 1.0f, 0.05f, "LEVEL"},
};
static Dial chor_d[] = {
    {0, 0, 1, 1, "ON"},
    {0.8f, 0.1f, 5.0f, 0.1f,  "RATE Hz"},
    {5.0f, 1.0f, 10.0f, 0.5f, "DEPTH ms"},
    {0.5f, 0.0f, 1.0f, 0.05f, "MIX"},
};
static Dial flan_d[] = {
    {0, 0, 1, 1, "ON"},
    {0.25f, 0.05f, 2.0f, 0.05f, "RATE Hz"},
    {2.0f,  0.5f,  5.0f, 0.25f, "DEPTH ms"},
    {0.5f,  0.0f,  0.9f, 0.05f, "FEEDBK"},
    {0.6f,  0.0f,  1.0f, 0.05f, "MIX"},
};
static Dial vib_d[] = {
    {0, 0, 1, 1, "ON"},
    {4.0f, 0.5f, 8.0f, 0.5f,  "RATE Hz"},
    {1.5f, 0.5f, 5.0f, 0.25f, "DEPTH ms"},
};
static Dial trem_d[] = {
    {0, 0, 1, 1, "ON"},
    {5.0f, 0.5f, 12.0f, 0.5f, "RATE Hz"},
    {0.7f, 0.0f,  1.0f, 0.05f,"DEPTH"},
};
static Dial delay_d[] = {
    {1, 0, 1, 1, "ON"},
    {400.0f, 50.0f, 1000.0f, 25.0f, "TIME ms"},
    {0.45f, 0.0f, 0.95f, 0.05f, "FEEDBK"},
    {0.5f,  0.0f, 1.0f,  0.05f, "MIX"},
};
static Dial verb_d[] = {
    {0, 0, 1, 1, "ON"},
    {0.85f, 0.2f, 0.98f, 0.02f, "SIZE"},
    {0.4f,  0.0f, 0.95f, 0.05f, "DAMP"},
    {0.35f, 0.0f, 1.0f,  0.05f, "MIX"},
};
static Dial crush_d[] = {
    {0, 0, 1, 1, "ON"},
    {8.0f, 2.0f, 16.0f, 1.0f, "BITS"},
    {2.0f, 1.0f, 16.0f, 1.0f, "DOWNSMP"},
};
static Dial menu_d[] = {
    {0, 0, 1, 1, "SNR"},
    {0, 0, 1, 1, "VIZ"},
    {2, 0, 15, 1, "r"},
    {0.024, 0, 1, 0.002, "F"},
    {0.050, 0, 1, 0.002, "K"},
    {0, 0, 9, 1, "Preset"},
};
//     {0.0375, 0, 1, 0.01, "F"}, good
//     {0.059, 0, 1, 0.01, "K"},

//     {0.018, 0, 1, 0.002, "F"},
//     {0.051, 0, 1, 0.002, "K"},

Panel menu_p = Panel(6, menu_d,  "Options");

Panel s_panels[P_COUNT] = {
    Panel(5, gain_d,  "GAIN"),
    Panel(3, fuzz_d,  "FUZZ"),
    Panel(4, chor_d,  "CHORUS"),
    Panel(5, flan_d,  "FLANGER"),
    Panel(3, vib_d,   "VIBRATO"),
    Panel(3, trem_d,  "TREMOLO"),
    Panel(4, delay_d, "DELAY"),
    Panel(4, verb_d,  "REVERB"),
    Panel(3, crush_d, "CRUSH"),
};
int s_panel_count = P_COUNT;

/* Push dial value into the fx params or codec */
static void apply_param(int p, int d)
{
    float v = s_panels[p].dial(d);
    switch (p) {
    case P_GAIN:
        switch (d) {
        case 0: codec_set_pga(v);           break;
        case 1: codec_set_adc(v);           break;
        case 2: codec_set_dac(v);           break;
        case 3: codec_set_hp(v);            break;
        case 4: codec_set_micboost((int)v); break;
        }
        break;
    case P_FUZZ:
        switch (d) {
        case 0: fx.fuzz_on = v > 0.5f; break;
        case 1: fx.fuzz_drive = v;     break;
        case 2: fx.fuzz_level = v;     break;
        }
        break;
    case P_CHOR:
        switch (d) {
        case 0: fx.chor_on = v > 0.5f;  break;
        case 1: fx.chor_rate = v;       break;
        case 2: fx.chor_depth_ms = v;   break;
        case 3: fx.chor_mix = v;        break;
        }
        break;
    case P_FLAN:
        switch (d) {
        case 0: fx.flan_on = v > 0.5f;  break;
        case 1: fx.flan_rate = v;       break;
        case 2: fx.flan_depth_ms = v;   break;
        case 3: fx.flan_fb = v;         break;
        case 4: fx.flan_mix = v;        break;
        }
        break;
    case P_VIB:
        switch (d) {
        case 0: fx.vib_on = v > 0.5f;   break;
        case 1: fx.vib_rate = v;        break;
        case 2: fx.vib_depth_ms = v;    break;
        }
        break;
    case P_TREM:
        switch (d) {
        case 0: fx.trem_on = v > 0.5f;  break;
        case 1: fx.trem_rate = v;       break;
        case 2: fx.trem_depth = v;      break;
        }
        break;
    case P_DELAY:
        switch (d) {
        case 0: fx.delay_on = v > 0.5f; break;
        case 1: fx.delay_samps = (int)(v * SAMPLE_RATE_HZ / 1000.0f); break;
        case 2: fx.delay_fb = v;        break;
        case 3: fx.delay_mix = v;       break;
        }
        break;
    case P_VERB:
        switch (d) {
        case 0: fx.verb_on = v > 0.5f;  break;
        case 1: fx.verb_size = v;       break;
        case 2: fx.verb_damp = v;       break;
        case 3: fx.verb_mix = v;        break;
        }
        break;
    case P_CRUSH:
        switch (d) {
        case 0: fx.crush_on = v > 0.5f;              break;
        case 1: fx.crush_q  = powf(2.0f, v - 1.0f);  break;  /* precompute here, not per-sample */
        case 2: fx.crush_ds = (int)v;                break;
        }
        break;
    }
}
static void apply_all(void)
{
    for (int p = 0; p < P_COUNT; ++p)
        for (int d = 0; d < s_panels[p].count(); ++d)
            apply_param(p, d);
}

/* ================================================================
 *  Joystick control task
 *   - 20 ms polling, dead zone 600-3400 out of 0-4095 range
 *   - X axis: select dial
 *   - Y axis: change value
 *   - button cycles to next effect
 * ================================================================ */
Joystick stick;

#define POLL_MS        20
#define REPEAT_FIRST  350   /* ms before auto-repeat kicks in   */
#define REPEAT_RATE   120   /* ms between repeats while held    */
#define AXIS_LO       600   /* dead-zone thresholds (raw ADC)   */
#define AXIS_HI      3400
#define SAVE_PERIOD_MS  60000

static inline int axis_dir(int raw, int lo, int hi)
{
    if (raw < lo) return  1;
    if (raw > hi) return -1;
    return 0;
}
static void grayscott()
{
// 	if (u != NULL) gs_free();
	init_grid();
	ui_viz = true;
	switch((int)menu_p.dial(5)){
		case 0: F = menu_p.dial(3); k = menu_p.dial(4); break;
		case 1: F = 0.040f; k = 0.060f; break;	//coral
		case 2: F = 0.018f; k = 0.051f; break;	//particles
		case 3: F = 0.022f; k = 0.053f; break;	//particles
		case 4: F = 0.010f; k = 0.050f; break;	//gliders
		case 5: F = 0.018f; k = 0.048f; break;	//small waves
		case 6: F = 0.014f; k = 0.042f; break;	//big waves
		case 7: F = 0.030f; k = 0.054f; break;	//chaos
		case 8: F = 0.024f; k = 0.050f; break;	//chaos
		case 9: F = 0.073f; k = 0.062f; break;	//chaos
	}
	rad = menu_p.dial(2);
	TickType_t now = xTaskGetTickCount();
	TickType_t fps = 0;
	vTaskDelay(pdMS_TO_TICKS(50));
    lcd_clear(C_BLACK);
    lcd_flush();
    for (int i = 0; !stick.sw(); i++) {
		if (i%200 == 0) {
			fps = now;
			now = xTaskGetTickCount();
			ESP_LOGI(TAG, "frame %d in %d", i, now - fps);
		}
        update_grid();
//         if (i%200 == 0) {
// 			ESP_LOGI(TAG, "update in %d", xTaskGetTickCount() - now);
// 		}
        if (!(i&7)) {
        	lcd_draw_map(0, 16, 128, 128, 1, 1);
        	lcd_flush_rows(16, 144);
		}
//         if (i%200 == 0) {
// 			ESP_LOGI(TAG, "lcdd in %d", xTaskGetTickCount() - now);
// 		}
//         if (i%200 == 0) {
// 			ESP_LOGI(TAG, "lcdf in %d", xTaskGetTickCount() - now);
// 		}
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    ui_viz = false;
}
static void control_task(void *arg)
{
    int panel = 0, sel = 0;
    int x_dir = 0, y_dir = 0;
    int x_timer = 0, y_timer = 0;
    int sw_state = 0, sw_cnt = 0;
    
	settings_load(&panel); //nvs -> dial structs
    apply_all(); // -> fx struct
    bool save_pending = false;
    int  save_timer   = SAVE_PERIOD_MS;
    int held_ms = 0, long_fired = 0;
    Panel active_p = s_panels[panel];
    while (1) {
        bool dirty = false;
		active_p = ui_menu ? menu_p : s_panels[panel];
        /* ---- X: dial selection ---- */
        int dx = axis_dir(stick.x(), 600, 3400);
//     	ESP_LOGI(TAG, "x = %d \t dx = %d", stick.x(), dx);
        if (dx != 0) {
            bool fire = false;
            if (dx != x_dir)               { fire = true; x_timer = REPEAT_FIRST; }
            else if ((x_timer -= POLL_MS) <= 0) { fire = true; x_timer = REPEAT_RATE; }
            if (fire) {
                int n = active_p.count();
                sel = (sel + dx + n) % n;
                dirty = true;
            }
        }
        x_dir = dx;

        /* ---- Y: value change ---- */
        int dy = axis_dir(4095 - stick.y(), 600, 3400);
    	//ESP_LOGI(TAG, "y = %d \t dy = %d", stick.y(), dy);
        if (dy != 0) {
            bool fire = false;
            if (dy != y_dir)               { fire = true; y_timer = REPEAT_FIRST; }
            else if ((y_timer -= POLL_MS) <= 0) { fire = true; y_timer = REPEAT_RATE; }
            if (fire) {
                active_p.change(sel, dy > 0);
                if (!ui_menu) apply_param(panel, sel);
                dirty = true;
            }
        }
        y_dir = dy;

		/* ---- Button: short press = page, long press = menu ---- */
        int raw_sw = stick.sw() ? 1 : 0;

        if (raw_sw) {
            held_ms += POLL_MS;
            if (held_ms >= 1000 && !long_fired) {
                long_fired = 1;
                ui_menu = true;
                dirty = true;
            }
        } else {
            if (held_ms >= 40 && !long_fired) {   /* debounced short press */
            	if (!ui_menu) {
            		panel = (panel + 1) % P_COUNT;
					sel = 0;
					dirty = true;
            	} else {
            		menu_p.change(sel, !(bool)menu_p.dial(sel));
            		switch (sel){
            		case 0: ui_snr_req++; ui_menu = false; break;
            		case 1: grayscott(); break;
            		case 2: break;
            		}
            		
            	}
            }
            held_ms = 0;
            long_fired = 0;
        }
        
        if (dirty) {
        	save_pending = true;
        	ui_panel = panel;
        	ui_sel = sel;
        	ui_ver++;
            ESP_LOGI(TAG, "[%s] %s = %.2f",
                     active_p.name(),
                     active_p.label(sel),
                     active_p.dial(sel));
        }
		save_timer -= POLL_MS;
		if (save_timer < 0) {
			save_timer = SAVE_PERIOD_MS;
			if (save_pending && ui_level < 0.05f) {
				save_pending = false;
				settings_save(panel);
			}
		}
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

static void display_task(void *arg)
{
    lcd_init();
    uint32_t last_ver = ~0u, last_req = 0;
    float last_level = -1;

    snr_state_t snr = SNR_OFF;
    TickType_t  snr_t0 = 0;
    float noise_rms = 1e-5f, noise_min = 1.0f;
    float snr_peak = 0;

    while (1) {
        TickType_t now = xTaskGetTickCount();

        if (ui_snr_req != last_req) {              /* long-press arrived */
            last_req = ui_snr_req;
            if (snr == SNR_OFF) {                  /* enter: start calibration */
                snr = SNR_CAL;
                snr_t0 = now;
                noise_min = 1.0f;
                snr_peak = 0;
            } else {
                snr = SNR_OFF;                     /* long-press again = exit early */
            }
            last_ver = ~0u;                        /* force redraw */
        }

        if (snr == SNR_CAL) {
            /* track the MINIMUM smoothed RMS over 2 s — robust against
               accidental string noise during calibration */
            float r = ui_rms;
            if (r < noise_min) noise_min = r;
            if (now - snr_t0 > pdMS_TO_TICKS(2000)) {
                noise_rms = (noise_min > 1e-7f) ? noise_min : 1e-7f;
                snr = SNR_SHOW;
                snr_t0 = now;
            }
        }
        if (snr == SNR_SHOW && now - snr_t0 > pdMS_TO_TICKS(15000))
            snr = SNR_OFF;                         /* auto-revert after 15 s */

        /* --- redraw --- */
        bool need = (ui_ver != last_ver) ||
                    (snr != SNR_OFF) ||            /* SNR modes animate every frame */
                    fabsf(ui_level - last_level) > 0.03f;
        if (need) {
            last_ver = ui_ver;
            last_level = ui_level;

            if (snr == SNR_OFF) {
                if (menu_p.dial(1) && ui_viz) {
                	while (ui_viz) {vTaskDelay(pdMS_TO_TICKS(1000));}
//                 	lcd_draw_map(LCD_W - 35, LCD_H - 35, 32, 32);
//                 	ui_gs_req++;
                } else {
                            draw_screen_body();                    /* header + dials as before,
                                                      WITHOUT the level meter  */

					draw_level_meter();                /* the old bottom strip */
                }
            } else {
                float sdb = 20.0f * log10f((ui_rms + 1e-7f) / noise_rms);
                if (sdb < 0) sdb = 0;
                if (snr == SNR_SHOW && sdb > snr_peak) snr_peak = sdb;
                draw_snr(snr, noise_rms, sdb, snr_peak);
            }
            lcd_flush();
        }
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

// static void display_task(void *arg)
// {
//     lcd_init();
//     uint32_t last_ver = ~0u;
//     float last_level = -1;
// 
//     while (1) {
//         if (ui_ver != last_ver || fabsf(ui_level - last_level) > 0.03f) {
//             last_ver = ui_ver;
//             last_level = ui_level;
//             draw_screen();
//         }
//         vTaskDelay(pdMS_TO_TICKS(33));      /* ~30 fps */
//     }
// }


extern "C" void app_main(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    i2c_setup();
    vTaskDelay(pdMS_TO_TICKS(20));
    i2s_setup();
    vTaskDelay(pdMS_TO_TICKS(200));

    esp_err_t e = wm8960_setup(10.0f, 0.0f, 0.0f, -10.0f, 3);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "codec init failed with %s, restarting...", esp_err_to_name(e));
        esp_restart();
    }

    fx_init();
    xTaskCreatePinnedToCore(dsp_task, "dsp", 8192, NULL, 6, NULL, 1);  /* audio core 1 */
    xTaskCreate(control_task, "ctrl", 4096, NULL, 4, NULL);            /* UI core 0*/
    xTaskCreate(display_task, "lcd", 4096, NULL, 3, NULL);  // lowest priority
    ESP_LOGI(TAG, "running");
    uint32_t free_ram = esp_get_free_heap_size();
	ESP_LOGI(TAG, "heap memory %u bytes", free_ram);
}
