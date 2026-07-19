#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "stick.h"
#define BUTTON_PIN GPIO_NUM_7
#define X_PIN   ADC_CHANNEL_4
#define Y_PIN   ADC_CHANNEL_5
static adc_oneshot_unit_handle_t adc_handle;
static adc_oneshot_unit_init_cfg_t init_config;
static adc_oneshot_chan_cfg_t channel_config;

Joystick::Joystick() {
	init_config = {
		.unit_id = ADC_UNIT_1,
		.clk_src = ADC_RTC_CLK_SRC_DEFAULT,
	};
	adc_oneshot_new_unit(&init_config, &adc_handle);
	channel_config = {
		.atten = ADC_ATTEN_DB_12, // For ~0.1V to 2.5V (or up to 3.3V with less accuracy)
		.bitwidth = ADC_BITWIDTH_DEFAULT, // Typically 12-bit
	};
	adc_oneshot_config_channel(adc_handle, X_PIN, &channel_config);
	adc_oneshot_config_channel(adc_handle, Y_PIN, &channel_config);
	
	gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN), // Bit mask of the pins to set
        .mode = GPIO_MODE_INPUT,                 // Set as input mode
        .pull_up_en = GPIO_PULLUP_ENABLE,        // Enable internal pull-up resistor
        .pull_down_en = GPIO_PULLDOWN_DISABLE,   // Disable internal pull-down resistor
        .intr_type = GPIO_INTR_DISABLE           // Disable interrupts for standard polling
    };
    gpio_config(&io_conf);
	
}

int Joystick::x() {
	int raw_value;
	ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, X_PIN, &raw_value));
	return raw_value;
}
int Joystick::y() {
	int raw_value;
	ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, Y_PIN, &raw_value));
	return raw_value;
}
bool Joystick::sw() {
	return !(bool)gpio_get_level(BUTTON_PIN);
}
