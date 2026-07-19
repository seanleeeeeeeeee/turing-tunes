#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

class Joystick {
public:
	Joystick();
	int x();
	int y();
	bool sw();

};