/*
 * DS18B20.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: iltis
 */

#include "driver/gpio.h"
#include "DS18B20.h"
#include "DallasRmt.h"
#include <esp_log.h>
#include "sdkconfig.h"
#include <stdio.h>
#include <logdef.h>

// #include "sensor.h"   // for NOSENSORS & SUNTON28

// OnewireRmt owInst( GPIO_NUM_23, 0, 1);
// DallasRmt dallasInst;

DS18B20::DS18B20(gpio_num_t pin, uint8_t res, int max_dev ) {
	_pin = pin;
	_res = res;
	_max_dev = max_dev;
	numDevices = 0;
}

bool DS18B20::begin(){
#if defined(NOSENSORS)
	return false;
#else
	ESP_LOGI(FNAME,"DS18B20::begin");
	gpio_set_pull_mode(_pin, GPIO_PULLUP_ONLY);

	ow = new OnewireRmt(_pin, RMT_CHANNEL_0, RMT_CHANNEL_1);

	dallas = new DallasRmt( ow );
	dallas->begin();
	numDevices = dallas->getDeviceCount();
	ESP_LOGI(FNAME,"Found %d Dallas temperature devices", numDevices);
	if( numDevices )
		return true;
	else
		return false;
#endif
}

float DS18B20::getTemp(){
#if defined(NOSENSORS)
	return 0.0;
#else
	float temp = DEVICE_DISCONNECTED_C;
	if( numDevices ) {
		dallas->requestTemperatures();
		temp = dallas->getTempCByIndex(0);
	}
	return temp;
#endif
}

DS18B20::~DS18B20() {

}
