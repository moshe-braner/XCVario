/*
  BME280_ESP32_SPI.h  Bosch BMP/BME280 for ESP32
  Version 1.1

Reference library:
https://github.com/BoschSensortec/BMP280_driver

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.   See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.   If not, see <http://www.gnu.org/licenses/>.

Written: Dec 7 2017.
Last Updated: Dec 07 2017.
This header must be included in any derived code or copies of the code.
Based on the data sheet provided by Bosch for the BMP280 environmental sensor,
calibration code based on algorithms provided by referenced Bosch library,
altitude calculation by open source community on github.

 */

#ifndef BME280_ESP32_SPI_h_
#define BME280_ESP32_SPI_h_
#include <esp_system.h>
#include "driver/gpio.h"
#include <SPI.h>
#include <esp32-hal-spi.h>
#include <math.h>
#include <hal/gpio_types.h>
#include "PressureSensor.h"


class BME280_ESP32_SPI: public PressureSensor
{
public:
	BME280_ESP32_SPI();
	bool  setBus( I2C_t *_theBus ) { return true; };  // for future
	bool  setSPIBus(gpio_num_t sclk, gpio_num_t mosi, gpio_num_t miso, gpio_num_t cs, uint32_t freq );
	bool begin();
    bool selfTest( float& p, float& t );

	float readTemperature( bool& success );
	float readPressure(bool &ok);
	float readPressureAVG( float alpha=0.1 );
	float readHumidity();
	float readAltitude(float SeaLevel_Pres, bool &ok);
//	inline float calcAltitude(float SeaLevel_Pres, float pressure) { return ( 44330.0 * (1.0 - pow(pressure / SeaLevel_Pres, (1.0/5.255))) ); }
//	float calcAltitude(float SeaLevel_Pres, float pressure);
//	float calcAltitudeSTD( float p );
	uint8_t readID();

private:
	void WriteRegister(uint8_t reg_address, uint8_t data);
	void readCalibration(void);
	int32_t compensate_T(int32_t adc_T);
	uint32_t compensate_P(int32_t adc_P);
	uint32_t compensate_H(int32_t adc_H);
	uint16_t read16bit(uint8_t reg);
	uint8_t read8bit(uint8_t reg);
	float _avg_alt;
	float _avg_alt_std;


private:
	gpio_num_t _sclk, _mosi, _miso;
	uint8_t _cs;
	uint32_t _freq;
	int32_t  _t_fine;

	uint16_t _dig_T1;
	int16_t  _dig_T2;
	int16_t  _dig_T3;

	uint16_t _dig_P1;
	int16_t  _dig_P2;
	int16_t  _dig_P3;
	int16_t  _dig_P4;
	int16_t  _dig_P5;
	int16_t  _dig_P6;
	int16_t  _dig_P7;
	int16_t  _dig_P8;
	int16_t  _dig_P9;

	uint8_t _dig_H1;
	int16_t _dig_H2;
	uint8_t _dig_H3;
	int16_t _dig_H4;
	int16_t _dig_H5;
	int8_t  _dig_H6;
	float exponential_average;
	bool init_err;
	SPISettings spis;
};

#endif
