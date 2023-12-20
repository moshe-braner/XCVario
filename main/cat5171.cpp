#include "cat5171.h"
#include "I2C.h"
#include <logdef.h>

//Create instance  CAT5171(gpio_num_t sda, gpio_num_t scl);
CAT5171::CAT5171()
{
	errorcount=0;
	_noDevice = false;
	wiper = CAT5171RANGE/2;
	bus = 0;
}

bool CAT5171::begin()
{
#if defined(NOSENSORS)
	return( false );
#else
	errorcount=0;
	int local_wiper;
	if( readWiper( local_wiper ) ) {
		wiper = local_wiper;
		ESP_LOGI(FNAME,"CAT5171 wiper=%d", wiper );
		return(true);
	}
	// else
	ESP_LOGE(FNAME,"CAT5171 Error reading wiper!");
	return( false );
#endif
}

//destroy instance
CAT5171::~CAT5171()
{
}

bool CAT5171::haveDevice() {
#if defined(NOSENSORS)
	    return( false );
#else
	ESP_LOGI(FNAME,"CAT5171 haveDevice");
	esp_err_t err = bus->testConnection(CAT5171_I2C_ADDR);
	if( err == ESP_OK ) {
		ESP_LOGI(FNAME,"CAT5171 haveDevice: OK");
		return true;
	}
	// else
	ESP_LOGI(FNAME,"CAT5171 haveDevice: NONE");
	return false;
#endif
}

bool CAT5171::readWiper( int &val ) {
#if defined(NOSENSORS)
	return false;
#else
	uint16_t i16val;
	esp_err_t err = bus->read8bit(CAT5171_I2C_ADDR, &i16val );
	if( err == ESP_OK ){
		// ESP_LOGI(FNAME,"CAT5171 read wiper val=%d  OK", i16val );
		val = i16val;
		return true;
	}
	// else
	ESP_LOGE(FNAME,"CAT5171 Error reading wiper, error count %d", errorcount);
	errorcount++;
	return false;
#endif
}

bool CAT5171::writeWiper( int val ) {
#if defined(NOSENSORS)
	return false;
#else
    // ESP_LOGI(FNAME,"CAT5171 write wiper %d", val );
	esp_err_t err = bus->write2bytes( CAT5171_I2C_ADDR, 0, (uint8_t)val );  // 0x40 = RS = midscale
	if( err != ESP_OK ){
		// ESP_LOGV(FNAME,"CAT5171 write wiper OK");
		ESP_LOGE(FNAME,"CAT5171 Error writing wiper, error count %d", errorcount);
		errorcount++;
		return false;
	}
	int b;
	bool ret=readWiper( b );

	if( (b != val) || !ret ){
		ESP_LOGE(FNAME,"CAT5171 Error writing wiper, error count %d, write %d != read %d", errorcount, val, b );
	}

	// ESP_LOGI(FNAME,"CAT5171 write wiper OK");
	return true;
#endif
}

bool CAT5171::readVolume( float &val ) {
	int ival;
	if ( readWiper( ival ) ) {
		val = (float)(100 * ival) * getInvRange();
		return true;
	}
	return false;
}

bool CAT5171::writeVolume( float val ) {
	int ival = (int)(val * getRange());
	ival /= 100;
	return writeWiper( ival );
}

bool CAT5171::reset() {
#if defined(NOSENSORS)
	return false;
#else
    ESP_LOGI(FNAME,"CAT5171 reset");
	esp_err_t err = bus->write2bytes( CAT5171_I2C_ADDR, 0x60, 128 );  // 0x40 = RS = midscale
	if( err != ESP_OK ){
		// ESP_LOGV(FNAME,"CAT5171 write wiper OK");
		ESP_LOGE(FNAME,"CAT5171 Error writing wiper, error count %d", errorcount);
		errorcount++;
		return false;
	}
	ESP_LOGI(FNAME,"CAT5171 reset OK");
	return true;
#endif
}

