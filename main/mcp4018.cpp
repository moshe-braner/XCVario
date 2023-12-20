#include "mcp4018.h"
#include "I2C.h"
#include <logdef.h>

//Create instance  MCP4018(gpio_num_t sda, gpio_num_t scl);
MCP4018::MCP4018()
{
	errorcount=0;
	_noDevice = false;
	wiper = MCP4018RANGE/2;
	bus = 0;
}

bool MCP4018::begin()
{
#if defined(NOSENSORS)
	    return( false );
#else
	errorcount=0;
	if( readWiper( wiper ) ) {
		ESP_LOGI(FNAME,"MCP4018 wiper=%d", wiper );
		return(true);
	}
	else {
		ESP_LOGE(FNAME,"MCP4018 Error reading wiper!");
	    return( false );
	}
#endif
}

//destroy instance
MCP4018::~MCP4018()
{
}

bool MCP4018::haveDevice() {
#if defined(NOSENSORS)
	return false;
#else
	  ESP_LOGI(FNAME,"MCP4018 haveDevice");
	  esp_err_t err = bus->testConnection(MPC4018_I2C_ADDR);
	  if( err == ESP_OK ) {
		 ESP_LOGI(FNAME,"MCP4018 haveDevice: OK");
	     return true;
	  }
	  else{
		 ESP_LOGI(FNAME,"MCP4018 haveDevice: NONE");
		 return false;
	  }
#endif
}

bool MCP4018::readWiper( int &val ) {
#if defined(NOSENSORS)
	return false;
#else
	uint16_t i16val;
	esp_err_t err = bus->read8bit(MPC4018_I2C_ADDR, &i16val );
	if( err == ESP_OK ){
		//ESP_LOGI(FNAME,"MCP4018 read wiper val=%d  OK", i16val );
		val = i16val;
		return true;
	}
	else
	{
		ESP_LOGE(FNAME,"MCP4018 Error reading wiper, error count %d", errorcount);
		errorcount++;
	    return false;
	}
#endif
}

bool MCP4018::writeWiper( int val ) {
#if defined(NOSENSORS)
	return false;
#else
    // ESP_LOGI(FNAME,"MCP4018 write wiper %d", val );
	esp_err_t err = bus->write8bit(MPC4018_I2C_ADDR, (uint16_t)val );
	if( err == ESP_OK ){
		// ESP_LOGV(FNAME,"MCP4018 write wiper OK");
		return true;
	}
	else
	{
		ESP_LOGE(FNAME,"MCP4018 Error writing wiper, error count %d", errorcount);
		errorcount++;
	    return false;
	}
#endif
}

bool MCP4018::readVolume( float &val ) {
	int ival;
	if ( readWiper( ival ) ) {
		val = (float)(100 * ival) * getInvRange();
		return true;
	}
	else
	{
	    return false;
	}
}

bool MCP4018::writeVolume( float val ) {
	int ival = (int)(val * getRange());
	ival /= 100;
	return writeWiper( ival );
}
