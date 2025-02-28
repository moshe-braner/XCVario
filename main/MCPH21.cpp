#include "MCPH21.h"
#include <math.h>
#include <logdef.h>

MCPH21::MCPH21()
{
	// address = I2C_ADDRESS_MCPH21;
	psi = 0;
	temperature = 0;
	airspeed = 0;
	P_dat = 0;  // 14 bit pressure data
	T_dat = 0;  // 11 bit temperature data
	error = ESP_OK;
	_status = 0;
	_offset = 0;
	bus = 0;
	changeConfig();
}

MCPH21::~MCPH21()
{
}

// combine measure and collect into one function that returns the status code only, and holds the results in variables
// other subroutines for returning clean values should be get functions

void MCPH21::changeConfig(){
	multiplier = ((100.0f + speedcal.get()) / 100.0f);
	ESP_LOGI(FNAME,"changeConfig, speed multiplier %f, speed cal: %f ", multiplier, speedcal.get() );
}

bool MCPH21::measure()
{
	bool ret = fetch_pressure(P_dat, T_dat);
	// ESP_LOGI(FNAME,"MCPH21::fetch_pressure: %d", P_dat );
	return ret;
}

// #define RANDOM_TEST
#define Press_H data[0]
#define Press_L data[1]
#define Temp_H  data[2]
#define Temp_L  data[3]

bool MCPH21::fetch_pressure(uint32_t &P_dat, uint16_t &T_dat)
{
	// ESP_LOGI(FNAME,"MCPH21::fetch_pressure");
	uint8_t pres[3];
	esp_err_t err = bus->writeByte( address, 0x30, 0x0A );
	if( err != ESP_OK ){
		ESP_LOGI(FNAME,"fetch_pressure register 0x30 write failed");
		return false;
	}
	delay( 5 );
	bool ready=false;
	int end=10;
	while( !ready && end ){
		uint8_t status;
		err = bus->readBytes(address, 0x02, 1, &status );
		if( err != ESP_OK ) {
			ESP_LOGW(FNAME,"status reading error");
			return false;
		}
		// ESP_LOGI(FNAME,"status %x", status);
		if( status & 0x01 )
			ready = true;
		delay(2);
		end--;
	}
	err = bus->readBytes(address, 0x06, 3, pres );
	if( err != ESP_OK ) {
		ESP_LOGW(FNAME,"fetch_pressure() readBytes I2C error");
		return false;
	}
#ifdef RANDOM_TEST
	Press_H = esp_random() % 255;
	Press_L = esp_random() % 255;
	Temp_L = esp_random() % 255;
#endif
	P_dat = (pres[0]*65536)+(pres[1]*256)+pres[2];
	return true;
}

float MCPH21::readPascal( float minimum, bool &ok ){
	ok=true;
	bool status = measure();
	if( status != true ){
		ok=true;
		ESP_LOGE(FNAME,"Retry measure, status :%d  p=%d", _status, P_dat );
		status = measure();
		if( status != true ){
			ESP_LOGE(FNAME,"Warning, status :%d  p=%d, bad even retry", status, P_dat );
			ok=false;
		}
	}
	float _pascal = ((1000.f*6.25f/8388608.f) * (float)(P_dat-_offset))*multiplier;
	// ESP_LOGI(FNAME,"Praw: %d  Off: %d, Pa: %.2f, mp: %.2f", P_dat, _offset, _pascal, multiplier );
	if ( _pascal < minimum ) {
		_pascal = 0.0;
	};
	// ESP_LOGI(FNAME,"pressure: %f offset: %d raw: %d  raw-off:%f m:%f", _pascal, _offset, P_dat,  (_offset - P_dat),  MCPH21multiplier );
	return( _pascal );
}


bool MCPH21::selfTest( int& adval ){
	ESP_LOGI(FNAME,"MCPH21 selftest adval: %d, I2C addr: %x", adval, address );
	bool okay=true;
	uint8_t byte[2] = { 0 };
	esp_err_t err = bus->testConnection(address);
	if( err != ESP_OK ) {
		ESP_LOGI(FNAME,"MCPH21 testConnection: FAIL I2C addr: %x", address );
		return false;
	}
	err = bus->readByte( address, 0x01, byte );
	if( err != ESP_OK ){
		ESP_LOGI(FNAME,"MCPH21 selftest read Chip ID reg 0x01 failed");
		return false;
	}else{
		ESP_LOGI(FNAME,"MCPH21 selftest read Chip ID reg 0x01: %x ", byte[0] );
	}
	err = bus->readByte( address, 0xA8, byte );
	if( err != ESP_OK ){
		ESP_LOGI(FNAME,"MCPH21 selftest read Chip ID reg 0xA8 failed");
		return false;
	}else{
		ESP_LOGI(FNAME,"MCPH21 selftest read Chip ID reg 0xA8: %x", byte[0] );
	}

	float p = readPascal( 0, okay );
	float t = getTemperature();
	if( okay ){
		ESP_LOGI(FNAME,"MCPH21 selftest OKAY, T: %.2f °C, P(off=%d): %.2f Pa", t, _offset, p );
	}else{
		ESP_LOGI(FNAME,"MCPH21 selftest, scan for I2C address %02x FAILED",address );
		return false;
	}
	ESP_LOGI(FNAME,"MCPH21 selftest, for I2C address %02x PASSED",address );
	adval = P_dat;
	return true;
}

float MCPH21::getTemperature(void){     // returns temperature of last measurement
	uint8_t temp[2];
	float T=0.0;
	esp_err_t err = bus->readBytes(address, 0x09, 2, temp );
	if( err != ESP_OK ){
		ESP_LOGI(FNAME,"MCPH21 read pressure 0x09 failed");
		return 0.0;
	}else{
		int t =((temp[0]*256)+temp[1]);
		//T = (float)(t/256.0);
		t += 128;
		t >>= 8;
		T = (float)t;
		ESP_LOGI(FNAME,"MCPH21 T val read ok: T: %.2f", T );
	}
	return T;
}

float MCPH21::getAirSpeed(void){        // calculates and returns the airspeed in m/s IAS
	/* Velocity calculation from a pitot tube explanation */
	/* +/- 1PSI, approximately 100 m/s */
	const float rhom = (2.0f*100.0f)/1.225f; // density of air plus multiplier
	// velocity = sqrt( (2*psi) / rho )   or sqt( psi /
	float velocity = abs( sqrt(psi*rhom) );
	// ESP_LOGI(FNAME,"velocity %f", velocity );
	return velocity;
}


bool MCPH21::offsetPlausible(uint32_t aoffset )
{
	ESP_LOGI(FNAME,"MCPH21 offsetPlausible( %d )", aoffset );
	const int lower_val = 838861-MAX_AUTO_OFFSET_MCPH21;
	const int upper_val = 838861+MAX_AUTO_OFFSET_MCPH21;
	if( (aoffset > lower_val ) && (aoffset < upper_val )  )
		return true;
	else
		return false;
}

bool MCPH21::doOffset( bool force ){
	ESP_LOGI(FNAME,"MCPH21 () force:%d", force );

	_offset = (int) as_offset.get();
	if( _offset < 0 )
		ESP_LOGI(FNAME,"offset not yet done: need to recalibrate. Current offset NVR: %d, autozero: %d", _offset, autozero.get() );
	else
		ESP_LOGI(FNAME,"offset from NVS: %d", _offset );

	uint32_t adcval;
	uint16_t T;
	fetch_pressure( adcval, T );

	ESP_LOGI(FNAME,"offset from ADC %d", adcval );

	bool plausible = offsetPlausible( adcval );
	if( plausible )
		ESP_LOGI(FNAME,"offset from ADC is plausible");
	else
		ESP_LOGI(FNAME,"offset from ADC is NOT plausible");

	int deviation = abs( _offset - (int)adcval );
	if( deviation < MAX_AUTO_OFFSET_MCPH21 )
		ESP_LOGI(FNAME,"Deviation in bounds");
	else
		ESP_LOGI(FNAME,"Deviation out of bounds");

	if( (_offset < 0 ) || ( plausible && (deviation < MAX_AUTO_OFFSET_MCPH21 ) ) || autozero.get() )
	{
		ESP_LOGI(FNAME,"Airspeed OFFSET correction ongoing, calculate new _offset, autozero: %d", autozero.get() );
		if( autozero.get() )
			autozero.set(0);
		uint32_t rawOffset=0;
		for( int i=0; i<100; i++){
			fetch_pressure( adcval, T );
			rawOffset += adcval;
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		_offset = rawOffset / 100;
		if( offsetPlausible( _offset ) )
		{
			ESP_LOGI(FNAME,"Offset procedure finished, offset: %d", _offset);
			if( (int) as_offset.get() != _offset ){
				as_offset.set( (float) _offset );
				ESP_LOGI(FNAME,"Stored new offset in NVS");
			}
			else
				ESP_LOGI(FNAME,"New offset equal to value from NVS");
		}
		else{
			ESP_LOGW(FNAME,"Offset out of tolerance, ignore odd offset value");
		}
	}
	else
	{
		ESP_LOGI(FNAME,"No new Calibration: flying with plausible pressure");
	}
	return true;
}


