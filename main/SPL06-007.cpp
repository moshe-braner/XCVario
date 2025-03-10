#include "SPL06-007.h"
#include "logdef.h"

SPL06_007::SPL06_007( char _addr ){
	bus = 0;
	address = _addr;
	c00 = c10 = c01 = c11 = c20 = c21 = c30 = c0 = c1 = 0;
	_scale_factor_p = 0;
	_scale_factor_t = 0;
	errors = 0;
	_praw = 0;
	_traw = 0;
	last_praw = 0;
	last_traw = 0;
	tick = 0;
}

// Addr. 0x06 PM_RATE Bits 6-4:    110  - 64 measurements pr. sec.
//            PM_PRC  Bis  3-0:    0011 - 8 times oversampling

// Addr. 0x08 MEAS_CTRL Bits 2-0:  111  - Continuous pressure and temperature measurement

bool SPL06_007::begin() {
#if defined(NOSENSORS)
    return false;
#else
	// ---- Oversampling of >8x for temperature or pressuse requires FIFO operational mode which is not implemented ---
	// ---- Use rates of 8x or less until feature is implemented ---
	errors = 0;

	i2c_write_uint8( 0X06, 0x63);	// Pressure    7=128 samples per second and 3 = 8x oversampling 128/8 = 16 mS
	i2c_write_uint8( 0X07, 0X80);	// Temperature 0=1   sample  per second and 0 = no oversampling
	i2c_write_uint8( 0X08, 0B0111);	// continuous temp and pressure measurement
	i2c_write_uint8( 0X09, 0x00);	// FIFO Pressure measurement

	errors = 0x81;
	for(int i=0; i<10; i++ ){
		int8_t status = i2c_read_uint8( 0x08 );
		if( (status & 0x80) != 0x80 ){
			ESP_LOGW(FNAME,"Coefficients not ready, status byte %02x", status );
			delay(10);
		}else{
			errors=0;
			break;
		}
	}
	if( errors )
		return false;

	c00 = get_c00();
	if( errors )
		return false;
	c10 = get_c10();

	c01 = get_16bit( 0X18 );  // ok
	c11 = get_16bit( 0X1A );  //
	c20 = get_16bit( 0X1C );
	c21 = get_16bit( 0X1E );
	c30 = get_16bit( 0X20 );

	get_c0();
	get_c1();

	_scale_factor_p = get_scale_factor( 0x06 );
	_scale_factor_t = get_scale_factor( 0x07 );
	if( errors )
		return false;
	else
		return true;
#endif
}

float SPL06_007::readTemperature( bool& success ){

	float t = float(c0) * 0.5f + float(c1) * float(_traw)/_scale_factor_t;
	success = true;
	return t;
}

bool SPL06_007::selfTest( float& t, float& p ){

#if defined(NOSENSORS)
	return false;
#else

	uint8_t rdata = 0xFF;
	delay(100); // give first measurement time to settle
	esp_err_t err = bus->readByte(address, 0x0D, &rdata );  // ID
	if( err != ESP_OK ){
		ESP_LOGE(FNAME,"Error I2C read, status :%d", err );
		return false;
	}

	ESP_LOGI(FNAME,"SPL06_007 selftest, scan for I2C address %02x PASSED, Product ID: %d, Revision ID:%d", address, rdata>>4 , rdata&0x0F );
	bool ok;
	p=0;
	for(int i=0; i<10;i++){
		p += (float)get_pressure(ok);
		delay( 50 );
		if( !ok )
			break;
	}
	p=p/10.0;
	if( !ok ) {
		ESP_LOGI(FNAME,"SPL06_007 selftest failed, getPressure returnd false, retry");
		delay(2);
		p = (float)get_pressure(ok);
		if( !ok ){
			ESP_LOGW(FNAME,"SPL06_007 selftest failed, getPressure returnd false, abort");
			return false;
		}
	}
	t=0.0;
	for(int i=0; i<10;i++){
		t += (float)get_temp_c(ok);
		delay( 50 );
		if( !ok )
			break;
	}
	t=t/10.0;
	if( ok ){
		ESP_LOGI(FNAME,"SPL06_007 selftest, p=%f t=%f", p, t );
	}

	if( p < 1200 && p > 0 && ok ) {
		ESP_LOGI(FNAME,"SPL06_007 selftest addr: %d PASSED, p=%f t=%f", address, p, t );
		return true;
	}
	else{
		ESP_LOGI(FNAME,"SPL06_007 selftest addr: %d FAILED, p=%f t=%f", address, p, t );
		return false;
	}
#endif
}

#if 0 // moved to PressureSensor.h
float SPL06_007::get_altitude(float pressure, float seaLevelhPa) {
    //float altitude = 44330.0 * (1.0 - pow(pressure / seaLevelhPa, 0.19029495718));
    // polynomial approximation of altitude as a function of pressure ratio
    // - courtesy of Rick Sheppe
    // - faster to compute than with pow()
    float ratio = pressure / seaLevelhPa;
    float altitude = ratio * -1.752317e+04;
    altitude = ratio * (altitude + 6.801427e+04);
    altitude = ratio * (altitude - 1.087470e+05);
    altitude = ratio * (altitude + 9.498147e+04);
    altitude = ratio * (altitude - 5.669573e+04);
    altitude += 1.997137e+04;
    // ESP_LOGI(FNAME,"SPL06_007::get_altitude: %f, p: %f, qnh: %f",altitude, pressure, seaLevelhPa );
    return altitude;
}
#endif

float SPL06_007::get_traw_sc( bool &ok )
{
	get_traw( ok );
	return (float(_traw)/_scale_factor_t);
}

float SPL06_007::get_temp_c( bool &ok )
{
	float traw_sc = get_traw_sc(ok);
	float t = float(c0) * 0.5f + float(c1) * traw_sc;
	// ESP_LOGI(FNAME,"T=%f °C", t);
	return (t);
}

float SPL06_007::get_temp_f()
{
	bool ok;
	float traw_sc = get_traw_sc(ok);
	return (((float(c0) * 0.5f) + (float(c1) * traw_sc)) * 9/5) + 32;
}

int32_t SPL06_007::get_traw( bool &ok )
{
#if defined(NOSENSORS)
	ok = false;
	return 0;
#else
	uint8_t data[3];
	tick++;
	if( !(tick%10) || (last_traw == 0) ){       // Temperature won't quickly change so its okay to read value every second
		ok = i2c_read_bytes( 0X03, 3, data );   // use correct method to read 3 bytes of volatile data
		if( ok ){
			_traw = (data[0] << 16) | data[1] << 8 | data[2];
			if(_traw & (1 << 23))
				_traw = _traw | 0XFF000000; // Set left bits to one for 2's complement conversion of negative number
			last_traw = _traw;
			return _traw;
		}
		ESP_LOGW(FNAME,"T raw read error, data: %02x%02x%02x", data[0],data[1],data[2] );
		return last_traw;
	}
	ok = true;
	return last_traw;
#endif
}

float SPL06_007::get_praw_sc( bool &ok )
{
	get_praw( ok );
	return (float(_praw)/_scale_factor_p);
}


float SPL06_007::get_pcomp(bool &ok)
{
#if defined(NOSENSORS)
	ok = false;
	return 1000.0;
#else
	bool ok_t, ok_p;
	ok = false;
	int i=0;
	uint8_t status = 0;
	for( i=0; i<10; i++ ){
		status = i2c_read_uint8( 0x08 );
		if( (status & 0x10) != 0x10 ){  // sensor, temp, and pressure ready  D7
			delay( 10 );
		}else{
			ok=true;
			break;
		}
	}
	if( !ok ){
		ESP_LOGE(FNAME,"Sensor temp and pressure ready bits not set %02x", status );
		return last_p;
	}
	if( i>0 ){
		ESP_LOGW(FNAME,"Sensor temp and pressure ready bits took %d attempts", i );
	}
	float traw_sc = get_traw_sc( ok_t );
	float praw_sc = get_praw_sc( ok_p );
	if( !ok_t || !ok_p ){
		ESP_LOGW(FNAME,"T %d or P %d reading returned false", ok_t, ok_p );
	}
	float p = float(c00) + praw_sc * (float(c10) + praw_sc * (float(c20) + praw_sc * float(c30))) + traw_sc * float(c01) + traw_sc * praw_sc * ( float(c11) + praw_sc * float(c21));
	// if( address == 0x76 ) {
	// float t = (float(c0) * 0.5f) + (float(c1) * traw_sc);
	// ESP_LOGI(FNAME,"P:%06x,%d  T:%06x PC:%f T:%f I2C E:%d",_praw, _praw, _traw, p/100, t , errors );
	// }
	ok = true;
	last_p = p;
	return p;
#endif
}


float SPL06_007::readAltitude( float qnh, bool &ok ) {

	float p =  get_pressure( ok );
	return calcAltitude( qnh, p );
};

float SPL06_007::get_pressure( bool &ok )
{
	float pcomp = get_pcomp( ok );
	return pcomp * 0.01; // convert to mb
}

float SPL06_007::get_scale_factor( int reg )
{
#if defined(NOSENSORS)
	return 0;
#else
	float k = 0;
	uint8_t tmp_Byte;
	tmp_Byte = i2c_read_uint8( reg );   // MSB
	tmp_Byte = tmp_Byte & 0B00000111;   // Focus on 2-0 oversampling rate

	ESP_LOGI(FNAME,"Oversampling reg %d = %02x", reg, tmp_Byte );

	switch (tmp_Byte) // oversampling rate
	{
	case 0B000:
		k = 524288.0;
		break;

	case 0B001:
		k = 1572864.0;
		break;

	case 0B010:
		k = 3670016.0;
		break;

	case 0B011:
		k = 7864320.0;
		break;

	case 0B100:
		k = 253952.0;
		break;

	case 0B101:
		k = 516096.0;
		break;

	case 0B110:
		k = 1040384.0;
		break;

	case 0B111:
		k = 2088960.0;
		break;
	}
	return k;
#endif
}

// #define RANDOM_TEST


int32_t SPL06_007::get_praw( bool &ok )
{
#if defined(NOSENSORS)
	ok = false;
	return 0;
#else
	uint8_t data[3];
	ok = i2c_read_bytes( 0X00, 3, data );
	if( ok )
	{
#ifdef RANDOM_TEST
		data[2] = esp_random() % 255;
		data[1] = esp_random() % 255;
#endif
		_praw = data[0] << 16 | data[1] << 8 | data[2];
		if(_praw & (1 << 23)){
			_praw = _praw | 0XFF000000; // Set left bits to one for 2's complement conversion of negative number
		}
	}
	else{
		ESP_LOGW(FNAME,"P raw read error, data: %02x%02x%02x", data[0],data[1],data[2] );
		return last_praw;
	}
#ifdef I2C_ISSUE_TEST
	if( address == 0x76 ){
		if( abs( _praw - last_praw )  > 400 ){
			ESP_LOGW(FNAME,"P raw delta OOB: %d %06x last: %d %06x, delta %d", _praw, _praw, last_praw, last_praw, abs( _praw - last_praw ) );
		}
		last_praw = _praw;
	}
#endif
	last_praw = _praw;
	return _praw;
#endif  // NOSENSORS
}

int16_t SPL06_007::get_c0()
{
#if defined(NOSENSORS)
	return 0;
#else
	uint8_t bytes[2];
	i2c_read_bytes( 0x10, 2, bytes );
	c0 = (bytes[0] << 4) | (bytes[1] >> 4);
	if(c0 & (1 << 11)) // Check for 2's complement negative number
		c0 = c0 | 0XF000; // Set left bits to one for 2's complement conversion of negitive number
	return c0;
#endif
}

int16_t SPL06_007::get_c1()
{
#if defined(NOSENSORS)
	return 0;
#else
	uint8_t bytes[2];
	i2c_read_bytes( 0x11, 2, bytes );
	bytes[0] = bytes[0] & 0XF;
	c1 = (bytes[0] << 8) | bytes[1];
	if(c1 & (1 << 11)) // Check for 2's complement negative number
		c1 = c1 | 0XF000; // Set left bits to one for 2's complement conversion of negitive number
	return c1;
#endif
}

int32_t SPL06_007::get_c00()
{
#if defined(NOSENSORS)
	return 0;
#else
	int32_t ret;
	uint8_t bytes[4];
	i2c_read_bytes( 0x13, 3, bytes );
	bytes[2] = bytes[2] >> 4;
	ret = (uint32_t)bytes[0] << 12 | (uint32_t)bytes[1] << 4 | (uint32_t)bytes[2];
	if(ret & (1 << 19))
		ret = ret | 0XFFF00000; // Set left bits to one for 2's complement conversion of negitive number
	return ret;
#endif
}

int32_t SPL06_007::get_c10()
{
#if defined(NOSENSORS)
	return 0;
#else
	int32_t ret;
	uint8_t bytes[4];
	i2c_read_bytes( 0x15, 3, bytes );
	bytes[0] = bytes[0] & 0b00001111;
	ret = (uint32_t)bytes[0] << 16 | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2];
	if(ret & (1 << 19))
		ret = ret | 0XFFF00000; // Set left bits to one for 2's complement conversion of negitive number
	return ret;
#endif
}

int16_t SPL06_007::get_16bit( uint8_t addr )
{
#if defined(NOSENSORS)
	return 0;
#else
	uint8_t tmp_MSB,tmp_LSB;
	tmp_MSB = i2c_read_uint8( addr );
	tmp_LSB = i2c_read_uint8( addr+1 );
	return (int16_t)(tmp_LSB | (tmp_MSB<<8));
#endif
}

void SPL06_007::i2c_write_uint8( uint8_t eeaddress, uint8_t data )
{
#if !defined(NOSENSORS)
	esp_err_t err = bus->writeByte(address, eeaddress, data );
	if( err != ESP_OK ){
		ESP_LOGE(FNAME,"Error I2C write, status :%d", err );
		errors++;
	}
#endif
}

bool SPL06_007::i2c_read_bytes( uint8_t eeaddress, int num, uint8_t *data )
{
#if defined(NOSENSORS)
    return false;
#else
	esp_err_t err = bus->readBytes(address, eeaddress, num, data );
	if( err != ESP_OK ){
		ESP_LOGE(FNAME,"Error I2C bytes read, status :%d", err );
		errors++;
		return false;
	}
	return true;
#endif
}

uint8_t SPL06_007::i2c_read_uint8( uint8_t eeaddress )
{
	uint8_t rdata = 0xFF;
#if !defined(NOSENSORS)
	esp_err_t err = bus->readByte(address, eeaddress, &rdata );
	if( err != ESP_OK ){
		ESP_LOGE(FNAME,"Error I2C read, status :%d", err );
		errors++;
	}
#endif
	return rdata;
}
