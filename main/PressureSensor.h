#ifndef PressureSensor_H
#define PressureSensor_H

#include "I2Cbus.hpp"
#include <cmath>
#include <hal/gpio_types.h>

class PressureSensor {
public:
	virtual bool  setSPIBus(gpio_num_t sclk, gpio_num_t mosi, gpio_num_t miso, gpio_num_t cs, uint32_t freq ) = 0;
	virtual bool  setBus( I2C_t *theBus ) = 0;
	virtual bool  begin() = 0;
	virtual bool  selfTest( float& p, float &t ) = 0;
	virtual float readPressure(bool &success) = 0;
	virtual float readTemperature( bool& success ) = 0;
	virtual float readAltitude( float qnh, bool &success ) = 0;
//	virtual float calcAltitude( float qnh, float p ) = 0;
    float calcAltitude(float SeaLevel_Pres, float pressure) {
       return ( 44330.0f * (1.0f - pow(pressure / SeaLevel_Pres, (1.0f/5.255f))) );
    }
/*
        // polynomial approximation of altitude as a function of pressure ratio
        // - courtesy of Rick Sheppe
        // - maybe faster to compute than with pow() - or maybe not
        float ratio = pressure / SeaLevel_Pres;
        float altitude = ratio * -1.752317e+04;
        altitude = ratio * (altitude + 6.801427e+04);
        altitude = ratio * (altitude - 1.087470e+05);
        altitude = ratio * (altitude + 9.498147e+04);
        altitude = ratio * (altitude - 5.669573e+04);
        altitude += 1.997137e+04;
        return altitude;
    }
*/
//	virtual float calcAltitudeSTD( float p ) = 0;
	inline float calcAltitudeSTD( float p ) { return calcAltitude( 1013.25, p ); };
    // this is apparently only called from client loop, and can do less often?
    float calcPressure(float SeaLevel_Pres, float altitude) { return SeaLevel_Pres * pow(1.0 - (altitude / 44330.171), 5.255); }
};

#endif
