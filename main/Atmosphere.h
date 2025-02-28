/*
 * Methods for Athmosphere Model used in Aviation
 *
 *
 */
#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H


// With density of water from: http://www.csgnetwork.com/waterinformation.html
// @ 22.8 degree: 0.997585
// earth gravity: 9.0807 m/s^2
// and standard ICAO air density with 1.225 kg/m3 there is:
// V(km/h) = sqrt(2*( <mmH2O> * 0.997585 * 9.807  )/1.225) * 3.6

//   Speed
// mmH2O m/s	km/h
// 100	 40,0	143,9
// 105          147,4
// 110	 41,9	150,9
// 116	 43,0	155,0
// 120	 43,8	157,6
// 130	 45,6	164,0
// 140	 47,3	170,2

class Atmosphere {
	Atmosphere() {};
	~Atmosphere() {};
public:
	static inline float calcPressure(float alti) {
			return ( 1013.25f * pow( (1.0f - (6.5f * alti / 288150.0f)), 5.255f ));
	};
	static inline float TAS( float ias, float baro, float temp ) {
		//return( ias * sqrt( 1.225 / ( baro*100.0 / (287.058 * (273.15+temp)))));
		return( ias * sqrt((0.01f * 1.225f * 287.058f) * (273.15f+temp) / baro ));
	};
	// TAS=IAS/sqrt( 288.15/(T+273.15) * (P/1013.25) )
	static inline float TAS2( float ias, float altitude, float temp ) {
			//return( ias / sqrt( 288.15/(temp+273.15) * ( calcPressure( altitude )/1013.25 )) );
			return( ias * sqrt( (1013.25f/288.15f) * (temp+273.15f) / calcPressure( altitude )));
			// - slightly different constant from above
	};
	static inline float CAS( float dp ) {
			return( 1225.0f * sqrt( 5.0f * ( pow( (dp/101325.0f)+1.0f, (2.0f/7.0f) ) - 1.0f)));
	};
	static inline float IAS( float tas, float alti, float temp ) {
			//return( tas / sqrt( 1.225 / ( calcPressure(alti)*100.0 / (287.058 * (273.15+temp)))));
			return( tas * sqrt( (100.0f/1.225f/287.058f) * calcPressure(alti) / (273.15f+temp)));
			// - slightly different constant from above
	};
	static inline float pascal2kmh( float pascal ){
		//return sqrt( 2*pascal / 1.225 )*3.6;
		return sqrt( (3.6f*3.6f*2.0f/1.225f)*pascal );
	};
	static inline float kmh2pascal( float kmh ){
		//return ((kmh/3.6)*(kmh/3.6)) * 1.225/2.;
		return (kmh*kmh * (1.225f/2.0f/3.6f/3.6f));
	};

// This are not actually used?  Instead there is a version in PressureSensor
// calcPressure() is used in client loop and also in TAS2() and IAS() above

//	static inline double calcAltitude(double SeaLevel_Pres, double pressure) {
//			return ( 44330.0 * (1.0 - pow(pressure / SeaLevel_Pres, (1.0/5.255))) );
//	};

};

#endif
