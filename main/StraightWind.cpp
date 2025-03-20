/*
 * Wind.cpp
 *
 *  Module to calculate straight wind
 *
 *  Created on: Mar 21, 2021
 *
 *  Author: Eckhard Völlm, Axel Pauli
 *
 *
       Calculate wind continuously by using wind triangle, see more here:
       http://klspublishing.de/downloads/KLSP%20061%20Allgemeine%20Navigation%20DREHMEIER.pdf

       The Wind Correction Angle is the angle between the Heading and the
       Desired Course:

       WCA = Heading - DesiredCourse
 *
 *
 *
 *  Last update: 2021-04-21
 */
#include <algorithm>
#include <cmath>
#include "esp_system.h"
#include "Compass.h"
#include "Flarm.h"
#include "SetupNG.h"
#include "StraightWind.h"
#include "Units.h"
#include "sensor.h"
#include "math.h"
#include "CircleWind.h"
#include "Router.h"
#include "sensor.h"
#include "KalmanMPU6050.h"
#include "vector.h"  // D2R, R2D

StraightWind::StraightWind() :
averageTas(0),
averageTH( 0.0 ),
averageTC( 0.0 ),
averageGS(0.0),
swindDir( -1.0 ),
swindSpeed( -1.0 ),
lowAirspeed( false ),
circlingWindDir( -1.0 ),
circlingWindDirReverse( -1.0 ),
circlingWindSpeed( -1.0 ),
circlingWindAge( 10000 ),
airspeedCorrection( 1.0 ),
_tick(0),
_n_avg(0),
gpsStatus(false),
deviation_cur(0),
magneticHeading(0),
status( "Initial" ),
jitter(0),
result(0,0),
newWindSpeed(0),
newWindDir(0),
zwindDir( 0 ),
zwindSpeed( 0 ),
zcount(-999),
zminDir(-1.0),
zmaxDir(-1.0),
zWgt(0),
zWgtChg(false),
slipAverage(0),
lastHeading(0),
lastGroundCourse(0)
{
}

void StraightWind::begin(){
	if( compass_dev_auto.get() )
		airspeedCorrection = wind_as_calibration.get();
}

int StraightWind::_age = 10000;

void StraightWind::tick(){
	_age++;
	circlingWindAge++;
	_tick++;
}

bool StraightWind::getWind( int* direction, float* speed, int *age )
{
	*direction = int( swind_dir.get() + 0.5 );
	*speed = float( swind_speed.get() );
	*age = _age;
	if( _age < 7200 )
		return true;
	else
		return false;
}

/**
 * Measurement cycle for wind calculation in straight flight. Should be
 * triggered periodically, maybe once per second.
 *
 * Returns true, if a new wind was calculated.
 */
bool StraightWind::calculateWind()
{
	// ESP_LOGI(FNAME,"Straight wind, calculateWind()");

	if( SetupCommon::isClient() ){
		ESP_LOGI(FNAME,"No windcalc on client");
		return false;
	}

	if( gflags.inSetup && ! _external_data ){
		ESP_LOGI(FNAME,"No windcalc if setup active");
		return false;
	}

	if( ! (wind_enable.get() & WA_STRAIGHT) ){
		status = "Disabled";
		return false;
	}

	if( Flarm::gpsStatus() == false ) {
		// GPS status not valid
		status = "Bad GPS";
		gpsStatus = false;
	}else{
		gpsStatus = true;
	}

	// ESP_LOGI(FNAME,"calculateWind flightMode: %d", CircleStraightWind::getFlightMode() );

	bool compass_ok = true;
	// Check if compass-based straight wind requirements are fulfilled
	if( (!compass) || !compass_enable.get() ) {
		status = "Comps Dis";
		compass_ok = false;
	}
	else if( !compass_calibrated.get() ) {
		status = "Comps NoCal";
		compass_ok = false;
	}

	// Get current ground speed in km/h
	float cgs = Units::knots2kmh( Flarm::getGndSpeedKnots() );

	// Get current true course from GPS
	float ctc = Flarm::getGndCourse();

	// Get current TAS in km/h
	float ctas = float( getTAS() );

	// Check, if we have a AS value > minimum, default is 25 km/h.
	// If GS is nearly zero, the measurement makes also sense (wave), hence if we are not flying it doesn't

	if( ctas < Units::Airspeed2Kmh( wind_as_min.get() ) )
	{
		// We start a new measurement cycle.
		if( !lowAirspeed ) {
			ESP_LOGI(FNAME,"Low Airspeed, stop wind calculation, AS %3.1f  < %3.1f Kmh", ctas,  Units::Airspeed2Kmh( wind_as_min.get() ) );
			lowAirspeed = true;
		}
		status = "Low AS";
	}else{
		if( lowAirspeed ) {
			ESP_LOGI(FNAME,"Airspeed OK, start wind calculation, AS %3.1f  < %3.1f Kmh", ctas,  Units::Airspeed2Kmh( wind_as_min.get() ) );
			lowAirspeed = false;
			//status = "AS OK";
		}
	}

	if( lowAirspeed ){
		slipAverage = 0;   // ignore lowered wing before takeoff
		ESP_LOGI(FNAME,"Low Airspeed, stop ");
		return false;
	}

	bool THok = false;
	float deviation = 0;
	if (compass_ok) {
		// Get current true heading from compass.
		averageTH = compass->filteredTrueHeading( &THok, false );
		// no deviation considered here (we add ourselfs as for reverse calculation we need also the pure heading)
		if( THok == false ) {
			// No valid heading available
			status = "No Heading";
			//ESP_LOGI(FNAME,"Restart Cycle: No magnetic heading");
			compass_ok = false;
		}
		// WCA in radians
		magneticHeading = averageTH;
		deviation = compass->getDeviation( averageTH );
	}

	if( wind_logging.get() != WLOG_DISABLE ){
		char log[SSTRLEN];
		sprintf( log, "$WIND;");
		int pos = strlen(log);
		if( wind_logging.get() & WLOG_WIND ){
			sprintf( log+pos, "%d;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%d;%d;%.1f;%1.1f",
			 _tick, averageTC, cgs, averageTH, ctas, newWindDir, newWindSpeed,
			  swindDir, swindSpeed, circlingWindDir, circlingWindSpeed,
			   (airspeedCorrection-1)*100, CircleWind::getFlightMode(), gpsStatus, deviation, slipAngle );
		}
		pos=strlen(log);
		if( (wind_logging.get() & WLOG_GYRO_MAG) && compass_ok ){
			sprintf( log+pos, ";%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f",
					compass->rawX()/16384.0,compass->rawY()/16384.0,compass->rawZ()/16384.0,
					IMU::getGliderAccelX(), IMU::getGliderAccelY(), IMU::getGliderAccelZ(),
					IMU::getGliderGyroX(), IMU::getGliderGyroY(), IMU::getGliderGyroZ()  );
		}
		pos = strlen(log);
		sprintf( log+pos, "\n");
		Router::sendXCV( log );
		// ESP_LOGI( FNAME,"%s", log );
	}

	// averageTC = Vector::normalize( averageTC + (ctc - averageTC) * 1/wind_gps_lowpass.get());
	averageTC = ctc;
	averageGS += (cgs - averageGS) * 1/wind_gps_lowpass.get();

	averageTas = ctas;

	if( !gpsStatus ){
		ESP_LOGI(FNAME,"GPS bad, stop ");
		return false;
	}

	if( CircleWind::getFlightMode() != straight ){
		//ESP_LOGI(FNAME,"In Circling, stop ");
		status = "Circling";
		return false;
	}

	if ( compass_ok ) {
		// Slip prevents proper heading alignment
		slipAverage += (slipAngle - slipAverage) * 0.02;   // was 0.0005;
		if( compass_ok && abs(slipAngle - slipAverage) > swind_sideslip_lim.get() ){
			status = "Side Slip";
			ESP_LOGI( FNAME, "Slip overrun %.2f, average %.2f", slipAngle, slipAverage );
			return false;
		}
		float headingDelta = Vector::angleDiffDeg( averageTH , lastHeading );
		lastHeading = averageTH;
		if( abs(headingDelta) > wind_straight_course_tolerance.get() ){
			status = "hdg delta";
			ESP_LOGI(FNAME,"Not really straight flight, heading delta: %f", headingDelta );
			return false;
		}
	}

	float groundCourseDelta = Vector::angleDiffDeg( averageTC , lastGroundCourse );
	lastGroundCourse = averageTC;
	if( abs(groundCourseDelta) > 7.5 ){
			status = "crs delta";
			ESP_LOGI(FNAME,"Not really straight flight, GND course delta: %f", groundCourseDelta );
			return false;
	}

	if ( compass_ok ) {
		status = "Calc sWind";
		// ESP_LOGI(FNAME,"%d TC: %3.1f (avg:%3.1f) GS:%3.1f TH: %3.1f (avg:%3.1f) TAS: %3.1f",
		//   nunberOfSamples, ctc, averageTC, cgs, cth, averageTH, ctas );
		calculateWind( averageTC, averageGS, averageTH, averageTas, deviation );
		// also compute zWind for testing
		(void) calculatezWind( averageTC, cgs, averageTas, false );
		return true;
	}

	status = "Calc zWind";
	return calculatezWind( averageTC, cgs, averageTas, true );   // current ground speed cgs, not averageGS
}


// length (or speed) of third vector in windtriangle
// and calculate WA (wind angle) in degree
// wind direction calculation taken from here:
// view-source:http://www.owoba.de/fliegerei/flugrechner.html
// direction in degrees of third vector in windtriangle
void StraightWind::calculateSpeedAndAngle( float angle1, float speed1, float angle2, float speed2, float& speed, float& angle ){
	float tcrad = D2R( angle1 );
	float thrad = D2R( angle2 );
	float wca = Vector::angleDiffRad( thrad, tcrad );
	float s2wca = speed2 * cos( wca );
	float ang = tcrad + atan2( speed2 * sin( wca ), s2wca - speed1 );
	// Cosinus sentence: c^2 = a^2 + b^2 − 2 * a * b * cos( α ) for wind speed in km/h
	speed = sqrt( (speed2 * speed2) + (speed1 * speed1 ) - ( 2 * s2wca * speed1  ) );
	angle = Vector::normalizeDeg( R2D( ang ) );  // convert radian to degree
	// ESP_LOGI(FNAME,"calcAngleSpeed( A1/S1=%3.1f°/%3.1f km/h  A2/S2=%3.1f°/%3.1f km/h): A/S: %3.2f°/%3.2f km/h", 
	//  angle1, speed1, angle2, speed2, angle, speed  );
}

float StraightWind::getAngle() { return swind_dir.get(); };
float StraightWind::getSpeed() { return swind_speed.get(); };

void StraightWind::init_zWgt()
{
	float f = wind_filter_lowpass.get();
	if (f < 11.0)  f = 11.0;        // menu allows down to 5 for compass-wind
	zWgt = 8.0 / (2.5 + 0.5*f);
	//if (zWgt > 1.0)  zWgt = 1.0;
	ESP_LOGI(FNAME,"initial averaging weight: %.3f", zWgt);
	zWgtChg = false;
}

// Compute wind without compass, using TAS and iterative approximation on a zig-zag path.
// Algorithm: estimate airspeed vector from TAS and a WCA based on previous wind estimate.
// The interesting thing is that this simple algorithm converges to the true wind.
bool StraightWind::calculatezWind( float tc, float gs, float tas, bool overwrite ){

	if (_tick & 0x03)
		return false;    // only calculate once every 4 sec

	if ( (wind_enable.get() == WA_BOTH) && circlingWindAge < 1200 && circlingWindAge < _age) {
		// circling wind estimate is more recent,
		// re-initialize the zwind estimate
		zwindSpeed = circlingWindSpeed;
		zwindDir   = circlingWindDir;
		zcount = (int)wind_filter_lowpass.get();        // report right away
		init_zWgt();
		ESP_LOGI(FNAME,"calculatezWind: initialized zwind based on cwind");
	}

	if (zcount == -999) {  // not initialized
		// initialize based on difference between GS and TAS
		if (gs < tas) {             // headwind
			zwindSpeed = tas - gs;
			zwindDir = tc;          // direction the wind is coming from
		} else {
			zwindSpeed = gs - tas;
			zwindDir = Vector::normalizeDeg(tc+180);
		}
		zminDir = zwindDir;
		zmaxDir = zwindDir;
		zcount = -(int)wind_filter_lowpass.get();
		init_zWgt();
		ESP_LOGI(FNAME,"calculatezWind: initial wind estimate: from %.1f, kph %.1f", zwindDir, zwindSpeed);
	}

	Vector oldwind( zwindDir, zwindSpeed );   // previous wind estimate
	Vector vgs( tc, gs );                     // groundspeed
	Vector newwind = vgs;
	newwind.add( oldwind );                   // gives approximate wind correction angle
	newwind.setSpeedKmh( tas );               // same direction, but the actual TAS
	newwind.subtract( vgs );                  // this is the new wind estimate

	newWindDir   = newwind.getAngleDeg();
	newWindSpeed = newwind.getSpeed();
	ESP_LOGI(FNAME,"New zWind: %3.1f deg,  %3.1f km/h", newwind.getAngleDeg(), newwind.getSpeed() );

	// as long as TAS, heading, and wind stay the same,
	//   the newwind will equal the oldwind.

	// average the new estimate into the established average estimate
	newwind.scale( zWgt );
	oldwind.scale( 1.0 - zWgt );
	newwind.add( oldwind );

	zwindDir   = newwind.getAngleDeg();
	zwindSpeed = newwind.getSpeed();
	ESP_LOGI(FNAME,"New AVG zWind: %3.1f deg,  %3.1f km/h", zwindDir, zwindSpeed );

	// Is the sample sufficient to bother to display it?
	// Require diverse directions plus some more samples
	// - even then it may not yet be a good estimate
	if (zcount < 0) {
		if (Vector::angleDiffDeg(zwindDir,zminDir) < 0)
			zminDir = zwindDir;
		if (Vector::angleDiffDeg(zwindDir,zmaxDir) > 0)
			zmaxDir = zwindDir;
		if (Vector::angleDiffDeg(zmaxDir,zminDir) > 40.0)
			zcount = 0;
	}
	zcount++;
	if (zcount < (((int)wind_filter_lowpass.get())>>2) && !testmode.get())
		return false;
	if (! zWgtChg && zcount >= (int)wind_filter_lowpass.get()) {
		zWgt *= 0.5;  // reduce noise in the average
		ESP_LOGI(FNAME,"lowered averaging weight to: %.3f", zWgt);
		zWgtChg = true;
	}

	if ( ! overwrite )   // do not overwrite swind_dir and swind.speed
		return true;

	_age = 0;   // not really needed, since also called from setupNG.cpp resetSWindAge()
	if( (int)zwindDir != (int)swind_dir.get() )
		swind_dir.set( zwindDir );
	if( (int)zwindSpeed != (int)swind_speed.get() )
		swind_speed.set( zwindSpeed );
	return true;
}

void StraightWind::calculateWind( float tc, float gs, float th, float tas, float deviation ){
	// ESP_LOGI(FNAME,"calculateWind: TC:%3.1f GS:%3.1f TH:%3.1f TAS:%3.1f Dev:%2.2f", tc, gs, th, tas, deviation );
	if( gs < 5 )
		tc = th;   // what will deliver heading and airspeed for wind
	bool devOK = true;
	// Reverse calculate windtriangle for deviation and airspeed calibration
	if( circlingWindSpeed > 0 && compass_dev_auto.get() ){
		if( circlingWindAge > 1200 ){
			status = "OLD CIRC WIND";
			ESP_LOGI(FNAME,"Circling Wind expired");
		}else{
			float airspeed = 0;
			float heading = 0;
			Vector wind( circlingWindDir, circlingWindSpeed );
			Vector groundTrack( tc, gs );
			groundTrack.add( wind );
			airspeed = groundTrack.getSpeed();
			heading = groundTrack.getAngleDeg();
//#ifdef VERBOSE_LOG
			ESP_LOGI(FNAME,"Using CWind: %.2f°/%.2f, TC/GS: %.1f°/%.1f, HD/AS: %.2f°/%.2f, tas=%.2f, ASdelta %.3f",
			  circlingWindDir, circlingWindSpeed, tc, gs, heading, airspeed, tas, airspeed-tas );
// #endif
			if( abs( airspeed - tas ) > wind_straight_speed_tolerance.get() ){  // 30 percent max deviation
				status = "AS OOB";
				ESP_LOGI(FNAME,"Estimated Airspeed/Groundspeed OOB, max delta: %f km/h, delta: %f km/h",
				  wind_straight_speed_tolerance.get(), abs( airspeed - tas ) );
				return;
			}
			airspeedCorrection +=  (airspeed/tas - airspeedCorrection) * wind_as_filter.get();
			if( airspeedCorrection > 1.01 ) // we consider 1% as maximum needed correction
				airspeedCorrection = 1.01;
			else if( airspeedCorrection < 0.99 )
				airspeedCorrection = 0.99;
			if( abs( wind_as_calibration.get() - airspeedCorrection )*100 > 0.5 )
					wind_as_calibration.set( airspeedCorrection );
			if( compass )
				devOK = compass->newDeviation( th, heading );
			else{
				status = "No Compass";
				return;
			}
			// ESP_LOGI(FNAME,"Calculated TH/TAS: %3.1f°/%3.1f km/h  Measured TH/TAS: %3.1f°/%3.1f, asCorr:%2.3f, deltaAS:%3.2f, Age:%d", tH, airspeed, averageTH, tas, airspeedCorrection , airspeed-tas, circlingWindAge );
		}
	}else{
		status = "No Circ Wind";
		// float airspeed = calculateSpeed( windDir, windSpeed, tc, gs );
		// airspeedCorrection +=  (airspeed/tas - airspeedCorrection) * wind_as_filter.get();
	}
	if( !devOK ){ // data is not plausible/useful
			ESP_LOGI( FNAME, "Calculated deviation out of bounds: Drop also this wind calculation");
			status = "Deviation OOB";
			return;
	}

    // wind speed and direction
	calculateSpeedAndAngle( tc, gs, th+deviation, tas*airspeedCorrection, newWindSpeed, newWindDir );
	// ESP_LOGI( FNAME, "Calculated raw windspeed %.1f jitter:%.1f", newWindSpeed, jitter );

	if (wind_enable.get() == WA_BOTH && circlingWindAge < 1200 && circlingWindAge < _age) {
		// circling wind estimate is more recent,
		// re-initialize the swind estimate
		windVectors.clear();
		result = Vector( circlingWindDir, circlingWindSpeed );
		windVectors.push_back( result );
		windVectors.push_back( result );
		windVectors.push_back( result );
		windVectors.push_back( result );   // weigh old circling wind 4x the new straight wind
		result.add( result );              // 2x
		result.add( result );              // 4x
		_n_avg = 0;
	}

	Vector v;
	v.setAngleDeg( newWindDir );
	v.setSpeedKmh( newWindSpeed );

	windVectors.push_back( v );
	result.add( v );
	while( windVectors.size() > (int)wind_filter_lowpass.get() ){
		result.subtract( windVectors.front() );
		windVectors.pop_front();
	}

	// >>> will doing this for a long time accumulate numerical errors?
	// >>> compute a clean average occasionally:
	_n_avg++;
	//if (_n_avg > (((int)wind_filter_lowpass.get())<<1)) {
	if (_n_avg > (int)wind_filter_lowpass.get()) {
		result = Vector( 0.0, 0.0 );
		for( auto it=std::begin(windVectors); it != std::end(windVectors); it++ ){
			result.add( *it );
		}
		_n_avg = 0;
	}

	swindDir   = result.getAngleDeg(); // Vector::normalizeDeg( result.getAngleDeg()/circle_wind_lowpass.get() );
	swindSpeed = result.getSpeed() / windVectors.size();

	// ESP_LOGI(FNAME,"New AVG WindDirection: %3.1f deg,  Strength: %3.1f km/h JI:%2.1f", windDir, windSpeed, jitter );
	_age = 0;   // not really needed, since also called from setupNG.cpp resetSWindAge()
	if( (int)swindDir != (int)swind_dir.get()  ){
		swind_dir.set( swindDir );
	}
	if( (int)swindSpeed != (int)swind_speed.get() ){
		swind_speed.set( swindSpeed );
	}
}


void StraightWind::newCirclingWind( float angle, float speed ){
	ESP_LOGI(FNAME,"New good circling wind %3.2f°/%3.2f", angle, speed );
	circlingWindDir = angle;
	circlingWindDirReverse = Vector::reverse( angle );      // reverse windvector
	ESP_LOGI(FNAME,"Wind dir %3.2f, reverse circling wind dir %3.2f", angle, circlingWindDirReverse );
	circlingWindSpeed = speed;
	circlingWindAge = 0;
}

#if 0
void StraightWind::test()
{    // Class Test, check here the results: http://www.owoba.de/fliegerei/flugrechner.html
	calculateWind( 90, 100, 0, 100, 0 );
	if( int( windSpeed ) != 141 || int(windDir +0.5) != 135 )
		ESP_LOGI(FNAME,"Failed");
	calculateWind( 270, 100, 0, 100, 0 );
	if( int( windSpeed ) != 141 || int(windDir +0.5) != 225 )
		ESP_LOGI(FNAME,"Failed");

	calculateWind( 0, 100, 90, 100, 0 );
	if( int( windSpeed ) != 141 || int(windDir +0.5) != 315 )
		ESP_LOGI(FNAME,"Failed");
	calculateWind( 0, 100, 270, 100, 0 );
	if( int( windSpeed ) != 141 || int(windDir +0.5) != 45 )
		ESP_LOGI(FNAME,"Failed");

	calculateWind( 90, 100, 180, 100, 0 );
	if( int( windSpeed ) != 141 || int(windDir +0.5) != 45 )
		ESP_LOGI(FNAME,"Failed");

	calculateWind( 180, 100, 270, 100, 0 );
	if( int( windSpeed ) != 141 || int(windDir +0.5) != 135  )
		ESP_LOGI(FNAME,"Failed");
}
#endif
