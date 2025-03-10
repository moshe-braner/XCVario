/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ****************************************************************************

File: ShowStraightWind.cpp

Class to display the compass status overview.

Author: Eckhard Völlm, May 2021

Last update: 2021-04-18

 ****************************************************************************/

#include "StraightWind.h"
#include "ShowStraightWind.h"
#include "SetupNG.h"
#include "Units.h"
#include "sensor.h"
#include "Compass.h"

#include <esp_log.h>


ShowStraightWind::ShowStraightWind( const char* title ) :
SetupMenuDisplay( title, nullptr )
{
	ESP_LOGI(FNAME, "ShowStraightWind(): title='%s'", title );
}

void ShowStraightWind::display( int mode )
{
	if( (selected != this) || !gflags.inSetup )
		return;

	//ESP_LOGI(FNAME, "display() mode=%d", mode );
	if( mode != 5 )
		clear();
	ucg->setFont( ucg_font_ncenR14_hr );
	uprintf( 5, 25, selected->_title );

	uint16_t y = 70;
	char buffer[32];

	semaphoreTake();

/*
	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Straight Wind enabled: %s", (wind_enable.get() & WA_STRAIGHT) ? "Yes" : "No  "  );
	ucg->printf( "%s", buffer );
	y += 25;
*/

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Status: %s     ", theWind.getStatus() );
	ucg->printf( "%s", buffer );
	y += 25;

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Last  Str Wind : %3.1f°/%2.1f   ", theWind.getAngle(), Units::Airspeed( theWind.getSpeed()) );
	ucg->printf( "%s", buffer );
	y += 25;

	if (compass) {    // also show ZZ wind for testing

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Last ZZ Wind :   %03d°/%2.1f   ", (int)theWind.getzAngle(), Units::Airspeed( theWind.getzSpeed()) );
	ucg->printf( "%s", buffer );
	y += 25;

	}

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "GPS Status : %s", (theWind.getGpsStatus() == true ) ? "Good" : "Bad  "  );
	ucg->printf( "%s", buffer );
	y += 25;

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "AS C/F: %+3.3f %%/%3.3f %%  ", (theWind.getAsCorrection()-1.0)*100, (wind_as_calibration.get()-1.0)*100 );
	ucg->printf( "%s", buffer );
	y += 25;

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "MH/Dev: %3.2f/%+3.2f   ", theWind.getMH(), theWind.getDeviation() );
	ucg->printf( "%s", buffer );
	y += 25;

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Wind Age : %d sec   ", theWind.getAge() );
	ucg->printf( "%s", buffer );
	y += 25;


	ucg->setPrintPos( 5, 310 );
	ucg->printf( "Press button to exit" );

	semaphoreGive();

}


ShowBothWinds::ShowBothWinds( const char* title ) :
SetupMenuDisplay( title, nullptr )
{
	ESP_LOGI(FNAME, "ShowBothWinds(): title='%s'", title );
}

void ShowBothWinds::display( int mode )
{
	if( (selected != this) || !gflags.inSetup )
		return;

	//ESP_LOGI(FNAME, "display() mode=%d", mode );
	if( mode != 5 )
		clear();
	ucg->setFont( ucg_font_ncenR14_hr );
	uprintf( 5, 25, selected->_title );

	uint16_t y = 70;
	char buffer[32];

	semaphoreTake();

	if (wind_enable.get() & WA_STRAIGHT) {

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Status: %s     ", theWind.getStatus() );
	ucg->printf( "%s", buffer );
	y += 25;

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Last Str Wind : %03d°/%2.1f   ", (int)theWind.getAngle(), Units::Airspeed( theWind.getSpeed()) );
	ucg->printf( "%s", buffer );
	y += 25;

	if (compass) {    // also show ZZ wind for testing

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Last ZZ Wind :  %03d°/%2.1f   ", (int)theWind.getzAngle(), Units::Airspeed( theWind.getzSpeed()) );
	ucg->printf( "%s", buffer );
	y += 25;

	}

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Str Wind Age : %d sec   ", theWind.getAge() );
	ucg->printf( "%s", buffer );
	y += 25;

	}

	int cwinddir=0;
	float cwind=0;
	int ageCircling;

	if (wind_enable.get() & WA_CIRCLING) {

	bool r = CircleWind::getWind( &cwinddir, &cwind, &ageCircling );
	if (r == false) {
		ucg->setPrintPos( 0, y );
		sprintf( buffer, "Circle Wind not current");
		ucg->printf( "%s", buffer );
		y += 25;
	}

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Last Cir Wind : %03d°/%2.1f   ", cwinddir, Units::Airspeed( cwind ) );
	ucg->printf( "%s", buffer );
	y += 25;

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Cir Wind Age : %d sec   ", ageCircling );
	ucg->printf( "%s", buffer );
	y += 25;

	}

	if ( wind_enable.get() == WA_OFF ) {

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "Wind estimation disabled   ");
	ucg->printf( "%s", buffer );
	y += 25;

	}

	ucg->setPrintPos( 0, y );
	sprintf( buffer, "GPS Status : %s", (theWind.getGpsStatus() == true ) ? "Good" : "Bad  "  );
	ucg->printf( "%s", buffer );
	y += 25;

	ucg->setPrintPos( 5, 310 );
	ucg->printf( "Press button to exit" );

	semaphoreGive();
}
