/*
 * Horizon.cpp
 *
 *  Created: 2023-2024
 *  Author: Moshe Braner
 *
 */

#include "IpsDisplay.h"
#include "sensor.h"
#include "Units.h"
#include "Flarm.h"
#include "vector.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <AdaptUGC.h>
#include <logdef.h>
#include "sdkconfig.h"
#include <cmath>
#include <cstdio>
#include <cstring>

extern xSemaphoreHandle spiMutex;

#define ucg IpsDisplay::ucg

#define WIDTH_2  (DISPLAY_W/2)        // 120
#define HEIGHT_2 (DISPLAY_H/2)        // 160
#define WIDTH_   (DISPLAY_W-1)        // 239
#define HEIGHT_  (DISPLAY_H-1)        // 319
#define AHRS_TOP (HEIGHT_2-WIDTH_2)   //  40
#define AHRS_BOT (HEIGHT_2+WIDTH_2)-1 // 279

const static ucg_color_t skycolor[4] = {{COLOR_DSKY},   {COLOR_LSKY},   {COLOR_LSKY},   {COLOR_BLACK}};
const static ucg_color_t gndcolor[4] = {{COLOR_DGROUND},{COLOR_LGROUND},{COLOR_LGROUND},{COLOR_BLACK}};
const static ucg_color_t hzncolor[4] = {{COLOR_WHITE},  {COLOR_BLACK},  {COLOR_WHITE},  {COLOR_WHITE}};
static const ucg_color_t *sky_color;
static const ucg_color_t *gnd_color;
static const ucg_color_t *hzn_color;

void IpsDisplay::HznSetColor( const ucg_color_t *color ) {
	ucg->setColor( (color->color)[0], (color->color)[1], (color->color)[2] );
}

static int hzn_prange = -1;
static int p5 = 0;
static int p10;
static int p15;
static int p20;
static int p25;
static int p30;
static int p35;
static int p40;
static int p45;
static int p50;
static int p60;
static int p70;
// p80 is too close to p70 to bother
static int p90;

static float pitch_offset = 0;   // reflects the NG horizon_offset

// current and previous horizon state variables
static int hzn_x0, hzn_x1, hzn_y0, hzn_y1;
static int old_x0, old_x1, old_y0, old_y1;
static int pitchpixels, oldpitchpixels;        // unlimited
static bool pitch_ticks_drawn = false;
static bool bank_ticks_drawn = false;
// these are sin() & cos() of the bank angle, as integers scaled 256x
static int sin_bank = 0;
static int cos_bank = 0x100;
static int old_sin_bank = 0;
static int old_cos_bank = 0x100;

static float limited_pitch( float p ) {
//	if (hzn_prange) {   // '90-degree' range
		if (p >  D2R(88))  return  D2R(88);
		if (p < -D2R(88))  return -D2R(88);
//	} else {            // '30-degree' range
//		if (p >  D2R(48))  return  D2R(48);
//		if (p < -D2R(48))  return -D2R(48);
//	}
	return p;
}

static float limited_bank( float b ) {
	// limit to a bit under 90 degrees
	if (b > D2R(88))  return D2R(88);
	else if (b < -D2R(88))  return -D2R(88);
	else  return b;
}

// these approximations take angles in radians:
static float approxsin( float x ) {
	if (x > 0) {
		if (x > 2.3562)                      // x > 135 degrees (but < 180)
			return approxsin( 3.1416-x );   // now x < 45
		else if (x > 0.7854)                // x > 45 degrees
			return approxcos( 1.5708-x );   // approxcos() more accurate
	} else {
		if (x < -2.3562)
			return approxsin( -3.1416-x );
		else if (x < -0.7854)
			return -approxcos( x+1.5708 );
	}
	return (x*(1-0.1667*x*x));
}
static float approxcos( float x ) {
	if (x > 0) {
		if (x > 2.3562)  // 135 degrees
			return -approxcos( 3.1416-x );
		if (x > 0.7854)  // 45 degrees
			return approxsin( 1.5708-x );
	} else {
		if (x < -2.3562)  // -135 degrees
			return -approxcos( x+3.1416 );
		if (x < -0.7854)  // -45 degrees
			return approxsin( x+1.5708 );
	}
	x = 0.5*x*x;
	return (1.0 - x + 0.1667*x*x);
}

static int pitch2pixels( float p ) {
	if( pitch_offset != 0 )
		p += D2R(pitch_offset);
	// Move center of horizon up or down by approximately sin(pitch)
	// The 2.3 is a visual exaggeration factor
	float sp = approxsin( p );
	if (hzn_prange != 0)
		return( (int)(WIDTH_2 * sp) );
	return( (int)((WIDTH_2 * 2.3) * sp) );
}

// Draw a pair of lines, after checking that they are in display range
// The "which" parameter allows changing colors in-between the two lines.
// Try and draw the second line "below" the first one (towards ground).
void IpsDisplay::double_line( int x1, int y1, int x2, int y2, int which ) {
	xSemaphoreTake(spiMutex, portMAX_DELAY );
	if ( x1 >= 0 && x1 <= WIDTH_
	  && x2 >= 0 && x2 <= WIDTH_
	  && y1 >= AHRS_TOP && y1 <= AHRS_BOT
	  && y2 >= AHRS_TOP && y2 <= AHRS_BOT) {
		if( which != 1 ) {   // 0 or 2
			ucg->drawLine( x1,y1, x2,y2 );
		}
		if( which != 0 ) {   // 1 or 2
			// draw another line next to it to make it thicker
			if( abs(y2-y1) > abs(x2-x1) ) {  // more vertical than horizontal
				if( y1 > y2 )
					ucg->drawLine( ++x1,y1, ++x2,y2 );
				else if( y1 < y2 )
					ucg->drawLine( --x1,y1, --x2,y2 );
			} else {
				if( y1 < AHRS_BOT && y2 < AHRS_BOT )
					ucg->drawLine( x1,++y1, x2,++y2 );
			}
		}
	}
	xSemaphoreGive(spiMutex);
}

// Draw (or erase) the pitch ticks parallel to the horizon
// Note: horizon, and ticks, move when pitch offset is changed
void IpsDisplay::pitch_tick( bool draw, bool major, int pt ) {
	int sb, cb, pp;
	if (draw) {
		sb = sin_bank;
		cb = cos_bank;
		pp = pitchpixels;
	} else {          // erase old tick
		sb = old_sin_bank;
		cb = old_cos_bank;
		pp = oldpitchpixels;
	}
	// point on horizon that is abeam the middle of the display/airplane:
//	int x = WIDTH_2  + ((pp*sb)>>8);
//	int y = HEIGHT_2 + ((pp*cb)>>8);
	// move it "up" (perpendicular to horizon) by pt pixels:
//	x -= ((pt*sb)>>8);
//	y -= ((pt*cb)>>8);
	// can combine the two calculations
	int x = WIDTH_2  + (((pp-pt)*sb)>>8);
	int y = HEIGHT_2 + (((pp-pt)*cb)>>8);
	int w =  ((WIDTH_ * cb)>>8);    // horizontal tick size, based on bank
	int v = -((WIDTH_ * sb)>>8);    // vertical tick size
	w >>= (major?4:5);     // half-width of tick projected along horizontal axis
	v >>= (major?4:5);     // half-width of tick projected along vertical axis
	if (draw)
		HznSetColor( hzn_color );
	// erase with color based on which side of the *new* horizon we are:
	else if (pitchpixels > oldpitchpixels - pt)   // may not work if bank changed a lot?
		HznSetColor( sky_color );
	else
		HznSetColor( gnd_color );
	double_line( x-w,y-v, x+w,y+v );
}

bool IpsDisplay::pitch_ticks( bool draw ) {
	bool drawn = false;
	int pp;
	if (draw)
		pp = pitchpixels;
	else
		pp = oldpitchpixels;
	bool all_ticks = (horizon_pticks.get() == 2);
	bool punlimited = (horizon_plimit.get() == 0);
	if (hzn_prange != 0) {       // 90-degree range
	if ( pp > p5 ) {
		// draw ticks above the horizon
		drawn = true;
		pitch_tick( draw, false,  p10 );   // 10 deg minor tick
	if ( pp > p10 ) {
		pitch_tick( draw, false,  p20 );   // 20 deg minor tick
	if ( all_ticks || pp > p20 ) {
		pitch_tick( draw, true,   p30 );   // 30 deg major tick
	if ( all_ticks || pp > p30 ) {
		pitch_tick( draw, false,  p40 );   // 40 deg minor tick
	if ( all_ticks || pp > p40 ) {
		pitch_tick( draw, false,  p50 );   // 50 deg minor tick
	if ( all_ticks || pp > p50 ) {
		pitch_tick( draw, true,   p60 );   // 60 deg major tick
	if ( punlimited && (all_ticks || pp > p60 )) {
//		pitch_tick( draw, false,  p70 );   // 70 deg minor tick
	if ( punlimited && (all_ticks || pp > p70 ))
		pitch_tick( draw, true,   p90 );   // 90 deg major tick
	}}}}}}} else
	if ( pp < -p5 ) {
		// draw ticks below the horizon
		drawn = true;
		pitch_tick( draw, false, -p10 );   // 10 deg minor tick
	if ( pp < -p10 ) {
		pitch_tick( draw, false, -p20 );   // 20 deg minor tick
	if ( all_ticks || pp < -p20 ) {
		pitch_tick( draw, true,  -p30 );   // 30 deg major tick
	if ( all_ticks || pp < -p30 ) {
		pitch_tick( draw, false, -p40 );   // 40 deg minor tick
	if ( all_ticks || pp < -p40 ) {
		pitch_tick( draw, false, -p50 );   // 50 deg minor tick
	if ( all_ticks || pp < -p50 ) {
		pitch_tick( draw, true,  -p60 );   // 60 deg major tick
	if ( punlimited && (all_ticks || pp < -p60 )) {
//		pitch_tick( draw, false, -p70 );   // 70 deg minor tick
	if ( punlimited && (all_ticks || pp < -p70 ))
		pitch_tick( draw, true,  -p90 );   // 90 deg major tick
	}}}}}}}
	} else {                               // 30-degree range
	if ( pp > 15 ) {              //  3-degree threshold
		// draw ticks above the horizon
		drawn = true;
		pitch_tick( draw, false,  p5 );    //  5 deg minor tick
	if ( pp > p5 ) {
		pitch_tick( draw, false,  p10 );   // 10 deg minor tick
	if ( all_ticks || pp > p10 ) {
		pitch_tick( draw, true,   p15 );   // 15 deg major tick
	if ( all_ticks || pp > p15 ) {
		pitch_tick( draw, false,  p20 );   // 20 deg minor tick
	if ( all_ticks || pp > p20 ) {
		pitch_tick( draw, false,  p25 );   // 25 deg minor tick
	if ( all_ticks || pp > p25 ) {
		pitch_tick( draw, true,   p30 );   // 30 deg major tick
	if ( all_ticks || pp > p30 ) {
		pitch_tick( draw, false,  p35 );   // 35 deg minor tick
	if ( all_ticks || pp > p35 ) {
		pitch_tick( draw, false,  p40 );   // 40 deg minor tick
	if ( punlimited && (all_ticks || pp > p40 )) {
		pitch_tick( draw, true,   p45 );   // 45 deg major tick
	if ( punlimited && (all_ticks || pp > p45 ))
		pitch_tick( draw, false,  p50 );   // 50 deg minor tick
	}}}}}}}}} else
	if ( pp < -15 ) {
		// draw ticks below the horizon
		drawn = true;
		pitch_tick( draw, false,  -p5 );   //  5 deg minor tick
	if ( pp < -p5 ) {
		pitch_tick( draw, false, -p10 );   // 10 deg minor tick
	if ( all_ticks || pp < -p10 ) {
		pitch_tick( draw, true,  -p15 );   // 15 deg major tick
	if ( all_ticks || pp < -p15 ) {
		pitch_tick( draw, false, -p20 );   // 20 deg minor tick
	if ( all_ticks || pp < -p20 ) {
		pitch_tick( draw, false, -p25 );   // 25 deg minor tick
	if ( all_ticks || pp < -p25 ) {
		pitch_tick( draw, true,  -p30 );   // 30 deg major tick
	if ( all_ticks || pp < -p30 ) {
		pitch_tick( draw, false, -p35 );   // 35 deg minor tick
	if ( all_ticks || pp < -p35 ) {
		pitch_tick( draw, false, -p40 );   // 40 deg minor tick
	if ( punlimited && (all_ticks || pp < -p40 )) {
		pitch_tick( draw, true,  -p45 );   // 45 deg major tick
	if ( punlimited && (all_ticks || pp < -p45 )) 
		pitch_tick( draw, false, -p50 );   // 50 deg major tick
	}}}}}}}}}
	}
	return drawn;
}

// Stages of erasing and drawing bank ticks:
//   0 - early - erase old ticks that cross new horizon
//   1 - late - erase old ticks that do not cross new horizon
//   2 - draw new ticks that do not cross new horizon
//   3 - replaces 0,1,2 sometimes - erase all old ticks, do not draw new

bool IpsDisplay::bank_tick( int stage, int x1, int y1, int x2, int y2 ) {
	//ESP_LOGI(FNAME,"bank_tick( stage=%d, y=%d )", stage, y1 );
	bool side1, side2;
	// extend past tick ends to be sure double-line does not cross
	int xx1, yy1, xx2, yy2;
	if (x1 < x2) {
		xx1 = x1 - 1;
		xx2 = x2 + 1;
	} else {
		xx1 = x1 + 1;
		xx2 = x2 - 1;
	}
	if (y1 < y2) {
		yy1 = y1 - 1;
		yy2 = y2 + 1;
	} else {
		yy1 = y1 + 1;
		yy2 = y2 - 1;
	}
	if (stage != 2) {    // erasing
		// determine whether tick crosses *old* horizon line
		side1 = ((xx1-old_x0)*(old_y1-old_y0) > (yy1-old_y0)*(old_x1-old_x0));
		side2 = ((xx2-old_x0)*(old_y1-old_y0) > (yy2-old_y0)*(old_x1-old_x0));
		if (side1 != side2) {   // ends of tick are on opposite sides of old horizon
			//ESP_LOGI(FNAME,"... old rejected");
			return false;    // old tick was not drawn, no need to erase
		}
		if( stage == 0 || stage == 3 ) {    // early erasing, color based on old horizon
			if (old_x1 == old_x0)           // horizon off-screen
				side1 = (old_y0+old_y1 > DISPLAY_H);
			if( side1 )
				HznSetColor( sky_color );
			else
				HznSetColor( gnd_color );
		}
	} else {   // stage == 2, drawing
		HznSetColor( hzn_color );
	}
	// determine whether tick crosses *new* horizon line
	side1 = ((xx1-hzn_x0)*(hzn_y1-hzn_y0) > (yy1-hzn_y0)*(hzn_x1-hzn_x0));
	side2 = ((xx2-hzn_x0)*(hzn_y1-hzn_y0) > (yy2-hzn_y0)*(hzn_x1-hzn_x0));
	if (side1 != side2) {    // ends of tick are on opposite sides of new horizon
		if (stage == 1) {     // should have been erased in stage 0
			//ESP_LOGI(FNAME,"... presumed done earlier");
			return false;
		} else if (stage == 2) {
			//ESP_LOGI(FNAME,"... new rejected");
			return false;      // do not draw
		}
	} else {                    // tick does not cross new horizon
		if (stage == 0) {
			//ESP_LOGI(FNAME,"... postponed");
			return false;      // erase it later in stage 1
		}
		// if stage == 3 (early & will not draw new) then do not postpone
	}
	if ( stage == 1 ) {     // late erasing, color based on new horizon position
		if (hzn_x1 == hzn_x0)           // horizon off-screen
			side1 = (hzn_y0+hzn_y1 > DISPLAY_H);
		if( side1 )
			HznSetColor( sky_color );
		else
			HznSetColor( gnd_color );
	}
	double_line( x1, y1, x2, y2 );
	return true;
}

bool IpsDisplay::ticks_bank( int stage, int bank ) {
	int sinbb, cosbb, sbt;
	bool major;
	if (bank == 15) {
		sinbb = 66;    // 256*sin(15deg)
		cosbb = 247;   // 256*cos(15deg)
		sbt = 33;      // threshold sin(current bank) for drawing this tick
		major = false;
	} else if (bank == 30) {
		sinbb = 128;   // 256*sin(30deg)
		cosbb = 222;   // 256*cos(30deg)
		sbt = 66;  // 96
		major = true;
	} else if (bank == 45) {
		sinbb = 181;   // 256*sin(45deg)
		cosbb = 181;   // 256*cos(45deg)
		sbt = 128;  // 156
		major = false;
	} else if (bank == 60) {
		sinbb = 219;   // 256*sin(60deg)
		cosbb = 116;   // 256*cos(60deg)
		sbt = 181;  // 202
		major = true;
	} else {
		bank = 0;
		sinbb = 0;
		cosbb = 256;
		sbt = 33;
	}
	int sb, cb, pp;
	if( stage == 2 ) {      // drawing new
		sb = sin_bank;
		cb = cos_bank;
		pp = pitchpixels;
	} else {                // erasing old
		sb = old_sin_bank;
		cb = old_cos_bank;
		pp = oldpitchpixels;
	}
	int abs_sb = abs(sb);
	// do not show ticks until current bank > threshold (or show-all option)
	if ( abs_sb <= sbt && horizon_bticks.get() != 2 )
		return false;
	// try not to draw *on* the horizon since that is hard to "erase"
	if ( bank == 0 && abs(pp) < 10 )
		return false;
	int m = WIDTH_2;        // draw the bank ticks around the airplane in the center
	int n = HEIGHT_2;
	int h = WIDTH_2 - 4;
	// use trig formulas for sin & cos of (bb-b), arranged for integer math
	int w4 = ((h * ((cb*cosbb + abs_sb*sinbb)>>8))>>8);   // horizontal size of tick (>0)
	int z4 = ((h * ((cb*sinbb - abs_sb*cosbb)>>8))>>8);   // vertical offset of tick
	int w3, z3;
	if (bank == 0) {
		// long ticks from 1/4-radius to full radius of circle (or more)
		w3 = (w4>>2);
		z3 = (z4>>2);
		if (cb < 210 && abs_sb < 210) {  // enough room on the far ends
			w4 = w4 + (w4>>3);
			z4 = z4 + (z4>>3);
		}
	} else {
		// if major, ticks from 3/4-radius to full radius of circle
		w3 = w4 - (w4>>2);
		z3 = z4 - (z4>>2);
		if ( ! major ) {  // ticks from 3/4-radius to 7/8-radius of circle
			w4 = w4 - (w4>>3);
			z4 = z4 - (z4>>3);
		}
	}
	// Only draw ticks in the 2 relevant quadrants out of 4
	bool drawn1 = false;
	bool drawn2 = false;
	if( sb > 0 ) {
		drawn1 = bank_tick( stage, m-w3,n-z3, m-w4,n-z4 );
		drawn2 = bank_tick( stage, m+w3,n+z3, m+w4,n+z4 );
	}
	if( sb < 0 ) {
		drawn1 = bank_tick( stage, m-w3,n+z3, m-w4,n+z4 );
		drawn2 = bank_tick( stage, m+w3,n-z3, m+w4,n-z4 );
	}
	return (drawn1 || drawn2);
}

bool IpsDisplay::bank_ticks( bool drawn, bool draw, bool early, int bank ) {
	if (early) {
		if (drawn) {
			if ( !draw ) {
				(void) ticks_bank( 3, bank );    // erase all old early
				drawn = false;                   // skip the late stages
			} else {
				(void) ticks_bank( 0, bank );    // erase some old
			}
		}
	} else {
		if (drawn) {
			(void) ticks_bank( 1, bank );        // erase rest of old
			drawn = false;
		}
		if (draw)
			drawn = ticks_bank( 2, bank );       // draw new
	}
	return drawn;
}

// ticks for 0, 15, 30, 45 and 60 degrees bank
bool IpsDisplay::banks_ticks( bool drawn, bool draw, bool early ) {
	if ( !drawn && !draw )
		return false;
	bool drawn0  = bank_ticks( drawn, draw, early,  0 );
	bool drawn15 = bank_ticks( drawn, draw, early, 15 );
	bool drawn30 = bank_ticks( drawn, draw, early, 30 );
	bool drawn45 = bank_ticks( drawn, draw, early, 45 );
	bool drawn60 = bank_ticks( drawn, draw, early, 60 );
	return (drawn0 || drawn15 || drawn30 || drawn45 || drawn60);   // at least one tick was drawn
	// note that higher-bank tick may be drawn while lower skipped due to crossing horizon
}

// draw a simple "airplane" icon, scaled to use 3/4 of the display width
void IpsDisplay::airplane_icon( bool draw ) {
	// The following variables are used to mark the unsafe zone, where other objects
	//   may have been painted over when the horizon was drawn.
	int k0 = ((old_y0 < hzn_y0)? old_y0 : hzn_y0);
	int k1 = ((old_y1 < hzn_y1)? old_y1 : hzn_y1);
	int g0 = ((old_y0 > hzn_y0)? old_y0 : hzn_y0);
	int g1 = ((old_y1 > hzn_y1)? old_y1 : hzn_y1);
	int kmid = (k0+k1)>>1;
	int gmid = (g0+g1)>>1;
	int m = WIDTH_2;
	int n = HEIGHT_2;
	int w, z, y;
	if( horizon_icon.get() == 1 ) {   // 1 = large icon
		w = DISPLAY_W/4 + DISPLAY_W/8;   // half wingspan
		z = w>>2;           // half-width of horizontal tail
		y = n - 3 - z;      // Y-position of top of vertical tail
	} else {                          // 0 = small icon
		w = DISPLAY_W/4;
		z = 0;              // no horizontal tail
		y = n - 3 - DISPLAY_W/10;
	}
	// skip redraw if likely no overlap with redrawn sky & ground parts
	// overlaps if k above bottom (largest y) & g below top (smallest y)
	HznSetColor( hzn_color );
	xSemaphoreTake(spiMutex, portMAX_DELAY );
	if ( draw || ((k0 < n+3 || kmid < n+3) && (g0 > n-3 || gmid > n-3)) )
		ucg->drawTetragon( m-w,n+3, m-w,n-3, m-10,n-3, m-10,n+3 );  // left wing
	if ( draw || ((k1 < n+3 || kmid < n+3) && (g1 > n-3 || gmid > n-3)) )
		ucg->drawTetragon( m+10,n+3, m+10,n-3, m+w,n-3, m+w,n+3 );  // right wing
	if ( draw || pitch_ticks_drawn || ( kmid < n && gmid+3 > y ))
		ucg->drawTetragon( m-3,n-9, m-3,y, m+3,y, m+3,n-9 );        // v tail
	if ( draw || pitch_ticks_drawn || (z && (k0<y || k1<y) && (g0>y-6 || g1>y-6)))
		ucg->drawTetragon( m-z,y, m-z,y-6, m+z,y-6, m+z,y );        // h tail
	if ( draw || ( kmid < n+15 && gmid >= n-15 ) || pitch_ticks_drawn) {
		ucg->drawCircle( m, n,  9, UCG_DRAW_ALL );
		ucg->drawCircle( m, n, 10, UCG_DRAW_ALL );                  // fuselage
	}
	xSemaphoreGive(spiMutex);
}

bool calc_horizon( int sb, int cb, int& x0, int& y0, int& x1, int& y1 ) {
	// prepare to draw new horizon line
	int m = WIDTH_2;
	int n = WIDTH_2;  // this function lives in a square world
	if ( cb <= 0 )   cb = 256;   // safety valve
	// Vertical position of horizon at center of width after offsetting for pitch:
	// Divide by cos(bank) since "pitch" is defined as perpendicular to ground
	int pc = (pitchpixels<<8) / cb;
	int y = n + pc;
	// move ends of horizon up or down by tan(bank)
	int h = (WIDTH_2 * sb) / cb;
	bool wrongpitch = false;
	if (horizon_plimit.get()) {
		// ensure some sky and some ground even with offset
		int abs_h = abs(h);
		if (y-abs_h > (WIDTH_-15)) {     // y > WIDTH_-15 + abs(h)
			y = (WIDTH_-15) + abs_h;
			wrongpitch = true;
		} else
		if (y+abs_h < 15) {              // y < 15 - abs(h)
			y = 15 - abs_h;
			wrongpitch = true;
		}
	}
	y0 = y + h;      // left end of horizon line, lower if h>0 i.e. bank>0
	y1 = y - h;      // right end of horizon line
	x0 = 0;
	x1 = WIDTH_;
	// In steep bank y0,y1 may be outside the display square Y range,
	// Compute where horizon touches top and/or bottom edges of square instead
	// if (h=0) h=m;  - in any case don't crash on a division-by-0
	if (y0 < 0) {
		x0 = m - (m*(m+pc))/(h?-h:m);   // h<0
		//ESP_LOGI(FNAME,"top-left y0=%d -> x0=%d, pc=%d h=%d", y0, x0, pc, h );
		y0 = 0;
	} else if (y0 > WIDTH_) {
		x0 = m - (m*(m-pc))/(h?h:m);    // h>0
		//ESP_LOGI(FNAME,"bot-left y0=%d -> x0=%d, pc=%d h=%d", y0, x0, pc, h );
		y0 = WIDTH_;
	}
	if (y1 < 0) {
		x1 = m + (m*(m+pc))/(h?h:m);    // h>0
		//ESP_LOGI(FNAME,"top-right y1=%d -> x1=%d, pc=%d h=%d", y1, x1, pc, h );
		y1 = 0;
	} else if (y1 > WIDTH_) {
		x1 = m + (m*(m-pc))/(h?-h:m);   // h<0
		//ESP_LOGI(FNAME,"bot-right y1=%d -> x1=%d, pc=%d h=%d", y1, x1, pc, h );
		y1 = WIDTH_;
	}
	if (x0 < 0)  x0 = 0;
	if (x1 > WIDTH_)  x1 = WIDTH_;
	if (horizon_plimit.get()) {
		// ensure still some sky and some ground, if only at a corner
		if (x0 > WIDTH_ - 25)  x0 = WIDTH_ - 25;
		if (x1 < 25)  x1 = 25;
	} else {
		if (x0 > WIDTH_)  x0 = WIDTH_;
		if (x1 < 0)  x1 = 0;
	}
	return wrongpitch;  // not affected by corner issues just above
}

// erase the thin line between ground and sky
//  - need elaborate procedure to avoid leaving "debris"
void IpsDisplay::erase_horizon_line( bool up0, bool up1 ) {
	int which = (horizon_line.get()==2? 2 : 0);
	if (up0 && up1) {             // the whole line moved up
		HznSetColor( gnd_color );
		double_line( old_x0,old_y0, old_x1,old_y1, which );
	} else if (!up0 && !up1) {    // the whole line moved down
		HznSetColor( sky_color );
		double_line( old_x0,old_y0, old_x1,old_y1, which );
	} else {
		// Old and new horizon lines crossed:
		// use different colors for the two parts of the line.
		// Try and compute the crossing point (xc,yc).
		// This simplified method is not exact when one end of the line
		// is on a side and the other is on a top or bottom - it is never
		// exact anyway: small changes in x and/or y and integer math.
		int sum = abs(hzn_y0-old_y0) + abs(hzn_x0-old_x0);
		int ratio = (sum << 8);
		sum += abs(hzn_y1-old_y1) + abs(hzn_x1-old_x1);
		// sum should always be > 0 since something moved, but guard anyway:
		ratio /= (sum?sum:256);
		int xc, yc;
		// Calling drawLine() for part of the line misses some pixels.
		// Instead, paint the whole line twice, once in each color,
		// but use clip range to limit where the paint sticks.
		int xdiff = abs(old_x1-old_x0);
		int ydiff = abs(old_y1-old_y0);
		if (xdiff > ydiff) {         // line more horizontal than vertical
			xc = old_x0 + (((old_x1-old_x0)*ratio) >> 8);
			ucg->setClipRange( 0, AHRS_TOP, xc, DISPLAY_W );              // left part
			if (up0)
				HznSetColor( gnd_color );
			else
				HznSetColor( sky_color );
		} else {             // more vertical
			yc = old_y0 + (((old_y1-old_y0)*ratio) >> 8);
			ucg->setClipRange( 0, AHRS_TOP, DISPLAY_W, yc - AHRS_TOP );   // top part
			if ( (old_y0 > old_y1)? up1 : up0 )
				HznSetColor( gnd_color );
			else
				HznSetColor( sky_color );
		}
		double_line( old_x0,old_y0, old_x1,old_y1, which );
		if (xdiff > ydiff) {
			ucg->setClipRange( xc, AHRS_TOP, DISPLAY_W - xc, DISPLAY_W ); // right part
			if (up1)
				HznSetColor( gnd_color );
			else
				HznSetColor( sky_color );
		} else {
			ucg->setClipRange( 0, yc, DISPLAY_W, (AHRS_BOT+1) - yc );     // bottom part
			if ( (old_y0 > old_y1)? up0 : up1)
				HznSetColor( gnd_color );
			else
				HznSetColor( sky_color );
		}
		double_line( old_x0,old_y0, old_x1,old_y1, which );
		ucg->undoClipRange();
	}
}

void IpsDisplay::drawHorizon( float p, float b, float yaw )   // ( pitch, roll, yaw )
{
//	if( _menu || !gflags.ahrsKeyValid )
//		return;

	tick++;
	if ( (tick&0x03) != 0 )    // redraw horizon every 80 ms
		return;

	static int horizon_done = 0;

	if( !(screens_init & INIT_DISPLAY_HORIZON) ){
		clear();
		horizon_done = 0;
		screens_init |= INIT_DISPLAY_HORIZON;
		int hc = horizon_colors.get();
		sky_color = &skycolor[hc];
		gnd_color = &gndcolor[hc];
		hzn_color = &hzncolor[hc];
		return;    // yield for now, will draw sky & ground on the next call
	}

	if (horizon_done == 0) {

		// paint AHRS square, half dark sky and half dark ground:
		xSemaphoreTake(spiMutex, portMAX_DELAY );
		HznSetColor( sky_color );
		ucg->drawTetragon( 0,AHRS_TOP, 0,HEIGHT_2, WIDTH_,HEIGHT_2, WIDTH_,AHRS_TOP );
		xSemaphoreGive(spiMutex);
		vTaskDelay(2);
		xSemaphoreTake(spiMutex, portMAX_DELAY );
		HznSetColor( gnd_color );
		ucg->drawTetragon( 0,AHRS_BOT, 0,HEIGHT_2, WIDTH_,HEIGHT_2, WIDTH_,AHRS_BOT );
		xSemaphoreGive(spiMutex);
		// if enabled, add thin brighter line at horizon:
		if (horizon_line.get()) {
			HznSetColor( hzn_color );
			double_line( 0,HEIGHT_2, WIDTH_,HEIGHT_2, horizon_line.get()==1? 0 : 2 );
		}
		//vTaskDelay(2);
		if (hzn_prange != horizon_prange.get()) {
			hzn_prange = horizon_prange.get();
			pitch_offset = 0;
			p5  = pitch2pixels( D2R(5) );
			p10 = pitch2pixels( D2R(10) );
			p15 = pitch2pixels( D2R(15) );
			p20 = pitch2pixels( D2R(20) );
			p25 = pitch2pixels( D2R(25) );
			p30 = pitch2pixels( D2R(30) );
			p35 = pitch2pixels( D2R(35) );
			p40 = pitch2pixels( D2R(40) );
			p45 = pitch2pixels( D2R(45) );
			p50 = pitch2pixels( D2R(50) );
			p60 = pitch2pixels( D2R(60) );
			p70 = pitch2pixels( D2R(70) );
			p90 = pitch2pixels( D2R(89) );
		}
		hzn_x0 = 0;
		hzn_x1 = WIDTH_;
		hzn_y0 = HEIGHT_2;
		hzn_y1 = HEIGHT_2;
		old_x0 = 0;
		old_x1 = WIDTH_;
		old_y0 = HEIGHT_2;
		old_y1 = HEIGHT_2;
		pitchpixels = 0;
		oldpitchpixels = 0;
		horizon_done = 1;

		airplane_icon( true );

		if( !gflags.ahrsKeyValid ) {
			ucg->setFont(ucg_font_fub14_hn, true);
			ucg->setPrintPos(35,310);
			ucg->setColor( COLOR_BRED );
			ucg->print( " AHRS disabled " );
			return;
		}

		// yield for now, draw real horizon etc on next call
		return;
	}

	if( !gflags.ahrsKeyValid )    // static demo does not change
		return;

    // pitch sign is reversed in the latest sensor.cpp code
    p = -p;

	// periodically clean up and print some numbers

	if ( (tick&0x1F) == 0 ) {

		// completely redraw the airplane icon periodically
		airplane_icon( true );

		// display heading (or course), if possible
		static int heading_old = -1;
		int heading = 999;
		int headingtype = 0;
		if( compass_enable.get() != CS_DISABLE ) {
			heading = static_cast<int>(rintf(mag_hdt.get()));
			headingtype = 1;
		} else if( Flarm::gpsStatus() ) {
			heading = static_cast<int>(rintf(Flarm::getGndCourse()));
			headingtype = 2;
		}
		ucg->setColor( COLOR_WHITE );
		if( heading != heading_old ){
			ucg->setFont(ucg_font_fub20_hr, true);
			ucg->setPrintPos(60,318);
			if( headingtype == 0 ) {
				ucg->print( "      ---°  " );
			} else {
				if( heading <= 0 )
					heading += 360;
				else if( heading > 360 )
					heading -= 360;
				if (headingtype == 1)
					ucg->printf( " hdg  %03d°  ", heading );
				else
					ucg->printf( " crs  %03d°  ", heading );
				heading_old = heading;
			}
		}

		// also print pitch and bank as numbers at top
		if ( testmode.get() && horizon_nums.get() ) {
			ucg->setFont(ucg_font_fub14_hn, true);
			// ucg->setColor( COLOR_WHITE );
			ucg->setPrintPos(10,38);
			ucg->printf( "b %2.0f  ", 57.296 * b );
			ucg->setPrintPos(170,38);
			ucg->printf( "p %4.1f  ", 57.296 * p );
		}

		// show a warning about pitch offset being applied
//		ucg->setFont(ucg_font_fub14_hn, true);
		ucg->setColor( COLOR_BRED );
		ucg->setPrintPos(215,315);
		if (pitch_offset > 0)
			ucg->print( "^" );
		else if (pitch_offset < 0)
			ucg->print( "v" );
		else  // == 0
			ucg->print( "  " );

		return;     // skip drawing horizon & ticks until next call
	}

	// force redraw horizon if pitch offset has been changed
	if ( horizon_offset.get() != pitch_offset ) {
		pitch_offset = horizon_offset.get();
		horizon_done = 1;
	}

	// if < 1 pixel change, wait until further change before processing
	// (about 1/2 degree bank or 1/4 degree pitch)
	static float old_p = 0;
	static float old_b = 0;
	float b_ = limited_bank(b);
	float p_ = limited_pitch(p);
	if ( horizon_done == 2 ) {
		if ( fabs(old_p-p_) < 0.004 && fabs(old_b-b_) < 0.009 )
			return;
	}
	horizon_done = 2;
	old_p = p_;
	old_b = b_;
	bool drawbticks = (horizon_bticks.get() != 0);
	bool drawpticks = (horizon_pticks.get() != 0);
	if ( p_ != p )       // limited_pitch() reduced it
		drawpticks = false;

	// prepare to draw new horizon line
	cos_bank = (int)(256 * approxcos(b_));
	sin_bank = (int)(256 * approxsin(b_));
	pitchpixels = pitch2pixels(p_);      // includes the offset

	int x0, y0, x1, y1;
	if ( b_ > 1.13 ) {
		// over about 65 degrees bank:
		// use calc_horizon in a world rotated 90 degrees:
		calc_horizon( -cos_bank, sin_bank, x0, y0, x1, y1 );
		hzn_x0 = y0;
		hzn_y0 = AHRS_BOT - x0;
		hzn_x1 = y1;
		hzn_y1 = AHRS_BOT - x1;
		//drawbticks = false;
		drawpticks = false;
	} else if ( b_ < -1.13 ) {
		// use calc_horizon in a world rotated 90 degrees the other way:
		calc_horizon( cos_bank, -sin_bank, x0, y0, x1, y1 );
		hzn_x0 = WIDTH_ - y0;
		hzn_y0 = AHRS_TOP + x0;
		hzn_x1 = WIDTH_ - y1;
		hzn_y1 = AHRS_TOP + x1;
		//drawbticks = false;
		drawpticks = false;
	} else {
		// under about 65 degrees, stay in the normal world:
		calc_horizon( sin_bank, cos_bank, x0, y0, x1, y1 );
		hzn_x0 = x0;
		hzn_y0 = AHRS_TOP + y0;
		hzn_x1 = x1;
		hzn_y1 = AHRS_TOP + y1;
	}

	// redraw only if change will be visible in pixel resolution
	bool change = (hzn_x0!=old_x0 || hzn_x1!=old_x1 || hzn_y0!=old_y0 || hzn_y1!=old_y1);

	int yy0 = AHRS_TOP;
	int yy1 = AHRS_TOP;
	bool up0 = (hzn_y0 < old_y0);
	bool up1 = (hzn_y1 < old_y1);
	int x2 = old_x1;
	int y2 = old_y1;

	if (change) {

	// find out in which direction each end of the horizon line moved
	// also set up yy0,yy1 for correct choice of top or bottom
	//   (only used if x0 != old_x0 or x1 != old_x1)
	if (hzn_y0==AHRS_TOP || old_y0==AHRS_TOP) {
		up0 = (up0 || hzn_x0 > old_x0);
		//ESP_LOGI(FNAME,"horiz passed top left corner, up0=%d", up0);
	} else
	if (hzn_y0==AHRS_BOT || old_y0==AHRS_BOT) {
		yy0 = AHRS_BOT;
		up0 = (up0 || hzn_x0 < old_x0);
		//ESP_LOGI(FNAME,"horiz passed bot left corner, up1=%d", up0);
	}
	if (hzn_y1==AHRS_TOP || old_y1==AHRS_TOP) {
		up1 = (up1 || hzn_x1 < old_x1);
		//ESP_LOGI(FNAME,"horiz passed top right corner, up1=%d", up1);
	} else
	if (hzn_y1==AHRS_BOT || old_y1==AHRS_BOT) {
		yy1 = AHRS_BOT;
		up1 = (up1 || hzn_x1 > old_x1);
		//ESP_LOGI(FNAME,"horiz passed bot right corner, up1=%d", up1);
	}

	if (up0 != up1) {  // old and new horizon lines crossed
		x2 = hzn_x1;
		y2 = hzn_y1;
	}

	// first erase the thin line between ground and sky
	if (horizon_line.get())
		erase_horizon_line( up0, up1 );

	}  // end if(change)

	// erase now any existing bank tick marks that cross the new horizon
	// (if drawbticks==false will erase all bank ticks now & return false)
	bank_ticks_drawn = banks_ticks( bank_ticks_drawn, drawbticks, true );

	if (change) {

	// Finally, redraw sky and ground for the new horizon position.
	// Paint only narrow triangles as needed to cover the change.
	// This algorithm may paint up to about double the actual changed area
	// when the lines cross, but is simpler than computing the crossing point
	// Re-arranged to always draw sky first, ground second.
	// The number of triangles actually drawn is never more than 4,
	// and only 2 if horizon hasn't crossed corners of display.
	//ESP_LOGI(FNAME,"about to draw triangles");

	xSemaphoreTake(spiMutex, portMAX_DELAY );
	HznSetColor( sky_color );
	if ( !up0 ) {
		if ( hzn_y0 != old_y0 )
			ucg->drawTriangle( x2,y2, 0,hzn_y0, 0,old_y0 );
		if ( hzn_x0 != old_x0 )
			ucg->drawTriangle( x2,y2, hzn_x0,yy0, old_x0,yy0 );
	}
	if ( !up1 ) {
		if ( hzn_y1 != old_y1 )
			ucg->drawTriangle( hzn_x0,hzn_y0, WIDTH_,hzn_y1, WIDTH_,old_y1 );
		if ( hzn_x1 != old_x1 )
			ucg->drawTriangle( hzn_x0,hzn_y0, hzn_x1,yy1, old_x1,yy1 );
	}
	HznSetColor( gnd_color );
	if ( up0 ) {
		if ( hzn_y0 != old_y0 )
			ucg->drawTriangle( x2,y2, 0,hzn_y0, 0,old_y0 );
		if ( hzn_x0 != old_x0 )
			ucg->drawTriangle( x2,y2, hzn_x0,yy0, old_x0,yy0 );
	}
	if ( up1 ) {
		if ( hzn_y1 != old_y1 )
			ucg->drawTriangle( hzn_x0,hzn_y0, WIDTH_,hzn_y1, WIDTH_,old_y1 );
		if ( hzn_x1 != old_x1 )
			ucg->drawTriangle( hzn_x0,hzn_y0, hzn_x1,yy1, old_x1,yy1 );
	}
	xSemaphoreGive(spiMutex);

	}  // end if(change)

	// late-erase bank ticks that do not cross the new horizon, and draw
	// new ticks immediately after erasing old ones, to minimize flicker:
	bank_ticks_drawn = banks_ticks( bank_ticks_drawn, drawbticks, false );

	if ( pitch_ticks_drawn ) {
		// erase old pitch tickmarks - they never cross the horizon
		// >>> but old ticks might cross the new horizon if bank changed abruptly
		pitch_ticks( false );
		pitch_ticks_drawn = false;
	}
	if ( drawpticks )
		pitch_ticks_drawn = pitch_ticks( true );

	if (change) {

	// add thin more obvious line at horizon
	if (horizon_line.get()) {
		HznSetColor( hzn_color );
		double_line( hzn_x0,hzn_y0, hzn_x1,hzn_y1, horizon_line.get()==1? 0 : 2 );
	}

	// redraw parts of airplane icon, only as needed
	airplane_icon( false );

	}  // end if(change)

	// store current values for future reference
	old_x0 = hzn_x0;
	old_x1 = hzn_x1;
	old_y0 = hzn_y0;
	old_y1 = hzn_y1;
	old_sin_bank = sin_bank;
	old_cos_bank = cos_bank;
	oldpitchpixels = pitchpixels;
}
