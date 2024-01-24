/*
 * SetupMenu.cpp
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#include "SetupMenu.h"
#include "IpsDisplay.h"
#include "ESPAudio.h"
#include "BMPVario.h"
#include "S2F.h"
#include "Version.h"
#include "Polars.h"
#include "Cipher.h"
#include "Units.h"
#include "Switch.h"
#include "Flap.h"
#include "SetupMenuSelect.h"
#include "SetupMenuValFloat.h"
#include "SetupMenuChar.h"
#include "DisplayDeviations.h"
#include "ShowCompassSettings.h"
#include "ShowCirclingWind.h"
#include "ShowStraightWind.h"
#include "MenuEntry.h"
#include "Compass.h"
#include "CompassMenu.h"
#include "esp_wifi.h"
#include "Flarm.h"
#include "WifiClient.h"
#include "Blackboard.h"
#include "DataMonitor.h"

#include <inttypes.h>
#include <iterator>
#include <algorithm>
#include <logdef.h>
#include <sensor.h>
#include <cstring>
#include <string>
#include "SetupNG.h"
#include "quaternion.h"

SetupMenuSelect * audio_range_sm = 0;
SetupMenuSelect * mpu = 0;

SetupMenuSelect *show_mode_menu = 0;
void update_show_mode_menu() {
	if (show_mode_menu)
		show_mode_menu->updateEntry( mode_shown, 0 );
}

SetupMenuValFloat *volume_menu = 0;
void update_volume_menu_max() {
	if (volume_menu)
		volume_menu->setMax( max_volume.get() );
}

// Menu for flap setup

float elev_step = 1;
static uint8_t screen_mask_len = 1;

bool SetupMenu::focus = false;

int gload_reset( SetupMenuSelect * p ){
	gload_pos_max.set(0);
	gload_neg_max.set(0);
	airspeed_max.set(0);
	return 0;
}

int compass_ena( SetupMenuSelect * p ){
	return 0;
}

void init_screens(){
	uint32_t scr = menu_screens.get();
	screen_gmeter.set( (scr >> SCREEN_GMETER) & 1);
	// 	screen_centeraid.set( (scr >> SCREEN_THERMAL_ASSISTANT) & 1);
	screen_horizon.set( (scr >> SCREEN_HORIZON) & 1);
	screen_mask_len = 1; // default vario
	while( scr ){
		scr = scr >> 1;
		screen_mask_len++;
	}
	ESP_LOGI(FNAME,"screens mask len: %d, screens: %d", screen_mask_len, menu_screens.get() );
}

int vario_setup(SetupMenuValFloat * p)
{
	bmpVario.configChange();
	return 0;
}

int audio_setup_s(SetupMenuSelect * p)
{
	Audio::setup();
	return 0;
}

int audio_setup_f(SetupMenuValFloat * p)
{
	Audio::setup();
	return 0;
}

int speedcal_change(SetupMenuValFloat * p)
{
	if( asSensor)
		asSensor->changeConfig();
	return 0;
}

gpio_num_t SetupMenu::getGearWarningIO(){
	gpio_num_t io = GPIO_NUM_0;
#if !defined(NOSENSORS)
	if( gear_warning.get() == GW_FLAP_SENSOR || gear_warning.get() == GW_FLAP_SENSOR_INV ){
		io = GPIO_NUM_34;
	}
	else if( gear_warning.get() == GW_S2_RS232_RX || gear_warning.get() == GW_S2_RS232_RX_INV ){
		io = GPIO_NUM_18;
	}
#endif
	return io;
}

void initGearWarning(){
	gpio_num_t io = SetupMenu::getGearWarningIO();
	if( io != GPIO_NUM_0 ){
		gpio_reset_pin( io );
		gpio_set_direction(io, GPIO_MODE_INPUT);
		gpio_set_pull_mode(io, GPIO_PULLUP_ONLY);
		gpio_pullup_en( io );
	}
	ESP_LOGI(FNAME,"initGearWarning: IO: %d", io );
}

int config_gear_warning( SetupMenuSelect * p ){
	initGearWarning();
	return 0;
}

int upd_screens( SetupMenuSelect * p ){
	uint32_t screens =
			( (uint32_t)screen_gmeter.get() << (SCREEN_GMETER)  |
			//		( (uint32_t)screen_centeraid.get() << (SCREEN_THERMAL_ASSISTANT) ) |
			( (uint32_t)screen_horizon.get() << (SCREEN_HORIZON) )
			);
	menu_screens.set( screens );
	// init_screens();
	return 0;
}

int do_display_test(SetupMenuSelect * p){
	if( display_test.get() ){
		xSemaphoreTake(spiMutex,portMAX_DELAY );
		p->ucg->setColor( 0,0,0 );
		p->ucg->drawBox( 0, 0, 240, 320 );
		xSemaphoreGive(spiMutex );
		while( !p->readSwitch() ){
			delay(100);
			ESP_LOGI(FNAME,"Wait for key press");
		}
		xSemaphoreTake(spiMutex,portMAX_DELAY );
		p->ucg->setColor( 255,255,255 );
		p->ucg->drawBox( 0, 0, 240, 320 );
		xSemaphoreGive(spiMutex );
		while( !p->readSwitch() ){
			delay(100);
			ESP_LOGI(FNAME,"Wait for key press");
		}
		esp_restart();
	}
	return 0;
}

int update_s2f_speed(SetupMenuValFloat * p)
{
	Switch::setCruiseSpeed( Units::Airspeed2Kmh( s2f_speed.get() ) );
	return 0;
}

static char rentry0[32];
static char rentry1[32];
static char rentry2[32];

int update_rentry(SetupMenuValFloat * p)
{
	// ESP_LOGI(FNAME,"update_rentry() vu:%s ar:%p", Units::VarioUnit(), audio_range_sm );
	sprintf( rentry0, "Fixed (5  %s)", Units::VarioUnit() );
	sprintf( rentry1, "Fixed (10 %s)", Units::VarioUnit() );
	sprintf( rentry2, "Variable (%d %s)", (int)(range.get()), Units::VarioUnit() );
	return 0;
}

int update_rentrys(SetupMenuSelect * p){
	update_rentry(0);
	return 0;
}

int update_wifi_power(SetupMenuValFloat * p)
{
	ESP_ERROR_CHECK(esp_wifi_set_max_tx_power( int(wifi_max_power.get()*80.0/100.0) ));
	return 0;
}

int data_mon( SetupMenuSelect * p ){
	ESP_LOGI(FNAME,"data_mon( %d ) ", data_monitor.get() );
	if( data_monitor.get() != MON_OFF ){
		DM.start(p);
	}
	return 0;
}

int update_id( SetupMenuChar * p){
	const char *c = p->getEntry();
	ESP_LOGI(FNAME,"New Letter %c Index: %d", *c, p->getCharIndex() );
	char id[10] = { 0 };
	strcpy( id, custom_wireless_id.get().id );
	id[p->getCharIndex()] = *c;
	ESP_LOGI(FNAME,"New ID %s", id );
	custom_wireless_id.set( id );
	return 0;
}

int add_key( SetupMenuSelect * p )
{
	ESP_LOGI(FNAME,"add_key( %d ) ", p->getSelect() );
	if( Cipher::checkKeyAHRS() ){
		if( !mpu->existsEntry( "Enable") )
			mpu->addEntry( "Enable");
	}
	else{
		if( mpu->existsEntry( "Enable") )
			mpu->delEntry( "Enable");
	}
	return 0;
}

int qnh_adj( SetupMenuValFloat * p )
{
	ESP_LOGI(FNAME,"qnh_adj %f", QNH.get() );
	float alt=0;
	if( Flarm::validExtAlt() && alt_select.get() == AS_EXTERNAL ){
		alt = alt_external + ( QNH.get() - 1013.25)*8.2296;  // correct altitude according to ISA model = 27ft / hPa
	}
	else{
		int samples = 0;
		for( int i=0; i<6; i++ ) {
			bool ok;
			float a = p->_bmp->readAltitude( QNH.get(), ok );
			if( ok ){  // only consider correct readouts
				alt += a;
				samples++;
			}
			delay(10);
		}
		alt = alt/(float)samples;
	}
	ESP_LOGI(FNAME,"Setup BA alt=%f QNH=%f hPa", alt, QNH.get()  );
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	p->ucg->setFont(ucg_font_fub25_hr, true);
	float altp;
	const char *u = "m";
	if( alt_unit.get() == 0 ){ // m
		altp = alt;
	}
	else {
		u = "ft";
		altp = Units::meters2feet( alt );
	}
	p->ucg->setPrintPos(1,120);
	p->ucg->printf("%5d %s  ", (int)(altp+0.5), u );

	p->ucg->setFont(ucg_font_ncenR14_hr);
	xSemaphoreGive(spiMutex );
	return 0;
}

// Battery Voltage Meter Calibration
int factv_adj( SetupMenuValFloat * p )
{
	ESP_LOGI(FNAME,"factv_adj");
	float bat = p->_adc->get(true);
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	p->ucg->setPrintPos(1,100);
	p->ucg->printf("%0.2f Volt", bat);
	xSemaphoreGive(spiMutex );
	return 0;
}

int master_xcv_lock( SetupMenuSelect * p ){
	ESP_LOGI(FNAME,"master_xcv_lock");
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	p->ucg->setPrintPos(1,130);
	int mxcv = WifiClient::getScannedMasterXCV();
	p->ucg->printf("Scanned: XCVario-%d", mxcv );
	xSemaphoreGive(spiMutex );
	if( master_xcvario_lock.get() == 1 )
		master_xcvario.set( mxcv );
	return 0;
}

int polar_select( SetupMenuSelect * p )
{
	int index = Polars::getPolar( p->getSelect() ).index;
	ESP_LOGI(FNAME,"glider-index %d, glider num %d", index, p->getSelect() );
	glider_type_index.set( index );
	return 0;
}

void print_fb( SetupMenuValFloat * p, float wingload ){
	p->ucg->setFont(ucg_font_fub25_hr, true);
	p->ucg->setPrintPos(1,110);
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	p->ucg->printf("%0.2f kg/m2  ", wingload );
	xSemaphoreGive(spiMutex );
	p->ucg->setFont(ucg_font_ncenR14_hr);
}

int water_adj( SetupMenuValFloat * p )
{
	if( (ballast_kg.get() > polar_max_ballast.get()) || (ballast_kg.get() < 0) ){
		ballast_kg.set(0);
		ballast_kg.commit();
		delay(1000);
	}
	p->setMax(polar_max_ballast.get());
	float wingload = (ballast_kg.get() + crew_weight.get()+ empty_weight.get()) / polar_wingarea.get();
	ESP_LOGI(FNAME,"water_adj() wingload:%.1f empty: %.1f cw:%.1f water:%.1f", wingload, empty_weight.get(), crew_weight.get(), ballast_kg.get() );
	print_fb( p, wingload );
	return 0;
}

int empty_weight_adj( SetupMenuValFloat * p )
{
	float wingload = (ballast_kg.get() + crew_weight.get()+ empty_weight.get()) / polar_wingarea.get();
	print_fb( p, wingload );
	return 0;
}


int crew_weight_adj( SetupMenuValFloat * p )
{
	float wingload = (ballast_kg.get() + empty_weight.get()+ crew_weight.get()) / polar_wingarea.get();
	print_fb( p, wingload );
	return 0;
}

int bug_adj( SetupMenuValFloat * p ){
	return 0;
}

int vol_adj( SetupMenuValFloat * p ){
	// do nothing, actually - change_volume() already called Audio::setVolume()
	return 0;
}

int cur_vol_dflt( SetupMenuSelect *p ){
	if( p->getSelect() != 0 )  // "set"
		default_volume.set( audio_volume.get() );
	p->setSelect( 0 );   // return to "cancel"
	return 0;
}

/**
 * C-Wrappers function to compass menu handlers.
 */
static int compassDeviationAction( SetupMenuSelect *p )
{
	if( p->getSelect() == 0 ){
		CompassMenu::deviationAction( p );
	}
	return 0;
}

static int compassResetDeviationAction( SetupMenuSelect *p )
{
	return CompassMenu::resetDeviationAction( p );
}

static int compassDeclinationAction( SetupMenuValFloat *p )
{
	return CompassMenu::declinationAction( p );
}

static int windResetAction( SetupMenuSelect *p )
{
	if( p->getSelect() == 1 )
	{
		// Reset is selected, set default values
		wind_as_min.set( 25 );
	}
	return 0;
}

static int eval_chop( SetupMenuSelect *p )
{
	Audio::evaluateChopping();
	return 0;
}

static int compassSensorCalibrateAction( SetupMenuSelect *p )
{
	ESP_LOGI(FNAME,"compassSensorCalibrateAction()");
	if( p->getSelect() != 0 ){ // Start, Show
		CompassMenu::sensorCalibrationAction( p );
	}
	p->setSelect( 0 );
	return 0;
}

SetupMenu::SetupMenu() : MenuEntry() {
	highlight = -1;
	_parent = 0;
	helptext = 0;
}

SetupMenu::SetupMenu( const char *title ) : MenuEntry() {
	// ESP_LOGI(FNAME,"SetupMenu::SetupMenu( %s ) ", title );
	attach(this);
	_title = title;
	highlight = -1;
}

SetupMenu::~SetupMenu()
{
	// ESP_LOGI(FNAME,"del SetupMenu( %s ) ", _title );
	detach(this);
}

void SetupMenu::begin( IpsDisplay* display, PressureSensor * bmp, AnalogInput *adc ){
	ESP_LOGI(FNAME,"SetupMenu() begin");
	_bmp = bmp;
	_display = display;
	ucg = display->getDisplay();
	_adc = adc;
	setup();
	audio_volume.set( default_volume.get() );
	// init_routing();   now done from SetupNG update_NGs()
	init_screens();
	initGearWarning();
}

void SetupMenu::catchFocus( bool activate ){
	focus = activate;
}

void SetupMenu::display( int mode ){
	if( (selected != this) || !gflags.inSetup || focus )
		return;
#if defined(SUNTON28)
	ESP_LOGI(FNAME,"SetupMenu::display %s", _title );
#endif
	// ESP_LOGI(FNAME,"SetupMenu display( %s)", _title );
	xSemaphoreTake(display_mutex,portMAX_DELAY);
	clear();
#if defined(SUNTON28)
	delay(100);
#endif
	int y=25;
	// ESP_LOGI(FNAME,"Title: %s y=%d child size:%d", selected->_title,y, _childs.size()  );
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	ucg->setFont(ucg_font_ncenR14_hr);
	ucg->setPrintPos(1,y);
	ucg->setFontPosBottom();
	ucg->printf("<< %s",selected->_title);
	ucg->drawFrame( 1,(selected->highlight+1)*25+3,238,25 );
	for (int i=0; i<_childs.size(); i++ ){
		MenuEntry * child = _childs[i];
		ucg->setPrintPos(1,(i+1)*25+25);
		ucg->setColor( COLOR_HEADER_LIGHT );
		ucg->printf("%s",child->_title);
		// ESP_LOGI(FNAME,"Child Title: %s", child->_title );
		if( child->value() ){
			int fl=ucg->getStrWidth( child->_title );
			ucg->setPrintPos(1+fl,(i+1)*25+25);
			ucg->printf(": ");
			ucg->setPrintPos(1+fl+ucg->getStrWidth( ":" ),(i+1)*25+25);
			ucg->setColor( COLOR_WHITE );
			ucg->printf(" %s",child->value());
		}
		ucg->setColor( COLOR_WHITE );
		// ESP_LOGI(FNAME,"Child: %s y=%d",child->_title ,y );
	}
	y+=170;
	xSemaphoreGive(spiMutex );
	showhelp( y );
	xSemaphoreGive(display_mutex);
}

void SetupMenu::down(int count){
	if( selected == this && !gflags.inSetup ) {
		// ESP_LOGI(FNAME,"root: down");
		if( gflags.horizon ) {
			float offset = horizon_offset.get();
			if (offset > 0 && offset <= 1) {
				offset = 0;
			} else {
				offset -= 0.5*count;
				if( offset < -10.0 )
					offset = -10.0;
			}
			horizon_offset.set( offset );
			//ESP_LOGI(FNAME,"down: horizon_offset pct to %f", offset );
		}
		else
		if( rot_default.get() == 1) {  // MC Value
			float mc = MC.get();
			float step = Units::Vario2ms( 0.1 );
			mc -= step*count;
			if( mc < 0.0 )
				mc = 0.0;
			MC.set( mc );
		}
		else{  // Volume
#if defined(SUNTON28)
			count = 1;
#endif
			float vol = audio_volume.get();
			if( vol<3.0 )
				vol=3.0;
			for( int i=0; i<count; i++ )
				vol = vol * 1.17;
			if( vol > max_volume.get() )
				vol = max_volume.get();
			audio_volume.set( vol );
			//ESP_LOGI(FNAME,"down: volume pct up to %f", vol );
		}
	}
	if( (selected != this) || !gflags.inSetup )
		return;
	// ESP_LOGI(FNAME,"down %d %d", highlight, _childs.size() );
	if( focus )
		return;
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	ucg->setColor(COLOR_BLACK);
	ucg->drawFrame( 1,(highlight+1)*25+3,238,25 );
	ucg->setColor(COLOR_WHITE);
	if( highlight  > -1 ){
		highlight --;
	}
	else
		highlight = (int)(_childs.size() -1 );
	ucg->drawFrame( 1,(highlight+1)*25+3,238,25 );
	xSemaphoreGive(spiMutex );
	pressed = true;
}

void SetupMenu::up(int count){
	if( selected == this && !gflags.inSetup ) {
		// ESP_LOGI(FNAME,"root: up");
		if( gflags.horizon ) {
			float offset = horizon_offset.get();
			if (offset < 0 && offset >= -1) {
				offset = 0;
			} else {
				offset += 0.5*count;
				if( offset > 10.0 )
					offset = 10.0;
			}
			horizon_offset.set( offset );
			//ESP_LOGI(FNAME,"up: horizon_offset pct to %f", offset );
		}
		else
		if(rot_default.get() == 1) {  // MC Value
			float mc = MC.get();
			// ESP_LOGI(FNAME,"MC up: %f count: %d", mc, count );
			float step = Units::Vario2ms( 0.1 );
			mc += step*count;
			if( mc > 9.9 )
				mc = 9.9;
			// ESP_LOGI(FNAME,"NEW MC: %f", mc );
			MC.set( mc );
		}
		else{  // Volume
#if defined(SUNTON28)
			count = 1;
#endif
			float vol = audio_volume.get();
			for( int i=0; i<count; i++ )
				vol = vol * 0.83;
			if( vol<3.0 )
				vol=0;
			audio_volume.set( vol );
			//ESP_LOGI(FNAME,"up: volume pct down to %f", vol );
		}
	}
	if( (selected != this) || !gflags.inSetup )
		return;
	// ESP_LOGI(FNAME,"SetupMenu::up %d %d", highlight, _childs.size() );
	if( focus )
		return;
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	ucg->setColor(COLOR_BLACK);
	ucg->drawFrame( 1,(highlight+1)*25+3,238,25 );
	ucg->setColor(COLOR_WHITE);
	if( highlight < (int)(_childs.size()-1) ){
		highlight ++;
	}
	else
		highlight = -1;
	ucg->drawFrame( 1,(highlight+1)*25+3,238,25 );
	pressed = true;
	xSemaphoreGive(spiMutex );
}

void SetupMenu::showMenu(){
	// ESP_LOGI(FNAME,"showMenu() p:%d h:%d parent:%x", pressed, highlight, (int)_parent );
	// default is not pressed, so just display, but we toogle pressed state at the end
	// so next time we either step up to parent,
	if( pressed )
	{
		if( highlight == -1 ) {
#if defined(SUNTON28)
			ESP_LOGI(FNAME,"SetupMenu to parent");
#endif
			// ESP_LOGI(FNAME,"SetupMenu to parent");
			if( _parent != 0 ){
				selected = _parent;
				selected->highlight = -1;
				selected->pressed = true;
				delete_subtree();
			}
		}
		else {
#if defined(SUNTON28)
			ESP_LOGI(FNAME,"SetupMenu to child");
#endif
			// ESP_LOGI(FNAME,"SetupMenu to child");
			if( (highlight >=0) && (highlight < (int)(_childs.size()) ) ){
				selected = _childs[highlight];
				selected->pressed = false;
			}
		}
	}
	if( (_parent == 0) && (highlight == -1) ) // entering setup menu root
	{
		if( !gflags.inSetup )
		{
			gflags.inSetup=true;
			// ESP_LOGI(FNAME,"Start Setup Menu");
			_display->doMenu(true);
			delay(200);  // fixme give display task time to finish drawing
		}
		else
		{
			// ESP_LOGI(FNAME,"End Setup Menu");
			screens_init = INIT_DISPLAY_NULL;
			_display->doMenu(false);
			if( selected->get_restart() )
				selected->restart();
			gflags.inSetup=false;
		}
	}
	// ESP_LOGI(FNAME,"end showMenu()");
}

static int screen_index = 0;

void SetupMenu::create_subtree(){
	if( !subtree_created && menu_create_ptr ){
		(menu_create_ptr)(this);
		subtree_created = true;
		// ESP_LOGI(FNAME,"create_subtree() %d", _childs.size() );
	}
}

void SetupMenu::delete_subtree(){
	// ESP_LOGI(FNAME,"delete_subtree() %d", _childs.size() );
	if( subtree_created && menu_create_ptr ){
		subtree_created = false;
		for (int i=0; i<_childs.size(); i++ ){
			delete _childs[i];
		}
		_childs.clear();
	}
}

void SetupMenu::press(){
	if( (selected != this) || focus )
		return;
	ESP_LOGI(FNAME,"press() active_srceen %d, pressed %d inSet %d  subtree_created: %d mptr: %p", active_screen, pressed, gflags.inSetup, subtree_created, menu_create_ptr );
	create_subtree();
	if( !gflags.inSetup ){
		active_screen = 0;
		while( !active_screen && (screen_index <= screen_mask_len) ){
			if( menu_screens.get() & (1 << screen_index) ){
				active_screen = screen_index;
				ESP_LOGI(FNAME,"New active_screen: %d", active_screen );
			}
			screen_index++;
		}
		if( screen_index > screen_mask_len ){
			ESP_LOGI(FNAME,"select vario screen");
			screen_index = 0;
			active_screen = 0; // fall back into default vario screen after optional screens
		}
	}
	if( !active_screen || gflags.inSetup ){
		// ESP_LOGI(FNAME,"press() gflags.inSetup");
		if( !menu_long_press.get() || gflags.inSetup )
			showMenu();
		if( pressed )
			pressed = false;
		else
			pressed = true;
	}
}

void SetupMenu::longPress(){
	if( (selected != this) )
		return;
	// ESP_LOGI(FNAME,"longPress()");
	ESP_LOGI(FNAME,"longPress() active_srceen %d, pressed %d inSet %d", active_screen, pressed, gflags.inSetup );
	if( menu_long_press.get() && !gflags.inSetup ){
		showMenu();
	}
	if( pressed ){
		pressed = false;
	}
	else{
		pressed = true;
#ifdef Quaternionen_Test
		Quaternion::quaternionen_test();
#endif
	}
}

void SetupMenu::escape(){
	if( gflags.inSetup ){
		ESP_LOGI(FNAME,"escape now Setup Menu");
		_display->clear();
		_display->doMenu(false);
		gflags.inSetup=false;
	}
}

void SetupMenu::vario_menu_create_meanclimb( MenuEntry *top ){
	SetupMenuValFloat * vccm = new SetupMenuValFloat( "Minimum climb", "",	0.0, 2.0, 0.1, 0, false, &core_climb_min );
	vccm->setHelp("Minimum climb rate that counts for arithmetic mean climb value");
	top->addEntry( vccm );

	SetupMenuValFloat * vcch = new SetupMenuValFloat( "Duration", "min", 1, 300, 1, 0, false, &core_climb_history );
	vcch->setHelp("Duration in minutes over which mean climb rate is computed, default is last 3 thermals or 45 min");
	top->addEntry( vcch );

	SetupMenuValFloat * vcp = new SetupMenuValFloat( "Cycle", "sec", 60, 300, 1, 0, false, &core_climb_period );
	vcp->setHelp("Cycle: number of seconds when mean climb value is recalculated, default is every 60 seconds");
	top->addEntry( vcp);

	SetupMenuValFloat * vcmc = new SetupMenuValFloat( "Major Change", "m/s", 0.1, 5.0, 0.1, 0, false, &mean_climb_major_change );
	vcmc->setHelp("Change in mean climb during last cycle (minute), that results in a major change indication (with arrow symbol)");
	top->addEntry( vcmc);
}

void SetupMenu::vario_menu_create_s2f( MenuEntry *top ){
	// repeat the MC menu here for convenience
	SetupMenuValFloat * mc = new SetupMenuValFloat( "MC", "",	0.0, 9.9, 0.1, 0, true, &MC );
	mc->setHelp("MacCready value for optimum cruise speed, or average climb rate to be provided in same unit as the variometer");
	mc->setPrecision(1);
	top->addEntry( mc );

	SetupMenuValFloat * vds2 = new SetupMenuValFloat( "Damping", "sec", 0.10001, 10.0, 0.1, 0, false, &s2f_delay );
	vds2->setHelp("Time constant of S2F low pass filter");
	top->addEntry( vds2 );

	SetupMenuSelect * blck = new SetupMenuSelect( "Blockspeed", RST_NONE, 0 , true, &s2f_blockspeed );
	blck->setHelp("With Blockspeed enabled, vertical movement of airmass or G-load is not considered for speed to fly (S2F) calculation");
	blck->addEntry( "DISABLE");
	blck->addEntry( "ENABLE");
	top->addEntry( blck );

	SetupMenuSelect * s2fmod = new SetupMenuSelect( "S2F Mode", RST_NONE, 0 , true, &s2f_switch_mode );
	s2fmod->setHelp( "Select data source for switching between S2F and Vario modes", 230 );
	s2fmod->addEntry( "Vario fix");
	s2fmod->addEntry( "Cruise fix");
	s2fmod->addEntry( "Switch");
	s2fmod->addEntry( "AutoSpeed");
	s2fmod->addEntry( "External");
	s2fmod->addEntry( "Flap");
	s2fmod->addEntry( "AHRS-Gyro");
	top->addEntry( s2fmod );

	SetupMenuSelect * s2fsw = new SetupMenuSelect( "S2F Switch", RST_NONE , 0, false, &s2f_switch_type );
	top->addEntry( s2fsw );
	s2fsw->setHelp( "Select S2F switch type: normal switch, push button (toggling S2F mode on each press), or disabled");
	s2fsw->addEntry( "Switch");
	s2fsw->addEntry( "Push Button");
	s2fsw->addEntry( "Switch Invert");
	s2fsw->addEntry( "Disable");

	SetupMenuValFloat * autospeed = new SetupMenuValFloat( "S2F AutoSpeed", "", 20.0, 250.0, 1.0, update_s2f_speed, false, &s2f_speed );
	top->addEntry( autospeed );
	autospeed->setHelp("Transition speed if using AutoSpeed to switch Vario <-> Cruise (S2F) mode");

	SetupMenuValFloat * s2f_flap = new SetupMenuValFloat( "S2F Flap Pos", "", -3, 3, 0.1, 0 , false, &s2f_flap_pos );
	top->addEntry( s2f_flap );
	s2f_flap->setHelp("Precise flap position used for switching Vario <-> Cruise (S2F)");

	SetupMenuValFloat * s2f_gyro = new SetupMenuValFloat( "S2F AHRS Deg", "°", 0, 100, 1, 0 , false, &s2f_gyro_deg );
	top->addEntry( s2f_gyro );
	s2f_gyro->setHelp("Attitude change in degrees per second to switch Vario <-> Cruise (S2F) mode");

	SetupMenuValFloat * s2fhy = new SetupMenuValFloat( "Hysteresis", "",	-20, 20, 1, 0, false, &s2f_hysteresis );
	s2fhy->setHelp("Hysteresis (+- this value) for Autospeed S2F transition");
	top->addEntry( s2fhy );

	SetupMenuSelect * s2fnc = new SetupMenuSelect( "Arrow Color", RST_NONE, 0 , true, &s2f_arrow_color );
	s2fnc->setHelp( "Select color of the S2F arrow when painted in Up/Down position" );
	s2fnc->addEntry( "White/White");
	s2fnc->addEntry( "Blue/Blue");
	s2fnc->addEntry( "Green/Red");
	top->addEntry( s2fnc );
}

void SetupMenu::vario_menu_create_ec( MenuEntry *top ){
	SetupMenuSelect * enac = new SetupMenuSelect( "eCompensation", RST_NONE, 0 , false, &te_comp_enable );
	enac->setHelp("Enable/Disable electronic TE compensation option; Enable only when TE port is connected to ST (static) pressure");
	enac->addEntry( "DISABLE");
	enac->addEntry( "ENABLE");
	top->addEntry( enac );

	SetupMenuValFloat * elca = new SetupMenuValFloat( "Adjustment", "%",	-100, 100, 0.1, 0, false, &te_comp_adjust );
	elca->setHelp("Adjustment option for electronic TE compensation in %. This affects the energy altitude calculated from airspeed");
	top->addEntry( elca );
};

void SetupMenu::vario_menu_create_more( MenuEntry *top ){

	SetupMenuSelect * ncolor = new SetupMenuSelect( "Needle Color", RST_NONE, 0 , true, &needle_color );
	ncolor->setHelp("Choose the color of the vario needle");
	ncolor->addEntry( "White");
	ncolor->addEntry( "Orange");
	ncolor->addEntry( "Red");
	top->addEntry( ncolor );

	SetupMenuValFloat * vda = new SetupMenuValFloat( "Damping", "sec", 2.0, 10.0, 0.1, vario_setup, false, &vario_delay );
	vda->setHelp("Response time, time constant of Vario low pass kalman filter");
	top->addEntry( vda );

	SetupMenuValFloat * vdav = new SetupMenuValFloat( "Averager", "sec", 2.0, 60.0, 1, 0, false, &vario_av_delay );
	vdav->setHelp("Response time, time constant of digital Average Vario Display");
	top->addEntry( vdav );

	SetupMenu * meanclimb = new SetupMenu( "Mean Climb" );
	meanclimb->setHelp("Options for calculation of Mean Climb (MC recommendation) displayed by green/red rhombus");
	top->addEntry( meanclimb );
	meanclimb->addCreator( vario_menu_create_meanclimb );

	SetupMenu * s2fs = new SetupMenu( "S2F Settings" );
	top->addEntry( s2fs, meanclimb );
	s2fs->addCreator( vario_menu_create_s2f );

	SetupMenu * elco = new SetupMenu( "Electronic Compensation" );
	top->addEntry( elco, s2fs );
	elco->addCreator( vario_menu_create_ec );
}

void SetupMenu::vario_menu_create( MenuEntry *vae ){
	//ESP_LOGI(FNAME,"SetupMenu::vario_menu_create( %p )", vae );

	SetupMenuValFloat * vga = new SetupMenuValFloat( "Range", "",	1.0, 30.0, 1, audio_setup_f, true, &range );
	vga->setHelp("Upper and lower value for Vario graphic display region");
	vga->setPrecision( 0 );
	vae->addEntry( vga );

	SetupMenuSelect *vlogscale = new SetupMenuSelect( "Log. Scale", RST_NONE, 0, true, &log_scale );
	vlogscale->setHelp("Use a logarithmic scale in the vario gauge");
	vlogscale->addEntry( "DISABLE" );
	vlogscale->addEntry( "ENABLE" );
	vae->addEntry( vlogscale /* , vga */ );

	SetupMenuSelect * vamod = new SetupMenuSelect( 	"Mode", RST_NONE, 0 , true, &vario_mode );
	vamod->setHelp("Controls if vario considers polar sink (=Netto), or not (=Brutto), or if Netto vario applies only in Cruise Mode");
	vamod->addEntry( "Brutto");
	vamod->addEntry( "Netto");
	vamod->addEntry( "Cruise-Netto");
	vae->addEntry( vamod /* , vlogscale */);

	SetupMenuSelect * nemod = new SetupMenuSelect( "Netto Mode", RST_NONE, 0 , true, &netto_mode );
	nemod->setHelp("In 'Relative' mode (also called 'Super-Netto') circling sink is considered,  to show climb rate as if you were circling there");
	nemod->addEntry( "Normal");
	nemod->addEntry( "Relative");
	vae->addEntry( nemod /* , vamod */ );

	SetupMenuSelect * sink = new SetupMenuSelect( "Polar Sink", RST_NONE, 0 , true, &ps_display );
	sink->setHelp("Display polar sink rate together with climb rate when Vario is in Brutto Mode (else disabled)");
	sink->addEntry( "DISABLE");
	sink->addEntry( "ENABLE");
	vae->addEntry( sink /* , nemod */ );

	SetupMenuSelect * scrcaid = new SetupMenuSelect( "Center-Aid", RST_ON_EXIT, 0, true, &screen_centeraid );
	scrcaid->setHelp("Enable/disable display of centering aid (reboots)");
	scrcaid->addEntry( "Disable");
	scrcaid->addEntry( "Enable");
	vae->addEntry( scrcaid );

	SetupMenu * vmore = new SetupMenu( "More Vario Options" );
	vmore->addCreator( vario_menu_create_more );
	vae->addEntry( vmore );
}

void SetupMenu::audio_menu_create_tonestyles( MenuEntry *top ){

	// the NG variable is still called "dual_tone" but it now includes choice of RICO style
	SetupMenuSelect * dt = new SetupMenuSelect( "Tone Flavor", RST_NONE, audio_setup_s, true, &dual_tone );
	dt->setHelp("Single tone (di/di/di), dual tone (ILEC style) (di/da/di), short beeps (di, di), or very short (RICO style) (tick, tick)");
	dt->addEntry( "Single Tone");      // 0=ATM_SINGLE_TONE
	dt->addEntry( "Dual Tone");        // 1=ATM_DUAL_TONE
	dt->addEntry( "Short Beeps");      // 2=ATM_RICO_LONG
	dt->addEntry( "Very Short");       // 3=ATM_RICO_SHORT
	top->addEntry( dt );

	SetupMenuSelect * tch = new SetupMenuSelect( "Chopping", RST_NONE, eval_chop, true, &chopping_mode );
	tch->setHelp("Select tone chopping option on positive values for Vario and or S2F");
	tch->addEntry( "Disabled");             // 0
	tch->addEntry( "Vario only");           // 1
	tch->addEntry( "S2F only");             // 2
	tch->addEntry( "Vario and S2F");        // 3  default
	top->addEntry( tch );

	SetupMenuSelect * tchs = new SetupMenuSelect( "Chopping Style", RST_NONE, audio_setup_s, true, &chopping_style );
	tchs->setHelp("Tone chopping style: hard, or soft with fadein/fadeout");
	tchs->addEntry( "Soft");             // 0  default
	tchs->addEntry( "Hard");             // 1
	tchs->addEntry( "Medium");           // 2
	top->addEntry( tchs );

	SetupMenuValFloat * cf = new SetupMenuValFloat( "CenterFreq", "Hz", 200.0, 2000.0, 10.0, audio_setup_f, false, &center_freq );
	cf->setHelp("Center frequency for Audio at zero Vario or zero S2F delta");
	top->addEntry( cf );

	SetupMenuValFloat * oc = new SetupMenuValFloat( "Octaves", "fold", 1.2, 4, 0.05, audio_setup_f, false, &tone_var );
	oc->setHelp("Maximum tone frequency variation");
	top->addEntry( oc );

	SetupMenuValFloat * htv = new SetupMenuValFloat( "Dual Tone Pitch", "%", 0, 50, 1.0, audio_setup_f, false, &high_tone_var );
	htv->setHelp("Tone variation in Dual Tone mode, percent of frequency pitch up for second tone");
	top->addEntry( htv );

	SetupMenuSelect * advarto = new SetupMenuSelect( "Variable Tone", RST_NONE, 0 , true, &audio_variable_frequency );
	advarto->setHelp("Enable audio frequency updates within climbing tone intervals, disable keeps frequency constant");
	advarto->addEntry( "Disable");       // 0
	advarto->addEntry( "Enable");        // 1
	top->addEntry( advarto );
}

void SetupMenu::audio_menu_create_deadbands( MenuEntry *top ){

	SetupMenuValFloat * dbminlv = new SetupMenuValFloat( "Lower Vario", "", -5.0, 0, 0.1, 0 , false, &deadband_neg );
	dbminlv->setHelp("Lower (sink) limit of deadband for Audio mute function when in Vario mode");
	top->addEntry( dbminlv );

	SetupMenuValFloat * dbmaxlv = new SetupMenuValFloat( "Upper Vario", "", 0, 5.0, 0.1, 0 , false, &deadband );
	dbmaxlv->setHelp("Upper (climb) limit of deadband for Audio mute function when in Vario mode");
	top->addEntry( dbmaxlv );

	SetupMenuValFloat * dbmaxls2fn = new SetupMenuValFloat(	"Lower S2F", "", -25.0, 0, 1, 0 , false, &s2f_deadband_neg );
	dbmaxls2fn->setHelp("Negative (too slow) speed deviation limit of deadband in S2F mode");
	top->addEntry( dbmaxls2fn );

	SetupMenuValFloat * dbmaxls2f = new SetupMenuValFloat( "Upper S2F", "", 0, 25.0, 1, 0 , false, &s2f_deadband );
	dbmaxls2f->setHelp("Positive (too fast) speed deviation limit of deadband in S2F mode");
	top->addEntry( dbmaxls2f );
}

void SetupMenu::audio_menu_create_equalizer( MenuEntry *top ){
	SetupMenuSelect * audeqt = new SetupMenuSelect( "Equalizer", RST_ON_EXIT, 0 , true, &audio_equalizer );
	audeqt->setHelp("Select the equalizer according to the type of loudspeaker used");
	audeqt->addEntry( "Disable");
	audeqt->addEntry( "Speaker 8 Ohms");
	audeqt->addEntry( "Speaker 4 Ohms");
	audeqt->addEntry( "Speaker External");
	top->addEntry( audeqt );

	SetupMenuValFloat * frqr = new SetupMenuValFloat( "Frequency Response", "%",
	    -70.0, 70.0, 1.0, 0, false, &frequency_response );
	frqr->setHelp("Setup frequency response, double frequency will be attenuated by the factor given, half frequency will be amplified");
	top->addEntry( frqr );
}

void SetupMenu::audio_menu_create_volume( MenuEntry *top ){

	SetupMenuValFloat * vol = new SetupMenuValFloat( "Current Volume", "%",
	    0.0, 200.0, 2.0, vol_adj, false, &audio_volume );
	// unlike top-level menu volume which exits setup, this returns to parent menu
	vol->setHelp("Audio volume level for variometer tone on internal and external speaker");
	vol->setMax(max_volume.get());   // this only works after leaving *parent* menu and returning
	volume_menu = vol;               // but this allows changing the volume menu max later
	top->addEntry( vol );

	SetupMenuSelect * cdv = new SetupMenuSelect( "Current->Default", RST_NONE, cur_vol_dflt, true );
	cdv->addEntry( "Cancel" );
	cdv->addEntry( "Set" );
	cdv->setHelp("Set current volume as default volume when device is switched on");
	top->addEntry( cdv );

	SetupMenuValFloat * dv = new SetupMenuValFloat( "Default Volume", "%",
		    0.0, 100.0, 2.0, 0, false, &default_volume );
	dv->setHelp("Default volume for Audio when device is switched on");
	top->addEntry( dv );

	SetupMenuValFloat * mv = new SetupMenuValFloat( "Max Volume", "%",
		    0.0, 200.0, 5.0, 0, false, &max_volume );
	mv->setHelp("Maximum audio volume setting allowed");
	top->addEntry( mv );

	SetupMenu * audeq = new SetupMenu( "Equalizer" );
	audeq->setHelp( "Equalization parameters for a constant perceived volume over a wide frequency range", 220);
	audeq->addCreator(audio_menu_create_equalizer);
	top->addEntry( audeq );

	SetupMenuSelect * amspvol = new SetupMenuSelect( "STF Volume", RST_NONE, 0 , true, &audio_split_vol );
	amspvol->setHelp("Enable independent audio volume in SpeedToFly and Vario modes, disable for one volume for both");
	amspvol->addEntry( "Disable");      // 0
	amspvol->addEntry( "Enable");       // 1
	top->addEntry( amspvol );

	SetupMenuSelect * sv = new SetupMenuSelect( "2-Seat Volume", RST_NONE, 0 , true, &sync_volume );
	sv->setHelp("Separate: independent volume control in master and slave XCvario units; Linked: controlled together");
	sv->addEntry( "Separate");      // 0
	sv->addEntry( "Linked");        // 1
	top->addEntry( sv );
}

void SetupMenu::audio_menu_create_mute( MenuEntry *top ){

	SetupMenu * db = new SetupMenu( "Deadbands" );
	top->addEntry( db );
	db->setHelp("Dead band limits within which audio remains silent.  1 m/s equals roughly 200 fpm or 2 knots");
	db->addCreator(audio_menu_create_deadbands);

	SetupMenuSelect * asida = new SetupMenuSelect( "In Sink", RST_NONE, 0 , true, &audio_mute_sink );
	asida->setHelp("Configure vario audio volume while in sink (below deadband)");
	asida->addEntry( "Normal");  // 0
	asida->addEntry( "Louder");  // 1
	asida->addEntry( "Softer");  // 2
	asida->addEntry( "Mute");    // 3
	top->addEntry( asida );

	SetupMenuSelect * ameda = new SetupMenuSelect( "In Setup", RST_NONE, 0 , true, &audio_mute_menu );
	ameda->setHelp("Select whether vario audio will be muted while Setup Menu is open");
	ameda->addEntry( "Stay On");  // 0
	ameda->addEntry( "Mute");     // 1
	top->addEntry( ameda );

	SetupMenuSelect * ageda = new SetupMenuSelect( "Generally", RST_NONE, 0 , true, &audio_mute_gen );
	ageda->setHelp("Select audio on, or vario audio muted, or all audio muted including alarms");
	ageda->addEntry( "Audio On");      // 0 = AUDIO_ON
	ageda->addEntry( "Alarms On");     // 1 = AUDIO_ALARMS
	ageda->addEntry( "Audio Off");     // 2 = AUDIO_OFF
	top->addEntry( ageda );

	SetupMenuSelect * amps = new SetupMenuSelect( "Amplifier", RST_NONE, 0 , true, &amplifier_shutdown );
	amps->setHelp("Whether amplifier always stays on, is shutdown in deadband immediately, or after 5 sec silence");
	amps->addEntry( "Stay On");   // 0 = AMP_STAY_ON
	amps->addEntry( "Shutdown");  // 1 = AMP_SHUTDOWN
	amps->addEntry( "After 5s");  // 2 = AMP_SHUTDOWN_5S
	top->addEntry( amps );
}

// a few audio options to be made accessible in student mode
void SetupMenu::audio_menu_create_student( MenuEntry *top ){

	SetupMenuSelect * dt = new SetupMenuSelect( "Tone Flavor", RST_NONE, audio_setup_s, true, &dual_tone );
	dt->setHelp("Single tone (di/di/di), dual tone (ILEC style) (di/da/di), short beeps (di, di), or very short (RICO style) (tick, tick)");
	dt->addEntry( "Single Tone");      // 0=ATM_SINGLE_TONE
	dt->addEntry( "Dual Tone");        // 1=ATM_DUAL_TONE
	dt->addEntry( "Short Beeps");      // 2=ATM_RICO_LONG
	dt->addEntry( "Very Short");       // 3=ATM_RICO_SHORT
	top->addEntry( dt );

	SetupMenuSelect * asida = new SetupMenuSelect( "In Sink", RST_NONE, 0 , true, &audio_mute_sink );
	asida->setHelp("Configure vario audio volume while in sink (below deadband)");
	asida->addEntry( "Normal");  // 0
	asida->addEntry( "Louder");  // 1
	asida->addEntry( "Softer");  // 2
	asida->addEntry( "Mute");    // 3
	top->addEntry( asida );

	// like top-level menu, exit setup (unlike the non-student volume menu)
	SetupMenuValFloat * vol = new SetupMenuValFloat( "Current Volume", "%",
	    0.0, 200.0, 2.0, vol_adj, true, &audio_volume );
	vol->setHelp("Audio volume level for variometer tone on internal and external speaker");
	vol->setMax(max_volume.get());   // only works after leaving this *parent* menu and returning
	top->addEntry( vol );

	// after max_volume exit menu, when re-entering will setMax() volume setting above
	SetupMenuValFloat * mv = new SetupMenuValFloat( "Max Volume", "%",
		    0.0, 200.0, 5.0, 0, true, &max_volume );
	mv->setHelp("Maximum audio volume setting allowed");
	top->addEntry( mv );

	SetupMenuSelect * amspvol = new SetupMenuSelect( "STF Volume", RST_NONE, 0 , true, &audio_split_vol );
	amspvol->setHelp("Enable independent audio volume in SpeedToFly and Vario modes, disable for one volume for both");
	amspvol->addEntry( "Disable");      // 0
	amspvol->addEntry( "Enable");       // 1
	top->addEntry( amspvol );
}

void SetupMenu::audio_menu_create( MenuEntry *audio ){

	// volume menu has gone out of scope by now
	// make sure update_volume_menu_max() does not try and dereference it
	volume_menu = 0;
	SetupMenu * volumes = new SetupMenu( "Volume options" );
	audio->addEntry( volumes );
	volumes->setHelp( "Configure audio volume options", 240);
	volumes->addCreator(audio_menu_create_volume);

	SetupMenu * mutes = new SetupMenu( "Mute Audio" );
	audio->addEntry( mutes );
	mutes->setHelp( "Configure audio muting options", 240);
	mutes->addCreator(audio_menu_create_mute);

	SetupMenuSelect * abnm = new SetupMenuSelect( "Cruise Audio", RST_NONE, 0 , true, &cruise_audio_mode );
	abnm->setHelp("Select either S2F command or Variometer (Netto/Brutto as selected) as audio source while cruising");
	abnm->addEntry( "Speed2Fly");       // 0
	abnm->addEntry( "Vario");           // 1
	audio->addEntry( abnm );

	SetupMenu * audios = new SetupMenu( "Tone Styles" );
	audio->addEntry( audios );
	audios->setHelp( "Configure vario audio frequencies and chopping", 240);
	audios->addCreator(audio_menu_create_tonestyles);

	update_rentry(0);
	audio_range_sm = new SetupMenuSelect( "Range", RST_NONE, audio_setup_s, true, &audio_range  );
	audio_range_sm->addEntry( rentry0  );
	audio_range_sm->addEntry( rentry1  );
	audio_range_sm->addEntry( rentry2  );
	audio_range_sm->setHelp("Audio range: fixed, or variable according to current Vario display range setting");
	audio->addEntry( audio_range_sm );

	SetupMenuValFloat * afac = new SetupMenuValFloat( "Audio Exponent", "", -0.5, 2.0, 0.1, 0 , false, &audio_factor );
	afac->setHelp("How the audio frequency responds to the climb rate: < 1 for logarithmic, and > 1 for exponential, response");
	audio->addEntry( afac);
}

void SetupMenu::glider_menu_create_polarpoints( MenuEntry *top ){
	SetupMenuValFloat * wil = new SetupMenuValFloat( "Ref Wingload", "kg/m2", 10.0, 100.0, 0.1, 0, false, &polar_wingload );
	wil->setHelp("Wingloading that corresponds to the 3 value pairs for speed/sink of polar");
	top->addEntry( wil );
	SetupMenuValFloat * pov1 = new SetupMenuValFloat( "Speed 1", "km/h", 50.0, 120.0, 1, 0, false, &polar_speed1);
	pov1->setHelp("Speed 1, near minimum sink from polar e.g. 80 km/h");
	top->addEntry( pov1 );
	SetupMenuValFloat * pos1 = new SetupMenuValFloat( "Sink  1", "m/s", -3.0, 0.0, 0.01, 0, false, &polar_sink1 );
	pos1->setHelp("Sink at Speed 1 from polar");
	top->addEntry( pos1 );
	SetupMenuValFloat * pov2 = new SetupMenuValFloat( "Speed 2", "km/h", 70.0, 180.0, 1, 0, false, &polar_speed2 );
	pov2->setHelp("Speed 2 for a moderate cruise from polar e.g. 120 km/h");
	top->addEntry( pov2 );
	SetupMenuValFloat * pos2 = new SetupMenuValFloat( "Sink  2",  "m/s", -5.0, 0.0, 0.01, 0, false, &polar_sink2 );
	pos2->setHelp("Sink at Speed 2 from polar");
	top->addEntry( pos2 );
	SetupMenuValFloat * pov3 = new SetupMenuValFloat( "Speed 3", "km/h", 100.0, 250.0, 1, 0, false, &polar_speed3 );
	pov3->setHelp("Speed 3 for a fast cruise from polar e.g. 170 km/h");
	top->addEntry( pov3 );
	SetupMenuValFloat * pos3 = new SetupMenuValFloat( "Sink  3", "m/s", -6.0, 0.0, 0.01, 0, false, &polar_sink3 );
	pos3->setHelp("Sink at Speed 3 from polar");
	top->addEntry( pos3 );
}

void SetupMenu::glider_menu_create( MenuEntry *poe ){
	SetupMenuSelect * glt = new SetupMenuSelect( "Glider-Type",	RST_NONE, polar_select, true, &glider_type );
	poe->addEntry( glt );
	for( int x=0; x< Polars::numPolars(); x++ ){
		glt->addEntry( Polars::getPolar(x).type );
	}
	poe->setHelp( "Weight and polar setup for best match with performance of glider", 220 );
	ESP_LOGI(FNAME, "Number of Polars installed: %d", Polars::numPolars() );

	SetupMenu * pa = new SetupMenu( "Polar Points" );
	pa->setHelp("Adjust the polar at 3 points, in the commonly used metric system", 230 );
	poe->addEntry( pa );
	pa->addCreator(glider_menu_create_polarpoints);

	SetupMenuValFloat * maxbal = new SetupMenuValFloat(	"Max Ballast", "liters", 0, 500, 1, 0, false, &polar_max_ballast );
	maxbal->setHelp("Maximum water ballast for selected glider, to sync with XCSoar using fraction of max ballast");
	poe->addEntry( maxbal );

	SetupMenuValFloat * wingarea = new SetupMenuValFloat( "Wing Area", "m2", 0, 50, 0.1, 0, false, &polar_wingarea );
	wingarea->setHelp("Wing area for the selected glider - adjustment to support wing extensions or new types, in square meters");
	poe->addEntry( wingarea );

	SetupMenuValFloat * fixball = new SetupMenuValFloat( "Empty Weight", "kg", 0, 1000, 1, empty_weight_adj, false, &empty_weight );
	fixball->setPrecision(0);
	fixball->setHelp("Net rigged weight of the glider, according to the weight and balance plan");
	poe->addEntry( fixball );
}

void SetupMenu::options_menu_create_units( MenuEntry *top ){
	SetupMenuSelect * alu = new SetupMenuSelect( "Altimeter", RST_NONE, 0, true, &alt_unit );
	alu->addEntry( "Meter (m)");
	alu->addEntry( "Feet (ft)");
	alu->addEntry( "FL (FL)");
	top->addEntry( alu );
	SetupMenuSelect * iau = new SetupMenuSelect( "Airspeed", RST_NONE , 0, true, &ias_unit );
	iau->addEntry( "Kilom./hour (Km/h)");
	iau->addEntry( "Miles/hour (mph)");
	iau->addEntry( "Knots (kt)");
	top->addEntry( iau );
	SetupMenuSelect * vau = new SetupMenuSelect( "Vario", RST_NONE , update_rentrys, true, &vario_unit );
	vau->addEntry( "Meters/sec (m/s)");
	vau->addEntry( "Feet/min x 100 (fpm)");
	vau->addEntry( "Knots (kt)");
	top->addEntry( vau );
	SetupMenuSelect * teu = new SetupMenuSelect( "Temperature", RST_NONE , 0, true, &temperature_unit );
	teu->addEntry( "Celcius");
	teu->addEntry( "Fahrenheit");
	teu->addEntry( "Kelvin");
	top->addEntry( teu );
	SetupMenuSelect * qnhi = new SetupMenuSelect( "QNH", RST_NONE, 0, true, &qnh_unit );
	qnhi->addEntry( "Hectopascal");
	qnhi->addEntry( "InchMercury");
	top->addEntry( qnhi );
	SetupMenuSelect * dst = new SetupMenuSelect( "Distance", RST_NONE , 0, true, &dst_unit );
	dst->addEntry( "Meter (m)");
	dst->addEntry( "Feet (ft)");
	top->addEntry( dst );
}

void SetupMenu::options_menu_create_flarm( MenuEntry *top ){

	SetupMenuSelect * flarml = new SetupMenuSelect( "Visual Alarm", RST_NONE, 0, true, &flarm_visual );
	flarml->setHelp( "FLARM level 1 is lowest with 13-18 sec, 2 medium 9-12 sec and 3 highest with 0-8 sec until impact");
	flarml->addEntry( "Disable");
	flarml->addEntry( "Level 1+");
	flarml->addEntry( "Level 2+");
	flarml->addEntry( "Level 3");
	top->addEntry( flarml );

	SetupMenuSelect * flarmi = new SetupMenuSelect( "Visual Style", RST_NONE, 0, true, &flarm_2icons );
	flarmi->setHelp( "FLARM Alarm screen style: single icon for traffic bearing, or add second icon for vertical angle");
	flarmi->addEntry( "One icon");
	flarmi->addEntry( "Two icons");
	top->addEntry( flarmi );

	SetupMenuSelect * flarma = new SetupMenuSelect( "Audio Alarm", RST_NONE, 0, true, &flarm_sound );
	flarma->setHelp( "FLARM level 1 is lowest with 13-18 sec, 2 medium 9-12 sec and 3 highest with 0-8 sec until impact");
	flarma->addEntry( "Disable");
	flarma->addEntry( "Level 1+");
	flarma->addEntry( "Level 2+");
	flarma->addEntry( "Level 3");
	top->addEntry( flarma );

	SetupMenuSelect * flarmc = new SetupMenuSelect( "Audio Style", RST_NONE, 0, true, &flarm_sound_continuous );
	flarmc->setHelp( "FLARM Alarm Audio: Short (<=2 sec) for each higher level or new aircraft, or Continuous while danger");
	flarmc->addEntry( "Short");
	flarmc->addEntry( "Continuous");
	top->addEntry( flarmc );

	SetupMenuValFloat * flarmv = new SetupMenuValFloat( "Alarm Volume", "%", 20, 100, 1, 0, false, &flarm_volume  );
	flarmv->setHelp( "Maximum audio volume of FLARM alarm warning");
	top->addEntry( flarmv );

	SetupMenuSelect * flarms = new SetupMenuSelect( "FLARM Simulation", RST_NONE, 0, true, &flarm_sim, false, true );
	flarms->setHelp( "Simulate an airplane crossing from left to right with different alarm levels and vertical distance 5 seconds after pressed (exits setup!)");
	flarms->addEntry( "Disable");
	flarms->addEntry( "Start Sim");
	top->addEntry( flarms );
}

void SetupMenu::system_menu_create_compass_dev( MenuEntry *top ){

	const char *skydirs[8] = { "0°", "45°", "90°", "135°", "180°", "225°", "270°", "315°" };
	for( int i = 0; i < 8; i++ )
	{
		SetupMenuSelect* sms = new SetupMenuSelect( "Direction", RST_NONE, compassDeviationAction, false, 0 );
		sms->setHelp( "Push button to start deviation action" );
		sms->addEntry( skydirs[i] );
		top->addEntry( sms );
	}
}

void SetupMenu::system_menu_create_compass_devs( MenuEntry *top ){

	SetupMenuSelect * devMenuA = new SetupMenuSelect( "AutoDeviation", RST_NONE, 0, true, &compass_dev_auto );
	devMenuA->setHelp( "Automatic adaptive deviation and precise airspeed evaluation method using data from circling wind");
	devMenuA->addEntry( "Disable");
	devMenuA->addEntry( "Enable");
	top->addEntry( devMenuA );

	SetupMenu * devMenu = new SetupMenu( "Setup Deviations" );
	devMenu->setHelp( "Compass Deviations", 280 );
	top->addEntry( devMenu );
	devMenu->addCreator( system_menu_create_compass_dev );

	// Show compass deviations
	DisplayDeviations* smd = new DisplayDeviations( "Show Deviations" );
	top->addEntry( smd );

	SetupMenuSelect* sms = new SetupMenuSelect( "Reset Deviations ", RST_NONE, compassResetDeviationAction, false,	0 );
	sms->setHelp( "Reset all deviation data to zero" );
	sms->addEntry( "Cancel" );
	sms->addEntry( "Reset" );
	top->addEntry( sms );
}

void SetupMenu::system_menu_create_compass_nmea( MenuEntry *top ){

	SetupMenuSelect * nmeaHdm = new SetupMenuSelect( "Magnetic Heading", RST_NONE, 0, true, &compass_nmea_hdm );
	nmeaHdm->addEntry( "Disable");
	nmeaHdm->addEntry( "Enable");
	nmeaHdm->setHelp( "Enable/disable NMEA '$HCHDM' sentence generation for magnetic heading" );
	top->addEntry( nmeaHdm );

	SetupMenuSelect * nmeaHdt = new SetupMenuSelect( "True Heading", RST_NONE, 0, true, &compass_nmea_hdt );
	nmeaHdt->addEntry( "Disable");
	nmeaHdt->addEntry( "Enable");
	nmeaHdt->setHelp( "Enable/disable NMEA '$HCHDT' sentence generation for true heading" );
	top->addEntry( nmeaHdt );
}

void SetupMenu::system_menu_create_compass_straightwind_filters( MenuEntry *top ){
	SetupMenuValFloat *smgsm = new SetupMenuValFloat( "Airspeed Lowpass", "", 0, 0.05, 0.001, nullptr, false, &wind_as_filter );
	smgsm->setPrecision(3);
	top->addEntry( smgsm );
	smgsm->setHelp("Lowpass filter factor (per sec) for airspeed estimation from AS/Compass and GPS tracks");

	SetupMenuValFloat *devlp = new SetupMenuValFloat( "Deviation Lowpass", "", 0, 0.05, 0.001, nullptr, false, &wind_dev_filter );
	devlp->setPrecision(3);
	top->addEntry( devlp );
	devlp->setHelp("Lowpass filter factor (per sec) for deviation table correction from AS/Compass and GPS tracks");

	SetupMenuValFloat *smgps = new SetupMenuValFloat( "GPS Lowpass", "sec", 0.1, 10.0, 0.1, nullptr, false, &wind_gps_lowpass );
	smgps->setPrecision(1);
	top->addEntry( smgps );
	smgps->setHelp("Lowpass filter factor for GPS track and speed, to correlate with Compass latency");

	SetupMenuValFloat *wlpf = new SetupMenuValFloat( "Averager", "", 5, 120, 1, nullptr, false, &wind_filter_lowpass );
	wlpf->setPrecision(0);
	top->addEntry( wlpf );
	wlpf->setHelp("Number of measurements (seconds) averaged in straight flight live wind estimation");
}

void SetupMenu::system_menu_create_compass_straightwind_limits( MenuEntry *top ){
	SetupMenuValFloat *smdev = new SetupMenuValFloat( "Deviation Limit", "°", 0.0, 180.0, 1.0,	nullptr, false, &wind_max_deviation );
	smdev->setHelp( "Maximum deviation change accepted when derived from AS/Compass and GPS tracks" );
	top->addEntry( smdev );

	SetupMenuValFloat *smslip = new SetupMenuValFloat( "Sideslip Limit", "°", 0, 45.0, 0.1, nullptr, false, &swind_sideslip_lim );
	smslip->setPrecision(1);
	top->addEntry( smslip );
	smslip->setHelp("Maximum side slip estimation in ° accepted for straight wind calculation");

	SetupMenuValFloat *smcourse = new SetupMenuValFloat( "Course Limit", "°", 2.0, 30.0, 0.1, nullptr, false, &wind_straight_course_tolerance );
	smcourse->setPrecision(1);
	top->addEntry( smcourse );
	smcourse->setHelp("Maximum delta angle in ° per second accepted for straight wind calculation");

	SetupMenuValFloat *aslim = new SetupMenuValFloat( "AS Delta Limit", "km/h", 1.0, 30.0, 1, nullptr, false, &wind_straight_speed_tolerance );
	aslim->setPrecision(0);
	top->addEntry( aslim );
	aslim->setHelp("Maximum delta in airspeed estimation from wind and GPS during straight flight accepted for straight wind calculation");
}

void SetupMenu::system_menu_create_compass_straightwind( MenuEntry *top ){
	SetupMenu * strWindFM = new SetupMenu( "Filters" );
	top->addEntry( strWindFM );
	strWindFM->addCreator( system_menu_create_compass_straightwind_filters );
	SetupMenu * strWindLM = new SetupMenu( "Limits" );
	top->addEntry( strWindLM );
	strWindLM->addCreator( system_menu_create_compass_straightwind_limits );
	ShowStraightWind* ssw = new ShowStraightWind( "Straight Wind Status" );
	top->addEntry( ssw );
}

void SetupMenu::system_menu_create_compass( MenuEntry *top ){

	SetupMenu * nmeaMenu = new SetupMenu( "Setup NMEA" );
	top->addEntry( nmeaMenu );
	nmeaMenu->addCreator( system_menu_create_compass_nmea );

	SetupMenu * strWindM = new SetupMenu( "Straight Wind" );
	top->addEntry( strWindM );
	strWindM->setHelp( "Straight flight wind calculation needs compass module active", 250 );
	strWindM->addCreator( system_menu_create_compass_straightwind );

	SetupMenuSelect * compSensorCal = new SetupMenuSelect( "Sensor Calibration", RST_NONE, compassSensorCalibrateAction, false );
	compSensorCal->addEntry( "Cancel");
	compSensorCal->addEntry( "Start");
	compSensorCal->addEntry( "Show");
	compSensorCal->addEntry( "Show Raw Data");
	compSensorCal->setHelp( "Calibrate Magnetic Sensor, mandatory for operation" );
	top->addEntry( compSensorCal );

	SetupMenuValFloat *cd = new SetupMenuValFloat( "Setup Declination",	"°",	-180, 180, 1.0, compassDeclinationAction, false, &compass_declination );
	cd->setHelp( "Set compass declination in degrees" );
	top->addEntry( cd );

	SetupMenu * devMenu = new SetupMenu( "Deviations" );
	devMenu->setHelp( "Compass Deviations", 280 );
	top->addEntry( devMenu );
	devMenu->addCreator( system_menu_create_compass_devs );

	SetupMenuValFloat * compdamp = new SetupMenuValFloat( "Damping", "sec", 0.1, 10.0, 0.1, 0, false, &compass_damping );
	compdamp->setPrecision(1);
	top->addEntry( compdamp );
	compdamp->setHelp("Compass or magnetic heading damping factor in seconds");

	SetupMenuSelect * compSensor = new SetupMenuSelect( "Sensor Option", RST_ON_EXIT, compass_ena, true, &compass_enable);
	compSensor->addEntry( "Disable");
	compSensor->addEntry( "Enable I2C sensor");
	compSensor->addEntry( "Disable");
	compSensor->addEntry( "Enable CAN sensor");
	compSensor->setHelp( "Option to enable/disable the Compass Sensor (reboots)" );
	top->addEntry( compSensor );

	SetupMenuValFloat * compi2c = new SetupMenuValFloat( "I2C Clock", "KHz", 10.0, 400.0, 10, 0, false, &compass_i2c_cl, RST_ON_EXIT );
	top->addEntry( compi2c );
	compi2c->setHelp("Setup compass I2C Bus clock in KHz (reboots)");

	// Show compass settings
	ShowCompassSettings* scs = new ShowCompassSettings( "Show Settings" );
	top->addEntry( scs );
}

void SetupMenu::options_menu_create_wind_circlingwind( MenuEntry *top ){
	// Show Circling Wind Status
	ShowCirclingWind* scw = new ShowCirclingWind( "Circling Wind Status" );
	top->addEntry( scw );

	SetupMenuValFloat *cirwd = new SetupMenuValFloat( "Max Delta", "°", 0, 90.0, 1.0, nullptr, false, &max_circle_wind_diff );
	top->addEntry( cirwd );
	cirwd->setHelp("Maximum accepted value for heading error in circling wind calculation");

	SetupMenuValFloat *cirlp = new SetupMenuValFloat( "Averager", "", 1, 10, 1, nullptr, false, &circle_wind_lowpass );
	cirlp->setPrecision(0);
	top->addEntry( cirlp );
	cirlp->setHelp("Number of circles used for circling wind averager. A value of 1 means no average");

	SetupMenuValFloat *cirwsd = new SetupMenuValFloat( "Max Speed Delta", "km/h", 0.0, 20.0, 0.1, nullptr, false, &max_circle_wind_delta_speed );
	top->addEntry( cirwsd );
	cirwsd->setPrecision(1);
	cirwsd->setHelp("Maximum wind speed delta from last measurement accepted for circling wind calculation");

	SetupMenuValFloat *cirwdd = new SetupMenuValFloat( "Max Dir Delta", "°", 0.0, 60.0, 0.1, nullptr, false, &max_circle_wind_delta_deg );
	top->addEntry( cirwdd );
	cirwdd->setPrecision(1);
	cirwdd->setHelp("Maximum wind direction delta from last measurement accepted for circling wind calculation");
}

void SetupMenu::options_menu_create_wind( MenuEntry *top ){

	// Wind speed observation window
	SetupMenuSelect * windcal = new SetupMenuSelect( "Wind Calculation", RST_NONE, 0, true, &wind_enable );
	windcal->addEntry( "Disable");
	windcal->addEntry( "Straight");
	windcal->addEntry( "Circling");
	windcal->addEntry( "Both");
	windcal->setHelp("Enable Wind calculation for straight flight (needs compass), circling, or both - display wind in retro display style");
	top->addEntry( windcal );

	// Display option
	SetupMenuSelect * winddis = new SetupMenuSelect( "Display", RST_NONE, 0, true, &wind_display );
	winddis->addEntry( "Disable");
	winddis->addEntry( "Wind Digits");
	winddis->addEntry( "Wind Arrow");
	winddis->addEntry( "Wind Both");
	winddis->addEntry( "Compass");
	winddis->setHelp( "What is to be displayed, as digits or arrow or both, on retro style screen. If no wind available, compass is shown");
	top->addEntry( winddis );

	// Wind speed observation window
	SetupMenuSelect * windref = new SetupMenuSelect( "Arrow Ref", RST_NONE, 0, true, &wind_reference );
	windref->addEntry( "North");
	windref->addEntry( "Mag Heading");
	windref->addEntry( "GPS Course");
	windref->setHelp( "Choose wind arrow relative to geographic north or to true aircraft heading");
	top->addEntry( windref );

	SetupMenu * cirWindM = new SetupMenu( "Circling Wind" );
	top->addEntry( cirWindM );
	cirWindM->addCreator( options_menu_create_wind_circlingwind );

	SetupMenuSelect * windlog = new SetupMenuSelect( "Wind Logging", RST_NONE, 0, true, &wind_logging );
	windlog->addEntry( "Disable");
	windlog->addEntry( "Enable WIND");
	windlog->addEntry( "Enable GYRO/MAG");
	windlog->addEntry( "Enable Both");
	windlog->setHelp("Enable Wind logging NMEA output, e.g. to WIFI port");
	top->addEntry( windlog );
}

void SetupMenu::options_menu_create_wireless_custom_id( MenuEntry *top ){
	SetupMenuChar * c1 = new SetupMenuChar( "Letter 1",	RST_NONE, update_id, false, custom_wireless_id.get().id, 0 );
	SetupMenuChar * c2 = new SetupMenuChar( "Letter 2",	RST_NONE, update_id, false, custom_wireless_id.get().id, 1 );
	SetupMenuChar * c3 = new SetupMenuChar( "Letter 3",	RST_NONE, update_id, false, custom_wireless_id.get().id, 2 );
	SetupMenuChar * c4 = new SetupMenuChar( "Letter 4",	RST_NONE, update_id, false, custom_wireless_id.get().id, 3 );
	SetupMenuChar * c5 = new SetupMenuChar( "Letter 5",	RST_NONE, update_id, false, custom_wireless_id.get().id, 4 );
	SetupMenuChar * c6 = new SetupMenuChar( "Letter 6",	RST_NONE, update_id, false, custom_wireless_id.get().id, 5 );
	top->addEntry( c1 );
	top->addEntry( c2 );
	top->addEntry( c3 );
	top->addEntry( c4 );
	top->addEntry( c5 );
	top->addEntry( c6 );
	static const char keys[][4] { "\0","0","1","2","3","4","5","6","7","8","9","-","A","B","C","D","E","F","G","H","I","J","K","L","M","N","O","P","Q","R","S","T","U","V","W","X","Y","Z"};
	c1->addEntryList( keys, sizeof(keys)/4 );
	c2->addEntryList( keys, sizeof(keys)/4 );
	c3->addEntryList( keys, sizeof(keys)/4 );
	c4->addEntryList( keys, sizeof(keys)/4 );
	c5->addEntryList( keys, sizeof(keys)/4 );
	c6->addEntryList( keys, sizeof(keys)/4 );
}

void SetupMenu::options_menu_create_gload_limits( MenuEntry *top ){

	SetupMenuValFloat * glpos = new SetupMenuValFloat( "Red positive limit", "", 1.0, 8.0, 0.1, 0, false, &gload_pos_limit );
	top->addEntry( glpos );
	glpos->setPrecision( 1 );
	glpos->setHelp("Positive g load factor limit the aircraft is able to handle below maneuvering speed, see manual");

	SetupMenuValFloat * glposl = new SetupMenuValFloat( "Yellow pos. Limit", "", 1.0, 8.0, 0.1, 0, false, &gload_pos_limit_low );
	top->addEntry( glposl );
	glposl->setPrecision( 1 );
	glposl->setHelp("Positive g load factor limit the aircraft is able to handle above maneuvering speed, see manual");

	SetupMenuValFloat * glneg = new SetupMenuValFloat( "Red negative limit", "", -8.0, 1.0, 0.1, 0, false, &gload_neg_limit );
	top->addEntry( glneg );
	glneg->setPrecision( 1 );
	glneg->setHelp("Negative g load factor limit the aircraft is able to handle below maneuvering speed, see manual");

	SetupMenuValFloat * glnegl = new SetupMenuValFloat( "Yellow neg. Limit", "", -8.0, 1.0, 0.1, 0, false, &gload_neg_limit_low );
	top->addEntry( glnegl );
	glnegl->setPrecision( 1 );
	glnegl->setHelp("Negative g load factor limit the aircraft is able to handle above maneuvering speed, see manual");
}

void SetupMenu::options_menu_create_gload( MenuEntry *top ){

	SetupMenuSelect * scrgmet = new SetupMenuSelect( "G-Meter", RST_NONE, upd_screens, true, &screen_gmeter );
	scrgmet->addEntry( "Disable");
	scrgmet->addEntry( "Enable");
	top->addEntry(scrgmet);

	SetupMenuSelect * glmod = new SetupMenuSelect( "Activation Mode", RST_NONE, 0, true, &gload_mode );
	glmod->setHelp( "Switch off G-Force screen, or activate by threshold G-Force 'Dynamic', or static by 'Always-On'");
	glmod->addEntry( "Off");
	glmod->addEntry( "Dynamic");
	glmod->addEntry( "Always-On");
	top->addEntry( glmod );

	SetupMenuValFloat * gtpos = new SetupMenuValFloat( "Positive Threshold", "", 1.0, 8.0, 0.1, 0, false, &gload_pos_thresh );
	top->addEntry( gtpos );
	gtpos->setPrecision( 1 );
	gtpos->setHelp("Positive threshold to launch G-Load display");

	SetupMenuValFloat * gtneg = new SetupMenuValFloat( "Negative Threshold", "", -8.0, 1.0, 0.1, 0, false, &gload_neg_thresh );
	top->addEntry( gtneg );
	gtneg->setPrecision( 1 );
	gtneg->setHelp("Negative threshold to launch G-Load display");

	SetupMenu * glimits = new SetupMenu( "G-Load Limits" );
	top->addEntry( glimits );
	glimits->setHelp("G loads the aircraft is able to handle");
	glimits->addCreator(options_menu_create_gload_limits);

	SetupMenuValFloat * gmpos = new SetupMenuValFloat( "Max Positive", "", 0.0, 0.0, 0.0, 0, false, &gload_pos_max );
	top->addEntry( gmpos );
	gmpos->setPrecision( 1 );
	gmpos->setHelp("Maximum positive G-Load measured since last reset");

	SetupMenuValFloat * gmneg = new SetupMenuValFloat( "Max Negative", "", 0.0, 0.0, 0.0, 0, false, &gload_neg_max );
	top->addEntry( gmneg );
	gmneg->setPrecision( 1 );
	gmneg->setHelp("Maximum negative G-Load measured since last reset");

	SetupMenuValFloat * gloadalvo = new SetupMenuValFloat( "Alarm Volume",  "%", 20, 100, 1, 0, false, &gload_alarm_volume  );
	gloadalvo->setHelp( "Maximum volume of G-Load alarm audio warning");
	top->addEntry( gloadalvo );

	SetupMenuSelect * gloadres = new SetupMenuSelect( "G-Load reset", RST_NONE, gload_reset, false, 0 );
	gloadres->setHelp("Option to reset stored maximum positive and negative G-load values");
	gloadres->addEntry( "Reset");
	gloadres->addEntry( "Cancel");
	top->addEntry( gloadres );
}

void SetupMenu::options_menu_create_stallwarning( MenuEntry *top ){

	SetupMenuSelect * stawaen = new SetupMenuSelect( "Stall Warning", RST_NONE, 0, false, &stall_warning );
	top->addEntry( stawaen );
	stawaen->setHelp( "Enable alarm sound when speed goes below configured stall speed (until 30% less)");
	stawaen->addEntry( "Disable");
	stawaen->addEntry( "Enable");

	SetupMenuValFloat * staspe = new SetupMenuValFloat( "Stall Speed", "", 20, 200, 1, 0, true, &stall_speed, RST_ON_EXIT  );
	staspe->setHelp("Configure stalling speed for corresponding aircraft type and reboot (reboots)");
	top->addEntry( staspe );
}

void SetupMenu::options_menu_create_altimeter_airspeed( MenuEntry *top ){

	SetupMenuSelect * atl = new SetupMenuSelect( "Auto Transition",	RST_NONE, 0, true, &fl_auto_transition );
	top->addEntry( atl );
	atl->setHelp( "Option to enable automatic altitude transition to QNH Standard (1013.25) above 'Transition Altitude'");
	atl->addEntry( "Disable");
	atl->addEntry( "Enable");

	SetupMenuSelect * altDisplayMode = new SetupMenuSelect( "Altitude Mode", RST_NONE, 0, true, &alt_display_mode );
	top->addEntry( altDisplayMode );
	altDisplayMode->setHelp( "Select altitude display mode");
	altDisplayMode->addEntry( "QNH");
	altDisplayMode->addEntry( "QFE");

	SetupMenuValFloat * tral = new SetupMenuValFloat( "Transition Altitude", "FL", 0, 400, 10, 0, false, &transition_alt  );
	tral->setHelp("Transition altitude (or transition height, when using QFE) is the altitude/height above which standard pressure (QNE) is set (1013.2 mb/hPa)", 100 );
	top->addEntry( tral );

	SetupMenuSelect * amode = new SetupMenuSelect( "Airspeed Mode",	RST_NONE, 0, true, &airspeed_mode );
	top->addEntry( amode );
	amode->setHelp( "Select mode of Airspeed indicator to display IAS (Indicated AirSpeed), TAS (True AirSpeed) or CAS (calibrated airspeed)", 180 );
	amode->addEntry( "IAS");
	amode->addEntry( "TAS");
	amode->addEntry( "CAS");
	amode->addEntry( "Slip Angle");

	SetupMenu * stallwa = new SetupMenu( "Stall Warning");
	top->addEntry( stallwa );
	stallwa->setHelp( "Configure stall warning parameters");
	stallwa->addCreator( options_menu_create_stallwarning );

	SetupMenuValFloat * vmax = new SetupMenuValFloat( "Maximum Speed", "", 70, 450, 1, 0, false, &v_max  );
	vmax->setHelp("Configure maximum speed for corresponding aircraft type");
	top->addEntry( vmax );
}

// this submenu will eventually have additional horizon screen options, such as:
// colors, size of airplane icon, whether to display pitch and bank ticks and/or numbers...
// - maybe also a default (boot-time) value for the horizon_offset.
void SetupMenu::options_menu_create_horizon_screen( MenuEntry *top ){

	// this should be here, not in hardware/rotary
	SetupMenuSelect * horizon = new SetupMenuSelect( "Horizon Screen", RST_NONE, upd_screens, true, &screen_horizon );
	horizon->addEntry( "Disable");
	if( gflags.ahrsKeyValid ) {
		horizon->addEntry( "Enable");
	} else {
		horizon->addEntry( "Static Demo");
		horizon->setHelp( "License key required for the real horizon screen");
	}
	top->addEntry(horizon);

	SetupMenuSelect * colors = new SetupMenuSelect( "Colors", RST_NONE, 0, true, &horizon_colors );
	colors->addEntry( "White on Dark");
	colors->addEntry( "Black on Bright");
	colors->addEntry( "White on Bright");
	colors->addEntry( "White on Black");
	horizon->setHelp( "Color scheme for the horizon screen");
	top->addEntry(colors);

	SetupMenuSelect * icon = new SetupMenuSelect( "Airplane Icon", RST_NONE, 0, true, &horizon_largeicon );
	icon->addEntry( "Small");
	icon->addEntry( "Large");
	top->addEntry(icon);

	SetupMenuValFloat * offset = new SetupMenuValFloat( "Horizon Pitch Offset", "°", -10, 10, 0.5, 0, false, &horizon_offset );
	offset->setPrecision( 1 );
	offset->setHelp("Move displayed horizon up/down (also via rotary). Returns to zero on reboot. (Not related to autozero)");
	top->addEntry( offset );
}

void SetupMenu::options_menu_create_advanced( MenuEntry *top ){

	SetupMenu * windMenu = new SetupMenu( "Wind" );
	top->addEntry( windMenu );
	windMenu->setHelp( "Setup Wind Estimation", 280 );
	windMenu->addCreator(options_menu_create_wind);

	SetupMenu * alt_as = new SetupMenu( "Altimeter, Airspeed" );
	top->addEntry( alt_as );
	alt_as->addCreator(options_menu_create_altimeter_airspeed);

	SetupMenu * display = new SetupMenu( "Display Setup" );
	top->addEntry( display );
	display->addCreator( options_menu_create_display );

	SetupMenu * rotary = new SetupMenu( "Rotary Setup" );
	top->addEntry( rotary );
	rotary->addCreator(options_menu_create_rotary);

	if( student_mode.get() == 0 ) {
		SetupMenuSelect *stumo  = new SetupMenuSelect( "Student Mode", RST_ON_EXIT, 0, true, &student_mode );
		top->addEntry( stumo );
		stumo->setHelp( "Student mode, disables all sophisticated setup to just basic pre-flight related items like MC, ballast or bugs  (reboots)");
		stumo->addEntry( "Disable");
		stumo->addEntry( "Enable");
	}
}

void SetupMenu::options_menu_create( MenuEntry *top ){

	// Units
	SetupMenu * un = new SetupMenu( "Units" );
	top->addEntry( un );
	un->setHelp( "Setup altimeter, airspeed indicator and variometer with European Metric, American, British or Australian units", 205);
	un->addCreator(options_menu_create_units);

	// Vario
	SetupMenu * va = new SetupMenu( "Vario and Speed 2 Fly" );
	top->addEntry( va );
	va->addCreator( vario_menu_create );

	SetupMenu * flarm = new SetupMenu( "FLARM" );
	top->addEntry( flarm );
	flarm->setHelp( "Option to display or sound warnings depending on FLARM alarm level", 240);
	flarm->addCreator(options_menu_create_flarm);

	SetupMenu * gload = new SetupMenu( "G-Load Display" );
	top->addEntry( gload );
	gload->addCreator(options_menu_create_gload);

	SetupMenu * horizon_screen = new SetupMenu( "Horizon Display");
	top->addEntry( horizon_screen );
	horizon_screen->setHelp("Options regarding the horizon screen");
	horizon_screen->addCreator( options_menu_create_horizon_screen );

	// Advanced Options submenu
	SetupMenu * advopt = new SetupMenu( "Advanced Options" );
	top->addEntry( advopt );
	advopt->addCreator(options_menu_create_advanced);
}

void SetupMenu::system_menu_create_software( MenuEntry *top ){
	Version V;
	SetupMenuSelect * ver = new SetupMenuSelect( "Software Vers.", RST_NONE, 0, false );
	ver->addEntry( V.version() );
	top->addEntry( ver );

	SetupMenuSelect * upd = new SetupMenuSelect( "Software Update", RST_IMMEDIATE, 0, true, &software_update );
	upd->setHelp("Software Update over the air (OTA). Starts Wifi AP 'ESP32 OTA' - connect and open http://192.168.4.1 in browser");
	upd->addEntry( "Cancel");
	upd->addEntry( "Start");
	top->addEntry( upd );

#if defined(SUNTON28)
	SetupMenuSelect * rbt = new SetupMenuSelect( "Reboot", RST_IMMEDIATE, 0, true, &reboot );
	rbt->addEntry( "Cancel");
	rbt->addEntry( "Go Ahead");
	top->addEntry( rbt );
#endif
}

void SetupMenu::system_menu_create_battery( MenuEntry *top ){
	SetupMenuValFloat * blow = new SetupMenuValFloat( "Battery Low", "Volt ", 0.0, 28.0, 0.1, 0, false, &bat_low_volt );
	SetupMenuValFloat * bred = new SetupMenuValFloat( "Battery Red", "Volt ", 0.0, 28.0, 0.1, 0, false, &bat_red_volt  );
	SetupMenuValFloat * byellow = new SetupMenuValFloat( "Battery Yellow", "Volt ", 0.0, 28.0, 0.1, 0, false, &bat_yellow_volt );
	SetupMenuValFloat * bfull = new SetupMenuValFloat( "Battery Full", "Volt ", 0.0, 28.0, 0.1, 0, false, &bat_full_volt  );

	SetupMenuSelect * batv = new SetupMenuSelect( "Battery Display", RST_NONE, 0, true, &battery_display  );
	batv->setHelp("Option to display battery charge state either in Percentage e.g. 75% or Voltage e.g. 12.5V");
	batv->addEntry( "Percentage");
	batv->addEntry( "Voltage");
	batv->addEntry( "Voltage Big");

	top->addEntry(blow);
	top->addEntry(bred);
	top->addEntry(byellow);
	top->addEntry(bfull);
	top->addEntry( batv );

#if defined(NOSENSORS)
	ESP_LOGI(FNAME,"create meter_adj in hw_menu");
#endif
	SetupMenuValFloat * met_adj = new SetupMenuValFloat( "Voltmeter Adjust", "%",	-25.0, 25.0, 0.01, factv_adj, false, &factory_volt_adjust,  RST_NONE, false, true);
	met_adj->setHelp("Option to factory factory fine-adjust voltmeter");
	top->addEntry( met_adj );
}

void SetupMenu::options_menu_create_display( MenuEntry *top ){

	SetupMenuSelect * disty = new SetupMenuSelect( "Style", RST_NONE , 0, false, &display_style );
	top->addEntry( disty );
	disty->setHelp( "Display style in more digital airliner stype or retro mode with classic vario meter needle");
	disty->addEntry( "Airliner");
	disty->addEntry( "Retro");
	disty->addEntry( "UL");

	SetupMenuSelect * disva = new SetupMenuSelect( "Color Variant", RST_NONE , 0, false, &display_variant );
	top->addEntry( disva );
	disva->setHelp( "Display variant white on black (W/B) or black on white (B/W)");
	disva->addEntry( "W/B");
	disva->addEntry( "B/W");

	// Orientation   _display_orientation
	SetupMenuSelect * diso = new SetupMenuSelect( "Orientation", RST_ON_EXIT, 0, true, &display_orientation );
	top->addEntry( diso );
#if defined(SUNTON28)
	diso->setHelp( "Display Orientation.  NORMAL means USB jack at bottom, TOPDOWN means USB at top (reboots)");
#else
	diso->setHelp( "Display Orientation.  NORMAL means Rotary on right, TOPDOWN means Rotary on left  (reboots)");
#endif
	diso->addEntry( "NORMAL");
	diso->addEntry( "TOPDOWN");

	//
	SetupMenuSelect * drawp = new SetupMenuSelect( "Needle Alignment", RST_NONE, 0, true, &drawing_prio );
	top->addEntry( drawp );
	drawp->setHelp( "Alignment of the variometer needle either in front of the displayed info, or in background");
	drawp->addEntry( "Front");
	drawp->addEntry( "Back");

	// UNIVERSAL, RAYSTAR_RFJ240L_40P, ST7789_2INCH_12P, ILI9341_TFT_18P
	SetupMenuSelect * dtype = new SetupMenuSelect( 	"HW Type", RST_NONE, 0, true, &display_type );
	dtype->setHelp( "Factory setup for corresponding display type used");
	dtype->addEntry( "UNIVERSAL");
	dtype->addEntry( "RAYSTAR");
	dtype->addEntry( "ST7789");
	dtype->addEntry( "ILI9341");
	top->addEntry( dtype );

	SetupMenuSelect * dtest = new SetupMenuSelect( "Display Test", RST_NONE, do_display_test, true, &display_test );
	top->addEntry( dtest );
	dtest->setHelp( "Start display test screens, press rotary to cancel");
	dtest->addEntry( "Cancel");
	dtest->addEntry( "Start Test");
}

void SetupMenu::options_menu_create_rotary( MenuEntry *top ){

	SetupMenuSelect * rotype;
	if( hardwareRevision.get() < XCVARIO_21 )
		rotype = new SetupMenuSelect( "Direction", RST_NONE , 0, false, &rotary_dir );
	else
		rotype = new SetupMenuSelect( "Direction", RST_NONE , 0, false, &rotary_dir_21 );
	top->addEntry( rotype );
	rotype->setHelp( "Select type of rotary switch, different brands may need adjustment");
	rotype->addEntry( "Clockwise");
	rotype->addEntry( "Counterclockwise");

	SetupMenuSelect * roinc = new SetupMenuSelect( "Sensitivity", RST_NONE , 0, false, &rotary_inc );
	top->addEntry( roinc );
	roinc->setHelp( "Select rotary sensitivity in number of Indent's for one increment, to your personal preference, 1 Indent is most sensitive");
	roinc->addEntry( "1 Indent");
	roinc->addEntry( "2 Indent");
	roinc->addEntry( "3 Indent");
	roinc->addEntry( "4 Indent");

	// Rotary Default
	SetupMenuSelect * rd = new SetupMenuSelect( "Rotation", RST_ON_EXIT, 0, true, &rot_default );
	top->addEntry( rd );
	rd->setHelp("Select value to be altered at rotary movement outside of setup menu (reboots)");
	rd->addEntry( "Volume");
	rd->addEntry( "MC Value");

	SetupMenuSelect * sact = new SetupMenuSelect( "Setup Menu by", RST_NONE, 0, true, &menu_long_press );
	top->addEntry( sact);
	sact->setHelp("Select Mode to activate setup menu either by short press or long press > 0.4 seconds");
	sact->addEntry( "Short Press");
	sact->addEntry( "Long Press");
}


static const char lkeys[][4] { "0","1","2","3","4","5","6","7","8","9",":",";","<","=",">","?","@","A","B","C","D","E","F","G","H","I","J","K","L","M","N","O","P","Q","R","S","T","U","V","W","X","Y","Z"};

void SetupMenu::system_menu_create_ahrs_lc( MenuEntry *top ){
	SetupMenuSelect * ahrslc1 = new SetupMenuSelect( "First    Letter",	RST_NONE, add_key, false, &ahrs_licence_dig1 );
	SetupMenuSelect * ahrslc2 = new SetupMenuSelect( "Second Letter",	RST_NONE, add_key, false, &ahrs_licence_dig2 );
	SetupMenuSelect * ahrslc3 = new SetupMenuSelect( "Third   Letter",	RST_NONE, add_key, false, &ahrs_licence_dig3 );
	SetupMenuSelect * ahrslc4 = new SetupMenuSelect( "Last     Letter",	RST_NONE, add_key, false, &ahrs_licence_dig4 );
	top->addEntry( ahrslc1 );
	top->addEntry( ahrslc2 );
	top->addEntry( ahrslc3 );
	top->addEntry( ahrslc4 );
	ahrslc1->addEntryList( lkeys, sizeof(lkeys)/4 );
	ahrslc2->addEntryList( lkeys, sizeof(lkeys)/4 );
	ahrslc3->addEntryList( lkeys, sizeof(lkeys)/4 );
	ahrslc4->addEntryList( lkeys, sizeof(lkeys)/4 );
}


void SetupMenu::system_menu_create_ahrs_parameter( MenuEntry *top ){
	SetupMenuValFloat * ahrsgf = new SetupMenuValFloat( "Gyro Max Trust", "x", 0, 100, 1, 0, false, &ahrs_gyro_factor  );
	ahrsgf->setPrecision( 0 );
	ahrsgf->setHelp("Maximum Gyro trust factor in artifical horizon");
	top->addEntry( ahrsgf );

	SetupMenuValFloat * ahrsgfm = new SetupMenuValFloat( "Gyro Min Trust", "x", 0, 100, 1, 0, false, &ahrs_min_gyro_factor  );
	ahrsgfm->setPrecision( 0 );
	ahrsgfm->setHelp("Minimum Gyro trust factor in artifical horizon");
	top->addEntry( ahrsgfm );

	SetupMenuValFloat * ahrsdgf = new SetupMenuValFloat( "Gyro Dynamics", "", 0.5, 10, 0.1, 0, false, &ahrs_dynamic_factor  );
	ahrsdgf->setHelp("Gyro dynamics factor, higher value trusts gyro more when load factor is different from one");
	top->addEntry( ahrsdgf );

	SetupMenuValFloat * gyrog = new SetupMenuValFloat( "Gyro Gating", "°", 0, 10, 0.1, 0, false, &gyro_gating  );
	gyrog->setHelp( "Minimum accepted gyro rate in degree per second");
	top->addEntry( gyrog );

	SetupMenuValFloat * gyrocal = new SetupMenuValFloat( "Gyro Calibration", "", -0.5, 1.5, 0.01, 0, false, &ahrs_gyro_cal  );
	gyrocal->setHelp( "Gyro calibration factor to increase accuracy of gyro in %/100");
	top->addEntry( gyrocal );

}

void SetupMenu::system_menu_create_ahrs( MenuEntry *top ){

	SetupMenuSelect * ahrsid = new SetupMenuSelect( "AHRS ID", RST_NONE, 0, false );
	ahrsid->addEntry( Cipher::id() );
	top->addEntry( ahrsid );

	mpu = new SetupMenuSelect( "AHRS Option", RST_ON_EXIT , 0, true, &attitude_indicator );
	top->addEntry( mpu );
	mpu->setHelp( "Enable High Accuracy Attitude Sensor (AHRS) NMEA messages (need valid license key entered, reboots)");
	mpu->addEntry( "Disable");
	if( gflags.ahrsKeyValid )
		mpu->addEntry( "Enable");

	SetupMenuSelect * ahrsaz = new SetupMenuSelect( "AHRS Autozero", RST_IMMEDIATE , 0, true, &ahrs_autozero );
	top->addEntry( ahrsaz );
	ahrsaz->setHelp( "Start Autozero of AHRS Sensor; Preconditions: On ground; Wings 100% horizontal, fuselage in flight position! (reboots)");
	ahrsaz->addEntry( "Cancel");
	ahrsaz->addEntry( "Start");

	SetupMenu * ahrslc = new SetupMenu( "AHRS License Key" );
	ahrslc->setHelp( "Enter valid AHRS License Key, then AHRS feature can be enabled under 'AHRS Option'");
	top->addEntry( ahrslc );
	ahrslc->addCreator( system_menu_create_ahrs_lc );

	SetupMenu * ahrspa = new SetupMenu( "AHRS Parameters" );
	ahrspa->setHelp( "AHRS constants such as gyro trust and filtering", 275 );
	top->addEntry( ahrspa );
	ahrspa->addCreator( system_menu_create_ahrs_parameter );

	SetupMenuSelect * ahrsdef = new SetupMenuSelect( "AHRS Defaults", RST_NONE, 0, true, &ahrs_defaults );
	top->addEntry( ahrsdef );
	ahrsdef->setHelp( "Set optimum default values for all AHRS Parameters as determined to the best practice");
	ahrsdef->addEntry( "Cancel");
	ahrsdef->addEntry( "Set Defaults");

	SetupMenuSelect * rpyl = new SetupMenuSelect( "AHRS RPYL", RST_NONE , 0, true, &ahrs_rpyl_dataset );
	top->addEntry( rpyl );
	rpyl->setHelp( "Send LEVIL AHRS like $RPYL sentence for artifical horizon");
	rpyl->addEntry( "Disable");
	rpyl->addEntry( "Enable");

	SetupMenuValFloat * tcontrol = new SetupMenuValFloat( "AHRS Temp Control", "", -1, 60, 1, 0, false, &mpu_temperature  );
	tcontrol->setPrecision( 0 );
	tcontrol->setHelp( "Regulated target temperature of AHRS silicon chip, if supported in hardware (model > 2023), -1 means OFF");
	top->addEntry( tcontrol );
}

void SetupMenu::system_menu_create_hardware( MenuEntry *top ){

	SetupMenuSelect * pstype = new SetupMenuSelect( "AS Sensor type", RST_ON_EXIT, 0, false, &airspeed_sensor_type );
	top->addEntry( pstype );
	pstype->setHelp( "Factory default for type of pressure sensor, will not erase on factory reset (reboots)");
	pstype->addEntry( "ABPMRR");
	pstype->addEntry( "TE4525");
	pstype->addEntry( "MP5004");
	pstype->addEntry( "Autodetect");

	Flap::setupMenue( top );

	SetupMenuSelect * gear = new SetupMenuSelect( "Gear Warn", RST_NONE , config_gear_warning, false, &gear_warning );
	top->addEntry( gear );
	gear->setHelp( "Enable gear warning on S2 flap sensor or serial RS232 pin (pos. or neg. signal) or by external command", 220 );
	gear->addEntry( "Disable");
	gear->addEntry( "S2 Flap positive");   // A positive signal, high signal or > 2V will start alarm
	gear->addEntry( "S2 RS232 positive");
	gear->addEntry( "S2 Flap negative");   // A negative signal, low signal or < 1V will start alarm
	gear->addEntry( "S2 RS232 negative");
	gear->addEntry( "External");           // A $g,w<n>*CS command from an external device

	SetupMenu * compassMenu = new SetupMenu( "Compass" );
	top->addEntry( compassMenu );
	compassMenu->addCreator( system_menu_create_compass );

	if( hardwareRevision.get() >= XCVARIO_21 ){
		SetupMenu * ahrs = new SetupMenu( "AHRS Setup" );
		top->addEntry( ahrs );
		ahrs->addCreator( system_menu_create_ahrs );
	}

	SetupMenu * bat = new SetupMenu( "Battery Setup" );
	bat->setHelp( "Adjust voltages for battery symbol display low,red,yellow and full", 260);
	top->addEntry( bat );
	bat->addCreator(system_menu_create_battery);
}

void SetupMenu::system_menu_create_altimeter_airspeed( MenuEntry *top ){
	SetupMenuSelect * als = new SetupMenuSelect( "Altimeter Source", RST_NONE, 0, true, &alt_select );
	top->addEntry( als );
	als->setHelp( "Select source for barometric altitude, either TE sensor or Baro sensor (recommended) or an external source e.g. FLARM (if avail)");
	als->addEntry( "TE Sensor");
	als->addEntry( "Baro Sensor");
	als->addEntry( "External");

	SetupMenuValFloat * spc = new SetupMenuValFloat( "AS Calibration", "%", -100, 100, 1, speedcal_change, false, &speedcal  );
	spc->setHelp("Calibration of airspeed sensor (AS). Normally not needed, unless pressure probes have systematic error");
	top->addEntry( spc );

	SetupMenuSelect * auze = new SetupMenuSelect( "AutoZero AS Sensor",	RST_IMMEDIATE, 0, true, &autozero );
	top->addEntry( auze );
	auze->setHelp( "Recalculate zero point for airspeed sensor on next power on");
	auze->addEntry( "Cancel");
	auze->addEntry( "Start");

	SetupMenuSelect * alq = new SetupMenuSelect( "Alt. Quantization", RST_NONE,  0, true, &alt_quantization );
	alq->setHelp( "Set altimeter mode with discrete steps and rolling last digits");
	alq->addEntry( "Disable");
	alq->addEntry( "2");
	alq->addEntry( "5");
	alq->addEntry( "10");
	alq->addEntry( "20");
	top->addEntry( alq );
}

void SetupMenu::system_menu_create_interfaceS1_routing( MenuEntry *top ){
	SetupMenuSelect * s1outxcv = new SetupMenuSelect( "XCVario", RST_ON_EXIT, 0, true, &rt_s1_xcv );
	s1outxcv->addEntry( "Disable");
	s1outxcv->addEntry( "Enable");
	s1outxcv->setHelp( "Whether vario data is routed to/from serial port S1");
	top->addEntry( s1outxcv );

	SetupMenuSelect * s1outwl = new SetupMenuSelect( "Wireless", RST_ON_EXIT, 0, true, &rt_s1_wl );
	s1outwl->addEntry( "Disable");
	s1outwl->addEntry( "Enable");
	s1outwl->setHelp( "Whether wireless (BT or WiFi port 8881) is routed to/from serial port S1");
	top->addEntry( s1outwl );

	SetupMenuSelect * s1outw0 = new SetupMenuSelect( "WiFi 8880", RST_ON_EXIT, 0, true, &rt_s1_w0 );
	s1outw0->addEntry( "Disable");
	s1outw0->addEntry( "Enable");
	s1outw0->setHelp( "Whether WiFi port 8880 is routed to/from serial port S1");
	top->addEntry( s1outw0 );

	if( hardwareRevision.get() >= XCVARIO_21 ) {
		SetupMenuSelect * s1outs2 = new SetupMenuSelect( "RS232 S2", RST_ON_EXIT, 0, true, &rt_s1_s2 );
		s1outs2->addEntry( "Disable");
		s1outs2->addEntry( "Enable");
		s1outs2->setHelp( "Whether serial port S2 data is routed to/from serial port S1");
		top->addEntry( s1outs2 );
	}
}

void SetupMenu::system_menu_create_interfaceS1( MenuEntry *top ){
	SetupMenuSelect * s2sp = new SetupMenuSelect( "Baudrate", RST_ON_EXIT, 0, true, &serial1_speed );
	top->addEntry( s2sp );
	s2sp->addEntry( "OFF");
	s2sp->addEntry( "4800 baud");
	s2sp->addEntry( "9600 baud");
	s2sp->addEntry( "19200 baud");
	s2sp->addEntry( "38400 baud");
	s2sp->addEntry( "57600 baud");
	s2sp->addEntry( "115200 baud");

	SetupMenuSelect * s1in = new SetupMenuSelect( "Serial Loops", RST_NONE, 0, true, &serial1_rxloop );
	top->addEntry( s1in );
	s1in->setHelp( "Option to loop serial S1 RX to S1 TX, e.g. for unidirectional OV or Kobo connection" );
	s1in->addEntry( "Disable");     // 0
	s1in->addEntry( "Enable");      // 1

	SetupMenuSelect * stxi = new SetupMenuSelect( "TX Inversion", RST_ON_EXIT , 0, true, &serial1_tx_inverted );
	top->addEntry( stxi );
	stxi->setHelp( "Serial RS232 (TTL) logic, a '1' will be sent as zero voltage level (RS232 standard and default) or vice versa  (reboots)");
	stxi->addEntry( "Normal");
	stxi->addEntry( "Inverted");

	SetupMenuSelect * srxi = new SetupMenuSelect( "RX Inversion", RST_ON_EXIT, 0, true, &serial1_rx_inverted );
	top->addEntry( srxi );
	srxi->setHelp( "Serial RS232 (TTL) logic, a '1' will be received at zero voltage level (RS232 standard and default) or vice versa  (reboots)");
	srxi->addEntry( "Normal");
	srxi->addEntry( "Inverted");

	SetupMenuSelect * srxtw1 = new SetupMenuSelect( "RX/TX Pins", RST_ON_EXIT, 0, true, &serial1_pins_twisted );
	top->addEntry( srxtw1 );
	srxtw1->setHelp( "Option to swap RX and TX line for S1, e.g. for OpenVario. After change also a true power-cycle is needed  (reboots)");
	srxtw1->addEntry( "Normal");
	srxtw1->addEntry( "Swapped");

	SetupMenuSelect * stxdis1 = new SetupMenuSelect( "TX Line", RST_ON_EXIT, 0, true, &serial1_tx_enable );
	top->addEntry( stxdis1 );
	stxdis1->setHelp( "Option to switch off RS232 TX line in case active sending is not required, e.g. for multiple devices connected to one device  (reboots)");
	stxdis1->addEntry( "Disable");
	stxdis1->addEntry( "Enable");

#if defined(SUNTON28)
	SetupMenuSelect * i2cpins = new SetupMenuSelect( "Pins", RST_ON_EXIT, 0, true, &i2c_pins );
	top->addEntry( i2cpins );
	i2cpins->setHelp( "S1 RX=35; I2C SDA=22; SCL=27 disables S1 TX, SCL=21 blinks TFT (reboots)");
	i2cpins->addEntry( "TX=22, No I2C");    // 0
	i2cpins->addEntry( "No TX, SCL=27");    // 1
	i2cpins->addEntry( "TX=27, SCL=21");    // 2
#endif
}

void SetupMenu::system_menu_create_interfaceS2_routing( MenuEntry *top ){

	SetupMenuSelect * s2outxcv = new SetupMenuSelect( "XCVario", RST_ON_EXIT, 0, true, &rt_s2_xcv );
	s2outxcv->addEntry( "Disable");
	s2outxcv->addEntry( "Enable");
	s2outxcv->setHelp( "Whether vario data is routed to/from serial port S2");
	top->addEntry( s2outxcv );

	SetupMenuSelect * s2outwl = new SetupMenuSelect( "Wireless", RST_ON_EXIT, 0, true, &rt_s2_wl );
	s2outwl->addEntry( "Disable");
	s2outwl->addEntry( "Enable");
	s2outwl->setHelp( "Whether wireless (BT or WiFi port 8882) is routed to/from serial port S2");
	top->addEntry( s2outwl );

	SetupMenuSelect * s2outw0 = new SetupMenuSelect( "Wifi 8880", RST_ON_EXIT, 0, true, &rt_s2_w0 );
	s2outw0->addEntry( "Disable");
	s2outw0->addEntry( "Enable");
	s2outw0->setHelp( "Whether WiFi port 8880 is routed to/from serial port S2");
	top->addEntry( s2outw0 );

	SetupMenuSelect * s2outw1 = new SetupMenuSelect( "Wifi 8881", RST_ON_EXIT, 0, true, &rt_s2_w1 );
	s2outw1->addEntry( "Disable");
	s2outw1->addEntry( "Enable");
	s2outw1->setHelp( "Whether WiFi port 8881 is routed to/from serial port S2");
	top->addEntry( s2outw1 );
}

void SetupMenu::system_menu_create_interfaceS2( MenuEntry *top ){
	SetupMenuSelect * s2sp2 = new SetupMenuSelect( "Baudrate", RST_ON_EXIT, 0, true, &serial2_speed );
	top->addEntry( s2sp2 );
	// s2sp->setHelp( "Serial RS232 (TTL) speed, pins RX:2, TX:3 on external RJ45 connector");
	s2sp2->addEntry( "OFF");
	s2sp2->addEntry( "4800 baud");
	s2sp2->addEntry( "9600 baud");
	s2sp2->addEntry( "19200 baud");
	s2sp2->addEntry( "38400 baud");
	s2sp2->addEntry( "57600 baud");
	s2sp2->addEntry( "115200 baud");

	SetupMenuSelect * stxi2 = new SetupMenuSelect( "TX Inversion", RST_ON_EXIT , 0, true, &serial2_tx_inverted );
	top->addEntry( stxi2 );
	stxi2->setHelp( "Serial RS232 (TTL) logic, a '1' will be sent as zero voltage level (RS232 standard and default) or vice versa  (reboots)");
	stxi2->addEntry( "Normal");
	stxi2->addEntry( "Inverted");

	SetupMenuSelect * srxi2 = new SetupMenuSelect( "RX Inversion", RST_ON_EXIT, 0, true, &serial2_rx_inverted );
	top->addEntry( srxi2 );
	srxi2->setHelp( "Serial RS232 (TTL) logic, a '1' will be received at zero voltage level (RS232 standard and default) or vice versa (reboots)");
	srxi2->addEntry( "Normal");
	srxi2->addEntry( "Inverted");

	SetupMenuSelect * srxtw2 = new SetupMenuSelect( "RX/TX Pins", RST_ON_EXIT, 0, true, &serial2_pins_twisted );
	top->addEntry( srxtw2 );
	srxtw2->setHelp( "Option to swap RX and TX line for S2, e.g. for OpenVario. After change also a true power-cycle is needed (reboots)");
	srxtw2->addEntry( "Normal");
	srxtw2->addEntry( "Swapped");

	SetupMenuSelect * stxdis2 = new SetupMenuSelect( "TX Line", RST_ON_EXIT, 0, true, &serial2_tx_enable );
	top->addEntry( stxdis2 );
	stxdis2->setHelp( "Option to switch off RS232 TX line in case active sending is not required, e.g. for multiple devices connected to one device (reboots)");
	stxdis2->addEntry( "Disable");
	stxdis2->addEntry( "Enable");
}

void SetupMenu::system_menu_create_interfaceCAN_routing( MenuEntry *top ){

	SetupMenuSelect * canoutxcv = new SetupMenuSelect( "XCVario", RST_ON_EXIT, 0, true, &rt_can_xcv );
	canoutxcv->addEntry( "Disable");
	canoutxcv->addEntry( "Enable");
	canoutxcv->setHelp( "Whether vario data is routed to/from the CAN bus");
	top->addEntry( canoutxcv );

	SetupMenuSelect * canoutwl = new SetupMenuSelect( "Wireless", RST_ON_EXIT, 0, true, &rt_wl_can );
	canoutwl->addEntry( "Disable");
	canoutwl->addEntry( "Enable");
	canoutwl->setHelp( "Whether wireless data (BT or WiFi port 8880) is routed to/from the CAN bus");
	top->addEntry( canoutwl );

	SetupMenuSelect * canouts1 = new SetupMenuSelect( "RS232 S1", RST_ON_EXIT, 0, true, &rt_s1_can );
	canouts1->addEntry( "Disable");
	canouts1->addEntry( "Enable");
	canouts1->setHelp( "Whether serial port S1 data is routed to/from the CAN bus");
	top->addEntry( canouts1 );

	SetupMenuSelect * canouts2 = new SetupMenuSelect( "RS232 S2", RST_ON_EXIT, 0, true, &rt_s2_can );
	canouts2->addEntry( "Disable");
	canouts2->addEntry( "Enable");
	canouts2->setHelp( "Whether serial port S2 data is routed to/from the CAN bus");
	top->addEntry( canouts2 );
}

void SetupMenu::system_menu_create_comm_wireless( MenuEntry *top ){
//#if 0
// better to avoid mixing master/client and wireless type:
	SetupMenuSelectCodes * btm = new SetupMenuSelectCodes( "Wireless", RST_ON_EXIT, 0, true, &wireless_type );
	btm->addEntryCode( "Disable", WL_DISABLE);                   // 0
	btm->addEntryCode( "Bluetooth", WL_BLUETOOTH);               // 1
	btm->addEntryCode( "Bluetooth LE", WL_BLUETOOTH_LE);         // 5
	btm->addEntryCode( "WiFi (Standalone)", WL_WLAN_STANDALONE); // 4
	btm->addEntryCode( "WiFi (Master)", WL_WLAN_MASTER);         // 2
	btm->addEntryCode( "WiFi (Client)", WL_WLAN_CLIENT);         // 3
	btm->setHelp( "Activate wireless interface type to connect navigation devices or another XCvario. (Reboots)", 220 );
	top->addEntry( btm );
//#else

// The selected entry will be internally translated into wireless_type (see SetupNG.cpp).
	SetupMenuSelect * wlm = new SetupMenuSelect( "Wireless", RST_ON_EXIT, 0, true, &wireless_mode );
	wlm->addEntry( "Disable");
	wlm->addEntry( "Bluetooth");
	wlm->addEntry( "Bluetooth LE");
	wlm->addEntry( "WiFi");
	wlm->setHelp( "Activate wireless interface type to connect navigation devices or another XCvario. (Reboots)", 220 );
	top->addEntry( wlm );

//#endif

	SetupMenuValFloat *wifip = new SetupMenuValFloat( "WIFI Power", "%", 10.0, 100.0, 5.0, update_wifi_power, false, &wifi_max_power );
	wifip->setPrecision(0);
	top->addEntry( wifip );
	wifip->setHelp("Maximum Wifi Power to be used 10..100% or 2..20dBm");

	SetupMenuSelect * wifimal = new SetupMenuSelect( "Lock Master", RST_NONE, 0, true, &master_xcvario_lock );
	wifimal->setHelp( "In wireless client role, lock this client to the scanned master XCVario ID above");
	wifimal->addEntry( "Unlock");
	wifimal->addEntry( "Lock");
	top->addEntry( wifimal );

	SetupMenu * cusid = new SetupMenu( "Custom-ID" );
	top->addEntry( cusid );
	cusid->setHelp( "Select custom ID (SSID) for wireless BT (or WIFI) interface, e.g. D-1234. Restart device to activate", 215);
	cusid->addCreator( options_menu_create_wireless_custom_id );

	SetupMenuSelect * w3txdis = new SetupMenuSelect( "Port 2000 TX", RST_NONE, 0, true, &w3_tx_enable );
	top->addEntry( w3txdis );
	w3txdis->setHelp( "Disable transmission (TX) on WiFi Port 2000 if not required (for devices that are output-only)");
	w3txdis->addEntry( "Disable");
	w3txdis->addEntry( "Enable");
}

void SetupMenu::system_menu_create_comm_wired( MenuEntry *top ){

	SetupMenu * rs232 = new SetupMenu( "RS232 Interface S1" );
	top->addEntry( rs232 );
	rs232->setHelp( "Configure serial interface S1 (reboots)", 240);
	rs232->addCreator(system_menu_create_interfaceS1);

	if( hardwareRevision.get() >= XCVARIO_21 ) {
		SetupMenu * rs232_2 = new SetupMenu( "RS232 Interface S2" );
		top->addEntry( rs232_2 );
		rs232_2->setHelp( "Configure serial interface S2 (reboots)");
		rs232_2->addCreator(system_menu_create_interfaceS2);
	}

	if( hardwareRevision.get() >= XCVARIO_22 ){
//#if 0
// hide this menu, use the master mode menu instead
		SetupMenuSelect * devmod = new SetupMenuSelect( "CAN Mode", RST_ON_EXIT , 0, false, &can_mode );
		top->addEntry( devmod );
		if (can_speed.get() == CAN_SPEED_OFF)
			devmod->setHelp( "Warning: CAN speed was set to 'off'");
		else
			devmod->setHelp( "Select 'Standalone' for single seater, 'Master' in front, 'Client' for secondary device in rear (reboots)");
		devmod->addEntry( "Master");
		devmod->addEntry( "Client");
		devmod->addEntry( "Standalone");
//#endif
		// Can Interface C1
		SetupMenuSelect * canmode = new SetupMenuSelect( "CAN speed", RST_ON_EXIT, 0, true, &can_speed );
		top->addEntry( canmode );
		canmode->setHelp( "Datarate on high speed serial CAN interace in kbit per second (reboots)");
		canmode->addEntry( "CAN OFF");
		canmode->addEntry( "250 kbit");
		canmode->addEntry( "500 kbit");
		canmode->addEntry( "1000 kbit");
	}
}

void SetupMenu::system_menu_create_interfaceW3_routing( MenuEntry *top ){

	SetupMenuSelect * w3outxcv = new SetupMenuSelect( "XCVario", RST_ON_EXIT, 0, true, &rt_w3_xcv );
	w3outxcv->addEntry( "Disable");
	w3outxcv->addEntry( "Enable");
	w3outxcv->setHelp("Send vario data to/from WiFi port 2000");
	top->addEntry( w3outxcv );

	SetupMenuSelect * w3outs1 = new SetupMenuSelect( "RS232 S1", RST_ON_EXIT, 0, true, &rt_w3_s1 );
	w3outs1->addEntry( "Disable");
	w3outs1->addEntry( "Enable");
	w3outs1->setHelp("Send S1 serial port data to/from WiFi port 2000");
	top->addEntry( w3outs1 );

	SetupMenuSelect * w3outs2 = new SetupMenuSelect( "RS232 S2", RST_ON_EXIT, 0, true, &rt_w3_s2 );
	w3outs2->addEntry( "Disable");
	w3outs2->addEntry( "Enable");
	w3outs2->setHelp("Send S2 serial port data to/from WiFi port 2000");
	top->addEntry( w3outs2 );

	SetupMenuSelect * w3outw0 = new SetupMenuSelect( "WiFi 8880", RST_ON_EXIT, 0, true, &rt_w3_w0 );
	w3outw0->addEntry( "Disable");
	w3outw0->addEntry( "Enable");
	w3outw0->setHelp("Send WiFi port 8880 data to/from WiFi port 2000");
	top->addEntry( w3outw0 );

	SetupMenuSelect * w3outw1 = new SetupMenuSelect( "WiFi 8881", RST_ON_EXIT, 0, true, &rt_w3_w1 );
	w3outw1->addEntry( "Disable");
	w3outw1->addEntry( "Enable");
	w3outw1->setHelp("Send WiFi port 8881 data to/from WiFi port 2000");
	top->addEntry( w3outw1 );
}

void SetupMenu::system_menu_create_comm_routing( MenuEntry *top ){

	SetupMenuSelect * wloutxcv = new SetupMenuSelect( "XCVario-WL", RST_NONE, 0, true, &rt_xcv_wl );
	wloutxcv->addEntry( "Disable");
	wloutxcv->addEntry( "Enable");
	wloutxcv->setHelp( "Whether vario data is routed from/to wireless (Bluetooth or WiFi port 8880)");
	top->addEntry( wloutxcv );

	SetupMenu * s1out = new SetupMenu( "S1 Routing");
	top->addEntry( s1out );
	if (serial1_speed.get() == 0)
		s1out->setHelp( "Warning: S1 baud rate was set to 'off'");
	else
		s1out->setHelp( "Select data sources to be routed from/to serial interface S1 (reboots)");
	s1out->addCreator( system_menu_create_interfaceS1_routing );

	if( hardwareRevision.get() >= XCVARIO_21 ) {
		SetupMenu * s2out = new SetupMenu( "S2 Routing" );
		if (serial2_speed.get() == 0)
			s2out->setHelp( "Warning: S2 baud rate was set to 'off'");
		else
			s2out->setHelp( "Select data sources to be routed from/to serial interface S2 (reboots)");
		top->addEntry( s2out );
		s2out->addCreator( system_menu_create_interfaceS2_routing );
	}

	if( hardwareRevision.get() >= XCVARIO_22 ){
		SetupMenu * canrt = new SetupMenu( "CAN Routing" );
		top->addEntry( canrt );
		canrt->setHelp( "Select data sources to be routed from/to CAN interface (reboots)");
		canrt->addCreator( system_menu_create_interfaceCAN_routing );
	}

	SetupMenu * w3rt = new SetupMenu( "Port 2000 Routing" );
	top->addEntry( w3rt );
	w3rt->setHelp( "Select data sources to be routed from/to WiFi port 2000 (reboots)");
	w3rt->addCreator( system_menu_create_interfaceW3_routing );

	SetupMenuSelectCodes * datamon = new SetupMenuSelectCodes( "Monitor", RST_NONE, data_mon, true, &data_monitor );
	datamon->setHelp( "Short press to start/pause, long press to terminate", 280);
	datamon->addEntryCode( "Disable", MON_OFF);
	if ((wireless == WL_BLUETOOTH) || (wireless == WL_BLUETOOTH_LE)) {
		datamon->addEntryCode( "Bluetooth", MON_BLUETOOTH);
	} else if (wireless != WL_DISABLE) {
		datamon->addEntryCode( "Wifi 8880", MON_WIFI_8880);
		datamon->addEntryCode( "Wifi 8881", MON_WIFI_8881);
		datamon->addEntryCode( "Wifi 8882", MON_WIFI_8882);
		datamon->addEntryCode( "Wifi 2000", MON_WIFI_2000);
	}
	datamon->addEntryCode( "RS232 S1", MON_S1);
	datamon->addEntryCode( "RS232 S2", MON_S2);
	datamon->addEntryCode( "CAN Bus", MON_CAN);
	top->addEntry( datamon );

	SetupMenuSelect * datamonmod = new SetupMenuSelect( "Monitor Mode", RST_NONE, data_mon, true, &data_monitor_mode );
	datamonmod->setHelp( "Select data display as ASCII text or as binary hexdump");
	datamonmod->addEntry( "ASCII");
	datamonmod->addEntry( "Binary");
	top->addEntry( datamonmod );
}

void SetupMenu::system_menu_create_comm( MenuEntry *top ){

	SetupMenuSelectCodes * mm = new SetupMenuSelectCodes( "Mode", RST_ON_EXIT, 0, true, &master_mode );
	mm->setHelp( "XCVario operation: standalone (can connect to nav devices), or connected to another XCVario. (Reboots)");
	mm->addEntryCode( "Standalone", MODE_STANDALONE);               // 0
	if( hardwareRevision.get() >= XCVARIO_22 ){
		mm->addEntryCode( "Master (via CAN)", MODE_CAN_MASTER);     // 3
		mm->addEntryCode( "Client (via CAN)", MODE_CAN_CLIENT);     // 4
	}
	mm->addEntryCode( "Master (via WiFi)", MODE_WL_MASTER);         // 1
	mm->addEntryCode( "Client (via WiFi)", MODE_WL_CLIENT);         // 2
	top->addEntry( mm );

	// just show the current master mode
	show_mode_change();       // reflect current mode into mode_shown
	SetupMenuSelect * sm = new SetupMenuSelect( "Mode", RST_ON_EXIT, 0, true, &show_mode );
	sm->setHelp( "XCVario operation: standalone (can connect to nav devices), or connected to another XCVario");
	sm->addEntry( mode_shown );
	show_mode_menu = sm;    // allows changing the label (in SetupNG.cpp) later if mode changes
	top->addEntry( sm );

	// NMEA protocol of variometer
	SetupMenuSelect * nmea = new SetupMenuSelect( "NMEA Protocol", RST_NONE , 0, true, &nmea_protocol );
	top->addEntry( nmea );
	nmea->setHelp( "Setup the protocol used for sending NMEA sentences. This needs to match the device driver chosen in XCSoar/LK8000");
	nmea->addEntry( "OpenVario");
	nmea->addEntry( "Borgelt");
	nmea->addEntry( "Cambridge");
	nmea->addEntry( "XCVario");
	nmea->addEntry( "Disable");

	SetupMenu * wireless = new SetupMenu( "Wireless" );
	wireless->addCreator(system_menu_create_comm_wireless);
	wireless->setHelp("Configure wireless communications, 240");
	top->addEntry( wireless );

	SetupMenu * wired = new SetupMenu( "Wired" );
	wired->addCreator(system_menu_create_comm_wired);
	wired->setHelp("Configure wired data ports");
	top->addEntry( wired );

	SetupMenu * routing = new SetupMenu( "Data Routing" );
	routing->addCreator(system_menu_create_comm_routing);
	routing->setHelp("How data is routed between sources and destinations (reboots)", 260);
	top->addEntry( routing );
}

void SetupMenu::system_menu_create( MenuEntry *sye ){

	// show-mode menu has gone out of scope by now
	// make sure SetupNG.cpp/show_mode_change() does not try and dereference it
	show_mode_menu = nullptr;
	SetupMenu * comm = new SetupMenu( "Communications" );
	comm->setHelp( "Setup wired and wireless communications, data routing, etc", 240 );
	sye->addEntry( comm );
	comm->addCreator(system_menu_create_comm);

	SetupMenu * hardware = new SetupMenu( "Hardware Setup" );
	hardware->setHelp( "Setup variometer hardware e.g. display, rotary, AS and AHRS sensor, voltmeter, etc", 240 );
	sye->addEntry( hardware );
	hardware->addCreator(system_menu_create_hardware);

	SetupMenu * soft = new SetupMenu( "Software Update" );
	sye->addEntry( soft );
	soft->addCreator(system_menu_create_software);

	SetupMenuSelect * fa = new SetupMenuSelect( "Factory Reset", RST_IMMEDIATE, 0, false, &factory_reset );
	fa->setHelp("Option to reset all settings to factory defaults, means metric system, 5 m/s vario range and more");
	fa->addEntry( "Cancel");
	fa->addEntry( "ResetAll");
	sye->addEntry( fa );
}

void SetupMenu::setup_create_root(MenuEntry *top ){
	ESP_LOGI(FNAME,"setup_create_root()");
	if( rot_default.get() == 0 ) {
		SetupMenuValFloat * mc = new SetupMenuValFloat( "MC", "",	0.0, 9.9, 0.1, 0, true, &MC );
		mc->setHelp("MacCready value for optimum cruise speed or average climb rate, in same unit as the variometer");
		mc->setPrecision(1);
		top->addEntry( mc );
	}
	else {
		SetupMenuValFloat * vol = new SetupMenuValFloat( "Audio Volume", "%", 0.0, 200, 2, vol_adj, true, &audio_volume );
		vol->setHelp("Audio volume level for variometer tone on internal and external speaker");
		vol->setMax(max_volume.get());
		top->addEntry( vol );
	}

	SetupMenuValFloat *qnh_menu = SetupMenu::createQNHMenu();
	top->addEntry( qnh_menu );

	SetupMenuValFloat * afe = new SetupMenuValFloat( "Airfield Elevation", "", -1, 3000, 1, 0, true, &elevation );
	afe->setHelp("Airfield elevation in meters for QNH auto adjust on ground according to this elevation");
	top->addEntry( afe );

	SetupMenuValFloat * bgs = new SetupMenuValFloat( "Bugs", "%", 0.0, 50, 1, bug_adj, true, &bugs  );
	bgs->setHelp("Percent degradation of gliding performance due to bugs contamination");
	top->addEntry( bgs );

	SetupMenuValFloat * bal = new SetupMenuValFloat( "Ballast", "litre", 0.0, 500, 1, water_adj, true, &ballast_kg  );
	bal->setHelp("Amount of water ballast added to the over all weight");
	bal->setPrecision(0);
	top->addEntry( bal );

	SetupMenuValFloat * crewball = new SetupMenuValFloat( "Crew Weight", "kg", 0, 300, 1, crew_weight_adj, false, &crew_weight );
	crewball->setPrecision(0);
	crewball->setHelp("Weight of the pilot(s) including parachute (everything on top of the empty weight apart from ballast)");
	top->addEntry( crewball );

	// Clear student mode, password correct
	if( student_mode.get() && (int)(password.get()) == 271 ) {
		student_mode.set( 0 );
		password.set( 0 );
	}
	// Student mode: Query password
	if( student_mode.get() )
	{
		// simplified audio menu
		SetupMenu * ad = new SetupMenu( "Audio" );
		top->addEntry( ad );
		ad->addCreator( audio_menu_create_student );

		// path to exit student mode
		SetupMenuValFloat * passw = new SetupMenuValFloat( "Expert Password", "", 0, 1000, 1, 0, false, &password  );
		passw->setPrecision( 0 );
		passw->setHelp( "To exit from student mode enter expert password and restart device after expert password has been set correctly");
		top->addEntry( passw );
	}
	else  // not student mode
	{
		// Glider Setup
		SetupMenu * po = new SetupMenu( "Glider Details" );
		top->addEntry( po );
		po->addCreator( glider_menu_create );

		// Audio
		SetupMenu * ad = new SetupMenu( "Audio" );
		top->addEntry( ad );
		ad->addCreator( audio_menu_create );

		// Options Setup
		SetupMenu * opt = new SetupMenu( "Options" );
		top->addEntry( opt );
		opt->addCreator( options_menu_create );

		// System Setup
		SetupMenu * sy = new SetupMenu( "System" );
		top->addEntry( sy );
		sy->addCreator( system_menu_create );
	}
}

SetupMenuValFloat * SetupMenu::createQNHMenu(){
	SetupMenuValFloat * qnh = new SetupMenuValFloat( "QNH", "", 900, 1100.0, 0.250, qnh_adj, true, &QNH );
	qnh->setHelp("QNH pressure value from ATC. On ground you may adjust to airfield altitude above MSL", 180 );
	return qnh;
}

void SetupMenu::setup( )
{
	ESP_LOGI(FNAME,"SetupMenu setup()");
	SetupMenu * root = new SetupMenu( "Setup" );
	root->setRoot( root );
	root->addEntry( root );
	// Create static menues
	if( NEED_VOLTAGE_ADJUST && !SetupMenuValFloat::meter_adj_menu ){
		SetupMenuValFloat::meter_adj_menu = new SetupMenuValFloat( "Voltmeter Adjust", "%",	-25.0, 25.0, 0.01, factv_adj, false, &factory_volt_adjust,  RST_IMMEDIATE, false, true);
	}
	setup_create_root( root );
}
