/*
 * SetupMenu.h
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#ifndef _SetupMenu_H_
#define _SetupMenu_H_
#include "MenuEntry.h"
#include <string>
#include <driver/gpio.h>
#include "SetupMenuValFloat.h"

class IpsDisplay;
class ESPRotary;
class PressureSensor;
class AnalogInput;

class SetupMenu:  public MenuEntry {
public:
	SetupMenu();
	SetupMenu( const char* title );
	virtual ~SetupMenu();
	void begin( IpsDisplay* display, PressureSensor * bmp, AnalogInput *adc );
	void setup();
	void display( int mode=0 );
	const char *value() { return 0; };
	void up( int count );  // step up to parent
	void down( int count );
	void press();
	void longPress();
	void escape();
	void showMenu();
	void create_subtree();
	void delete_subtree();


	static void catchFocus( bool activate );
	static bool focus;
	static gpio_num_t getGearWarningIO();

	static void setup_create_root( MenuEntry *top );

	static void vario_menu_create( MenuEntry *top );
	static void vario_menu_create_damping( MenuEntry *top );
	static void vario_menu_create_meanclimb( MenuEntry *top );
	static void vario_menu_create_s2f( MenuEntry *top );
	static void vario_menu_create_ec( MenuEntry *top );

	static void audio_menu_create( MenuEntry *top );
	static void audio_menu_create_volume( MenuEntry *top );
	static void audio_menu_create_tonestyles( MenuEntry *top );
	static void audio_menu_create_deadbands( MenuEntry *top );
	static void audio_menu_create_equalizer( MenuEntry *top );
	static void audio_menu_create_mute( MenuEntry *top );

	static void glider_menu_create( MenuEntry *top );
	static void glider_menu_create_polarpoints( MenuEntry *top );

	static void options_menu_create( MenuEntry *top );
	static void options_menu_create_units( MenuEntry *top );
	static void options_menu_create_flarm( MenuEntry *top );
	static void options_menu_create_advanced( MenuEntry *top );
	static void options_menu_create_wind( MenuEntry *top );
	static void options_menu_create_wind_circlingwind( MenuEntry *top );
	static void options_menu_create_wireless( MenuEntry *top );
	static void options_menu_create_wireless_custom_id( MenuEntry *top );
	static void options_menu_create_wireless_routing( MenuEntry *top );
	static void options_menu_create_gload( MenuEntry *top );
	static void options_menu_create_gload_limits( MenuEntry *top );
	static void options_menu_create_altimeter_airspeed( MenuEntry *top );
	static void options_menu_create_stallwarning( MenuEntry *top );
	static void options_menu_create_display( MenuEntry *top );
	static void options_menu_create_rotary( MenuEntry *top );
	static void options_menu_create_horizon_screen( MenuEntry *top );

	static void system_menu_create( MenuEntry *top );
	static void system_menu_create_software( MenuEntry *top );
	static void system_menu_create_comm( MenuEntry *top );
	static void system_menu_create_comm_wireless( MenuEntry *top );
	static void system_menu_create_comm_wired( MenuEntry *top );
	static void system_menu_create_comm_routing( MenuEntry *top );
	static void system_menu_create_altimeter_airspeed( MenuEntry *top );
	static void system_menu_create_interfaceS1( MenuEntry *top );
	static void system_menu_create_interfaceS1_routing( MenuEntry *top );
	static void system_menu_create_interfaceS2( MenuEntry *top );
	static void system_menu_create_interfaceS2_routing( MenuEntry *top );
	static void system_menu_create_interfaceCAN_routing( MenuEntry *top );
	static void system_menu_create_hardware( MenuEntry *top );
	static void system_menu_create_ahrs( MenuEntry *top );
	static void system_menu_create_ahrs_lc( MenuEntry *top );
	static void system_menu_create_ahrs_parameter( MenuEntry *top );
	static void system_menu_create_compass( MenuEntry *top );
	static void system_menu_create_compass_dev( MenuEntry *top );
	static void system_menu_create_compass_devs( MenuEntry *top );
	static void system_menu_create_compass_nmea( MenuEntry *top );
	static void system_menu_create_compass_straightwind( MenuEntry *top );
	static void system_menu_create_compass_straightwind_limits( MenuEntry *top );
	static void system_menu_create_compass_straightwind_filters( MenuEntry *top );
	static void system_menu_create_battery( MenuEntry *top );
	static SetupMenuValFloat * createQNHMenu();
};

#endif
