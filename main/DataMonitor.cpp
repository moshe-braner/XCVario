#include "DataMonitor.h"
#include "sensor.h"
#include "SetupNG.h"
#include "logdef.h"
#include "Flarm.h"

#define SCROLL_TOP      20
#define SCROLL_BOTTOM  320

xSemaphoreHandle DataMonitor::mutex = 0;

DataMonitor::DataMonitor(){
	mon_started = false;
	ucg = 0;
	scrollpos = SCROLL_BOTTOM;
	//paused = true;
	paused = false;
	setup = 0;
	channel = MON_OFF;
	mutex = xSemaphoreCreateMutex();
	first=true;
	rx_total = 0;
	tx_total = 0;
}

int DataMonitor::maxChar( const char *str, int pos, int len, bool binary ){
	int N=0;
	int i=0;
	char s[4] = { 0 };
	while( N <= 240 ){
		if( binary ){
			sprintf( s, "%02x ", str[i+pos] );
		}
		else{
			s[0] = str[i+pos];
		}
		N += ucg->getStrWidth( s );
		if( N<220 && (i+pos)<len ){
			i++;
		}else{
			break;
		}
	}
	return i;
}

void DataMonitor::header( int ch, bool binary, int len, e_dir_t dir ){
	if( dir == DIR_RX )
		rx_total += len;
	else
		tx_total += len;
	//ESP_LOGI(FNAME,"header() %d %d %d ", len, rx_total, tx_total );
	const char * what;
	switch( ch ) {
		case MON_BLUETOOTH: what = "BT"; break;
		case MON_WIFI_8880: what = "W 8880"; break;
		case MON_WIFI_8881: what = "W 8881"; break;
		case MON_WIFI_8882: what = "W 8882"; break;
		case MON_WIFI_2000: what = "W 2000"; break;
		case MON_S1:  what = "S1"; break;
		case MON_S2:  what = "S2"; break;
		case MON_CAN: what = "CAN"; break;
		default:      what = "OFF"; break;
	}
	const char * b;
	if( binary )
		b = "B-";
	else
		b = "";
	ucg->setColor( COLOR_WHITE );
	ucg->setFont(ucg_font_fub11_tr, true );
	ucg->setPrintPos( 20, SCROLL_TOP );
	if( paused )
		ucg->printf( "%s%s: RX:%d TX:%d hold    ", b, what, rx_total, tx_total );
	else
		ucg->printf( "%s%s: RX:%d TX:%d bytes   ", b, what, rx_total, tx_total );
}

void DataMonitor::monitorString( int ch, e_dir_t dir, const char *str, int len ){
	if( !mon_started || ch != channel )
		return;
	bool binary = (data_monitor_mode.get() == MON_MOD_BINARY);
	if( xSemaphoreTake(mutex,portMAX_DELAY ) ){
		if( paused )
			header( ch, binary, len, dir );     // just update header
		else
			printString( ch, dir, str, binary, len );   // also calls header()
		xSemaphoreGive(mutex);
	}
}

void DataMonitor::printString( int ch, e_dir_t dir, const char *str, bool binary, int len ){
	if (! binary)
		ESP_LOGI(FNAME,"DM ch:%d dir:%d len:%d data:%s", ch, dir, len, str );
	const int scroll_lines = 20;
	char dirsym;
	if( dir == DIR_RX )
		dirsym = '>';
	else
		dirsym = '<';
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	if( first ){
		first = false;
		ucg->setColor( COLOR_BLACK );
		ucg->drawBox( 0,SCROLL_TOP,240,320 );
	}
	ucg->setColor( COLOR_WHITE );
    header( ch, binary, len, dir );
	//if( !binary )
	// 	len = len-1;  // ignore the \n in ASCII mode
	int hunklen = 0;
	int pos=0;
	do {
		// ESP_LOGI(FNAME,"DM 1 len: %d pos: %d", len, pos );
		hunklen = maxChar( str, pos, len, binary );
		if( hunklen ){
			char hunk[64] = { 0 };
			memcpy( (void*)hunk, (void*)(str+pos), hunklen );
			// ESP_LOGI(FNAME,"DM 2 hunklen: %d pos: %d  h:%s", hunklen, pos, hunk );
			ucg->setColor( COLOR_BLACK );
			ucg->drawBox( 0, scrollpos, 240,scroll_lines );
			ucg->setColor( COLOR_WHITE );
			ucg->setPrintPos( 0, scrollpos+scroll_lines );
			ucg->setFont(ucg_font_fub11_tr, true );
			char txt[256];
			int hpos = 0;
			if( binary ){   // format data as readable text
				hpos += sprintf( txt, "%c ", dirsym );
				for( int i=0; i<hunklen && hpos<95 ; i++ ){
					hpos += sprintf( txt+hpos, "%02x ", hunk[i] );
				}
				txt[hpos] = 0; // zero terminate string
				ucg->print( txt );
				ESP_LOGI(FNAME,"DM binary ch:%d dir:%d string:%s", ch, dir, txt );
			}
			else{
				hpos += sprintf( txt, "%c ", dirsym );
				hpos += sprintf( txt+hpos, "%s", hunk );
				txt[hpos] = 0;
				ucg->print( txt );
				//ESP_LOGI(FNAME,"DM ascii ch:%d dir:%d data:%s", ch, dir, txt );
			}
			pos+=hunklen;
			// ESP_LOGI(FNAME,"DM 3 pos: %d", pos );
			scroll(scroll_lines);
		}
	}while( hunklen );
	xSemaphoreGive(spiMutex);
}

void DataMonitor::scroll(int scroll){
	scrollpos+=scroll;
	if( scrollpos >= SCROLL_BOTTOM )
		scrollpos = scroll;
	ucg->scrollLines( scrollpos );  // set frame origin
}

void DataMonitor::press(){
	if( !mon_started ){
		//ESP_LOGI(FNAME,"Press, but not started, return" );
		return;
	}
	ESP_LOGI(FNAME,"press paused: %d", paused );
	//if( !Rotary.readSwitch() ){ // only process press here
	if( paused )
		paused = false;
	else
		paused = true;
	//}
	delay( 100 );
}

void DataMonitor::longPress(){
	if( !mon_started ){
		//ESP_LOGI(FNAME,"longPress, but not started, return" );
		return;
	}
	ESP_LOGI(FNAME,"longPress" );
	gflags.ignorePress = true;    // so setup menu will not receive longpress
	stop();
}

void DataMonitor::start(SetupMenuSelect * p){
	ESP_LOGI(FNAME,"start");
	if( !setup )
		attach( this );     // maybe attach() on each start() and detach() at stop()?
	setup = p;
	tx_total = 0;
	rx_total = 0;
	//channel = p->getSelect();       // broken, for S1 & S2 menus with only 0,1 choices
	channel = data_monitor.get();     // action function SetupMenu.cpp data_mon() set it
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	SetupMenu::catchFocus( true );
	ucg->setColor( COLOR_BLACK );
	ucg->drawBox( 0,0,240,320 );
	ucg->setColor( COLOR_WHITE );
	ucg->setFont(ucg_font_fub11_tr, true );
	header( channel );
	if( display_orientation.get() == DISPLAY_TOPDOWN )
		ucg->scrollSetMargins( 0, SCROLL_TOP );
	else
		ucg->scrollSetMargins( SCROLL_TOP, 0 );
	xSemaphoreGive(spiMutex);
	delay(300);
	mon_started = true;
	//paused = true; // will resume with press()
	paused = false;
	//ESP_LOGI(FNAME,"started");
}

void DataMonitor::stop(){
	ESP_LOGI(FNAME,"stop");
	mon_started = false;
	delay( 200 );
	ucg->scrollLines( 0 );
	setup->setSelectCode( MON_OFF );    // causes setupMenu to stop ignoring longpress
	data_monitor.set( MON_OFF );        // was also done by setSelectCode()
	channel = MON_OFF;
	//paused = false;                  // let start() handle this
	first = true;
	//detach( this );
	//gflags.escapeSetup = false;      // in case longpress observed by setup menu and not ignored
	SetupMenu::catchFocus( false );
	delay( 100 );
}

