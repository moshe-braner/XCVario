/*
 * SetupMenuSelect.cpp
 *
 *  A replacment for the original SetupMenuSelect,
 *      that allows specifying the codes for each option
 *  Created on: Dec 23, 2024
 *      Author: Moshe Braner
 */

#include <logdef.h>
#include <sensor.h>
#include "Units.h"
#include "SetupMenuSelect.h"
#include "ESPAudio.h"

// return the selected label
const char * SetupMenuSelect::getEntry() const
{
	if(_entries)
		return _labels.at(_select);
	return NULL;
}

// return the menu index with the code that matches the nvs variable (or 0)
// side effect: selects that index
int SetupMenuSelect::getSelect() {
	_select = 0;
	if( _nvs && _entries) {
		int code = _nvs->get();
		for (int i=0; i<_entries; i++) {
			if (_codes.at(i) == (uint8_t) code) {
				_select = i;
				break;
			}
		}
	}
	return (int)_select;
}

// return the selected nvs code
int SetupMenuSelect::getSelectCode() {
	if(_entries)
		return _codes.at(_select);
	return 0;
}

const char *SetupMenuSelect::value() {
	getSelect();                               // adjust _select to fit nvs value
	return getEntry();                        // return the label
}

// add at current end of menu
void SetupMenuSelect::addEntry( const char* ent, int code ) {
	if (_entries == 255)
		return;
	_labels.push_back( ent );
	if (code < 0)
		_codes.push_back( _entries );    // code is same as index into vector
	else
		_codes.push_back( (uint8_t) code );     // code was specified
	_entries++;
	getSelect();      // sets _select to the index of the code that equals nvs value
}

// change the label of the entry at the index sel (and optionally also change the code)
void SetupMenuSelect::updateEntry( const char * ent, int sel, int code ) {
	//ESP_LOGI(FNAME,"updateEntry ent:%s  sel:%d code:%d total%d", ent, sel, code, _entries );
	if (sel < 0 || sel >= _entries)
        return;
	_labels.at(sel) = ent;
	if (code >= 0)                // code was specified
		_codes.at(sel) = (uint8_t) code;
}

// select the given index, and set the nvs variable to the associated code
void SetupMenuSelect::setSelect( int sel ) {
	if (_entries == 0) {
		_select = 0;
		return;
	}
	if (sel < 0)
		sel = 0;
	if (sel > _entries-1)
		sel = _entries-1;
	_select = (uint8_t)sel;
	if( _nvs )
		_nvs->set( _codes.at(_select) );
}

// select the index with the given code, and set the nvs variable to same code
void SetupMenuSelect::setSelectCode( int code ) {
	if (_entries == 0)
		return;
	_select = 0;
	for (int i=0; i<_entries; i++) {
		if (_codes.at(i) == code) {
			_select = i;
			break;
		}
	}
	if( _nvs )
		_nvs->set( _codes.at(_select) );

}

SetupMenuSelect::SetupMenuSelect( const char* title, e_restart_mode_t restart,
    int (*action)(SetupMenuSelect *p), bool save, SetupNG<int> *anvs,
    bool ext_handler, bool end_menu ) {
	// ESP_LOGI(FNAME,"SetupMenuSelect( %s ) action: %x", title, (int)action );
	attach(this);
	_ext_handler = ext_handler;
	_title = title;
	_nvs = 0;
	_select = 0;
	_select_save = 0;
	_end_menu = end_menu;
	highlight = -1;
	_entries = 0;
	_restart_mode = restart;
	_action = action;
	_save = save;
	if( anvs ) {
		_nvs = anvs;
		// ESP_LOGI(FNAME,"_nvs->key(): %s val: %d", _nvs->key(), (int)(_nvs->get()) );
		_select_save = (uint8_t)_nvs->get();
	}
// Before any entries are added, there is really nothing valid to set _select to!
// Instead initialize later, upon adding entries, by calling getSelect()
}

SetupMenuSelect::~SetupMenuSelect()
{
	detach(this);
}

// alas the rest of the code is repeated from SetupMenuSelect

void SetupMenuSelect::display( int mode ){
	if( (selected != this) || !gflags.inSetup  )
		return;
	if (_entries == 0)
		return;
	ESP_LOGI(FNAME,"display() pressed:%d title:%s action: %x hl:%d", pressed, _title, (int)(_action), highlight );
	clear();
	if( _ext_handler ){  // handling is done only in action method
		ESP_LOGI(FNAME,"ext handler");
		selected = _parent;
	}else
	{
		xSemaphoreTake(spiMutex,portMAX_DELAY );
		ucg->setPrintPos(1,25);
		ESP_LOGI(FNAME,"Title: %s ", _title );
		ucg->printf("<< %s",_title);
		xSemaphoreGive(spiMutex );
		if( _select > _labels.size() )
			_select = _entries-1;
		// ESP_LOGI(FNAME,"select=%d numsel=%d size=%d val=%s", _select, _entries, _labels.size(), _labels[_select]  );
		if( _entries > 9 ){
			xSemaphoreTake(spiMutex,portMAX_DELAY );
			ucg->setPrintPos( 1, 50 );
			ucg->printf( "%s                ", _labels[_select] );
			xSemaphoreGive(spiMutex );
		}else
		{
			xSemaphoreTake(spiMutex,portMAX_DELAY );
			for( int i=0; i<_entries && i<+10; i++ )	{
				ucg->setPrintPos( 1, 50+25*i );
				ucg->print( _labels[i] );
			}
			ucg->drawFrame( 1,(_select+1)*25+3,238,25 );
			xSemaphoreGive(spiMutex );
		}

		int y=_entries*25+50;
		showhelp( y );
		if(mode == 1 && _save == true ){
			xSemaphoreTake(spiMutex,portMAX_DELAY );
			ucg->setColor( COLOR_BLACK );
			ucg->drawBox( 1,280,240,40 );
			ucg->setPrintPos( 1, 300 );
			ucg->setColor( COLOR_WHITE );
			ucg->print( "Saved" );
			xSemaphoreGive(spiMutex );
		}
		if( mode == 1 )
			delay(1000);
	}
}

void SetupMenuSelect::down(int count){
	if( (selected != this) || !gflags.inSetup )
		return;
	if (_entries == 0)
		return;
	if( _entries > 9 ){
		xSemaphoreTake(spiMutex,portMAX_DELAY );
		while( count > 0 ) {
			if( (_select) > 0 )
				(_select)--;
			count--;
		}
		ucg->setPrintPos( 1, 50 );
		ucg->setFont(ucg_font_ncenR14_hr, true );
		ucg->printf("%s                  ",_labels[_select] );
		xSemaphoreGive(spiMutex );
	}else {
		xSemaphoreTake(spiMutex,portMAX_DELAY );
		ucg->setColor(COLOR_BLACK);
		ucg->drawFrame( 1,(_select+1)*25+3,238,25 );  // blank old frame
		ucg->setColor(COLOR_WHITE);
		while( (_select) >  0 && count > 0){
			(_select)--;
			count--;
		}
		ESP_LOGI(FNAME,"val down %d", _select );
		ucg->drawFrame( 1,(_select+1)*25+3,238,25 );  // draw new frame
		xSemaphoreGive(spiMutex );
	}
}

void SetupMenuSelect::up(int count){
	if( (selected != this) || !gflags.inSetup )
		return;
	if (_entries == 0)
		return;
	if( _entries > 9 )
	{
		xSemaphoreTake(spiMutex,portMAX_DELAY );
		while( count > 0 ) {
			if( (_select) <  _entries-1 )
				(_select)++;
			count--;
		}
		ucg->setPrintPos( 1, 50 );
		ucg->setFont(ucg_font_ncenR14_hr, true );
		ucg->printf("%s                   ", _labels[_select] );
		xSemaphoreGive(spiMutex );
	}else {
		xSemaphoreTake(spiMutex,portMAX_DELAY );
		ucg->setColor(COLOR_BLACK);
		ucg->drawFrame( 1,(_select+1)*25+3,238,25 );  // blank old frame
		ucg->setColor(COLOR_WHITE);
		while ( (_select) < _entries-1 && count > 0){
			(_select)++;
			count--;
		}
		ESP_LOGI(FNAME,"val up %d", _select );
		ucg->drawFrame( 1,(_select+1)*25+3,238,25 );  // draw new frame
		xSemaphoreGive(spiMutex );
	}
}

void SetupMenuSelect::longPress(){
	press();
}

void SetupMenuSelect::press(){
	if( (selected != this) || !gflags.inSetup  )
		return;
	if (_entries == 0)
		return;
	ESP_LOGI(FNAME,"press() ext handler: %d press: %d _select: %d selected %p", _ext_handler, pressed, _select, selected );
	if ( pressed ){
		if( _select_save != getSelectCode() )
			display( 1 );
		//else
		//	display();
		if( _end_menu ){                        // only used by FLARM Simulation?
			ESP_LOGI(FNAME,"press() end_menu");
			selected = root;                    // <<< is this a memory leak?
			//gflags.escapeSetup = true;        // alternative - but does not work for reason unknown
		} else
		if( _parent != 0) {
			ESP_LOGI(FNAME,"go to parent");
			selected = _parent;
		}
		selected->highlight = -1;
		selected->pressed = true;
		if( _nvs ){
			_nvs->set( getSelectCode(), false );   // do sync in next step
			_nvs->commit();
		}
		pressed = false;
		if( _action != 0 ){
			ESP_LOGI(FNAME,"calling action in press %d", _select );
			(*_action)( this );
		}
		if( _select_save !=  getSelectCode() ) {
			if( _restart_mode == RST_ON_EXIT ) {
				_restart = true;
			}else if( _restart_mode == RST_IMMEDIATE ){
				_nvs->commit();
				MenuEntry::restart();
			}
			_select_save = getSelectCode();
		}
		if( _end_menu ){
			selected->press();
		}
	}
	else{
		pressed = true;
	}
}

void SetupMenuSelect::addEntryList( const char ent[][4], int size )
{
	for( int i=0; i<size; i++ ) {
		_labels.push_back( (char *)ent[i] );
		_codes.push_back( _entries );    // code=position, like in the SetupMenuSelect class
		_entries++;
		if (_entries == 255)
			break;
	}
	getSelect();        // sets _select to the index of the code that equals nvs value
}

void SetupMenuSelect::delLastEntry() {
	if (_entries == 0)
		return;
	_labels.pop_back();
	_codes.pop_back();
	_entries--;
	setSelect( _select );   // if last entry was selected, change to last-1
}

#if 0

// the following functions not currently used, so can skip

bool SetupMenuSelect::existsEntry( std::string ent ){
	for( std::vector<const char*>::iterator iter = _labels.begin(); iter != _labels.end(); ++iter )
		if( std::string(*iter) == ent )
			return true;
	return false;
}

void SetupMenuSelect::delEntry( const char* ent ) {
	int i=0;
	for( std::vector<const char *>::iterator iter = _labels.begin(); iter != _labels.end(); ++iter ) {
		if( std::string(*iter) == std::string(ent) ) {
			int j = 0;
			for( std::vector<uint8_t>::iterator iter2 = _codes.begin(); iter2 != _codes.end(); ++iter2 ) {
				if ( j == i ) {
					_codes.erase( iter2 );
					break;
				}
				++j;
			}
			_labels.erase( iter );
			_entries--;
			if( _select >= _entries )
				_select = _entries-1;
			break;
		}
		++i;
	}
}

void SetupMenuSelect::delEntryByCode( const int code ) {
	int i=0;
	for( std::vector<uint8_t>::iterator iter = _codes.begin(); iter != _codes.end(); ++iter ) {
		if( *iter == code ) {
			int j = 0;
			for( std::vector<const char *>::iterator iter2 = _labels.begin(); iter2 != _labels.end(); ++iter2 ) {
				if ( j == i ) {
					_labels.erase( iter2 );
					break;
				}
				++j;
			}
			_codes.erase( iter );
			_entries--;
			if( _select >= _entries )
				_select = _entries-1;
			break;
		}
		++i;
	}
}

#endif
