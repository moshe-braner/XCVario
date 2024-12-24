/*
 * SetupMenuSelectCodes.h
 *
 *  Created on: Dec 23, 2024
 *      Author: Moshe Braner
 */

#ifndef _SetupMenuSelectCodes_H_
#define _SetupMenuSelectCodes_H_
#include "SetupNG.h"
#include "MenuEntry.h"
#include "SetupMenuValCommon.h"

class SetupMenuSelectCodes:  public MenuEntry
{
public:
	SetupMenuSelect( const char* title, e_restart_mode_t restart=RST_NONE,
	    int (*action)(SetupMenuSelectCodes *p) = 0, bool save=true, SetupNG<int> *anvs=0,
	    bool ext_handler=false, bool end_menu=false );
	virtual ~SetupMenuSelectCodes();
	void display( int mode=0 );
	//bool existsEntry( std::string ent );
	void addEntry( const char* ent, const int code=-1 );
	void addEntryList( const char ent[][4], int size );
	//void delEntry( const char * ent );
	//void delEntryByCode( const int code );
	void delLastEntry();
	void updateEntry( const char * ent, int num, const int code=-1 );
	void up( int count );  // step up to parent
	void down( int count );
	void press();
	void longPress();
	void escape() {};
	const char *value();
	int getSelect();
	int getSelectCode();
	void setSelect( int sel );
	void setSelectCode( int code );
	const char * getEntry() const ;
	int numEntries() { return _entries; };

private:
	uint8_t  _select;
	uint8_t  _select_save;
	uint8_t  _entries;
	e_restart_mode_t _restart_mode;
	bool _ext_handler;
	bool _save;
	bool _end_menu;
	std::vector<const char *> _labels;
	std::vector<uint8_t> _codes;
	int (*_action)( SetupMenuSelectCodes *p );
	SetupNG<int> *_nvs;
};
