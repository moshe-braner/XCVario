/*
 * Version.cpp
 *
 *  Created on: Feb 10, 2019
 *      Author: iltis 
 */

#include "Version.h"
#include <stdio.h>
#include <string.h>
#include <logdef.h>

char Version::_version[32];
char * program_version;

Version::Version() {
	const char data[]=__DATE__;
	const char tempo[]=__TIME__;
	const char nomes[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
	char omes[4];
	int ano, mes, dia, hora, min, seg;
	sscanf(data, "%s %d %d", omes, &dia, &ano);
	sscanf(tempo, "%d:%d:%d", &hora, &min, &seg);
	mes=(strstr(nomes, omes)-nomes)/3+1;

#if defined(NOSENSORS)
#if defined(SUNTON28)
	sprintf(_version,"%02d.%02d%02d%02dS", ano%100, mes, dia, hora );
#else
	sprintf(_version,"%02d.%02d%02d%02dB", ano%100, mes, dia, hora );
#endif
#else
	sprintf(_version,"%02d.%02d%02d-%02d", ano%100, mes, dia, hora );
#endif
	program_version = _version;
}

Version::~Version() {
}

