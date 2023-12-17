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
// flag Sunton version with 'S' at end of version date
	sprintf(_version,"%02d.%02d%02d%02dS", ano%100, mes, dia, hora );
#else
// flag other no-sensors version with 'B' at end of version date
	sprintf(_version,"%02d.%02d%02d%02dB", ano%100, mes, dia, hora );
#endif
#else
// flag private version for XCvario with 'M' at end of version date
	sprintf(_version,"%02d.%02d%02d%02dM", ano%100, mes, dia, hora );
#endif
	program_version = _version;
}

Version::~Version() {
}

