#ifndef SOUND_H
#define SOUND_H

#if defined(NOSENSORS)
#include "softPoti.h"
#else
#include "mcp4018.h"
#endif

typedef enum _sound { DING, HI } e_sound;

class Sound
{
public:

	Sound();

	static void playSound( e_sound a_sound, bool end=false );

private:
	static void timer_isr(void* arg);
	static void timerInitialise (int timer_period_us);
	static int pos;
	static bool ready;
#if defined(NOSENSORS)
	static SoftPoti *_poti;
#else
	static MCP4018 *_poti;
#endif
	static e_sound sound;
};

#endif
