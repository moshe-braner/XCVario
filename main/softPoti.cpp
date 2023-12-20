#if defined(NOSENSORS)

#include "soc/sens_reg.h"

#include "softPoti.h"
#include "ESPAudio.h"

//Create instance
SoftPoti::SoftPoti()
{
	wiper = SOFTPOTIRANGE/2;
}

bool SoftPoti::begin()
{
    return( writeWiper(wiper) );
}

//destroy instance
SoftPoti::~SoftPoti()
{
}

bool SoftPoti::haveDevice() {
	return true;
}

bool SoftPoti::readWiper( int &val ) {
	val = wiper;
	return true;
}

bool SoftPoti::writeWiper( int val ) {
    wiper = val;
    static int prev_scale = -1;
    if (wiper == 0) {
        Audio::dacDisable();   // changing the scale is neither necessary nor sufficient
        // prev_scale = -1;
        return true;
    }
    int scale;
    if (wiper < 33)  // ==32
        scale = 3;         // 1/8 amplitude
    else if (wiper < 65)
        scale = 2;         // 1/4 amplitude
    else // if (wiper < 97)
        scale = 1;         // 1/2 amplitude
    //else
    //    scale = 0;         // full amplitude
    if( scale != prev_scale ) {
        ESP_LOGI(FNAME,"writeWiper: scale -> %d", scale );
        Audio::rescale(scale);
        prev_scale = scale;
    }
    return true;
}

bool SoftPoti::readVolume( float &val ) {
	int ival;
	if ( readWiper( ival ) ) {
		val = (float)(100 * ival) * getInvRange();
		return true;
	}
	else
	{
	    return false;
	}
}

bool SoftPoti::writeVolume( float val ) {
	int ival = (int)(val * getRange());
	ival /= 100;
	return writeWiper( ival );
}

#endif
