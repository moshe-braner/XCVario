#if defined(NOSENSORS)

#include "soc/sens_reg.h"

#include "softPoti.h"
#include "ESPAudio.h"

//Create instance
SoftPoti::SoftPoti()
{
    wiper = 63;
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

bool SoftPoti::incWiper(){
	ESP_LOGI(FNAME,"wiper: %d", wiper );
	wiper = (wiper+1) & 0x1E0;    // multiples of 32==step
	wiper += 32;
	if( wiper > 127 )  wiper = 127;
	ESP_LOGI(FNAME,"wiper ++> %d", wiper );
	return( writeWiper(wiper) );
}

// decWiper() and incWiper() aren't actually called from anywhere

bool SoftPoti::decWiper(){
	ESP_LOGI(FNAME,"wiper: %d", wiper );
	wiper = (wiper+1) & 0x1E0;    // multiples of 32==step
	if( wiper > 32 )
		wiper -= 32;
	else
		wiper = 0;
	ESP_LOGI(FNAME,"wiper --> %d", wiper );
	return( writeWiper(wiper) );
}

bool SoftPoti::readWiper( uint16_t &val ) {
	val = wiper;
	return true;
}

bool SoftPoti::writeWiper( uint16_t val ) {
    wiper = val;
    static int prev_scale = -1;
    //if (wiper == 0) {
    //    Audio::dacDisable();
    //    // prev_scale = -1;
    //    return true;
    //}
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
        Audio::restart(scale);
        prev_scale = scale;
    }
    return true;
}

#endif
