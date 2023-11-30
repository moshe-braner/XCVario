#include "ESPRotary.h"
#include "Setup.h"
#include "RingBufCPP.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include <freertos/queue.h>
#include "freertos/task.h"
#include "string.h"
#include "esp_system.h"
#include <esp32/rom/ets_sys.h>
#include <sys/time.h>
#include <Arduino.h>
#include <logdef.h>
#include <list>
#include <algorithm>
#include "Flarm.h"

#if defined(SUNTON28)

#include "SetupNG.h"
#include "XPT2046.h"

enum
{
    TOUCH_NONE,
    TOUCH_UP,
    TOUCH_MIDDLE,
    TOUCH_DOWN
};

#define MAX_UP      (Y_MIN + (Y_MAX-Y_MIN)/4)
#define MIN_MIDDLE  (Y_MIN + (Y_MAX-Y_MIN)/4 + (Y_MAX-Y_MIN)/8)
#define MAX_MIDDLE  (Y_MAX - (Y_MAX-Y_MIN)/4 - (Y_MAX-Y_MIN)/8)
#define MIN_DOWN    (Y_MAX - (Y_MAX-Y_MIN)/4)

int ESPRotary::touch_state()
{
    // de-bounce: freeze state for 120 ms after change
    static int old_state = TOUCH_NONE;
    static uint32_t next_time = 0;
    uint32_t new_time = millis();
    if (new_time < next_time)
        return old_state;
    int new_state = TOUCH_NONE;
    // check if hardware "boot" button pressed
    // otherwise use the touchscreen
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    if( !gpio_get_level(GPIO_NUM_0) ) {
        new_state = TOUCH_MIDDLE;
    } else {
        int16_t x, y;
        if (getTouch(x,y) != false) {
            // Touching logical top of screen should send "up"
            // (touch screen is not inverted even when display is)
            if (y < MAX_UP) {
                if( display_orientation.get() == DISPLAY_TOPDOWN )
                    new_state = TOUCH_DOWN;
                else
                    new_state = TOUCH_UP;
            } else if (y > MIN_DOWN) {
                if( display_orientation.get() == DISPLAY_TOPDOWN )
                    new_state = TOUCH_UP;
                else
                    new_state = TOUCH_DOWN;
            } else if (y > MIN_MIDDLE && y < MAX_MIDDLE) {
                new_state = TOUCH_MIDDLE;
            }
        }
    }
    if (new_state != old_state) {
        ESP_LOGI(FNAME,"touch_state -> %d", new_state );  // 0=release, 2=middle
        old_state = new_state;
        next_time = new_time + 120;
    }
    return new_state;
}

#else

gpio_num_t ESPRotary::clk, ESPRotary::dt;
gpio_num_t ESPRotary::sw = GPIO_NUM_0;

pcnt_config_t ESPRotary::enc;
pcnt_config_t ESPRotary::enc2;
int16_t ESPRotary::r_enc_count  = 0;
int16_t ESPRotary::r_enc2_count = 0;

#define ROTARY_SINGLE_INC 0
#define ROTARY_DOUBLE_INC 1

#endif   // SUNTON28

std::list<RotaryObserver *> ESPRotary::observers;

int ESPRotary::timer = 0;
bool ESPRotary::released = true;
bool ESPRotary::pressed = false;
bool ESPRotary::longPressed = false;

#if defined(SUNTON28)
bool ESPRotary::up = false;
bool ESPRotary::down = false;
#endif

static TaskHandle_t pid = NULL;

void ESPRotary::attach(RotaryObserver *obs) {
	// ESP_LOGI(FNAME,"Attach obs: %p", obs );
	observers.push_back(obs);
}
void ESPRotary::detach(RotaryObserver *obs) {
	// ESP_LOGI(FNAME,"Detach obs: %p", obs );
	auto it = std::find(observers.begin(), observers.end(), obs);
	if ( it != observers.end() ) {
		observers.erase(it);
	}
}

#if defined(SUNTON28)

bool ESPRotary::readSwitch() {
    if( Flarm::bincom )
        return false;
    return (touch_state() == TOUCH_MIDDLE);
}

void ESPRotary::begin() {
    XPT2046_init();
    xTaskCreatePinnedToCore(&ESPRotary::informObservers, "informObservers", 5096, NULL, 14, &pid, 0);
}

static int old_touch = TOUCH_NONE;

#else

bool ESPRotary::readSwitch() {
  if( Flarm::bincom )
    return false;
	gpio_set_direction(sw,GPIO_MODE_INPUT);
	gpio_pullup_en(sw);
	if( gpio_get_level((gpio_num_t)sw) )
		return false;
	else
		return true;
}

void ESPRotary::begin(gpio_num_t aclk, gpio_num_t adt, gpio_num_t asw ) {
	clk = aclk;
	dt = adt;
	sw = asw;

	gpio_set_direction(sw,GPIO_MODE_INPUT);
	gpio_set_direction(dt,GPIO_MODE_INPUT);
	gpio_set_direction(clk,GPIO_MODE_INPUT);
	gpio_pullup_en(sw); // Rotary Encoder Button
	gpio_pullup_en(dt);
	gpio_pullup_en(clk);

	enc.pulse_gpio_num = clk; //Rotary Encoder Chan A
	enc.ctrl_gpio_num = dt;	 //Rotary Encoder Chan B
	enc.unit = PCNT_UNIT_0;
	enc.channel = PCNT_CHANNEL_0;

	enc.pos_mode = PCNT_COUNT_INC; //Count Only On Rising-Edges
	enc.neg_mode = PCNT_COUNT_DIS;	// Discard Falling-Edge

	enc.lctrl_mode = PCNT_MODE_KEEP;    // Rising A on HIGH B = CW Step
	enc.hctrl_mode = PCNT_MODE_REVERSE; // Rising A on LOW B = CCW Step

	enc.counter_h_lim = 32000;
	enc.counter_l_lim = -32000;

	pcnt_unit_config(&enc);
	pcnt_set_filter_value(PCNT_UNIT_0, 250);  // Filter Runt Pulses
	pcnt_filter_enable(PCNT_UNIT_0);

	pcnt_counter_pause(PCNT_UNIT_0); // Initial PCNT init
	pcnt_counter_clear(PCNT_UNIT_0);
	pcnt_counter_resume(PCNT_UNIT_0);

	// second encoder for incrementing on each rotary step:
	enc2.pulse_gpio_num = clk; //Rotary Encoder Chan A
	enc2.ctrl_gpio_num = dt;	 //Rotary Encoder Chan B
	enc2.unit = PCNT_UNIT_1;
	enc2.channel = PCNT_CHANNEL_1;

	enc2.pos_mode = PCNT_COUNT_DIS; //Count Only On Rising-Edges
	enc2.neg_mode = PCNT_COUNT_INC;	// Discard Falling-Edge

	enc2.lctrl_mode = PCNT_MODE_REVERSE;    // Rising A on HIGH B = CW Step
	enc2.hctrl_mode = PCNT_MODE_KEEP; // Rising A on LOW B = CCW Step

	enc2.counter_h_lim = 32000;
	enc2.counter_l_lim = -32000;

	pcnt_unit_config(&enc2);
	pcnt_set_filter_value(PCNT_UNIT_1, 250);  // Filter Runt Pulses
	pcnt_filter_enable(PCNT_UNIT_1);

	pcnt_counter_pause(PCNT_UNIT_1); // Initial PCNT init
	pcnt_counter_clear(PCNT_UNIT_1);
	pcnt_counter_resume(PCNT_UNIT_1);

	xTaskCreatePinnedToCore(&ESPRotary::informObservers, "informObservers", 5096, NULL, 14, &pid, 0);
}

int16_t old_cnt = 0;
// int old_button = RELEASE;

#endif   // SUNTON28

void ESPRotary::sendRelease(){
	// ESP_LOGI(FNAME,"Release action");
	if( Flarm::bincom )
	  return;
	for (auto &observer : observers)
		observer->release();
	// ESP_LOGI(FNAME,"End switch release action");
}

void ESPRotary::sendPress(){
	// ESP_LOGI(FNAME,"Pressed action");
	if( Flarm::bincom )
		return;
	for (auto &observer : observers)
		observer->press();
	// ESP_LOGI(FNAME,"End pressed action");

}

void ESPRotary::sendLongPress(){
	// ESP_LOGI(FNAME,"Long pressed action");
	if( Flarm::bincom )
		return;
	for (auto &observer : observers)
		observer->longPress();
	// ESP_LOGI(FNAME,"End long pressed action");
}

void ESPRotary::sendDown( int diff ){
	// ESP_LOGI(FNAME,"Rotary up action");
	if( Flarm::bincom )
		return;
	for (auto &observer : observers)
		observer->up( diff );   // tbd, clean up, this is named wrong in observers, shoud be down()
}

void ESPRotary::sendEsc(){
	// ESP_LOGI(FNAME,"Rotary Esc action");
	if( Flarm::bincom )
		return;
	for (auto &observer : observers)
		observer->escape();
}

void ESPRotary::sendUp( int diff ){
	// ESP_LOGI(FNAME,"Rotary down action");
	if( Flarm::bincom )
		return;
	for (auto &observer : observers)
		observer->down( diff );   // tbd, dito
}

void ESPRotary::informObservers( void * args )
{
	while( 1 ) {

	  if( Flarm::bincom ) {
	    vTaskDelay(20 / portTICK_PERIOD_MS);
	    continue;
	  }

#if defined(SUNTON28)

		int touch = touch_state();

		if( touch == TOUCH_MIDDLE ){  // pressed
			timer++;
			old_touch = TOUCH_MIDDLE;
			released = false;
			pressed = false;
			up = false;
			down = false;
			if( timer > 25 ){  // > 500 mS
				if( !longPressed ){
					ESP_LOGI(FNAME,"informObservers: sendLongPress");
					longPressed = true;
					sendLongPress();
					sendRelease();
				}
			}

		} else if( touch == TOUCH_NONE ){   // released

			if( old_touch==TOUCH_MIDDLE && !released ){
				// ESP_LOGI(FNAME,"timer=%d", timer );
				if( timer < 25 ){  // < 500 mS
					if( !pressed && !longPressed ){
						ESP_LOGI(FNAME,"informObservers: sendPress");
						pressed = true;
						sendPress();
						sendRelease();
					}
				}
			}
			else if ( old_touch==TOUCH_UP && !up && !released ) {
				ESP_LOGI(FNAME,"informObservers: sendUp");
				up = true;
				sendUp( 1 );
			}
			else if ( old_touch==TOUCH_DOWN && !down && !released ) {
				ESP_LOGI(FNAME,"informObservers: sendDown");
				down = true;
				sendDown( 1 );
			}

			// cleanup after release:
			old_touch = TOUCH_NONE;
			released = true;
			//sendRelease();
			longPressed = false;
			timer = 0;
			delay( 40 );

		} else {   // UP or DOWN

			old_touch = touch;
			timer++;
			if( timer > 12 ){   // long-pressed > 240 mS - repeat action
				if( touch == TOUCH_UP && !up ){
					ESP_LOGI(FNAME,"repeat up");
					up = true;
                    sendUp( 1 );
				} else
				if( touch == TOUCH_DOWN && !down ){
					ESP_LOGI(FNAME,"repeat down");
					down = true;
                    sendDown( 1 );
				}
				timer = 0;
				delay( 40 );

			} else {      // send up or down later, when released
				released = false;
				//pressed = false;
				up = false;
				down = false;
			}
		}

#else  // not SUNTON28

		int button = gpio_get_level((gpio_num_t)sw);
		if( button == 0 ){  // Push button is being pressed
			timer++;
			released = false;
			pressed = false;
			if( timer > 20 ){  // > 400 mS
				if( !longPressed ){
					longPressed = true;
					sendLongPress();
					sendRelease();

				}
			}
		}
		else{   // Push button is being released
			if( !released ){
				// ESP_LOGI(FNAME,"timer=%d", timer );
				longPressed = false;
				if( timer < 20 ){  // > 400 mS
					if( !pressed ){
						pressed = true;
						sendPress();
						sendRelease();
					}
				}
				timer = 0;
				released = true;
				delay( 20 );
			}
		}
		if( pcnt_get_counter_value(PCNT_UNIT_0, &r_enc_count) != ESP_OK ) {
			ESP_LOGE(FNAME,"Error get counter");
		}
		if( pcnt_get_counter_value(PCNT_UNIT_1, &r_enc2_count) != ESP_OK ) {
			ESP_LOGE(FNAME,"Error get counter");
		}

		if( abs( r_enc_count+r_enc2_count - old_cnt) > rotary_inc.get() )
		{
			// pcnt_counter_clear(PCNT_UNIT_0);
			// ESP_LOGI(FNAME,"Rotary counter %d %d", r_enc_count,  r_enc2_count);
			int diff = (r_enc_count+r_enc2_count) - old_cnt;
			diff = diff / ( rotary_inc.get()+1 );
			// ESP_LOGI(FNAME,"Rotary diff %d", diff );
			if( hardwareRevision.get() >= XCVARIO_21 ) {
				if( rotary_dir_21.get() == 1 ) // reverse default for 2021 series
					diff = -diff;
			}
			else{
				if( rotary_dir.get() == 1 )    // reverse type default for 2020 series
					diff = -diff;
			}
			old_cnt = r_enc_count+r_enc2_count;
			if( diff < 0 ) {
				// ESP_LOGI(FNAME,"Rotary down %d times",abs(diff) );
				sendDown( abs(diff) );
			}
			else{
				// ESP_LOGI(FNAME,"Rotary up %d times", abs(diff) );
				sendUp( abs(diff) );
			}
		}

#endif   // SUNTON28

		if( uxTaskGetStackHighWaterMark( pid ) < 256 )
			ESP_LOGW(FNAME,"Warning rotary task stack low: %d bytes", uxTaskGetStackHighWaterMark( pid ) );
		vTaskDelay(20 / portTICK_PERIOD_MS);
	}
}
