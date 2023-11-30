/*---------------------------------------------------------------------------------/
  Support the XPT2046 touch-screen controller of the Sunton ESP32-2432S028 board

  Adapted from:
  Lovyan GFX - Graphics library for embedded devices.
  https://github.com/lovyan03/LovyanGFX/

  This version does NOT use SPI.
----------------------------------------------------------------------------------*/

// for now hard-code the hardware details

#define TOUCH_CS   GPIO_NUM_33
#define TOUCH_CLK  GPIO_NUM_25
#define TOUCH_MOSI GPIO_NUM_32
#define TOUCH_MISO GPIO_NUM_39

#define X_MIN 300
#define X_MAX 3900
#define Y_MIN 200
#define Y_MAX 3700

extern void XPT2046_init();
extern bool getTouch(int16_t &x, int16_t &y);
