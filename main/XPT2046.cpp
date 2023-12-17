/*---------------------------------------------------------------------------------/
  Support the XPT2046 touch-screen controller of the Sunton ESP32-2432S028 board

  Adapted from:
    Lovyan GFX - Graphics library for embedded devices.
    https://github.com/lovyan03/LovyanGFX/

  This version does NOT use SPI - manually manipulates the pins instead.
----------------------------------------------------------------------------------*/

#include <Arduino.h>

#include <stdint.h>
#include <stddef.h>
#include <logdef.h>
//#include <esp_log.h>

#include "XPT2046.h"

static bool inited = false;

void XPT2046_init()
{
    pinMode(TOUCH_CS, OUTPUT);
    digitalWrite(TOUCH_CS, HIGH);
    pinMode(TOUCH_CLK, OUTPUT);
    //digitalWrite(TOUCH_CLK, LOW);  // <<< added
    pinMode(TOUCH_MOSI, OUTPUT);
    pinMode(TOUCH_MISO, INPUT_PULLUP);
    inited = true;
    ESP_LOGI(FNAME,"init() done"); 
}

static void transfer(uint8_t* read_data, const uint8_t* write_data, size_t len)
{
    if (len > 100)  len = 100;  // guard against do-while loop below taking too long
    digitalWrite(TOUCH_CS, LOW);
    do {
      uint_fast8_t r = 0;
      uint_fast8_t mask = 0x80;
      uint_fast8_t d = *write_data++;
      //semaphoreTake();
      do {
        digitalWrite(TOUCH_CLK, LOW);
        if (d & mask)
            digitalWrite(TOUCH_MOSI, HIGH);
        else
            digitalWrite(TOUCH_MOSI, LOW);
        digitalWrite(TOUCH_CLK, HIGH);
        if (digitalRead(TOUCH_MISO))
            r |= mask;
      } while (mask >>= 1);
      //semaphoreGive();
      *read_data++ = (uint8_t)r;
    } while (--len);
    digitalWrite(TOUCH_CLK, LOW);
    digitalWrite(TOUCH_CS, HIGH);
}

bool getTouch(int16_t &rx, int16_t &ry)
{
    if (!inited)
      return false;

    uint8_t data[57];

    data[ 0] = 0x91;
    data[ 1] = 0;
    data[ 2] = 0xB1;
    data[ 3] = 0;
    data[ 4] = 0xD1;
    data[ 5] = 0;
    data[ 6] = 0xC1;
    data[ 7] = 0;
    memcpy(&data[ 8], data,  8);
    memcpy(&data[16], data, 16);
    memcpy(&data[32], data, 24);
    data[56] = 0x80; // last power off.

    transfer(data, data, 57);

    uint_fast8_t ix = 0, iy = 0, iz = 0;
    uint32_t sumx=0, sumy=0, sumz=0;

    // keep track of 2 largest and 2 smallest outliers
    uint_fast16_t minx = X_MAX;
    uint_fast16_t min2x = X_MAX;
    uint_fast16_t maxx = X_MIN;
    uint_fast16_t max2x = X_MIN;
    uint_fast16_t miny = Y_MAX;
    uint_fast16_t min2y = Y_MAX;
    uint_fast16_t maxy = Y_MIN;
    uint_fast16_t max2y = Y_MIN;

    for (size_t j = 0; j < 7; ++j)
    {
      auto d = &data[j * 8];
      int x = (d[5] << 8 | d[6]) >> 3;
      int y = (d[1] << 8 | d[2]) >> 3;
      int z = 0x3200 + y - x
            + (((d[3] << 8 | d[4])
              - (d[7] << 8 | d[8])) >> 1);
      if (x > X_MIN && x < X_MAX)
      {
        ix++;
        sumx += x;
        if (x > maxx) {
            max2x = maxx;
            maxx = x;
        } else if (x > max2x) {
            max2x = x;
        }
        if (x < minx) {
            min2x = minx;
            minx = x;
        } else if (x < min2x) {
            min2x = x;
        }
      }
      if (y > Y_MIN && y < Y_MAX)
      {
        iy++;
        sumy += y;
        if (y > maxy) {
            max2y = maxy;
            maxy = y;
        } else if (y > max2y) {
            max2y = y;
        }
        if (y < miny) {
            min2y = miny;
            miny = y;
        } else if (y < min2y) {
            min2y = y;
        }
      }
      if (z > 0) {
        iz++;
        sumz += z;
      }
    }

    // require at least 5 (of possible 7) so that
    //    after dropping 4 outliers at least 1 is left
    if (ix < 5 || iy < 5 || iz < 5) {
        if (iz > 0 && (ix > 0 || iy > 0))
            ESP_LOGI(FNAME,"weak touch %d, %d, %d", ix, iy, iz); 
        return false;
    }

    // instead of sorting to get the median,
    //    drop outliers and average the rest
    sumx -= minx + min2x + maxx + max2x;
    rx = sumx / (ix-4);
    sumy -= miny + min2y + maxy + max2y;
    ry = sumy / (iy-4);

    return ((sumz > (((uint32_t)(iz-4))<<8)) ? true : false);
}
