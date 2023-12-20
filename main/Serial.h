/**
 * Serial.h
 *
 * 02.01.2022 Axel Pauli: handling of 2 uart channels in one method accomplished.
 * 01.01.2022 Axel Pauli: updates after first delivery.
 * 24.12.2021 Axel Pauli: added some RX/TX handling stuff.
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <cstring>
#include "SString.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include "RingBufCPP.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "Router.h"
#include "HardwareSerial.h"
#include "DataLink.h"

#if defined(SUNTON28)
//#define GPIO_TXD1 GPIO_NUM_21
  // - in expansion connector along with GPIO35
  //   (if not using I2C where it's SDA by default)
  // GPIO21 also needs to be high to activate TFT backlight
  // but the serial output is usually high, TFT will be "OK"?
// - appears that using gpio 21 makes the display blink badly
  // If not using TX then set GPIO_TXD1 to green or blue LED
  // Or perhaps can use GPIO21 for I2C in parallel with the TFT backlight:
  //    - in I2C "both SCL and SDA are high in idle state"
  // Another alternative is gpio27: temp probe connector
  //    - same connector also has +3.3V
  //
  // The options:
  //              serial      I2C
  //            RXD1  TXD1  SCL  SDA
  //             35    22   --   --  (should add pull-up to 35?  Can do in software?)
  //             35    21   --   --  (21 shared with TFT backlight)
  //             35    27   --   --  (note that on the ESP32 GPIO35 is input-only)
  //             35    22   21   27  (use 21 for SCL since SCL is output-only, but:)
  //             35    27   21   22  (note: BMP module has pull-up resistor on SCL)
  //             35    --   21   22  (this pinout opposite the default I2C pinout)
  //            (35    27   22   21  DON'T DO THIS, SINCE WE *OUTPUT* HIGH TO GPIO21)
  //             35    21   27   22  (this may cause less (or more) TFT blink?)
  //             35    21   22   27  (asks peripheral to pull down against the resistor on 27)
  // Conclusion:
  //  try this:  35    21   27   22  (but carefully protect 35,21 from high outside voltages)
  //        - but it appears that using gpio 21 makes the display blink badly,
  //            & tx-off makes the display black, so:
  //  use this:  35    --   27   22  (serial output not available, use wireless)
  //   or this:  35    22   --   --  (I2C not available)
  //   or this:  35    27   21   22  (if I2C does not affect display too much)
  //  (or this:  35    22   21   27) (ditto, but pull-up on 27 may stress the BMP?)
  // Note:
  //   S1 is separate from the USB/serial debug port!
  //   S1 output will be busy unless set to no-TX or S1 routing turned off
  //   I2C (if connected to baro sensor) will be used 10 episodes per second?
  //
#define GPIO_RXD1 GPIO_NUM_35
  // - in expansion connector - should add pull-up?  Can do that in software?
  //    - can use just RX, and skip S1 TX, if want to use 22,27 for I2C
  //    - note that on the ESP32 GPIO35 is input-only
#define GPIO_TXD1_I2C21 GPIO_NUM_27
#define GPIO_TXD1_I2C27 GPIO_NUM_17
#define GPIO_TXD1_NO_I2C GPIO_NUM_22
#define GPIO_TXD1 (gpio_txd1())
#define GPIO_NOTX1 GPIO_NUM_17   // connected to the ____ LED
//  (GPIO 4 seems to be the *red* LED which is also always on dimly)
#define GPIO_RXD2 GPIO_NUM_0
  // - only connected to "boot" switch (and USB-serial RTS)
  //    (boot switch also used for alternative "rotary" pusbutton)
  // - or use GPIO 19, also used for SD card (VSPI) MISO
#define GPIO_TXD2 GPIO_NUM_16     // connected to the ____ LED
#define GPIO_NOTX2 GPIO_NUM_16
#else
// these are the pins used in the XCvario:
#define GPIO_RXD1 GPIO_NUM_16
#define GPIO_TXD1 GPIO_NUM_17
#define GPIO_RXD2 GPIO_NUM_4
#define GPIO_TXD2 GPIO_NUM_18
#define GPIO_NOTX1 GPIO_NUM_36
#define GPIO_NOTX2 GPIO_NUM_36
#endif

#define SERIAL_STRLEN SSTRLEN

// Event mask definitions
#define RX0_CHAR 1
#define RX0_NL 2
#define RX1_CHAR 4
#define RX1_NL 8
#define RX2_CHAR 16
#define RX2_NL 32
#define TX1_REQ 64
#define TX2_REQ 128

// All data of one Uart channel
typedef struct xcv_serial {
	const char* name;
	RingBufCPP<SString, QUEUE_SIZE>* tx_q;
	RingBufCPP<SString, QUEUE_SIZE>* rx_q;
	void (*route)();
	HardwareSerial *uart;
	uint8_t rx_char;
	uint8_t rx_nl;
	uint8_t tx_req;
	uint8_t monitor;
	TaskHandle_t pid;
	xcv_serial *cfg2; // configuration of other Uart
	bool route_disable;
	DataLink *dl;
	int port;
} xcv_serial_t;

class Serial {
public:
	Serial(){
	}

	static void begin();
	static void taskStart();
	static void serialHandler(void *pvParameters);
	static bool selfTest( int num );
	/*
	 * Uart event bits
	 * Bit 0: Uart 0 RX any character received
	 * Bit 1: Uart 0 RX nl received
	 * Bit 2: Uart 1 RX any character received
	 * Bit 3: Uart 1 RX nl received
	 * Bit 4: Uart 2 RX RX any character received
	 * Bit 5: Uart 2 RX nl received
	 * Bit 6: Uart 1 TX characters to send
	 * Bit 7: Uart 1 TX characters to send
	 */
	static void setRxTxNotifier( const uint8_t eventMask )
	{
		if( rxTxNotifier )
			xEventGroupSetBits( rxTxNotifier, eventMask );
	};

	/*
	 *  Pacing for second serial interface, disable interrupt, clear queue, etc.
	 */
	static void enterBincomMode( xcv_serial_t *cfg );

	/*
	 *  Exit binary mode
	 */
	static void exitBincomMode( xcv_serial_t *cfg );

	/**
	 * Stop data routing of the Uart channel.
	 */
	static void setroutingStopped( xcv_serial_t *cfg, const bool flag )
	{
		cfg->route_disable = flag;
	};

	/**
	 * Query the stop routing flag.
	 */
	static bool routingStopped( xcv_serial_t *cfg )
	{
		return cfg->route_disable;
	};

private:
	static bool _selfTest;
	static EventGroupHandle_t rxTxNotifier;
	// Stop routing of TX/RX data. That is used in case of Flarm binary download.
	static bool bincom_mode;
	static xcv_serial_t S1;
	static xcv_serial_t S2;
};

#endif
