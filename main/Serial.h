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
#define GPIO_RXD1 GPIO_NUM_35
  // - in expansion connector - should add pull-up?  Can do that in software?
#define GPIO_TXD1 GPIO_NUM_22
  // in expansion connector next to GPIO35 (if not using I2C)
//#define GPIO_TXD1 GPIO_NUM_27
  // alternative: temp probe connector
  //   - same connector also has +3.3V for MAX232 (but can do without that!)
#define GPIO_RXD2 GPIO_NUM_0
  // - only connected to "boot" switch (and USB-serial RTS)
  //    (boot switch also used for alternative "rotary" pusbutton)
  // - or use GPIO 19, also used for SD card (VSPI) MISO
#define GPIO_TXD2 GPIO_NUM_16     // connected to the blue LED
#define GPIO_NOTX1 GPIO_NUM_4     // green LED
#define GPIO_NOTX2 GPIO_NUM_16    // blue LED
#else
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
