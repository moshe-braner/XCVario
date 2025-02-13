#include <esp_log.h>
#include "BLESender.h"
#include <string>
#include "sdkconfig.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include <freertos/semphr.h>
#include <algorithm>
#include "RingBufCPP.h"
#include <driver/uart.h>
#include "Protocols.h"
#include <logdef.h>
#include "Switch.h"
#include "sensor.h"
#include "Router.h"
#include "Flarm.h"
#include "DataMonitor.h"
#include "DataLink.h"
#include "SetupNG.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;

static TaskHandle_t pid = nullptr;
static DataLink *dlb;
static int tick=0;
static uint16_t peer_mtu;
static int congestion=0;

#define MAX_CONGESTION 300

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// To work with XCsoar, our UUIDs need to be in XCsoar's whitelist.
// For now need to pretend to be an HM-10 BT adapter.
// from XCSoar/android/src/BluetoothUuids.java:
//  UUID HM10_SERVICE = "0000FFE0-0000-1000-8000-00805F9B34FB"
// The HM-10 and compatible bluetooth modules use a (single) GATT characteristic
//    with this UUID for sending and receiving data:
//  UUID HM10_RX_TX_CHARACTERISTIC = "0000FFE1-0000-1000-8000-00805F9B34FB"
#define UART_SERVICE_UUID        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define UART_CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"

class MyServerCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		ESP_LOGI(FNAME,"BT LE device connected" );
		deviceConnected = true;
		peer_mtu = pServer->getPeerMTU(pServer->getConnId());
		ESP_LOGI(FNAME, "Peer MTU: %d", peer_mtu);
	};

	void onDisconnect(BLEServer* pServer) {
		ESP_LOGI(FNAME,"BT LE device disconnected" );
		deviceConnected = false;
		pServer->startAdvertising(); // restart advertising
	}
};

class MyRxCallbacks: public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		std::string rx = pCharacteristic->getValue();
		if (rx.length()) {
			dlb->process( rx.c_str(), rx.length(), 7 );
			DM.monitorString( MON_BLUETOOTH, DIR_RX, rx.c_str(), rx.length() );
			ESP_LOGI(FNAME,">BT LE RX: %d bytes",  rx.length()  );
			if ( testmode.get() ) {
				ESP_LOG_BUFFER_HEXDUMP(FNAME,rx.c_str(), rx.length() , ESP_LOG_INFO);
			}
		}
	}
};

class MyTxCallbacks: public BLECharacteristicCallbacks {
	void onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code) override {
		// ESP_LOGI(FNAME, "onStatus Status %d code: %d", s, code );
		if (s == BLECharacteristicCallbacks::ERROR_GATT && code == -1) {  // upon congestion, when using notify, the status logs ERROR_GATT
			// ESP_LOGI(FNAME, "Congested, pacing: %d", congestion );
			if( congestion < MAX_CONGESTION )
				congestion++;
		}
		if (s == BLECharacteristicCallbacks::SUCCESS_NOTIFY && code == 0) {
			// ESP_LOGI(FNAME, "Send OK, pacing: %d", congestion );
			if( congestion )
				congestion--;
		}
	}
};

bool BLESender::selfTest(){
	ESP_LOGI(FNAME,"SerialBLE::selfTest");
	return true;
}

int BLESender::queueFull() {
	if( wireless == WL_BLUETOOTH_LE ){
		if(bt_tx_q.isFull())
			return 1;
	}
	return 0;
}

void BLESender::btTask(void *pvParameters){
	while(1) {
		progress();
		Router::routeBT();
		if( uxTaskGetStackHighWaterMark( pid ) < 256 )
			ESP_LOGW(FNAME,"Warning BT task stack low: %d bytes", uxTaskGetStackHighWaterMark( pid ) );
		vTaskDelay( 100/portTICK_PERIOD_MS );
		tick++;
	}
}

void BLESender::progress(){
	char buf[256];
	int max_packet_size = std::min(peer_mtu - 3, 250);
	if (deviceConnected) {
		int len = Router::pullBlock( bt_tx_q, buf, max_packet_size );
		if( len ){
			// ESP_LOGI(FNAME,"BLE len=%d P:%d, %s",len, congestion, buf);
			int sent=min( len, max_packet_size );
			pTxCharacteristic->setValue((uint8_t*)buf, (size_t)sent);
			pTxCharacteristic->notify(); // No return value
			// ESP_LOGI(FNAME,"<BT LE TX %d bytes (pending: %d)", sent, indicationPending );
			// ESP_LOG_BUFFER_HEXDUMP(FNAME,&buf[pos],len, ESP_LOG_INFO);
			DM.monitorString( MON_BLUETOOTH, DIR_TX, buf, len );
			delay( congestion );  // slow down sender upon congestion
		}
	}
}

void BLESender::begin(){
	ESP_LOGI(FNAME,"BLESender::begin()" );
	ESP_LOGI(FNAME,"BT LE on, create BT master object" );

	// esp_log_level_set("GATTS", ESP_LOG_DEBUG);
	// esp_log_level_set("BT_BTM", ESP_LOG_DEBUG);

	dlb = new DataLink();
	// Create the BLE Device
	std::string ble_id( SetupCommon::getID() );
	ble_id += std::string("-LE");
	BLEDevice::init(ble_id.c_str());

	// Create the BLE Server
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	// Create the BLE Service
#if 0
	BLEService *pService = pServer->createService(SERVICE_UUID);
#else
	BLEService *pService = pServer->createService(UART_SERVICE_UUID);
#endif

	// Create a BLE Characteristic
	pTxCharacteristic = pService->createCharacteristic(
			UART_CHARACTERISTIC_UUID,                     // was CHARACTERISTIC_UUID_TX
			BLECharacteristic::PROPERTY_NOTIFY
//			| BLECharacteristic::PROPERTY_READ
//			| BLECharacteristic::PROPERTY_WRITE_NR        // may also be needed
	);

// GATT Descriptor 0x2901 Characteristic User Description
// GATT Descriptor 0x2902 Client Characteristic Configuration
	BLEDescriptor *pUserDescriptor = new BLEDescriptor("2901");
	pUserDescriptor->setValue("HMSoft");
	pTxCharacteristic->addDescriptor(pUserDescriptor);
	pTxCharacteristic->addDescriptor(new BLE2902());

	BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
			UART_CHARACTERISTIC_UUID,                     // was CHARACTERISTIC_UUID_RX,
			BLECharacteristic::PROPERTY_WRITE
	);

	pTxCharacteristic->setCallbacks(new MyTxCallbacks());
	pRxCharacteristic->setCallbacks(new MyRxCallbacks());

	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

	// Tune advertising parameters
	pAdvertising->setMinInterval(0x20);  // Minimum advertising interval (32 * 0.625 ms = 20 ms)
	pAdvertising->setMaxInterval(0x40);  // Maximum advertising interval (64 * 0.625 ms = 40 ms)

	// Start the service
	pService->start();

	// Start advertising
	pServer->getAdvertising()->start();

	xTaskCreatePinnedToCore(&btTask, "bleTask", 4096, NULL, 12, &pid, 0);  // stay below compass task
}
