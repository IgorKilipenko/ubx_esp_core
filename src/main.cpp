/* ESP DEBUG LEVEL CONSTANT
	ESP_LOG_VERBOSE = 5
	ESP_LOG_DEBUG = 4
	ESP_LOG_INFO = 3
	ESP_LOG_WARN = 2
	ESP_LOG_ERROR = 1
	ESP_LOG_NONE = 0
*/
//#define LOG_LOCAL_LEVEL 3

#include <Arduino.h>
#include <thread>
#include <string>

/* SD card */
#ifdef ESP32
#ifdef TTGO_BOARD
#define CS_PIN 13
#define RXD2 32
#define TXD2 33
#define RXD1 13
#define TXD1 23
#else
#define CS_PIN 5 // SD card cs_pin (default D5 GPIO5 on ESP32 DevkitV1)
#define RXD2 16
#define TXD2 17
#define RXD1 4
#define TXD1 15
#endif // TTGO_BOARD
#else
#error Platform not supported
#endif // ESP32

HardwareSerial *RTCM{&Serial1};
HardwareSerial *Receiver{&Serial2};

/* TCP Client */
#define MAX_TCP_CLIENTS 5 // Default max clients
#define TCP_PORT 7042	 // Default tcp port (GPS receiver communication)

/* Serial */
#define BAUD_SERIAL 115200	// Debug Serial baund rate
#define BAUND_RECEIVER 921600 // GPS receiver baund rate
#define SERIAL_SIZE_RX 1024

#include "utils.h"
#include "ATcpServer.h"
#include "WiFiWithEvents.h"
#include "Queue.h"

WiFiWithEvents wifi{};
SDStore store{CS_PIN};
ATcpServer telnetServer{Receiver, &store, TCP_PORT}; // GPS receiver communication

std::thread receiverThread;
Queue<std::vector<char>> queue_buffer;

void readReceiverData() {
	int count = 0;
	if ((count = Receiver->available()) > 0) {
		std::vector<char> buffer(count);
		int len = Receiver->readBytes(buffer.data(), count);
		if (len != count) {
			log_e("Not all bytes read from uart, available: [%i], read: [%i]\n", count, len);
		}
		if (len > 0) {
			queue_buffer.push(buffer);
		}
	}
}

void setup() {
	Serial.begin(BAUD_SERIAL);
	Receiver->begin(BAUND_RECEIVER, SERIAL_8N1, RXD2, TXD2);
	RTCM->begin(38400, SERIAL_8N1, RXD1, TXD1);

	Receiver->setRxBufferSize(SERIAL_SIZE_RX);
	std::string hostName{"ESP_GPS_"};
	hostName.append(utils::getEspChipId().c_str());

	wifi.connectSta();

	telnetServer.setup();
	telnetServer.startReceive(false, true);
	receiverThread = std::thread([]() {
		vTaskPrioritySet(nullptr, 1);
		for (;;) {
			readReceiverData();
			delay(1);
		}
	});
	receiverThread.detach();
	// receiverThread.join();
}

/*volatile*/
void loop() {
	while (!queue_buffer.empty()) {
		auto data = queue_buffer.pop();
		std::thread sender([&] {
			vTaskPrioritySet(nullptr, 1);
			telnetServer.processData(data);
		});
		sender.join();
		delay(1);
	}
	delay(1);
}
