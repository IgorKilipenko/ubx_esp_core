#include "ATcpServer.h"

ATcpServer::ATcpServer(HardwareSerial *receiver, SDStore *store, int port) : clients{MAX_TCP_CLIENTS, nullptr}, _receiver{receiver}, _store{store}, _port{port} {}
ATcpServer::~ATcpServer() { end(); }

void ATcpServer::processData(std::vector<char> data) {
	
	std::lock_guard<std::mutex> mlock(_mutex);
	
	int bytesCount = data.size();
	const char * buffer = data.data();

	if (bytesCount > 0) {
		if (receiveData) {
			if (_writeToSd && _store->isInitialize() && _store->isOpenFile()) {
				_store->writeToSD(buffer, bytesCount);
				delay(1);
			}

			if (_sendToTcp && WiFi.status() == WL_CONNECTED) {
				sendDataToClients(buffer, bytesCount);
			}
		}

		if (_seralDataCallback != nullptr) {
			_seralDataCallback(std::vector<uint8_t>(data.begin(), data.end()));
		}

	} else if (bytesCount < 0) {
		log_d("Error reading serial, available bytes count: [%d]\n", bytesCount);
	}
}

void ATcpServer::handleError(AsyncClient *client, int8_t error) { log_v("\n Connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str()); }

void ATcpServer::handleData(AsyncClient *client, void *data, size_t len) {
	uint8_t buffer[len]{0};

	log_d("\n Handle data -> Data from client to receiver %s: \n", client->remoteIP().toString().c_str());
	log_v("=========================\n");
	// logger.write((uint8_t *)data, len);
	log_v("\n-- packet count : [%i] bytes--\n", len);
	log_v("=========================\n");

	memcpy(buffer, (uint8_t *)data, len);
	_receiver->write(buffer, len);
}

void ATcpServer::handleDisconnect(AsyncClient *client) {
	log_d("\n Client %s disconnected \n", client->remoteIP().toString().c_str());
	// delete client;
	// client = nullptr;
	const size_t availableClients = freeClients();
	log_d("Available clints count: %i \n", availableClients);
}

void ATcpServer::handleTimeOut(AsyncClient *client, uint32_t time) { log_v("\n Client ACK timeout ip: %s \n", client->remoteIP().toString().c_str()); }

void ATcpServer::handleNewClient(AsyncClient *client) {
	log_v("\n New client connecting to server, ip: %s\n", client->remoteIP().toString().c_str());

	// add to list
	int i = 0;
	for (; i < MAX_TCP_CLIENTS; i++) {
		if (!clients[i] || clients[i]->free()) {
			clients[i] = client;
			log_v("Client connected, ip address: %s\n\t-> Connected clients count: %i \n", client->remoteIP().toString().c_str(), availableClientsCount());
			break;
		}
	}

	if (i == MAX_TCP_CLIENTS) {
		// reply to client
		const char reply[] = "403 Forbidden";
		sendMessage(client, reply, strlen(reply));
		log_v("Client not connected, server is busy with %d active connections\n", MAX_TCP_CLIENTS);
		client->close(true);
	}

	// register events
	client->onData([](void *r, AsyncClient *c, void *buf, size_t len) { ((ATcpServer *)(r))->handleData(c, buf, len); }, this);
	client->onError([](void *r, AsyncClient *c, int8_t error) { ((ATcpServer *)(r))->handleError(c, error); }, this);
	client->onDisconnect([](void *r, AsyncClient *c) { ((ATcpServer *)(r))->handleDisconnect(c); }, this);
	client->onTimeout([](void *r, AsyncClient *c, uint32_t time) { ((ATcpServer *)(r))->handleTimeOut(c, time); }, this);
}

bool ATcpServer::isInProgress() const { return receiveData; }
bool ATcpServer::isSdInitialize() const { return _store != nullptr && _store->isInitialize(); }

void ATcpServer::stopReceive() {
	if (_store) {
		log_d("Stop store\n");
		_store->closeFile();
	}
	if (server) {
		server->end();
	}
	receiveData = false;
	_timeEnd = millis();
}

void ATcpServer::startReceive(bool writeToSd, bool sendToTcp) {
	log_v("Receiver starting...\n");
	log_v("Write to SD card : [%s]\n", writeToSd ? "ENABLED" : "DISABLED");
	log_v("Send to TCP : [%s]\n", sendToTcp ? "ENABLED" : "DISABLED");

	_writeToSd = writeToSd;
	_sendToTcp = sendToTcp;

	if (_writeToSd && (_store->isInitialize() || _store->initSdCard())) {
		log_v("Stored initialized\n");
		_store->createFile();
		log_v("File created\n");
	}

	// server->setNoDelay(true);
	if (_sendToTcp) {
		server->begin();
		log_v("TCP Server started\n");
	}

	receiveData = true;
	_timeEnd = 0;
	_timeStart = millis();

	log_v("Receive data started\n");
}

unsigned long ATcpServer::getTimeReceive() const {
	if (!isInProgress()) {
		return 0;
	}
	return millis() - _timeStart;
}

unsigned long ATcpServer::getTimeStart() const {
	if (!isInProgress()) {
		return 0;
	}
	return _timeStart;
}

void ATcpServer::end() {
	freeClients();
	clients.clear();

	if (server) {
		delete server;
	}
	server = nullptr;
}

size_t ATcpServer::freeClients() {
	size_t clientCount = 0;
	for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
		if (!clients[i] || clients[i]->free()) {
			if (clients[i]) {
				delete clients[i];
			}
			clients[i] = nullptr;
		} else {
			clientCount++;
		}
	}
	return clientCount;
}

size_t ATcpServer::sendMessage(AsyncClient *client, const char *msg, size_t len) {
	const size_t will_send = client->add(msg, len);
	if (!will_send || !client->send()) {
		return 0;
	}
	return will_send;
}

size_t ATcpServer::sendMessage(AsyncClient *client, String str) {
	const char *msg = str.c_str();
	return sendMessage(client, msg, strlen(msg));
}

void ATcpServer::setup() {
	if (_writeToSd) {
		_store->initSdCard();
	}
	server = new AsyncServer(_port);
	server->onClient([](void *s, AsyncClient *c) { ((ATcpServer *)(s))->handleNewClient(c); }, this);
}

void ATcpServer::sendDataToClients(const char *buffer, size_t bytesCount) {
	log_v("Send data to clients\n");
	for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
		if (clients[i] && clients[i]->connected()) {
			sendMessage(clients[i], buffer, bytesCount);
		}
	}
}

size_t ATcpServer::availableClientsCount() {
	size_t clientCount = 0;
	for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
		if (clients[i]) {
			clientCount++;
		}
	}
	return clientCount;
}