/*/
 *   Gateway.h - Header for TCP/RTU Modbus Gateway
 *   Copyright (C) 2022 Hadler Henrique Tempesta
/*/

#ifndef GATEWAY_H
#define GATEWAY_H

#include "Arduino.h"
#include <Ethernet.h>
#include <SPI.h>
#include <Stream.h>

#define KEEP_TRYING 0
#define GATEWAY_CONNECTED  0
#define MISSING_PARAMETERS 1
#define HARDWARE_NOT_FOUND 2
#define FAILED_TO_CONNECT  3

//Configuration
//#define DATA_VALIDATION
#define SERIAL_FEEDBACK
//#define HARDWARE_FEEDBACK

#if defined(HARDWARE_FEEDBACK)
#define RTU_TO_TCP_PIN 2
#define TCP_TO_RTU_PIN 3
#define CONNECTION_PIN 5
#endif

class Gateway {

	public :
		Gateway(
		    IPAddress *_ip = NULL,
		    IPAddress *_subnet = NULL
		);

		void configRTU(
		    HardwareSerial *_serial,
		    uint32_t baudrate,
		    uint8_t serialConfig = SERIAL_8N1
		);

		uint8_t configTCP(
		    IPAddress _serverIP,
		    uint32_t _serverPort = 502,
		    uint16_t timeOut = 500,
		    uint8_t tryToConnect = KEEP_TRYING
		);

		void run(void);

	protected:
		EthernetClient client;
		IPAddress *selfIP, *subnet;
		IPAddress serverIP;
		HardwareSerial *serial;

		uint16_t t35us = 1750;
		uint32_t serverPort;

		uint16_t CRC16(uint8_t *data, uint16_t dSize, uint16_t startByte = 0);
		void RTU_to_TCP(void);
		void TCP_to_RTU(void);
		bool isIPNull(IPAddress* ip);

	private:
		IPAddress null_IP = IPAddress(0, 0, 0, 0);
		const uint8_t mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

#if defined(DATA_VALIDATION)
		const uint8_t modbusFunctions[19] = {
			0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0F, 0x10,
			0x07, 0x08, 0x0B, 0x0C, 0x11,
			0x14, 0x15, 0x16, 0x17, 0x18, 0x2B
		};

#endif


};

#endif
