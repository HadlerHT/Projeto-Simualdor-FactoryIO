/*  /
     Gateway.h - Source for TCP/RTU Modbus Gateway
     Copyright (C) 2022 Hadler Henrique Tempesta
    /*/

#include "Gateway.h"


Gateway::Gateway(IPAddress *_ip = NULL, IPAddress *_subnet = NULL) {
    
    selfIP = _ip;
    subnet = _subnet;

#ifdef HARDWARE_FEEDBACK
    pinMode(RTU_TO_TCP_PIN, OUTPUT);
    pinMode(TCP_TO_RTU_PIN, OUTPUT);
    pinMode(CONNECTION_PIN, OUTPUT);
#endif
}


void Gateway::configRTU(HardwareSerial* _serial, uint32_t baudrate, uint8_t serialConfig = SERIAL_8N1) {

    serial = _serial;
    (*serial).begin(baudrate, serialConfig);

    if (baudrate < 22000)
        t35us = (uint16_t)((uint32_t)(38500000 / baudrate));

    while ((*serial).available() > 0)
        (*serial).read();
}


uint8_t Gateway::configTCP(IPAddress _serverIP, uint32_t _serverPort = 502, uint16_t timeOut = 500, uint8_t tryToConnect = KEEP_TRYING) {

    serverIP = _serverIP;
    serverPort = _serverPort;

    //    Four possible returns:
    //    GATEWAY_CONNECTED
    //    MISSING_PARAMETERS
    //    HARDWARE_NOT_FOUND
    //    FAILED_TO_CONNECT

#ifdef SERIAL_FEEDBACK
    Serial.begin(115200);
    delay(100);
    Serial.println(F("Initializing Ethernet Module."));
    uint32_t startTime = millis();
#endif

    if (selfIP != NULL)
        Ethernet.begin(mac, *selfIP);
    else
        Ethernet.begin(mac);

    if (subnet != NULL and * subnet != Ethernet.subnetMask())
        Ethernet.setSubnetMask(*subnet);


    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
#if defined (SERIAL_FEEDBACK)
        Serial.println(F("Hardware Not Found"));
#endif

        return HARDWARE_NOT_FOUND;
    }

#ifdef SERIAL_FEEDBACK
    Serial.print(millis() - startTime);
    Serial.println(F("ms"));
    Serial.println(F("\nEthernet Module Started."));
    Serial.print(F("Local IP: \t"));
    Serial.println(Ethernet.localIP());
    Serial.print(F("Subnet: \t"));
    Serial.println(Ethernet.subnetMask());
#endif

    client.setConnectionTimeout(timeOut);

#ifdef SERIAL_FEEDBACK
    Serial.println("\nTrying to connect to server.");
#endif

    for (uint8_t attempt = 0; attempt < tryToConnect || tryToConnect == 0; attempt++) {
        if (client.connect(serverIP, serverPort))
            break;

#ifdef SERIAL_FEEDBACK
        Serial.print(F("."));
#endif

#ifdef HARDWARE_FEEDBACK
        digitalWrite(CONNECTION_PIN, attempt & 1);
#endif
    }

    if (!client.connected()) {
#ifdef SERIAL_FEEDBACK
        Serial.println(F("Couldn't Connect To Server"));
#endif

        return FAILED_TO_CONNECT;
    }

    else {
#ifdef SERIAL_FEEDBACK
        Serial.println(F("Connected To Server."));
#endif

#ifdef HARDWARE_FEEDBACK
        digitalWrite(CONNECTION_PIN, HIGH);
#endif

        return GATEWAY_CONNECTED;
    }
}


void Gateway::run (void) {

    Ethernet.maintain();

    if (!client.connected()) {
#ifdef HARDWARE_FEEDBACK
        digitalWrite(CONNECTION_PIN, LOW);
#endif

#ifdef SERIAL_FEEDBACK
        Serial.println(F("Trying to Reconnect."));
#endif

        //Broadcast exeption code 11: Gateway Target Device Failed to Respond
        uint8_t lostConnectionMSG[] = {0x00, 0x80, 0x0B, 0x51, 0xC7};
        (*serial).write(lostConnectionMSG, 5);


        while (!client.connect(serverIP, serverPort)) {
#ifdef SERIAL_FEEDBACK
            Serial.print(F("."));
#endif

#ifdef HARDWARE_FEEDBACK
            bool state = 0;
            digitalWrite(CONNECTION_PIN, state ^= 1);
#endif
        }

#ifdef HARDWARE_FEEDBACK
        digitalWrite(CONNECTION_PIN, HIGH);
#endif

#ifdef SERIAL_FEEDBACK
        Serial.println(F("Reconnection Successful."));
#endif
    }

    if ((*serial).available())
        this->RTU_to_TCP();

    if (client.available())
        this->TCP_to_RTU();
}


void Gateway::RTU_to_TCP (void) {

#ifdef HARDWARE_FEEDBACK
    digitalWrite(RTU_TO_TCP_PIN, HIGH);
#endif

    //Waits for a silent interval of 3.5 modbus characters on USART
    uint16_t incomingBytes;
    do {
        incomingBytes = (*serial).available();
        delayMicroseconds(t35us);
    } while (incomingBytes != (*serial).available());


    // The vector "modbusData" is used not only to receive the RTU data,
    // but also to compose the TCP packet. The byte usage is:
    //  Byte      Use
    //  0, 1      TCP: Transaction ID
    //  2, 3      TCP: Protocol ID
    //  4, 5      TCP: Length
    //  Others    Modbus: Address/ID - Function - Data
    //  Last 2    RTU: CRC
    uint16_t byteCounter = 6 + incomingBytes;
    uint8_t modbusData[byteCounter];


    //The incoming modbus data is read from the USART
    for (uint16_t k = 6; k < byteCounter; k++)
        modbusData[k] = (*serial).read();


#if defined(DATA_VALIDATION)
    bool isDataValid = false;
    for (uint8_t index = 0; index < 19 && !isDataValid; index++)
        isDataValid = (modbusData[7] == modbusFunctions[index]);

    if (!isDataValid) {
#ifdef SERIAL_FEEDBACK
        Serial.println("Invalid Data");
#endif

        return;
    }
#endif

    //Evaluates the received CRC
    uint16_t incomingCRC = modbusData[byteCounter - 2] | modbusData[byteCounter - 1] << 8;

    //The CRC is calculated with the received data
    uint16_t CRC = CRC16(modbusData, byteCounter - 8, 6);

    //The calculated CRC is compared with the received one
    //In case it checks, the data can be sent
    if (incomingCRC == CRC) {

#ifdef SERIAL_FEEDBACK
        Serial.print(F("R "));
#endif

        //TCP: Transaction ID = 0x0000
        //TCP: Protocol ID = 0x0000
        modbusData[0] = 0x00;
        modbusData[1] = 0x00;
        modbusData[2] = 0x00;
        modbusData[3] = 0x00;

        //TCP: Length
        modbusData[4] = (uint8_t)(byteCounter - 8) >> 8;
        modbusData[5] = (uint8_t)(byteCounter - 8);

        client.write(modbusData, byteCounter - 2);

#ifdef SERIAL_FEEDBACK
        for (int i = 7; i < byteCounter - 2; i++) {
            Serial.print(modbusData[i], HEX);
            Serial.print(" ");
        }
        Serial.println(millis());
#endif
    }

    //If not, an exeption is send back to the master
    else {
        //Unrecoverable error occurred.
        uint8_t exep[5] = {modbusData[6], 0x80 | modbusData[7], 0x04, 0x00, 0x00};
        uint16_t exepCRC = CRC16(exep, 3);
        exep[3] = (uint8_t)(exepCRC >> 8);
        exep[4] = (uint8_t)(exepCRC);

        (*serial).write(exep, 5);

#ifdef SERIAL_FEEDBACK
        Serial.print(F("XR "));

        for (int i = 0; i < 5; i++) {
            Serial.print(exep[i], HEX);
            Serial.print(" ");
        }
        Serial.println(millis());
#endif
    }

#ifdef HARDWARE_FEEDBACK
    digitalWrite(RTU_TO_TCP_PIN, LOW);
#endif
}


void Gateway::TCP_to_RTU (void) {

#ifdef HARDWARE_FEEDBACK
    digitalWrite(TCP_TO_RTU_PIN, HIGH);
#endif

#ifdef SERIAL_FEEDBACK
    Serial.print(F("T "));
#endif

    //uint16_t byteCounter;
    //Read the first 6 bytes incoming from TCP server
    //  Byte      Data
    //  0, 1      TCP: Transaction ID
    //  2, 3      TCP: Protocol ID
    //  4, 5      TCP: Length

    while (client.available() < 6);

    uint8_t tcpID[6];
    for (uint16_t k = 0; k < 6; k++)
        tcpID[k] = client.read();

    //Evaluates the length of the modbus data
    //2 extra bytes are alocated for CRC
    uint16_t modbusLen = tcpID[4] << 8 | tcpID[5];
    uint8_t modbusData[modbusLen + 2];

    //Reads the remaining data
    for (uint16_t k = 0; k < modbusLen; k++)
        modbusData[k] = client.read();

    //Evaluates CRC
    uint16_t CRC = CRC16(modbusData, modbusLen);
    modbusData[modbusLen] = (uint8_t)(CRC);
    modbusData[modbusLen + 1] = (uint8_t)(CRC >> 8);

    //Sends data do USART
    (*serial).write(modbusData, modbusLen + 2);


#ifdef SERIAL_FEEDBACK
    for (int i = 1; i < modbusLen; i++) {
        Serial.print(modbusData[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
#endif

#ifdef HARDWARE_FEEDBACK
    digitalWrite(TCP_TO_RTU_PIN, LOW);
#endif
}


uint16_t Gateway::CRC16(uint8_t* data, uint16_t dSize, uint16_t startByte = 0) {
    //CRC-16/Modbus Generator
    //Processing Time: t[N] ~= 6 + 10*N
    // N : Number of bytes to be processed.
    // t : Time required in microseconds.

    // CRC-16/IBM inverse polynomial representation
    const uint16_t CRCPoly = 0xA001;
    uint16_t CRC = 0xFFFF;

    for (uint16_t nByte = startByte; nByte < (startByte + dSize); nByte++) {
        CRC ^= data[nByte];
        for (uint8_t shift = 0; shift < 8; shift++)
            if (CRC & 0x01)
                CRC = (CRC >> 1) ^ CRCPoly;
            else
                CRC >>= 1;
    }

    return CRC;
}
