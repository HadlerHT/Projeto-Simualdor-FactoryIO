#include <Gateway.h>

Gateway mbGateway;

void setup() {

    //  MODBUS TCP/IP CONFIGURATION
    //  - Server (modbus slave) IP Address
    mbGateway.configTCP(IPAddress(192, 168, 0, 6));

    //  MODBUS RTU CONFIGURATION
    //  - Hardware Serial (USART) to be used
    //  - Baudrate [bps]
    mbGateway.configRTU(&Serial, 38400);

}

void loop() {

    mbGateway.run();

}