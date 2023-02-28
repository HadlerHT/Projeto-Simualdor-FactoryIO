#include <Gateway.h>

IPAddress ip(192, 168, 3, 121);
IPAddress subnet(255, 255, 252, 0);

Gateway mbGateway(&ip, &subnet);
// Gateway mbGateway(&ip);
// Gateway mbGateway;

void setup() {

    //  MODBUS TCP/IP CONFIGURATION
    //  - Server (modbus slave) IP Address
    //  - UDP Port (Default = 502)
    //  - Timeout[ms] (Default = 500ms)
    //  - Attempts to Connect (Default = KEEP_TRYING)
    mbGateway.configTCP(IPAddress(192, 168, 3, 120));

    //  MODBUS RTU CONFIGURATION
    //  - Hardware Serial (USART) to be used
    //  - Baudrate [bps]
    //  - Serial Parameters (Default = SERIAL_8N1)
    mbGateway.configRTU(&Serial1, 38400);

}

void loop() {
  
    mbGateway.run();   

}