#pragma once

#include "stm32f411xe.h"

#include "GPIO/GPIO.h"
#include "USART/USART.h"

class Bluetooth
{
private:
    USART usart;

    // Max msg size
    const uint8_t MAX_SIZE = 255;

    // AT mode terminators
    const char *CMD_TERMINATOR_1 = "OK\r\n";
    const char *CMD_TERMINATOR_2 = "FAIL\r\n";

    // Start command
    const char *START_CMD = "start\r\n";

    // Read incoming data
    void read(char *response, uint16_t size);

public:
    enum class COMMAND
    {
        AT,
        AT_NAME,
        AT_VERSION,
        AT_NAME_newname,
        AT_PIN,
        AT_BAUDx
    };

    // HC-05. Uses USART 1: ports A15 (TX), B3 (RX).
    Bluetooth(USART::BaudRate baudRate);

    // Change baudrate
    void changeBaudRate(USART::BaudRate baudRate);

    // Simple inquiry commands
    void sendCMD(COMMAND cmd, char *response);

    // To change module name / password
    void sendCMD(COMMAND cmd, const char *cmdString, char *response);

    // Wait for start signal
    bool waitStartSignal();
};
