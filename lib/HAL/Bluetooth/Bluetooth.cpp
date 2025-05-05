#include "Bluetooth.h"

#include <cctype>
#include <cstring>

#include "stm32f411xe.h"
#include "USART/USART.h"

Bluetooth::Bluetooth(USART::BaudRate baudRate) : usart(USART1, baudRate) {}

void Bluetooth::changeBaudRate(USART::BaudRate baudRate)
{
    usart.changeBaudRate(baudRate);
}

void Bluetooth::sendCMD(COMMAND cmd, char *response)
{
    switch (cmd)
    {
    case COMMAND::AT:
        usart.transmitString("AT\r\n");
        usart.receiveString(response, 4, CMD_TERMINATOR_1, CMD_TERMINATOR_2);
        break;
    case COMMAND::AT_NAME:
        usart.transmitString("AT+NAME?\r\n");
        usart.receiveString(response, MAX_SIZE, CMD_TERMINATOR_1, CMD_TERMINATOR_2);
        break;
    case COMMAND::AT_VERSION:
        usart.transmitString("AT+VERSION?\r\n");
        usart.receiveString(response, MAX_SIZE, CMD_TERMINATOR_1, CMD_TERMINATOR_2);
        break;
    default:
        break;
    }
}

void Bluetooth::sendCMD(COMMAND cmd, const char *cmdString, char *response)
{
    if (cmd == COMMAND::AT_NAME_newname)
    {
        char concat_cmd[32] = "AT+NAME=";
        strcat(concat_cmd, cmdString);
        strcat(concat_cmd, "\r\n");

        usart.transmitString(concat_cmd);
        usart.receiveString(response, MAX_SIZE, CMD_TERMINATOR_1, CMD_TERMINATOR_2);
    }
}

bool Bluetooth::waitStartSignal()
{
    char response[MAX_SIZE] = {0};
    uint32_t cmd_length = strlen(START_CMD);

    bool received = false;
    while (!received)
    {
        usart.receiveString(response, MAX_SIZE, "\r\n", "\n\r");

        for (uint32_t i = 0; i < strlen(response); ++i)
            response[i] = tolower(response[i]);

        received = strncmp(response, START_CMD, cmd_length) == 0;
    }

    return true;
}