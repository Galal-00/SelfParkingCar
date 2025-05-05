#pragma once

#include "stm32f411xe.h"

class USART
{

public:
    enum class BaudRate : uint32_t
    {
        BR_9600 = 9600,
        BR_19200 = 19200,
        BR_38400 = 38400,
        BR_57600 = 57600,
        BR_115200 = 115200
    };

    struct USARTConfig
    {
        USART_TypeDef *USARTx;

        GPIO_TypeDef *port;
        uint8_t txPin;
        uint8_t rxPin;

        BaudRate baudRate;
    };

    USART(USART_TypeDef *USARTx = USART2, BaudRate BaudRate = BaudRate::BR_9600);

    // Change baudrate
    void changeBaudRate(BaudRate baudRate);

    void transmitChar(char c);
    void transmitString(const char *str);

    char receiveChar();
    bool receiveString(char *buffer, uint32_t size, const char *terminator1 = "\0", const char *terminator2 = "\n");

private:
    USART_TypeDef *USARTx;
    BaudRate baudRate;

    void configureUSART();
    uint16_t computeBRR(uint32_t clk);
};
