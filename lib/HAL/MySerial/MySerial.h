#include "USART/USART.h"

class MySerial
{
private:
    USART CustomSerial;

    /* Private constructors (singleton pattern) */

    // Uses USART 2: ports A2 (TX), A3 (RX). Default baudrate = 9600
    MySerial();
    // Uses USART 2: ports A2 (TX), A3 (RX).
    MySerial(USART::BaudRate BaudRate);

public:
    // Delete copy constructor and assignment operator
    MySerial(const MySerial &) = delete;
    MySerial &operator=(const MySerial &) = delete;

    // Singleton instance. Uses USART 2: ports A2 (TX), A3 (RX).
    static MySerial &getInstance(USART::BaudRate baudRate = USART::BaudRate::BR_9600);

    void print(const char *str);
    void println(const char *str);

    char readChar();
    void readString(char *buffer, uint16_t size, const char *terminator1 = "\r", const char *terminator2 = "\n");
};
