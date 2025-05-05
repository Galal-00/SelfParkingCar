#include "MySerial.h"

#include "USART/USART.h"

/* Private constructors defn*/

MySerial::MySerial() : CustomSerial(USART2, USART::BaudRate::BR_9600) {}

MySerial::MySerial(USART::BaudRate BaudRate) : CustomSerial(USART2, BaudRate) {}

// Singleton accessor
MySerial &MySerial::getInstance(USART::BaudRate baudRate)
{
    static MySerial instance(baudRate); // Initialized only once
    return instance;
}

void MySerial::print(const char *str)
{
    CustomSerial.transmitString(str);
}

void MySerial::println(const char *str)
{
    CustomSerial.transmitString(str);
    CustomSerial.transmitChar('\n');
}

char MySerial::readChar()
{
    return CustomSerial.receiveChar();
}

void MySerial::readString(char *buffer, uint16_t size, const char *terminator1, const char *terminator2)
{
    CustomSerial.receiveString(buffer, size, terminator1, terminator2);
}