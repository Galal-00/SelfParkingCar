#include "USART.h"

#include "stm32f411xe.h"
#include <cstring>

#include "RCC/RCCInterface.h"
#include "GPIO/GPIO.h"

USART::USART(USART_TypeDef *USARTx, BaudRate baudRate) : USARTx(USARTx), baudRate(baudRate)
{
    RCCInterface::enableUSART(USARTx);

    USARTx->CR1 &= ~USART_CR1_UE;              // Disable USART during setup
    USARTx->CR1 = USART_CR1_TE | USART_CR1_RE; // Enable Transmit and Receive

    configureUSART();

    USARTx->CR1 |= USART_CR1_UE; // Enable USART
}

void USART::changeBaudRate(BaudRate baudRate)
{
    this->baudRate = baudRate;

    USARTx->CR1 &= ~USART_CR1_UE;              // Disable USART during setup
    USARTx->CR1 = USART_CR1_TE | USART_CR1_RE; // Enable Transmit and Receive

    if (USARTx == USART2)
    {
        USARTx->BRR = computeBRR(SystemCoreClock / 2);
    }
    else if (USARTx == USART1 || USARTx == USART6)
    {
        USARTx->BRR = computeBRR(SystemCoreClock);
    }

    USARTx->CR1 |= USART_CR1_UE; // Enable USART
}

void USART::transmitChar(char c)
{
    // Wait until previous char is sent
    while (!(USARTx->SR & USART_SR_TXE))
        ;
    USARTx->DR = c;
}

void USART::transmitString(const char *str)
{
    while (*str)
        transmitChar(*str++);
}

char USART::receiveChar()
{
    // Wait until char is received
    while (!(USARTx->SR & USART_SR_RXNE))
        ;

    return USARTx->DR;
}

bool USART::receiveString(char *buffer, uint32_t size, const char *terminator1, const char *terminator2)
{
    if (size == 0)
        return false;

    size_t i = 0, t1_size = strlen(terminator1), t2_size = strlen(terminator2);

    do
    {
        buffer[i++] = receiveChar();
        if ((i >= t1_size && strncmp(&buffer[i - t1_size], terminator1, t1_size) == 0) ||
            (i >= t2_size && strncmp(&buffer[i - t2_size], terminator2, t2_size) == 0))
        {
            // Terminate C string with null character
            buffer[i] = '\0';
            // received string correctly
            return true;
        }
    } while (i < size); // Stop when buffer size is reached or terminating char received

    // Terminate C string with null character
    buffer[i] = 0;

    // buffer overflowed; return
    return false;
}

void USART::configureUSART()
{
    if (USARTx == USART2)
    {
        // PA2 -> TX (AF7), PA3 -> RX (AF7)
        GPIO txPin(GPIOA, 2);
        GPIO rxPin(GPIOA, 3);

        txPin.init(GPIOMode::AlternateFunction, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::VeryHigh, 7);
        rxPin.init(GPIOMode::AlternateFunction, GPIOPull::PullDown, GPIOOutputType::PushPull, GPIOSpeed::VeryHigh, 7);

        USARTx->BRR = computeBRR(SystemCoreClock / 2);
    }
    else if (USARTx == USART1)
    {
        // PA15 -> TX (AF7), PB3 -> RX (AF7)
        GPIO txPin(GPIOA, 15);
        GPIO rxPin(GPIOB, 3);

        txPin.init(GPIOMode::AlternateFunction, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::VeryHigh, 7);
        rxPin.init(GPIOMode::AlternateFunction, GPIOPull::PullDown, GPIOOutputType::PushPull, GPIOSpeed::VeryHigh, 7);

        USARTx->BRR = computeBRR(SystemCoreClock);
    }
    else if (USARTx == USART6)
    {
        // PA11 -> TX (AF8), PA12 -> RX (AF8)
        GPIO txPin(GPIOA, 11);
        GPIO rxPin(GPIOA, 12);

        txPin.init(GPIOMode::AlternateFunction, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::VeryHigh, 8);
        rxPin.init(GPIOMode::AlternateFunction, GPIOPull::PullDown, GPIOOutputType::PushPull, GPIOSpeed::VeryHigh, 8);

        USARTx->BRR = computeBRR(SystemCoreClock);
    }
}

uint16_t USART::computeBRR(uint32_t pclk)
{
    float usartdiv = static_cast<float>(pclk) / (16.0f * static_cast<float>(baudRate));

    uint32_t mantissa = static_cast<uint32_t>(usartdiv);
    uint32_t fraction = static_cast<uint32_t>((usartdiv - mantissa) * 16.0f + 0.5f); // +0.5 for rounding

    if (fraction >= 16)
    {
        mantissa += 1;
        fraction = 0;
    }

    return (mantissa << 4) | (fraction & 0xF);
}
