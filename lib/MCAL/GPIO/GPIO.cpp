#include "GPIO.h"
#include "RCC/RCCInterface.h"

GPIO::GPIO() {}

GPIO::GPIO(GPIO_TypeDef *port, uint8_t pin) : _port(port), _pin(pin) {}

void GPIO::setPinPort(GPIO_TypeDef *port, uint8_t pin)
{
    _port = port;
    _pin = pin;
}

void GPIO::getPinPort(GPIO_TypeDef *&portRef, uint8_t &pinRef)
{
    portRef = _port;
    pinRef = _pin;
}

void GPIO::init(GPIOMode mode, GPIOPull pull, GPIOOutputType outputType,
                GPIOSpeed speed, uint8_t altFunc)
{
    // Depending on the register we shift by either pos or __pin
    uint32_t pos = _pin * 2;

    // Enable GPIO clock
    if (!RCCInterface::enableGPIO(_port))
        return;

    // Set mode
    _port->MODER &= ~(0b11 << pos); // clear first
    _port->MODER |= (static_cast<uint32_t>(mode) << pos);

    // Set pull-up/down
    _port->PUPDR &= ~(0b11 << pos); // clear first
    _port->PUPDR |= (static_cast<uint32_t>(pull) << pos);

    // Set output type
    if (mode == GPIOMode::Output || mode == GPIOMode::AlternateFunction)
    {
        _port->OTYPER &= ~(1 << _pin); // clear first
        _port->OTYPER |= (static_cast<uint32_t>(outputType) << _pin);

        // Set speed
        _port->OSPEEDR &= ~(0b11 << pos); // clear first
        _port->OSPEEDR |= (static_cast<uint32_t>(speed) << pos);
    }

    // Set alternate function
    if (mode == GPIOMode::AlternateFunction)
    {
        uint32_t afr_index = _pin / 8;                          // 8 pins in AFRL, AFRH
        uint32_t afr_pos = (_pin % 8) * 4;                      // Each pin has 4 bits of control
        _port->AFR[afr_index] &= ~(0xF << afr_pos);             // clear first
        _port->AFR[afr_index] |= ((altFunc & 0x0F) << afr_pos); // Only altFunc lower 4 bits matter
    }
}

void GPIO::set()
{
    _port->BSRR = (1 << _pin);
}

void GPIO::reset()
{
    _port->BSRR = (1 << (_pin + 16));
}

void GPIO::toggle()
{
    _port->ODR ^= (1 << _pin);
}

bool GPIO::read()
{
    return (_port->IDR & (1 << _pin));
}