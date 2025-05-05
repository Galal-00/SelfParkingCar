#pragma once

#include "stm32f411xe.h"

enum class GPIOMode
{
    Input,
    Output,
    AlternateFunction,
    Analog
};

enum class GPIOOutputType
{
    PushPull,
    OpenDrain
};

enum class GPIOPull
{
    None,
    PullUp,
    PullDown
};

enum class GPIOSpeed
{
    Low,
    Medium,
    High,
    VeryHigh
};

class GPIO
{
private:
    GPIO_TypeDef *_port;
    uint8_t _pin;

public:
    GPIO();

    GPIO(GPIO_TypeDef *port, uint8_t pin);

    void setPinPort(GPIO_TypeDef *port, uint8_t pin);

    void getPinPort(GPIO_TypeDef *&portRef, uint8_t &pinRef);
    
    void init(GPIOMode mode, GPIOPull pull = GPIOPull::None,
              GPIOOutputType outputType = GPIOOutputType::PushPull,
              GPIOSpeed speed = GPIOSpeed::Medium, uint8_t altFunc = 0);
    void set();
    void reset();
    void toggle();
    bool read();
};