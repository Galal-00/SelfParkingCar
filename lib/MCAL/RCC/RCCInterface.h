#pragma once

#include <cstdint>
#include "stm32f411xe.h"

class RCCInterface
{
private:
    static bool validatePLL(uint8_t PLLM, uint16_t PLLN, uint8_t PLLP);

public:
    // Default values for 84MHz clock speed
    static bool initSystemClock(uint8_t PLLM = 25, uint16_t PLLN = 336, uint8_t PLLP = 4);
    static bool enableGPIO(GPIO_TypeDef *gpio);
    static bool enableTimer(TIM_TypeDef *timer);
    static bool enableUSART(USART_TypeDef *USARTx);
};
