#pragma once

#include "stm32f411xe.h"

class GPTimer
{
private:
    static bool enableGpioPwmAf(TIM_TypeDef *TIMx, uint8_t channel);

public:
    // Initialize timer in counting mode
    static void initBasic(TIM_TypeDef *TIMx, uint16_t prescaler, uint32_t arr);
    // Maximum delay is 6553 ms
    static void delayMs(TIM_TypeDef *TIMx, uint32_t ms);
    // Maximum delay is 65535 us
    static void delayUs(TIM_TypeDef *TIMx, uint32_t us);
    // Return counter register value for this timer
    static uint32_t getCounterValue(TIM_TypeDef *TIMx);
    // @ 84MHz: This ensures 20KHz PWM w/ 1% resoultion
    static bool initPwm(TIM_TypeDef *TIMx, uint8_t channel, uint16_t prescaler = 41, uint32_t arr = 99);
    // Set PWM DC given duty cycle in percentage
    static void setPwmDuty(TIM_TypeDef *TIMx, uint8_t channel, float dutyPercentage);
};