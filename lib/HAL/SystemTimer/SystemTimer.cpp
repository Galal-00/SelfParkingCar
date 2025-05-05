#include "SystemTimer.h"

#include "stm32f411xe.h"

#include "GPTimer/GPTimer.h"

void SystemTimer::init()
{
    GPTimer::initBasic(TIM5, 84 - 1, 0xFFFFFFFF);
}

uint32_t SystemTimer::millis()
{
    return GPTimer::getCounterValue(TIM5) / 1000;
}

uint32_t SystemTimer::micros()
{
    return GPTimer::getCounterValue(TIM5);
}
