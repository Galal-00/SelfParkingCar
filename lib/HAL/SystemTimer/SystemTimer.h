#pragma once

#include "stm32f411xe.h"

class SystemTimer
{
public:
    /* It uses TIMER 5 By default */
    static void init();
    static uint32_t millis();
    static uint32_t micros();
};
