#pragma once

#include "stm32f411xe.h"
#include "GPIO/GPIO.h"

class UltrasonicSensor
{
public:
    /* By default, use TIM3 */
    UltrasonicSensor(GPIO trig, GPIO echo, TIM_TypeDef *TIMx = TIM3);

    // returns distance in cm
    float measureDistanceCm();

private:
    GPIO trig;
    GPIO echo;
    TIM_TypeDef *TIMx;

    void sendTriggerPulse();
    uint32_t measureEchoTimeUs();
};