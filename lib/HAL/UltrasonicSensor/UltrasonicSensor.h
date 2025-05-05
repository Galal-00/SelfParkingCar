#pragma once

#include "stm32f411xe.h"
#include "GPIO/GPIO.h"

class UltrasonicSensor
{
public:
    /* Default constructor for arrays of Ultrasonic sensors */
    UltrasonicSensor();

    /* By default, use TIM3 */
    UltrasonicSensor(GPIO trig, GPIO echo, TIM_TypeDef *TIMx = TIM3);

    /* Set (or change) US config */
    void setConfig(GPIO trig, GPIO echo, TIM_TypeDef *TIMx = TIM3);

    // returns distance in cm
    float measureDistanceCm();

    // returns used timer
    TIM_TypeDef *getTimer();

private:
    GPIO trig;
    GPIO echo;
    TIM_TypeDef *TIMx;

    void sendTriggerPulse();
    uint32_t measureEchoTimeUs();
};