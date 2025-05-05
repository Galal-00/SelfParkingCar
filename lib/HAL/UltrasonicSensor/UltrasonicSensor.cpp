#include "UltrasonicSensor.h"

#include "GPIO/GPIO.h"
#include "GPTimer/GPTimer.h"

UltrasonicSensor::UltrasonicSensor(GPIO trig, GPIO echo, TIM_TypeDef *TIMx)
    : trig(trig), echo(echo), TIMx(TIMx)
{
    trig.init(GPIOMode::Output);
    echo.init(GPIOMode::Input, GPIOPull::None); // MUST be left floating
}

void UltrasonicSensor::sendTriggerPulse()
{
    trig.reset();
    GPTimer::delayUs(TIMx, 2); // stabilize
    trig.set();
    GPTimer::delayUs(TIMx, 10); // 10 µs pulse
    trig.reset();
}

uint32_t UltrasonicSensor::measureEchoTimeUs()
{
    GPTimer::initBasic(TIMx, 84 - 1, 0xFFFF); // 1 MHz = 1 µs per tick
    TIMx->CNT = 0;

    // Wait for echo to go HIGH
    while (!echo.read())
        ;

    TIMx->CNT = 0; // reset counter
    while (echo.read())
        ; // wait until echo goes LOW again

    uint32_t duration = TIMx->CNT; // Avoid race condition

    TIMx->CR1 &= ~TIM_CR1_CEN;

    return duration;
}

float UltrasonicSensor::measureDistanceCm()
{
    sendTriggerPulse();
    uint32_t time_us = measureEchoTimeUs();

    // Sound speed = 343 m/s = 0.0343 cm/us -> Round trip -> divide by 2
    return time_us * 0.0343f / 2.0f;
}