#include "UltrasonicSensor.h"

#include "GPIO/GPIO.h"
#include "GPTimer/GPTimer.h"

UltrasonicSensor::UltrasonicSensor() {}

UltrasonicSensor::UltrasonicSensor(GPIO trig, GPIO echo, TIM_TypeDef *TIMx)
    : trig(trig), echo(echo), TIMx(TIMx)
{
    setConfig(trig, echo, TIMx);
}

void UltrasonicSensor::setConfig(GPIO trig, GPIO echo, TIM_TypeDef *TIMx)
{
    uint8_t trig_af = 0, echo_af = 0;
    GPIO_TypeDef *trig_port = GPIOA, *echo_port = GPIOA;
    uint8_t trig_pin = 0, echo_pin = 0;

    trig.getPinPort(trig_port, trig_pin);
    echo.getPinPort(echo_port, echo_pin);

    // Check if AFIO needs update (Handles only PB4 for now)
    if (trig_port == GPIOB && trig_pin == 4)
    {
        trig_af = 3;
    }
    else if (echo_port == GPIOB && echo_pin == 4)
    {
        echo_af = 3;
    }

    trig.init(GPIOMode::Output, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::Medium, trig_af);
    echo.init(GPIOMode::Input, GPIOPull::None, GPIOOutputType::PushPull, GPIOSpeed::Medium, echo_af); // MUST be left floating

    // Update (if needed) the stored parameters
    this->trig = trig;
    this->echo = echo;
    this->TIMx = TIMx;
}

TIM_TypeDef *UltrasonicSensor::getTimer()
{
    return TIMx;
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