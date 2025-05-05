#include "UltrasonicManager.h"

#include "GPIO/GPIO.h"
#include "GPTimer/GPTimer.h"

template <uint8_t SensorCount>
UltrasonicManager<SensorCount>::UltrasonicManager(GPTimer &timer, uint32_t settleDelayMs)
    : timer(timer), delayMs(settleDelayMs), sensorIndex(0) {}

template <uint8_t SensorCount>
void UltrasonicManager<SensorCount>::setSensor(uint8_t index, const UltrasonicSensor &sensor)
{
    if (index < SensorCount)
        sensors[index] = sensor;
}

template <uint8_t SensorCount>
void UltrasonicManager<SensorCount>::measureAll(float (&distances)[SensorCount])
{
    for (uint8_t i = 0; i < SensorCount; ++i)
    {
        distances[i] = sensors[i].measureDistanceCm();
        timer.delayMs(sensors[i].getTimer(), delayMs);
    }
}