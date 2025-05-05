#pragma once

#include "stm32f411xe.h"

#include "GPTimer/GPTimer.h"
#include "UltrasonicSensor/UltrasonicSensor.h"

template <uint8_t SensorCount>
class UltrasonicManager
{
private:
    UltrasonicSensor sensors[SensorCount];

    uint32_t settleDelayMs;

    uint8_t sensorIndex = 0;

public:
    UltrasonicManager() : settleDelayMs(60) {}

    UltrasonicManager(uint32_t settleDelayMs) : settleDelayMs(settleDelayMs) {}

    uint8_t setSensor(const UltrasonicSensor &sensor)
    {
        if (sensorIndex < SensorCount)
        {
            sensors[sensorIndex] = sensor;
            return sensorIndex++;
        }

        return 0xFF;
    }

    void measureAll(float (&distances)[SensorCount])
    {
        for (uint8_t i = 0; i < SensorCount; ++i)
        {
            distances[i] = sensors[i].measureDistanceCm();

            GPTimer::delayMs(sensors[i].getTimer(), settleDelayMs);
        }
    }
};