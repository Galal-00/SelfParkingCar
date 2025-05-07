#pragma once

#include "stm32f411xe.h"

#include "GPTimer/GPTimer.h"
#include "UltrasonicSensor/UltrasonicSensor.h"

class UltrasonicManager
{
private:
    UltrasonicSensor *sensors;

    uint32_t settleDelayMs;

    const uint8_t sensorCount;

    uint8_t lastSensorIndex = 0;

public:
    UltrasonicManager(uint8_t sensorCount, uint32_t settleDelayMs = 60) : settleDelayMs(settleDelayMs), sensorCount(sensorCount)
    {
        sensors = new UltrasonicSensor[sensorCount];
    }

    uint8_t setSensor(const UltrasonicSensor &sensor)
    {
        if (lastSensorIndex < sensorCount)
        {
            sensors[lastSensorIndex] = sensor;
            return lastSensorIndex++;
        }

        return 0xFF;
    }

    float measureOne(uint8_t idx)
    {
        if (idx < sensorCount)
        {
            float distance = sensors[idx].measureDistanceCm();
            return distance;
        }

        return 0.0f;
    }

    void measureAll(float *distances)
    {
        for (uint8_t i = 0; i < sensorCount; ++i)
        {
            distances[i] = sensors[i].measureDistanceCm();

            GPTimer::delayMs(sensors[i].getTimer(), settleDelayMs);
        }
    }
};