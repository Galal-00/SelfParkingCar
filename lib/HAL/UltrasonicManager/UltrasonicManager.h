#pragma once

#include "stm32f411xe.h"

#include "GPTimer/GPTimer.h"
#include "UltrasonicSensor/UltrasonicSensor.h"

template <uint8_t SensorCount>
class UltrasonicManager
{
private:
    UltrasonicSensor sensors[SensorCount];
    GPTimer& timer;

    uint32_t delayMs;
    uint8_t sensorIndex;

public:
    UltrasonicManager(GPTimer &timer, uint32_t settleDelayMs = 60);

    void setSensor(uint8_t index, const UltrasonicSensor &sensor);

    void measureAll(float (&distances)[SensorCount]);
};