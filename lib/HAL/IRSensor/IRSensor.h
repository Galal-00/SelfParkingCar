#pragma once

#include "stm32f411xe.h"
#include "GPIO/GPIO.h"

class IRSensor
{
private:
    GPIO IRSensorGPIO;
public:
    IRSensor(GPIO_TypeDef *sensorPort, uint8_t sensorPin);

    // Sensor is active low (LOW when object (black) detected), range is ~2-30 cm
    bool objectDetected();
};

