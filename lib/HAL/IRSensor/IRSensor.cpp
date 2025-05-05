#include "IRSensor.h"
#include "GPIO/GPIO.h"

IRSensor::IRSensor(GPIO_TypeDef *sensorPort, uint8_t sensorPin) : IRSensorGPIO(sensorPort, sensorPin)
{
    IRSensorGPIO.init(GPIOMode::Input, GPIOPull::None);
}

bool IRSensor::objectDetected()
{
    return !IRSensorGPIO.read();
}
