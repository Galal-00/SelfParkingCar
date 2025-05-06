#pragma once

#include "Motor/Motor.h"

#include "stm32f411xe.h"

class MotorManager
{
private:
    Motor motorsRight;
    uint8_t motor_countR;

    Motor motorsLeft;
    uint8_t motor_countL;

public:
    MotorManager(Motor &motorsRight, uint8_t motor_countR, Motor &motorsLeft, uint8_t motor_countL);

    // Stop all motors, direction is forward and speed is 0
    void stopMotors();

    // Currently speed is hardcoded to 100L, 75R
    void driveForward(uint8_t speed);

    // Currently speed is hardcoded to 100L, 75R
    void driveReverse(uint8_t speed);

    // Currently speed is hardcoded to 100L, 75R
    void rotateClockwise(uint8_t speed);

    // Currently speed is hardcoded to 100L, 75R
    void rotateCounterClockwise(uint8_t speed);
};
