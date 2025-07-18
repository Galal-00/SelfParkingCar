#include "MotorManager.h"

MotorManager::MotorManager(Motor &motorsRight, uint8_t motor_countR, Motor &motorsLeft, uint8_t motor_countL) : motorsRight(motorsRight), motor_countR(motor_countR),
                                                                                                                motorsLeft(motorsLeft), motor_countL(motor_countL)
{
}

void MotorManager::stopMotors()
{
    motorsLeft.stopMotor();

    motorsRight.stopMotor();
}

void MotorManager::driveForward(uint8_t speed)
{
    motorsLeft.setDirection(Motor::Direction::Forward);
    motorsLeft.setSpeed(speed);

    motorsRight.setDirection(Motor::Direction::Forward);
    motorsRight.setSpeed(speed * 0.9);
}

void MotorManager::driveReverse(uint8_t speed)
{
    motorsLeft.setDirection(Motor::Direction::Reverse);
    motorsLeft.setSpeed(speed);

    motorsRight.setDirection(Motor::Direction::Reverse);
    motorsRight.setSpeed(speed * 0.9);
}

void MotorManager::rotateClockwise(uint8_t speed)
{
    motorsLeft.setDirection(Motor::Direction::Forward);
    motorsLeft.setSpeed(speed);

    motorsRight.setDirection(Motor::Direction::Reverse);
    motorsRight.setSpeed(speed);
}

void MotorManager::rotateCounterClockwise(uint8_t speed)
{
    motorsRight.setDirection(Motor::Direction::Forward);
    motorsRight.setSpeed(speed);

    motorsLeft.setDirection(Motor::Direction::Reverse);
    motorsLeft.setSpeed(speed);
}