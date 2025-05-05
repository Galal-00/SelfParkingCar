#include "Motor.h"

#include "GPIO/GPIO.h"
#include "GPTimer/GPTimer.h"

Motor::Motor(const MotorConfig &config) : fwdPin(config.dirGPIOForward, config.dirPinForward),
                                          revPin(config.dirGPIOReverse, config.dirPinReverse),
                                          pwmTimer(config.pwmTimer), pwmChannel(config.pwmChannel)
{
    // Setup Forward and backward pins
    fwdPin.init(GPIOMode::Output);
    revPin.init(GPIOMode::Output);

    // Setup PWM
    GPTimer::initPwm(pwmTimer, pwmChannel);
}

void Motor::setDirection(Direction dir)
{
    if (dir == Direction::Forward)
    {
        // Set forward direction and ensure reverse is off
        fwdPin.set();
        revPin.reset();
    }
    else
    {
        // Set reverse direction and ensure forward is off
        fwdPin.reset();
        revPin.set();
    }
}

void Motor::stopMotor()
{
    setSpeed(0);
    fwdPin.reset();
    revPin.reset();
}

void Motor::setSpeed(float speed)
{
    if (speed < 0 || speed > 100)
        return; // Keep speed as is

    GPTimer::setPwmDuty(pwmTimer, pwmChannel, speed);
}