#pragma once

#include "stm32f411xe.h"
#include "GPIO/GPIO.h"
#include "GPTimer/GPTimer.h"

struct MotorConfig
{
    GPIO_TypeDef *dirGPIOForward; // GPIO port for direction control
    uint8_t dirPinForward; // GPIO pin for forward direction
    GPIO_TypeDef *dirGPIOReverse; // GPIO port for direction control
    uint8_t dirPinReverse; // GPIO pin for reverse direction

    TIM_TypeDef *pwmTimer; // Timer for PWM signal
    uint8_t pwmChannel;    // PWM channel (e.g., 1 or 2 for TIM2)
};

class Motor
{
private:
    GPIO fwdPin; // GPIO pin for forward direction
    GPIO revPin; // GPIO pin for reverse direction

    TIM_TypeDef *pwmTimer; // Timer for PWM signal
    uint8_t pwmChannel;    // PWM channel (e.g., 1 or 2 for TIM2)

public:
    enum class Direction
    {
        Forward = 0,
        Reverse = 1
    };

    Motor(const MotorConfig &config);
    // Method to set the motor direction
    void setDirection(Direction dir);
    // Stop Motor
    void stopMotor();
    // Method to set the motor speed (0 to 100 percent)
    void setSpeed(float speed);
};