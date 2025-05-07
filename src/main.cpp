#include <cstring>
#include <stdio.h>  // For sprintf, snprintf
#include <stdlib.h> // For itoa

#include "RCC/RCCInterface.h"

#include "GPIO/GPIO.h"
#include "GPTimer/GPTimer.h"
#include "USART/USART.h"

#include "UltrasonicSensor/UltrasonicSensor.h"
#include "UltrasonicManager/UltrasonicManager.h"
#include "MotorManager/MotorManager.h"

#include "IRSensor/IRSensor.h"
#include "Bluetooth/Bluetooth.h"
#include "SystemTimer/SystemTimer.h"
#include "MySerial/MySerial.h"
#include "Motor/Motor.h"

#include "Parking.h"

GPIO led_switch(GPIOB, 6);

// Active LOW internal LED
GPIO internal_led(GPIOC, 13);

MotorConfig motorLeftConfig = {
    GPIOA, // PWM GPIO port for left motor forward
    4,     // Direction GPIO pin for forward
    GPIOA, // PWM GPIO port for left motor reverse
    5,     // Direction GPIO pin for reverse

    TIM2, // Timer for PWM generation
    1     // PWM channel (channel 1 for TIM2: A0)

};

MotorConfig motorRightConfig = {
    GPIOB, // PWM GPIO port for right motor forward
    13,    // Direction GPIO pin for forward
    GPIOB, // PWM GPIO port for right motor reverse
    12,    // Direction GPIO pin for reverse

    TIM2, // Timer for PWM generation
    2     // PWM channel (channel 2 for TIM2: A1)
};

int main()
{
    led_switch.init(GPIOMode::Output, GPIOPull::None);
    internal_led.init(GPIOMode::Output, GPIOPull::None);
    led_switch.set();
    internal_led.set();

    // Initialize clock
    if (!RCCInterface::initSystemClock()) // By default 84MHz PLL (HSE based) clock
        internal_led.set();               // Clock Init failed indicator

    // Serial on USART 2
    MySerial &serial = MySerial::getInstance();

    GPTimer::delayMs(TIM4, 1000); // Wait for 1 second (power reaches ultrasonic / others)

    serial.println("Begin");

    // Create left motors
    Motor motorsLeft(motorLeftConfig);

    // Create right motors
    Motor motorsRight(motorRightConfig);

    // Motor manager
    uint8_t motor_countL = 1;
    uint8_t motor_countR = 2;
    MotorManager motorManager(motorsRight, motor_countL, motorsLeft, motor_countR);

    // Ultrasonic sensors
    UltrasonicSensor ultrasonicFront(GPIO(GPIOB, 8), GPIO(GPIOB, 9));      // Trigger on PB8, Echo on PB9
    UltrasonicSensor ultrasonicRight(GPIO(GPIOB, 6), GPIO(GPIOB, 7));      // Trigger on PB6, Echo on PB7
    UltrasonicSensor ultrasonicLeft(GPIO(GPIOB, 4), GPIO(GPIOB, 5));       // Trigger on PB4, Echo on PB5
    UltrasonicSensor ultrasonicFrontRight(GPIO(GPIOA, 8), GPIO(GPIOA, 9)); // Trigger on PA8, Echo on PA9
    // UltrasonicSensor ultrasonicBack(GPIO(GPIOB, 8), GPIO(GPIOB, 9));              // Trigger on PA6, Echo on PA7

    // Ultrasonic manager (front, right, left)
    UltrasonicManager ultrasonicManager(4, 30); // 4 sensors, 30ms settle time
    ultrasonicManager.setSensor(ultrasonicFront);
    ultrasonicManager.setSensor(ultrasonicRight);
    ultrasonicManager.setSensor(ultrasonicLeft);
    ultrasonicManager.setSensor(ultrasonicFrontRight);
    // ultrasonicManager.setSensor(ultrasonicBack);

    // Parking
    Parking parking(motorManager, ultrasonicManager);

    internal_led.reset();

    uint8_t speed = 65;          // Speed for motors
    uint8_t rotation_speed = 80; // Slow speed for parking or rotation

    // motorManager.driveForward(speed);
    // GPTimer::delayMs(TIM4, 1000); // Move forward for 1 second
    // motorManager.stopMotors();    // Stop motors
    // GPTimer::delayMs(TIM4, 500);  // Wait for 500ms

    // motorManager.rotateClockwise(rotation_speed);
    // GPTimer::delayMs(TIM4, 560); // Rotate clockwise for 1 second
    // motorManager.stopMotors();   // Stop motors
    // GPTimer::delayMs(TIM4, 500); // Wait for 500ms

    while (true)
    {
        // serial.println("Main loop");
        // motorManager.rotateClockwise(speed); // Move forward
        // parking.updateParkingLogic(); // Update parking logic
        serial.println("Measure US"); // Debug message
        float frontLeftDist = ultrasonicManager.measureOne(3);
        serial.println("Done"); // Debug message
        char buffer[80]; // Buffer for debug messages
        snprintf(buffer, sizeof(buffer), "Front Left Distance: %.2f cm", frontLeftDist);
        serial.println(buffer);       // Print distances for debugging
        GPTimer::delayMs(TIM4, 1000); // Wait for 1 second

        // GPTimer::delayMs(TIM4, 1000); // Move forward for 1 second
        // motorManager.stopMotors();    // Stop motors
        // GPTimer::delayMs(TIM4, 500);  // Wait for 500ms
        // float distances[3]; // Array to hold distances from sensors
        // serial.println("Measure US"); // Debug message
        // ultrasonicManager.measureAll(distances); // Measure distances
        // serial.println("Done"); // Debug message

        // float frontDist = distances[0], rightDist = distances[1], leftDist = distances[2];

        // char buffer[80]; // Buffer for debug messages
        // sniprintf(buffer, sizeof(buffer), "Distances: Front: %.2f, Right: %.2f, Left: %.2f", frontDist, rightDist, leftDist);

        // MySerial::getInstance().println(buffer); // Print distances for debugging,

        // GPTimer::delayMs(TIM4, 1000); // Wait for 1 second

        // motorManager.driveForward(65);

        // motorManager.driveReverse(speed);
        // GPTimer::delayMs(TIM4, 1000); // Move backward for 1 second
        // motorManager.stopMotors();    // Stop motors
        // GPTimer::delayMs(TIM4, 500);  // Wait for 500ms

        // motorManager.rotateClockwise(75);
        // GPTimer::delayMs(TIM4, 650); // Rotate clockwise for 1 second
        // motorManager.stopMotors();    // Stop motors
        // GPTimer::delayMs(TIM4, 500);  // Wait for 500ms

        // motorManager.rotateCounterClockwise(speed);
        // GPTimer::delayMs(TIM4, 1000); // Rotate counterclockwise for 1 second
        // motorManager.stopMotors();    // Stop motors
        // GPTimer::delayMs(TIM4, 500);  // Wait for 500ms

        // Parking
        // parking.updateParkingLogic();
    }
}
