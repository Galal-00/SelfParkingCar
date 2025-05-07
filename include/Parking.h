#include <cstring>
#include <stdio.h>  // For sprintf, snprintf
#include <stdlib.h> // For itoa

#include "RCC/RCCInterface.h"

#include "GPIO/GPIO.h"
#include "GPTimer/GPTimer.h"
#include "USART/USART.h"

#include "UltrasonicSensor/UltrasonicSensor.h"
#include "UltrasonicManager/UltrasonicManager.h"
#include "IRSensor/IRSensor.h"
#include "Bluetooth/Bluetooth.h"
#include "SystemTimer/SystemTimer.h"
#include "MySerial/MySerial.h"
#include "Motor/Motor.h"
#include "MotorManager/MotorManager.h"

enum class ParkingState
{
    SearchingForward,
    SearchingBackward,
    // Parking backwards
    ReverseAligningRight,
    ReverseAligningLeft,
    ReverseParking,
    // Parking forwards
    ForwardAligningRight,
    ForwardAligningLeft,
    ForwardParking,
    Done
};

class Parking
{
private:
    MotorManager motorManager;
    UltrasonicManager ultrasonicManager; // Ultrasonic manager for distance measurement

    ParkingState state = ParkingState::SearchingForward; // Current parking state

    const float PARKING_WIDTH_THRESHOLD = 18.0f; // cm
    const float CAR_WIDTH = 15.0f;               // cm
    const float PARKING_DEPTH_THRESHOLD = 25.0f; // cm
    const float SETTLE_DISTANCE = 5.0f;          // cm buffer on each side

    float distances[3]; // Array to hold distances from sensors

    // Define sensor indices for clarity
    const uint8_t FRONT_SENSOR_INDEX = 0;
    const uint8_t RIGHT_SENSOR_INDEX = 1;
    const uint8_t LEFT_SENSOR_INDEX = 2;
    const uint8_t FRONT_RIGHT_SENSOR_INDEX = 3;

public:
    Parking(MotorManager &motorManager, UltrasonicManager &ultrasonicManager);

    void updateParkingLogic();
};

Parking::Parking(MotorManager &motorManager, UltrasonicManager &ultrasonicManager) : motorManager(motorManager), ultrasonicManager(ultrasonicManager) {};

void Parking::updateParkingLogic()
{
    MySerial &serial = MySerial::getInstance(); // Serial instance for debugging
    serial.println("Measure US");               // Debug message
    // ultrasonicManager.measureAll(distances); // Measure distances
    distances[FRONT_SENSOR_INDEX] = ultrasonicManager.measureOne(FRONT_SENSOR_INDEX); // Measure distances
    GPTimer::delayMs(TIM4, 30);                                                     // Wait for a short time to stabilize
    //serial.println("Front US");                                                       // Debug message
    distances[RIGHT_SENSOR_INDEX] = ultrasonicManager.measureOne(RIGHT_SENSOR_INDEX); // Measure distances
    serial.println("Right US");                                                       // Debug message
    GPTimer::delayMs(TIM4, 30);                                                     // Wait for a short time to stabilize
    //distances[LEFT_SENSOR_INDEX] = ultrasonicManager.measureOne(LEFT_SENSOR_INDEX);   // Measure distances
    //serial.println("Left US");                                                        // Debug message

    serial.println("Done"); // Debug message

    float frontDist = distances[FRONT_SENSOR_INDEX], rightDist = distances[RIGHT_SENSOR_INDEX], leftDist = 0;
    float frontRightDist = distances[FRONT_RIGHT_SENSOR_INDEX];

    // char buffer[128]; // Buffer for debug messages
    // sniprintf(buffer, sizeof(buffer), "Distances: Front: %.2f, Right: %.2f, Left: %.2f", frontDist, rightDist, leftDist);

    // MySerial::getInstance().println(buffer); // Print distances for debugging

    uint8_t speed = 70;          // Speed for motors
    uint8_t rotation_speed = 75; // Slow speed for parking or rotation

    switch (state)
    {
    case ParkingState::SearchingForward:
        if (rightDist >= PARKING_DEPTH_THRESHOLD + SETTLE_DISTANCE / 2)
        {
            motorManager.stopMotors();
            state = ParkingState::ForwardAligningRight;
        }
        // else if (leftDist >= PARKING_DEPTH_THRESHOLD + SETTLE_DISTANCE / 2)
        // {
        //     motorManager.stopMotors();
        //     state = ParkingState::ForwardAligningLeft;
        // }
        else
        {
            motorManager.driveForward(speed); // keep scanning
        }
        break;

    case ParkingState::ForwardAligningRight:
        // Align so you're centered and ready to park
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 1000);

        while (1)
        {
            /* code */
        }

        motorManager.rotateClockwise(80); // Rotate slowly to align with the parking space
        GPTimer::delayMs(TIM4, 500);      // Rotate for a period
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 1000); // Wait for a short time to stabilize

        state = ParkingState::ForwardParking;
        break;

    case ParkingState::ForwardAligningLeft:
        // Align so you're centered and ready to park
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 2000);

        // Move slightly forward
        // motorManager.driveForward(slow_speed);
        // GPTimer::delayMs(TIM4, 500); // Move forward for a short time
        // motorManager.stopMotors();
        // GPTimer::delayMs(TIM4, 500); // Wait for a short time to stabilize

        // Recheck left ultrasonic
        leftDist = ultrasonicManager.measureOne(LEFT_SENSOR_INDEX); // Measure distances again
        if (leftDist <= PARKING_DEPTH_THRESHOLD - SETTLE_DISTANCE / 2)
        {
            // Return to the previous state if not enough space
            state = ParkingState::SearchingForward;
        }

        motorManager.rotateCounterClockwise(rotation_speed); // Rotate slowly to align with the parking space
        GPTimer::delayMs(TIM4, 650);                         // Rotate for a period
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 2000); // Wait for a short time to stabilize

        state = ParkingState::ForwardParking;
        break;

    case ParkingState::ForwardParking:
        // Move forward and adjust angle if needed
        motorManager.driveForward(speed);
        GPTimer::delayMs(TIM4, 700); // Move forward for a short time

        motorManager.stopMotors();
        state = ParkingState::Done;
        break;

    case ParkingState::ReverseAligningRight:
        // Align so you're centered and ready to reverse
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500);

        motorManager.rotateCounterClockwise(speed); // Rotate to align with the parking space
        GPTimer::delayMs(TIM4, 500);                // Rotate for a short time
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500); // Wait for a short time to stabilize

        state = ParkingState::ReverseParking;
        break;

    case ParkingState::ReverseAligningLeft:
        // Align so you're centered and ready to reverse
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500);

        motorManager.rotateClockwise(speed); // Rotate to align with the parking space
        GPTimer::delayMs(TIM4, 500);         // Rotate for a short time
        motorManager.stopMotors();

        state = ParkingState::ReverseParking;
        break;

    case ParkingState::ReverseParking:
        // Reverse and adjust angle if needed
        motorManager.driveReverse(speed);
        GPTimer::delayMs(TIM4, 500); // Reverse for a short time

        motorManager.stopMotors();
        state = ParkingState::Done;
        break;

    case ParkingState::Done:
        // Car is parked
        motorManager.stopMotors();                       // Stop motors
        MySerial::getInstance().println("Parking Done"); // Debug message
        break;

    default:
        break;
    }
}