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

    float distances[5]; // Array to hold distances from sensors

    // Define sensor indices for clarity
    const uint8_t FRONT_SENSOR_INDEX = 0;
    const uint8_t RIGHT_SENSOR_INDEX = 1;
    const uint8_t LEFT_SENSOR_INDEX = 2;
    const uint8_t FRONT_RIGHT_SENSOR_INDEX = 3;
    const uint8_t FRONT_LEFT_SENSOR_INDEX = 4;

    const float initialLeftDist = 9;

    // Move forward and adjust angle if needed
    uint32_t parkStart = 0;
    uint32_t parkEnd = 0;

    void checkForwardObstable();

    void rotateIntoPosition(bool isClockWise);

public:
    Parking(MotorManager &motorManager, UltrasonicManager &ultrasonicManager);

    void updateParkingLogic();
};

Parking::Parking(MotorManager &motorManager, UltrasonicManager &ultrasonicManager) : motorManager(motorManager), ultrasonicManager(ultrasonicManager) {};

void Parking::checkForwardObstable()
{
    /* Check for obstacles, if exist reverse & stop*/

    float frontDist = ultrasonicManager.measureOne(FRONT_SENSOR_INDEX); // Measure distances again

    if (frontDist <= 7.0f)
    {
        motorManager.driveReverse(100);
        GPTimer::delayMs(TIM4, 200);
        motorManager.stopMotors();
        while (1)
        {
        } // Stop everything
        return;
    }
}

void Parking::rotateIntoPosition(bool isClockWise)
{
    /* Incremental rotations to align with parking space */
    float frontDist = ultrasonicManager.measureOne(FRONT_SENSOR_INDEX); // Measure distances again

    while (frontDist < 15.0f)
    {
        if (isClockWise)
        {
            motorManager.rotateClockwise(70); // Rotate clockwise
            GPTimer::delayMs(TIM4, 500);      // Rotate for 1 second
        }
        else
        {
            motorManager.rotateCounterClockwise(70); // Rotate counterclockwise
            GPTimer::delayMs(TIM4, 500);             // Rotate for 1 second
        }
        motorManager.stopMotors();   // Stop motors
        GPTimer::delayMs(TIM4, 500); // Wait for 500ms
    }
}

void Parking::updateParkingLogic()
{
    /* For parking left */
    distances[RIGHT_SENSOR_INDEX] = 0;
    distances[LEFT_SENSOR_INDEX] = ultrasonicManager.measureOne(LEFT_SENSOR_INDEX);
    distances[FRONT_RIGHT_SENSOR_INDEX] = 0;
    distances[FRONT_LEFT_SENSOR_INDEX] = ultrasonicManager.measureOne(FRONT_LEFT_SENSOR_INDEX);

    /* For parking right */
    distances[LEFT_SENSOR_INDEX] = 0;
    distances[RIGHT_SENSOR_INDEX] = ultrasonicManager.measureOne(RIGHT_SENSOR_INDEX);
    distances[FRONT_LEFT_SENSOR_INDEX] = 0;
    distances[FRONT_RIGHT_SENSOR_INDEX] = ultrasonicManager.measureOne(FRONT_RIGHT_SENSOR_INDEX);

    // US distances
    float rightDist = distances[RIGHT_SENSOR_INDEX], leftDist = distances[LEFT_SENSOR_INDEX];
    float frontRightDist = distances[FRONT_RIGHT_SENSOR_INDEX];
    float frontLeftDist = distances[FRONT_LEFT_SENSOR_INDEX];

    uint8_t speed = 70; // Common speed for motors

    switch (state)
    {
    case ParkingState::SearchingForward:

        if (rightDist >= PARKING_DEPTH_THRESHOLD + SETTLE_DISTANCE / 2)
        {
            motorManager.stopMotors();
            GPTimer::delayMs(TIM4, 1000);
            state = ParkingState::ForwardAligningRight;
        }
        else if (leftDist >= PARKING_DEPTH_THRESHOLD + SETTLE_DISTANCE / 2)
        {
            motorManager.stopMotors();
            GPTimer::delayMs(TIM4, 1000);
            state = ParkingState::ForwardAligningLeft;
        }
        else
        {
            checkForwardObstable(); // Check for obstacles in front

            /* for parking left, handle fwd left motor drift */

            if (ultrasonicManager.measureOne(LEFT_SENSOR_INDEX) <= 4 || ultrasonicManager.measureOne(FRONT_LEFT_SENSOR_INDEX) <= 2)

            {

                motorManager.rotateClockwise(70);
                GPTimer::delayMs(TIM4, 300);
                motorManager.stopMotors();
                GPTimer::delayMs(TIM4, 300);

                motorManager.driveForward(speed); // keep scanning
                GPTimer::delayMs(TIM4, 500);
                motorManager.stopMotors();
                GPTimer::delayMs(TIM4, 300);
                motorManager.rotateCounterClockwise(70);
                GPTimer::delayMs(TIM4, 200);
            }

            motorManager.driveForward(speed); // keep scanning
        }

        break;

    case ParkingState::ForwardAligningRight:
        // Align so you're centered and ready to park
        checkForwardObstable(); // Check for obstacles in front

        // Check if space is wide enough to park
        motorManager.driveReverse(speed);
        GPTimer::delayMs(TIM4, 225);
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500);

        frontRightDist = ultrasonicManager.measureOne(FRONT_RIGHT_SENSOR_INDEX); // Measure distances again

        if (frontRightDist < 13.0f)
        {
            motorManager.driveForward(speed);
            GPTimer::delayMs(TIM4, 450); // Move forward for a short time
            state = ParkingState::SearchingForward;
            break;
        }

        // Space is wide enough, so continue parking
        motorManager.driveForward(speed);
        GPTimer::delayMs(TIM4, 400);
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500);

        motorManager.rotateClockwise(100); // Rotate slowly to align with the parking space
        GPTimer::delayMs(TIM4, 440);
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 1000); // Wait for a short time to stabilize

        // Incremental rotation to align with the parking space
        rotateIntoPosition(true);

        state = ParkingState::ForwardParking;
        break;

    case ParkingState::ForwardAligningLeft:
        // Align so you're centered and ready to park
        checkForwardObstable(); // Check for obstacles in front

        // Check if space is wide enough to park
        motorManager.driveReverse(speed);
        GPTimer::delayMs(TIM4, 300);
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500);

        frontLeftDist = ultrasonicManager.measureOne(FRONT_LEFT_SENSOR_INDEX); // Measure distances again

        if (frontLeftDist < 13.0f)
        {
            motorManager.driveForward(speed);
            GPTimer::delayMs(TIM4, 450); // Move forward for a short time
            state = ParkingState::SearchingForward;
            break;
        }

        // Space is wide enough, so continue parking
        motorManager.driveForward(speed);
        GPTimer::delayMs(TIM4, 200);
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500);

        motorManager.rotateCounterClockwise(70); // Rotate slowly to align with the parking space
        GPTimer::delayMs(TIM4, 700);
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 1000);

        // Incremental rotation to align with the parking space
        rotateIntoPosition(false);

        state = ParkingState::ForwardParking;
        break;

    case ParkingState::ForwardParking:
    {
        // Park but mind sudden obstacles in front
        parkStart = SystemTimer::millis(); // Get the current time in milliseconds
        parkEnd = SystemTimer::millis();   // Initialize end time
        while (parkEnd - parkStart < 1900)
        {
            motorManager.driveForward(speed);
            checkForwardObstable();          // Check for obstacles in front
            parkEnd = SystemTimer::millis(); // Update end time
        }

        motorManager.stopMotors();
        state = ParkingState::Done;
        break;
    }

    case ParkingState::Done:
        // Car is parked
        motorManager.stopMotors(); // Stop motors
        break;

    default:
        break;
    }
}