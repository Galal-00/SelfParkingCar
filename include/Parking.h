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
    // MySerial &serial = MySerial::getInstance(); // Serial instance for debugging
    //  serial.println("Measure US");               // Debug message
    //  ultrasonicManager.measureAll(distances); // Measure distances
    //  serial.println("Done"); // Debug message

    // char buffer[128]; // Buffer for debug messages
    // snprintf(buffer, sizeof(buffer), "Distances: Front: %.2f, Right: %.2f, Left: %.2f", frontDist, rightDist, leftDist);

    // MySerial::getInstance().println(buffer); // Print distances for debugging

    float frontDist = ultrasonicManager.measureOne(FRONT_SENSOR_INDEX); // Measure distances again

    if (frontDist <= 7.0f)
    {
        motorManager.driveReverse(100); // Move backward
        GPTimer::delayMs(TIM4, 200);    // Move backward for 1 second
        motorManager.stopMotors();
        while (1)
        {
        } // Stop everything
        return;
    }
}

void Parking::rotateIntoPosition(bool isClockWise)
{
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
    MySerial &serial = MySerial::getInstance(); // Serial instance for debugging
    // serial.println("Measure US");               // Debug message
    // ultrasonicManager.measureAll(distances); // Measure distances
    // distances[RIGHT_SENSOR_INDEX] = 0; // Measure distances
    // distances[LEFT_SENSOR_INDEX] = ultrasonicManager.measureOne(LEFT_SENSOR_INDEX);

    distances[LEFT_SENSOR_INDEX] = 0;
    distances[RIGHT_SENSOR_INDEX] = ultrasonicManager.measureOne(RIGHT_SENSOR_INDEX);

    // distances[FRONT_RIGHT_SENSOR_INDEX] = 0; // Measure distances
    //  distances[FRONT_LEFT_SENSOR_INDEX] = ultrasonicManager.measureOne(FRONT_LEFT_SENSOR_INDEX); // Measure distances
    //  serial.println("Front Right US");                                                             // Debug message
    distances[FRONT_LEFT_SENSOR_INDEX] = 0;
    distances[FRONT_RIGHT_SENSOR_INDEX] = ultrasonicManager.measureOne(FRONT_RIGHT_SENSOR_INDEX);
    // serial.println("Front Left US");                                                            // Debug message

    // serial.println("Done"); // Debug message

    float rightDist = distances[RIGHT_SENSOR_INDEX], leftDist = distances[LEFT_SENSOR_INDEX];
    float frontRightDist = distances[FRONT_RIGHT_SENSOR_INDEX];
    float frontLeftDist = distances[FRONT_LEFT_SENSOR_INDEX];

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

            // if (ultrasonicManager.measureOne(LEFT_SENSOR_INDEX) <= 4 || ultrasonicManager.measureOne(FRONT_LEFT_SENSOR_INDEX) <= 2)

            // {

            //     motorManager.rotateClockwise(70);
            //     GPTimer::delayMs(TIM4, 300);
            //     motorManager.stopMotors();
            //     GPTimer::delayMs(TIM4, 300);

            //     motorManager.driveForward(speed); // keep scanning
            //     GPTimer::delayMs(TIM4, 500);
            //     motorManager.stopMotors();
            //     GPTimer::delayMs(TIM4, 300);
            //     motorManager.rotateCounterClockwise(70);
            //     GPTimer::delayMs(TIM4, 200);
            // }

            motorManager.driveForward(speed); // keep scanning
        }

        break;

    case ParkingState::ForwardAligningRight:
        // Align so you're centered and ready to park
        checkForwardObstable(); // Check for obstacles in front

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

        // while (1)
        //{
        /* code */
        //}

        motorManager.driveForward(speed);
        GPTimer::delayMs(TIM4, 400);
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500);

        motorManager.rotateClockwise(100); // Rotate slowly to align with the parking space
        GPTimer::delayMs(TIM4, 440);       // Rotate for a period
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 1000); // Wait for a short time to stabilize

        rotateIntoPosition(true); // Rotate slowly to align with the parking space

        state = ParkingState::ForwardParking;
        break;

    case ParkingState::ForwardAligningLeft:
        // Align so you're centered and ready to park
        checkForwardObstable(); // Check for obstacles in front

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

        // while (1)
        //{
        /* code */
        //}

        motorManager.driveForward(speed);
        GPTimer::delayMs(TIM4, 200);
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500);

        motorManager.rotateCounterClockwise(70); // Rotate slowly to align with the parking space
        GPTimer::delayMs(TIM4, 700);             // Rotate for a period
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 1000); // Wait for a short time to stabilize

        rotateIntoPosition(false); // Rotate slowly to align with the parking space

        state = ParkingState::ForwardParking;
        break;

    case ParkingState::ForwardParking:
    {
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
        motorManager.stopMotors();                       // Stop motors
        MySerial::getInstance().println("Parking Done"); // Debug message
        break;

    default:
        break;
    }
}