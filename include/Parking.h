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

    float PARKING_WIDTH_THRESHOLD = 18.0f; // cm
    float CAR_WIDTH = 15.0f;               // cm
    float SETTLE_DISTANCE = 5.0f;          // cm buffer on each side

    float distances[3]; // Array to hold distances from sensors

public:
    Parking(MotorManager &motorManager, UltrasonicManager &ultrasonicManager);

    void updateParkingLogic();
};

Parking::Parking(MotorManager &motorManager, UltrasonicManager &ultrasonicManager) : motorManager(motorManager), ultrasonicManager(ultrasonicManager) {};

void Parking::updateParkingLogic()
{
    ultrasonicManager.measureAll(distances); // Measure distances

    float frontDist = distances[0], rightDist = distances[1], leftDist = distances[2];

    char buffer[80]; // Buffer for debug messages
    sniprintf(buffer, sizeof(buffer), "Distances: Front: %.2f, Right: %.2f, Left: %.2f", frontDist, rightDist, leftDist);

    MySerial::getInstance().println(buffer); // Print distances for debugging

    switch (state)
    {
    case ParkingState::SearchingForward:
        if (rightDist >= PARKING_WIDTH_THRESHOLD + SETTLE_DISTANCE * 2)
        {
            motorManager.stopMotors();
            state = ParkingState::ReverseAligningRight;
        }
        else if (leftDist >= PARKING_WIDTH_THRESHOLD + SETTLE_DISTANCE * 2)
        {
            motorManager.stopMotors();
            state = ParkingState::ReverseAligningLeft;
        }
        else if (frontDist < PARKING_WIDTH_THRESHOLD + SETTLE_DISTANCE * 2)
        {
            motorManager.stopMotors();
            state = ParkingState::SearchingBackward; // Move to the next state
        }
        else
        {
            motorManager.driveForward(100); // keep scanning
        }
        break;

    case ParkingState::ReverseAligningRight:
        // Align so you're centered and ready to reverse
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500);

        motorManager.rotateCounterClockwise(100); // Rotate to align with the parking space
        GPTimer::delayMs(TIM4, 500);              // Rotate for a short time
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500); // Wait for a short time to stabilize

        state = ParkingState::ReverseParking;
        break;

    case ParkingState::ReverseAligningLeft:
        // Align so you're centered and ready to reverse
        motorManager.stopMotors();
        GPTimer::delayMs(TIM4, 500);

        motorManager.rotateClockwise(100); // Rotate to align with the parking space
        GPTimer::delayMs(TIM4, 500);       // Rotate for a short time
        motorManager.stopMotors();

        state = ParkingState::ReverseParking;
        break;

    case ParkingState::ReverseParking:
        // Reverse and adjust angle if needed
        motorManager.driveReverse(100);
        GPTimer::delayMs(TIM4, 500); // Reverse for a short time

        motorManager.stopMotors();
        state = ParkingState::Done;
        break;

    case ParkingState::Done:
        // Car is parked
        break;

    default:
        break;
    }
}