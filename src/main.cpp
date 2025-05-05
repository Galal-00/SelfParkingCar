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

GPIO led_switch(GPIOA, 7);

// Active LOW internal LED
GPIO internal_led(GPIOC, 13);

MotorConfig motor1Config = {
    GPIOB, // PWM GPIO port for motor 1 forward
    0,     // Direction GPIO pin for forward
    GPIOB, // PWM GPIO port for motor 1 reverse
    1,     // Direction GPIO pin for reverse

    TIM2, // Timer for PWM generation
    1     // PWM channel (channel 1 for TIM2)
};

int main()
{
    led_switch.init(GPIOMode::Input, GPIOPull::PullUp);
    internal_led.init(GPIOMode::Output, GPIOPull::None);

    internal_led.reset();

    // Initialize clock
    if (!RCCInterface::initSystemClock()) // By default 84MHz PLL (HSE based) clock
        internal_led.set();               // Clock Init failed indicator

    // Create motor
    // Motor motor1(motor1Config);

    // Serial on USART 2
    MySerial &serial = MySerial::getInstance();

    // Bluetooth on USART 1
    // Bluetooth BT(USART::BaudRate::BR_38400);
    // BT.changeBaudRate(USART::BaudRate::BR_9600);

    // Delay until platformio serial monitor opens
    GPTimer::delayMs(TIM4, 6000);

    serial.println("Begin");

    internal_led.set(); // Delay ended indicator

    // char response[255] = {0};
    // serial.println("Attempt");
    // BT.sendCMD(Bluetooth::COMMAND::AT, response);
    // serial.print(response);
    // serial.println("pass\n");

    // serial.println("Attempt 2");
    // BT.sendCMD(Bluetooth::COMMAND::AT_NAME_newname, "galal_HC-05", response);
    // serial.print(response);
    // serial.println("pass 2");

    // serial.println("Attempt 3");
    // BT.sendCMD(Bluetooth::COMMAND::AT_NAME, response);
    // serial.print(response);
    // serial.println("pass 3 \n\n\n");

    // bool start = false;
    // serial.println("Attempt receive start");
    // if (BT.waitStartSignal())
    // {
    //     start = true;
    //     serial.println("Received start!");
    // }

    /* Create IR */
    // IRSensor IR1(GPIOA, 4);

    internal_led.reset();

    /* Create Ultrasonic Sensors */
    UltrasonicSensor US[] = {UltrasonicSensor(GPIO(GPIOA, 12), GPIO(GPIOA, 11)),
                             UltrasonicSensor(GPIO(GPIOA, 10), GPIO(GPIOA, 9))};

    // UltrasonicSensor US1(GPIO(GPIOA, 12), GPIO(GPIOA, 11));
    // UltrasonicSensor US2(GPIO(GPIOA, 10), GPIO(GPIOA, 9));
    // UltrasonicSensor US3(GPIO(GPIOA, 8), GPIO(GPIOB, 15));
    const uint8_t count_US = 2;
    UltrasonicManager<count_US> USM;

    for (auto &&us : US)
    {
        USM.setSensor(us);
    }

    char buff[32];
    float distances[count_US];
    int j = 0;
    while (true)
    {
        // float dist = US[0].measureDistanceCm();
        // snprintf(buff, sizeof(buff), "%.3f", dist);

        USM.measureAll(distances);

        for (int i = 0; i < count_US; i++)
        {
            snprintf(buff, sizeof(buff), "Distance %d: %.3f", i, distances[i]);
            serial.println(buff);
        }

        // Settle delay for US is at minimum is 60 ms

        if (++j >= 1100)
        {
            break;
        }
        GPTimer::delayMs(TIM4, 100);
        // if (IR1.objectDetected())
        // {
        //     serial.println("Detected");
        // }
        // else
        // {
        //     serial.println("No object");
        // }

        // GPTimer::delayMs(TIM4, 500);
    }
}
