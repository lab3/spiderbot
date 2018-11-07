#include "Servo_Control.h"

#define SERVOMIN 205    //150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 560    //600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOCENTER 382 //375 //((SERVOMAX - SERVOMIN) / 2) + SERVOMIN
#define SERVOFREQ 50    //60

Servo_Control::Servo_Control()
{
    _pwm = Adafruit_PWMServoDriver();
    _pwm.setPWMFreq(SERVOFREQ);
    pinMode(PIN_A3, OUTPUT);
    digitalWrite(PIN_A3, HIGH);
}

void Servo_Control::begin(void)
{
    _pwm.setPWM(0, 0, SERVOCENTER);
    // delay(100);
    // Drive each servo one at a time

    // Serial.println("min");
    // delay(1000);

    // for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
    // {
    //   pwm.setPWM(0, 0, pulselen);

    //   if (pulselen == SERVOCENTER)
    //   {
    //     delay(1000);
    //   }

    //   delay(1);
    // }

    // Serial.println("max");
    // delay(1000);

    // for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
    // {
    //   pwm.setPWM(0, 0, pulselen);

    //   if (pulselen == SERVOCENTER)
    //   {
    //     delay(1000);
    //   }

    //   delay(1);
    // }
}