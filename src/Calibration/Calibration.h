#ifndef _Calibration_H
#define _Calibration_H

#include <Arduino.h>
#include "Servo_Control/Servo_Control.h"

#define SERVO_CENTER 305

class Servo_Calibration
{
  public:
    void TryCalibrate(Servo_Control *servo_control);

  private:
    void RunCalibration(Servo_Control *servo_control);
    void InitServos(Servo_Control *servo_control);

    const int servoCount = 16;
    int servoPositions[16];
};

void Servo_Calibration::InitServos(Servo_Control *servo_control)
{
    for (int i = 0; i < 16; i++)
    {
        servoPositions[i] = SERVO_CENTER;
    }

    for (int i = 0; i < 16; i++)
    {
        servo_control->setServoPos(i, SERVO_CENTER);
    }
}

void Servo_Calibration::TryCalibrate(Servo_Control *servo_control)
{
    if (Serial.available() > 0)
    {
        char input = Serial.read();
        if (input == 'c')
        {
            Serial.println("Starting calibration");
            Serial.println("x:\t Exit");
            Serial.println("n:\t Next Servo");
            Serial.println("+:\t increase offset");
            Serial.println("-:\t decrease offset");
            InitServos(servo_control);
            RunCalibration(servo_control);
        }
    }
}

void Servo_Calibration::RunCalibration(Servo_Control *servo_control)
{
    bool done = false;
    short servoIndex = 0;
    Serial.print("Calibrating servo:");
    Serial.println(servoIndex);

    while (!done)
    {
        if (Serial.available() > 0)
        {
            char input = Serial.read();
            switch (input)
            {
            case 'x':
                done = true;
                break;
            case '+':
            case '=':
                servoPositions[servoIndex]++;
                Serial.print("Calibrating servo:");
                Serial.println(servoIndex);

                break;
            case '-':
                servoPositions[servoIndex]--;
                Serial.print("Calibrating servo:");
                Serial.println(servoIndex);
                break;
            case 'n':
                if (servoIndex < 15)
                {
                    servoIndex++;
                    Serial.print("Calibrating servo:");
                    Serial.println(servoIndex);
                }
                break;
            case 'p':
                if (servoIndex > 0)
                {
                    servoIndex--;
                    Serial.print("Calibrating servo:");
                    Serial.println(servoIndex);
                }
                break;
            default:
                Serial.print("invalid input");
            }

            servo_control->setServoPos(servoIndex, servoPositions[servoIndex]);
        }
    }

    Serial.println("Servo Offsets\n");
    for (int i = 0; i < 16; i++)
    {
        Serial.print("Servo ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(servoPositions[i]);
    }

    Serial.println("Calibration finished");
}
#endif