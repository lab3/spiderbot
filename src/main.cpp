#include <Arduino.h>
#include <stdarg.h>
#include <PID_v1.h>
#include "Servo_Control/Servo_Control.h"
#include "Calibration/Calibration.h"
#include "MemoryFree.h"

//unsigned long last = micros();
//unsigned long cur = last;

//Define Variables we'll be connecting to
//double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
//double aggKp = 4, aggKi = 0.2, aggKd = 1;
//double consKp = 1, consKi = 0.05, consKd = 0.25;

//Specify the links and initial tuning parameters
//PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

Servo_Control servo = Servo_Control();
Servo_Calibration calibration;

void setup()
{
  Serial.begin(9600);
  Serial.println("Spiderbot starting.");
  delay(2000);
  servo.begin();
  delay(10);
}

void loop()
{
  // Serial.print("freeMemory()=");
  // Serial.println(freeMemory());
  calibration.TryCalibrate(&servo);
  //servo.step(0);

  //servo.sweep();
}
