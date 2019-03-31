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

bool running = false;
bool servos_initialized = false;
bool servos_started = false;
unsigned long button_debounce = millis();
unsigned long button_last = millis();

void button_pressed()
{
  button_debounce = millis();

  if (button_debounce - button_last > 1000)
  {
    Serial.println("button");
    running = !running;
    button_last = millis();
  }
}

void handle_running_change()
{
  if (running)
  {
    if (!servos_initialized)
    {
      servo.init();
      servos_initialized = true;
    }

    if (!servos_started)
    {
      servo.start();
      servos_started = true;
    }
  }
  else
  {
    servos_started = false;
    servo.stop();
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Spiderbot starting.");

  pinMode(PIN2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN2), button_pressed, FALLING);

  delay(2000);
  //servo.begin();
  delay(10);
}

void loop()
{
  handle_running_change();

  if (running)
  {
    calibration.TryCalibrate(&servo);
  }
}


