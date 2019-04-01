#include <Arduino.h>
#include <stdarg.h>
#include <PID_v1.h>
#include "Servo_Control/Servo_Control.h"
#include "Calibration/Calibration.h"
#include "MemoryFree.h"

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
    Serial.println("button_pressed");
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

      //TODO: remove that after testing
      servo.setServoTargetPos(0 + 0, 400);
      servo.setServoTargetPos(0 + 4, 400);
      servo.setServoTargetPos(0 + 8, 400);
      servo.setServoTargetPos(0 + 12, 400);
      servo.setServoTargetPos(1 + 0, 400);
      servo.setServoTargetPos(1 + 4, 400);
      servo.setServoTargetPos(1 + 8, 400);
      servo.setServoTargetPos(1 + 12, 400);
      servo.setServoTargetPos(2 + 0, 400);
      servo.setServoTargetPos(2 + 4, 400);
      servo.setServoTargetPos(2 + 8, 400);
      servo.setServoTargetPos(2 + 12, 400);

    }
  }
  else
  {
    servos_started = false;
    servo.stop();
  }
}

void nextMove()
{
  servo.setServoTargetPos(0 + 0, 200);
  servo.setServoTargetPos(0 + 4, 200);
  servo.setServoTargetPos(0 + 8, 200);
  servo.setServoTargetPos(0 + 12, 200);
  servo.setServoTargetPos(1 + 0, 200);
  servo.setServoTargetPos(1 + 4, 200);
  servo.setServoTargetPos(1 + 8, 200);
  servo.setServoTargetPos(1 + 12, 200);
  servo.setServoTargetPos(2 + 0, 200);
  servo.setServoTargetPos(2 + 4, 200);
  servo.setServoTargetPos(2 + 8, 200);
  servo.setServoTargetPos(2 + 12, 200);
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Spiderbot starting.");

  pinMode(PIN2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN2), button_pressed, FALLING);

  //This is now triggered by button push
  //delay(2000);
  //servo.begin();
  //delay(10);
}

void loop()
{
  handle_running_change();

  if (running)
  {
    if (servo.moveToTargets())
    {
      nextMove();
    }
    calibration.TryCalibrate(&servo);
  }
}
