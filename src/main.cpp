#include <Arduino.h>
#include <stdarg.h>
#include <PID_v1.h>
#include "Servo_Control/Servo_Control.h"

#define MICROS_PER_SECOND 1000000

int16_t ax, ay, az;
int16_t gx, gy, gz;

short degreesOfTravel = 165;
short degreesPerSecondMax = 600;
short degreesPerSecondLimit = 22;
short elapsedTimeMicros = 0;
short lastTime = -1;

unsigned long last = micros();
unsigned long cur = last;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{
  Serial.begin(9600);
  Serial.println("Spiderbot starting");

  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  // cur = micros();
  // Serial.println(cur - last);
  // delay(1000);
  // last = cur;

  myPID.Compute();

  Serial.println(Output);
  delay(500);
}

void p(char *fmt, ...)
{
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, 128, fmt, args);
  va_end(args);
  Serial.print(buf);
}
