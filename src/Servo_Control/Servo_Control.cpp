
#include <stdarg.h>
#include "Servo_Control.h"

//  #define SERVOMIN 205    //150 // this is the 'minimum' pulse length count (out of 4096)
//  #define SERVOMAX 560    //600 // this is the 'maximum' pulse length count (out of 4096)
//  #define SERVOCENTER 382 //375 //((SERVOMAX - SERVOMIN) / 2) + SERVOMIN
//  #define SERVOFREQ 60    //

//Base servo config: TowerPro MG90s
#define SERVO_MIN 170    //150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_MAX 440    //600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_CENTER 305 //375 //((SERVOMAX - SERVOMIN) / 2) + SERVOMIN
#define SERVO_FREQ 50
#define SERVO_DEGREES 165

//Movement tracking
#define MICROS_PER_SECOND 1000000
#define DELTA_PER_MICRO
#define MAX_PULSE_DELTA 10
#define DEGREES_PER_PULSE 0.61111111
#define DEGREES_PER_MICRO_LIMIT 0.000022

#define LEG_1 0

short degreesOfTravel = 165;
short degreesPerSecondCapable = 600;
short degreesPerSecondLimit = 22;
long elapsedTimeMicros = 0;
long lastTime = -1;

short leg1Pos = SERVO_CENTER;
bool leg1Moving = false;
bool leg1Forward = true;

Servo_Control::Servo_Control()
{
  Serial.println("Servo_Control::Servo_Control()");
}

void Servo_Control::step(uint8_t leg)
{
  unsigned long currentMicros = micros();

  if (!leg1Moving && leg1Pos != SERVO_MAX)
  {
    //offset so we actually get some movement on this pass
    lastTime = micros() - 1000;
    leg1Moving = true;
    leg1Forward = true;
  }

  //calculate the amount to move forward
  elapsedTimeMicros = currentMicros - lastTime;
  if (leg1Forward)
  {
    double degreesToMove = elapsedTimeMicros * DEGREES_PER_MICRO_LIMIT;

    Serial.print("degreesToMove: ");
    Serial.println(degreesToMove);

    Serial.print("elapsedTimeMicros: ");
    Serial.println(elapsedTimeMicros);

    double delta = degreesToMove / DEGREES_PER_PULSE;

    if (delta > MAX_PULSE_DELTA)
    {
      delta = MAX_PULSE_DELTA;
    }

    if (delta + leg1Pos > SERVO_MAX)
    {
      delta = SERVO_MAX - leg1Pos;
    }

    if (delta)
    {
      leg1Pos += delta;
      _pwm.setPWM(LEG_1, 0, leg1Pos);
    }

    if (leg1Pos == SERVO_MAX)
    {
      leg1Moving = false;
      leg1Forward = false;
    }
  }

  lastTime = currentMicros;
}

void Servo_Control::begin(void)
{
  Serial.println("Servo_Control::begin()");

  //initialze and center all servos
  _pwm = Adafruit_PWMServoDriver();
  _pwm.begin();
  _pwm.setPWMFreq(SERVO_FREQ);

  for (size_t i = 0; i < 12; i++)
  {
    _pwm.setPWM(i, 0, SERVO_CENTER);
  }

  //enable power to the servo board by engaging the relay
  pinMode(PIND3, OUTPUT);
  digitalWrite(PIND3, HIGH);

  // wired to the OE of the servo board
  // pinMode(PIND2, OUTPUT);
  // digitalWrite(PIND2, LOW);

  Serial.println("Servo_Control::begin() end");
}

void Servo_Control::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
  _pwm.setPWM(num, on, off);
}

void Servo_Control::sweep(void)
{

  //Drive each servo one at a time
  delay(1000);
  Serial.println("min");

  for (uint16_t pulselen = SERVO_MIN; pulselen < SERVO_MAX; pulselen++)
  {
    _pwm.setPWM(0, 0, pulselen);

    if (pulselen == SERVO_CENTER)
    {
      delay(1000);
    }

    delay(1);
  }

  Serial.println("max");
  delay(1000);

  for (uint16_t pulselen = SERVO_MAX; pulselen > SERVO_MIN; pulselen--)
  {
    _pwm.setPWM(0, 0, pulselen);

    if (pulselen == SERVO_CENTER)
    {
      delay(1000);
    }

    delay(1);
  }
}
