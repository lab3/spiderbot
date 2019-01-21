
#include <stdarg.h>
#include "Servo_Control.h"

//  #define SERVOMIN 205    //150 // this is the 'minimum' pulse length count (out of 4096)
//  #define SERVOMAX 560    //600 // this is the 'maximum' pulse length count (out of 4096)
//  #define SERVOCENTER 382 //375 //((SERVOMAX - SERVOMIN) / 2) + SERVOMIN
//  #define SERVOFREQ 60    //

//Base servo config: TowerPro MG90s
#define MOVEMENT_REDUCTION 30
#define SERVO_MIN 170+MOVEMENT_REDUCTION    //150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_MAX 440-MOVEMENT_REDUCTION    //600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_CENTER 305 //375 //((SERVOMAX - SERVOMIN) / 2) + SERVOMIN
#define SERVO_FREQ 50
#define SERVO_DEGREES 165

//outer, mid, inner, not used
int servo_offsets[] = {
  311, 308, 293, 0, //leg 1, front right
  313, 296, 328, 0, //leg 2, front left
  335, 317, 325, 0, //leg 3, rear left
  327, 309, 322, 0  //leg 4, rear right
};

//Movement tracking
#define MICROS_PER_SECOND 1000000
#define DELTA_PER_MICRO
#define MAX_PULSE_DELTA 10
#define DEGREES_PER_PULSE 0.61111111
//#define DEGREES_PER_MICRO_LIMIT 0.000022
#define DEGREES_PER_MICRO_LIMIT 0.000032

#define LEG_1 0

long elapsedTimeMicros = 0;
long lastTime = -1;

double leg1Pos = SERVO_CENTER;
bool leg1Moving = false;
bool leg1Forward = true;

Servo_Control::Servo_Control()
{
  //Serial.println("Servo_Control::Servo_Control()");
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
  double degreesToMove = elapsedTimeMicros * DEGREES_PER_MICRO_LIMIT;
  double deltaPulseLen = degreesToMove / DEGREES_PER_PULSE;

  if (leg1Forward)
  {

    // Serial.print("degreesToMove: ");
    // Serial.println(degreesToMove);
    // Serial.print("elapsedTimeMicros: ");
    // Serial.println(elapsedTimeMicros);

    if (deltaPulseLen > MAX_PULSE_DELTA)
    {
      deltaPulseLen = MAX_PULSE_DELTA;
    }

    if (deltaPulseLen + leg1Pos > SERVO_MAX)
    {
      deltaPulseLen = SERVO_MAX - leg1Pos;
    }

    if (deltaPulseLen > 0)
    {
      leg1Pos += deltaPulseLen;
      _pwm.setPWM(LEG_1, 0, leg1Pos);
    }

    if (leg1Pos == SERVO_MAX)
    {
      leg1Forward = false;
    }
  }
  else
  {
    if (deltaPulseLen > MAX_PULSE_DELTA)
    {
      deltaPulseLen = MAX_PULSE_DELTA;
    }

    if (leg1Pos - deltaPulseLen < SERVO_MIN)
    {
      deltaPulseLen = leg1Pos - SERVO_MIN;
    }

    if (deltaPulseLen > 0)
    {
      leg1Pos -= deltaPulseLen;
      _pwm.setPWM(LEG_1, 0, leg1Pos);
    }

    if (leg1Pos == SERVO_MIN)
    {
      leg1Forward = true;
    }
  }

  lastTime = currentMicros;
}

void Servo_Control::begin(void)
{
  //Serial.println("Servo_Control::begin()");

  //initialze and center all servos
  _pwm = Adafruit_PWMServoDriver();
  _pwm.begin();
  _pwm.setPWMFreq(SERVO_FREQ);

  for (size_t i = 0; i < 16; i++)
  {
    _pwm.setPWM(i, 0, servo_offsets[i]);
  }

  //enable power to the servo board by engaging the relay
  pinMode(PIND3, OUTPUT);
  digitalWrite(PIND3, HIGH);

  // wired to the OE of the servo board
  // pinMode(PIND2, OUTPUT);
  // digitalWrite(PIND2, LOW);

  //Serial.println("Servo_Control::begin() end");
}

void Servo_Control::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
  _pwm.setPWM(num, on, off);
}

void Servo_Control::setServo(uint8_t num, uint16_t pos)
{
  _pwm.setPWM(num, 0, pos);
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
