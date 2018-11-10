
#include "Servo_Control.h"

//  #define SERVOMIN 205    //150 // this is the 'minimum' pulse length count (out of 4096)
//  #define SERVOMAX 560    //600 // this is the 'maximum' pulse length count (out of 4096)
//  #define SERVOCENTER 382 //375 //((SERVOMAX - SERVOMIN) / 2) + SERVOMIN
//  #define SERVOFREQ 60    //

//Base servo config: TowerPro MG90s
#define SERVOMIN 170    //150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 440    //600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOCENTER 305 //375 //((SERVOMAX - SERVOMIN) / 2) + SERVOMIN
#define SERVOFREQ 50    //

//Movement tracking
#define MICROS_PER_SECOND 1000000
#define DELTA_PER_MICRO
#define MAX_DELTA

short degreesOfTravel = 165;
short degreesPerSecondCapable = 600;
short degreesPerSecondLimit = 22;
short elapsedTimeMicros = 0;
short lastTime = -1;

short leg1Pos = SERVOCENTER;
bool leg1Moving = false;
bool leg1Forward = true;

Servo_Control::Servo_Control()
{
  Serial.println("Servo_Control::Servo_Control()");
}

void Servo_Control::step(uint8_t leg)
{
  unsigned long currentMicros = micros();

  if (!leg1Moving)
  {
    //offset so we actually get some movement on this pass
    lastTime = micros() - 1000;
    leg1Moving = true;
    leg1Forward = true;
  }

  //calculate the amount to move forward
  if (leg1Forward)
  {
  }

  elapsedTimeMicros = currentMicros - lastTime;

  lastTime = currentMicros;
}

void Servo_Control::begin(void)
{
  Serial.println("Servo_Control::begin()");

  //initialze and center all servos
  _pwm = Adafruit_PWMServoDriver();
  _pwm.begin();
  _pwm.setPWMFreq(SERVOFREQ);

  for (size_t i = 0; i < 12; i++)
  {
    _pwm.setPWM(i, 0, SERVOCENTER);
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

  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
  {
    _pwm.setPWM(0, 0, pulselen);

    if (pulselen == SERVOCENTER)
    {
      delay(1000);
    }

    delay(1000);
  }

  Serial.println("max");
  delay(1000);

  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
  {
    _pwm.setPWM(0, 0, pulselen);

    if (pulselen == SERVOCENTER)
    {
      delay(1000);
    }

    delay(1);
  }
}