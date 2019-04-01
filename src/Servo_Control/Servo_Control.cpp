
#include <stdarg.h>
#include "Servo_Control.h"

//  #define SERVOMIN 205    //150 // this is the 'minimum' pulse length count (out of 4096)
//  #define SERVOMAX 560    //600 // this is the 'maximum' pulse length count (out of 4096)
//  #define SERVOCENTER 382 //375 //((SERVOMAX - SERVOMIN) / 2) + SERVOMIN
//  #define SERVOFREQ 60    //

//Base servo config: TowerPro MG90s
#define MOVEMENT_REDUCTION 30
#define SERVO_MIN 170 + MOVEMENT_REDUCTION //150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_MAX 440 - MOVEMENT_REDUCTION //600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_CENTER 305                   //375 //((SERVOMAX - SERVOMIN) / 2) + SERVOMIN
#define SERVO_FREQ 50
#define SERVO_DEGREES 165
#define MAX_ITEMS 16

//These indexes map to pins on the
//outer, mid, inner, not used
//fill this array up with the values resulting from the calibration
int servo_centers[] = {
    311, 308, 293, 0, //leg 1, front right
    313, 296, 328, 0, //leg 2, front left
    335, 317, 325, 0, //leg 3, rear left
    327, 309, 322, 0  //leg 4, rear right
};

double current_positions[16];
int target_positions[16];

//Movement tracking
#define MICROS_PER_SECOND 1000000
#define MAX_PULSE_DELTA 5
#define DEGREES_PER_PULSE 0.61111111
#define DEGREES_PER_MICRO_LIMIT 0.000032
//#define DEGREES_PER_MICRO_LIMIT 0.000564
#define LEG_1 0

long elapsedMicros = 0;
long lastMicros = -1;

double leg1Pos = SERVO_CENTER;
bool leg1Moving = false;
bool leg1Forward = true;

Servo_Control::Servo_Control()
{
  for (int i = 0; i < MAX_ITEMS; i++)
  {
    target_positions[i] = current_positions[i] = servo_centers[i];
  }
}

void Servo_Control::init(void)
{
  //initialze and center all servos
  _pwm = Adafruit_PWMServoDriver();
  _pwm.begin();
  _pwm.setPWMFreq(SERVO_FREQ);

  for (size_t i = 0; i < 16; i++)
  {
    _pwm.setPWM(i, 0, servo_centers[i]);
  }

  //enable power to the servo board by engaging the relay
  pinMode(PIND3, OUTPUT);

  // wired to the OE of the servo board
  // pinMode(PIND2, OUTPUT);
  // digitalWrite(PIND2, LOW);
}

void Servo_Control::stop(void)
{
  digitalWrite(PIND3, LOW);
}

void Servo_Control::start(void)
{
  digitalWrite(PIND3, HIGH);
}

bool Servo_Control::moveToTargets()
{
  unsigned long currentMicros = micros();
  elapsedMicros = currentMicros - lastMicros;
  int mod = 1;
  bool moveComplete = true;

  for (int i = 0; i < MAX_ITEMS; i++)
  {
    if (current_positions[i] != target_positions[i])
    {
      moveComplete = false;
      // Serial.print(elapsedMicros);
      // Serial.print("\t");
      // Serial.print(i);
      // Serial.print("\t");
      // Serial.println(current_positions[i]);

      double deltaPulses = 0;
      if (current_positions[i] < target_positions[i])
      {
        deltaPulses = target_positions[i] - current_positions[i];
        mod = 1;
      }
      else if (current_positions[i] > target_positions[i])
      {
        deltaPulses = current_positions[i] - target_positions[i];
        mod = -1;
      }

      double degreesToMove = DEGREES_PER_PULSE * deltaPulses;
      double maxDegreestoMove = DEGREES_PER_MICRO_LIMIT * elapsedMicros;

      if (degreesToMove > maxDegreestoMove)
      {
        degreesToMove = maxDegreestoMove;
      }

      current_positions[i] += degreesToMove * mod;
      setServoPos(i, current_positions[i]);
    }
  }

  lastMicros = currentMicros;
  return moveComplete;
}

//This should not really be used for servos
void Servo_Control::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
  _pwm.setPWM(num, on, off);
}

void Servo_Control::setServoPos(uint8_t num, uint16_t pos)
{
  _pwm.setPWM(num, 0, pos);
}

void Servo_Control::setServoTargetPos(uint8_t num, uint16_t pos)
{
  if (num >= 0 && num < MAX_ITEMS && pos >= SERVO_MIN && pos <= SERVO_MAX)
  {
    target_positions[num] = pos;
  }
}
