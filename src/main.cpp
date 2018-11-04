#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 220    //150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 560    //600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOCENTER 382 //375 //((SERVOMAX - SERVOMIN) / 2) + SERVOMIN

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);


//pca8685 OE pin - low all outputs enabled, board pulls it low by default


void setup()
{
  Serial.begin(9600);
  Serial.println("Spiderbot testing");

  //D2 wired to pca8685 OE
  pinMode(PIND2, OUTPUT);

  //disable
  digitalWrite(PIND2, HIGH);

  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  delay(10);
  delay(5000);
  Serial.println("Start");
  digitalWrite(PIND2, LOW);
  pwm.setPWM(0, 0, SERVOMIN);
  delay(10);
}

void loop()
{
    //pwm.setPWM(0, 0, SERVOMIN);
    //delay(100);
  //Drive each servo one at a time

  Serial.println("min");
  delay(1000);

  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
  {
    pwm.setPWM(0, 0, pulselen);

    if (pulselen == SERVOCENTER)
    {
      delay(1000);
    }

    delay(1);
  }

  Serial.println("max");
  delay(1000);

  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
  {
    pwm.setPWM(0, 0, pulselen);

    if (pulselen == SERVOCENTER)
    {
      delay(1000);
    }

    delay(1);
  }
}
