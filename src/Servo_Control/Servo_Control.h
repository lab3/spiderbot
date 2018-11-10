#ifndef _ServoControl_H
#define _ServoControl_H

#include <Adafruit_PWMServoDriver.h>

class Servo_Control
{
  public:
    Servo_Control();
    void begin(void);
    void sweep(void);
    void setPWM(uint8_t num, uint16_t on, uint16_t off);
    void step(uint8_t leg);

  private:
    Adafruit_PWMServoDriver _pwm;
    // called this way, it uses the default address 0x40
    // you can also call it with a different address you want
    //Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
    // you can also call it with a different address and I2C interface
    //Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);
};

#endif