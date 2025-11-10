#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <Arduino.h>

class MotorControl {
  public:
    MotorControl(int IN1_pin, int IN2_pin);

    void sendPWM(int pwmVal);
    int getDirection();

  private:
    int in1Pin, in2Pin;
    int dir = 1;

    void setForwardDirection();
    void setReverseDirection();
    void setHalt();

};

#endif