#include "motor_control.h"

MotorControl::MotorControl(int IN1_pin, int IN2_pin)
{
  in1Pin = IN1_pin;
  in2Pin = IN2_pin;

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  analogWrite(in1Pin, 0);
  analogWrite(in2Pin, 0);
}

void MotorControl::sendPWM(int pwmVal)
{
  if (pwmVal > 0)
  {
    analogWrite(in1Pin, abs(pwmVal));
    analogWrite(in2Pin, 0);
    setForwardDirection();
  }
  else if (pwmVal < 0)
  {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, abs(pwmVal));
    setReverseDirection();
  }
  else
  {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, 0);
    setHalt();
  }
}

int MotorControl::getDirection()
{
  return dir;
}

void MotorControl::setForwardDirection()
{
  dir = 1;
  // digitalWrite(in1Pin, HIGH);
  // digitalWrite(in2Pin, LOW);
}

void MotorControl::setReverseDirection()
{
  dir = 0;
  // digitalWrite(in1Pin, LOW);
  // digitalWrite(in2Pin, HIGH);
}

void MotorControl::setHalt()
{
  dir = 0;
  // digitalWrite(in1Pin, LOW);
  // digitalWrite(in2Pin, LOW);
}
