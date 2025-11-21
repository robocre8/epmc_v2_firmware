#include "l298n_motor_control.h"

MotorControl::MotorControl(int IN1_pin, int IN2_pin, int en_pin)
{
  in1Pin = IN1_pin;
  in2Pin = IN2_pin;
  enPin = en_pin;

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enPin, OUTPUT);

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
}

void MotorControl::sendPWM(int pwmVal)
{
  if (pwmVal > 0)
  {
    analogWrite(enPin, abs(pwmVal));
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
  else if (pwmVal < 0)
  {
    analogWrite(enPin, abs(pwmVal));
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  else
  {
    analogWrite(enPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}