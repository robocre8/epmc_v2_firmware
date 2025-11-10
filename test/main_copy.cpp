#include <Arduino.h>

// MOTOR 1
#define IN1 0
#define IN2 1

// MOTOT 2
#define IN3 3
#define IN4 4


void setup() {
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);

}

void loop() {
  analogWrite(IN1, 100);
  analogWrite(IN2, 0);

  analogWrite(IN3, 100);
  analogWrite(IN4, 0);

  delay(5000);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  analogWrite(IN3, 0);
  analogWrite(IN4, 0);

  delay(2000);

  analogWrite(IN1, 0);
  analogWrite(IN2, 100);

  analogWrite(IN3, 0);
  analogWrite(IN4, 100);

  delay(5000);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  analogWrite(IN3, 0);
  analogWrite(IN4, 0);

  delay(2000);

}