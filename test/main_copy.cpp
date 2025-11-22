#include <Arduino.h>

// // motor 0 H-Bridge Connection
// int IN1_0 = 5, IN2_0 = 17, EN_0 = 16;
// // motor 1 H-Bridge Connection
// int IN1_1 = 19, IN2_1 = 18, EN_1 = 23;
// // motor 2 H-Bridge Connection
// int IN1_2 = 26, IN2_2 = 27, EN_2 = 12;
// // motor 3 H-Bridge Connection
// int IN1_3 = 33, IN2_3 = 25, EN_3 = 32;

// MOTOR 1
#define IN1 5
#define IN2 17

// MOTOT 2
#define IN3 19
#define IN4 18


void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void loop() {
  delay(2000);

  analogWrite(IN1, 100);
  analogWrite(IN2, 0);

  // analogWrite(IN3, 100);
  // analogWrite(IN4, 0);

  delay(4000);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  analogWrite(IN3, 0);
  analogWrite(IN4, 0);

  delay(2000);

  analogWrite(IN1, 0);
  analogWrite(IN2, 150);

  // analogWrite(IN3, 0);
  // analogWrite(IN4, 150);

  delay(4000);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  analogWrite(IN3, 0);
  analogWrite(IN4, 0);

}