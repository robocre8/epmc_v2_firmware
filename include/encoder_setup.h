#ifndef ENCODER_SETUP_H
#define ENCODER_SETUP_H
#include <Arduino.h>
#include "driver/periph_ctrl.h"

// For critical sections on ESP32
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;


class QuadEncoder {
public:
  volatile long tickCount;
  int clkPin, dirPin;
  double pulsePerRev;
  uint64_t lastTime;

  QuadEncoder(int clk_pin, int dir_pin, double ppr);

  void setPulsePerRev(double ppr);
  double getAngPos();
  double getAngVel();

private:
  long prevTickCount;
  
};

#endif