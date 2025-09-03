#ifndef ENCODER_SETUP_H
#define ENCODER_SETUP_H
#include <Arduino.h>
#include "driver/periph_ctrl.h"

// For critical sections on ESP32
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;


class QuadEncoder {
public:
  volatile long tickCount;
  volatile int8_t dir;
  int clkPin, dirPin;
  double pulsePerRev;
  unsigned long periodPerTick;
  unsigned long stopPeriodPerTick;
  volatile unsigned long oldTickTime;
  volatile double freqPerTick;

  QuadEncoder(int clk_pin, int dir_pin, double ppr);

  void setPulsePerRev(double ppr);
  void setZeroPeriodPerTick(double zeroVel);
  double getAngPos();
  double getAngVel();
  void resetAngVelToZero();

private:
  double prevTickCount;
  
};

#endif