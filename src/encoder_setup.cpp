#include "encoder_setup.h"

QuadEncoder::QuadEncoder(int clk_pin, int dir_pin, double ppr)
{
  clkPin = clk_pin;
  dirPin = dir_pin;
  pulsePerRev = ppr;

  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dirPin, INPUT_PULLUP);

  tickCount = 0;
  prevTickCount = 0;
  dir = 1;
}

void QuadEncoder::setPulsePerRev(double ppr)
{
  pulsePerRev = ppr;
}

double QuadEncoder::getAngPos()
{
  portENTER_CRITICAL(&encoderMux);
  long ticks = tickCount;
  portEXIT_CRITICAL(&encoderMux);
  return (2.00 * PI * (double)ticks) / pulsePerRev;
}

double QuadEncoder::getAngVel()
{
  double dt = (double)(esp_timer_get_time() - lastTime)/1000000.0;

  portENTER_CRITICAL(&encoderMux);
  long ticks = tickCount;
  portEXIT_CRITICAL(&encoderMux);

  long dTicks = ticks - prevTickCount;
  prevTickCount = ticks;
  lastTime = esp_timer_get_time();
  return (2.00 * PI * (double)dTicks) / (pulsePerRev*dt);
}