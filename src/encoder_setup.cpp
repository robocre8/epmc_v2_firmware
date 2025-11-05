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
  setZeroPeriodPerTick(2);
  oldTickTime = esp_timer_get_time();
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
  double ang_vel;

  portENTER_CRITICAL(&encoderMux);
  double direction = (double)dir;
  uint64_t dt = periodPerTick;
  portEXIT_CRITICAL(&encoderMux);

  if (dt == 0) {
    return 0.0;
  }

  double frequency = 1e6 / ((double)dt * pulsePerRev);
  ang_vel = direction * 2.00 * PI * frequency;
  return ang_vel;
}

void QuadEncoder::resetAngVelToZero()
{
  uint64_t t = esp_timer_get_time();
  
  portENTER_CRITICAL(&encoderMux);
  if ((t - oldTickTime) >= stopPeriodPerTick)
  {
    periodPerTick = 0;
  }
  portEXIT_CRITICAL(&encoderMux);
}

void QuadEncoder::setZeroPeriodPerTick(uint64_t stop_timer_ms)
{
  stopPeriodPerTick = 1000*stop_timer_ms;
}