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
  setZeroPeriodPerTick(0.1);
  oldTickTime = micros();
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
  unsigned long dt = periodPerTick;
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
  unsigned long t = micros();
  
  portENTER_CRITICAL(&encoderMux);
  if ((t - oldTickTime) >= stopPeriodPerTick)
  {
    periodPerTick = 0;
  }
  portEXIT_CRITICAL(&encoderMux);
}

void QuadEncoder::setZeroPeriodPerTick(double zeroVel)
{
  stopPeriodPerTick = (unsigned long)((4.0 * PI * 1e6)/(pulsePerRev*zeroVel));
}