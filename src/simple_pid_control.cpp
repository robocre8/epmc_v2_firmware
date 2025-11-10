#include "simple_pid_control.h"

SimplePID::SimplePID(double Kp, double Ki, double Kd, double out_min, double out_max)
{
  reset();

  kp = Kp;
  ki = Ki;
  kd = Kd;
  outMax = out_max;
  outMin = out_min;

  errorPrev = error;
}

void SimplePID::setParameters(double Kp, double Ki, double Kd, double out_min, double out_max)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
  outMax = out_max;
  outMin = out_min;
}

void SimplePID::setGains(double Kp, double Ki, double Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void SimplePID::setKp(double Kp)
{
  kp = Kp;
}

void SimplePID::setKi(double Ki)
{
  ki = Ki;
}

void SimplePID::setKd(double Kd)
{
  kd = Kd;
}

void SimplePID::setOutLimit(double out_max, double out_min)
{
  outMax = out_max;
  outMin = out_min;
}

void SimplePID::begin()
{
  reset();
}

double SimplePID::compute(double target, double actual)
{
  double dt = (double)(esp_timer_get_time() - lastTime)/1000000.0;

  error = target - actual;

  if (integratorIsOn)
  {
    errorInt += (error * dt);
  }
  else
  {
    errorInt += 0.0;
  }

  errorDot = (error - errorPrev)/dt;

  outUnsat = (kp * error) + (ki * errorInt) + (kd * errorDot);

  if (outUnsat > outMax)
  {
    outSat = outMax;
    integratorIsOn = false;
  }
  else if (outUnsat < outMin)
  {
    outSat = outMin;
    integratorIsOn = false;
  }
  else
  {
    outSat = outUnsat;
    integratorIsOn = true;
  }

  errorPrev = error;
  lastTime = esp_timer_get_time();

  return outSat;
}

void SimplePID::reset()
{
  error = 0.0;
  errorPrev = 0.0;
  errorInt = 0.0;
  errorDot = 0.0;
  outSat = 0.0;
  outUnsat = 0.0;
  integratorIsOn = false;
  lastTime = esp_timer_get_time();
}