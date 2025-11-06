#ifndef SIMPLE_PID_CONTROL_H
#define SIMPLE_PID_CONTROL_H
#include <Arduino.h>


class SimplePID {
  public:
    SimplePID(double Kp, double Ki, double Kd, double out_min, double out_max);

    void setParameters(double Kp, double Ki, double Kd, double out_min, double out_max);
    void setGains(double Kp, double Ki, double Kd);
    void setKp(double Kp);
    void setKi(double Ki);
    void setKd(double Kd);
    void setOutLimit(double out_max, double out_min);
    void begin();
    double compute(double target, double actual);

  private:
    double error, errorPrev, errorInt, errorDot;
    uint64_t lastTime;
    double kp, ki, kd;
    double outMax, outMin, outSat, outUnsat;
    bool integratorIsOn;

    void reset();
};

#endif
