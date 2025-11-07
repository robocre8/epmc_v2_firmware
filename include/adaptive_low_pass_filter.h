#ifndef ADAPTIVE_LOW_PASS_FILTER_H
#define ADAPTIVE_LOW_PASS_FILTER_H
#include <Arduino.h>


class AdaptiveLowPassFilter
{
  private:
    double y_prev;
    double Tf;
    uint64_t lastTime;

  public:  
    AdaptiveLowPassFilter(double);
    void setCutOffFreq(double);
    double filter(double);
    void clear();
};

#endif