#include "adaptive_low_pass_filter.h"

double map_(double x, double in_min, double in_max, double out_min, double out_max) {
    const double run = in_max - in_min;
    // if(run == 0){
    //     log_e("map(): Invalid input range, min == max");
    //     return -1; // AVR returns -1, SAM returns 0
    // }
    const double rise = out_max - out_min;
    const double delta = x - in_min;
    return (delta * rise) / run + out_min;
}

void AdaptiveLowPassFilter::clear()
{
  y_prev = 0.0;
  lastTime = esp_timer_get_time();
}

AdaptiveLowPassFilter::AdaptiveLowPassFilter(double f0)
{
  Tf = map_(f0, 0.1, 1.0, 98, 99.5);
  y_prev = 0.0;
  lastTime = esp_timer_get_time();
}

void AdaptiveLowPassFilter::setCutOffFreq(double f0)
{
  Tf = map_(f0, 0.1, 1.0, 98, 99.5);
  y_prev = 0.0;
  lastTime = esp_timer_get_time();
}

double AdaptiveLowPassFilter::filter(double xn)
{
  // double dt = (double)(esp_timer_get_time() - lastTime)/1000000.0;

  // calculate the filtering 
  double alpha = Tf/100.0;
  double y = alpha*y_prev + (1.0f - alpha)*xn;

  // save the variables
  y_prev = y;
  // lastTime = esp_timer_get_time();
  return y;
}