#include <Arduino.h>
#include "command_functions.h"
#include "serial_comm.h"
#include "i2c_comm.h"
#include "driver/periph_ctrl.h"

//------------------------------------------------------------------------------//
void IRAM_ATTR readEncoder0()
{
  if (digitalRead(encoder[0].clkPin) == digitalRead(encoder[0].dirPin))
  {
    encoder[0].tickCount -= 1;
    encoder[0].dir = -1;
  }
  else
  {
    encoder[0].tickCount += 1;
    encoder[0].dir = 1;
  }
}

void IRAM_ATTR readEncoder1()
{
  if (digitalRead(encoder[1].clkPin) == digitalRead(encoder[1].dirPin))
  {
    encoder[1].tickCount -= 1;
    encoder[1].dir = -1;
  }
  else
  {
    encoder[1].tickCount += 1;
    encoder[1].dir = 1;
  }
}

void IRAM_ATTR readEncoder2()
{
  if (digitalRead(encoder[2].clkPin) == digitalRead(encoder[2].dirPin))
  {
    encoder[2].tickCount -= 1;
    encoder[2].dir = -1;
  }
  else
  {
    encoder[2].tickCount += 1;
    encoder[2].dir = 1;
  }
}

void IRAM_ATTR readEncoder3()
{
  if (digitalRead(encoder[3].clkPin) == digitalRead(encoder[3].dirPin))
  {
    encoder[3].tickCount -= 1;
    encoder[3].dir = -1;
  }
  else
  {
    encoder[3].tickCount += 1;
    encoder[3].dir = 1;
  }
}
//----------------------------------------------------------------------------------------------//

void encoderInit()
{
  for (int i = 0; i < num_of_motors; i += 1)
  {
    encoder[i].setPulsePerRev(enc_ppr[i]);
  }

  attachInterrupt(digitalPinToInterrupt(encoder[0].clkPin), readEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder[1].clkPin), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder[2].clkPin), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder[3].clkPin), readEncoder3, RISING);
}

void velFilterInit()
{
  for (int i = 0; i < num_of_motors; i += 1)
  {
    velFilter[i].setCutOffFreq(cutOffFreq[i]);
  }
}

void pidInit()
{
  for (int i = 0; i < num_of_motors; i += 1)
  {
    pidMotor[i].setParameters(kp[i], ki[i], kd[i], outMin, outMax);
    pidMotor[i].begin();
  }
}

//---------------------------------------------------------------------------------------------
// Timing variables in microseconds
// please do not adjust any of the values as it can affect important operations
uint64_t sensorReadTime, sensorReadTimeInterval = 1000;
uint64_t pidTime, pidTimeInterval = 5000;
uint64_t pidStopTime, pidStopTimeInterval = 1000000;
//---------------------------------------------------------------------------------------------


void setup()
{
  loadStoredParams();

  Serial.begin(115200);
  // Serial.begin(460800);
  // Serial.begin(921600);
  // Serial.setTimeout(2);

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin(i2cAddress);

  pinMode(LED_BUILTIN, OUTPUT);

  analogWriteResolution(8); // 8 Bit resolution
  analogWriteFrequency(1000); // 1kHz

  encoderInit();
  velFilterInit();
  pidInit();

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize timing markers
  uint64_t now_us = esp_timer_get_time();
  sensorReadTime = now_us;
  pidTime = now_us;
  pidStopTime = now_us;
  cmdVelTimeout = now_us;

}

void loop()
{
  // Serial comm loop
  recieve_and_send_data();
  
  if ((esp_timer_get_time() - sensorReadTime) >= sensorReadTimeInterval)
  {
    for (int i = 0; i < num_of_motors; i += 1)
    {
      unfilteredVel[i] = encoder[i].getAngVel();
      filteredVel[i] = velFilter[i].filter(unfilteredVel[i]);
    }
    sensorReadTime = esp_timer_get_time();
  }

  // PID control loop
  if ((esp_timer_get_time() - pidTime) >= pidTimeInterval)
  {
    if (pidMode)
    {
      for (int i = 0; i < num_of_motors; i += 1)
      {
        output[i] = pidMotor[i].compute(target[i], filteredVel[i]);
        motor[i].sendPWM((int)output[i]);
      }
    }
    pidTime = esp_timer_get_time();
  }

  // check to see if motor has stopped
  if (abs(target[0]) < 0.001 && abs(target[1]) < 0.001 && abs(target[2]) < 0.001 && abs(target[3]) < 0.001)
  {
    if (pidMode == 1)
    {
      if ((esp_timer_get_time() - pidStopTime) >= pidStopTimeInterval)
      {
        target[0] = 0.00;
        target[1] = 0.00;
        target[2] = 0.00;
        target[3] = 0.00;
        setPidModeFunc(0);
        pidStopTime = esp_timer_get_time();
      }
    }
    else
    {
      pidStopTime = esp_timer_get_time();
    }
  }
  else
  {
    if (pidMode == 0)
    {
      setPidModeFunc(1);
    }
    pidStopTime = esp_timer_get_time();
  }

  // command timeout
  if (cmdVelTimeoutInterval > 0)
  {
    if ((esp_timer_get_time() - cmdVelTimeout) >= cmdVelTimeoutInterval)
    {
      target[0] = 0.00;
      target[1] = 0.00;
      target[2] = 0.00;
      target[3] = 0.00;
      setPidModeFunc(0);
    }
  }
}