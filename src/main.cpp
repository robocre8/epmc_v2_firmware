#include <Arduino.h>
#include "command_functions.h"
#include "serial_comm.h"
#include "i2c_comm.h"
#include "driver/periph_ctrl.h"

//------------------------------------------------------------------------------//
void IRAM_ATTR readEncoder0()
{
  uint64_t currentTickTime = esp_timer_get_time();

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

  uint64_t period = currentTickTime - encoder[0].oldTickTime;
  encoder[0].periodPerTick = period;
  encoder[0].oldTickTime = currentTickTime;
}

void IRAM_ATTR readEncoder1()
{
  uint64_t currentTickTime = esp_timer_get_time();

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

  uint64_t period = currentTickTime - encoder[1].oldTickTime;
  encoder[1].periodPerTick = period;
  encoder[1].oldTickTime = currentTickTime;
}

void IRAM_ATTR readEncoder2()
{
  uint64_t currentTickTime = esp_timer_get_time();

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

  uint64_t period = currentTickTime - encoder[2].oldTickTime;
  encoder[2].periodPerTick = period;
  encoder[2].oldTickTime = currentTickTime;
}

void IRAM_ATTR readEncoder3()
{
  uint64_t currentTickTime = esp_timer_get_time();

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

  uint64_t period = currentTickTime - encoder[3].oldTickTime;
  encoder[3].periodPerTick = period;
  encoder[3].oldTickTime = currentTickTime;
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
uint64_t serialLoopTime, serialLoopTimeInterval = 200;
uint64_t pidTime, pidTimeInterval = 1000;
uint64_t pidStopTime[num_of_motors], pidStopTimeInterval = 1000000;
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
  serialLoopTime = now_us;
  pidTime = now_us;
  for (int i = 0; i < num_of_motors; i += 1)
  {
    pidStopTime[i] = now_us;
    cmdVelTimeout[i] = now_us;
    isMotorCommanded[i] = 0;
  }
}

void loop()
{
  // Serial comm loop
  recieve_and_send_data();
  uint64_t current_time = esp_timer_get_time();
  if ((current_time - serialLoopTime) >= serialLoopTimeInterval)
  {
    // recieve_and_send_data();
    // Sensor update
    for (int i = 0; i < num_of_motors; i += 1)
    {
      encoder[i].resetAngVelToZero();
      unfilteredVel[i] = encoder[i].getAngVel();
      filteredVel[i] = velFilter[i].filter(unfilteredVel[i]);

      // if (pidMode[i])
      // {
      //   output[i] = pidMotor[i].compute(target[i], filteredVel[i]);
      //   motor[i].sendPWM((int)output[i]);
      // }
    }
    serialLoopTime = current_time;
  }

  

  // PID control loop
  if ((current_time - pidTime) >= pidTimeInterval)
  {
    for (int i = 0; i < num_of_motors; i += 1)
    {
      if (pidMode[i])
      {
        output[i] = pidMotor[i].compute(target[i], filteredVel[i]);
        motor[i].sendPWM((int)output[i]);
      }
    }
    pidTime = current_time;
  }

  // check to see if motor has stopped
  for (int i = 0; i < num_of_motors; i += 1)
  {
    int target_int = (int)fabs(target[i]) * 1000;
    if (target_int < 10 && pidMode[i])
    {
      if ((current_time - pidStopTime[i]) >= pidStopTimeInterval)
      {
        pidMotor[i].begin();
        isMotorCommanded[i] = 0;
        pidMode[i] = 0;
        motor[i].sendPWM(0);
        pidStopTime[i] = current_time;
      }
    }
    else
    {
      pidStopTime[i] = current_time;
    }
  }

  // command timeout
  int cmdTimeout = (int)cmdVelTimeoutInterval;
  if (cmdVelTimeoutInterval > 0)
  {
    for (int i = 0; i < num_of_motors; i += 1)
    {
      if (!isMotorCommanded[i])
      {
        cmdVelTimeout[i] = current_time;
      }
      if (isMotorCommanded[i] && ((current_time - cmdVelTimeout[i]) >= cmdVelTimeoutInterval))
      {
        if(pidMode[i]){
          target[i] = 0.000;
        }
        else {
          motor[i].sendPWM(0);
        }
        isMotorCommanded[i] = 0;
      }
    }
  }
}