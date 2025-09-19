#ifndef COMMAND_FUNCTIONS_H
#define COMMAND_FUNCTIONS_H

#include <Arduino.h>
#include <Preferences.h>
#include "motor_control.h"
#include "encoder_setup.h"
#include "adaptive_low_pass_filter.h"
#include "simple_pid_control.h"
#include "mpu6050.h"

int LED_PIN = 10;

//------------ Communication Command IDs --------------//
const uint8_t START_BYTE = 0xAA;
const uint8_t WRITE_VEL = 0x01;
const uint8_t WRITE_PWM = 0x02;
const uint8_t READ_POS = 0x03;
const uint8_t READ_VEL = 0x04;
const uint8_t READ_UVEL = 0x05;
const uint8_t READ_TVEL = 0x06;
const uint8_t SET_PPR = 0x07;
const uint8_t GET_PPR = 0x08;
const uint8_t SET_KP = 0x09;
const uint8_t GET_KP = 0x0A;
const uint8_t SET_KI = 0x0B;
const uint8_t GET_KI = 0x0C;
const uint8_t SET_KD = 0x0D;
const uint8_t GET_KD = 0x0E;
const uint8_t SET_RDIR = 0x0F;
const uint8_t GET_RDIR = 0x10;
const uint8_t SET_CUT_FREQ = 0x11;
const uint8_t GET_CUT_FREQ = 0x12;
const uint8_t SET_MAX_VEL = 0x13;
const uint8_t GET_MAX_VEL = 0x14;
const uint8_t SET_PID_MODE = 0x15;
const uint8_t GET_PID_MODE = 0x16;
const uint8_t SET_CMD_TIMEOUT = 0x17;
const uint8_t GET_CMD_TIMEOUT = 0x18;
const uint8_t SET_I2C_ADDR = 0x19;
const uint8_t GET_I2C_ADDR = 0x1A;
const uint8_t RESET_PARAMS = 0x1B;
const uint8_t SET_USE_IMU = 0x1C;
const uint8_t GET_USE_IMU = 0x1D;
const uint8_t READ_ACC = 0x1E;
const uint8_t READ_ACC_RAW = 0x1F;
const uint8_t READ_ACC_OFF = 0x20;
const uint8_t READ_ACC_VAR = 0x21;
const uint8_t WRITE_ACC_OFF = 0x22;
const uint8_t WRITE_ACC_VAR = 0x23;
const uint8_t READ_GYRO = 0x24;
const uint8_t READ_GYRO_RAW = 0x25;
const uint8_t READ_GYRO_OFF = 0x26;
const uint8_t READ_GYRO_VAR = 0x27;
const uint8_t WRITE_GYRO_OFF = 0x28;
const uint8_t WRITE_GYRO_VAR = 0x29;
const uint8_t READ_MOTOR_DATA = 0x2A;
const uint8_t READ_IMU_DATA = 0x2B;
//---------------------------------------------------//

//--------------- global variables -----------------//
const int num_of_motors = 2;

// motor 0 H-Bridge Connection
int IN1_0 = 6, IN2_0 = 7;
// motor 1 H-Bridge Connection
int IN1_1 = 0, IN2_1 = 5;

MotorControl motor[num_of_motors] = {
  MotorControl(IN1_0, IN2_0), // motor 0
  MotorControl(IN1_1, IN2_1), // motor 1
};

double enc_ppr[num_of_motors]={
  1000.0, // motor 0 encoder pulse per revolution parameter
  1000.0, // motor 1 encoder pulse per revolution parameter
};

// motor 0 encoder connection
int enc0_clkPin = 1, enc0_dirPin = 2;
// motor 1 encoder connection
int enc1_clkPin = 3, enc1_dirPin = 4;

QuadEncoder encoder[num_of_motors] = {
  QuadEncoder(enc0_clkPin, enc0_dirPin, enc_ppr[0]), // motor 0 encoder connection
  QuadEncoder(enc1_clkPin, enc1_dirPin, enc_ppr[1]), // motor 1 encoder connection
};

// adaptive lowpass Filter
const int filterOrder = 1;
double cutOffFreq[num_of_motors] = {
  1.5, // motor 0 velocity filter cutoff frequency
  1.5 // motor 1 velocity filter cutoff frequency
};

AdaptiveLowPassFilter velFilter[num_of_motors] = {
  AdaptiveLowPassFilter(filterOrder, cutOffFreq[0]), // motor 0 velocity filter
  AdaptiveLowPassFilter(filterOrder, cutOffFreq[1]) // motor 1 velocity filter
};

double filteredVel[num_of_motors] = {
  0.0,
  0.0
};

double unfilteredVel[num_of_motors] = {
  0.0,
  0.0
};

// motor PID parameters
double outMin = -255.0, outMax = 255.0;

double kp[num_of_motors] = {
  0.0,
  0.0
};

double ki[num_of_motors] = {
  0.0,
  0.0
};

double kd[num_of_motors] = {
  0.0,
  0.0
};

double target[num_of_motors] = {
  0.0,
  0.0
};

double output[num_of_motors] = {
  0.0,
  0.0
};

SimplePID pidMotor[num_of_motors] = {
  SimplePID(kp[0], ki[0], kd[0], outMin, outMax),
  SimplePID(kp[1], ki[1], kd[1], outMin, outMax)
};


// check if in PID or PWM mode
int pidMode[num_of_motors] = {
  0,
  0
}; // 1-PID MODE, 0-SETUP/PWM MODE

int isMotorCommanded[num_of_motors] = {
  0,
  0
};

int rdir[num_of_motors] = {
  1,
  1
};

// // maximum motor velocity that can be commanded
double maxVel[num_of_motors] = {
  10.0,
  10.0
};

// for command timeout.
unsigned long cmdVelTimeoutInterval = 0; // us -> (1000000/sampleTime) hz
unsigned long cmdVelTimeout[num_of_motors];

// initial i2cAddress
uint8_t i2cAddress = 0x55;

// for stored initialization and reset
bool firstLoad = false;
//-------------------------------------------------//


//-------------- IMU MPU6050 ---------------------//
float accOff[3] = {
  0.0,
  0.0,
  0.0
};

float accVar[3] = {
  0.0,
  0.0,
  0.0
};

float accRaw[3] = {
  0.0,
  0.0,
  0.0
};

float accCal[3] = {
  0.0,
  0.0,
  0.0
};

float gyroOff[3] = {
  0.0,
  0.0,
  0.0
};

float gyroVar[3] = {
  0.0,
  0.0,
  0.0
};

float gyroRaw[3] = {
  0.0,
  0.0,
  0.0
};

float gyroCal[3] = {
  0.0,
  0.0,
  0.0
};

int useIMU = 0;

MPU6050 imu;
//------------------------------------------------//



//--------------- storage variables -----------------//
Preferences storage;

const char * ppr_key[num_of_motors] = {
  "ppr0",
  "ppr1"
};

const char * cf_key[num_of_motors] = {
  "cf0",
  "cf1"
};

const char * kp_key[num_of_motors] = {
  "kp0",
  "kp1"
};

const char * ki_key[num_of_motors] = {
  "ki0",
  "ki1"
};

const char * kd_key[num_of_motors] = {
  "kd0",
  "kd1"
};

const char * rdir_key[num_of_motors] = {
  "rdir0",
  "rdir1"
};

const char * maxVel_key[num_of_motors] = {
  "maxVel0",
  "maxVel1"
};

const char * accOff_key[3] = {
  "accOff0",
  "accOff1",
  "accOff2",
};

const char * accVar_key[3] = {
  "accVar0",
  "accVar1",
  "accVar2",
};

const char * gyroOff_key[3] = {
  "gyroOff0",
  "gyroOff1",
  "gyroOff2",
};

const char * gyroVar_key[3] = {
  "gyroVar0",
  "gyroVar1",
  "gyroVar2",
};

const char * useIMU_key = "useIMU";

const char * i2cAddress_key = "i2cAddress";

const char * firstLoad_key = "firstLoad";

const char * params_ns = "params"; // preference namespace

void resetParamsInStorage(){
  storage.begin(params_ns, false);

  for (int i=0; i<num_of_motors; i+=1){
    storage.putDouble(ppr_key[i], 1000.0);
    storage.putDouble(kp_key[i], 0.0);
    storage.putDouble(ki_key[i], 0.0);
    storage.putDouble(kd_key[i], 0.0);
    storage.putDouble(cf_key[i], 1.5);
    storage.putInt(rdir_key[i], 1);
    storage.putDouble(maxVel_key[i], 10.0);
  }
  for (int i=0; i<3; i+=1){
    storage.putFloat(accOff_key[i], 0.0);
    storage.putFloat(accVar_key[i], 0.0);
    storage.putFloat(gyroOff_key[i], 0.0);
    storage.putFloat(gyroVar_key[i], 0.0);
  }
  storage.putUChar(i2cAddress_key, 0x55);
  storage.putInt(useIMU_key, 0);

  storage.end();
}

void initParams(){
  //check for firstLoad
  storage.begin(params_ns, true);
  firstLoad = storage.getBool(firstLoad_key);
  storage.end();
  // if firsLoad -> reset all params and set firstLoad to false
  if(firstLoad == true){
    resetParamsInStorage();
    firstLoad = false;
    storage.begin(params_ns, false);
    storage.putBool(firstLoad_key, firstLoad);
    storage.end();
  }

}

void loadStoredParams(){
  initParams();
  // load each parameter form the storage to the local variables
  storage.begin(params_ns, true);

  for (int i=0; i<num_of_motors; i+=1){
    enc_ppr[i] = storage.getDouble(ppr_key[i], 1000.0);
    kp[i] = storage.getDouble(kp_key[i], 0.0);
    ki[i] = storage.getDouble(ki_key[i], 0.0);
    kd[i] = storage.getDouble(kd_key[i], 0.0);
    cutOffFreq[i] = storage.getDouble(cf_key[i], 1.5);
    rdir[i] = storage.getInt(rdir_key[i], 1);
    maxVel[i] = storage.getDouble(maxVel_key[i], 10.0);
  }
  for (int i=0; i<3; i+=1){
    accOff[i] = storage.getFloat(accOff_key[i], 0.0);
    accVar[i] = storage.getFloat(accVar_key[i], 0.0);
    gyroOff[i] = storage.getFloat(gyroOff_key[i], 0.0);
    gyroVar[i] = storage.getFloat(gyroVar_key[i], 0.0);
  }
  i2cAddress = storage.getUChar(i2cAddress_key, 0x55);
  useIMU = storage.getInt(useIMU_key, 0);

  storage.end();
}


//-------------------------------------------------//




//--------------- global functions ----------------//
float writeSpeed(float v0, float v1)
{
  float targetVel[num_of_motors] = {v0, v1};
  for (int i = 0; i < num_of_motors; i += 1)
  {
    pidMode[i] = 1;

    double vel;
    if (targetVel[i] > maxVel[i]){
      vel = maxVel[i];
    }
    else if (targetVel[i] < (-1.00 * maxVel[i])){
      vel = -1.00 * maxVel[i];
    }
    else {
      vel = targetVel[i];
    }
      
    target[i] = (double)rdir[i] * vel;
    isMotorCommanded[i] = 1;

    cmdVelTimeout[i] = millis();
  }

  return 1.0;
}

float writePWM(int pwm0, int pwm1)
{
  int pwm[num_of_motors] = {pwm0, pwm1};
  for (int i = 0; i < num_of_motors; i += 1){
    pidMode[i] = 0;

    int p;
    if (pwm[i]>255)
      p = 255;
    else if (pwm[i]<-255)
      p = -255;
    else
      p = pwm[i];
    isMotorCommanded[i] = 1;
    if (p == 0) isMotorCommanded[i] = 0;
    motor[i].sendPWM(rdir[i] * p);

    cmdVelTimeout[i] = millis();
  }
  return 1.0;
}

void readPos(float &pos0, float &pos1)
{  
  double posData[num_of_motors];
  for (int i = 0; i < num_of_motors; i += 1){
    posData[i] = rdir[i] * encoder[i].getAngPos();
  }
  pos0 = (float)posData[0];
  pos1 = (float)posData[1];
}

void readFilteredVel(float &v0, float &v1)
{
  double velData[num_of_motors];
  for (int i = 0; i < num_of_motors; i += 1){
    velData[i] = (double)rdir[i] * filteredVel[i];
  }
  v0 = (float)velData[0];
  v1 = (float)velData[1];
}

void readUnfilteredVel(float &v0, float &v1)
{
  double velData[num_of_motors];
  for (int i = 0; i < num_of_motors; i += 1){
    velData[i] = (double)rdir[i] * unfilteredVel[i];
  }
  v0 = (float)velData[0];
  v1 = (float)velData[1];
}

void readTargetVel(float &v0, float &v1)
{
  double velData[num_of_motors];
  for (int i = 0; i < num_of_motors; i += 1){
    velData[i] = (double)rdir[i] * target[i];
  }
  v0 = (float)velData[0];
  v1 = (float)velData[1];
}

float setEncoderPPR(int motor_no, double ppr)
{
  enc_ppr[motor_no] = ppr;
  storage.begin(params_ns, false);
  storage.putDouble(ppr_key[motor_no], enc_ppr[motor_no]);
  storage.end();
  encoder[motor_no].setPulsePerRev(enc_ppr[motor_no]);
  return 1.0;
}
float getEncoderPPR(int motor_no)
{
  return (float)enc_ppr[motor_no];
}


float setMotorKp(int motor_no, double Kp)
{
  kp[motor_no] = Kp;
  storage.begin(params_ns, false);
  storage.putDouble(kp_key[motor_no], kp[motor_no]);
  storage.end();
  pidMotor[motor_no].setKp(kp[motor_no]);
  pidMotor[motor_no].begin();
  return 1.0;
}
float getMotorKp(int motor_no)
{
  return (float)kp[motor_no];
}


float setMotorKi(int motor_no, double Ki)
{
  ki[motor_no] = Ki;
  storage.begin(params_ns, false);
  storage.putDouble(ki_key[motor_no], ki[motor_no]);
  storage.end();
  pidMotor[motor_no].setKi(ki[motor_no]);
  pidMotor[motor_no].begin();
  return 1.0;
}
float getMotorKi(int motor_no)
{
  return (float)ki[motor_no];
}


float setMotorKd(int motor_no, double Kd)
{
  kd[motor_no] = Kd;
  storage.begin(params_ns, false);
  storage.putDouble(kd_key[motor_no], kd[motor_no]);
  storage.end();
  pidMotor[motor_no].setKd(kd[motor_no]);
  pidMotor[motor_no].begin();
  return 1.0;
}
float getMotorKd(int motor_no)
{
  return (float)kd[motor_no];
}


float setRdir(int motor_no, double dir)
{
  if (dir >= 0)
    rdir[motor_no] = 1;
  else
    rdir[motor_no] = -1;
  storage.begin(params_ns, false);
  storage.putInt(rdir_key[motor_no], rdir[motor_no]);
  storage.end();
  return 1.0;
}
float getRdir(int motor_no)
{
  return (float)rdir[motor_no];
}



float setCutoffFreq(int motor_no, double f0)
{
  cutOffFreq[motor_no] = f0;
  storage.begin(params_ns, false);
  storage.putDouble(cf_key[motor_no], cutOffFreq[motor_no]);
  storage.end();
  velFilter[motor_no].setCutOffFreq(cutOffFreq[motor_no]);
  return 1.0;
}
float getCutoffFreq(int motor_no)
{
  return (float)cutOffFreq[motor_no];
}



float setMaxVel(int motor_no, double max_vel)
{
  maxVel[motor_no] = fabs(max_vel);
  storage.begin(params_ns, false);
  storage.putDouble(maxVel_key[motor_no], maxVel[motor_no]);
  storage.end();
  return 1.0;
}
float getMaxVel(int motor_no)
{
  return (float)maxVel[motor_no];
}



float setPidModeFunc(int motor_no, int mode)
{
  pidMode[motor_no] = mode;
  motor[motor_no].sendPWM(0);
  pidMotor[motor_no].begin();

  return 1.0;
}
float getPidModeFunc(int motor_no)
{
  return (float)pidMode[motor_no];
}




float setCmdTimeout(int timeout_ms)
{
  unsigned long cmdTimeout = timeout_ms;
  if (cmdTimeout < 10)
  {
    cmdVelTimeoutInterval = 0;
  }
  else
  {
    cmdVelTimeoutInterval = cmdTimeout;
    for (int i = 0; i < num_of_motors; i += 1)
    {
      cmdVelTimeout[i] = millis();
    }
  }
  return 1.0;
}
float getCmdTimeout()
{
  return (float)cmdVelTimeoutInterval;
}




#include "i2c_comm.h"
float setI2cAddress(int address)
{
  if(useIMU==0){
    if((address <= 0) || (address > 255)){
      return 0.0;
    }
    else {
      i2cAddress = (uint8_t)address;
      storage.begin(params_ns, false);
      storage.putUChar(i2cAddress_key, i2cAddress);
      storage.end();

      Wire.begin(i2cAddress);

      return 1.0;
    }
  }
  else {
    return 0.0;
  }
  
}
float getI2cAddress()
{
  return (float)i2cAddress;
}



float triggerResetParams()
{
  storage.begin(params_ns, false);
  firstLoad = true;
  storage.putBool(firstLoad_key, firstLoad);
  storage.end();
  // reload to reset
  loadStoredParams();
  return 1.0;
}
//-----------------------------------------------------------------//



//------------------------------------------------------------------//
float setUseIMU(int val)
{
  if((val < 0) || (val > 1)){
    return 0.0;
  }
  storage.begin(params_ns, false);
  storage.putInt(useIMU_key, val);
  storage.end();

  return 1.0;
}
float getUseIMU()
{
  return (float)useIMU;
}


void readAcc(float &ax, float &ay, float &az)
{  
  ax = accCal[0];
  ay = accCal[1];
  az = accCal[2];
}


void readAccRaw(float &ax, float &ay, float &az)
{  
  ax = accRaw[0];
  ay = accRaw[1];
  az = accRaw[2];
}


void readAccOffset(float &ax, float &ay, float &az)
{  
  ax = accOff[0];
  ay = accOff[1];
  az = accOff[2];
}
float writeAccOffset(float ax, float ay, float az) {
  float val[3] = {ax, ay, az};
  for (int i = 0; i < 3; i += 1)
  {
    accOff[i] = val[i];
    storage.begin(params_ns, false);
    storage.putFloat(accOff_key[i], accOff[i]);
    storage.end();
  }
  return 1.0;
}


void readAccVariance(float &ax, float &ay, float &az)
{  
  ax = accVar[0];
  ay = accVar[1];
  az = accVar[2];
}
float writeAccVariance(float ax, float ay, float az) {
  float val[3] = {ax, ay, az};
  for (int i = 0; i < 3; i += 1)
  {
    accVar[i] = val[i];
    storage.begin(params_ns, false);
    storage.putFloat(accVar_key[i], accVar[i]);
    storage.end();
  }
  return 1.0;
}



void readGyro(float &gx, float &gy, float &gz)
{  
  gx = gyroCal[0];
  gy = gyroCal[1];
  gz = gyroCal[2];
}


void readGyroRaw(float &gx, float &gy, float &gz)
{  
  gx = gyroRaw[0];
  gy = gyroRaw[1];
  gz = gyroRaw[2];
}


void readGyroOffset(float &gx, float &gy, float &gz)
{  
  gx = gyroOff[0];
  gy = gyroOff[1];
  gz = gyroOff[2];
}
float writeGyroOffset(float gx, float gy, float gz) {
  float val[3] = {gx, gy, gz};
  for (int i = 0; i < 3; i += 1)
  {
    gyroOff[i] = val[i];
    storage.begin(params_ns, false);
    storage.putFloat(gyroOff_key[i], gyroOff[i]);
    storage.end();
  }
  return 1.0;
}


void readGyroVariance(float &gx, float &gy, float &gz)
{  
  gx = gyroVar[0];
  gy = gyroVar[1];
  gz = gyroVar[2];
}
float writeGyroVariance(float gx, float gy, float gz) {
  float val[3] = {gx, gy, gz};
  for (int i = 0; i < 3; i += 1)
  {
    gyroVar[i] = val[i];
    storage.begin(params_ns, false);
    storage.putFloat(gyroVar_key[i], gyroVar[i]);
    storage.end();
  }
  return 1.0;
}

//-------------------------------------------------------------------//


#endif