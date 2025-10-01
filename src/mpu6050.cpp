#include<mpu6050.h>


// Write a single byte to MPU6050
void MPU6050::writeMPU6050(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// Read two bytes from MPU6050 (big-endian) and combine into signed int16
int MPU6050::readMPU6050Word(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 2);
  uint16_t value = Wire.read() << 8 | Wire.read();
  if (value > 32768)
      return value - 65536;
  return value;
}

void MPU6050::begin(){
  // Wake up MPU6050
  writeMPU6050(PWR_MGMT_1, 0x00); // Clear sleep bit

  // Set sample rate = Gyroscope output rate / (1 + SMPLRT_DIV)
  writeMPU6050(SMPLRT_DIV, 0x07); // 1kHz / (1+7) = 125Hz

  writeMPU6050(CONFIG, 0x00); // No DLPF
  writeMPU6050(GYRO_CONFIG, 0x00); // ±250°/s
  writeMPU6050(ACCEL_CONFIG, 0x00); // ±2g
  writeMPU6050(INT_ENABLE, 0x01); // Enable data ready interrupt
}

int MPU6050::readAccX_raw(){
  return readMPU6050Word(ACCEL_XOUT_H);
}

int MPU6050::readAccY_raw(){
  return readMPU6050Word(ACCEL_XOUT_H + 2);
}

int MPU6050::readAccZ_raw(){
  return readMPU6050Word(ACCEL_XOUT_H + 4);
}


int MPU6050::readGyroX_raw(){
  return readMPU6050Word(GYRO_XOUT_H);
}

int MPU6050::readGyroY_raw(){
  return readMPU6050Word(GYRO_XOUT_H + 2);
}

int MPU6050::readGyroZ_raw(){
  return readMPU6050Word(GYRO_XOUT_H + 4);
}


double MPU6050::readAccX_mps2(){
  int accX_raw = readAccX_raw();
  return ((double)accX_raw / ACCEL_SENS) * G_TO_MS2; // m/s²
}

double MPU6050::readAccY_mps2(){
  int accY_raw = readAccY_raw();
  return ((double)accY_raw / ACCEL_SENS) * G_TO_MS2; // m/s²
}

double MPU6050::readAccZ_mps2(){
  int accZ_raw = readAccZ_raw();
  return ((double)accZ_raw / ACCEL_SENS) * G_TO_MS2; // m/s²
}


double MPU6050::readGyroX_rps(){
  int gyroX_raw = readGyroX_raw();
  return ((double)gyroX_raw / GYRO_SENS) * DEG_TO_RAD; // rad/s
}

double MPU6050::readGyroY_rps(){
  int gyroY_raw = readGyroY_raw();
  return ((double)gyroY_raw / GYRO_SENS) * DEG_TO_RAD; // rad/s
}

double MPU6050::readGyroZ_rps(){
  int gyroZ_raw = readGyroZ_raw();
  return ((double)gyroZ_raw / GYRO_SENS) * DEG_TO_RAD; // rad/s
}