#ifndef MPU6050_H
#define MPU6050_H
#include <Arduino.h>
#include <Wire.h>


class MPU6050
{
  private:
    // MPU6050 Registers
    const int MPU6050_ADDR = 0x68;
    const uint8_t SMPLRT_DIV   = 0x19;
    const uint8_t CONFIG       = 0x1A;
    const uint8_t GYRO_CONFIG  = 0x1B;
    const uint8_t ACCEL_CONFIG = 0x1C;
    const uint8_t INT_ENABLE   = 0x38;
    const uint8_t PWR_MGMT_1   = 0x6B;
    const uint8_t ACCEL_XOUT_H = 0x3B;
    const uint8_t GYRO_XOUT_H  = 0x43;
    // Sensitivity scale factors
    // ±2g range => 16384 LSB/g, g = 9.80665 m/s²
    const double ACCEL_SENS   = 16384.0;
    const double G_TO_MS2     = 9.80665;
    // ±250°/s range => 131 LSB/(°/s), deg/s to rad/s = π/180
    const double GYRO_SENS    = 131.0;

    // Write a single byte to MPU6050
    void writeMPU6050(uint8_t reg, uint8_t data);
    // Read two bytes from MPU6050 (big-endian) and combine into signed int16
    int readMPU6050Word(uint8_t reg);

    int readAccX_raw();
    int readAccY_raw();
    int readAccZ_raw();

    int readGyroX_raw();
    int readGyroY_raw();
    int readGyroZ_raw();

  public:
    void begin();

    double readAccX_mps2();
    double readAccY_mps2();
    double readAccZ_mps2();

    double readGyroX_rps();
    double readGyroY_rps();
    double readGyroZ_rps();
    
};

#endif

