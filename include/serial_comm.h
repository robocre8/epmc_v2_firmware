#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "command_functions.h"


void processCommand(uint8_t cmd, uint8_t* data, uint8_t length) {

  gpio_set_level((gpio_num_t)LED_BUILTIN, 1);
  switch (cmd) {
    case WRITE_VEL: {
      float v0, v1, v2, v3;
      memcpy(&v0, &data[0], sizeof(float));
      memcpy(&v1, &data[4], sizeof(float));
      memcpy(&v2, &data[8], sizeof(float));
      memcpy(&v3, &data[12], sizeof(float));
      float res = writeSpeed(v0, v1, v2, v3);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case WRITE_PWM: {
      float pwm0, pwm1, pwm2, pwm3;
      memcpy(&pwm0, &data[0], sizeof(float));
      memcpy(&pwm1, &data[4], sizeof(float));
      memcpy(&pwm2, &data[8], sizeof(float));
      memcpy(&pwm3, &data[12], sizeof(float));
      float res = writePWM((int)pwm0, (int)pwm1, (int)pwm2, (int)pwm3);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case READ_POS: {
      float pos0, pos1, pos2, pos3;
      readPos(pos0, pos1, pos2, pos3);
      Serial.write((uint8_t*)&pos0, sizeof(pos0));
      Serial.write((uint8_t*)&pos1, sizeof(pos1));
      Serial.write((uint8_t*)&pos2, sizeof(pos2));
      Serial.write((uint8_t*)&pos3, sizeof(pos3));
      break;
    }


    case READ_VEL: {
      float v0, v1, v2, v3;
      readFilteredVel(v0, v1, v2, v3);
      Serial.write((uint8_t*)&v0, sizeof(v0));
      Serial.write((uint8_t*)&v1, sizeof(v1));
      Serial.write((uint8_t*)&v2, sizeof(v2));
      Serial.write((uint8_t*)&v3, sizeof(v3));
      break;
    }


    case READ_UVEL: {
      float v0, v1, v2, v3;
      readUnfilteredVel(v0, v1, v2, v3);
      Serial.write((uint8_t*)&v0, sizeof(v0));
      Serial.write((uint8_t*)&v1, sizeof(v1));
      Serial.write((uint8_t*)&v2, sizeof(v2));
      Serial.write((uint8_t*)&v3, sizeof(v3));
      break;
    }


    case READ_TVEL: {
      float v0, v1, v2, v3;
      readTargetVel(v0, v1, v2, v3);
      Serial.write((uint8_t*)&v0, sizeof(v0));
      Serial.write((uint8_t*)&v1, sizeof(v1));
      Serial.write((uint8_t*)&v2, sizeof(v2));
      Serial.write((uint8_t*)&v3, sizeof(v3));
      break;
    }

    case SET_PPR: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setEncoderPPR((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_PPR: {
      uint8_t pos = data[0];
      float res = getEncoderPPR((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_KP: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setMotorKp((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_KP: {
      uint8_t pos = data[0];
      float res = getMotorKp((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_KI: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setMotorKi((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_KI: {
      uint8_t pos = data[0];
      float res = getMotorKi((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    
    case SET_KD: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setMotorKd((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_KD: {
      uint8_t pos = data[0];
      float res = getMotorKd((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_RDIR: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setRdir((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_RDIR: {
      uint8_t pos = data[0];
      float res = getRdir((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_CUT_FREQ: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setCutoffFreq((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_CUT_FREQ: {
      uint8_t pos = data[0];
      float res = getCutoffFreq((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_MAX_VEL: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setMaxVel((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_MAX_VEL: {
      uint8_t pos = data[0];
      float res = getMaxVel((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_PID_MODE: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setPidModeFunc((int)pos, (int)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_PID_MODE: {
      uint8_t pos = data[0];
      float res = getPidModeFunc((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_CMD_TIMEOUT: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setCmdTimeout((int)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_CMD_TIMEOUT: {
      float res = getCmdTimeout();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_I2C_ADDR: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setI2cAddress((int)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_I2C_ADDR: {
      float res = getI2cAddress();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case RESET_PARAMS: {
      float res = triggerResetParams();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_USE_IMU: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setUseIMU((int)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_USE_IMU: {
      float res = getUseIMU();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case READ_ACC: {
      float ax, ay, az;
      readAcc(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      break;
    }


    case READ_ACC_RAW: {
      float ax, ay, az;
      readAccRaw(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      break;
    }


    case READ_ACC_OFF: {
      float ax, ay, az;
      readAccOffset(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      break;
    }


    case READ_ACC_VAR: {
      float ax, ay, az;
      readAccVariance(ax, ay, az);
      Serial.write((uint8_t*)&ax, sizeof(ax));
      Serial.write((uint8_t*)&ay, sizeof(ay));
      Serial.write((uint8_t*)&az, sizeof(az));
      break;
    }


    case WRITE_ACC_OFF: {
      float ax, ay, az;
      memcpy(&ax, &data[0], sizeof(float));
      memcpy(&ay, &data[4], sizeof(float));
      memcpy(&az, &data[8], sizeof(float));
      float res = writeAccOffset(ax, ay, az);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case WRITE_ACC_VAR: {
      float ax, ay, az;
      memcpy(&ax, &data[0], sizeof(float));
      memcpy(&ay, &data[4], sizeof(float));
      memcpy(&az, &data[8], sizeof(float));
      float res = writeAccVariance(ax, ay, az);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case READ_GYRO: {
      float gx, gy, gz;
      readGyro(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      break;
    }


    case READ_GYRO_RAW: {
      float gx, gy, gz;
      readGyroRaw(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      break;
    }


    case READ_GYRO_OFF: {
      float gx, gy, gz;
      readGyroOffset(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      break;
    }


    case READ_GYRO_VAR: {
      float gx, gy, gz;
      readGyroVariance(gx, gy, gz);
      Serial.write((uint8_t*)&gx, sizeof(gx));
      Serial.write((uint8_t*)&gy, sizeof(gy));
      Serial.write((uint8_t*)&gz, sizeof(gz));
      break;
    }


    case WRITE_GYRO_OFF: {
      float gx, gy, gz;
      memcpy(&gx, &data[0], sizeof(float));
      memcpy(&gy, &data[4], sizeof(float));
      memcpy(&gz, &data[8], sizeof(float));
      float res = writeGyroOffset(gx, gy, gz);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case WRITE_GYRO_VAR: {
      float gx, gy, gz;
      memcpy(&gx, &data[0], sizeof(float));
      memcpy(&gy, &data[4], sizeof(float));
      memcpy(&gz, &data[8], sizeof(float));
      float res = writeGyroVariance(gx, gy, gz);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    default: {
      float error = 0.0;
      Serial.write((uint8_t*)&error, sizeof(error));
      break;
    }
  }

  gpio_set_level((gpio_num_t)LED_BUILTIN, 0);
}












void recieve_and_send_data() {
  static uint8_t state = 0;
  static uint8_t cmd, length;
  static uint8_t buffer[32];
  static uint8_t index = 0;
  static uint8_t checksum = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();

    switch (state) {
      case 0: // Wait for start
        if (b == START_BYTE) {
          state = 1;
          checksum = b;
        }
        break;

      case 1: // Command
        cmd = b;
        checksum += b;
        state = 2;
        break;

      case 2: // Length
        length = b;
        checksum += b;
        if (length==0){
          state = 4;
        }
        else{
          index = 0;
          state = 3;
        }
        break;

      case 3: // Payload
        buffer[index++] = b;
        checksum += b;
        if (index >= length) state = 4;
        break;

      case 4: // Checksum
        if ((checksum & 0xFF) == b) {
          processCommand(cmd, buffer, length);
        } else {
          float error = 0.0;
          Serial.write((uint8_t*)&error, sizeof(error));
        }
        state = 0; // reset for next packet
        break;
    }
  }
}

#endif