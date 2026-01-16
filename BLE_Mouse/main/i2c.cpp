#include "i2c.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

double gyro_bias_x, gyro_bias_y, gyro_bias_z;
double gx, gy, gz;

double gyro_x;
double gyro_y;

int dx, dy;

Adafruit_MPU6050 mpu;


int clamp_mpu(int v){
  if (v < low_mpu) v = low_mpu;
  if (v > high_mpu) v = high_mpu;

  return v;
}

double deadzone_mpu(double v) {
  if (fabs(v) < deadzone_gyro) return 0.0;
  return v;
}

void calibrate_mpu(){
  int N = 200;

  double sumx = 0.0, sumy = 0.0, sumz = 0.0;

  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, temp;

    mpu.getEvent(&a, &g, &temp);
    gx = g.gyro.x * 57.2957795f;
    gy = g.gyro.y * 57.2957795f;  
    gz = g.gyro.z * 57.2957795f;

    sumx += gx;
    sumy += gy;
    sumz += gz;

    delay(5);
  }

  gyro_bias_x = sumx/N;
  gyro_bias_y = sumy/N;
  gyro_bias_z = sumz/N;

  gx = gy = gz = 0.0;

  gyro_x = 0.0;
  gyro_y = 0.0;
  dx = dy = 0;

}

void mpu_setup(){
  Wire.begin(SDA_pin, SCL_pin);
  delay(1000);
  Wire.setClock(100000);

  Serial.println("I2C scan starting...");
  for (int addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    int err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("Found device at 0x%02X\n", addr);
    }
  }
  
  Serial.println("I2C scan done.");
  if(!mpu.begin(0x68)){
    Serial.println("MPU6050 failed to start");
  }
  else{
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 initialized");
    calibrate_mpu();
    Serial.println("MPU calibrated");
  }
}

void read_mpu(){

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gx = g.gyro.x * 57.2957795f - gyro_bias_x;
  gy = g.gyro.y * 57.2957795f - gyro_bias_y;  
  gz = g.gyro.z * 57.2957795f - gyro_bias_z;

  gyro_x = gyro_alpha*gyro_x + (1.0 - gyro_alpha)*gx;
  gyro_y = gyro_alpha*gyro_y + (1.0 - gyro_alpha)*gy;

  double fx, fy;
  fx = deadzone_mpu(gyro_x);
  fy = deadzone_mpu(gyro_y);

  dx = (int)(fy * sens_gyro_x);
  dy = (int)(-fx * sens_gyro_y);
  dx = clamp_mpu(dx);
  dy = clamp_mpu(dy);

  // Serial.print("Gyro x ");
  // Serial.print(dx);
  // Serial.print(" Gyro y ");
  // Serial.println(dy);

  // Serial.println();
}