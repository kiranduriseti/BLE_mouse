#include <i2c.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include <math.h>


Adafruit_MPU6050 mpu;


int clamp(int v){
  if (v < low_mpu) v = low_mpu;
  if (v > high_mpu) v = high_mpu;

  return v;
}

double deadzone(double v) {
  if (fabs(v) < deadzone_gyro) return 0.0;
  return v;
}

void calibrate_mpu(){
  int N = 200;

  double sumx, sumy, sumz = 0.0;

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

    gyro_bias_x = sumx/N;
    gyro_bias_y = sumy/N;
    gyro_bias_z = sumz/N;

    gx, gy, gz = 0.0;
  }
}

void mpu_setup(){
  Wire.begin(SDA_pin, SCL_pin);
  delay(1000);
  if(mpu.begin() == 0){
    Serial.println("MPU6050 failed to start");
  }
  else{
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 initialized");
  }

  calibrate_mpu();
  Serial.println("MPU calibrated");
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
  fx = deadzone(gyro_x);
  fy = deadzone(gyro_y);

  dx = (int)(fy * sens_gyro_x);
  dy = (int)(-fx * sens_gyro_y);
  dx = clamp(dx);
  dy = clamp(dy);



}