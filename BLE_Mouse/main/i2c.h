#ifndef I2C_H
#define I2C_H

#define deadzone_gyro 2.0
#define sens_gyro_x .18
#define sens_gyro_y .18
#define gyro_alpha .85


void mpu_setup();
void read_mpu();
double gyro_bias_x, gyro_bias_y, gyro_bias_z;
double gx, gy, gz;

double gyro_x;
double gyro_y;

int dx, dy;

#define SCL_pin 36
#define SDA_pin 33

#define low_mpu -30
#define high_mpu 30

#endif