#ifndef I2C_H
#define I2C_H

void mpu_setup();
void read_mpu();
void mpu_sleep();
void mpu_wake();

#define deadzone_gyro 2.0
#define sens_gyro_x .3
#define sens_gyro_y .2
#define sens_gyro_z .1
#define gyro_alpha .85

extern double gyro_bias_x, gyro_bias_y, gyro_bias_z;
extern double gx, gy, gz;

extern double gyro_x;
extern double gyro_y;
extern double gyro_z;

extern int dx, dy, dz;

#define SCL_pin 22
#define SDA_pin 21

#define low_mpu -30
#define high_mpu 30

#endif