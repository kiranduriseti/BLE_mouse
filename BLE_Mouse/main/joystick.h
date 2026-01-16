#ifndef JOYSTICK_H
#define JOYSTICK_H

#define deadzone_joy 120
#define sens_joy 700
#define max_joy 6

#define vx 3
#define vy 4

int joy_center_x;
int joy_center_y;

int joy_x;
int joy_y;


void read_joy();
void joy_setup();

#endif