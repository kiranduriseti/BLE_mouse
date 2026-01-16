#ifndef JOYSTICK_H
#define JOYSTICK_H

#define deadzone_joy 120
#define sens_joy 700
#define max_joy 6

#define vx 36
#define vy 39

extern int joy_center_x;
extern int joy_center_y;

extern int joy_x;
extern int joy_y;


void read_joy();
void joy_setup();

#endif