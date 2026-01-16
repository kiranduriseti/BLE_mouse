#include "joystick.h"
#include <Arduino.h>
#include <math.h>

int joy_center_x;
int joy_center_y;

int joy_x;
int joy_y;

int clamp(int v) {
  if (v > max_joy) return max_joy;
  else if (v < -max_joy) return -max_joy;
  return v;
}
int deadzone(int v) {
  if (fabs(v) < deadzone_joy) return 0;
  return v;
}

void calibrateJoystickCenter() {
  int N = 60;
  long sumx = 0, sumy = 0;
  for (int i = 0; i < N; i++) {
    sumx += analogRead(vx);
    sumy += analogRead(vy);
    delay(5);
  }

  joy_center_x = (int)(sumx/N);
  joy_center_y = (int)(sumy/N);

  Serial.println("Calibrated Joystick Center");
}

void joy_setup() {
  analogReadResolution(12);
  calibrateJoystickCenter();
  Serial.println("Joystick initialized");
}

void read_joy(){
  joy_x = (deadzone(analogRead(vx) - joy_center_x))/341;
  joy_y = (deadzone(analogRead(vy) - joy_center_y))/341;
  joy_x = clamp(joy_x);
  joy_y = clamp(joy_y);

  //Serial.printf("RAW rx=%d ry=%d centerx=%d centery=%d\n", joy_x, joy_y, joy_center_x, joy_center_y);
}